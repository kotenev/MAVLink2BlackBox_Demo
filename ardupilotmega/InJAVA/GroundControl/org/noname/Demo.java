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
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            char  flags = pack.flags_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  vibration_x = pack.vibration_x_GET();
            float  vibration_y = pack.vibration_y_GET();
            float  vibration_z = pack.vibration_z_GET();
            long  clipping_0 = pack.clipping_0_GET();
            long  clipping_1 = pack.clipping_1_GET();
            long  clipping_2 = pack.clipping_2_GET();
        });
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            char  message_id = pack.message_id_GET();
            int  interval_us = pack.interval_us_GET();
        });
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            @MAV_VTOL_STATE int  vtol_state = pack.vtol_state_GET();
            @MAV_LANDED_STATE int  landed_state = pack.landed_state_GET();
        });
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            @MAV_COLLISION_SRC int  src_ = pack.src__GET();
            long  id = pack.id_GET();
            @MAV_COLLISION_ACTION int  action = pack.action_GET();
            @MAV_COLLISION_THREAT_LEVEL int  threat_level = pack.threat_level_GET();
            float  time_to_minimum_delta = pack.time_to_minimum_delta_GET();
            float  altitude_minimum_delta = pack.altitude_minimum_delta_GET();
            float  horizontal_minimum_delta = pack.horizontal_minimum_delta_GET();
        });
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  message_type = pack.message_type_GET();
            char[]  payload = pack.payload_GET();
        });
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            char  address = pack.address_GET();
            char  ver = pack.ver_GET();
            char  type = pack.type_GET();
            byte[]  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            String name = pack.name_TRY(ph);
            long  time_usec = pack.time_usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            float  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            int  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            @MAV_SEVERITY int  severity = pack.severity_GET();
            String text = pack.text_TRY(ph);
        });
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  ind = pack.ind_GET();
            float  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  secret_key = pack.secret_key_GET();
            long  initial_timestamp = pack.initial_timestamp_GET();
        });
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  last_change_ms = pack.last_change_ms_GET();
            char  state = pack.state_GET();
        });
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String tune = pack.tune_TRY(ph);
        });
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @CAMERA_MODE int  mode_id = pack.mode_id_GET();
        });
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  image_status = pack.image_status_GET();
            char  video_status = pack.video_status_GET();
            float  image_interval = pack.image_interval_GET();
            long  recording_time_ms = pack.recording_time_ms_GET();
            float  available_capacity = pack.available_capacity_GET();
        });
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
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
        CommunicationChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  arming_time_utc = pack.arming_time_utc_GET();
            long  takeoff_time_utc = pack.takeoff_time_utc_GET();
            long  flight_uuid = pack.flight_uuid_GET();
        });
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(1933775403147573678L);
        p103.x_SET(-3.044794E38F);
        p103.y_SET(-4.0907947E37F);
        p103.z_SET(-2.5009734E38F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(8215072174168265875L);
        p104.x_SET(2.4278213E38F);
        p104.y_SET(2.284898E38F);
        p104.z_SET(-1.1308046E38F);
        p104.roll_SET(2.3939778E38F);
        p104.pitch_SET(-1.5251931E38F);
        p104.yaw_SET(-1.440664E38F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(21577941080800232L);
        p105.xacc_SET(-3.302404E38F);
        p105.yacc_SET(-1.6019427E38F);
        p105.zacc_SET(-8.2279697E37F);
        p105.xgyro_SET(-2.4066872E38F);
        p105.ygyro_SET(2.574073E38F);
        p105.zgyro_SET(-1.4825549E38F);
        p105.xmag_SET(-1.0034425E37F);
        p105.ymag_SET(-2.4800767E38F);
        p105.zmag_SET(1.2670875E37F);
        p105.abs_pressure_SET(2.6079459E38F);
        p105.diff_pressure_SET(-3.2693305E38F);
        p105.pressure_alt_SET(2.7657589E38F);
        p105.temperature_SET(-1.7795817E38F);
        p105.fields_updated_SET((char)45223);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(5365799651886013541L);
        p106.sensor_id_SET((char)40);
        p106.integration_time_us_SET(4266970291L);
        p106.integrated_x_SET(2.691076E38F);
        p106.integrated_y_SET(-1.1547915E38F);
        p106.integrated_xgyro_SET(-2.8179127E37F);
        p106.integrated_ygyro_SET(4.2399067E37F);
        p106.integrated_zgyro_SET(-2.0226212E37F);
        p106.temperature_SET((short)18181);
        p106.quality_SET((char)190);
        p106.time_delta_distance_us_SET(181057161L);
        p106.distance_SET(-2.8839426E38F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(6552438989653105017L);
        p107.xacc_SET(-1.7800526E38F);
        p107.yacc_SET(8.354294E37F);
        p107.zacc_SET(-5.933887E37F);
        p107.xgyro_SET(-1.3555127E38F);
        p107.ygyro_SET(8.406358E37F);
        p107.zgyro_SET(-1.4682792E38F);
        p107.xmag_SET(2.3340417E38F);
        p107.ymag_SET(-1.8513994E38F);
        p107.zmag_SET(1.3798604E37F);
        p107.abs_pressure_SET(-2.7740073E38F);
        p107.diff_pressure_SET(8.4525685E37F);
        p107.pressure_alt_SET(-1.7998793E38F);
        p107.temperature_SET(-6.150274E37F);
        p107.fields_updated_SET(589286341L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(1.4575784E38F);
        p108.q2_SET(3.1405194E38F);
        p108.q3_SET(-3.164024E38F);
        p108.q4_SET(-4.780478E37F);
        p108.roll_SET(-2.0322443E38F);
        p108.pitch_SET(-1.9605786E38F);
        p108.yaw_SET(-2.7941644E38F);
        p108.xacc_SET(-1.9550176E38F);
        p108.yacc_SET(7.9038593E37F);
        p108.zacc_SET(-1.6033895E38F);
        p108.xgyro_SET(2.128211E38F);
        p108.ygyro_SET(-1.9768153E38F);
        p108.zgyro_SET(-1.9010394E38F);
        p108.lat_SET(-7.2336555E37F);
        p108.lon_SET(1.8584439E37F);
        p108.alt_SET(3.3214308E38F);
        p108.std_dev_horz_SET(1.1534796E38F);
        p108.std_dev_vert_SET(-2.2985383E38F);
        p108.vn_SET(-2.6096687E38F);
        p108.ve_SET(2.6000589E38F);
        p108.vd_SET(7.09425E37F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)108);
        p109.remrssi_SET((char)162);
        p109.txbuf_SET((char)155);
        p109.noise_SET((char)125);
        p109.remnoise_SET((char)85);
        p109.rxerrors_SET((char)7515);
        p109.fixed__SET((char)29192);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)121);
        p110.target_system_SET((char)23);
        p110.target_component_SET((char)155);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(4174845287918997954L);
        p111.ts1_SET(833083356159728209L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8507414130621353811L);
        p112.seq_SET(1026297037L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(7448778933656492284L);
        p113.fix_type_SET((char)131);
        p113.lat_SET(-373500899);
        p113.lon_SET(1919885740);
        p113.alt_SET(-1729383137);
        p113.eph_SET((char)13025);
        p113.epv_SET((char)33524);
        p113.vel_SET((char)37405);
        p113.vn_SET((short)8077);
        p113.ve_SET((short)21320);
        p113.vd_SET((short)29511);
        p113.cog_SET((char)36186);
        p113.satellites_visible_SET((char)112);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(4091151009294310176L);
        p114.sensor_id_SET((char)248);
        p114.integration_time_us_SET(2560632185L);
        p114.integrated_x_SET(1.1380076E37F);
        p114.integrated_y_SET(-8.632405E37F);
        p114.integrated_xgyro_SET(2.6935904E38F);
        p114.integrated_ygyro_SET(2.567398E38F);
        p114.integrated_zgyro_SET(-1.4346849E38F);
        p114.temperature_SET((short) -290);
        p114.quality_SET((char)53);
        p114.time_delta_distance_us_SET(1554759025L);
        p114.distance_SET(1.3561884E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(3366136295773569369L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-1.1903257E38F);
        p115.pitchspeed_SET(-1.7714547E38F);
        p115.yawspeed_SET(-4.5252936E37F);
        p115.lat_SET(-1921385984);
        p115.lon_SET(429966794);
        p115.alt_SET(1820474907);
        p115.vx_SET((short)21489);
        p115.vy_SET((short)10404);
        p115.vz_SET((short)17764);
        p115.ind_airspeed_SET((char)47567);
        p115.true_airspeed_SET((char)52104);
        p115.xacc_SET((short)20628);
        p115.yacc_SET((short) -14026);
        p115.zacc_SET((short)22354);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(678535744L);
        p116.xacc_SET((short)17137);
        p116.yacc_SET((short) -28168);
        p116.zacc_SET((short)6947);
        p116.xgyro_SET((short)11472);
        p116.ygyro_SET((short)9196);
        p116.zgyro_SET((short)15306);
        p116.xmag_SET((short)22427);
        p116.ymag_SET((short) -9023);
        p116.zmag_SET((short) -8094);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)248);
        p117.target_component_SET((char)29);
        p117.start_SET((char)51);
        p117.end_SET((char)47607);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)4584);
        p118.num_logs_SET((char)33700);
        p118.last_log_num_SET((char)32652);
        p118.time_utc_SET(717299001L);
        p118.size_SET(3480212324L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)149);
        p119.target_component_SET((char)147);
        p119.id_SET((char)2608);
        p119.ofs_SET(1280760449L);
        p119.count_SET(3722251592L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)7409);
        p120.ofs_SET(3624616326L);
        p120.count_SET((char)129);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)165);
        p121.target_component_SET((char)18);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)73);
        p122.target_component_SET((char)166);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)237);
        p123.target_component_SET((char)98);
        p123.len_SET((char)127);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(8662659810583757670L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
        p124.lat_SET(-92289230);
        p124.lon_SET(-163606135);
        p124.alt_SET(-488243799);
        p124.eph_SET((char)53120);
        p124.epv_SET((char)55579);
        p124.vel_SET((char)17328);
        p124.cog_SET((char)24028);
        p124.satellites_visible_SET((char)234);
        p124.dgps_numch_SET((char)250);
        p124.dgps_age_SET(2454473369L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)46911);
        p125.Vservo_SET((char)60915);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
        p126.timeout_SET((char)39682);
        p126.baudrate_SET(1345059712L);
        p126.count_SET((char)55);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(1605259116L);
        p127.rtk_receiver_id_SET((char)50);
        p127.wn_SET((char)15440);
        p127.tow_SET(2103586487L);
        p127.rtk_health_SET((char)96);
        p127.rtk_rate_SET((char)67);
        p127.nsats_SET((char)178);
        p127.baseline_coords_type_SET((char)15);
        p127.baseline_a_mm_SET(2120443348);
        p127.baseline_b_mm_SET(-434875018);
        p127.baseline_c_mm_SET(1522782792);
        p127.accuracy_SET(3512556483L);
        p127.iar_num_hypotheses_SET(-1157807698);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(634552495L);
        p128.rtk_receiver_id_SET((char)29);
        p128.wn_SET((char)40381);
        p128.tow_SET(4095583970L);
        p128.rtk_health_SET((char)21);
        p128.rtk_rate_SET((char)60);
        p128.nsats_SET((char)77);
        p128.baseline_coords_type_SET((char)2);
        p128.baseline_a_mm_SET(-736758913);
        p128.baseline_b_mm_SET(1985056633);
        p128.baseline_c_mm_SET(-574613915);
        p128.accuracy_SET(3091652933L);
        p128.iar_num_hypotheses_SET(745141832);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(786725057L);
        p129.xacc_SET((short) -18206);
        p129.yacc_SET((short)3064);
        p129.zacc_SET((short) -9201);
        p129.xgyro_SET((short) -4729);
        p129.ygyro_SET((short)28807);
        p129.zgyro_SET((short)16052);
        p129.xmag_SET((short)31840);
        p129.ymag_SET((short)29495);
        p129.zmag_SET((short) -27168);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)205);
        p130.size_SET(2124319896L);
        p130.width_SET((char)21810);
        p130.height_SET((char)15650);
        p130.packets_SET((char)42778);
        p130.payload_SET((char)131);
        p130.jpg_quality_SET((char)153);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)52470);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(24982145L);
        p132.min_distance_SET((char)54491);
        p132.max_distance_SET((char)4095);
        p132.current_distance_SET((char)12893);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p132.id_SET((char)31);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
        p132.covariance_SET((char)43);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(371933671);
        p133.lon_SET(1822302970);
        p133.grid_spacing_SET((char)49861);
        p133.mask_SET(4929801589416894525L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(22217747);
        p134.lon_SET(-703882626);
        p134.grid_spacing_SET((char)40070);
        p134.gridbit_SET((char)76);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1275012827);
        p135.lon_SET(946047036);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(1856143059);
        p136.lon_SET(435405639);
        p136.spacing_SET((char)21125);
        p136.terrain_height_SET(-1.4331835E38F);
        p136.current_height_SET(-3.2275435E38F);
        p136.pending_SET((char)12302);
        p136.loaded_SET((char)57531);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(2638917336L);
        p137.press_abs_SET(1.4086405E37F);
        p137.press_diff_SET(6.6939243E37F);
        p137.temperature_SET((short) -13371);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(5872086659222933914L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(8.029012E37F);
        p138.y_SET(2.0961313E38F);
        p138.z_SET(1.4834881E38F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(7477612090624967069L);
        p139.group_mlx_SET((char)146);
        p139.target_system_SET((char)152);
        p139.target_component_SET((char)163);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(6223698892321561559L);
        p140.group_mlx_SET((char)244);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(1579014462635147612L);
        p141.altitude_monotonic_SET(2.455372E38F);
        p141.altitude_amsl_SET(6.25675E37F);
        p141.altitude_local_SET(-1.918259E38F);
        p141.altitude_relative_SET(-2.998915E38F);
        p141.altitude_terrain_SET(4.1901917E37F);
        p141.bottom_clearance_SET(5.610175E37F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)129);
        p142.uri_type_SET((char)176);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)173);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3418568771L);
        p143.press_abs_SET(3.0550884E38F);
        p143.press_diff_SET(-3.006061E38F);
        p143.temperature_SET((short)18762);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(1735141730676479651L);
        p144.est_capabilities_SET((char)98);
        p144.lat_SET(-566136392);
        p144.lon_SET(-1504673686);
        p144.alt_SET(-3.0254488E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(924392831963166916L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(202391607703829554L);
        p146.x_acc_SET(2.0274027E38F);
        p146.y_acc_SET(-1.90621E38F);
        p146.z_acc_SET(2.6338717E38F);
        p146.x_vel_SET(-6.821374E37F);
        p146.y_vel_SET(-2.5455842E38F);
        p146.z_vel_SET(-2.891269E38F);
        p146.x_pos_SET(-8.3867967E36F);
        p146.y_pos_SET(1.0558267E38F);
        p146.z_pos_SET(1.342028E38F);
        p146.airspeed_SET(-1.0293646E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(1.1585645E38F);
        p146.pitch_rate_SET(-3.2952174E38F);
        p146.yaw_rate_SET(1.9522592E38F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)49);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
        p147.temperature_SET((short) -3014);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -1535);
        p147.current_consumed_SET(780544553);
        p147.energy_consumed_SET(138570313);
        p147.battery_remaining_SET((byte) - 70);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION));
        p148.flight_sw_version_SET(1150358721L);
        p148.middleware_sw_version_SET(585525609L);
        p148.os_sw_version_SET(3029955908L);
        p148.board_version_SET(2543554062L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)3165);
        p148.product_id_SET((char)6819);
        p148.uid_SET(3273067945860979896L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(6888280231230576750L);
        p149.target_num_SET((char)135);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p149.angle_x_SET(1.9952074E38F);
        p149.angle_y_SET(-1.2321036E38F);
        p149.distance_SET(-4.3931674E37F);
        p149.size_x_SET(-3.0125585E38F);
        p149.size_y_SET(3.3406654E38F);
        p149.x_SET(-2.4419125E38F, PH);
        p149.y_SET(-2.192523E38F, PH);
        p149.z_SET(2.967258E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
        p149.position_valid_SET((char)254, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SENSOR_OFFSETS p150 = CommunicationChannel.instance.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.mag_ofs_x_SET((short) -18762);
        p150.mag_ofs_y_SET((short) -12220);
        p150.mag_ofs_z_SET((short) -9751);
        p150.mag_declination_SET(1.798056E38F);
        p150.raw_press_SET(948121784);
        p150.raw_temp_SET(-637978646);
        p150.gyro_cal_x_SET(-6.3595926E37F);
        p150.gyro_cal_y_SET(-3.2279692E38F);
        p150.gyro_cal_z_SET(3.758708E37F);
        p150.accel_cal_x_SET(-7.8104745E36F);
        p150.accel_cal_y_SET(4.4651654E37F);
        p150.accel_cal_z_SET(-3.1255019E38F);
        CommunicationChannel.instance.send(p150); //===============================
        SET_MAG_OFFSETS p151 = CommunicationChannel.instance.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.target_system_SET((char)22);
        p151.target_component_SET((char)246);
        p151.mag_ofs_x_SET((short)22612);
        p151.mag_ofs_y_SET((short) -17390);
        p151.mag_ofs_z_SET((short) -1176);
        CommunicationChannel.instance.send(p151); //===============================
        MEMINFO p152 = CommunicationChannel.instance.new_MEMINFO();
        PH.setPack(p152);
        p152.brkval_SET((char)48713);
        p152.freemem_SET((char)9252);
        p152.freemem32_SET(762190695L, PH);
        CommunicationChannel.instance.send(p152); //===============================
        AP_ADC p153 = CommunicationChannel.instance.new_AP_ADC();
        PH.setPack(p153);
        p153.adc1_SET((char)17411);
        p153.adc2_SET((char)32734);
        p153.adc3_SET((char)6157);
        p153.adc4_SET((char)5344);
        p153.adc5_SET((char)11394);
        p153.adc6_SET((char)56559);
        CommunicationChannel.instance.send(p153); //===============================
        DIGICAM_CONFIGURE p154 = CommunicationChannel.instance.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_system_SET((char)68);
        p154.target_component_SET((char)242);
        p154.mode_SET((char)176);
        p154.shutter_speed_SET((char)14332);
        p154.aperture_SET((char)91);
        p154.iso_SET((char)167);
        p154.exposure_type_SET((char)231);
        p154.command_id_SET((char)92);
        p154.engine_cut_off_SET((char)142);
        p154.extra_param_SET((char)192);
        p154.extra_value_SET(-6.0634953E37F);
        CommunicationChannel.instance.send(p154); //===============================
        DIGICAM_CONTROL p155 = CommunicationChannel.instance.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.target_system_SET((char)9);
        p155.target_component_SET((char)223);
        p155.session_SET((char)70);
        p155.zoom_pos_SET((char)181);
        p155.zoom_step_SET((byte)43);
        p155.focus_lock_SET((char)168);
        p155.shot_SET((char)129);
        p155.command_id_SET((char)92);
        p155.extra_param_SET((char)138);
        p155.extra_value_SET(-5.021856E37F);
        CommunicationChannel.instance.send(p155); //===============================
        MOUNT_CONFIGURE p156 = CommunicationChannel.instance.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.target_system_SET((char)89);
        p156.target_component_SET((char)188);
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT);
        p156.stab_roll_SET((char)153);
        p156.stab_pitch_SET((char)243);
        p156.stab_yaw_SET((char)238);
        CommunicationChannel.instance.send(p156); //===============================
        MOUNT_CONTROL p157 = CommunicationChannel.instance.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.target_system_SET((char)183);
        p157.target_component_SET((char)57);
        p157.input_a_SET(-1035333909);
        p157.input_b_SET(-1021952490);
        p157.input_c_SET(2011789460);
        p157.save_position_SET((char)221);
        CommunicationChannel.instance.send(p157); //===============================
        MOUNT_STATUS p158 = CommunicationChannel.instance.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_system_SET((char)14);
        p158.target_component_SET((char)36);
        p158.pointing_a_SET(1464583078);
        p158.pointing_b_SET(50946399);
        p158.pointing_c_SET(1394947178);
        CommunicationChannel.instance.send(p158); //===============================
        FENCE_POINT p160 = CommunicationChannel.instance.new_FENCE_POINT();
        PH.setPack(p160);
        p160.target_system_SET((char)133);
        p160.target_component_SET((char)203);
        p160.idx_SET((char)131);
        p160.count_SET((char)109);
        p160.lat_SET(1.6543593E38F);
        p160.lng_SET(-1.968077E38F);
        CommunicationChannel.instance.send(p160); //===============================
        FENCE_FETCH_POINT p161 = CommunicationChannel.instance.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.target_system_SET((char)83);
        p161.target_component_SET((char)252);
        p161.idx_SET((char)158);
        CommunicationChannel.instance.send(p161); //===============================
        FENCE_STATUS p162 = CommunicationChannel.instance.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_status_SET((char)134);
        p162.breach_count_SET((char)50925);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_NONE);
        p162.breach_time_SET(947174136L);
        CommunicationChannel.instance.send(p162); //===============================
        AHRS p163 = CommunicationChannel.instance.new_AHRS();
        PH.setPack(p163);
        p163.omegaIx_SET(7.336968E37F);
        p163.omegaIy_SET(-1.7839765E38F);
        p163.omegaIz_SET(2.9875714E38F);
        p163.accel_weight_SET(-2.7931789E38F);
        p163.renorm_val_SET(6.4399145E37F);
        p163.error_rp_SET(-4.3451716E37F);
        p163.error_yaw_SET(-2.7832435E37F);
        CommunicationChannel.instance.send(p163); //===============================
        SIMSTATE p164 = CommunicationChannel.instance.new_SIMSTATE();
        PH.setPack(p164);
        p164.roll_SET(-1.943251E38F);
        p164.pitch_SET(-2.1915882E38F);
        p164.yaw_SET(1.5748989E38F);
        p164.xacc_SET(3.292845E37F);
        p164.yacc_SET(1.5004233E38F);
        p164.zacc_SET(-3.3205233E38F);
        p164.xgyro_SET(3.1542465E38F);
        p164.ygyro_SET(4.9522485E37F);
        p164.zgyro_SET(2.7123405E38F);
        p164.lat_SET(2060931923);
        p164.lng_SET(-988807431);
        CommunicationChannel.instance.send(p164); //===============================
        HWSTATUS p165 = CommunicationChannel.instance.new_HWSTATUS();
        PH.setPack(p165);
        p165.Vcc_SET((char)58799);
        p165.I2Cerr_SET((char)170);
        CommunicationChannel.instance.send(p165); //===============================
        RADIO p166 = CommunicationChannel.instance.new_RADIO();
        PH.setPack(p166);
        p166.rssi_SET((char)241);
        p166.remrssi_SET((char)30);
        p166.txbuf_SET((char)40);
        p166.noise_SET((char)221);
        p166.remnoise_SET((char)118);
        p166.rxerrors_SET((char)16206);
        p166.fixed__SET((char)30965);
        CommunicationChannel.instance.send(p166); //===============================
        LIMITS_STATUS p167 = CommunicationChannel.instance.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.limits_state_SET(LIMITS_STATE.LIMITS_TRIGGERED);
        p167.last_trigger_SET(3441238915L);
        p167.last_action_SET(3888117788L);
        p167.last_recovery_SET(3319631972L);
        p167.last_clear_SET(534374765L);
        p167.breach_count_SET((char)49370);
        p167.mods_enabled_SET((LIMIT_MODULE.LIMIT_GPSLOCK |
                               LIMIT_MODULE.LIMIT_GEOFENCE));
        p167.mods_required_SET((LIMIT_MODULE.LIMIT_GEOFENCE |
                                LIMIT_MODULE.LIMIT_ALTITUDE));
        p167.mods_triggered_SET((LIMIT_MODULE.LIMIT_ALTITUDE));
        CommunicationChannel.instance.send(p167); //===============================
        WIND p168 = CommunicationChannel.instance.new_WIND();
        PH.setPack(p168);
        p168.direction_SET(1.3010436E38F);
        p168.speed_SET(8.5461695E36F);
        p168.speed_z_SET(-1.8908783E37F);
        CommunicationChannel.instance.send(p168); //===============================
        DATA16 p169 = CommunicationChannel.instance.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)191);
        p169.len_SET((char)53);
        p169.data__SET(new char[16], 0);
        CommunicationChannel.instance.send(p169); //===============================
        DATA32 p170 = CommunicationChannel.instance.new_DATA32();
        PH.setPack(p170);
        p170.type_SET((char)85);
        p170.len_SET((char)13);
        p170.data__SET(new char[32], 0);
        CommunicationChannel.instance.send(p170); //===============================
        DATA64 p171 = CommunicationChannel.instance.new_DATA64();
        PH.setPack(p171);
        p171.type_SET((char)162);
        p171.len_SET((char)107);
        p171.data__SET(new char[64], 0);
        CommunicationChannel.instance.send(p171); //===============================
        DATA96 p172 = CommunicationChannel.instance.new_DATA96();
        PH.setPack(p172);
        p172.type_SET((char)233);
        p172.len_SET((char)95);
        p172.data__SET(new char[96], 0);
        CommunicationChannel.instance.send(p172); //===============================
        RANGEFINDER p173 = CommunicationChannel.instance.new_RANGEFINDER();
        PH.setPack(p173);
        p173.distance_SET(-1.6670531E38F);
        p173.voltage_SET(6.3819175E37F);
        CommunicationChannel.instance.send(p173); //===============================
        AIRSPEED_AUTOCAL p174 = CommunicationChannel.instance.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.vx_SET(-4.724813E37F);
        p174.vy_SET(-2.5922502E38F);
        p174.vz_SET(-1.0115971E38F);
        p174.diff_pressure_SET(1.327701E37F);
        p174.EAS2TAS_SET(9.755543E37F);
        p174.ratio_SET(-1.3119773E38F);
        p174.state_x_SET(5.2726294E37F);
        p174.state_y_SET(1.8693017E38F);
        p174.state_z_SET(2.7076437E38F);
        p174.Pax_SET(4.7400955E37F);
        p174.Pby_SET(-3.0159438E38F);
        p174.Pcz_SET(-1.7977764E38F);
        CommunicationChannel.instance.send(p174); //===============================
        RALLY_POINT p175 = CommunicationChannel.instance.new_RALLY_POINT();
        PH.setPack(p175);
        p175.target_system_SET((char)111);
        p175.target_component_SET((char)251);
        p175.idx_SET((char)172);
        p175.count_SET((char)205);
        p175.lat_SET(-1016544010);
        p175.lng_SET(510677073);
        p175.alt_SET((short) -20069);
        p175.break_alt_SET((short)26555);
        p175.land_dir_SET((char)27055);
        p175.flags_SET(RALLY_FLAGS.FAVORABLE_WIND);
        CommunicationChannel.instance.send(p175); //===============================
        RALLY_FETCH_POINT p176 = CommunicationChannel.instance.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)124);
        p176.target_component_SET((char)191);
        p176.idx_SET((char)216);
        CommunicationChannel.instance.send(p176); //===============================
        COMPASSMOT_STATUS p177 = CommunicationChannel.instance.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.throttle_SET((char)37037);
        p177.current_SET(1.4951202E38F);
        p177.interference_SET((char)38405);
        p177.CompensationX_SET(2.952163E38F);
        p177.CompensationY_SET(7.401481E37F);
        p177.CompensationZ_SET(-9.901337E37F);
        CommunicationChannel.instance.send(p177); //===============================
        AHRS2 p178 = CommunicationChannel.instance.new_AHRS2();
        PH.setPack(p178);
        p178.roll_SET(-3.5079348E37F);
        p178.pitch_SET(-3.0918292E38F);
        p178.yaw_SET(-2.184111E38F);
        p178.altitude_SET(3.1967897E38F);
        p178.lat_SET(-1310490096);
        p178.lng_SET(-1410239056);
        CommunicationChannel.instance.send(p178); //===============================
        CAMERA_STATUS p179 = CommunicationChannel.instance.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.time_usec_SET(5164454633833872029L);
        p179.target_system_SET((char)112);
        p179.cam_idx_SET((char)103);
        p179.img_idx_SET((char)12255);
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_DISCONNECT);
        p179.p1_SET(-1.7847748E38F);
        p179.p2_SET(3.3435015E38F);
        p179.p3_SET(3.3320792E37F);
        p179.p4_SET(1.7079752E38F);
        CommunicationChannel.instance.send(p179); //===============================
        CAMERA_FEEDBACK p180 = CommunicationChannel.instance.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.time_usec_SET(2751929680773205705L);
        p180.target_system_SET((char)225);
        p180.cam_idx_SET((char)175);
        p180.img_idx_SET((char)48421);
        p180.lat_SET(-602854943);
        p180.lng_SET(134139916);
        p180.alt_msl_SET(-5.9713366E37F);
        p180.alt_rel_SET(-8.457612E37F);
        p180.roll_SET(1.0214277E38F);
        p180.pitch_SET(-2.9743344E38F);
        p180.yaw_SET(-2.6291288E38F);
        p180.foc_len_SET(-2.9227565E38F);
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_VIDEO);
        CommunicationChannel.instance.send(p180); //===============================
        BATTERY2 p181 = CommunicationChannel.instance.new_BATTERY2();
        PH.setPack(p181);
        p181.voltage_SET((char)31770);
        p181.current_battery_SET((short)19073);
        CommunicationChannel.instance.send(p181); //===============================
        AHRS3 p182 = CommunicationChannel.instance.new_AHRS3();
        PH.setPack(p182);
        p182.roll_SET(-2.4142306E38F);
        p182.pitch_SET(-5.783134E36F);
        p182.yaw_SET(-4.2676167E37F);
        p182.altitude_SET(2.3735709E38F);
        p182.lat_SET(2079440599);
        p182.lng_SET(-443339517);
        p182.v1_SET(-2.230447E38F);
        p182.v2_SET(3.1128605E38F);
        p182.v3_SET(-3.261617E36F);
        p182.v4_SET(3.1869933E38F);
        CommunicationChannel.instance.send(p182); //===============================
        AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.instance.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)171);
        p183.target_component_SET((char)207);
        CommunicationChannel.instance.send(p183); //===============================
        REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.instance.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.target_system_SET((char)81);
        p184.target_component_SET((char)110);
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
        p184.data__SET(new char[200], 0);
        CommunicationChannel.instance.send(p184); //===============================
        REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.instance.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_system_SET((char)59);
        p185.target_component_SET((char)173);
        p185.seqno_SET(400845412L);
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
        CommunicationChannel.instance.send(p185); //===============================
        LED_CONTROL p186 = CommunicationChannel.instance.new_LED_CONTROL();
        PH.setPack(p186);
        p186.target_system_SET((char)215);
        p186.target_component_SET((char)92);
        p186.instance_SET((char)17);
        p186.pattern_SET((char)113);
        p186.custom_len_SET((char)144);
        p186.custom_bytes_SET(new char[24], 0);
        CommunicationChannel.instance.send(p186); //===============================
        MAG_CAL_PROGRESS p191 = CommunicationChannel.instance.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.compass_id_SET((char)192);
        p191.cal_mask_SET((char)245);
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE);
        p191.attempt_SET((char)170);
        p191.completion_pct_SET((char)118);
        p191.completion_mask_SET(new char[10], 0);
        p191.direction_x_SET(-1.885494E38F);
        p191.direction_y_SET(2.5113217E38F);
        p191.direction_z_SET(-1.8311878E38F);
        CommunicationChannel.instance.send(p191); //===============================
        MAG_CAL_REPORT p192 = CommunicationChannel.instance.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.compass_id_SET((char)166);
        p192.cal_mask_SET((char)47);
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_TWO);
        p192.autosaved_SET((char)198);
        p192.fitness_SET(-1.7119191E38F);
        p192.ofs_x_SET(-1.7002053E38F);
        p192.ofs_y_SET(-3.5575633E37F);
        p192.ofs_z_SET(1.4464704E38F);
        p192.diag_x_SET(1.4162635E38F);
        p192.diag_y_SET(4.334997E37F);
        p192.diag_z_SET(-7.3651585E37F);
        p192.offdiag_x_SET(3.0674891E38F);
        p192.offdiag_y_SET(1.7278717E38F);
        p192.offdiag_z_SET(1.8230605E38F);
        CommunicationChannel.instance.send(p192); //===============================
        EKF_STATUS_REPORT p193 = CommunicationChannel.instance.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.flags_SET((EKF_STATUS_FLAGS.EKF_VELOCITY_VERT |
                        EKF_STATUS_FLAGS.EKF_ATTITUDE |
                        EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL |
                        EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ));
        p193.velocity_variance_SET(1.0703758E38F);
        p193.pos_horiz_variance_SET(-8.3282976E37F);
        p193.pos_vert_variance_SET(1.6159044E38F);
        p193.compass_variance_SET(1.0558667E38F);
        p193.terrain_alt_variance_SET(-1.7285452E38F);
        CommunicationChannel.instance.send(p193); //===============================
        PID_TUNING p194 = CommunicationChannel.instance.new_PID_TUNING();
        PH.setPack(p194);
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_STEER);
        p194.desired_SET(1.9613459E37F);
        p194.achieved_SET(1.6553028E38F);
        p194.FF_SET(-1.1921541E38F);
        p194.P_SET(-1.4813992E38F);
        p194.I_SET(-1.989926E38F);
        p194.D_SET(2.6887358E38F);
        CommunicationChannel.instance.send(p194); //===============================
        GIMBAL_REPORT p200 = CommunicationChannel.instance.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.target_system_SET((char)36);
        p200.target_component_SET((char)237);
        p200.delta_time_SET(4.2108592E37F);
        p200.delta_angle_x_SET(-6.1527557E37F);
        p200.delta_angle_y_SET(-9.059697E37F);
        p200.delta_angle_z_SET(-1.0964755E38F);
        p200.delta_velocity_x_SET(7.16887E37F);
        p200.delta_velocity_y_SET(1.2623653E38F);
        p200.delta_velocity_z_SET(1.4628888E38F);
        p200.joint_roll_SET(-2.2297224E38F);
        p200.joint_el_SET(5.5400286E37F);
        p200.joint_az_SET(5.74015E36F);
        CommunicationChannel.instance.send(p200); //===============================
        GIMBAL_CONTROL p201 = CommunicationChannel.instance.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.target_system_SET((char)238);
        p201.target_component_SET((char)163);
        p201.demanded_rate_x_SET(-2.8940425E38F);
        p201.demanded_rate_y_SET(-9.503731E37F);
        p201.demanded_rate_z_SET(-2.445953E38F);
        CommunicationChannel.instance.send(p201); //===============================
        GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.instance.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)48);
        p214.target_component_SET((char)209);
        p214.rl_torque_cmd_SET((short)17042);
        p214.el_torque_cmd_SET((short) -14712);
        p214.az_torque_cmd_SET((short) -29905);
        CommunicationChannel.instance.send(p214); //===============================
        GOPRO_HEARTBEAT p215 = CommunicationChannel.instance.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR);
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN);
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
        CommunicationChannel.instance.send(p215); //===============================
        GOPRO_GET_REQUEST p216 = CommunicationChannel.instance.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.target_system_SET((char)209);
        p216.target_component_SET((char)90);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
        CommunicationChannel.instance.send(p216); //===============================
        GOPRO_GET_RESPONSE p217 = CommunicationChannel.instance.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
        p217.value_SET(new char[4], 0);
        CommunicationChannel.instance.send(p217); //===============================
        GOPRO_SET_REQUEST p218 = CommunicationChannel.instance.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.target_system_SET((char)15);
        p218.target_component_SET((char)147);
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_SHARPNESS);
        p218.value_SET(new char[4], 0);
        CommunicationChannel.instance.send(p218); //===============================
        GOPRO_SET_RESPONSE p219 = CommunicationChannel.instance.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        CommunicationChannel.instance.send(p219); //===============================
        RPM p226 = CommunicationChannel.instance.new_RPM();
        PH.setPack(p226);
        p226.rpm1_SET(-1.1986555E38F);
        p226.rpm2_SET(-1.2476497E38F);
        CommunicationChannel.instance.send(p226); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(3467633907727309294L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL));
        p230.vel_ratio_SET(1.7618613E38F);
        p230.pos_horiz_ratio_SET(2.3096527E38F);
        p230.pos_vert_ratio_SET(-8.749852E37F);
        p230.mag_ratio_SET(4.192427E36F);
        p230.hagl_ratio_SET(-2.544176E38F);
        p230.tas_ratio_SET(3.1556357E38F);
        p230.pos_horiz_accuracy_SET(2.4367715E38F);
        p230.pos_vert_accuracy_SET(1.963665E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(6011338639710123295L);
        p231.wind_x_SET(-7.128008E37F);
        p231.wind_y_SET(2.7882715E38F);
        p231.wind_z_SET(1.3694718E38F);
        p231.var_horiz_SET(2.9479766E38F);
        p231.var_vert_SET(-2.0852242E38F);
        p231.wind_alt_SET(2.3980753E38F);
        p231.horiz_accuracy_SET(-2.4836036E38F);
        p231.vert_accuracy_SET(1.6882974E37F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(3936721647783074171L);
        p232.gps_id_SET((char)180);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
        p232.time_week_ms_SET(3730018534L);
        p232.time_week_SET((char)33341);
        p232.fix_type_SET((char)189);
        p232.lat_SET(1431616726);
        p232.lon_SET(-219631971);
        p232.alt_SET(7.022135E37F);
        p232.hdop_SET(2.5063653E38F);
        p232.vdop_SET(2.546738E38F);
        p232.vn_SET(1.6409816E38F);
        p232.ve_SET(-7.294446E37F);
        p232.vd_SET(-1.9912882E38F);
        p232.speed_accuracy_SET(-3.0968511E38F);
        p232.horiz_accuracy_SET(7.8183375E37F);
        p232.vert_accuracy_SET(1.8644789E37F);
        p232.satellites_visible_SET((char)17);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)103);
        p233.len_SET((char)253);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        p234.custom_mode_SET(4070765624L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        p234.roll_SET((short)24638);
        p234.pitch_SET((short)1010);
        p234.heading_SET((char)7135);
        p234.throttle_SET((byte) - 63);
        p234.heading_sp_SET((short)22819);
        p234.latitude_SET(-1149268673);
        p234.longitude_SET(-1092482976);
        p234.altitude_amsl_SET((short)3974);
        p234.altitude_sp_SET((short) -16920);
        p234.airspeed_SET((char)143);
        p234.airspeed_sp_SET((char)249);
        p234.groundspeed_SET((char)185);
        p234.climb_rate_SET((byte) - 45);
        p234.gps_nsat_SET((char)93);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
        p234.battery_remaining_SET((char)82);
        p234.temperature_SET((byte)15);
        p234.temperature_air_SET((byte)67);
        p234.failsafe_SET((char)197);
        p234.wp_num_SET((char)17);
        p234.wp_distance_SET((char)7996);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(5037331634711754434L);
        p241.vibration_x_SET(3.0125402E37F);
        p241.vibration_y_SET(-2.1628115E38F);
        p241.vibration_z_SET(2.2215508E37F);
        p241.clipping_0_SET(4238339766L);
        p241.clipping_1_SET(1093088378L);
        p241.clipping_2_SET(1156748469L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-1293518257);
        p242.longitude_SET(23594035);
        p242.altitude_SET(-175632735);
        p242.x_SET(3.7776378E37F);
        p242.y_SET(1.1540217E38F);
        p242.z_SET(-3.2409044E37F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(-1.0358201E38F);
        p242.approach_y_SET(-5.570964E37F);
        p242.approach_z_SET(2.2878734E38F);
        p242.time_usec_SET(2332235953722853444L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)39);
        p243.latitude_SET(1772586471);
        p243.longitude_SET(-2023055237);
        p243.altitude_SET(-181491392);
        p243.x_SET(2.6943924E38F);
        p243.y_SET(2.5460973E38F);
        p243.z_SET(-1.9671169E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-2.1805744E38F);
        p243.approach_y_SET(1.9625274E38F);
        p243.approach_z_SET(7.3803257E37F);
        p243.time_usec_SET(5846956058087320008L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)20156);
        p244.interval_us_SET(1801733096);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(1587764754L);
        p246.lat_SET(-1171849852);
        p246.lon_SET(1036731614);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(880285626);
        p246.heading_SET((char)7526);
        p246.hor_velocity_SET((char)8015);
        p246.ver_velocity_SET((short) -6334);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
        p246.tslc_SET((char)218);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
        p246.squawk_SET((char)63755);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(3519277289L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
        p247.time_to_minimum_delta_SET(-1.568165E38F);
        p247.altitude_minimum_delta_SET(-2.5065882E38F);
        p247.horizontal_minimum_delta_SET(-2.288486E38F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)21);
        p248.target_system_SET((char)202);
        p248.target_component_SET((char)86);
        p248.message_type_SET((char)11718);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)49928);
        p249.ver_SET((char)117);
        p249.type_SET((char)38);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(1202695877269651458L);
        p250.x_SET(-2.1418042E38F);
        p250.y_SET(-2.3754768E38F);
        p250.z_SET(1.0536408E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2181227496L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(2.6117022E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(2521326316L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-1256283886);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(1893485469L);
        p254.ind_SET((char)195);
        p254.value_SET(-2.3296879E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)221);
        p256.target_component_SET((char)167);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(8130427101718790876L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(4087037877L);
        p257.last_change_ms_SET(1358077362L);
        p257.state_SET((char)229);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)6);
        p258.target_component_SET((char)162);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(240179050L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(3881726818L);
        p259.focal_length_SET(1.2834869E38F);
        p259.sensor_size_h_SET(-2.3930256E38F);
        p259.sensor_size_v_SET(1.7461523E37F);
        p259.resolution_h_SET((char)15859);
        p259.resolution_v_SET((char)28402);
        p259.lens_id_SET((char)250);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
        p259.cam_definition_version_SET((char)50207);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(834636249L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3385460085L);
        p261.storage_id_SET((char)0);
        p261.storage_count_SET((char)161);
        p261.status_SET((char)129);
        p261.total_capacity_SET(-1.2125914E38F);
        p261.used_capacity_SET(2.6704332E38F);
        p261.available_capacity_SET(3.0503568E37F);
        p261.read_speed_SET(1.0759089E38F);
        p261.write_speed_SET(-5.4079927E37F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2167450015L);
        p262.image_status_SET((char)79);
        p262.video_status_SET((char)134);
        p262.image_interval_SET(1.4526071E38F);
        p262.recording_time_ms_SET(2798561321L);
        p262.available_capacity_SET(3.331007E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(1630094920L);
        p263.time_utc_SET(7410931330622230157L);
        p263.camera_id_SET((char)248);
        p263.lat_SET(-774211526);
        p263.lon_SET(352885827);
        p263.alt_SET(-1720777569);
        p263.relative_alt_SET(-1957180606);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(480762826);
        p263.capture_result_SET((byte) - 62);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3431182714L);
        p264.arming_time_utc_SET(995862892132476689L);
        p264.takeoff_time_utc_SET(5930564851473386182L);
        p264.flight_uuid_SET(5384674222029600842L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1750779815L);
        p265.roll_SET(-2.355464E38F);
        p265.pitch_SET(-2.5513237E38F);
        p265.yaw_SET(1.1318899E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)53);
        p266.target_component_SET((char)150);
        p266.sequence_SET((char)54530);
        p266.length_SET((char)82);
        p266.first_message_offset_SET((char)186);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)116);
        p267.target_component_SET((char)23);
        p267.sequence_SET((char)44163);
        p267.length_SET((char)44);
        p267.first_message_offset_SET((char)226);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)248);
        p268.target_component_SET((char)7);
        p268.sequence_SET((char)9198);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)135);
        p269.status_SET((char)183);
        p269.framerate_SET(2.9278382E38F);
        p269.resolution_h_SET((char)65033);
        p269.resolution_v_SET((char)16833);
        p269.bitrate_SET(29883968L);
        p269.rotation_SET((char)60272);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)55);
        p270.target_component_SET((char)159);
        p270.camera_id_SET((char)75);
        p270.framerate_SET(1.4766785E38F);
        p270.resolution_h_SET((char)15879);
        p270.resolution_v_SET((char)38798);
        p270.bitrate_SET(3835514094L);
        p270.rotation_SET((char)56444);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)2322);
        p300.min_version_SET((char)50674);
        p300.max_version_SET((char)22259);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(4747560687967502087L);
        p310.uptime_sec_SET(1760056377L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        p310.sub_mode_SET((char)67);
        p310.vendor_specific_status_code_SET((char)7326);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(1675516313329566113L);
        p311.uptime_sec_SET(2659217882L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)107);
        p311.hw_version_minor_SET((char)189);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)92);
        p311.sw_version_minor_SET((char)61);
        p311.sw_vcs_commit_SET(791567117L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)198);
        p320.target_component_SET((char)53);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -25128);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)201);
        p321.target_component_SET((char)235);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p322.param_count_SET((char)39888);
        p322.param_index_SET((char)5145);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)205);
        p323.target_component_SET((char)152);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(3379856835266957810L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)90);
        p330.min_distance_SET((char)19449);
        p330.max_distance_SET((char)13588);
        CommunicationChannel.instance.send(p330); //===============================
        UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.instance.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.ICAO_SET(606401956L);
        p10001.callsign_SET("DEMO", PH);
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M);
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
        p10001.stallSpeed_SET((char)8923);
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
        CommunicationChannel.instance.send(p10001); //===============================
        UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.instance.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.utcTime_SET(2364719858L);
        p10002.gpsLat_SET(1820591457);
        p10002.gpsLon_SET(151179780);
        p10002.gpsAlt_SET(1057971032);
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D);
        p10002.numSats_SET((char)187);
        p10002.baroAltMSL_SET(-1700629761);
        p10002.accuracyHor_SET(152873333L);
        p10002.accuracyVert_SET((char)42164);
        p10002.accuracyVel_SET((char)46043);
        p10002.velVert_SET((short)28555);
        p10002.velNS_SET((short)3260);
        p10002.VelEW_SET((short) -28664);
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
        p10002.state_SET((UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND |
                          UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE));
        p10002.squawk_SET((char)4530);
        CommunicationChannel.instance.send(p10002); //===============================
        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.instance.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
        CommunicationChannel.instance.send(p10003); //===============================
        DEVICE_OP_READ p11000 = CommunicationChannel.instance.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.target_system_SET((char)56);
        p11000.target_component_SET((char)94);
        p11000.request_id_SET(2155536292L);
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11000.bus_SET((char)215);
        p11000.address_SET((char)17);
        p11000.busname_SET("DEMO", PH);
        p11000.regstart_SET((char)100);
        p11000.count_SET((char)51);
        CommunicationChannel.instance.send(p11000); //===============================
        DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.instance.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.request_id_SET(1523667750L);
        p11001.result_SET((char)146);
        p11001.regstart_SET((char)198);
        p11001.count_SET((char)102);
        p11001.data__SET(new char[128], 0);
        CommunicationChannel.instance.send(p11001); //===============================
        DEVICE_OP_WRITE p11002 = CommunicationChannel.instance.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.target_system_SET((char)85);
        p11002.target_component_SET((char)135);
        p11002.request_id_SET(1603896683L);
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11002.bus_SET((char)72);
        p11002.address_SET((char)112);
        p11002.busname_SET("DEMO", PH);
        p11002.regstart_SET((char)186);
        p11002.count_SET((char)72);
        p11002.data__SET(new char[128], 0);
        CommunicationChannel.instance.send(p11002); //===============================
        DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.instance.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.request_id_SET(596945981L);
        p11003.result_SET((char)192);
        CommunicationChannel.instance.send(p11003); //===============================
        ADAP_TUNING p11010 = CommunicationChannel.instance.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_YAW);
        p11010.desired_SET(1.5801986E37F);
        p11010.achieved_SET(-2.2098546E36F);
        p11010.error_SET(-1.3698587E38F);
        p11010.theta_SET(2.7141976E37F);
        p11010.omega_SET(-9.578386E37F);
        p11010.sigma_SET(1.486802E38F);
        p11010.theta_dot_SET(-3.328746E38F);
        p11010.omega_dot_SET(-9.764313E36F);
        p11010.sigma_dot_SET(-2.9731688E38F);
        p11010.f_SET(1.5635817E37F);
        p11010.f_dot_SET(-7.5983596E37F);
        p11010.u_SET(1.4498732E38F);
        CommunicationChannel.instance.send(p11010); //===============================
        VISION_POSITION_DELTA p11011 = CommunicationChannel.instance.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.time_usec_SET(9076395679858095512L);
        p11011.time_delta_usec_SET(58113789563649525L);
        p11011.angle_delta_SET(new float[3], 0);
        p11011.position_delta_SET(new float[3], 0);
        p11011.confidence_SET(-7.7314583E37F);
        CommunicationChannel.instance.send(p11011); //===============================
    }
}

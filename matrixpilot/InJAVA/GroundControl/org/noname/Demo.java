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
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(4198027050L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p87.type_mask_SET((char)56712);
        p87.lat_int_SET(-1937837202);
        p87.lon_int_SET(-1659278074);
        p87.alt_SET(-1.8500314E38F);
        p87.vx_SET(2.5243816E38F);
        p87.vy_SET(7.7925337E37F);
        p87.vz_SET(2.9820939E38F);
        p87.afx_SET(2.905997E38F);
        p87.afy_SET(1.2617546E38F);
        p87.afz_SET(1.1076254E38F);
        p87.yaw_SET(-1.2012545E38F);
        p87.yaw_rate_SET(1.0180978E38F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(3245655026L);
        p89.x_SET(-1.5654658E38F);
        p89.y_SET(-7.930581E37F);
        p89.z_SET(3.3581052E38F);
        p89.roll_SET(-2.1600912E36F);
        p89.pitch_SET(-1.2013295E38F);
        p89.yaw_SET(-5.8979234E37F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(406521005127781439L);
        p90.roll_SET(1.3421473E38F);
        p90.pitch_SET(-1.2684617E38F);
        p90.yaw_SET(1.0091131E38F);
        p90.rollspeed_SET(1.0407458E38F);
        p90.pitchspeed_SET(1.827645E38F);
        p90.yawspeed_SET(-2.62548E38F);
        p90.lat_SET(435769754);
        p90.lon_SET(1201393061);
        p90.alt_SET(1863189288);
        p90.vx_SET((short) -5516);
        p90.vy_SET((short)26711);
        p90.vz_SET((short)11222);
        p90.xacc_SET((short) -6862);
        p90.yacc_SET((short)24774);
        p90.zacc_SET((short)29665);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(9027485485958712012L);
        p91.roll_ailerons_SET(2.3963888E38F);
        p91.pitch_elevator_SET(2.431092E38F);
        p91.yaw_rudder_SET(-2.0804978E38F);
        p91.throttle_SET(1.4472923E38F);
        p91.aux1_SET(3.3903092E38F);
        p91.aux2_SET(-2.9416852E38F);
        p91.aux3_SET(2.1489594E38F);
        p91.aux4_SET(-2.6782431E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
        p91.nav_mode_SET((char)92);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(2057993809251560444L);
        p92.chan1_raw_SET((char)54352);
        p92.chan2_raw_SET((char)57433);
        p92.chan3_raw_SET((char)56122);
        p92.chan4_raw_SET((char)20390);
        p92.chan5_raw_SET((char)2883);
        p92.chan6_raw_SET((char)60810);
        p92.chan7_raw_SET((char)5373);
        p92.chan8_raw_SET((char)40876);
        p92.chan9_raw_SET((char)48504);
        p92.chan10_raw_SET((char)25273);
        p92.chan11_raw_SET((char)60445);
        p92.chan12_raw_SET((char)51800);
        p92.rssi_SET((char)29);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(1961382184282126489L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED);
        p93.flags_SET(2838616489606345188L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(5531133559192058396L);
        p100.sensor_id_SET((char)162);
        p100.flow_x_SET((short)8695);
        p100.flow_y_SET((short) -2389);
        p100.flow_comp_m_x_SET(4.0452807E37F);
        p100.flow_comp_m_y_SET(-3.388562E38F);
        p100.quality_SET((char)209);
        p100.ground_distance_SET(-1.1966163E38F);
        p100.flow_rate_x_SET(-1.2647183E38F, PH);
        p100.flow_rate_y_SET(-3.1318939E38F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(2802220338969540288L);
        p101.x_SET(-1.954839E38F);
        p101.y_SET(2.9738018E38F);
        p101.z_SET(-1.3315907E38F);
        p101.roll_SET(3.0807745E38F);
        p101.pitch_SET(3.109643E38F);
        p101.yaw_SET(-6.169221E37F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(1839200319971999983L);
        p102.x_SET(2.7196365E38F);
        p102.y_SET(1.2902964E38F);
        p102.z_SET(2.554875E38F);
        p102.roll_SET(-1.3548386E38F);
        p102.pitch_SET(-2.0242392E38F);
        p102.yaw_SET(1.7687823E38F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(3324656705555920129L);
        p103.x_SET(2.878068E38F);
        p103.y_SET(1.5879834E38F);
        p103.z_SET(-4.260168E37F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(5098575440077535151L);
        p104.x_SET(2.2876798E38F);
        p104.y_SET(2.2418543E38F);
        p104.z_SET(-8.564666E37F);
        p104.roll_SET(-1.5060174E38F);
        p104.pitch_SET(3.2159209E38F);
        p104.yaw_SET(-1.4569498E38F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(2642671217021357830L);
        p105.xacc_SET(4.462838E37F);
        p105.yacc_SET(1.7495075E38F);
        p105.zacc_SET(-6.1176194E37F);
        p105.xgyro_SET(3.9543557E37F);
        p105.ygyro_SET(1.1714197E38F);
        p105.zgyro_SET(4.4539675E37F);
        p105.xmag_SET(-5.2679873E37F);
        p105.ymag_SET(-2.7204066E38F);
        p105.zmag_SET(-3.1094538E38F);
        p105.abs_pressure_SET(-2.1904055E38F);
        p105.diff_pressure_SET(-2.203651E38F);
        p105.pressure_alt_SET(-1.2727796E38F);
        p105.temperature_SET(2.3544338E38F);
        p105.fields_updated_SET((char)17145);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(5142833101834568329L);
        p106.sensor_id_SET((char)89);
        p106.integration_time_us_SET(1482240510L);
        p106.integrated_x_SET(-3.2567481E38F);
        p106.integrated_y_SET(8.955543E37F);
        p106.integrated_xgyro_SET(3.0279108E38F);
        p106.integrated_ygyro_SET(-2.2656007E38F);
        p106.integrated_zgyro_SET(2.6594054E38F);
        p106.temperature_SET((short)15749);
        p106.quality_SET((char)218);
        p106.time_delta_distance_us_SET(2487718518L);
        p106.distance_SET(-9.082663E37F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(8983140532741053628L);
        p107.xacc_SET(-1.6947604E37F);
        p107.yacc_SET(-1.1362642E38F);
        p107.zacc_SET(-2.0529735E38F);
        p107.xgyro_SET(2.4090272E38F);
        p107.ygyro_SET(-2.9138659E38F);
        p107.zgyro_SET(2.0819928E37F);
        p107.xmag_SET(7.7358677E37F);
        p107.ymag_SET(-1.1244844E38F);
        p107.zmag_SET(1.1447678E38F);
        p107.abs_pressure_SET(1.393086E37F);
        p107.diff_pressure_SET(-2.4603814E38F);
        p107.pressure_alt_SET(-2.7694663E38F);
        p107.temperature_SET(-1.4208056E38F);
        p107.fields_updated_SET(3296782287L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-7.8714085E37F);
        p108.q2_SET(2.7130763E38F);
        p108.q3_SET(1.4159356E38F);
        p108.q4_SET(6.0712143E37F);
        p108.roll_SET(1.2422327E38F);
        p108.pitch_SET(1.0821904E38F);
        p108.yaw_SET(3.2024629E38F);
        p108.xacc_SET(-1.6228871E37F);
        p108.yacc_SET(-3.156469E38F);
        p108.zacc_SET(2.7807904E37F);
        p108.xgyro_SET(6.21858E37F);
        p108.ygyro_SET(2.3012554E38F);
        p108.zgyro_SET(-2.5491527E36F);
        p108.lat_SET(1.6779289E38F);
        p108.lon_SET(6.8052783E37F);
        p108.alt_SET(-2.8605584E38F);
        p108.std_dev_horz_SET(2.4866575E38F);
        p108.std_dev_vert_SET(2.9879675E38F);
        p108.vn_SET(-2.8488675E38F);
        p108.ve_SET(-2.0862624E38F);
        p108.vd_SET(-1.3090461E37F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)187);
        p109.remrssi_SET((char)190);
        p109.txbuf_SET((char)155);
        p109.noise_SET((char)58);
        p109.remnoise_SET((char)203);
        p109.rxerrors_SET((char)47069);
        p109.fixed__SET((char)60289);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)164);
        p110.target_system_SET((char)62);
        p110.target_component_SET((char)173);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(7800058828807514458L);
        p111.ts1_SET(3592846269891078405L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8399402422918761932L);
        p112.seq_SET(57190715L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(1717054611187300862L);
        p113.fix_type_SET((char)251);
        p113.lat_SET(778176635);
        p113.lon_SET(-459588408);
        p113.alt_SET(643175129);
        p113.eph_SET((char)47778);
        p113.epv_SET((char)12049);
        p113.vel_SET((char)45996);
        p113.vn_SET((short) -5770);
        p113.ve_SET((short) -16066);
        p113.vd_SET((short) -109);
        p113.cog_SET((char)37247);
        p113.satellites_visible_SET((char)105);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(4675146663313022725L);
        p114.sensor_id_SET((char)206);
        p114.integration_time_us_SET(4039847884L);
        p114.integrated_x_SET(-2.6265205E38F);
        p114.integrated_y_SET(-1.2994193E38F);
        p114.integrated_xgyro_SET(3.3195013E38F);
        p114.integrated_ygyro_SET(3.4873555E37F);
        p114.integrated_zgyro_SET(2.2255888E38F);
        p114.temperature_SET((short) -3293);
        p114.quality_SET((char)128);
        p114.time_delta_distance_us_SET(471974836L);
        p114.distance_SET(3.1158866E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(2856074091375171703L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(3.1862063E38F);
        p115.pitchspeed_SET(-1.0850351E38F);
        p115.yawspeed_SET(3.853519E37F);
        p115.lat_SET(507211532);
        p115.lon_SET(1623899703);
        p115.alt_SET(-1528961565);
        p115.vx_SET((short) -1531);
        p115.vy_SET((short) -26912);
        p115.vz_SET((short) -19082);
        p115.ind_airspeed_SET((char)3372);
        p115.true_airspeed_SET((char)47709);
        p115.xacc_SET((short) -31414);
        p115.yacc_SET((short) -19351);
        p115.zacc_SET((short)1419);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(4030027606L);
        p116.xacc_SET((short) -17694);
        p116.yacc_SET((short) -15351);
        p116.zacc_SET((short)348);
        p116.xgyro_SET((short) -9596);
        p116.ygyro_SET((short) -23584);
        p116.zgyro_SET((short) -19554);
        p116.xmag_SET((short)19381);
        p116.ymag_SET((short) -4052);
        p116.zmag_SET((short)1000);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)162);
        p117.target_component_SET((char)16);
        p117.start_SET((char)14834);
        p117.end_SET((char)17454);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)27694);
        p118.num_logs_SET((char)60482);
        p118.last_log_num_SET((char)52545);
        p118.time_utc_SET(4001487616L);
        p118.size_SET(637312980L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)82);
        p119.target_component_SET((char)171);
        p119.id_SET((char)13577);
        p119.ofs_SET(2738725218L);
        p119.count_SET(1959121139L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)49121);
        p120.ofs_SET(2590649496L);
        p120.count_SET((char)73);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)127);
        p121.target_component_SET((char)146);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)163);
        p122.target_component_SET((char)100);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)183);
        p123.target_component_SET((char)239);
        p123.len_SET((char)236);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(3127523355706216328L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
        p124.lat_SET(-796832572);
        p124.lon_SET(-323918424);
        p124.alt_SET(565332079);
        p124.eph_SET((char)43275);
        p124.epv_SET((char)11124);
        p124.vel_SET((char)31843);
        p124.cog_SET((char)10120);
        p124.satellites_visible_SET((char)125);
        p124.dgps_numch_SET((char)178);
        p124.dgps_age_SET(3178477176L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)2473);
        p125.Vservo_SET((char)48096);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
        p126.timeout_SET((char)22119);
        p126.baudrate_SET(2122927866L);
        p126.count_SET((char)10);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(4126358615L);
        p127.rtk_receiver_id_SET((char)42);
        p127.wn_SET((char)50587);
        p127.tow_SET(3692267065L);
        p127.rtk_health_SET((char)165);
        p127.rtk_rate_SET((char)53);
        p127.nsats_SET((char)207);
        p127.baseline_coords_type_SET((char)125);
        p127.baseline_a_mm_SET(1687772204);
        p127.baseline_b_mm_SET(1852940686);
        p127.baseline_c_mm_SET(2116186410);
        p127.accuracy_SET(540928718L);
        p127.iar_num_hypotheses_SET(-2026227349);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(931633964L);
        p128.rtk_receiver_id_SET((char)147);
        p128.wn_SET((char)26729);
        p128.tow_SET(861026974L);
        p128.rtk_health_SET((char)129);
        p128.rtk_rate_SET((char)54);
        p128.nsats_SET((char)80);
        p128.baseline_coords_type_SET((char)90);
        p128.baseline_a_mm_SET(1230056157);
        p128.baseline_b_mm_SET(-135469545);
        p128.baseline_c_mm_SET(-58357057);
        p128.accuracy_SET(3360030492L);
        p128.iar_num_hypotheses_SET(141315809);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(1699253960L);
        p129.xacc_SET((short) -10171);
        p129.yacc_SET((short) -31326);
        p129.zacc_SET((short) -6077);
        p129.xgyro_SET((short) -29869);
        p129.ygyro_SET((short) -23734);
        p129.zgyro_SET((short) -14946);
        p129.xmag_SET((short) -14221);
        p129.ymag_SET((short) -11351);
        p129.zmag_SET((short) -27342);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)249);
        p130.size_SET(21060246L);
        p130.width_SET((char)26492);
        p130.height_SET((char)38043);
        p130.packets_SET((char)33039);
        p130.payload_SET((char)171);
        p130.jpg_quality_SET((char)199);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)28925);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(2571152850L);
        p132.min_distance_SET((char)2510);
        p132.max_distance_SET((char)50076);
        p132.current_distance_SET((char)58406);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
        p132.id_SET((char)162);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90);
        p132.covariance_SET((char)201);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(151363082);
        p133.lon_SET(1668951715);
        p133.grid_spacing_SET((char)51845);
        p133.mask_SET(466229192314270787L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-420119788);
        p134.lon_SET(-254324497);
        p134.grid_spacing_SET((char)61072);
        p134.gridbit_SET((char)109);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1952129159);
        p135.lon_SET(-261450349);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(1248329858);
        p136.lon_SET(-1278835347);
        p136.spacing_SET((char)3231);
        p136.terrain_height_SET(-9.622248E37F);
        p136.current_height_SET(4.364251E37F);
        p136.pending_SET((char)15250);
        p136.loaded_SET((char)37979);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1411694115L);
        p137.press_abs_SET(4.917875E37F);
        p137.press_diff_SET(-5.6219437E37F);
        p137.temperature_SET((short) -18491);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(8555850688548567276L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-2.5737043E38F);
        p138.y_SET(8.888613E37F);
        p138.z_SET(7.6306816E37F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(9164354150897353421L);
        p139.group_mlx_SET((char)231);
        p139.target_system_SET((char)155);
        p139.target_component_SET((char)105);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5387452950239896247L);
        p140.group_mlx_SET((char)165);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(4707211027588198933L);
        p141.altitude_monotonic_SET(2.8494455E38F);
        p141.altitude_amsl_SET(7.64031E37F);
        p141.altitude_local_SET(2.1794997E38F);
        p141.altitude_relative_SET(-2.2742429E38F);
        p141.altitude_terrain_SET(1.2619601E38F);
        p141.bottom_clearance_SET(1.0922803E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)195);
        p142.uri_type_SET((char)102);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)202);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1631044985L);
        p143.press_abs_SET(-1.1097165E38F);
        p143.press_diff_SET(1.7008578E38F);
        p143.temperature_SET((short)1117);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(3352332654542602738L);
        p144.est_capabilities_SET((char)78);
        p144.lat_SET(-1113923888);
        p144.lon_SET(1319683989);
        p144.alt_SET(-5.953311E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(5800196840714450280L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(29095026372523945L);
        p146.x_acc_SET(2.0826057E38F);
        p146.y_acc_SET(-1.0639214E38F);
        p146.z_acc_SET(2.3770282E38F);
        p146.x_vel_SET(-9.990401E37F);
        p146.y_vel_SET(-1.943241E38F);
        p146.z_vel_SET(-2.2203476E37F);
        p146.x_pos_SET(-6.165138E37F);
        p146.y_pos_SET(2.9967692E38F);
        p146.z_pos_SET(-3.0879186E38F);
        p146.airspeed_SET(-4.7088007E37F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-2.3245554E38F);
        p146.pitch_rate_SET(1.4733174E38F);
        p146.yaw_rate_SET(1.9754763E38F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)247);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
        p147.temperature_SET((short) -3335);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)20470);
        p147.current_consumed_SET(-51164244);
        p147.energy_consumed_SET(-2076742426);
        p147.battery_remaining_SET((byte) - 35);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
        p148.flight_sw_version_SET(202351366L);
        p148.middleware_sw_version_SET(2761290561L);
        p148.os_sw_version_SET(793691536L);
        p148.board_version_SET(4092406962L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)1863);
        p148.product_id_SET((char)29148);
        p148.uid_SET(1367186541955535022L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(7497603861888438976L);
        p149.target_num_SET((char)227);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p149.angle_x_SET(-1.1987371E38F);
        p149.angle_y_SET(3.0001644E37F);
        p149.distance_SET(9.4170366E36F);
        p149.size_x_SET(-1.281935E38F);
        p149.size_y_SET(-1.5767509E38F);
        p149.x_SET(1.7005801E38F, PH);
        p149.y_SET(2.3790684E38F, PH);
        p149.z_SET(-2.972086E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
        p149.position_valid_SET((char)62, PH);
        CommunicationChannel.instance.send(p149); //===============================
        FLEXIFUNCTION_SET p150 = CommunicationChannel.instance.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_system_SET((char)212);
        p150.target_component_SET((char)195);
        CommunicationChannel.instance.send(p150); //===============================
        FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.instance.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.target_system_SET((char)81);
        p151.target_component_SET((char)29);
        p151.read_req_type_SET((short)16845);
        p151.data_index_SET((short)24077);
        CommunicationChannel.instance.send(p151); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.target_system_SET((char)94);
        p152.target_component_SET((char)10);
        p152.func_index_SET((char)14414);
        p152.func_count_SET((char)2541);
        p152.data_address_SET((char)41941);
        p152.data_size_SET((char)9340);
        p152.data__SET(new byte[48], 0);
        CommunicationChannel.instance.send(p152); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.target_system_SET((char)152);
        p153.target_component_SET((char)35);
        p153.func_index_SET((char)16147);
        p153.result_SET((char)36016);
        CommunicationChannel.instance.send(p153); //===============================
        FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.instance.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.target_system_SET((char)0);
        p155.target_component_SET((char)136);
        p155.directory_type_SET((char)134);
        p155.start_index_SET((char)44);
        p155.count_SET((char)152);
        p155.directory_data_SET(new byte[48], 0);
        CommunicationChannel.instance.send(p155); //===============================
        FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.instance.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.target_system_SET((char)114);
        p156.target_component_SET((char)241);
        p156.directory_type_SET((char)55);
        p156.start_index_SET((char)23);
        p156.count_SET((char)169);
        p156.result_SET((char)6648);
        CommunicationChannel.instance.send(p156); //===============================
        FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.instance.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)177);
        p157.target_component_SET((char)99);
        p157.command_type_SET((char)129);
        CommunicationChannel.instance.send(p157); //===============================
        FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.instance.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)39669);
        p158.result_SET((char)26256);
        CommunicationChannel.instance.send(p158); //===============================
        SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_time_SET(3711125963L);
        p170.sue_status_SET((char)158);
        p170.sue_latitude_SET(1937731957);
        p170.sue_longitude_SET(1146791586);
        p170.sue_altitude_SET(-1117266604);
        p170.sue_waypoint_index_SET((char)46987);
        p170.sue_rmat0_SET((short) -17125);
        p170.sue_rmat1_SET((short) -18045);
        p170.sue_rmat2_SET((short) -19841);
        p170.sue_rmat3_SET((short)16578);
        p170.sue_rmat4_SET((short)26056);
        p170.sue_rmat5_SET((short) -22194);
        p170.sue_rmat6_SET((short)21138);
        p170.sue_rmat7_SET((short) -22338);
        p170.sue_rmat8_SET((short)21637);
        p170.sue_cog_SET((char)18073);
        p170.sue_sog_SET((short)4073);
        p170.sue_cpu_load_SET((char)26549);
        p170.sue_air_speed_3DIMU_SET((char)62860);
        p170.sue_estimated_wind_0_SET((short) -15584);
        p170.sue_estimated_wind_1_SET((short) -30034);
        p170.sue_estimated_wind_2_SET((short) -31941);
        p170.sue_magFieldEarth0_SET((short)26068);
        p170.sue_magFieldEarth1_SET((short) -26271);
        p170.sue_magFieldEarth2_SET((short) -15109);
        p170.sue_svs_SET((short)946);
        p170.sue_hdop_SET((short)27712);
        CommunicationChannel.instance.send(p170); //===============================
        SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_time_SET(3320258505L);
        p171.sue_pwm_input_1_SET((short) -16847);
        p171.sue_pwm_input_2_SET((short) -31801);
        p171.sue_pwm_input_3_SET((short) -11281);
        p171.sue_pwm_input_4_SET((short) -5140);
        p171.sue_pwm_input_5_SET((short)466);
        p171.sue_pwm_input_6_SET((short) -32618);
        p171.sue_pwm_input_7_SET((short)29300);
        p171.sue_pwm_input_8_SET((short)19648);
        p171.sue_pwm_input_9_SET((short) -23014);
        p171.sue_pwm_input_10_SET((short) -16980);
        p171.sue_pwm_input_11_SET((short) -25546);
        p171.sue_pwm_input_12_SET((short) -18433);
        p171.sue_pwm_output_1_SET((short) -9499);
        p171.sue_pwm_output_2_SET((short)614);
        p171.sue_pwm_output_3_SET((short)31681);
        p171.sue_pwm_output_4_SET((short)8982);
        p171.sue_pwm_output_5_SET((short)17441);
        p171.sue_pwm_output_6_SET((short) -20502);
        p171.sue_pwm_output_7_SET((short) -11683);
        p171.sue_pwm_output_8_SET((short)13010);
        p171.sue_pwm_output_9_SET((short)769);
        p171.sue_pwm_output_10_SET((short) -22491);
        p171.sue_pwm_output_11_SET((short)14411);
        p171.sue_pwm_output_12_SET((short)23204);
        p171.sue_imu_location_x_SET((short) -28505);
        p171.sue_imu_location_y_SET((short) -28694);
        p171.sue_imu_location_z_SET((short) -416);
        p171.sue_location_error_earth_x_SET((short)28819);
        p171.sue_location_error_earth_y_SET((short)31087);
        p171.sue_location_error_earth_z_SET((short) -11234);
        p171.sue_flags_SET(3894721869L);
        p171.sue_osc_fails_SET((short) -25656);
        p171.sue_imu_velocity_x_SET((short)3810);
        p171.sue_imu_velocity_y_SET((short)14084);
        p171.sue_imu_velocity_z_SET((short)31996);
        p171.sue_waypoint_goal_x_SET((short)17140);
        p171.sue_waypoint_goal_y_SET((short) -6137);
        p171.sue_waypoint_goal_z_SET((short)15745);
        p171.sue_aero_x_SET((short) -12024);
        p171.sue_aero_y_SET((short) -29431);
        p171.sue_aero_z_SET((short)774);
        p171.sue_barom_temp_SET((short)16255);
        p171.sue_barom_press_SET(-1100714400);
        p171.sue_barom_alt_SET(-1448182553);
        p171.sue_bat_volt_SET((short) -12586);
        p171.sue_bat_amp_SET((short)20111);
        p171.sue_bat_amp_hours_SET((short)27485);
        p171.sue_desired_height_SET((short) -22279);
        p171.sue_memory_stack_free_SET((short)26962);
        CommunicationChannel.instance.send(p171); //===============================
        SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)132);
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)123);
        p172.sue_PITCH_STABILIZATION_SET((char)9);
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)243);
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)165);
        p172.sue_AILERON_NAVIGATION_SET((char)100);
        p172.sue_RUDDER_NAVIGATION_SET((char)91);
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)183);
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)98);
        p172.sue_RACING_MODE_SET((char)63);
        CommunicationChannel.instance.send(p172); //===============================
        SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_YAWKP_AILERON_SET(-3.3912544E38F);
        p173.sue_YAWKD_AILERON_SET(-2.9593486E38F);
        p173.sue_ROLLKP_SET(-2.216251E38F);
        p173.sue_ROLLKD_SET(-1.8087808E38F);
        CommunicationChannel.instance.send(p173); //===============================
        SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_PITCHGAIN_SET(8.034445E37F);
        p174.sue_PITCHKD_SET(-1.5748096E38F);
        p174.sue_RUDDER_ELEV_MIX_SET(1.3942948E38F);
        p174.sue_ROLL_ELEV_MIX_SET(-1.1636141E38F);
        p174.sue_ELEVATOR_BOOST_SET(1.2561755E38F);
        CommunicationChannel.instance.send(p174); //===============================
        SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_YAWKP_RUDDER_SET(2.7544096E38F);
        p175.sue_YAWKD_RUDDER_SET(-8.008532E37F);
        p175.sue_ROLLKP_RUDDER_SET(3.7963296E37F);
        p175.sue_ROLLKD_RUDDER_SET(1.2672228E38F);
        p175.sue_RUDDER_BOOST_SET(-1.5279299E38F);
        p175.sue_RTL_PITCH_DOWN_SET(-2.1442845E38F);
        CommunicationChannel.instance.send(p175); //===============================
        SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_HEIGHT_TARGET_MAX_SET(1.7205067E38F);
        p176.sue_HEIGHT_TARGET_MIN_SET(4.0098476E37F);
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-2.0265813E38F);
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(1.8003302E38F);
        p176.sue_ALT_HOLD_PITCH_MIN_SET(-2.2418379E38F);
        p176.sue_ALT_HOLD_PITCH_MAX_SET(9.565401E37F);
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(-1.7179004E38F);
        CommunicationChannel.instance.send(p176); //===============================
        SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_week_no_SET((short) -27747);
        p177.sue_lat_origin_SET(57730085);
        p177.sue_lon_origin_SET(-603441597);
        p177.sue_alt_origin_SET(-1320977997);
        CommunicationChannel.instance.send(p177); //===============================
        SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_WIND_ESTIMATION_SET((char)66);
        p178.sue_GPS_TYPE_SET((char)186);
        p178.sue_DR_SET((char)251);
        p178.sue_BOARD_TYPE_SET((char)217);
        p178.sue_AIRFRAME_SET((char)167);
        p178.sue_RCON_SET((short)5290);
        p178.sue_TRAP_FLAGS_SET((short) -22304);
        p178.sue_TRAP_SOURCE_SET(79880598L);
        p178.sue_osc_fail_count_SET((short)21629);
        p178.sue_CLOCK_CONFIG_SET((char)23);
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)73);
        CommunicationChannel.instance.send(p178); //===============================
        SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[40], 0);
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[20], 0);
        CommunicationChannel.instance.send(p179); //===============================
        SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[40], 0);
        p180.sue_ID_DIY_DRONES_URL_SET(new char[70], 0);
        CommunicationChannel.instance.send(p180); //===============================
        ALTITUDES p181 = CommunicationChannel.instance.new_ALTITUDES();
        PH.setPack(p181);
        p181.time_boot_ms_SET(3710054044L);
        p181.alt_gps_SET(923956119);
        p181.alt_imu_SET(1976293550);
        p181.alt_barometric_SET(160751492);
        p181.alt_optical_flow_SET(758214856);
        p181.alt_range_finder_SET(1332685796);
        p181.alt_extra_SET(-1488663375);
        CommunicationChannel.instance.send(p181); //===============================
        AIRSPEEDS p182 = CommunicationChannel.instance.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.time_boot_ms_SET(1588897327L);
        p182.airspeed_imu_SET((short) -372);
        p182.airspeed_pitot_SET((short) -15706);
        p182.airspeed_hot_wire_SET((short)11404);
        p182.airspeed_ultrasonic_SET((short) -21012);
        p182.aoa_SET((short) -11647);
        p182.aoy_SET((short) -1282);
        CommunicationChannel.instance.send(p182); //===============================
        SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_feed_forward_SET(-3.3692072E38F);
        p183.sue_turn_rate_nav_SET(-2.1213447E38F);
        p183.sue_turn_rate_fbw_SET(-2.2851931E38F);
        CommunicationChannel.instance.send(p183); //===============================
        SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.angle_of_attack_normal_SET(2.8983377E38F);
        p184.angle_of_attack_inverted_SET(3.3879003E38F);
        p184.elevator_trim_normal_SET(8.79315E36F);
        p184.elevator_trim_inverted_SET(2.5840404E38F);
        p184.reference_speed_SET(2.1893662E38F);
        CommunicationChannel.instance.send(p184); //===============================
        SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_aileron_output_channel_SET((char)112);
        p185.sue_aileron_reversed_SET((char)156);
        p185.sue_elevator_output_channel_SET((char)175);
        p185.sue_elevator_reversed_SET((char)241);
        p185.sue_throttle_output_channel_SET((char)108);
        p185.sue_throttle_reversed_SET((char)223);
        p185.sue_rudder_output_channel_SET((char)58);
        p185.sue_rudder_reversed_SET((char)47);
        CommunicationChannel.instance.send(p185); //===============================
        SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_number_of_inputs_SET((char)255);
        p186.sue_trim_value_input_1_SET((short)25331);
        p186.sue_trim_value_input_2_SET((short)20323);
        p186.sue_trim_value_input_3_SET((short)3132);
        p186.sue_trim_value_input_4_SET((short)20049);
        p186.sue_trim_value_input_5_SET((short) -32030);
        p186.sue_trim_value_input_6_SET((short)22924);
        p186.sue_trim_value_input_7_SET((short)9272);
        p186.sue_trim_value_input_8_SET((short)23450);
        p186.sue_trim_value_input_9_SET((short)30945);
        p186.sue_trim_value_input_10_SET((short) -1900);
        p186.sue_trim_value_input_11_SET((short) -12268);
        p186.sue_trim_value_input_12_SET((short)30590);
        CommunicationChannel.instance.send(p186); //===============================
        SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_accel_x_offset_SET((short) -6533);
        p187.sue_accel_y_offset_SET((short) -15734);
        p187.sue_accel_z_offset_SET((short) -14719);
        p187.sue_gyro_x_offset_SET((short) -15902);
        p187.sue_gyro_y_offset_SET((short)28553);
        p187.sue_gyro_z_offset_SET((short)5377);
        CommunicationChannel.instance.send(p187); //===============================
        SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_x_at_calibration_SET((short) -23540);
        p188.sue_accel_y_at_calibration_SET((short) -6465);
        p188.sue_accel_z_at_calibration_SET((short) -5507);
        p188.sue_gyro_x_at_calibration_SET((short)3400);
        p188.sue_gyro_y_at_calibration_SET((short)24657);
        p188.sue_gyro_z_at_calibration_SET((short)21657);
        CommunicationChannel.instance.send(p188); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(3114635346365680301L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL));
        p230.vel_ratio_SET(-7.112354E37F);
        p230.pos_horiz_ratio_SET(3.0126589E38F);
        p230.pos_vert_ratio_SET(-2.9568542E38F);
        p230.mag_ratio_SET(-3.2194993E38F);
        p230.hagl_ratio_SET(-1.4906205E38F);
        p230.tas_ratio_SET(-2.37922E38F);
        p230.pos_horiz_accuracy_SET(-1.7588778E38F);
        p230.pos_vert_accuracy_SET(-8.715432E37F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(3671138408402475027L);
        p231.wind_x_SET(2.8654191E38F);
        p231.wind_y_SET(-2.7870915E38F);
        p231.wind_z_SET(7.768007E37F);
        p231.var_horiz_SET(-7.0708805E37F);
        p231.var_vert_SET(2.1617647E38F);
        p231.wind_alt_SET(-7.7280174E37F);
        p231.horiz_accuracy_SET(4.8899105E37F);
        p231.vert_accuracy_SET(-2.7377452E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(4976975536443501733L);
        p232.gps_id_SET((char)237);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
        p232.time_week_ms_SET(2602987843L);
        p232.time_week_SET((char)3846);
        p232.fix_type_SET((char)199);
        p232.lat_SET(1030271256);
        p232.lon_SET(1516736173);
        p232.alt_SET(-1.792589E38F);
        p232.hdop_SET(2.4388164E38F);
        p232.vdop_SET(3.3993154E38F);
        p232.vn_SET(2.3198535E38F);
        p232.ve_SET(1.2293764E38F);
        p232.vd_SET(-7.476802E37F);
        p232.speed_accuracy_SET(1.0159799E37F);
        p232.horiz_accuracy_SET(-2.3783575E38F);
        p232.vert_accuracy_SET(3.1533224E38F);
        p232.satellites_visible_SET((char)186);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)57);
        p233.len_SET((char)20);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
        p234.custom_mode_SET(1293168334L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        p234.roll_SET((short)139);
        p234.pitch_SET((short) -27582);
        p234.heading_SET((char)10593);
        p234.throttle_SET((byte)52);
        p234.heading_sp_SET((short) -22813);
        p234.latitude_SET(-212758333);
        p234.longitude_SET(1594918872);
        p234.altitude_amsl_SET((short) -30523);
        p234.altitude_sp_SET((short)14403);
        p234.airspeed_SET((char)165);
        p234.airspeed_sp_SET((char)240);
        p234.groundspeed_SET((char)254);
        p234.climb_rate_SET((byte) - 25);
        p234.gps_nsat_SET((char)163);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
        p234.battery_remaining_SET((char)199);
        p234.temperature_SET((byte)69);
        p234.temperature_air_SET((byte) - 87);
        p234.failsafe_SET((char)185);
        p234.wp_num_SET((char)105);
        p234.wp_distance_SET((char)44464);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(7091009440680891783L);
        p241.vibration_x_SET(1.3273168E38F);
        p241.vibration_y_SET(1.786427E38F);
        p241.vibration_z_SET(-4.3974363E37F);
        p241.clipping_0_SET(1413868141L);
        p241.clipping_1_SET(512280243L);
        p241.clipping_2_SET(2713408613L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(2126839091);
        p242.longitude_SET(-524149178);
        p242.altitude_SET(2072366438);
        p242.x_SET(2.8938709E38F);
        p242.y_SET(2.9348981E38F);
        p242.z_SET(1.788629E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(2.4459533E38F);
        p242.approach_y_SET(1.5377545E38F);
        p242.approach_z_SET(3.1437475E38F);
        p242.time_usec_SET(1414295216122354220L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)50);
        p243.latitude_SET(1093500308);
        p243.longitude_SET(-1203400265);
        p243.altitude_SET(1573683246);
        p243.x_SET(9.825786E37F);
        p243.y_SET(1.4810961E38F);
        p243.z_SET(2.2212772E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-2.4856734E38F);
        p243.approach_y_SET(2.9661235E38F);
        p243.approach_z_SET(9.423185E37F);
        p243.time_usec_SET(6555144999049456374L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)9919);
        p244.interval_us_SET(1439438113);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(2982170422L);
        p246.lat_SET(701528522);
        p246.lon_SET(1294514026);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-1544231005);
        p246.heading_SET((char)19316);
        p246.hor_velocity_SET((char)61598);
        p246.ver_velocity_SET((short)13806);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
        p246.tslc_SET((char)157);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
        p246.squawk_SET((char)34816);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(2281973253L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                               MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
        p247.time_to_minimum_delta_SET(2.2132516E38F);
        p247.altitude_minimum_delta_SET(1.5149752E38F);
        p247.horizontal_minimum_delta_SET(-2.872455E38F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)26);
        p248.target_system_SET((char)58);
        p248.target_component_SET((char)185);
        p248.message_type_SET((char)27211);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)1725);
        p249.ver_SET((char)85);
        p249.type_SET((char)193);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(3491860275091871289L);
        p250.x_SET(1.5513529E36F);
        p250.y_SET(3.0150321E38F);
        p250.z_SET(2.4604417E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3870521638L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-7.184175E37F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(43426388L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(487937246);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2025754251L);
        p254.ind_SET((char)53);
        p254.value_SET(5.3361695E36F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)165);
        p256.target_component_SET((char)106);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(5781137014779009120L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(4033427922L);
        p257.last_change_ms_SET(2032770503L);
        p257.state_SET((char)248);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)224);
        p258.target_component_SET((char)142);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(3852238514L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(836978778L);
        p259.focal_length_SET(2.7447434E36F);
        p259.sensor_size_h_SET(-2.646906E38F);
        p259.sensor_size_v_SET(8.0571827E37F);
        p259.resolution_h_SET((char)43434);
        p259.resolution_v_SET((char)31950);
        p259.lens_id_SET((char)149);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
        p259.cam_definition_version_SET((char)45795);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1849851684L);
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_IMAGE));
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(1160002111L);
        p261.storage_id_SET((char)126);
        p261.storage_count_SET((char)27);
        p261.status_SET((char)14);
        p261.total_capacity_SET(-2.627727E38F);
        p261.used_capacity_SET(-1.1087079E38F);
        p261.available_capacity_SET(9.996294E37F);
        p261.read_speed_SET(-1.59793E38F);
        p261.write_speed_SET(-1.3999524E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2049972100L);
        p262.image_status_SET((char)58);
        p262.video_status_SET((char)90);
        p262.image_interval_SET(-3.1945158E38F);
        p262.recording_time_ms_SET(936073697L);
        p262.available_capacity_SET(2.7503274E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(723627977L);
        p263.time_utc_SET(6023084618333896180L);
        p263.camera_id_SET((char)134);
        p263.lat_SET(-1728423526);
        p263.lon_SET(979177887);
        p263.alt_SET(-780775548);
        p263.relative_alt_SET(1221413816);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-1205767954);
        p263.capture_result_SET((byte) - 52);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3188956733L);
        p264.arming_time_utc_SET(673121133282284234L);
        p264.takeoff_time_utc_SET(3847957753430808620L);
        p264.flight_uuid_SET(8619443659929889313L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(2976284630L);
        p265.roll_SET(-2.1991362E37F);
        p265.pitch_SET(-2.643783E38F);
        p265.yaw_SET(-1.8870431E37F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)71);
        p266.target_component_SET((char)136);
        p266.sequence_SET((char)7499);
        p266.length_SET((char)231);
        p266.first_message_offset_SET((char)105);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)15);
        p267.target_component_SET((char)56);
        p267.sequence_SET((char)32539);
        p267.length_SET((char)136);
        p267.first_message_offset_SET((char)32);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)5);
        p268.target_component_SET((char)49);
        p268.sequence_SET((char)39611);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)89);
        p269.status_SET((char)51);
        p269.framerate_SET(1.9669297E38F);
        p269.resolution_h_SET((char)27575);
        p269.resolution_v_SET((char)14505);
        p269.bitrate_SET(2786508120L);
        p269.rotation_SET((char)4732);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)6);
        p270.target_component_SET((char)172);
        p270.camera_id_SET((char)31);
        p270.framerate_SET(-2.7643707E38F);
        p270.resolution_h_SET((char)46164);
        p270.resolution_v_SET((char)32422);
        p270.bitrate_SET(940070119L);
        p270.rotation_SET((char)12810);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)18857);
        p300.min_version_SET((char)53617);
        p300.max_version_SET((char)5690);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(5130004440834521067L);
        p310.uptime_sec_SET(723659866L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        p310.sub_mode_SET((char)247);
        p310.vendor_specific_status_code_SET((char)35712);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(5037395558814154786L);
        p311.uptime_sec_SET(4283113942L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)116);
        p311.hw_version_minor_SET((char)61);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)68);
        p311.sw_version_minor_SET((char)116);
        p311.sw_vcs_commit_SET(2638270850L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)125);
        p320.target_component_SET((char)205);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)16687);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)125);
        p321.target_component_SET((char)25);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p322.param_count_SET((char)14600);
        p322.param_index_SET((char)44644);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)53);
        p323.target_component_SET((char)105);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(2582450383191358807L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)235);
        p330.min_distance_SET((char)19224);
        p330.max_distance_SET((char)64721);
        CommunicationChannel.instance.send(p330); //===============================
    }
}

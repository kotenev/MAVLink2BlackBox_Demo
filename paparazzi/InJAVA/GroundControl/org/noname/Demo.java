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
        POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(2977384800L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        p3.type_mask_SET((char)35704);
        p3.x_SET(3.060417E38F);
        p3.y_SET(-3.8669065E37F);
        p3.z_SET(1.9578571E38F);
        p3.vx_SET(-1.4199029E37F);
        p3.vy_SET(3.376157E38F);
        p3.vz_SET(1.4558012E38F);
        p3.afx_SET(3.0851306E38F);
        p3.afy_SET(1.4552648E38F);
        p3.afz_SET(-1.1184392E38F);
        p3.yaw_SET(1.716608E38F);
        p3.yaw_rate_SET(1.3043615E38F);
        CommunicationChannel.instance.send(p3); //===============================
        COMMAND_LONG p76 = CommunicationChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)92);
        p76.target_component_SET((char)150);
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION);
        p76.confirmation_SET((char)32);
        p76.param1_SET(7.4730726E37F);
        p76.param2_SET(-3.2235075E38F);
        p76.param3_SET(-1.3242195E38F);
        p76.param4_SET(2.8305985E38F);
        p76.param5_SET(8.945737E37F);
        p76.param6_SET(-2.549798E38F);
        p76.param7_SET(-2.9853695E38F);
        CommunicationChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = CommunicationChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_LOGGING_START);
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED);
        p77.progress_SET((char)134, PH);
        p77.result_param2_SET(657177811, PH);
        p77.target_system_SET((char)140, PH);
        p77.target_component_SET((char)25, PH);
        CommunicationChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = CommunicationChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(4107336960L);
        p81.roll_SET(-8.533306E36F);
        p81.pitch_SET(-2.2889914E38F);
        p81.yaw_SET(-2.0627182E38F);
        p81.thrust_SET(-2.204071E37F);
        p81.mode_switch_SET((char)199);
        p81.manual_override_switch_SET((char)192);
        CommunicationChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3022889626L);
        p82.target_system_SET((char)33);
        p82.target_component_SET((char)9);
        p82.type_mask_SET((char)3);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(2.146787E38F);
        p82.body_pitch_rate_SET(-1.8321612E38F);
        p82.body_yaw_rate_SET(-1.692625E38F);
        p82.thrust_SET(-2.8565454E38F);
        CommunicationChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(2917393850L);
        p83.type_mask_SET((char)35);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(-8.2618813E37F);
        p83.body_pitch_rate_SET(-7.0365926E37F);
        p83.body_yaw_rate_SET(1.5780795E38F);
        p83.thrust_SET(1.3534536E38F);
        CommunicationChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(1691024927L);
        p84.target_system_SET((char)133);
        p84.target_component_SET((char)183);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p84.type_mask_SET((char)46343);
        p84.x_SET(2.161193E38F);
        p84.y_SET(2.151126E38F);
        p84.z_SET(-4.8928514E37F);
        p84.vx_SET(-9.15712E37F);
        p84.vy_SET(-1.4714163E38F);
        p84.vz_SET(2.5589385E38F);
        p84.afx_SET(-1.2516014E38F);
        p84.afy_SET(-1.848761E38F);
        p84.afz_SET(-2.9107394E38F);
        p84.yaw_SET(2.6501244E38F);
        p84.yaw_rate_SET(-7.207425E37F);
        CommunicationChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(3945371827L);
        p86.target_system_SET((char)27);
        p86.target_component_SET((char)89);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p86.type_mask_SET((char)37509);
        p86.lat_int_SET(-1013832556);
        p86.lon_int_SET(1702610284);
        p86.alt_SET(1.876462E38F);
        p86.vx_SET(2.9795872E38F);
        p86.vy_SET(-2.2431035E38F);
        p86.vz_SET(-7.6613613E37F);
        p86.afx_SET(1.7214914E38F);
        p86.afy_SET(1.451751E38F);
        p86.afz_SET(4.297065E37F);
        p86.yaw_SET(-2.873559E38F);
        p86.yaw_rate_SET(-1.1995102E38F);
        CommunicationChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(2942068396L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p87.type_mask_SET((char)42038);
        p87.lat_int_SET(-594008739);
        p87.lon_int_SET(149093532);
        p87.alt_SET(-2.7627118E38F);
        p87.vx_SET(-3.3711872E38F);
        p87.vy_SET(-4.355252E37F);
        p87.vz_SET(-2.3650963E38F);
        p87.afx_SET(-3.351996E38F);
        p87.afy_SET(-2.129951E38F);
        p87.afz_SET(-8.468407E36F);
        p87.yaw_SET(-3.0226157E38F);
        p87.yaw_rate_SET(1.5438041E38F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(2325296099L);
        p89.x_SET(-2.4688128E38F);
        p89.y_SET(1.9369064E38F);
        p89.z_SET(3.4836968E37F);
        p89.roll_SET(2.6605065E38F);
        p89.pitch_SET(2.8261852E38F);
        p89.yaw_SET(-1.7810527E38F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(4415774558212729766L);
        p90.roll_SET(-1.3920733E37F);
        p90.pitch_SET(-2.6463748E38F);
        p90.yaw_SET(-2.653398E38F);
        p90.rollspeed_SET(-2.484547E37F);
        p90.pitchspeed_SET(-3.0602647E38F);
        p90.yawspeed_SET(1.840804E38F);
        p90.lat_SET(-1873222921);
        p90.lon_SET(402149286);
        p90.alt_SET(-162531294);
        p90.vx_SET((short)30203);
        p90.vy_SET((short)17216);
        p90.vz_SET((short)30978);
        p90.xacc_SET((short) -16229);
        p90.yacc_SET((short) -15551);
        p90.zacc_SET((short)31117);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(2764625790300886492L);
        p91.roll_ailerons_SET(2.1271958E37F);
        p91.pitch_elevator_SET(-8.729608E37F);
        p91.yaw_rudder_SET(-2.4943731E38F);
        p91.throttle_SET(-3.4448124E37F);
        p91.aux1_SET(-2.8619886E38F);
        p91.aux2_SET(-3.241078E37F);
        p91.aux3_SET(3.3967815E38F);
        p91.aux4_SET(3.844495E37F);
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        p91.nav_mode_SET((char)203);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(4960750528000993194L);
        p92.chan1_raw_SET((char)16078);
        p92.chan2_raw_SET((char)52330);
        p92.chan3_raw_SET((char)30158);
        p92.chan4_raw_SET((char)25641);
        p92.chan5_raw_SET((char)55588);
        p92.chan6_raw_SET((char)1884);
        p92.chan7_raw_SET((char)24739);
        p92.chan8_raw_SET((char)94);
        p92.chan9_raw_SET((char)22781);
        p92.chan10_raw_SET((char)9388);
        p92.chan11_raw_SET((char)44471);
        p92.chan12_raw_SET((char)44771);
        p92.rssi_SET((char)49);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(3981123212139410873L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
        p93.flags_SET(6416516524908644986L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(4259951156910738392L);
        p100.sensor_id_SET((char)163);
        p100.flow_x_SET((short) -18909);
        p100.flow_y_SET((short) -13210);
        p100.flow_comp_m_x_SET(1.1575795E38F);
        p100.flow_comp_m_y_SET(1.44156E38F);
        p100.quality_SET((char)99);
        p100.ground_distance_SET(5.5685274E37F);
        p100.flow_rate_x_SET(-1.5181134E38F, PH);
        p100.flow_rate_y_SET(1.4462943E38F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(1882291259110467066L);
        p101.x_SET(1.049897E38F);
        p101.y_SET(4.2422313E37F);
        p101.z_SET(-4.3030897E37F);
        p101.roll_SET(2.8912881E38F);
        p101.pitch_SET(3.2343063E37F);
        p101.yaw_SET(1.6966111E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(467669190508619643L);
        p102.x_SET(3.037395E38F);
        p102.y_SET(-8.3726205E36F);
        p102.z_SET(-6.4339246E37F);
        p102.roll_SET(1.3458992E38F);
        p102.pitch_SET(-1.950063E38F);
        p102.yaw_SET(1.8746059E37F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(3452429364540375042L);
        p103.x_SET(-3.3059778E38F);
        p103.y_SET(-6.1450205E37F);
        p103.z_SET(1.9590651E37F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(20771201470883330L);
        p104.x_SET(-1.6189945E38F);
        p104.y_SET(-3.2642828E38F);
        p104.z_SET(-3.0232191E38F);
        p104.roll_SET(1.8010699E38F);
        p104.pitch_SET(-2.5355933E38F);
        p104.yaw_SET(-2.5593868E38F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(4606755827658367112L);
        p105.xacc_SET(-3.1868556E38F);
        p105.yacc_SET(-1.1393935E37F);
        p105.zacc_SET(-2.3759314E38F);
        p105.xgyro_SET(-2.8108722E38F);
        p105.ygyro_SET(-2.1548468E38F);
        p105.zgyro_SET(-7.666075E36F);
        p105.xmag_SET(7.282176E37F);
        p105.ymag_SET(-1.6577196E38F);
        p105.zmag_SET(1.1409863E38F);
        p105.abs_pressure_SET(1.2495242E38F);
        p105.diff_pressure_SET(2.8148955E37F);
        p105.pressure_alt_SET(-2.508756E38F);
        p105.temperature_SET(2.2780535E38F);
        p105.fields_updated_SET((char)10601);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(8029884257673746867L);
        p106.sensor_id_SET((char)109);
        p106.integration_time_us_SET(3665435613L);
        p106.integrated_x_SET(2.7798902E38F);
        p106.integrated_y_SET(2.337414E38F);
        p106.integrated_xgyro_SET(-3.3236805E38F);
        p106.integrated_ygyro_SET(3.3541418E38F);
        p106.integrated_zgyro_SET(-4.975501E37F);
        p106.temperature_SET((short) -16985);
        p106.quality_SET((char)56);
        p106.time_delta_distance_us_SET(3354326268L);
        p106.distance_SET(-7.8531974E37F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(8222202862176472861L);
        p107.xacc_SET(-1.4327796E37F);
        p107.yacc_SET(1.6861261E38F);
        p107.zacc_SET(-5.631965E37F);
        p107.xgyro_SET(-1.0936973E38F);
        p107.ygyro_SET(3.3327592E37F);
        p107.zgyro_SET(-2.3695562E38F);
        p107.xmag_SET(6.071147E36F);
        p107.ymag_SET(-8.3927577E37F);
        p107.zmag_SET(-7.958404E37F);
        p107.abs_pressure_SET(1.973129E38F);
        p107.diff_pressure_SET(-2.534822E38F);
        p107.pressure_alt_SET(-4.998694E37F);
        p107.temperature_SET(2.2751272E38F);
        p107.fields_updated_SET(2915287046L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(2.8153794E38F);
        p108.q2_SET(2.9122792E37F);
        p108.q3_SET(-2.0215484E38F);
        p108.q4_SET(-2.6442697E38F);
        p108.roll_SET(-9.238966E37F);
        p108.pitch_SET(2.516532E38F);
        p108.yaw_SET(-1.5040228E38F);
        p108.xacc_SET(-8.346481E37F);
        p108.yacc_SET(2.2768522E38F);
        p108.zacc_SET(-3.131455E38F);
        p108.xgyro_SET(-2.9341215E38F);
        p108.ygyro_SET(-2.30338E38F);
        p108.zgyro_SET(3.2170313E38F);
        p108.lat_SET(2.146913E38F);
        p108.lon_SET(3.231459E38F);
        p108.alt_SET(2.6409817E38F);
        p108.std_dev_horz_SET(1.946215E38F);
        p108.std_dev_vert_SET(-3.0977484E37F);
        p108.vn_SET(-2.8718724E38F);
        p108.ve_SET(1.2553026E38F);
        p108.vd_SET(-7.7307124E37F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)136);
        p109.remrssi_SET((char)221);
        p109.txbuf_SET((char)137);
        p109.noise_SET((char)133);
        p109.remnoise_SET((char)46);
        p109.rxerrors_SET((char)59879);
        p109.fixed__SET((char)26676);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)86);
        p110.target_system_SET((char)168);
        p110.target_component_SET((char)140);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-1660204006884968713L);
        p111.ts1_SET(8845825871640399055L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(7123261205290303123L);
        p112.seq_SET(3247976056L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(6819239568374024699L);
        p113.fix_type_SET((char)124);
        p113.lat_SET(1538222991);
        p113.lon_SET(-1088968500);
        p113.alt_SET(-443730437);
        p113.eph_SET((char)46344);
        p113.epv_SET((char)13291);
        p113.vel_SET((char)60286);
        p113.vn_SET((short)17864);
        p113.ve_SET((short) -3687);
        p113.vd_SET((short) -18984);
        p113.cog_SET((char)55244);
        p113.satellites_visible_SET((char)41);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(2007939311330929515L);
        p114.sensor_id_SET((char)236);
        p114.integration_time_us_SET(2502828150L);
        p114.integrated_x_SET(-1.0418526E38F);
        p114.integrated_y_SET(-2.319938E38F);
        p114.integrated_xgyro_SET(-1.427623E37F);
        p114.integrated_ygyro_SET(3.7987952E37F);
        p114.integrated_zgyro_SET(2.923667E38F);
        p114.temperature_SET((short)31753);
        p114.quality_SET((char)5);
        p114.time_delta_distance_us_SET(1502364812L);
        p114.distance_SET(2.623277E37F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(123746009427335161L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(1.4198072E38F);
        p115.pitchspeed_SET(1.0528516E38F);
        p115.yawspeed_SET(2.2578674E38F);
        p115.lat_SET(-1977936201);
        p115.lon_SET(875845619);
        p115.alt_SET(348841051);
        p115.vx_SET((short)25424);
        p115.vy_SET((short)9490);
        p115.vz_SET((short)12722);
        p115.ind_airspeed_SET((char)21269);
        p115.true_airspeed_SET((char)19252);
        p115.xacc_SET((short)20883);
        p115.yacc_SET((short) -6456);
        p115.zacc_SET((short) -27812);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2107279581L);
        p116.xacc_SET((short)14093);
        p116.yacc_SET((short) -14204);
        p116.zacc_SET((short)3083);
        p116.xgyro_SET((short) -24941);
        p116.ygyro_SET((short) -23287);
        p116.zgyro_SET((short)28940);
        p116.xmag_SET((short)17394);
        p116.ymag_SET((short)10870);
        p116.zmag_SET((short)17266);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)220);
        p117.target_component_SET((char)27);
        p117.start_SET((char)24657);
        p117.end_SET((char)45389);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)8051);
        p118.num_logs_SET((char)11933);
        p118.last_log_num_SET((char)52021);
        p118.time_utc_SET(113747728L);
        p118.size_SET(2272366824L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)89);
        p119.target_component_SET((char)32);
        p119.id_SET((char)55015);
        p119.ofs_SET(806093523L);
        p119.count_SET(810478403L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)40927);
        p120.ofs_SET(2158045748L);
        p120.count_SET((char)95);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)113);
        p121.target_component_SET((char)55);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)203);
        p122.target_component_SET((char)89);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)125);
        p123.target_component_SET((char)67);
        p123.len_SET((char)240);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(6793854003584238813L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p124.lat_SET(-1508026323);
        p124.lon_SET(-39235941);
        p124.alt_SET(618401038);
        p124.eph_SET((char)10941);
        p124.epv_SET((char)28300);
        p124.vel_SET((char)62819);
        p124.cog_SET((char)11262);
        p124.satellites_visible_SET((char)95);
        p124.dgps_numch_SET((char)172);
        p124.dgps_age_SET(399594304L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)29294);
        p125.Vservo_SET((char)65369);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
        p126.timeout_SET((char)9977);
        p126.baudrate_SET(203807896L);
        p126.count_SET((char)160);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2408448631L);
        p127.rtk_receiver_id_SET((char)18);
        p127.wn_SET((char)32012);
        p127.tow_SET(764461584L);
        p127.rtk_health_SET((char)84);
        p127.rtk_rate_SET((char)27);
        p127.nsats_SET((char)218);
        p127.baseline_coords_type_SET((char)109);
        p127.baseline_a_mm_SET(-195252939);
        p127.baseline_b_mm_SET(-1952943721);
        p127.baseline_c_mm_SET(1606523326);
        p127.accuracy_SET(2066035820L);
        p127.iar_num_hypotheses_SET(-2047099767);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(1656579485L);
        p128.rtk_receiver_id_SET((char)67);
        p128.wn_SET((char)23868);
        p128.tow_SET(105534931L);
        p128.rtk_health_SET((char)215);
        p128.rtk_rate_SET((char)58);
        p128.nsats_SET((char)138);
        p128.baseline_coords_type_SET((char)13);
        p128.baseline_a_mm_SET(-1703797136);
        p128.baseline_b_mm_SET(-820013837);
        p128.baseline_c_mm_SET(-1217358657);
        p128.accuracy_SET(3453089067L);
        p128.iar_num_hypotheses_SET(1231492083);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(2664474530L);
        p129.xacc_SET((short)30337);
        p129.yacc_SET((short) -18506);
        p129.zacc_SET((short)26001);
        p129.xgyro_SET((short)22692);
        p129.ygyro_SET((short) -22689);
        p129.zgyro_SET((short) -7914);
        p129.xmag_SET((short)22756);
        p129.ymag_SET((short)16210);
        p129.zmag_SET((short) -31673);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)231);
        p130.size_SET(1084330752L);
        p130.width_SET((char)31263);
        p130.height_SET((char)35659);
        p130.packets_SET((char)54241);
        p130.payload_SET((char)43);
        p130.jpg_quality_SET((char)159);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)2789);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(1934266951L);
        p132.min_distance_SET((char)56426);
        p132.max_distance_SET((char)33630);
        p132.current_distance_SET((char)1204);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p132.id_SET((char)87);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_315);
        p132.covariance_SET((char)167);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1193717692);
        p133.lon_SET(-1899408536);
        p133.grid_spacing_SET((char)3631);
        p133.mask_SET(7665230548557342462L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(82032220);
        p134.lon_SET(-1333704902);
        p134.grid_spacing_SET((char)5938);
        p134.gridbit_SET((char)70);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-2015936894);
        p135.lon_SET(132733867);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(2101568973);
        p136.lon_SET(-1864723074);
        p136.spacing_SET((char)3734);
        p136.terrain_height_SET(-5.159521E36F);
        p136.current_height_SET(1.7825707E38F);
        p136.pending_SET((char)47395);
        p136.loaded_SET((char)62323);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3448236724L);
        p137.press_abs_SET(2.121576E38F);
        p137.press_diff_SET(-3.882541E37F);
        p137.temperature_SET((short) -27079);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(2072337203656137357L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(3.227331E38F);
        p138.y_SET(2.9755088E38F);
        p138.z_SET(4.881587E37F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(7120050101550826118L);
        p139.group_mlx_SET((char)119);
        p139.target_system_SET((char)122);
        p139.target_component_SET((char)242);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(3468300586437439855L);
        p140.group_mlx_SET((char)53);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(8987870226281067393L);
        p141.altitude_monotonic_SET(3.2759387E38F);
        p141.altitude_amsl_SET(3.0562918E38F);
        p141.altitude_local_SET(2.2136836E38F);
        p141.altitude_relative_SET(2.6755472E38F);
        p141.altitude_terrain_SET(2.947339E38F);
        p141.bottom_clearance_SET(-9.423539E37F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)110);
        p142.uri_type_SET((char)168);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)173);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1148188147L);
        p143.press_abs_SET(2.2318425E38F);
        p143.press_diff_SET(3.2582423E38F);
        p143.temperature_SET((short)26799);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(7837146752282003685L);
        p144.est_capabilities_SET((char)12);
        p144.lat_SET(1780028326);
        p144.lon_SET(-1612467786);
        p144.alt_SET(2.4696831E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(4289403055549315192L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(2563744069681253469L);
        p146.x_acc_SET(2.4150794E38F);
        p146.y_acc_SET(-1.6457564E38F);
        p146.z_acc_SET(7.297164E37F);
        p146.x_vel_SET(1.4381024E37F);
        p146.y_vel_SET(2.1047499E38F);
        p146.z_vel_SET(2.7988512E38F);
        p146.x_pos_SET(-3.7060746E37F);
        p146.y_pos_SET(-2.7183608E37F);
        p146.z_pos_SET(1.8908554E38F);
        p146.airspeed_SET(2.5984073E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-2.7026877E38F);
        p146.pitch_rate_SET(1.9894939E38F);
        p146.yaw_rate_SET(8.523082E37F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)71);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
        p147.temperature_SET((short)5404);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -3116);
        p147.current_consumed_SET(1157789390);
        p147.energy_consumed_SET(-1971637747);
        p147.battery_remaining_SET((byte)32);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
        p148.flight_sw_version_SET(940530712L);
        p148.middleware_sw_version_SET(639241163L);
        p148.os_sw_version_SET(1920763313L);
        p148.board_version_SET(2703363549L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)40946);
        p148.product_id_SET((char)19901);
        p148.uid_SET(1135966074135064810L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(2408247343228251441L);
        p149.target_num_SET((char)185);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p149.angle_x_SET(-7.4774313E37F);
        p149.angle_y_SET(-5.221904E37F);
        p149.distance_SET(1.765697E38F);
        p149.size_x_SET(-2.733062E38F);
        p149.size_y_SET(-1.2977454E37F);
        p149.x_SET(9.21086E36F, PH);
        p149.y_SET(2.8337201E38F, PH);
        p149.z_SET(1.755197E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
        p149.position_valid_SET((char)39, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SCRIPT_ITEM p180 = CommunicationChannel.instance.new_SCRIPT_ITEM();
        PH.setPack(p180);
        p180.target_system_SET((char)141);
        p180.target_component_SET((char)209);
        p180.seq_SET((char)57039);
        p180.name_SET("DEMO", PH);
        CommunicationChannel.instance.send(p180); //===============================
        SCRIPT_REQUEST p181 = CommunicationChannel.instance.new_SCRIPT_REQUEST();
        PH.setPack(p181);
        p181.target_system_SET((char)37);
        p181.target_component_SET((char)204);
        p181.seq_SET((char)57252);
        CommunicationChannel.instance.send(p181); //===============================
        SCRIPT_REQUEST_LIST p182 = CommunicationChannel.instance.new_SCRIPT_REQUEST_LIST();
        PH.setPack(p182);
        p182.target_system_SET((char)9);
        p182.target_component_SET((char)167);
        CommunicationChannel.instance.send(p182); //===============================
        SCRIPT_COUNT p183 = CommunicationChannel.instance.new_SCRIPT_COUNT();
        PH.setPack(p183);
        p183.target_system_SET((char)250);
        p183.target_component_SET((char)250);
        p183.count_SET((char)3209);
        CommunicationChannel.instance.send(p183); //===============================
        SCRIPT_CURRENT p184 = CommunicationChannel.instance.new_SCRIPT_CURRENT();
        PH.setPack(p184);
        p184.seq_SET((char)2920);
        CommunicationChannel.instance.send(p184); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(2645587400022383264L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
        p230.vel_ratio_SET(1.6292095E38F);
        p230.pos_horiz_ratio_SET(-1.4988039E37F);
        p230.pos_vert_ratio_SET(-1.1504182E38F);
        p230.mag_ratio_SET(2.8752253E38F);
        p230.hagl_ratio_SET(5.478984E37F);
        p230.tas_ratio_SET(-9.817404E37F);
        p230.pos_horiz_accuracy_SET(3.1982314E38F);
        p230.pos_vert_accuracy_SET(2.7442741E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(7029826409724827662L);
        p231.wind_x_SET(2.4881286E38F);
        p231.wind_y_SET(-2.6757867E38F);
        p231.wind_z_SET(2.4005195E37F);
        p231.var_horiz_SET(-1.0664708E38F);
        p231.var_vert_SET(2.8074289E38F);
        p231.wind_alt_SET(2.1136867E38F);
        p231.horiz_accuracy_SET(3.239318E38F);
        p231.vert_accuracy_SET(-2.4535244E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(2392268104156385631L);
        p232.gps_id_SET((char)9);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
        p232.time_week_ms_SET(1927187550L);
        p232.time_week_SET((char)51184);
        p232.fix_type_SET((char)92);
        p232.lat_SET(102353880);
        p232.lon_SET(-513360933);
        p232.alt_SET(-1.002115E38F);
        p232.hdop_SET(1.5544967E38F);
        p232.vdop_SET(2.9073654E38F);
        p232.vn_SET(-3.2001393E38F);
        p232.ve_SET(1.4082469E38F);
        p232.vd_SET(1.8345243E38F);
        p232.speed_accuracy_SET(-3.2672183E38F);
        p232.horiz_accuracy_SET(-1.4294056E38F);
        p232.vert_accuracy_SET(-2.6444652E38F);
        p232.satellites_visible_SET((char)198);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)88);
        p233.len_SET((char)237);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        p234.custom_mode_SET(3702187492L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        p234.roll_SET((short)24435);
        p234.pitch_SET((short)5837);
        p234.heading_SET((char)59313);
        p234.throttle_SET((byte)80);
        p234.heading_sp_SET((short)21403);
        p234.latitude_SET(715121079);
        p234.longitude_SET(367306401);
        p234.altitude_amsl_SET((short) -24778);
        p234.altitude_sp_SET((short) -32033);
        p234.airspeed_SET((char)229);
        p234.airspeed_sp_SET((char)165);
        p234.groundspeed_SET((char)182);
        p234.climb_rate_SET((byte)97);
        p234.gps_nsat_SET((char)123);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
        p234.battery_remaining_SET((char)13);
        p234.temperature_SET((byte)52);
        p234.temperature_air_SET((byte)106);
        p234.failsafe_SET((char)148);
        p234.wp_num_SET((char)206);
        p234.wp_distance_SET((char)32281);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(4131529034654722081L);
        p241.vibration_x_SET(-2.6907253E38F);
        p241.vibration_y_SET(1.8463048E37F);
        p241.vibration_z_SET(-2.041334E38F);
        p241.clipping_0_SET(2282672100L);
        p241.clipping_1_SET(1009805635L);
        p241.clipping_2_SET(2507151813L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(477251798);
        p242.longitude_SET(-1293084105);
        p242.altitude_SET(1131817445);
        p242.x_SET(1.9628734E38F);
        p242.y_SET(-2.3993192E38F);
        p242.z_SET(9.055486E36F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(2.7838994E38F);
        p242.approach_y_SET(-3.7998172E37F);
        p242.approach_z_SET(2.19684E37F);
        p242.time_usec_SET(1318280493857107996L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)78);
        p243.latitude_SET(-1239538024);
        p243.longitude_SET(-1405323136);
        p243.altitude_SET(-913600709);
        p243.x_SET(3.2445568E37F);
        p243.y_SET(1.3251551E38F);
        p243.z_SET(2.9690393E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-1.3067572E37F);
        p243.approach_y_SET(3.3282599E38F);
        p243.approach_z_SET(2.3957244E38F);
        p243.time_usec_SET(6742465884162594842L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)26433);
        p244.interval_us_SET(-1558154831);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(846580623L);
        p246.lat_SET(1816025302);
        p246.lon_SET(-1844192987);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(-823188307);
        p246.heading_SET((char)34644);
        p246.hor_velocity_SET((char)25277);
        p246.ver_velocity_SET((short) -8137);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
        p246.tslc_SET((char)88);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
        p246.squawk_SET((char)27887);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(1108649192L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(6.5063176E37F);
        p247.altitude_minimum_delta_SET(3.3072826E38F);
        p247.horizontal_minimum_delta_SET(-4.0891792E37F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)223);
        p248.target_system_SET((char)31);
        p248.target_component_SET((char)166);
        p248.message_type_SET((char)43271);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)46028);
        p249.ver_SET((char)137);
        p249.type_SET((char)31);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(2207536019623293294L);
        p250.x_SET(-6.949039E37F);
        p250.y_SET(1.4743306E38F);
        p250.z_SET(2.694791E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2193286646L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-5.183989E37F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(191804048L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(1654809616);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ERROR);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(1622953524L);
        p254.ind_SET((char)54);
        p254.value_SET(-2.2391204E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)216);
        p256.target_component_SET((char)29);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(7145449348804445469L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(4057320793L);
        p257.last_change_ms_SET(1472628598L);
        p257.state_SET((char)38);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)2);
        p258.target_component_SET((char)184);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(2005216394L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(1948137481L);
        p259.focal_length_SET(3.1402125E38F);
        p259.sensor_size_h_SET(-2.3404653E38F);
        p259.sensor_size_v_SET(1.0071782E38F);
        p259.resolution_h_SET((char)40652);
        p259.resolution_v_SET((char)21265);
        p259.lens_id_SET((char)112);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
        p259.cam_definition_version_SET((char)3866);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2698053442L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3338844678L);
        p261.storage_id_SET((char)144);
        p261.storage_count_SET((char)42);
        p261.status_SET((char)128);
        p261.total_capacity_SET(1.2527676E38F);
        p261.used_capacity_SET(2.5887092E38F);
        p261.available_capacity_SET(-1.5441017E38F);
        p261.read_speed_SET(1.4854726E38F);
        p261.write_speed_SET(2.0156212E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(3331321650L);
        p262.image_status_SET((char)206);
        p262.video_status_SET((char)25);
        p262.image_interval_SET(1.2706704E36F);
        p262.recording_time_ms_SET(1964618594L);
        p262.available_capacity_SET(-3.6881914E36F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(192999735L);
        p263.time_utc_SET(5485049314527705467L);
        p263.camera_id_SET((char)241);
        p263.lat_SET(-1484057276);
        p263.lon_SET(1420845212);
        p263.alt_SET(1803610922);
        p263.relative_alt_SET(900558385);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(346688547);
        p263.capture_result_SET((byte)22);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(2460496572L);
        p264.arming_time_utc_SET(4338983147740742446L);
        p264.takeoff_time_utc_SET(3213817011160717832L);
        p264.flight_uuid_SET(1951157617965320417L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3142335756L);
        p265.roll_SET(2.8885977E38F);
        p265.pitch_SET(3.0870343E38F);
        p265.yaw_SET(-3.3900912E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)68);
        p266.target_component_SET((char)223);
        p266.sequence_SET((char)34668);
        p266.length_SET((char)193);
        p266.first_message_offset_SET((char)255);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)73);
        p267.target_component_SET((char)253);
        p267.sequence_SET((char)14870);
        p267.length_SET((char)242);
        p267.first_message_offset_SET((char)47);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)219);
        p268.target_component_SET((char)163);
        p268.sequence_SET((char)21362);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)177);
        p269.status_SET((char)113);
        p269.framerate_SET(1.4072175E38F);
        p269.resolution_h_SET((char)31789);
        p269.resolution_v_SET((char)37870);
        p269.bitrate_SET(2525234004L);
        p269.rotation_SET((char)58382);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)131);
        p270.target_component_SET((char)89);
        p270.camera_id_SET((char)70);
        p270.framerate_SET(2.2571111E38F);
        p270.resolution_h_SET((char)23341);
        p270.resolution_v_SET((char)39572);
        p270.bitrate_SET(1528577870L);
        p270.rotation_SET((char)52120);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)9226);
        p300.min_version_SET((char)17141);
        p300.max_version_SET((char)7390);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(1665889762800816916L);
        p310.uptime_sec_SET(4072787311L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        p310.sub_mode_SET((char)246);
        p310.vendor_specific_status_code_SET((char)17083);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(6239792346612541865L);
        p311.uptime_sec_SET(4028790060L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)168);
        p311.hw_version_minor_SET((char)193);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)151);
        p311.sw_version_minor_SET((char)92);
        p311.sw_vcs_commit_SET(3906531052L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)42);
        p320.target_component_SET((char)189);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)30355);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)251);
        p321.target_component_SET((char)206);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        p322.param_count_SET((char)15676);
        p322.param_index_SET((char)60321);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)46);
        p323.target_component_SET((char)139);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(8219778803911379280L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)33);
        p330.min_distance_SET((char)46209);
        p330.max_distance_SET((char)53234);
        CommunicationChannel.instance.send(p330); //===============================
    }
}

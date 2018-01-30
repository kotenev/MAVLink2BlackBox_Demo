
#include "DemoDevice.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_TYPE  type = p0_type_GET(pack);
    e_MAV_AUTOPILOT  autopilot = p0_autopilot_GET(pack);
    e_MAV_MODE_FLAG  base_mode = p0_base_mode_GET(pack);
    uint32_t  custom_mode = p0_custom_mode_GET(pack);
    e_MAV_STATE  system_status = p0_system_status_GET(pack);
    uint8_t  mavlink_version = p0_mavlink_version_GET(pack);
}
void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_present = p1_onboard_control_sensors_present_GET(pack);
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_enabled = p1_onboard_control_sensors_enabled_GET(pack);
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_health = p1_onboard_control_sensors_health_GET(pack);
    uint16_t  load = p1_load_GET(pack);
    uint16_t  voltage_battery = p1_voltage_battery_GET(pack);
    int16_t  current_battery = p1_current_battery_GET(pack);
    int8_t  battery_remaining = p1_battery_remaining_GET(pack);
    uint16_t  drop_rate_comm = p1_drop_rate_comm_GET(pack);
    uint16_t  errors_comm = p1_errors_comm_GET(pack);
    uint16_t  errors_count1 = p1_errors_count1_GET(pack);
    uint16_t  errors_count2 = p1_errors_count2_GET(pack);
    uint16_t  errors_count3 = p1_errors_count3_GET(pack);
    uint16_t  errors_count4 = p1_errors_count4_GET(pack);
}
void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_unix_usec = p2_time_unix_usec_GET(pack);
    uint32_t  time_boot_ms = p2_time_boot_ms_GET(pack);
}
void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p3_time_boot_ms_GET(pack);
    e_MAV_FRAME  coordinate_frame = p3_coordinate_frame_GET(pack);
    uint16_t  type_mask = p3_type_mask_GET(pack);
    float  x = p3_x_GET(pack);
    float  y = p3_y_GET(pack);
    float  z = p3_z_GET(pack);
    float  vx = p3_vx_GET(pack);
    float  vy = p3_vy_GET(pack);
    float  vz = p3_vz_GET(pack);
    float  afx = p3_afx_GET(pack);
    float  afy = p3_afy_GET(pack);
    float  afz = p3_afz_GET(pack);
    float  yaw = p3_yaw_GET(pack);
    float  yaw_rate = p3_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p4_time_usec_GET(pack);
    uint32_t  seq = p4_seq_GET(pack);
    uint8_t  target_system = p4_target_system_GET(pack);
    uint8_t  target_component = p4_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p5_target_system_GET(pack);
    uint8_t  control_request = p5_control_request_GET(pack);
    uint8_t  version = p5_version_GET(pack);
    char16_t *  passkey = p5_passkey_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  gcs_system_id = p6_gcs_system_id_GET(pack);
    uint8_t  control_request = p6_control_request_GET(pack);
    uint8_t  ack = p6_ack_GET(pack);
}
void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  key = p7_key_TRY_(ph);
}
void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11_target_system_GET(pack);
    e_MAV_MODE  base_mode = p11_base_mode_GET(pack);
    uint32_t  custom_mode = p11_custom_mode_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p20_target_system_GET(pack);
    uint8_t  target_component = p20_target_component_GET(pack);
    char16_t *  param_id = p20_param_id_TRY_(ph);
    int16_t  param_index = p20_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p21_target_system_GET(pack);
    uint8_t  target_component = p21_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p22_param_id_TRY_(ph);
    float  param_value = p22_param_value_GET(pack);
    e_MAV_PARAM_TYPE  param_type = p22_param_type_GET(pack);
    uint16_t  param_count = p22_param_count_GET(pack);
    uint16_t  param_index = p22_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p23_target_system_GET(pack);
    uint8_t  target_component = p23_target_component_GET(pack);
    char16_t *  param_id = p23_param_id_TRY_(ph);
    float  param_value = p23_param_value_GET(pack);
    e_MAV_PARAM_TYPE  param_type = p23_param_type_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p24_time_usec_GET(pack);
    e_GPS_FIX_TYPE  fix_type = p24_fix_type_GET(pack);
    int32_t  lat = p24_lat_GET(pack);
    int32_t  lon = p24_lon_GET(pack);
    int32_t  alt = p24_alt_GET(pack);
    uint16_t  eph = p24_eph_GET(pack);
    uint16_t  epv = p24_epv_GET(pack);
    uint16_t  vel = p24_vel_GET(pack);
    uint16_t  cog = p24_cog_GET(pack);
    uint8_t  satellites_visible = p24_satellites_visible_GET(pack);
    int32_t  alt_ellipsoid = p24_alt_ellipsoid_TRY(ph);
    uint32_t  h_acc = p24_h_acc_TRY(ph);
    uint32_t  v_acc = p24_v_acc_TRY(ph);
    uint32_t  vel_acc = p24_vel_acc_TRY(ph);
    uint32_t  hdg_acc = p24_hdg_acc_TRY(ph);
}
void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  satellites_visible = p25_satellites_visible_GET(pack);
    uint8_t*  satellite_prn = p25_satellite_prn_GET_(pack);
//process data in satellite_prn
    free(satellite_prn);//never forget to dispose
    uint8_t*  satellite_used = p25_satellite_used_GET_(pack);
//process data in satellite_used
    free(satellite_used);//never forget to dispose
    uint8_t*  satellite_elevation = p25_satellite_elevation_GET_(pack);
//process data in satellite_elevation
    free(satellite_elevation);//never forget to dispose
    uint8_t*  satellite_azimuth = p25_satellite_azimuth_GET_(pack);
//process data in satellite_azimuth
    free(satellite_azimuth);//never forget to dispose
    uint8_t*  satellite_snr = p25_satellite_snr_GET_(pack);
//process data in satellite_snr
    free(satellite_snr);//never forget to dispose
}
void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p26_time_boot_ms_GET(pack);
    int16_t  xacc = p26_xacc_GET(pack);
    int16_t  yacc = p26_yacc_GET(pack);
    int16_t  zacc = p26_zacc_GET(pack);
    int16_t  xgyro = p26_xgyro_GET(pack);
    int16_t  ygyro = p26_ygyro_GET(pack);
    int16_t  zgyro = p26_zgyro_GET(pack);
    int16_t  xmag = p26_xmag_GET(pack);
    int16_t  ymag = p26_ymag_GET(pack);
    int16_t  zmag = p26_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p27_time_usec_GET(pack);
    int16_t  xacc = p27_xacc_GET(pack);
    int16_t  yacc = p27_yacc_GET(pack);
    int16_t  zacc = p27_zacc_GET(pack);
    int16_t  xgyro = p27_xgyro_GET(pack);
    int16_t  ygyro = p27_ygyro_GET(pack);
    int16_t  zgyro = p27_zgyro_GET(pack);
    int16_t  xmag = p27_xmag_GET(pack);
    int16_t  ymag = p27_ymag_GET(pack);
    int16_t  zmag = p27_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p28_time_usec_GET(pack);
    int16_t  press_abs = p28_press_abs_GET(pack);
    int16_t  press_diff1 = p28_press_diff1_GET(pack);
    int16_t  press_diff2 = p28_press_diff2_GET(pack);
    int16_t  temperature = p28_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p29_time_boot_ms_GET(pack);
    float  press_abs = p29_press_abs_GET(pack);
    float  press_diff = p29_press_diff_GET(pack);
    int16_t  temperature = p29_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p30_time_boot_ms_GET(pack);
    float  roll = p30_roll_GET(pack);
    float  pitch = p30_pitch_GET(pack);
    float  yaw = p30_yaw_GET(pack);
    float  rollspeed = p30_rollspeed_GET(pack);
    float  pitchspeed = p30_pitchspeed_GET(pack);
    float  yawspeed = p30_yawspeed_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p31_time_boot_ms_GET(pack);
    float  q1 = p31_q1_GET(pack);
    float  q2 = p31_q2_GET(pack);
    float  q3 = p31_q3_GET(pack);
    float  q4 = p31_q4_GET(pack);
    float  rollspeed = p31_rollspeed_GET(pack);
    float  pitchspeed = p31_pitchspeed_GET(pack);
    float  yawspeed = p31_yawspeed_GET(pack);
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p32_time_boot_ms_GET(pack);
    float  x = p32_x_GET(pack);
    float  y = p32_y_GET(pack);
    float  z = p32_z_GET(pack);
    float  vx = p32_vx_GET(pack);
    float  vy = p32_vy_GET(pack);
    float  vz = p32_vz_GET(pack);
}
void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p33_time_boot_ms_GET(pack);
    int32_t  lat = p33_lat_GET(pack);
    int32_t  lon = p33_lon_GET(pack);
    int32_t  alt = p33_alt_GET(pack);
    int32_t  relative_alt = p33_relative_alt_GET(pack);
    int16_t  vx = p33_vx_GET(pack);
    int16_t  vy = p33_vy_GET(pack);
    int16_t  vz = p33_vz_GET(pack);
    uint16_t  hdg = p33_hdg_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p34_time_boot_ms_GET(pack);
    uint8_t  port = p34_port_GET(pack);
    int16_t  chan1_scaled = p34_chan1_scaled_GET(pack);
    int16_t  chan2_scaled = p34_chan2_scaled_GET(pack);
    int16_t  chan3_scaled = p34_chan3_scaled_GET(pack);
    int16_t  chan4_scaled = p34_chan4_scaled_GET(pack);
    int16_t  chan5_scaled = p34_chan5_scaled_GET(pack);
    int16_t  chan6_scaled = p34_chan6_scaled_GET(pack);
    int16_t  chan7_scaled = p34_chan7_scaled_GET(pack);
    int16_t  chan8_scaled = p34_chan8_scaled_GET(pack);
    uint8_t  rssi = p34_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p35_time_boot_ms_GET(pack);
    uint8_t  port = p35_port_GET(pack);
    uint16_t  chan1_raw = p35_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p35_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p35_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p35_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p35_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p35_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p35_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p35_chan8_raw_GET(pack);
    uint8_t  rssi = p35_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_usec = p36_time_usec_GET(pack);
    uint8_t  port = p36_port_GET(pack);
    uint16_t  servo1_raw = p36_servo1_raw_GET(pack);
    uint16_t  servo2_raw = p36_servo2_raw_GET(pack);
    uint16_t  servo3_raw = p36_servo3_raw_GET(pack);
    uint16_t  servo4_raw = p36_servo4_raw_GET(pack);
    uint16_t  servo5_raw = p36_servo5_raw_GET(pack);
    uint16_t  servo6_raw = p36_servo6_raw_GET(pack);
    uint16_t  servo7_raw = p36_servo7_raw_GET(pack);
    uint16_t  servo8_raw = p36_servo8_raw_GET(pack);
    uint16_t  servo9_raw = p36_servo9_raw_TRY(ph);
    uint16_t  servo10_raw = p36_servo10_raw_TRY(ph);
    uint16_t  servo11_raw = p36_servo11_raw_TRY(ph);
    uint16_t  servo12_raw = p36_servo12_raw_TRY(ph);
    uint16_t  servo13_raw = p36_servo13_raw_TRY(ph);
    uint16_t  servo14_raw = p36_servo14_raw_TRY(ph);
    uint16_t  servo15_raw = p36_servo15_raw_TRY(ph);
    uint16_t  servo16_raw = p36_servo16_raw_TRY(ph);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p37_target_system_GET(pack);
    uint8_t  target_component = p37_target_component_GET(pack);
    int16_t  start_index = p37_start_index_GET(pack);
    int16_t  end_index = p37_end_index_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p37_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p38_target_system_GET(pack);
    uint8_t  target_component = p38_target_component_GET(pack);
    int16_t  start_index = p38_start_index_GET(pack);
    int16_t  end_index = p38_end_index_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p38_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p39_target_system_GET(pack);
    uint8_t  target_component = p39_target_component_GET(pack);
    uint16_t  seq = p39_seq_GET(pack);
    e_MAV_FRAME  frame = p39_frame_GET(pack);
    e_MAV_CMD  command = p39_command_GET(pack);
    uint8_t  current = p39_current_GET(pack);
    uint8_t  autocontinue = p39_autocontinue_GET(pack);
    float  param1 = p39_param1_GET(pack);
    float  param2 = p39_param2_GET(pack);
    float  param3 = p39_param3_GET(pack);
    float  param4 = p39_param4_GET(pack);
    float  x = p39_x_GET(pack);
    float  y = p39_y_GET(pack);
    float  z = p39_z_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p39_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p40_target_system_GET(pack);
    uint8_t  target_component = p40_target_component_GET(pack);
    uint16_t  seq = p40_seq_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p40_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p41_target_system_GET(pack);
    uint8_t  target_component = p41_target_component_GET(pack);
    uint16_t  seq = p41_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seq = p42_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p43_target_system_GET(pack);
    uint8_t  target_component = p43_target_component_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p43_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p44_target_system_GET(pack);
    uint8_t  target_component = p44_target_component_GET(pack);
    uint16_t  count = p44_count_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p44_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p45_target_system_GET(pack);
    uint8_t  target_component = p45_target_component_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p45_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seq = p46_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p47_target_system_GET(pack);
    uint8_t  target_component = p47_target_component_GET(pack);
    e_MAV_MISSION_RESULT  type = p47_type_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p47_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p48_target_system_GET(pack);
    int32_t  latitude = p48_latitude_GET(pack);
    int32_t  longitude = p48_longitude_GET(pack);
    int32_t  altitude = p48_altitude_GET(pack);
    uint64_t  time_usec = p48_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    int32_t  latitude = p49_latitude_GET(pack);
    int32_t  longitude = p49_longitude_GET(pack);
    int32_t  altitude = p49_altitude_GET(pack);
    uint64_t  time_usec = p49_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p50_target_system_GET(pack);
    uint8_t  target_component = p50_target_component_GET(pack);
    char16_t *  param_id = p50_param_id_TRY_(ph);
    int16_t  param_index = p50_param_index_GET(pack);
    uint8_t  parameter_rc_channel_index = p50_parameter_rc_channel_index_GET(pack);
    float  param_value0 = p50_param_value0_GET(pack);
    float  scale = p50_scale_GET(pack);
    float  param_value_min = p50_param_value_min_GET(pack);
    float  param_value_max = p50_param_value_max_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p51_target_system_GET(pack);
    uint8_t  target_component = p51_target_component_GET(pack);
    uint16_t  seq = p51_seq_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p51_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p54_target_system_GET(pack);
    uint8_t  target_component = p54_target_component_GET(pack);
    e_MAV_FRAME  frame = p54_frame_GET(pack);
    float  p1x = p54_p1x_GET(pack);
    float  p1y = p54_p1y_GET(pack);
    float  p1z = p54_p1z_GET(pack);
    float  p2x = p54_p2x_GET(pack);
    float  p2y = p54_p2y_GET(pack);
    float  p2z = p54_p2z_GET(pack);
}
void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_FRAME  frame = p55_frame_GET(pack);
    float  p1x = p55_p1x_GET(pack);
    float  p1y = p55_p1y_GET(pack);
    float  p1z = p55_p1z_GET(pack);
    float  p2x = p55_p2x_GET(pack);
    float  p2y = p55_p2y_GET(pack);
    float  p2z = p55_p2z_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p61_time_usec_GET(pack);
    float*  q = p61_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  rollspeed = p61_rollspeed_GET(pack);
    float  pitchspeed = p61_pitchspeed_GET(pack);
    float  yawspeed = p61_yawspeed_GET(pack);
    float*  covariance = p61_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    float  nav_roll = p62_nav_roll_GET(pack);
    float  nav_pitch = p62_nav_pitch_GET(pack);
    int16_t  nav_bearing = p62_nav_bearing_GET(pack);
    int16_t  target_bearing = p62_target_bearing_GET(pack);
    uint16_t  wp_dist = p62_wp_dist_GET(pack);
    float  alt_error = p62_alt_error_GET(pack);
    float  aspd_error = p62_aspd_error_GET(pack);
    float  xtrack_error = p62_xtrack_error_GET(pack);
}
void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p63_time_usec_GET(pack);
    e_MAV_ESTIMATOR_TYPE  estimator_type = p63_estimator_type_GET(pack);
    int32_t  lat = p63_lat_GET(pack);
    int32_t  lon = p63_lon_GET(pack);
    int32_t  alt = p63_alt_GET(pack);
    int32_t  relative_alt = p63_relative_alt_GET(pack);
    float  vx = p63_vx_GET(pack);
    float  vy = p63_vy_GET(pack);
    float  vz = p63_vz_GET(pack);
    float*  covariance = p63_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p64_time_usec_GET(pack);
    e_MAV_ESTIMATOR_TYPE  estimator_type = p64_estimator_type_GET(pack);
    float  x = p64_x_GET(pack);
    float  y = p64_y_GET(pack);
    float  z = p64_z_GET(pack);
    float  vx = p64_vx_GET(pack);
    float  vy = p64_vy_GET(pack);
    float  vz = p64_vz_GET(pack);
    float  ax = p64_ax_GET(pack);
    float  ay = p64_ay_GET(pack);
    float  az = p64_az_GET(pack);
    float*  covariance = p64_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p65_time_boot_ms_GET(pack);
    uint8_t  chancount = p65_chancount_GET(pack);
    uint16_t  chan1_raw = p65_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p65_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p65_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p65_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p65_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p65_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p65_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p65_chan8_raw_GET(pack);
    uint16_t  chan9_raw = p65_chan9_raw_GET(pack);
    uint16_t  chan10_raw = p65_chan10_raw_GET(pack);
    uint16_t  chan11_raw = p65_chan11_raw_GET(pack);
    uint16_t  chan12_raw = p65_chan12_raw_GET(pack);
    uint16_t  chan13_raw = p65_chan13_raw_GET(pack);
    uint16_t  chan14_raw = p65_chan14_raw_GET(pack);
    uint16_t  chan15_raw = p65_chan15_raw_GET(pack);
    uint16_t  chan16_raw = p65_chan16_raw_GET(pack);
    uint16_t  chan17_raw = p65_chan17_raw_GET(pack);
    uint16_t  chan18_raw = p65_chan18_raw_GET(pack);
    uint8_t  rssi = p65_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p66_target_system_GET(pack);
    uint8_t  target_component = p66_target_component_GET(pack);
    uint8_t  req_stream_id = p66_req_stream_id_GET(pack);
    uint16_t  req_message_rate = p66_req_message_rate_GET(pack);
    uint8_t  start_stop = p66_start_stop_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  stream_id = p67_stream_id_GET(pack);
    uint16_t  message_rate = p67_message_rate_GET(pack);
    uint8_t  on_off = p67_on_off_GET(pack);
}
void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p69_target_GET(pack);
    int16_t  x = p69_x_GET(pack);
    int16_t  y = p69_y_GET(pack);
    int16_t  z = p69_z_GET(pack);
    int16_t  r = p69_r_GET(pack);
    uint16_t  buttons = p69_buttons_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p70_target_system_GET(pack);
    uint8_t  target_component = p70_target_component_GET(pack);
    uint16_t  chan1_raw = p70_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p70_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p70_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p70_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p70_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p70_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p70_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p70_chan8_raw_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p73_target_system_GET(pack);
    uint8_t  target_component = p73_target_component_GET(pack);
    uint16_t  seq = p73_seq_GET(pack);
    e_MAV_FRAME  frame = p73_frame_GET(pack);
    e_MAV_CMD  command = p73_command_GET(pack);
    uint8_t  current = p73_current_GET(pack);
    uint8_t  autocontinue = p73_autocontinue_GET(pack);
    float  param1 = p73_param1_GET(pack);
    float  param2 = p73_param2_GET(pack);
    float  param3 = p73_param3_GET(pack);
    float  param4 = p73_param4_GET(pack);
    int32_t  x = p73_x_GET(pack);
    int32_t  y = p73_y_GET(pack);
    float  z = p73_z_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p73_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    float  airspeed = p74_airspeed_GET(pack);
    float  groundspeed = p74_groundspeed_GET(pack);
    int16_t  heading = p74_heading_GET(pack);
    uint16_t  throttle = p74_throttle_GET(pack);
    float  alt = p74_alt_GET(pack);
    float  climb = p74_climb_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p75_target_system_GET(pack);
    uint8_t  target_component = p75_target_component_GET(pack);
    e_MAV_FRAME  frame = p75_frame_GET(pack);
    e_MAV_CMD  command = p75_command_GET(pack);
    uint8_t  current = p75_current_GET(pack);
    uint8_t  autocontinue = p75_autocontinue_GET(pack);
    float  param1 = p75_param1_GET(pack);
    float  param2 = p75_param2_GET(pack);
    float  param3 = p75_param3_GET(pack);
    float  param4 = p75_param4_GET(pack);
    int32_t  x = p75_x_GET(pack);
    int32_t  y = p75_y_GET(pack);
    float  z = p75_z_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p76_target_system_GET(pack);
    uint8_t  target_component = p76_target_component_GET(pack);
    e_MAV_CMD  command = p76_command_GET(pack);
    uint8_t  confirmation = p76_confirmation_GET(pack);
    float  param1 = p76_param1_GET(pack);
    float  param2 = p76_param2_GET(pack);
    float  param3 = p76_param3_GET(pack);
    float  param4 = p76_param4_GET(pack);
    float  param5 = p76_param5_GET(pack);
    float  param6 = p76_param6_GET(pack);
    float  param7 = p76_param7_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_CMD  command = p77_command_GET(pack);
    e_MAV_RESULT  result = p77_result_GET(pack);
    uint8_t  progress = p77_progress_TRY(ph);
    int32_t  result_param2 = p77_result_param2_TRY(ph);
    uint8_t  target_system = p77_target_system_TRY(ph);
    uint8_t  target_component = p77_target_component_TRY(ph);
}
void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p81_time_boot_ms_GET(pack);
    float  roll = p81_roll_GET(pack);
    float  pitch = p81_pitch_GET(pack);
    float  yaw = p81_yaw_GET(pack);
    float  thrust = p81_thrust_GET(pack);
    uint8_t  mode_switch = p81_mode_switch_GET(pack);
    uint8_t  manual_override_switch = p81_manual_override_switch_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p82_time_boot_ms_GET(pack);
    uint8_t  target_system = p82_target_system_GET(pack);
    uint8_t  target_component = p82_target_component_GET(pack);
    uint8_t  type_mask = p82_type_mask_GET(pack);
    float*  q = p82_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  body_roll_rate = p82_body_roll_rate_GET(pack);
    float  body_pitch_rate = p82_body_pitch_rate_GET(pack);
    float  body_yaw_rate = p82_body_yaw_rate_GET(pack);
    float  thrust = p82_thrust_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p83_time_boot_ms_GET(pack);
    uint8_t  type_mask = p83_type_mask_GET(pack);
    float*  q = p83_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  body_roll_rate = p83_body_roll_rate_GET(pack);
    float  body_pitch_rate = p83_body_pitch_rate_GET(pack);
    float  body_yaw_rate = p83_body_yaw_rate_GET(pack);
    float  thrust = p83_thrust_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p84_time_boot_ms_GET(pack);
    uint8_t  target_system = p84_target_system_GET(pack);
    uint8_t  target_component = p84_target_component_GET(pack);
    e_MAV_FRAME  coordinate_frame = p84_coordinate_frame_GET(pack);
    uint16_t  type_mask = p84_type_mask_GET(pack);
    float  x = p84_x_GET(pack);
    float  y = p84_y_GET(pack);
    float  z = p84_z_GET(pack);
    float  vx = p84_vx_GET(pack);
    float  vy = p84_vy_GET(pack);
    float  vz = p84_vz_GET(pack);
    float  afx = p84_afx_GET(pack);
    float  afy = p84_afy_GET(pack);
    float  afz = p84_afz_GET(pack);
    float  yaw = p84_yaw_GET(pack);
    float  yaw_rate = p84_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p86_time_boot_ms_GET(pack);
    uint8_t  target_system = p86_target_system_GET(pack);
    uint8_t  target_component = p86_target_component_GET(pack);
    e_MAV_FRAME  coordinate_frame = p86_coordinate_frame_GET(pack);
    uint16_t  type_mask = p86_type_mask_GET(pack);
    int32_t  lat_int = p86_lat_int_GET(pack);
    int32_t  lon_int = p86_lon_int_GET(pack);
    float  alt = p86_alt_GET(pack);
    float  vx = p86_vx_GET(pack);
    float  vy = p86_vy_GET(pack);
    float  vz = p86_vz_GET(pack);
    float  afx = p86_afx_GET(pack);
    float  afy = p86_afy_GET(pack);
    float  afz = p86_afz_GET(pack);
    float  yaw = p86_yaw_GET(pack);
    float  yaw_rate = p86_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p87_time_boot_ms_GET(pack);
    e_MAV_FRAME  coordinate_frame = p87_coordinate_frame_GET(pack);
    uint16_t  type_mask = p87_type_mask_GET(pack);
    int32_t  lat_int = p87_lat_int_GET(pack);
    int32_t  lon_int = p87_lon_int_GET(pack);
    float  alt = p87_alt_GET(pack);
    float  vx = p87_vx_GET(pack);
    float  vy = p87_vy_GET(pack);
    float  vz = p87_vz_GET(pack);
    float  afx = p87_afx_GET(pack);
    float  afy = p87_afy_GET(pack);
    float  afz = p87_afz_GET(pack);
    float  yaw = p87_yaw_GET(pack);
    float  yaw_rate = p87_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p89_time_boot_ms_GET(pack);
    float  x = p89_x_GET(pack);
    float  y = p89_y_GET(pack);
    float  z = p89_z_GET(pack);
    float  roll = p89_roll_GET(pack);
    float  pitch = p89_pitch_GET(pack);
    float  yaw = p89_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p90_time_usec_GET(pack);
    float  roll = p90_roll_GET(pack);
    float  pitch = p90_pitch_GET(pack);
    float  yaw = p90_yaw_GET(pack);
    float  rollspeed = p90_rollspeed_GET(pack);
    float  pitchspeed = p90_pitchspeed_GET(pack);
    float  yawspeed = p90_yawspeed_GET(pack);
    int32_t  lat = p90_lat_GET(pack);
    int32_t  lon = p90_lon_GET(pack);
    int32_t  alt = p90_alt_GET(pack);
    int16_t  vx = p90_vx_GET(pack);
    int16_t  vy = p90_vy_GET(pack);
    int16_t  vz = p90_vz_GET(pack);
    int16_t  xacc = p90_xacc_GET(pack);
    int16_t  yacc = p90_yacc_GET(pack);
    int16_t  zacc = p90_zacc_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p91_time_usec_GET(pack);
    float  roll_ailerons = p91_roll_ailerons_GET(pack);
    float  pitch_elevator = p91_pitch_elevator_GET(pack);
    float  yaw_rudder = p91_yaw_rudder_GET(pack);
    float  throttle = p91_throttle_GET(pack);
    float  aux1 = p91_aux1_GET(pack);
    float  aux2 = p91_aux2_GET(pack);
    float  aux3 = p91_aux3_GET(pack);
    float  aux4 = p91_aux4_GET(pack);
    e_MAV_MODE  mode = p91_mode_GET(pack);
    uint8_t  nav_mode = p91_nav_mode_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p92_time_usec_GET(pack);
    uint16_t  chan1_raw = p92_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p92_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p92_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p92_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p92_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p92_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p92_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p92_chan8_raw_GET(pack);
    uint16_t  chan9_raw = p92_chan9_raw_GET(pack);
    uint16_t  chan10_raw = p92_chan10_raw_GET(pack);
    uint16_t  chan11_raw = p92_chan11_raw_GET(pack);
    uint16_t  chan12_raw = p92_chan12_raw_GET(pack);
    uint8_t  rssi = p92_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p93_time_usec_GET(pack);
    float*  controls = p93_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
    e_MAV_MODE  mode = p93_mode_GET(pack);
    uint64_t  flags = p93_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p100_time_usec_GET(pack);
    uint8_t  sensor_id = p100_sensor_id_GET(pack);
    int16_t  flow_x = p100_flow_x_GET(pack);
    int16_t  flow_y = p100_flow_y_GET(pack);
    float  flow_comp_m_x = p100_flow_comp_m_x_GET(pack);
    float  flow_comp_m_y = p100_flow_comp_m_y_GET(pack);
    uint8_t  quality = p100_quality_GET(pack);
    float  ground_distance = p100_ground_distance_GET(pack);
    float  flow_rate_x = p100_flow_rate_x_TRY(ph);
    float  flow_rate_y = p100_flow_rate_y_TRY(ph);
}
void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p101_usec_GET(pack);
    float  x = p101_x_GET(pack);
    float  y = p101_y_GET(pack);
    float  z = p101_z_GET(pack);
    float  roll = p101_roll_GET(pack);
    float  pitch = p101_pitch_GET(pack);
    float  yaw = p101_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p102_usec_GET(pack);
    float  x = p102_x_GET(pack);
    float  y = p102_y_GET(pack);
    float  z = p102_z_GET(pack);
    float  roll = p102_roll_GET(pack);
    float  pitch = p102_pitch_GET(pack);
    float  yaw = p102_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p103_usec_GET(pack);
    float  x = p103_x_GET(pack);
    float  y = p103_y_GET(pack);
    float  z = p103_z_GET(pack);
}
void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p104_usec_GET(pack);
    float  x = p104_x_GET(pack);
    float  y = p104_y_GET(pack);
    float  z = p104_z_GET(pack);
    float  roll = p104_roll_GET(pack);
    float  pitch = p104_pitch_GET(pack);
    float  yaw = p104_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p105_time_usec_GET(pack);
    float  xacc = p105_xacc_GET(pack);
    float  yacc = p105_yacc_GET(pack);
    float  zacc = p105_zacc_GET(pack);
    float  xgyro = p105_xgyro_GET(pack);
    float  ygyro = p105_ygyro_GET(pack);
    float  zgyro = p105_zgyro_GET(pack);
    float  xmag = p105_xmag_GET(pack);
    float  ymag = p105_ymag_GET(pack);
    float  zmag = p105_zmag_GET(pack);
    float  abs_pressure = p105_abs_pressure_GET(pack);
    float  diff_pressure = p105_diff_pressure_GET(pack);
    float  pressure_alt = p105_pressure_alt_GET(pack);
    float  temperature = p105_temperature_GET(pack);
    uint16_t  fields_updated = p105_fields_updated_GET(pack);
}
void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p106_time_usec_GET(pack);
    uint8_t  sensor_id = p106_sensor_id_GET(pack);
    uint32_t  integration_time_us = p106_integration_time_us_GET(pack);
    float  integrated_x = p106_integrated_x_GET(pack);
    float  integrated_y = p106_integrated_y_GET(pack);
    float  integrated_xgyro = p106_integrated_xgyro_GET(pack);
    float  integrated_ygyro = p106_integrated_ygyro_GET(pack);
    float  integrated_zgyro = p106_integrated_zgyro_GET(pack);
    int16_t  temperature = p106_temperature_GET(pack);
    uint8_t  quality = p106_quality_GET(pack);
    uint32_t  time_delta_distance_us = p106_time_delta_distance_us_GET(pack);
    float  distance = p106_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p107_time_usec_GET(pack);
    float  xacc = p107_xacc_GET(pack);
    float  yacc = p107_yacc_GET(pack);
    float  zacc = p107_zacc_GET(pack);
    float  xgyro = p107_xgyro_GET(pack);
    float  ygyro = p107_ygyro_GET(pack);
    float  zgyro = p107_zgyro_GET(pack);
    float  xmag = p107_xmag_GET(pack);
    float  ymag = p107_ymag_GET(pack);
    float  zmag = p107_zmag_GET(pack);
    float  abs_pressure = p107_abs_pressure_GET(pack);
    float  diff_pressure = p107_diff_pressure_GET(pack);
    float  pressure_alt = p107_pressure_alt_GET(pack);
    float  temperature = p107_temperature_GET(pack);
    uint32_t  fields_updated = p107_fields_updated_GET(pack);
}
void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    float  q1 = p108_q1_GET(pack);
    float  q2 = p108_q2_GET(pack);
    float  q3 = p108_q3_GET(pack);
    float  q4 = p108_q4_GET(pack);
    float  roll = p108_roll_GET(pack);
    float  pitch = p108_pitch_GET(pack);
    float  yaw = p108_yaw_GET(pack);
    float  xacc = p108_xacc_GET(pack);
    float  yacc = p108_yacc_GET(pack);
    float  zacc = p108_zacc_GET(pack);
    float  xgyro = p108_xgyro_GET(pack);
    float  ygyro = p108_ygyro_GET(pack);
    float  zgyro = p108_zgyro_GET(pack);
    float  lat = p108_lat_GET(pack);
    float  lon = p108_lon_GET(pack);
    float  alt = p108_alt_GET(pack);
    float  std_dev_horz = p108_std_dev_horz_GET(pack);
    float  std_dev_vert = p108_std_dev_vert_GET(pack);
    float  vn = p108_vn_GET(pack);
    float  ve = p108_ve_GET(pack);
    float  vd = p108_vd_GET(pack);
}
void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p109_rssi_GET(pack);
    uint8_t  remrssi = p109_remrssi_GET(pack);
    uint8_t  txbuf = p109_txbuf_GET(pack);
    uint8_t  noise = p109_noise_GET(pack);
    uint8_t  remnoise = p109_remnoise_GET(pack);
    uint16_t  rxerrors = p109_rxerrors_GET(pack);
    uint16_t  fixed_ = p109_fixed__GET(pack);
}
void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p110_target_network_GET(pack);
    uint8_t  target_system = p110_target_system_GET(pack);
    uint8_t  target_component = p110_target_component_GET(pack);
    uint8_t*  payload = p110_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    int64_t  tc1 = p111_tc1_GET(pack);
    int64_t  ts1 = p111_ts1_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p112_time_usec_GET(pack);
    uint32_t  seq = p112_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p113_time_usec_GET(pack);
    uint8_t  fix_type = p113_fix_type_GET(pack);
    int32_t  lat = p113_lat_GET(pack);
    int32_t  lon = p113_lon_GET(pack);
    int32_t  alt = p113_alt_GET(pack);
    uint16_t  eph = p113_eph_GET(pack);
    uint16_t  epv = p113_epv_GET(pack);
    uint16_t  vel = p113_vel_GET(pack);
    int16_t  vn = p113_vn_GET(pack);
    int16_t  ve = p113_ve_GET(pack);
    int16_t  vd = p113_vd_GET(pack);
    uint16_t  cog = p113_cog_GET(pack);
    uint8_t  satellites_visible = p113_satellites_visible_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p114_time_usec_GET(pack);
    uint8_t  sensor_id = p114_sensor_id_GET(pack);
    uint32_t  integration_time_us = p114_integration_time_us_GET(pack);
    float  integrated_x = p114_integrated_x_GET(pack);
    float  integrated_y = p114_integrated_y_GET(pack);
    float  integrated_xgyro = p114_integrated_xgyro_GET(pack);
    float  integrated_ygyro = p114_integrated_ygyro_GET(pack);
    float  integrated_zgyro = p114_integrated_zgyro_GET(pack);
    int16_t  temperature = p114_temperature_GET(pack);
    uint8_t  quality = p114_quality_GET(pack);
    uint32_t  time_delta_distance_us = p114_time_delta_distance_us_GET(pack);
    float  distance = p114_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p115_time_usec_GET(pack);
    float*  attitude_quaternion = p115_attitude_quaternion_GET_(pack);
//process data in attitude_quaternion
    free(attitude_quaternion);//never forget to dispose
    float  rollspeed = p115_rollspeed_GET(pack);
    float  pitchspeed = p115_pitchspeed_GET(pack);
    float  yawspeed = p115_yawspeed_GET(pack);
    int32_t  lat = p115_lat_GET(pack);
    int32_t  lon = p115_lon_GET(pack);
    int32_t  alt = p115_alt_GET(pack);
    int16_t  vx = p115_vx_GET(pack);
    int16_t  vy = p115_vy_GET(pack);
    int16_t  vz = p115_vz_GET(pack);
    uint16_t  ind_airspeed = p115_ind_airspeed_GET(pack);
    uint16_t  true_airspeed = p115_true_airspeed_GET(pack);
    int16_t  xacc = p115_xacc_GET(pack);
    int16_t  yacc = p115_yacc_GET(pack);
    int16_t  zacc = p115_zacc_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p116_time_boot_ms_GET(pack);
    int16_t  xacc = p116_xacc_GET(pack);
    int16_t  yacc = p116_yacc_GET(pack);
    int16_t  zacc = p116_zacc_GET(pack);
    int16_t  xgyro = p116_xgyro_GET(pack);
    int16_t  ygyro = p116_ygyro_GET(pack);
    int16_t  zgyro = p116_zgyro_GET(pack);
    int16_t  xmag = p116_xmag_GET(pack);
    int16_t  ymag = p116_ymag_GET(pack);
    int16_t  zmag = p116_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p117_target_system_GET(pack);
    uint8_t  target_component = p117_target_component_GET(pack);
    uint16_t  start = p117_start_GET(pack);
    uint16_t  end = p117_end_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p118_id_GET(pack);
    uint16_t  num_logs = p118_num_logs_GET(pack);
    uint16_t  last_log_num = p118_last_log_num_GET(pack);
    uint32_t  time_utc = p118_time_utc_GET(pack);
    uint32_t  size = p118_size_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p119_target_system_GET(pack);
    uint8_t  target_component = p119_target_component_GET(pack);
    uint16_t  id = p119_id_GET(pack);
    uint32_t  ofs = p119_ofs_GET(pack);
    uint32_t  count = p119_count_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p120_id_GET(pack);
    uint32_t  ofs = p120_ofs_GET(pack);
    uint8_t  count = p120_count_GET(pack);
    uint8_t*  data_ = p120_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p121_target_system_GET(pack);
    uint8_t  target_component = p121_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p122_target_system_GET(pack);
    uint8_t  target_component = p122_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p123_target_system_GET(pack);
    uint8_t  target_component = p123_target_component_GET(pack);
    uint8_t  len = p123_len_GET(pack);
    uint8_t*  data_ = p123_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p124_time_usec_GET(pack);
    e_GPS_FIX_TYPE  fix_type = p124_fix_type_GET(pack);
    int32_t  lat = p124_lat_GET(pack);
    int32_t  lon = p124_lon_GET(pack);
    int32_t  alt = p124_alt_GET(pack);
    uint16_t  eph = p124_eph_GET(pack);
    uint16_t  epv = p124_epv_GET(pack);
    uint16_t  vel = p124_vel_GET(pack);
    uint16_t  cog = p124_cog_GET(pack);
    uint8_t  satellites_visible = p124_satellites_visible_GET(pack);
    uint8_t  dgps_numch = p124_dgps_numch_GET(pack);
    uint32_t  dgps_age = p124_dgps_age_GET(pack);
}
void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p125_Vcc_GET(pack);
    uint16_t  Vservo = p125_Vservo_GET(pack);
    e_MAV_POWER_STATUS  flags = p125_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    e_SERIAL_CONTROL_DEV  device = p126_device_GET(pack);
    e_SERIAL_CONTROL_FLAG  flags = p126_flags_GET(pack);
    uint16_t  timeout = p126_timeout_GET(pack);
    uint32_t  baudrate = p126_baudrate_GET(pack);
    uint8_t  count = p126_count_GET(pack);
    uint8_t*  data_ = p126_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_last_baseline_ms = p127_time_last_baseline_ms_GET(pack);
    uint8_t  rtk_receiver_id = p127_rtk_receiver_id_GET(pack);
    uint16_t  wn = p127_wn_GET(pack);
    uint32_t  tow = p127_tow_GET(pack);
    uint8_t  rtk_health = p127_rtk_health_GET(pack);
    uint8_t  rtk_rate = p127_rtk_rate_GET(pack);
    uint8_t  nsats = p127_nsats_GET(pack);
    uint8_t  baseline_coords_type = p127_baseline_coords_type_GET(pack);
    int32_t  baseline_a_mm = p127_baseline_a_mm_GET(pack);
    int32_t  baseline_b_mm = p127_baseline_b_mm_GET(pack);
    int32_t  baseline_c_mm = p127_baseline_c_mm_GET(pack);
    uint32_t  accuracy = p127_accuracy_GET(pack);
    int32_t  iar_num_hypotheses = p127_iar_num_hypotheses_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_last_baseline_ms = p128_time_last_baseline_ms_GET(pack);
    uint8_t  rtk_receiver_id = p128_rtk_receiver_id_GET(pack);
    uint16_t  wn = p128_wn_GET(pack);
    uint32_t  tow = p128_tow_GET(pack);
    uint8_t  rtk_health = p128_rtk_health_GET(pack);
    uint8_t  rtk_rate = p128_rtk_rate_GET(pack);
    uint8_t  nsats = p128_nsats_GET(pack);
    uint8_t  baseline_coords_type = p128_baseline_coords_type_GET(pack);
    int32_t  baseline_a_mm = p128_baseline_a_mm_GET(pack);
    int32_t  baseline_b_mm = p128_baseline_b_mm_GET(pack);
    int32_t  baseline_c_mm = p128_baseline_c_mm_GET(pack);
    uint32_t  accuracy = p128_accuracy_GET(pack);
    int32_t  iar_num_hypotheses = p128_iar_num_hypotheses_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p129_time_boot_ms_GET(pack);
    int16_t  xacc = p129_xacc_GET(pack);
    int16_t  yacc = p129_yacc_GET(pack);
    int16_t  zacc = p129_zacc_GET(pack);
    int16_t  xgyro = p129_xgyro_GET(pack);
    int16_t  ygyro = p129_ygyro_GET(pack);
    int16_t  zgyro = p129_zgyro_GET(pack);
    int16_t  xmag = p129_xmag_GET(pack);
    int16_t  ymag = p129_ymag_GET(pack);
    int16_t  zmag = p129_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p130_type_GET(pack);
    uint32_t  size = p130_size_GET(pack);
    uint16_t  width = p130_width_GET(pack);
    uint16_t  height = p130_height_GET(pack);
    uint16_t  packets = p130_packets_GET(pack);
    uint8_t  payload = p130_payload_GET(pack);
    uint8_t  jpg_quality = p130_jpg_quality_GET(pack);
}
void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seqnr = p131_seqnr_GET(pack);
    uint8_t*  data_ = p131_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p132_time_boot_ms_GET(pack);
    uint16_t  min_distance = p132_min_distance_GET(pack);
    uint16_t  max_distance = p132_max_distance_GET(pack);
    uint16_t  current_distance = p132_current_distance_GET(pack);
    e_MAV_DISTANCE_SENSOR  type = p132_type_GET(pack);
    uint8_t  id = p132_id_GET(pack);
    e_MAV_SENSOR_ORIENTATION  orientation = p132_orientation_GET(pack);
    uint8_t  covariance = p132_covariance_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p133_lat_GET(pack);
    int32_t  lon = p133_lon_GET(pack);
    uint16_t  grid_spacing = p133_grid_spacing_GET(pack);
    uint64_t  mask = p133_mask_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p134_lat_GET(pack);
    int32_t  lon = p134_lon_GET(pack);
    uint16_t  grid_spacing = p134_grid_spacing_GET(pack);
    uint8_t  gridbit = p134_gridbit_GET(pack);
    int16_t*  data_ = p134_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p135_lat_GET(pack);
    int32_t  lon = p135_lon_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p136_lat_GET(pack);
    int32_t  lon = p136_lon_GET(pack);
    uint16_t  spacing = p136_spacing_GET(pack);
    float  terrain_height = p136_terrain_height_GET(pack);
    float  current_height = p136_current_height_GET(pack);
    uint16_t  pending = p136_pending_GET(pack);
    uint16_t  loaded = p136_loaded_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p137_time_boot_ms_GET(pack);
    float  press_abs = p137_press_abs_GET(pack);
    float  press_diff = p137_press_diff_GET(pack);
    int16_t  temperature = p137_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p138_time_usec_GET(pack);
    float*  q = p138_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  x = p138_x_GET(pack);
    float  y = p138_y_GET(pack);
    float  z = p138_z_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p139_time_usec_GET(pack);
    uint8_t  group_mlx = p139_group_mlx_GET(pack);
    uint8_t  target_system = p139_target_system_GET(pack);
    uint8_t  target_component = p139_target_component_GET(pack);
    float*  controls = p139_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p140_time_usec_GET(pack);
    uint8_t  group_mlx = p140_group_mlx_GET(pack);
    float*  controls = p140_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p141_time_usec_GET(pack);
    float  altitude_monotonic = p141_altitude_monotonic_GET(pack);
    float  altitude_amsl = p141_altitude_amsl_GET(pack);
    float  altitude_local = p141_altitude_local_GET(pack);
    float  altitude_relative = p141_altitude_relative_GET(pack);
    float  altitude_terrain = p141_altitude_terrain_GET(pack);
    float  bottom_clearance = p141_bottom_clearance_GET(pack);
}
void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  request_id = p142_request_id_GET(pack);
    uint8_t  uri_type = p142_uri_type_GET(pack);
    uint8_t*  uri = p142_uri_GET_(pack);
//process data in uri
    free(uri);//never forget to dispose
    uint8_t  transfer_type = p142_transfer_type_GET(pack);
    uint8_t*  storage = p142_storage_GET_(pack);
//process data in storage
    free(storage);//never forget to dispose
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p143_time_boot_ms_GET(pack);
    float  press_abs = p143_press_abs_GET(pack);
    float  press_diff = p143_press_diff_GET(pack);
    int16_t  temperature = p143_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p144_timestamp_GET(pack);
    uint8_t  est_capabilities = p144_est_capabilities_GET(pack);
    int32_t  lat = p144_lat_GET(pack);
    int32_t  lon = p144_lon_GET(pack);
    float  alt = p144_alt_GET(pack);
    float*  vel = p144_vel_GET_(pack);
//process data in vel
    free(vel);//never forget to dispose
    float*  acc = p144_acc_GET_(pack);
//process data in acc
    free(acc);//never forget to dispose
    float*  attitude_q = p144_attitude_q_GET_(pack);
//process data in attitude_q
    free(attitude_q);//never forget to dispose
    float*  rates = p144_rates_GET_(pack);
//process data in rates
    free(rates);//never forget to dispose
    float*  position_cov = p144_position_cov_GET_(pack);
//process data in position_cov
    free(position_cov);//never forget to dispose
    uint64_t  custom_state = p144_custom_state_GET(pack);
}
void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p146_time_usec_GET(pack);
    float  x_acc = p146_x_acc_GET(pack);
    float  y_acc = p146_y_acc_GET(pack);
    float  z_acc = p146_z_acc_GET(pack);
    float  x_vel = p146_x_vel_GET(pack);
    float  y_vel = p146_y_vel_GET(pack);
    float  z_vel = p146_z_vel_GET(pack);
    float  x_pos = p146_x_pos_GET(pack);
    float  y_pos = p146_y_pos_GET(pack);
    float  z_pos = p146_z_pos_GET(pack);
    float  airspeed = p146_airspeed_GET(pack);
    float*  vel_variance = p146_vel_variance_GET_(pack);
//process data in vel_variance
    free(vel_variance);//never forget to dispose
    float*  pos_variance = p146_pos_variance_GET_(pack);
//process data in pos_variance
    free(pos_variance);//never forget to dispose
    float*  q = p146_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  roll_rate = p146_roll_rate_GET(pack);
    float  pitch_rate = p146_pitch_rate_GET(pack);
    float  yaw_rate = p146_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  id = p147_id_GET(pack);
    e_MAV_BATTERY_FUNCTION  battery_function = p147_battery_function_GET(pack);
    e_MAV_BATTERY_TYPE  type = p147_type_GET(pack);
    int16_t  temperature = p147_temperature_GET(pack);
    uint16_t*  voltages = p147_voltages_GET_(pack);
//process data in voltages
    free(voltages);//never forget to dispose
    int16_t  current_battery = p147_current_battery_GET(pack);
    int32_t  current_consumed = p147_current_consumed_GET(pack);
    int32_t  energy_consumed = p147_energy_consumed_GET(pack);
    int8_t  battery_remaining = p147_battery_remaining_GET(pack);
}
void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_PROTOCOL_CAPABILITY  capabilities = p148_capabilities_GET(pack);
    uint32_t  flight_sw_version = p148_flight_sw_version_GET(pack);
    uint32_t  middleware_sw_version = p148_middleware_sw_version_GET(pack);
    uint32_t  os_sw_version = p148_os_sw_version_GET(pack);
    uint32_t  board_version = p148_board_version_GET(pack);
    uint8_t*  flight_custom_version = p148_flight_custom_version_GET_(pack);
//process data in flight_custom_version
    free(flight_custom_version);//never forget to dispose
    uint8_t*  middleware_custom_version = p148_middleware_custom_version_GET_(pack);
//process data in middleware_custom_version
    free(middleware_custom_version);//never forget to dispose
    uint8_t*  os_custom_version = p148_os_custom_version_GET_(pack);
//process data in os_custom_version
    free(os_custom_version);//never forget to dispose
    uint16_t  vendor_id = p148_vendor_id_GET(pack);
    uint16_t  product_id = p148_product_id_GET(pack);
    uint64_t  uid = p148_uid_GET(pack);
    uint8_t*  uid2 = p148_uid2_TRY(ph);
}
void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p149_time_usec_GET(pack);
    uint8_t  target_num = p149_target_num_GET(pack);
    e_MAV_FRAME  frame = p149_frame_GET(pack);
    float  angle_x = p149_angle_x_GET(pack);
    float  angle_y = p149_angle_y_GET(pack);
    float  distance = p149_distance_GET(pack);
    float  size_x = p149_size_x_GET(pack);
    float  size_y = p149_size_y_GET(pack);
    float  x = p149_x_TRY(ph);
    float  y = p149_y_TRY(ph);
    float  z = p149_z_TRY(ph);
    float*  q = p149_q_TRY(ph);
    e_LANDING_TARGET_TYPE  type = p149_type_GET(pack);
    uint8_t  position_valid = p149_position_valid_TRY(ph);
}
void c_LoopBackDemoChannel_on_SENS_POWER_201(Bounds_Inside * ph, Pack * pack)
{
    float  adc121_vspb_volt = p201_adc121_vspb_volt_GET(pack);
    float  adc121_cspb_amp = p201_adc121_cspb_amp_GET(pack);
    float  adc121_cs1_amp = p201_adc121_cs1_amp_GET(pack);
    float  adc121_cs2_amp = p201_adc121_cs2_amp_GET(pack);
}
void c_LoopBackDemoChannel_on_SENS_MPPT_202(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  mppt_timestamp = p202_mppt_timestamp_GET(pack);
    float  mppt1_volt = p202_mppt1_volt_GET(pack);
    float  mppt1_amp = p202_mppt1_amp_GET(pack);
    uint16_t  mppt1_pwm = p202_mppt1_pwm_GET(pack);
    uint8_t  mppt1_status = p202_mppt1_status_GET(pack);
    float  mppt2_volt = p202_mppt2_volt_GET(pack);
    float  mppt2_amp = p202_mppt2_amp_GET(pack);
    uint16_t  mppt2_pwm = p202_mppt2_pwm_GET(pack);
    uint8_t  mppt2_status = p202_mppt2_status_GET(pack);
    float  mppt3_volt = p202_mppt3_volt_GET(pack);
    float  mppt3_amp = p202_mppt3_amp_GET(pack);
    uint16_t  mppt3_pwm = p202_mppt3_pwm_GET(pack);
    uint8_t  mppt3_status = p202_mppt3_status_GET(pack);
}
void c_LoopBackDemoChannel_on_ASLCTRL_DATA_203(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p203_timestamp_GET(pack);
    uint8_t  aslctrl_mode = p203_aslctrl_mode_GET(pack);
    float  h = p203_h_GET(pack);
    float  hRef = p203_hRef_GET(pack);
    float  hRef_t = p203_hRef_t_GET(pack);
    float  PitchAngle = p203_PitchAngle_GET(pack);
    float  PitchAngleRef = p203_PitchAngleRef_GET(pack);
    float  q = p203_q_GET(pack);
    float  qRef = p203_qRef_GET(pack);
    float  uElev = p203_uElev_GET(pack);
    float  uThrot = p203_uThrot_GET(pack);
    float  uThrot2 = p203_uThrot2_GET(pack);
    float  nZ = p203_nZ_GET(pack);
    float  AirspeedRef = p203_AirspeedRef_GET(pack);
    uint8_t  SpoilersEngaged = p203_SpoilersEngaged_GET(pack);
    float  YawAngle = p203_YawAngle_GET(pack);
    float  YawAngleRef = p203_YawAngleRef_GET(pack);
    float  RollAngle = p203_RollAngle_GET(pack);
    float  RollAngleRef = p203_RollAngleRef_GET(pack);
    float  p = p203_p_GET(pack);
    float  pRef = p203_pRef_GET(pack);
    float  r = p203_r_GET(pack);
    float  rRef = p203_rRef_GET(pack);
    float  uAil = p203_uAil_GET(pack);
    float  uRud = p203_uRud_GET(pack);
}
void c_LoopBackDemoChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  i32_1 = p204_i32_1_GET(pack);
    uint8_t  i8_1 = p204_i8_1_GET(pack);
    uint8_t  i8_2 = p204_i8_2_GET(pack);
    float  f_1 = p204_f_1_GET(pack);
    float  f_2 = p204_f_2_GET(pack);
    float  f_3 = p204_f_3_GET(pack);
    float  f_4 = p204_f_4_GET(pack);
    float  f_5 = p204_f_5_GET(pack);
    float  f_6 = p204_f_6_GET(pack);
    float  f_7 = p204_f_7_GET(pack);
    float  f_8 = p204_f_8_GET(pack);
}
void c_LoopBackDemoChannel_on_ASLUAV_STATUS_205(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  LED_status = p205_LED_status_GET(pack);
    uint8_t  SATCOM_status = p205_SATCOM_status_GET(pack);
    uint8_t*  Servo_status = p205_Servo_status_GET_(pack);
//process data in Servo_status
    free(Servo_status);//never forget to dispose
    float  Motor_rpm = p205_Motor_rpm_GET(pack);
}
void c_LoopBackDemoChannel_on_EKF_EXT_206(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p206_timestamp_GET(pack);
    float  Windspeed = p206_Windspeed_GET(pack);
    float  WindDir = p206_WindDir_GET(pack);
    float  WindZ = p206_WindZ_GET(pack);
    float  Airspeed = p206_Airspeed_GET(pack);
    float  beta = p206_beta_GET(pack);
    float  alpha = p206_alpha_GET(pack);
}
void c_LoopBackDemoChannel_on_ASL_OBCTRL_207(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p207_timestamp_GET(pack);
    float  uElev = p207_uElev_GET(pack);
    float  uThrot = p207_uThrot_GET(pack);
    float  uThrot2 = p207_uThrot2_GET(pack);
    float  uAilL = p207_uAilL_GET(pack);
    float  uAilR = p207_uAilR_GET(pack);
    float  uRud = p207_uRud_GET(pack);
    uint8_t  obctrl_status = p207_obctrl_status_GET(pack);
}
void c_LoopBackDemoChannel_on_SENS_ATMOS_208(Bounds_Inside * ph, Pack * pack)
{
    float  TempAmbient = p208_TempAmbient_GET(pack);
    float  Humidity = p208_Humidity_GET(pack);
}
void c_LoopBackDemoChannel_on_SENS_BATMON_209(Bounds_Inside * ph, Pack * pack)
{
    float  temperature = p209_temperature_GET(pack);
    uint16_t  voltage = p209_voltage_GET(pack);
    int16_t  current = p209_current_GET(pack);
    uint8_t  SoC = p209_SoC_GET(pack);
    uint16_t  batterystatus = p209_batterystatus_GET(pack);
    uint16_t  serialnumber = p209_serialnumber_GET(pack);
    uint16_t  hostfetcontrol = p209_hostfetcontrol_GET(pack);
    uint16_t  cellvoltage1 = p209_cellvoltage1_GET(pack);
    uint16_t  cellvoltage2 = p209_cellvoltage2_GET(pack);
    uint16_t  cellvoltage3 = p209_cellvoltage3_GET(pack);
    uint16_t  cellvoltage4 = p209_cellvoltage4_GET(pack);
    uint16_t  cellvoltage5 = p209_cellvoltage5_GET(pack);
    uint16_t  cellvoltage6 = p209_cellvoltage6_GET(pack);
}
void c_LoopBackDemoChannel_on_FW_SOARING_DATA_210(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p210_timestamp_GET(pack);
    uint64_t  timestampModeChanged = p210_timestampModeChanged_GET(pack);
    float  xW = p210_xW_GET(pack);
    float  xR = p210_xR_GET(pack);
    float  xLat = p210_xLat_GET(pack);
    float  xLon = p210_xLon_GET(pack);
    float  VarW = p210_VarW_GET(pack);
    float  VarR = p210_VarR_GET(pack);
    float  VarLat = p210_VarLat_GET(pack);
    float  VarLon = p210_VarLon_GET(pack);
    float  LoiterRadius = p210_LoiterRadius_GET(pack);
    float  LoiterDirection = p210_LoiterDirection_GET(pack);
    float  DistToSoarPoint = p210_DistToSoarPoint_GET(pack);
    float  vSinkExp = p210_vSinkExp_GET(pack);
    float  z1_LocalUpdraftSpeed = p210_z1_LocalUpdraftSpeed_GET(pack);
    float  z2_DeltaRoll = p210_z2_DeltaRoll_GET(pack);
    float  z1_exp = p210_z1_exp_GET(pack);
    float  z2_exp = p210_z2_exp_GET(pack);
    float  ThermalGSNorth = p210_ThermalGSNorth_GET(pack);
    float  ThermalGSEast = p210_ThermalGSEast_GET(pack);
    float  TSE_dot = p210_TSE_dot_GET(pack);
    float  DebugVar1 = p210_DebugVar1_GET(pack);
    float  DebugVar2 = p210_DebugVar2_GET(pack);
    uint8_t  ControlMode = p210_ControlMode_GET(pack);
    uint8_t  valid = p210_valid_GET(pack);
}
void c_LoopBackDemoChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p211_timestamp_GET(pack);
    uint8_t  visensor_rate_1 = p211_visensor_rate_1_GET(pack);
    uint8_t  visensor_rate_2 = p211_visensor_rate_2_GET(pack);
    uint8_t  visensor_rate_3 = p211_visensor_rate_3_GET(pack);
    uint8_t  visensor_rate_4 = p211_visensor_rate_4_GET(pack);
    uint8_t  recording_nodes_count = p211_recording_nodes_count_GET(pack);
    uint8_t  cpu_temp = p211_cpu_temp_GET(pack);
    uint16_t  free_space = p211_free_space_GET(pack);
}
void c_LoopBackDemoChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p212_timestamp_GET(pack);
    uint8_t  pwr_brd_status = p212_pwr_brd_status_GET(pack);
    uint8_t  pwr_brd_led_status = p212_pwr_brd_led_status_GET(pack);
    float  pwr_brd_system_volt = p212_pwr_brd_system_volt_GET(pack);
    float  pwr_brd_servo_volt = p212_pwr_brd_servo_volt_GET(pack);
    float  pwr_brd_mot_l_amp = p212_pwr_brd_mot_l_amp_GET(pack);
    float  pwr_brd_mot_r_amp = p212_pwr_brd_mot_r_amp_GET(pack);
    float  pwr_brd_servo_1_amp = p212_pwr_brd_servo_1_amp_GET(pack);
    float  pwr_brd_servo_2_amp = p212_pwr_brd_servo_2_amp_GET(pack);
    float  pwr_brd_servo_3_amp = p212_pwr_brd_servo_3_amp_GET(pack);
    float  pwr_brd_servo_4_amp = p212_pwr_brd_servo_4_amp_GET(pack);
    float  pwr_brd_aux_amp = p212_pwr_brd_aux_amp_GET(pack);
}
void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p230_time_usec_GET(pack);
    e_ESTIMATOR_STATUS_FLAGS  flags = p230_flags_GET(pack);
    float  vel_ratio = p230_vel_ratio_GET(pack);
    float  pos_horiz_ratio = p230_pos_horiz_ratio_GET(pack);
    float  pos_vert_ratio = p230_pos_vert_ratio_GET(pack);
    float  mag_ratio = p230_mag_ratio_GET(pack);
    float  hagl_ratio = p230_hagl_ratio_GET(pack);
    float  tas_ratio = p230_tas_ratio_GET(pack);
    float  pos_horiz_accuracy = p230_pos_horiz_accuracy_GET(pack);
    float  pos_vert_accuracy = p230_pos_vert_accuracy_GET(pack);
}
void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p231_time_usec_GET(pack);
    float  wind_x = p231_wind_x_GET(pack);
    float  wind_y = p231_wind_y_GET(pack);
    float  wind_z = p231_wind_z_GET(pack);
    float  var_horiz = p231_var_horiz_GET(pack);
    float  var_vert = p231_var_vert_GET(pack);
    float  wind_alt = p231_wind_alt_GET(pack);
    float  horiz_accuracy = p231_horiz_accuracy_GET(pack);
    float  vert_accuracy = p231_vert_accuracy_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p232_time_usec_GET(pack);
    uint8_t  gps_id = p232_gps_id_GET(pack);
    e_GPS_INPUT_IGNORE_FLAGS  ignore_flags = p232_ignore_flags_GET(pack);
    uint32_t  time_week_ms = p232_time_week_ms_GET(pack);
    uint16_t  time_week = p232_time_week_GET(pack);
    uint8_t  fix_type = p232_fix_type_GET(pack);
    int32_t  lat = p232_lat_GET(pack);
    int32_t  lon = p232_lon_GET(pack);
    float  alt = p232_alt_GET(pack);
    float  hdop = p232_hdop_GET(pack);
    float  vdop = p232_vdop_GET(pack);
    float  vn = p232_vn_GET(pack);
    float  ve = p232_ve_GET(pack);
    float  vd = p232_vd_GET(pack);
    float  speed_accuracy = p232_speed_accuracy_GET(pack);
    float  horiz_accuracy = p232_horiz_accuracy_GET(pack);
    float  vert_accuracy = p232_vert_accuracy_GET(pack);
    uint8_t  satellites_visible = p232_satellites_visible_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  flags = p233_flags_GET(pack);
    uint8_t  len = p233_len_GET(pack);
    uint8_t*  data_ = p233_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_MODE_FLAG  base_mode = p234_base_mode_GET(pack);
    uint32_t  custom_mode = p234_custom_mode_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p234_landed_state_GET(pack);
    int16_t  roll = p234_roll_GET(pack);
    int16_t  pitch = p234_pitch_GET(pack);
    uint16_t  heading = p234_heading_GET(pack);
    int8_t  throttle = p234_throttle_GET(pack);
    int16_t  heading_sp = p234_heading_sp_GET(pack);
    int32_t  latitude = p234_latitude_GET(pack);
    int32_t  longitude = p234_longitude_GET(pack);
    int16_t  altitude_amsl = p234_altitude_amsl_GET(pack);
    int16_t  altitude_sp = p234_altitude_sp_GET(pack);
    uint8_t  airspeed = p234_airspeed_GET(pack);
    uint8_t  airspeed_sp = p234_airspeed_sp_GET(pack);
    uint8_t  groundspeed = p234_groundspeed_GET(pack);
    int8_t  climb_rate = p234_climb_rate_GET(pack);
    uint8_t  gps_nsat = p234_gps_nsat_GET(pack);
    e_GPS_FIX_TYPE  gps_fix_type = p234_gps_fix_type_GET(pack);
    uint8_t  battery_remaining = p234_battery_remaining_GET(pack);
    int8_t  temperature = p234_temperature_GET(pack);
    int8_t  temperature_air = p234_temperature_air_GET(pack);
    uint8_t  failsafe = p234_failsafe_GET(pack);
    uint8_t  wp_num = p234_wp_num_GET(pack);
    uint16_t  wp_distance = p234_wp_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p241_time_usec_GET(pack);
    float  vibration_x = p241_vibration_x_GET(pack);
    float  vibration_y = p241_vibration_y_GET(pack);
    float  vibration_z = p241_vibration_z_GET(pack);
    uint32_t  clipping_0 = p241_clipping_0_GET(pack);
    uint32_t  clipping_1 = p241_clipping_1_GET(pack);
    uint32_t  clipping_2 = p241_clipping_2_GET(pack);
}
void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    int32_t  latitude = p242_latitude_GET(pack);
    int32_t  longitude = p242_longitude_GET(pack);
    int32_t  altitude = p242_altitude_GET(pack);
    float  x = p242_x_GET(pack);
    float  y = p242_y_GET(pack);
    float  z = p242_z_GET(pack);
    float*  q = p242_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  approach_x = p242_approach_x_GET(pack);
    float  approach_y = p242_approach_y_GET(pack);
    float  approach_z = p242_approach_z_GET(pack);
    uint64_t  time_usec = p242_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p243_target_system_GET(pack);
    int32_t  latitude = p243_latitude_GET(pack);
    int32_t  longitude = p243_longitude_GET(pack);
    int32_t  altitude = p243_altitude_GET(pack);
    float  x = p243_x_GET(pack);
    float  y = p243_y_GET(pack);
    float  z = p243_z_GET(pack);
    float*  q = p243_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  approach_x = p243_approach_x_GET(pack);
    float  approach_y = p243_approach_y_GET(pack);
    float  approach_z = p243_approach_z_GET(pack);
    uint64_t  time_usec = p243_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  message_id = p244_message_id_GET(pack);
    int32_t  interval_us = p244_interval_us_GET(pack);
}
void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_VTOL_STATE  vtol_state = p245_vtol_state_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p245_landed_state_GET(pack);
}
void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  ICAO_address = p246_ICAO_address_GET(pack);
    int32_t  lat = p246_lat_GET(pack);
    int32_t  lon = p246_lon_GET(pack);
    e_ADSB_ALTITUDE_TYPE  altitude_type = p246_altitude_type_GET(pack);
    int32_t  altitude = p246_altitude_GET(pack);
    uint16_t  heading = p246_heading_GET(pack);
    uint16_t  hor_velocity = p246_hor_velocity_GET(pack);
    int16_t  ver_velocity = p246_ver_velocity_GET(pack);
    char16_t *  callsign = p246_callsign_TRY_(ph);
    e_ADSB_EMITTER_TYPE  emitter_type = p246_emitter_type_GET(pack);
    uint8_t  tslc = p246_tslc_GET(pack);
    e_ADSB_FLAGS  flags = p246_flags_GET(pack);
    uint16_t  squawk = p246_squawk_GET(pack);
}
void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_COLLISION_SRC  src_ = p247_src__GET(pack);
    uint32_t  id = p247_id_GET(pack);
    e_MAV_COLLISION_ACTION  action = p247_action_GET(pack);
    e_MAV_COLLISION_THREAT_LEVEL  threat_level = p247_threat_level_GET(pack);
    float  time_to_minimum_delta = p247_time_to_minimum_delta_GET(pack);
    float  altitude_minimum_delta = p247_altitude_minimum_delta_GET(pack);
    float  horizontal_minimum_delta = p247_horizontal_minimum_delta_GET(pack);
}
void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p248_target_network_GET(pack);
    uint8_t  target_system = p248_target_system_GET(pack);
    uint8_t  target_component = p248_target_component_GET(pack);
    uint16_t  message_type = p248_message_type_GET(pack);
    uint8_t*  payload = p248_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  address = p249_address_GET(pack);
    uint8_t  ver = p249_ver_GET(pack);
    uint8_t  type = p249_type_GET(pack);
    int8_t*  value = p249_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  name = p250_name_TRY_(ph);
    uint64_t  time_usec = p250_time_usec_GET(pack);
    float  x = p250_x_GET(pack);
    float  y = p250_y_GET(pack);
    float  z = p250_z_GET(pack);
}
void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p251_time_boot_ms_GET(pack);
    char16_t *  name = p251_name_TRY_(ph);
    float  value = p251_value_GET(pack);
}
void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p252_time_boot_ms_GET(pack);
    char16_t *  name = p252_name_TRY_(ph);
    int32_t  value = p252_value_GET(pack);
}
void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SEVERITY  severity = p253_severity_GET(pack);
    char16_t *  text = p253_text_TRY_(ph);
}
void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p254_time_boot_ms_GET(pack);
    uint8_t  ind = p254_ind_GET(pack);
    float  value = p254_value_GET(pack);
}
void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p256_target_system_GET(pack);
    uint8_t  target_component = p256_target_component_GET(pack);
    uint8_t*  secret_key = p256_secret_key_GET_(pack);
//process data in secret_key
    free(secret_key);//never forget to dispose
    uint64_t  initial_timestamp = p256_initial_timestamp_GET(pack);
}
void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p257_time_boot_ms_GET(pack);
    uint32_t  last_change_ms = p257_last_change_ms_GET(pack);
    uint8_t  state = p257_state_GET(pack);
}
void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p258_target_system_GET(pack);
    uint8_t  target_component = p258_target_component_GET(pack);
    char16_t *  tune = p258_tune_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p259_time_boot_ms_GET(pack);
    uint8_t*  vendor_name = p259_vendor_name_GET_(pack);
//process data in vendor_name
    free(vendor_name);//never forget to dispose
    uint8_t*  model_name = p259_model_name_GET_(pack);
//process data in model_name
    free(model_name);//never forget to dispose
    uint32_t  firmware_version = p259_firmware_version_GET(pack);
    float  focal_length = p259_focal_length_GET(pack);
    float  sensor_size_h = p259_sensor_size_h_GET(pack);
    float  sensor_size_v = p259_sensor_size_v_GET(pack);
    uint16_t  resolution_h = p259_resolution_h_GET(pack);
    uint16_t  resolution_v = p259_resolution_v_GET(pack);
    uint8_t  lens_id = p259_lens_id_GET(pack);
    e_CAMERA_CAP_FLAGS  flags = p259_flags_GET(pack);
    uint16_t  cam_definition_version = p259_cam_definition_version_GET(pack);
    char16_t *  cam_definition_uri = p259_cam_definition_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p260_time_boot_ms_GET(pack);
    e_CAMERA_MODE  mode_id = p260_mode_id_GET(pack);
}
void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p261_time_boot_ms_GET(pack);
    uint8_t  storage_id = p261_storage_id_GET(pack);
    uint8_t  storage_count = p261_storage_count_GET(pack);
    uint8_t  status = p261_status_GET(pack);
    float  total_capacity = p261_total_capacity_GET(pack);
    float  used_capacity = p261_used_capacity_GET(pack);
    float  available_capacity = p261_available_capacity_GET(pack);
    float  read_speed = p261_read_speed_GET(pack);
    float  write_speed = p261_write_speed_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p262_time_boot_ms_GET(pack);
    uint8_t  image_status = p262_image_status_GET(pack);
    uint8_t  video_status = p262_video_status_GET(pack);
    float  image_interval = p262_image_interval_GET(pack);
    uint32_t  recording_time_ms = p262_recording_time_ms_GET(pack);
    float  available_capacity = p262_available_capacity_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p263_time_boot_ms_GET(pack);
    uint64_t  time_utc = p263_time_utc_GET(pack);
    uint8_t  camera_id = p263_camera_id_GET(pack);
    int32_t  lat = p263_lat_GET(pack);
    int32_t  lon = p263_lon_GET(pack);
    int32_t  alt = p263_alt_GET(pack);
    int32_t  relative_alt = p263_relative_alt_GET(pack);
    float*  q = p263_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    int32_t  image_index = p263_image_index_GET(pack);
    int8_t  capture_result = p263_capture_result_GET(pack);
    char16_t *  file_url = p263_file_url_TRY_(ph);
}
void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p264_time_boot_ms_GET(pack);
    uint64_t  arming_time_utc = p264_arming_time_utc_GET(pack);
    uint64_t  takeoff_time_utc = p264_takeoff_time_utc_GET(pack);
    uint64_t  flight_uuid = p264_flight_uuid_GET(pack);
}
void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p265_time_boot_ms_GET(pack);
    float  roll = p265_roll_GET(pack);
    float  pitch = p265_pitch_GET(pack);
    float  yaw = p265_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p266_target_system_GET(pack);
    uint8_t  target_component = p266_target_component_GET(pack);
    uint16_t  sequence = p266_sequence_GET(pack);
    uint8_t  length = p266_length_GET(pack);
    uint8_t  first_message_offset = p266_first_message_offset_GET(pack);
    uint8_t*  data_ = p266_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p267_target_system_GET(pack);
    uint8_t  target_component = p267_target_component_GET(pack);
    uint16_t  sequence = p267_sequence_GET(pack);
    uint8_t  length = p267_length_GET(pack);
    uint8_t  first_message_offset = p267_first_message_offset_GET(pack);
    uint8_t*  data_ = p267_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p268_target_system_GET(pack);
    uint8_t  target_component = p268_target_component_GET(pack);
    uint16_t  sequence = p268_sequence_GET(pack);
}
void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  camera_id = p269_camera_id_GET(pack);
    uint8_t  status = p269_status_GET(pack);
    float  framerate = p269_framerate_GET(pack);
    uint16_t  resolution_h = p269_resolution_h_GET(pack);
    uint16_t  resolution_v = p269_resolution_v_GET(pack);
    uint32_t  bitrate = p269_bitrate_GET(pack);
    uint16_t  rotation = p269_rotation_GET(pack);
    char16_t *  uri = p269_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p270_target_system_GET(pack);
    uint8_t  target_component = p270_target_component_GET(pack);
    uint8_t  camera_id = p270_camera_id_GET(pack);
    float  framerate = p270_framerate_GET(pack);
    uint16_t  resolution_h = p270_resolution_h_GET(pack);
    uint16_t  resolution_v = p270_resolution_v_GET(pack);
    uint32_t  bitrate = p270_bitrate_GET(pack);
    uint16_t  rotation = p270_rotation_GET(pack);
    char16_t *  uri = p270_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  ssid = p299_ssid_TRY_(ph);
    char16_t *  password = p299_password_TRY_(ph);
}
void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  version = p300_version_GET(pack);
    uint16_t  min_version = p300_min_version_GET(pack);
    uint16_t  max_version = p300_max_version_GET(pack);
    uint8_t*  spec_version_hash = p300_spec_version_hash_GET_(pack);
//process data in spec_version_hash
    free(spec_version_hash);//never forget to dispose
    uint8_t*  library_version_hash = p300_library_version_hash_GET_(pack);
//process data in library_version_hash
    free(library_version_hash);//never forget to dispose
}
void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p310_time_usec_GET(pack);
    uint32_t  uptime_sec = p310_uptime_sec_GET(pack);
    e_UAVCAN_NODE_HEALTH  health = p310_health_GET(pack);
    e_UAVCAN_NODE_MODE  mode = p310_mode_GET(pack);
    uint8_t  sub_mode = p310_sub_mode_GET(pack);
    uint16_t  vendor_specific_status_code = p310_vendor_specific_status_code_GET(pack);
}
void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p311_time_usec_GET(pack);
    uint32_t  uptime_sec = p311_uptime_sec_GET(pack);
    char16_t *  name = p311_name_TRY_(ph);
    uint8_t  hw_version_major = p311_hw_version_major_GET(pack);
    uint8_t  hw_version_minor = p311_hw_version_minor_GET(pack);
    uint8_t*  hw_unique_id = p311_hw_unique_id_GET_(pack);
//process data in hw_unique_id
    free(hw_unique_id);//never forget to dispose
    uint8_t  sw_version_major = p311_sw_version_major_GET(pack);
    uint8_t  sw_version_minor = p311_sw_version_minor_GET(pack);
    uint32_t  sw_vcs_commit = p311_sw_vcs_commit_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p320_target_system_GET(pack);
    uint8_t  target_component = p320_target_component_GET(pack);
    char16_t *  param_id = p320_param_id_TRY_(ph);
    int16_t  param_index = p320_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p321_target_system_GET(pack);
    uint8_t  target_component = p321_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p322_param_id_TRY_(ph);
    char16_t *  param_value = p322_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p322_param_type_GET(pack);
    uint16_t  param_count = p322_param_count_GET(pack);
    uint16_t  param_index = p322_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p323_target_system_GET(pack);
    uint8_t  target_component = p323_target_component_GET(pack);
    char16_t *  param_id = p323_param_id_TRY_(ph);
    char16_t *  param_value = p323_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p323_param_type_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p324_param_id_TRY_(ph);
    char16_t *  param_value = p324_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p324_param_type_GET(pack);
    e_PARAM_ACK  param_result = p324_param_result_GET(pack);
}
void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p330_time_usec_GET(pack);
    e_MAV_DISTANCE_SENSOR  sensor_type = p330_sensor_type_GET(pack);
    uint16_t*  distances = p330_distances_GET_(pack);
//process data in distances
    free(distances);//never forget to dispose
    uint8_t  increment = p330_increment_GET(pack);
    uint16_t  min_distance = p330_min_distance_GET(pack);
    uint16_t  max_distance = p330_max_distance_GET(pack);
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1808193748L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)9574, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)22344, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)15129, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -43, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)8218, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)19928, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)18232, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)34909, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)31613, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)39208, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)2373307896670196878L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)3317860452L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)86969440L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)41201, PH.base.pack) ;
    p3_x_SET((float)2.4228691E38F, PH.base.pack) ;
    p3_y_SET((float) -3.1619654E38F, PH.base.pack) ;
    p3_z_SET((float) -1.6958443E38F, PH.base.pack) ;
    p3_vx_SET((float) -1.3259162E38F, PH.base.pack) ;
    p3_vy_SET((float)3.06111E38F, PH.base.pack) ;
    p3_vz_SET((float)8.630111E37F, PH.base.pack) ;
    p3_afx_SET((float) -2.9651238E38F, PH.base.pack) ;
    p3_afy_SET((float) -7.204787E37F, PH.base.pack) ;
    p3_afz_SET((float) -2.053844E38F, PH.base.pack) ;
    p3_yaw_SET((float)2.2924342E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -3.342527E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)751679050501388289L, PH.base.pack) ;
    p4_seq_SET((uint32_t)428984962L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    {
        char16_t   passkey = "mjuzwahb";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "cisxk";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)1177924290L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    {
        char16_t   param_id = "yZlmyiy";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)24239, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "xrzbfmpxb";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -1.3482015E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)17294, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)22903, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    {
        char16_t   param_id = "Alxknvim";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)3.804476E37F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)2307209422434668407L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
    p24_lat_SET((int32_t) -915386834, PH.base.pack) ;
    p24_lon_SET((int32_t)1462881575, PH.base.pack) ;
    p24_alt_SET((int32_t)1990099835, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)5061, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)60070, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)12050, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)53139, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)314099072, &PH) ;
    p24_h_acc_SET((uint32_t)1998411712L, &PH) ;
    p24_v_acc_SET((uint32_t)2395394825L, &PH) ;
    p24_vel_acc_SET((uint32_t)507850507L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3030320894L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)178, (uint8_t)201, (uint8_t)75, (uint8_t)205, (uint8_t)224, (uint8_t)101, (uint8_t)198, (uint8_t)253, (uint8_t)226, (uint8_t)125, (uint8_t)219, (uint8_t)13, (uint8_t)57, (uint8_t)39, (uint8_t)68, (uint8_t)158, (uint8_t)7, (uint8_t)42, (uint8_t)29, (uint8_t)97};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)216, (uint8_t)160, (uint8_t)164, (uint8_t)74, (uint8_t)189, (uint8_t)219, (uint8_t)56, (uint8_t)22, (uint8_t)233, (uint8_t)246, (uint8_t)40, (uint8_t)97, (uint8_t)227, (uint8_t)160, (uint8_t)173, (uint8_t)179, (uint8_t)146, (uint8_t)70, (uint8_t)180, (uint8_t)223};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)4, (uint8_t)242, (uint8_t)150, (uint8_t)39, (uint8_t)116, (uint8_t)34, (uint8_t)7, (uint8_t)170, (uint8_t)33, (uint8_t)88, (uint8_t)220, (uint8_t)255, (uint8_t)102, (uint8_t)17, (uint8_t)238, (uint8_t)162, (uint8_t)209, (uint8_t)169, (uint8_t)7, (uint8_t)2};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)16, (uint8_t)25, (uint8_t)27, (uint8_t)157, (uint8_t)4, (uint8_t)113, (uint8_t)127, (uint8_t)207, (uint8_t)59, (uint8_t)78, (uint8_t)233, (uint8_t)178, (uint8_t)45, (uint8_t)74, (uint8_t)38, (uint8_t)238, (uint8_t)15, (uint8_t)36, (uint8_t)100, (uint8_t)208};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)192, (uint8_t)186, (uint8_t)135, (uint8_t)35, (uint8_t)13, (uint8_t)64, (uint8_t)91, (uint8_t)136, (uint8_t)234, (uint8_t)65, (uint8_t)6, (uint8_t)108, (uint8_t)104, (uint8_t)206, (uint8_t)40, (uint8_t)148, (uint8_t)132, (uint8_t)11, (uint8_t)56, (uint8_t)129};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)413526069L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -18640, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)21332, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)31152, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -1483, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)3488, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -28562, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -5867, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -24239, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)3880, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)2545020203790971688L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -19696, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -28984, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)30880, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -3247, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -5337, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -23559, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -29970, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -11959, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -4604, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)8914866215378929292L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -30812, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)30104, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -10511, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)13421, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)75688950L, PH.base.pack) ;
    p29_press_abs_SET((float) -2.8967439E38F, PH.base.pack) ;
    p29_press_diff_SET((float)2.0515937E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)10869, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)88002673L, PH.base.pack) ;
    p30_roll_SET((float)1.9167674E38F, PH.base.pack) ;
    p30_pitch_SET((float) -2.106882E38F, PH.base.pack) ;
    p30_yaw_SET((float)2.947759E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)3.1392791E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -2.191643E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -1.6117378E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1008432104L, PH.base.pack) ;
    p31_q1_SET((float)9.108892E37F, PH.base.pack) ;
    p31_q2_SET((float) -3.9831428E37F, PH.base.pack) ;
    p31_q3_SET((float)2.7341071E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.4544766E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -3.671281E37F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -3.3872143E38F, PH.base.pack) ;
    p31_yawspeed_SET((float) -2.3431091E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)2125908081L, PH.base.pack) ;
    p32_x_SET((float) -1.473871E38F, PH.base.pack) ;
    p32_y_SET((float) -2.256563E38F, PH.base.pack) ;
    p32_z_SET((float)1.6786446E37F, PH.base.pack) ;
    p32_vx_SET((float) -2.8534847E38F, PH.base.pack) ;
    p32_vy_SET((float) -7.881989E37F, PH.base.pack) ;
    p32_vz_SET((float)1.6305765E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)2634469464L, PH.base.pack) ;
    p33_lat_SET((int32_t)1296606353, PH.base.pack) ;
    p33_lon_SET((int32_t)2028124469, PH.base.pack) ;
    p33_alt_SET((int32_t) -565681485, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1907073129, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)6854, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -21292, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -26856, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)56946, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)763453829L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)2455, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)23543, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)25893, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -31570, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)122, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -20475, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -16981, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)11180, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)311545913L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)56979, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)49043, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)14193, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)11849, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)34531, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)19440, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)42491, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)9614, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)686399410L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)32321, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)20914, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)42816, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)47342, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)61973, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)32960, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)26122, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)55581, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)52552, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)15004, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)19018, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)9901, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)37758, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)33868, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)57223, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)59520, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -15825, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -3067, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -17751, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)15945, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)7813, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_GET_HOME_POSITION, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p39_param1_SET((float) -1.2698684E37F, PH.base.pack) ;
    p39_param2_SET((float) -3.205653E38F, PH.base.pack) ;
    p39_param3_SET((float) -7.5320254E37F, PH.base.pack) ;
    p39_param4_SET((float)4.022661E37F, PH.base.pack) ;
    p39_x_SET((float)1.1044797E38F, PH.base.pack) ;
    p39_y_SET((float)2.6348385E38F, PH.base.pack) ;
    p39_z_SET((float) -9.489111E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)24822, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)55033, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)48855, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)52112, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)59359, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p48_latitude_SET((int32_t)1970575683, PH.base.pack) ;
    p48_longitude_SET((int32_t)651510002, PH.base.pack) ;
    p48_altitude_SET((int32_t)1163005425, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)5165630706854226726L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -1632163964, PH.base.pack) ;
    p49_longitude_SET((int32_t) -840180796, PH.base.pack) ;
    p49_altitude_SET((int32_t)2095334877, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)3190789441106017404L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    {
        char16_t   param_id = "ycuXqwtagcbwy";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -3215, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p50_param_value0_SET((float)2.324339E38F, PH.base.pack) ;
    p50_scale_SET((float)2.5094148E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.8883494E37F, PH.base.pack) ;
    p50_param_value_max_SET((float) -2.349575E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)7392, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p54_p1x_SET((float)2.2619448E38F, PH.base.pack) ;
    p54_p1y_SET((float) -1.4042879E38F, PH.base.pack) ;
    p54_p1z_SET((float) -3.004668E37F, PH.base.pack) ;
    p54_p2x_SET((float)2.44712E38F, PH.base.pack) ;
    p54_p2y_SET((float) -2.8380772E38F, PH.base.pack) ;
    p54_p2z_SET((float)3.2295245E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float)2.1888418E36F, PH.base.pack) ;
    p55_p1y_SET((float) -1.6368879E38F, PH.base.pack) ;
    p55_p1z_SET((float)1.5282249E38F, PH.base.pack) ;
    p55_p2x_SET((float)2.9197626E38F, PH.base.pack) ;
    p55_p2y_SET((float) -1.6276923E38F, PH.base.pack) ;
    p55_p2z_SET((float) -5.6152003E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)2435860898378856256L, PH.base.pack) ;
    {
        float  q [] =  {-2.9925012E38F, 2.8531795E38F, -3.0627605E36F, -1.8422902E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -3.0819101E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -3.2366223E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -2.545438E38F, PH.base.pack) ;
    {
        float  covariance [] =  {5.8850963E37F, -2.949392E38F, -2.9286276E38F, 1.1554705E38F, 1.1252831E38F, -5.757452E37F, -1.6202409E38F, -1.4092212E38F, -6.654917E37F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -1.3609081E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)1.9555924E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)520, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -18176, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)26436, PH.base.pack) ;
    p62_alt_error_SET((float) -3.3801094E37F, PH.base.pack) ;
    p62_aspd_error_SET((float)1.350418E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)3.0904099E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)6471172656694026399L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p63_lat_SET((int32_t) -347404482, PH.base.pack) ;
    p63_lon_SET((int32_t) -230772717, PH.base.pack) ;
    p63_alt_SET((int32_t)1172290180, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -845208843, PH.base.pack) ;
    p63_vx_SET((float) -1.8481311E38F, PH.base.pack) ;
    p63_vy_SET((float) -3.2646674E38F, PH.base.pack) ;
    p63_vz_SET((float)3.2690257E38F, PH.base.pack) ;
    {
        float  covariance [] =  {2.2988158E38F, -2.7512882E38F, -2.9046589E38F, 2.8697754E38F, -1.0402975E38F, -1.687676E38F, -1.1270742E38F, -8.615706E37F, -2.1680675E38F, -2.7031467E38F, 2.5430582E38F, 2.735506E38F, 2.2190035E38F, 1.9194453E38F, 6.0705044E37F, 2.4106435E38F, -3.1369996E38F, 3.0640957E38F, 1.3661777E38F, 1.0980678E38F, 5.8965964E37F, -1.520047E38F, 2.974559E38F, -1.6910669E38F, -8.4441357E36F, -9.594784E37F, 8.898618E37F, 2.65377E38F, 3.2119978E38F, 2.4392695E38F, -4.9718053E37F, 3.3880834E38F, 7.6227826E37F, 8.520777E37F, 3.1135547E38F, -1.2640009E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)6209426250207328944L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p64_x_SET((float)6.1577933E37F, PH.base.pack) ;
    p64_y_SET((float)2.987137E37F, PH.base.pack) ;
    p64_z_SET((float)2.1419571E38F, PH.base.pack) ;
    p64_vx_SET((float)1.3884556E38F, PH.base.pack) ;
    p64_vy_SET((float)2.0922174E37F, PH.base.pack) ;
    p64_vz_SET((float)2.4566738E38F, PH.base.pack) ;
    p64_ax_SET((float) -2.266374E37F, PH.base.pack) ;
    p64_ay_SET((float) -1.6024826E38F, PH.base.pack) ;
    p64_az_SET((float)6.957039E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-7.727588E37F, 8.519631E37F, -8.06775E37F, 9.016801E37F, 2.1006187E38F, -1.2993393E38F, 7.8048025E37F, -2.8206157E38F, 8.3769227E37F, 4.277226E37F, 2.5702565E38F, 2.0335823E38F, 9.964571E37F, 9.865606E37F, 1.913126E38F, 1.763013E38F, -1.6167393E38F, -1.6963505E38F, -1.6645205E38F, 1.1683071E37F, -2.7203926E38F, 3.1682886E38F, -3.32329E38F, 3.2085237E38F, 3.207585E38F, 2.7363774E38F, -3.1810319E38F, -2.6136921E38F, 4.5757076E37F, -2.8817252E38F, -3.3299532E38F, 7.5586664E37F, 2.366916E38F, -2.6870047E38F, -2.8980392E37F, 2.6762508E37F, -1.3007174E37F, 2.941468E37F, -2.4252324E38F, -2.5355933E38F, -2.7352774E38F, 1.7013072E38F, 2.3119316E38F, 2.2050397E38F, -3.803092E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)87691032L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)19917, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)28015, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)58613, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)26685, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)27177, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)3558, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)9299, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)29100, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)11876, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)56322, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)33417, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)10131, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)51404, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)13077, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)24840, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)64621, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)64937, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)24018, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)20743, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)54195, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -7963, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -32262, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)19441, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)3551, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)53474, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)54534, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)39848, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)55840, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)8384, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)50008, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)8702, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)58446, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)20267, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)12276, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p73_param1_SET((float)3.9145824E37F, PH.base.pack) ;
    p73_param2_SET((float) -1.3417531E38F, PH.base.pack) ;
    p73_param3_SET((float)7.574931E37F, PH.base.pack) ;
    p73_param4_SET((float)3.1207211E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -783245266, PH.base.pack) ;
    p73_y_SET((int32_t) -1433623323, PH.base.pack) ;
    p73_z_SET((float) -3.0061336E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.8402693E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.2379474E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -23571, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)33711, PH.base.pack) ;
    p74_alt_SET((float)2.4543782E38F, PH.base.pack) ;
    p74_climb_SET((float) -1.5148514E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p75_param1_SET((float) -1.0544095E38F, PH.base.pack) ;
    p75_param2_SET((float)1.6790034E38F, PH.base.pack) ;
    p75_param3_SET((float)1.9431634E38F, PH.base.pack) ;
    p75_param4_SET((float) -5.57734E37F, PH.base.pack) ;
    p75_x_SET((int32_t) -351421230, PH.base.pack) ;
    p75_y_SET((int32_t)2062470981, PH.base.pack) ;
    p75_z_SET((float)2.1976859E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p76_param1_SET((float)3.2011585E38F, PH.base.pack) ;
    p76_param2_SET((float)2.9139874E38F, PH.base.pack) ;
    p76_param3_SET((float) -2.1620339E38F, PH.base.pack) ;
    p76_param4_SET((float) -5.7831047E37F, PH.base.pack) ;
    p76_param5_SET((float)1.5062858E38F, PH.base.pack) ;
    p76_param6_SET((float) -9.517601E37F, PH.base.pack) ;
    p76_param7_SET((float) -6.76329E35F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FOLLOW, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)154, &PH) ;
    p77_result_param2_SET((int32_t)968428476, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)19, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)24, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)3125116619L, PH.base.pack) ;
    p81_roll_SET((float)1.4103053E38F, PH.base.pack) ;
    p81_pitch_SET((float) -3.2226027E38F, PH.base.pack) ;
    p81_yaw_SET((float) -3.228729E38F, PH.base.pack) ;
    p81_thrust_SET((float)6.3761745E37F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)1576408056L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    {
        float  q [] =  {-2.7745642E36F, 8.4657023E37F, -2.8450817E38F, -2.6836745E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)2.75072E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)2.3151212E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -2.4216224E38F, PH.base.pack) ;
    p82_thrust_SET((float) -2.908585E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)1754721779L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    {
        float  q [] =  {-1.0373185E38F, -2.3125223E38F, 3.3062703E38F, -2.4828953E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -2.4562158E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)2.1893667E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)1.8098203E38F, PH.base.pack) ;
    p83_thrust_SET((float)2.8794464E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)239909899L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)39954, PH.base.pack) ;
    p84_x_SET((float)1.5679886E37F, PH.base.pack) ;
    p84_y_SET((float) -1.7230313E38F, PH.base.pack) ;
    p84_z_SET((float)1.5653717E38F, PH.base.pack) ;
    p84_vx_SET((float) -3.1740407E38F, PH.base.pack) ;
    p84_vy_SET((float)1.868397E38F, PH.base.pack) ;
    p84_vz_SET((float) -3.3542676E38F, PH.base.pack) ;
    p84_afx_SET((float) -2.4048658E38F, PH.base.pack) ;
    p84_afy_SET((float) -1.1408448E38F, PH.base.pack) ;
    p84_afz_SET((float) -1.5116113E38F, PH.base.pack) ;
    p84_yaw_SET((float)3.1734708E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -2.6463245E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1286638932L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)48650, PH.base.pack) ;
    p86_lat_int_SET((int32_t)697440873, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -834251814, PH.base.pack) ;
    p86_alt_SET((float)3.0312743E38F, PH.base.pack) ;
    p86_vx_SET((float)4.160277E37F, PH.base.pack) ;
    p86_vy_SET((float) -2.9664726E38F, PH.base.pack) ;
    p86_vz_SET((float)5.3555774E37F, PH.base.pack) ;
    p86_afx_SET((float) -3.245746E38F, PH.base.pack) ;
    p86_afy_SET((float) -1.6089106E38F, PH.base.pack) ;
    p86_afz_SET((float) -1.1278616E38F, PH.base.pack) ;
    p86_yaw_SET((float) -1.2333084E37F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -2.440278E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2275818910L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)60343, PH.base.pack) ;
    p87_lat_int_SET((int32_t)2067583429, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -176144364, PH.base.pack) ;
    p87_alt_SET((float)2.4599766E38F, PH.base.pack) ;
    p87_vx_SET((float) -2.7526948E38F, PH.base.pack) ;
    p87_vy_SET((float)2.3164341E38F, PH.base.pack) ;
    p87_vz_SET((float) -1.2772936E38F, PH.base.pack) ;
    p87_afx_SET((float) -3.427866E37F, PH.base.pack) ;
    p87_afy_SET((float) -9.705116E37F, PH.base.pack) ;
    p87_afz_SET((float) -3.0050258E38F, PH.base.pack) ;
    p87_yaw_SET((float)1.3495812E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)8.65595E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1158054511L, PH.base.pack) ;
    p89_x_SET((float) -2.3861385E38F, PH.base.pack) ;
    p89_y_SET((float)2.0333452E38F, PH.base.pack) ;
    p89_z_SET((float) -3.7283714E37F, PH.base.pack) ;
    p89_roll_SET((float)1.4337626E38F, PH.base.pack) ;
    p89_pitch_SET((float) -8.470648E37F, PH.base.pack) ;
    p89_yaw_SET((float)2.4528473E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)1400805650701399300L, PH.base.pack) ;
    p90_roll_SET((float) -5.154517E37F, PH.base.pack) ;
    p90_pitch_SET((float)6.314291E37F, PH.base.pack) ;
    p90_yaw_SET((float) -2.9480912E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -1.222795E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.2226309E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -2.169425E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -627918746, PH.base.pack) ;
    p90_lon_SET((int32_t) -1104667975, PH.base.pack) ;
    p90_alt_SET((int32_t)496785550, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)1349, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)17129, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)7268, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -12871, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)8294, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)14887, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)3933494429912318988L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -9.288576E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -8.814216E37F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -2.754135E38F, PH.base.pack) ;
    p91_throttle_SET((float) -2.4139818E38F, PH.base.pack) ;
    p91_aux1_SET((float) -9.616658E37F, PH.base.pack) ;
    p91_aux2_SET((float)8.672773E37F, PH.base.pack) ;
    p91_aux3_SET((float)1.9095811E36F, PH.base.pack) ;
    p91_aux4_SET((float) -5.8225367E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)7679525685489627686L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)53935, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)29295, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)177, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)22031, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)28000, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)7864, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)11972, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)65055, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)59359, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)38103, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)12337, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)60859, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)3828244662800304948L, PH.base.pack) ;
    {
        float  controls [] =  {-1.7348518E38F, -2.5262473E38F, -2.9212225E38F, 1.9695313E38F, -3.2748222E38F, 3.2211975E38F, -3.1740624E38F, -1.9259016E38F, -1.3902182E38F, 3.2170867E38F, -3.280897E38F, 1.3230428E38F, -2.4490452E37F, 1.6835233E38F, -3.1947935E38F, 4.4077007E36F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)4707684047341533450L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)6629573604572415440L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)9940, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)26387, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)7.552788E36F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -2.0375068E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p100_ground_distance_SET((float)2.5254543E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -1.5923947E38F, &PH) ;
    p100_flow_rate_y_SET((float) -2.2230004E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)7212122875797810103L, PH.base.pack) ;
    p101_x_SET((float)1.17183214E36F, PH.base.pack) ;
    p101_y_SET((float)1.9661863E38F, PH.base.pack) ;
    p101_z_SET((float) -1.8254236E38F, PH.base.pack) ;
    p101_roll_SET((float)9.845696E36F, PH.base.pack) ;
    p101_pitch_SET((float) -2.997911E38F, PH.base.pack) ;
    p101_yaw_SET((float) -1.9126604E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)7145665677522436438L, PH.base.pack) ;
    p102_x_SET((float)2.4038166E38F, PH.base.pack) ;
    p102_y_SET((float) -1.8427271E37F, PH.base.pack) ;
    p102_z_SET((float) -1.4220183E38F, PH.base.pack) ;
    p102_roll_SET((float)1.2559485E38F, PH.base.pack) ;
    p102_pitch_SET((float) -1.9559573E38F, PH.base.pack) ;
    p102_yaw_SET((float) -3.3180544E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)7251408170864547771L, PH.base.pack) ;
    p103_x_SET((float) -2.4610035E38F, PH.base.pack) ;
    p103_y_SET((float) -9.903871E37F, PH.base.pack) ;
    p103_z_SET((float) -6.735385E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)3947944570324980596L, PH.base.pack) ;
    p104_x_SET((float)2.2092114E38F, PH.base.pack) ;
    p104_y_SET((float) -5.1870707E36F, PH.base.pack) ;
    p104_z_SET((float)2.2209206E38F, PH.base.pack) ;
    p104_roll_SET((float)2.661594E38F, PH.base.pack) ;
    p104_pitch_SET((float)2.9677195E38F, PH.base.pack) ;
    p104_yaw_SET((float)2.4026573E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)1694933628841244816L, PH.base.pack) ;
    p105_xacc_SET((float)1.5126746E38F, PH.base.pack) ;
    p105_yacc_SET((float)5.423815E37F, PH.base.pack) ;
    p105_zacc_SET((float)9.690227E37F, PH.base.pack) ;
    p105_xgyro_SET((float)2.9028138E38F, PH.base.pack) ;
    p105_ygyro_SET((float)4.9795737E36F, PH.base.pack) ;
    p105_zgyro_SET((float)1.7084485E37F, PH.base.pack) ;
    p105_xmag_SET((float) -2.8584842E38F, PH.base.pack) ;
    p105_ymag_SET((float)3.033296E38F, PH.base.pack) ;
    p105_zmag_SET((float)1.9262016E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -2.0602645E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -2.577168E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -8.2020594E36F, PH.base.pack) ;
    p105_temperature_SET((float)1.9385002E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)25154, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)8606702039726730971L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)181017610L, PH.base.pack) ;
    p106_integrated_x_SET((float)9.265946E37F, PH.base.pack) ;
    p106_integrated_y_SET((float) -3.2937302E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.5784693E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)7.0133124E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)1.0655648E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -27255, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)4186688135L, PH.base.pack) ;
    p106_distance_SET((float)1.6661681E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)2517155554050387223L, PH.base.pack) ;
    p107_xacc_SET((float)9.620617E37F, PH.base.pack) ;
    p107_yacc_SET((float)2.7163574E38F, PH.base.pack) ;
    p107_zacc_SET((float) -3.1280946E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -9.571045E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -3.660781E37F, PH.base.pack) ;
    p107_zgyro_SET((float)1.7054207E38F, PH.base.pack) ;
    p107_xmag_SET((float) -1.7643305E38F, PH.base.pack) ;
    p107_ymag_SET((float) -1.9903842E38F, PH.base.pack) ;
    p107_zmag_SET((float) -1.0339852E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)6.7931956E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float)3.296782E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)1.8215078E38F, PH.base.pack) ;
    p107_temperature_SET((float) -3.1476367E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2845615856L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)2.5349156E37F, PH.base.pack) ;
    p108_q2_SET((float)9.276345E37F, PH.base.pack) ;
    p108_q3_SET((float) -2.580386E38F, PH.base.pack) ;
    p108_q4_SET((float) -1.328896E38F, PH.base.pack) ;
    p108_roll_SET((float) -1.6044862E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.6612422E38F, PH.base.pack) ;
    p108_yaw_SET((float) -2.2082149E38F, PH.base.pack) ;
    p108_xacc_SET((float)1.7383729E38F, PH.base.pack) ;
    p108_yacc_SET((float) -1.1346744E38F, PH.base.pack) ;
    p108_zacc_SET((float) -1.6807812E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -2.1537047E38F, PH.base.pack) ;
    p108_ygyro_SET((float)1.6109828E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.4223949E38F, PH.base.pack) ;
    p108_lat_SET((float)1.955279E38F, PH.base.pack) ;
    p108_lon_SET((float) -3.0052692E37F, PH.base.pack) ;
    p108_alt_SET((float)2.633754E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)3.1724333E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)2.3707492E38F, PH.base.pack) ;
    p108_vn_SET((float)1.4149842E37F, PH.base.pack) ;
    p108_ve_SET((float)6.4524186E37F, PH.base.pack) ;
    p108_vd_SET((float) -9.226867E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)59085, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)52339, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)182, (uint8_t)72, (uint8_t)94, (uint8_t)68, (uint8_t)126, (uint8_t)53, (uint8_t)91, (uint8_t)72, (uint8_t)186, (uint8_t)154, (uint8_t)234, (uint8_t)37, (uint8_t)20, (uint8_t)166, (uint8_t)127, (uint8_t)10, (uint8_t)22, (uint8_t)80, (uint8_t)170, (uint8_t)98, (uint8_t)52, (uint8_t)129, (uint8_t)197, (uint8_t)8, (uint8_t)181, (uint8_t)67, (uint8_t)162, (uint8_t)245, (uint8_t)15, (uint8_t)250, (uint8_t)12, (uint8_t)162, (uint8_t)135, (uint8_t)190, (uint8_t)47, (uint8_t)13, (uint8_t)184, (uint8_t)103, (uint8_t)171, (uint8_t)204, (uint8_t)190, (uint8_t)172, (uint8_t)117, (uint8_t)169, (uint8_t)141, (uint8_t)63, (uint8_t)200, (uint8_t)58, (uint8_t)167, (uint8_t)212, (uint8_t)122, (uint8_t)83, (uint8_t)66, (uint8_t)116, (uint8_t)40, (uint8_t)235, (uint8_t)123, (uint8_t)183, (uint8_t)67, (uint8_t)68, (uint8_t)119, (uint8_t)164, (uint8_t)126, (uint8_t)220, (uint8_t)34, (uint8_t)187, (uint8_t)171, (uint8_t)118, (uint8_t)175, (uint8_t)120, (uint8_t)139, (uint8_t)37, (uint8_t)98, (uint8_t)46, (uint8_t)219, (uint8_t)217, (uint8_t)6, (uint8_t)32, (uint8_t)83, (uint8_t)247, (uint8_t)4, (uint8_t)62, (uint8_t)44, (uint8_t)203, (uint8_t)156, (uint8_t)127, (uint8_t)217, (uint8_t)94, (uint8_t)170, (uint8_t)9, (uint8_t)136, (uint8_t)98, (uint8_t)78, (uint8_t)129, (uint8_t)45, (uint8_t)165, (uint8_t)128, (uint8_t)138, (uint8_t)8, (uint8_t)254, (uint8_t)178, (uint8_t)213, (uint8_t)248, (uint8_t)239, (uint8_t)86, (uint8_t)163, (uint8_t)162, (uint8_t)71, (uint8_t)224, (uint8_t)82, (uint8_t)163, (uint8_t)100, (uint8_t)246, (uint8_t)150, (uint8_t)86, (uint8_t)122, (uint8_t)96, (uint8_t)131, (uint8_t)235, (uint8_t)125, (uint8_t)199, (uint8_t)51, (uint8_t)123, (uint8_t)128, (uint8_t)228, (uint8_t)115, (uint8_t)42, (uint8_t)113, (uint8_t)11, (uint8_t)103, (uint8_t)67, (uint8_t)181, (uint8_t)142, (uint8_t)242, (uint8_t)130, (uint8_t)29, (uint8_t)123, (uint8_t)109, (uint8_t)130, (uint8_t)149, (uint8_t)123, (uint8_t)36, (uint8_t)31, (uint8_t)56, (uint8_t)193, (uint8_t)64, (uint8_t)31, (uint8_t)14, (uint8_t)220, (uint8_t)31, (uint8_t)208, (uint8_t)41, (uint8_t)42, (uint8_t)177, (uint8_t)195, (uint8_t)11, (uint8_t)156, (uint8_t)194, (uint8_t)119, (uint8_t)151, (uint8_t)185, (uint8_t)123, (uint8_t)39, (uint8_t)181, (uint8_t)113, (uint8_t)238, (uint8_t)161, (uint8_t)72, (uint8_t)181, (uint8_t)123, (uint8_t)187, (uint8_t)7, (uint8_t)94, (uint8_t)154, (uint8_t)60, (uint8_t)32, (uint8_t)23, (uint8_t)68, (uint8_t)1, (uint8_t)246, (uint8_t)42, (uint8_t)1, (uint8_t)5, (uint8_t)22, (uint8_t)99, (uint8_t)239, (uint8_t)58, (uint8_t)217, (uint8_t)247, (uint8_t)200, (uint8_t)15, (uint8_t)101, (uint8_t)81, (uint8_t)255, (uint8_t)90, (uint8_t)148, (uint8_t)226, (uint8_t)233, (uint8_t)50, (uint8_t)125, (uint8_t)236, (uint8_t)46, (uint8_t)166, (uint8_t)233, (uint8_t)247, (uint8_t)45, (uint8_t)96, (uint8_t)147, (uint8_t)167, (uint8_t)248, (uint8_t)227, (uint8_t)50, (uint8_t)191, (uint8_t)217, (uint8_t)89, (uint8_t)44, (uint8_t)137, (uint8_t)51, (uint8_t)174, (uint8_t)39, (uint8_t)4, (uint8_t)155, (uint8_t)184, (uint8_t)234, (uint8_t)168, (uint8_t)105, (uint8_t)224, (uint8_t)231, (uint8_t)242, (uint8_t)45, (uint8_t)208, (uint8_t)117, (uint8_t)145, (uint8_t)249, (uint8_t)157, (uint8_t)149, (uint8_t)5, (uint8_t)137, (uint8_t)62, (uint8_t)27, (uint8_t)200, (uint8_t)64, (uint8_t)227, (uint8_t)154, (uint8_t)46, (uint8_t)137, (uint8_t)44, (uint8_t)11, (uint8_t)107, (uint8_t)151, (uint8_t)224};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -2482168619184751559L, PH.base.pack) ;
    p111_ts1_SET((int64_t)8925046541841033784L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)6740066414556314760L, PH.base.pack) ;
    p112_seq_SET((uint32_t)3464252467L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)1651692737143457925L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p113_lat_SET((int32_t) -55249078, PH.base.pack) ;
    p113_lon_SET((int32_t) -246643717, PH.base.pack) ;
    p113_alt_SET((int32_t) -339085207, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)13284, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)37491, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)33855, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)306, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -26049, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)12098, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)51101, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)2258809083465580223L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)1022009849L, PH.base.pack) ;
    p114_integrated_x_SET((float)8.901387E37F, PH.base.pack) ;
    p114_integrated_y_SET((float) -3.0507908E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -1.3830071E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -3.1652088E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -1.5109155E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -16545, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)495682768L, PH.base.pack) ;
    p114_distance_SET((float) -2.9049187E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)19678690470434435L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {4.7046864E37F, 3.2296326E38F, 2.6455726E38F, -2.0685117E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)3.1340144E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -2.028491E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.6130283E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)919300502, PH.base.pack) ;
    p115_lon_SET((int32_t)1876612281, PH.base.pack) ;
    p115_alt_SET((int32_t) -6836275, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -30343, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)21575, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)2729, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)49594, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)51061, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -22354, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -15571, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)21691, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)3370165244L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)20739, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)9475, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)14434, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)30456, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -17167, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)6650, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)26546, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -12865, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)23030, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)9350, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)18835, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)39749, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)39665, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)30053, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)36372769L, PH.base.pack) ;
    p118_size_SET((uint32_t)409400889L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)60956, PH.base.pack) ;
    p119_ofs_SET((uint32_t)362459920L, PH.base.pack) ;
    p119_count_SET((uint32_t)925225278L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)17914, PH.base.pack) ;
    p120_ofs_SET((uint32_t)124973176L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)181, (uint8_t)110, (uint8_t)195, (uint8_t)231, (uint8_t)130, (uint8_t)119, (uint8_t)127, (uint8_t)40, (uint8_t)113, (uint8_t)10, (uint8_t)202, (uint8_t)25, (uint8_t)151, (uint8_t)145, (uint8_t)21, (uint8_t)9, (uint8_t)252, (uint8_t)183, (uint8_t)84, (uint8_t)224, (uint8_t)86, (uint8_t)163, (uint8_t)70, (uint8_t)226, (uint8_t)78, (uint8_t)101, (uint8_t)86, (uint8_t)198, (uint8_t)94, (uint8_t)189, (uint8_t)232, (uint8_t)127, (uint8_t)31, (uint8_t)73, (uint8_t)166, (uint8_t)188, (uint8_t)154, (uint8_t)139, (uint8_t)19, (uint8_t)161, (uint8_t)32, (uint8_t)228, (uint8_t)140, (uint8_t)118, (uint8_t)101, (uint8_t)64, (uint8_t)118, (uint8_t)196, (uint8_t)229, (uint8_t)224, (uint8_t)19, (uint8_t)142, (uint8_t)135, (uint8_t)211, (uint8_t)212, (uint8_t)172, (uint8_t)28, (uint8_t)51, (uint8_t)137, (uint8_t)125, (uint8_t)196, (uint8_t)41, (uint8_t)118, (uint8_t)47, (uint8_t)18, (uint8_t)134, (uint8_t)47, (uint8_t)234, (uint8_t)14, (uint8_t)92, (uint8_t)106, (uint8_t)100, (uint8_t)43, (uint8_t)127, (uint8_t)181, (uint8_t)120, (uint8_t)34, (uint8_t)154, (uint8_t)172, (uint8_t)96, (uint8_t)16, (uint8_t)213, (uint8_t)210, (uint8_t)73, (uint8_t)213, (uint8_t)192, (uint8_t)33, (uint8_t)135, (uint8_t)174, (uint8_t)138};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)82, (uint8_t)108, (uint8_t)17, (uint8_t)85, (uint8_t)9, (uint8_t)118, (uint8_t)207, (uint8_t)189, (uint8_t)138, (uint8_t)253, (uint8_t)21, (uint8_t)226, (uint8_t)62, (uint8_t)23, (uint8_t)63, (uint8_t)122, (uint8_t)49, (uint8_t)255, (uint8_t)174, (uint8_t)133, (uint8_t)79, (uint8_t)74, (uint8_t)167, (uint8_t)92, (uint8_t)125, (uint8_t)202, (uint8_t)198, (uint8_t)121, (uint8_t)255, (uint8_t)96, (uint8_t)184, (uint8_t)13, (uint8_t)162, (uint8_t)142, (uint8_t)56, (uint8_t)213, (uint8_t)166, (uint8_t)146, (uint8_t)68, (uint8_t)99, (uint8_t)100, (uint8_t)233, (uint8_t)5, (uint8_t)6, (uint8_t)187, (uint8_t)175, (uint8_t)124, (uint8_t)111, (uint8_t)152, (uint8_t)136, (uint8_t)28, (uint8_t)118, (uint8_t)120, (uint8_t)150, (uint8_t)194, (uint8_t)107, (uint8_t)231, (uint8_t)63, (uint8_t)7, (uint8_t)85, (uint8_t)46, (uint8_t)22, (uint8_t)163, (uint8_t)230, (uint8_t)126, (uint8_t)116, (uint8_t)58, (uint8_t)155, (uint8_t)208, (uint8_t)249, (uint8_t)107, (uint8_t)147, (uint8_t)75, (uint8_t)25, (uint8_t)43, (uint8_t)28, (uint8_t)226, (uint8_t)26, (uint8_t)188, (uint8_t)92, (uint8_t)239, (uint8_t)71, (uint8_t)52, (uint8_t)104, (uint8_t)170, (uint8_t)195, (uint8_t)99, (uint8_t)189, (uint8_t)79, (uint8_t)111, (uint8_t)137, (uint8_t)49, (uint8_t)163, (uint8_t)47, (uint8_t)231, (uint8_t)39, (uint8_t)151, (uint8_t)64, (uint8_t)170, (uint8_t)130, (uint8_t)148, (uint8_t)219, (uint8_t)69, (uint8_t)162, (uint8_t)150, (uint8_t)15, (uint8_t)91, (uint8_t)68, (uint8_t)67, (uint8_t)50};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)7078426445815867215L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p124_lat_SET((int32_t)126012004, PH.base.pack) ;
    p124_lon_SET((int32_t) -523639563, PH.base.pack) ;
    p124_alt_SET((int32_t)2078289198, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)5737, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)54426, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)24763, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)12238, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2088772989L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)10562, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)13386, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)28109, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1556868770L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)127, (uint8_t)215, (uint8_t)6, (uint8_t)135, (uint8_t)243, (uint8_t)185, (uint8_t)235, (uint8_t)38, (uint8_t)223, (uint8_t)144, (uint8_t)215, (uint8_t)105, (uint8_t)138, (uint8_t)168, (uint8_t)119, (uint8_t)159, (uint8_t)206, (uint8_t)59, (uint8_t)21, (uint8_t)109, (uint8_t)23, (uint8_t)21, (uint8_t)78, (uint8_t)205, (uint8_t)224, (uint8_t)73, (uint8_t)213, (uint8_t)234, (uint8_t)28, (uint8_t)108, (uint8_t)93, (uint8_t)109, (uint8_t)111, (uint8_t)80, (uint8_t)163, (uint8_t)6, (uint8_t)65, (uint8_t)89, (uint8_t)163, (uint8_t)97, (uint8_t)243, (uint8_t)245, (uint8_t)179, (uint8_t)191, (uint8_t)166, (uint8_t)149, (uint8_t)124, (uint8_t)42, (uint8_t)205, (uint8_t)29, (uint8_t)178, (uint8_t)88, (uint8_t)196, (uint8_t)26, (uint8_t)195, (uint8_t)126, (uint8_t)171, (uint8_t)126, (uint8_t)4, (uint8_t)30, (uint8_t)4, (uint8_t)0, (uint8_t)53, (uint8_t)139, (uint8_t)119, (uint8_t)101, (uint8_t)224, (uint8_t)120, (uint8_t)65, (uint8_t)184};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)2935657946L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)51403, PH.base.pack) ;
    p127_tow_SET((uint32_t)2429634138L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)1844909697, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -556436174, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)271239457, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)1762071086L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)1496640270, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)1360711729L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)28136, PH.base.pack) ;
    p128_tow_SET((uint32_t)2599454837L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)1801906618, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -1803550712, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)1153160828, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)996214460L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -970978553, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)352960897L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)1742, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -7708, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -6461, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)2995, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)21406, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)2606, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)24838, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)5115, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)27713, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p130_size_SET((uint32_t)1030981683L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)39567, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)40823, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)39695, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)31582, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)63, (uint8_t)46, (uint8_t)104, (uint8_t)144, (uint8_t)226, (uint8_t)203, (uint8_t)238, (uint8_t)83, (uint8_t)26, (uint8_t)172, (uint8_t)130, (uint8_t)87, (uint8_t)140, (uint8_t)188, (uint8_t)105, (uint8_t)10, (uint8_t)127, (uint8_t)46, (uint8_t)203, (uint8_t)118, (uint8_t)146, (uint8_t)68, (uint8_t)140, (uint8_t)182, (uint8_t)30, (uint8_t)162, (uint8_t)7, (uint8_t)18, (uint8_t)13, (uint8_t)99, (uint8_t)232, (uint8_t)139, (uint8_t)234, (uint8_t)143, (uint8_t)166, (uint8_t)144, (uint8_t)124, (uint8_t)209, (uint8_t)53, (uint8_t)146, (uint8_t)106, (uint8_t)245, (uint8_t)126, (uint8_t)128, (uint8_t)166, (uint8_t)12, (uint8_t)98, (uint8_t)190, (uint8_t)181, (uint8_t)174, (uint8_t)115, (uint8_t)253, (uint8_t)197, (uint8_t)206, (uint8_t)94, (uint8_t)229, (uint8_t)138, (uint8_t)183, (uint8_t)194, (uint8_t)193, (uint8_t)65, (uint8_t)67, (uint8_t)105, (uint8_t)209, (uint8_t)245, (uint8_t)76, (uint8_t)22, (uint8_t)159, (uint8_t)57, (uint8_t)143, (uint8_t)138, (uint8_t)134, (uint8_t)57, (uint8_t)105, (uint8_t)38, (uint8_t)181, (uint8_t)5, (uint8_t)127, (uint8_t)101, (uint8_t)64, (uint8_t)171, (uint8_t)62, (uint8_t)48, (uint8_t)21, (uint8_t)10, (uint8_t)208, (uint8_t)100, (uint8_t)82, (uint8_t)139, (uint8_t)211, (uint8_t)225, (uint8_t)235, (uint8_t)148, (uint8_t)214, (uint8_t)102, (uint8_t)87, (uint8_t)132, (uint8_t)244, (uint8_t)215, (uint8_t)64, (uint8_t)125, (uint8_t)223, (uint8_t)7, (uint8_t)18, (uint8_t)29, (uint8_t)179, (uint8_t)232, (uint8_t)183, (uint8_t)214, (uint8_t)142, (uint8_t)61, (uint8_t)94, (uint8_t)232, (uint8_t)219, (uint8_t)3, (uint8_t)78, (uint8_t)81, (uint8_t)73, (uint8_t)191, (uint8_t)196, (uint8_t)215, (uint8_t)14, (uint8_t)136, (uint8_t)248, (uint8_t)148, (uint8_t)184, (uint8_t)41, (uint8_t)76, (uint8_t)91, (uint8_t)0, (uint8_t)143, (uint8_t)219, (uint8_t)233, (uint8_t)188, (uint8_t)146, (uint8_t)177, (uint8_t)87, (uint8_t)8, (uint8_t)203, (uint8_t)169, (uint8_t)240, (uint8_t)138, (uint8_t)113, (uint8_t)244, (uint8_t)143, (uint8_t)16, (uint8_t)61, (uint8_t)155, (uint8_t)126, (uint8_t)181, (uint8_t)230, (uint8_t)98, (uint8_t)175, (uint8_t)63, (uint8_t)86, (uint8_t)53, (uint8_t)93, (uint8_t)55, (uint8_t)108, (uint8_t)175, (uint8_t)150, (uint8_t)1, (uint8_t)53, (uint8_t)38, (uint8_t)135, (uint8_t)153, (uint8_t)253, (uint8_t)103, (uint8_t)138, (uint8_t)225, (uint8_t)31, (uint8_t)179, (uint8_t)240, (uint8_t)115, (uint8_t)0, (uint8_t)48, (uint8_t)24, (uint8_t)143, (uint8_t)245, (uint8_t)1, (uint8_t)44, (uint8_t)246, (uint8_t)216, (uint8_t)191, (uint8_t)253, (uint8_t)102, (uint8_t)57, (uint8_t)24, (uint8_t)196, (uint8_t)152, (uint8_t)219, (uint8_t)102, (uint8_t)140, (uint8_t)132, (uint8_t)75, (uint8_t)74, (uint8_t)206, (uint8_t)243, (uint8_t)154, (uint8_t)189, (uint8_t)196, (uint8_t)13, (uint8_t)48, (uint8_t)28, (uint8_t)204, (uint8_t)104, (uint8_t)165, (uint8_t)221, (uint8_t)149, (uint8_t)14, (uint8_t)144, (uint8_t)98, (uint8_t)3, (uint8_t)3, (uint8_t)55, (uint8_t)179, (uint8_t)206, (uint8_t)120, (uint8_t)48, (uint8_t)222, (uint8_t)202, (uint8_t)54, (uint8_t)178, (uint8_t)64, (uint8_t)179, (uint8_t)156, (uint8_t)23, (uint8_t)235, (uint8_t)97, (uint8_t)14, (uint8_t)185, (uint8_t)192, (uint8_t)104, (uint8_t)59, (uint8_t)102, (uint8_t)181, (uint8_t)8, (uint8_t)55, (uint8_t)61, (uint8_t)209, (uint8_t)3, (uint8_t)164, (uint8_t)139, (uint8_t)109, (uint8_t)189, (uint8_t)123, (uint8_t)49, (uint8_t)220, (uint8_t)114, (uint8_t)222, (uint8_t)46, (uint8_t)118, (uint8_t)26};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)3562296286L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)31454, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)19361, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)17893, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_135, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1592700530, PH.base.pack) ;
    p133_lon_SET((int32_t) -2081970674, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)51368, PH.base.pack) ;
    p133_mask_SET((uint64_t)3584884256583515753L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)2008279172, PH.base.pack) ;
    p134_lon_SET((int32_t) -1976977158, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)22062, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -17246, (int16_t)16434, (int16_t) -30023, (int16_t)3792, (int16_t) -11875, (int16_t)15451, (int16_t)19606, (int16_t)28436, (int16_t) -19816, (int16_t)18108, (int16_t)9282, (int16_t) -12675, (int16_t)2771, (int16_t) -1313, (int16_t) -4191, (int16_t) -18015};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1660332131, PH.base.pack) ;
    p135_lon_SET((int32_t) -1837388606, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)1993418472, PH.base.pack) ;
    p136_lon_SET((int32_t) -2123196796, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)26166, PH.base.pack) ;
    p136_terrain_height_SET((float)2.6509698E37F, PH.base.pack) ;
    p136_current_height_SET((float)6.610106E37F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)34020, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)38473, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)1514830026L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.7896108E37F, PH.base.pack) ;
    p137_press_diff_SET((float)3.261394E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -23964, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)2816406115575446026L, PH.base.pack) ;
    {
        float  q [] =  {1.9975441E38F, 5.7099146E37F, 2.9352232E37F, 2.1868034E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -1.1251689E38F, PH.base.pack) ;
    p138_y_SET((float) -2.0775848E38F, PH.base.pack) ;
    p138_z_SET((float)3.3119327E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)3517847566116293633L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    {
        float  controls [] =  {-1.9322636E38F, 2.2265202E38F, -1.008703E38F, 2.4345372E38F, 6.5078804E37F, -1.4275112E38F, -2.8317175E38F, -2.0817839E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)7476825916750306042L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    {
        float  controls [] =  {-1.4146674E38F, -2.2928576E38F, 1.1072715E38F, 1.7262016E38F, -1.510119E38F, -2.561289E38F, 2.5245154E38F, 1.074759E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)7758686716020483857L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -1.7235558E37F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -1.990799E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.4348442E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.5680415E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -1.3572434E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float) -9.249569E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)12, (uint8_t)75, (uint8_t)222, (uint8_t)136, (uint8_t)200, (uint8_t)111, (uint8_t)211, (uint8_t)54, (uint8_t)66, (uint8_t)246, (uint8_t)214, (uint8_t)155, (uint8_t)89, (uint8_t)12, (uint8_t)10, (uint8_t)237, (uint8_t)142, (uint8_t)155, (uint8_t)31, (uint8_t)2, (uint8_t)177, (uint8_t)193, (uint8_t)181, (uint8_t)164, (uint8_t)170, (uint8_t)52, (uint8_t)32, (uint8_t)5, (uint8_t)144, (uint8_t)176, (uint8_t)16, (uint8_t)247, (uint8_t)151, (uint8_t)91, (uint8_t)7, (uint8_t)101, (uint8_t)51, (uint8_t)51, (uint8_t)28, (uint8_t)179, (uint8_t)144, (uint8_t)75, (uint8_t)207, (uint8_t)79, (uint8_t)183, (uint8_t)237, (uint8_t)189, (uint8_t)203, (uint8_t)45, (uint8_t)67, (uint8_t)8, (uint8_t)197, (uint8_t)32, (uint8_t)125, (uint8_t)66, (uint8_t)234, (uint8_t)44, (uint8_t)149, (uint8_t)81, (uint8_t)55, (uint8_t)135, (uint8_t)11, (uint8_t)99, (uint8_t)242, (uint8_t)214, (uint8_t)138, (uint8_t)222, (uint8_t)169, (uint8_t)81, (uint8_t)160, (uint8_t)137, (uint8_t)8, (uint8_t)155, (uint8_t)189, (uint8_t)30, (uint8_t)151, (uint8_t)223, (uint8_t)123, (uint8_t)223, (uint8_t)194, (uint8_t)196, (uint8_t)142, (uint8_t)33, (uint8_t)15, (uint8_t)205, (uint8_t)161, (uint8_t)71, (uint8_t)52, (uint8_t)138, (uint8_t)251, (uint8_t)144, (uint8_t)106, (uint8_t)227, (uint8_t)89, (uint8_t)56, (uint8_t)128, (uint8_t)55, (uint8_t)21, (uint8_t)58, (uint8_t)146, (uint8_t)64, (uint8_t)228, (uint8_t)77, (uint8_t)112, (uint8_t)248, (uint8_t)44, (uint8_t)116, (uint8_t)127, (uint8_t)237, (uint8_t)29, (uint8_t)141, (uint8_t)100, (uint8_t)128, (uint8_t)110, (uint8_t)178, (uint8_t)193, (uint8_t)62, (uint8_t)20, (uint8_t)232, (uint8_t)153};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)174, (uint8_t)98, (uint8_t)252, (uint8_t)42, (uint8_t)136, (uint8_t)131, (uint8_t)80, (uint8_t)73, (uint8_t)110, (uint8_t)238, (uint8_t)95, (uint8_t)107, (uint8_t)87, (uint8_t)60, (uint8_t)57, (uint8_t)126, (uint8_t)111, (uint8_t)58, (uint8_t)233, (uint8_t)250, (uint8_t)31, (uint8_t)245, (uint8_t)49, (uint8_t)20, (uint8_t)53, (uint8_t)55, (uint8_t)61, (uint8_t)176, (uint8_t)16, (uint8_t)17, (uint8_t)216, (uint8_t)146, (uint8_t)132, (uint8_t)116, (uint8_t)164, (uint8_t)50, (uint8_t)178, (uint8_t)173, (uint8_t)31, (uint8_t)111, (uint8_t)198, (uint8_t)83, (uint8_t)25, (uint8_t)31, (uint8_t)126, (uint8_t)70, (uint8_t)94, (uint8_t)70, (uint8_t)48, (uint8_t)142, (uint8_t)153, (uint8_t)212, (uint8_t)253, (uint8_t)232, (uint8_t)184, (uint8_t)203, (uint8_t)207, (uint8_t)171, (uint8_t)42, (uint8_t)27, (uint8_t)221, (uint8_t)175, (uint8_t)124, (uint8_t)6, (uint8_t)193, (uint8_t)48, (uint8_t)170, (uint8_t)252, (uint8_t)103, (uint8_t)251, (uint8_t)164, (uint8_t)200, (uint8_t)171, (uint8_t)41, (uint8_t)230, (uint8_t)156, (uint8_t)133, (uint8_t)234, (uint8_t)28, (uint8_t)63, (uint8_t)202, (uint8_t)203, (uint8_t)10, (uint8_t)49, (uint8_t)81, (uint8_t)38, (uint8_t)17, (uint8_t)129, (uint8_t)154, (uint8_t)153, (uint8_t)240, (uint8_t)195, (uint8_t)40, (uint8_t)142, (uint8_t)237, (uint8_t)173, (uint8_t)15, (uint8_t)222, (uint8_t)67, (uint8_t)78, (uint8_t)241, (uint8_t)194, (uint8_t)37, (uint8_t)136, (uint8_t)189, (uint8_t)22, (uint8_t)38, (uint8_t)77, (uint8_t)133, (uint8_t)41, (uint8_t)121, (uint8_t)169, (uint8_t)114, (uint8_t)25, (uint8_t)10, (uint8_t)218, (uint8_t)190, (uint8_t)104, (uint8_t)248, (uint8_t)175};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)621054031L, PH.base.pack) ;
    p143_press_abs_SET((float) -1.0928285E38F, PH.base.pack) ;
    p143_press_diff_SET((float)1.1136147E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)8337, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)5002000076939897050L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p144_lat_SET((int32_t) -2076733826, PH.base.pack) ;
    p144_lon_SET((int32_t) -1738363961, PH.base.pack) ;
    p144_alt_SET((float) -2.6876627E38F, PH.base.pack) ;
    {
        float  vel [] =  {4.944843E37F, -2.8232974E38F, -9.561812E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-1.01383336E37F, 1.4969613E35F, 1.3103202E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-2.2499382E38F, -2.5338017E38F, 2.2673396E37F, 1.1304593E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {-2.5936561E38F, -6.039901E37F, -1.3105162E37F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-1.8833371E38F, -1.164947E38F, -3.3693648E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)8351343641461288340L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)1431851634104206918L, PH.base.pack) ;
    p146_x_acc_SET((float)2.1239656E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -2.557482E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -3.1912341E38F, PH.base.pack) ;
    p146_x_vel_SET((float) -2.2484584E38F, PH.base.pack) ;
    p146_y_vel_SET((float)2.5384427E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -1.6617065E37F, PH.base.pack) ;
    p146_x_pos_SET((float)3.021833E38F, PH.base.pack) ;
    p146_y_pos_SET((float)2.169623E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -6.9823655E37F, PH.base.pack) ;
    p146_airspeed_SET((float)2.7802853E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {-2.9248204E36F, 4.6658073E36F, 3.1385569E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.9558934E38F, -2.6294124E38F, 1.6565052E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-4.0984062E37F, 2.1906167E38F, 1.3917678E37F, 1.8288298E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -4.4015866E37F, PH.base.pack) ;
    p146_pitch_rate_SET((float)1.1436455E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)1.2520586E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t)28148, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)27794, (uint16_t)62468, (uint16_t)1498, (uint16_t)29510, (uint16_t)21215, (uint16_t)32889, (uint16_t)39039, (uint16_t)55887, (uint16_t)24535, (uint16_t)52188};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -28006, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)155121771, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -479307035, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)119, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)2335836062L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)2221871710L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)453222534L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)3757435938L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)29, (uint8_t)217, (uint8_t)99, (uint8_t)51, (uint8_t)16, (uint8_t)180, (uint8_t)232, (uint8_t)184};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)69, (uint8_t)107, (uint8_t)4, (uint8_t)169, (uint8_t)181, (uint8_t)33, (uint8_t)164, (uint8_t)201};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)155, (uint8_t)167, (uint8_t)130, (uint8_t)228, (uint8_t)162, (uint8_t)152, (uint8_t)127, (uint8_t)163};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)49191, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)25249, PH.base.pack) ;
    p148_uid_SET((uint64_t)4821502821506087471L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)212, (uint8_t)202, (uint8_t)124, (uint8_t)62, (uint8_t)64, (uint8_t)220, (uint8_t)143, (uint8_t)18, (uint8_t)82, (uint8_t)38, (uint8_t)33, (uint8_t)215, (uint8_t)90, (uint8_t)253, (uint8_t)122, (uint8_t)203, (uint8_t)36, (uint8_t)122};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)3485676440146618510L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p149_angle_x_SET((float)2.9410607E38F, PH.base.pack) ;
    p149_angle_y_SET((float)3.396571E37F, PH.base.pack) ;
    p149_distance_SET((float)1.9955295E38F, PH.base.pack) ;
    p149_size_x_SET((float) -2.7901489E38F, PH.base.pack) ;
    p149_size_y_SET((float)1.4073315E37F, PH.base.pack) ;
    p149_x_SET((float) -1.211777E38F, &PH) ;
    p149_y_SET((float)6.8550374E36F, &PH) ;
    p149_z_SET((float) -1.2310861E38F, &PH) ;
    {
        float  q [] =  {-3.1770097E38F, -1.487532E38F, -3.3074006E38F, 2.5758796E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)79, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENS_POWER_201(), &PH);
    p201_adc121_vspb_volt_SET((float) -3.7891793E37F, PH.base.pack) ;
    p201_adc121_cspb_amp_SET((float)3.022148E38F, PH.base.pack) ;
    p201_adc121_cs1_amp_SET((float)2.1674732E38F, PH.base.pack) ;
    p201_adc121_cs2_amp_SET((float)2.0996377E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENS_MPPT_202(), &PH);
    p202_mppt_timestamp_SET((uint64_t)4006046460820775527L, PH.base.pack) ;
    p202_mppt1_volt_SET((float)9.407964E36F, PH.base.pack) ;
    p202_mppt1_amp_SET((float) -5.541292E37F, PH.base.pack) ;
    p202_mppt1_pwm_SET((uint16_t)(uint16_t)27760, PH.base.pack) ;
    p202_mppt1_status_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p202_mppt2_volt_SET((float)2.0997099E38F, PH.base.pack) ;
    p202_mppt2_amp_SET((float)3.105406E38F, PH.base.pack) ;
    p202_mppt2_pwm_SET((uint16_t)(uint16_t)29385, PH.base.pack) ;
    p202_mppt2_status_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p202_mppt3_volt_SET((float)3.3699566E38F, PH.base.pack) ;
    p202_mppt3_amp_SET((float)1.1816162E38F, PH.base.pack) ;
    p202_mppt3_pwm_SET((uint16_t)(uint16_t)12235, PH.base.pack) ;
    p202_mppt3_status_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ASLCTRL_DATA_203(), &PH);
    p203_timestamp_SET((uint64_t)5014066190525291176L, PH.base.pack) ;
    p203_aslctrl_mode_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p203_h_SET((float)3.2547893E38F, PH.base.pack) ;
    p203_hRef_SET((float)2.7392023E38F, PH.base.pack) ;
    p203_hRef_t_SET((float)4.6479373E37F, PH.base.pack) ;
    p203_PitchAngle_SET((float)1.0246162E38F, PH.base.pack) ;
    p203_PitchAngleRef_SET((float)3.121793E38F, PH.base.pack) ;
    p203_q_SET((float) -3.2498758E38F, PH.base.pack) ;
    p203_qRef_SET((float) -2.8144847E38F, PH.base.pack) ;
    p203_uElev_SET((float)1.6691515E38F, PH.base.pack) ;
    p203_uThrot_SET((float)1.7162553E38F, PH.base.pack) ;
    p203_uThrot2_SET((float)1.9259859E37F, PH.base.pack) ;
    p203_nZ_SET((float) -3.1703404E38F, PH.base.pack) ;
    p203_AirspeedRef_SET((float) -3.3738677E38F, PH.base.pack) ;
    p203_SpoilersEngaged_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p203_YawAngle_SET((float) -1.86742E38F, PH.base.pack) ;
    p203_YawAngleRef_SET((float)6.5513765E37F, PH.base.pack) ;
    p203_RollAngle_SET((float) -2.906286E38F, PH.base.pack) ;
    p203_RollAngleRef_SET((float)3.2633669E38F, PH.base.pack) ;
    p203_p_SET((float) -3.08349E38F, PH.base.pack) ;
    p203_pRef_SET((float) -2.383468E38F, PH.base.pack) ;
    p203_r_SET((float) -2.9289645E38F, PH.base.pack) ;
    p203_rRef_SET((float)1.7406502E38F, PH.base.pack) ;
    p203_uAil_SET((float)1.4592119E38F, PH.base.pack) ;
    p203_uRud_SET((float) -1.411964E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ASLCTRL_DEBUG_204(), &PH);
    p204_i32_1_SET((uint32_t)3672216316L, PH.base.pack) ;
    p204_i8_1_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p204_i8_2_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p204_f_1_SET((float) -2.5717671E38F, PH.base.pack) ;
    p204_f_2_SET((float)1.7826642E38F, PH.base.pack) ;
    p204_f_3_SET((float)1.7562005E38F, PH.base.pack) ;
    p204_f_4_SET((float) -1.9811498E37F, PH.base.pack) ;
    p204_f_5_SET((float) -1.0590895E38F, PH.base.pack) ;
    p204_f_6_SET((float) -1.4570105E37F, PH.base.pack) ;
    p204_f_7_SET((float)1.8837649E38F, PH.base.pack) ;
    p204_f_8_SET((float)7.4563817E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ASLUAV_STATUS_205(), &PH);
    p205_LED_status_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p205_SATCOM_status_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    {
        uint8_t  Servo_status [] =  {(uint8_t)217, (uint8_t)202, (uint8_t)187, (uint8_t)170, (uint8_t)102, (uint8_t)160, (uint8_t)255, (uint8_t)171};
        p205_Servo_status_SET(&Servo_status, 0, &PH.base.pack) ;
    }
    p205_Motor_rpm_SET((float) -3.2581456E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EKF_EXT_206(), &PH);
    p206_timestamp_SET((uint64_t)2164539029466802242L, PH.base.pack) ;
    p206_Windspeed_SET((float) -1.76518E38F, PH.base.pack) ;
    p206_WindDir_SET((float) -2.2416E38F, PH.base.pack) ;
    p206_WindZ_SET((float) -1.1936422E38F, PH.base.pack) ;
    p206_Airspeed_SET((float) -2.5706435E38F, PH.base.pack) ;
    p206_beta_SET((float) -1.6321222E38F, PH.base.pack) ;
    p206_alpha_SET((float)8.000619E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ASL_OBCTRL_207(), &PH);
    p207_timestamp_SET((uint64_t)7586747163119959589L, PH.base.pack) ;
    p207_uElev_SET((float)4.050924E37F, PH.base.pack) ;
    p207_uThrot_SET((float)6.2406525E37F, PH.base.pack) ;
    p207_uThrot2_SET((float)3.1740334E38F, PH.base.pack) ;
    p207_uAilL_SET((float)4.829731E37F, PH.base.pack) ;
    p207_uAilR_SET((float)3.3473568E38F, PH.base.pack) ;
    p207_uRud_SET((float)2.4376718E38F, PH.base.pack) ;
    p207_obctrl_status_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENS_ATMOS_208(), &PH);
    p208_TempAmbient_SET((float) -1.0962562E38F, PH.base.pack) ;
    p208_Humidity_SET((float) -1.1368058E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENS_BATMON_209(), &PH);
    p209_temperature_SET((float)3.760687E37F, PH.base.pack) ;
    p209_voltage_SET((uint16_t)(uint16_t)2198, PH.base.pack) ;
    p209_current_SET((int16_t)(int16_t) -8715, PH.base.pack) ;
    p209_SoC_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p209_batterystatus_SET((uint16_t)(uint16_t)3004, PH.base.pack) ;
    p209_serialnumber_SET((uint16_t)(uint16_t)21716, PH.base.pack) ;
    p209_hostfetcontrol_SET((uint16_t)(uint16_t)24075, PH.base.pack) ;
    p209_cellvoltage1_SET((uint16_t)(uint16_t)35506, PH.base.pack) ;
    p209_cellvoltage2_SET((uint16_t)(uint16_t)12525, PH.base.pack) ;
    p209_cellvoltage3_SET((uint16_t)(uint16_t)36157, PH.base.pack) ;
    p209_cellvoltage4_SET((uint16_t)(uint16_t)37218, PH.base.pack) ;
    p209_cellvoltage5_SET((uint16_t)(uint16_t)47643, PH.base.pack) ;
    p209_cellvoltage6_SET((uint16_t)(uint16_t)36033, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FW_SOARING_DATA_210(), &PH);
    p210_timestamp_SET((uint64_t)1562468932328501743L, PH.base.pack) ;
    p210_timestampModeChanged_SET((uint64_t)7918071470073117564L, PH.base.pack) ;
    p210_xW_SET((float) -8.3595163E37F, PH.base.pack) ;
    p210_xR_SET((float)2.908655E38F, PH.base.pack) ;
    p210_xLat_SET((float) -1.5344975E38F, PH.base.pack) ;
    p210_xLon_SET((float) -8.518281E37F, PH.base.pack) ;
    p210_VarW_SET((float) -3.3214594E38F, PH.base.pack) ;
    p210_VarR_SET((float)2.599027E38F, PH.base.pack) ;
    p210_VarLat_SET((float) -8.2559485E36F, PH.base.pack) ;
    p210_VarLon_SET((float)1.2365774E38F, PH.base.pack) ;
    p210_LoiterRadius_SET((float)1.71034E38F, PH.base.pack) ;
    p210_LoiterDirection_SET((float)6.9902463E37F, PH.base.pack) ;
    p210_DistToSoarPoint_SET((float) -5.031418E37F, PH.base.pack) ;
    p210_vSinkExp_SET((float) -6.693885E37F, PH.base.pack) ;
    p210_z1_LocalUpdraftSpeed_SET((float)3.1159387E38F, PH.base.pack) ;
    p210_z2_DeltaRoll_SET((float) -7.588455E37F, PH.base.pack) ;
    p210_z1_exp_SET((float)1.9604397E38F, PH.base.pack) ;
    p210_z2_exp_SET((float)2.4282392E37F, PH.base.pack) ;
    p210_ThermalGSNorth_SET((float)2.3778338E38F, PH.base.pack) ;
    p210_ThermalGSEast_SET((float)1.1287604E38F, PH.base.pack) ;
    p210_TSE_dot_SET((float) -3.4006731E38F, PH.base.pack) ;
    p210_DebugVar1_SET((float)1.9603725E37F, PH.base.pack) ;
    p210_DebugVar2_SET((float) -9.80025E37F, PH.base.pack) ;
    p210_ControlMode_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p210_valid_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENSORPOD_STATUS_211(), &PH);
    p211_timestamp_SET((uint64_t)3141162134237631990L, PH.base.pack) ;
    p211_visensor_rate_1_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p211_visensor_rate_2_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p211_visensor_rate_3_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p211_visensor_rate_4_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p211_recording_nodes_count_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p211_cpu_temp_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p211_free_space_SET((uint16_t)(uint16_t)23064, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENS_POWER_BOARD_212(), &PH);
    p212_timestamp_SET((uint64_t)7671953688006562498L, PH.base.pack) ;
    p212_pwr_brd_status_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p212_pwr_brd_led_status_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p212_pwr_brd_system_volt_SET((float)1.9085021E38F, PH.base.pack) ;
    p212_pwr_brd_servo_volt_SET((float) -1.1745512E38F, PH.base.pack) ;
    p212_pwr_brd_mot_l_amp_SET((float) -9.358171E36F, PH.base.pack) ;
    p212_pwr_brd_mot_r_amp_SET((float)5.6109983E37F, PH.base.pack) ;
    p212_pwr_brd_servo_1_amp_SET((float)1.4576402E37F, PH.base.pack) ;
    p212_pwr_brd_servo_2_amp_SET((float) -2.1251075E38F, PH.base.pack) ;
    p212_pwr_brd_servo_3_amp_SET((float) -8.543342E37F, PH.base.pack) ;
    p212_pwr_brd_servo_4_amp_SET((float) -1.8218453E38F, PH.base.pack) ;
    p212_pwr_brd_aux_amp_SET((float) -9.444722E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)3332582287583770901L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
    p230_vel_ratio_SET((float) -1.816742E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -1.4350132E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -7.584295E37F, PH.base.pack) ;
    p230_mag_ratio_SET((float)1.6390156E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -3.3053856E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -7.2961314E37F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -1.0552218E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)1.9660636E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)1980308209436197494L, PH.base.pack) ;
    p231_wind_x_SET((float) -2.9647837E38F, PH.base.pack) ;
    p231_wind_y_SET((float) -1.4144963E38F, PH.base.pack) ;
    p231_wind_z_SET((float)1.1206654E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -4.32213E37F, PH.base.pack) ;
    p231_var_vert_SET((float)3.2311104E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -8.4991034E37F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float) -7.2260835E37F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -2.440249E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)7949789212644145216L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)1464292866L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)32712, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p232_lat_SET((int32_t)1539421264, PH.base.pack) ;
    p232_lon_SET((int32_t) -2092810519, PH.base.pack) ;
    p232_alt_SET((float)1.4006827E38F, PH.base.pack) ;
    p232_hdop_SET((float) -1.2053307E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.3730567E38F, PH.base.pack) ;
    p232_vn_SET((float) -2.3288703E38F, PH.base.pack) ;
    p232_ve_SET((float)1.2616061E38F, PH.base.pack) ;
    p232_vd_SET((float)3.1159154E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -3.569991E37F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -3.282087E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -5.943728E37F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)12, (uint8_t)227, (uint8_t)248, (uint8_t)114, (uint8_t)176, (uint8_t)164, (uint8_t)7, (uint8_t)209, (uint8_t)205, (uint8_t)33, (uint8_t)199, (uint8_t)19, (uint8_t)61, (uint8_t)127, (uint8_t)224, (uint8_t)123, (uint8_t)144, (uint8_t)69, (uint8_t)109, (uint8_t)202, (uint8_t)5, (uint8_t)42, (uint8_t)142, (uint8_t)137, (uint8_t)57, (uint8_t)226, (uint8_t)94, (uint8_t)224, (uint8_t)174, (uint8_t)237, (uint8_t)171, (uint8_t)20, (uint8_t)31, (uint8_t)22, (uint8_t)162, (uint8_t)9, (uint8_t)214, (uint8_t)7, (uint8_t)166, (uint8_t)31, (uint8_t)241, (uint8_t)46, (uint8_t)239, (uint8_t)26, (uint8_t)124, (uint8_t)111, (uint8_t)87, (uint8_t)209, (uint8_t)93, (uint8_t)94, (uint8_t)81, (uint8_t)46, (uint8_t)227, (uint8_t)92, (uint8_t)207, (uint8_t)96, (uint8_t)80, (uint8_t)75, (uint8_t)222, (uint8_t)246, (uint8_t)84, (uint8_t)214, (uint8_t)132, (uint8_t)144, (uint8_t)58, (uint8_t)20, (uint8_t)94, (uint8_t)164, (uint8_t)167, (uint8_t)150, (uint8_t)200, (uint8_t)77, (uint8_t)183, (uint8_t)213, (uint8_t)240, (uint8_t)109, (uint8_t)47, (uint8_t)94, (uint8_t)210, (uint8_t)247, (uint8_t)219, (uint8_t)79, (uint8_t)188, (uint8_t)171, (uint8_t)37, (uint8_t)91, (uint8_t)173, (uint8_t)227, (uint8_t)223, (uint8_t)62, (uint8_t)34, (uint8_t)161, (uint8_t)233, (uint8_t)209, (uint8_t)245, (uint8_t)74, (uint8_t)145, (uint8_t)220, (uint8_t)176, (uint8_t)193, (uint8_t)194, (uint8_t)159, (uint8_t)159, (uint8_t)152, (uint8_t)121, (uint8_t)133, (uint8_t)244, (uint8_t)12, (uint8_t)16, (uint8_t)164, (uint8_t)114, (uint8_t)0, (uint8_t)151, (uint8_t)4, (uint8_t)202, (uint8_t)242, (uint8_t)207, (uint8_t)47, (uint8_t)133, (uint8_t)183, (uint8_t)218, (uint8_t)82, (uint8_t)138, (uint8_t)1, (uint8_t)220, (uint8_t)128, (uint8_t)72, (uint8_t)43, (uint8_t)12, (uint8_t)131, (uint8_t)173, (uint8_t)17, (uint8_t)71, (uint8_t)254, (uint8_t)240, (uint8_t)51, (uint8_t)35, (uint8_t)242, (uint8_t)111, (uint8_t)95, (uint8_t)224, (uint8_t)52, (uint8_t)70, (uint8_t)140, (uint8_t)117, (uint8_t)140, (uint8_t)129, (uint8_t)149, (uint8_t)90, (uint8_t)38, (uint8_t)121, (uint8_t)165, (uint8_t)111, (uint8_t)243, (uint8_t)252, (uint8_t)117, (uint8_t)119, (uint8_t)168, (uint8_t)9, (uint8_t)85, (uint8_t)88, (uint8_t)113, (uint8_t)167, (uint8_t)227, (uint8_t)225, (uint8_t)108, (uint8_t)21, (uint8_t)253, (uint8_t)169, (uint8_t)33, (uint8_t)45, (uint8_t)3, (uint8_t)43, (uint8_t)38, (uint8_t)108, (uint8_t)36, (uint8_t)77, (uint8_t)122, (uint8_t)57, (uint8_t)6};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)3736841692L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -32293, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -6357, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)41146, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -8, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)1545, PH.base.pack) ;
    p234_latitude_SET((int32_t) -677599854, PH.base.pack) ;
    p234_longitude_SET((int32_t) -1654380998, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)29608, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)26264, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t) -80, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)69, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -90, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)62874, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)2970768748132978206L, PH.base.pack) ;
    p241_vibration_x_SET((float) -2.5783064E38F, PH.base.pack) ;
    p241_vibration_y_SET((float)1.2879567E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)6.637157E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3724142874L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)49501937L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)267300283L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -1986381436, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1873277552, PH.base.pack) ;
    p242_altitude_SET((int32_t)2113141908, PH.base.pack) ;
    p242_x_SET((float)1.4171695E38F, PH.base.pack) ;
    p242_y_SET((float)8.411795E37F, PH.base.pack) ;
    p242_z_SET((float) -2.674322E38F, PH.base.pack) ;
    {
        float  q [] =  {1.7332179E38F, 1.2240487E38F, 2.0516345E38F, 2.295362E37F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)1.9617232E38F, PH.base.pack) ;
    p242_approach_y_SET((float) -4.397882E37F, PH.base.pack) ;
    p242_approach_z_SET((float) -5.594915E37F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)24524355049635140L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p243_latitude_SET((int32_t)1875441950, PH.base.pack) ;
    p243_longitude_SET((int32_t)1134169217, PH.base.pack) ;
    p243_altitude_SET((int32_t)698794759, PH.base.pack) ;
    p243_x_SET((float)2.5275521E38F, PH.base.pack) ;
    p243_y_SET((float) -2.4290737E38F, PH.base.pack) ;
    p243_z_SET((float)2.551439E38F, PH.base.pack) ;
    {
        float  q [] =  {8.256234E37F, -1.4431112E38F, 2.561438E38F, -5.824325E37F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float)1.8035222E38F, PH.base.pack) ;
    p243_approach_y_SET((float) -2.8442621E37F, PH.base.pack) ;
    p243_approach_z_SET((float)1.249592E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)3684110418151328569L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)58448, PH.base.pack) ;
    p244_interval_us_SET((int32_t)528912612, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)663273670L, PH.base.pack) ;
    p246_lat_SET((int32_t) -1187888019, PH.base.pack) ;
    p246_lon_SET((int32_t)1459559580, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t)77586871, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)63153, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)63002, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -28155, PH.base.pack) ;
    {
        char16_t   callsign = "huuMa";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)20562, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)926328084L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float) -2.7936555E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -2.4088075E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float) -1.5685539E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)42385, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)243, (uint8_t)245, (uint8_t)26, (uint8_t)208, (uint8_t)79, (uint8_t)161, (uint8_t)72, (uint8_t)200, (uint8_t)90, (uint8_t)31, (uint8_t)31, (uint8_t)58, (uint8_t)12, (uint8_t)212, (uint8_t)171, (uint8_t)163, (uint8_t)8, (uint8_t)251, (uint8_t)215, (uint8_t)203, (uint8_t)144, (uint8_t)52, (uint8_t)146, (uint8_t)6, (uint8_t)93, (uint8_t)149, (uint8_t)188, (uint8_t)72, (uint8_t)179, (uint8_t)101, (uint8_t)144, (uint8_t)35, (uint8_t)135, (uint8_t)252, (uint8_t)52, (uint8_t)228, (uint8_t)166, (uint8_t)64, (uint8_t)9, (uint8_t)234, (uint8_t)189, (uint8_t)11, (uint8_t)137, (uint8_t)227, (uint8_t)129, (uint8_t)212, (uint8_t)230, (uint8_t)87, (uint8_t)60, (uint8_t)232, (uint8_t)53, (uint8_t)213, (uint8_t)93, (uint8_t)189, (uint8_t)206, (uint8_t)138, (uint8_t)54, (uint8_t)72, (uint8_t)71, (uint8_t)13, (uint8_t)230, (uint8_t)65, (uint8_t)238, (uint8_t)94, (uint8_t)207, (uint8_t)110, (uint8_t)26, (uint8_t)78, (uint8_t)4, (uint8_t)109, (uint8_t)202, (uint8_t)251, (uint8_t)154, (uint8_t)29, (uint8_t)240, (uint8_t)231, (uint8_t)83, (uint8_t)10, (uint8_t)228, (uint8_t)17, (uint8_t)166, (uint8_t)43, (uint8_t)145, (uint8_t)56, (uint8_t)116, (uint8_t)205, (uint8_t)5, (uint8_t)223, (uint8_t)24, (uint8_t)203, (uint8_t)133, (uint8_t)214, (uint8_t)205, (uint8_t)40, (uint8_t)1, (uint8_t)56, (uint8_t)186, (uint8_t)71, (uint8_t)140, (uint8_t)99, (uint8_t)207, (uint8_t)213, (uint8_t)150, (uint8_t)116, (uint8_t)245, (uint8_t)37, (uint8_t)252, (uint8_t)200, (uint8_t)209, (uint8_t)15, (uint8_t)139, (uint8_t)25, (uint8_t)30, (uint8_t)148, (uint8_t)141, (uint8_t)202, (uint8_t)6, (uint8_t)131, (uint8_t)12, (uint8_t)106, (uint8_t)166, (uint8_t)20, (uint8_t)174, (uint8_t)242, (uint8_t)177, (uint8_t)175, (uint8_t)199, (uint8_t)175, (uint8_t)205, (uint8_t)137, (uint8_t)34, (uint8_t)84, (uint8_t)57, (uint8_t)51, (uint8_t)77, (uint8_t)246, (uint8_t)12, (uint8_t)131, (uint8_t)8, (uint8_t)120, (uint8_t)138, (uint8_t)215, (uint8_t)37, (uint8_t)73, (uint8_t)148, (uint8_t)147, (uint8_t)197, (uint8_t)234, (uint8_t)15, (uint8_t)112, (uint8_t)24, (uint8_t)192, (uint8_t)122, (uint8_t)62, (uint8_t)135, (uint8_t)247, (uint8_t)101, (uint8_t)20, (uint8_t)8, (uint8_t)15, (uint8_t)233, (uint8_t)61, (uint8_t)0, (uint8_t)245, (uint8_t)170, (uint8_t)80, (uint8_t)164, (uint8_t)7, (uint8_t)91, (uint8_t)83, (uint8_t)20, (uint8_t)124, (uint8_t)65, (uint8_t)165, (uint8_t)213, (uint8_t)67, (uint8_t)126, (uint8_t)183, (uint8_t)92, (uint8_t)33, (uint8_t)170, (uint8_t)143, (uint8_t)136, (uint8_t)66, (uint8_t)211, (uint8_t)50, (uint8_t)195, (uint8_t)111, (uint8_t)137, (uint8_t)45, (uint8_t)116, (uint8_t)118, (uint8_t)15, (uint8_t)223, (uint8_t)77, (uint8_t)92, (uint8_t)123, (uint8_t)152, (uint8_t)3, (uint8_t)250, (uint8_t)238, (uint8_t)47, (uint8_t)254, (uint8_t)60, (uint8_t)138, (uint8_t)122, (uint8_t)121, (uint8_t)255, (uint8_t)47, (uint8_t)37, (uint8_t)242, (uint8_t)30, (uint8_t)143, (uint8_t)79, (uint8_t)247, (uint8_t)254, (uint8_t)202, (uint8_t)176, (uint8_t)193, (uint8_t)223, (uint8_t)1, (uint8_t)188, (uint8_t)16, (uint8_t)138, (uint8_t)143, (uint8_t)3, (uint8_t)218, (uint8_t)66, (uint8_t)60, (uint8_t)101, (uint8_t)113, (uint8_t)150, (uint8_t)164, (uint8_t)44, (uint8_t)131, (uint8_t)55, (uint8_t)235, (uint8_t)148, (uint8_t)56, (uint8_t)97, (uint8_t)215, (uint8_t)243, (uint8_t)101, (uint8_t)34, (uint8_t)81, (uint8_t)57, (uint8_t)133, (uint8_t)255, (uint8_t)230};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)32095, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)94, (int8_t) -30, (int8_t) -67, (int8_t) -78, (int8_t)57, (int8_t) -54, (int8_t) -112, (int8_t)26, (int8_t) -120, (int8_t) -94, (int8_t) -44, (int8_t)69, (int8_t)82, (int8_t) -32, (int8_t)88, (int8_t) -20, (int8_t)25, (int8_t) -62, (int8_t)85, (int8_t)51, (int8_t)0, (int8_t)1, (int8_t)87, (int8_t)23, (int8_t) -36, (int8_t)30, (int8_t) -57, (int8_t) -58, (int8_t) -20, (int8_t)81, (int8_t)49, (int8_t)63};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "wgvbdqapn";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)800937371587892502L, PH.base.pack) ;
    p250_x_SET((float)3.3697076E38F, PH.base.pack) ;
    p250_y_SET((float)3.1913866E38F, PH.base.pack) ;
    p250_z_SET((float) -7.3949275E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3200864033L, PH.base.pack) ;
    {
        char16_t   name = "irxbxzyiqr";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -2.3614542E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)656842513L, PH.base.pack) ;
    {
        char16_t   name = "eCvRrpN";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -1962371071, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL, PH.base.pack) ;
    {
        char16_t   text = "gfYqhxzqRfdvwetbGcCcuegasnuzcrocikmslVNsmtjr";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)1044866404L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p254_value_SET((float) -2.3079997E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)29, (uint8_t)34, (uint8_t)80, (uint8_t)73, (uint8_t)49, (uint8_t)133, (uint8_t)162, (uint8_t)147, (uint8_t)95, (uint8_t)234, (uint8_t)88, (uint8_t)232, (uint8_t)177, (uint8_t)236, (uint8_t)235, (uint8_t)54, (uint8_t)45, (uint8_t)107, (uint8_t)16, (uint8_t)75, (uint8_t)99, (uint8_t)209, (uint8_t)233, (uint8_t)253, (uint8_t)157, (uint8_t)32, (uint8_t)45, (uint8_t)175, (uint8_t)249, (uint8_t)61, (uint8_t)89, (uint8_t)194};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)5818001249619406628L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)4230235035L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)3072084670L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    {
        char16_t   tune = "sNwhcXeqFPojl";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)1903380256L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)102, (uint8_t)239, (uint8_t)96, (uint8_t)151, (uint8_t)25, (uint8_t)77, (uint8_t)109, (uint8_t)224, (uint8_t)6, (uint8_t)143, (uint8_t)104, (uint8_t)213, (uint8_t)130, (uint8_t)149, (uint8_t)146, (uint8_t)170, (uint8_t)3, (uint8_t)67, (uint8_t)116, (uint8_t)65, (uint8_t)136, (uint8_t)134, (uint8_t)99, (uint8_t)154, (uint8_t)245, (uint8_t)58, (uint8_t)85, (uint8_t)172, (uint8_t)88, (uint8_t)207, (uint8_t)33, (uint8_t)153};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)193, (uint8_t)159, (uint8_t)166, (uint8_t)228, (uint8_t)109, (uint8_t)52, (uint8_t)168, (uint8_t)73, (uint8_t)56, (uint8_t)224, (uint8_t)144, (uint8_t)121, (uint8_t)22, (uint8_t)107, (uint8_t)245, (uint8_t)215, (uint8_t)198, (uint8_t)208, (uint8_t)5, (uint8_t)199, (uint8_t)232, (uint8_t)97, (uint8_t)28, (uint8_t)165, (uint8_t)103, (uint8_t)102, (uint8_t)211, (uint8_t)193, (uint8_t)199, (uint8_t)163, (uint8_t)233, (uint8_t)35};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)3408412532L, PH.base.pack) ;
    p259_focal_length_SET((float)3.2119074E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -2.7618652E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -2.0804104E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)63328, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)39265, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)24819, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "ejUvhjezWgpdxiajmaujgJhmycqcanyfqyvvcnah";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)3353274652L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)635488550L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p261_total_capacity_SET((float)1.3128485E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)1.7665687E38F, PH.base.pack) ;
    p261_available_capacity_SET((float) -8.6084E37F, PH.base.pack) ;
    p261_read_speed_SET((float)1.9188368E38F, PH.base.pack) ;
    p261_write_speed_SET((float)1.74933E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)2815872602L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p262_image_interval_SET((float) -2.1347508E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)140829647L, PH.base.pack) ;
    p262_available_capacity_SET((float)1.858629E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)768114846L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)7829069239473839278L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p263_lat_SET((int32_t)145858704, PH.base.pack) ;
    p263_lon_SET((int32_t) -158623297, PH.base.pack) ;
    p263_alt_SET((int32_t)1696382272, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)413536056, PH.base.pack) ;
    {
        float  q [] =  {1.4138155E38F, 1.3556817E38F, -5.093595E37F, 8.436254E37F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1595731997, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t) -50, PH.base.pack) ;
    {
        char16_t   file_url = "iJyukoquycuvzmywzrjviuacIFTkbpKygGWpOgbfvundvlgpgvtpjojvfiiuyplyycpjxjEbtMhfvUaazelbzlyptdmhIpyaogunxgnzjqj";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)971427218L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)7966140069936256926L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)7120578415498743055L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)2165412964283899152L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)2563042843L, PH.base.pack) ;
    p265_roll_SET((float)3.1903934E38F, PH.base.pack) ;
    p265_pitch_SET((float) -2.2471577E38F, PH.base.pack) ;
    p265_yaw_SET((float)1.5024247E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)27909, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)12, (uint8_t)59, (uint8_t)68, (uint8_t)100, (uint8_t)183, (uint8_t)222, (uint8_t)103, (uint8_t)161, (uint8_t)216, (uint8_t)47, (uint8_t)206, (uint8_t)223, (uint8_t)224, (uint8_t)235, (uint8_t)6, (uint8_t)213, (uint8_t)141, (uint8_t)204, (uint8_t)80, (uint8_t)92, (uint8_t)212, (uint8_t)200, (uint8_t)159, (uint8_t)166, (uint8_t)224, (uint8_t)119, (uint8_t)100, (uint8_t)201, (uint8_t)240, (uint8_t)27, (uint8_t)217, (uint8_t)77, (uint8_t)98, (uint8_t)176, (uint8_t)68, (uint8_t)108, (uint8_t)12, (uint8_t)132, (uint8_t)158, (uint8_t)21, (uint8_t)2, (uint8_t)28, (uint8_t)73, (uint8_t)201, (uint8_t)175, (uint8_t)18, (uint8_t)238, (uint8_t)206, (uint8_t)220, (uint8_t)175, (uint8_t)160, (uint8_t)209, (uint8_t)21, (uint8_t)121, (uint8_t)159, (uint8_t)144, (uint8_t)68, (uint8_t)24, (uint8_t)126, (uint8_t)35, (uint8_t)33, (uint8_t)206, (uint8_t)94, (uint8_t)188, (uint8_t)153, (uint8_t)18, (uint8_t)46, (uint8_t)194, (uint8_t)136, (uint8_t)110, (uint8_t)41, (uint8_t)134, (uint8_t)34, (uint8_t)162, (uint8_t)242, (uint8_t)9, (uint8_t)234, (uint8_t)116, (uint8_t)91, (uint8_t)23, (uint8_t)141, (uint8_t)23, (uint8_t)119, (uint8_t)18, (uint8_t)71, (uint8_t)156, (uint8_t)130, (uint8_t)197, (uint8_t)71, (uint8_t)60, (uint8_t)121, (uint8_t)212, (uint8_t)99, (uint8_t)110, (uint8_t)203, (uint8_t)40, (uint8_t)46, (uint8_t)139, (uint8_t)17, (uint8_t)155, (uint8_t)120, (uint8_t)5, (uint8_t)2, (uint8_t)249, (uint8_t)156, (uint8_t)21, (uint8_t)1, (uint8_t)243, (uint8_t)191, (uint8_t)193, (uint8_t)135, (uint8_t)104, (uint8_t)117, (uint8_t)136, (uint8_t)126, (uint8_t)173, (uint8_t)139, (uint8_t)156, (uint8_t)201, (uint8_t)242, (uint8_t)219, (uint8_t)62, (uint8_t)215, (uint8_t)185, (uint8_t)12, (uint8_t)33, (uint8_t)210, (uint8_t)18, (uint8_t)77, (uint8_t)123, (uint8_t)120, (uint8_t)205, (uint8_t)125, (uint8_t)93, (uint8_t)252, (uint8_t)128, (uint8_t)213, (uint8_t)4, (uint8_t)213, (uint8_t)243, (uint8_t)225, (uint8_t)187, (uint8_t)129, (uint8_t)168, (uint8_t)80, (uint8_t)6, (uint8_t)121, (uint8_t)50, (uint8_t)112, (uint8_t)206, (uint8_t)88, (uint8_t)123, (uint8_t)129, (uint8_t)10, (uint8_t)251, (uint8_t)233, (uint8_t)3, (uint8_t)158, (uint8_t)89, (uint8_t)177, (uint8_t)21, (uint8_t)217, (uint8_t)34, (uint8_t)93, (uint8_t)180, (uint8_t)48, (uint8_t)137, (uint8_t)243, (uint8_t)207, (uint8_t)86, (uint8_t)34, (uint8_t)125, (uint8_t)171, (uint8_t)137, (uint8_t)29, (uint8_t)88, (uint8_t)119, (uint8_t)40, (uint8_t)242, (uint8_t)252, (uint8_t)9, (uint8_t)129, (uint8_t)36, (uint8_t)159, (uint8_t)210, (uint8_t)162, (uint8_t)253, (uint8_t)97, (uint8_t)181, (uint8_t)219, (uint8_t)28, (uint8_t)111, (uint8_t)62, (uint8_t)205, (uint8_t)90, (uint8_t)192, (uint8_t)45, (uint8_t)201, (uint8_t)80, (uint8_t)118, (uint8_t)175, (uint8_t)145, (uint8_t)159, (uint8_t)143, (uint8_t)184, (uint8_t)33, (uint8_t)28, (uint8_t)131, (uint8_t)200, (uint8_t)100, (uint8_t)144, (uint8_t)109, (uint8_t)44, (uint8_t)10, (uint8_t)92, (uint8_t)182, (uint8_t)120, (uint8_t)50, (uint8_t)195, (uint8_t)224, (uint8_t)253, (uint8_t)178, (uint8_t)226, (uint8_t)249, (uint8_t)87, (uint8_t)9, (uint8_t)94, (uint8_t)175, (uint8_t)157, (uint8_t)98, (uint8_t)24, (uint8_t)133, (uint8_t)245, (uint8_t)223, (uint8_t)44, (uint8_t)158, (uint8_t)89, (uint8_t)29, (uint8_t)63, (uint8_t)232, (uint8_t)124, (uint8_t)91, (uint8_t)31, (uint8_t)156, (uint8_t)59, (uint8_t)255, (uint8_t)231, (uint8_t)182, (uint8_t)189};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)42374, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)176, (uint8_t)123, (uint8_t)76, (uint8_t)121, (uint8_t)184, (uint8_t)69, (uint8_t)177, (uint8_t)108, (uint8_t)202, (uint8_t)98, (uint8_t)204, (uint8_t)88, (uint8_t)13, (uint8_t)90, (uint8_t)166, (uint8_t)248, (uint8_t)210, (uint8_t)63, (uint8_t)120, (uint8_t)11, (uint8_t)38, (uint8_t)160, (uint8_t)74, (uint8_t)212, (uint8_t)228, (uint8_t)236, (uint8_t)33, (uint8_t)80, (uint8_t)146, (uint8_t)78, (uint8_t)89, (uint8_t)180, (uint8_t)61, (uint8_t)124, (uint8_t)179, (uint8_t)132, (uint8_t)1, (uint8_t)45, (uint8_t)26, (uint8_t)151, (uint8_t)231, (uint8_t)194, (uint8_t)8, (uint8_t)40, (uint8_t)209, (uint8_t)96, (uint8_t)85, (uint8_t)49, (uint8_t)99, (uint8_t)98, (uint8_t)8, (uint8_t)121, (uint8_t)15, (uint8_t)201, (uint8_t)139, (uint8_t)86, (uint8_t)207, (uint8_t)157, (uint8_t)255, (uint8_t)65, (uint8_t)71, (uint8_t)180, (uint8_t)113, (uint8_t)47, (uint8_t)103, (uint8_t)144, (uint8_t)79, (uint8_t)83, (uint8_t)118, (uint8_t)170, (uint8_t)213, (uint8_t)131, (uint8_t)156, (uint8_t)49, (uint8_t)250, (uint8_t)215, (uint8_t)48, (uint8_t)159, (uint8_t)44, (uint8_t)98, (uint8_t)11, (uint8_t)237, (uint8_t)132, (uint8_t)139, (uint8_t)157, (uint8_t)6, (uint8_t)63, (uint8_t)243, (uint8_t)191, (uint8_t)165, (uint8_t)72, (uint8_t)253, (uint8_t)61, (uint8_t)153, (uint8_t)253, (uint8_t)97, (uint8_t)225, (uint8_t)44, (uint8_t)43, (uint8_t)81, (uint8_t)146, (uint8_t)48, (uint8_t)205, (uint8_t)169, (uint8_t)220, (uint8_t)52, (uint8_t)3, (uint8_t)71, (uint8_t)161, (uint8_t)8, (uint8_t)60, (uint8_t)56, (uint8_t)50, (uint8_t)111, (uint8_t)57, (uint8_t)16, (uint8_t)74, (uint8_t)202, (uint8_t)251, (uint8_t)57, (uint8_t)207, (uint8_t)114, (uint8_t)158, (uint8_t)12, (uint8_t)136, (uint8_t)248, (uint8_t)33, (uint8_t)22, (uint8_t)204, (uint8_t)48, (uint8_t)207, (uint8_t)46, (uint8_t)185, (uint8_t)63, (uint8_t)135, (uint8_t)60, (uint8_t)124, (uint8_t)79, (uint8_t)183, (uint8_t)2, (uint8_t)254, (uint8_t)200, (uint8_t)59, (uint8_t)91, (uint8_t)165, (uint8_t)204, (uint8_t)205, (uint8_t)26, (uint8_t)183, (uint8_t)165, (uint8_t)145, (uint8_t)199, (uint8_t)118, (uint8_t)101, (uint8_t)169, (uint8_t)212, (uint8_t)73, (uint8_t)246, (uint8_t)215, (uint8_t)182, (uint8_t)154, (uint8_t)21, (uint8_t)42, (uint8_t)59, (uint8_t)224, (uint8_t)108, (uint8_t)19, (uint8_t)255, (uint8_t)120, (uint8_t)154, (uint8_t)121, (uint8_t)70, (uint8_t)235, (uint8_t)118, (uint8_t)135, (uint8_t)84, (uint8_t)64, (uint8_t)9, (uint8_t)16, (uint8_t)100, (uint8_t)189, (uint8_t)186, (uint8_t)233, (uint8_t)119, (uint8_t)215, (uint8_t)175, (uint8_t)224, (uint8_t)39, (uint8_t)188, (uint8_t)176, (uint8_t)248, (uint8_t)12, (uint8_t)57, (uint8_t)89, (uint8_t)13, (uint8_t)114, (uint8_t)181, (uint8_t)150, (uint8_t)171, (uint8_t)52, (uint8_t)67, (uint8_t)197, (uint8_t)10, (uint8_t)207, (uint8_t)11, (uint8_t)98, (uint8_t)110, (uint8_t)121, (uint8_t)240, (uint8_t)126, (uint8_t)61, (uint8_t)177, (uint8_t)157, (uint8_t)2, (uint8_t)230, (uint8_t)190, (uint8_t)182, (uint8_t)155, (uint8_t)53, (uint8_t)41, (uint8_t)208, (uint8_t)21, (uint8_t)84, (uint8_t)95, (uint8_t)208, (uint8_t)95, (uint8_t)80, (uint8_t)15, (uint8_t)84, (uint8_t)136, (uint8_t)202, (uint8_t)220, (uint8_t)137, (uint8_t)175, (uint8_t)59, (uint8_t)111, (uint8_t)135, (uint8_t)5, (uint8_t)171, (uint8_t)161, (uint8_t)98, (uint8_t)165, (uint8_t)26, (uint8_t)30, (uint8_t)63, (uint8_t)86, (uint8_t)133, (uint8_t)191, (uint8_t)52};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)57488, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p269_framerate_SET((float) -1.809741E38F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)14249, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)51031, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)2218586275L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)59745, PH.base.pack) ;
    {
        char16_t   uri = "myzlzbmitka";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p270_framerate_SET((float)6.929274E37F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)7832, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)56977, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)1084741923L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)27270, PH.base.pack) ;
    {
        char16_t   uri = "ayknrsxpiDuerJgzjYqbjbzedbCozwzyDlwieuwJsxhp";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "ethlpXtm";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "ygzhyHkyNypppjimbobdjexqqo";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)21902, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)1228, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)28256, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)154, (uint8_t)213, (uint8_t)214, (uint8_t)140, (uint8_t)255, (uint8_t)61, (uint8_t)103, (uint8_t)191};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)167, (uint8_t)189, (uint8_t)85, (uint8_t)89, (uint8_t)156, (uint8_t)211, (uint8_t)174, (uint8_t)45};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)8470828588559101125L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)820103014L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)45883, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)323276701808814873L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)2279113597L, PH.base.pack) ;
    {
        char16_t   name = "qtqlkUmncwuldwmjCxdyrZ";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)71, (uint8_t)107, (uint8_t)254, (uint8_t)57, (uint8_t)62, (uint8_t)215, (uint8_t)184, (uint8_t)43, (uint8_t)35, (uint8_t)22, (uint8_t)54, (uint8_t)187, (uint8_t)99, (uint8_t)229, (uint8_t)78, (uint8_t)207};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)2749040742L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    {
        char16_t   param_id = "iswxVDg";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t)1666, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "s";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "frjTcrmJprjjhkgragpspqznFajpSzqsixpqoekzftbdlswaycyk";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)2269, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)1168, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    {
        char16_t   param_id = "mqmqayfnk";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "iFishjzpsoFi";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "puxog";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "nqqnzthxtaddwttpjqzyuvnnxqsosadiUpqelODhBroNgjczuwuljvnmvSgzupod";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)4086999563729987849L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)26358, (uint16_t)20090, (uint16_t)40832, (uint16_t)32692, (uint16_t)59765, (uint16_t)39414, (uint16_t)2599, (uint16_t)38806, (uint16_t)23315, (uint16_t)45841, (uint16_t)53086, (uint16_t)21864, (uint16_t)723, (uint16_t)38548, (uint16_t)1161, (uint16_t)55890, (uint16_t)53096, (uint16_t)24644, (uint16_t)21594, (uint16_t)38664, (uint16_t)23535, (uint16_t)58329, (uint16_t)60436, (uint16_t)35041, (uint16_t)60156, (uint16_t)25534, (uint16_t)47191, (uint16_t)46204, (uint16_t)38536, (uint16_t)59064, (uint16_t)9390, (uint16_t)16006, (uint16_t)52925, (uint16_t)52530, (uint16_t)34274, (uint16_t)18847, (uint16_t)48522, (uint16_t)55593, (uint16_t)29855, (uint16_t)54290, (uint16_t)57884, (uint16_t)64406, (uint16_t)41493, (uint16_t)37228, (uint16_t)29656, (uint16_t)11479, (uint16_t)34215, (uint16_t)60087, (uint16_t)55039, (uint16_t)25164, (uint16_t)17622, (uint16_t)40880, (uint16_t)44114, (uint16_t)12694, (uint16_t)37942, (uint16_t)18797, (uint16_t)43943, (uint16_t)13854, (uint16_t)28545, (uint16_t)55430, (uint16_t)48775, (uint16_t)36331, (uint16_t)52725, (uint16_t)55583, (uint16_t)5463, (uint16_t)35789, (uint16_t)6881, (uint16_t)51520, (uint16_t)52000, (uint16_t)22534, (uint16_t)61145, (uint16_t)293};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)65289, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)21016, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

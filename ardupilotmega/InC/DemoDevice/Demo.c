
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
void c_LoopBackDemoChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    int16_t  mag_ofs_x = p150_mag_ofs_x_GET(pack);
    int16_t  mag_ofs_y = p150_mag_ofs_y_GET(pack);
    int16_t  mag_ofs_z = p150_mag_ofs_z_GET(pack);
    float  mag_declination = p150_mag_declination_GET(pack);
    int32_t  raw_press = p150_raw_press_GET(pack);
    int32_t  raw_temp = p150_raw_temp_GET(pack);
    float  gyro_cal_x = p150_gyro_cal_x_GET(pack);
    float  gyro_cal_y = p150_gyro_cal_y_GET(pack);
    float  gyro_cal_z = p150_gyro_cal_z_GET(pack);
    float  accel_cal_x = p150_accel_cal_x_GET(pack);
    float  accel_cal_y = p150_accel_cal_y_GET(pack);
    float  accel_cal_z = p150_accel_cal_z_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  mag_ofs_x = p151_mag_ofs_x_GET(pack);
    int16_t  mag_ofs_y = p151_mag_ofs_y_GET(pack);
    int16_t  mag_ofs_z = p151_mag_ofs_z_GET(pack);
}
void c_LoopBackDemoChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  brkval = p152_brkval_GET(pack);
    uint16_t  freemem = p152_freemem_GET(pack);
    uint32_t  freemem32 = p152_freemem32_TRY(ph);
}
void c_LoopBackDemoChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  adc1 = p153_adc1_GET(pack);
    uint16_t  adc2 = p153_adc2_GET(pack);
    uint16_t  adc3 = p153_adc3_GET(pack);
    uint16_t  adc4 = p153_adc4_GET(pack);
    uint16_t  adc5 = p153_adc5_GET(pack);
    uint16_t  adc6 = p153_adc6_GET(pack);
}
void c_LoopBackDemoChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p154_target_system_GET(pack);
    uint8_t  target_component = p154_target_component_GET(pack);
    uint8_t  mode = p154_mode_GET(pack);
    uint16_t  shutter_speed = p154_shutter_speed_GET(pack);
    uint8_t  aperture = p154_aperture_GET(pack);
    uint8_t  iso = p154_iso_GET(pack);
    uint8_t  exposure_type = p154_exposure_type_GET(pack);
    uint8_t  command_id = p154_command_id_GET(pack);
    uint8_t  engine_cut_off = p154_engine_cut_off_GET(pack);
    uint8_t  extra_param = p154_extra_param_GET(pack);
    float  extra_value = p154_extra_value_GET(pack);
}
void c_LoopBackDemoChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p155_target_system_GET(pack);
    uint8_t  target_component = p155_target_component_GET(pack);
    uint8_t  session = p155_session_GET(pack);
    uint8_t  zoom_pos = p155_zoom_pos_GET(pack);
    int8_t  zoom_step = p155_zoom_step_GET(pack);
    uint8_t  focus_lock = p155_focus_lock_GET(pack);
    uint8_t  shot = p155_shot_GET(pack);
    uint8_t  command_id = p155_command_id_GET(pack);
    uint8_t  extra_param = p155_extra_param_GET(pack);
    float  extra_value = p155_extra_value_GET(pack);
}
void c_LoopBackDemoChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    e_MAV_MOUNT_MODE  mount_mode = p156_mount_mode_GET(pack);
    uint8_t  stab_roll = p156_stab_roll_GET(pack);
    uint8_t  stab_pitch = p156_stab_pitch_GET(pack);
    uint8_t  stab_yaw = p156_stab_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    int32_t  input_a = p157_input_a_GET(pack);
    int32_t  input_b = p157_input_b_GET(pack);
    int32_t  input_c = p157_input_c_GET(pack);
    uint8_t  save_position = p157_save_position_GET(pack);
}
void c_LoopBackDemoChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p158_target_system_GET(pack);
    uint8_t  target_component = p158_target_component_GET(pack);
    int32_t  pointing_a = p158_pointing_a_GET(pack);
    int32_t  pointing_b = p158_pointing_b_GET(pack);
    int32_t  pointing_c = p158_pointing_c_GET(pack);
}
void c_LoopBackDemoChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p160_target_system_GET(pack);
    uint8_t  target_component = p160_target_component_GET(pack);
    uint8_t  idx = p160_idx_GET(pack);
    uint8_t  count = p160_count_GET(pack);
    float  lat = p160_lat_GET(pack);
    float  lng = p160_lng_GET(pack);
}
void c_LoopBackDemoChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p161_target_system_GET(pack);
    uint8_t  target_component = p161_target_component_GET(pack);
    uint8_t  idx = p161_idx_GET(pack);
}
void c_LoopBackDemoChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  breach_status = p162_breach_status_GET(pack);
    uint16_t  breach_count = p162_breach_count_GET(pack);
    e_FENCE_BREACH  breach_type = p162_breach_type_GET(pack);
    uint32_t  breach_time = p162_breach_time_GET(pack);
}
void c_LoopBackDemoChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    float  omegaIx = p163_omegaIx_GET(pack);
    float  omegaIy = p163_omegaIy_GET(pack);
    float  omegaIz = p163_omegaIz_GET(pack);
    float  accel_weight = p163_accel_weight_GET(pack);
    float  renorm_val = p163_renorm_val_GET(pack);
    float  error_rp = p163_error_rp_GET(pack);
    float  error_yaw = p163_error_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p164_roll_GET(pack);
    float  pitch = p164_pitch_GET(pack);
    float  yaw = p164_yaw_GET(pack);
    float  xacc = p164_xacc_GET(pack);
    float  yacc = p164_yacc_GET(pack);
    float  zacc = p164_zacc_GET(pack);
    float  xgyro = p164_xgyro_GET(pack);
    float  ygyro = p164_ygyro_GET(pack);
    float  zgyro = p164_zgyro_GET(pack);
    int32_t  lat = p164_lat_GET(pack);
    int32_t  lng = p164_lng_GET(pack);
}
void c_LoopBackDemoChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p165_Vcc_GET(pack);
    uint8_t  I2Cerr = p165_I2Cerr_GET(pack);
}
void c_LoopBackDemoChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p166_rssi_GET(pack);
    uint8_t  remrssi = p166_remrssi_GET(pack);
    uint8_t  txbuf = p166_txbuf_GET(pack);
    uint8_t  noise = p166_noise_GET(pack);
    uint8_t  remnoise = p166_remnoise_GET(pack);
    uint16_t  rxerrors = p166_rxerrors_GET(pack);
    uint16_t  fixed_ = p166_fixed__GET(pack);
}
void c_LoopBackDemoChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    e_LIMITS_STATE  limits_state = p167_limits_state_GET(pack);
    uint32_t  last_trigger = p167_last_trigger_GET(pack);
    uint32_t  last_action = p167_last_action_GET(pack);
    uint32_t  last_recovery = p167_last_recovery_GET(pack);
    uint32_t  last_clear = p167_last_clear_GET(pack);
    uint16_t  breach_count = p167_breach_count_GET(pack);
    e_LIMIT_MODULE  mods_enabled = p167_mods_enabled_GET(pack);
    e_LIMIT_MODULE  mods_required = p167_mods_required_GET(pack);
    e_LIMIT_MODULE  mods_triggered = p167_mods_triggered_GET(pack);
}
void c_LoopBackDemoChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    float  direction = p168_direction_GET(pack);
    float  speed = p168_speed_GET(pack);
    float  speed_z = p168_speed_z_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p169_type_GET(pack);
    uint8_t  len = p169_len_GET(pack);
    uint8_t*  data_ = p169_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p170_type_GET(pack);
    uint8_t  len = p170_len_GET(pack);
    uint8_t*  data_ = p170_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p171_type_GET(pack);
    uint8_t  len = p171_len_GET(pack);
    uint8_t*  data_ = p171_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p172_type_GET(pack);
    uint8_t  len = p172_len_GET(pack);
    uint8_t*  data_ = p172_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    float  distance = p173_distance_GET(pack);
    float  voltage = p173_voltage_GET(pack);
}
void c_LoopBackDemoChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    float  vx = p174_vx_GET(pack);
    float  vy = p174_vy_GET(pack);
    float  vz = p174_vz_GET(pack);
    float  diff_pressure = p174_diff_pressure_GET(pack);
    float  EAS2TAS = p174_EAS2TAS_GET(pack);
    float  ratio = p174_ratio_GET(pack);
    float  state_x = p174_state_x_GET(pack);
    float  state_y = p174_state_y_GET(pack);
    float  state_z = p174_state_z_GET(pack);
    float  Pax = p174_Pax_GET(pack);
    float  Pby = p174_Pby_GET(pack);
    float  Pcz = p174_Pcz_GET(pack);
}
void c_LoopBackDemoChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p175_target_system_GET(pack);
    uint8_t  target_component = p175_target_component_GET(pack);
    uint8_t  idx = p175_idx_GET(pack);
    uint8_t  count = p175_count_GET(pack);
    int32_t  lat = p175_lat_GET(pack);
    int32_t  lng = p175_lng_GET(pack);
    int16_t  alt = p175_alt_GET(pack);
    int16_t  break_alt = p175_break_alt_GET(pack);
    uint16_t  land_dir = p175_land_dir_GET(pack);
    e_RALLY_FLAGS  flags = p175_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p176_target_system_GET(pack);
    uint8_t  target_component = p176_target_component_GET(pack);
    uint8_t  idx = p176_idx_GET(pack);
}
void c_LoopBackDemoChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  throttle = p177_throttle_GET(pack);
    float  current = p177_current_GET(pack);
    uint16_t  interference = p177_interference_GET(pack);
    float  CompensationX = p177_CompensationX_GET(pack);
    float  CompensationY = p177_CompensationY_GET(pack);
    float  CompensationZ = p177_CompensationZ_GET(pack);
}
void c_LoopBackDemoChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p178_roll_GET(pack);
    float  pitch = p178_pitch_GET(pack);
    float  yaw = p178_yaw_GET(pack);
    float  altitude = p178_altitude_GET(pack);
    int32_t  lat = p178_lat_GET(pack);
    int32_t  lng = p178_lng_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p179_time_usec_GET(pack);
    uint8_t  target_system = p179_target_system_GET(pack);
    uint8_t  cam_idx = p179_cam_idx_GET(pack);
    uint16_t  img_idx = p179_img_idx_GET(pack);
    e_CAMERA_STATUS_TYPES  event_id = p179_event_id_GET(pack);
    float  p1 = p179_p1_GET(pack);
    float  p2 = p179_p2_GET(pack);
    float  p3 = p179_p3_GET(pack);
    float  p4 = p179_p4_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p180_time_usec_GET(pack);
    uint8_t  target_system = p180_target_system_GET(pack);
    uint8_t  cam_idx = p180_cam_idx_GET(pack);
    uint16_t  img_idx = p180_img_idx_GET(pack);
    int32_t  lat = p180_lat_GET(pack);
    int32_t  lng = p180_lng_GET(pack);
    float  alt_msl = p180_alt_msl_GET(pack);
    float  alt_rel = p180_alt_rel_GET(pack);
    float  roll = p180_roll_GET(pack);
    float  pitch = p180_pitch_GET(pack);
    float  yaw = p180_yaw_GET(pack);
    float  foc_len = p180_foc_len_GET(pack);
    e_CAMERA_FEEDBACK_FLAGS  flags = p180_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  voltage = p181_voltage_GET(pack);
    int16_t  current_battery = p181_current_battery_GET(pack);
}
void c_LoopBackDemoChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p182_roll_GET(pack);
    float  pitch = p182_pitch_GET(pack);
    float  yaw = p182_yaw_GET(pack);
    float  altitude = p182_altitude_GET(pack);
    int32_t  lat = p182_lat_GET(pack);
    int32_t  lng = p182_lng_GET(pack);
    float  v1 = p182_v1_GET(pack);
    float  v2 = p182_v2_GET(pack);
    float  v3 = p182_v3_GET(pack);
    float  v4 = p182_v4_GET(pack);
}
void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p183_target_system_GET(pack);
    uint8_t  target_component = p183_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p184_target_system_GET(pack);
    uint8_t  target_component = p184_target_component_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  seqno = p184_seqno_GET(pack);
    uint8_t*  data_ = p184_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p185_target_system_GET(pack);
    uint8_t  target_component = p185_target_component_GET(pack);
    uint32_t  seqno = p185_seqno_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  status = p185_status_GET(pack);
}
void c_LoopBackDemoChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p186_target_system_GET(pack);
    uint8_t  target_component = p186_target_component_GET(pack);
    uint8_t  instance = p186_instance_GET(pack);
    uint8_t  pattern = p186_pattern_GET(pack);
    uint8_t  custom_len = p186_custom_len_GET(pack);
    uint8_t*  custom_bytes = p186_custom_bytes_GET_(pack);
//process data in custom_bytes
    free(custom_bytes);//never forget to dispose
}
void c_LoopBackDemoChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  compass_id = p191_compass_id_GET(pack);
    uint8_t  cal_mask = p191_cal_mask_GET(pack);
    e_MAG_CAL_STATUS  cal_status = p191_cal_status_GET(pack);
    uint8_t  attempt = p191_attempt_GET(pack);
    uint8_t  completion_pct = p191_completion_pct_GET(pack);
    uint8_t*  completion_mask = p191_completion_mask_GET_(pack);
//process data in completion_mask
    free(completion_mask);//never forget to dispose
    float  direction_x = p191_direction_x_GET(pack);
    float  direction_y = p191_direction_y_GET(pack);
    float  direction_z = p191_direction_z_GET(pack);
}
void c_LoopBackDemoChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  compass_id = p192_compass_id_GET(pack);
    uint8_t  cal_mask = p192_cal_mask_GET(pack);
    e_MAG_CAL_STATUS  cal_status = p192_cal_status_GET(pack);
    uint8_t  autosaved = p192_autosaved_GET(pack);
    float  fitness = p192_fitness_GET(pack);
    float  ofs_x = p192_ofs_x_GET(pack);
    float  ofs_y = p192_ofs_y_GET(pack);
    float  ofs_z = p192_ofs_z_GET(pack);
    float  diag_x = p192_diag_x_GET(pack);
    float  diag_y = p192_diag_y_GET(pack);
    float  diag_z = p192_diag_z_GET(pack);
    float  offdiag_x = p192_offdiag_x_GET(pack);
    float  offdiag_y = p192_offdiag_y_GET(pack);
    float  offdiag_z = p192_offdiag_z_GET(pack);
}
void c_LoopBackDemoChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    e_EKF_STATUS_FLAGS  flags = p193_flags_GET(pack);
    float  velocity_variance = p193_velocity_variance_GET(pack);
    float  pos_horiz_variance = p193_pos_horiz_variance_GET(pack);
    float  pos_vert_variance = p193_pos_vert_variance_GET(pack);
    float  compass_variance = p193_compass_variance_GET(pack);
    float  terrain_alt_variance = p193_terrain_alt_variance_GET(pack);
}
void c_LoopBackDemoChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    e_PID_TUNING_AXIS  axis = p194_axis_GET(pack);
    float  desired = p194_desired_GET(pack);
    float  achieved = p194_achieved_GET(pack);
    float  FF = p194_FF_GET(pack);
    float  P = p194_P_GET(pack);
    float  I = p194_I_GET(pack);
    float  D = p194_D_GET(pack);
}
void c_LoopBackDemoChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p200_target_system_GET(pack);
    uint8_t  target_component = p200_target_component_GET(pack);
    float  delta_time = p200_delta_time_GET(pack);
    float  delta_angle_x = p200_delta_angle_x_GET(pack);
    float  delta_angle_y = p200_delta_angle_y_GET(pack);
    float  delta_angle_z = p200_delta_angle_z_GET(pack);
    float  delta_velocity_x = p200_delta_velocity_x_GET(pack);
    float  delta_velocity_y = p200_delta_velocity_y_GET(pack);
    float  delta_velocity_z = p200_delta_velocity_z_GET(pack);
    float  joint_roll = p200_joint_roll_GET(pack);
    float  joint_el = p200_joint_el_GET(pack);
    float  joint_az = p200_joint_az_GET(pack);
}
void c_LoopBackDemoChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p201_target_system_GET(pack);
    uint8_t  target_component = p201_target_component_GET(pack);
    float  demanded_rate_x = p201_demanded_rate_x_GET(pack);
    float  demanded_rate_y = p201_demanded_rate_y_GET(pack);
    float  demanded_rate_z = p201_demanded_rate_z_GET(pack);
}
void c_LoopBackDemoChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p214_target_system_GET(pack);
    uint8_t  target_component = p214_target_component_GET(pack);
    int16_t  rl_torque_cmd = p214_rl_torque_cmd_GET(pack);
    int16_t  el_torque_cmd = p214_el_torque_cmd_GET(pack);
    int16_t  az_torque_cmd = p214_az_torque_cmd_GET(pack);
}
void c_LoopBackDemoChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_HEARTBEAT_STATUS  status = p215_status_GET(pack);
    e_GOPRO_CAPTURE_MODE  capture_mode = p215_capture_mode_GET(pack);
    e_GOPRO_HEARTBEAT_FLAGS  flags = p215_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p216_target_system_GET(pack);
    uint8_t  target_component = p216_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p216_cmd_id_GET(pack);
}
void c_LoopBackDemoChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p217_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p217_status_GET(pack);
    uint8_t*  value = p217_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p218_target_system_GET(pack);
    uint8_t  target_component = p218_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p218_cmd_id_GET(pack);
    uint8_t*  value = p218_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p219_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p219_status_GET(pack);
}
void c_LoopBackDemoChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    float  rpm1 = p226_rpm1_GET(pack);
    float  rpm2 = p226_rpm2_GET(pack);
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
void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  ICAO = p10001_ICAO_GET(pack);
    char16_t *  callsign = p10001_callsign_TRY_(ph);
    e_ADSB_EMITTER_TYPE  emitterType = p10001_emitterType_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE  aircraftSize = p10001_aircraftSize_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT  gpsOffsetLat = p10001_gpsOffsetLat_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON  gpsOffsetLon = p10001_gpsOffsetLon_GET(pack);
    uint16_t  stallSpeed = p10001_stallSpeed_GET(pack);
    e_UAVIONIX_ADSB_OUT_RF_SELECT  rfSelect = p10001_rfSelect_GET(pack);
}
void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  utcTime = p10002_utcTime_GET(pack);
    int32_t  gpsLat = p10002_gpsLat_GET(pack);
    int32_t  gpsLon = p10002_gpsLon_GET(pack);
    int32_t  gpsAlt = p10002_gpsAlt_GET(pack);
    e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX  gpsFix = p10002_gpsFix_GET(pack);
    uint8_t  numSats = p10002_numSats_GET(pack);
    int32_t  baroAltMSL = p10002_baroAltMSL_GET(pack);
    uint32_t  accuracyHor = p10002_accuracyHor_GET(pack);
    uint16_t  accuracyVert = p10002_accuracyVert_GET(pack);
    uint16_t  accuracyVel = p10002_accuracyVel_GET(pack);
    int16_t  velVert = p10002_velVert_GET(pack);
    int16_t  velNS = p10002_velNS_GET(pack);
    int16_t  VelEW = p10002_VelEW_GET(pack);
    e_UAVIONIX_ADSB_EMERGENCY_STATUS  emergencyStatus = p10002_emergencyStatus_GET(pack);
    e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE  state = p10002_state_GET(pack);
    uint16_t  squawk = p10002_squawk_GET(pack);
}
void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    e_UAVIONIX_ADSB_RF_HEALTH  rfHealth = p10003_rfHealth_GET(pack);
}
void c_LoopBackDemoChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11000_target_system_GET(pack);
    uint8_t  target_component = p11000_target_component_GET(pack);
    uint32_t  request_id = p11000_request_id_GET(pack);
    e_DEVICE_OP_BUSTYPE  bustype = p11000_bustype_GET(pack);
    uint8_t  bus = p11000_bus_GET(pack);
    uint8_t  address = p11000_address_GET(pack);
    char16_t *  busname = p11000_busname_TRY_(ph);
    uint8_t  regstart = p11000_regstart_GET(pack);
    uint8_t  count = p11000_count_GET(pack);
}
void c_LoopBackDemoChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11001_request_id_GET(pack);
    uint8_t  result = p11001_result_GET(pack);
    uint8_t  regstart = p11001_regstart_GET(pack);
    uint8_t  count = p11001_count_GET(pack);
    uint8_t*  data_ = p11001_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11002_target_system_GET(pack);
    uint8_t  target_component = p11002_target_component_GET(pack);
    uint32_t  request_id = p11002_request_id_GET(pack);
    e_DEVICE_OP_BUSTYPE  bustype = p11002_bustype_GET(pack);
    uint8_t  bus = p11002_bus_GET(pack);
    uint8_t  address = p11002_address_GET(pack);
    char16_t *  busname = p11002_busname_TRY_(ph);
    uint8_t  regstart = p11002_regstart_GET(pack);
    uint8_t  count = p11002_count_GET(pack);
    uint8_t*  data_ = p11002_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11003_request_id_GET(pack);
    uint8_t  result = p11003_result_GET(pack);
}
void c_LoopBackDemoChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    e_PID_TUNING_AXIS  axis = p11010_axis_GET(pack);
    float  desired = p11010_desired_GET(pack);
    float  achieved = p11010_achieved_GET(pack);
    float  error = p11010_error_GET(pack);
    float  theta = p11010_theta_GET(pack);
    float  omega = p11010_omega_GET(pack);
    float  sigma = p11010_sigma_GET(pack);
    float  theta_dot = p11010_theta_dot_GET(pack);
    float  omega_dot = p11010_omega_dot_GET(pack);
    float  sigma_dot = p11010_sigma_dot_GET(pack);
    float  f = p11010_f_GET(pack);
    float  f_dot = p11010_f_dot_GET(pack);
    float  u = p11010_u_GET(pack);
}
void c_LoopBackDemoChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p11011_time_usec_GET(pack);
    uint64_t  time_delta_usec = p11011_time_delta_usec_GET(pack);
    float*  angle_delta = p11011_angle_delta_GET_(pack);
//process data in angle_delta
    free(angle_delta);//never forget to dispose
    float*  position_delta = p11011_position_delta_GET_(pack);
//process data in position_delta
    free(position_delta);//never forget to dispose
    float  confidence = p11011_confidence_GET(pack);
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_SUBMARINE, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)4165504742L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)63485, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)49202, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -7791, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)53, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)26024, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)2284, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)14067, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)50812, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)38417, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)23363, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)5357044018673481060L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)13760096L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)2440046547L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)56394, PH.base.pack) ;
    p3_x_SET((float)3.1586271E38F, PH.base.pack) ;
    p3_y_SET((float)1.2402142E38F, PH.base.pack) ;
    p3_z_SET((float) -1.3282818E38F, PH.base.pack) ;
    p3_vx_SET((float) -2.060629E38F, PH.base.pack) ;
    p3_vy_SET((float) -1.957628E38F, PH.base.pack) ;
    p3_vz_SET((float) -2.5032465E38F, PH.base.pack) ;
    p3_afx_SET((float) -1.2949039E38F, PH.base.pack) ;
    p3_afy_SET((float) -1.067485E38F, PH.base.pack) ;
    p3_afz_SET((float) -1.8681194E38F, PH.base.pack) ;
    p3_yaw_SET((float) -3.388894E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)3.9398378E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)7346663164358295329L, PH.base.pack) ;
    p4_seq_SET((uint32_t)950954184L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    {
        char16_t   passkey = "wy";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "vwy";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)1019550150L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    {
        char16_t   param_id = "nxvwellSeac";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -12813, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "zfufmviminpbobD";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -1.2200344E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)46132, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)6316, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    {
        char16_t   param_id = "e";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)3.3077468E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)4529157182864507503L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -1263872948, PH.base.pack) ;
    p24_lon_SET((int32_t) -1767912996, PH.base.pack) ;
    p24_alt_SET((int32_t)2073740167, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)3878, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)36017, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)6337, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)39508, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1189551648, &PH) ;
    p24_h_acc_SET((uint32_t)3354178816L, &PH) ;
    p24_v_acc_SET((uint32_t)3594964389L, &PH) ;
    p24_vel_acc_SET((uint32_t)2624264230L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3944000414L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)178, (uint8_t)101, (uint8_t)119, (uint8_t)168, (uint8_t)18, (uint8_t)117, (uint8_t)55, (uint8_t)125, (uint8_t)192, (uint8_t)243, (uint8_t)228, (uint8_t)125, (uint8_t)143, (uint8_t)200, (uint8_t)53, (uint8_t)254, (uint8_t)45, (uint8_t)84, (uint8_t)155, (uint8_t)225};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)5, (uint8_t)180, (uint8_t)86, (uint8_t)92, (uint8_t)50, (uint8_t)102, (uint8_t)149, (uint8_t)82, (uint8_t)251, (uint8_t)179, (uint8_t)185, (uint8_t)84, (uint8_t)73, (uint8_t)190, (uint8_t)227, (uint8_t)137, (uint8_t)28, (uint8_t)168, (uint8_t)0, (uint8_t)244};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)62, (uint8_t)183, (uint8_t)3, (uint8_t)156, (uint8_t)197, (uint8_t)119, (uint8_t)110, (uint8_t)92, (uint8_t)70, (uint8_t)50, (uint8_t)238, (uint8_t)172, (uint8_t)103, (uint8_t)76, (uint8_t)107, (uint8_t)20, (uint8_t)135, (uint8_t)17, (uint8_t)40, (uint8_t)154};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)53, (uint8_t)94, (uint8_t)72, (uint8_t)149, (uint8_t)110, (uint8_t)13, (uint8_t)246, (uint8_t)159, (uint8_t)137, (uint8_t)149, (uint8_t)96, (uint8_t)13, (uint8_t)142, (uint8_t)60, (uint8_t)143, (uint8_t)170, (uint8_t)203, (uint8_t)151, (uint8_t)159, (uint8_t)131};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)253, (uint8_t)197, (uint8_t)76, (uint8_t)30, (uint8_t)85, (uint8_t)23, (uint8_t)183, (uint8_t)88, (uint8_t)14, (uint8_t)255, (uint8_t)141, (uint8_t)210, (uint8_t)42, (uint8_t)81, (uint8_t)173, (uint8_t)31, (uint8_t)21, (uint8_t)56, (uint8_t)165, (uint8_t)21};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)4102916540L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -19108, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t) -19539, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -15497, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -13478, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -26473, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -19883, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)14991, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -22168, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)15247, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)6250467929136578810L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)21842, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)14995, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)27999, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)21166, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)2500, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -19739, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)26290, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)4193, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -20944, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)9180508788903634822L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -10382, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -2987, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -25184, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)13747, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)1761266407L, PH.base.pack) ;
    p29_press_abs_SET((float)1.2165957E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -1.917649E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -15736, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)3390393347L, PH.base.pack) ;
    p30_roll_SET((float) -2.8080826E38F, PH.base.pack) ;
    p30_pitch_SET((float)3.331969E38F, PH.base.pack) ;
    p30_yaw_SET((float)2.8721239E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -1.6854041E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -2.5421228E38F, PH.base.pack) ;
    p30_yawspeed_SET((float)6.14097E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)2728009047L, PH.base.pack) ;
    p31_q1_SET((float) -1.9256166E38F, PH.base.pack) ;
    p31_q2_SET((float) -2.1300505E38F, PH.base.pack) ;
    p31_q3_SET((float) -1.3033688E38F, PH.base.pack) ;
    p31_q4_SET((float)2.6855099E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)1.1804635E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)1.444227E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)5.8412795E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1161395638L, PH.base.pack) ;
    p32_x_SET((float) -1.4723217E37F, PH.base.pack) ;
    p32_y_SET((float)1.0667426E38F, PH.base.pack) ;
    p32_z_SET((float) -2.554527E38F, PH.base.pack) ;
    p32_vx_SET((float)1.9542455E38F, PH.base.pack) ;
    p32_vy_SET((float) -3.29135E38F, PH.base.pack) ;
    p32_vz_SET((float)4.385481E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1163970275L, PH.base.pack) ;
    p33_lat_SET((int32_t) -354437259, PH.base.pack) ;
    p33_lon_SET((int32_t) -1611351596, PH.base.pack) ;
    p33_alt_SET((int32_t) -1244058626, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1398767436, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -22897, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)32702, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)10806, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)57641, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)1615306907L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -27192, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)22225, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -5122, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -4696, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)1287, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)10424, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -14392, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)5831, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)2408207046L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)43292, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)37573, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)61454, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)32656, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)15291, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)26263, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)45438, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)15351, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)2960196524L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)64956, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)19844, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)25701, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)56279, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)28538, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)6139, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)14129, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)1663, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)28038, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)62631, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)52123, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)61583, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)48072, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)31494, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)8091, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)39880, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -11713, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -18883, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -27611, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -28460, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)28422, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_DELAY, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p39_param1_SET((float)2.5855466E37F, PH.base.pack) ;
    p39_param2_SET((float)3.329339E38F, PH.base.pack) ;
    p39_param3_SET((float) -7.0522927E37F, PH.base.pack) ;
    p39_param4_SET((float)1.7801966E38F, PH.base.pack) ;
    p39_x_SET((float)6.7039144E37F, PH.base.pack) ;
    p39_y_SET((float)2.7583371E38F, PH.base.pack) ;
    p39_z_SET((float) -1.6771646E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)16929, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)27565, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)52979, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)33948, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)20265, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p48_latitude_SET((int32_t) -866390581, PH.base.pack) ;
    p48_longitude_SET((int32_t) -834342126, PH.base.pack) ;
    p48_altitude_SET((int32_t) -1507571952, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)7492981668144685719L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1240089654, PH.base.pack) ;
    p49_longitude_SET((int32_t) -937853287, PH.base.pack) ;
    p49_altitude_SET((int32_t)1502130069, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)6107010767381429785L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    {
        char16_t   param_id = "qqcY";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -19261, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p50_param_value0_SET((float)2.329035E38F, PH.base.pack) ;
    p50_scale_SET((float) -1.0541992E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.0017625E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)9.754752E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)20837, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p54_p1x_SET((float)2.7188909E38F, PH.base.pack) ;
    p54_p1y_SET((float)6.332755E37F, PH.base.pack) ;
    p54_p1z_SET((float)2.2619274E38F, PH.base.pack) ;
    p54_p2x_SET((float)2.1934495E38F, PH.base.pack) ;
    p54_p2y_SET((float) -2.7452686E38F, PH.base.pack) ;
    p54_p2z_SET((float)1.8667163E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p55_p1x_SET((float) -6.000454E36F, PH.base.pack) ;
    p55_p1y_SET((float) -2.2969575E38F, PH.base.pack) ;
    p55_p1z_SET((float)3.1209769E38F, PH.base.pack) ;
    p55_p2x_SET((float) -2.4701916E38F, PH.base.pack) ;
    p55_p2y_SET((float) -1.1035779E38F, PH.base.pack) ;
    p55_p2z_SET((float)2.1582494E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)5244325753433757743L, PH.base.pack) ;
    {
        float  q [] =  {-2.339091E38F, -7.3067127E37F, -1.9295435E38F, -2.3809906E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -3.3324536E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -1.7615366E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.8477555E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-3.385277E38F, 3.1432113E38F, 3.1279155E38F, 2.3130624E38F, 1.6048161E38F, -2.997138E38F, 2.024877E37F, 1.3029493E38F, -1.237156E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.4168028E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)2.8553416E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)7910, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)19331, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)34999, PH.base.pack) ;
    p62_alt_error_SET((float) -4.2525936E37F, PH.base.pack) ;
    p62_aspd_error_SET((float)3.334686E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -2.7802263E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)8489549263594765998L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t)1986586024, PH.base.pack) ;
    p63_lon_SET((int32_t)132653166, PH.base.pack) ;
    p63_alt_SET((int32_t) -751031737, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)1907301574, PH.base.pack) ;
    p63_vx_SET((float)2.3992145E38F, PH.base.pack) ;
    p63_vy_SET((float)3.1162499E38F, PH.base.pack) ;
    p63_vz_SET((float) -2.2818445E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-5.083738E36F, 1.8556456E38F, -1.2806158E38F, -1.5631056E38F, 3.2754355E38F, 3.3070613E38F, -2.597434E38F, -9.1088973E36F, -1.9924285E38F, -1.8521988E38F, 2.8006545E38F, -2.1978642E38F, -2.126693E38F, 1.4940383E38F, 2.6796613E38F, -6.2948273E37F, -1.2731798E38F, -1.7684355E38F, 1.0457154E38F, -2.1078827E38F, -2.8072246E38F, 1.7645214E38F, 2.0710334E38F, -3.7196225E37F, -4.471913E37F, -7.2725085E37F, -7.107622E37F, -2.1967411E38F, -2.9291572E38F, -2.0836653E38F, -3.1536123E38F, 9.541899E37F, -3.1261469E38F, 1.670642E38F, -1.1392208E38F, 1.963272E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)7215363039518738590L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p64_x_SET((float)4.4960235E37F, PH.base.pack) ;
    p64_y_SET((float)9.052983E37F, PH.base.pack) ;
    p64_z_SET((float)3.659501E37F, PH.base.pack) ;
    p64_vx_SET((float)2.7680956E38F, PH.base.pack) ;
    p64_vy_SET((float) -3.1723948E38F, PH.base.pack) ;
    p64_vz_SET((float) -2.176494E38F, PH.base.pack) ;
    p64_ax_SET((float)2.1638072E38F, PH.base.pack) ;
    p64_ay_SET((float) -2.3900315E37F, PH.base.pack) ;
    p64_az_SET((float) -2.6051847E37F, PH.base.pack) ;
    {
        float  covariance [] =  {4.945755E37F, 2.5713197E38F, -2.0714048E38F, -3.27302E38F, -1.9582752E38F, -6.2710564E37F, -1.0183728E37F, 2.06337E38F, -2.6239842E38F, -3.3248165E38F, -2.0819033E38F, -2.3602689E38F, -2.604049E38F, 2.3339118E37F, -2.0925605E38F, 1.7281493E38F, -3.3735496E37F, 3.2871026E38F, -6.3726134E37F, -1.194345E38F, 2.7714572E38F, 1.2754581E38F, -2.518137E37F, 1.2985002E38F, -1.97488E38F, 9.876548E37F, -1.5612016E38F, 7.6403477E37F, 4.4608944E37F, 8.788108E37F, -1.0962862E38F, -1.2081836E38F, -1.3382272E38F, -2.1832053E38F, 3.0856662E38F, 9.368695E37F, 1.7945803E38F, -2.0255732E38F, 1.9790024E38F, -2.9090089E38F, -1.1422535E38F, 1.5415572E38F, -5.8290636E37F, -1.9374395E38F, 9.436159E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)3331073927L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)17820, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)40076, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)33994, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)28270, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)39573, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)17057, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)58901, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)35843, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)53368, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)54097, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)37091, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)43783, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)19324, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)34515, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)49511, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)61915, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)3143, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)4776, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)27797, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)25843, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -7067, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -1595, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)22255, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -1392, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)50750, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)48202, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)42317, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)64350, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)50451, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)10649, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)33545, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)35905, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)61557, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)10115, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p73_param1_SET((float)4.5448747E36F, PH.base.pack) ;
    p73_param2_SET((float) -1.817705E38F, PH.base.pack) ;
    p73_param3_SET((float)2.0360233E38F, PH.base.pack) ;
    p73_param4_SET((float) -2.3853485E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -1602885976, PH.base.pack) ;
    p73_y_SET((int32_t) -942409532, PH.base.pack) ;
    p73_z_SET((float)2.7455687E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.8440692E37F, PH.base.pack) ;
    p74_groundspeed_SET((float)2.0533356E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)479, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)28698, PH.base.pack) ;
    p74_alt_SET((float) -1.3492344E38F, PH.base.pack) ;
    p74_climb_SET((float) -2.127818E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_YAW, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p75_param1_SET((float)1.0246189E38F, PH.base.pack) ;
    p75_param2_SET((float) -2.4690148E38F, PH.base.pack) ;
    p75_param3_SET((float)2.0773944E38F, PH.base.pack) ;
    p75_param4_SET((float) -5.797023E37F, PH.base.pack) ;
    p75_x_SET((int32_t)677607838, PH.base.pack) ;
    p75_y_SET((int32_t)335329354, PH.base.pack) ;
    p75_z_SET((float)1.5896824E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p76_param1_SET((float) -2.0677113E38F, PH.base.pack) ;
    p76_param2_SET((float) -3.3239058E38F, PH.base.pack) ;
    p76_param3_SET((float)1.833397E38F, PH.base.pack) ;
    p76_param4_SET((float)1.2013108E38F, PH.base.pack) ;
    p76_param5_SET((float)2.7061886E38F, PH.base.pack) ;
    p76_param6_SET((float) -1.4659574E37F, PH.base.pack) ;
    p76_param7_SET((float) -1.3971028E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_DO_RALLY_LAND, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)200, &PH) ;
    p77_result_param2_SET((int32_t)552056438, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)29, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)180, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)974868954L, PH.base.pack) ;
    p81_roll_SET((float)2.5323128E38F, PH.base.pack) ;
    p81_pitch_SET((float) -7.646426E37F, PH.base.pack) ;
    p81_yaw_SET((float)7.6020535E37F, PH.base.pack) ;
    p81_thrust_SET((float)2.9232408E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)3034750159L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    {
        float  q [] =  {2.0565282E38F, -1.5426344E38F, 1.6932946E38F, -7.373143E36F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -3.2152186E36F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)3.0187276E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -3.338031E38F, PH.base.pack) ;
    p82_thrust_SET((float) -2.9868826E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2004785614L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    {
        float  q [] =  {-1.3488595E38F, 5.08099E37F, 2.9992717E38F, -3.3166476E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)2.8485397E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)1.1007535E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -1.1384232E38F, PH.base.pack) ;
    p83_thrust_SET((float) -1.581679E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1240337382L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)26395, PH.base.pack) ;
    p84_x_SET((float)4.6015387E37F, PH.base.pack) ;
    p84_y_SET((float)1.7036786E38F, PH.base.pack) ;
    p84_z_SET((float) -2.182325E38F, PH.base.pack) ;
    p84_vx_SET((float) -3.143199E37F, PH.base.pack) ;
    p84_vy_SET((float) -3.0322796E37F, PH.base.pack) ;
    p84_vz_SET((float)5.811679E37F, PH.base.pack) ;
    p84_afx_SET((float) -2.1111536E38F, PH.base.pack) ;
    p84_afy_SET((float)1.1197601E38F, PH.base.pack) ;
    p84_afz_SET((float) -1.4856605E38F, PH.base.pack) ;
    p84_yaw_SET((float) -6.069875E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)3.2682758E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)989001620L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)64777, PH.base.pack) ;
    p86_lat_int_SET((int32_t)1209323302, PH.base.pack) ;
    p86_lon_int_SET((int32_t)1034702432, PH.base.pack) ;
    p86_alt_SET((float) -2.4547303E38F, PH.base.pack) ;
    p86_vx_SET((float) -9.795461E37F, PH.base.pack) ;
    p86_vy_SET((float) -2.878576E38F, PH.base.pack) ;
    p86_vz_SET((float)1.5872075E38F, PH.base.pack) ;
    p86_afx_SET((float)2.4320098E38F, PH.base.pack) ;
    p86_afy_SET((float) -3.0689728E38F, PH.base.pack) ;
    p86_afz_SET((float) -9.272694E37F, PH.base.pack) ;
    p86_yaw_SET((float)2.8427724E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -1.7660639E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2487728127L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)23832, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -409932941, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -63007863, PH.base.pack) ;
    p87_alt_SET((float) -2.6785638E38F, PH.base.pack) ;
    p87_vx_SET((float)2.4671963E38F, PH.base.pack) ;
    p87_vy_SET((float) -2.9722472E38F, PH.base.pack) ;
    p87_vz_SET((float) -2.7354102E37F, PH.base.pack) ;
    p87_afx_SET((float)8.503686E37F, PH.base.pack) ;
    p87_afy_SET((float)9.617801E37F, PH.base.pack) ;
    p87_afz_SET((float)1.4318789E38F, PH.base.pack) ;
    p87_yaw_SET((float) -1.823795E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -3.3062244E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)2381954300L, PH.base.pack) ;
    p89_x_SET((float)2.2745582E38F, PH.base.pack) ;
    p89_y_SET((float) -2.9094382E38F, PH.base.pack) ;
    p89_z_SET((float)2.5764723E38F, PH.base.pack) ;
    p89_roll_SET((float) -7.558889E37F, PH.base.pack) ;
    p89_pitch_SET((float)3.1197364E38F, PH.base.pack) ;
    p89_yaw_SET((float) -1.7202658E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)100095086922651635L, PH.base.pack) ;
    p90_roll_SET((float) -1.8029545E38F, PH.base.pack) ;
    p90_pitch_SET((float) -1.5252143E38F, PH.base.pack) ;
    p90_yaw_SET((float)1.7316407E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -1.8083914E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.9602489E37F, PH.base.pack) ;
    p90_yawspeed_SET((float) -3.084627E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)2026433968, PH.base.pack) ;
    p90_lon_SET((int32_t)1366771768, PH.base.pack) ;
    p90_alt_SET((int32_t)624769624, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)30124, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)17444, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)2339, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)5960, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)3849, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)6072, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)8276871129802051543L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)7.783939E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -2.9579124E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.8692488E38F, PH.base.pack) ;
    p91_throttle_SET((float) -7.4505297E37F, PH.base.pack) ;
    p91_aux1_SET((float) -1.7647643E38F, PH.base.pack) ;
    p91_aux2_SET((float) -3.3074777E38F, PH.base.pack) ;
    p91_aux3_SET((float) -1.1185842E38F, PH.base.pack) ;
    p91_aux4_SET((float)5.8232197E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)1518834563973811230L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)50234, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)52682, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)42167, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)60887, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)19015, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)5868, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)42529, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)18875, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)37327, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)4648, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)55570, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)55146, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)8805831702339712601L, PH.base.pack) ;
    {
        float  controls [] =  {-1.2346399E38F, -8.790376E37F, 2.687297E38F, 3.676243E37F, 2.1178886E38F, 1.2721424E38F, 2.8730206E38F, -1.1984754E38F, 1.7195289E38F, -1.3997221E38F, -1.0616649E38F, 2.4534412E38F, 1.6632277E38F, 1.9286282E38F, 9.096921E37F, 1.6763626E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)5683866254618606130L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)2919114677870727774L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)20107, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)32215, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)2.3057386E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)3.1919805E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p100_ground_distance_SET((float) -3.2511642E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -2.2345256E36F, &PH) ;
    p100_flow_rate_y_SET((float)3.2446365E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)1000955746612086361L, PH.base.pack) ;
    p101_x_SET((float)4.34888E37F, PH.base.pack) ;
    p101_y_SET((float)2.8799762E38F, PH.base.pack) ;
    p101_z_SET((float)3.43275E37F, PH.base.pack) ;
    p101_roll_SET((float)3.243035E36F, PH.base.pack) ;
    p101_pitch_SET((float) -1.7113912E38F, PH.base.pack) ;
    p101_yaw_SET((float) -1.1750446E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)2704424078576916277L, PH.base.pack) ;
    p102_x_SET((float) -2.6065744E38F, PH.base.pack) ;
    p102_y_SET((float)1.1999519E38F, PH.base.pack) ;
    p102_z_SET((float) -1.4691145E38F, PH.base.pack) ;
    p102_roll_SET((float) -2.9459965E38F, PH.base.pack) ;
    p102_pitch_SET((float)6.217232E37F, PH.base.pack) ;
    p102_yaw_SET((float)1.2967062E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)7650824744372824994L, PH.base.pack) ;
    p103_x_SET((float)2.3620994E38F, PH.base.pack) ;
    p103_y_SET((float)2.5012627E38F, PH.base.pack) ;
    p103_z_SET((float) -1.7299936E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)671957216359701522L, PH.base.pack) ;
    p104_x_SET((float)1.8349766E38F, PH.base.pack) ;
    p104_y_SET((float)2.7540786E38F, PH.base.pack) ;
    p104_z_SET((float)1.8373078E38F, PH.base.pack) ;
    p104_roll_SET((float) -3.5821608E37F, PH.base.pack) ;
    p104_pitch_SET((float) -5.7533063E37F, PH.base.pack) ;
    p104_yaw_SET((float)3.081531E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)9105767507366174900L, PH.base.pack) ;
    p105_xacc_SET((float)2.3530562E38F, PH.base.pack) ;
    p105_yacc_SET((float) -1.4296466E38F, PH.base.pack) ;
    p105_zacc_SET((float)3.2653905E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -4.916843E37F, PH.base.pack) ;
    p105_ygyro_SET((float)1.4901666E38F, PH.base.pack) ;
    p105_zgyro_SET((float)3.038686E37F, PH.base.pack) ;
    p105_xmag_SET((float)1.3901198E38F, PH.base.pack) ;
    p105_ymag_SET((float) -1.4400856E38F, PH.base.pack) ;
    p105_zmag_SET((float)1.5212518E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float)2.5272752E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)2.5472707E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -1.8406386E38F, PH.base.pack) ;
    p105_temperature_SET((float) -2.201529E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)32233, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)3832947010805184741L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2459363773L, PH.base.pack) ;
    p106_integrated_x_SET((float) -5.466414E36F, PH.base.pack) ;
    p106_integrated_y_SET((float)8.921182E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.8525927E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)1.3033758E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)3.21736E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)26241, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2540127048L, PH.base.pack) ;
    p106_distance_SET((float)2.2651967E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)842902180459192279L, PH.base.pack) ;
    p107_xacc_SET((float) -3.1200078E38F, PH.base.pack) ;
    p107_yacc_SET((float) -2.4008499E37F, PH.base.pack) ;
    p107_zacc_SET((float) -2.982526E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.106648E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -3.098579E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -2.7480393E38F, PH.base.pack) ;
    p107_xmag_SET((float) -1.6836433E38F, PH.base.pack) ;
    p107_ymag_SET((float) -1.7395998E38F, PH.base.pack) ;
    p107_zmag_SET((float) -1.4982897E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.5965715E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -1.2294903E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -3.293446E38F, PH.base.pack) ;
    p107_temperature_SET((float) -1.7221396E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2128381586L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.524131E38F, PH.base.pack) ;
    p108_q2_SET((float) -1.2680501E38F, PH.base.pack) ;
    p108_q3_SET((float) -2.6684092E38F, PH.base.pack) ;
    p108_q4_SET((float)2.513792E38F, PH.base.pack) ;
    p108_roll_SET((float)8.355951E37F, PH.base.pack) ;
    p108_pitch_SET((float)9.055788E37F, PH.base.pack) ;
    p108_yaw_SET((float) -1.1650722E38F, PH.base.pack) ;
    p108_xacc_SET((float)2.750261E38F, PH.base.pack) ;
    p108_yacc_SET((float)1.3307842E38F, PH.base.pack) ;
    p108_zacc_SET((float)1.18586E38F, PH.base.pack) ;
    p108_xgyro_SET((float)8.5537374E36F, PH.base.pack) ;
    p108_ygyro_SET((float)1.1286145E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -2.8258009E38F, PH.base.pack) ;
    p108_lat_SET((float)6.4909866E37F, PH.base.pack) ;
    p108_lon_SET((float) -2.5644934E37F, PH.base.pack) ;
    p108_alt_SET((float)1.0194881E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)3.0629091E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -1.2031722E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.4421303E38F, PH.base.pack) ;
    p108_ve_SET((float)2.6848408E38F, PH.base.pack) ;
    p108_vd_SET((float) -1.155397E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)43952, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)15681, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)14, (uint8_t)96, (uint8_t)121, (uint8_t)46, (uint8_t)201, (uint8_t)0, (uint8_t)203, (uint8_t)231, (uint8_t)181, (uint8_t)156, (uint8_t)250, (uint8_t)74, (uint8_t)163, (uint8_t)177, (uint8_t)142, (uint8_t)94, (uint8_t)152, (uint8_t)115, (uint8_t)61, (uint8_t)251, (uint8_t)139, (uint8_t)170, (uint8_t)5, (uint8_t)41, (uint8_t)163, (uint8_t)242, (uint8_t)142, (uint8_t)10, (uint8_t)105, (uint8_t)224, (uint8_t)27, (uint8_t)203, (uint8_t)198, (uint8_t)30, (uint8_t)110, (uint8_t)65, (uint8_t)26, (uint8_t)0, (uint8_t)111, (uint8_t)112, (uint8_t)93, (uint8_t)34, (uint8_t)2, (uint8_t)115, (uint8_t)183, (uint8_t)37, (uint8_t)63, (uint8_t)156, (uint8_t)8, (uint8_t)19, (uint8_t)118, (uint8_t)83, (uint8_t)83, (uint8_t)215, (uint8_t)18, (uint8_t)125, (uint8_t)108, (uint8_t)11, (uint8_t)1, (uint8_t)241, (uint8_t)51, (uint8_t)251, (uint8_t)213, (uint8_t)191, (uint8_t)249, (uint8_t)130, (uint8_t)228, (uint8_t)232, (uint8_t)94, (uint8_t)80, (uint8_t)68, (uint8_t)5, (uint8_t)19, (uint8_t)19, (uint8_t)46, (uint8_t)67, (uint8_t)215, (uint8_t)170, (uint8_t)41, (uint8_t)131, (uint8_t)34, (uint8_t)153, (uint8_t)160, (uint8_t)230, (uint8_t)86, (uint8_t)184, (uint8_t)220, (uint8_t)4, (uint8_t)171, (uint8_t)53, (uint8_t)115, (uint8_t)72, (uint8_t)27, (uint8_t)17, (uint8_t)113, (uint8_t)183, (uint8_t)95, (uint8_t)202, (uint8_t)134, (uint8_t)108, (uint8_t)244, (uint8_t)150, (uint8_t)184, (uint8_t)28, (uint8_t)216, (uint8_t)185, (uint8_t)194, (uint8_t)109, (uint8_t)72, (uint8_t)129, (uint8_t)75, (uint8_t)173, (uint8_t)195, (uint8_t)181, (uint8_t)248, (uint8_t)117, (uint8_t)147, (uint8_t)1, (uint8_t)69, (uint8_t)25, (uint8_t)42, (uint8_t)208, (uint8_t)231, (uint8_t)111, (uint8_t)135, (uint8_t)193, (uint8_t)157, (uint8_t)143, (uint8_t)125, (uint8_t)227, (uint8_t)192, (uint8_t)194, (uint8_t)146, (uint8_t)116, (uint8_t)69, (uint8_t)131, (uint8_t)45, (uint8_t)149, (uint8_t)88, (uint8_t)31, (uint8_t)194, (uint8_t)244, (uint8_t)82, (uint8_t)229, (uint8_t)166, (uint8_t)210, (uint8_t)13, (uint8_t)184, (uint8_t)109, (uint8_t)39, (uint8_t)246, (uint8_t)104, (uint8_t)99, (uint8_t)235, (uint8_t)78, (uint8_t)243, (uint8_t)53, (uint8_t)70, (uint8_t)34, (uint8_t)164, (uint8_t)151, (uint8_t)149, (uint8_t)177, (uint8_t)96, (uint8_t)43, (uint8_t)208, (uint8_t)126, (uint8_t)93, (uint8_t)74, (uint8_t)60, (uint8_t)28, (uint8_t)28, (uint8_t)175, (uint8_t)55, (uint8_t)217, (uint8_t)55, (uint8_t)45, (uint8_t)58, (uint8_t)91, (uint8_t)40, (uint8_t)216, (uint8_t)15, (uint8_t)215, (uint8_t)120, (uint8_t)42, (uint8_t)98, (uint8_t)10, (uint8_t)206, (uint8_t)5, (uint8_t)25, (uint8_t)114, (uint8_t)98, (uint8_t)221, (uint8_t)73, (uint8_t)48, (uint8_t)249, (uint8_t)228, (uint8_t)45, (uint8_t)243, (uint8_t)112, (uint8_t)6, (uint8_t)24, (uint8_t)85, (uint8_t)174, (uint8_t)110, (uint8_t)120, (uint8_t)107, (uint8_t)64, (uint8_t)247, (uint8_t)134, (uint8_t)186, (uint8_t)154, (uint8_t)148, (uint8_t)86, (uint8_t)58, (uint8_t)253, (uint8_t)105, (uint8_t)167, (uint8_t)221, (uint8_t)123, (uint8_t)194, (uint8_t)232, (uint8_t)149, (uint8_t)126, (uint8_t)234, (uint8_t)211, (uint8_t)254, (uint8_t)116, (uint8_t)144, (uint8_t)108, (uint8_t)160, (uint8_t)213, (uint8_t)185, (uint8_t)162, (uint8_t)187, (uint8_t)55, (uint8_t)225, (uint8_t)82, (uint8_t)224, (uint8_t)63, (uint8_t)42, (uint8_t)242, (uint8_t)159, (uint8_t)246, (uint8_t)241, (uint8_t)244, (uint8_t)197, (uint8_t)15, (uint8_t)86, (uint8_t)53, (uint8_t)173};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -830685498907785120L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -608682603290553340L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)8691274444723338034L, PH.base.pack) ;
    p112_seq_SET((uint32_t)4185920755L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)2775627020649395165L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p113_lat_SET((int32_t) -403367038, PH.base.pack) ;
    p113_lon_SET((int32_t)1723277034, PH.base.pack) ;
    p113_alt_SET((int32_t) -1532293170, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)25130, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)64857, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)49514, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)24385, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -12721, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -2946, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)16885, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)2497809088562065869L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)2342420977L, PH.base.pack) ;
    p114_integrated_x_SET((float) -2.410606E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.4524585E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -1.1041553E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)5.702412E36F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)8.423773E37F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)27175, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)1611508718L, PH.base.pack) ;
    p114_distance_SET((float)7.8963594E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)6845482638089030444L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {1.8171157E38F, -7.122344E37F, -1.5885053E38F, -2.6989226E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)1.922892E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -1.3290817E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)3.251477E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1263073613, PH.base.pack) ;
    p115_lon_SET((int32_t)726763742, PH.base.pack) ;
    p115_alt_SET((int32_t) -669573636, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -25323, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -17428, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -1588, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)33035, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)49140, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)32453, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -23594, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)1318, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)3386554798L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -16471, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -6021, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)29259, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -32391, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -9783, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -23899, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -17524, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)16599, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)2373, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)25337, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)3134, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)2422, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)40666, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)55338, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)3657492681L, PH.base.pack) ;
    p118_size_SET((uint32_t)100234030L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)8574, PH.base.pack) ;
    p119_ofs_SET((uint32_t)369975682L, PH.base.pack) ;
    p119_count_SET((uint32_t)1011912265L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)9212, PH.base.pack) ;
    p120_ofs_SET((uint32_t)344724280L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)39, (uint8_t)193, (uint8_t)101, (uint8_t)11, (uint8_t)44, (uint8_t)35, (uint8_t)113, (uint8_t)37, (uint8_t)168, (uint8_t)170, (uint8_t)31, (uint8_t)71, (uint8_t)245, (uint8_t)31, (uint8_t)89, (uint8_t)172, (uint8_t)140, (uint8_t)88, (uint8_t)108, (uint8_t)26, (uint8_t)141, (uint8_t)238, (uint8_t)3, (uint8_t)115, (uint8_t)62, (uint8_t)68, (uint8_t)224, (uint8_t)225, (uint8_t)152, (uint8_t)124, (uint8_t)84, (uint8_t)244, (uint8_t)251, (uint8_t)220, (uint8_t)214, (uint8_t)220, (uint8_t)94, (uint8_t)94, (uint8_t)116, (uint8_t)122, (uint8_t)80, (uint8_t)43, (uint8_t)66, (uint8_t)223, (uint8_t)214, (uint8_t)41, (uint8_t)19, (uint8_t)79, (uint8_t)141, (uint8_t)14, (uint8_t)91, (uint8_t)138, (uint8_t)210, (uint8_t)234, (uint8_t)204, (uint8_t)64, (uint8_t)183, (uint8_t)57, (uint8_t)51, (uint8_t)78, (uint8_t)133, (uint8_t)157, (uint8_t)205, (uint8_t)150, (uint8_t)13, (uint8_t)4, (uint8_t)139, (uint8_t)196, (uint8_t)56, (uint8_t)110, (uint8_t)93, (uint8_t)224, (uint8_t)41, (uint8_t)104, (uint8_t)195, (uint8_t)6, (uint8_t)229, (uint8_t)226, (uint8_t)214, (uint8_t)252, (uint8_t)56, (uint8_t)176, (uint8_t)167, (uint8_t)247, (uint8_t)181, (uint8_t)131, (uint8_t)222, (uint8_t)31, (uint8_t)170, (uint8_t)110};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)222, (uint8_t)188, (uint8_t)234, (uint8_t)59, (uint8_t)236, (uint8_t)82, (uint8_t)44, (uint8_t)167, (uint8_t)83, (uint8_t)85, (uint8_t)96, (uint8_t)74, (uint8_t)236, (uint8_t)147, (uint8_t)250, (uint8_t)134, (uint8_t)204, (uint8_t)250, (uint8_t)140, (uint8_t)82, (uint8_t)17, (uint8_t)160, (uint8_t)219, (uint8_t)201, (uint8_t)168, (uint8_t)193, (uint8_t)205, (uint8_t)39, (uint8_t)164, (uint8_t)9, (uint8_t)242, (uint8_t)249, (uint8_t)59, (uint8_t)45, (uint8_t)88, (uint8_t)207, (uint8_t)107, (uint8_t)107, (uint8_t)183, (uint8_t)107, (uint8_t)27, (uint8_t)104, (uint8_t)107, (uint8_t)156, (uint8_t)133, (uint8_t)48, (uint8_t)239, (uint8_t)137, (uint8_t)208, (uint8_t)220, (uint8_t)119, (uint8_t)250, (uint8_t)224, (uint8_t)185, (uint8_t)139, (uint8_t)210, (uint8_t)174, (uint8_t)113, (uint8_t)254, (uint8_t)4, (uint8_t)157, (uint8_t)72, (uint8_t)61, (uint8_t)167, (uint8_t)139, (uint8_t)200, (uint8_t)249, (uint8_t)41, (uint8_t)254, (uint8_t)170, (uint8_t)10, (uint8_t)203, (uint8_t)78, (uint8_t)70, (uint8_t)4, (uint8_t)201, (uint8_t)177, (uint8_t)79, (uint8_t)54, (uint8_t)245, (uint8_t)157, (uint8_t)125, (uint8_t)24, (uint8_t)220, (uint8_t)108, (uint8_t)234, (uint8_t)32, (uint8_t)12, (uint8_t)110, (uint8_t)87, (uint8_t)13, (uint8_t)140, (uint8_t)41, (uint8_t)106, (uint8_t)194, (uint8_t)75, (uint8_t)97, (uint8_t)152, (uint8_t)121, (uint8_t)111, (uint8_t)216, (uint8_t)237, (uint8_t)234, (uint8_t)129, (uint8_t)188, (uint8_t)221, (uint8_t)60, (uint8_t)167, (uint8_t)77, (uint8_t)170};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)8970100615545670614L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t) -1832263977, PH.base.pack) ;
    p124_lon_SET((int32_t)407305280, PH.base.pack) ;
    p124_alt_SET((int32_t) -2078587763, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)21388, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)20288, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)31514, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)25152, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1938868451L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)48901, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)47450, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)38103, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)952345210L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)14, (uint8_t)252, (uint8_t)213, (uint8_t)104, (uint8_t)5, (uint8_t)42, (uint8_t)88, (uint8_t)100, (uint8_t)6, (uint8_t)218, (uint8_t)249, (uint8_t)99, (uint8_t)9, (uint8_t)47, (uint8_t)20, (uint8_t)56, (uint8_t)181, (uint8_t)81, (uint8_t)166, (uint8_t)62, (uint8_t)150, (uint8_t)94, (uint8_t)164, (uint8_t)62, (uint8_t)59, (uint8_t)231, (uint8_t)162, (uint8_t)72, (uint8_t)186, (uint8_t)10, (uint8_t)111, (uint8_t)36, (uint8_t)64, (uint8_t)134, (uint8_t)37, (uint8_t)4, (uint8_t)73, (uint8_t)108, (uint8_t)19, (uint8_t)65, (uint8_t)210, (uint8_t)230, (uint8_t)191, (uint8_t)125, (uint8_t)140, (uint8_t)95, (uint8_t)108, (uint8_t)24, (uint8_t)122, (uint8_t)183, (uint8_t)17, (uint8_t)161, (uint8_t)189, (uint8_t)208, (uint8_t)17, (uint8_t)77, (uint8_t)105, (uint8_t)79, (uint8_t)46, (uint8_t)191, (uint8_t)40, (uint8_t)201, (uint8_t)252, (uint8_t)33, (uint8_t)61, (uint8_t)60, (uint8_t)92, (uint8_t)96, (uint8_t)8, (uint8_t)73};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)2177686656L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)45087, PH.base.pack) ;
    p127_tow_SET((uint32_t)723742601L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)568977111, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -1988941395, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)976304075, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3917235790L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -936156737, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)1723510977L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)6137, PH.base.pack) ;
    p128_tow_SET((uint32_t)2286283818L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1273899339, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)1403991676, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)712908778, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)3524194008L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)894579672, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)917017279L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -21676, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -1863, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)15423, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)27829, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -2601, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)6714, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -23610, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -7356, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -25092, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p130_size_SET((uint32_t)554062654L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)56049, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)43733, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)33331, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)53162, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)251, (uint8_t)29, (uint8_t)182, (uint8_t)226, (uint8_t)1, (uint8_t)247, (uint8_t)191, (uint8_t)67, (uint8_t)49, (uint8_t)102, (uint8_t)4, (uint8_t)202, (uint8_t)132, (uint8_t)197, (uint8_t)68, (uint8_t)88, (uint8_t)112, (uint8_t)200, (uint8_t)30, (uint8_t)53, (uint8_t)170, (uint8_t)152, (uint8_t)239, (uint8_t)254, (uint8_t)199, (uint8_t)237, (uint8_t)150, (uint8_t)69, (uint8_t)73, (uint8_t)219, (uint8_t)90, (uint8_t)254, (uint8_t)201, (uint8_t)229, (uint8_t)76, (uint8_t)114, (uint8_t)64, (uint8_t)35, (uint8_t)117, (uint8_t)61, (uint8_t)139, (uint8_t)86, (uint8_t)228, (uint8_t)29, (uint8_t)155, (uint8_t)145, (uint8_t)41, (uint8_t)190, (uint8_t)3, (uint8_t)13, (uint8_t)88, (uint8_t)6, (uint8_t)111, (uint8_t)245, (uint8_t)233, (uint8_t)26, (uint8_t)222, (uint8_t)171, (uint8_t)100, (uint8_t)248, (uint8_t)86, (uint8_t)187, (uint8_t)16, (uint8_t)132, (uint8_t)46, (uint8_t)22, (uint8_t)176, (uint8_t)115, (uint8_t)108, (uint8_t)84, (uint8_t)233, (uint8_t)215, (uint8_t)45, (uint8_t)156, (uint8_t)64, (uint8_t)141, (uint8_t)193, (uint8_t)150, (uint8_t)221, (uint8_t)149, (uint8_t)154, (uint8_t)59, (uint8_t)148, (uint8_t)142, (uint8_t)78, (uint8_t)152, (uint8_t)83, (uint8_t)188, (uint8_t)219, (uint8_t)4, (uint8_t)237, (uint8_t)199, (uint8_t)66, (uint8_t)127, (uint8_t)151, (uint8_t)41, (uint8_t)192, (uint8_t)119, (uint8_t)176, (uint8_t)127, (uint8_t)159, (uint8_t)62, (uint8_t)113, (uint8_t)248, (uint8_t)171, (uint8_t)192, (uint8_t)207, (uint8_t)171, (uint8_t)109, (uint8_t)120, (uint8_t)149, (uint8_t)160, (uint8_t)205, (uint8_t)223, (uint8_t)125, (uint8_t)95, (uint8_t)221, (uint8_t)101, (uint8_t)133, (uint8_t)121, (uint8_t)161, (uint8_t)38, (uint8_t)249, (uint8_t)41, (uint8_t)92, (uint8_t)102, (uint8_t)86, (uint8_t)160, (uint8_t)88, (uint8_t)26, (uint8_t)54, (uint8_t)169, (uint8_t)5, (uint8_t)19, (uint8_t)78, (uint8_t)40, (uint8_t)150, (uint8_t)227, (uint8_t)191, (uint8_t)203, (uint8_t)168, (uint8_t)82, (uint8_t)3, (uint8_t)19, (uint8_t)216, (uint8_t)142, (uint8_t)78, (uint8_t)24, (uint8_t)217, (uint8_t)158, (uint8_t)184, (uint8_t)148, (uint8_t)191, (uint8_t)106, (uint8_t)46, (uint8_t)60, (uint8_t)166, (uint8_t)126, (uint8_t)190, (uint8_t)166, (uint8_t)154, (uint8_t)185, (uint8_t)117, (uint8_t)229, (uint8_t)4, (uint8_t)149, (uint8_t)254, (uint8_t)78, (uint8_t)211, (uint8_t)96, (uint8_t)125, (uint8_t)144, (uint8_t)247, (uint8_t)163, (uint8_t)143, (uint8_t)99, (uint8_t)149, (uint8_t)120, (uint8_t)207, (uint8_t)159, (uint8_t)75, (uint8_t)152, (uint8_t)50, (uint8_t)234, (uint8_t)15, (uint8_t)170, (uint8_t)110, (uint8_t)32, (uint8_t)71, (uint8_t)198, (uint8_t)29, (uint8_t)15, (uint8_t)186, (uint8_t)62, (uint8_t)139, (uint8_t)215, (uint8_t)224, (uint8_t)149, (uint8_t)38, (uint8_t)8, (uint8_t)45, (uint8_t)151, (uint8_t)21, (uint8_t)114, (uint8_t)243, (uint8_t)210, (uint8_t)144, (uint8_t)210, (uint8_t)63, (uint8_t)216, (uint8_t)209, (uint8_t)9, (uint8_t)159, (uint8_t)211, (uint8_t)90, (uint8_t)145, (uint8_t)243, (uint8_t)52, (uint8_t)170, (uint8_t)185, (uint8_t)25, (uint8_t)192, (uint8_t)75, (uint8_t)112, (uint8_t)25, (uint8_t)105, (uint8_t)60, (uint8_t)173, (uint8_t)57, (uint8_t)255, (uint8_t)201, (uint8_t)21, (uint8_t)190, (uint8_t)248, (uint8_t)68, (uint8_t)155, (uint8_t)7, (uint8_t)177, (uint8_t)127, (uint8_t)214, (uint8_t)16, (uint8_t)176, (uint8_t)186, (uint8_t)32, (uint8_t)116, (uint8_t)119, (uint8_t)206, (uint8_t)25, (uint8_t)212, (uint8_t)154, (uint8_t)225, (uint8_t)106, (uint8_t)25};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2426154147L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)53815, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)58726, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)21412, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_135, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -110574062, PH.base.pack) ;
    p133_lon_SET((int32_t) -1007301756, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)59797, PH.base.pack) ;
    p133_mask_SET((uint64_t)134344405265236275L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1292843240, PH.base.pack) ;
    p134_lon_SET((int32_t) -1094139496, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)59809, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)19439, (int16_t) -18336, (int16_t) -15559, (int16_t)10029, (int16_t)18498, (int16_t)23031, (int16_t)32103, (int16_t)8599, (int16_t)17809, (int16_t) -26928, (int16_t)20184, (int16_t) -3062, (int16_t) -5157, (int16_t)8671, (int16_t) -23012, (int16_t)7727};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)872029395, PH.base.pack) ;
    p135_lon_SET((int32_t)878067224, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1685374061, PH.base.pack) ;
    p136_lon_SET((int32_t) -13084057, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)48887, PH.base.pack) ;
    p136_terrain_height_SET((float)1.1699441E38F, PH.base.pack) ;
    p136_current_height_SET((float)2.8865976E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)21880, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)8093, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)1570213619L, PH.base.pack) ;
    p137_press_abs_SET((float)2.787743E38F, PH.base.pack) ;
    p137_press_diff_SET((float) -1.5257299E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -10245, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)4144765390270293749L, PH.base.pack) ;
    {
        float  q [] =  {2.1358773E38F, -1.8851325E38F, 2.506304E38F, -2.6070936E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -2.44136E38F, PH.base.pack) ;
    p138_y_SET((float) -1.6765616E37F, PH.base.pack) ;
    p138_z_SET((float) -1.6475112E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)1539752069332124323L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    {
        float  controls [] =  {2.0582305E38F, 3.0909646E38F, 3.3744344E37F, 1.0575506E38F, 1.6710575E38F, -1.3937097E38F, -2.786834E38F, -9.956005E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)1621465663768873456L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    {
        float  controls [] =  {-7.1220814E37F, -2.4979757E38F, 2.163306E38F, 8.1532366E37F, -3.0763917E38F, -1.5132098E38F, 3.2094684E38F, 9.958881E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)1456354030460763150L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -3.1249642E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -2.8775273E38F, PH.base.pack) ;
    p141_altitude_local_SET((float)3.2148704E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -3.2582582E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)1.6400674E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)8.859849E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)151, (uint8_t)76, (uint8_t)238, (uint8_t)96, (uint8_t)59, (uint8_t)244, (uint8_t)129, (uint8_t)113, (uint8_t)143, (uint8_t)245, (uint8_t)97, (uint8_t)142, (uint8_t)185, (uint8_t)146, (uint8_t)48, (uint8_t)156, (uint8_t)145, (uint8_t)167, (uint8_t)4, (uint8_t)172, (uint8_t)56, (uint8_t)184, (uint8_t)80, (uint8_t)55, (uint8_t)202, (uint8_t)88, (uint8_t)134, (uint8_t)38, (uint8_t)68, (uint8_t)92, (uint8_t)178, (uint8_t)245, (uint8_t)86, (uint8_t)69, (uint8_t)221, (uint8_t)248, (uint8_t)116, (uint8_t)247, (uint8_t)222, (uint8_t)128, (uint8_t)238, (uint8_t)255, (uint8_t)239, (uint8_t)150, (uint8_t)121, (uint8_t)241, (uint8_t)98, (uint8_t)7, (uint8_t)139, (uint8_t)165, (uint8_t)89, (uint8_t)255, (uint8_t)229, (uint8_t)210, (uint8_t)143, (uint8_t)52, (uint8_t)173, (uint8_t)213, (uint8_t)46, (uint8_t)113, (uint8_t)179, (uint8_t)154, (uint8_t)101, (uint8_t)88, (uint8_t)205, (uint8_t)45, (uint8_t)191, (uint8_t)43, (uint8_t)27, (uint8_t)160, (uint8_t)192, (uint8_t)214, (uint8_t)156, (uint8_t)35, (uint8_t)81, (uint8_t)83, (uint8_t)127, (uint8_t)224, (uint8_t)0, (uint8_t)32, (uint8_t)149, (uint8_t)62, (uint8_t)244, (uint8_t)174, (uint8_t)77, (uint8_t)99, (uint8_t)182, (uint8_t)255, (uint8_t)230, (uint8_t)228, (uint8_t)11, (uint8_t)169, (uint8_t)194, (uint8_t)168, (uint8_t)28, (uint8_t)101, (uint8_t)158, (uint8_t)142, (uint8_t)63, (uint8_t)155, (uint8_t)143, (uint8_t)73, (uint8_t)173, (uint8_t)118, (uint8_t)238, (uint8_t)149, (uint8_t)129, (uint8_t)113, (uint8_t)248, (uint8_t)177, (uint8_t)71, (uint8_t)93, (uint8_t)108, (uint8_t)226, (uint8_t)106, (uint8_t)172, (uint8_t)246, (uint8_t)50, (uint8_t)184, (uint8_t)62};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)233, (uint8_t)196, (uint8_t)156, (uint8_t)103, (uint8_t)212, (uint8_t)240, (uint8_t)16, (uint8_t)1, (uint8_t)186, (uint8_t)137, (uint8_t)242, (uint8_t)234, (uint8_t)246, (uint8_t)64, (uint8_t)250, (uint8_t)204, (uint8_t)79, (uint8_t)203, (uint8_t)17, (uint8_t)126, (uint8_t)175, (uint8_t)221, (uint8_t)231, (uint8_t)237, (uint8_t)32, (uint8_t)201, (uint8_t)135, (uint8_t)183, (uint8_t)210, (uint8_t)206, (uint8_t)122, (uint8_t)230, (uint8_t)135, (uint8_t)137, (uint8_t)86, (uint8_t)123, (uint8_t)182, (uint8_t)40, (uint8_t)210, (uint8_t)219, (uint8_t)100, (uint8_t)10, (uint8_t)147, (uint8_t)213, (uint8_t)178, (uint8_t)222, (uint8_t)75, (uint8_t)151, (uint8_t)119, (uint8_t)137, (uint8_t)108, (uint8_t)226, (uint8_t)4, (uint8_t)14, (uint8_t)55, (uint8_t)234, (uint8_t)73, (uint8_t)235, (uint8_t)175, (uint8_t)183, (uint8_t)58, (uint8_t)11, (uint8_t)50, (uint8_t)9, (uint8_t)131, (uint8_t)198, (uint8_t)75, (uint8_t)187, (uint8_t)76, (uint8_t)138, (uint8_t)64, (uint8_t)110, (uint8_t)68, (uint8_t)179, (uint8_t)102, (uint8_t)4, (uint8_t)214, (uint8_t)6, (uint8_t)122, (uint8_t)48, (uint8_t)24, (uint8_t)137, (uint8_t)114, (uint8_t)62, (uint8_t)202, (uint8_t)177, (uint8_t)219, (uint8_t)213, (uint8_t)71, (uint8_t)139, (uint8_t)161, (uint8_t)158, (uint8_t)91, (uint8_t)27, (uint8_t)242, (uint8_t)17, (uint8_t)51, (uint8_t)149, (uint8_t)24, (uint8_t)224, (uint8_t)228, (uint8_t)192, (uint8_t)116, (uint8_t)78, (uint8_t)17, (uint8_t)160, (uint8_t)139, (uint8_t)146, (uint8_t)29, (uint8_t)76, (uint8_t)159, (uint8_t)234, (uint8_t)128, (uint8_t)119, (uint8_t)241, (uint8_t)81, (uint8_t)229, (uint8_t)168, (uint8_t)94, (uint8_t)147};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)3612262941L, PH.base.pack) ;
    p143_press_abs_SET((float)1.1827268E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -2.2902761E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)7756, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)3723008227556733388L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p144_lat_SET((int32_t)2108427938, PH.base.pack) ;
    p144_lon_SET((int32_t) -845127295, PH.base.pack) ;
    p144_alt_SET((float) -3.4021907E38F, PH.base.pack) ;
    {
        float  vel [] =  {7.254184E37F, 2.1972277E38F, -1.9317638E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-2.8045697E38F, 3.1529576E38F, -7.837866E37F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {2.1973711E38F, -2.7317518E38F, -2.2330318E38F, -2.1026036E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.3256505E38F, 2.7623524E38F, 1.1072574E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {1.3840014E38F, 2.0096629E38F, -8.660389E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)4074668469203938236L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)8666276027543761148L, PH.base.pack) ;
    p146_x_acc_SET((float) -8.905827E37F, PH.base.pack) ;
    p146_y_acc_SET((float)7.6013887E37F, PH.base.pack) ;
    p146_z_acc_SET((float)3.3427425E38F, PH.base.pack) ;
    p146_x_vel_SET((float) -4.3165293E36F, PH.base.pack) ;
    p146_y_vel_SET((float) -1.9643254E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -1.7429265E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -3.3622347E38F, PH.base.pack) ;
    p146_y_pos_SET((float)2.5728034E38F, PH.base.pack) ;
    p146_z_pos_SET((float)1.3557065E38F, PH.base.pack) ;
    p146_airspeed_SET((float)2.6730796E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.0549826E38F, -1.3756735E38F, -1.2163395E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.8067587E38F, -7.7973524E36F, -3.0490003E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-1.4852668E38F, -2.1592801E38F, 3.2808126E38F, -2.829219E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -6.581706E37F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -1.6890159E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)6.1011947E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -19379, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)14986, (uint16_t)46023, (uint16_t)36499, (uint16_t)24961, (uint16_t)36357, (uint16_t)3275, (uint16_t)28346, (uint16_t)28484, (uint16_t)50781, (uint16_t)30};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)22302, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -1536905033, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)1017145008, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)2581036291L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)1534709517L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)2188260633L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)778063468L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)39, (uint8_t)134, (uint8_t)116, (uint8_t)27, (uint8_t)183, (uint8_t)28, (uint8_t)207, (uint8_t)185};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)131, (uint8_t)151, (uint8_t)73, (uint8_t)224, (uint8_t)107, (uint8_t)93, (uint8_t)243, (uint8_t)11};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)197, (uint8_t)152, (uint8_t)129, (uint8_t)80, (uint8_t)131, (uint8_t)194, (uint8_t)190, (uint8_t)141};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)58100, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)50315, PH.base.pack) ;
    p148_uid_SET((uint64_t)1575412420269061339L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)141, (uint8_t)108, (uint8_t)241, (uint8_t)107, (uint8_t)44, (uint8_t)119, (uint8_t)81, (uint8_t)153, (uint8_t)194, (uint8_t)23, (uint8_t)30, (uint8_t)162, (uint8_t)17, (uint8_t)95, (uint8_t)108, (uint8_t)219, (uint8_t)243, (uint8_t)170};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)8163706957017309798L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p149_angle_x_SET((float) -3.1226182E37F, PH.base.pack) ;
    p149_angle_y_SET((float)4.871679E37F, PH.base.pack) ;
    p149_distance_SET((float)1.35059E38F, PH.base.pack) ;
    p149_size_x_SET((float) -1.0340506E38F, PH.base.pack) ;
    p149_size_y_SET((float) -1.2823787E37F, PH.base.pack) ;
    p149_x_SET((float) -5.263246E37F, &PH) ;
    p149_y_SET((float)2.6650504E38F, &PH) ;
    p149_z_SET((float)1.75535E38F, &PH) ;
    {
        float  q [] =  {4.5044605E37F, -2.533686E38F, 2.0110484E38F, 1.5338783E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)187, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENSOR_OFFSETS_150(), &PH);
    p150_mag_ofs_x_SET((int16_t)(int16_t)1770, PH.base.pack) ;
    p150_mag_ofs_y_SET((int16_t)(int16_t) -29199, PH.base.pack) ;
    p150_mag_ofs_z_SET((int16_t)(int16_t) -6091, PH.base.pack) ;
    p150_mag_declination_SET((float)3.089695E38F, PH.base.pack) ;
    p150_raw_press_SET((int32_t)1341969453, PH.base.pack) ;
    p150_raw_temp_SET((int32_t) -1380063801, PH.base.pack) ;
    p150_gyro_cal_x_SET((float) -3.1200297E38F, PH.base.pack) ;
    p150_gyro_cal_y_SET((float) -2.8511076E38F, PH.base.pack) ;
    p150_gyro_cal_z_SET((float) -1.0741242E38F, PH.base.pack) ;
    p150_accel_cal_x_SET((float) -8.109864E37F, PH.base.pack) ;
    p150_accel_cal_y_SET((float)6.0252914E37F, PH.base.pack) ;
    p150_accel_cal_z_SET((float) -1.4281038E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MAG_OFFSETS_151(), &PH);
    p151_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p151_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p151_mag_ofs_x_SET((int16_t)(int16_t)18960, PH.base.pack) ;
    p151_mag_ofs_y_SET((int16_t)(int16_t) -12008, PH.base.pack) ;
    p151_mag_ofs_z_SET((int16_t)(int16_t) -4515, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMINFO_152(), &PH);
    p152_brkval_SET((uint16_t)(uint16_t)42818, PH.base.pack) ;
    p152_freemem_SET((uint16_t)(uint16_t)39793, PH.base.pack) ;
    p152_freemem32_SET((uint32_t)145501237L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AP_ADC_153(), &PH);
    p153_adc1_SET((uint16_t)(uint16_t)56444, PH.base.pack) ;
    p153_adc2_SET((uint16_t)(uint16_t)44017, PH.base.pack) ;
    p153_adc3_SET((uint16_t)(uint16_t)46464, PH.base.pack) ;
    p153_adc4_SET((uint16_t)(uint16_t)14314, PH.base.pack) ;
    p153_adc5_SET((uint16_t)(uint16_t)41418, PH.base.pack) ;
    p153_adc6_SET((uint16_t)(uint16_t)55045, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DIGICAM_CONFIGURE_154(), &PH);
    p154_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p154_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p154_mode_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p154_shutter_speed_SET((uint16_t)(uint16_t)34826, PH.base.pack) ;
    p154_aperture_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p154_iso_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p154_exposure_type_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p154_command_id_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p154_engine_cut_off_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p154_extra_param_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    p154_extra_value_SET((float)1.9130628E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DIGICAM_CONTROL_155(), &PH);
    p155_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p155_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p155_session_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p155_zoom_pos_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p155_zoom_step_SET((int8_t)(int8_t) -6, PH.base.pack) ;
    p155_focus_lock_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p155_shot_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    p155_command_id_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p155_extra_param_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p155_extra_value_SET((float) -2.437087E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_CONFIGURE_156(), &PH);
    p156_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p156_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p156_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL, PH.base.pack) ;
    p156_stab_roll_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p156_stab_pitch_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p156_stab_yaw_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_CONTROL_157(), &PH);
    p157_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p157_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p157_input_a_SET((int32_t)1069467835, PH.base.pack) ;
    p157_input_b_SET((int32_t)600008341, PH.base.pack) ;
    p157_input_c_SET((int32_t)1351427663, PH.base.pack) ;
    p157_save_position_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_STATUS_158(), &PH);
    p158_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p158_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p158_pointing_a_SET((int32_t)15593295, PH.base.pack) ;
    p158_pointing_b_SET((int32_t) -1929262455, PH.base.pack) ;
    p158_pointing_c_SET((int32_t) -1644818567, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FENCE_POINT_160(), &PH);
    p160_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p160_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p160_idx_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p160_count_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p160_lat_SET((float) -1.726714E38F, PH.base.pack) ;
    p160_lng_SET((float)6.247152E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FENCE_FETCH_POINT_161(), &PH);
    p161_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p161_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p161_idx_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FENCE_STATUS_162(), &PH);
    p162_breach_status_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    p162_breach_count_SET((uint16_t)(uint16_t)45119, PH.base.pack) ;
    p162_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_MINALT, PH.base.pack) ;
    p162_breach_time_SET((uint32_t)874663433L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AHRS_163(), &PH);
    p163_omegaIx_SET((float) -8.863034E37F, PH.base.pack) ;
    p163_omegaIy_SET((float)2.0631056E38F, PH.base.pack) ;
    p163_omegaIz_SET((float) -3.1357118E38F, PH.base.pack) ;
    p163_accel_weight_SET((float) -2.792978E38F, PH.base.pack) ;
    p163_renorm_val_SET((float)3.3640654E38F, PH.base.pack) ;
    p163_error_rp_SET((float) -1.734839E38F, PH.base.pack) ;
    p163_error_yaw_SET((float) -2.0022422E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIMSTATE_164(), &PH);
    p164_roll_SET((float)2.8523013E38F, PH.base.pack) ;
    p164_pitch_SET((float) -1.4285829E38F, PH.base.pack) ;
    p164_yaw_SET((float)1.6154898E38F, PH.base.pack) ;
    p164_xacc_SET((float)3.0242495E38F, PH.base.pack) ;
    p164_yacc_SET((float)9.426024E37F, PH.base.pack) ;
    p164_zacc_SET((float) -2.216259E38F, PH.base.pack) ;
    p164_xgyro_SET((float) -6.9955415E37F, PH.base.pack) ;
    p164_ygyro_SET((float) -3.121637E38F, PH.base.pack) ;
    p164_zgyro_SET((float)1.6868568E38F, PH.base.pack) ;
    p164_lat_SET((int32_t) -337662421, PH.base.pack) ;
    p164_lng_SET((int32_t)398944636, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HWSTATUS_165(), &PH);
    p165_Vcc_SET((uint16_t)(uint16_t)40253, PH.base.pack) ;
    p165_I2Cerr_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_166(), &PH);
    p166_rssi_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p166_remrssi_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p166_txbuf_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p166_noise_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p166_remnoise_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p166_rxerrors_SET((uint16_t)(uint16_t)57106, PH.base.pack) ;
    p166_fixed__SET((uint16_t)(uint16_t)56109, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LIMITS_STATUS_167(), &PH);
    p167_limits_state_SET(e_LIMITS_STATE_LIMITS_INIT, PH.base.pack) ;
    p167_last_trigger_SET((uint32_t)3014679755L, PH.base.pack) ;
    p167_last_action_SET((uint32_t)1468062666L, PH.base.pack) ;
    p167_last_recovery_SET((uint32_t)2606316241L, PH.base.pack) ;
    p167_last_clear_SET((uint32_t)3498207505L, PH.base.pack) ;
    p167_breach_count_SET((uint16_t)(uint16_t)2687, PH.base.pack) ;
    p167_mods_enabled_SET(e_LIMIT_MODULE_LIMIT_GEOFENCE, PH.base.pack) ;
    p167_mods_required_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, PH.base.pack) ;
    p167_mods_triggered_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_168(), &PH);
    p168_direction_SET((float) -3.2341168E38F, PH.base.pack) ;
    p168_speed_SET((float)1.2615131E38F, PH.base.pack) ;
    p168_speed_z_SET((float) -2.1225116E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA16_169(), &PH);
    p169_type_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p169_len_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)140, (uint8_t)231, (uint8_t)59, (uint8_t)221, (uint8_t)80, (uint8_t)155, (uint8_t)133, (uint8_t)50, (uint8_t)254, (uint8_t)100, (uint8_t)20, (uint8_t)194, (uint8_t)165, (uint8_t)157, (uint8_t)29, (uint8_t)226};
        p169_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA32_170(), &PH);
    p170_type_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p170_len_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)22, (uint8_t)246, (uint8_t)154, (uint8_t)53, (uint8_t)104, (uint8_t)181, (uint8_t)24, (uint8_t)28, (uint8_t)170, (uint8_t)124, (uint8_t)30, (uint8_t)146, (uint8_t)176, (uint8_t)117, (uint8_t)52, (uint8_t)163, (uint8_t)178, (uint8_t)230, (uint8_t)178, (uint8_t)240, (uint8_t)167, (uint8_t)234, (uint8_t)47, (uint8_t)83, (uint8_t)184, (uint8_t)9, (uint8_t)69, (uint8_t)194, (uint8_t)144, (uint8_t)34, (uint8_t)76, (uint8_t)25};
        p170_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA64_171(), &PH);
    p171_type_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    p171_len_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)37, (uint8_t)197, (uint8_t)205, (uint8_t)116, (uint8_t)200, (uint8_t)240, (uint8_t)189, (uint8_t)230, (uint8_t)87, (uint8_t)68, (uint8_t)52, (uint8_t)221, (uint8_t)77, (uint8_t)184, (uint8_t)80, (uint8_t)161, (uint8_t)198, (uint8_t)166, (uint8_t)148, (uint8_t)135, (uint8_t)7, (uint8_t)30, (uint8_t)116, (uint8_t)104, (uint8_t)99, (uint8_t)19, (uint8_t)29, (uint8_t)182, (uint8_t)170, (uint8_t)164, (uint8_t)133, (uint8_t)64, (uint8_t)94, (uint8_t)51, (uint8_t)128, (uint8_t)57, (uint8_t)22, (uint8_t)175, (uint8_t)222, (uint8_t)75, (uint8_t)78, (uint8_t)249, (uint8_t)198, (uint8_t)194, (uint8_t)177, (uint8_t)124, (uint8_t)85, (uint8_t)112, (uint8_t)192, (uint8_t)69, (uint8_t)183, (uint8_t)83, (uint8_t)132, (uint8_t)165, (uint8_t)166, (uint8_t)196, (uint8_t)25, (uint8_t)236, (uint8_t)172, (uint8_t)174, (uint8_t)127, (uint8_t)232, (uint8_t)11, (uint8_t)49};
        p171_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA96_172(), &PH);
    p172_type_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p172_len_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)127, (uint8_t)133, (uint8_t)78, (uint8_t)238, (uint8_t)196, (uint8_t)180, (uint8_t)232, (uint8_t)206, (uint8_t)192, (uint8_t)82, (uint8_t)8, (uint8_t)86, (uint8_t)71, (uint8_t)156, (uint8_t)170, (uint8_t)50, (uint8_t)217, (uint8_t)3, (uint8_t)85, (uint8_t)174, (uint8_t)125, (uint8_t)215, (uint8_t)156, (uint8_t)96, (uint8_t)166, (uint8_t)47, (uint8_t)60, (uint8_t)228, (uint8_t)1, (uint8_t)72, (uint8_t)3, (uint8_t)121, (uint8_t)99, (uint8_t)120, (uint8_t)218, (uint8_t)3, (uint8_t)157, (uint8_t)206, (uint8_t)98, (uint8_t)238, (uint8_t)10, (uint8_t)238, (uint8_t)115, (uint8_t)189, (uint8_t)130, (uint8_t)6, (uint8_t)12, (uint8_t)234, (uint8_t)148, (uint8_t)168, (uint8_t)194, (uint8_t)117, (uint8_t)78, (uint8_t)11, (uint8_t)29, (uint8_t)61, (uint8_t)88, (uint8_t)205, (uint8_t)30, (uint8_t)222, (uint8_t)185, (uint8_t)175, (uint8_t)54, (uint8_t)68, (uint8_t)211, (uint8_t)52, (uint8_t)5, (uint8_t)9, (uint8_t)208, (uint8_t)22, (uint8_t)204, (uint8_t)63, (uint8_t)123, (uint8_t)51, (uint8_t)253, (uint8_t)8, (uint8_t)152, (uint8_t)29, (uint8_t)189, (uint8_t)157, (uint8_t)124, (uint8_t)162, (uint8_t)42, (uint8_t)56, (uint8_t)65, (uint8_t)56, (uint8_t)75, (uint8_t)43, (uint8_t)143, (uint8_t)229, (uint8_t)57, (uint8_t)61, (uint8_t)156, (uint8_t)126, (uint8_t)78, (uint8_t)104};
        p172_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RANGEFINDER_173(), &PH);
    p173_distance_SET((float)1.5171014E37F, PH.base.pack) ;
    p173_voltage_SET((float) -2.8984776E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AIRSPEED_AUTOCAL_174(), &PH);
    p174_vx_SET((float)4.1504648E37F, PH.base.pack) ;
    p174_vy_SET((float) -8.681984E37F, PH.base.pack) ;
    p174_vz_SET((float)2.1275505E38F, PH.base.pack) ;
    p174_diff_pressure_SET((float)1.0719701E38F, PH.base.pack) ;
    p174_EAS2TAS_SET((float)2.0517343E38F, PH.base.pack) ;
    p174_ratio_SET((float) -3.079363E38F, PH.base.pack) ;
    p174_state_x_SET((float) -2.1553518E38F, PH.base.pack) ;
    p174_state_y_SET((float) -1.5534688E38F, PH.base.pack) ;
    p174_state_z_SET((float)1.4832725E38F, PH.base.pack) ;
    p174_Pax_SET((float) -1.5253592E38F, PH.base.pack) ;
    p174_Pby_SET((float) -3.062732E38F, PH.base.pack) ;
    p174_Pcz_SET((float)9.036443E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RALLY_POINT_175(), &PH);
    p175_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    p175_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p175_idx_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p175_count_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p175_lat_SET((int32_t) -426105810, PH.base.pack) ;
    p175_lng_SET((int32_t)1409800517, PH.base.pack) ;
    p175_alt_SET((int16_t)(int16_t) -32600, PH.base.pack) ;
    p175_break_alt_SET((int16_t)(int16_t) -23865, PH.base.pack) ;
    p175_land_dir_SET((uint16_t)(uint16_t)8386, PH.base.pack) ;
    p175_flags_SET(e_RALLY_FLAGS_LAND_IMMEDIATELY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RALLY_FETCH_POINT_176(), &PH);
    p176_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p176_target_component_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p176_idx_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMPASSMOT_STATUS_177(), &PH);
    p177_throttle_SET((uint16_t)(uint16_t)25418, PH.base.pack) ;
    p177_current_SET((float)1.0417737E38F, PH.base.pack) ;
    p177_interference_SET((uint16_t)(uint16_t)49570, PH.base.pack) ;
    p177_CompensationX_SET((float) -3.1123169E38F, PH.base.pack) ;
    p177_CompensationY_SET((float)2.461472E38F, PH.base.pack) ;
    p177_CompensationZ_SET((float) -3.3356443E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AHRS2_178(), &PH);
    p178_roll_SET((float) -2.320773E38F, PH.base.pack) ;
    p178_pitch_SET((float)2.4260908E38F, PH.base.pack) ;
    p178_yaw_SET((float)1.0240134E38F, PH.base.pack) ;
    p178_altitude_SET((float)2.7927047E38F, PH.base.pack) ;
    p178_lat_SET((int32_t) -1925084129, PH.base.pack) ;
    p178_lng_SET((int32_t) -1660519942, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_STATUS_179(), &PH);
    p179_time_usec_SET((uint64_t)7203943303734384146L, PH.base.pack) ;
    p179_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p179_cam_idx_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p179_img_idx_SET((uint16_t)(uint16_t)46368, PH.base.pack) ;
    p179_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_DISCONNECT, PH.base.pack) ;
    p179_p1_SET((float) -3.3374766E38F, PH.base.pack) ;
    p179_p2_SET((float)1.7323944E38F, PH.base.pack) ;
    p179_p3_SET((float) -9.716745E37F, PH.base.pack) ;
    p179_p4_SET((float)3.645232E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_FEEDBACK_180(), &PH);
    p180_time_usec_SET((uint64_t)1896977499797967563L, PH.base.pack) ;
    p180_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p180_cam_idx_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    p180_img_idx_SET((uint16_t)(uint16_t)9987, PH.base.pack) ;
    p180_lat_SET((int32_t) -908872151, PH.base.pack) ;
    p180_lng_SET((int32_t) -202511300, PH.base.pack) ;
    p180_alt_msl_SET((float) -1.1892164E38F, PH.base.pack) ;
    p180_alt_rel_SET((float)2.760855E38F, PH.base.pack) ;
    p180_roll_SET((float) -2.6545494E38F, PH.base.pack) ;
    p180_pitch_SET((float)3.4880718E36F, PH.base.pack) ;
    p180_yaw_SET((float) -1.1237756E37F, PH.base.pack) ;
    p180_foc_len_SET((float)1.5038611E38F, PH.base.pack) ;
    p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY2_181(), &PH);
    p181_voltage_SET((uint16_t)(uint16_t)17440, PH.base.pack) ;
    p181_current_battery_SET((int16_t)(int16_t)27507, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AHRS3_182(), &PH);
    p182_roll_SET((float)2.2888026E38F, PH.base.pack) ;
    p182_pitch_SET((float)1.3978117E38F, PH.base.pack) ;
    p182_yaw_SET((float) -1.2709685E37F, PH.base.pack) ;
    p182_altitude_SET((float) -2.4894031E38F, PH.base.pack) ;
    p182_lat_SET((int32_t)370699438, PH.base.pack) ;
    p182_lng_SET((int32_t) -229486963, PH.base.pack) ;
    p182_v1_SET((float) -6.255554E37F, PH.base.pack) ;
    p182_v2_SET((float)1.8087013E38F, PH.base.pack) ;
    p182_v3_SET((float) -1.0782062E38F, PH.base.pack) ;
    p182_v4_SET((float)1.3361268E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_REQUEST_183(), &PH);
    p183_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p183_target_component_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REMOTE_LOG_DATA_BLOCK_184(), &PH);
    p184_target_system_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p184_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_START, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)242, (uint8_t)131, (uint8_t)127, (uint8_t)210, (uint8_t)73, (uint8_t)39, (uint8_t)118, (uint8_t)249, (uint8_t)178, (uint8_t)57, (uint8_t)31, (uint8_t)88, (uint8_t)235, (uint8_t)0, (uint8_t)148, (uint8_t)37, (uint8_t)144, (uint8_t)197, (uint8_t)74, (uint8_t)83, (uint8_t)141, (uint8_t)73, (uint8_t)170, (uint8_t)195, (uint8_t)47, (uint8_t)76, (uint8_t)66, (uint8_t)33, (uint8_t)3, (uint8_t)210, (uint8_t)131, (uint8_t)79, (uint8_t)214, (uint8_t)1, (uint8_t)163, (uint8_t)251, (uint8_t)194, (uint8_t)248, (uint8_t)144, (uint8_t)73, (uint8_t)213, (uint8_t)35, (uint8_t)153, (uint8_t)250, (uint8_t)79, (uint8_t)247, (uint8_t)52, (uint8_t)72, (uint8_t)177, (uint8_t)37, (uint8_t)217, (uint8_t)51, (uint8_t)106, (uint8_t)154, (uint8_t)100, (uint8_t)242, (uint8_t)184, (uint8_t)48, (uint8_t)119, (uint8_t)13, (uint8_t)248, (uint8_t)202, (uint8_t)220, (uint8_t)233, (uint8_t)162, (uint8_t)65, (uint8_t)114, (uint8_t)18, (uint8_t)209, (uint8_t)45, (uint8_t)178, (uint8_t)90, (uint8_t)71, (uint8_t)36, (uint8_t)179, (uint8_t)207, (uint8_t)99, (uint8_t)240, (uint8_t)136, (uint8_t)186, (uint8_t)208, (uint8_t)39, (uint8_t)45, (uint8_t)160, (uint8_t)252, (uint8_t)3, (uint8_t)237, (uint8_t)73, (uint8_t)185, (uint8_t)133, (uint8_t)213, (uint8_t)214, (uint8_t)101, (uint8_t)160, (uint8_t)158, (uint8_t)14, (uint8_t)14, (uint8_t)158, (uint8_t)94, (uint8_t)254, (uint8_t)251, (uint8_t)7, (uint8_t)189, (uint8_t)39, (uint8_t)245, (uint8_t)130, (uint8_t)176, (uint8_t)198, (uint8_t)11, (uint8_t)187, (uint8_t)147, (uint8_t)9, (uint8_t)145, (uint8_t)213, (uint8_t)164, (uint8_t)82, (uint8_t)99, (uint8_t)153, (uint8_t)147, (uint8_t)65, (uint8_t)24, (uint8_t)205, (uint8_t)30, (uint8_t)58, (uint8_t)81, (uint8_t)73, (uint8_t)93, (uint8_t)218, (uint8_t)0, (uint8_t)231, (uint8_t)188, (uint8_t)205, (uint8_t)48, (uint8_t)91, (uint8_t)236, (uint8_t)175, (uint8_t)78, (uint8_t)220, (uint8_t)43, (uint8_t)97, (uint8_t)46, (uint8_t)177, (uint8_t)66, (uint8_t)178, (uint8_t)131, (uint8_t)242, (uint8_t)21, (uint8_t)2, (uint8_t)214, (uint8_t)129, (uint8_t)201, (uint8_t)19, (uint8_t)134, (uint8_t)238, (uint8_t)29, (uint8_t)214, (uint8_t)168, (uint8_t)121, (uint8_t)222, (uint8_t)13, (uint8_t)17, (uint8_t)227, (uint8_t)229, (uint8_t)125, (uint8_t)95, (uint8_t)119, (uint8_t)47, (uint8_t)236, (uint8_t)102, (uint8_t)175, (uint8_t)252, (uint8_t)202, (uint8_t)238, (uint8_t)184, (uint8_t)106, (uint8_t)233, (uint8_t)157, (uint8_t)34, (uint8_t)146, (uint8_t)176, (uint8_t)66, (uint8_t)82, (uint8_t)94, (uint8_t)19, (uint8_t)193, (uint8_t)8, (uint8_t)220, (uint8_t)121, (uint8_t)159, (uint8_t)58, (uint8_t)88, (uint8_t)107, (uint8_t)21, (uint8_t)86, (uint8_t)178, (uint8_t)240, (uint8_t)192, (uint8_t)166, (uint8_t)91, (uint8_t)191};
        p184_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REMOTE_LOG_BLOCK_STATUS_185(), &PH);
    p185_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p185_target_component_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p185_seqno_SET((uint32_t)1982003803L, PH.base.pack) ;
    p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LED_CONTROL_186(), &PH);
    p186_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p186_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p186_instance_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p186_pattern_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p186_custom_len_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    {
        uint8_t  custom_bytes [] =  {(uint8_t)57, (uint8_t)153, (uint8_t)44, (uint8_t)157, (uint8_t)137, (uint8_t)108, (uint8_t)110, (uint8_t)37, (uint8_t)69, (uint8_t)40, (uint8_t)198, (uint8_t)253, (uint8_t)190, (uint8_t)181, (uint8_t)54, (uint8_t)172, (uint8_t)146, (uint8_t)56, (uint8_t)210, (uint8_t)81, (uint8_t)144, (uint8_t)28, (uint8_t)57, (uint8_t)56};
        p186_custom_bytes_SET(&custom_bytes, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MAG_CAL_PROGRESS_191(), &PH);
    p191_compass_id_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p191_cal_mask_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p191_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, PH.base.pack) ;
    p191_attempt_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p191_completion_pct_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    {
        uint8_t  completion_mask [] =  {(uint8_t)105, (uint8_t)59, (uint8_t)14, (uint8_t)255, (uint8_t)179, (uint8_t)198, (uint8_t)212, (uint8_t)227, (uint8_t)51, (uint8_t)209};
        p191_completion_mask_SET(&completion_mask, 0, &PH.base.pack) ;
    }
    p191_direction_x_SET((float)1.6605941E38F, PH.base.pack) ;
    p191_direction_y_SET((float)5.987014E37F, PH.base.pack) ;
    p191_direction_z_SET((float) -2.0054374E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MAG_CAL_REPORT_192(), &PH);
    p192_compass_id_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p192_cal_mask_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p192_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START, PH.base.pack) ;
    p192_autosaved_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p192_fitness_SET((float)1.0415138E38F, PH.base.pack) ;
    p192_ofs_x_SET((float) -1.161588E38F, PH.base.pack) ;
    p192_ofs_y_SET((float) -7.9641594E37F, PH.base.pack) ;
    p192_ofs_z_SET((float)8.175608E37F, PH.base.pack) ;
    p192_diag_x_SET((float) -5.844945E37F, PH.base.pack) ;
    p192_diag_y_SET((float) -1.5261097E38F, PH.base.pack) ;
    p192_diag_z_SET((float) -2.3251507E38F, PH.base.pack) ;
    p192_offdiag_x_SET((float)3.2739271E38F, PH.base.pack) ;
    p192_offdiag_y_SET((float) -3.8611517E37F, PH.base.pack) ;
    p192_offdiag_z_SET((float)6.3514335E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EKF_STATUS_REPORT_193(), &PH);
    p193_flags_SET(e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL, PH.base.pack) ;
    p193_velocity_variance_SET((float) -1.720841E38F, PH.base.pack) ;
    p193_pos_horiz_variance_SET((float)4.5899632E36F, PH.base.pack) ;
    p193_pos_vert_variance_SET((float)8.0805627E37F, PH.base.pack) ;
    p193_compass_variance_SET((float)1.4332344E38F, PH.base.pack) ;
    p193_terrain_alt_variance_SET((float)3.1763174E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PID_TUNING_194(), &PH);
    p194_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_LANDING, PH.base.pack) ;
    p194_desired_SET((float) -2.6093486E38F, PH.base.pack) ;
    p194_achieved_SET((float) -1.8250287E38F, PH.base.pack) ;
    p194_FF_SET((float) -3.1938879E38F, PH.base.pack) ;
    p194_P_SET((float)3.3887035E38F, PH.base.pack) ;
    p194_I_SET((float) -1.0936037E38F, PH.base.pack) ;
    p194_D_SET((float) -5.492052E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GIMBAL_REPORT_200(), &PH);
    p200_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p200_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p200_delta_time_SET((float) -7.2699833E37F, PH.base.pack) ;
    p200_delta_angle_x_SET((float)1.3880865E38F, PH.base.pack) ;
    p200_delta_angle_y_SET((float)2.1878118E38F, PH.base.pack) ;
    p200_delta_angle_z_SET((float) -3.5060665E37F, PH.base.pack) ;
    p200_delta_velocity_x_SET((float) -3.2999748E38F, PH.base.pack) ;
    p200_delta_velocity_y_SET((float) -2.3985164E38F, PH.base.pack) ;
    p200_delta_velocity_z_SET((float)3.6153418E37F, PH.base.pack) ;
    p200_joint_roll_SET((float) -1.4739613E38F, PH.base.pack) ;
    p200_joint_el_SET((float)1.0216549E38F, PH.base.pack) ;
    p200_joint_az_SET((float)5.229547E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GIMBAL_CONTROL_201(), &PH);
    p201_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p201_target_component_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p201_demanded_rate_x_SET((float) -2.0649663E38F, PH.base.pack) ;
    p201_demanded_rate_y_SET((float)1.941876E38F, PH.base.pack) ;
    p201_demanded_rate_z_SET((float) -1.2426258E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GIMBAL_TORQUE_CMD_REPORT_214(), &PH);
    p214_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p214_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p214_rl_torque_cmd_SET((int16_t)(int16_t)28772, PH.base.pack) ;
    p214_el_torque_cmd_SET((int16_t)(int16_t) -3682, PH.base.pack) ;
    p214_az_torque_cmd_SET((int16_t)(int16_t) -11601, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GOPRO_HEARTBEAT_215(), &PH);
    p215_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_CONNECTED, PH.base.pack) ;
    p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP, PH.base.pack) ;
    p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GOPRO_GET_REQUEST_216(), &PH);
    p216_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p216_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p216_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GOPRO_GET_RESPONSE_217(), &PH);
    p217_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_CAPTURE_MODE, PH.base.pack) ;
    p217_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
    {
        uint8_t  value [] =  {(uint8_t)207, (uint8_t)237, (uint8_t)139, (uint8_t)1};
        p217_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GOPRO_SET_REQUEST_218(), &PH);
    p218_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p218_target_component_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p218_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, PH.base.pack) ;
    {
        uint8_t  value [] =  {(uint8_t)116, (uint8_t)201, (uint8_t)73, (uint8_t)48};
        p218_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GOPRO_SET_RESPONSE_219(), &PH);
    p219_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION, PH.base.pack) ;
    p219_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RPM_226(), &PH);
    p226_rpm1_SET((float)1.4071362E38F, PH.base.pack) ;
    p226_rpm2_SET((float)4.586658E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)820550761628798249L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS, PH.base.pack) ;
    p230_vel_ratio_SET((float) -4.0927502E37F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)2.79027E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)1.0264479E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -1.150139E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)2.8800336E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -2.4657536E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)2.835507E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)3.082611E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)5289903258195614585L, PH.base.pack) ;
    p231_wind_x_SET((float)1.1422856E38F, PH.base.pack) ;
    p231_wind_y_SET((float)2.6123955E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -5.6972533E37F, PH.base.pack) ;
    p231_var_horiz_SET((float)2.1025481E37F, PH.base.pack) ;
    p231_var_vert_SET((float) -6.8855647E37F, PH.base.pack) ;
    p231_wind_alt_SET((float) -2.2063676E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float) -3.029041E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)2.9618786E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)5581773586965562059L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)3463726227L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)57868, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p232_lat_SET((int32_t)1312005847, PH.base.pack) ;
    p232_lon_SET((int32_t)1619833122, PH.base.pack) ;
    p232_alt_SET((float)2.0966608E38F, PH.base.pack) ;
    p232_hdop_SET((float)1.7956987E37F, PH.base.pack) ;
    p232_vdop_SET((float) -2.5007195E38F, PH.base.pack) ;
    p232_vn_SET((float) -3.106812E38F, PH.base.pack) ;
    p232_ve_SET((float) -1.0323504E38F, PH.base.pack) ;
    p232_vd_SET((float) -1.2379972E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -1.874615E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -1.2029831E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)1.9670258E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)56, (uint8_t)87, (uint8_t)57, (uint8_t)88, (uint8_t)188, (uint8_t)146, (uint8_t)30, (uint8_t)236, (uint8_t)148, (uint8_t)9, (uint8_t)175, (uint8_t)88, (uint8_t)158, (uint8_t)241, (uint8_t)9, (uint8_t)120, (uint8_t)31, (uint8_t)64, (uint8_t)48, (uint8_t)145, (uint8_t)221, (uint8_t)90, (uint8_t)61, (uint8_t)92, (uint8_t)30, (uint8_t)32, (uint8_t)49, (uint8_t)188, (uint8_t)221, (uint8_t)186, (uint8_t)187, (uint8_t)116, (uint8_t)88, (uint8_t)135, (uint8_t)23, (uint8_t)8, (uint8_t)212, (uint8_t)67, (uint8_t)254, (uint8_t)145, (uint8_t)83, (uint8_t)199, (uint8_t)137, (uint8_t)27, (uint8_t)83, (uint8_t)176, (uint8_t)247, (uint8_t)144, (uint8_t)97, (uint8_t)100, (uint8_t)243, (uint8_t)129, (uint8_t)40, (uint8_t)238, (uint8_t)249, (uint8_t)231, (uint8_t)31, (uint8_t)64, (uint8_t)41, (uint8_t)143, (uint8_t)62, (uint8_t)183, (uint8_t)49, (uint8_t)25, (uint8_t)82, (uint8_t)176, (uint8_t)41, (uint8_t)170, (uint8_t)61, (uint8_t)238, (uint8_t)188, (uint8_t)191, (uint8_t)1, (uint8_t)40, (uint8_t)211, (uint8_t)38, (uint8_t)150, (uint8_t)80, (uint8_t)39, (uint8_t)53, (uint8_t)102, (uint8_t)97, (uint8_t)53, (uint8_t)81, (uint8_t)206, (uint8_t)37, (uint8_t)239, (uint8_t)255, (uint8_t)109, (uint8_t)149, (uint8_t)189, (uint8_t)184, (uint8_t)93, (uint8_t)139, (uint8_t)159, (uint8_t)126, (uint8_t)64, (uint8_t)31, (uint8_t)143, (uint8_t)228, (uint8_t)240, (uint8_t)155, (uint8_t)115, (uint8_t)248, (uint8_t)99, (uint8_t)230, (uint8_t)97, (uint8_t)156, (uint8_t)212, (uint8_t)179, (uint8_t)149, (uint8_t)125, (uint8_t)8, (uint8_t)224, (uint8_t)210, (uint8_t)227, (uint8_t)126, (uint8_t)151, (uint8_t)172, (uint8_t)170, (uint8_t)160, (uint8_t)122, (uint8_t)251, (uint8_t)65, (uint8_t)100, (uint8_t)229, (uint8_t)211, (uint8_t)203, (uint8_t)64, (uint8_t)162, (uint8_t)230, (uint8_t)110, (uint8_t)210, (uint8_t)135, (uint8_t)65, (uint8_t)39, (uint8_t)43, (uint8_t)171, (uint8_t)5, (uint8_t)77, (uint8_t)0, (uint8_t)0, (uint8_t)224, (uint8_t)127, (uint8_t)108, (uint8_t)76, (uint8_t)253, (uint8_t)14, (uint8_t)113, (uint8_t)62, (uint8_t)62, (uint8_t)246, (uint8_t)212, (uint8_t)146, (uint8_t)11, (uint8_t)135, (uint8_t)157, (uint8_t)124, (uint8_t)159, (uint8_t)4, (uint8_t)169, (uint8_t)178, (uint8_t)35, (uint8_t)172, (uint8_t)183, (uint8_t)61, (uint8_t)38, (uint8_t)138, (uint8_t)200, (uint8_t)100, (uint8_t)15, (uint8_t)188, (uint8_t)121, (uint8_t)150, (uint8_t)99, (uint8_t)13, (uint8_t)178, (uint8_t)224, (uint8_t)163, (uint8_t)91};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)2782768391L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -11595, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -20400, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)48842, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -126, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -8394, PH.base.pack) ;
    p234_latitude_SET((int32_t)49442594, PH.base.pack) ;
    p234_longitude_SET((int32_t) -421563627, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)4846, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)10254, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)88, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t) -115, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)42, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)52931, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)4238057955105045796L, PH.base.pack) ;
    p241_vibration_x_SET((float)1.717929E38F, PH.base.pack) ;
    p241_vibration_y_SET((float)3.3686636E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)9.690696E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)254086248L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)3661187789L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)1274449597L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)1185921728, PH.base.pack) ;
    p242_longitude_SET((int32_t) -2048981557, PH.base.pack) ;
    p242_altitude_SET((int32_t)254818863, PH.base.pack) ;
    p242_x_SET((float)2.6953174E38F, PH.base.pack) ;
    p242_y_SET((float) -1.5439293E38F, PH.base.pack) ;
    p242_z_SET((float)1.2607468E38F, PH.base.pack) ;
    {
        float  q [] =  {-1.2697791E38F, -2.4499907E38F, 1.2569669E38F, -3.6967128E37F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -3.139502E38F, PH.base.pack) ;
    p242_approach_y_SET((float)7.552654E37F, PH.base.pack) ;
    p242_approach_z_SET((float) -2.1852264E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)9098923822663929823L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p243_latitude_SET((int32_t) -272900681, PH.base.pack) ;
    p243_longitude_SET((int32_t)894238789, PH.base.pack) ;
    p243_altitude_SET((int32_t) -395171485, PH.base.pack) ;
    p243_x_SET((float)1.9290385E38F, PH.base.pack) ;
    p243_y_SET((float) -2.7157994E38F, PH.base.pack) ;
    p243_z_SET((float)1.3620734E38F, PH.base.pack) ;
    {
        float  q [] =  {-2.286334E38F, 1.9958631E38F, -1.3216367E38F, 1.3291411E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -2.4131905E38F, PH.base.pack) ;
    p243_approach_y_SET((float)2.2884044E38F, PH.base.pack) ;
    p243_approach_z_SET((float) -1.8229304E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)1455916026424765481L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)22975, PH.base.pack) ;
    p244_interval_us_SET((int32_t)1630565713, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)4005886433L, PH.base.pack) ;
    p246_lat_SET((int32_t)957751219, PH.base.pack) ;
    p246_lon_SET((int32_t) -1837626028, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
    p246_altitude_SET((int32_t) -1875655692, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)1229, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)19757, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)9612, PH.base.pack) ;
    {
        char16_t   callsign = "fg";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SPACE, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)53161, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
    p247_id_SET((uint32_t)1584208987L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)1.5822795E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float)1.0254195E37F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float) -5.9528314E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)12330, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)35, (uint8_t)207, (uint8_t)186, (uint8_t)212, (uint8_t)85, (uint8_t)152, (uint8_t)133, (uint8_t)164, (uint8_t)167, (uint8_t)3, (uint8_t)70, (uint8_t)113, (uint8_t)61, (uint8_t)17, (uint8_t)134, (uint8_t)67, (uint8_t)179, (uint8_t)48, (uint8_t)49, (uint8_t)35, (uint8_t)118, (uint8_t)88, (uint8_t)85, (uint8_t)211, (uint8_t)225, (uint8_t)5, (uint8_t)226, (uint8_t)121, (uint8_t)36, (uint8_t)110, (uint8_t)151, (uint8_t)215, (uint8_t)108, (uint8_t)7, (uint8_t)82, (uint8_t)3, (uint8_t)74, (uint8_t)238, (uint8_t)239, (uint8_t)121, (uint8_t)90, (uint8_t)245, (uint8_t)192, (uint8_t)149, (uint8_t)140, (uint8_t)20, (uint8_t)41, (uint8_t)56, (uint8_t)87, (uint8_t)218, (uint8_t)60, (uint8_t)30, (uint8_t)205, (uint8_t)28, (uint8_t)203, (uint8_t)185, (uint8_t)166, (uint8_t)137, (uint8_t)143, (uint8_t)175, (uint8_t)137, (uint8_t)42, (uint8_t)112, (uint8_t)108, (uint8_t)219, (uint8_t)122, (uint8_t)181, (uint8_t)208, (uint8_t)165, (uint8_t)42, (uint8_t)222, (uint8_t)131, (uint8_t)202, (uint8_t)162, (uint8_t)170, (uint8_t)78, (uint8_t)36, (uint8_t)103, (uint8_t)227, (uint8_t)137, (uint8_t)113, (uint8_t)213, (uint8_t)175, (uint8_t)119, (uint8_t)86, (uint8_t)198, (uint8_t)80, (uint8_t)33, (uint8_t)21, (uint8_t)254, (uint8_t)11, (uint8_t)151, (uint8_t)72, (uint8_t)209, (uint8_t)65, (uint8_t)55, (uint8_t)71, (uint8_t)132, (uint8_t)143, (uint8_t)197, (uint8_t)146, (uint8_t)84, (uint8_t)164, (uint8_t)107, (uint8_t)17, (uint8_t)167, (uint8_t)8, (uint8_t)239, (uint8_t)245, (uint8_t)94, (uint8_t)207, (uint8_t)166, (uint8_t)178, (uint8_t)162, (uint8_t)12, (uint8_t)174, (uint8_t)24, (uint8_t)86, (uint8_t)169, (uint8_t)136, (uint8_t)27, (uint8_t)165, (uint8_t)42, (uint8_t)176, (uint8_t)96, (uint8_t)60, (uint8_t)157, (uint8_t)23, (uint8_t)219, (uint8_t)72, (uint8_t)40, (uint8_t)102, (uint8_t)76, (uint8_t)45, (uint8_t)179, (uint8_t)97, (uint8_t)40, (uint8_t)107, (uint8_t)59, (uint8_t)50, (uint8_t)228, (uint8_t)210, (uint8_t)175, (uint8_t)255, (uint8_t)199, (uint8_t)89, (uint8_t)92, (uint8_t)41, (uint8_t)82, (uint8_t)142, (uint8_t)169, (uint8_t)25, (uint8_t)52, (uint8_t)23, (uint8_t)132, (uint8_t)178, (uint8_t)64, (uint8_t)251, (uint8_t)158, (uint8_t)205, (uint8_t)126, (uint8_t)47, (uint8_t)168, (uint8_t)108, (uint8_t)70, (uint8_t)135, (uint8_t)112, (uint8_t)57, (uint8_t)69, (uint8_t)180, (uint8_t)4, (uint8_t)81, (uint8_t)115, (uint8_t)220, (uint8_t)2, (uint8_t)38, (uint8_t)8, (uint8_t)51, (uint8_t)239, (uint8_t)189, (uint8_t)243, (uint8_t)45, (uint8_t)44, (uint8_t)217, (uint8_t)223, (uint8_t)58, (uint8_t)185, (uint8_t)196, (uint8_t)38, (uint8_t)17, (uint8_t)148, (uint8_t)23, (uint8_t)252, (uint8_t)127, (uint8_t)10, (uint8_t)251, (uint8_t)221, (uint8_t)154, (uint8_t)139, (uint8_t)229, (uint8_t)116, (uint8_t)220, (uint8_t)108, (uint8_t)146, (uint8_t)110, (uint8_t)117, (uint8_t)4, (uint8_t)224, (uint8_t)7, (uint8_t)11, (uint8_t)208, (uint8_t)127, (uint8_t)136, (uint8_t)70, (uint8_t)149, (uint8_t)49, (uint8_t)40, (uint8_t)218, (uint8_t)3, (uint8_t)110, (uint8_t)187, (uint8_t)38, (uint8_t)13, (uint8_t)22, (uint8_t)167, (uint8_t)171, (uint8_t)139, (uint8_t)161, (uint8_t)69, (uint8_t)197, (uint8_t)207, (uint8_t)57, (uint8_t)94, (uint8_t)37, (uint8_t)10, (uint8_t)26, (uint8_t)240, (uint8_t)64, (uint8_t)116, (uint8_t)220, (uint8_t)26, (uint8_t)55, (uint8_t)157, (uint8_t)115, (uint8_t)69, (uint8_t)231, (uint8_t)173, (uint8_t)75, (uint8_t)80};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)31667, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)81, (int8_t)47, (int8_t)24, (int8_t) -65, (int8_t) -112, (int8_t) -118, (int8_t)97, (int8_t) -56, (int8_t)60, (int8_t) -119, (int8_t)23, (int8_t)117, (int8_t)83, (int8_t)117, (int8_t) -56, (int8_t) -117, (int8_t)69, (int8_t) -11, (int8_t)102, (int8_t)106, (int8_t)61, (int8_t) -101, (int8_t)39, (int8_t) -20, (int8_t) -55, (int8_t) -127, (int8_t)3, (int8_t)100, (int8_t) -16, (int8_t) -96, (int8_t) -123, (int8_t) -25};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "gX";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)8243186836178290093L, PH.base.pack) ;
    p250_x_SET((float) -1.2047479E38F, PH.base.pack) ;
    p250_y_SET((float)2.2174069E38F, PH.base.pack) ;
    p250_z_SET((float) -7.6880945E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)60909250L, PH.base.pack) ;
    {
        char16_t   name = "lzsvos";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -2.6168635E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)4067889861L, PH.base.pack) ;
    {
        char16_t   name = "Mk";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t)1695146512, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
    {
        char16_t   text = "uwvbqwmaqCbuTmjdzumpn";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)2688023610L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p254_value_SET((float)1.7304562E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)255, (uint8_t)58, (uint8_t)101, (uint8_t)66, (uint8_t)113, (uint8_t)50, (uint8_t)16, (uint8_t)112, (uint8_t)146, (uint8_t)210, (uint8_t)27, (uint8_t)74, (uint8_t)87, (uint8_t)230, (uint8_t)84, (uint8_t)67, (uint8_t)189, (uint8_t)155, (uint8_t)30, (uint8_t)229, (uint8_t)39, (uint8_t)49, (uint8_t)187, (uint8_t)129, (uint8_t)218, (uint8_t)250, (uint8_t)5, (uint8_t)14, (uint8_t)36, (uint8_t)197, (uint8_t)169, (uint8_t)167};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)1817703205492013959L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)3441124940L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)1578385367L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    {
        char16_t   tune = "fbzmwe";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)4046347395L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)201, (uint8_t)224, (uint8_t)153, (uint8_t)70, (uint8_t)4, (uint8_t)137, (uint8_t)209, (uint8_t)60, (uint8_t)216, (uint8_t)140, (uint8_t)241, (uint8_t)111, (uint8_t)174, (uint8_t)25, (uint8_t)153, (uint8_t)2, (uint8_t)167, (uint8_t)20, (uint8_t)237, (uint8_t)215, (uint8_t)153, (uint8_t)180, (uint8_t)61, (uint8_t)160, (uint8_t)134, (uint8_t)188, (uint8_t)202, (uint8_t)42, (uint8_t)140, (uint8_t)156, (uint8_t)200, (uint8_t)78};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)77, (uint8_t)61, (uint8_t)41, (uint8_t)99, (uint8_t)173, (uint8_t)187, (uint8_t)37, (uint8_t)226, (uint8_t)181, (uint8_t)146, (uint8_t)182, (uint8_t)116, (uint8_t)152, (uint8_t)247, (uint8_t)250, (uint8_t)114, (uint8_t)77, (uint8_t)183, (uint8_t)78, (uint8_t)105, (uint8_t)146, (uint8_t)219, (uint8_t)114, (uint8_t)215, (uint8_t)221, (uint8_t)222, (uint8_t)158, (uint8_t)173, (uint8_t)202, (uint8_t)103, (uint8_t)91, (uint8_t)224};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)2106265431L, PH.base.pack) ;
    p259_focal_length_SET((float) -2.931126E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -9.306668E37F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -3.155447E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)33739, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)39519, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)40581, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "avflmnedhhqaiBukeiuywpafuuddmboqhw";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)690807394L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)1288560342L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p261_total_capacity_SET((float) -2.3850726E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)1.2862189E37F, PH.base.pack) ;
    p261_available_capacity_SET((float) -1.3182602E38F, PH.base.pack) ;
    p261_read_speed_SET((float)8.243184E37F, PH.base.pack) ;
    p261_write_speed_SET((float) -1.0148476E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)1805771025L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p262_image_interval_SET((float) -1.1312713E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)2271878827L, PH.base.pack) ;
    p262_available_capacity_SET((float)1.4681038E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)3115699349L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)8555479013607038991L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p263_lat_SET((int32_t) -365303613, PH.base.pack) ;
    p263_lon_SET((int32_t) -2008164322, PH.base.pack) ;
    p263_alt_SET((int32_t) -1826161648, PH.base.pack) ;
    p263_relative_alt_SET((int32_t) -2113004304, PH.base.pack) ;
    {
        float  q [] =  {1.8753674E38F, 2.5965038E38F, 1.1068075E38F, 9.072655E37F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1677744629, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)118, PH.base.pack) ;
    {
        char16_t   file_url = "cuyavmxVktdfvcpxkqkaDNxhtnhoeumelvtbjawoweocbajnErdcfxscclzrktfqzKaUcpcuomiyxpxcddiNDDmgyrsonhfslFdkjxMmvktcqjgivyddtgsdqsnJhqqnrxvfjfacrwfbojqobtxmckluhcirvyyBgEWuxxrpqzkjbjtosxzgjgdfjmztnyxsiuwkwcnftxh";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)2776145989L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)9206376407231526896L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)2423766238794888913L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)2595533913580680786L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)396462586L, PH.base.pack) ;
    p265_roll_SET((float) -1.3151086E38F, PH.base.pack) ;
    p265_pitch_SET((float) -2.7262672E38F, PH.base.pack) ;
    p265_yaw_SET((float)2.2331493E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)39645, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)40, (uint8_t)155, (uint8_t)82, (uint8_t)47, (uint8_t)26, (uint8_t)66, (uint8_t)94, (uint8_t)50, (uint8_t)165, (uint8_t)118, (uint8_t)35, (uint8_t)28, (uint8_t)47, (uint8_t)178, (uint8_t)151, (uint8_t)106, (uint8_t)108, (uint8_t)224, (uint8_t)88, (uint8_t)52, (uint8_t)137, (uint8_t)214, (uint8_t)209, (uint8_t)205, (uint8_t)138, (uint8_t)3, (uint8_t)221, (uint8_t)185, (uint8_t)106, (uint8_t)109, (uint8_t)68, (uint8_t)51, (uint8_t)121, (uint8_t)121, (uint8_t)180, (uint8_t)69, (uint8_t)80, (uint8_t)203, (uint8_t)102, (uint8_t)197, (uint8_t)55, (uint8_t)252, (uint8_t)213, (uint8_t)252, (uint8_t)19, (uint8_t)123, (uint8_t)204, (uint8_t)25, (uint8_t)41, (uint8_t)180, (uint8_t)208, (uint8_t)90, (uint8_t)178, (uint8_t)2, (uint8_t)234, (uint8_t)179, (uint8_t)3, (uint8_t)235, (uint8_t)48, (uint8_t)123, (uint8_t)179, (uint8_t)139, (uint8_t)159, (uint8_t)186, (uint8_t)40, (uint8_t)174, (uint8_t)166, (uint8_t)167, (uint8_t)100, (uint8_t)25, (uint8_t)61, (uint8_t)39, (uint8_t)69, (uint8_t)54, (uint8_t)214, (uint8_t)253, (uint8_t)71, (uint8_t)53, (uint8_t)226, (uint8_t)177, (uint8_t)74, (uint8_t)126, (uint8_t)228, (uint8_t)207, (uint8_t)206, (uint8_t)47, (uint8_t)219, (uint8_t)180, (uint8_t)9, (uint8_t)30, (uint8_t)36, (uint8_t)215, (uint8_t)16, (uint8_t)62, (uint8_t)107, (uint8_t)39, (uint8_t)39, (uint8_t)243, (uint8_t)133, (uint8_t)104, (uint8_t)225, (uint8_t)127, (uint8_t)212, (uint8_t)99, (uint8_t)39, (uint8_t)199, (uint8_t)251, (uint8_t)1, (uint8_t)143, (uint8_t)203, (uint8_t)139, (uint8_t)126, (uint8_t)62, (uint8_t)139, (uint8_t)217, (uint8_t)9, (uint8_t)167, (uint8_t)33, (uint8_t)74, (uint8_t)57, (uint8_t)41, (uint8_t)89, (uint8_t)187, (uint8_t)140, (uint8_t)75, (uint8_t)146, (uint8_t)54, (uint8_t)62, (uint8_t)86, (uint8_t)209, (uint8_t)174, (uint8_t)9, (uint8_t)13, (uint8_t)245, (uint8_t)77, (uint8_t)190, (uint8_t)18, (uint8_t)73, (uint8_t)248, (uint8_t)71, (uint8_t)5, (uint8_t)60, (uint8_t)121, (uint8_t)39, (uint8_t)128, (uint8_t)88, (uint8_t)132, (uint8_t)179, (uint8_t)65, (uint8_t)103, (uint8_t)175, (uint8_t)12, (uint8_t)159, (uint8_t)144, (uint8_t)158, (uint8_t)92, (uint8_t)115, (uint8_t)146, (uint8_t)130, (uint8_t)135, (uint8_t)205, (uint8_t)5, (uint8_t)206, (uint8_t)65, (uint8_t)162, (uint8_t)211, (uint8_t)135, (uint8_t)131, (uint8_t)4, (uint8_t)113, (uint8_t)46, (uint8_t)14, (uint8_t)10, (uint8_t)160, (uint8_t)122, (uint8_t)4, (uint8_t)168, (uint8_t)35, (uint8_t)176, (uint8_t)71, (uint8_t)82, (uint8_t)111, (uint8_t)0, (uint8_t)129, (uint8_t)189, (uint8_t)233, (uint8_t)233, (uint8_t)126, (uint8_t)252, (uint8_t)161, (uint8_t)55, (uint8_t)105, (uint8_t)56, (uint8_t)81, (uint8_t)224, (uint8_t)69, (uint8_t)115, (uint8_t)79, (uint8_t)146, (uint8_t)222, (uint8_t)117, (uint8_t)233, (uint8_t)121, (uint8_t)125, (uint8_t)244, (uint8_t)250, (uint8_t)182, (uint8_t)225, (uint8_t)74, (uint8_t)176, (uint8_t)48, (uint8_t)239, (uint8_t)210, (uint8_t)105, (uint8_t)135, (uint8_t)199, (uint8_t)173, (uint8_t)143, (uint8_t)170, (uint8_t)187, (uint8_t)31, (uint8_t)245, (uint8_t)210, (uint8_t)127, (uint8_t)205, (uint8_t)252, (uint8_t)169, (uint8_t)40, (uint8_t)207, (uint8_t)32, (uint8_t)195, (uint8_t)173, (uint8_t)178, (uint8_t)244, (uint8_t)235, (uint8_t)54, (uint8_t)204, (uint8_t)72, (uint8_t)118, (uint8_t)15, (uint8_t)123, (uint8_t)127, (uint8_t)63, (uint8_t)27, (uint8_t)39, (uint8_t)70, (uint8_t)89, (uint8_t)55, (uint8_t)124};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)16972, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)219, (uint8_t)193, (uint8_t)68, (uint8_t)91, (uint8_t)250, (uint8_t)229, (uint8_t)124, (uint8_t)43, (uint8_t)72, (uint8_t)240, (uint8_t)25, (uint8_t)113, (uint8_t)194, (uint8_t)114, (uint8_t)155, (uint8_t)91, (uint8_t)92, (uint8_t)3, (uint8_t)191, (uint8_t)18, (uint8_t)192, (uint8_t)53, (uint8_t)129, (uint8_t)234, (uint8_t)254, (uint8_t)164, (uint8_t)218, (uint8_t)94, (uint8_t)148, (uint8_t)8, (uint8_t)157, (uint8_t)64, (uint8_t)82, (uint8_t)52, (uint8_t)126, (uint8_t)194, (uint8_t)230, (uint8_t)20, (uint8_t)224, (uint8_t)231, (uint8_t)192, (uint8_t)195, (uint8_t)106, (uint8_t)198, (uint8_t)180, (uint8_t)206, (uint8_t)164, (uint8_t)128, (uint8_t)103, (uint8_t)135, (uint8_t)78, (uint8_t)246, (uint8_t)185, (uint8_t)185, (uint8_t)33, (uint8_t)113, (uint8_t)223, (uint8_t)197, (uint8_t)139, (uint8_t)65, (uint8_t)136, (uint8_t)118, (uint8_t)241, (uint8_t)14, (uint8_t)242, (uint8_t)50, (uint8_t)212, (uint8_t)244, (uint8_t)55, (uint8_t)131, (uint8_t)164, (uint8_t)154, (uint8_t)88, (uint8_t)145, (uint8_t)91, (uint8_t)10, (uint8_t)86, (uint8_t)26, (uint8_t)204, (uint8_t)21, (uint8_t)111, (uint8_t)38, (uint8_t)38, (uint8_t)225, (uint8_t)150, (uint8_t)255, (uint8_t)142, (uint8_t)167, (uint8_t)151, (uint8_t)5, (uint8_t)241, (uint8_t)235, (uint8_t)52, (uint8_t)145, (uint8_t)229, (uint8_t)31, (uint8_t)137, (uint8_t)13, (uint8_t)70, (uint8_t)198, (uint8_t)208, (uint8_t)2, (uint8_t)174, (uint8_t)84, (uint8_t)50, (uint8_t)199, (uint8_t)59, (uint8_t)2, (uint8_t)44, (uint8_t)250, (uint8_t)96, (uint8_t)69, (uint8_t)84, (uint8_t)237, (uint8_t)108, (uint8_t)0, (uint8_t)110, (uint8_t)151, (uint8_t)184, (uint8_t)82, (uint8_t)25, (uint8_t)181, (uint8_t)91, (uint8_t)104, (uint8_t)152, (uint8_t)246, (uint8_t)210, (uint8_t)141, (uint8_t)84, (uint8_t)124, (uint8_t)114, (uint8_t)152, (uint8_t)166, (uint8_t)7, (uint8_t)142, (uint8_t)204, (uint8_t)50, (uint8_t)184, (uint8_t)7, (uint8_t)211, (uint8_t)2, (uint8_t)110, (uint8_t)107, (uint8_t)32, (uint8_t)50, (uint8_t)197, (uint8_t)114, (uint8_t)59, (uint8_t)56, (uint8_t)135, (uint8_t)189, (uint8_t)230, (uint8_t)70, (uint8_t)192, (uint8_t)84, (uint8_t)169, (uint8_t)96, (uint8_t)78, (uint8_t)127, (uint8_t)141, (uint8_t)240, (uint8_t)12, (uint8_t)117, (uint8_t)66, (uint8_t)206, (uint8_t)100, (uint8_t)246, (uint8_t)220, (uint8_t)48, (uint8_t)8, (uint8_t)122, (uint8_t)4, (uint8_t)28, (uint8_t)37, (uint8_t)234, (uint8_t)33, (uint8_t)25, (uint8_t)169, (uint8_t)219, (uint8_t)96, (uint8_t)99, (uint8_t)91, (uint8_t)161, (uint8_t)51, (uint8_t)148, (uint8_t)10, (uint8_t)55, (uint8_t)141, (uint8_t)108, (uint8_t)18, (uint8_t)10, (uint8_t)137, (uint8_t)137, (uint8_t)180, (uint8_t)131, (uint8_t)28, (uint8_t)112, (uint8_t)19, (uint8_t)16, (uint8_t)52, (uint8_t)52, (uint8_t)197, (uint8_t)30, (uint8_t)131, (uint8_t)149, (uint8_t)35, (uint8_t)53, (uint8_t)157, (uint8_t)118, (uint8_t)242, (uint8_t)234, (uint8_t)135, (uint8_t)171, (uint8_t)71, (uint8_t)193, (uint8_t)246, (uint8_t)153, (uint8_t)238, (uint8_t)102, (uint8_t)183, (uint8_t)209, (uint8_t)109, (uint8_t)29, (uint8_t)186, (uint8_t)85, (uint8_t)165, (uint8_t)222, (uint8_t)185, (uint8_t)17, (uint8_t)46, (uint8_t)5, (uint8_t)154, (uint8_t)14, (uint8_t)243, (uint8_t)140, (uint8_t)35, (uint8_t)176, (uint8_t)114, (uint8_t)183, (uint8_t)240, (uint8_t)245, (uint8_t)83, (uint8_t)26, (uint8_t)11, (uint8_t)119, (uint8_t)190, (uint8_t)28, (uint8_t)63, (uint8_t)119};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)3609, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p269_framerate_SET((float) -2.9312536E37F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)13041, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)13976, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)3612306118L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)52313, PH.base.pack) ;
    {
        char16_t   uri = "MsjiYswspmkxffswwgbQTbwodvoUmwnWAtcpugvofaFQxfawmtzsxljctjheildnrpznrgUozztewzlnzctixttte";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p270_framerate_SET((float)7.46272E37F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)42543, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)48120, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)492561778L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)43885, PH.base.pack) ;
    {
        char16_t   uri = "rgfmxpqbdeqdadbxrXm";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "gsjpplq";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "fxhzhsgdbLUfyora";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)28461, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)21992, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)61523, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)237, (uint8_t)44, (uint8_t)244, (uint8_t)171, (uint8_t)65, (uint8_t)76, (uint8_t)30, (uint8_t)241};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)18, (uint8_t)193, (uint8_t)238, (uint8_t)102, (uint8_t)193, (uint8_t)230, (uint8_t)153, (uint8_t)37};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)3433626821932601454L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)230297497L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)15455, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)8684206111317576883L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)4279141759L, PH.base.pack) ;
    {
        char16_t   name = "PxkWqqbkxgktrsjtfkPcckebtkkicbecavkfJrn";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)137, (uint8_t)69, (uint8_t)93, (uint8_t)254, (uint8_t)90, (uint8_t)119, (uint8_t)187, (uint8_t)93, (uint8_t)65, (uint8_t)151, (uint8_t)164, (uint8_t)168, (uint8_t)94, (uint8_t)69, (uint8_t)2, (uint8_t)33};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)1820067178L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    {
        char16_t   param_id = "Hevm";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t)23280, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "rancv";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "iufhamslhacdbeepxwdhcuvnvyrulnvixafabwihwbotqsqpbpntfpsmx";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)18392, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)25661, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    {
        char16_t   param_id = "xtuilbby";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "WnanmfnogcuqbhuawNdnjxVqqvtzjfpluqnhmGpiqqtarkzuFcbyovqsgmnig";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "uiyaVryhagcvkgn";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "ngxzyvsjcvnrdoFagwqvafkzwvvfScVZVxLzwiuqodiGsstmoekwkmmhhozypnayVJitfkKtxotLEzgTgVelciljCuWantkdakdamnxyhh";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)8683960768006585251L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)20711, (uint16_t)60061, (uint16_t)26455, (uint16_t)50901, (uint16_t)3593, (uint16_t)52870, (uint16_t)26633, (uint16_t)48687, (uint16_t)5983, (uint16_t)19819, (uint16_t)61273, (uint16_t)20396, (uint16_t)54304, (uint16_t)65117, (uint16_t)17823, (uint16_t)1256, (uint16_t)29969, (uint16_t)24995, (uint16_t)51869, (uint16_t)28714, (uint16_t)12115, (uint16_t)49483, (uint16_t)51624, (uint16_t)1463, (uint16_t)6390, (uint16_t)28921, (uint16_t)57394, (uint16_t)39645, (uint16_t)62281, (uint16_t)14522, (uint16_t)54194, (uint16_t)25417, (uint16_t)40828, (uint16_t)45687, (uint16_t)7646, (uint16_t)58661, (uint16_t)60345, (uint16_t)59649, (uint16_t)27978, (uint16_t)56428, (uint16_t)14331, (uint16_t)33671, (uint16_t)38054, (uint16_t)46310, (uint16_t)1951, (uint16_t)59493, (uint16_t)6833, (uint16_t)42108, (uint16_t)62225, (uint16_t)20745, (uint16_t)16818, (uint16_t)50075, (uint16_t)64809, (uint16_t)42233, (uint16_t)39118, (uint16_t)56983, (uint16_t)42430, (uint16_t)65136, (uint16_t)5295, (uint16_t)16890, (uint16_t)5974, (uint16_t)743, (uint16_t)16106, (uint16_t)63505, (uint16_t)49061, (uint16_t)31715, (uint16_t)27565, (uint16_t)61424, (uint16_t)29890, (uint16_t)59806, (uint16_t)5981, (uint16_t)22419};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)22770, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)49976, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
    p10001_ICAO_SET((uint32_t)3721860699L, PH.base.pack) ;
    {
        char16_t   callsign = "tyGPtj";
        p10001_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, PH.base.pack) ;
    p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M, PH.base.pack) ;
    p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M, PH.base.pack) ;
    p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, PH.base.pack) ;
    p10001_stallSpeed_SET((uint16_t)(uint16_t)19055, PH.base.pack) ;
    p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
    p10002_utcTime_SET((uint32_t)3856178608L, PH.base.pack) ;
    p10002_gpsLat_SET((int32_t) -169164622, PH.base.pack) ;
    p10002_gpsLon_SET((int32_t) -147130386, PH.base.pack) ;
    p10002_gpsAlt_SET((int32_t)1061137712, PH.base.pack) ;
    p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D, PH.base.pack) ;
    p10002_numSats_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p10002_baroAltMSL_SET((int32_t)2084828594, PH.base.pack) ;
    p10002_accuracyHor_SET((uint32_t)322526892L, PH.base.pack) ;
    p10002_accuracyVert_SET((uint16_t)(uint16_t)40974, PH.base.pack) ;
    p10002_accuracyVel_SET((uint16_t)(uint16_t)64089, PH.base.pack) ;
    p10002_velVert_SET((int16_t)(int16_t)4197, PH.base.pack) ;
    p10002_velNS_SET((int16_t)(int16_t)10367, PH.base.pack) ;
    p10002_VelEW_SET((int16_t)(int16_t) -18891, PH.base.pack) ;
    p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY, PH.base.pack) ;
    p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED, PH.base.pack) ;
    p10002_squawk_SET((uint16_t)(uint16_t)24698, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(), &PH);
    p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEVICE_OP_READ_11000(), &PH);
    p11000_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p11000_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p11000_request_id_SET((uint32_t)3826072529L, PH.base.pack) ;
    p11000_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
    p11000_bus_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p11000_address_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    {
        char16_t   busname = "bnUxwspcbrjazQhoodliurfxkj";
        p11000_busname_SET(&busname, 0,  sizeof(busname), &PH) ;
    }
    p11000_regstart_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p11000_count_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEVICE_OP_READ_REPLY_11001(), &PH);
    p11001_request_id_SET((uint32_t)3271757999L, PH.base.pack) ;
    p11001_result_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p11001_regstart_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p11001_count_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)242, (uint8_t)227, (uint8_t)172, (uint8_t)188, (uint8_t)206, (uint8_t)233, (uint8_t)80, (uint8_t)229, (uint8_t)110, (uint8_t)199, (uint8_t)241, (uint8_t)112, (uint8_t)176, (uint8_t)44, (uint8_t)149, (uint8_t)143, (uint8_t)86, (uint8_t)12, (uint8_t)163, (uint8_t)30, (uint8_t)22, (uint8_t)131, (uint8_t)101, (uint8_t)217, (uint8_t)132, (uint8_t)62, (uint8_t)142, (uint8_t)95, (uint8_t)163, (uint8_t)181, (uint8_t)9, (uint8_t)174, (uint8_t)146, (uint8_t)228, (uint8_t)194, (uint8_t)123, (uint8_t)165, (uint8_t)225, (uint8_t)44, (uint8_t)223, (uint8_t)119, (uint8_t)42, (uint8_t)132, (uint8_t)54, (uint8_t)45, (uint8_t)95, (uint8_t)90, (uint8_t)36, (uint8_t)76, (uint8_t)22, (uint8_t)197, (uint8_t)11, (uint8_t)247, (uint8_t)142, (uint8_t)242, (uint8_t)203, (uint8_t)147, (uint8_t)33, (uint8_t)221, (uint8_t)25, (uint8_t)121, (uint8_t)213, (uint8_t)51, (uint8_t)10, (uint8_t)210, (uint8_t)76, (uint8_t)169, (uint8_t)127, (uint8_t)74, (uint8_t)111, (uint8_t)80, (uint8_t)45, (uint8_t)248, (uint8_t)143, (uint8_t)34, (uint8_t)191, (uint8_t)62, (uint8_t)185, (uint8_t)105, (uint8_t)90, (uint8_t)59, (uint8_t)48, (uint8_t)80, (uint8_t)16, (uint8_t)246, (uint8_t)238, (uint8_t)19, (uint8_t)214, (uint8_t)116, (uint8_t)187, (uint8_t)16, (uint8_t)88, (uint8_t)139, (uint8_t)199, (uint8_t)163, (uint8_t)3, (uint8_t)128, (uint8_t)65, (uint8_t)20, (uint8_t)170, (uint8_t)56, (uint8_t)177, (uint8_t)194, (uint8_t)14, (uint8_t)23, (uint8_t)173, (uint8_t)114, (uint8_t)22, (uint8_t)231, (uint8_t)191, (uint8_t)4, (uint8_t)244, (uint8_t)71, (uint8_t)133, (uint8_t)242, (uint8_t)250, (uint8_t)26, (uint8_t)251, (uint8_t)62, (uint8_t)155, (uint8_t)130, (uint8_t)97, (uint8_t)14, (uint8_t)181, (uint8_t)71, (uint8_t)30, (uint8_t)19, (uint8_t)219};
        p11001_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEVICE_OP_WRITE_11002(), &PH);
    p11002_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p11002_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    p11002_request_id_SET((uint32_t)410863372L, PH.base.pack) ;
    p11002_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, PH.base.pack) ;
    p11002_bus_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p11002_address_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    {
        char16_t   busname = "vyGUOqmjqjjIsfdiymjmJshghrdbdMbun";
        p11002_busname_SET(&busname, 0,  sizeof(busname), &PH) ;
    }
    p11002_regstart_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p11002_count_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)145, (uint8_t)67, (uint8_t)216, (uint8_t)160, (uint8_t)9, (uint8_t)11, (uint8_t)157, (uint8_t)176, (uint8_t)21, (uint8_t)79, (uint8_t)205, (uint8_t)228, (uint8_t)133, (uint8_t)141, (uint8_t)150, (uint8_t)190, (uint8_t)237, (uint8_t)200, (uint8_t)215, (uint8_t)227, (uint8_t)119, (uint8_t)129, (uint8_t)162, (uint8_t)119, (uint8_t)239, (uint8_t)0, (uint8_t)113, (uint8_t)46, (uint8_t)217, (uint8_t)179, (uint8_t)123, (uint8_t)167, (uint8_t)34, (uint8_t)191, (uint8_t)174, (uint8_t)213, (uint8_t)90, (uint8_t)207, (uint8_t)253, (uint8_t)175, (uint8_t)36, (uint8_t)142, (uint8_t)251, (uint8_t)71, (uint8_t)10, (uint8_t)104, (uint8_t)95, (uint8_t)101, (uint8_t)201, (uint8_t)39, (uint8_t)34, (uint8_t)29, (uint8_t)19, (uint8_t)140, (uint8_t)193, (uint8_t)229, (uint8_t)51, (uint8_t)194, (uint8_t)29, (uint8_t)21, (uint8_t)194, (uint8_t)64, (uint8_t)205, (uint8_t)191, (uint8_t)71, (uint8_t)94, (uint8_t)23, (uint8_t)60, (uint8_t)116, (uint8_t)231, (uint8_t)116, (uint8_t)230, (uint8_t)225, (uint8_t)61, (uint8_t)36, (uint8_t)133, (uint8_t)185, (uint8_t)54, (uint8_t)138, (uint8_t)127, (uint8_t)169, (uint8_t)211, (uint8_t)164, (uint8_t)168, (uint8_t)60, (uint8_t)212, (uint8_t)148, (uint8_t)25, (uint8_t)186, (uint8_t)53, (uint8_t)148, (uint8_t)144, (uint8_t)81, (uint8_t)83, (uint8_t)87, (uint8_t)187, (uint8_t)98, (uint8_t)67, (uint8_t)160, (uint8_t)246, (uint8_t)132, (uint8_t)19, (uint8_t)25, (uint8_t)221, (uint8_t)89, (uint8_t)58, (uint8_t)147, (uint8_t)159, (uint8_t)77, (uint8_t)136, (uint8_t)158, (uint8_t)86, (uint8_t)57, (uint8_t)21, (uint8_t)105, (uint8_t)0, (uint8_t)171, (uint8_t)110, (uint8_t)145, (uint8_t)52, (uint8_t)49, (uint8_t)70, (uint8_t)201, (uint8_t)179, (uint8_t)67, (uint8_t)22, (uint8_t)62, (uint8_t)64};
        p11002_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEVICE_OP_WRITE_REPLY_11003(), &PH);
    p11003_request_id_SET((uint32_t)4282532064L, PH.base.pack) ;
    p11003_result_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADAP_TUNING_11010(), &PH);
    p11010_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_ACCZ, PH.base.pack) ;
    p11010_desired_SET((float)1.3900287E38F, PH.base.pack) ;
    p11010_achieved_SET((float) -2.6387614E38F, PH.base.pack) ;
    p11010_error_SET((float) -3.3913726E38F, PH.base.pack) ;
    p11010_theta_SET((float) -3.3753996E38F, PH.base.pack) ;
    p11010_omega_SET((float) -1.8975257E38F, PH.base.pack) ;
    p11010_sigma_SET((float)1.8067522E38F, PH.base.pack) ;
    p11010_theta_dot_SET((float) -1.3368894E37F, PH.base.pack) ;
    p11010_omega_dot_SET((float)2.7186593E38F, PH.base.pack) ;
    p11010_sigma_dot_SET((float)2.7709877E38F, PH.base.pack) ;
    p11010_f_SET((float)1.6968602E38F, PH.base.pack) ;
    p11010_f_dot_SET((float) -2.6535075E38F, PH.base.pack) ;
    p11010_u_SET((float) -2.6301401E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_DELTA_11011(), &PH);
    p11011_time_usec_SET((uint64_t)2886580152713752027L, PH.base.pack) ;
    p11011_time_delta_usec_SET((uint64_t)3551465080914680800L, PH.base.pack) ;
    {
        float  angle_delta [] =  {-1.9214466E38F, 1.9031478E38F, -1.5785736E38F};
        p11011_angle_delta_SET(&angle_delta, 0, &PH.base.pack) ;
    }
    {
        float  position_delta [] =  {-4.442883E37F, -1.861992E37F, 1.3372198E38F};
        p11011_position_delta_SET(&position_delta, 0, &PH.base.pack) ;
    }
    p11011_confidence_SET((float) -2.0822822E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

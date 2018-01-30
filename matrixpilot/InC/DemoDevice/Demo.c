
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
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p150_target_system_GET(pack);
    uint8_t  target_component = p150_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  read_req_type = p151_read_req_type_GET(pack);
    int16_t  data_index = p151_data_index_GET(pack);
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p152_target_system_GET(pack);
    uint8_t  target_component = p152_target_component_GET(pack);
    uint16_t  func_index = p152_func_index_GET(pack);
    uint16_t  func_count = p152_func_count_GET(pack);
    uint16_t  data_address = p152_data_address_GET(pack);
    uint16_t  data_size = p152_data_size_GET(pack);
    int8_t*  data_ = p152_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p153_target_system_GET(pack);
    uint8_t  target_component = p153_target_component_GET(pack);
    uint16_t  func_index = p153_func_index_GET(pack);
    uint16_t  result = p153_result_GET(pack);
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p155_target_system_GET(pack);
    uint8_t  target_component = p155_target_component_GET(pack);
    uint8_t  directory_type = p155_directory_type_GET(pack);
    uint8_t  start_index = p155_start_index_GET(pack);
    uint8_t  count = p155_count_GET(pack);
    int8_t*  directory_data = p155_directory_data_GET_(pack);
//process data in directory_data
    free(directory_data);//never forget to dispose
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    uint8_t  directory_type = p156_directory_type_GET(pack);
    uint8_t  start_index = p156_start_index_GET(pack);
    uint8_t  count = p156_count_GET(pack);
    uint16_t  result = p156_result_GET(pack);
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    uint8_t  command_type = p157_command_type_GET(pack);
}
void c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  command_type = p158_command_type_GET(pack);
    uint16_t  result = p158_result_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  sue_time = p170_sue_time_GET(pack);
    uint8_t  sue_status = p170_sue_status_GET(pack);
    int32_t  sue_latitude = p170_sue_latitude_GET(pack);
    int32_t  sue_longitude = p170_sue_longitude_GET(pack);
    int32_t  sue_altitude = p170_sue_altitude_GET(pack);
    uint16_t  sue_waypoint_index = p170_sue_waypoint_index_GET(pack);
    int16_t  sue_rmat0 = p170_sue_rmat0_GET(pack);
    int16_t  sue_rmat1 = p170_sue_rmat1_GET(pack);
    int16_t  sue_rmat2 = p170_sue_rmat2_GET(pack);
    int16_t  sue_rmat3 = p170_sue_rmat3_GET(pack);
    int16_t  sue_rmat4 = p170_sue_rmat4_GET(pack);
    int16_t  sue_rmat5 = p170_sue_rmat5_GET(pack);
    int16_t  sue_rmat6 = p170_sue_rmat6_GET(pack);
    int16_t  sue_rmat7 = p170_sue_rmat7_GET(pack);
    int16_t  sue_rmat8 = p170_sue_rmat8_GET(pack);
    uint16_t  sue_cog = p170_sue_cog_GET(pack);
    int16_t  sue_sog = p170_sue_sog_GET(pack);
    uint16_t  sue_cpu_load = p170_sue_cpu_load_GET(pack);
    uint16_t  sue_air_speed_3DIMU = p170_sue_air_speed_3DIMU_GET(pack);
    int16_t  sue_estimated_wind_0 = p170_sue_estimated_wind_0_GET(pack);
    int16_t  sue_estimated_wind_1 = p170_sue_estimated_wind_1_GET(pack);
    int16_t  sue_estimated_wind_2 = p170_sue_estimated_wind_2_GET(pack);
    int16_t  sue_magFieldEarth0 = p170_sue_magFieldEarth0_GET(pack);
    int16_t  sue_magFieldEarth1 = p170_sue_magFieldEarth1_GET(pack);
    int16_t  sue_magFieldEarth2 = p170_sue_magFieldEarth2_GET(pack);
    int16_t  sue_svs = p170_sue_svs_GET(pack);
    int16_t  sue_hdop = p170_sue_hdop_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  sue_time = p171_sue_time_GET(pack);
    int16_t  sue_pwm_input_1 = p171_sue_pwm_input_1_GET(pack);
    int16_t  sue_pwm_input_2 = p171_sue_pwm_input_2_GET(pack);
    int16_t  sue_pwm_input_3 = p171_sue_pwm_input_3_GET(pack);
    int16_t  sue_pwm_input_4 = p171_sue_pwm_input_4_GET(pack);
    int16_t  sue_pwm_input_5 = p171_sue_pwm_input_5_GET(pack);
    int16_t  sue_pwm_input_6 = p171_sue_pwm_input_6_GET(pack);
    int16_t  sue_pwm_input_7 = p171_sue_pwm_input_7_GET(pack);
    int16_t  sue_pwm_input_8 = p171_sue_pwm_input_8_GET(pack);
    int16_t  sue_pwm_input_9 = p171_sue_pwm_input_9_GET(pack);
    int16_t  sue_pwm_input_10 = p171_sue_pwm_input_10_GET(pack);
    int16_t  sue_pwm_input_11 = p171_sue_pwm_input_11_GET(pack);
    int16_t  sue_pwm_input_12 = p171_sue_pwm_input_12_GET(pack);
    int16_t  sue_pwm_output_1 = p171_sue_pwm_output_1_GET(pack);
    int16_t  sue_pwm_output_2 = p171_sue_pwm_output_2_GET(pack);
    int16_t  sue_pwm_output_3 = p171_sue_pwm_output_3_GET(pack);
    int16_t  sue_pwm_output_4 = p171_sue_pwm_output_4_GET(pack);
    int16_t  sue_pwm_output_5 = p171_sue_pwm_output_5_GET(pack);
    int16_t  sue_pwm_output_6 = p171_sue_pwm_output_6_GET(pack);
    int16_t  sue_pwm_output_7 = p171_sue_pwm_output_7_GET(pack);
    int16_t  sue_pwm_output_8 = p171_sue_pwm_output_8_GET(pack);
    int16_t  sue_pwm_output_9 = p171_sue_pwm_output_9_GET(pack);
    int16_t  sue_pwm_output_10 = p171_sue_pwm_output_10_GET(pack);
    int16_t  sue_pwm_output_11 = p171_sue_pwm_output_11_GET(pack);
    int16_t  sue_pwm_output_12 = p171_sue_pwm_output_12_GET(pack);
    int16_t  sue_imu_location_x = p171_sue_imu_location_x_GET(pack);
    int16_t  sue_imu_location_y = p171_sue_imu_location_y_GET(pack);
    int16_t  sue_imu_location_z = p171_sue_imu_location_z_GET(pack);
    int16_t  sue_location_error_earth_x = p171_sue_location_error_earth_x_GET(pack);
    int16_t  sue_location_error_earth_y = p171_sue_location_error_earth_y_GET(pack);
    int16_t  sue_location_error_earth_z = p171_sue_location_error_earth_z_GET(pack);
    uint32_t  sue_flags = p171_sue_flags_GET(pack);
    int16_t  sue_osc_fails = p171_sue_osc_fails_GET(pack);
    int16_t  sue_imu_velocity_x = p171_sue_imu_velocity_x_GET(pack);
    int16_t  sue_imu_velocity_y = p171_sue_imu_velocity_y_GET(pack);
    int16_t  sue_imu_velocity_z = p171_sue_imu_velocity_z_GET(pack);
    int16_t  sue_waypoint_goal_x = p171_sue_waypoint_goal_x_GET(pack);
    int16_t  sue_waypoint_goal_y = p171_sue_waypoint_goal_y_GET(pack);
    int16_t  sue_waypoint_goal_z = p171_sue_waypoint_goal_z_GET(pack);
    int16_t  sue_aero_x = p171_sue_aero_x_GET(pack);
    int16_t  sue_aero_y = p171_sue_aero_y_GET(pack);
    int16_t  sue_aero_z = p171_sue_aero_z_GET(pack);
    int16_t  sue_barom_temp = p171_sue_barom_temp_GET(pack);
    int32_t  sue_barom_press = p171_sue_barom_press_GET(pack);
    int32_t  sue_barom_alt = p171_sue_barom_alt_GET(pack);
    int16_t  sue_bat_volt = p171_sue_bat_volt_GET(pack);
    int16_t  sue_bat_amp = p171_sue_bat_amp_GET(pack);
    int16_t  sue_bat_amp_hours = p171_sue_bat_amp_hours_GET(pack);
    int16_t  sue_desired_height = p171_sue_desired_height_GET(pack);
    int16_t  sue_memory_stack_free = p171_sue_memory_stack_free_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_ROLL_STABILIZATION_AILERONS = p172_sue_ROLL_STABILIZATION_AILERONS_GET(pack);
    uint8_t  sue_ROLL_STABILIZATION_RUDDER = p172_sue_ROLL_STABILIZATION_RUDDER_GET(pack);
    uint8_t  sue_PITCH_STABILIZATION = p172_sue_PITCH_STABILIZATION_GET(pack);
    uint8_t  sue_YAW_STABILIZATION_RUDDER = p172_sue_YAW_STABILIZATION_RUDDER_GET(pack);
    uint8_t  sue_YAW_STABILIZATION_AILERON = p172_sue_YAW_STABILIZATION_AILERON_GET(pack);
    uint8_t  sue_AILERON_NAVIGATION = p172_sue_AILERON_NAVIGATION_GET(pack);
    uint8_t  sue_RUDDER_NAVIGATION = p172_sue_RUDDER_NAVIGATION_GET(pack);
    uint8_t  sue_ALTITUDEHOLD_STABILIZED = p172_sue_ALTITUDEHOLD_STABILIZED_GET(pack);
    uint8_t  sue_ALTITUDEHOLD_WAYPOINT = p172_sue_ALTITUDEHOLD_WAYPOINT_GET(pack);
    uint8_t  sue_RACING_MODE = p172_sue_RACING_MODE_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_AILERON = p173_sue_YAWKP_AILERON_GET(pack);
    float  sue_YAWKD_AILERON = p173_sue_YAWKD_AILERON_GET(pack);
    float  sue_ROLLKP = p173_sue_ROLLKP_GET(pack);
    float  sue_ROLLKD = p173_sue_ROLLKD_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    float  sue_PITCHGAIN = p174_sue_PITCHGAIN_GET(pack);
    float  sue_PITCHKD = p174_sue_PITCHKD_GET(pack);
    float  sue_RUDDER_ELEV_MIX = p174_sue_RUDDER_ELEV_MIX_GET(pack);
    float  sue_ROLL_ELEV_MIX = p174_sue_ROLL_ELEV_MIX_GET(pack);
    float  sue_ELEVATOR_BOOST = p174_sue_ELEVATOR_BOOST_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_RUDDER = p175_sue_YAWKP_RUDDER_GET(pack);
    float  sue_YAWKD_RUDDER = p175_sue_YAWKD_RUDDER_GET(pack);
    float  sue_ROLLKP_RUDDER = p175_sue_ROLLKP_RUDDER_GET(pack);
    float  sue_ROLLKD_RUDDER = p175_sue_ROLLKD_RUDDER_GET(pack);
    float  sue_RUDDER_BOOST = p175_sue_RUDDER_BOOST_GET(pack);
    float  sue_RTL_PITCH_DOWN = p175_sue_RTL_PITCH_DOWN_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    float  sue_HEIGHT_TARGET_MAX = p176_sue_HEIGHT_TARGET_MAX_GET(pack);
    float  sue_HEIGHT_TARGET_MIN = p176_sue_HEIGHT_TARGET_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MIN = p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MAX = p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_MIN = p176_sue_ALT_HOLD_PITCH_MIN_GET(pack);
    float  sue_ALT_HOLD_PITCH_MAX = p176_sue_ALT_HOLD_PITCH_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_HIGH = p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_week_no = p177_sue_week_no_GET(pack);
    int32_t  sue_lat_origin = p177_sue_lat_origin_GET(pack);
    int32_t  sue_lon_origin = p177_sue_lon_origin_GET(pack);
    int32_t  sue_alt_origin = p177_sue_alt_origin_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_WIND_ESTIMATION = p178_sue_WIND_ESTIMATION_GET(pack);
    uint8_t  sue_GPS_TYPE = p178_sue_GPS_TYPE_GET(pack);
    uint8_t  sue_DR = p178_sue_DR_GET(pack);
    uint8_t  sue_BOARD_TYPE = p178_sue_BOARD_TYPE_GET(pack);
    uint8_t  sue_AIRFRAME = p178_sue_AIRFRAME_GET(pack);
    int16_t  sue_RCON = p178_sue_RCON_GET(pack);
    int16_t  sue_TRAP_FLAGS = p178_sue_TRAP_FLAGS_GET(pack);
    uint32_t  sue_TRAP_SOURCE = p178_sue_TRAP_SOURCE_GET(pack);
    int16_t  sue_osc_fail_count = p178_sue_osc_fail_count_GET(pack);
    uint8_t  sue_CLOCK_CONFIG = p178_sue_CLOCK_CONFIG_GET(pack);
    uint8_t  sue_FLIGHT_PLAN_TYPE = p178_sue_FLIGHT_PLAN_TYPE_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_VEHICLE_MODEL_NAME = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
//process data in sue_ID_VEHICLE_MODEL_NAME
    free(sue_ID_VEHICLE_MODEL_NAME);//never forget to dispose
    uint8_t*  sue_ID_VEHICLE_REGISTRATION = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
//process data in sue_ID_VEHICLE_REGISTRATION
    free(sue_ID_VEHICLE_REGISTRATION);//never forget to dispose
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_LEAD_PILOT = p180_sue_ID_LEAD_PILOT_GET_(pack);
//process data in sue_ID_LEAD_PILOT
    free(sue_ID_LEAD_PILOT);//never forget to dispose
    uint8_t*  sue_ID_DIY_DRONES_URL = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
//process data in sue_ID_DIY_DRONES_URL
    free(sue_ID_DIY_DRONES_URL);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p181_time_boot_ms_GET(pack);
    int32_t  alt_gps = p181_alt_gps_GET(pack);
    int32_t  alt_imu = p181_alt_imu_GET(pack);
    int32_t  alt_barometric = p181_alt_barometric_GET(pack);
    int32_t  alt_optical_flow = p181_alt_optical_flow_GET(pack);
    int32_t  alt_range_finder = p181_alt_range_finder_GET(pack);
    int32_t  alt_extra = p181_alt_extra_GET(pack);
}
void c_LoopBackDemoChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p182_time_boot_ms_GET(pack);
    int16_t  airspeed_imu = p182_airspeed_imu_GET(pack);
    int16_t  airspeed_pitot = p182_airspeed_pitot_GET(pack);
    int16_t  airspeed_hot_wire = p182_airspeed_hot_wire_GET(pack);
    int16_t  airspeed_ultrasonic = p182_airspeed_ultrasonic_GET(pack);
    int16_t  aoa = p182_aoa_GET(pack);
    int16_t  aoy = p182_aoy_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    float  sue_feed_forward = p183_sue_feed_forward_GET(pack);
    float  sue_turn_rate_nav = p183_sue_turn_rate_nav_GET(pack);
    float  sue_turn_rate_fbw = p183_sue_turn_rate_fbw_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    float  angle_of_attack_normal = p184_angle_of_attack_normal_GET(pack);
    float  angle_of_attack_inverted = p184_angle_of_attack_inverted_GET(pack);
    float  elevator_trim_normal = p184_elevator_trim_normal_GET(pack);
    float  elevator_trim_inverted = p184_elevator_trim_inverted_GET(pack);
    float  reference_speed = p184_reference_speed_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_aileron_output_channel = p185_sue_aileron_output_channel_GET(pack);
    uint8_t  sue_aileron_reversed = p185_sue_aileron_reversed_GET(pack);
    uint8_t  sue_elevator_output_channel = p185_sue_elevator_output_channel_GET(pack);
    uint8_t  sue_elevator_reversed = p185_sue_elevator_reversed_GET(pack);
    uint8_t  sue_throttle_output_channel = p185_sue_throttle_output_channel_GET(pack);
    uint8_t  sue_throttle_reversed = p185_sue_throttle_reversed_GET(pack);
    uint8_t  sue_rudder_output_channel = p185_sue_rudder_output_channel_GET(pack);
    uint8_t  sue_rudder_reversed = p185_sue_rudder_reversed_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_number_of_inputs = p186_sue_number_of_inputs_GET(pack);
    int16_t  sue_trim_value_input_1 = p186_sue_trim_value_input_1_GET(pack);
    int16_t  sue_trim_value_input_2 = p186_sue_trim_value_input_2_GET(pack);
    int16_t  sue_trim_value_input_3 = p186_sue_trim_value_input_3_GET(pack);
    int16_t  sue_trim_value_input_4 = p186_sue_trim_value_input_4_GET(pack);
    int16_t  sue_trim_value_input_5 = p186_sue_trim_value_input_5_GET(pack);
    int16_t  sue_trim_value_input_6 = p186_sue_trim_value_input_6_GET(pack);
    int16_t  sue_trim_value_input_7 = p186_sue_trim_value_input_7_GET(pack);
    int16_t  sue_trim_value_input_8 = p186_sue_trim_value_input_8_GET(pack);
    int16_t  sue_trim_value_input_9 = p186_sue_trim_value_input_9_GET(pack);
    int16_t  sue_trim_value_input_10 = p186_sue_trim_value_input_10_GET(pack);
    int16_t  sue_trim_value_input_11 = p186_sue_trim_value_input_11_GET(pack);
    int16_t  sue_trim_value_input_12 = p186_sue_trim_value_input_12_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_offset = p187_sue_accel_x_offset_GET(pack);
    int16_t  sue_accel_y_offset = p187_sue_accel_y_offset_GET(pack);
    int16_t  sue_accel_z_offset = p187_sue_accel_z_offset_GET(pack);
    int16_t  sue_gyro_x_offset = p187_sue_gyro_x_offset_GET(pack);
    int16_t  sue_gyro_y_offset = p187_sue_gyro_y_offset_GET(pack);
    int16_t  sue_gyro_z_offset = p187_sue_gyro_z_offset_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_at_calibration = p188_sue_accel_x_at_calibration_GET(pack);
    int16_t  sue_accel_y_at_calibration = p188_sue_accel_y_at_calibration_GET(pack);
    int16_t  sue_accel_z_at_calibration = p188_sue_accel_z_at_calibration_GET(pack);
    int16_t  sue_gyro_x_at_calibration = p188_sue_gyro_x_at_calibration_GET(pack);
    int16_t  sue_gyro_y_at_calibration = p188_sue_gyro_y_at_calibration_GET(pack);
    int16_t  sue_gyro_z_at_calibration = p188_sue_gyro_z_at_calibration_GET(pack);
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1594999338L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)43278, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)46821, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -3905, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -60, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)18102, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)11333, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)50914, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)45666, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)61230, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)33947, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)3310540065465535120L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)374907540L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)3233985122L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)2299, PH.base.pack) ;
    p3_x_SET((float)8.29368E37F, PH.base.pack) ;
    p3_y_SET((float) -2.237619E38F, PH.base.pack) ;
    p3_z_SET((float)3.3790541E38F, PH.base.pack) ;
    p3_vx_SET((float)1.9470553E38F, PH.base.pack) ;
    p3_vy_SET((float) -3.2834437E37F, PH.base.pack) ;
    p3_vz_SET((float)2.6994246E38F, PH.base.pack) ;
    p3_afx_SET((float)2.4523926E38F, PH.base.pack) ;
    p3_afy_SET((float) -7.690169E37F, PH.base.pack) ;
    p3_afz_SET((float)2.6151115E37F, PH.base.pack) ;
    p3_yaw_SET((float) -8.0881296E37F, PH.base.pack) ;
    p3_yaw_rate_SET((float)2.8352788E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)1911427304911793251L, PH.base.pack) ;
    p4_seq_SET((uint32_t)429829101L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    {
        char16_t   passkey = "bpMzaTrhzezaluyvewomjn";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "uadDeylwz";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)2058038304L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    {
        char16_t   param_id = "qprHmKveqoDfgJr";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)31801, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "rfjvOpgN";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)1.996266E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)9653, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)52294, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    {
        char16_t   param_id = "oEPuiqr";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -6.6026296E37F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)8040520970953166920L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -430125537, PH.base.pack) ;
    p24_lon_SET((int32_t)1625696585, PH.base.pack) ;
    p24_alt_SET((int32_t)254464400, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)30421, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)5076, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)4429, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)50459, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -552256136, &PH) ;
    p24_h_acc_SET((uint32_t)4096695584L, &PH) ;
    p24_v_acc_SET((uint32_t)2220117944L, &PH) ;
    p24_vel_acc_SET((uint32_t)4199041692L, &PH) ;
    p24_hdg_acc_SET((uint32_t)2599097421L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)118, (uint8_t)82, (uint8_t)91, (uint8_t)225, (uint8_t)34, (uint8_t)185, (uint8_t)157, (uint8_t)116, (uint8_t)145, (uint8_t)222, (uint8_t)24, (uint8_t)109, (uint8_t)231, (uint8_t)57, (uint8_t)99, (uint8_t)215, (uint8_t)181, (uint8_t)88, (uint8_t)33, (uint8_t)108};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)241, (uint8_t)201, (uint8_t)219, (uint8_t)238, (uint8_t)30, (uint8_t)134, (uint8_t)184, (uint8_t)62, (uint8_t)194, (uint8_t)106, (uint8_t)121, (uint8_t)116, (uint8_t)40, (uint8_t)153, (uint8_t)252, (uint8_t)197, (uint8_t)140, (uint8_t)53, (uint8_t)65, (uint8_t)83};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)208, (uint8_t)154, (uint8_t)94, (uint8_t)39, (uint8_t)65, (uint8_t)141, (uint8_t)144, (uint8_t)144, (uint8_t)201, (uint8_t)32, (uint8_t)172, (uint8_t)133, (uint8_t)69, (uint8_t)86, (uint8_t)26, (uint8_t)131, (uint8_t)236, (uint8_t)135, (uint8_t)238, (uint8_t)0};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)88, (uint8_t)97, (uint8_t)223, (uint8_t)53, (uint8_t)24, (uint8_t)40, (uint8_t)138, (uint8_t)246, (uint8_t)116, (uint8_t)36, (uint8_t)7, (uint8_t)187, (uint8_t)79, (uint8_t)21, (uint8_t)105, (uint8_t)5, (uint8_t)190, (uint8_t)68, (uint8_t)92, (uint8_t)22};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)83, (uint8_t)15, (uint8_t)3, (uint8_t)0, (uint8_t)167, (uint8_t)33, (uint8_t)171, (uint8_t)115, (uint8_t)26, (uint8_t)61, (uint8_t)23, (uint8_t)232, (uint8_t)79, (uint8_t)157, (uint8_t)55, (uint8_t)208, (uint8_t)143, (uint8_t)175, (uint8_t)196, (uint8_t)251};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3413782868L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -21246, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)11907, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -19021, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t)27083, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -516, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)13415, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)19355, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)17525, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)10423, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)7516851052335150842L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -12638, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -27743, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)20364, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -13542, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -12813, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)25490, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)27247, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)6758, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)9706, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)7284581787627184382L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -16169, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)5167, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -6434, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -3521, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)790871278L, PH.base.pack) ;
    p29_press_abs_SET((float) -2.8372888E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -2.555503E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)15378, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)221434907L, PH.base.pack) ;
    p30_roll_SET((float)2.1322864E37F, PH.base.pack) ;
    p30_pitch_SET((float)1.4506111E38F, PH.base.pack) ;
    p30_yaw_SET((float)3.3937172E37F, PH.base.pack) ;
    p30_rollspeed_SET((float)3.3904743E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -2.481619E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -3.1000558E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1942765740L, PH.base.pack) ;
    p31_q1_SET((float)1.9673477E38F, PH.base.pack) ;
    p31_q2_SET((float) -1.6881667E38F, PH.base.pack) ;
    p31_q3_SET((float) -2.7788449E38F, PH.base.pack) ;
    p31_q4_SET((float)1.2118278E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)2.002353E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -2.2305314E38F, PH.base.pack) ;
    p31_yawspeed_SET((float) -2.5128457E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)797940571L, PH.base.pack) ;
    p32_x_SET((float) -6.46591E37F, PH.base.pack) ;
    p32_y_SET((float)3.8963662E37F, PH.base.pack) ;
    p32_z_SET((float) -3.0170155E38F, PH.base.pack) ;
    p32_vx_SET((float) -3.3979409E38F, PH.base.pack) ;
    p32_vy_SET((float)1.9258122E38F, PH.base.pack) ;
    p32_vz_SET((float)2.3328531E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1768461608L, PH.base.pack) ;
    p33_lat_SET((int32_t)1343890115, PH.base.pack) ;
    p33_lon_SET((int32_t)680389949, PH.base.pack) ;
    p33_alt_SET((int32_t) -242423933, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -397622701, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -21139, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -15516, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)14476, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)55984, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3114906146L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -31998, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -10655, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)8300, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)23357, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)30820, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)32068, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -12773, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -13361, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)750322545L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)48053, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)63995, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)45697, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)9680, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)12794, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)24769, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)52543, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)33468, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)3405600353L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)1199, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)12901, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)35090, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)46326, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)63914, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)2859, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)1222, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)41978, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)16455, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)56722, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)39857, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)21698, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)14430, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)111, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)56688, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)42024, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -25415, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)25759, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -29321, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)16566, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)17585, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p39_param1_SET((float) -1.6372959E38F, PH.base.pack) ;
    p39_param2_SET((float) -2.3712274E37F, PH.base.pack) ;
    p39_param3_SET((float) -1.562434E38F, PH.base.pack) ;
    p39_param4_SET((float)8.0762917E37F, PH.base.pack) ;
    p39_x_SET((float) -8.896299E37F, PH.base.pack) ;
    p39_y_SET((float) -6.396371E37F, PH.base.pack) ;
    p39_z_SET((float)2.7313276E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)32154, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)50154, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)3381, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)18384, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)55488, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p48_latitude_SET((int32_t)49108954, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1175906272, PH.base.pack) ;
    p48_altitude_SET((int32_t) -1766080823, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)5215076746568578506L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -870446315, PH.base.pack) ;
    p49_longitude_SET((int32_t)1074974003, PH.base.pack) ;
    p49_altitude_SET((int32_t) -438370046, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)3229928482369080369L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    {
        char16_t   param_id = "dqMZGpJr";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -15502, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p50_param_value0_SET((float)1.4139723E38F, PH.base.pack) ;
    p50_scale_SET((float) -2.6540492E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -1.984798E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -2.102852E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)19718, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p54_p1x_SET((float)2.4280774E38F, PH.base.pack) ;
    p54_p1y_SET((float) -2.3204632E38F, PH.base.pack) ;
    p54_p1z_SET((float) -1.7303913E38F, PH.base.pack) ;
    p54_p2x_SET((float) -3.2554478E38F, PH.base.pack) ;
    p54_p2y_SET((float)1.5498607E38F, PH.base.pack) ;
    p54_p2z_SET((float) -4.474383E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float) -2.484681E37F, PH.base.pack) ;
    p55_p1y_SET((float)3.3735025E37F, PH.base.pack) ;
    p55_p1z_SET((float)1.3300493E38F, PH.base.pack) ;
    p55_p2x_SET((float)7.657402E37F, PH.base.pack) ;
    p55_p2y_SET((float) -2.2014979E36F, PH.base.pack) ;
    p55_p2z_SET((float) -3.0348894E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)6010906317665686872L, PH.base.pack) ;
    {
        float  q [] =  {8.819425E37F, 2.5899075E38F, 2.4269248E38F, 1.5218195E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)2.2984769E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -7.6882735E37F, PH.base.pack) ;
    p61_yawspeed_SET((float) -1.558844E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-5.480781E37F, 2.2260166E38F, -2.3619462E38F, 1.3531223E38F, 8.775059E37F, -3.108815E38F, -2.4250992E38F, 1.0997405E38F, -1.7511997E37F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.0789809E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -2.4548236E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -32668, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -16309, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)30674, PH.base.pack) ;
    p62_alt_error_SET((float)1.1744116E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -2.986041E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -3.3017554E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)2015283066787412608L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t)1298291586, PH.base.pack) ;
    p63_lon_SET((int32_t) -818812999, PH.base.pack) ;
    p63_alt_SET((int32_t) -1326989031, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1199659012, PH.base.pack) ;
    p63_vx_SET((float)1.6604485E37F, PH.base.pack) ;
    p63_vy_SET((float)7.280984E37F, PH.base.pack) ;
    p63_vz_SET((float)9.723237E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-7.217401E37F, 5.091763E37F, -1.2213006E37F, 1.4076216E38F, -3.168616E38F, 1.8561023E38F, 2.9501635E38F, -2.8378663E38F, 7.680475E37F, -1.7738346E38F, 1.6713522E38F, 1.49147E36F, 2.671719E38F, -1.8554764E38F, -2.939022E38F, 3.0808648E38F, -1.1734716E38F, 2.6963565E37F, 3.2418785E38F, 2.0601529E38F, -3.1902447E38F, 1.8475795E38F, 1.8178655E37F, -1.8047562E38F, 2.9933583E38F, 2.0239679E38F, -7.978047E37F, 1.1560929E38F, 1.9140914E38F, -1.5551203E38F, 1.2102884E38F, 1.2308614E38F, -2.7573475E38F, 2.709567E38F, -7.646086E37F, 3.1888477E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)5892660415258489207L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p64_x_SET((float)1.201785E38F, PH.base.pack) ;
    p64_y_SET((float) -1.6286222E38F, PH.base.pack) ;
    p64_z_SET((float)1.86348E38F, PH.base.pack) ;
    p64_vx_SET((float) -2.1263916E38F, PH.base.pack) ;
    p64_vy_SET((float)6.5467415E37F, PH.base.pack) ;
    p64_vz_SET((float) -1.5236731E38F, PH.base.pack) ;
    p64_ax_SET((float) -7.704774E37F, PH.base.pack) ;
    p64_ay_SET((float) -2.0786294E38F, PH.base.pack) ;
    p64_az_SET((float) -1.1553262E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-9.652002E37F, 3.3227621E38F, -2.1104902E38F, 1.2652629E38F, -2.9679516E38F, 4.347531E36F, -3.1678837E37F, -2.4995344E38F, 2.2481678E38F, -2.472278E38F, -2.158108E38F, -7.871566E37F, -3.2275378E38F, 2.2945447E36F, 1.3101349E38F, 1.3349187E38F, 1.0347964E38F, 1.881133E38F, 2.8028343E38F, -2.4549764E38F, -1.1780622E38F, 3.105566E38F, 2.101918E38F, 2.8549844E37F, -2.6191613E38F, -2.5162552E38F, 3.0568796E38F, -1.4182731E38F, -1.828052E38F, 6.375024E37F, -8.312061E37F, -1.663105E38F, 5.185277E37F, -1.4389396E37F, -1.9724443E38F, -2.6100928E38F, 2.1487657E38F, 4.901604E37F, 3.2241823E38F, 1.5813771E38F, -1.5493374E38F, -3.3571176E37F, -2.6377466E38F, 2.635736E37F, -8.512988E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)3289623758L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)11767, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)62937, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)46622, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)58327, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)6953, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)28735, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)43572, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)13168, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)31916, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)38096, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)29051, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)7361, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)21831, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)51238, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)711, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)13076, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)53614, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)63888, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)36937, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)39286, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -10896, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -24020, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)13331, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -5061, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)9454, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)62905, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)23588, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)3516, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)3615, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)51156, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)47809, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)50807, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)63521, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)4035, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p73_param1_SET((float) -1.4759479E37F, PH.base.pack) ;
    p73_param2_SET((float) -1.4037859E38F, PH.base.pack) ;
    p73_param3_SET((float) -1.9642944E38F, PH.base.pack) ;
    p73_param4_SET((float)2.1872386E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -1972018821, PH.base.pack) ;
    p73_y_SET((int32_t)1520505804, PH.base.pack) ;
    p73_z_SET((float)6.935635E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)9.520423E37F, PH.base.pack) ;
    p74_groundspeed_SET((float) -6.2254204E37F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)12316, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)48331, PH.base.pack) ;
    p74_alt_SET((float) -1.9775055E38F, PH.base.pack) ;
    p74_climb_SET((float) -1.9299453E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p75_param1_SET((float) -9.782453E37F, PH.base.pack) ;
    p75_param2_SET((float)3.1789063E38F, PH.base.pack) ;
    p75_param3_SET((float)2.4360056E38F, PH.base.pack) ;
    p75_param4_SET((float)2.5323777E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1231099348, PH.base.pack) ;
    p75_y_SET((int32_t) -780610472, PH.base.pack) ;
    p75_z_SET((float) -1.5510394E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p76_param1_SET((float) -1.4952808E38F, PH.base.pack) ;
    p76_param2_SET((float) -2.5294282E37F, PH.base.pack) ;
    p76_param3_SET((float) -1.5494264E38F, PH.base.pack) ;
    p76_param4_SET((float) -6.539415E37F, PH.base.pack) ;
    p76_param5_SET((float)1.0242885E38F, PH.base.pack) ;
    p76_param6_SET((float)3.0538512E38F, PH.base.pack) ;
    p76_param7_SET((float)3.0805901E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_LOGGING_STOP, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)131, &PH) ;
    p77_result_param2_SET((int32_t) -131929740, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)47, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)30, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)3484846398L, PH.base.pack) ;
    p81_roll_SET((float) -2.9552951E38F, PH.base.pack) ;
    p81_pitch_SET((float) -2.9928949E38F, PH.base.pack) ;
    p81_yaw_SET((float)2.8053272E38F, PH.base.pack) ;
    p81_thrust_SET((float) -3.023759E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2237265964L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    {
        float  q [] =  {3.1770614E38F, 5.27319E37F, 3.2121443E38F, 3.2535506E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -4.915552E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)8.127322E37F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)1.3996025E38F, PH.base.pack) ;
    p82_thrust_SET((float)1.6869456E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2463468948L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    {
        float  q [] =  {-1.5110257E38F, -2.5133889E38F, -2.1406256E38F, -1.9683764E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)3.9749053E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -2.981023E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -3.0523803E38F, PH.base.pack) ;
    p83_thrust_SET((float)1.945576E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)4182206120L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)9146, PH.base.pack) ;
    p84_x_SET((float)3.0357285E38F, PH.base.pack) ;
    p84_y_SET((float) -1.57165E38F, PH.base.pack) ;
    p84_z_SET((float)1.5594176E38F, PH.base.pack) ;
    p84_vx_SET((float)1.5567203E38F, PH.base.pack) ;
    p84_vy_SET((float) -1.7364243E38F, PH.base.pack) ;
    p84_vz_SET((float)1.365653E38F, PH.base.pack) ;
    p84_afx_SET((float) -1.5171839E38F, PH.base.pack) ;
    p84_afy_SET((float) -1.0611332E38F, PH.base.pack) ;
    p84_afz_SET((float)3.0908798E38F, PH.base.pack) ;
    p84_yaw_SET((float) -2.3755255E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -2.9810708E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)2942812397L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)63533, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -909348916, PH.base.pack) ;
    p86_lon_int_SET((int32_t)463751951, PH.base.pack) ;
    p86_alt_SET((float)6.478225E37F, PH.base.pack) ;
    p86_vx_SET((float) -2.7242765E38F, PH.base.pack) ;
    p86_vy_SET((float)3.1137322E38F, PH.base.pack) ;
    p86_vz_SET((float) -1.8658626E38F, PH.base.pack) ;
    p86_afx_SET((float)3.2315708E38F, PH.base.pack) ;
    p86_afy_SET((float) -4.2498876E37F, PH.base.pack) ;
    p86_afz_SET((float)2.404169E38F, PH.base.pack) ;
    p86_yaw_SET((float) -2.9911808E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)1.2479014E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)1277054865L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)18096, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -540369994, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -1120900633, PH.base.pack) ;
    p87_alt_SET((float) -1.7208091E38F, PH.base.pack) ;
    p87_vx_SET((float) -1.0484882E38F, PH.base.pack) ;
    p87_vy_SET((float)2.810759E38F, PH.base.pack) ;
    p87_vz_SET((float)3.1332684E38F, PH.base.pack) ;
    p87_afx_SET((float)2.523717E37F, PH.base.pack) ;
    p87_afy_SET((float) -4.3261335E37F, PH.base.pack) ;
    p87_afz_SET((float)2.8827713E38F, PH.base.pack) ;
    p87_yaw_SET((float) -3.0948888E37F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -8.021266E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)124256800L, PH.base.pack) ;
    p89_x_SET((float)2.1632269E38F, PH.base.pack) ;
    p89_y_SET((float)1.3236162E38F, PH.base.pack) ;
    p89_z_SET((float)1.3347306E38F, PH.base.pack) ;
    p89_roll_SET((float) -1.1113593E38F, PH.base.pack) ;
    p89_pitch_SET((float) -1.6860572E38F, PH.base.pack) ;
    p89_yaw_SET((float) -1.4599637E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)4966552925077419386L, PH.base.pack) ;
    p90_roll_SET((float)1.4599463E38F, PH.base.pack) ;
    p90_pitch_SET((float) -9.867726E37F, PH.base.pack) ;
    p90_yaw_SET((float)3.1933025E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -2.4924678E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float)1.3858559E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -1.7383169E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -77726136, PH.base.pack) ;
    p90_lon_SET((int32_t)131416028, PH.base.pack) ;
    p90_alt_SET((int32_t) -140203074, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -1473, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)24230, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)18359, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)22649, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)705, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)11001, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)6396081019214713440L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)2.7459744E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)2.8875251E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.3165521E38F, PH.base.pack) ;
    p91_throttle_SET((float)2.1519042E38F, PH.base.pack) ;
    p91_aux1_SET((float)2.6626938E38F, PH.base.pack) ;
    p91_aux2_SET((float) -1.6450488E37F, PH.base.pack) ;
    p91_aux3_SET((float)7.2815004E36F, PH.base.pack) ;
    p91_aux4_SET((float)1.6177931E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)8226391519781741079L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)8862, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)17281, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)51676, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)54231, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)19766, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)54602, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)46090, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)26773, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)19891, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)2566, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)62588, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)10870, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)1779789412587414145L, PH.base.pack) ;
    {
        float  controls [] =  {-1.8689096E37F, 1.7034575E38F, 1.6253668E38F, 1.479788E38F, -3.3524278E38F, -1.8088484E38F, 1.233947E38F, -2.557367E38F, -6.861498E37F, -2.2143734E38F, -1.1408735E38F, 2.8679822E38F, -1.1474357E38F, -5.11341E37F, 2.08494E38F, 2.9167012E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)653884966165270536L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)7075012865007793890L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -2589, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -15477, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -2.8268779E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)6.469784E36F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p100_ground_distance_SET((float) -1.8818423E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)2.7299621E37F, &PH) ;
    p100_flow_rate_y_SET((float)4.7852974E37F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)8903018815301065300L, PH.base.pack) ;
    p101_x_SET((float)2.9004426E38F, PH.base.pack) ;
    p101_y_SET((float) -1.8558028E38F, PH.base.pack) ;
    p101_z_SET((float)3.0770638E38F, PH.base.pack) ;
    p101_roll_SET((float) -1.8948397E38F, PH.base.pack) ;
    p101_pitch_SET((float) -1.0824086E38F, PH.base.pack) ;
    p101_yaw_SET((float) -1.3938023E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)3234140997363300423L, PH.base.pack) ;
    p102_x_SET((float)1.4617665E38F, PH.base.pack) ;
    p102_y_SET((float)6.717744E37F, PH.base.pack) ;
    p102_z_SET((float) -1.9866223E38F, PH.base.pack) ;
    p102_roll_SET((float) -3.9687306E37F, PH.base.pack) ;
    p102_pitch_SET((float) -9.832328E37F, PH.base.pack) ;
    p102_yaw_SET((float) -1.7717151E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)3105714696698579095L, PH.base.pack) ;
    p103_x_SET((float) -1.484447E38F, PH.base.pack) ;
    p103_y_SET((float) -9.755492E37F, PH.base.pack) ;
    p103_z_SET((float)2.962987E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)2262287752541598780L, PH.base.pack) ;
    p104_x_SET((float)5.566885E37F, PH.base.pack) ;
    p104_y_SET((float) -1.3888337E38F, PH.base.pack) ;
    p104_z_SET((float) -1.0256445E38F, PH.base.pack) ;
    p104_roll_SET((float)1.0909452E38F, PH.base.pack) ;
    p104_pitch_SET((float)1.6151913E38F, PH.base.pack) ;
    p104_yaw_SET((float)9.4721724E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)5149146213638547577L, PH.base.pack) ;
    p105_xacc_SET((float) -6.871093E37F, PH.base.pack) ;
    p105_yacc_SET((float)2.2220824E38F, PH.base.pack) ;
    p105_zacc_SET((float) -1.3573251E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -1.6779531E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -7.3932836E37F, PH.base.pack) ;
    p105_zgyro_SET((float)2.080398E38F, PH.base.pack) ;
    p105_xmag_SET((float)2.7840676E38F, PH.base.pack) ;
    p105_ymag_SET((float)9.308741E37F, PH.base.pack) ;
    p105_zmag_SET((float)1.4872744E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -2.2230131E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)2.3091996E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)4.4598116E36F, PH.base.pack) ;
    p105_temperature_SET((float) -1.789833E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)16615, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)4406097185365688088L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)4123925282L, PH.base.pack) ;
    p106_integrated_x_SET((float) -2.046529E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)2.7913762E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float)1.899594E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -7.207099E36F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -2.0327337E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -9549, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)3150878994L, PH.base.pack) ;
    p106_distance_SET((float) -2.679474E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)7335287273432684697L, PH.base.pack) ;
    p107_xacc_SET((float)1.4821006E38F, PH.base.pack) ;
    p107_yacc_SET((float) -1.4708126E38F, PH.base.pack) ;
    p107_zacc_SET((float)1.7327293E38F, PH.base.pack) ;
    p107_xgyro_SET((float)2.4725764E38F, PH.base.pack) ;
    p107_ygyro_SET((float)2.2793066E38F, PH.base.pack) ;
    p107_zgyro_SET((float)2.6222463E37F, PH.base.pack) ;
    p107_xmag_SET((float) -2.8684173E38F, PH.base.pack) ;
    p107_ymag_SET((float)1.9704085E38F, PH.base.pack) ;
    p107_zmag_SET((float)1.7638277E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)6.887001E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float)1.2266984E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)8.985575E37F, PH.base.pack) ;
    p107_temperature_SET((float)2.9196648E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1938280518L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)8.941946E37F, PH.base.pack) ;
    p108_q2_SET((float)8.3904013E37F, PH.base.pack) ;
    p108_q3_SET((float)9.055965E37F, PH.base.pack) ;
    p108_q4_SET((float)1.8640881E37F, PH.base.pack) ;
    p108_roll_SET((float) -1.5730961E38F, PH.base.pack) ;
    p108_pitch_SET((float) -3.0243918E38F, PH.base.pack) ;
    p108_yaw_SET((float) -1.1255244E38F, PH.base.pack) ;
    p108_xacc_SET((float)2.2749183E38F, PH.base.pack) ;
    p108_yacc_SET((float) -9.612675E37F, PH.base.pack) ;
    p108_zacc_SET((float)1.097432E38F, PH.base.pack) ;
    p108_xgyro_SET((float)9.030737E37F, PH.base.pack) ;
    p108_ygyro_SET((float)1.9857883E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.0332875E38F, PH.base.pack) ;
    p108_lat_SET((float) -2.9396284E38F, PH.base.pack) ;
    p108_lon_SET((float)1.9098582E38F, PH.base.pack) ;
    p108_alt_SET((float) -1.169805E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)1.047681E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -9.306671E37F, PH.base.pack) ;
    p108_vn_SET((float) -1.4213141E38F, PH.base.pack) ;
    p108_ve_SET((float) -2.7098737E38F, PH.base.pack) ;
    p108_vd_SET((float) -2.6125703E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)17957, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)34306, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)216, (uint8_t)117, (uint8_t)161, (uint8_t)35, (uint8_t)186, (uint8_t)252, (uint8_t)198, (uint8_t)182, (uint8_t)111, (uint8_t)107, (uint8_t)213, (uint8_t)57, (uint8_t)137, (uint8_t)61, (uint8_t)23, (uint8_t)224, (uint8_t)84, (uint8_t)70, (uint8_t)233, (uint8_t)154, (uint8_t)210, (uint8_t)188, (uint8_t)202, (uint8_t)243, (uint8_t)146, (uint8_t)254, (uint8_t)16, (uint8_t)85, (uint8_t)107, (uint8_t)98, (uint8_t)191, (uint8_t)156, (uint8_t)6, (uint8_t)169, (uint8_t)165, (uint8_t)39, (uint8_t)139, (uint8_t)160, (uint8_t)109, (uint8_t)188, (uint8_t)193, (uint8_t)184, (uint8_t)46, (uint8_t)160, (uint8_t)15, (uint8_t)34, (uint8_t)86, (uint8_t)243, (uint8_t)14, (uint8_t)56, (uint8_t)224, (uint8_t)172, (uint8_t)211, (uint8_t)144, (uint8_t)204, (uint8_t)171, (uint8_t)0, (uint8_t)159, (uint8_t)237, (uint8_t)12, (uint8_t)140, (uint8_t)251, (uint8_t)50, (uint8_t)84, (uint8_t)170, (uint8_t)94, (uint8_t)17, (uint8_t)83, (uint8_t)173, (uint8_t)83, (uint8_t)96, (uint8_t)14, (uint8_t)103, (uint8_t)53, (uint8_t)59, (uint8_t)170, (uint8_t)242, (uint8_t)178, (uint8_t)11, (uint8_t)94, (uint8_t)213, (uint8_t)167, (uint8_t)247, (uint8_t)14, (uint8_t)187, (uint8_t)13, (uint8_t)137, (uint8_t)36, (uint8_t)167, (uint8_t)4, (uint8_t)216, (uint8_t)163, (uint8_t)124, (uint8_t)247, (uint8_t)104, (uint8_t)155, (uint8_t)18, (uint8_t)55, (uint8_t)155, (uint8_t)244, (uint8_t)51, (uint8_t)84, (uint8_t)230, (uint8_t)61, (uint8_t)95, (uint8_t)45, (uint8_t)255, (uint8_t)12, (uint8_t)195, (uint8_t)104, (uint8_t)201, (uint8_t)86, (uint8_t)149, (uint8_t)4, (uint8_t)251, (uint8_t)231, (uint8_t)117, (uint8_t)26, (uint8_t)241, (uint8_t)68, (uint8_t)56, (uint8_t)7, (uint8_t)88, (uint8_t)195, (uint8_t)170, (uint8_t)3, (uint8_t)170, (uint8_t)169, (uint8_t)55, (uint8_t)137, (uint8_t)236, (uint8_t)177, (uint8_t)148, (uint8_t)226, (uint8_t)130, (uint8_t)97, (uint8_t)166, (uint8_t)128, (uint8_t)42, (uint8_t)0, (uint8_t)43, (uint8_t)231, (uint8_t)11, (uint8_t)40, (uint8_t)134, (uint8_t)167, (uint8_t)153, (uint8_t)53, (uint8_t)59, (uint8_t)224, (uint8_t)242, (uint8_t)145, (uint8_t)255, (uint8_t)114, (uint8_t)69, (uint8_t)46, (uint8_t)182, (uint8_t)196, (uint8_t)111, (uint8_t)219, (uint8_t)69, (uint8_t)205, (uint8_t)188, (uint8_t)224, (uint8_t)175, (uint8_t)244, (uint8_t)228, (uint8_t)198, (uint8_t)120, (uint8_t)159, (uint8_t)224, (uint8_t)192, (uint8_t)31, (uint8_t)231, (uint8_t)61, (uint8_t)25, (uint8_t)182, (uint8_t)254, (uint8_t)154, (uint8_t)172, (uint8_t)123, (uint8_t)119, (uint8_t)93, (uint8_t)210, (uint8_t)9, (uint8_t)203, (uint8_t)25, (uint8_t)139, (uint8_t)196, (uint8_t)188, (uint8_t)255, (uint8_t)23, (uint8_t)137, (uint8_t)43, (uint8_t)36, (uint8_t)38, (uint8_t)75, (uint8_t)93, (uint8_t)165, (uint8_t)20, (uint8_t)95, (uint8_t)208, (uint8_t)213, (uint8_t)164, (uint8_t)29, (uint8_t)177, (uint8_t)185, (uint8_t)44, (uint8_t)175, (uint8_t)141, (uint8_t)56, (uint8_t)236, (uint8_t)2, (uint8_t)77, (uint8_t)96, (uint8_t)246, (uint8_t)166, (uint8_t)15, (uint8_t)109, (uint8_t)31, (uint8_t)148, (uint8_t)237, (uint8_t)147, (uint8_t)127, (uint8_t)16, (uint8_t)90, (uint8_t)186, (uint8_t)173, (uint8_t)137, (uint8_t)88, (uint8_t)219, (uint8_t)112, (uint8_t)165, (uint8_t)59, (uint8_t)135, (uint8_t)160, (uint8_t)227, (uint8_t)238, (uint8_t)107, (uint8_t)104, (uint8_t)40, (uint8_t)27, (uint8_t)67, (uint8_t)34, (uint8_t)149, (uint8_t)1, (uint8_t)31, (uint8_t)249, (uint8_t)223, (uint8_t)234, (uint8_t)116};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)6042789090675186856L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -1509642448877789389L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)2993709138679941921L, PH.base.pack) ;
    p112_seq_SET((uint32_t)2221963629L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)1385604458720795658L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p113_lat_SET((int32_t) -796208617, PH.base.pack) ;
    p113_lon_SET((int32_t)132816217, PH.base.pack) ;
    p113_alt_SET((int32_t)1175453500, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)47078, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)24198, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)29608, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)16513, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)13322, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -28972, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)4972, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)8636829849765881453L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)2234816177L, PH.base.pack) ;
    p114_integrated_x_SET((float) -9.18617E37F, PH.base.pack) ;
    p114_integrated_y_SET((float) -7.525761E37F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)3.0828395E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -2.1178716E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -5.237039E37F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)30957, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)823719814L, PH.base.pack) ;
    p114_distance_SET((float)2.5633475E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)7884938667769122977L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {1.3477098E38F, 1.6752809E38F, -2.27729E38F, -1.7851644E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -1.207192E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)2.7662195E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)3.2018722E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -23138258, PH.base.pack) ;
    p115_lon_SET((int32_t) -1191370547, PH.base.pack) ;
    p115_alt_SET((int32_t) -1260035016, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)17808, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)29841, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -151, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)2763, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)59389, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)8232, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -31008, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)21630, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)505453523L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)18939, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -2476, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -31867, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)796, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -20688, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)16036, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)8738, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -25291, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -17322, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)24179, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)9919, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)64732, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)62245, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)32509, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2623837786L, PH.base.pack) ;
    p118_size_SET((uint32_t)3771801236L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)16373, PH.base.pack) ;
    p119_ofs_SET((uint32_t)1946608679L, PH.base.pack) ;
    p119_count_SET((uint32_t)2545470950L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)17990, PH.base.pack) ;
    p120_ofs_SET((uint32_t)3429653995L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)54, (uint8_t)232, (uint8_t)147, (uint8_t)160, (uint8_t)98, (uint8_t)234, (uint8_t)153, (uint8_t)230, (uint8_t)61, (uint8_t)6, (uint8_t)58, (uint8_t)130, (uint8_t)35, (uint8_t)104, (uint8_t)78, (uint8_t)0, (uint8_t)160, (uint8_t)245, (uint8_t)25, (uint8_t)72, (uint8_t)161, (uint8_t)73, (uint8_t)208, (uint8_t)245, (uint8_t)117, (uint8_t)227, (uint8_t)243, (uint8_t)120, (uint8_t)127, (uint8_t)183, (uint8_t)46, (uint8_t)43, (uint8_t)7, (uint8_t)28, (uint8_t)217, (uint8_t)29, (uint8_t)233, (uint8_t)99, (uint8_t)69, (uint8_t)192, (uint8_t)91, (uint8_t)212, (uint8_t)80, (uint8_t)113, (uint8_t)192, (uint8_t)196, (uint8_t)98, (uint8_t)94, (uint8_t)227, (uint8_t)48, (uint8_t)235, (uint8_t)224, (uint8_t)156, (uint8_t)227, (uint8_t)16, (uint8_t)214, (uint8_t)240, (uint8_t)48, (uint8_t)242, (uint8_t)111, (uint8_t)236, (uint8_t)46, (uint8_t)166, (uint8_t)78, (uint8_t)72, (uint8_t)112, (uint8_t)73, (uint8_t)45, (uint8_t)227, (uint8_t)253, (uint8_t)69, (uint8_t)124, (uint8_t)32, (uint8_t)163, (uint8_t)211, (uint8_t)194, (uint8_t)73, (uint8_t)146, (uint8_t)184, (uint8_t)127, (uint8_t)190, (uint8_t)26, (uint8_t)147, (uint8_t)234, (uint8_t)101, (uint8_t)109, (uint8_t)57, (uint8_t)75, (uint8_t)78, (uint8_t)242};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)230, (uint8_t)0, (uint8_t)3, (uint8_t)30, (uint8_t)234, (uint8_t)118, (uint8_t)28, (uint8_t)80, (uint8_t)124, (uint8_t)14, (uint8_t)11, (uint8_t)230, (uint8_t)135, (uint8_t)176, (uint8_t)184, (uint8_t)145, (uint8_t)146, (uint8_t)253, (uint8_t)223, (uint8_t)94, (uint8_t)251, (uint8_t)167, (uint8_t)31, (uint8_t)10, (uint8_t)31, (uint8_t)81, (uint8_t)155, (uint8_t)9, (uint8_t)185, (uint8_t)145, (uint8_t)208, (uint8_t)194, (uint8_t)13, (uint8_t)127, (uint8_t)141, (uint8_t)86, (uint8_t)193, (uint8_t)50, (uint8_t)189, (uint8_t)182, (uint8_t)95, (uint8_t)229, (uint8_t)252, (uint8_t)178, (uint8_t)253, (uint8_t)106, (uint8_t)33, (uint8_t)235, (uint8_t)19, (uint8_t)246, (uint8_t)212, (uint8_t)212, (uint8_t)189, (uint8_t)247, (uint8_t)89, (uint8_t)66, (uint8_t)205, (uint8_t)239, (uint8_t)118, (uint8_t)100, (uint8_t)213, (uint8_t)133, (uint8_t)144, (uint8_t)99, (uint8_t)88, (uint8_t)24, (uint8_t)82, (uint8_t)46, (uint8_t)24, (uint8_t)55, (uint8_t)111, (uint8_t)23, (uint8_t)17, (uint8_t)49, (uint8_t)124, (uint8_t)228, (uint8_t)211, (uint8_t)122, (uint8_t)197, (uint8_t)124, (uint8_t)87, (uint8_t)216, (uint8_t)198, (uint8_t)128, (uint8_t)221, (uint8_t)171, (uint8_t)5, (uint8_t)135, (uint8_t)140, (uint8_t)195, (uint8_t)226, (uint8_t)5, (uint8_t)50, (uint8_t)253, (uint8_t)235, (uint8_t)234, (uint8_t)96, (uint8_t)109, (uint8_t)254, (uint8_t)100, (uint8_t)251, (uint8_t)102, (uint8_t)34, (uint8_t)129, (uint8_t)105, (uint8_t)231, (uint8_t)73, (uint8_t)223, (uint8_t)97, (uint8_t)50};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)4572814608159728356L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t)1800074506, PH.base.pack) ;
    p124_lon_SET((int32_t) -66250220, PH.base.pack) ;
    p124_alt_SET((int32_t) -1926605188, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)30027, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)40512, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)55074, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)36108, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2060007364L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)13528, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)56164, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)17654, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)2900570829L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)75, (uint8_t)155, (uint8_t)253, (uint8_t)146, (uint8_t)160, (uint8_t)163, (uint8_t)144, (uint8_t)52, (uint8_t)145, (uint8_t)17, (uint8_t)188, (uint8_t)48, (uint8_t)182, (uint8_t)102, (uint8_t)153, (uint8_t)15, (uint8_t)89, (uint8_t)228, (uint8_t)79, (uint8_t)172, (uint8_t)226, (uint8_t)95, (uint8_t)48, (uint8_t)225, (uint8_t)1, (uint8_t)194, (uint8_t)155, (uint8_t)143, (uint8_t)126, (uint8_t)176, (uint8_t)251, (uint8_t)197, (uint8_t)40, (uint8_t)214, (uint8_t)152, (uint8_t)4, (uint8_t)17, (uint8_t)112, (uint8_t)187, (uint8_t)103, (uint8_t)140, (uint8_t)126, (uint8_t)158, (uint8_t)116, (uint8_t)198, (uint8_t)106, (uint8_t)50, (uint8_t)149, (uint8_t)216, (uint8_t)67, (uint8_t)42, (uint8_t)68, (uint8_t)219, (uint8_t)136, (uint8_t)74, (uint8_t)102, (uint8_t)114, (uint8_t)65, (uint8_t)101, (uint8_t)123, (uint8_t)163, (uint8_t)68, (uint8_t)249, (uint8_t)202, (uint8_t)31, (uint8_t)204, (uint8_t)182, (uint8_t)34, (uint8_t)33, (uint8_t)158};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)104181435L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)43350, PH.base.pack) ;
    p127_tow_SET((uint32_t)3957779213L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)1142045337, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -785827315, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)1051423778, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)526217254L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1911355890, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)1913147554L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)1516, PH.base.pack) ;
    p128_tow_SET((uint32_t)4087925416L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)1623307599, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -612138516, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -249648975, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)3127175519L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -822604092, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)1848192185L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -6545, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)18712, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)21598, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -31581, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -29862, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -23263, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -11165, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)3372, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -24832, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p130_size_SET((uint32_t)925986643L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)6072, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)21008, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)54057, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)32166, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)29, (uint8_t)138, (uint8_t)198, (uint8_t)61, (uint8_t)13, (uint8_t)33, (uint8_t)33, (uint8_t)207, (uint8_t)22, (uint8_t)99, (uint8_t)38, (uint8_t)214, (uint8_t)226, (uint8_t)200, (uint8_t)242, (uint8_t)155, (uint8_t)195, (uint8_t)191, (uint8_t)46, (uint8_t)147, (uint8_t)158, (uint8_t)181, (uint8_t)18, (uint8_t)190, (uint8_t)186, (uint8_t)121, (uint8_t)196, (uint8_t)196, (uint8_t)183, (uint8_t)201, (uint8_t)76, (uint8_t)104, (uint8_t)40, (uint8_t)65, (uint8_t)212, (uint8_t)197, (uint8_t)147, (uint8_t)200, (uint8_t)149, (uint8_t)163, (uint8_t)236, (uint8_t)139, (uint8_t)56, (uint8_t)60, (uint8_t)181, (uint8_t)86, (uint8_t)68, (uint8_t)212, (uint8_t)13, (uint8_t)156, (uint8_t)49, (uint8_t)6, (uint8_t)49, (uint8_t)250, (uint8_t)245, (uint8_t)123, (uint8_t)230, (uint8_t)36, (uint8_t)53, (uint8_t)171, (uint8_t)252, (uint8_t)250, (uint8_t)19, (uint8_t)51, (uint8_t)14, (uint8_t)154, (uint8_t)243, (uint8_t)150, (uint8_t)189, (uint8_t)229, (uint8_t)8, (uint8_t)218, (uint8_t)14, (uint8_t)49, (uint8_t)16, (uint8_t)192, (uint8_t)48, (uint8_t)173, (uint8_t)52, (uint8_t)101, (uint8_t)12, (uint8_t)152, (uint8_t)213, (uint8_t)112, (uint8_t)41, (uint8_t)26, (uint8_t)221, (uint8_t)145, (uint8_t)70, (uint8_t)185, (uint8_t)16, (uint8_t)140, (uint8_t)0, (uint8_t)213, (uint8_t)209, (uint8_t)228, (uint8_t)250, (uint8_t)125, (uint8_t)82, (uint8_t)243, (uint8_t)72, (uint8_t)117, (uint8_t)22, (uint8_t)223, (uint8_t)90, (uint8_t)254, (uint8_t)1, (uint8_t)73, (uint8_t)111, (uint8_t)247, (uint8_t)182, (uint8_t)58, (uint8_t)8, (uint8_t)55, (uint8_t)105, (uint8_t)11, (uint8_t)13, (uint8_t)174, (uint8_t)206, (uint8_t)207, (uint8_t)54, (uint8_t)25, (uint8_t)186, (uint8_t)206, (uint8_t)81, (uint8_t)122, (uint8_t)204, (uint8_t)73, (uint8_t)22, (uint8_t)51, (uint8_t)236, (uint8_t)40, (uint8_t)16, (uint8_t)96, (uint8_t)250, (uint8_t)120, (uint8_t)244, (uint8_t)221, (uint8_t)193, (uint8_t)4, (uint8_t)175, (uint8_t)250, (uint8_t)171, (uint8_t)188, (uint8_t)13, (uint8_t)248, (uint8_t)195, (uint8_t)138, (uint8_t)129, (uint8_t)190, (uint8_t)98, (uint8_t)148, (uint8_t)254, (uint8_t)141, (uint8_t)9, (uint8_t)154, (uint8_t)33, (uint8_t)68, (uint8_t)94, (uint8_t)25, (uint8_t)172, (uint8_t)93, (uint8_t)191, (uint8_t)164, (uint8_t)33, (uint8_t)89, (uint8_t)71, (uint8_t)138, (uint8_t)129, (uint8_t)75, (uint8_t)37, (uint8_t)12, (uint8_t)72, (uint8_t)17, (uint8_t)53, (uint8_t)228, (uint8_t)15, (uint8_t)113, (uint8_t)37, (uint8_t)118, (uint8_t)113, (uint8_t)252, (uint8_t)159, (uint8_t)162, (uint8_t)206, (uint8_t)131, (uint8_t)200, (uint8_t)64, (uint8_t)90, (uint8_t)58, (uint8_t)181, (uint8_t)39, (uint8_t)106, (uint8_t)195, (uint8_t)157, (uint8_t)138, (uint8_t)234, (uint8_t)184, (uint8_t)39, (uint8_t)154, (uint8_t)235, (uint8_t)218, (uint8_t)23, (uint8_t)65, (uint8_t)117, (uint8_t)244, (uint8_t)55, (uint8_t)121, (uint8_t)166, (uint8_t)153, (uint8_t)71, (uint8_t)79, (uint8_t)148, (uint8_t)125, (uint8_t)63, (uint8_t)229, (uint8_t)225, (uint8_t)132, (uint8_t)155, (uint8_t)31, (uint8_t)2, (uint8_t)233, (uint8_t)190, (uint8_t)135, (uint8_t)182, (uint8_t)157, (uint8_t)156, (uint8_t)236, (uint8_t)125, (uint8_t)97, (uint8_t)133, (uint8_t)165, (uint8_t)255, (uint8_t)129, (uint8_t)61, (uint8_t)136, (uint8_t)175, (uint8_t)179, (uint8_t)241, (uint8_t)32, (uint8_t)255, (uint8_t)31, (uint8_t)41, (uint8_t)127, (uint8_t)79, (uint8_t)14, (uint8_t)173, (uint8_t)168, (uint8_t)12, (uint8_t)220, (uint8_t)95, (uint8_t)163, (uint8_t)105};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2653354898L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)7313, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)48266, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)60097, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_315, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)215772594, PH.base.pack) ;
    p133_lon_SET((int32_t) -1878565637, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)34853, PH.base.pack) ;
    p133_mask_SET((uint64_t)217866037769797240L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1577107950, PH.base.pack) ;
    p134_lon_SET((int32_t) -91244086, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)14664, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -30903, (int16_t) -10993, (int16_t) -1564, (int16_t) -12808, (int16_t)5824, (int16_t) -8351, (int16_t)30500, (int16_t)26242, (int16_t) -4902, (int16_t)29604, (int16_t)17204, (int16_t) -30692, (int16_t)23450, (int16_t)30216, (int16_t)25926, (int16_t)32072};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1760687259, PH.base.pack) ;
    p135_lon_SET((int32_t)771311624, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)162954270, PH.base.pack) ;
    p136_lon_SET((int32_t) -2037962970, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)6555, PH.base.pack) ;
    p136_terrain_height_SET((float) -9.755849E37F, PH.base.pack) ;
    p136_current_height_SET((float) -2.1298258E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)12800, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)8499, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)2995381698L, PH.base.pack) ;
    p137_press_abs_SET((float) -1.8742021E36F, PH.base.pack) ;
    p137_press_diff_SET((float)5.221684E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)4247, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)4743725291535163921L, PH.base.pack) ;
    {
        float  q [] =  {-6.669713E37F, 2.415272E38F, 2.367555E38F, -4.2452952E37F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -6.9483935E37F, PH.base.pack) ;
    p138_y_SET((float) -4.4476743E37F, PH.base.pack) ;
    p138_z_SET((float) -2.9417832E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)9095405092083369228L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    {
        float  controls [] =  {5.484137E37F, 7.59644E37F, -7.81321E37F, 1.9381439E38F, -1.602539E38F, 2.6335125E38F, 1.2785359E38F, -7.410354E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)2679021332177409699L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    {
        float  controls [] =  {2.495076E38F, 1.4497523E37F, 2.374608E38F, -2.6050512E38F, 5.956831E37F, 2.233262E38F, -2.0638743E38F, -3.3352753E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)196250401628956262L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.0233338E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -1.8000783E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.0408229E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.5216677E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -2.7632104E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)9.315781E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)200, (uint8_t)41, (uint8_t)108, (uint8_t)94, (uint8_t)247, (uint8_t)208, (uint8_t)217, (uint8_t)2, (uint8_t)187, (uint8_t)86, (uint8_t)138, (uint8_t)3, (uint8_t)69, (uint8_t)12, (uint8_t)248, (uint8_t)162, (uint8_t)146, (uint8_t)112, (uint8_t)105, (uint8_t)188, (uint8_t)96, (uint8_t)248, (uint8_t)105, (uint8_t)19, (uint8_t)149, (uint8_t)21, (uint8_t)217, (uint8_t)7, (uint8_t)228, (uint8_t)217, (uint8_t)219, (uint8_t)25, (uint8_t)221, (uint8_t)110, (uint8_t)176, (uint8_t)189, (uint8_t)24, (uint8_t)15, (uint8_t)110, (uint8_t)142, (uint8_t)125, (uint8_t)125, (uint8_t)246, (uint8_t)99, (uint8_t)155, (uint8_t)112, (uint8_t)135, (uint8_t)115, (uint8_t)225, (uint8_t)9, (uint8_t)157, (uint8_t)63, (uint8_t)33, (uint8_t)223, (uint8_t)155, (uint8_t)189, (uint8_t)227, (uint8_t)123, (uint8_t)53, (uint8_t)180, (uint8_t)122, (uint8_t)215, (uint8_t)171, (uint8_t)212, (uint8_t)58, (uint8_t)52, (uint8_t)222, (uint8_t)98, (uint8_t)58, (uint8_t)222, (uint8_t)38, (uint8_t)48, (uint8_t)121, (uint8_t)28, (uint8_t)58, (uint8_t)144, (uint8_t)125, (uint8_t)193, (uint8_t)62, (uint8_t)146, (uint8_t)246, (uint8_t)135, (uint8_t)193, (uint8_t)177, (uint8_t)74, (uint8_t)95, (uint8_t)14, (uint8_t)223, (uint8_t)248, (uint8_t)111, (uint8_t)126, (uint8_t)106, (uint8_t)106, (uint8_t)253, (uint8_t)64, (uint8_t)76, (uint8_t)139, (uint8_t)70, (uint8_t)240, (uint8_t)246, (uint8_t)113, (uint8_t)143, (uint8_t)79, (uint8_t)160, (uint8_t)221, (uint8_t)210, (uint8_t)119, (uint8_t)211, (uint8_t)7, (uint8_t)197, (uint8_t)97, (uint8_t)130, (uint8_t)155, (uint8_t)43, (uint8_t)90, (uint8_t)244, (uint8_t)228, (uint8_t)160, (uint8_t)33, (uint8_t)194};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)170, (uint8_t)171, (uint8_t)255, (uint8_t)24, (uint8_t)225, (uint8_t)229, (uint8_t)188, (uint8_t)253, (uint8_t)163, (uint8_t)155, (uint8_t)190, (uint8_t)209, (uint8_t)225, (uint8_t)117, (uint8_t)120, (uint8_t)254, (uint8_t)35, (uint8_t)38, (uint8_t)143, (uint8_t)31, (uint8_t)21, (uint8_t)176, (uint8_t)25, (uint8_t)186, (uint8_t)188, (uint8_t)139, (uint8_t)36, (uint8_t)17, (uint8_t)142, (uint8_t)135, (uint8_t)95, (uint8_t)41, (uint8_t)13, (uint8_t)49, (uint8_t)60, (uint8_t)214, (uint8_t)79, (uint8_t)154, (uint8_t)105, (uint8_t)104, (uint8_t)222, (uint8_t)158, (uint8_t)217, (uint8_t)125, (uint8_t)157, (uint8_t)218, (uint8_t)185, (uint8_t)125, (uint8_t)208, (uint8_t)33, (uint8_t)223, (uint8_t)83, (uint8_t)247, (uint8_t)130, (uint8_t)244, (uint8_t)201, (uint8_t)134, (uint8_t)22, (uint8_t)4, (uint8_t)170, (uint8_t)14, (uint8_t)147, (uint8_t)65, (uint8_t)151, (uint8_t)26, (uint8_t)158, (uint8_t)3, (uint8_t)150, (uint8_t)220, (uint8_t)84, (uint8_t)119, (uint8_t)204, (uint8_t)54, (uint8_t)222, (uint8_t)98, (uint8_t)7, (uint8_t)142, (uint8_t)124, (uint8_t)249, (uint8_t)228, (uint8_t)211, (uint8_t)118, (uint8_t)252, (uint8_t)121, (uint8_t)28, (uint8_t)159, (uint8_t)59, (uint8_t)119, (uint8_t)12, (uint8_t)97, (uint8_t)46, (uint8_t)254, (uint8_t)96, (uint8_t)196, (uint8_t)64, (uint8_t)61, (uint8_t)81, (uint8_t)16, (uint8_t)249, (uint8_t)15, (uint8_t)139, (uint8_t)222, (uint8_t)85, (uint8_t)97, (uint8_t)96, (uint8_t)86, (uint8_t)73, (uint8_t)58, (uint8_t)215, (uint8_t)236, (uint8_t)181, (uint8_t)62, (uint8_t)165, (uint8_t)78, (uint8_t)62, (uint8_t)42, (uint8_t)111, (uint8_t)143, (uint8_t)165, (uint8_t)183};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)3656290222L, PH.base.pack) ;
    p143_press_abs_SET((float)7.504156E37F, PH.base.pack) ;
    p143_press_diff_SET((float)2.4243223E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)5468, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)4446082311327752277L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p144_lat_SET((int32_t)380678884, PH.base.pack) ;
    p144_lon_SET((int32_t)1188774745, PH.base.pack) ;
    p144_alt_SET((float) -1.792808E38F, PH.base.pack) ;
    {
        float  vel [] =  {-2.4838756E38F, 1.3831676E38F, 4.259463E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-5.899156E37F, 2.4197485E38F, 8.2938403E37F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-1.9749958E38F, -1.9612922E38F, -1.7161618E38F, -1.239149E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.3354078E38F, -2.7206074E38F, -2.0671227E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-9.620084E37F, 6.5137973E37F, 4.627098E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)4230233583320543126L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)278272342486133794L, PH.base.pack) ;
    p146_x_acc_SET((float)4.707193E37F, PH.base.pack) ;
    p146_y_acc_SET((float) -2.419196E38F, PH.base.pack) ;
    p146_z_acc_SET((float)4.1975504E37F, PH.base.pack) ;
    p146_x_vel_SET((float)2.8420483E38F, PH.base.pack) ;
    p146_y_vel_SET((float)2.4459189E38F, PH.base.pack) ;
    p146_z_vel_SET((float)2.7856056E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -2.10913E37F, PH.base.pack) ;
    p146_y_pos_SET((float)2.5626153E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -1.4628889E38F, PH.base.pack) ;
    p146_airspeed_SET((float) -6.7260735E37F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.3903909E38F, 2.0263644E38F, -3.199873E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {2.7987873E38F, -2.8151678E38F, 6.769613E37F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-2.91137E38F, 3.2568272E38F, -1.8275098E38F, 2.5958564E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)1.4803347E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)1.0359767E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -1.7400271E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -26911, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)40667, (uint16_t)46301, (uint16_t)33742, (uint16_t)26311, (uint16_t)35695, (uint16_t)31200, (uint16_t)49296, (uint16_t)45118, (uint16_t)49085, (uint16_t)57378};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -11965, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -453309660, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -1157436477, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -92, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)3039119823L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)134398346L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)3533573331L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)256018627L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)18, (uint8_t)125, (uint8_t)40, (uint8_t)54, (uint8_t)164, (uint8_t)153, (uint8_t)108, (uint8_t)228};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)204, (uint8_t)36, (uint8_t)150, (uint8_t)62, (uint8_t)240, (uint8_t)64, (uint8_t)9, (uint8_t)99};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)46, (uint8_t)133, (uint8_t)187, (uint8_t)172, (uint8_t)126, (uint8_t)1, (uint8_t)203, (uint8_t)0};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)24214, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)52040, PH.base.pack) ;
    p148_uid_SET((uint64_t)6081962366572518352L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)218, (uint8_t)90, (uint8_t)255, (uint8_t)185, (uint8_t)227, (uint8_t)75, (uint8_t)233, (uint8_t)77, (uint8_t)28, (uint8_t)49, (uint8_t)130, (uint8_t)152, (uint8_t)59, (uint8_t)171, (uint8_t)194, (uint8_t)138, (uint8_t)55, (uint8_t)48};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)2574798722009597077L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p149_angle_x_SET((float) -1.5779184E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -1.9538844E38F, PH.base.pack) ;
    p149_distance_SET((float) -1.3526172E38F, PH.base.pack) ;
    p149_size_x_SET((float) -2.235263E37F, PH.base.pack) ;
    p149_size_y_SET((float) -9.650431E37F, PH.base.pack) ;
    p149_x_SET((float)2.6993246E38F, &PH) ;
    p149_y_SET((float) -2.4408541E38F, &PH) ;
    p149_z_SET((float)1.8892322E38F, &PH) ;
    {
        float  q [] =  {-2.195213E38F, 9.320201E37F, -2.4026583E38F, 1.6319673E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)45, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_SET_150(), &PH);
    p150_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p150_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_READ_REQ_151(), &PH);
    p151_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p151_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p151_read_req_type_SET((int16_t)(int16_t)23179, PH.base.pack) ;
    p151_data_index_SET((int16_t)(int16_t) -5561, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_BUFFER_FUNCTION_152(), &PH);
    p152_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p152_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p152_func_index_SET((uint16_t)(uint16_t)16293, PH.base.pack) ;
    p152_func_count_SET((uint16_t)(uint16_t)56429, PH.base.pack) ;
    p152_data_address_SET((uint16_t)(uint16_t)9247, PH.base.pack) ;
    p152_data_size_SET((uint16_t)(uint16_t)15641, PH.base.pack) ;
    {
        int8_t  data_ [] =  {(int8_t)23, (int8_t) -70, (int8_t) -44, (int8_t)22, (int8_t)106, (int8_t)11, (int8_t) -48, (int8_t)102, (int8_t) -25, (int8_t)105, (int8_t) -35, (int8_t)11, (int8_t) -4, (int8_t)126, (int8_t)82, (int8_t)48, (int8_t) -60, (int8_t)1, (int8_t)54, (int8_t) -93, (int8_t)98, (int8_t)76, (int8_t) -80, (int8_t) -111, (int8_t)35, (int8_t)58, (int8_t)90, (int8_t)103, (int8_t) -117, (int8_t)15, (int8_t)47, (int8_t) -80, (int8_t)77, (int8_t) -37, (int8_t)35, (int8_t)61, (int8_t)100, (int8_t) -26, (int8_t)78, (int8_t) -127, (int8_t)2, (int8_t) -91, (int8_t)114, (int8_t)76, (int8_t) -12, (int8_t) -59, (int8_t)18, (int8_t)72};
        p152_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(), &PH);
    p153_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p153_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p153_func_index_SET((uint16_t)(uint16_t)20798, PH.base.pack) ;
    p153_result_SET((uint16_t)(uint16_t)63631, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_DIRECTORY_155(), &PH);
    p155_target_system_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p155_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p155_directory_type_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p155_start_index_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p155_count_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    {
        int8_t  directory_data [] =  {(int8_t) -35, (int8_t) -90, (int8_t) -74, (int8_t) -14, (int8_t)87, (int8_t) -31, (int8_t) -100, (int8_t)122, (int8_t) -122, (int8_t) -94, (int8_t)101, (int8_t) -67, (int8_t)107, (int8_t)42, (int8_t)9, (int8_t)24, (int8_t) -88, (int8_t) -41, (int8_t) -72, (int8_t)77, (int8_t)90, (int8_t)8, (int8_t)63, (int8_t)77, (int8_t) -1, (int8_t) -22, (int8_t)118, (int8_t) -74, (int8_t)114, (int8_t) -33, (int8_t)20, (int8_t)37, (int8_t)96, (int8_t) -49, (int8_t) -36, (int8_t) -42, (int8_t) -72, (int8_t)113, (int8_t)127, (int8_t) -110, (int8_t)93, (int8_t)113, (int8_t)114, (int8_t) -83, (int8_t)109, (int8_t) -18, (int8_t)48, (int8_t)47};
        p155_directory_data_SET(&directory_data, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_DIRECTORY_ACK_156(), &PH);
    p156_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p156_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p156_directory_type_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p156_start_index_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p156_count_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p156_result_SET((uint16_t)(uint16_t)17636, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_COMMAND_157(), &PH);
    p157_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p157_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p157_command_type_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_COMMAND_ACK_158(), &PH);
    p158_command_type_SET((uint16_t)(uint16_t)39316, PH.base.pack) ;
    p158_result_SET((uint16_t)(uint16_t)39173, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F2_A_170(), &PH);
    p170_sue_time_SET((uint32_t)462987714L, PH.base.pack) ;
    p170_sue_status_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p170_sue_latitude_SET((int32_t)1328331546, PH.base.pack) ;
    p170_sue_longitude_SET((int32_t) -99724847, PH.base.pack) ;
    p170_sue_altitude_SET((int32_t) -506206721, PH.base.pack) ;
    p170_sue_waypoint_index_SET((uint16_t)(uint16_t)53652, PH.base.pack) ;
    p170_sue_rmat0_SET((int16_t)(int16_t)4385, PH.base.pack) ;
    p170_sue_rmat1_SET((int16_t)(int16_t) -22858, PH.base.pack) ;
    p170_sue_rmat2_SET((int16_t)(int16_t)16849, PH.base.pack) ;
    p170_sue_rmat3_SET((int16_t)(int16_t) -14776, PH.base.pack) ;
    p170_sue_rmat4_SET((int16_t)(int16_t) -17648, PH.base.pack) ;
    p170_sue_rmat5_SET((int16_t)(int16_t) -23790, PH.base.pack) ;
    p170_sue_rmat6_SET((int16_t)(int16_t)19618, PH.base.pack) ;
    p170_sue_rmat7_SET((int16_t)(int16_t) -1914, PH.base.pack) ;
    p170_sue_rmat8_SET((int16_t)(int16_t)10597, PH.base.pack) ;
    p170_sue_cog_SET((uint16_t)(uint16_t)17220, PH.base.pack) ;
    p170_sue_sog_SET((int16_t)(int16_t)5390, PH.base.pack) ;
    p170_sue_cpu_load_SET((uint16_t)(uint16_t)56423, PH.base.pack) ;
    p170_sue_air_speed_3DIMU_SET((uint16_t)(uint16_t)5484, PH.base.pack) ;
    p170_sue_estimated_wind_0_SET((int16_t)(int16_t)17822, PH.base.pack) ;
    p170_sue_estimated_wind_1_SET((int16_t)(int16_t)29168, PH.base.pack) ;
    p170_sue_estimated_wind_2_SET((int16_t)(int16_t)15022, PH.base.pack) ;
    p170_sue_magFieldEarth0_SET((int16_t)(int16_t)28338, PH.base.pack) ;
    p170_sue_magFieldEarth1_SET((int16_t)(int16_t) -12957, PH.base.pack) ;
    p170_sue_magFieldEarth2_SET((int16_t)(int16_t) -17615, PH.base.pack) ;
    p170_sue_svs_SET((int16_t)(int16_t) -7553, PH.base.pack) ;
    p170_sue_hdop_SET((int16_t)(int16_t)10567, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F2_B_171(), &PH);
    p171_sue_time_SET((uint32_t)3252855841L, PH.base.pack) ;
    p171_sue_pwm_input_1_SET((int16_t)(int16_t) -28271, PH.base.pack) ;
    p171_sue_pwm_input_2_SET((int16_t)(int16_t)18719, PH.base.pack) ;
    p171_sue_pwm_input_3_SET((int16_t)(int16_t) -7635, PH.base.pack) ;
    p171_sue_pwm_input_4_SET((int16_t)(int16_t)20381, PH.base.pack) ;
    p171_sue_pwm_input_5_SET((int16_t)(int16_t)838, PH.base.pack) ;
    p171_sue_pwm_input_6_SET((int16_t)(int16_t)23558, PH.base.pack) ;
    p171_sue_pwm_input_7_SET((int16_t)(int16_t)2412, PH.base.pack) ;
    p171_sue_pwm_input_8_SET((int16_t)(int16_t) -20378, PH.base.pack) ;
    p171_sue_pwm_input_9_SET((int16_t)(int16_t)7601, PH.base.pack) ;
    p171_sue_pwm_input_10_SET((int16_t)(int16_t)12067, PH.base.pack) ;
    p171_sue_pwm_input_11_SET((int16_t)(int16_t) -8208, PH.base.pack) ;
    p171_sue_pwm_input_12_SET((int16_t)(int16_t) -10979, PH.base.pack) ;
    p171_sue_pwm_output_1_SET((int16_t)(int16_t)31844, PH.base.pack) ;
    p171_sue_pwm_output_2_SET((int16_t)(int16_t) -15568, PH.base.pack) ;
    p171_sue_pwm_output_3_SET((int16_t)(int16_t)4475, PH.base.pack) ;
    p171_sue_pwm_output_4_SET((int16_t)(int16_t)32101, PH.base.pack) ;
    p171_sue_pwm_output_5_SET((int16_t)(int16_t) -12794, PH.base.pack) ;
    p171_sue_pwm_output_6_SET((int16_t)(int16_t)12173, PH.base.pack) ;
    p171_sue_pwm_output_7_SET((int16_t)(int16_t) -3055, PH.base.pack) ;
    p171_sue_pwm_output_8_SET((int16_t)(int16_t) -939, PH.base.pack) ;
    p171_sue_pwm_output_9_SET((int16_t)(int16_t)7974, PH.base.pack) ;
    p171_sue_pwm_output_10_SET((int16_t)(int16_t) -13456, PH.base.pack) ;
    p171_sue_pwm_output_11_SET((int16_t)(int16_t)13370, PH.base.pack) ;
    p171_sue_pwm_output_12_SET((int16_t)(int16_t)30306, PH.base.pack) ;
    p171_sue_imu_location_x_SET((int16_t)(int16_t)7656, PH.base.pack) ;
    p171_sue_imu_location_y_SET((int16_t)(int16_t)31593, PH.base.pack) ;
    p171_sue_imu_location_z_SET((int16_t)(int16_t) -29315, PH.base.pack) ;
    p171_sue_location_error_earth_x_SET((int16_t)(int16_t)25308, PH.base.pack) ;
    p171_sue_location_error_earth_y_SET((int16_t)(int16_t) -31461, PH.base.pack) ;
    p171_sue_location_error_earth_z_SET((int16_t)(int16_t)9114, PH.base.pack) ;
    p171_sue_flags_SET((uint32_t)712353028L, PH.base.pack) ;
    p171_sue_osc_fails_SET((int16_t)(int16_t) -15310, PH.base.pack) ;
    p171_sue_imu_velocity_x_SET((int16_t)(int16_t) -23698, PH.base.pack) ;
    p171_sue_imu_velocity_y_SET((int16_t)(int16_t) -14926, PH.base.pack) ;
    p171_sue_imu_velocity_z_SET((int16_t)(int16_t) -18453, PH.base.pack) ;
    p171_sue_waypoint_goal_x_SET((int16_t)(int16_t) -9238, PH.base.pack) ;
    p171_sue_waypoint_goal_y_SET((int16_t)(int16_t)7270, PH.base.pack) ;
    p171_sue_waypoint_goal_z_SET((int16_t)(int16_t) -13725, PH.base.pack) ;
    p171_sue_aero_x_SET((int16_t)(int16_t)20911, PH.base.pack) ;
    p171_sue_aero_y_SET((int16_t)(int16_t)26017, PH.base.pack) ;
    p171_sue_aero_z_SET((int16_t)(int16_t) -13888, PH.base.pack) ;
    p171_sue_barom_temp_SET((int16_t)(int16_t)29158, PH.base.pack) ;
    p171_sue_barom_press_SET((int32_t) -1186251251, PH.base.pack) ;
    p171_sue_barom_alt_SET((int32_t) -592623826, PH.base.pack) ;
    p171_sue_bat_volt_SET((int16_t)(int16_t) -27141, PH.base.pack) ;
    p171_sue_bat_amp_SET((int16_t)(int16_t) -8634, PH.base.pack) ;
    p171_sue_bat_amp_hours_SET((int16_t)(int16_t)7354, PH.base.pack) ;
    p171_sue_desired_height_SET((int16_t)(int16_t) -5352, PH.base.pack) ;
    p171_sue_memory_stack_free_SET((int16_t)(int16_t)27003, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F4_172(), &PH);
    p172_sue_ROLL_STABILIZATION_AILERONS_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p172_sue_ROLL_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p172_sue_PITCH_STABILIZATION_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p172_sue_YAW_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p172_sue_YAW_STABILIZATION_AILERON_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p172_sue_AILERON_NAVIGATION_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p172_sue_RUDDER_NAVIGATION_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p172_sue_ALTITUDEHOLD_STABILIZED_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p172_sue_ALTITUDEHOLD_WAYPOINT_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p172_sue_RACING_MODE_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F5_173(), &PH);
    p173_sue_YAWKP_AILERON_SET((float) -2.9715233E38F, PH.base.pack) ;
    p173_sue_YAWKD_AILERON_SET((float)1.3698274E37F, PH.base.pack) ;
    p173_sue_ROLLKP_SET((float) -1.8999736E38F, PH.base.pack) ;
    p173_sue_ROLLKD_SET((float) -2.774806E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F6_174(), &PH);
    p174_sue_PITCHGAIN_SET((float)7.3158646E37F, PH.base.pack) ;
    p174_sue_PITCHKD_SET((float) -3.0382246E38F, PH.base.pack) ;
    p174_sue_RUDDER_ELEV_MIX_SET((float) -8.3162443E37F, PH.base.pack) ;
    p174_sue_ROLL_ELEV_MIX_SET((float) -1.4835666E38F, PH.base.pack) ;
    p174_sue_ELEVATOR_BOOST_SET((float)2.6464805E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F7_175(), &PH);
    p175_sue_YAWKP_RUDDER_SET((float) -5.4726287E37F, PH.base.pack) ;
    p175_sue_YAWKD_RUDDER_SET((float) -2.1782308E38F, PH.base.pack) ;
    p175_sue_ROLLKP_RUDDER_SET((float) -2.3233095E38F, PH.base.pack) ;
    p175_sue_ROLLKD_RUDDER_SET((float) -1.2508326E38F, PH.base.pack) ;
    p175_sue_RUDDER_BOOST_SET((float) -1.1388589E38F, PH.base.pack) ;
    p175_sue_RTL_PITCH_DOWN_SET((float)2.7950428E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F8_176(), &PH);
    p176_sue_HEIGHT_TARGET_MAX_SET((float)2.260615E38F, PH.base.pack) ;
    p176_sue_HEIGHT_TARGET_MIN_SET((float)1.7594147E38F, PH.base.pack) ;
    p176_sue_ALT_HOLD_THROTTLE_MIN_SET((float)2.6770438E38F, PH.base.pack) ;
    p176_sue_ALT_HOLD_THROTTLE_MAX_SET((float)2.017413E38F, PH.base.pack) ;
    p176_sue_ALT_HOLD_PITCH_MIN_SET((float)1.4270443E38F, PH.base.pack) ;
    p176_sue_ALT_HOLD_PITCH_MAX_SET((float) -1.1848122E38F, PH.base.pack) ;
    p176_sue_ALT_HOLD_PITCH_HIGH_SET((float)3.0962297E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F13_177(), &PH);
    p177_sue_week_no_SET((int16_t)(int16_t) -27066, PH.base.pack) ;
    p177_sue_lat_origin_SET((int32_t) -1599148286, PH.base.pack) ;
    p177_sue_lon_origin_SET((int32_t) -1266870843, PH.base.pack) ;
    p177_sue_alt_origin_SET((int32_t) -1063475413, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F14_178(), &PH);
    p178_sue_WIND_ESTIMATION_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p178_sue_GPS_TYPE_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p178_sue_DR_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p178_sue_BOARD_TYPE_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p178_sue_AIRFRAME_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p178_sue_RCON_SET((int16_t)(int16_t) -25859, PH.base.pack) ;
    p178_sue_TRAP_FLAGS_SET((int16_t)(int16_t)13691, PH.base.pack) ;
    p178_sue_TRAP_SOURCE_SET((uint32_t)655367780L, PH.base.pack) ;
    p178_sue_osc_fail_count_SET((int16_t)(int16_t)21224, PH.base.pack) ;
    p178_sue_CLOCK_CONFIG_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p178_sue_FLIGHT_PLAN_TYPE_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F15_179(), &PH);
    {
        uint8_t  sue_ID_VEHICLE_MODEL_NAME [] =  {(uint8_t)222, (uint8_t)119, (uint8_t)196, (uint8_t)223, (uint8_t)105, (uint8_t)8, (uint8_t)229, (uint8_t)81, (uint8_t)246, (uint8_t)7, (uint8_t)110, (uint8_t)11, (uint8_t)110, (uint8_t)195, (uint8_t)163, (uint8_t)101, (uint8_t)46, (uint8_t)188, (uint8_t)122, (uint8_t)144, (uint8_t)242, (uint8_t)6, (uint8_t)103, (uint8_t)254, (uint8_t)163, (uint8_t)137, (uint8_t)234, (uint8_t)169, (uint8_t)133, (uint8_t)113, (uint8_t)86, (uint8_t)248, (uint8_t)78, (uint8_t)44, (uint8_t)16, (uint8_t)145, (uint8_t)41, (uint8_t)95, (uint8_t)206, (uint8_t)193};
        p179_sue_ID_VEHICLE_MODEL_NAME_SET(&sue_ID_VEHICLE_MODEL_NAME, 0, &PH.base.pack) ;
    }
    {
        uint8_t  sue_ID_VEHICLE_REGISTRATION [] =  {(uint8_t)27, (uint8_t)160, (uint8_t)147, (uint8_t)107, (uint8_t)137, (uint8_t)209, (uint8_t)193, (uint8_t)162, (uint8_t)189, (uint8_t)79, (uint8_t)162, (uint8_t)5, (uint8_t)64, (uint8_t)144, (uint8_t)138, (uint8_t)138, (uint8_t)64, (uint8_t)241, (uint8_t)237, (uint8_t)119};
        p179_sue_ID_VEHICLE_REGISTRATION_SET(&sue_ID_VEHICLE_REGISTRATION, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F16_180(), &PH);
    {
        uint8_t  sue_ID_LEAD_PILOT [] =  {(uint8_t)68, (uint8_t)205, (uint8_t)23, (uint8_t)103, (uint8_t)166, (uint8_t)130, (uint8_t)209, (uint8_t)86, (uint8_t)34, (uint8_t)97, (uint8_t)132, (uint8_t)103, (uint8_t)100, (uint8_t)150, (uint8_t)183, (uint8_t)167, (uint8_t)56, (uint8_t)193, (uint8_t)107, (uint8_t)253, (uint8_t)140, (uint8_t)130, (uint8_t)176, (uint8_t)184, (uint8_t)45, (uint8_t)230, (uint8_t)10, (uint8_t)36, (uint8_t)176, (uint8_t)208, (uint8_t)69, (uint8_t)160, (uint8_t)22, (uint8_t)85, (uint8_t)32, (uint8_t)132, (uint8_t)16, (uint8_t)149, (uint8_t)221, (uint8_t)46};
        p180_sue_ID_LEAD_PILOT_SET(&sue_ID_LEAD_PILOT, 0, &PH.base.pack) ;
    }
    {
        uint8_t  sue_ID_DIY_DRONES_URL [] =  {(uint8_t)112, (uint8_t)99, (uint8_t)230, (uint8_t)51, (uint8_t)89, (uint8_t)255, (uint8_t)201, (uint8_t)120, (uint8_t)246, (uint8_t)171, (uint8_t)132, (uint8_t)104, (uint8_t)91, (uint8_t)201, (uint8_t)165, (uint8_t)162, (uint8_t)90, (uint8_t)21, (uint8_t)27, (uint8_t)201, (uint8_t)77, (uint8_t)79, (uint8_t)61, (uint8_t)233, (uint8_t)191, (uint8_t)198, (uint8_t)210, (uint8_t)87, (uint8_t)247, (uint8_t)223, (uint8_t)66, (uint8_t)163, (uint8_t)246, (uint8_t)107, (uint8_t)30, (uint8_t)197, (uint8_t)227, (uint8_t)236, (uint8_t)150, (uint8_t)239, (uint8_t)246, (uint8_t)63, (uint8_t)74, (uint8_t)185, (uint8_t)39, (uint8_t)239, (uint8_t)108, (uint8_t)188, (uint8_t)113, (uint8_t)178, (uint8_t)205, (uint8_t)49, (uint8_t)34, (uint8_t)189, (uint8_t)41, (uint8_t)123, (uint8_t)231, (uint8_t)157, (uint8_t)146, (uint8_t)15, (uint8_t)139, (uint8_t)0, (uint8_t)228, (uint8_t)20, (uint8_t)130, (uint8_t)234, (uint8_t)107, (uint8_t)251, (uint8_t)206, (uint8_t)202};
        p180_sue_ID_DIY_DRONES_URL_SET(&sue_ID_DIY_DRONES_URL, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDES_181(), &PH);
    p181_time_boot_ms_SET((uint32_t)654759534L, PH.base.pack) ;
    p181_alt_gps_SET((int32_t)1111964855, PH.base.pack) ;
    p181_alt_imu_SET((int32_t) -228238939, PH.base.pack) ;
    p181_alt_barometric_SET((int32_t)504620889, PH.base.pack) ;
    p181_alt_optical_flow_SET((int32_t)1511201351, PH.base.pack) ;
    p181_alt_range_finder_SET((int32_t)1492488692, PH.base.pack) ;
    p181_alt_extra_SET((int32_t)973958867, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AIRSPEEDS_182(), &PH);
    p182_time_boot_ms_SET((uint32_t)3769199941L, PH.base.pack) ;
    p182_airspeed_imu_SET((int16_t)(int16_t)10129, PH.base.pack) ;
    p182_airspeed_pitot_SET((int16_t)(int16_t)24603, PH.base.pack) ;
    p182_airspeed_hot_wire_SET((int16_t)(int16_t) -28135, PH.base.pack) ;
    p182_airspeed_ultrasonic_SET((int16_t)(int16_t)28734, PH.base.pack) ;
    p182_aoa_SET((int16_t)(int16_t)31105, PH.base.pack) ;
    p182_aoy_SET((int16_t)(int16_t) -31713, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F17_183(), &PH);
    p183_sue_feed_forward_SET((float) -3.190232E38F, PH.base.pack) ;
    p183_sue_turn_rate_nav_SET((float) -4.3575053E37F, PH.base.pack) ;
    p183_sue_turn_rate_fbw_SET((float) -3.3834763E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F18_184(), &PH);
    p184_angle_of_attack_normal_SET((float)1.625082E38F, PH.base.pack) ;
    p184_angle_of_attack_inverted_SET((float) -1.7825792E38F, PH.base.pack) ;
    p184_elevator_trim_normal_SET((float)6.8656464E36F, PH.base.pack) ;
    p184_elevator_trim_inverted_SET((float) -2.5741836E37F, PH.base.pack) ;
    p184_reference_speed_SET((float) -2.204162E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F19_185(), &PH);
    p185_sue_aileron_output_channel_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p185_sue_aileron_reversed_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p185_sue_elevator_output_channel_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p185_sue_elevator_reversed_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p185_sue_throttle_output_channel_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p185_sue_throttle_reversed_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p185_sue_rudder_output_channel_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p185_sue_rudder_reversed_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F20_186(), &PH);
    p186_sue_number_of_inputs_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p186_sue_trim_value_input_1_SET((int16_t)(int16_t)9204, PH.base.pack) ;
    p186_sue_trim_value_input_2_SET((int16_t)(int16_t) -11973, PH.base.pack) ;
    p186_sue_trim_value_input_3_SET((int16_t)(int16_t) -18116, PH.base.pack) ;
    p186_sue_trim_value_input_4_SET((int16_t)(int16_t) -16435, PH.base.pack) ;
    p186_sue_trim_value_input_5_SET((int16_t)(int16_t)6565, PH.base.pack) ;
    p186_sue_trim_value_input_6_SET((int16_t)(int16_t) -4565, PH.base.pack) ;
    p186_sue_trim_value_input_7_SET((int16_t)(int16_t) -11043, PH.base.pack) ;
    p186_sue_trim_value_input_8_SET((int16_t)(int16_t)10074, PH.base.pack) ;
    p186_sue_trim_value_input_9_SET((int16_t)(int16_t)5543, PH.base.pack) ;
    p186_sue_trim_value_input_10_SET((int16_t)(int16_t)31179, PH.base.pack) ;
    p186_sue_trim_value_input_11_SET((int16_t)(int16_t) -18921, PH.base.pack) ;
    p186_sue_trim_value_input_12_SET((int16_t)(int16_t)2342, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F21_187(), &PH);
    p187_sue_accel_x_offset_SET((int16_t)(int16_t) -19697, PH.base.pack) ;
    p187_sue_accel_y_offset_SET((int16_t)(int16_t)20295, PH.base.pack) ;
    p187_sue_accel_z_offset_SET((int16_t)(int16_t)7548, PH.base.pack) ;
    p187_sue_gyro_x_offset_SET((int16_t)(int16_t)12653, PH.base.pack) ;
    p187_sue_gyro_y_offset_SET((int16_t)(int16_t)16541, PH.base.pack) ;
    p187_sue_gyro_z_offset_SET((int16_t)(int16_t)10730, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F22_188(), &PH);
    p188_sue_accel_x_at_calibration_SET((int16_t)(int16_t)27860, PH.base.pack) ;
    p188_sue_accel_y_at_calibration_SET((int16_t)(int16_t)8262, PH.base.pack) ;
    p188_sue_accel_z_at_calibration_SET((int16_t)(int16_t)31094, PH.base.pack) ;
    p188_sue_gyro_x_at_calibration_SET((int16_t)(int16_t)16405, PH.base.pack) ;
    p188_sue_gyro_y_at_calibration_SET((int16_t)(int16_t) -5267, PH.base.pack) ;
    p188_sue_gyro_z_at_calibration_SET((int16_t)(int16_t)29846, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)2729222569055290263L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, PH.base.pack) ;
    p230_vel_ratio_SET((float) -3.2646455E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)4.296614E37F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -2.906958E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -1.7607724E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -2.962552E37F, PH.base.pack) ;
    p230_tas_ratio_SET((float)2.462106E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)2.4606346E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -9.756958E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)7041447471909567687L, PH.base.pack) ;
    p231_wind_x_SET((float)2.6574248E38F, PH.base.pack) ;
    p231_wind_y_SET((float)1.6714037E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -1.0714527E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -2.2390044E38F, PH.base.pack) ;
    p231_var_vert_SET((float) -2.6690288E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -3.6110617E37F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)1.3562212E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -2.0590397E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)583114680347980260L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)1294205390L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)141, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p232_lat_SET((int32_t)1183928987, PH.base.pack) ;
    p232_lon_SET((int32_t) -1786632269, PH.base.pack) ;
    p232_alt_SET((float) -2.0593354E37F, PH.base.pack) ;
    p232_hdop_SET((float) -2.217654E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.676965E38F, PH.base.pack) ;
    p232_vn_SET((float) -1.1077688E37F, PH.base.pack) ;
    p232_ve_SET((float) -2.3744834E38F, PH.base.pack) ;
    p232_vd_SET((float)3.0687773E37F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -2.8424497E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -2.2837324E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -3.3772182E37F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)168, (uint8_t)62, (uint8_t)228, (uint8_t)181, (uint8_t)202, (uint8_t)136, (uint8_t)205, (uint8_t)38, (uint8_t)57, (uint8_t)56, (uint8_t)134, (uint8_t)28, (uint8_t)217, (uint8_t)47, (uint8_t)177, (uint8_t)132, (uint8_t)39, (uint8_t)16, (uint8_t)123, (uint8_t)85, (uint8_t)121, (uint8_t)92, (uint8_t)205, (uint8_t)18, (uint8_t)41, (uint8_t)57, (uint8_t)152, (uint8_t)114, (uint8_t)60, (uint8_t)8, (uint8_t)185, (uint8_t)139, (uint8_t)240, (uint8_t)218, (uint8_t)41, (uint8_t)14, (uint8_t)82, (uint8_t)164, (uint8_t)119, (uint8_t)69, (uint8_t)118, (uint8_t)119, (uint8_t)217, (uint8_t)53, (uint8_t)132, (uint8_t)61, (uint8_t)223, (uint8_t)88, (uint8_t)47, (uint8_t)9, (uint8_t)39, (uint8_t)205, (uint8_t)104, (uint8_t)148, (uint8_t)207, (uint8_t)198, (uint8_t)231, (uint8_t)230, (uint8_t)215, (uint8_t)80, (uint8_t)40, (uint8_t)18, (uint8_t)236, (uint8_t)214, (uint8_t)108, (uint8_t)183, (uint8_t)59, (uint8_t)184, (uint8_t)21, (uint8_t)180, (uint8_t)144, (uint8_t)124, (uint8_t)8, (uint8_t)62, (uint8_t)202, (uint8_t)23, (uint8_t)87, (uint8_t)139, (uint8_t)22, (uint8_t)169, (uint8_t)5, (uint8_t)196, (uint8_t)57, (uint8_t)11, (uint8_t)53, (uint8_t)41, (uint8_t)56, (uint8_t)79, (uint8_t)82, (uint8_t)28, (uint8_t)72, (uint8_t)150, (uint8_t)234, (uint8_t)253, (uint8_t)224, (uint8_t)67, (uint8_t)95, (uint8_t)39, (uint8_t)89, (uint8_t)30, (uint8_t)72, (uint8_t)31, (uint8_t)222, (uint8_t)56, (uint8_t)104, (uint8_t)222, (uint8_t)15, (uint8_t)9, (uint8_t)188, (uint8_t)41, (uint8_t)81, (uint8_t)100, (uint8_t)5, (uint8_t)119, (uint8_t)136, (uint8_t)180, (uint8_t)186, (uint8_t)157, (uint8_t)222, (uint8_t)94, (uint8_t)164, (uint8_t)71, (uint8_t)186, (uint8_t)27, (uint8_t)132, (uint8_t)143, (uint8_t)143, (uint8_t)187, (uint8_t)107, (uint8_t)252, (uint8_t)187, (uint8_t)114, (uint8_t)14, (uint8_t)106, (uint8_t)12, (uint8_t)61, (uint8_t)4, (uint8_t)249, (uint8_t)54, (uint8_t)39, (uint8_t)88, (uint8_t)103, (uint8_t)116, (uint8_t)102, (uint8_t)27, (uint8_t)158, (uint8_t)18, (uint8_t)67, (uint8_t)133, (uint8_t)120, (uint8_t)163, (uint8_t)195, (uint8_t)7, (uint8_t)32, (uint8_t)42, (uint8_t)202, (uint8_t)133, (uint8_t)139, (uint8_t)161, (uint8_t)22, (uint8_t)204, (uint8_t)232, (uint8_t)224, (uint8_t)100, (uint8_t)149, (uint8_t)11, (uint8_t)118, (uint8_t)254, (uint8_t)156, (uint8_t)105, (uint8_t)185, (uint8_t)73, (uint8_t)32, (uint8_t)250, (uint8_t)198, (uint8_t)70, (uint8_t)160, (uint8_t)17, (uint8_t)198, (uint8_t)219};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1193679907L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)11342, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -23893, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)22625, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -117, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -21137, PH.base.pack) ;
    p234_latitude_SET((int32_t) -634471077, PH.base.pack) ;
    p234_longitude_SET((int32_t) -701263945, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)16709, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -18642, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t) -56, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)7, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -10, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)16316, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)1254360600956636139L, PH.base.pack) ;
    p241_vibration_x_SET((float) -1.9901104E38F, PH.base.pack) ;
    p241_vibration_y_SET((float) -2.4915472E38F, PH.base.pack) ;
    p241_vibration_z_SET((float) -2.2546442E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3749994795L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)2523536984L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)4119458647L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)1400326720, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1859734129, PH.base.pack) ;
    p242_altitude_SET((int32_t)2076801364, PH.base.pack) ;
    p242_x_SET((float)1.3137179E38F, PH.base.pack) ;
    p242_y_SET((float) -8.285351E36F, PH.base.pack) ;
    p242_z_SET((float)3.0526055E38F, PH.base.pack) ;
    {
        float  q [] =  {-5.9692825E37F, 1.2388832E38F, 3.3327277E38F, 1.0693282E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)1.7811398E38F, PH.base.pack) ;
    p242_approach_y_SET((float) -2.412761E36F, PH.base.pack) ;
    p242_approach_z_SET((float)2.5000423E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)3620252202907962388L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p243_latitude_SET((int32_t) -911442525, PH.base.pack) ;
    p243_longitude_SET((int32_t)1865922576, PH.base.pack) ;
    p243_altitude_SET((int32_t)1152923198, PH.base.pack) ;
    p243_x_SET((float)1.5552327E38F, PH.base.pack) ;
    p243_y_SET((float) -2.0552652E38F, PH.base.pack) ;
    p243_z_SET((float) -1.8063103E37F, PH.base.pack) ;
    {
        float  q [] =  {2.253098E36F, 1.2887757E37F, -3.1669129E38F, 1.1231728E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -7.4882235E37F, PH.base.pack) ;
    p243_approach_y_SET((float)3.0313053E38F, PH.base.pack) ;
    p243_approach_z_SET((float)1.4021485E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)5828178078782114317L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)3988, PH.base.pack) ;
    p244_interval_us_SET((int32_t)6493955, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)912441552L, PH.base.pack) ;
    p246_lat_SET((int32_t) -111499161, PH.base.pack) ;
    p246_lon_SET((int32_t)993574372, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
    p246_altitude_SET((int32_t) -2026254742, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)33789, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)61501, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)14615, PH.base.pack) ;
    {
        char16_t   callsign = "gYa";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)61352, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)1762743709L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)7.888122E37F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float)2.7081966E37F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)2.1934412E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)15372, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)215, (uint8_t)192, (uint8_t)197, (uint8_t)230, (uint8_t)77, (uint8_t)26, (uint8_t)15, (uint8_t)118, (uint8_t)22, (uint8_t)55, (uint8_t)117, (uint8_t)101, (uint8_t)79, (uint8_t)46, (uint8_t)158, (uint8_t)138, (uint8_t)253, (uint8_t)21, (uint8_t)1, (uint8_t)149, (uint8_t)152, (uint8_t)131, (uint8_t)101, (uint8_t)195, (uint8_t)3, (uint8_t)27, (uint8_t)255, (uint8_t)156, (uint8_t)9, (uint8_t)76, (uint8_t)131, (uint8_t)126, (uint8_t)13, (uint8_t)87, (uint8_t)161, (uint8_t)130, (uint8_t)65, (uint8_t)184, (uint8_t)244, (uint8_t)158, (uint8_t)216, (uint8_t)157, (uint8_t)106, (uint8_t)219, (uint8_t)167, (uint8_t)101, (uint8_t)92, (uint8_t)211, (uint8_t)199, (uint8_t)162, (uint8_t)32, (uint8_t)246, (uint8_t)87, (uint8_t)198, (uint8_t)251, (uint8_t)100, (uint8_t)143, (uint8_t)147, (uint8_t)50, (uint8_t)97, (uint8_t)105, (uint8_t)128, (uint8_t)176, (uint8_t)80, (uint8_t)43, (uint8_t)99, (uint8_t)87, (uint8_t)189, (uint8_t)120, (uint8_t)254, (uint8_t)70, (uint8_t)80, (uint8_t)227, (uint8_t)140, (uint8_t)209, (uint8_t)28, (uint8_t)84, (uint8_t)185, (uint8_t)15, (uint8_t)26, (uint8_t)172, (uint8_t)159, (uint8_t)158, (uint8_t)221, (uint8_t)191, (uint8_t)97, (uint8_t)62, (uint8_t)5, (uint8_t)15, (uint8_t)138, (uint8_t)41, (uint8_t)147, (uint8_t)179, (uint8_t)218, (uint8_t)39, (uint8_t)6, (uint8_t)6, (uint8_t)73, (uint8_t)188, (uint8_t)213, (uint8_t)201, (uint8_t)51, (uint8_t)177, (uint8_t)68, (uint8_t)237, (uint8_t)58, (uint8_t)100, (uint8_t)7, (uint8_t)157, (uint8_t)43, (uint8_t)255, (uint8_t)227, (uint8_t)237, (uint8_t)140, (uint8_t)11, (uint8_t)83, (uint8_t)45, (uint8_t)106, (uint8_t)76, (uint8_t)224, (uint8_t)27, (uint8_t)84, (uint8_t)33, (uint8_t)94, (uint8_t)209, (uint8_t)165, (uint8_t)74, (uint8_t)91, (uint8_t)5, (uint8_t)90, (uint8_t)173, (uint8_t)177, (uint8_t)178, (uint8_t)11, (uint8_t)107, (uint8_t)4, (uint8_t)235, (uint8_t)223, (uint8_t)1, (uint8_t)225, (uint8_t)48, (uint8_t)104, (uint8_t)236, (uint8_t)166, (uint8_t)5, (uint8_t)159, (uint8_t)38, (uint8_t)11, (uint8_t)8, (uint8_t)46, (uint8_t)26, (uint8_t)73, (uint8_t)84, (uint8_t)93, (uint8_t)73, (uint8_t)197, (uint8_t)106, (uint8_t)103, (uint8_t)88, (uint8_t)191, (uint8_t)115, (uint8_t)124, (uint8_t)195, (uint8_t)68, (uint8_t)208, (uint8_t)118, (uint8_t)185, (uint8_t)11, (uint8_t)91, (uint8_t)91, (uint8_t)94, (uint8_t)45, (uint8_t)194, (uint8_t)139, (uint8_t)251, (uint8_t)187, (uint8_t)247, (uint8_t)217, (uint8_t)62, (uint8_t)154, (uint8_t)67, (uint8_t)1, (uint8_t)136, (uint8_t)128, (uint8_t)159, (uint8_t)254, (uint8_t)57, (uint8_t)66, (uint8_t)230, (uint8_t)94, (uint8_t)76, (uint8_t)123, (uint8_t)109, (uint8_t)78, (uint8_t)88, (uint8_t)106, (uint8_t)251, (uint8_t)103, (uint8_t)65, (uint8_t)102, (uint8_t)167, (uint8_t)23, (uint8_t)64, (uint8_t)249, (uint8_t)89, (uint8_t)203, (uint8_t)138, (uint8_t)88, (uint8_t)36, (uint8_t)115, (uint8_t)198, (uint8_t)55, (uint8_t)206, (uint8_t)158, (uint8_t)209, (uint8_t)74, (uint8_t)67, (uint8_t)255, (uint8_t)108, (uint8_t)197, (uint8_t)101, (uint8_t)218, (uint8_t)44, (uint8_t)65, (uint8_t)71, (uint8_t)129, (uint8_t)127, (uint8_t)108, (uint8_t)90, (uint8_t)254, (uint8_t)250, (uint8_t)91, (uint8_t)241, (uint8_t)103, (uint8_t)229, (uint8_t)89, (uint8_t)139, (uint8_t)165, (uint8_t)131, (uint8_t)30, (uint8_t)177, (uint8_t)197, (uint8_t)209, (uint8_t)245, (uint8_t)52, (uint8_t)151, (uint8_t)109, (uint8_t)189, (uint8_t)134};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)56448, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)106, (int8_t) -13, (int8_t)43, (int8_t) -76, (int8_t) -121, (int8_t) -48, (int8_t) -6, (int8_t)45, (int8_t) -84, (int8_t) -110, (int8_t) -121, (int8_t)93, (int8_t) -81, (int8_t)36, (int8_t)77, (int8_t)110, (int8_t)72, (int8_t)105, (int8_t) -108, (int8_t)58, (int8_t)98, (int8_t) -105, (int8_t) -88, (int8_t)73, (int8_t) -117, (int8_t)52, (int8_t) -106, (int8_t) -55, (int8_t)62, (int8_t) -38, (int8_t) -9, (int8_t) -44};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "sshpNm";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)1135746268215463386L, PH.base.pack) ;
    p250_x_SET((float)1.2289761E37F, PH.base.pack) ;
    p250_y_SET((float)1.7953141E38F, PH.base.pack) ;
    p250_z_SET((float) -1.4954167E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)2794917486L, PH.base.pack) ;
    {
        char16_t   name = "GmlCf";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)3.165584E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)4236203528L, PH.base.pack) ;
    {
        char16_t   name = "fnuxerli";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -1476715483, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
    {
        char16_t   text = "pvdglyexzmcgnxhenlWizwez";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)1671356822L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p254_value_SET((float)2.5543504E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)200, (uint8_t)81, (uint8_t)49, (uint8_t)122, (uint8_t)0, (uint8_t)154, (uint8_t)184, (uint8_t)247, (uint8_t)126, (uint8_t)193, (uint8_t)122, (uint8_t)252, (uint8_t)94, (uint8_t)122, (uint8_t)74, (uint8_t)97, (uint8_t)65, (uint8_t)170, (uint8_t)169, (uint8_t)187, (uint8_t)149, (uint8_t)250, (uint8_t)233, (uint8_t)221, (uint8_t)216, (uint8_t)109, (uint8_t)54, (uint8_t)152, (uint8_t)185, (uint8_t)57, (uint8_t)237, (uint8_t)234};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)3390688475361934039L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)407316645L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)2471199323L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    {
        char16_t   tune = "Mbnolqvyijkgdsqn";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)927203292L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)159, (uint8_t)4, (uint8_t)128, (uint8_t)4, (uint8_t)105, (uint8_t)81, (uint8_t)130, (uint8_t)15, (uint8_t)13, (uint8_t)218, (uint8_t)19, (uint8_t)153, (uint8_t)217, (uint8_t)109, (uint8_t)129, (uint8_t)46, (uint8_t)232, (uint8_t)214, (uint8_t)78, (uint8_t)25, (uint8_t)215, (uint8_t)97, (uint8_t)128, (uint8_t)58, (uint8_t)226, (uint8_t)83, (uint8_t)235, (uint8_t)200, (uint8_t)62, (uint8_t)200, (uint8_t)191, (uint8_t)235};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)223, (uint8_t)112, (uint8_t)144, (uint8_t)189, (uint8_t)72, (uint8_t)108, (uint8_t)172, (uint8_t)250, (uint8_t)16, (uint8_t)83, (uint8_t)98, (uint8_t)131, (uint8_t)120, (uint8_t)198, (uint8_t)52, (uint8_t)94, (uint8_t)178, (uint8_t)178, (uint8_t)44, (uint8_t)40, (uint8_t)8, (uint8_t)251, (uint8_t)214, (uint8_t)50, (uint8_t)39, (uint8_t)21, (uint8_t)160, (uint8_t)176, (uint8_t)222, (uint8_t)240, (uint8_t)131, (uint8_t)232};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)3956727089L, PH.base.pack) ;
    p259_focal_length_SET((float)1.2309926E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -3.3425375E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -1.045357E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)32138, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)5262, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)48888, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "hzchmfCqqeidaTamnwwqwxvjorsbonpxXhjhckuomesryghuzyymdmkjyTdmnghxsOcYurpvrftuVuztuavqaAjdbyjtbaalxsxnzal";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)2203670411L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)2921268511L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p261_total_capacity_SET((float) -3.2153899E38F, PH.base.pack) ;
    p261_used_capacity_SET((float) -2.6064894E38F, PH.base.pack) ;
    p261_available_capacity_SET((float)1.0158604E38F, PH.base.pack) ;
    p261_read_speed_SET((float)2.8141768E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -3.3945961E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)1506149871L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p262_image_interval_SET((float)2.802753E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)306904208L, PH.base.pack) ;
    p262_available_capacity_SET((float)1.0561719E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)1780906528L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)2068491367597352420L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p263_lat_SET((int32_t)1320738156, PH.base.pack) ;
    p263_lon_SET((int32_t) -1099558934, PH.base.pack) ;
    p263_alt_SET((int32_t)1472033433, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)902056958, PH.base.pack) ;
    {
        float  q [] =  {-3.3472436E38F, -1.3268793E37F, -1.3171777E38F, -1.5657078E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1940353651, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)28, PH.base.pack) ;
    {
        char16_t   file_url = "rxvkgvdmIdxywutovokcvwhpfsefkoKmimgfsoxsugtssmrmayfoxhmeUSjtwlqudpbrexdvkhSfuvxpnupgympujQUktdiqedwhkxwTgWjmwjxlfRdzievrPlhokvkaevtrdxasxijeehenzbRdshgqnnvkymciqhekfzbvaCljnbncUIdi";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)32369520L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)1212986241398460611L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)4508121246428229488L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)2828224293217936088L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)3513481953L, PH.base.pack) ;
    p265_roll_SET((float) -1.4267359E38F, PH.base.pack) ;
    p265_pitch_SET((float)3.3511374E38F, PH.base.pack) ;
    p265_yaw_SET((float) -3.364198E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)31904, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)191, (uint8_t)78, (uint8_t)42, (uint8_t)47, (uint8_t)244, (uint8_t)222, (uint8_t)194, (uint8_t)237, (uint8_t)224, (uint8_t)11, (uint8_t)112, (uint8_t)0, (uint8_t)78, (uint8_t)186, (uint8_t)223, (uint8_t)142, (uint8_t)1, (uint8_t)160, (uint8_t)104, (uint8_t)21, (uint8_t)178, (uint8_t)236, (uint8_t)238, (uint8_t)46, (uint8_t)86, (uint8_t)87, (uint8_t)178, (uint8_t)37, (uint8_t)192, (uint8_t)135, (uint8_t)211, (uint8_t)29, (uint8_t)41, (uint8_t)40, (uint8_t)56, (uint8_t)241, (uint8_t)226, (uint8_t)214, (uint8_t)211, (uint8_t)241, (uint8_t)1, (uint8_t)53, (uint8_t)150, (uint8_t)46, (uint8_t)225, (uint8_t)119, (uint8_t)247, (uint8_t)66, (uint8_t)36, (uint8_t)202, (uint8_t)54, (uint8_t)122, (uint8_t)173, (uint8_t)94, (uint8_t)243, (uint8_t)20, (uint8_t)128, (uint8_t)59, (uint8_t)189, (uint8_t)202, (uint8_t)217, (uint8_t)159, (uint8_t)225, (uint8_t)167, (uint8_t)107, (uint8_t)13, (uint8_t)162, (uint8_t)177, (uint8_t)210, (uint8_t)95, (uint8_t)198, (uint8_t)255, (uint8_t)178, (uint8_t)120, (uint8_t)162, (uint8_t)65, (uint8_t)34, (uint8_t)188, (uint8_t)214, (uint8_t)101, (uint8_t)222, (uint8_t)170, (uint8_t)112, (uint8_t)224, (uint8_t)224, (uint8_t)73, (uint8_t)232, (uint8_t)226, (uint8_t)181, (uint8_t)229, (uint8_t)173, (uint8_t)58, (uint8_t)175, (uint8_t)154, (uint8_t)227, (uint8_t)169, (uint8_t)86, (uint8_t)21, (uint8_t)150, (uint8_t)65, (uint8_t)29, (uint8_t)50, (uint8_t)191, (uint8_t)175, (uint8_t)50, (uint8_t)109, (uint8_t)0, (uint8_t)150, (uint8_t)226, (uint8_t)29, (uint8_t)10, (uint8_t)150, (uint8_t)73, (uint8_t)9, (uint8_t)154, (uint8_t)139, (uint8_t)22, (uint8_t)86, (uint8_t)242, (uint8_t)5, (uint8_t)229, (uint8_t)163, (uint8_t)224, (uint8_t)42, (uint8_t)204, (uint8_t)54, (uint8_t)52, (uint8_t)125, (uint8_t)81, (uint8_t)5, (uint8_t)159, (uint8_t)113, (uint8_t)78, (uint8_t)66, (uint8_t)246, (uint8_t)30, (uint8_t)193, (uint8_t)196, (uint8_t)53, (uint8_t)4, (uint8_t)87, (uint8_t)237, (uint8_t)74, (uint8_t)81, (uint8_t)62, (uint8_t)122, (uint8_t)140, (uint8_t)153, (uint8_t)99, (uint8_t)247, (uint8_t)113, (uint8_t)117, (uint8_t)36, (uint8_t)33, (uint8_t)245, (uint8_t)93, (uint8_t)134, (uint8_t)161, (uint8_t)15, (uint8_t)200, (uint8_t)158, (uint8_t)238, (uint8_t)235, (uint8_t)1, (uint8_t)40, (uint8_t)188, (uint8_t)174, (uint8_t)185, (uint8_t)132, (uint8_t)127, (uint8_t)196, (uint8_t)126, (uint8_t)214, (uint8_t)227, (uint8_t)87, (uint8_t)170, (uint8_t)187, (uint8_t)179, (uint8_t)160, (uint8_t)47, (uint8_t)53, (uint8_t)247, (uint8_t)238, (uint8_t)120, (uint8_t)30, (uint8_t)200, (uint8_t)224, (uint8_t)209, (uint8_t)120, (uint8_t)135, (uint8_t)74, (uint8_t)231, (uint8_t)207, (uint8_t)145, (uint8_t)3, (uint8_t)115, (uint8_t)240, (uint8_t)3, (uint8_t)20, (uint8_t)210, (uint8_t)29, (uint8_t)190, (uint8_t)239, (uint8_t)44, (uint8_t)253, (uint8_t)7, (uint8_t)50, (uint8_t)108, (uint8_t)86, (uint8_t)141, (uint8_t)89, (uint8_t)208, (uint8_t)33, (uint8_t)198, (uint8_t)97, (uint8_t)21, (uint8_t)14, (uint8_t)176, (uint8_t)59, (uint8_t)68, (uint8_t)60, (uint8_t)23, (uint8_t)152, (uint8_t)104, (uint8_t)59, (uint8_t)170, (uint8_t)69, (uint8_t)0, (uint8_t)217, (uint8_t)180, (uint8_t)156, (uint8_t)60, (uint8_t)170, (uint8_t)222, (uint8_t)61, (uint8_t)29, (uint8_t)77, (uint8_t)205, (uint8_t)129, (uint8_t)124, (uint8_t)91, (uint8_t)61, (uint8_t)236, (uint8_t)21, (uint8_t)178, (uint8_t)132, (uint8_t)47, (uint8_t)251, (uint8_t)126};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)29522, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)166, (uint8_t)255, (uint8_t)33, (uint8_t)89, (uint8_t)169, (uint8_t)235, (uint8_t)20, (uint8_t)12, (uint8_t)19, (uint8_t)93, (uint8_t)177, (uint8_t)150, (uint8_t)126, (uint8_t)199, (uint8_t)11, (uint8_t)28, (uint8_t)223, (uint8_t)153, (uint8_t)69, (uint8_t)173, (uint8_t)167, (uint8_t)227, (uint8_t)249, (uint8_t)110, (uint8_t)243, (uint8_t)181, (uint8_t)60, (uint8_t)62, (uint8_t)2, (uint8_t)73, (uint8_t)60, (uint8_t)65, (uint8_t)227, (uint8_t)210, (uint8_t)126, (uint8_t)200, (uint8_t)252, (uint8_t)250, (uint8_t)118, (uint8_t)199, (uint8_t)166, (uint8_t)32, (uint8_t)4, (uint8_t)98, (uint8_t)80, (uint8_t)93, (uint8_t)63, (uint8_t)187, (uint8_t)183, (uint8_t)252, (uint8_t)255, (uint8_t)129, (uint8_t)171, (uint8_t)88, (uint8_t)215, (uint8_t)142, (uint8_t)248, (uint8_t)14, (uint8_t)235, (uint8_t)9, (uint8_t)11, (uint8_t)196, (uint8_t)100, (uint8_t)166, (uint8_t)141, (uint8_t)198, (uint8_t)249, (uint8_t)56, (uint8_t)136, (uint8_t)31, (uint8_t)127, (uint8_t)161, (uint8_t)37, (uint8_t)255, (uint8_t)50, (uint8_t)238, (uint8_t)158, (uint8_t)246, (uint8_t)6, (uint8_t)194, (uint8_t)126, (uint8_t)108, (uint8_t)40, (uint8_t)99, (uint8_t)97, (uint8_t)136, (uint8_t)175, (uint8_t)175, (uint8_t)110, (uint8_t)3, (uint8_t)154, (uint8_t)154, (uint8_t)254, (uint8_t)96, (uint8_t)130, (uint8_t)96, (uint8_t)84, (uint8_t)31, (uint8_t)43, (uint8_t)198, (uint8_t)133, (uint8_t)128, (uint8_t)223, (uint8_t)54, (uint8_t)149, (uint8_t)242, (uint8_t)152, (uint8_t)251, (uint8_t)120, (uint8_t)75, (uint8_t)187, (uint8_t)93, (uint8_t)123, (uint8_t)248, (uint8_t)189, (uint8_t)131, (uint8_t)15, (uint8_t)224, (uint8_t)29, (uint8_t)154, (uint8_t)94, (uint8_t)4, (uint8_t)26, (uint8_t)176, (uint8_t)15, (uint8_t)206, (uint8_t)187, (uint8_t)154, (uint8_t)10, (uint8_t)35, (uint8_t)19, (uint8_t)111, (uint8_t)144, (uint8_t)1, (uint8_t)100, (uint8_t)60, (uint8_t)46, (uint8_t)186, (uint8_t)216, (uint8_t)196, (uint8_t)250, (uint8_t)187, (uint8_t)101, (uint8_t)195, (uint8_t)206, (uint8_t)31, (uint8_t)46, (uint8_t)231, (uint8_t)120, (uint8_t)143, (uint8_t)245, (uint8_t)178, (uint8_t)61, (uint8_t)249, (uint8_t)247, (uint8_t)207, (uint8_t)155, (uint8_t)207, (uint8_t)117, (uint8_t)130, (uint8_t)235, (uint8_t)108, (uint8_t)8, (uint8_t)0, (uint8_t)235, (uint8_t)91, (uint8_t)226, (uint8_t)255, (uint8_t)185, (uint8_t)243, (uint8_t)201, (uint8_t)150, (uint8_t)255, (uint8_t)62, (uint8_t)36, (uint8_t)165, (uint8_t)238, (uint8_t)250, (uint8_t)113, (uint8_t)90, (uint8_t)59, (uint8_t)162, (uint8_t)221, (uint8_t)103, (uint8_t)233, (uint8_t)4, (uint8_t)107, (uint8_t)200, (uint8_t)150, (uint8_t)2, (uint8_t)63, (uint8_t)52, (uint8_t)176, (uint8_t)223, (uint8_t)79, (uint8_t)128, (uint8_t)145, (uint8_t)136, (uint8_t)233, (uint8_t)56, (uint8_t)200, (uint8_t)231, (uint8_t)245, (uint8_t)69, (uint8_t)185, (uint8_t)219, (uint8_t)36, (uint8_t)115, (uint8_t)161, (uint8_t)133, (uint8_t)139, (uint8_t)137, (uint8_t)25, (uint8_t)13, (uint8_t)230, (uint8_t)216, (uint8_t)60, (uint8_t)3, (uint8_t)72, (uint8_t)214, (uint8_t)27, (uint8_t)210, (uint8_t)213, (uint8_t)145, (uint8_t)171, (uint8_t)37, (uint8_t)127, (uint8_t)229, (uint8_t)132, (uint8_t)93, (uint8_t)222, (uint8_t)50, (uint8_t)227, (uint8_t)232, (uint8_t)31, (uint8_t)227, (uint8_t)219, (uint8_t)162, (uint8_t)99, (uint8_t)181, (uint8_t)165, (uint8_t)240, (uint8_t)150, (uint8_t)239, (uint8_t)21, (uint8_t)178, (uint8_t)202, (uint8_t)165, (uint8_t)201};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)49607, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p269_framerate_SET((float)2.1539063E38F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)36062, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)43590, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)3351360857L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)7700, PH.base.pack) ;
    {
        char16_t   uri = "pzSUeDeijtBtgabxzpxsyeoieylbilkqohpiyqkItyqxyfazTrlupxbcHaVoulmmzdglprbnolwvslcjutikzxEjighyvsaomxzgkacuVkbzhXveohznpvrqcAjvxExtirHvfwnqjeaockbmiawewcxcaecciql";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p270_framerate_SET((float)1.4328091E37F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)14075, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)32078, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)1982632651L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)51187, PH.base.pack) ;
    {
        char16_t   uri = "qvuENgmypfumEazmwrcYsxhxlkqufxtlDdqmbtpcicGuiwhgseccCQljklFgqcyirxZgsRkBjfttiasglXylosdHStknvupftssYsyHmne";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "rtjVovvNxhVa";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "szkmtdqppqiyldiOfeocxXRjpwoqxymhfsnq";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)59122, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)8396, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)43221, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)202, (uint8_t)222, (uint8_t)50, (uint8_t)247, (uint8_t)252, (uint8_t)105, (uint8_t)4, (uint8_t)151};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)173, (uint8_t)95, (uint8_t)18, (uint8_t)214, (uint8_t)33, (uint8_t)141, (uint8_t)63, (uint8_t)197};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)6064275665474962780L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)811131008L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)29201, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)5645573230558983195L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)3073954921L, PH.base.pack) ;
    {
        char16_t   name = "uysutetdcmCixfvbTmkiAvyqvnpaeaDNpwiadqYdWddDjybmtplkpbwXocwntc";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)182, (uint8_t)235, (uint8_t)86, (uint8_t)173, (uint8_t)95, (uint8_t)42, (uint8_t)255, (uint8_t)63, (uint8_t)194, (uint8_t)56, (uint8_t)202, (uint8_t)142, (uint8_t)174, (uint8_t)165, (uint8_t)110, (uint8_t)194};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)3044081538L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    {
        char16_t   param_id = "buOtudFbkpcguux";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t) -15827, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "dzxyaBd";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "oZqcvntfpkialmluSozzyDmlOrohnfjvpasfjuSmveoezyvuixuoqcazbilfievkbccnpwzrndqawvjtrKQxzzjndvnrbprytyOhLdfautjzlsiOubpeusudjE";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)12834, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)65412, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    {
        char16_t   param_id = "yspbuivofuyywtb";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "ugflrtinmnxhshghahBsqzvuflKauygquyxqkrhrEevgtvsbectiNwisdyWxwelodyjxvpsaebhlnqcmmkexuofskmypvylhsrgbnrszpgkwtcbEvgqfbylrwrp";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "uefehkmfgivejsm";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "agpkIdyeaipantiazpeylvgappBubvbzjdevpIcpdsipx";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)1043146996809324371L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)33669, (uint16_t)24587, (uint16_t)8151, (uint16_t)20387, (uint16_t)51390, (uint16_t)10530, (uint16_t)1345, (uint16_t)29469, (uint16_t)49492, (uint16_t)46183, (uint16_t)51375, (uint16_t)34322, (uint16_t)6149, (uint16_t)46515, (uint16_t)9135, (uint16_t)34064, (uint16_t)58004, (uint16_t)23787, (uint16_t)10604, (uint16_t)6522, (uint16_t)15626, (uint16_t)22433, (uint16_t)50573, (uint16_t)47137, (uint16_t)9245, (uint16_t)1786, (uint16_t)20605, (uint16_t)51596, (uint16_t)14388, (uint16_t)26093, (uint16_t)27846, (uint16_t)26422, (uint16_t)11711, (uint16_t)4926, (uint16_t)4168, (uint16_t)58837, (uint16_t)9148, (uint16_t)48759, (uint16_t)30252, (uint16_t)42318, (uint16_t)47164, (uint16_t)32710, (uint16_t)7606, (uint16_t)41863, (uint16_t)54870, (uint16_t)8423, (uint16_t)12540, (uint16_t)37977, (uint16_t)50549, (uint16_t)2533, (uint16_t)9649, (uint16_t)16982, (uint16_t)53236, (uint16_t)29912, (uint16_t)8276, (uint16_t)40228, (uint16_t)42798, (uint16_t)36016, (uint16_t)59789, (uint16_t)41164, (uint16_t)12996, (uint16_t)24231, (uint16_t)59388, (uint16_t)38487, (uint16_t)57702, (uint16_t)14638, (uint16_t)4578, (uint16_t)34401, (uint16_t)16186, (uint16_t)63557, (uint16_t)29954, (uint16_t)41375};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)2651, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)14830, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}


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
void c_LoopBackDemoChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sensLoad = p170_sensLoad_GET(pack);
    uint8_t  ctrlLoad = p170_ctrlLoad_GET(pack);
    uint16_t  batVolt = p170_batVolt_GET(pack);
}
void c_LoopBackDemoChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    float  axBias = p172_axBias_GET(pack);
    float  ayBias = p172_ayBias_GET(pack);
    float  azBias = p172_azBias_GET(pack);
    float  gxBias = p172_gxBias_GET(pack);
    float  gyBias = p172_gyBias_GET(pack);
    float  gzBias = p172_gzBias_GET(pack);
}
void c_LoopBackDemoChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    float  diagFl1 = p173_diagFl1_GET(pack);
    float  diagFl2 = p173_diagFl2_GET(pack);
    float  diagFl3 = p173_diagFl3_GET(pack);
    int16_t  diagSh1 = p173_diagSh1_GET(pack);
    int16_t  diagSh2 = p173_diagSh2_GET(pack);
    int16_t  diagSh3 = p173_diagSh3_GET(pack);
}
void c_LoopBackDemoChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    float  u_m = p176_u_m_GET(pack);
    float  phi_c = p176_phi_c_GET(pack);
    float  theta_c = p176_theta_c_GET(pack);
    float  psiDot_c = p176_psiDot_c_GET(pack);
    float  ay_body = p176_ay_body_GET(pack);
    float  totalDist = p176_totalDist_GET(pack);
    float  dist2Go = p176_dist2Go_GET(pack);
    uint8_t  fromWP = p176_fromWP_GET(pack);
    uint8_t  toWP = p176_toWP_GET(pack);
    uint16_t  h_c = p176_h_c_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    float  fl_1 = p177_fl_1_GET(pack);
    float  fl_2 = p177_fl_2_GET(pack);
    float  fl_3 = p177_fl_3_GET(pack);
    float  fl_4 = p177_fl_4_GET(pack);
    float  fl_5 = p177_fl_5_GET(pack);
    float  fl_6 = p177_fl_6_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  year = p179_year_GET(pack);
    uint8_t  month = p179_month_GET(pack);
    uint8_t  day = p179_day_GET(pack);
    uint8_t  hour = p179_hour_GET(pack);
    uint8_t  min = p179_min_GET(pack);
    uint8_t  sec = p179_sec_GET(pack);
    uint8_t  clockStat = p179_clockStat_GET(pack);
    uint8_t  visSat = p179_visSat_GET(pack);
    uint8_t  useSat = p179_useSat_GET(pack);
    uint8_t  GppGl = p179_GppGl_GET(pack);
    uint8_t  sigUsedMask = p179_sigUsedMask_GET(pack);
    uint8_t  percentUsed = p179_percentUsed_GET(pack);
}
void c_LoopBackDemoChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p180_target_GET(pack);
    float  hCommand = p180_hCommand_GET(pack);
    float  uCommand = p180_uCommand_GET(pack);
    float  rCommand = p180_rCommand_GET(pack);
}
void c_LoopBackDemoChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p181_target_GET(pack);
    uint16_t  bitfieldPt = p181_bitfieldPt_GET(pack);
}
void c_LoopBackDemoChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p184_target_GET(pack);
    int8_t  pan = p184_pan_GET(pack);
    int8_t  tilt = p184_tilt_GET(pack);
    int8_t  zoom = p184_zoom_GET(pack);
    int8_t  moveHome = p184_moveHome_GET(pack);
}
void c_LoopBackDemoChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p185_target_GET(pack);
    uint8_t  idSurface = p185_idSurface_GET(pack);
    float  mControl = p185_mControl_GET(pack);
    float  bControl = p185_bControl_GET(pack);
}
void c_LoopBackDemoChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p186_target_GET(pack);
    float  latitude = p186_latitude_GET(pack);
    float  longitude = p186_longitude_GET(pack);
}
void c_LoopBackDemoChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p188_target_GET(pack);
    uint8_t  idOrder = p188_idOrder_GET(pack);
    uint8_t  order = p188_order_GET(pack);
}
void c_LoopBackDemoChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p189_target_GET(pack);
    float  latitude = p189_latitude_GET(pack);
    float  longitude = p189_longitude_GET(pack);
    float  height = p189_height_GET(pack);
    uint8_t  option1 = p189_option1_GET(pack);
    uint8_t  option2 = p189_option2_GET(pack);
    uint8_t  option3 = p189_option3_GET(pack);
}
void c_LoopBackDemoChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  r2Type = p191_r2Type_GET(pack);
    uint16_t  voltage = p191_voltage_GET(pack);
    uint16_t  reading2 = p191_reading2_GET(pack);
}
void c_LoopBackDemoChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  zoom = p192_zoom_GET(pack);
    int16_t  pan = p192_pan_GET(pack);
    int16_t  tilt = p192_tilt_GET(pack);
}
void c_LoopBackDemoChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p193_target_GET(pack);
    float  latitude = p193_latitude_GET(pack);
    float  longitude = p193_longitude_GET(pack);
    float  altitude = p193_altitude_GET(pack);
    float  speed = p193_speed_GET(pack);
    float  course = p193_course_GET(pack);
}
void c_LoopBackDemoChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  csFails = p194_csFails_GET(pack);
    uint8_t  gpsQuality = p194_gpsQuality_GET(pack);
    uint8_t  msgsType = p194_msgsType_GET(pack);
    uint8_t  posStatus = p194_posStatus_GET(pack);
    float  magVar = p194_magVar_GET(pack);
    int8_t  magDir = p194_magDir_GET(pack);
    uint8_t  modeInd = p194_modeInd_GET(pack);
}
void c_LoopBackDemoChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  timeStatus = p195_timeStatus_GET(pack);
    uint32_t  receiverStatus = p195_receiverStatus_GET(pack);
    uint8_t  solStatus = p195_solStatus_GET(pack);
    uint8_t  posType = p195_posType_GET(pack);
    uint8_t  velType = p195_velType_GET(pack);
    float  posSolAge = p195_posSolAge_GET(pack);
    uint16_t  csFails = p195_csFails_GET(pack);
}
void c_LoopBackDemoChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    float  float1 = p196_float1_GET(pack);
    float  float2 = p196_float2_GET(pack);
    int16_t  int1 = p196_int1_GET(pack);
    int8_t  char1 = p196_char1_GET(pack);
}
void c_LoopBackDemoChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  version = p197_version_GET(pack);
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_GIMBAL, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2517504102L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)12325, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)37437, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)15325, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)118, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)7681, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)20117, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)31361, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)43066, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)30878, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)40408, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)2223470342185529590L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)1703688642L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)128195161L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)64150, PH.base.pack) ;
    p3_x_SET((float)2.2152089E38F, PH.base.pack) ;
    p3_y_SET((float)2.3138445E38F, PH.base.pack) ;
    p3_z_SET((float)3.3018298E38F, PH.base.pack) ;
    p3_vx_SET((float)2.6863932E38F, PH.base.pack) ;
    p3_vy_SET((float) -4.684877E37F, PH.base.pack) ;
    p3_vz_SET((float) -1.3444384E38F, PH.base.pack) ;
    p3_afx_SET((float)2.3285235E38F, PH.base.pack) ;
    p3_afy_SET((float) -2.1128405E38F, PH.base.pack) ;
    p3_afz_SET((float)2.1264877E38F, PH.base.pack) ;
    p3_yaw_SET((float) -2.7043413E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -2.5949534E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)4731576345188888062L, PH.base.pack) ;
    p4_seq_SET((uint32_t)1593449866L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    {
        char16_t   passkey = "ianvsgh";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "ateaGlmZfukezkx";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3638454770L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    {
        char16_t   param_id = "VkhrupklkaoEjkof";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)1299, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "wduomyxc";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -8.3647715E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)548, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)65214, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    {
        char16_t   param_id = "zQog";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)1.6197988E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5508049524517604487L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -1402452394, PH.base.pack) ;
    p24_lon_SET((int32_t) -1046274458, PH.base.pack) ;
    p24_alt_SET((int32_t)1016887389, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)46688, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)25793, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)60389, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)53476, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1961802643, &PH) ;
    p24_h_acc_SET((uint32_t)3329999377L, &PH) ;
    p24_v_acc_SET((uint32_t)1138723207L, &PH) ;
    p24_vel_acc_SET((uint32_t)1880507362L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1119306710L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)64, (uint8_t)149, (uint8_t)151, (uint8_t)127, (uint8_t)142, (uint8_t)87, (uint8_t)248, (uint8_t)177, (uint8_t)19, (uint8_t)40, (uint8_t)163, (uint8_t)38, (uint8_t)70, (uint8_t)202, (uint8_t)232, (uint8_t)113, (uint8_t)96, (uint8_t)61, (uint8_t)195, (uint8_t)15};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)2, (uint8_t)71, (uint8_t)67, (uint8_t)14, (uint8_t)26, (uint8_t)166, (uint8_t)178, (uint8_t)193, (uint8_t)93, (uint8_t)89, (uint8_t)90, (uint8_t)222, (uint8_t)126, (uint8_t)116, (uint8_t)201, (uint8_t)10, (uint8_t)125, (uint8_t)149, (uint8_t)55, (uint8_t)166};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)148, (uint8_t)127, (uint8_t)151, (uint8_t)131, (uint8_t)49, (uint8_t)227, (uint8_t)73, (uint8_t)216, (uint8_t)192, (uint8_t)52, (uint8_t)39, (uint8_t)36, (uint8_t)186, (uint8_t)53, (uint8_t)66, (uint8_t)5, (uint8_t)56, (uint8_t)31, (uint8_t)105, (uint8_t)39};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)77, (uint8_t)153, (uint8_t)175, (uint8_t)74, (uint8_t)169, (uint8_t)34, (uint8_t)247, (uint8_t)77, (uint8_t)69, (uint8_t)58, (uint8_t)5, (uint8_t)203, (uint8_t)177, (uint8_t)29, (uint8_t)176, (uint8_t)126, (uint8_t)101, (uint8_t)146, (uint8_t)37, (uint8_t)12};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)66, (uint8_t)133, (uint8_t)24, (uint8_t)140, (uint8_t)26, (uint8_t)12, (uint8_t)46, (uint8_t)173, (uint8_t)149, (uint8_t)244, (uint8_t)183, (uint8_t)99, (uint8_t)186, (uint8_t)71, (uint8_t)225, (uint8_t)222, (uint8_t)117, (uint8_t)39, (uint8_t)199, (uint8_t)134};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)1557417224L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -23476, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)8279, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -12845, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -3323, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)26353, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)29789, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -12346, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -7436, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -4180, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)41989589506010641L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)14935, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)16462, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -23847, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -20656, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -24541, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)6646, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)31933, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -26740, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)16056, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)2940458431040942102L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)24329, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -18583, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)3901, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)414, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)795925947L, PH.base.pack) ;
    p29_press_abs_SET((float) -2.9940992E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -8.882408E37F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -27012, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2065431874L, PH.base.pack) ;
    p30_roll_SET((float)1.4091378E37F, PH.base.pack) ;
    p30_pitch_SET((float)1.4260598E38F, PH.base.pack) ;
    p30_yaw_SET((float)3.2931544E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -2.553471E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.5055183E38F, PH.base.pack) ;
    p30_yawspeed_SET((float)2.5420143E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)294188348L, PH.base.pack) ;
    p31_q1_SET((float) -1.385519E38F, PH.base.pack) ;
    p31_q2_SET((float)2.2433114E38F, PH.base.pack) ;
    p31_q3_SET((float)2.508419E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.8047514E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)1.6022343E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)2.541062E38F, PH.base.pack) ;
    p31_yawspeed_SET((float) -2.6869702E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)2820832231L, PH.base.pack) ;
    p32_x_SET((float)1.5332636E38F, PH.base.pack) ;
    p32_y_SET((float) -2.4461474E38F, PH.base.pack) ;
    p32_z_SET((float)2.429306E38F, PH.base.pack) ;
    p32_vx_SET((float) -3.2285371E38F, PH.base.pack) ;
    p32_vy_SET((float) -2.2877459E38F, PH.base.pack) ;
    p32_vz_SET((float)1.7374635E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)4041782160L, PH.base.pack) ;
    p33_lat_SET((int32_t)250475573, PH.base.pack) ;
    p33_lon_SET((int32_t) -1712546592, PH.base.pack) ;
    p33_alt_SET((int32_t) -1044827164, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1810066836, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -19279, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)3894, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)21859, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)3784, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)96351316L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -5848, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)2946, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)24928, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -28768, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -3322, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -28019, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -26124, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)24420, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3495266828L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)39693, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)18754, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)7379, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)30262, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)16453, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)12830, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)20151, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)28483, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)3554523210L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)25343, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)46180, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)5270, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)20973, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)51469, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)44738, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)53074, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)21058, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)56021, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)27977, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)36112, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)18287, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)9710, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)61598, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)16975, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)21284, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -25813, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -7985, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -28882, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)32488, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)14513, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p39_param1_SET((float) -3.1930052E38F, PH.base.pack) ;
    p39_param2_SET((float) -1.0820228E38F, PH.base.pack) ;
    p39_param3_SET((float)2.9023996E37F, PH.base.pack) ;
    p39_param4_SET((float) -1.381215E38F, PH.base.pack) ;
    p39_x_SET((float) -7.1003955E37F, PH.base.pack) ;
    p39_y_SET((float)6.59605E37F, PH.base.pack) ;
    p39_z_SET((float) -1.2456656E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)27537, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)4246, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)6770, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)27090, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)14331, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p48_latitude_SET((int32_t) -368903818, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1391193501, PH.base.pack) ;
    p48_altitude_SET((int32_t)1919946, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)7731624826665569278L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -228819401, PH.base.pack) ;
    p49_longitude_SET((int32_t) -1933348016, PH.base.pack) ;
    p49_altitude_SET((int32_t)857227328, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)7799013905538320333L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    {
        char16_t   param_id = "fdhsdPbqiqe";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -27814, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p50_param_value0_SET((float) -7.3489103E37F, PH.base.pack) ;
    p50_scale_SET((float)1.4578112E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)3.2994884E37F, PH.base.pack) ;
    p50_param_value_max_SET((float)6.9760785E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)1554, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p54_p1x_SET((float)2.8444325E38F, PH.base.pack) ;
    p54_p1y_SET((float)2.7023173E38F, PH.base.pack) ;
    p54_p1z_SET((float)2.3325864E38F, PH.base.pack) ;
    p54_p2x_SET((float)9.681213E37F, PH.base.pack) ;
    p54_p2y_SET((float) -3.1281887E38F, PH.base.pack) ;
    p54_p2z_SET((float)8.126191E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float)5.9857184E37F, PH.base.pack) ;
    p55_p1y_SET((float)5.732664E37F, PH.base.pack) ;
    p55_p1z_SET((float) -8.751624E37F, PH.base.pack) ;
    p55_p2x_SET((float) -2.7443084E38F, PH.base.pack) ;
    p55_p2y_SET((float) -9.994615E37F, PH.base.pack) ;
    p55_p2z_SET((float) -2.5589223E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)3656898717440733159L, PH.base.pack) ;
    {
        float  q [] =  {2.9977513E38F, -2.463683E38F, -1.7377573E38F, -4.1668743E37F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -1.8447196E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)3.3214807E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -6.099539E37F, PH.base.pack) ;
    {
        float  covariance [] =  {2.0778071E38F, 1.1044357E38F, -3.303718E38F, 3.1719904E38F, 3.363377E38F, 1.9067623E38F, 9.581108E37F, -1.9011206E36F, 1.8627381E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.299493E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -1.6009512E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)125, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -11661, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)7487, PH.base.pack) ;
    p62_alt_error_SET((float)3.0518842E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)9.278714E37F, PH.base.pack) ;
    p62_xtrack_error_SET((float)2.686264E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)8038850195846040534L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p63_lat_SET((int32_t)1447857579, PH.base.pack) ;
    p63_lon_SET((int32_t)1910169474, PH.base.pack) ;
    p63_alt_SET((int32_t)617550587, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)1946447258, PH.base.pack) ;
    p63_vx_SET((float)2.6525122E38F, PH.base.pack) ;
    p63_vy_SET((float)2.3613077E38F, PH.base.pack) ;
    p63_vz_SET((float)1.2156525E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.5221453E38F, 4.7752094E37F, -8.385139E37F, -2.3819917E38F, 1.4548196E38F, -2.9078682E38F, -1.1636313E38F, 3.2618218E38F, 2.1453692E38F, -1.8004922E38F, 5.2662156E37F, 3.0891491E38F, 9.858551E37F, 1.4255093E38F, 2.3154255E38F, 2.0716652E38F, -1.7594475E38F, -3.3414106E37F, 5.0510725E37F, -9.513615E37F, -2.0761062E38F, 1.1255895E38F, -2.4068588E38F, 2.150266E38F, 1.1924865E38F, -4.92847E37F, -1.2396509E38F, 8.5005746E36F, 2.554379E38F, -2.9608002E38F, -1.2497154E37F, -2.07912E38F, -2.4583986E38F, -1.8326216E36F, -4.452192E37F, -1.1743388E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)1731160792147006607L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p64_x_SET((float)2.1350303E38F, PH.base.pack) ;
    p64_y_SET((float)1.4888157E38F, PH.base.pack) ;
    p64_z_SET((float)4.758651E37F, PH.base.pack) ;
    p64_vx_SET((float) -3.0305512E38F, PH.base.pack) ;
    p64_vy_SET((float)3.0724684E38F, PH.base.pack) ;
    p64_vz_SET((float)1.0895041E38F, PH.base.pack) ;
    p64_ax_SET((float) -1.3867226E38F, PH.base.pack) ;
    p64_ay_SET((float) -9.200721E37F, PH.base.pack) ;
    p64_az_SET((float)1.3378441E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-3.090383E37F, -5.2848267E37F, 1.2390305E38F, -1.1132978E38F, 1.8666632E38F, -1.4485999E38F, 3.2171451E38F, 2.4618864E38F, 1.50582E38F, -1.7390637E38F, 2.598361E38F, 1.3550157E38F, 2.9883455E38F, 1.9239654E37F, 1.1427592E38F, -2.9240043E38F, -3.0170518E38F, 2.0155403E38F, -2.0282773E38F, -3.3007825E38F, 5.2042036E37F, 8.4650584E37F, -2.876545E38F, -9.416233E37F, -5.7160495E37F, -6.6219603E37F, 2.6903799E38F, 1.0129828E38F, -2.7340082E38F, -3.2330145E38F, 1.918617E37F, -1.751E38F, 6.4230264E37F, 1.7126767E38F, -2.3339654E38F, 1.9219228E38F, 9.523668E37F, 3.2532727E38F, 1.876506E38F, -4.8875596E35F, -1.5366827E38F, -3.2729384E38F, 2.3290498E38F, -3.1954496E38F, -1.3220557E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)182953841L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)30300, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)30709, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)45574, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)62866, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)37718, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)58966, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)63343, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)62135, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)49680, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)61424, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)55955, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)29291, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)1719, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)19297, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)15857, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)33856, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)13532, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)36818, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)3094, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)47975, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)32755, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -13508, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)17815, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -12685, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)64856, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)35941, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)27405, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)2547, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)11612, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)59525, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)61358, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)4301, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)908, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)48506, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_LAST, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p73_param1_SET((float)1.6355167E38F, PH.base.pack) ;
    p73_param2_SET((float)1.6087059E38F, PH.base.pack) ;
    p73_param3_SET((float) -1.4372823E38F, PH.base.pack) ;
    p73_param4_SET((float)3.247551E37F, PH.base.pack) ;
    p73_x_SET((int32_t)276573353, PH.base.pack) ;
    p73_y_SET((int32_t)791068570, PH.base.pack) ;
    p73_z_SET((float)2.9665273E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -2.6092746E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.0734406E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -17581, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)36904, PH.base.pack) ;
    p74_alt_SET((float)2.0635964E37F, PH.base.pack) ;
    p74_climb_SET((float)3.668043E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_LAST, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p75_param1_SET((float)1.5080065E38F, PH.base.pack) ;
    p75_param2_SET((float)1.799771E38F, PH.base.pack) ;
    p75_param3_SET((float)7.7304437E37F, PH.base.pack) ;
    p75_param4_SET((float)2.4551277E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -1652537816, PH.base.pack) ;
    p75_y_SET((int32_t)479446861, PH.base.pack) ;
    p75_z_SET((float) -2.0954652E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p76_param1_SET((float)2.8345858E38F, PH.base.pack) ;
    p76_param2_SET((float) -2.8444978E38F, PH.base.pack) ;
    p76_param3_SET((float)5.1575075E37F, PH.base.pack) ;
    p76_param4_SET((float)1.3624743E38F, PH.base.pack) ;
    p76_param5_SET((float)1.3328309E38F, PH.base.pack) ;
    p76_param6_SET((float) -2.4762261E37F, PH.base.pack) ;
    p76_param7_SET((float)9.981984E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)89, &PH) ;
    p77_result_param2_SET((int32_t)735581616, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)208, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)170, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)1311419940L, PH.base.pack) ;
    p81_roll_SET((float) -4.6299863E37F, PH.base.pack) ;
    p81_pitch_SET((float)1.3340668E38F, PH.base.pack) ;
    p81_yaw_SET((float) -4.7162144E37F, PH.base.pack) ;
    p81_thrust_SET((float) -2.0535473E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)4218821802L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    {
        float  q [] =  {2.2595787E38F, 4.2429554E37F, -2.2342147E37F, 2.7280737E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -2.1684417E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -6.721316E37F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -7.63669E37F, PH.base.pack) ;
    p82_thrust_SET((float)2.5788605E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2858800181L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    {
        float  q [] =  {-1.6775486E38F, -8.320612E37F, -1.8224781E38F, -3.0859893E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.4346883E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)1.2239067E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)7.274545E37F, PH.base.pack) ;
    p83_thrust_SET((float) -3.2369809E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1875108138L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)36641, PH.base.pack) ;
    p84_x_SET((float)3.1478397E38F, PH.base.pack) ;
    p84_y_SET((float) -9.566191E37F, PH.base.pack) ;
    p84_z_SET((float) -1.2291602E38F, PH.base.pack) ;
    p84_vx_SET((float)2.6996447E38F, PH.base.pack) ;
    p84_vy_SET((float) -2.3480299E38F, PH.base.pack) ;
    p84_vz_SET((float)2.8260988E38F, PH.base.pack) ;
    p84_afx_SET((float)2.1408605E38F, PH.base.pack) ;
    p84_afy_SET((float) -1.6967231E38F, PH.base.pack) ;
    p84_afz_SET((float) -2.8540097E38F, PH.base.pack) ;
    p84_yaw_SET((float) -2.3567493E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)2.9741827E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)4234898111L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)45044, PH.base.pack) ;
    p86_lat_int_SET((int32_t)168377553, PH.base.pack) ;
    p86_lon_int_SET((int32_t)648544029, PH.base.pack) ;
    p86_alt_SET((float) -3.2297109E38F, PH.base.pack) ;
    p86_vx_SET((float)6.837885E37F, PH.base.pack) ;
    p86_vy_SET((float) -1.6880145E38F, PH.base.pack) ;
    p86_vz_SET((float)3.23875E37F, PH.base.pack) ;
    p86_afx_SET((float)4.45513E37F, PH.base.pack) ;
    p86_afy_SET((float)3.2251492E38F, PH.base.pack) ;
    p86_afz_SET((float)3.1712833E38F, PH.base.pack) ;
    p86_yaw_SET((float)5.9255805E37F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -2.3538645E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)3717869869L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)53345, PH.base.pack) ;
    p87_lat_int_SET((int32_t)35931969, PH.base.pack) ;
    p87_lon_int_SET((int32_t)754132690, PH.base.pack) ;
    p87_alt_SET((float) -1.1712878E36F, PH.base.pack) ;
    p87_vx_SET((float)1.8002256E37F, PH.base.pack) ;
    p87_vy_SET((float) -2.9003262E38F, PH.base.pack) ;
    p87_vz_SET((float) -3.005528E38F, PH.base.pack) ;
    p87_afx_SET((float)6.922689E37F, PH.base.pack) ;
    p87_afy_SET((float) -6.760996E37F, PH.base.pack) ;
    p87_afz_SET((float) -1.6076523E38F, PH.base.pack) ;
    p87_yaw_SET((float)3.1182243E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)1.2771306E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)4087316055L, PH.base.pack) ;
    p89_x_SET((float)1.4042512E38F, PH.base.pack) ;
    p89_y_SET((float) -3.106543E38F, PH.base.pack) ;
    p89_z_SET((float)9.334634E37F, PH.base.pack) ;
    p89_roll_SET((float) -2.8727076E38F, PH.base.pack) ;
    p89_pitch_SET((float) -2.4185767E38F, PH.base.pack) ;
    p89_yaw_SET((float) -3.2577085E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)7845173993986294445L, PH.base.pack) ;
    p90_roll_SET((float) -3.0951E38F, PH.base.pack) ;
    p90_pitch_SET((float) -2.0178136E38F, PH.base.pack) ;
    p90_yaw_SET((float) -2.3441803E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)2.3435913E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float)2.722123E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -6.245651E37F, PH.base.pack) ;
    p90_lat_SET((int32_t) -107172653, PH.base.pack) ;
    p90_lon_SET((int32_t) -1797747893, PH.base.pack) ;
    p90_alt_SET((int32_t)1997390051, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)15245, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -8734, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -17222, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -2810, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)2929, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -11514, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)3726296660629471216L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)1.0276498E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -1.5091921E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.343431E38F, PH.base.pack) ;
    p91_throttle_SET((float)9.66928E36F, PH.base.pack) ;
    p91_aux1_SET((float) -2.8394728E38F, PH.base.pack) ;
    p91_aux2_SET((float) -3.3618185E38F, PH.base.pack) ;
    p91_aux3_SET((float) -2.1889705E38F, PH.base.pack) ;
    p91_aux4_SET((float) -3.3235095E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)4941274036692469576L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)7231, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)52549, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)37044, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)9181, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)34468, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)22517, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)17475, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)53173, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)12850, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)42461, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)16030, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)13413, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)6794005300086973105L, PH.base.pack) ;
    {
        float  controls [] =  {-1.9895117E38F, -7.1997686E37F, -1.3177757E38F, -2.0231685E38F, 1.1951884E38F, -2.3170106E38F, 2.6225772E38F, 3.350684E38F, 3.008789E38F, 4.837972E36F, -5.194221E37F, -2.0536195E38F, -1.245009E38F, -1.051266E38F, 7.866889E37F, 1.550157E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)5929656449171791155L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)292731884318552842L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -19959, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -5469, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)5.213959E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)2.415198E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p100_ground_distance_SET((float)1.141804E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -3.5811003E37F, &PH) ;
    p100_flow_rate_y_SET((float) -1.8456593E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)6454122411154142140L, PH.base.pack) ;
    p101_x_SET((float)5.726693E37F, PH.base.pack) ;
    p101_y_SET((float)2.6330281E38F, PH.base.pack) ;
    p101_z_SET((float)3.0782583E38F, PH.base.pack) ;
    p101_roll_SET((float) -2.6799637E38F, PH.base.pack) ;
    p101_pitch_SET((float) -2.6689686E38F, PH.base.pack) ;
    p101_yaw_SET((float)2.0721633E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)6525397155193345281L, PH.base.pack) ;
    p102_x_SET((float)2.843301E38F, PH.base.pack) ;
    p102_y_SET((float)1.915899E38F, PH.base.pack) ;
    p102_z_SET((float)2.3809652E38F, PH.base.pack) ;
    p102_roll_SET((float) -4.2932826E37F, PH.base.pack) ;
    p102_pitch_SET((float)5.484929E37F, PH.base.pack) ;
    p102_yaw_SET((float)2.9314442E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)736077572130504839L, PH.base.pack) ;
    p103_x_SET((float) -3.0344318E38F, PH.base.pack) ;
    p103_y_SET((float)2.242169E38F, PH.base.pack) ;
    p103_z_SET((float)1.2166891E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)6095041232878298888L, PH.base.pack) ;
    p104_x_SET((float) -1.8079896E38F, PH.base.pack) ;
    p104_y_SET((float)1.264718E38F, PH.base.pack) ;
    p104_z_SET((float)1.884979E38F, PH.base.pack) ;
    p104_roll_SET((float)1.539996E38F, PH.base.pack) ;
    p104_pitch_SET((float) -1.6379408E38F, PH.base.pack) ;
    p104_yaw_SET((float)5.520922E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)9019240499458611571L, PH.base.pack) ;
    p105_xacc_SET((float)1.0664458E38F, PH.base.pack) ;
    p105_yacc_SET((float)2.7696857E38F, PH.base.pack) ;
    p105_zacc_SET((float)2.1295838E38F, PH.base.pack) ;
    p105_xgyro_SET((float)3.1723944E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -1.4409752E36F, PH.base.pack) ;
    p105_zgyro_SET((float)1.0548733E38F, PH.base.pack) ;
    p105_xmag_SET((float) -3.1080038E38F, PH.base.pack) ;
    p105_ymag_SET((float)1.0186887E38F, PH.base.pack) ;
    p105_zmag_SET((float)8.2302535E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float)6.0153413E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float)1.1267701E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)4.207581E34F, PH.base.pack) ;
    p105_temperature_SET((float) -3.0617743E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)7514, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)1610294862143049955L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2636933887L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.4756585E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -5.4007696E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.4073003E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -1.7115863E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)1.1555664E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)224, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)4281670181L, PH.base.pack) ;
    p106_distance_SET((float)5.745236E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)6421054737076270571L, PH.base.pack) ;
    p107_xacc_SET((float) -2.278437E38F, PH.base.pack) ;
    p107_yacc_SET((float) -2.266655E38F, PH.base.pack) ;
    p107_zacc_SET((float)3.1772068E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.5969157E38F, PH.base.pack) ;
    p107_ygyro_SET((float)1.348741E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -2.4327683E37F, PH.base.pack) ;
    p107_xmag_SET((float) -1.7991899E37F, PH.base.pack) ;
    p107_ymag_SET((float) -2.7126331E38F, PH.base.pack) ;
    p107_zmag_SET((float)3.144912E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -2.0481104E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float)3.2819395E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)3.5444091E37F, PH.base.pack) ;
    p107_temperature_SET((float)4.9629437E36F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)3223042593L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.9848013E38F, PH.base.pack) ;
    p108_q2_SET((float)1.5499181E38F, PH.base.pack) ;
    p108_q3_SET((float)2.9419988E38F, PH.base.pack) ;
    p108_q4_SET((float) -1.4008248E38F, PH.base.pack) ;
    p108_roll_SET((float)2.5877517E38F, PH.base.pack) ;
    p108_pitch_SET((float) -9.308348E37F, PH.base.pack) ;
    p108_yaw_SET((float) -6.133714E37F, PH.base.pack) ;
    p108_xacc_SET((float) -3.1782276E38F, PH.base.pack) ;
    p108_yacc_SET((float)2.874486E38F, PH.base.pack) ;
    p108_zacc_SET((float) -2.7085093E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -1.2255934E37F, PH.base.pack) ;
    p108_ygyro_SET((float)1.5468658E38F, PH.base.pack) ;
    p108_zgyro_SET((float)3.2362623E38F, PH.base.pack) ;
    p108_lat_SET((float)3.262923E38F, PH.base.pack) ;
    p108_lon_SET((float) -2.821367E38F, PH.base.pack) ;
    p108_alt_SET((float)1.8152254E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -9.256076E36F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)3.3114518E38F, PH.base.pack) ;
    p108_vn_SET((float) -3.26483E38F, PH.base.pack) ;
    p108_ve_SET((float) -2.4232997E38F, PH.base.pack) ;
    p108_vd_SET((float)3.1439658E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)60141, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)18102, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)226, (uint8_t)169, (uint8_t)136, (uint8_t)19, (uint8_t)168, (uint8_t)195, (uint8_t)123, (uint8_t)194, (uint8_t)218, (uint8_t)67, (uint8_t)1, (uint8_t)116, (uint8_t)74, (uint8_t)22, (uint8_t)63, (uint8_t)120, (uint8_t)82, (uint8_t)81, (uint8_t)33, (uint8_t)97, (uint8_t)240, (uint8_t)113, (uint8_t)123, (uint8_t)192, (uint8_t)77, (uint8_t)178, (uint8_t)28, (uint8_t)59, (uint8_t)226, (uint8_t)35, (uint8_t)60, (uint8_t)140, (uint8_t)255, (uint8_t)174, (uint8_t)60, (uint8_t)211, (uint8_t)72, (uint8_t)254, (uint8_t)148, (uint8_t)67, (uint8_t)171, (uint8_t)56, (uint8_t)198, (uint8_t)84, (uint8_t)12, (uint8_t)151, (uint8_t)235, (uint8_t)186, (uint8_t)232, (uint8_t)192, (uint8_t)242, (uint8_t)81, (uint8_t)47, (uint8_t)246, (uint8_t)238, (uint8_t)121, (uint8_t)191, (uint8_t)222, (uint8_t)98, (uint8_t)249, (uint8_t)200, (uint8_t)187, (uint8_t)65, (uint8_t)53, (uint8_t)140, (uint8_t)244, (uint8_t)179, (uint8_t)175, (uint8_t)80, (uint8_t)26, (uint8_t)186, (uint8_t)223, (uint8_t)30, (uint8_t)42, (uint8_t)158, (uint8_t)74, (uint8_t)1, (uint8_t)101, (uint8_t)133, (uint8_t)231, (uint8_t)77, (uint8_t)156, (uint8_t)189, (uint8_t)155, (uint8_t)90, (uint8_t)38, (uint8_t)66, (uint8_t)240, (uint8_t)218, (uint8_t)200, (uint8_t)237, (uint8_t)202, (uint8_t)155, (uint8_t)112, (uint8_t)50, (uint8_t)72, (uint8_t)239, (uint8_t)68, (uint8_t)182, (uint8_t)148, (uint8_t)1, (uint8_t)243, (uint8_t)1, (uint8_t)215, (uint8_t)133, (uint8_t)185, (uint8_t)20, (uint8_t)232, (uint8_t)117, (uint8_t)252, (uint8_t)177, (uint8_t)10, (uint8_t)228, (uint8_t)99, (uint8_t)157, (uint8_t)214, (uint8_t)248, (uint8_t)147, (uint8_t)70, (uint8_t)66, (uint8_t)132, (uint8_t)14, (uint8_t)201, (uint8_t)142, (uint8_t)18, (uint8_t)134, (uint8_t)3, (uint8_t)164, (uint8_t)85, (uint8_t)175, (uint8_t)251, (uint8_t)184, (uint8_t)240, (uint8_t)182, (uint8_t)194, (uint8_t)136, (uint8_t)16, (uint8_t)213, (uint8_t)22, (uint8_t)214, (uint8_t)20, (uint8_t)60, (uint8_t)84, (uint8_t)192, (uint8_t)5, (uint8_t)66, (uint8_t)32, (uint8_t)252, (uint8_t)32, (uint8_t)233, (uint8_t)19, (uint8_t)181, (uint8_t)43, (uint8_t)121, (uint8_t)154, (uint8_t)142, (uint8_t)165, (uint8_t)162, (uint8_t)10, (uint8_t)125, (uint8_t)167, (uint8_t)64, (uint8_t)229, (uint8_t)251, (uint8_t)171, (uint8_t)165, (uint8_t)203, (uint8_t)129, (uint8_t)56, (uint8_t)48, (uint8_t)182, (uint8_t)227, (uint8_t)146, (uint8_t)151, (uint8_t)49, (uint8_t)52, (uint8_t)36, (uint8_t)30, (uint8_t)4, (uint8_t)187, (uint8_t)62, (uint8_t)222, (uint8_t)185, (uint8_t)247, (uint8_t)253, (uint8_t)224, (uint8_t)126, (uint8_t)217, (uint8_t)49, (uint8_t)248, (uint8_t)17, (uint8_t)197, (uint8_t)39, (uint8_t)0, (uint8_t)65, (uint8_t)252, (uint8_t)79, (uint8_t)133, (uint8_t)193, (uint8_t)116, (uint8_t)4, (uint8_t)86, (uint8_t)245, (uint8_t)87, (uint8_t)163, (uint8_t)99, (uint8_t)199, (uint8_t)3, (uint8_t)97, (uint8_t)249, (uint8_t)232, (uint8_t)221, (uint8_t)133, (uint8_t)51, (uint8_t)19, (uint8_t)163, (uint8_t)116, (uint8_t)95, (uint8_t)118, (uint8_t)184, (uint8_t)10, (uint8_t)214, (uint8_t)13, (uint8_t)142, (uint8_t)223, (uint8_t)1, (uint8_t)229, (uint8_t)235, (uint8_t)247, (uint8_t)207, (uint8_t)2, (uint8_t)34, (uint8_t)151, (uint8_t)61, (uint8_t)119, (uint8_t)223, (uint8_t)70, (uint8_t)110, (uint8_t)37, (uint8_t)96, (uint8_t)206, (uint8_t)17, (uint8_t)235, (uint8_t)142, (uint8_t)153, (uint8_t)145, (uint8_t)53, (uint8_t)175, (uint8_t)173, (uint8_t)175, (uint8_t)54};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -6440631365135035851L, PH.base.pack) ;
    p111_ts1_SET((int64_t)6723917535126633337L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)8593987282316295978L, PH.base.pack) ;
    p112_seq_SET((uint32_t)430080825L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)4622690575827457723L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p113_lat_SET((int32_t) -797906805, PH.base.pack) ;
    p113_lon_SET((int32_t)623424772, PH.base.pack) ;
    p113_alt_SET((int32_t) -115502051, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)33054, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)34425, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)32494, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)14622, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)29397, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -1882, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)33026, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)8718043004646910467L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)1917891900L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.5259957E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.4996046E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)3.6244985E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.9073798E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -1.8500448E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)6496, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)3511552774L, PH.base.pack) ;
    p114_distance_SET((float) -5.341343E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)4197252354668699124L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-2.45888E38F, -3.3122511E38F, -1.9312722E38F, -1.32662E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)1.0890438E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)1.8185789E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.0996971E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -276394237, PH.base.pack) ;
    p115_lon_SET((int32_t) -451653568, PH.base.pack) ;
    p115_alt_SET((int32_t)702735812, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)28132, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -26324, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)21397, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)54141, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)43775, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -16389, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)14400, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)22702, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)2204743480L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -7647, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)3520, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -4167, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -1374, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)14425, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)22217, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -13252, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)8318, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)10438, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)10589, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)62887, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)17966, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)1248, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)16802, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)1070212821L, PH.base.pack) ;
    p118_size_SET((uint32_t)1701214686L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)64904, PH.base.pack) ;
    p119_ofs_SET((uint32_t)2945900884L, PH.base.pack) ;
    p119_count_SET((uint32_t)2284412192L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)9210, PH.base.pack) ;
    p120_ofs_SET((uint32_t)2622622309L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)38, (uint8_t)126, (uint8_t)172, (uint8_t)73, (uint8_t)154, (uint8_t)179, (uint8_t)168, (uint8_t)61, (uint8_t)139, (uint8_t)212, (uint8_t)105, (uint8_t)171, (uint8_t)160, (uint8_t)134, (uint8_t)7, (uint8_t)188, (uint8_t)208, (uint8_t)66, (uint8_t)60, (uint8_t)204, (uint8_t)23, (uint8_t)12, (uint8_t)120, (uint8_t)27, (uint8_t)198, (uint8_t)143, (uint8_t)21, (uint8_t)106, (uint8_t)250, (uint8_t)33, (uint8_t)21, (uint8_t)211, (uint8_t)131, (uint8_t)101, (uint8_t)79, (uint8_t)31, (uint8_t)24, (uint8_t)24, (uint8_t)16, (uint8_t)169, (uint8_t)248, (uint8_t)35, (uint8_t)16, (uint8_t)158, (uint8_t)173, (uint8_t)61, (uint8_t)89, (uint8_t)233, (uint8_t)76, (uint8_t)255, (uint8_t)170, (uint8_t)200, (uint8_t)155, (uint8_t)140, (uint8_t)26, (uint8_t)64, (uint8_t)208, (uint8_t)60, (uint8_t)10, (uint8_t)179, (uint8_t)33, (uint8_t)147, (uint8_t)9, (uint8_t)103, (uint8_t)189, (uint8_t)79, (uint8_t)221, (uint8_t)72, (uint8_t)4, (uint8_t)186, (uint8_t)207, (uint8_t)238, (uint8_t)221, (uint8_t)249, (uint8_t)192, (uint8_t)92, (uint8_t)170, (uint8_t)195, (uint8_t)85, (uint8_t)167, (uint8_t)233, (uint8_t)58, (uint8_t)137, (uint8_t)220, (uint8_t)65, (uint8_t)141, (uint8_t)37, (uint8_t)80, (uint8_t)138, (uint8_t)51};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)129, (uint8_t)66, (uint8_t)188, (uint8_t)225, (uint8_t)240, (uint8_t)227, (uint8_t)20, (uint8_t)72, (uint8_t)224, (uint8_t)245, (uint8_t)193, (uint8_t)68, (uint8_t)69, (uint8_t)86, (uint8_t)232, (uint8_t)204, (uint8_t)31, (uint8_t)210, (uint8_t)14, (uint8_t)85, (uint8_t)150, (uint8_t)224, (uint8_t)233, (uint8_t)139, (uint8_t)62, (uint8_t)196, (uint8_t)185, (uint8_t)75, (uint8_t)56, (uint8_t)20, (uint8_t)82, (uint8_t)140, (uint8_t)7, (uint8_t)114, (uint8_t)129, (uint8_t)176, (uint8_t)206, (uint8_t)42, (uint8_t)183, (uint8_t)147, (uint8_t)89, (uint8_t)154, (uint8_t)225, (uint8_t)150, (uint8_t)37, (uint8_t)85, (uint8_t)38, (uint8_t)85, (uint8_t)154, (uint8_t)98, (uint8_t)199, (uint8_t)125, (uint8_t)28, (uint8_t)66, (uint8_t)181, (uint8_t)212, (uint8_t)97, (uint8_t)120, (uint8_t)162, (uint8_t)15, (uint8_t)226, (uint8_t)155, (uint8_t)32, (uint8_t)66, (uint8_t)50, (uint8_t)200, (uint8_t)59, (uint8_t)81, (uint8_t)193, (uint8_t)53, (uint8_t)247, (uint8_t)70, (uint8_t)84, (uint8_t)53, (uint8_t)52, (uint8_t)186, (uint8_t)38, (uint8_t)200, (uint8_t)2, (uint8_t)37, (uint8_t)236, (uint8_t)94, (uint8_t)156, (uint8_t)47, (uint8_t)110, (uint8_t)99, (uint8_t)43, (uint8_t)165, (uint8_t)120, (uint8_t)105, (uint8_t)222, (uint8_t)149, (uint8_t)32, (uint8_t)222, (uint8_t)147, (uint8_t)195, (uint8_t)30, (uint8_t)24, (uint8_t)153, (uint8_t)239, (uint8_t)103, (uint8_t)228, (uint8_t)130, (uint8_t)39, (uint8_t)119, (uint8_t)56, (uint8_t)200, (uint8_t)16, (uint8_t)250, (uint8_t)246};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)973662631101024101L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p124_lat_SET((int32_t)1575085529, PH.base.pack) ;
    p124_lon_SET((int32_t) -1820676897, PH.base.pack) ;
    p124_alt_SET((int32_t) -1138271717, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)18575, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)38046, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)14893, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)38890, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1860016856L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)12383, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)18211, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)63447, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)3175333986L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)61, (uint8_t)216, (uint8_t)253, (uint8_t)142, (uint8_t)130, (uint8_t)239, (uint8_t)113, (uint8_t)170, (uint8_t)20, (uint8_t)81, (uint8_t)64, (uint8_t)54, (uint8_t)150, (uint8_t)212, (uint8_t)224, (uint8_t)11, (uint8_t)144, (uint8_t)219, (uint8_t)47, (uint8_t)235, (uint8_t)4, (uint8_t)175, (uint8_t)2, (uint8_t)96, (uint8_t)40, (uint8_t)221, (uint8_t)54, (uint8_t)87, (uint8_t)220, (uint8_t)109, (uint8_t)5, (uint8_t)248, (uint8_t)225, (uint8_t)200, (uint8_t)183, (uint8_t)108, (uint8_t)104, (uint8_t)126, (uint8_t)197, (uint8_t)77, (uint8_t)47, (uint8_t)124, (uint8_t)181, (uint8_t)221, (uint8_t)194, (uint8_t)215, (uint8_t)16, (uint8_t)166, (uint8_t)4, (uint8_t)166, (uint8_t)114, (uint8_t)146, (uint8_t)220, (uint8_t)28, (uint8_t)152, (uint8_t)0, (uint8_t)255, (uint8_t)212, (uint8_t)18, (uint8_t)16, (uint8_t)255, (uint8_t)184, (uint8_t)140, (uint8_t)221, (uint8_t)225, (uint8_t)176, (uint8_t)52, (uint8_t)62, (uint8_t)52, (uint8_t)219};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3968674258L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)13261, PH.base.pack) ;
    p127_tow_SET((uint32_t)890958818L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -909175853, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)912394693, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -682955782, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2144577906L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)31920134, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)665835003L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)291, PH.base.pack) ;
    p128_tow_SET((uint32_t)4173764398L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1655722255, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -714463590, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -1247958155, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)3947084670L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)976681062, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2965477898L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)3836, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -31929, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -473, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)27851, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -12529, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -21396, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)32026, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)24872, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -4317, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p130_size_SET((uint32_t)3741444885L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)21708, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)63705, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)12095, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)61316, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)253, (uint8_t)159, (uint8_t)11, (uint8_t)161, (uint8_t)148, (uint8_t)79, (uint8_t)73, (uint8_t)234, (uint8_t)205, (uint8_t)245, (uint8_t)177, (uint8_t)250, (uint8_t)141, (uint8_t)213, (uint8_t)149, (uint8_t)79, (uint8_t)140, (uint8_t)222, (uint8_t)234, (uint8_t)174, (uint8_t)165, (uint8_t)56, (uint8_t)53, (uint8_t)242, (uint8_t)157, (uint8_t)90, (uint8_t)131, (uint8_t)54, (uint8_t)197, (uint8_t)38, (uint8_t)95, (uint8_t)158, (uint8_t)35, (uint8_t)224, (uint8_t)250, (uint8_t)235, (uint8_t)63, (uint8_t)128, (uint8_t)8, (uint8_t)116, (uint8_t)148, (uint8_t)236, (uint8_t)200, (uint8_t)21, (uint8_t)211, (uint8_t)202, (uint8_t)6, (uint8_t)156, (uint8_t)114, (uint8_t)46, (uint8_t)166, (uint8_t)53, (uint8_t)156, (uint8_t)2, (uint8_t)145, (uint8_t)155, (uint8_t)66, (uint8_t)228, (uint8_t)38, (uint8_t)135, (uint8_t)28, (uint8_t)66, (uint8_t)50, (uint8_t)204, (uint8_t)161, (uint8_t)76, (uint8_t)109, (uint8_t)155, (uint8_t)29, (uint8_t)58, (uint8_t)234, (uint8_t)150, (uint8_t)112, (uint8_t)65, (uint8_t)246, (uint8_t)153, (uint8_t)50, (uint8_t)18, (uint8_t)13, (uint8_t)27, (uint8_t)61, (uint8_t)60, (uint8_t)230, (uint8_t)38, (uint8_t)52, (uint8_t)152, (uint8_t)9, (uint8_t)232, (uint8_t)114, (uint8_t)24, (uint8_t)9, (uint8_t)83, (uint8_t)47, (uint8_t)255, (uint8_t)227, (uint8_t)170, (uint8_t)54, (uint8_t)173, (uint8_t)178, (uint8_t)188, (uint8_t)231, (uint8_t)143, (uint8_t)170, (uint8_t)82, (uint8_t)227, (uint8_t)241, (uint8_t)169, (uint8_t)191, (uint8_t)18, (uint8_t)96, (uint8_t)70, (uint8_t)154, (uint8_t)196, (uint8_t)100, (uint8_t)183, (uint8_t)18, (uint8_t)255, (uint8_t)179, (uint8_t)60, (uint8_t)39, (uint8_t)115, (uint8_t)67, (uint8_t)168, (uint8_t)79, (uint8_t)76, (uint8_t)135, (uint8_t)188, (uint8_t)161, (uint8_t)112, (uint8_t)94, (uint8_t)64, (uint8_t)255, (uint8_t)52, (uint8_t)42, (uint8_t)85, (uint8_t)16, (uint8_t)117, (uint8_t)161, (uint8_t)100, (uint8_t)2, (uint8_t)238, (uint8_t)106, (uint8_t)241, (uint8_t)201, (uint8_t)218, (uint8_t)209, (uint8_t)165, (uint8_t)190, (uint8_t)35, (uint8_t)3, (uint8_t)228, (uint8_t)190, (uint8_t)147, (uint8_t)170, (uint8_t)81, (uint8_t)80, (uint8_t)159, (uint8_t)18, (uint8_t)146, (uint8_t)60, (uint8_t)133, (uint8_t)146, (uint8_t)57, (uint8_t)233, (uint8_t)236, (uint8_t)27, (uint8_t)217, (uint8_t)237, (uint8_t)33, (uint8_t)218, (uint8_t)201, (uint8_t)157, (uint8_t)248, (uint8_t)252, (uint8_t)167, (uint8_t)191, (uint8_t)13, (uint8_t)82, (uint8_t)57, (uint8_t)213, (uint8_t)48, (uint8_t)152, (uint8_t)195, (uint8_t)102, (uint8_t)168, (uint8_t)180, (uint8_t)130, (uint8_t)55, (uint8_t)105, (uint8_t)158, (uint8_t)76, (uint8_t)222, (uint8_t)152, (uint8_t)153, (uint8_t)239, (uint8_t)61, (uint8_t)226, (uint8_t)153, (uint8_t)171, (uint8_t)132, (uint8_t)38, (uint8_t)44, (uint8_t)155, (uint8_t)80, (uint8_t)139, (uint8_t)192, (uint8_t)57, (uint8_t)99, (uint8_t)1, (uint8_t)242, (uint8_t)222, (uint8_t)196, (uint8_t)15, (uint8_t)38, (uint8_t)59, (uint8_t)62, (uint8_t)35, (uint8_t)28, (uint8_t)112, (uint8_t)180, (uint8_t)166, (uint8_t)186, (uint8_t)146, (uint8_t)234, (uint8_t)185, (uint8_t)20, (uint8_t)9, (uint8_t)166, (uint8_t)94, (uint8_t)133, (uint8_t)71, (uint8_t)224, (uint8_t)151, (uint8_t)87, (uint8_t)26, (uint8_t)230, (uint8_t)13, (uint8_t)80, (uint8_t)178, (uint8_t)124, (uint8_t)180, (uint8_t)74, (uint8_t)110, (uint8_t)36, (uint8_t)73, (uint8_t)27, (uint8_t)223, (uint8_t)167, (uint8_t)119, (uint8_t)201, (uint8_t)190, (uint8_t)239, (uint8_t)237};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)3472869518L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)35213, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)14875, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)52180, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1640037053, PH.base.pack) ;
    p133_lon_SET((int32_t)551384158, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)19947, PH.base.pack) ;
    p133_mask_SET((uint64_t)4542666213796413055L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1511468463, PH.base.pack) ;
    p134_lon_SET((int32_t)1454124596, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)61022, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)20607, (int16_t) -30200, (int16_t)22980, (int16_t)27177, (int16_t)13357, (int16_t) -9135, (int16_t)25917, (int16_t)20457, (int16_t)716, (int16_t) -7505, (int16_t)3623, (int16_t) -3013, (int16_t)659, (int16_t) -28971, (int16_t) -9425, (int16_t) -32007};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1497950207, PH.base.pack) ;
    p135_lon_SET((int32_t) -1477334464, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)449220449, PH.base.pack) ;
    p136_lon_SET((int32_t) -2063363146, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)23692, PH.base.pack) ;
    p136_terrain_height_SET((float)2.4905884E38F, PH.base.pack) ;
    p136_current_height_SET((float) -2.4811898E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)16288, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)37116, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)578013535L, PH.base.pack) ;
    p137_press_abs_SET((float)3.8357378E37F, PH.base.pack) ;
    p137_press_diff_SET((float) -2.9397625E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)16827, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)5861726385947612676L, PH.base.pack) ;
    {
        float  q [] =  {3.1718007E38F, 9.804059E36F, -1.5660582E38F, -3.1879161E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -2.5262443E38F, PH.base.pack) ;
    p138_y_SET((float) -1.4023006E38F, PH.base.pack) ;
    p138_z_SET((float)2.5950084E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)6163999830215029574L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    {
        float  controls [] =  {1.6211112E38F, 2.6738205E38F, -2.823222E38F, -2.7483927E38F, 2.6103502E38F, 1.4494644E38F, 3.3542193E38F, 1.8896874E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)4052361423819550863L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    {
        float  controls [] =  {-3.331724E38F, -3.2861897E38F, -2.266272E38F, -4.722784E37F, 2.8878687E38F, -1.4450091E38F, -2.477504E38F, -2.8472045E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)5120233723022597456L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -1.8959309E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -2.6833522E38F, PH.base.pack) ;
    p141_altitude_local_SET((float)2.6619312E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -4.80422E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -1.0542212E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)1.0908865E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)194, (uint8_t)202, (uint8_t)73, (uint8_t)231, (uint8_t)15, (uint8_t)129, (uint8_t)64, (uint8_t)40, (uint8_t)139, (uint8_t)81, (uint8_t)57, (uint8_t)121, (uint8_t)40, (uint8_t)5, (uint8_t)215, (uint8_t)153, (uint8_t)72, (uint8_t)252, (uint8_t)78, (uint8_t)181, (uint8_t)133, (uint8_t)146, (uint8_t)122, (uint8_t)82, (uint8_t)42, (uint8_t)115, (uint8_t)200, (uint8_t)223, (uint8_t)70, (uint8_t)52, (uint8_t)133, (uint8_t)132, (uint8_t)86, (uint8_t)45, (uint8_t)218, (uint8_t)36, (uint8_t)193, (uint8_t)30, (uint8_t)3, (uint8_t)121, (uint8_t)208, (uint8_t)100, (uint8_t)189, (uint8_t)94, (uint8_t)216, (uint8_t)84, (uint8_t)55, (uint8_t)188, (uint8_t)236, (uint8_t)5, (uint8_t)65, (uint8_t)182, (uint8_t)223, (uint8_t)209, (uint8_t)115, (uint8_t)157, (uint8_t)135, (uint8_t)179, (uint8_t)66, (uint8_t)224, (uint8_t)25, (uint8_t)60, (uint8_t)177, (uint8_t)138, (uint8_t)161, (uint8_t)132, (uint8_t)153, (uint8_t)236, (uint8_t)153, (uint8_t)252, (uint8_t)75, (uint8_t)36, (uint8_t)124, (uint8_t)112, (uint8_t)38, (uint8_t)233, (uint8_t)6, (uint8_t)183, (uint8_t)66, (uint8_t)154, (uint8_t)67, (uint8_t)211, (uint8_t)193, (uint8_t)147, (uint8_t)96, (uint8_t)129, (uint8_t)106, (uint8_t)134, (uint8_t)51, (uint8_t)123, (uint8_t)160, (uint8_t)210, (uint8_t)162, (uint8_t)43, (uint8_t)110, (uint8_t)221, (uint8_t)48, (uint8_t)78, (uint8_t)177, (uint8_t)68, (uint8_t)199, (uint8_t)100, (uint8_t)121, (uint8_t)170, (uint8_t)82, (uint8_t)156, (uint8_t)245, (uint8_t)86, (uint8_t)183, (uint8_t)27, (uint8_t)99, (uint8_t)88, (uint8_t)94, (uint8_t)117, (uint8_t)56, (uint8_t)221, (uint8_t)225, (uint8_t)116, (uint8_t)243, (uint8_t)119};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)65, (uint8_t)19, (uint8_t)220, (uint8_t)227, (uint8_t)66, (uint8_t)249, (uint8_t)70, (uint8_t)254, (uint8_t)211, (uint8_t)126, (uint8_t)68, (uint8_t)161, (uint8_t)91, (uint8_t)46, (uint8_t)90, (uint8_t)218, (uint8_t)148, (uint8_t)113, (uint8_t)162, (uint8_t)157, (uint8_t)117, (uint8_t)54, (uint8_t)191, (uint8_t)56, (uint8_t)53, (uint8_t)201, (uint8_t)111, (uint8_t)30, (uint8_t)142, (uint8_t)46, (uint8_t)100, (uint8_t)139, (uint8_t)143, (uint8_t)226, (uint8_t)23, (uint8_t)139, (uint8_t)95, (uint8_t)41, (uint8_t)203, (uint8_t)90, (uint8_t)6, (uint8_t)90, (uint8_t)16, (uint8_t)42, (uint8_t)70, (uint8_t)17, (uint8_t)97, (uint8_t)224, (uint8_t)208, (uint8_t)183, (uint8_t)231, (uint8_t)56, (uint8_t)202, (uint8_t)102, (uint8_t)33, (uint8_t)225, (uint8_t)17, (uint8_t)230, (uint8_t)167, (uint8_t)109, (uint8_t)225, (uint8_t)37, (uint8_t)249, (uint8_t)130, (uint8_t)228, (uint8_t)135, (uint8_t)106, (uint8_t)25, (uint8_t)157, (uint8_t)57, (uint8_t)27, (uint8_t)114, (uint8_t)7, (uint8_t)202, (uint8_t)200, (uint8_t)13, (uint8_t)253, (uint8_t)127, (uint8_t)98, (uint8_t)154, (uint8_t)111, (uint8_t)37, (uint8_t)220, (uint8_t)72, (uint8_t)60, (uint8_t)120, (uint8_t)32, (uint8_t)50, (uint8_t)53, (uint8_t)23, (uint8_t)152, (uint8_t)73, (uint8_t)246, (uint8_t)92, (uint8_t)127, (uint8_t)167, (uint8_t)184, (uint8_t)66, (uint8_t)18, (uint8_t)163, (uint8_t)237, (uint8_t)118, (uint8_t)239, (uint8_t)114, (uint8_t)175, (uint8_t)138, (uint8_t)0, (uint8_t)165, (uint8_t)70, (uint8_t)66, (uint8_t)211, (uint8_t)141, (uint8_t)179, (uint8_t)207, (uint8_t)182, (uint8_t)188, (uint8_t)64, (uint8_t)183, (uint8_t)175, (uint8_t)112};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)2370313558L, PH.base.pack) ;
    p143_press_abs_SET((float)3.1964737E38F, PH.base.pack) ;
    p143_press_diff_SET((float)3.215529E36F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -26794, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)7533659079676487551L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p144_lat_SET((int32_t)1751808273, PH.base.pack) ;
    p144_lon_SET((int32_t) -790813177, PH.base.pack) ;
    p144_alt_SET((float) -6.534419E37F, PH.base.pack) ;
    {
        float  vel [] =  {1.4298151E38F, 5.4890833E37F, 5.3897263E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {1.4468715E38F, 1.5695912E38F, -1.1453632E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-2.9919284E38F, -7.0627655E37F, 5.746036E37F, -2.6190653E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {5.8249255E37F, 3.7808752E37F, 1.6183941E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {2.7519127E38F, -2.1831527E38F, 2.5449082E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)742184281453650599L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)4146276275395207374L, PH.base.pack) ;
    p146_x_acc_SET((float)8.566209E37F, PH.base.pack) ;
    p146_y_acc_SET((float)2.3923707E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -1.7761205E38F, PH.base.pack) ;
    p146_x_vel_SET((float)2.1963495E38F, PH.base.pack) ;
    p146_y_vel_SET((float)1.2253248E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -4.9258073E37F, PH.base.pack) ;
    p146_x_pos_SET((float)1.2517778E38F, PH.base.pack) ;
    p146_y_pos_SET((float)2.5980187E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -1.7776607E38F, PH.base.pack) ;
    p146_airspeed_SET((float)2.0654674E37F, PH.base.pack) ;
    {
        float  vel_variance [] =  {-2.7801448E38F, 2.70751E38F, -1.962639E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.1934989E38F, -2.2601928E38F, -2.9636063E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {5.129875E37F, 1.7673465E38F, 3.268617E38F, -2.7766626E37F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)1.1389389E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)2.2379725E37F, PH.base.pack) ;
    p146_yaw_rate_SET((float)1.1029187E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -19761, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)40900, (uint16_t)19949, (uint16_t)36434, (uint16_t)50001, (uint16_t)29127, (uint16_t)56586, (uint16_t)41906, (uint16_t)64311, (uint16_t)27855, (uint16_t)44884};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -26859, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)494483314, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)575430738, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1114796113L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)532794443L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1411935234L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)2935035326L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)148, (uint8_t)74, (uint8_t)134, (uint8_t)89, (uint8_t)88, (uint8_t)6, (uint8_t)176, (uint8_t)42};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)213, (uint8_t)251, (uint8_t)182, (uint8_t)7, (uint8_t)104, (uint8_t)71, (uint8_t)120, (uint8_t)176};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)187, (uint8_t)155, (uint8_t)173, (uint8_t)170, (uint8_t)38, (uint8_t)119, (uint8_t)198, (uint8_t)22};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)52422, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)6353, PH.base.pack) ;
    p148_uid_SET((uint64_t)3012360017692878009L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)145, (uint8_t)42, (uint8_t)119, (uint8_t)199, (uint8_t)111, (uint8_t)204, (uint8_t)137, (uint8_t)48, (uint8_t)19, (uint8_t)90, (uint8_t)228, (uint8_t)179, (uint8_t)65, (uint8_t)48, (uint8_t)35, (uint8_t)162, (uint8_t)213, (uint8_t)224};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)468843133039996338L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p149_angle_x_SET((float)1.9365797E38F, PH.base.pack) ;
    p149_angle_y_SET((float)7.488897E37F, PH.base.pack) ;
    p149_distance_SET((float)1.0400683E38F, PH.base.pack) ;
    p149_size_x_SET((float)5.5306014E37F, PH.base.pack) ;
    p149_size_y_SET((float) -5.802479E37F, PH.base.pack) ;
    p149_x_SET((float)1.939688E38F, &PH) ;
    p149_y_SET((float)2.7868327E38F, &PH) ;
    p149_z_SET((float) -1.4454403E38F, &PH) ;
    {
        float  q [] =  {-3.227817E38F, 2.9956271E38F, 6.6979194E37F, 3.125E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)34, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CPU_LOAD_170(), &PH);
    p170_sensLoad_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p170_ctrlLoad_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p170_batVolt_SET((uint16_t)(uint16_t)46165, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENSOR_BIAS_172(), &PH);
    p172_axBias_SET((float)1.9570292E37F, PH.base.pack) ;
    p172_ayBias_SET((float) -3.1251699E38F, PH.base.pack) ;
    p172_azBias_SET((float)5.082292E37F, PH.base.pack) ;
    p172_gxBias_SET((float)2.0336735E37F, PH.base.pack) ;
    p172_gyBias_SET((float) -1.0052259E38F, PH.base.pack) ;
    p172_gzBias_SET((float)1.6087611E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DIAGNOSTIC_173(), &PH);
    p173_diagFl1_SET((float) -2.7019196E38F, PH.base.pack) ;
    p173_diagFl2_SET((float) -2.2005325E38F, PH.base.pack) ;
    p173_diagFl3_SET((float)3.518465E37F, PH.base.pack) ;
    p173_diagSh1_SET((int16_t)(int16_t) -1198, PH.base.pack) ;
    p173_diagSh2_SET((int16_t)(int16_t) -21066, PH.base.pack) ;
    p173_diagSh3_SET((int16_t)(int16_t) -25270, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SLUGS_NAVIGATION_176(), &PH);
    p176_u_m_SET((float) -1.7769549E38F, PH.base.pack) ;
    p176_phi_c_SET((float)7.92009E37F, PH.base.pack) ;
    p176_theta_c_SET((float)9.04791E37F, PH.base.pack) ;
    p176_psiDot_c_SET((float)2.0886635E38F, PH.base.pack) ;
    p176_ay_body_SET((float)2.0005269E38F, PH.base.pack) ;
    p176_totalDist_SET((float) -2.470639E38F, PH.base.pack) ;
    p176_dist2Go_SET((float)1.720753E38F, PH.base.pack) ;
    p176_fromWP_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p176_toWP_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p176_h_c_SET((uint16_t)(uint16_t)29199, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_LOG_177(), &PH);
    p177_fl_1_SET((float)2.1073965E38F, PH.base.pack) ;
    p177_fl_2_SET((float) -2.6864905E38F, PH.base.pack) ;
    p177_fl_3_SET((float) -2.1096633E38F, PH.base.pack) ;
    p177_fl_4_SET((float)2.0562747E38F, PH.base.pack) ;
    p177_fl_5_SET((float) -1.7274855E38F, PH.base.pack) ;
    p177_fl_6_SET((float)1.8875958E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_DATE_TIME_179(), &PH);
    p179_year_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p179_month_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    p179_day_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p179_hour_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p179_min_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p179_sec_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p179_clockStat_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p179_visSat_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p179_useSat_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p179_GppGl_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p179_sigUsedMask_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p179_percentUsed_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MID_LVL_CMDS_180(), &PH);
    p180_target_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p180_hCommand_SET((float)1.2063221E38F, PH.base.pack) ;
    p180_uCommand_SET((float) -3.0759707E37F, PH.base.pack) ;
    p180_rCommand_SET((float)1.1248769E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CTRL_SRFC_PT_181(), &PH);
    p181_target_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p181_bitfieldPt_SET((uint16_t)(uint16_t)6247, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SLUGS_CAMERA_ORDER_184(), &PH);
    p184_target_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    p184_pan_SET((int8_t)(int8_t)33, PH.base.pack) ;
    p184_tilt_SET((int8_t)(int8_t)51, PH.base.pack) ;
    p184_zoom_SET((int8_t)(int8_t)11, PH.base.pack) ;
    p184_moveHome_SET((int8_t)(int8_t)54, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SURFACE_185(), &PH);
    p185_target_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p185_idSurface_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p185_mControl_SET((float) -3.8615728E37F, PH.base.pack) ;
    p185_bControl_SET((float)1.3525103E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SLUGS_MOBILE_LOCATION_186(), &PH);
    p186_target_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p186_latitude_SET((float)3.1382202E38F, PH.base.pack) ;
    p186_longitude_SET((float) -3.2297127E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SLUGS_CONFIGURATION_CAMERA_188(), &PH);
    p188_target_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p188_idOrder_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p188_order_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ISR_LOCATION_189(), &PH);
    p189_target_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p189_latitude_SET((float) -5.555066E37F, PH.base.pack) ;
    p189_longitude_SET((float)6.1559547E37F, PH.base.pack) ;
    p189_height_SET((float) -8.2923293E37F, PH.base.pack) ;
    p189_option1_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p189_option2_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p189_option3_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VOLT_SENSOR_191(), &PH);
    p191_r2Type_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p191_voltage_SET((uint16_t)(uint16_t)41011, PH.base.pack) ;
    p191_reading2_SET((uint16_t)(uint16_t)24279, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PTZ_STATUS_192(), &PH);
    p192_zoom_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p192_pan_SET((int16_t)(int16_t)30483, PH.base.pack) ;
    p192_tilt_SET((int16_t)(int16_t) -15296, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAV_STATUS_193(), &PH);
    p193_target_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p193_latitude_SET((float) -1.7503888E38F, PH.base.pack) ;
    p193_longitude_SET((float)1.546702E38F, PH.base.pack) ;
    p193_altitude_SET((float) -9.496325E37F, PH.base.pack) ;
    p193_speed_SET((float)1.7727195E38F, PH.base.pack) ;
    p193_course_SET((float) -1.6718253E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUS_GPS_194(), &PH);
    p194_csFails_SET((uint16_t)(uint16_t)5475, PH.base.pack) ;
    p194_gpsQuality_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p194_msgsType_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p194_posStatus_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p194_magVar_SET((float)3.3967144E38F, PH.base.pack) ;
    p194_magDir_SET((int8_t)(int8_t)79, PH.base.pack) ;
    p194_modeInd_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NOVATEL_DIAG_195(), &PH);
    p195_timeStatus_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p195_receiverStatus_SET((uint32_t)2186018460L, PH.base.pack) ;
    p195_solStatus_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p195_posType_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p195_velType_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p195_posSolAge_SET((float) -1.784663E38F, PH.base.pack) ;
    p195_csFails_SET((uint16_t)(uint16_t)64485, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SENSOR_DIAG_196(), &PH);
    p196_float1_SET((float) -3.1285937E38F, PH.base.pack) ;
    p196_float2_SET((float)3.3990083E38F, PH.base.pack) ;
    p196_int1_SET((int16_t)(int16_t)10952, PH.base.pack) ;
    p196_char1_SET((int8_t)(int8_t)4, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BOOT_197(), &PH);
    p197_version_SET((uint32_t)2768731385L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)7048561162857022298L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS, PH.base.pack) ;
    p230_vel_ratio_SET((float)1.7489136E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)2.884761E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)1.546152E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float)3.3909883E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)1.868105E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float)2.2976999E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)1.3937228E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -1.7659215E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)432780070888383919L, PH.base.pack) ;
    p231_wind_x_SET((float) -2.0490544E38F, PH.base.pack) ;
    p231_wind_y_SET((float)2.859976E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -1.9305771E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -8.788568E37F, PH.base.pack) ;
    p231_var_vert_SET((float) -1.9149034E37F, PH.base.pack) ;
    p231_wind_alt_SET((float)1.1497217E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)5.546334E37F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)2.7485117E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)2314858932803442997L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)3869700590L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)8051, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p232_lat_SET((int32_t) -259042730, PH.base.pack) ;
    p232_lon_SET((int32_t)670260626, PH.base.pack) ;
    p232_alt_SET((float)2.129949E38F, PH.base.pack) ;
    p232_hdop_SET((float)1.729176E38F, PH.base.pack) ;
    p232_vdop_SET((float)9.007092E37F, PH.base.pack) ;
    p232_vn_SET((float)2.2474194E38F, PH.base.pack) ;
    p232_ve_SET((float)4.2141348E37F, PH.base.pack) ;
    p232_vd_SET((float) -2.1604902E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float)1.4003364E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -6.597593E37F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -2.5033619E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)237, (uint8_t)89, (uint8_t)144, (uint8_t)16, (uint8_t)46, (uint8_t)51, (uint8_t)163, (uint8_t)131, (uint8_t)86, (uint8_t)31, (uint8_t)225, (uint8_t)182, (uint8_t)179, (uint8_t)182, (uint8_t)133, (uint8_t)103, (uint8_t)29, (uint8_t)46, (uint8_t)222, (uint8_t)242, (uint8_t)190, (uint8_t)225, (uint8_t)143, (uint8_t)186, (uint8_t)125, (uint8_t)26, (uint8_t)224, (uint8_t)127, (uint8_t)120, (uint8_t)143, (uint8_t)82, (uint8_t)240, (uint8_t)118, (uint8_t)23, (uint8_t)38, (uint8_t)82, (uint8_t)139, (uint8_t)34, (uint8_t)96, (uint8_t)184, (uint8_t)88, (uint8_t)183, (uint8_t)104, (uint8_t)67, (uint8_t)66, (uint8_t)92, (uint8_t)15, (uint8_t)149, (uint8_t)199, (uint8_t)97, (uint8_t)40, (uint8_t)58, (uint8_t)142, (uint8_t)51, (uint8_t)247, (uint8_t)125, (uint8_t)101, (uint8_t)196, (uint8_t)57, (uint8_t)82, (uint8_t)132, (uint8_t)181, (uint8_t)53, (uint8_t)249, (uint8_t)75, (uint8_t)244, (uint8_t)12, (uint8_t)101, (uint8_t)173, (uint8_t)2, (uint8_t)71, (uint8_t)244, (uint8_t)13, (uint8_t)207, (uint8_t)40, (uint8_t)198, (uint8_t)158, (uint8_t)31, (uint8_t)77, (uint8_t)23, (uint8_t)115, (uint8_t)54, (uint8_t)59, (uint8_t)184, (uint8_t)172, (uint8_t)223, (uint8_t)216, (uint8_t)75, (uint8_t)194, (uint8_t)110, (uint8_t)222, (uint8_t)121, (uint8_t)139, (uint8_t)205, (uint8_t)156, (uint8_t)113, (uint8_t)112, (uint8_t)245, (uint8_t)99, (uint8_t)80, (uint8_t)183, (uint8_t)48, (uint8_t)99, (uint8_t)163, (uint8_t)120, (uint8_t)136, (uint8_t)233, (uint8_t)142, (uint8_t)105, (uint8_t)128, (uint8_t)136, (uint8_t)41, (uint8_t)173, (uint8_t)97, (uint8_t)231, (uint8_t)226, (uint8_t)192, (uint8_t)0, (uint8_t)246, (uint8_t)234, (uint8_t)90, (uint8_t)158, (uint8_t)168, (uint8_t)35, (uint8_t)92, (uint8_t)10, (uint8_t)183, (uint8_t)105, (uint8_t)100, (uint8_t)47, (uint8_t)165, (uint8_t)0, (uint8_t)135, (uint8_t)37, (uint8_t)1, (uint8_t)160, (uint8_t)142, (uint8_t)121, (uint8_t)126, (uint8_t)206, (uint8_t)93, (uint8_t)64, (uint8_t)218, (uint8_t)199, (uint8_t)215, (uint8_t)22, (uint8_t)215, (uint8_t)236, (uint8_t)17, (uint8_t)168, (uint8_t)67, (uint8_t)253, (uint8_t)48, (uint8_t)53, (uint8_t)145, (uint8_t)27, (uint8_t)71, (uint8_t)21, (uint8_t)194, (uint8_t)59, (uint8_t)126, (uint8_t)40, (uint8_t)35, (uint8_t)116, (uint8_t)193, (uint8_t)73, (uint8_t)57, (uint8_t)12, (uint8_t)148, (uint8_t)173, (uint8_t)121, (uint8_t)132, (uint8_t)1, (uint8_t)71, (uint8_t)251, (uint8_t)217, (uint8_t)45, (uint8_t)217, (uint8_t)243, (uint8_t)72};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)2026159903L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)31277, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)23634, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)22807, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -22, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -7992, PH.base.pack) ;
    p234_latitude_SET((int32_t) -2022119424, PH.base.pack) ;
    p234_longitude_SET((int32_t)193016345, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t) -29896, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)7715, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)102, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)96, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)24, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)33798, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)4853613869904135722L, PH.base.pack) ;
    p241_vibration_x_SET((float)2.9874385E38F, PH.base.pack) ;
    p241_vibration_y_SET((float)2.9982064E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)4.7280452E36F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)2280818925L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)294090111L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)730846760L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)1094363176, PH.base.pack) ;
    p242_longitude_SET((int32_t)2029281191, PH.base.pack) ;
    p242_altitude_SET((int32_t)337973780, PH.base.pack) ;
    p242_x_SET((float) -6.913188E37F, PH.base.pack) ;
    p242_y_SET((float) -1.924647E38F, PH.base.pack) ;
    p242_z_SET((float)2.2182944E38F, PH.base.pack) ;
    {
        float  q [] =  {2.5792897E38F, 3.2970385E38F, -1.3010421E38F, -1.0078856E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -1.934327E38F, PH.base.pack) ;
    p242_approach_y_SET((float) -1.5996046E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -7.96692E37F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)8572574967815908851L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p243_latitude_SET((int32_t)479481751, PH.base.pack) ;
    p243_longitude_SET((int32_t)367048674, PH.base.pack) ;
    p243_altitude_SET((int32_t)1111353684, PH.base.pack) ;
    p243_x_SET((float)2.793345E38F, PH.base.pack) ;
    p243_y_SET((float)1.3236415E38F, PH.base.pack) ;
    p243_z_SET((float)1.8676999E37F, PH.base.pack) ;
    {
        float  q [] =  {1.3723874E37F, -1.516985E38F, -3.020856E38F, -2.625992E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -1.4064604E38F, PH.base.pack) ;
    p243_approach_y_SET((float) -1.208034E38F, PH.base.pack) ;
    p243_approach_z_SET((float)3.0596656E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)3793739389749488349L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)6290, PH.base.pack) ;
    p244_interval_us_SET((int32_t)191975690, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)2975797796L, PH.base.pack) ;
    p246_lat_SET((int32_t) -432960953, PH.base.pack) ;
    p246_lon_SET((int32_t)1870846747, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t) -1704057536, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)30363, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)57688, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -11396, PH.base.pack) ;
    {
        char16_t   callsign = "gzuRhs";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)3208, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
    p247_id_SET((uint32_t)959661761L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float) -1.213825E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -2.6403917E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float) -3.2123036E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)20053, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)48, (uint8_t)118, (uint8_t)61, (uint8_t)120, (uint8_t)13, (uint8_t)83, (uint8_t)111, (uint8_t)179, (uint8_t)213, (uint8_t)179, (uint8_t)172, (uint8_t)155, (uint8_t)157, (uint8_t)242, (uint8_t)55, (uint8_t)138, (uint8_t)86, (uint8_t)177, (uint8_t)74, (uint8_t)249, (uint8_t)97, (uint8_t)203, (uint8_t)34, (uint8_t)7, (uint8_t)85, (uint8_t)18, (uint8_t)117, (uint8_t)60, (uint8_t)11, (uint8_t)246, (uint8_t)85, (uint8_t)177, (uint8_t)110, (uint8_t)117, (uint8_t)63, (uint8_t)160, (uint8_t)92, (uint8_t)136, (uint8_t)206, (uint8_t)239, (uint8_t)247, (uint8_t)243, (uint8_t)136, (uint8_t)101, (uint8_t)78, (uint8_t)138, (uint8_t)147, (uint8_t)29, (uint8_t)33, (uint8_t)153, (uint8_t)226, (uint8_t)151, (uint8_t)179, (uint8_t)234, (uint8_t)136, (uint8_t)77, (uint8_t)228, (uint8_t)249, (uint8_t)151, (uint8_t)204, (uint8_t)62, (uint8_t)168, (uint8_t)170, (uint8_t)64, (uint8_t)191, (uint8_t)57, (uint8_t)108, (uint8_t)165, (uint8_t)128, (uint8_t)104, (uint8_t)115, (uint8_t)244, (uint8_t)199, (uint8_t)232, (uint8_t)60, (uint8_t)253, (uint8_t)218, (uint8_t)30, (uint8_t)94, (uint8_t)156, (uint8_t)155, (uint8_t)42, (uint8_t)52, (uint8_t)243, (uint8_t)47, (uint8_t)205, (uint8_t)177, (uint8_t)106, (uint8_t)128, (uint8_t)177, (uint8_t)94, (uint8_t)198, (uint8_t)110, (uint8_t)84, (uint8_t)148, (uint8_t)89, (uint8_t)254, (uint8_t)51, (uint8_t)200, (uint8_t)193, (uint8_t)215, (uint8_t)141, (uint8_t)52, (uint8_t)211, (uint8_t)214, (uint8_t)102, (uint8_t)189, (uint8_t)172, (uint8_t)250, (uint8_t)252, (uint8_t)184, (uint8_t)161, (uint8_t)252, (uint8_t)202, (uint8_t)200, (uint8_t)126, (uint8_t)152, (uint8_t)160, (uint8_t)78, (uint8_t)106, (uint8_t)91, (uint8_t)206, (uint8_t)5, (uint8_t)221, (uint8_t)210, (uint8_t)3, (uint8_t)233, (uint8_t)91, (uint8_t)55, (uint8_t)201, (uint8_t)56, (uint8_t)97, (uint8_t)234, (uint8_t)125, (uint8_t)132, (uint8_t)253, (uint8_t)49, (uint8_t)238, (uint8_t)11, (uint8_t)114, (uint8_t)134, (uint8_t)193, (uint8_t)167, (uint8_t)232, (uint8_t)146, (uint8_t)84, (uint8_t)129, (uint8_t)103, (uint8_t)228, (uint8_t)40, (uint8_t)99, (uint8_t)8, (uint8_t)39, (uint8_t)32, (uint8_t)7, (uint8_t)67, (uint8_t)124, (uint8_t)63, (uint8_t)251, (uint8_t)203, (uint8_t)237, (uint8_t)3, (uint8_t)237, (uint8_t)157, (uint8_t)213, (uint8_t)75, (uint8_t)26, (uint8_t)93, (uint8_t)185, (uint8_t)31, (uint8_t)193, (uint8_t)109, (uint8_t)229, (uint8_t)202, (uint8_t)178, (uint8_t)126, (uint8_t)86, (uint8_t)175, (uint8_t)32, (uint8_t)122, (uint8_t)23, (uint8_t)70, (uint8_t)192, (uint8_t)134, (uint8_t)0, (uint8_t)14, (uint8_t)12, (uint8_t)50, (uint8_t)92, (uint8_t)110, (uint8_t)111, (uint8_t)232, (uint8_t)83, (uint8_t)251, (uint8_t)76, (uint8_t)95, (uint8_t)193, (uint8_t)231, (uint8_t)246, (uint8_t)106, (uint8_t)7, (uint8_t)193, (uint8_t)192, (uint8_t)25, (uint8_t)198, (uint8_t)127, (uint8_t)243, (uint8_t)88, (uint8_t)26, (uint8_t)39, (uint8_t)72, (uint8_t)33, (uint8_t)180, (uint8_t)79, (uint8_t)236, (uint8_t)62, (uint8_t)43, (uint8_t)133, (uint8_t)103, (uint8_t)84, (uint8_t)209, (uint8_t)218, (uint8_t)113, (uint8_t)105, (uint8_t)47, (uint8_t)175, (uint8_t)27, (uint8_t)18, (uint8_t)219, (uint8_t)198, (uint8_t)163, (uint8_t)106, (uint8_t)29, (uint8_t)68, (uint8_t)136, (uint8_t)108, (uint8_t)178, (uint8_t)185, (uint8_t)180, (uint8_t)135, (uint8_t)122, (uint8_t)150, (uint8_t)115, (uint8_t)53, (uint8_t)91, (uint8_t)74, (uint8_t)27, (uint8_t)255, (uint8_t)144};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)61179, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)46, (int8_t) -79, (int8_t) -47, (int8_t) -110, (int8_t) -88, (int8_t) -92, (int8_t) -20, (int8_t) -11, (int8_t) -61, (int8_t) -98, (int8_t)23, (int8_t) -114, (int8_t)22, (int8_t) -80, (int8_t)51, (int8_t)50, (int8_t)77, (int8_t) -109, (int8_t) -113, (int8_t)118, (int8_t) -4, (int8_t) -13, (int8_t)104, (int8_t)101, (int8_t) -59, (int8_t)84, (int8_t)33, (int8_t) -8, (int8_t) -16, (int8_t)80, (int8_t)30, (int8_t)72};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "olidnuaajw";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)7621237997266755818L, PH.base.pack) ;
    p250_x_SET((float)7.143035E37F, PH.base.pack) ;
    p250_y_SET((float) -2.3689925E38F, PH.base.pack) ;
    p250_z_SET((float) -2.7983022E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)450711374L, PH.base.pack) ;
    {
        char16_t   name = "mu";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)3.4000851E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)4123804095L, PH.base.pack) ;
    {
        char16_t   name = "kcvnqke";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -123218438, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, PH.base.pack) ;
    {
        char16_t   text = "jsqaIjdxquT";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)2527513085L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p254_value_SET((float)2.7513427E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)128, (uint8_t)124, (uint8_t)49, (uint8_t)114, (uint8_t)182, (uint8_t)45, (uint8_t)200, (uint8_t)166, (uint8_t)179, (uint8_t)90, (uint8_t)125, (uint8_t)69, (uint8_t)38, (uint8_t)124, (uint8_t)172, (uint8_t)1, (uint8_t)51, (uint8_t)120, (uint8_t)54, (uint8_t)41, (uint8_t)29, (uint8_t)222, (uint8_t)86, (uint8_t)155, (uint8_t)39, (uint8_t)243, (uint8_t)204, (uint8_t)30, (uint8_t)152, (uint8_t)165, (uint8_t)111, (uint8_t)239};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)4532117452172353123L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)10216669L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)2416575394L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    {
        char16_t   tune = "XEyiRpeyFdlphcdskdvedimiaei";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)3397857702L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)64, (uint8_t)98, (uint8_t)235, (uint8_t)177, (uint8_t)121, (uint8_t)113, (uint8_t)251, (uint8_t)49, (uint8_t)183, (uint8_t)77, (uint8_t)76, (uint8_t)151, (uint8_t)44, (uint8_t)236, (uint8_t)15, (uint8_t)39, (uint8_t)191, (uint8_t)58, (uint8_t)128, (uint8_t)198, (uint8_t)121, (uint8_t)44, (uint8_t)134, (uint8_t)90, (uint8_t)49, (uint8_t)139, (uint8_t)164, (uint8_t)79, (uint8_t)2, (uint8_t)50, (uint8_t)5, (uint8_t)156};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)121, (uint8_t)155, (uint8_t)112, (uint8_t)238, (uint8_t)211, (uint8_t)59, (uint8_t)27, (uint8_t)183, (uint8_t)81, (uint8_t)151, (uint8_t)253, (uint8_t)182, (uint8_t)197, (uint8_t)110, (uint8_t)206, (uint8_t)231, (uint8_t)194, (uint8_t)19, (uint8_t)91, (uint8_t)41, (uint8_t)47, (uint8_t)177, (uint8_t)24, (uint8_t)118, (uint8_t)30, (uint8_t)18, (uint8_t)133, (uint8_t)107, (uint8_t)150, (uint8_t)125, (uint8_t)116, (uint8_t)121};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1227614443L, PH.base.pack) ;
    p259_focal_length_SET((float)1.2333806E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -1.940516E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -3.2550335E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)10251, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)9121, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)11401, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "yxipbeafyxu";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)2396560562L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)545339339L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p261_total_capacity_SET((float)2.26341E38F, PH.base.pack) ;
    p261_used_capacity_SET((float) -1.5250574E38F, PH.base.pack) ;
    p261_available_capacity_SET((float) -2.2134356E38F, PH.base.pack) ;
    p261_read_speed_SET((float)4.2027277E37F, PH.base.pack) ;
    p261_write_speed_SET((float) -1.716693E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)3123190039L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p262_image_interval_SET((float) -4.2019375E37F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)809311580L, PH.base.pack) ;
    p262_available_capacity_SET((float) -2.36925E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)4179487614L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)3367738285070437293L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p263_lat_SET((int32_t) -1414950979, PH.base.pack) ;
    p263_lon_SET((int32_t)651633775, PH.base.pack) ;
    p263_alt_SET((int32_t) -1133243120, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)710949048, PH.base.pack) ;
    {
        float  q [] =  {-2.926868E38F, -6.62502E37F, 2.3590016E38F, 2.7379523E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1255594637, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)79, PH.base.pack) ;
    {
        char16_t   file_url = "dtrrcbeuwvcXfgvibgkasarnoAtphcfkqfeVxbcadgiNttnyhqcxibglpwwcaHogqbiaiqdudjayetsnev";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)456903881L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)3138231146171916544L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)1519013269530195788L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)4928487227061021186L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)2142584610L, PH.base.pack) ;
    p265_roll_SET((float) -2.8574938E38F, PH.base.pack) ;
    p265_pitch_SET((float)2.4614838E38F, PH.base.pack) ;
    p265_yaw_SET((float) -6.549951E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)38608, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)218, (uint8_t)200, (uint8_t)62, (uint8_t)141, (uint8_t)65, (uint8_t)62, (uint8_t)96, (uint8_t)253, (uint8_t)182, (uint8_t)5, (uint8_t)121, (uint8_t)7, (uint8_t)181, (uint8_t)204, (uint8_t)154, (uint8_t)189, (uint8_t)203, (uint8_t)146, (uint8_t)147, (uint8_t)182, (uint8_t)31, (uint8_t)106, (uint8_t)242, (uint8_t)34, (uint8_t)76, (uint8_t)237, (uint8_t)21, (uint8_t)91, (uint8_t)243, (uint8_t)35, (uint8_t)244, (uint8_t)41, (uint8_t)246, (uint8_t)182, (uint8_t)199, (uint8_t)57, (uint8_t)102, (uint8_t)18, (uint8_t)45, (uint8_t)174, (uint8_t)166, (uint8_t)102, (uint8_t)85, (uint8_t)109, (uint8_t)105, (uint8_t)150, (uint8_t)146, (uint8_t)8, (uint8_t)63, (uint8_t)36, (uint8_t)208, (uint8_t)194, (uint8_t)132, (uint8_t)205, (uint8_t)91, (uint8_t)66, (uint8_t)210, (uint8_t)227, (uint8_t)47, (uint8_t)14, (uint8_t)210, (uint8_t)49, (uint8_t)212, (uint8_t)2, (uint8_t)68, (uint8_t)112, (uint8_t)159, (uint8_t)169, (uint8_t)176, (uint8_t)92, (uint8_t)27, (uint8_t)167, (uint8_t)148, (uint8_t)131, (uint8_t)72, (uint8_t)170, (uint8_t)109, (uint8_t)57, (uint8_t)230, (uint8_t)141, (uint8_t)75, (uint8_t)131, (uint8_t)98, (uint8_t)123, (uint8_t)21, (uint8_t)1, (uint8_t)185, (uint8_t)172, (uint8_t)75, (uint8_t)84, (uint8_t)138, (uint8_t)183, (uint8_t)134, (uint8_t)255, (uint8_t)5, (uint8_t)159, (uint8_t)51, (uint8_t)145, (uint8_t)207, (uint8_t)223, (uint8_t)254, (uint8_t)146, (uint8_t)20, (uint8_t)2, (uint8_t)76, (uint8_t)19, (uint8_t)8, (uint8_t)58, (uint8_t)100, (uint8_t)29, (uint8_t)229, (uint8_t)82, (uint8_t)121, (uint8_t)161, (uint8_t)157, (uint8_t)176, (uint8_t)43, (uint8_t)193, (uint8_t)29, (uint8_t)209, (uint8_t)155, (uint8_t)184, (uint8_t)45, (uint8_t)234, (uint8_t)111, (uint8_t)95, (uint8_t)24, (uint8_t)242, (uint8_t)171, (uint8_t)126, (uint8_t)50, (uint8_t)44, (uint8_t)43, (uint8_t)161, (uint8_t)102, (uint8_t)74, (uint8_t)251, (uint8_t)254, (uint8_t)85, (uint8_t)21, (uint8_t)239, (uint8_t)56, (uint8_t)138, (uint8_t)101, (uint8_t)12, (uint8_t)161, (uint8_t)86, (uint8_t)125, (uint8_t)149, (uint8_t)37, (uint8_t)124, (uint8_t)162, (uint8_t)122, (uint8_t)188, (uint8_t)90, (uint8_t)91, (uint8_t)254, (uint8_t)35, (uint8_t)235, (uint8_t)115, (uint8_t)58, (uint8_t)128, (uint8_t)16, (uint8_t)141, (uint8_t)211, (uint8_t)254, (uint8_t)160, (uint8_t)204, (uint8_t)145, (uint8_t)74, (uint8_t)64, (uint8_t)172, (uint8_t)37, (uint8_t)137, (uint8_t)236, (uint8_t)98, (uint8_t)20, (uint8_t)145, (uint8_t)42, (uint8_t)177, (uint8_t)19, (uint8_t)63, (uint8_t)245, (uint8_t)173, (uint8_t)93, (uint8_t)43, (uint8_t)128, (uint8_t)248, (uint8_t)123, (uint8_t)7, (uint8_t)164, (uint8_t)212, (uint8_t)93, (uint8_t)141, (uint8_t)184, (uint8_t)145, (uint8_t)64, (uint8_t)123, (uint8_t)145, (uint8_t)156, (uint8_t)226, (uint8_t)238, (uint8_t)46, (uint8_t)196, (uint8_t)21, (uint8_t)136, (uint8_t)28, (uint8_t)26, (uint8_t)84, (uint8_t)167, (uint8_t)82, (uint8_t)243, (uint8_t)251, (uint8_t)117, (uint8_t)22, (uint8_t)236, (uint8_t)33, (uint8_t)122, (uint8_t)196, (uint8_t)152, (uint8_t)150, (uint8_t)248, (uint8_t)99, (uint8_t)4, (uint8_t)202, (uint8_t)56, (uint8_t)148, (uint8_t)43, (uint8_t)39, (uint8_t)196, (uint8_t)132, (uint8_t)96, (uint8_t)250, (uint8_t)40, (uint8_t)34, (uint8_t)70, (uint8_t)1, (uint8_t)88, (uint8_t)160, (uint8_t)46, (uint8_t)81, (uint8_t)205, (uint8_t)150, (uint8_t)204, (uint8_t)158, (uint8_t)14, (uint8_t)143, (uint8_t)20, (uint8_t)102};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)20428, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)40, (uint8_t)236, (uint8_t)160, (uint8_t)105, (uint8_t)2, (uint8_t)213, (uint8_t)161, (uint8_t)33, (uint8_t)112, (uint8_t)187, (uint8_t)20, (uint8_t)58, (uint8_t)135, (uint8_t)137, (uint8_t)229, (uint8_t)1, (uint8_t)2, (uint8_t)162, (uint8_t)23, (uint8_t)45, (uint8_t)90, (uint8_t)109, (uint8_t)244, (uint8_t)99, (uint8_t)241, (uint8_t)170, (uint8_t)201, (uint8_t)235, (uint8_t)21, (uint8_t)139, (uint8_t)211, (uint8_t)216, (uint8_t)122, (uint8_t)239, (uint8_t)25, (uint8_t)216, (uint8_t)2, (uint8_t)18, (uint8_t)4, (uint8_t)128, (uint8_t)41, (uint8_t)211, (uint8_t)231, (uint8_t)116, (uint8_t)1, (uint8_t)55, (uint8_t)80, (uint8_t)202, (uint8_t)111, (uint8_t)102, (uint8_t)31, (uint8_t)94, (uint8_t)177, (uint8_t)102, (uint8_t)232, (uint8_t)232, (uint8_t)115, (uint8_t)99, (uint8_t)65, (uint8_t)9, (uint8_t)237, (uint8_t)235, (uint8_t)35, (uint8_t)191, (uint8_t)158, (uint8_t)116, (uint8_t)237, (uint8_t)9, (uint8_t)255, (uint8_t)2, (uint8_t)104, (uint8_t)39, (uint8_t)56, (uint8_t)32, (uint8_t)234, (uint8_t)175, (uint8_t)171, (uint8_t)61, (uint8_t)214, (uint8_t)181, (uint8_t)225, (uint8_t)61, (uint8_t)76, (uint8_t)139, (uint8_t)125, (uint8_t)218, (uint8_t)239, (uint8_t)210, (uint8_t)68, (uint8_t)226, (uint8_t)116, (uint8_t)32, (uint8_t)119, (uint8_t)56, (uint8_t)231, (uint8_t)208, (uint8_t)54, (uint8_t)185, (uint8_t)175, (uint8_t)6, (uint8_t)210, (uint8_t)52, (uint8_t)187, (uint8_t)204, (uint8_t)69, (uint8_t)121, (uint8_t)84, (uint8_t)133, (uint8_t)82, (uint8_t)190, (uint8_t)177, (uint8_t)25, (uint8_t)200, (uint8_t)255, (uint8_t)228, (uint8_t)203, (uint8_t)89, (uint8_t)70, (uint8_t)226, (uint8_t)58, (uint8_t)102, (uint8_t)66, (uint8_t)107, (uint8_t)156, (uint8_t)15, (uint8_t)69, (uint8_t)71, (uint8_t)210, (uint8_t)185, (uint8_t)81, (uint8_t)179, (uint8_t)162, (uint8_t)65, (uint8_t)168, (uint8_t)27, (uint8_t)218, (uint8_t)37, (uint8_t)210, (uint8_t)216, (uint8_t)206, (uint8_t)59, (uint8_t)42, (uint8_t)145, (uint8_t)95, (uint8_t)125, (uint8_t)55, (uint8_t)115, (uint8_t)40, (uint8_t)51, (uint8_t)164, (uint8_t)199, (uint8_t)161, (uint8_t)250, (uint8_t)44, (uint8_t)209, (uint8_t)54, (uint8_t)108, (uint8_t)46, (uint8_t)180, (uint8_t)61, (uint8_t)184, (uint8_t)87, (uint8_t)251, (uint8_t)74, (uint8_t)123, (uint8_t)246, (uint8_t)255, (uint8_t)161, (uint8_t)29, (uint8_t)172, (uint8_t)96, (uint8_t)33, (uint8_t)73, (uint8_t)217, (uint8_t)236, (uint8_t)240, (uint8_t)61, (uint8_t)92, (uint8_t)216, (uint8_t)48, (uint8_t)101, (uint8_t)44, (uint8_t)132, (uint8_t)102, (uint8_t)231, (uint8_t)154, (uint8_t)78, (uint8_t)70, (uint8_t)153, (uint8_t)8, (uint8_t)192, (uint8_t)55, (uint8_t)121, (uint8_t)10, (uint8_t)102, (uint8_t)64, (uint8_t)167, (uint8_t)204, (uint8_t)41, (uint8_t)126, (uint8_t)82, (uint8_t)26, (uint8_t)243, (uint8_t)76, (uint8_t)135, (uint8_t)15, (uint8_t)133, (uint8_t)37, (uint8_t)200, (uint8_t)118, (uint8_t)171, (uint8_t)18, (uint8_t)247, (uint8_t)23, (uint8_t)117, (uint8_t)186, (uint8_t)36, (uint8_t)133, (uint8_t)245, (uint8_t)95, (uint8_t)249, (uint8_t)40, (uint8_t)193, (uint8_t)171, (uint8_t)77, (uint8_t)74, (uint8_t)162, (uint8_t)68, (uint8_t)199, (uint8_t)29, (uint8_t)75, (uint8_t)128, (uint8_t)243, (uint8_t)5, (uint8_t)104, (uint8_t)116, (uint8_t)195, (uint8_t)54, (uint8_t)218, (uint8_t)136, (uint8_t)49, (uint8_t)70, (uint8_t)249, (uint8_t)249, (uint8_t)225, (uint8_t)48, (uint8_t)247, (uint8_t)203, (uint8_t)146};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)60651, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    p269_framerate_SET((float)2.87543E38F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)6489, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)35024, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)426594956L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)9638, PH.base.pack) ;
    {
        char16_t   uri = "nmgjtqgglmornEsoepyehdtAlrnuztvikjacvoulWxypamvZbsdgbwlhlvbzilddbcTbYcnmhcdcPKyx";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p270_framerate_SET((float)2.4284636E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)24744, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)24564, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)1278017175L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)60710, PH.base.pack) ;
    {
        char16_t   uri = "ljqxtoqikofnKaocaprlznzbvPRvdmntrpfvzewtHoakiWIhruvgmbqpdtpteIuutsddhyyfuswephhajubjxmtwkzpdvqbemwwngcnathohxvrifciemjhhtnixfqchbbktYwpbpo";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "sbasokumpazovyvQvbrcoi";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "acjsutmdfozijuwxcvboWsy";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)16674, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)49840, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)2920, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)250, (uint8_t)49, (uint8_t)53, (uint8_t)35, (uint8_t)65, (uint8_t)201, (uint8_t)118, (uint8_t)143};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)106, (uint8_t)110, (uint8_t)50, (uint8_t)3, (uint8_t)144, (uint8_t)240, (uint8_t)198, (uint8_t)62};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)6382059375830457323L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)1471445538L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)55653, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)7127330892752533951L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)2733182451L, PH.base.pack) ;
    {
        char16_t   name = "rqimnyPBedjoccwxicxyiobswmzauihzeehfoyfdCqsGbSubuyvjtxiglotayoxjoptynebkjadbnay";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)71, (uint8_t)181, (uint8_t)149, (uint8_t)66, (uint8_t)203, (uint8_t)38, (uint8_t)71, (uint8_t)150, (uint8_t)90, (uint8_t)182, (uint8_t)195, (uint8_t)19, (uint8_t)67, (uint8_t)141, (uint8_t)193, (uint8_t)122};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)1114772410L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    {
        char16_t   param_id = "qmuztg";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t)5964, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "gufatPxsOhBM";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "YAixtpqouwSjuWOvxgwgldmssnvlmcsutUo";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)13779, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)49571, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    {
        char16_t   param_id = "zieUqylpLhjd";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "fgemwwvthuaaJHzntIAazhjxXzmlkqgtcwb";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "xdsrfrkzeOs";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "kcQDwmqunrnuewOgOpoocoznaGkbrtkhszrslvBvpdznhxxksjyejfTyp";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)5853624930596020343L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)46536, (uint16_t)26373, (uint16_t)19317, (uint16_t)30983, (uint16_t)60147, (uint16_t)40872, (uint16_t)32707, (uint16_t)31862, (uint16_t)40457, (uint16_t)21516, (uint16_t)43863, (uint16_t)17189, (uint16_t)35243, (uint16_t)9474, (uint16_t)34532, (uint16_t)9267, (uint16_t)55676, (uint16_t)17353, (uint16_t)58777, (uint16_t)45633, (uint16_t)60152, (uint16_t)40587, (uint16_t)3416, (uint16_t)60079, (uint16_t)55163, (uint16_t)54925, (uint16_t)30833, (uint16_t)56645, (uint16_t)30451, (uint16_t)65387, (uint16_t)18190, (uint16_t)10703, (uint16_t)31722, (uint16_t)45884, (uint16_t)1756, (uint16_t)5309, (uint16_t)4147, (uint16_t)26632, (uint16_t)29189, (uint16_t)64692, (uint16_t)25274, (uint16_t)48552, (uint16_t)41928, (uint16_t)14191, (uint16_t)64376, (uint16_t)51765, (uint16_t)63948, (uint16_t)58533, (uint16_t)53552, (uint16_t)53511, (uint16_t)62370, (uint16_t)20782, (uint16_t)36953, (uint16_t)34213, (uint16_t)1233, (uint16_t)20549, (uint16_t)1062, (uint16_t)11735, (uint16_t)3966, (uint16_t)65427, (uint16_t)48461, (uint16_t)22031, (uint16_t)3795, (uint16_t)62662, (uint16_t)14486, (uint16_t)9959, (uint16_t)11985, (uint16_t)1193, (uint16_t)18579, (uint16_t)30867, (uint16_t)9091, (uint16_t)39449};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)23913, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)54408, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

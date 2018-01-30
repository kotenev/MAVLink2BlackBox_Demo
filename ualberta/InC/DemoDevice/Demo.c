
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
void c_LoopBackDemoChannel_on_NAV_FILTER_BIAS_220(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p220_usec_GET(pack);
    float  accel_0 = p220_accel_0_GET(pack);
    float  accel_1 = p220_accel_1_GET(pack);
    float  accel_2 = p220_accel_2_GET(pack);
    float  gyro_0 = p220_gyro_0_GET(pack);
    float  gyro_1 = p220_gyro_1_GET(pack);
    float  gyro_2 = p220_gyro_2_GET(pack);
}
void c_LoopBackDemoChannel_on_RADIO_CALIBRATION_221(Bounds_Inside * ph, Pack * pack)
{
    uint16_t*  aileron = p221_aileron_GET_(pack);
//process data in aileron
    free(aileron);//never forget to dispose
    uint16_t*  elevator = p221_elevator_GET_(pack);
//process data in elevator
    free(elevator);//never forget to dispose
    uint16_t*  rudder = p221_rudder_GET_(pack);
//process data in rudder
    free(rudder);//never forget to dispose
    uint16_t*  gyro = p221_gyro_GET_(pack);
//process data in gyro
    free(gyro);//never forget to dispose
    uint16_t*  pitch = p221_pitch_GET_(pack);
//process data in pitch
    free(pitch);//never forget to dispose
    uint16_t*  throttle = p221_throttle_GET_(pack);
//process data in throttle
    free(throttle);//never forget to dispose
}
void c_LoopBackDemoChannel_on_UALBERTA_SYS_STATUS_222(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  mode = p222_mode_GET(pack);
    uint8_t  nav_mode = p222_nav_mode_GET(pack);
    uint8_t  pilot = p222_pilot_GET(pack);
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
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_ASLUAV, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)3704074269L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_STANDBY, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)34978, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)28986, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)16878, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -92, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)46333, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)55794, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)14876, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)3665, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)36407, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)10576, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)7221546478520915297L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)4224591071L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)1026685715L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)913, PH.base.pack) ;
    p3_x_SET((float) -2.9592675E38F, PH.base.pack) ;
    p3_y_SET((float)1.5909014E38F, PH.base.pack) ;
    p3_z_SET((float) -3.1357761E38F, PH.base.pack) ;
    p3_vx_SET((float)3.0454545E38F, PH.base.pack) ;
    p3_vy_SET((float)1.9732813E38F, PH.base.pack) ;
    p3_vz_SET((float) -8.244595E37F, PH.base.pack) ;
    p3_afx_SET((float) -2.2466012E38F, PH.base.pack) ;
    p3_afy_SET((float)2.0471204E38F, PH.base.pack) ;
    p3_afz_SET((float) -2.7503997E37F, PH.base.pack) ;
    p3_yaw_SET((float) -1.6948398E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -3.3434875E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)3711760683376099290L, PH.base.pack) ;
    p4_seq_SET((uint32_t)23612869L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    {
        char16_t   passkey = "zizUxgywsiucHev";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "cvnrbpjoYuldjmrski";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)182995678L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    {
        char16_t   param_id = "srryppgsurxzwau";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)31848, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "li";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -6.0990144E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)14800, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)41026, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    {
        char16_t   param_id = "lBeNanihyt";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)3.2819233E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)1103559766890997461L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
    p24_lat_SET((int32_t) -1473853603, PH.base.pack) ;
    p24_lon_SET((int32_t)1424346992, PH.base.pack) ;
    p24_alt_SET((int32_t) -1217715572, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)57557, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)31597, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)28937, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)13586, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1306869741, &PH) ;
    p24_h_acc_SET((uint32_t)2374910532L, &PH) ;
    p24_v_acc_SET((uint32_t)1519202641L, &PH) ;
    p24_vel_acc_SET((uint32_t)3427693053L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1038574557L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)133, (uint8_t)113, (uint8_t)65, (uint8_t)56, (uint8_t)137, (uint8_t)158, (uint8_t)198, (uint8_t)25, (uint8_t)22, (uint8_t)119, (uint8_t)24, (uint8_t)159, (uint8_t)156, (uint8_t)89, (uint8_t)162, (uint8_t)66, (uint8_t)61, (uint8_t)145, (uint8_t)12, (uint8_t)220};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)91, (uint8_t)61, (uint8_t)94, (uint8_t)239, (uint8_t)162, (uint8_t)136, (uint8_t)73, (uint8_t)112, (uint8_t)44, (uint8_t)162, (uint8_t)127, (uint8_t)74, (uint8_t)32, (uint8_t)171, (uint8_t)13, (uint8_t)148, (uint8_t)71, (uint8_t)151, (uint8_t)175, (uint8_t)201};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)184, (uint8_t)191, (uint8_t)127, (uint8_t)138, (uint8_t)56, (uint8_t)42, (uint8_t)220, (uint8_t)183, (uint8_t)223, (uint8_t)166, (uint8_t)71, (uint8_t)45, (uint8_t)240, (uint8_t)118, (uint8_t)103, (uint8_t)94, (uint8_t)154, (uint8_t)58, (uint8_t)221, (uint8_t)7};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)63, (uint8_t)222, (uint8_t)93, (uint8_t)175, (uint8_t)170, (uint8_t)61, (uint8_t)203, (uint8_t)53, (uint8_t)51, (uint8_t)63, (uint8_t)183, (uint8_t)125, (uint8_t)146, (uint8_t)215, (uint8_t)137, (uint8_t)151, (uint8_t)72, (uint8_t)73, (uint8_t)151, (uint8_t)155};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)140, (uint8_t)176, (uint8_t)39, (uint8_t)208, (uint8_t)21, (uint8_t)44, (uint8_t)241, (uint8_t)70, (uint8_t)147, (uint8_t)0, (uint8_t)51, (uint8_t)253, (uint8_t)1, (uint8_t)187, (uint8_t)90, (uint8_t)249, (uint8_t)83, (uint8_t)157, (uint8_t)185, (uint8_t)54};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3085726428L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -6838, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)10483, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)32427, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -27272, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)8282, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)5700, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)23981, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -13630, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)18200, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)3943396125557436891L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)15632, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -16106, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -23541, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)6637, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)3761, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)25821, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -15460, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -9712, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -18626, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)884084126891375739L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)28918, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)11455, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -7764, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -32633, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2015081930L, PH.base.pack) ;
    p29_press_abs_SET((float) -1.7355337E38F, PH.base.pack) ;
    p29_press_diff_SET((float)1.3695158E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -29491, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)86886200L, PH.base.pack) ;
    p30_roll_SET((float) -3.3340489E38F, PH.base.pack) ;
    p30_pitch_SET((float)2.8598895E38F, PH.base.pack) ;
    p30_yaw_SET((float)2.65589E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -3.6982132E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -3.2579708E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -2.390628E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)3882103909L, PH.base.pack) ;
    p31_q1_SET((float) -9.763935E37F, PH.base.pack) ;
    p31_q2_SET((float) -2.7305904E38F, PH.base.pack) ;
    p31_q3_SET((float) -8.1783285E37F, PH.base.pack) ;
    p31_q4_SET((float)3.4534643E36F, PH.base.pack) ;
    p31_rollspeed_SET((float) -1.7633624E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)2.076885E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.2021498E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1607481466L, PH.base.pack) ;
    p32_x_SET((float) -1.7551653E38F, PH.base.pack) ;
    p32_y_SET((float) -3.2085987E38F, PH.base.pack) ;
    p32_z_SET((float) -2.8761067E38F, PH.base.pack) ;
    p32_vx_SET((float) -1.0360813E38F, PH.base.pack) ;
    p32_vy_SET((float)7.828289E37F, PH.base.pack) ;
    p32_vz_SET((float)3.2653797E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)803044701L, PH.base.pack) ;
    p33_lat_SET((int32_t) -952514924, PH.base.pack) ;
    p33_lon_SET((int32_t)754736095, PH.base.pack) ;
    p33_alt_SET((int32_t) -2113532147, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -284168068, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)4366, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)27626, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -30678, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)63018, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)1233094486L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)15215, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -15889, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)30160, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)18196, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)19761, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)20081, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -26097, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -12338, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)1978058060L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)53403, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)6700, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)40772, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)63357, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)59553, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)10734, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)28709, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)49519, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)1864888189L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)45885, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)49655, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)29033, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)2420, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)3934, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)56096, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)17010, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)3169, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)59691, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)12284, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)44059, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)22181, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)45860, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)7803, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)356, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)45283, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)16093, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)24080, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -22453, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)11455, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)41713, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p39_param1_SET((float) -1.5110914E38F, PH.base.pack) ;
    p39_param2_SET((float) -2.268777E38F, PH.base.pack) ;
    p39_param3_SET((float) -1.9101395E38F, PH.base.pack) ;
    p39_param4_SET((float)6.7989035E37F, PH.base.pack) ;
    p39_x_SET((float) -2.5626487E37F, PH.base.pack) ;
    p39_y_SET((float) -1.7294815E38F, PH.base.pack) ;
    p39_z_SET((float) -1.0680272E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)13053, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)1011, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)52935, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)40407, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)47075, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p48_latitude_SET((int32_t)1896062083, PH.base.pack) ;
    p48_longitude_SET((int32_t)1202634251, PH.base.pack) ;
    p48_altitude_SET((int32_t) -419800208, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)4527999478942812912L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)836931905, PH.base.pack) ;
    p49_longitude_SET((int32_t) -1971911644, PH.base.pack) ;
    p49_altitude_SET((int32_t)846028698, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)7249683151639622652L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    {
        char16_t   param_id = "vivx";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -26275, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p50_param_value0_SET((float)8.173095E37F, PH.base.pack) ;
    p50_scale_SET((float) -6.192184E37F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.5239361E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)1.233668E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)11787, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p54_p1x_SET((float) -1.4331756E38F, PH.base.pack) ;
    p54_p1y_SET((float) -1.2797019E37F, PH.base.pack) ;
    p54_p1z_SET((float)3.0868815E38F, PH.base.pack) ;
    p54_p2x_SET((float) -3.974568E37F, PH.base.pack) ;
    p54_p2y_SET((float)2.4880902E38F, PH.base.pack) ;
    p54_p2z_SET((float) -3.1182018E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float) -1.9593941E37F, PH.base.pack) ;
    p55_p1y_SET((float)7.18095E37F, PH.base.pack) ;
    p55_p1z_SET((float)3.9050334E37F, PH.base.pack) ;
    p55_p2x_SET((float)2.1907964E38F, PH.base.pack) ;
    p55_p2y_SET((float)1.4209238E38F, PH.base.pack) ;
    p55_p2z_SET((float) -2.38317E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)7333103884332776701L, PH.base.pack) ;
    {
        float  q [] =  {2.4669032E38F, 1.8054387E38F, 3.28545E38F, 3.073164E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -2.7957594E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)5.5660976E37F, PH.base.pack) ;
    p61_yawspeed_SET((float)2.6003167E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.74479E38F, -3.3008818E38F, 3.097063E38F, 2.3240709E38F, 3.2184158E38F, -1.8392844E38F, 5.7386124E36F, 9.602972E37F, 3.003475E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -1.793277E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)2.951209E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -16352, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)5596, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)28968, PH.base.pack) ;
    p62_alt_error_SET((float)1.2219879E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -3.9928266E37F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -1.6202762E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)3708208314178792413L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p63_lat_SET((int32_t)1851596803, PH.base.pack) ;
    p63_lon_SET((int32_t)208672592, PH.base.pack) ;
    p63_alt_SET((int32_t) -938537557, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -176846549, PH.base.pack) ;
    p63_vx_SET((float)2.0913069E38F, PH.base.pack) ;
    p63_vy_SET((float)2.3030445E38F, PH.base.pack) ;
    p63_vz_SET((float)1.0749334E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.5172786E38F, -1.9272756E36F, -1.9866983E38F, 2.7608905E37F, -3.1260475E38F, 2.702867E38F, -1.5126453E38F, -2.8823298E38F, 2.868304E38F, 6.3535576E37F, -2.9569244E38F, -3.3047323E38F, 3.0551104E38F, 3.3583322E38F, -2.9051943E38F, 2.8262862E38F, 2.872724E38F, -1.2360839E38F, 8.086947E36F, 2.505589E38F, 1.8843965E38F, 3.235073E38F, -2.9082588E38F, 2.6058532E38F, -3.0877525E38F, -2.4582487E38F, 1.6111944E38F, -5.4937244E37F, -1.3881086E38F, -2.3810987E38F, 2.8503527E38F, 3.2283075E38F, 2.1501743E38F, 1.1985303E38F, 2.28476E37F, -3.179922E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)2283405285445346807L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p64_x_SET((float) -3.3631555E38F, PH.base.pack) ;
    p64_y_SET((float) -2.1196788E36F, PH.base.pack) ;
    p64_z_SET((float)3.4928812E37F, PH.base.pack) ;
    p64_vx_SET((float) -2.7810926E38F, PH.base.pack) ;
    p64_vy_SET((float)1.4557387E38F, PH.base.pack) ;
    p64_vz_SET((float) -2.3674628E38F, PH.base.pack) ;
    p64_ax_SET((float) -2.161307E38F, PH.base.pack) ;
    p64_ay_SET((float) -2.8225788E38F, PH.base.pack) ;
    p64_az_SET((float)1.0697416E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.933884E38F, -3.0665776E38F, 2.624891E38F, 1.6256558E38F, -1.8271847E38F, 1.2918424E38F, -2.4848132E38F, 2.6965362E38F, -5.575828E37F, 3.4311463E37F, -6.694799E36F, -1.4714384E38F, 2.0304866E38F, -2.8954488E38F, 1.2695687E38F, 6.268437E37F, -1.2507112E38F, 2.1363334E38F, 3.0242957E38F, -2.3682962E38F, -8.784419E37F, -2.2863143E38F, 2.9999682E37F, -6.0275027E37F, 1.734332E36F, -6.5508725E37F, 1.513261E38F, -2.844091E37F, -1.96632E38F, -2.3440348E38F, -5.5675143E37F, -2.3524488E38F, 1.9700211E38F, 7.580944E37F, -2.9408378E38F, -4.0889787E37F, 6.2856136E37F, -1.9630338E38F, -1.2180114E38F, -1.9366724E38F, -2.1385813E38F, -2.2960909E38F, 2.310791E38F, -1.8980693E38F, -4.4276794E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)2490608060L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)52584, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)62619, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)60249, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)15390, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)670, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)50437, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)60848, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)10051, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)7844, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)54420, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)63339, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)61289, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)43004, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)55, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)51340, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)2894, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)37227, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)61289, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)63124, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)44889, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -996, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)28530, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -532, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)19793, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)1337, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)36233, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)51265, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)42956, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)52719, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)27737, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)60872, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)52189, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)64999, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)26964, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p73_param1_SET((float) -4.77225E37F, PH.base.pack) ;
    p73_param2_SET((float) -7.350763E37F, PH.base.pack) ;
    p73_param3_SET((float)1.7354344E38F, PH.base.pack) ;
    p73_param4_SET((float)3.099198E38F, PH.base.pack) ;
    p73_x_SET((int32_t)1703336667, PH.base.pack) ;
    p73_y_SET((int32_t) -458439950, PH.base.pack) ;
    p73_z_SET((float) -9.99416E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)3.2138198E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.0451E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -13529, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)20760, PH.base.pack) ;
    p74_alt_SET((float)2.0727578E38F, PH.base.pack) ;
    p74_climb_SET((float)1.7593483E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p75_param1_SET((float) -3.9648942E37F, PH.base.pack) ;
    p75_param2_SET((float)2.6525771E38F, PH.base.pack) ;
    p75_param3_SET((float) -1.0000656E38F, PH.base.pack) ;
    p75_param4_SET((float)1.0350149E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1093463632, PH.base.pack) ;
    p75_y_SET((int32_t)940671776, PH.base.pack) ;
    p75_z_SET((float)2.8585594E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p76_param1_SET((float)5.6949016E37F, PH.base.pack) ;
    p76_param2_SET((float)1.207571E38F, PH.base.pack) ;
    p76_param3_SET((float) -1.7349642E37F, PH.base.pack) ;
    p76_param4_SET((float)2.4460026E38F, PH.base.pack) ;
    p76_param5_SET((float) -1.8376244E38F, PH.base.pack) ;
    p76_param6_SET((float)3.0956198E38F, PH.base.pack) ;
    p76_param7_SET((float) -3.0559367E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)13, &PH) ;
    p77_result_param2_SET((int32_t)119027729, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)55, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)151, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)3413647544L, PH.base.pack) ;
    p81_roll_SET((float) -9.019945E37F, PH.base.pack) ;
    p81_pitch_SET((float) -6.0918476E37F, PH.base.pack) ;
    p81_yaw_SET((float) -1.465722E38F, PH.base.pack) ;
    p81_thrust_SET((float)4.0395134E37F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)580711282L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    {
        float  q [] =  {1.7289955E38F, -2.9330977E38F, -1.6083868E38F, -1.9188942E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)2.1929435E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)9.31167E37F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -2.4893697E37F, PH.base.pack) ;
    p82_thrust_SET((float)1.1157502E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2666726817L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    {
        float  q [] =  {3.085006E36F, 2.5335204E38F, 2.863473E38F, -4.104378E37F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -9.942118E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -3.0170656E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)5.490907E37F, PH.base.pack) ;
    p83_thrust_SET((float) -3.0635746E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)637976862L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)62336, PH.base.pack) ;
    p84_x_SET((float)7.066141E37F, PH.base.pack) ;
    p84_y_SET((float) -6.4120455E37F, PH.base.pack) ;
    p84_z_SET((float) -1.8451367E37F, PH.base.pack) ;
    p84_vx_SET((float)2.3560834E38F, PH.base.pack) ;
    p84_vy_SET((float)2.3502181E38F, PH.base.pack) ;
    p84_vz_SET((float) -7.5934686E36F, PH.base.pack) ;
    p84_afx_SET((float)6.822383E37F, PH.base.pack) ;
    p84_afy_SET((float)2.8782526E38F, PH.base.pack) ;
    p84_afz_SET((float)2.305113E38F, PH.base.pack) ;
    p84_yaw_SET((float)3.018141E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -1.9386636E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)3241097237L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)56178, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -1715299369, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -1092559799, PH.base.pack) ;
    p86_alt_SET((float)1.65065E38F, PH.base.pack) ;
    p86_vx_SET((float) -8.896288E37F, PH.base.pack) ;
    p86_vy_SET((float)1.8737068E38F, PH.base.pack) ;
    p86_vz_SET((float) -3.0693861E38F, PH.base.pack) ;
    p86_afx_SET((float) -2.3995908E38F, PH.base.pack) ;
    p86_afy_SET((float)2.4971244E38F, PH.base.pack) ;
    p86_afz_SET((float) -5.4246643E37F, PH.base.pack) ;
    p86_yaw_SET((float)3.224086E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -2.5582343E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2116546632L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)24084, PH.base.pack) ;
    p87_lat_int_SET((int32_t)388337980, PH.base.pack) ;
    p87_lon_int_SET((int32_t)1547372276, PH.base.pack) ;
    p87_alt_SET((float) -1.500091E38F, PH.base.pack) ;
    p87_vx_SET((float)2.7740588E38F, PH.base.pack) ;
    p87_vy_SET((float)3.1899034E38F, PH.base.pack) ;
    p87_vz_SET((float)9.089016E36F, PH.base.pack) ;
    p87_afx_SET((float)6.380863E37F, PH.base.pack) ;
    p87_afy_SET((float) -1.7573335E38F, PH.base.pack) ;
    p87_afz_SET((float) -1.8619485E38F, PH.base.pack) ;
    p87_yaw_SET((float)3.1882484E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -1.515818E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)2088568958L, PH.base.pack) ;
    p89_x_SET((float)2.270122E38F, PH.base.pack) ;
    p89_y_SET((float)2.4209492E38F, PH.base.pack) ;
    p89_z_SET((float)2.3665262E38F, PH.base.pack) ;
    p89_roll_SET((float) -3.031377E38F, PH.base.pack) ;
    p89_pitch_SET((float) -1.6711076E38F, PH.base.pack) ;
    p89_yaw_SET((float) -1.380918E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)2782720604189318664L, PH.base.pack) ;
    p90_roll_SET((float) -2.8817469E38F, PH.base.pack) ;
    p90_pitch_SET((float)1.4975341E38F, PH.base.pack) ;
    p90_yaw_SET((float) -2.3609274E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -2.9318096E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -3.1971065E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)1.9768158E37F, PH.base.pack) ;
    p90_lat_SET((int32_t) -2100956565, PH.base.pack) ;
    p90_lon_SET((int32_t)758772646, PH.base.pack) ;
    p90_alt_SET((int32_t)1505927236, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -13123, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)31373, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)19010, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)21458, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)8363, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -15076, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)2821365229861564873L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)2.5644987E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -6.50513E37F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)3.2309337E38F, PH.base.pack) ;
    p91_throttle_SET((float) -3.0320535E38F, PH.base.pack) ;
    p91_aux1_SET((float) -8.599793E37F, PH.base.pack) ;
    p91_aux2_SET((float) -1.0770825E38F, PH.base.pack) ;
    p91_aux3_SET((float)2.6707774E38F, PH.base.pack) ;
    p91_aux4_SET((float) -1.1447806E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)5559574971504142033L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)18188, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)27128, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)19745, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)59375, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)42400, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)50987, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)11960, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)34833, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)16645, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)17414, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)9929, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)34941, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)983472203360166765L, PH.base.pack) ;
    {
        float  controls [] =  {3.2811706E38F, -3.290716E37F, 1.0401273E36F, 1.1218905E38F, -1.2584215E38F, 3.3761898E38F, -1.1812827E38F, 2.4538683E38F, -2.4471826E38F, -1.4052481E38F, 4.69358E37F, 3.3863978E38F, 2.7984803E38F, 8.978629E37F, -1.7434822E38F, -1.267402E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)8477580014851130498L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)7430752305555352463L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -6653, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -26261, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)2.2591615E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -3.2047941E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p100_ground_distance_SET((float)2.6844684E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -3.1078101E38F, &PH) ;
    p100_flow_rate_y_SET((float) -3.36025E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)1878548194946665290L, PH.base.pack) ;
    p101_x_SET((float) -2.1373934E38F, PH.base.pack) ;
    p101_y_SET((float) -8.774418E37F, PH.base.pack) ;
    p101_z_SET((float) -1.6134858E38F, PH.base.pack) ;
    p101_roll_SET((float)4.8762645E37F, PH.base.pack) ;
    p101_pitch_SET((float)2.988819E38F, PH.base.pack) ;
    p101_yaw_SET((float)6.330747E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)5791725255715876462L, PH.base.pack) ;
    p102_x_SET((float)2.842458E38F, PH.base.pack) ;
    p102_y_SET((float)1.9394008E38F, PH.base.pack) ;
    p102_z_SET((float) -3.3518634E37F, PH.base.pack) ;
    p102_roll_SET((float)3.1791227E38F, PH.base.pack) ;
    p102_pitch_SET((float) -5.7896265E37F, PH.base.pack) ;
    p102_yaw_SET((float)2.3601906E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)425509136000947289L, PH.base.pack) ;
    p103_x_SET((float)3.3833307E38F, PH.base.pack) ;
    p103_y_SET((float)9.7625747E36F, PH.base.pack) ;
    p103_z_SET((float) -2.4281728E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)5447754927780319612L, PH.base.pack) ;
    p104_x_SET((float) -5.6951404E37F, PH.base.pack) ;
    p104_y_SET((float)2.7516737E37F, PH.base.pack) ;
    p104_z_SET((float) -2.2437353E38F, PH.base.pack) ;
    p104_roll_SET((float)6.0258643E37F, PH.base.pack) ;
    p104_pitch_SET((float)2.4269678E38F, PH.base.pack) ;
    p104_yaw_SET((float) -1.1572807E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4221851991654848275L, PH.base.pack) ;
    p105_xacc_SET((float)1.0137221E38F, PH.base.pack) ;
    p105_yacc_SET((float) -2.6942585E38F, PH.base.pack) ;
    p105_zacc_SET((float)2.7736883E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -1.9804822E38F, PH.base.pack) ;
    p105_ygyro_SET((float)1.3753955E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -1.0455846E38F, PH.base.pack) ;
    p105_xmag_SET((float) -3.0328165E38F, PH.base.pack) ;
    p105_ymag_SET((float)2.0759012E38F, PH.base.pack) ;
    p105_zmag_SET((float)8.832186E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -3.2526574E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)8.1719294E37F, PH.base.pack) ;
    p105_pressure_alt_SET((float)2.633961E38F, PH.base.pack) ;
    p105_temperature_SET((float) -8.750775E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)23451, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)7174604670508973774L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)1927096902L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.1746672E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -2.1516632E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.5478692E37F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -7.355088E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)2.3362874E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -2487, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2226166847L, PH.base.pack) ;
    p106_distance_SET((float) -2.6147184E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)7391921554981863394L, PH.base.pack) ;
    p107_xacc_SET((float)4.040169E37F, PH.base.pack) ;
    p107_yacc_SET((float) -2.380239E38F, PH.base.pack) ;
    p107_zacc_SET((float)4.569485E37F, PH.base.pack) ;
    p107_xgyro_SET((float)2.1516913E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -6.015183E37F, PH.base.pack) ;
    p107_zgyro_SET((float)1.8885106E38F, PH.base.pack) ;
    p107_xmag_SET((float)5.1052473E37F, PH.base.pack) ;
    p107_ymag_SET((float)5.461741E37F, PH.base.pack) ;
    p107_zmag_SET((float)1.7761164E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -3.2018015E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -3.0298466E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -3.917946E37F, PH.base.pack) ;
    p107_temperature_SET((float) -6.308832E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)3339801657L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -1.60781E38F, PH.base.pack) ;
    p108_q2_SET((float) -3.6967374E37F, PH.base.pack) ;
    p108_q3_SET((float) -8.178773E36F, PH.base.pack) ;
    p108_q4_SET((float)1.2483813E37F, PH.base.pack) ;
    p108_roll_SET((float) -1.230394E38F, PH.base.pack) ;
    p108_pitch_SET((float)1.4324787E38F, PH.base.pack) ;
    p108_yaw_SET((float) -3.0609472E38F, PH.base.pack) ;
    p108_xacc_SET((float)3.345437E38F, PH.base.pack) ;
    p108_yacc_SET((float) -3.2668916E38F, PH.base.pack) ;
    p108_zacc_SET((float)1.8018077E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -7.693596E37F, PH.base.pack) ;
    p108_ygyro_SET((float)2.1080157E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -6.080347E37F, PH.base.pack) ;
    p108_lat_SET((float)1.0597309E38F, PH.base.pack) ;
    p108_lon_SET((float) -8.2533506E37F, PH.base.pack) ;
    p108_alt_SET((float)2.4667396E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)3.348802E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)1.7345606E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.388933E38F, PH.base.pack) ;
    p108_ve_SET((float) -3.2776715E38F, PH.base.pack) ;
    p108_vd_SET((float)7.3176084E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)55825, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)1913, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)171, (uint8_t)211, (uint8_t)243, (uint8_t)112, (uint8_t)134, (uint8_t)4, (uint8_t)29, (uint8_t)220, (uint8_t)94, (uint8_t)15, (uint8_t)185, (uint8_t)143, (uint8_t)242, (uint8_t)142, (uint8_t)161, (uint8_t)219, (uint8_t)72, (uint8_t)190, (uint8_t)197, (uint8_t)225, (uint8_t)71, (uint8_t)175, (uint8_t)170, (uint8_t)149, (uint8_t)150, (uint8_t)146, (uint8_t)122, (uint8_t)30, (uint8_t)83, (uint8_t)180, (uint8_t)119, (uint8_t)96, (uint8_t)187, (uint8_t)87, (uint8_t)26, (uint8_t)143, (uint8_t)219, (uint8_t)103, (uint8_t)98, (uint8_t)114, (uint8_t)237, (uint8_t)87, (uint8_t)112, (uint8_t)139, (uint8_t)24, (uint8_t)163, (uint8_t)214, (uint8_t)205, (uint8_t)40, (uint8_t)71, (uint8_t)147, (uint8_t)125, (uint8_t)102, (uint8_t)18, (uint8_t)17, (uint8_t)46, (uint8_t)100, (uint8_t)219, (uint8_t)5, (uint8_t)37, (uint8_t)7, (uint8_t)161, (uint8_t)53, (uint8_t)22, (uint8_t)59, (uint8_t)182, (uint8_t)182, (uint8_t)34, (uint8_t)147, (uint8_t)250, (uint8_t)206, (uint8_t)30, (uint8_t)169, (uint8_t)224, (uint8_t)218, (uint8_t)253, (uint8_t)74, (uint8_t)121, (uint8_t)172, (uint8_t)148, (uint8_t)232, (uint8_t)70, (uint8_t)175, (uint8_t)101, (uint8_t)219, (uint8_t)117, (uint8_t)15, (uint8_t)80, (uint8_t)79, (uint8_t)176, (uint8_t)31, (uint8_t)232, (uint8_t)79, (uint8_t)30, (uint8_t)244, (uint8_t)6, (uint8_t)20, (uint8_t)160, (uint8_t)20, (uint8_t)192, (uint8_t)49, (uint8_t)158, (uint8_t)166, (uint8_t)123, (uint8_t)202, (uint8_t)187, (uint8_t)80, (uint8_t)51, (uint8_t)169, (uint8_t)12, (uint8_t)222, (uint8_t)147, (uint8_t)50, (uint8_t)117, (uint8_t)79, (uint8_t)104, (uint8_t)75, (uint8_t)197, (uint8_t)13, (uint8_t)116, (uint8_t)116, (uint8_t)18, (uint8_t)2, (uint8_t)198, (uint8_t)119, (uint8_t)34, (uint8_t)55, (uint8_t)166, (uint8_t)23, (uint8_t)144, (uint8_t)10, (uint8_t)215, (uint8_t)7, (uint8_t)207, (uint8_t)155, (uint8_t)58, (uint8_t)116, (uint8_t)164, (uint8_t)99, (uint8_t)223, (uint8_t)28, (uint8_t)213, (uint8_t)4, (uint8_t)91, (uint8_t)86, (uint8_t)105, (uint8_t)159, (uint8_t)154, (uint8_t)79, (uint8_t)241, (uint8_t)27, (uint8_t)144, (uint8_t)24, (uint8_t)130, (uint8_t)234, (uint8_t)11, (uint8_t)82, (uint8_t)249, (uint8_t)179, (uint8_t)177, (uint8_t)82, (uint8_t)157, (uint8_t)131, (uint8_t)102, (uint8_t)155, (uint8_t)212, (uint8_t)254, (uint8_t)20, (uint8_t)156, (uint8_t)12, (uint8_t)0, (uint8_t)206, (uint8_t)218, (uint8_t)102, (uint8_t)86, (uint8_t)88, (uint8_t)128, (uint8_t)32, (uint8_t)149, (uint8_t)149, (uint8_t)219, (uint8_t)58, (uint8_t)154, (uint8_t)110, (uint8_t)131, (uint8_t)123, (uint8_t)128, (uint8_t)13, (uint8_t)124, (uint8_t)85, (uint8_t)139, (uint8_t)217, (uint8_t)131, (uint8_t)240, (uint8_t)104, (uint8_t)156, (uint8_t)28, (uint8_t)173, (uint8_t)123, (uint8_t)195, (uint8_t)188, (uint8_t)70, (uint8_t)127, (uint8_t)169, (uint8_t)34, (uint8_t)75, (uint8_t)203, (uint8_t)195, (uint8_t)120, (uint8_t)108, (uint8_t)202, (uint8_t)27, (uint8_t)244, (uint8_t)255, (uint8_t)219, (uint8_t)24, (uint8_t)167, (uint8_t)233, (uint8_t)77, (uint8_t)81, (uint8_t)17, (uint8_t)232, (uint8_t)126, (uint8_t)208, (uint8_t)118, (uint8_t)128, (uint8_t)157, (uint8_t)180, (uint8_t)219, (uint8_t)163, (uint8_t)246, (uint8_t)165, (uint8_t)31, (uint8_t)168, (uint8_t)93, (uint8_t)238, (uint8_t)165, (uint8_t)178, (uint8_t)201, (uint8_t)148, (uint8_t)98, (uint8_t)161, (uint8_t)72, (uint8_t)248, (uint8_t)48, (uint8_t)47, (uint8_t)64, (uint8_t)252, (uint8_t)90, (uint8_t)0, (uint8_t)120};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)1754416690650925042L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -2021807218117879146L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)7266332324990830903L, PH.base.pack) ;
    p112_seq_SET((uint32_t)3417681399L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)6651694321495263516L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p113_lat_SET((int32_t)1893697982, PH.base.pack) ;
    p113_lon_SET((int32_t) -164623232, PH.base.pack) ;
    p113_alt_SET((int32_t)490581004, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)35465, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)65070, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)29529, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -6873, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -32006, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)12481, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)29410, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)3793638448560854176L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)2595549974L, PH.base.pack) ;
    p114_integrated_x_SET((float) -1.9657604E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)1.0834882E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -2.446705E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.4166329E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)2.2697653E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)18006, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)3863767994L, PH.base.pack) ;
    p114_distance_SET((float) -8.0295134E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)6875237902209941873L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {1.476483E38F, -2.0434353E38F, 4.879649E37F, 2.0702525E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -9.91899E37F, PH.base.pack) ;
    p115_pitchspeed_SET((float)7.3940633E36F, PH.base.pack) ;
    p115_yawspeed_SET((float) -2.4897408E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1588266627, PH.base.pack) ;
    p115_lon_SET((int32_t) -1060158793, PH.base.pack) ;
    p115_alt_SET((int32_t)1481474076, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -12248, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -14344, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -5558, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)10575, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)34946, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -12471, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)18283, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -7304, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)1370913933L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)21082, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)4973, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -32419, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -30885, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)19207, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)7113, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)11547, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)6542, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)29117, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)55567, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)51400, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)42065, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)36241, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)39946, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2308693789L, PH.base.pack) ;
    p118_size_SET((uint32_t)1863127398L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)14391, PH.base.pack) ;
    p119_ofs_SET((uint32_t)677071736L, PH.base.pack) ;
    p119_count_SET((uint32_t)2804440586L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)12218, PH.base.pack) ;
    p120_ofs_SET((uint32_t)3676376360L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)163, (uint8_t)94, (uint8_t)154, (uint8_t)133, (uint8_t)107, (uint8_t)237, (uint8_t)56, (uint8_t)18, (uint8_t)97, (uint8_t)14, (uint8_t)5, (uint8_t)3, (uint8_t)207, (uint8_t)66, (uint8_t)17, (uint8_t)222, (uint8_t)64, (uint8_t)246, (uint8_t)10, (uint8_t)123, (uint8_t)240, (uint8_t)137, (uint8_t)162, (uint8_t)183, (uint8_t)27, (uint8_t)53, (uint8_t)8, (uint8_t)71, (uint8_t)79, (uint8_t)27, (uint8_t)246, (uint8_t)214, (uint8_t)207, (uint8_t)248, (uint8_t)66, (uint8_t)82, (uint8_t)53, (uint8_t)190, (uint8_t)128, (uint8_t)25, (uint8_t)75, (uint8_t)167, (uint8_t)163, (uint8_t)10, (uint8_t)161, (uint8_t)8, (uint8_t)253, (uint8_t)205, (uint8_t)5, (uint8_t)43, (uint8_t)148, (uint8_t)29, (uint8_t)252, (uint8_t)3, (uint8_t)15, (uint8_t)111, (uint8_t)64, (uint8_t)190, (uint8_t)174, (uint8_t)195, (uint8_t)122, (uint8_t)80, (uint8_t)23, (uint8_t)158, (uint8_t)157, (uint8_t)213, (uint8_t)98, (uint8_t)137, (uint8_t)115, (uint8_t)145, (uint8_t)131, (uint8_t)170, (uint8_t)44, (uint8_t)164, (uint8_t)222, (uint8_t)145, (uint8_t)96, (uint8_t)144, (uint8_t)81, (uint8_t)91, (uint8_t)153, (uint8_t)173, (uint8_t)234, (uint8_t)145, (uint8_t)181, (uint8_t)169, (uint8_t)190, (uint8_t)227, (uint8_t)152, (uint8_t)5};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)199, (uint8_t)219, (uint8_t)145, (uint8_t)246, (uint8_t)135, (uint8_t)124, (uint8_t)29, (uint8_t)232, (uint8_t)119, (uint8_t)189, (uint8_t)111, (uint8_t)1, (uint8_t)122, (uint8_t)28, (uint8_t)105, (uint8_t)134, (uint8_t)55, (uint8_t)7, (uint8_t)77, (uint8_t)123, (uint8_t)244, (uint8_t)62, (uint8_t)204, (uint8_t)82, (uint8_t)60, (uint8_t)152, (uint8_t)134, (uint8_t)105, (uint8_t)31, (uint8_t)241, (uint8_t)75, (uint8_t)159, (uint8_t)34, (uint8_t)178, (uint8_t)224, (uint8_t)194, (uint8_t)204, (uint8_t)100, (uint8_t)94, (uint8_t)48, (uint8_t)14, (uint8_t)125, (uint8_t)147, (uint8_t)87, (uint8_t)76, (uint8_t)141, (uint8_t)92, (uint8_t)20, (uint8_t)77, (uint8_t)61, (uint8_t)206, (uint8_t)196, (uint8_t)254, (uint8_t)254, (uint8_t)12, (uint8_t)215, (uint8_t)128, (uint8_t)20, (uint8_t)132, (uint8_t)96, (uint8_t)105, (uint8_t)200, (uint8_t)50, (uint8_t)105, (uint8_t)8, (uint8_t)29, (uint8_t)70, (uint8_t)245, (uint8_t)65, (uint8_t)125, (uint8_t)123, (uint8_t)138, (uint8_t)63, (uint8_t)70, (uint8_t)63, (uint8_t)254, (uint8_t)97, (uint8_t)172, (uint8_t)248, (uint8_t)171, (uint8_t)11, (uint8_t)204, (uint8_t)248, (uint8_t)177, (uint8_t)92, (uint8_t)250, (uint8_t)46, (uint8_t)89, (uint8_t)58, (uint8_t)133, (uint8_t)186, (uint8_t)195, (uint8_t)152, (uint8_t)2, (uint8_t)81, (uint8_t)149, (uint8_t)111, (uint8_t)160, (uint8_t)17, (uint8_t)17, (uint8_t)140, (uint8_t)164, (uint8_t)245, (uint8_t)139, (uint8_t)9, (uint8_t)213, (uint8_t)175, (uint8_t)46, (uint8_t)124, (uint8_t)163};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)8134144421306488866L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t)61545861, PH.base.pack) ;
    p124_lon_SET((int32_t)1468358225, PH.base.pack) ;
    p124_alt_SET((int32_t)755859588, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)15982, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)48311, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)9319, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)40884, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)3610672316L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)36835, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)57176, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)9311, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)485071962L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)244, (uint8_t)143, (uint8_t)119, (uint8_t)202, (uint8_t)41, (uint8_t)115, (uint8_t)188, (uint8_t)133, (uint8_t)24, (uint8_t)100, (uint8_t)45, (uint8_t)22, (uint8_t)44, (uint8_t)187, (uint8_t)209, (uint8_t)246, (uint8_t)142, (uint8_t)153, (uint8_t)172, (uint8_t)50, (uint8_t)202, (uint8_t)94, (uint8_t)226, (uint8_t)74, (uint8_t)58, (uint8_t)143, (uint8_t)251, (uint8_t)118, (uint8_t)45, (uint8_t)242, (uint8_t)123, (uint8_t)112, (uint8_t)0, (uint8_t)85, (uint8_t)210, (uint8_t)94, (uint8_t)206, (uint8_t)147, (uint8_t)24, (uint8_t)44, (uint8_t)47, (uint8_t)242, (uint8_t)33, (uint8_t)64, (uint8_t)203, (uint8_t)8, (uint8_t)83, (uint8_t)179, (uint8_t)169, (uint8_t)198, (uint8_t)18, (uint8_t)17, (uint8_t)30, (uint8_t)26, (uint8_t)17, (uint8_t)38, (uint8_t)220, (uint8_t)180, (uint8_t)10, (uint8_t)220, (uint8_t)145, (uint8_t)63, (uint8_t)70, (uint8_t)207, (uint8_t)219, (uint8_t)27, (uint8_t)173, (uint8_t)152, (uint8_t)221, (uint8_t)14};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)664535970L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)52436, PH.base.pack) ;
    p127_tow_SET((uint32_t)119835529L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)799026860, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)1798862759, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)460599958, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2051296158L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1025497717, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)3048376663L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)26801, PH.base.pack) ;
    p128_tow_SET((uint32_t)2194098266L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1182317137, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -916768663, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -1882989675, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1427930986L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -1648369513, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)894014403L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -27303, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)18084, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -9256, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)22985, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -21721, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -17794, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -1039, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -31913, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -21010, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p130_size_SET((uint32_t)3742775557L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)6633, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)23378, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)13669, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)62577, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)57, (uint8_t)82, (uint8_t)210, (uint8_t)159, (uint8_t)87, (uint8_t)176, (uint8_t)102, (uint8_t)171, (uint8_t)161, (uint8_t)107, (uint8_t)192, (uint8_t)122, (uint8_t)97, (uint8_t)73, (uint8_t)60, (uint8_t)225, (uint8_t)180, (uint8_t)78, (uint8_t)76, (uint8_t)248, (uint8_t)232, (uint8_t)14, (uint8_t)12, (uint8_t)193, (uint8_t)106, (uint8_t)205, (uint8_t)174, (uint8_t)73, (uint8_t)176, (uint8_t)181, (uint8_t)80, (uint8_t)200, (uint8_t)148, (uint8_t)205, (uint8_t)205, (uint8_t)164, (uint8_t)37, (uint8_t)86, (uint8_t)158, (uint8_t)214, (uint8_t)251, (uint8_t)168, (uint8_t)152, (uint8_t)94, (uint8_t)248, (uint8_t)225, (uint8_t)82, (uint8_t)116, (uint8_t)60, (uint8_t)133, (uint8_t)65, (uint8_t)176, (uint8_t)235, (uint8_t)69, (uint8_t)137, (uint8_t)217, (uint8_t)155, (uint8_t)87, (uint8_t)250, (uint8_t)107, (uint8_t)21, (uint8_t)246, (uint8_t)194, (uint8_t)106, (uint8_t)78, (uint8_t)160, (uint8_t)60, (uint8_t)154, (uint8_t)179, (uint8_t)146, (uint8_t)160, (uint8_t)74, (uint8_t)214, (uint8_t)229, (uint8_t)112, (uint8_t)149, (uint8_t)192, (uint8_t)91, (uint8_t)30, (uint8_t)177, (uint8_t)231, (uint8_t)48, (uint8_t)206, (uint8_t)18, (uint8_t)65, (uint8_t)97, (uint8_t)219, (uint8_t)246, (uint8_t)133, (uint8_t)36, (uint8_t)108, (uint8_t)156, (uint8_t)17, (uint8_t)57, (uint8_t)255, (uint8_t)46, (uint8_t)141, (uint8_t)65, (uint8_t)17, (uint8_t)139, (uint8_t)34, (uint8_t)52, (uint8_t)176, (uint8_t)58, (uint8_t)110, (uint8_t)162, (uint8_t)16, (uint8_t)35, (uint8_t)232, (uint8_t)67, (uint8_t)126, (uint8_t)152, (uint8_t)69, (uint8_t)177, (uint8_t)157, (uint8_t)180, (uint8_t)4, (uint8_t)57, (uint8_t)6, (uint8_t)122, (uint8_t)149, (uint8_t)151, (uint8_t)57, (uint8_t)200, (uint8_t)237, (uint8_t)56, (uint8_t)2, (uint8_t)143, (uint8_t)79, (uint8_t)14, (uint8_t)21, (uint8_t)223, (uint8_t)69, (uint8_t)195, (uint8_t)70, (uint8_t)238, (uint8_t)213, (uint8_t)112, (uint8_t)12, (uint8_t)89, (uint8_t)246, (uint8_t)47, (uint8_t)44, (uint8_t)72, (uint8_t)184, (uint8_t)66, (uint8_t)20, (uint8_t)80, (uint8_t)15, (uint8_t)162, (uint8_t)230, (uint8_t)164, (uint8_t)3, (uint8_t)44, (uint8_t)118, (uint8_t)118, (uint8_t)7, (uint8_t)61, (uint8_t)59, (uint8_t)100, (uint8_t)65, (uint8_t)58, (uint8_t)191, (uint8_t)21, (uint8_t)14, (uint8_t)174, (uint8_t)64, (uint8_t)63, (uint8_t)248, (uint8_t)222, (uint8_t)104, (uint8_t)231, (uint8_t)45, (uint8_t)226, (uint8_t)116, (uint8_t)249, (uint8_t)145, (uint8_t)108, (uint8_t)243, (uint8_t)221, (uint8_t)178, (uint8_t)102, (uint8_t)13, (uint8_t)31, (uint8_t)237, (uint8_t)72, (uint8_t)45, (uint8_t)132, (uint8_t)82, (uint8_t)163, (uint8_t)172, (uint8_t)228, (uint8_t)215, (uint8_t)115, (uint8_t)155, (uint8_t)190, (uint8_t)146, (uint8_t)211, (uint8_t)12, (uint8_t)68, (uint8_t)25, (uint8_t)118, (uint8_t)195, (uint8_t)245, (uint8_t)199, (uint8_t)147, (uint8_t)145, (uint8_t)61, (uint8_t)110, (uint8_t)222, (uint8_t)173, (uint8_t)43, (uint8_t)204, (uint8_t)95, (uint8_t)162, (uint8_t)172, (uint8_t)20, (uint8_t)78, (uint8_t)39, (uint8_t)197, (uint8_t)12, (uint8_t)187, (uint8_t)113, (uint8_t)97, (uint8_t)167, (uint8_t)118, (uint8_t)108, (uint8_t)31, (uint8_t)107, (uint8_t)200, (uint8_t)193, (uint8_t)27, (uint8_t)240, (uint8_t)176, (uint8_t)34, (uint8_t)15, (uint8_t)247, (uint8_t)143, (uint8_t)130, (uint8_t)151, (uint8_t)224, (uint8_t)143, (uint8_t)99, (uint8_t)135, (uint8_t)104, (uint8_t)236, (uint8_t)197, (uint8_t)103, (uint8_t)235, (uint8_t)247, (uint8_t)152, (uint8_t)251, (uint8_t)24};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2188970940L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)23422, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)363, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)13755, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1173016974, PH.base.pack) ;
    p133_lon_SET((int32_t)1582260730, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)26453, PH.base.pack) ;
    p133_mask_SET((uint64_t)3743007941073417733L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1630062763, PH.base.pack) ;
    p134_lon_SET((int32_t)1858374686, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)38326, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -1629, (int16_t) -29904, (int16_t)14017, (int16_t)988, (int16_t)27521, (int16_t) -8596, (int16_t) -21187, (int16_t) -7930, (int16_t)4721, (int16_t)29721, (int16_t) -8568, (int16_t)30844, (int16_t)31662, (int16_t) -14029, (int16_t) -20766, (int16_t)13413};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1673195080, PH.base.pack) ;
    p135_lon_SET((int32_t) -1317447510, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)685483783, PH.base.pack) ;
    p136_lon_SET((int32_t) -1475060300, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)44503, PH.base.pack) ;
    p136_terrain_height_SET((float) -2.732385E38F, PH.base.pack) ;
    p136_current_height_SET((float) -8.879255E37F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)64790, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)10062, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)3565475963L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.2444671E38F, PH.base.pack) ;
    p137_press_diff_SET((float)2.8551307E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -6861, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)3443827773991667576L, PH.base.pack) ;
    {
        float  q [] =  {-1.7006542E37F, -1.5907692E38F, -2.2642635E38F, 3.022803E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -1.2999026E38F, PH.base.pack) ;
    p138_y_SET((float) -1.3363149E38F, PH.base.pack) ;
    p138_z_SET((float) -2.8514626E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)3214526908384050426L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    {
        float  controls [] =  {-3.8107785E37F, -1.4166871E38F, -7.237228E37F, 9.282648E37F, -2.9914646E38F, 3.3154647E38F, -2.015206E38F, 2.7711077E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)612376303852276419L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    {
        float  controls [] =  {-3.303376E38F, -1.5256482E38F, -2.0746595E38F, -2.4310516E37F, -1.2913142E38F, -3.389656E38F, 2.9352137E38F, 9.570689E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)1845678021779308622L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.0063735E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -4.3019863E37F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.8704578E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.7743484E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)5.802128E37F, PH.base.pack) ;
    p141_bottom_clearance_SET((float) -7.1316704E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)56, (uint8_t)203, (uint8_t)85, (uint8_t)173, (uint8_t)87, (uint8_t)52, (uint8_t)195, (uint8_t)110, (uint8_t)7, (uint8_t)34, (uint8_t)164, (uint8_t)200, (uint8_t)214, (uint8_t)138, (uint8_t)208, (uint8_t)94, (uint8_t)45, (uint8_t)251, (uint8_t)3, (uint8_t)209, (uint8_t)244, (uint8_t)145, (uint8_t)15, (uint8_t)48, (uint8_t)61, (uint8_t)213, (uint8_t)39, (uint8_t)6, (uint8_t)184, (uint8_t)21, (uint8_t)3, (uint8_t)94, (uint8_t)136, (uint8_t)129, (uint8_t)37, (uint8_t)81, (uint8_t)233, (uint8_t)244, (uint8_t)244, (uint8_t)112, (uint8_t)187, (uint8_t)20, (uint8_t)68, (uint8_t)13, (uint8_t)135, (uint8_t)98, (uint8_t)132, (uint8_t)6, (uint8_t)193, (uint8_t)207, (uint8_t)163, (uint8_t)56, (uint8_t)185, (uint8_t)134, (uint8_t)57, (uint8_t)43, (uint8_t)211, (uint8_t)0, (uint8_t)163, (uint8_t)102, (uint8_t)107, (uint8_t)216, (uint8_t)24, (uint8_t)187, (uint8_t)7, (uint8_t)126, (uint8_t)116, (uint8_t)214, (uint8_t)179, (uint8_t)34, (uint8_t)70, (uint8_t)220, (uint8_t)21, (uint8_t)45, (uint8_t)143, (uint8_t)217, (uint8_t)68, (uint8_t)214, (uint8_t)12, (uint8_t)64, (uint8_t)218, (uint8_t)138, (uint8_t)207, (uint8_t)15, (uint8_t)149, (uint8_t)176, (uint8_t)233, (uint8_t)10, (uint8_t)232, (uint8_t)53, (uint8_t)117, (uint8_t)129, (uint8_t)219, (uint8_t)15, (uint8_t)231, (uint8_t)235, (uint8_t)164, (uint8_t)74, (uint8_t)69, (uint8_t)149, (uint8_t)239, (uint8_t)121, (uint8_t)137, (uint8_t)41, (uint8_t)177, (uint8_t)42, (uint8_t)87, (uint8_t)97, (uint8_t)137, (uint8_t)18, (uint8_t)236, (uint8_t)114, (uint8_t)180, (uint8_t)40, (uint8_t)126, (uint8_t)34, (uint8_t)220, (uint8_t)167, (uint8_t)56, (uint8_t)110};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)102, (uint8_t)197, (uint8_t)166, (uint8_t)171, (uint8_t)205, (uint8_t)76, (uint8_t)139, (uint8_t)156, (uint8_t)151, (uint8_t)59, (uint8_t)119, (uint8_t)233, (uint8_t)1, (uint8_t)212, (uint8_t)211, (uint8_t)202, (uint8_t)170, (uint8_t)173, (uint8_t)242, (uint8_t)247, (uint8_t)168, (uint8_t)207, (uint8_t)111, (uint8_t)71, (uint8_t)134, (uint8_t)201, (uint8_t)66, (uint8_t)34, (uint8_t)200, (uint8_t)235, (uint8_t)122, (uint8_t)89, (uint8_t)173, (uint8_t)11, (uint8_t)133, (uint8_t)121, (uint8_t)246, (uint8_t)205, (uint8_t)211, (uint8_t)5, (uint8_t)62, (uint8_t)55, (uint8_t)114, (uint8_t)205, (uint8_t)238, (uint8_t)153, (uint8_t)232, (uint8_t)29, (uint8_t)134, (uint8_t)101, (uint8_t)211, (uint8_t)111, (uint8_t)184, (uint8_t)222, (uint8_t)91, (uint8_t)79, (uint8_t)175, (uint8_t)86, (uint8_t)94, (uint8_t)25, (uint8_t)14, (uint8_t)143, (uint8_t)27, (uint8_t)156, (uint8_t)68, (uint8_t)158, (uint8_t)142, (uint8_t)117, (uint8_t)246, (uint8_t)122, (uint8_t)58, (uint8_t)250, (uint8_t)27, (uint8_t)52, (uint8_t)201, (uint8_t)0, (uint8_t)42, (uint8_t)43, (uint8_t)145, (uint8_t)35, (uint8_t)6, (uint8_t)127, (uint8_t)74, (uint8_t)236, (uint8_t)90, (uint8_t)205, (uint8_t)54, (uint8_t)16, (uint8_t)79, (uint8_t)70, (uint8_t)33, (uint8_t)235, (uint8_t)24, (uint8_t)3, (uint8_t)247, (uint8_t)6, (uint8_t)135, (uint8_t)3, (uint8_t)34, (uint8_t)67, (uint8_t)65, (uint8_t)89, (uint8_t)112, (uint8_t)195, (uint8_t)185, (uint8_t)93, (uint8_t)166, (uint8_t)6, (uint8_t)157, (uint8_t)211, (uint8_t)18, (uint8_t)194, (uint8_t)104, (uint8_t)216, (uint8_t)233, (uint8_t)98, (uint8_t)213, (uint8_t)227, (uint8_t)202, (uint8_t)21};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)1030997711L, PH.base.pack) ;
    p143_press_abs_SET((float)2.0002632E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.836853E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -11488, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)632220521054089487L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p144_lat_SET((int32_t)117376495, PH.base.pack) ;
    p144_lon_SET((int32_t)112976853, PH.base.pack) ;
    p144_alt_SET((float)3.8269664E37F, PH.base.pack) ;
    {
        float  vel [] =  {-2.9977407E38F, 2.544571E38F, -1.446168E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-4.9532124E37F, 1.6312249E38F, -2.3694337E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {2.9001501E38F, 1.2288659E38F, -2.1302962E38F, 2.9785696E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {-3.2967538E38F, -3.0676335E38F, -2.9136042E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {1.3710671E38F, 2.852445E38F, 8.192818E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)549060740079652137L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)6066284286041339279L, PH.base.pack) ;
    p146_x_acc_SET((float) -1.0012965E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -5.744663E37F, PH.base.pack) ;
    p146_z_acc_SET((float) -1.9654267E38F, PH.base.pack) ;
    p146_x_vel_SET((float)1.0463767E38F, PH.base.pack) ;
    p146_y_vel_SET((float)1.0953125E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -4.992463E37F, PH.base.pack) ;
    p146_x_pos_SET((float) -5.956713E37F, PH.base.pack) ;
    p146_y_pos_SET((float) -1.3205727E38F, PH.base.pack) ;
    p146_z_pos_SET((float)3.057784E38F, PH.base.pack) ;
    p146_airspeed_SET((float) -2.7579682E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {2.7997587E38F, 1.7167873E37F, -7.5410207E37F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {1.9408313E38F, -2.9426789E38F, 1.3751269E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-1.4557635E37F, 2.2531952E38F, 3.3192508E38F, 8.81137E37F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)2.8717987E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -2.6223099E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)2.9935392E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -11604, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)23949, (uint16_t)7368, (uint16_t)22499, (uint16_t)30626, (uint16_t)36728, (uint16_t)40034, (uint16_t)21525, (uint16_t)55254, (uint16_t)25233, (uint16_t)36449};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)8388, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -1870715779, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -2061741243, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -113, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)2210675807L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)3126427522L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)4287658494L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)3849041402L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)90, (uint8_t)250, (uint8_t)128, (uint8_t)18, (uint8_t)56, (uint8_t)185, (uint8_t)89, (uint8_t)101};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)162, (uint8_t)245, (uint8_t)167, (uint8_t)53, (uint8_t)207, (uint8_t)164, (uint8_t)235, (uint8_t)83};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)64, (uint8_t)204, (uint8_t)132, (uint8_t)244, (uint8_t)71, (uint8_t)67, (uint8_t)150, (uint8_t)6};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)33697, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)8468, PH.base.pack) ;
    p148_uid_SET((uint64_t)775760028431170840L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)69, (uint8_t)56, (uint8_t)140, (uint8_t)51, (uint8_t)108, (uint8_t)19, (uint8_t)43, (uint8_t)236, (uint8_t)234, (uint8_t)45, (uint8_t)227, (uint8_t)35, (uint8_t)173, (uint8_t)253, (uint8_t)132, (uint8_t)29, (uint8_t)244, (uint8_t)184};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)9174389689449746091L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p149_angle_x_SET((float)2.7011722E38F, PH.base.pack) ;
    p149_angle_y_SET((float)2.3194779E37F, PH.base.pack) ;
    p149_distance_SET((float)2.936992E38F, PH.base.pack) ;
    p149_size_x_SET((float) -2.9168878E38F, PH.base.pack) ;
    p149_size_y_SET((float) -2.3780367E38F, PH.base.pack) ;
    p149_x_SET((float) -1.1710633E38F, &PH) ;
    p149_y_SET((float) -2.6514784E38F, &PH) ;
    p149_z_SET((float) -3.1448489E38F, &PH) ;
    {
        float  q [] =  {-1.2006134E37F, -3.1643173E38F, -1.1295933E38F, -2.9270864E37F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)47, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_FILTER_BIAS_220(), &PH);
    p220_usec_SET((uint64_t)3259599507556444480L, PH.base.pack) ;
    p220_accel_0_SET((float)1.7372018E38F, PH.base.pack) ;
    p220_accel_1_SET((float)3.2250883E38F, PH.base.pack) ;
    p220_accel_2_SET((float)1.0298642E38F, PH.base.pack) ;
    p220_gyro_0_SET((float) -1.5044832E38F, PH.base.pack) ;
    p220_gyro_1_SET((float) -3.500036E37F, PH.base.pack) ;
    p220_gyro_2_SET((float)7.176736E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_CALIBRATION_221(), &PH);
    {
        uint16_t  aileron [] =  {(uint16_t)2995, (uint16_t)39586, (uint16_t)23828};
        p221_aileron_SET(&aileron, 0, &PH.base.pack) ;
    }
    {
        uint16_t  elevator [] =  {(uint16_t)37917, (uint16_t)8428, (uint16_t)61942};
        p221_elevator_SET(&elevator, 0, &PH.base.pack) ;
    }
    {
        uint16_t  rudder [] =  {(uint16_t)28595, (uint16_t)48988, (uint16_t)17059};
        p221_rudder_SET(&rudder, 0, &PH.base.pack) ;
    }
    {
        uint16_t  gyro [] =  {(uint16_t)42397, (uint16_t)47971};
        p221_gyro_SET(&gyro, 0, &PH.base.pack) ;
    }
    {
        uint16_t  pitch [] =  {(uint16_t)10155, (uint16_t)55035, (uint16_t)29675, (uint16_t)12753, (uint16_t)27266};
        p221_pitch_SET(&pitch, 0, &PH.base.pack) ;
    }
    {
        uint16_t  throttle [] =  {(uint16_t)65152, (uint16_t)65190, (uint16_t)29244, (uint16_t)36121, (uint16_t)63406};
        p221_throttle_SET(&throttle, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UALBERTA_SYS_STATUS_222(), &PH);
    p222_mode_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p222_nav_mode_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p222_pilot_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)6919007690519547418L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
    p230_vel_ratio_SET((float)1.998523E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)3.153904E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -3.271831E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -3.3873486E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -2.9751859E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -2.7398554E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -8.706567E37F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)3.1723976E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)7223908556452099566L, PH.base.pack) ;
    p231_wind_x_SET((float)4.6148728E36F, PH.base.pack) ;
    p231_wind_y_SET((float)1.3751157E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -2.700934E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -6.2610764E37F, PH.base.pack) ;
    p231_var_vert_SET((float)1.647736E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -9.611919E37F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float) -2.1285257E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)1.6640074E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)399525956556531335L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)3549278585L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)16312, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p232_lat_SET((int32_t)1029097477, PH.base.pack) ;
    p232_lon_SET((int32_t)1733980689, PH.base.pack) ;
    p232_alt_SET((float) -1.6713629E38F, PH.base.pack) ;
    p232_hdop_SET((float)8.654067E37F, PH.base.pack) ;
    p232_vdop_SET((float) -5.501569E37F, PH.base.pack) ;
    p232_vn_SET((float)2.3689771E38F, PH.base.pack) ;
    p232_ve_SET((float)6.2725116E37F, PH.base.pack) ;
    p232_vd_SET((float) -2.515147E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -1.7804913E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float)2.0063921E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -1.0034246E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)125, (uint8_t)80, (uint8_t)141, (uint8_t)82, (uint8_t)12, (uint8_t)23, (uint8_t)83, (uint8_t)30, (uint8_t)71, (uint8_t)165, (uint8_t)212, (uint8_t)37, (uint8_t)160, (uint8_t)185, (uint8_t)123, (uint8_t)89, (uint8_t)227, (uint8_t)47, (uint8_t)237, (uint8_t)101, (uint8_t)115, (uint8_t)30, (uint8_t)131, (uint8_t)159, (uint8_t)14, (uint8_t)30, (uint8_t)179, (uint8_t)108, (uint8_t)168, (uint8_t)37, (uint8_t)119, (uint8_t)237, (uint8_t)131, (uint8_t)234, (uint8_t)164, (uint8_t)252, (uint8_t)108, (uint8_t)45, (uint8_t)14, (uint8_t)65, (uint8_t)27, (uint8_t)246, (uint8_t)198, (uint8_t)2, (uint8_t)218, (uint8_t)87, (uint8_t)178, (uint8_t)99, (uint8_t)153, (uint8_t)118, (uint8_t)105, (uint8_t)176, (uint8_t)75, (uint8_t)190, (uint8_t)24, (uint8_t)12, (uint8_t)231, (uint8_t)165, (uint8_t)15, (uint8_t)213, (uint8_t)213, (uint8_t)208, (uint8_t)211, (uint8_t)67, (uint8_t)247, (uint8_t)255, (uint8_t)104, (uint8_t)166, (uint8_t)240, (uint8_t)58, (uint8_t)204, (uint8_t)18, (uint8_t)88, (uint8_t)247, (uint8_t)188, (uint8_t)158, (uint8_t)50, (uint8_t)31, (uint8_t)177, (uint8_t)167, (uint8_t)71, (uint8_t)68, (uint8_t)202, (uint8_t)92, (uint8_t)194, (uint8_t)136, (uint8_t)70, (uint8_t)178, (uint8_t)147, (uint8_t)101, (uint8_t)107, (uint8_t)76, (uint8_t)60, (uint8_t)213, (uint8_t)108, (uint8_t)38, (uint8_t)51, (uint8_t)12, (uint8_t)32, (uint8_t)33, (uint8_t)246, (uint8_t)251, (uint8_t)103, (uint8_t)212, (uint8_t)214, (uint8_t)157, (uint8_t)24, (uint8_t)13, (uint8_t)189, (uint8_t)108, (uint8_t)159, (uint8_t)228, (uint8_t)105, (uint8_t)187, (uint8_t)5, (uint8_t)132, (uint8_t)44, (uint8_t)176, (uint8_t)191, (uint8_t)150, (uint8_t)167, (uint8_t)176, (uint8_t)137, (uint8_t)172, (uint8_t)43, (uint8_t)21, (uint8_t)30, (uint8_t)165, (uint8_t)6, (uint8_t)59, (uint8_t)254, (uint8_t)172, (uint8_t)83, (uint8_t)9, (uint8_t)86, (uint8_t)235, (uint8_t)70, (uint8_t)183, (uint8_t)37, (uint8_t)222, (uint8_t)69, (uint8_t)249, (uint8_t)7, (uint8_t)50, (uint8_t)123, (uint8_t)214, (uint8_t)194, (uint8_t)239, (uint8_t)233, (uint8_t)223, (uint8_t)53, (uint8_t)212, (uint8_t)210, (uint8_t)200, (uint8_t)130, (uint8_t)200, (uint8_t)186, (uint8_t)202, (uint8_t)209, (uint8_t)167, (uint8_t)222, (uint8_t)43, (uint8_t)4, (uint8_t)157, (uint8_t)242, (uint8_t)212, (uint8_t)251, (uint8_t)93, (uint8_t)9, (uint8_t)157, (uint8_t)45, (uint8_t)105, (uint8_t)88, (uint8_t)86, (uint8_t)9, (uint8_t)227, (uint8_t)38, (uint8_t)16, (uint8_t)38, (uint8_t)167};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1209996561L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)4675, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)18176, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)5812, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t)109, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)15446, PH.base.pack) ;
    p234_latitude_SET((int32_t) -1563838449, PH.base.pack) ;
    p234_longitude_SET((int32_t) -181282114, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)7037, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)368, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)111, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)78, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -53, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)43955, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)5684873975258334420L, PH.base.pack) ;
    p241_vibration_x_SET((float)1.3989572E38F, PH.base.pack) ;
    p241_vibration_y_SET((float)2.5649525E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)7.8284493E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3225072438L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)4001560271L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)3842501095L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)500767506, PH.base.pack) ;
    p242_longitude_SET((int32_t)837494753, PH.base.pack) ;
    p242_altitude_SET((int32_t) -1190396933, PH.base.pack) ;
    p242_x_SET((float)3.3152308E38F, PH.base.pack) ;
    p242_y_SET((float)2.3588704E38F, PH.base.pack) ;
    p242_z_SET((float) -1.9793861E38F, PH.base.pack) ;
    {
        float  q [] =  {1.9339856E38F, -3.0511291E38F, 5.858898E37F, 1.7932147E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)9.803065E37F, PH.base.pack) ;
    p242_approach_y_SET((float)2.446682E38F, PH.base.pack) ;
    p242_approach_z_SET((float)2.8699835E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)65627972080842823L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p243_latitude_SET((int32_t) -2039857033, PH.base.pack) ;
    p243_longitude_SET((int32_t)108588608, PH.base.pack) ;
    p243_altitude_SET((int32_t)558288637, PH.base.pack) ;
    p243_x_SET((float) -9.536285E37F, PH.base.pack) ;
    p243_y_SET((float)1.0730666E38F, PH.base.pack) ;
    p243_z_SET((float)5.7457764E37F, PH.base.pack) ;
    {
        float  q [] =  {-2.8087223E38F, 2.9787984E38F, 8.206583E37F, 2.4173132E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -2.6603735E38F, PH.base.pack) ;
    p243_approach_y_SET((float)1.432416E38F, PH.base.pack) ;
    p243_approach_z_SET((float)2.0978607E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)5767326654313134915L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)32573, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -1516549565, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)1334634204L, PH.base.pack) ;
    p246_lat_SET((int32_t) -1500418206, PH.base.pack) ;
    p246_lon_SET((int32_t)244793041, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t) -1744837945, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)54238, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)50090, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -1464, PH.base.pack) ;
    {
        char16_t   callsign = "gusH";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)24785, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
    p247_id_SET((uint32_t)1189015986L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)3.3389645E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -1.3248717E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float) -3.0621675E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)15434, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)201, (uint8_t)222, (uint8_t)68, (uint8_t)47, (uint8_t)18, (uint8_t)18, (uint8_t)58, (uint8_t)124, (uint8_t)241, (uint8_t)82, (uint8_t)18, (uint8_t)163, (uint8_t)64, (uint8_t)84, (uint8_t)102, (uint8_t)49, (uint8_t)104, (uint8_t)114, (uint8_t)53, (uint8_t)26, (uint8_t)68, (uint8_t)239, (uint8_t)125, (uint8_t)252, (uint8_t)205, (uint8_t)206, (uint8_t)4, (uint8_t)195, (uint8_t)27, (uint8_t)223, (uint8_t)2, (uint8_t)137, (uint8_t)161, (uint8_t)147, (uint8_t)28, (uint8_t)99, (uint8_t)36, (uint8_t)245, (uint8_t)89, (uint8_t)164, (uint8_t)244, (uint8_t)105, (uint8_t)9, (uint8_t)255, (uint8_t)36, (uint8_t)139, (uint8_t)45, (uint8_t)182, (uint8_t)248, (uint8_t)155, (uint8_t)175, (uint8_t)221, (uint8_t)52, (uint8_t)184, (uint8_t)166, (uint8_t)184, (uint8_t)130, (uint8_t)8, (uint8_t)252, (uint8_t)59, (uint8_t)38, (uint8_t)223, (uint8_t)50, (uint8_t)244, (uint8_t)163, (uint8_t)199, (uint8_t)195, (uint8_t)57, (uint8_t)92, (uint8_t)159, (uint8_t)97, (uint8_t)91, (uint8_t)127, (uint8_t)45, (uint8_t)200, (uint8_t)109, (uint8_t)155, (uint8_t)7, (uint8_t)232, (uint8_t)42, (uint8_t)238, (uint8_t)161, (uint8_t)65, (uint8_t)166, (uint8_t)21, (uint8_t)221, (uint8_t)186, (uint8_t)168, (uint8_t)227, (uint8_t)3, (uint8_t)112, (uint8_t)22, (uint8_t)64, (uint8_t)8, (uint8_t)42, (uint8_t)159, (uint8_t)115, (uint8_t)150, (uint8_t)211, (uint8_t)251, (uint8_t)25, (uint8_t)51, (uint8_t)90, (uint8_t)196, (uint8_t)84, (uint8_t)133, (uint8_t)43, (uint8_t)44, (uint8_t)107, (uint8_t)121, (uint8_t)174, (uint8_t)232, (uint8_t)229, (uint8_t)1, (uint8_t)93, (uint8_t)53, (uint8_t)156, (uint8_t)169, (uint8_t)245, (uint8_t)195, (uint8_t)142, (uint8_t)77, (uint8_t)27, (uint8_t)70, (uint8_t)174, (uint8_t)126, (uint8_t)77, (uint8_t)134, (uint8_t)17, (uint8_t)187, (uint8_t)34, (uint8_t)67, (uint8_t)111, (uint8_t)7, (uint8_t)86, (uint8_t)54, (uint8_t)109, (uint8_t)164, (uint8_t)184, (uint8_t)142, (uint8_t)190, (uint8_t)90, (uint8_t)95, (uint8_t)178, (uint8_t)2, (uint8_t)130, (uint8_t)22, (uint8_t)40, (uint8_t)6, (uint8_t)20, (uint8_t)75, (uint8_t)98, (uint8_t)107, (uint8_t)249, (uint8_t)117, (uint8_t)171, (uint8_t)100, (uint8_t)3, (uint8_t)56, (uint8_t)151, (uint8_t)139, (uint8_t)208, (uint8_t)27, (uint8_t)24, (uint8_t)157, (uint8_t)11, (uint8_t)231, (uint8_t)98, (uint8_t)147, (uint8_t)12, (uint8_t)121, (uint8_t)190, (uint8_t)246, (uint8_t)76, (uint8_t)144, (uint8_t)206, (uint8_t)49, (uint8_t)141, (uint8_t)126, (uint8_t)205, (uint8_t)183, (uint8_t)115, (uint8_t)4, (uint8_t)17, (uint8_t)187, (uint8_t)216, (uint8_t)251, (uint8_t)219, (uint8_t)131, (uint8_t)42, (uint8_t)59, (uint8_t)227, (uint8_t)133, (uint8_t)179, (uint8_t)222, (uint8_t)162, (uint8_t)140, (uint8_t)20, (uint8_t)61, (uint8_t)190, (uint8_t)220, (uint8_t)249, (uint8_t)233, (uint8_t)45, (uint8_t)102, (uint8_t)92, (uint8_t)7, (uint8_t)104, (uint8_t)93, (uint8_t)34, (uint8_t)168, (uint8_t)241, (uint8_t)245, (uint8_t)45, (uint8_t)244, (uint8_t)212, (uint8_t)248, (uint8_t)131, (uint8_t)113, (uint8_t)187, (uint8_t)18, (uint8_t)159, (uint8_t)225, (uint8_t)52, (uint8_t)223, (uint8_t)17, (uint8_t)173, (uint8_t)55, (uint8_t)146, (uint8_t)159, (uint8_t)66, (uint8_t)254, (uint8_t)246, (uint8_t)174, (uint8_t)67, (uint8_t)64, (uint8_t)201, (uint8_t)164, (uint8_t)175, (uint8_t)60, (uint8_t)121, (uint8_t)90, (uint8_t)43, (uint8_t)238, (uint8_t)165, (uint8_t)127, (uint8_t)160, (uint8_t)143, (uint8_t)47};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)36728, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t) -52, (int8_t) -49, (int8_t)105, (int8_t) -88, (int8_t)121, (int8_t)7, (int8_t) -66, (int8_t)8, (int8_t) -125, (int8_t)67, (int8_t) -1, (int8_t) -83, (int8_t)98, (int8_t) -50, (int8_t)18, (int8_t) -116, (int8_t) -124, (int8_t) -66, (int8_t)90, (int8_t) -12, (int8_t)51, (int8_t)59, (int8_t) -71, (int8_t) -121, (int8_t) -105, (int8_t)31, (int8_t)47, (int8_t) -104, (int8_t)46, (int8_t) -109, (int8_t)59, (int8_t) -5};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "b";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)6588521712384173040L, PH.base.pack) ;
    p250_x_SET((float) -2.569482E38F, PH.base.pack) ;
    p250_y_SET((float) -1.569229E37F, PH.base.pack) ;
    p250_z_SET((float)1.0506429E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3328619535L, PH.base.pack) ;
    {
        char16_t   name = "dldg";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)2.8996112E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)210686474L, PH.base.pack) ;
    {
        char16_t   name = "wQ";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -1758229798, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ALERT, PH.base.pack) ;
    {
        char16_t   text = "jpwgdhbovvdacgvxotgwnbcqtwbuogloykFyiqqgywnw";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)2759552297L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p254_value_SET((float)1.997847E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)223, (uint8_t)100, (uint8_t)28, (uint8_t)162, (uint8_t)151, (uint8_t)9, (uint8_t)239, (uint8_t)137, (uint8_t)35, (uint8_t)79, (uint8_t)15, (uint8_t)26, (uint8_t)201, (uint8_t)141, (uint8_t)60, (uint8_t)220, (uint8_t)68, (uint8_t)99, (uint8_t)206, (uint8_t)5, (uint8_t)74, (uint8_t)160, (uint8_t)130, (uint8_t)102, (uint8_t)164, (uint8_t)40, (uint8_t)151, (uint8_t)53, (uint8_t)90, (uint8_t)211, (uint8_t)40, (uint8_t)123};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)5490762747404806043L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)2807633381L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)3354635314L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    {
        char16_t   tune = "siyBrozyrNbtUlkda";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)2716232765L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)236, (uint8_t)116, (uint8_t)120, (uint8_t)202, (uint8_t)165, (uint8_t)68, (uint8_t)100, (uint8_t)181, (uint8_t)251, (uint8_t)160, (uint8_t)79, (uint8_t)40, (uint8_t)88, (uint8_t)43, (uint8_t)111, (uint8_t)132, (uint8_t)100, (uint8_t)200, (uint8_t)71, (uint8_t)232, (uint8_t)44, (uint8_t)86, (uint8_t)15, (uint8_t)142, (uint8_t)63, (uint8_t)61, (uint8_t)65, (uint8_t)32, (uint8_t)236, (uint8_t)124, (uint8_t)13, (uint8_t)186};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)209, (uint8_t)193, (uint8_t)102, (uint8_t)124, (uint8_t)143, (uint8_t)205, (uint8_t)252, (uint8_t)59, (uint8_t)209, (uint8_t)110, (uint8_t)128, (uint8_t)175, (uint8_t)116, (uint8_t)18, (uint8_t)54, (uint8_t)60, (uint8_t)156, (uint8_t)49, (uint8_t)101, (uint8_t)152, (uint8_t)87, (uint8_t)50, (uint8_t)70, (uint8_t)42, (uint8_t)145, (uint8_t)216, (uint8_t)217, (uint8_t)60, (uint8_t)118, (uint8_t)251, (uint8_t)210, (uint8_t)18};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1372645779L, PH.base.pack) ;
    p259_focal_length_SET((float) -1.9461303E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -2.554796E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -1.6133346E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)28265, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)36683, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)23596, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "tmmsjpxyNjgwkfnrfVkwxwihzhnmoagTamxtqykmaijywnmDzyryqsettrrIjkODyaojnikkzUckbCwkwWppyEgzUhewN";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)29696508L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)637164491L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p261_total_capacity_SET((float) -1.9848175E38F, PH.base.pack) ;
    p261_used_capacity_SET((float) -1.1399283E37F, PH.base.pack) ;
    p261_available_capacity_SET((float) -7.510853E37F, PH.base.pack) ;
    p261_read_speed_SET((float)1.6822314E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -2.0622704E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)4114108780L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p262_image_interval_SET((float) -2.9422757E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)936961172L, PH.base.pack) ;
    p262_available_capacity_SET((float) -2.7174628E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)1820407277L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)9137682678310092684L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p263_lat_SET((int32_t)1159819745, PH.base.pack) ;
    p263_lon_SET((int32_t)140621109, PH.base.pack) ;
    p263_alt_SET((int32_t) -1337500013, PH.base.pack) ;
    p263_relative_alt_SET((int32_t) -203481698, PH.base.pack) ;
    {
        float  q [] =  {3.2950813E38F, -2.599393E38F, -2.3872844E38F, -2.445846E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1181373849, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t) -50, PH.base.pack) ;
    {
        char16_t   file_url = "rglcpVeuvytfyxvfchqmazHbKeefytwkvunoixvngqgbvMrcogpPpyfnxxoihhnhsTPrzwevtouncoEtjvhvgavxmtkorawqmcuccoqgxllbkasxkzkbogJygjfwxmdrgnjnwrffrwzDijvlkgmrLqkduofjDjovdaWfpvVxzkdkthgqRistubas";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)2759230201L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)4237331723200465201L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)8120935666242999948L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)8140415548076203136L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)3710711070L, PH.base.pack) ;
    p265_roll_SET((float) -9.420488E37F, PH.base.pack) ;
    p265_pitch_SET((float) -1.5144317E38F, PH.base.pack) ;
    p265_yaw_SET((float)1.3833407E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)3721, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)3, (uint8_t)79, (uint8_t)105, (uint8_t)71, (uint8_t)41, (uint8_t)160, (uint8_t)87, (uint8_t)19, (uint8_t)49, (uint8_t)76, (uint8_t)157, (uint8_t)192, (uint8_t)77, (uint8_t)252, (uint8_t)242, (uint8_t)168, (uint8_t)186, (uint8_t)97, (uint8_t)56, (uint8_t)159, (uint8_t)187, (uint8_t)169, (uint8_t)9, (uint8_t)66, (uint8_t)82, (uint8_t)183, (uint8_t)227, (uint8_t)247, (uint8_t)210, (uint8_t)250, (uint8_t)158, (uint8_t)233, (uint8_t)103, (uint8_t)239, (uint8_t)205, (uint8_t)84, (uint8_t)143, (uint8_t)54, (uint8_t)18, (uint8_t)171, (uint8_t)106, (uint8_t)163, (uint8_t)222, (uint8_t)250, (uint8_t)89, (uint8_t)19, (uint8_t)245, (uint8_t)112, (uint8_t)164, (uint8_t)121, (uint8_t)85, (uint8_t)0, (uint8_t)187, (uint8_t)17, (uint8_t)77, (uint8_t)108, (uint8_t)149, (uint8_t)98, (uint8_t)39, (uint8_t)9, (uint8_t)29, (uint8_t)254, (uint8_t)14, (uint8_t)123, (uint8_t)171, (uint8_t)214, (uint8_t)119, (uint8_t)167, (uint8_t)111, (uint8_t)80, (uint8_t)1, (uint8_t)44, (uint8_t)89, (uint8_t)142, (uint8_t)254, (uint8_t)79, (uint8_t)118, (uint8_t)115, (uint8_t)184, (uint8_t)103, (uint8_t)122, (uint8_t)71, (uint8_t)32, (uint8_t)210, (uint8_t)190, (uint8_t)24, (uint8_t)96, (uint8_t)27, (uint8_t)151, (uint8_t)123, (uint8_t)32, (uint8_t)18, (uint8_t)123, (uint8_t)174, (uint8_t)114, (uint8_t)154, (uint8_t)39, (uint8_t)160, (uint8_t)63, (uint8_t)216, (uint8_t)88, (uint8_t)177, (uint8_t)3, (uint8_t)51, (uint8_t)23, (uint8_t)67, (uint8_t)249, (uint8_t)149, (uint8_t)250, (uint8_t)80, (uint8_t)191, (uint8_t)30, (uint8_t)210, (uint8_t)224, (uint8_t)67, (uint8_t)193, (uint8_t)175, (uint8_t)40, (uint8_t)181, (uint8_t)233, (uint8_t)89, (uint8_t)229, (uint8_t)41, (uint8_t)79, (uint8_t)206, (uint8_t)111, (uint8_t)235, (uint8_t)15, (uint8_t)69, (uint8_t)173, (uint8_t)156, (uint8_t)63, (uint8_t)247, (uint8_t)166, (uint8_t)29, (uint8_t)25, (uint8_t)165, (uint8_t)214, (uint8_t)16, (uint8_t)16, (uint8_t)173, (uint8_t)225, (uint8_t)117, (uint8_t)51, (uint8_t)22, (uint8_t)6, (uint8_t)64, (uint8_t)103, (uint8_t)209, (uint8_t)132, (uint8_t)142, (uint8_t)197, (uint8_t)25, (uint8_t)123, (uint8_t)2, (uint8_t)185, (uint8_t)192, (uint8_t)164, (uint8_t)91, (uint8_t)69, (uint8_t)231, (uint8_t)55, (uint8_t)145, (uint8_t)208, (uint8_t)209, (uint8_t)116, (uint8_t)162, (uint8_t)28, (uint8_t)203, (uint8_t)126, (uint8_t)86, (uint8_t)91, (uint8_t)113, (uint8_t)97, (uint8_t)198, (uint8_t)30, (uint8_t)152, (uint8_t)82, (uint8_t)135, (uint8_t)201, (uint8_t)175, (uint8_t)77, (uint8_t)167, (uint8_t)77, (uint8_t)231, (uint8_t)142, (uint8_t)93, (uint8_t)251, (uint8_t)75, (uint8_t)92, (uint8_t)188, (uint8_t)241, (uint8_t)176, (uint8_t)236, (uint8_t)149, (uint8_t)251, (uint8_t)57, (uint8_t)158, (uint8_t)134, (uint8_t)14, (uint8_t)209, (uint8_t)222, (uint8_t)136, (uint8_t)203, (uint8_t)233, (uint8_t)194, (uint8_t)109, (uint8_t)8, (uint8_t)136, (uint8_t)101, (uint8_t)0, (uint8_t)235, (uint8_t)93, (uint8_t)18, (uint8_t)154, (uint8_t)1, (uint8_t)135, (uint8_t)184, (uint8_t)137, (uint8_t)125, (uint8_t)206, (uint8_t)135, (uint8_t)12, (uint8_t)214, (uint8_t)180, (uint8_t)100, (uint8_t)48, (uint8_t)179, (uint8_t)164, (uint8_t)101, (uint8_t)241, (uint8_t)105, (uint8_t)184, (uint8_t)244, (uint8_t)66, (uint8_t)116, (uint8_t)132, (uint8_t)4, (uint8_t)123, (uint8_t)217, (uint8_t)204, (uint8_t)134, (uint8_t)26, (uint8_t)191, (uint8_t)85, (uint8_t)39, (uint8_t)38, (uint8_t)12, (uint8_t)46};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)35386, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)215, (uint8_t)180, (uint8_t)104, (uint8_t)196, (uint8_t)58, (uint8_t)196, (uint8_t)108, (uint8_t)224, (uint8_t)140, (uint8_t)247, (uint8_t)227, (uint8_t)216, (uint8_t)177, (uint8_t)244, (uint8_t)192, (uint8_t)123, (uint8_t)60, (uint8_t)242, (uint8_t)77, (uint8_t)88, (uint8_t)106, (uint8_t)173, (uint8_t)108, (uint8_t)147, (uint8_t)126, (uint8_t)30, (uint8_t)133, (uint8_t)20, (uint8_t)239, (uint8_t)200, (uint8_t)14, (uint8_t)52, (uint8_t)153, (uint8_t)119, (uint8_t)250, (uint8_t)91, (uint8_t)223, (uint8_t)44, (uint8_t)33, (uint8_t)3, (uint8_t)219, (uint8_t)183, (uint8_t)192, (uint8_t)105, (uint8_t)104, (uint8_t)161, (uint8_t)99, (uint8_t)116, (uint8_t)92, (uint8_t)215, (uint8_t)140, (uint8_t)6, (uint8_t)81, (uint8_t)118, (uint8_t)194, (uint8_t)166, (uint8_t)102, (uint8_t)186, (uint8_t)77, (uint8_t)62, (uint8_t)178, (uint8_t)27, (uint8_t)8, (uint8_t)100, (uint8_t)191, (uint8_t)77, (uint8_t)201, (uint8_t)18, (uint8_t)190, (uint8_t)139, (uint8_t)148, (uint8_t)190, (uint8_t)1, (uint8_t)241, (uint8_t)89, (uint8_t)118, (uint8_t)166, (uint8_t)219, (uint8_t)129, (uint8_t)25, (uint8_t)101, (uint8_t)106, (uint8_t)86, (uint8_t)67, (uint8_t)63, (uint8_t)30, (uint8_t)163, (uint8_t)219, (uint8_t)35, (uint8_t)120, (uint8_t)31, (uint8_t)162, (uint8_t)96, (uint8_t)69, (uint8_t)145, (uint8_t)207, (uint8_t)56, (uint8_t)103, (uint8_t)58, (uint8_t)204, (uint8_t)2, (uint8_t)78, (uint8_t)152, (uint8_t)13, (uint8_t)136, (uint8_t)192, (uint8_t)200, (uint8_t)237, (uint8_t)67, (uint8_t)32, (uint8_t)199, (uint8_t)5, (uint8_t)238, (uint8_t)48, (uint8_t)128, (uint8_t)255, (uint8_t)18, (uint8_t)207, (uint8_t)86, (uint8_t)163, (uint8_t)122, (uint8_t)162, (uint8_t)19, (uint8_t)1, (uint8_t)104, (uint8_t)124, (uint8_t)109, (uint8_t)132, (uint8_t)217, (uint8_t)16, (uint8_t)225, (uint8_t)169, (uint8_t)229, (uint8_t)183, (uint8_t)200, (uint8_t)83, (uint8_t)71, (uint8_t)13, (uint8_t)24, (uint8_t)185, (uint8_t)141, (uint8_t)12, (uint8_t)192, (uint8_t)66, (uint8_t)139, (uint8_t)164, (uint8_t)102, (uint8_t)63, (uint8_t)99, (uint8_t)255, (uint8_t)164, (uint8_t)110, (uint8_t)135, (uint8_t)146, (uint8_t)153, (uint8_t)238, (uint8_t)59, (uint8_t)127, (uint8_t)87, (uint8_t)133, (uint8_t)253, (uint8_t)136, (uint8_t)108, (uint8_t)251, (uint8_t)168, (uint8_t)198, (uint8_t)149, (uint8_t)78, (uint8_t)153, (uint8_t)154, (uint8_t)68, (uint8_t)8, (uint8_t)243, (uint8_t)225, (uint8_t)1, (uint8_t)241, (uint8_t)56, (uint8_t)246, (uint8_t)229, (uint8_t)236, (uint8_t)156, (uint8_t)57, (uint8_t)69, (uint8_t)75, (uint8_t)202, (uint8_t)117, (uint8_t)213, (uint8_t)115, (uint8_t)123, (uint8_t)116, (uint8_t)71, (uint8_t)13, (uint8_t)112, (uint8_t)131, (uint8_t)216, (uint8_t)240, (uint8_t)47, (uint8_t)23, (uint8_t)121, (uint8_t)115, (uint8_t)189, (uint8_t)161, (uint8_t)52, (uint8_t)43, (uint8_t)231, (uint8_t)114, (uint8_t)57, (uint8_t)165, (uint8_t)9, (uint8_t)14, (uint8_t)156, (uint8_t)233, (uint8_t)198, (uint8_t)229, (uint8_t)18, (uint8_t)184, (uint8_t)189, (uint8_t)252, (uint8_t)208, (uint8_t)40, (uint8_t)15, (uint8_t)240, (uint8_t)72, (uint8_t)161, (uint8_t)205, (uint8_t)63, (uint8_t)245, (uint8_t)236, (uint8_t)42, (uint8_t)194, (uint8_t)202, (uint8_t)145, (uint8_t)154, (uint8_t)205, (uint8_t)162, (uint8_t)220, (uint8_t)99, (uint8_t)140, (uint8_t)144, (uint8_t)199, (uint8_t)217, (uint8_t)42, (uint8_t)19, (uint8_t)189, (uint8_t)117, (uint8_t)180, (uint8_t)247, (uint8_t)183, (uint8_t)206};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)2871, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p269_framerate_SET((float) -8.016904E37F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)45638, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)58398, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)1581623646L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)56389, PH.base.pack) ;
    {
        char16_t   uri = "jfxunmoMifUxwlroZajskgcwhtfkqkkbapujohlluxxrdkyFcqwavaYjMbn";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p270_framerate_SET((float) -1.4451668E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)36415, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)21474, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)3061362857L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)61594, PH.base.pack) ;
    {
        char16_t   uri = "hshrsycnfqgFLzziNyogpaUmuuwwxQj";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "qjxnojjmkmbalFbUy";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "rEvisftwwpLfzimiRxrzxrediczDgliBbypjSk";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)35179, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)57096, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)13119, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)232, (uint8_t)238, (uint8_t)153, (uint8_t)46, (uint8_t)20, (uint8_t)186, (uint8_t)50, (uint8_t)32};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)45, (uint8_t)94, (uint8_t)47, (uint8_t)204, (uint8_t)140, (uint8_t)246, (uint8_t)11, (uint8_t)37};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)1522059095920886292L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)4067751897L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)30962, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)4868413398879437714L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)638926567L, PH.base.pack) ;
    {
        char16_t   name = "jbfowpqiawsEwhwfuxvsieAlrlutjMAocusdlqYiemvzpymvvlhubgveehhi";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)146, (uint8_t)136, (uint8_t)44, (uint8_t)173, (uint8_t)232, (uint8_t)52, (uint8_t)197, (uint8_t)60, (uint8_t)174, (uint8_t)181, (uint8_t)8, (uint8_t)24, (uint8_t)37, (uint8_t)67, (uint8_t)169, (uint8_t)177};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)4204181390L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    {
        char16_t   param_id = "uDzwghvfnLVyget";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t) -1888, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "nmkosepgnqcysh";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "kDqvvylvexZcdoGlyeofuondpomolwmaaifynpeWnxohhtnvhrerevkdvuoQlUavctzpcqQHtmbcpvwfdu";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)26449, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)43574, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    {
        char16_t   param_id = "siqdxvzcdtfakiD";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "vdljegvRgywspoawlo";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "atKpvblgw";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "axtatOkPsnbukdmzurnkjztcghpgprzjsuiwiyoqwmubnygqxhphzrujivQqmvbwolhpaoMoetyetgdutinfguqtkgelQjcO";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)4725771537728264383L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)63768, (uint16_t)16661, (uint16_t)38164, (uint16_t)62456, (uint16_t)51521, (uint16_t)28678, (uint16_t)22559, (uint16_t)30541, (uint16_t)41984, (uint16_t)26793, (uint16_t)15871, (uint16_t)53106, (uint16_t)38643, (uint16_t)24280, (uint16_t)44214, (uint16_t)49286, (uint16_t)34624, (uint16_t)17199, (uint16_t)20111, (uint16_t)8877, (uint16_t)53541, (uint16_t)3557, (uint16_t)34141, (uint16_t)22616, (uint16_t)5708, (uint16_t)59541, (uint16_t)25558, (uint16_t)55288, (uint16_t)28865, (uint16_t)19076, (uint16_t)9142, (uint16_t)57185, (uint16_t)14820, (uint16_t)22155, (uint16_t)44808, (uint16_t)57820, (uint16_t)52376, (uint16_t)43762, (uint16_t)43425, (uint16_t)34746, (uint16_t)31644, (uint16_t)49808, (uint16_t)37233, (uint16_t)16214, (uint16_t)54735, (uint16_t)24116, (uint16_t)23096, (uint16_t)32889, (uint16_t)17891, (uint16_t)1230, (uint16_t)30694, (uint16_t)63141, (uint16_t)22477, (uint16_t)28841, (uint16_t)11024, (uint16_t)56737, (uint16_t)18722, (uint16_t)55665, (uint16_t)24970, (uint16_t)53043, (uint16_t)57434, (uint16_t)41773, (uint16_t)33073, (uint16_t)9008, (uint16_t)933, (uint16_t)24394, (uint16_t)26611, (uint16_t)62630, (uint16_t)28543, (uint16_t)7432, (uint16_t)37727, (uint16_t)35559};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)38703, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)7774, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

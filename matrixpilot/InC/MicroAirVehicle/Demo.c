
#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p89_time_boot_ms_GET(pack);
    float  x = p89_x_GET(pack);
    float  y = p89_y_GET(pack);
    float  z = p89_z_GET(pack);
    float  roll = p89_roll_GET(pack);
    float  pitch = p89_pitch_GET(pack);
    float  yaw = p89_yaw_GET(pack);
}
void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p93_time_usec_GET(pack);
    float*  controls = p93_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
    e_MAV_MODE  mode = p93_mode_GET(pack);
    uint64_t  flags = p93_flags_GET(pack);
}
void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p101_usec_GET(pack);
    float  x = p101_x_GET(pack);
    float  y = p101_y_GET(pack);
    float  z = p101_z_GET(pack);
    float  roll = p101_roll_GET(pack);
    float  pitch = p101_pitch_GET(pack);
    float  yaw = p101_yaw_GET(pack);
}
void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p102_usec_GET(pack);
    float  x = p102_x_GET(pack);
    float  y = p102_y_GET(pack);
    float  z = p102_z_GET(pack);
    float  roll = p102_roll_GET(pack);
    float  pitch = p102_pitch_GET(pack);
    float  yaw = p102_yaw_GET(pack);
}
void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p103_usec_GET(pack);
    float  x = p103_x_GET(pack);
    float  y = p103_y_GET(pack);
    float  z = p103_z_GET(pack);
}
void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p104_usec_GET(pack);
    float  x = p104_x_GET(pack);
    float  y = p104_y_GET(pack);
    float  z = p104_z_GET(pack);
    float  roll = p104_roll_GET(pack);
    float  pitch = p104_pitch_GET(pack);
    float  yaw = p104_yaw_GET(pack);
}
void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p109_rssi_GET(pack);
    uint8_t  remrssi = p109_remrssi_GET(pack);
    uint8_t  txbuf = p109_txbuf_GET(pack);
    uint8_t  noise = p109_noise_GET(pack);
    uint8_t  remnoise = p109_remnoise_GET(pack);
    uint16_t  rxerrors = p109_rxerrors_GET(pack);
    uint16_t  fixed_ = p109_fixed__GET(pack);
}
void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p110_target_network_GET(pack);
    uint8_t  target_system = p110_target_system_GET(pack);
    uint8_t  target_component = p110_target_component_GET(pack);
    uint8_t*  payload = p110_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    int64_t  tc1 = p111_tc1_GET(pack);
    int64_t  ts1 = p111_ts1_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p112_time_usec_GET(pack);
    uint32_t  seq = p112_seq_GET(pack);
}
void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p117_target_system_GET(pack);
    uint8_t  target_component = p117_target_component_GET(pack);
    uint16_t  start = p117_start_GET(pack);
    uint16_t  end = p117_end_GET(pack);
}
void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p118_id_GET(pack);
    uint16_t  num_logs = p118_num_logs_GET(pack);
    uint16_t  last_log_num = p118_last_log_num_GET(pack);
    uint32_t  time_utc = p118_time_utc_GET(pack);
    uint32_t  size = p118_size_GET(pack);
}
void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p119_target_system_GET(pack);
    uint8_t  target_component = p119_target_component_GET(pack);
    uint16_t  id = p119_id_GET(pack);
    uint32_t  ofs = p119_ofs_GET(pack);
    uint32_t  count = p119_count_GET(pack);
}
void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p120_id_GET(pack);
    uint32_t  ofs = p120_ofs_GET(pack);
    uint8_t  count = p120_count_GET(pack);
    uint8_t*  data_ = p120_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p121_target_system_GET(pack);
    uint8_t  target_component = p121_target_component_GET(pack);
}
void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p122_target_system_GET(pack);
    uint8_t  target_component = p122_target_component_GET(pack);
}
void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p123_target_system_GET(pack);
    uint8_t  target_component = p123_target_component_GET(pack);
    uint8_t  len = p123_len_GET(pack);
    uint8_t*  data_ = p123_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p125_Vcc_GET(pack);
    uint16_t  Vservo = p125_Vservo_GET(pack);
    e_MAV_POWER_STATUS  flags = p125_flags_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p130_type_GET(pack);
    uint32_t  size = p130_size_GET(pack);
    uint16_t  width = p130_width_GET(pack);
    uint16_t  height = p130_height_GET(pack);
    uint16_t  packets = p130_packets_GET(pack);
    uint8_t  payload = p130_payload_GET(pack);
    uint8_t  jpg_quality = p130_jpg_quality_GET(pack);
}
void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seqnr = p131_seqnr_GET(pack);
    uint8_t*  data_ = p131_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p133_lat_GET(pack);
    int32_t  lon = p133_lon_GET(pack);
    uint16_t  grid_spacing = p133_grid_spacing_GET(pack);
    uint64_t  mask = p133_mask_GET(pack);
}
void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p134_lat_GET(pack);
    int32_t  lon = p134_lon_GET(pack);
    uint16_t  grid_spacing = p134_grid_spacing_GET(pack);
    uint8_t  gridbit = p134_gridbit_GET(pack);
    int16_t*  data_ = p134_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p135_lat_GET(pack);
    int32_t  lon = p135_lon_GET(pack);
}
void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p136_lat_GET(pack);
    int32_t  lon = p136_lon_GET(pack);
    uint16_t  spacing = p136_spacing_GET(pack);
    float  terrain_height = p136_terrain_height_GET(pack);
    float  current_height = p136_current_height_GET(pack);
    uint16_t  pending = p136_pending_GET(pack);
    uint16_t  loaded = p136_loaded_GET(pack);
}
void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p137_time_boot_ms_GET(pack);
    float  press_abs = p137_press_abs_GET(pack);
    float  press_diff = p137_press_diff_GET(pack);
    int16_t  temperature = p137_temperature_GET(pack);
}
void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p138_time_usec_GET(pack);
    float*  q = p138_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  x = p138_x_GET(pack);
    float  y = p138_y_GET(pack);
    float  z = p138_z_GET(pack);
}
void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p139_time_usec_GET(pack);
    uint8_t  group_mlx = p139_group_mlx_GET(pack);
    uint8_t  target_system = p139_target_system_GET(pack);
    uint8_t  target_component = p139_target_component_GET(pack);
    float*  controls = p139_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p140_time_usec_GET(pack);
    uint8_t  group_mlx = p140_group_mlx_GET(pack);
    float*  controls = p140_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p141_time_usec_GET(pack);
    float  altitude_monotonic = p141_altitude_monotonic_GET(pack);
    float  altitude_amsl = p141_altitude_amsl_GET(pack);
    float  altitude_local = p141_altitude_local_GET(pack);
    float  altitude_relative = p141_altitude_relative_GET(pack);
    float  altitude_terrain = p141_altitude_terrain_GET(pack);
    float  bottom_clearance = p141_bottom_clearance_GET(pack);
}
void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p143_time_boot_ms_GET(pack);
    float  press_abs = p143_press_abs_GET(pack);
    float  press_diff = p143_press_diff_GET(pack);
    int16_t  temperature = p143_temperature_GET(pack);
}
void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p150_target_system_GET(pack);
    uint8_t  target_component = p150_target_component_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  read_req_type = p151_read_req_type_GET(pack);
    int16_t  data_index = p151_data_index_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p153_target_system_GET(pack);
    uint8_t  target_component = p153_target_component_GET(pack);
    uint16_t  func_index = p153_func_index_GET(pack);
    uint16_t  result = p153_result_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    uint8_t  directory_type = p156_directory_type_GET(pack);
    uint8_t  start_index = p156_start_index_GET(pack);
    uint8_t  count = p156_count_GET(pack);
    uint16_t  result = p156_result_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    uint8_t  command_type = p157_command_type_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  command_type = p158_command_type_GET(pack);
    uint16_t  result = p158_result_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_AILERON = p173_sue_YAWKP_AILERON_GET(pack);
    float  sue_YAWKD_AILERON = p173_sue_YAWKD_AILERON_GET(pack);
    float  sue_ROLLKP = p173_sue_ROLLKP_GET(pack);
    float  sue_ROLLKD = p173_sue_ROLLKD_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    float  sue_PITCHGAIN = p174_sue_PITCHGAIN_GET(pack);
    float  sue_PITCHKD = p174_sue_PITCHKD_GET(pack);
    float  sue_RUDDER_ELEV_MIX = p174_sue_RUDDER_ELEV_MIX_GET(pack);
    float  sue_ROLL_ELEV_MIX = p174_sue_ROLL_ELEV_MIX_GET(pack);
    float  sue_ELEVATOR_BOOST = p174_sue_ELEVATOR_BOOST_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_RUDDER = p175_sue_YAWKP_RUDDER_GET(pack);
    float  sue_YAWKD_RUDDER = p175_sue_YAWKD_RUDDER_GET(pack);
    float  sue_ROLLKP_RUDDER = p175_sue_ROLLKP_RUDDER_GET(pack);
    float  sue_ROLLKD_RUDDER = p175_sue_ROLLKD_RUDDER_GET(pack);
    float  sue_RUDDER_BOOST = p175_sue_RUDDER_BOOST_GET(pack);
    float  sue_RTL_PITCH_DOWN = p175_sue_RTL_PITCH_DOWN_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    float  sue_HEIGHT_TARGET_MAX = p176_sue_HEIGHT_TARGET_MAX_GET(pack);
    float  sue_HEIGHT_TARGET_MIN = p176_sue_HEIGHT_TARGET_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MIN = p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MAX = p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_MIN = p176_sue_ALT_HOLD_PITCH_MIN_GET(pack);
    float  sue_ALT_HOLD_PITCH_MAX = p176_sue_ALT_HOLD_PITCH_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_HIGH = p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_week_no = p177_sue_week_no_GET(pack);
    int32_t  sue_lat_origin = p177_sue_lat_origin_GET(pack);
    int32_t  sue_lon_origin = p177_sue_lon_origin_GET(pack);
    int32_t  sue_alt_origin = p177_sue_alt_origin_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_VEHICLE_MODEL_NAME = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
//process data in sue_ID_VEHICLE_MODEL_NAME
    free(sue_ID_VEHICLE_MODEL_NAME);//never forget to dispose
    uint8_t*  sue_ID_VEHICLE_REGISTRATION = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
//process data in sue_ID_VEHICLE_REGISTRATION
    free(sue_ID_VEHICLE_REGISTRATION);//never forget to dispose
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_LEAD_PILOT = p180_sue_ID_LEAD_PILOT_GET_(pack);
//process data in sue_ID_LEAD_PILOT
    free(sue_ID_LEAD_PILOT);//never forget to dispose
    uint8_t*  sue_ID_DIY_DRONES_URL = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
//process data in sue_ID_DIY_DRONES_URL
    free(sue_ID_DIY_DRONES_URL);//never forget to dispose
}
void c_CommunicationChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p181_time_boot_ms_GET(pack);
    int32_t  alt_gps = p181_alt_gps_GET(pack);
    int32_t  alt_imu = p181_alt_imu_GET(pack);
    int32_t  alt_barometric = p181_alt_barometric_GET(pack);
    int32_t  alt_optical_flow = p181_alt_optical_flow_GET(pack);
    int32_t  alt_range_finder = p181_alt_range_finder_GET(pack);
    int32_t  alt_extra = p181_alt_extra_GET(pack);
}
void c_CommunicationChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p182_time_boot_ms_GET(pack);
    int16_t  airspeed_imu = p182_airspeed_imu_GET(pack);
    int16_t  airspeed_pitot = p182_airspeed_pitot_GET(pack);
    int16_t  airspeed_hot_wire = p182_airspeed_hot_wire_GET(pack);
    int16_t  airspeed_ultrasonic = p182_airspeed_ultrasonic_GET(pack);
    int16_t  aoa = p182_aoa_GET(pack);
    int16_t  aoy = p182_aoy_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    float  sue_feed_forward = p183_sue_feed_forward_GET(pack);
    float  sue_turn_rate_nav = p183_sue_turn_rate_nav_GET(pack);
    float  sue_turn_rate_fbw = p183_sue_turn_rate_fbw_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    float  angle_of_attack_normal = p184_angle_of_attack_normal_GET(pack);
    float  angle_of_attack_inverted = p184_angle_of_attack_inverted_GET(pack);
    float  elevator_trim_normal = p184_elevator_trim_normal_GET(pack);
    float  elevator_trim_inverted = p184_elevator_trim_inverted_GET(pack);
    float  reference_speed = p184_reference_speed_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_offset = p187_sue_accel_x_offset_GET(pack);
    int16_t  sue_accel_y_offset = p187_sue_accel_y_offset_GET(pack);
    int16_t  sue_accel_z_offset = p187_sue_accel_z_offset_GET(pack);
    int16_t  sue_gyro_x_offset = p187_sue_gyro_x_offset_GET(pack);
    int16_t  sue_gyro_y_offset = p187_sue_gyro_y_offset_GET(pack);
    int16_t  sue_gyro_z_offset = p187_sue_gyro_z_offset_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_at_calibration = p188_sue_accel_x_at_calibration_GET(pack);
    int16_t  sue_accel_y_at_calibration = p188_sue_accel_y_at_calibration_GET(pack);
    int16_t  sue_accel_z_at_calibration = p188_sue_accel_z_at_calibration_GET(pack);
    int16_t  sue_gyro_x_at_calibration = p188_sue_gyro_x_at_calibration_GET(pack);
    int16_t  sue_gyro_y_at_calibration = p188_sue_gyro_y_at_calibration_GET(pack);
    int16_t  sue_gyro_z_at_calibration = p188_sue_gyro_z_at_calibration_GET(pack);
}
void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  flags = p233_flags_GET(pack);
    uint8_t  len = p233_len_GET(pack);
    uint8_t*  data_ = p233_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p241_time_usec_GET(pack);
    float  vibration_x = p241_vibration_x_GET(pack);
    float  vibration_y = p241_vibration_y_GET(pack);
    float  vibration_z = p241_vibration_z_GET(pack);
    uint32_t  clipping_0 = p241_clipping_0_GET(pack);
    uint32_t  clipping_1 = p241_clipping_1_GET(pack);
    uint32_t  clipping_2 = p241_clipping_2_GET(pack);
}
void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  message_id = p244_message_id_GET(pack);
    int32_t  interval_us = p244_interval_us_GET(pack);
}
void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_VTOL_STATE  vtol_state = p245_vtol_state_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p245_landed_state_GET(pack);
}
void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_COLLISION_SRC  src_ = p247_src__GET(pack);
    uint32_t  id = p247_id_GET(pack);
    e_MAV_COLLISION_ACTION  action = p247_action_GET(pack);
    e_MAV_COLLISION_THREAT_LEVEL  threat_level = p247_threat_level_GET(pack);
    float  time_to_minimum_delta = p247_time_to_minimum_delta_GET(pack);
    float  altitude_minimum_delta = p247_altitude_minimum_delta_GET(pack);
    float  horizontal_minimum_delta = p247_horizontal_minimum_delta_GET(pack);
}
void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p248_target_network_GET(pack);
    uint8_t  target_system = p248_target_system_GET(pack);
    uint8_t  target_component = p248_target_component_GET(pack);
    uint16_t  message_type = p248_message_type_GET(pack);
    uint8_t*  payload = p248_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  address = p249_address_GET(pack);
    uint8_t  ver = p249_ver_GET(pack);
    uint8_t  type = p249_type_GET(pack);
    int8_t*  value = p249_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  name = p250_name_TRY_(ph);
    uint64_t  time_usec = p250_time_usec_GET(pack);
    float  x = p250_x_GET(pack);
    float  y = p250_y_GET(pack);
    float  z = p250_z_GET(pack);
}
void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p251_time_boot_ms_GET(pack);
    char16_t *  name = p251_name_TRY_(ph);
    float  value = p251_value_GET(pack);
}
void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p252_time_boot_ms_GET(pack);
    char16_t *  name = p252_name_TRY_(ph);
    int32_t  value = p252_value_GET(pack);
}
void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SEVERITY  severity = p253_severity_GET(pack);
    char16_t *  text = p253_text_TRY_(ph);
}
void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p254_time_boot_ms_GET(pack);
    uint8_t  ind = p254_ind_GET(pack);
    float  value = p254_value_GET(pack);
}
void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p256_target_system_GET(pack);
    uint8_t  target_component = p256_target_component_GET(pack);
    uint8_t*  secret_key = p256_secret_key_GET_(pack);
//process data in secret_key
    free(secret_key);//never forget to dispose
    uint64_t  initial_timestamp = p256_initial_timestamp_GET(pack);
}
void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p257_time_boot_ms_GET(pack);
    uint32_t  last_change_ms = p257_last_change_ms_GET(pack);
    uint8_t  state = p257_state_GET(pack);
}
void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p258_target_system_GET(pack);
    uint8_t  target_component = p258_target_component_GET(pack);
    char16_t *  tune = p258_tune_TRY_(ph);
}
void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p260_time_boot_ms_GET(pack);
    e_CAMERA_MODE  mode_id = p260_mode_id_GET(pack);
}
void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p262_time_boot_ms_GET(pack);
    uint8_t  image_status = p262_image_status_GET(pack);
    uint8_t  video_status = p262_video_status_GET(pack);
    float  image_interval = p262_image_interval_GET(pack);
    uint32_t  recording_time_ms = p262_recording_time_ms_GET(pack);
    float  available_capacity = p262_available_capacity_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p264_time_boot_ms_GET(pack);
    uint64_t  arming_time_utc = p264_arming_time_utc_GET(pack);
    uint64_t  takeoff_time_utc = p264_takeoff_time_utc_GET(pack);
    uint64_t  flight_uuid = p264_flight_uuid_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p265_time_boot_ms_GET(pack);
    float  roll = p265_roll_GET(pack);
    float  pitch = p265_pitch_GET(pack);
    float  yaw = p265_yaw_GET(pack);
}
void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p268_target_system_GET(pack);
    uint8_t  target_component = p268_target_component_GET(pack);
    uint16_t  sequence = p268_sequence_GET(pack);
}
void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  ssid = p299_ssid_TRY_(ph);
    char16_t *  password = p299_password_TRY_(ph);
}
void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p310_time_usec_GET(pack);
    uint32_t  uptime_sec = p310_uptime_sec_GET(pack);
    e_UAVCAN_NODE_HEALTH  health = p310_health_GET(pack);
    e_UAVCAN_NODE_MODE  mode = p310_mode_GET(pack);
    uint8_t  sub_mode = p310_sub_mode_GET(pack);
    uint16_t  vendor_specific_status_code = p310_vendor_specific_status_code_GET(pack);
}
void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p320_target_system_GET(pack);
    uint8_t  target_component = p320_target_component_GET(pack);
    char16_t *  param_id = p320_param_id_TRY_(ph);
    int16_t  param_index = p320_param_index_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p321_target_system_GET(pack);
    uint8_t  target_component = p321_target_component_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p322_param_id_TRY_(ph);
    char16_t *  param_value = p322_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p322_param_type_GET(pack);
    uint16_t  param_count = p322_param_count_GET(pack);
    uint16_t  param_index = p322_param_index_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p323_target_system_GET(pack);
    uint8_t  target_component = p323_target_component_GET(pack);
    char16_t *  param_id = p323_param_id_TRY_(ph);
    char16_t *  param_value = p323_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p323_param_type_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p324_param_id_TRY_(ph);
    char16_t *  param_value = p324_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p324_param_type_GET(pack);
    e_PARAM_ACK  param_result = p324_param_result_GET(pack);
}
void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
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
    setPack(c_CommunicationChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_QUADROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARMAZILA, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2372560875L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)30435, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)5855, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)11096, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -100, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)37098, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)14321, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)39492, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)17366, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)32690, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)60883, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)748396822473461165L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)32035545L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)1949311602L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)310, PH.base.pack) ;
    p3_x_SET((float)2.5616251E38F, PH.base.pack) ;
    p3_y_SET((float) -1.0284106E38F, PH.base.pack) ;
    p3_z_SET((float)1.5157496E38F, PH.base.pack) ;
    p3_vx_SET((float)3.0906087E37F, PH.base.pack) ;
    p3_vy_SET((float)2.5243071E38F, PH.base.pack) ;
    p3_vz_SET((float)2.675062E38F, PH.base.pack) ;
    p3_afx_SET((float) -2.4705636E37F, PH.base.pack) ;
    p3_afy_SET((float) -1.728721E38F, PH.base.pack) ;
    p3_afz_SET((float) -2.3472096E38F, PH.base.pack) ;
    p3_yaw_SET((float) -2.1008157E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -1.21653E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)7133146906044101892L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2573540610L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    {
        char16_t   passkey = "varjivvojtHl";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "rjdvpknkqdgpdqjHQoPgFSyh";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3019816856L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    {
        char16_t   param_id = "ssjr";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -16671, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "sbmbgo";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)2.3685684E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)32758, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)11384, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    {
        char16_t   param_id = "E";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -3.8594948E37F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)8320522091492175269L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
    p24_lat_SET((int32_t) -2032942262, PH.base.pack) ;
    p24_lon_SET((int32_t)711045041, PH.base.pack) ;
    p24_alt_SET((int32_t) -875771320, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)60043, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)35994, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)54542, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)64800, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1023682327, &PH) ;
    p24_h_acc_SET((uint32_t)786945513L, &PH) ;
    p24_v_acc_SET((uint32_t)728815781L, &PH) ;
    p24_vel_acc_SET((uint32_t)2719553363L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1052955133L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)210, (uint8_t)197, (uint8_t)23, (uint8_t)222, (uint8_t)175, (uint8_t)76, (uint8_t)5, (uint8_t)107, (uint8_t)233, (uint8_t)121, (uint8_t)66, (uint8_t)45, (uint8_t)179, (uint8_t)153, (uint8_t)246, (uint8_t)179, (uint8_t)65, (uint8_t)159, (uint8_t)142, (uint8_t)143};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)152, (uint8_t)99, (uint8_t)70, (uint8_t)35, (uint8_t)247, (uint8_t)85, (uint8_t)239, (uint8_t)238, (uint8_t)208, (uint8_t)190, (uint8_t)167, (uint8_t)149, (uint8_t)88, (uint8_t)34, (uint8_t)180, (uint8_t)74, (uint8_t)222, (uint8_t)33, (uint8_t)131, (uint8_t)189};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)124, (uint8_t)101, (uint8_t)90, (uint8_t)81, (uint8_t)92, (uint8_t)82, (uint8_t)80, (uint8_t)26, (uint8_t)198, (uint8_t)249, (uint8_t)174, (uint8_t)228, (uint8_t)20, (uint8_t)93, (uint8_t)128, (uint8_t)118, (uint8_t)79, (uint8_t)42, (uint8_t)159, (uint8_t)132};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)212, (uint8_t)175, (uint8_t)220, (uint8_t)113, (uint8_t)151, (uint8_t)140, (uint8_t)214, (uint8_t)114, (uint8_t)244, (uint8_t)13, (uint8_t)254, (uint8_t)50, (uint8_t)173, (uint8_t)62, (uint8_t)165, (uint8_t)21, (uint8_t)103, (uint8_t)34, (uint8_t)253, (uint8_t)132};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)254, (uint8_t)5, (uint8_t)211, (uint8_t)72, (uint8_t)114, (uint8_t)114, (uint8_t)96, (uint8_t)160, (uint8_t)163, (uint8_t)6, (uint8_t)48, (uint8_t)161, (uint8_t)59, (uint8_t)125, (uint8_t)111, (uint8_t)171, (uint8_t)13, (uint8_t)7, (uint8_t)21, (uint8_t)13};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)785335536L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -25658, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t) -76, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -21985, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -18421, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -20297, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)22590, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)7820, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -5080, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -31643, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)2749411830657643171L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -12840, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)364, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)1864, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -30317, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)30892, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -31352, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -29047, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -31087, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)12391, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)7487273587252337377L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)12795, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)28916, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -18727, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -7730, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2501529717L, PH.base.pack) ;
    p29_press_abs_SET((float) -1.1504747E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -3.3489284E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)12378, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2461259049L, PH.base.pack) ;
    p30_roll_SET((float)7.2224596E36F, PH.base.pack) ;
    p30_pitch_SET((float) -2.9453472E38F, PH.base.pack) ;
    p30_yaw_SET((float)1.89198E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)2.1593572E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.1136256E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -3.7410157E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)4228621527L, PH.base.pack) ;
    p31_q1_SET((float)9.162724E37F, PH.base.pack) ;
    p31_q2_SET((float) -1.1364821E38F, PH.base.pack) ;
    p31_q3_SET((float)2.469094E38F, PH.base.pack) ;
    p31_q4_SET((float)2.9240113E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)1.6127207E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)2.3115343E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)3.0310597E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1535378642L, PH.base.pack) ;
    p32_x_SET((float) -5.6521924E37F, PH.base.pack) ;
    p32_y_SET((float)7.3635886E37F, PH.base.pack) ;
    p32_z_SET((float) -2.799873E38F, PH.base.pack) ;
    p32_vx_SET((float)2.8930421E38F, PH.base.pack) ;
    p32_vy_SET((float)2.976522E38F, PH.base.pack) ;
    p32_vz_SET((float)1.0471822E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)2343533153L, PH.base.pack) ;
    p33_lat_SET((int32_t)1123347693, PH.base.pack) ;
    p33_lon_SET((int32_t) -1488405480, PH.base.pack) ;
    p33_alt_SET((int32_t) -2122012843, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)1181190700, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -6309, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -5902, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -6902, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)25825, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)656124379L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -19477, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -4253, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -24742, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)16443, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -5966, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -7992, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)15843, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -25786, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)1698278386L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)10731, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)10468, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)40372, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)16000, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)35452, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)57477, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)45334, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)53693, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)358703904L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)2549, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)31330, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)10055, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)18750, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)57786, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)54834, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)22260, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)21884, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)35228, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)4678, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)58875, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)38282, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)19218, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)3708, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)28325, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)55067, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -28814, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -26129, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -3951, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)32354, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)27954, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p39_param1_SET((float) -2.562667E38F, PH.base.pack) ;
    p39_param2_SET((float) -5.6825663E37F, PH.base.pack) ;
    p39_param3_SET((float) -2.2923082E38F, PH.base.pack) ;
    p39_param4_SET((float)8.851787E37F, PH.base.pack) ;
    p39_x_SET((float) -1.9277323E38F, PH.base.pack) ;
    p39_y_SET((float)1.5961683E38F, PH.base.pack) ;
    p39_z_SET((float)1.4316161E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)39534, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)53334, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)39918, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)19062, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)37543, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p48_latitude_SET((int32_t) -1899596033, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1982296536, PH.base.pack) ;
    p48_altitude_SET((int32_t) -392252722, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)3549757642485415029L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)617657907, PH.base.pack) ;
    p49_longitude_SET((int32_t) -1267226399, PH.base.pack) ;
    p49_altitude_SET((int32_t)437049110, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)2980358471245105435L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    {
        char16_t   param_id = "zkmy";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)7548, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p50_param_value0_SET((float)4.044516E37F, PH.base.pack) ;
    p50_scale_SET((float) -2.8352314E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)6.9458886E37F, PH.base.pack) ;
    p50_param_value_max_SET((float) -2.0113849E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)31184, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p54_p1x_SET((float)1.9207803E38F, PH.base.pack) ;
    p54_p1y_SET((float) -2.6424775E37F, PH.base.pack) ;
    p54_p1z_SET((float)5.132204E36F, PH.base.pack) ;
    p54_p2x_SET((float) -9.784824E37F, PH.base.pack) ;
    p54_p2y_SET((float) -8.712563E37F, PH.base.pack) ;
    p54_p2z_SET((float) -4.785458E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p55_p1x_SET((float)1.0416336E38F, PH.base.pack) ;
    p55_p1y_SET((float)1.7137778E38F, PH.base.pack) ;
    p55_p1z_SET((float) -1.3567299E37F, PH.base.pack) ;
    p55_p2x_SET((float)2.9147796E38F, PH.base.pack) ;
    p55_p2y_SET((float) -1.887209E38F, PH.base.pack) ;
    p55_p2z_SET((float)1.2003752E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)4162175186991569709L, PH.base.pack) ;
    {
        float  q [] =  {2.202815E38F, -3.6625088E37F, 1.9193573E38F, 6.5156607E37F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -3.3636423E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -3.2823088E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.2965067E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.1559706E38F, 1.6947745E38F, 2.9991583E38F, 2.6319797E38F, 4.490633E36F, -1.1794259E38F, -4.722573E37F, -1.8453705E38F, 2.974957E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.3091824E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)8.609527E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)30956, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -30117, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)25332, PH.base.pack) ;
    p62_alt_error_SET((float)8.787762E37F, PH.base.pack) ;
    p62_aspd_error_SET((float) -2.7286035E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)4.3570094E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)6322397914149359609L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p63_lat_SET((int32_t)1926342261, PH.base.pack) ;
    p63_lon_SET((int32_t) -627977807, PH.base.pack) ;
    p63_alt_SET((int32_t)1583733090, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)1323198532, PH.base.pack) ;
    p63_vx_SET((float) -4.7695374E37F, PH.base.pack) ;
    p63_vy_SET((float)7.833849E37F, PH.base.pack) ;
    p63_vz_SET((float) -2.3327494E37F, PH.base.pack) ;
    {
        float  covariance [] =  {3.0468962E38F, 3.3738024E38F, -2.7859123E38F, 2.529197E37F, 2.6164501E38F, 6.9423975E37F, 1.9390055E38F, -2.0036504E38F, 2.9996693E37F, -1.1042302E38F, -1.3626161E38F, -2.7227028E38F, 1.6573844E38F, -9.145625E37F, -2.527956E38F, -1.4442246E38F, 1.05466755E37F, 1.7938163E38F, 4.549856E37F, 1.1037968E38F, 5.100089E37F, -6.772994E36F, -3.3190604E38F, 2.8560606E38F, 1.5238922E38F, -2.2328132E38F, 1.6583394E38F, -2.2116735E38F, -3.143511E38F, 2.6780072E38F, -1.6128972E38F, -9.651769E37F, 2.8219334E38F, -1.216276E36F, 2.4036282E38F, -1.6905013E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)727015386920715943L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p64_x_SET((float) -7.2831385E37F, PH.base.pack) ;
    p64_y_SET((float)1.5063589E38F, PH.base.pack) ;
    p64_z_SET((float)1.3794093E38F, PH.base.pack) ;
    p64_vx_SET((float)1.7860282E38F, PH.base.pack) ;
    p64_vy_SET((float)6.2955446E36F, PH.base.pack) ;
    p64_vz_SET((float) -2.2228322E38F, PH.base.pack) ;
    p64_ax_SET((float) -2.0276286E38F, PH.base.pack) ;
    p64_ay_SET((float) -1.3917428E37F, PH.base.pack) ;
    p64_az_SET((float) -1.4575265E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.3730211E38F, 2.8324965E38F, 3.3011396E38F, -4.2212112E36F, -1.2965424E38F, 1.6060366E38F, -3.005902E37F, 1.4121407E38F, -2.889412E38F, -2.7245673E38F, -1.5997118E38F, 2.8480424E38F, -1.4795963E38F, 2.1344478E38F, -4.975728E37F, 2.5887819E38F, -3.4901365E37F, 8.1856327E37F, 2.449402E38F, -1.2884509E38F, 9.605526E37F, -1.3287611E38F, 2.0077914E38F, -2.780111E38F, -2.8198967E38F, 1.6101897E38F, 3.1855924E38F, -3.3693504E38F, -9.882066E37F, 1.4977488E38F, 2.6852797E38F, -1.0313529E38F, 9.643728E37F, 3.3708643E38F, -7.823266E37F, 2.2650062E38F, 1.918694E38F, -2.6307072E38F, 1.1645417E38F, -1.00065746E37F, 4.437388E36F, 1.1324074E38F, 2.3943954E38F, 2.8255086E38F, 1.7345756E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)1821136078L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)22725, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)38168, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)42863, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)57891, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)45499, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)39046, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)12723, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)44077, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)24748, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)874, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)42346, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)32188, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)20194, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)15273, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)51452, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)20989, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)52857, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)2911, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)51083, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)24993, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)641, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)780, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)12757, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -19774, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)20037, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)13236, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)13958, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)48633, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)53315, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)7298, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)61175, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)35169, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)11375, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)13483, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p73_param1_SET((float)1.730147E38F, PH.base.pack) ;
    p73_param2_SET((float) -2.8131303E38F, PH.base.pack) ;
    p73_param3_SET((float) -1.902858E37F, PH.base.pack) ;
    p73_param4_SET((float)2.4512923E38F, PH.base.pack) ;
    p73_x_SET((int32_t)1181531945, PH.base.pack) ;
    p73_y_SET((int32_t) -625165687, PH.base.pack) ;
    p73_z_SET((float)2.8659919E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)4.7137567E37F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.7479918E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -10652, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)8941, PH.base.pack) ;
    p74_alt_SET((float)2.4565728E38F, PH.base.pack) ;
    p74_climb_SET((float) -5.8789066E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_SPATIAL_USER_4, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p75_param1_SET((float) -3.0962992E37F, PH.base.pack) ;
    p75_param2_SET((float)6.715227E36F, PH.base.pack) ;
    p75_param3_SET((float) -1.7653174E38F, PH.base.pack) ;
    p75_param4_SET((float) -1.6673622E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1426603208, PH.base.pack) ;
    p75_y_SET((int32_t)755048126, PH.base.pack) ;
    p75_z_SET((float)2.0387085E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p76_param1_SET((float) -2.8900038E38F, PH.base.pack) ;
    p76_param2_SET((float)1.7048692E38F, PH.base.pack) ;
    p76_param3_SET((float)1.5647043E38F, PH.base.pack) ;
    p76_param4_SET((float)2.7225577E38F, PH.base.pack) ;
    p76_param5_SET((float)3.235254E38F, PH.base.pack) ;
    p76_param6_SET((float) -8.474615E37F, PH.base.pack) ;
    p76_param7_SET((float)1.4467754E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)58, &PH) ;
    p77_result_param2_SET((int32_t)612595856, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)243, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)69, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)925177475L, PH.base.pack) ;
    p81_roll_SET((float) -2.7442978E38F, PH.base.pack) ;
    p81_pitch_SET((float)2.5623247E37F, PH.base.pack) ;
    p81_yaw_SET((float)1.9995949E38F, PH.base.pack) ;
    p81_thrust_SET((float)2.0679742E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)3696879021L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    {
        float  q [] =  {-1.1127314E38F, 2.4014126E38F, -4.5260963E37F, 2.7404939E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)2.2254036E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)2.0812965E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -8.753924E37F, PH.base.pack) ;
    p82_thrust_SET((float)1.4918876E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)4234144528L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        float  q [] =  {-2.2328296E38F, 1.5632083E38F, -2.6435795E38F, 2.2940034E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.4588546E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -9.942512E37F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -6.2425616E37F, PH.base.pack) ;
    p83_thrust_SET((float) -1.6028629E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)2983693288L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)8146, PH.base.pack) ;
    p84_x_SET((float) -2.9845967E38F, PH.base.pack) ;
    p84_y_SET((float) -1.3481812E38F, PH.base.pack) ;
    p84_z_SET((float) -2.9373693E38F, PH.base.pack) ;
    p84_vx_SET((float)6.818487E37F, PH.base.pack) ;
    p84_vy_SET((float)2.2512189E38F, PH.base.pack) ;
    p84_vz_SET((float)3.0998688E38F, PH.base.pack) ;
    p84_afx_SET((float)3.3656233E38F, PH.base.pack) ;
    p84_afy_SET((float)7.0487037E37F, PH.base.pack) ;
    p84_afz_SET((float) -3.09008E38F, PH.base.pack) ;
    p84_yaw_SET((float)9.528072E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)1.2618044E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)410060845L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)16804, PH.base.pack) ;
    p86_lat_int_SET((int32_t)551527042, PH.base.pack) ;
    p86_lon_int_SET((int32_t)1671735514, PH.base.pack) ;
    p86_alt_SET((float)3.119552E38F, PH.base.pack) ;
    p86_vx_SET((float)1.3153313E38F, PH.base.pack) ;
    p86_vy_SET((float)2.7124695E38F, PH.base.pack) ;
    p86_vz_SET((float) -1.6176387E38F, PH.base.pack) ;
    p86_afx_SET((float)5.994986E37F, PH.base.pack) ;
    p86_afy_SET((float)3.271364E38F, PH.base.pack) ;
    p86_afz_SET((float) -6.158595E37F, PH.base.pack) ;
    p86_yaw_SET((float)1.9485139E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)2.4300228E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)537178058L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)41737, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -1474713817, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -524455709, PH.base.pack) ;
    p87_alt_SET((float) -1.7766764E38F, PH.base.pack) ;
    p87_vx_SET((float) -1.6398652E38F, PH.base.pack) ;
    p87_vy_SET((float) -1.8231114E38F, PH.base.pack) ;
    p87_vz_SET((float)5.6428364E36F, PH.base.pack) ;
    p87_afx_SET((float) -3.1622672E38F, PH.base.pack) ;
    p87_afy_SET((float) -2.8055872E38F, PH.base.pack) ;
    p87_afz_SET((float)7.8236114E37F, PH.base.pack) ;
    p87_yaw_SET((float) -2.87704E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)3.1937808E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1951118401L, PH.base.pack) ;
    p89_x_SET((float)2.0380051E38F, PH.base.pack) ;
    p89_y_SET((float) -3.2356236E38F, PH.base.pack) ;
    p89_z_SET((float) -1.4064257E38F, PH.base.pack) ;
    p89_roll_SET((float)1.6641493E38F, PH.base.pack) ;
    p89_pitch_SET((float) -1.7464656E38F, PH.base.pack) ;
    p89_yaw_SET((float)2.8693987E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)1884725686135311622L, PH.base.pack) ;
    p90_roll_SET((float) -1.6861863E38F, PH.base.pack) ;
    p90_pitch_SET((float)1.7511874E37F, PH.base.pack) ;
    p90_yaw_SET((float)1.9481646E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)1.7331806E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.2175482E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)1.8245255E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)172166341, PH.base.pack) ;
    p90_lon_SET((int32_t)1877260244, PH.base.pack) ;
    p90_alt_SET((int32_t) -548391737, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -4867, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -19657, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)20185, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)19556, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -25833, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)6901, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)3059617004087893390L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -6.499919E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -1.3993328E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.5802178E38F, PH.base.pack) ;
    p91_throttle_SET((float) -3.0173224E38F, PH.base.pack) ;
    p91_aux1_SET((float) -3.7948997E37F, PH.base.pack) ;
    p91_aux2_SET((float) -1.0956838E38F, PH.base.pack) ;
    p91_aux3_SET((float)2.1617603E38F, PH.base.pack) ;
    p91_aux4_SET((float) -2.527618E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)5541628298773770216L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)35789, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)17301, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)26589, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)6237, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)33059, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)54063, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)37191, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)54998, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)38451, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)36657, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)36972, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)26934, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)8297693285883910573L, PH.base.pack) ;
    {
        float  controls [] =  {-2.6377692E38F, 5.8273254E37F, 1.2718354E36F, -2.5162832E38F, -4.6059116E37F, 1.328688E38F, 1.5501988E38F, -2.9414366E38F, 1.5581967E38F, 1.1333336E38F, 9.106611E37F, -2.5566728E38F, -2.6077315E38F, 1.4411932E37F, 2.9579205E38F, 2.5834376E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
    p93_flags_SET((uint64_t)5144102929198213130L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)8706504730389186851L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -11644, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -5386, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)2.8707019E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -2.9897598E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p100_ground_distance_SET((float) -2.1333963E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -1.5556299E38F, &PH) ;
    p100_flow_rate_y_SET((float) -2.0563686E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)3409895205802141180L, PH.base.pack) ;
    p101_x_SET((float) -1.8016299E37F, PH.base.pack) ;
    p101_y_SET((float)4.6115856E37F, PH.base.pack) ;
    p101_z_SET((float) -2.3086043E38F, PH.base.pack) ;
    p101_roll_SET((float)2.0300341E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.3597972E38F, PH.base.pack) ;
    p101_yaw_SET((float)1.044103E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)5611428921188246428L, PH.base.pack) ;
    p102_x_SET((float) -3.0309085E37F, PH.base.pack) ;
    p102_y_SET((float) -2.5894025E38F, PH.base.pack) ;
    p102_z_SET((float) -3.9085593E37F, PH.base.pack) ;
    p102_roll_SET((float) -1.1972073E38F, PH.base.pack) ;
    p102_pitch_SET((float)1.5449217E38F, PH.base.pack) ;
    p102_yaw_SET((float)5.972019E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)2372669449302784954L, PH.base.pack) ;
    p103_x_SET((float)2.2966328E38F, PH.base.pack) ;
    p103_y_SET((float) -2.1002762E38F, PH.base.pack) ;
    p103_z_SET((float) -2.6062343E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)1734554782546288790L, PH.base.pack) ;
    p104_x_SET((float)2.5593983E38F, PH.base.pack) ;
    p104_y_SET((float) -1.0229012E38F, PH.base.pack) ;
    p104_z_SET((float) -2.3956793E38F, PH.base.pack) ;
    p104_roll_SET((float)2.747994E37F, PH.base.pack) ;
    p104_pitch_SET((float) -1.3109111E38F, PH.base.pack) ;
    p104_yaw_SET((float)2.2366734E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)3644994553541251107L, PH.base.pack) ;
    p105_xacc_SET((float)2.2259805E38F, PH.base.pack) ;
    p105_yacc_SET((float)1.1979118E38F, PH.base.pack) ;
    p105_zacc_SET((float) -7.665454E37F, PH.base.pack) ;
    p105_xgyro_SET((float)2.9413634E37F, PH.base.pack) ;
    p105_ygyro_SET((float)3.2934727E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -2.8632641E38F, PH.base.pack) ;
    p105_xmag_SET((float)2.9489895E38F, PH.base.pack) ;
    p105_ymag_SET((float) -6.003621E37F, PH.base.pack) ;
    p105_zmag_SET((float) -9.695246E36F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -9.113161E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -3.778307E37F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -2.9897483E38F, PH.base.pack) ;
    p105_temperature_SET((float) -1.5694953E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)59290, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)5336082648136124739L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)475315909L, PH.base.pack) ;
    p106_integrated_x_SET((float)3.0767468E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)1.5491913E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -7.30039E37F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -1.03306894E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)2.1915075E37F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -10177, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)1385983291L, PH.base.pack) ;
    p106_distance_SET((float) -1.6785268E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)1992308427363870542L, PH.base.pack) ;
    p107_xacc_SET((float) -2.4370895E38F, PH.base.pack) ;
    p107_yacc_SET((float)7.9493487E37F, PH.base.pack) ;
    p107_zacc_SET((float)1.7067548E38F, PH.base.pack) ;
    p107_xgyro_SET((float)7.6139025E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -2.5431359E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -6.491036E37F, PH.base.pack) ;
    p107_xmag_SET((float)1.9979502E38F, PH.base.pack) ;
    p107_ymag_SET((float)8.902022E37F, PH.base.pack) ;
    p107_zmag_SET((float) -1.3350575E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)7.484211E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float)1.0911406E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)4.338538E37F, PH.base.pack) ;
    p107_temperature_SET((float)2.77896E36F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)4157707275L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -1.227951E38F, PH.base.pack) ;
    p108_q2_SET((float)2.4914468E37F, PH.base.pack) ;
    p108_q3_SET((float)3.0686624E37F, PH.base.pack) ;
    p108_q4_SET((float) -1.6584858E38F, PH.base.pack) ;
    p108_roll_SET((float) -2.7826407E38F, PH.base.pack) ;
    p108_pitch_SET((float)1.098796E38F, PH.base.pack) ;
    p108_yaw_SET((float) -2.4154151E38F, PH.base.pack) ;
    p108_xacc_SET((float)8.927408E37F, PH.base.pack) ;
    p108_yacc_SET((float)3.386384E38F, PH.base.pack) ;
    p108_zacc_SET((float) -1.6582588E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -3.2565463E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -2.5752022E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.7948827E38F, PH.base.pack) ;
    p108_lat_SET((float) -4.5759154E37F, PH.base.pack) ;
    p108_lon_SET((float) -2.1954163E38F, PH.base.pack) ;
    p108_alt_SET((float)3.9598456E37F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)1.0886067E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)7.352168E37F, PH.base.pack) ;
    p108_vn_SET((float)3.8731145E37F, PH.base.pack) ;
    p108_ve_SET((float)1.2242609E38F, PH.base.pack) ;
    p108_vd_SET((float)4.0266077E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)27786, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)14032, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)248, (uint8_t)193, (uint8_t)214, (uint8_t)209, (uint8_t)156, (uint8_t)186, (uint8_t)220, (uint8_t)254, (uint8_t)175, (uint8_t)112, (uint8_t)139, (uint8_t)136, (uint8_t)197, (uint8_t)82, (uint8_t)149, (uint8_t)211, (uint8_t)190, (uint8_t)71, (uint8_t)119, (uint8_t)31, (uint8_t)179, (uint8_t)177, (uint8_t)10, (uint8_t)3, (uint8_t)134, (uint8_t)158, (uint8_t)94, (uint8_t)80, (uint8_t)67, (uint8_t)44, (uint8_t)50, (uint8_t)205, (uint8_t)101, (uint8_t)138, (uint8_t)255, (uint8_t)40, (uint8_t)171, (uint8_t)194, (uint8_t)101, (uint8_t)45, (uint8_t)123, (uint8_t)203, (uint8_t)197, (uint8_t)10, (uint8_t)238, (uint8_t)72, (uint8_t)162, (uint8_t)53, (uint8_t)180, (uint8_t)187, (uint8_t)40, (uint8_t)160, (uint8_t)150, (uint8_t)8, (uint8_t)98, (uint8_t)158, (uint8_t)134, (uint8_t)66, (uint8_t)175, (uint8_t)237, (uint8_t)31, (uint8_t)56, (uint8_t)228, (uint8_t)149, (uint8_t)2, (uint8_t)19, (uint8_t)141, (uint8_t)230, (uint8_t)4, (uint8_t)140, (uint8_t)200, (uint8_t)64, (uint8_t)198, (uint8_t)83, (uint8_t)188, (uint8_t)20, (uint8_t)238, (uint8_t)210, (uint8_t)14, (uint8_t)187, (uint8_t)32, (uint8_t)27, (uint8_t)169, (uint8_t)15, (uint8_t)250, (uint8_t)233, (uint8_t)110, (uint8_t)250, (uint8_t)125, (uint8_t)98, (uint8_t)115, (uint8_t)92, (uint8_t)141, (uint8_t)78, (uint8_t)128, (uint8_t)145, (uint8_t)133, (uint8_t)229, (uint8_t)138, (uint8_t)221, (uint8_t)11, (uint8_t)161, (uint8_t)75, (uint8_t)160, (uint8_t)144, (uint8_t)100, (uint8_t)142, (uint8_t)170, (uint8_t)1, (uint8_t)137, (uint8_t)26, (uint8_t)56, (uint8_t)224, (uint8_t)79, (uint8_t)73, (uint8_t)68, (uint8_t)24, (uint8_t)191, (uint8_t)158, (uint8_t)248, (uint8_t)37, (uint8_t)226, (uint8_t)38, (uint8_t)120, (uint8_t)20, (uint8_t)133, (uint8_t)113, (uint8_t)121, (uint8_t)187, (uint8_t)196, (uint8_t)70, (uint8_t)202, (uint8_t)240, (uint8_t)155, (uint8_t)197, (uint8_t)93, (uint8_t)208, (uint8_t)58, (uint8_t)6, (uint8_t)7, (uint8_t)86, (uint8_t)146, (uint8_t)191, (uint8_t)197, (uint8_t)71, (uint8_t)220, (uint8_t)203, (uint8_t)104, (uint8_t)29, (uint8_t)101, (uint8_t)34, (uint8_t)252, (uint8_t)130, (uint8_t)131, (uint8_t)119, (uint8_t)36, (uint8_t)11, (uint8_t)143, (uint8_t)245, (uint8_t)110, (uint8_t)208, (uint8_t)156, (uint8_t)176, (uint8_t)193, (uint8_t)234, (uint8_t)87, (uint8_t)10, (uint8_t)143, (uint8_t)73, (uint8_t)36, (uint8_t)200, (uint8_t)16, (uint8_t)45, (uint8_t)110, (uint8_t)204, (uint8_t)30, (uint8_t)15, (uint8_t)38, (uint8_t)210, (uint8_t)245, (uint8_t)252, (uint8_t)66, (uint8_t)98, (uint8_t)45, (uint8_t)146, (uint8_t)182, (uint8_t)251, (uint8_t)5, (uint8_t)16, (uint8_t)10, (uint8_t)170, (uint8_t)218, (uint8_t)240, (uint8_t)137, (uint8_t)142, (uint8_t)87, (uint8_t)200, (uint8_t)194, (uint8_t)158, (uint8_t)104, (uint8_t)206, (uint8_t)5, (uint8_t)170, (uint8_t)120, (uint8_t)216, (uint8_t)131, (uint8_t)187, (uint8_t)17, (uint8_t)118, (uint8_t)47, (uint8_t)0, (uint8_t)126, (uint8_t)138, (uint8_t)218, (uint8_t)36, (uint8_t)52, (uint8_t)142, (uint8_t)63, (uint8_t)40, (uint8_t)81, (uint8_t)164, (uint8_t)178, (uint8_t)20, (uint8_t)155, (uint8_t)185, (uint8_t)30, (uint8_t)34, (uint8_t)164, (uint8_t)82, (uint8_t)118, (uint8_t)23, (uint8_t)214, (uint8_t)102, (uint8_t)115, (uint8_t)194, (uint8_t)16, (uint8_t)191, (uint8_t)96, (uint8_t)216, (uint8_t)128, (uint8_t)23, (uint8_t)50, (uint8_t)82, (uint8_t)25, (uint8_t)130, (uint8_t)138, (uint8_t)179, (uint8_t)196, (uint8_t)43, (uint8_t)178, (uint8_t)156};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -4272650432644181428L, PH.base.pack) ;
    p111_ts1_SET((int64_t)3364563930391328906L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)2516046041118520575L, PH.base.pack) ;
    p112_seq_SET((uint32_t)2977016479L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)2586517421763311606L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p113_lat_SET((int32_t) -98422159, PH.base.pack) ;
    p113_lon_SET((int32_t)575438438, PH.base.pack) ;
    p113_alt_SET((int32_t) -1801319805, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)57294, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)56218, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)43269, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -16340, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -14139, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)25704, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)23879, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)2804677263777183425L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)884580L, PH.base.pack) ;
    p114_integrated_x_SET((float)4.975687E37F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.5223871E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)5.9575304E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -7.8074053E37F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -2.5210358E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -32241, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)339355238L, PH.base.pack) ;
    p114_distance_SET((float) -2.2267185E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)8805331181997735504L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-8.020679E37F, 2.3959545E37F, -1.4560496E38F, 2.5793443E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -4.1444893E37F, PH.base.pack) ;
    p115_pitchspeed_SET((float)2.3373212E38F, PH.base.pack) ;
    p115_yawspeed_SET((float) -2.3594251E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -695521102, PH.base.pack) ;
    p115_lon_SET((int32_t) -277904126, PH.base.pack) ;
    p115_alt_SET((int32_t) -1327947676, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)30716, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -5970, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -11169, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)14218, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)7938, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)18978, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)8431, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -31055, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)3052406963L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)16940, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)25695, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)22189, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)8472, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)9777, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)12118, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -19194, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -15100, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -13612, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)59572, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)47052, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)35298, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)11863, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)18834, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)4043934222L, PH.base.pack) ;
    p118_size_SET((uint32_t)699960098L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)31105, PH.base.pack) ;
    p119_ofs_SET((uint32_t)2122610225L, PH.base.pack) ;
    p119_count_SET((uint32_t)567962857L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)41220, PH.base.pack) ;
    p120_ofs_SET((uint32_t)105263949L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)189, (uint8_t)37, (uint8_t)45, (uint8_t)141, (uint8_t)212, (uint8_t)87, (uint8_t)156, (uint8_t)40, (uint8_t)217, (uint8_t)253, (uint8_t)121, (uint8_t)168, (uint8_t)23, (uint8_t)123, (uint8_t)63, (uint8_t)161, (uint8_t)157, (uint8_t)61, (uint8_t)68, (uint8_t)249, (uint8_t)134, (uint8_t)192, (uint8_t)89, (uint8_t)248, (uint8_t)132, (uint8_t)196, (uint8_t)86, (uint8_t)13, (uint8_t)187, (uint8_t)120, (uint8_t)134, (uint8_t)81, (uint8_t)84, (uint8_t)147, (uint8_t)205, (uint8_t)11, (uint8_t)217, (uint8_t)8, (uint8_t)93, (uint8_t)24, (uint8_t)146, (uint8_t)124, (uint8_t)76, (uint8_t)208, (uint8_t)86, (uint8_t)61, (uint8_t)40, (uint8_t)84, (uint8_t)29, (uint8_t)115, (uint8_t)154, (uint8_t)212, (uint8_t)0, (uint8_t)159, (uint8_t)148, (uint8_t)55, (uint8_t)109, (uint8_t)108, (uint8_t)158, (uint8_t)92, (uint8_t)9, (uint8_t)185, (uint8_t)217, (uint8_t)75, (uint8_t)251, (uint8_t)241, (uint8_t)131, (uint8_t)144, (uint8_t)39, (uint8_t)148, (uint8_t)44, (uint8_t)132, (uint8_t)227, (uint8_t)86, (uint8_t)186, (uint8_t)113, (uint8_t)127, (uint8_t)16, (uint8_t)86, (uint8_t)212, (uint8_t)156, (uint8_t)76, (uint8_t)157, (uint8_t)23, (uint8_t)70, (uint8_t)134, (uint8_t)138, (uint8_t)167, (uint8_t)39, (uint8_t)159};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)22, (uint8_t)94, (uint8_t)40, (uint8_t)139, (uint8_t)227, (uint8_t)136, (uint8_t)230, (uint8_t)137, (uint8_t)72, (uint8_t)72, (uint8_t)212, (uint8_t)213, (uint8_t)255, (uint8_t)138, (uint8_t)53, (uint8_t)27, (uint8_t)230, (uint8_t)171, (uint8_t)204, (uint8_t)251, (uint8_t)5, (uint8_t)185, (uint8_t)223, (uint8_t)115, (uint8_t)254, (uint8_t)83, (uint8_t)96, (uint8_t)18, (uint8_t)40, (uint8_t)253, (uint8_t)58, (uint8_t)118, (uint8_t)198, (uint8_t)42, (uint8_t)114, (uint8_t)218, (uint8_t)49, (uint8_t)84, (uint8_t)144, (uint8_t)73, (uint8_t)65, (uint8_t)35, (uint8_t)91, (uint8_t)232, (uint8_t)130, (uint8_t)36, (uint8_t)199, (uint8_t)196, (uint8_t)8, (uint8_t)195, (uint8_t)51, (uint8_t)94, (uint8_t)169, (uint8_t)228, (uint8_t)227, (uint8_t)105, (uint8_t)250, (uint8_t)27, (uint8_t)8, (uint8_t)47, (uint8_t)38, (uint8_t)82, (uint8_t)180, (uint8_t)5, (uint8_t)88, (uint8_t)61, (uint8_t)68, (uint8_t)169, (uint8_t)140, (uint8_t)118, (uint8_t)154, (uint8_t)204, (uint8_t)31, (uint8_t)90, (uint8_t)32, (uint8_t)183, (uint8_t)44, (uint8_t)239, (uint8_t)87, (uint8_t)23, (uint8_t)29, (uint8_t)97, (uint8_t)155, (uint8_t)210, (uint8_t)183, (uint8_t)146, (uint8_t)188, (uint8_t)173, (uint8_t)177, (uint8_t)200, (uint8_t)197, (uint8_t)209, (uint8_t)14, (uint8_t)23, (uint8_t)163, (uint8_t)197, (uint8_t)191, (uint8_t)204, (uint8_t)35, (uint8_t)130, (uint8_t)170, (uint8_t)174, (uint8_t)105, (uint8_t)122, (uint8_t)78, (uint8_t)139, (uint8_t)188, (uint8_t)75, (uint8_t)38, (uint8_t)105};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)3134430821922696580L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
    p124_lat_SET((int32_t) -2051631836, PH.base.pack) ;
    p124_lon_SET((int32_t)1447717213, PH.base.pack) ;
    p124_alt_SET((int32_t)1508608061, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)55619, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)57914, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)1904, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)6182, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)181948366L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)638, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)9183, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)36376, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)3197325674L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)228, (uint8_t)107, (uint8_t)221, (uint8_t)235, (uint8_t)165, (uint8_t)162, (uint8_t)154, (uint8_t)220, (uint8_t)142, (uint8_t)239, (uint8_t)232, (uint8_t)54, (uint8_t)98, (uint8_t)182, (uint8_t)211, (uint8_t)143, (uint8_t)21, (uint8_t)62, (uint8_t)232, (uint8_t)234, (uint8_t)213, (uint8_t)187, (uint8_t)11, (uint8_t)40, (uint8_t)32, (uint8_t)243, (uint8_t)216, (uint8_t)15, (uint8_t)215, (uint8_t)8, (uint8_t)79, (uint8_t)226, (uint8_t)152, (uint8_t)140, (uint8_t)218, (uint8_t)15, (uint8_t)92, (uint8_t)186, (uint8_t)181, (uint8_t)128, (uint8_t)210, (uint8_t)123, (uint8_t)201, (uint8_t)169, (uint8_t)22, (uint8_t)187, (uint8_t)74, (uint8_t)149, (uint8_t)20, (uint8_t)49, (uint8_t)40, (uint8_t)108, (uint8_t)47, (uint8_t)29, (uint8_t)6, (uint8_t)162, (uint8_t)41, (uint8_t)42, (uint8_t)50, (uint8_t)119, (uint8_t)5, (uint8_t)45, (uint8_t)123, (uint8_t)17, (uint8_t)213, (uint8_t)28, (uint8_t)205, (uint8_t)99, (uint8_t)115, (uint8_t)46};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3720788684L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)56022, PH.base.pack) ;
    p127_tow_SET((uint32_t)3572048735L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1305638007, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -197441639, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -1983490020, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)543598953L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1468558081, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)855300802L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)44817, PH.base.pack) ;
    p128_tow_SET((uint32_t)3163591035L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)989571924, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -1004151083, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -1807056857, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1795226583L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -466457013, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)1901065147L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)23280, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)5743, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -31669, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)26922, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)559, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)25185, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -21578, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)13270, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)5929, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p130_size_SET((uint32_t)2004640976L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)36758, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)29398, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)18971, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)48867, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)62, (uint8_t)29, (uint8_t)6, (uint8_t)17, (uint8_t)198, (uint8_t)162, (uint8_t)76, (uint8_t)236, (uint8_t)83, (uint8_t)141, (uint8_t)173, (uint8_t)246, (uint8_t)140, (uint8_t)205, (uint8_t)2, (uint8_t)143, (uint8_t)166, (uint8_t)153, (uint8_t)175, (uint8_t)64, (uint8_t)151, (uint8_t)249, (uint8_t)168, (uint8_t)239, (uint8_t)70, (uint8_t)22, (uint8_t)140, (uint8_t)239, (uint8_t)1, (uint8_t)33, (uint8_t)184, (uint8_t)124, (uint8_t)27, (uint8_t)146, (uint8_t)142, (uint8_t)247, (uint8_t)7, (uint8_t)54, (uint8_t)204, (uint8_t)9, (uint8_t)104, (uint8_t)252, (uint8_t)110, (uint8_t)167, (uint8_t)204, (uint8_t)125, (uint8_t)51, (uint8_t)77, (uint8_t)230, (uint8_t)125, (uint8_t)0, (uint8_t)230, (uint8_t)39, (uint8_t)138, (uint8_t)239, (uint8_t)166, (uint8_t)69, (uint8_t)60, (uint8_t)61, (uint8_t)114, (uint8_t)144, (uint8_t)99, (uint8_t)99, (uint8_t)116, (uint8_t)206, (uint8_t)4, (uint8_t)239, (uint8_t)73, (uint8_t)20, (uint8_t)55, (uint8_t)15, (uint8_t)180, (uint8_t)230, (uint8_t)134, (uint8_t)11, (uint8_t)169, (uint8_t)7, (uint8_t)194, (uint8_t)27, (uint8_t)225, (uint8_t)105, (uint8_t)16, (uint8_t)85, (uint8_t)109, (uint8_t)118, (uint8_t)88, (uint8_t)12, (uint8_t)201, (uint8_t)37, (uint8_t)100, (uint8_t)49, (uint8_t)52, (uint8_t)80, (uint8_t)130, (uint8_t)212, (uint8_t)85, (uint8_t)156, (uint8_t)29, (uint8_t)11, (uint8_t)23, (uint8_t)31, (uint8_t)199, (uint8_t)185, (uint8_t)21, (uint8_t)215, (uint8_t)229, (uint8_t)62, (uint8_t)113, (uint8_t)219, (uint8_t)121, (uint8_t)145, (uint8_t)152, (uint8_t)116, (uint8_t)167, (uint8_t)62, (uint8_t)224, (uint8_t)204, (uint8_t)237, (uint8_t)13, (uint8_t)97, (uint8_t)9, (uint8_t)3, (uint8_t)251, (uint8_t)91, (uint8_t)4, (uint8_t)10, (uint8_t)128, (uint8_t)148, (uint8_t)126, (uint8_t)206, (uint8_t)38, (uint8_t)15, (uint8_t)239, (uint8_t)244, (uint8_t)242, (uint8_t)16, (uint8_t)173, (uint8_t)122, (uint8_t)145, (uint8_t)55, (uint8_t)142, (uint8_t)208, (uint8_t)184, (uint8_t)114, (uint8_t)167, (uint8_t)32, (uint8_t)205, (uint8_t)237, (uint8_t)52, (uint8_t)165, (uint8_t)215, (uint8_t)173, (uint8_t)165, (uint8_t)167, (uint8_t)42, (uint8_t)4, (uint8_t)242, (uint8_t)72, (uint8_t)205, (uint8_t)161, (uint8_t)100, (uint8_t)12, (uint8_t)211, (uint8_t)245, (uint8_t)161, (uint8_t)211, (uint8_t)180, (uint8_t)126, (uint8_t)98, (uint8_t)35, (uint8_t)174, (uint8_t)224, (uint8_t)26, (uint8_t)47, (uint8_t)163, (uint8_t)246, (uint8_t)224, (uint8_t)132, (uint8_t)48, (uint8_t)176, (uint8_t)80, (uint8_t)173, (uint8_t)20, (uint8_t)107, (uint8_t)12, (uint8_t)196, (uint8_t)170, (uint8_t)1, (uint8_t)8, (uint8_t)159, (uint8_t)207, (uint8_t)221, (uint8_t)38, (uint8_t)199, (uint8_t)82, (uint8_t)25, (uint8_t)215, (uint8_t)148, (uint8_t)220, (uint8_t)60, (uint8_t)78, (uint8_t)156, (uint8_t)176, (uint8_t)57, (uint8_t)66, (uint8_t)201, (uint8_t)230, (uint8_t)149, (uint8_t)211, (uint8_t)30, (uint8_t)148, (uint8_t)247, (uint8_t)84, (uint8_t)192, (uint8_t)226, (uint8_t)224, (uint8_t)177, (uint8_t)95, (uint8_t)97, (uint8_t)106, (uint8_t)241, (uint8_t)18, (uint8_t)159, (uint8_t)56, (uint8_t)140, (uint8_t)152, (uint8_t)188, (uint8_t)43, (uint8_t)180, (uint8_t)147, (uint8_t)73, (uint8_t)130, (uint8_t)144, (uint8_t)202, (uint8_t)21, (uint8_t)49, (uint8_t)129, (uint8_t)66, (uint8_t)94, (uint8_t)84, (uint8_t)132, (uint8_t)23, (uint8_t)250, (uint8_t)104, (uint8_t)201, (uint8_t)204, (uint8_t)73, (uint8_t)222, (uint8_t)121, (uint8_t)113, (uint8_t)15, (uint8_t)180, (uint8_t)147};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1260774721L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)59242, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)39885, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)63378, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_270, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)879133405, PH.base.pack) ;
    p133_lon_SET((int32_t) -1334880026, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)34735, PH.base.pack) ;
    p133_mask_SET((uint64_t)1321112846260847244L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1734690428, PH.base.pack) ;
    p134_lon_SET((int32_t) -994035803, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)50699, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)24142, (int16_t)4520, (int16_t)20849, (int16_t) -21829, (int16_t) -14853, (int16_t) -23663, (int16_t) -17153, (int16_t) -13711, (int16_t)6590, (int16_t) -16957, (int16_t)6854, (int16_t)908, (int16_t)15368, (int16_t)8894, (int16_t) -11631, (int16_t)30949};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1046941452, PH.base.pack) ;
    p135_lon_SET((int32_t)831133731, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -924845854, PH.base.pack) ;
    p136_lon_SET((int32_t) -1198409906, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)64486, PH.base.pack) ;
    p136_terrain_height_SET((float)3.3437169E38F, PH.base.pack) ;
    p136_current_height_SET((float)1.3642453E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)53210, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)11911, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)4276386140L, PH.base.pack) ;
    p137_press_abs_SET((float)5.949115E37F, PH.base.pack) ;
    p137_press_diff_SET((float)3.28222E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -14301, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)2864443477230483424L, PH.base.pack) ;
    {
        float  q [] =  {-2.2667923E38F, 5.298405E37F, -6.5852065E37F, -7.0490713E37F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -5.2472916E37F, PH.base.pack) ;
    p138_y_SET((float) -2.9906478E38F, PH.base.pack) ;
    p138_z_SET((float) -3.2387558E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)8855525976058398113L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    {
        float  controls [] =  {-1.1676463E38F, -1.8153469E38F, -2.9632617E38F, -2.8972345E38F, 3.5413805E37F, 2.8284877E38F, -1.9554502E38F, -2.8252928E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)8282422646535260645L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    {
        float  controls [] =  {-1.6798389E38F, 2.760693E38F, 1.1605295E38F, -1.0803153E35F, -3.2505612E38F, -1.7294083E38F, 4.829121E37F, 9.025102E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)86662461060788315L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.081623E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -2.4395325E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -7.851548E37F, PH.base.pack) ;
    p141_altitude_relative_SET((float)6.389955E36F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -1.9030867E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.3874284E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)31, (uint8_t)121, (uint8_t)54, (uint8_t)236, (uint8_t)61, (uint8_t)176, (uint8_t)86, (uint8_t)224, (uint8_t)149, (uint8_t)160, (uint8_t)83, (uint8_t)215, (uint8_t)29, (uint8_t)222, (uint8_t)255, (uint8_t)110, (uint8_t)15, (uint8_t)65, (uint8_t)109, (uint8_t)113, (uint8_t)61, (uint8_t)88, (uint8_t)115, (uint8_t)16, (uint8_t)238, (uint8_t)146, (uint8_t)254, (uint8_t)117, (uint8_t)41, (uint8_t)123, (uint8_t)168, (uint8_t)111, (uint8_t)187, (uint8_t)230, (uint8_t)81, (uint8_t)102, (uint8_t)206, (uint8_t)164, (uint8_t)93, (uint8_t)26, (uint8_t)62, (uint8_t)14, (uint8_t)170, (uint8_t)30, (uint8_t)18, (uint8_t)116, (uint8_t)5, (uint8_t)61, (uint8_t)57, (uint8_t)230, (uint8_t)38, (uint8_t)105, (uint8_t)220, (uint8_t)17, (uint8_t)94, (uint8_t)62, (uint8_t)150, (uint8_t)180, (uint8_t)79, (uint8_t)9, (uint8_t)192, (uint8_t)27, (uint8_t)255, (uint8_t)116, (uint8_t)131, (uint8_t)140, (uint8_t)226, (uint8_t)166, (uint8_t)45, (uint8_t)110, (uint8_t)117, (uint8_t)70, (uint8_t)17, (uint8_t)138, (uint8_t)128, (uint8_t)214, (uint8_t)136, (uint8_t)244, (uint8_t)162, (uint8_t)15, (uint8_t)21, (uint8_t)171, (uint8_t)25, (uint8_t)220, (uint8_t)222, (uint8_t)157, (uint8_t)213, (uint8_t)211, (uint8_t)170, (uint8_t)115, (uint8_t)26, (uint8_t)169, (uint8_t)1, (uint8_t)120, (uint8_t)49, (uint8_t)251, (uint8_t)45, (uint8_t)238, (uint8_t)238, (uint8_t)217, (uint8_t)130, (uint8_t)70, (uint8_t)165, (uint8_t)208, (uint8_t)4, (uint8_t)139, (uint8_t)16, (uint8_t)252, (uint8_t)231, (uint8_t)249, (uint8_t)32, (uint8_t)236, (uint8_t)243, (uint8_t)241, (uint8_t)176, (uint8_t)197, (uint8_t)93, (uint8_t)137, (uint8_t)92, (uint8_t)128};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)41, (uint8_t)92, (uint8_t)81, (uint8_t)42, (uint8_t)232, (uint8_t)203, (uint8_t)75, (uint8_t)196, (uint8_t)2, (uint8_t)105, (uint8_t)132, (uint8_t)253, (uint8_t)201, (uint8_t)2, (uint8_t)44, (uint8_t)22, (uint8_t)228, (uint8_t)81, (uint8_t)112, (uint8_t)6, (uint8_t)163, (uint8_t)97, (uint8_t)51, (uint8_t)251, (uint8_t)13, (uint8_t)33, (uint8_t)59, (uint8_t)115, (uint8_t)94, (uint8_t)26, (uint8_t)123, (uint8_t)111, (uint8_t)174, (uint8_t)9, (uint8_t)11, (uint8_t)110, (uint8_t)107, (uint8_t)85, (uint8_t)191, (uint8_t)181, (uint8_t)126, (uint8_t)213, (uint8_t)215, (uint8_t)1, (uint8_t)57, (uint8_t)231, (uint8_t)18, (uint8_t)83, (uint8_t)9, (uint8_t)224, (uint8_t)113, (uint8_t)90, (uint8_t)144, (uint8_t)172, (uint8_t)188, (uint8_t)0, (uint8_t)246, (uint8_t)59, (uint8_t)255, (uint8_t)7, (uint8_t)197, (uint8_t)243, (uint8_t)60, (uint8_t)31, (uint8_t)159, (uint8_t)192, (uint8_t)136, (uint8_t)183, (uint8_t)14, (uint8_t)250, (uint8_t)100, (uint8_t)81, (uint8_t)193, (uint8_t)35, (uint8_t)165, (uint8_t)108, (uint8_t)191, (uint8_t)196, (uint8_t)53, (uint8_t)22, (uint8_t)8, (uint8_t)41, (uint8_t)205, (uint8_t)162, (uint8_t)191, (uint8_t)139, (uint8_t)251, (uint8_t)242, (uint8_t)223, (uint8_t)113, (uint8_t)253, (uint8_t)25, (uint8_t)206, (uint8_t)30, (uint8_t)177, (uint8_t)107, (uint8_t)154, (uint8_t)146, (uint8_t)152, (uint8_t)160, (uint8_t)157, (uint8_t)39, (uint8_t)147, (uint8_t)244, (uint8_t)25, (uint8_t)235, (uint8_t)243, (uint8_t)131, (uint8_t)25, (uint8_t)243, (uint8_t)33, (uint8_t)78, (uint8_t)9, (uint8_t)225, (uint8_t)58, (uint8_t)162, (uint8_t)75, (uint8_t)98, (uint8_t)87, (uint8_t)47};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)2229217068L, PH.base.pack) ;
    p143_press_abs_SET((float)1.83899E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -5.1401183E37F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)32552, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)5043886074216427395L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p144_lat_SET((int32_t)150840986, PH.base.pack) ;
    p144_lon_SET((int32_t) -2045309650, PH.base.pack) ;
    p144_alt_SET((float) -5.5643777E37F, PH.base.pack) ;
    {
        float  vel [] =  {3.007693E38F, -1.2539023E37F, 2.5217096E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-1.840285E38F, -2.620047E37F, 1.3068964E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {2.4025074E38F, -2.4392356E38F, -9.615277E37F, -2.7983075E37F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.23717E38F, -1.207552E38F, -2.8901643E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {7.9510456E36F, 2.8197517E38F, 1.3159074E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)6748097919654097498L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)2664559087975895844L, PH.base.pack) ;
    p146_x_acc_SET((float) -3.195973E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -1.808574E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -1.0548009E38F, PH.base.pack) ;
    p146_x_vel_SET((float) -1.4119424E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -1.5358576E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -2.7722603E37F, PH.base.pack) ;
    p146_x_pos_SET((float) -9.89059E37F, PH.base.pack) ;
    p146_y_pos_SET((float)1.485244E38F, PH.base.pack) ;
    p146_z_pos_SET((float)2.5886224E38F, PH.base.pack) ;
    p146_airspeed_SET((float) -2.692302E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.8304745E38F, 1.9580997E38F, 6.9885456E37F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.1209933E38F, -5.835639E37F, 2.3228034E37F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {1.321169E38F, 1.5723245E38F, 2.4681516E38F, -8.740398E37F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)3.2510933E37F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -2.3070294E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -3.3600353E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -24098, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)47840, (uint16_t)2379, (uint16_t)53328, (uint16_t)45566, (uint16_t)64943, (uint16_t)50286, (uint16_t)47531, (uint16_t)31374, (uint16_t)23932, (uint16_t)24857};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)4079, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)140755842, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)72965939, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -103, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1969004032L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)4030461196L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)3727441260L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)3261160793L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)34, (uint8_t)244, (uint8_t)225, (uint8_t)167, (uint8_t)17, (uint8_t)52, (uint8_t)117, (uint8_t)188};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)64, (uint8_t)42, (uint8_t)227, (uint8_t)230, (uint8_t)203, (uint8_t)123, (uint8_t)216, (uint8_t)115};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)130, (uint8_t)16, (uint8_t)169, (uint8_t)31, (uint8_t)30, (uint8_t)214, (uint8_t)31, (uint8_t)100};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)57050, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)36459, PH.base.pack) ;
    p148_uid_SET((uint64_t)7479195194862291083L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)230, (uint8_t)95, (uint8_t)189, (uint8_t)223, (uint8_t)127, (uint8_t)3, (uint8_t)164, (uint8_t)108, (uint8_t)40, (uint8_t)22, (uint8_t)148, (uint8_t)100, (uint8_t)190, (uint8_t)242, (uint8_t)99, (uint8_t)255, (uint8_t)150, (uint8_t)201};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)637446869129775152L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p149_angle_x_SET((float)2.6692414E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -1.2258633E38F, PH.base.pack) ;
    p149_distance_SET((float) -1.4160964E38F, PH.base.pack) ;
    p149_size_x_SET((float) -3.2401447E38F, PH.base.pack) ;
    p149_size_y_SET((float) -1.5644271E38F, PH.base.pack) ;
    p149_x_SET((float) -7.955359E37F, &PH) ;
    p149_y_SET((float) -2.2474125E38F, &PH) ;
    p149_z_SET((float)1.9429654E38F, &PH) ;
    {
        float  q [] =  {-2.954618E38F, 1.3353732E38F, -3.1276163E38F, 6.725191E36F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)9, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)8695360250042957439L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
    p230_vel_ratio_SET((float) -1.8766504E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -3.3457577E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)2.82991E37F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -1.2124762E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -2.6699916E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -2.7218623E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -7.2100934E37F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -3.1725153E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)593581894841145989L, PH.base.pack) ;
    p231_wind_x_SET((float) -3.3905924E38F, PH.base.pack) ;
    p231_wind_y_SET((float)5.191006E37F, PH.base.pack) ;
    p231_wind_z_SET((float) -7.2071783E37F, PH.base.pack) ;
    p231_var_horiz_SET((float) -6.096169E37F, PH.base.pack) ;
    p231_var_vert_SET((float)2.0183543E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -1.1085251E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)2.9641957E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -2.5811626E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)4745354102314031284L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)2262711455L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)21457, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p232_lat_SET((int32_t)856693515, PH.base.pack) ;
    p232_lon_SET((int32_t)936232851, PH.base.pack) ;
    p232_alt_SET((float)8.2409504E37F, PH.base.pack) ;
    p232_hdop_SET((float)3.2846685E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.4967778E38F, PH.base.pack) ;
    p232_vn_SET((float) -2.796169E38F, PH.base.pack) ;
    p232_ve_SET((float)1.5977149E38F, PH.base.pack) ;
    p232_vd_SET((float)2.7284056E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -3.2681955E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float)3.1561744E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)1.8413012E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)159, (uint8_t)92, (uint8_t)154, (uint8_t)11, (uint8_t)110, (uint8_t)17, (uint8_t)40, (uint8_t)51, (uint8_t)138, (uint8_t)100, (uint8_t)41, (uint8_t)24, (uint8_t)174, (uint8_t)164, (uint8_t)36, (uint8_t)225, (uint8_t)214, (uint8_t)18, (uint8_t)222, (uint8_t)23, (uint8_t)178, (uint8_t)57, (uint8_t)157, (uint8_t)149, (uint8_t)216, (uint8_t)145, (uint8_t)71, (uint8_t)140, (uint8_t)188, (uint8_t)240, (uint8_t)51, (uint8_t)56, (uint8_t)216, (uint8_t)14, (uint8_t)99, (uint8_t)200, (uint8_t)58, (uint8_t)79, (uint8_t)56, (uint8_t)76, (uint8_t)165, (uint8_t)39, (uint8_t)143, (uint8_t)204, (uint8_t)206, (uint8_t)250, (uint8_t)117, (uint8_t)7, (uint8_t)82, (uint8_t)237, (uint8_t)11, (uint8_t)106, (uint8_t)27, (uint8_t)73, (uint8_t)151, (uint8_t)155, (uint8_t)221, (uint8_t)42, (uint8_t)188, (uint8_t)254, (uint8_t)185, (uint8_t)97, (uint8_t)178, (uint8_t)139, (uint8_t)84, (uint8_t)108, (uint8_t)240, (uint8_t)75, (uint8_t)91, (uint8_t)243, (uint8_t)220, (uint8_t)9, (uint8_t)63, (uint8_t)248, (uint8_t)180, (uint8_t)243, (uint8_t)183, (uint8_t)67, (uint8_t)248, (uint8_t)226, (uint8_t)6, (uint8_t)33, (uint8_t)82, (uint8_t)96, (uint8_t)246, (uint8_t)160, (uint8_t)22, (uint8_t)140, (uint8_t)211, (uint8_t)78, (uint8_t)178, (uint8_t)233, (uint8_t)232, (uint8_t)158, (uint8_t)88, (uint8_t)125, (uint8_t)104, (uint8_t)159, (uint8_t)57, (uint8_t)165, (uint8_t)148, (uint8_t)105, (uint8_t)243, (uint8_t)54, (uint8_t)242, (uint8_t)150, (uint8_t)97, (uint8_t)34, (uint8_t)244, (uint8_t)219, (uint8_t)61, (uint8_t)59, (uint8_t)12, (uint8_t)208, (uint8_t)211, (uint8_t)5, (uint8_t)24, (uint8_t)30, (uint8_t)246, (uint8_t)5, (uint8_t)43, (uint8_t)176, (uint8_t)72, (uint8_t)60, (uint8_t)174, (uint8_t)136, (uint8_t)198, (uint8_t)51, (uint8_t)50, (uint8_t)42, (uint8_t)83, (uint8_t)132, (uint8_t)92, (uint8_t)108, (uint8_t)79, (uint8_t)234, (uint8_t)205, (uint8_t)76, (uint8_t)241, (uint8_t)200, (uint8_t)254, (uint8_t)9, (uint8_t)234, (uint8_t)86, (uint8_t)61, (uint8_t)10, (uint8_t)180, (uint8_t)139, (uint8_t)191, (uint8_t)165, (uint8_t)31, (uint8_t)92, (uint8_t)125, (uint8_t)201, (uint8_t)25, (uint8_t)90, (uint8_t)165, (uint8_t)189, (uint8_t)57, (uint8_t)90, (uint8_t)48, (uint8_t)43, (uint8_t)134, (uint8_t)72, (uint8_t)186, (uint8_t)222, (uint8_t)94, (uint8_t)127, (uint8_t)187, (uint8_t)203, (uint8_t)183, (uint8_t)118, (uint8_t)128, (uint8_t)61, (uint8_t)247, (uint8_t)60, (uint8_t)27, (uint8_t)181, (uint8_t)159, (uint8_t)249};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1171313280L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -6050, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)14944, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)52939, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -38, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)5631, PH.base.pack) ;
    p234_latitude_SET((int32_t) -529329458, PH.base.pack) ;
    p234_longitude_SET((int32_t)786040093, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t) -8397, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)1163, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)120, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t) -115, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -35, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)15977, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)1308664261685363265L, PH.base.pack) ;
    p241_vibration_x_SET((float) -6.039086E37F, PH.base.pack) ;
    p241_vibration_y_SET((float) -2.1517499E38F, PH.base.pack) ;
    p241_vibration_z_SET((float) -1.16947E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)4282147897L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)2155519146L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)310148304L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)543195556, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1198094259, PH.base.pack) ;
    p242_altitude_SET((int32_t)902572756, PH.base.pack) ;
    p242_x_SET((float)1.7859199E37F, PH.base.pack) ;
    p242_y_SET((float) -1.865461E38F, PH.base.pack) ;
    p242_z_SET((float)2.145764E38F, PH.base.pack) ;
    {
        float  q [] =  {6.689676E37F, -2.5879515E38F, -1.7334625E38F, 2.259692E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -1.3407018E37F, PH.base.pack) ;
    p242_approach_y_SET((float)3.2810252E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -2.7831076E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)3787719321912021485L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
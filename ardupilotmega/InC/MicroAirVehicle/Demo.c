
#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
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
void c_CommunicationChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  mag_ofs_x = p151_mag_ofs_x_GET(pack);
    int16_t  mag_ofs_y = p151_mag_ofs_y_GET(pack);
    int16_t  mag_ofs_z = p151_mag_ofs_z_GET(pack);
}
void c_CommunicationChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  brkval = p152_brkval_GET(pack);
    uint16_t  freemem = p152_freemem_GET(pack);
    uint32_t  freemem32 = p152_freemem32_TRY(ph);
}
void c_CommunicationChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  adc1 = p153_adc1_GET(pack);
    uint16_t  adc2 = p153_adc2_GET(pack);
    uint16_t  adc3 = p153_adc3_GET(pack);
    uint16_t  adc4 = p153_adc4_GET(pack);
    uint16_t  adc5 = p153_adc5_GET(pack);
    uint16_t  adc6 = p153_adc6_GET(pack);
}
void c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    e_MAV_MOUNT_MODE  mount_mode = p156_mount_mode_GET(pack);
    uint8_t  stab_roll = p156_stab_roll_GET(pack);
    uint8_t  stab_pitch = p156_stab_pitch_GET(pack);
    uint8_t  stab_yaw = p156_stab_yaw_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    int32_t  input_a = p157_input_a_GET(pack);
    int32_t  input_b = p157_input_b_GET(pack);
    int32_t  input_c = p157_input_c_GET(pack);
    uint8_t  save_position = p157_save_position_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p158_target_system_GET(pack);
    uint8_t  target_component = p158_target_component_GET(pack);
    int32_t  pointing_a = p158_pointing_a_GET(pack);
    int32_t  pointing_b = p158_pointing_b_GET(pack);
    int32_t  pointing_c = p158_pointing_c_GET(pack);
}
void c_CommunicationChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p160_target_system_GET(pack);
    uint8_t  target_component = p160_target_component_GET(pack);
    uint8_t  idx = p160_idx_GET(pack);
    uint8_t  count = p160_count_GET(pack);
    float  lat = p160_lat_GET(pack);
    float  lng = p160_lng_GET(pack);
}
void c_CommunicationChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p161_target_system_GET(pack);
    uint8_t  target_component = p161_target_component_GET(pack);
    uint8_t  idx = p161_idx_GET(pack);
}
void c_CommunicationChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  breach_status = p162_breach_status_GET(pack);
    uint16_t  breach_count = p162_breach_count_GET(pack);
    e_FENCE_BREACH  breach_type = p162_breach_type_GET(pack);
    uint32_t  breach_time = p162_breach_time_GET(pack);
}
void c_CommunicationChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    float  omegaIx = p163_omegaIx_GET(pack);
    float  omegaIy = p163_omegaIy_GET(pack);
    float  omegaIz = p163_omegaIz_GET(pack);
    float  accel_weight = p163_accel_weight_GET(pack);
    float  renorm_val = p163_renorm_val_GET(pack);
    float  error_rp = p163_error_rp_GET(pack);
    float  error_yaw = p163_error_yaw_GET(pack);
}
void c_CommunicationChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p165_Vcc_GET(pack);
    uint8_t  I2Cerr = p165_I2Cerr_GET(pack);
}
void c_CommunicationChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p166_rssi_GET(pack);
    uint8_t  remrssi = p166_remrssi_GET(pack);
    uint8_t  txbuf = p166_txbuf_GET(pack);
    uint8_t  noise = p166_noise_GET(pack);
    uint8_t  remnoise = p166_remnoise_GET(pack);
    uint16_t  rxerrors = p166_rxerrors_GET(pack);
    uint16_t  fixed_ = p166_fixed__GET(pack);
}
void c_CommunicationChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    float  direction = p168_direction_GET(pack);
    float  speed = p168_speed_GET(pack);
    float  speed_z = p168_speed_z_GET(pack);
}
void c_CommunicationChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p169_type_GET(pack);
    uint8_t  len = p169_len_GET(pack);
    uint8_t*  data_ = p169_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p170_type_GET(pack);
    uint8_t  len = p170_len_GET(pack);
    uint8_t*  data_ = p170_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p171_type_GET(pack);
    uint8_t  len = p171_len_GET(pack);
    uint8_t*  data_ = p171_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p172_type_GET(pack);
    uint8_t  len = p172_len_GET(pack);
    uint8_t*  data_ = p172_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    float  distance = p173_distance_GET(pack);
    float  voltage = p173_voltage_GET(pack);
}
void c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p176_target_system_GET(pack);
    uint8_t  target_component = p176_target_component_GET(pack);
    uint8_t  idx = p176_idx_GET(pack);
}
void c_CommunicationChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  throttle = p177_throttle_GET(pack);
    float  current = p177_current_GET(pack);
    uint16_t  interference = p177_interference_GET(pack);
    float  CompensationX = p177_CompensationX_GET(pack);
    float  CompensationY = p177_CompensationY_GET(pack);
    float  CompensationZ = p177_CompensationZ_GET(pack);
}
void c_CommunicationChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p178_roll_GET(pack);
    float  pitch = p178_pitch_GET(pack);
    float  yaw = p178_yaw_GET(pack);
    float  altitude = p178_altitude_GET(pack);
    int32_t  lat = p178_lat_GET(pack);
    int32_t  lng = p178_lng_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  voltage = p181_voltage_GET(pack);
    int16_t  current_battery = p181_current_battery_GET(pack);
}
void c_CommunicationChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p183_target_system_GET(pack);
    uint8_t  target_component = p183_target_component_GET(pack);
}
void c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p184_target_system_GET(pack);
    uint8_t  target_component = p184_target_component_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  seqno = p184_seqno_GET(pack);
    uint8_t*  data_ = p184_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p185_target_system_GET(pack);
    uint8_t  target_component = p185_target_component_GET(pack);
    uint32_t  seqno = p185_seqno_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  status = p185_status_GET(pack);
}
void c_CommunicationChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    e_EKF_STATUS_FLAGS  flags = p193_flags_GET(pack);
    float  velocity_variance = p193_velocity_variance_GET(pack);
    float  pos_horiz_variance = p193_pos_horiz_variance_GET(pack);
    float  pos_vert_variance = p193_pos_vert_variance_GET(pack);
    float  compass_variance = p193_compass_variance_GET(pack);
    float  terrain_alt_variance = p193_terrain_alt_variance_GET(pack);
}
void c_CommunicationChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    e_PID_TUNING_AXIS  axis = p194_axis_GET(pack);
    float  desired = p194_desired_GET(pack);
    float  achieved = p194_achieved_GET(pack);
    float  FF = p194_FF_GET(pack);
    float  P = p194_P_GET(pack);
    float  I = p194_I_GET(pack);
    float  D = p194_D_GET(pack);
}
void c_CommunicationChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p201_target_system_GET(pack);
    uint8_t  target_component = p201_target_component_GET(pack);
    float  demanded_rate_x = p201_demanded_rate_x_GET(pack);
    float  demanded_rate_y = p201_demanded_rate_y_GET(pack);
    float  demanded_rate_z = p201_demanded_rate_z_GET(pack);
}
void c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p214_target_system_GET(pack);
    uint8_t  target_component = p214_target_component_GET(pack);
    int16_t  rl_torque_cmd = p214_rl_torque_cmd_GET(pack);
    int16_t  el_torque_cmd = p214_el_torque_cmd_GET(pack);
    int16_t  az_torque_cmd = p214_az_torque_cmd_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_HEARTBEAT_STATUS  status = p215_status_GET(pack);
    e_GOPRO_CAPTURE_MODE  capture_mode = p215_capture_mode_GET(pack);
    e_GOPRO_HEARTBEAT_FLAGS  flags = p215_flags_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p216_target_system_GET(pack);
    uint8_t  target_component = p216_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p216_cmd_id_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p217_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p217_status_GET(pack);
    uint8_t*  value = p217_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p218_target_system_GET(pack);
    uint8_t  target_component = p218_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p218_cmd_id_GET(pack);
    uint8_t*  value = p218_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p219_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p219_status_GET(pack);
}
void c_CommunicationChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    float  rpm1 = p226_rpm1_GET(pack);
    float  rpm2 = p226_rpm2_GET(pack);
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
void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    e_UAVIONIX_ADSB_RF_HEALTH  rfHealth = p10003_rfHealth_GET(pack);
}
void c_CommunicationChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11001_request_id_GET(pack);
    uint8_t  result = p11001_result_GET(pack);
    uint8_t  regstart = p11001_regstart_GET(pack);
    uint8_t  count = p11001_count_GET(pack);
    uint8_t*  data_ = p11001_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11003_request_id_GET(pack);
    uint8_t  result = p11003_result_GET(pack);
}
void c_CommunicationChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
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
    setPack(c_CommunicationChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_QUADROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)902748409L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_EMERGENCY, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)38742, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)22979, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -26919, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)24, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)59039, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)28621, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)40312, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)36083, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)14645, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)4483, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)2576970344467052221L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)3962557245L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)1319172431L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)43433, PH.base.pack) ;
    p3_x_SET((float) -1.381459E38F, PH.base.pack) ;
    p3_y_SET((float) -3.5521484E37F, PH.base.pack) ;
    p3_z_SET((float)2.7266231E38F, PH.base.pack) ;
    p3_vx_SET((float) -1.5276421E38F, PH.base.pack) ;
    p3_vy_SET((float) -2.9233923E38F, PH.base.pack) ;
    p3_vz_SET((float)2.1860282E38F, PH.base.pack) ;
    p3_afx_SET((float)1.1849544E37F, PH.base.pack) ;
    p3_afy_SET((float)6.5140726E37F, PH.base.pack) ;
    p3_afz_SET((float) -2.3367893E38F, PH.base.pack) ;
    p3_yaw_SET((float) -3.3396434E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -2.627612E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)2028633547685169586L, PH.base.pack) ;
    p4_seq_SET((uint32_t)327733590L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    {
        char16_t   passkey = "ahjwPkvbuwul";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "jGwAwprtzeenoswtWh";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)2160556212L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    {
        char16_t   param_id = "lgmouksRk";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)28979, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "tumYvuei";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)1.8162297E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)10197, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)9808, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    {
        char16_t   param_id = "cjjcgbpwyxfE";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -2.0299704E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)21900665109915761L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
    p24_lat_SET((int32_t) -725933653, PH.base.pack) ;
    p24_lon_SET((int32_t) -1428504828, PH.base.pack) ;
    p24_alt_SET((int32_t) -1899958515, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)934, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)26586, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)6043, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)169, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -2068493973, &PH) ;
    p24_h_acc_SET((uint32_t)299212471L, &PH) ;
    p24_v_acc_SET((uint32_t)3293538922L, &PH) ;
    p24_vel_acc_SET((uint32_t)2853879149L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1440967463L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)146, (uint8_t)90, (uint8_t)248, (uint8_t)19, (uint8_t)170, (uint8_t)42, (uint8_t)10, (uint8_t)68, (uint8_t)62, (uint8_t)236, (uint8_t)134, (uint8_t)162, (uint8_t)116, (uint8_t)166, (uint8_t)2, (uint8_t)201, (uint8_t)31, (uint8_t)11, (uint8_t)203, (uint8_t)188};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)93, (uint8_t)108, (uint8_t)192, (uint8_t)25, (uint8_t)153, (uint8_t)236, (uint8_t)22, (uint8_t)44, (uint8_t)26, (uint8_t)46, (uint8_t)110, (uint8_t)223, (uint8_t)106, (uint8_t)228, (uint8_t)17, (uint8_t)96, (uint8_t)252, (uint8_t)153, (uint8_t)214, (uint8_t)6};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)50, (uint8_t)228, (uint8_t)148, (uint8_t)243, (uint8_t)74, (uint8_t)224, (uint8_t)210, (uint8_t)204, (uint8_t)212, (uint8_t)126, (uint8_t)15, (uint8_t)238, (uint8_t)219, (uint8_t)232, (uint8_t)227, (uint8_t)217, (uint8_t)204, (uint8_t)249, (uint8_t)15, (uint8_t)235};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)119, (uint8_t)74, (uint8_t)69, (uint8_t)185, (uint8_t)130, (uint8_t)133, (uint8_t)156, (uint8_t)1, (uint8_t)176, (uint8_t)165, (uint8_t)240, (uint8_t)16, (uint8_t)180, (uint8_t)212, (uint8_t)230, (uint8_t)40, (uint8_t)66, (uint8_t)143, (uint8_t)65, (uint8_t)225};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)60, (uint8_t)198, (uint8_t)203, (uint8_t)182, (uint8_t)65, (uint8_t)186, (uint8_t)106, (uint8_t)222, (uint8_t)223, (uint8_t)210, (uint8_t)37, (uint8_t)144, (uint8_t)132, (uint8_t)113, (uint8_t)105, (uint8_t)99, (uint8_t)238, (uint8_t)227, (uint8_t)9, (uint8_t)194};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)2956587433L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)15866, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)28281, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)22571, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -24851, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)30417, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -11587, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)29655, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -11511, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -13616, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)5669947668623419165L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)27563, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -10263, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)9497, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)24541, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)28904, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)28247, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -21197, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)27618, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -26630, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)4978346048737498722L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -6972, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -17572, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)19276, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)1355, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2324686405L, PH.base.pack) ;
    p29_press_abs_SET((float)9.657887E37F, PH.base.pack) ;
    p29_press_diff_SET((float) -2.2673042E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -3909, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)4220629041L, PH.base.pack) ;
    p30_roll_SET((float) -8.026949E37F, PH.base.pack) ;
    p30_pitch_SET((float)2.1377893E38F, PH.base.pack) ;
    p30_yaw_SET((float)3.130973E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)2.577529E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -1.872319E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -1.87065E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)3810765205L, PH.base.pack) ;
    p31_q1_SET((float)2.251514E38F, PH.base.pack) ;
    p31_q2_SET((float)4.150471E37F, PH.base.pack) ;
    p31_q3_SET((float)1.6284123E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.1908702E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -2.5821856E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -2.7783037E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)3.1679704E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)2447010006L, PH.base.pack) ;
    p32_x_SET((float)1.1485051E38F, PH.base.pack) ;
    p32_y_SET((float)7.206134E37F, PH.base.pack) ;
    p32_z_SET((float) -2.9008784E38F, PH.base.pack) ;
    p32_vx_SET((float) -1.1787368E38F, PH.base.pack) ;
    p32_vy_SET((float) -1.3779693E38F, PH.base.pack) ;
    p32_vz_SET((float)2.8525372E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)3984704987L, PH.base.pack) ;
    p33_lat_SET((int32_t)1474735945, PH.base.pack) ;
    p33_lon_SET((int32_t)1204239249, PH.base.pack) ;
    p33_alt_SET((int32_t) -2089978999, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -2008568829, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)18682, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)22893, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)11201, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)1553, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)1347569932L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -27323, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)11888, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -13835, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -9087, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -7791, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)27023, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -28835, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)19637, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)1542749622L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)61703, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)39445, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)1872, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)58369, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)40909, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)46550, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)62988, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)35460, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)100625686L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)65341, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)10536, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)43949, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)42271, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)4867, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)45790, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)25378, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)50202, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)47218, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)20275, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)57690, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)58190, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)62209, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)39467, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)12471, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)40154, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)18712, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -14433, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)3803, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)19212, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)11601, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_GIMBAL_RESET, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p39_param1_SET((float) -2.4362504E38F, PH.base.pack) ;
    p39_param2_SET((float)2.0122207E38F, PH.base.pack) ;
    p39_param3_SET((float)2.890126E38F, PH.base.pack) ;
    p39_param4_SET((float)3.0429058E38F, PH.base.pack) ;
    p39_x_SET((float) -1.7681943E38F, PH.base.pack) ;
    p39_y_SET((float) -3.1473657E38F, PH.base.pack) ;
    p39_z_SET((float) -9.455409E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)50000, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)54161, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)382, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)17607, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)40243, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p48_latitude_SET((int32_t)318419662, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1945619367, PH.base.pack) ;
    p48_altitude_SET((int32_t)1974680580, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)1772641140734412642L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -509355130, PH.base.pack) ;
    p49_longitude_SET((int32_t) -75745313, PH.base.pack) ;
    p49_altitude_SET((int32_t) -1949572094, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)126820278028772799L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    {
        char16_t   param_id = "pDnvzadgnqn";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)31901, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p50_param_value0_SET((float)1.3901278E36F, PH.base.pack) ;
    p50_scale_SET((float) -1.987513E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.9465444E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)2.050965E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)10200, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p54_p1x_SET((float)3.214908E37F, PH.base.pack) ;
    p54_p1y_SET((float)3.1614378E38F, PH.base.pack) ;
    p54_p1z_SET((float)1.9630624E38F, PH.base.pack) ;
    p54_p2x_SET((float) -2.8483062E38F, PH.base.pack) ;
    p54_p2y_SET((float) -2.9021223E37F, PH.base.pack) ;
    p54_p2z_SET((float)1.6377742E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p55_p1x_SET((float)2.014302E38F, PH.base.pack) ;
    p55_p1y_SET((float)2.7247334E38F, PH.base.pack) ;
    p55_p1z_SET((float)1.9597858E38F, PH.base.pack) ;
    p55_p2x_SET((float) -1.2051207E38F, PH.base.pack) ;
    p55_p2y_SET((float)1.6798859E38F, PH.base.pack) ;
    p55_p2z_SET((float)3.0604004E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)4627656768621944129L, PH.base.pack) ;
    {
        float  q [] =  {2.007579E38F, -3.3618846E38F, -8.556479E36F, -3.2178288E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)2.1749538E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)2.9673013E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)3.1927551E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.277241E38F, 1.5542717E38F, 2.4515728E38F, 2.8380697E38F, 9.877844E37F, -5.5390987E37F, -2.3484903E38F, -2.9225983E38F, 2.8587294E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -1.3934667E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -1.842519E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)9985, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)1213, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)9338, PH.base.pack) ;
    p62_alt_error_SET((float) -4.6632383E37F, PH.base.pack) ;
    p62_aspd_error_SET((float) -9.235859E37F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -1.7600964E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)6436841421683259053L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t)1361396808, PH.base.pack) ;
    p63_lon_SET((int32_t) -1038354189, PH.base.pack) ;
    p63_alt_SET((int32_t)1042864321, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1822587350, PH.base.pack) ;
    p63_vx_SET((float)1.7300066E38F, PH.base.pack) ;
    p63_vy_SET((float) -2.0220122E38F, PH.base.pack) ;
    p63_vz_SET((float)8.0951503E37F, PH.base.pack) ;
    {
        float  covariance [] =  {1.8364515E38F, 2.4559641E38F, 3.0370483E38F, -5.100427E37F, -2.0095947E38F, 9.081663E37F, 2.2849978E37F, 2.1344845E38F, 3.3678661E38F, 1.684776E38F, -1.7223717E38F, -1.1913087E38F, 2.3587061E38F, 1.7339675E38F, -1.750446E38F, -3.3611476E38F, 2.8619005E38F, 3.0554393E38F, 2.6240686E38F, 5.346042E37F, -2.148115E38F, 1.22967E38F, -6.9407654E36F, 2.4254701E38F, -1.9940803E38F, 1.883921E38F, -9.96544E36F, -2.1906513E38F, 2.7386058E38F, -1.8710926E38F, -2.5335462E38F, -9.258799E37F, -5.7929903E37F, -2.5835E38F, -7.506146E37F, -2.8737455E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)1097785098529539372L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p64_x_SET((float)2.8622812E38F, PH.base.pack) ;
    p64_y_SET((float) -2.5639521E38F, PH.base.pack) ;
    p64_z_SET((float) -3.090438E38F, PH.base.pack) ;
    p64_vx_SET((float) -3.3301443E38F, PH.base.pack) ;
    p64_vy_SET((float) -1.6597861E37F, PH.base.pack) ;
    p64_vz_SET((float) -3.2067552E38F, PH.base.pack) ;
    p64_ax_SET((float)3.8409975E37F, PH.base.pack) ;
    p64_ay_SET((float)2.482608E37F, PH.base.pack) ;
    p64_az_SET((float) -2.6855705E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.9931149E38F, 2.8890248E38F, -2.7588227E37F, -3.397143E38F, -1.3613182E38F, 1.0957596E38F, -2.399727E38F, 8.171124E37F, -5.982767E37F, -2.3254862E38F, 5.386471E37F, -2.9357494E38F, -1.5223249E38F, -3.1058261E38F, -2.3319175E38F, -7.48799E36F, -9.869108E37F, 1.8816845E38F, 2.669811E38F, -1.7097743E38F, 9.3589325E36F, 2.273477E38F, 1.0728635E38F, 2.9465277E38F, -1.8499003E37F, 2.2087725E38F, 4.6509612E36F, 3.016908E38F, -2.188984E38F, -1.8562534E38F, 1.905424E38F, 2.660307E37F, 2.5908884E38F, 9.3536116E36F, -2.8771125E38F, 1.8877985E38F, -2.9462313E38F, 6.297875E37F, -3.2327938E38F, 3.3685135E38F, -1.6785072E38F, 2.2748381E38F, 1.0101162E38F, 2.194478E38F, -6.778576E36F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)2121503391L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)15407, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)29356, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)18153, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)30028, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)43566, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)9470, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)34852, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)26427, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)57307, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)23448, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)161, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)29221, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)63291, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)12227, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)5816, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)8218, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)17681, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)49859, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)19066, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)60718, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -11615, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -20883, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -28028, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)32241, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)59911, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)11374, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)9101, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)44827, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)10627, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)2638, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)38092, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)37347, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)37312, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)28286, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p73_param1_SET((float) -1.1933716E38F, PH.base.pack) ;
    p73_param2_SET((float) -2.5280766E38F, PH.base.pack) ;
    p73_param3_SET((float)7.109458E37F, PH.base.pack) ;
    p73_param4_SET((float)1.3498824E38F, PH.base.pack) ;
    p73_x_SET((int32_t)2082329235, PH.base.pack) ;
    p73_y_SET((int32_t)1451767973, PH.base.pack) ;
    p73_z_SET((float)1.8652736E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -3.2700055E36F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.6034743E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -20850, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)47616, PH.base.pack) ;
    p74_alt_SET((float)2.9763396E38F, PH.base.pack) ;
    p74_climb_SET((float) -1.2467532E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_PANORAMA_CREATE, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p75_param1_SET((float) -2.2766688E38F, PH.base.pack) ;
    p75_param2_SET((float)2.4960306E38F, PH.base.pack) ;
    p75_param3_SET((float)6.028808E37F, PH.base.pack) ;
    p75_param4_SET((float)1.8977232E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -1317636642, PH.base.pack) ;
    p75_y_SET((int32_t)604051811, PH.base.pack) ;
    p75_z_SET((float)2.0330631E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_HOME, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p76_param1_SET((float)1.9214186E38F, PH.base.pack) ;
    p76_param2_SET((float) -2.168263E38F, PH.base.pack) ;
    p76_param3_SET((float)2.4299883E37F, PH.base.pack) ;
    p76_param4_SET((float) -2.1607228E38F, PH.base.pack) ;
    p76_param5_SET((float) -1.7974341E36F, PH.base.pack) ;
    p76_param6_SET((float) -9.243461E37F, PH.base.pack) ;
    p76_param7_SET((float) -2.877551E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)215, &PH) ;
    p77_result_param2_SET((int32_t) -2009668145, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)199, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)218, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)2938673902L, PH.base.pack) ;
    p81_roll_SET((float) -2.2769356E38F, PH.base.pack) ;
    p81_pitch_SET((float)1.1702322E38F, PH.base.pack) ;
    p81_yaw_SET((float)8.2528176E37F, PH.base.pack) ;
    p81_thrust_SET((float)2.8460963E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)3233990947L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    {
        float  q [] =  {-1.4739107E38F, -2.8156554E38F, 1.9361057E38F, 2.3175746E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -4.275357E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -3.1435129E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -9.6220784E36F, PH.base.pack) ;
    p82_thrust_SET((float) -7.139026E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)4043805613L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    {
        float  q [] =  {-2.2946247E38F, -2.939486E38F, -8.0728584E37F, -1.6326117E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -3.1971817E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -2.1897257E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)9.450082E37F, PH.base.pack) ;
    p83_thrust_SET((float) -1.5299162E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)322248660L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)63133, PH.base.pack) ;
    p84_x_SET((float)2.9177299E38F, PH.base.pack) ;
    p84_y_SET((float) -9.589559E35F, PH.base.pack) ;
    p84_z_SET((float)1.7002512E38F, PH.base.pack) ;
    p84_vx_SET((float)6.1948604E37F, PH.base.pack) ;
    p84_vy_SET((float) -2.146684E38F, PH.base.pack) ;
    p84_vz_SET((float) -3.3655558E38F, PH.base.pack) ;
    p84_afx_SET((float)1.4376373E38F, PH.base.pack) ;
    p84_afy_SET((float) -1.6658015E38F, PH.base.pack) ;
    p84_afz_SET((float) -5.0889554E37F, PH.base.pack) ;
    p84_yaw_SET((float)2.0163885E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)1.1725341E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1366686271L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)47625, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -1913749158, PH.base.pack) ;
    p86_lon_int_SET((int32_t)131632362, PH.base.pack) ;
    p86_alt_SET((float) -1.3153617E38F, PH.base.pack) ;
    p86_vx_SET((float)1.0911667E38F, PH.base.pack) ;
    p86_vy_SET((float)3.0189131E38F, PH.base.pack) ;
    p86_vz_SET((float)2.444909E38F, PH.base.pack) ;
    p86_afx_SET((float) -1.8520166E38F, PH.base.pack) ;
    p86_afy_SET((float) -1.8256465E38F, PH.base.pack) ;
    p86_afz_SET((float) -1.6606462E38F, PH.base.pack) ;
    p86_yaw_SET((float) -2.549555E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)2.637669E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)3231794608L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)50888, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -1272107931, PH.base.pack) ;
    p87_lon_int_SET((int32_t)582328490, PH.base.pack) ;
    p87_alt_SET((float)1.6810336E38F, PH.base.pack) ;
    p87_vx_SET((float) -2.522049E38F, PH.base.pack) ;
    p87_vy_SET((float)3.0171451E38F, PH.base.pack) ;
    p87_vz_SET((float)2.3143716E38F, PH.base.pack) ;
    p87_afx_SET((float)3.3450774E38F, PH.base.pack) ;
    p87_afy_SET((float)1.6849319E38F, PH.base.pack) ;
    p87_afz_SET((float) -2.4619687E38F, PH.base.pack) ;
    p87_yaw_SET((float)3.362078E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -2.9212895E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1077692470L, PH.base.pack) ;
    p89_x_SET((float)2.5284085E37F, PH.base.pack) ;
    p89_y_SET((float)4.2434257E37F, PH.base.pack) ;
    p89_z_SET((float)6.6111853E37F, PH.base.pack) ;
    p89_roll_SET((float)6.0046363E37F, PH.base.pack) ;
    p89_pitch_SET((float) -1.0919834E38F, PH.base.pack) ;
    p89_yaw_SET((float) -7.077903E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)1088703656815420412L, PH.base.pack) ;
    p90_roll_SET((float)9.361811E37F, PH.base.pack) ;
    p90_pitch_SET((float)2.4430235E38F, PH.base.pack) ;
    p90_yaw_SET((float) -1.3381326E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)2.546428E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float)2.1356801E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -1.6473182E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -1162913719, PH.base.pack) ;
    p90_lon_SET((int32_t)2042127240, PH.base.pack) ;
    p90_alt_SET((int32_t) -1423692272, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)516, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -5701, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)10346, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)17701, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)24224, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)29231, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)190423206241869979L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.7818122E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)6.0266244E37F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.0084652E38F, PH.base.pack) ;
    p91_throttle_SET((float)1.6549271E38F, PH.base.pack) ;
    p91_aux1_SET((float) -2.7217E36F, PH.base.pack) ;
    p91_aux2_SET((float)9.427464E37F, PH.base.pack) ;
    p91_aux3_SET((float) -1.8450326E38F, PH.base.pack) ;
    p91_aux4_SET((float)2.9452517E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)2015362750207906487L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)11180, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)49615, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)45947, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)54864, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)59306, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)42795, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)30031, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)20968, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)10002, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)44133, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)23910, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)21448, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)1216949423507807702L, PH.base.pack) ;
    {
        float  controls [] =  {1.2591294E38F, 2.4784758E38F, -9.830717E37F, -1.5822508E37F, -2.3996533E38F, -1.1558377E38F, -1.3231224E37F, -2.6653839E38F, 3.1820636E38F, -5.955974E37F, 2.2363056E38F, -1.0439756E38F, 2.5805807E38F, 1.3756324E38F, 2.8686556E38F, 2.163067E37F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)7061778567733370731L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)9162152307446148579L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -32761, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -27949, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -1.8159957E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)3.2900492E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p100_ground_distance_SET((float) -1.0422816E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)1.8262592E38F, &PH) ;
    p100_flow_rate_y_SET((float)2.553456E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)4166380978497194554L, PH.base.pack) ;
    p101_x_SET((float)9.271941E37F, PH.base.pack) ;
    p101_y_SET((float) -1.3698392E38F, PH.base.pack) ;
    p101_z_SET((float) -1.9944645E38F, PH.base.pack) ;
    p101_roll_SET((float) -1.0709461E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.0016323E37F, PH.base.pack) ;
    p101_yaw_SET((float)5.611324E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)8645096612316781725L, PH.base.pack) ;
    p102_x_SET((float) -2.4655208E38F, PH.base.pack) ;
    p102_y_SET((float)1.982442E38F, PH.base.pack) ;
    p102_z_SET((float) -1.148932E38F, PH.base.pack) ;
    p102_roll_SET((float) -2.8756518E38F, PH.base.pack) ;
    p102_pitch_SET((float) -1.9781008E38F, PH.base.pack) ;
    p102_yaw_SET((float)1.5186703E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)2789224259078844385L, PH.base.pack) ;
    p103_x_SET((float) -1.5163968E38F, PH.base.pack) ;
    p103_y_SET((float) -3.0585281E38F, PH.base.pack) ;
    p103_z_SET((float)4.593343E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)5867117564377255797L, PH.base.pack) ;
    p104_x_SET((float)3.0512434E37F, PH.base.pack) ;
    p104_y_SET((float)2.3729547E38F, PH.base.pack) ;
    p104_z_SET((float)3.3961562E38F, PH.base.pack) ;
    p104_roll_SET((float) -2.447323E38F, PH.base.pack) ;
    p104_pitch_SET((float)2.7313491E38F, PH.base.pack) ;
    p104_yaw_SET((float) -3.3198698E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)2973492867888629925L, PH.base.pack) ;
    p105_xacc_SET((float)7.963581E37F, PH.base.pack) ;
    p105_yacc_SET((float) -3.1828255E37F, PH.base.pack) ;
    p105_zacc_SET((float) -5.3495515E37F, PH.base.pack) ;
    p105_xgyro_SET((float)8.9670986E36F, PH.base.pack) ;
    p105_ygyro_SET((float) -3.1807516E37F, PH.base.pack) ;
    p105_zgyro_SET((float) -3.0963023E37F, PH.base.pack) ;
    p105_xmag_SET((float) -2.490698E38F, PH.base.pack) ;
    p105_ymag_SET((float) -1.6139335E38F, PH.base.pack) ;
    p105_zmag_SET((float) -1.2668782E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -2.1700986E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -2.8886518E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)9.649266E37F, PH.base.pack) ;
    p105_temperature_SET((float)1.2628759E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)59119, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)4957407941435115677L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)845736147L, PH.base.pack) ;
    p106_integrated_x_SET((float)2.3600543E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -1.7778664E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.3917274E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -8.015406E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)3.2027347E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -10090, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)3137702193L, PH.base.pack) ;
    p106_distance_SET((float) -2.9927768E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)6833344566523354902L, PH.base.pack) ;
    p107_xacc_SET((float)1.9137705E38F, PH.base.pack) ;
    p107_yacc_SET((float) -1.0191301E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.0863367E38F, PH.base.pack) ;
    p107_xgyro_SET((float)5.092242E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -1.2197172E37F, PH.base.pack) ;
    p107_zgyro_SET((float) -1.9297096E38F, PH.base.pack) ;
    p107_xmag_SET((float) -8.396875E37F, PH.base.pack) ;
    p107_ymag_SET((float) -2.521797E38F, PH.base.pack) ;
    p107_zmag_SET((float)1.3087481E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -2.874984E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -1.8195593E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -1.5210211E38F, PH.base.pack) ;
    p107_temperature_SET((float) -2.8161012E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1458835971L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -3.3984585E38F, PH.base.pack) ;
    p108_q2_SET((float)4.442004E37F, PH.base.pack) ;
    p108_q3_SET((float) -4.5519315E37F, PH.base.pack) ;
    p108_q4_SET((float) -1.9713634E38F, PH.base.pack) ;
    p108_roll_SET((float) -3.1304157E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.8997228E38F, PH.base.pack) ;
    p108_yaw_SET((float) -1.4939017E38F, PH.base.pack) ;
    p108_xacc_SET((float)3.1806901E38F, PH.base.pack) ;
    p108_yacc_SET((float) -2.8678556E38F, PH.base.pack) ;
    p108_zacc_SET((float) -3.5944392E37F, PH.base.pack) ;
    p108_xgyro_SET((float)2.4753281E37F, PH.base.pack) ;
    p108_ygyro_SET((float) -2.6075847E38F, PH.base.pack) ;
    p108_zgyro_SET((float)1.0606866E38F, PH.base.pack) ;
    p108_lat_SET((float) -3.0412408E36F, PH.base.pack) ;
    p108_lon_SET((float)8.3015264E37F, PH.base.pack) ;
    p108_alt_SET((float)1.1777907E37F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -3.993069E36F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)3.1822249E38F, PH.base.pack) ;
    p108_vn_SET((float) -3.337453E38F, PH.base.pack) ;
    p108_ve_SET((float) -2.865884E38F, PH.base.pack) ;
    p108_vd_SET((float)2.1485445E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)64004, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)17061, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)194, (uint8_t)77, (uint8_t)227, (uint8_t)141, (uint8_t)48, (uint8_t)125, (uint8_t)104, (uint8_t)73, (uint8_t)22, (uint8_t)55, (uint8_t)23, (uint8_t)50, (uint8_t)5, (uint8_t)182, (uint8_t)237, (uint8_t)194, (uint8_t)172, (uint8_t)223, (uint8_t)233, (uint8_t)60, (uint8_t)104, (uint8_t)236, (uint8_t)188, (uint8_t)235, (uint8_t)175, (uint8_t)71, (uint8_t)54, (uint8_t)191, (uint8_t)43, (uint8_t)248, (uint8_t)175, (uint8_t)226, (uint8_t)11, (uint8_t)176, (uint8_t)189, (uint8_t)174, (uint8_t)16, (uint8_t)42, (uint8_t)54, (uint8_t)122, (uint8_t)240, (uint8_t)28, (uint8_t)169, (uint8_t)210, (uint8_t)39, (uint8_t)131, (uint8_t)153, (uint8_t)183, (uint8_t)146, (uint8_t)71, (uint8_t)181, (uint8_t)185, (uint8_t)14, (uint8_t)38, (uint8_t)35, (uint8_t)123, (uint8_t)147, (uint8_t)223, (uint8_t)227, (uint8_t)200, (uint8_t)45, (uint8_t)200, (uint8_t)150, (uint8_t)181, (uint8_t)93, (uint8_t)123, (uint8_t)156, (uint8_t)82, (uint8_t)118, (uint8_t)205, (uint8_t)93, (uint8_t)73, (uint8_t)151, (uint8_t)9, (uint8_t)88, (uint8_t)76, (uint8_t)184, (uint8_t)142, (uint8_t)147, (uint8_t)155, (uint8_t)246, (uint8_t)131, (uint8_t)229, (uint8_t)37, (uint8_t)144, (uint8_t)217, (uint8_t)0, (uint8_t)120, (uint8_t)1, (uint8_t)57, (uint8_t)30, (uint8_t)17, (uint8_t)18, (uint8_t)211, (uint8_t)73, (uint8_t)164, (uint8_t)191, (uint8_t)59, (uint8_t)186, (uint8_t)143, (uint8_t)59, (uint8_t)254, (uint8_t)104, (uint8_t)136, (uint8_t)114, (uint8_t)26, (uint8_t)16, (uint8_t)242, (uint8_t)81, (uint8_t)27, (uint8_t)62, (uint8_t)17, (uint8_t)90, (uint8_t)236, (uint8_t)242, (uint8_t)140, (uint8_t)62, (uint8_t)147, (uint8_t)194, (uint8_t)98, (uint8_t)182, (uint8_t)177, (uint8_t)47, (uint8_t)231, (uint8_t)225, (uint8_t)196, (uint8_t)85, (uint8_t)193, (uint8_t)148, (uint8_t)193, (uint8_t)106, (uint8_t)27, (uint8_t)161, (uint8_t)6, (uint8_t)131, (uint8_t)80, (uint8_t)246, (uint8_t)128, (uint8_t)65, (uint8_t)133, (uint8_t)146, (uint8_t)47, (uint8_t)221, (uint8_t)218, (uint8_t)31, (uint8_t)151, (uint8_t)221, (uint8_t)88, (uint8_t)206, (uint8_t)16, (uint8_t)224, (uint8_t)186, (uint8_t)206, (uint8_t)214, (uint8_t)20, (uint8_t)176, (uint8_t)164, (uint8_t)111, (uint8_t)101, (uint8_t)24, (uint8_t)8, (uint8_t)189, (uint8_t)139, (uint8_t)109, (uint8_t)227, (uint8_t)11, (uint8_t)216, (uint8_t)110, (uint8_t)254, (uint8_t)135, (uint8_t)160, (uint8_t)111, (uint8_t)226, (uint8_t)229, (uint8_t)183, (uint8_t)92, (uint8_t)63, (uint8_t)216, (uint8_t)50, (uint8_t)76, (uint8_t)194, (uint8_t)90, (uint8_t)101, (uint8_t)153, (uint8_t)153, (uint8_t)118, (uint8_t)243, (uint8_t)99, (uint8_t)192, (uint8_t)128, (uint8_t)255, (uint8_t)174, (uint8_t)115, (uint8_t)247, (uint8_t)204, (uint8_t)131, (uint8_t)43, (uint8_t)118, (uint8_t)223, (uint8_t)40, (uint8_t)205, (uint8_t)216, (uint8_t)8, (uint8_t)85, (uint8_t)119, (uint8_t)17, (uint8_t)251, (uint8_t)240, (uint8_t)85, (uint8_t)138, (uint8_t)2, (uint8_t)189, (uint8_t)185, (uint8_t)109, (uint8_t)199, (uint8_t)135, (uint8_t)64, (uint8_t)249, (uint8_t)175, (uint8_t)85, (uint8_t)96, (uint8_t)81, (uint8_t)143, (uint8_t)168, (uint8_t)185, (uint8_t)20, (uint8_t)131, (uint8_t)163, (uint8_t)164, (uint8_t)11, (uint8_t)230, (uint8_t)230, (uint8_t)112, (uint8_t)159, (uint8_t)60, (uint8_t)182, (uint8_t)188, (uint8_t)62, (uint8_t)206, (uint8_t)88, (uint8_t)80, (uint8_t)67, (uint8_t)22, (uint8_t)199, (uint8_t)19, (uint8_t)95, (uint8_t)106, (uint8_t)77, (uint8_t)52, (uint8_t)22, (uint8_t)249};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)1002803806616146943L, PH.base.pack) ;
    p111_ts1_SET((int64_t)2389055101252862701L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)3695689229949931455L, PH.base.pack) ;
    p112_seq_SET((uint32_t)2994300669L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)5111985635162566129L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p113_lat_SET((int32_t) -1886948536, PH.base.pack) ;
    p113_lon_SET((int32_t)876617655, PH.base.pack) ;
    p113_alt_SET((int32_t) -352685716, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)38496, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)18601, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)26772, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -22319, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)13682, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)15792, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)11571, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)4909282218224285242L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)2325048911L, PH.base.pack) ;
    p114_integrated_x_SET((float)2.8100218E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)1.5520405E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -8.27468E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)3.0156438E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)1.1655877E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -8948, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)1235270491L, PH.base.pack) ;
    p114_distance_SET((float) -6.169455E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)2959035432251936866L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-1.6225749E38F, 8.2310683E37F, -2.821384E38F, -2.109951E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)6.386644E37F, PH.base.pack) ;
    p115_pitchspeed_SET((float)1.012991E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.1807432E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1234440937, PH.base.pack) ;
    p115_lon_SET((int32_t)1763105805, PH.base.pack) ;
    p115_alt_SET((int32_t)57588916, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -21867, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)18768, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -8014, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)17110, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)1262, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)28543, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)1724, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)24064, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)347868480L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -20837, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)7798, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)3236, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)22118, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -22201, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -1788, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)18571, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -27877, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -1871, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)40279, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)65340, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)8235, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)34513, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)54216, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)263083190L, PH.base.pack) ;
    p118_size_SET((uint32_t)521087405L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)61885, PH.base.pack) ;
    p119_ofs_SET((uint32_t)758857841L, PH.base.pack) ;
    p119_count_SET((uint32_t)3244398928L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)12969, PH.base.pack) ;
    p120_ofs_SET((uint32_t)1460339783L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)208, (uint8_t)109, (uint8_t)136, (uint8_t)237, (uint8_t)30, (uint8_t)46, (uint8_t)200, (uint8_t)33, (uint8_t)151, (uint8_t)241, (uint8_t)34, (uint8_t)111, (uint8_t)3, (uint8_t)169, (uint8_t)110, (uint8_t)171, (uint8_t)236, (uint8_t)152, (uint8_t)153, (uint8_t)142, (uint8_t)0, (uint8_t)101, (uint8_t)155, (uint8_t)150, (uint8_t)111, (uint8_t)156, (uint8_t)106, (uint8_t)185, (uint8_t)214, (uint8_t)183, (uint8_t)117, (uint8_t)36, (uint8_t)167, (uint8_t)231, (uint8_t)76, (uint8_t)8, (uint8_t)105, (uint8_t)112, (uint8_t)100, (uint8_t)253, (uint8_t)7, (uint8_t)4, (uint8_t)78, (uint8_t)225, (uint8_t)118, (uint8_t)199, (uint8_t)63, (uint8_t)9, (uint8_t)41, (uint8_t)106, (uint8_t)118, (uint8_t)130, (uint8_t)36, (uint8_t)38, (uint8_t)117, (uint8_t)105, (uint8_t)159, (uint8_t)156, (uint8_t)55, (uint8_t)95, (uint8_t)235, (uint8_t)115, (uint8_t)55, (uint8_t)161, (uint8_t)223, (uint8_t)166, (uint8_t)188, (uint8_t)251, (uint8_t)126, (uint8_t)89, (uint8_t)249, (uint8_t)239, (uint8_t)105, (uint8_t)96, (uint8_t)42, (uint8_t)55, (uint8_t)247, (uint8_t)25, (uint8_t)90, (uint8_t)210, (uint8_t)105, (uint8_t)235, (uint8_t)24, (uint8_t)245, (uint8_t)82, (uint8_t)105, (uint8_t)32, (uint8_t)102, (uint8_t)138, (uint8_t)57};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)46, (uint8_t)167, (uint8_t)229, (uint8_t)231, (uint8_t)27, (uint8_t)42, (uint8_t)224, (uint8_t)204, (uint8_t)92, (uint8_t)19, (uint8_t)214, (uint8_t)24, (uint8_t)81, (uint8_t)169, (uint8_t)87, (uint8_t)234, (uint8_t)161, (uint8_t)20, (uint8_t)176, (uint8_t)94, (uint8_t)99, (uint8_t)68, (uint8_t)142, (uint8_t)115, (uint8_t)165, (uint8_t)239, (uint8_t)117, (uint8_t)88, (uint8_t)50, (uint8_t)150, (uint8_t)125, (uint8_t)58, (uint8_t)222, (uint8_t)201, (uint8_t)26, (uint8_t)141, (uint8_t)55, (uint8_t)207, (uint8_t)96, (uint8_t)228, (uint8_t)106, (uint8_t)160, (uint8_t)138, (uint8_t)81, (uint8_t)19, (uint8_t)116, (uint8_t)104, (uint8_t)59, (uint8_t)162, (uint8_t)158, (uint8_t)56, (uint8_t)137, (uint8_t)197, (uint8_t)70, (uint8_t)169, (uint8_t)76, (uint8_t)99, (uint8_t)109, (uint8_t)25, (uint8_t)213, (uint8_t)180, (uint8_t)161, (uint8_t)137, (uint8_t)117, (uint8_t)192, (uint8_t)130, (uint8_t)199, (uint8_t)41, (uint8_t)247, (uint8_t)138, (uint8_t)89, (uint8_t)238, (uint8_t)102, (uint8_t)239, (uint8_t)73, (uint8_t)16, (uint8_t)96, (uint8_t)181, (uint8_t)92, (uint8_t)74, (uint8_t)189, (uint8_t)248, (uint8_t)170, (uint8_t)70, (uint8_t)6, (uint8_t)85, (uint8_t)77, (uint8_t)155, (uint8_t)233, (uint8_t)158, (uint8_t)203, (uint8_t)202, (uint8_t)146, (uint8_t)157, (uint8_t)236, (uint8_t)116, (uint8_t)171, (uint8_t)28, (uint8_t)182, (uint8_t)100, (uint8_t)177, (uint8_t)25, (uint8_t)50, (uint8_t)220, (uint8_t)179, (uint8_t)104, (uint8_t)45, (uint8_t)9, (uint8_t)177, (uint8_t)40};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)3151860332989641905L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t)447722298, PH.base.pack) ;
    p124_lon_SET((int32_t) -737740301, PH.base.pack) ;
    p124_alt_SET((int32_t)635740608, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)56163, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)59960, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)28534, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)41507, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)805647235L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)16478, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)6239, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)60998, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)4075021516L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)198, (uint8_t)22, (uint8_t)25, (uint8_t)109, (uint8_t)206, (uint8_t)19, (uint8_t)48, (uint8_t)151, (uint8_t)95, (uint8_t)36, (uint8_t)18, (uint8_t)61, (uint8_t)205, (uint8_t)237, (uint8_t)18, (uint8_t)69, (uint8_t)253, (uint8_t)11, (uint8_t)227, (uint8_t)126, (uint8_t)174, (uint8_t)66, (uint8_t)55, (uint8_t)225, (uint8_t)102, (uint8_t)82, (uint8_t)247, (uint8_t)87, (uint8_t)13, (uint8_t)60, (uint8_t)119, (uint8_t)19, (uint8_t)81, (uint8_t)146, (uint8_t)103, (uint8_t)172, (uint8_t)64, (uint8_t)35, (uint8_t)112, (uint8_t)71, (uint8_t)198, (uint8_t)117, (uint8_t)1, (uint8_t)146, (uint8_t)103, (uint8_t)252, (uint8_t)165, (uint8_t)94, (uint8_t)194, (uint8_t)66, (uint8_t)12, (uint8_t)34, (uint8_t)20, (uint8_t)106, (uint8_t)28, (uint8_t)118, (uint8_t)166, (uint8_t)85, (uint8_t)143, (uint8_t)56, (uint8_t)233, (uint8_t)231, (uint8_t)240, (uint8_t)13, (uint8_t)53, (uint8_t)157, (uint8_t)129, (uint8_t)65, (uint8_t)2, (uint8_t)5};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)973238766L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)10059, PH.base.pack) ;
    p127_tow_SET((uint32_t)2198562919L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)1797994122, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)295624395, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -48462184, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2414534181L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)477486363, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)1338939290L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)58237, PH.base.pack) ;
    p128_tow_SET((uint32_t)4080779864L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)1242769666, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -172122106, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -1280245578, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1599455590L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)81206087, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)762905140L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)3252, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -6139, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)24431, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -18385, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -15259, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -10010, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -18640, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)13836, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -5725, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p130_size_SET((uint32_t)1088802749L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)44467, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)38931, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)64852, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)57495, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)29, (uint8_t)104, (uint8_t)74, (uint8_t)163, (uint8_t)200, (uint8_t)61, (uint8_t)1, (uint8_t)33, (uint8_t)232, (uint8_t)201, (uint8_t)10, (uint8_t)242, (uint8_t)82, (uint8_t)41, (uint8_t)223, (uint8_t)239, (uint8_t)7, (uint8_t)48, (uint8_t)163, (uint8_t)157, (uint8_t)251, (uint8_t)83, (uint8_t)210, (uint8_t)242, (uint8_t)243, (uint8_t)146, (uint8_t)21, (uint8_t)88, (uint8_t)27, (uint8_t)139, (uint8_t)116, (uint8_t)99, (uint8_t)72, (uint8_t)131, (uint8_t)39, (uint8_t)15, (uint8_t)235, (uint8_t)142, (uint8_t)19, (uint8_t)243, (uint8_t)93, (uint8_t)57, (uint8_t)159, (uint8_t)168, (uint8_t)141, (uint8_t)31, (uint8_t)90, (uint8_t)27, (uint8_t)240, (uint8_t)76, (uint8_t)23, (uint8_t)98, (uint8_t)165, (uint8_t)232, (uint8_t)217, (uint8_t)56, (uint8_t)136, (uint8_t)236, (uint8_t)70, (uint8_t)93, (uint8_t)75, (uint8_t)125, (uint8_t)73, (uint8_t)137, (uint8_t)22, (uint8_t)85, (uint8_t)247, (uint8_t)127, (uint8_t)99, (uint8_t)126, (uint8_t)74, (uint8_t)42, (uint8_t)89, (uint8_t)159, (uint8_t)215, (uint8_t)244, (uint8_t)36, (uint8_t)89, (uint8_t)82, (uint8_t)89, (uint8_t)134, (uint8_t)250, (uint8_t)21, (uint8_t)24, (uint8_t)230, (uint8_t)111, (uint8_t)165, (uint8_t)66, (uint8_t)179, (uint8_t)160, (uint8_t)68, (uint8_t)194, (uint8_t)146, (uint8_t)38, (uint8_t)98, (uint8_t)125, (uint8_t)187, (uint8_t)45, (uint8_t)127, (uint8_t)158, (uint8_t)251, (uint8_t)119, (uint8_t)206, (uint8_t)188, (uint8_t)92, (uint8_t)152, (uint8_t)126, (uint8_t)102, (uint8_t)129, (uint8_t)93, (uint8_t)82, (uint8_t)104, (uint8_t)72, (uint8_t)167, (uint8_t)214, (uint8_t)12, (uint8_t)233, (uint8_t)148, (uint8_t)248, (uint8_t)51, (uint8_t)181, (uint8_t)77, (uint8_t)55, (uint8_t)192, (uint8_t)4, (uint8_t)112, (uint8_t)166, (uint8_t)114, (uint8_t)203, (uint8_t)78, (uint8_t)158, (uint8_t)65, (uint8_t)144, (uint8_t)109, (uint8_t)79, (uint8_t)240, (uint8_t)31, (uint8_t)86, (uint8_t)101, (uint8_t)207, (uint8_t)247, (uint8_t)172, (uint8_t)86, (uint8_t)68, (uint8_t)49, (uint8_t)76, (uint8_t)91, (uint8_t)31, (uint8_t)153, (uint8_t)204, (uint8_t)36, (uint8_t)149, (uint8_t)142, (uint8_t)35, (uint8_t)143, (uint8_t)109, (uint8_t)85, (uint8_t)200, (uint8_t)231, (uint8_t)98, (uint8_t)8, (uint8_t)205, (uint8_t)14, (uint8_t)218, (uint8_t)199, (uint8_t)7, (uint8_t)105, (uint8_t)5, (uint8_t)245, (uint8_t)98, (uint8_t)96, (uint8_t)43, (uint8_t)166, (uint8_t)126, (uint8_t)194, (uint8_t)165, (uint8_t)214, (uint8_t)42, (uint8_t)69, (uint8_t)110, (uint8_t)41, (uint8_t)31, (uint8_t)161, (uint8_t)47, (uint8_t)171, (uint8_t)170, (uint8_t)151, (uint8_t)133, (uint8_t)3, (uint8_t)62, (uint8_t)203, (uint8_t)194, (uint8_t)223, (uint8_t)94, (uint8_t)239, (uint8_t)156, (uint8_t)203, (uint8_t)249, (uint8_t)213, (uint8_t)22, (uint8_t)0, (uint8_t)16, (uint8_t)65, (uint8_t)214, (uint8_t)58, (uint8_t)9, (uint8_t)119, (uint8_t)61, (uint8_t)228, (uint8_t)97, (uint8_t)176, (uint8_t)234, (uint8_t)90, (uint8_t)204, (uint8_t)71, (uint8_t)121, (uint8_t)164, (uint8_t)160, (uint8_t)11, (uint8_t)175, (uint8_t)111, (uint8_t)129, (uint8_t)230, (uint8_t)230, (uint8_t)104, (uint8_t)57, (uint8_t)78, (uint8_t)89, (uint8_t)241, (uint8_t)55, (uint8_t)236, (uint8_t)211, (uint8_t)15, (uint8_t)111, (uint8_t)196, (uint8_t)25, (uint8_t)99, (uint8_t)91, (uint8_t)232, (uint8_t)128, (uint8_t)27, (uint8_t)84, (uint8_t)68, (uint8_t)243, (uint8_t)114, (uint8_t)84, (uint8_t)166, (uint8_t)35, (uint8_t)195, (uint8_t)175, (uint8_t)127, (uint8_t)111, (uint8_t)150};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1274496853L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)53044, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)59591, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)4357, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1361929616, PH.base.pack) ;
    p133_lon_SET((int32_t)1448868564, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)21412, PH.base.pack) ;
    p133_mask_SET((uint64_t)6751572478215876997L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -1059049305, PH.base.pack) ;
    p134_lon_SET((int32_t)90980378, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)4657, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)19904, (int16_t) -16837, (int16_t)2292, (int16_t)3545, (int16_t) -10600, (int16_t) -24036, (int16_t)3838, (int16_t)17457, (int16_t) -23579, (int16_t)15657, (int16_t)18505, (int16_t)28603, (int16_t)13630, (int16_t) -28373, (int16_t) -6079, (int16_t) -8528};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1239371339, PH.base.pack) ;
    p135_lon_SET((int32_t)763540852, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -497900058, PH.base.pack) ;
    p136_lon_SET((int32_t) -1792958650, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)39960, PH.base.pack) ;
    p136_terrain_height_SET((float)1.935782E38F, PH.base.pack) ;
    p136_current_height_SET((float) -1.3325036E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)54015, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)25918, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)2876193623L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.4311344E37F, PH.base.pack) ;
    p137_press_diff_SET((float) -2.5262703E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)8677, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)7407675982385319990L, PH.base.pack) ;
    {
        float  q [] =  {-9.079832E36F, -2.191129E38F, -4.9609395E37F, -1.7900959E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -4.7723364E37F, PH.base.pack) ;
    p138_y_SET((float)3.3799745E38F, PH.base.pack) ;
    p138_z_SET((float) -2.509648E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)4026649571297949966L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    {
        float  controls [] =  {7.077803E37F, 1.235005E38F, -2.7756092E37F, -2.6937906E38F, -2.4155078E38F, 3.086462E38F, -2.5940969E38F, -6.1301895E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)6008245172791753906L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    {
        float  controls [] =  {1.1748112E38F, -7.87661E37F, -1.8203759E38F, 1.912045E38F, -3.3515175E38F, -9.728546E37F, -3.6014988E37F, -1.981855E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)3001778954476449869L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float)1.5925982E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float)3.306794E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.9243025E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -1.9808286E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)3.336783E37F, PH.base.pack) ;
    p141_bottom_clearance_SET((float) -1.6301701E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)244, (uint8_t)25, (uint8_t)114, (uint8_t)110, (uint8_t)164, (uint8_t)64, (uint8_t)35, (uint8_t)226, (uint8_t)158, (uint8_t)174, (uint8_t)84, (uint8_t)87, (uint8_t)148, (uint8_t)181, (uint8_t)115, (uint8_t)246, (uint8_t)46, (uint8_t)23, (uint8_t)11, (uint8_t)56, (uint8_t)18, (uint8_t)26, (uint8_t)18, (uint8_t)59, (uint8_t)26, (uint8_t)175, (uint8_t)246, (uint8_t)244, (uint8_t)7, (uint8_t)189, (uint8_t)147, (uint8_t)230, (uint8_t)122, (uint8_t)71, (uint8_t)146, (uint8_t)213, (uint8_t)160, (uint8_t)17, (uint8_t)26, (uint8_t)146, (uint8_t)38, (uint8_t)127, (uint8_t)97, (uint8_t)36, (uint8_t)115, (uint8_t)81, (uint8_t)199, (uint8_t)7, (uint8_t)27, (uint8_t)75, (uint8_t)255, (uint8_t)40, (uint8_t)185, (uint8_t)60, (uint8_t)15, (uint8_t)57, (uint8_t)19, (uint8_t)99, (uint8_t)228, (uint8_t)60, (uint8_t)47, (uint8_t)137, (uint8_t)141, (uint8_t)120, (uint8_t)48, (uint8_t)180, (uint8_t)100, (uint8_t)162, (uint8_t)123, (uint8_t)254, (uint8_t)255, (uint8_t)11, (uint8_t)58, (uint8_t)102, (uint8_t)49, (uint8_t)6, (uint8_t)44, (uint8_t)214, (uint8_t)120, (uint8_t)118, (uint8_t)183, (uint8_t)46, (uint8_t)166, (uint8_t)222, (uint8_t)11, (uint8_t)195, (uint8_t)38, (uint8_t)114, (uint8_t)5, (uint8_t)58, (uint8_t)160, (uint8_t)58, (uint8_t)29, (uint8_t)91, (uint8_t)1, (uint8_t)229, (uint8_t)230, (uint8_t)127, (uint8_t)224, (uint8_t)119, (uint8_t)27, (uint8_t)51, (uint8_t)234, (uint8_t)88, (uint8_t)61, (uint8_t)150, (uint8_t)57, (uint8_t)1, (uint8_t)233, (uint8_t)23, (uint8_t)203, (uint8_t)25, (uint8_t)245, (uint8_t)59, (uint8_t)62, (uint8_t)242, (uint8_t)89, (uint8_t)176, (uint8_t)196, (uint8_t)124};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)138, (uint8_t)41, (uint8_t)180, (uint8_t)68, (uint8_t)49, (uint8_t)99, (uint8_t)114, (uint8_t)136, (uint8_t)133, (uint8_t)73, (uint8_t)144, (uint8_t)170, (uint8_t)225, (uint8_t)194, (uint8_t)227, (uint8_t)249, (uint8_t)133, (uint8_t)20, (uint8_t)58, (uint8_t)97, (uint8_t)14, (uint8_t)148, (uint8_t)84, (uint8_t)90, (uint8_t)210, (uint8_t)58, (uint8_t)219, (uint8_t)146, (uint8_t)89, (uint8_t)75, (uint8_t)244, (uint8_t)179, (uint8_t)6, (uint8_t)61, (uint8_t)63, (uint8_t)89, (uint8_t)92, (uint8_t)187, (uint8_t)231, (uint8_t)172, (uint8_t)147, (uint8_t)182, (uint8_t)168, (uint8_t)109, (uint8_t)167, (uint8_t)172, (uint8_t)116, (uint8_t)126, (uint8_t)64, (uint8_t)23, (uint8_t)227, (uint8_t)125, (uint8_t)31, (uint8_t)8, (uint8_t)176, (uint8_t)94, (uint8_t)98, (uint8_t)237, (uint8_t)241, (uint8_t)17, (uint8_t)29, (uint8_t)107, (uint8_t)185, (uint8_t)166, (uint8_t)131, (uint8_t)169, (uint8_t)85, (uint8_t)68, (uint8_t)64, (uint8_t)44, (uint8_t)63, (uint8_t)188, (uint8_t)143, (uint8_t)57, (uint8_t)48, (uint8_t)75, (uint8_t)253, (uint8_t)103, (uint8_t)59, (uint8_t)237, (uint8_t)247, (uint8_t)100, (uint8_t)83, (uint8_t)160, (uint8_t)8, (uint8_t)89, (uint8_t)195, (uint8_t)240, (uint8_t)27, (uint8_t)240, (uint8_t)165, (uint8_t)142, (uint8_t)82, (uint8_t)213, (uint8_t)95, (uint8_t)195, (uint8_t)201, (uint8_t)174, (uint8_t)188, (uint8_t)174, (uint8_t)231, (uint8_t)183, (uint8_t)33, (uint8_t)192, (uint8_t)235, (uint8_t)229, (uint8_t)114, (uint8_t)48, (uint8_t)133, (uint8_t)186, (uint8_t)194, (uint8_t)101, (uint8_t)231, (uint8_t)220, (uint8_t)32, (uint8_t)236, (uint8_t)239, (uint8_t)175, (uint8_t)96, (uint8_t)209};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)4064971413L, PH.base.pack) ;
    p143_press_abs_SET((float)1.823381E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.3030647E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -19305, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)3322048043900943169L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p144_lat_SET((int32_t) -1915835208, PH.base.pack) ;
    p144_lon_SET((int32_t) -391938531, PH.base.pack) ;
    p144_alt_SET((float)1.017745E38F, PH.base.pack) ;
    {
        float  vel [] =  {6.8601E36F, -2.23291E38F, -9.722542E35F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {3.790598E37F, -4.8209173E37F, -1.3089799E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {1.718048E38F, -2.2357754E38F, 1.6145371E38F, 3.0726615E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.1093222E38F, -5.779545E37F, -2.2621097E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {3.2223504E38F, 1.9478543E38F, -3.29298E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)3133920735802685074L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)1603033739778505388L, PH.base.pack) ;
    p146_x_acc_SET((float)1.6556763E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -3.0213375E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -2.8237974E38F, PH.base.pack) ;
    p146_x_vel_SET((float)2.0328568E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -2.2204695E38F, PH.base.pack) ;
    p146_z_vel_SET((float)2.337027E38F, PH.base.pack) ;
    p146_x_pos_SET((float)2.1667917E38F, PH.base.pack) ;
    p146_y_pos_SET((float)1.3383952E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -6.380707E36F, PH.base.pack) ;
    p146_airspeed_SET((float)1.8742995E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {9.71783E37F, -2.5972224E38F, -2.6221255E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-2.7302145E38F, -1.7844957E38F, 3.3186317E37F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-2.8064701E38F, 2.920258E38F, -2.3293045E38F, -3.3838765E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -2.320359E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -3.329316E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)1.6045265E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t)24279, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)9252, (uint16_t)36865, (uint16_t)32441, (uint16_t)13078, (uint16_t)24929, (uint16_t)28680, (uint16_t)25085, (uint16_t)64232, (uint16_t)37409, (uint16_t)32947};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -22548, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -1571546174, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -452802755, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)102, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1634001620L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)3948794647L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)3486127943L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)2941910691L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)196, (uint8_t)124, (uint8_t)20, (uint8_t)112, (uint8_t)82, (uint8_t)165, (uint8_t)91, (uint8_t)145};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)97, (uint8_t)158, (uint8_t)53, (uint8_t)135, (uint8_t)134, (uint8_t)238, (uint8_t)142, (uint8_t)139};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)234, (uint8_t)27, (uint8_t)96, (uint8_t)85, (uint8_t)232, (uint8_t)14, (uint8_t)54, (uint8_t)193};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)39879, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)4376, PH.base.pack) ;
    p148_uid_SET((uint64_t)3052731985024669542L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)161, (uint8_t)79, (uint8_t)152, (uint8_t)85, (uint8_t)179, (uint8_t)158, (uint8_t)0, (uint8_t)210, (uint8_t)251, (uint8_t)94, (uint8_t)137, (uint8_t)143, (uint8_t)180, (uint8_t)176, (uint8_t)70, (uint8_t)239, (uint8_t)19, (uint8_t)18};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)3334077466982715478L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p149_angle_x_SET((float)1.5677371E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -6.324302E37F, PH.base.pack) ;
    p149_distance_SET((float)2.5137883E38F, PH.base.pack) ;
    p149_size_x_SET((float)9.421149E37F, PH.base.pack) ;
    p149_size_y_SET((float)2.4178697E38F, PH.base.pack) ;
    p149_x_SET((float)3.1316507E37F, &PH) ;
    p149_y_SET((float) -3.3610553E38F, &PH) ;
    p149_z_SET((float) -9.715622E37F, &PH) ;
    {
        float  q [] =  {-5.0399303E37F, -1.8672081E37F, 1.5801711E38F, 1.2233957E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)43, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)5932833818981809309L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, PH.base.pack) ;
    p230_vel_ratio_SET((float)2.810005E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)2.8654534E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -2.1514385E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float)8.613473E36F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)2.8961886E37F, PH.base.pack) ;
    p230_tas_ratio_SET((float)2.2586195E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)2.2737011E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)1.6447096E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)4141569287957461083L, PH.base.pack) ;
    p231_wind_x_SET((float) -2.2155549E38F, PH.base.pack) ;
    p231_wind_y_SET((float)2.1836981E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -2.4371153E38F, PH.base.pack) ;
    p231_var_horiz_SET((float)2.5562053E38F, PH.base.pack) ;
    p231_var_vert_SET((float) -2.090957E38F, PH.base.pack) ;
    p231_wind_alt_SET((float)6.198796E37F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)8.821844E36F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)5.5421035E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)8059382939713231713L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)3187688318L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)48201, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p232_lat_SET((int32_t) -256250198, PH.base.pack) ;
    p232_lon_SET((int32_t)1129419865, PH.base.pack) ;
    p232_alt_SET((float)3.3221154E37F, PH.base.pack) ;
    p232_hdop_SET((float) -4.7342313E37F, PH.base.pack) ;
    p232_vdop_SET((float)1.236975E38F, PH.base.pack) ;
    p232_vn_SET((float)3.313871E38F, PH.base.pack) ;
    p232_ve_SET((float) -5.735308E37F, PH.base.pack) ;
    p232_vd_SET((float)2.0025934E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float)1.94552E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float)6.8996075E35F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -1.1476737E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)156, (uint8_t)118, (uint8_t)72, (uint8_t)184, (uint8_t)77, (uint8_t)60, (uint8_t)205, (uint8_t)79, (uint8_t)154, (uint8_t)44, (uint8_t)62, (uint8_t)139, (uint8_t)197, (uint8_t)212, (uint8_t)42, (uint8_t)144, (uint8_t)12, (uint8_t)245, (uint8_t)25, (uint8_t)252, (uint8_t)246, (uint8_t)213, (uint8_t)163, (uint8_t)238, (uint8_t)210, (uint8_t)214, (uint8_t)92, (uint8_t)48, (uint8_t)87, (uint8_t)70, (uint8_t)236, (uint8_t)243, (uint8_t)82, (uint8_t)143, (uint8_t)136, (uint8_t)172, (uint8_t)129, (uint8_t)140, (uint8_t)153, (uint8_t)91, (uint8_t)196, (uint8_t)133, (uint8_t)202, (uint8_t)92, (uint8_t)209, (uint8_t)173, (uint8_t)2, (uint8_t)106, (uint8_t)222, (uint8_t)95, (uint8_t)115, (uint8_t)19, (uint8_t)15, (uint8_t)131, (uint8_t)240, (uint8_t)143, (uint8_t)171, (uint8_t)21, (uint8_t)194, (uint8_t)126, (uint8_t)68, (uint8_t)230, (uint8_t)154, (uint8_t)255, (uint8_t)180, (uint8_t)148, (uint8_t)106, (uint8_t)66, (uint8_t)221, (uint8_t)188, (uint8_t)111, (uint8_t)170, (uint8_t)139, (uint8_t)165, (uint8_t)96, (uint8_t)131, (uint8_t)9, (uint8_t)161, (uint8_t)181, (uint8_t)67, (uint8_t)33, (uint8_t)61, (uint8_t)69, (uint8_t)25, (uint8_t)142, (uint8_t)85, (uint8_t)181, (uint8_t)57, (uint8_t)65, (uint8_t)62, (uint8_t)146, (uint8_t)215, (uint8_t)171, (uint8_t)182, (uint8_t)188, (uint8_t)65, (uint8_t)73, (uint8_t)75, (uint8_t)51, (uint8_t)151, (uint8_t)1, (uint8_t)188, (uint8_t)125, (uint8_t)47, (uint8_t)51, (uint8_t)227, (uint8_t)166, (uint8_t)56, (uint8_t)92, (uint8_t)119, (uint8_t)54, (uint8_t)220, (uint8_t)246, (uint8_t)37, (uint8_t)250, (uint8_t)165, (uint8_t)203, (uint8_t)120, (uint8_t)247, (uint8_t)64, (uint8_t)61, (uint8_t)244, (uint8_t)8, (uint8_t)103, (uint8_t)38, (uint8_t)24, (uint8_t)136, (uint8_t)254, (uint8_t)8, (uint8_t)153, (uint8_t)58, (uint8_t)85, (uint8_t)61, (uint8_t)181, (uint8_t)221, (uint8_t)158, (uint8_t)103, (uint8_t)198, (uint8_t)58, (uint8_t)56, (uint8_t)124, (uint8_t)59, (uint8_t)0, (uint8_t)83, (uint8_t)244, (uint8_t)35, (uint8_t)36, (uint8_t)60, (uint8_t)103, (uint8_t)42, (uint8_t)128, (uint8_t)144, (uint8_t)16, (uint8_t)243, (uint8_t)224, (uint8_t)35, (uint8_t)73, (uint8_t)144, (uint8_t)162, (uint8_t)151, (uint8_t)184, (uint8_t)217, (uint8_t)219, (uint8_t)2, (uint8_t)82, (uint8_t)8, (uint8_t)133, (uint8_t)158, (uint8_t)176, (uint8_t)235, (uint8_t)71, (uint8_t)236, (uint8_t)74, (uint8_t)117, (uint8_t)100, (uint8_t)192, (uint8_t)237, (uint8_t)70, (uint8_t)167, (uint8_t)51};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1966916979L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -15958, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)851, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)27288, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -3, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)10099, PH.base.pack) ;
    p234_latitude_SET((int32_t) -2193127, PH.base.pack) ;
    p234_longitude_SET((int32_t) -528018342, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)16349, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -13730, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)30, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t) -32, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)25, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)57225, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)5929917709757305741L, PH.base.pack) ;
    p241_vibration_x_SET((float) -6.2322085E37F, PH.base.pack) ;
    p241_vibration_y_SET((float) -7.9694374E37F, PH.base.pack) ;
    p241_vibration_z_SET((float)4.1822275E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)1526812063L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)231479529L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)3494648436L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)2009447798, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1897296663, PH.base.pack) ;
    p242_altitude_SET((int32_t)1451867133, PH.base.pack) ;
    p242_x_SET((float)2.0599063E38F, PH.base.pack) ;
    p242_y_SET((float) -1.0863524E38F, PH.base.pack) ;
    p242_z_SET((float) -2.3786632E38F, PH.base.pack) ;
    {
        float  q [] =  {2.3682052E38F, 2.3609122E38F, 8.88458E37F, -2.688284E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -3.549692E37F, PH.base.pack) ;
    p242_approach_y_SET((float) -2.808499E38F, PH.base.pack) ;
    p242_approach_z_SET((float)3.1674449E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)7401873806245784175L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p243_latitude_SET((int32_t)993366398, PH.base.pack) ;
    p243_longitude_SET((int32_t) -818786509, PH.base.pack) ;
    p243_altitude_SET((int32_t) -1231706538, PH.base.pack) ;
    p243_x_SET((float)3.8945274E37F, PH.base.pack) ;
    p243_y_SET((float)2.4744651E38F, PH.base.pack) ;
    p243_z_SET((float) -2.0162825E37F, PH.base.pack) ;
    {
        float  q [] =  {1.2918856E38F, -2.6317974E38F, 1.5781615E38F, 1.6288425E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -2.2074393E38F, PH.base.pack) ;
    p243_approach_y_SET((float)1.2425277E38F, PH.base.pack) ;
    p243_approach_z_SET((float) -3.0245174E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)7442191481268169240L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)31407, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -2002444413, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)2281453986L, PH.base.pack) ;
    p246_lat_SET((int32_t)939939866, PH.base.pack) ;
    p246_lon_SET((int32_t) -85750512, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t)1749817602, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)23383, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)48000, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -10115, PH.base.pack) ;
    {
        char16_t   callsign = "ffk";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)25345, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)2102395517L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)3.349017E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -2.115504E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)1.9855652E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)26426, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)201, (uint8_t)174, (uint8_t)78, (uint8_t)112, (uint8_t)29, (uint8_t)236, (uint8_t)197, (uint8_t)66, (uint8_t)235, (uint8_t)156, (uint8_t)250, (uint8_t)98, (uint8_t)191, (uint8_t)162, (uint8_t)154, (uint8_t)50, (uint8_t)28, (uint8_t)9, (uint8_t)81, (uint8_t)56, (uint8_t)171, (uint8_t)56, (uint8_t)236, (uint8_t)182, (uint8_t)252, (uint8_t)12, (uint8_t)87, (uint8_t)243, (uint8_t)188, (uint8_t)33, (uint8_t)16, (uint8_t)215, (uint8_t)217, (uint8_t)6, (uint8_t)170, (uint8_t)37, (uint8_t)143, (uint8_t)38, (uint8_t)236, (uint8_t)60, (uint8_t)241, (uint8_t)39, (uint8_t)147, (uint8_t)222, (uint8_t)153, (uint8_t)163, (uint8_t)77, (uint8_t)79, (uint8_t)70, (uint8_t)216, (uint8_t)150, (uint8_t)237, (uint8_t)157, (uint8_t)198, (uint8_t)246, (uint8_t)138, (uint8_t)46, (uint8_t)150, (uint8_t)21, (uint8_t)58, (uint8_t)161, (uint8_t)15, (uint8_t)98, (uint8_t)163, (uint8_t)9, (uint8_t)63, (uint8_t)126, (uint8_t)120, (uint8_t)16, (uint8_t)181, (uint8_t)241, (uint8_t)220, (uint8_t)183, (uint8_t)102, (uint8_t)64, (uint8_t)142, (uint8_t)73, (uint8_t)147, (uint8_t)179, (uint8_t)79, (uint8_t)203, (uint8_t)72, (uint8_t)189, (uint8_t)59, (uint8_t)21, (uint8_t)145, (uint8_t)74, (uint8_t)76, (uint8_t)96, (uint8_t)137, (uint8_t)190, (uint8_t)38, (uint8_t)241, (uint8_t)0, (uint8_t)28, (uint8_t)254, (uint8_t)140, (uint8_t)70, (uint8_t)128, (uint8_t)151, (uint8_t)248, (uint8_t)23, (uint8_t)62, (uint8_t)158, (uint8_t)13, (uint8_t)28, (uint8_t)10, (uint8_t)47, (uint8_t)137, (uint8_t)73, (uint8_t)91, (uint8_t)72, (uint8_t)153, (uint8_t)87, (uint8_t)177, (uint8_t)37, (uint8_t)217, (uint8_t)162, (uint8_t)88, (uint8_t)218, (uint8_t)242, (uint8_t)133, (uint8_t)113, (uint8_t)69, (uint8_t)245, (uint8_t)238, (uint8_t)77, (uint8_t)200, (uint8_t)209, (uint8_t)65, (uint8_t)186, (uint8_t)82, (uint8_t)9, (uint8_t)87, (uint8_t)109, (uint8_t)130, (uint8_t)60, (uint8_t)189, (uint8_t)221, (uint8_t)136, (uint8_t)193, (uint8_t)73, (uint8_t)41, (uint8_t)154, (uint8_t)151, (uint8_t)89, (uint8_t)242, (uint8_t)241, (uint8_t)79, (uint8_t)236, (uint8_t)98, (uint8_t)106, (uint8_t)94, (uint8_t)103, (uint8_t)248, (uint8_t)24, (uint8_t)3, (uint8_t)69, (uint8_t)2, (uint8_t)115, (uint8_t)143, (uint8_t)8, (uint8_t)50, (uint8_t)134, (uint8_t)242, (uint8_t)61, (uint8_t)56, (uint8_t)140, (uint8_t)203, (uint8_t)133, (uint8_t)6, (uint8_t)250, (uint8_t)139, (uint8_t)207, (uint8_t)236, (uint8_t)241, (uint8_t)19, (uint8_t)52, (uint8_t)20, (uint8_t)71, (uint8_t)176, (uint8_t)62, (uint8_t)126, (uint8_t)237, (uint8_t)151, (uint8_t)46, (uint8_t)42, (uint8_t)48, (uint8_t)205, (uint8_t)180, (uint8_t)177, (uint8_t)171, (uint8_t)186, (uint8_t)114, (uint8_t)113, (uint8_t)251, (uint8_t)48, (uint8_t)159, (uint8_t)209, (uint8_t)125, (uint8_t)237, (uint8_t)173, (uint8_t)152, (uint8_t)251, (uint8_t)254, (uint8_t)18, (uint8_t)143, (uint8_t)25, (uint8_t)64, (uint8_t)189, (uint8_t)8, (uint8_t)251, (uint8_t)127, (uint8_t)165, (uint8_t)69, (uint8_t)5, (uint8_t)239, (uint8_t)227, (uint8_t)104, (uint8_t)217, (uint8_t)111, (uint8_t)86, (uint8_t)35, (uint8_t)97, (uint8_t)248, (uint8_t)251, (uint8_t)170, (uint8_t)163, (uint8_t)153, (uint8_t)159, (uint8_t)90, (uint8_t)153, (uint8_t)92, (uint8_t)234, (uint8_t)226, (uint8_t)130, (uint8_t)189, (uint8_t)181, (uint8_t)221, (uint8_t)62, (uint8_t)230, (uint8_t)184, (uint8_t)203, (uint8_t)195, (uint8_t)129, (uint8_t)242, (uint8_t)150, (uint8_t)142, (uint8_t)77};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)39938, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t) -99, (int8_t) -93, (int8_t)13, (int8_t) -91, (int8_t)125, (int8_t) -71, (int8_t)58, (int8_t) -24, (int8_t) -36, (int8_t)83, (int8_t) -6, (int8_t) -34, (int8_t)90, (int8_t) -91, (int8_t)102, (int8_t) -70, (int8_t) -91, (int8_t)105, (int8_t) -102, (int8_t)43, (int8_t)36, (int8_t)18, (int8_t) -8, (int8_t) -12, (int8_t)102, (int8_t) -108, (int8_t)0, (int8_t)84, (int8_t)72, (int8_t)60, (int8_t)38, (int8_t) -45};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "pgpS";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)4668349394847648062L, PH.base.pack) ;
    p250_x_SET((float)3.0690373E38F, PH.base.pack) ;
    p250_y_SET((float)3.0284842E38F, PH.base.pack) ;
    p250_z_SET((float)2.1013945E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3575214406L, PH.base.pack) ;
    {
        char16_t   name = "czfljr";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -2.6650725E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)1025777516L, PH.base.pack) ;
    {
        char16_t   name = "dpdpkvJqow";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -142974680, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, PH.base.pack) ;
    {
        char16_t   text = "smtbxoxxwnvcidsDxmeenEqrjdvwdzswbN";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)4067948571L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p254_value_SET((float) -2.1978782E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)140, (uint8_t)100, (uint8_t)24, (uint8_t)99, (uint8_t)8, (uint8_t)75, (uint8_t)151, (uint8_t)88, (uint8_t)195, (uint8_t)201, (uint8_t)89, (uint8_t)74, (uint8_t)239, (uint8_t)6, (uint8_t)95, (uint8_t)220, (uint8_t)140, (uint8_t)209, (uint8_t)35, (uint8_t)18, (uint8_t)183, (uint8_t)210, (uint8_t)69, (uint8_t)182, (uint8_t)92, (uint8_t)59, (uint8_t)169, (uint8_t)101, (uint8_t)211, (uint8_t)213, (uint8_t)254, (uint8_t)18};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)6094068557551727727L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)982469760L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)2898249555L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    {
        char16_t   tune = "uwjVuhhpgtvLaupibfjVDEvrc";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)2800396374L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)7, (uint8_t)62, (uint8_t)214, (uint8_t)27, (uint8_t)37, (uint8_t)153, (uint8_t)74, (uint8_t)54, (uint8_t)157, (uint8_t)15, (uint8_t)171, (uint8_t)184, (uint8_t)143, (uint8_t)174, (uint8_t)62, (uint8_t)249, (uint8_t)115, (uint8_t)68, (uint8_t)85, (uint8_t)145, (uint8_t)100, (uint8_t)192, (uint8_t)64, (uint8_t)226, (uint8_t)204, (uint8_t)74, (uint8_t)13, (uint8_t)27, (uint8_t)211, (uint8_t)228, (uint8_t)87, (uint8_t)159};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)97, (uint8_t)5, (uint8_t)168, (uint8_t)79, (uint8_t)18, (uint8_t)6, (uint8_t)189, (uint8_t)24, (uint8_t)127, (uint8_t)249, (uint8_t)175, (uint8_t)17, (uint8_t)219, (uint8_t)234, (uint8_t)7, (uint8_t)200, (uint8_t)79, (uint8_t)114, (uint8_t)175, (uint8_t)232, (uint8_t)136, (uint8_t)81, (uint8_t)165, (uint8_t)184, (uint8_t)203, (uint8_t)251, (uint8_t)175, (uint8_t)95, (uint8_t)169, (uint8_t)173, (uint8_t)218, (uint8_t)181};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)3590937264L, PH.base.pack) ;
    p259_focal_length_SET((float) -2.9881636E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -3.9408565E37F, PH.base.pack) ;
    p259_sensor_size_v_SET((float)1.0903081E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)12859, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)13060, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)5629, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "ecRqufjQcfgdsijexvlylqjsvokuiyoenkqItqlnqponiryRrxvldnmiLknvErhqwkshemqqfictbndfixbakUudvgpkyzdfegrciVLveKitqFmejxnkcxcxpyysbjuiisfmEulpcW";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)1166523207L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)4144797256L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p261_total_capacity_SET((float) -2.3152945E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)8.002347E37F, PH.base.pack) ;
    p261_available_capacity_SET((float)2.8848667E38F, PH.base.pack) ;
    p261_read_speed_SET((float) -3.2413481E38F, PH.base.pack) ;
    p261_write_speed_SET((float)2.1879124E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)4221557928L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    p262_image_interval_SET((float)2.3419818E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)3682293796L, PH.base.pack) ;
    p262_available_capacity_SET((float) -1.5755784E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)2029386775L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)4152189040866237212L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p263_lat_SET((int32_t) -1342429706, PH.base.pack) ;
    p263_lon_SET((int32_t)1051024409, PH.base.pack) ;
    p263_alt_SET((int32_t) -961369222, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)1213249999, PH.base.pack) ;
    {
        float  q [] =  {1.3677586E38F, 3.384075E38F, 2.003851E38F, 2.5519273E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1408545671, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)48, PH.base.pack) ;
    {
        char16_t   file_url = "cXfjggxtykqicjubyabnisAagpEhwfvfUwzilfalFuxtxzzYvdhcgzVbesjMyflfihnuenMAuhvgnjosvkwGQBUxmwirnturhvkViliDcrokWqhvjsxgktrqbnWzatEHubfzwhixcnwuttuyawsgWegAvghroijKcjzvrtfnqjQc";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}


#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_ANTENNA_TRACKER, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1514906527L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)36816, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)29787, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -5865, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)92, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)3104, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)30803, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)38816, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)41566, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)62161, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)3033, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)8429103993167416070L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)3324769572L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)3533679038L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)13243, PH.base.pack) ;
    p3_x_SET((float)5.4120476E37F, PH.base.pack) ;
    p3_y_SET((float) -1.1488181E38F, PH.base.pack) ;
    p3_z_SET((float) -3.3245731E38F, PH.base.pack) ;
    p3_vx_SET((float) -2.8239803E38F, PH.base.pack) ;
    p3_vy_SET((float)2.8119898E37F, PH.base.pack) ;
    p3_vz_SET((float) -3.2094289E38F, PH.base.pack) ;
    p3_afx_SET((float)1.6222458E38F, PH.base.pack) ;
    p3_afy_SET((float) -7.8340873E37F, PH.base.pack) ;
    p3_afz_SET((float) -3.3283651E38F, PH.base.pack) ;
    p3_yaw_SET((float) -1.5399871E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -1.9208438E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)6126161634074996230L, PH.base.pack) ;
    p4_seq_SET((uint32_t)1808872925L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    {
        char16_t   passkey = "xjb";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "Wlxak";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)1438353806L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    {
        char16_t   param_id = "dxwCldLpmdphym";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)8895, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "xHlhhgrx";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -3.1989526E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)10694, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)63630, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    {
        char16_t   param_id = "mtsKwazwvRa";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -3.17777E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)3522691579905962332L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -25191939, PH.base.pack) ;
    p24_lon_SET((int32_t)634106789, PH.base.pack) ;
    p24_alt_SET((int32_t)451660565, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)56299, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)7210, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)11561, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)65236, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)178232567, &PH) ;
    p24_h_acc_SET((uint32_t)3849025890L, &PH) ;
    p24_v_acc_SET((uint32_t)3899228355L, &PH) ;
    p24_vel_acc_SET((uint32_t)4028927851L, &PH) ;
    p24_hdg_acc_SET((uint32_t)205543962L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)28, (uint8_t)54, (uint8_t)80, (uint8_t)82, (uint8_t)171, (uint8_t)217, (uint8_t)211, (uint8_t)255, (uint8_t)193, (uint8_t)10, (uint8_t)48, (uint8_t)115, (uint8_t)197, (uint8_t)61, (uint8_t)131, (uint8_t)254, (uint8_t)36, (uint8_t)106, (uint8_t)173, (uint8_t)128};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)121, (uint8_t)42, (uint8_t)42, (uint8_t)218, (uint8_t)97, (uint8_t)61, (uint8_t)53, (uint8_t)52, (uint8_t)183, (uint8_t)223, (uint8_t)141, (uint8_t)97, (uint8_t)218, (uint8_t)196, (uint8_t)4, (uint8_t)134, (uint8_t)102, (uint8_t)56, (uint8_t)121, (uint8_t)245};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)191, (uint8_t)139, (uint8_t)237, (uint8_t)138, (uint8_t)160, (uint8_t)215, (uint8_t)82, (uint8_t)173, (uint8_t)173, (uint8_t)171, (uint8_t)119, (uint8_t)21, (uint8_t)44, (uint8_t)231, (uint8_t)43, (uint8_t)219, (uint8_t)31, (uint8_t)194, (uint8_t)61, (uint8_t)49};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)138, (uint8_t)233, (uint8_t)141, (uint8_t)175, (uint8_t)186, (uint8_t)51, (uint8_t)231, (uint8_t)33, (uint8_t)104, (uint8_t)191, (uint8_t)131, (uint8_t)132, (uint8_t)119, (uint8_t)66, (uint8_t)151, (uint8_t)51, (uint8_t)119, (uint8_t)145, (uint8_t)27, (uint8_t)242};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)192, (uint8_t)154, (uint8_t)33, (uint8_t)32, (uint8_t)115, (uint8_t)128, (uint8_t)61, (uint8_t)45, (uint8_t)55, (uint8_t)60, (uint8_t)229, (uint8_t)253, (uint8_t)249, (uint8_t)110, (uint8_t)162, (uint8_t)86, (uint8_t)199, (uint8_t)32, (uint8_t)6, (uint8_t)205};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3451443455L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)28219, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)21386, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)12410, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -32628, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -30730, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -28996, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -26820, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)25698, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -24847, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)2716862560303204871L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -24187, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -7075, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -10308, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -24109, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -16484, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)8452, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -32317, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -14606, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)16153, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)8624962109097190586L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -5753, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -32725, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -22849, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)22771, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)3424217503L, PH.base.pack) ;
    p29_press_abs_SET((float)9.686366E37F, PH.base.pack) ;
    p29_press_diff_SET((float)3.287669E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -1411, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)4275661692L, PH.base.pack) ;
    p30_roll_SET((float) -1.2741388E38F, PH.base.pack) ;
    p30_pitch_SET((float) -1.796411E38F, PH.base.pack) ;
    p30_yaw_SET((float) -2.1307223E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)2.789648E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -8.187507E37F, PH.base.pack) ;
    p30_yawspeed_SET((float)9.337939E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1504564668L, PH.base.pack) ;
    p31_q1_SET((float)8.037524E37F, PH.base.pack) ;
    p31_q2_SET((float)8.4557523E37F, PH.base.pack) ;
    p31_q3_SET((float)1.6317869E38F, PH.base.pack) ;
    p31_q4_SET((float)7.0690084E37F, PH.base.pack) ;
    p31_rollspeed_SET((float)1.9970991E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -2.4429138E38F, PH.base.pack) ;
    p31_yawspeed_SET((float) -1.6241723E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1362652362L, PH.base.pack) ;
    p32_x_SET((float)2.20818E38F, PH.base.pack) ;
    p32_y_SET((float) -1.0085008E38F, PH.base.pack) ;
    p32_z_SET((float)6.499234E37F, PH.base.pack) ;
    p32_vx_SET((float)2.0541266E38F, PH.base.pack) ;
    p32_vy_SET((float)1.6637312E38F, PH.base.pack) ;
    p32_vz_SET((float) -1.3712722E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1998294198L, PH.base.pack) ;
    p33_lat_SET((int32_t) -19438296, PH.base.pack) ;
    p33_lon_SET((int32_t)2031143147, PH.base.pack) ;
    p33_alt_SET((int32_t) -1697524838, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)2079014801, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)26776, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -12753, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -6513, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)39045, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3310696945L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)15905, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -17607, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)5861, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)12716, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -22794, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -14231, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)25095, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)16162, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3087974686L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)55049, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)2548, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)9086, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)16883, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)9165, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)54057, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)8770, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)10323, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)1010495970L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)27000, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)38559, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)1698, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)12434, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)9989, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)35737, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)21953, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)44415, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)38886, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)45488, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)2304, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)43747, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)20415, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)23355, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)10764, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)33045, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)13921, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)21075, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)6192, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)6994, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)20585, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_REVERSE, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p39_param1_SET((float)2.7755682E38F, PH.base.pack) ;
    p39_param2_SET((float)2.0335024E38F, PH.base.pack) ;
    p39_param3_SET((float) -1.916022E38F, PH.base.pack) ;
    p39_param4_SET((float)2.3360446E38F, PH.base.pack) ;
    p39_x_SET((float) -2.8017238E38F, PH.base.pack) ;
    p39_y_SET((float) -3.0421627E38F, PH.base.pack) ;
    p39_z_SET((float)2.602435E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)8641, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)45137, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)61145, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)39022, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)26826, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p48_latitude_SET((int32_t) -1838884217, PH.base.pack) ;
    p48_longitude_SET((int32_t)2061799803, PH.base.pack) ;
    p48_altitude_SET((int32_t) -1353696181, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)6630973852172325382L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)2036135120, PH.base.pack) ;
    p49_longitude_SET((int32_t)603087357, PH.base.pack) ;
    p49_altitude_SET((int32_t)1290401154, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)348011991075201288L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    {
        char16_t   param_id = "qkjrsgem";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)671, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p50_param_value0_SET((float) -3.797767E36F, PH.base.pack) ;
    p50_scale_SET((float) -1.3776376E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)2.7922428E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -2.8900227E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)18119, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p54_p1x_SET((float) -4.2366835E37F, PH.base.pack) ;
    p54_p1y_SET((float)4.7557266E37F, PH.base.pack) ;
    p54_p1z_SET((float) -5.1956282E36F, PH.base.pack) ;
    p54_p2x_SET((float)3.5082279E37F, PH.base.pack) ;
    p54_p2y_SET((float)2.5755508E38F, PH.base.pack) ;
    p54_p2z_SET((float) -1.178034E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p55_p1x_SET((float) -7.8241925E37F, PH.base.pack) ;
    p55_p1y_SET((float) -3.0130603E38F, PH.base.pack) ;
    p55_p1z_SET((float)1.5596257E38F, PH.base.pack) ;
    p55_p2x_SET((float) -3.201525E38F, PH.base.pack) ;
    p55_p2y_SET((float) -2.3662013E38F, PH.base.pack) ;
    p55_p2z_SET((float)1.3295042E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)8921737928766096334L, PH.base.pack) ;
    {
        float  q [] =  {-2.2907797E38F, 1.3257251E38F, -9.869742E37F, 1.3285613E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)2.1901944E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)5.7483477E37F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.602354E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-3.022778E38F, 9.749621E37F, -2.2921486E38F, 2.125588E38F, 2.5878543E38F, -1.3428389E38F, 1.1497935E38F, -2.3477443E38F, -3.0215503E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -3.1081446E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -1.4651521E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -31225, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)12781, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)42604, PH.base.pack) ;
    p62_alt_error_SET((float)1.1247444E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -1.2933421E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -4.826031E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)7297338461029136902L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t) -515877070, PH.base.pack) ;
    p63_lon_SET((int32_t)39375635, PH.base.pack) ;
    p63_alt_SET((int32_t) -502313274, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1949814984, PH.base.pack) ;
    p63_vx_SET((float) -1.5390688E38F, PH.base.pack) ;
    p63_vy_SET((float)3.5282116E37F, PH.base.pack) ;
    p63_vz_SET((float) -2.8984582E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-3.0224133E38F, -4.9297933E37F, -1.6770843E38F, -3.0469505E38F, -1.995482E38F, 1.6035094E38F, -6.7942234E37F, -3.1490493E38F, -2.6160627E38F, -1.2282506E38F, -2.6252997E38F, 2.2264545E38F, 1.3548695E38F, 9.06429E37F, 2.866521E38F, 2.4543204E38F, 1.301419E37F, -2.0068613E38F, 3.037291E38F, 2.532589E38F, 1.405047E38F, -2.6243572E38F, -2.819131E38F, -2.6823945E38F, -7.604455E37F, 3.3095094E38F, -1.6856189E38F, 1.6500242E38F, 3.3337568E38F, -1.6167583E38F, 1.2513145E38F, 2.0414476E38F, -2.7366068E38F, 1.1455597E38F, 2.0640394E38F, 7.78088E37F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)920568743540706921L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p64_x_SET((float)1.4165012E38F, PH.base.pack) ;
    p64_y_SET((float)1.1817266E38F, PH.base.pack) ;
    p64_z_SET((float) -2.8739475E38F, PH.base.pack) ;
    p64_vx_SET((float) -3.2838375E38F, PH.base.pack) ;
    p64_vy_SET((float)2.319763E38F, PH.base.pack) ;
    p64_vz_SET((float) -1.2927428E38F, PH.base.pack) ;
    p64_ax_SET((float)3.0073657E37F, PH.base.pack) ;
    p64_ay_SET((float)2.6967058E38F, PH.base.pack) ;
    p64_az_SET((float)2.067841E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-3.0365858E38F, 2.4963797E38F, -2.760781E38F, -1.6605597E38F, -1.2627984E38F, -1.2002038E38F, 2.02544E38F, 1.3259031E38F, -3.1008838E37F, 1.6933983E38F, 1.724829E38F, 1.2669311E36F, -3.1152911E38F, -2.6095087E38F, 3.347451E38F, -1.3742071E38F, -2.8170486E38F, -1.0897753E38F, 7.733219E37F, 4.8576214E37F, -2.499952E37F, -2.6664185E38F, 3.274426E38F, 2.2632293E38F, 1.6539272E38F, 7.3980713E37F, 1.4140147E38F, -1.7374764E38F, 3.0799395E38F, -7.2971886E37F, -2.4682348E38F, 2.858492E38F, 3.7208156E37F, 2.6687217E38F, -1.830927E38F, -9.345119E37F, -2.2194578E38F, 1.2087525E37F, -1.5486641E38F, 1.5695658E38F, 5.183058E37F, -2.2221154E38F, 1.7384682E38F, 2.3549888E38F, 6.086003E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)4107348007L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)1087, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)20362, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)51547, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)13142, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)37213, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)52717, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)64249, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)47714, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)3371, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)42216, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)47163, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)25734, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)44578, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)35408, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)30839, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)20072, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)39640, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)32098, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)51382, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)5755, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -26166, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)9653, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)13503, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)5335, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)16759, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)64881, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)11186, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)64819, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)36113, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)22463, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)20834, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)37039, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)64101, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)5445, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p73_param1_SET((float) -1.3683516E38F, PH.base.pack) ;
    p73_param2_SET((float)3.3286458E38F, PH.base.pack) ;
    p73_param3_SET((float)2.0582902E38F, PH.base.pack) ;
    p73_param4_SET((float) -4.1577228E37F, PH.base.pack) ;
    p73_x_SET((int32_t) -1118821381, PH.base.pack) ;
    p73_y_SET((int32_t)1655839244, PH.base.pack) ;
    p73_z_SET((float) -1.4090742E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -2.9306632E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.5527952E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -4438, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)14410, PH.base.pack) ;
    p74_alt_SET((float) -1.4695403E38F, PH.base.pack) ;
    p74_climb_SET((float) -1.2670438E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_DO_JUMP, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p75_param1_SET((float) -4.1318156E37F, PH.base.pack) ;
    p75_param2_SET((float)2.7479387E38F, PH.base.pack) ;
    p75_param3_SET((float)1.0027284E38F, PH.base.pack) ;
    p75_param4_SET((float) -1.7239553E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1553927611, PH.base.pack) ;
    p75_y_SET((int32_t) -646426988, PH.base.pack) ;
    p75_z_SET((float)3.0903202E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p76_param1_SET((float)2.6452073E37F, PH.base.pack) ;
    p76_param2_SET((float)1.2848654E38F, PH.base.pack) ;
    p76_param3_SET((float)2.9971625E38F, PH.base.pack) ;
    p76_param4_SET((float) -8.232114E37F, PH.base.pack) ;
    p76_param5_SET((float) -2.6713118E38F, PH.base.pack) ;
    p76_param6_SET((float)2.583241E38F, PH.base.pack) ;
    p76_param7_SET((float)3.0206915E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)119, &PH) ;
    p77_result_param2_SET((int32_t) -1968998502, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)39, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)91, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)1015452278L, PH.base.pack) ;
    p81_roll_SET((float)4.0727664E37F, PH.base.pack) ;
    p81_pitch_SET((float) -1.2163296E38F, PH.base.pack) ;
    p81_yaw_SET((float) -2.1944287E38F, PH.base.pack) ;
    p81_thrust_SET((float) -1.0849174E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2963422735L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    {
        float  q [] =  {3.233137E38F, -3.1312073E38F, -2.805588E38F, 2.8554148E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)3.8586103E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -2.7300413E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)1.0815758E37F, PH.base.pack) ;
    p82_thrust_SET((float) -2.5570218E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2368195431L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    {
        float  q [] =  {-2.185458E38F, -1.6050738E38F, -3.3177501E38F, 2.7698137E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -4.782848E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -8.709164E37F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -1.1034813E38F, PH.base.pack) ;
    p83_thrust_SET((float)3.2852995E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1800676185L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)22448, PH.base.pack) ;
    p84_x_SET((float)2.2137447E38F, PH.base.pack) ;
    p84_y_SET((float) -1.7689482E38F, PH.base.pack) ;
    p84_z_SET((float)2.4578832E38F, PH.base.pack) ;
    p84_vx_SET((float)2.5016056E38F, PH.base.pack) ;
    p84_vy_SET((float) -1.6121762E38F, PH.base.pack) ;
    p84_vz_SET((float)3.2405522E38F, PH.base.pack) ;
    p84_afx_SET((float)6.452369E37F, PH.base.pack) ;
    p84_afy_SET((float)1.2110784E38F, PH.base.pack) ;
    p84_afz_SET((float)5.3024876E37F, PH.base.pack) ;
    p84_yaw_SET((float) -9.878822E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)3.2452106E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)3947608318L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)32069, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -588638347, PH.base.pack) ;
    p86_lon_int_SET((int32_t)79232102, PH.base.pack) ;
    p86_alt_SET((float)2.693763E38F, PH.base.pack) ;
    p86_vx_SET((float) -8.468536E37F, PH.base.pack) ;
    p86_vy_SET((float)9.686313E37F, PH.base.pack) ;
    p86_vz_SET((float) -1.5106777E37F, PH.base.pack) ;
    p86_afx_SET((float) -2.41939E38F, PH.base.pack) ;
    p86_afy_SET((float) -1.1722259E38F, PH.base.pack) ;
    p86_afz_SET((float)4.844382E37F, PH.base.pack) ;
    p86_yaw_SET((float)2.3904694E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)3.3334256E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2597384020L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)41110, PH.base.pack) ;
    p87_lat_int_SET((int32_t)1311150512, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -2080400663, PH.base.pack) ;
    p87_alt_SET((float)3.0330557E38F, PH.base.pack) ;
    p87_vx_SET((float) -6.357825E37F, PH.base.pack) ;
    p87_vy_SET((float)9.146058E37F, PH.base.pack) ;
    p87_vz_SET((float) -9.166334E37F, PH.base.pack) ;
    p87_afx_SET((float)3.3499793E38F, PH.base.pack) ;
    p87_afy_SET((float)1.7378486E38F, PH.base.pack) ;
    p87_afz_SET((float)9.061006E37F, PH.base.pack) ;
    p87_yaw_SET((float)3.2306086E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -4.000939E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1618036999L, PH.base.pack) ;
    p89_x_SET((float)2.3681762E38F, PH.base.pack) ;
    p89_y_SET((float) -3.16722E38F, PH.base.pack) ;
    p89_z_SET((float)2.2615371E38F, PH.base.pack) ;
    p89_roll_SET((float) -2.2360961E38F, PH.base.pack) ;
    p89_pitch_SET((float) -3.2859781E38F, PH.base.pack) ;
    p89_yaw_SET((float) -2.0561642E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)5779350498517461331L, PH.base.pack) ;
    p90_roll_SET((float) -2.371559E38F, PH.base.pack) ;
    p90_pitch_SET((float) -9.336538E37F, PH.base.pack) ;
    p90_yaw_SET((float)3.3293638E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -4.1283947E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -2.912696E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -1.9135312E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)661040106, PH.base.pack) ;
    p90_lon_SET((int32_t) -1103888409, PH.base.pack) ;
    p90_alt_SET((int32_t) -1423854311, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -26503, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -28619, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -4058, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -21090, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -15868, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -23643, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)2474576821172116214L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -3.2608012E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -2.252151E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -2.7461498E38F, PH.base.pack) ;
    p91_throttle_SET((float) -2.467925E38F, PH.base.pack) ;
    p91_aux1_SET((float) -1.427065E38F, PH.base.pack) ;
    p91_aux2_SET((float) -3.3779631E38F, PH.base.pack) ;
    p91_aux3_SET((float)2.9897205E38F, PH.base.pack) ;
    p91_aux4_SET((float)3.0590825E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)8455863994019914598L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)14094, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)61194, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)19452, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)42127, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)51898, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)34055, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)36097, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)54687, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)41056, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)44074, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)24573, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)40826, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)3947536050455619826L, PH.base.pack) ;
    {
        float  controls [] =  {2.914469E38F, -2.81463E38F, -1.5954991E38F, -1.5535421E38F, -3.018136E38F, -2.938941E38F, -2.3235311E38F, -3.3985623E38F, -3.163864E38F, 1.8349395E38F, 5.4885473E37F, -2.3703557E38F, -1.0970396E38F, -1.9050383E37F, -2.5320619E38F, 1.3518614E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)5966618708415950619L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)2774316900409318435L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)12747, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)29890, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -2.3905688E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)1.02221354E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p100_ground_distance_SET((float)2.939569E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)2.2934675E38F, &PH) ;
    p100_flow_rate_y_SET((float) -3.2338327E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)2249047053209265499L, PH.base.pack) ;
    p101_x_SET((float) -7.4974505E37F, PH.base.pack) ;
    p101_y_SET((float) -3.22728E38F, PH.base.pack) ;
    p101_z_SET((float) -7.5232746E37F, PH.base.pack) ;
    p101_roll_SET((float)1.3241821E38F, PH.base.pack) ;
    p101_pitch_SET((float)1.5476584E38F, PH.base.pack) ;
    p101_yaw_SET((float)2.7966012E35F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)2085604303834300281L, PH.base.pack) ;
    p102_x_SET((float)4.1622032E37F, PH.base.pack) ;
    p102_y_SET((float)3.3061504E38F, PH.base.pack) ;
    p102_z_SET((float) -3.137554E38F, PH.base.pack) ;
    p102_roll_SET((float) -2.9249137E38F, PH.base.pack) ;
    p102_pitch_SET((float)3.3831246E38F, PH.base.pack) ;
    p102_yaw_SET((float) -1.8822569E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)382019221560040051L, PH.base.pack) ;
    p103_x_SET((float)2.592186E38F, PH.base.pack) ;
    p103_y_SET((float) -1.0869225E38F, PH.base.pack) ;
    p103_z_SET((float) -1.0922257E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)129529132955773295L, PH.base.pack) ;
    p104_x_SET((float) -9.557381E37F, PH.base.pack) ;
    p104_y_SET((float)2.627348E38F, PH.base.pack) ;
    p104_z_SET((float)1.3809048E37F, PH.base.pack) ;
    p104_roll_SET((float) -6.658233E37F, PH.base.pack) ;
    p104_pitch_SET((float) -1.8690482E38F, PH.base.pack) ;
    p104_yaw_SET((float) -2.5786311E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)5767386981300665845L, PH.base.pack) ;
    p105_xacc_SET((float)2.7884474E38F, PH.base.pack) ;
    p105_yacc_SET((float) -2.3771584E38F, PH.base.pack) ;
    p105_zacc_SET((float)3.3623785E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -2.1180302E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -9.180745E37F, PH.base.pack) ;
    p105_zgyro_SET((float) -2.5900556E38F, PH.base.pack) ;
    p105_xmag_SET((float) -2.414917E38F, PH.base.pack) ;
    p105_ymag_SET((float)1.9486153E38F, PH.base.pack) ;
    p105_zmag_SET((float) -2.7267423E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float)7.748714E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float)2.209585E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)3.1340584E38F, PH.base.pack) ;
    p105_temperature_SET((float) -1.5553445E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)46657, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)7945145354102301450L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)4176918124L, PH.base.pack) ;
    p106_integrated_x_SET((float)2.4759226E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)3.383756E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.8921342E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -2.7336684E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -1.88312E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -1326, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)3271914001L, PH.base.pack) ;
    p106_distance_SET((float)2.3271418E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)559150833039827731L, PH.base.pack) ;
    p107_xacc_SET((float)1.798548E38F, PH.base.pack) ;
    p107_yacc_SET((float) -2.9516269E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.872431E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.8745862E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -1.7715886E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -1.0018478E38F, PH.base.pack) ;
    p107_xmag_SET((float) -4.8759974E36F, PH.base.pack) ;
    p107_ymag_SET((float)3.152461E38F, PH.base.pack) ;
    p107_zmag_SET((float)1.339836E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -5.364193E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -2.1442233E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)3.006195E38F, PH.base.pack) ;
    p107_temperature_SET((float) -1.4341323E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2444495859L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)2.4159786E38F, PH.base.pack) ;
    p108_q2_SET((float)6.5443476E37F, PH.base.pack) ;
    p108_q3_SET((float) -2.3468734E38F, PH.base.pack) ;
    p108_q4_SET((float) -1.8471742E38F, PH.base.pack) ;
    p108_roll_SET((float) -2.7046528E38F, PH.base.pack) ;
    p108_pitch_SET((float) -4.5863573E37F, PH.base.pack) ;
    p108_yaw_SET((float)1.5865875E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.7832256E38F, PH.base.pack) ;
    p108_yacc_SET((float) -2.1934037E38F, PH.base.pack) ;
    p108_zacc_SET((float)1.0288795E38F, PH.base.pack) ;
    p108_xgyro_SET((float)2.6391873E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -1.2602327E37F, PH.base.pack) ;
    p108_zgyro_SET((float)1.3663084E38F, PH.base.pack) ;
    p108_lat_SET((float)2.3860912E38F, PH.base.pack) ;
    p108_lon_SET((float)1.452196E38F, PH.base.pack) ;
    p108_alt_SET((float) -2.2659646E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -3.205598E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -2.7120334E38F, PH.base.pack) ;
    p108_vn_SET((float)6.1972005E37F, PH.base.pack) ;
    p108_ve_SET((float) -1.5181178E38F, PH.base.pack) ;
    p108_vd_SET((float) -1.797904E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)5086, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)9059, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)144, (uint8_t)110, (uint8_t)191, (uint8_t)72, (uint8_t)188, (uint8_t)130, (uint8_t)9, (uint8_t)213, (uint8_t)80, (uint8_t)103, (uint8_t)125, (uint8_t)67, (uint8_t)110, (uint8_t)77, (uint8_t)166, (uint8_t)142, (uint8_t)185, (uint8_t)228, (uint8_t)39, (uint8_t)52, (uint8_t)18, (uint8_t)159, (uint8_t)102, (uint8_t)8, (uint8_t)61, (uint8_t)20, (uint8_t)79, (uint8_t)107, (uint8_t)128, (uint8_t)1, (uint8_t)130, (uint8_t)64, (uint8_t)10, (uint8_t)200, (uint8_t)12, (uint8_t)72, (uint8_t)33, (uint8_t)81, (uint8_t)98, (uint8_t)143, (uint8_t)143, (uint8_t)158, (uint8_t)156, (uint8_t)43, (uint8_t)102, (uint8_t)87, (uint8_t)62, (uint8_t)115, (uint8_t)41, (uint8_t)111, (uint8_t)124, (uint8_t)11, (uint8_t)149, (uint8_t)210, (uint8_t)98, (uint8_t)3, (uint8_t)129, (uint8_t)252, (uint8_t)46, (uint8_t)174, (uint8_t)5, (uint8_t)36, (uint8_t)76, (uint8_t)237, (uint8_t)1, (uint8_t)14, (uint8_t)153, (uint8_t)55, (uint8_t)62, (uint8_t)187, (uint8_t)143, (uint8_t)239, (uint8_t)61, (uint8_t)79, (uint8_t)77, (uint8_t)218, (uint8_t)185, (uint8_t)226, (uint8_t)245, (uint8_t)158, (uint8_t)164, (uint8_t)234, (uint8_t)241, (uint8_t)6, (uint8_t)55, (uint8_t)29, (uint8_t)80, (uint8_t)108, (uint8_t)43, (uint8_t)135, (uint8_t)107, (uint8_t)5, (uint8_t)9, (uint8_t)96, (uint8_t)12, (uint8_t)253, (uint8_t)79, (uint8_t)27, (uint8_t)81, (uint8_t)85, (uint8_t)39, (uint8_t)159, (uint8_t)78, (uint8_t)196, (uint8_t)157, (uint8_t)89, (uint8_t)168, (uint8_t)147, (uint8_t)152, (uint8_t)43, (uint8_t)175, (uint8_t)206, (uint8_t)190, (uint8_t)150, (uint8_t)115, (uint8_t)144, (uint8_t)3, (uint8_t)122, (uint8_t)79, (uint8_t)189, (uint8_t)233, (uint8_t)85, (uint8_t)59, (uint8_t)5, (uint8_t)125, (uint8_t)169, (uint8_t)165, (uint8_t)74, (uint8_t)170, (uint8_t)110, (uint8_t)151, (uint8_t)248, (uint8_t)180, (uint8_t)76, (uint8_t)36, (uint8_t)150, (uint8_t)210, (uint8_t)207, (uint8_t)84, (uint8_t)235, (uint8_t)48, (uint8_t)195, (uint8_t)103, (uint8_t)162, (uint8_t)76, (uint8_t)11, (uint8_t)228, (uint8_t)14, (uint8_t)16, (uint8_t)125, (uint8_t)54, (uint8_t)124, (uint8_t)23, (uint8_t)1, (uint8_t)152, (uint8_t)108, (uint8_t)156, (uint8_t)34, (uint8_t)236, (uint8_t)205, (uint8_t)37, (uint8_t)12, (uint8_t)141, (uint8_t)188, (uint8_t)193, (uint8_t)180, (uint8_t)3, (uint8_t)144, (uint8_t)24, (uint8_t)64, (uint8_t)221, (uint8_t)3, (uint8_t)247, (uint8_t)250, (uint8_t)95, (uint8_t)182, (uint8_t)110, (uint8_t)190, (uint8_t)248, (uint8_t)215, (uint8_t)113, (uint8_t)220, (uint8_t)202, (uint8_t)47, (uint8_t)33, (uint8_t)116, (uint8_t)149, (uint8_t)168, (uint8_t)10, (uint8_t)131, (uint8_t)69, (uint8_t)183, (uint8_t)50, (uint8_t)141, (uint8_t)71, (uint8_t)147, (uint8_t)159, (uint8_t)85, (uint8_t)253, (uint8_t)176, (uint8_t)202, (uint8_t)16, (uint8_t)250, (uint8_t)239, (uint8_t)167, (uint8_t)25, (uint8_t)17, (uint8_t)99, (uint8_t)24, (uint8_t)62, (uint8_t)18, (uint8_t)142, (uint8_t)196, (uint8_t)14, (uint8_t)115, (uint8_t)193, (uint8_t)124, (uint8_t)110, (uint8_t)51, (uint8_t)99, (uint8_t)12, (uint8_t)173, (uint8_t)85, (uint8_t)116, (uint8_t)66, (uint8_t)167, (uint8_t)128, (uint8_t)114, (uint8_t)133, (uint8_t)154, (uint8_t)96, (uint8_t)242, (uint8_t)230, (uint8_t)131, (uint8_t)203, (uint8_t)38, (uint8_t)39, (uint8_t)64, (uint8_t)180, (uint8_t)163, (uint8_t)14, (uint8_t)106, (uint8_t)127, (uint8_t)104, (uint8_t)132, (uint8_t)205, (uint8_t)49, (uint8_t)173, (uint8_t)211, (uint8_t)241, (uint8_t)29};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -1649629917488247855L, PH.base.pack) ;
    p111_ts1_SET((int64_t)5437779542326692501L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)1140798959706390600L, PH.base.pack) ;
    p112_seq_SET((uint32_t)1016993695L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)5559852237016954257L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p113_lat_SET((int32_t)1578020320, PH.base.pack) ;
    p113_lon_SET((int32_t) -1218740336, PH.base.pack) ;
    p113_alt_SET((int32_t)74621501, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)52592, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)17674, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)14321, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)26444, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -20317, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -11449, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)38173, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)7124163618885376603L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)1201375207L, PH.base.pack) ;
    p114_integrated_x_SET((float) -3.1478533E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.2746909E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -7.379115E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)9.587907E37F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)3.271125E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -12330, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)168480750L, PH.base.pack) ;
    p114_distance_SET((float)1.861101E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)8588148332528644288L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-3.1212018E38F, -2.1406357E38F, 2.691836E38F, -1.4984307E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -2.845373E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -2.2841025E37F, PH.base.pack) ;
    p115_yawspeed_SET((float)1.5564492E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -571923508, PH.base.pack) ;
    p115_lon_SET((int32_t) -907314746, PH.base.pack) ;
    p115_alt_SET((int32_t) -1766141513, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)28164, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)29651, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)13946, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)18116, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)15274, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)21941, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)4127, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)31381, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)1704658956L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -27850, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)24622, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)28141, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -31148, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -8311, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)16390, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -26390, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)18481, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -15650, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)960, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)1225, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)62032, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)15698, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)5622, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)1485620198L, PH.base.pack) ;
    p118_size_SET((uint32_t)4062345251L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)62837, PH.base.pack) ;
    p119_ofs_SET((uint32_t)2941285201L, PH.base.pack) ;
    p119_count_SET((uint32_t)1513524570L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)52973, PH.base.pack) ;
    p120_ofs_SET((uint32_t)1060778879L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)161, (uint8_t)153, (uint8_t)250, (uint8_t)9, (uint8_t)232, (uint8_t)110, (uint8_t)115, (uint8_t)76, (uint8_t)34, (uint8_t)9, (uint8_t)118, (uint8_t)44, (uint8_t)88, (uint8_t)2, (uint8_t)5, (uint8_t)203, (uint8_t)165, (uint8_t)198, (uint8_t)233, (uint8_t)47, (uint8_t)187, (uint8_t)195, (uint8_t)29, (uint8_t)73, (uint8_t)56, (uint8_t)116, (uint8_t)123, (uint8_t)244, (uint8_t)91, (uint8_t)226, (uint8_t)214, (uint8_t)4, (uint8_t)142, (uint8_t)152, (uint8_t)6, (uint8_t)235, (uint8_t)84, (uint8_t)189, (uint8_t)128, (uint8_t)154, (uint8_t)183, (uint8_t)223, (uint8_t)250, (uint8_t)210, (uint8_t)111, (uint8_t)126, (uint8_t)2, (uint8_t)175, (uint8_t)135, (uint8_t)190, (uint8_t)148, (uint8_t)10, (uint8_t)149, (uint8_t)125, (uint8_t)21, (uint8_t)126, (uint8_t)129, (uint8_t)174, (uint8_t)225, (uint8_t)86, (uint8_t)249, (uint8_t)156, (uint8_t)86, (uint8_t)187, (uint8_t)133, (uint8_t)131, (uint8_t)116, (uint8_t)220, (uint8_t)71, (uint8_t)93, (uint8_t)82, (uint8_t)184, (uint8_t)31, (uint8_t)95, (uint8_t)198, (uint8_t)74, (uint8_t)242, (uint8_t)31, (uint8_t)75, (uint8_t)63, (uint8_t)74, (uint8_t)22, (uint8_t)60, (uint8_t)5, (uint8_t)83, (uint8_t)86, (uint8_t)215, (uint8_t)173, (uint8_t)84, (uint8_t)210};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)11, (uint8_t)88, (uint8_t)37, (uint8_t)88, (uint8_t)59, (uint8_t)193, (uint8_t)192, (uint8_t)141, (uint8_t)61, (uint8_t)94, (uint8_t)149, (uint8_t)169, (uint8_t)91, (uint8_t)153, (uint8_t)17, (uint8_t)192, (uint8_t)190, (uint8_t)76, (uint8_t)201, (uint8_t)171, (uint8_t)44, (uint8_t)115, (uint8_t)55, (uint8_t)198, (uint8_t)231, (uint8_t)71, (uint8_t)92, (uint8_t)255, (uint8_t)179, (uint8_t)182, (uint8_t)170, (uint8_t)36, (uint8_t)93, (uint8_t)244, (uint8_t)194, (uint8_t)232, (uint8_t)14, (uint8_t)248, (uint8_t)158, (uint8_t)133, (uint8_t)16, (uint8_t)96, (uint8_t)19, (uint8_t)131, (uint8_t)9, (uint8_t)182, (uint8_t)194, (uint8_t)1, (uint8_t)33, (uint8_t)254, (uint8_t)154, (uint8_t)231, (uint8_t)159, (uint8_t)195, (uint8_t)62, (uint8_t)243, (uint8_t)193, (uint8_t)43, (uint8_t)208, (uint8_t)1, (uint8_t)82, (uint8_t)125, (uint8_t)81, (uint8_t)136, (uint8_t)221, (uint8_t)34, (uint8_t)173, (uint8_t)187, (uint8_t)226, (uint8_t)111, (uint8_t)254, (uint8_t)112, (uint8_t)230, (uint8_t)121, (uint8_t)28, (uint8_t)37, (uint8_t)47, (uint8_t)195, (uint8_t)94, (uint8_t)197, (uint8_t)117, (uint8_t)220, (uint8_t)150, (uint8_t)14, (uint8_t)107, (uint8_t)51, (uint8_t)138, (uint8_t)217, (uint8_t)108, (uint8_t)49, (uint8_t)123, (uint8_t)74, (uint8_t)166, (uint8_t)0, (uint8_t)143, (uint8_t)47, (uint8_t)55, (uint8_t)224, (uint8_t)208, (uint8_t)68, (uint8_t)130, (uint8_t)139, (uint8_t)145, (uint8_t)229, (uint8_t)254, (uint8_t)147, (uint8_t)165, (uint8_t)194, (uint8_t)73, (uint8_t)16};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)4231843368585203714L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p124_lat_SET((int32_t)1889181606, PH.base.pack) ;
    p124_lon_SET((int32_t) -816261948, PH.base.pack) ;
    p124_alt_SET((int32_t)757660667, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)15418, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)92, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)2791, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)53409, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1063400984L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)26026, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)64565, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)51878, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)2805217739L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)180, (uint8_t)67, (uint8_t)186, (uint8_t)59, (uint8_t)75, (uint8_t)93, (uint8_t)87, (uint8_t)52, (uint8_t)99, (uint8_t)251, (uint8_t)120, (uint8_t)19, (uint8_t)169, (uint8_t)27, (uint8_t)241, (uint8_t)141, (uint8_t)16, (uint8_t)79, (uint8_t)86, (uint8_t)132, (uint8_t)63, (uint8_t)175, (uint8_t)195, (uint8_t)120, (uint8_t)132, (uint8_t)245, (uint8_t)201, (uint8_t)208, (uint8_t)46, (uint8_t)158, (uint8_t)161, (uint8_t)127, (uint8_t)125, (uint8_t)137, (uint8_t)137, (uint8_t)192, (uint8_t)120, (uint8_t)202, (uint8_t)1, (uint8_t)144, (uint8_t)207, (uint8_t)216, (uint8_t)63, (uint8_t)53, (uint8_t)178, (uint8_t)219, (uint8_t)198, (uint8_t)78, (uint8_t)181, (uint8_t)206, (uint8_t)153, (uint8_t)100, (uint8_t)159, (uint8_t)63, (uint8_t)103, (uint8_t)0, (uint8_t)6, (uint8_t)28, (uint8_t)5, (uint8_t)112, (uint8_t)16, (uint8_t)98, (uint8_t)155, (uint8_t)111, (uint8_t)211, (uint8_t)24, (uint8_t)122, (uint8_t)16, (uint8_t)0, (uint8_t)254};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)1706499475L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)1261, PH.base.pack) ;
    p127_tow_SET((uint32_t)542440018L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -91753918, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)996071645, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)258804598, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)1083525386L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1278873162, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2523627935L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)21640, PH.base.pack) ;
    p128_tow_SET((uint32_t)366462190L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)643831859, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)829932033, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -794727397, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2793205671L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -53177490, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2300034006L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)10607, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)14777, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)26497, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -15214, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)28295, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)32021, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -18502, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -4004, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)14521, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p130_size_SET((uint32_t)671298091L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)47501, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)56500, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)836, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)45713, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)183, (uint8_t)71, (uint8_t)153, (uint8_t)181, (uint8_t)148, (uint8_t)156, (uint8_t)142, (uint8_t)177, (uint8_t)103, (uint8_t)3, (uint8_t)38, (uint8_t)174, (uint8_t)221, (uint8_t)0, (uint8_t)160, (uint8_t)201, (uint8_t)146, (uint8_t)190, (uint8_t)96, (uint8_t)75, (uint8_t)158, (uint8_t)227, (uint8_t)225, (uint8_t)163, (uint8_t)116, (uint8_t)21, (uint8_t)117, (uint8_t)188, (uint8_t)200, (uint8_t)49, (uint8_t)5, (uint8_t)6, (uint8_t)76, (uint8_t)161, (uint8_t)142, (uint8_t)70, (uint8_t)191, (uint8_t)166, (uint8_t)42, (uint8_t)169, (uint8_t)27, (uint8_t)248, (uint8_t)5, (uint8_t)85, (uint8_t)248, (uint8_t)171, (uint8_t)138, (uint8_t)219, (uint8_t)4, (uint8_t)189, (uint8_t)32, (uint8_t)71, (uint8_t)84, (uint8_t)252, (uint8_t)151, (uint8_t)24, (uint8_t)57, (uint8_t)157, (uint8_t)71, (uint8_t)61, (uint8_t)176, (uint8_t)109, (uint8_t)61, (uint8_t)160, (uint8_t)140, (uint8_t)82, (uint8_t)42, (uint8_t)239, (uint8_t)5, (uint8_t)208, (uint8_t)15, (uint8_t)173, (uint8_t)175, (uint8_t)7, (uint8_t)119, (uint8_t)78, (uint8_t)105, (uint8_t)8, (uint8_t)254, (uint8_t)65, (uint8_t)252, (uint8_t)102, (uint8_t)10, (uint8_t)180, (uint8_t)11, (uint8_t)0, (uint8_t)112, (uint8_t)221, (uint8_t)35, (uint8_t)13, (uint8_t)148, (uint8_t)59, (uint8_t)166, (uint8_t)174, (uint8_t)92, (uint8_t)60, (uint8_t)118, (uint8_t)52, (uint8_t)122, (uint8_t)208, (uint8_t)28, (uint8_t)32, (uint8_t)155, (uint8_t)232, (uint8_t)47, (uint8_t)141, (uint8_t)161, (uint8_t)121, (uint8_t)253, (uint8_t)96, (uint8_t)111, (uint8_t)93, (uint8_t)115, (uint8_t)98, (uint8_t)124, (uint8_t)105, (uint8_t)214, (uint8_t)156, (uint8_t)106, (uint8_t)200, (uint8_t)152, (uint8_t)248, (uint8_t)14, (uint8_t)154, (uint8_t)234, (uint8_t)147, (uint8_t)242, (uint8_t)201, (uint8_t)2, (uint8_t)9, (uint8_t)45, (uint8_t)193, (uint8_t)8, (uint8_t)170, (uint8_t)199, (uint8_t)137, (uint8_t)158, (uint8_t)34, (uint8_t)131, (uint8_t)115, (uint8_t)247, (uint8_t)41, (uint8_t)207, (uint8_t)230, (uint8_t)31, (uint8_t)125, (uint8_t)239, (uint8_t)197, (uint8_t)66, (uint8_t)16, (uint8_t)146, (uint8_t)205, (uint8_t)168, (uint8_t)95, (uint8_t)134, (uint8_t)226, (uint8_t)192, (uint8_t)113, (uint8_t)201, (uint8_t)188, (uint8_t)160, (uint8_t)47, (uint8_t)190, (uint8_t)89, (uint8_t)247, (uint8_t)230, (uint8_t)145, (uint8_t)150, (uint8_t)118, (uint8_t)123, (uint8_t)20, (uint8_t)4, (uint8_t)163, (uint8_t)188, (uint8_t)117, (uint8_t)197, (uint8_t)165, (uint8_t)97, (uint8_t)99, (uint8_t)201, (uint8_t)114, (uint8_t)180, (uint8_t)63, (uint8_t)133, (uint8_t)28, (uint8_t)66, (uint8_t)130, (uint8_t)85, (uint8_t)87, (uint8_t)129, (uint8_t)180, (uint8_t)38, (uint8_t)238, (uint8_t)218, (uint8_t)69, (uint8_t)7, (uint8_t)63, (uint8_t)132, (uint8_t)136, (uint8_t)176, (uint8_t)163, (uint8_t)6, (uint8_t)247, (uint8_t)29, (uint8_t)124, (uint8_t)36, (uint8_t)83, (uint8_t)64, (uint8_t)244, (uint8_t)36, (uint8_t)213, (uint8_t)41, (uint8_t)165, (uint8_t)154, (uint8_t)68, (uint8_t)141, (uint8_t)252, (uint8_t)64, (uint8_t)103, (uint8_t)32, (uint8_t)109, (uint8_t)212, (uint8_t)9, (uint8_t)62, (uint8_t)57, (uint8_t)253, (uint8_t)121, (uint8_t)61, (uint8_t)58, (uint8_t)28, (uint8_t)9, (uint8_t)58, (uint8_t)213, (uint8_t)244, (uint8_t)176, (uint8_t)150, (uint8_t)44, (uint8_t)103, (uint8_t)30, (uint8_t)72, (uint8_t)34, (uint8_t)143, (uint8_t)87, (uint8_t)152, (uint8_t)150, (uint8_t)106, (uint8_t)128, (uint8_t)64, (uint8_t)187, (uint8_t)255, (uint8_t)12, (uint8_t)103, (uint8_t)50};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2062284290L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)5532, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)15440, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)38075, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1848489290, PH.base.pack) ;
    p133_lon_SET((int32_t)461594371, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)43305, PH.base.pack) ;
    p133_mask_SET((uint64_t)1230131514662294089L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -30422144, PH.base.pack) ;
    p134_lon_SET((int32_t) -365573654, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)60026, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -29020, (int16_t)15387, (int16_t) -25933, (int16_t)30726, (int16_t)28900, (int16_t)24188, (int16_t)2520, (int16_t)8522, (int16_t) -11553, (int16_t) -12513, (int16_t)31146, (int16_t)7467, (int16_t) -16563, (int16_t) -10816, (int16_t)31452, (int16_t) -5002};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1890912222, PH.base.pack) ;
    p135_lon_SET((int32_t) -879274331, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)1461552208, PH.base.pack) ;
    p136_lon_SET((int32_t)2115749882, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)6532, PH.base.pack) ;
    p136_terrain_height_SET((float)1.2263252E38F, PH.base.pack) ;
    p136_current_height_SET((float)1.4697217E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)45523, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)56369, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)26015549L, PH.base.pack) ;
    p137_press_abs_SET((float)2.7933758E38F, PH.base.pack) ;
    p137_press_diff_SET((float) -2.828115E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -12590, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)8600027646442623793L, PH.base.pack) ;
    {
        float  q [] =  {-2.2683535E38F, 2.5594174E38F, -1.8401725E38F, -8.1540165E37F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)9.269201E37F, PH.base.pack) ;
    p138_y_SET((float) -1.0470124E38F, PH.base.pack) ;
    p138_z_SET((float)1.0860756E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)8402738297676581827L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    {
        float  controls [] =  {7.701896E37F, 1.8698528E38F, -1.2930654E38F, 1.4221807E38F, -2.29273E37F, 2.7053098E38F, 2.2290188E37F, 8.133756E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)3587128266092451748L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    {
        float  controls [] =  {1.8958388E38F, 1.789854E38F, -1.8614876E37F, -5.6413535E37F, -9.827106E37F, 5.3969094E37F, 7.805868E36F, 2.745368E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)3882622145164691348L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -1.5693888E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float)2.2205147E37F, PH.base.pack) ;
    p141_altitude_local_SET((float)2.5781669E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)5.773206E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)2.7572607E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float) -8.4110017E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)111, (uint8_t)104, (uint8_t)233, (uint8_t)135, (uint8_t)176, (uint8_t)224, (uint8_t)82, (uint8_t)188, (uint8_t)171, (uint8_t)102, (uint8_t)132, (uint8_t)18, (uint8_t)67, (uint8_t)160, (uint8_t)191, (uint8_t)122, (uint8_t)227, (uint8_t)207, (uint8_t)230, (uint8_t)34, (uint8_t)87, (uint8_t)85, (uint8_t)128, (uint8_t)175, (uint8_t)187, (uint8_t)15, (uint8_t)255, (uint8_t)131, (uint8_t)43, (uint8_t)234, (uint8_t)7, (uint8_t)56, (uint8_t)169, (uint8_t)154, (uint8_t)247, (uint8_t)109, (uint8_t)177, (uint8_t)166, (uint8_t)141, (uint8_t)82, (uint8_t)77, (uint8_t)145, (uint8_t)165, (uint8_t)95, (uint8_t)130, (uint8_t)91, (uint8_t)255, (uint8_t)8, (uint8_t)48, (uint8_t)178, (uint8_t)176, (uint8_t)94, (uint8_t)157, (uint8_t)248, (uint8_t)38, (uint8_t)233, (uint8_t)230, (uint8_t)225, (uint8_t)189, (uint8_t)95, (uint8_t)46, (uint8_t)158, (uint8_t)49, (uint8_t)75, (uint8_t)241, (uint8_t)22, (uint8_t)35, (uint8_t)66, (uint8_t)193, (uint8_t)94, (uint8_t)120, (uint8_t)208, (uint8_t)194, (uint8_t)221, (uint8_t)212, (uint8_t)254, (uint8_t)58, (uint8_t)168, (uint8_t)176, (uint8_t)115, (uint8_t)109, (uint8_t)198, (uint8_t)36, (uint8_t)125, (uint8_t)216, (uint8_t)237, (uint8_t)149, (uint8_t)19, (uint8_t)168, (uint8_t)136, (uint8_t)50, (uint8_t)19, (uint8_t)62, (uint8_t)154, (uint8_t)206, (uint8_t)211, (uint8_t)172, (uint8_t)222, (uint8_t)36, (uint8_t)121, (uint8_t)203, (uint8_t)158, (uint8_t)55, (uint8_t)254, (uint8_t)140, (uint8_t)11, (uint8_t)13, (uint8_t)88, (uint8_t)187, (uint8_t)99, (uint8_t)129, (uint8_t)186, (uint8_t)18, (uint8_t)147, (uint8_t)146, (uint8_t)177, (uint8_t)8, (uint8_t)21, (uint8_t)19, (uint8_t)228};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)111, (uint8_t)92, (uint8_t)254, (uint8_t)102, (uint8_t)131, (uint8_t)5, (uint8_t)178, (uint8_t)113, (uint8_t)174, (uint8_t)170, (uint8_t)190, (uint8_t)35, (uint8_t)78, (uint8_t)48, (uint8_t)186, (uint8_t)113, (uint8_t)188, (uint8_t)112, (uint8_t)78, (uint8_t)27, (uint8_t)75, (uint8_t)86, (uint8_t)225, (uint8_t)84, (uint8_t)139, (uint8_t)58, (uint8_t)20, (uint8_t)22, (uint8_t)237, (uint8_t)76, (uint8_t)254, (uint8_t)35, (uint8_t)42, (uint8_t)106, (uint8_t)88, (uint8_t)40, (uint8_t)81, (uint8_t)215, (uint8_t)26, (uint8_t)141, (uint8_t)81, (uint8_t)195, (uint8_t)9, (uint8_t)7, (uint8_t)60, (uint8_t)85, (uint8_t)152, (uint8_t)42, (uint8_t)178, (uint8_t)150, (uint8_t)188, (uint8_t)84, (uint8_t)167, (uint8_t)140, (uint8_t)206, (uint8_t)221, (uint8_t)210, (uint8_t)154, (uint8_t)222, (uint8_t)180, (uint8_t)99, (uint8_t)48, (uint8_t)233, (uint8_t)140, (uint8_t)107, (uint8_t)50, (uint8_t)54, (uint8_t)78, (uint8_t)154, (uint8_t)176, (uint8_t)41, (uint8_t)16, (uint8_t)113, (uint8_t)90, (uint8_t)235, (uint8_t)92, (uint8_t)193, (uint8_t)133, (uint8_t)98, (uint8_t)113, (uint8_t)187, (uint8_t)24, (uint8_t)108, (uint8_t)141, (uint8_t)193, (uint8_t)80, (uint8_t)129, (uint8_t)45, (uint8_t)84, (uint8_t)144, (uint8_t)60, (uint8_t)124, (uint8_t)20, (uint8_t)92, (uint8_t)130, (uint8_t)51, (uint8_t)97, (uint8_t)75, (uint8_t)174, (uint8_t)235, (uint8_t)58, (uint8_t)208, (uint8_t)95, (uint8_t)191, (uint8_t)14, (uint8_t)164, (uint8_t)230, (uint8_t)138, (uint8_t)221, (uint8_t)252, (uint8_t)132, (uint8_t)139, (uint8_t)24, (uint8_t)151, (uint8_t)229, (uint8_t)223, (uint8_t)153, (uint8_t)189, (uint8_t)176, (uint8_t)72};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)548613561L, PH.base.pack) ;
    p143_press_abs_SET((float) -2.0226734E37F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.1783855E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -3055, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)8225978872793437700L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p144_lat_SET((int32_t) -1833634373, PH.base.pack) ;
    p144_lon_SET((int32_t)1596747227, PH.base.pack) ;
    p144_alt_SET((float) -2.502023E38F, PH.base.pack) ;
    {
        float  vel [] =  {1.2796322E37F, 1.091112E38F, -1.2501849E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-5.793863E37F, 3.1408366E38F, -3.2868028E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-1.2682248E38F, -1.053906E38F, 2.8468587E38F, 1.9002642E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {9.402332E36F, -1.7636673E38F, -3.1002817E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {3.197309E38F, 7.0272956E37F, -9.481259E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)4708054050331720030L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)6844288869690952161L, PH.base.pack) ;
    p146_x_acc_SET((float) -1.9023616E38F, PH.base.pack) ;
    p146_y_acc_SET((float)1.3125778E37F, PH.base.pack) ;
    p146_z_acc_SET((float) -8.0771953E37F, PH.base.pack) ;
    p146_x_vel_SET((float) -1.4047634E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -3.3106385E38F, PH.base.pack) ;
    p146_z_vel_SET((float)6.491157E37F, PH.base.pack) ;
    p146_x_pos_SET((float) -1.1861859E38F, PH.base.pack) ;
    p146_y_pos_SET((float) -1.4307145E38F, PH.base.pack) ;
    p146_z_pos_SET((float)6.902965E37F, PH.base.pack) ;
    p146_airspeed_SET((float)2.9227471E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.4646196E38F, -2.7405503E38F, -3.142051E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.3142487E37F, 6.7065694E37F, -1.3539145E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {1.9331023E38F, -7.27644E36F, 1.5131641E38F, -3.2472464E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -2.8952716E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)2.4849753E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -1.0775393E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t)25308, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)5234, (uint16_t)65513, (uint16_t)43166, (uint16_t)35054, (uint16_t)13092, (uint16_t)57406, (uint16_t)53069, (uint16_t)30054, (uint16_t)49531, (uint16_t)45802};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)28884, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -2112675598, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)1125137737, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -12, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE), PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1010413873L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)1551873665L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)2039142722L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)3893149290L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)8, (uint8_t)93, (uint8_t)169, (uint8_t)120, (uint8_t)223, (uint8_t)133, (uint8_t)148, (uint8_t)240};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)112, (uint8_t)145, (uint8_t)123, (uint8_t)241, (uint8_t)152, (uint8_t)150, (uint8_t)48, (uint8_t)168};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)178, (uint8_t)207, (uint8_t)3, (uint8_t)136, (uint8_t)51, (uint8_t)37, (uint8_t)224, (uint8_t)119};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)521, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)40126, PH.base.pack) ;
    p148_uid_SET((uint64_t)5218263924014167895L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)244, (uint8_t)71, (uint8_t)45, (uint8_t)225, (uint8_t)173, (uint8_t)3, (uint8_t)90, (uint8_t)124, (uint8_t)25, (uint8_t)230, (uint8_t)104, (uint8_t)132, (uint8_t)24, (uint8_t)110, (uint8_t)150, (uint8_t)106, (uint8_t)14, (uint8_t)203};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)2725110159005739868L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p149_angle_x_SET((float)1.6390727E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -1.2009774E37F, PH.base.pack) ;
    p149_distance_SET((float) -2.8597741E38F, PH.base.pack) ;
    p149_size_x_SET((float)7.371409E37F, PH.base.pack) ;
    p149_size_y_SET((float) -2.9643977E38F, PH.base.pack) ;
    p149_x_SET((float) -1.6092015E38F, &PH) ;
    p149_y_SET((float) -1.405609E38F, &PH) ;
    p149_z_SET((float)2.2842715E38F, &PH) ;
    {
        float  q [] =  {-2.205957E38F, 1.4907016E38F, 1.8170532E38F, 6.774007E37F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)209, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)5694449605748144849L, PH.base.pack) ;
    p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS), PH.base.pack) ;
    p230_vel_ratio_SET((float)1.0312634E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -3.2665397E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -2.4497084E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float)2.831506E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -9.37261E37F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -1.0720813E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)1.8891695E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -2.58072E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)7011036054398136551L, PH.base.pack) ;
    p231_wind_x_SET((float)3.0679402E38F, PH.base.pack) ;
    p231_wind_y_SET((float)1.6757703E38F, PH.base.pack) ;
    p231_wind_z_SET((float)2.5697168E38F, PH.base.pack) ;
    p231_var_horiz_SET((float)6.1348914E37F, PH.base.pack) ;
    p231_var_vert_SET((float)2.4792072E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -2.3277278E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)6.30432E37F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -7.0410466E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)5629507748122795535L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ), PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)1778963595L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)58207, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p232_lat_SET((int32_t) -1418323191, PH.base.pack) ;
    p232_lon_SET((int32_t) -1878726431, PH.base.pack) ;
    p232_alt_SET((float)1.260756E37F, PH.base.pack) ;
    p232_hdop_SET((float)2.6476708E38F, PH.base.pack) ;
    p232_vdop_SET((float)7.740611E37F, PH.base.pack) ;
    p232_vn_SET((float) -2.8084132E38F, PH.base.pack) ;
    p232_ve_SET((float) -3.0164843E38F, PH.base.pack) ;
    p232_vd_SET((float)1.5206377E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -2.7886999E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -2.6359853E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)2.9394233E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)161, (uint8_t)171, (uint8_t)48, (uint8_t)217, (uint8_t)86, (uint8_t)25, (uint8_t)5, (uint8_t)83, (uint8_t)52, (uint8_t)221, (uint8_t)1, (uint8_t)203, (uint8_t)244, (uint8_t)143, (uint8_t)176, (uint8_t)20, (uint8_t)38, (uint8_t)242, (uint8_t)255, (uint8_t)60, (uint8_t)139, (uint8_t)127, (uint8_t)94, (uint8_t)86, (uint8_t)61, (uint8_t)59, (uint8_t)173, (uint8_t)59, (uint8_t)178, (uint8_t)6, (uint8_t)188, (uint8_t)148, (uint8_t)176, (uint8_t)212, (uint8_t)232, (uint8_t)33, (uint8_t)3, (uint8_t)249, (uint8_t)48, (uint8_t)22, (uint8_t)69, (uint8_t)148, (uint8_t)233, (uint8_t)255, (uint8_t)1, (uint8_t)246, (uint8_t)236, (uint8_t)92, (uint8_t)100, (uint8_t)107, (uint8_t)20, (uint8_t)2, (uint8_t)0, (uint8_t)192, (uint8_t)167, (uint8_t)44, (uint8_t)32, (uint8_t)17, (uint8_t)73, (uint8_t)92, (uint8_t)116, (uint8_t)253, (uint8_t)70, (uint8_t)50, (uint8_t)76, (uint8_t)165, (uint8_t)252, (uint8_t)227, (uint8_t)22, (uint8_t)43, (uint8_t)186, (uint8_t)70, (uint8_t)178, (uint8_t)194, (uint8_t)18, (uint8_t)147, (uint8_t)179, (uint8_t)87, (uint8_t)105, (uint8_t)16, (uint8_t)37, (uint8_t)222, (uint8_t)37, (uint8_t)216, (uint8_t)137, (uint8_t)158, (uint8_t)198, (uint8_t)234, (uint8_t)195, (uint8_t)111, (uint8_t)145, (uint8_t)253, (uint8_t)122, (uint8_t)255, (uint8_t)230, (uint8_t)88, (uint8_t)146, (uint8_t)194, (uint8_t)125, (uint8_t)144, (uint8_t)125, (uint8_t)184, (uint8_t)155, (uint8_t)174, (uint8_t)119, (uint8_t)11, (uint8_t)14, (uint8_t)1, (uint8_t)164, (uint8_t)7, (uint8_t)148, (uint8_t)87, (uint8_t)145, (uint8_t)0, (uint8_t)168, (uint8_t)223, (uint8_t)66, (uint8_t)132, (uint8_t)58, (uint8_t)40, (uint8_t)202, (uint8_t)223, (uint8_t)149, (uint8_t)82, (uint8_t)58, (uint8_t)75, (uint8_t)75, (uint8_t)207, (uint8_t)3, (uint8_t)16, (uint8_t)170, (uint8_t)197, (uint8_t)197, (uint8_t)106, (uint8_t)79, (uint8_t)51, (uint8_t)152, (uint8_t)57, (uint8_t)56, (uint8_t)83, (uint8_t)11, (uint8_t)120, (uint8_t)12, (uint8_t)95, (uint8_t)140, (uint8_t)46, (uint8_t)109, (uint8_t)184, (uint8_t)162, (uint8_t)76, (uint8_t)205, (uint8_t)154, (uint8_t)236, (uint8_t)5, (uint8_t)68, (uint8_t)31, (uint8_t)76, (uint8_t)60, (uint8_t)179, (uint8_t)163, (uint8_t)215, (uint8_t)221, (uint8_t)17, (uint8_t)179, (uint8_t)7, (uint8_t)238, (uint8_t)53, (uint8_t)72, (uint8_t)32, (uint8_t)6, (uint8_t)76, (uint8_t)58, (uint8_t)126, (uint8_t)198, (uint8_t)225, (uint8_t)182, (uint8_t)40, (uint8_t)13, (uint8_t)132, (uint8_t)194};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)3103346800L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)24226, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -470, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)36493, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -108, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -26424, PH.base.pack) ;
    p234_latitude_SET((int32_t) -199229593, PH.base.pack) ;
    p234_longitude_SET((int32_t)863253104, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)25225, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)31079, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)99, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)32, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -119, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)23960, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)5961002224373116964L, PH.base.pack) ;
    p241_vibration_x_SET((float) -2.1309884E38F, PH.base.pack) ;
    p241_vibration_y_SET((float) -3.3441708E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)2.0983853E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3904407644L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)2028863697L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)1708402621L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -984319841, PH.base.pack) ;
    p242_longitude_SET((int32_t)906764939, PH.base.pack) ;
    p242_altitude_SET((int32_t)331849668, PH.base.pack) ;
    p242_x_SET((float)2.171251E38F, PH.base.pack) ;
    p242_y_SET((float)2.3478848E38F, PH.base.pack) ;
    p242_z_SET((float)1.1546246E38F, PH.base.pack) ;
    {
        float  q [] =  {1.7951093E38F, 1.5165629E38F, -5.201891E37F, 7.3649896E37F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -7.1918884E37F, PH.base.pack) ;
    p242_approach_y_SET((float) -3.3722962E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -1.5168451E37F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)3535742108706276473L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p243_latitude_SET((int32_t) -171961888, PH.base.pack) ;
    p243_longitude_SET((int32_t)1323300338, PH.base.pack) ;
    p243_altitude_SET((int32_t) -206482079, PH.base.pack) ;
    p243_x_SET((float)1.2294104E38F, PH.base.pack) ;
    p243_y_SET((float)2.2425129E38F, PH.base.pack) ;
    p243_z_SET((float) -5.0252023E37F, PH.base.pack) ;
    {
        float  q [] =  {4.62265E37F, -2.6403935E38F, -3.0021678E38F, -2.8957448E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -1.3619359E38F, PH.base.pack) ;
    p243_approach_y_SET((float)2.3257715E38F, PH.base.pack) ;
    p243_approach_z_SET((float)1.608548E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)4841642330789671011L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)14931, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -1236556944, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)2579152872L, PH.base.pack) ;
    p246_lat_SET((int32_t) -1501437818, PH.base.pack) ;
    p246_lon_SET((int32_t) -1786595960, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t)1042366175, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)25393, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)25400, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)4641, PH.base.pack) ;
    {
        char16_t   callsign = "sKI";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN |
                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE), PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)62952, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
    p247_id_SET((uint32_t)1455125007L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float) -2.573618E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -3.0269519E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)3.3884006E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)23372, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)41, (uint8_t)215, (uint8_t)246, (uint8_t)122, (uint8_t)240, (uint8_t)32, (uint8_t)237, (uint8_t)196, (uint8_t)48, (uint8_t)15, (uint8_t)156, (uint8_t)125, (uint8_t)102, (uint8_t)2, (uint8_t)35, (uint8_t)149, (uint8_t)213, (uint8_t)64, (uint8_t)52, (uint8_t)119, (uint8_t)30, (uint8_t)13, (uint8_t)178, (uint8_t)142, (uint8_t)201, (uint8_t)41, (uint8_t)85, (uint8_t)37, (uint8_t)179, (uint8_t)0, (uint8_t)103, (uint8_t)20, (uint8_t)81, (uint8_t)245, (uint8_t)228, (uint8_t)131, (uint8_t)9, (uint8_t)111, (uint8_t)89, (uint8_t)77, (uint8_t)94, (uint8_t)219, (uint8_t)95, (uint8_t)40, (uint8_t)91, (uint8_t)94, (uint8_t)75, (uint8_t)29, (uint8_t)9, (uint8_t)3, (uint8_t)107, (uint8_t)141, (uint8_t)238, (uint8_t)139, (uint8_t)31, (uint8_t)216, (uint8_t)187, (uint8_t)44, (uint8_t)11, (uint8_t)6, (uint8_t)218, (uint8_t)253, (uint8_t)149, (uint8_t)30, (uint8_t)41, (uint8_t)159, (uint8_t)184, (uint8_t)152, (uint8_t)187, (uint8_t)150, (uint8_t)17, (uint8_t)173, (uint8_t)7, (uint8_t)53, (uint8_t)236, (uint8_t)78, (uint8_t)47, (uint8_t)29, (uint8_t)252, (uint8_t)238, (uint8_t)126, (uint8_t)16, (uint8_t)160, (uint8_t)190, (uint8_t)242, (uint8_t)186, (uint8_t)153, (uint8_t)40, (uint8_t)247, (uint8_t)25, (uint8_t)196, (uint8_t)23, (uint8_t)33, (uint8_t)203, (uint8_t)247, (uint8_t)89, (uint8_t)142, (uint8_t)12, (uint8_t)98, (uint8_t)179, (uint8_t)85, (uint8_t)52, (uint8_t)156, (uint8_t)94, (uint8_t)18, (uint8_t)115, (uint8_t)211, (uint8_t)241, (uint8_t)56, (uint8_t)192, (uint8_t)217, (uint8_t)160, (uint8_t)127, (uint8_t)198, (uint8_t)104, (uint8_t)41, (uint8_t)26, (uint8_t)2, (uint8_t)228, (uint8_t)164, (uint8_t)86, (uint8_t)29, (uint8_t)191, (uint8_t)243, (uint8_t)96, (uint8_t)56, (uint8_t)91, (uint8_t)154, (uint8_t)90, (uint8_t)68, (uint8_t)126, (uint8_t)225, (uint8_t)132, (uint8_t)163, (uint8_t)40, (uint8_t)75, (uint8_t)75, (uint8_t)54, (uint8_t)37, (uint8_t)227, (uint8_t)177, (uint8_t)3, (uint8_t)236, (uint8_t)84, (uint8_t)153, (uint8_t)3, (uint8_t)172, (uint8_t)71, (uint8_t)10, (uint8_t)82, (uint8_t)56, (uint8_t)225, (uint8_t)82, (uint8_t)142, (uint8_t)190, (uint8_t)78, (uint8_t)24, (uint8_t)195, (uint8_t)6, (uint8_t)117, (uint8_t)223, (uint8_t)180, (uint8_t)140, (uint8_t)240, (uint8_t)85, (uint8_t)186, (uint8_t)250, (uint8_t)172, (uint8_t)71, (uint8_t)240, (uint8_t)210, (uint8_t)2, (uint8_t)57, (uint8_t)135, (uint8_t)177, (uint8_t)206, (uint8_t)147, (uint8_t)142, (uint8_t)85, (uint8_t)216, (uint8_t)141, (uint8_t)124, (uint8_t)64, (uint8_t)55, (uint8_t)201, (uint8_t)108, (uint8_t)179, (uint8_t)143, (uint8_t)147, (uint8_t)110, (uint8_t)41, (uint8_t)207, (uint8_t)44, (uint8_t)3, (uint8_t)52, (uint8_t)234, (uint8_t)60, (uint8_t)204, (uint8_t)183, (uint8_t)91, (uint8_t)5, (uint8_t)14, (uint8_t)76, (uint8_t)198, (uint8_t)133, (uint8_t)186, (uint8_t)72, (uint8_t)238, (uint8_t)87, (uint8_t)65, (uint8_t)211, (uint8_t)3, (uint8_t)231, (uint8_t)210, (uint8_t)213, (uint8_t)67, (uint8_t)186, (uint8_t)223, (uint8_t)171, (uint8_t)118, (uint8_t)17, (uint8_t)134, (uint8_t)38, (uint8_t)238, (uint8_t)46, (uint8_t)63, (uint8_t)133, (uint8_t)162, (uint8_t)164, (uint8_t)138, (uint8_t)178, (uint8_t)126, (uint8_t)213, (uint8_t)74, (uint8_t)54, (uint8_t)209, (uint8_t)193, (uint8_t)77, (uint8_t)161, (uint8_t)138, (uint8_t)163, (uint8_t)235, (uint8_t)67, (uint8_t)153, (uint8_t)203, (uint8_t)245, (uint8_t)29, (uint8_t)90, (uint8_t)163};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)4697, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)79, (int8_t)46, (int8_t) -31, (int8_t)12, (int8_t) -62, (int8_t) -12, (int8_t) -52, (int8_t)29, (int8_t) -46, (int8_t)112, (int8_t)76, (int8_t) -111, (int8_t)93, (int8_t) -98, (int8_t)100, (int8_t)29, (int8_t) -78, (int8_t)19, (int8_t)77, (int8_t) -1, (int8_t)127, (int8_t) -102, (int8_t) -46, (int8_t)74, (int8_t) -13, (int8_t) -99, (int8_t) -6, (int8_t) -42, (int8_t)15, (int8_t) -49, (int8_t)83, (int8_t) -42};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "xzrqej";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)1708593993010267191L, PH.base.pack) ;
    p250_x_SET((float) -2.1327574E38F, PH.base.pack) ;
    p250_y_SET((float)4.2246474E37F, PH.base.pack) ;
    p250_z_SET((float) -2.5151356E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)917758962L, PH.base.pack) ;
    {
        char16_t   name = "pngohwfrt";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -2.5714063E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)2789306544L, PH.base.pack) ;
    {
        char16_t   name = "uwkzgykawD";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -2030667548, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ALERT, PH.base.pack) ;
    {
        char16_t   text = "cswkcwqulxesdctPzosscktst";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)47383574L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p254_value_SET((float)1.0166335E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)95, (uint8_t)255, (uint8_t)242, (uint8_t)47, (uint8_t)192, (uint8_t)141, (uint8_t)13, (uint8_t)123, (uint8_t)213, (uint8_t)105, (uint8_t)21, (uint8_t)251, (uint8_t)131, (uint8_t)138, (uint8_t)208, (uint8_t)82, (uint8_t)139, (uint8_t)77, (uint8_t)86, (uint8_t)251, (uint8_t)187, (uint8_t)21, (uint8_t)94, (uint8_t)130, (uint8_t)209, (uint8_t)12, (uint8_t)61, (uint8_t)246, (uint8_t)24, (uint8_t)92, (uint8_t)183, (uint8_t)8};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)3577151375470809274L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)3062988427L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)2302489849L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    {
        char16_t   tune = "ftr";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)1967186106L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)175, (uint8_t)42, (uint8_t)199, (uint8_t)106, (uint8_t)190, (uint8_t)8, (uint8_t)163, (uint8_t)176, (uint8_t)240, (uint8_t)167, (uint8_t)1, (uint8_t)202, (uint8_t)149, (uint8_t)104, (uint8_t)94, (uint8_t)236, (uint8_t)69, (uint8_t)77, (uint8_t)181, (uint8_t)167, (uint8_t)63, (uint8_t)225, (uint8_t)43, (uint8_t)173, (uint8_t)83, (uint8_t)90, (uint8_t)84, (uint8_t)92, (uint8_t)143, (uint8_t)164, (uint8_t)48, (uint8_t)208};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)227, (uint8_t)215, (uint8_t)252, (uint8_t)143, (uint8_t)72, (uint8_t)178, (uint8_t)127, (uint8_t)74, (uint8_t)41, (uint8_t)73, (uint8_t)177, (uint8_t)243, (uint8_t)199, (uint8_t)152, (uint8_t)219, (uint8_t)228, (uint8_t)219, (uint8_t)70, (uint8_t)133, (uint8_t)43, (uint8_t)32, (uint8_t)28, (uint8_t)21, (uint8_t)89, (uint8_t)171, (uint8_t)17, (uint8_t)124, (uint8_t)169, (uint8_t)232, (uint8_t)130, (uint8_t)197, (uint8_t)8};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1960955118L, PH.base.pack) ;
    p259_focal_length_SET((float) -3.350482E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -3.2920685E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -3.3949953E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)39447, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)44520, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE), PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)18860, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "pjMsntdjocvdksaysfbbjhWsfvbiMhwrteljsouftscdHxfztOoYppiqtftunpixKfjeEswvxctibDaVjpHlfwyerrrmruxnasmpHSgrj";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)3658184330L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)1807865764L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p261_total_capacity_SET((float) -1.939173E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)2.8138334E38F, PH.base.pack) ;
    p261_available_capacity_SET((float)2.9992917E38F, PH.base.pack) ;
    p261_read_speed_SET((float) -2.2919819E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -3.1087283E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)4151743601L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p262_image_interval_SET((float) -3.2979537E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)2367262729L, PH.base.pack) ;
    p262_available_capacity_SET((float)3.0171705E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)2537418212L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)5951320051660555775L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    p263_lat_SET((int32_t) -2004705593, PH.base.pack) ;
    p263_lon_SET((int32_t)1906108823, PH.base.pack) ;
    p263_alt_SET((int32_t) -200087790, PH.base.pack) ;
    p263_relative_alt_SET((int32_t) -1786213350, PH.base.pack) ;
    {
        float  q [] =  {-1.5551847E38F, -2.408114E38F, -3.0848612E38F, 1.7885118E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -683396586, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)8, PH.base.pack) ;
    {
        char16_t   file_url = "obdzhZwlvbdjgqwTnduxmnhlnixsynbwhxitxtndhcHYcdxCUvlbirEYyuzkprdovgdtopTptujxGMbiuCzpaceyosncjqusffcnGpklnymkrUqcsvSlYdkcjeoniuju";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)2992966323L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)1891010399827465802L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)4380688142177527227L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)2074605085277684381L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}


#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_CMD  command = p77_command_GET(pack);
    e_MAV_RESULT  result = p77_result_GET(pack);
    uint8_t  progress = p77_progress_TRY(ph);
    int32_t  result_param2 = p77_result_param2_TRY(ph);
    uint8_t  target_system = p77_target_system_TRY(ph);
    uint8_t  target_component = p77_target_component_TRY(ph);
}
void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p81_time_boot_ms_GET(pack);
    float  roll = p81_roll_GET(pack);
    float  pitch = p81_pitch_GET(pack);
    float  yaw = p81_yaw_GET(pack);
    float  thrust = p81_thrust_GET(pack);
    uint8_t  mode_switch = p81_mode_switch_GET(pack);
    uint8_t  manual_override_switch = p81_manual_override_switch_GET(pack);
}
void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_ARRAY_TEST_0_150(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v1 = p150_v1_GET(pack);
    int8_t*  ar_i8 = p150_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    uint8_t*  ar_u8 = p150_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    uint16_t*  ar_u16 = p150_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    uint32_t*  ar_u32 = p150_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_CommunicationChannel_on_ARRAY_TEST_1_151(Bounds_Inside * ph, Pack * pack)
{
    uint32_t*  ar_u32 = p151_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_CommunicationChannel_on_ARRAY_TEST_3_153(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v = p153_v_GET(pack);
    uint32_t*  ar_u32 = p153_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_CommunicationChannel_on_ARRAY_TEST_4_154(Bounds_Inside * ph, Pack * pack)
{
    uint32_t*  ar_u32 = p154_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    uint8_t  v = p154_v_GET(pack);
}
void c_CommunicationChannel_on_ARRAY_TEST_5_155(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  c1 = p155_c1_TRY_(ph);
    char16_t *  c2 = p155_c2_TRY_(ph);
}
void c_CommunicationChannel_on_ARRAY_TEST_6_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v1 = p156_v1_GET(pack);
    uint16_t  v2 = p156_v2_GET(pack);
    uint32_t  v3 = p156_v3_GET(pack);
    uint32_t*  ar_u32 = p156_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    int32_t*  ar_i32 = p156_ar_i32_GET_(pack);
//process data in ar_i32
    free(ar_i32);//never forget to dispose
    uint16_t*  ar_u16 = p156_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    int16_t*  ar_i16 = p156_ar_i16_GET_(pack);
//process data in ar_i16
    free(ar_i16);//never forget to dispose
    uint8_t*  ar_u8 = p156_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    int8_t*  ar_i8 = p156_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    char16_t *  ar_c = p156_ar_c_TRY_(ph);
    double*  ar_d = p156_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    float*  ar_f = p156_ar_f_GET_(pack);
//process data in ar_f
    free(ar_f);//never forget to dispose
}
void c_CommunicationChannel_on_ARRAY_TEST_7_157(Bounds_Inside * ph, Pack * pack)
{
    double*  ar_d = p157_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    float*  ar_f = p157_ar_f_GET_(pack);
//process data in ar_f
    free(ar_f);//never forget to dispose
    uint32_t*  ar_u32 = p157_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    int32_t*  ar_i32 = p157_ar_i32_GET_(pack);
//process data in ar_i32
    free(ar_i32);//never forget to dispose
    uint16_t*  ar_u16 = p157_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    int16_t*  ar_i16 = p157_ar_i16_GET_(pack);
//process data in ar_i16
    free(ar_i16);//never forget to dispose
    uint8_t*  ar_u8 = p157_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    int8_t*  ar_i8 = p157_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    char16_t *  ar_c = p157_ar_c_TRY_(ph);
}
void c_CommunicationChannel_on_ARRAY_TEST_8_158(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  v3 = p158_v3_GET(pack);
    double*  ar_d = p158_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    uint16_t*  ar_u16 = p158_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMACCMPILOT, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)24038917L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)58893, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)54408, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -13705, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -17, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)59323, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)49153, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)40638, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)61501, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)37646, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)59985, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)5517387005682188781L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)1335415358L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)765578322L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)32389, PH.base.pack) ;
    p3_x_SET((float) -2.1884093E38F, PH.base.pack) ;
    p3_y_SET((float) -2.9540792E38F, PH.base.pack) ;
    p3_z_SET((float)2.3459947E38F, PH.base.pack) ;
    p3_vx_SET((float)2.2650424E38F, PH.base.pack) ;
    p3_vy_SET((float)5.0489834E37F, PH.base.pack) ;
    p3_vz_SET((float)5.9888753E37F, PH.base.pack) ;
    p3_afx_SET((float)6.2575123E37F, PH.base.pack) ;
    p3_afy_SET((float) -8.412625E37F, PH.base.pack) ;
    p3_afz_SET((float)7.224483E36F, PH.base.pack) ;
    p3_yaw_SET((float)2.5599894E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -2.7509505E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)422780818978382981L, PH.base.pack) ;
    p4_seq_SET((uint32_t)3804176743L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    {
        char16_t   passkey = "iuihJwrnbvewwslosir";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "umchuzwmevoyhvodkmtvzaprdmjehi";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)2132668980L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    {
        char16_t   param_id = "dlhtrcsocbgkezc";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)1982, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "oh";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -4.0142212E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)61241, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)40913, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    {
        char16_t   param_id = "Vainvo";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -3.254795E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)8649789953179810427L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -464317936, PH.base.pack) ;
    p24_lon_SET((int32_t) -723591629, PH.base.pack) ;
    p24_alt_SET((int32_t) -2043856008, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)14588, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)55662, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)21242, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)4606, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -554279587, &PH) ;
    p24_h_acc_SET((uint32_t)3573158782L, &PH) ;
    p24_v_acc_SET((uint32_t)3199681881L, &PH) ;
    p24_vel_acc_SET((uint32_t)788430194L, &PH) ;
    p24_hdg_acc_SET((uint32_t)108386662L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)218, (uint8_t)115, (uint8_t)31, (uint8_t)201, (uint8_t)224, (uint8_t)142, (uint8_t)228, (uint8_t)115, (uint8_t)38, (uint8_t)143, (uint8_t)5, (uint8_t)63, (uint8_t)163, (uint8_t)153, (uint8_t)87, (uint8_t)84, (uint8_t)168, (uint8_t)230, (uint8_t)81, (uint8_t)21};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)80, (uint8_t)115, (uint8_t)125, (uint8_t)230, (uint8_t)76, (uint8_t)77, (uint8_t)3, (uint8_t)53, (uint8_t)138, (uint8_t)96, (uint8_t)77, (uint8_t)64, (uint8_t)195, (uint8_t)137, (uint8_t)224, (uint8_t)34, (uint8_t)101, (uint8_t)27, (uint8_t)46, (uint8_t)218};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)41, (uint8_t)105, (uint8_t)106, (uint8_t)254, (uint8_t)44, (uint8_t)52, (uint8_t)175, (uint8_t)149, (uint8_t)227, (uint8_t)4, (uint8_t)90, (uint8_t)166, (uint8_t)231, (uint8_t)217, (uint8_t)34, (uint8_t)47, (uint8_t)246, (uint8_t)93, (uint8_t)66, (uint8_t)163};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)178, (uint8_t)81, (uint8_t)199, (uint8_t)238, (uint8_t)253, (uint8_t)42, (uint8_t)137, (uint8_t)125, (uint8_t)115, (uint8_t)102, (uint8_t)173, (uint8_t)8, (uint8_t)185, (uint8_t)88, (uint8_t)150, (uint8_t)200, (uint8_t)219, (uint8_t)41, (uint8_t)38, (uint8_t)176};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)244, (uint8_t)170, (uint8_t)208, (uint8_t)170, (uint8_t)105, (uint8_t)192, (uint8_t)97, (uint8_t)168, (uint8_t)249, (uint8_t)19, (uint8_t)76, (uint8_t)39, (uint8_t)93, (uint8_t)11, (uint8_t)157, (uint8_t)168, (uint8_t)162, (uint8_t)232, (uint8_t)1, (uint8_t)83};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)2870571554L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -26991, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)24772, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -3438, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -29047, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)25262, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)8570, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)844, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)22705, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)15435, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)1999930296634946388L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -5391, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)19009, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -28835, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -274, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -11647, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)9632, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)21491, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)23275, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)25969, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)459837615572292730L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)21957, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)30149, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -12952, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -18663, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)1692422831L, PH.base.pack) ;
    p29_press_abs_SET((float) -2.2513092E37F, PH.base.pack) ;
    p29_press_diff_SET((float)1.5113891E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)24700, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2570667012L, PH.base.pack) ;
    p30_roll_SET((float) -1.1623048E38F, PH.base.pack) ;
    p30_pitch_SET((float) -2.1169952E38F, PH.base.pack) ;
    p30_yaw_SET((float)3.3098452E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -6.131384E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -2.3880118E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -1.3149674E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)3492396101L, PH.base.pack) ;
    p31_q1_SET((float)2.178161E38F, PH.base.pack) ;
    p31_q2_SET((float) -9.756698E37F, PH.base.pack) ;
    p31_q3_SET((float)2.35091E38F, PH.base.pack) ;
    p31_q4_SET((float) -1.0147516E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -2.5352965E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)6.9379253E37F, PH.base.pack) ;
    p31_yawspeed_SET((float)3.2717038E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)3737385745L, PH.base.pack) ;
    p32_x_SET((float)1.3773384E38F, PH.base.pack) ;
    p32_y_SET((float) -1.8218082E38F, PH.base.pack) ;
    p32_z_SET((float)9.527568E37F, PH.base.pack) ;
    p32_vx_SET((float)3.133115E37F, PH.base.pack) ;
    p32_vy_SET((float)2.1331339E38F, PH.base.pack) ;
    p32_vz_SET((float) -2.255935E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)2148314457L, PH.base.pack) ;
    p33_lat_SET((int32_t) -880642418, PH.base.pack) ;
    p33_lon_SET((int32_t)1807113472, PH.base.pack) ;
    p33_alt_SET((int32_t)1012719947, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -569375762, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -1088, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -27366, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)28170, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)49640, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)164781854L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)1009, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)25063, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)10561, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)23763, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -11430, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -13902, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -15436, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -25560, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)883005774L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)38053, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)2238, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)2300, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)38560, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)413, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)25619, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)14496, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)11198, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)3300976726L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)37760, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)45364, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)8082, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)19339, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)20371, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)2466, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)29004, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)50308, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)29154, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)31043, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)39011, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)31239, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)20125, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)23879, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)34462, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)6207, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -13361, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -28980, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)5133, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)25373, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)56119, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_SPATIAL_USER_5, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p39_param1_SET((float)2.4957053E38F, PH.base.pack) ;
    p39_param2_SET((float)3.1291507E38F, PH.base.pack) ;
    p39_param3_SET((float)2.2979236E38F, PH.base.pack) ;
    p39_param4_SET((float) -1.6000372E38F, PH.base.pack) ;
    p39_x_SET((float) -9.610366E37F, PH.base.pack) ;
    p39_y_SET((float) -9.544767E37F, PH.base.pack) ;
    p39_z_SET((float)3.2609963E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)64228, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)32061, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)25624, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)27242, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)19179, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p48_latitude_SET((int32_t) -1625902777, PH.base.pack) ;
    p48_longitude_SET((int32_t)489999186, PH.base.pack) ;
    p48_altitude_SET((int32_t)183696465, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)4637844891339876139L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -247530984, PH.base.pack) ;
    p49_longitude_SET((int32_t) -1539739512, PH.base.pack) ;
    p49_altitude_SET((int32_t)594562662, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)5323570391280432025L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    {
        char16_t   param_id = "sQwxdgqCo";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -16891, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p50_param_value0_SET((float) -1.8730377E38F, PH.base.pack) ;
    p50_scale_SET((float) -1.8754769E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.1587291E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)2.4408977E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)23364, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p54_p1x_SET((float)4.5297593E37F, PH.base.pack) ;
    p54_p1y_SET((float)2.5780052E38F, PH.base.pack) ;
    p54_p1z_SET((float) -8.51427E37F, PH.base.pack) ;
    p54_p2x_SET((float) -2.6112834E38F, PH.base.pack) ;
    p54_p2y_SET((float)2.3380339E38F, PH.base.pack) ;
    p54_p2z_SET((float)1.8254225E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p55_p1x_SET((float)1.1280634E38F, PH.base.pack) ;
    p55_p1y_SET((float) -1.8062395E37F, PH.base.pack) ;
    p55_p1z_SET((float) -2.1566412E38F, PH.base.pack) ;
    p55_p2x_SET((float) -1.7556928E38F, PH.base.pack) ;
    p55_p2y_SET((float) -1.00822E38F, PH.base.pack) ;
    p55_p2z_SET((float) -2.5549386E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)7474023323763966134L, PH.base.pack) ;
    {
        float  q [] =  {1.8961041E38F, -7.47449E37F, -2.7237307E38F, 2.6247983E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)2.917224E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -3.055306E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.5476457E37F, PH.base.pack) ;
    {
        float  covariance [] =  {2.4638116E38F, 8.150396E36F, -5.206762E37F, -1.1742804E38F, -5.2865867E37F, 6.357071E37F, -1.9196635E38F, -1.5346911E38F, -2.092823E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.4936494E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)9.842741E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -161, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)26338, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)4179, PH.base.pack) ;
    p62_alt_error_SET((float)3.3341687E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -1.5177255E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -3.3448332E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)5953932255085220262L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t) -2101547942, PH.base.pack) ;
    p63_lon_SET((int32_t) -1994733029, PH.base.pack) ;
    p63_alt_SET((int32_t)1391616545, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)1145056090, PH.base.pack) ;
    p63_vx_SET((float) -3.2825397E38F, PH.base.pack) ;
    p63_vy_SET((float) -3.3359434E38F, PH.base.pack) ;
    p63_vz_SET((float) -1.1945403E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.7585496E38F, -7.391727E37F, 1.5216454E38F, -5.0165457E37F, 1.1649201E38F, 2.2297018E37F, -2.7563277E38F, -1.5739557E38F, -6.1577294E37F, -2.608185E38F, 2.4945147E38F, -3.1320334E38F, -2.8110187E38F, 1.0500404E38F, -1.8451155E38F, -3.364022E38F, -2.146501E38F, 2.9324898E38F, -3.0701126E38F, -3.363769E38F, 1.5793533E38F, 9.27358E37F, -1.617957E37F, 3.3710273E38F, 1.5338952E38F, 2.9736124E38F, 2.112237E38F, -3.0416968E38F, -1.7778767E38F, -1.499348E37F, -3.3400336E38F, 1.8483703E38F, 8.999348E37F, 1.8590603E38F, -1.7726483E38F, -8.5816384E36F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)9034576992131041031L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p64_x_SET((float)2.319598E38F, PH.base.pack) ;
    p64_y_SET((float)3.246033E38F, PH.base.pack) ;
    p64_z_SET((float)1.7626284E38F, PH.base.pack) ;
    p64_vx_SET((float)1.6953605E38F, PH.base.pack) ;
    p64_vy_SET((float) -1.6225907E38F, PH.base.pack) ;
    p64_vz_SET((float) -1.5638161E38F, PH.base.pack) ;
    p64_ax_SET((float)1.9661547E38F, PH.base.pack) ;
    p64_ay_SET((float) -2.941907E38F, PH.base.pack) ;
    p64_az_SET((float) -2.0992736E38F, PH.base.pack) ;
    {
        float  covariance [] =  {7.2164666E37F, 2.1430577E38F, -1.721034E36F, -6.5404093E37F, 1.7641549E38F, -2.8632812E38F, 1.8389647E38F, -1.5781656E38F, -3.0668312E38F, 5.574658E37F, -3.120089E37F, 2.2111027E38F, 1.6932932E38F, -2.7915104E38F, -1.4236865E38F, -2.2218712E38F, -8.386829E37F, 5.1686557E37F, 3.1222074E38F, -1.5709818E38F, -1.8484524E38F, -2.6225225E38F, 3.0932662E37F, 2.2170185E38F, 7.8463323E37F, 7.056537E37F, 9.403728E37F, 2.7350773E38F, -1.6389993E37F, 3.0312582E38F, 2.6327411E38F, -1.4275987E38F, -2.841501E38F, -9.512848E37F, -1.246443E38F, -3.0563393E38F, 2.8271357E38F, 1.5850056E38F, 2.0785125E38F, 4.422726E37F, 7.042569E37F, -1.5145595E38F, 3.4105612E37F, -8.159925E37F, -2.7978077E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)4245809773L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)47300, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)15580, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)52274, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)31312, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)15018, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)49056, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)36065, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)24651, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)62359, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)63510, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)11196, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)12079, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)47304, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)52475, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)13913, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)56938, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)62538, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)33119, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)13210, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)41381, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -24697, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -29601, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)6254, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)29683, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)9637, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)59762, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)57730, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)29560, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)39378, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)28897, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)41844, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)845, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)49207, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)40507, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p73_param1_SET((float)3.0987074E38F, PH.base.pack) ;
    p73_param2_SET((float) -3.0307686E38F, PH.base.pack) ;
    p73_param3_SET((float)2.8288287E37F, PH.base.pack) ;
    p73_param4_SET((float)2.840317E38F, PH.base.pack) ;
    p73_x_SET((int32_t)1898712279, PH.base.pack) ;
    p73_y_SET((int32_t)1139731341, PH.base.pack) ;
    p73_z_SET((float) -2.1929572E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -3.2643033E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.4173007E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)5344, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)37270, PH.base.pack) ;
    p74_alt_SET((float) -3.4021468E38F, PH.base.pack) ;
    p74_climb_SET((float)8.3001715E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p75_param1_SET((float) -8.650059E37F, PH.base.pack) ;
    p75_param2_SET((float)2.577573E38F, PH.base.pack) ;
    p75_param3_SET((float)2.4062785E38F, PH.base.pack) ;
    p75_param4_SET((float) -3.1453103E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1809450420, PH.base.pack) ;
    p75_y_SET((int32_t)701043208, PH.base.pack) ;
    p75_z_SET((float)2.3115925E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p76_param1_SET((float)1.4003758E38F, PH.base.pack) ;
    p76_param2_SET((float)2.8576266E38F, PH.base.pack) ;
    p76_param3_SET((float)1.6183667E38F, PH.base.pack) ;
    p76_param4_SET((float)1.7537223E37F, PH.base.pack) ;
    p76_param5_SET((float)1.2284279E38F, PH.base.pack) ;
    p76_param6_SET((float)5.511416E37F, PH.base.pack) ;
    p76_param7_SET((float) -5.994752E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)211, &PH) ;
    p77_result_param2_SET((int32_t)16238416, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)247, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)70, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)908026803L, PH.base.pack) ;
    p81_roll_SET((float) -2.6558016E38F, PH.base.pack) ;
    p81_pitch_SET((float) -2.9274654E38F, PH.base.pack) ;
    p81_yaw_SET((float)1.6217272E38F, PH.base.pack) ;
    p81_thrust_SET((float) -2.7596877E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2489428794L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    {
        float  q [] =  {-2.6998652E38F, -2.6494388E38F, 1.401086E38F, 5.944573E37F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)7.954066E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)7.459864E36F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -1.4220866E38F, PH.base.pack) ;
    p82_thrust_SET((float) -2.2870668E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2795939124L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    {
        float  q [] =  {3.3116427E38F, -2.4973096E38F, -1.592219E38F, -2.7182546E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -1.1013182E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -2.2571111E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)2.1618222E37F, PH.base.pack) ;
    p83_thrust_SET((float) -2.8345718E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)4206593448L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)31580, PH.base.pack) ;
    p84_x_SET((float) -2.0019775E38F, PH.base.pack) ;
    p84_y_SET((float) -1.0256919E38F, PH.base.pack) ;
    p84_z_SET((float) -8.081772E37F, PH.base.pack) ;
    p84_vx_SET((float) -2.803328E38F, PH.base.pack) ;
    p84_vy_SET((float)8.513065E36F, PH.base.pack) ;
    p84_vz_SET((float) -6.7126257E37F, PH.base.pack) ;
    p84_afx_SET((float)2.918829E37F, PH.base.pack) ;
    p84_afy_SET((float)7.9263936E36F, PH.base.pack) ;
    p84_afz_SET((float) -2.3024888E38F, PH.base.pack) ;
    p84_yaw_SET((float)1.4324324E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)2.8477718E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)295150383L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)57584, PH.base.pack) ;
    p86_lat_int_SET((int32_t)2067490150, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -1397599547, PH.base.pack) ;
    p86_alt_SET((float)1.9451275E38F, PH.base.pack) ;
    p86_vx_SET((float) -1.7726203E38F, PH.base.pack) ;
    p86_vy_SET((float)1.2765112E38F, PH.base.pack) ;
    p86_vz_SET((float) -2.9663695E38F, PH.base.pack) ;
    p86_afx_SET((float) -2.6566831E38F, PH.base.pack) ;
    p86_afy_SET((float)1.5982203E38F, PH.base.pack) ;
    p86_afz_SET((float)1.3272E38F, PH.base.pack) ;
    p86_yaw_SET((float)1.6375283E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)2.6355184E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)424365745L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)36917, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -1982970214, PH.base.pack) ;
    p87_lon_int_SET((int32_t)1789896119, PH.base.pack) ;
    p87_alt_SET((float)2.9970114E38F, PH.base.pack) ;
    p87_vx_SET((float) -6.921011E37F, PH.base.pack) ;
    p87_vy_SET((float)3.165991E37F, PH.base.pack) ;
    p87_vz_SET((float) -1.7525811E38F, PH.base.pack) ;
    p87_afx_SET((float)2.928474E38F, PH.base.pack) ;
    p87_afy_SET((float)5.803045E37F, PH.base.pack) ;
    p87_afz_SET((float)1.776255E38F, PH.base.pack) ;
    p87_yaw_SET((float)2.4294335E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)2.8549694E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1286459033L, PH.base.pack) ;
    p89_x_SET((float) -1.1978734E38F, PH.base.pack) ;
    p89_y_SET((float)1.343857E38F, PH.base.pack) ;
    p89_z_SET((float) -6.3528797E37F, PH.base.pack) ;
    p89_roll_SET((float)8.542136E37F, PH.base.pack) ;
    p89_pitch_SET((float)1.9695356E38F, PH.base.pack) ;
    p89_yaw_SET((float)2.765523E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)4544800534346053015L, PH.base.pack) ;
    p90_roll_SET((float)6.394619E37F, PH.base.pack) ;
    p90_pitch_SET((float) -8.595819E37F, PH.base.pack) ;
    p90_yaw_SET((float) -1.52766E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)2.9961133E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float)1.0298274E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)1.1777789E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)326451506, PH.base.pack) ;
    p90_lon_SET((int32_t)1111977086, PH.base.pack) ;
    p90_alt_SET((int32_t)313882401, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -18342, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -25920, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -46, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -15745, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)12185, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -6032, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)231209926263920739L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -1.4491105E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.2969268E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.4924826E38F, PH.base.pack) ;
    p91_throttle_SET((float)4.4579697E37F, PH.base.pack) ;
    p91_aux1_SET((float) -1.729639E38F, PH.base.pack) ;
    p91_aux2_SET((float) -2.336883E38F, PH.base.pack) ;
    p91_aux3_SET((float)3.1036656E38F, PH.base.pack) ;
    p91_aux4_SET((float) -1.388987E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)2965573327479572163L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)16195, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)60191, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)63666, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)2530, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)41627, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)38622, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)31589, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)48247, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)40050, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)33218, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)55449, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)64673, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)65676926295904298L, PH.base.pack) ;
    {
        float  controls [] =  {4.4105347E37F, 1.0573242E38F, -3.0691993E38F, 3.1744562E37F, -1.6538062E38F, 1.6366575E38F, 1.3726493E37F, 2.882962E38F, -1.4439553E38F, 1.896236E38F, 7.8985316E37F, 2.9557134E38F, 1.2593575E38F, 2.1207458E38F, -2.9268497E38F, -1.3339956E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)3853101043636121980L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)3738680653772069589L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -14771, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -2997, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)1.7501227E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)2.6463926E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p100_ground_distance_SET((float) -4.683221E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -2.9739059E38F, &PH) ;
    p100_flow_rate_y_SET((float) -3.1790975E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)600153556369485072L, PH.base.pack) ;
    p101_x_SET((float) -3.1092857E38F, PH.base.pack) ;
    p101_y_SET((float)3.8161298E37F, PH.base.pack) ;
    p101_z_SET((float) -2.26798E38F, PH.base.pack) ;
    p101_roll_SET((float) -1.7420637E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.2366671E38F, PH.base.pack) ;
    p101_yaw_SET((float)1.7632945E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)105886602845505433L, PH.base.pack) ;
    p102_x_SET((float) -2.7502866E38F, PH.base.pack) ;
    p102_y_SET((float)3.1153085E38F, PH.base.pack) ;
    p102_z_SET((float) -1.842086E38F, PH.base.pack) ;
    p102_roll_SET((float)2.8507854E38F, PH.base.pack) ;
    p102_pitch_SET((float)2.8599092E38F, PH.base.pack) ;
    p102_yaw_SET((float)2.3492576E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)5556958553556452324L, PH.base.pack) ;
    p103_x_SET((float) -3.218353E38F, PH.base.pack) ;
    p103_y_SET((float) -2.0436465E38F, PH.base.pack) ;
    p103_z_SET((float)3.1119822E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)1215157338841218011L, PH.base.pack) ;
    p104_x_SET((float)2.4757928E38F, PH.base.pack) ;
    p104_y_SET((float) -2.499095E38F, PH.base.pack) ;
    p104_z_SET((float) -5.6123126E36F, PH.base.pack) ;
    p104_roll_SET((float) -3.2448484E38F, PH.base.pack) ;
    p104_pitch_SET((float)4.779737E37F, PH.base.pack) ;
    p104_yaw_SET((float) -1.1327309E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)8992019940231225432L, PH.base.pack) ;
    p105_xacc_SET((float) -8.2756105E37F, PH.base.pack) ;
    p105_yacc_SET((float)1.0247072E38F, PH.base.pack) ;
    p105_zacc_SET((float) -3.2319983E38F, PH.base.pack) ;
    p105_xgyro_SET((float)2.6757676E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -2.6913293E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -3.3610435E38F, PH.base.pack) ;
    p105_xmag_SET((float) -1.1856175E38F, PH.base.pack) ;
    p105_ymag_SET((float) -2.9797561E38F, PH.base.pack) ;
    p105_zmag_SET((float)1.2416719E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -2.2807101E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -6.675178E37F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -2.0514627E38F, PH.base.pack) ;
    p105_temperature_SET((float) -7.3878834E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)28456, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)320667675271710858L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)3722470670L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.714945E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)5.689078E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float)1.4565106E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -2.4531935E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)2.913214E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)13104, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)1509991054L, PH.base.pack) ;
    p106_distance_SET((float) -2.2327363E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)2957267346601387857L, PH.base.pack) ;
    p107_xacc_SET((float)3.4018844E38F, PH.base.pack) ;
    p107_yacc_SET((float) -3.386065E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.6768088E37F, PH.base.pack) ;
    p107_xgyro_SET((float) -3.1131448E38F, PH.base.pack) ;
    p107_ygyro_SET((float)3.3049933E37F, PH.base.pack) ;
    p107_zgyro_SET((float)1.6822245E38F, PH.base.pack) ;
    p107_xmag_SET((float) -6.063789E37F, PH.base.pack) ;
    p107_ymag_SET((float) -1.9685654E38F, PH.base.pack) ;
    p107_zmag_SET((float)1.1061009E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.0936054E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -9.1683324E36F, PH.base.pack) ;
    p107_pressure_alt_SET((float)6.940571E37F, PH.base.pack) ;
    p107_temperature_SET((float) -1.7506622E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2336362678L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.089241E38F, PH.base.pack) ;
    p108_q2_SET((float) -8.524686E37F, PH.base.pack) ;
    p108_q3_SET((float)8.3655676E37F, PH.base.pack) ;
    p108_q4_SET((float) -1.7128776E37F, PH.base.pack) ;
    p108_roll_SET((float)1.3233542E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.475987E37F, PH.base.pack) ;
    p108_yaw_SET((float) -5.8497385E36F, PH.base.pack) ;
    p108_xacc_SET((float) -1.5207712E38F, PH.base.pack) ;
    p108_yacc_SET((float)1.807406E37F, PH.base.pack) ;
    p108_zacc_SET((float)1.4637212E38F, PH.base.pack) ;
    p108_xgyro_SET((float)2.13285E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -9.654191E37F, PH.base.pack) ;
    p108_zgyro_SET((float) -3.814507E37F, PH.base.pack) ;
    p108_lat_SET((float) -3.8975264E37F, PH.base.pack) ;
    p108_lon_SET((float)2.540903E38F, PH.base.pack) ;
    p108_alt_SET((float) -1.5455703E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -1.161728E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)2.7816236E38F, PH.base.pack) ;
    p108_vn_SET((float) -1.6131875E38F, PH.base.pack) ;
    p108_ve_SET((float)2.2523011E38F, PH.base.pack) ;
    p108_vd_SET((float)2.3221073E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)41392, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)7456, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)75, (uint8_t)136, (uint8_t)229, (uint8_t)40, (uint8_t)224, (uint8_t)226, (uint8_t)82, (uint8_t)210, (uint8_t)36, (uint8_t)40, (uint8_t)86, (uint8_t)178, (uint8_t)226, (uint8_t)163, (uint8_t)186, (uint8_t)16, (uint8_t)234, (uint8_t)29, (uint8_t)40, (uint8_t)223, (uint8_t)58, (uint8_t)26, (uint8_t)188, (uint8_t)172, (uint8_t)221, (uint8_t)218, (uint8_t)190, (uint8_t)191, (uint8_t)242, (uint8_t)2, (uint8_t)126, (uint8_t)182, (uint8_t)32, (uint8_t)159, (uint8_t)139, (uint8_t)50, (uint8_t)95, (uint8_t)30, (uint8_t)168, (uint8_t)144, (uint8_t)138, (uint8_t)210, (uint8_t)233, (uint8_t)102, (uint8_t)205, (uint8_t)216, (uint8_t)77, (uint8_t)205, (uint8_t)137, (uint8_t)161, (uint8_t)100, (uint8_t)167, (uint8_t)75, (uint8_t)71, (uint8_t)167, (uint8_t)72, (uint8_t)229, (uint8_t)252, (uint8_t)62, (uint8_t)160, (uint8_t)233, (uint8_t)24, (uint8_t)233, (uint8_t)199, (uint8_t)97, (uint8_t)212, (uint8_t)81, (uint8_t)71, (uint8_t)38, (uint8_t)84, (uint8_t)62, (uint8_t)23, (uint8_t)93, (uint8_t)58, (uint8_t)248, (uint8_t)211, (uint8_t)28, (uint8_t)217, (uint8_t)29, (uint8_t)29, (uint8_t)85, (uint8_t)80, (uint8_t)118, (uint8_t)255, (uint8_t)11, (uint8_t)244, (uint8_t)41, (uint8_t)211, (uint8_t)60, (uint8_t)16, (uint8_t)233, (uint8_t)225, (uint8_t)214, (uint8_t)37, (uint8_t)1, (uint8_t)35, (uint8_t)6, (uint8_t)134, (uint8_t)105, (uint8_t)243, (uint8_t)50, (uint8_t)90, (uint8_t)62, (uint8_t)85, (uint8_t)142, (uint8_t)194, (uint8_t)179, (uint8_t)119, (uint8_t)152, (uint8_t)130, (uint8_t)123, (uint8_t)147, (uint8_t)88, (uint8_t)214, (uint8_t)136, (uint8_t)143, (uint8_t)187, (uint8_t)78, (uint8_t)156, (uint8_t)92, (uint8_t)162, (uint8_t)221, (uint8_t)169, (uint8_t)162, (uint8_t)105, (uint8_t)201, (uint8_t)61, (uint8_t)9, (uint8_t)84, (uint8_t)122, (uint8_t)243, (uint8_t)21, (uint8_t)47, (uint8_t)122, (uint8_t)236, (uint8_t)110, (uint8_t)254, (uint8_t)124, (uint8_t)170, (uint8_t)170, (uint8_t)110, (uint8_t)240, (uint8_t)141, (uint8_t)120, (uint8_t)124, (uint8_t)29, (uint8_t)18, (uint8_t)59, (uint8_t)113, (uint8_t)227, (uint8_t)248, (uint8_t)157, (uint8_t)110, (uint8_t)61, (uint8_t)70, (uint8_t)108, (uint8_t)202, (uint8_t)147, (uint8_t)132, (uint8_t)2, (uint8_t)89, (uint8_t)49, (uint8_t)59, (uint8_t)181, (uint8_t)173, (uint8_t)178, (uint8_t)13, (uint8_t)64, (uint8_t)152, (uint8_t)175, (uint8_t)82, (uint8_t)115, (uint8_t)169, (uint8_t)59, (uint8_t)216, (uint8_t)169, (uint8_t)209, (uint8_t)105, (uint8_t)28, (uint8_t)134, (uint8_t)145, (uint8_t)255, (uint8_t)193, (uint8_t)248, (uint8_t)237, (uint8_t)65, (uint8_t)83, (uint8_t)68, (uint8_t)133, (uint8_t)157, (uint8_t)119, (uint8_t)90, (uint8_t)218, (uint8_t)1, (uint8_t)208, (uint8_t)177, (uint8_t)84, (uint8_t)253, (uint8_t)239, (uint8_t)158, (uint8_t)1, (uint8_t)196, (uint8_t)111, (uint8_t)207, (uint8_t)195, (uint8_t)26, (uint8_t)201, (uint8_t)61, (uint8_t)94, (uint8_t)97, (uint8_t)41, (uint8_t)124, (uint8_t)85, (uint8_t)38, (uint8_t)78, (uint8_t)5, (uint8_t)248, (uint8_t)204, (uint8_t)130, (uint8_t)187, (uint8_t)55, (uint8_t)73, (uint8_t)45, (uint8_t)100, (uint8_t)239, (uint8_t)169, (uint8_t)230, (uint8_t)140, (uint8_t)215, (uint8_t)80, (uint8_t)68, (uint8_t)206, (uint8_t)247, (uint8_t)118, (uint8_t)130, (uint8_t)45, (uint8_t)193, (uint8_t)105, (uint8_t)81, (uint8_t)255, (uint8_t)97, (uint8_t)4, (uint8_t)65, (uint8_t)82, (uint8_t)1, (uint8_t)255, (uint8_t)5, (uint8_t)65, (uint8_t)196, (uint8_t)82, (uint8_t)18};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -6157615492157924977L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -4945600819941238228L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)1008320902814705512L, PH.base.pack) ;
    p112_seq_SET((uint32_t)1665790553L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)7452933373326117010L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p113_lat_SET((int32_t) -894165396, PH.base.pack) ;
    p113_lon_SET((int32_t)1503582981, PH.base.pack) ;
    p113_alt_SET((int32_t) -1318877219, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)5204, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)42544, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)49949, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -5930, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)12714, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)32767, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)22677, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)5845941289196084044L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)235826373L, PH.base.pack) ;
    p114_integrated_x_SET((float) -2.7718884E38F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.4268293E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)7.6611945E34F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)3.2865477E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)1.9891265E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)5256, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)2259054025L, PH.base.pack) ;
    p114_distance_SET((float)2.0705136E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)3241987997728533050L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-1.769362E38F, -1.7347444E38F, 1.7556622E38F, 8.843782E37F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)3.2379871E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -2.5707784E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)8.3807697E37F, PH.base.pack) ;
    p115_lat_SET((int32_t) -197661348, PH.base.pack) ;
    p115_lon_SET((int32_t)1974322556, PH.base.pack) ;
    p115_alt_SET((int32_t)1185778667, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -25359, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)31910, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -30125, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)52439, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)39057, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)26243, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)14711, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -28515, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)1837118960L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -8716, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -9900, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -14412, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)28146, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)20039, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)22336, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)19433, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -2681, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -30517, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)41909, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)27167, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)55802, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)35145, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)19665, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)4013213375L, PH.base.pack) ;
    p118_size_SET((uint32_t)3277677579L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)23942, PH.base.pack) ;
    p119_ofs_SET((uint32_t)4186206277L, PH.base.pack) ;
    p119_count_SET((uint32_t)179321379L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)19702, PH.base.pack) ;
    p120_ofs_SET((uint32_t)1402311991L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)91, (uint8_t)223, (uint8_t)64, (uint8_t)172, (uint8_t)41, (uint8_t)121, (uint8_t)158, (uint8_t)243, (uint8_t)74, (uint8_t)60, (uint8_t)66, (uint8_t)120, (uint8_t)7, (uint8_t)113, (uint8_t)60, (uint8_t)174, (uint8_t)110, (uint8_t)94, (uint8_t)176, (uint8_t)65, (uint8_t)118, (uint8_t)228, (uint8_t)254, (uint8_t)93, (uint8_t)239, (uint8_t)23, (uint8_t)184, (uint8_t)78, (uint8_t)211, (uint8_t)37, (uint8_t)38, (uint8_t)146, (uint8_t)224, (uint8_t)161, (uint8_t)48, (uint8_t)145, (uint8_t)234, (uint8_t)32, (uint8_t)91, (uint8_t)118, (uint8_t)6, (uint8_t)228, (uint8_t)5, (uint8_t)140, (uint8_t)121, (uint8_t)60, (uint8_t)248, (uint8_t)244, (uint8_t)183, (uint8_t)188, (uint8_t)17, (uint8_t)37, (uint8_t)193, (uint8_t)125, (uint8_t)115, (uint8_t)32, (uint8_t)91, (uint8_t)195, (uint8_t)92, (uint8_t)240, (uint8_t)12, (uint8_t)5, (uint8_t)209, (uint8_t)50, (uint8_t)193, (uint8_t)60, (uint8_t)232, (uint8_t)141, (uint8_t)213, (uint8_t)28, (uint8_t)13, (uint8_t)65, (uint8_t)209, (uint8_t)151, (uint8_t)2, (uint8_t)7, (uint8_t)79, (uint8_t)185, (uint8_t)25, (uint8_t)201, (uint8_t)86, (uint8_t)186, (uint8_t)102, (uint8_t)187, (uint8_t)125, (uint8_t)184, (uint8_t)200, (uint8_t)145, (uint8_t)187, (uint8_t)12};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)223, (uint8_t)135, (uint8_t)196, (uint8_t)36, (uint8_t)226, (uint8_t)250, (uint8_t)140, (uint8_t)67, (uint8_t)232, (uint8_t)44, (uint8_t)41, (uint8_t)193, (uint8_t)250, (uint8_t)83, (uint8_t)199, (uint8_t)165, (uint8_t)75, (uint8_t)218, (uint8_t)207, (uint8_t)217, (uint8_t)49, (uint8_t)216, (uint8_t)23, (uint8_t)176, (uint8_t)47, (uint8_t)97, (uint8_t)102, (uint8_t)253, (uint8_t)88, (uint8_t)71, (uint8_t)123, (uint8_t)235, (uint8_t)196, (uint8_t)216, (uint8_t)143, (uint8_t)145, (uint8_t)40, (uint8_t)97, (uint8_t)242, (uint8_t)82, (uint8_t)62, (uint8_t)152, (uint8_t)41, (uint8_t)212, (uint8_t)146, (uint8_t)175, (uint8_t)72, (uint8_t)121, (uint8_t)211, (uint8_t)15, (uint8_t)198, (uint8_t)211, (uint8_t)226, (uint8_t)221, (uint8_t)248, (uint8_t)77, (uint8_t)240, (uint8_t)122, (uint8_t)51, (uint8_t)45, (uint8_t)118, (uint8_t)55, (uint8_t)78, (uint8_t)69, (uint8_t)225, (uint8_t)24, (uint8_t)66, (uint8_t)150, (uint8_t)191, (uint8_t)158, (uint8_t)255, (uint8_t)112, (uint8_t)227, (uint8_t)178, (uint8_t)222, (uint8_t)122, (uint8_t)252, (uint8_t)122, (uint8_t)244, (uint8_t)227, (uint8_t)219, (uint8_t)151, (uint8_t)192, (uint8_t)145, (uint8_t)126, (uint8_t)87, (uint8_t)12, (uint8_t)107, (uint8_t)23, (uint8_t)228, (uint8_t)7, (uint8_t)251, (uint8_t)99, (uint8_t)24, (uint8_t)187, (uint8_t)101, (uint8_t)2, (uint8_t)13, (uint8_t)218, (uint8_t)47, (uint8_t)127, (uint8_t)162, (uint8_t)82, (uint8_t)107, (uint8_t)50, (uint8_t)63, (uint8_t)2, (uint8_t)59, (uint8_t)153, (uint8_t)69};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)381972188019712631L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t) -2073533174, PH.base.pack) ;
    p124_lon_SET((int32_t) -12718545, PH.base.pack) ;
    p124_alt_SET((int32_t) -109471259, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)63612, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)48206, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)6084, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)27966, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)4066245863L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)47873, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)1967, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)31336, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)2473890496L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)18, (uint8_t)246, (uint8_t)57, (uint8_t)201, (uint8_t)59, (uint8_t)230, (uint8_t)23, (uint8_t)230, (uint8_t)99, (uint8_t)65, (uint8_t)97, (uint8_t)215, (uint8_t)244, (uint8_t)202, (uint8_t)104, (uint8_t)230, (uint8_t)132, (uint8_t)72, (uint8_t)34, (uint8_t)47, (uint8_t)2, (uint8_t)45, (uint8_t)49, (uint8_t)51, (uint8_t)123, (uint8_t)164, (uint8_t)126, (uint8_t)105, (uint8_t)235, (uint8_t)142, (uint8_t)242, (uint8_t)72, (uint8_t)35, (uint8_t)16, (uint8_t)124, (uint8_t)39, (uint8_t)221, (uint8_t)31, (uint8_t)42, (uint8_t)34, (uint8_t)151, (uint8_t)7, (uint8_t)192, (uint8_t)18, (uint8_t)35, (uint8_t)231, (uint8_t)110, (uint8_t)30, (uint8_t)229, (uint8_t)97, (uint8_t)192, (uint8_t)214, (uint8_t)132, (uint8_t)206, (uint8_t)39, (uint8_t)37, (uint8_t)151, (uint8_t)153, (uint8_t)111, (uint8_t)221, (uint8_t)36, (uint8_t)152, (uint8_t)70, (uint8_t)40, (uint8_t)119, (uint8_t)136, (uint8_t)230, (uint8_t)39, (uint8_t)183, (uint8_t)236};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)1554943364L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)42792, PH.base.pack) ;
    p127_tow_SET((uint32_t)2426074383L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -2113133520, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -197775466, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)1555872528, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3928310298L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1997680193, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2085336706L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)58698, PH.base.pack) ;
    p128_tow_SET((uint32_t)1833579819L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -39562982, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -2097149010, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)1737192036, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2407514259L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)870997389, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)421752832L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)20540, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)17210, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)5916, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)19735, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -6663, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)14287, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)15986, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -16012, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -7859, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p130_size_SET((uint32_t)4202549101L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)57433, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)4375, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)21889, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)8791, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)231, (uint8_t)58, (uint8_t)29, (uint8_t)16, (uint8_t)179, (uint8_t)198, (uint8_t)145, (uint8_t)230, (uint8_t)42, (uint8_t)157, (uint8_t)41, (uint8_t)155, (uint8_t)233, (uint8_t)124, (uint8_t)154, (uint8_t)51, (uint8_t)207, (uint8_t)205, (uint8_t)108, (uint8_t)237, (uint8_t)59, (uint8_t)103, (uint8_t)126, (uint8_t)188, (uint8_t)214, (uint8_t)152, (uint8_t)150, (uint8_t)27, (uint8_t)234, (uint8_t)255, (uint8_t)163, (uint8_t)68, (uint8_t)167, (uint8_t)102, (uint8_t)100, (uint8_t)36, (uint8_t)145, (uint8_t)142, (uint8_t)113, (uint8_t)128, (uint8_t)239, (uint8_t)240, (uint8_t)11, (uint8_t)98, (uint8_t)242, (uint8_t)1, (uint8_t)7, (uint8_t)246, (uint8_t)86, (uint8_t)182, (uint8_t)55, (uint8_t)213, (uint8_t)38, (uint8_t)72, (uint8_t)97, (uint8_t)172, (uint8_t)71, (uint8_t)55, (uint8_t)213, (uint8_t)24, (uint8_t)177, (uint8_t)120, (uint8_t)201, (uint8_t)241, (uint8_t)173, (uint8_t)239, (uint8_t)235, (uint8_t)49, (uint8_t)239, (uint8_t)199, (uint8_t)115, (uint8_t)233, (uint8_t)165, (uint8_t)212, (uint8_t)226, (uint8_t)243, (uint8_t)164, (uint8_t)172, (uint8_t)23, (uint8_t)129, (uint8_t)120, (uint8_t)157, (uint8_t)82, (uint8_t)60, (uint8_t)186, (uint8_t)211, (uint8_t)238, (uint8_t)72, (uint8_t)231, (uint8_t)18, (uint8_t)133, (uint8_t)13, (uint8_t)23, (uint8_t)23, (uint8_t)10, (uint8_t)143, (uint8_t)194, (uint8_t)116, (uint8_t)66, (uint8_t)250, (uint8_t)28, (uint8_t)251, (uint8_t)20, (uint8_t)41, (uint8_t)4, (uint8_t)37, (uint8_t)187, (uint8_t)6, (uint8_t)105, (uint8_t)193, (uint8_t)57, (uint8_t)58, (uint8_t)211, (uint8_t)62, (uint8_t)179, (uint8_t)1, (uint8_t)151, (uint8_t)213, (uint8_t)223, (uint8_t)142, (uint8_t)35, (uint8_t)94, (uint8_t)109, (uint8_t)68, (uint8_t)5, (uint8_t)73, (uint8_t)4, (uint8_t)73, (uint8_t)208, (uint8_t)223, (uint8_t)234, (uint8_t)179, (uint8_t)160, (uint8_t)207, (uint8_t)127, (uint8_t)98, (uint8_t)243, (uint8_t)67, (uint8_t)54, (uint8_t)149, (uint8_t)46, (uint8_t)104, (uint8_t)85, (uint8_t)130, (uint8_t)72, (uint8_t)203, (uint8_t)0, (uint8_t)135, (uint8_t)57, (uint8_t)210, (uint8_t)108, (uint8_t)55, (uint8_t)17, (uint8_t)37, (uint8_t)159, (uint8_t)201, (uint8_t)210, (uint8_t)36, (uint8_t)64, (uint8_t)0, (uint8_t)83, (uint8_t)61, (uint8_t)246, (uint8_t)55, (uint8_t)222, (uint8_t)168, (uint8_t)3, (uint8_t)101, (uint8_t)232, (uint8_t)157, (uint8_t)58, (uint8_t)244, (uint8_t)73, (uint8_t)53, (uint8_t)34, (uint8_t)217, (uint8_t)237, (uint8_t)77, (uint8_t)33, (uint8_t)242, (uint8_t)217, (uint8_t)30, (uint8_t)228, (uint8_t)126, (uint8_t)208, (uint8_t)182, (uint8_t)76, (uint8_t)65, (uint8_t)208, (uint8_t)253, (uint8_t)221, (uint8_t)110, (uint8_t)152, (uint8_t)250, (uint8_t)88, (uint8_t)15, (uint8_t)197, (uint8_t)205, (uint8_t)80, (uint8_t)23, (uint8_t)140, (uint8_t)133, (uint8_t)125, (uint8_t)134, (uint8_t)241, (uint8_t)54, (uint8_t)119, (uint8_t)249, (uint8_t)99, (uint8_t)194, (uint8_t)195, (uint8_t)16, (uint8_t)113, (uint8_t)114, (uint8_t)25, (uint8_t)113, (uint8_t)151, (uint8_t)252, (uint8_t)248, (uint8_t)210, (uint8_t)128, (uint8_t)89, (uint8_t)220, (uint8_t)222, (uint8_t)48, (uint8_t)37, (uint8_t)60, (uint8_t)213, (uint8_t)51, (uint8_t)124, (uint8_t)222, (uint8_t)132, (uint8_t)46, (uint8_t)214, (uint8_t)186, (uint8_t)179, (uint8_t)127, (uint8_t)214, (uint8_t)23, (uint8_t)202, (uint8_t)166, (uint8_t)136, (uint8_t)153, (uint8_t)189, (uint8_t)69, (uint8_t)250, (uint8_t)178, (uint8_t)145, (uint8_t)140, (uint8_t)228, (uint8_t)160, (uint8_t)104, (uint8_t)237};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2385732161L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)2138, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)24379, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)54425, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1797090995, PH.base.pack) ;
    p133_lon_SET((int32_t) -1934617139, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)37514, PH.base.pack) ;
    p133_mask_SET((uint64_t)5953140793592508281L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -1061250974, PH.base.pack) ;
    p134_lon_SET((int32_t) -961800123, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)57873, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -14679, (int16_t)27230, (int16_t) -8816, (int16_t) -31245, (int16_t) -431, (int16_t)24131, (int16_t)4209, (int16_t)10823, (int16_t) -11663, (int16_t) -6071, (int16_t) -18824, (int16_t) -25047, (int16_t)13115, (int16_t) -7823, (int16_t)5477, (int16_t) -15790};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1501927389, PH.base.pack) ;
    p135_lon_SET((int32_t) -1795420760, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1101036755, PH.base.pack) ;
    p136_lon_SET((int32_t)2099304945, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)24569, PH.base.pack) ;
    p136_terrain_height_SET((float)9.24469E37F, PH.base.pack) ;
    p136_current_height_SET((float)2.779933E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)2064, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)53429, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)2471851565L, PH.base.pack) ;
    p137_press_abs_SET((float) -2.350036E37F, PH.base.pack) ;
    p137_press_diff_SET((float) -1.0771205E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -24696, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)354025022425644082L, PH.base.pack) ;
    {
        float  q [] =  {2.7377422E37F, -1.3651668E38F, -3.3898095E38F, -1.1478446E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)1.7055683E38F, PH.base.pack) ;
    p138_y_SET((float)2.0544408E38F, PH.base.pack) ;
    p138_z_SET((float)9.115703E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)8018418592432130540L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    {
        float  controls [] =  {-9.53209E36F, -1.8229288E38F, 8.633815E37F, 3.2152647E38F, -1.2501127E38F, 2.7664152E38F, -1.4892203E37F, -1.9158332E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)5350397282465304271L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    {
        float  controls [] =  {-8.2099416E37F, 3.5241318E37F, 1.4121083E38F, -7.428774E37F, 2.2964103E38F, -2.3417656E38F, -1.3204681E38F, -9.83616E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)1204021134817276625L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.7412849E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -8.616533E37F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.6140559E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)3.211514E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -1.9775047E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float) -2.59962E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)76, (uint8_t)98, (uint8_t)17, (uint8_t)220, (uint8_t)217, (uint8_t)251, (uint8_t)100, (uint8_t)45, (uint8_t)234, (uint8_t)234, (uint8_t)166, (uint8_t)49, (uint8_t)45, (uint8_t)215, (uint8_t)149, (uint8_t)149, (uint8_t)245, (uint8_t)187, (uint8_t)213, (uint8_t)8, (uint8_t)35, (uint8_t)76, (uint8_t)30, (uint8_t)145, (uint8_t)179, (uint8_t)208, (uint8_t)166, (uint8_t)229, (uint8_t)178, (uint8_t)23, (uint8_t)115, (uint8_t)93, (uint8_t)56, (uint8_t)117, (uint8_t)184, (uint8_t)50, (uint8_t)99, (uint8_t)33, (uint8_t)242, (uint8_t)54, (uint8_t)188, (uint8_t)208, (uint8_t)94, (uint8_t)149, (uint8_t)18, (uint8_t)190, (uint8_t)89, (uint8_t)0, (uint8_t)117, (uint8_t)145, (uint8_t)93, (uint8_t)9, (uint8_t)241, (uint8_t)209, (uint8_t)86, (uint8_t)82, (uint8_t)119, (uint8_t)116, (uint8_t)200, (uint8_t)26, (uint8_t)246, (uint8_t)62, (uint8_t)178, (uint8_t)13, (uint8_t)176, (uint8_t)243, (uint8_t)252, (uint8_t)138, (uint8_t)237, (uint8_t)26, (uint8_t)144, (uint8_t)255, (uint8_t)110, (uint8_t)246, (uint8_t)126, (uint8_t)114, (uint8_t)79, (uint8_t)253, (uint8_t)9, (uint8_t)141, (uint8_t)177, (uint8_t)75, (uint8_t)22, (uint8_t)254, (uint8_t)92, (uint8_t)165, (uint8_t)151, (uint8_t)70, (uint8_t)1, (uint8_t)27, (uint8_t)171, (uint8_t)155, (uint8_t)220, (uint8_t)217, (uint8_t)213, (uint8_t)221, (uint8_t)120, (uint8_t)83, (uint8_t)205, (uint8_t)30, (uint8_t)11, (uint8_t)89, (uint8_t)82, (uint8_t)71, (uint8_t)241, (uint8_t)222, (uint8_t)232, (uint8_t)238, (uint8_t)6, (uint8_t)103, (uint8_t)201, (uint8_t)18, (uint8_t)38, (uint8_t)39, (uint8_t)56, (uint8_t)191, (uint8_t)2, (uint8_t)88, (uint8_t)87, (uint8_t)3};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)245, (uint8_t)204, (uint8_t)175, (uint8_t)253, (uint8_t)73, (uint8_t)195, (uint8_t)9, (uint8_t)26, (uint8_t)119, (uint8_t)118, (uint8_t)185, (uint8_t)98, (uint8_t)161, (uint8_t)2, (uint8_t)168, (uint8_t)222, (uint8_t)46, (uint8_t)149, (uint8_t)224, (uint8_t)76, (uint8_t)69, (uint8_t)169, (uint8_t)2, (uint8_t)200, (uint8_t)141, (uint8_t)245, (uint8_t)252, (uint8_t)95, (uint8_t)17, (uint8_t)19, (uint8_t)28, (uint8_t)229, (uint8_t)92, (uint8_t)187, (uint8_t)65, (uint8_t)66, (uint8_t)13, (uint8_t)149, (uint8_t)129, (uint8_t)181, (uint8_t)38, (uint8_t)122, (uint8_t)239, (uint8_t)36, (uint8_t)128, (uint8_t)132, (uint8_t)142, (uint8_t)166, (uint8_t)85, (uint8_t)11, (uint8_t)67, (uint8_t)7, (uint8_t)52, (uint8_t)80, (uint8_t)52, (uint8_t)128, (uint8_t)152, (uint8_t)132, (uint8_t)168, (uint8_t)78, (uint8_t)136, (uint8_t)207, (uint8_t)53, (uint8_t)144, (uint8_t)16, (uint8_t)13, (uint8_t)149, (uint8_t)9, (uint8_t)101, (uint8_t)107, (uint8_t)233, (uint8_t)126, (uint8_t)23, (uint8_t)201, (uint8_t)159, (uint8_t)181, (uint8_t)201, (uint8_t)22, (uint8_t)239, (uint8_t)207, (uint8_t)172, (uint8_t)142, (uint8_t)212, (uint8_t)110, (uint8_t)116, (uint8_t)24, (uint8_t)46, (uint8_t)119, (uint8_t)156, (uint8_t)199, (uint8_t)134, (uint8_t)205, (uint8_t)253, (uint8_t)247, (uint8_t)47, (uint8_t)12, (uint8_t)125, (uint8_t)41, (uint8_t)101, (uint8_t)170, (uint8_t)44, (uint8_t)174, (uint8_t)181, (uint8_t)9, (uint8_t)61, (uint8_t)176, (uint8_t)234, (uint8_t)66, (uint8_t)49, (uint8_t)210, (uint8_t)23, (uint8_t)38, (uint8_t)91, (uint8_t)61, (uint8_t)85, (uint8_t)213, (uint8_t)233, (uint8_t)166, (uint8_t)125, (uint8_t)120};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)1546947898L, PH.base.pack) ;
    p143_press_abs_SET((float) -1.8264845E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -2.8084245E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -10809, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

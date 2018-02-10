
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
void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_ADSB, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2944528895L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)65376, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)5553, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -26969, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -43, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)14040, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)56964, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)2400, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)3583, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)47064, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)63673, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)3834910481343816692L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)53883911L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)4020818914L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)19489, PH.base.pack) ;
    p3_x_SET((float)3.249076E38F, PH.base.pack) ;
    p3_y_SET((float)1.1709555E38F, PH.base.pack) ;
    p3_z_SET((float) -6.555757E37F, PH.base.pack) ;
    p3_vx_SET((float) -2.5187331E38F, PH.base.pack) ;
    p3_vy_SET((float) -3.259089E38F, PH.base.pack) ;
    p3_vz_SET((float) -1.6784421E38F, PH.base.pack) ;
    p3_afx_SET((float)1.9950064E38F, PH.base.pack) ;
    p3_afy_SET((float)1.0602692E38F, PH.base.pack) ;
    p3_afz_SET((float) -1.9514274E38F, PH.base.pack) ;
    p3_yaw_SET((float) -2.4580195E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)3.2664155E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)7562143473259980638L, PH.base.pack) ;
    p4_seq_SET((uint32_t)508183811L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    {
        char16_t   passkey = "xobwiugtYtCbc";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "lrcmmaxbeonilp";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)773424643L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    {
        char16_t   param_id = "t";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)10568, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "nnrfquam";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)8.718917E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)48860, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)17269, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    {
        char16_t   param_id = "ggqbmgufqxNiage";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -2.0655642E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)2169568532552228799L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p24_lat_SET((int32_t)2090913181, PH.base.pack) ;
    p24_lon_SET((int32_t)466342890, PH.base.pack) ;
    p24_alt_SET((int32_t)229225343, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)34258, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)5154, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)14656, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)10534, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -614444496, &PH) ;
    p24_h_acc_SET((uint32_t)114090501L, &PH) ;
    p24_v_acc_SET((uint32_t)3019631877L, &PH) ;
    p24_vel_acc_SET((uint32_t)152960151L, &PH) ;
    p24_hdg_acc_SET((uint32_t)4132872709L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)115, (uint8_t)137, (uint8_t)48, (uint8_t)67, (uint8_t)230, (uint8_t)215, (uint8_t)46, (uint8_t)69, (uint8_t)58, (uint8_t)114, (uint8_t)64, (uint8_t)57, (uint8_t)134, (uint8_t)43, (uint8_t)97, (uint8_t)193, (uint8_t)182, (uint8_t)137, (uint8_t)174, (uint8_t)127};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)152, (uint8_t)118, (uint8_t)75, (uint8_t)141, (uint8_t)112, (uint8_t)16, (uint8_t)92, (uint8_t)5, (uint8_t)117, (uint8_t)70, (uint8_t)206, (uint8_t)116, (uint8_t)200, (uint8_t)133, (uint8_t)112, (uint8_t)43, (uint8_t)161, (uint8_t)163, (uint8_t)91, (uint8_t)82};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)164, (uint8_t)83, (uint8_t)67, (uint8_t)8, (uint8_t)89, (uint8_t)252, (uint8_t)212, (uint8_t)108, (uint8_t)197, (uint8_t)187, (uint8_t)239, (uint8_t)162, (uint8_t)213, (uint8_t)76, (uint8_t)14, (uint8_t)149, (uint8_t)178, (uint8_t)45, (uint8_t)173, (uint8_t)56};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)111, (uint8_t)49, (uint8_t)170, (uint8_t)61, (uint8_t)84, (uint8_t)111, (uint8_t)108, (uint8_t)49, (uint8_t)6, (uint8_t)220, (uint8_t)230, (uint8_t)200, (uint8_t)184, (uint8_t)239, (uint8_t)23, (uint8_t)235, (uint8_t)41, (uint8_t)250, (uint8_t)178, (uint8_t)217};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)34, (uint8_t)31, (uint8_t)15, (uint8_t)227, (uint8_t)168, (uint8_t)172, (uint8_t)151, (uint8_t)53, (uint8_t)210, (uint8_t)165, (uint8_t)70, (uint8_t)67, (uint8_t)232, (uint8_t)81, (uint8_t)136, (uint8_t)6, (uint8_t)86, (uint8_t)91, (uint8_t)43, (uint8_t)38};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)594382907L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)28974, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t) -23057, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -8222, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -28657, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -30049, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -6046, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)12141, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -536, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -30904, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)1743143274782603261L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)13786, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)13376, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -28015, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)24204, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -16714, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -30848, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)28602, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -27520, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -6152, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)7388017251458273273L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)11321, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -29230, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)18446, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -1957, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2574681731L, PH.base.pack) ;
    p29_press_abs_SET((float)3.3454447E38F, PH.base.pack) ;
    p29_press_diff_SET((float)1.1637158E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)27682, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)600925249L, PH.base.pack) ;
    p30_roll_SET((float) -9.22614E37F, PH.base.pack) ;
    p30_pitch_SET((float) -4.1803215E37F, PH.base.pack) ;
    p30_yaw_SET((float)1.3782587E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)2.2593182E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -1.9999007E37F, PH.base.pack) ;
    p30_yawspeed_SET((float) -5.783546E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)3028966317L, PH.base.pack) ;
    p31_q1_SET((float) -3.2101785E38F, PH.base.pack) ;
    p31_q2_SET((float)1.0040307E38F, PH.base.pack) ;
    p31_q3_SET((float) -1.6176976E38F, PH.base.pack) ;
    p31_q4_SET((float)1.4623741E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -1.2490623E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -1.0889477E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.834406E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)53780256L, PH.base.pack) ;
    p32_x_SET((float)2.9738724E38F, PH.base.pack) ;
    p32_y_SET((float) -2.7876485E38F, PH.base.pack) ;
    p32_z_SET((float)1.4456649E37F, PH.base.pack) ;
    p32_vx_SET((float) -1.7679856E37F, PH.base.pack) ;
    p32_vy_SET((float)2.582112E38F, PH.base.pack) ;
    p32_vz_SET((float)2.0149126E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1449425582L, PH.base.pack) ;
    p33_lat_SET((int32_t) -1209223103, PH.base.pack) ;
    p33_lon_SET((int32_t) -183647704, PH.base.pack) ;
    p33_alt_SET((int32_t)1746875027, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)559845526, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)17221, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -23573, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)16349, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)44397, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)487625057L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)30466, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)8176, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -4150, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)8772, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)21860, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -26964, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)30286, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -21861, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)4228230404L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)63576, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)55413, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)33545, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)53246, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)27003, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)53512, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)41935, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)30534, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)100244543L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)61077, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)30479, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)51493, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)493, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)11852, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)49049, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)58053, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)33460, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)10591, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)28168, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)11458, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)48463, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)21795, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)43531, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)33973, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)13102, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)9696, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -31432, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -20666, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -27497, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)16887, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p39_param1_SET((float) -2.3345729E38F, PH.base.pack) ;
    p39_param2_SET((float) -2.3935324E38F, PH.base.pack) ;
    p39_param3_SET((float)3.015225E37F, PH.base.pack) ;
    p39_param4_SET((float)1.2596503E38F, PH.base.pack) ;
    p39_x_SET((float) -1.2904758E38F, PH.base.pack) ;
    p39_y_SET((float) -1.1059851E38F, PH.base.pack) ;
    p39_z_SET((float)2.5154514E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)54380, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)59991, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)8652, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)7606, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)27206, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p48_latitude_SET((int32_t) -264657916, PH.base.pack) ;
    p48_longitude_SET((int32_t)1470017740, PH.base.pack) ;
    p48_altitude_SET((int32_t)2072052432, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)7556648356502732224L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1509287376, PH.base.pack) ;
    p49_longitude_SET((int32_t)847194413, PH.base.pack) ;
    p49_altitude_SET((int32_t) -240179824, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)4946819852869479342L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    {
        char16_t   param_id = "byiqbbffzwGbnf";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)1556, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p50_param_value0_SET((float) -1.5724729E38F, PH.base.pack) ;
    p50_scale_SET((float) -2.2193631E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -3.1588453E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)9.126468E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)14135, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p54_p1x_SET((float) -2.5904616E38F, PH.base.pack) ;
    p54_p1y_SET((float)4.1093417E37F, PH.base.pack) ;
    p54_p1z_SET((float) -2.2968575E38F, PH.base.pack) ;
    p54_p2x_SET((float)7.1716623E37F, PH.base.pack) ;
    p54_p2y_SET((float) -3.1081884E38F, PH.base.pack) ;
    p54_p2z_SET((float)2.318718E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p55_p1x_SET((float)1.6128431E38F, PH.base.pack) ;
    p55_p1y_SET((float)2.3355213E38F, PH.base.pack) ;
    p55_p1z_SET((float) -2.4595294E38F, PH.base.pack) ;
    p55_p2x_SET((float)2.9002162E38F, PH.base.pack) ;
    p55_p2y_SET((float) -3.3929542E38F, PH.base.pack) ;
    p55_p2z_SET((float) -6.0051124E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)1131079755720440193L, PH.base.pack) ;
    {
        float  q [] =  {-2.793574E38F, -3.380361E38F, -4.6923816E37F, 2.1402633E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)3.15116E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -3.3276031E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)2.8515084E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-5.134611E37F, 3.3259059E38F, -1.8835093E38F, 1.0353877E38F, -2.7208533E37F, 1.840065E38F, 9.255732E37F, 1.7976693E38F, -1.0303858E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)1.7821683E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)1.0305808E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)8672, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -19706, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)40508, PH.base.pack) ;
    p62_alt_error_SET((float)6.43889E37F, PH.base.pack) ;
    p62_aspd_error_SET((float)2.0404729E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -2.8085408E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)1098629511286151619L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t) -190578471, PH.base.pack) ;
    p63_lon_SET((int32_t)166976276, PH.base.pack) ;
    p63_alt_SET((int32_t)292101740, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -358864243, PH.base.pack) ;
    p63_vx_SET((float) -1.5807196E38F, PH.base.pack) ;
    p63_vy_SET((float) -7.641224E37F, PH.base.pack) ;
    p63_vz_SET((float)2.7539745E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-7.9710215E37F, -8.679912E36F, -2.1550695E38F, 2.289151E38F, -2.175413E38F, -5.5300755E37F, -1.0551005E38F, -2.724853E38F, 1.4026256E38F, 2.100951E38F, 2.3910592E38F, -5.0352983E37F, -3.2376861E38F, -2.227751E38F, 6.181233E37F, 3.235589E38F, 1.7949817E37F, 3.0160906E38F, 4.8631554E37F, 1.8211377E38F, -4.1722367E37F, 7.655585E37F, 2.6864007E38F, -2.8469761E38F, -1.0250353E38F, -1.6591628E37F, 1.0916055E38F, -4.4132414E37F, -2.6425126E38F, -6.8911733E37F, 1.109007E38F, -9.395084E37F, 2.0756697E37F, -1.484886E38F, -1.2776397E38F, 1.4136565E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)3425497266663742708L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p64_x_SET((float)7.547833E37F, PH.base.pack) ;
    p64_y_SET((float)3.1786365E38F, PH.base.pack) ;
    p64_z_SET((float)1.1452831E37F, PH.base.pack) ;
    p64_vx_SET((float)2.4953382E38F, PH.base.pack) ;
    p64_vy_SET((float)8.464418E36F, PH.base.pack) ;
    p64_vz_SET((float) -6.853733E37F, PH.base.pack) ;
    p64_ax_SET((float)3.5323867E37F, PH.base.pack) ;
    p64_ay_SET((float)1.3690614E38F, PH.base.pack) ;
    p64_az_SET((float) -2.6930586E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.1491203E38F, 2.7179402E38F, 1.0866415E38F, 2.9388948E38F, -1.3568445E38F, 6.899293E37F, -8.923811E37F, -1.7484263E38F, -2.9470029E38F, -3.1148548E38F, -1.6012035E38F, -8.1948E37F, 1.5364285E38F, 1.3455199E38F, 2.31507E38F, 3.2897064E38F, -2.699435E38F, 3.231146E38F, -1.6690292E37F, -2.8311155E38F, 1.4379815E38F, -2.6700798E38F, -2.6050386E38F, 2.7642294E38F, -3.787466E36F, 2.4869027E38F, -1.9127353E38F, 2.0529236E38F, 3.0745405E38F, 9.942586E37F, -2.1026172E38F, -2.6853663E38F, 1.9018661E38F, -7.8220005E37F, 9.714017E37F, 1.9938381E38F, -6.096901E37F, 3.1629016E38F, 1.8240028E38F, 2.1371999E38F, -4.1166728E37F, 1.3902506E38F, 2.9839449E38F, 3.2618473E38F, -2.4951887E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)626116904L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)61847, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)21999, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)985, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)49097, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)59389, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)34451, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)50406, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)4477, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)28744, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)59205, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)44902, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)24128, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)49011, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)25967, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)59191, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)53726, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)47892, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)56418, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)33504, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)16018, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -8665, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)21032, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -20934, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)30064, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)14741, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)62410, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)21821, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)28869, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)21651, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)5817, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)19302, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)30223, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)47723, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)22441, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p73_param1_SET((float) -8.0514463E37F, PH.base.pack) ;
    p73_param2_SET((float)2.973434E38F, PH.base.pack) ;
    p73_param3_SET((float)3.2220969E38F, PH.base.pack) ;
    p73_param4_SET((float)1.7764683E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -640911175, PH.base.pack) ;
    p73_y_SET((int32_t) -1071738239, PH.base.pack) ;
    p73_z_SET((float) -2.6974128E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -1.1514413E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -2.5483477E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)15659, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)20608, PH.base.pack) ;
    p74_alt_SET((float) -1.7274991E36F, PH.base.pack) ;
    p74_climb_SET((float)7.24275E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p75_param1_SET((float) -3.3022913E38F, PH.base.pack) ;
    p75_param2_SET((float) -2.8288155E38F, PH.base.pack) ;
    p75_param3_SET((float)2.3356438E38F, PH.base.pack) ;
    p75_param4_SET((float) -1.1114913E38F, PH.base.pack) ;
    p75_x_SET((int32_t)389692910, PH.base.pack) ;
    p75_y_SET((int32_t) -13741814, PH.base.pack) ;
    p75_z_SET((float) -1.3000738E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p76_param1_SET((float)3.040321E38F, PH.base.pack) ;
    p76_param2_SET((float)1.5620688E38F, PH.base.pack) ;
    p76_param3_SET((float)1.7212565E38F, PH.base.pack) ;
    p76_param4_SET((float)2.661079E38F, PH.base.pack) ;
    p76_param5_SET((float) -6.6034125E37F, PH.base.pack) ;
    p76_param6_SET((float) -1.6395693E36F, PH.base.pack) ;
    p76_param7_SET((float)3.0514405E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_HOME, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)152, &PH) ;
    p77_result_param2_SET((int32_t)298273572, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)44, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)152, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)1697658365L, PH.base.pack) ;
    p81_roll_SET((float) -5.211202E37F, PH.base.pack) ;
    p81_pitch_SET((float) -1.3959491E38F, PH.base.pack) ;
    p81_yaw_SET((float) -3.2808834E38F, PH.base.pack) ;
    p81_thrust_SET((float) -2.6641275E36F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)1103777532L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    {
        float  q [] =  {1.9680882E38F, -3.0593999E38F, 3.2099177E38F, -2.9640861E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)1.7151655E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)1.1726175E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)2.8614004E38F, PH.base.pack) ;
    p82_thrust_SET((float)9.441925E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)1267613251L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    {
        float  q [] =  {-2.9217E38F, 1.8819086E38F, 2.1555952E38F, -2.8883066E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -1.3006062E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -3.06073E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -8.1798167E37F, PH.base.pack) ;
    p83_thrust_SET((float)2.4808825E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1006333302L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)50306, PH.base.pack) ;
    p84_x_SET((float)2.0563289E38F, PH.base.pack) ;
    p84_y_SET((float) -1.108495E38F, PH.base.pack) ;
    p84_z_SET((float)1.2321399E38F, PH.base.pack) ;
    p84_vx_SET((float)2.4093063E38F, PH.base.pack) ;
    p84_vy_SET((float)1.5397983E38F, PH.base.pack) ;
    p84_vz_SET((float)6.5022206E37F, PH.base.pack) ;
    p84_afx_SET((float) -1.2501633E38F, PH.base.pack) ;
    p84_afy_SET((float)3.0851778E38F, PH.base.pack) ;
    p84_afz_SET((float)1.3825968E38F, PH.base.pack) ;
    p84_yaw_SET((float)3.9726337E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -2.8084884E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1997062914L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)14418, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -581561636, PH.base.pack) ;
    p86_lon_int_SET((int32_t)930468205, PH.base.pack) ;
    p86_alt_SET((float)9.925931E37F, PH.base.pack) ;
    p86_vx_SET((float)1.1794945E37F, PH.base.pack) ;
    p86_vy_SET((float)3.111897E38F, PH.base.pack) ;
    p86_vz_SET((float) -1.6704961E38F, PH.base.pack) ;
    p86_afx_SET((float)3.1899687E38F, PH.base.pack) ;
    p86_afy_SET((float)1.8992728E38F, PH.base.pack) ;
    p86_afz_SET((float) -2.109045E38F, PH.base.pack) ;
    p86_yaw_SET((float) -3.1196592E37F, PH.base.pack) ;
    p86_yaw_rate_SET((float)2.1955992E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)389899300L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)50637, PH.base.pack) ;
    p87_lat_int_SET((int32_t)1767276338, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -191772741, PH.base.pack) ;
    p87_alt_SET((float) -2.8216572E38F, PH.base.pack) ;
    p87_vx_SET((float) -1.9317414E37F, PH.base.pack) ;
    p87_vy_SET((float) -1.3762462E38F, PH.base.pack) ;
    p87_vz_SET((float)3.2239825E38F, PH.base.pack) ;
    p87_afx_SET((float)7.388952E37F, PH.base.pack) ;
    p87_afy_SET((float)1.743983E38F, PH.base.pack) ;
    p87_afz_SET((float)9.828443E37F, PH.base.pack) ;
    p87_yaw_SET((float)1.3799791E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)3.6379782E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)2500643215L, PH.base.pack) ;
    p89_x_SET((float)2.4996614E38F, PH.base.pack) ;
    p89_y_SET((float)1.5014499E38F, PH.base.pack) ;
    p89_z_SET((float) -1.7011127E38F, PH.base.pack) ;
    p89_roll_SET((float) -3.2393038E38F, PH.base.pack) ;
    p89_pitch_SET((float) -4.438219E37F, PH.base.pack) ;
    p89_yaw_SET((float) -2.2216606E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)3866550445802278836L, PH.base.pack) ;
    p90_roll_SET((float)2.6664387E38F, PH.base.pack) ;
    p90_pitch_SET((float)1.2391647E38F, PH.base.pack) ;
    p90_yaw_SET((float) -3.101508E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)4.781822E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -7.0665335E37F, PH.base.pack) ;
    p90_yawspeed_SET((float)1.0046193E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)32969412, PH.base.pack) ;
    p90_lon_SET((int32_t)596563758, PH.base.pack) ;
    p90_alt_SET((int32_t)1658229884, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)8988, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)11776, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -15390, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)4221, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)8920, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)32473, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)1718670370139097559L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)3.9300383E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.8152327E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.3330799E38F, PH.base.pack) ;
    p91_throttle_SET((float)1.475963E38F, PH.base.pack) ;
    p91_aux1_SET((float)1.2454663E38F, PH.base.pack) ;
    p91_aux2_SET((float)1.4146184E38F, PH.base.pack) ;
    p91_aux3_SET((float) -3.2467743E38F, PH.base.pack) ;
    p91_aux4_SET((float) -1.1830079E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)7221347536016757632L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)46970, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)262, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)8940, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)12148, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)39271, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)8038, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)697, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)9984, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)47536, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)44454, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)7518, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)14012, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)2440817470630421498L, PH.base.pack) ;
    {
        float  controls [] =  {2.5448841E37F, -2.4806219E37F, -3.201258E38F, -5.848295E36F, 1.7767934E38F, -2.18377E38F, -6.093237E37F, -6.8344647E37F, -1.5909028E38F, 2.2089504E37F, -2.1694315E38F, 1.6481138E38F, 3.0862475E37F, -1.4815235E38F, -3.0947008E38F, -3.2542374E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)7279865737476253287L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)2384523937985061398L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -17312, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -25294, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -4.805943E36F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)1.0174459E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p100_ground_distance_SET((float)1.294382E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)3.355761E38F, &PH) ;
    p100_flow_rate_y_SET((float)1.1685023E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)5151610772350203842L, PH.base.pack) ;
    p101_x_SET((float) -2.7191649E38F, PH.base.pack) ;
    p101_y_SET((float) -1.2516941E38F, PH.base.pack) ;
    p101_z_SET((float) -2.0675185E37F, PH.base.pack) ;
    p101_roll_SET((float) -1.9085474E38F, PH.base.pack) ;
    p101_pitch_SET((float) -2.8810761E38F, PH.base.pack) ;
    p101_yaw_SET((float)3.2923624E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)2636210363004685026L, PH.base.pack) ;
    p102_x_SET((float) -2.4467979E38F, PH.base.pack) ;
    p102_y_SET((float)1.7902236E38F, PH.base.pack) ;
    p102_z_SET((float) -2.7940104E38F, PH.base.pack) ;
    p102_roll_SET((float) -1.1395148E38F, PH.base.pack) ;
    p102_pitch_SET((float)2.4973244E38F, PH.base.pack) ;
    p102_yaw_SET((float)9.962183E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)2891035313702893727L, PH.base.pack) ;
    p103_x_SET((float) -1.2515086E38F, PH.base.pack) ;
    p103_y_SET((float) -1.9752964E36F, PH.base.pack) ;
    p103_z_SET((float) -6.683635E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)8010343066489080561L, PH.base.pack) ;
    p104_x_SET((float) -1.1126373E38F, PH.base.pack) ;
    p104_y_SET((float) -2.9175338E38F, PH.base.pack) ;
    p104_z_SET((float)2.6009479E38F, PH.base.pack) ;
    p104_roll_SET((float)1.931422E38F, PH.base.pack) ;
    p104_pitch_SET((float)2.175191E38F, PH.base.pack) ;
    p104_yaw_SET((float) -2.2538844E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)3873006290689430140L, PH.base.pack) ;
    p105_xacc_SET((float)9.373739E37F, PH.base.pack) ;
    p105_yacc_SET((float)1.3737401E38F, PH.base.pack) ;
    p105_zacc_SET((float) -5.767599E37F, PH.base.pack) ;
    p105_xgyro_SET((float)1.9605713E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -1.1837731E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -1.7273512E38F, PH.base.pack) ;
    p105_xmag_SET((float) -3.3214863E38F, PH.base.pack) ;
    p105_ymag_SET((float)1.7678067E38F, PH.base.pack) ;
    p105_zmag_SET((float) -6.0560537E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -7.845761E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -1.4845405E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)3.1957717E38F, PH.base.pack) ;
    p105_temperature_SET((float) -2.5223166E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)65258, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)2730659811584543663L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2646816514L, PH.base.pack) ;
    p106_integrated_x_SET((float)3.0289432E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)3.2759462E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.8655597E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -8.966751E36F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -2.0768396E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)7121, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2530593725L, PH.base.pack) ;
    p106_distance_SET((float)2.3201598E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)894100462690190521L, PH.base.pack) ;
    p107_xacc_SET((float) -2.7201786E38F, PH.base.pack) ;
    p107_yacc_SET((float)2.877031E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.6409107E38F, PH.base.pack) ;
    p107_xgyro_SET((float)7.2205784E37F, PH.base.pack) ;
    p107_ygyro_SET((float)9.461998E37F, PH.base.pack) ;
    p107_zgyro_SET((float) -1.8940172E38F, PH.base.pack) ;
    p107_xmag_SET((float) -2.2392955E38F, PH.base.pack) ;
    p107_ymag_SET((float) -2.9437021E38F, PH.base.pack) ;
    p107_zmag_SET((float)3.1754735E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)2.5488965E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -7.145027E36F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -1.6684731E38F, PH.base.pack) ;
    p107_temperature_SET((float) -1.0102546E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)776150293L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.9235133E38F, PH.base.pack) ;
    p108_q2_SET((float) -7.035578E37F, PH.base.pack) ;
    p108_q3_SET((float)2.758116E38F, PH.base.pack) ;
    p108_q4_SET((float) -3.0155755E38F, PH.base.pack) ;
    p108_roll_SET((float)1.7832116E37F, PH.base.pack) ;
    p108_pitch_SET((float) -1.7651584E38F, PH.base.pack) ;
    p108_yaw_SET((float)2.1699415E36F, PH.base.pack) ;
    p108_xacc_SET((float)1.2440001E38F, PH.base.pack) ;
    p108_yacc_SET((float)1.3370993E38F, PH.base.pack) ;
    p108_zacc_SET((float)1.4593614E38F, PH.base.pack) ;
    p108_xgyro_SET((float)9.890596E37F, PH.base.pack) ;
    p108_ygyro_SET((float) -1.1952489E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -4.836422E37F, PH.base.pack) ;
    p108_lat_SET((float)1.742923E38F, PH.base.pack) ;
    p108_lon_SET((float) -3.2557054E38F, PH.base.pack) ;
    p108_alt_SET((float)1.961596E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)3.1832998E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -2.5297402E38F, PH.base.pack) ;
    p108_vn_SET((float)6.0758184E37F, PH.base.pack) ;
    p108_ve_SET((float)1.2639071E38F, PH.base.pack) ;
    p108_vd_SET((float) -1.4547846E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)3630, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)51070, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)54, (uint8_t)104, (uint8_t)212, (uint8_t)109, (uint8_t)216, (uint8_t)56, (uint8_t)101, (uint8_t)8, (uint8_t)246, (uint8_t)141, (uint8_t)77, (uint8_t)125, (uint8_t)6, (uint8_t)103, (uint8_t)62, (uint8_t)4, (uint8_t)240, (uint8_t)48, (uint8_t)31, (uint8_t)104, (uint8_t)85, (uint8_t)38, (uint8_t)10, (uint8_t)80, (uint8_t)190, (uint8_t)139, (uint8_t)107, (uint8_t)158, (uint8_t)209, (uint8_t)203, (uint8_t)206, (uint8_t)65, (uint8_t)70, (uint8_t)93, (uint8_t)94, (uint8_t)223, (uint8_t)88, (uint8_t)44, (uint8_t)166, (uint8_t)226, (uint8_t)64, (uint8_t)252, (uint8_t)7, (uint8_t)29, (uint8_t)97, (uint8_t)23, (uint8_t)45, (uint8_t)8, (uint8_t)45, (uint8_t)145, (uint8_t)251, (uint8_t)83, (uint8_t)127, (uint8_t)174, (uint8_t)151, (uint8_t)80, (uint8_t)197, (uint8_t)205, (uint8_t)35, (uint8_t)224, (uint8_t)94, (uint8_t)47, (uint8_t)183, (uint8_t)10, (uint8_t)161, (uint8_t)101, (uint8_t)110, (uint8_t)238, (uint8_t)161, (uint8_t)43, (uint8_t)138, (uint8_t)182, (uint8_t)208, (uint8_t)167, (uint8_t)202, (uint8_t)157, (uint8_t)254, (uint8_t)195, (uint8_t)91, (uint8_t)62, (uint8_t)140, (uint8_t)47, (uint8_t)164, (uint8_t)59, (uint8_t)158, (uint8_t)239, (uint8_t)166, (uint8_t)75, (uint8_t)39, (uint8_t)0, (uint8_t)42, (uint8_t)1, (uint8_t)196, (uint8_t)16, (uint8_t)62, (uint8_t)32, (uint8_t)116, (uint8_t)102, (uint8_t)147, (uint8_t)125, (uint8_t)153, (uint8_t)92, (uint8_t)102, (uint8_t)42, (uint8_t)121, (uint8_t)67, (uint8_t)30, (uint8_t)14, (uint8_t)62, (uint8_t)97, (uint8_t)20, (uint8_t)162, (uint8_t)175, (uint8_t)244, (uint8_t)238, (uint8_t)3, (uint8_t)121, (uint8_t)44, (uint8_t)234, (uint8_t)119, (uint8_t)160, (uint8_t)139, (uint8_t)189, (uint8_t)85, (uint8_t)208, (uint8_t)201, (uint8_t)96, (uint8_t)200, (uint8_t)180, (uint8_t)180, (uint8_t)73, (uint8_t)90, (uint8_t)3, (uint8_t)229, (uint8_t)122, (uint8_t)106, (uint8_t)209, (uint8_t)227, (uint8_t)216, (uint8_t)237, (uint8_t)199, (uint8_t)21, (uint8_t)210, (uint8_t)195, (uint8_t)40, (uint8_t)239, (uint8_t)15, (uint8_t)130, (uint8_t)2, (uint8_t)149, (uint8_t)238, (uint8_t)159, (uint8_t)107, (uint8_t)41, (uint8_t)42, (uint8_t)208, (uint8_t)67, (uint8_t)14, (uint8_t)50, (uint8_t)87, (uint8_t)199, (uint8_t)244, (uint8_t)238, (uint8_t)133, (uint8_t)150, (uint8_t)111, (uint8_t)63, (uint8_t)194, (uint8_t)66, (uint8_t)229, (uint8_t)240, (uint8_t)89, (uint8_t)61, (uint8_t)68, (uint8_t)210, (uint8_t)78, (uint8_t)151, (uint8_t)137, (uint8_t)180, (uint8_t)223, (uint8_t)167, (uint8_t)251, (uint8_t)136, (uint8_t)99, (uint8_t)147, (uint8_t)127, (uint8_t)241, (uint8_t)165, (uint8_t)232, (uint8_t)80, (uint8_t)95, (uint8_t)135, (uint8_t)32, (uint8_t)194, (uint8_t)80, (uint8_t)236, (uint8_t)13, (uint8_t)23, (uint8_t)19, (uint8_t)200, (uint8_t)187, (uint8_t)169, (uint8_t)7, (uint8_t)58, (uint8_t)148, (uint8_t)232, (uint8_t)239, (uint8_t)155, (uint8_t)241, (uint8_t)68, (uint8_t)190, (uint8_t)93, (uint8_t)177, (uint8_t)29, (uint8_t)248, (uint8_t)118, (uint8_t)3, (uint8_t)75, (uint8_t)25, (uint8_t)48, (uint8_t)162, (uint8_t)48, (uint8_t)127, (uint8_t)148, (uint8_t)201, (uint8_t)158, (uint8_t)11, (uint8_t)148, (uint8_t)154, (uint8_t)180, (uint8_t)131, (uint8_t)33, (uint8_t)144, (uint8_t)45, (uint8_t)55, (uint8_t)124, (uint8_t)115, (uint8_t)125, (uint8_t)18, (uint8_t)231, (uint8_t)49, (uint8_t)240, (uint8_t)34, (uint8_t)181, (uint8_t)245, (uint8_t)109, (uint8_t)53, (uint8_t)239, (uint8_t)124, (uint8_t)190, (uint8_t)46};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -2336366768895657327L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -1750171630545700359L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)3272571008433415167L, PH.base.pack) ;
    p112_seq_SET((uint32_t)2651073514L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)2387975799826107922L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p113_lat_SET((int32_t) -125682529, PH.base.pack) ;
    p113_lon_SET((int32_t) -848395887, PH.base.pack) ;
    p113_alt_SET((int32_t) -1991585366, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)21517, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)9139, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)23205, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -19061, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -10987, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)7792, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)23253, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)8928489191898060013L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3350997580L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.9491765E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)1.2580247E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -1.4107408E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -1.6730114E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -3.3896949E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)18233, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)3535782596L, PH.base.pack) ;
    p114_distance_SET((float)2.1920268E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)4305118953513763853L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {2.5302373E38F, -6.418942E37F, 8.4499667E37F, 1.4955197E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -2.6367907E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)3.9114576E37F, PH.base.pack) ;
    p115_yawspeed_SET((float) -3.110631E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -441290778, PH.base.pack) ;
    p115_lon_SET((int32_t) -887347156, PH.base.pack) ;
    p115_alt_SET((int32_t) -223693707, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)7248, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -11507, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)31958, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)46163, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)53278, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -3639, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)31395, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -23515, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)2103992465L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)22835, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)6317, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -5322, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -30511, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -20217, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)26558, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)9531, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)30209, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)16709, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)55319, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)58592, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)5371, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)8004, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)19111, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)708338374L, PH.base.pack) ;
    p118_size_SET((uint32_t)3999644086L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)55722, PH.base.pack) ;
    p119_ofs_SET((uint32_t)705445225L, PH.base.pack) ;
    p119_count_SET((uint32_t)4070611984L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)35396, PH.base.pack) ;
    p120_ofs_SET((uint32_t)580841649L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)31, (uint8_t)89, (uint8_t)0, (uint8_t)88, (uint8_t)103, (uint8_t)114, (uint8_t)44, (uint8_t)60, (uint8_t)124, (uint8_t)8, (uint8_t)243, (uint8_t)242, (uint8_t)228, (uint8_t)224, (uint8_t)157, (uint8_t)30, (uint8_t)51, (uint8_t)254, (uint8_t)106, (uint8_t)218, (uint8_t)222, (uint8_t)150, (uint8_t)235, (uint8_t)57, (uint8_t)201, (uint8_t)151, (uint8_t)94, (uint8_t)12, (uint8_t)235, (uint8_t)75, (uint8_t)203, (uint8_t)227, (uint8_t)109, (uint8_t)92, (uint8_t)197, (uint8_t)133, (uint8_t)22, (uint8_t)217, (uint8_t)170, (uint8_t)190, (uint8_t)105, (uint8_t)44, (uint8_t)29, (uint8_t)45, (uint8_t)238, (uint8_t)123, (uint8_t)71, (uint8_t)98, (uint8_t)218, (uint8_t)180, (uint8_t)228, (uint8_t)190, (uint8_t)87, (uint8_t)55, (uint8_t)14, (uint8_t)84, (uint8_t)97, (uint8_t)205, (uint8_t)231, (uint8_t)34, (uint8_t)97, (uint8_t)167, (uint8_t)78, (uint8_t)73, (uint8_t)22, (uint8_t)158, (uint8_t)76, (uint8_t)244, (uint8_t)252, (uint8_t)83, (uint8_t)73, (uint8_t)125, (uint8_t)97, (uint8_t)23, (uint8_t)238, (uint8_t)186, (uint8_t)119, (uint8_t)128, (uint8_t)182, (uint8_t)170, (uint8_t)46, (uint8_t)161, (uint8_t)185, (uint8_t)86, (uint8_t)146, (uint8_t)117, (uint8_t)227, (uint8_t)235, (uint8_t)199, (uint8_t)88};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)181, (uint8_t)38, (uint8_t)166, (uint8_t)208, (uint8_t)191, (uint8_t)27, (uint8_t)159, (uint8_t)0, (uint8_t)134, (uint8_t)211, (uint8_t)71, (uint8_t)100, (uint8_t)77, (uint8_t)15, (uint8_t)42, (uint8_t)113, (uint8_t)181, (uint8_t)53, (uint8_t)151, (uint8_t)53, (uint8_t)132, (uint8_t)150, (uint8_t)88, (uint8_t)3, (uint8_t)55, (uint8_t)4, (uint8_t)61, (uint8_t)39, (uint8_t)221, (uint8_t)129, (uint8_t)205, (uint8_t)55, (uint8_t)116, (uint8_t)144, (uint8_t)78, (uint8_t)38, (uint8_t)235, (uint8_t)116, (uint8_t)104, (uint8_t)38, (uint8_t)40, (uint8_t)254, (uint8_t)14, (uint8_t)102, (uint8_t)205, (uint8_t)201, (uint8_t)87, (uint8_t)82, (uint8_t)179, (uint8_t)60, (uint8_t)79, (uint8_t)122, (uint8_t)88, (uint8_t)143, (uint8_t)91, (uint8_t)10, (uint8_t)109, (uint8_t)210, (uint8_t)63, (uint8_t)1, (uint8_t)141, (uint8_t)251, (uint8_t)121, (uint8_t)32, (uint8_t)19, (uint8_t)38, (uint8_t)7, (uint8_t)95, (uint8_t)34, (uint8_t)244, (uint8_t)2, (uint8_t)243, (uint8_t)232, (uint8_t)58, (uint8_t)38, (uint8_t)141, (uint8_t)51, (uint8_t)106, (uint8_t)160, (uint8_t)235, (uint8_t)64, (uint8_t)23, (uint8_t)201, (uint8_t)118, (uint8_t)46, (uint8_t)8, (uint8_t)5, (uint8_t)204, (uint8_t)118, (uint8_t)180, (uint8_t)71, (uint8_t)0, (uint8_t)172, (uint8_t)99, (uint8_t)71, (uint8_t)181, (uint8_t)53, (uint8_t)87, (uint8_t)195, (uint8_t)241, (uint8_t)18, (uint8_t)94, (uint8_t)218, (uint8_t)1, (uint8_t)200, (uint8_t)180, (uint8_t)145, (uint8_t)182, (uint8_t)154, (uint8_t)194};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)4899436152435120604L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p124_lat_SET((int32_t) -574195316, PH.base.pack) ;
    p124_lon_SET((int32_t)604966491, PH.base.pack) ;
    p124_alt_SET((int32_t) -394164425, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)48762, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)24282, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)42040, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)59617, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)3341058728L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)32407, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)35391, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)47585, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1244853830L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)228, (uint8_t)200, (uint8_t)138, (uint8_t)215, (uint8_t)247, (uint8_t)84, (uint8_t)199, (uint8_t)97, (uint8_t)91, (uint8_t)20, (uint8_t)235, (uint8_t)203, (uint8_t)25, (uint8_t)74, (uint8_t)209, (uint8_t)131, (uint8_t)147, (uint8_t)209, (uint8_t)215, (uint8_t)156, (uint8_t)94, (uint8_t)129, (uint8_t)254, (uint8_t)65, (uint8_t)203, (uint8_t)193, (uint8_t)254, (uint8_t)169, (uint8_t)240, (uint8_t)197, (uint8_t)24, (uint8_t)90, (uint8_t)96, (uint8_t)212, (uint8_t)49, (uint8_t)244, (uint8_t)23, (uint8_t)81, (uint8_t)178, (uint8_t)37, (uint8_t)27, (uint8_t)243, (uint8_t)165, (uint8_t)242, (uint8_t)165, (uint8_t)58, (uint8_t)129, (uint8_t)25, (uint8_t)184, (uint8_t)93, (uint8_t)202, (uint8_t)119, (uint8_t)38, (uint8_t)254, (uint8_t)110, (uint8_t)232, (uint8_t)116, (uint8_t)148, (uint8_t)26, (uint8_t)233, (uint8_t)19, (uint8_t)35, (uint8_t)87, (uint8_t)114, (uint8_t)57, (uint8_t)44, (uint8_t)254, (uint8_t)167, (uint8_t)18, (uint8_t)105};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)1700985630L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)56623, PH.base.pack) ;
    p127_tow_SET((uint32_t)809288006L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1533729724, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)436072584, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -1562988949, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)1296079052L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -2038958444, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)4216912195L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)27506, PH.base.pack) ;
    p128_tow_SET((uint32_t)2838460343L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -220276002, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -613523257, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)510609370, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2130153438L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -293592100, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2141028979L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -22439, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)24703, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)15382, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)5434, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -15980, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -22407, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -24064, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -5373, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)7344, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p130_size_SET((uint32_t)3124851653L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)14136, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)6005, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)28884, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)39712, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)214, (uint8_t)167, (uint8_t)36, (uint8_t)97, (uint8_t)74, (uint8_t)21, (uint8_t)138, (uint8_t)92, (uint8_t)42, (uint8_t)129, (uint8_t)246, (uint8_t)122, (uint8_t)139, (uint8_t)189, (uint8_t)238, (uint8_t)19, (uint8_t)115, (uint8_t)35, (uint8_t)211, (uint8_t)64, (uint8_t)107, (uint8_t)56, (uint8_t)115, (uint8_t)3, (uint8_t)122, (uint8_t)87, (uint8_t)81, (uint8_t)169, (uint8_t)223, (uint8_t)174, (uint8_t)48, (uint8_t)245, (uint8_t)223, (uint8_t)232, (uint8_t)147, (uint8_t)83, (uint8_t)26, (uint8_t)42, (uint8_t)167, (uint8_t)107, (uint8_t)141, (uint8_t)243, (uint8_t)96, (uint8_t)229, (uint8_t)224, (uint8_t)0, (uint8_t)54, (uint8_t)39, (uint8_t)19, (uint8_t)138, (uint8_t)181, (uint8_t)25, (uint8_t)142, (uint8_t)147, (uint8_t)28, (uint8_t)172, (uint8_t)2, (uint8_t)249, (uint8_t)176, (uint8_t)254, (uint8_t)233, (uint8_t)182, (uint8_t)78, (uint8_t)208, (uint8_t)185, (uint8_t)174, (uint8_t)213, (uint8_t)35, (uint8_t)26, (uint8_t)157, (uint8_t)15, (uint8_t)159, (uint8_t)215, (uint8_t)243, (uint8_t)223, (uint8_t)70, (uint8_t)61, (uint8_t)148, (uint8_t)128, (uint8_t)138, (uint8_t)232, (uint8_t)217, (uint8_t)253, (uint8_t)247, (uint8_t)11, (uint8_t)77, (uint8_t)42, (uint8_t)43, (uint8_t)19, (uint8_t)174, (uint8_t)148, (uint8_t)33, (uint8_t)211, (uint8_t)203, (uint8_t)155, (uint8_t)49, (uint8_t)237, (uint8_t)236, (uint8_t)84, (uint8_t)252, (uint8_t)131, (uint8_t)29, (uint8_t)208, (uint8_t)190, (uint8_t)193, (uint8_t)116, (uint8_t)120, (uint8_t)117, (uint8_t)82, (uint8_t)241, (uint8_t)197, (uint8_t)3, (uint8_t)206, (uint8_t)228, (uint8_t)131, (uint8_t)96, (uint8_t)103, (uint8_t)163, (uint8_t)141, (uint8_t)199, (uint8_t)35, (uint8_t)134, (uint8_t)230, (uint8_t)157, (uint8_t)116, (uint8_t)164, (uint8_t)240, (uint8_t)151, (uint8_t)124, (uint8_t)227, (uint8_t)203, (uint8_t)7, (uint8_t)9, (uint8_t)237, (uint8_t)71, (uint8_t)128, (uint8_t)179, (uint8_t)68, (uint8_t)125, (uint8_t)96, (uint8_t)173, (uint8_t)25, (uint8_t)163, (uint8_t)241, (uint8_t)197, (uint8_t)238, (uint8_t)48, (uint8_t)208, (uint8_t)55, (uint8_t)231, (uint8_t)151, (uint8_t)162, (uint8_t)82, (uint8_t)100, (uint8_t)144, (uint8_t)32, (uint8_t)202, (uint8_t)176, (uint8_t)206, (uint8_t)129, (uint8_t)160, (uint8_t)41, (uint8_t)87, (uint8_t)211, (uint8_t)235, (uint8_t)34, (uint8_t)9, (uint8_t)84, (uint8_t)161, (uint8_t)252, (uint8_t)125, (uint8_t)90, (uint8_t)101, (uint8_t)13, (uint8_t)123, (uint8_t)24, (uint8_t)171, (uint8_t)24, (uint8_t)206, (uint8_t)76, (uint8_t)179, (uint8_t)69, (uint8_t)226, (uint8_t)53, (uint8_t)210, (uint8_t)16, (uint8_t)107, (uint8_t)180, (uint8_t)130, (uint8_t)205, (uint8_t)124, (uint8_t)5, (uint8_t)19, (uint8_t)103, (uint8_t)200, (uint8_t)123, (uint8_t)150, (uint8_t)113, (uint8_t)66, (uint8_t)70, (uint8_t)132, (uint8_t)2, (uint8_t)187, (uint8_t)35, (uint8_t)103, (uint8_t)102, (uint8_t)76, (uint8_t)32, (uint8_t)230, (uint8_t)88, (uint8_t)225, (uint8_t)194, (uint8_t)125, (uint8_t)39, (uint8_t)243, (uint8_t)153, (uint8_t)88, (uint8_t)136, (uint8_t)36, (uint8_t)73, (uint8_t)84, (uint8_t)22, (uint8_t)175, (uint8_t)185, (uint8_t)202, (uint8_t)236, (uint8_t)162, (uint8_t)163, (uint8_t)12, (uint8_t)115, (uint8_t)200, (uint8_t)96, (uint8_t)198, (uint8_t)218, (uint8_t)103, (uint8_t)190, (uint8_t)11, (uint8_t)172, (uint8_t)69, (uint8_t)197, (uint8_t)112, (uint8_t)39, (uint8_t)105, (uint8_t)11, (uint8_t)200, (uint8_t)217, (uint8_t)80, (uint8_t)206, (uint8_t)195, (uint8_t)224, (uint8_t)50, (uint8_t)103, (uint8_t)108};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1435058233L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)14407, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)4612, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)13445, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_135, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1157749777, PH.base.pack) ;
    p133_lon_SET((int32_t)581379865, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)58771, PH.base.pack) ;
    p133_mask_SET((uint64_t)7295214799308492156L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)352213423, PH.base.pack) ;
    p134_lon_SET((int32_t) -1525595245, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)22174, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)9267, (int16_t)30129, (int16_t)23816, (int16_t)5807, (int16_t)25635, (int16_t)20180, (int16_t)28301, (int16_t) -21879, (int16_t) -2025, (int16_t)15528, (int16_t)6775, (int16_t)31136, (int16_t) -8757, (int16_t)26964, (int16_t)28666, (int16_t) -3304};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)2011725076, PH.base.pack) ;
    p135_lon_SET((int32_t)797316266, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1292446652, PH.base.pack) ;
    p136_lon_SET((int32_t)573823557, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)23989, PH.base.pack) ;
    p136_terrain_height_SET((float) -1.4528661E38F, PH.base.pack) ;
    p136_current_height_SET((float) -3.2759702E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)60308, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)36869, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)614914022L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.0776502E38F, PH.base.pack) ;
    p137_press_diff_SET((float)1.4877604E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -21891, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)2440245417889663929L, PH.base.pack) ;
    {
        float  q [] =  {1.6609018E37F, 2.259885E38F, -1.593599E38F, 2.1953745E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)2.8640277E38F, PH.base.pack) ;
    p138_y_SET((float) -1.196745E38F, PH.base.pack) ;
    p138_z_SET((float)6.895503E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}


#pragma once
#include "DemoDevice.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

INLINER int32_t Arrays_equals(uint8_t * L, uint8_t * R, int32_t bytes)
{
    for(int32_t i = 0; i < bytes; i++) if(L[i] != R[i]) return i;
    return -1;
}
static Pack * zero2empty(Pack * pack)
{
    pack = realloc(pack, sizeof(Pack) + pack->meta->packMinBytes);
    for(int32_t i = pack->meta->packMinBytes; -1 < --i;)pack->data[i] = 0; //clean bytes
    return pack;
}
void failure(Channel * ch, int32_t id, int32_t arg) {assert(false);}




void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_STANDBY);
    assert(p0_custom_mode_GET(pack) == (uint32_t)3617508889L);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_PARAFOIL);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)11);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)30051);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)60072);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)36551);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)15738);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)30484);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)16335);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)26092);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)34565);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)48120);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)7455903056027467077L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)3483604726L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_afy_GET(pack) == (float) -1.0408845E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)29113);
    assert(p3_yaw_GET(pack) == (float)7.614097E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3764808958L);
    assert(p3_y_GET(pack) == (float) -8.4528985E37F);
    assert(p3_afz_GET(pack) == (float) -3.0350421E38F);
    assert(p3_vy_GET(pack) == (float) -2.2582429E38F);
    assert(p3_afx_GET(pack) == (float) -4.3503096E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p3_vx_GET(pack) == (float)3.7031314E37F);
    assert(p3_vz_GET(pack) == (float)1.9146382E37F);
    assert(p3_yaw_rate_GET(pack) == (float)1.4493706E38F);
    assert(p3_x_GET(pack) == (float) -2.8317927E38F);
    assert(p3_z_GET(pack) == (float)1.6214277E38F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)6016755486125480140L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p4_seq_GET(pack) == (uint32_t)1428375337L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p5_passkey_LEN(ph) == 7);
    {
        char16_t * exemplary = u"mqskttq";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)7);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 16);
    {
        char16_t * exemplary = u"oelpiKydKKpbqtep";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)4125735860L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)46);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)21043);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p20_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"dgXge";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8);
    assert(p22_param_value_GET(pack) == (float) -7.6812086E37F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)58412);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)42015);
    assert(p22_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"oJwafVgglF";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"mTjnqPeu";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float) -3.3026468E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)48238);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)16182);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1099497925L);
    assert(p24_time_usec_GET(pack) == (uint64_t)1577532863152499062L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p24_alt_GET(pack) == (int32_t)533595427);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2366603790L);
    assert(p24_lat_GET(pack) == (int32_t) -1252323868);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)528295584L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1699637358);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)38631);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1910064237L);
    assert(p24_lon_GET(pack) == (int32_t) -1123758319);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)50839);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)75);
    {
        uint8_t exemplary[] =  {(uint8_t)49, (uint8_t)15, (uint8_t)195, (uint8_t)106, (uint8_t)234, (uint8_t)177, (uint8_t)29, (uint8_t)13, (uint8_t)167, (uint8_t)109, (uint8_t)30, (uint8_t)162, (uint8_t)232, (uint8_t)203, (uint8_t)102, (uint8_t)168, (uint8_t)227, (uint8_t)84, (uint8_t)218, (uint8_t)2} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)0, (uint8_t)12, (uint8_t)150, (uint8_t)33, (uint8_t)42, (uint8_t)33, (uint8_t)151, (uint8_t)20, (uint8_t)4, (uint8_t)203, (uint8_t)53, (uint8_t)101, (uint8_t)129, (uint8_t)234, (uint8_t)159, (uint8_t)92, (uint8_t)198, (uint8_t)29, (uint8_t)126, (uint8_t)219} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)69, (uint8_t)201, (uint8_t)204, (uint8_t)229, (uint8_t)71, (uint8_t)52, (uint8_t)32, (uint8_t)92, (uint8_t)224, (uint8_t)233, (uint8_t)99, (uint8_t)192, (uint8_t)215, (uint8_t)117, (uint8_t)106, (uint8_t)199, (uint8_t)138, (uint8_t)230, (uint8_t)100, (uint8_t)199} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)168, (uint8_t)249, (uint8_t)100, (uint8_t)84, (uint8_t)24, (uint8_t)139, (uint8_t)147, (uint8_t)247, (uint8_t)194, (uint8_t)223, (uint8_t)248, (uint8_t)164, (uint8_t)111, (uint8_t)83, (uint8_t)249, (uint8_t)110, (uint8_t)105, (uint8_t)10, (uint8_t)32, (uint8_t)125} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)167, (uint8_t)20, (uint8_t)238, (uint8_t)96, (uint8_t)129, (uint8_t)147, (uint8_t)190, (uint8_t)131, (uint8_t)184, (uint8_t)48, (uint8_t)1, (uint8_t)18, (uint8_t)82, (uint8_t)196, (uint8_t)255, (uint8_t)141, (uint8_t)38, (uint8_t)182, (uint8_t)230, (uint8_t)134} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -20988);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)4067460867L);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -24363);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)14279);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)11809);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)19059);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -8275);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -12380);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -22419);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -4536);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)4206);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)6868);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -26605);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -7580);
    assert(p27_time_usec_GET(pack) == (uint64_t)3898387824519353061L);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -24579);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)14643);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)8861);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)31576);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)21793);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)10121);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -17583);
    assert(p28_time_usec_GET(pack) == (uint64_t)6933831594040421553L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)8206);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)23813);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float)1.4726469E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)4006652409L);
    assert(p29_press_diff_GET(pack) == (float) -2.1091337E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)14315);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3227982141L);
    assert(p30_rollspeed_GET(pack) == (float)1.6320244E38F);
    assert(p30_pitchspeed_GET(pack) == (float)2.075219E38F);
    assert(p30_yawspeed_GET(pack) == (float) -9.4642E37F);
    assert(p30_pitch_GET(pack) == (float)2.7141961E38F);
    assert(p30_yaw_GET(pack) == (float) -7.508522E37F);
    assert(p30_roll_GET(pack) == (float) -1.1657487E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float)2.733698E38F);
    assert(p31_q1_GET(pack) == (float) -2.825359E37F);
    assert(p31_pitchspeed_GET(pack) == (float) -2.0527486E38F);
    assert(p31_yawspeed_GET(pack) == (float) -3.7193013E37F);
    assert(p31_rollspeed_GET(pack) == (float) -8.632634E37F);
    assert(p31_q4_GET(pack) == (float) -1.0680295E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2526476201L);
    assert(p31_q2_GET(pack) == (float)5.2415704E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vx_GET(pack) == (float)1.6011387E38F);
    assert(p32_x_GET(pack) == (float)7.701238E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2820412127L);
    assert(p32_z_GET(pack) == (float)2.8486131E38F);
    assert(p32_vz_GET(pack) == (float) -1.9379143E38F);
    assert(p32_vy_GET(pack) == (float)7.4436195E37F);
    assert(p32_y_GET(pack) == (float)1.6163105E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lat_GET(pack) == (int32_t)1844865853);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)39130);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)10896);
    assert(p33_relative_alt_GET(pack) == (int32_t)1463293013);
    assert(p33_alt_GET(pack) == (int32_t) -2116483046);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -18210);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)3393130207L);
    assert(p33_lon_GET(pack) == (int32_t) -1476077417);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)30820);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -26475);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -456);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -26819);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)23449);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)1404);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)17074);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)19027);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2880283449L);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -608);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)638735234L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)16624);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)560);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)59645);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)47780);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)55581);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)37378);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)23438);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)12933);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)42586);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)44946);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)57279);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)5565);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)49516);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)36829);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)65099);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)3250);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)16113);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)26734);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)50737);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)9892);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)64442);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)46875);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)10332);
    assert(p36_time_usec_GET(pack) == (uint32_t)1212007112L);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)40970);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)31727);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)29352);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)6593);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -14597);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)19343);
    assert(p39_y_GET(pack) == (float) -8.810614E36F);
    assert(p39_z_GET(pack) == (float) -2.5521435E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p39_param4_GET(pack) == (float) -3.1340144E38F);
    assert(p39_x_GET(pack) == (float) -4.1689068E37F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p39_param3_GET(pack) == (float) -1.1451857E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_param2_GET(pack) == (float) -2.172295E38F);
    assert(p39_param1_GET(pack) == (float)3.7062123E37F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)41474);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)49201);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)36852);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)54055);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)22983);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)41);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_longitude_GET(pack) == (int32_t) -222548970);
    assert(p48_latitude_GET(pack) == (int32_t) -1122936066);
    assert(p48_time_usec_TRY(ph) == (uint64_t)8740216364329059295L);
    assert(p48_altitude_GET(pack) == (int32_t)1706791551);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_altitude_GET(pack) == (int32_t) -1162562093);
    assert(p49_latitude_GET(pack) == (int32_t)1839456416);
    assert(p49_time_usec_TRY(ph) == (uint64_t)7444961138158831020L);
    assert(p49_longitude_GET(pack) == (int32_t)1387480659);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p50_param_value_min_GET(pack) == (float) -2.0173343E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)25538);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p50_param_value0_GET(pack) == (float) -3.1013112E38F);
    assert(p50_scale_GET(pack) == (float) -6.2502557E37F);
    assert(p50_param_value_max_GET(pack) == (float) -2.951122E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p50_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"jlverfXbhcbUv";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)59596);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)89);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2y_GET(pack) == (float) -1.9521834E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p54_p2x_GET(pack) == (float)1.2959997E38F);
    assert(p54_p1y_GET(pack) == (float)1.3460432E38F);
    assert(p54_p1z_GET(pack) == (float)1.6721672E38F);
    assert(p54_p1x_GET(pack) == (float)2.994492E38F);
    assert(p54_p2z_GET(pack) == (float) -1.6964131E37F);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2y_GET(pack) == (float) -3.1619782E38F);
    assert(p55_p1x_GET(pack) == (float) -3.126215E37F);
    assert(p55_p1y_GET(pack) == (float) -1.3039103E38F);
    assert(p55_p2z_GET(pack) == (float)1.3000434E38F);
    assert(p55_p2x_GET(pack) == (float) -2.300767E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p55_p1z_GET(pack) == (float)1.7503173E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float) -1.2578748E38F);
    assert(p61_pitchspeed_GET(pack) == (float)9.989793E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)2027791967211918421L);
    assert(p61_rollspeed_GET(pack) == (float)2.0019258E38F);
    {
        float exemplary[] =  {-1.108803E38F, 1.7025884E38F, -1.3794083E38F, -2.9264087E38F, 2.8564018E38F, 1.7094767E38F, 3.0830956E38F, -6.8458984E37F, -2.7950333E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.391345E37F, 1.0572284E38F, -2.0668305E38F, -3.1345902E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -9793);
    assert(p62_nav_roll_GET(pack) == (float)1.1504797E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)33219);
    assert(p62_aspd_error_GET(pack) == (float)7.2696816E37F);
    assert(p62_alt_error_GET(pack) == (float) -3.0891848E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)25514);
    assert(p62_nav_pitch_GET(pack) == (float)2.6911632E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -1.7783272E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_relative_alt_GET(pack) == (int32_t) -367647136);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_lon_GET(pack) == (int32_t)1034173399);
    {
        float exemplary[] =  {3.0005324E38F, -3.0656065E38F, -2.9412519E37F, 2.902984E38F, 3.1711854E38F, -1.7256475E38F, 2.1323828E38F, -2.5616367E38F, 4.4682757E37F, -3.1932474E38F, 7.307989E37F, -3.345869E38F, -2.0500587E38F, -1.4007778E36F, 2.921074E38F, -7.323332E37F, -2.9377312E38F, 2.6275397E38F, -2.6006448E38F, 1.7188746E38F, 2.1920119E38F, -2.053355E38F, 2.3375518E38F, -2.1780103E38F, -2.9451363E38F, -2.4323698E38F, -2.398968E38F, 5.151753E37F, -3.1523953E38F, -1.4156894E38F, -3.0402728E38F, 2.7337733E38F, 2.3061244E38F, 2.4130135E38F, 5.109266E37F, -3.3395766E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vy_GET(pack) == (float)9.539879E37F);
    assert(p63_time_usec_GET(pack) == (uint64_t)6391039618137430672L);
    assert(p63_vx_GET(pack) == (float)7.23833E37F);
    assert(p63_alt_GET(pack) == (int32_t)298516236);
    assert(p63_vz_GET(pack) == (float)3.2373557E38F);
    assert(p63_lat_GET(pack) == (int32_t)1986298089);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_x_GET(pack) == (float) -2.8144652E38F);
    assert(p64_az_GET(pack) == (float) -1.1913441E38F);
    assert(p64_y_GET(pack) == (float)2.9551106E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)4587342838889156421L);
    assert(p64_ay_GET(pack) == (float) -4.785444E37F);
    {
        float exemplary[] =  {1.6568051E37F, -1.6586529E37F, 3.0643654E38F, -2.4797596E38F, 4.9344618E36F, -8.842301E37F, -3.363545E38F, 8.612128E37F, 1.0828075E38F, 1.283671E37F, -1.9150817E38F, -1.8925212E38F, -5.1305957E37F, -1.0551119E38F, 1.2599584E38F, -2.3406039E38F, 2.0024855E38F, -1.7743545E38F, 2.905582E38F, 9.027283E37F, 1.4216638E38F, -2.4418725E38F, 7.277873E36F, 1.0867775E38F, -5.273786E37F, 3.9209204E37F, 4.0969887E37F, 5.4108733E37F, 2.4326909E38F, 1.058768E38F, 2.1807606E38F, -3.5542456E37F, 2.2019835E38F, -2.660663E38F, -1.9945774E38F, -1.8458814E38F, 1.3935153E38F, -3.0834426E38F, 3.017297E38F, -1.177706E38F, 6.801151E37F, 1.0918232E38F, -1.3379232E38F, -2.5226387E38F, 3.3453618E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_z_GET(pack) == (float)3.0782433E38F);
    assert(p64_vy_GET(pack) == (float)2.9969668E38F);
    assert(p64_vz_GET(pack) == (float)1.4863218E37F);
    assert(p64_vx_GET(pack) == (float)2.8738609E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p64_ax_GET(pack) == (float)1.7176637E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)52762);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)3247);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)2343);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)63737);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)23123);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)49913);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)53372);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)6990);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)40691);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)688245098L);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)50387);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)65023);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)32269);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)52540);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)5540);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)29725);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)27583);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)882);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)55756);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)57269);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)32544);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)16549);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -25459);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)5206);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -21095);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -16423);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)12093);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)55092);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)20233);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)37973);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)25928);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)4347);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)59272);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)49695);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_x_GET(pack) == (int32_t) -567850687);
    assert(p73_param3_GET(pack) == (float)2.835644E38F);
    assert(p73_y_GET(pack) == (int32_t)1523056048);
    assert(p73_param1_GET(pack) == (float)1.461516E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p73_param4_GET(pack) == (float) -3.213096E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)6634);
    assert(p73_param2_GET(pack) == (float)1.5421401E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p73_z_GET(pack) == (float)8.589111E37F);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_climb_GET(pack) == (float)3.3127785E38F);
    assert(p74_groundspeed_GET(pack) == (float) -1.9747439E38F);
    assert(p74_airspeed_GET(pack) == (float)3.1280617E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -27810);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)62915);
    assert(p74_alt_GET(pack) == (float) -2.3465373E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p75_param2_GET(pack) == (float) -4.443308E37F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY);
    assert(p75_x_GET(pack) == (int32_t)1787578862);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p75_z_GET(pack) == (float) -1.0416443E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p75_y_GET(pack) == (int32_t) -676236502);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p75_param3_GET(pack) == (float)3.338227E38F);
    assert(p75_param1_GET(pack) == (float)2.4395569E38F);
    assert(p75_param4_GET(pack) == (float)2.434049E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p76_param2_GET(pack) == (float) -3.236385E38F);
    assert(p76_param4_GET(pack) == (float)3.292259E38F);
    assert(p76_param3_GET(pack) == (float) -1.102308E38F);
    assert(p76_param7_GET(pack) == (float) -3.242351E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_MISSION_START);
    assert(p76_param6_GET(pack) == (float)2.9327314E38F);
    assert(p76_param5_GET(pack) == (float)3.3381366E38F);
    assert(p76_param1_GET(pack) == (float)2.257781E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH);
    assert(p77_result_param2_TRY(ph) == (int32_t)24463716);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)157);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)222);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)110);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float) -2.8416605E38F);
    assert(p81_yaw_GET(pack) == (float) -2.923228E38F);
    assert(p81_pitch_GET(pack) == (float) -2.3800865E37F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1682517381L);
    assert(p81_roll_GET(pack) == (float)2.188377E38F);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2500219612L);
    {
        float exemplary[] =  {2.4748552E38F, 6.0566556E37F, 2.536195E38F, 1.929861E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_yaw_rate_GET(pack) == (float) -3.2221007E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float)9.893436E37F);
    assert(p82_thrust_GET(pack) == (float) -4.220465E37F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p82_body_roll_rate_GET(pack) == (float) -1.3796622E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float) -1.8613275E38F);
    {
        float exemplary[] =  {1.7721518E38F, 3.0902537E38F, 2.87777E38F, -9.584393E36F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_roll_rate_GET(pack) == (float) -2.4911162E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.0433104E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)2672265594L);
    assert(p83_body_pitch_rate_GET(pack) == (float)2.2451112E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_rate_GET(pack) == (float)1.1441252E38F);
    assert(p84_vz_GET(pack) == (float)1.0527862E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3796226928L);
    assert(p84_afy_GET(pack) == (float) -2.5444062E38F);
    assert(p84_afx_GET(pack) == (float)2.843456E38F);
    assert(p84_vx_GET(pack) == (float) -2.0397658E38F);
    assert(p84_y_GET(pack) == (float) -1.6850334E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)61493);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p84_yaw_GET(pack) == (float) -2.359963E38F);
    assert(p84_afz_GET(pack) == (float) -1.4492079E38F);
    assert(p84_z_GET(pack) == (float) -2.6593703E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p84_x_GET(pack) == (float) -1.3308509E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p84_vy_GET(pack) == (float) -4.5610284E36F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_afx_GET(pack) == (float) -2.0282502E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)30554);
    assert(p86_afz_GET(pack) == (float) -2.3382773E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)784125164);
    assert(p86_vx_GET(pack) == (float)5.366485E36F);
    assert(p86_afy_GET(pack) == (float) -2.6553599E38F);
    assert(p86_vz_GET(pack) == (float) -2.8427608E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p86_alt_GET(pack) == (float) -5.064966E37F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p86_vy_GET(pack) == (float) -2.7623715E38F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1260717749);
    assert(p86_yaw_rate_GET(pack) == (float) -3.240795E38F);
    assert(p86_yaw_GET(pack) == (float)1.1623619E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3143850606L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afx_GET(pack) == (float) -1.8007295E38F);
    assert(p87_vx_GET(pack) == (float)1.6270098E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -68437071);
    assert(p87_vy_GET(pack) == (float) -3.3192147E38F);
    assert(p87_vz_GET(pack) == (float)2.4959466E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -2.929623E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)1607);
    assert(p87_lon_int_GET(pack) == (int32_t)766756140);
    assert(p87_afy_GET(pack) == (float) -2.5990273E38F);
    assert(p87_alt_GET(pack) == (float)2.7313467E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p87_yaw_GET(pack) == (float) -1.3985569E38F);
    assert(p87_afz_GET(pack) == (float) -1.1373055E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)160637390L);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_roll_GET(pack) == (float)2.4542022E38F);
    assert(p89_x_GET(pack) == (float)1.7289902E38F);
    assert(p89_yaw_GET(pack) == (float)1.6791922E38F);
    assert(p89_z_GET(pack) == (float)8.778696E34F);
    assert(p89_y_GET(pack) == (float) -3.0559117E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)923618542L);
    assert(p89_pitch_GET(pack) == (float) -2.4562953E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_rollspeed_GET(pack) == (float) -2.7190795E38F);
    assert(p90_lat_GET(pack) == (int32_t)916457848);
    assert(p90_pitch_GET(pack) == (float) -5.3456527E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -29840);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)10258);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -32449);
    assert(p90_lon_GET(pack) == (int32_t)863569409);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)22132);
    assert(p90_yaw_GET(pack) == (float) -3.0691571E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -32530);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)4926);
    assert(p90_pitchspeed_GET(pack) == (float)6.8162947E37F);
    assert(p90_yawspeed_GET(pack) == (float) -8.0923417E37F);
    assert(p90_roll_GET(pack) == (float)4.1556378E37F);
    assert(p90_alt_GET(pack) == (int32_t) -507710499);
    assert(p90_time_usec_GET(pack) == (uint64_t)3447564714181013022L);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux2_GET(pack) == (float) -4.3006333E36F);
    assert(p91_throttle_GET(pack) == (float)9.491502E37F);
    assert(p91_pitch_elevator_GET(pack) == (float)1.959696E38F);
    assert(p91_aux1_GET(pack) == (float) -1.4373478E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)2.3599348E38F);
    assert(p91_aux3_GET(pack) == (float) -2.0237713E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)2.7379866E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    assert(p91_aux4_GET(pack) == (float) -4.188664E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p91_time_usec_GET(pack) == (uint64_t)4341852157303520636L);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)60064);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)4317);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)56621);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)40366);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)44697);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)47446);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)535);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)40927);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)26685);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)34747);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)9331);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)52601);
    assert(p92_time_usec_GET(pack) == (uint64_t)957206696110007356L);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
    {
        float exemplary[] =  {2.3424153E38F, 1.9260963E38F, -3.030215E38F, 2.2887008E38F, 3.318765E38F, 7.801857E36F, -5.549598E37F, -2.2479252E38F, -2.703805E38F, 3.3163738E38F, -4.262854E37F, 2.7588196E38F, -1.6102602E38F, -1.9484132E38F, -3.087779E38F, -1.0329464E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)5889341250926626420L);
    assert(p93_flags_GET(pack) == (uint64_t)8752074945038503827L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)24735);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)24797);
    assert(p100_flow_comp_m_y_GET(pack) == (float)9.797926E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)7763854895862686174L);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.7177292E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p100_ground_distance_GET(pack) == (float) -2.2328844E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.1390192E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)3.0318324E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_z_GET(pack) == (float) -7.7209267E37F);
    assert(p101_usec_GET(pack) == (uint64_t)498736983225992002L);
    assert(p101_roll_GET(pack) == (float) -1.471016E38F);
    assert(p101_x_GET(pack) == (float) -3.1666423E38F);
    assert(p101_yaw_GET(pack) == (float)7.239128E37F);
    assert(p101_pitch_GET(pack) == (float)1.7960997E38F);
    assert(p101_y_GET(pack) == (float) -3.3241478E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float) -4.5523245E37F);
    assert(p102_usec_GET(pack) == (uint64_t)4911048895249079937L);
    assert(p102_yaw_GET(pack) == (float) -3.1047631E38F);
    assert(p102_pitch_GET(pack) == (float)2.939542E38F);
    assert(p102_z_GET(pack) == (float)2.4484337E38F);
    assert(p102_x_GET(pack) == (float) -3.195099E38F);
    assert(p102_roll_GET(pack) == (float)9.709445E37F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)3265568937877337875L);
    assert(p103_z_GET(pack) == (float) -1.645416E38F);
    assert(p103_x_GET(pack) == (float) -1.6243359E38F);
    assert(p103_y_GET(pack) == (float) -1.7685377E38F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_z_GET(pack) == (float)7.562716E37F);
    assert(p104_usec_GET(pack) == (uint64_t)1918031982379744370L);
    assert(p104_pitch_GET(pack) == (float)2.4189149E38F);
    assert(p104_y_GET(pack) == (float) -1.0649707E38F);
    assert(p104_yaw_GET(pack) == (float) -8.244017E37F);
    assert(p104_roll_GET(pack) == (float)2.9616658E38F);
    assert(p104_x_GET(pack) == (float)3.0280911E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_ymag_GET(pack) == (float)6.7563566E37F);
    assert(p105_zgyro_GET(pack) == (float) -1.5315398E38F);
    assert(p105_abs_pressure_GET(pack) == (float)2.5709122E38F);
    assert(p105_temperature_GET(pack) == (float)3.1427963E38F);
    assert(p105_xgyro_GET(pack) == (float)1.2908451E38F);
    assert(p105_ygyro_GET(pack) == (float)6.3474967E37F);
    assert(p105_zacc_GET(pack) == (float)6.6506756E37F);
    assert(p105_pressure_alt_GET(pack) == (float)1.1903938E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4564595723709509319L);
    assert(p105_diff_pressure_GET(pack) == (float) -1.7654166E38F);
    assert(p105_xmag_GET(pack) == (float) -2.9743598E38F);
    assert(p105_yacc_GET(pack) == (float) -4.446091E37F);
    assert(p105_zmag_GET(pack) == (float)7.5062556E37F);
    assert(p105_xacc_GET(pack) == (float) -1.2509853E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)29146);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p106_time_usec_GET(pack) == (uint64_t)6584155107695955668L);
    assert(p106_distance_GET(pack) == (float)3.3831516E37F);
    assert(p106_integrated_zgyro_GET(pack) == (float)3.1871215E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -8.654267E37F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3273405669L);
    assert(p106_integrated_ygyro_GET(pack) == (float)6.018457E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)16865);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)638273837L);
    assert(p106_integrated_y_GET(pack) == (float) -2.4607356E38F);
    assert(p106_integrated_x_GET(pack) == (float)1.7000046E38F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float)2.7040164E38F);
    assert(p107_xacc_GET(pack) == (float)6.4466716E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)7885941136092111018L);
    assert(p107_ygyro_GET(pack) == (float) -1.8384848E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)3133699325L);
    assert(p107_diff_pressure_GET(pack) == (float)2.5824938E37F);
    assert(p107_yacc_GET(pack) == (float) -7.4801907E37F);
    assert(p107_xgyro_GET(pack) == (float) -1.4399179E38F);
    assert(p107_temperature_GET(pack) == (float) -2.6580256E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -3.0530198E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -2.4415762E38F);
    assert(p107_xmag_GET(pack) == (float)4.1332204E37F);
    assert(p107_zgyro_GET(pack) == (float) -3.2676245E37F);
    assert(p107_ymag_GET(pack) == (float) -1.0368956E38F);
    assert(p107_zacc_GET(pack) == (float)1.5552454E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_q4_GET(pack) == (float)1.6076391E38F);
    assert(p108_vn_GET(pack) == (float) -2.8212233E38F);
    assert(p108_roll_GET(pack) == (float) -2.3329314E38F);
    assert(p108_lon_GET(pack) == (float) -3.1920255E38F);
    assert(p108_yacc_GET(pack) == (float)2.7717784E37F);
    assert(p108_xgyro_GET(pack) == (float) -6.0401553E37F);
    assert(p108_pitch_GET(pack) == (float)2.4784305E38F);
    assert(p108_xacc_GET(pack) == (float) -3.1031008E38F);
    assert(p108_ve_GET(pack) == (float) -1.8168112E38F);
    assert(p108_zacc_GET(pack) == (float)5.8608406E37F);
    assert(p108_q2_GET(pack) == (float)4.7518963E37F);
    assert(p108_ygyro_GET(pack) == (float) -8.193333E37F);
    assert(p108_vd_GET(pack) == (float) -8.514871E37F);
    assert(p108_q3_GET(pack) == (float)1.6711406E38F);
    assert(p108_alt_GET(pack) == (float) -2.5369098E38F);
    assert(p108_yaw_GET(pack) == (float)5.735078E37F);
    assert(p108_std_dev_horz_GET(pack) == (float) -1.9438864E38F);
    assert(p108_q1_GET(pack) == (float) -2.3387914E38F);
    assert(p108_lat_GET(pack) == (float)1.0806649E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -2.0098345E38F);
    assert(p108_zgyro_GET(pack) == (float)2.9587237E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)54302);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)48485);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)142);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)54);
    {
        uint8_t exemplary[] =  {(uint8_t)14, (uint8_t)20, (uint8_t)66, (uint8_t)83, (uint8_t)122, (uint8_t)228, (uint8_t)39, (uint8_t)250, (uint8_t)170, (uint8_t)71, (uint8_t)219, (uint8_t)71, (uint8_t)15, (uint8_t)251, (uint8_t)47, (uint8_t)91, (uint8_t)12, (uint8_t)255, (uint8_t)77, (uint8_t)98, (uint8_t)203, (uint8_t)63, (uint8_t)180, (uint8_t)219, (uint8_t)136, (uint8_t)48, (uint8_t)152, (uint8_t)87, (uint8_t)76, (uint8_t)129, (uint8_t)67, (uint8_t)97, (uint8_t)159, (uint8_t)94, (uint8_t)239, (uint8_t)215, (uint8_t)112, (uint8_t)39, (uint8_t)217, (uint8_t)255, (uint8_t)176, (uint8_t)83, (uint8_t)6, (uint8_t)143, (uint8_t)211, (uint8_t)6, (uint8_t)63, (uint8_t)185, (uint8_t)119, (uint8_t)186, (uint8_t)161, (uint8_t)101, (uint8_t)163, (uint8_t)66, (uint8_t)190, (uint8_t)124, (uint8_t)16, (uint8_t)208, (uint8_t)254, (uint8_t)154, (uint8_t)135, (uint8_t)238, (uint8_t)80, (uint8_t)77, (uint8_t)169, (uint8_t)37, (uint8_t)250, (uint8_t)206, (uint8_t)220, (uint8_t)166, (uint8_t)161, (uint8_t)89, (uint8_t)159, (uint8_t)79, (uint8_t)79, (uint8_t)216, (uint8_t)192, (uint8_t)147, (uint8_t)85, (uint8_t)184, (uint8_t)9, (uint8_t)175, (uint8_t)82, (uint8_t)74, (uint8_t)168, (uint8_t)15, (uint8_t)152, (uint8_t)223, (uint8_t)60, (uint8_t)117, (uint8_t)67, (uint8_t)201, (uint8_t)149, (uint8_t)69, (uint8_t)166, (uint8_t)183, (uint8_t)84, (uint8_t)198, (uint8_t)194, (uint8_t)195, (uint8_t)216, (uint8_t)206, (uint8_t)252, (uint8_t)14, (uint8_t)20, (uint8_t)14, (uint8_t)206, (uint8_t)13, (uint8_t)243, (uint8_t)98, (uint8_t)56, (uint8_t)193, (uint8_t)155, (uint8_t)151, (uint8_t)184, (uint8_t)179, (uint8_t)29, (uint8_t)207, (uint8_t)182, (uint8_t)180, (uint8_t)16, (uint8_t)85, (uint8_t)190, (uint8_t)154, (uint8_t)110, (uint8_t)86, (uint8_t)157, (uint8_t)65, (uint8_t)30, (uint8_t)49, (uint8_t)195, (uint8_t)84, (uint8_t)181, (uint8_t)93, (uint8_t)151, (uint8_t)75, (uint8_t)152, (uint8_t)110, (uint8_t)241, (uint8_t)58, (uint8_t)236, (uint8_t)214, (uint8_t)69, (uint8_t)230, (uint8_t)103, (uint8_t)237, (uint8_t)171, (uint8_t)181, (uint8_t)63, (uint8_t)125, (uint8_t)117, (uint8_t)92, (uint8_t)3, (uint8_t)42, (uint8_t)70, (uint8_t)97, (uint8_t)171, (uint8_t)46, (uint8_t)62, (uint8_t)94, (uint8_t)59, (uint8_t)99, (uint8_t)223, (uint8_t)137, (uint8_t)180, (uint8_t)138, (uint8_t)152, (uint8_t)184, (uint8_t)110, (uint8_t)35, (uint8_t)54, (uint8_t)104, (uint8_t)84, (uint8_t)38, (uint8_t)218, (uint8_t)206, (uint8_t)109, (uint8_t)175, (uint8_t)122, (uint8_t)64, (uint8_t)1, (uint8_t)242, (uint8_t)118, (uint8_t)193, (uint8_t)11, (uint8_t)197, (uint8_t)24, (uint8_t)74, (uint8_t)128, (uint8_t)69, (uint8_t)75, (uint8_t)0, (uint8_t)151, (uint8_t)157, (uint8_t)24, (uint8_t)212, (uint8_t)18, (uint8_t)8, (uint8_t)105, (uint8_t)249, (uint8_t)254, (uint8_t)241, (uint8_t)199, (uint8_t)88, (uint8_t)74, (uint8_t)90, (uint8_t)52, (uint8_t)7, (uint8_t)14, (uint8_t)71, (uint8_t)39, (uint8_t)177, (uint8_t)9, (uint8_t)175, (uint8_t)194, (uint8_t)156, (uint8_t)137, (uint8_t)63, (uint8_t)218, (uint8_t)247, (uint8_t)120, (uint8_t)84, (uint8_t)254, (uint8_t)209, (uint8_t)79, (uint8_t)19, (uint8_t)166, (uint8_t)235, (uint8_t)74, (uint8_t)58, (uint8_t)79, (uint8_t)152, (uint8_t)197, (uint8_t)62, (uint8_t)145, (uint8_t)177, (uint8_t)75, (uint8_t)121, (uint8_t)235, (uint8_t)202, (uint8_t)158, (uint8_t)241, (uint8_t)165, (uint8_t)190, (uint8_t)99, (uint8_t)158, (uint8_t)101, (uint8_t)126, (uint8_t)9, (uint8_t)50, (uint8_t)49} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t)2072807529630901935L);
    assert(p111_tc1_GET(pack) == (int64_t) -5642557073663450733L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)3681329487L);
    assert(p112_time_usec_GET(pack) == (uint64_t)7754738809755496466L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -21928);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -20095);
    assert(p113_time_usec_GET(pack) == (uint64_t)5321507550030243487L);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p113_lat_GET(pack) == (int32_t) -359161562);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)18619);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)11246);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)57847);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)30882);
    assert(p113_lon_GET(pack) == (int32_t) -859842702);
    assert(p113_alt_GET(pack) == (int32_t) -1962844231);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)42054);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)14844);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3914441816L);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p114_integrated_x_GET(pack) == (float) -1.7674457E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)3261581816L);
    assert(p114_integrated_zgyro_GET(pack) == (float)3.2810434E38F);
    assert(p114_distance_GET(pack) == (float)1.4585309E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -3.261008E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)463398191010215363L);
    assert(p114_integrated_ygyro_GET(pack) == (float)2.1671686E38F);
    assert(p114_integrated_y_GET(pack) == (float)3.933954E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_time_usec_GET(pack) == (uint64_t)5630975430830068510L);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -27195);
    assert(p115_rollspeed_GET(pack) == (float) -2.6867082E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)11830);
    assert(p115_yawspeed_GET(pack) == (float) -6.1356216E37F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -8869);
    assert(p115_lon_GET(pack) == (int32_t)63297552);
    assert(p115_alt_GET(pack) == (int32_t)245572400);
    assert(p115_lat_GET(pack) == (int32_t)1120886219);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -30940);
    {
        float exemplary[] =  {8.983725E37F, 3.189822E38F, 1.8287655E38F, 3.0924265E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_pitchspeed_GET(pack) == (float)1.1836137E38F);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -2517);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -30651);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)8092);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)26195);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)22481);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -4082);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -5106);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)6505);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2526705085L);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)17560);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -11926);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -23854);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -31171);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -7104);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)18245);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)44350);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)2964086391L);
    assert(p118_time_utc_GET(pack) == (uint32_t)919832568L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)2393);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)37183);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)3956);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_ofs_GET(pack) == (uint32_t)2228877277L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)1629);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p119_count_GET(pack) == (uint32_t)2743970986L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)227, (uint8_t)64, (uint8_t)59, (uint8_t)166, (uint8_t)102, (uint8_t)113, (uint8_t)254, (uint8_t)115, (uint8_t)45, (uint8_t)15, (uint8_t)139, (uint8_t)138, (uint8_t)255, (uint8_t)235, (uint8_t)119, (uint8_t)37, (uint8_t)152, (uint8_t)168, (uint8_t)124, (uint8_t)64, (uint8_t)5, (uint8_t)1, (uint8_t)60, (uint8_t)145, (uint8_t)70, (uint8_t)195, (uint8_t)214, (uint8_t)152, (uint8_t)167, (uint8_t)122, (uint8_t)204, (uint8_t)237, (uint8_t)15, (uint8_t)74, (uint8_t)96, (uint8_t)37, (uint8_t)123, (uint8_t)179, (uint8_t)175, (uint8_t)55, (uint8_t)26, (uint8_t)76, (uint8_t)154, (uint8_t)139, (uint8_t)237, (uint8_t)236, (uint8_t)143, (uint8_t)36, (uint8_t)147, (uint8_t)221, (uint8_t)140, (uint8_t)175, (uint8_t)197, (uint8_t)124, (uint8_t)168, (uint8_t)227, (uint8_t)137, (uint8_t)76, (uint8_t)27, (uint8_t)4, (uint8_t)184, (uint8_t)208, (uint8_t)75, (uint8_t)103, (uint8_t)234, (uint8_t)15, (uint8_t)164, (uint8_t)197, (uint8_t)89, (uint8_t)60, (uint8_t)137, (uint8_t)153, (uint8_t)10, (uint8_t)210, (uint8_t)5, (uint8_t)154, (uint8_t)216, (uint8_t)167, (uint8_t)223, (uint8_t)25, (uint8_t)160, (uint8_t)229, (uint8_t)26, (uint8_t)73, (uint8_t)51, (uint8_t)169, (uint8_t)63, (uint8_t)166, (uint8_t)254, (uint8_t)65} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)9281);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p120_ofs_GET(pack) == (uint32_t)3943977850L);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)31);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)181);
    {
        uint8_t exemplary[] =  {(uint8_t)98, (uint8_t)98, (uint8_t)36, (uint8_t)190, (uint8_t)51, (uint8_t)101, (uint8_t)115, (uint8_t)39, (uint8_t)173, (uint8_t)87, (uint8_t)197, (uint8_t)252, (uint8_t)242, (uint8_t)75, (uint8_t)85, (uint8_t)147, (uint8_t)144, (uint8_t)37, (uint8_t)230, (uint8_t)225, (uint8_t)81, (uint8_t)161, (uint8_t)223, (uint8_t)213, (uint8_t)164, (uint8_t)229, (uint8_t)249, (uint8_t)13, (uint8_t)30, (uint8_t)245, (uint8_t)238, (uint8_t)180, (uint8_t)30, (uint8_t)170, (uint8_t)28, (uint8_t)28, (uint8_t)73, (uint8_t)205, (uint8_t)182, (uint8_t)1, (uint8_t)182, (uint8_t)3, (uint8_t)247, (uint8_t)171, (uint8_t)112, (uint8_t)183, (uint8_t)19, (uint8_t)0, (uint8_t)58, (uint8_t)92, (uint8_t)236, (uint8_t)91, (uint8_t)226, (uint8_t)165, (uint8_t)7, (uint8_t)93, (uint8_t)178, (uint8_t)24, (uint8_t)211, (uint8_t)48, (uint8_t)68, (uint8_t)141, (uint8_t)59, (uint8_t)21, (uint8_t)228, (uint8_t)145, (uint8_t)223, (uint8_t)249, (uint8_t)113, (uint8_t)4, (uint8_t)161, (uint8_t)110, (uint8_t)36, (uint8_t)161, (uint8_t)173, (uint8_t)187, (uint8_t)18, (uint8_t)8, (uint8_t)72, (uint8_t)46, (uint8_t)43, (uint8_t)2, (uint8_t)241, (uint8_t)9, (uint8_t)27, (uint8_t)9, (uint8_t)112, (uint8_t)246, (uint8_t)58, (uint8_t)35, (uint8_t)221, (uint8_t)227, (uint8_t)96, (uint8_t)111, (uint8_t)11, (uint8_t)227, (uint8_t)192, (uint8_t)51, (uint8_t)154, (uint8_t)203, (uint8_t)34, (uint8_t)211, (uint8_t)60, (uint8_t)188, (uint8_t)224, (uint8_t)185, (uint8_t)51, (uint8_t)30, (uint8_t)224, (uint8_t)248} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)195);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)18280);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)56856);
    assert(p124_time_usec_GET(pack) == (uint64_t)5727218306717901684L);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)27234);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p124_dgps_age_GET(pack) == (uint32_t)347524259L);
    assert(p124_alt_GET(pack) == (int32_t) -1789284962);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p124_lon_GET(pack) == (int32_t) -80919083);
    assert(p124_lat_GET(pack) == (int32_t)1064529040);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)45361);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)7077);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)25359);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)114, (uint8_t)87, (uint8_t)151, (uint8_t)191, (uint8_t)15, (uint8_t)237, (uint8_t)245, (uint8_t)82, (uint8_t)189, (uint8_t)2, (uint8_t)154, (uint8_t)251, (uint8_t)145, (uint8_t)113, (uint8_t)215, (uint8_t)138, (uint8_t)210, (uint8_t)0, (uint8_t)170, (uint8_t)136, (uint8_t)155, (uint8_t)187, (uint8_t)178, (uint8_t)93, (uint8_t)55, (uint8_t)110, (uint8_t)47, (uint8_t)7, (uint8_t)183, (uint8_t)186, (uint8_t)208, (uint8_t)235, (uint8_t)192, (uint8_t)39, (uint8_t)80, (uint8_t)202, (uint8_t)99, (uint8_t)245, (uint8_t)188, (uint8_t)212, (uint8_t)184, (uint8_t)235, (uint8_t)254, (uint8_t)8, (uint8_t)198, (uint8_t)65, (uint8_t)205, (uint8_t)123, (uint8_t)207, (uint8_t)251, (uint8_t)51, (uint8_t)75, (uint8_t)106, (uint8_t)35, (uint8_t)123, (uint8_t)138, (uint8_t)146, (uint8_t)159, (uint8_t)142, (uint8_t)48, (uint8_t)8, (uint8_t)100, (uint8_t)8, (uint8_t)175, (uint8_t)70, (uint8_t)50, (uint8_t)93, (uint8_t)197, (uint8_t)2, (uint8_t)49} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)62080);
    assert(p126_baudrate_GET(pack) == (uint32_t)1758113542L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)244);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -1698760304);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1791319386);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p127_tow_GET(pack) == (uint32_t)2574609806L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -305280526);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p127_accuracy_GET(pack) == (uint32_t)2762410331L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)487283407);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)5872);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)452625575L);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1750290247);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)9580373L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1391414462);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)4095);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)485408491);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)1268649990);
    assert(p128_accuracy_GET(pack) == (uint32_t)242853902L);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p128_tow_GET(pack) == (uint32_t)4018184508L);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)802012471L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -29710);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)25836);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)22691);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)16750);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)14195);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -20794);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -11514);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)9437);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)6717);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_size_GET(pack) == (uint32_t)3993553081L);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)43076);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)59564);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)7129);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)42, (uint8_t)183, (uint8_t)21, (uint8_t)128, (uint8_t)202, (uint8_t)65, (uint8_t)43, (uint8_t)194, (uint8_t)38, (uint8_t)163, (uint8_t)144, (uint8_t)32, (uint8_t)110, (uint8_t)39, (uint8_t)161, (uint8_t)213, (uint8_t)80, (uint8_t)141, (uint8_t)44, (uint8_t)215, (uint8_t)113, (uint8_t)11, (uint8_t)254, (uint8_t)132, (uint8_t)128, (uint8_t)209, (uint8_t)197, (uint8_t)91, (uint8_t)216, (uint8_t)21, (uint8_t)181, (uint8_t)239, (uint8_t)7, (uint8_t)82, (uint8_t)75, (uint8_t)79, (uint8_t)171, (uint8_t)186, (uint8_t)140, (uint8_t)164, (uint8_t)231, (uint8_t)178, (uint8_t)21, (uint8_t)120, (uint8_t)5, (uint8_t)189, (uint8_t)29, (uint8_t)114, (uint8_t)5, (uint8_t)134, (uint8_t)245, (uint8_t)51, (uint8_t)94, (uint8_t)35, (uint8_t)200, (uint8_t)238, (uint8_t)69, (uint8_t)200, (uint8_t)66, (uint8_t)55, (uint8_t)240, (uint8_t)209, (uint8_t)173, (uint8_t)180, (uint8_t)138, (uint8_t)207, (uint8_t)103, (uint8_t)216, (uint8_t)68, (uint8_t)152, (uint8_t)6, (uint8_t)83, (uint8_t)87, (uint8_t)151, (uint8_t)154, (uint8_t)137, (uint8_t)79, (uint8_t)193, (uint8_t)240, (uint8_t)235, (uint8_t)200, (uint8_t)136, (uint8_t)90, (uint8_t)255, (uint8_t)190, (uint8_t)183, (uint8_t)66, (uint8_t)103, (uint8_t)75, (uint8_t)236, (uint8_t)39, (uint8_t)213, (uint8_t)229, (uint8_t)46, (uint8_t)127, (uint8_t)77, (uint8_t)125, (uint8_t)143, (uint8_t)104, (uint8_t)229, (uint8_t)136, (uint8_t)241, (uint8_t)208, (uint8_t)234, (uint8_t)48, (uint8_t)112, (uint8_t)246, (uint8_t)52, (uint8_t)88, (uint8_t)95, (uint8_t)118, (uint8_t)224, (uint8_t)154, (uint8_t)243, (uint8_t)92, (uint8_t)129, (uint8_t)188, (uint8_t)229, (uint8_t)15, (uint8_t)87, (uint8_t)36, (uint8_t)228, (uint8_t)59, (uint8_t)66, (uint8_t)158, (uint8_t)126, (uint8_t)172, (uint8_t)175, (uint8_t)92, (uint8_t)64, (uint8_t)236, (uint8_t)250, (uint8_t)49, (uint8_t)185, (uint8_t)126, (uint8_t)156, (uint8_t)15, (uint8_t)7, (uint8_t)82, (uint8_t)12, (uint8_t)141, (uint8_t)243, (uint8_t)138, (uint8_t)52, (uint8_t)255, (uint8_t)242, (uint8_t)145, (uint8_t)69, (uint8_t)119, (uint8_t)110, (uint8_t)188, (uint8_t)57, (uint8_t)69, (uint8_t)211, (uint8_t)210, (uint8_t)160, (uint8_t)185, (uint8_t)65, (uint8_t)62, (uint8_t)239, (uint8_t)139, (uint8_t)30, (uint8_t)59, (uint8_t)190, (uint8_t)73, (uint8_t)136, (uint8_t)148, (uint8_t)129, (uint8_t)210, (uint8_t)34, (uint8_t)191, (uint8_t)155, (uint8_t)150, (uint8_t)134, (uint8_t)240, (uint8_t)244, (uint8_t)181, (uint8_t)26, (uint8_t)221, (uint8_t)85, (uint8_t)197, (uint8_t)8, (uint8_t)139, (uint8_t)182, (uint8_t)65, (uint8_t)72, (uint8_t)216, (uint8_t)77, (uint8_t)38, (uint8_t)5, (uint8_t)57, (uint8_t)251, (uint8_t)85, (uint8_t)251, (uint8_t)215, (uint8_t)28, (uint8_t)4, (uint8_t)217, (uint8_t)242, (uint8_t)43, (uint8_t)133, (uint8_t)173, (uint8_t)173, (uint8_t)247, (uint8_t)14, (uint8_t)223, (uint8_t)222, (uint8_t)244, (uint8_t)179, (uint8_t)49, (uint8_t)18, (uint8_t)141, (uint8_t)74, (uint8_t)197, (uint8_t)203, (uint8_t)83, (uint8_t)146, (uint8_t)190, (uint8_t)156, (uint8_t)162, (uint8_t)124, (uint8_t)82, (uint8_t)126, (uint8_t)248, (uint8_t)228, (uint8_t)43, (uint8_t)213, (uint8_t)140, (uint8_t)99, (uint8_t)12, (uint8_t)89, (uint8_t)193, (uint8_t)15, (uint8_t)195, (uint8_t)57, (uint8_t)203, (uint8_t)230, (uint8_t)16, (uint8_t)49, (uint8_t)56, (uint8_t)47, (uint8_t)171, (uint8_t)18, (uint8_t)118, (uint8_t)105, (uint8_t)232, (uint8_t)129, (uint8_t)90, (uint8_t)118, (uint8_t)169, (uint8_t)105, (uint8_t)169, (uint8_t)59} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)62662);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)63639);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)2993);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)11107);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1649280374L);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)3783131348454693120L);
    assert(p133_lat_GET(pack) == (int32_t)1639293627);
    assert(p133_lon_GET(pack) == (int32_t) -1370742673);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)16126);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)3332);
    assert(p134_lon_GET(pack) == (int32_t) -306727170);
    assert(p134_lat_GET(pack) == (int32_t) -2037070691);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)87);
    {
        int16_t exemplary[] =  {(int16_t) -26496, (int16_t)24825, (int16_t) -7534, (int16_t)14640, (int16_t)13663, (int16_t)6436, (int16_t) -17802, (int16_t) -255, (int16_t)24639, (int16_t)1126, (int16_t) -873, (int16_t)18967, (int16_t)17736, (int16_t) -22037, (int16_t) -18096, (int16_t) -29197} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)1276463005);
    assert(p135_lat_GET(pack) == (int32_t) -1511507621);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)16529);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)9241);
    assert(p136_current_height_GET(pack) == (float)7.0899E37F);
    assert(p136_lon_GET(pack) == (int32_t) -2075150205);
    assert(p136_lat_GET(pack) == (int32_t) -2142818411);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)24566);
    assert(p136_terrain_height_GET(pack) == (float)1.1256461E38F);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)2.0420736E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)268122830L);
    assert(p137_press_diff_GET(pack) == (float) -4.128494E37F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)5622);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.5228152E38F, 2.4162102E38F, -2.8994816E38F, -2.003331E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_time_usec_GET(pack) == (uint64_t)4117327214053500813L);
    assert(p138_y_GET(pack) == (float) -2.6174052E38F);
    assert(p138_x_GET(pack) == (float)2.0875993E38F);
    assert(p138_z_GET(pack) == (float) -7.5116746E37F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)235);
    {
        float exemplary[] =  {-1.995323E38F, 2.330003E38F, 4.713177E37F, -7.841549E37F, 6.7411727E37F, -4.5442085E37F, -1.6482445E38F, 8.355753E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_time_usec_GET(pack) == (uint64_t)2673352738059338382L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.3179131E38F, 2.6066943E38F, 2.117493E38F, 2.2581536E38F, 3.8239986E37F, -2.431956E38F, -8.110514E37F, -1.7838856E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p140_time_usec_GET(pack) == (uint64_t)2693871983898212670L);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float)1.8491404E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -5.154936E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)1550673287045278835L);
    assert(p141_altitude_local_GET(pack) == (float)1.7011174E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)4.4963623E37F);
    assert(p141_altitude_amsl_GET(pack) == (float)9.385674E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -2.7448753E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)90);
    {
        uint8_t exemplary[] =  {(uint8_t)222, (uint8_t)94, (uint8_t)36, (uint8_t)220, (uint8_t)106, (uint8_t)33, (uint8_t)21, (uint8_t)183, (uint8_t)165, (uint8_t)38, (uint8_t)159, (uint8_t)102, (uint8_t)43, (uint8_t)113, (uint8_t)181, (uint8_t)206, (uint8_t)128, (uint8_t)198, (uint8_t)96, (uint8_t)132, (uint8_t)32, (uint8_t)89, (uint8_t)42, (uint8_t)246, (uint8_t)224, (uint8_t)3, (uint8_t)69, (uint8_t)86, (uint8_t)147, (uint8_t)143, (uint8_t)26, (uint8_t)68, (uint8_t)137, (uint8_t)200, (uint8_t)254, (uint8_t)111, (uint8_t)178, (uint8_t)152, (uint8_t)63, (uint8_t)175, (uint8_t)71, (uint8_t)231, (uint8_t)138, (uint8_t)102, (uint8_t)133, (uint8_t)174, (uint8_t)33, (uint8_t)185, (uint8_t)129, (uint8_t)49, (uint8_t)143, (uint8_t)239, (uint8_t)31, (uint8_t)242, (uint8_t)212, (uint8_t)202, (uint8_t)166, (uint8_t)70, (uint8_t)71, (uint8_t)11, (uint8_t)21, (uint8_t)79, (uint8_t)146, (uint8_t)31, (uint8_t)38, (uint8_t)251, (uint8_t)4, (uint8_t)68, (uint8_t)62, (uint8_t)160, (uint8_t)185, (uint8_t)109, (uint8_t)117, (uint8_t)246, (uint8_t)48, (uint8_t)118, (uint8_t)98, (uint8_t)226, (uint8_t)127, (uint8_t)158, (uint8_t)80, (uint8_t)136, (uint8_t)86, (uint8_t)213, (uint8_t)86, (uint8_t)179, (uint8_t)246, (uint8_t)58, (uint8_t)250, (uint8_t)138, (uint8_t)84, (uint8_t)39, (uint8_t)180, (uint8_t)6, (uint8_t)223, (uint8_t)148, (uint8_t)42, (uint8_t)244, (uint8_t)2, (uint8_t)16, (uint8_t)52, (uint8_t)119, (uint8_t)11, (uint8_t)170, (uint8_t)227, (uint8_t)173, (uint8_t)216, (uint8_t)239, (uint8_t)250, (uint8_t)2, (uint8_t)246, (uint8_t)85, (uint8_t)3, (uint8_t)149, (uint8_t)7, (uint8_t)186, (uint8_t)63, (uint8_t)72, (uint8_t)229, (uint8_t)126} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)56);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)102, (uint8_t)17, (uint8_t)51, (uint8_t)108, (uint8_t)183, (uint8_t)195, (uint8_t)221, (uint8_t)50, (uint8_t)223, (uint8_t)184, (uint8_t)2, (uint8_t)38, (uint8_t)183, (uint8_t)214, (uint8_t)143, (uint8_t)0, (uint8_t)236, (uint8_t)8, (uint8_t)25, (uint8_t)16, (uint8_t)143, (uint8_t)219, (uint8_t)31, (uint8_t)182, (uint8_t)12, (uint8_t)158, (uint8_t)25, (uint8_t)230, (uint8_t)133, (uint8_t)145, (uint8_t)212, (uint8_t)186, (uint8_t)48, (uint8_t)93, (uint8_t)24, (uint8_t)29, (uint8_t)78, (uint8_t)4, (uint8_t)115, (uint8_t)99, (uint8_t)18, (uint8_t)198, (uint8_t)243, (uint8_t)227, (uint8_t)107, (uint8_t)3, (uint8_t)198, (uint8_t)17, (uint8_t)253, (uint8_t)61, (uint8_t)180, (uint8_t)114, (uint8_t)121, (uint8_t)27, (uint8_t)160, (uint8_t)74, (uint8_t)238, (uint8_t)31, (uint8_t)170, (uint8_t)121, (uint8_t)136, (uint8_t)164, (uint8_t)224, (uint8_t)74, (uint8_t)44, (uint8_t)17, (uint8_t)161, (uint8_t)78, (uint8_t)11, (uint8_t)163, (uint8_t)167, (uint8_t)130, (uint8_t)245, (uint8_t)168, (uint8_t)46, (uint8_t)237, (uint8_t)110, (uint8_t)87, (uint8_t)20, (uint8_t)183, (uint8_t)69, (uint8_t)188, (uint8_t)197, (uint8_t)1, (uint8_t)12, (uint8_t)135, (uint8_t)141, (uint8_t)2, (uint8_t)191, (uint8_t)250, (uint8_t)123, (uint8_t)240, (uint8_t)63, (uint8_t)60, (uint8_t)251, (uint8_t)254, (uint8_t)41, (uint8_t)83, (uint8_t)157, (uint8_t)247, (uint8_t)78, (uint8_t)69, (uint8_t)146, (uint8_t)94, (uint8_t)3, (uint8_t)248, (uint8_t)228, (uint8_t)164, (uint8_t)173, (uint8_t)22, (uint8_t)177, (uint8_t)70, (uint8_t)226, (uint8_t)12, (uint8_t)120, (uint8_t)17, (uint8_t)65, (uint8_t)11, (uint8_t)129} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)87);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)793992420L);
    assert(p143_press_diff_GET(pack) == (float) -3.3907245E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)9888);
    assert(p143_press_abs_GET(pack) == (float) -1.2654204E38F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_alt_GET(pack) == (float)3.1091362E38F);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)192);
    {
        float exemplary[] =  {-1.8785556E38F, -3.281628E38F, -4.33999E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-8.576083E37F, -2.1649042E38F, -1.6689853E38F, 2.3794222E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.6081995E37F, 3.7325987E37F, 2.4682496E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)7931151958274776094L);
    assert(p144_lon_GET(pack) == (int32_t) -1527013454);
    {
        float exemplary[] =  {1.8461781E38F, 1.4835346E38F, -1.6679438E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)7026173012231353721L);
    assert(p144_lat_GET(pack) == (int32_t)170039208);
    {
        float exemplary[] =  {-2.0137859E38F, -2.5366966E38F, 1.4678234E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_airspeed_GET(pack) == (float) -4.443636E37F);
    assert(p146_pitch_rate_GET(pack) == (float)8.1398416E37F);
    {
        float exemplary[] =  {2.86957E38F, -1.4717307E38F, -1.2641365E34F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float)1.0838679E38F);
    {
        float exemplary[] =  {2.3154042E38F, -2.1347092E38F, -1.8570838E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_roll_rate_GET(pack) == (float) -8.0073614E37F);
    assert(p146_y_acc_GET(pack) == (float)1.5718313E38F);
    assert(p146_x_vel_GET(pack) == (float) -1.1809581E38F);
    assert(p146_x_pos_GET(pack) == (float) -2.7768823E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.7620203E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)4276370110219286570L);
    assert(p146_x_acc_GET(pack) == (float)1.1410252E38F);
    {
        float exemplary[] =  {3.003171E38F, -3.1161547E38F, -2.0528409E38F, 2.44463E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_vel_GET(pack) == (float) -2.3183167E38F);
    assert(p146_y_pos_GET(pack) == (float) -3.2774149E38F);
    assert(p146_z_acc_GET(pack) == (float) -2.0092164E37F);
    assert(p146_z_pos_GET(pack) == (float)2.9831875E38F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_consumed_GET(pack) == (int32_t)1505651378);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)26180);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -717531605);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)18750);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD);
    {
        uint16_t exemplary[] =  {(uint16_t)22205, (uint16_t)16855, (uint16_t)13772, (uint16_t)47645, (uint16_t)23905, (uint16_t)21013, (uint16_t)51831, (uint16_t)32675, (uint16_t)35861, (uint16_t)37154} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)50);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)236);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)37, (uint8_t)220, (uint8_t)190, (uint8_t)114, (uint8_t)49, (uint8_t)81, (uint8_t)82, (uint8_t)244, (uint8_t)0, (uint8_t)21, (uint8_t)144, (uint8_t)73, (uint8_t)16, (uint8_t)85, (uint8_t)176, (uint8_t)211, (uint8_t)52, (uint8_t)143} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)1696);
    {
        uint8_t exemplary[] =  {(uint8_t)242, (uint8_t)151, (uint8_t)106, (uint8_t)22, (uint8_t)172, (uint8_t)164, (uint8_t)150, (uint8_t)196} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)4000539540780997984L);
    assert(p148_board_version_GET(pack) == (uint32_t)1896205892L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)19134);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2645309967L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1330271977L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)2778212739L);
    {
        uint8_t exemplary[] =  {(uint8_t)178, (uint8_t)44, (uint8_t)80, (uint8_t)108, (uint8_t)205, (uint8_t)96, (uint8_t)53, (uint8_t)107} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)175, (uint8_t)5, (uint8_t)160, (uint8_t)230, (uint8_t)9, (uint8_t)123, (uint8_t)77, (uint8_t)233} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p149_x_TRY(ph) == (float)1.5990729E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_angle_x_GET(pack) == (float) -2.6288402E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)114);
    {
        float exemplary[] =  {1.1664952E38F, 4.452393E36F, -3.121072E38F, 3.0649627E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_angle_y_GET(pack) == (float) -1.8082924E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)6450284443771666817L);
    assert(p149_size_x_GET(pack) == (float) -5.5350346E37F);
    assert(p149_y_TRY(ph) == (float)8.249089E37F);
    assert(p149_z_TRY(ph) == (float)4.0765108E37F);
    assert(p149_size_y_GET(pack) == (float)2.1829864E38F);
    assert(p149_distance_GET(pack) == (float) -1.0360258E38F);
};


void c_LoopBackDemoChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_sensLoad_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p170_ctrlLoad_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p170_batVolt_GET(pack) == (uint16_t)(uint16_t)41539);
};


void c_LoopBackDemoChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_gxBias_GET(pack) == (float) -2.2086232E38F);
    assert(p172_gzBias_GET(pack) == (float) -7.6081996E37F);
    assert(p172_azBias_GET(pack) == (float)2.632925E38F);
    assert(p172_ayBias_GET(pack) == (float) -5.430878E37F);
    assert(p172_gyBias_GET(pack) == (float)7.201545E37F);
    assert(p172_axBias_GET(pack) == (float) -1.7762564E38F);
};


void c_LoopBackDemoChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_diagFl1_GET(pack) == (float) -1.2771473E38F);
    assert(p173_diagSh3_GET(pack) == (int16_t)(int16_t) -18293);
    assert(p173_diagSh2_GET(pack) == (int16_t)(int16_t)15830);
    assert(p173_diagSh1_GET(pack) == (int16_t)(int16_t)24448);
    assert(p173_diagFl3_GET(pack) == (float) -4.092314E37F);
    assert(p173_diagFl2_GET(pack) == (float)1.1557893E38F);
};


void c_LoopBackDemoChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_phi_c_GET(pack) == (float)2.864455E38F);
    assert(p176_theta_c_GET(pack) == (float) -5.6543124E37F);
    assert(p176_u_m_GET(pack) == (float)1.4214203E38F);
    assert(p176_dist2Go_GET(pack) == (float)3.2128385E38F);
    assert(p176_totalDist_GET(pack) == (float) -2.833827E38F);
    assert(p176_psiDot_c_GET(pack) == (float) -2.8015699E38F);
    assert(p176_toWP_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p176_h_c_GET(pack) == (uint16_t)(uint16_t)29944);
    assert(p176_ay_body_GET(pack) == (float) -1.614989E38F);
    assert(p176_fromWP_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_LoopBackDemoChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_fl_6_GET(pack) == (float) -1.1764757E38F);
    assert(p177_fl_5_GET(pack) == (float)2.9485835E38F);
    assert(p177_fl_3_GET(pack) == (float) -2.6412721E38F);
    assert(p177_fl_1_GET(pack) == (float) -2.7393566E38F);
    assert(p177_fl_4_GET(pack) == (float)1.3052289E38F);
    assert(p177_fl_2_GET(pack) == (float) -1.803697E38F);
};


void c_LoopBackDemoChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_useSat_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p179_sec_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p179_GppGl_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p179_sigUsedMask_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p179_min_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p179_year_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p179_month_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p179_day_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p179_clockStat_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p179_hour_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p179_visSat_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p179_percentUsed_GET(pack) == (uint8_t)(uint8_t)213);
};


void c_LoopBackDemoChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_uCommand_GET(pack) == (float) -1.2558346E38F);
    assert(p180_target_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p180_rCommand_GET(pack) == (float) -1.303029E38F);
    assert(p180_hCommand_GET(pack) == (float) -6.995418E37F);
};


void c_LoopBackDemoChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_target_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p181_bitfieldPt_GET(pack) == (uint16_t)(uint16_t)53985);
};


void c_LoopBackDemoChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_pan_GET(pack) == (int8_t)(int8_t) -51);
    assert(p184_target_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p184_zoom_GET(pack) == (int8_t)(int8_t) -108);
    assert(p184_tilt_GET(pack) == (int8_t)(int8_t)50);
    assert(p184_moveHome_GET(pack) == (int8_t)(int8_t)54);
};


void c_LoopBackDemoChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_idSurface_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p185_target_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p185_mControl_GET(pack) == (float) -9.873779E37F);
    assert(p185_bControl_GET(pack) == (float)3.2672019E38F);
};


void c_LoopBackDemoChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_latitude_GET(pack) == (float)2.5424844E38F);
    assert(p186_longitude_GET(pack) == (float)1.5200676E38F);
    assert(p186_target_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_LoopBackDemoChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_idOrder_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p188_order_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p188_target_GET(pack) == (uint8_t)(uint8_t)247);
};


void c_LoopBackDemoChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    assert(p189_target_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p189_height_GET(pack) == (float) -3.8417325E37F);
    assert(p189_option2_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p189_latitude_GET(pack) == (float) -3.0602231E38F);
    assert(p189_option3_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p189_longitude_GET(pack) == (float)1.0230297E38F);
    assert(p189_option1_GET(pack) == (uint8_t)(uint8_t)126);
};


void c_LoopBackDemoChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_r2Type_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p191_voltage_GET(pack) == (uint16_t)(uint16_t)49925);
    assert(p191_reading2_GET(pack) == (uint16_t)(uint16_t)24568);
};


void c_LoopBackDemoChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_tilt_GET(pack) == (int16_t)(int16_t) -13852);
    assert(p192_pan_GET(pack) == (int16_t)(int16_t) -29536);
    assert(p192_zoom_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_LoopBackDemoChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_course_GET(pack) == (float) -3.1016354E38F);
    assert(p193_latitude_GET(pack) == (float) -2.3605613E38F);
    assert(p193_longitude_GET(pack) == (float)3.6145528E37F);
    assert(p193_target_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p193_speed_GET(pack) == (float) -3.3286852E38F);
    assert(p193_altitude_GET(pack) == (float) -7.506675E36F);
};


void c_LoopBackDemoChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_modeInd_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p194_csFails_GET(pack) == (uint16_t)(uint16_t)45155);
    assert(p194_magVar_GET(pack) == (float)3.1055884E38F);
    assert(p194_magDir_GET(pack) == (int8_t)(int8_t) -1);
    assert(p194_posStatus_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p194_msgsType_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p194_gpsQuality_GET(pack) == (uint8_t)(uint8_t)124);
};


void c_LoopBackDemoChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    assert(p195_solStatus_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p195_csFails_GET(pack) == (uint16_t)(uint16_t)492);
    assert(p195_timeStatus_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p195_posSolAge_GET(pack) == (float) -1.9603454E38F);
    assert(p195_posType_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p195_velType_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p195_receiverStatus_GET(pack) == (uint32_t)1072074731L);
};


void c_LoopBackDemoChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    assert(p196_char1_GET(pack) == (int8_t)(int8_t) -72);
    assert(p196_float2_GET(pack) == (float) -2.0139625E38F);
    assert(p196_int1_GET(pack) == (int16_t)(int16_t)30310);
    assert(p196_float1_GET(pack) == (float) -9.128335E37F);
};


void c_LoopBackDemoChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    assert(p197_version_GET(pack) == (uint32_t)2699144086L);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_hagl_ratio_GET(pack) == (float)1.5866122E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.2384308E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -1.089248E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -3.0043161E38F);
    assert(p230_mag_ratio_GET(pack) == (float)2.1546089E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE);
    assert(p230_tas_ratio_GET(pack) == (float)1.3311353E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)6623687166019401223L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.8682345E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)5.366357E37F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_vert_accuracy_GET(pack) == (float) -1.6021569E38F);
    assert(p231_wind_y_GET(pack) == (float)1.1430512E38F);
    assert(p231_wind_x_GET(pack) == (float)6.5854266E37F);
    assert(p231_time_usec_GET(pack) == (uint64_t)3053613563962840942L);
    assert(p231_horiz_accuracy_GET(pack) == (float) -1.3629698E38F);
    assert(p231_wind_z_GET(pack) == (float)1.989475E38F);
    assert(p231_var_vert_GET(pack) == (float)5.995671E37F);
    assert(p231_wind_alt_GET(pack) == (float) -3.2538182E37F);
    assert(p231_var_horiz_GET(pack) == (float)3.2861067E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_vdop_GET(pack) == (float)3.087965E38F);
    assert(p232_lat_GET(pack) == (int32_t) -1352637103);
    assert(p232_time_usec_GET(pack) == (uint64_t)4569276684848568024L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p232_speed_accuracy_GET(pack) == (float) -1.0888324E38F);
    assert(p232_vn_GET(pack) == (float)9.467793E37F);
    assert(p232_ve_GET(pack) == (float) -2.8145563E38F);
    assert(p232_lon_GET(pack) == (int32_t) -1618227355);
    assert(p232_vert_accuracy_GET(pack) == (float) -2.4085765E38F);
    assert(p232_vd_GET(pack) == (float)2.2649592E38F);
    assert(p232_hdop_GET(pack) == (float)1.8845454E38F);
    assert(p232_alt_GET(pack) == (float)2.4192607E37F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p232_horiz_accuracy_GET(pack) == (float) -8.2376555E37F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)1199585044L);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)3602);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)180);
    {
        uint8_t exemplary[] =  {(uint8_t)85, (uint8_t)208, (uint8_t)148, (uint8_t)51, (uint8_t)166, (uint8_t)129, (uint8_t)237, (uint8_t)4, (uint8_t)208, (uint8_t)37, (uint8_t)79, (uint8_t)184, (uint8_t)123, (uint8_t)61, (uint8_t)58, (uint8_t)106, (uint8_t)23, (uint8_t)73, (uint8_t)150, (uint8_t)255, (uint8_t)148, (uint8_t)216, (uint8_t)112, (uint8_t)246, (uint8_t)80, (uint8_t)167, (uint8_t)14, (uint8_t)248, (uint8_t)45, (uint8_t)106, (uint8_t)146, (uint8_t)169, (uint8_t)174, (uint8_t)0, (uint8_t)194, (uint8_t)47, (uint8_t)253, (uint8_t)157, (uint8_t)135, (uint8_t)84, (uint8_t)66, (uint8_t)5, (uint8_t)66, (uint8_t)221, (uint8_t)79, (uint8_t)219, (uint8_t)6, (uint8_t)4, (uint8_t)196, (uint8_t)67, (uint8_t)159, (uint8_t)230, (uint8_t)15, (uint8_t)13, (uint8_t)106, (uint8_t)146, (uint8_t)240, (uint8_t)143, (uint8_t)24, (uint8_t)20, (uint8_t)196, (uint8_t)158, (uint8_t)74, (uint8_t)127, (uint8_t)64, (uint8_t)53, (uint8_t)169, (uint8_t)193, (uint8_t)94, (uint8_t)244, (uint8_t)133, (uint8_t)160, (uint8_t)142, (uint8_t)18, (uint8_t)204, (uint8_t)231, (uint8_t)67, (uint8_t)120, (uint8_t)196, (uint8_t)60, (uint8_t)234, (uint8_t)203, (uint8_t)24, (uint8_t)249, (uint8_t)116, (uint8_t)240, (uint8_t)106, (uint8_t)55, (uint8_t)248, (uint8_t)102, (uint8_t)52, (uint8_t)245, (uint8_t)104, (uint8_t)151, (uint8_t)35, (uint8_t)50, (uint8_t)26, (uint8_t)160, (uint8_t)147, (uint8_t)207, (uint8_t)50, (uint8_t)70, (uint8_t)83, (uint8_t)156, (uint8_t)110, (uint8_t)106, (uint8_t)154, (uint8_t)198, (uint8_t)62, (uint8_t)197, (uint8_t)144, (uint8_t)139, (uint8_t)61, (uint8_t)249, (uint8_t)205, (uint8_t)2, (uint8_t)246, (uint8_t)224, (uint8_t)219, (uint8_t)214, (uint8_t)232, (uint8_t)250, (uint8_t)6, (uint8_t)46, (uint8_t)42, (uint8_t)77, (uint8_t)181, (uint8_t)167, (uint8_t)174, (uint8_t)74, (uint8_t)211, (uint8_t)221, (uint8_t)30, (uint8_t)47, (uint8_t)116, (uint8_t)158, (uint8_t)240, (uint8_t)219, (uint8_t)243, (uint8_t)206, (uint8_t)68, (uint8_t)148, (uint8_t)18, (uint8_t)31, (uint8_t)200, (uint8_t)94, (uint8_t)189, (uint8_t)54, (uint8_t)162, (uint8_t)124, (uint8_t)154, (uint8_t)173, (uint8_t)78, (uint8_t)3, (uint8_t)255, (uint8_t)234, (uint8_t)172, (uint8_t)14, (uint8_t)225, (uint8_t)116, (uint8_t)1, (uint8_t)244, (uint8_t)82, (uint8_t)225, (uint8_t)27, (uint8_t)18, (uint8_t)26, (uint8_t)248, (uint8_t)20, (uint8_t)24, (uint8_t)103, (uint8_t)36, (uint8_t)47, (uint8_t)35, (uint8_t)223, (uint8_t)215, (uint8_t)238, (uint8_t)252, (uint8_t)31, (uint8_t)101} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)71);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)81);
    assert(p234_custom_mode_GET(pack) == (uint32_t)2562130396L);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)31485);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)28732);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -5710);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p234_longitude_GET(pack) == (int32_t) -1525836642);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -15989);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -104);
    assert(p234_latitude_GET(pack) == (int32_t)1348653872);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -48);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)58191);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)24105);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)37);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -27399);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_0_GET(pack) == (uint32_t)2056511427L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2359828007L);
    assert(p241_vibration_y_GET(pack) == (float) -3.3572286E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)1370157642L);
    assert(p241_vibration_z_GET(pack) == (float) -1.4737699E38F);
    assert(p241_vibration_x_GET(pack) == (float) -2.1036714E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)6639173384410457884L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_longitude_GET(pack) == (int32_t)1782130640);
    assert(p242_approach_z_GET(pack) == (float) -2.422618E38F);
    assert(p242_latitude_GET(pack) == (int32_t)6223292);
    assert(p242_time_usec_TRY(ph) == (uint64_t)2957070934689285937L);
    assert(p242_altitude_GET(pack) == (int32_t) -1881079676);
    assert(p242_approach_x_GET(pack) == (float) -8.903295E37F);
    {
        float exemplary[] =  {1.7185555E38F, 1.5826498E38F, 1.9136595E38F, -3.3555458E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_y_GET(pack) == (float) -5.0542735E36F);
    assert(p242_approach_y_GET(pack) == (float) -1.5261746E38F);
    assert(p242_x_GET(pack) == (float)1.742057E38F);
    assert(p242_z_GET(pack) == (float) -3.0636783E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_altitude_GET(pack) == (int32_t)223965135);
    assert(p243_y_GET(pack) == (float) -3.6551942E37F);
    assert(p243_x_GET(pack) == (float)2.0487469E38F);
    assert(p243_latitude_GET(pack) == (int32_t)2242953);
    {
        float exemplary[] =  {-7.8307286E35F, -8.034711E37F, 3.206773E37F, 3.3709677E37F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_time_usec_TRY(ph) == (uint64_t)7092725404800400376L);
    assert(p243_z_GET(pack) == (float)2.8832255E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.0674547E38F);
    assert(p243_longitude_GET(pack) == (int32_t) -1532408151);
    assert(p243_approach_x_GET(pack) == (float) -1.3199435E37F);
    assert(p243_approach_y_GET(pack) == (float)1.8259225E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)64);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)50800);
    assert(p244_interval_us_GET(pack) == (int32_t)2102380356);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)22054);
    assert(p246_altitude_GET(pack) == (int32_t)1069741034);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)55446);
    assert(p246_lon_GET(pack) == (int32_t) -174771616);
    assert(p246_lat_GET(pack) == (int32_t)63931917);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)64865);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)585015769L);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -13169);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p246_callsign_LEN(ph) == 3);
    {
        char16_t * exemplary = u"klo";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)3.0818292E38F);
    assert(p247_id_GET(pack) == (uint32_t)3549377175L);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.4510701E37F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.731188E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)61);
    {
        uint8_t exemplary[] =  {(uint8_t)159, (uint8_t)1, (uint8_t)13, (uint8_t)144, (uint8_t)216, (uint8_t)97, (uint8_t)91, (uint8_t)164, (uint8_t)220, (uint8_t)230, (uint8_t)36, (uint8_t)28, (uint8_t)180, (uint8_t)147, (uint8_t)106, (uint8_t)163, (uint8_t)209, (uint8_t)210, (uint8_t)23, (uint8_t)113, (uint8_t)64, (uint8_t)55, (uint8_t)217, (uint8_t)211, (uint8_t)79, (uint8_t)92, (uint8_t)153, (uint8_t)114, (uint8_t)196, (uint8_t)177, (uint8_t)138, (uint8_t)93, (uint8_t)227, (uint8_t)233, (uint8_t)226, (uint8_t)9, (uint8_t)3, (uint8_t)190, (uint8_t)201, (uint8_t)67, (uint8_t)200, (uint8_t)15, (uint8_t)113, (uint8_t)1, (uint8_t)106, (uint8_t)46, (uint8_t)83, (uint8_t)25, (uint8_t)245, (uint8_t)149, (uint8_t)215, (uint8_t)166, (uint8_t)195, (uint8_t)145, (uint8_t)83, (uint8_t)103, (uint8_t)131, (uint8_t)60, (uint8_t)30, (uint8_t)209, (uint8_t)104, (uint8_t)145, (uint8_t)84, (uint8_t)83, (uint8_t)148, (uint8_t)151, (uint8_t)86, (uint8_t)231, (uint8_t)217, (uint8_t)108, (uint8_t)52, (uint8_t)8, (uint8_t)42, (uint8_t)252, (uint8_t)123, (uint8_t)181, (uint8_t)91, (uint8_t)255, (uint8_t)198, (uint8_t)145, (uint8_t)250, (uint8_t)134, (uint8_t)247, (uint8_t)20, (uint8_t)16, (uint8_t)119, (uint8_t)127, (uint8_t)214, (uint8_t)2, (uint8_t)156, (uint8_t)250, (uint8_t)116, (uint8_t)35, (uint8_t)211, (uint8_t)129, (uint8_t)123, (uint8_t)182, (uint8_t)18, (uint8_t)123, (uint8_t)200, (uint8_t)235, (uint8_t)160, (uint8_t)227, (uint8_t)178, (uint8_t)238, (uint8_t)59, (uint8_t)106, (uint8_t)161, (uint8_t)209, (uint8_t)11, (uint8_t)141, (uint8_t)38, (uint8_t)172, (uint8_t)15, (uint8_t)167, (uint8_t)148, (uint8_t)156, (uint8_t)10, (uint8_t)73, (uint8_t)161, (uint8_t)78, (uint8_t)51, (uint8_t)241, (uint8_t)126, (uint8_t)5, (uint8_t)129, (uint8_t)39, (uint8_t)95, (uint8_t)239, (uint8_t)236, (uint8_t)152, (uint8_t)115, (uint8_t)189, (uint8_t)71, (uint8_t)239, (uint8_t)115, (uint8_t)126, (uint8_t)26, (uint8_t)140, (uint8_t)15, (uint8_t)115, (uint8_t)12, (uint8_t)5, (uint8_t)132, (uint8_t)118, (uint8_t)38, (uint8_t)82, (uint8_t)135, (uint8_t)182, (uint8_t)48, (uint8_t)233, (uint8_t)191, (uint8_t)114, (uint8_t)199, (uint8_t)142, (uint8_t)203, (uint8_t)52, (uint8_t)251, (uint8_t)161, (uint8_t)191, (uint8_t)49, (uint8_t)247, (uint8_t)211, (uint8_t)192, (uint8_t)231, (uint8_t)165, (uint8_t)188, (uint8_t)132, (uint8_t)193, (uint8_t)125, (uint8_t)104, (uint8_t)234, (uint8_t)45, (uint8_t)54, (uint8_t)131, (uint8_t)229, (uint8_t)138, (uint8_t)74, (uint8_t)95, (uint8_t)224, (uint8_t)148, (uint8_t)246, (uint8_t)52, (uint8_t)140, (uint8_t)27, (uint8_t)168, (uint8_t)243, (uint8_t)104, (uint8_t)1, (uint8_t)126, (uint8_t)58, (uint8_t)205, (uint8_t)247, (uint8_t)107, (uint8_t)212, (uint8_t)182, (uint8_t)152, (uint8_t)22, (uint8_t)29, (uint8_t)111, (uint8_t)206, (uint8_t)73, (uint8_t)84, (uint8_t)122, (uint8_t)48, (uint8_t)200, (uint8_t)104, (uint8_t)168, (uint8_t)9, (uint8_t)208, (uint8_t)155, (uint8_t)144, (uint8_t)158, (uint8_t)26, (uint8_t)231, (uint8_t)216, (uint8_t)39, (uint8_t)197, (uint8_t)206, (uint8_t)75, (uint8_t)196, (uint8_t)120, (uint8_t)30, (uint8_t)57, (uint8_t)217, (uint8_t)240, (uint8_t)114, (uint8_t)80, (uint8_t)85, (uint8_t)127, (uint8_t)149, (uint8_t)65, (uint8_t)30, (uint8_t)203, (uint8_t)119, (uint8_t)94, (uint8_t)15, (uint8_t)141, (uint8_t)182, (uint8_t)247, (uint8_t)203, (uint8_t)150, (uint8_t)92, (uint8_t)213, (uint8_t)211, (uint8_t)101, (uint8_t)56, (uint8_t)248, (uint8_t)170} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)50382);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)164);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)40289);
    {
        int8_t exemplary[] =  {(int8_t)48, (int8_t) -28, (int8_t) -11, (int8_t)64, (int8_t) -107, (int8_t)11, (int8_t) -23, (int8_t) -67, (int8_t)44, (int8_t)55, (int8_t)23, (int8_t)39, (int8_t)52, (int8_t) -84, (int8_t) -39, (int8_t)76, (int8_t)43, (int8_t)44, (int8_t)87, (int8_t) -46, (int8_t) -41, (int8_t)1, (int8_t) -20, (int8_t)43, (int8_t) -113, (int8_t)0, (int8_t) -109, (int8_t) -126, (int8_t)115, (int8_t)106, (int8_t) -39, (int8_t)21} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)86);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_z_GET(pack) == (float) -2.087759E38F);
    assert(p250_y_GET(pack) == (float)2.6132846E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)9077288534719290114L);
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"aOOfp";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_x_GET(pack) == (float)1.1230683E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"at";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2120336383L);
    assert(p251_value_GET(pack) == (float) -1.7512494E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"qFhdx";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)265359874L);
    assert(p252_value_GET(pack) == (int32_t) -937667782);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL);
    assert(p253_text_LEN(ph) == 50);
    {
        char16_t * exemplary = u"meJikcicxfqnfculpcwbdorebyywEFdRycndmTkvqvBlyzggih";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 100);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1808688832L);
    assert(p254_value_GET(pack) == (float)3.3100722E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)179);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)608093649166045309L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)73);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)116, (uint8_t)73, (uint8_t)168, (uint8_t)231, (uint8_t)162, (uint8_t)159, (uint8_t)6, (uint8_t)137, (uint8_t)40, (uint8_t)249, (uint8_t)210, (uint8_t)235, (uint8_t)133, (uint8_t)115, (uint8_t)153, (uint8_t)247, (uint8_t)225, (uint8_t)17, (uint8_t)153, (uint8_t)107, (uint8_t)255, (uint8_t)81, (uint8_t)147, (uint8_t)183, (uint8_t)145, (uint8_t)169, (uint8_t)131, (uint8_t)103, (uint8_t)216, (uint8_t)216, (uint8_t)80} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2330095207L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)4095445856L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)142);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 15);
    {
        char16_t * exemplary = u"zvwkzkrgqwclbab";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)38);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)16665);
    assert(p259_sensor_size_v_GET(pack) == (float) -3.0073698E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)35, (uint8_t)86, (uint8_t)135, (uint8_t)166, (uint8_t)177, (uint8_t)143, (uint8_t)92, (uint8_t)90, (uint8_t)107, (uint8_t)112, (uint8_t)14, (uint8_t)47, (uint8_t)102, (uint8_t)14, (uint8_t)214, (uint8_t)95, (uint8_t)32, (uint8_t)63, (uint8_t)223, (uint8_t)37, (uint8_t)247, (uint8_t)99, (uint8_t)200, (uint8_t)63, (uint8_t)17, (uint8_t)208, (uint8_t)179, (uint8_t)188, (uint8_t)209, (uint8_t)253, (uint8_t)208, (uint8_t)157} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)21449);
    assert(p259_firmware_version_GET(pack) == (uint32_t)235397576L);
    assert(p259_sensor_size_h_GET(pack) == (float)2.663238E38F);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
    assert(p259_cam_definition_uri_LEN(ph) == 82);
    {
        char16_t * exemplary = u"qrwbxiodfdqecqfecirwrqmshilwvwqzxtspxvKNsjlgczhxseltbwnnhxiuzcrxeftvfwXkydbcestlfx";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 164);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)102, (uint8_t)94, (uint8_t)77, (uint8_t)74, (uint8_t)140, (uint8_t)138, (uint8_t)20, (uint8_t)163, (uint8_t)6, (uint8_t)90, (uint8_t)152, (uint8_t)241, (uint8_t)76, (uint8_t)32, (uint8_t)149, (uint8_t)7, (uint8_t)136, (uint8_t)34, (uint8_t)168, (uint8_t)242, (uint8_t)177, (uint8_t)27, (uint8_t)140, (uint8_t)239, (uint8_t)130, (uint8_t)162, (uint8_t)209, (uint8_t)187, (uint8_t)128, (uint8_t)0, (uint8_t)26} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float)5.542363E37F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1723126269L);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)62113);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3125861768L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1865184936L);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p261_write_speed_GET(pack) == (float)3.1662855E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p261_used_capacity_GET(pack) == (float)3.958427E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p261_read_speed_GET(pack) == (float)3.3733251E38F);
    assert(p261_total_capacity_GET(pack) == (float) -2.3701346E38F);
    assert(p261_available_capacity_GET(pack) == (float)3.3976113E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2021985350L);
    assert(p262_image_interval_GET(pack) == (float) -2.1585621E38F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p262_available_capacity_GET(pack) == (float) -1.6419071E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1559516473L);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_relative_alt_GET(pack) == (int32_t) -643904844);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)74);
    assert(p263_lat_GET(pack) == (int32_t) -335310084);
    assert(p263_lon_GET(pack) == (int32_t)353557546);
    assert(p263_image_index_GET(pack) == (int32_t)1862195420);
    {
        float exemplary[] =  {-2.1447028E38F, -2.9809463E38F, 7.058471E37F, 1.6023305E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t) -1502427323);
    assert(p263_file_url_LEN(ph) == 82);
    {
        char16_t * exemplary = u"mdfjstnyntwtkewhbxfjjbgfdqurbmnrnQstqfbpowpUfmpJpKhqqtmxlplrcuCypqiyuohmyjzhplronj";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 164);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1182481144L);
    assert(p263_time_utc_GET(pack) == (uint64_t)6246139348066080893L);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)907661539495810316L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)599481002415728509L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)1003341798L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)2879844880056816946L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float)2.837548E38F);
    assert(p265_pitch_GET(pack) == (float) -9.549214E37F);
    assert(p265_yaw_GET(pack) == (float) -9.134388E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)727623439L);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)131, (uint8_t)199, (uint8_t)248, (uint8_t)100, (uint8_t)53, (uint8_t)26, (uint8_t)220, (uint8_t)91, (uint8_t)224, (uint8_t)27, (uint8_t)77, (uint8_t)35, (uint8_t)177, (uint8_t)83, (uint8_t)181, (uint8_t)205, (uint8_t)104, (uint8_t)231, (uint8_t)87, (uint8_t)87, (uint8_t)240, (uint8_t)248, (uint8_t)158, (uint8_t)249, (uint8_t)26, (uint8_t)124, (uint8_t)130, (uint8_t)35, (uint8_t)201, (uint8_t)227, (uint8_t)115, (uint8_t)208, (uint8_t)239, (uint8_t)82, (uint8_t)43, (uint8_t)253, (uint8_t)173, (uint8_t)38, (uint8_t)192, (uint8_t)69, (uint8_t)217, (uint8_t)125, (uint8_t)186, (uint8_t)174, (uint8_t)168, (uint8_t)72, (uint8_t)105, (uint8_t)20, (uint8_t)4, (uint8_t)255, (uint8_t)160, (uint8_t)158, (uint8_t)201, (uint8_t)181, (uint8_t)43, (uint8_t)170, (uint8_t)22, (uint8_t)83, (uint8_t)37, (uint8_t)44, (uint8_t)234, (uint8_t)241, (uint8_t)217, (uint8_t)197, (uint8_t)43, (uint8_t)31, (uint8_t)237, (uint8_t)221, (uint8_t)49, (uint8_t)226, (uint8_t)97, (uint8_t)184, (uint8_t)126, (uint8_t)0, (uint8_t)33, (uint8_t)9, (uint8_t)164, (uint8_t)27, (uint8_t)232, (uint8_t)230, (uint8_t)61, (uint8_t)123, (uint8_t)61, (uint8_t)184, (uint8_t)197, (uint8_t)8, (uint8_t)213, (uint8_t)229, (uint8_t)200, (uint8_t)5, (uint8_t)248, (uint8_t)159, (uint8_t)95, (uint8_t)243, (uint8_t)155, (uint8_t)213, (uint8_t)176, (uint8_t)161, (uint8_t)170, (uint8_t)158, (uint8_t)75, (uint8_t)67, (uint8_t)49, (uint8_t)200, (uint8_t)28, (uint8_t)54, (uint8_t)228, (uint8_t)170, (uint8_t)140, (uint8_t)97, (uint8_t)156, (uint8_t)192, (uint8_t)114, (uint8_t)119, (uint8_t)112, (uint8_t)52, (uint8_t)8, (uint8_t)41, (uint8_t)211, (uint8_t)194, (uint8_t)117, (uint8_t)213, (uint8_t)137, (uint8_t)11, (uint8_t)15, (uint8_t)106, (uint8_t)165, (uint8_t)194, (uint8_t)255, (uint8_t)108, (uint8_t)163, (uint8_t)88, (uint8_t)121, (uint8_t)13, (uint8_t)23, (uint8_t)2, (uint8_t)144, (uint8_t)187, (uint8_t)180, (uint8_t)78, (uint8_t)121, (uint8_t)118, (uint8_t)84, (uint8_t)201, (uint8_t)23, (uint8_t)253, (uint8_t)103, (uint8_t)6, (uint8_t)108, (uint8_t)174, (uint8_t)196, (uint8_t)84, (uint8_t)44, (uint8_t)134, (uint8_t)14, (uint8_t)24, (uint8_t)235, (uint8_t)217, (uint8_t)226, (uint8_t)90, (uint8_t)61, (uint8_t)219, (uint8_t)137, (uint8_t)115, (uint8_t)63, (uint8_t)48, (uint8_t)140, (uint8_t)47, (uint8_t)36, (uint8_t)184, (uint8_t)22, (uint8_t)64, (uint8_t)21, (uint8_t)239, (uint8_t)94, (uint8_t)158, (uint8_t)11, (uint8_t)119, (uint8_t)66, (uint8_t)212, (uint8_t)120, (uint8_t)9, (uint8_t)38, (uint8_t)205, (uint8_t)30, (uint8_t)210, (uint8_t)204, (uint8_t)135, (uint8_t)169, (uint8_t)188, (uint8_t)229, (uint8_t)162, (uint8_t)4, (uint8_t)52, (uint8_t)23, (uint8_t)145, (uint8_t)172, (uint8_t)64, (uint8_t)151, (uint8_t)200, (uint8_t)102, (uint8_t)134, (uint8_t)42, (uint8_t)25, (uint8_t)120, (uint8_t)236, (uint8_t)251, (uint8_t)203, (uint8_t)144, (uint8_t)11, (uint8_t)21, (uint8_t)241, (uint8_t)251, (uint8_t)207, (uint8_t)143, (uint8_t)229, (uint8_t)113, (uint8_t)211, (uint8_t)232, (uint8_t)130, (uint8_t)47, (uint8_t)119, (uint8_t)128, (uint8_t)113, (uint8_t)207, (uint8_t)18, (uint8_t)13, (uint8_t)190, (uint8_t)229, (uint8_t)227, (uint8_t)152, (uint8_t)18, (uint8_t)153, (uint8_t)246, (uint8_t)219, (uint8_t)113, (uint8_t)120, (uint8_t)100, (uint8_t)89, (uint8_t)211, (uint8_t)168, (uint8_t)1, (uint8_t)48, (uint8_t)65, (uint8_t)12, (uint8_t)40, (uint8_t)138, (uint8_t)111, (uint8_t)44} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)49471);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)33445);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    {
        uint8_t exemplary[] =  {(uint8_t)36, (uint8_t)243, (uint8_t)118, (uint8_t)198, (uint8_t)231, (uint8_t)3, (uint8_t)173, (uint8_t)37, (uint8_t)106, (uint8_t)58, (uint8_t)70, (uint8_t)223, (uint8_t)171, (uint8_t)86, (uint8_t)159, (uint8_t)217, (uint8_t)230, (uint8_t)238, (uint8_t)163, (uint8_t)240, (uint8_t)128, (uint8_t)255, (uint8_t)98, (uint8_t)202, (uint8_t)253, (uint8_t)111, (uint8_t)243, (uint8_t)102, (uint8_t)52, (uint8_t)95, (uint8_t)38, (uint8_t)134, (uint8_t)146, (uint8_t)241, (uint8_t)128, (uint8_t)52, (uint8_t)54, (uint8_t)151, (uint8_t)246, (uint8_t)109, (uint8_t)1, (uint8_t)236, (uint8_t)153, (uint8_t)139, (uint8_t)178, (uint8_t)130, (uint8_t)116, (uint8_t)67, (uint8_t)3, (uint8_t)116, (uint8_t)233, (uint8_t)207, (uint8_t)217, (uint8_t)204, (uint8_t)83, (uint8_t)141, (uint8_t)233, (uint8_t)238, (uint8_t)3, (uint8_t)32, (uint8_t)9, (uint8_t)180, (uint8_t)147, (uint8_t)170, (uint8_t)173, (uint8_t)242, (uint8_t)15, (uint8_t)250, (uint8_t)61, (uint8_t)161, (uint8_t)174, (uint8_t)88, (uint8_t)180, (uint8_t)252, (uint8_t)1, (uint8_t)64, (uint8_t)87, (uint8_t)66, (uint8_t)154, (uint8_t)19, (uint8_t)77, (uint8_t)179, (uint8_t)151, (uint8_t)147, (uint8_t)87, (uint8_t)76, (uint8_t)234, (uint8_t)134, (uint8_t)76, (uint8_t)65, (uint8_t)218, (uint8_t)72, (uint8_t)16, (uint8_t)186, (uint8_t)161, (uint8_t)67, (uint8_t)160, (uint8_t)48, (uint8_t)222, (uint8_t)169, (uint8_t)154, (uint8_t)13, (uint8_t)12, (uint8_t)122, (uint8_t)199, (uint8_t)53, (uint8_t)140, (uint8_t)188, (uint8_t)164, (uint8_t)241, (uint8_t)91, (uint8_t)39, (uint8_t)15, (uint8_t)234, (uint8_t)186, (uint8_t)202, (uint8_t)159, (uint8_t)49, (uint8_t)111, (uint8_t)245, (uint8_t)226, (uint8_t)190, (uint8_t)210, (uint8_t)104, (uint8_t)10, (uint8_t)21, (uint8_t)84, (uint8_t)191, (uint8_t)110, (uint8_t)193, (uint8_t)138, (uint8_t)79, (uint8_t)128, (uint8_t)148, (uint8_t)124, (uint8_t)95, (uint8_t)140, (uint8_t)53, (uint8_t)215, (uint8_t)17, (uint8_t)200, (uint8_t)74, (uint8_t)228, (uint8_t)72, (uint8_t)144, (uint8_t)91, (uint8_t)82, (uint8_t)254, (uint8_t)45, (uint8_t)117, (uint8_t)221, (uint8_t)249, (uint8_t)31, (uint8_t)248, (uint8_t)246, (uint8_t)216, (uint8_t)124, (uint8_t)85, (uint8_t)40, (uint8_t)75, (uint8_t)18, (uint8_t)178, (uint8_t)113, (uint8_t)237, (uint8_t)151, (uint8_t)73, (uint8_t)23, (uint8_t)140, (uint8_t)201, (uint8_t)187, (uint8_t)81, (uint8_t)20, (uint8_t)152, (uint8_t)244, (uint8_t)173, (uint8_t)65, (uint8_t)64, (uint8_t)141, (uint8_t)188, (uint8_t)190, (uint8_t)255, (uint8_t)163, (uint8_t)62, (uint8_t)46, (uint8_t)182, (uint8_t)26, (uint8_t)69, (uint8_t)68, (uint8_t)117, (uint8_t)138, (uint8_t)16, (uint8_t)189, (uint8_t)38, (uint8_t)186, (uint8_t)68, (uint8_t)103, (uint8_t)8, (uint8_t)101, (uint8_t)200, (uint8_t)152, (uint8_t)25, (uint8_t)135, (uint8_t)63, (uint8_t)162, (uint8_t)23, (uint8_t)10, (uint8_t)49, (uint8_t)166, (uint8_t)140, (uint8_t)120, (uint8_t)50, (uint8_t)12, (uint8_t)94, (uint8_t)130, (uint8_t)142, (uint8_t)5, (uint8_t)93, (uint8_t)249, (uint8_t)22, (uint8_t)141, (uint8_t)206, (uint8_t)101, (uint8_t)54, (uint8_t)113, (uint8_t)49, (uint8_t)97, (uint8_t)75, (uint8_t)226, (uint8_t)69, (uint8_t)230, (uint8_t)186, (uint8_t)189, (uint8_t)45, (uint8_t)136, (uint8_t)154, (uint8_t)106, (uint8_t)247, (uint8_t)142, (uint8_t)159, (uint8_t)211, (uint8_t)83, (uint8_t)255, (uint8_t)149, (uint8_t)6, (uint8_t)225, (uint8_t)101, (uint8_t)207, (uint8_t)237, (uint8_t)212} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)7085);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)50210);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)21600);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p269_bitrate_GET(pack) == (uint32_t)2961916442L);
    assert(p269_uri_LEN(ph) == 60);
    {
        char16_t * exemplary = u"tgbyacpUicxbaryfsvChuqwsEAPewgvrfusxwmoPuPsbvkzloacqlmBkcNdf";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float) -2.0645676E38F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)33685);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)49855);
    assert(p270_bitrate_GET(pack) == (uint32_t)1056494506L);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p270_uri_LEN(ph) == 16);
    {
        char16_t * exemplary = u"FwareNlcxfjsqbes";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)61078);
    assert(p270_framerate_GET(pack) == (float) -3.3073377E38F);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)25238);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)63);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 4);
    {
        char16_t * exemplary = u"zekc";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 47);
    {
        char16_t * exemplary = u"pqzggbcmfhuaAmuiytxfbpvxcMautcFpzokzchonxzaxupz";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 94);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)4388);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)1867);
    {
        uint8_t exemplary[] =  {(uint8_t)80, (uint8_t)50, (uint8_t)19, (uint8_t)132, (uint8_t)4, (uint8_t)119, (uint8_t)46, (uint8_t)32} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)112, (uint8_t)226, (uint8_t)155, (uint8_t)69, (uint8_t)212, (uint8_t)126, (uint8_t)190, (uint8_t)16} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)10012);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)35153);
    assert(p310_time_usec_GET(pack) == (uint64_t)5558423984996120340L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1141044434L);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)87);
    {
        uint8_t exemplary[] =  {(uint8_t)224, (uint8_t)167, (uint8_t)104, (uint8_t)247, (uint8_t)196, (uint8_t)244, (uint8_t)92, (uint8_t)193, (uint8_t)56, (uint8_t)235, (uint8_t)89, (uint8_t)254, (uint8_t)64, (uint8_t)12, (uint8_t)105, (uint8_t)248} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2183409286L);
    assert(p311_time_usec_GET(pack) == (uint64_t)7788764367137332022L);
    assert(p311_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"p";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2862478411L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)91);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"ziXpu";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)20420);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)123);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)48);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)36467);
    assert(p322_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"b";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 58);
    {
        char16_t * exemplary = u"UaamslhkbcncuxfifoyqcWrPGzzlteqQirTkcXbdszbmvrvewssvoxxjkz";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)17426);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p323_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"wjwttpaqffOlkkN";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_value_LEN(ph) == 58);
    {
        char16_t * exemplary = u"ymraDLyrCkHxtVptymjacernwPmsnomdhhnlzhadfefwZqzVegrxMfkhdh";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"VemsgvdgQC";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_value_LEN(ph) == 105);
    {
        char16_t * exemplary = u"rcNvjjnpefCouiIxhrkyrblnbhklsbytxfukqLhqnfropwglyfgjtvAvAsTsudwiMqGwyqbyzymEtbwwvyelzpxrevvMRsszbuhtobefk";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 210);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)38382);
    {
        uint16_t exemplary[] =  {(uint16_t)18929, (uint16_t)65266, (uint16_t)6479, (uint16_t)44526, (uint16_t)20192, (uint16_t)28755, (uint16_t)59046, (uint16_t)27053, (uint16_t)58578, (uint16_t)61766, (uint16_t)65210, (uint16_t)3504, (uint16_t)48029, (uint16_t)49970, (uint16_t)9244, (uint16_t)53299, (uint16_t)15637, (uint16_t)40710, (uint16_t)40312, (uint16_t)20625, (uint16_t)60240, (uint16_t)21105, (uint16_t)15057, (uint16_t)56352, (uint16_t)10813, (uint16_t)6801, (uint16_t)42927, (uint16_t)59858, (uint16_t)64953, (uint16_t)13150, (uint16_t)17510, (uint16_t)26890, (uint16_t)21058, (uint16_t)58052, (uint16_t)32292, (uint16_t)39314, (uint16_t)45020, (uint16_t)60812, (uint16_t)43587, (uint16_t)61445, (uint16_t)56909, (uint16_t)51111, (uint16_t)55356, (uint16_t)38722, (uint16_t)15219, (uint16_t)46424, (uint16_t)62370, (uint16_t)880, (uint16_t)58627, (uint16_t)55996, (uint16_t)11930, (uint16_t)31163, (uint16_t)21917, (uint16_t)31049, (uint16_t)30821, (uint16_t)17747, (uint16_t)38075, (uint16_t)53359, (uint16_t)41913, (uint16_t)2955, (uint16_t)38576, (uint16_t)11150, (uint16_t)34597, (uint16_t)58687, (uint16_t)46829, (uint16_t)14596, (uint16_t)48346, (uint16_t)41836, (uint16_t)27493, (uint16_t)27612, (uint16_t)27955, (uint16_t)61971} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)54669);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p330_time_usec_GET(pack) == (uint64_t)5565474932719285631L);
};



Pack * c_TEST_Channel_process(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id) /*220*/
        {
            default:
                assert(0);
                return NULL;
            case PROCESS_CHANNEL_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
                if(RBUF_ISFULL(received_packs)) return pack;
                RBUF_PUT(received_packs, pack)
                return NULL;
            case PROCESS_HOST_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(received_packs)) ? NULL : RBUF_GET(received_packs);
                if(RBUF_ISFULL(sendout_packs)) return pack;
                RBUF_PUT(sendout_packs, pack)
                return NULL;
            case PROCESS_RECEIVED_PACKS:
                LOOP = true;
                goto next_pack;
        }
        free(pack); // dispose pack
        if(!LOOP) return NULL;
next_pack:
        if(RBUF_ISEMPTY(received_packs)) return NULL;
        pack = RBUF_GET(received_packs);
        id = pack->meta->id;
        setPack(pack, &ph);
    }
}
Channel c_TEST_Channel = {.process = c_TEST_Channel_process};


void c_TEST_Channel_process_received() { c_TEST_Channel_process(NULL, PROCESS_RECEIVED_PACKS); }
void c_TEST_Channel_send(Pack* pack) { c_TEST_Channel_process(pack, PROCESS_HOST_REQEST); }
int main()
{
    static uint8_t buff[512];
    static Bounds_Inside PH;
    {
        setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_STANDBY, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_PARAFOIL, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)3617508889L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count2_SET((uint16_t)(uint16_t)36551, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)11, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)26092, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)48120, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)16335, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)30484, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)34565, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)60072, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)15738, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)30051, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)3483604726L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)7455903056027467077L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_type_mask_SET((uint16_t)(uint16_t)29113, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.4493706E38F, PH.base.pack) ;
        p3_afz_SET((float) -3.0350421E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p3_x_SET((float) -2.8317927E38F, PH.base.pack) ;
        p3_afx_SET((float) -4.3503096E37F, PH.base.pack) ;
        p3_afy_SET((float) -1.0408845E38F, PH.base.pack) ;
        p3_vx_SET((float)3.7031314E37F, PH.base.pack) ;
        p3_y_SET((float) -8.4528985E37F, PH.base.pack) ;
        p3_yaw_SET((float)7.614097E37F, PH.base.pack) ;
        p3_vy_SET((float) -2.2582429E38F, PH.base.pack) ;
        p3_vz_SET((float)1.9146382E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3764808958L, PH.base.pack) ;
        p3_z_SET((float)1.6214277E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)6016755486125480140L, PH.base.pack) ;
        p4_seq_SET((uint32_t)1428375337L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"mqskttq";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"oelpiKydKKpbqtep";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)4125735860L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"dgXge";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)21043, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)42015, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)58412, PH.base.pack) ;
        p22_param_value_SET((float) -7.6812086E37F, PH.base.pack) ;
        {
            char16_t* param_id = u"oJwafVgglF";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        {
            char16_t* param_id = u"mTjnqPeu";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_value_SET((float) -3.3026468E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)1099497925L, &PH) ;
        p24_v_acc_SET((uint32_t)1910064237L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)38631, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)50839, PH.base.pack) ;
        p24_lon_SET((int32_t) -1123758319, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)48238, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)2366603790L, &PH) ;
        p24_alt_SET((int32_t)533595427, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)528295584L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)16182, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)1577532863152499062L, PH.base.pack) ;
        p24_lat_SET((int32_t) -1252323868, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)1699637358, &PH) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        p25_satellites_visible_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)49, (uint8_t)15, (uint8_t)195, (uint8_t)106, (uint8_t)234, (uint8_t)177, (uint8_t)29, (uint8_t)13, (uint8_t)167, (uint8_t)109, (uint8_t)30, (uint8_t)162, (uint8_t)232, (uint8_t)203, (uint8_t)102, (uint8_t)168, (uint8_t)227, (uint8_t)84, (uint8_t)218, (uint8_t)2};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)168, (uint8_t)249, (uint8_t)100, (uint8_t)84, (uint8_t)24, (uint8_t)139, (uint8_t)147, (uint8_t)247, (uint8_t)194, (uint8_t)223, (uint8_t)248, (uint8_t)164, (uint8_t)111, (uint8_t)83, (uint8_t)249, (uint8_t)110, (uint8_t)105, (uint8_t)10, (uint8_t)32, (uint8_t)125};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)0, (uint8_t)12, (uint8_t)150, (uint8_t)33, (uint8_t)42, (uint8_t)33, (uint8_t)151, (uint8_t)20, (uint8_t)4, (uint8_t)203, (uint8_t)53, (uint8_t)101, (uint8_t)129, (uint8_t)234, (uint8_t)159, (uint8_t)92, (uint8_t)198, (uint8_t)29, (uint8_t)126, (uint8_t)219};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)167, (uint8_t)20, (uint8_t)238, (uint8_t)96, (uint8_t)129, (uint8_t)147, (uint8_t)190, (uint8_t)131, (uint8_t)184, (uint8_t)48, (uint8_t)1, (uint8_t)18, (uint8_t)82, (uint8_t)196, (uint8_t)255, (uint8_t)141, (uint8_t)38, (uint8_t)182, (uint8_t)230, (uint8_t)134};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)69, (uint8_t)201, (uint8_t)204, (uint8_t)229, (uint8_t)71, (uint8_t)52, (uint8_t)32, (uint8_t)92, (uint8_t)224, (uint8_t)233, (uint8_t)99, (uint8_t)192, (uint8_t)215, (uint8_t)117, (uint8_t)106, (uint8_t)199, (uint8_t)138, (uint8_t)230, (uint8_t)100, (uint8_t)199};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_yacc_SET((int16_t)(int16_t) -8275, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -12380, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)19059, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -20988, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)11809, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -24363, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -22419, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)4067460867L, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)14279, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -4536, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_xacc_SET((int16_t)(int16_t)21793, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)8861, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)4206, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)3898387824519353061L, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)31576, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -26605, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)6868, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)14643, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -24579, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -7580, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_abs_SET((int16_t)(int16_t)23813, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -17583, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)6933831594040421553L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)10121, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)8206, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float) -2.1091337E38F, PH.base.pack) ;
        p29_press_abs_SET((float)1.4726469E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)4006652409L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)14315, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float) -7.508522E37F, PH.base.pack) ;
        p30_pitchspeed_SET((float)2.075219E38F, PH.base.pack) ;
        p30_roll_SET((float) -1.1657487E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)1.6320244E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3227982141L, PH.base.pack) ;
        p30_yawspeed_SET((float) -9.4642E37F, PH.base.pack) ;
        p30_pitch_SET((float)2.7141961E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_rollspeed_SET((float) -8.632634E37F, PH.base.pack) ;
        p31_q2_SET((float)5.2415704E37F, PH.base.pack) ;
        p31_q4_SET((float) -1.0680295E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -2.0527486E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -3.7193013E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2526476201L, PH.base.pack) ;
        p31_q3_SET((float)2.733698E38F, PH.base.pack) ;
        p31_q1_SET((float) -2.825359E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)2820412127L, PH.base.pack) ;
        p32_x_SET((float)7.701238E37F, PH.base.pack) ;
        p32_vx_SET((float)1.6011387E38F, PH.base.pack) ;
        p32_vz_SET((float) -1.9379143E38F, PH.base.pack) ;
        p32_z_SET((float)2.8486131E38F, PH.base.pack) ;
        p32_y_SET((float)1.6163105E38F, PH.base.pack) ;
        p32_vy_SET((float)7.4436195E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_hdg_SET((uint16_t)(uint16_t)39130, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)3393130207L, PH.base.pack) ;
        p33_alt_SET((int32_t) -2116483046, PH.base.pack) ;
        p33_lat_SET((int32_t)1844865853, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)10896, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1463293013, PH.base.pack) ;
        p33_lon_SET((int32_t) -1476077417, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -18210, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)30820, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_port_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -608, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -456, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)1404, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)19027, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)17074, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)23449, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -26819, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2880283449L, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -26475, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)37378, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)47780, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)12933, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)55581, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)23438, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)59645, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)560, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)638735234L, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)16624, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo7_raw_SET((uint16_t)(uint16_t)5565, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)1212007112L, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)50737, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)40970, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)44946, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)36829, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)10332, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)42586, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)26734, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)65099, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)9892, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)49516, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)57279, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)3250, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)64442, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)46875, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)16113, &PH) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)31727, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)29352, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_start_index_SET((int16_t)(int16_t) -14597, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)6593, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)19343, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p39_y_SET((float) -8.810614E36F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p39_param3_SET((float) -1.1451857E38F, PH.base.pack) ;
        p39_param1_SET((float)3.7062123E37F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p39_z_SET((float) -2.5521435E38F, PH.base.pack) ;
        p39_param4_SET((float) -3.1340144E38F, PH.base.pack) ;
        p39_x_SET((float) -4.1689068E37F, PH.base.pack) ;
        p39_param2_SET((float) -2.172295E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)41474, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)49201, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)36852, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)54055, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)22983, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t) -222548970, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p48_altitude_SET((int32_t)1706791551, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)8740216364329059295L, &PH) ;
        p48_latitude_SET((int32_t) -1122936066, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t)1839456416, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1162562093, PH.base.pack) ;
        p49_longitude_SET((int32_t)1387480659, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)7444961138158831020L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_min_SET((float) -2.0173343E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.951122E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"jlverfXbhcbUv";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_scale_SET((float) -6.2502557E37F, PH.base.pack) ;
        p50_param_value0_SET((float) -3.1013112E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)25538, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)59596, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p54_p2y_SET((float) -1.9521834E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p54_p1y_SET((float)1.3460432E38F, PH.base.pack) ;
        p54_p1z_SET((float)1.6721672E38F, PH.base.pack) ;
        p54_p2x_SET((float)1.2959997E38F, PH.base.pack) ;
        p54_p1x_SET((float)2.994492E38F, PH.base.pack) ;
        p54_p2z_SET((float) -1.6964131E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float) -2.300767E38F, PH.base.pack) ;
        p55_p2z_SET((float)1.3000434E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p55_p1x_SET((float) -3.126215E37F, PH.base.pack) ;
        p55_p1z_SET((float)1.7503173E37F, PH.base.pack) ;
        p55_p1y_SET((float) -1.3039103E38F, PH.base.pack) ;
        p55_p2y_SET((float) -3.1619782E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        {
            float q[] =  {-3.391345E37F, 1.0572284E38F, -2.0668305E38F, -3.1345902E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)2.0019258E38F, PH.base.pack) ;
        p61_yawspeed_SET((float) -1.2578748E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.108803E38F, 1.7025884E38F, -1.3794083E38F, -2.9264087E38F, 2.8564018E38F, 1.7094767E38F, 3.0830956E38F, -6.8458984E37F, -2.7950333E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)2027791967211918421L, PH.base.pack) ;
        p61_pitchspeed_SET((float)9.989793E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_target_bearing_SET((int16_t)(int16_t)25514, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -9793, PH.base.pack) ;
        p62_nav_roll_SET((float)1.1504797E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)33219, PH.base.pack) ;
        p62_alt_error_SET((float) -3.0891848E38F, PH.base.pack) ;
        p62_aspd_error_SET((float)7.2696816E37F, PH.base.pack) ;
        p62_nav_pitch_SET((float)2.6911632E38F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -1.7783272E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lon_SET((int32_t)1034173399, PH.base.pack) ;
        p63_lat_SET((int32_t)1986298089, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)6391039618137430672L, PH.base.pack) ;
        p63_vz_SET((float)3.2373557E38F, PH.base.pack) ;
        p63_alt_SET((int32_t)298516236, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_vy_SET((float)9.539879E37F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -367647136, PH.base.pack) ;
        p63_vx_SET((float)7.23833E37F, PH.base.pack) ;
        {
            float covariance[] =  {3.0005324E38F, -3.0656065E38F, -2.9412519E37F, 2.902984E38F, 3.1711854E38F, -1.7256475E38F, 2.1323828E38F, -2.5616367E38F, 4.4682757E37F, -3.1932474E38F, 7.307989E37F, -3.345869E38F, -2.0500587E38F, -1.4007778E36F, 2.921074E38F, -7.323332E37F, -2.9377312E38F, 2.6275397E38F, -2.6006448E38F, 1.7188746E38F, 2.1920119E38F, -2.053355E38F, 2.3375518E38F, -2.1780103E38F, -2.9451363E38F, -2.4323698E38F, -2.398968E38F, 5.151753E37F, -3.1523953E38F, -1.4156894E38F, -3.0402728E38F, 2.7337733E38F, 2.3061244E38F, 2.4130135E38F, 5.109266E37F, -3.3395766E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)4587342838889156421L, PH.base.pack) ;
        p64_vx_SET((float)2.8738609E38F, PH.base.pack) ;
        p64_vy_SET((float)2.9969668E38F, PH.base.pack) ;
        p64_az_SET((float) -1.1913441E38F, PH.base.pack) ;
        p64_x_SET((float) -2.8144652E38F, PH.base.pack) ;
        p64_y_SET((float)2.9551106E38F, PH.base.pack) ;
        p64_ax_SET((float)1.7176637E38F, PH.base.pack) ;
        p64_z_SET((float)3.0782433E38F, PH.base.pack) ;
        {
            float covariance[] =  {1.6568051E37F, -1.6586529E37F, 3.0643654E38F, -2.4797596E38F, 4.9344618E36F, -8.842301E37F, -3.363545E38F, 8.612128E37F, 1.0828075E38F, 1.283671E37F, -1.9150817E38F, -1.8925212E38F, -5.1305957E37F, -1.0551119E38F, 1.2599584E38F, -2.3406039E38F, 2.0024855E38F, -1.7743545E38F, 2.905582E38F, 9.027283E37F, 1.4216638E38F, -2.4418725E38F, 7.277873E36F, 1.0867775E38F, -5.273786E37F, 3.9209204E37F, 4.0969887E37F, 5.4108733E37F, 2.4326909E38F, 1.058768E38F, 2.1807606E38F, -3.5542456E37F, 2.2019835E38F, -2.660663E38F, -1.9945774E38F, -1.8458814E38F, 1.3935153E38F, -3.0834426E38F, 3.017297E38F, -1.177706E38F, 6.801151E37F, 1.0918232E38F, -1.3379232E38F, -2.5226387E38F, 3.3453618E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vz_SET((float)1.4863218E37F, PH.base.pack) ;
        p64_ay_SET((float) -4.785444E37F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan1_raw_SET((uint16_t)(uint16_t)5540, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)32269, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)63737, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)23123, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)65023, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)40691, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)882, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)688245098L, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)29725, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)50387, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)27583, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)52762, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)2343, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)49913, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)52540, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)3247, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)6990, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)55756, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)53372, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)57269, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)32544, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)16549, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -16423, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)5206, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -21095, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -25459, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan1_raw_SET((uint16_t)(uint16_t)49695, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)4347, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)59272, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)20233, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)12093, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)25928, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)55092, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)37973, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_x_SET((int32_t) -567850687, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p73_z_SET((float)8.589111E37F, PH.base.pack) ;
        p73_param3_SET((float)2.835644E38F, PH.base.pack) ;
        p73_param4_SET((float) -3.213096E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p73_y_SET((int32_t)1523056048, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p73_param1_SET((float)1.461516E38F, PH.base.pack) ;
        p73_param2_SET((float)1.5421401E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)6634, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float) -1.9747439E38F, PH.base.pack) ;
        p74_alt_SET((float) -2.3465373E38F, PH.base.pack) ;
        p74_airspeed_SET((float)3.1280617E38F, PH.base.pack) ;
        p74_climb_SET((float)3.3127785E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -27810, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)62915, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p75_y_SET((int32_t) -676236502, PH.base.pack) ;
        p75_param3_SET((float)3.338227E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p75_x_SET((int32_t)1787578862, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p75_param2_SET((float) -4.443308E37F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p75_z_SET((float) -1.0416443E38F, PH.base.pack) ;
        p75_param1_SET((float)2.4395569E38F, PH.base.pack) ;
        p75_param4_SET((float)2.434049E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param4_SET((float)3.292259E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p76_param5_SET((float)3.3381366E38F, PH.base.pack) ;
        p76_param6_SET((float)2.9327314E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p76_param2_SET((float) -3.236385E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_MISSION_START, PH.base.pack) ;
        p76_param7_SET((float) -3.242351E38F, PH.base.pack) ;
        p76_param3_SET((float) -1.102308E38F, PH.base.pack) ;
        p76_param1_SET((float)2.257781E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)110, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)222, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)157, &PH) ;
        p77_result_param2_SET((int32_t)24463716, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float) -2.8416605E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1682517381L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p81_roll_SET((float)2.188377E38F, PH.base.pack) ;
        p81_yaw_SET((float) -2.923228E38F, PH.base.pack) ;
        p81_pitch_SET((float) -2.3800865E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p82_thrust_SET((float) -4.220465E37F, PH.base.pack) ;
        {
            float q[] =  {2.4748552E38F, 6.0566556E37F, 2.536195E38F, 1.929861E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_time_boot_ms_SET((uint32_t)2500219612L, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)9.893436E37F, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -1.3796622E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -3.2221007E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_thrust_SET((float) -1.8613275E38F, PH.base.pack) ;
        {
            float q[] =  {1.7721518E38F, 3.0902537E38F, 2.87777E38F, -9.584393E36F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)2.0433104E38F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)2.2451112E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)2672265594L, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -2.4911162E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_time_boot_ms_SET((uint32_t)3796226928L, PH.base.pack) ;
        p84_z_SET((float) -2.6593703E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.1441252E38F, PH.base.pack) ;
        p84_vx_SET((float) -2.0397658E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p84_yaw_SET((float) -2.359963E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)61493, PH.base.pack) ;
        p84_vy_SET((float) -4.5610284E36F, PH.base.pack) ;
        p84_x_SET((float) -1.3308509E38F, PH.base.pack) ;
        p84_vz_SET((float)1.0527862E38F, PH.base.pack) ;
        p84_afy_SET((float) -2.5444062E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p84_y_SET((float) -1.6850334E38F, PH.base.pack) ;
        p84_afz_SET((float) -1.4492079E38F, PH.base.pack) ;
        p84_afx_SET((float)2.843456E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_lon_int_SET((int32_t) -1260717749, PH.base.pack) ;
        p86_lat_int_SET((int32_t)784125164, PH.base.pack) ;
        p86_vz_SET((float) -2.8427608E38F, PH.base.pack) ;
        p86_vx_SET((float)5.366485E36F, PH.base.pack) ;
        p86_afx_SET((float) -2.0282502E37F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)30554, PH.base.pack) ;
        p86_yaw_SET((float)1.1623619E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p86_afz_SET((float) -2.3382773E38F, PH.base.pack) ;
        p86_alt_SET((float) -5.064966E37F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3143850606L, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p86_vy_SET((float) -2.7623715E38F, PH.base.pack) ;
        p86_afy_SET((float) -2.6553599E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p86_yaw_rate_SET((float) -3.240795E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afz_SET((float) -1.1373055E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)766756140, PH.base.pack) ;
        p87_yaw_SET((float) -1.3985569E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p87_afy_SET((float) -2.5990273E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)1607, PH.base.pack) ;
        p87_vy_SET((float) -3.3192147E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float) -2.929623E38F, PH.base.pack) ;
        p87_afx_SET((float) -1.8007295E38F, PH.base.pack) ;
        p87_vx_SET((float)1.6270098E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)160637390L, PH.base.pack) ;
        p87_vz_SET((float)2.4959466E38F, PH.base.pack) ;
        p87_alt_SET((float)2.7313467E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -68437071, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_yaw_SET((float)1.6791922E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)923618542L, PH.base.pack) ;
        p89_y_SET((float) -3.0559117E38F, PH.base.pack) ;
        p89_x_SET((float)1.7289902E38F, PH.base.pack) ;
        p89_pitch_SET((float) -2.4562953E38F, PH.base.pack) ;
        p89_roll_SET((float)2.4542022E38F, PH.base.pack) ;
        p89_z_SET((float)8.778696E34F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_pitchspeed_SET((float)6.8162947E37F, PH.base.pack) ;
        p90_roll_SET((float)4.1556378E37F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -29840, PH.base.pack) ;
        p90_yaw_SET((float) -3.0691571E38F, PH.base.pack) ;
        p90_alt_SET((int32_t) -507710499, PH.base.pack) ;
        p90_pitch_SET((float) -5.3456527E37F, PH.base.pack) ;
        p90_lon_SET((int32_t)863569409, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)10258, PH.base.pack) ;
        p90_yawspeed_SET((float) -8.0923417E37F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -32530, PH.base.pack) ;
        p90_rollspeed_SET((float) -2.7190795E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)3447564714181013022L, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -32449, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)22132, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)4926, PH.base.pack) ;
        p90_lat_SET((int32_t)916457848, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p91_throttle_SET((float)9.491502E37F, PH.base.pack) ;
        p91_aux2_SET((float) -4.3006333E36F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)2.3599348E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)4341852157303520636L, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p91_aux1_SET((float) -1.4373478E38F, PH.base.pack) ;
        p91_aux3_SET((float) -2.0237713E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)2.7379866E38F, PH.base.pack) ;
        p91_aux4_SET((float) -4.188664E37F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.959696E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan5_raw_SET((uint16_t)(uint16_t)26685, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)40927, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)52601, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)535, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)40366, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)44697, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)4317, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)34747, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)9331, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)60064, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)56621, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)47446, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)957206696110007356L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {2.3424153E38F, 1.9260963E38F, -3.030215E38F, 2.2887008E38F, 3.318765E38F, 7.801857E36F, -5.549598E37F, -2.2479252E38F, -2.703805E38F, 3.3163738E38F, -4.262854E37F, 2.7588196E38F, -1.6102602E38F, -1.9484132E38F, -3.087779E38F, -1.0329464E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_time_usec_SET((uint64_t)5889341250926626420L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)8752074945038503827L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_y_SET((int16_t)(int16_t)24735, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)7763854895862686174L, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.7177292E38F, &PH) ;
        p100_flow_comp_m_x_SET((float) -1.1390192E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)3.0318324E38F, &PH) ;
        p100_flow_x_SET((int16_t)(int16_t)24797, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)9.797926E37F, PH.base.pack) ;
        p100_ground_distance_SET((float) -2.2328844E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_yaw_SET((float)7.239128E37F, PH.base.pack) ;
        p101_y_SET((float) -3.3241478E38F, PH.base.pack) ;
        p101_z_SET((float) -7.7209267E37F, PH.base.pack) ;
        p101_x_SET((float) -3.1666423E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)498736983225992002L, PH.base.pack) ;
        p101_roll_SET((float) -1.471016E38F, PH.base.pack) ;
        p101_pitch_SET((float)1.7960997E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_yaw_SET((float) -3.1047631E38F, PH.base.pack) ;
        p102_y_SET((float) -4.5523245E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)4911048895249079937L, PH.base.pack) ;
        p102_pitch_SET((float)2.939542E38F, PH.base.pack) ;
        p102_z_SET((float)2.4484337E38F, PH.base.pack) ;
        p102_x_SET((float) -3.195099E38F, PH.base.pack) ;
        p102_roll_SET((float)9.709445E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -1.645416E38F, PH.base.pack) ;
        p103_y_SET((float) -1.7685377E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)3265568937877337875L, PH.base.pack) ;
        p103_x_SET((float) -1.6243359E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_roll_SET((float)2.9616658E38F, PH.base.pack) ;
        p104_x_SET((float)3.0280911E38F, PH.base.pack) ;
        p104_z_SET((float)7.562716E37F, PH.base.pack) ;
        p104_y_SET((float) -1.0649707E38F, PH.base.pack) ;
        p104_pitch_SET((float)2.4189149E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)1918031982379744370L, PH.base.pack) ;
        p104_yaw_SET((float) -8.244017E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_ygyro_SET((float)6.3474967E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -1.7654166E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.1903938E37F, PH.base.pack) ;
        p105_xmag_SET((float) -2.9743598E38F, PH.base.pack) ;
        p105_yacc_SET((float) -4.446091E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float)2.5709122E38F, PH.base.pack) ;
        p105_xacc_SET((float) -1.2509853E38F, PH.base.pack) ;
        p105_zacc_SET((float)6.6506756E37F, PH.base.pack) ;
        p105_xgyro_SET((float)1.2908451E38F, PH.base.pack) ;
        p105_temperature_SET((float)3.1427963E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4564595723709509319L, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)29146, PH.base.pack) ;
        p105_ymag_SET((float)6.7563566E37F, PH.base.pack) ;
        p105_zmag_SET((float)7.5062556E37F, PH.base.pack) ;
        p105_zgyro_SET((float) -1.5315398E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_xgyro_SET((float) -8.654267E37F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)16865, PH.base.pack) ;
        p106_integrated_y_SET((float) -2.4607356E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)638273837L, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6584155107695955668L, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p106_distance_SET((float)3.3831516E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p106_integrated_x_SET((float)1.7000046E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)6.018457E37F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)3.1871215E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3273405669L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_ymag_SET((float) -1.0368956E38F, PH.base.pack) ;
        p107_zmag_SET((float)2.7040164E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -2.4415762E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -3.0530198E38F, PH.base.pack) ;
        p107_zacc_SET((float)1.5552454E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)3133699325L, PH.base.pack) ;
        p107_ygyro_SET((float) -1.8384848E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -3.2676245E37F, PH.base.pack) ;
        p107_xacc_SET((float)6.4466716E37F, PH.base.pack) ;
        p107_temperature_SET((float) -2.6580256E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -1.4399179E38F, PH.base.pack) ;
        p107_xmag_SET((float)4.1332204E37F, PH.base.pack) ;
        p107_yacc_SET((float) -7.4801907E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)7885941136092111018L, PH.base.pack) ;
        p107_diff_pressure_SET((float)2.5824938E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_zacc_SET((float)5.8608406E37F, PH.base.pack) ;
        p108_vd_SET((float) -8.514871E37F, PH.base.pack) ;
        p108_yacc_SET((float)2.7717784E37F, PH.base.pack) ;
        p108_lat_SET((float)1.0806649E38F, PH.base.pack) ;
        p108_roll_SET((float) -2.3329314E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.4784305E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -8.193333E37F, PH.base.pack) ;
        p108_xacc_SET((float) -3.1031008E38F, PH.base.pack) ;
        p108_vn_SET((float) -2.8212233E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -1.9438864E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -6.0401553E37F, PH.base.pack) ;
        p108_lon_SET((float) -3.1920255E38F, PH.base.pack) ;
        p108_zgyro_SET((float)2.9587237E38F, PH.base.pack) ;
        p108_q3_SET((float)1.6711406E38F, PH.base.pack) ;
        p108_ve_SET((float) -1.8168112E38F, PH.base.pack) ;
        p108_q1_SET((float) -2.3387914E38F, PH.base.pack) ;
        p108_alt_SET((float) -2.5369098E38F, PH.base.pack) ;
        p108_yaw_SET((float)5.735078E37F, PH.base.pack) ;
        p108_q2_SET((float)4.7518963E37F, PH.base.pack) ;
        p108_q4_SET((float)1.6076391E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -2.0098345E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)48485, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)54302, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)14, (uint8_t)20, (uint8_t)66, (uint8_t)83, (uint8_t)122, (uint8_t)228, (uint8_t)39, (uint8_t)250, (uint8_t)170, (uint8_t)71, (uint8_t)219, (uint8_t)71, (uint8_t)15, (uint8_t)251, (uint8_t)47, (uint8_t)91, (uint8_t)12, (uint8_t)255, (uint8_t)77, (uint8_t)98, (uint8_t)203, (uint8_t)63, (uint8_t)180, (uint8_t)219, (uint8_t)136, (uint8_t)48, (uint8_t)152, (uint8_t)87, (uint8_t)76, (uint8_t)129, (uint8_t)67, (uint8_t)97, (uint8_t)159, (uint8_t)94, (uint8_t)239, (uint8_t)215, (uint8_t)112, (uint8_t)39, (uint8_t)217, (uint8_t)255, (uint8_t)176, (uint8_t)83, (uint8_t)6, (uint8_t)143, (uint8_t)211, (uint8_t)6, (uint8_t)63, (uint8_t)185, (uint8_t)119, (uint8_t)186, (uint8_t)161, (uint8_t)101, (uint8_t)163, (uint8_t)66, (uint8_t)190, (uint8_t)124, (uint8_t)16, (uint8_t)208, (uint8_t)254, (uint8_t)154, (uint8_t)135, (uint8_t)238, (uint8_t)80, (uint8_t)77, (uint8_t)169, (uint8_t)37, (uint8_t)250, (uint8_t)206, (uint8_t)220, (uint8_t)166, (uint8_t)161, (uint8_t)89, (uint8_t)159, (uint8_t)79, (uint8_t)79, (uint8_t)216, (uint8_t)192, (uint8_t)147, (uint8_t)85, (uint8_t)184, (uint8_t)9, (uint8_t)175, (uint8_t)82, (uint8_t)74, (uint8_t)168, (uint8_t)15, (uint8_t)152, (uint8_t)223, (uint8_t)60, (uint8_t)117, (uint8_t)67, (uint8_t)201, (uint8_t)149, (uint8_t)69, (uint8_t)166, (uint8_t)183, (uint8_t)84, (uint8_t)198, (uint8_t)194, (uint8_t)195, (uint8_t)216, (uint8_t)206, (uint8_t)252, (uint8_t)14, (uint8_t)20, (uint8_t)14, (uint8_t)206, (uint8_t)13, (uint8_t)243, (uint8_t)98, (uint8_t)56, (uint8_t)193, (uint8_t)155, (uint8_t)151, (uint8_t)184, (uint8_t)179, (uint8_t)29, (uint8_t)207, (uint8_t)182, (uint8_t)180, (uint8_t)16, (uint8_t)85, (uint8_t)190, (uint8_t)154, (uint8_t)110, (uint8_t)86, (uint8_t)157, (uint8_t)65, (uint8_t)30, (uint8_t)49, (uint8_t)195, (uint8_t)84, (uint8_t)181, (uint8_t)93, (uint8_t)151, (uint8_t)75, (uint8_t)152, (uint8_t)110, (uint8_t)241, (uint8_t)58, (uint8_t)236, (uint8_t)214, (uint8_t)69, (uint8_t)230, (uint8_t)103, (uint8_t)237, (uint8_t)171, (uint8_t)181, (uint8_t)63, (uint8_t)125, (uint8_t)117, (uint8_t)92, (uint8_t)3, (uint8_t)42, (uint8_t)70, (uint8_t)97, (uint8_t)171, (uint8_t)46, (uint8_t)62, (uint8_t)94, (uint8_t)59, (uint8_t)99, (uint8_t)223, (uint8_t)137, (uint8_t)180, (uint8_t)138, (uint8_t)152, (uint8_t)184, (uint8_t)110, (uint8_t)35, (uint8_t)54, (uint8_t)104, (uint8_t)84, (uint8_t)38, (uint8_t)218, (uint8_t)206, (uint8_t)109, (uint8_t)175, (uint8_t)122, (uint8_t)64, (uint8_t)1, (uint8_t)242, (uint8_t)118, (uint8_t)193, (uint8_t)11, (uint8_t)197, (uint8_t)24, (uint8_t)74, (uint8_t)128, (uint8_t)69, (uint8_t)75, (uint8_t)0, (uint8_t)151, (uint8_t)157, (uint8_t)24, (uint8_t)212, (uint8_t)18, (uint8_t)8, (uint8_t)105, (uint8_t)249, (uint8_t)254, (uint8_t)241, (uint8_t)199, (uint8_t)88, (uint8_t)74, (uint8_t)90, (uint8_t)52, (uint8_t)7, (uint8_t)14, (uint8_t)71, (uint8_t)39, (uint8_t)177, (uint8_t)9, (uint8_t)175, (uint8_t)194, (uint8_t)156, (uint8_t)137, (uint8_t)63, (uint8_t)218, (uint8_t)247, (uint8_t)120, (uint8_t)84, (uint8_t)254, (uint8_t)209, (uint8_t)79, (uint8_t)19, (uint8_t)166, (uint8_t)235, (uint8_t)74, (uint8_t)58, (uint8_t)79, (uint8_t)152, (uint8_t)197, (uint8_t)62, (uint8_t)145, (uint8_t)177, (uint8_t)75, (uint8_t)121, (uint8_t)235, (uint8_t)202, (uint8_t)158, (uint8_t)241, (uint8_t)165, (uint8_t)190, (uint8_t)99, (uint8_t)158, (uint8_t)101, (uint8_t)126, (uint8_t)9, (uint8_t)50, (uint8_t)49};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)2072807529630901935L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -5642557073663450733L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)7754738809755496466L, PH.base.pack) ;
        p112_seq_SET((uint32_t)3681329487L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t)30882, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)18619, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)42054, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)57847, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)5321507550030243487L, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)11246, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -21928, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p113_lat_SET((int32_t) -359161562, PH.base.pack) ;
        p113_lon_SET((int32_t) -859842702, PH.base.pack) ;
        p113_alt_SET((int32_t) -1962844231, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -20095, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_y_SET((float)3.933954E37F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)3261581816L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -3.261008E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)463398191010215363L, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.7674457E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3914441816L, PH.base.pack) ;
        p114_distance_SET((float)1.4585309E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)3.2810434E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)2.1671686E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)14844, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_zacc_SET((int16_t)(int16_t) -30940, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -2517, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)26195, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -30651, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)11830, PH.base.pack) ;
        p115_alt_SET((int32_t)245572400, PH.base.pack) ;
        p115_lat_SET((int32_t)1120886219, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -8869, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)5630975430830068510L, PH.base.pack) ;
        p115_yawspeed_SET((float) -6.1356216E37F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)8092, PH.base.pack) ;
        p115_lon_SET((int32_t)63297552, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -27195, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {8.983725E37F, 3.189822E38F, 1.8287655E38F, 3.0924265E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_pitchspeed_SET((float)1.1836137E38F, PH.base.pack) ;
        p115_rollspeed_SET((float) -2.6867082E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_zgyro_SET((int16_t)(int16_t)17560, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)22481, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)6505, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -31171, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -23854, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -7104, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -11926, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -5106, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2526705085L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -4082, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)18245, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)44350, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)37183, PH.base.pack) ;
        p118_size_SET((uint32_t)2964086391L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)3956, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)919832568L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)2393, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_ofs_SET((uint32_t)2228877277L, PH.base.pack) ;
        p119_count_SET((uint32_t)2743970986L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)1629, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)227, (uint8_t)64, (uint8_t)59, (uint8_t)166, (uint8_t)102, (uint8_t)113, (uint8_t)254, (uint8_t)115, (uint8_t)45, (uint8_t)15, (uint8_t)139, (uint8_t)138, (uint8_t)255, (uint8_t)235, (uint8_t)119, (uint8_t)37, (uint8_t)152, (uint8_t)168, (uint8_t)124, (uint8_t)64, (uint8_t)5, (uint8_t)1, (uint8_t)60, (uint8_t)145, (uint8_t)70, (uint8_t)195, (uint8_t)214, (uint8_t)152, (uint8_t)167, (uint8_t)122, (uint8_t)204, (uint8_t)237, (uint8_t)15, (uint8_t)74, (uint8_t)96, (uint8_t)37, (uint8_t)123, (uint8_t)179, (uint8_t)175, (uint8_t)55, (uint8_t)26, (uint8_t)76, (uint8_t)154, (uint8_t)139, (uint8_t)237, (uint8_t)236, (uint8_t)143, (uint8_t)36, (uint8_t)147, (uint8_t)221, (uint8_t)140, (uint8_t)175, (uint8_t)197, (uint8_t)124, (uint8_t)168, (uint8_t)227, (uint8_t)137, (uint8_t)76, (uint8_t)27, (uint8_t)4, (uint8_t)184, (uint8_t)208, (uint8_t)75, (uint8_t)103, (uint8_t)234, (uint8_t)15, (uint8_t)164, (uint8_t)197, (uint8_t)89, (uint8_t)60, (uint8_t)137, (uint8_t)153, (uint8_t)10, (uint8_t)210, (uint8_t)5, (uint8_t)154, (uint8_t)216, (uint8_t)167, (uint8_t)223, (uint8_t)25, (uint8_t)160, (uint8_t)229, (uint8_t)26, (uint8_t)73, (uint8_t)51, (uint8_t)169, (uint8_t)63, (uint8_t)166, (uint8_t)254, (uint8_t)65};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_id_SET((uint16_t)(uint16_t)9281, PH.base.pack) ;
        p120_ofs_SET((uint32_t)3943977850L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)98, (uint8_t)98, (uint8_t)36, (uint8_t)190, (uint8_t)51, (uint8_t)101, (uint8_t)115, (uint8_t)39, (uint8_t)173, (uint8_t)87, (uint8_t)197, (uint8_t)252, (uint8_t)242, (uint8_t)75, (uint8_t)85, (uint8_t)147, (uint8_t)144, (uint8_t)37, (uint8_t)230, (uint8_t)225, (uint8_t)81, (uint8_t)161, (uint8_t)223, (uint8_t)213, (uint8_t)164, (uint8_t)229, (uint8_t)249, (uint8_t)13, (uint8_t)30, (uint8_t)245, (uint8_t)238, (uint8_t)180, (uint8_t)30, (uint8_t)170, (uint8_t)28, (uint8_t)28, (uint8_t)73, (uint8_t)205, (uint8_t)182, (uint8_t)1, (uint8_t)182, (uint8_t)3, (uint8_t)247, (uint8_t)171, (uint8_t)112, (uint8_t)183, (uint8_t)19, (uint8_t)0, (uint8_t)58, (uint8_t)92, (uint8_t)236, (uint8_t)91, (uint8_t)226, (uint8_t)165, (uint8_t)7, (uint8_t)93, (uint8_t)178, (uint8_t)24, (uint8_t)211, (uint8_t)48, (uint8_t)68, (uint8_t)141, (uint8_t)59, (uint8_t)21, (uint8_t)228, (uint8_t)145, (uint8_t)223, (uint8_t)249, (uint8_t)113, (uint8_t)4, (uint8_t)161, (uint8_t)110, (uint8_t)36, (uint8_t)161, (uint8_t)173, (uint8_t)187, (uint8_t)18, (uint8_t)8, (uint8_t)72, (uint8_t)46, (uint8_t)43, (uint8_t)2, (uint8_t)241, (uint8_t)9, (uint8_t)27, (uint8_t)9, (uint8_t)112, (uint8_t)246, (uint8_t)58, (uint8_t)35, (uint8_t)221, (uint8_t)227, (uint8_t)96, (uint8_t)111, (uint8_t)11, (uint8_t)227, (uint8_t)192, (uint8_t)51, (uint8_t)154, (uint8_t)203, (uint8_t)34, (uint8_t)211, (uint8_t)60, (uint8_t)188, (uint8_t)224, (uint8_t)185, (uint8_t)51, (uint8_t)30, (uint8_t)224, (uint8_t)248};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)5727218306717901684L, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)18280, PH.base.pack) ;
        p124_lon_SET((int32_t) -80919083, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)45361, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)347524259L, PH.base.pack) ;
        p124_alt_SET((int32_t) -1789284962, PH.base.pack) ;
        p124_lat_SET((int32_t)1064529040, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)27234, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)56856, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)25359, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)7077, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)114, (uint8_t)87, (uint8_t)151, (uint8_t)191, (uint8_t)15, (uint8_t)237, (uint8_t)245, (uint8_t)82, (uint8_t)189, (uint8_t)2, (uint8_t)154, (uint8_t)251, (uint8_t)145, (uint8_t)113, (uint8_t)215, (uint8_t)138, (uint8_t)210, (uint8_t)0, (uint8_t)170, (uint8_t)136, (uint8_t)155, (uint8_t)187, (uint8_t)178, (uint8_t)93, (uint8_t)55, (uint8_t)110, (uint8_t)47, (uint8_t)7, (uint8_t)183, (uint8_t)186, (uint8_t)208, (uint8_t)235, (uint8_t)192, (uint8_t)39, (uint8_t)80, (uint8_t)202, (uint8_t)99, (uint8_t)245, (uint8_t)188, (uint8_t)212, (uint8_t)184, (uint8_t)235, (uint8_t)254, (uint8_t)8, (uint8_t)198, (uint8_t)65, (uint8_t)205, (uint8_t)123, (uint8_t)207, (uint8_t)251, (uint8_t)51, (uint8_t)75, (uint8_t)106, (uint8_t)35, (uint8_t)123, (uint8_t)138, (uint8_t)146, (uint8_t)159, (uint8_t)142, (uint8_t)48, (uint8_t)8, (uint8_t)100, (uint8_t)8, (uint8_t)175, (uint8_t)70, (uint8_t)50, (uint8_t)93, (uint8_t)197, (uint8_t)2, (uint8_t)49};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)62080, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1758113542L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_wn_SET((uint16_t)(uint16_t)5872, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -1698760304, PH.base.pack) ;
        p127_tow_SET((uint32_t)2574609806L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2762410331L, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1791319386, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)452625575L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -305280526, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)487283407, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_health_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)4095, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -1391414462, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1750290247, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)9580373L, PH.base.pack) ;
        p128_tow_SET((uint32_t)4018184508L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)242853902L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)1268649990, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)485408491, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_ygyro_SET((int16_t)(int16_t)16750, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -29710, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -20794, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -11514, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)25836, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)14195, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)802012471L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)22691, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)6717, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)9437, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_jpg_quality_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)7129, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p130_size_SET((uint32_t)3993553081L, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)59564, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)43076, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)62662, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)42, (uint8_t)183, (uint8_t)21, (uint8_t)128, (uint8_t)202, (uint8_t)65, (uint8_t)43, (uint8_t)194, (uint8_t)38, (uint8_t)163, (uint8_t)144, (uint8_t)32, (uint8_t)110, (uint8_t)39, (uint8_t)161, (uint8_t)213, (uint8_t)80, (uint8_t)141, (uint8_t)44, (uint8_t)215, (uint8_t)113, (uint8_t)11, (uint8_t)254, (uint8_t)132, (uint8_t)128, (uint8_t)209, (uint8_t)197, (uint8_t)91, (uint8_t)216, (uint8_t)21, (uint8_t)181, (uint8_t)239, (uint8_t)7, (uint8_t)82, (uint8_t)75, (uint8_t)79, (uint8_t)171, (uint8_t)186, (uint8_t)140, (uint8_t)164, (uint8_t)231, (uint8_t)178, (uint8_t)21, (uint8_t)120, (uint8_t)5, (uint8_t)189, (uint8_t)29, (uint8_t)114, (uint8_t)5, (uint8_t)134, (uint8_t)245, (uint8_t)51, (uint8_t)94, (uint8_t)35, (uint8_t)200, (uint8_t)238, (uint8_t)69, (uint8_t)200, (uint8_t)66, (uint8_t)55, (uint8_t)240, (uint8_t)209, (uint8_t)173, (uint8_t)180, (uint8_t)138, (uint8_t)207, (uint8_t)103, (uint8_t)216, (uint8_t)68, (uint8_t)152, (uint8_t)6, (uint8_t)83, (uint8_t)87, (uint8_t)151, (uint8_t)154, (uint8_t)137, (uint8_t)79, (uint8_t)193, (uint8_t)240, (uint8_t)235, (uint8_t)200, (uint8_t)136, (uint8_t)90, (uint8_t)255, (uint8_t)190, (uint8_t)183, (uint8_t)66, (uint8_t)103, (uint8_t)75, (uint8_t)236, (uint8_t)39, (uint8_t)213, (uint8_t)229, (uint8_t)46, (uint8_t)127, (uint8_t)77, (uint8_t)125, (uint8_t)143, (uint8_t)104, (uint8_t)229, (uint8_t)136, (uint8_t)241, (uint8_t)208, (uint8_t)234, (uint8_t)48, (uint8_t)112, (uint8_t)246, (uint8_t)52, (uint8_t)88, (uint8_t)95, (uint8_t)118, (uint8_t)224, (uint8_t)154, (uint8_t)243, (uint8_t)92, (uint8_t)129, (uint8_t)188, (uint8_t)229, (uint8_t)15, (uint8_t)87, (uint8_t)36, (uint8_t)228, (uint8_t)59, (uint8_t)66, (uint8_t)158, (uint8_t)126, (uint8_t)172, (uint8_t)175, (uint8_t)92, (uint8_t)64, (uint8_t)236, (uint8_t)250, (uint8_t)49, (uint8_t)185, (uint8_t)126, (uint8_t)156, (uint8_t)15, (uint8_t)7, (uint8_t)82, (uint8_t)12, (uint8_t)141, (uint8_t)243, (uint8_t)138, (uint8_t)52, (uint8_t)255, (uint8_t)242, (uint8_t)145, (uint8_t)69, (uint8_t)119, (uint8_t)110, (uint8_t)188, (uint8_t)57, (uint8_t)69, (uint8_t)211, (uint8_t)210, (uint8_t)160, (uint8_t)185, (uint8_t)65, (uint8_t)62, (uint8_t)239, (uint8_t)139, (uint8_t)30, (uint8_t)59, (uint8_t)190, (uint8_t)73, (uint8_t)136, (uint8_t)148, (uint8_t)129, (uint8_t)210, (uint8_t)34, (uint8_t)191, (uint8_t)155, (uint8_t)150, (uint8_t)134, (uint8_t)240, (uint8_t)244, (uint8_t)181, (uint8_t)26, (uint8_t)221, (uint8_t)85, (uint8_t)197, (uint8_t)8, (uint8_t)139, (uint8_t)182, (uint8_t)65, (uint8_t)72, (uint8_t)216, (uint8_t)77, (uint8_t)38, (uint8_t)5, (uint8_t)57, (uint8_t)251, (uint8_t)85, (uint8_t)251, (uint8_t)215, (uint8_t)28, (uint8_t)4, (uint8_t)217, (uint8_t)242, (uint8_t)43, (uint8_t)133, (uint8_t)173, (uint8_t)173, (uint8_t)247, (uint8_t)14, (uint8_t)223, (uint8_t)222, (uint8_t)244, (uint8_t)179, (uint8_t)49, (uint8_t)18, (uint8_t)141, (uint8_t)74, (uint8_t)197, (uint8_t)203, (uint8_t)83, (uint8_t)146, (uint8_t)190, (uint8_t)156, (uint8_t)162, (uint8_t)124, (uint8_t)82, (uint8_t)126, (uint8_t)248, (uint8_t)228, (uint8_t)43, (uint8_t)213, (uint8_t)140, (uint8_t)99, (uint8_t)12, (uint8_t)89, (uint8_t)193, (uint8_t)15, (uint8_t)195, (uint8_t)57, (uint8_t)203, (uint8_t)230, (uint8_t)16, (uint8_t)49, (uint8_t)56, (uint8_t)47, (uint8_t)171, (uint8_t)18, (uint8_t)118, (uint8_t)105, (uint8_t)232, (uint8_t)129, (uint8_t)90, (uint8_t)118, (uint8_t)169, (uint8_t)105, (uint8_t)169, (uint8_t)59};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)2993, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)63639, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)11107, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1649280374L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)3783131348454693120L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)16126, PH.base.pack) ;
        p133_lon_SET((int32_t) -1370742673, PH.base.pack) ;
        p133_lat_SET((int32_t)1639293627, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_gridbit_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)3332, PH.base.pack) ;
        p134_lat_SET((int32_t) -2037070691, PH.base.pack) ;
        p134_lon_SET((int32_t) -306727170, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -26496, (int16_t)24825, (int16_t) -7534, (int16_t)14640, (int16_t)13663, (int16_t)6436, (int16_t) -17802, (int16_t) -255, (int16_t)24639, (int16_t)1126, (int16_t) -873, (int16_t)18967, (int16_t)17736, (int16_t) -22037, (int16_t) -18096, (int16_t) -29197};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t)1276463005, PH.base.pack) ;
        p135_lat_SET((int32_t) -1511507621, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float)7.0899E37F, PH.base.pack) ;
        p136_terrain_height_SET((float)1.1256461E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)16529, PH.base.pack) ;
        p136_lon_SET((int32_t) -2075150205, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)9241, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)24566, PH.base.pack) ;
        p136_lat_SET((int32_t) -2142818411, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)268122830L, PH.base.pack) ;
        p137_press_abs_SET((float)2.0420736E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -4.128494E37F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)5622, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_z_SET((float) -7.5116746E37F, PH.base.pack) ;
        p138_y_SET((float) -2.6174052E38F, PH.base.pack) ;
        {
            float q[] =  {1.5228152E38F, 2.4162102E38F, -2.8994816E38F, -2.003331E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_time_usec_SET((uint64_t)4117327214053500813L, PH.base.pack) ;
        p138_x_SET((float)2.0875993E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_group_mlx_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)2673352738059338382L, PH.base.pack) ;
        {
            float controls[] =  {-1.995323E38F, 2.330003E38F, 4.713177E37F, -7.841549E37F, 6.7411727E37F, -4.5442085E37F, -1.6482445E38F, 8.355753E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_group_mlx_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        {
            float controls[] =  {1.3179131E38F, 2.6066943E38F, 2.117493E38F, 2.2581536E38F, 3.8239986E37F, -2.431956E38F, -8.110514E37F, -1.7838856E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)2693871983898212670L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_monotonic_SET((float) -2.7448753E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)1550673287045278835L, PH.base.pack) ;
        p141_altitude_amsl_SET((float)9.385674E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -5.154936E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)1.8491404E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)4.4963623E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)1.7011174E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t uri[] =  {(uint8_t)174, (uint8_t)102, (uint8_t)17, (uint8_t)51, (uint8_t)108, (uint8_t)183, (uint8_t)195, (uint8_t)221, (uint8_t)50, (uint8_t)223, (uint8_t)184, (uint8_t)2, (uint8_t)38, (uint8_t)183, (uint8_t)214, (uint8_t)143, (uint8_t)0, (uint8_t)236, (uint8_t)8, (uint8_t)25, (uint8_t)16, (uint8_t)143, (uint8_t)219, (uint8_t)31, (uint8_t)182, (uint8_t)12, (uint8_t)158, (uint8_t)25, (uint8_t)230, (uint8_t)133, (uint8_t)145, (uint8_t)212, (uint8_t)186, (uint8_t)48, (uint8_t)93, (uint8_t)24, (uint8_t)29, (uint8_t)78, (uint8_t)4, (uint8_t)115, (uint8_t)99, (uint8_t)18, (uint8_t)198, (uint8_t)243, (uint8_t)227, (uint8_t)107, (uint8_t)3, (uint8_t)198, (uint8_t)17, (uint8_t)253, (uint8_t)61, (uint8_t)180, (uint8_t)114, (uint8_t)121, (uint8_t)27, (uint8_t)160, (uint8_t)74, (uint8_t)238, (uint8_t)31, (uint8_t)170, (uint8_t)121, (uint8_t)136, (uint8_t)164, (uint8_t)224, (uint8_t)74, (uint8_t)44, (uint8_t)17, (uint8_t)161, (uint8_t)78, (uint8_t)11, (uint8_t)163, (uint8_t)167, (uint8_t)130, (uint8_t)245, (uint8_t)168, (uint8_t)46, (uint8_t)237, (uint8_t)110, (uint8_t)87, (uint8_t)20, (uint8_t)183, (uint8_t)69, (uint8_t)188, (uint8_t)197, (uint8_t)1, (uint8_t)12, (uint8_t)135, (uint8_t)141, (uint8_t)2, (uint8_t)191, (uint8_t)250, (uint8_t)123, (uint8_t)240, (uint8_t)63, (uint8_t)60, (uint8_t)251, (uint8_t)254, (uint8_t)41, (uint8_t)83, (uint8_t)157, (uint8_t)247, (uint8_t)78, (uint8_t)69, (uint8_t)146, (uint8_t)94, (uint8_t)3, (uint8_t)248, (uint8_t)228, (uint8_t)164, (uint8_t)173, (uint8_t)22, (uint8_t)177, (uint8_t)70, (uint8_t)226, (uint8_t)12, (uint8_t)120, (uint8_t)17, (uint8_t)65, (uint8_t)11, (uint8_t)129};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)222, (uint8_t)94, (uint8_t)36, (uint8_t)220, (uint8_t)106, (uint8_t)33, (uint8_t)21, (uint8_t)183, (uint8_t)165, (uint8_t)38, (uint8_t)159, (uint8_t)102, (uint8_t)43, (uint8_t)113, (uint8_t)181, (uint8_t)206, (uint8_t)128, (uint8_t)198, (uint8_t)96, (uint8_t)132, (uint8_t)32, (uint8_t)89, (uint8_t)42, (uint8_t)246, (uint8_t)224, (uint8_t)3, (uint8_t)69, (uint8_t)86, (uint8_t)147, (uint8_t)143, (uint8_t)26, (uint8_t)68, (uint8_t)137, (uint8_t)200, (uint8_t)254, (uint8_t)111, (uint8_t)178, (uint8_t)152, (uint8_t)63, (uint8_t)175, (uint8_t)71, (uint8_t)231, (uint8_t)138, (uint8_t)102, (uint8_t)133, (uint8_t)174, (uint8_t)33, (uint8_t)185, (uint8_t)129, (uint8_t)49, (uint8_t)143, (uint8_t)239, (uint8_t)31, (uint8_t)242, (uint8_t)212, (uint8_t)202, (uint8_t)166, (uint8_t)70, (uint8_t)71, (uint8_t)11, (uint8_t)21, (uint8_t)79, (uint8_t)146, (uint8_t)31, (uint8_t)38, (uint8_t)251, (uint8_t)4, (uint8_t)68, (uint8_t)62, (uint8_t)160, (uint8_t)185, (uint8_t)109, (uint8_t)117, (uint8_t)246, (uint8_t)48, (uint8_t)118, (uint8_t)98, (uint8_t)226, (uint8_t)127, (uint8_t)158, (uint8_t)80, (uint8_t)136, (uint8_t)86, (uint8_t)213, (uint8_t)86, (uint8_t)179, (uint8_t)246, (uint8_t)58, (uint8_t)250, (uint8_t)138, (uint8_t)84, (uint8_t)39, (uint8_t)180, (uint8_t)6, (uint8_t)223, (uint8_t)148, (uint8_t)42, (uint8_t)244, (uint8_t)2, (uint8_t)16, (uint8_t)52, (uint8_t)119, (uint8_t)11, (uint8_t)170, (uint8_t)227, (uint8_t)173, (uint8_t)216, (uint8_t)239, (uint8_t)250, (uint8_t)2, (uint8_t)246, (uint8_t)85, (uint8_t)3, (uint8_t)149, (uint8_t)7, (uint8_t)186, (uint8_t)63, (uint8_t)72, (uint8_t)229, (uint8_t)126};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float) -1.2654204E38F, PH.base.pack) ;
        p143_press_diff_SET((float) -3.3907245E37F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)793992420L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)9888, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_lon_SET((int32_t) -1527013454, PH.base.pack) ;
        p144_lat_SET((int32_t)170039208, PH.base.pack) ;
        p144_alt_SET((float)3.1091362E38F, PH.base.pack) ;
        {
            float attitude_q[] =  {-8.576083E37F, -2.1649042E38F, -1.6689853E38F, 2.3794222E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-2.0137859E38F, -2.5366966E38F, 1.4678234E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)7026173012231353721L, PH.base.pack) ;
        {
            float acc[] =  {1.8461781E38F, 1.4835346E38F, -1.6679438E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-1.8785556E38F, -3.281628E38F, -4.33999E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-2.6081995E37F, 3.7325987E37F, 2.4682496E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)7931151958274776094L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_pos_SET((float) -2.7768823E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)4276370110219286570L, PH.base.pack) ;
        {
            float pos_variance[] =  {2.86957E38F, -1.4717307E38F, -1.2641365E34F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_yaw_rate_SET((float) -1.7620203E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -1.1809581E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -2.0092164E37F, PH.base.pack) ;
        p146_y_vel_SET((float)1.0838679E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.5718313E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {2.3154042E38F, -2.1347092E38F, -1.8570838E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float) -4.443636E37F, PH.base.pack) ;
        p146_roll_rate_SET((float) -8.0073614E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float)8.1398416E37F, PH.base.pack) ;
        p146_z_pos_SET((float)2.9831875E38F, PH.base.pack) ;
        p146_y_pos_SET((float) -3.2774149E38F, PH.base.pack) ;
        {
            float q[] =  {3.003171E38F, -3.1161547E38F, -2.0528409E38F, 2.44463E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float) -2.3183167E38F, PH.base.pack) ;
        p146_x_acc_SET((float)1.1410252E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_temperature_SET((int16_t)(int16_t)18750, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)26180, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -717531605, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1505651378, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)22205, (uint16_t)16855, (uint16_t)13772, (uint16_t)47645, (uint16_t)23905, (uint16_t)21013, (uint16_t)51831, (uint16_t)32675, (uint16_t)35861, (uint16_t)37154};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_battery_remaining_SET((int8_t)(int8_t)50, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_vendor_id_SET((uint16_t)(uint16_t)1696, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1330271977L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)178, (uint8_t)44, (uint8_t)80, (uint8_t)108, (uint8_t)205, (uint8_t)96, (uint8_t)53, (uint8_t)107};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)4000539540780997984L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)242, (uint8_t)151, (uint8_t)106, (uint8_t)22, (uint8_t)172, (uint8_t)164, (uint8_t)150, (uint8_t)196};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)1896205892L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)19134, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)37, (uint8_t)220, (uint8_t)190, (uint8_t)114, (uint8_t)49, (uint8_t)81, (uint8_t)82, (uint8_t)244, (uint8_t)0, (uint8_t)21, (uint8_t)144, (uint8_t)73, (uint8_t)16, (uint8_t)85, (uint8_t)176, (uint8_t)211, (uint8_t)52, (uint8_t)143};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)175, (uint8_t)5, (uint8_t)160, (uint8_t)230, (uint8_t)9, (uint8_t)123, (uint8_t)77, (uint8_t)233};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)2778212739L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2645309967L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_distance_SET((float) -1.0360258E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p149_z_SET((float)4.0765108E37F, &PH) ;
        p149_angle_y_SET((float) -1.8082924E38F, PH.base.pack) ;
        p149_x_SET((float)1.5990729E38F, &PH) ;
        p149_size_y_SET((float)2.1829864E38F, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        {
            float q[] =  {1.1664952E38F, 4.452393E36F, -3.121072E38F, 3.0649627E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_position_valid_SET((uint8_t)(uint8_t)114, &PH) ;
        p149_y_SET((float)8.249089E37F, &PH) ;
        p149_angle_x_SET((float) -2.6288402E38F, PH.base.pack) ;
        p149_size_x_SET((float) -5.5350346E37F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)6450284443771666817L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CPU_LOAD_170(), &PH);
        p170_sensLoad_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p170_ctrlLoad_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p170_batVolt_SET((uint16_t)(uint16_t)41539, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CPU_LOAD_170(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENSOR_BIAS_172(), &PH);
        p172_gzBias_SET((float) -7.6081996E37F, PH.base.pack) ;
        p172_ayBias_SET((float) -5.430878E37F, PH.base.pack) ;
        p172_azBias_SET((float)2.632925E38F, PH.base.pack) ;
        p172_axBias_SET((float) -1.7762564E38F, PH.base.pack) ;
        p172_gyBias_SET((float)7.201545E37F, PH.base.pack) ;
        p172_gxBias_SET((float) -2.2086232E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENSOR_BIAS_172(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DIAGNOSTIC_173(), &PH);
        p173_diagFl2_SET((float)1.1557893E38F, PH.base.pack) ;
        p173_diagSh2_SET((int16_t)(int16_t)15830, PH.base.pack) ;
        p173_diagFl3_SET((float) -4.092314E37F, PH.base.pack) ;
        p173_diagSh1_SET((int16_t)(int16_t)24448, PH.base.pack) ;
        p173_diagSh3_SET((int16_t)(int16_t) -18293, PH.base.pack) ;
        p173_diagFl1_SET((float) -1.2771473E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DIAGNOSTIC_173(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SLUGS_NAVIGATION_176(), &PH);
        p176_u_m_SET((float)1.4214203E38F, PH.base.pack) ;
        p176_dist2Go_SET((float)3.2128385E38F, PH.base.pack) ;
        p176_toWP_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p176_totalDist_SET((float) -2.833827E38F, PH.base.pack) ;
        p176_psiDot_c_SET((float) -2.8015699E38F, PH.base.pack) ;
        p176_ay_body_SET((float) -1.614989E38F, PH.base.pack) ;
        p176_theta_c_SET((float) -5.6543124E37F, PH.base.pack) ;
        p176_phi_c_SET((float)2.864455E38F, PH.base.pack) ;
        p176_fromWP_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p176_h_c_SET((uint16_t)(uint16_t)29944, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SLUGS_NAVIGATION_176(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_LOG_177(), &PH);
        p177_fl_3_SET((float) -2.6412721E38F, PH.base.pack) ;
        p177_fl_4_SET((float)1.3052289E38F, PH.base.pack) ;
        p177_fl_5_SET((float)2.9485835E38F, PH.base.pack) ;
        p177_fl_6_SET((float) -1.1764757E38F, PH.base.pack) ;
        p177_fl_1_SET((float) -2.7393566E38F, PH.base.pack) ;
        p177_fl_2_SET((float) -1.803697E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_LOG_177(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_DATE_TIME_179(), &PH);
        p179_useSat_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p179_day_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p179_month_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p179_GppGl_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p179_hour_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p179_sec_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p179_year_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p179_visSat_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p179_percentUsed_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p179_min_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p179_clockStat_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p179_sigUsedMask_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_DATE_TIME_179(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MID_LVL_CMDS_180(), &PH);
        p180_target_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p180_hCommand_SET((float) -6.995418E37F, PH.base.pack) ;
        p180_uCommand_SET((float) -1.2558346E38F, PH.base.pack) ;
        p180_rCommand_SET((float) -1.303029E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MID_LVL_CMDS_180(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CTRL_SRFC_PT_181(), &PH);
        p181_target_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p181_bitfieldPt_SET((uint16_t)(uint16_t)53985, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CTRL_SRFC_PT_181(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SLUGS_CAMERA_ORDER_184(), &PH);
        p184_zoom_SET((int8_t)(int8_t) -108, PH.base.pack) ;
        p184_tilt_SET((int8_t)(int8_t)50, PH.base.pack) ;
        p184_pan_SET((int8_t)(int8_t) -51, PH.base.pack) ;
        p184_target_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p184_moveHome_SET((int8_t)(int8_t)54, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SLUGS_CAMERA_ORDER_184(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SURFACE_185(), &PH);
        p185_bControl_SET((float)3.2672019E38F, PH.base.pack) ;
        p185_idSurface_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p185_target_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p185_mControl_SET((float) -9.873779E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SURFACE_185(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SLUGS_MOBILE_LOCATION_186(), &PH);
        p186_longitude_SET((float)1.5200676E38F, PH.base.pack) ;
        p186_target_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p186_latitude_SET((float)2.5424844E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SLUGS_MOBILE_LOCATION_186(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SLUGS_CONFIGURATION_CAMERA_188(), &PH);
        p188_target_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p188_order_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p188_idOrder_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SLUGS_CONFIGURATION_CAMERA_188(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ISR_LOCATION_189(), &PH);
        p189_longitude_SET((float)1.0230297E38F, PH.base.pack) ;
        p189_height_SET((float) -3.8417325E37F, PH.base.pack) ;
        p189_latitude_SET((float) -3.0602231E38F, PH.base.pack) ;
        p189_target_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p189_option1_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p189_option2_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p189_option3_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ISR_LOCATION_189(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VOLT_SENSOR_191(), &PH);
        p191_voltage_SET((uint16_t)(uint16_t)49925, PH.base.pack) ;
        p191_reading2_SET((uint16_t)(uint16_t)24568, PH.base.pack) ;
        p191_r2Type_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VOLT_SENSOR_191(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PTZ_STATUS_192(), &PH);
        p192_zoom_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p192_pan_SET((int16_t)(int16_t) -29536, PH.base.pack) ;
        p192_tilt_SET((int16_t)(int16_t) -13852, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PTZ_STATUS_192(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAV_STATUS_193(), &PH);
        p193_speed_SET((float) -3.3286852E38F, PH.base.pack) ;
        p193_altitude_SET((float) -7.506675E36F, PH.base.pack) ;
        p193_target_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p193_longitude_SET((float)3.6145528E37F, PH.base.pack) ;
        p193_course_SET((float) -3.1016354E38F, PH.base.pack) ;
        p193_latitude_SET((float) -2.3605613E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAV_STATUS_193(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUS_GPS_194(), &PH);
        p194_csFails_SET((uint16_t)(uint16_t)45155, PH.base.pack) ;
        p194_posStatus_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p194_magVar_SET((float)3.1055884E38F, PH.base.pack) ;
        p194_msgsType_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p194_magDir_SET((int8_t)(int8_t) -1, PH.base.pack) ;
        p194_gpsQuality_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p194_modeInd_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUS_GPS_194(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NOVATEL_DIAG_195(), &PH);
        p195_posSolAge_SET((float) -1.9603454E38F, PH.base.pack) ;
        p195_solStatus_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p195_posType_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p195_timeStatus_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p195_csFails_SET((uint16_t)(uint16_t)492, PH.base.pack) ;
        p195_receiverStatus_SET((uint32_t)1072074731L, PH.base.pack) ;
        p195_velType_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NOVATEL_DIAG_195(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENSOR_DIAG_196(), &PH);
        p196_char1_SET((int8_t)(int8_t) -72, PH.base.pack) ;
        p196_float2_SET((float) -2.0139625E38F, PH.base.pack) ;
        p196_int1_SET((int16_t)(int16_t)30310, PH.base.pack) ;
        p196_float1_SET((float) -9.128335E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENSOR_DIAG_196(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BOOT_197(), &PH);
        p197_version_SET((uint32_t)2699144086L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BOOT_197(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_time_usec_SET((uint64_t)6623687166019401223L, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.5866122E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -1.8682345E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.2384308E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float)1.3311353E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -3.0043161E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -1.089248E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)5.366357E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float)2.1546089E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float)5.995671E37F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -1.6021569E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)3053613563962840942L, PH.base.pack) ;
        p231_wind_x_SET((float)6.5854266E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -1.3629698E38F, PH.base.pack) ;
        p231_wind_y_SET((float)1.1430512E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.989475E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)3.2861067E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.2538182E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_time_usec_SET((uint64_t)4569276684848568024L, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -1.0888324E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -2.4085765E38F, PH.base.pack) ;
        p232_vdop_SET((float)3.087965E38F, PH.base.pack) ;
        p232_hdop_SET((float)1.8845454E38F, PH.base.pack) ;
        p232_vd_SET((float)2.2649592E38F, PH.base.pack) ;
        p232_alt_SET((float)2.4192607E37F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, PH.base.pack) ;
        p232_lat_SET((int32_t) -1352637103, PH.base.pack) ;
        p232_lon_SET((int32_t) -1618227355, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p232_vn_SET((float)9.467793E37F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)1199585044L, PH.base.pack) ;
        p232_ve_SET((float) -2.8145563E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -8.2376555E37F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)3602, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)85, (uint8_t)208, (uint8_t)148, (uint8_t)51, (uint8_t)166, (uint8_t)129, (uint8_t)237, (uint8_t)4, (uint8_t)208, (uint8_t)37, (uint8_t)79, (uint8_t)184, (uint8_t)123, (uint8_t)61, (uint8_t)58, (uint8_t)106, (uint8_t)23, (uint8_t)73, (uint8_t)150, (uint8_t)255, (uint8_t)148, (uint8_t)216, (uint8_t)112, (uint8_t)246, (uint8_t)80, (uint8_t)167, (uint8_t)14, (uint8_t)248, (uint8_t)45, (uint8_t)106, (uint8_t)146, (uint8_t)169, (uint8_t)174, (uint8_t)0, (uint8_t)194, (uint8_t)47, (uint8_t)253, (uint8_t)157, (uint8_t)135, (uint8_t)84, (uint8_t)66, (uint8_t)5, (uint8_t)66, (uint8_t)221, (uint8_t)79, (uint8_t)219, (uint8_t)6, (uint8_t)4, (uint8_t)196, (uint8_t)67, (uint8_t)159, (uint8_t)230, (uint8_t)15, (uint8_t)13, (uint8_t)106, (uint8_t)146, (uint8_t)240, (uint8_t)143, (uint8_t)24, (uint8_t)20, (uint8_t)196, (uint8_t)158, (uint8_t)74, (uint8_t)127, (uint8_t)64, (uint8_t)53, (uint8_t)169, (uint8_t)193, (uint8_t)94, (uint8_t)244, (uint8_t)133, (uint8_t)160, (uint8_t)142, (uint8_t)18, (uint8_t)204, (uint8_t)231, (uint8_t)67, (uint8_t)120, (uint8_t)196, (uint8_t)60, (uint8_t)234, (uint8_t)203, (uint8_t)24, (uint8_t)249, (uint8_t)116, (uint8_t)240, (uint8_t)106, (uint8_t)55, (uint8_t)248, (uint8_t)102, (uint8_t)52, (uint8_t)245, (uint8_t)104, (uint8_t)151, (uint8_t)35, (uint8_t)50, (uint8_t)26, (uint8_t)160, (uint8_t)147, (uint8_t)207, (uint8_t)50, (uint8_t)70, (uint8_t)83, (uint8_t)156, (uint8_t)110, (uint8_t)106, (uint8_t)154, (uint8_t)198, (uint8_t)62, (uint8_t)197, (uint8_t)144, (uint8_t)139, (uint8_t)61, (uint8_t)249, (uint8_t)205, (uint8_t)2, (uint8_t)246, (uint8_t)224, (uint8_t)219, (uint8_t)214, (uint8_t)232, (uint8_t)250, (uint8_t)6, (uint8_t)46, (uint8_t)42, (uint8_t)77, (uint8_t)181, (uint8_t)167, (uint8_t)174, (uint8_t)74, (uint8_t)211, (uint8_t)221, (uint8_t)30, (uint8_t)47, (uint8_t)116, (uint8_t)158, (uint8_t)240, (uint8_t)219, (uint8_t)243, (uint8_t)206, (uint8_t)68, (uint8_t)148, (uint8_t)18, (uint8_t)31, (uint8_t)200, (uint8_t)94, (uint8_t)189, (uint8_t)54, (uint8_t)162, (uint8_t)124, (uint8_t)154, (uint8_t)173, (uint8_t)78, (uint8_t)3, (uint8_t)255, (uint8_t)234, (uint8_t)172, (uint8_t)14, (uint8_t)225, (uint8_t)116, (uint8_t)1, (uint8_t)244, (uint8_t)82, (uint8_t)225, (uint8_t)27, (uint8_t)18, (uint8_t)26, (uint8_t)248, (uint8_t)20, (uint8_t)24, (uint8_t)103, (uint8_t)36, (uint8_t)47, (uint8_t)35, (uint8_t)223, (uint8_t)215, (uint8_t)238, (uint8_t)252, (uint8_t)31, (uint8_t)101};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_groundspeed_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -48, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -15989, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1525836642, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)24105, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)81, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p234_latitude_SET((int32_t)1348653872, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)2562130396L, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)37, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -5710, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -27399, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)28732, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)58191, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)31485, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -104, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_clipping_0_SET((uint32_t)2056511427L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1370157642L, PH.base.pack) ;
        p241_vibration_y_SET((float) -3.3572286E38F, PH.base.pack) ;
        p241_vibration_x_SET((float) -2.1036714E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2359828007L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6639173384410457884L, PH.base.pack) ;
        p241_vibration_z_SET((float) -1.4737699E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {1.7185555E38F, 1.5826498E38F, 1.9136595E38F, -3.3555458E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_longitude_SET((int32_t)1782130640, PH.base.pack) ;
        p242_x_SET((float)1.742057E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)2957070934689285937L, &PH) ;
        p242_y_SET((float) -5.0542735E36F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1881079676, PH.base.pack) ;
        p242_approach_x_SET((float) -8.903295E37F, PH.base.pack) ;
        p242_latitude_SET((int32_t)6223292, PH.base.pack) ;
        p242_approach_z_SET((float) -2.422618E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -1.5261746E38F, PH.base.pack) ;
        p242_z_SET((float) -3.0636783E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_y_SET((float)1.8259225E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1532408151, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)7092725404800400376L, &PH) ;
        p243_y_SET((float) -3.6551942E37F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p243_approach_x_SET((float) -1.3199435E37F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.0674547E38F, PH.base.pack) ;
        p243_z_SET((float)2.8832255E38F, PH.base.pack) ;
        {
            float q[] =  {-7.8307286E35F, -8.034711E37F, 3.206773E37F, 3.3709677E37F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_x_SET((float)2.0487469E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)223965135, PH.base.pack) ;
        p243_latitude_SET((int32_t)2242953, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)2102380356, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)50800, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE, PH.base.pack) ;
        p246_altitude_SET((int32_t)1069741034, PH.base.pack) ;
        p246_lon_SET((int32_t) -174771616, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)64865, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -13169, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)55446, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)585015769L, PH.base.pack) ;
        p246_lat_SET((int32_t)63931917, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)22054, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        {
            char16_t* callsign = u"klo";
            p246_callsign_SET_(callsign, &PH) ;
        }
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_altitude_minimum_delta_SET((float)3.0818292E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)3549377175L, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.731188E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.4510701E37F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)50382, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)159, (uint8_t)1, (uint8_t)13, (uint8_t)144, (uint8_t)216, (uint8_t)97, (uint8_t)91, (uint8_t)164, (uint8_t)220, (uint8_t)230, (uint8_t)36, (uint8_t)28, (uint8_t)180, (uint8_t)147, (uint8_t)106, (uint8_t)163, (uint8_t)209, (uint8_t)210, (uint8_t)23, (uint8_t)113, (uint8_t)64, (uint8_t)55, (uint8_t)217, (uint8_t)211, (uint8_t)79, (uint8_t)92, (uint8_t)153, (uint8_t)114, (uint8_t)196, (uint8_t)177, (uint8_t)138, (uint8_t)93, (uint8_t)227, (uint8_t)233, (uint8_t)226, (uint8_t)9, (uint8_t)3, (uint8_t)190, (uint8_t)201, (uint8_t)67, (uint8_t)200, (uint8_t)15, (uint8_t)113, (uint8_t)1, (uint8_t)106, (uint8_t)46, (uint8_t)83, (uint8_t)25, (uint8_t)245, (uint8_t)149, (uint8_t)215, (uint8_t)166, (uint8_t)195, (uint8_t)145, (uint8_t)83, (uint8_t)103, (uint8_t)131, (uint8_t)60, (uint8_t)30, (uint8_t)209, (uint8_t)104, (uint8_t)145, (uint8_t)84, (uint8_t)83, (uint8_t)148, (uint8_t)151, (uint8_t)86, (uint8_t)231, (uint8_t)217, (uint8_t)108, (uint8_t)52, (uint8_t)8, (uint8_t)42, (uint8_t)252, (uint8_t)123, (uint8_t)181, (uint8_t)91, (uint8_t)255, (uint8_t)198, (uint8_t)145, (uint8_t)250, (uint8_t)134, (uint8_t)247, (uint8_t)20, (uint8_t)16, (uint8_t)119, (uint8_t)127, (uint8_t)214, (uint8_t)2, (uint8_t)156, (uint8_t)250, (uint8_t)116, (uint8_t)35, (uint8_t)211, (uint8_t)129, (uint8_t)123, (uint8_t)182, (uint8_t)18, (uint8_t)123, (uint8_t)200, (uint8_t)235, (uint8_t)160, (uint8_t)227, (uint8_t)178, (uint8_t)238, (uint8_t)59, (uint8_t)106, (uint8_t)161, (uint8_t)209, (uint8_t)11, (uint8_t)141, (uint8_t)38, (uint8_t)172, (uint8_t)15, (uint8_t)167, (uint8_t)148, (uint8_t)156, (uint8_t)10, (uint8_t)73, (uint8_t)161, (uint8_t)78, (uint8_t)51, (uint8_t)241, (uint8_t)126, (uint8_t)5, (uint8_t)129, (uint8_t)39, (uint8_t)95, (uint8_t)239, (uint8_t)236, (uint8_t)152, (uint8_t)115, (uint8_t)189, (uint8_t)71, (uint8_t)239, (uint8_t)115, (uint8_t)126, (uint8_t)26, (uint8_t)140, (uint8_t)15, (uint8_t)115, (uint8_t)12, (uint8_t)5, (uint8_t)132, (uint8_t)118, (uint8_t)38, (uint8_t)82, (uint8_t)135, (uint8_t)182, (uint8_t)48, (uint8_t)233, (uint8_t)191, (uint8_t)114, (uint8_t)199, (uint8_t)142, (uint8_t)203, (uint8_t)52, (uint8_t)251, (uint8_t)161, (uint8_t)191, (uint8_t)49, (uint8_t)247, (uint8_t)211, (uint8_t)192, (uint8_t)231, (uint8_t)165, (uint8_t)188, (uint8_t)132, (uint8_t)193, (uint8_t)125, (uint8_t)104, (uint8_t)234, (uint8_t)45, (uint8_t)54, (uint8_t)131, (uint8_t)229, (uint8_t)138, (uint8_t)74, (uint8_t)95, (uint8_t)224, (uint8_t)148, (uint8_t)246, (uint8_t)52, (uint8_t)140, (uint8_t)27, (uint8_t)168, (uint8_t)243, (uint8_t)104, (uint8_t)1, (uint8_t)126, (uint8_t)58, (uint8_t)205, (uint8_t)247, (uint8_t)107, (uint8_t)212, (uint8_t)182, (uint8_t)152, (uint8_t)22, (uint8_t)29, (uint8_t)111, (uint8_t)206, (uint8_t)73, (uint8_t)84, (uint8_t)122, (uint8_t)48, (uint8_t)200, (uint8_t)104, (uint8_t)168, (uint8_t)9, (uint8_t)208, (uint8_t)155, (uint8_t)144, (uint8_t)158, (uint8_t)26, (uint8_t)231, (uint8_t)216, (uint8_t)39, (uint8_t)197, (uint8_t)206, (uint8_t)75, (uint8_t)196, (uint8_t)120, (uint8_t)30, (uint8_t)57, (uint8_t)217, (uint8_t)240, (uint8_t)114, (uint8_t)80, (uint8_t)85, (uint8_t)127, (uint8_t)149, (uint8_t)65, (uint8_t)30, (uint8_t)203, (uint8_t)119, (uint8_t)94, (uint8_t)15, (uint8_t)141, (uint8_t)182, (uint8_t)247, (uint8_t)203, (uint8_t)150, (uint8_t)92, (uint8_t)213, (uint8_t)211, (uint8_t)101, (uint8_t)56, (uint8_t)248, (uint8_t)170};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t)48, (int8_t) -28, (int8_t) -11, (int8_t)64, (int8_t) -107, (int8_t)11, (int8_t) -23, (int8_t) -67, (int8_t)44, (int8_t)55, (int8_t)23, (int8_t)39, (int8_t)52, (int8_t) -84, (int8_t) -39, (int8_t)76, (int8_t)43, (int8_t)44, (int8_t)87, (int8_t) -46, (int8_t) -41, (int8_t)1, (int8_t) -20, (int8_t)43, (int8_t) -113, (int8_t)0, (int8_t) -109, (int8_t) -126, (int8_t)115, (int8_t)106, (int8_t) -39, (int8_t)21};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)40289, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float)1.1230683E38F, PH.base.pack) ;
        p250_y_SET((float)2.6132846E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)9077288534719290114L, PH.base.pack) ;
        p250_z_SET((float) -2.087759E38F, PH.base.pack) ;
        {
            char16_t* name = u"aOOfp";
            p250_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"at";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)2120336383L, PH.base.pack) ;
        p251_value_SET((float) -1.7512494E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"qFhdx";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)265359874L, PH.base.pack) ;
        p252_value_SET((int32_t) -937667782, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"meJikcicxfqnfculpcwbdorebyywEFdRycndmTkvqvBlyzggih";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_CRITICAL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_value_SET((float)3.3100722E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1808688832L, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)608093649166045309L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)174, (uint8_t)116, (uint8_t)73, (uint8_t)168, (uint8_t)231, (uint8_t)162, (uint8_t)159, (uint8_t)6, (uint8_t)137, (uint8_t)40, (uint8_t)249, (uint8_t)210, (uint8_t)235, (uint8_t)133, (uint8_t)115, (uint8_t)153, (uint8_t)247, (uint8_t)225, (uint8_t)17, (uint8_t)153, (uint8_t)107, (uint8_t)255, (uint8_t)81, (uint8_t)147, (uint8_t)183, (uint8_t)145, (uint8_t)169, (uint8_t)131, (uint8_t)103, (uint8_t)216, (uint8_t)216, (uint8_t)80};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)4095445856L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)2330095207L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        {
            char16_t* tune = u"zvwkzkrgqwclbab";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_v_SET((uint16_t)(uint16_t)16665, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"qrwbxiodfdqecqfecirwrqmshilwvwqzxtspxvKNsjlgczhxseltbwnnhxiuzcrxeftvfwXkydbcestlfx";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        {
            uint8_t vendor_name[] =  {(uint8_t)35, (uint8_t)86, (uint8_t)135, (uint8_t)166, (uint8_t)177, (uint8_t)143, (uint8_t)92, (uint8_t)90, (uint8_t)107, (uint8_t)112, (uint8_t)14, (uint8_t)47, (uint8_t)102, (uint8_t)14, (uint8_t)214, (uint8_t)95, (uint8_t)32, (uint8_t)63, (uint8_t)223, (uint8_t)37, (uint8_t)247, (uint8_t)99, (uint8_t)200, (uint8_t)63, (uint8_t)17, (uint8_t)208, (uint8_t)179, (uint8_t)188, (uint8_t)209, (uint8_t)253, (uint8_t)208, (uint8_t)157};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)62113, PH.base.pack) ;
        p259_focal_length_SET((float)5.542363E37F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)235397576L, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1723126269L, PH.base.pack) ;
        p259_sensor_size_h_SET((float)2.663238E38F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)21449, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)157, (uint8_t)102, (uint8_t)94, (uint8_t)77, (uint8_t)74, (uint8_t)140, (uint8_t)138, (uint8_t)20, (uint8_t)163, (uint8_t)6, (uint8_t)90, (uint8_t)152, (uint8_t)241, (uint8_t)76, (uint8_t)32, (uint8_t)149, (uint8_t)7, (uint8_t)136, (uint8_t)34, (uint8_t)168, (uint8_t)242, (uint8_t)177, (uint8_t)27, (uint8_t)140, (uint8_t)239, (uint8_t)130, (uint8_t)162, (uint8_t)209, (uint8_t)187, (uint8_t)128, (uint8_t)0, (uint8_t)26};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float) -3.0073698E38F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3125861768L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_used_capacity_SET((float)3.958427E37F, PH.base.pack) ;
        p261_read_speed_SET((float)3.3733251E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.3701346E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p261_available_capacity_SET((float)3.3976113E38F, PH.base.pack) ;
        p261_write_speed_SET((float)3.1662855E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1865184936L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float) -2.1585621E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2021985350L, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)1559516473L, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.6419071E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lat_SET((int32_t) -335310084, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -643904844, PH.base.pack) ;
        {
            float q[] =  {-2.1447028E38F, -2.9809463E38F, 7.058471E37F, 1.6023305E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            char16_t* file_url = u"mdfjstnyntwtkewhbxfjjbgfdqurbmnrnQstqfbpowpUfmpJpKhqqtmxlplrcuCypqiyuohmyjzhplronj";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1182481144L, PH.base.pack) ;
        p263_alt_SET((int32_t) -1502427323, PH.base.pack) ;
        p263_lon_SET((int32_t)353557546, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)6246139348066080893L, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)74, PH.base.pack) ;
        p263_image_index_SET((int32_t)1862195420, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)599481002415728509L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)907661539495810316L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)1003341798L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)2879844880056816946L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)727623439L, PH.base.pack) ;
        p265_yaw_SET((float) -9.134388E37F, PH.base.pack) ;
        p265_roll_SET((float)2.837548E38F, PH.base.pack) ;
        p265_pitch_SET((float) -9.549214E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_first_message_offset_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)131, (uint8_t)199, (uint8_t)248, (uint8_t)100, (uint8_t)53, (uint8_t)26, (uint8_t)220, (uint8_t)91, (uint8_t)224, (uint8_t)27, (uint8_t)77, (uint8_t)35, (uint8_t)177, (uint8_t)83, (uint8_t)181, (uint8_t)205, (uint8_t)104, (uint8_t)231, (uint8_t)87, (uint8_t)87, (uint8_t)240, (uint8_t)248, (uint8_t)158, (uint8_t)249, (uint8_t)26, (uint8_t)124, (uint8_t)130, (uint8_t)35, (uint8_t)201, (uint8_t)227, (uint8_t)115, (uint8_t)208, (uint8_t)239, (uint8_t)82, (uint8_t)43, (uint8_t)253, (uint8_t)173, (uint8_t)38, (uint8_t)192, (uint8_t)69, (uint8_t)217, (uint8_t)125, (uint8_t)186, (uint8_t)174, (uint8_t)168, (uint8_t)72, (uint8_t)105, (uint8_t)20, (uint8_t)4, (uint8_t)255, (uint8_t)160, (uint8_t)158, (uint8_t)201, (uint8_t)181, (uint8_t)43, (uint8_t)170, (uint8_t)22, (uint8_t)83, (uint8_t)37, (uint8_t)44, (uint8_t)234, (uint8_t)241, (uint8_t)217, (uint8_t)197, (uint8_t)43, (uint8_t)31, (uint8_t)237, (uint8_t)221, (uint8_t)49, (uint8_t)226, (uint8_t)97, (uint8_t)184, (uint8_t)126, (uint8_t)0, (uint8_t)33, (uint8_t)9, (uint8_t)164, (uint8_t)27, (uint8_t)232, (uint8_t)230, (uint8_t)61, (uint8_t)123, (uint8_t)61, (uint8_t)184, (uint8_t)197, (uint8_t)8, (uint8_t)213, (uint8_t)229, (uint8_t)200, (uint8_t)5, (uint8_t)248, (uint8_t)159, (uint8_t)95, (uint8_t)243, (uint8_t)155, (uint8_t)213, (uint8_t)176, (uint8_t)161, (uint8_t)170, (uint8_t)158, (uint8_t)75, (uint8_t)67, (uint8_t)49, (uint8_t)200, (uint8_t)28, (uint8_t)54, (uint8_t)228, (uint8_t)170, (uint8_t)140, (uint8_t)97, (uint8_t)156, (uint8_t)192, (uint8_t)114, (uint8_t)119, (uint8_t)112, (uint8_t)52, (uint8_t)8, (uint8_t)41, (uint8_t)211, (uint8_t)194, (uint8_t)117, (uint8_t)213, (uint8_t)137, (uint8_t)11, (uint8_t)15, (uint8_t)106, (uint8_t)165, (uint8_t)194, (uint8_t)255, (uint8_t)108, (uint8_t)163, (uint8_t)88, (uint8_t)121, (uint8_t)13, (uint8_t)23, (uint8_t)2, (uint8_t)144, (uint8_t)187, (uint8_t)180, (uint8_t)78, (uint8_t)121, (uint8_t)118, (uint8_t)84, (uint8_t)201, (uint8_t)23, (uint8_t)253, (uint8_t)103, (uint8_t)6, (uint8_t)108, (uint8_t)174, (uint8_t)196, (uint8_t)84, (uint8_t)44, (uint8_t)134, (uint8_t)14, (uint8_t)24, (uint8_t)235, (uint8_t)217, (uint8_t)226, (uint8_t)90, (uint8_t)61, (uint8_t)219, (uint8_t)137, (uint8_t)115, (uint8_t)63, (uint8_t)48, (uint8_t)140, (uint8_t)47, (uint8_t)36, (uint8_t)184, (uint8_t)22, (uint8_t)64, (uint8_t)21, (uint8_t)239, (uint8_t)94, (uint8_t)158, (uint8_t)11, (uint8_t)119, (uint8_t)66, (uint8_t)212, (uint8_t)120, (uint8_t)9, (uint8_t)38, (uint8_t)205, (uint8_t)30, (uint8_t)210, (uint8_t)204, (uint8_t)135, (uint8_t)169, (uint8_t)188, (uint8_t)229, (uint8_t)162, (uint8_t)4, (uint8_t)52, (uint8_t)23, (uint8_t)145, (uint8_t)172, (uint8_t)64, (uint8_t)151, (uint8_t)200, (uint8_t)102, (uint8_t)134, (uint8_t)42, (uint8_t)25, (uint8_t)120, (uint8_t)236, (uint8_t)251, (uint8_t)203, (uint8_t)144, (uint8_t)11, (uint8_t)21, (uint8_t)241, (uint8_t)251, (uint8_t)207, (uint8_t)143, (uint8_t)229, (uint8_t)113, (uint8_t)211, (uint8_t)232, (uint8_t)130, (uint8_t)47, (uint8_t)119, (uint8_t)128, (uint8_t)113, (uint8_t)207, (uint8_t)18, (uint8_t)13, (uint8_t)190, (uint8_t)229, (uint8_t)227, (uint8_t)152, (uint8_t)18, (uint8_t)153, (uint8_t)246, (uint8_t)219, (uint8_t)113, (uint8_t)120, (uint8_t)100, (uint8_t)89, (uint8_t)211, (uint8_t)168, (uint8_t)1, (uint8_t)48, (uint8_t)65, (uint8_t)12, (uint8_t)40, (uint8_t)138, (uint8_t)111, (uint8_t)44};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)49471, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)36, (uint8_t)243, (uint8_t)118, (uint8_t)198, (uint8_t)231, (uint8_t)3, (uint8_t)173, (uint8_t)37, (uint8_t)106, (uint8_t)58, (uint8_t)70, (uint8_t)223, (uint8_t)171, (uint8_t)86, (uint8_t)159, (uint8_t)217, (uint8_t)230, (uint8_t)238, (uint8_t)163, (uint8_t)240, (uint8_t)128, (uint8_t)255, (uint8_t)98, (uint8_t)202, (uint8_t)253, (uint8_t)111, (uint8_t)243, (uint8_t)102, (uint8_t)52, (uint8_t)95, (uint8_t)38, (uint8_t)134, (uint8_t)146, (uint8_t)241, (uint8_t)128, (uint8_t)52, (uint8_t)54, (uint8_t)151, (uint8_t)246, (uint8_t)109, (uint8_t)1, (uint8_t)236, (uint8_t)153, (uint8_t)139, (uint8_t)178, (uint8_t)130, (uint8_t)116, (uint8_t)67, (uint8_t)3, (uint8_t)116, (uint8_t)233, (uint8_t)207, (uint8_t)217, (uint8_t)204, (uint8_t)83, (uint8_t)141, (uint8_t)233, (uint8_t)238, (uint8_t)3, (uint8_t)32, (uint8_t)9, (uint8_t)180, (uint8_t)147, (uint8_t)170, (uint8_t)173, (uint8_t)242, (uint8_t)15, (uint8_t)250, (uint8_t)61, (uint8_t)161, (uint8_t)174, (uint8_t)88, (uint8_t)180, (uint8_t)252, (uint8_t)1, (uint8_t)64, (uint8_t)87, (uint8_t)66, (uint8_t)154, (uint8_t)19, (uint8_t)77, (uint8_t)179, (uint8_t)151, (uint8_t)147, (uint8_t)87, (uint8_t)76, (uint8_t)234, (uint8_t)134, (uint8_t)76, (uint8_t)65, (uint8_t)218, (uint8_t)72, (uint8_t)16, (uint8_t)186, (uint8_t)161, (uint8_t)67, (uint8_t)160, (uint8_t)48, (uint8_t)222, (uint8_t)169, (uint8_t)154, (uint8_t)13, (uint8_t)12, (uint8_t)122, (uint8_t)199, (uint8_t)53, (uint8_t)140, (uint8_t)188, (uint8_t)164, (uint8_t)241, (uint8_t)91, (uint8_t)39, (uint8_t)15, (uint8_t)234, (uint8_t)186, (uint8_t)202, (uint8_t)159, (uint8_t)49, (uint8_t)111, (uint8_t)245, (uint8_t)226, (uint8_t)190, (uint8_t)210, (uint8_t)104, (uint8_t)10, (uint8_t)21, (uint8_t)84, (uint8_t)191, (uint8_t)110, (uint8_t)193, (uint8_t)138, (uint8_t)79, (uint8_t)128, (uint8_t)148, (uint8_t)124, (uint8_t)95, (uint8_t)140, (uint8_t)53, (uint8_t)215, (uint8_t)17, (uint8_t)200, (uint8_t)74, (uint8_t)228, (uint8_t)72, (uint8_t)144, (uint8_t)91, (uint8_t)82, (uint8_t)254, (uint8_t)45, (uint8_t)117, (uint8_t)221, (uint8_t)249, (uint8_t)31, (uint8_t)248, (uint8_t)246, (uint8_t)216, (uint8_t)124, (uint8_t)85, (uint8_t)40, (uint8_t)75, (uint8_t)18, (uint8_t)178, (uint8_t)113, (uint8_t)237, (uint8_t)151, (uint8_t)73, (uint8_t)23, (uint8_t)140, (uint8_t)201, (uint8_t)187, (uint8_t)81, (uint8_t)20, (uint8_t)152, (uint8_t)244, (uint8_t)173, (uint8_t)65, (uint8_t)64, (uint8_t)141, (uint8_t)188, (uint8_t)190, (uint8_t)255, (uint8_t)163, (uint8_t)62, (uint8_t)46, (uint8_t)182, (uint8_t)26, (uint8_t)69, (uint8_t)68, (uint8_t)117, (uint8_t)138, (uint8_t)16, (uint8_t)189, (uint8_t)38, (uint8_t)186, (uint8_t)68, (uint8_t)103, (uint8_t)8, (uint8_t)101, (uint8_t)200, (uint8_t)152, (uint8_t)25, (uint8_t)135, (uint8_t)63, (uint8_t)162, (uint8_t)23, (uint8_t)10, (uint8_t)49, (uint8_t)166, (uint8_t)140, (uint8_t)120, (uint8_t)50, (uint8_t)12, (uint8_t)94, (uint8_t)130, (uint8_t)142, (uint8_t)5, (uint8_t)93, (uint8_t)249, (uint8_t)22, (uint8_t)141, (uint8_t)206, (uint8_t)101, (uint8_t)54, (uint8_t)113, (uint8_t)49, (uint8_t)97, (uint8_t)75, (uint8_t)226, (uint8_t)69, (uint8_t)230, (uint8_t)186, (uint8_t)189, (uint8_t)45, (uint8_t)136, (uint8_t)154, (uint8_t)106, (uint8_t)247, (uint8_t)142, (uint8_t)159, (uint8_t)211, (uint8_t)83, (uint8_t)255, (uint8_t)149, (uint8_t)6, (uint8_t)225, (uint8_t)101, (uint8_t)207, (uint8_t)237, (uint8_t)212};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_length_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)33445, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)7085, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_bitrate_SET((uint32_t)2961916442L, PH.base.pack) ;
        {
            char16_t* uri = u"tgbyacpUicxbaryfsvChuqwsEAPewgvrfusxwmoPuPsbvkzloacqlmBkcNdf";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_rotation_SET((uint16_t)(uint16_t)21600, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)50210, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)33685, PH.base.pack) ;
        p269_framerate_SET((float) -2.0645676E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        {
            char16_t* uri = u"FwareNlcxfjsqbes";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p270_framerate_SET((float) -3.3073377E38F, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)25238, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)1056494506L, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)61078, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)49855, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"zekc";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"pqzggbcmfhuaAmuiytxfbpvxcMautcFpzokzchonxzaxupz";
            p299_password_SET_(password, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)4388, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)80, (uint8_t)50, (uint8_t)19, (uint8_t)132, (uint8_t)4, (uint8_t)119, (uint8_t)46, (uint8_t)32};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)10012, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)1867, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)112, (uint8_t)226, (uint8_t)155, (uint8_t)69, (uint8_t)212, (uint8_t)126, (uint8_t)190, (uint8_t)16};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_sub_mode_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1141044434L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)35153, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)5558423984996120340L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_minor_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)224, (uint8_t)167, (uint8_t)104, (uint8_t)247, (uint8_t)196, (uint8_t)244, (uint8_t)92, (uint8_t)193, (uint8_t)56, (uint8_t)235, (uint8_t)89, (uint8_t)254, (uint8_t)64, (uint8_t)12, (uint8_t)105, (uint8_t)248};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        {
            char16_t* name = u"p";
            p311_name_SET_(name, &PH) ;
        }
        p311_uptime_sec_SET((uint32_t)2862478411L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)7788764367137332022L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)2183409286L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t)20420, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        {
            char16_t* param_id = u"ziXpu";
            p320_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)36467, PH.base.pack) ;
        {
            char16_t* param_value = u"UaamslhkbcncuxfifoyqcWrPGzzlteqQirTkcXbdszbmvrvewssvoxxjkz";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)17426, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"b";
            p322_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_value = u"ymraDLyrCkHxtVptymjacernwPmsnomdhhnlzhadfefwZqzVegrxMfkhdh";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        {
            char16_t* param_id = u"wjwttpaqffOlkkN";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_value = u"rcNvjjnpefCouiIxhrkyrblnbhklsbytxfukqLhqnfropwglyfgjtvAvAsTsudwiMqGwyqbyzymEtbwwvyelzpxrevvMRsszbuhtobefk";
            p324_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"VemsgvdgQC";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_FAILED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        {
            uint16_t distances[] =  {(uint16_t)18929, (uint16_t)65266, (uint16_t)6479, (uint16_t)44526, (uint16_t)20192, (uint16_t)28755, (uint16_t)59046, (uint16_t)27053, (uint16_t)58578, (uint16_t)61766, (uint16_t)65210, (uint16_t)3504, (uint16_t)48029, (uint16_t)49970, (uint16_t)9244, (uint16_t)53299, (uint16_t)15637, (uint16_t)40710, (uint16_t)40312, (uint16_t)20625, (uint16_t)60240, (uint16_t)21105, (uint16_t)15057, (uint16_t)56352, (uint16_t)10813, (uint16_t)6801, (uint16_t)42927, (uint16_t)59858, (uint16_t)64953, (uint16_t)13150, (uint16_t)17510, (uint16_t)26890, (uint16_t)21058, (uint16_t)58052, (uint16_t)32292, (uint16_t)39314, (uint16_t)45020, (uint16_t)60812, (uint16_t)43587, (uint16_t)61445, (uint16_t)56909, (uint16_t)51111, (uint16_t)55356, (uint16_t)38722, (uint16_t)15219, (uint16_t)46424, (uint16_t)62370, (uint16_t)880, (uint16_t)58627, (uint16_t)55996, (uint16_t)11930, (uint16_t)31163, (uint16_t)21917, (uint16_t)31049, (uint16_t)30821, (uint16_t)17747, (uint16_t)38075, (uint16_t)53359, (uint16_t)41913, (uint16_t)2955, (uint16_t)38576, (uint16_t)11150, (uint16_t)34597, (uint16_t)58687, (uint16_t)46829, (uint16_t)14596, (uint16_t)48346, (uint16_t)41836, (uint16_t)27493, (uint16_t)27612, (uint16_t)27955, (uint16_t)61971};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)5565474932719285631L, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)38382, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)54669, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


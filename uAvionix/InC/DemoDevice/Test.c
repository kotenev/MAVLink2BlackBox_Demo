
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2846872621L);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_HEXAROTOR);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)29834);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -69);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)12767);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)315);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)4176);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)38161);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)32826);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)24566);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)9943);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)30758);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)4784959757485567515L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1075659526L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2849205793L);
    assert(p3_afx_GET(pack) == (float) -1.100892E38F);
    assert(p3_vy_GET(pack) == (float)8.581586E37F);
    assert(p3_yaw_rate_GET(pack) == (float) -1.5233748E38F);
    assert(p3_y_GET(pack) == (float)1.7919209E38F);
    assert(p3_z_GET(pack) == (float) -9.140362E37F);
    assert(p3_afy_GET(pack) == (float)8.1050456E37F);
    assert(p3_x_GET(pack) == (float) -1.2205249E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)33017);
    assert(p3_yaw_GET(pack) == (float) -4.1118952E37F);
    assert(p3_vz_GET(pack) == (float)4.4696682E36F);
    assert(p3_afz_GET(pack) == (float) -2.9859808E38F);
    assert(p3_vx_GET(pack) == (float) -2.2676996E38F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p4_time_usec_GET(pack) == (uint64_t)9184937301410296783L);
    assert(p4_seq_GET(pack) == (uint32_t)4171955146L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_passkey_LEN(ph) == 25);
    {
        char16_t * exemplary = u"WiFTddvjfxacmRkntpPzpgahx";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)255);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 12);
    {
        char16_t * exemplary = u"znfdbecnopbz";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1909983533L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)20500);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p20_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"inhkhf";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)108);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)137);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"bgmrGwwepRad";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)57869);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)22917);
    assert(p22_param_value_GET(pack) == (float)1.4985009E38F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"yqxvadhlgcsnt";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p23_param_value_GET(pack) == (float) -2.9887366E38F);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_h_acc_TRY(ph) == (uint32_t)180808995L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)37223);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)5670);
    assert(p24_v_acc_TRY(ph) == (uint32_t)2853813640L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)708316385L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)4129936019L);
    assert(p24_lat_GET(pack) == (int32_t)2085981001);
    assert(p24_time_usec_GET(pack) == (uint64_t)1302991863874669400L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)29235);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p24_lon_GET(pack) == (int32_t)106586940);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1025913492);
    assert(p24_alt_GET(pack) == (int32_t)1302720578);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)17780);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)158);
    {
        uint8_t exemplary[] =  {(uint8_t)148, (uint8_t)150, (uint8_t)102, (uint8_t)247, (uint8_t)155, (uint8_t)135, (uint8_t)74, (uint8_t)96, (uint8_t)219, (uint8_t)204, (uint8_t)218, (uint8_t)91, (uint8_t)124, (uint8_t)189, (uint8_t)10, (uint8_t)134, (uint8_t)170, (uint8_t)142, (uint8_t)72, (uint8_t)254} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)43, (uint8_t)207, (uint8_t)235, (uint8_t)131, (uint8_t)6, (uint8_t)10, (uint8_t)21, (uint8_t)244, (uint8_t)228, (uint8_t)40, (uint8_t)195, (uint8_t)201, (uint8_t)77, (uint8_t)206, (uint8_t)142, (uint8_t)28, (uint8_t)203, (uint8_t)239, (uint8_t)73, (uint8_t)102} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)55, (uint8_t)38, (uint8_t)95, (uint8_t)228, (uint8_t)74, (uint8_t)240, (uint8_t)83, (uint8_t)48, (uint8_t)100, (uint8_t)146, (uint8_t)27, (uint8_t)115, (uint8_t)140, (uint8_t)75, (uint8_t)215, (uint8_t)225, (uint8_t)238, (uint8_t)199, (uint8_t)251} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)135, (uint8_t)96, (uint8_t)218, (uint8_t)51, (uint8_t)154, (uint8_t)207, (uint8_t)8, (uint8_t)156, (uint8_t)107, (uint8_t)235, (uint8_t)137, (uint8_t)198, (uint8_t)234, (uint8_t)69, (uint8_t)20, (uint8_t)75, (uint8_t)132, (uint8_t)16, (uint8_t)87} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)188, (uint8_t)181, (uint8_t)240, (uint8_t)137, (uint8_t)190, (uint8_t)195, (uint8_t)15, (uint8_t)9, (uint8_t)123, (uint8_t)226, (uint8_t)100, (uint8_t)169, (uint8_t)186, (uint8_t)202, (uint8_t)93, (uint8_t)224, (uint8_t)151, (uint8_t)213, (uint8_t)129} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -18339);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -25768);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -19837);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -4430);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)8172);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)30900);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)26373);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -10575);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3283396644L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)19392);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)8539);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)3607);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)20834);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -30307);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -9528);
    assert(p27_time_usec_GET(pack) == (uint64_t)3009493261919783439L);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -21070);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -8065);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)11670);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)15892);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -31281);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)26324);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)13508);
    assert(p28_time_usec_GET(pack) == (uint64_t)1081201902607922921L);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -23166);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)15038);
    assert(p29_press_abs_GET(pack) == (float)3.6982352E37F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)2351876590L);
    assert(p29_press_diff_GET(pack) == (float) -1.0771516E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitchspeed_GET(pack) == (float)1.2689446E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)4039989638L);
    assert(p30_yaw_GET(pack) == (float) -3.1590895E38F);
    assert(p30_yawspeed_GET(pack) == (float) -3.1675122E38F);
    assert(p30_roll_GET(pack) == (float) -2.9920722E38F);
    assert(p30_rollspeed_GET(pack) == (float) -7.690485E37F);
    assert(p30_pitch_GET(pack) == (float)1.8053663E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float) -1.1834767E38F);
    assert(p31_yawspeed_GET(pack) == (float)1.0085906E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1249481086L);
    assert(p31_q4_GET(pack) == (float) -1.5622091E38F);
    assert(p31_q1_GET(pack) == (float)1.3108195E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -1.8682282E38F);
    assert(p31_rollspeed_GET(pack) == (float)2.7053203E38F);
    assert(p31_q2_GET(pack) == (float) -2.7060572E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float)2.1729438E37F);
    assert(p32_y_GET(pack) == (float)2.6119474E38F);
    assert(p32_x_GET(pack) == (float)2.8229315E38F);
    assert(p32_vx_GET(pack) == (float) -2.3827655E38F);
    assert(p32_vz_GET(pack) == (float)2.617849E38F);
    assert(p32_vy_GET(pack) == (float)1.6222978E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2048823475L);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)562960163L);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -5393);
    assert(p33_relative_alt_GET(pack) == (int32_t)48482744);
    assert(p33_lat_GET(pack) == (int32_t) -1505531515);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)31556);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)14715);
    assert(p33_alt_GET(pack) == (int32_t)80931478);
    assert(p33_lon_GET(pack) == (int32_t) -830453906);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -20899);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -21670);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)17096);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)8402);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)7889);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -29336);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -22112);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)9406);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)20910);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3821546188L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)27927);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)2255);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3071884236L);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)11110);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)52468);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)10642);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)19512);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)62621);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)7534);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)163);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)58369);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)50575);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)39191);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)49939);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)13074);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)17297);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)7914);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)34716);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)64146);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)764);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)19692);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)45989);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)36503);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)59702);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)29315);
    assert(p36_time_usec_GET(pack) == (uint32_t)3797162634L);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)10193);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)0);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)8566);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)19848);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)117);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -24076);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)31436);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)250);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_z_GET(pack) == (float) -1.672688E38F);
    assert(p39_param4_GET(pack) == (float)2.8304539E37F);
    assert(p39_param3_GET(pack) == (float)1.1399684E37F);
    assert(p39_y_GET(pack) == (float) -1.8020665E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p39_param1_GET(pack) == (float)1.2820037E38F);
    assert(p39_param2_GET(pack) == (float)2.6977538E38F);
    assert(p39_x_GET(pack) == (float)1.0684488E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_TAKEOFF);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)51226);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)60105);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)206);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)4123);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)36635);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)53587);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)72);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)1329);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t) -1025151751);
    assert(p48_latitude_GET(pack) == (int32_t)1995037183);
    assert(p48_longitude_GET(pack) == (int32_t) -1317611143);
    assert(p48_time_usec_TRY(ph) == (uint64_t)8338677883610571123L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)19);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)6460606635588797508L);
    assert(p49_longitude_GET(pack) == (int32_t) -15738783);
    assert(p49_altitude_GET(pack) == (int32_t) -282844977);
    assert(p49_latitude_GET(pack) == (int32_t)1991085064);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p50_param_value_max_GET(pack) == (float) -2.5403398E38F);
    assert(p50_param_value_min_GET(pack) == (float) -2.6752198E38F);
    assert(p50_param_value0_GET(pack) == (float) -1.847334E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)3023);
    assert(p50_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"bs";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p50_scale_GET(pack) == (float) -3.3366048E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)11825);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2z_GET(pack) == (float)1.9626367E38F);
    assert(p54_p1x_GET(pack) == (float) -8.579643E37F);
    assert(p54_p1z_GET(pack) == (float) -2.1910043E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p54_p1y_GET(pack) == (float)3.2434236E38F);
    assert(p54_p2y_GET(pack) == (float) -9.834256E37F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p54_p2x_GET(pack) == (float)1.882319E38F);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)1.3479748E38F);
    assert(p55_p1z_GET(pack) == (float) -2.6329831E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p55_p2y_GET(pack) == (float) -1.7949535E38F);
    assert(p55_p1y_GET(pack) == (float)1.2456274E38F);
    assert(p55_p1x_GET(pack) == (float) -1.7283765E38F);
    assert(p55_p2x_GET(pack) == (float) -2.5488403E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_rollspeed_GET(pack) == (float) -2.696671E38F);
    assert(p61_yawspeed_GET(pack) == (float)3.1148378E38F);
    assert(p61_pitchspeed_GET(pack) == (float) -2.4340182E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)4675482351010357898L);
    {
        float exemplary[] =  {1.0621552E38F, 3.0290988E38F, 2.436592E38F, 5.0377413E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {8.165272E37F, 3.0927054E38F, -2.2008067E38F, -3.9067437E37F, -1.4170093E38F, -1.346112E38F, 6.344775E37F, -2.3702724E38F, -1.4782414E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -24338);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)37962);
    assert(p62_alt_error_GET(pack) == (float) -2.5891293E38F);
    assert(p62_xtrack_error_GET(pack) == (float)2.204979E38F);
    assert(p62_nav_roll_GET(pack) == (float) -6.2034865E37F);
    assert(p62_nav_pitch_GET(pack) == (float)1.3883771E38F);
    assert(p62_aspd_error_GET(pack) == (float)3.3461378E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)28099);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lat_GET(pack) == (int32_t)474236843);
    assert(p63_vx_GET(pack) == (float) -3.2289359E38F);
    {
        float exemplary[] =  {2.9942266E38F, -1.2676215E38F, 2.0007615E38F, 9.554482E37F, 2.191364E38F, -1.5273871E38F, 2.94559E37F, -2.1359898E38F, 2.3401942E38F, 1.1799885E38F, 1.6316442E38F, -2.1682502E38F, -2.3230861E38F, -2.3886298E38F, -1.2376459E38F, 1.720023E38F, -1.1834975E38F, -2.6051906E37F, -2.055309E38F, 7.6769863E37F, -2.8145001E38F, 4.1323713E37F, 3.0285567E37F, 3.1631838E38F, 2.1605476E38F, 5.7672976E37F, -7.726237E37F, 1.6889639E38F, 2.6946877E38F, 1.592536E38F, 3.1202952E38F, 8.905509E37F, -3.3556614E38F, -3.6483116E37F, 2.4812873E38F, 1.1961792E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_relative_alt_GET(pack) == (int32_t)1414163134);
    assert(p63_time_usec_GET(pack) == (uint64_t)1287536392926164806L);
    assert(p63_vz_GET(pack) == (float)3.0441579E38F);
    assert(p63_lon_GET(pack) == (int32_t) -1632462143);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p63_vy_GET(pack) == (float) -3.2279201E38F);
    assert(p63_alt_GET(pack) == (int32_t)72903641);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-5.717297E37F, -1.2810899E38F, -2.459921E38F, -2.7537954E36F, 2.810831E38F, 2.131665E38F, 1.9206236E36F, -2.0909554E38F, 3.055756E36F, -6.3793127E37F, -1.7584476E37F, -1.668108E38F, -1.6850094E38F, 2.0916148E38F, -2.4743497E38F, 2.4811788E38F, 3.0421574E38F, 1.9162004E37F, 1.545504E38F, 1.2703041E38F, 2.4839425E38F, -1.0850406E37F, 1.5645537E38F, -2.1544168E38F, -6.177713E37F, -3.2730207E38F, 6.8880326E37F, -4.8085253E37F, 8.553787E37F, 1.0169862E38F, 1.6755359E38F, 2.8271085E38F, -2.1191072E38F, 1.4756658E38F, 1.676619E38F, -2.0598183E37F, -2.9134748E37F, -3.025896E38F, -1.3771971E38F, -2.4190211E38F, 1.8940404E38F, -3.6071272E37F, 1.154085E37F, 3.0751542E38F, -2.429455E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_ay_GET(pack) == (float) -1.9531493E37F);
    assert(p64_z_GET(pack) == (float) -1.8349934E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)7253467739226297578L);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_vz_GET(pack) == (float) -2.422461E38F);
    assert(p64_x_GET(pack) == (float) -1.5725996E38F);
    assert(p64_ax_GET(pack) == (float)7.0729797E37F);
    assert(p64_vx_GET(pack) == (float) -5.124944E37F);
    assert(p64_vy_GET(pack) == (float) -2.9586277E38F);
    assert(p64_y_GET(pack) == (float)1.1485664E38F);
    assert(p64_az_GET(pack) == (float)5.3661876E37F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)54088);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)58149);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)50943);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)31721);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)30374);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)34005);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)33072);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)34166);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)57063);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)23593);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)43430);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)27409);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)874558232L);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)51220);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)20426);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)50702);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)46471);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)12349);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)11949);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)47301);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)64597);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)25);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -20692);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -7526);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -24412);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -16539);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)38975);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)49918);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)14544);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)54725);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)37363);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)30187);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)25765);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)2313);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)42564);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float)1.3679726E38F);
    assert(p73_x_GET(pack) == (int32_t) -1192387100);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)36015);
    assert(p73_param1_GET(pack) == (float) -2.1044831E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p73_y_GET(pack) == (int32_t)752618337);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p73_param3_GET(pack) == (float)1.4755377E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p73_param2_GET(pack) == (float)2.1886637E38F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM);
    assert(p73_z_GET(pack) == (float)2.6817875E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)25341);
    assert(p74_airspeed_GET(pack) == (float)3.7827148E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -9154);
    assert(p74_climb_GET(pack) == (float) -1.8870782E37F);
    assert(p74_groundspeed_GET(pack) == (float)4.0363235E37F);
    assert(p74_alt_GET(pack) == (float)1.8620475E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param2_GET(pack) == (float)1.4385889E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p75_param1_GET(pack) == (float) -1.3971386E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p75_x_GET(pack) == (int32_t)397943247);
    assert(p75_y_GET(pack) == (int32_t) -1136960657);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p75_param3_GET(pack) == (float)2.5616286E38F);
    assert(p75_z_GET(pack) == (float) -3.0146338E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5);
    assert(p75_param4_GET(pack) == (float) -6.847252E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p76_param1_GET(pack) == (float)7.9311204E37F);
    assert(p76_param3_GET(pack) == (float)2.2642313E38F);
    assert(p76_param2_GET(pack) == (float) -3.3003907E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p76_param7_GET(pack) == (float) -3.3735247E38F);
    assert(p76_param5_GET(pack) == (float) -2.7794087E38F);
    assert(p76_param6_GET(pack) == (float)2.5125912E38F);
    assert(p76_param4_GET(pack) == (float)2.9241515E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)104);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)195);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)248);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)77);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_DENIED);
    assert(p77_result_param2_TRY(ph) == (int32_t)1182527019);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p81_roll_GET(pack) == (float)7.6374235E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2969319959L);
    assert(p81_yaw_GET(pack) == (float) -1.3941062E38F);
    assert(p81_pitch_GET(pack) == (float) -2.4076883E38F);
    assert(p81_thrust_GET(pack) == (float)1.033224E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)96);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_roll_rate_GET(pack) == (float) -2.7712706E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float)1.2590558E38F);
    {
        float exemplary[] =  {2.7069113E38F, -2.2669607E38F, 3.1499022E38F, 8.0021565E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_thrust_GET(pack) == (float) -4.7566374E36F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)188274701L);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p82_body_yaw_rate_GET(pack) == (float) -6.422117E37F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_roll_rate_GET(pack) == (float)2.7926404E37F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)955469886L);
    assert(p83_body_yaw_rate_GET(pack) == (float) -8.541115E37F);
    assert(p83_thrust_GET(pack) == (float) -1.1036724E38F);
    assert(p83_body_pitch_rate_GET(pack) == (float) -2.5176433E38F);
    {
        float exemplary[] =  {-3.2788339E38F, 1.3004757E38F, -2.2170625E38F, 8.753965E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p84_vz_GET(pack) == (float) -1.1032094E38F);
    assert(p84_x_GET(pack) == (float) -9.961106E37F);
    assert(p84_afz_GET(pack) == (float)3.1849688E37F);
    assert(p84_afx_GET(pack) == (float)6.714006E37F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)49748);
    assert(p84_z_GET(pack) == (float)2.5513117E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p84_vx_GET(pack) == (float)2.9856378E38F);
    assert(p84_afy_GET(pack) == (float)1.1183315E38F);
    assert(p84_yaw_GET(pack) == (float) -1.4218246E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -2.3743722E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)76500234L);
    assert(p84_y_GET(pack) == (float) -2.1280946E37F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p84_vy_GET(pack) == (float)2.9379032E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lon_int_GET(pack) == (int32_t) -1475867935);
    assert(p86_lat_int_GET(pack) == (int32_t) -1767351976);
    assert(p86_yaw_GET(pack) == (float) -1.4781159E38F);
    assert(p86_vz_GET(pack) == (float)9.854613E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)31058);
    assert(p86_afy_GET(pack) == (float) -4.6877075E37F);
    assert(p86_afz_GET(pack) == (float)3.288963E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p86_vx_GET(pack) == (float) -2.8735469E38F);
    assert(p86_vy_GET(pack) == (float)2.0595909E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p86_yaw_rate_GET(pack) == (float) -1.1078116E38F);
    assert(p86_alt_GET(pack) == (float) -1.4217686E37F);
    assert(p86_afx_GET(pack) == (float) -1.971117E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)217912311L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lat_int_GET(pack) == (int32_t)1073302634);
    assert(p87_yaw_rate_GET(pack) == (float)1.7711387E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -964071520);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p87_afx_GET(pack) == (float) -2.3531459E38F);
    assert(p87_alt_GET(pack) == (float)2.8615237E38F);
    assert(p87_afy_GET(pack) == (float)2.5270262E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)3844022306L);
    assert(p87_yaw_GET(pack) == (float) -2.2203101E38F);
    assert(p87_vx_GET(pack) == (float) -5.2810534E35F);
    assert(p87_vz_GET(pack) == (float) -1.1625726E38F);
    assert(p87_vy_GET(pack) == (float)9.99504E37F);
    assert(p87_afz_GET(pack) == (float) -1.9494164E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)63742);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_z_GET(pack) == (float)1.0395975E37F);
    assert(p89_y_GET(pack) == (float) -1.0811489E38F);
    assert(p89_roll_GET(pack) == (float) -3.2488635E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3292092632L);
    assert(p89_x_GET(pack) == (float) -2.4427605E38F);
    assert(p89_yaw_GET(pack) == (float)2.706544E38F);
    assert(p89_pitch_GET(pack) == (float) -3.1900529E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_lat_GET(pack) == (int32_t) -1129176904);
    assert(p90_alt_GET(pack) == (int32_t)1858100650);
    assert(p90_pitch_GET(pack) == (float) -3.2701412E38F);
    assert(p90_rollspeed_GET(pack) == (float)8.3409746E37F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -10612);
    assert(p90_pitchspeed_GET(pack) == (float)1.7553162E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)1183);
    assert(p90_yawspeed_GET(pack) == (float)1.7386274E38F);
    assert(p90_roll_GET(pack) == (float)3.1373184E38F);
    assert(p90_lon_GET(pack) == (int32_t) -890497869);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -18500);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -23750);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -14976);
    assert(p90_time_usec_GET(pack) == (uint64_t)7497671567642414074L);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)15968);
    assert(p90_yaw_GET(pack) == (float)8.62847E37F);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p91_pitch_elevator_GET(pack) == (float)3.0146788E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)1552290244768926648L);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p91_aux3_GET(pack) == (float)3.0611118E35F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.0673766E37F);
    assert(p91_aux2_GET(pack) == (float)1.9018047E38F);
    assert(p91_aux1_GET(pack) == (float) -1.2274248E38F);
    assert(p91_aux4_GET(pack) == (float) -2.9718275E38F);
    assert(p91_throttle_GET(pack) == (float)2.4743282E38F);
    assert(p91_roll_ailerons_GET(pack) == (float) -1.3848734E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)54070);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)29756);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)42221);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)4432);
    assert(p92_time_usec_GET(pack) == (uint64_t)412064918094905856L);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)30011);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)38488);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)64059);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)58821);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)4145);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)20679);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)62218);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)41935);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)4426328200019213071L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
    {
        float exemplary[] =  {-1.5533956E38F, 1.9830187E38F, -2.429995E38F, 5.6048664E37F, -7.39881E37F, -5.9986043E37F, -2.3820433E38F, -1.7342282E38F, 2.8929196E38F, 1.7221893E38F, -2.8397059E38F, -2.0028443E38F, -1.3416745E38F, -3.0867355E38F, 1.8827463E38F, 4.0581532E37F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)3135938441777206533L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_comp_m_y_GET(pack) == (float) -1.1332797E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -2.0314075E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)3.1196204E38F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -8.8852724E36F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -6719);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p100_time_usec_GET(pack) == (uint64_t)1317262055054710568L);
    assert(p100_ground_distance_GET(pack) == (float) -3.818688E37F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -2554);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_x_GET(pack) == (float)3.2970036E38F);
    assert(p101_pitch_GET(pack) == (float) -1.6935885E38F);
    assert(p101_roll_GET(pack) == (float)2.9305695E38F);
    assert(p101_usec_GET(pack) == (uint64_t)6476344392136675534L);
    assert(p101_z_GET(pack) == (float) -2.5197444E38F);
    assert(p101_y_GET(pack) == (float) -1.3543031E38F);
    assert(p101_yaw_GET(pack) == (float)2.1679953E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float)1.7099578E38F);
    assert(p102_x_GET(pack) == (float)8.964018E37F);
    assert(p102_roll_GET(pack) == (float)4.315045E37F);
    assert(p102_pitch_GET(pack) == (float) -5.3576294E37F);
    assert(p102_y_GET(pack) == (float)9.452749E37F);
    assert(p102_usec_GET(pack) == (uint64_t)4638222224801586001L);
    assert(p102_yaw_GET(pack) == (float) -3.2377847E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)2.8554413E36F);
    assert(p103_x_GET(pack) == (float)1.2931913E38F);
    assert(p103_z_GET(pack) == (float) -5.3343604E37F);
    assert(p103_usec_GET(pack) == (uint64_t)8438729123838443953L);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float)2.5389275E38F);
    assert(p104_pitch_GET(pack) == (float) -5.831651E37F);
    assert(p104_yaw_GET(pack) == (float)1.8242476E38F);
    assert(p104_z_GET(pack) == (float)2.1864047E37F);
    assert(p104_y_GET(pack) == (float) -2.0415018E38F);
    assert(p104_usec_GET(pack) == (uint64_t)3059256821134229087L);
    assert(p104_roll_GET(pack) == (float) -1.0812915E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_xacc_GET(pack) == (float)2.289248E38F);
    assert(p105_zacc_GET(pack) == (float)9.116989E37F);
    assert(p105_xmag_GET(pack) == (float) -3.257899E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.0692252E38F);
    assert(p105_yacc_GET(pack) == (float) -2.0078577E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)3963);
    assert(p105_abs_pressure_GET(pack) == (float)3.2357757E38F);
    assert(p105_pressure_alt_GET(pack) == (float)2.6658236E38F);
    assert(p105_zgyro_GET(pack) == (float) -2.2355635E38F);
    assert(p105_ygyro_GET(pack) == (float)2.0541045E38F);
    assert(p105_xgyro_GET(pack) == (float) -1.9783618E37F);
    assert(p105_zmag_GET(pack) == (float) -1.0806922E37F);
    assert(p105_temperature_GET(pack) == (float)3.0856023E38F);
    assert(p105_ymag_GET(pack) == (float) -1.7502079E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8745238437357896093L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_zgyro_GET(pack) == (float)1.1066599E38F);
    assert(p106_integrated_y_GET(pack) == (float)1.7496268E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float)3.3041473E37F);
    assert(p106_distance_GET(pack) == (float) -1.5039352E37F);
    assert(p106_time_usec_GET(pack) == (uint64_t)3510015996464041946L);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1939953679L);
    assert(p106_integrated_x_GET(pack) == (float) -2.057138E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)1722980125L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)24090);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p106_integrated_xgyro_GET(pack) == (float) -9.1160925E36F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_ygyro_GET(pack) == (float) -2.3266449E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)3128645984L);
    assert(p107_abs_pressure_GET(pack) == (float)7.1292097E37F);
    assert(p107_zgyro_GET(pack) == (float)8.606418E37F);
    assert(p107_pressure_alt_GET(pack) == (float) -3.3569491E38F);
    assert(p107_temperature_GET(pack) == (float)4.068874E36F);
    assert(p107_ymag_GET(pack) == (float)2.7712556E38F);
    assert(p107_xmag_GET(pack) == (float)2.5724967E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)857491189792234929L);
    assert(p107_xgyro_GET(pack) == (float)2.1570616E37F);
    assert(p107_xacc_GET(pack) == (float) -4.529736E36F);
    assert(p107_zacc_GET(pack) == (float) -2.059394E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -2.3436265E38F);
    assert(p107_yacc_GET(pack) == (float)2.8312613E38F);
    assert(p107_zmag_GET(pack) == (float) -2.8535174E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_alt_GET(pack) == (float)7.2477533E37F);
    assert(p108_vn_GET(pack) == (float) -2.3093105E38F);
    assert(p108_xgyro_GET(pack) == (float) -1.0773022E38F);
    assert(p108_vd_GET(pack) == (float)2.5978625E38F);
    assert(p108_q2_GET(pack) == (float)3.9042437E37F);
    assert(p108_lat_GET(pack) == (float)2.7417903E38F);
    assert(p108_yacc_GET(pack) == (float) -2.8748974E37F);
    assert(p108_ygyro_GET(pack) == (float)5.3953223E37F);
    assert(p108_std_dev_vert_GET(pack) == (float) -4.502821E37F);
    assert(p108_q1_GET(pack) == (float)1.3534657E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)1.0239214E38F);
    assert(p108_lon_GET(pack) == (float) -8.897602E37F);
    assert(p108_ve_GET(pack) == (float)3.1097086E38F);
    assert(p108_q3_GET(pack) == (float)3.1104574E38F);
    assert(p108_q4_GET(pack) == (float)2.6790418E38F);
    assert(p108_xacc_GET(pack) == (float) -2.3081778E37F);
    assert(p108_yaw_GET(pack) == (float)9.253704E37F);
    assert(p108_roll_GET(pack) == (float)1.4329623E38F);
    assert(p108_pitch_GET(pack) == (float) -2.9004842E38F);
    assert(p108_zacc_GET(pack) == (float)8.582227E36F);
    assert(p108_zgyro_GET(pack) == (float)1.8721875E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)22239);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)28695);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)69);
    {
        uint8_t exemplary[] =  {(uint8_t)51, (uint8_t)232, (uint8_t)154, (uint8_t)130, (uint8_t)17, (uint8_t)92, (uint8_t)157, (uint8_t)224, (uint8_t)40, (uint8_t)84, (uint8_t)148, (uint8_t)158, (uint8_t)102, (uint8_t)77, (uint8_t)96, (uint8_t)118, (uint8_t)156, (uint8_t)76, (uint8_t)116, (uint8_t)118, (uint8_t)214, (uint8_t)218, (uint8_t)37, (uint8_t)61, (uint8_t)174, (uint8_t)156, (uint8_t)115, (uint8_t)131, (uint8_t)104, (uint8_t)95, (uint8_t)169, (uint8_t)83, (uint8_t)244, (uint8_t)49, (uint8_t)212, (uint8_t)228, (uint8_t)253, (uint8_t)95, (uint8_t)79, (uint8_t)88, (uint8_t)35, (uint8_t)172, (uint8_t)7, (uint8_t)106, (uint8_t)3, (uint8_t)210, (uint8_t)11, (uint8_t)55, (uint8_t)96, (uint8_t)211, (uint8_t)176, (uint8_t)11, (uint8_t)229, (uint8_t)109, (uint8_t)201, (uint8_t)64, (uint8_t)7, (uint8_t)192, (uint8_t)47, (uint8_t)168, (uint8_t)63, (uint8_t)93, (uint8_t)156, (uint8_t)251, (uint8_t)182, (uint8_t)181, (uint8_t)45, (uint8_t)143, (uint8_t)206, (uint8_t)152, (uint8_t)7, (uint8_t)211, (uint8_t)25, (uint8_t)218, (uint8_t)248, (uint8_t)63, (uint8_t)125, (uint8_t)151, (uint8_t)162, (uint8_t)160, (uint8_t)150, (uint8_t)103, (uint8_t)96, (uint8_t)135, (uint8_t)29, (uint8_t)242, (uint8_t)97, (uint8_t)56, (uint8_t)125, (uint8_t)170, (uint8_t)177, (uint8_t)164, (uint8_t)226, (uint8_t)26, (uint8_t)39, (uint8_t)212, (uint8_t)15, (uint8_t)95, (uint8_t)136, (uint8_t)97, (uint8_t)131, (uint8_t)188, (uint8_t)79, (uint8_t)117, (uint8_t)54, (uint8_t)80, (uint8_t)238, (uint8_t)253, (uint8_t)227, (uint8_t)217, (uint8_t)92, (uint8_t)161, (uint8_t)37, (uint8_t)14, (uint8_t)136, (uint8_t)175, (uint8_t)177, (uint8_t)220, (uint8_t)235, (uint8_t)38, (uint8_t)173, (uint8_t)161, (uint8_t)24, (uint8_t)246, (uint8_t)117, (uint8_t)8, (uint8_t)223, (uint8_t)102, (uint8_t)201, (uint8_t)1, (uint8_t)80, (uint8_t)31, (uint8_t)252, (uint8_t)201, (uint8_t)206, (uint8_t)46, (uint8_t)60, (uint8_t)27, (uint8_t)28, (uint8_t)45, (uint8_t)126, (uint8_t)100, (uint8_t)68, (uint8_t)241, (uint8_t)196, (uint8_t)171, (uint8_t)74, (uint8_t)11, (uint8_t)39, (uint8_t)245, (uint8_t)173, (uint8_t)11, (uint8_t)146, (uint8_t)56, (uint8_t)238, (uint8_t)100, (uint8_t)97, (uint8_t)107, (uint8_t)160, (uint8_t)191, (uint8_t)128, (uint8_t)14, (uint8_t)220, (uint8_t)65, (uint8_t)124, (uint8_t)152, (uint8_t)146, (uint8_t)172, (uint8_t)240, (uint8_t)226, (uint8_t)85, (uint8_t)183, (uint8_t)17, (uint8_t)35, (uint8_t)88, (uint8_t)55, (uint8_t)205, (uint8_t)192, (uint8_t)63, (uint8_t)110, (uint8_t)235, (uint8_t)255, (uint8_t)31, (uint8_t)139, (uint8_t)205, (uint8_t)40, (uint8_t)6, (uint8_t)35, (uint8_t)65, (uint8_t)217, (uint8_t)129, (uint8_t)10, (uint8_t)172, (uint8_t)174, (uint8_t)84, (uint8_t)225, (uint8_t)241, (uint8_t)6, (uint8_t)95, (uint8_t)106, (uint8_t)81, (uint8_t)247, (uint8_t)198, (uint8_t)164, (uint8_t)237, (uint8_t)121, (uint8_t)173, (uint8_t)212, (uint8_t)233, (uint8_t)30, (uint8_t)118, (uint8_t)160, (uint8_t)31, (uint8_t)73, (uint8_t)31, (uint8_t)149, (uint8_t)14, (uint8_t)248, (uint8_t)58, (uint8_t)124, (uint8_t)106, (uint8_t)133, (uint8_t)164, (uint8_t)189, (uint8_t)148, (uint8_t)154, (uint8_t)246, (uint8_t)127, (uint8_t)126, (uint8_t)65, (uint8_t)142, (uint8_t)75, (uint8_t)151, (uint8_t)14, (uint8_t)154, (uint8_t)28, (uint8_t)28, (uint8_t)239, (uint8_t)235, (uint8_t)37, (uint8_t)17, (uint8_t)133, (uint8_t)140, (uint8_t)77, (uint8_t)104, (uint8_t)161, (uint8_t)219, (uint8_t)115, (uint8_t)209, (uint8_t)62, (uint8_t)206} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -7382875968158081303L);
    assert(p111_tc1_GET(pack) == (int64_t)4029272846493900268L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)3743966414L);
    assert(p112_time_usec_GET(pack) == (uint64_t)2846838210756728428L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_alt_GET(pack) == (int32_t)1007628643);
    assert(p113_lon_GET(pack) == (int32_t) -1400140437);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)36402);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)17074);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)24730);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)15119);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)20449);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p113_lat_GET(pack) == (int32_t)1121042780);
    assert(p113_time_usec_GET(pack) == (uint64_t)7728435531944599256L);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -22595);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -8336);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)173);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_zgyro_GET(pack) == (float)1.0729588E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)9130195839528726736L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.5710275E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3067354825L);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.7592193E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -15623);
    assert(p114_integrated_y_GET(pack) == (float)3.3415677E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4015842988L);
    assert(p114_integrated_x_GET(pack) == (float) -1.9672211E38F);
    assert(p114_distance_GET(pack) == (float)2.798916E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -19949);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)18700);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)32275);
    {
        float exemplary[] =  {-3.944703E37F, 3.325525E38F, -1.5990773E38F, -2.5448662E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_time_usec_GET(pack) == (uint64_t)9045377146671676229L);
    assert(p115_rollspeed_GET(pack) == (float)7.899727E37F);
    assert(p115_pitchspeed_GET(pack) == (float)4.555618E37F);
    assert(p115_lon_GET(pack) == (int32_t) -443038763);
    assert(p115_lat_GET(pack) == (int32_t) -2015383438);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -8702);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -15754);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)5988);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -31567);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)9556);
    assert(p115_yawspeed_GET(pack) == (float)2.2835967E38F);
    assert(p115_alt_GET(pack) == (int32_t) -534474673);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2576133825L);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -31689);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)31260);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -18188);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -16566);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -19061);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)22577);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)6281);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)32420);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -18965);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)289);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)317);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)8735);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)841);
    assert(p118_size_GET(pack) == (uint32_t)4229396479L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)53364);
    assert(p118_time_utc_GET(pack) == (uint32_t)3042105300L);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_count_GET(pack) == (uint32_t)4053127005L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)35008);
    assert(p119_ofs_GET(pack) == (uint32_t)1266636614L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)62);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)59679);
    {
        uint8_t exemplary[] =  {(uint8_t)59, (uint8_t)225, (uint8_t)34, (uint8_t)139, (uint8_t)116, (uint8_t)217, (uint8_t)174, (uint8_t)22, (uint8_t)184, (uint8_t)71, (uint8_t)63, (uint8_t)123, (uint8_t)89, (uint8_t)250, (uint8_t)226, (uint8_t)72, (uint8_t)113, (uint8_t)137, (uint8_t)135, (uint8_t)47, (uint8_t)236, (uint8_t)94, (uint8_t)11, (uint8_t)82, (uint8_t)185, (uint8_t)109, (uint8_t)83, (uint8_t)223, (uint8_t)133, (uint8_t)159, (uint8_t)224, (uint8_t)13, (uint8_t)62, (uint8_t)208, (uint8_t)69, (uint8_t)100, (uint8_t)210, (uint8_t)241, (uint8_t)179, (uint8_t)202, (uint8_t)57, (uint8_t)169, (uint8_t)174, (uint8_t)133, (uint8_t)206, (uint8_t)212, (uint8_t)125, (uint8_t)232, (uint8_t)39, (uint8_t)137, (uint8_t)163, (uint8_t)99, (uint8_t)239, (uint8_t)105, (uint8_t)179, (uint8_t)25, (uint8_t)135, (uint8_t)223, (uint8_t)57, (uint8_t)99, (uint8_t)17, (uint8_t)126, (uint8_t)23, (uint8_t)140, (uint8_t)28, (uint8_t)136, (uint8_t)129, (uint8_t)236, (uint8_t)191, (uint8_t)86, (uint8_t)241, (uint8_t)53, (uint8_t)14, (uint8_t)54, (uint8_t)157, (uint8_t)99, (uint8_t)205, (uint8_t)14, (uint8_t)50, (uint8_t)252, (uint8_t)121, (uint8_t)250, (uint8_t)240, (uint8_t)167, (uint8_t)121, (uint8_t)218, (uint8_t)98, (uint8_t)194, (uint8_t)167, (uint8_t)178} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)1085411480L);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)62);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)18);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)131);
    {
        uint8_t exemplary[] =  {(uint8_t)172, (uint8_t)148, (uint8_t)59, (uint8_t)30, (uint8_t)246, (uint8_t)21, (uint8_t)138, (uint8_t)99, (uint8_t)106, (uint8_t)178, (uint8_t)34, (uint8_t)141, (uint8_t)41, (uint8_t)245, (uint8_t)244, (uint8_t)6, (uint8_t)168, (uint8_t)159, (uint8_t)116, (uint8_t)63, (uint8_t)86, (uint8_t)232, (uint8_t)59, (uint8_t)0, (uint8_t)118, (uint8_t)191, (uint8_t)115, (uint8_t)47, (uint8_t)230, (uint8_t)231, (uint8_t)160, (uint8_t)31, (uint8_t)88, (uint8_t)151, (uint8_t)105, (uint8_t)33, (uint8_t)92, (uint8_t)98, (uint8_t)205, (uint8_t)228, (uint8_t)203, (uint8_t)52, (uint8_t)204, (uint8_t)125, (uint8_t)96, (uint8_t)220, (uint8_t)211, (uint8_t)253, (uint8_t)180, (uint8_t)28, (uint8_t)43, (uint8_t)167, (uint8_t)82, (uint8_t)202, (uint8_t)103, (uint8_t)3, (uint8_t)91, (uint8_t)211, (uint8_t)36, (uint8_t)105, (uint8_t)32, (uint8_t)17, (uint8_t)101, (uint8_t)111, (uint8_t)183, (uint8_t)189, (uint8_t)22, (uint8_t)36, (uint8_t)157, (uint8_t)119, (uint8_t)101, (uint8_t)175, (uint8_t)75, (uint8_t)17, (uint8_t)87, (uint8_t)246, (uint8_t)111, (uint8_t)248, (uint8_t)212, (uint8_t)26, (uint8_t)82, (uint8_t)65, (uint8_t)130, (uint8_t)5, (uint8_t)236, (uint8_t)34, (uint8_t)114, (uint8_t)123, (uint8_t)47, (uint8_t)107, (uint8_t)191, (uint8_t)192, (uint8_t)11, (uint8_t)191, (uint8_t)210, (uint8_t)115, (uint8_t)198, (uint8_t)79, (uint8_t)34, (uint8_t)85, (uint8_t)253, (uint8_t)35, (uint8_t)224, (uint8_t)255, (uint8_t)150, (uint8_t)201, (uint8_t)137, (uint8_t)10, (uint8_t)145, (uint8_t)16} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)32651);
    assert(p124_lon_GET(pack) == (int32_t) -949333529);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)10132);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)21719);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)57083);
    assert(p124_lat_GET(pack) == (int32_t) -1576904728);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p124_dgps_age_GET(pack) == (uint32_t)2972374466L);
    assert(p124_time_usec_GET(pack) == (uint64_t)7422050247186353722L);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p124_alt_GET(pack) == (int32_t)598671089);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)12349);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)37582);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)90, (uint8_t)222, (uint8_t)194, (uint8_t)133, (uint8_t)240, (uint8_t)185, (uint8_t)205, (uint8_t)204, (uint8_t)153, (uint8_t)242, (uint8_t)117, (uint8_t)102, (uint8_t)134, (uint8_t)185, (uint8_t)40, (uint8_t)248, (uint8_t)135, (uint8_t)17, (uint8_t)90, (uint8_t)236, (uint8_t)53, (uint8_t)14, (uint8_t)86, (uint8_t)206, (uint8_t)206, (uint8_t)67, (uint8_t)148, (uint8_t)169, (uint8_t)27, (uint8_t)239, (uint8_t)131, (uint8_t)192, (uint8_t)214, (uint8_t)133, (uint8_t)101, (uint8_t)202, (uint8_t)8, (uint8_t)167, (uint8_t)159, (uint8_t)27, (uint8_t)101, (uint8_t)51, (uint8_t)76, (uint8_t)207, (uint8_t)23, (uint8_t)199, (uint8_t)195, (uint8_t)136, (uint8_t)225, (uint8_t)119, (uint8_t)66, (uint8_t)13, (uint8_t)182, (uint8_t)71, (uint8_t)150, (uint8_t)90, (uint8_t)250, (uint8_t)246, (uint8_t)23, (uint8_t)242, (uint8_t)45, (uint8_t)86, (uint8_t)170, (uint8_t)27, (uint8_t)53, (uint8_t)170, (uint8_t)134, (uint8_t)164, (uint8_t)1, (uint8_t)18} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)58887);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1);
    assert(p126_baudrate_GET(pack) == (uint32_t)1550268696L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1153436026);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -117076246);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)2188112759L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1525180840);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -2019317851);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p127_tow_GET(pack) == (uint32_t)1128995994L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p127_accuracy_GET(pack) == (uint32_t)2169457486L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)23376);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1881188428);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1576002052);
    assert(p128_tow_GET(pack) == (uint32_t)247034447L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)1321384928);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)743958766L);
    assert(p128_accuracy_GET(pack) == (uint32_t)2130017182L);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)40214);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1658909605);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -4341);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)9185);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)3127429087L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -11059);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)28385);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -29519);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)26781);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)14121);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)32600);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -20221);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)34223);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p130_size_GET(pack) == (uint32_t)2114239819L);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)53845);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)18726);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)115, (uint8_t)62, (uint8_t)105, (uint8_t)240, (uint8_t)244, (uint8_t)11, (uint8_t)167, (uint8_t)147, (uint8_t)22, (uint8_t)59, (uint8_t)226, (uint8_t)169, (uint8_t)45, (uint8_t)8, (uint8_t)207, (uint8_t)97, (uint8_t)67, (uint8_t)40, (uint8_t)128, (uint8_t)23, (uint8_t)145, (uint8_t)143, (uint8_t)49, (uint8_t)64, (uint8_t)191, (uint8_t)98, (uint8_t)65, (uint8_t)143, (uint8_t)179, (uint8_t)28, (uint8_t)25, (uint8_t)159, (uint8_t)232, (uint8_t)184, (uint8_t)178, (uint8_t)150, (uint8_t)9, (uint8_t)32, (uint8_t)186, (uint8_t)103, (uint8_t)191, (uint8_t)248, (uint8_t)18, (uint8_t)147, (uint8_t)123, (uint8_t)253, (uint8_t)233, (uint8_t)94, (uint8_t)254, (uint8_t)35, (uint8_t)61, (uint8_t)227, (uint8_t)23, (uint8_t)35, (uint8_t)161, (uint8_t)223, (uint8_t)47, (uint8_t)41, (uint8_t)69, (uint8_t)121, (uint8_t)201, (uint8_t)236, (uint8_t)229, (uint8_t)224, (uint8_t)73, (uint8_t)121, (uint8_t)179, (uint8_t)81, (uint8_t)181, (uint8_t)121, (uint8_t)192, (uint8_t)94, (uint8_t)69, (uint8_t)253, (uint8_t)205, (uint8_t)52, (uint8_t)114, (uint8_t)125, (uint8_t)180, (uint8_t)113, (uint8_t)139, (uint8_t)221, (uint8_t)106, (uint8_t)243, (uint8_t)115, (uint8_t)76, (uint8_t)163, (uint8_t)169, (uint8_t)192, (uint8_t)153, (uint8_t)151, (uint8_t)188, (uint8_t)119, (uint8_t)96, (uint8_t)130, (uint8_t)97, (uint8_t)234, (uint8_t)72, (uint8_t)235, (uint8_t)131, (uint8_t)98, (uint8_t)74, (uint8_t)15, (uint8_t)185, (uint8_t)37, (uint8_t)207, (uint8_t)59, (uint8_t)167, (uint8_t)207, (uint8_t)249, (uint8_t)0, (uint8_t)48, (uint8_t)10, (uint8_t)173, (uint8_t)205, (uint8_t)91, (uint8_t)115, (uint8_t)140, (uint8_t)18, (uint8_t)198, (uint8_t)165, (uint8_t)195, (uint8_t)83, (uint8_t)149, (uint8_t)93, (uint8_t)211, (uint8_t)43, (uint8_t)67, (uint8_t)232, (uint8_t)22, (uint8_t)199, (uint8_t)55, (uint8_t)230, (uint8_t)153, (uint8_t)193, (uint8_t)206, (uint8_t)77, (uint8_t)69, (uint8_t)239, (uint8_t)131, (uint8_t)50, (uint8_t)48, (uint8_t)215, (uint8_t)77, (uint8_t)234, (uint8_t)106, (uint8_t)152, (uint8_t)45, (uint8_t)207, (uint8_t)152, (uint8_t)118, (uint8_t)11, (uint8_t)135, (uint8_t)52, (uint8_t)206, (uint8_t)166, (uint8_t)66, (uint8_t)66, (uint8_t)207, (uint8_t)38, (uint8_t)224, (uint8_t)252, (uint8_t)253, (uint8_t)17, (uint8_t)193, (uint8_t)19, (uint8_t)58, (uint8_t)4, (uint8_t)115, (uint8_t)74, (uint8_t)33, (uint8_t)230, (uint8_t)221, (uint8_t)115, (uint8_t)86, (uint8_t)196, (uint8_t)76, (uint8_t)69, (uint8_t)52, (uint8_t)250, (uint8_t)57, (uint8_t)73, (uint8_t)100, (uint8_t)105, (uint8_t)33, (uint8_t)33, (uint8_t)235, (uint8_t)175, (uint8_t)50, (uint8_t)160, (uint8_t)245, (uint8_t)145, (uint8_t)40, (uint8_t)206, (uint8_t)249, (uint8_t)81, (uint8_t)120, (uint8_t)20, (uint8_t)22, (uint8_t)15, (uint8_t)198, (uint8_t)155, (uint8_t)70, (uint8_t)101, (uint8_t)240, (uint8_t)150, (uint8_t)246, (uint8_t)38, (uint8_t)196, (uint8_t)170, (uint8_t)157, (uint8_t)120, (uint8_t)159, (uint8_t)22, (uint8_t)8, (uint8_t)120, (uint8_t)204, (uint8_t)201, (uint8_t)157, (uint8_t)177, (uint8_t)176, (uint8_t)167, (uint8_t)144, (uint8_t)211, (uint8_t)230, (uint8_t)3, (uint8_t)129, (uint8_t)216, (uint8_t)208, (uint8_t)106, (uint8_t)147, (uint8_t)28, (uint8_t)27, (uint8_t)71, (uint8_t)138, (uint8_t)237, (uint8_t)164, (uint8_t)94, (uint8_t)131, (uint8_t)65, (uint8_t)28, (uint8_t)58, (uint8_t)27, (uint8_t)21, (uint8_t)194, (uint8_t)41, (uint8_t)47, (uint8_t)217, (uint8_t)205, (uint8_t)44, (uint8_t)222, (uint8_t)35} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)33152);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)50780);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_90);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)36934);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)80718004L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)45972);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)36);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)1790440619027252403L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)2657);
    assert(p133_lat_GET(pack) == (int32_t) -1496781748);
    assert(p133_lon_GET(pack) == (int32_t) -1595998616);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lon_GET(pack) == (int32_t)665220919);
    {
        int16_t exemplary[] =  {(int16_t)16577, (int16_t)1369, (int16_t) -21649, (int16_t) -11482, (int16_t) -12972, (int16_t)27129, (int16_t)9037, (int16_t) -28602, (int16_t) -28164, (int16_t)3084, (int16_t)27117, (int16_t)10776, (int16_t) -31708, (int16_t)19216, (int16_t) -10963, (int16_t)30771} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p134_lat_GET(pack) == (int32_t)874430662);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)24981);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t) -88084675);
    assert(p135_lon_GET(pack) == (int32_t)212413905);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)20228);
    assert(p136_lon_GET(pack) == (int32_t) -1524817939);
    assert(p136_current_height_GET(pack) == (float) -2.890101E38F);
    assert(p136_terrain_height_GET(pack) == (float) -2.7449623E38F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)27198);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)53079);
    assert(p136_lat_GET(pack) == (int32_t)1707727787);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)14312);
    assert(p137_press_diff_GET(pack) == (float)1.4807171E38F);
    assert(p137_press_abs_GET(pack) == (float)5.148311E37F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)2982233351L);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.3018991E38F, -2.8215404E38F, -1.0518461E38F, -5.5664257E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_time_usec_GET(pack) == (uint64_t)4742482759855228014L);
    assert(p138_y_GET(pack) == (float)1.6115177E38F);
    assert(p138_z_GET(pack) == (float)1.902488E38F);
    assert(p138_x_GET(pack) == (float)1.2202362E38F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)278930189502719008L);
    {
        float exemplary[] =  {-2.0828486E37F, -2.139906E38F, 1.1019805E37F, -2.8056986E38F, -2.7953426E36F, 2.4706206E38F, -1.4725866E37F, 1.5392862E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)197);
    {
        float exemplary[] =  {-2.5029583E38F, 2.7819063E38F, -3.1139555E38F, -3.2830358E38F, -9.493404E37F, 2.0154515E38F, -1.6140118E38F, 1.7444854E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)8481397963540108728L);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_amsl_GET(pack) == (float) -2.796283E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -3.199418E38F);
    assert(p141_altitude_local_GET(pack) == (float) -1.9326767E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.272274E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)5555626788292339978L);
    assert(p141_altitude_relative_GET(pack) == (float)2.84208E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)2.5904266E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)248);
    {
        uint8_t exemplary[] =  {(uint8_t)44, (uint8_t)36, (uint8_t)217, (uint8_t)208, (uint8_t)150, (uint8_t)120, (uint8_t)113, (uint8_t)86, (uint8_t)229, (uint8_t)53, (uint8_t)171, (uint8_t)127, (uint8_t)162, (uint8_t)34, (uint8_t)61, (uint8_t)89, (uint8_t)251, (uint8_t)110, (uint8_t)246, (uint8_t)225, (uint8_t)220, (uint8_t)0, (uint8_t)99, (uint8_t)235, (uint8_t)17, (uint8_t)3, (uint8_t)18, (uint8_t)195, (uint8_t)187, (uint8_t)91, (uint8_t)62, (uint8_t)145, (uint8_t)40, (uint8_t)113, (uint8_t)43, (uint8_t)130, (uint8_t)214, (uint8_t)173, (uint8_t)195, (uint8_t)99, (uint8_t)238, (uint8_t)112, (uint8_t)83, (uint8_t)13, (uint8_t)79, (uint8_t)193, (uint8_t)89, (uint8_t)99, (uint8_t)182, (uint8_t)9, (uint8_t)248, (uint8_t)98, (uint8_t)22, (uint8_t)153, (uint8_t)85, (uint8_t)108, (uint8_t)45, (uint8_t)191, (uint8_t)45, (uint8_t)128, (uint8_t)163, (uint8_t)93, (uint8_t)177, (uint8_t)126, (uint8_t)111, (uint8_t)21, (uint8_t)223, (uint8_t)143, (uint8_t)202, (uint8_t)39, (uint8_t)168, (uint8_t)240, (uint8_t)83, (uint8_t)94, (uint8_t)125, (uint8_t)115, (uint8_t)55, (uint8_t)102, (uint8_t)10, (uint8_t)184, (uint8_t)130, (uint8_t)252, (uint8_t)22, (uint8_t)66, (uint8_t)126, (uint8_t)82, (uint8_t)38, (uint8_t)32, (uint8_t)178, (uint8_t)10, (uint8_t)112, (uint8_t)220, (uint8_t)49, (uint8_t)221, (uint8_t)122, (uint8_t)57, (uint8_t)91, (uint8_t)254, (uint8_t)115, (uint8_t)205, (uint8_t)132, (uint8_t)151, (uint8_t)99, (uint8_t)34, (uint8_t)45, (uint8_t)65, (uint8_t)89, (uint8_t)198, (uint8_t)251, (uint8_t)214, (uint8_t)201, (uint8_t)95, (uint8_t)46, (uint8_t)241, (uint8_t)109, (uint8_t)194, (uint8_t)138, (uint8_t)133, (uint8_t)238, (uint8_t)108} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)76);
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)16, (uint8_t)219, (uint8_t)64, (uint8_t)112, (uint8_t)253, (uint8_t)170, (uint8_t)75, (uint8_t)173, (uint8_t)188, (uint8_t)184, (uint8_t)216, (uint8_t)100, (uint8_t)250, (uint8_t)230, (uint8_t)63, (uint8_t)4, (uint8_t)67, (uint8_t)136, (uint8_t)1, (uint8_t)225, (uint8_t)42, (uint8_t)39, (uint8_t)48, (uint8_t)127, (uint8_t)218, (uint8_t)68, (uint8_t)114, (uint8_t)181, (uint8_t)221, (uint8_t)23, (uint8_t)188, (uint8_t)158, (uint8_t)208, (uint8_t)204, (uint8_t)51, (uint8_t)34, (uint8_t)18, (uint8_t)171, (uint8_t)95, (uint8_t)93, (uint8_t)108, (uint8_t)78, (uint8_t)130, (uint8_t)64, (uint8_t)240, (uint8_t)35, (uint8_t)13, (uint8_t)65, (uint8_t)162, (uint8_t)123, (uint8_t)9, (uint8_t)26, (uint8_t)135, (uint8_t)37, (uint8_t)19, (uint8_t)180, (uint8_t)41, (uint8_t)79, (uint8_t)205, (uint8_t)82, (uint8_t)213, (uint8_t)169, (uint8_t)97, (uint8_t)137, (uint8_t)98, (uint8_t)128, (uint8_t)101, (uint8_t)130, (uint8_t)140, (uint8_t)138, (uint8_t)189, (uint8_t)100, (uint8_t)220, (uint8_t)238, (uint8_t)98, (uint8_t)233, (uint8_t)16, (uint8_t)201, (uint8_t)20, (uint8_t)88, (uint8_t)42, (uint8_t)222, (uint8_t)2, (uint8_t)252, (uint8_t)199, (uint8_t)92, (uint8_t)248, (uint8_t)204, (uint8_t)216, (uint8_t)122, (uint8_t)171, (uint8_t)143, (uint8_t)28, (uint8_t)146, (uint8_t)18, (uint8_t)214, (uint8_t)240, (uint8_t)157, (uint8_t)20, (uint8_t)231, (uint8_t)106, (uint8_t)156, (uint8_t)115, (uint8_t)172, (uint8_t)93, (uint8_t)172, (uint8_t)44, (uint8_t)61, (uint8_t)134, (uint8_t)38, (uint8_t)87, (uint8_t)253, (uint8_t)57, (uint8_t)82, (uint8_t)222, (uint8_t)26, (uint8_t)123, (uint8_t)13, (uint8_t)137} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_diff_GET(pack) == (float)2.5586268E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)4710);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3358482159L);
    assert(p143_press_abs_GET(pack) == (float) -1.1777193E38F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.7012186E38F, 2.9918455E38F, 1.6493914E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)6169813363336251809L);
    {
        float exemplary[] =  {-3.3127688E38F, -4.618932E37F, 7.0801645E36F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)869469894);
    {
        float exemplary[] =  {-6.325812E37F, -1.1832166E38F, -9.201623E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)311671921);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p144_custom_state_GET(pack) == (uint64_t)3645589240007241588L);
    {
        float exemplary[] =  {-1.0152855E38F, -2.8780494E38F, -1.7972888E38F, 3.1926598E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -4.1297017E37F);
    {
        float exemplary[] =  {-9.670178E37F, 2.8169148E38F, 1.0335474E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_airspeed_GET(pack) == (float)2.1851222E38F);
    assert(p146_x_acc_GET(pack) == (float) -2.9038028E38F);
    assert(p146_y_acc_GET(pack) == (float)2.2699251E38F);
    {
        float exemplary[] =  {-1.6868375E38F, -7.090539E37F, 3.155329E38F, -3.1603987E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float)1.8269802E38F);
    assert(p146_roll_rate_GET(pack) == (float)3.1901383E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -1.0881253E38F);
    assert(p146_z_pos_GET(pack) == (float)2.6698847E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)453514606673973763L);
    assert(p146_z_vel_GET(pack) == (float)2.2633814E38F);
    {
        float exemplary[] =  {2.3921287E38F, 1.2362515E38F, -1.2020262E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float)2.1469328E38F);
    assert(p146_x_pos_GET(pack) == (float) -2.7937612E38F);
    {
        float exemplary[] =  {-7.6388E37F, -3.2424253E38F, 1.7838286E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float) -7.5991886E37F);
    assert(p146_y_pos_GET(pack) == (float) -2.6433517E38F);
    assert(p146_x_vel_GET(pack) == (float)3.1959966E38F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)23387);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)116);
    assert(p147_current_consumed_GET(pack) == (int32_t) -1896168286);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -715949126);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)104);
    {
        uint16_t exemplary[] =  {(uint16_t)3773, (uint16_t)59578, (uint16_t)5277, (uint16_t)47280, (uint16_t)56908, (uint16_t)43586, (uint16_t)43013, (uint16_t)24813, (uint16_t)12749, (uint16_t)32197} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -4808);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2501623102L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3302490645L);
    {
        uint8_t exemplary[] =  {(uint8_t)118, (uint8_t)60, (uint8_t)26, (uint8_t)106, (uint8_t)224, (uint8_t)229, (uint8_t)8, (uint8_t)89} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)6714320671013403829L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)61240);
    {
        uint8_t exemplary[] =  {(uint8_t)194, (uint8_t)201, (uint8_t)154, (uint8_t)161, (uint8_t)117, (uint8_t)72, (uint8_t)213, (uint8_t)93, (uint8_t)135, (uint8_t)12, (uint8_t)101, (uint8_t)251, (uint8_t)72, (uint8_t)193, (uint8_t)47, (uint8_t)67, (uint8_t)247, (uint8_t)188} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3769520591L);
    {
        uint8_t exemplary[] =  {(uint8_t)197, (uint8_t)32, (uint8_t)66, (uint8_t)169, (uint8_t)232, (uint8_t)128, (uint8_t)155, (uint8_t)160} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)158, (uint8_t)245, (uint8_t)218, (uint8_t)44, (uint8_t)35, (uint8_t)82, (uint8_t)32, (uint8_t)2} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)21627);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP);
    assert(p148_board_version_GET(pack) == (uint32_t)823564179L);
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_x_TRY(ph) == (float)1.3726588E37F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)215);
    assert(p149_distance_GET(pack) == (float)2.577104E37F);
    {
        float exemplary[] =  {1.5537684E38F, -2.4062142E38F, -3.1071384E38F, -3.0903247E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p149_size_y_GET(pack) == (float)1.1373532E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)8221851501947560040L);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_z_TRY(ph) == (float)1.6351372E37F);
    assert(p149_angle_x_GET(pack) == (float)4.456888E37F);
    assert(p149_y_TRY(ph) == (float) -1.4828003E38F);
    assert(p149_angle_y_GET(pack) == (float) -1.2141077E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p149_size_x_GET(pack) == (float)1.795537E38F);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.3442267E38F);
    assert(p230_hagl_ratio_GET(pack) == (float)1.4364985E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -1.1789038E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -7.8796746E37F);
    assert(p230_mag_ratio_GET(pack) == (float)2.9867674E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -8.5555647E36F);
    assert(p230_tas_ratio_GET(pack) == (float)1.8966558E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)7132569226164688200L);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.6497106E38F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_vert_GET(pack) == (float) -2.3672093E38F);
    assert(p231_wind_x_GET(pack) == (float) -1.5034378E38F);
    assert(p231_wind_alt_GET(pack) == (float) -1.03935814E37F);
    assert(p231_wind_y_GET(pack) == (float)1.6453541E37F);
    assert(p231_time_usec_GET(pack) == (uint64_t)3575546442530607479L);
    assert(p231_wind_z_GET(pack) == (float) -2.8771703E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -1.8896423E38F);
    assert(p231_var_horiz_GET(pack) == (float)1.4777038E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)1.6430148E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_usec_GET(pack) == (uint64_t)7707018555425654356L);
    assert(p232_lat_GET(pack) == (int32_t)1722335177);
    assert(p232_lon_GET(pack) == (int32_t)365602560);
    assert(p232_hdop_GET(pack) == (float)2.6039464E38F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p232_horiz_accuracy_GET(pack) == (float)8.1026396E37F);
    assert(p232_vert_accuracy_GET(pack) == (float) -2.0663238E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)15404);
    assert(p232_vn_GET(pack) == (float) -7.175949E37F);
    assert(p232_ve_GET(pack) == (float)1.4387982E38F);
    assert(p232_speed_accuracy_GET(pack) == (float) -3.0455044E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)1879250457L);
    assert(p232_alt_GET(pack) == (float) -2.2203856E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP);
    assert(p232_vdop_GET(pack) == (float)1.8061296E37F);
    assert(p232_vd_GET(pack) == (float) -3.2947841E38F);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)9);
    {
        uint8_t exemplary[] =  {(uint8_t)119, (uint8_t)33, (uint8_t)37, (uint8_t)154, (uint8_t)210, (uint8_t)246, (uint8_t)168, (uint8_t)62, (uint8_t)226, (uint8_t)147, (uint8_t)48, (uint8_t)105, (uint8_t)157, (uint8_t)128, (uint8_t)165, (uint8_t)21, (uint8_t)93, (uint8_t)89, (uint8_t)94, (uint8_t)85, (uint8_t)116, (uint8_t)109, (uint8_t)242, (uint8_t)145, (uint8_t)207, (uint8_t)110, (uint8_t)102, (uint8_t)152, (uint8_t)106, (uint8_t)130, (uint8_t)11, (uint8_t)182, (uint8_t)226, (uint8_t)112, (uint8_t)214, (uint8_t)220, (uint8_t)171, (uint8_t)1, (uint8_t)233, (uint8_t)226, (uint8_t)255, (uint8_t)248, (uint8_t)23, (uint8_t)203, (uint8_t)127, (uint8_t)138, (uint8_t)244, (uint8_t)36, (uint8_t)222, (uint8_t)223, (uint8_t)35, (uint8_t)183, (uint8_t)90, (uint8_t)12, (uint8_t)81, (uint8_t)227, (uint8_t)166, (uint8_t)126, (uint8_t)226, (uint8_t)236, (uint8_t)213, (uint8_t)186, (uint8_t)143, (uint8_t)67, (uint8_t)164, (uint8_t)209, (uint8_t)32, (uint8_t)123, (uint8_t)9, (uint8_t)225, (uint8_t)169, (uint8_t)71, (uint8_t)93, (uint8_t)50, (uint8_t)38, (uint8_t)174, (uint8_t)172, (uint8_t)255, (uint8_t)66, (uint8_t)58, (uint8_t)220, (uint8_t)88, (uint8_t)189, (uint8_t)145, (uint8_t)37, (uint8_t)96, (uint8_t)147, (uint8_t)89, (uint8_t)125, (uint8_t)76, (uint8_t)208, (uint8_t)64, (uint8_t)162, (uint8_t)145, (uint8_t)193, (uint8_t)117, (uint8_t)186, (uint8_t)213, (uint8_t)107, (uint8_t)110, (uint8_t)68, (uint8_t)38, (uint8_t)143, (uint8_t)103, (uint8_t)164, (uint8_t)157, (uint8_t)105, (uint8_t)141, (uint8_t)145, (uint8_t)79, (uint8_t)31, (uint8_t)128, (uint8_t)234, (uint8_t)96, (uint8_t)142, (uint8_t)242, (uint8_t)17, (uint8_t)249, (uint8_t)244, (uint8_t)5, (uint8_t)207, (uint8_t)9, (uint8_t)169, (uint8_t)38, (uint8_t)32, (uint8_t)191, (uint8_t)157, (uint8_t)165, (uint8_t)39, (uint8_t)49, (uint8_t)29, (uint8_t)248, (uint8_t)197, (uint8_t)72, (uint8_t)225, (uint8_t)56, (uint8_t)154, (uint8_t)243, (uint8_t)41, (uint8_t)206, (uint8_t)178, (uint8_t)85, (uint8_t)113, (uint8_t)193, (uint8_t)121, (uint8_t)37, (uint8_t)84, (uint8_t)247, (uint8_t)187, (uint8_t)87, (uint8_t)40, (uint8_t)158, (uint8_t)129, (uint8_t)86, (uint8_t)26, (uint8_t)251, (uint8_t)133, (uint8_t)51, (uint8_t)213, (uint8_t)72, (uint8_t)66, (uint8_t)20, (uint8_t)223, (uint8_t)55, (uint8_t)181, (uint8_t)37, (uint8_t)187, (uint8_t)190, (uint8_t)55, (uint8_t)189, (uint8_t)28, (uint8_t)84, (uint8_t)244, (uint8_t)129, (uint8_t)116, (uint8_t)116, (uint8_t)50, (uint8_t)79, (uint8_t)109, (uint8_t)117} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1987214699L);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)57385);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)12841);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -7);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -22869);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -104);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p234_longitude_GET(pack) == (int32_t) -729839490);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)8446);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)41226);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)87);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)125);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)19998);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)2998);
    assert(p234_latitude_GET(pack) == (int32_t)1067234945);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_y_GET(pack) == (float) -1.1040916E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)6279131286665898101L);
    assert(p241_vibration_x_GET(pack) == (float)3.2088395E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)1414447387L);
    assert(p241_vibration_z_GET(pack) == (float) -2.7237839E37F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)650224710L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3842944251L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-6.0963335E37F, -8.482872E37F, 1.5134563E38F, 1.5783302E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t) -1942099713);
    assert(p242_approach_y_GET(pack) == (float)2.5455381E38F);
    assert(p242_y_GET(pack) == (float)2.0025014E38F);
    assert(p242_approach_z_GET(pack) == (float)2.248477E37F);
    assert(p242_x_GET(pack) == (float) -1.1923596E38F);
    assert(p242_approach_x_GET(pack) == (float)1.8525675E38F);
    assert(p242_altitude_GET(pack) == (int32_t) -1523001498);
    assert(p242_latitude_GET(pack) == (int32_t) -1903186662);
    assert(p242_time_usec_TRY(ph) == (uint64_t)4047951678379787651L);
    assert(p242_z_GET(pack) == (float) -1.1551764E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_x_GET(pack) == (float)1.6758945E38F);
    assert(p243_approach_z_GET(pack) == (float)9.967283E37F);
    assert(p243_z_GET(pack) == (float)1.5450746E38F);
    assert(p243_longitude_GET(pack) == (int32_t) -1143546080);
    assert(p243_y_GET(pack) == (float) -2.0730635E38F);
    assert(p243_approach_y_GET(pack) == (float) -5.3254316E37F);
    assert(p243_altitude_GET(pack) == (int32_t)84849262);
    assert(p243_time_usec_TRY(ph) == (uint64_t)5372670906738874843L);
    assert(p243_x_GET(pack) == (float)1.2117227E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)107);
    {
        float exemplary[] =  {2.2234397E38F, 1.3394776E38F, 2.1702168E38F, 2.167638E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_latitude_GET(pack) == (int32_t)809130388);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)54992);
    assert(p244_interval_us_GET(pack) == (int32_t) -1270423094);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_lat_GET(pack) == (int32_t)1746529933);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)5196);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)13730);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p246_lon_GET(pack) == (int32_t)962908736);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2515568583L);
    assert(p246_callsign_LEN(ph) == 7);
    {
        char16_t * exemplary = u"cpnvlui";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)44590);
    assert(p246_altitude_GET(pack) == (int32_t)1827827252);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -3921);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -3.0405348E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.8287462E38F);
    assert(p247_id_GET(pack) == (uint32_t)1611159779L);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.9198912E38F);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)53479);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)79);
    {
        uint8_t exemplary[] =  {(uint8_t)68, (uint8_t)161, (uint8_t)31, (uint8_t)182, (uint8_t)14, (uint8_t)134, (uint8_t)92, (uint8_t)46, (uint8_t)157, (uint8_t)144, (uint8_t)191, (uint8_t)6, (uint8_t)180, (uint8_t)223, (uint8_t)98, (uint8_t)17, (uint8_t)3, (uint8_t)192, (uint8_t)220, (uint8_t)109, (uint8_t)52, (uint8_t)71, (uint8_t)188, (uint8_t)192, (uint8_t)41, (uint8_t)77, (uint8_t)189, (uint8_t)53, (uint8_t)95, (uint8_t)137, (uint8_t)122, (uint8_t)104, (uint8_t)43, (uint8_t)0, (uint8_t)66, (uint8_t)12, (uint8_t)93, (uint8_t)234, (uint8_t)243, (uint8_t)147, (uint8_t)209, (uint8_t)202, (uint8_t)69, (uint8_t)191, (uint8_t)65, (uint8_t)74, (uint8_t)23, (uint8_t)4, (uint8_t)245, (uint8_t)124, (uint8_t)134, (uint8_t)157, (uint8_t)0, (uint8_t)26, (uint8_t)142, (uint8_t)232, (uint8_t)163, (uint8_t)107, (uint8_t)234, (uint8_t)89, (uint8_t)68, (uint8_t)59, (uint8_t)27, (uint8_t)43, (uint8_t)190, (uint8_t)156, (uint8_t)140, (uint8_t)101, (uint8_t)37, (uint8_t)42, (uint8_t)112, (uint8_t)23, (uint8_t)227, (uint8_t)31, (uint8_t)2, (uint8_t)166, (uint8_t)95, (uint8_t)79, (uint8_t)38, (uint8_t)164, (uint8_t)151, (uint8_t)168, (uint8_t)251, (uint8_t)246, (uint8_t)29, (uint8_t)9, (uint8_t)79, (uint8_t)119, (uint8_t)253, (uint8_t)165, (uint8_t)153, (uint8_t)95, (uint8_t)20, (uint8_t)43, (uint8_t)236, (uint8_t)225, (uint8_t)61, (uint8_t)234, (uint8_t)211, (uint8_t)62, (uint8_t)38, (uint8_t)233, (uint8_t)232, (uint8_t)112, (uint8_t)248, (uint8_t)217, (uint8_t)93, (uint8_t)208, (uint8_t)115, (uint8_t)40, (uint8_t)209, (uint8_t)180, (uint8_t)119, (uint8_t)10, (uint8_t)98, (uint8_t)182, (uint8_t)48, (uint8_t)212, (uint8_t)146, (uint8_t)154, (uint8_t)169, (uint8_t)229, (uint8_t)147, (uint8_t)91, (uint8_t)222, (uint8_t)192, (uint8_t)171, (uint8_t)68, (uint8_t)166, (uint8_t)10, (uint8_t)169, (uint8_t)37, (uint8_t)255, (uint8_t)223, (uint8_t)127, (uint8_t)4, (uint8_t)55, (uint8_t)232, (uint8_t)219, (uint8_t)29, (uint8_t)250, (uint8_t)142, (uint8_t)68, (uint8_t)156, (uint8_t)162, (uint8_t)31, (uint8_t)15, (uint8_t)95, (uint8_t)166, (uint8_t)44, (uint8_t)251, (uint8_t)248, (uint8_t)140, (uint8_t)92, (uint8_t)152, (uint8_t)146, (uint8_t)197, (uint8_t)75, (uint8_t)231, (uint8_t)244, (uint8_t)238, (uint8_t)115, (uint8_t)25, (uint8_t)12, (uint8_t)5, (uint8_t)19, (uint8_t)2, (uint8_t)224, (uint8_t)117, (uint8_t)254, (uint8_t)217, (uint8_t)16, (uint8_t)216, (uint8_t)113, (uint8_t)58, (uint8_t)103, (uint8_t)228, (uint8_t)130, (uint8_t)19, (uint8_t)12, (uint8_t)215, (uint8_t)146, (uint8_t)191, (uint8_t)249, (uint8_t)198, (uint8_t)214, (uint8_t)25, (uint8_t)52, (uint8_t)168, (uint8_t)227, (uint8_t)214, (uint8_t)133, (uint8_t)16, (uint8_t)215, (uint8_t)87, (uint8_t)65, (uint8_t)229, (uint8_t)194, (uint8_t)207, (uint8_t)159, (uint8_t)1, (uint8_t)193, (uint8_t)180, (uint8_t)255, (uint8_t)191, (uint8_t)162, (uint8_t)206, (uint8_t)87, (uint8_t)230, (uint8_t)60, (uint8_t)82, (uint8_t)74, (uint8_t)21, (uint8_t)61, (uint8_t)50, (uint8_t)65, (uint8_t)79, (uint8_t)112, (uint8_t)7, (uint8_t)103, (uint8_t)195, (uint8_t)244, (uint8_t)5, (uint8_t)237, (uint8_t)120, (uint8_t)125, (uint8_t)15, (uint8_t)20, (uint8_t)211, (uint8_t)33, (uint8_t)226, (uint8_t)186, (uint8_t)230, (uint8_t)156, (uint8_t)102, (uint8_t)113, (uint8_t)137, (uint8_t)62, (uint8_t)193, (uint8_t)159, (uint8_t)52, (uint8_t)184, (uint8_t)109, (uint8_t)199, (uint8_t)164, (uint8_t)64, (uint8_t)119, (uint8_t)92, (uint8_t)125} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)238);
    {
        int8_t exemplary[] =  {(int8_t) -32, (int8_t)117, (int8_t)19, (int8_t)118, (int8_t)91, (int8_t) -69, (int8_t) -95, (int8_t) -7, (int8_t)122, (int8_t)14, (int8_t) -115, (int8_t)89, (int8_t) -15, (int8_t)98, (int8_t)47, (int8_t)9, (int8_t)88, (int8_t)35, (int8_t) -16, (int8_t) -7, (int8_t)94, (int8_t) -95, (int8_t)85, (int8_t) -50, (int8_t) -115, (int8_t) -12, (int8_t) -10, (int8_t)48, (int8_t)42, (int8_t) -35, (int8_t)39, (int8_t)71} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)10853);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"hRRRSymfF";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_x_GET(pack) == (float)2.254717E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5134450796028266579L);
    assert(p250_y_GET(pack) == (float) -2.3732458E38F);
    assert(p250_z_GET(pack) == (float) -1.0718088E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -2.27537E38F);
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2056207963L);
    assert(p251_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"zekcxd";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -1634765001);
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)987372422L);
    assert(p252_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"dwXgniadcv";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 13);
    {
        char16_t * exemplary = u"Kzioglpguprub";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p254_value_GET(pack) == (float)2.33075E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3820264182L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)8077228205420765776L);
    {
        uint8_t exemplary[] =  {(uint8_t)243, (uint8_t)44, (uint8_t)43, (uint8_t)137, (uint8_t)52, (uint8_t)82, (uint8_t)244, (uint8_t)51, (uint8_t)199, (uint8_t)142, (uint8_t)50, (uint8_t)67, (uint8_t)118, (uint8_t)224, (uint8_t)157, (uint8_t)234, (uint8_t)161, (uint8_t)236, (uint8_t)129, (uint8_t)186, (uint8_t)122, (uint8_t)115, (uint8_t)0, (uint8_t)113, (uint8_t)66, (uint8_t)35, (uint8_t)108, (uint8_t)14, (uint8_t)215, (uint8_t)12, (uint8_t)240, (uint8_t)171} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)104);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1550240066L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2043659085L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p258_tune_LEN(ph) == 19);
    {
        char16_t * exemplary = u"cpakwreojucvfrvscbw";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
    assert(p259_firmware_version_GET(pack) == (uint32_t)355087958L);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)50029);
    assert(p259_cam_definition_uri_LEN(ph) == 59);
    {
        char16_t * exemplary = u"EjljcimefErzqgqahofefiorvbkznkulqsluyvbaQmyvcilevnTzyPdxmzF";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)20533);
    assert(p259_sensor_size_v_GET(pack) == (float) -2.5809105E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3824532545L);
    assert(p259_focal_length_GET(pack) == (float) -5.9014886E37F);
    {
        uint8_t exemplary[] =  {(uint8_t)216, (uint8_t)134, (uint8_t)39, (uint8_t)232, (uint8_t)133, (uint8_t)217, (uint8_t)117, (uint8_t)99, (uint8_t)207, (uint8_t)54, (uint8_t)208, (uint8_t)121, (uint8_t)214, (uint8_t)225, (uint8_t)44, (uint8_t)244, (uint8_t)196, (uint8_t)16, (uint8_t)183, (uint8_t)236, (uint8_t)106, (uint8_t)250, (uint8_t)12, (uint8_t)65, (uint8_t)147, (uint8_t)242, (uint8_t)230, (uint8_t)88, (uint8_t)160, (uint8_t)211, (uint8_t)170, (uint8_t)12} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)3829);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.8339965E38F);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)117);
    {
        uint8_t exemplary[] =  {(uint8_t)204, (uint8_t)202, (uint8_t)137, (uint8_t)226, (uint8_t)253, (uint8_t)163, (uint8_t)62, (uint8_t)166, (uint8_t)130, (uint8_t)19, (uint8_t)141, (uint8_t)233, (uint8_t)106, (uint8_t)52, (uint8_t)39, (uint8_t)18, (uint8_t)179, (uint8_t)77, (uint8_t)251, (uint8_t)251, (uint8_t)248, (uint8_t)19, (uint8_t)180, (uint8_t)155, (uint8_t)52, (uint8_t)168, (uint8_t)38, (uint8_t)20, (uint8_t)70, (uint8_t)97, (uint8_t)124, (uint8_t)85} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)826266580L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p261_used_capacity_GET(pack) == (float)1.9407658E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p261_read_speed_GET(pack) == (float)4.451694E36F);
    assert(p261_write_speed_GET(pack) == (float) -9.046608E37F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1708617092L);
    assert(p261_total_capacity_GET(pack) == (float) -2.8186118E38F);
    assert(p261_available_capacity_GET(pack) == (float) -1.0529648E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2588881319L);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3358054712L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p262_available_capacity_GET(pack) == (float) -3.2614072E38F);
    assert(p262_image_interval_GET(pack) == (float)2.3469458E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_relative_alt_GET(pack) == (int32_t)1486977049);
    assert(p263_image_index_GET(pack) == (int32_t)1263044855);
    assert(p263_time_utc_GET(pack) == (uint64_t)8446569823491706432L);
    {
        float exemplary[] =  {6.292579E35F, -3.2314499E38F, -1.5814466E38F, -2.4917622E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_file_url_LEN(ph) == 46);
    {
        char16_t * exemplary = u"oHFanvgdvmmwdnfXaycqfnvjpuhqalxFjkesfnwrvtdmed";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 92);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p263_lon_GET(pack) == (int32_t) -74153920);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)35);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2808535412L);
    assert(p263_lat_GET(pack) == (int32_t)404877262);
    assert(p263_alt_GET(pack) == (int32_t) -97698208);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)6828627005519425455L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)3671817226360538816L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3131938477L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)3412698755959984176L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float) -2.0267478E38F);
    assert(p265_yaw_GET(pack) == (float)3.0737586E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2863815506L);
    assert(p265_pitch_GET(pack) == (float)2.9501661E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)48);
    {
        uint8_t exemplary[] =  {(uint8_t)17, (uint8_t)252, (uint8_t)202, (uint8_t)67, (uint8_t)126, (uint8_t)30, (uint8_t)139, (uint8_t)15, (uint8_t)175, (uint8_t)113, (uint8_t)200, (uint8_t)135, (uint8_t)245, (uint8_t)154, (uint8_t)121, (uint8_t)97, (uint8_t)111, (uint8_t)130, (uint8_t)244, (uint8_t)71, (uint8_t)241, (uint8_t)143, (uint8_t)36, (uint8_t)189, (uint8_t)131, (uint8_t)197, (uint8_t)211, (uint8_t)20, (uint8_t)160, (uint8_t)204, (uint8_t)82, (uint8_t)130, (uint8_t)177, (uint8_t)101, (uint8_t)201, (uint8_t)164, (uint8_t)156, (uint8_t)21, (uint8_t)122, (uint8_t)29, (uint8_t)116, (uint8_t)16, (uint8_t)75, (uint8_t)201, (uint8_t)223, (uint8_t)185, (uint8_t)69, (uint8_t)72, (uint8_t)233, (uint8_t)10, (uint8_t)179, (uint8_t)55, (uint8_t)50, (uint8_t)70, (uint8_t)39, (uint8_t)24, (uint8_t)70, (uint8_t)68, (uint8_t)234, (uint8_t)176, (uint8_t)234, (uint8_t)132, (uint8_t)190, (uint8_t)57, (uint8_t)93, (uint8_t)32, (uint8_t)140, (uint8_t)92, (uint8_t)28, (uint8_t)167, (uint8_t)173, (uint8_t)177, (uint8_t)62, (uint8_t)22, (uint8_t)31, (uint8_t)45, (uint8_t)162, (uint8_t)174, (uint8_t)159, (uint8_t)102, (uint8_t)242, (uint8_t)68, (uint8_t)76, (uint8_t)54, (uint8_t)62, (uint8_t)142, (uint8_t)164, (uint8_t)56, (uint8_t)127, (uint8_t)36, (uint8_t)241, (uint8_t)237, (uint8_t)130, (uint8_t)243, (uint8_t)149, (uint8_t)2, (uint8_t)186, (uint8_t)224, (uint8_t)110, (uint8_t)225, (uint8_t)137, (uint8_t)73, (uint8_t)54, (uint8_t)117, (uint8_t)108, (uint8_t)239, (uint8_t)23, (uint8_t)216, (uint8_t)210, (uint8_t)23, (uint8_t)111, (uint8_t)22, (uint8_t)15, (uint8_t)210, (uint8_t)34, (uint8_t)201, (uint8_t)69, (uint8_t)44, (uint8_t)32, (uint8_t)50, (uint8_t)144, (uint8_t)167, (uint8_t)255, (uint8_t)32, (uint8_t)33, (uint8_t)162, (uint8_t)133, (uint8_t)2, (uint8_t)253, (uint8_t)44, (uint8_t)251, (uint8_t)255, (uint8_t)70, (uint8_t)32, (uint8_t)29, (uint8_t)173, (uint8_t)225, (uint8_t)150, (uint8_t)209, (uint8_t)68, (uint8_t)149, (uint8_t)217, (uint8_t)41, (uint8_t)249, (uint8_t)28, (uint8_t)82, (uint8_t)231, (uint8_t)208, (uint8_t)224, (uint8_t)25, (uint8_t)7, (uint8_t)228, (uint8_t)156, (uint8_t)69, (uint8_t)147, (uint8_t)72, (uint8_t)151, (uint8_t)225, (uint8_t)16, (uint8_t)197, (uint8_t)6, (uint8_t)59, (uint8_t)11, (uint8_t)57, (uint8_t)30, (uint8_t)20, (uint8_t)38, (uint8_t)159, (uint8_t)82, (uint8_t)227, (uint8_t)25, (uint8_t)74, (uint8_t)84, (uint8_t)137, (uint8_t)175, (uint8_t)235, (uint8_t)133, (uint8_t)116, (uint8_t)126, (uint8_t)166, (uint8_t)198, (uint8_t)168, (uint8_t)163, (uint8_t)222, (uint8_t)117, (uint8_t)243, (uint8_t)83, (uint8_t)95, (uint8_t)219, (uint8_t)15, (uint8_t)57, (uint8_t)101, (uint8_t)158, (uint8_t)72, (uint8_t)59, (uint8_t)92, (uint8_t)245, (uint8_t)188, (uint8_t)237, (uint8_t)121, (uint8_t)219, (uint8_t)116, (uint8_t)21, (uint8_t)25, (uint8_t)116, (uint8_t)35, (uint8_t)208, (uint8_t)1, (uint8_t)186, (uint8_t)7, (uint8_t)201, (uint8_t)27, (uint8_t)197, (uint8_t)197, (uint8_t)118, (uint8_t)238, (uint8_t)89, (uint8_t)227, (uint8_t)47, (uint8_t)220, (uint8_t)62, (uint8_t)248, (uint8_t)8, (uint8_t)51, (uint8_t)154, (uint8_t)84, (uint8_t)143, (uint8_t)75, (uint8_t)223, (uint8_t)0, (uint8_t)186, (uint8_t)58, (uint8_t)227, (uint8_t)202, (uint8_t)145, (uint8_t)150, (uint8_t)110, (uint8_t)40, (uint8_t)102, (uint8_t)65, (uint8_t)186, (uint8_t)74, (uint8_t)83, (uint8_t)4, (uint8_t)223, (uint8_t)201, (uint8_t)169, (uint8_t)30, (uint8_t)175} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)22158);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)191);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)151);
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)12, (uint8_t)224, (uint8_t)53, (uint8_t)255, (uint8_t)171, (uint8_t)49, (uint8_t)156, (uint8_t)127, (uint8_t)155, (uint8_t)233, (uint8_t)191, (uint8_t)49, (uint8_t)238, (uint8_t)75, (uint8_t)20, (uint8_t)162, (uint8_t)144, (uint8_t)34, (uint8_t)78, (uint8_t)110, (uint8_t)173, (uint8_t)132, (uint8_t)111, (uint8_t)62, (uint8_t)95, (uint8_t)169, (uint8_t)160, (uint8_t)124, (uint8_t)49, (uint8_t)103, (uint8_t)128, (uint8_t)240, (uint8_t)173, (uint8_t)253, (uint8_t)247, (uint8_t)135, (uint8_t)83, (uint8_t)85, (uint8_t)160, (uint8_t)230, (uint8_t)224, (uint8_t)193, (uint8_t)104, (uint8_t)95, (uint8_t)221, (uint8_t)57, (uint8_t)216, (uint8_t)9, (uint8_t)39, (uint8_t)176, (uint8_t)140, (uint8_t)94, (uint8_t)240, (uint8_t)169, (uint8_t)136, (uint8_t)211, (uint8_t)135, (uint8_t)200, (uint8_t)139, (uint8_t)13, (uint8_t)247, (uint8_t)236, (uint8_t)198, (uint8_t)229, (uint8_t)168, (uint8_t)215, (uint8_t)190, (uint8_t)172, (uint8_t)244, (uint8_t)128, (uint8_t)42, (uint8_t)217, (uint8_t)219, (uint8_t)35, (uint8_t)214, (uint8_t)126, (uint8_t)224, (uint8_t)51, (uint8_t)250, (uint8_t)109, (uint8_t)102, (uint8_t)97, (uint8_t)9, (uint8_t)45, (uint8_t)99, (uint8_t)141, (uint8_t)3, (uint8_t)101, (uint8_t)49, (uint8_t)113, (uint8_t)238, (uint8_t)101, (uint8_t)133, (uint8_t)105, (uint8_t)160, (uint8_t)68, (uint8_t)20, (uint8_t)194, (uint8_t)105, (uint8_t)19, (uint8_t)29, (uint8_t)224, (uint8_t)71, (uint8_t)115, (uint8_t)155, (uint8_t)154, (uint8_t)169, (uint8_t)146, (uint8_t)72, (uint8_t)71, (uint8_t)217, (uint8_t)46, (uint8_t)67, (uint8_t)104, (uint8_t)84, (uint8_t)102, (uint8_t)6, (uint8_t)64, (uint8_t)252, (uint8_t)98, (uint8_t)61, (uint8_t)224, (uint8_t)45, (uint8_t)144, (uint8_t)172, (uint8_t)229, (uint8_t)103, (uint8_t)150, (uint8_t)97, (uint8_t)74, (uint8_t)230, (uint8_t)215, (uint8_t)148, (uint8_t)84, (uint8_t)172, (uint8_t)118, (uint8_t)14, (uint8_t)209, (uint8_t)108, (uint8_t)52, (uint8_t)230, (uint8_t)18, (uint8_t)194, (uint8_t)253, (uint8_t)154, (uint8_t)26, (uint8_t)90, (uint8_t)48, (uint8_t)197, (uint8_t)113, (uint8_t)100, (uint8_t)30, (uint8_t)27, (uint8_t)219, (uint8_t)126, (uint8_t)14, (uint8_t)96, (uint8_t)0, (uint8_t)152, (uint8_t)239, (uint8_t)206, (uint8_t)201, (uint8_t)161, (uint8_t)200, (uint8_t)76, (uint8_t)84, (uint8_t)130, (uint8_t)201, (uint8_t)146, (uint8_t)205, (uint8_t)145, (uint8_t)244, (uint8_t)229, (uint8_t)210, (uint8_t)88, (uint8_t)119, (uint8_t)235, (uint8_t)225, (uint8_t)92, (uint8_t)65, (uint8_t)108, (uint8_t)255, (uint8_t)165, (uint8_t)102, (uint8_t)39, (uint8_t)241, (uint8_t)215, (uint8_t)44, (uint8_t)71, (uint8_t)110, (uint8_t)62, (uint8_t)179, (uint8_t)16, (uint8_t)113, (uint8_t)116, (uint8_t)144, (uint8_t)67, (uint8_t)167, (uint8_t)166, (uint8_t)3, (uint8_t)134, (uint8_t)177, (uint8_t)204, (uint8_t)249, (uint8_t)212, (uint8_t)223, (uint8_t)146, (uint8_t)128, (uint8_t)230, (uint8_t)41, (uint8_t)148, (uint8_t)162, (uint8_t)205, (uint8_t)37, (uint8_t)230, (uint8_t)108, (uint8_t)252, (uint8_t)221, (uint8_t)161, (uint8_t)204, (uint8_t)102, (uint8_t)197, (uint8_t)43, (uint8_t)175, (uint8_t)234, (uint8_t)182, (uint8_t)244, (uint8_t)193, (uint8_t)239, (uint8_t)108, (uint8_t)52, (uint8_t)39, (uint8_t)85, (uint8_t)255, (uint8_t)201, (uint8_t)30, (uint8_t)250, (uint8_t)107, (uint8_t)243, (uint8_t)103, (uint8_t)11, (uint8_t)97, (uint8_t)240, (uint8_t)245, (uint8_t)213, (uint8_t)187, (uint8_t)108, (uint8_t)81} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)2690);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)73);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)11446);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)112);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_bitrate_GET(pack) == (uint32_t)1589196946L);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)51696);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)41232);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)64128);
    assert(p269_uri_LEN(ph) == 31);
    {
        char16_t * exemplary = u"dtiodoosnvrhrYzealsnuxbdicFlhlg";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 62);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p269_framerate_GET(pack) == (float) -2.6357056E38F);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float) -1.6488021E38F);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)62745);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)56672);
    assert(p270_uri_LEN(ph) == 73);
    {
        char16_t * exemplary = u"LktxzejawkcmszmcfrsrtmhujnmafeFnnzaieifQsdhfzlkdfdtCSasiilcuPpbRcRNsCyFJr";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 146);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)328450502L);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)17688);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)90);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 62);
    {
        char16_t * exemplary = u"oRixcArijpkuqNvijwKgpunbdnybvzlccgfhiTwmdTgeaqjjfZcktsyjeeBfmo";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 124);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 1);
    {
        char16_t * exemplary = u"e";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)26866);
    {
        uint8_t exemplary[] =  {(uint8_t)237, (uint8_t)128, (uint8_t)165, (uint8_t)23, (uint8_t)29, (uint8_t)67, (uint8_t)230, (uint8_t)110} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)48596);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)38417);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)135, (uint8_t)177, (uint8_t)124, (uint8_t)250, (uint8_t)195, (uint8_t)229, (uint8_t)223} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)11118);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)3762083391L);
    assert(p310_time_usec_GET(pack) == (uint64_t)4241906575056261761L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)0);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p311_time_usec_GET(pack) == (uint64_t)528307447190856753L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3623846736L);
    assert(p311_name_LEN(ph) == 58);
    {
        char16_t * exemplary = u"jjlosewcuertvpzwumgtzgfipsjhilzzzqgyponhnqpovpxwhvspfygybn";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)20, (uint8_t)151, (uint8_t)87, (uint8_t)165, (uint8_t)170, (uint8_t)85, (uint8_t)211, (uint8_t)141, (uint8_t)5, (uint8_t)122, (uint8_t)151, (uint8_t)247, (uint8_t)45, (uint8_t)23, (uint8_t)147, (uint8_t)252} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2245230888L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)21214);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p320_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"Iwbjazmbwa";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)226);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 96);
    {
        char16_t * exemplary = u"zvxieakgrzFjgcekqucyZVnnmfyynlnrawkajeWthrFvybgWecwacmwmdeyPkzeebolumfxQtpgskcYoufzutTzYacadcHwr";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 192);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)6054);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)30930);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8);
    assert(p322_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"itN";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"syxgbsarse";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
    assert(p323_param_value_LEN(ph) == 8);
    {
        char16_t * exemplary = u"Jumpbvnu";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)102);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p324_param_value_LEN(ph) == 4);
    {
        char16_t * exemplary = u"iaqg";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"yoote";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_time_usec_GET(pack) == (uint64_t)2308428759319616585L);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)5234);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)29549);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)60);
    {
        uint16_t exemplary[] =  {(uint16_t)41724, (uint16_t)62548, (uint16_t)31352, (uint16_t)39312, (uint16_t)17716, (uint16_t)41540, (uint16_t)62234, (uint16_t)59911, (uint16_t)28916, (uint16_t)60223, (uint16_t)60274, (uint16_t)10668, (uint16_t)15506, (uint16_t)36269, (uint16_t)59939, (uint16_t)11822, (uint16_t)3025, (uint16_t)15918, (uint16_t)62608, (uint16_t)37643, (uint16_t)56129, (uint16_t)21078, (uint16_t)13382, (uint16_t)44372, (uint16_t)59670, (uint16_t)43419, (uint16_t)15171, (uint16_t)65450, (uint16_t)44516, (uint16_t)36770, (uint16_t)58703, (uint16_t)35312, (uint16_t)39262, (uint16_t)7804, (uint16_t)21901, (uint16_t)36160, (uint16_t)55794, (uint16_t)32315, (uint16_t)17505, (uint16_t)35149, (uint16_t)57892, (uint16_t)21451, (uint16_t)19543, (uint16_t)45440, (uint16_t)43031, (uint16_t)10579, (uint16_t)55578, (uint16_t)10565, (uint16_t)26001, (uint16_t)13878, (uint16_t)20362, (uint16_t)56013, (uint16_t)38741, (uint16_t)25986, (uint16_t)33429, (uint16_t)64576, (uint16_t)32768, (uint16_t)21516, (uint16_t)21032, (uint16_t)29211, (uint16_t)49396, (uint16_t)21785, (uint16_t)9052, (uint16_t)26606, (uint16_t)4340, (uint16_t)33487, (uint16_t)15703, (uint16_t)1753, (uint16_t)22059, (uint16_t)38344, (uint16_t)46215, (uint16_t)2097} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    assert(p10001_gpsOffsetLat_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M);
    assert(p10001_stallSpeed_GET(pack) == (uint16_t)(uint16_t)62979);
    assert(p10001_rfSelect_GET(pack) == e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
    assert(p10001_callsign_LEN(ph) == 6);
    {
        char16_t * exemplary = u"kytGun";
        char16_t * sample = p10001_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p10001_gpsOffsetLon_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
    assert(p10001_aircraftSize_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M);
    assert(p10001_emitterType_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE);
    assert(p10001_ICAO_GET(pack) == (uint32_t)3445797945L);
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    assert(p10002_accuracyVert_GET(pack) == (uint16_t)(uint16_t)7711);
    assert(p10002_accuracyVel_GET(pack) == (uint16_t)(uint16_t)55560);
    assert(p10002_accuracyHor_GET(pack) == (uint32_t)1632152107L);
    assert(p10002_velNS_GET(pack) == (int16_t)(int16_t) -15467);
    assert(p10002_gpsLon_GET(pack) == (int32_t)428171422);
    assert(p10002_squawk_GET(pack) == (uint16_t)(uint16_t)5601);
    assert(p10002_baroAltMSL_GET(pack) == (int32_t) -818346132);
    assert(p10002_utcTime_GET(pack) == (uint32_t)2241811018L);
    assert(p10002_gpsLat_GET(pack) == (int32_t) -803107212);
    assert(p10002_gpsAlt_GET(pack) == (int32_t)5921046);
    assert(p10002_emergencyStatus_GET(pack) == e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY);
    assert(p10002_gpsFix_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK);
    assert(p10002_state_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED);
    assert(p10002_velVert_GET(pack) == (int16_t)(int16_t) -12708);
    assert(p10002_VelEW_GET(pack) == (int16_t)(int16_t)28491);
    assert(p10002_numSats_GET(pack) == (uint8_t)(uint8_t)19);
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    assert(p10003_rfHealth_GET(pack) == e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK);
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
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2846872621L, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_HEXAROTOR, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count1_SET((uint16_t)(uint16_t)30758, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)4176, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)315, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)24566, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)12767, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)38161, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)32826, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -69, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)9943, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)29834, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)1075659526L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)4784959757485567515L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float)4.4696682E36F, PH.base.pack) ;
        p3_afy_SET((float)8.1050456E37F, PH.base.pack) ;
        p3_vy_SET((float)8.581586E37F, PH.base.pack) ;
        p3_yaw_SET((float) -4.1118952E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.5233748E38F, PH.base.pack) ;
        p3_afz_SET((float) -2.9859808E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)33017, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p3_x_SET((float) -1.2205249E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2849205793L, PH.base.pack) ;
        p3_y_SET((float)1.7919209E38F, PH.base.pack) ;
        p3_afx_SET((float) -1.100892E38F, PH.base.pack) ;
        p3_vx_SET((float) -2.2676996E38F, PH.base.pack) ;
        p3_z_SET((float) -9.140362E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p4_seq_SET((uint32_t)4171955146L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)9184937301410296783L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        {
            char16_t* passkey = u"WiFTddvjfxacmRkntpPzpgahx";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_gcs_system_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"znfdbecnopbz";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)1909983533L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t)20500, PH.base.pack) ;
        {
            char16_t* param_id = u"inhkhf";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"bgmrGwwepRad";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_index_SET((uint16_t)(uint16_t)57869, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)22917, PH.base.pack) ;
        p22_param_value_SET((float)1.4985009E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"yqxvadhlgcsnt";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p23_param_value_SET((float) -2.9887366E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_alt_ellipsoid_SET((int32_t)1025913492, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)37223, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)5670, PH.base.pack) ;
        p24_lat_SET((int32_t)2085981001, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)29235, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)2853813640L, &PH) ;
        p24_vel_acc_SET((uint32_t)708316385L, &PH) ;
        p24_h_acc_SET((uint32_t)180808995L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)4129936019L, &PH) ;
        p24_time_usec_SET((uint64_t)1302991863874669400L, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)17780, PH.base.pack) ;
        p24_alt_SET((int32_t)1302720578, PH.base.pack) ;
        p24_lon_SET((int32_t)106586940, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)43, (uint8_t)207, (uint8_t)235, (uint8_t)131, (uint8_t)6, (uint8_t)10, (uint8_t)21, (uint8_t)244, (uint8_t)228, (uint8_t)40, (uint8_t)195, (uint8_t)201, (uint8_t)77, (uint8_t)206, (uint8_t)142, (uint8_t)28, (uint8_t)203, (uint8_t)239, (uint8_t)73, (uint8_t)102};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)232, (uint8_t)55, (uint8_t)38, (uint8_t)95, (uint8_t)228, (uint8_t)74, (uint8_t)240, (uint8_t)83, (uint8_t)48, (uint8_t)100, (uint8_t)146, (uint8_t)27, (uint8_t)115, (uint8_t)140, (uint8_t)75, (uint8_t)215, (uint8_t)225, (uint8_t)238, (uint8_t)199, (uint8_t)251};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)179, (uint8_t)188, (uint8_t)181, (uint8_t)240, (uint8_t)137, (uint8_t)190, (uint8_t)195, (uint8_t)15, (uint8_t)9, (uint8_t)123, (uint8_t)226, (uint8_t)100, (uint8_t)169, (uint8_t)186, (uint8_t)202, (uint8_t)93, (uint8_t)224, (uint8_t)151, (uint8_t)213, (uint8_t)129};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)148, (uint8_t)150, (uint8_t)102, (uint8_t)247, (uint8_t)155, (uint8_t)135, (uint8_t)74, (uint8_t)96, (uint8_t)219, (uint8_t)204, (uint8_t)218, (uint8_t)91, (uint8_t)124, (uint8_t)189, (uint8_t)10, (uint8_t)134, (uint8_t)170, (uint8_t)142, (uint8_t)72, (uint8_t)254};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)115, (uint8_t)135, (uint8_t)96, (uint8_t)218, (uint8_t)51, (uint8_t)154, (uint8_t)207, (uint8_t)8, (uint8_t)156, (uint8_t)107, (uint8_t)235, (uint8_t)137, (uint8_t)198, (uint8_t)234, (uint8_t)69, (uint8_t)20, (uint8_t)75, (uint8_t)132, (uint8_t)16, (uint8_t)87};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_xgyro_SET((int16_t)(int16_t) -18339, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)19392, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -4430, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -10575, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)8172, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)30900, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -19837, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)26373, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -25768, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3283396644L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_xmag_SET((int16_t)(int16_t) -21070, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)8539, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -9528, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -8065, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)3607, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)3009493261919783439L, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)20834, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)11670, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -30307, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)15892, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t)26324, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)1081201902607922921L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -23166, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)13508, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -31281, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_time_boot_ms_SET((uint32_t)2351876590L, PH.base.pack) ;
        p29_press_diff_SET((float) -1.0771516E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)15038, PH.base.pack) ;
        p29_press_abs_SET((float)3.6982352E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)1.2689446E37F, PH.base.pack) ;
        p30_rollspeed_SET((float) -7.690485E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)4039989638L, PH.base.pack) ;
        p30_yawspeed_SET((float) -3.1675122E38F, PH.base.pack) ;
        p30_roll_SET((float) -2.9920722E38F, PH.base.pack) ;
        p30_pitch_SET((float)1.8053663E38F, PH.base.pack) ;
        p30_yaw_SET((float) -3.1590895E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_time_boot_ms_SET((uint32_t)1249481086L, PH.base.pack) ;
        p31_q2_SET((float) -2.7060572E38F, PH.base.pack) ;
        p31_q3_SET((float) -1.1834767E38F, PH.base.pack) ;
        p31_q1_SET((float)1.3108195E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -1.8682282E38F, PH.base.pack) ;
        p31_q4_SET((float) -1.5622091E38F, PH.base.pack) ;
        p31_rollspeed_SET((float)2.7053203E38F, PH.base.pack) ;
        p31_yawspeed_SET((float)1.0085906E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)2048823475L, PH.base.pack) ;
        p32_y_SET((float)2.6119474E38F, PH.base.pack) ;
        p32_vz_SET((float)2.617849E38F, PH.base.pack) ;
        p32_z_SET((float)2.1729438E37F, PH.base.pack) ;
        p32_vx_SET((float) -2.3827655E38F, PH.base.pack) ;
        p32_x_SET((float)2.8229315E38F, PH.base.pack) ;
        p32_vy_SET((float)1.6222978E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_alt_SET((int32_t)80931478, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)31556, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -5393, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)562960163L, PH.base.pack) ;
        p33_lat_SET((int32_t) -1505531515, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -20899, PH.base.pack) ;
        p33_lon_SET((int32_t) -830453906, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)48482744, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)14715, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan2_scaled_SET((int16_t)(int16_t) -29336, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)20910, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)8402, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)9406, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -22112, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -21670, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)7889, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)17096, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3821546188L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)10642, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)7534, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3071884236L, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)27927, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)52468, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)19512, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)11110, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)2255, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)62621, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo12_raw_SET((uint16_t)(uint16_t)764, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)10193, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)45989, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)17297, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)36503, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)13074, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)7914, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)29315, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)49939, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)3797162634L, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)64146, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)34716, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)50575, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)58369, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)19692, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)59702, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)39191, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_end_index_SET((int16_t)(int16_t)8566, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)19848, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -24076, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)31436, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_param2_SET((float)2.6977538E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p39_param1_SET((float)1.2820037E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_TAKEOFF, PH.base.pack) ;
        p39_x_SET((float)1.0684488E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)51226, PH.base.pack) ;
        p39_param4_SET((float)2.8304539E37F, PH.base.pack) ;
        p39_y_SET((float) -1.8020665E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_z_SET((float) -1.672688E38F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p39_param3_SET((float)1.1399684E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)60105, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)4123, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)36635, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)53587, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)1329, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_latitude_SET((int32_t)1995037183, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1025151751, PH.base.pack) ;
        p48_longitude_SET((int32_t) -1317611143, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)8338677883610571123L, &PH) ;
        p48_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)6460606635588797508L, &PH) ;
        p49_latitude_SET((int32_t)1991085064, PH.base.pack) ;
        p49_altitude_SET((int32_t) -282844977, PH.base.pack) ;
        p49_longitude_SET((int32_t) -15738783, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p50_param_value_min_SET((float) -2.6752198E38F, PH.base.pack) ;
        p50_param_value0_SET((float) -1.847334E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)3023, PH.base.pack) ;
        p50_scale_SET((float) -3.3366048E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"bs";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_max_SET((float) -2.5403398E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)11825, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p54_p1z_SET((float) -2.1910043E38F, PH.base.pack) ;
        p54_p1y_SET((float)3.2434236E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p54_p1x_SET((float) -8.579643E37F, PH.base.pack) ;
        p54_p2y_SET((float) -9.834256E37F, PH.base.pack) ;
        p54_p2z_SET((float)1.9626367E38F, PH.base.pack) ;
        p54_p2x_SET((float)1.882319E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2z_SET((float)1.3479748E38F, PH.base.pack) ;
        p55_p2x_SET((float) -2.5488403E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p55_p1y_SET((float)1.2456274E38F, PH.base.pack) ;
        p55_p2y_SET((float) -1.7949535E38F, PH.base.pack) ;
        p55_p1x_SET((float) -1.7283765E38F, PH.base.pack) ;
        p55_p1z_SET((float) -2.6329831E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        {
            float covariance[] =  {8.165272E37F, 3.0927054E38F, -2.2008067E38F, -3.9067437E37F, -1.4170093E38F, -1.346112E38F, 6.344775E37F, -2.3702724E38F, -1.4782414E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float) -2.4340182E37F, PH.base.pack) ;
        {
            float q[] =  {1.0621552E38F, 3.0290988E38F, 2.436592E38F, 5.0377413E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float) -2.696671E38F, PH.base.pack) ;
        p61_yawspeed_SET((float)3.1148378E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)4675482351010357898L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_aspd_error_SET((float)3.3461378E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -24338, PH.base.pack) ;
        p62_nav_pitch_SET((float)1.3883771E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -6.2034865E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.204979E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)37962, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)28099, PH.base.pack) ;
        p62_alt_error_SET((float) -2.5891293E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p63_alt_SET((int32_t)72903641, PH.base.pack) ;
        p63_vx_SET((float) -3.2289359E38F, PH.base.pack) ;
        {
            float covariance[] =  {2.9942266E38F, -1.2676215E38F, 2.0007615E38F, 9.554482E37F, 2.191364E38F, -1.5273871E38F, 2.94559E37F, -2.1359898E38F, 2.3401942E38F, 1.1799885E38F, 1.6316442E38F, -2.1682502E38F, -2.3230861E38F, -2.3886298E38F, -1.2376459E38F, 1.720023E38F, -1.1834975E38F, -2.6051906E37F, -2.055309E38F, 7.6769863E37F, -2.8145001E38F, 4.1323713E37F, 3.0285567E37F, 3.1631838E38F, 2.1605476E38F, 5.7672976E37F, -7.726237E37F, 1.6889639E38F, 2.6946877E38F, 1.592536E38F, 3.1202952E38F, 8.905509E37F, -3.3556614E38F, -3.6483116E37F, 2.4812873E38F, 1.1961792E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float)3.0441579E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)1287536392926164806L, PH.base.pack) ;
        p63_vy_SET((float) -3.2279201E38F, PH.base.pack) ;
        p63_lon_SET((int32_t) -1632462143, PH.base.pack) ;
        p63_lat_SET((int32_t)474236843, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1414163134, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)7253467739226297578L, PH.base.pack) ;
        {
            float covariance[] =  {-5.717297E37F, -1.2810899E38F, -2.459921E38F, -2.7537954E36F, 2.810831E38F, 2.131665E38F, 1.9206236E36F, -2.0909554E38F, 3.055756E36F, -6.3793127E37F, -1.7584476E37F, -1.668108E38F, -1.6850094E38F, 2.0916148E38F, -2.4743497E38F, 2.4811788E38F, 3.0421574E38F, 1.9162004E37F, 1.545504E38F, 1.2703041E38F, 2.4839425E38F, -1.0850406E37F, 1.5645537E38F, -2.1544168E38F, -6.177713E37F, -3.2730207E38F, 6.8880326E37F, -4.8085253E37F, 8.553787E37F, 1.0169862E38F, 1.6755359E38F, 2.8271085E38F, -2.1191072E38F, 1.4756658E38F, 1.676619E38F, -2.0598183E37F, -2.9134748E37F, -3.025896E38F, -1.3771971E38F, -2.4190211E38F, 1.8940404E38F, -3.6071272E37F, 1.154085E37F, 3.0751542E38F, -2.429455E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_vx_SET((float) -5.124944E37F, PH.base.pack) ;
        p64_z_SET((float) -1.8349934E38F, PH.base.pack) ;
        p64_ay_SET((float) -1.9531493E37F, PH.base.pack) ;
        p64_az_SET((float)5.3661876E37F, PH.base.pack) ;
        p64_vz_SET((float) -2.422461E38F, PH.base.pack) ;
        p64_vy_SET((float) -2.9586277E38F, PH.base.pack) ;
        p64_y_SET((float)1.1485664E38F, PH.base.pack) ;
        p64_ax_SET((float)7.0729797E37F, PH.base.pack) ;
        p64_x_SET((float) -1.5725996E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_rssi_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)50943, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)58149, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)34166, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)27409, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)54088, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)51220, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)43430, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)46471, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)23593, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)31721, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)57063, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)20426, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)50702, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)34005, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)874558232L, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)30374, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)12349, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)33072, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)11949, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_component_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)47301, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)64597, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_target_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -7526, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -24412, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -16539, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)38975, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -20692, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan7_raw_SET((uint16_t)(uint16_t)2313, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)14544, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)25765, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)37363, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)42564, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)49918, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)30187, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)54725, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)36015, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p73_param2_SET((float)2.1886637E38F, PH.base.pack) ;
        p73_param4_SET((float)1.3679726E38F, PH.base.pack) ;
        p73_param1_SET((float) -2.1044831E38F, PH.base.pack) ;
        p73_x_SET((int32_t) -1192387100, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p73_param3_SET((float)1.4755377E38F, PH.base.pack) ;
        p73_y_SET((int32_t)752618337, PH.base.pack) ;
        p73_z_SET((float)2.6817875E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float)1.8620475E38F, PH.base.pack) ;
        p74_climb_SET((float) -1.8870782E37F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -9154, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)25341, PH.base.pack) ;
        p74_airspeed_SET((float)3.7827148E37F, PH.base.pack) ;
        p74_groundspeed_SET((float)4.0363235E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_y_SET((int32_t) -1136960657, PH.base.pack) ;
        p75_param3_SET((float)2.5616286E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5, PH.base.pack) ;
        p75_param2_SET((float)1.4385889E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p75_z_SET((float) -3.0146338E38F, PH.base.pack) ;
        p75_param1_SET((float) -1.3971386E38F, PH.base.pack) ;
        p75_x_SET((int32_t)397943247, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p75_param4_SET((float) -6.847252E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param5_SET((float) -2.7794087E38F, PH.base.pack) ;
        p76_param7_SET((float) -3.3735247E38F, PH.base.pack) ;
        p76_param2_SET((float) -3.3003907E37F, PH.base.pack) ;
        p76_param6_SET((float)2.5125912E38F, PH.base.pack) ;
        p76_param1_SET((float)7.9311204E37F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, PH.base.pack) ;
        p76_param3_SET((float)2.2642313E38F, PH.base.pack) ;
        p76_param4_SET((float)2.9241515E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)195, &PH) ;
        p77_result_param2_SET((int32_t)1182527019, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)77, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)248, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p81_thrust_SET((float)1.033224E38F, PH.base.pack) ;
        p81_yaw_SET((float) -1.3941062E38F, PH.base.pack) ;
        p81_roll_SET((float)7.6374235E37F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2969319959L, PH.base.pack) ;
        p81_pitch_SET((float) -2.4076883E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_pitch_rate_SET((float)1.2590558E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        {
            float q[] =  {2.7069113E38F, -2.2669607E38F, 3.1499022E38F, 8.0021565E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float) -6.422117E37F, PH.base.pack) ;
        p82_thrust_SET((float) -4.7566374E36F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)188274701L, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -2.7712706E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        {
            float q[] =  {-3.2788339E38F, 1.3004757E38F, -2.2170625E38F, 8.753965E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p83_body_pitch_rate_SET((float) -2.5176433E38F, PH.base.pack) ;
        p83_thrust_SET((float) -1.1036724E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)955469886L, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -8.541115E37F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)2.7926404E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_z_SET((float)2.5513117E38F, PH.base.pack) ;
        p84_yaw_SET((float) -1.4218246E38F, PH.base.pack) ;
        p84_vy_SET((float)2.9379032E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -2.3743722E38F, PH.base.pack) ;
        p84_vx_SET((float)2.9856378E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)76500234L, PH.base.pack) ;
        p84_y_SET((float) -2.1280946E37F, PH.base.pack) ;
        p84_x_SET((float) -9.961106E37F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p84_vz_SET((float) -1.1032094E38F, PH.base.pack) ;
        p84_afx_SET((float)6.714006E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)49748, PH.base.pack) ;
        p84_afz_SET((float)3.1849688E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p84_afy_SET((float)1.1183315E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_vx_SET((float) -2.8735469E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)31058, PH.base.pack) ;
        p86_yaw_rate_SET((float) -1.1078116E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1475867935, PH.base.pack) ;
        p86_afz_SET((float)3.288963E38F, PH.base.pack) ;
        p86_alt_SET((float) -1.4217686E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p86_afx_SET((float) -1.971117E38F, PH.base.pack) ;
        p86_vz_SET((float)9.854613E37F, PH.base.pack) ;
        p86_yaw_SET((float) -1.4781159E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1767351976, PH.base.pack) ;
        p86_afy_SET((float) -4.6877075E37F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)217912311L, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p86_vy_SET((float)2.0595909E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vy_SET((float)9.99504E37F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1073302634, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)3844022306L, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -964071520, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.7711387E38F, PH.base.pack) ;
        p87_afy_SET((float)2.5270262E38F, PH.base.pack) ;
        p87_alt_SET((float)2.8615237E38F, PH.base.pack) ;
        p87_vx_SET((float) -5.2810534E35F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)63742, PH.base.pack) ;
        p87_yaw_SET((float) -2.2203101E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p87_afx_SET((float) -2.3531459E38F, PH.base.pack) ;
        p87_vz_SET((float) -1.1625726E38F, PH.base.pack) ;
        p87_afz_SET((float) -1.9494164E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_pitch_SET((float) -3.1900529E38F, PH.base.pack) ;
        p89_y_SET((float) -1.0811489E38F, PH.base.pack) ;
        p89_roll_SET((float) -3.2488635E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)3292092632L, PH.base.pack) ;
        p89_x_SET((float) -2.4427605E38F, PH.base.pack) ;
        p89_yaw_SET((float)2.706544E38F, PH.base.pack) ;
        p89_z_SET((float)1.0395975E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_zacc_SET((int16_t)(int16_t)15968, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)7497671567642414074L, PH.base.pack) ;
        p90_rollspeed_SET((float)8.3409746E37F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -10612, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -23750, PH.base.pack) ;
        p90_lat_SET((int32_t) -1129176904, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -18500, PH.base.pack) ;
        p90_roll_SET((float)3.1373184E38F, PH.base.pack) ;
        p90_lon_SET((int32_t) -890497869, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.7553162E38F, PH.base.pack) ;
        p90_yaw_SET((float)8.62847E37F, PH.base.pack) ;
        p90_alt_SET((int32_t)1858100650, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)1183, PH.base.pack) ;
        p90_yawspeed_SET((float)1.7386274E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -14976, PH.base.pack) ;
        p90_pitch_SET((float) -3.2701412E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux1_SET((float) -1.2274248E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.0673766E37F, PH.base.pack) ;
        p91_throttle_SET((float)2.4743282E38F, PH.base.pack) ;
        p91_aux2_SET((float)1.9018047E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)3.0146788E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -1.3848734E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)1552290244768926648L, PH.base.pack) ;
        p91_aux4_SET((float) -2.9718275E38F, PH.base.pack) ;
        p91_aux3_SET((float)3.0611118E35F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan8_raw_SET((uint16_t)(uint16_t)4432, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)20679, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)38488, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)30011, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)54070, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)29756, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)41935, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)42221, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)64059, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)4145, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)58821, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)412064918094905856L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)62218, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)3135938441777206533L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)4426328200019213071L, PH.base.pack) ;
        {
            float controls[] =  {-1.5533956E38F, 1.9830187E38F, -2.429995E38F, 5.6048664E37F, -7.39881E37F, -5.9986043E37F, -2.3820433E38F, -1.7342282E38F, 2.8929196E38F, 1.7221893E38F, -2.8397059E38F, -2.0028443E38F, -1.3416745E38F, -3.0867355E38F, 1.8827463E38F, 4.0581532E37F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_rate_x_SET((float) -2.0314075E38F, &PH) ;
        p100_flow_x_SET((int16_t)(int16_t) -2554, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -6719, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)1317262055054710568L, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p100_ground_distance_SET((float) -3.818688E37F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -1.1332797E38F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -8.8852724E36F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)3.1196204E38F, &PH) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_x_SET((float)3.2970036E38F, PH.base.pack) ;
        p101_yaw_SET((float)2.1679953E38F, PH.base.pack) ;
        p101_roll_SET((float)2.9305695E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)6476344392136675534L, PH.base.pack) ;
        p101_y_SET((float) -1.3543031E38F, PH.base.pack) ;
        p101_pitch_SET((float) -1.6935885E38F, PH.base.pack) ;
        p101_z_SET((float) -2.5197444E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_pitch_SET((float) -5.3576294E37F, PH.base.pack) ;
        p102_x_SET((float)8.964018E37F, PH.base.pack) ;
        p102_y_SET((float)9.452749E37F, PH.base.pack) ;
        p102_roll_SET((float)4.315045E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)4638222224801586001L, PH.base.pack) ;
        p102_z_SET((float)1.7099578E38F, PH.base.pack) ;
        p102_yaw_SET((float) -3.2377847E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float)1.2931913E38F, PH.base.pack) ;
        p103_y_SET((float)2.8554413E36F, PH.base.pack) ;
        p103_z_SET((float) -5.3343604E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)8438729123838443953L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_yaw_SET((float)1.8242476E38F, PH.base.pack) ;
        p104_roll_SET((float) -1.0812915E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)3059256821134229087L, PH.base.pack) ;
        p104_pitch_SET((float) -5.831651E37F, PH.base.pack) ;
        p104_y_SET((float) -2.0415018E38F, PH.base.pack) ;
        p104_z_SET((float)2.1864047E37F, PH.base.pack) ;
        p104_x_SET((float)2.5389275E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_xacc_SET((float)2.289248E38F, PH.base.pack) ;
        p105_ymag_SET((float) -1.7502079E38F, PH.base.pack) ;
        p105_ygyro_SET((float)2.0541045E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)2.6658236E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8745238437357896093L, PH.base.pack) ;
        p105_zacc_SET((float)9.116989E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)3963, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.0692252E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.0806922E37F, PH.base.pack) ;
        p105_yacc_SET((float) -2.0078577E38F, PH.base.pack) ;
        p105_xgyro_SET((float) -1.9783618E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.2357757E38F, PH.base.pack) ;
        p105_xmag_SET((float) -3.257899E38F, PH.base.pack) ;
        p105_temperature_SET((float)3.0856023E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.2355635E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_ygyro_SET((float)3.3041473E37F, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.057138E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)24090, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1939953679L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p106_distance_SET((float) -1.5039352E37F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)1722980125L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)1.1066599E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)3510015996464041946L, PH.base.pack) ;
        p106_integrated_y_SET((float)1.7496268E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -9.1160925E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_ygyro_SET((float) -2.3266449E38F, PH.base.pack) ;
        p107_temperature_SET((float)4.068874E36F, PH.base.pack) ;
        p107_xmag_SET((float)2.5724967E38F, PH.base.pack) ;
        p107_zmag_SET((float) -2.8535174E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)7.1292097E37F, PH.base.pack) ;
        p107_xgyro_SET((float)2.1570616E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -2.3436265E38F, PH.base.pack) ;
        p107_yacc_SET((float)2.8312613E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -3.3569491E38F, PH.base.pack) ;
        p107_xacc_SET((float) -4.529736E36F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)857491189792234929L, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)3128645984L, PH.base.pack) ;
        p107_zgyro_SET((float)8.606418E37F, PH.base.pack) ;
        p107_zacc_SET((float) -2.059394E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.7712556E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_roll_SET((float)1.4329623E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.0773022E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.9004842E38F, PH.base.pack) ;
        p108_q1_SET((float)1.3534657E38F, PH.base.pack) ;
        p108_zgyro_SET((float)1.8721875E38F, PH.base.pack) ;
        p108_vd_SET((float)2.5978625E38F, PH.base.pack) ;
        p108_yaw_SET((float)9.253704E37F, PH.base.pack) ;
        p108_lon_SET((float) -8.897602E37F, PH.base.pack) ;
        p108_yacc_SET((float) -2.8748974E37F, PH.base.pack) ;
        p108_q3_SET((float)3.1104574E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -4.502821E37F, PH.base.pack) ;
        p108_alt_SET((float)7.2477533E37F, PH.base.pack) ;
        p108_q4_SET((float)2.6790418E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)1.0239214E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.3081778E37F, PH.base.pack) ;
        p108_zacc_SET((float)8.582227E36F, PH.base.pack) ;
        p108_ve_SET((float)3.1097086E38F, PH.base.pack) ;
        p108_lat_SET((float)2.7417903E38F, PH.base.pack) ;
        p108_q2_SET((float)3.9042437E37F, PH.base.pack) ;
        p108_ygyro_SET((float)5.3953223E37F, PH.base.pack) ;
        p108_vn_SET((float) -2.3093105E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_fixed__SET((uint16_t)(uint16_t)28695, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)22239, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)51, (uint8_t)232, (uint8_t)154, (uint8_t)130, (uint8_t)17, (uint8_t)92, (uint8_t)157, (uint8_t)224, (uint8_t)40, (uint8_t)84, (uint8_t)148, (uint8_t)158, (uint8_t)102, (uint8_t)77, (uint8_t)96, (uint8_t)118, (uint8_t)156, (uint8_t)76, (uint8_t)116, (uint8_t)118, (uint8_t)214, (uint8_t)218, (uint8_t)37, (uint8_t)61, (uint8_t)174, (uint8_t)156, (uint8_t)115, (uint8_t)131, (uint8_t)104, (uint8_t)95, (uint8_t)169, (uint8_t)83, (uint8_t)244, (uint8_t)49, (uint8_t)212, (uint8_t)228, (uint8_t)253, (uint8_t)95, (uint8_t)79, (uint8_t)88, (uint8_t)35, (uint8_t)172, (uint8_t)7, (uint8_t)106, (uint8_t)3, (uint8_t)210, (uint8_t)11, (uint8_t)55, (uint8_t)96, (uint8_t)211, (uint8_t)176, (uint8_t)11, (uint8_t)229, (uint8_t)109, (uint8_t)201, (uint8_t)64, (uint8_t)7, (uint8_t)192, (uint8_t)47, (uint8_t)168, (uint8_t)63, (uint8_t)93, (uint8_t)156, (uint8_t)251, (uint8_t)182, (uint8_t)181, (uint8_t)45, (uint8_t)143, (uint8_t)206, (uint8_t)152, (uint8_t)7, (uint8_t)211, (uint8_t)25, (uint8_t)218, (uint8_t)248, (uint8_t)63, (uint8_t)125, (uint8_t)151, (uint8_t)162, (uint8_t)160, (uint8_t)150, (uint8_t)103, (uint8_t)96, (uint8_t)135, (uint8_t)29, (uint8_t)242, (uint8_t)97, (uint8_t)56, (uint8_t)125, (uint8_t)170, (uint8_t)177, (uint8_t)164, (uint8_t)226, (uint8_t)26, (uint8_t)39, (uint8_t)212, (uint8_t)15, (uint8_t)95, (uint8_t)136, (uint8_t)97, (uint8_t)131, (uint8_t)188, (uint8_t)79, (uint8_t)117, (uint8_t)54, (uint8_t)80, (uint8_t)238, (uint8_t)253, (uint8_t)227, (uint8_t)217, (uint8_t)92, (uint8_t)161, (uint8_t)37, (uint8_t)14, (uint8_t)136, (uint8_t)175, (uint8_t)177, (uint8_t)220, (uint8_t)235, (uint8_t)38, (uint8_t)173, (uint8_t)161, (uint8_t)24, (uint8_t)246, (uint8_t)117, (uint8_t)8, (uint8_t)223, (uint8_t)102, (uint8_t)201, (uint8_t)1, (uint8_t)80, (uint8_t)31, (uint8_t)252, (uint8_t)201, (uint8_t)206, (uint8_t)46, (uint8_t)60, (uint8_t)27, (uint8_t)28, (uint8_t)45, (uint8_t)126, (uint8_t)100, (uint8_t)68, (uint8_t)241, (uint8_t)196, (uint8_t)171, (uint8_t)74, (uint8_t)11, (uint8_t)39, (uint8_t)245, (uint8_t)173, (uint8_t)11, (uint8_t)146, (uint8_t)56, (uint8_t)238, (uint8_t)100, (uint8_t)97, (uint8_t)107, (uint8_t)160, (uint8_t)191, (uint8_t)128, (uint8_t)14, (uint8_t)220, (uint8_t)65, (uint8_t)124, (uint8_t)152, (uint8_t)146, (uint8_t)172, (uint8_t)240, (uint8_t)226, (uint8_t)85, (uint8_t)183, (uint8_t)17, (uint8_t)35, (uint8_t)88, (uint8_t)55, (uint8_t)205, (uint8_t)192, (uint8_t)63, (uint8_t)110, (uint8_t)235, (uint8_t)255, (uint8_t)31, (uint8_t)139, (uint8_t)205, (uint8_t)40, (uint8_t)6, (uint8_t)35, (uint8_t)65, (uint8_t)217, (uint8_t)129, (uint8_t)10, (uint8_t)172, (uint8_t)174, (uint8_t)84, (uint8_t)225, (uint8_t)241, (uint8_t)6, (uint8_t)95, (uint8_t)106, (uint8_t)81, (uint8_t)247, (uint8_t)198, (uint8_t)164, (uint8_t)237, (uint8_t)121, (uint8_t)173, (uint8_t)212, (uint8_t)233, (uint8_t)30, (uint8_t)118, (uint8_t)160, (uint8_t)31, (uint8_t)73, (uint8_t)31, (uint8_t)149, (uint8_t)14, (uint8_t)248, (uint8_t)58, (uint8_t)124, (uint8_t)106, (uint8_t)133, (uint8_t)164, (uint8_t)189, (uint8_t)148, (uint8_t)154, (uint8_t)246, (uint8_t)127, (uint8_t)126, (uint8_t)65, (uint8_t)142, (uint8_t)75, (uint8_t)151, (uint8_t)14, (uint8_t)154, (uint8_t)28, (uint8_t)28, (uint8_t)239, (uint8_t)235, (uint8_t)37, (uint8_t)17, (uint8_t)133, (uint8_t)140, (uint8_t)77, (uint8_t)104, (uint8_t)161, (uint8_t)219, (uint8_t)115, (uint8_t)209, (uint8_t)62, (uint8_t)206};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)4029272846493900268L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -7382875968158081303L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)3743966414L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)2846838210756728428L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_time_usec_SET((uint64_t)7728435531944599256L, PH.base.pack) ;
        p113_lon_SET((int32_t) -1400140437, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -22595, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)24730, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)36402, PH.base.pack) ;
        p113_lat_SET((int32_t)1121042780, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)20449, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -8336, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)15119, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)17074, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p113_alt_SET((int32_t)1007628643, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_temperature_SET((int16_t)(int16_t) -15623, PH.base.pack) ;
        p114_distance_SET((float)2.798916E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)3.3415677E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.5710275E38F, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.9672211E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.7592193E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)9130195839528726736L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.0729588E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4015842988L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3067354825L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_yawspeed_SET((float)2.2835967E38F, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)5988, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -19949, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)32275, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)9556, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)18700, PH.base.pack) ;
        p115_rollspeed_SET((float)7.899727E37F, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -8702, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -31567, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)9045377146671676229L, PH.base.pack) ;
        p115_lat_SET((int32_t) -2015383438, PH.base.pack) ;
        p115_alt_SET((int32_t) -534474673, PH.base.pack) ;
        p115_lon_SET((int32_t) -443038763, PH.base.pack) ;
        p115_pitchspeed_SET((float)4.555618E37F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -15754, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-3.944703E37F, 3.325525E38F, -1.5990773E38F, -2.5448662E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_xacc_SET((int16_t)(int16_t)22577, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -19061, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)6281, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)31260, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -16566, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -18188, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -31689, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2576133825L, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)32420, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -18965, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)289, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)317, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_num_logs_SET((uint16_t)(uint16_t)841, PH.base.pack) ;
        p118_size_SET((uint32_t)4229396479L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)53364, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)8735, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)3042105300L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)35008, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1266636614L, PH.base.pack) ;
        p119_count_SET((uint32_t)4053127005L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)59679, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)59, (uint8_t)225, (uint8_t)34, (uint8_t)139, (uint8_t)116, (uint8_t)217, (uint8_t)174, (uint8_t)22, (uint8_t)184, (uint8_t)71, (uint8_t)63, (uint8_t)123, (uint8_t)89, (uint8_t)250, (uint8_t)226, (uint8_t)72, (uint8_t)113, (uint8_t)137, (uint8_t)135, (uint8_t)47, (uint8_t)236, (uint8_t)94, (uint8_t)11, (uint8_t)82, (uint8_t)185, (uint8_t)109, (uint8_t)83, (uint8_t)223, (uint8_t)133, (uint8_t)159, (uint8_t)224, (uint8_t)13, (uint8_t)62, (uint8_t)208, (uint8_t)69, (uint8_t)100, (uint8_t)210, (uint8_t)241, (uint8_t)179, (uint8_t)202, (uint8_t)57, (uint8_t)169, (uint8_t)174, (uint8_t)133, (uint8_t)206, (uint8_t)212, (uint8_t)125, (uint8_t)232, (uint8_t)39, (uint8_t)137, (uint8_t)163, (uint8_t)99, (uint8_t)239, (uint8_t)105, (uint8_t)179, (uint8_t)25, (uint8_t)135, (uint8_t)223, (uint8_t)57, (uint8_t)99, (uint8_t)17, (uint8_t)126, (uint8_t)23, (uint8_t)140, (uint8_t)28, (uint8_t)136, (uint8_t)129, (uint8_t)236, (uint8_t)191, (uint8_t)86, (uint8_t)241, (uint8_t)53, (uint8_t)14, (uint8_t)54, (uint8_t)157, (uint8_t)99, (uint8_t)205, (uint8_t)14, (uint8_t)50, (uint8_t)252, (uint8_t)121, (uint8_t)250, (uint8_t)240, (uint8_t)167, (uint8_t)121, (uint8_t)218, (uint8_t)98, (uint8_t)194, (uint8_t)167, (uint8_t)178};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)1085411480L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)172, (uint8_t)148, (uint8_t)59, (uint8_t)30, (uint8_t)246, (uint8_t)21, (uint8_t)138, (uint8_t)99, (uint8_t)106, (uint8_t)178, (uint8_t)34, (uint8_t)141, (uint8_t)41, (uint8_t)245, (uint8_t)244, (uint8_t)6, (uint8_t)168, (uint8_t)159, (uint8_t)116, (uint8_t)63, (uint8_t)86, (uint8_t)232, (uint8_t)59, (uint8_t)0, (uint8_t)118, (uint8_t)191, (uint8_t)115, (uint8_t)47, (uint8_t)230, (uint8_t)231, (uint8_t)160, (uint8_t)31, (uint8_t)88, (uint8_t)151, (uint8_t)105, (uint8_t)33, (uint8_t)92, (uint8_t)98, (uint8_t)205, (uint8_t)228, (uint8_t)203, (uint8_t)52, (uint8_t)204, (uint8_t)125, (uint8_t)96, (uint8_t)220, (uint8_t)211, (uint8_t)253, (uint8_t)180, (uint8_t)28, (uint8_t)43, (uint8_t)167, (uint8_t)82, (uint8_t)202, (uint8_t)103, (uint8_t)3, (uint8_t)91, (uint8_t)211, (uint8_t)36, (uint8_t)105, (uint8_t)32, (uint8_t)17, (uint8_t)101, (uint8_t)111, (uint8_t)183, (uint8_t)189, (uint8_t)22, (uint8_t)36, (uint8_t)157, (uint8_t)119, (uint8_t)101, (uint8_t)175, (uint8_t)75, (uint8_t)17, (uint8_t)87, (uint8_t)246, (uint8_t)111, (uint8_t)248, (uint8_t)212, (uint8_t)26, (uint8_t)82, (uint8_t)65, (uint8_t)130, (uint8_t)5, (uint8_t)236, (uint8_t)34, (uint8_t)114, (uint8_t)123, (uint8_t)47, (uint8_t)107, (uint8_t)191, (uint8_t)192, (uint8_t)11, (uint8_t)191, (uint8_t)210, (uint8_t)115, (uint8_t)198, (uint8_t)79, (uint8_t)34, (uint8_t)85, (uint8_t)253, (uint8_t)35, (uint8_t)224, (uint8_t)255, (uint8_t)150, (uint8_t)201, (uint8_t)137, (uint8_t)10, (uint8_t)145, (uint8_t)16};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_lat_SET((int32_t) -1576904728, PH.base.pack) ;
        p124_alt_SET((int32_t)598671089, PH.base.pack) ;
        p124_lon_SET((int32_t) -949333529, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)7422050247186353722L, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)10132, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)21719, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)57083, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)2972374466L, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)32651, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)37582, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)12349, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)90, (uint8_t)222, (uint8_t)194, (uint8_t)133, (uint8_t)240, (uint8_t)185, (uint8_t)205, (uint8_t)204, (uint8_t)153, (uint8_t)242, (uint8_t)117, (uint8_t)102, (uint8_t)134, (uint8_t)185, (uint8_t)40, (uint8_t)248, (uint8_t)135, (uint8_t)17, (uint8_t)90, (uint8_t)236, (uint8_t)53, (uint8_t)14, (uint8_t)86, (uint8_t)206, (uint8_t)206, (uint8_t)67, (uint8_t)148, (uint8_t)169, (uint8_t)27, (uint8_t)239, (uint8_t)131, (uint8_t)192, (uint8_t)214, (uint8_t)133, (uint8_t)101, (uint8_t)202, (uint8_t)8, (uint8_t)167, (uint8_t)159, (uint8_t)27, (uint8_t)101, (uint8_t)51, (uint8_t)76, (uint8_t)207, (uint8_t)23, (uint8_t)199, (uint8_t)195, (uint8_t)136, (uint8_t)225, (uint8_t)119, (uint8_t)66, (uint8_t)13, (uint8_t)182, (uint8_t)71, (uint8_t)150, (uint8_t)90, (uint8_t)250, (uint8_t)246, (uint8_t)23, (uint8_t)242, (uint8_t)45, (uint8_t)86, (uint8_t)170, (uint8_t)27, (uint8_t)53, (uint8_t)170, (uint8_t)134, (uint8_t)164, (uint8_t)1, (uint8_t)18};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)58887, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1550268696L, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1525180840, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2169457486L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -2019317851, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p127_tow_SET((uint32_t)1128995994L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -117076246, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)23376, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)2188112759L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1153436026, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_wn_SET((uint16_t)(uint16_t)40214, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)1321384928, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2130017182L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)743958766L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1576002052, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1881188428, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1658909605, PH.base.pack) ;
        p128_tow_SET((uint32_t)247034447L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_xmag_SET((int16_t)(int16_t)32600, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)28385, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)9185, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -11059, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -20221, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -29519, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)14121, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -4341, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)26781, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)3127429087L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_payload_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)18726, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p130_size_SET((uint32_t)2114239819L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)34223, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)53845, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)50, (uint8_t)115, (uint8_t)62, (uint8_t)105, (uint8_t)240, (uint8_t)244, (uint8_t)11, (uint8_t)167, (uint8_t)147, (uint8_t)22, (uint8_t)59, (uint8_t)226, (uint8_t)169, (uint8_t)45, (uint8_t)8, (uint8_t)207, (uint8_t)97, (uint8_t)67, (uint8_t)40, (uint8_t)128, (uint8_t)23, (uint8_t)145, (uint8_t)143, (uint8_t)49, (uint8_t)64, (uint8_t)191, (uint8_t)98, (uint8_t)65, (uint8_t)143, (uint8_t)179, (uint8_t)28, (uint8_t)25, (uint8_t)159, (uint8_t)232, (uint8_t)184, (uint8_t)178, (uint8_t)150, (uint8_t)9, (uint8_t)32, (uint8_t)186, (uint8_t)103, (uint8_t)191, (uint8_t)248, (uint8_t)18, (uint8_t)147, (uint8_t)123, (uint8_t)253, (uint8_t)233, (uint8_t)94, (uint8_t)254, (uint8_t)35, (uint8_t)61, (uint8_t)227, (uint8_t)23, (uint8_t)35, (uint8_t)161, (uint8_t)223, (uint8_t)47, (uint8_t)41, (uint8_t)69, (uint8_t)121, (uint8_t)201, (uint8_t)236, (uint8_t)229, (uint8_t)224, (uint8_t)73, (uint8_t)121, (uint8_t)179, (uint8_t)81, (uint8_t)181, (uint8_t)121, (uint8_t)192, (uint8_t)94, (uint8_t)69, (uint8_t)253, (uint8_t)205, (uint8_t)52, (uint8_t)114, (uint8_t)125, (uint8_t)180, (uint8_t)113, (uint8_t)139, (uint8_t)221, (uint8_t)106, (uint8_t)243, (uint8_t)115, (uint8_t)76, (uint8_t)163, (uint8_t)169, (uint8_t)192, (uint8_t)153, (uint8_t)151, (uint8_t)188, (uint8_t)119, (uint8_t)96, (uint8_t)130, (uint8_t)97, (uint8_t)234, (uint8_t)72, (uint8_t)235, (uint8_t)131, (uint8_t)98, (uint8_t)74, (uint8_t)15, (uint8_t)185, (uint8_t)37, (uint8_t)207, (uint8_t)59, (uint8_t)167, (uint8_t)207, (uint8_t)249, (uint8_t)0, (uint8_t)48, (uint8_t)10, (uint8_t)173, (uint8_t)205, (uint8_t)91, (uint8_t)115, (uint8_t)140, (uint8_t)18, (uint8_t)198, (uint8_t)165, (uint8_t)195, (uint8_t)83, (uint8_t)149, (uint8_t)93, (uint8_t)211, (uint8_t)43, (uint8_t)67, (uint8_t)232, (uint8_t)22, (uint8_t)199, (uint8_t)55, (uint8_t)230, (uint8_t)153, (uint8_t)193, (uint8_t)206, (uint8_t)77, (uint8_t)69, (uint8_t)239, (uint8_t)131, (uint8_t)50, (uint8_t)48, (uint8_t)215, (uint8_t)77, (uint8_t)234, (uint8_t)106, (uint8_t)152, (uint8_t)45, (uint8_t)207, (uint8_t)152, (uint8_t)118, (uint8_t)11, (uint8_t)135, (uint8_t)52, (uint8_t)206, (uint8_t)166, (uint8_t)66, (uint8_t)66, (uint8_t)207, (uint8_t)38, (uint8_t)224, (uint8_t)252, (uint8_t)253, (uint8_t)17, (uint8_t)193, (uint8_t)19, (uint8_t)58, (uint8_t)4, (uint8_t)115, (uint8_t)74, (uint8_t)33, (uint8_t)230, (uint8_t)221, (uint8_t)115, (uint8_t)86, (uint8_t)196, (uint8_t)76, (uint8_t)69, (uint8_t)52, (uint8_t)250, (uint8_t)57, (uint8_t)73, (uint8_t)100, (uint8_t)105, (uint8_t)33, (uint8_t)33, (uint8_t)235, (uint8_t)175, (uint8_t)50, (uint8_t)160, (uint8_t)245, (uint8_t)145, (uint8_t)40, (uint8_t)206, (uint8_t)249, (uint8_t)81, (uint8_t)120, (uint8_t)20, (uint8_t)22, (uint8_t)15, (uint8_t)198, (uint8_t)155, (uint8_t)70, (uint8_t)101, (uint8_t)240, (uint8_t)150, (uint8_t)246, (uint8_t)38, (uint8_t)196, (uint8_t)170, (uint8_t)157, (uint8_t)120, (uint8_t)159, (uint8_t)22, (uint8_t)8, (uint8_t)120, (uint8_t)204, (uint8_t)201, (uint8_t)157, (uint8_t)177, (uint8_t)176, (uint8_t)167, (uint8_t)144, (uint8_t)211, (uint8_t)230, (uint8_t)3, (uint8_t)129, (uint8_t)216, (uint8_t)208, (uint8_t)106, (uint8_t)147, (uint8_t)28, (uint8_t)27, (uint8_t)71, (uint8_t)138, (uint8_t)237, (uint8_t)164, (uint8_t)94, (uint8_t)131, (uint8_t)65, (uint8_t)28, (uint8_t)58, (uint8_t)27, (uint8_t)21, (uint8_t)194, (uint8_t)41, (uint8_t)47, (uint8_t)217, (uint8_t)205, (uint8_t)44, (uint8_t)222, (uint8_t)35};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)33152, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_current_distance_SET((uint16_t)(uint16_t)36934, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_90, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)80718004L, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)45972, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)50780, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t) -1496781748, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)2657, PH.base.pack) ;
        p133_lon_SET((int32_t) -1595998616, PH.base.pack) ;
        p133_mask_SET((uint64_t)1790440619027252403L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        {
            int16_t data_[] =  {(int16_t)16577, (int16_t)1369, (int16_t) -21649, (int16_t) -11482, (int16_t) -12972, (int16_t)27129, (int16_t)9037, (int16_t) -28602, (int16_t) -28164, (int16_t)3084, (int16_t)27117, (int16_t)10776, (int16_t) -31708, (int16_t)19216, (int16_t) -10963, (int16_t)30771};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lat_SET((int32_t)874430662, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)24981, PH.base.pack) ;
        p134_lon_SET((int32_t)665220919, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -88084675, PH.base.pack) ;
        p135_lon_SET((int32_t)212413905, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_lon_SET((int32_t) -1524817939, PH.base.pack) ;
        p136_lat_SET((int32_t)1707727787, PH.base.pack) ;
        p136_current_height_SET((float) -2.890101E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)20228, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)27198, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)53079, PH.base.pack) ;
        p136_terrain_height_SET((float) -2.7449623E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)2982233351L, PH.base.pack) ;
        p137_press_diff_SET((float)1.4807171E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)14312, PH.base.pack) ;
        p137_press_abs_SET((float)5.148311E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_y_SET((float)1.6115177E38F, PH.base.pack) ;
        p138_z_SET((float)1.902488E38F, PH.base.pack) ;
        {
            float q[] =  {1.3018991E38F, -2.8215404E38F, -1.0518461E38F, -5.5664257E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_time_usec_SET((uint64_t)4742482759855228014L, PH.base.pack) ;
        p138_x_SET((float)1.2202362E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)278930189502719008L, PH.base.pack) ;
        {
            float controls[] =  {-2.0828486E37F, -2.139906E38F, 1.1019805E37F, -2.8056986E38F, -2.7953426E36F, 2.4706206E38F, -1.4725866E37F, 1.5392862E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)8481397963540108728L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        {
            float controls[] =  {-2.5029583E38F, 2.7819063E38F, -3.1139555E38F, -3.2830358E38F, -9.493404E37F, 2.0154515E38F, -1.6140118E38F, 1.7444854E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float)2.5904266E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)5555626788292339978L, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.9326767E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -2.796283E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -3.199418E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float)2.84208E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.272274E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_transfer_type_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)44, (uint8_t)36, (uint8_t)217, (uint8_t)208, (uint8_t)150, (uint8_t)120, (uint8_t)113, (uint8_t)86, (uint8_t)229, (uint8_t)53, (uint8_t)171, (uint8_t)127, (uint8_t)162, (uint8_t)34, (uint8_t)61, (uint8_t)89, (uint8_t)251, (uint8_t)110, (uint8_t)246, (uint8_t)225, (uint8_t)220, (uint8_t)0, (uint8_t)99, (uint8_t)235, (uint8_t)17, (uint8_t)3, (uint8_t)18, (uint8_t)195, (uint8_t)187, (uint8_t)91, (uint8_t)62, (uint8_t)145, (uint8_t)40, (uint8_t)113, (uint8_t)43, (uint8_t)130, (uint8_t)214, (uint8_t)173, (uint8_t)195, (uint8_t)99, (uint8_t)238, (uint8_t)112, (uint8_t)83, (uint8_t)13, (uint8_t)79, (uint8_t)193, (uint8_t)89, (uint8_t)99, (uint8_t)182, (uint8_t)9, (uint8_t)248, (uint8_t)98, (uint8_t)22, (uint8_t)153, (uint8_t)85, (uint8_t)108, (uint8_t)45, (uint8_t)191, (uint8_t)45, (uint8_t)128, (uint8_t)163, (uint8_t)93, (uint8_t)177, (uint8_t)126, (uint8_t)111, (uint8_t)21, (uint8_t)223, (uint8_t)143, (uint8_t)202, (uint8_t)39, (uint8_t)168, (uint8_t)240, (uint8_t)83, (uint8_t)94, (uint8_t)125, (uint8_t)115, (uint8_t)55, (uint8_t)102, (uint8_t)10, (uint8_t)184, (uint8_t)130, (uint8_t)252, (uint8_t)22, (uint8_t)66, (uint8_t)126, (uint8_t)82, (uint8_t)38, (uint8_t)32, (uint8_t)178, (uint8_t)10, (uint8_t)112, (uint8_t)220, (uint8_t)49, (uint8_t)221, (uint8_t)122, (uint8_t)57, (uint8_t)91, (uint8_t)254, (uint8_t)115, (uint8_t)205, (uint8_t)132, (uint8_t)151, (uint8_t)99, (uint8_t)34, (uint8_t)45, (uint8_t)65, (uint8_t)89, (uint8_t)198, (uint8_t)251, (uint8_t)214, (uint8_t)201, (uint8_t)95, (uint8_t)46, (uint8_t)241, (uint8_t)109, (uint8_t)194, (uint8_t)138, (uint8_t)133, (uint8_t)238, (uint8_t)108};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)142, (uint8_t)16, (uint8_t)219, (uint8_t)64, (uint8_t)112, (uint8_t)253, (uint8_t)170, (uint8_t)75, (uint8_t)173, (uint8_t)188, (uint8_t)184, (uint8_t)216, (uint8_t)100, (uint8_t)250, (uint8_t)230, (uint8_t)63, (uint8_t)4, (uint8_t)67, (uint8_t)136, (uint8_t)1, (uint8_t)225, (uint8_t)42, (uint8_t)39, (uint8_t)48, (uint8_t)127, (uint8_t)218, (uint8_t)68, (uint8_t)114, (uint8_t)181, (uint8_t)221, (uint8_t)23, (uint8_t)188, (uint8_t)158, (uint8_t)208, (uint8_t)204, (uint8_t)51, (uint8_t)34, (uint8_t)18, (uint8_t)171, (uint8_t)95, (uint8_t)93, (uint8_t)108, (uint8_t)78, (uint8_t)130, (uint8_t)64, (uint8_t)240, (uint8_t)35, (uint8_t)13, (uint8_t)65, (uint8_t)162, (uint8_t)123, (uint8_t)9, (uint8_t)26, (uint8_t)135, (uint8_t)37, (uint8_t)19, (uint8_t)180, (uint8_t)41, (uint8_t)79, (uint8_t)205, (uint8_t)82, (uint8_t)213, (uint8_t)169, (uint8_t)97, (uint8_t)137, (uint8_t)98, (uint8_t)128, (uint8_t)101, (uint8_t)130, (uint8_t)140, (uint8_t)138, (uint8_t)189, (uint8_t)100, (uint8_t)220, (uint8_t)238, (uint8_t)98, (uint8_t)233, (uint8_t)16, (uint8_t)201, (uint8_t)20, (uint8_t)88, (uint8_t)42, (uint8_t)222, (uint8_t)2, (uint8_t)252, (uint8_t)199, (uint8_t)92, (uint8_t)248, (uint8_t)204, (uint8_t)216, (uint8_t)122, (uint8_t)171, (uint8_t)143, (uint8_t)28, (uint8_t)146, (uint8_t)18, (uint8_t)214, (uint8_t)240, (uint8_t)157, (uint8_t)20, (uint8_t)231, (uint8_t)106, (uint8_t)156, (uint8_t)115, (uint8_t)172, (uint8_t)93, (uint8_t)172, (uint8_t)44, (uint8_t)61, (uint8_t)134, (uint8_t)38, (uint8_t)87, (uint8_t)253, (uint8_t)57, (uint8_t)82, (uint8_t)222, (uint8_t)26, (uint8_t)123, (uint8_t)13, (uint8_t)137};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t)4710, PH.base.pack) ;
        p143_press_abs_SET((float) -1.1777193E38F, PH.base.pack) ;
        p143_press_diff_SET((float)2.5586268E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3358482159L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_alt_SET((float) -4.1297017E37F, PH.base.pack) ;
        {
            float position_cov[] =  {-3.3127688E38F, -4.618932E37F, 7.0801645E36F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)869469894, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)6169813363336251809L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p144_lat_SET((int32_t)311671921, PH.base.pack) ;
        {
            float acc[] =  {-9.670178E37F, 2.8169148E38F, 1.0335474E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)3645589240007241588L, PH.base.pack) ;
        {
            float attitude_q[] =  {-1.0152855E38F, -2.8780494E38F, -1.7972888E38F, 3.1926598E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-6.325812E37F, -1.1832166E38F, -9.201623E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-1.7012186E38F, 2.9918455E38F, 1.6493914E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_pos_SET((float)2.6698847E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -7.5991886E37F, PH.base.pack) ;
        p146_z_vel_SET((float)2.2633814E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -2.7937612E38F, PH.base.pack) ;
        p146_y_acc_SET((float)2.2699251E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {2.3921287E38F, 1.2362515E38F, -1.2020262E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float)2.1851222E38F, PH.base.pack) ;
        {
            float q[] =  {-1.6868375E38F, -7.090539E37F, 3.155329E38F, -3.1603987E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float) -2.9038028E38F, PH.base.pack) ;
        p146_y_vel_SET((float)1.8269802E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)453514606673973763L, PH.base.pack) ;
        p146_z_acc_SET((float)2.1469328E38F, PH.base.pack) ;
        p146_roll_rate_SET((float)3.1901383E38F, PH.base.pack) ;
        p146_x_vel_SET((float)3.1959966E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-7.6388E37F, -3.2424253E38F, 1.7838286E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_y_pos_SET((float) -2.6433517E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -1.0881253E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_id_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)116, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -1896168286, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -4808, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)23387, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)3773, (uint16_t)59578, (uint16_t)5277, (uint16_t)47280, (uint16_t)56908, (uint16_t)43586, (uint16_t)43013, (uint16_t)24813, (uint16_t)12749, (uint16_t)32197};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -715949126, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_middleware_sw_version_SET((uint32_t)2501623102L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)197, (uint8_t)32, (uint8_t)66, (uint8_t)169, (uint8_t)232, (uint8_t)128, (uint8_t)155, (uint8_t)160};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)61240, PH.base.pack) ;
        p148_uid_SET((uint64_t)6714320671013403829L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)194, (uint8_t)201, (uint8_t)154, (uint8_t)161, (uint8_t)117, (uint8_t)72, (uint8_t)213, (uint8_t)93, (uint8_t)135, (uint8_t)12, (uint8_t)101, (uint8_t)251, (uint8_t)72, (uint8_t)193, (uint8_t)47, (uint8_t)67, (uint8_t)247, (uint8_t)188};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3769520591L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)158, (uint8_t)245, (uint8_t)218, (uint8_t)44, (uint8_t)35, (uint8_t)82, (uint8_t)32, (uint8_t)2};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)823564179L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)21627, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)118, (uint8_t)60, (uint8_t)26, (uint8_t)106, (uint8_t)224, (uint8_t)229, (uint8_t)8, (uint8_t)89};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)3302490645L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_size_y_SET((float)1.1373532E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)8221851501947560040L, PH.base.pack) ;
        p149_angle_x_SET((float)4.456888E37F, PH.base.pack) ;
        p149_y_SET((float) -1.4828003E38F, &PH) ;
        p149_angle_y_SET((float) -1.2141077E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)215, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p149_z_SET((float)1.6351372E37F, &PH) ;
        {
            float q[] =  {1.5537684E38F, -2.4062142E38F, -3.1071384E38F, -3.0903247E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_size_x_SET((float)1.795537E38F, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p149_x_SET((float)1.3726588E37F, &PH) ;
        p149_distance_SET((float)2.577104E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -7.8796746E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.6497106E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.3442267E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -1.1789038E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.4364985E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)7132569226164688200L, PH.base.pack) ;
        p230_mag_ratio_SET((float)2.9867674E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -8.5555647E36F, PH.base.pack) ;
        p230_tas_ratio_SET((float)1.8966558E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_wind_alt_SET((float) -1.03935814E37F, PH.base.pack) ;
        p231_var_horiz_SET((float)1.4777038E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)3575546442530607479L, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -1.8896423E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)1.6430148E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -1.5034378E38F, PH.base.pack) ;
        p231_wind_y_SET((float)1.6453541E37F, PH.base.pack) ;
        p231_var_vert_SET((float) -2.3672093E38F, PH.base.pack) ;
        p231_wind_z_SET((float) -2.8771703E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_ve_SET((float)1.4387982E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p232_lon_SET((int32_t)365602560, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)15404, PH.base.pack) ;
        p232_alt_SET((float) -2.2203856E38F, PH.base.pack) ;
        p232_vdop_SET((float)1.8061296E37F, PH.base.pack) ;
        p232_lat_SET((int32_t)1722335177, PH.base.pack) ;
        p232_vd_SET((float) -3.2947841E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -2.0663238E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)8.1026396E37F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)1879250457L, PH.base.pack) ;
        p232_hdop_SET((float)2.6039464E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -3.0455044E38F, PH.base.pack) ;
        p232_vn_SET((float) -7.175949E37F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)7707018555425654356L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)119, (uint8_t)33, (uint8_t)37, (uint8_t)154, (uint8_t)210, (uint8_t)246, (uint8_t)168, (uint8_t)62, (uint8_t)226, (uint8_t)147, (uint8_t)48, (uint8_t)105, (uint8_t)157, (uint8_t)128, (uint8_t)165, (uint8_t)21, (uint8_t)93, (uint8_t)89, (uint8_t)94, (uint8_t)85, (uint8_t)116, (uint8_t)109, (uint8_t)242, (uint8_t)145, (uint8_t)207, (uint8_t)110, (uint8_t)102, (uint8_t)152, (uint8_t)106, (uint8_t)130, (uint8_t)11, (uint8_t)182, (uint8_t)226, (uint8_t)112, (uint8_t)214, (uint8_t)220, (uint8_t)171, (uint8_t)1, (uint8_t)233, (uint8_t)226, (uint8_t)255, (uint8_t)248, (uint8_t)23, (uint8_t)203, (uint8_t)127, (uint8_t)138, (uint8_t)244, (uint8_t)36, (uint8_t)222, (uint8_t)223, (uint8_t)35, (uint8_t)183, (uint8_t)90, (uint8_t)12, (uint8_t)81, (uint8_t)227, (uint8_t)166, (uint8_t)126, (uint8_t)226, (uint8_t)236, (uint8_t)213, (uint8_t)186, (uint8_t)143, (uint8_t)67, (uint8_t)164, (uint8_t)209, (uint8_t)32, (uint8_t)123, (uint8_t)9, (uint8_t)225, (uint8_t)169, (uint8_t)71, (uint8_t)93, (uint8_t)50, (uint8_t)38, (uint8_t)174, (uint8_t)172, (uint8_t)255, (uint8_t)66, (uint8_t)58, (uint8_t)220, (uint8_t)88, (uint8_t)189, (uint8_t)145, (uint8_t)37, (uint8_t)96, (uint8_t)147, (uint8_t)89, (uint8_t)125, (uint8_t)76, (uint8_t)208, (uint8_t)64, (uint8_t)162, (uint8_t)145, (uint8_t)193, (uint8_t)117, (uint8_t)186, (uint8_t)213, (uint8_t)107, (uint8_t)110, (uint8_t)68, (uint8_t)38, (uint8_t)143, (uint8_t)103, (uint8_t)164, (uint8_t)157, (uint8_t)105, (uint8_t)141, (uint8_t)145, (uint8_t)79, (uint8_t)31, (uint8_t)128, (uint8_t)234, (uint8_t)96, (uint8_t)142, (uint8_t)242, (uint8_t)17, (uint8_t)249, (uint8_t)244, (uint8_t)5, (uint8_t)207, (uint8_t)9, (uint8_t)169, (uint8_t)38, (uint8_t)32, (uint8_t)191, (uint8_t)157, (uint8_t)165, (uint8_t)39, (uint8_t)49, (uint8_t)29, (uint8_t)248, (uint8_t)197, (uint8_t)72, (uint8_t)225, (uint8_t)56, (uint8_t)154, (uint8_t)243, (uint8_t)41, (uint8_t)206, (uint8_t)178, (uint8_t)85, (uint8_t)113, (uint8_t)193, (uint8_t)121, (uint8_t)37, (uint8_t)84, (uint8_t)247, (uint8_t)187, (uint8_t)87, (uint8_t)40, (uint8_t)158, (uint8_t)129, (uint8_t)86, (uint8_t)26, (uint8_t)251, (uint8_t)133, (uint8_t)51, (uint8_t)213, (uint8_t)72, (uint8_t)66, (uint8_t)20, (uint8_t)223, (uint8_t)55, (uint8_t)181, (uint8_t)37, (uint8_t)187, (uint8_t)190, (uint8_t)55, (uint8_t)189, (uint8_t)28, (uint8_t)84, (uint8_t)244, (uint8_t)129, (uint8_t)116, (uint8_t)116, (uint8_t)50, (uint8_t)79, (uint8_t)109, (uint8_t)117};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p234_latitude_SET((int32_t)1067234945, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)19998, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -104, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p234_longitude_SET((int32_t) -729839490, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)125, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -7, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)41226, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -22869, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)12841, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)87, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1987214699L, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)57385, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)2998, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)8446, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_clipping_0_SET((uint32_t)1414447387L, PH.base.pack) ;
        p241_vibration_x_SET((float)3.2088395E38F, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.1040916E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3842944251L, PH.base.pack) ;
        p241_vibration_z_SET((float) -2.7237839E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6279131286665898101L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)650224710L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {-6.0963335E37F, -8.482872E37F, 1.5134563E38F, 1.5783302E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_time_usec_SET((uint64_t)4047951678379787651L, &PH) ;
        p242_approach_y_SET((float)2.5455381E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1903186662, PH.base.pack) ;
        p242_approach_x_SET((float)1.8525675E38F, PH.base.pack) ;
        p242_z_SET((float) -1.1551764E38F, PH.base.pack) ;
        p242_approach_z_SET((float)2.248477E37F, PH.base.pack) ;
        p242_x_SET((float) -1.1923596E38F, PH.base.pack) ;
        p242_y_SET((float)2.0025014E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1523001498, PH.base.pack) ;
        p242_longitude_SET((int32_t) -1942099713, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p243_y_SET((float) -2.0730635E38F, PH.base.pack) ;
        p243_approach_z_SET((float)9.967283E37F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)5372670906738874843L, &PH) ;
        p243_approach_x_SET((float)1.6758945E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -5.3254316E37F, PH.base.pack) ;
        p243_latitude_SET((int32_t)809130388, PH.base.pack) ;
        p243_x_SET((float)1.2117227E38F, PH.base.pack) ;
        {
            float q[] =  {2.2234397E38F, 1.3394776E38F, 2.1702168E38F, 2.167638E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float)1.5450746E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1143546080, PH.base.pack) ;
        p243_altitude_SET((int32_t)84849262, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1270423094, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)54992, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_lon_SET((int32_t)962908736, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -3921, PH.base.pack) ;
        p246_lat_SET((int32_t)1746529933, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)5196, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)44590, PH.base.pack) ;
        {
            char16_t* callsign = u"cpnvlui";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ULTRA_LIGHT, PH.base.pack) ;
        p246_altitude_SET((int32_t)1827827252, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)13730, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2515568583L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.8287462E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -3.0405348E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.9198912E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)1611159779L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_message_type_SET((uint16_t)(uint16_t)53479, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)68, (uint8_t)161, (uint8_t)31, (uint8_t)182, (uint8_t)14, (uint8_t)134, (uint8_t)92, (uint8_t)46, (uint8_t)157, (uint8_t)144, (uint8_t)191, (uint8_t)6, (uint8_t)180, (uint8_t)223, (uint8_t)98, (uint8_t)17, (uint8_t)3, (uint8_t)192, (uint8_t)220, (uint8_t)109, (uint8_t)52, (uint8_t)71, (uint8_t)188, (uint8_t)192, (uint8_t)41, (uint8_t)77, (uint8_t)189, (uint8_t)53, (uint8_t)95, (uint8_t)137, (uint8_t)122, (uint8_t)104, (uint8_t)43, (uint8_t)0, (uint8_t)66, (uint8_t)12, (uint8_t)93, (uint8_t)234, (uint8_t)243, (uint8_t)147, (uint8_t)209, (uint8_t)202, (uint8_t)69, (uint8_t)191, (uint8_t)65, (uint8_t)74, (uint8_t)23, (uint8_t)4, (uint8_t)245, (uint8_t)124, (uint8_t)134, (uint8_t)157, (uint8_t)0, (uint8_t)26, (uint8_t)142, (uint8_t)232, (uint8_t)163, (uint8_t)107, (uint8_t)234, (uint8_t)89, (uint8_t)68, (uint8_t)59, (uint8_t)27, (uint8_t)43, (uint8_t)190, (uint8_t)156, (uint8_t)140, (uint8_t)101, (uint8_t)37, (uint8_t)42, (uint8_t)112, (uint8_t)23, (uint8_t)227, (uint8_t)31, (uint8_t)2, (uint8_t)166, (uint8_t)95, (uint8_t)79, (uint8_t)38, (uint8_t)164, (uint8_t)151, (uint8_t)168, (uint8_t)251, (uint8_t)246, (uint8_t)29, (uint8_t)9, (uint8_t)79, (uint8_t)119, (uint8_t)253, (uint8_t)165, (uint8_t)153, (uint8_t)95, (uint8_t)20, (uint8_t)43, (uint8_t)236, (uint8_t)225, (uint8_t)61, (uint8_t)234, (uint8_t)211, (uint8_t)62, (uint8_t)38, (uint8_t)233, (uint8_t)232, (uint8_t)112, (uint8_t)248, (uint8_t)217, (uint8_t)93, (uint8_t)208, (uint8_t)115, (uint8_t)40, (uint8_t)209, (uint8_t)180, (uint8_t)119, (uint8_t)10, (uint8_t)98, (uint8_t)182, (uint8_t)48, (uint8_t)212, (uint8_t)146, (uint8_t)154, (uint8_t)169, (uint8_t)229, (uint8_t)147, (uint8_t)91, (uint8_t)222, (uint8_t)192, (uint8_t)171, (uint8_t)68, (uint8_t)166, (uint8_t)10, (uint8_t)169, (uint8_t)37, (uint8_t)255, (uint8_t)223, (uint8_t)127, (uint8_t)4, (uint8_t)55, (uint8_t)232, (uint8_t)219, (uint8_t)29, (uint8_t)250, (uint8_t)142, (uint8_t)68, (uint8_t)156, (uint8_t)162, (uint8_t)31, (uint8_t)15, (uint8_t)95, (uint8_t)166, (uint8_t)44, (uint8_t)251, (uint8_t)248, (uint8_t)140, (uint8_t)92, (uint8_t)152, (uint8_t)146, (uint8_t)197, (uint8_t)75, (uint8_t)231, (uint8_t)244, (uint8_t)238, (uint8_t)115, (uint8_t)25, (uint8_t)12, (uint8_t)5, (uint8_t)19, (uint8_t)2, (uint8_t)224, (uint8_t)117, (uint8_t)254, (uint8_t)217, (uint8_t)16, (uint8_t)216, (uint8_t)113, (uint8_t)58, (uint8_t)103, (uint8_t)228, (uint8_t)130, (uint8_t)19, (uint8_t)12, (uint8_t)215, (uint8_t)146, (uint8_t)191, (uint8_t)249, (uint8_t)198, (uint8_t)214, (uint8_t)25, (uint8_t)52, (uint8_t)168, (uint8_t)227, (uint8_t)214, (uint8_t)133, (uint8_t)16, (uint8_t)215, (uint8_t)87, (uint8_t)65, (uint8_t)229, (uint8_t)194, (uint8_t)207, (uint8_t)159, (uint8_t)1, (uint8_t)193, (uint8_t)180, (uint8_t)255, (uint8_t)191, (uint8_t)162, (uint8_t)206, (uint8_t)87, (uint8_t)230, (uint8_t)60, (uint8_t)82, (uint8_t)74, (uint8_t)21, (uint8_t)61, (uint8_t)50, (uint8_t)65, (uint8_t)79, (uint8_t)112, (uint8_t)7, (uint8_t)103, (uint8_t)195, (uint8_t)244, (uint8_t)5, (uint8_t)237, (uint8_t)120, (uint8_t)125, (uint8_t)15, (uint8_t)20, (uint8_t)211, (uint8_t)33, (uint8_t)226, (uint8_t)186, (uint8_t)230, (uint8_t)156, (uint8_t)102, (uint8_t)113, (uint8_t)137, (uint8_t)62, (uint8_t)193, (uint8_t)159, (uint8_t)52, (uint8_t)184, (uint8_t)109, (uint8_t)199, (uint8_t)164, (uint8_t)64, (uint8_t)119, (uint8_t)92, (uint8_t)125};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -32, (int8_t)117, (int8_t)19, (int8_t)118, (int8_t)91, (int8_t) -69, (int8_t) -95, (int8_t) -7, (int8_t)122, (int8_t)14, (int8_t) -115, (int8_t)89, (int8_t) -15, (int8_t)98, (int8_t)47, (int8_t)9, (int8_t)88, (int8_t)35, (int8_t) -16, (int8_t) -7, (int8_t)94, (int8_t) -95, (int8_t)85, (int8_t) -50, (int8_t) -115, (int8_t) -12, (int8_t) -10, (int8_t)48, (int8_t)42, (int8_t) -35, (int8_t)39, (int8_t)71};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)10853, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        {
            char16_t* name = u"hRRRSymfF";
            p250_name_SET_(name, &PH) ;
        }
        p250_x_SET((float)2.254717E38F, PH.base.pack) ;
        p250_z_SET((float) -1.0718088E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)5134450796028266579L, PH.base.pack) ;
        p250_y_SET((float) -2.3732458E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"zekcxd";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -2.27537E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)2056207963L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)987372422L, PH.base.pack) ;
        {
            char16_t* name = u"dwXgniadcv";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t) -1634765001, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"Kzioglpguprub";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ERROR, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_value_SET((float)2.33075E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)3820264182L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)8077228205420765776L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)243, (uint8_t)44, (uint8_t)43, (uint8_t)137, (uint8_t)52, (uint8_t)82, (uint8_t)244, (uint8_t)51, (uint8_t)199, (uint8_t)142, (uint8_t)50, (uint8_t)67, (uint8_t)118, (uint8_t)224, (uint8_t)157, (uint8_t)234, (uint8_t)161, (uint8_t)236, (uint8_t)129, (uint8_t)186, (uint8_t)122, (uint8_t)115, (uint8_t)0, (uint8_t)113, (uint8_t)66, (uint8_t)35, (uint8_t)108, (uint8_t)14, (uint8_t)215, (uint8_t)12, (uint8_t)240, (uint8_t)171};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1550240066L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)2043659085L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"cpakwreojucvfrvscbw";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_v_SET((uint16_t)(uint16_t)50029, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)3824532545L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"EjljcimefErzqgqahofefiorvbkznkulqsluyvbaQmyvcilevnTzyPdxmzF";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)20533, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -1.8339965E38F, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)204, (uint8_t)202, (uint8_t)137, (uint8_t)226, (uint8_t)253, (uint8_t)163, (uint8_t)62, (uint8_t)166, (uint8_t)130, (uint8_t)19, (uint8_t)141, (uint8_t)233, (uint8_t)106, (uint8_t)52, (uint8_t)39, (uint8_t)18, (uint8_t)179, (uint8_t)77, (uint8_t)251, (uint8_t)251, (uint8_t)248, (uint8_t)19, (uint8_t)180, (uint8_t)155, (uint8_t)52, (uint8_t)168, (uint8_t)38, (uint8_t)20, (uint8_t)70, (uint8_t)97, (uint8_t)124, (uint8_t)85};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p259_focal_length_SET((float) -5.9014886E37F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)3829, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)216, (uint8_t)134, (uint8_t)39, (uint8_t)232, (uint8_t)133, (uint8_t)217, (uint8_t)117, (uint8_t)99, (uint8_t)207, (uint8_t)54, (uint8_t)208, (uint8_t)121, (uint8_t)214, (uint8_t)225, (uint8_t)44, (uint8_t)244, (uint8_t)196, (uint8_t)16, (uint8_t)183, (uint8_t)236, (uint8_t)106, (uint8_t)250, (uint8_t)12, (uint8_t)65, (uint8_t)147, (uint8_t)242, (uint8_t)230, (uint8_t)88, (uint8_t)160, (uint8_t)211, (uint8_t)170, (uint8_t)12};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float) -2.5809105E38F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)355087958L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)826266580L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_read_speed_SET((float)4.451694E36F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1708617092L, PH.base.pack) ;
        p261_write_speed_SET((float) -9.046608E37F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.0529648E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)1.9407658E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.8186118E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float)2.3469458E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2588881319L, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3358054712L, PH.base.pack) ;
        p262_available_capacity_SET((float) -3.2614072E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_capture_result_SET((int8_t)(int8_t)35, PH.base.pack) ;
        p263_image_index_SET((int32_t)1263044855, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p263_alt_SET((int32_t) -97698208, PH.base.pack) ;
        {
            float q[] =  {6.292579E35F, -3.2314499E38F, -1.5814466E38F, -2.4917622E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            char16_t* file_url = u"oHFanvgdvmmwdnfXaycqfnvjpuhqalxFjkesfnwrvtdmed";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_utc_SET((uint64_t)8446569823491706432L, PH.base.pack) ;
        p263_lon_SET((int32_t) -74153920, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2808535412L, PH.base.pack) ;
        p263_lat_SET((int32_t)404877262, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)1486977049, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)3131938477L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)3412698755959984176L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)6828627005519425455L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)3671817226360538816L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float)2.9501661E38F, PH.base.pack) ;
        p265_roll_SET((float) -2.0267478E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)2863815506L, PH.base.pack) ;
        p265_yaw_SET((float)3.0737586E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_target_system_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)17, (uint8_t)252, (uint8_t)202, (uint8_t)67, (uint8_t)126, (uint8_t)30, (uint8_t)139, (uint8_t)15, (uint8_t)175, (uint8_t)113, (uint8_t)200, (uint8_t)135, (uint8_t)245, (uint8_t)154, (uint8_t)121, (uint8_t)97, (uint8_t)111, (uint8_t)130, (uint8_t)244, (uint8_t)71, (uint8_t)241, (uint8_t)143, (uint8_t)36, (uint8_t)189, (uint8_t)131, (uint8_t)197, (uint8_t)211, (uint8_t)20, (uint8_t)160, (uint8_t)204, (uint8_t)82, (uint8_t)130, (uint8_t)177, (uint8_t)101, (uint8_t)201, (uint8_t)164, (uint8_t)156, (uint8_t)21, (uint8_t)122, (uint8_t)29, (uint8_t)116, (uint8_t)16, (uint8_t)75, (uint8_t)201, (uint8_t)223, (uint8_t)185, (uint8_t)69, (uint8_t)72, (uint8_t)233, (uint8_t)10, (uint8_t)179, (uint8_t)55, (uint8_t)50, (uint8_t)70, (uint8_t)39, (uint8_t)24, (uint8_t)70, (uint8_t)68, (uint8_t)234, (uint8_t)176, (uint8_t)234, (uint8_t)132, (uint8_t)190, (uint8_t)57, (uint8_t)93, (uint8_t)32, (uint8_t)140, (uint8_t)92, (uint8_t)28, (uint8_t)167, (uint8_t)173, (uint8_t)177, (uint8_t)62, (uint8_t)22, (uint8_t)31, (uint8_t)45, (uint8_t)162, (uint8_t)174, (uint8_t)159, (uint8_t)102, (uint8_t)242, (uint8_t)68, (uint8_t)76, (uint8_t)54, (uint8_t)62, (uint8_t)142, (uint8_t)164, (uint8_t)56, (uint8_t)127, (uint8_t)36, (uint8_t)241, (uint8_t)237, (uint8_t)130, (uint8_t)243, (uint8_t)149, (uint8_t)2, (uint8_t)186, (uint8_t)224, (uint8_t)110, (uint8_t)225, (uint8_t)137, (uint8_t)73, (uint8_t)54, (uint8_t)117, (uint8_t)108, (uint8_t)239, (uint8_t)23, (uint8_t)216, (uint8_t)210, (uint8_t)23, (uint8_t)111, (uint8_t)22, (uint8_t)15, (uint8_t)210, (uint8_t)34, (uint8_t)201, (uint8_t)69, (uint8_t)44, (uint8_t)32, (uint8_t)50, (uint8_t)144, (uint8_t)167, (uint8_t)255, (uint8_t)32, (uint8_t)33, (uint8_t)162, (uint8_t)133, (uint8_t)2, (uint8_t)253, (uint8_t)44, (uint8_t)251, (uint8_t)255, (uint8_t)70, (uint8_t)32, (uint8_t)29, (uint8_t)173, (uint8_t)225, (uint8_t)150, (uint8_t)209, (uint8_t)68, (uint8_t)149, (uint8_t)217, (uint8_t)41, (uint8_t)249, (uint8_t)28, (uint8_t)82, (uint8_t)231, (uint8_t)208, (uint8_t)224, (uint8_t)25, (uint8_t)7, (uint8_t)228, (uint8_t)156, (uint8_t)69, (uint8_t)147, (uint8_t)72, (uint8_t)151, (uint8_t)225, (uint8_t)16, (uint8_t)197, (uint8_t)6, (uint8_t)59, (uint8_t)11, (uint8_t)57, (uint8_t)30, (uint8_t)20, (uint8_t)38, (uint8_t)159, (uint8_t)82, (uint8_t)227, (uint8_t)25, (uint8_t)74, (uint8_t)84, (uint8_t)137, (uint8_t)175, (uint8_t)235, (uint8_t)133, (uint8_t)116, (uint8_t)126, (uint8_t)166, (uint8_t)198, (uint8_t)168, (uint8_t)163, (uint8_t)222, (uint8_t)117, (uint8_t)243, (uint8_t)83, (uint8_t)95, (uint8_t)219, (uint8_t)15, (uint8_t)57, (uint8_t)101, (uint8_t)158, (uint8_t)72, (uint8_t)59, (uint8_t)92, (uint8_t)245, (uint8_t)188, (uint8_t)237, (uint8_t)121, (uint8_t)219, (uint8_t)116, (uint8_t)21, (uint8_t)25, (uint8_t)116, (uint8_t)35, (uint8_t)208, (uint8_t)1, (uint8_t)186, (uint8_t)7, (uint8_t)201, (uint8_t)27, (uint8_t)197, (uint8_t)197, (uint8_t)118, (uint8_t)238, (uint8_t)89, (uint8_t)227, (uint8_t)47, (uint8_t)220, (uint8_t)62, (uint8_t)248, (uint8_t)8, (uint8_t)51, (uint8_t)154, (uint8_t)84, (uint8_t)143, (uint8_t)75, (uint8_t)223, (uint8_t)0, (uint8_t)186, (uint8_t)58, (uint8_t)227, (uint8_t)202, (uint8_t)145, (uint8_t)150, (uint8_t)110, (uint8_t)40, (uint8_t)102, (uint8_t)65, (uint8_t)186, (uint8_t)74, (uint8_t)83, (uint8_t)4, (uint8_t)223, (uint8_t)201, (uint8_t)169, (uint8_t)30, (uint8_t)175};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)22158, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_sequence_SET((uint16_t)(uint16_t)2690, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)95, (uint8_t)12, (uint8_t)224, (uint8_t)53, (uint8_t)255, (uint8_t)171, (uint8_t)49, (uint8_t)156, (uint8_t)127, (uint8_t)155, (uint8_t)233, (uint8_t)191, (uint8_t)49, (uint8_t)238, (uint8_t)75, (uint8_t)20, (uint8_t)162, (uint8_t)144, (uint8_t)34, (uint8_t)78, (uint8_t)110, (uint8_t)173, (uint8_t)132, (uint8_t)111, (uint8_t)62, (uint8_t)95, (uint8_t)169, (uint8_t)160, (uint8_t)124, (uint8_t)49, (uint8_t)103, (uint8_t)128, (uint8_t)240, (uint8_t)173, (uint8_t)253, (uint8_t)247, (uint8_t)135, (uint8_t)83, (uint8_t)85, (uint8_t)160, (uint8_t)230, (uint8_t)224, (uint8_t)193, (uint8_t)104, (uint8_t)95, (uint8_t)221, (uint8_t)57, (uint8_t)216, (uint8_t)9, (uint8_t)39, (uint8_t)176, (uint8_t)140, (uint8_t)94, (uint8_t)240, (uint8_t)169, (uint8_t)136, (uint8_t)211, (uint8_t)135, (uint8_t)200, (uint8_t)139, (uint8_t)13, (uint8_t)247, (uint8_t)236, (uint8_t)198, (uint8_t)229, (uint8_t)168, (uint8_t)215, (uint8_t)190, (uint8_t)172, (uint8_t)244, (uint8_t)128, (uint8_t)42, (uint8_t)217, (uint8_t)219, (uint8_t)35, (uint8_t)214, (uint8_t)126, (uint8_t)224, (uint8_t)51, (uint8_t)250, (uint8_t)109, (uint8_t)102, (uint8_t)97, (uint8_t)9, (uint8_t)45, (uint8_t)99, (uint8_t)141, (uint8_t)3, (uint8_t)101, (uint8_t)49, (uint8_t)113, (uint8_t)238, (uint8_t)101, (uint8_t)133, (uint8_t)105, (uint8_t)160, (uint8_t)68, (uint8_t)20, (uint8_t)194, (uint8_t)105, (uint8_t)19, (uint8_t)29, (uint8_t)224, (uint8_t)71, (uint8_t)115, (uint8_t)155, (uint8_t)154, (uint8_t)169, (uint8_t)146, (uint8_t)72, (uint8_t)71, (uint8_t)217, (uint8_t)46, (uint8_t)67, (uint8_t)104, (uint8_t)84, (uint8_t)102, (uint8_t)6, (uint8_t)64, (uint8_t)252, (uint8_t)98, (uint8_t)61, (uint8_t)224, (uint8_t)45, (uint8_t)144, (uint8_t)172, (uint8_t)229, (uint8_t)103, (uint8_t)150, (uint8_t)97, (uint8_t)74, (uint8_t)230, (uint8_t)215, (uint8_t)148, (uint8_t)84, (uint8_t)172, (uint8_t)118, (uint8_t)14, (uint8_t)209, (uint8_t)108, (uint8_t)52, (uint8_t)230, (uint8_t)18, (uint8_t)194, (uint8_t)253, (uint8_t)154, (uint8_t)26, (uint8_t)90, (uint8_t)48, (uint8_t)197, (uint8_t)113, (uint8_t)100, (uint8_t)30, (uint8_t)27, (uint8_t)219, (uint8_t)126, (uint8_t)14, (uint8_t)96, (uint8_t)0, (uint8_t)152, (uint8_t)239, (uint8_t)206, (uint8_t)201, (uint8_t)161, (uint8_t)200, (uint8_t)76, (uint8_t)84, (uint8_t)130, (uint8_t)201, (uint8_t)146, (uint8_t)205, (uint8_t)145, (uint8_t)244, (uint8_t)229, (uint8_t)210, (uint8_t)88, (uint8_t)119, (uint8_t)235, (uint8_t)225, (uint8_t)92, (uint8_t)65, (uint8_t)108, (uint8_t)255, (uint8_t)165, (uint8_t)102, (uint8_t)39, (uint8_t)241, (uint8_t)215, (uint8_t)44, (uint8_t)71, (uint8_t)110, (uint8_t)62, (uint8_t)179, (uint8_t)16, (uint8_t)113, (uint8_t)116, (uint8_t)144, (uint8_t)67, (uint8_t)167, (uint8_t)166, (uint8_t)3, (uint8_t)134, (uint8_t)177, (uint8_t)204, (uint8_t)249, (uint8_t)212, (uint8_t)223, (uint8_t)146, (uint8_t)128, (uint8_t)230, (uint8_t)41, (uint8_t)148, (uint8_t)162, (uint8_t)205, (uint8_t)37, (uint8_t)230, (uint8_t)108, (uint8_t)252, (uint8_t)221, (uint8_t)161, (uint8_t)204, (uint8_t)102, (uint8_t)197, (uint8_t)43, (uint8_t)175, (uint8_t)234, (uint8_t)182, (uint8_t)244, (uint8_t)193, (uint8_t)239, (uint8_t)108, (uint8_t)52, (uint8_t)39, (uint8_t)85, (uint8_t)255, (uint8_t)201, (uint8_t)30, (uint8_t)250, (uint8_t)107, (uint8_t)243, (uint8_t)103, (uint8_t)11, (uint8_t)97, (uint8_t)240, (uint8_t)245, (uint8_t)213, (uint8_t)187, (uint8_t)108, (uint8_t)81};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_length_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)11446, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_rotation_SET((uint16_t)(uint16_t)41232, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1589196946L, PH.base.pack) ;
        p269_framerate_SET((float) -2.6357056E38F, PH.base.pack) ;
        {
            char16_t* uri = u"dtiodoosnvrhrYzealsnuxbdicFlhlg";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_camera_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)64128, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)51696, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_bitrate_SET((uint32_t)328450502L, PH.base.pack) ;
        p270_framerate_SET((float) -1.6488021E38F, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)17688, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)62745, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)56672, PH.base.pack) ;
        {
            char16_t* uri = u"LktxzejawkcmszmcfrsrtmhujnmafeFnnzaieifQsdhfzlkdfdtCSasiilcuPpbRcRNsCyFJr";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_component_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"oRixcArijpkuqNvijwKgpunbdnybvzlccgfhiTwmdTgeaqjjfZcktsyjeeBfmo";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"e";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)38417, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)245, (uint8_t)135, (uint8_t)177, (uint8_t)124, (uint8_t)250, (uint8_t)195, (uint8_t)229, (uint8_t)223};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)48596, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)26866, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)237, (uint8_t)128, (uint8_t)165, (uint8_t)23, (uint8_t)29, (uint8_t)67, (uint8_t)230, (uint8_t)110};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)11118, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)4241906575056261761L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)3762083391L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_major_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)528307447190856753L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3623846736L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2245230888L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        {
            char16_t* name = u"jjlosewcuertvpzwumgtzgfipsjhilzzzqgyponhnqpovpxwhvspfygybn";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)20, (uint8_t)151, (uint8_t)87, (uint8_t)165, (uint8_t)170, (uint8_t)85, (uint8_t)211, (uint8_t)141, (uint8_t)5, (uint8_t)122, (uint8_t)151, (uint8_t)247, (uint8_t)45, (uint8_t)23, (uint8_t)147, (uint8_t)252};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)21214, PH.base.pack) ;
        {
            char16_t* param_id = u"Iwbjazmbwa";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_count_SET((uint16_t)(uint16_t)30930, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)6054, PH.base.pack) ;
        {
            char16_t* param_id = u"itN";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"zvxieakgrzFjgcekqucyZVnnmfyynlnrawkajeWthrFvybgWecwacmwmdeyPkzeebolumfxQtpgskcYoufzutTzYacadcHwr";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_id = u"syxgbsarse";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"Jumpbvnu";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"yoote";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"iaqg";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_min_distance_SET((uint16_t)(uint16_t)5234, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)41724, (uint16_t)62548, (uint16_t)31352, (uint16_t)39312, (uint16_t)17716, (uint16_t)41540, (uint16_t)62234, (uint16_t)59911, (uint16_t)28916, (uint16_t)60223, (uint16_t)60274, (uint16_t)10668, (uint16_t)15506, (uint16_t)36269, (uint16_t)59939, (uint16_t)11822, (uint16_t)3025, (uint16_t)15918, (uint16_t)62608, (uint16_t)37643, (uint16_t)56129, (uint16_t)21078, (uint16_t)13382, (uint16_t)44372, (uint16_t)59670, (uint16_t)43419, (uint16_t)15171, (uint16_t)65450, (uint16_t)44516, (uint16_t)36770, (uint16_t)58703, (uint16_t)35312, (uint16_t)39262, (uint16_t)7804, (uint16_t)21901, (uint16_t)36160, (uint16_t)55794, (uint16_t)32315, (uint16_t)17505, (uint16_t)35149, (uint16_t)57892, (uint16_t)21451, (uint16_t)19543, (uint16_t)45440, (uint16_t)43031, (uint16_t)10579, (uint16_t)55578, (uint16_t)10565, (uint16_t)26001, (uint16_t)13878, (uint16_t)20362, (uint16_t)56013, (uint16_t)38741, (uint16_t)25986, (uint16_t)33429, (uint16_t)64576, (uint16_t)32768, (uint16_t)21516, (uint16_t)21032, (uint16_t)29211, (uint16_t)49396, (uint16_t)21785, (uint16_t)9052, (uint16_t)26606, (uint16_t)4340, (uint16_t)33487, (uint16_t)15703, (uint16_t)1753, (uint16_t)22059, (uint16_t)38344, (uint16_t)46215, (uint16_t)2097};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_max_distance_SET((uint16_t)(uint16_t)29549, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)2308428759319616585L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
        p10001_ICAO_SET((uint32_t)3445797945L, PH.base.pack) ;
        p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, PH.base.pack) ;
        p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M, PH.base.pack) ;
        p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M, PH.base.pack) ;
        {
            char16_t* callsign = u"kytGun";
            p10001_callsign_SET_(callsign, &PH) ;
        }
        p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, PH.base.pack) ;
        p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, PH.base.pack) ;
        p10001_stallSpeed_SET((uint16_t)(uint16_t)62979, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
        p10002_utcTime_SET((uint32_t)2241811018L, PH.base.pack) ;
        p10002_VelEW_SET((int16_t)(int16_t)28491, PH.base.pack) ;
        p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY, PH.base.pack) ;
        p10002_gpsLat_SET((int32_t) -803107212, PH.base.pack) ;
        p10002_squawk_SET((uint16_t)(uint16_t)5601, PH.base.pack) ;
        p10002_accuracyVel_SET((uint16_t)(uint16_t)55560, PH.base.pack) ;
        p10002_accuracyHor_SET((uint32_t)1632152107L, PH.base.pack) ;
        p10002_velNS_SET((int16_t)(int16_t) -15467, PH.base.pack) ;
        p10002_velVert_SET((int16_t)(int16_t) -12708, PH.base.pack) ;
        p10002_accuracyVert_SET((uint16_t)(uint16_t)7711, PH.base.pack) ;
        p10002_baroAltMSL_SET((int32_t) -818346132, PH.base.pack) ;
        p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK, PH.base.pack) ;
        p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED, PH.base.pack) ;
        p10002_gpsAlt_SET((int32_t)5921046, PH.base.pack) ;
        p10002_numSats_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p10002_gpsLon_SET((int32_t)428171422, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(), &PH);
        p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


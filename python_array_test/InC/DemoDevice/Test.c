
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p0_custom_mode_GET(pack) == (uint32_t)184298100L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_PARAFOIL);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)44337);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)5851);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)10218);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)12522);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)58245);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)125);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)31872);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)34221);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)44474);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)58736);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2552315123L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2279738558827753448L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p3_y_GET(pack) == (float) -1.8444707E37F);
    assert(p3_vy_GET(pack) == (float)3.355583E38F);
    assert(p3_vx_GET(pack) == (float)3.2750672E38F);
    assert(p3_vz_GET(pack) == (float) -7.1766665E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1961281031L);
    assert(p3_yaw_rate_GET(pack) == (float)2.0963635E38F);
    assert(p3_afy_GET(pack) == (float)1.0247324E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)29263);
    assert(p3_x_GET(pack) == (float)1.3676993E38F);
    assert(p3_afx_GET(pack) == (float) -3.6901753E37F);
    assert(p3_afz_GET(pack) == (float) -9.346072E37F);
    assert(p3_yaw_GET(pack) == (float) -3.122238E38F);
    assert(p3_z_GET(pack) == (float) -3.8835064E37F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)1320198589L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p4_time_usec_GET(pack) == (uint64_t)2020876444672277531L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_passkey_LEN(ph) == 25);
    {
        char16_t * exemplary = u"amhrvUysfaxmlXbbobdxrgwlx";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)75);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 7);
    {
        char16_t * exemplary = u"gguHvcm";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)3732254066L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p20_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"nWhkmzZqfw";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)24524);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)97);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)4318);
    assert(p22_param_value_GET(pack) == (float)8.992899E37F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)10652);
    assert(p22_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"wjz";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p23_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"itPezbSBsqjmp";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p23_param_value_GET(pack) == (float) -3.338038E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)8893);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)27371);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1153457132L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)3466307624L);
    assert(p24_lat_GET(pack) == (int32_t) -183577078);
    assert(p24_time_usec_GET(pack) == (uint64_t)8319495582188055860L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)3682061049L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)4242);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)59004);
    assert(p24_lon_GET(pack) == (int32_t)1906199254);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_alt_GET(pack) == (int32_t)154647944);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)2714991281L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -1954955793);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)30);
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)188, (uint8_t)12, (uint8_t)25, (uint8_t)183, (uint8_t)227, (uint8_t)173, (uint8_t)116, (uint8_t)176, (uint8_t)24, (uint8_t)112, (uint8_t)219, (uint8_t)26, (uint8_t)173, (uint8_t)222, (uint8_t)147, (uint8_t)5, (uint8_t)216, (uint8_t)52, (uint8_t)105} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)31, (uint8_t)184, (uint8_t)124, (uint8_t)163, (uint8_t)22, (uint8_t)101, (uint8_t)159, (uint8_t)123, (uint8_t)161, (uint8_t)75, (uint8_t)172, (uint8_t)46, (uint8_t)56, (uint8_t)160, (uint8_t)29, (uint8_t)197, (uint8_t)173, (uint8_t)89, (uint8_t)204} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)213, (uint8_t)237, (uint8_t)246, (uint8_t)34, (uint8_t)236, (uint8_t)251, (uint8_t)179, (uint8_t)202, (uint8_t)136, (uint8_t)18, (uint8_t)200, (uint8_t)143, (uint8_t)138, (uint8_t)18, (uint8_t)66, (uint8_t)73, (uint8_t)171, (uint8_t)13, (uint8_t)220} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)149, (uint8_t)18, (uint8_t)17, (uint8_t)139, (uint8_t)151, (uint8_t)11, (uint8_t)27, (uint8_t)104, (uint8_t)71, (uint8_t)203, (uint8_t)72, (uint8_t)130, (uint8_t)139, (uint8_t)159, (uint8_t)218, (uint8_t)10, (uint8_t)22, (uint8_t)190, (uint8_t)235, (uint8_t)204} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)249, (uint8_t)90, (uint8_t)121, (uint8_t)101, (uint8_t)192, (uint8_t)64, (uint8_t)134, (uint8_t)105, (uint8_t)173, (uint8_t)32, (uint8_t)207, (uint8_t)156, (uint8_t)124, (uint8_t)98, (uint8_t)24, (uint8_t)165, (uint8_t)212, (uint8_t)81, (uint8_t)168, (uint8_t)211} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -3991);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)213780530L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)26357);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -5034);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -32601);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)8024);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -32611);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)29465);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)30769);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)5783);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)16290);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -793);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -32149);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -28099);
    assert(p27_time_usec_GET(pack) == (uint64_t)3445167811204283695L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)11123);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)2784);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)32423);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -18765);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -5592);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)7295918234406803521L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -31549);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -2171);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -6578);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)1905);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_diff_GET(pack) == (float) -5.9451505E37F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3521892411L);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -2163);
    assert(p29_press_abs_GET(pack) == (float)3.2243176E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float)4.7489098E36F);
    assert(p30_yawspeed_GET(pack) == (float)2.3006799E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)415076639L);
    assert(p30_pitchspeed_GET(pack) == (float)2.341393E38F);
    assert(p30_pitch_GET(pack) == (float) -3.6014443E37F);
    assert(p30_yaw_GET(pack) == (float) -2.3679326E38F);
    assert(p30_roll_GET(pack) == (float) -2.7450714E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_rollspeed_GET(pack) == (float)1.3657712E38F);
    assert(p31_q4_GET(pack) == (float) -2.4153297E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2051756946L);
    assert(p31_yawspeed_GET(pack) == (float) -1.8101093E38F);
    assert(p31_pitchspeed_GET(pack) == (float)2.68719E38F);
    assert(p31_q1_GET(pack) == (float) -3.0439437E38F);
    assert(p31_q2_GET(pack) == (float) -7.221792E37F);
    assert(p31_q3_GET(pack) == (float) -2.405194E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_y_GET(pack) == (float)1.2640851E38F);
    assert(p32_vy_GET(pack) == (float)3.177842E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1719995293L);
    assert(p32_vz_GET(pack) == (float) -1.802511E38F);
    assert(p32_vx_GET(pack) == (float) -1.6673352E38F);
    assert(p32_x_GET(pack) == (float)9.711358E37F);
    assert(p32_z_GET(pack) == (float) -2.2405187E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)23450);
    assert(p33_alt_GET(pack) == (int32_t) -690595929);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)903123258L);
    assert(p33_lat_GET(pack) == (int32_t)2126117360);
    assert(p33_relative_alt_GET(pack) == (int32_t)1898297063);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)27567);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)30021);
    assert(p33_lon_GET(pack) == (int32_t)1842457633);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -2818);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -16502);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)18895);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)317998694L);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -13690);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -707);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -15834);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)26402);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -11367);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)13253);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)169);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)26309);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)26909);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)49122);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1112302091L);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)13218);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)62805);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)33129);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)53603);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)4569);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)42507);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)25425);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)57280);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)15657);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)39790);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)55151);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)54219);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)14779);
    assert(p36_time_usec_GET(pack) == (uint32_t)4212164411L);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)52762);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)38226);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)15045);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)38761);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)45176);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)13427);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)5503);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)7714);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)5860);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)24681);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)17212);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -31397);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)38199);
    assert(p39_y_GET(pack) == (float)9.752141E37F);
    assert(p39_param3_GET(pack) == (float)2.5961991E38F);
    assert(p39_param1_GET(pack) == (float) -1.924048E37F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p39_param2_GET(pack) == (float) -3.187678E38F);
    assert(p39_param4_GET(pack) == (float)5.2771686E37F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p39_z_GET(pack) == (float) -7.8834795E37F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p39_x_GET(pack) == (float)1.1949789E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_SPATIAL_USER_1);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)54712);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)47227);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)74);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)60831);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)137);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)26158);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)53446);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_DENIED);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)52);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t) -1958375718);
    assert(p48_latitude_GET(pack) == (int32_t) -1231956917);
    assert(p48_time_usec_TRY(ph) == (uint64_t)5433765293434278445L);
    assert(p48_longitude_GET(pack) == (int32_t) -529140576);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)197);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)6467990883328548507L);
    assert(p49_latitude_GET(pack) == (int32_t) -1383741629);
    assert(p49_altitude_GET(pack) == (int32_t)1979946280);
    assert(p49_longitude_GET(pack) == (int32_t) -607592046);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -11155);
    assert(p50_param_value_max_GET(pack) == (float)2.3284725E38F);
    assert(p50_param_value_min_GET(pack) == (float) -2.7356452E38F);
    assert(p50_scale_GET(pack) == (float)5.2741313E37F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p50_param_value0_GET(pack) == (float) -1.5714171E38F);
    assert(p50_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"zsqfLm";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)18241);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p54_p1z_GET(pack) == (float)2.8186842E38F);
    assert(p54_p2z_GET(pack) == (float) -1.6268046E38F);
    assert(p54_p2y_GET(pack) == (float)3.7001824E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p54_p1y_GET(pack) == (float) -2.500045E38F);
    assert(p54_p1x_GET(pack) == (float) -2.1781059E38F);
    assert(p54_p2x_GET(pack) == (float)2.9238189E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)32);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float)2.8144178E38F);
    assert(p55_p2x_GET(pack) == (float) -6.964617E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p55_p2y_GET(pack) == (float)3.1776194E38F);
    assert(p55_p1z_GET(pack) == (float)2.350852E38F);
    assert(p55_p2z_GET(pack) == (float)8.3516645E37F);
    assert(p55_p1y_GET(pack) == (float) -1.2030537E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.0487484E38F, 1.5700071E38F, -6.853099E37F, 2.9847906E38F, 2.0949433E36F, -2.5233778E38F, -8.1740316E37F, -1.9685E38F, -1.1176586E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)6306638105032442683L);
    {
        float exemplary[] =  {-1.1947945E38F, 1.7486333E38F, 1.5463933E38F, 1.5928487E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -4.033196E37F);
    assert(p61_pitchspeed_GET(pack) == (float)6.9542724E37F);
    assert(p61_yawspeed_GET(pack) == (float) -1.5157994E37F);
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)25348);
    assert(p62_nav_pitch_GET(pack) == (float)5.515423E37F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -28119);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)24833);
    assert(p62_xtrack_error_GET(pack) == (float)3.009603E38F);
    assert(p62_aspd_error_GET(pack) == (float) -2.3458572E38F);
    assert(p62_alt_error_GET(pack) == (float)3.7862312E37F);
    assert(p62_nav_roll_GET(pack) == (float) -3.1549347E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vz_GET(pack) == (float) -2.7959803E38F);
    assert(p63_lon_GET(pack) == (int32_t) -1441668792);
    assert(p63_relative_alt_GET(pack) == (int32_t) -1938971262);
    assert(p63_lat_GET(pack) == (int32_t)1318290615);
    {
        float exemplary[] =  {-2.9964723E38F, -1.70171E38F, -3.0696782E38F, 1.4040336E37F, 7.2252824E37F, 3.188119E38F, -2.3127965E38F, -9.000442E37F, -3.078845E38F, -6.260935E37F, -1.4774072E38F, -2.3889214E38F, -2.592742E38F, -7.8900363E37F, 1.0241738E38F, -2.7887914E38F, 3.3943065E38F, -1.5500341E38F, -8.810487E37F, -1.28523E38F, 2.439279E38F, 4.50192E37F, 5.787117E37F, -5.1232677E37F, 1.6248863E38F, 1.2087478E38F, -1.6342212E38F, -1.3178036E38F, -2.8608653E38F, -5.8988894E37F, 2.2315407E38F, 8.902111E37F, -7.838258E37F, -1.9133216E38F, 2.2683873E38F, 2.4023419E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_time_usec_GET(pack) == (uint64_t)9009503613895826651L);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_vx_GET(pack) == (float) -1.0199118E38F);
    assert(p63_alt_GET(pack) == (int32_t)79314874);
    assert(p63_vy_GET(pack) == (float) -1.6488041E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_ay_GET(pack) == (float) -7.6730227E37F);
    assert(p64_vx_GET(pack) == (float)1.423285E38F);
    assert(p64_az_GET(pack) == (float)1.6112387E38F);
    assert(p64_ax_GET(pack) == (float) -2.9581049E38F);
    assert(p64_z_GET(pack) == (float) -2.8368848E38F);
    assert(p64_y_GET(pack) == (float)1.3751176E38F);
    assert(p64_x_GET(pack) == (float)2.0516674E37F);
    {
        float exemplary[] =  {-2.07983E38F, 1.746442E38F, 3.3696358E38F, 3.4010794E38F, 1.0797534E38F, 2.6759536E38F, -4.0495395E37F, 2.6002363E38F, 2.5975583E38F, -8.961078E37F, 2.4542355E38F, 3.2882562E37F, 7.864917E37F, 2.5093793E36F, 3.0233511E38F, 1.4311627E38F, 1.3553786E38F, -1.5742004E38F, -1.430415E38F, -7.758954E37F, -3.9298525E37F, -5.965118E37F, -6.4200504E37F, -1.5390408E38F, 3.05372E38F, -2.5885417E38F, 1.8400482E38F, -2.076366E38F, 2.404641E38F, -1.866247E38F, -1.5342695E38F, 2.6882869E37F, 1.1706791E38F, -1.822737E38F, 9.682143E37F, 2.998175E37F, 3.0644204E38F, -1.140123E38F, 7.6661885E37F, -2.9762016E38F, 1.8645084E38F, -9.776732E36F, 2.912632E38F, -4.106516E37F, -3.0343408E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vz_GET(pack) == (float) -4.706771E37F);
    assert(p64_vy_GET(pack) == (float)1.8198485E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_time_usec_GET(pack) == (uint64_t)6905901488268524191L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)28029);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)32478);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)15289);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)55610);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)8240);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2591288394L);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)8340);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11551);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)26531);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)24852);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)6227);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)15348);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)16148);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)5965);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)54511);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)30922);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)18801);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)28080);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)51401);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)53366);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)65236);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -31258);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)29761);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)30395);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)11388);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)6805);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)29509);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)13746);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)11061);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)33390);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)13539);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)3514);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)42294);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)58116);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float) -3.272787E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p73_z_GET(pack) == (float)1.6850296E38F);
    assert(p73_param1_GET(pack) == (float) -1.9704984E38F);
    assert(p73_y_GET(pack) == (int32_t) -1623503035);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)48578);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_MODE);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p73_param2_GET(pack) == (float)1.3940561E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_x_GET(pack) == (int32_t) -1603170470);
    assert(p73_param3_GET(pack) == (float) -6.9644354E37F);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)11424);
    assert(p74_alt_GET(pack) == (float) -9.024563E37F);
    assert(p74_airspeed_GET(pack) == (float)5.45719E37F);
    assert(p74_groundspeed_GET(pack) == (float)3.1759154E38F);
    assert(p74_climb_GET(pack) == (float)1.8752303E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)14210);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_z_GET(pack) == (float)2.6995983E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p75_param3_GET(pack) == (float)1.87522E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_MODE);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p75_y_GET(pack) == (int32_t) -1433460313);
    assert(p75_x_GET(pack) == (int32_t) -463853292);
    assert(p75_param2_GET(pack) == (float) -2.9690606E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p75_param1_GET(pack) == (float)3.0039924E38F);
    assert(p75_param4_GET(pack) == (float) -5.781586E37F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)213);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p76_param6_GET(pack) == (float)1.8383392E38F);
    assert(p76_param1_GET(pack) == (float)2.391765E38F);
    assert(p76_param4_GET(pack) == (float)4.9128737E37F);
    assert(p76_param3_GET(pack) == (float)3.1153984E38F);
    assert(p76_param5_GET(pack) == (float) -9.323963E37F);
    assert(p76_param2_GET(pack) == (float) -2.9351498E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p76_param7_GET(pack) == (float) -4.2231498E37F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)126);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_DENIED);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)220);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)159);
    assert(p77_result_param2_TRY(ph) == (int32_t)419485011);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float)3.3402837E38F);
    assert(p81_roll_GET(pack) == (float) -1.815447E38F);
    assert(p81_pitch_GET(pack) == (float) -2.780718E38F);
    assert(p81_yaw_GET(pack) == (float) -2.5572395E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)955267283L);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)168);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.7205252E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)6.4078607E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)4224173035L);
    {
        float exemplary[] =  {3.3574708E38F, -6.634659E37F, 1.2825042E38F, 1.3600473E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float) -3.270389E37F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p82_thrust_GET(pack) == (float)2.9086578E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float)1.7803992E37F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3627862708L);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p83_thrust_GET(pack) == (float)4.0675612E37F);
    assert(p83_body_roll_rate_GET(pack) == (float) -2.0769624E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)6.1145735E37F);
    {
        float exemplary[] =  {8.1305076E37F, -3.4834656E37F, -9.95716E37F, -6.6531526E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_GET(pack) == (float) -2.0028283E38F);
    assert(p84_afx_GET(pack) == (float) -1.849735E38F);
    assert(p84_z_GET(pack) == (float) -2.9868288E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p84_y_GET(pack) == (float)3.515688E36F);
    assert(p84_yaw_rate_GET(pack) == (float) -1.90365E38F);
    assert(p84_afz_GET(pack) == (float) -7.277603E37F);
    assert(p84_afy_GET(pack) == (float) -7.37306E37F);
    assert(p84_vx_GET(pack) == (float)2.461804E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)6592);
    assert(p84_vy_GET(pack) == (float)2.0559058E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2178676938L);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p84_x_GET(pack) == (float)7.701868E36F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p84_vz_GET(pack) == (float)1.6350337E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lat_int_GET(pack) == (int32_t) -1353990491);
    assert(p86_afy_GET(pack) == (float)2.902049E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p86_afz_GET(pack) == (float) -2.63269E38F);
    assert(p86_vz_GET(pack) == (float)1.0859401E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)58662);
    assert(p86_yaw_GET(pack) == (float)1.458137E38F);
    assert(p86_vy_GET(pack) == (float)2.7260964E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p86_afx_GET(pack) == (float)1.4919264E37F);
    assert(p86_alt_GET(pack) == (float)8.182368E37F);
    assert(p86_vx_GET(pack) == (float)3.2697398E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p86_lon_int_GET(pack) == (int32_t) -1094802344);
    assert(p86_yaw_rate_GET(pack) == (float)1.3960652E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2185977917L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lon_int_GET(pack) == (int32_t) -512926534);
    assert(p87_afz_GET(pack) == (float) -1.2463477E37F);
    assert(p87_vx_GET(pack) == (float) -1.6785394E38F);
    assert(p87_yaw_GET(pack) == (float) -2.6637018E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)36103);
    assert(p87_alt_GET(pack) == (float)2.9980166E38F);
    assert(p87_vy_GET(pack) == (float)4.011601E37F);
    assert(p87_afy_GET(pack) == (float)3.3538348E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -1.270991E38F);
    assert(p87_vz_GET(pack) == (float) -6.038733E37F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)3399169150L);
    assert(p87_afx_GET(pack) == (float) -3.0428981E38F);
    assert(p87_lat_int_GET(pack) == (int32_t)352716676);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_x_GET(pack) == (float)2.6938156E37F);
    assert(p89_yaw_GET(pack) == (float) -1.6150837E38F);
    assert(p89_y_GET(pack) == (float)2.8004047E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2056433888L);
    assert(p89_roll_GET(pack) == (float) -2.7465804E38F);
    assert(p89_pitch_GET(pack) == (float)7.1685956E37F);
    assert(p89_z_GET(pack) == (float)2.7644765E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -23368);
    assert(p90_time_usec_GET(pack) == (uint64_t)1215292360395745218L);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)9539);
    assert(p90_alt_GET(pack) == (int32_t) -1413700150);
    assert(p90_rollspeed_GET(pack) == (float)7.982009E37F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)16216);
    assert(p90_roll_GET(pack) == (float) -3.0510056E38F);
    assert(p90_yaw_GET(pack) == (float) -7.844748E37F);
    assert(p90_lat_GET(pack) == (int32_t) -1221436726);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)15885);
    assert(p90_lon_GET(pack) == (int32_t) -526082261);
    assert(p90_pitch_GET(pack) == (float) -2.5964871E38F);
    assert(p90_yawspeed_GET(pack) == (float)1.5819686E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -2008);
    assert(p90_pitchspeed_GET(pack) == (float)2.7628736E37F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -11658);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux2_GET(pack) == (float)1.782576E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p91_yaw_rudder_GET(pack) == (float) -2.5578116E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.171415E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)8730561058553656915L);
    assert(p91_aux3_GET(pack) == (float) -4.828447E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p91_aux1_GET(pack) == (float)3.2506689E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)6.550317E37F);
    assert(p91_aux4_GET(pack) == (float) -9.998156E36F);
    assert(p91_throttle_GET(pack) == (float) -5.73919E36F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)34602);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)18462);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)57549);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)56728);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p92_time_usec_GET(pack) == (uint64_t)8578799396806267938L);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)44903);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)29329);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)58671);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)25719);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)22518);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)47179);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)30189);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)44510);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.1460635E38F, 6.050361E37F, -6.450423E37F, 2.5336307E38F, -9.472211E37F, -3.1798673E36F, -3.4789053E37F, -3.3233489E38F, 3.699044E37F, -3.5849557E37F, -6.5716173E37F, -2.621653E38F, 1.8690167E38F, -4.7093696E37F, -2.2475642E38F, 5.1953945E37F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
    assert(p93_flags_GET(pack) == (uint64_t)168694075062650006L);
    assert(p93_time_usec_GET(pack) == (uint64_t)49833716346640058L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -22895);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.2916832E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)1.1781716E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p100_flow_comp_m_y_GET(pack) == (float)2.2742153E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)20463);
    assert(p100_flow_rate_x_TRY(ph) == (float) -2.6205897E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)577890460702558031L);
    assert(p100_ground_distance_GET(pack) == (float)2.2859878E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_z_GET(pack) == (float) -2.0796948E38F);
    assert(p101_pitch_GET(pack) == (float) -1.5551849E38F);
    assert(p101_yaw_GET(pack) == (float)1.7056847E38F);
    assert(p101_y_GET(pack) == (float)5.4613684E37F);
    assert(p101_x_GET(pack) == (float)3.3883343E38F);
    assert(p101_roll_GET(pack) == (float) -1.1584056E38F);
    assert(p101_usec_GET(pack) == (uint64_t)3656345359988369404L);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_usec_GET(pack) == (uint64_t)4864910255147853062L);
    assert(p102_z_GET(pack) == (float)1.8943773E38F);
    assert(p102_x_GET(pack) == (float) -1.4472943E38F);
    assert(p102_roll_GET(pack) == (float)7.2034955E37F);
    assert(p102_yaw_GET(pack) == (float) -1.870084E38F);
    assert(p102_y_GET(pack) == (float)4.0959185E37F);
    assert(p102_pitch_GET(pack) == (float) -7.1297674E37F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float)1.4500191E38F);
    assert(p103_x_GET(pack) == (float)3.196816E37F);
    assert(p103_usec_GET(pack) == (uint64_t)5388120042841254762L);
    assert(p103_y_GET(pack) == (float)2.321164E38F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float)1.3667108E38F);
    assert(p104_roll_GET(pack) == (float) -2.0740082E38F);
    assert(p104_y_GET(pack) == (float) -1.6190809E38F);
    assert(p104_x_GET(pack) == (float)1.1164233E38F);
    assert(p104_z_GET(pack) == (float)1.1665321E38F);
    assert(p104_pitch_GET(pack) == (float) -1.9158843E38F);
    assert(p104_usec_GET(pack) == (uint64_t)3609152004019200799L);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_temperature_GET(pack) == (float) -3.0288203E38F);
    assert(p105_xgyro_GET(pack) == (float) -6.1680755E37F);
    assert(p105_zacc_GET(pack) == (float) -2.4213276E38F);
    assert(p105_xacc_GET(pack) == (float)2.7571368E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4295216794757674738L);
    assert(p105_xmag_GET(pack) == (float)2.2578858E37F);
    assert(p105_ygyro_GET(pack) == (float) -1.0704826E37F);
    assert(p105_zmag_GET(pack) == (float) -1.7549225E38F);
    assert(p105_ymag_GET(pack) == (float)1.0682177E38F);
    assert(p105_abs_pressure_GET(pack) == (float)3.0428237E38F);
    assert(p105_yacc_GET(pack) == (float) -1.0591981E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)15159);
    assert(p105_pressure_alt_GET(pack) == (float)1.4163672E38F);
    assert(p105_zgyro_GET(pack) == (float) -6.0265646E37F);
    assert(p105_diff_pressure_GET(pack) == (float)9.899173E37F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_zgyro_GET(pack) == (float) -2.3582972E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)221965493L);
    assert(p106_integrated_y_GET(pack) == (float)1.3160767E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)12627);
    assert(p106_time_usec_GET(pack) == (uint64_t)7347305312564553915L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2898612392L);
    assert(p106_integrated_xgyro_GET(pack) == (float)6.123266E37F);
    assert(p106_distance_GET(pack) == (float)1.677659E38F);
    assert(p106_integrated_x_GET(pack) == (float)2.1305582E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p106_integrated_ygyro_GET(pack) == (float)1.0326875E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xgyro_GET(pack) == (float)1.6736438E38F);
    assert(p107_yacc_GET(pack) == (float)2.1900589E38F);
    assert(p107_xmag_GET(pack) == (float) -3.597683E36F);
    assert(p107_zacc_GET(pack) == (float)1.908543E38F);
    assert(p107_ymag_GET(pack) == (float)2.7615326E38F);
    assert(p107_ygyro_GET(pack) == (float) -2.3143791E38F);
    assert(p107_zmag_GET(pack) == (float) -2.8196806E37F);
    assert(p107_pressure_alt_GET(pack) == (float)6.179117E37F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.9562108E38F);
    assert(p107_zgyro_GET(pack) == (float) -1.4229941E38F);
    assert(p107_xacc_GET(pack) == (float) -1.8668873E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -5.440174E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)253825335358618503L);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2330668296L);
    assert(p107_temperature_GET(pack) == (float) -9.901184E37F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_ve_GET(pack) == (float) -2.2543115E38F);
    assert(p108_xacc_GET(pack) == (float) -7.581322E36F);
    assert(p108_q1_GET(pack) == (float) -7.9453977E37F);
    assert(p108_pitch_GET(pack) == (float) -9.22427E37F);
    assert(p108_yaw_GET(pack) == (float) -8.940691E37F);
    assert(p108_vd_GET(pack) == (float)2.4501625E38F);
    assert(p108_roll_GET(pack) == (float) -5.7321167E37F);
    assert(p108_q4_GET(pack) == (float) -3.3845478E38F);
    assert(p108_lon_GET(pack) == (float)2.6340765E38F);
    assert(p108_xgyro_GET(pack) == (float) -5.9008035E37F);
    assert(p108_q3_GET(pack) == (float) -2.6911478E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.7172928E38F);
    assert(p108_yacc_GET(pack) == (float) -2.4077135E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -1.9974048E38F);
    assert(p108_q2_GET(pack) == (float) -1.5890178E38F);
    assert(p108_zgyro_GET(pack) == (float) -7.2145023E37F);
    assert(p108_vn_GET(pack) == (float) -1.957746E38F);
    assert(p108_ygyro_GET(pack) == (float)2.8531734E38F);
    assert(p108_zacc_GET(pack) == (float)1.4740144E38F);
    assert(p108_lat_GET(pack) == (float) -3.3987948E38F);
    assert(p108_alt_GET(pack) == (float)1.192319E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)59238);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)54843);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)234, (uint8_t)74, (uint8_t)107, (uint8_t)101, (uint8_t)241, (uint8_t)249, (uint8_t)173, (uint8_t)26, (uint8_t)206, (uint8_t)43, (uint8_t)131, (uint8_t)210, (uint8_t)254, (uint8_t)133, (uint8_t)108, (uint8_t)224, (uint8_t)208, (uint8_t)34, (uint8_t)163, (uint8_t)232, (uint8_t)209, (uint8_t)242, (uint8_t)0, (uint8_t)91, (uint8_t)231, (uint8_t)36, (uint8_t)180, (uint8_t)92, (uint8_t)137, (uint8_t)0, (uint8_t)108, (uint8_t)89, (uint8_t)188, (uint8_t)44, (uint8_t)128, (uint8_t)138, (uint8_t)74, (uint8_t)84, (uint8_t)30, (uint8_t)141, (uint8_t)161, (uint8_t)57, (uint8_t)180, (uint8_t)53, (uint8_t)206, (uint8_t)150, (uint8_t)46, (uint8_t)146, (uint8_t)112, (uint8_t)84, (uint8_t)79, (uint8_t)190, (uint8_t)6, (uint8_t)171, (uint8_t)231, (uint8_t)111, (uint8_t)192, (uint8_t)236, (uint8_t)17, (uint8_t)210, (uint8_t)80, (uint8_t)133, (uint8_t)83, (uint8_t)192, (uint8_t)34, (uint8_t)170, (uint8_t)98, (uint8_t)69, (uint8_t)232, (uint8_t)28, (uint8_t)251, (uint8_t)254, (uint8_t)11, (uint8_t)131, (uint8_t)140, (uint8_t)213, (uint8_t)223, (uint8_t)84, (uint8_t)11, (uint8_t)93, (uint8_t)158, (uint8_t)90, (uint8_t)194, (uint8_t)217, (uint8_t)142, (uint8_t)85, (uint8_t)43, (uint8_t)200, (uint8_t)180, (uint8_t)71, (uint8_t)210, (uint8_t)51, (uint8_t)217, (uint8_t)164, (uint8_t)243, (uint8_t)53, (uint8_t)60, (uint8_t)42, (uint8_t)230, (uint8_t)224, (uint8_t)8, (uint8_t)179, (uint8_t)153, (uint8_t)241, (uint8_t)24, (uint8_t)82, (uint8_t)200, (uint8_t)119, (uint8_t)45, (uint8_t)6, (uint8_t)132, (uint8_t)94, (uint8_t)224, (uint8_t)101, (uint8_t)53, (uint8_t)46, (uint8_t)70, (uint8_t)146, (uint8_t)43, (uint8_t)77, (uint8_t)174, (uint8_t)83, (uint8_t)8, (uint8_t)167, (uint8_t)91, (uint8_t)194, (uint8_t)124, (uint8_t)170, (uint8_t)23, (uint8_t)160, (uint8_t)120, (uint8_t)0, (uint8_t)214, (uint8_t)29, (uint8_t)167, (uint8_t)244, (uint8_t)112, (uint8_t)22, (uint8_t)112, (uint8_t)68, (uint8_t)124, (uint8_t)2, (uint8_t)81, (uint8_t)127, (uint8_t)27, (uint8_t)225, (uint8_t)150, (uint8_t)99, (uint8_t)160, (uint8_t)169, (uint8_t)251, (uint8_t)235, (uint8_t)160, (uint8_t)191, (uint8_t)255, (uint8_t)119, (uint8_t)201, (uint8_t)206, (uint8_t)228, (uint8_t)225, (uint8_t)226, (uint8_t)31, (uint8_t)156, (uint8_t)157, (uint8_t)137, (uint8_t)198, (uint8_t)169, (uint8_t)179, (uint8_t)22, (uint8_t)107, (uint8_t)195, (uint8_t)91, (uint8_t)246, (uint8_t)238, (uint8_t)223, (uint8_t)177, (uint8_t)39, (uint8_t)113, (uint8_t)136, (uint8_t)168, (uint8_t)69, (uint8_t)128, (uint8_t)251, (uint8_t)227, (uint8_t)185, (uint8_t)14, (uint8_t)85, (uint8_t)176, (uint8_t)200, (uint8_t)172, (uint8_t)109, (uint8_t)200, (uint8_t)0, (uint8_t)28, (uint8_t)255, (uint8_t)201, (uint8_t)204, (uint8_t)192, (uint8_t)207, (uint8_t)140, (uint8_t)207, (uint8_t)255, (uint8_t)191, (uint8_t)219, (uint8_t)102, (uint8_t)125, (uint8_t)107, (uint8_t)52, (uint8_t)168, (uint8_t)69, (uint8_t)97, (uint8_t)87, (uint8_t)88, (uint8_t)23, (uint8_t)97, (uint8_t)253, (uint8_t)53, (uint8_t)56, (uint8_t)31, (uint8_t)194, (uint8_t)19, (uint8_t)201, (uint8_t)215, (uint8_t)33, (uint8_t)61, (uint8_t)37, (uint8_t)203, (uint8_t)242, (uint8_t)250, (uint8_t)44, (uint8_t)224, (uint8_t)151, (uint8_t)174, (uint8_t)12, (uint8_t)22, (uint8_t)177, (uint8_t)44, (uint8_t)202, (uint8_t)249, (uint8_t)154, (uint8_t)46, (uint8_t)113, (uint8_t)170, (uint8_t)134, (uint8_t)167, (uint8_t)113, (uint8_t)9, (uint8_t)40, (uint8_t)156, (uint8_t)27, (uint8_t)219} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)4406578966040699344L);
    assert(p111_ts1_GET(pack) == (int64_t)324528185307996431L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)437874409L);
    assert(p112_time_usec_GET(pack) == (uint64_t)5058394971118309688L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)35539);
    assert(p113_lon_GET(pack) == (int32_t)1126133904);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)32321);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)44727);
    assert(p113_time_usec_GET(pack) == (uint64_t)2437075640337640591L);
    assert(p113_alt_GET(pack) == (int32_t)1909407522);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -14578);
    assert(p113_lat_GET(pack) == (int32_t) -2036301055);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)24247);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)48483);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)9471);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)94);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integration_time_us_GET(pack) == (uint32_t)998444259L);
    assert(p114_integrated_zgyro_GET(pack) == (float)1.4973657E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p114_integrated_ygyro_GET(pack) == (float) -1.2112714E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float)1.7526983E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -4899);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)4113429182L);
    assert(p114_time_usec_GET(pack) == (uint64_t)3882103311772958860L);
    assert(p114_distance_GET(pack) == (float) -1.2397315E38F);
    assert(p114_integrated_x_GET(pack) == (float)1.5405808E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p114_integrated_y_GET(pack) == (float)2.5294027E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_lon_GET(pack) == (int32_t)2035973576);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)11866);
    assert(p115_pitchspeed_GET(pack) == (float)1.3080606E38F);
    {
        float exemplary[] =  {-1.00804964E37F, -2.7537864E38F, 3.3346403E37F, 3.191878E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)6285);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)59946);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -987);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -27523);
    assert(p115_time_usec_GET(pack) == (uint64_t)2564160551250285376L);
    assert(p115_rollspeed_GET(pack) == (float)2.8825734E38F);
    assert(p115_yawspeed_GET(pack) == (float)2.6786457E38F);
    assert(p115_alt_GET(pack) == (int32_t)2038110369);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)32086);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)23693);
    assert(p115_lat_GET(pack) == (int32_t) -865203578);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)23319);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -31576);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)32474);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -19649);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)7396);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2006244883L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)5379);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -31557);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -10784);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -16444);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)24027);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)30400);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)58092);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)4018541039L);
    assert(p118_time_utc_GET(pack) == (uint32_t)2751415951L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)307);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)27043);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)11144);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)56381);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p119_ofs_GET(pack) == (uint32_t)1812985061L);
    assert(p119_count_GET(pack) == (uint32_t)464275940L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)2498578782L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)48528);
    {
        uint8_t exemplary[] =  {(uint8_t)111, (uint8_t)255, (uint8_t)34, (uint8_t)197, (uint8_t)26, (uint8_t)101, (uint8_t)57, (uint8_t)149, (uint8_t)169, (uint8_t)200, (uint8_t)223, (uint8_t)68, (uint8_t)73, (uint8_t)167, (uint8_t)61, (uint8_t)59, (uint8_t)145, (uint8_t)98, (uint8_t)225, (uint8_t)171, (uint8_t)135, (uint8_t)37, (uint8_t)210, (uint8_t)224, (uint8_t)140, (uint8_t)212, (uint8_t)104, (uint8_t)154, (uint8_t)49, (uint8_t)101, (uint8_t)255, (uint8_t)219, (uint8_t)12, (uint8_t)100, (uint8_t)21, (uint8_t)20, (uint8_t)45, (uint8_t)146, (uint8_t)188, (uint8_t)207, (uint8_t)92, (uint8_t)96, (uint8_t)174, (uint8_t)155, (uint8_t)7, (uint8_t)234, (uint8_t)68, (uint8_t)175, (uint8_t)130, (uint8_t)197, (uint8_t)77, (uint8_t)105, (uint8_t)141, (uint8_t)238, (uint8_t)160, (uint8_t)172, (uint8_t)21, (uint8_t)95, (uint8_t)108, (uint8_t)206, (uint8_t)147, (uint8_t)18, (uint8_t)195, (uint8_t)98, (uint8_t)79, (uint8_t)46, (uint8_t)144, (uint8_t)168, (uint8_t)72, (uint8_t)250, (uint8_t)194, (uint8_t)56, (uint8_t)220, (uint8_t)136, (uint8_t)200, (uint8_t)36, (uint8_t)98, (uint8_t)247, (uint8_t)175, (uint8_t)106, (uint8_t)30, (uint8_t)231, (uint8_t)185, (uint8_t)109, (uint8_t)233, (uint8_t)46, (uint8_t)215, (uint8_t)1, (uint8_t)210, (uint8_t)156} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)12);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)214);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)168, (uint8_t)187, (uint8_t)3, (uint8_t)54, (uint8_t)54, (uint8_t)115, (uint8_t)238, (uint8_t)12, (uint8_t)44, (uint8_t)132, (uint8_t)109, (uint8_t)149, (uint8_t)47, (uint8_t)112, (uint8_t)244, (uint8_t)73, (uint8_t)49, (uint8_t)126, (uint8_t)2, (uint8_t)184, (uint8_t)142, (uint8_t)210, (uint8_t)164, (uint8_t)102, (uint8_t)191, (uint8_t)168, (uint8_t)67, (uint8_t)54, (uint8_t)187, (uint8_t)129, (uint8_t)243, (uint8_t)150, (uint8_t)60, (uint8_t)134, (uint8_t)55, (uint8_t)28, (uint8_t)174, (uint8_t)158, (uint8_t)212, (uint8_t)111, (uint8_t)47, (uint8_t)84, (uint8_t)153, (uint8_t)85, (uint8_t)55, (uint8_t)102, (uint8_t)217, (uint8_t)103, (uint8_t)161, (uint8_t)38, (uint8_t)134, (uint8_t)203, (uint8_t)222, (uint8_t)207, (uint8_t)138, (uint8_t)185, (uint8_t)205, (uint8_t)145, (uint8_t)132, (uint8_t)63, (uint8_t)11, (uint8_t)203, (uint8_t)18, (uint8_t)198, (uint8_t)65, (uint8_t)78, (uint8_t)19, (uint8_t)92, (uint8_t)213, (uint8_t)204, (uint8_t)84, (uint8_t)0, (uint8_t)113, (uint8_t)95, (uint8_t)254, (uint8_t)194, (uint8_t)119, (uint8_t)186, (uint8_t)100, (uint8_t)60, (uint8_t)87, (uint8_t)207, (uint8_t)130, (uint8_t)239, (uint8_t)170, (uint8_t)27, (uint8_t)1, (uint8_t)234, (uint8_t)67, (uint8_t)7, (uint8_t)118, (uint8_t)72, (uint8_t)228, (uint8_t)90, (uint8_t)138, (uint8_t)251, (uint8_t)79, (uint8_t)212, (uint8_t)86, (uint8_t)156, (uint8_t)157, (uint8_t)160, (uint8_t)166, (uint8_t)154, (uint8_t)47, (uint8_t)87, (uint8_t)56, (uint8_t)40, (uint8_t)204, (uint8_t)206} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)121);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)30487);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)60884);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)55804);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p124_lat_GET(pack) == (int32_t)1322850547);
    assert(p124_lon_GET(pack) == (int32_t) -1360564537);
    assert(p124_time_usec_GET(pack) == (uint64_t)9052834151693481692L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)8076);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1040243977L);
    assert(p124_alt_GET(pack) == (int32_t) -626716540);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)57513);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)7507);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)3678472208L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)4745);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND);
    {
        uint8_t exemplary[] =  {(uint8_t)222, (uint8_t)212, (uint8_t)53, (uint8_t)190, (uint8_t)121, (uint8_t)0, (uint8_t)186, (uint8_t)9, (uint8_t)202, (uint8_t)94, (uint8_t)85, (uint8_t)218, (uint8_t)117, (uint8_t)78, (uint8_t)137, (uint8_t)137, (uint8_t)150, (uint8_t)74, (uint8_t)224, (uint8_t)51, (uint8_t)149, (uint8_t)89, (uint8_t)156, (uint8_t)57, (uint8_t)69, (uint8_t)78, (uint8_t)148, (uint8_t)116, (uint8_t)215, (uint8_t)218, (uint8_t)234, (uint8_t)237, (uint8_t)161, (uint8_t)143, (uint8_t)202, (uint8_t)78, (uint8_t)108, (uint8_t)65, (uint8_t)50, (uint8_t)1, (uint8_t)207, (uint8_t)178, (uint8_t)185, (uint8_t)193, (uint8_t)145, (uint8_t)64, (uint8_t)24, (uint8_t)135, (uint8_t)247, (uint8_t)7, (uint8_t)168, (uint8_t)105, (uint8_t)241, (uint8_t)105, (uint8_t)67, (uint8_t)43, (uint8_t)115, (uint8_t)68, (uint8_t)8, (uint8_t)247, (uint8_t)121, (uint8_t)25, (uint8_t)219, (uint8_t)228, (uint8_t)96, (uint8_t)44, (uint8_t)183, (uint8_t)31, (uint8_t)15, (uint8_t)162} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1993486393);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)20938);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)4212498538L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -1751305681);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -786893060);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p127_accuracy_GET(pack) == (uint32_t)4179374820L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1487215922);
    assert(p127_tow_GET(pack) == (uint32_t)1033030650L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)96913701);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p128_accuracy_GET(pack) == (uint32_t)516360011L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -208129164);
    assert(p128_tow_GET(pack) == (uint32_t)58132851L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -403094357);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)1421188753L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)53345);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1177036880);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)170);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)13897);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)2285935416L);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)26387);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)3240);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)8014);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -29000);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -3312);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)22825);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -28060);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)14532);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)485);
    assert(p130_size_GET(pack) == (uint32_t)3345489078L);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)9931);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)30879);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)111, (uint8_t)101, (uint8_t)232, (uint8_t)155, (uint8_t)61, (uint8_t)134, (uint8_t)11, (uint8_t)42, (uint8_t)54, (uint8_t)136, (uint8_t)169, (uint8_t)95, (uint8_t)61, (uint8_t)65, (uint8_t)24, (uint8_t)160, (uint8_t)124, (uint8_t)19, (uint8_t)57, (uint8_t)14, (uint8_t)49, (uint8_t)159, (uint8_t)59, (uint8_t)235, (uint8_t)138, (uint8_t)26, (uint8_t)65, (uint8_t)95, (uint8_t)105, (uint8_t)228, (uint8_t)165, (uint8_t)195, (uint8_t)29, (uint8_t)215, (uint8_t)60, (uint8_t)197, (uint8_t)17, (uint8_t)34, (uint8_t)248, (uint8_t)228, (uint8_t)30, (uint8_t)0, (uint8_t)0, (uint8_t)139, (uint8_t)4, (uint8_t)125, (uint8_t)104, (uint8_t)147, (uint8_t)19, (uint8_t)218, (uint8_t)76, (uint8_t)176, (uint8_t)95, (uint8_t)160, (uint8_t)10, (uint8_t)43, (uint8_t)199, (uint8_t)209, (uint8_t)223, (uint8_t)77, (uint8_t)219, (uint8_t)19, (uint8_t)127, (uint8_t)209, (uint8_t)231, (uint8_t)228, (uint8_t)137, (uint8_t)233, (uint8_t)77, (uint8_t)15, (uint8_t)109, (uint8_t)224, (uint8_t)222, (uint8_t)73, (uint8_t)202, (uint8_t)230, (uint8_t)171, (uint8_t)26, (uint8_t)103, (uint8_t)253, (uint8_t)151, (uint8_t)250, (uint8_t)130, (uint8_t)231, (uint8_t)159, (uint8_t)128, (uint8_t)229, (uint8_t)247, (uint8_t)199, (uint8_t)90, (uint8_t)127, (uint8_t)55, (uint8_t)81, (uint8_t)180, (uint8_t)48, (uint8_t)38, (uint8_t)213, (uint8_t)198, (uint8_t)247, (uint8_t)219, (uint8_t)29, (uint8_t)80, (uint8_t)240, (uint8_t)40, (uint8_t)3, (uint8_t)27, (uint8_t)104, (uint8_t)244, (uint8_t)8, (uint8_t)133, (uint8_t)19, (uint8_t)98, (uint8_t)165, (uint8_t)52, (uint8_t)217, (uint8_t)129, (uint8_t)6, (uint8_t)126, (uint8_t)255, (uint8_t)220, (uint8_t)188, (uint8_t)52, (uint8_t)87, (uint8_t)210, (uint8_t)91, (uint8_t)155, (uint8_t)184, (uint8_t)178, (uint8_t)101, (uint8_t)129, (uint8_t)18, (uint8_t)254, (uint8_t)68, (uint8_t)61, (uint8_t)214, (uint8_t)85, (uint8_t)81, (uint8_t)22, (uint8_t)59, (uint8_t)76, (uint8_t)124, (uint8_t)116, (uint8_t)4, (uint8_t)101, (uint8_t)75, (uint8_t)140, (uint8_t)67, (uint8_t)103, (uint8_t)243, (uint8_t)96, (uint8_t)34, (uint8_t)57, (uint8_t)238, (uint8_t)52, (uint8_t)235, (uint8_t)120, (uint8_t)51, (uint8_t)212, (uint8_t)170, (uint8_t)47, (uint8_t)96, (uint8_t)242, (uint8_t)20, (uint8_t)157, (uint8_t)187, (uint8_t)206, (uint8_t)57, (uint8_t)252, (uint8_t)224, (uint8_t)137, (uint8_t)29, (uint8_t)112, (uint8_t)196, (uint8_t)14, (uint8_t)98, (uint8_t)67, (uint8_t)225, (uint8_t)250, (uint8_t)171, (uint8_t)86, (uint8_t)132, (uint8_t)199, (uint8_t)134, (uint8_t)119, (uint8_t)78, (uint8_t)97, (uint8_t)128, (uint8_t)78, (uint8_t)248, (uint8_t)34, (uint8_t)85, (uint8_t)198, (uint8_t)191, (uint8_t)163, (uint8_t)160, (uint8_t)196, (uint8_t)220, (uint8_t)105, (uint8_t)165, (uint8_t)24, (uint8_t)126, (uint8_t)208, (uint8_t)119, (uint8_t)233, (uint8_t)253, (uint8_t)3, (uint8_t)251, (uint8_t)253, (uint8_t)29, (uint8_t)150, (uint8_t)80, (uint8_t)211, (uint8_t)80, (uint8_t)181, (uint8_t)223, (uint8_t)76, (uint8_t)173, (uint8_t)92, (uint8_t)215, (uint8_t)21, (uint8_t)2, (uint8_t)41, (uint8_t)131, (uint8_t)91, (uint8_t)30, (uint8_t)131, (uint8_t)129, (uint8_t)138, (uint8_t)230, (uint8_t)11, (uint8_t)45, (uint8_t)229, (uint8_t)38, (uint8_t)46, (uint8_t)82, (uint8_t)75, (uint8_t)73, (uint8_t)112, (uint8_t)157, (uint8_t)53, (uint8_t)203, (uint8_t)132, (uint8_t)181, (uint8_t)7, (uint8_t)100, (uint8_t)160, (uint8_t)68, (uint8_t)42, (uint8_t)154, (uint8_t)96, (uint8_t)59, (uint8_t)252, (uint8_t)75} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)9748);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)1806);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)43393);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2556871790L);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)53683);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lon_GET(pack) == (int32_t) -239508316);
    assert(p133_lat_GET(pack) == (int32_t)1701585401);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)47097);
    assert(p133_mask_GET(pack) == (uint64_t)2469415147006425469L);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t)18055, (int16_t) -4873, (int16_t) -645, (int16_t)397, (int16_t) -20581, (int16_t)20427, (int16_t) -5100, (int16_t)16160, (int16_t) -12782, (int16_t) -32527, (int16_t) -14409, (int16_t) -32463, (int16_t) -18927, (int16_t) -20456, (int16_t) -6533, (int16_t) -24030} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p134_lat_GET(pack) == (int32_t)54815037);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)51495);
    assert(p134_lon_GET(pack) == (int32_t)1939081233);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)66304830);
    assert(p135_lat_GET(pack) == (int32_t)63915068);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float) -1.3636265E38F);
    assert(p136_lon_GET(pack) == (int32_t)616597511);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)29651);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)49268);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)2574);
    assert(p136_current_height_GET(pack) == (float)1.9297962E38F);
    assert(p136_lat_GET(pack) == (int32_t) -2098738873);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)409207858L);
    assert(p137_press_diff_GET(pack) == (float) -1.3526793E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)1656);
    assert(p137_press_abs_GET(pack) == (float)1.072452E38F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.5593295E38F, -2.952754E38F, 1.1108961E38F, 9.676067E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float) -2.501139E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)2568917624339497981L);
    assert(p138_y_GET(pack) == (float)3.0054748E38F);
    assert(p138_x_GET(pack) == (float)3.3744015E38F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)3863701341537056843L);
    {
        float exemplary[] =  {-1.5859447E38F, -1.4240247E37F, -7.3221273E37F, 5.816556E37F, 3.3034644E38F, -4.7016838E36F, 2.9042174E37F, -2.1428023E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)27);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p140_time_usec_GET(pack) == (uint64_t)4907995812894589616L);
    {
        float exemplary[] =  {2.7467441E38F, 2.2238946E38F, 2.5618215E38F, -3.1073682E38F, 1.1358218E38F, 1.0920187E38F, -3.2025943E38F, -9.063457E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_terrain_GET(pack) == (float)2.977244E38F);
    assert(p141_altitude_local_GET(pack) == (float) -4.343885E37F);
    assert(p141_altitude_relative_GET(pack) == (float) -2.7799833E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)5.0308773E37F);
    assert(p141_altitude_amsl_GET(pack) == (float)3.3379346E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)4831941631403013107L);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.4004804E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)138);
    {
        uint8_t exemplary[] =  {(uint8_t)250, (uint8_t)20, (uint8_t)36, (uint8_t)22, (uint8_t)174, (uint8_t)86, (uint8_t)9, (uint8_t)101, (uint8_t)236, (uint8_t)247, (uint8_t)135, (uint8_t)144, (uint8_t)246, (uint8_t)165, (uint8_t)78, (uint8_t)149, (uint8_t)229, (uint8_t)160, (uint8_t)204, (uint8_t)175, (uint8_t)142, (uint8_t)60, (uint8_t)218, (uint8_t)143, (uint8_t)7, (uint8_t)29, (uint8_t)86, (uint8_t)166, (uint8_t)35, (uint8_t)208, (uint8_t)117, (uint8_t)211, (uint8_t)11, (uint8_t)158, (uint8_t)26, (uint8_t)242, (uint8_t)158, (uint8_t)187, (uint8_t)103, (uint8_t)119, (uint8_t)117, (uint8_t)33, (uint8_t)122, (uint8_t)86, (uint8_t)107, (uint8_t)9, (uint8_t)12, (uint8_t)32, (uint8_t)128, (uint8_t)29, (uint8_t)170, (uint8_t)111, (uint8_t)17, (uint8_t)106, (uint8_t)227, (uint8_t)161, (uint8_t)213, (uint8_t)205, (uint8_t)120, (uint8_t)105, (uint8_t)4, (uint8_t)84, (uint8_t)240, (uint8_t)89, (uint8_t)58, (uint8_t)87, (uint8_t)63, (uint8_t)77, (uint8_t)73, (uint8_t)160, (uint8_t)166, (uint8_t)156, (uint8_t)101, (uint8_t)44, (uint8_t)32, (uint8_t)84, (uint8_t)166, (uint8_t)233, (uint8_t)214, (uint8_t)255, (uint8_t)37, (uint8_t)113, (uint8_t)71, (uint8_t)192, (uint8_t)177, (uint8_t)14, (uint8_t)143, (uint8_t)223, (uint8_t)214, (uint8_t)150, (uint8_t)16, (uint8_t)156, (uint8_t)182, (uint8_t)12, (uint8_t)247, (uint8_t)61, (uint8_t)142, (uint8_t)110, (uint8_t)176, (uint8_t)33, (uint8_t)144, (uint8_t)42, (uint8_t)132, (uint8_t)184, (uint8_t)201, (uint8_t)20, (uint8_t)195, (uint8_t)69, (uint8_t)108, (uint8_t)36, (uint8_t)243, (uint8_t)207, (uint8_t)158, (uint8_t)156, (uint8_t)172, (uint8_t)185, (uint8_t)135, (uint8_t)82, (uint8_t)0, (uint8_t)149} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)195, (uint8_t)71, (uint8_t)167, (uint8_t)47, (uint8_t)229, (uint8_t)67, (uint8_t)131, (uint8_t)87, (uint8_t)188, (uint8_t)162, (uint8_t)177, (uint8_t)8, (uint8_t)48, (uint8_t)234, (uint8_t)104, (uint8_t)33, (uint8_t)239, (uint8_t)218, (uint8_t)82, (uint8_t)231, (uint8_t)44, (uint8_t)0, (uint8_t)103, (uint8_t)159, (uint8_t)238, (uint8_t)77, (uint8_t)158, (uint8_t)143, (uint8_t)36, (uint8_t)224, (uint8_t)74, (uint8_t)112, (uint8_t)184, (uint8_t)114, (uint8_t)182, (uint8_t)138, (uint8_t)63, (uint8_t)191, (uint8_t)116, (uint8_t)209, (uint8_t)162, (uint8_t)181, (uint8_t)36, (uint8_t)48, (uint8_t)1, (uint8_t)183, (uint8_t)97, (uint8_t)81, (uint8_t)222, (uint8_t)100, (uint8_t)110, (uint8_t)92, (uint8_t)40, (uint8_t)160, (uint8_t)86, (uint8_t)76, (uint8_t)231, (uint8_t)37, (uint8_t)32, (uint8_t)84, (uint8_t)87, (uint8_t)161, (uint8_t)215, (uint8_t)123, (uint8_t)183, (uint8_t)224, (uint8_t)42, (uint8_t)117, (uint8_t)181, (uint8_t)246, (uint8_t)88, (uint8_t)15, (uint8_t)169, (uint8_t)96, (uint8_t)177, (uint8_t)23, (uint8_t)40, (uint8_t)85, (uint8_t)145, (uint8_t)141, (uint8_t)46, (uint8_t)96, (uint8_t)35, (uint8_t)53, (uint8_t)167, (uint8_t)126, (uint8_t)23, (uint8_t)193, (uint8_t)116, (uint8_t)142, (uint8_t)190, (uint8_t)146, (uint8_t)154, (uint8_t)150, (uint8_t)135, (uint8_t)230, (uint8_t)220, (uint8_t)232, (uint8_t)175, (uint8_t)168, (uint8_t)29, (uint8_t)143, (uint8_t)23, (uint8_t)143, (uint8_t)19, (uint8_t)254, (uint8_t)119, (uint8_t)217, (uint8_t)222, (uint8_t)240, (uint8_t)64, (uint8_t)188, (uint8_t)11, (uint8_t)47, (uint8_t)46, (uint8_t)22, (uint8_t)198, (uint8_t)7, (uint8_t)20} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float) -6.7409283E37F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3083435410L);
    assert(p143_press_diff_GET(pack) == (float) -2.1479976E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)30518);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.1560682E38F, -1.9940371E38F, -1.7369639E38F, -8.566889E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1233725301);
    assert(p144_custom_state_GET(pack) == (uint64_t)7611626468207190935L);
    {
        float exemplary[] =  {4.277534E37F, 1.9245123E38F, 3.0287014E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {9.772088E37F, 2.6677384E38F, -2.5802608E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -1381031714);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)71);
    {
        float exemplary[] =  {3.1998787E38F, 1.4189346E38F, 1.1006192E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)1.8822642E38F);
    assert(p144_timestamp_GET(pack) == (uint64_t)1027662723932484674L);
    {
        float exemplary[] =  {-6.2203727E37F, 1.2391125E38F, 1.9576032E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_pos_GET(pack) == (float)2.8958539E38F);
    assert(p146_airspeed_GET(pack) == (float) -8.295231E37F);
    assert(p146_roll_rate_GET(pack) == (float)2.5473128E38F);
    {
        float exemplary[] =  {2.9139796E37F, 3.9929627E37F, -1.0108796E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float) -2.5226515E38F);
    assert(p146_x_vel_GET(pack) == (float) -1.5234298E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -1.1209093E38F);
    assert(p146_x_acc_GET(pack) == (float)8.562535E37F);
    assert(p146_y_pos_GET(pack) == (float)2.18816E38F);
    assert(p146_y_acc_GET(pack) == (float)1.1658708E38F);
    {
        float exemplary[] =  {-1.6951348E38F, 1.481384E37F, 3.254648E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float) -2.8709264E38F);
    assert(p146_z_vel_GET(pack) == (float) -3.2774881E38F);
    assert(p146_z_pos_GET(pack) == (float) -1.4504239E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)4137036009367442065L);
    {
        float exemplary[] =  {-2.9168758E38F, 1.5810412E38F, 1.2234324E38F, 3.3707228E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float) -3.173627E38F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -32683);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -1681);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_energy_consumed_GET(pack) == (int32_t)946863824);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -122);
    assert(p147_current_consumed_GET(pack) == (int32_t)917116290);
    {
        uint16_t exemplary[] =  {(uint16_t)59731, (uint16_t)44355, (uint16_t)51226, (uint16_t)17151, (uint16_t)7097, (uint16_t)442, (uint16_t)6500, (uint16_t)61985, (uint16_t)25856, (uint16_t)12393} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)64, (uint8_t)219, (uint8_t)146, (uint8_t)118, (uint8_t)254, (uint8_t)197, (uint8_t)3, (uint8_t)43} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_os_sw_version_GET(pack) == (uint32_t)2232875071L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)13420);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3737757436L);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)41751);
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)123, (uint8_t)69, (uint8_t)12, (uint8_t)114, (uint8_t)86, (uint8_t)219, (uint8_t)114, (uint8_t)170, (uint8_t)28, (uint8_t)230, (uint8_t)225, (uint8_t)44, (uint8_t)27, (uint8_t)53, (uint8_t)34, (uint8_t)9, (uint8_t)112} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)177, (uint8_t)198, (uint8_t)103, (uint8_t)102, (uint8_t)88, (uint8_t)164, (uint8_t)213, (uint8_t)102} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)7028536797449197557L);
    assert(p148_board_version_GET(pack) == (uint32_t)2132612289L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1834184794L);
    {
        uint8_t exemplary[] =  {(uint8_t)173, (uint8_t)39, (uint8_t)75, (uint8_t)58, (uint8_t)177, (uint8_t)200, (uint8_t)171, (uint8_t)16} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_angle_x_GET(pack) == (float) -7.7403334E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p149_angle_y_GET(pack) == (float) -6.7220246E37F);
    assert(p149_size_x_GET(pack) == (float) -2.919885E38F);
    assert(p149_x_TRY(ph) == (float)3.3175824E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)4826716991536228529L);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p149_z_TRY(ph) == (float) -3.0467015E38F);
    {
        float exemplary[] =  {-1.0330466E38F, 7.550206E37F, 1.8218115E38F, 1.6707741E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_y_TRY(ph) == (float)3.240437E38F);
    assert(p149_size_y_GET(pack) == (float)8.662373E37F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)180);
    assert(p149_distance_GET(pack) == (float)1.7257708E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON);
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_0_150(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)49633, (uint16_t)25017, (uint16_t)54860, (uint16_t)13742} ;
        uint16_t*  sample = p150_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {1070103093L, 2573278646L, 3837541876L, 3678985243L} ;
        uint32_t*  sample = p150_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)158, (uint8_t)204, (uint8_t)48, (uint8_t)107} ;
        uint8_t*  sample = p150_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p150_v1_GET(pack) == (uint8_t)(uint8_t)178);
    {
        int8_t exemplary[] =  {(int8_t)91, (int8_t) -111, (int8_t)109, (int8_t)29} ;
        int8_t*  sample = p150_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_1_151(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {2559956767L, 1809613838L, 651767041L, 791995732L} ;
        uint32_t*  sample = p151_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_3_153(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {3058757168L, 934383469L, 2919727653L, 653147222L} ;
        uint32_t*  sample = p153_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p153_v_GET(pack) == (uint8_t)(uint8_t)224);
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_4_154(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {2275405475L, 2192856120L, 1685695046L, 2461895618L} ;
        uint32_t*  sample = p154_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p154_v_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_5_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_c1_LEN(ph) == 1);
    {
        char16_t * exemplary = u"p";
        char16_t * sample = p155_c1_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p155_c2_LEN(ph) == 1);
    {
        char16_t * exemplary = u"b";
        char16_t * sample = p155_c2_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_6_156(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -22, (int8_t)56} ;
        int8_t*  sample = p156_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)102, (uint8_t)218} ;
        uint8_t*  sample = p156_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)49006, (uint16_t)50737} ;
        uint16_t*  sample = p156_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {944175862L, 3191762652L} ;
        uint32_t*  sample = p156_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        double exemplary[] =  {1.7915369678372592E307, -8.957701084993507E307} ;
        double*  sample = p156_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_ar_c_LEN(ph) == 25);
    {
        char16_t * exemplary = u"jaHgnlvrqodqecpcggymqajUn";
        char16_t * sample = p156_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.359192E38F, -2.8764266E38F} ;
        float*  sample = p156_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t)22494, (int16_t)30641} ;
        int16_t*  sample = p156_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {-212590080, 52844573} ;
        int32_t*  sample = p156_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_v1_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p156_v3_GET(pack) == (uint32_t)1244075667L);
    assert(p156_v2_GET(pack) == (uint16_t)(uint16_t)2362);
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_7_157(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {1821499610L, 2981607307L} ;
        uint32_t*  sample = p157_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)32967, (uint16_t)57113} ;
        uint16_t*  sample = p157_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.0777216E38F, 2.8393305E38F} ;
        float*  sample = p157_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        double exemplary[] =  {-1.2869180416484966E308, -5.170226971360175E307} ;
        double*  sample = p157_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p157_ar_c_LEN(ph) == 5);
    {
        char16_t * exemplary = u"lgWze";
        char16_t * sample = p157_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int8_t exemplary[] =  {(int8_t) -98, (int8_t) -121} ;
        int8_t*  sample = p157_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {-2055682717, -93782493} ;
        int32_t*  sample = p157_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)117, (uint8_t)183} ;
        uint8_t*  sample = p157_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t) -647, (int16_t)16549} ;
        int16_t*  sample = p157_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ARRAY_TEST_8_158(Bounds_Inside * ph, Pack * pack)
{
    {
        double exemplary[] =  {1.771128595392106E308, -1.1811289097694477E308} ;
        double*  sample = p158_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)55297, (uint16_t)40354} ;
        uint16_t*  sample = p158_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p158_v3_GET(pack) == (uint32_t)1882202859L);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_vel_ratio_GET(pack) == (float)2.2527526E38F);
    assert(p230_tas_ratio_GET(pack) == (float)2.606916E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -2.187133E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -3.7206962E37F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.5244018E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -3.0794724E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH);
    assert(p230_mag_ratio_GET(pack) == (float) -1.4302764E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)7210861050112885033L);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -1.1257258E36F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_horiz_GET(pack) == (float) -7.455072E37F);
    assert(p231_var_vert_GET(pack) == (float) -3.0420344E37F);
    assert(p231_wind_z_GET(pack) == (float)2.1160208E38F);
    assert(p231_wind_alt_GET(pack) == (float)2.3766534E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)7147062452474096228L);
    assert(p231_vert_accuracy_GET(pack) == (float) -2.083737E37F);
    assert(p231_wind_y_GET(pack) == (float) -3.070166E38F);
    assert(p231_wind_x_GET(pack) == (float) -3.1457865E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -2.5385588E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_lat_GET(pack) == (int32_t) -1097047024);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p232_time_usec_GET(pack) == (uint64_t)7265021984978685924L);
    assert(p232_ve_GET(pack) == (float)9.775556E37F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)31980);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2759792741L);
    assert(p232_horiz_accuracy_GET(pack) == (float) -3.2703138E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT);
    assert(p232_vert_accuracy_GET(pack) == (float)3.6212258E35F);
    assert(p232_alt_GET(pack) == (float) -2.1943734E37F);
    assert(p232_lon_GET(pack) == (int32_t)1583100332);
    assert(p232_vdop_GET(pack) == (float) -1.7152412E38F);
    assert(p232_vd_GET(pack) == (float)3.1808927E38F);
    assert(p232_vn_GET(pack) == (float) -1.1732524E38F);
    assert(p232_hdop_GET(pack) == (float) -1.4263299E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p232_speed_accuracy_GET(pack) == (float)2.6892732E36F);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)227);
    {
        uint8_t exemplary[] =  {(uint8_t)92, (uint8_t)136, (uint8_t)63, (uint8_t)100, (uint8_t)177, (uint8_t)98, (uint8_t)252, (uint8_t)113, (uint8_t)241, (uint8_t)176, (uint8_t)111, (uint8_t)93, (uint8_t)196, (uint8_t)91, (uint8_t)89, (uint8_t)63, (uint8_t)117, (uint8_t)246, (uint8_t)162, (uint8_t)102, (uint8_t)75, (uint8_t)66, (uint8_t)241, (uint8_t)179, (uint8_t)233, (uint8_t)160, (uint8_t)244, (uint8_t)132, (uint8_t)224, (uint8_t)49, (uint8_t)211, (uint8_t)99, (uint8_t)143, (uint8_t)92, (uint8_t)52, (uint8_t)31, (uint8_t)233, (uint8_t)153, (uint8_t)175, (uint8_t)28, (uint8_t)105, (uint8_t)159, (uint8_t)33, (uint8_t)235, (uint8_t)254, (uint8_t)10, (uint8_t)121, (uint8_t)69, (uint8_t)106, (uint8_t)101, (uint8_t)143, (uint8_t)184, (uint8_t)142, (uint8_t)48, (uint8_t)23, (uint8_t)201, (uint8_t)178, (uint8_t)147, (uint8_t)200, (uint8_t)75, (uint8_t)3, (uint8_t)4, (uint8_t)133, (uint8_t)226, (uint8_t)142, (uint8_t)241, (uint8_t)155, (uint8_t)57, (uint8_t)29, (uint8_t)139, (uint8_t)115, (uint8_t)240, (uint8_t)140, (uint8_t)244, (uint8_t)12, (uint8_t)172, (uint8_t)48, (uint8_t)253, (uint8_t)85, (uint8_t)57, (uint8_t)44, (uint8_t)229, (uint8_t)236, (uint8_t)41, (uint8_t)9, (uint8_t)186, (uint8_t)153, (uint8_t)32, (uint8_t)46, (uint8_t)26, (uint8_t)83, (uint8_t)1, (uint8_t)210, (uint8_t)178, (uint8_t)104, (uint8_t)79, (uint8_t)137, (uint8_t)56, (uint8_t)81, (uint8_t)70, (uint8_t)122, (uint8_t)96, (uint8_t)255, (uint8_t)175, (uint8_t)95, (uint8_t)47, (uint8_t)67, (uint8_t)231, (uint8_t)232, (uint8_t)200, (uint8_t)120, (uint8_t)21, (uint8_t)166, (uint8_t)254, (uint8_t)129, (uint8_t)54, (uint8_t)176, (uint8_t)62, (uint8_t)120, (uint8_t)212, (uint8_t)60, (uint8_t)75, (uint8_t)81, (uint8_t)84, (uint8_t)79, (uint8_t)41, (uint8_t)130, (uint8_t)95, (uint8_t)38, (uint8_t)240, (uint8_t)31, (uint8_t)140, (uint8_t)89, (uint8_t)47, (uint8_t)85, (uint8_t)254, (uint8_t)96, (uint8_t)161, (uint8_t)46, (uint8_t)91, (uint8_t)50, (uint8_t)160, (uint8_t)77, (uint8_t)25, (uint8_t)171, (uint8_t)123, (uint8_t)63, (uint8_t)105, (uint8_t)245, (uint8_t)28, (uint8_t)36, (uint8_t)183, (uint8_t)206, (uint8_t)108, (uint8_t)0, (uint8_t)159, (uint8_t)18, (uint8_t)162, (uint8_t)40, (uint8_t)234, (uint8_t)103, (uint8_t)6, (uint8_t)240, (uint8_t)47, (uint8_t)67, (uint8_t)173, (uint8_t)171, (uint8_t)58, (uint8_t)226, (uint8_t)169, (uint8_t)200, (uint8_t)155, (uint8_t)190, (uint8_t)40, (uint8_t)59, (uint8_t)199, (uint8_t)138, (uint8_t)178, (uint8_t)67, (uint8_t)153} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)3345);
    assert(p234_latitude_GET(pack) == (int32_t)573416631);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)24404);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)5200);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)1267);
    assert(p234_longitude_GET(pack) == (int32_t) -1479518463);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -116);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)24759);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3803848859L);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)59918);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -70);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -43);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)19136);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -98);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_2_GET(pack) == (uint32_t)386306982L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)2510905696L);
    assert(p241_vibration_y_GET(pack) == (float)2.9600155E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3224094308L);
    assert(p241_vibration_x_GET(pack) == (float) -2.4007615E38F);
    assert(p241_vibration_z_GET(pack) == (float) -5.477924E37F);
    assert(p241_time_usec_GET(pack) == (uint64_t)7742573720835496418L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_latitude_GET(pack) == (int32_t)1544369251);
    assert(p242_z_GET(pack) == (float) -4.887864E37F);
    assert(p242_longitude_GET(pack) == (int32_t)1655230131);
    assert(p242_altitude_GET(pack) == (int32_t)2036810561);
    assert(p242_approach_x_GET(pack) == (float) -7.440782E37F);
    assert(p242_y_GET(pack) == (float)1.4409693E38F);
    assert(p242_x_GET(pack) == (float)1.4198509E38F);
    assert(p242_approach_y_GET(pack) == (float)1.8712407E38F);
    assert(p242_approach_z_GET(pack) == (float) -2.68449E38F);
    {
        float exemplary[] =  {-2.0932503E38F, -1.9481652E38F, -2.7686018E38F, -1.9022823E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_time_usec_TRY(ph) == (uint64_t)4386107775584825145L);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p243_longitude_GET(pack) == (int32_t)743907440);
    assert(p243_y_GET(pack) == (float)2.2656707E38F);
    assert(p243_approach_x_GET(pack) == (float)1.5302499E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)2128536564805702523L);
    assert(p243_latitude_GET(pack) == (int32_t) -490573467);
    assert(p243_approach_y_GET(pack) == (float) -3.0660059E38F);
    assert(p243_z_GET(pack) == (float) -1.0746817E38F);
    assert(p243_approach_z_GET(pack) == (float) -5.9370953E37F);
    {
        float exemplary[] =  {-4.8809913E36F, -2.476255E38F, -1.9203091E38F, 2.5105031E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_x_GET(pack) == (float)2.4213686E37F);
    assert(p243_altitude_GET(pack) == (int32_t)1268911772);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)627514319);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)55325);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
    assert(p246_lat_GET(pack) == (int32_t)1941730077);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)38134);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2789168372L);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)41468);
    assert(p246_altitude_GET(pack) == (int32_t)1393761085);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p246_lon_GET(pack) == (int32_t) -2089112037);
    assert(p246_callsign_LEN(ph) == 7);
    {
        char16_t * exemplary = u"uxiheqq";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)21783);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)7451);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)2610360599L);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -3.2800356E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -2.2732352E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.421376E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)160, (uint8_t)67, (uint8_t)93, (uint8_t)91, (uint8_t)199, (uint8_t)119, (uint8_t)122, (uint8_t)64, (uint8_t)46, (uint8_t)17, (uint8_t)37, (uint8_t)34, (uint8_t)165, (uint8_t)214, (uint8_t)216, (uint8_t)77, (uint8_t)110, (uint8_t)251, (uint8_t)112, (uint8_t)75, (uint8_t)255, (uint8_t)149, (uint8_t)244, (uint8_t)131, (uint8_t)228, (uint8_t)212, (uint8_t)173, (uint8_t)230, (uint8_t)132, (uint8_t)6, (uint8_t)20, (uint8_t)72, (uint8_t)11, (uint8_t)201, (uint8_t)11, (uint8_t)85, (uint8_t)45, (uint8_t)35, (uint8_t)11, (uint8_t)223, (uint8_t)117, (uint8_t)191, (uint8_t)189, (uint8_t)78, (uint8_t)6, (uint8_t)47, (uint8_t)52, (uint8_t)198, (uint8_t)122, (uint8_t)114, (uint8_t)59, (uint8_t)22, (uint8_t)146, (uint8_t)151, (uint8_t)236, (uint8_t)23, (uint8_t)129, (uint8_t)196, (uint8_t)119, (uint8_t)143, (uint8_t)235, (uint8_t)48, (uint8_t)142, (uint8_t)206, (uint8_t)57, (uint8_t)99, (uint8_t)200, (uint8_t)142, (uint8_t)196, (uint8_t)197, (uint8_t)65, (uint8_t)204, (uint8_t)224, (uint8_t)41, (uint8_t)155, (uint8_t)223, (uint8_t)137, (uint8_t)54, (uint8_t)62, (uint8_t)201, (uint8_t)183, (uint8_t)107, (uint8_t)112, (uint8_t)222, (uint8_t)235, (uint8_t)167, (uint8_t)186, (uint8_t)166, (uint8_t)49, (uint8_t)49, (uint8_t)216, (uint8_t)0, (uint8_t)250, (uint8_t)217, (uint8_t)209, (uint8_t)83, (uint8_t)191, (uint8_t)242, (uint8_t)217, (uint8_t)136, (uint8_t)42, (uint8_t)139, (uint8_t)149, (uint8_t)166, (uint8_t)22, (uint8_t)200, (uint8_t)104, (uint8_t)177, (uint8_t)18, (uint8_t)220, (uint8_t)204, (uint8_t)170, (uint8_t)180, (uint8_t)186, (uint8_t)132, (uint8_t)233, (uint8_t)155, (uint8_t)210, (uint8_t)233, (uint8_t)105, (uint8_t)189, (uint8_t)104, (uint8_t)92, (uint8_t)178, (uint8_t)193, (uint8_t)150, (uint8_t)110, (uint8_t)130, (uint8_t)63, (uint8_t)166, (uint8_t)220, (uint8_t)34, (uint8_t)236, (uint8_t)21, (uint8_t)137, (uint8_t)151, (uint8_t)186, (uint8_t)71, (uint8_t)150, (uint8_t)19, (uint8_t)48, (uint8_t)17, (uint8_t)136, (uint8_t)137, (uint8_t)239, (uint8_t)237, (uint8_t)63, (uint8_t)242, (uint8_t)226, (uint8_t)202, (uint8_t)9, (uint8_t)247, (uint8_t)99, (uint8_t)4, (uint8_t)65, (uint8_t)142, (uint8_t)153, (uint8_t)121, (uint8_t)127, (uint8_t)105, (uint8_t)147, (uint8_t)234, (uint8_t)112, (uint8_t)73, (uint8_t)68, (uint8_t)164, (uint8_t)214, (uint8_t)218, (uint8_t)169, (uint8_t)77, (uint8_t)56, (uint8_t)191, (uint8_t)145, (uint8_t)63, (uint8_t)173, (uint8_t)200, (uint8_t)166, (uint8_t)211, (uint8_t)237, (uint8_t)60, (uint8_t)144, (uint8_t)213, (uint8_t)236, (uint8_t)178, (uint8_t)227, (uint8_t)231, (uint8_t)127, (uint8_t)117, (uint8_t)210, (uint8_t)111, (uint8_t)27, (uint8_t)17, (uint8_t)51, (uint8_t)111, (uint8_t)136, (uint8_t)74, (uint8_t)167, (uint8_t)2, (uint8_t)165, (uint8_t)33, (uint8_t)224, (uint8_t)35, (uint8_t)51, (uint8_t)136, (uint8_t)216, (uint8_t)170, (uint8_t)242, (uint8_t)88, (uint8_t)245, (uint8_t)123, (uint8_t)23, (uint8_t)1, (uint8_t)82, (uint8_t)131, (uint8_t)2, (uint8_t)178, (uint8_t)151, (uint8_t)7, (uint8_t)58, (uint8_t)155, (uint8_t)212, (uint8_t)98, (uint8_t)111, (uint8_t)151, (uint8_t)165, (uint8_t)184, (uint8_t)40, (uint8_t)209, (uint8_t)163, (uint8_t)117, (uint8_t)120, (uint8_t)154, (uint8_t)247, (uint8_t)45, (uint8_t)129, (uint8_t)180, (uint8_t)179, (uint8_t)19, (uint8_t)9, (uint8_t)111, (uint8_t)241, (uint8_t)124, (uint8_t)159, (uint8_t)209, (uint8_t)6, (uint8_t)253, (uint8_t)107, (uint8_t)194, (uint8_t)149} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)30133);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -65, (int8_t)94, (int8_t)27, (int8_t)6, (int8_t) -18, (int8_t)59, (int8_t)14, (int8_t)41, (int8_t) -105, (int8_t)72, (int8_t)46, (int8_t)121, (int8_t) -53, (int8_t) -22, (int8_t) -81, (int8_t)72, (int8_t)99, (int8_t) -72, (int8_t) -43, (int8_t)82, (int8_t)92, (int8_t)109, (int8_t)109, (int8_t) -23, (int8_t)58, (int8_t)54, (int8_t)65, (int8_t)35, (int8_t)79, (int8_t)32, (int8_t)59, (int8_t) -24} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)30556);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"uGltjgawp";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)4321880139608474991L);
    assert(p250_y_GET(pack) == (float)3.272026E38F);
    assert(p250_z_GET(pack) == (float) -1.7058105E38F);
    assert(p250_x_GET(pack) == (float) -1.6519366E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 7);
    {
        char16_t * exemplary = u"mxmnxfJ";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)308342692L);
    assert(p251_value_GET(pack) == (float) -2.988002E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"gyezkual";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)729674006L);
    assert(p252_value_GET(pack) == (int32_t) -326393567);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 43);
    {
        char16_t * exemplary = u"nKpymcvkaimvijwgvedaueibmapjzemBbmwuqgvtqco";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 86);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p254_value_GET(pack) == (float) -2.033366E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)4256164049L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)6495626601099691128L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)102);
    {
        uint8_t exemplary[] =  {(uint8_t)129, (uint8_t)136, (uint8_t)62, (uint8_t)216, (uint8_t)25, (uint8_t)176, (uint8_t)92, (uint8_t)240, (uint8_t)36, (uint8_t)95, (uint8_t)203, (uint8_t)143, (uint8_t)59, (uint8_t)231, (uint8_t)188, (uint8_t)64, (uint8_t)30, (uint8_t)252, (uint8_t)150, (uint8_t)41, (uint8_t)29, (uint8_t)122, (uint8_t)22, (uint8_t)166, (uint8_t)28, (uint8_t)106, (uint8_t)198, (uint8_t)201, (uint8_t)241, (uint8_t)38, (uint8_t)100, (uint8_t)35} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2474077378L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)2459288216L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p258_tune_LEN(ph) == 12);
    {
        char16_t * exemplary = u"pfdwaaftEmux";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_h_GET(pack) == (float)1.0271349E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)46310);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)51271);
    assert(p259_sensor_size_v_GET(pack) == (float)3.390025E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)33, (uint8_t)158, (uint8_t)208, (uint8_t)62, (uint8_t)142, (uint8_t)109, (uint8_t)107, (uint8_t)11, (uint8_t)18, (uint8_t)95, (uint8_t)62, (uint8_t)2, (uint8_t)13, (uint8_t)28, (uint8_t)52, (uint8_t)117, (uint8_t)143, (uint8_t)113, (uint8_t)205, (uint8_t)196, (uint8_t)150, (uint8_t)94, (uint8_t)236, (uint8_t)46, (uint8_t)170, (uint8_t)131, (uint8_t)209, (uint8_t)75, (uint8_t)87, (uint8_t)63, (uint8_t)81, (uint8_t)151} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1999852364L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
    assert(p259_cam_definition_uri_LEN(ph) == 38);
    {
        char16_t * exemplary = u"ohubidjGsBihMkjgiHtofOqkgyxjppBrgaOrlk";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 76);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float) -5.0221503E37F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)14542);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1990059433L);
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)35, (uint8_t)24, (uint8_t)71, (uint8_t)208, (uint8_t)208, (uint8_t)90, (uint8_t)63, (uint8_t)218, (uint8_t)207, (uint8_t)15, (uint8_t)240, (uint8_t)184, (uint8_t)217, (uint8_t)95, (uint8_t)178, (uint8_t)108, (uint8_t)11, (uint8_t)7, (uint8_t)51, (uint8_t)50, (uint8_t)83, (uint8_t)19, (uint8_t)232, (uint8_t)235, (uint8_t)39, (uint8_t)90, (uint8_t)136, (uint8_t)141, (uint8_t)35, (uint8_t)254, (uint8_t)15} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)2513621686L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p261_used_capacity_GET(pack) == (float) -1.4259675E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)4021656444L);
    assert(p261_write_speed_GET(pack) == (float) -1.9443896E38F);
    assert(p261_total_capacity_GET(pack) == (float) -2.0573385E38F);
    assert(p261_available_capacity_GET(pack) == (float) -9.061751E37F);
    assert(p261_read_speed_GET(pack) == (float)2.9980789E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float) -2.9164125E37F);
    assert(p262_available_capacity_GET(pack) == (float)3.1788111E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)1261418963L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)779200727L);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_file_url_LEN(ph) == 181);
    {
        char16_t * exemplary = u"eevnvzvfntgkywxehtusRxohbeTeltxbexhfsygmgvgxbygwDlxfyfdfpnzQPlzulofcigqsgrrgiezslwfmrOBficrljurYwyxdsrpwyLxgdxybebVqwdjWtpkizOvHqbPtXsruIwezUYgYeceubqezuminddutdmhgXldyukkrnriytrybe";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 362);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t)1991231729);
    assert(p263_image_index_GET(pack) == (int32_t)543732767);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -124);
    assert(p263_lon_GET(pack) == (int32_t) -212802312);
    assert(p263_relative_alt_GET(pack) == (int32_t)1472755315);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)582492926L);
    {
        float exemplary[] =  {6.968933E36F, 2.5623782E38F, -1.2017734E38F, -8.678639E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_utc_GET(pack) == (uint64_t)3102988958579850518L);
    assert(p263_lat_GET(pack) == (int32_t) -2016475026);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)21);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)6143084963745794454L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)1838320977141573217L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)981807421516801206L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2048609765L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float)3.269327E38F);
    assert(p265_roll_GET(pack) == (float) -8.803456E35F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)723413627L);
    assert(p265_yaw_GET(pack) == (float)2.4084603E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)150);
    {
        uint8_t exemplary[] =  {(uint8_t)186, (uint8_t)234, (uint8_t)34, (uint8_t)229, (uint8_t)203, (uint8_t)146, (uint8_t)3, (uint8_t)53, (uint8_t)229, (uint8_t)195, (uint8_t)195, (uint8_t)186, (uint8_t)92, (uint8_t)71, (uint8_t)106, (uint8_t)174, (uint8_t)225, (uint8_t)203, (uint8_t)228, (uint8_t)113, (uint8_t)246, (uint8_t)130, (uint8_t)20, (uint8_t)161, (uint8_t)178, (uint8_t)8, (uint8_t)251, (uint8_t)72, (uint8_t)125, (uint8_t)179, (uint8_t)68, (uint8_t)53, (uint8_t)56, (uint8_t)164, (uint8_t)211, (uint8_t)78, (uint8_t)119, (uint8_t)251, (uint8_t)197, (uint8_t)54, (uint8_t)117, (uint8_t)91, (uint8_t)251, (uint8_t)111, (uint8_t)73, (uint8_t)98, (uint8_t)25, (uint8_t)100, (uint8_t)75, (uint8_t)128, (uint8_t)249, (uint8_t)52, (uint8_t)6, (uint8_t)252, (uint8_t)10, (uint8_t)191, (uint8_t)26, (uint8_t)112, (uint8_t)176, (uint8_t)149, (uint8_t)139, (uint8_t)135, (uint8_t)223, (uint8_t)160, (uint8_t)133, (uint8_t)2, (uint8_t)8, (uint8_t)145, (uint8_t)92, (uint8_t)222, (uint8_t)10, (uint8_t)149, (uint8_t)129, (uint8_t)178, (uint8_t)119, (uint8_t)61, (uint8_t)138, (uint8_t)26, (uint8_t)194, (uint8_t)6, (uint8_t)20, (uint8_t)213, (uint8_t)250, (uint8_t)112, (uint8_t)225, (uint8_t)150, (uint8_t)53, (uint8_t)101, (uint8_t)26, (uint8_t)85, (uint8_t)226, (uint8_t)95, (uint8_t)69, (uint8_t)99, (uint8_t)106, (uint8_t)2, (uint8_t)197, (uint8_t)83, (uint8_t)44, (uint8_t)168, (uint8_t)218, (uint8_t)223, (uint8_t)169, (uint8_t)94, (uint8_t)96, (uint8_t)172, (uint8_t)152, (uint8_t)130, (uint8_t)28, (uint8_t)191, (uint8_t)52, (uint8_t)197, (uint8_t)192, (uint8_t)249, (uint8_t)124, (uint8_t)231, (uint8_t)102, (uint8_t)43, (uint8_t)4, (uint8_t)253, (uint8_t)61, (uint8_t)165, (uint8_t)167, (uint8_t)148, (uint8_t)197, (uint8_t)63, (uint8_t)119, (uint8_t)222, (uint8_t)56, (uint8_t)19, (uint8_t)88, (uint8_t)79, (uint8_t)130, (uint8_t)120, (uint8_t)58, (uint8_t)203, (uint8_t)128, (uint8_t)15, (uint8_t)178, (uint8_t)23, (uint8_t)210, (uint8_t)112, (uint8_t)63, (uint8_t)237, (uint8_t)100, (uint8_t)240, (uint8_t)119, (uint8_t)217, (uint8_t)191, (uint8_t)85, (uint8_t)141, (uint8_t)60, (uint8_t)166, (uint8_t)100, (uint8_t)158, (uint8_t)227, (uint8_t)53, (uint8_t)128, (uint8_t)7, (uint8_t)190, (uint8_t)220, (uint8_t)68, (uint8_t)215, (uint8_t)136, (uint8_t)15, (uint8_t)231, (uint8_t)56, (uint8_t)39, (uint8_t)199, (uint8_t)195, (uint8_t)140, (uint8_t)17, (uint8_t)86, (uint8_t)232, (uint8_t)177, (uint8_t)203, (uint8_t)205, (uint8_t)230, (uint8_t)138, (uint8_t)235, (uint8_t)166, (uint8_t)24, (uint8_t)220, (uint8_t)163, (uint8_t)193, (uint8_t)195, (uint8_t)82, (uint8_t)78, (uint8_t)141, (uint8_t)99, (uint8_t)69, (uint8_t)182, (uint8_t)17, (uint8_t)141, (uint8_t)54, (uint8_t)84, (uint8_t)99, (uint8_t)167, (uint8_t)230, (uint8_t)115, (uint8_t)76, (uint8_t)171, (uint8_t)132, (uint8_t)147, (uint8_t)71, (uint8_t)86, (uint8_t)167, (uint8_t)179, (uint8_t)244, (uint8_t)36, (uint8_t)79, (uint8_t)191, (uint8_t)247, (uint8_t)23, (uint8_t)56, (uint8_t)67, (uint8_t)51, (uint8_t)71, (uint8_t)3, (uint8_t)128, (uint8_t)27, (uint8_t)99, (uint8_t)51, (uint8_t)215, (uint8_t)164, (uint8_t)24, (uint8_t)100, (uint8_t)88, (uint8_t)158, (uint8_t)77, (uint8_t)51, (uint8_t)78, (uint8_t)59, (uint8_t)109, (uint8_t)167, (uint8_t)223, (uint8_t)164, (uint8_t)193, (uint8_t)75, (uint8_t)168, (uint8_t)213, (uint8_t)242, (uint8_t)142, (uint8_t)224, (uint8_t)180, (uint8_t)77, (uint8_t)60, (uint8_t)116, (uint8_t)26} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)20746);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)195);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)58139);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)188);
    {
        uint8_t exemplary[] =  {(uint8_t)246, (uint8_t)238, (uint8_t)93, (uint8_t)228, (uint8_t)144, (uint8_t)2, (uint8_t)170, (uint8_t)252, (uint8_t)57, (uint8_t)209, (uint8_t)114, (uint8_t)20, (uint8_t)118, (uint8_t)194, (uint8_t)10, (uint8_t)52, (uint8_t)39, (uint8_t)14, (uint8_t)80, (uint8_t)62, (uint8_t)30, (uint8_t)189, (uint8_t)175, (uint8_t)41, (uint8_t)174, (uint8_t)203, (uint8_t)213, (uint8_t)239, (uint8_t)123, (uint8_t)88, (uint8_t)219, (uint8_t)135, (uint8_t)227, (uint8_t)61, (uint8_t)231, (uint8_t)66, (uint8_t)111, (uint8_t)39, (uint8_t)234, (uint8_t)14, (uint8_t)62, (uint8_t)129, (uint8_t)152, (uint8_t)166, (uint8_t)154, (uint8_t)255, (uint8_t)69, (uint8_t)101, (uint8_t)30, (uint8_t)50, (uint8_t)1, (uint8_t)142, (uint8_t)19, (uint8_t)255, (uint8_t)144, (uint8_t)42, (uint8_t)89, (uint8_t)208, (uint8_t)164, (uint8_t)232, (uint8_t)121, (uint8_t)207, (uint8_t)243, (uint8_t)225, (uint8_t)49, (uint8_t)172, (uint8_t)224, (uint8_t)107, (uint8_t)75, (uint8_t)109, (uint8_t)7, (uint8_t)19, (uint8_t)76, (uint8_t)116, (uint8_t)68, (uint8_t)64, (uint8_t)25, (uint8_t)144, (uint8_t)217, (uint8_t)255, (uint8_t)212, (uint8_t)65, (uint8_t)111, (uint8_t)184, (uint8_t)63, (uint8_t)210, (uint8_t)54, (uint8_t)58, (uint8_t)5, (uint8_t)208, (uint8_t)6, (uint8_t)10, (uint8_t)112, (uint8_t)168, (uint8_t)174, (uint8_t)241, (uint8_t)217, (uint8_t)8, (uint8_t)164, (uint8_t)39, (uint8_t)3, (uint8_t)22, (uint8_t)242, (uint8_t)72, (uint8_t)191, (uint8_t)137, (uint8_t)166, (uint8_t)11, (uint8_t)32, (uint8_t)54, (uint8_t)58, (uint8_t)197, (uint8_t)187, (uint8_t)14, (uint8_t)203, (uint8_t)246, (uint8_t)23, (uint8_t)149, (uint8_t)206, (uint8_t)162, (uint8_t)97, (uint8_t)252, (uint8_t)141, (uint8_t)201, (uint8_t)186, (uint8_t)217, (uint8_t)105, (uint8_t)194, (uint8_t)187, (uint8_t)48, (uint8_t)16, (uint8_t)175, (uint8_t)148, (uint8_t)106, (uint8_t)40, (uint8_t)115, (uint8_t)82, (uint8_t)8, (uint8_t)239, (uint8_t)193, (uint8_t)27, (uint8_t)219, (uint8_t)109, (uint8_t)85, (uint8_t)63, (uint8_t)47, (uint8_t)63, (uint8_t)178, (uint8_t)34, (uint8_t)166, (uint8_t)225, (uint8_t)117, (uint8_t)110, (uint8_t)53, (uint8_t)17, (uint8_t)149, (uint8_t)121, (uint8_t)34, (uint8_t)198, (uint8_t)175, (uint8_t)35, (uint8_t)146, (uint8_t)225, (uint8_t)221, (uint8_t)72, (uint8_t)169, (uint8_t)71, (uint8_t)240, (uint8_t)252, (uint8_t)216, (uint8_t)120, (uint8_t)67, (uint8_t)210, (uint8_t)84, (uint8_t)30, (uint8_t)126, (uint8_t)195, (uint8_t)32, (uint8_t)204, (uint8_t)217, (uint8_t)161, (uint8_t)84, (uint8_t)44, (uint8_t)19, (uint8_t)190, (uint8_t)120, (uint8_t)23, (uint8_t)247, (uint8_t)182, (uint8_t)75, (uint8_t)60, (uint8_t)60, (uint8_t)0, (uint8_t)141, (uint8_t)98, (uint8_t)158, (uint8_t)163, (uint8_t)217, (uint8_t)41, (uint8_t)231, (uint8_t)237, (uint8_t)32, (uint8_t)59, (uint8_t)187, (uint8_t)175, (uint8_t)115, (uint8_t)95, (uint8_t)161, (uint8_t)72, (uint8_t)95, (uint8_t)228, (uint8_t)111, (uint8_t)216, (uint8_t)84, (uint8_t)129, (uint8_t)66, (uint8_t)45, (uint8_t)98, (uint8_t)235, (uint8_t)68, (uint8_t)19, (uint8_t)93, (uint8_t)191, (uint8_t)40, (uint8_t)68, (uint8_t)192, (uint8_t)251, (uint8_t)185, (uint8_t)250, (uint8_t)221, (uint8_t)22, (uint8_t)247, (uint8_t)2, (uint8_t)22, (uint8_t)55, (uint8_t)28, (uint8_t)248, (uint8_t)156, (uint8_t)165, (uint8_t)161, (uint8_t)90, (uint8_t)0, (uint8_t)38, (uint8_t)100, (uint8_t)71, (uint8_t)210, (uint8_t)29, (uint8_t)86, (uint8_t)53} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)14295);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float) -3.063106E38F);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)63010);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)12122);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)62303);
    assert(p269_bitrate_GET(pack) == (uint32_t)2548008596L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p269_uri_LEN(ph) == 33);
    {
        char16_t * exemplary = u"dyauKztwsPmqouidhksvsrlstvzErnakT";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 66);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)42040);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)20457);
    assert(p270_bitrate_GET(pack) == (uint32_t)3672179374L);
    assert(p270_uri_LEN(ph) == 126);
    {
        char16_t * exemplary = u"klridftCmnmirniPsrqfhIhmecOdslzklgmetsusyrbazleehjyqhmsvgksbcwhntkdbkURzzvbvgicgejiheiivyctcDkfkuVnzpxklcljlnykrdzeynUqszCbnba";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 252);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p270_framerate_GET(pack) == (float)8.747865E37F);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)17814);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 32);
    {
        char16_t * exemplary = u"zAWzlpurtxsdwrZuykxuaspeldkBmbkh";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 6);
    {
        char16_t * exemplary = u"uyitJK";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)2221);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)22722);
    {
        uint8_t exemplary[] =  {(uint8_t)81, (uint8_t)172, (uint8_t)247, (uint8_t)141, (uint8_t)210, (uint8_t)33, (uint8_t)7, (uint8_t)197} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)53779);
    {
        uint8_t exemplary[] =  {(uint8_t)33, (uint8_t)54, (uint8_t)49, (uint8_t)214, (uint8_t)9, (uint8_t)235, (uint8_t)58, (uint8_t)84} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_time_usec_GET(pack) == (uint64_t)3222877728636035996L);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)4189471863L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)35545);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)102);
    {
        uint8_t exemplary[] =  {(uint8_t)111, (uint8_t)208, (uint8_t)131, (uint8_t)76, (uint8_t)82, (uint8_t)233, (uint8_t)11, (uint8_t)206, (uint8_t)51, (uint8_t)85, (uint8_t)178, (uint8_t)220, (uint8_t)223, (uint8_t)244, (uint8_t)109, (uint8_t)192} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p311_time_usec_GET(pack) == (uint64_t)4791133795981892936L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)728438091L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)1350655072L);
    assert(p311_name_LEN(ph) == 42);
    {
        char16_t * exemplary = u"evxbfpzhnwgVkpgiRowdahcseahkjSYjSnjwsxuIiZ";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 84);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)9502);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p320_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"m";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)97);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)12335);
    assert(p322_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"vviouowba";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p322_param_value_LEN(ph) == 26);
    {
        char16_t * exemplary = u"oetgpejNpaShahipugCstagsrm";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 52);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)2049);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p323_param_value_LEN(ph) == 120);
    {
        char16_t * exemplary = u"rkrzbgmbmhXyofawawDfQqrhLrdShzvSdrarkdztxvydxldzhxxuydwDbzxugvgHgaFqyhfVudkStjmgihoosvpknbljdkoJbBmmReioiahtfzjzxqqIxgor";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 240);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p323_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"wi";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
    assert(p324_param_value_LEN(ph) == 90);
    {
        char16_t * exemplary = u"jUhiktttkcxClmaspcesgnkszsdoclKehtoqvolOdouentskIdtnhqpsazsmzNsazbzttmiqqTxrxWuQztxadxmqdp";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"fsjfpXQwmOrdk";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)23851);
    {
        uint16_t exemplary[] =  {(uint16_t)55123, (uint16_t)44277, (uint16_t)211, (uint16_t)48585, (uint16_t)7237, (uint16_t)36613, (uint16_t)64924, (uint16_t)2496, (uint16_t)32268, (uint16_t)26335, (uint16_t)53036, (uint16_t)31792, (uint16_t)17002, (uint16_t)12014, (uint16_t)1647, (uint16_t)47043, (uint16_t)22510, (uint16_t)32039, (uint16_t)57609, (uint16_t)36266, (uint16_t)41034, (uint16_t)41796, (uint16_t)27013, (uint16_t)40828, (uint16_t)14809, (uint16_t)31541, (uint16_t)26461, (uint16_t)14540, (uint16_t)38560, (uint16_t)11479, (uint16_t)43362, (uint16_t)31478, (uint16_t)56895, (uint16_t)45366, (uint16_t)47978, (uint16_t)20850, (uint16_t)13086, (uint16_t)33587, (uint16_t)59578, (uint16_t)27927, (uint16_t)6812, (uint16_t)38871, (uint16_t)15458, (uint16_t)55970, (uint16_t)57929, (uint16_t)40098, (uint16_t)17076, (uint16_t)63355, (uint16_t)59032, (uint16_t)58918, (uint16_t)60275, (uint16_t)9323, (uint16_t)5992, (uint16_t)8322, (uint16_t)57663, (uint16_t)39373, (uint16_t)36332, (uint16_t)24966, (uint16_t)55321, (uint16_t)22690, (uint16_t)6052, (uint16_t)24363, (uint16_t)34215, (uint16_t)50788, (uint16_t)28650, (uint16_t)47473, (uint16_t)23643, (uint16_t)52530, (uint16_t)60806, (uint16_t)57343, (uint16_t)9284, (uint16_t)59276} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)1629726689801356760L);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)61745);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
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
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)184298100L, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_PARAFOIL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_battery_remaining_SET((int8_t)(int8_t)125, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)44474, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)5851, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)10218, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)12522, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)31872, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)58245, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)44337, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)58736, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)34221, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2279738558827753448L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)2552315123L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_SET((float) -3.122238E38F, PH.base.pack) ;
        p3_afz_SET((float) -9.346072E37F, PH.base.pack) ;
        p3_afx_SET((float) -3.6901753E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float)2.0963635E38F, PH.base.pack) ;
        p3_y_SET((float) -1.8444707E37F, PH.base.pack) ;
        p3_x_SET((float)1.3676993E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p3_afy_SET((float)1.0247324E38F, PH.base.pack) ;
        p3_z_SET((float) -3.8835064E37F, PH.base.pack) ;
        p3_vx_SET((float)3.2750672E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)29263, PH.base.pack) ;
        p3_vy_SET((float)3.355583E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1961281031L, PH.base.pack) ;
        p3_vz_SET((float) -7.1766665E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p4_seq_SET((uint32_t)1320198589L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)2020876444672277531L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"amhrvUysfaxmlXbbobdxrgwlx";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"gguHvcm";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)3732254066L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"nWhkmzZqfw";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)24524, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_value_SET((float)8.992899E37F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)4318, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)10652, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_id = u"wjz";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        {
            char16_t* param_id = u"itPezbSBsqjmp";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p23_param_value_SET((float) -3.338038E38F, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_hdg_acc_SET((uint32_t)1153457132L, &PH) ;
        p24_v_acc_SET((uint32_t)3466307624L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)8893, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)59004, PH.base.pack) ;
        p24_lon_SET((int32_t)1906199254, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)3682061049L, &PH) ;
        p24_lat_SET((int32_t) -183577078, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)8319495582188055860L, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)27371, PH.base.pack) ;
        p24_alt_SET((int32_t)154647944, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)2714991281L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)4242, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -1954955793, &PH) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)149, (uint8_t)18, (uint8_t)17, (uint8_t)139, (uint8_t)151, (uint8_t)11, (uint8_t)27, (uint8_t)104, (uint8_t)71, (uint8_t)203, (uint8_t)72, (uint8_t)130, (uint8_t)139, (uint8_t)159, (uint8_t)218, (uint8_t)10, (uint8_t)22, (uint8_t)190, (uint8_t)235, (uint8_t)204};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            uint8_t satellite_used[] =  {(uint8_t)88, (uint8_t)188, (uint8_t)12, (uint8_t)25, (uint8_t)183, (uint8_t)227, (uint8_t)173, (uint8_t)116, (uint8_t)176, (uint8_t)24, (uint8_t)112, (uint8_t)219, (uint8_t)26, (uint8_t)173, (uint8_t)222, (uint8_t)147, (uint8_t)5, (uint8_t)216, (uint8_t)52, (uint8_t)105};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)40, (uint8_t)213, (uint8_t)237, (uint8_t)246, (uint8_t)34, (uint8_t)236, (uint8_t)251, (uint8_t)179, (uint8_t)202, (uint8_t)136, (uint8_t)18, (uint8_t)200, (uint8_t)143, (uint8_t)138, (uint8_t)18, (uint8_t)66, (uint8_t)73, (uint8_t)171, (uint8_t)13, (uint8_t)220};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)179, (uint8_t)31, (uint8_t)184, (uint8_t)124, (uint8_t)163, (uint8_t)22, (uint8_t)101, (uint8_t)159, (uint8_t)123, (uint8_t)161, (uint8_t)75, (uint8_t)172, (uint8_t)46, (uint8_t)56, (uint8_t)160, (uint8_t)29, (uint8_t)197, (uint8_t)173, (uint8_t)89, (uint8_t)204};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)249, (uint8_t)90, (uint8_t)121, (uint8_t)101, (uint8_t)192, (uint8_t)64, (uint8_t)134, (uint8_t)105, (uint8_t)173, (uint8_t)32, (uint8_t)207, (uint8_t)156, (uint8_t)124, (uint8_t)98, (uint8_t)24, (uint8_t)165, (uint8_t)212, (uint8_t)81, (uint8_t)168, (uint8_t)211};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_yacc_SET((int16_t)(int16_t)8024, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)5783, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -32601, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -32611, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)213780530L, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)29465, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)30769, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -3991, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -5034, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)26357, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_zmag_SET((int16_t)(int16_t) -5592, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -28099, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)3445167811204283695L, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -18765, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)11123, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)2784, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)16290, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)32423, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -32149, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -793, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_abs_SET((int16_t)(int16_t) -31549, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -6578, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)1905, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -2171, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)7295918234406803521L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)3.2243176E38F, PH.base.pack) ;
        p29_press_diff_SET((float) -5.9451505E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3521892411L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -2163, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_yawspeed_SET((float)2.3006799E37F, PH.base.pack) ;
        p30_rollspeed_SET((float)4.7489098E36F, PH.base.pack) ;
        p30_pitch_SET((float) -3.6014443E37F, PH.base.pack) ;
        p30_pitchspeed_SET((float)2.341393E38F, PH.base.pack) ;
        p30_roll_SET((float) -2.7450714E38F, PH.base.pack) ;
        p30_yaw_SET((float) -2.3679326E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)415076639L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float)2.68719E38F, PH.base.pack) ;
        p31_q2_SET((float) -7.221792E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2051756946L, PH.base.pack) ;
        p31_rollspeed_SET((float)1.3657712E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -1.8101093E38F, PH.base.pack) ;
        p31_q4_SET((float) -2.4153297E38F, PH.base.pack) ;
        p31_q1_SET((float) -3.0439437E38F, PH.base.pack) ;
        p31_q3_SET((float) -2.405194E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)1719995293L, PH.base.pack) ;
        p32_x_SET((float)9.711358E37F, PH.base.pack) ;
        p32_vx_SET((float) -1.6673352E38F, PH.base.pack) ;
        p32_z_SET((float) -2.2405187E38F, PH.base.pack) ;
        p32_vz_SET((float) -1.802511E38F, PH.base.pack) ;
        p32_vy_SET((float)3.177842E38F, PH.base.pack) ;
        p32_y_SET((float)1.2640851E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t)1898297063, PH.base.pack) ;
        p33_lon_SET((int32_t)1842457633, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)27567, PH.base.pack) ;
        p33_lat_SET((int32_t)2126117360, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)30021, PH.base.pack) ;
        p33_alt_SET((int32_t) -690595929, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)903123258L, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -2818, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)23450, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan2_scaled_SET((int16_t)(int16_t) -15834, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -11367, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -16502, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -707, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)26402, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -13690, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)18895, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)13253, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)317998694L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_port_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)26309, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1112302091L, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)13218, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)49122, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)4569, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)33129, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)26909, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)53603, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)62805, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo7_raw_SET((uint16_t)(uint16_t)55151, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)15045, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)57280, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)38761, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)45176, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)13427, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)39790, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)42507, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)52762, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)7714, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)15657, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)4212164411L, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)38226, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)54219, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)14779, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)25425, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)5503, &PH) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_end_index_SET((int16_t)(int16_t)24681, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)5860, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)17212, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -31397, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_command_SET(e_MAV_CMD_MAV_CMD_SPATIAL_USER_1, PH.base.pack) ;
        p39_param3_SET((float)2.5961991E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p39_param4_SET((float)5.2771686E37F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p39_y_SET((float)9.752141E37F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p39_x_SET((float)1.1949789E38F, PH.base.pack) ;
        p39_param1_SET((float) -1.924048E37F, PH.base.pack) ;
        p39_param2_SET((float) -3.187678E38F, PH.base.pack) ;
        p39_z_SET((float) -7.8834795E37F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)38199, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)54712, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)47227, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)60831, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)26158, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)53446, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_DENIED, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)5433765293434278445L, &PH) ;
        p48_longitude_SET((int32_t) -529140576, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1958375718, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1231956917, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_longitude_SET((int32_t) -607592046, PH.base.pack) ;
        p49_altitude_SET((int32_t)1979946280, PH.base.pack) ;
        p49_latitude_SET((int32_t) -1383741629, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)6467990883328548507L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_scale_SET((float)5.2741313E37F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -11155, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p50_param_value_min_SET((float) -2.7356452E38F, PH.base.pack) ;
        p50_param_value0_SET((float) -1.5714171E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"zsqfLm";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p50_param_value_max_SET((float)2.3284725E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)18241, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1y_SET((float) -2.500045E38F, PH.base.pack) ;
        p54_p1x_SET((float) -2.1781059E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p54_p1z_SET((float)2.8186842E38F, PH.base.pack) ;
        p54_p2z_SET((float) -1.6268046E38F, PH.base.pack) ;
        p54_p2y_SET((float)3.7001824E37F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p54_p2x_SET((float)2.9238189E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float)2.8144178E38F, PH.base.pack) ;
        p55_p2z_SET((float)8.3516645E37F, PH.base.pack) ;
        p55_p1y_SET((float) -1.2030537E38F, PH.base.pack) ;
        p55_p1z_SET((float)2.350852E38F, PH.base.pack) ;
        p55_p2y_SET((float)3.1776194E38F, PH.base.pack) ;
        p55_p2x_SET((float) -6.964617E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float) -4.033196E37F, PH.base.pack) ;
        p61_pitchspeed_SET((float)6.9542724E37F, PH.base.pack) ;
        {
            float q[] =  {-1.1947945E38F, 1.7486333E38F, 1.5463933E38F, 1.5928487E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float covariance[] =  {1.0487484E38F, 1.5700071E38F, -6.853099E37F, 2.9847906E38F, 2.0949433E36F, -2.5233778E38F, -8.1740316E37F, -1.9685E38F, -1.1176586E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)6306638105032442683L, PH.base.pack) ;
        p61_yawspeed_SET((float) -1.5157994E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_alt_error_SET((float)3.7862312E37F, PH.base.pack) ;
        p62_nav_roll_SET((float) -3.1549347E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)5.515423E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float)3.009603E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -28119, PH.base.pack) ;
        p62_aspd_error_SET((float) -2.3458572E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)24833, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)25348, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vz_SET((float) -2.7959803E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)9009503613895826651L, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p63_vx_SET((float) -1.0199118E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.9964723E38F, -1.70171E38F, -3.0696782E38F, 1.4040336E37F, 7.2252824E37F, 3.188119E38F, -2.3127965E38F, -9.000442E37F, -3.078845E38F, -6.260935E37F, -1.4774072E38F, -2.3889214E38F, -2.592742E38F, -7.8900363E37F, 1.0241738E38F, -2.7887914E38F, 3.3943065E38F, -1.5500341E38F, -8.810487E37F, -1.28523E38F, 2.439279E38F, 4.50192E37F, 5.787117E37F, -5.1232677E37F, 1.6248863E38F, 1.2087478E38F, -1.6342212E38F, -1.3178036E38F, -2.8608653E38F, -5.8988894E37F, 2.2315407E38F, 8.902111E37F, -7.838258E37F, -1.9133216E38F, 2.2683873E38F, 2.4023419E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_relative_alt_SET((int32_t) -1938971262, PH.base.pack) ;
        p63_lon_SET((int32_t) -1441668792, PH.base.pack) ;
        p63_vy_SET((float) -1.6488041E38F, PH.base.pack) ;
        p63_lat_SET((int32_t)1318290615, PH.base.pack) ;
        p63_alt_SET((int32_t)79314874, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ay_SET((float) -7.6730227E37F, PH.base.pack) ;
        p64_z_SET((float) -2.8368848E38F, PH.base.pack) ;
        p64_az_SET((float)1.6112387E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.07983E38F, 1.746442E38F, 3.3696358E38F, 3.4010794E38F, 1.0797534E38F, 2.6759536E38F, -4.0495395E37F, 2.6002363E38F, 2.5975583E38F, -8.961078E37F, 2.4542355E38F, 3.2882562E37F, 7.864917E37F, 2.5093793E36F, 3.0233511E38F, 1.4311627E38F, 1.3553786E38F, -1.5742004E38F, -1.430415E38F, -7.758954E37F, -3.9298525E37F, -5.965118E37F, -6.4200504E37F, -1.5390408E38F, 3.05372E38F, -2.5885417E38F, 1.8400482E38F, -2.076366E38F, 2.404641E38F, -1.866247E38F, -1.5342695E38F, 2.6882869E37F, 1.1706791E38F, -1.822737E38F, 9.682143E37F, 2.998175E37F, 3.0644204E38F, -1.140123E38F, 7.6661885E37F, -2.9762016E38F, 1.8645084E38F, -9.776732E36F, 2.912632E38F, -4.106516E37F, -3.0343408E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vy_SET((float)1.8198485E38F, PH.base.pack) ;
        p64_vz_SET((float) -4.706771E37F, PH.base.pack) ;
        p64_x_SET((float)2.0516674E37F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)6905901488268524191L, PH.base.pack) ;
        p64_ax_SET((float) -2.9581049E38F, PH.base.pack) ;
        p64_vx_SET((float)1.423285E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_y_SET((float)1.3751176E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chancount_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)8240, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)51401, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)8340, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)28080, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)32478, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)28029, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)15348, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)54511, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2591288394L, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)18801, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)11551, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)30922, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)26531, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)6227, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)15289, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)55610, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)16148, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)24852, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)5965, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)53366, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)65236, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)11388, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)30395, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)6805, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -31258, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)29761, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan1_raw_SET((uint16_t)(uint16_t)29509, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)13746, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)11061, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)13539, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)33390, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)58116, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)42294, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)3514, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_MODE, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p73_param3_SET((float) -6.9644354E37F, PH.base.pack) ;
        p73_y_SET((int32_t) -1623503035, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p73_param4_SET((float) -3.272787E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)48578, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_x_SET((int32_t) -1603170470, PH.base.pack) ;
        p73_param1_SET((float) -1.9704984E38F, PH.base.pack) ;
        p73_param2_SET((float)1.3940561E38F, PH.base.pack) ;
        p73_z_SET((float)1.6850296E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float) -9.024563E37F, PH.base.pack) ;
        p74_climb_SET((float)1.8752303E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)3.1759154E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)11424, PH.base.pack) ;
        p74_airspeed_SET((float)5.45719E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)14210, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_x_SET((int32_t) -463853292, PH.base.pack) ;
        p75_param4_SET((float) -5.781586E37F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p75_z_SET((float)2.6995983E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -1433460313, PH.base.pack) ;
        p75_param1_SET((float)3.0039924E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_MODE, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p75_param3_SET((float)1.87522E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p75_param2_SET((float) -2.9690606E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float)1.8383392E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST, PH.base.pack) ;
        p76_param1_SET((float)2.391765E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p76_param3_SET((float)3.1153984E38F, PH.base.pack) ;
        p76_param2_SET((float) -2.9351498E38F, PH.base.pack) ;
        p76_param7_SET((float) -4.2231498E37F, PH.base.pack) ;
        p76_param5_SET((float) -9.323963E37F, PH.base.pack) ;
        p76_param4_SET((float)4.9128737E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)220, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)126, &PH) ;
        p77_result_param2_SET((int32_t)419485011, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)159, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p81_thrust_SET((float)3.3402837E38F, PH.base.pack) ;
        p81_pitch_SET((float) -2.780718E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)955267283L, PH.base.pack) ;
        p81_roll_SET((float) -1.815447E38F, PH.base.pack) ;
        p81_yaw_SET((float) -2.5572395E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_yaw_rate_SET((float) -1.7205252E38F, PH.base.pack) ;
        p82_thrust_SET((float)2.9086578E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)6.4078607E37F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        {
            float q[] =  {3.3574708E38F, -6.634659E37F, 1.2825042E38F, 1.3600473E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -3.270389E37F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)4224173035L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float) -2.0769624E38F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.7803992E37F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)3627862708L, PH.base.pack) ;
        p83_thrust_SET((float)4.0675612E37F, PH.base.pack) ;
        {
            float q[] =  {8.1305076E37F, -3.4834656E37F, -9.95716E37F, -6.6531526E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)6.1145735E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_yaw_SET((float) -2.0028283E38F, PH.base.pack) ;
        p84_afx_SET((float) -1.849735E38F, PH.base.pack) ;
        p84_vx_SET((float)2.461804E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2178676938L, PH.base.pack) ;
        p84_x_SET((float)7.701868E36F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)6592, PH.base.pack) ;
        p84_vy_SET((float)2.0559058E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.90365E38F, PH.base.pack) ;
        p84_y_SET((float)3.515688E36F, PH.base.pack) ;
        p84_z_SET((float) -2.9868288E38F, PH.base.pack) ;
        p84_afz_SET((float) -7.277603E37F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p84_afy_SET((float) -7.37306E37F, PH.base.pack) ;
        p84_vz_SET((float)1.6350337E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_afz_SET((float) -2.63269E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2185977917L, PH.base.pack) ;
        p86_vy_SET((float)2.7260964E38F, PH.base.pack) ;
        p86_yaw_SET((float)1.458137E38F, PH.base.pack) ;
        p86_vx_SET((float)3.2697398E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)58662, PH.base.pack) ;
        p86_yaw_rate_SET((float)1.3960652E38F, PH.base.pack) ;
        p86_alt_SET((float)8.182368E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p86_vz_SET((float)1.0859401E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p86_afx_SET((float)1.4919264E37F, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1094802344, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1353990491, PH.base.pack) ;
        p86_afy_SET((float)2.902049E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_alt_SET((float)2.9980166E38F, PH.base.pack) ;
        p87_vy_SET((float)4.011601E37F, PH.base.pack) ;
        p87_afx_SET((float) -3.0428981E38F, PH.base.pack) ;
        p87_afy_SET((float)3.3538348E38F, PH.base.pack) ;
        p87_afz_SET((float) -1.2463477E37F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)3399169150L, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.270991E38F, PH.base.pack) ;
        p87_yaw_SET((float) -2.6637018E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)352716676, PH.base.pack) ;
        p87_vx_SET((float) -1.6785394E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -512926534, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)36103, PH.base.pack) ;
        p87_vz_SET((float) -6.038733E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_yaw_SET((float) -1.6150837E38F, PH.base.pack) ;
        p89_x_SET((float)2.6938156E37F, PH.base.pack) ;
        p89_y_SET((float)2.8004047E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2056433888L, PH.base.pack) ;
        p89_roll_SET((float) -2.7465804E38F, PH.base.pack) ;
        p89_z_SET((float)2.7644765E37F, PH.base.pack) ;
        p89_pitch_SET((float)7.1685956E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_yaw_SET((float) -7.844748E37F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)15885, PH.base.pack) ;
        p90_roll_SET((float) -3.0510056E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)1215292360395745218L, PH.base.pack) ;
        p90_lat_SET((int32_t) -1221436726, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)16216, PH.base.pack) ;
        p90_rollspeed_SET((float)7.982009E37F, PH.base.pack) ;
        p90_alt_SET((int32_t) -1413700150, PH.base.pack) ;
        p90_pitch_SET((float) -2.5964871E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -23368, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -11658, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -2008, PH.base.pack) ;
        p90_lon_SET((int32_t) -526082261, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)9539, PH.base.pack) ;
        p90_yawspeed_SET((float)1.5819686E38F, PH.base.pack) ;
        p90_pitchspeed_SET((float)2.7628736E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_time_usec_SET((uint64_t)8730561058553656915L, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.171415E38F, PH.base.pack) ;
        p91_aux2_SET((float)1.782576E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -2.5578116E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p91_aux3_SET((float) -4.828447E37F, PH.base.pack) ;
        p91_throttle_SET((float) -5.73919E36F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)6.550317E37F, PH.base.pack) ;
        p91_aux1_SET((float)3.2506689E38F, PH.base.pack) ;
        p91_aux4_SET((float) -9.998156E36F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan5_raw_SET((uint16_t)(uint16_t)25719, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)29329, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)18462, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)44510, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)44903, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)22518, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)8578799396806267938L, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)34602, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)56728, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)47179, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)58671, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)57549, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)30189, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)168694075062650006L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        {
            float controls[] =  {-2.1460635E38F, 6.050361E37F, -6.450423E37F, 2.5336307E38F, -9.472211E37F, -3.1798673E36F, -3.4789053E37F, -3.3233489E38F, 3.699044E37F, -3.5849557E37F, -6.5716173E37F, -2.621653E38F, 1.8690167E38F, -4.7093696E37F, -2.2475642E38F, 5.1953945E37F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_time_usec_SET((uint64_t)49833716346640058L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_x_SET((int16_t)(int16_t)20463, PH.base.pack) ;
        p100_ground_distance_SET((float)2.2859878E38F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)2.2742153E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)1.1781716E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t) -22895, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)577890460702558031L, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)1.2916832E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -2.6205897E38F, &PH) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_pitch_SET((float) -1.5551849E38F, PH.base.pack) ;
        p101_z_SET((float) -2.0796948E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)3656345359988369404L, PH.base.pack) ;
        p101_y_SET((float)5.4613684E37F, PH.base.pack) ;
        p101_x_SET((float)3.3883343E38F, PH.base.pack) ;
        p101_yaw_SET((float)1.7056847E38F, PH.base.pack) ;
        p101_roll_SET((float) -1.1584056E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_pitch_SET((float) -7.1297674E37F, PH.base.pack) ;
        p102_yaw_SET((float) -1.870084E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)4864910255147853062L, PH.base.pack) ;
        p102_y_SET((float)4.0959185E37F, PH.base.pack) ;
        p102_x_SET((float) -1.4472943E38F, PH.base.pack) ;
        p102_z_SET((float)1.8943773E38F, PH.base.pack) ;
        p102_roll_SET((float)7.2034955E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_y_SET((float)2.321164E38F, PH.base.pack) ;
        p103_x_SET((float)3.196816E37F, PH.base.pack) ;
        p103_z_SET((float)1.4500191E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)5388120042841254762L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_x_SET((float)1.1164233E38F, PH.base.pack) ;
        p104_z_SET((float)1.1665321E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)3609152004019200799L, PH.base.pack) ;
        p104_pitch_SET((float) -1.9158843E38F, PH.base.pack) ;
        p104_roll_SET((float) -2.0740082E38F, PH.base.pack) ;
        p104_y_SET((float) -1.6190809E38F, PH.base.pack) ;
        p104_yaw_SET((float)1.3667108E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zmag_SET((float) -1.7549225E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4295216794757674738L, PH.base.pack) ;
        p105_temperature_SET((float) -3.0288203E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -1.0704826E37F, PH.base.pack) ;
        p105_zacc_SET((float) -2.4213276E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)15159, PH.base.pack) ;
        p105_yacc_SET((float) -1.0591981E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.4163672E38F, PH.base.pack) ;
        p105_xacc_SET((float)2.7571368E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.0428237E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)9.899173E37F, PH.base.pack) ;
        p105_xmag_SET((float)2.2578858E37F, PH.base.pack) ;
        p105_ymag_SET((float)1.0682177E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -6.0265646E37F, PH.base.pack) ;
        p105_xgyro_SET((float) -6.1680755E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_ygyro_SET((float)1.0326875E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)12627, PH.base.pack) ;
        p106_integrated_y_SET((float)1.3160767E38F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)221965493L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -2.3582972E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)2898612392L, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)7347305312564553915L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)6.123266E37F, PH.base.pack) ;
        p106_integrated_x_SET((float)2.1305582E38F, PH.base.pack) ;
        p106_distance_SET((float)1.677659E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_time_usec_SET((uint64_t)253825335358618503L, PH.base.pack) ;
        p107_yacc_SET((float)2.1900589E38F, PH.base.pack) ;
        p107_xacc_SET((float) -1.8668873E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -1.4229941E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.7615326E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -5.440174E37F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2330668296L, PH.base.pack) ;
        p107_temperature_SET((float) -9.901184E37F, PH.base.pack) ;
        p107_pressure_alt_SET((float)6.179117E37F, PH.base.pack) ;
        p107_ygyro_SET((float) -2.3143791E38F, PH.base.pack) ;
        p107_xgyro_SET((float)1.6736438E38F, PH.base.pack) ;
        p107_zmag_SET((float) -2.8196806E37F, PH.base.pack) ;
        p107_xmag_SET((float) -3.597683E36F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.9562108E38F, PH.base.pack) ;
        p107_zacc_SET((float)1.908543E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_vn_SET((float) -1.957746E38F, PH.base.pack) ;
        p108_ygyro_SET((float)2.8531734E38F, PH.base.pack) ;
        p108_xacc_SET((float) -7.581322E36F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -1.9974048E38F, PH.base.pack) ;
        p108_q2_SET((float) -1.5890178E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.7172928E38F, PH.base.pack) ;
        p108_q4_SET((float) -3.3845478E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -7.2145023E37F, PH.base.pack) ;
        p108_yacc_SET((float) -2.4077135E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -5.9008035E37F, PH.base.pack) ;
        p108_yaw_SET((float) -8.940691E37F, PH.base.pack) ;
        p108_zacc_SET((float)1.4740144E38F, PH.base.pack) ;
        p108_pitch_SET((float) -9.22427E37F, PH.base.pack) ;
        p108_roll_SET((float) -5.7321167E37F, PH.base.pack) ;
        p108_alt_SET((float)1.192319E38F, PH.base.pack) ;
        p108_lon_SET((float)2.6340765E38F, PH.base.pack) ;
        p108_ve_SET((float) -2.2543115E38F, PH.base.pack) ;
        p108_q3_SET((float) -2.6911478E38F, PH.base.pack) ;
        p108_vd_SET((float)2.4501625E38F, PH.base.pack) ;
        p108_q1_SET((float) -7.9453977E37F, PH.base.pack) ;
        p108_lat_SET((float) -3.3987948E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_remrssi_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)59238, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)54843, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)234, (uint8_t)74, (uint8_t)107, (uint8_t)101, (uint8_t)241, (uint8_t)249, (uint8_t)173, (uint8_t)26, (uint8_t)206, (uint8_t)43, (uint8_t)131, (uint8_t)210, (uint8_t)254, (uint8_t)133, (uint8_t)108, (uint8_t)224, (uint8_t)208, (uint8_t)34, (uint8_t)163, (uint8_t)232, (uint8_t)209, (uint8_t)242, (uint8_t)0, (uint8_t)91, (uint8_t)231, (uint8_t)36, (uint8_t)180, (uint8_t)92, (uint8_t)137, (uint8_t)0, (uint8_t)108, (uint8_t)89, (uint8_t)188, (uint8_t)44, (uint8_t)128, (uint8_t)138, (uint8_t)74, (uint8_t)84, (uint8_t)30, (uint8_t)141, (uint8_t)161, (uint8_t)57, (uint8_t)180, (uint8_t)53, (uint8_t)206, (uint8_t)150, (uint8_t)46, (uint8_t)146, (uint8_t)112, (uint8_t)84, (uint8_t)79, (uint8_t)190, (uint8_t)6, (uint8_t)171, (uint8_t)231, (uint8_t)111, (uint8_t)192, (uint8_t)236, (uint8_t)17, (uint8_t)210, (uint8_t)80, (uint8_t)133, (uint8_t)83, (uint8_t)192, (uint8_t)34, (uint8_t)170, (uint8_t)98, (uint8_t)69, (uint8_t)232, (uint8_t)28, (uint8_t)251, (uint8_t)254, (uint8_t)11, (uint8_t)131, (uint8_t)140, (uint8_t)213, (uint8_t)223, (uint8_t)84, (uint8_t)11, (uint8_t)93, (uint8_t)158, (uint8_t)90, (uint8_t)194, (uint8_t)217, (uint8_t)142, (uint8_t)85, (uint8_t)43, (uint8_t)200, (uint8_t)180, (uint8_t)71, (uint8_t)210, (uint8_t)51, (uint8_t)217, (uint8_t)164, (uint8_t)243, (uint8_t)53, (uint8_t)60, (uint8_t)42, (uint8_t)230, (uint8_t)224, (uint8_t)8, (uint8_t)179, (uint8_t)153, (uint8_t)241, (uint8_t)24, (uint8_t)82, (uint8_t)200, (uint8_t)119, (uint8_t)45, (uint8_t)6, (uint8_t)132, (uint8_t)94, (uint8_t)224, (uint8_t)101, (uint8_t)53, (uint8_t)46, (uint8_t)70, (uint8_t)146, (uint8_t)43, (uint8_t)77, (uint8_t)174, (uint8_t)83, (uint8_t)8, (uint8_t)167, (uint8_t)91, (uint8_t)194, (uint8_t)124, (uint8_t)170, (uint8_t)23, (uint8_t)160, (uint8_t)120, (uint8_t)0, (uint8_t)214, (uint8_t)29, (uint8_t)167, (uint8_t)244, (uint8_t)112, (uint8_t)22, (uint8_t)112, (uint8_t)68, (uint8_t)124, (uint8_t)2, (uint8_t)81, (uint8_t)127, (uint8_t)27, (uint8_t)225, (uint8_t)150, (uint8_t)99, (uint8_t)160, (uint8_t)169, (uint8_t)251, (uint8_t)235, (uint8_t)160, (uint8_t)191, (uint8_t)255, (uint8_t)119, (uint8_t)201, (uint8_t)206, (uint8_t)228, (uint8_t)225, (uint8_t)226, (uint8_t)31, (uint8_t)156, (uint8_t)157, (uint8_t)137, (uint8_t)198, (uint8_t)169, (uint8_t)179, (uint8_t)22, (uint8_t)107, (uint8_t)195, (uint8_t)91, (uint8_t)246, (uint8_t)238, (uint8_t)223, (uint8_t)177, (uint8_t)39, (uint8_t)113, (uint8_t)136, (uint8_t)168, (uint8_t)69, (uint8_t)128, (uint8_t)251, (uint8_t)227, (uint8_t)185, (uint8_t)14, (uint8_t)85, (uint8_t)176, (uint8_t)200, (uint8_t)172, (uint8_t)109, (uint8_t)200, (uint8_t)0, (uint8_t)28, (uint8_t)255, (uint8_t)201, (uint8_t)204, (uint8_t)192, (uint8_t)207, (uint8_t)140, (uint8_t)207, (uint8_t)255, (uint8_t)191, (uint8_t)219, (uint8_t)102, (uint8_t)125, (uint8_t)107, (uint8_t)52, (uint8_t)168, (uint8_t)69, (uint8_t)97, (uint8_t)87, (uint8_t)88, (uint8_t)23, (uint8_t)97, (uint8_t)253, (uint8_t)53, (uint8_t)56, (uint8_t)31, (uint8_t)194, (uint8_t)19, (uint8_t)201, (uint8_t)215, (uint8_t)33, (uint8_t)61, (uint8_t)37, (uint8_t)203, (uint8_t)242, (uint8_t)250, (uint8_t)44, (uint8_t)224, (uint8_t)151, (uint8_t)174, (uint8_t)12, (uint8_t)22, (uint8_t)177, (uint8_t)44, (uint8_t)202, (uint8_t)249, (uint8_t)154, (uint8_t)46, (uint8_t)113, (uint8_t)170, (uint8_t)134, (uint8_t)167, (uint8_t)113, (uint8_t)9, (uint8_t)40, (uint8_t)156, (uint8_t)27, (uint8_t)219};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)324528185307996431L, PH.base.pack) ;
        p111_tc1_SET((int64_t)4406578966040699344L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)5058394971118309688L, PH.base.pack) ;
        p112_seq_SET((uint32_t)437874409L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_cog_SET((uint16_t)(uint16_t)48483, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)9471, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)35539, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)2437075640337640591L, PH.base.pack) ;
        p113_lat_SET((int32_t) -2036301055, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p113_lon_SET((int32_t)1126133904, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p113_alt_SET((int32_t)1909407522, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)44727, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)24247, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)32321, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -14578, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_distance_SET((float) -1.2397315E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -4899, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)998444259L, PH.base.pack) ;
        p114_integrated_y_SET((float)2.5294027E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)1.7526983E38F, PH.base.pack) ;
        p114_integrated_x_SET((float)1.5405808E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -1.2112714E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.4973657E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)3882103311772958860L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)4113429182L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_xacc_SET((int16_t)(int16_t)11866, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)23693, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -27523, PH.base.pack) ;
        p115_pitchspeed_SET((float)1.3080606E38F, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)23319, PH.base.pack) ;
        p115_lat_SET((int32_t) -865203578, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -987, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)6285, PH.base.pack) ;
        p115_yawspeed_SET((float)2.6786457E38F, PH.base.pack) ;
        p115_lon_SET((int32_t)2035973576, PH.base.pack) ;
        p115_alt_SET((int32_t)2038110369, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-1.00804964E37F, -2.7537864E38F, 3.3346403E37F, 3.191878E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_true_airspeed_SET((uint16_t)(uint16_t)59946, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)32086, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)2564160551250285376L, PH.base.pack) ;
        p115_rollspeed_SET((float)2.8825734E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t)5379, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -10784, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -31576, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -31557, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -19649, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -16444, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)24027, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)32474, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2006244883L, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)7396, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)30400, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)58092, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)4018541039L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)11144, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)307, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)27043, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)2751415951L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)56381, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1812985061L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p119_count_SET((uint32_t)464275940L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)48528, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p120_ofs_SET((uint32_t)2498578782L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)111, (uint8_t)255, (uint8_t)34, (uint8_t)197, (uint8_t)26, (uint8_t)101, (uint8_t)57, (uint8_t)149, (uint8_t)169, (uint8_t)200, (uint8_t)223, (uint8_t)68, (uint8_t)73, (uint8_t)167, (uint8_t)61, (uint8_t)59, (uint8_t)145, (uint8_t)98, (uint8_t)225, (uint8_t)171, (uint8_t)135, (uint8_t)37, (uint8_t)210, (uint8_t)224, (uint8_t)140, (uint8_t)212, (uint8_t)104, (uint8_t)154, (uint8_t)49, (uint8_t)101, (uint8_t)255, (uint8_t)219, (uint8_t)12, (uint8_t)100, (uint8_t)21, (uint8_t)20, (uint8_t)45, (uint8_t)146, (uint8_t)188, (uint8_t)207, (uint8_t)92, (uint8_t)96, (uint8_t)174, (uint8_t)155, (uint8_t)7, (uint8_t)234, (uint8_t)68, (uint8_t)175, (uint8_t)130, (uint8_t)197, (uint8_t)77, (uint8_t)105, (uint8_t)141, (uint8_t)238, (uint8_t)160, (uint8_t)172, (uint8_t)21, (uint8_t)95, (uint8_t)108, (uint8_t)206, (uint8_t)147, (uint8_t)18, (uint8_t)195, (uint8_t)98, (uint8_t)79, (uint8_t)46, (uint8_t)144, (uint8_t)168, (uint8_t)72, (uint8_t)250, (uint8_t)194, (uint8_t)56, (uint8_t)220, (uint8_t)136, (uint8_t)200, (uint8_t)36, (uint8_t)98, (uint8_t)247, (uint8_t)175, (uint8_t)106, (uint8_t)30, (uint8_t)231, (uint8_t)185, (uint8_t)109, (uint8_t)233, (uint8_t)46, (uint8_t)215, (uint8_t)1, (uint8_t)210, (uint8_t)156};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)168, (uint8_t)187, (uint8_t)3, (uint8_t)54, (uint8_t)54, (uint8_t)115, (uint8_t)238, (uint8_t)12, (uint8_t)44, (uint8_t)132, (uint8_t)109, (uint8_t)149, (uint8_t)47, (uint8_t)112, (uint8_t)244, (uint8_t)73, (uint8_t)49, (uint8_t)126, (uint8_t)2, (uint8_t)184, (uint8_t)142, (uint8_t)210, (uint8_t)164, (uint8_t)102, (uint8_t)191, (uint8_t)168, (uint8_t)67, (uint8_t)54, (uint8_t)187, (uint8_t)129, (uint8_t)243, (uint8_t)150, (uint8_t)60, (uint8_t)134, (uint8_t)55, (uint8_t)28, (uint8_t)174, (uint8_t)158, (uint8_t)212, (uint8_t)111, (uint8_t)47, (uint8_t)84, (uint8_t)153, (uint8_t)85, (uint8_t)55, (uint8_t)102, (uint8_t)217, (uint8_t)103, (uint8_t)161, (uint8_t)38, (uint8_t)134, (uint8_t)203, (uint8_t)222, (uint8_t)207, (uint8_t)138, (uint8_t)185, (uint8_t)205, (uint8_t)145, (uint8_t)132, (uint8_t)63, (uint8_t)11, (uint8_t)203, (uint8_t)18, (uint8_t)198, (uint8_t)65, (uint8_t)78, (uint8_t)19, (uint8_t)92, (uint8_t)213, (uint8_t)204, (uint8_t)84, (uint8_t)0, (uint8_t)113, (uint8_t)95, (uint8_t)254, (uint8_t)194, (uint8_t)119, (uint8_t)186, (uint8_t)100, (uint8_t)60, (uint8_t)87, (uint8_t)207, (uint8_t)130, (uint8_t)239, (uint8_t)170, (uint8_t)27, (uint8_t)1, (uint8_t)234, (uint8_t)67, (uint8_t)7, (uint8_t)118, (uint8_t)72, (uint8_t)228, (uint8_t)90, (uint8_t)138, (uint8_t)251, (uint8_t)79, (uint8_t)212, (uint8_t)86, (uint8_t)156, (uint8_t)157, (uint8_t)160, (uint8_t)166, (uint8_t)154, (uint8_t)47, (uint8_t)87, (uint8_t)56, (uint8_t)40, (uint8_t)204, (uint8_t)206};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_vel_SET((uint16_t)(uint16_t)8076, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1040243977L, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)9052834151693481692L, PH.base.pack) ;
        p124_alt_SET((int32_t) -626716540, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)60884, PH.base.pack) ;
        p124_lat_SET((int32_t)1322850547, PH.base.pack) ;
        p124_lon_SET((int32_t) -1360564537, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)30487, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)55804, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)7507, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)57513, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)3678472208L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)4745, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)222, (uint8_t)212, (uint8_t)53, (uint8_t)190, (uint8_t)121, (uint8_t)0, (uint8_t)186, (uint8_t)9, (uint8_t)202, (uint8_t)94, (uint8_t)85, (uint8_t)218, (uint8_t)117, (uint8_t)78, (uint8_t)137, (uint8_t)137, (uint8_t)150, (uint8_t)74, (uint8_t)224, (uint8_t)51, (uint8_t)149, (uint8_t)89, (uint8_t)156, (uint8_t)57, (uint8_t)69, (uint8_t)78, (uint8_t)148, (uint8_t)116, (uint8_t)215, (uint8_t)218, (uint8_t)234, (uint8_t)237, (uint8_t)161, (uint8_t)143, (uint8_t)202, (uint8_t)78, (uint8_t)108, (uint8_t)65, (uint8_t)50, (uint8_t)1, (uint8_t)207, (uint8_t)178, (uint8_t)185, (uint8_t)193, (uint8_t)145, (uint8_t)64, (uint8_t)24, (uint8_t)135, (uint8_t)247, (uint8_t)7, (uint8_t)168, (uint8_t)105, (uint8_t)241, (uint8_t)105, (uint8_t)67, (uint8_t)43, (uint8_t)115, (uint8_t)68, (uint8_t)8, (uint8_t)247, (uint8_t)121, (uint8_t)25, (uint8_t)219, (uint8_t)228, (uint8_t)96, (uint8_t)44, (uint8_t)183, (uint8_t)31, (uint8_t)15, (uint8_t)162};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_b_mm_SET((int32_t) -1993486393, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1487215922, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p127_tow_SET((uint32_t)1033030650L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)20938, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -786893060, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -1751305681, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)4179374820L, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)4212498538L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_b_mm_SET((int32_t) -1177036880, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -208129164, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)516360011L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p128_tow_SET((uint32_t)58132851L, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -403094357, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)53345, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)96913701, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)1421188753L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_yacc_SET((int16_t)(int16_t)8014, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)3240, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)22825, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)2285935416L, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)26387, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -28060, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -3312, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)14532, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)13897, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -29000, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_size_SET((uint32_t)3345489078L, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)30879, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)9931, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)485, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)9748, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)111, (uint8_t)101, (uint8_t)232, (uint8_t)155, (uint8_t)61, (uint8_t)134, (uint8_t)11, (uint8_t)42, (uint8_t)54, (uint8_t)136, (uint8_t)169, (uint8_t)95, (uint8_t)61, (uint8_t)65, (uint8_t)24, (uint8_t)160, (uint8_t)124, (uint8_t)19, (uint8_t)57, (uint8_t)14, (uint8_t)49, (uint8_t)159, (uint8_t)59, (uint8_t)235, (uint8_t)138, (uint8_t)26, (uint8_t)65, (uint8_t)95, (uint8_t)105, (uint8_t)228, (uint8_t)165, (uint8_t)195, (uint8_t)29, (uint8_t)215, (uint8_t)60, (uint8_t)197, (uint8_t)17, (uint8_t)34, (uint8_t)248, (uint8_t)228, (uint8_t)30, (uint8_t)0, (uint8_t)0, (uint8_t)139, (uint8_t)4, (uint8_t)125, (uint8_t)104, (uint8_t)147, (uint8_t)19, (uint8_t)218, (uint8_t)76, (uint8_t)176, (uint8_t)95, (uint8_t)160, (uint8_t)10, (uint8_t)43, (uint8_t)199, (uint8_t)209, (uint8_t)223, (uint8_t)77, (uint8_t)219, (uint8_t)19, (uint8_t)127, (uint8_t)209, (uint8_t)231, (uint8_t)228, (uint8_t)137, (uint8_t)233, (uint8_t)77, (uint8_t)15, (uint8_t)109, (uint8_t)224, (uint8_t)222, (uint8_t)73, (uint8_t)202, (uint8_t)230, (uint8_t)171, (uint8_t)26, (uint8_t)103, (uint8_t)253, (uint8_t)151, (uint8_t)250, (uint8_t)130, (uint8_t)231, (uint8_t)159, (uint8_t)128, (uint8_t)229, (uint8_t)247, (uint8_t)199, (uint8_t)90, (uint8_t)127, (uint8_t)55, (uint8_t)81, (uint8_t)180, (uint8_t)48, (uint8_t)38, (uint8_t)213, (uint8_t)198, (uint8_t)247, (uint8_t)219, (uint8_t)29, (uint8_t)80, (uint8_t)240, (uint8_t)40, (uint8_t)3, (uint8_t)27, (uint8_t)104, (uint8_t)244, (uint8_t)8, (uint8_t)133, (uint8_t)19, (uint8_t)98, (uint8_t)165, (uint8_t)52, (uint8_t)217, (uint8_t)129, (uint8_t)6, (uint8_t)126, (uint8_t)255, (uint8_t)220, (uint8_t)188, (uint8_t)52, (uint8_t)87, (uint8_t)210, (uint8_t)91, (uint8_t)155, (uint8_t)184, (uint8_t)178, (uint8_t)101, (uint8_t)129, (uint8_t)18, (uint8_t)254, (uint8_t)68, (uint8_t)61, (uint8_t)214, (uint8_t)85, (uint8_t)81, (uint8_t)22, (uint8_t)59, (uint8_t)76, (uint8_t)124, (uint8_t)116, (uint8_t)4, (uint8_t)101, (uint8_t)75, (uint8_t)140, (uint8_t)67, (uint8_t)103, (uint8_t)243, (uint8_t)96, (uint8_t)34, (uint8_t)57, (uint8_t)238, (uint8_t)52, (uint8_t)235, (uint8_t)120, (uint8_t)51, (uint8_t)212, (uint8_t)170, (uint8_t)47, (uint8_t)96, (uint8_t)242, (uint8_t)20, (uint8_t)157, (uint8_t)187, (uint8_t)206, (uint8_t)57, (uint8_t)252, (uint8_t)224, (uint8_t)137, (uint8_t)29, (uint8_t)112, (uint8_t)196, (uint8_t)14, (uint8_t)98, (uint8_t)67, (uint8_t)225, (uint8_t)250, (uint8_t)171, (uint8_t)86, (uint8_t)132, (uint8_t)199, (uint8_t)134, (uint8_t)119, (uint8_t)78, (uint8_t)97, (uint8_t)128, (uint8_t)78, (uint8_t)248, (uint8_t)34, (uint8_t)85, (uint8_t)198, (uint8_t)191, (uint8_t)163, (uint8_t)160, (uint8_t)196, (uint8_t)220, (uint8_t)105, (uint8_t)165, (uint8_t)24, (uint8_t)126, (uint8_t)208, (uint8_t)119, (uint8_t)233, (uint8_t)253, (uint8_t)3, (uint8_t)251, (uint8_t)253, (uint8_t)29, (uint8_t)150, (uint8_t)80, (uint8_t)211, (uint8_t)80, (uint8_t)181, (uint8_t)223, (uint8_t)76, (uint8_t)173, (uint8_t)92, (uint8_t)215, (uint8_t)21, (uint8_t)2, (uint8_t)41, (uint8_t)131, (uint8_t)91, (uint8_t)30, (uint8_t)131, (uint8_t)129, (uint8_t)138, (uint8_t)230, (uint8_t)11, (uint8_t)45, (uint8_t)229, (uint8_t)38, (uint8_t)46, (uint8_t)82, (uint8_t)75, (uint8_t)73, (uint8_t)112, (uint8_t)157, (uint8_t)53, (uint8_t)203, (uint8_t)132, (uint8_t)181, (uint8_t)7, (uint8_t)100, (uint8_t)160, (uint8_t)68, (uint8_t)42, (uint8_t)154, (uint8_t)96, (uint8_t)59, (uint8_t)252, (uint8_t)75};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_max_distance_SET((uint16_t)(uint16_t)1806, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2556871790L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)43393, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)53683, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)2469415147006425469L, PH.base.pack) ;
        p133_lat_SET((int32_t)1701585401, PH.base.pack) ;
        p133_lon_SET((int32_t) -239508316, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)47097, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t)1939081233, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)18055, (int16_t) -4873, (int16_t) -645, (int16_t)397, (int16_t) -20581, (int16_t)20427, (int16_t) -5100, (int16_t)16160, (int16_t) -12782, (int16_t) -32527, (int16_t) -14409, (int16_t) -32463, (int16_t) -18927, (int16_t) -20456, (int16_t) -6533, (int16_t) -24030};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p134_lat_SET((int32_t)54815037, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)51495, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)63915068, PH.base.pack) ;
        p135_lon_SET((int32_t)66304830, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)29651, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)2574, PH.base.pack) ;
        p136_lat_SET((int32_t) -2098738873, PH.base.pack) ;
        p136_terrain_height_SET((float) -1.3636265E38F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)49268, PH.base.pack) ;
        p136_lon_SET((int32_t)616597511, PH.base.pack) ;
        p136_current_height_SET((float)1.9297962E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)409207858L, PH.base.pack) ;
        p137_press_abs_SET((float)1.072452E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -1.3526793E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)1656, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)2568917624339497981L, PH.base.pack) ;
        p138_z_SET((float) -2.501139E38F, PH.base.pack) ;
        {
            float q[] =  {1.5593295E38F, -2.952754E38F, 1.1108961E38F, 9.676067E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)3.3744015E38F, PH.base.pack) ;
        p138_y_SET((float)3.0054748E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_time_usec_SET((uint64_t)3863701341537056843L, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        {
            float controls[] =  {-1.5859447E38F, -1.4240247E37F, -7.3221273E37F, 5.816556E37F, 3.3034644E38F, -4.7016838E36F, 2.9042174E37F, -2.1428023E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_group_mlx_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)4907995812894589616L, PH.base.pack) ;
        {
            float controls[] =  {2.7467441E38F, 2.2238946E38F, 2.5618215E38F, -3.1073682E38F, 1.1358218E38F, 1.0920187E38F, -3.2025943E38F, -9.063457E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_local_SET((float) -4.343885E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.4004804E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)2.977244E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)5.0308773E37F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)3.3379346E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.7799833E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)4831941631403013107L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)31, (uint8_t)195, (uint8_t)71, (uint8_t)167, (uint8_t)47, (uint8_t)229, (uint8_t)67, (uint8_t)131, (uint8_t)87, (uint8_t)188, (uint8_t)162, (uint8_t)177, (uint8_t)8, (uint8_t)48, (uint8_t)234, (uint8_t)104, (uint8_t)33, (uint8_t)239, (uint8_t)218, (uint8_t)82, (uint8_t)231, (uint8_t)44, (uint8_t)0, (uint8_t)103, (uint8_t)159, (uint8_t)238, (uint8_t)77, (uint8_t)158, (uint8_t)143, (uint8_t)36, (uint8_t)224, (uint8_t)74, (uint8_t)112, (uint8_t)184, (uint8_t)114, (uint8_t)182, (uint8_t)138, (uint8_t)63, (uint8_t)191, (uint8_t)116, (uint8_t)209, (uint8_t)162, (uint8_t)181, (uint8_t)36, (uint8_t)48, (uint8_t)1, (uint8_t)183, (uint8_t)97, (uint8_t)81, (uint8_t)222, (uint8_t)100, (uint8_t)110, (uint8_t)92, (uint8_t)40, (uint8_t)160, (uint8_t)86, (uint8_t)76, (uint8_t)231, (uint8_t)37, (uint8_t)32, (uint8_t)84, (uint8_t)87, (uint8_t)161, (uint8_t)215, (uint8_t)123, (uint8_t)183, (uint8_t)224, (uint8_t)42, (uint8_t)117, (uint8_t)181, (uint8_t)246, (uint8_t)88, (uint8_t)15, (uint8_t)169, (uint8_t)96, (uint8_t)177, (uint8_t)23, (uint8_t)40, (uint8_t)85, (uint8_t)145, (uint8_t)141, (uint8_t)46, (uint8_t)96, (uint8_t)35, (uint8_t)53, (uint8_t)167, (uint8_t)126, (uint8_t)23, (uint8_t)193, (uint8_t)116, (uint8_t)142, (uint8_t)190, (uint8_t)146, (uint8_t)154, (uint8_t)150, (uint8_t)135, (uint8_t)230, (uint8_t)220, (uint8_t)232, (uint8_t)175, (uint8_t)168, (uint8_t)29, (uint8_t)143, (uint8_t)23, (uint8_t)143, (uint8_t)19, (uint8_t)254, (uint8_t)119, (uint8_t)217, (uint8_t)222, (uint8_t)240, (uint8_t)64, (uint8_t)188, (uint8_t)11, (uint8_t)47, (uint8_t)46, (uint8_t)22, (uint8_t)198, (uint8_t)7, (uint8_t)20};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)250, (uint8_t)20, (uint8_t)36, (uint8_t)22, (uint8_t)174, (uint8_t)86, (uint8_t)9, (uint8_t)101, (uint8_t)236, (uint8_t)247, (uint8_t)135, (uint8_t)144, (uint8_t)246, (uint8_t)165, (uint8_t)78, (uint8_t)149, (uint8_t)229, (uint8_t)160, (uint8_t)204, (uint8_t)175, (uint8_t)142, (uint8_t)60, (uint8_t)218, (uint8_t)143, (uint8_t)7, (uint8_t)29, (uint8_t)86, (uint8_t)166, (uint8_t)35, (uint8_t)208, (uint8_t)117, (uint8_t)211, (uint8_t)11, (uint8_t)158, (uint8_t)26, (uint8_t)242, (uint8_t)158, (uint8_t)187, (uint8_t)103, (uint8_t)119, (uint8_t)117, (uint8_t)33, (uint8_t)122, (uint8_t)86, (uint8_t)107, (uint8_t)9, (uint8_t)12, (uint8_t)32, (uint8_t)128, (uint8_t)29, (uint8_t)170, (uint8_t)111, (uint8_t)17, (uint8_t)106, (uint8_t)227, (uint8_t)161, (uint8_t)213, (uint8_t)205, (uint8_t)120, (uint8_t)105, (uint8_t)4, (uint8_t)84, (uint8_t)240, (uint8_t)89, (uint8_t)58, (uint8_t)87, (uint8_t)63, (uint8_t)77, (uint8_t)73, (uint8_t)160, (uint8_t)166, (uint8_t)156, (uint8_t)101, (uint8_t)44, (uint8_t)32, (uint8_t)84, (uint8_t)166, (uint8_t)233, (uint8_t)214, (uint8_t)255, (uint8_t)37, (uint8_t)113, (uint8_t)71, (uint8_t)192, (uint8_t)177, (uint8_t)14, (uint8_t)143, (uint8_t)223, (uint8_t)214, (uint8_t)150, (uint8_t)16, (uint8_t)156, (uint8_t)182, (uint8_t)12, (uint8_t)247, (uint8_t)61, (uint8_t)142, (uint8_t)110, (uint8_t)176, (uint8_t)33, (uint8_t)144, (uint8_t)42, (uint8_t)132, (uint8_t)184, (uint8_t)201, (uint8_t)20, (uint8_t)195, (uint8_t)69, (uint8_t)108, (uint8_t)36, (uint8_t)243, (uint8_t)207, (uint8_t)158, (uint8_t)156, (uint8_t)172, (uint8_t)185, (uint8_t)135, (uint8_t)82, (uint8_t)0, (uint8_t)149};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_time_boot_ms_SET((uint32_t)3083435410L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)30518, PH.base.pack) ;
        p143_press_diff_SET((float) -2.1479976E38F, PH.base.pack) ;
        p143_press_abs_SET((float) -6.7409283E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float position_cov[] =  {9.772088E37F, 2.6677384E38F, -2.5802608E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-6.2203727E37F, 1.2391125E38F, 1.9576032E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)1.8822642E38F, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p144_lon_SET((int32_t)1233725301, PH.base.pack) ;
        {
            float rates[] =  {4.277534E37F, 1.9245123E38F, 3.0287014E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {3.1998787E38F, 1.4189346E38F, 1.1006192E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)1027662723932484674L, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)7611626468207190935L, PH.base.pack) ;
        p144_lat_SET((int32_t) -1381031714, PH.base.pack) ;
        {
            float attitude_q[] =  {1.1560682E38F, -1.9940371E38F, -1.7369639E38F, -8.566889E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        {
            float vel_variance[] =  {-1.6951348E38F, 1.481384E37F, 3.254648E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float) -3.2774881E38F, PH.base.pack) ;
        p146_roll_rate_SET((float)2.5473128E38F, PH.base.pack) ;
        {
            float q[] =  {-2.9168758E38F, 1.5810412E38F, 1.2234324E38F, 3.3707228E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float pos_variance[] =  {2.9139796E37F, 3.9929627E37F, -1.0108796E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float)8.562535E37F, PH.base.pack) ;
        p146_z_acc_SET((float) -3.173627E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -1.4504239E38F, PH.base.pack) ;
        p146_y_pos_SET((float)2.18816E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -2.5226515E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -2.8709264E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -1.1209093E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -8.295231E37F, PH.base.pack) ;
        p146_x_pos_SET((float)2.8958539E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -1.5234298E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.1658708E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)4137036009367442065L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t)917116290, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -122, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -1681, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)946863824, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)59731, (uint16_t)44355, (uint16_t)51226, (uint16_t)17151, (uint16_t)7097, (uint16_t)442, (uint16_t)6500, (uint16_t)61985, (uint16_t)25856, (uint16_t)12393};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_temperature_SET((int16_t)(int16_t) -32683, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_product_id_SET((uint16_t)(uint16_t)13420, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3737757436L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)2232875071L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)41751, PH.base.pack) ;
        p148_board_version_SET((uint32_t)2132612289L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)177, (uint8_t)198, (uint8_t)103, (uint8_t)102, (uint8_t)88, (uint8_t)164, (uint8_t)213, (uint8_t)102};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)7028536797449197557L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)57, (uint8_t)123, (uint8_t)69, (uint8_t)12, (uint8_t)114, (uint8_t)86, (uint8_t)219, (uint8_t)114, (uint8_t)170, (uint8_t)28, (uint8_t)230, (uint8_t)225, (uint8_t)44, (uint8_t)27, (uint8_t)53, (uint8_t)34, (uint8_t)9, (uint8_t)112};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)64, (uint8_t)219, (uint8_t)146, (uint8_t)118, (uint8_t)254, (uint8_t)197, (uint8_t)3, (uint8_t)43};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)1834184794L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)173, (uint8_t)39, (uint8_t)75, (uint8_t)58, (uint8_t)177, (uint8_t)200, (uint8_t)171, (uint8_t)16};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_size_x_SET((float) -2.919885E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
        p149_angle_y_SET((float) -6.7220246E37F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)180, &PH) ;
        p149_angle_x_SET((float) -7.7403334E37F, PH.base.pack) ;
        {
            float q[] =  {-1.0330466E38F, 7.550206E37F, 1.8218115E38F, 1.6707741E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_x_SET((float)3.3175824E38F, &PH) ;
        p149_distance_SET((float)1.7257708E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p149_y_SET((float)3.240437E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p149_z_SET((float) -3.0467015E38F, &PH) ;
        p149_size_y_SET((float)8.662373E37F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)4826716991536228529L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_0_150(), &PH);
        {
            int8_t ar_i8[] =  {(int8_t)91, (int8_t) -111, (int8_t)109, (int8_t)29};
            p150_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)49633, (uint16_t)25017, (uint16_t)54860, (uint16_t)13742};
            p150_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        p150_v1_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        {
            uint8_t ar_u8[] =  {(uint8_t)158, (uint8_t)204, (uint8_t)48, (uint8_t)107};
            p150_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            uint32_t ar_u32[] =  {1070103093L, 2573278646L, 3837541876L, 3678985243L};
            p150_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_0_150(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_1_151(), &PH);
        {
            uint32_t ar_u32[] =  {2559956767L, 1809613838L, 651767041L, 791995732L};
            p151_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_1_151(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_3_153(), &PH);
        {
            uint32_t ar_u32[] =  {3058757168L, 934383469L, 2919727653L, 653147222L};
            p153_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        p153_v_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ARRAY_TEST_3_153(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_4_154(), &PH);
        {
            uint32_t ar_u32[] =  {2275405475L, 2192856120L, 1685695046L, 2461895618L};
            p154_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        p154_v_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ARRAY_TEST_4_154(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_5_155(), &PH);
        {
            char16_t* c2 = u"b";
            p155_c2_SET_(c2, &PH) ;
        }
        {
            char16_t* c1 = u"p";
            p155_c1_SET_(c1, &PH) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_5_155(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_6_156(), &PH);
        p156_v2_SET((uint16_t)(uint16_t)2362, PH.base.pack) ;
        {
            uint8_t ar_u8[] =  {(uint8_t)102, (uint8_t)218};
            p156_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t) -22, (int8_t)56};
            p156_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            char16_t* ar_c = u"jaHgnlvrqodqecpcggymqajUn";
            p156_ar_c_SET_(ar_c, &PH) ;
        }
        {
            int32_t ar_i32[] =  {-212590080, 52844573};
            p156_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        {
            float ar_f[] =  {-2.359192E38F, -2.8764266E38F};
            p156_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            int16_t ar_i16[] =  {(int16_t)22494, (int16_t)30641};
            p156_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        p156_v3_SET((uint32_t)1244075667L, PH.base.pack) ;
        {
            uint32_t ar_u32[] =  {944175862L, 3191762652L};
            p156_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)49006, (uint16_t)50737};
            p156_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        p156_v1_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        {
            double ar_d[] =  {1.7915369678372592E307, -8.957701084993507E307};
            p156_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_6_156(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_7_157(), &PH);
        {
            int16_t ar_i16[] =  {(int16_t) -647, (int16_t)16549};
            p157_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)32967, (uint16_t)57113};
            p157_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            float ar_f[] =  {-1.0777216E38F, 2.8393305E38F};
            p157_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            int32_t ar_i32[] =  {-2055682717, -93782493};
            p157_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        {
            char16_t* ar_c = u"lgWze";
            p157_ar_c_SET_(ar_c, &PH) ;
        }
        {
            uint8_t ar_u8[] =  {(uint8_t)117, (uint8_t)183};
            p157_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t) -98, (int8_t) -121};
            p157_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            uint32_t ar_u32[] =  {1821499610L, 2981607307L};
            p157_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        {
            double ar_d[] =  {-1.2869180416484966E308, -5.170226971360175E307};
            p157_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_7_157(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_8_158(), &PH);
        p158_v3_SET((uint32_t)1882202859L, PH.base.pack) ;
        {
            double ar_d[] =  {1.771128595392106E308, -1.1811289097694477E308};
            p158_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)55297, (uint16_t)40354};
            p158_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ARRAY_TEST_8_158(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.5244018E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -3.7206962E37F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -3.0794724E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)2.2527526E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float)2.606916E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -1.1257258E36F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)7210861050112885033L, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -2.187133E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.4302764E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_var_horiz_SET((float) -7.455072E37F, PH.base.pack) ;
        p231_wind_x_SET((float) -3.1457865E37F, PH.base.pack) ;
        p231_wind_y_SET((float) -3.070166E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -2.083737E37F, PH.base.pack) ;
        p231_wind_alt_SET((float)2.3766534E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)7147062452474096228L, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -2.5385588E38F, PH.base.pack) ;
        p231_wind_z_SET((float)2.1160208E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -3.0420344E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_vn_SET((float) -1.1732524E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.6892732E36F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)3.6212258E35F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)7265021984978685924L, PH.base.pack) ;
        p232_ve_SET((float)9.775556E37F, PH.base.pack) ;
        p232_lat_SET((int32_t) -1097047024, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p232_lon_SET((int32_t)1583100332, PH.base.pack) ;
        p232_vdop_SET((float) -1.7152412E38F, PH.base.pack) ;
        p232_hdop_SET((float) -1.4263299E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)31980, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2759792741L, PH.base.pack) ;
        p232_vd_SET((float)3.1808927E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -3.2703138E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT, PH.base.pack) ;
        p232_alt_SET((float) -2.1943734E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)92, (uint8_t)136, (uint8_t)63, (uint8_t)100, (uint8_t)177, (uint8_t)98, (uint8_t)252, (uint8_t)113, (uint8_t)241, (uint8_t)176, (uint8_t)111, (uint8_t)93, (uint8_t)196, (uint8_t)91, (uint8_t)89, (uint8_t)63, (uint8_t)117, (uint8_t)246, (uint8_t)162, (uint8_t)102, (uint8_t)75, (uint8_t)66, (uint8_t)241, (uint8_t)179, (uint8_t)233, (uint8_t)160, (uint8_t)244, (uint8_t)132, (uint8_t)224, (uint8_t)49, (uint8_t)211, (uint8_t)99, (uint8_t)143, (uint8_t)92, (uint8_t)52, (uint8_t)31, (uint8_t)233, (uint8_t)153, (uint8_t)175, (uint8_t)28, (uint8_t)105, (uint8_t)159, (uint8_t)33, (uint8_t)235, (uint8_t)254, (uint8_t)10, (uint8_t)121, (uint8_t)69, (uint8_t)106, (uint8_t)101, (uint8_t)143, (uint8_t)184, (uint8_t)142, (uint8_t)48, (uint8_t)23, (uint8_t)201, (uint8_t)178, (uint8_t)147, (uint8_t)200, (uint8_t)75, (uint8_t)3, (uint8_t)4, (uint8_t)133, (uint8_t)226, (uint8_t)142, (uint8_t)241, (uint8_t)155, (uint8_t)57, (uint8_t)29, (uint8_t)139, (uint8_t)115, (uint8_t)240, (uint8_t)140, (uint8_t)244, (uint8_t)12, (uint8_t)172, (uint8_t)48, (uint8_t)253, (uint8_t)85, (uint8_t)57, (uint8_t)44, (uint8_t)229, (uint8_t)236, (uint8_t)41, (uint8_t)9, (uint8_t)186, (uint8_t)153, (uint8_t)32, (uint8_t)46, (uint8_t)26, (uint8_t)83, (uint8_t)1, (uint8_t)210, (uint8_t)178, (uint8_t)104, (uint8_t)79, (uint8_t)137, (uint8_t)56, (uint8_t)81, (uint8_t)70, (uint8_t)122, (uint8_t)96, (uint8_t)255, (uint8_t)175, (uint8_t)95, (uint8_t)47, (uint8_t)67, (uint8_t)231, (uint8_t)232, (uint8_t)200, (uint8_t)120, (uint8_t)21, (uint8_t)166, (uint8_t)254, (uint8_t)129, (uint8_t)54, (uint8_t)176, (uint8_t)62, (uint8_t)120, (uint8_t)212, (uint8_t)60, (uint8_t)75, (uint8_t)81, (uint8_t)84, (uint8_t)79, (uint8_t)41, (uint8_t)130, (uint8_t)95, (uint8_t)38, (uint8_t)240, (uint8_t)31, (uint8_t)140, (uint8_t)89, (uint8_t)47, (uint8_t)85, (uint8_t)254, (uint8_t)96, (uint8_t)161, (uint8_t)46, (uint8_t)91, (uint8_t)50, (uint8_t)160, (uint8_t)77, (uint8_t)25, (uint8_t)171, (uint8_t)123, (uint8_t)63, (uint8_t)105, (uint8_t)245, (uint8_t)28, (uint8_t)36, (uint8_t)183, (uint8_t)206, (uint8_t)108, (uint8_t)0, (uint8_t)159, (uint8_t)18, (uint8_t)162, (uint8_t)40, (uint8_t)234, (uint8_t)103, (uint8_t)6, (uint8_t)240, (uint8_t)47, (uint8_t)67, (uint8_t)173, (uint8_t)171, (uint8_t)58, (uint8_t)226, (uint8_t)169, (uint8_t)200, (uint8_t)155, (uint8_t)190, (uint8_t)40, (uint8_t)59, (uint8_t)199, (uint8_t)138, (uint8_t)178, (uint8_t)67, (uint8_t)153};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_airspeed_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p234_latitude_SET((int32_t)573416631, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)24759, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3803848859L, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -43, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -98, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)3345, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)5200, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)59918, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -116, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -70, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1479518463, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)19136, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)1267, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)24404, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_clipping_0_SET((uint32_t)2510905696L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)386306982L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3224094308L, PH.base.pack) ;
        p241_vibration_x_SET((float) -2.4007615E38F, PH.base.pack) ;
        p241_vibration_z_SET((float) -5.477924E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)7742573720835496418L, PH.base.pack) ;
        p241_vibration_y_SET((float)2.9600155E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_latitude_SET((int32_t)1544369251, PH.base.pack) ;
        p242_approach_x_SET((float) -7.440782E37F, PH.base.pack) ;
        p242_z_SET((float) -4.887864E37F, PH.base.pack) ;
        p242_longitude_SET((int32_t)1655230131, PH.base.pack) ;
        p242_y_SET((float)1.4409693E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)2036810561, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)4386107775584825145L, &PH) ;
        p242_approach_z_SET((float) -2.68449E38F, PH.base.pack) ;
        p242_approach_y_SET((float)1.8712407E38F, PH.base.pack) ;
        p242_x_SET((float)1.4198509E38F, PH.base.pack) ;
        {
            float q[] =  {-2.0932503E38F, -1.9481652E38F, -2.7686018E38F, -1.9022823E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_x_SET((float)1.5302499E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -3.0660059E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)1268911772, PH.base.pack) ;
        p243_latitude_SET((int32_t) -490573467, PH.base.pack) ;
        p243_approach_z_SET((float) -5.9370953E37F, PH.base.pack) ;
        p243_y_SET((float)2.2656707E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)2128536564805702523L, &PH) ;
        p243_longitude_SET((int32_t)743907440, PH.base.pack) ;
        p243_z_SET((float) -1.0746817E38F, PH.base.pack) ;
        p243_x_SET((float)2.4213686E37F, PH.base.pack) ;
        {
            float q[] =  {-4.8809913E36F, -2.476255E38F, -1.9203091E38F, 2.5105031E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)55325, PH.base.pack) ;
        p244_interval_us_SET((int32_t)627514319, PH.base.pack) ;
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
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)41468, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2789168372L, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)21783, PH.base.pack) ;
        p246_altitude_SET((int32_t)1393761085, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)7451, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS, PH.base.pack) ;
        {
            char16_t* callsign = u"uxiheqq";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_tslc_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p246_lon_SET((int32_t) -2089112037, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)38134, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE, PH.base.pack) ;
        p246_lat_SET((int32_t)1941730077, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -3.2800356E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.421376E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)2610360599L, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -2.2732352E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_message_type_SET((uint16_t)(uint16_t)30133, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)160, (uint8_t)67, (uint8_t)93, (uint8_t)91, (uint8_t)199, (uint8_t)119, (uint8_t)122, (uint8_t)64, (uint8_t)46, (uint8_t)17, (uint8_t)37, (uint8_t)34, (uint8_t)165, (uint8_t)214, (uint8_t)216, (uint8_t)77, (uint8_t)110, (uint8_t)251, (uint8_t)112, (uint8_t)75, (uint8_t)255, (uint8_t)149, (uint8_t)244, (uint8_t)131, (uint8_t)228, (uint8_t)212, (uint8_t)173, (uint8_t)230, (uint8_t)132, (uint8_t)6, (uint8_t)20, (uint8_t)72, (uint8_t)11, (uint8_t)201, (uint8_t)11, (uint8_t)85, (uint8_t)45, (uint8_t)35, (uint8_t)11, (uint8_t)223, (uint8_t)117, (uint8_t)191, (uint8_t)189, (uint8_t)78, (uint8_t)6, (uint8_t)47, (uint8_t)52, (uint8_t)198, (uint8_t)122, (uint8_t)114, (uint8_t)59, (uint8_t)22, (uint8_t)146, (uint8_t)151, (uint8_t)236, (uint8_t)23, (uint8_t)129, (uint8_t)196, (uint8_t)119, (uint8_t)143, (uint8_t)235, (uint8_t)48, (uint8_t)142, (uint8_t)206, (uint8_t)57, (uint8_t)99, (uint8_t)200, (uint8_t)142, (uint8_t)196, (uint8_t)197, (uint8_t)65, (uint8_t)204, (uint8_t)224, (uint8_t)41, (uint8_t)155, (uint8_t)223, (uint8_t)137, (uint8_t)54, (uint8_t)62, (uint8_t)201, (uint8_t)183, (uint8_t)107, (uint8_t)112, (uint8_t)222, (uint8_t)235, (uint8_t)167, (uint8_t)186, (uint8_t)166, (uint8_t)49, (uint8_t)49, (uint8_t)216, (uint8_t)0, (uint8_t)250, (uint8_t)217, (uint8_t)209, (uint8_t)83, (uint8_t)191, (uint8_t)242, (uint8_t)217, (uint8_t)136, (uint8_t)42, (uint8_t)139, (uint8_t)149, (uint8_t)166, (uint8_t)22, (uint8_t)200, (uint8_t)104, (uint8_t)177, (uint8_t)18, (uint8_t)220, (uint8_t)204, (uint8_t)170, (uint8_t)180, (uint8_t)186, (uint8_t)132, (uint8_t)233, (uint8_t)155, (uint8_t)210, (uint8_t)233, (uint8_t)105, (uint8_t)189, (uint8_t)104, (uint8_t)92, (uint8_t)178, (uint8_t)193, (uint8_t)150, (uint8_t)110, (uint8_t)130, (uint8_t)63, (uint8_t)166, (uint8_t)220, (uint8_t)34, (uint8_t)236, (uint8_t)21, (uint8_t)137, (uint8_t)151, (uint8_t)186, (uint8_t)71, (uint8_t)150, (uint8_t)19, (uint8_t)48, (uint8_t)17, (uint8_t)136, (uint8_t)137, (uint8_t)239, (uint8_t)237, (uint8_t)63, (uint8_t)242, (uint8_t)226, (uint8_t)202, (uint8_t)9, (uint8_t)247, (uint8_t)99, (uint8_t)4, (uint8_t)65, (uint8_t)142, (uint8_t)153, (uint8_t)121, (uint8_t)127, (uint8_t)105, (uint8_t)147, (uint8_t)234, (uint8_t)112, (uint8_t)73, (uint8_t)68, (uint8_t)164, (uint8_t)214, (uint8_t)218, (uint8_t)169, (uint8_t)77, (uint8_t)56, (uint8_t)191, (uint8_t)145, (uint8_t)63, (uint8_t)173, (uint8_t)200, (uint8_t)166, (uint8_t)211, (uint8_t)237, (uint8_t)60, (uint8_t)144, (uint8_t)213, (uint8_t)236, (uint8_t)178, (uint8_t)227, (uint8_t)231, (uint8_t)127, (uint8_t)117, (uint8_t)210, (uint8_t)111, (uint8_t)27, (uint8_t)17, (uint8_t)51, (uint8_t)111, (uint8_t)136, (uint8_t)74, (uint8_t)167, (uint8_t)2, (uint8_t)165, (uint8_t)33, (uint8_t)224, (uint8_t)35, (uint8_t)51, (uint8_t)136, (uint8_t)216, (uint8_t)170, (uint8_t)242, (uint8_t)88, (uint8_t)245, (uint8_t)123, (uint8_t)23, (uint8_t)1, (uint8_t)82, (uint8_t)131, (uint8_t)2, (uint8_t)178, (uint8_t)151, (uint8_t)7, (uint8_t)58, (uint8_t)155, (uint8_t)212, (uint8_t)98, (uint8_t)111, (uint8_t)151, (uint8_t)165, (uint8_t)184, (uint8_t)40, (uint8_t)209, (uint8_t)163, (uint8_t)117, (uint8_t)120, (uint8_t)154, (uint8_t)247, (uint8_t)45, (uint8_t)129, (uint8_t)180, (uint8_t)179, (uint8_t)19, (uint8_t)9, (uint8_t)111, (uint8_t)241, (uint8_t)124, (uint8_t)159, (uint8_t)209, (uint8_t)6, (uint8_t)253, (uint8_t)107, (uint8_t)194, (uint8_t)149};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)30556, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -65, (int8_t)94, (int8_t)27, (int8_t)6, (int8_t) -18, (int8_t)59, (int8_t)14, (int8_t)41, (int8_t) -105, (int8_t)72, (int8_t)46, (int8_t)121, (int8_t) -53, (int8_t) -22, (int8_t) -81, (int8_t)72, (int8_t)99, (int8_t) -72, (int8_t) -43, (int8_t)82, (int8_t)92, (int8_t)109, (int8_t)109, (int8_t) -23, (int8_t)58, (int8_t)54, (int8_t)65, (int8_t)35, (int8_t)79, (int8_t)32, (int8_t)59, (int8_t) -24};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_z_SET((float) -1.7058105E38F, PH.base.pack) ;
        {
            char16_t* name = u"uGltjgawp";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)4321880139608474991L, PH.base.pack) ;
        p250_x_SET((float) -1.6519366E38F, PH.base.pack) ;
        p250_y_SET((float)3.272026E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)308342692L, PH.base.pack) ;
        p251_value_SET((float) -2.988002E38F, PH.base.pack) ;
        {
            char16_t* name = u"mxmnxfJ";
            p251_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"gyezkual";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)729674006L, PH.base.pack) ;
        p252_value_SET((int32_t) -326393567, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"nKpymcvkaimvijwgvedaueibmapjzemBbmwuqgvtqco";
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
        p254_ind_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)4256164049L, PH.base.pack) ;
        p254_value_SET((float) -2.033366E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        {
            uint8_t secret_key[] =  {(uint8_t)129, (uint8_t)136, (uint8_t)62, (uint8_t)216, (uint8_t)25, (uint8_t)176, (uint8_t)92, (uint8_t)240, (uint8_t)36, (uint8_t)95, (uint8_t)203, (uint8_t)143, (uint8_t)59, (uint8_t)231, (uint8_t)188, (uint8_t)64, (uint8_t)30, (uint8_t)252, (uint8_t)150, (uint8_t)41, (uint8_t)29, (uint8_t)122, (uint8_t)22, (uint8_t)166, (uint8_t)28, (uint8_t)106, (uint8_t)198, (uint8_t)201, (uint8_t)241, (uint8_t)38, (uint8_t)100, (uint8_t)35};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)6495626601099691128L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)2474077378L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)2459288216L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        {
            char16_t* tune = u"pfdwaaftEmux";
            p258_tune_SET_(tune, &PH) ;
        }
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_v_SET((float)3.390025E38F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)1990059433L, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)46310, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
        p259_sensor_size_h_SET((float)1.0271349E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)33, (uint8_t)158, (uint8_t)208, (uint8_t)62, (uint8_t)142, (uint8_t)109, (uint8_t)107, (uint8_t)11, (uint8_t)18, (uint8_t)95, (uint8_t)62, (uint8_t)2, (uint8_t)13, (uint8_t)28, (uint8_t)52, (uint8_t)117, (uint8_t)143, (uint8_t)113, (uint8_t)205, (uint8_t)196, (uint8_t)150, (uint8_t)94, (uint8_t)236, (uint8_t)46, (uint8_t)170, (uint8_t)131, (uint8_t)209, (uint8_t)75, (uint8_t)87, (uint8_t)63, (uint8_t)81, (uint8_t)151};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        {
            uint8_t vendor_name[] =  {(uint8_t)31, (uint8_t)35, (uint8_t)24, (uint8_t)71, (uint8_t)208, (uint8_t)208, (uint8_t)90, (uint8_t)63, (uint8_t)218, (uint8_t)207, (uint8_t)15, (uint8_t)240, (uint8_t)184, (uint8_t)217, (uint8_t)95, (uint8_t)178, (uint8_t)108, (uint8_t)11, (uint8_t)7, (uint8_t)51, (uint8_t)50, (uint8_t)83, (uint8_t)19, (uint8_t)232, (uint8_t)235, (uint8_t)39, (uint8_t)90, (uint8_t)136, (uint8_t)141, (uint8_t)35, (uint8_t)254, (uint8_t)15};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_focal_length_SET((float) -5.0221503E37F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)14542, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1999852364L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)51271, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"ohubidjGsBihMkjgiHtofOqkgyxjppBrgaOrlk";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)2513621686L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float) -9.061751E37F, PH.base.pack) ;
        p261_write_speed_SET((float) -1.9443896E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)4021656444L, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.4259675E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.0573385E38F, PH.base.pack) ;
        p261_read_speed_SET((float)2.9980789E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_video_status_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)1261418963L, PH.base.pack) ;
        p262_image_interval_SET((float) -2.9164125E37F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)779200727L, PH.base.pack) ;
        p262_available_capacity_SET((float)3.1788111E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_image_index_SET((int32_t)543732767, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)582492926L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)1472755315, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -124, PH.base.pack) ;
        {
            char16_t* file_url = u"eevnvzvfntgkywxehtusRxohbeTeltxbexhfsygmgvgxbygwDlxfyfdfpnzQPlzulofcigqsgrrgiezslwfmrOBficrljurYwyxdsrpwyLxgdxybebVqwdjWtpkizOvHqbPtXsruIwezUYgYeceubqezuminddutdmhgXldyukkrnriytrybe";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_alt_SET((int32_t)1991231729, PH.base.pack) ;
        p263_lat_SET((int32_t) -2016475026, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p263_lon_SET((int32_t) -212802312, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)3102988958579850518L, PH.base.pack) ;
        {
            float q[] =  {6.968933E36F, 2.5623782E38F, -1.2017734E38F, -8.678639E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)1838320977141573217L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)6143084963745794454L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2048609765L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)981807421516801206L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)723413627L, PH.base.pack) ;
        p265_yaw_SET((float)2.4084603E38F, PH.base.pack) ;
        p265_pitch_SET((float)3.269327E38F, PH.base.pack) ;
        p265_roll_SET((float) -8.803456E35F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)186, (uint8_t)234, (uint8_t)34, (uint8_t)229, (uint8_t)203, (uint8_t)146, (uint8_t)3, (uint8_t)53, (uint8_t)229, (uint8_t)195, (uint8_t)195, (uint8_t)186, (uint8_t)92, (uint8_t)71, (uint8_t)106, (uint8_t)174, (uint8_t)225, (uint8_t)203, (uint8_t)228, (uint8_t)113, (uint8_t)246, (uint8_t)130, (uint8_t)20, (uint8_t)161, (uint8_t)178, (uint8_t)8, (uint8_t)251, (uint8_t)72, (uint8_t)125, (uint8_t)179, (uint8_t)68, (uint8_t)53, (uint8_t)56, (uint8_t)164, (uint8_t)211, (uint8_t)78, (uint8_t)119, (uint8_t)251, (uint8_t)197, (uint8_t)54, (uint8_t)117, (uint8_t)91, (uint8_t)251, (uint8_t)111, (uint8_t)73, (uint8_t)98, (uint8_t)25, (uint8_t)100, (uint8_t)75, (uint8_t)128, (uint8_t)249, (uint8_t)52, (uint8_t)6, (uint8_t)252, (uint8_t)10, (uint8_t)191, (uint8_t)26, (uint8_t)112, (uint8_t)176, (uint8_t)149, (uint8_t)139, (uint8_t)135, (uint8_t)223, (uint8_t)160, (uint8_t)133, (uint8_t)2, (uint8_t)8, (uint8_t)145, (uint8_t)92, (uint8_t)222, (uint8_t)10, (uint8_t)149, (uint8_t)129, (uint8_t)178, (uint8_t)119, (uint8_t)61, (uint8_t)138, (uint8_t)26, (uint8_t)194, (uint8_t)6, (uint8_t)20, (uint8_t)213, (uint8_t)250, (uint8_t)112, (uint8_t)225, (uint8_t)150, (uint8_t)53, (uint8_t)101, (uint8_t)26, (uint8_t)85, (uint8_t)226, (uint8_t)95, (uint8_t)69, (uint8_t)99, (uint8_t)106, (uint8_t)2, (uint8_t)197, (uint8_t)83, (uint8_t)44, (uint8_t)168, (uint8_t)218, (uint8_t)223, (uint8_t)169, (uint8_t)94, (uint8_t)96, (uint8_t)172, (uint8_t)152, (uint8_t)130, (uint8_t)28, (uint8_t)191, (uint8_t)52, (uint8_t)197, (uint8_t)192, (uint8_t)249, (uint8_t)124, (uint8_t)231, (uint8_t)102, (uint8_t)43, (uint8_t)4, (uint8_t)253, (uint8_t)61, (uint8_t)165, (uint8_t)167, (uint8_t)148, (uint8_t)197, (uint8_t)63, (uint8_t)119, (uint8_t)222, (uint8_t)56, (uint8_t)19, (uint8_t)88, (uint8_t)79, (uint8_t)130, (uint8_t)120, (uint8_t)58, (uint8_t)203, (uint8_t)128, (uint8_t)15, (uint8_t)178, (uint8_t)23, (uint8_t)210, (uint8_t)112, (uint8_t)63, (uint8_t)237, (uint8_t)100, (uint8_t)240, (uint8_t)119, (uint8_t)217, (uint8_t)191, (uint8_t)85, (uint8_t)141, (uint8_t)60, (uint8_t)166, (uint8_t)100, (uint8_t)158, (uint8_t)227, (uint8_t)53, (uint8_t)128, (uint8_t)7, (uint8_t)190, (uint8_t)220, (uint8_t)68, (uint8_t)215, (uint8_t)136, (uint8_t)15, (uint8_t)231, (uint8_t)56, (uint8_t)39, (uint8_t)199, (uint8_t)195, (uint8_t)140, (uint8_t)17, (uint8_t)86, (uint8_t)232, (uint8_t)177, (uint8_t)203, (uint8_t)205, (uint8_t)230, (uint8_t)138, (uint8_t)235, (uint8_t)166, (uint8_t)24, (uint8_t)220, (uint8_t)163, (uint8_t)193, (uint8_t)195, (uint8_t)82, (uint8_t)78, (uint8_t)141, (uint8_t)99, (uint8_t)69, (uint8_t)182, (uint8_t)17, (uint8_t)141, (uint8_t)54, (uint8_t)84, (uint8_t)99, (uint8_t)167, (uint8_t)230, (uint8_t)115, (uint8_t)76, (uint8_t)171, (uint8_t)132, (uint8_t)147, (uint8_t)71, (uint8_t)86, (uint8_t)167, (uint8_t)179, (uint8_t)244, (uint8_t)36, (uint8_t)79, (uint8_t)191, (uint8_t)247, (uint8_t)23, (uint8_t)56, (uint8_t)67, (uint8_t)51, (uint8_t)71, (uint8_t)3, (uint8_t)128, (uint8_t)27, (uint8_t)99, (uint8_t)51, (uint8_t)215, (uint8_t)164, (uint8_t)24, (uint8_t)100, (uint8_t)88, (uint8_t)158, (uint8_t)77, (uint8_t)51, (uint8_t)78, (uint8_t)59, (uint8_t)109, (uint8_t)167, (uint8_t)223, (uint8_t)164, (uint8_t)193, (uint8_t)75, (uint8_t)168, (uint8_t)213, (uint8_t)242, (uint8_t)142, (uint8_t)224, (uint8_t)180, (uint8_t)77, (uint8_t)60, (uint8_t)116, (uint8_t)26};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)20746, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)246, (uint8_t)238, (uint8_t)93, (uint8_t)228, (uint8_t)144, (uint8_t)2, (uint8_t)170, (uint8_t)252, (uint8_t)57, (uint8_t)209, (uint8_t)114, (uint8_t)20, (uint8_t)118, (uint8_t)194, (uint8_t)10, (uint8_t)52, (uint8_t)39, (uint8_t)14, (uint8_t)80, (uint8_t)62, (uint8_t)30, (uint8_t)189, (uint8_t)175, (uint8_t)41, (uint8_t)174, (uint8_t)203, (uint8_t)213, (uint8_t)239, (uint8_t)123, (uint8_t)88, (uint8_t)219, (uint8_t)135, (uint8_t)227, (uint8_t)61, (uint8_t)231, (uint8_t)66, (uint8_t)111, (uint8_t)39, (uint8_t)234, (uint8_t)14, (uint8_t)62, (uint8_t)129, (uint8_t)152, (uint8_t)166, (uint8_t)154, (uint8_t)255, (uint8_t)69, (uint8_t)101, (uint8_t)30, (uint8_t)50, (uint8_t)1, (uint8_t)142, (uint8_t)19, (uint8_t)255, (uint8_t)144, (uint8_t)42, (uint8_t)89, (uint8_t)208, (uint8_t)164, (uint8_t)232, (uint8_t)121, (uint8_t)207, (uint8_t)243, (uint8_t)225, (uint8_t)49, (uint8_t)172, (uint8_t)224, (uint8_t)107, (uint8_t)75, (uint8_t)109, (uint8_t)7, (uint8_t)19, (uint8_t)76, (uint8_t)116, (uint8_t)68, (uint8_t)64, (uint8_t)25, (uint8_t)144, (uint8_t)217, (uint8_t)255, (uint8_t)212, (uint8_t)65, (uint8_t)111, (uint8_t)184, (uint8_t)63, (uint8_t)210, (uint8_t)54, (uint8_t)58, (uint8_t)5, (uint8_t)208, (uint8_t)6, (uint8_t)10, (uint8_t)112, (uint8_t)168, (uint8_t)174, (uint8_t)241, (uint8_t)217, (uint8_t)8, (uint8_t)164, (uint8_t)39, (uint8_t)3, (uint8_t)22, (uint8_t)242, (uint8_t)72, (uint8_t)191, (uint8_t)137, (uint8_t)166, (uint8_t)11, (uint8_t)32, (uint8_t)54, (uint8_t)58, (uint8_t)197, (uint8_t)187, (uint8_t)14, (uint8_t)203, (uint8_t)246, (uint8_t)23, (uint8_t)149, (uint8_t)206, (uint8_t)162, (uint8_t)97, (uint8_t)252, (uint8_t)141, (uint8_t)201, (uint8_t)186, (uint8_t)217, (uint8_t)105, (uint8_t)194, (uint8_t)187, (uint8_t)48, (uint8_t)16, (uint8_t)175, (uint8_t)148, (uint8_t)106, (uint8_t)40, (uint8_t)115, (uint8_t)82, (uint8_t)8, (uint8_t)239, (uint8_t)193, (uint8_t)27, (uint8_t)219, (uint8_t)109, (uint8_t)85, (uint8_t)63, (uint8_t)47, (uint8_t)63, (uint8_t)178, (uint8_t)34, (uint8_t)166, (uint8_t)225, (uint8_t)117, (uint8_t)110, (uint8_t)53, (uint8_t)17, (uint8_t)149, (uint8_t)121, (uint8_t)34, (uint8_t)198, (uint8_t)175, (uint8_t)35, (uint8_t)146, (uint8_t)225, (uint8_t)221, (uint8_t)72, (uint8_t)169, (uint8_t)71, (uint8_t)240, (uint8_t)252, (uint8_t)216, (uint8_t)120, (uint8_t)67, (uint8_t)210, (uint8_t)84, (uint8_t)30, (uint8_t)126, (uint8_t)195, (uint8_t)32, (uint8_t)204, (uint8_t)217, (uint8_t)161, (uint8_t)84, (uint8_t)44, (uint8_t)19, (uint8_t)190, (uint8_t)120, (uint8_t)23, (uint8_t)247, (uint8_t)182, (uint8_t)75, (uint8_t)60, (uint8_t)60, (uint8_t)0, (uint8_t)141, (uint8_t)98, (uint8_t)158, (uint8_t)163, (uint8_t)217, (uint8_t)41, (uint8_t)231, (uint8_t)237, (uint8_t)32, (uint8_t)59, (uint8_t)187, (uint8_t)175, (uint8_t)115, (uint8_t)95, (uint8_t)161, (uint8_t)72, (uint8_t)95, (uint8_t)228, (uint8_t)111, (uint8_t)216, (uint8_t)84, (uint8_t)129, (uint8_t)66, (uint8_t)45, (uint8_t)98, (uint8_t)235, (uint8_t)68, (uint8_t)19, (uint8_t)93, (uint8_t)191, (uint8_t)40, (uint8_t)68, (uint8_t)192, (uint8_t)251, (uint8_t)185, (uint8_t)250, (uint8_t)221, (uint8_t)22, (uint8_t)247, (uint8_t)2, (uint8_t)22, (uint8_t)55, (uint8_t)28, (uint8_t)248, (uint8_t)156, (uint8_t)165, (uint8_t)161, (uint8_t)90, (uint8_t)0, (uint8_t)38, (uint8_t)100, (uint8_t)71, (uint8_t)210, (uint8_t)29, (uint8_t)86, (uint8_t)53};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)58139, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)14295, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_bitrate_SET((uint32_t)2548008596L, PH.base.pack) ;
        p269_framerate_SET((float) -3.063106E38F, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)62303, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        {
            char16_t* uri = u"dyauKztwsPmqouidhksvsrlstvzErnakT";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_v_SET((uint16_t)(uint16_t)12122, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)63010, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_camera_id_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)42040, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)17814, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)3672179374L, PH.base.pack) ;
        p270_framerate_SET((float)8.747865E37F, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        {
            char16_t* uri = u"klridftCmnmirniPsrqfhIhmecOdslzklgmetsusyrbazleehjyqhmsvgksbcwhntkdbkURzzvbvgicgejiheiivyctcDkfkuVnzpxklcljlnykrdzeynUqszCbnba";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_resolution_v_SET((uint16_t)(uint16_t)20457, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"uyitJK";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"zAWzlpurtxsdwrZuykxuaspeldkBmbkh";
            p299_password_SET_(password, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)2221, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)22722, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)53779, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)81, (uint8_t)172, (uint8_t)247, (uint8_t)141, (uint8_t)210, (uint8_t)33, (uint8_t)7, (uint8_t)197};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t library_version_hash[] =  {(uint8_t)33, (uint8_t)54, (uint8_t)49, (uint8_t)214, (uint8_t)9, (uint8_t)235, (uint8_t)58, (uint8_t)84};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)3222877728636035996L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)4189471863L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)35545, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        {
            char16_t* name = u"evxbfpzhnwgVkpgiRowdahcseahkjSYjSnjwsxuIiZ";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)4791133795981892936L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)728438091L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)1350655072L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)111, (uint8_t)208, (uint8_t)131, (uint8_t)76, (uint8_t)82, (uint8_t)233, (uint8_t)11, (uint8_t)206, (uint8_t)51, (uint8_t)85, (uint8_t)178, (uint8_t)220, (uint8_t)223, (uint8_t)244, (uint8_t)109, (uint8_t)192};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"m";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t)9502, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)2049, PH.base.pack) ;
        {
            char16_t* param_id = u"vviouowba";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"oetgpejNpaShahipugCstagsrm";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)12335, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_id = u"wi";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"rkrzbgmbmhXyofawawDfQqrhLrdShzvSdrarkdztxvydxldzhxxuydwDbzxugvgHgaFqyhfVudkStjmgihoosvpknbljdkoJbBmmReioiahtfzjzxqqIxgor";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        {
            char16_t* param_id = u"fsjfpXQwmOrdk";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"jUhiktttkcxClmaspcesgnkszsdoclKehtoqvolOdouentskIdtnhqpsazsmzNsazbzttmiqqTxrxWuQztxadxmqdp";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)23851, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)1629726689801356760L, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)61745, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)55123, (uint16_t)44277, (uint16_t)211, (uint16_t)48585, (uint16_t)7237, (uint16_t)36613, (uint16_t)64924, (uint16_t)2496, (uint16_t)32268, (uint16_t)26335, (uint16_t)53036, (uint16_t)31792, (uint16_t)17002, (uint16_t)12014, (uint16_t)1647, (uint16_t)47043, (uint16_t)22510, (uint16_t)32039, (uint16_t)57609, (uint16_t)36266, (uint16_t)41034, (uint16_t)41796, (uint16_t)27013, (uint16_t)40828, (uint16_t)14809, (uint16_t)31541, (uint16_t)26461, (uint16_t)14540, (uint16_t)38560, (uint16_t)11479, (uint16_t)43362, (uint16_t)31478, (uint16_t)56895, (uint16_t)45366, (uint16_t)47978, (uint16_t)20850, (uint16_t)13086, (uint16_t)33587, (uint16_t)59578, (uint16_t)27927, (uint16_t)6812, (uint16_t)38871, (uint16_t)15458, (uint16_t)55970, (uint16_t)57929, (uint16_t)40098, (uint16_t)17076, (uint16_t)63355, (uint16_t)59032, (uint16_t)58918, (uint16_t)60275, (uint16_t)9323, (uint16_t)5992, (uint16_t)8322, (uint16_t)57663, (uint16_t)39373, (uint16_t)36332, (uint16_t)24966, (uint16_t)55321, (uint16_t)22690, (uint16_t)6052, (uint16_t)24363, (uint16_t)34215, (uint16_t)50788, (uint16_t)28650, (uint16_t)47473, (uint16_t)23643, (uint16_t)52530, (uint16_t)60806, (uint16_t)57343, (uint16_t)9284, (uint16_t)59276};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


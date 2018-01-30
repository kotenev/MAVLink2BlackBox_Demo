
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
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p0_custom_mode_GET(pack) == (uint32_t)1876038293L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_FLAPPING_WING);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)56015);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)92);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)39883);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)22620);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)62304);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)16047);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)5479);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)54362);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)64799);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)867);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1586137288L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)976968066136330067L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p3_afz_GET(pack) == (float)2.2049222E38F);
    assert(p3_yaw_GET(pack) == (float) -7.9829896E37F);
    assert(p3_x_GET(pack) == (float)2.1405365E38F);
    assert(p3_y_GET(pack) == (float)3.0587547E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -1.746817E38F);
    assert(p3_z_GET(pack) == (float) -2.0583182E38F);
    assert(p3_vz_GET(pack) == (float) -2.5394152E37F);
    assert(p3_vx_GET(pack) == (float)3.2553499E38F);
    assert(p3_vy_GET(pack) == (float)1.5999024E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2275752076L);
    assert(p3_afx_GET(pack) == (float) -2.3047115E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)29061);
    assert(p3_afy_GET(pack) == (float) -3.195795E38F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)1777777790L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p4_time_usec_GET(pack) == (uint64_t)1847951500076775548L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p5_passkey_LEN(ph) == 3);
    {
        char16_t * exemplary = u"flo";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 8);
    {
        char16_t * exemplary = u"miuhxnoI";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1264363521L);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -12649);
    assert(p20_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"fgqxhpTTu";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)51308);
    assert(p22_param_value_GET(pack) == (float) -2.523951E38F);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"s";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)58799);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"vYjyclr";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float) -7.567178E37F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)100);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)56582);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)15312);
    assert(p24_lon_GET(pack) == (int32_t)1552045530);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1699658146L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1600972762L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1582898667L);
    assert(p24_alt_GET(pack) == (int32_t) -1480835848);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)970900958);
    assert(p24_lat_GET(pack) == (int32_t)388476210);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)41897);
    assert(p24_time_usec_GET(pack) == (uint64_t)2317819909003995518L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1559443660L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)4300);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)35, (uint8_t)38, (uint8_t)200, (uint8_t)82, (uint8_t)5, (uint8_t)62, (uint8_t)192, (uint8_t)32, (uint8_t)105, (uint8_t)142, (uint8_t)150, (uint8_t)189, (uint8_t)139, (uint8_t)254, (uint8_t)83, (uint8_t)29, (uint8_t)137, (uint8_t)122, (uint8_t)190, (uint8_t)208} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)94, (uint8_t)172, (uint8_t)151, (uint8_t)50, (uint8_t)231, (uint8_t)120, (uint8_t)225, (uint8_t)76, (uint8_t)31, (uint8_t)228, (uint8_t)211, (uint8_t)1, (uint8_t)102, (uint8_t)211, (uint8_t)172, (uint8_t)32, (uint8_t)47, (uint8_t)118, (uint8_t)162, (uint8_t)96} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)139);
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)192, (uint8_t)196, (uint8_t)70, (uint8_t)195, (uint8_t)148, (uint8_t)191, (uint8_t)99, (uint8_t)79, (uint8_t)94, (uint8_t)211, (uint8_t)78, (uint8_t)136, (uint8_t)134, (uint8_t)189, (uint8_t)132, (uint8_t)207, (uint8_t)145, (uint8_t)250, (uint8_t)247} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)148, (uint8_t)118, (uint8_t)253, (uint8_t)86, (uint8_t)39, (uint8_t)18, (uint8_t)82, (uint8_t)116, (uint8_t)42, (uint8_t)232, (uint8_t)162, (uint8_t)103, (uint8_t)113, (uint8_t)8, (uint8_t)76, (uint8_t)68, (uint8_t)130, (uint8_t)90, (uint8_t)112, (uint8_t)254} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)185, (uint8_t)101, (uint8_t)145, (uint8_t)248, (uint8_t)151, (uint8_t)56, (uint8_t)45, (uint8_t)163, (uint8_t)192, (uint8_t)44, (uint8_t)30, (uint8_t)130, (uint8_t)115, (uint8_t)168, (uint8_t)78, (uint8_t)84, (uint8_t)81, (uint8_t)108, (uint8_t)136, (uint8_t)124} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2834321512L);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -12261);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)10853);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -31212);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)8185);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)11127);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)29201);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)2831);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)5606);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)27674);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -31507);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)21658);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -10835);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -29468);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -30269);
    assert(p27_time_usec_GET(pack) == (uint64_t)9008196673823760817L);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)5778);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)9344);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -6584);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -18728);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -846);
    assert(p28_time_usec_GET(pack) == (uint64_t)1393218421041089717L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)23818);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)19291);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)20610);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1728010497L);
    assert(p29_press_diff_GET(pack) == (float)1.2925525E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)32222);
    assert(p29_press_abs_GET(pack) == (float) -3.250265E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)1430481363L);
    assert(p30_roll_GET(pack) == (float)3.5725677E36F);
    assert(p30_yaw_GET(pack) == (float) -1.2797062E38F);
    assert(p30_pitchspeed_GET(pack) == (float)2.548005E38F);
    assert(p30_yawspeed_GET(pack) == (float) -2.8723407E38F);
    assert(p30_rollspeed_GET(pack) == (float)3.076291E38F);
    assert(p30_pitch_GET(pack) == (float) -1.9590714E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_pitchspeed_GET(pack) == (float) -2.445654E37F);
    assert(p31_q4_GET(pack) == (float) -1.8475425E38F);
    assert(p31_q2_GET(pack) == (float) -1.9525097E38F);
    assert(p31_q1_GET(pack) == (float)1.6750358E38F);
    assert(p31_rollspeed_GET(pack) == (float) -1.5441449E38F);
    assert(p31_q3_GET(pack) == (float) -3.6730554E37F);
    assert(p31_yawspeed_GET(pack) == (float) -2.0061258E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3394159494L);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vx_GET(pack) == (float) -2.7508205E38F);
    assert(p32_y_GET(pack) == (float)2.5387182E38F);
    assert(p32_vz_GET(pack) == (float) -1.5798528E38F);
    assert(p32_z_GET(pack) == (float) -1.3026281E38F);
    assert(p32_vy_GET(pack) == (float)3.274229E38F);
    assert(p32_x_GET(pack) == (float) -2.9666519E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)4267658155L);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_alt_GET(pack) == (int32_t) -228417238);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)10630);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)5944);
    assert(p33_lat_GET(pack) == (int32_t)1230220408);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -17582);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)25232);
    assert(p33_relative_alt_GET(pack) == (int32_t)771197254);
    assert(p33_lon_GET(pack) == (int32_t)1659738536);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)114445972L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -28463);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)4812);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2135895971L);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)26719);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -6270);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)3081);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -12804);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)19674);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)20735);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)64990);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)6705);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)59769);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)30510);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)14576);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3803522764L);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)25779);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)32572);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)9950);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)35480);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)27839);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)22750);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)57803);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)30862);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)6389);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)22337);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)48428);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)2671);
    assert(p36_time_usec_GET(pack) == (uint32_t)432260142L);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)43080);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)39318);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)20662);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)41558);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)4056);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)33800);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)59422);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)1312);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -2058);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)7114);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)8727);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE);
    assert(p39_param4_GET(pack) == (float)1.3071147E38F);
    assert(p39_x_GET(pack) == (float)1.1464012E37F);
    assert(p39_param2_GET(pack) == (float) -1.8457725E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)19356);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p39_z_GET(pack) == (float) -1.2036875E38F);
    assert(p39_y_GET(pack) == (float) -2.6960008E38F);
    assert(p39_param3_GET(pack) == (float) -2.3451873E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p39_param1_GET(pack) == (float) -2.0027119E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)42081);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)77);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)51695);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)6395);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)28404);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)195);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)26);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)35467);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_time_usec_TRY(ph) == (uint64_t)1134207937723503564L);
    assert(p48_longitude_GET(pack) == (int32_t) -819759111);
    assert(p48_altitude_GET(pack) == (int32_t) -556956958);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p48_latitude_GET(pack) == (int32_t) -1341515638);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)8787628916244956455L);
    assert(p49_altitude_GET(pack) == (int32_t) -1432225484);
    assert(p49_latitude_GET(pack) == (int32_t) -1155171582);
    assert(p49_longitude_GET(pack) == (int32_t) -653830860);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p50_param_value_min_GET(pack) == (float)1.198202E38F);
    assert(p50_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"gnezfhqq";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float) -2.8927333E37F);
    assert(p50_param_value0_GET(pack) == (float)1.7040735E38F);
    assert(p50_scale_GET(pack) == (float)1.6090551E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)30088);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)244);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)30676);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p54_p2z_GET(pack) == (float) -2.3951765E38F);
    assert(p54_p1x_GET(pack) == (float) -3.116855E37F);
    assert(p54_p1z_GET(pack) == (float) -3.6892042E37F);
    assert(p54_p2y_GET(pack) == (float) -1.1139006E38F);
    assert(p54_p1y_GET(pack) == (float)1.455382E38F);
    assert(p54_p2x_GET(pack) == (float)2.8880052E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p55_p2z_GET(pack) == (float)3.39955E37F);
    assert(p55_p1y_GET(pack) == (float)1.2029999E38F);
    assert(p55_p2y_GET(pack) == (float)2.9326048E38F);
    assert(p55_p2x_GET(pack) == (float)4.1098756E37F);
    assert(p55_p1z_GET(pack) == (float) -1.1926291E38F);
    assert(p55_p1x_GET(pack) == (float) -1.7310753E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_pitchspeed_GET(pack) == (float)1.6708638E38F);
    assert(p61_rollspeed_GET(pack) == (float)1.0128617E38F);
    {
        float exemplary[] =  {3.2396778E38F, -1.4478285E38F, 1.9762573E38F, 1.332308E38F, 2.2706338E38F, -1.8749875E38F, 7.3721995E37F, 1.1377008E38F, 6.9072045E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)7562865263529368383L);
    {
        float exemplary[] =  {3.2074793E38F, -2.3531925E38F, 2.5557422E38F, -2.8241629E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float)3.2784479E38F);
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_alt_error_GET(pack) == (float) -2.7895337E38F);
    assert(p62_nav_pitch_GET(pack) == (float)9.251857E37F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)29911);
    assert(p62_nav_roll_GET(pack) == (float) -2.1038132E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)22329);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)15804);
    assert(p62_aspd_error_GET(pack) == (float) -2.9195517E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -2.702578E36F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_time_usec_GET(pack) == (uint64_t)2558070013190918344L);
    assert(p63_relative_alt_GET(pack) == (int32_t)700664959);
    assert(p63_vy_GET(pack) == (float) -4.1794473E37F);
    assert(p63_lat_GET(pack) == (int32_t)584554462);
    assert(p63_vz_GET(pack) == (float)2.912355E38F);
    assert(p63_vx_GET(pack) == (float) -1.852078E38F);
    assert(p63_alt_GET(pack) == (int32_t) -748837603);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
    {
        float exemplary[] =  {-1.9469913E38F, 2.852813E38F, -1.3888434E38F, 2.0217123E38F, 2.9468333E38F, -5.7301037E37F, 2.0488558E38F, -3.8996517E37F, -1.3574205E38F, -2.468015E38F, -1.3809641E38F, -1.3139917E37F, -8.342899E37F, -3.2070122E38F, 2.124228E38F, -3.717357E37F, 2.85092E38F, -1.3201705E38F, -3.3298573E38F, 1.6268772E38F, -2.7246809E38F, 1.2375039E38F, -1.5727344E38F, 1.7154592E38F, 5.472361E37F, -5.8748466E37F, -2.0540556E38F, -9.159018E37F, -2.913951E38F, 2.8964798E38F, 8.3233477E37F, 2.5323623E38F, 1.7475367E38F, -6.916938E37F, -4.1466844E37F, -2.6789307E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_lon_GET(pack) == (int32_t)1181305154);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p64_y_GET(pack) == (float) -2.2289188E38F);
    assert(p64_ay_GET(pack) == (float)2.2931887E38F);
    assert(p64_vz_GET(pack) == (float) -2.9254437E37F);
    assert(p64_ax_GET(pack) == (float) -2.182815E38F);
    assert(p64_x_GET(pack) == (float) -9.096956E37F);
    assert(p64_vy_GET(pack) == (float) -1.707929E38F);
    assert(p64_vx_GET(pack) == (float) -2.8438881E38F);
    {
        float exemplary[] =  {-3.0773723E38F, -1.2690684E38F, 2.8567833E38F, -1.4099822E38F, -3.1868746E38F, -1.1810983E38F, 1.9182286E38F, -1.4963854E38F, -7.3238235E37F, -1.1640761E38F, -1.2470471E38F, 9.537331E37F, 1.6706842E38F, -1.7827688E38F, -1.4841388E38F, -3.3668453E38F, -2.2775927E38F, -2.1734598E38F, 2.2240587E38F, 3.3952303E38F, 2.2996848E37F, 1.489214E38F, -6.709353E37F, 2.7005937E38F, -2.4244392E38F, 1.0871049E38F, -2.0634496E38F, 2.3333677E38F, -1.7428245E38F, -3.1336134E38F, -1.4657602E38F, -2.6647018E38F, 1.8258312E38F, 3.318637E37F, 8.045426E37F, -2.4497208E38F, -1.451819E37F, -2.2961827E38F, 1.3258447E38F, -2.793597E38F, 2.493665E38F, -2.9957677E38F, -2.5904426E38F, -3.2676816E38F, -2.6064728E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_time_usec_GET(pack) == (uint64_t)2054404724743728818L);
    assert(p64_az_GET(pack) == (float)7.9310626E37F);
    assert(p64_z_GET(pack) == (float) -1.5939361E37F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)20893);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)52202);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)58142);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)30178);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)61397);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)15836);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)13284);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)6863);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)6917);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2701548636L);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)35625);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)18533);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)14375);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)32350);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)10374);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)37871);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)22962);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)63286);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)62251);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)24002);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)50432);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)234);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -31745);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)13710);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)24430);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -4026);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)9138);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)8646);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)20268);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)55389);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)56246);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)29838);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)37805);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)55365);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)41652);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)27);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param2_GET(pack) == (float) -2.5003702E38F);
    assert(p73_x_GET(pack) == (int32_t) -2050573992);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)28310);
    assert(p73_y_GET(pack) == (int32_t)338822673);
    assert(p73_z_GET(pack) == (float)3.3361651E38F);
    assert(p73_param1_GET(pack) == (float)2.7546287E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_param3_GET(pack) == (float)1.0163971E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p73_param4_GET(pack) == (float)2.3332196E38F);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_climb_GET(pack) == (float) -2.750339E38F);
    assert(p74_groundspeed_GET(pack) == (float) -8.864647E37F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)31153);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -21823);
    assert(p74_alt_GET(pack) == (float) -9.744469E37F);
    assert(p74_airspeed_GET(pack) == (float)7.584404E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param2_GET(pack) == (float)1.5594974E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p75_x_GET(pack) == (int32_t) -101414060);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p75_param4_GET(pack) == (float)1.884784E38F);
    assert(p75_z_GET(pack) == (float) -1.2674495E38F);
    assert(p75_y_GET(pack) == (int32_t)1709683284);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p75_param1_GET(pack) == (float)1.0255435E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION);
    assert(p75_param3_GET(pack) == (float)2.1283676E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float) -2.1955414E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p76_param7_GET(pack) == (float) -2.0090325E38F);
    assert(p76_param3_GET(pack) == (float)7.563703E37F);
    assert(p76_param1_GET(pack) == (float)2.8360615E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p76_param6_GET(pack) == (float) -3.4279714E37F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p76_param4_GET(pack) == (float)1.2003641E37F);
    assert(p76_param5_GET(pack) == (float) -2.2489923E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t)1766749511);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)190);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)254);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)98);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)636563537L);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p81_thrust_GET(pack) == (float)1.7286485E38F);
    assert(p81_pitch_GET(pack) == (float) -9.399901E37F);
    assert(p81_yaw_GET(pack) == (float)3.3326314E37F);
    assert(p81_roll_GET(pack) == (float)2.207206E38F);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_thrust_GET(pack) == (float) -2.8250301E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p82_body_roll_rate_GET(pack) == (float)2.4491525E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float)3.2856846E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p82_body_pitch_rate_GET(pack) == (float)1.0139677E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)231);
    {
        float exemplary[] =  {-2.888454E36F, 1.4167809E38F, 2.905799E38F, 3.079011E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1661009619L);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_yaw_rate_GET(pack) == (float) -2.9802344E37F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)2440290737L);
    assert(p83_thrust_GET(pack) == (float)2.0431921E38F);
    assert(p83_body_pitch_rate_GET(pack) == (float)1.4800156E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)2.1207034E38F);
    {
        float exemplary[] =  {-2.351521E38F, 2.9463044E38F, -4.2016266E37F, 8.2916387E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_rate_GET(pack) == (float)2.4671134E38F);
    assert(p84_afx_GET(pack) == (float)2.031474E38F);
    assert(p84_z_GET(pack) == (float)2.0757416E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p84_yaw_GET(pack) == (float) -3.2909244E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p84_y_GET(pack) == (float)1.1593487E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p84_afy_GET(pack) == (float) -2.2688325E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)509944675L);
    assert(p84_vx_GET(pack) == (float) -2.0643802E38F);
    assert(p84_afz_GET(pack) == (float)1.8637409E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)7);
    assert(p84_x_GET(pack) == (float)2.7720282E38F);
    assert(p84_vz_GET(pack) == (float)2.679293E38F);
    assert(p84_vy_GET(pack) == (float) -2.5018452E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)6030);
    assert(p86_vx_GET(pack) == (float) -6.8669683E37F);
    assert(p86_afx_GET(pack) == (float) -2.5817278E38F);
    assert(p86_yaw_rate_GET(pack) == (float)2.5726648E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)253135359L);
    assert(p86_vz_GET(pack) == (float)3.0932861E38F);
    assert(p86_lat_int_GET(pack) == (int32_t) -1449078658);
    assert(p86_afy_GET(pack) == (float) -3.0040129E38F);
    assert(p86_yaw_GET(pack) == (float) -6.092749E37F);
    assert(p86_alt_GET(pack) == (float) -2.6298702E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p86_afz_GET(pack) == (float) -2.205653E38F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1109651881);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p86_vy_GET(pack) == (float) -7.7085123E37F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)224);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_vx_GET(pack) == (float) -2.683171E38F);
    assert(p87_afy_GET(pack) == (float) -4.269809E37F);
    assert(p87_yaw_rate_GET(pack) == (float)2.712872E38F);
    assert(p87_vz_GET(pack) == (float)1.7011022E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)64669);
    assert(p87_vy_GET(pack) == (float)3.6093996E36F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)1276858005L);
    assert(p87_yaw_GET(pack) == (float) -2.5778697E38F);
    assert(p87_afz_GET(pack) == (float) -1.9693668E38F);
    assert(p87_alt_GET(pack) == (float) -1.3466457E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -673726200);
    assert(p87_afx_GET(pack) == (float) -3.1168528E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p87_lat_int_GET(pack) == (int32_t) -496316436);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_pitch_GET(pack) == (float) -7.250178E36F);
    assert(p89_x_GET(pack) == (float) -2.8398743E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)166670646L);
    assert(p89_roll_GET(pack) == (float) -1.6072379E38F);
    assert(p89_z_GET(pack) == (float) -1.7228075E38F);
    assert(p89_y_GET(pack) == (float)2.1710969E38F);
    assert(p89_yaw_GET(pack) == (float) -1.5443548E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)17407);
    assert(p90_pitchspeed_GET(pack) == (float) -2.7343378E37F);
    assert(p90_roll_GET(pack) == (float) -6.1198327E37F);
    assert(p90_time_usec_GET(pack) == (uint64_t)6742601611161197277L);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -12467);
    assert(p90_lat_GET(pack) == (int32_t)386982471);
    assert(p90_yawspeed_GET(pack) == (float) -2.6965403E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -26121);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)29112);
    assert(p90_lon_GET(pack) == (int32_t)2087364998);
    assert(p90_rollspeed_GET(pack) == (float)1.4706485E38F);
    assert(p90_yaw_GET(pack) == (float) -1.8274938E38F);
    assert(p90_alt_GET(pack) == (int32_t)848984422);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)20289);
    assert(p90_pitch_GET(pack) == (float) -2.5410963E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)13864);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux3_GET(pack) == (float)2.6362042E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)6152716310341722422L);
    assert(p91_pitch_elevator_GET(pack) == (float)3.1876516E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    assert(p91_yaw_rudder_GET(pack) == (float) -4.1075994E37F);
    assert(p91_aux4_GET(pack) == (float) -2.006213E37F);
    assert(p91_roll_ailerons_GET(pack) == (float)3.222416E38F);
    assert(p91_aux1_GET(pack) == (float)5.1834127E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p91_aux2_GET(pack) == (float)2.5425743E38F);
    assert(p91_throttle_GET(pack) == (float) -1.1705424E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)60982);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)44031);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)14402);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)16106);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)7295);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)48996);
    assert(p92_time_usec_GET(pack) == (uint64_t)7887155782041998968L);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)13972);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)51203);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)33480);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)63278);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)59272);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)55596);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_PREFLIGHT);
    assert(p93_time_usec_GET(pack) == (uint64_t)4013659342510638650L);
    {
        float exemplary[] =  {1.7202911E38F, -6.4353773E37F, 2.2451033E38F, 2.8113543E38F, 1.5791976E38F, -1.7981472E38F, 3.1023846E38F, -8.6496256E36F, -4.5189645E37F, 2.9182256E38F, -2.1115901E38F, -2.6974005E37F, 9.961425E37F, -1.8361274E38F, 3.0944495E37F, 2.1731582E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)4272882714694281413L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_ground_distance_GET(pack) == (float)8.523298E36F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)5262);
    assert(p100_flow_rate_y_TRY(ph) == (float)4.2548414E36F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)3.2144477E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)28743);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p100_time_usec_GET(pack) == (uint64_t)4519168474515564870L);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.4418709E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -1.3381496E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_roll_GET(pack) == (float) -2.9597111E37F);
    assert(p101_z_GET(pack) == (float)3.1320162E38F);
    assert(p101_x_GET(pack) == (float)2.0241869E37F);
    assert(p101_y_GET(pack) == (float)7.129221E37F);
    assert(p101_usec_GET(pack) == (uint64_t)5639921305148834868L);
    assert(p101_yaw_GET(pack) == (float) -2.1095026E37F);
    assert(p101_pitch_GET(pack) == (float) -3.2584918E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float)1.3866295E38F);
    assert(p102_roll_GET(pack) == (float) -2.2368015E38F);
    assert(p102_pitch_GET(pack) == (float)1.1617526E38F);
    assert(p102_usec_GET(pack) == (uint64_t)5791775399724101132L);
    assert(p102_z_GET(pack) == (float) -2.506053E38F);
    assert(p102_x_GET(pack) == (float)7.719659E37F);
    assert(p102_yaw_GET(pack) == (float) -2.8435113E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)7.627558E37F);
    assert(p103_usec_GET(pack) == (uint64_t)7311028900765663440L);
    assert(p103_x_GET(pack) == (float)1.763141E38F);
    assert(p103_z_GET(pack) == (float) -9.514893E37F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float) -2.235618E38F);
    assert(p104_y_GET(pack) == (float) -2.093941E38F);
    assert(p104_roll_GET(pack) == (float) -2.316775E38F);
    assert(p104_x_GET(pack) == (float) -2.5803016E38F);
    assert(p104_z_GET(pack) == (float)2.8741681E38F);
    assert(p104_pitch_GET(pack) == (float)1.5519846E38F);
    assert(p104_usec_GET(pack) == (uint64_t)2583946965858146363L);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_ymag_GET(pack) == (float) -8.2054597E37F);
    assert(p105_zmag_GET(pack) == (float)7.4772695E37F);
    assert(p105_pressure_alt_GET(pack) == (float)3.002738E38F);
    assert(p105_xmag_GET(pack) == (float) -2.8901403E38F);
    assert(p105_yacc_GET(pack) == (float) -2.836348E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)5849786615138987329L);
    assert(p105_diff_pressure_GET(pack) == (float) -1.749136E38F);
    assert(p105_abs_pressure_GET(pack) == (float)3.104046E38F);
    assert(p105_temperature_GET(pack) == (float) -2.0755592E38F);
    assert(p105_ygyro_GET(pack) == (float)7.087489E37F);
    assert(p105_zgyro_GET(pack) == (float)2.7720971E38F);
    assert(p105_zacc_GET(pack) == (float) -3.0432918E38F);
    assert(p105_xacc_GET(pack) == (float) -9.727169E36F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)49272);
    assert(p105_xgyro_GET(pack) == (float) -2.0656162E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_xgyro_GET(pack) == (float)2.2298501E37F);
    assert(p106_time_usec_GET(pack) == (uint64_t)6125659941458459298L);
    assert(p106_integrated_x_GET(pack) == (float)2.0730649E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)23140);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2337196831L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2352695594L);
    assert(p106_distance_GET(pack) == (float)3.102812E38F);
    assert(p106_integrated_y_GET(pack) == (float) -7.5729563E37F);
    assert(p106_integrated_zgyro_GET(pack) == (float)2.1277254E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p106_integrated_ygyro_GET(pack) == (float)7.159597E37F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xacc_GET(pack) == (float)2.4295E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)3401699506L);
    assert(p107_xmag_GET(pack) == (float) -3.180734E37F);
    assert(p107_xgyro_GET(pack) == (float) -3.405609E37F);
    assert(p107_pressure_alt_GET(pack) == (float)2.1797562E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)1704027065617135820L);
    assert(p107_temperature_GET(pack) == (float) -1.096365E38F);
    assert(p107_zmag_GET(pack) == (float)3.1894165E37F);
    assert(p107_ymag_GET(pack) == (float) -2.8477276E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -2.6642144E38F);
    assert(p107_abs_pressure_GET(pack) == (float)3.1240728E38F);
    assert(p107_ygyro_GET(pack) == (float)1.8301944E38F);
    assert(p107_yacc_GET(pack) == (float) -1.1197476E38F);
    assert(p107_zgyro_GET(pack) == (float)9.687189E37F);
    assert(p107_zacc_GET(pack) == (float) -2.5276712E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_yaw_GET(pack) == (float)1.8154183E37F);
    assert(p108_q1_GET(pack) == (float)2.228905E37F);
    assert(p108_yacc_GET(pack) == (float) -1.0868063E38F);
    assert(p108_ygyro_GET(pack) == (float) -3.1466599E38F);
    assert(p108_roll_GET(pack) == (float)9.768196E37F);
    assert(p108_ve_GET(pack) == (float)2.9679954E38F);
    assert(p108_zacc_GET(pack) == (float) -5.062194E37F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.1940415E38F);
    assert(p108_q2_GET(pack) == (float)2.4481154E38F);
    assert(p108_q4_GET(pack) == (float) -1.0878806E38F);
    assert(p108_pitch_GET(pack) == (float)3.1193642E38F);
    assert(p108_lat_GET(pack) == (float) -3.1256866E38F);
    assert(p108_q3_GET(pack) == (float) -1.5919881E38F);
    assert(p108_zgyro_GET(pack) == (float) -7.199785E37F);
    assert(p108_alt_GET(pack) == (float) -1.6319332E38F);
    assert(p108_xgyro_GET(pack) == (float) -1.6763646E38F);
    assert(p108_lon_GET(pack) == (float)1.3189123E38F);
    assert(p108_vn_GET(pack) == (float) -2.404912E37F);
    assert(p108_xacc_GET(pack) == (float) -8.771818E37F);
    assert(p108_std_dev_horz_GET(pack) == (float)6.3173074E37F);
    assert(p108_vd_GET(pack) == (float) -5.402685E37F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)8917);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)24652);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)112);
    {
        uint8_t exemplary[] =  {(uint8_t)239, (uint8_t)230, (uint8_t)49, (uint8_t)145, (uint8_t)7, (uint8_t)164, (uint8_t)242, (uint8_t)229, (uint8_t)210, (uint8_t)42, (uint8_t)137, (uint8_t)66, (uint8_t)254, (uint8_t)245, (uint8_t)173, (uint8_t)175, (uint8_t)137, (uint8_t)217, (uint8_t)54, (uint8_t)37, (uint8_t)255, (uint8_t)172, (uint8_t)92, (uint8_t)196, (uint8_t)94, (uint8_t)155, (uint8_t)11, (uint8_t)124, (uint8_t)192, (uint8_t)32, (uint8_t)4, (uint8_t)82, (uint8_t)113, (uint8_t)10, (uint8_t)10, (uint8_t)168, (uint8_t)186, (uint8_t)10, (uint8_t)249, (uint8_t)103, (uint8_t)190, (uint8_t)37, (uint8_t)87, (uint8_t)255, (uint8_t)221, (uint8_t)160, (uint8_t)240, (uint8_t)121, (uint8_t)161, (uint8_t)204, (uint8_t)137, (uint8_t)53, (uint8_t)208, (uint8_t)188, (uint8_t)13, (uint8_t)45, (uint8_t)18, (uint8_t)96, (uint8_t)240, (uint8_t)44, (uint8_t)67, (uint8_t)46, (uint8_t)20, (uint8_t)94, (uint8_t)103, (uint8_t)204, (uint8_t)94, (uint8_t)113, (uint8_t)158, (uint8_t)90, (uint8_t)255, (uint8_t)8, (uint8_t)31, (uint8_t)124, (uint8_t)53, (uint8_t)161, (uint8_t)63, (uint8_t)42, (uint8_t)16, (uint8_t)248, (uint8_t)90, (uint8_t)214, (uint8_t)64, (uint8_t)0, (uint8_t)174, (uint8_t)42, (uint8_t)149, (uint8_t)217, (uint8_t)242, (uint8_t)12, (uint8_t)247, (uint8_t)61, (uint8_t)250, (uint8_t)185, (uint8_t)60, (uint8_t)2, (uint8_t)182, (uint8_t)101, (uint8_t)193, (uint8_t)10, (uint8_t)177, (uint8_t)120, (uint8_t)138, (uint8_t)129, (uint8_t)92, (uint8_t)30, (uint8_t)244, (uint8_t)166, (uint8_t)16, (uint8_t)151, (uint8_t)162, (uint8_t)30, (uint8_t)61, (uint8_t)103, (uint8_t)130, (uint8_t)92, (uint8_t)83, (uint8_t)165, (uint8_t)102, (uint8_t)141, (uint8_t)248, (uint8_t)63, (uint8_t)153, (uint8_t)170, (uint8_t)74, (uint8_t)134, (uint8_t)134, (uint8_t)62, (uint8_t)197, (uint8_t)118, (uint8_t)154, (uint8_t)254, (uint8_t)170, (uint8_t)231, (uint8_t)219, (uint8_t)77, (uint8_t)194, (uint8_t)202, (uint8_t)22, (uint8_t)200, (uint8_t)220, (uint8_t)169, (uint8_t)150, (uint8_t)221, (uint8_t)251, (uint8_t)159, (uint8_t)151, (uint8_t)23, (uint8_t)220, (uint8_t)174, (uint8_t)0, (uint8_t)37, (uint8_t)186, (uint8_t)164, (uint8_t)108, (uint8_t)132, (uint8_t)18, (uint8_t)216, (uint8_t)163, (uint8_t)26, (uint8_t)93, (uint8_t)72, (uint8_t)19, (uint8_t)227, (uint8_t)133, (uint8_t)215, (uint8_t)186, (uint8_t)105, (uint8_t)198, (uint8_t)20, (uint8_t)204, (uint8_t)49, (uint8_t)129, (uint8_t)156, (uint8_t)149, (uint8_t)181, (uint8_t)9, (uint8_t)97, (uint8_t)191, (uint8_t)17, (uint8_t)56, (uint8_t)101, (uint8_t)71, (uint8_t)2, (uint8_t)163, (uint8_t)152, (uint8_t)118, (uint8_t)142, (uint8_t)139, (uint8_t)145, (uint8_t)131, (uint8_t)61, (uint8_t)152, (uint8_t)112, (uint8_t)214, (uint8_t)138, (uint8_t)238, (uint8_t)244, (uint8_t)211, (uint8_t)39, (uint8_t)101, (uint8_t)206, (uint8_t)2, (uint8_t)177, (uint8_t)203, (uint8_t)148, (uint8_t)216, (uint8_t)115, (uint8_t)156, (uint8_t)171, (uint8_t)185, (uint8_t)153, (uint8_t)195, (uint8_t)37, (uint8_t)122, (uint8_t)108, (uint8_t)38, (uint8_t)30, (uint8_t)36, (uint8_t)64, (uint8_t)71, (uint8_t)57, (uint8_t)197, (uint8_t)73, (uint8_t)92, (uint8_t)252, (uint8_t)20, (uint8_t)55, (uint8_t)86, (uint8_t)122, (uint8_t)222, (uint8_t)95, (uint8_t)17, (uint8_t)203, (uint8_t)148, (uint8_t)172, (uint8_t)105, (uint8_t)68, (uint8_t)47, (uint8_t)168, (uint8_t)23, (uint8_t)198, (uint8_t)193, (uint8_t)157, (uint8_t)86, (uint8_t)247, (uint8_t)35, (uint8_t)189, (uint8_t)196, (uint8_t)51, (uint8_t)100} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)79);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)6954607405756756048L);
    assert(p111_ts1_GET(pack) == (int64_t) -4271923815245869356L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)267142085484793790L);
    assert(p112_seq_GET(pack) == (uint32_t)2980162530L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_alt_GET(pack) == (int32_t) -939279853);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)4809);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)30551);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)22467);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)18281);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p113_lon_GET(pack) == (int32_t)191708389);
    assert(p113_time_usec_GET(pack) == (uint64_t)7268399503943060421L);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)7460);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)49264);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)45001);
    assert(p113_lat_GET(pack) == (int32_t)1307144115);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_xgyro_GET(pack) == (float)2.2735658E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)4325286303404438081L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p114_integrated_x_GET(pack) == (float)8.3874873E37F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -18973);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p114_integrated_ygyro_GET(pack) == (float)3.3694082E38F);
    assert(p114_distance_GET(pack) == (float) -2.8587728E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -2.2611015E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)3067982059L);
    assert(p114_integrated_y_GET(pack) == (float)1.0757223E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2956013886L);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -27598);
    assert(p115_yawspeed_GET(pack) == (float) -8.204661E37F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)4892);
    assert(p115_alt_GET(pack) == (int32_t) -1052604667);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)44916);
    assert(p115_lat_GET(pack) == (int32_t)878771475);
    assert(p115_pitchspeed_GET(pack) == (float) -3.375693E38F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -17930);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)8151);
    assert(p115_lon_GET(pack) == (int32_t)1194102551);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)20306);
    assert(p115_rollspeed_GET(pack) == (float)1.049093E38F);
    {
        float exemplary[] =  {-3.0971795E38F, -1.2068994E38F, -2.6729137E38F, -1.8779921E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)45662);
    assert(p115_time_usec_GET(pack) == (uint64_t)7020354322673331638L);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)370);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)6917);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -18595);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -15366);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -2760);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)16932);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -22203);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2301176427L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)28857);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -6685);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)5418);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)466);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)31482);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)3666288349L);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)57905);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)44327);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)39380);
    assert(p118_time_utc_GET(pack) == (uint32_t)2604078058L);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p119_ofs_GET(pack) == (uint32_t)487743224L);
    assert(p119_count_GET(pack) == (uint32_t)2797930403L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)39159);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)123);
    {
        uint8_t exemplary[] =  {(uint8_t)32, (uint8_t)51, (uint8_t)28, (uint8_t)73, (uint8_t)245, (uint8_t)176, (uint8_t)98, (uint8_t)233, (uint8_t)137, (uint8_t)113, (uint8_t)157, (uint8_t)88, (uint8_t)42, (uint8_t)105, (uint8_t)59, (uint8_t)49, (uint8_t)238, (uint8_t)91, (uint8_t)160, (uint8_t)202, (uint8_t)72, (uint8_t)218, (uint8_t)136, (uint8_t)237, (uint8_t)29, (uint8_t)248, (uint8_t)91, (uint8_t)133, (uint8_t)116, (uint8_t)39, (uint8_t)71, (uint8_t)102, (uint8_t)76, (uint8_t)143, (uint8_t)245, (uint8_t)136, (uint8_t)153, (uint8_t)255, (uint8_t)8, (uint8_t)246, (uint8_t)8, (uint8_t)120, (uint8_t)123, (uint8_t)72, (uint8_t)179, (uint8_t)26, (uint8_t)1, (uint8_t)97, (uint8_t)98, (uint8_t)124, (uint8_t)234, (uint8_t)160, (uint8_t)156, (uint8_t)204, (uint8_t)254, (uint8_t)3, (uint8_t)238, (uint8_t)17, (uint8_t)15, (uint8_t)194, (uint8_t)193, (uint8_t)249, (uint8_t)211, (uint8_t)41, (uint8_t)60, (uint8_t)85, (uint8_t)133, (uint8_t)69, (uint8_t)123, (uint8_t)138, (uint8_t)173, (uint8_t)96, (uint8_t)173, (uint8_t)63, (uint8_t)206, (uint8_t)45, (uint8_t)128, (uint8_t)14, (uint8_t)10, (uint8_t)211, (uint8_t)182, (uint8_t)50, (uint8_t)172, (uint8_t)83, (uint8_t)176, (uint8_t)113, (uint8_t)208, (uint8_t)120, (uint8_t)83, (uint8_t)97} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)38777);
    assert(p120_ofs_GET(pack) == (uint32_t)3333682693L);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)13);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)84);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)116, (uint8_t)13, (uint8_t)33, (uint8_t)242, (uint8_t)172, (uint8_t)177, (uint8_t)33, (uint8_t)213, (uint8_t)147, (uint8_t)2, (uint8_t)187, (uint8_t)208, (uint8_t)127, (uint8_t)251, (uint8_t)128, (uint8_t)104, (uint8_t)51, (uint8_t)164, (uint8_t)88, (uint8_t)187, (uint8_t)37, (uint8_t)23, (uint8_t)110, (uint8_t)107, (uint8_t)185, (uint8_t)141, (uint8_t)72, (uint8_t)252, (uint8_t)195, (uint8_t)1, (uint8_t)194, (uint8_t)158, (uint8_t)144, (uint8_t)123, (uint8_t)18, (uint8_t)152, (uint8_t)116, (uint8_t)134, (uint8_t)44, (uint8_t)33, (uint8_t)143, (uint8_t)89, (uint8_t)243, (uint8_t)160, (uint8_t)74, (uint8_t)197, (uint8_t)186, (uint8_t)77, (uint8_t)235, (uint8_t)183, (uint8_t)214, (uint8_t)164, (uint8_t)133, (uint8_t)115, (uint8_t)26, (uint8_t)114, (uint8_t)55, (uint8_t)231, (uint8_t)157, (uint8_t)249, (uint8_t)193, (uint8_t)113, (uint8_t)128, (uint8_t)247, (uint8_t)161, (uint8_t)48, (uint8_t)60, (uint8_t)210, (uint8_t)119, (uint8_t)146, (uint8_t)199, (uint8_t)29, (uint8_t)182, (uint8_t)122, (uint8_t)237, (uint8_t)81, (uint8_t)183, (uint8_t)248, (uint8_t)184, (uint8_t)164, (uint8_t)39, (uint8_t)62, (uint8_t)21, (uint8_t)152, (uint8_t)1, (uint8_t)245, (uint8_t)125, (uint8_t)228, (uint8_t)187, (uint8_t)144, (uint8_t)200, (uint8_t)71, (uint8_t)32, (uint8_t)149, (uint8_t)251, (uint8_t)249, (uint8_t)142, (uint8_t)213, (uint8_t)114, (uint8_t)217, (uint8_t)192, (uint8_t)161, (uint8_t)141, (uint8_t)70, (uint8_t)98, (uint8_t)106, (uint8_t)151, (uint8_t)238, (uint8_t)173, (uint8_t)190} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)232);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lon_GET(pack) == (int32_t) -1496438604);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1423325231L);
    assert(p124_lat_GET(pack) == (int32_t)2129404312);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)58966);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)9091);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)48062);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)49316);
    assert(p124_time_usec_GET(pack) == (uint64_t)4997264006353989599L);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p124_alt_GET(pack) == (int32_t)1313475703);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)24440);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)52119);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)3144);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p126_baudrate_GET(pack) == (uint32_t)1887991453L);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI);
    {
        uint8_t exemplary[] =  {(uint8_t)136, (uint8_t)5, (uint8_t)218, (uint8_t)114, (uint8_t)21, (uint8_t)94, (uint8_t)34, (uint8_t)43, (uint8_t)127, (uint8_t)166, (uint8_t)64, (uint8_t)97, (uint8_t)68, (uint8_t)62, (uint8_t)150, (uint8_t)220, (uint8_t)227, (uint8_t)93, (uint8_t)98, (uint8_t)90, (uint8_t)248, (uint8_t)158, (uint8_t)195, (uint8_t)225, (uint8_t)252, (uint8_t)204, (uint8_t)84, (uint8_t)246, (uint8_t)103, (uint8_t)225, (uint8_t)244, (uint8_t)181, (uint8_t)174, (uint8_t)138, (uint8_t)250, (uint8_t)245, (uint8_t)127, (uint8_t)53, (uint8_t)153, (uint8_t)24, (uint8_t)226, (uint8_t)130, (uint8_t)203, (uint8_t)255, (uint8_t)34, (uint8_t)38, (uint8_t)208, (uint8_t)82, (uint8_t)80, (uint8_t)180, (uint8_t)192, (uint8_t)118, (uint8_t)39, (uint8_t)218, (uint8_t)247, (uint8_t)71, (uint8_t)207, (uint8_t)86, (uint8_t)184, (uint8_t)126, (uint8_t)4, (uint8_t)13, (uint8_t)127, (uint8_t)42, (uint8_t)82, (uint8_t)156, (uint8_t)60, (uint8_t)167, (uint8_t)111, (uint8_t)92} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1782721771L);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)47196);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)2035252008);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1413042033);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)250639321);
    assert(p127_accuracy_GET(pack) == (uint32_t)470290111L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -952453813);
    assert(p127_tow_GET(pack) == (uint32_t)3994438851L);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)939483234);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1827633840);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)1661229394);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)1552286010);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p128_tow_GET(pack) == (uint32_t)2692707107L);
    assert(p128_accuracy_GET(pack) == (uint32_t)1973632655L);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)1698350476L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)10399);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -14455);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)27364);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)9467);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -4552);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)4502);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)19138);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)2505977931L);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -22916);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)29548);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -20860);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)11363);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)27529);
    assert(p130_size_GET(pack) == (uint32_t)2691718608L);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)22166);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)109, (uint8_t)181, (uint8_t)129, (uint8_t)236, (uint8_t)48, (uint8_t)108, (uint8_t)222, (uint8_t)198, (uint8_t)238, (uint8_t)71, (uint8_t)71, (uint8_t)13, (uint8_t)187, (uint8_t)165, (uint8_t)110, (uint8_t)119, (uint8_t)224, (uint8_t)187, (uint8_t)185, (uint8_t)228, (uint8_t)114, (uint8_t)128, (uint8_t)89, (uint8_t)28, (uint8_t)235, (uint8_t)202, (uint8_t)110, (uint8_t)148, (uint8_t)66, (uint8_t)81, (uint8_t)218, (uint8_t)66, (uint8_t)223, (uint8_t)175, (uint8_t)229, (uint8_t)106, (uint8_t)5, (uint8_t)96, (uint8_t)49, (uint8_t)147, (uint8_t)93, (uint8_t)162, (uint8_t)240, (uint8_t)116, (uint8_t)163, (uint8_t)72, (uint8_t)235, (uint8_t)208, (uint8_t)117, (uint8_t)193, (uint8_t)134, (uint8_t)198, (uint8_t)130, (uint8_t)255, (uint8_t)142, (uint8_t)144, (uint8_t)8, (uint8_t)172, (uint8_t)6, (uint8_t)111, (uint8_t)70, (uint8_t)49, (uint8_t)249, (uint8_t)17, (uint8_t)158, (uint8_t)134, (uint8_t)87, (uint8_t)155, (uint8_t)235, (uint8_t)184, (uint8_t)84, (uint8_t)169, (uint8_t)90, (uint8_t)209, (uint8_t)209, (uint8_t)160, (uint8_t)43, (uint8_t)177, (uint8_t)171, (uint8_t)235, (uint8_t)35, (uint8_t)88, (uint8_t)20, (uint8_t)137, (uint8_t)55, (uint8_t)208, (uint8_t)139, (uint8_t)113, (uint8_t)200, (uint8_t)189, (uint8_t)46, (uint8_t)251, (uint8_t)182, (uint8_t)82, (uint8_t)123, (uint8_t)241, (uint8_t)123, (uint8_t)255, (uint8_t)33, (uint8_t)54, (uint8_t)166, (uint8_t)89, (uint8_t)109, (uint8_t)37, (uint8_t)153, (uint8_t)50, (uint8_t)144, (uint8_t)158, (uint8_t)98, (uint8_t)180, (uint8_t)78, (uint8_t)210, (uint8_t)42, (uint8_t)221, (uint8_t)74, (uint8_t)222, (uint8_t)44, (uint8_t)92, (uint8_t)175, (uint8_t)219, (uint8_t)63, (uint8_t)242, (uint8_t)148, (uint8_t)10, (uint8_t)251, (uint8_t)14, (uint8_t)159, (uint8_t)94, (uint8_t)130, (uint8_t)186, (uint8_t)144, (uint8_t)156, (uint8_t)238, (uint8_t)173, (uint8_t)34, (uint8_t)161, (uint8_t)201, (uint8_t)36, (uint8_t)104, (uint8_t)230, (uint8_t)7, (uint8_t)101, (uint8_t)198, (uint8_t)185, (uint8_t)213, (uint8_t)242, (uint8_t)251, (uint8_t)229, (uint8_t)234, (uint8_t)69, (uint8_t)62, (uint8_t)240, (uint8_t)34, (uint8_t)22, (uint8_t)48, (uint8_t)145, (uint8_t)145, (uint8_t)174, (uint8_t)104, (uint8_t)142, (uint8_t)24, (uint8_t)240, (uint8_t)71, (uint8_t)136, (uint8_t)225, (uint8_t)55, (uint8_t)124, (uint8_t)6, (uint8_t)254, (uint8_t)153, (uint8_t)235, (uint8_t)214, (uint8_t)223, (uint8_t)58, (uint8_t)88, (uint8_t)118, (uint8_t)131, (uint8_t)220, (uint8_t)183, (uint8_t)18, (uint8_t)65, (uint8_t)199, (uint8_t)164, (uint8_t)157, (uint8_t)49, (uint8_t)42, (uint8_t)111, (uint8_t)215, (uint8_t)227, (uint8_t)76, (uint8_t)34, (uint8_t)249, (uint8_t)182, (uint8_t)255, (uint8_t)211, (uint8_t)6, (uint8_t)62, (uint8_t)175, (uint8_t)250, (uint8_t)149, (uint8_t)166, (uint8_t)69, (uint8_t)218, (uint8_t)16, (uint8_t)84, (uint8_t)175, (uint8_t)92, (uint8_t)35, (uint8_t)39, (uint8_t)94, (uint8_t)198, (uint8_t)137, (uint8_t)34, (uint8_t)202, (uint8_t)102, (uint8_t)190, (uint8_t)3, (uint8_t)123, (uint8_t)233, (uint8_t)6, (uint8_t)88, (uint8_t)57, (uint8_t)98, (uint8_t)112, (uint8_t)154, (uint8_t)95, (uint8_t)154, (uint8_t)25, (uint8_t)204, (uint8_t)217, (uint8_t)183, (uint8_t)165, (uint8_t)34, (uint8_t)10, (uint8_t)140, (uint8_t)13, (uint8_t)199, (uint8_t)52, (uint8_t)199, (uint8_t)103, (uint8_t)105, (uint8_t)3, (uint8_t)178, (uint8_t)196, (uint8_t)126, (uint8_t)3, (uint8_t)97, (uint8_t)57, (uint8_t)171, (uint8_t)101, (uint8_t)173, (uint8_t)101, (uint8_t)35} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)25748);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2842064883L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)2299);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)52715);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)32426);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t) -1680792476);
    assert(p133_mask_GET(pack) == (uint64_t)1175738442202476441L);
    assert(p133_lon_GET(pack) == (int32_t)1877846739);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)13429);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)46267);
    assert(p134_lat_GET(pack) == (int32_t)1104242771);
    {
        int16_t exemplary[] =  {(int16_t)1199, (int16_t)20481, (int16_t)4272, (int16_t) -22772, (int16_t) -13635, (int16_t) -9386, (int16_t) -12757, (int16_t)19427, (int16_t)7735, (int16_t)1018, (int16_t) -15474, (int16_t)27771, (int16_t) -1486, (int16_t) -3322, (int16_t) -10197, (int16_t)13074} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t) -3716587);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)870113501);
    assert(p135_lat_GET(pack) == (int32_t) -782270563);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float)1.9848585E38F);
    assert(p136_lon_GET(pack) == (int32_t)1099994742);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)47899);
    assert(p136_current_height_GET(pack) == (float)1.5118206E37F);
    assert(p136_lat_GET(pack) == (int32_t) -1179153348);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)42629);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)62198);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)14148);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)625015381L);
    assert(p137_press_abs_GET(pack) == (float) -2.7006213E38F);
    assert(p137_press_diff_GET(pack) == (float) -5.11746E37F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_y_GET(pack) == (float) -2.3964334E38F);
    {
        float exemplary[] =  {4.266117E37F, 2.7252516E38F, -3.2727635E38F, -1.5981577E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float)9.982675E37F);
    assert(p138_time_usec_GET(pack) == (uint64_t)8863411341774034002L);
    assert(p138_x_GET(pack) == (float)5.086798E37F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)1069360279198331310L);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)58);
    {
        float exemplary[] =  {-2.488098E38F, 2.444599E38F, 5.283872E37F, 9.4195174E36F, -1.906638E38F, -1.4362858E38F, -1.4689941E38F, 2.8238375E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)239);
    {
        float exemplary[] =  {-2.9663774E38F, 2.516918E38F, 2.206229E38F, -3.0407508E37F, -6.315314E36F, -7.1270673E37F, -2.9817274E38F, -1.2165541E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)7918130278400697566L);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_relative_GET(pack) == (float) -2.2497905E37F);
    assert(p141_altitude_local_GET(pack) == (float)3.1540402E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -9.548136E37F);
    assert(p141_altitude_terrain_GET(pack) == (float)7.978406E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.7685378E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)1.1745619E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)6905251012933281502L);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)172, (uint8_t)209, (uint8_t)242, (uint8_t)112, (uint8_t)118, (uint8_t)139, (uint8_t)214, (uint8_t)215, (uint8_t)208, (uint8_t)83, (uint8_t)103, (uint8_t)238, (uint8_t)170, (uint8_t)1, (uint8_t)116, (uint8_t)236, (uint8_t)176, (uint8_t)242, (uint8_t)195, (uint8_t)207, (uint8_t)87, (uint8_t)105, (uint8_t)147, (uint8_t)100, (uint8_t)128, (uint8_t)80, (uint8_t)250, (uint8_t)59, (uint8_t)51, (uint8_t)173, (uint8_t)98, (uint8_t)70, (uint8_t)114, (uint8_t)20, (uint8_t)137, (uint8_t)217, (uint8_t)80, (uint8_t)182, (uint8_t)15, (uint8_t)57, (uint8_t)199, (uint8_t)6, (uint8_t)167, (uint8_t)3, (uint8_t)10, (uint8_t)127, (uint8_t)47, (uint8_t)217, (uint8_t)148, (uint8_t)55, (uint8_t)123, (uint8_t)70, (uint8_t)149, (uint8_t)213, (uint8_t)44, (uint8_t)4, (uint8_t)164, (uint8_t)193, (uint8_t)214, (uint8_t)39, (uint8_t)71, (uint8_t)210, (uint8_t)125, (uint8_t)131, (uint8_t)235, (uint8_t)220, (uint8_t)38, (uint8_t)163, (uint8_t)16, (uint8_t)249, (uint8_t)117, (uint8_t)169, (uint8_t)43, (uint8_t)95, (uint8_t)60, (uint8_t)66, (uint8_t)238, (uint8_t)221, (uint8_t)82, (uint8_t)208, (uint8_t)11, (uint8_t)131, (uint8_t)166, (uint8_t)47, (uint8_t)163, (uint8_t)214, (uint8_t)211, (uint8_t)56, (uint8_t)178, (uint8_t)184, (uint8_t)244, (uint8_t)229, (uint8_t)203, (uint8_t)251, (uint8_t)73, (uint8_t)46, (uint8_t)71, (uint8_t)86, (uint8_t)252, (uint8_t)26, (uint8_t)148, (uint8_t)224, (uint8_t)124, (uint8_t)122, (uint8_t)146, (uint8_t)221, (uint8_t)180, (uint8_t)176, (uint8_t)55, (uint8_t)39, (uint8_t)214, (uint8_t)39, (uint8_t)45, (uint8_t)31, (uint8_t)115, (uint8_t)198, (uint8_t)242, (uint8_t)36, (uint8_t)110, (uint8_t)159} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)47);
    {
        uint8_t exemplary[] =  {(uint8_t)76, (uint8_t)96, (uint8_t)62, (uint8_t)145, (uint8_t)128, (uint8_t)116, (uint8_t)142, (uint8_t)79, (uint8_t)170, (uint8_t)19, (uint8_t)131, (uint8_t)50, (uint8_t)125, (uint8_t)68, (uint8_t)184, (uint8_t)198, (uint8_t)252, (uint8_t)107, (uint8_t)84, (uint8_t)27, (uint8_t)134, (uint8_t)196, (uint8_t)40, (uint8_t)61, (uint8_t)73, (uint8_t)51, (uint8_t)3, (uint8_t)183, (uint8_t)171, (uint8_t)228, (uint8_t)195, (uint8_t)193, (uint8_t)111, (uint8_t)210, (uint8_t)30, (uint8_t)223, (uint8_t)174, (uint8_t)228, (uint8_t)100, (uint8_t)247, (uint8_t)73, (uint8_t)116, (uint8_t)136, (uint8_t)22, (uint8_t)192, (uint8_t)10, (uint8_t)190, (uint8_t)65, (uint8_t)172, (uint8_t)81, (uint8_t)206, (uint8_t)165, (uint8_t)33, (uint8_t)170, (uint8_t)244, (uint8_t)159, (uint8_t)52, (uint8_t)106, (uint8_t)55, (uint8_t)107, (uint8_t)220, (uint8_t)195, (uint8_t)211, (uint8_t)64, (uint8_t)167, (uint8_t)198, (uint8_t)14, (uint8_t)125, (uint8_t)104, (uint8_t)244, (uint8_t)146, (uint8_t)119, (uint8_t)81, (uint8_t)3, (uint8_t)195, (uint8_t)176, (uint8_t)29, (uint8_t)13, (uint8_t)46, (uint8_t)16, (uint8_t)175, (uint8_t)209, (uint8_t)215, (uint8_t)57, (uint8_t)7, (uint8_t)28, (uint8_t)27, (uint8_t)146, (uint8_t)156, (uint8_t)33, (uint8_t)239, (uint8_t)14, (uint8_t)123, (uint8_t)40, (uint8_t)201, (uint8_t)240, (uint8_t)91, (uint8_t)59, (uint8_t)57, (uint8_t)235, (uint8_t)15, (uint8_t)254, (uint8_t)111, (uint8_t)150, (uint8_t)192, (uint8_t)213, (uint8_t)126, (uint8_t)224, (uint8_t)248, (uint8_t)90, (uint8_t)231, (uint8_t)192, (uint8_t)158, (uint8_t)227, (uint8_t)135, (uint8_t)202, (uint8_t)137, (uint8_t)3, (uint8_t)211, (uint8_t)1} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2289145068L);
    assert(p143_press_diff_GET(pack) == (float) -7.645716E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)14901);
    assert(p143_press_abs_GET(pack) == (float)2.9312181E37F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_timestamp_GET(pack) == (uint64_t)6890959893554910070L);
    {
        float exemplary[] =  {1.6618977E38F, -6.6025085E37F, 2.3967916E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)158);
    {
        float exemplary[] =  {3.1859015E38F, 1.0463319E38F, -4.615008E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.3635044E37F, -2.111124E38F, -3.0626132E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.4290118E38F, -1.8876088E38F, 2.9783577E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -2.2856869E37F);
    assert(p144_lon_GET(pack) == (int32_t)41194787);
    assert(p144_custom_state_GET(pack) == (uint64_t)4101833158502450362L);
    assert(p144_lat_GET(pack) == (int32_t) -526236837);
    {
        float exemplary[] =  {-2.0183509E38F, 2.1867902E38F, 2.4532112E38F, 2.947612E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_vel_GET(pack) == (float)7.0129266E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)7751694203198306303L);
    assert(p146_x_acc_GET(pack) == (float) -1.6312138E38F);
    assert(p146_yaw_rate_GET(pack) == (float)2.067521E38F);
    {
        float exemplary[] =  {-1.04631E38F, 3.0032764E38F, -1.5624978E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_roll_rate_GET(pack) == (float) -5.265567E36F);
    assert(p146_y_vel_GET(pack) == (float)1.0919504E38F);
    assert(p146_airspeed_GET(pack) == (float) -2.2696787E38F);
    assert(p146_z_acc_GET(pack) == (float) -1.9393036E38F);
    {
        float exemplary[] =  {1.743435E38F, 1.8560248E38F, -1.9426107E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float)1.5792956E38F);
    assert(p146_y_pos_GET(pack) == (float)1.6672912E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.2672485E38F);
    {
        float exemplary[] =  {-2.0648306E38F, -2.5839072E38F, -2.3972295E38F, -2.7311654E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_pos_GET(pack) == (float)1.7579904E38F);
    assert(p146_z_vel_GET(pack) == (float)2.6961792E38F);
    assert(p146_x_pos_GET(pack) == (float)1.0029275E37F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -3);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_current_consumed_GET(pack) == (int32_t) -961453572);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)24962);
    {
        uint16_t exemplary[] =  {(uint16_t)61430, (uint16_t)64356, (uint16_t)11150, (uint16_t)46203, (uint16_t)12615, (uint16_t)20543, (uint16_t)10671, (uint16_t)2463, (uint16_t)64724, (uint16_t)53278} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_energy_consumed_GET(pack) == (int32_t)1927950203);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -8846);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)41120);
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)223, (uint8_t)249, (uint8_t)183, (uint8_t)32, (uint8_t)251, (uint8_t)195, (uint8_t)148} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY);
    assert(p148_uid_GET(pack) == (uint64_t)9156423010208491189L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3778533596L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)3982303932L);
    assert(p148_board_version_GET(pack) == (uint32_t)3766229091L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)47616);
    {
        uint8_t exemplary[] =  {(uint8_t)208, (uint8_t)244, (uint8_t)65, (uint8_t)248, (uint8_t)224, (uint8_t)182, (uint8_t)63, (uint8_t)67, (uint8_t)109, (uint8_t)105, (uint8_t)83, (uint8_t)252, (uint8_t)130, (uint8_t)28, (uint8_t)180, (uint8_t)196, (uint8_t)237, (uint8_t)104} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)79, (uint8_t)158, (uint8_t)33, (uint8_t)208, (uint8_t)155, (uint8_t)106, (uint8_t)253} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1050163185L);
    {
        uint8_t exemplary[] =  {(uint8_t)60, (uint8_t)1, (uint8_t)216, (uint8_t)238, (uint8_t)229, (uint8_t)160, (uint8_t)153, (uint8_t)185} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_x_TRY(ph) == (float) -1.3125192E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)8986449325971131466L);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    {
        float exemplary[] =  {3.1419189E38F, 5.5303326E37F, 1.2348504E37F, 3.1526268E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_size_y_GET(pack) == (float)4.6096294E37F);
    assert(p149_distance_GET(pack) == (float)3.2421403E38F);
    assert(p149_y_TRY(ph) == (float)2.429335E37F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p149_angle_x_GET(pack) == (float)2.9094518E37F);
    assert(p149_angle_y_GET(pack) == (float) -1.646775E38F);
    assert(p149_size_x_GET(pack) == (float) -2.125828E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)135);
    assert(p149_z_TRY(ph) == (float) -3.6774815E37F);
};


void c_LoopBackDemoChannel_on_NAV_FILTER_BIAS_220(Bounds_Inside * ph, Pack * pack)
{
    assert(p220_gyro_0_GET(pack) == (float) -1.1491147E37F);
    assert(p220_accel_0_GET(pack) == (float)1.4097145E38F);
    assert(p220_usec_GET(pack) == (uint64_t)5324030965785478683L);
    assert(p220_accel_2_GET(pack) == (float) -3.3129923E38F);
    assert(p220_gyro_2_GET(pack) == (float)1.130906E38F);
    assert(p220_accel_1_GET(pack) == (float)1.6166497E38F);
    assert(p220_gyro_1_GET(pack) == (float) -3.1352808E38F);
};


void c_LoopBackDemoChannel_on_RADIO_CALIBRATION_221(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)30293, (uint16_t)12962, (uint16_t)18473} ;
        uint16_t*  sample = p221_rudder_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)20665, (uint16_t)34904} ;
        uint16_t*  sample = p221_gyro_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)3196, (uint16_t)28507, (uint16_t)64175, (uint16_t)30191, (uint16_t)55358} ;
        uint16_t*  sample = p221_throttle_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)33819, (uint16_t)35773, (uint16_t)11830} ;
        uint16_t*  sample = p221_elevator_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)20908, (uint16_t)42337, (uint16_t)39982, (uint16_t)33878, (uint16_t)32016} ;
        uint16_t*  sample = p221_pitch_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)2477, (uint16_t)57981, (uint16_t)61712} ;
        uint16_t*  sample = p221_aileron_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UALBERTA_SYS_STATUS_222(Bounds_Inside * ph, Pack * pack)
{
    assert(p222_mode_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p222_pilot_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p222_nav_mode_GET(pack) == (uint8_t)(uint8_t)173);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.7870049E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -7.8235846E37F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.3639668E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.5975047E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)1398132538116136273L);
    assert(p230_hagl_ratio_GET(pack) == (float)2.1463241E37F);
    assert(p230_tas_ratio_GET(pack) == (float) -6.5162616E37F);
    assert(p230_vel_ratio_GET(pack) == (float)2.7015105E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -6.9030887E37F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_vert_GET(pack) == (float)2.5889583E38F);
    assert(p231_wind_x_GET(pack) == (float) -2.1351173E38F);
    assert(p231_wind_alt_GET(pack) == (float) -3.24497E38F);
    assert(p231_var_horiz_GET(pack) == (float)1.7079896E38F);
    assert(p231_wind_z_GET(pack) == (float) -3.5234764E37F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.5244966E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)9196155483526045920L);
    assert(p231_wind_y_GET(pack) == (float)1.766822E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)2.0813239E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_lon_GET(pack) == (int32_t)1782520517);
    assert(p232_hdop_GET(pack) == (float)1.2711774E38F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)40394);
    assert(p232_alt_GET(pack) == (float) -2.9249888E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)553443691L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p232_time_usec_GET(pack) == (uint64_t)458902237110125009L);
    assert(p232_ve_GET(pack) == (float)1.4887388E38F);
    assert(p232_vd_GET(pack) == (float) -4.9028273E37F);
    assert(p232_vn_GET(pack) == (float) -8.930495E37F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
    assert(p232_horiz_accuracy_GET(pack) == (float) -3.0532574E38F);
    assert(p232_vdop_GET(pack) == (float)2.4216615E38F);
    assert(p232_lat_GET(pack) == (int32_t)1112953767);
    assert(p232_vert_accuracy_GET(pack) == (float)3.0314302E38F);
    assert(p232_speed_accuracy_GET(pack) == (float)1.673422E38F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)150);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)102, (uint8_t)89, (uint8_t)219, (uint8_t)71, (uint8_t)174, (uint8_t)43, (uint8_t)125, (uint8_t)207, (uint8_t)177, (uint8_t)174, (uint8_t)241, (uint8_t)33, (uint8_t)211, (uint8_t)128, (uint8_t)248, (uint8_t)74, (uint8_t)23, (uint8_t)49, (uint8_t)97, (uint8_t)132, (uint8_t)77, (uint8_t)245, (uint8_t)98, (uint8_t)21, (uint8_t)130, (uint8_t)123, (uint8_t)123, (uint8_t)96, (uint8_t)152, (uint8_t)137, (uint8_t)30, (uint8_t)153, (uint8_t)139, (uint8_t)128, (uint8_t)146, (uint8_t)188, (uint8_t)184, (uint8_t)185, (uint8_t)27, (uint8_t)66, (uint8_t)52, (uint8_t)239, (uint8_t)27, (uint8_t)8, (uint8_t)121, (uint8_t)43, (uint8_t)231, (uint8_t)53, (uint8_t)41, (uint8_t)123, (uint8_t)136, (uint8_t)175, (uint8_t)97, (uint8_t)107, (uint8_t)118, (uint8_t)170, (uint8_t)240, (uint8_t)152, (uint8_t)214, (uint8_t)69, (uint8_t)182, (uint8_t)79, (uint8_t)108, (uint8_t)42, (uint8_t)90, (uint8_t)105, (uint8_t)200, (uint8_t)238, (uint8_t)100, (uint8_t)36, (uint8_t)25, (uint8_t)96, (uint8_t)85, (uint8_t)6, (uint8_t)125, (uint8_t)27, (uint8_t)164, (uint8_t)32, (uint8_t)17, (uint8_t)251, (uint8_t)7, (uint8_t)250, (uint8_t)64, (uint8_t)137, (uint8_t)37, (uint8_t)40, (uint8_t)193, (uint8_t)233, (uint8_t)198, (uint8_t)6, (uint8_t)81, (uint8_t)245, (uint8_t)135, (uint8_t)145, (uint8_t)192, (uint8_t)235, (uint8_t)130, (uint8_t)204, (uint8_t)105, (uint8_t)96, (uint8_t)75, (uint8_t)249, (uint8_t)243, (uint8_t)240, (uint8_t)254, (uint8_t)200, (uint8_t)165, (uint8_t)153, (uint8_t)65, (uint8_t)12, (uint8_t)43, (uint8_t)99, (uint8_t)66, (uint8_t)19, (uint8_t)189, (uint8_t)213, (uint8_t)175, (uint8_t)112, (uint8_t)169, (uint8_t)30, (uint8_t)191, (uint8_t)242, (uint8_t)195, (uint8_t)224, (uint8_t)31, (uint8_t)91, (uint8_t)244, (uint8_t)92, (uint8_t)42, (uint8_t)177, (uint8_t)63, (uint8_t)252, (uint8_t)19, (uint8_t)118, (uint8_t)246, (uint8_t)192, (uint8_t)41, (uint8_t)113, (uint8_t)252, (uint8_t)255, (uint8_t)45, (uint8_t)222, (uint8_t)18, (uint8_t)121, (uint8_t)171, (uint8_t)224, (uint8_t)105, (uint8_t)39, (uint8_t)145, (uint8_t)130, (uint8_t)227, (uint8_t)180, (uint8_t)134, (uint8_t)160, (uint8_t)11, (uint8_t)179, (uint8_t)154, (uint8_t)85, (uint8_t)153, (uint8_t)31, (uint8_t)214, (uint8_t)199, (uint8_t)121, (uint8_t)68, (uint8_t)88, (uint8_t)75, (uint8_t)251, (uint8_t)151, (uint8_t)217, (uint8_t)217, (uint8_t)162, (uint8_t)255, (uint8_t)4, (uint8_t)40, (uint8_t)254, (uint8_t)87, (uint8_t)58, (uint8_t)182, (uint8_t)205, (uint8_t)121} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -113);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)2885);
    assert(p234_latitude_GET(pack) == (int32_t) -2092738884);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)15170);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -30409);
    assert(p234_custom_mode_GET(pack) == (uint32_t)21292181L);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)74);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p234_longitude_GET(pack) == (int32_t) -1423512871);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)112);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -46);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -31505);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)1431);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -20373);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -6004);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_y_GET(pack) == (float) -1.1851617E37F);
    assert(p241_vibration_z_GET(pack) == (float) -6.5502417E37F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3588942527L);
    assert(p241_vibration_x_GET(pack) == (float)2.8100149E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)2972302286L);
    assert(p241_time_usec_GET(pack) == (uint64_t)3427803076081836493L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)1298888144L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_time_usec_TRY(ph) == (uint64_t)3204076889359991205L);
    assert(p242_approach_y_GET(pack) == (float) -6.358341E37F);
    assert(p242_longitude_GET(pack) == (int32_t) -994547043);
    assert(p242_altitude_GET(pack) == (int32_t) -1469350989);
    assert(p242_latitude_GET(pack) == (int32_t)705549743);
    assert(p242_approach_x_GET(pack) == (float)3.4199007E37F);
    assert(p242_x_GET(pack) == (float)1.1953838E38F);
    assert(p242_z_GET(pack) == (float) -4.2308719E37F);
    assert(p242_y_GET(pack) == (float)1.0480425E38F);
    assert(p242_approach_z_GET(pack) == (float)1.5086979E38F);
    {
        float exemplary[] =  {2.269594E38F, 2.4262498E38F, 1.1884889E38F, 1.3956528E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_z_GET(pack) == (float)2.045931E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)6850788107716223279L);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p243_altitude_GET(pack) == (int32_t) -340506091);
    assert(p243_approach_z_GET(pack) == (float)5.1239756E37F);
    assert(p243_x_GET(pack) == (float)2.521497E38F);
    assert(p243_approach_x_GET(pack) == (float)4.985288E37F);
    assert(p243_latitude_GET(pack) == (int32_t)1503359932);
    assert(p243_y_GET(pack) == (float) -2.5579928E38F);
    {
        float exemplary[] =  {-2.3561726E38F, -3.0368767E38F, 2.290081E38F, 2.1302633E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_longitude_GET(pack) == (int32_t) -2065631606);
    assert(p243_approach_y_GET(pack) == (float)4.7577366E37F);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)34541);
    assert(p244_interval_us_GET(pack) == (int32_t)154349688);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE);
    assert(p246_callsign_LEN(ph) == 1);
    {
        char16_t * exemplary = u"d";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_GET(pack) == (int32_t) -653651418);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)28779);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)308556026L);
    assert(p246_lon_GET(pack) == (int32_t) -1913461667);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)4066);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)47283);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -30619);
    assert(p246_lat_GET(pack) == (int32_t)1049337264);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)731044899L);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.9970724E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -4.1224936E37F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.860847E38F);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)30, (uint8_t)184, (uint8_t)104, (uint8_t)105, (uint8_t)199, (uint8_t)114, (uint8_t)193, (uint8_t)103, (uint8_t)199, (uint8_t)245, (uint8_t)9, (uint8_t)1, (uint8_t)38, (uint8_t)49, (uint8_t)233, (uint8_t)249, (uint8_t)120, (uint8_t)215, (uint8_t)213, (uint8_t)27, (uint8_t)64, (uint8_t)168, (uint8_t)84, (uint8_t)202, (uint8_t)46, (uint8_t)35, (uint8_t)151, (uint8_t)5, (uint8_t)196, (uint8_t)222, (uint8_t)16, (uint8_t)118, (uint8_t)101, (uint8_t)221, (uint8_t)162, (uint8_t)253, (uint8_t)233, (uint8_t)38, (uint8_t)179, (uint8_t)56, (uint8_t)41, (uint8_t)247, (uint8_t)119, (uint8_t)189, (uint8_t)198, (uint8_t)43, (uint8_t)33, (uint8_t)174, (uint8_t)148, (uint8_t)121, (uint8_t)48, (uint8_t)102, (uint8_t)67, (uint8_t)162, (uint8_t)94, (uint8_t)179, (uint8_t)224, (uint8_t)173, (uint8_t)136, (uint8_t)148, (uint8_t)101, (uint8_t)224, (uint8_t)97, (uint8_t)237, (uint8_t)98, (uint8_t)61, (uint8_t)104, (uint8_t)157, (uint8_t)1, (uint8_t)127, (uint8_t)120, (uint8_t)217, (uint8_t)227, (uint8_t)30, (uint8_t)84, (uint8_t)97, (uint8_t)175, (uint8_t)127, (uint8_t)141, (uint8_t)155, (uint8_t)204, (uint8_t)192, (uint8_t)121, (uint8_t)181, (uint8_t)221, (uint8_t)108, (uint8_t)169, (uint8_t)177, (uint8_t)250, (uint8_t)216, (uint8_t)40, (uint8_t)9, (uint8_t)181, (uint8_t)129, (uint8_t)200, (uint8_t)103, (uint8_t)131, (uint8_t)61, (uint8_t)178, (uint8_t)47, (uint8_t)161, (uint8_t)226, (uint8_t)186, (uint8_t)5, (uint8_t)236, (uint8_t)101, (uint8_t)136, (uint8_t)100, (uint8_t)171, (uint8_t)187, (uint8_t)81, (uint8_t)27, (uint8_t)159, (uint8_t)97, (uint8_t)60, (uint8_t)151, (uint8_t)224, (uint8_t)185, (uint8_t)229, (uint8_t)32, (uint8_t)106, (uint8_t)118, (uint8_t)52, (uint8_t)191, (uint8_t)85, (uint8_t)29, (uint8_t)1, (uint8_t)224, (uint8_t)215, (uint8_t)187, (uint8_t)169, (uint8_t)73, (uint8_t)44, (uint8_t)16, (uint8_t)99, (uint8_t)253, (uint8_t)72, (uint8_t)136, (uint8_t)199, (uint8_t)138, (uint8_t)40, (uint8_t)121, (uint8_t)237, (uint8_t)158, (uint8_t)94, (uint8_t)115, (uint8_t)246, (uint8_t)141, (uint8_t)2, (uint8_t)51, (uint8_t)140, (uint8_t)139, (uint8_t)9, (uint8_t)101, (uint8_t)165, (uint8_t)139, (uint8_t)60, (uint8_t)246, (uint8_t)119, (uint8_t)105, (uint8_t)207, (uint8_t)199, (uint8_t)127, (uint8_t)52, (uint8_t)249, (uint8_t)69, (uint8_t)30, (uint8_t)199, (uint8_t)210, (uint8_t)78, (uint8_t)39, (uint8_t)47, (uint8_t)202, (uint8_t)226, (uint8_t)65, (uint8_t)162, (uint8_t)32, (uint8_t)45, (uint8_t)71, (uint8_t)176, (uint8_t)24, (uint8_t)97, (uint8_t)22, (uint8_t)22, (uint8_t)0, (uint8_t)196, (uint8_t)242, (uint8_t)79, (uint8_t)82, (uint8_t)7, (uint8_t)73, (uint8_t)176, (uint8_t)128, (uint8_t)205, (uint8_t)187, (uint8_t)129, (uint8_t)173, (uint8_t)150, (uint8_t)196, (uint8_t)232, (uint8_t)183, (uint8_t)143, (uint8_t)255, (uint8_t)181, (uint8_t)10, (uint8_t)42, (uint8_t)65, (uint8_t)67, (uint8_t)63, (uint8_t)71, (uint8_t)82, (uint8_t)240, (uint8_t)152, (uint8_t)149, (uint8_t)115, (uint8_t)103, (uint8_t)193, (uint8_t)141, (uint8_t)23, (uint8_t)208, (uint8_t)184, (uint8_t)115, (uint8_t)210, (uint8_t)104, (uint8_t)159, (uint8_t)180, (uint8_t)136, (uint8_t)246, (uint8_t)87, (uint8_t)179, (uint8_t)37, (uint8_t)72, (uint8_t)54, (uint8_t)235, (uint8_t)195, (uint8_t)229, (uint8_t)132, (uint8_t)84, (uint8_t)31, (uint8_t)177, (uint8_t)88, (uint8_t)203, (uint8_t)163, (uint8_t)143, (uint8_t)21, (uint8_t)111, (uint8_t)210, (uint8_t)57} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)48270);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)94);
    {
        int8_t exemplary[] =  {(int8_t) -123, (int8_t)78, (int8_t) -60, (int8_t)78, (int8_t) -59, (int8_t) -44, (int8_t) -27, (int8_t) -53, (int8_t) -7, (int8_t)34, (int8_t) -99, (int8_t) -86, (int8_t)13, (int8_t)4, (int8_t) -120, (int8_t)124, (int8_t) -22, (int8_t)20, (int8_t) -94, (int8_t)45, (int8_t)108, (int8_t) -29, (int8_t) -127, (int8_t) -39, (int8_t)40, (int8_t)93, (int8_t)81, (int8_t)89, (int8_t) -93, (int8_t)39, (int8_t) -4, (int8_t)96} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)65079);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)189);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_z_GET(pack) == (float)3.3709065E38F);
    assert(p250_x_GET(pack) == (float) -2.179323E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5858136002014523667L);
    assert(p250_y_GET(pack) == (float) -3.0941467E37F);
    assert(p250_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"Wsqdtxxny";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3963163495L);
    assert(p251_value_GET(pack) == (float)8.667248E37F);
    assert(p251_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"g";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3709889355L);
    assert(p252_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"fejakn";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_value_GET(pack) == (int32_t) -1373588649);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_WARNING);
    assert(p253_text_LEN(ph) == 19);
    {
        char16_t * exemplary = u"lhhAbwJpvbnoacchRfv";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1128144753L);
    assert(p254_value_GET(pack) == (float)2.8320008E38F);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)8737286716707572250L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)121);
    {
        uint8_t exemplary[] =  {(uint8_t)55, (uint8_t)157, (uint8_t)180, (uint8_t)219, (uint8_t)33, (uint8_t)119, (uint8_t)112, (uint8_t)120, (uint8_t)200, (uint8_t)207, (uint8_t)242, (uint8_t)245, (uint8_t)3, (uint8_t)147, (uint8_t)242, (uint8_t)94, (uint8_t)148, (uint8_t)225, (uint8_t)253, (uint8_t)59, (uint8_t)145, (uint8_t)154, (uint8_t)119, (uint8_t)203, (uint8_t)169, (uint8_t)109, (uint8_t)233, (uint8_t)49, (uint8_t)96, (uint8_t)234, (uint8_t)176, (uint8_t)214} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)43);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1637976829L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1240174145L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p258_tune_LEN(ph) == 14);
    {
        char16_t * exemplary = u"tamxjsbqditlqw";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_v_GET(pack) == (float) -1.2934404E38F);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1989799924L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES);
    assert(p259_focal_length_GET(pack) == (float)3.031503E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1887218845L);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)14796);
    {
        uint8_t exemplary[] =  {(uint8_t)201, (uint8_t)167, (uint8_t)143, (uint8_t)56, (uint8_t)7, (uint8_t)192, (uint8_t)255, (uint8_t)118, (uint8_t)102, (uint8_t)29, (uint8_t)121, (uint8_t)59, (uint8_t)229, (uint8_t)20, (uint8_t)31, (uint8_t)182, (uint8_t)198, (uint8_t)148, (uint8_t)28, (uint8_t)65, (uint8_t)229, (uint8_t)173, (uint8_t)80, (uint8_t)1, (uint8_t)197, (uint8_t)106, (uint8_t)87, (uint8_t)122, (uint8_t)154, (uint8_t)45, (uint8_t)180, (uint8_t)46} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)232);
    {
        uint8_t exemplary[] =  {(uint8_t)51, (uint8_t)144, (uint8_t)97, (uint8_t)226, (uint8_t)90, (uint8_t)150, (uint8_t)195, (uint8_t)54, (uint8_t)158, (uint8_t)183, (uint8_t)40, (uint8_t)16, (uint8_t)22, (uint8_t)46, (uint8_t)19, (uint8_t)128, (uint8_t)59, (uint8_t)249, (uint8_t)248, (uint8_t)166, (uint8_t)103, (uint8_t)251, (uint8_t)239, (uint8_t)149, (uint8_t)47, (uint8_t)170, (uint8_t)45, (uint8_t)182, (uint8_t)31, (uint8_t)70, (uint8_t)77, (uint8_t)143} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)36038);
    assert(p259_cam_definition_uri_LEN(ph) == 59);
    {
        char16_t * exemplary = u"ynguqjoglrskqMuyBezUnxUtoeHeaHyVumxaXlShqltpxgiazzqinzkkYwe";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_h_GET(pack) == (float)2.8187355E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)48276);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1854242732L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p261_read_speed_GET(pack) == (float) -2.3516484E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3163884028L);
    assert(p261_total_capacity_GET(pack) == (float) -2.2829105E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p261_used_capacity_GET(pack) == (float) -1.3838944E38F);
    assert(p261_write_speed_GET(pack) == (float) -2.6683212E38F);
    assert(p261_available_capacity_GET(pack) == (float)1.3658905E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)224);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)4165679333L);
    assert(p262_available_capacity_GET(pack) == (float)7.132658E37F);
    assert(p262_image_interval_GET(pack) == (float) -1.2164434E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)3029243807L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)207);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_lat_GET(pack) == (int32_t) -673971637);
    assert(p263_time_utc_GET(pack) == (uint64_t)2280144257959568461L);
    assert(p263_image_index_GET(pack) == (int32_t)1643373056);
    assert(p263_lon_GET(pack) == (int32_t) -1715385798);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -77);
    assert(p263_alt_GET(pack) == (int32_t) -1822330);
    {
        float exemplary[] =  {2.49254E38F, 5.3601703E37F, 1.973693E38F, -3.3311589E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t) -216761281);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1599961843L);
    assert(p263_file_url_LEN(ph) == 81);
    {
        char16_t * exemplary = u"tyGvibyihimtroznnndbAiaojtjsumyjpshYfvGcnpxogaMwlhvfyeiaztkduzwxsvnlcsoaIFrazlyOn";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 162);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)6541853961459594206L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)8842626941677475250L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3046414163L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)4349507890888381586L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)2.1129192E38F);
    assert(p265_roll_GET(pack) == (float)3.5628405E37F);
    assert(p265_pitch_GET(pack) == (float)2.1953926E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)4227332680L);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)11614);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)46);
    {
        uint8_t exemplary[] =  {(uint8_t)155, (uint8_t)255, (uint8_t)105, (uint8_t)199, (uint8_t)251, (uint8_t)34, (uint8_t)250, (uint8_t)197, (uint8_t)114, (uint8_t)56, (uint8_t)48, (uint8_t)95, (uint8_t)227, (uint8_t)3, (uint8_t)53, (uint8_t)235, (uint8_t)199, (uint8_t)42, (uint8_t)211, (uint8_t)135, (uint8_t)71, (uint8_t)168, (uint8_t)121, (uint8_t)111, (uint8_t)248, (uint8_t)17, (uint8_t)214, (uint8_t)12, (uint8_t)245, (uint8_t)155, (uint8_t)128, (uint8_t)191, (uint8_t)90, (uint8_t)198, (uint8_t)188, (uint8_t)172, (uint8_t)115, (uint8_t)94, (uint8_t)141, (uint8_t)191, (uint8_t)121, (uint8_t)247, (uint8_t)198, (uint8_t)125, (uint8_t)18, (uint8_t)85, (uint8_t)99, (uint8_t)7, (uint8_t)193, (uint8_t)15, (uint8_t)74, (uint8_t)180, (uint8_t)190, (uint8_t)101, (uint8_t)205, (uint8_t)213, (uint8_t)138, (uint8_t)249, (uint8_t)165, (uint8_t)130, (uint8_t)81, (uint8_t)237, (uint8_t)180, (uint8_t)37, (uint8_t)232, (uint8_t)27, (uint8_t)88, (uint8_t)76, (uint8_t)166, (uint8_t)78, (uint8_t)159, (uint8_t)29, (uint8_t)61, (uint8_t)135, (uint8_t)97, (uint8_t)241, (uint8_t)4, (uint8_t)173, (uint8_t)45, (uint8_t)169, (uint8_t)52, (uint8_t)175, (uint8_t)20, (uint8_t)74, (uint8_t)145, (uint8_t)41, (uint8_t)208, (uint8_t)244, (uint8_t)64, (uint8_t)223, (uint8_t)202, (uint8_t)26, (uint8_t)100, (uint8_t)143, (uint8_t)128, (uint8_t)164, (uint8_t)181, (uint8_t)152, (uint8_t)74, (uint8_t)185, (uint8_t)124, (uint8_t)59, (uint8_t)160, (uint8_t)167, (uint8_t)3, (uint8_t)98, (uint8_t)128, (uint8_t)171, (uint8_t)219, (uint8_t)27, (uint8_t)114, (uint8_t)56, (uint8_t)126, (uint8_t)30, (uint8_t)45, (uint8_t)104, (uint8_t)199, (uint8_t)39, (uint8_t)181, (uint8_t)122, (uint8_t)123, (uint8_t)228, (uint8_t)0, (uint8_t)3, (uint8_t)38, (uint8_t)19, (uint8_t)90, (uint8_t)60, (uint8_t)39, (uint8_t)146, (uint8_t)104, (uint8_t)116, (uint8_t)187, (uint8_t)243, (uint8_t)103, (uint8_t)203, (uint8_t)8, (uint8_t)0, (uint8_t)193, (uint8_t)103, (uint8_t)88, (uint8_t)40, (uint8_t)21, (uint8_t)25, (uint8_t)150, (uint8_t)140, (uint8_t)178, (uint8_t)76, (uint8_t)175, (uint8_t)254, (uint8_t)169, (uint8_t)240, (uint8_t)127, (uint8_t)94, (uint8_t)199, (uint8_t)27, (uint8_t)157, (uint8_t)21, (uint8_t)16, (uint8_t)112, (uint8_t)143, (uint8_t)109, (uint8_t)26, (uint8_t)7, (uint8_t)2, (uint8_t)101, (uint8_t)4, (uint8_t)13, (uint8_t)30, (uint8_t)95, (uint8_t)138, (uint8_t)145, (uint8_t)98, (uint8_t)54, (uint8_t)51, (uint8_t)219, (uint8_t)34, (uint8_t)12, (uint8_t)191, (uint8_t)61, (uint8_t)80, (uint8_t)196, (uint8_t)17, (uint8_t)128, (uint8_t)158, (uint8_t)40, (uint8_t)142, (uint8_t)29, (uint8_t)100, (uint8_t)84, (uint8_t)80, (uint8_t)125, (uint8_t)8, (uint8_t)197, (uint8_t)160, (uint8_t)232, (uint8_t)192, (uint8_t)48, (uint8_t)94, (uint8_t)48, (uint8_t)66, (uint8_t)240, (uint8_t)119, (uint8_t)151, (uint8_t)149, (uint8_t)244, (uint8_t)226, (uint8_t)128, (uint8_t)89, (uint8_t)83, (uint8_t)248, (uint8_t)181, (uint8_t)180, (uint8_t)157, (uint8_t)214, (uint8_t)76, (uint8_t)3, (uint8_t)214, (uint8_t)185, (uint8_t)233, (uint8_t)49, (uint8_t)129, (uint8_t)62, (uint8_t)61, (uint8_t)2, (uint8_t)70, (uint8_t)237, (uint8_t)193, (uint8_t)93, (uint8_t)198, (uint8_t)166, (uint8_t)213, (uint8_t)152, (uint8_t)28, (uint8_t)63, (uint8_t)149, (uint8_t)63, (uint8_t)137, (uint8_t)73, (uint8_t)80, (uint8_t)130, (uint8_t)87, (uint8_t)112, (uint8_t)201, (uint8_t)106, (uint8_t)63, (uint8_t)170, (uint8_t)171, (uint8_t)205} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)117);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)3, (uint8_t)246, (uint8_t)47, (uint8_t)239, (uint8_t)22, (uint8_t)246, (uint8_t)251, (uint8_t)238, (uint8_t)61, (uint8_t)188, (uint8_t)114, (uint8_t)106, (uint8_t)106, (uint8_t)2, (uint8_t)218, (uint8_t)40, (uint8_t)159, (uint8_t)23, (uint8_t)165, (uint8_t)122, (uint8_t)137, (uint8_t)153, (uint8_t)169, (uint8_t)40, (uint8_t)32, (uint8_t)152, (uint8_t)124, (uint8_t)232, (uint8_t)60, (uint8_t)98, (uint8_t)56, (uint8_t)174, (uint8_t)5, (uint8_t)205, (uint8_t)236, (uint8_t)201, (uint8_t)239, (uint8_t)36, (uint8_t)25, (uint8_t)23, (uint8_t)253, (uint8_t)128, (uint8_t)192, (uint8_t)119, (uint8_t)48, (uint8_t)63, (uint8_t)220, (uint8_t)178, (uint8_t)65, (uint8_t)147, (uint8_t)109, (uint8_t)88, (uint8_t)214, (uint8_t)62, (uint8_t)135, (uint8_t)95, (uint8_t)73, (uint8_t)151, (uint8_t)129, (uint8_t)209, (uint8_t)15, (uint8_t)170, (uint8_t)235, (uint8_t)139, (uint8_t)82, (uint8_t)236, (uint8_t)195, (uint8_t)34, (uint8_t)14, (uint8_t)208, (uint8_t)149, (uint8_t)56, (uint8_t)214, (uint8_t)113, (uint8_t)176, (uint8_t)108, (uint8_t)204, (uint8_t)187, (uint8_t)47, (uint8_t)50, (uint8_t)236, (uint8_t)22, (uint8_t)96, (uint8_t)203, (uint8_t)111, (uint8_t)40, (uint8_t)28, (uint8_t)187, (uint8_t)38, (uint8_t)54, (uint8_t)252, (uint8_t)179, (uint8_t)108, (uint8_t)149, (uint8_t)181, (uint8_t)71, (uint8_t)140, (uint8_t)194, (uint8_t)61, (uint8_t)150, (uint8_t)85, (uint8_t)252, (uint8_t)89, (uint8_t)234, (uint8_t)152, (uint8_t)117, (uint8_t)33, (uint8_t)15, (uint8_t)249, (uint8_t)242, (uint8_t)193, (uint8_t)148, (uint8_t)98, (uint8_t)26, (uint8_t)161, (uint8_t)116, (uint8_t)12, (uint8_t)82, (uint8_t)230, (uint8_t)162, (uint8_t)197, (uint8_t)17, (uint8_t)197, (uint8_t)89, (uint8_t)113, (uint8_t)68, (uint8_t)218, (uint8_t)66, (uint8_t)2, (uint8_t)248, (uint8_t)88, (uint8_t)103, (uint8_t)148, (uint8_t)214, (uint8_t)39, (uint8_t)32, (uint8_t)104, (uint8_t)190, (uint8_t)104, (uint8_t)220, (uint8_t)87, (uint8_t)138, (uint8_t)52, (uint8_t)165, (uint8_t)194, (uint8_t)80, (uint8_t)84, (uint8_t)99, (uint8_t)79, (uint8_t)43, (uint8_t)251, (uint8_t)0, (uint8_t)214, (uint8_t)218, (uint8_t)218, (uint8_t)13, (uint8_t)135, (uint8_t)198, (uint8_t)167, (uint8_t)100, (uint8_t)156, (uint8_t)67, (uint8_t)153, (uint8_t)233, (uint8_t)227, (uint8_t)124, (uint8_t)143, (uint8_t)93, (uint8_t)227, (uint8_t)34, (uint8_t)187, (uint8_t)161, (uint8_t)187, (uint8_t)38, (uint8_t)113, (uint8_t)24, (uint8_t)37, (uint8_t)208, (uint8_t)159, (uint8_t)49, (uint8_t)251, (uint8_t)39, (uint8_t)148, (uint8_t)32, (uint8_t)180, (uint8_t)76, (uint8_t)144, (uint8_t)161, (uint8_t)14, (uint8_t)141, (uint8_t)156, (uint8_t)246, (uint8_t)148, (uint8_t)108, (uint8_t)204, (uint8_t)19, (uint8_t)115, (uint8_t)35, (uint8_t)132, (uint8_t)70, (uint8_t)194, (uint8_t)225, (uint8_t)110, (uint8_t)175, (uint8_t)144, (uint8_t)71, (uint8_t)128, (uint8_t)172, (uint8_t)113, (uint8_t)131, (uint8_t)59, (uint8_t)236, (uint8_t)204, (uint8_t)222, (uint8_t)251, (uint8_t)28, (uint8_t)201, (uint8_t)222, (uint8_t)244, (uint8_t)112, (uint8_t)113, (uint8_t)172, (uint8_t)17, (uint8_t)20, (uint8_t)57, (uint8_t)221, (uint8_t)206, (uint8_t)254, (uint8_t)145, (uint8_t)228, (uint8_t)17, (uint8_t)248, (uint8_t)202, (uint8_t)60, (uint8_t)150, (uint8_t)248, (uint8_t)198, (uint8_t)33, (uint8_t)88, (uint8_t)209, (uint8_t)7, (uint8_t)49, (uint8_t)73, (uint8_t)248, (uint8_t)249, (uint8_t)22, (uint8_t)80, (uint8_t)3} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)26605);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)255);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)51323);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)32515);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)8438);
    assert(p269_bitrate_GET(pack) == (uint32_t)2816672418L);
    assert(p269_framerate_GET(pack) == (float) -2.0777426E38F);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)22884);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p269_uri_LEN(ph) == 112);
    {
        char16_t * exemplary = u"cpddwldptvzpacuicqmwjyucxterAttbozcihxcecfdhvnbDvUtinfytxusTzyrmqWmhesaocZigqVuxGtrbpxsrcjzznswFxohnrtwgptWtxjhq";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 224);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_bitrate_GET(pack) == (uint32_t)2950591103L);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)7505);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)34890);
    assert(p270_framerate_GET(pack) == (float) -1.0060439E38F);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)60760);
    assert(p270_uri_LEN(ph) == 59);
    {
        char16_t * exemplary = u"uuktoKcutyePidpdutocwgcjhkbrcsuvqXqusNdPfiWydhtmSmumgwQfYfI";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)175);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 3);
    {
        char16_t * exemplary = u"gUg";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 12);
    {
        char16_t * exemplary = u"ddvoapmsoruf";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)21280);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)3179);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)38836);
    {
        uint8_t exemplary[] =  {(uint8_t)182, (uint8_t)180, (uint8_t)147, (uint8_t)160, (uint8_t)77, (uint8_t)137, (uint8_t)189, (uint8_t)208} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)148, (uint8_t)205, (uint8_t)19, (uint8_t)22, (uint8_t)10, (uint8_t)100, (uint8_t)87, (uint8_t)237} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)2552444837L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_time_usec_GET(pack) == (uint64_t)7269482200254147010L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)13812);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)167);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)160);
    {
        uint8_t exemplary[] =  {(uint8_t)221, (uint8_t)253, (uint8_t)3, (uint8_t)231, (uint8_t)25, (uint8_t)66, (uint8_t)194, (uint8_t)50, (uint8_t)238, (uint8_t)97, (uint8_t)185, (uint8_t)124, (uint8_t)88, (uint8_t)150, (uint8_t)98, (uint8_t)113} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)275851539L);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2784715429L);
    assert(p311_name_LEN(ph) == 68);
    {
        char16_t * exemplary = u"dsrctpfmcggnkhrpahqpbsHaixjkOtIvhlwjsiklOxmqgyAadgwNgqsumtiaqyvlijcu";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 136);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)6325359743379233643L);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)10726);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p320_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"ebjskji";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)254);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 51);
    {
        char16_t * exemplary = u"stiqzqetnygWEhgzkdzkekvcswggfybgsyxewnmkSjfTziiebbx";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 102);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)54569);
    assert(p322_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"zsn";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)11926);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p323_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"n";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_value_LEN(ph) == 36);
    {
        char16_t * exemplary = u"nPmqgeMyeJuoSvjyfnwieowrqphowrscmfrl";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 72);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"cpcKxEsmzh";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
    assert(p324_param_value_LEN(ph) == 58);
    {
        char16_t * exemplary = u"ZJllgvkifbmsctjfinhMlogcPjpdoerunlxbooLmflgujjOnxzmajtebrj";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)42);
    {
        uint16_t exemplary[] =  {(uint16_t)27645, (uint16_t)53720, (uint16_t)27339, (uint16_t)61161, (uint16_t)23271, (uint16_t)8219, (uint16_t)25997, (uint16_t)10378, (uint16_t)62568, (uint16_t)34440, (uint16_t)51501, (uint16_t)1601, (uint16_t)43008, (uint16_t)24281, (uint16_t)35314, (uint16_t)57920, (uint16_t)14480, (uint16_t)24892, (uint16_t)2186, (uint16_t)7800, (uint16_t)29133, (uint16_t)61558, (uint16_t)6422, (uint16_t)45281, (uint16_t)48858, (uint16_t)22651, (uint16_t)61593, (uint16_t)60131, (uint16_t)22915, (uint16_t)33306, (uint16_t)20816, (uint16_t)27670, (uint16_t)36511, (uint16_t)25521, (uint16_t)64181, (uint16_t)26007, (uint16_t)44255, (uint16_t)13866, (uint16_t)36209, (uint16_t)19644, (uint16_t)39343, (uint16_t)25399, (uint16_t)36077, (uint16_t)35925, (uint16_t)62050, (uint16_t)11017, (uint16_t)12030, (uint16_t)58226, (uint16_t)9293, (uint16_t)48548, (uint16_t)59796, (uint16_t)38533, (uint16_t)44871, (uint16_t)38494, (uint16_t)46923, (uint16_t)17761, (uint16_t)56244, (uint16_t)2424, (uint16_t)47293, (uint16_t)60398, (uint16_t)28004, (uint16_t)3268, (uint16_t)43626, (uint16_t)17965, (uint16_t)20021, (uint16_t)2626, (uint16_t)54435, (uint16_t)41394, (uint16_t)42559, (uint16_t)46004, (uint16_t)6468, (uint16_t)14540} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)5372);
    assert(p330_time_usec_GET(pack) == (uint64_t)7155201189049258274L);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)34673);
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
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)1876038293L, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_FLIGHT_TERMINATION, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_FLAPPING_WING, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count1_SET((uint16_t)(uint16_t)39883, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)54362, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)64799, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)867, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)5479, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)62304, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)16047, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)56015, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)22620, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)92, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)1586137288L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)976968066136330067L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_type_mask_SET((uint16_t)(uint16_t)29061, PH.base.pack) ;
        p3_x_SET((float)2.1405365E38F, PH.base.pack) ;
        p3_afz_SET((float)2.2049222E38F, PH.base.pack) ;
        p3_afy_SET((float) -3.195795E38F, PH.base.pack) ;
        p3_z_SET((float) -2.0583182E38F, PH.base.pack) ;
        p3_y_SET((float)3.0587547E38F, PH.base.pack) ;
        p3_afx_SET((float) -2.3047115E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p3_vy_SET((float)1.5999024E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.746817E38F, PH.base.pack) ;
        p3_yaw_SET((float) -7.9829896E37F, PH.base.pack) ;
        p3_vz_SET((float) -2.5394152E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2275752076L, PH.base.pack) ;
        p3_vx_SET((float)3.2553499E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)1847951500076775548L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p4_seq_SET((uint32_t)1777777790L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        {
            char16_t* passkey = u"flo";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"miuhxnoI";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)1264363521L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"fgqxhpTTu";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t) -12649, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"s";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float) -2.523951E38F, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)51308, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)58799, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"vYjyclr";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p23_param_value_SET((float) -7.567178E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_alt_ellipsoid_SET((int32_t)970900958, &PH) ;
        p24_lat_SET((int32_t)388476210, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)2317819909003995518L, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p24_lon_SET((int32_t)1552045530, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)56582, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1600972762L, &PH) ;
        p24_h_acc_SET((uint32_t)1582898667L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)41897, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)15312, PH.base.pack) ;
        p24_alt_SET((int32_t) -1480835848, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1699658146L, &PH) ;
        p24_v_acc_SET((uint32_t)1559443660L, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)4300, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)185, (uint8_t)101, (uint8_t)145, (uint8_t)248, (uint8_t)151, (uint8_t)56, (uint8_t)45, (uint8_t)163, (uint8_t)192, (uint8_t)44, (uint8_t)30, (uint8_t)130, (uint8_t)115, (uint8_t)168, (uint8_t)78, (uint8_t)84, (uint8_t)81, (uint8_t)108, (uint8_t)136, (uint8_t)124};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)148, (uint8_t)118, (uint8_t)253, (uint8_t)86, (uint8_t)39, (uint8_t)18, (uint8_t)82, (uint8_t)116, (uint8_t)42, (uint8_t)232, (uint8_t)162, (uint8_t)103, (uint8_t)113, (uint8_t)8, (uint8_t)76, (uint8_t)68, (uint8_t)130, (uint8_t)90, (uint8_t)112, (uint8_t)254};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)115, (uint8_t)192, (uint8_t)196, (uint8_t)70, (uint8_t)195, (uint8_t)148, (uint8_t)191, (uint8_t)99, (uint8_t)79, (uint8_t)94, (uint8_t)211, (uint8_t)78, (uint8_t)136, (uint8_t)134, (uint8_t)189, (uint8_t)132, (uint8_t)207, (uint8_t)145, (uint8_t)250, (uint8_t)247};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)94, (uint8_t)172, (uint8_t)151, (uint8_t)50, (uint8_t)231, (uint8_t)120, (uint8_t)225, (uint8_t)76, (uint8_t)31, (uint8_t)228, (uint8_t)211, (uint8_t)1, (uint8_t)102, (uint8_t)211, (uint8_t)172, (uint8_t)32, (uint8_t)47, (uint8_t)118, (uint8_t)162, (uint8_t)96};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)35, (uint8_t)38, (uint8_t)200, (uint8_t)82, (uint8_t)5, (uint8_t)62, (uint8_t)192, (uint8_t)32, (uint8_t)105, (uint8_t)142, (uint8_t)150, (uint8_t)189, (uint8_t)139, (uint8_t)254, (uint8_t)83, (uint8_t)29, (uint8_t)137, (uint8_t)122, (uint8_t)190, (uint8_t)208};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_ymag_SET((int16_t)(int16_t) -12261, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)11127, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)29201, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)2834321512L, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)8185, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)27674, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -31212, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)2831, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)10853, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)5606, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_zmag_SET((int16_t)(int16_t) -10835, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -6584, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -29468, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -31507, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)5778, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)21658, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)9344, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -30269, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -18728, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)9008196673823760817L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t)23818, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)20610, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)19291, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)1393218421041089717L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -846, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float)1.2925525E38F, PH.base.pack) ;
        p29_press_abs_SET((float) -3.250265E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1728010497L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)32222, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_time_boot_ms_SET((uint32_t)1430481363L, PH.base.pack) ;
        p30_pitch_SET((float) -1.9590714E37F, PH.base.pack) ;
        p30_rollspeed_SET((float)3.076291E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)2.548005E38F, PH.base.pack) ;
        p30_yaw_SET((float) -1.2797062E38F, PH.base.pack) ;
        p30_yawspeed_SET((float) -2.8723407E38F, PH.base.pack) ;
        p30_roll_SET((float)3.5725677E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float) -2.445654E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)3394159494L, PH.base.pack) ;
        p31_q4_SET((float) -1.8475425E38F, PH.base.pack) ;
        p31_q2_SET((float) -1.9525097E38F, PH.base.pack) ;
        p31_q1_SET((float)1.6750358E38F, PH.base.pack) ;
        p31_q3_SET((float) -3.6730554E37F, PH.base.pack) ;
        p31_yawspeed_SET((float) -2.0061258E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -1.5441449E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vy_SET((float)3.274229E38F, PH.base.pack) ;
        p32_z_SET((float) -1.3026281E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)4267658155L, PH.base.pack) ;
        p32_vz_SET((float) -1.5798528E38F, PH.base.pack) ;
        p32_y_SET((float)2.5387182E38F, PH.base.pack) ;
        p32_x_SET((float) -2.9666519E38F, PH.base.pack) ;
        p32_vx_SET((float) -2.7508205E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lon_SET((int32_t)1659738536, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)771197254, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)5944, PH.base.pack) ;
        p33_alt_SET((int32_t) -228417238, PH.base.pack) ;
        p33_lat_SET((int32_t)1230220408, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)25232, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -17582, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)10630, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)114445972L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan7_scaled_SET((int16_t)(int16_t)20735, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -28463, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2135895971L, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)19674, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)4812, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -6270, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)3081, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)26719, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -12804, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan7_raw_SET((uint16_t)(uint16_t)64990, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)30510, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)25779, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3803522764L, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)6705, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)59769, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)14576, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)9950, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)32572, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo10_raw_SET((uint16_t)(uint16_t)33800, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)27839, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)6389, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)22750, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)43080, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)30862, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)4056, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)59422, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)35480, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)57803, &PH) ;
        p36_time_usec_SET((uint32_t)432260142L, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)2671, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)48428, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)20662, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)41558, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)39318, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)22337, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)1312, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -2058, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)8727, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)7114, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_param2_SET((float) -1.8457725E38F, PH.base.pack) ;
        p39_param1_SET((float) -2.0027119E38F, PH.base.pack) ;
        p39_param3_SET((float) -2.3451873E38F, PH.base.pack) ;
        p39_z_SET((float) -1.2036875E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE, PH.base.pack) ;
        p39_x_SET((float)1.1464012E37F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p39_y_SET((float) -2.6960008E38F, PH.base.pack) ;
        p39_param4_SET((float)1.3071147E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)19356, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_system_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)42081, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)51695, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)6395, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)28404, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)35467, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -556956958, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1134207937723503564L, &PH) ;
        p48_latitude_SET((int32_t) -1341515638, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p48_longitude_SET((int32_t) -819759111, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t) -1155171582, PH.base.pack) ;
        p49_longitude_SET((int32_t) -653830860, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1432225484, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)8787628916244956455L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_index_SET((int16_t)(int16_t)30088, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        {
            char16_t* param_id = u"gnezfhqq";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value0_SET((float)1.7040735E38F, PH.base.pack) ;
        p50_param_value_min_SET((float)1.198202E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.8927333E37F, PH.base.pack) ;
        p50_scale_SET((float)1.6090551E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)30676, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p2y_SET((float) -1.1139006E38F, PH.base.pack) ;
        p54_p1z_SET((float) -3.6892042E37F, PH.base.pack) ;
        p54_p2x_SET((float)2.8880052E37F, PH.base.pack) ;
        p54_p2z_SET((float) -2.3951765E38F, PH.base.pack) ;
        p54_p1y_SET((float)1.455382E38F, PH.base.pack) ;
        p54_p1x_SET((float) -3.116855E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1z_SET((float) -1.1926291E38F, PH.base.pack) ;
        p55_p2y_SET((float)2.9326048E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p55_p2x_SET((float)4.1098756E37F, PH.base.pack) ;
        p55_p2z_SET((float)3.39955E37F, PH.base.pack) ;
        p55_p1x_SET((float) -1.7310753E38F, PH.base.pack) ;
        p55_p1y_SET((float)1.2029999E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)7562865263529368383L, PH.base.pack) ;
        {
            float covariance[] =  {3.2396778E38F, -1.4478285E38F, 1.9762573E38F, 1.332308E38F, 2.2706338E38F, -1.8749875E38F, 7.3721995E37F, 1.1377008E38F, 6.9072045E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float)3.2784479E38F, PH.base.pack) ;
        {
            float q[] =  {3.2074793E38F, -2.3531925E38F, 2.5557422E38F, -2.8241629E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)1.6708638E38F, PH.base.pack) ;
        p61_rollspeed_SET((float)1.0128617E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_pitch_SET((float)9.251857E37F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)22329, PH.base.pack) ;
        p62_aspd_error_SET((float) -2.9195517E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)15804, PH.base.pack) ;
        p62_nav_roll_SET((float) -2.1038132E38F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.702578E36F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)29911, PH.base.pack) ;
        p62_alt_error_SET((float) -2.7895337E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        {
            float covariance[] =  {-1.9469913E38F, 2.852813E38F, -1.3888434E38F, 2.0217123E38F, 2.9468333E38F, -5.7301037E37F, 2.0488558E38F, -3.8996517E37F, -1.3574205E38F, -2.468015E38F, -1.3809641E38F, -1.3139917E37F, -8.342899E37F, -3.2070122E38F, 2.124228E38F, -3.717357E37F, 2.85092E38F, -1.3201705E38F, -3.3298573E38F, 1.6268772E38F, -2.7246809E38F, 1.2375039E38F, -1.5727344E38F, 1.7154592E38F, 5.472361E37F, -5.8748466E37F, -2.0540556E38F, -9.159018E37F, -2.913951E38F, 2.8964798E38F, 8.3233477E37F, 2.5323623E38F, 1.7475367E38F, -6.916938E37F, -4.1466844E37F, -2.6789307E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vy_SET((float) -4.1794473E37F, PH.base.pack) ;
        p63_vx_SET((float) -1.852078E38F, PH.base.pack) ;
        p63_vz_SET((float)2.912355E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)2558070013190918344L, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)700664959, PH.base.pack) ;
        p63_lat_SET((int32_t)584554462, PH.base.pack) ;
        p63_alt_SET((int32_t) -748837603, PH.base.pack) ;
        p63_lon_SET((int32_t)1181305154, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)2054404724743728818L, PH.base.pack) ;
        p64_y_SET((float) -2.2289188E38F, PH.base.pack) ;
        p64_z_SET((float) -1.5939361E37F, PH.base.pack) ;
        {
            float covariance[] =  {-3.0773723E38F, -1.2690684E38F, 2.8567833E38F, -1.4099822E38F, -3.1868746E38F, -1.1810983E38F, 1.9182286E38F, -1.4963854E38F, -7.3238235E37F, -1.1640761E38F, -1.2470471E38F, 9.537331E37F, 1.6706842E38F, -1.7827688E38F, -1.4841388E38F, -3.3668453E38F, -2.2775927E38F, -2.1734598E38F, 2.2240587E38F, 3.3952303E38F, 2.2996848E37F, 1.489214E38F, -6.709353E37F, 2.7005937E38F, -2.4244392E38F, 1.0871049E38F, -2.0634496E38F, 2.3333677E38F, -1.7428245E38F, -3.1336134E38F, -1.4657602E38F, -2.6647018E38F, 1.8258312E38F, 3.318637E37F, 8.045426E37F, -2.4497208E38F, -1.451819E37F, -2.2961827E38F, 1.3258447E38F, -2.793597E38F, 2.493665E38F, -2.9957677E38F, -2.5904426E38F, -3.2676816E38F, -2.6064728E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vy_SET((float) -1.707929E38F, PH.base.pack) ;
        p64_x_SET((float) -9.096956E37F, PH.base.pack) ;
        p64_ax_SET((float) -2.182815E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p64_vz_SET((float) -2.9254437E37F, PH.base.pack) ;
        p64_ay_SET((float)2.2931887E38F, PH.base.pack) ;
        p64_vx_SET((float) -2.8438881E38F, PH.base.pack) ;
        p64_az_SET((float)7.9310626E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_time_boot_ms_SET((uint32_t)2701548636L, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)18533, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)58142, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)52202, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)32350, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)35625, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)14375, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)20893, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)37871, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)10374, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)22962, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)62251, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)63286, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)13284, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)30178, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)6917, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)61397, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)6863, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)15836, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_component_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)24002, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)50432, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_x_SET((int16_t)(int16_t) -31745, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -4026, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)13710, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)24430, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)9138, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan4_raw_SET((uint16_t)(uint16_t)56246, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)37805, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)41652, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)55365, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)55389, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)29838, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)8646, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)20268, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_z_SET((float)3.3361651E38F, PH.base.pack) ;
        p73_param3_SET((float)1.0163971E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p73_y_SET((int32_t)338822673, PH.base.pack) ;
        p73_param4_SET((float)2.3332196E38F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)28310, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p73_param1_SET((float)2.7546287E38F, PH.base.pack) ;
        p73_param2_SET((float) -2.5003702E38F, PH.base.pack) ;
        p73_x_SET((int32_t) -2050573992, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float) -8.864647E37F, PH.base.pack) ;
        p74_airspeed_SET((float)7.584404E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)31153, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -21823, PH.base.pack) ;
        p74_climb_SET((float) -2.750339E38F, PH.base.pack) ;
        p74_alt_SET((float) -9.744469E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p75_param2_SET((float)1.5594974E38F, PH.base.pack) ;
        p75_param1_SET((float)1.0255435E38F, PH.base.pack) ;
        p75_param4_SET((float)1.884784E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p75_y_SET((int32_t)1709683284, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p75_x_SET((int32_t) -101414060, PH.base.pack) ;
        p75_param3_SET((float)2.1283676E37F, PH.base.pack) ;
        p75_z_SET((float) -1.2674495E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float) -3.4279714E37F, PH.base.pack) ;
        p76_param1_SET((float)2.8360615E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p76_param3_SET((float)7.563703E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY, PH.base.pack) ;
        p76_param2_SET((float) -2.1955414E38F, PH.base.pack) ;
        p76_param7_SET((float) -2.0090325E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p76_param4_SET((float)1.2003641E37F, PH.base.pack) ;
        p76_param5_SET((float) -2.2489923E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_param2_SET((int32_t)1766749511, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)190, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)98, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)254, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_roll_SET((float)2.207206E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p81_thrust_SET((float)1.7286485E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)636563537L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p81_yaw_SET((float)3.3326314E37F, PH.base.pack) ;
        p81_pitch_SET((float) -9.399901E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_component_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p82_body_roll_rate_SET((float)2.4491525E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)1.0139677E38F, PH.base.pack) ;
        p82_thrust_SET((float) -2.8250301E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)3.2856846E38F, PH.base.pack) ;
        {
            float q[] =  {-2.888454E36F, 1.4167809E38F, 2.905799E38F, 3.079011E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_time_boot_ms_SET((uint32_t)1661009619L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_time_boot_ms_SET((uint32_t)2440290737L, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.4800156E38F, PH.base.pack) ;
        {
            float q[] =  {-2.351521E38F, 2.9463044E38F, -4.2016266E37F, 8.2916387E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -2.9802344E37F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)2.1207034E38F, PH.base.pack) ;
        p83_thrust_SET((float)2.0431921E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p84_vx_SET((float) -2.0643802E38F, PH.base.pack) ;
        p84_yaw_SET((float) -3.2909244E38F, PH.base.pack) ;
        p84_z_SET((float)2.0757416E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)7, PH.base.pack) ;
        p84_afx_SET((float)2.031474E38F, PH.base.pack) ;
        p84_x_SET((float)2.7720282E38F, PH.base.pack) ;
        p84_vy_SET((float) -2.5018452E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)509944675L, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p84_y_SET((float)1.1593487E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float)2.4671134E38F, PH.base.pack) ;
        p84_afz_SET((float)1.8637409E38F, PH.base.pack) ;
        p84_afy_SET((float) -2.2688325E38F, PH.base.pack) ;
        p84_vz_SET((float)2.679293E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_afy_SET((float) -3.0040129E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)6030, PH.base.pack) ;
        p86_yaw_SET((float) -6.092749E37F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1449078658, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1109651881, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)253135359L, PH.base.pack) ;
        p86_afx_SET((float) -2.5817278E38F, PH.base.pack) ;
        p86_afz_SET((float) -2.205653E38F, PH.base.pack) ;
        p86_alt_SET((float) -2.6298702E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.5726648E38F, PH.base.pack) ;
        p86_vy_SET((float) -7.7085123E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p86_vz_SET((float)3.0932861E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p86_vx_SET((float) -6.8669683E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_yaw_SET((float) -2.5778697E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)64669, PH.base.pack) ;
        p87_afy_SET((float) -4.269809E37F, PH.base.pack) ;
        p87_vx_SET((float) -2.683171E38F, PH.base.pack) ;
        p87_afx_SET((float) -3.1168528E38F, PH.base.pack) ;
        p87_alt_SET((float) -1.3466457E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -496316436, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)1276858005L, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -673726200, PH.base.pack) ;
        p87_vz_SET((float)1.7011022E37F, PH.base.pack) ;
        p87_afz_SET((float) -1.9693668E38F, PH.base.pack) ;
        p87_vy_SET((float)3.6093996E36F, PH.base.pack) ;
        p87_yaw_rate_SET((float)2.712872E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_z_SET((float) -1.7228075E38F, PH.base.pack) ;
        p89_yaw_SET((float) -1.5443548E37F, PH.base.pack) ;
        p89_roll_SET((float) -1.6072379E38F, PH.base.pack) ;
        p89_pitch_SET((float) -7.250178E36F, PH.base.pack) ;
        p89_y_SET((float)2.1710969E38F, PH.base.pack) ;
        p89_x_SET((float) -2.8398743E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)166670646L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_time_usec_SET((uint64_t)6742601611161197277L, PH.base.pack) ;
        p90_rollspeed_SET((float)1.4706485E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)20289, PH.base.pack) ;
        p90_alt_SET((int32_t)848984422, PH.base.pack) ;
        p90_roll_SET((float) -6.1198327E37F, PH.base.pack) ;
        p90_yaw_SET((float) -1.8274938E38F, PH.base.pack) ;
        p90_pitch_SET((float) -2.5410963E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)17407, PH.base.pack) ;
        p90_yawspeed_SET((float) -2.6965403E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)13864, PH.base.pack) ;
        p90_lat_SET((int32_t)386982471, PH.base.pack) ;
        p90_lon_SET((int32_t)2087364998, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)29112, PH.base.pack) ;
        p90_pitchspeed_SET((float) -2.7343378E37F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -12467, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -26121, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux4_SET((float) -2.006213E37F, PH.base.pack) ;
        p91_throttle_SET((float) -1.1705424E38F, PH.base.pack) ;
        p91_aux2_SET((float)2.5425743E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p91_aux1_SET((float)5.1834127E37F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)6152716310341722422L, PH.base.pack) ;
        p91_roll_ailerons_SET((float)3.222416E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -4.1075994E37F, PH.base.pack) ;
        p91_aux3_SET((float)2.6362042E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)3.1876516E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan1_raw_SET((uint16_t)(uint16_t)59272, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)60982, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)16106, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)55596, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)63278, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)13972, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)7887155782041998968L, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)14402, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)48996, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)33480, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)44031, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)7295, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)51203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
        {
            float controls[] =  {1.7202911E38F, -6.4353773E37F, 2.2451033E38F, 2.8113543E38F, 1.5791976E38F, -1.7981472E38F, 3.1023846E38F, -8.6496256E36F, -4.5189645E37F, 2.9182256E38F, -2.1115901E38F, -2.6974005E37F, 9.961425E37F, -1.8361274E38F, 3.0944495E37F, 2.1731582E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_time_usec_SET((uint64_t)4013659342510638650L, PH.base.pack) ;
        p93_flags_SET((uint64_t)4272882714694281413L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_x_SET((int16_t)(int16_t)28743, PH.base.pack) ;
        p100_flow_rate_y_SET((float)4.2548414E36F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.4418709E38F, &PH) ;
        p100_flow_comp_m_x_SET((float)3.2144477E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)4519168474515564870L, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)5262, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -1.3381496E38F, PH.base.pack) ;
        p100_ground_distance_SET((float)8.523298E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_z_SET((float)3.1320162E38F, PH.base.pack) ;
        p101_y_SET((float)7.129221E37F, PH.base.pack) ;
        p101_roll_SET((float) -2.9597111E37F, PH.base.pack) ;
        p101_pitch_SET((float) -3.2584918E38F, PH.base.pack) ;
        p101_yaw_SET((float) -2.1095026E37F, PH.base.pack) ;
        p101_x_SET((float)2.0241869E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)5639921305148834868L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float)1.3866295E38F, PH.base.pack) ;
        p102_x_SET((float)7.719659E37F, PH.base.pack) ;
        p102_roll_SET((float) -2.2368015E38F, PH.base.pack) ;
        p102_yaw_SET((float) -2.8435113E38F, PH.base.pack) ;
        p102_pitch_SET((float)1.1617526E38F, PH.base.pack) ;
        p102_z_SET((float) -2.506053E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)5791775399724101132L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -9.514893E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)7311028900765663440L, PH.base.pack) ;
        p103_y_SET((float)7.627558E37F, PH.base.pack) ;
        p103_x_SET((float)1.763141E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)2583946965858146363L, PH.base.pack) ;
        p104_z_SET((float)2.8741681E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.235618E38F, PH.base.pack) ;
        p104_x_SET((float) -2.5803016E38F, PH.base.pack) ;
        p104_y_SET((float) -2.093941E38F, PH.base.pack) ;
        p104_pitch_SET((float)1.5519846E38F, PH.base.pack) ;
        p104_roll_SET((float) -2.316775E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_xmag_SET((float) -2.8901403E38F, PH.base.pack) ;
        p105_xgyro_SET((float) -2.0656162E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)5849786615138987329L, PH.base.pack) ;
        p105_zgyro_SET((float)2.7720971E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -1.749136E38F, PH.base.pack) ;
        p105_ygyro_SET((float)7.087489E37F, PH.base.pack) ;
        p105_ymag_SET((float) -8.2054597E37F, PH.base.pack) ;
        p105_zmag_SET((float)7.4772695E37F, PH.base.pack) ;
        p105_yacc_SET((float) -2.836348E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.104046E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)49272, PH.base.pack) ;
        p105_zacc_SET((float) -3.0432918E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)3.002738E38F, PH.base.pack) ;
        p105_xacc_SET((float) -9.727169E36F, PH.base.pack) ;
        p105_temperature_SET((float) -2.0755592E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_usec_SET((uint64_t)6125659941458459298L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p106_integrated_x_SET((float)2.0730649E38F, PH.base.pack) ;
        p106_integrated_y_SET((float) -7.5729563E37F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)2.1277254E37F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p106_distance_SET((float)3.102812E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.2298501E37F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)23140, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)7.159597E37F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)2352695594L, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2337196831L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_ygyro_SET((float)1.8301944E38F, PH.base.pack) ;
        p107_ymag_SET((float) -2.8477276E38F, PH.base.pack) ;
        p107_xmag_SET((float) -3.180734E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float)3.1240728E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -3.405609E37F, PH.base.pack) ;
        p107_zacc_SET((float) -2.5276712E38F, PH.base.pack) ;
        p107_xacc_SET((float)2.4295E38F, PH.base.pack) ;
        p107_yacc_SET((float) -1.1197476E38F, PH.base.pack) ;
        p107_zmag_SET((float)3.1894165E37F, PH.base.pack) ;
        p107_zgyro_SET((float)9.687189E37F, PH.base.pack) ;
        p107_temperature_SET((float) -1.096365E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)3401699506L, PH.base.pack) ;
        p107_diff_pressure_SET((float) -2.6642144E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.1797562E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)1704027065617135820L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_lat_SET((float) -3.1256866E38F, PH.base.pack) ;
        p108_q1_SET((float)2.228905E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.1940415E38F, PH.base.pack) ;
        p108_q2_SET((float)2.4481154E38F, PH.base.pack) ;
        p108_lon_SET((float)1.3189123E38F, PH.base.pack) ;
        p108_q3_SET((float) -1.5919881E38F, PH.base.pack) ;
        p108_zacc_SET((float) -5.062194E37F, PH.base.pack) ;
        p108_vd_SET((float) -5.402685E37F, PH.base.pack) ;
        p108_roll_SET((float)9.768196E37F, PH.base.pack) ;
        p108_xacc_SET((float) -8.771818E37F, PH.base.pack) ;
        p108_ygyro_SET((float) -3.1466599E38F, PH.base.pack) ;
        p108_q4_SET((float) -1.0878806E38F, PH.base.pack) ;
        p108_alt_SET((float) -1.6319332E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)6.3173074E37F, PH.base.pack) ;
        p108_yaw_SET((float)1.8154183E37F, PH.base.pack) ;
        p108_vn_SET((float) -2.404912E37F, PH.base.pack) ;
        p108_ve_SET((float)2.9679954E38F, PH.base.pack) ;
        p108_yacc_SET((float) -1.0868063E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.6763646E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -7.199785E37F, PH.base.pack) ;
        p108_pitch_SET((float)3.1193642E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)24652, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)8917, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)239, (uint8_t)230, (uint8_t)49, (uint8_t)145, (uint8_t)7, (uint8_t)164, (uint8_t)242, (uint8_t)229, (uint8_t)210, (uint8_t)42, (uint8_t)137, (uint8_t)66, (uint8_t)254, (uint8_t)245, (uint8_t)173, (uint8_t)175, (uint8_t)137, (uint8_t)217, (uint8_t)54, (uint8_t)37, (uint8_t)255, (uint8_t)172, (uint8_t)92, (uint8_t)196, (uint8_t)94, (uint8_t)155, (uint8_t)11, (uint8_t)124, (uint8_t)192, (uint8_t)32, (uint8_t)4, (uint8_t)82, (uint8_t)113, (uint8_t)10, (uint8_t)10, (uint8_t)168, (uint8_t)186, (uint8_t)10, (uint8_t)249, (uint8_t)103, (uint8_t)190, (uint8_t)37, (uint8_t)87, (uint8_t)255, (uint8_t)221, (uint8_t)160, (uint8_t)240, (uint8_t)121, (uint8_t)161, (uint8_t)204, (uint8_t)137, (uint8_t)53, (uint8_t)208, (uint8_t)188, (uint8_t)13, (uint8_t)45, (uint8_t)18, (uint8_t)96, (uint8_t)240, (uint8_t)44, (uint8_t)67, (uint8_t)46, (uint8_t)20, (uint8_t)94, (uint8_t)103, (uint8_t)204, (uint8_t)94, (uint8_t)113, (uint8_t)158, (uint8_t)90, (uint8_t)255, (uint8_t)8, (uint8_t)31, (uint8_t)124, (uint8_t)53, (uint8_t)161, (uint8_t)63, (uint8_t)42, (uint8_t)16, (uint8_t)248, (uint8_t)90, (uint8_t)214, (uint8_t)64, (uint8_t)0, (uint8_t)174, (uint8_t)42, (uint8_t)149, (uint8_t)217, (uint8_t)242, (uint8_t)12, (uint8_t)247, (uint8_t)61, (uint8_t)250, (uint8_t)185, (uint8_t)60, (uint8_t)2, (uint8_t)182, (uint8_t)101, (uint8_t)193, (uint8_t)10, (uint8_t)177, (uint8_t)120, (uint8_t)138, (uint8_t)129, (uint8_t)92, (uint8_t)30, (uint8_t)244, (uint8_t)166, (uint8_t)16, (uint8_t)151, (uint8_t)162, (uint8_t)30, (uint8_t)61, (uint8_t)103, (uint8_t)130, (uint8_t)92, (uint8_t)83, (uint8_t)165, (uint8_t)102, (uint8_t)141, (uint8_t)248, (uint8_t)63, (uint8_t)153, (uint8_t)170, (uint8_t)74, (uint8_t)134, (uint8_t)134, (uint8_t)62, (uint8_t)197, (uint8_t)118, (uint8_t)154, (uint8_t)254, (uint8_t)170, (uint8_t)231, (uint8_t)219, (uint8_t)77, (uint8_t)194, (uint8_t)202, (uint8_t)22, (uint8_t)200, (uint8_t)220, (uint8_t)169, (uint8_t)150, (uint8_t)221, (uint8_t)251, (uint8_t)159, (uint8_t)151, (uint8_t)23, (uint8_t)220, (uint8_t)174, (uint8_t)0, (uint8_t)37, (uint8_t)186, (uint8_t)164, (uint8_t)108, (uint8_t)132, (uint8_t)18, (uint8_t)216, (uint8_t)163, (uint8_t)26, (uint8_t)93, (uint8_t)72, (uint8_t)19, (uint8_t)227, (uint8_t)133, (uint8_t)215, (uint8_t)186, (uint8_t)105, (uint8_t)198, (uint8_t)20, (uint8_t)204, (uint8_t)49, (uint8_t)129, (uint8_t)156, (uint8_t)149, (uint8_t)181, (uint8_t)9, (uint8_t)97, (uint8_t)191, (uint8_t)17, (uint8_t)56, (uint8_t)101, (uint8_t)71, (uint8_t)2, (uint8_t)163, (uint8_t)152, (uint8_t)118, (uint8_t)142, (uint8_t)139, (uint8_t)145, (uint8_t)131, (uint8_t)61, (uint8_t)152, (uint8_t)112, (uint8_t)214, (uint8_t)138, (uint8_t)238, (uint8_t)244, (uint8_t)211, (uint8_t)39, (uint8_t)101, (uint8_t)206, (uint8_t)2, (uint8_t)177, (uint8_t)203, (uint8_t)148, (uint8_t)216, (uint8_t)115, (uint8_t)156, (uint8_t)171, (uint8_t)185, (uint8_t)153, (uint8_t)195, (uint8_t)37, (uint8_t)122, (uint8_t)108, (uint8_t)38, (uint8_t)30, (uint8_t)36, (uint8_t)64, (uint8_t)71, (uint8_t)57, (uint8_t)197, (uint8_t)73, (uint8_t)92, (uint8_t)252, (uint8_t)20, (uint8_t)55, (uint8_t)86, (uint8_t)122, (uint8_t)222, (uint8_t)95, (uint8_t)17, (uint8_t)203, (uint8_t)148, (uint8_t)172, (uint8_t)105, (uint8_t)68, (uint8_t)47, (uint8_t)168, (uint8_t)23, (uint8_t)198, (uint8_t)193, (uint8_t)157, (uint8_t)86, (uint8_t)247, (uint8_t)35, (uint8_t)189, (uint8_t)196, (uint8_t)51, (uint8_t)100};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)6954607405756756048L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -4271923815245869356L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)2980162530L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)267142085484793790L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_epv_SET((uint16_t)(uint16_t)22467, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)18281, PH.base.pack) ;
        p113_lon_SET((int32_t)191708389, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)49264, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)7460, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)30551, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)7268399503943060421L, PH.base.pack) ;
        p113_alt_SET((int32_t) -939279853, PH.base.pack) ;
        p113_lat_SET((int32_t)1307144115, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)4809, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)45001, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_ygyro_SET((float)3.3694082E38F, PH.base.pack) ;
        p114_integrated_x_SET((float)8.3874873E37F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2956013886L, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p114_distance_SET((float) -2.8587728E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)2.2735658E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -18973, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -2.2611015E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)4325286303404438081L, PH.base.pack) ;
        p114_integrated_y_SET((float)1.0757223E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)3067982059L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        {
            float attitude_quaternion[] =  {-3.0971795E38F, -1.2068994E38F, -2.6729137E38F, -1.8779921E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_lon_SET((int32_t)1194102551, PH.base.pack) ;
        p115_pitchspeed_SET((float) -3.375693E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)20306, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)44916, PH.base.pack) ;
        p115_lat_SET((int32_t)878771475, PH.base.pack) ;
        p115_yawspeed_SET((float) -8.204661E37F, PH.base.pack) ;
        p115_alt_SET((int32_t) -1052604667, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)8151, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)45662, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)7020354322673331638L, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -27598, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)370, PH.base.pack) ;
        p115_rollspeed_SET((float)1.049093E38F, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)4892, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -17930, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_ygyro_SET((int16_t)(int16_t) -22203, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -6685, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)5418, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -18595, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)6917, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2301176427L, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)28857, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)16932, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -15366, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -2760, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)466, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)31482, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_time_utc_SET((uint32_t)2604078058L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)57905, PH.base.pack) ;
        p118_size_SET((uint32_t)3666288349L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)44327, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)39380, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)39159, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p119_ofs_SET((uint32_t)487743224L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p119_count_SET((uint32_t)2797930403L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)38777, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)32, (uint8_t)51, (uint8_t)28, (uint8_t)73, (uint8_t)245, (uint8_t)176, (uint8_t)98, (uint8_t)233, (uint8_t)137, (uint8_t)113, (uint8_t)157, (uint8_t)88, (uint8_t)42, (uint8_t)105, (uint8_t)59, (uint8_t)49, (uint8_t)238, (uint8_t)91, (uint8_t)160, (uint8_t)202, (uint8_t)72, (uint8_t)218, (uint8_t)136, (uint8_t)237, (uint8_t)29, (uint8_t)248, (uint8_t)91, (uint8_t)133, (uint8_t)116, (uint8_t)39, (uint8_t)71, (uint8_t)102, (uint8_t)76, (uint8_t)143, (uint8_t)245, (uint8_t)136, (uint8_t)153, (uint8_t)255, (uint8_t)8, (uint8_t)246, (uint8_t)8, (uint8_t)120, (uint8_t)123, (uint8_t)72, (uint8_t)179, (uint8_t)26, (uint8_t)1, (uint8_t)97, (uint8_t)98, (uint8_t)124, (uint8_t)234, (uint8_t)160, (uint8_t)156, (uint8_t)204, (uint8_t)254, (uint8_t)3, (uint8_t)238, (uint8_t)17, (uint8_t)15, (uint8_t)194, (uint8_t)193, (uint8_t)249, (uint8_t)211, (uint8_t)41, (uint8_t)60, (uint8_t)85, (uint8_t)133, (uint8_t)69, (uint8_t)123, (uint8_t)138, (uint8_t)173, (uint8_t)96, (uint8_t)173, (uint8_t)63, (uint8_t)206, (uint8_t)45, (uint8_t)128, (uint8_t)14, (uint8_t)10, (uint8_t)211, (uint8_t)182, (uint8_t)50, (uint8_t)172, (uint8_t)83, (uint8_t)176, (uint8_t)113, (uint8_t)208, (uint8_t)120, (uint8_t)83, (uint8_t)97};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)3333682693L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)116, (uint8_t)13, (uint8_t)33, (uint8_t)242, (uint8_t)172, (uint8_t)177, (uint8_t)33, (uint8_t)213, (uint8_t)147, (uint8_t)2, (uint8_t)187, (uint8_t)208, (uint8_t)127, (uint8_t)251, (uint8_t)128, (uint8_t)104, (uint8_t)51, (uint8_t)164, (uint8_t)88, (uint8_t)187, (uint8_t)37, (uint8_t)23, (uint8_t)110, (uint8_t)107, (uint8_t)185, (uint8_t)141, (uint8_t)72, (uint8_t)252, (uint8_t)195, (uint8_t)1, (uint8_t)194, (uint8_t)158, (uint8_t)144, (uint8_t)123, (uint8_t)18, (uint8_t)152, (uint8_t)116, (uint8_t)134, (uint8_t)44, (uint8_t)33, (uint8_t)143, (uint8_t)89, (uint8_t)243, (uint8_t)160, (uint8_t)74, (uint8_t)197, (uint8_t)186, (uint8_t)77, (uint8_t)235, (uint8_t)183, (uint8_t)214, (uint8_t)164, (uint8_t)133, (uint8_t)115, (uint8_t)26, (uint8_t)114, (uint8_t)55, (uint8_t)231, (uint8_t)157, (uint8_t)249, (uint8_t)193, (uint8_t)113, (uint8_t)128, (uint8_t)247, (uint8_t)161, (uint8_t)48, (uint8_t)60, (uint8_t)210, (uint8_t)119, (uint8_t)146, (uint8_t)199, (uint8_t)29, (uint8_t)182, (uint8_t)122, (uint8_t)237, (uint8_t)81, (uint8_t)183, (uint8_t)248, (uint8_t)184, (uint8_t)164, (uint8_t)39, (uint8_t)62, (uint8_t)21, (uint8_t)152, (uint8_t)1, (uint8_t)245, (uint8_t)125, (uint8_t)228, (uint8_t)187, (uint8_t)144, (uint8_t)200, (uint8_t)71, (uint8_t)32, (uint8_t)149, (uint8_t)251, (uint8_t)249, (uint8_t)142, (uint8_t)213, (uint8_t)114, (uint8_t)217, (uint8_t)192, (uint8_t)161, (uint8_t)141, (uint8_t)70, (uint8_t)98, (uint8_t)106, (uint8_t)151, (uint8_t)238, (uint8_t)173, (uint8_t)190};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_lon_SET((int32_t) -1496438604, PH.base.pack) ;
        p124_lat_SET((int32_t)2129404312, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)58966, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)9091, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1423325231L, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)49316, PH.base.pack) ;
        p124_alt_SET((int32_t)1313475703, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)48062, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)4997264006353989599L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)24440, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)52119, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)136, (uint8_t)5, (uint8_t)218, (uint8_t)114, (uint8_t)21, (uint8_t)94, (uint8_t)34, (uint8_t)43, (uint8_t)127, (uint8_t)166, (uint8_t)64, (uint8_t)97, (uint8_t)68, (uint8_t)62, (uint8_t)150, (uint8_t)220, (uint8_t)227, (uint8_t)93, (uint8_t)98, (uint8_t)90, (uint8_t)248, (uint8_t)158, (uint8_t)195, (uint8_t)225, (uint8_t)252, (uint8_t)204, (uint8_t)84, (uint8_t)246, (uint8_t)103, (uint8_t)225, (uint8_t)244, (uint8_t)181, (uint8_t)174, (uint8_t)138, (uint8_t)250, (uint8_t)245, (uint8_t)127, (uint8_t)53, (uint8_t)153, (uint8_t)24, (uint8_t)226, (uint8_t)130, (uint8_t)203, (uint8_t)255, (uint8_t)34, (uint8_t)38, (uint8_t)208, (uint8_t)82, (uint8_t)80, (uint8_t)180, (uint8_t)192, (uint8_t)118, (uint8_t)39, (uint8_t)218, (uint8_t)247, (uint8_t)71, (uint8_t)207, (uint8_t)86, (uint8_t)184, (uint8_t)126, (uint8_t)4, (uint8_t)13, (uint8_t)127, (uint8_t)42, (uint8_t)82, (uint8_t)156, (uint8_t)60, (uint8_t)167, (uint8_t)111, (uint8_t)92};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_count_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)3144, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1887991453L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)3994438851L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)250639321, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1413042033, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)47196, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)2035252008, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1782721771L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)470290111L, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -952453813, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_a_mm_SET((int32_t)1552286010, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)10399, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)939483234, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)1661229394, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1827633840, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)1698350476L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)1973632655L, PH.base.pack) ;
        p128_tow_SET((uint32_t)2692707107L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_xmag_SET((int16_t)(int16_t) -20860, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -4552, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)2505977931L, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)9467, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)27364, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)29548, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -22916, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -14455, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)19138, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)4502, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_packets_SET((uint16_t)(uint16_t)27529, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)11363, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)22166, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p130_size_SET((uint32_t)2691718608L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)109, (uint8_t)181, (uint8_t)129, (uint8_t)236, (uint8_t)48, (uint8_t)108, (uint8_t)222, (uint8_t)198, (uint8_t)238, (uint8_t)71, (uint8_t)71, (uint8_t)13, (uint8_t)187, (uint8_t)165, (uint8_t)110, (uint8_t)119, (uint8_t)224, (uint8_t)187, (uint8_t)185, (uint8_t)228, (uint8_t)114, (uint8_t)128, (uint8_t)89, (uint8_t)28, (uint8_t)235, (uint8_t)202, (uint8_t)110, (uint8_t)148, (uint8_t)66, (uint8_t)81, (uint8_t)218, (uint8_t)66, (uint8_t)223, (uint8_t)175, (uint8_t)229, (uint8_t)106, (uint8_t)5, (uint8_t)96, (uint8_t)49, (uint8_t)147, (uint8_t)93, (uint8_t)162, (uint8_t)240, (uint8_t)116, (uint8_t)163, (uint8_t)72, (uint8_t)235, (uint8_t)208, (uint8_t)117, (uint8_t)193, (uint8_t)134, (uint8_t)198, (uint8_t)130, (uint8_t)255, (uint8_t)142, (uint8_t)144, (uint8_t)8, (uint8_t)172, (uint8_t)6, (uint8_t)111, (uint8_t)70, (uint8_t)49, (uint8_t)249, (uint8_t)17, (uint8_t)158, (uint8_t)134, (uint8_t)87, (uint8_t)155, (uint8_t)235, (uint8_t)184, (uint8_t)84, (uint8_t)169, (uint8_t)90, (uint8_t)209, (uint8_t)209, (uint8_t)160, (uint8_t)43, (uint8_t)177, (uint8_t)171, (uint8_t)235, (uint8_t)35, (uint8_t)88, (uint8_t)20, (uint8_t)137, (uint8_t)55, (uint8_t)208, (uint8_t)139, (uint8_t)113, (uint8_t)200, (uint8_t)189, (uint8_t)46, (uint8_t)251, (uint8_t)182, (uint8_t)82, (uint8_t)123, (uint8_t)241, (uint8_t)123, (uint8_t)255, (uint8_t)33, (uint8_t)54, (uint8_t)166, (uint8_t)89, (uint8_t)109, (uint8_t)37, (uint8_t)153, (uint8_t)50, (uint8_t)144, (uint8_t)158, (uint8_t)98, (uint8_t)180, (uint8_t)78, (uint8_t)210, (uint8_t)42, (uint8_t)221, (uint8_t)74, (uint8_t)222, (uint8_t)44, (uint8_t)92, (uint8_t)175, (uint8_t)219, (uint8_t)63, (uint8_t)242, (uint8_t)148, (uint8_t)10, (uint8_t)251, (uint8_t)14, (uint8_t)159, (uint8_t)94, (uint8_t)130, (uint8_t)186, (uint8_t)144, (uint8_t)156, (uint8_t)238, (uint8_t)173, (uint8_t)34, (uint8_t)161, (uint8_t)201, (uint8_t)36, (uint8_t)104, (uint8_t)230, (uint8_t)7, (uint8_t)101, (uint8_t)198, (uint8_t)185, (uint8_t)213, (uint8_t)242, (uint8_t)251, (uint8_t)229, (uint8_t)234, (uint8_t)69, (uint8_t)62, (uint8_t)240, (uint8_t)34, (uint8_t)22, (uint8_t)48, (uint8_t)145, (uint8_t)145, (uint8_t)174, (uint8_t)104, (uint8_t)142, (uint8_t)24, (uint8_t)240, (uint8_t)71, (uint8_t)136, (uint8_t)225, (uint8_t)55, (uint8_t)124, (uint8_t)6, (uint8_t)254, (uint8_t)153, (uint8_t)235, (uint8_t)214, (uint8_t)223, (uint8_t)58, (uint8_t)88, (uint8_t)118, (uint8_t)131, (uint8_t)220, (uint8_t)183, (uint8_t)18, (uint8_t)65, (uint8_t)199, (uint8_t)164, (uint8_t)157, (uint8_t)49, (uint8_t)42, (uint8_t)111, (uint8_t)215, (uint8_t)227, (uint8_t)76, (uint8_t)34, (uint8_t)249, (uint8_t)182, (uint8_t)255, (uint8_t)211, (uint8_t)6, (uint8_t)62, (uint8_t)175, (uint8_t)250, (uint8_t)149, (uint8_t)166, (uint8_t)69, (uint8_t)218, (uint8_t)16, (uint8_t)84, (uint8_t)175, (uint8_t)92, (uint8_t)35, (uint8_t)39, (uint8_t)94, (uint8_t)198, (uint8_t)137, (uint8_t)34, (uint8_t)202, (uint8_t)102, (uint8_t)190, (uint8_t)3, (uint8_t)123, (uint8_t)233, (uint8_t)6, (uint8_t)88, (uint8_t)57, (uint8_t)98, (uint8_t)112, (uint8_t)154, (uint8_t)95, (uint8_t)154, (uint8_t)25, (uint8_t)204, (uint8_t)217, (uint8_t)183, (uint8_t)165, (uint8_t)34, (uint8_t)10, (uint8_t)140, (uint8_t)13, (uint8_t)199, (uint8_t)52, (uint8_t)199, (uint8_t)103, (uint8_t)105, (uint8_t)3, (uint8_t)178, (uint8_t)196, (uint8_t)126, (uint8_t)3, (uint8_t)97, (uint8_t)57, (uint8_t)171, (uint8_t)101, (uint8_t)173, (uint8_t)101, (uint8_t)35};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)25748, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_max_distance_SET((uint16_t)(uint16_t)2299, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)52715, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2842064883L, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)32426, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)1175738442202476441L, PH.base.pack) ;
        p133_lon_SET((int32_t)1877846739, PH.base.pack) ;
        p133_lat_SET((int32_t) -1680792476, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)13429, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t)1104242771, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p134_lon_SET((int32_t) -3716587, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)46267, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)1199, (int16_t)20481, (int16_t)4272, (int16_t) -22772, (int16_t) -13635, (int16_t) -9386, (int16_t) -12757, (int16_t)19427, (int16_t)7735, (int16_t)1018, (int16_t) -15474, (int16_t)27771, (int16_t) -1486, (int16_t) -3322, (int16_t) -10197, (int16_t)13074};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t)870113501, PH.base.pack) ;
        p135_lat_SET((int32_t) -782270563, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_lon_SET((int32_t)1099994742, PH.base.pack) ;
        p136_terrain_height_SET((float)1.9848585E38F, PH.base.pack) ;
        p136_current_height_SET((float)1.5118206E37F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)62198, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)42629, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)47899, PH.base.pack) ;
        p136_lat_SET((int32_t) -1179153348, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t)14148, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)625015381L, PH.base.pack) ;
        p137_press_diff_SET((float) -5.11746E37F, PH.base.pack) ;
        p137_press_abs_SET((float) -2.7006213E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)8863411341774034002L, PH.base.pack) ;
        p138_z_SET((float)9.982675E37F, PH.base.pack) ;
        p138_y_SET((float) -2.3964334E38F, PH.base.pack) ;
        {
            float q[] =  {4.266117E37F, 2.7252516E38F, -3.2727635E38F, -1.5981577E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)5.086798E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_time_usec_SET((uint64_t)1069360279198331310L, PH.base.pack) ;
        {
            float controls[] =  {-2.488098E38F, 2.444599E38F, 5.283872E37F, 9.4195174E36F, -1.906638E38F, -1.4362858E38F, -1.4689941E38F, 2.8238375E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_group_mlx_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        {
            float controls[] =  {-2.9663774E38F, 2.516918E38F, 2.206229E38F, -3.0407508E37F, -6.315314E36F, -7.1270673E37F, -2.9817274E38F, -1.2165541E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)7918130278400697566L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_monotonic_SET((float)2.7685378E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.1745619E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)6905251012933281502L, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -9.548136E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.2497905E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)7.978406E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)3.1540402E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)172, (uint8_t)209, (uint8_t)242, (uint8_t)112, (uint8_t)118, (uint8_t)139, (uint8_t)214, (uint8_t)215, (uint8_t)208, (uint8_t)83, (uint8_t)103, (uint8_t)238, (uint8_t)170, (uint8_t)1, (uint8_t)116, (uint8_t)236, (uint8_t)176, (uint8_t)242, (uint8_t)195, (uint8_t)207, (uint8_t)87, (uint8_t)105, (uint8_t)147, (uint8_t)100, (uint8_t)128, (uint8_t)80, (uint8_t)250, (uint8_t)59, (uint8_t)51, (uint8_t)173, (uint8_t)98, (uint8_t)70, (uint8_t)114, (uint8_t)20, (uint8_t)137, (uint8_t)217, (uint8_t)80, (uint8_t)182, (uint8_t)15, (uint8_t)57, (uint8_t)199, (uint8_t)6, (uint8_t)167, (uint8_t)3, (uint8_t)10, (uint8_t)127, (uint8_t)47, (uint8_t)217, (uint8_t)148, (uint8_t)55, (uint8_t)123, (uint8_t)70, (uint8_t)149, (uint8_t)213, (uint8_t)44, (uint8_t)4, (uint8_t)164, (uint8_t)193, (uint8_t)214, (uint8_t)39, (uint8_t)71, (uint8_t)210, (uint8_t)125, (uint8_t)131, (uint8_t)235, (uint8_t)220, (uint8_t)38, (uint8_t)163, (uint8_t)16, (uint8_t)249, (uint8_t)117, (uint8_t)169, (uint8_t)43, (uint8_t)95, (uint8_t)60, (uint8_t)66, (uint8_t)238, (uint8_t)221, (uint8_t)82, (uint8_t)208, (uint8_t)11, (uint8_t)131, (uint8_t)166, (uint8_t)47, (uint8_t)163, (uint8_t)214, (uint8_t)211, (uint8_t)56, (uint8_t)178, (uint8_t)184, (uint8_t)244, (uint8_t)229, (uint8_t)203, (uint8_t)251, (uint8_t)73, (uint8_t)46, (uint8_t)71, (uint8_t)86, (uint8_t)252, (uint8_t)26, (uint8_t)148, (uint8_t)224, (uint8_t)124, (uint8_t)122, (uint8_t)146, (uint8_t)221, (uint8_t)180, (uint8_t)176, (uint8_t)55, (uint8_t)39, (uint8_t)214, (uint8_t)39, (uint8_t)45, (uint8_t)31, (uint8_t)115, (uint8_t)198, (uint8_t)242, (uint8_t)36, (uint8_t)110, (uint8_t)159};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        {
            uint8_t uri[] =  {(uint8_t)76, (uint8_t)96, (uint8_t)62, (uint8_t)145, (uint8_t)128, (uint8_t)116, (uint8_t)142, (uint8_t)79, (uint8_t)170, (uint8_t)19, (uint8_t)131, (uint8_t)50, (uint8_t)125, (uint8_t)68, (uint8_t)184, (uint8_t)198, (uint8_t)252, (uint8_t)107, (uint8_t)84, (uint8_t)27, (uint8_t)134, (uint8_t)196, (uint8_t)40, (uint8_t)61, (uint8_t)73, (uint8_t)51, (uint8_t)3, (uint8_t)183, (uint8_t)171, (uint8_t)228, (uint8_t)195, (uint8_t)193, (uint8_t)111, (uint8_t)210, (uint8_t)30, (uint8_t)223, (uint8_t)174, (uint8_t)228, (uint8_t)100, (uint8_t)247, (uint8_t)73, (uint8_t)116, (uint8_t)136, (uint8_t)22, (uint8_t)192, (uint8_t)10, (uint8_t)190, (uint8_t)65, (uint8_t)172, (uint8_t)81, (uint8_t)206, (uint8_t)165, (uint8_t)33, (uint8_t)170, (uint8_t)244, (uint8_t)159, (uint8_t)52, (uint8_t)106, (uint8_t)55, (uint8_t)107, (uint8_t)220, (uint8_t)195, (uint8_t)211, (uint8_t)64, (uint8_t)167, (uint8_t)198, (uint8_t)14, (uint8_t)125, (uint8_t)104, (uint8_t)244, (uint8_t)146, (uint8_t)119, (uint8_t)81, (uint8_t)3, (uint8_t)195, (uint8_t)176, (uint8_t)29, (uint8_t)13, (uint8_t)46, (uint8_t)16, (uint8_t)175, (uint8_t)209, (uint8_t)215, (uint8_t)57, (uint8_t)7, (uint8_t)28, (uint8_t)27, (uint8_t)146, (uint8_t)156, (uint8_t)33, (uint8_t)239, (uint8_t)14, (uint8_t)123, (uint8_t)40, (uint8_t)201, (uint8_t)240, (uint8_t)91, (uint8_t)59, (uint8_t)57, (uint8_t)235, (uint8_t)15, (uint8_t)254, (uint8_t)111, (uint8_t)150, (uint8_t)192, (uint8_t)213, (uint8_t)126, (uint8_t)224, (uint8_t)248, (uint8_t)90, (uint8_t)231, (uint8_t)192, (uint8_t)158, (uint8_t)227, (uint8_t)135, (uint8_t)202, (uint8_t)137, (uint8_t)3, (uint8_t)211, (uint8_t)1};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t)14901, PH.base.pack) ;
        p143_press_diff_SET((float) -7.645716E37F, PH.base.pack) ;
        p143_press_abs_SET((float)2.9312181E37F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)2289145068L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float attitude_q[] =  {-2.0183509E38F, 2.1867902E38F, 2.4532112E38F, 2.947612E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {1.6618977E38F, -6.6025085E37F, 2.3967916E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)41194787, PH.base.pack) ;
        {
            float vel[] =  {-1.3635044E37F, -2.111124E38F, -3.0626132E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -526236837, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)6890959893554910070L, PH.base.pack) ;
        p144_alt_SET((float) -2.2856869E37F, PH.base.pack) ;
        {
            float acc[] =  {-2.4290118E38F, -1.8876088E38F, 2.9783577E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)4101833158502450362L, PH.base.pack) ;
        {
            float position_cov[] =  {3.1859015E38F, 1.0463319E38F, -4.615008E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_acc_SET((float) -1.9393036E38F, PH.base.pack) ;
        p146_y_pos_SET((float)1.6672912E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.2672485E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.5792956E38F, PH.base.pack) ;
        p146_x_pos_SET((float)1.0029275E37F, PH.base.pack) ;
        p146_y_vel_SET((float)1.0919504E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-1.04631E38F, 3.0032764E38F, -1.5624978E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float) -5.265567E36F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)7751694203198306303L, PH.base.pack) ;
        {
            float pos_variance[] =  {1.743435E38F, 1.8560248E38F, -1.9426107E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float) -2.2696787E38F, PH.base.pack) ;
        {
            float q[] =  {-2.0648306E38F, -2.5839072E38F, -2.3972295E38F, -2.7311654E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float)2.6961792E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -1.6312138E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)2.067521E38F, PH.base.pack) ;
        p146_x_vel_SET((float)7.0129266E37F, PH.base.pack) ;
        p146_z_pos_SET((float)1.7579904E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_remaining_SET((int8_t)(int8_t) -3, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)61430, (uint16_t)64356, (uint16_t)11150, (uint16_t)46203, (uint16_t)12615, (uint16_t)20543, (uint16_t)10671, (uint16_t)2463, (uint16_t)64724, (uint16_t)53278};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -961453572, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1927950203, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)24962, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -8846, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_board_version_SET((uint32_t)3766229091L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)41120, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)60, (uint8_t)1, (uint8_t)216, (uint8_t)238, (uint8_t)229, (uint8_t)160, (uint8_t)153, (uint8_t)185};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)9156423010208491189L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3778533596L, PH.base.pack) ;
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)47616, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)73, (uint8_t)79, (uint8_t)158, (uint8_t)33, (uint8_t)208, (uint8_t)155, (uint8_t)106, (uint8_t)253};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)208, (uint8_t)244, (uint8_t)65, (uint8_t)248, (uint8_t)224, (uint8_t)182, (uint8_t)63, (uint8_t)67, (uint8_t)109, (uint8_t)105, (uint8_t)83, (uint8_t)252, (uint8_t)130, (uint8_t)28, (uint8_t)180, (uint8_t)196, (uint8_t)237, (uint8_t)104};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_middleware_sw_version_SET((uint32_t)3982303932L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)157, (uint8_t)223, (uint8_t)249, (uint8_t)183, (uint8_t)32, (uint8_t)251, (uint8_t)195, (uint8_t)148};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)1050163185L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_position_valid_SET((uint8_t)(uint8_t)135, &PH) ;
        p149_angle_x_SET((float)2.9094518E37F, PH.base.pack) ;
        p149_size_y_SET((float)4.6096294E37F, PH.base.pack) ;
        p149_size_x_SET((float) -2.125828E38F, PH.base.pack) ;
        {
            float q[] =  {3.1419189E38F, 5.5303326E37F, 1.2348504E37F, 3.1526268E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p149_z_SET((float) -3.6774815E37F, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p149_angle_y_SET((float) -1.646775E38F, PH.base.pack) ;
        p149_y_SET((float)2.429335E37F, &PH) ;
        p149_distance_SET((float)3.2421403E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)8986449325971131466L, PH.base.pack) ;
        p149_x_SET((float) -1.3125192E38F, &PH) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_FILTER_BIAS_220(), &PH);
        p220_gyro_1_SET((float) -3.1352808E38F, PH.base.pack) ;
        p220_gyro_2_SET((float)1.130906E38F, PH.base.pack) ;
        p220_usec_SET((uint64_t)5324030965785478683L, PH.base.pack) ;
        p220_accel_0_SET((float)1.4097145E38F, PH.base.pack) ;
        p220_gyro_0_SET((float) -1.1491147E37F, PH.base.pack) ;
        p220_accel_2_SET((float) -3.3129923E38F, PH.base.pack) ;
        p220_accel_1_SET((float)1.6166497E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_FILTER_BIAS_220(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_CALIBRATION_221(), &PH);
        {
            uint16_t gyro[] =  {(uint16_t)20665, (uint16_t)34904};
            p221_gyro_SET(&gyro, 0, PH.base.pack) ;
        }
        {
            uint16_t elevator[] =  {(uint16_t)33819, (uint16_t)35773, (uint16_t)11830};
            p221_elevator_SET(&elevator, 0, PH.base.pack) ;
        }
        {
            uint16_t throttle[] =  {(uint16_t)3196, (uint16_t)28507, (uint16_t)64175, (uint16_t)30191, (uint16_t)55358};
            p221_throttle_SET(&throttle, 0, PH.base.pack) ;
        }
        {
            uint16_t pitch[] =  {(uint16_t)20908, (uint16_t)42337, (uint16_t)39982, (uint16_t)33878, (uint16_t)32016};
            p221_pitch_SET(&pitch, 0, PH.base.pack) ;
        }
        {
            uint16_t aileron[] =  {(uint16_t)2477, (uint16_t)57981, (uint16_t)61712};
            p221_aileron_SET(&aileron, 0, PH.base.pack) ;
        }
        {
            uint16_t rudder[] =  {(uint16_t)30293, (uint16_t)12962, (uint16_t)18473};
            p221_rudder_SET(&rudder, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RADIO_CALIBRATION_221(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UALBERTA_SYS_STATUS_222(), &PH);
        p222_nav_mode_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p222_mode_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p222_pilot_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UALBERTA_SYS_STATUS_222(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_tas_ratio_SET((float) -6.5162616E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.3639668E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)2.7015105E37F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
        p230_hagl_ratio_SET((float)2.1463241E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -6.9030887E37F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.7870049E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)1398132538116136273L, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -7.8235846E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.5975047E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float) -3.5234764E37F, PH.base.pack) ;
        p231_wind_y_SET((float)1.766822E38F, PH.base.pack) ;
        p231_var_vert_SET((float)2.5889583E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.0813239E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -2.1351173E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.5244966E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)9196155483526045920L, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.24497E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)1.7079896E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_vn_SET((float) -8.930495E37F, PH.base.pack) ;
        p232_vdop_SET((float)2.4216615E38F, PH.base.pack) ;
        p232_ve_SET((float)1.4887388E38F, PH.base.pack) ;
        p232_alt_SET((float) -2.9249888E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -3.0532574E38F, PH.base.pack) ;
        p232_hdop_SET((float)1.2711774E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)458902237110125009L, PH.base.pack) ;
        p232_lat_SET((int32_t)1112953767, PH.base.pack) ;
        p232_vd_SET((float) -4.9028273E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)40394, PH.base.pack) ;
        p232_vert_accuracy_SET((float)3.0314302E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)1.673422E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)553443691L, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
        p232_lon_SET((int32_t)1782520517, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)102, (uint8_t)89, (uint8_t)219, (uint8_t)71, (uint8_t)174, (uint8_t)43, (uint8_t)125, (uint8_t)207, (uint8_t)177, (uint8_t)174, (uint8_t)241, (uint8_t)33, (uint8_t)211, (uint8_t)128, (uint8_t)248, (uint8_t)74, (uint8_t)23, (uint8_t)49, (uint8_t)97, (uint8_t)132, (uint8_t)77, (uint8_t)245, (uint8_t)98, (uint8_t)21, (uint8_t)130, (uint8_t)123, (uint8_t)123, (uint8_t)96, (uint8_t)152, (uint8_t)137, (uint8_t)30, (uint8_t)153, (uint8_t)139, (uint8_t)128, (uint8_t)146, (uint8_t)188, (uint8_t)184, (uint8_t)185, (uint8_t)27, (uint8_t)66, (uint8_t)52, (uint8_t)239, (uint8_t)27, (uint8_t)8, (uint8_t)121, (uint8_t)43, (uint8_t)231, (uint8_t)53, (uint8_t)41, (uint8_t)123, (uint8_t)136, (uint8_t)175, (uint8_t)97, (uint8_t)107, (uint8_t)118, (uint8_t)170, (uint8_t)240, (uint8_t)152, (uint8_t)214, (uint8_t)69, (uint8_t)182, (uint8_t)79, (uint8_t)108, (uint8_t)42, (uint8_t)90, (uint8_t)105, (uint8_t)200, (uint8_t)238, (uint8_t)100, (uint8_t)36, (uint8_t)25, (uint8_t)96, (uint8_t)85, (uint8_t)6, (uint8_t)125, (uint8_t)27, (uint8_t)164, (uint8_t)32, (uint8_t)17, (uint8_t)251, (uint8_t)7, (uint8_t)250, (uint8_t)64, (uint8_t)137, (uint8_t)37, (uint8_t)40, (uint8_t)193, (uint8_t)233, (uint8_t)198, (uint8_t)6, (uint8_t)81, (uint8_t)245, (uint8_t)135, (uint8_t)145, (uint8_t)192, (uint8_t)235, (uint8_t)130, (uint8_t)204, (uint8_t)105, (uint8_t)96, (uint8_t)75, (uint8_t)249, (uint8_t)243, (uint8_t)240, (uint8_t)254, (uint8_t)200, (uint8_t)165, (uint8_t)153, (uint8_t)65, (uint8_t)12, (uint8_t)43, (uint8_t)99, (uint8_t)66, (uint8_t)19, (uint8_t)189, (uint8_t)213, (uint8_t)175, (uint8_t)112, (uint8_t)169, (uint8_t)30, (uint8_t)191, (uint8_t)242, (uint8_t)195, (uint8_t)224, (uint8_t)31, (uint8_t)91, (uint8_t)244, (uint8_t)92, (uint8_t)42, (uint8_t)177, (uint8_t)63, (uint8_t)252, (uint8_t)19, (uint8_t)118, (uint8_t)246, (uint8_t)192, (uint8_t)41, (uint8_t)113, (uint8_t)252, (uint8_t)255, (uint8_t)45, (uint8_t)222, (uint8_t)18, (uint8_t)121, (uint8_t)171, (uint8_t)224, (uint8_t)105, (uint8_t)39, (uint8_t)145, (uint8_t)130, (uint8_t)227, (uint8_t)180, (uint8_t)134, (uint8_t)160, (uint8_t)11, (uint8_t)179, (uint8_t)154, (uint8_t)85, (uint8_t)153, (uint8_t)31, (uint8_t)214, (uint8_t)199, (uint8_t)121, (uint8_t)68, (uint8_t)88, (uint8_t)75, (uint8_t)251, (uint8_t)151, (uint8_t)217, (uint8_t)217, (uint8_t)162, (uint8_t)255, (uint8_t)4, (uint8_t)40, (uint8_t)254, (uint8_t)87, (uint8_t)58, (uint8_t)182, (uint8_t)205, (uint8_t)121};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_airspeed_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -113, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)74, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -46, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)15170, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -20373, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)2885, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)112, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -31505, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)21292181L, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)1431, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1423512871, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p234_latitude_SET((int32_t) -2092738884, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -6004, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -30409, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)3427803076081836493L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.1851617E37F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)2972302286L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3588942527L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)1298888144L, PH.base.pack) ;
        p241_vibration_z_SET((float) -6.5502417E37F, PH.base.pack) ;
        p241_vibration_x_SET((float)2.8100149E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_latitude_SET((int32_t)705549743, PH.base.pack) ;
        p242_approach_z_SET((float)1.5086979E38F, PH.base.pack) ;
        {
            float q[] =  {2.269594E38F, 2.4262498E38F, 1.1884889E38F, 1.3956528E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_time_usec_SET((uint64_t)3204076889359991205L, &PH) ;
        p242_approach_x_SET((float)3.4199007E37F, PH.base.pack) ;
        p242_approach_y_SET((float) -6.358341E37F, PH.base.pack) ;
        p242_z_SET((float) -4.2308719E37F, PH.base.pack) ;
        p242_x_SET((float)1.1953838E38F, PH.base.pack) ;
        p242_y_SET((float)1.0480425E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t) -994547043, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1469350989, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_z_SET((float)2.045931E38F, PH.base.pack) ;
        p243_approach_z_SET((float)5.1239756E37F, PH.base.pack) ;
        p243_approach_y_SET((float)4.7577366E37F, PH.base.pack) ;
        p243_altitude_SET((int32_t) -340506091, PH.base.pack) ;
        p243_longitude_SET((int32_t) -2065631606, PH.base.pack) ;
        p243_approach_x_SET((float)4.985288E37F, PH.base.pack) ;
        {
            float q[] =  {-2.3561726E38F, -3.0368767E38F, 2.290081E38F, 2.1302633E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_y_SET((float) -2.5579928E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p243_latitude_SET((int32_t)1503359932, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)6850788107716223279L, &PH) ;
        p243_x_SET((float)2.521497E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)154349688, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)34541, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_lat_SET((int32_t)1049337264, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)4066, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)47283, PH.base.pack) ;
        {
            char16_t* callsign = u"d";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_ICAO_address_SET((uint32_t)308556026L, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)28779, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE, PH.base.pack) ;
        p246_lon_SET((int32_t) -1913461667, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -30619, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_altitude_SET((int32_t) -653651418, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
        p247_id_SET((uint32_t)731044899L, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.860847E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.9970724E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -4.1224936E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)23, (uint8_t)30, (uint8_t)184, (uint8_t)104, (uint8_t)105, (uint8_t)199, (uint8_t)114, (uint8_t)193, (uint8_t)103, (uint8_t)199, (uint8_t)245, (uint8_t)9, (uint8_t)1, (uint8_t)38, (uint8_t)49, (uint8_t)233, (uint8_t)249, (uint8_t)120, (uint8_t)215, (uint8_t)213, (uint8_t)27, (uint8_t)64, (uint8_t)168, (uint8_t)84, (uint8_t)202, (uint8_t)46, (uint8_t)35, (uint8_t)151, (uint8_t)5, (uint8_t)196, (uint8_t)222, (uint8_t)16, (uint8_t)118, (uint8_t)101, (uint8_t)221, (uint8_t)162, (uint8_t)253, (uint8_t)233, (uint8_t)38, (uint8_t)179, (uint8_t)56, (uint8_t)41, (uint8_t)247, (uint8_t)119, (uint8_t)189, (uint8_t)198, (uint8_t)43, (uint8_t)33, (uint8_t)174, (uint8_t)148, (uint8_t)121, (uint8_t)48, (uint8_t)102, (uint8_t)67, (uint8_t)162, (uint8_t)94, (uint8_t)179, (uint8_t)224, (uint8_t)173, (uint8_t)136, (uint8_t)148, (uint8_t)101, (uint8_t)224, (uint8_t)97, (uint8_t)237, (uint8_t)98, (uint8_t)61, (uint8_t)104, (uint8_t)157, (uint8_t)1, (uint8_t)127, (uint8_t)120, (uint8_t)217, (uint8_t)227, (uint8_t)30, (uint8_t)84, (uint8_t)97, (uint8_t)175, (uint8_t)127, (uint8_t)141, (uint8_t)155, (uint8_t)204, (uint8_t)192, (uint8_t)121, (uint8_t)181, (uint8_t)221, (uint8_t)108, (uint8_t)169, (uint8_t)177, (uint8_t)250, (uint8_t)216, (uint8_t)40, (uint8_t)9, (uint8_t)181, (uint8_t)129, (uint8_t)200, (uint8_t)103, (uint8_t)131, (uint8_t)61, (uint8_t)178, (uint8_t)47, (uint8_t)161, (uint8_t)226, (uint8_t)186, (uint8_t)5, (uint8_t)236, (uint8_t)101, (uint8_t)136, (uint8_t)100, (uint8_t)171, (uint8_t)187, (uint8_t)81, (uint8_t)27, (uint8_t)159, (uint8_t)97, (uint8_t)60, (uint8_t)151, (uint8_t)224, (uint8_t)185, (uint8_t)229, (uint8_t)32, (uint8_t)106, (uint8_t)118, (uint8_t)52, (uint8_t)191, (uint8_t)85, (uint8_t)29, (uint8_t)1, (uint8_t)224, (uint8_t)215, (uint8_t)187, (uint8_t)169, (uint8_t)73, (uint8_t)44, (uint8_t)16, (uint8_t)99, (uint8_t)253, (uint8_t)72, (uint8_t)136, (uint8_t)199, (uint8_t)138, (uint8_t)40, (uint8_t)121, (uint8_t)237, (uint8_t)158, (uint8_t)94, (uint8_t)115, (uint8_t)246, (uint8_t)141, (uint8_t)2, (uint8_t)51, (uint8_t)140, (uint8_t)139, (uint8_t)9, (uint8_t)101, (uint8_t)165, (uint8_t)139, (uint8_t)60, (uint8_t)246, (uint8_t)119, (uint8_t)105, (uint8_t)207, (uint8_t)199, (uint8_t)127, (uint8_t)52, (uint8_t)249, (uint8_t)69, (uint8_t)30, (uint8_t)199, (uint8_t)210, (uint8_t)78, (uint8_t)39, (uint8_t)47, (uint8_t)202, (uint8_t)226, (uint8_t)65, (uint8_t)162, (uint8_t)32, (uint8_t)45, (uint8_t)71, (uint8_t)176, (uint8_t)24, (uint8_t)97, (uint8_t)22, (uint8_t)22, (uint8_t)0, (uint8_t)196, (uint8_t)242, (uint8_t)79, (uint8_t)82, (uint8_t)7, (uint8_t)73, (uint8_t)176, (uint8_t)128, (uint8_t)205, (uint8_t)187, (uint8_t)129, (uint8_t)173, (uint8_t)150, (uint8_t)196, (uint8_t)232, (uint8_t)183, (uint8_t)143, (uint8_t)255, (uint8_t)181, (uint8_t)10, (uint8_t)42, (uint8_t)65, (uint8_t)67, (uint8_t)63, (uint8_t)71, (uint8_t)82, (uint8_t)240, (uint8_t)152, (uint8_t)149, (uint8_t)115, (uint8_t)103, (uint8_t)193, (uint8_t)141, (uint8_t)23, (uint8_t)208, (uint8_t)184, (uint8_t)115, (uint8_t)210, (uint8_t)104, (uint8_t)159, (uint8_t)180, (uint8_t)136, (uint8_t)246, (uint8_t)87, (uint8_t)179, (uint8_t)37, (uint8_t)72, (uint8_t)54, (uint8_t)235, (uint8_t)195, (uint8_t)229, (uint8_t)132, (uint8_t)84, (uint8_t)31, (uint8_t)177, (uint8_t)88, (uint8_t)203, (uint8_t)163, (uint8_t)143, (uint8_t)21, (uint8_t)111, (uint8_t)210, (uint8_t)57};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)48270, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -123, (int8_t)78, (int8_t) -60, (int8_t)78, (int8_t) -59, (int8_t) -44, (int8_t) -27, (int8_t) -53, (int8_t) -7, (int8_t)34, (int8_t) -99, (int8_t) -86, (int8_t)13, (int8_t)4, (int8_t) -120, (int8_t)124, (int8_t) -22, (int8_t)20, (int8_t) -94, (int8_t)45, (int8_t)108, (int8_t) -29, (int8_t) -127, (int8_t) -39, (int8_t)40, (int8_t)93, (int8_t)81, (int8_t)89, (int8_t) -93, (int8_t)39, (int8_t) -4, (int8_t)96};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)65079, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_z_SET((float)3.3709065E38F, PH.base.pack) ;
        p250_x_SET((float) -2.179323E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)5858136002014523667L, PH.base.pack) ;
        p250_y_SET((float) -3.0941467E37F, PH.base.pack) ;
        {
            char16_t* name = u"Wsqdtxxny";
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
            char16_t* name = u"g";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float)8.667248E37F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)3963163495L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)3709889355L, PH.base.pack) ;
        {
            char16_t* name = u"fejakn";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t) -1373588649, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_WARNING, PH.base.pack) ;
        {
            char16_t* text = u"lhhAbwJpvbnoacchRfv";
            p253_text_SET_(text, &PH) ;
        }
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p254_value_SET((float)2.8320008E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1128144753L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)55, (uint8_t)157, (uint8_t)180, (uint8_t)219, (uint8_t)33, (uint8_t)119, (uint8_t)112, (uint8_t)120, (uint8_t)200, (uint8_t)207, (uint8_t)242, (uint8_t)245, (uint8_t)3, (uint8_t)147, (uint8_t)242, (uint8_t)94, (uint8_t)148, (uint8_t)225, (uint8_t)253, (uint8_t)59, (uint8_t)145, (uint8_t)154, (uint8_t)119, (uint8_t)203, (uint8_t)169, (uint8_t)109, (uint8_t)233, (uint8_t)49, (uint8_t)96, (uint8_t)234, (uint8_t)176, (uint8_t)214};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)8737286716707572250L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)1637976829L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1240174145L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"tamxjsbqditlqw";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p258_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        {
            uint8_t vendor_name[] =  {(uint8_t)201, (uint8_t)167, (uint8_t)143, (uint8_t)56, (uint8_t)7, (uint8_t)192, (uint8_t)255, (uint8_t)118, (uint8_t)102, (uint8_t)29, (uint8_t)121, (uint8_t)59, (uint8_t)229, (uint8_t)20, (uint8_t)31, (uint8_t)182, (uint8_t)198, (uint8_t)148, (uint8_t)28, (uint8_t)65, (uint8_t)229, (uint8_t)173, (uint8_t)80, (uint8_t)1, (uint8_t)197, (uint8_t)106, (uint8_t)87, (uint8_t)122, (uint8_t)154, (uint8_t)45, (uint8_t)180, (uint8_t)46};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_focal_length_SET((float)3.031503E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"ynguqjoglrskqMuyBezUnxUtoeHeaHyVumxaXlShqltpxgiazzqinzkkYwe";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)48276, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)51, (uint8_t)144, (uint8_t)97, (uint8_t)226, (uint8_t)90, (uint8_t)150, (uint8_t)195, (uint8_t)54, (uint8_t)158, (uint8_t)183, (uint8_t)40, (uint8_t)16, (uint8_t)22, (uint8_t)46, (uint8_t)19, (uint8_t)128, (uint8_t)59, (uint8_t)249, (uint8_t)248, (uint8_t)166, (uint8_t)103, (uint8_t)251, (uint8_t)239, (uint8_t)149, (uint8_t)47, (uint8_t)170, (uint8_t)45, (uint8_t)182, (uint8_t)31, (uint8_t)70, (uint8_t)77, (uint8_t)143};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)1989799924L, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)36038, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1887218845L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)14796, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -1.2934404E38F, PH.base.pack) ;
        p259_sensor_size_h_SET((float)2.8187355E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)1854242732L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_time_boot_ms_SET((uint32_t)3163884028L, PH.base.pack) ;
        p261_write_speed_SET((float) -2.6683212E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.2829105E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.3838944E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -2.3516484E38F, PH.base.pack) ;
        p261_available_capacity_SET((float)1.3658905E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_video_status_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p262_available_capacity_SET((float)7.132658E37F, PH.base.pack) ;
        p262_image_interval_SET((float) -1.2164434E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)4165679333L, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)3029243807L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)2280144257959568461L, PH.base.pack) ;
        p263_lat_SET((int32_t) -673971637, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -216761281, PH.base.pack) ;
        {
            char16_t* file_url = u"tyGvibyihimtroznnndbAiaojtjsumyjpshYfvGcnpxogaMwlhvfyeiaztkduzwxsvnlcsoaIFrazlyOn";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_image_index_SET((int32_t)1643373056, PH.base.pack) ;
        p263_lon_SET((int32_t) -1715385798, PH.base.pack) ;
        {
            float q[] =  {2.49254E38F, 5.3601703E37F, 1.973693E38F, -3.3311589E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_alt_SET((int32_t) -1822330, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -77, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1599961843L, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)6541853961459594206L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)3046414163L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)4349507890888381586L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)8842626941677475250L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float)2.1129192E38F, PH.base.pack) ;
        p265_pitch_SET((float)2.1953926E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)4227332680L, PH.base.pack) ;
        p265_roll_SET((float)3.5628405E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_length_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)155, (uint8_t)255, (uint8_t)105, (uint8_t)199, (uint8_t)251, (uint8_t)34, (uint8_t)250, (uint8_t)197, (uint8_t)114, (uint8_t)56, (uint8_t)48, (uint8_t)95, (uint8_t)227, (uint8_t)3, (uint8_t)53, (uint8_t)235, (uint8_t)199, (uint8_t)42, (uint8_t)211, (uint8_t)135, (uint8_t)71, (uint8_t)168, (uint8_t)121, (uint8_t)111, (uint8_t)248, (uint8_t)17, (uint8_t)214, (uint8_t)12, (uint8_t)245, (uint8_t)155, (uint8_t)128, (uint8_t)191, (uint8_t)90, (uint8_t)198, (uint8_t)188, (uint8_t)172, (uint8_t)115, (uint8_t)94, (uint8_t)141, (uint8_t)191, (uint8_t)121, (uint8_t)247, (uint8_t)198, (uint8_t)125, (uint8_t)18, (uint8_t)85, (uint8_t)99, (uint8_t)7, (uint8_t)193, (uint8_t)15, (uint8_t)74, (uint8_t)180, (uint8_t)190, (uint8_t)101, (uint8_t)205, (uint8_t)213, (uint8_t)138, (uint8_t)249, (uint8_t)165, (uint8_t)130, (uint8_t)81, (uint8_t)237, (uint8_t)180, (uint8_t)37, (uint8_t)232, (uint8_t)27, (uint8_t)88, (uint8_t)76, (uint8_t)166, (uint8_t)78, (uint8_t)159, (uint8_t)29, (uint8_t)61, (uint8_t)135, (uint8_t)97, (uint8_t)241, (uint8_t)4, (uint8_t)173, (uint8_t)45, (uint8_t)169, (uint8_t)52, (uint8_t)175, (uint8_t)20, (uint8_t)74, (uint8_t)145, (uint8_t)41, (uint8_t)208, (uint8_t)244, (uint8_t)64, (uint8_t)223, (uint8_t)202, (uint8_t)26, (uint8_t)100, (uint8_t)143, (uint8_t)128, (uint8_t)164, (uint8_t)181, (uint8_t)152, (uint8_t)74, (uint8_t)185, (uint8_t)124, (uint8_t)59, (uint8_t)160, (uint8_t)167, (uint8_t)3, (uint8_t)98, (uint8_t)128, (uint8_t)171, (uint8_t)219, (uint8_t)27, (uint8_t)114, (uint8_t)56, (uint8_t)126, (uint8_t)30, (uint8_t)45, (uint8_t)104, (uint8_t)199, (uint8_t)39, (uint8_t)181, (uint8_t)122, (uint8_t)123, (uint8_t)228, (uint8_t)0, (uint8_t)3, (uint8_t)38, (uint8_t)19, (uint8_t)90, (uint8_t)60, (uint8_t)39, (uint8_t)146, (uint8_t)104, (uint8_t)116, (uint8_t)187, (uint8_t)243, (uint8_t)103, (uint8_t)203, (uint8_t)8, (uint8_t)0, (uint8_t)193, (uint8_t)103, (uint8_t)88, (uint8_t)40, (uint8_t)21, (uint8_t)25, (uint8_t)150, (uint8_t)140, (uint8_t)178, (uint8_t)76, (uint8_t)175, (uint8_t)254, (uint8_t)169, (uint8_t)240, (uint8_t)127, (uint8_t)94, (uint8_t)199, (uint8_t)27, (uint8_t)157, (uint8_t)21, (uint8_t)16, (uint8_t)112, (uint8_t)143, (uint8_t)109, (uint8_t)26, (uint8_t)7, (uint8_t)2, (uint8_t)101, (uint8_t)4, (uint8_t)13, (uint8_t)30, (uint8_t)95, (uint8_t)138, (uint8_t)145, (uint8_t)98, (uint8_t)54, (uint8_t)51, (uint8_t)219, (uint8_t)34, (uint8_t)12, (uint8_t)191, (uint8_t)61, (uint8_t)80, (uint8_t)196, (uint8_t)17, (uint8_t)128, (uint8_t)158, (uint8_t)40, (uint8_t)142, (uint8_t)29, (uint8_t)100, (uint8_t)84, (uint8_t)80, (uint8_t)125, (uint8_t)8, (uint8_t)197, (uint8_t)160, (uint8_t)232, (uint8_t)192, (uint8_t)48, (uint8_t)94, (uint8_t)48, (uint8_t)66, (uint8_t)240, (uint8_t)119, (uint8_t)151, (uint8_t)149, (uint8_t)244, (uint8_t)226, (uint8_t)128, (uint8_t)89, (uint8_t)83, (uint8_t)248, (uint8_t)181, (uint8_t)180, (uint8_t)157, (uint8_t)214, (uint8_t)76, (uint8_t)3, (uint8_t)214, (uint8_t)185, (uint8_t)233, (uint8_t)49, (uint8_t)129, (uint8_t)62, (uint8_t)61, (uint8_t)2, (uint8_t)70, (uint8_t)237, (uint8_t)193, (uint8_t)93, (uint8_t)198, (uint8_t)166, (uint8_t)213, (uint8_t)152, (uint8_t)28, (uint8_t)63, (uint8_t)149, (uint8_t)63, (uint8_t)137, (uint8_t)73, (uint8_t)80, (uint8_t)130, (uint8_t)87, (uint8_t)112, (uint8_t)201, (uint8_t)106, (uint8_t)63, (uint8_t)170, (uint8_t)171, (uint8_t)205};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_first_message_offset_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)11614, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_first_message_offset_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)26605, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)23, (uint8_t)3, (uint8_t)246, (uint8_t)47, (uint8_t)239, (uint8_t)22, (uint8_t)246, (uint8_t)251, (uint8_t)238, (uint8_t)61, (uint8_t)188, (uint8_t)114, (uint8_t)106, (uint8_t)106, (uint8_t)2, (uint8_t)218, (uint8_t)40, (uint8_t)159, (uint8_t)23, (uint8_t)165, (uint8_t)122, (uint8_t)137, (uint8_t)153, (uint8_t)169, (uint8_t)40, (uint8_t)32, (uint8_t)152, (uint8_t)124, (uint8_t)232, (uint8_t)60, (uint8_t)98, (uint8_t)56, (uint8_t)174, (uint8_t)5, (uint8_t)205, (uint8_t)236, (uint8_t)201, (uint8_t)239, (uint8_t)36, (uint8_t)25, (uint8_t)23, (uint8_t)253, (uint8_t)128, (uint8_t)192, (uint8_t)119, (uint8_t)48, (uint8_t)63, (uint8_t)220, (uint8_t)178, (uint8_t)65, (uint8_t)147, (uint8_t)109, (uint8_t)88, (uint8_t)214, (uint8_t)62, (uint8_t)135, (uint8_t)95, (uint8_t)73, (uint8_t)151, (uint8_t)129, (uint8_t)209, (uint8_t)15, (uint8_t)170, (uint8_t)235, (uint8_t)139, (uint8_t)82, (uint8_t)236, (uint8_t)195, (uint8_t)34, (uint8_t)14, (uint8_t)208, (uint8_t)149, (uint8_t)56, (uint8_t)214, (uint8_t)113, (uint8_t)176, (uint8_t)108, (uint8_t)204, (uint8_t)187, (uint8_t)47, (uint8_t)50, (uint8_t)236, (uint8_t)22, (uint8_t)96, (uint8_t)203, (uint8_t)111, (uint8_t)40, (uint8_t)28, (uint8_t)187, (uint8_t)38, (uint8_t)54, (uint8_t)252, (uint8_t)179, (uint8_t)108, (uint8_t)149, (uint8_t)181, (uint8_t)71, (uint8_t)140, (uint8_t)194, (uint8_t)61, (uint8_t)150, (uint8_t)85, (uint8_t)252, (uint8_t)89, (uint8_t)234, (uint8_t)152, (uint8_t)117, (uint8_t)33, (uint8_t)15, (uint8_t)249, (uint8_t)242, (uint8_t)193, (uint8_t)148, (uint8_t)98, (uint8_t)26, (uint8_t)161, (uint8_t)116, (uint8_t)12, (uint8_t)82, (uint8_t)230, (uint8_t)162, (uint8_t)197, (uint8_t)17, (uint8_t)197, (uint8_t)89, (uint8_t)113, (uint8_t)68, (uint8_t)218, (uint8_t)66, (uint8_t)2, (uint8_t)248, (uint8_t)88, (uint8_t)103, (uint8_t)148, (uint8_t)214, (uint8_t)39, (uint8_t)32, (uint8_t)104, (uint8_t)190, (uint8_t)104, (uint8_t)220, (uint8_t)87, (uint8_t)138, (uint8_t)52, (uint8_t)165, (uint8_t)194, (uint8_t)80, (uint8_t)84, (uint8_t)99, (uint8_t)79, (uint8_t)43, (uint8_t)251, (uint8_t)0, (uint8_t)214, (uint8_t)218, (uint8_t)218, (uint8_t)13, (uint8_t)135, (uint8_t)198, (uint8_t)167, (uint8_t)100, (uint8_t)156, (uint8_t)67, (uint8_t)153, (uint8_t)233, (uint8_t)227, (uint8_t)124, (uint8_t)143, (uint8_t)93, (uint8_t)227, (uint8_t)34, (uint8_t)187, (uint8_t)161, (uint8_t)187, (uint8_t)38, (uint8_t)113, (uint8_t)24, (uint8_t)37, (uint8_t)208, (uint8_t)159, (uint8_t)49, (uint8_t)251, (uint8_t)39, (uint8_t)148, (uint8_t)32, (uint8_t)180, (uint8_t)76, (uint8_t)144, (uint8_t)161, (uint8_t)14, (uint8_t)141, (uint8_t)156, (uint8_t)246, (uint8_t)148, (uint8_t)108, (uint8_t)204, (uint8_t)19, (uint8_t)115, (uint8_t)35, (uint8_t)132, (uint8_t)70, (uint8_t)194, (uint8_t)225, (uint8_t)110, (uint8_t)175, (uint8_t)144, (uint8_t)71, (uint8_t)128, (uint8_t)172, (uint8_t)113, (uint8_t)131, (uint8_t)59, (uint8_t)236, (uint8_t)204, (uint8_t)222, (uint8_t)251, (uint8_t)28, (uint8_t)201, (uint8_t)222, (uint8_t)244, (uint8_t)112, (uint8_t)113, (uint8_t)172, (uint8_t)17, (uint8_t)20, (uint8_t)57, (uint8_t)221, (uint8_t)206, (uint8_t)254, (uint8_t)145, (uint8_t)228, (uint8_t)17, (uint8_t)248, (uint8_t)202, (uint8_t)60, (uint8_t)150, (uint8_t)248, (uint8_t)198, (uint8_t)33, (uint8_t)88, (uint8_t)209, (uint8_t)7, (uint8_t)49, (uint8_t)73, (uint8_t)248, (uint8_t)249, (uint8_t)22, (uint8_t)80, (uint8_t)3};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_length_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)51323, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_status_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p269_framerate_SET((float) -2.0777426E38F, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)8438, PH.base.pack) ;
        {
            char16_t* uri = u"cpddwldptvzpacuicqmwjyucxterAttbozcihxcecfdhvnbDvUtinfytxusTzyrmqWmhesaocZigqVuxGtrbpxsrcjzznswFxohnrtwgptWtxjhq";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_h_SET((uint16_t)(uint16_t)22884, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)2816672418L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)32515, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_v_SET((uint16_t)(uint16_t)60760, PH.base.pack) ;
        p270_framerate_SET((float) -1.0060439E38F, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        {
            char16_t* uri = u"uuktoKcutyePidpdutocwgcjhkbrcsuvqXqusNdPfiWydhtmSmumgwQfYfI";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_rotation_SET((uint16_t)(uint16_t)7505, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)34890, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2950591103L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"gUg";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"ddvoapmsoruf";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)21280, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)3179, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)38836, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)182, (uint8_t)180, (uint8_t)147, (uint8_t)160, (uint8_t)77, (uint8_t)137, (uint8_t)189, (uint8_t)208};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t spec_version_hash[] =  {(uint8_t)148, (uint8_t)205, (uint8_t)19, (uint8_t)22, (uint8_t)10, (uint8_t)100, (uint8_t)87, (uint8_t)237};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_sub_mode_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)7269482200254147010L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)2552444837L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)13812, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)6325359743379233643L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        {
            char16_t* name = u"dsrctpfmcggnkhrpahqpbsHaixjkOtIvhlwjsiklOxmqgyAadgwNgqsumtiaqyvlijcu";
            p311_name_SET_(name, &PH) ;
        }
        {
            uint8_t hw_unique_id[] =  {(uint8_t)221, (uint8_t)253, (uint8_t)3, (uint8_t)231, (uint8_t)25, (uint8_t)66, (uint8_t)194, (uint8_t)50, (uint8_t)238, (uint8_t)97, (uint8_t)185, (uint8_t)124, (uint8_t)88, (uint8_t)150, (uint8_t)98, (uint8_t)113};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_hw_version_major_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2784715429L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)275851539L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_component_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)10726, PH.base.pack) ;
        {
            char16_t* param_id = u"ebjskji";
            p320_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"zsn";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)54569, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)11926, PH.base.pack) ;
        {
            char16_t* param_value = u"stiqzqetnygWEhgzkdzkekvcswggfybgsyxewnmkSjfTziiebbx";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        {
            char16_t* param_id = u"n";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        {
            char16_t* param_value = u"nPmqgeMyeJuoSvjyfnwieowrqphowrscmfrl";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        {
            char16_t* param_id = u"cpcKxEsmzh";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"ZJllgvkifbmsctjfinhMlogcPjpdoerunlxbooLmflgujjOnxzmajtebrj";
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
        p330_increment_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)7155201189049258274L, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)5372, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)27645, (uint16_t)53720, (uint16_t)27339, (uint16_t)61161, (uint16_t)23271, (uint16_t)8219, (uint16_t)25997, (uint16_t)10378, (uint16_t)62568, (uint16_t)34440, (uint16_t)51501, (uint16_t)1601, (uint16_t)43008, (uint16_t)24281, (uint16_t)35314, (uint16_t)57920, (uint16_t)14480, (uint16_t)24892, (uint16_t)2186, (uint16_t)7800, (uint16_t)29133, (uint16_t)61558, (uint16_t)6422, (uint16_t)45281, (uint16_t)48858, (uint16_t)22651, (uint16_t)61593, (uint16_t)60131, (uint16_t)22915, (uint16_t)33306, (uint16_t)20816, (uint16_t)27670, (uint16_t)36511, (uint16_t)25521, (uint16_t)64181, (uint16_t)26007, (uint16_t)44255, (uint16_t)13866, (uint16_t)36209, (uint16_t)19644, (uint16_t)39343, (uint16_t)25399, (uint16_t)36077, (uint16_t)35925, (uint16_t)62050, (uint16_t)11017, (uint16_t)12030, (uint16_t)58226, (uint16_t)9293, (uint16_t)48548, (uint16_t)59796, (uint16_t)38533, (uint16_t)44871, (uint16_t)38494, (uint16_t)46923, (uint16_t)17761, (uint16_t)56244, (uint16_t)2424, (uint16_t)47293, (uint16_t)60398, (uint16_t)28004, (uint16_t)3268, (uint16_t)43626, (uint16_t)17965, (uint16_t)20021, (uint16_t)2626, (uint16_t)54435, (uint16_t)41394, (uint16_t)42559, (uint16_t)46004, (uint16_t)6468, (uint16_t)14540};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_max_distance_SET((uint16_t)(uint16_t)34673, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


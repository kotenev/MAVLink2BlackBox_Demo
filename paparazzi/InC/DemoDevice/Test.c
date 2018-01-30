
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p0_custom_mode_GET(pack) == (uint32_t)809137286L);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)11986);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)21167);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)43126);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)28571);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)44159);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)29154);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)12980);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)8179);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)61);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)15629);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2622098734589340627L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)3194082523L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vy_GET(pack) == (float) -1.4164575E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)9692);
    assert(p3_vx_GET(pack) == (float)2.776923E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3566854029L);
    assert(p3_yaw_rate_GET(pack) == (float) -1.0917272E37F);
    assert(p3_afy_GET(pack) == (float)2.351771E38F);
    assert(p3_vz_GET(pack) == (float)2.038012E38F);
    assert(p3_afz_GET(pack) == (float) -2.4110873E38F);
    assert(p3_y_GET(pack) == (float)1.1096203E37F);
    assert(p3_afx_GET(pack) == (float)1.806732E38F);
    assert(p3_x_GET(pack) == (float) -2.4023005E38F);
    assert(p3_yaw_GET(pack) == (float)3.352143E38F);
    assert(p3_z_GET(pack) == (float)2.875108E38F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p4_seq_GET(pack) == (uint32_t)3894362690L);
    assert(p4_time_usec_GET(pack) == (uint64_t)3283251225089904289L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p5_passkey_LEN(ph) == 5);
    {
        char16_t * exemplary = u"ayzkk";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)236);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 20);
    {
        char16_t * exemplary = u"hMvcllXxnkizbaxizkbR";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)808621476L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)167);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"dkanyiAu";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -30822);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)103);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"Gpzgdskbyahv";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)43371);
    assert(p22_param_value_GET(pack) == (float)1.8775855E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)36037);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32);
    assert(p23_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"odkuiwfQdzFfr";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p23_param_value_GET(pack) == (float) -2.7998846E38F);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_v_acc_TRY(ph) == (uint32_t)1092499746L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)18942);
    assert(p24_lat_GET(pack) == (int32_t)1131206628);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)51077);
    assert(p24_lon_GET(pack) == (int32_t)917063662);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)21092);
    assert(p24_time_usec_GET(pack) == (uint64_t)1965996827686821947L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)2072254636);
    assert(p24_h_acc_TRY(ph) == (uint32_t)203254143L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1040543488L);
    assert(p24_alt_GET(pack) == (int32_t)1698831682);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)271104242L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)61423);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)19, (uint8_t)36, (uint8_t)237, (uint8_t)101, (uint8_t)49, (uint8_t)158, (uint8_t)149, (uint8_t)51, (uint8_t)87, (uint8_t)184, (uint8_t)118, (uint8_t)78, (uint8_t)160, (uint8_t)56, (uint8_t)222, (uint8_t)231, (uint8_t)124, (uint8_t)185, (uint8_t)142, (uint8_t)95} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)16, (uint8_t)159, (uint8_t)56, (uint8_t)248, (uint8_t)143, (uint8_t)250, (uint8_t)9, (uint8_t)140, (uint8_t)82, (uint8_t)191, (uint8_t)192, (uint8_t)56, (uint8_t)196, (uint8_t)142, (uint8_t)115, (uint8_t)204, (uint8_t)14, (uint8_t)204, (uint8_t)84} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)226, (uint8_t)82, (uint8_t)56, (uint8_t)51, (uint8_t)169, (uint8_t)35, (uint8_t)33, (uint8_t)26, (uint8_t)9, (uint8_t)192, (uint8_t)100, (uint8_t)240, (uint8_t)171, (uint8_t)62, (uint8_t)255, (uint8_t)99, (uint8_t)220, (uint8_t)71, (uint8_t)232, (uint8_t)232} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)121, (uint8_t)60, (uint8_t)0, (uint8_t)146, (uint8_t)136, (uint8_t)186, (uint8_t)88, (uint8_t)248, (uint8_t)81, (uint8_t)213, (uint8_t)27, (uint8_t)32, (uint8_t)9, (uint8_t)67, (uint8_t)172, (uint8_t)81, (uint8_t)2, (uint8_t)61, (uint8_t)37, (uint8_t)251} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)8, (uint8_t)89, (uint8_t)177, (uint8_t)113, (uint8_t)131, (uint8_t)211, (uint8_t)133, (uint8_t)36, (uint8_t)52, (uint8_t)81, (uint8_t)110, (uint8_t)239, (uint8_t)249, (uint8_t)104, (uint8_t)98, (uint8_t)63, (uint8_t)99, (uint8_t)133, (uint8_t)81} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)111);
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -9454);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)31093);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3975318625L);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)7129);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)6687);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -6643);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)27370);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)1892);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -12842);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)12712);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)9941);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -23267);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)22080);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -16617);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)8963);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -4920);
    assert(p27_time_usec_GET(pack) == (uint64_t)5940732124011629387L);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -15378);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)20131);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)31379);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)26369);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)12958);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)20497);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -30655);
    assert(p28_time_usec_GET(pack) == (uint64_t)2291462735257379762L);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)16570);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3414860797L);
    assert(p29_press_diff_GET(pack) == (float)3.8142357E37F);
    assert(p29_press_abs_GET(pack) == (float)3.141997E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitchspeed_GET(pack) == (float) -1.8285525E38F);
    assert(p30_rollspeed_GET(pack) == (float) -2.653771E38F);
    assert(p30_yawspeed_GET(pack) == (float) -3.258774E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2335728506L);
    assert(p30_pitch_GET(pack) == (float) -3.9584297E37F);
    assert(p30_yaw_GET(pack) == (float)2.485853E38F);
    assert(p30_roll_GET(pack) == (float) -3.3953707E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2826235990L);
    assert(p31_q1_GET(pack) == (float)1.0023267E38F);
    assert(p31_rollspeed_GET(pack) == (float)2.5915043E37F);
    assert(p31_q4_GET(pack) == (float) -1.6826351E38F);
    assert(p31_q2_GET(pack) == (float) -2.6987969E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -1.6823233E38F);
    assert(p31_yawspeed_GET(pack) == (float) -9.146004E37F);
    assert(p31_q3_GET(pack) == (float)2.8121533E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float) -2.1209843E38F);
    assert(p32_y_GET(pack) == (float) -1.4877643E38F);
    assert(p32_vx_GET(pack) == (float) -3.0096021E38F);
    assert(p32_vz_GET(pack) == (float)2.800312E38F);
    assert(p32_vy_GET(pack) == (float)2.4714272E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)3110168308L);
    assert(p32_x_GET(pack) == (float) -6.7666494E37F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -8950);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -6222);
    assert(p33_alt_GET(pack) == (int32_t)353592647);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)37292);
    assert(p33_lat_GET(pack) == (int32_t) -1329296951);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)13176);
    assert(p33_relative_alt_GET(pack) == (int32_t) -1678664400);
    assert(p33_lon_GET(pack) == (int32_t) -1171601082);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)868030831L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -23397);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -3723);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)2174);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -7782);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2082238375L);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)19500);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -17996);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -19055);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)32361);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)116);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)40410);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)64542);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)32965);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)51998);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)966);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)12535);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3879832184L);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)45784);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)35764);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)24235);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)938);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)32409);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)32245);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)40668);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)46791);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)14170);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)62612);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)31968);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)12348);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)45810);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)297);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p36_time_usec_GET(pack) == (uint32_t)144066584L);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)30453);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)35506);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)54474);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)54628);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -12710);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)466);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)24287);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -23285);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_y_GET(pack) == (float)1.8165956E38F);
    assert(p39_param4_GET(pack) == (float)2.1190548E38F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)58776);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p39_param1_GET(pack) == (float)2.6505805E38F);
    assert(p39_z_GET(pack) == (float) -1.2766981E38F);
    assert(p39_x_GET(pack) == (float)1.3559911E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p39_param3_GET(pack) == (float) -7.8541486E37F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_HOME);
    assert(p39_param2_GET(pack) == (float)1.7989696E37F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)57266);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)7104);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)54);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)63981);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)4639);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)53715);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t) -2119536804);
    assert(p48_latitude_GET(pack) == (int32_t)1754146992);
    assert(p48_longitude_GET(pack) == (int32_t)1198185430);
    assert(p48_time_usec_TRY(ph) == (uint64_t)4617856014626162964L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)105);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)4447735428779582035L);
    assert(p49_latitude_GET(pack) == (int32_t) -2070343737);
    assert(p49_altitude_GET(pack) == (int32_t) -201984389);
    assert(p49_longitude_GET(pack) == (int32_t)1731454014);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)28948);
    assert(p50_scale_GET(pack) == (float)7.608052E37F);
    assert(p50_param_value_min_GET(pack) == (float)2.341577E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"u";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p50_param_value_max_GET(pack) == (float)1.9067686E38F);
    assert(p50_param_value0_GET(pack) == (float)3.1569676E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)57435);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)3);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1y_GET(pack) == (float)3.2041502E38F);
    assert(p54_p2x_GET(pack) == (float) -1.5836841E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p54_p2z_GET(pack) == (float)1.5300057E37F);
    assert(p54_p1z_GET(pack) == (float)8.142682E37F);
    assert(p54_p1x_GET(pack) == (float) -3.1517416E38F);
    assert(p54_p2y_GET(pack) == (float) -9.99712E37F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)1.1272875E38F);
    assert(p55_p1z_GET(pack) == (float) -3.1930875E38F);
    assert(p55_p1y_GET(pack) == (float)2.3435122E38F);
    assert(p55_p2y_GET(pack) == (float)5.628565E37F);
    assert(p55_p2x_GET(pack) == (float)1.72727E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p55_p1x_GET(pack) == (float) -3.3058752E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float) -9.75791E37F);
    assert(p61_pitchspeed_GET(pack) == (float) -2.797504E38F);
    {
        float exemplary[] =  {9.789241E37F, 3.3434416E38F, 2.6899905E38F, -1.3859009E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)5.4175665E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)1707846319331419233L);
    {
        float exemplary[] =  {3.394102E38F, -6.2298937E37F, 2.0002888E38F, -1.3581386E38F, 3.2715545E38F, 8.185516E37F, 9.821624E37F, 8.473732E37F, 1.2709218E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float)3.3510461E38F);
    assert(p62_xtrack_error_GET(pack) == (float)3.7627014E37F);
    assert(p62_nav_pitch_GET(pack) == (float)2.412902E38F);
    assert(p62_aspd_error_GET(pack) == (float) -2.3203097E38F);
    assert(p62_alt_error_GET(pack) == (float)8.0771654E37F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -4959);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)29976);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -10054);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vx_GET(pack) == (float)1.7129617E38F);
    assert(p63_vz_GET(pack) == (float)1.1649187E38F);
    assert(p63_lon_GET(pack) == (int32_t) -1872223117);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_time_usec_GET(pack) == (uint64_t)2639236667969392442L);
    {
        float exemplary[] =  {-1.064123E38F, -1.4620767E38F, 3.86923E36F, 1.3462043E38F, 2.684396E38F, 2.7896266E38F, 3.2636293E38F, -1.1491052E38F, 1.5616369E38F, -2.7785275E38F, -3.003372E38F, -1.0468292E38F, 1.1434643E37F, 1.4722228E37F, 1.2581943E38F, 4.955536E37F, 2.5238378E38F, -2.5531479E38F, 2.9823365E38F, 2.6868331E38F, 2.237061E38F, -2.5427262E38F, 3.069777E37F, -2.5956977E37F, 3.0141786E38F, 3.114397E38F, -2.1987211E38F, 6.554806E37F, -3.0029637E38F, -2.3893096E38F, -3.8811047E37F, -7.2221594E37F, -3.0740823E38F, 1.7966424E38F, 2.8085152E38F, 8.71469E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vy_GET(pack) == (float)1.4705049E38F);
    assert(p63_relative_alt_GET(pack) == (int32_t)1387443891);
    assert(p63_lat_GET(pack) == (int32_t) -996470391);
    assert(p63_alt_GET(pack) == (int32_t)190649253);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_az_GET(pack) == (float) -3.3532192E38F);
    assert(p64_x_GET(pack) == (float)2.1294559E38F);
    assert(p64_ax_GET(pack) == (float) -2.4372753E38F);
    assert(p64_y_GET(pack) == (float)3.6606824E37F);
    assert(p64_z_GET(pack) == (float)1.55938E38F);
    assert(p64_ay_GET(pack) == (float) -5.0981474E37F);
    assert(p64_vx_GET(pack) == (float) -2.892513E38F);
    assert(p64_vz_GET(pack) == (float) -2.767883E38F);
    assert(p64_vy_GET(pack) == (float) -2.6319187E37F);
    {
        float exemplary[] =  {1.7250078E38F, 1.6597572E38F, -3.3449937E38F, 2.6677624E38F, 1.9270476E38F, 1.3629696E37F, -1.8444174E38F, -1.5628041E38F, 2.8566285E38F, 2.39489E38F, -3.3419977E38F, -1.6509677E38F, -1.6596258E38F, -5.353558E37F, 1.8653868E38F, -7.036635E37F, -2.3141298E38F, 2.4965267E38F, -2.067062E38F, 2.9809051E38F, -1.513602E38F, 3.4606415E37F, -3.3697607E38F, 2.958012E38F, -2.782608E38F, 5.922633E37F, 1.5916379E38F, -1.571914E38F, -7.55337E36F, -1.970722E38F, 1.082225E38F, -1.0061431E38F, -3.1080892E38F, -3.044912E37F, 3.580379E36F, -2.3510775E38F, 2.909231E38F, -2.8620338E37F, -1.2790894E38F, 2.0760592E38F, -1.4390315E38F, -1.8236772E38F, -9.606877E37F, -2.7597775E37F, 2.9291116E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_time_usec_GET(pack) == (uint64_t)3742173354121202861L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)11211);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)21199);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)27098);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)40527);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)56046);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)4424);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)59300);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)11886);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)61509);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)38867);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)50750);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)46712);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)50229);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)49534);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)353);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)123295187L);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)53160);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)37180);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)63063);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)60249);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)32426);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)28160);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)24492);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)19128);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -16344);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)21203);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)58739);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)28720);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)42251);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)61998);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)65263);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)53593);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)4187);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)10385);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p73_param4_GET(pack) == (float) -7.4471874E36F);
    assert(p73_param2_GET(pack) == (float)3.2073686E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p73_x_GET(pack) == (int32_t)199609060);
    assert(p73_param3_GET(pack) == (float) -3.0292566E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LAND);
    assert(p73_y_GET(pack) == (int32_t)928416433);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p73_param1_GET(pack) == (float) -1.8959855E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_z_GET(pack) == (float)2.005527E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)17401);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_climb_GET(pack) == (float)1.9968639E38F);
    assert(p74_alt_GET(pack) == (float)9.806183E37F);
    assert(p74_groundspeed_GET(pack) == (float) -1.8439098E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -12961);
    assert(p74_airspeed_GET(pack) == (float)4.093992E37F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)43515);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param2_GET(pack) == (float) -7.214364E37F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p75_z_GET(pack) == (float) -3.7584177E37F);
    assert(p75_param1_GET(pack) == (float)1.1513333E38F);
    assert(p75_y_GET(pack) == (int32_t) -2004701205);
    assert(p75_param3_GET(pack) == (float) -2.5265925E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p75_x_GET(pack) == (int32_t)916018655);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_RELAY);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p75_param4_GET(pack) == (float) -3.2105736E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_START_RX_PAIR);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p76_param2_GET(pack) == (float) -2.2437582E38F);
    assert(p76_param1_GET(pack) == (float) -6.16854E37F);
    assert(p76_param5_GET(pack) == (float)2.807578E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p76_param7_GET(pack) == (float)2.3864723E38F);
    assert(p76_param6_GET(pack) == (float)6.501324E37F);
    assert(p76_param4_GET(pack) == (float) -5.559202E37F);
    assert(p76_param3_GET(pack) == (float)1.1372228E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)153);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t) -622216880);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)124);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)2);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)171);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p81_yaw_GET(pack) == (float)2.0505435E38F);
    assert(p81_thrust_GET(pack) == (float) -3.059072E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)346732401L);
    assert(p81_pitch_GET(pack) == (float)3.1491802E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p81_roll_GET(pack) == (float)1.7530843E38F);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.7977903E38F, 3.157124E38F, -3.0569477E38F, 9.025518E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_roll_rate_GET(pack) == (float)1.9717005E38F);
    assert(p82_thrust_GET(pack) == (float) -2.4591272E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.5995273E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3348316302L);
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.5692799E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float)3.6655717E37F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.2535083E37F);
    assert(p83_body_roll_rate_GET(pack) == (float) -1.5964063E38F);
    {
        float exemplary[] =  {8.3714814E37F, -1.3839322E38F, 2.1511329E38F, -3.3106452E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_pitch_rate_GET(pack) == (float)2.5937085E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)1013579285L);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p84_vz_GET(pack) == (float)1.2724967E37F);
    assert(p84_x_GET(pack) == (float)2.8099092E38F);
    assert(p84_yaw_rate_GET(pack) == (float)2.0444245E38F);
    assert(p84_vy_GET(pack) == (float) -1.9653446E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p84_vx_GET(pack) == (float) -3.3444525E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)39594);
    assert(p84_afz_GET(pack) == (float)1.5313226E38F);
    assert(p84_yaw_GET(pack) == (float) -1.7453833E38F);
    assert(p84_z_GET(pack) == (float) -1.8345414E37F);
    assert(p84_y_GET(pack) == (float) -1.4119158E38F);
    assert(p84_afy_GET(pack) == (float) -1.2519933E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3723016848L);
    assert(p84_afx_GET(pack) == (float)2.3955136E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p86_afx_GET(pack) == (float) -1.83156E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2860700765L);
    assert(p86_yaw_rate_GET(pack) == (float)2.0534605E38F);
    assert(p86_alt_GET(pack) == (float) -2.6305186E38F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1927959094);
    assert(p86_vx_GET(pack) == (float) -2.9169097E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)32079);
    assert(p86_vy_GET(pack) == (float)1.3126874E38F);
    assert(p86_afz_GET(pack) == (float)2.6953945E38F);
    assert(p86_vz_GET(pack) == (float) -2.9608262E38F);
    assert(p86_yaw_GET(pack) == (float)6.531067E37F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p86_lat_int_GET(pack) == (int32_t) -357258785);
    assert(p86_afy_GET(pack) == (float) -1.3162108E38F);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_vz_GET(pack) == (float) -1.0659862E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)283447721);
    assert(p87_yaw_rate_GET(pack) == (float) -3.1254315E38F);
    assert(p87_yaw_GET(pack) == (float)3.2509044E38F);
    assert(p87_alt_GET(pack) == (float) -1.8553971E38F);
    assert(p87_lat_int_GET(pack) == (int32_t)64037149);
    assert(p87_vy_GET(pack) == (float) -8.928258E37F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p87_vx_GET(pack) == (float)8.515631E37F);
    assert(p87_afx_GET(pack) == (float) -9.322101E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)29692);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)3722973485L);
    assert(p87_afy_GET(pack) == (float)7.9995446E36F);
    assert(p87_afz_GET(pack) == (float) -2.9991818E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_z_GET(pack) == (float)1.9170575E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2367025658L);
    assert(p89_yaw_GET(pack) == (float)2.7643186E38F);
    assert(p89_pitch_GET(pack) == (float)1.9429466E37F);
    assert(p89_x_GET(pack) == (float) -1.7460009E38F);
    assert(p89_roll_GET(pack) == (float)2.8697155E38F);
    assert(p89_y_GET(pack) == (float) -8.318043E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)14956);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -14321);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -15346);
    assert(p90_pitchspeed_GET(pack) == (float) -4.2990567E36F);
    assert(p90_roll_GET(pack) == (float) -3.1154694E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -15632);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -9669);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)14552);
    assert(p90_pitch_GET(pack) == (float) -7.3767697E37F);
    assert(p90_lon_GET(pack) == (int32_t)1683014417);
    assert(p90_lat_GET(pack) == (int32_t) -1870703239);
    assert(p90_time_usec_GET(pack) == (uint64_t)6237614395626182240L);
    assert(p90_rollspeed_GET(pack) == (float) -1.1489919E38F);
    assert(p90_yaw_GET(pack) == (float)2.1192041E37F);
    assert(p90_alt_GET(pack) == (int32_t)1134764075);
    assert(p90_yawspeed_GET(pack) == (float) -1.062251E38F);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux3_GET(pack) == (float)2.133539E38F);
    assert(p91_roll_ailerons_GET(pack) == (float) -1.1589304E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -3.2267306E37F);
    assert(p91_aux2_GET(pack) == (float)9.865074E37F);
    assert(p91_throttle_GET(pack) == (float)2.6573046E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)8.942426E37F);
    assert(p91_time_usec_GET(pack) == (uint64_t)1688263909165782869L);
    assert(p91_aux4_GET(pack) == (float) -1.2625034E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
    assert(p91_aux1_GET(pack) == (float) -1.4540211E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)69);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)7728);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)12930);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)2333);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)48191);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)24240);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)57732);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)23682);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)57623);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)52050);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)60403);
    assert(p92_time_usec_GET(pack) == (uint64_t)6380605128291382382L);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)36876);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)40820);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)56);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)5591373957122626965L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    {
        float exemplary[] =  {-6.9457325E34F, 9.588394E37F, -1.6412083E38F, -1.5688698E38F, -2.0872066E37F, -4.215634E37F, 1.9843924E38F, 6.154986E36F, 1.947034E38F, 1.1911214E38F, -2.0842131E38F, 2.622365E38F, -3.3010068E37F, -2.3603376E38F, -1.1928385E38F, -3.1356106E37F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)7538939790698347276L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)12873);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -1633);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.145519E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)2.0379802E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)2.3880548E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p100_ground_distance_GET(pack) == (float) -2.5729028E38F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)2.5538766E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)2876477832941302416L);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_usec_GET(pack) == (uint64_t)5072099899845589435L);
    assert(p101_pitch_GET(pack) == (float)1.1661158E38F);
    assert(p101_z_GET(pack) == (float) -2.8426186E38F);
    assert(p101_x_GET(pack) == (float)1.3041516E37F);
    assert(p101_roll_GET(pack) == (float) -1.5907354E38F);
    assert(p101_y_GET(pack) == (float)1.3845789E38F);
    assert(p101_yaw_GET(pack) == (float) -4.2893443E37F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_yaw_GET(pack) == (float)7.9243273E37F);
    assert(p102_y_GET(pack) == (float)8.1284266E37F);
    assert(p102_roll_GET(pack) == (float)2.595175E38F);
    assert(p102_usec_GET(pack) == (uint64_t)6768382064556339294L);
    assert(p102_pitch_GET(pack) == (float) -1.2704027E38F);
    assert(p102_x_GET(pack) == (float) -2.745278E38F);
    assert(p102_z_GET(pack) == (float)3.0613154E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float) -1.5690088E38F);
    assert(p103_usec_GET(pack) == (uint64_t)4757277920096620815L);
    assert(p103_x_GET(pack) == (float) -2.3506861E37F);
    assert(p103_y_GET(pack) == (float)7.76821E37F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_usec_GET(pack) == (uint64_t)1057352700114596413L);
    assert(p104_x_GET(pack) == (float)2.7020956E37F);
    assert(p104_roll_GET(pack) == (float)1.998713E38F);
    assert(p104_pitch_GET(pack) == (float) -1.321195E38F);
    assert(p104_z_GET(pack) == (float)1.7282014E37F);
    assert(p104_yaw_GET(pack) == (float) -2.5387036E38F);
    assert(p104_y_GET(pack) == (float) -8.783111E37F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)61970);
    assert(p105_zacc_GET(pack) == (float)1.6075153E38F);
    assert(p105_xgyro_GET(pack) == (float)2.8875123E38F);
    assert(p105_xacc_GET(pack) == (float)2.2996701E38F);
    assert(p105_ymag_GET(pack) == (float)6.564841E37F);
    assert(p105_pressure_alt_GET(pack) == (float) -1.4918057E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)164606632381597214L);
    assert(p105_zmag_GET(pack) == (float)1.9945237E38F);
    assert(p105_abs_pressure_GET(pack) == (float)2.017198E38F);
    assert(p105_zgyro_GET(pack) == (float) -2.5111073E38F);
    assert(p105_ygyro_GET(pack) == (float)2.340282E38F);
    assert(p105_temperature_GET(pack) == (float) -8.16845E37F);
    assert(p105_xmag_GET(pack) == (float) -1.0109421E37F);
    assert(p105_diff_pressure_GET(pack) == (float) -5.3251086E37F);
    assert(p105_yacc_GET(pack) == (float)1.9359769E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3718457175L);
    assert(p106_integrated_y_GET(pack) == (float) -3.0598207E38F);
    assert(p106_distance_GET(pack) == (float)1.2486697E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -2.3346915E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1300120916L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -20839);
    assert(p106_integrated_x_GET(pack) == (float)2.1570026E37F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.108388E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p106_time_usec_GET(pack) == (uint64_t)5478967713702435458L);
    assert(p106_integrated_ygyro_GET(pack) == (float)1.6622408E38F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xmag_GET(pack) == (float) -6.28088E37F);
    assert(p107_xacc_GET(pack) == (float) -1.544409E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)7826221670743917278L);
    assert(p107_ygyro_GET(pack) == (float) -2.8887137E38F);
    assert(p107_zgyro_GET(pack) == (float)2.9397894E38F);
    assert(p107_zmag_GET(pack) == (float)9.455484E37F);
    assert(p107_ymag_GET(pack) == (float) -1.9506064E38F);
    assert(p107_yacc_GET(pack) == (float) -2.0933402E38F);
    assert(p107_zacc_GET(pack) == (float)2.8934886E38F);
    assert(p107_temperature_GET(pack) == (float) -6.8936954E37F);
    assert(p107_xgyro_GET(pack) == (float) -1.3944457E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -2.9189357E38F);
    assert(p107_pressure_alt_GET(pack) == (float)5.942866E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2518383931L);
    assert(p107_diff_pressure_GET(pack) == (float) -3.3190026E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_lon_GET(pack) == (float) -3.344497E38F);
    assert(p108_zacc_GET(pack) == (float) -1.2597261E38F);
    assert(p108_lat_GET(pack) == (float) -2.5837236E38F);
    assert(p108_alt_GET(pack) == (float) -1.7326378E38F);
    assert(p108_vd_GET(pack) == (float) -1.6003793E38F);
    assert(p108_q3_GET(pack) == (float) -8.327572E37F);
    assert(p108_q2_GET(pack) == (float)1.7836552E38F);
    assert(p108_ve_GET(pack) == (float)2.841509E38F);
    assert(p108_xgyro_GET(pack) == (float)3.0570045E38F);
    assert(p108_xacc_GET(pack) == (float)6.31271E37F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.579776E38F);
    assert(p108_zgyro_GET(pack) == (float) -2.3790303E38F);
    assert(p108_ygyro_GET(pack) == (float)3.310029E38F);
    assert(p108_roll_GET(pack) == (float) -1.9949555E38F);
    assert(p108_q4_GET(pack) == (float) -2.4594424E38F);
    assert(p108_yacc_GET(pack) == (float) -1.8147777E38F);
    assert(p108_vn_GET(pack) == (float) -1.783158E38F);
    assert(p108_yaw_GET(pack) == (float)3.0329727E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -2.101267E38F);
    assert(p108_pitch_GET(pack) == (float) -2.6902961E38F);
    assert(p108_q1_GET(pack) == (float) -4.434584E37F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)18774);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)58370);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)51, (uint8_t)203, (uint8_t)72, (uint8_t)186, (uint8_t)158, (uint8_t)121, (uint8_t)166, (uint8_t)91, (uint8_t)68, (uint8_t)137, (uint8_t)129, (uint8_t)252, (uint8_t)244, (uint8_t)67, (uint8_t)248, (uint8_t)223, (uint8_t)13, (uint8_t)165, (uint8_t)156, (uint8_t)118, (uint8_t)61, (uint8_t)98, (uint8_t)85, (uint8_t)22, (uint8_t)233, (uint8_t)10, (uint8_t)240, (uint8_t)84, (uint8_t)203, (uint8_t)33, (uint8_t)200, (uint8_t)99, (uint8_t)101, (uint8_t)254, (uint8_t)12, (uint8_t)110, (uint8_t)187, (uint8_t)244, (uint8_t)100, (uint8_t)101, (uint8_t)161, (uint8_t)181, (uint8_t)175, (uint8_t)172, (uint8_t)48, (uint8_t)85, (uint8_t)131, (uint8_t)192, (uint8_t)91, (uint8_t)214, (uint8_t)49, (uint8_t)241, (uint8_t)9, (uint8_t)90, (uint8_t)249, (uint8_t)240, (uint8_t)38, (uint8_t)7, (uint8_t)170, (uint8_t)95, (uint8_t)122, (uint8_t)253, (uint8_t)154, (uint8_t)223, (uint8_t)214, (uint8_t)206, (uint8_t)136, (uint8_t)86, (uint8_t)173, (uint8_t)129, (uint8_t)186, (uint8_t)211, (uint8_t)45, (uint8_t)103, (uint8_t)142, (uint8_t)118, (uint8_t)139, (uint8_t)28, (uint8_t)69, (uint8_t)130, (uint8_t)138, (uint8_t)246, (uint8_t)56, (uint8_t)143, (uint8_t)190, (uint8_t)99, (uint8_t)85, (uint8_t)181, (uint8_t)152, (uint8_t)94, (uint8_t)90, (uint8_t)131, (uint8_t)59, (uint8_t)112, (uint8_t)81, (uint8_t)38, (uint8_t)74, (uint8_t)71, (uint8_t)171, (uint8_t)135, (uint8_t)180, (uint8_t)234, (uint8_t)181, (uint8_t)153, (uint8_t)104, (uint8_t)232, (uint8_t)17, (uint8_t)70, (uint8_t)178, (uint8_t)109, (uint8_t)169, (uint8_t)88, (uint8_t)184, (uint8_t)207, (uint8_t)152, (uint8_t)47, (uint8_t)185, (uint8_t)236, (uint8_t)26, (uint8_t)171, (uint8_t)53, (uint8_t)75, (uint8_t)106, (uint8_t)209, (uint8_t)36, (uint8_t)222, (uint8_t)97, (uint8_t)57, (uint8_t)83, (uint8_t)140, (uint8_t)134, (uint8_t)154, (uint8_t)34, (uint8_t)77, (uint8_t)183, (uint8_t)73, (uint8_t)233, (uint8_t)242, (uint8_t)43, (uint8_t)253, (uint8_t)219, (uint8_t)253, (uint8_t)88, (uint8_t)27, (uint8_t)142, (uint8_t)163, (uint8_t)79, (uint8_t)226, (uint8_t)82, (uint8_t)129, (uint8_t)215, (uint8_t)191, (uint8_t)104, (uint8_t)64, (uint8_t)111, (uint8_t)122, (uint8_t)140, (uint8_t)121, (uint8_t)182, (uint8_t)130, (uint8_t)17, (uint8_t)19, (uint8_t)162, (uint8_t)125, (uint8_t)147, (uint8_t)14, (uint8_t)118, (uint8_t)138, (uint8_t)229, (uint8_t)102, (uint8_t)158, (uint8_t)130, (uint8_t)107, (uint8_t)76, (uint8_t)91, (uint8_t)86, (uint8_t)54, (uint8_t)228, (uint8_t)162, (uint8_t)55, (uint8_t)87, (uint8_t)24, (uint8_t)12, (uint8_t)26, (uint8_t)132, (uint8_t)17, (uint8_t)190, (uint8_t)227, (uint8_t)41, (uint8_t)254, (uint8_t)150, (uint8_t)222, (uint8_t)185, (uint8_t)37, (uint8_t)6, (uint8_t)77, (uint8_t)131, (uint8_t)104, (uint8_t)124, (uint8_t)236, (uint8_t)177, (uint8_t)66, (uint8_t)129, (uint8_t)14, (uint8_t)180, (uint8_t)17, (uint8_t)110, (uint8_t)96, (uint8_t)134, (uint8_t)188, (uint8_t)7, (uint8_t)141, (uint8_t)244, (uint8_t)42, (uint8_t)231, (uint8_t)55, (uint8_t)110, (uint8_t)232, (uint8_t)114, (uint8_t)172, (uint8_t)67, (uint8_t)118, (uint8_t)189, (uint8_t)147, (uint8_t)163, (uint8_t)152, (uint8_t)68, (uint8_t)108, (uint8_t)60, (uint8_t)157, (uint8_t)109, (uint8_t)83, (uint8_t)237, (uint8_t)142, (uint8_t)226, (uint8_t)95, (uint8_t)115, (uint8_t)166, (uint8_t)117, (uint8_t)104, (uint8_t)253, (uint8_t)244, (uint8_t)203, (uint8_t)237, (uint8_t)62, (uint8_t)58, (uint8_t)7, (uint8_t)143, (uint8_t)30, (uint8_t)18} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)35);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t)4400939869508876977L);
    assert(p111_tc1_GET(pack) == (int64_t)3391325052206998022L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)837452302L);
    assert(p112_time_usec_GET(pack) == (uint64_t)68510322077938214L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -25020);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)50959);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)3347);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -25746);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)27091);
    assert(p113_lat_GET(pack) == (int32_t) -1417756295);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p113_lon_GET(pack) == (int32_t)1782107865);
    assert(p113_alt_GET(pack) == (int32_t)673712895);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)21190);
    assert(p113_time_usec_GET(pack) == (uint64_t)2544601951611260644L);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)53096);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_zgyro_GET(pack) == (float)1.3367782E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float)2.8032574E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3197318107L);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -25447);
    assert(p114_integrated_x_GET(pack) == (float) -2.7142904E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float)1.0876661E38F);
    assert(p114_integrated_y_GET(pack) == (float)2.824809E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1344308918L);
    assert(p114_distance_GET(pack) == (float) -2.1574509E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)7798136658250401868L);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -22941);
    assert(p115_rollspeed_GET(pack) == (float)1.9170913E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)29804);
    assert(p115_alt_GET(pack) == (int32_t) -1548855636);
    assert(p115_yawspeed_GET(pack) == (float)2.1577567E38F);
    assert(p115_lat_GET(pack) == (int32_t) -1062434343);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)8943);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)9695);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -18829);
    assert(p115_time_usec_GET(pack) == (uint64_t)4188623246934525087L);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)57598);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)31169);
    {
        float exemplary[] =  {1.9065962E38F, -2.9552718E38F, 5.0015063E37F, 2.4556554E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_lon_GET(pack) == (int32_t) -1461145352);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -1181);
    assert(p115_pitchspeed_GET(pack) == (float)3.30891E38F);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -26555);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -7796);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -2278);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)1500903903L);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)19152);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)17311);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)4114);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -26646);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -7438);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -30084);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)10334);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)27631);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)3001);
    assert(p118_time_utc_GET(pack) == (uint32_t)729912963L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)6686);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)64868);
    assert(p118_size_GET(pack) == (uint32_t)4240443197L);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)6130);
    assert(p119_ofs_GET(pack) == (uint32_t)2169406938L);
    assert(p119_count_GET(pack) == (uint32_t)2192094089L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)45076);
    assert(p120_ofs_GET(pack) == (uint32_t)3456955194L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)21);
    {
        uint8_t exemplary[] =  {(uint8_t)164, (uint8_t)155, (uint8_t)184, (uint8_t)202, (uint8_t)25, (uint8_t)15, (uint8_t)128, (uint8_t)20, (uint8_t)76, (uint8_t)208, (uint8_t)47, (uint8_t)238, (uint8_t)205, (uint8_t)110, (uint8_t)165, (uint8_t)156, (uint8_t)110, (uint8_t)218, (uint8_t)167, (uint8_t)205, (uint8_t)14, (uint8_t)2, (uint8_t)192, (uint8_t)246, (uint8_t)115, (uint8_t)155, (uint8_t)126, (uint8_t)5, (uint8_t)30, (uint8_t)234, (uint8_t)134, (uint8_t)183, (uint8_t)4, (uint8_t)228, (uint8_t)32, (uint8_t)65, (uint8_t)131, (uint8_t)218, (uint8_t)68, (uint8_t)249, (uint8_t)47, (uint8_t)248, (uint8_t)28, (uint8_t)176, (uint8_t)120, (uint8_t)164, (uint8_t)42, (uint8_t)214, (uint8_t)192, (uint8_t)159, (uint8_t)173, (uint8_t)0, (uint8_t)171, (uint8_t)196, (uint8_t)227, (uint8_t)42, (uint8_t)65, (uint8_t)167, (uint8_t)30, (uint8_t)108, (uint8_t)189, (uint8_t)25, (uint8_t)232, (uint8_t)247, (uint8_t)94, (uint8_t)210, (uint8_t)66, (uint8_t)151, (uint8_t)12, (uint8_t)10, (uint8_t)171, (uint8_t)180, (uint8_t)55, (uint8_t)223, (uint8_t)125, (uint8_t)61, (uint8_t)137, (uint8_t)197, (uint8_t)237, (uint8_t)235, (uint8_t)56, (uint8_t)98, (uint8_t)65, (uint8_t)195, (uint8_t)38, (uint8_t)234, (uint8_t)41, (uint8_t)9, (uint8_t)126, (uint8_t)41} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)25);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)193);
    {
        uint8_t exemplary[] =  {(uint8_t)252, (uint8_t)15, (uint8_t)23, (uint8_t)207, (uint8_t)20, (uint8_t)46, (uint8_t)46, (uint8_t)110, (uint8_t)156, (uint8_t)194, (uint8_t)135, (uint8_t)93, (uint8_t)149, (uint8_t)141, (uint8_t)69, (uint8_t)25, (uint8_t)107, (uint8_t)207, (uint8_t)41, (uint8_t)65, (uint8_t)9, (uint8_t)154, (uint8_t)152, (uint8_t)134, (uint8_t)106, (uint8_t)121, (uint8_t)234, (uint8_t)210, (uint8_t)98, (uint8_t)246, (uint8_t)209, (uint8_t)168, (uint8_t)134, (uint8_t)4, (uint8_t)68, (uint8_t)48, (uint8_t)222, (uint8_t)142, (uint8_t)76, (uint8_t)95, (uint8_t)140, (uint8_t)66, (uint8_t)242, (uint8_t)167, (uint8_t)97, (uint8_t)112, (uint8_t)198, (uint8_t)15, (uint8_t)101, (uint8_t)97, (uint8_t)92, (uint8_t)37, (uint8_t)222, (uint8_t)163, (uint8_t)158, (uint8_t)230, (uint8_t)95, (uint8_t)18, (uint8_t)231, (uint8_t)227, (uint8_t)90, (uint8_t)233, (uint8_t)92, (uint8_t)75, (uint8_t)55, (uint8_t)232, (uint8_t)48, (uint8_t)42, (uint8_t)38, (uint8_t)33, (uint8_t)171, (uint8_t)164, (uint8_t)18, (uint8_t)188, (uint8_t)225, (uint8_t)106, (uint8_t)141, (uint8_t)254, (uint8_t)61, (uint8_t)106, (uint8_t)227, (uint8_t)132, (uint8_t)203, (uint8_t)137, (uint8_t)82, (uint8_t)166, (uint8_t)120, (uint8_t)200, (uint8_t)61, (uint8_t)181, (uint8_t)194, (uint8_t)148, (uint8_t)125, (uint8_t)91, (uint8_t)63, (uint8_t)131, (uint8_t)8, (uint8_t)137, (uint8_t)13, (uint8_t)247, (uint8_t)48, (uint8_t)212, (uint8_t)152, (uint8_t)85, (uint8_t)9, (uint8_t)213, (uint8_t)232, (uint8_t)73, (uint8_t)105, (uint8_t)10} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_dgps_age_GET(pack) == (uint32_t)3938637533L);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p124_lon_GET(pack) == (int32_t)788441707);
    assert(p124_lat_GET(pack) == (int32_t)1945479960);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)27268);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)20889);
    assert(p124_time_usec_GET(pack) == (uint64_t)1848076250824157886L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)52076);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)11591);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p124_alt_GET(pack) == (int32_t) -2066340423);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)28721);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)44899);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    {
        uint8_t exemplary[] =  {(uint8_t)163, (uint8_t)94, (uint8_t)210, (uint8_t)91, (uint8_t)78, (uint8_t)23, (uint8_t)151, (uint8_t)216, (uint8_t)79, (uint8_t)244, (uint8_t)25, (uint8_t)83, (uint8_t)218, (uint8_t)239, (uint8_t)14, (uint8_t)31, (uint8_t)225, (uint8_t)132, (uint8_t)40, (uint8_t)176, (uint8_t)29, (uint8_t)35, (uint8_t)96, (uint8_t)216, (uint8_t)204, (uint8_t)53, (uint8_t)249, (uint8_t)4, (uint8_t)58, (uint8_t)115, (uint8_t)84, (uint8_t)35, (uint8_t)5, (uint8_t)160, (uint8_t)54, (uint8_t)185, (uint8_t)16, (uint8_t)8, (uint8_t)43, (uint8_t)69, (uint8_t)113, (uint8_t)60, (uint8_t)250, (uint8_t)81, (uint8_t)250, (uint8_t)219, (uint8_t)129, (uint8_t)31, (uint8_t)56, (uint8_t)23, (uint8_t)253, (uint8_t)32, (uint8_t)160, (uint8_t)91, (uint8_t)147, (uint8_t)157, (uint8_t)184, (uint8_t)191, (uint8_t)119, (uint8_t)141, (uint8_t)24, (uint8_t)92, (uint8_t)134, (uint8_t)186, (uint8_t)115, (uint8_t)217, (uint8_t)126, (uint8_t)154, (uint8_t)141, (uint8_t)156} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)44561);
    assert(p126_baudrate_GET(pack) == (uint32_t)2935628589L);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1645021295);
    assert(p127_accuracy_GET(pack) == (uint32_t)1132530282L);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)50887);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -2136166562);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)1489766);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1057323788L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p127_tow_GET(pack) == (uint32_t)2087229437L);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1885870731);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)182);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)562962203);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)12004);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p128_accuracy_GET(pack) == (uint32_t)3730047014L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -110320269);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)1909077877);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4202056894L);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)313048227);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p128_tow_GET(pack) == (uint32_t)2838881581L);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)25676);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)1623);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -18543);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)9123);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -572);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)2868);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -13164);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -5606);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)15768);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1799482499L);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p130_size_GET(pack) == (uint32_t)2301328077L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)54471);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)54648);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)26132);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)18, (uint8_t)58, (uint8_t)133, (uint8_t)55, (uint8_t)108, (uint8_t)168, (uint8_t)46, (uint8_t)22, (uint8_t)176, (uint8_t)227, (uint8_t)161, (uint8_t)237, (uint8_t)218, (uint8_t)218, (uint8_t)53, (uint8_t)24, (uint8_t)136, (uint8_t)83, (uint8_t)20, (uint8_t)188, (uint8_t)199, (uint8_t)14, (uint8_t)132, (uint8_t)3, (uint8_t)109, (uint8_t)114, (uint8_t)43, (uint8_t)119, (uint8_t)126, (uint8_t)146, (uint8_t)193, (uint8_t)225, (uint8_t)201, (uint8_t)8, (uint8_t)121, (uint8_t)58, (uint8_t)226, (uint8_t)140, (uint8_t)106, (uint8_t)4, (uint8_t)78, (uint8_t)228, (uint8_t)145, (uint8_t)97, (uint8_t)60, (uint8_t)223, (uint8_t)213, (uint8_t)33, (uint8_t)125, (uint8_t)43, (uint8_t)143, (uint8_t)23, (uint8_t)78, (uint8_t)43, (uint8_t)149, (uint8_t)122, (uint8_t)32, (uint8_t)136, (uint8_t)105, (uint8_t)81, (uint8_t)162, (uint8_t)76, (uint8_t)137, (uint8_t)245, (uint8_t)6, (uint8_t)206, (uint8_t)247, (uint8_t)158, (uint8_t)228, (uint8_t)238, (uint8_t)245, (uint8_t)103, (uint8_t)100, (uint8_t)209, (uint8_t)32, (uint8_t)200, (uint8_t)244, (uint8_t)5, (uint8_t)211, (uint8_t)19, (uint8_t)134, (uint8_t)216, (uint8_t)246, (uint8_t)253, (uint8_t)105, (uint8_t)32, (uint8_t)31, (uint8_t)58, (uint8_t)135, (uint8_t)54, (uint8_t)127, (uint8_t)251, (uint8_t)183, (uint8_t)228, (uint8_t)154, (uint8_t)126, (uint8_t)176, (uint8_t)53, (uint8_t)183, (uint8_t)155, (uint8_t)50, (uint8_t)105, (uint8_t)216, (uint8_t)137, (uint8_t)11, (uint8_t)43, (uint8_t)155, (uint8_t)45, (uint8_t)152, (uint8_t)127, (uint8_t)97, (uint8_t)210, (uint8_t)167, (uint8_t)195, (uint8_t)171, (uint8_t)123, (uint8_t)196, (uint8_t)211, (uint8_t)163, (uint8_t)11, (uint8_t)54, (uint8_t)61, (uint8_t)205, (uint8_t)202, (uint8_t)227, (uint8_t)42, (uint8_t)144, (uint8_t)222, (uint8_t)57, (uint8_t)32, (uint8_t)56, (uint8_t)76, (uint8_t)224, (uint8_t)7, (uint8_t)79, (uint8_t)158, (uint8_t)52, (uint8_t)56, (uint8_t)45, (uint8_t)129, (uint8_t)86, (uint8_t)148, (uint8_t)10, (uint8_t)35, (uint8_t)183, (uint8_t)237, (uint8_t)159, (uint8_t)173, (uint8_t)35, (uint8_t)35, (uint8_t)39, (uint8_t)67, (uint8_t)133, (uint8_t)226, (uint8_t)127, (uint8_t)135, (uint8_t)216, (uint8_t)179, (uint8_t)55, (uint8_t)238, (uint8_t)153, (uint8_t)253, (uint8_t)63, (uint8_t)57, (uint8_t)234, (uint8_t)202, (uint8_t)51, (uint8_t)105, (uint8_t)46, (uint8_t)201, (uint8_t)96, (uint8_t)190, (uint8_t)127, (uint8_t)67, (uint8_t)77, (uint8_t)250, (uint8_t)255, (uint8_t)59, (uint8_t)83, (uint8_t)188, (uint8_t)94, (uint8_t)109, (uint8_t)167, (uint8_t)211, (uint8_t)4, (uint8_t)255, (uint8_t)3, (uint8_t)215, (uint8_t)235, (uint8_t)7, (uint8_t)141, (uint8_t)232, (uint8_t)98, (uint8_t)201, (uint8_t)213, (uint8_t)247, (uint8_t)160, (uint8_t)186, (uint8_t)151, (uint8_t)214, (uint8_t)199, (uint8_t)149, (uint8_t)249, (uint8_t)149, (uint8_t)186, (uint8_t)253, (uint8_t)243, (uint8_t)195, (uint8_t)188, (uint8_t)34, (uint8_t)71, (uint8_t)250, (uint8_t)70, (uint8_t)145, (uint8_t)32, (uint8_t)164, (uint8_t)95, (uint8_t)140, (uint8_t)80, (uint8_t)76, (uint8_t)136, (uint8_t)245, (uint8_t)139, (uint8_t)190, (uint8_t)133, (uint8_t)146, (uint8_t)124, (uint8_t)89, (uint8_t)131, (uint8_t)170, (uint8_t)142, (uint8_t)98, (uint8_t)203, (uint8_t)104, (uint8_t)58, (uint8_t)232, (uint8_t)55, (uint8_t)14, (uint8_t)57, (uint8_t)226, (uint8_t)226, (uint8_t)253, (uint8_t)118, (uint8_t)180, (uint8_t)236, (uint8_t)248, (uint8_t)39, (uint8_t)38, (uint8_t)100, (uint8_t)85, (uint8_t)234, (uint8_t)169, (uint8_t)218} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)20800);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)4677);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)55116);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)30892);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1076379289L);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)36429);
    assert(p133_lat_GET(pack) == (int32_t)111543362);
    assert(p133_lon_GET(pack) == (int32_t) -1191283920);
    assert(p133_mask_GET(pack) == (uint64_t)757765720942836527L);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)8767);
    assert(p134_lon_GET(pack) == (int32_t) -1230306497);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p134_lat_GET(pack) == (int32_t) -1904838548);
    {
        int16_t exemplary[] =  {(int16_t)9679, (int16_t)27335, (int16_t) -13923, (int16_t)4770, (int16_t) -20002, (int16_t)22873, (int16_t) -26869, (int16_t)5644, (int16_t) -11268, (int16_t)4336, (int16_t)14078, (int16_t)12847, (int16_t)19127, (int16_t)8190, (int16_t)29455, (int16_t) -7937} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t) -1695931072);
    assert(p135_lon_GET(pack) == (int32_t)2073044756);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)1049);
    assert(p136_lat_GET(pack) == (int32_t) -2086115371);
    assert(p136_lon_GET(pack) == (int32_t) -195234269);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)19048);
    assert(p136_current_height_GET(pack) == (float) -1.858375E38F);
    assert(p136_terrain_height_GET(pack) == (float) -2.9021755E38F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)61538);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)1430890528L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)26419);
    assert(p137_press_abs_GET(pack) == (float) -1.3592657E38F);
    assert(p137_press_diff_GET(pack) == (float)9.045696E37F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_y_GET(pack) == (float) -2.8312932E38F);
    {
        float exemplary[] =  {2.1926958E38F, -2.5558045E38F, -1.0133807E38F, -1.764377E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float) -3.2886714E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)7626421284007542195L);
    assert(p138_z_GET(pack) == (float)5.1526492E36F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)83);
    {
        float exemplary[] =  {-2.4449668E38F, -2.9697409E38F, -2.2237283E38F, -7.00052E37F, -7.311077E37F, -1.6090775E37F, -7.3563605E37F, -9.8801924E36F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p139_time_usec_GET(pack) == (uint64_t)4764698875336345703L);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.285983E38F, 3.0769239E38F, 1.2346158E38F, -1.4833366E38F, 4.9829456E36F, 3.213132E38F, -7.071099E37F, 2.3048038E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)6627060401212112667L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)219);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float) -3.2900285E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)8.0489135E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)285187208009281337L);
    assert(p141_altitude_local_GET(pack) == (float)2.947392E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -8.794429E37F);
    assert(p141_altitude_terrain_GET(pack) == (float) -1.2572677E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -1.4610955E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)0, (uint8_t)152, (uint8_t)227, (uint8_t)81, (uint8_t)99, (uint8_t)204, (uint8_t)23, (uint8_t)201, (uint8_t)74, (uint8_t)47, (uint8_t)141, (uint8_t)66, (uint8_t)75, (uint8_t)28, (uint8_t)100, (uint8_t)114, (uint8_t)223, (uint8_t)19, (uint8_t)27, (uint8_t)194, (uint8_t)202, (uint8_t)100, (uint8_t)178, (uint8_t)248, (uint8_t)23, (uint8_t)82, (uint8_t)229, (uint8_t)130, (uint8_t)79, (uint8_t)74, (uint8_t)28, (uint8_t)253, (uint8_t)32, (uint8_t)245, (uint8_t)190, (uint8_t)55, (uint8_t)173, (uint8_t)108, (uint8_t)88, (uint8_t)193, (uint8_t)213, (uint8_t)107, (uint8_t)16, (uint8_t)47, (uint8_t)167, (uint8_t)163, (uint8_t)100, (uint8_t)184, (uint8_t)20, (uint8_t)34, (uint8_t)64, (uint8_t)238, (uint8_t)65, (uint8_t)144, (uint8_t)225, (uint8_t)101, (uint8_t)85, (uint8_t)5, (uint8_t)245, (uint8_t)229, (uint8_t)234, (uint8_t)208, (uint8_t)73, (uint8_t)37, (uint8_t)169, (uint8_t)175, (uint8_t)190, (uint8_t)16, (uint8_t)152, (uint8_t)101, (uint8_t)110, (uint8_t)223, (uint8_t)241, (uint8_t)109, (uint8_t)211, (uint8_t)210, (uint8_t)50, (uint8_t)150, (uint8_t)186, (uint8_t)190, (uint8_t)139, (uint8_t)1, (uint8_t)114, (uint8_t)247, (uint8_t)126, (uint8_t)189, (uint8_t)180, (uint8_t)84, (uint8_t)81, (uint8_t)210, (uint8_t)17, (uint8_t)173, (uint8_t)58, (uint8_t)164, (uint8_t)46, (uint8_t)203, (uint8_t)53, (uint8_t)100, (uint8_t)113, (uint8_t)72, (uint8_t)252, (uint8_t)125, (uint8_t)114, (uint8_t)110, (uint8_t)42, (uint8_t)167, (uint8_t)98, (uint8_t)44, (uint8_t)108, (uint8_t)214, (uint8_t)222, (uint8_t)54, (uint8_t)52, (uint8_t)53, (uint8_t)89, (uint8_t)28, (uint8_t)146, (uint8_t)252, (uint8_t)56, (uint8_t)210} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)68, (uint8_t)247, (uint8_t)194, (uint8_t)223, (uint8_t)62, (uint8_t)55, (uint8_t)25, (uint8_t)5, (uint8_t)111, (uint8_t)0, (uint8_t)28, (uint8_t)254, (uint8_t)3, (uint8_t)50, (uint8_t)134, (uint8_t)244, (uint8_t)186, (uint8_t)163, (uint8_t)62, (uint8_t)164, (uint8_t)115, (uint8_t)96, (uint8_t)46, (uint8_t)107, (uint8_t)219, (uint8_t)97, (uint8_t)60, (uint8_t)82, (uint8_t)111, (uint8_t)136, (uint8_t)50, (uint8_t)148, (uint8_t)34, (uint8_t)94, (uint8_t)82, (uint8_t)103, (uint8_t)9, (uint8_t)141, (uint8_t)172, (uint8_t)247, (uint8_t)109, (uint8_t)232, (uint8_t)20, (uint8_t)157, (uint8_t)112, (uint8_t)160, (uint8_t)60, (uint8_t)19, (uint8_t)220, (uint8_t)3, (uint8_t)114, (uint8_t)213, (uint8_t)27, (uint8_t)36, (uint8_t)235, (uint8_t)62, (uint8_t)191, (uint8_t)251, (uint8_t)57, (uint8_t)93, (uint8_t)134, (uint8_t)24, (uint8_t)0, (uint8_t)141, (uint8_t)156, (uint8_t)236, (uint8_t)37, (uint8_t)155, (uint8_t)245, (uint8_t)26, (uint8_t)254, (uint8_t)141, (uint8_t)215, (uint8_t)84, (uint8_t)204, (uint8_t)237, (uint8_t)109, (uint8_t)15, (uint8_t)16, (uint8_t)214, (uint8_t)116, (uint8_t)15, (uint8_t)203, (uint8_t)46, (uint8_t)84, (uint8_t)7, (uint8_t)48, (uint8_t)238, (uint8_t)167, (uint8_t)178, (uint8_t)133, (uint8_t)99, (uint8_t)9, (uint8_t)177, (uint8_t)35, (uint8_t)61, (uint8_t)139, (uint8_t)208, (uint8_t)174, (uint8_t)160, (uint8_t)56, (uint8_t)50, (uint8_t)108, (uint8_t)106, (uint8_t)41, (uint8_t)196, (uint8_t)118, (uint8_t)37, (uint8_t)58, (uint8_t)125, (uint8_t)28, (uint8_t)83, (uint8_t)58, (uint8_t)116, (uint8_t)26, (uint8_t)115, (uint8_t)103, (uint8_t)157, (uint8_t)33} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_diff_GET(pack) == (float)1.5811225E38F);
    assert(p143_press_abs_GET(pack) == (float)1.7287671E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -2371);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3192136812L);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.6399637E38F, -7.5049357E37F, 2.9603002E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1153132091);
    {
        float exemplary[] =  {2.066759E38F, -3.308082E38F, -8.649907E37F, 2.012091E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)1642110997);
    {
        float exemplary[] =  {2.2424849E38F, -1.1978608E38F, 2.415125E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)8143407707634026774L);
    {
        float exemplary[] =  {5.468965E36F, 1.8181203E38F, 3.8270519E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)240);
    {
        float exemplary[] =  {-2.3655427E38F, -3.3705716E38F, -1.5373522E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -1.6735704E38F);
    assert(p144_timestamp_GET(pack) == (uint64_t)7350658214342341882L);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_roll_rate_GET(pack) == (float) -2.2437304E38F);
    assert(p146_x_vel_GET(pack) == (float)1.4165854E38F);
    assert(p146_airspeed_GET(pack) == (float)2.3333565E38F);
    assert(p146_yaw_rate_GET(pack) == (float)1.3707618E37F);
    assert(p146_y_vel_GET(pack) == (float) -2.956488E38F);
    {
        float exemplary[] =  {1.9789517E38F, -2.7346373E38F, -9.384848E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_pos_GET(pack) == (float)7.4672936E36F);
    assert(p146_time_usec_GET(pack) == (uint64_t)8348686574283279071L);
    assert(p146_y_acc_GET(pack) == (float) -2.4173164E38F);
    assert(p146_z_pos_GET(pack) == (float) -3.1330713E38F);
    assert(p146_y_pos_GET(pack) == (float)2.406145E38F);
    {
        float exemplary[] =  {2.1570026E38F, -7.9232914E37F, 4.079743E37F, 7.4501393E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float) -1.6991726E38F);
    {
        float exemplary[] =  {-1.3197985E37F, 2.6671412E37F, -2.9831611E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_vel_GET(pack) == (float)8.542802E37F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.8484385E38F);
    assert(p146_x_acc_GET(pack) == (float) -3.1190225E38F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_consumed_GET(pack) == (int32_t) -1581397532);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -8515);
    assert(p147_energy_consumed_GET(pack) == (int32_t)735872958);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -32669);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -74);
    {
        uint16_t exemplary[] =  {(uint16_t)31965, (uint16_t)2872, (uint16_t)54451, (uint16_t)2002, (uint16_t)15879, (uint16_t)51530, (uint16_t)10449, (uint16_t)8495, (uint16_t)10138, (uint16_t)14146} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)26);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_board_version_GET(pack) == (uint32_t)2148673780L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2246494267L);
    {
        uint8_t exemplary[] =  {(uint8_t)211, (uint8_t)144, (uint8_t)217, (uint8_t)153, (uint8_t)85, (uint8_t)232, (uint8_t)246, (uint8_t)186} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)600);
    assert(p148_uid_GET(pack) == (uint64_t)1417252784304658752L);
    {
        uint8_t exemplary[] =  {(uint8_t)110, (uint8_t)218, (uint8_t)28, (uint8_t)135, (uint8_t)75, (uint8_t)195, (uint8_t)32, (uint8_t)91, (uint8_t)43, (uint8_t)191, (uint8_t)22, (uint8_t)197, (uint8_t)223, (uint8_t)144, (uint8_t)128, (uint8_t)0, (uint8_t)218, (uint8_t)90} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)40110);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)252184840L);
    {
        uint8_t exemplary[] =  {(uint8_t)182, (uint8_t)251, (uint8_t)147, (uint8_t)31, (uint8_t)156, (uint8_t)130, (uint8_t)116, (uint8_t)16} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3000371771L);
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)249, (uint8_t)154, (uint8_t)134, (uint8_t)64, (uint8_t)76, (uint8_t)231, (uint8_t)251} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)166);
    {
        float exemplary[] =  {9.511704E37F, -7.194361E37F, -9.826633E37F, 1.6267844E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_x_TRY(ph) == (float) -2.8514214E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_y_TRY(ph) == (float) -1.387668E37F);
    assert(p149_angle_y_GET(pack) == (float)2.2505696E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)5076760438945755179L);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p149_distance_GET(pack) == (float) -1.9240638E38F);
    assert(p149_z_TRY(ph) == (float)2.082231E38F);
    assert(p149_size_x_GET(pack) == (float)6.1662926E37F);
    assert(p149_size_y_GET(pack) == (float)2.9775115E38F);
    assert(p149_angle_x_GET(pack) == (float) -2.3783097E38F);
};


void c_LoopBackDemoChannel_on_SCRIPT_ITEM_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p180_name_LEN(ph) == 32);
    {
        char16_t * exemplary = u"ftTgmjdjlpiszpfbgjgBxbxnTjaaQplo";
        char16_t * sample = p180_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p180_seq_GET(pack) == (uint16_t)(uint16_t)35282);
    assert(p180_target_component_GET(pack) == (uint8_t)(uint8_t)231);
};


void c_LoopBackDemoChannel_on_SCRIPT_REQUEST_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_seq_GET(pack) == (uint16_t)(uint16_t)31441);
    assert(p181_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p181_target_system_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_LoopBackDemoChannel_on_SCRIPT_REQUEST_LIST_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p182_target_system_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_LoopBackDemoChannel_on_SCRIPT_COUNT_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p183_count_GET(pack) == (uint16_t)(uint16_t)57196);
};


void c_LoopBackDemoChannel_on_SCRIPT_CURRENT_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_seq_GET(pack) == (uint16_t)(uint16_t)26780);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.4508246E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)1662667736514506330L);
    assert(p230_hagl_ratio_GET(pack) == (float)8.480281E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.2325635E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -4.1036943E37F);
    assert(p230_mag_ratio_GET(pack) == (float)2.41557E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.9911634E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)1.3342913E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -1.2830937E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float) -7.4296E37F);
    assert(p231_wind_z_GET(pack) == (float)3.7594765E37F);
    assert(p231_wind_y_GET(pack) == (float)2.870686E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)7182055481447931563L);
    assert(p231_horiz_accuracy_GET(pack) == (float)3.2084892E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -2.3929065E38F);
    assert(p231_var_vert_GET(pack) == (float)3.2111636E38F);
    assert(p231_wind_x_GET(pack) == (float) -3.2916544E38F);
    assert(p231_var_horiz_GET(pack) == (float) -1.55098E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_usec_GET(pack) == (uint64_t)7190222732715917819L);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP);
    assert(p232_vn_GET(pack) == (float) -6.517405E37F);
    assert(p232_hdop_GET(pack) == (float)2.2049202E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)152474736L);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p232_lat_GET(pack) == (int32_t) -791170523);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)11753);
    assert(p232_horiz_accuracy_GET(pack) == (float)5.9054107E37F);
    assert(p232_vert_accuracy_GET(pack) == (float)2.7895288E38F);
    assert(p232_vdop_GET(pack) == (float) -2.5852927E38F);
    assert(p232_speed_accuracy_GET(pack) == (float)2.1962028E38F);
    assert(p232_alt_GET(pack) == (float) -2.127398E38F);
    assert(p232_lon_GET(pack) == (int32_t) -666836380);
    assert(p232_vd_GET(pack) == (float) -1.0264464E38F);
    assert(p232_ve_GET(pack) == (float) -9.424775E37F);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)72);
    {
        uint8_t exemplary[] =  {(uint8_t)132, (uint8_t)192, (uint8_t)144, (uint8_t)173, (uint8_t)103, (uint8_t)199, (uint8_t)41, (uint8_t)57, (uint8_t)170, (uint8_t)128, (uint8_t)182, (uint8_t)72, (uint8_t)12, (uint8_t)192, (uint8_t)118, (uint8_t)252, (uint8_t)40, (uint8_t)134, (uint8_t)210, (uint8_t)217, (uint8_t)19, (uint8_t)75, (uint8_t)51, (uint8_t)69, (uint8_t)177, (uint8_t)225, (uint8_t)191, (uint8_t)178, (uint8_t)244, (uint8_t)69, (uint8_t)149, (uint8_t)33, (uint8_t)36, (uint8_t)247, (uint8_t)173, (uint8_t)221, (uint8_t)159, (uint8_t)129, (uint8_t)1, (uint8_t)175, (uint8_t)207, (uint8_t)131, (uint8_t)160, (uint8_t)199, (uint8_t)200, (uint8_t)151, (uint8_t)174, (uint8_t)78, (uint8_t)176, (uint8_t)79, (uint8_t)9, (uint8_t)128, (uint8_t)251, (uint8_t)225, (uint8_t)72, (uint8_t)5, (uint8_t)4, (uint8_t)252, (uint8_t)224, (uint8_t)240, (uint8_t)137, (uint8_t)182, (uint8_t)49, (uint8_t)242, (uint8_t)47, (uint8_t)34, (uint8_t)44, (uint8_t)118, (uint8_t)130, (uint8_t)34, (uint8_t)105, (uint8_t)38, (uint8_t)215, (uint8_t)120, (uint8_t)182, (uint8_t)33, (uint8_t)137, (uint8_t)247, (uint8_t)176, (uint8_t)149, (uint8_t)105, (uint8_t)198, (uint8_t)237, (uint8_t)59, (uint8_t)217, (uint8_t)153, (uint8_t)186, (uint8_t)229, (uint8_t)54, (uint8_t)119, (uint8_t)148, (uint8_t)31, (uint8_t)111, (uint8_t)30, (uint8_t)226, (uint8_t)46, (uint8_t)162, (uint8_t)195, (uint8_t)152, (uint8_t)220, (uint8_t)145, (uint8_t)27, (uint8_t)19, (uint8_t)62, (uint8_t)115, (uint8_t)57, (uint8_t)157, (uint8_t)17, (uint8_t)191, (uint8_t)180, (uint8_t)210, (uint8_t)113, (uint8_t)122, (uint8_t)112, (uint8_t)207, (uint8_t)18, (uint8_t)85, (uint8_t)9, (uint8_t)230, (uint8_t)110, (uint8_t)131, (uint8_t)75, (uint8_t)204, (uint8_t)60, (uint8_t)132, (uint8_t)75, (uint8_t)47, (uint8_t)184, (uint8_t)237, (uint8_t)183, (uint8_t)30, (uint8_t)34, (uint8_t)244, (uint8_t)65, (uint8_t)227, (uint8_t)118, (uint8_t)237, (uint8_t)236, (uint8_t)41, (uint8_t)170, (uint8_t)28, (uint8_t)166, (uint8_t)163, (uint8_t)235, (uint8_t)109, (uint8_t)225, (uint8_t)62, (uint8_t)166, (uint8_t)174, (uint8_t)199, (uint8_t)92, (uint8_t)132, (uint8_t)80, (uint8_t)246, (uint8_t)28, (uint8_t)138, (uint8_t)104, (uint8_t)56, (uint8_t)246, (uint8_t)11, (uint8_t)2, (uint8_t)226, (uint8_t)57, (uint8_t)8, (uint8_t)131, (uint8_t)227, (uint8_t)92, (uint8_t)197, (uint8_t)111, (uint8_t)101, (uint8_t)185, (uint8_t)191, (uint8_t)92, (uint8_t)89, (uint8_t)53, (uint8_t)21, (uint8_t)179, (uint8_t)119, (uint8_t)53, (uint8_t)213} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -7285);
    assert(p234_latitude_GET(pack) == (int32_t)1818218974);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)14885);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -40);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)34);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -30424);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)62);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -6847);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)53);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)5410);
    assert(p234_longitude_GET(pack) == (int32_t)1553936748);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)63563);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3410364383L);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)5294);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_0_GET(pack) == (uint32_t)3245080149L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3065876711L);
    assert(p241_vibration_z_GET(pack) == (float)1.3878362E38F);
    assert(p241_vibration_x_GET(pack) == (float)3.517538E37F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)2601814907L);
    assert(p241_vibration_y_GET(pack) == (float)1.5764512E37F);
    assert(p241_time_usec_GET(pack) == (uint64_t)7112826981975066358L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.2630208E38F, -1.2790843E38F, -1.2075706E38F, -1.8321488E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_altitude_GET(pack) == (int32_t)1982676518);
    assert(p242_x_GET(pack) == (float) -1.311692E38F);
    assert(p242_approach_x_GET(pack) == (float) -2.824099E37F);
    assert(p242_y_GET(pack) == (float)2.5701798E38F);
    assert(p242_longitude_GET(pack) == (int32_t) -1725962238);
    assert(p242_latitude_GET(pack) == (int32_t)971856752);
    assert(p242_z_GET(pack) == (float)1.2248006E38F);
    assert(p242_approach_z_GET(pack) == (float) -2.3858985E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)2868915822043231427L);
    assert(p242_approach_y_GET(pack) == (float) -1.8135237E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_longitude_GET(pack) == (int32_t)1041210971);
    assert(p243_x_GET(pack) == (float) -2.052369E38F);
    {
        float exemplary[] =  {2.316682E38F, -1.8092025E38F, -2.702062E38F, 3.2273427E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_latitude_GET(pack) == (int32_t) -1986629888);
    assert(p243_z_GET(pack) == (float) -1.4942327E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)6719374518839870456L);
    assert(p243_y_GET(pack) == (float)7.973893E37F);
    assert(p243_approach_y_GET(pack) == (float) -1.302164E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p243_altitude_GET(pack) == (int32_t) -187229160);
    assert(p243_approach_x_GET(pack) == (float) -7.7476837E37F);
    assert(p243_approach_z_GET(pack) == (float)3.3072244E38F);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)48559);
    assert(p244_interval_us_GET(pack) == (int32_t) -1807338229);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -4319);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)11783);
    assert(p246_lat_GET(pack) == (int32_t)1570758488);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)15536);
    assert(p246_lon_GET(pack) == (int32_t) -2133582170);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)64533);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN);
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"bbvjt";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_GET(pack) == (int32_t)1509011478);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)4048866499L);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -3.2987826E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)3.1742691E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_id_GET(pack) == (uint32_t)307192492L);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -9.183165E36F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)49421);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    {
        uint8_t exemplary[] =  {(uint8_t)209, (uint8_t)60, (uint8_t)241, (uint8_t)72, (uint8_t)249, (uint8_t)122, (uint8_t)251, (uint8_t)243, (uint8_t)222, (uint8_t)29, (uint8_t)88, (uint8_t)132, (uint8_t)34, (uint8_t)193, (uint8_t)153, (uint8_t)44, (uint8_t)126, (uint8_t)171, (uint8_t)148, (uint8_t)97, (uint8_t)177, (uint8_t)181, (uint8_t)150, (uint8_t)221, (uint8_t)210, (uint8_t)22, (uint8_t)237, (uint8_t)173, (uint8_t)63, (uint8_t)72, (uint8_t)112, (uint8_t)218, (uint8_t)109, (uint8_t)22, (uint8_t)137, (uint8_t)226, (uint8_t)154, (uint8_t)52, (uint8_t)171, (uint8_t)25, (uint8_t)162, (uint8_t)167, (uint8_t)238, (uint8_t)99, (uint8_t)21, (uint8_t)255, (uint8_t)208, (uint8_t)231, (uint8_t)171, (uint8_t)114, (uint8_t)19, (uint8_t)187, (uint8_t)218, (uint8_t)21, (uint8_t)61, (uint8_t)158, (uint8_t)104, (uint8_t)17, (uint8_t)230, (uint8_t)78, (uint8_t)208, (uint8_t)41, (uint8_t)217, (uint8_t)17, (uint8_t)180, (uint8_t)152, (uint8_t)39, (uint8_t)128, (uint8_t)220, (uint8_t)59, (uint8_t)205, (uint8_t)65, (uint8_t)75, (uint8_t)7, (uint8_t)156, (uint8_t)251, (uint8_t)147, (uint8_t)204, (uint8_t)57, (uint8_t)158, (uint8_t)142, (uint8_t)112, (uint8_t)111, (uint8_t)43, (uint8_t)148, (uint8_t)173, (uint8_t)190, (uint8_t)192, (uint8_t)72, (uint8_t)109, (uint8_t)99, (uint8_t)67, (uint8_t)200, (uint8_t)98, (uint8_t)187, (uint8_t)92, (uint8_t)137, (uint8_t)222, (uint8_t)202, (uint8_t)122, (uint8_t)114, (uint8_t)253, (uint8_t)211, (uint8_t)255, (uint8_t)199, (uint8_t)129, (uint8_t)36, (uint8_t)165, (uint8_t)165, (uint8_t)1, (uint8_t)128, (uint8_t)123, (uint8_t)15, (uint8_t)65, (uint8_t)117, (uint8_t)200, (uint8_t)203, (uint8_t)181, (uint8_t)37, (uint8_t)24, (uint8_t)99, (uint8_t)251, (uint8_t)180, (uint8_t)1, (uint8_t)211, (uint8_t)137, (uint8_t)155, (uint8_t)152, (uint8_t)249, (uint8_t)235, (uint8_t)158, (uint8_t)73, (uint8_t)191, (uint8_t)131, (uint8_t)22, (uint8_t)202, (uint8_t)198, (uint8_t)71, (uint8_t)48, (uint8_t)145, (uint8_t)156, (uint8_t)91, (uint8_t)63, (uint8_t)92, (uint8_t)253, (uint8_t)211, (uint8_t)229, (uint8_t)145, (uint8_t)145, (uint8_t)204, (uint8_t)37, (uint8_t)1, (uint8_t)137, (uint8_t)230, (uint8_t)33, (uint8_t)178, (uint8_t)5, (uint8_t)183, (uint8_t)98, (uint8_t)245, (uint8_t)115, (uint8_t)161, (uint8_t)235, (uint8_t)96, (uint8_t)88, (uint8_t)83, (uint8_t)186, (uint8_t)251, (uint8_t)99, (uint8_t)222, (uint8_t)28, (uint8_t)147, (uint8_t)236, (uint8_t)127, (uint8_t)247, (uint8_t)159, (uint8_t)54, (uint8_t)27, (uint8_t)47, (uint8_t)2, (uint8_t)183, (uint8_t)13, (uint8_t)104, (uint8_t)240, (uint8_t)55, (uint8_t)245, (uint8_t)124, (uint8_t)101, (uint8_t)150, (uint8_t)198, (uint8_t)34, (uint8_t)171, (uint8_t)116, (uint8_t)188, (uint8_t)133, (uint8_t)154, (uint8_t)251, (uint8_t)229, (uint8_t)111, (uint8_t)96, (uint8_t)35, (uint8_t)241, (uint8_t)236, (uint8_t)184, (uint8_t)161, (uint8_t)242, (uint8_t)15, (uint8_t)196, (uint8_t)14, (uint8_t)97, (uint8_t)157, (uint8_t)26, (uint8_t)172, (uint8_t)18, (uint8_t)16, (uint8_t)43, (uint8_t)219, (uint8_t)49, (uint8_t)69, (uint8_t)198, (uint8_t)103, (uint8_t)72, (uint8_t)89, (uint8_t)195, (uint8_t)38, (uint8_t)230, (uint8_t)66, (uint8_t)25, (uint8_t)29, (uint8_t)35, (uint8_t)103, (uint8_t)188, (uint8_t)53, (uint8_t)247, (uint8_t)143, (uint8_t)56, (uint8_t)88, (uint8_t)210, (uint8_t)75, (uint8_t)195, (uint8_t)89, (uint8_t)195, (uint8_t)177, (uint8_t)254, (uint8_t)252, (uint8_t)34, (uint8_t)18, (uint8_t)107, (uint8_t)171} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)253);
    {
        int8_t exemplary[] =  {(int8_t)80, (int8_t) -125, (int8_t) -24, (int8_t)81, (int8_t)48, (int8_t)45, (int8_t)72, (int8_t)92, (int8_t)83, (int8_t)63, (int8_t)107, (int8_t)50, (int8_t)110, (int8_t)45, (int8_t)78, (int8_t)88, (int8_t)28, (int8_t)34, (int8_t)49, (int8_t)36, (int8_t) -64, (int8_t) -59, (int8_t) -73, (int8_t) -116, (int8_t) -77, (int8_t) -12, (int8_t) -30, (int8_t)67, (int8_t) -45, (int8_t) -95, (int8_t)98, (int8_t)75} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)9305);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_time_usec_GET(pack) == (uint64_t)4650371507381707764L);
    assert(p250_z_GET(pack) == (float) -1.3048703E38F);
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"yweiu";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_x_GET(pack) == (float)2.4529138E38F);
    assert(p250_y_GET(pack) == (float)2.0080038E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2238007142L);
    assert(p251_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"edPtsL";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float)2.4445225E37F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -1435564799);
    assert(p252_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"as";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)197211436L);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_DEBUG);
    assert(p253_text_LEN(ph) == 49);
    {
        char16_t * exemplary = u"umXqpvxkhjfiviutsdjivcifpRyfdvfyiwOwfqwilauupkYag";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 98);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)2.894353E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1133347627L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)7111097222356842759L);
    {
        uint8_t exemplary[] =  {(uint8_t)94, (uint8_t)135, (uint8_t)7, (uint8_t)171, (uint8_t)246, (uint8_t)108, (uint8_t)93, (uint8_t)10, (uint8_t)50, (uint8_t)12, (uint8_t)7, (uint8_t)15, (uint8_t)155, (uint8_t)185, (uint8_t)146, (uint8_t)117, (uint8_t)75, (uint8_t)149, (uint8_t)207, (uint8_t)30, (uint8_t)166, (uint8_t)57, (uint8_t)179, (uint8_t)44, (uint8_t)231, (uint8_t)73, (uint8_t)252, (uint8_t)24, (uint8_t)230, (uint8_t)231, (uint8_t)186, (uint8_t)111} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3272807597L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1537249705L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p258_tune_LEN(ph) == 28);
    {
        char16_t * exemplary = u"oyocvpbmesWfkYvRnunbxrLvahwz";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)128);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)22616);
    assert(p259_firmware_version_GET(pack) == (uint32_t)3985822893L);
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)224, (uint8_t)191, (uint8_t)144, (uint8_t)210, (uint8_t)206, (uint8_t)236, (uint8_t)19, (uint8_t)98, (uint8_t)20, (uint8_t)24, (uint8_t)110, (uint8_t)198, (uint8_t)186, (uint8_t)41, (uint8_t)138, (uint8_t)178, (uint8_t)159, (uint8_t)159, (uint8_t)174, (uint8_t)228, (uint8_t)115, (uint8_t)169, (uint8_t)242, (uint8_t)109, (uint8_t)2, (uint8_t)212, (uint8_t)215, (uint8_t)58, (uint8_t)179, (uint8_t)157, (uint8_t)84} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)199, (uint8_t)47, (uint8_t)29, (uint8_t)92, (uint8_t)71, (uint8_t)214, (uint8_t)211, (uint8_t)133, (uint8_t)60, (uint8_t)134, (uint8_t)46, (uint8_t)55, (uint8_t)194, (uint8_t)17, (uint8_t)148, (uint8_t)245, (uint8_t)201, (uint8_t)25, (uint8_t)166, (uint8_t)245, (uint8_t)105, (uint8_t)238, (uint8_t)148, (uint8_t)231, (uint8_t)213, (uint8_t)29, (uint8_t)50, (uint8_t)114, (uint8_t)57, (uint8_t)134, (uint8_t)12, (uint8_t)161} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)19659);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.7812122E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1137553395L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES);
    assert(p259_sensor_size_v_GET(pack) == (float) -4.868836E37F);
    assert(p259_cam_definition_uri_LEN(ph) == 68);
    {
        char16_t * exemplary = u"kuesDqNjMbafnnvvNobszjaordnuvewmriLknbUabtJzfvlbsvwYdpcsqqiKxtmalzsf";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 136);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)36550);
    assert(p259_focal_length_GET(pack) == (float) -1.5061874E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3248843086L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_total_capacity_GET(pack) == (float) -7.5645716E37F);
    assert(p261_available_capacity_GET(pack) == (float)1.025265E38F);
    assert(p261_used_capacity_GET(pack) == (float)2.8568423E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3119234615L);
    assert(p261_write_speed_GET(pack) == (float) -2.0145284E37F);
    assert(p261_read_speed_GET(pack) == (float) -6.582356E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float)3.304827E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2799026135L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1725655155L);
    assert(p262_available_capacity_GET(pack) == (float)1.3053787E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)841051333L);
    assert(p263_file_url_LEN(ph) == 53);
    {
        char16_t * exemplary = u"gmnprkOeOheyppprsuhbevtppzxwfrwliyugycgklcqszlwuUnbSp";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 106);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_image_index_GET(pack) == (int32_t)530902819);
    assert(p263_lat_GET(pack) == (int32_t) -976291802);
    assert(p263_lon_GET(pack) == (int32_t)814775136);
    assert(p263_relative_alt_GET(pack) == (int32_t)1964828273);
    assert(p263_alt_GET(pack) == (int32_t) -202031009);
    {
        float exemplary[] =  {3.2164322E38F, 1.9580393E38F, 3.1587819E38F, -2.609203E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_utc_GET(pack) == (uint64_t)7586065036574521422L);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -15);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)8027458140053502740L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)2482331377318891808L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)4293329736L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)1866953520352671609L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float)2.5208903E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)3038542671L);
    assert(p265_roll_GET(pack) == (float) -5.4497283E37F);
    assert(p265_yaw_GET(pack) == (float)2.8706193E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)49184);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)174);
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)10, (uint8_t)138, (uint8_t)216, (uint8_t)42, (uint8_t)206, (uint8_t)39, (uint8_t)203, (uint8_t)230, (uint8_t)100, (uint8_t)12, (uint8_t)205, (uint8_t)244, (uint8_t)123, (uint8_t)159, (uint8_t)13, (uint8_t)177, (uint8_t)104, (uint8_t)236, (uint8_t)184, (uint8_t)77, (uint8_t)114, (uint8_t)44, (uint8_t)255, (uint8_t)157, (uint8_t)79, (uint8_t)205, (uint8_t)83, (uint8_t)127, (uint8_t)16, (uint8_t)52, (uint8_t)167, (uint8_t)206, (uint8_t)40, (uint8_t)41, (uint8_t)101, (uint8_t)77, (uint8_t)127, (uint8_t)212, (uint8_t)215, (uint8_t)227, (uint8_t)199, (uint8_t)124, (uint8_t)160, (uint8_t)202, (uint8_t)255, (uint8_t)165, (uint8_t)97, (uint8_t)13, (uint8_t)53, (uint8_t)91, (uint8_t)186, (uint8_t)53, (uint8_t)218, (uint8_t)159, (uint8_t)252, (uint8_t)134, (uint8_t)242, (uint8_t)48, (uint8_t)118, (uint8_t)248, (uint8_t)228, (uint8_t)84, (uint8_t)88, (uint8_t)155, (uint8_t)217, (uint8_t)101, (uint8_t)15, (uint8_t)182, (uint8_t)199, (uint8_t)198, (uint8_t)123, (uint8_t)21, (uint8_t)146, (uint8_t)170, (uint8_t)253, (uint8_t)106, (uint8_t)107, (uint8_t)159, (uint8_t)122, (uint8_t)27, (uint8_t)37, (uint8_t)68, (uint8_t)62, (uint8_t)141, (uint8_t)102, (uint8_t)97, (uint8_t)130, (uint8_t)64, (uint8_t)8, (uint8_t)40, (uint8_t)40, (uint8_t)155, (uint8_t)72, (uint8_t)247, (uint8_t)195, (uint8_t)35, (uint8_t)216, (uint8_t)150, (uint8_t)237, (uint8_t)101, (uint8_t)47, (uint8_t)194, (uint8_t)79, (uint8_t)208, (uint8_t)139, (uint8_t)236, (uint8_t)25, (uint8_t)236, (uint8_t)235, (uint8_t)32, (uint8_t)110, (uint8_t)176, (uint8_t)19, (uint8_t)32, (uint8_t)59, (uint8_t)37, (uint8_t)12, (uint8_t)222, (uint8_t)144, (uint8_t)142, (uint8_t)68, (uint8_t)75, (uint8_t)55, (uint8_t)216, (uint8_t)92, (uint8_t)127, (uint8_t)219, (uint8_t)92, (uint8_t)181, (uint8_t)66, (uint8_t)150, (uint8_t)120, (uint8_t)187, (uint8_t)65, (uint8_t)221, (uint8_t)12, (uint8_t)91, (uint8_t)207, (uint8_t)188, (uint8_t)138, (uint8_t)139, (uint8_t)131, (uint8_t)179, (uint8_t)7, (uint8_t)182, (uint8_t)35, (uint8_t)235, (uint8_t)155, (uint8_t)215, (uint8_t)108, (uint8_t)46, (uint8_t)181, (uint8_t)46, (uint8_t)2, (uint8_t)124, (uint8_t)84, (uint8_t)58, (uint8_t)67, (uint8_t)64, (uint8_t)193, (uint8_t)235, (uint8_t)181, (uint8_t)209, (uint8_t)195, (uint8_t)237, (uint8_t)174, (uint8_t)55, (uint8_t)78, (uint8_t)153, (uint8_t)138, (uint8_t)236, (uint8_t)99, (uint8_t)232, (uint8_t)234, (uint8_t)216, (uint8_t)99, (uint8_t)6, (uint8_t)24, (uint8_t)194, (uint8_t)76, (uint8_t)8, (uint8_t)13, (uint8_t)124, (uint8_t)83, (uint8_t)154, (uint8_t)131, (uint8_t)202, (uint8_t)136, (uint8_t)151, (uint8_t)43, (uint8_t)61, (uint8_t)184, (uint8_t)173, (uint8_t)69, (uint8_t)76, (uint8_t)147, (uint8_t)87, (uint8_t)179, (uint8_t)109, (uint8_t)87, (uint8_t)27, (uint8_t)190, (uint8_t)51, (uint8_t)190, (uint8_t)2, (uint8_t)102, (uint8_t)49, (uint8_t)44, (uint8_t)111, (uint8_t)137, (uint8_t)132, (uint8_t)207, (uint8_t)149, (uint8_t)36, (uint8_t)22, (uint8_t)224, (uint8_t)67, (uint8_t)54, (uint8_t)77, (uint8_t)96, (uint8_t)202, (uint8_t)231, (uint8_t)205, (uint8_t)190, (uint8_t)65, (uint8_t)1, (uint8_t)168, (uint8_t)7, (uint8_t)181, (uint8_t)149, (uint8_t)139, (uint8_t)174, (uint8_t)213, (uint8_t)175, (uint8_t)253, (uint8_t)190, (uint8_t)235, (uint8_t)151, (uint8_t)212, (uint8_t)2, (uint8_t)224, (uint8_t)212, (uint8_t)130, (uint8_t)145, (uint8_t)66, (uint8_t)162, (uint8_t)100, (uint8_t)41} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)13992);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)17);
    {
        uint8_t exemplary[] =  {(uint8_t)98, (uint8_t)109, (uint8_t)146, (uint8_t)170, (uint8_t)144, (uint8_t)208, (uint8_t)249, (uint8_t)47, (uint8_t)114, (uint8_t)234, (uint8_t)134, (uint8_t)40, (uint8_t)240, (uint8_t)55, (uint8_t)236, (uint8_t)98, (uint8_t)160, (uint8_t)198, (uint8_t)238, (uint8_t)57, (uint8_t)184, (uint8_t)108, (uint8_t)164, (uint8_t)181, (uint8_t)95, (uint8_t)92, (uint8_t)43, (uint8_t)129, (uint8_t)38, (uint8_t)31, (uint8_t)130, (uint8_t)139, (uint8_t)151, (uint8_t)58, (uint8_t)22, (uint8_t)42, (uint8_t)95, (uint8_t)200, (uint8_t)43, (uint8_t)4, (uint8_t)163, (uint8_t)102, (uint8_t)188, (uint8_t)87, (uint8_t)168, (uint8_t)144, (uint8_t)182, (uint8_t)102, (uint8_t)15, (uint8_t)138, (uint8_t)164, (uint8_t)203, (uint8_t)86, (uint8_t)107, (uint8_t)105, (uint8_t)7, (uint8_t)191, (uint8_t)104, (uint8_t)41, (uint8_t)60, (uint8_t)186, (uint8_t)146, (uint8_t)49, (uint8_t)160, (uint8_t)56, (uint8_t)240, (uint8_t)162, (uint8_t)136, (uint8_t)209, (uint8_t)130, (uint8_t)78, (uint8_t)197, (uint8_t)26, (uint8_t)25, (uint8_t)81, (uint8_t)93, (uint8_t)0, (uint8_t)114, (uint8_t)45, (uint8_t)255, (uint8_t)158, (uint8_t)41, (uint8_t)198, (uint8_t)7, (uint8_t)56, (uint8_t)57, (uint8_t)217, (uint8_t)10, (uint8_t)212, (uint8_t)4, (uint8_t)237, (uint8_t)149, (uint8_t)118, (uint8_t)21, (uint8_t)73, (uint8_t)148, (uint8_t)174, (uint8_t)45, (uint8_t)254, (uint8_t)245, (uint8_t)219, (uint8_t)151, (uint8_t)165, (uint8_t)14, (uint8_t)232, (uint8_t)90, (uint8_t)219, (uint8_t)78, (uint8_t)184, (uint8_t)12, (uint8_t)131, (uint8_t)75, (uint8_t)111, (uint8_t)146, (uint8_t)95, (uint8_t)39, (uint8_t)9, (uint8_t)181, (uint8_t)30, (uint8_t)134, (uint8_t)125, (uint8_t)65, (uint8_t)181, (uint8_t)148, (uint8_t)39, (uint8_t)150, (uint8_t)103, (uint8_t)204, (uint8_t)208, (uint8_t)102, (uint8_t)76, (uint8_t)164, (uint8_t)187, (uint8_t)124, (uint8_t)151, (uint8_t)153, (uint8_t)3, (uint8_t)214, (uint8_t)225, (uint8_t)43, (uint8_t)64, (uint8_t)57, (uint8_t)211, (uint8_t)239, (uint8_t)0, (uint8_t)244, (uint8_t)180, (uint8_t)151, (uint8_t)186, (uint8_t)73, (uint8_t)206, (uint8_t)241, (uint8_t)222, (uint8_t)45, (uint8_t)35, (uint8_t)216, (uint8_t)227, (uint8_t)176, (uint8_t)245, (uint8_t)221, (uint8_t)236, (uint8_t)157, (uint8_t)28, (uint8_t)236, (uint8_t)10, (uint8_t)114, (uint8_t)195, (uint8_t)9, (uint8_t)220, (uint8_t)170, (uint8_t)7, (uint8_t)126, (uint8_t)180, (uint8_t)41, (uint8_t)50, (uint8_t)136, (uint8_t)77, (uint8_t)129, (uint8_t)122, (uint8_t)167, (uint8_t)145, (uint8_t)113, (uint8_t)107, (uint8_t)130, (uint8_t)17, (uint8_t)85, (uint8_t)86, (uint8_t)30, (uint8_t)10, (uint8_t)243, (uint8_t)64, (uint8_t)74, (uint8_t)32, (uint8_t)202, (uint8_t)246, (uint8_t)65, (uint8_t)179, (uint8_t)220, (uint8_t)95, (uint8_t)13, (uint8_t)116, (uint8_t)228, (uint8_t)103, (uint8_t)171, (uint8_t)156, (uint8_t)134, (uint8_t)115, (uint8_t)109, (uint8_t)126, (uint8_t)220, (uint8_t)149, (uint8_t)243, (uint8_t)219, (uint8_t)197, (uint8_t)227, (uint8_t)12, (uint8_t)188, (uint8_t)41, (uint8_t)227, (uint8_t)40, (uint8_t)180, (uint8_t)92, (uint8_t)18, (uint8_t)176, (uint8_t)247, (uint8_t)53, (uint8_t)225, (uint8_t)105, (uint8_t)196, (uint8_t)12, (uint8_t)104, (uint8_t)112, (uint8_t)97, (uint8_t)119, (uint8_t)44, (uint8_t)254, (uint8_t)161, (uint8_t)171, (uint8_t)30, (uint8_t)85, (uint8_t)165, (uint8_t)31, (uint8_t)85, (uint8_t)36, (uint8_t)127, (uint8_t)31, (uint8_t)216, (uint8_t)132, (uint8_t)119} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)74);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)27573);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)215);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float)9.24535E37F);
    assert(p269_uri_LEN(ph) == 42);
    {
        char16_t * exemplary = u"kdwhpguuaibxmpXSfrcvtmqlclvekuqlcwInrEyjlf";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 84);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_bitrate_GET(pack) == (uint32_t)902381921L);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)21514);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)38983);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)58018);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)95);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float) -1.8604894E38F);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)42710);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p270_uri_LEN(ph) == 124);
    {
        char16_t * exemplary = u"JqaigwrsFnqxsrmskgnfgufvwymasLjsondgymrghmdoIffniuwWcjbbrewhpxwwbgpwkoFevtyimdceegyrAjwjyjcbvljtmxhcgqiwnmlhVnybkonzfccajKwt";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 248);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p270_bitrate_GET(pack) == (uint32_t)2252073493L);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)49052);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)27930);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 10);
    {
        char16_t * exemplary = u"zeMQflMnly";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 32);
    {
        char16_t * exemplary = u"rfdfbxbvowKcctgkztivdssdkhkgBngk";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)43070);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)44919);
    {
        uint8_t exemplary[] =  {(uint8_t)80, (uint8_t)109, (uint8_t)101, (uint8_t)89, (uint8_t)62, (uint8_t)255, (uint8_t)88, (uint8_t)23} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)54251);
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)107, (uint8_t)237, (uint8_t)184, (uint8_t)113, (uint8_t)214, (uint8_t)228, (uint8_t)5} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)31477);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)3341117272L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
    assert(p310_time_usec_GET(pack) == (uint64_t)4991602045285669298L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_name_LEN(ph) == 26);
    {
        char16_t * exemplary = u"lrnkrwztiqrrclhGuxrvahopyw";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 52);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1963456487L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2030341977L);
    assert(p311_time_usec_GET(pack) == (uint64_t)1782529378361858282L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)23);
    {
        uint8_t exemplary[] =  {(uint8_t)4, (uint8_t)10, (uint8_t)111, (uint8_t)237, (uint8_t)104, (uint8_t)43, (uint8_t)188, (uint8_t)200, (uint8_t)219, (uint8_t)108, (uint8_t)142, (uint8_t)59, (uint8_t)82, (uint8_t)233, (uint8_t)226, (uint8_t)28} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -13348);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p320_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"xkmnicyqldfU";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)13224);
    assert(p322_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"kfypjqyfgui";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)8766);
    assert(p322_param_value_LEN(ph) == 59);
    {
        char16_t * exemplary = u"hwsqmomllycwdznandjbhzlgygYtvfuGfuSvmwfkxadatrctlaxvqHnxkal";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p323_param_value_LEN(ph) == 48);
    {
        char16_t * exemplary = u"hlebxbuabHxyemwoyFratwzurrrvsggnxcRofasddxlQwtld";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p323_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"ykyqjDvagQscf";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"kdkghme";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 81);
    {
        char16_t * exemplary = u"irxmywjmumbcisQbTfxtxxktvyrmjocyckdpxrxysjfsustbwjscmfbiiBuWHaedymuzhajuvtpjsaKhz";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 162);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)47195);
    {
        uint16_t exemplary[] =  {(uint16_t)60663, (uint16_t)8433, (uint16_t)8362, (uint16_t)33594, (uint16_t)13534, (uint16_t)64316, (uint16_t)60904, (uint16_t)18783, (uint16_t)15767, (uint16_t)22628, (uint16_t)18302, (uint16_t)37047, (uint16_t)12631, (uint16_t)34881, (uint16_t)56581, (uint16_t)17253, (uint16_t)33155, (uint16_t)56774, (uint16_t)51397, (uint16_t)1579, (uint16_t)6284, (uint16_t)1238, (uint16_t)50250, (uint16_t)25333, (uint16_t)46169, (uint16_t)44627, (uint16_t)63276, (uint16_t)38936, (uint16_t)59887, (uint16_t)60194, (uint16_t)63492, (uint16_t)21417, (uint16_t)9541, (uint16_t)48164, (uint16_t)38697, (uint16_t)11983, (uint16_t)10258, (uint16_t)47781, (uint16_t)44094, (uint16_t)9330, (uint16_t)19172, (uint16_t)38290, (uint16_t)38439, (uint16_t)45060, (uint16_t)12589, (uint16_t)10837, (uint16_t)10441, (uint16_t)63084, (uint16_t)42418, (uint16_t)18367, (uint16_t)9748, (uint16_t)21531, (uint16_t)60141, (uint16_t)13014, (uint16_t)57109, (uint16_t)13577, (uint16_t)57594, (uint16_t)31860, (uint16_t)60316, (uint16_t)24423, (uint16_t)7110, (uint16_t)353, (uint16_t)64380, (uint16_t)1904, (uint16_t)62473, (uint16_t)51702, (uint16_t)6103, (uint16_t)51897, (uint16_t)19873, (uint16_t)995, (uint16_t)40693, (uint16_t)33354} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)50494);
    assert(p330_time_usec_GET(pack) == (uint64_t)4755424246889221691L);
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
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)809137286L, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_comm_SET((uint16_t)(uint16_t)12980, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)21167, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)11986, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)61, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)8179, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)29154, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)28571, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)43126, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)44159, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)15629, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2622098734589340627L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)3194082523L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_y_SET((float)1.1096203E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3566854029L, PH.base.pack) ;
        p3_z_SET((float)2.875108E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)9692, PH.base.pack) ;
        p3_x_SET((float) -2.4023005E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.0917272E37F, PH.base.pack) ;
        p3_yaw_SET((float)3.352143E38F, PH.base.pack) ;
        p3_vy_SET((float) -1.4164575E38F, PH.base.pack) ;
        p3_afy_SET((float)2.351771E38F, PH.base.pack) ;
        p3_vz_SET((float)2.038012E38F, PH.base.pack) ;
        p3_vx_SET((float)2.776923E38F, PH.base.pack) ;
        p3_afx_SET((float)1.806732E38F, PH.base.pack) ;
        p3_afz_SET((float) -2.4110873E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)3283251225089904289L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p4_seq_SET((uint32_t)3894362690L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        {
            char16_t* passkey = u"ayzkk";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"hMvcllXxnkizbaxizkbR";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)808621476L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -30822, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        {
            char16_t* param_id = u"dkanyiAu";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_index_SET((uint16_t)(uint16_t)43371, PH.base.pack) ;
        {
            char16_t* param_id = u"Gpzgdskbyahv";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p22_param_value_SET((float)1.8775855E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)36037, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, PH.base.pack) ;
        p23_param_value_SET((float) -2.7998846E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        {
            char16_t* param_id = u"odkuiwfQdzFfr";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_eph_SET((uint16_t)(uint16_t)51077, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)1092499746L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t)2072254636, &PH) ;
        p24_hdg_acc_SET((uint32_t)1040543488L, &PH) ;
        p24_alt_SET((int32_t)1698831682, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)1965996827686821947L, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)61423, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)271104242L, &PH) ;
        p24_h_acc_SET((uint32_t)203254143L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)21092, PH.base.pack) ;
        p24_lon_SET((int32_t)917063662, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)18942, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p24_lat_SET((int32_t)1131206628, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)23, (uint8_t)8, (uint8_t)89, (uint8_t)177, (uint8_t)113, (uint8_t)131, (uint8_t)211, (uint8_t)133, (uint8_t)36, (uint8_t)52, (uint8_t)81, (uint8_t)110, (uint8_t)239, (uint8_t)249, (uint8_t)104, (uint8_t)98, (uint8_t)63, (uint8_t)99, (uint8_t)133, (uint8_t)81};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)121, (uint8_t)60, (uint8_t)0, (uint8_t)146, (uint8_t)136, (uint8_t)186, (uint8_t)88, (uint8_t)248, (uint8_t)81, (uint8_t)213, (uint8_t)27, (uint8_t)32, (uint8_t)9, (uint8_t)67, (uint8_t)172, (uint8_t)81, (uint8_t)2, (uint8_t)61, (uint8_t)37, (uint8_t)251};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)226, (uint8_t)82, (uint8_t)56, (uint8_t)51, (uint8_t)169, (uint8_t)35, (uint8_t)33, (uint8_t)26, (uint8_t)9, (uint8_t)192, (uint8_t)100, (uint8_t)240, (uint8_t)171, (uint8_t)62, (uint8_t)255, (uint8_t)99, (uint8_t)220, (uint8_t)71, (uint8_t)232, (uint8_t)232};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        {
            uint8_t satellite_snr[] =  {(uint8_t)73, (uint8_t)16, (uint8_t)159, (uint8_t)56, (uint8_t)248, (uint8_t)143, (uint8_t)250, (uint8_t)9, (uint8_t)140, (uint8_t)82, (uint8_t)191, (uint8_t)192, (uint8_t)56, (uint8_t)196, (uint8_t)142, (uint8_t)115, (uint8_t)204, (uint8_t)14, (uint8_t)204, (uint8_t)84};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)19, (uint8_t)36, (uint8_t)237, (uint8_t)101, (uint8_t)49, (uint8_t)158, (uint8_t)149, (uint8_t)51, (uint8_t)87, (uint8_t)184, (uint8_t)118, (uint8_t)78, (uint8_t)160, (uint8_t)56, (uint8_t)222, (uint8_t)231, (uint8_t)124, (uint8_t)185, (uint8_t)142, (uint8_t)95};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_ygyro_SET((int16_t)(int16_t)31093, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)7129, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)1892, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)6687, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -12842, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)27370, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -6643, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3975318625L, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)12712, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -9454, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t) -16617, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)20131, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)9941, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -4920, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -15378, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)5940732124011629387L, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)8963, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)22080, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -23267, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)31379, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_temperature_SET((int16_t)(int16_t)26369, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)12958, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)20497, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -30655, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)2291462735257379762L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)3.141997E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3414860797L, PH.base.pack) ;
        p29_press_diff_SET((float)3.8142357E37F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)16570, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_rollspeed_SET((float) -2.653771E38F, PH.base.pack) ;
        p30_yawspeed_SET((float) -3.258774E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -1.8285525E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2335728506L, PH.base.pack) ;
        p30_yaw_SET((float)2.485853E38F, PH.base.pack) ;
        p30_pitch_SET((float) -3.9584297E37F, PH.base.pack) ;
        p30_roll_SET((float) -3.3953707E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_time_boot_ms_SET((uint32_t)2826235990L, PH.base.pack) ;
        p31_rollspeed_SET((float)2.5915043E37F, PH.base.pack) ;
        p31_q1_SET((float)1.0023267E38F, PH.base.pack) ;
        p31_q3_SET((float)2.8121533E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -1.6823233E38F, PH.base.pack) ;
        p31_q2_SET((float) -2.6987969E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -9.146004E37F, PH.base.pack) ;
        p31_q4_SET((float) -1.6826351E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vz_SET((float)2.800312E38F, PH.base.pack) ;
        p32_y_SET((float) -1.4877643E38F, PH.base.pack) ;
        p32_x_SET((float) -6.7666494E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)3110168308L, PH.base.pack) ;
        p32_z_SET((float) -2.1209843E38F, PH.base.pack) ;
        p32_vy_SET((float)2.4714272E38F, PH.base.pack) ;
        p32_vx_SET((float) -3.0096021E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t) -1678664400, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)868030831L, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -8950, PH.base.pack) ;
        p33_alt_SET((int32_t)353592647, PH.base.pack) ;
        p33_lon_SET((int32_t) -1171601082, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)13176, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -6222, PH.base.pack) ;
        p33_lat_SET((int32_t) -1329296951, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)37292, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan5_scaled_SET((int16_t)(int16_t) -23397, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -7782, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)32361, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)2174, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -17996, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -19055, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -3723, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2082238375L, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)19500, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3879832184L, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)45784, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)966, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)51998, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)32965, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)12535, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)40410, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)35764, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)64542, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo9_raw_SET((uint16_t)(uint16_t)46791, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)45810, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)32245, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)54628, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)62612, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)40668, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)297, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)31968, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)35506, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)30453, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)14170, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)54474, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)24235, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)938, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)144066584L, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)32409, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)12348, &PH) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -12710, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)466, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)24287, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -23285, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p39_z_SET((float) -1.2766981E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p39_param1_SET((float)2.6505805E38F, PH.base.pack) ;
        p39_param4_SET((float)2.1190548E38F, PH.base.pack) ;
        p39_param2_SET((float)1.7989696E37F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)58776, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_HOME, PH.base.pack) ;
        p39_param3_SET((float) -7.8541486E37F, PH.base.pack) ;
        p39_y_SET((float)1.8165956E38F, PH.base.pack) ;
        p39_x_SET((float)1.3559911E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)57266, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)7104, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)63981, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)4639, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)53715, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p48_altitude_SET((int32_t) -2119536804, PH.base.pack) ;
        p48_longitude_SET((int32_t)1198185430, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)4617856014626162964L, &PH) ;
        p48_latitude_SET((int32_t)1754146992, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t) -2070343737, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)4447735428779582035L, &PH) ;
        p49_altitude_SET((int32_t) -201984389, PH.base.pack) ;
        p49_longitude_SET((int32_t)1731454014, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        {
            char16_t* param_id = u"u";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_scale_SET((float)7.608052E37F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p50_param_value_max_SET((float)1.9067686E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)28948, PH.base.pack) ;
        p50_param_value_min_SET((float)2.341577E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p50_param_value0_SET((float)3.1569676E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_system_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)57435, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p2z_SET((float)1.5300057E37F, PH.base.pack) ;
        p54_p2x_SET((float) -1.5836841E38F, PH.base.pack) ;
        p54_p1y_SET((float)3.2041502E38F, PH.base.pack) ;
        p54_p2y_SET((float) -9.99712E37F, PH.base.pack) ;
        p54_p1x_SET((float) -3.1517416E38F, PH.base.pack) ;
        p54_p1z_SET((float)8.142682E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float)1.72727E37F, PH.base.pack) ;
        p55_p2y_SET((float)5.628565E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p55_p2z_SET((float)1.1272875E38F, PH.base.pack) ;
        p55_p1x_SET((float) -3.3058752E38F, PH.base.pack) ;
        p55_p1z_SET((float) -3.1930875E38F, PH.base.pack) ;
        p55_p1y_SET((float)2.3435122E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_yawspeed_SET((float) -9.75791E37F, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.797504E38F, PH.base.pack) ;
        {
            float q[] =  {9.789241E37F, 3.3434416E38F, 2.6899905E38F, -1.3859009E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)5.4175665E37F, PH.base.pack) ;
        {
            float covariance[] =  {3.394102E38F, -6.2298937E37F, 2.0002888E38F, -1.3581386E38F, 3.2715545E38F, 8.185516E37F, 9.821624E37F, 8.473732E37F, 1.2709218E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)1707846319331419233L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_xtrack_error_SET((float)3.7627014E37F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -10054, PH.base.pack) ;
        p62_nav_roll_SET((float)3.3510461E38F, PH.base.pack) ;
        p62_aspd_error_SET((float) -2.3203097E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -4959, PH.base.pack) ;
        p62_alt_error_SET((float)8.0771654E37F, PH.base.pack) ;
        p62_nav_pitch_SET((float)2.412902E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)29976, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        {
            float covariance[] =  {-1.064123E38F, -1.4620767E38F, 3.86923E36F, 1.3462043E38F, 2.684396E38F, 2.7896266E38F, 3.2636293E38F, -1.1491052E38F, 1.5616369E38F, -2.7785275E38F, -3.003372E38F, -1.0468292E38F, 1.1434643E37F, 1.4722228E37F, 1.2581943E38F, 4.955536E37F, 2.5238378E38F, -2.5531479E38F, 2.9823365E38F, 2.6868331E38F, 2.237061E38F, -2.5427262E38F, 3.069777E37F, -2.5956977E37F, 3.0141786E38F, 3.114397E38F, -2.1987211E38F, 6.554806E37F, -3.0029637E38F, -2.3893096E38F, -3.8811047E37F, -7.2221594E37F, -3.0740823E38F, 1.7966424E38F, 2.8085152E38F, 8.71469E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vx_SET((float)1.7129617E38F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1387443891, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)2639236667969392442L, PH.base.pack) ;
        p63_vy_SET((float)1.4705049E38F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p63_lat_SET((int32_t) -996470391, PH.base.pack) ;
        p63_alt_SET((int32_t)190649253, PH.base.pack) ;
        p63_vz_SET((float)1.1649187E38F, PH.base.pack) ;
        p63_lon_SET((int32_t) -1872223117, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ax_SET((float) -2.4372753E38F, PH.base.pack) ;
        p64_y_SET((float)3.6606824E37F, PH.base.pack) ;
        p64_x_SET((float)2.1294559E38F, PH.base.pack) ;
        p64_vy_SET((float) -2.6319187E37F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)3742173354121202861L, PH.base.pack) ;
        p64_vz_SET((float) -2.767883E38F, PH.base.pack) ;
        p64_ay_SET((float) -5.0981474E37F, PH.base.pack) ;
        {
            float covariance[] =  {1.7250078E38F, 1.6597572E38F, -3.3449937E38F, 2.6677624E38F, 1.9270476E38F, 1.3629696E37F, -1.8444174E38F, -1.5628041E38F, 2.8566285E38F, 2.39489E38F, -3.3419977E38F, -1.6509677E38F, -1.6596258E38F, -5.353558E37F, 1.8653868E38F, -7.036635E37F, -2.3141298E38F, 2.4965267E38F, -2.067062E38F, 2.9809051E38F, -1.513602E38F, 3.4606415E37F, -3.3697607E38F, 2.958012E38F, -2.782608E38F, 5.922633E37F, 1.5916379E38F, -1.571914E38F, -7.55337E36F, -1.970722E38F, 1.082225E38F, -1.0061431E38F, -3.1080892E38F, -3.044912E37F, 3.580379E36F, -2.3510775E38F, 2.909231E38F, -2.8620338E37F, -1.2790894E38F, 2.0760592E38F, -1.4390315E38F, -1.8236772E38F, -9.606877E37F, -2.7597775E37F, 2.9291116E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vx_SET((float) -2.892513E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_z_SET((float)1.55938E38F, PH.base.pack) ;
        p64_az_SET((float) -3.3532192E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan6_raw_SET((uint16_t)(uint16_t)53160, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)4424, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)59300, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)21199, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)63063, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)40527, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)49534, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)37180, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)11886, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)56046, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)123295187L, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)46712, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)11211, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)38867, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)50750, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)27098, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)61509, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)353, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)50229, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)60249, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)32426, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)21203, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)24492, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -16344, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)19128, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)28160, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan7_raw_SET((uint16_t)(uint16_t)10385, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)53593, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)28720, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)4187, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)58739, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)42251, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)61998, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)65263, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param3_SET((float) -3.0292566E38F, PH.base.pack) ;
        p73_x_SET((int32_t)199609060, PH.base.pack) ;
        p73_y_SET((int32_t)928416433, PH.base.pack) ;
        p73_param2_SET((float)3.2073686E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p73_param1_SET((float) -1.8959855E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)17401, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND, PH.base.pack) ;
        p73_param4_SET((float) -7.4471874E36F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p73_z_SET((float)2.005527E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_climb_SET((float)1.9968639E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)43515, PH.base.pack) ;
        p74_airspeed_SET((float)4.093992E37F, PH.base.pack) ;
        p74_alt_SET((float)9.806183E37F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -12961, PH.base.pack) ;
        p74_groundspeed_SET((float) -1.8439098E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_param2_SET((float) -7.214364E37F, PH.base.pack) ;
        p75_y_SET((int32_t) -2004701205, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p75_param4_SET((float) -3.2105736E38F, PH.base.pack) ;
        p75_z_SET((float) -3.7584177E37F, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_RELAY, PH.base.pack) ;
        p75_param3_SET((float) -2.5265925E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p75_param1_SET((float)1.1513333E38F, PH.base.pack) ;
        p75_x_SET((int32_t)916018655, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p76_param3_SET((float)1.1372228E38F, PH.base.pack) ;
        p76_param2_SET((float) -2.2437582E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p76_param4_SET((float) -5.559202E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p76_param1_SET((float) -6.16854E37F, PH.base.pack) ;
        p76_param6_SET((float)6.501324E37F, PH.base.pack) ;
        p76_param7_SET((float)2.3864723E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_START_RX_PAIR, PH.base.pack) ;
        p76_param5_SET((float)2.807578E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)2, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)171, &PH) ;
        p77_result_param2_SET((int32_t) -622216880, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)124, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_yaw_SET((float)2.0505435E38F, PH.base.pack) ;
        p81_thrust_SET((float) -3.059072E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p81_roll_SET((float)1.7530843E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)346732401L, PH.base.pack) ;
        p81_pitch_SET((float)3.1491802E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_time_boot_ms_SET((uint32_t)3348316302L, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p82_thrust_SET((float) -2.4591272E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.9717005E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -1.5692799E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        {
            float q[] =  {-2.7977903E38F, 3.157124E38F, -3.0569477E38F, 9.025518E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float) -2.5995273E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_type_mask_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)2.5937085E38F, PH.base.pack) ;
        {
            float q[] =  {8.3714814E37F, -1.3839322E38F, 2.1511329E38F, -3.3106452E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)2.2535083E37F, PH.base.pack) ;
        p83_thrust_SET((float)3.6655717E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)1013579285L, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -1.5964063E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afz_SET((float)1.5313226E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.2519933E38F, PH.base.pack) ;
        p84_y_SET((float) -1.4119158E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float)2.0444245E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)39594, PH.base.pack) ;
        p84_z_SET((float) -1.8345414E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3723016848L, PH.base.pack) ;
        p84_vx_SET((float) -3.3444525E38F, PH.base.pack) ;
        p84_x_SET((float)2.8099092E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p84_yaw_SET((float) -1.7453833E38F, PH.base.pack) ;
        p84_vy_SET((float) -1.9653446E38F, PH.base.pack) ;
        p84_vz_SET((float)1.2724967E37F, PH.base.pack) ;
        p84_afx_SET((float)2.3955136E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_afx_SET((float) -1.83156E38F, PH.base.pack) ;
        p86_vy_SET((float)1.3126874E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p86_alt_SET((float) -2.6305186E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.0534605E38F, PH.base.pack) ;
        p86_vx_SET((float) -2.9169097E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)32079, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2860700765L, PH.base.pack) ;
        p86_afy_SET((float) -1.3162108E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -357258785, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1927959094, PH.base.pack) ;
        p86_yaw_SET((float)6.531067E37F, PH.base.pack) ;
        p86_vz_SET((float) -2.9608262E38F, PH.base.pack) ;
        p86_afz_SET((float)2.6953945E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afz_SET((float) -2.9991818E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)3722973485L, PH.base.pack) ;
        p87_afx_SET((float) -9.322101E37F, PH.base.pack) ;
        p87_alt_SET((float) -1.8553971E38F, PH.base.pack) ;
        p87_vy_SET((float) -8.928258E37F, PH.base.pack) ;
        p87_vz_SET((float) -1.0659862E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float) -3.1254315E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)64037149, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)29692, PH.base.pack) ;
        p87_lon_int_SET((int32_t)283447721, PH.base.pack) ;
        p87_afy_SET((float)7.9995446E36F, PH.base.pack) ;
        p87_vx_SET((float)8.515631E37F, PH.base.pack) ;
        p87_yaw_SET((float)3.2509044E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_roll_SET((float)2.8697155E38F, PH.base.pack) ;
        p89_x_SET((float) -1.7460009E38F, PH.base.pack) ;
        p89_pitch_SET((float)1.9429466E37F, PH.base.pack) ;
        p89_z_SET((float)1.9170575E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2367025658L, PH.base.pack) ;
        p89_y_SET((float) -8.318043E37F, PH.base.pack) ;
        p89_yaw_SET((float)2.7643186E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_yawspeed_SET((float) -1.062251E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -14321, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6237614395626182240L, PH.base.pack) ;
        p90_rollspeed_SET((float) -1.1489919E38F, PH.base.pack) ;
        p90_roll_SET((float) -3.1154694E38F, PH.base.pack) ;
        p90_pitch_SET((float) -7.3767697E37F, PH.base.pack) ;
        p90_alt_SET((int32_t)1134764075, PH.base.pack) ;
        p90_lat_SET((int32_t) -1870703239, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -9669, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)14956, PH.base.pack) ;
        p90_yaw_SET((float)2.1192041E37F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -15632, PH.base.pack) ;
        p90_lon_SET((int32_t)1683014417, PH.base.pack) ;
        p90_pitchspeed_SET((float) -4.2990567E36F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -15346, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)14552, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_yaw_rudder_SET((float) -3.2267306E37F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -1.1589304E38F, PH.base.pack) ;
        p91_aux1_SET((float) -1.4540211E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)1688263909165782869L, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        p91_aux2_SET((float)9.865074E37F, PH.base.pack) ;
        p91_aux4_SET((float) -1.2625034E38F, PH.base.pack) ;
        p91_aux3_SET((float)2.133539E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)8.942426E37F, PH.base.pack) ;
        p91_throttle_SET((float)2.6573046E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan12_raw_SET((uint16_t)(uint16_t)57732, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)12930, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)7728, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)57623, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)36876, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)52050, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)40820, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)6380605128291382382L, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)60403, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)24240, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)23682, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)2333, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)48191, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)5591373957122626965L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        {
            float controls[] =  {-6.9457325E34F, 9.588394E37F, -1.6412083E38F, -1.5688698E38F, -2.0872066E37F, -4.215634E37F, 1.9843924E38F, 6.154986E36F, 1.947034E38F, 1.1911214E38F, -2.0842131E38F, 2.622365E38F, -3.3010068E37F, -2.3603376E38F, -1.1928385E38F, -3.1356106E37F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_time_usec_SET((uint64_t)7538939790698347276L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_y_SET((int16_t)(int16_t) -1633, PH.base.pack) ;
        p100_ground_distance_SET((float) -2.5729028E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)2.5538766E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)2.3880548E38F, &PH) ;
        p100_flow_x_SET((int16_t)(int16_t)12873, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)2876477832941302416L, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.145519E38F, &PH) ;
        p100_flow_comp_m_y_SET((float)2.0379802E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_roll_SET((float) -1.5907354E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)5072099899845589435L, PH.base.pack) ;
        p101_pitch_SET((float)1.1661158E38F, PH.base.pack) ;
        p101_yaw_SET((float) -4.2893443E37F, PH.base.pack) ;
        p101_y_SET((float)1.3845789E38F, PH.base.pack) ;
        p101_x_SET((float)1.3041516E37F, PH.base.pack) ;
        p101_z_SET((float) -2.8426186E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_z_SET((float)3.0613154E38F, PH.base.pack) ;
        p102_roll_SET((float)2.595175E38F, PH.base.pack) ;
        p102_yaw_SET((float)7.9243273E37F, PH.base.pack) ;
        p102_x_SET((float) -2.745278E38F, PH.base.pack) ;
        p102_pitch_SET((float) -1.2704027E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)6768382064556339294L, PH.base.pack) ;
        p102_y_SET((float)8.1284266E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -1.5690088E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)4757277920096620815L, PH.base.pack) ;
        p103_x_SET((float) -2.3506861E37F, PH.base.pack) ;
        p103_y_SET((float)7.76821E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_x_SET((float)2.7020956E37F, PH.base.pack) ;
        p104_y_SET((float) -8.783111E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)1057352700114596413L, PH.base.pack) ;
        p104_roll_SET((float)1.998713E38F, PH.base.pack) ;
        p104_z_SET((float)1.7282014E37F, PH.base.pack) ;
        p104_pitch_SET((float) -1.321195E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.5387036E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_temperature_SET((float) -8.16845E37F, PH.base.pack) ;
        p105_xgyro_SET((float)2.8875123E38F, PH.base.pack) ;
        p105_ygyro_SET((float)2.340282E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -5.3251086E37F, PH.base.pack) ;
        p105_zmag_SET((float)1.9945237E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)61970, PH.base.pack) ;
        p105_xacc_SET((float)2.2996701E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)2.017198E38F, PH.base.pack) ;
        p105_ymag_SET((float)6.564841E37F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.5111073E38F, PH.base.pack) ;
        p105_xmag_SET((float) -1.0109421E37F, PH.base.pack) ;
        p105_zacc_SET((float)1.6075153E38F, PH.base.pack) ;
        p105_yacc_SET((float)1.9359769E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)164606632381597214L, PH.base.pack) ;
        p105_pressure_alt_SET((float) -1.4918057E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_ygyro_SET((float)1.6622408E38F, PH.base.pack) ;
        p106_integrated_x_SET((float)2.1570026E37F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3718457175L, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)5478967713702435458L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.108388E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p106_integrated_y_SET((float) -3.0598207E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -2.3346915E38F, PH.base.pack) ;
        p106_distance_SET((float)1.2486697E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -20839, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1300120916L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_xacc_SET((float) -1.544409E38F, PH.base.pack) ;
        p107_temperature_SET((float) -6.8936954E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -3.3190026E38F, PH.base.pack) ;
        p107_zacc_SET((float)2.8934886E38F, PH.base.pack) ;
        p107_xmag_SET((float) -6.28088E37F, PH.base.pack) ;
        p107_ygyro_SET((float) -2.8887137E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)7826221670743917278L, PH.base.pack) ;
        p107_zmag_SET((float)9.455484E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -2.9189357E38F, PH.base.pack) ;
        p107_yacc_SET((float) -2.0933402E38F, PH.base.pack) ;
        p107_ymag_SET((float) -1.9506064E38F, PH.base.pack) ;
        p107_zgyro_SET((float)2.9397894E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)5.942866E37F, PH.base.pack) ;
        p107_xgyro_SET((float) -1.3944457E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2518383931L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_q4_SET((float) -2.4594424E38F, PH.base.pack) ;
        p108_vn_SET((float) -1.783158E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.579776E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.6902961E38F, PH.base.pack) ;
        p108_roll_SET((float) -1.9949555E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -2.101267E38F, PH.base.pack) ;
        p108_yaw_SET((float)3.0329727E38F, PH.base.pack) ;
        p108_yacc_SET((float) -1.8147777E38F, PH.base.pack) ;
        p108_ygyro_SET((float)3.310029E38F, PH.base.pack) ;
        p108_xacc_SET((float)6.31271E37F, PH.base.pack) ;
        p108_alt_SET((float) -1.7326378E38F, PH.base.pack) ;
        p108_lon_SET((float) -3.344497E38F, PH.base.pack) ;
        p108_zacc_SET((float) -1.2597261E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -2.3790303E38F, PH.base.pack) ;
        p108_q1_SET((float) -4.434584E37F, PH.base.pack) ;
        p108_xgyro_SET((float)3.0570045E38F, PH.base.pack) ;
        p108_lat_SET((float) -2.5837236E38F, PH.base.pack) ;
        p108_q2_SET((float)1.7836552E38F, PH.base.pack) ;
        p108_ve_SET((float)2.841509E38F, PH.base.pack) ;
        p108_q3_SET((float) -8.327572E37F, PH.base.pack) ;
        p108_vd_SET((float) -1.6003793E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_fixed__SET((uint16_t)(uint16_t)18774, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)58370, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)184, (uint8_t)51, (uint8_t)203, (uint8_t)72, (uint8_t)186, (uint8_t)158, (uint8_t)121, (uint8_t)166, (uint8_t)91, (uint8_t)68, (uint8_t)137, (uint8_t)129, (uint8_t)252, (uint8_t)244, (uint8_t)67, (uint8_t)248, (uint8_t)223, (uint8_t)13, (uint8_t)165, (uint8_t)156, (uint8_t)118, (uint8_t)61, (uint8_t)98, (uint8_t)85, (uint8_t)22, (uint8_t)233, (uint8_t)10, (uint8_t)240, (uint8_t)84, (uint8_t)203, (uint8_t)33, (uint8_t)200, (uint8_t)99, (uint8_t)101, (uint8_t)254, (uint8_t)12, (uint8_t)110, (uint8_t)187, (uint8_t)244, (uint8_t)100, (uint8_t)101, (uint8_t)161, (uint8_t)181, (uint8_t)175, (uint8_t)172, (uint8_t)48, (uint8_t)85, (uint8_t)131, (uint8_t)192, (uint8_t)91, (uint8_t)214, (uint8_t)49, (uint8_t)241, (uint8_t)9, (uint8_t)90, (uint8_t)249, (uint8_t)240, (uint8_t)38, (uint8_t)7, (uint8_t)170, (uint8_t)95, (uint8_t)122, (uint8_t)253, (uint8_t)154, (uint8_t)223, (uint8_t)214, (uint8_t)206, (uint8_t)136, (uint8_t)86, (uint8_t)173, (uint8_t)129, (uint8_t)186, (uint8_t)211, (uint8_t)45, (uint8_t)103, (uint8_t)142, (uint8_t)118, (uint8_t)139, (uint8_t)28, (uint8_t)69, (uint8_t)130, (uint8_t)138, (uint8_t)246, (uint8_t)56, (uint8_t)143, (uint8_t)190, (uint8_t)99, (uint8_t)85, (uint8_t)181, (uint8_t)152, (uint8_t)94, (uint8_t)90, (uint8_t)131, (uint8_t)59, (uint8_t)112, (uint8_t)81, (uint8_t)38, (uint8_t)74, (uint8_t)71, (uint8_t)171, (uint8_t)135, (uint8_t)180, (uint8_t)234, (uint8_t)181, (uint8_t)153, (uint8_t)104, (uint8_t)232, (uint8_t)17, (uint8_t)70, (uint8_t)178, (uint8_t)109, (uint8_t)169, (uint8_t)88, (uint8_t)184, (uint8_t)207, (uint8_t)152, (uint8_t)47, (uint8_t)185, (uint8_t)236, (uint8_t)26, (uint8_t)171, (uint8_t)53, (uint8_t)75, (uint8_t)106, (uint8_t)209, (uint8_t)36, (uint8_t)222, (uint8_t)97, (uint8_t)57, (uint8_t)83, (uint8_t)140, (uint8_t)134, (uint8_t)154, (uint8_t)34, (uint8_t)77, (uint8_t)183, (uint8_t)73, (uint8_t)233, (uint8_t)242, (uint8_t)43, (uint8_t)253, (uint8_t)219, (uint8_t)253, (uint8_t)88, (uint8_t)27, (uint8_t)142, (uint8_t)163, (uint8_t)79, (uint8_t)226, (uint8_t)82, (uint8_t)129, (uint8_t)215, (uint8_t)191, (uint8_t)104, (uint8_t)64, (uint8_t)111, (uint8_t)122, (uint8_t)140, (uint8_t)121, (uint8_t)182, (uint8_t)130, (uint8_t)17, (uint8_t)19, (uint8_t)162, (uint8_t)125, (uint8_t)147, (uint8_t)14, (uint8_t)118, (uint8_t)138, (uint8_t)229, (uint8_t)102, (uint8_t)158, (uint8_t)130, (uint8_t)107, (uint8_t)76, (uint8_t)91, (uint8_t)86, (uint8_t)54, (uint8_t)228, (uint8_t)162, (uint8_t)55, (uint8_t)87, (uint8_t)24, (uint8_t)12, (uint8_t)26, (uint8_t)132, (uint8_t)17, (uint8_t)190, (uint8_t)227, (uint8_t)41, (uint8_t)254, (uint8_t)150, (uint8_t)222, (uint8_t)185, (uint8_t)37, (uint8_t)6, (uint8_t)77, (uint8_t)131, (uint8_t)104, (uint8_t)124, (uint8_t)236, (uint8_t)177, (uint8_t)66, (uint8_t)129, (uint8_t)14, (uint8_t)180, (uint8_t)17, (uint8_t)110, (uint8_t)96, (uint8_t)134, (uint8_t)188, (uint8_t)7, (uint8_t)141, (uint8_t)244, (uint8_t)42, (uint8_t)231, (uint8_t)55, (uint8_t)110, (uint8_t)232, (uint8_t)114, (uint8_t)172, (uint8_t)67, (uint8_t)118, (uint8_t)189, (uint8_t)147, (uint8_t)163, (uint8_t)152, (uint8_t)68, (uint8_t)108, (uint8_t)60, (uint8_t)157, (uint8_t)109, (uint8_t)83, (uint8_t)237, (uint8_t)142, (uint8_t)226, (uint8_t)95, (uint8_t)115, (uint8_t)166, (uint8_t)117, (uint8_t)104, (uint8_t)253, (uint8_t)244, (uint8_t)203, (uint8_t)237, (uint8_t)62, (uint8_t)58, (uint8_t)7, (uint8_t)143, (uint8_t)30, (uint8_t)18};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)3391325052206998022L, PH.base.pack) ;
        p111_ts1_SET((int64_t)4400939869508876977L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)68510322077938214L, PH.base.pack) ;
        p112_seq_SET((uint32_t)837452302L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t) -25746, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -25020, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)2544601951611260644L, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)21190, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)53096, PH.base.pack) ;
        p113_lon_SET((int32_t)1782107865, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p113_lat_SET((int32_t) -1417756295, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)27091, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p113_alt_SET((int32_t)673712895, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)50959, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)3347, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_xgyro_SET((float)1.0876661E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)7798136658250401868L, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.3367782E38F, PH.base.pack) ;
        p114_distance_SET((float) -2.1574509E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1344308918L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)2.8032574E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -25447, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.7142904E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)2.824809E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3197318107L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        {
            float attitude_quaternion[] =  {1.9065962E38F, -2.9552718E38F, 5.0015063E37F, 2.4556554E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_vy_SET((int16_t)(int16_t) -18829, PH.base.pack) ;
        p115_pitchspeed_SET((float)3.30891E38F, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)8943, PH.base.pack) ;
        p115_rollspeed_SET((float)1.9170913E38F, PH.base.pack) ;
        p115_lon_SET((int32_t) -1461145352, PH.base.pack) ;
        p115_alt_SET((int32_t) -1548855636, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)4188623246934525087L, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)9695, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -1181, PH.base.pack) ;
        p115_yawspeed_SET((float)2.1577567E38F, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)57598, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)29804, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)31169, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -22941, PH.base.pack) ;
        p115_lat_SET((int32_t) -1062434343, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_zgyro_SET((int16_t)(int16_t) -7796, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -30084, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)17311, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -26646, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)1500903903L, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -7438, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -26555, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)4114, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -2278, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)19152, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)27631, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)10334, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_num_logs_SET((uint16_t)(uint16_t)3001, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)729912963L, PH.base.pack) ;
        p118_size_SET((uint32_t)4240443197L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)64868, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)6686, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2169406938L, PH.base.pack) ;
        p119_count_SET((uint32_t)2192094089L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)6130, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)45076, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p120_ofs_SET((uint32_t)3456955194L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)164, (uint8_t)155, (uint8_t)184, (uint8_t)202, (uint8_t)25, (uint8_t)15, (uint8_t)128, (uint8_t)20, (uint8_t)76, (uint8_t)208, (uint8_t)47, (uint8_t)238, (uint8_t)205, (uint8_t)110, (uint8_t)165, (uint8_t)156, (uint8_t)110, (uint8_t)218, (uint8_t)167, (uint8_t)205, (uint8_t)14, (uint8_t)2, (uint8_t)192, (uint8_t)246, (uint8_t)115, (uint8_t)155, (uint8_t)126, (uint8_t)5, (uint8_t)30, (uint8_t)234, (uint8_t)134, (uint8_t)183, (uint8_t)4, (uint8_t)228, (uint8_t)32, (uint8_t)65, (uint8_t)131, (uint8_t)218, (uint8_t)68, (uint8_t)249, (uint8_t)47, (uint8_t)248, (uint8_t)28, (uint8_t)176, (uint8_t)120, (uint8_t)164, (uint8_t)42, (uint8_t)214, (uint8_t)192, (uint8_t)159, (uint8_t)173, (uint8_t)0, (uint8_t)171, (uint8_t)196, (uint8_t)227, (uint8_t)42, (uint8_t)65, (uint8_t)167, (uint8_t)30, (uint8_t)108, (uint8_t)189, (uint8_t)25, (uint8_t)232, (uint8_t)247, (uint8_t)94, (uint8_t)210, (uint8_t)66, (uint8_t)151, (uint8_t)12, (uint8_t)10, (uint8_t)171, (uint8_t)180, (uint8_t)55, (uint8_t)223, (uint8_t)125, (uint8_t)61, (uint8_t)137, (uint8_t)197, (uint8_t)237, (uint8_t)235, (uint8_t)56, (uint8_t)98, (uint8_t)65, (uint8_t)195, (uint8_t)38, (uint8_t)234, (uint8_t)41, (uint8_t)9, (uint8_t)126, (uint8_t)41};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)252, (uint8_t)15, (uint8_t)23, (uint8_t)207, (uint8_t)20, (uint8_t)46, (uint8_t)46, (uint8_t)110, (uint8_t)156, (uint8_t)194, (uint8_t)135, (uint8_t)93, (uint8_t)149, (uint8_t)141, (uint8_t)69, (uint8_t)25, (uint8_t)107, (uint8_t)207, (uint8_t)41, (uint8_t)65, (uint8_t)9, (uint8_t)154, (uint8_t)152, (uint8_t)134, (uint8_t)106, (uint8_t)121, (uint8_t)234, (uint8_t)210, (uint8_t)98, (uint8_t)246, (uint8_t)209, (uint8_t)168, (uint8_t)134, (uint8_t)4, (uint8_t)68, (uint8_t)48, (uint8_t)222, (uint8_t)142, (uint8_t)76, (uint8_t)95, (uint8_t)140, (uint8_t)66, (uint8_t)242, (uint8_t)167, (uint8_t)97, (uint8_t)112, (uint8_t)198, (uint8_t)15, (uint8_t)101, (uint8_t)97, (uint8_t)92, (uint8_t)37, (uint8_t)222, (uint8_t)163, (uint8_t)158, (uint8_t)230, (uint8_t)95, (uint8_t)18, (uint8_t)231, (uint8_t)227, (uint8_t)90, (uint8_t)233, (uint8_t)92, (uint8_t)75, (uint8_t)55, (uint8_t)232, (uint8_t)48, (uint8_t)42, (uint8_t)38, (uint8_t)33, (uint8_t)171, (uint8_t)164, (uint8_t)18, (uint8_t)188, (uint8_t)225, (uint8_t)106, (uint8_t)141, (uint8_t)254, (uint8_t)61, (uint8_t)106, (uint8_t)227, (uint8_t)132, (uint8_t)203, (uint8_t)137, (uint8_t)82, (uint8_t)166, (uint8_t)120, (uint8_t)200, (uint8_t)61, (uint8_t)181, (uint8_t)194, (uint8_t)148, (uint8_t)125, (uint8_t)91, (uint8_t)63, (uint8_t)131, (uint8_t)8, (uint8_t)137, (uint8_t)13, (uint8_t)247, (uint8_t)48, (uint8_t)212, (uint8_t)152, (uint8_t)85, (uint8_t)9, (uint8_t)213, (uint8_t)232, (uint8_t)73, (uint8_t)105, (uint8_t)10};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_epv_SET((uint16_t)(uint16_t)20889, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)27268, PH.base.pack) ;
        p124_lon_SET((int32_t)788441707, PH.base.pack) ;
        p124_lat_SET((int32_t)1945479960, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)52076, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)1848076250824157886L, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)3938637533L, PH.base.pack) ;
        p124_alt_SET((int32_t) -2066340423, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)11591, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)44899, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)28721, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)2935628589L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)163, (uint8_t)94, (uint8_t)210, (uint8_t)91, (uint8_t)78, (uint8_t)23, (uint8_t)151, (uint8_t)216, (uint8_t)79, (uint8_t)244, (uint8_t)25, (uint8_t)83, (uint8_t)218, (uint8_t)239, (uint8_t)14, (uint8_t)31, (uint8_t)225, (uint8_t)132, (uint8_t)40, (uint8_t)176, (uint8_t)29, (uint8_t)35, (uint8_t)96, (uint8_t)216, (uint8_t)204, (uint8_t)53, (uint8_t)249, (uint8_t)4, (uint8_t)58, (uint8_t)115, (uint8_t)84, (uint8_t)35, (uint8_t)5, (uint8_t)160, (uint8_t)54, (uint8_t)185, (uint8_t)16, (uint8_t)8, (uint8_t)43, (uint8_t)69, (uint8_t)113, (uint8_t)60, (uint8_t)250, (uint8_t)81, (uint8_t)250, (uint8_t)219, (uint8_t)129, (uint8_t)31, (uint8_t)56, (uint8_t)23, (uint8_t)253, (uint8_t)32, (uint8_t)160, (uint8_t)91, (uint8_t)147, (uint8_t)157, (uint8_t)184, (uint8_t)191, (uint8_t)119, (uint8_t)141, (uint8_t)24, (uint8_t)92, (uint8_t)134, (uint8_t)186, (uint8_t)115, (uint8_t)217, (uint8_t)126, (uint8_t)154, (uint8_t)141, (uint8_t)156};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_timeout_SET((uint16_t)(uint16_t)44561, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)2087229437L, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -2136166562, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1057323788L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)50887, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1885870731, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)1489766, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)1132530282L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1645021295, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_iar_num_hypotheses_SET((int32_t)1909077877, PH.base.pack) ;
        p128_tow_SET((uint32_t)2838881581L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)12004, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)562962203, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3730047014L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)313048227, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -110320269, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4202056894L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_yacc_SET((int16_t)(int16_t)2868, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)15768, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)9123, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -13164, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -5606, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -572, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)25676, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -18543, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1799482499L, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)1623, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_size_SET((uint32_t)2301328077L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)26132, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)54471, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)54648, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)20800, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)18, (uint8_t)58, (uint8_t)133, (uint8_t)55, (uint8_t)108, (uint8_t)168, (uint8_t)46, (uint8_t)22, (uint8_t)176, (uint8_t)227, (uint8_t)161, (uint8_t)237, (uint8_t)218, (uint8_t)218, (uint8_t)53, (uint8_t)24, (uint8_t)136, (uint8_t)83, (uint8_t)20, (uint8_t)188, (uint8_t)199, (uint8_t)14, (uint8_t)132, (uint8_t)3, (uint8_t)109, (uint8_t)114, (uint8_t)43, (uint8_t)119, (uint8_t)126, (uint8_t)146, (uint8_t)193, (uint8_t)225, (uint8_t)201, (uint8_t)8, (uint8_t)121, (uint8_t)58, (uint8_t)226, (uint8_t)140, (uint8_t)106, (uint8_t)4, (uint8_t)78, (uint8_t)228, (uint8_t)145, (uint8_t)97, (uint8_t)60, (uint8_t)223, (uint8_t)213, (uint8_t)33, (uint8_t)125, (uint8_t)43, (uint8_t)143, (uint8_t)23, (uint8_t)78, (uint8_t)43, (uint8_t)149, (uint8_t)122, (uint8_t)32, (uint8_t)136, (uint8_t)105, (uint8_t)81, (uint8_t)162, (uint8_t)76, (uint8_t)137, (uint8_t)245, (uint8_t)6, (uint8_t)206, (uint8_t)247, (uint8_t)158, (uint8_t)228, (uint8_t)238, (uint8_t)245, (uint8_t)103, (uint8_t)100, (uint8_t)209, (uint8_t)32, (uint8_t)200, (uint8_t)244, (uint8_t)5, (uint8_t)211, (uint8_t)19, (uint8_t)134, (uint8_t)216, (uint8_t)246, (uint8_t)253, (uint8_t)105, (uint8_t)32, (uint8_t)31, (uint8_t)58, (uint8_t)135, (uint8_t)54, (uint8_t)127, (uint8_t)251, (uint8_t)183, (uint8_t)228, (uint8_t)154, (uint8_t)126, (uint8_t)176, (uint8_t)53, (uint8_t)183, (uint8_t)155, (uint8_t)50, (uint8_t)105, (uint8_t)216, (uint8_t)137, (uint8_t)11, (uint8_t)43, (uint8_t)155, (uint8_t)45, (uint8_t)152, (uint8_t)127, (uint8_t)97, (uint8_t)210, (uint8_t)167, (uint8_t)195, (uint8_t)171, (uint8_t)123, (uint8_t)196, (uint8_t)211, (uint8_t)163, (uint8_t)11, (uint8_t)54, (uint8_t)61, (uint8_t)205, (uint8_t)202, (uint8_t)227, (uint8_t)42, (uint8_t)144, (uint8_t)222, (uint8_t)57, (uint8_t)32, (uint8_t)56, (uint8_t)76, (uint8_t)224, (uint8_t)7, (uint8_t)79, (uint8_t)158, (uint8_t)52, (uint8_t)56, (uint8_t)45, (uint8_t)129, (uint8_t)86, (uint8_t)148, (uint8_t)10, (uint8_t)35, (uint8_t)183, (uint8_t)237, (uint8_t)159, (uint8_t)173, (uint8_t)35, (uint8_t)35, (uint8_t)39, (uint8_t)67, (uint8_t)133, (uint8_t)226, (uint8_t)127, (uint8_t)135, (uint8_t)216, (uint8_t)179, (uint8_t)55, (uint8_t)238, (uint8_t)153, (uint8_t)253, (uint8_t)63, (uint8_t)57, (uint8_t)234, (uint8_t)202, (uint8_t)51, (uint8_t)105, (uint8_t)46, (uint8_t)201, (uint8_t)96, (uint8_t)190, (uint8_t)127, (uint8_t)67, (uint8_t)77, (uint8_t)250, (uint8_t)255, (uint8_t)59, (uint8_t)83, (uint8_t)188, (uint8_t)94, (uint8_t)109, (uint8_t)167, (uint8_t)211, (uint8_t)4, (uint8_t)255, (uint8_t)3, (uint8_t)215, (uint8_t)235, (uint8_t)7, (uint8_t)141, (uint8_t)232, (uint8_t)98, (uint8_t)201, (uint8_t)213, (uint8_t)247, (uint8_t)160, (uint8_t)186, (uint8_t)151, (uint8_t)214, (uint8_t)199, (uint8_t)149, (uint8_t)249, (uint8_t)149, (uint8_t)186, (uint8_t)253, (uint8_t)243, (uint8_t)195, (uint8_t)188, (uint8_t)34, (uint8_t)71, (uint8_t)250, (uint8_t)70, (uint8_t)145, (uint8_t)32, (uint8_t)164, (uint8_t)95, (uint8_t)140, (uint8_t)80, (uint8_t)76, (uint8_t)136, (uint8_t)245, (uint8_t)139, (uint8_t)190, (uint8_t)133, (uint8_t)146, (uint8_t)124, (uint8_t)89, (uint8_t)131, (uint8_t)170, (uint8_t)142, (uint8_t)98, (uint8_t)203, (uint8_t)104, (uint8_t)58, (uint8_t)232, (uint8_t)55, (uint8_t)14, (uint8_t)57, (uint8_t)226, (uint8_t)226, (uint8_t)253, (uint8_t)118, (uint8_t)180, (uint8_t)236, (uint8_t)248, (uint8_t)39, (uint8_t)38, (uint8_t)100, (uint8_t)85, (uint8_t)234, (uint8_t)169, (uint8_t)218};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_YAW_45, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)55116, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)30892, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)4677, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1076379289L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t)111543362, PH.base.pack) ;
        p133_mask_SET((uint64_t)757765720942836527L, PH.base.pack) ;
        p133_lon_SET((int32_t) -1191283920, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)36429, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)8767, PH.base.pack) ;
        p134_lon_SET((int32_t) -1230306497, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)9679, (int16_t)27335, (int16_t) -13923, (int16_t)4770, (int16_t) -20002, (int16_t)22873, (int16_t) -26869, (int16_t)5644, (int16_t) -11268, (int16_t)4336, (int16_t)14078, (int16_t)12847, (int16_t)19127, (int16_t)8190, (int16_t)29455, (int16_t) -7937};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lat_SET((int32_t) -1904838548, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t)2073044756, PH.base.pack) ;
        p135_lat_SET((int32_t) -1695931072, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)61538, PH.base.pack) ;
        p136_terrain_height_SET((float) -2.9021755E38F, PH.base.pack) ;
        p136_current_height_SET((float) -1.858375E38F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)1049, PH.base.pack) ;
        p136_lat_SET((int32_t) -2086115371, PH.base.pack) ;
        p136_lon_SET((int32_t) -195234269, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)19048, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float)9.045696E37F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)1430890528L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)26419, PH.base.pack) ;
        p137_press_abs_SET((float) -1.3592657E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float) -3.2886714E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)7626421284007542195L, PH.base.pack) ;
        p138_z_SET((float)5.1526492E36F, PH.base.pack) ;
        p138_y_SET((float) -2.8312932E38F, PH.base.pack) ;
        {
            float q[] =  {2.1926958E38F, -2.5558045E38F, -1.0133807E38F, -1.764377E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)4764698875336345703L, PH.base.pack) ;
        {
            float controls[] =  {-2.4449668E38F, -2.9697409E38F, -2.2237283E38F, -7.00052E37F, -7.311077E37F, -1.6090775E37F, -7.3563605E37F, -9.8801924E36F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)6627060401212112667L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        {
            float controls[] =  {1.285983E38F, 3.0769239E38F, 1.2346158E38F, -1.4833366E38F, 4.9829456E36F, 3.213132E38F, -7.071099E37F, 2.3048038E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float) -1.2572677E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -8.794429E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -3.2900285E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)8.0489135E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)2.947392E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -1.4610955E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)285187208009281337L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_transfer_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)0, (uint8_t)152, (uint8_t)227, (uint8_t)81, (uint8_t)99, (uint8_t)204, (uint8_t)23, (uint8_t)201, (uint8_t)74, (uint8_t)47, (uint8_t)141, (uint8_t)66, (uint8_t)75, (uint8_t)28, (uint8_t)100, (uint8_t)114, (uint8_t)223, (uint8_t)19, (uint8_t)27, (uint8_t)194, (uint8_t)202, (uint8_t)100, (uint8_t)178, (uint8_t)248, (uint8_t)23, (uint8_t)82, (uint8_t)229, (uint8_t)130, (uint8_t)79, (uint8_t)74, (uint8_t)28, (uint8_t)253, (uint8_t)32, (uint8_t)245, (uint8_t)190, (uint8_t)55, (uint8_t)173, (uint8_t)108, (uint8_t)88, (uint8_t)193, (uint8_t)213, (uint8_t)107, (uint8_t)16, (uint8_t)47, (uint8_t)167, (uint8_t)163, (uint8_t)100, (uint8_t)184, (uint8_t)20, (uint8_t)34, (uint8_t)64, (uint8_t)238, (uint8_t)65, (uint8_t)144, (uint8_t)225, (uint8_t)101, (uint8_t)85, (uint8_t)5, (uint8_t)245, (uint8_t)229, (uint8_t)234, (uint8_t)208, (uint8_t)73, (uint8_t)37, (uint8_t)169, (uint8_t)175, (uint8_t)190, (uint8_t)16, (uint8_t)152, (uint8_t)101, (uint8_t)110, (uint8_t)223, (uint8_t)241, (uint8_t)109, (uint8_t)211, (uint8_t)210, (uint8_t)50, (uint8_t)150, (uint8_t)186, (uint8_t)190, (uint8_t)139, (uint8_t)1, (uint8_t)114, (uint8_t)247, (uint8_t)126, (uint8_t)189, (uint8_t)180, (uint8_t)84, (uint8_t)81, (uint8_t)210, (uint8_t)17, (uint8_t)173, (uint8_t)58, (uint8_t)164, (uint8_t)46, (uint8_t)203, (uint8_t)53, (uint8_t)100, (uint8_t)113, (uint8_t)72, (uint8_t)252, (uint8_t)125, (uint8_t)114, (uint8_t)110, (uint8_t)42, (uint8_t)167, (uint8_t)98, (uint8_t)44, (uint8_t)108, (uint8_t)214, (uint8_t)222, (uint8_t)54, (uint8_t)52, (uint8_t)53, (uint8_t)89, (uint8_t)28, (uint8_t)146, (uint8_t)252, (uint8_t)56, (uint8_t)210};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        {
            uint8_t storage[] =  {(uint8_t)184, (uint8_t)68, (uint8_t)247, (uint8_t)194, (uint8_t)223, (uint8_t)62, (uint8_t)55, (uint8_t)25, (uint8_t)5, (uint8_t)111, (uint8_t)0, (uint8_t)28, (uint8_t)254, (uint8_t)3, (uint8_t)50, (uint8_t)134, (uint8_t)244, (uint8_t)186, (uint8_t)163, (uint8_t)62, (uint8_t)164, (uint8_t)115, (uint8_t)96, (uint8_t)46, (uint8_t)107, (uint8_t)219, (uint8_t)97, (uint8_t)60, (uint8_t)82, (uint8_t)111, (uint8_t)136, (uint8_t)50, (uint8_t)148, (uint8_t)34, (uint8_t)94, (uint8_t)82, (uint8_t)103, (uint8_t)9, (uint8_t)141, (uint8_t)172, (uint8_t)247, (uint8_t)109, (uint8_t)232, (uint8_t)20, (uint8_t)157, (uint8_t)112, (uint8_t)160, (uint8_t)60, (uint8_t)19, (uint8_t)220, (uint8_t)3, (uint8_t)114, (uint8_t)213, (uint8_t)27, (uint8_t)36, (uint8_t)235, (uint8_t)62, (uint8_t)191, (uint8_t)251, (uint8_t)57, (uint8_t)93, (uint8_t)134, (uint8_t)24, (uint8_t)0, (uint8_t)141, (uint8_t)156, (uint8_t)236, (uint8_t)37, (uint8_t)155, (uint8_t)245, (uint8_t)26, (uint8_t)254, (uint8_t)141, (uint8_t)215, (uint8_t)84, (uint8_t)204, (uint8_t)237, (uint8_t)109, (uint8_t)15, (uint8_t)16, (uint8_t)214, (uint8_t)116, (uint8_t)15, (uint8_t)203, (uint8_t)46, (uint8_t)84, (uint8_t)7, (uint8_t)48, (uint8_t)238, (uint8_t)167, (uint8_t)178, (uint8_t)133, (uint8_t)99, (uint8_t)9, (uint8_t)177, (uint8_t)35, (uint8_t)61, (uint8_t)139, (uint8_t)208, (uint8_t)174, (uint8_t)160, (uint8_t)56, (uint8_t)50, (uint8_t)108, (uint8_t)106, (uint8_t)41, (uint8_t)196, (uint8_t)118, (uint8_t)37, (uint8_t)58, (uint8_t)125, (uint8_t)28, (uint8_t)83, (uint8_t)58, (uint8_t)116, (uint8_t)26, (uint8_t)115, (uint8_t)103, (uint8_t)157, (uint8_t)33};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)1.7287671E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -2371, PH.base.pack) ;
        p143_press_diff_SET((float)1.5811225E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3192136812L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_timestamp_SET((uint64_t)7350658214342341882L, PH.base.pack) ;
        p144_lat_SET((int32_t)1642110997, PH.base.pack) ;
        {
            float position_cov[] =  {2.6399637E38F, -7.5049357E37F, 2.9603002E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {2.2424849E38F, -1.1978608E38F, 2.415125E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)1153132091, PH.base.pack) ;
        {
            float attitude_q[] =  {2.066759E38F, -3.308082E38F, -8.649907E37F, 2.012091E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        {
            float rates[] =  {-2.3655427E38F, -3.3705716E38F, -1.5373522E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)8143407707634026774L, PH.base.pack) ;
        {
            float acc[] =  {5.468965E36F, 1.8181203E38F, 3.8270519E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -1.6735704E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_roll_rate_SET((float) -2.2437304E38F, PH.base.pack) ;
        {
            float q[] =  {2.1570026E38F, -7.9232914E37F, 4.079743E37F, 7.4501393E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_yaw_rate_SET((float)1.3707618E37F, PH.base.pack) ;
        p146_x_pos_SET((float)7.4672936E36F, PH.base.pack) ;
        p146_x_vel_SET((float)1.4165854E38F, PH.base.pack) ;
        p146_z_vel_SET((float)8.542802E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.8484385E38F, PH.base.pack) ;
        p146_airspeed_SET((float)2.3333565E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-1.3197985E37F, 2.6671412E37F, -2.9831611E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_z_acc_SET((float) -1.6991726E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)8348686574283279071L, PH.base.pack) ;
        p146_y_acc_SET((float) -2.4173164E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {1.9789517E38F, -2.7346373E38F, -9.384848E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float) -2.956488E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -3.1330713E38F, PH.base.pack) ;
        p146_y_pos_SET((float)2.406145E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -3.1190225E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -74, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)735872958, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -32669, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -1581397532, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)31965, (uint16_t)2872, (uint16_t)54451, (uint16_t)2002, (uint16_t)15879, (uint16_t)51530, (uint16_t)10449, (uint16_t)8495, (uint16_t)10138, (uint16_t)14146};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_battery_SET((int16_t)(int16_t) -8515, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t flight_custom_version[] =  {(uint8_t)211, (uint8_t)144, (uint8_t)217, (uint8_t)153, (uint8_t)85, (uint8_t)232, (uint8_t)246, (uint8_t)186};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)195, (uint8_t)249, (uint8_t)154, (uint8_t)134, (uint8_t)64, (uint8_t)76, (uint8_t)231, (uint8_t)251};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)2148673780L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)600, PH.base.pack) ;
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2246494267L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)40110, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)182, (uint8_t)251, (uint8_t)147, (uint8_t)31, (uint8_t)156, (uint8_t)130, (uint8_t)116, (uint8_t)16};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)110, (uint8_t)218, (uint8_t)28, (uint8_t)135, (uint8_t)75, (uint8_t)195, (uint8_t)32, (uint8_t)91, (uint8_t)43, (uint8_t)191, (uint8_t)22, (uint8_t)197, (uint8_t)223, (uint8_t)144, (uint8_t)128, (uint8_t)0, (uint8_t)218, (uint8_t)90};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_os_sw_version_SET((uint32_t)252184840L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3000371771L, PH.base.pack) ;
        p148_uid_SET((uint64_t)1417252784304658752L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)5076760438945755179L, PH.base.pack) ;
        p149_angle_y_SET((float)2.2505696E38F, PH.base.pack) ;
        p149_size_y_SET((float)2.9775115E38F, PH.base.pack) ;
        {
            float q[] =  {9.511704E37F, -7.194361E37F, -9.826633E37F, 1.6267844E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_x_SET((float) -2.8514214E38F, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p149_angle_x_SET((float) -2.3783097E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)166, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p149_distance_SET((float) -1.9240638E38F, PH.base.pack) ;
        p149_y_SET((float) -1.387668E37F, &PH) ;
        p149_size_x_SET((float)6.1662926E37F, PH.base.pack) ;
        p149_z_SET((float)2.082231E38F, &PH) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCRIPT_ITEM_180(), &PH);
        p180_seq_SET((uint16_t)(uint16_t)35282, PH.base.pack) ;
        p180_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        {
            char16_t* name = u"ftTgmjdjlpiszpfbgjgBxbxnTjaaQplo";
            p180_name_SET_(name, &PH) ;
        }
        p180_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCRIPT_ITEM_180(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCRIPT_REQUEST_181(), &PH);
        p181_seq_SET((uint16_t)(uint16_t)31441, PH.base.pack) ;
        p181_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p181_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCRIPT_REQUEST_181(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCRIPT_REQUEST_LIST_182(), &PH);
        p182_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p182_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCRIPT_REQUEST_LIST_182(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCRIPT_COUNT_183(), &PH);
        p183_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p183_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p183_count_SET((uint16_t)(uint16_t)57196, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCRIPT_COUNT_183(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCRIPT_CURRENT_184(), &PH);
        p184_seq_SET((uint16_t)(uint16_t)26780, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCRIPT_CURRENT_184(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)1.3342913E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.9911634E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)1662667736514506330L, PH.base.pack) ;
        p230_mag_ratio_SET((float)2.41557E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -4.1036943E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.2325635E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -1.2830937E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)8.480281E37F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.4508246E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_wind_y_SET((float)2.870686E38F, PH.base.pack) ;
        p231_wind_z_SET((float)3.7594765E37F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -2.3929065E38F, PH.base.pack) ;
        p231_var_vert_SET((float)3.2111636E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)3.2084892E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)7182055481447931563L, PH.base.pack) ;
        p231_wind_x_SET((float) -3.2916544E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -1.55098E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -7.4296E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_horiz_accuracy_SET((float)5.9054107E37F, PH.base.pack) ;
        p232_alt_SET((float) -2.127398E38F, PH.base.pack) ;
        p232_vn_SET((float) -6.517405E37F, PH.base.pack) ;
        p232_vdop_SET((float) -2.5852927E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p232_lat_SET((int32_t) -791170523, PH.base.pack) ;
        p232_vert_accuracy_SET((float)2.7895288E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.1962028E38F, PH.base.pack) ;
        p232_ve_SET((float) -9.424775E37F, PH.base.pack) ;
        p232_vd_SET((float) -1.0264464E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p232_lon_SET((int32_t) -666836380, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p232_hdop_SET((float)2.2049202E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)11753, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)7190222732715917819L, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)152474736L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)132, (uint8_t)192, (uint8_t)144, (uint8_t)173, (uint8_t)103, (uint8_t)199, (uint8_t)41, (uint8_t)57, (uint8_t)170, (uint8_t)128, (uint8_t)182, (uint8_t)72, (uint8_t)12, (uint8_t)192, (uint8_t)118, (uint8_t)252, (uint8_t)40, (uint8_t)134, (uint8_t)210, (uint8_t)217, (uint8_t)19, (uint8_t)75, (uint8_t)51, (uint8_t)69, (uint8_t)177, (uint8_t)225, (uint8_t)191, (uint8_t)178, (uint8_t)244, (uint8_t)69, (uint8_t)149, (uint8_t)33, (uint8_t)36, (uint8_t)247, (uint8_t)173, (uint8_t)221, (uint8_t)159, (uint8_t)129, (uint8_t)1, (uint8_t)175, (uint8_t)207, (uint8_t)131, (uint8_t)160, (uint8_t)199, (uint8_t)200, (uint8_t)151, (uint8_t)174, (uint8_t)78, (uint8_t)176, (uint8_t)79, (uint8_t)9, (uint8_t)128, (uint8_t)251, (uint8_t)225, (uint8_t)72, (uint8_t)5, (uint8_t)4, (uint8_t)252, (uint8_t)224, (uint8_t)240, (uint8_t)137, (uint8_t)182, (uint8_t)49, (uint8_t)242, (uint8_t)47, (uint8_t)34, (uint8_t)44, (uint8_t)118, (uint8_t)130, (uint8_t)34, (uint8_t)105, (uint8_t)38, (uint8_t)215, (uint8_t)120, (uint8_t)182, (uint8_t)33, (uint8_t)137, (uint8_t)247, (uint8_t)176, (uint8_t)149, (uint8_t)105, (uint8_t)198, (uint8_t)237, (uint8_t)59, (uint8_t)217, (uint8_t)153, (uint8_t)186, (uint8_t)229, (uint8_t)54, (uint8_t)119, (uint8_t)148, (uint8_t)31, (uint8_t)111, (uint8_t)30, (uint8_t)226, (uint8_t)46, (uint8_t)162, (uint8_t)195, (uint8_t)152, (uint8_t)220, (uint8_t)145, (uint8_t)27, (uint8_t)19, (uint8_t)62, (uint8_t)115, (uint8_t)57, (uint8_t)157, (uint8_t)17, (uint8_t)191, (uint8_t)180, (uint8_t)210, (uint8_t)113, (uint8_t)122, (uint8_t)112, (uint8_t)207, (uint8_t)18, (uint8_t)85, (uint8_t)9, (uint8_t)230, (uint8_t)110, (uint8_t)131, (uint8_t)75, (uint8_t)204, (uint8_t)60, (uint8_t)132, (uint8_t)75, (uint8_t)47, (uint8_t)184, (uint8_t)237, (uint8_t)183, (uint8_t)30, (uint8_t)34, (uint8_t)244, (uint8_t)65, (uint8_t)227, (uint8_t)118, (uint8_t)237, (uint8_t)236, (uint8_t)41, (uint8_t)170, (uint8_t)28, (uint8_t)166, (uint8_t)163, (uint8_t)235, (uint8_t)109, (uint8_t)225, (uint8_t)62, (uint8_t)166, (uint8_t)174, (uint8_t)199, (uint8_t)92, (uint8_t)132, (uint8_t)80, (uint8_t)246, (uint8_t)28, (uint8_t)138, (uint8_t)104, (uint8_t)56, (uint8_t)246, (uint8_t)11, (uint8_t)2, (uint8_t)226, (uint8_t)57, (uint8_t)8, (uint8_t)131, (uint8_t)227, (uint8_t)92, (uint8_t)197, (uint8_t)111, (uint8_t)101, (uint8_t)185, (uint8_t)191, (uint8_t)92, (uint8_t)89, (uint8_t)53, (uint8_t)21, (uint8_t)179, (uint8_t)119, (uint8_t)53, (uint8_t)213};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_groundspeed_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)34, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -6847, PH.base.pack) ;
        p234_latitude_SET((int32_t)1818218974, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3410364383L, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)14885, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -40, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)63563, PH.base.pack) ;
        p234_longitude_SET((int32_t)1553936748, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)53, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)5294, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)5410, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -30424, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -7285, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)62, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_vibration_z_SET((float)1.3878362E38F, PH.base.pack) ;
        p241_vibration_y_SET((float)1.5764512E37F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3065876711L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)2601814907L, PH.base.pack) ;
        p241_vibration_x_SET((float)3.517538E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)7112826981975066358L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)3245080149L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {-2.2630208E38F, -1.2790843E38F, -1.2075706E38F, -1.8321488E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float) -1.8135237E38F, PH.base.pack) ;
        p242_x_SET((float) -1.311692E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -2.3858985E38F, PH.base.pack) ;
        p242_y_SET((float)2.5701798E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)971856752, PH.base.pack) ;
        p242_longitude_SET((int32_t) -1725962238, PH.base.pack) ;
        p242_approach_x_SET((float) -2.824099E37F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1982676518, PH.base.pack) ;
        p242_z_SET((float)1.2248006E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)2868915822043231427L, &PH) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        {
            float q[] =  {2.316682E38F, -1.8092025E38F, -2.702062E38F, 3.2273427E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_longitude_SET((int32_t)1041210971, PH.base.pack) ;
        p243_x_SET((float) -2.052369E38F, PH.base.pack) ;
        p243_y_SET((float)7.973893E37F, PH.base.pack) ;
        p243_z_SET((float) -1.4942327E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t) -187229160, PH.base.pack) ;
        p243_approach_z_SET((float)3.3072244E38F, PH.base.pack) ;
        p243_approach_x_SET((float) -7.7476837E37F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)6719374518839870456L, &PH) ;
        p243_approach_y_SET((float) -1.302164E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t) -1986629888, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)48559, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -1807338229, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_heading_SET((uint16_t)(uint16_t)11783, PH.base.pack) ;
        {
            char16_t* callsign = u"bbvjt";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN, PH.base.pack) ;
        p246_lat_SET((int32_t)1570758488, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)64533, PH.base.pack) ;
        p246_lon_SET((int32_t) -2133582170, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p246_altitude_SET((int32_t)1509011478, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -4319, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)4048866499L, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)15536, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_horizontal_minimum_delta_SET((float)3.1742691E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -9.183165E36F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_HORIZONTALLY, PH.base.pack) ;
        p247_id_SET((uint32_t)307192492L, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -3.2987826E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)209, (uint8_t)60, (uint8_t)241, (uint8_t)72, (uint8_t)249, (uint8_t)122, (uint8_t)251, (uint8_t)243, (uint8_t)222, (uint8_t)29, (uint8_t)88, (uint8_t)132, (uint8_t)34, (uint8_t)193, (uint8_t)153, (uint8_t)44, (uint8_t)126, (uint8_t)171, (uint8_t)148, (uint8_t)97, (uint8_t)177, (uint8_t)181, (uint8_t)150, (uint8_t)221, (uint8_t)210, (uint8_t)22, (uint8_t)237, (uint8_t)173, (uint8_t)63, (uint8_t)72, (uint8_t)112, (uint8_t)218, (uint8_t)109, (uint8_t)22, (uint8_t)137, (uint8_t)226, (uint8_t)154, (uint8_t)52, (uint8_t)171, (uint8_t)25, (uint8_t)162, (uint8_t)167, (uint8_t)238, (uint8_t)99, (uint8_t)21, (uint8_t)255, (uint8_t)208, (uint8_t)231, (uint8_t)171, (uint8_t)114, (uint8_t)19, (uint8_t)187, (uint8_t)218, (uint8_t)21, (uint8_t)61, (uint8_t)158, (uint8_t)104, (uint8_t)17, (uint8_t)230, (uint8_t)78, (uint8_t)208, (uint8_t)41, (uint8_t)217, (uint8_t)17, (uint8_t)180, (uint8_t)152, (uint8_t)39, (uint8_t)128, (uint8_t)220, (uint8_t)59, (uint8_t)205, (uint8_t)65, (uint8_t)75, (uint8_t)7, (uint8_t)156, (uint8_t)251, (uint8_t)147, (uint8_t)204, (uint8_t)57, (uint8_t)158, (uint8_t)142, (uint8_t)112, (uint8_t)111, (uint8_t)43, (uint8_t)148, (uint8_t)173, (uint8_t)190, (uint8_t)192, (uint8_t)72, (uint8_t)109, (uint8_t)99, (uint8_t)67, (uint8_t)200, (uint8_t)98, (uint8_t)187, (uint8_t)92, (uint8_t)137, (uint8_t)222, (uint8_t)202, (uint8_t)122, (uint8_t)114, (uint8_t)253, (uint8_t)211, (uint8_t)255, (uint8_t)199, (uint8_t)129, (uint8_t)36, (uint8_t)165, (uint8_t)165, (uint8_t)1, (uint8_t)128, (uint8_t)123, (uint8_t)15, (uint8_t)65, (uint8_t)117, (uint8_t)200, (uint8_t)203, (uint8_t)181, (uint8_t)37, (uint8_t)24, (uint8_t)99, (uint8_t)251, (uint8_t)180, (uint8_t)1, (uint8_t)211, (uint8_t)137, (uint8_t)155, (uint8_t)152, (uint8_t)249, (uint8_t)235, (uint8_t)158, (uint8_t)73, (uint8_t)191, (uint8_t)131, (uint8_t)22, (uint8_t)202, (uint8_t)198, (uint8_t)71, (uint8_t)48, (uint8_t)145, (uint8_t)156, (uint8_t)91, (uint8_t)63, (uint8_t)92, (uint8_t)253, (uint8_t)211, (uint8_t)229, (uint8_t)145, (uint8_t)145, (uint8_t)204, (uint8_t)37, (uint8_t)1, (uint8_t)137, (uint8_t)230, (uint8_t)33, (uint8_t)178, (uint8_t)5, (uint8_t)183, (uint8_t)98, (uint8_t)245, (uint8_t)115, (uint8_t)161, (uint8_t)235, (uint8_t)96, (uint8_t)88, (uint8_t)83, (uint8_t)186, (uint8_t)251, (uint8_t)99, (uint8_t)222, (uint8_t)28, (uint8_t)147, (uint8_t)236, (uint8_t)127, (uint8_t)247, (uint8_t)159, (uint8_t)54, (uint8_t)27, (uint8_t)47, (uint8_t)2, (uint8_t)183, (uint8_t)13, (uint8_t)104, (uint8_t)240, (uint8_t)55, (uint8_t)245, (uint8_t)124, (uint8_t)101, (uint8_t)150, (uint8_t)198, (uint8_t)34, (uint8_t)171, (uint8_t)116, (uint8_t)188, (uint8_t)133, (uint8_t)154, (uint8_t)251, (uint8_t)229, (uint8_t)111, (uint8_t)96, (uint8_t)35, (uint8_t)241, (uint8_t)236, (uint8_t)184, (uint8_t)161, (uint8_t)242, (uint8_t)15, (uint8_t)196, (uint8_t)14, (uint8_t)97, (uint8_t)157, (uint8_t)26, (uint8_t)172, (uint8_t)18, (uint8_t)16, (uint8_t)43, (uint8_t)219, (uint8_t)49, (uint8_t)69, (uint8_t)198, (uint8_t)103, (uint8_t)72, (uint8_t)89, (uint8_t)195, (uint8_t)38, (uint8_t)230, (uint8_t)66, (uint8_t)25, (uint8_t)29, (uint8_t)35, (uint8_t)103, (uint8_t)188, (uint8_t)53, (uint8_t)247, (uint8_t)143, (uint8_t)56, (uint8_t)88, (uint8_t)210, (uint8_t)75, (uint8_t)195, (uint8_t)89, (uint8_t)195, (uint8_t)177, (uint8_t)254, (uint8_t)252, (uint8_t)34, (uint8_t)18, (uint8_t)107, (uint8_t)171};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_message_type_SET((uint16_t)(uint16_t)49421, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)9305, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)80, (int8_t) -125, (int8_t) -24, (int8_t)81, (int8_t)48, (int8_t)45, (int8_t)72, (int8_t)92, (int8_t)83, (int8_t)63, (int8_t)107, (int8_t)50, (int8_t)110, (int8_t)45, (int8_t)78, (int8_t)88, (int8_t)28, (int8_t)34, (int8_t)49, (int8_t)36, (int8_t) -64, (int8_t) -59, (int8_t) -73, (int8_t) -116, (int8_t) -77, (int8_t) -12, (int8_t) -30, (int8_t)67, (int8_t) -45, (int8_t) -95, (int8_t)98, (int8_t)75};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float)2.0080038E38F, PH.base.pack) ;
        {
            char16_t* name = u"yweiu";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -1.3048703E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)4650371507381707764L, PH.base.pack) ;
        p250_x_SET((float)2.4529138E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"edPtsL";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)2238007142L, PH.base.pack) ;
        p251_value_SET((float)2.4445225E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t) -1435564799, PH.base.pack) ;
        {
            char16_t* name = u"as";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)197211436L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"umXqpvxkhjfiviutsdjivcifpRyfdvfyiwOwfqwilauupkYag";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p254_value_SET((float)2.894353E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1133347627L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)7111097222356842759L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)94, (uint8_t)135, (uint8_t)7, (uint8_t)171, (uint8_t)246, (uint8_t)108, (uint8_t)93, (uint8_t)10, (uint8_t)50, (uint8_t)12, (uint8_t)7, (uint8_t)15, (uint8_t)155, (uint8_t)185, (uint8_t)146, (uint8_t)117, (uint8_t)75, (uint8_t)149, (uint8_t)207, (uint8_t)30, (uint8_t)166, (uint8_t)57, (uint8_t)179, (uint8_t)44, (uint8_t)231, (uint8_t)73, (uint8_t)252, (uint8_t)24, (uint8_t)230, (uint8_t)231, (uint8_t)186, (uint8_t)111};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)1537249705L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)3272807597L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"oyocvpbmesWfkYvRnunbxrLvahwz";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)88, (uint8_t)224, (uint8_t)191, (uint8_t)144, (uint8_t)210, (uint8_t)206, (uint8_t)236, (uint8_t)19, (uint8_t)98, (uint8_t)20, (uint8_t)24, (uint8_t)110, (uint8_t)198, (uint8_t)186, (uint8_t)41, (uint8_t)138, (uint8_t)178, (uint8_t)159, (uint8_t)159, (uint8_t)174, (uint8_t)228, (uint8_t)115, (uint8_t)169, (uint8_t)242, (uint8_t)109, (uint8_t)2, (uint8_t)212, (uint8_t)215, (uint8_t)58, (uint8_t)179, (uint8_t)157, (uint8_t)84};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_h_SET((float) -1.7812122E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"kuesDqNjMbafnnvvNobszjaordnuvewmriLknbUabtJzfvlbsvwYdpcsqqiKxtmalzsf";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_time_boot_ms_SET((uint32_t)1137553395L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)36550, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)19659, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -4.868836E37F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)22616, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)3985822893L, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)199, (uint8_t)47, (uint8_t)29, (uint8_t)92, (uint8_t)71, (uint8_t)214, (uint8_t)211, (uint8_t)133, (uint8_t)60, (uint8_t)134, (uint8_t)46, (uint8_t)55, (uint8_t)194, (uint8_t)17, (uint8_t)148, (uint8_t)245, (uint8_t)201, (uint8_t)25, (uint8_t)166, (uint8_t)245, (uint8_t)105, (uint8_t)238, (uint8_t)148, (uint8_t)231, (uint8_t)213, (uint8_t)29, (uint8_t)50, (uint8_t)114, (uint8_t)57, (uint8_t)134, (uint8_t)12, (uint8_t)161};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_focal_length_SET((float) -1.5061874E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3248843086L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_total_capacity_SET((float) -7.5645716E37F, PH.base.pack) ;
        p261_used_capacity_SET((float)2.8568423E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3119234615L, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p261_read_speed_SET((float) -6.582356E37F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p261_write_speed_SET((float) -2.0145284E37F, PH.base.pack) ;
        p261_available_capacity_SET((float)1.025265E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_status_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2799026135L, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)1725655155L, PH.base.pack) ;
        p262_image_interval_SET((float)3.304827E38F, PH.base.pack) ;
        p262_available_capacity_SET((float)1.3053787E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_image_index_SET((int32_t)530902819, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)841051333L, PH.base.pack) ;
        p263_alt_SET((int32_t) -202031009, PH.base.pack) ;
        p263_lat_SET((int32_t) -976291802, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)7586065036574521422L, PH.base.pack) ;
        p263_lon_SET((int32_t)814775136, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        {
            float q[] =  {3.2164322E38F, 1.9580393E38F, 3.1587819E38F, -2.609203E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_relative_alt_SET((int32_t)1964828273, PH.base.pack) ;
        {
            char16_t* file_url = u"gmnprkOeOheyppprsuhbevtppzxwfrwliyugycgklcqszlwuUnbSp";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_capture_result_SET((int8_t)(int8_t) -15, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)2482331377318891808L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)8027458140053502740L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)4293329736L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)1866953520352671609L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_roll_SET((float) -5.4497283E37F, PH.base.pack) ;
        p265_pitch_SET((float)2.5208903E38F, PH.base.pack) ;
        p265_yaw_SET((float)2.8706193E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)3038542671L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_first_message_offset_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)49184, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)202, (uint8_t)10, (uint8_t)138, (uint8_t)216, (uint8_t)42, (uint8_t)206, (uint8_t)39, (uint8_t)203, (uint8_t)230, (uint8_t)100, (uint8_t)12, (uint8_t)205, (uint8_t)244, (uint8_t)123, (uint8_t)159, (uint8_t)13, (uint8_t)177, (uint8_t)104, (uint8_t)236, (uint8_t)184, (uint8_t)77, (uint8_t)114, (uint8_t)44, (uint8_t)255, (uint8_t)157, (uint8_t)79, (uint8_t)205, (uint8_t)83, (uint8_t)127, (uint8_t)16, (uint8_t)52, (uint8_t)167, (uint8_t)206, (uint8_t)40, (uint8_t)41, (uint8_t)101, (uint8_t)77, (uint8_t)127, (uint8_t)212, (uint8_t)215, (uint8_t)227, (uint8_t)199, (uint8_t)124, (uint8_t)160, (uint8_t)202, (uint8_t)255, (uint8_t)165, (uint8_t)97, (uint8_t)13, (uint8_t)53, (uint8_t)91, (uint8_t)186, (uint8_t)53, (uint8_t)218, (uint8_t)159, (uint8_t)252, (uint8_t)134, (uint8_t)242, (uint8_t)48, (uint8_t)118, (uint8_t)248, (uint8_t)228, (uint8_t)84, (uint8_t)88, (uint8_t)155, (uint8_t)217, (uint8_t)101, (uint8_t)15, (uint8_t)182, (uint8_t)199, (uint8_t)198, (uint8_t)123, (uint8_t)21, (uint8_t)146, (uint8_t)170, (uint8_t)253, (uint8_t)106, (uint8_t)107, (uint8_t)159, (uint8_t)122, (uint8_t)27, (uint8_t)37, (uint8_t)68, (uint8_t)62, (uint8_t)141, (uint8_t)102, (uint8_t)97, (uint8_t)130, (uint8_t)64, (uint8_t)8, (uint8_t)40, (uint8_t)40, (uint8_t)155, (uint8_t)72, (uint8_t)247, (uint8_t)195, (uint8_t)35, (uint8_t)216, (uint8_t)150, (uint8_t)237, (uint8_t)101, (uint8_t)47, (uint8_t)194, (uint8_t)79, (uint8_t)208, (uint8_t)139, (uint8_t)236, (uint8_t)25, (uint8_t)236, (uint8_t)235, (uint8_t)32, (uint8_t)110, (uint8_t)176, (uint8_t)19, (uint8_t)32, (uint8_t)59, (uint8_t)37, (uint8_t)12, (uint8_t)222, (uint8_t)144, (uint8_t)142, (uint8_t)68, (uint8_t)75, (uint8_t)55, (uint8_t)216, (uint8_t)92, (uint8_t)127, (uint8_t)219, (uint8_t)92, (uint8_t)181, (uint8_t)66, (uint8_t)150, (uint8_t)120, (uint8_t)187, (uint8_t)65, (uint8_t)221, (uint8_t)12, (uint8_t)91, (uint8_t)207, (uint8_t)188, (uint8_t)138, (uint8_t)139, (uint8_t)131, (uint8_t)179, (uint8_t)7, (uint8_t)182, (uint8_t)35, (uint8_t)235, (uint8_t)155, (uint8_t)215, (uint8_t)108, (uint8_t)46, (uint8_t)181, (uint8_t)46, (uint8_t)2, (uint8_t)124, (uint8_t)84, (uint8_t)58, (uint8_t)67, (uint8_t)64, (uint8_t)193, (uint8_t)235, (uint8_t)181, (uint8_t)209, (uint8_t)195, (uint8_t)237, (uint8_t)174, (uint8_t)55, (uint8_t)78, (uint8_t)153, (uint8_t)138, (uint8_t)236, (uint8_t)99, (uint8_t)232, (uint8_t)234, (uint8_t)216, (uint8_t)99, (uint8_t)6, (uint8_t)24, (uint8_t)194, (uint8_t)76, (uint8_t)8, (uint8_t)13, (uint8_t)124, (uint8_t)83, (uint8_t)154, (uint8_t)131, (uint8_t)202, (uint8_t)136, (uint8_t)151, (uint8_t)43, (uint8_t)61, (uint8_t)184, (uint8_t)173, (uint8_t)69, (uint8_t)76, (uint8_t)147, (uint8_t)87, (uint8_t)179, (uint8_t)109, (uint8_t)87, (uint8_t)27, (uint8_t)190, (uint8_t)51, (uint8_t)190, (uint8_t)2, (uint8_t)102, (uint8_t)49, (uint8_t)44, (uint8_t)111, (uint8_t)137, (uint8_t)132, (uint8_t)207, (uint8_t)149, (uint8_t)36, (uint8_t)22, (uint8_t)224, (uint8_t)67, (uint8_t)54, (uint8_t)77, (uint8_t)96, (uint8_t)202, (uint8_t)231, (uint8_t)205, (uint8_t)190, (uint8_t)65, (uint8_t)1, (uint8_t)168, (uint8_t)7, (uint8_t)181, (uint8_t)149, (uint8_t)139, (uint8_t)174, (uint8_t)213, (uint8_t)175, (uint8_t)253, (uint8_t)190, (uint8_t)235, (uint8_t)151, (uint8_t)212, (uint8_t)2, (uint8_t)224, (uint8_t)212, (uint8_t)130, (uint8_t)145, (uint8_t)66, (uint8_t)162, (uint8_t)100, (uint8_t)41};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_first_message_offset_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)13992, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)98, (uint8_t)109, (uint8_t)146, (uint8_t)170, (uint8_t)144, (uint8_t)208, (uint8_t)249, (uint8_t)47, (uint8_t)114, (uint8_t)234, (uint8_t)134, (uint8_t)40, (uint8_t)240, (uint8_t)55, (uint8_t)236, (uint8_t)98, (uint8_t)160, (uint8_t)198, (uint8_t)238, (uint8_t)57, (uint8_t)184, (uint8_t)108, (uint8_t)164, (uint8_t)181, (uint8_t)95, (uint8_t)92, (uint8_t)43, (uint8_t)129, (uint8_t)38, (uint8_t)31, (uint8_t)130, (uint8_t)139, (uint8_t)151, (uint8_t)58, (uint8_t)22, (uint8_t)42, (uint8_t)95, (uint8_t)200, (uint8_t)43, (uint8_t)4, (uint8_t)163, (uint8_t)102, (uint8_t)188, (uint8_t)87, (uint8_t)168, (uint8_t)144, (uint8_t)182, (uint8_t)102, (uint8_t)15, (uint8_t)138, (uint8_t)164, (uint8_t)203, (uint8_t)86, (uint8_t)107, (uint8_t)105, (uint8_t)7, (uint8_t)191, (uint8_t)104, (uint8_t)41, (uint8_t)60, (uint8_t)186, (uint8_t)146, (uint8_t)49, (uint8_t)160, (uint8_t)56, (uint8_t)240, (uint8_t)162, (uint8_t)136, (uint8_t)209, (uint8_t)130, (uint8_t)78, (uint8_t)197, (uint8_t)26, (uint8_t)25, (uint8_t)81, (uint8_t)93, (uint8_t)0, (uint8_t)114, (uint8_t)45, (uint8_t)255, (uint8_t)158, (uint8_t)41, (uint8_t)198, (uint8_t)7, (uint8_t)56, (uint8_t)57, (uint8_t)217, (uint8_t)10, (uint8_t)212, (uint8_t)4, (uint8_t)237, (uint8_t)149, (uint8_t)118, (uint8_t)21, (uint8_t)73, (uint8_t)148, (uint8_t)174, (uint8_t)45, (uint8_t)254, (uint8_t)245, (uint8_t)219, (uint8_t)151, (uint8_t)165, (uint8_t)14, (uint8_t)232, (uint8_t)90, (uint8_t)219, (uint8_t)78, (uint8_t)184, (uint8_t)12, (uint8_t)131, (uint8_t)75, (uint8_t)111, (uint8_t)146, (uint8_t)95, (uint8_t)39, (uint8_t)9, (uint8_t)181, (uint8_t)30, (uint8_t)134, (uint8_t)125, (uint8_t)65, (uint8_t)181, (uint8_t)148, (uint8_t)39, (uint8_t)150, (uint8_t)103, (uint8_t)204, (uint8_t)208, (uint8_t)102, (uint8_t)76, (uint8_t)164, (uint8_t)187, (uint8_t)124, (uint8_t)151, (uint8_t)153, (uint8_t)3, (uint8_t)214, (uint8_t)225, (uint8_t)43, (uint8_t)64, (uint8_t)57, (uint8_t)211, (uint8_t)239, (uint8_t)0, (uint8_t)244, (uint8_t)180, (uint8_t)151, (uint8_t)186, (uint8_t)73, (uint8_t)206, (uint8_t)241, (uint8_t)222, (uint8_t)45, (uint8_t)35, (uint8_t)216, (uint8_t)227, (uint8_t)176, (uint8_t)245, (uint8_t)221, (uint8_t)236, (uint8_t)157, (uint8_t)28, (uint8_t)236, (uint8_t)10, (uint8_t)114, (uint8_t)195, (uint8_t)9, (uint8_t)220, (uint8_t)170, (uint8_t)7, (uint8_t)126, (uint8_t)180, (uint8_t)41, (uint8_t)50, (uint8_t)136, (uint8_t)77, (uint8_t)129, (uint8_t)122, (uint8_t)167, (uint8_t)145, (uint8_t)113, (uint8_t)107, (uint8_t)130, (uint8_t)17, (uint8_t)85, (uint8_t)86, (uint8_t)30, (uint8_t)10, (uint8_t)243, (uint8_t)64, (uint8_t)74, (uint8_t)32, (uint8_t)202, (uint8_t)246, (uint8_t)65, (uint8_t)179, (uint8_t)220, (uint8_t)95, (uint8_t)13, (uint8_t)116, (uint8_t)228, (uint8_t)103, (uint8_t)171, (uint8_t)156, (uint8_t)134, (uint8_t)115, (uint8_t)109, (uint8_t)126, (uint8_t)220, (uint8_t)149, (uint8_t)243, (uint8_t)219, (uint8_t)197, (uint8_t)227, (uint8_t)12, (uint8_t)188, (uint8_t)41, (uint8_t)227, (uint8_t)40, (uint8_t)180, (uint8_t)92, (uint8_t)18, (uint8_t)176, (uint8_t)247, (uint8_t)53, (uint8_t)225, (uint8_t)105, (uint8_t)196, (uint8_t)12, (uint8_t)104, (uint8_t)112, (uint8_t)97, (uint8_t)119, (uint8_t)44, (uint8_t)254, (uint8_t)161, (uint8_t)171, (uint8_t)30, (uint8_t)85, (uint8_t)165, (uint8_t)31, (uint8_t)85, (uint8_t)36, (uint8_t)127, (uint8_t)31, (uint8_t)216, (uint8_t)132, (uint8_t)119};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_length_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)27573, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_framerate_SET((float)9.24535E37F, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)21514, PH.base.pack) ;
        {
            char16_t* uri = u"kdwhpguuaibxmpXSfrcvtmqlclvekuqlcwInrEyjlf";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_bitrate_SET((uint32_t)902381921L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)58018, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)38983, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_camera_id_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        {
            char16_t* uri = u"JqaigwrsFnqxsrmskgnfgufvwymasLjsondgymrghmdoIffniuwWcjbbrewhpxwwbgpwkoFevtyimdceegyrAjwjyjcbvljtmxhcgqiwnmlhVnybkonzfccajKwt";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_resolution_h_SET((uint16_t)(uint16_t)49052, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)27930, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2252073493L, PH.base.pack) ;
        p270_framerate_SET((float) -1.8604894E38F, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)42710, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"rfdfbxbvowKcctgkztivdssdkhkgBngk";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"zeMQflMnly";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        {
            uint8_t spec_version_hash[] =  {(uint8_t)115, (uint8_t)107, (uint8_t)237, (uint8_t)184, (uint8_t)113, (uint8_t)214, (uint8_t)228, (uint8_t)5};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)43070, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)44919, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)54251, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)80, (uint8_t)109, (uint8_t)101, (uint8_t)89, (uint8_t)62, (uint8_t)255, (uint8_t)88, (uint8_t)23};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)3341117272L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)4991602045285669298L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)31477, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_vcs_commit_SET((uint32_t)2030341977L, PH.base.pack) ;
        {
            char16_t* name = u"lrnkrwztiqrrclhGuxrvahopyw";
            p311_name_SET_(name, &PH) ;
        }
        {
            uint8_t hw_unique_id[] =  {(uint8_t)4, (uint8_t)10, (uint8_t)111, (uint8_t)237, (uint8_t)104, (uint8_t)43, (uint8_t)188, (uint8_t)200, (uint8_t)219, (uint8_t)108, (uint8_t)142, (uint8_t)59, (uint8_t)82, (uint8_t)233, (uint8_t)226, (uint8_t)28};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_minor_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)1782529378361858282L, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1963456487L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t) -13348, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        {
            char16_t* param_id = u"xkmnicyqldfU";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_value = u"hwsqmomllycwdznandjbhzlgygYtvfuGfuSvmwfkxadatrctlaxvqHnxkal";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)8766, PH.base.pack) ;
        {
            char16_t* param_id = u"kfypjqyfgui";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)13224, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        {
            char16_t* param_id = u"ykyqjDvagQscf";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        {
            char16_t* param_value = u"hlebxbuabHxyemwoyFratwzurrrvsggnxcRofasddxlQwtld";
            p323_param_value_SET_(param_value, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"irxmywjmumbcisQbTfxtxxktvyrmjocyckdpxrxysjfsustbwjscmfbiiBuWHaedymuzhajuvtpjsaKhz";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"kdkghme";
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
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)47195, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)50494, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)4755424246889221691L, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)60663, (uint16_t)8433, (uint16_t)8362, (uint16_t)33594, (uint16_t)13534, (uint16_t)64316, (uint16_t)60904, (uint16_t)18783, (uint16_t)15767, (uint16_t)22628, (uint16_t)18302, (uint16_t)37047, (uint16_t)12631, (uint16_t)34881, (uint16_t)56581, (uint16_t)17253, (uint16_t)33155, (uint16_t)56774, (uint16_t)51397, (uint16_t)1579, (uint16_t)6284, (uint16_t)1238, (uint16_t)50250, (uint16_t)25333, (uint16_t)46169, (uint16_t)44627, (uint16_t)63276, (uint16_t)38936, (uint16_t)59887, (uint16_t)60194, (uint16_t)63492, (uint16_t)21417, (uint16_t)9541, (uint16_t)48164, (uint16_t)38697, (uint16_t)11983, (uint16_t)10258, (uint16_t)47781, (uint16_t)44094, (uint16_t)9330, (uint16_t)19172, (uint16_t)38290, (uint16_t)38439, (uint16_t)45060, (uint16_t)12589, (uint16_t)10837, (uint16_t)10441, (uint16_t)63084, (uint16_t)42418, (uint16_t)18367, (uint16_t)9748, (uint16_t)21531, (uint16_t)60141, (uint16_t)13014, (uint16_t)57109, (uint16_t)13577, (uint16_t)57594, (uint16_t)31860, (uint16_t)60316, (uint16_t)24423, (uint16_t)7110, (uint16_t)353, (uint16_t)64380, (uint16_t)1904, (uint16_t)62473, (uint16_t)51702, (uint16_t)6103, (uint16_t)51897, (uint16_t)19873, (uint16_t)995, (uint16_t)40693, (uint16_t)33354};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


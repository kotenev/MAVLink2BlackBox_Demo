
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
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_POWEROFF);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2111593741L);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)28802);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)55501);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)24067);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)55396);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -158);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)53389);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)30353);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)85);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)23331);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)29809);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1306368784L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3149460314444622265L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_yaw_rate_GET(pack) == (float)5.458907E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p3_z_GET(pack) == (float) -1.9552417E38F);
    assert(p3_afz_GET(pack) == (float)1.482443E38F);
    assert(p3_afy_GET(pack) == (float) -2.9042804E38F);
    assert(p3_x_GET(pack) == (float) -1.736333E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)15522);
    assert(p3_vy_GET(pack) == (float) -1.2542174E38F);
    assert(p3_y_GET(pack) == (float) -1.9548856E38F);
    assert(p3_afx_GET(pack) == (float) -2.4853387E38F);
    assert(p3_yaw_GET(pack) == (float)3.929907E37F);
    assert(p3_vz_GET(pack) == (float) -3.4025482E38F);
    assert(p3_vx_GET(pack) == (float) -8.388056E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3370480666L);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)3731123607L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p4_time_usec_GET(pack) == (uint64_t)3275526724013108700L);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p5_passkey_LEN(ph) == 9);
    {
        char16_t * exemplary = u"ogeqziTpP";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)79);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 30);
    {
        char16_t * exemplary = u"ybnzwjTvcniswSeDxseqboDknkhgxr";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1668372511L);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"nkywkcwza";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)21175);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)71);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float) -1.4988212E38F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8);
    assert(p22_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"adxeblnf";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)15328);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)52718);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32);
    assert(p23_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"heecfzlsn";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p23_param_value_GET(pack) == (float) -1.3995217E38F);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)35696);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)58561);
    assert(p24_lon_GET(pack) == (int32_t) -89674982);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -925238291);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p24_alt_GET(pack) == (int32_t)546502488);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1621055977L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)3268060685L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)48752286L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)59763);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)10748);
    assert(p24_lat_GET(pack) == (int32_t) -972758839);
    assert(p24_time_usec_GET(pack) == (uint64_t)5686604473737045278L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1231474419L);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)227, (uint8_t)33, (uint8_t)238, (uint8_t)164, (uint8_t)78, (uint8_t)56, (uint8_t)197, (uint8_t)35, (uint8_t)242, (uint8_t)163, (uint8_t)127, (uint8_t)74, (uint8_t)144, (uint8_t)93, (uint8_t)84, (uint8_t)231, (uint8_t)38, (uint8_t)54, (uint8_t)73, (uint8_t)30} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)183, (uint8_t)123, (uint8_t)1, (uint8_t)195, (uint8_t)9, (uint8_t)71, (uint8_t)39, (uint8_t)127, (uint8_t)30, (uint8_t)99, (uint8_t)99, (uint8_t)140, (uint8_t)68, (uint8_t)38, (uint8_t)162, (uint8_t)240, (uint8_t)34, (uint8_t)236, (uint8_t)124, (uint8_t)33} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)21, (uint8_t)170, (uint8_t)6, (uint8_t)167, (uint8_t)29, (uint8_t)145, (uint8_t)90, (uint8_t)187, (uint8_t)42, (uint8_t)211, (uint8_t)140, (uint8_t)71, (uint8_t)98, (uint8_t)192, (uint8_t)57, (uint8_t)141, (uint8_t)210, (uint8_t)157, (uint8_t)119, (uint8_t)14} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)21);
    {
        uint8_t exemplary[] =  {(uint8_t)189, (uint8_t)98, (uint8_t)195, (uint8_t)212, (uint8_t)39, (uint8_t)111, (uint8_t)113, (uint8_t)255, (uint8_t)199, (uint8_t)222, (uint8_t)1, (uint8_t)134, (uint8_t)234, (uint8_t)84, (uint8_t)100, (uint8_t)60, (uint8_t)135, (uint8_t)94, (uint8_t)169, (uint8_t)113} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)189, (uint8_t)91, (uint8_t)211, (uint8_t)193, (uint8_t)7, (uint8_t)72, (uint8_t)251, (uint8_t)91, (uint8_t)57, (uint8_t)56, (uint8_t)13, (uint8_t)133, (uint8_t)27, (uint8_t)40, (uint8_t)70, (uint8_t)243, (uint8_t)123, (uint8_t)23, (uint8_t)104, (uint8_t)67} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)13741);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -25490);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)1435);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3276058377L);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)15673);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -18464);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)9952);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -22027);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)1719);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -25882);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)1084);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -31244);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)3167);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -1079);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)25140);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -7531);
    assert(p27_time_usec_GET(pack) == (uint64_t)5515856878550614031L);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -414);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -26508);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -2500);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)7307129725482014966L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -10744);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)12385);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)6763);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)32299);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)12360);
    assert(p29_press_diff_GET(pack) == (float) -1.809151E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1357191805L);
    assert(p29_press_abs_GET(pack) == (float) -3.0272145E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)995463298L);
    assert(p30_pitchspeed_GET(pack) == (float)5.470659E37F);
    assert(p30_roll_GET(pack) == (float)6.3257393E37F);
    assert(p30_pitch_GET(pack) == (float) -1.5683162E38F);
    assert(p30_rollspeed_GET(pack) == (float)3.084712E38F);
    assert(p30_yawspeed_GET(pack) == (float) -2.3486207E38F);
    assert(p30_yaw_GET(pack) == (float)3.0267474E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_pitchspeed_GET(pack) == (float)2.6504726E38F);
    assert(p31_q4_GET(pack) == (float) -1.64537E38F);
    assert(p31_q2_GET(pack) == (float)2.6901554E38F);
    assert(p31_q1_GET(pack) == (float)8.809495E37F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2568461791L);
    assert(p31_rollspeed_GET(pack) == (float)1.4016034E37F);
    assert(p31_yawspeed_GET(pack) == (float) -8.524665E37F);
    assert(p31_q3_GET(pack) == (float) -4.690723E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vx_GET(pack) == (float) -2.9863246E38F);
    assert(p32_vz_GET(pack) == (float) -2.3719651E38F);
    assert(p32_vy_GET(pack) == (float)2.5522618E38F);
    assert(p32_y_GET(pack) == (float) -2.4520746E38F);
    assert(p32_z_GET(pack) == (float)1.8166175E38F);
    assert(p32_x_GET(pack) == (float) -5.959607E36F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)3546715808L);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lon_GET(pack) == (int32_t) -1994171876);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -25073);
    assert(p33_relative_alt_GET(pack) == (int32_t)1176544429);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)7210);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)20977);
    assert(p33_lat_GET(pack) == (int32_t)1231645915);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1832901716L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)7649);
    assert(p33_alt_GET(pack) == (int32_t)2038312002);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -23928);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)28234);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3323129562L);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -29369);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -26358);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -891);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)3980);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -26007);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)23600);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)35);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)25759);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)29136);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1542424469L);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)1153);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)14844);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)2604);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)17703);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)6942);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)3664);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)2712);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)19113);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)10623);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)61514);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)20042);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)60981);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)21618);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)56665);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)47058);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)63031);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)9501);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)52910);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)21214);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)39805);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)45711);
    assert(p36_time_usec_GET(pack) == (uint32_t)4047164298L);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)30790);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -13634);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -8459);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)120);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)11294);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)10835);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_param2_GET(pack) == (float)1.9824595E38F);
    assert(p39_param1_GET(pack) == (float)1.8376893E38F);
    assert(p39_param4_GET(pack) == (float)1.829423E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p39_param3_GET(pack) == (float) -8.863477E37F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)55122);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_FOLLOW);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p39_z_GET(pack) == (float) -2.5373112E38F);
    assert(p39_x_GET(pack) == (float)1.7626203E38F);
    assert(p39_y_GET(pack) == (float)2.7549504E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)168);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)10895);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)217);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)24428);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)57364);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)26);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)54229);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)120);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)15528);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t)1481644176);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p48_longitude_GET(pack) == (int32_t)734026057);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1761394955046902823L);
    assert(p48_latitude_GET(pack) == (int32_t) -1998078549);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_latitude_GET(pack) == (int32_t) -699738576);
    assert(p49_longitude_GET(pack) == (int32_t) -383806453);
    assert(p49_altitude_GET(pack) == (int32_t)1870943713);
    assert(p49_time_usec_TRY(ph) == (uint64_t)4225157407281284095L);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value_min_GET(pack) == (float) -2.8901838E37F);
    assert(p50_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"wkxferonrbdpYiy";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float)1.97105E38F);
    assert(p50_param_value0_GET(pack) == (float)5.0163115E37F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -21170);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p50_scale_GET(pack) == (float)1.356137E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)25159);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2x_GET(pack) == (float)1.3893395E37F);
    assert(p54_p2z_GET(pack) == (float) -3.7206293E37F);
    assert(p54_p1z_GET(pack) == (float) -3.1185241E38F);
    assert(p54_p2y_GET(pack) == (float) -2.6476637E38F);
    assert(p54_p1x_GET(pack) == (float)9.897746E37F);
    assert(p54_p1y_GET(pack) == (float)1.6032298E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)167);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float) -1.5222091E38F);
    assert(p55_p1x_GET(pack) == (float) -1.325486E38F);
    assert(p55_p2y_GET(pack) == (float)2.41154E38F);
    assert(p55_p2z_GET(pack) == (float)2.452846E38F);
    assert(p55_p1z_GET(pack) == (float) -6.3022446E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p55_p2x_GET(pack) == (float)1.4757776E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float)3.0602625E38F);
    {
        float exemplary[] =  {-1.8253769E37F, -3.1119325E38F, -3.0696287E38F, -1.1853452E38F, 2.9096948E38F, 1.1220174E37F, -1.0562288E38F, 1.6955624E38F, 2.1708222E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)8159047715814304118L);
    {
        float exemplary[] =  {-3.6274654E37F, 8.1353085E37F, -1.3520544E36F, -8.4541474E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -3.3046546E38F);
    assert(p61_pitchspeed_GET(pack) == (float) -2.776401E38F);
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)20544);
    assert(p62_xtrack_error_GET(pack) == (float) -2.9961066E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -32404);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)21972);
    assert(p62_nav_pitch_GET(pack) == (float)1.548782E38F);
    assert(p62_nav_roll_GET(pack) == (float) -1.2290909E38F);
    assert(p62_aspd_error_GET(pack) == (float) -3.0807496E38F);
    assert(p62_alt_error_GET(pack) == (float)5.2900033E37F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-8.4655735E37F, 4.4738077E37F, -2.625713E38F, -3.2142186E38F, 2.9044415E38F, -1.7398507E38F, 2.4776541E38F, -3.1327039E37F, 7.138031E37F, 2.2474403E38F, 2.5989545E38F, -2.5140975E37F, -1.0259047E38F, -1.0317322E38F, -1.8669331E38F, -3.2212012E38F, -2.195169E38F, -2.456672E38F, -2.6350844E38F, 2.1797502E38F, 1.9091181E38F, -1.1911467E37F, 1.8414002E38F, 9.809157E37F, 7.981286E37F, 1.4740694E37F, -1.6152132E38F, -1.057354E38F, -1.8586786E38F, -2.9146626E38F, -3.27326E37F, -2.3123095E38F, 1.7064236E38F, -1.169135E37F, -5.8219414E37F, 5.113582E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_lon_GET(pack) == (int32_t)16359475);
    assert(p63_alt_GET(pack) == (int32_t)807412950);
    assert(p63_time_usec_GET(pack) == (uint64_t)2691314528630411946L);
    assert(p63_relative_alt_GET(pack) == (int32_t)1825919170);
    assert(p63_vz_GET(pack) == (float)3.13737E38F);
    assert(p63_lat_GET(pack) == (int32_t)1698118391);
    assert(p63_vx_GET(pack) == (float)1.1290167E38F);
    assert(p63_vy_GET(pack) == (float) -1.8683495E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_y_GET(pack) == (float)1.1836205E38F);
    assert(p64_z_GET(pack) == (float) -1.5203018E38F);
    assert(p64_vx_GET(pack) == (float)2.3368538E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)6392751380170744104L);
    {
        float exemplary[] =  {-1.3068659E37F, -2.9641644E38F, -2.1893549E38F, 5.1765374E36F, -3.2726357E38F, -1.7337162E38F, 2.2496861E38F, -2.6517273E38F, -1.8319474E38F, -3.6809475E37F, -2.7261927E38F, -3.3598448E38F, 2.4617463E37F, 2.9085052E38F, 3.3689314E38F, 9.239743E37F, 6.736276E37F, -2.9155292E38F, 2.46671E38F, -2.1858507E38F, -3.6421273E37F, -1.2404782E38F, 3.2295127E38F, -3.5699896E37F, 2.1693366E38F, 2.7511085E38F, -4.2377953E37F, -2.4217298E38F, -1.2586465E38F, 3.9122725E37F, -2.1571422E38F, 2.1301163E38F, -1.569228E38F, -7.935842E37F, -6.862774E37F, 1.8901399E38F, 9.471448E37F, -2.1452788E38F, -1.4200475E38F, -1.6170903E38F, 7.5479466E37F, -3.1555537E38F, -2.4012541E38F, -1.4899933E38F, 8.760589E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_x_GET(pack) == (float)1.1427123E38F);
    assert(p64_vz_GET(pack) == (float)1.8523227E38F);
    assert(p64_ay_GET(pack) == (float)2.824649E38F);
    assert(p64_az_GET(pack) == (float) -3.2923507E38F);
    assert(p64_vy_GET(pack) == (float)1.784613E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_ax_GET(pack) == (float) -1.37784E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)56914);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)10063);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)33594);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)5518);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)34057);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)21594);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)15218);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)41266);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)53605);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)3019);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)55673);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)13700);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)51366);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)56799);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)64536);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)4326);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)29561);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3679398204L);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)6206);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)32143);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)10439);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)1307);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)20076);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -15118);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -8488);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -8919);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)62687);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)36618);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)26881);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)55654);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)42956);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)41971);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)58597);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)34569);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_x_GET(pack) == (int32_t)1680223369);
    assert(p73_param1_GET(pack) == (float)1.0663975E38F);
    assert(p73_param3_GET(pack) == (float)1.0575775E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p73_param4_GET(pack) == (float) -1.5198803E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)26616);
    assert(p73_y_GET(pack) == (int32_t) -613839222);
    assert(p73_param2_GET(pack) == (float)6.378401E37F);
    assert(p73_z_GET(pack) == (float) -1.9534958E38F);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_airspeed_GET(pack) == (float) -3.3661403E38F);
    assert(p74_groundspeed_GET(pack) == (float)2.6539344E38F);
    assert(p74_climb_GET(pack) == (float)1.7025929E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -20234);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)31696);
    assert(p74_alt_GET(pack) == (float) -4.001697E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_y_GET(pack) == (int32_t) -746616668);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p75_z_GET(pack) == (float)9.447587E37F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p75_param4_GET(pack) == (float)4.2364262E37F);
    assert(p75_param1_GET(pack) == (float)1.9112052E38F);
    assert(p75_x_GET(pack) == (int32_t)257455975);
    assert(p75_param3_GET(pack) == (float) -2.3211405E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p75_param2_GET(pack) == (float) -9.25285E36F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float) -2.6910358E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p76_param1_GET(pack) == (float) -1.745496E38F);
    assert(p76_param5_GET(pack) == (float)2.5295745E37F);
    assert(p76_param4_GET(pack) == (float)3.3825897E38F);
    assert(p76_param3_GET(pack) == (float) -9.293185E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p76_param7_GET(pack) == (float)1.3793901E38F);
    assert(p76_param6_GET(pack) == (float)1.0089471E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)144);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)108);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)91);
    assert(p77_result_param2_TRY(ph) == (int32_t)127963839);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_FAILED);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_yaw_GET(pack) == (float)2.4722047E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)139448512L);
    assert(p81_roll_GET(pack) == (float)7.598536E37F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p81_thrust_GET(pack) == (float)7.2413506E37F);
    assert(p81_pitch_GET(pack) == (float) -2.1980891E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1754171402L);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p82_thrust_GET(pack) == (float) -2.2886504E37F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p82_body_yaw_rate_GET(pack) == (float) -2.0500084E38F);
    {
        float exemplary[] =  {1.7556098E37F, -2.2365922E38F, -2.9810694E38F, 3.0277007E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.4917303E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)1.6738734E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float)2.2634974E38F);
    {
        float exemplary[] =  {2.3367745E38F, 3.1722638E38F, 3.556615E37F, 5.7753157E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p83_body_roll_rate_GET(pack) == (float) -2.974885E38F);
    assert(p83_body_pitch_rate_GET(pack) == (float)4.1139293E37F);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.7647622E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3644901033L);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_rate_GET(pack) == (float) -1.1102756E38F);
    assert(p84_vz_GET(pack) == (float)4.8457486E37F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)41753);
    assert(p84_z_GET(pack) == (float)3.2596749E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p84_y_GET(pack) == (float)2.1332055E38F);
    assert(p84_vy_GET(pack) == (float) -1.7486741E38F);
    assert(p84_afz_GET(pack) == (float)3.2391716E38F);
    assert(p84_x_GET(pack) == (float)7.6423486E37F);
    assert(p84_afy_GET(pack) == (float) -1.6580732E38F);
    assert(p84_yaw_GET(pack) == (float)2.3241972E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3609167808L);
    assert(p84_afx_GET(pack) == (float)7.8366647E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p84_vx_GET(pack) == (float) -2.2088738E37F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lon_int_GET(pack) == (int32_t)910175209);
    assert(p86_yaw_GET(pack) == (float) -3.9198822E37F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p86_yaw_rate_GET(pack) == (float)1.1332725E38F);
    assert(p86_vz_GET(pack) == (float)2.6043793E37F);
    assert(p86_vy_GET(pack) == (float) -3.3223258E38F);
    assert(p86_alt_GET(pack) == (float)5.0939343E37F);
    assert(p86_afz_GET(pack) == (float)2.6407282E38F);
    assert(p86_afx_GET(pack) == (float) -2.7682771E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p86_afy_GET(pack) == (float)1.846277E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)994588484);
    assert(p86_vx_GET(pack) == (float) -1.2514833E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)54353);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2821341128L);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_GET(pack) == (float)2.6642274E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)765720318);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2491022205L);
    assert(p87_afz_GET(pack) == (float)1.3478934E38F);
    assert(p87_afx_GET(pack) == (float)2.8235623E38F);
    assert(p87_yaw_rate_GET(pack) == (float)5.903961E37F);
    assert(p87_afy_GET(pack) == (float)2.1076547E38F);
    assert(p87_vy_GET(pack) == (float) -1.6508696E38F);
    assert(p87_alt_GET(pack) == (float) -3.0346E38F);
    assert(p87_vz_GET(pack) == (float) -8.826181E37F);
    assert(p87_lat_int_GET(pack) == (int32_t) -2137100437);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)19978);
    assert(p87_vx_GET(pack) == (float)3.0807169E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float)2.3236226E38F);
    assert(p89_yaw_GET(pack) == (float)9.315444E37F);
    assert(p89_roll_GET(pack) == (float)3.9994349E37F);
    assert(p89_pitch_GET(pack) == (float)1.177011E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2406014081L);
    assert(p89_x_GET(pack) == (float)3.3168562E37F);
    assert(p89_z_GET(pack) == (float)2.6685195E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_roll_GET(pack) == (float) -8.176157E37F);
    assert(p90_pitch_GET(pack) == (float) -3.2921388E37F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -31165);
    assert(p90_rollspeed_GET(pack) == (float) -1.3157117E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)23037);
    assert(p90_pitchspeed_GET(pack) == (float)7.3471766E37F);
    assert(p90_lon_GET(pack) == (int32_t) -1207336887);
    assert(p90_alt_GET(pack) == (int32_t) -2147294216);
    assert(p90_yaw_GET(pack) == (float) -2.7251764E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)6418);
    assert(p90_yawspeed_GET(pack) == (float)3.5662355E36F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)8248);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -11408);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)6939);
    assert(p90_time_usec_GET(pack) == (uint64_t)6605171793525838041L);
    assert(p90_lat_GET(pack) == (int32_t)2051616150);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float) -1.5737381E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)7990636338854914622L);
    assert(p91_aux4_GET(pack) == (float) -3.0279104E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)4.635186E37F);
    assert(p91_aux3_GET(pack) == (float) -1.415079E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -2.0384596E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
    assert(p91_pitch_elevator_GET(pack) == (float)1.7670175E38F);
    assert(p91_aux2_GET(pack) == (float)1.2676066E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p91_throttle_GET(pack) == (float) -1.6719508E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)32719);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)51688);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)27392);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)53313);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)61107);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)10100);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)8520);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)372);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)60544);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)57210);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)6086);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p92_time_usec_GET(pack) == (uint64_t)6519904937898176219L);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)23534);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)5341938393403580195L);
    assert(p93_flags_GET(pack) == (uint64_t)3722358325685448759L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    {
        float exemplary[] =  {-5.8018517E37F, 6.7757907E37F, -2.0470067E38F, -8.552335E37F, 1.7536843E38F, -2.040806E37F, 4.4147666E37F, -1.9649811E38F, 3.2685669E38F, -3.4029573E37F, 3.3270595E38F, -3.3992513E38F, 1.0989142E38F, -2.2854641E38F, 2.7626157E38F, -1.0657792E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_time_usec_GET(pack) == (uint64_t)2001902961552274345L);
    assert(p100_ground_distance_GET(pack) == (float)2.3035599E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p100_flow_rate_y_TRY(ph) == (float) -7.718696E37F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)7856);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.6119547E38F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -3.1668652E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -26775);
    assert(p100_flow_comp_m_y_GET(pack) == (float)2.5560489E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_pitch_GET(pack) == (float)2.725869E38F);
    assert(p101_z_GET(pack) == (float) -7.8831545E37F);
    assert(p101_roll_GET(pack) == (float)2.0991178E38F);
    assert(p101_x_GET(pack) == (float)1.8082127E38F);
    assert(p101_y_GET(pack) == (float) -1.6386792E38F);
    assert(p101_yaw_GET(pack) == (float) -1.2284003E38F);
    assert(p101_usec_GET(pack) == (uint64_t)7267746115248919677L);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float) -2.346737E38F);
    assert(p102_pitch_GET(pack) == (float) -8.4355013E37F);
    assert(p102_x_GET(pack) == (float) -6.236664E37F);
    assert(p102_y_GET(pack) == (float) -2.7219866E38F);
    assert(p102_yaw_GET(pack) == (float) -1.8064238E38F);
    assert(p102_usec_GET(pack) == (uint64_t)9056670794392650258L);
    assert(p102_roll_GET(pack) == (float)8.1237206E37F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)644584205284985019L);
    assert(p103_z_GET(pack) == (float) -1.946152E38F);
    assert(p103_y_GET(pack) == (float) -2.2335217E38F);
    assert(p103_x_GET(pack) == (float)4.1197078E37F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_roll_GET(pack) == (float) -1.4355618E38F);
    assert(p104_pitch_GET(pack) == (float)2.5118592E38F);
    assert(p104_yaw_GET(pack) == (float) -1.6583686E38F);
    assert(p104_z_GET(pack) == (float)8.726704E36F);
    assert(p104_x_GET(pack) == (float) -2.703523E38F);
    assert(p104_usec_GET(pack) == (uint64_t)425501550247838189L);
    assert(p104_y_GET(pack) == (float) -1.8226862E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_yacc_GET(pack) == (float) -3.292288E38F);
    assert(p105_xmag_GET(pack) == (float) -1.6085343E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)13779);
    assert(p105_diff_pressure_GET(pack) == (float)9.925683E37F);
    assert(p105_zacc_GET(pack) == (float) -2.1256785E38F);
    assert(p105_zgyro_GET(pack) == (float)5.1701226E37F);
    assert(p105_xgyro_GET(pack) == (float)1.2799663E38F);
    assert(p105_xacc_GET(pack) == (float)7.7781073E37F);
    assert(p105_abs_pressure_GET(pack) == (float)3.1910222E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4809521626914798354L);
    assert(p105_ygyro_GET(pack) == (float) -2.640599E38F);
    assert(p105_temperature_GET(pack) == (float) -2.477878E38F);
    assert(p105_ymag_GET(pack) == (float)1.7367915E37F);
    assert(p105_pressure_alt_GET(pack) == (float)1.7266356E38F);
    assert(p105_zmag_GET(pack) == (float)6.1177305E37F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p106_integrated_zgyro_GET(pack) == (float) -9.065592E37F);
    assert(p106_integrated_y_GET(pack) == (float) -6.525318E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p106_distance_GET(pack) == (float) -1.635358E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3849740708L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)4001788444L);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.1623402E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)5695183757555482773L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -16220);
    assert(p106_integrated_x_GET(pack) == (float) -1.9101166E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -1.5864946E38F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zacc_GET(pack) == (float) -2.3373698E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)258807589L);
    assert(p107_xacc_GET(pack) == (float)2.212779E38F);
    assert(p107_yacc_GET(pack) == (float) -9.520627E37F);
    assert(p107_pressure_alt_GET(pack) == (float)2.5689538E38F);
    assert(p107_zgyro_GET(pack) == (float)2.4483112E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)2769187365011362530L);
    assert(p107_abs_pressure_GET(pack) == (float)1.8695968E37F);
    assert(p107_xmag_GET(pack) == (float)2.6043011E38F);
    assert(p107_ygyro_GET(pack) == (float) -1.1353716E38F);
    assert(p107_ymag_GET(pack) == (float) -1.9952216E38F);
    assert(p107_diff_pressure_GET(pack) == (float)1.3088171E38F);
    assert(p107_xgyro_GET(pack) == (float) -2.0448261E37F);
    assert(p107_temperature_GET(pack) == (float)1.7574907E38F);
    assert(p107_zmag_GET(pack) == (float)1.0690842E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_q4_GET(pack) == (float)2.645436E38F);
    assert(p108_xgyro_GET(pack) == (float)1.636591E38F);
    assert(p108_alt_GET(pack) == (float)2.7758258E38F);
    assert(p108_yaw_GET(pack) == (float) -3.1187428E38F);
    assert(p108_zgyro_GET(pack) == (float) -3.3212016E38F);
    assert(p108_lon_GET(pack) == (float)2.3065787E37F);
    assert(p108_q1_GET(pack) == (float)1.8273782E38F);
    assert(p108_xacc_GET(pack) == (float)2.582188E38F);
    assert(p108_pitch_GET(pack) == (float)2.5539957E38F);
    assert(p108_yacc_GET(pack) == (float) -1.0804414E38F);
    assert(p108_q2_GET(pack) == (float) -2.7935064E38F);
    assert(p108_roll_GET(pack) == (float) -2.7444471E38F);
    assert(p108_vn_GET(pack) == (float)8.1131185E37F);
    assert(p108_std_dev_horz_GET(pack) == (float) -1.1826189E38F);
    assert(p108_ve_GET(pack) == (float) -2.402552E38F);
    assert(p108_ygyro_GET(pack) == (float) -4.4152793E37F);
    assert(p108_q3_GET(pack) == (float)9.447934E37F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.8213584E38F);
    assert(p108_zacc_GET(pack) == (float) -3.2358495E38F);
    assert(p108_vd_GET(pack) == (float)1.9676312E38F);
    assert(p108_lat_GET(pack) == (float) -2.3966912E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)12141);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)2337);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)46, (uint8_t)49, (uint8_t)56, (uint8_t)30, (uint8_t)4, (uint8_t)155, (uint8_t)98, (uint8_t)6, (uint8_t)195, (uint8_t)10, (uint8_t)127, (uint8_t)214, (uint8_t)195, (uint8_t)29, (uint8_t)118, (uint8_t)31, (uint8_t)154, (uint8_t)170, (uint8_t)121, (uint8_t)193, (uint8_t)239, (uint8_t)95, (uint8_t)178, (uint8_t)83, (uint8_t)4, (uint8_t)135, (uint8_t)50, (uint8_t)147, (uint8_t)173, (uint8_t)6, (uint8_t)33, (uint8_t)209, (uint8_t)74, (uint8_t)191, (uint8_t)32, (uint8_t)206, (uint8_t)245, (uint8_t)119, (uint8_t)247, (uint8_t)3, (uint8_t)148, (uint8_t)181, (uint8_t)60, (uint8_t)229, (uint8_t)152, (uint8_t)193, (uint8_t)194, (uint8_t)246, (uint8_t)171, (uint8_t)180, (uint8_t)158, (uint8_t)203, (uint8_t)182, (uint8_t)103, (uint8_t)91, (uint8_t)60, (uint8_t)222, (uint8_t)176, (uint8_t)198, (uint8_t)136, (uint8_t)242, (uint8_t)88, (uint8_t)243, (uint8_t)142, (uint8_t)159, (uint8_t)111, (uint8_t)171, (uint8_t)146, (uint8_t)25, (uint8_t)104, (uint8_t)194, (uint8_t)97, (uint8_t)143, (uint8_t)31, (uint8_t)255, (uint8_t)118, (uint8_t)159, (uint8_t)36, (uint8_t)215, (uint8_t)74, (uint8_t)208, (uint8_t)85, (uint8_t)145, (uint8_t)116, (uint8_t)119, (uint8_t)164, (uint8_t)201, (uint8_t)118, (uint8_t)132, (uint8_t)172, (uint8_t)163, (uint8_t)200, (uint8_t)165, (uint8_t)132, (uint8_t)57, (uint8_t)159, (uint8_t)32, (uint8_t)89, (uint8_t)177, (uint8_t)241, (uint8_t)31, (uint8_t)74, (uint8_t)229, (uint8_t)109, (uint8_t)95, (uint8_t)172, (uint8_t)118, (uint8_t)44, (uint8_t)23, (uint8_t)230, (uint8_t)178, (uint8_t)184, (uint8_t)236, (uint8_t)64, (uint8_t)211, (uint8_t)199, (uint8_t)178, (uint8_t)128, (uint8_t)119, (uint8_t)168, (uint8_t)200, (uint8_t)148, (uint8_t)230, (uint8_t)60, (uint8_t)51, (uint8_t)72, (uint8_t)103, (uint8_t)159, (uint8_t)96, (uint8_t)80, (uint8_t)113, (uint8_t)71, (uint8_t)88, (uint8_t)194, (uint8_t)123, (uint8_t)53, (uint8_t)116, (uint8_t)199, (uint8_t)197, (uint8_t)61, (uint8_t)165, (uint8_t)87, (uint8_t)101, (uint8_t)136, (uint8_t)164, (uint8_t)251, (uint8_t)87, (uint8_t)100, (uint8_t)24, (uint8_t)179, (uint8_t)118, (uint8_t)32, (uint8_t)77, (uint8_t)115, (uint8_t)79, (uint8_t)95, (uint8_t)141, (uint8_t)147, (uint8_t)83, (uint8_t)215, (uint8_t)104, (uint8_t)238, (uint8_t)151, (uint8_t)142, (uint8_t)191, (uint8_t)161, (uint8_t)93, (uint8_t)124, (uint8_t)19, (uint8_t)84, (uint8_t)244, (uint8_t)116, (uint8_t)15, (uint8_t)162, (uint8_t)199, (uint8_t)241, (uint8_t)139, (uint8_t)52, (uint8_t)145, (uint8_t)237, (uint8_t)138, (uint8_t)192, (uint8_t)178, (uint8_t)193, (uint8_t)8, (uint8_t)120, (uint8_t)86, (uint8_t)8, (uint8_t)122, (uint8_t)52, (uint8_t)110, (uint8_t)10, (uint8_t)50, (uint8_t)151, (uint8_t)192, (uint8_t)3, (uint8_t)130, (uint8_t)237, (uint8_t)244, (uint8_t)97, (uint8_t)166, (uint8_t)211, (uint8_t)122, (uint8_t)196, (uint8_t)178, (uint8_t)246, (uint8_t)75, (uint8_t)109, (uint8_t)150, (uint8_t)243, (uint8_t)241, (uint8_t)10, (uint8_t)192, (uint8_t)51, (uint8_t)118, (uint8_t)171, (uint8_t)167, (uint8_t)208, (uint8_t)200, (uint8_t)47, (uint8_t)116, (uint8_t)30, (uint8_t)59, (uint8_t)240, (uint8_t)120, (uint8_t)212, (uint8_t)197, (uint8_t)249, (uint8_t)14, (uint8_t)247, (uint8_t)218, (uint8_t)144, (uint8_t)50, (uint8_t)196, (uint8_t)115, (uint8_t)55, (uint8_t)89, (uint8_t)211, (uint8_t)116, (uint8_t)201, (uint8_t)127, (uint8_t)153, (uint8_t)228, (uint8_t)114, (uint8_t)123, (uint8_t)112, (uint8_t)100, (uint8_t)12, (uint8_t)75, (uint8_t)22, (uint8_t)255} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)984070289590459344L);
    assert(p111_ts1_GET(pack) == (int64_t)1952061322550441358L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)3452781018247207385L);
    assert(p112_seq_GET(pack) == (uint32_t)1753248257L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -1495);
    assert(p113_lon_GET(pack) == (int32_t)1127925067);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)37615);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)5659);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)49005);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -5803);
    assert(p113_alt_GET(pack) == (int32_t) -1921574456);
    assert(p113_lat_GET(pack) == (int32_t)757436846);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)23776);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)49520);
    assert(p113_time_usec_GET(pack) == (uint64_t)794400969753929107L);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p114_distance_GET(pack) == (float) -2.3556648E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.0557119E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)6539942226715359330L);
    assert(p114_integrated_x_GET(pack) == (float)3.0022706E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)2588);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1524641126L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.1636468E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3515813319L);
    assert(p114_integrated_y_GET(pack) == (float)2.3279194E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float) -3.2740553E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yawspeed_GET(pack) == (float)3.356234E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)38268);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)24512);
    assert(p115_pitchspeed_GET(pack) == (float) -3.1884869E38F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)17337);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)13747);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -10068);
    {
        float exemplary[] =  {1.6246718E38F, -9.145671E37F, 3.439822E37F, 3.2895733E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_rollspeed_GET(pack) == (float) -7.0586705E37F);
    assert(p115_lat_GET(pack) == (int32_t)781813158);
    assert(p115_time_usec_GET(pack) == (uint64_t)1013835421286389586L);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)15732);
    assert(p115_alt_GET(pack) == (int32_t)30453367);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -16983);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)58304);
    assert(p115_lon_GET(pack) == (int32_t)1687209570);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)20322);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)8622);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -13732);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)17169);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -18258);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)27634);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3288106365L);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -14649);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -14830);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)12979);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)54121);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)36426);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)38002);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)4142);
    assert(p118_time_utc_GET(pack) == (uint32_t)958348128L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)13527);
    assert(p118_size_GET(pack) == (uint32_t)4144396705L);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)31087);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p119_count_GET(pack) == (uint32_t)3339152813L);
    assert(p119_ofs_GET(pack) == (uint32_t)1900353624L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)116);
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)123, (uint8_t)146, (uint8_t)188, (uint8_t)18, (uint8_t)127, (uint8_t)52, (uint8_t)177, (uint8_t)127, (uint8_t)240, (uint8_t)210, (uint8_t)40, (uint8_t)223, (uint8_t)62, (uint8_t)63, (uint8_t)248, (uint8_t)113, (uint8_t)22, (uint8_t)183, (uint8_t)211, (uint8_t)76, (uint8_t)222, (uint8_t)30, (uint8_t)91, (uint8_t)103, (uint8_t)186, (uint8_t)103, (uint8_t)67, (uint8_t)225, (uint8_t)202, (uint8_t)207, (uint8_t)161, (uint8_t)40, (uint8_t)158, (uint8_t)251, (uint8_t)35, (uint8_t)27, (uint8_t)159, (uint8_t)117, (uint8_t)178, (uint8_t)175, (uint8_t)8, (uint8_t)209, (uint8_t)104, (uint8_t)23, (uint8_t)176, (uint8_t)142, (uint8_t)240, (uint8_t)147, (uint8_t)254, (uint8_t)73, (uint8_t)17, (uint8_t)140, (uint8_t)55, (uint8_t)92, (uint8_t)242, (uint8_t)47, (uint8_t)53, (uint8_t)56, (uint8_t)214, (uint8_t)124, (uint8_t)213, (uint8_t)242, (uint8_t)126, (uint8_t)226, (uint8_t)222, (uint8_t)240, (uint8_t)236, (uint8_t)240, (uint8_t)198, (uint8_t)36, (uint8_t)58, (uint8_t)180, (uint8_t)241, (uint8_t)117, (uint8_t)46, (uint8_t)12, (uint8_t)102, (uint8_t)246, (uint8_t)123, (uint8_t)158, (uint8_t)150, (uint8_t)26, (uint8_t)32, (uint8_t)23, (uint8_t)175, (uint8_t)67, (uint8_t)189, (uint8_t)90, (uint8_t)247} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)3276131960L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)47006);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)102);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)19);
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)26, (uint8_t)138, (uint8_t)22, (uint8_t)137, (uint8_t)26, (uint8_t)42, (uint8_t)119, (uint8_t)97, (uint8_t)86, (uint8_t)102, (uint8_t)86, (uint8_t)73, (uint8_t)9, (uint8_t)6, (uint8_t)189, (uint8_t)52, (uint8_t)178, (uint8_t)70, (uint8_t)237, (uint8_t)38, (uint8_t)198, (uint8_t)110, (uint8_t)78, (uint8_t)224, (uint8_t)67, (uint8_t)253, (uint8_t)72, (uint8_t)216, (uint8_t)166, (uint8_t)147, (uint8_t)14, (uint8_t)187, (uint8_t)132, (uint8_t)177, (uint8_t)128, (uint8_t)211, (uint8_t)35, (uint8_t)184, (uint8_t)9, (uint8_t)91, (uint8_t)30, (uint8_t)97, (uint8_t)254, (uint8_t)190, (uint8_t)99, (uint8_t)198, (uint8_t)171, (uint8_t)17, (uint8_t)63, (uint8_t)206, (uint8_t)239, (uint8_t)112, (uint8_t)156, (uint8_t)116, (uint8_t)223, (uint8_t)2, (uint8_t)11, (uint8_t)190, (uint8_t)6, (uint8_t)104, (uint8_t)71, (uint8_t)85, (uint8_t)176, (uint8_t)191, (uint8_t)164, (uint8_t)206, (uint8_t)175, (uint8_t)33, (uint8_t)41, (uint8_t)202, (uint8_t)219, (uint8_t)122, (uint8_t)213, (uint8_t)72, (uint8_t)130, (uint8_t)81, (uint8_t)223, (uint8_t)212, (uint8_t)10, (uint8_t)55, (uint8_t)79, (uint8_t)17, (uint8_t)70, (uint8_t)143, (uint8_t)202, (uint8_t)128, (uint8_t)176, (uint8_t)109, (uint8_t)98, (uint8_t)52, (uint8_t)109, (uint8_t)33, (uint8_t)208, (uint8_t)204, (uint8_t)33, (uint8_t)41, (uint8_t)95, (uint8_t)154, (uint8_t)207, (uint8_t)32, (uint8_t)115, (uint8_t)218, (uint8_t)129, (uint8_t)235, (uint8_t)235, (uint8_t)66, (uint8_t)181, (uint8_t)117, (uint8_t)127} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)142);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)40086);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p124_dgps_age_GET(pack) == (uint32_t)3142296150L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)14433);
    assert(p124_lat_GET(pack) == (int32_t) -701916125);
    assert(p124_alt_GET(pack) == (int32_t) -973263044);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p124_time_usec_GET(pack) == (uint64_t)8271421492108452511L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)63672);
    assert(p124_lon_GET(pack) == (int32_t)719808003);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)5704);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)55795);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)15023);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)205, (uint8_t)140, (uint8_t)73, (uint8_t)66, (uint8_t)157, (uint8_t)63, (uint8_t)148, (uint8_t)252, (uint8_t)161, (uint8_t)62, (uint8_t)43, (uint8_t)25, (uint8_t)225, (uint8_t)174, (uint8_t)134, (uint8_t)157, (uint8_t)171, (uint8_t)240, (uint8_t)107, (uint8_t)242, (uint8_t)67, (uint8_t)100, (uint8_t)88, (uint8_t)191, (uint8_t)191, (uint8_t)150, (uint8_t)81, (uint8_t)184, (uint8_t)152, (uint8_t)94, (uint8_t)119, (uint8_t)120, (uint8_t)169, (uint8_t)252, (uint8_t)108, (uint8_t)138, (uint8_t)230, (uint8_t)55, (uint8_t)141, (uint8_t)252, (uint8_t)15, (uint8_t)192, (uint8_t)15, (uint8_t)143, (uint8_t)189, (uint8_t)207, (uint8_t)170, (uint8_t)151, (uint8_t)156, (uint8_t)99, (uint8_t)47, (uint8_t)11, (uint8_t)81, (uint8_t)193, (uint8_t)229, (uint8_t)168, (uint8_t)144, (uint8_t)137, (uint8_t)137, (uint8_t)107, (uint8_t)161, (uint8_t)58, (uint8_t)230, (uint8_t)217, (uint8_t)221, (uint8_t)179, (uint8_t)118, (uint8_t)112, (uint8_t)131, (uint8_t)171} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)31567);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI);
    assert(p126_baudrate_GET(pack) == (uint32_t)2060999350L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)18);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)39132);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)346431268L);
    assert(p127_tow_GET(pack) == (uint32_t)3738954107L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -632227583);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -788715736);
    assert(p127_accuracy_GET(pack) == (uint32_t)2646524226L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -2014927820);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)1391241703);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4040283679L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -676710899);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)3532);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1707436860);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -538954998);
    assert(p128_accuracy_GET(pack) == (uint32_t)2404709418L);
    assert(p128_tow_GET(pack) == (uint32_t)3249469675L);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)337913507);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -28043);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)12728);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)4194656063L);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)3978);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)18031);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -7657);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)1801);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)136);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)21092);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)1860);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)804);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)46626);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p130_size_GET(pack) == (uint32_t)1035263706L);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)17385);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)26813);
    {
        uint8_t exemplary[] =  {(uint8_t)246, (uint8_t)237, (uint8_t)75, (uint8_t)106, (uint8_t)105, (uint8_t)42, (uint8_t)73, (uint8_t)177, (uint8_t)25, (uint8_t)24, (uint8_t)113, (uint8_t)114, (uint8_t)30, (uint8_t)154, (uint8_t)176, (uint8_t)75, (uint8_t)67, (uint8_t)98, (uint8_t)190, (uint8_t)115, (uint8_t)130, (uint8_t)57, (uint8_t)143, (uint8_t)140, (uint8_t)156, (uint8_t)69, (uint8_t)229, (uint8_t)142, (uint8_t)223, (uint8_t)16, (uint8_t)252, (uint8_t)96, (uint8_t)140, (uint8_t)107, (uint8_t)128, (uint8_t)220, (uint8_t)131, (uint8_t)25, (uint8_t)66, (uint8_t)232, (uint8_t)1, (uint8_t)190, (uint8_t)152, (uint8_t)154, (uint8_t)157, (uint8_t)23, (uint8_t)74, (uint8_t)106, (uint8_t)29, (uint8_t)10, (uint8_t)255, (uint8_t)180, (uint8_t)147, (uint8_t)101, (uint8_t)16, (uint8_t)2, (uint8_t)73, (uint8_t)64, (uint8_t)88, (uint8_t)159, (uint8_t)175, (uint8_t)222, (uint8_t)75, (uint8_t)18, (uint8_t)23, (uint8_t)42, (uint8_t)206, (uint8_t)157, (uint8_t)178, (uint8_t)146, (uint8_t)96, (uint8_t)19, (uint8_t)160, (uint8_t)116, (uint8_t)33, (uint8_t)20, (uint8_t)110, (uint8_t)49, (uint8_t)121, (uint8_t)61, (uint8_t)250, (uint8_t)239, (uint8_t)30, (uint8_t)142, (uint8_t)138, (uint8_t)62, (uint8_t)76, (uint8_t)228, (uint8_t)181, (uint8_t)94, (uint8_t)2, (uint8_t)190, (uint8_t)126, (uint8_t)140, (uint8_t)55, (uint8_t)245, (uint8_t)98, (uint8_t)68, (uint8_t)99, (uint8_t)168, (uint8_t)250, (uint8_t)202, (uint8_t)96, (uint8_t)129, (uint8_t)192, (uint8_t)171, (uint8_t)11, (uint8_t)119, (uint8_t)203, (uint8_t)124, (uint8_t)84, (uint8_t)20, (uint8_t)37, (uint8_t)242, (uint8_t)222, (uint8_t)48, (uint8_t)153, (uint8_t)6, (uint8_t)154, (uint8_t)83, (uint8_t)65, (uint8_t)69, (uint8_t)26, (uint8_t)222, (uint8_t)37, (uint8_t)97, (uint8_t)193, (uint8_t)145, (uint8_t)14, (uint8_t)195, (uint8_t)226, (uint8_t)64, (uint8_t)69, (uint8_t)160, (uint8_t)27, (uint8_t)182, (uint8_t)107, (uint8_t)154, (uint8_t)218, (uint8_t)110, (uint8_t)60, (uint8_t)60, (uint8_t)136, (uint8_t)221, (uint8_t)177, (uint8_t)148, (uint8_t)188, (uint8_t)22, (uint8_t)97, (uint8_t)38, (uint8_t)208, (uint8_t)41, (uint8_t)144, (uint8_t)210, (uint8_t)176, (uint8_t)48, (uint8_t)31, (uint8_t)223, (uint8_t)80, (uint8_t)38, (uint8_t)66, (uint8_t)218, (uint8_t)111, (uint8_t)119, (uint8_t)241, (uint8_t)224, (uint8_t)178, (uint8_t)193, (uint8_t)63, (uint8_t)172, (uint8_t)76, (uint8_t)148, (uint8_t)49, (uint8_t)115, (uint8_t)86, (uint8_t)197, (uint8_t)13, (uint8_t)55, (uint8_t)91, (uint8_t)44, (uint8_t)225, (uint8_t)131, (uint8_t)82, (uint8_t)204, (uint8_t)156, (uint8_t)212, (uint8_t)95, (uint8_t)197, (uint8_t)182, (uint8_t)171, (uint8_t)6, (uint8_t)125, (uint8_t)165, (uint8_t)44, (uint8_t)207, (uint8_t)134, (uint8_t)87, (uint8_t)60, (uint8_t)132, (uint8_t)200, (uint8_t)195, (uint8_t)166, (uint8_t)59, (uint8_t)1, (uint8_t)35, (uint8_t)91, (uint8_t)167, (uint8_t)222, (uint8_t)68, (uint8_t)6, (uint8_t)55, (uint8_t)209, (uint8_t)26, (uint8_t)16, (uint8_t)167, (uint8_t)231, (uint8_t)164, (uint8_t)252, (uint8_t)197, (uint8_t)21, (uint8_t)194, (uint8_t)57, (uint8_t)54, (uint8_t)250, (uint8_t)68, (uint8_t)211, (uint8_t)103, (uint8_t)166, (uint8_t)193, (uint8_t)63, (uint8_t)145, (uint8_t)194, (uint8_t)41, (uint8_t)189, (uint8_t)8, (uint8_t)119, (uint8_t)71, (uint8_t)62, (uint8_t)140, (uint8_t)194, (uint8_t)96, (uint8_t)119, (uint8_t)20, (uint8_t)89, (uint8_t)33, (uint8_t)74, (uint8_t)44, (uint8_t)250, (uint8_t)178, (uint8_t)104, (uint8_t)138, (uint8_t)8, (uint8_t)33} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_225);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)63277);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)63288);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)57434);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)4127518180L);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)16561);
    assert(p133_mask_GET(pack) == (uint64_t)2539027389232551789L);
    assert(p133_lon_GET(pack) == (int32_t)1353225415);
    assert(p133_lat_GET(pack) == (int32_t) -1240274137);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p134_lat_GET(pack) == (int32_t) -1573945037);
    {
        int16_t exemplary[] =  {(int16_t) -7152, (int16_t)31459, (int16_t) -30229, (int16_t)24298, (int16_t) -19626, (int16_t) -20170, (int16_t) -30970, (int16_t) -19882, (int16_t) -17857, (int16_t)12823, (int16_t) -15123, (int16_t)17850, (int16_t) -22127, (int16_t)31021, (int16_t)824, (int16_t) -24386} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t) -1151468842);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)29290);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -1976495672);
    assert(p135_lat_GET(pack) == (int32_t) -1268481740);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lon_GET(pack) == (int32_t) -1844185232);
    assert(p136_terrain_height_GET(pack) == (float) -2.5712383E37F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)54033);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)11965);
    assert(p136_current_height_GET(pack) == (float)2.120251E38F);
    assert(p136_lat_GET(pack) == (int32_t) -1561717303);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)62352);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)2.6145425E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3459624371L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)8247);
    assert(p137_press_diff_GET(pack) == (float)1.3462372E38F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_y_GET(pack) == (float)3.2069408E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)4492177325691604603L);
    assert(p138_z_GET(pack) == (float)1.7400366E38F);
    {
        float exemplary[] =  {-2.614686E38F, -2.6357888E38F, 2.6234964E38F, -2.8229224E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float) -1.1645493E38F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p139_time_usec_GET(pack) == (uint64_t)7348825432026364559L);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)62);
    {
        float exemplary[] =  {-1.8311521E38F, 3.3853751E38F, -5.014698E37F, -3.1049724E38F, 2.12802E38F, -6.185365E37F, 2.7076203E38F, 2.4331797E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.5450512E38F, 4.2444157E37F, 3.1426886E38F, -2.2601673E38F, -9.625697E37F, 1.0860842E38F, 2.1582467E38F, -6.143432E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p140_time_usec_GET(pack) == (uint64_t)2463457250857692133L);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_relative_GET(pack) == (float)1.2489406E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.8233982E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)4.7013947E36F);
    assert(p141_altitude_amsl_GET(pack) == (float) -2.76371E38F);
    assert(p141_altitude_local_GET(pack) == (float) -3.1094492E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -1.1744562E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)1007474590083344186L);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)12, (uint8_t)58, (uint8_t)157, (uint8_t)75, (uint8_t)133, (uint8_t)244, (uint8_t)101, (uint8_t)114, (uint8_t)90, (uint8_t)178, (uint8_t)153, (uint8_t)177, (uint8_t)188, (uint8_t)197, (uint8_t)243, (uint8_t)81, (uint8_t)131, (uint8_t)238, (uint8_t)168, (uint8_t)6, (uint8_t)146, (uint8_t)20, (uint8_t)72, (uint8_t)215, (uint8_t)79, (uint8_t)9, (uint8_t)77, (uint8_t)201, (uint8_t)103, (uint8_t)198, (uint8_t)158, (uint8_t)23, (uint8_t)195, (uint8_t)242, (uint8_t)34, (uint8_t)17, (uint8_t)163, (uint8_t)21, (uint8_t)20, (uint8_t)204, (uint8_t)144, (uint8_t)143, (uint8_t)75, (uint8_t)217, (uint8_t)10, (uint8_t)16, (uint8_t)71, (uint8_t)27, (uint8_t)111, (uint8_t)234, (uint8_t)253, (uint8_t)137, (uint8_t)250, (uint8_t)225, (uint8_t)148, (uint8_t)142, (uint8_t)79, (uint8_t)149, (uint8_t)232, (uint8_t)20, (uint8_t)171, (uint8_t)102, (uint8_t)129, (uint8_t)71, (uint8_t)23, (uint8_t)46, (uint8_t)90, (uint8_t)116, (uint8_t)11, (uint8_t)91, (uint8_t)235, (uint8_t)144, (uint8_t)240, (uint8_t)93, (uint8_t)103, (uint8_t)40, (uint8_t)90, (uint8_t)120, (uint8_t)3, (uint8_t)56, (uint8_t)242, (uint8_t)25, (uint8_t)48, (uint8_t)45, (uint8_t)151, (uint8_t)50, (uint8_t)130, (uint8_t)70, (uint8_t)29, (uint8_t)135, (uint8_t)51, (uint8_t)121, (uint8_t)30, (uint8_t)129, (uint8_t)218, (uint8_t)148, (uint8_t)29, (uint8_t)248, (uint8_t)89, (uint8_t)165, (uint8_t)229, (uint8_t)107, (uint8_t)134, (uint8_t)184, (uint8_t)142, (uint8_t)36, (uint8_t)209, (uint8_t)178, (uint8_t)170, (uint8_t)240, (uint8_t)74, (uint8_t)54, (uint8_t)224, (uint8_t)109, (uint8_t)169, (uint8_t)94, (uint8_t)4, (uint8_t)232, (uint8_t)238, (uint8_t)207} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)4);
    {
        uint8_t exemplary[] =  {(uint8_t)171, (uint8_t)124, (uint8_t)56, (uint8_t)65, (uint8_t)162, (uint8_t)92, (uint8_t)0, (uint8_t)225, (uint8_t)185, (uint8_t)215, (uint8_t)58, (uint8_t)93, (uint8_t)89, (uint8_t)101, (uint8_t)226, (uint8_t)33, (uint8_t)8, (uint8_t)59, (uint8_t)221, (uint8_t)244, (uint8_t)153, (uint8_t)71, (uint8_t)30, (uint8_t)182, (uint8_t)54, (uint8_t)70, (uint8_t)124, (uint8_t)45, (uint8_t)50, (uint8_t)13, (uint8_t)129, (uint8_t)180, (uint8_t)25, (uint8_t)177, (uint8_t)112, (uint8_t)89, (uint8_t)218, (uint8_t)116, (uint8_t)149, (uint8_t)54, (uint8_t)248, (uint8_t)130, (uint8_t)220, (uint8_t)182, (uint8_t)103, (uint8_t)89, (uint8_t)63, (uint8_t)178, (uint8_t)219, (uint8_t)88, (uint8_t)89, (uint8_t)157, (uint8_t)74, (uint8_t)60, (uint8_t)121, (uint8_t)124, (uint8_t)27, (uint8_t)212, (uint8_t)40, (uint8_t)255, (uint8_t)118, (uint8_t)246, (uint8_t)245, (uint8_t)124, (uint8_t)83, (uint8_t)146, (uint8_t)207, (uint8_t)177, (uint8_t)155, (uint8_t)84, (uint8_t)94, (uint8_t)176, (uint8_t)54, (uint8_t)146, (uint8_t)13, (uint8_t)11, (uint8_t)223, (uint8_t)7, (uint8_t)148, (uint8_t)177, (uint8_t)130, (uint8_t)207, (uint8_t)209, (uint8_t)102, (uint8_t)47, (uint8_t)107, (uint8_t)227, (uint8_t)205, (uint8_t)101, (uint8_t)1, (uint8_t)94, (uint8_t)13, (uint8_t)236, (uint8_t)98, (uint8_t)84, (uint8_t)73, (uint8_t)161, (uint8_t)86, (uint8_t)186, (uint8_t)240, (uint8_t)157, (uint8_t)18, (uint8_t)86, (uint8_t)21, (uint8_t)84, (uint8_t)47, (uint8_t)163, (uint8_t)1, (uint8_t)64, (uint8_t)82, (uint8_t)78, (uint8_t)5, (uint8_t)161, (uint8_t)44, (uint8_t)137, (uint8_t)249, (uint8_t)22, (uint8_t)236, (uint8_t)58, (uint8_t)72} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)2.5559578E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2103703163L);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)18118);
    assert(p143_press_diff_GET(pack) == (float)1.9553263E38F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_custom_state_GET(pack) == (uint64_t)7798861023712896359L);
    assert(p144_lon_GET(pack) == (int32_t)44688403);
    {
        float exemplary[] =  {2.5700304E38F, 1.6943772E38F, 4.977151E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.645347E38F, 2.884403E38F, -6.575374E36F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -5.4936884E37F);
    {
        float exemplary[] =  {2.9928008E38F, -3.4233067E37F, 9.03231E37F, -2.3800045E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.604132E37F, 1.2674056E38F, 3.2318261E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-8.1476934E37F, -2.2586126E38F, 4.7315236E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -336251820);
    assert(p144_timestamp_GET(pack) == (uint64_t)5401330020222628162L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)22);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_y_acc_GET(pack) == (float) -1.8054283E38F);
    assert(p146_roll_rate_GET(pack) == (float) -2.7742888E37F);
    {
        float exemplary[] =  {-3.0934881E38F, -2.1641177E38F, 1.139701E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float)2.4275101E38F);
    assert(p146_x_acc_GET(pack) == (float) -2.1923368E38F);
    assert(p146_z_pos_GET(pack) == (float) -1.4343964E38F);
    assert(p146_airspeed_GET(pack) == (float) -3.0007509E38F);
    assert(p146_x_vel_GET(pack) == (float) -2.7880182E38F);
    assert(p146_x_pos_GET(pack) == (float)2.1059666E38F);
    {
        float exemplary[] =  {-3.1731528E38F, 2.7952562E38F, -1.9558847E38F, -1.5322047E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float)1.6525003E38F);
    assert(p146_y_pos_GET(pack) == (float) -2.0396626E38F);
    assert(p146_z_vel_GET(pack) == (float) -9.710111E37F);
    assert(p146_pitch_rate_GET(pack) == (float) -4.268993E37F);
    assert(p146_y_vel_GET(pack) == (float)2.5466052E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)4299980209147537323L);
    {
        float exemplary[] =  {-8.701573E37F, 3.0642648E38F, 2.57798E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_energy_consumed_GET(pack) == (int32_t) -1997240039);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)13037);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -28115);
    {
        uint16_t exemplary[] =  {(uint16_t)33210, (uint16_t)29978, (uint16_t)44123, (uint16_t)52096, (uint16_t)11444, (uint16_t)53248, (uint16_t)34798, (uint16_t)14272, (uint16_t)40369, (uint16_t)28660} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_consumed_GET(pack) == (int32_t) -175299373);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)64);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)20029);
    {
        uint8_t exemplary[] =  {(uint8_t)130, (uint8_t)203, (uint8_t)148, (uint8_t)66, (uint8_t)243, (uint8_t)178, (uint8_t)65, (uint8_t)54} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)91, (uint8_t)7, (uint8_t)213, (uint8_t)141, (uint8_t)160, (uint8_t)181, (uint8_t)223, (uint8_t)214, (uint8_t)98, (uint8_t)167, (uint8_t)3, (uint8_t)247, (uint8_t)41, (uint8_t)208, (uint8_t)47, (uint8_t)96, (uint8_t)249, (uint8_t)77} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)2033622656L);
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)50, (uint8_t)249, (uint8_t)208, (uint8_t)133, (uint8_t)213, (uint8_t)68, (uint8_t)198} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3542781889L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)112435539L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)37612);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1345676899L);
    {
        uint8_t exemplary[] =  {(uint8_t)162, (uint8_t)236, (uint8_t)237, (uint8_t)106, (uint8_t)21, (uint8_t)93, (uint8_t)187, (uint8_t)11} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP);
    assert(p148_uid_GET(pack) == (uint64_t)8917899589896546499L);
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p149_angle_x_GET(pack) == (float) -1.6949234E38F);
    assert(p149_x_TRY(ph) == (float)1.4663395E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_size_x_GET(pack) == (float) -1.8422515E38F);
    {
        float exemplary[] =  {1.6526461E38F, -4.1241215E37F, -8.176324E36F, -2.7780774E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)135);
    assert(p149_size_y_GET(pack) == (float)2.3309452E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)7101268281404681725L);
    assert(p149_angle_y_GET(pack) == (float) -1.8193962E38F);
    assert(p149_distance_GET(pack) == (float)9.652033E37F);
    assert(p149_y_TRY(ph) == (float)3.5069947E36F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p149_z_TRY(ph) == (float) -1.7675467E38F);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -1.5432947E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -1.3658147E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -2.849157E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)4124172350540905314L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.2530092E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -1.991166E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -1.0746971E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -2.6913839E38F);
    assert(p230_mag_ratio_GET(pack) == (float)3.0555533E38F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_horiz_GET(pack) == (float) -3.1989964E38F);
    assert(p231_var_vert_GET(pack) == (float) -1.8538331E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -1.8510583E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)5851683437562058257L);
    assert(p231_wind_alt_GET(pack) == (float)2.374925E38F);
    assert(p231_wind_y_GET(pack) == (float) -2.1744822E38F);
    assert(p231_wind_z_GET(pack) == (float)2.223529E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)4.152779E37F);
    assert(p231_wind_x_GET(pack) == (float)9.929221E37F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p232_speed_accuracy_GET(pack) == (float)3.30279E37F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)3244619747L);
    assert(p232_alt_GET(pack) == (float)2.9555517E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
    assert(p232_horiz_accuracy_GET(pack) == (float) -6.015086E37F);
    assert(p232_hdop_GET(pack) == (float)7.746753E37F);
    assert(p232_vd_GET(pack) == (float) -8.76137E37F);
    assert(p232_ve_GET(pack) == (float)2.5670925E38F);
    assert(p232_vdop_GET(pack) == (float) -7.4021805E37F);
    assert(p232_vn_GET(pack) == (float) -5.972222E37F);
    assert(p232_lon_GET(pack) == (int32_t)1181191409);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p232_lat_GET(pack) == (int32_t)626231187);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p232_vert_accuracy_GET(pack) == (float) -1.7444286E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)4954183652961731119L);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)32191);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)70);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)185, (uint8_t)45, (uint8_t)221, (uint8_t)178, (uint8_t)16, (uint8_t)132, (uint8_t)79, (uint8_t)73, (uint8_t)158, (uint8_t)51, (uint8_t)168, (uint8_t)45, (uint8_t)38, (uint8_t)30, (uint8_t)219, (uint8_t)150, (uint8_t)129, (uint8_t)226, (uint8_t)156, (uint8_t)140, (uint8_t)155, (uint8_t)106, (uint8_t)33, (uint8_t)5, (uint8_t)214, (uint8_t)73, (uint8_t)225, (uint8_t)116, (uint8_t)108, (uint8_t)227, (uint8_t)60, (uint8_t)108, (uint8_t)47, (uint8_t)205, (uint8_t)248, (uint8_t)98, (uint8_t)233, (uint8_t)83, (uint8_t)137, (uint8_t)145, (uint8_t)243, (uint8_t)56, (uint8_t)190, (uint8_t)114, (uint8_t)140, (uint8_t)201, (uint8_t)56, (uint8_t)196, (uint8_t)56, (uint8_t)210, (uint8_t)42, (uint8_t)16, (uint8_t)34, (uint8_t)95, (uint8_t)200, (uint8_t)185, (uint8_t)200, (uint8_t)204, (uint8_t)58, (uint8_t)222, (uint8_t)118, (uint8_t)78, (uint8_t)89, (uint8_t)125, (uint8_t)106, (uint8_t)173, (uint8_t)169, (uint8_t)70, (uint8_t)216, (uint8_t)57, (uint8_t)130, (uint8_t)76, (uint8_t)235, (uint8_t)64, (uint8_t)122, (uint8_t)37, (uint8_t)50, (uint8_t)217, (uint8_t)204, (uint8_t)237, (uint8_t)51, (uint8_t)17, (uint8_t)53, (uint8_t)218, (uint8_t)208, (uint8_t)48, (uint8_t)149, (uint8_t)191, (uint8_t)98, (uint8_t)254, (uint8_t)236, (uint8_t)139, (uint8_t)88, (uint8_t)159, (uint8_t)171, (uint8_t)21, (uint8_t)66, (uint8_t)30, (uint8_t)61, (uint8_t)197, (uint8_t)61, (uint8_t)37, (uint8_t)95, (uint8_t)19, (uint8_t)50, (uint8_t)42, (uint8_t)160, (uint8_t)174, (uint8_t)190, (uint8_t)36, (uint8_t)145, (uint8_t)71, (uint8_t)55, (uint8_t)92, (uint8_t)143, (uint8_t)124, (uint8_t)104, (uint8_t)89, (uint8_t)47, (uint8_t)199, (uint8_t)186, (uint8_t)223, (uint8_t)217, (uint8_t)50, (uint8_t)146, (uint8_t)106, (uint8_t)251, (uint8_t)219, (uint8_t)107, (uint8_t)112, (uint8_t)189, (uint8_t)171, (uint8_t)30, (uint8_t)96, (uint8_t)250, (uint8_t)31, (uint8_t)9, (uint8_t)27, (uint8_t)8, (uint8_t)79, (uint8_t)253, (uint8_t)31, (uint8_t)196, (uint8_t)100, (uint8_t)164, (uint8_t)1, (uint8_t)89, (uint8_t)51, (uint8_t)159, (uint8_t)147, (uint8_t)75, (uint8_t)46, (uint8_t)9, (uint8_t)208, (uint8_t)130, (uint8_t)43, (uint8_t)103, (uint8_t)1, (uint8_t)130, (uint8_t)67, (uint8_t)50, (uint8_t)82, (uint8_t)10, (uint8_t)107, (uint8_t)73, (uint8_t)228, (uint8_t)245, (uint8_t)19, (uint8_t)141, (uint8_t)166, (uint8_t)210, (uint8_t)153, (uint8_t)137, (uint8_t)101, (uint8_t)138, (uint8_t)9, (uint8_t)193, (uint8_t)245, (uint8_t)27} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -7522);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -35);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -7553);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)42820);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1720033366L);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -25710);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)45);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)78);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -16);
    assert(p234_longitude_GET(pack) == (int32_t) -380281959);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)45256);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)20217);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -10792);
    assert(p234_latitude_GET(pack) == (int32_t) -1734152714);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)1992743661L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)799830589L);
    assert(p241_vibration_y_GET(pack) == (float) -2.866285E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)6560928051464057959L);
    assert(p241_vibration_x_GET(pack) == (float)1.3767085E38F);
    assert(p241_vibration_z_GET(pack) == (float) -1.5970138E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)214989208L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_latitude_GET(pack) == (int32_t) -1298436124);
    assert(p242_z_GET(pack) == (float) -1.0028086E38F);
    {
        float exemplary[] =  {1.3792931E38F, 3.3747617E38F, -1.3354088E38F, -1.9748911E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t)988792597);
    assert(p242_altitude_GET(pack) == (int32_t) -1154741668);
    assert(p242_approach_z_GET(pack) == (float)2.9670378E38F);
    assert(p242_approach_y_GET(pack) == (float)6.2852023E37F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)575141900146189234L);
    assert(p242_y_GET(pack) == (float)2.0701268E38F);
    assert(p242_approach_x_GET(pack) == (float)2.1065004E38F);
    assert(p242_x_GET(pack) == (float) -2.5291796E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_y_GET(pack) == (float)1.4497441E38F);
    assert(p243_latitude_GET(pack) == (int32_t)1298592727);
    {
        float exemplary[] =  {5.222147E37F, -4.6233885E37F, -8.85017E37F, -1.2642371E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p243_y_GET(pack) == (float)1.939387E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)8717836766971549615L);
    assert(p243_x_GET(pack) == (float) -2.9684404E38F);
    assert(p243_approach_z_GET(pack) == (float)4.0756807E37F);
    assert(p243_z_GET(pack) == (float) -1.9945505E38F);
    assert(p243_approach_x_GET(pack) == (float) -1.0705927E38F);
    assert(p243_altitude_GET(pack) == (int32_t)1821608584);
    assert(p243_longitude_GET(pack) == (int32_t) -1355640098);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)950098489);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)39067);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)34182);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE);
    assert(p246_lat_GET(pack) == (int32_t)237967964);
    assert(p246_altitude_GET(pack) == (int32_t) -138872184);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)54717);
    assert(p246_callsign_LEN(ph) == 9);
    {
        char16_t * exemplary = u"iRnnmteum";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)45390);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -1387);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)665356756L);
    assert(p246_lon_GET(pack) == (int32_t)491717234);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -2.2626225E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)2.7013074E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.7566149E38F);
    assert(p247_id_GET(pack) == (uint32_t)2395526728L);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    {
        uint8_t exemplary[] =  {(uint8_t)56, (uint8_t)61, (uint8_t)142, (uint8_t)34, (uint8_t)251, (uint8_t)178, (uint8_t)177, (uint8_t)153, (uint8_t)231, (uint8_t)87, (uint8_t)129, (uint8_t)178, (uint8_t)43, (uint8_t)61, (uint8_t)121, (uint8_t)200, (uint8_t)232, (uint8_t)136, (uint8_t)242, (uint8_t)55, (uint8_t)113, (uint8_t)32, (uint8_t)137, (uint8_t)238, (uint8_t)105, (uint8_t)237, (uint8_t)173, (uint8_t)190, (uint8_t)66, (uint8_t)241, (uint8_t)92, (uint8_t)122, (uint8_t)49, (uint8_t)157, (uint8_t)252, (uint8_t)17, (uint8_t)253, (uint8_t)126, (uint8_t)102, (uint8_t)12, (uint8_t)86, (uint8_t)85, (uint8_t)13, (uint8_t)175, (uint8_t)43, (uint8_t)92, (uint8_t)69, (uint8_t)182, (uint8_t)150, (uint8_t)204, (uint8_t)63, (uint8_t)130, (uint8_t)169, (uint8_t)8, (uint8_t)107, (uint8_t)87, (uint8_t)84, (uint8_t)75, (uint8_t)16, (uint8_t)118, (uint8_t)85, (uint8_t)89, (uint8_t)131, (uint8_t)96, (uint8_t)98, (uint8_t)218, (uint8_t)191, (uint8_t)233, (uint8_t)199, (uint8_t)43, (uint8_t)31, (uint8_t)168, (uint8_t)134, (uint8_t)209, (uint8_t)117, (uint8_t)63, (uint8_t)170, (uint8_t)41, (uint8_t)171, (uint8_t)119, (uint8_t)30, (uint8_t)54, (uint8_t)52, (uint8_t)46, (uint8_t)217, (uint8_t)179, (uint8_t)72, (uint8_t)140, (uint8_t)79, (uint8_t)200, (uint8_t)180, (uint8_t)35, (uint8_t)213, (uint8_t)240, (uint8_t)128, (uint8_t)185, (uint8_t)24, (uint8_t)30, (uint8_t)6, (uint8_t)242, (uint8_t)222, (uint8_t)128, (uint8_t)198, (uint8_t)22, (uint8_t)140, (uint8_t)200, (uint8_t)252, (uint8_t)73, (uint8_t)129, (uint8_t)223, (uint8_t)223, (uint8_t)31, (uint8_t)162, (uint8_t)15, (uint8_t)199, (uint8_t)202, (uint8_t)53, (uint8_t)29, (uint8_t)42, (uint8_t)67, (uint8_t)191, (uint8_t)115, (uint8_t)216, (uint8_t)182, (uint8_t)145, (uint8_t)140, (uint8_t)216, (uint8_t)109, (uint8_t)177, (uint8_t)147, (uint8_t)157, (uint8_t)46, (uint8_t)43, (uint8_t)186, (uint8_t)209, (uint8_t)231, (uint8_t)238, (uint8_t)109, (uint8_t)172, (uint8_t)160, (uint8_t)127, (uint8_t)209, (uint8_t)225, (uint8_t)207, (uint8_t)176, (uint8_t)87, (uint8_t)89, (uint8_t)82, (uint8_t)213, (uint8_t)236, (uint8_t)187, (uint8_t)194, (uint8_t)219, (uint8_t)222, (uint8_t)87, (uint8_t)133, (uint8_t)98, (uint8_t)76, (uint8_t)13, (uint8_t)111, (uint8_t)202, (uint8_t)88, (uint8_t)209, (uint8_t)81, (uint8_t)194, (uint8_t)7, (uint8_t)248, (uint8_t)214, (uint8_t)161, (uint8_t)187, (uint8_t)155, (uint8_t)155, (uint8_t)218, (uint8_t)86, (uint8_t)26, (uint8_t)221, (uint8_t)173, (uint8_t)68, (uint8_t)175, (uint8_t)80, (uint8_t)64, (uint8_t)166, (uint8_t)124, (uint8_t)1, (uint8_t)246, (uint8_t)46, (uint8_t)178, (uint8_t)74, (uint8_t)72, (uint8_t)41, (uint8_t)90, (uint8_t)156, (uint8_t)114, (uint8_t)209, (uint8_t)62, (uint8_t)41, (uint8_t)13, (uint8_t)90, (uint8_t)25, (uint8_t)0, (uint8_t)230, (uint8_t)245, (uint8_t)51, (uint8_t)177, (uint8_t)46, (uint8_t)81, (uint8_t)129, (uint8_t)148, (uint8_t)116, (uint8_t)58, (uint8_t)180, (uint8_t)120, (uint8_t)161, (uint8_t)208, (uint8_t)244, (uint8_t)65, (uint8_t)243, (uint8_t)29, (uint8_t)212, (uint8_t)76, (uint8_t)222, (uint8_t)38, (uint8_t)162, (uint8_t)5, (uint8_t)48, (uint8_t)98, (uint8_t)38, (uint8_t)216, (uint8_t)39, (uint8_t)159, (uint8_t)239, (uint8_t)117, (uint8_t)103, (uint8_t)190, (uint8_t)250, (uint8_t)76, (uint8_t)36, (uint8_t)95, (uint8_t)88, (uint8_t)120, (uint8_t)248, (uint8_t)52, (uint8_t)190, (uint8_t)32, (uint8_t)235, (uint8_t)150, (uint8_t)154, (uint8_t)207, (uint8_t)140} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)30348);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)8403);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)240);
    {
        int8_t exemplary[] =  {(int8_t)4, (int8_t)73, (int8_t) -117, (int8_t) -103, (int8_t) -103, (int8_t)28, (int8_t) -19, (int8_t) -10, (int8_t) -112, (int8_t)46, (int8_t)7, (int8_t) -74, (int8_t)1, (int8_t) -91, (int8_t) -98, (int8_t) -17, (int8_t)93, (int8_t)56, (int8_t)93, (int8_t) -113, (int8_t)107, (int8_t)80, (int8_t) -20, (int8_t)21, (int8_t) -73, (int8_t) -53, (int8_t) -11, (int8_t) -112, (int8_t)6, (int8_t)64, (int8_t) -78, (int8_t)67} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float)1.168707E38F);
    assert(p250_x_GET(pack) == (float)3.1071465E38F);
    assert(p250_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"iczehvof";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_z_GET(pack) == (float)1.6441413E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5023732387116550688L);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"dkzCwiJwzr";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3644101452L);
    assert(p251_value_GET(pack) == (float) -7.5580397E36F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t)1067250052);
    assert(p252_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"mTddzrpddj";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)599121679L);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 18);
    {
        char16_t * exemplary = u"ogtezpnevcgvhitcns";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float) -2.9182597E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)2356209675L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)4759514120119289553L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)116);
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)219, (uint8_t)217, (uint8_t)158, (uint8_t)254, (uint8_t)244, (uint8_t)204, (uint8_t)85, (uint8_t)77, (uint8_t)192, (uint8_t)151, (uint8_t)245, (uint8_t)255, (uint8_t)58, (uint8_t)147, (uint8_t)46, (uint8_t)217, (uint8_t)79, (uint8_t)204, (uint8_t)81, (uint8_t)161, (uint8_t)127, (uint8_t)228, (uint8_t)45, (uint8_t)9, (uint8_t)74, (uint8_t)144, (uint8_t)99, (uint8_t)67, (uint8_t)160, (uint8_t)250, (uint8_t)215} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3052068457L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)33263741L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p258_tune_LEN(ph) == 5);
    {
        char16_t * exemplary = u"tozcg";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_h_GET(pack) == (float)1.9747617E38F);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)15894);
    assert(p259_sensor_size_v_GET(pack) == (float) -8.4406495E37F);
    assert(p259_cam_definition_uri_LEN(ph) == 110);
    {
        char16_t * exemplary = u"qmRdxzgiudepldoapmwaqtmVxcwlKvupgfOrmxzjmtnhklEpkeUebgvBsyUqitWpievInigmntcUdwvrkexedhKisdlxgdtyyofpcwahnpztnz";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 220);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float) -9.893087E37F);
    {
        uint8_t exemplary[] =  {(uint8_t)48, (uint8_t)250, (uint8_t)33, (uint8_t)88, (uint8_t)61, (uint8_t)167, (uint8_t)59, (uint8_t)7, (uint8_t)90, (uint8_t)225, (uint8_t)83, (uint8_t)99, (uint8_t)82, (uint8_t)74, (uint8_t)233, (uint8_t)215, (uint8_t)218, (uint8_t)86, (uint8_t)170, (uint8_t)174, (uint8_t)132, (uint8_t)7, (uint8_t)71, (uint8_t)5, (uint8_t)250, (uint8_t)161, (uint8_t)104, (uint8_t)3, (uint8_t)94, (uint8_t)109, (uint8_t)115, (uint8_t)105} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)44754);
    assert(p259_firmware_version_GET(pack) == (uint32_t)586924252L);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)577165625L);
    {
        uint8_t exemplary[] =  {(uint8_t)72, (uint8_t)183, (uint8_t)249, (uint8_t)185, (uint8_t)196, (uint8_t)19, (uint8_t)182, (uint8_t)240, (uint8_t)139, (uint8_t)77, (uint8_t)49, (uint8_t)103, (uint8_t)108, (uint8_t)81, (uint8_t)143, (uint8_t)30, (uint8_t)43, (uint8_t)14, (uint8_t)178, (uint8_t)241, (uint8_t)158, (uint8_t)159, (uint8_t)186, (uint8_t)225, (uint8_t)89, (uint8_t)226, (uint8_t)120, (uint8_t)68, (uint8_t)114, (uint8_t)147, (uint8_t)90, (uint8_t)68} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)35800);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)2108997621L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_write_speed_GET(pack) == (float) -2.8285828E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p261_total_capacity_GET(pack) == (float) -1.5792938E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p261_available_capacity_GET(pack) == (float)1.1602998E38F);
    assert(p261_read_speed_GET(pack) == (float) -1.5280247E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p261_used_capacity_GET(pack) == (float)2.3382012E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3302372977L);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p262_available_capacity_GET(pack) == (float) -4.9887887E37F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3070506308L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p262_image_interval_GET(pack) == (float) -2.7720014E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2642282647L);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)53);
    assert(p263_alt_GET(pack) == (int32_t)109063020);
    assert(p263_image_index_GET(pack) == (int32_t) -2000711644);
    assert(p263_time_utc_GET(pack) == (uint64_t)8668743606971487687L);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p263_lon_GET(pack) == (int32_t) -1275919850);
    assert(p263_file_url_LEN(ph) == 198);
    {
        char16_t * exemplary = u"phQmNhomoctwnssknomxufhzqlkhyzchdrpcQTsrbggazxeslKwcrlcpQMgrbasfmzyhmNdwBqjYlwistsjxztesKeRnbywidbvvhhcjlinbrmddblwsZfjukinolgjQsaqzpOzwfYrmuxbpzovkydfqprYAmzsxeczevirlmzltrPeffwrUhubyxifoLpkbnmqnuu";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 396);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {8.2884533E37F, 2.5477144E38F, -7.694992E37F, 3.3994414E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)3895740126L);
    assert(p263_lat_GET(pack) == (int32_t) -1078123946);
    assert(p263_relative_alt_GET(pack) == (int32_t) -24478503);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)4879600289714646429L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)1686345821L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)255503551503881297L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)874959079588848711L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float)3.3692891E38F);
    assert(p265_roll_GET(pack) == (float)3.2338298E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1448016743L);
    assert(p265_yaw_GET(pack) == (float)5.8667184E37F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)50);
    {
        uint8_t exemplary[] =  {(uint8_t)60, (uint8_t)74, (uint8_t)129, (uint8_t)189, (uint8_t)248, (uint8_t)70, (uint8_t)91, (uint8_t)239, (uint8_t)35, (uint8_t)181, (uint8_t)138, (uint8_t)193, (uint8_t)173, (uint8_t)245, (uint8_t)161, (uint8_t)127, (uint8_t)177, (uint8_t)197, (uint8_t)159, (uint8_t)47, (uint8_t)83, (uint8_t)162, (uint8_t)7, (uint8_t)178, (uint8_t)242, (uint8_t)156, (uint8_t)14, (uint8_t)49, (uint8_t)100, (uint8_t)124, (uint8_t)42, (uint8_t)155, (uint8_t)100, (uint8_t)155, (uint8_t)244, (uint8_t)94, (uint8_t)152, (uint8_t)246, (uint8_t)249, (uint8_t)89, (uint8_t)199, (uint8_t)92, (uint8_t)76, (uint8_t)164, (uint8_t)130, (uint8_t)37, (uint8_t)110, (uint8_t)193, (uint8_t)28, (uint8_t)140, (uint8_t)91, (uint8_t)189, (uint8_t)226, (uint8_t)161, (uint8_t)42, (uint8_t)91, (uint8_t)163, (uint8_t)209, (uint8_t)131, (uint8_t)7, (uint8_t)31, (uint8_t)54, (uint8_t)158, (uint8_t)105, (uint8_t)186, (uint8_t)114, (uint8_t)20, (uint8_t)51, (uint8_t)254, (uint8_t)19, (uint8_t)54, (uint8_t)230, (uint8_t)236, (uint8_t)184, (uint8_t)205, (uint8_t)33, (uint8_t)101, (uint8_t)109, (uint8_t)133, (uint8_t)68, (uint8_t)224, (uint8_t)173, (uint8_t)156, (uint8_t)35, (uint8_t)171, (uint8_t)218, (uint8_t)235, (uint8_t)77, (uint8_t)238, (uint8_t)215, (uint8_t)164, (uint8_t)241, (uint8_t)38, (uint8_t)51, (uint8_t)43, (uint8_t)114, (uint8_t)221, (uint8_t)167, (uint8_t)74, (uint8_t)73, (uint8_t)57, (uint8_t)234, (uint8_t)119, (uint8_t)221, (uint8_t)69, (uint8_t)124, (uint8_t)72, (uint8_t)254, (uint8_t)36, (uint8_t)35, (uint8_t)32, (uint8_t)183, (uint8_t)28, (uint8_t)241, (uint8_t)24, (uint8_t)7, (uint8_t)22, (uint8_t)73, (uint8_t)200, (uint8_t)235, (uint8_t)224, (uint8_t)70, (uint8_t)103, (uint8_t)115, (uint8_t)29, (uint8_t)18, (uint8_t)209, (uint8_t)82, (uint8_t)119, (uint8_t)39, (uint8_t)76, (uint8_t)27, (uint8_t)79, (uint8_t)76, (uint8_t)35, (uint8_t)18, (uint8_t)204, (uint8_t)6, (uint8_t)18, (uint8_t)142, (uint8_t)251, (uint8_t)230, (uint8_t)250, (uint8_t)185, (uint8_t)97, (uint8_t)204, (uint8_t)83, (uint8_t)193, (uint8_t)99, (uint8_t)185, (uint8_t)184, (uint8_t)233, (uint8_t)108, (uint8_t)146, (uint8_t)188, (uint8_t)138, (uint8_t)96, (uint8_t)134, (uint8_t)4, (uint8_t)205, (uint8_t)116, (uint8_t)74, (uint8_t)56, (uint8_t)38, (uint8_t)145, (uint8_t)229, (uint8_t)158, (uint8_t)146, (uint8_t)243, (uint8_t)62, (uint8_t)119, (uint8_t)164, (uint8_t)61, (uint8_t)207, (uint8_t)29, (uint8_t)190, (uint8_t)211, (uint8_t)70, (uint8_t)132, (uint8_t)85, (uint8_t)49, (uint8_t)7, (uint8_t)112, (uint8_t)27, (uint8_t)102, (uint8_t)5, (uint8_t)193, (uint8_t)198, (uint8_t)20, (uint8_t)101, (uint8_t)55, (uint8_t)173, (uint8_t)120, (uint8_t)220, (uint8_t)145, (uint8_t)152, (uint8_t)170, (uint8_t)232, (uint8_t)148, (uint8_t)213, (uint8_t)190, (uint8_t)246, (uint8_t)49, (uint8_t)155, (uint8_t)209, (uint8_t)74, (uint8_t)39, (uint8_t)23, (uint8_t)54, (uint8_t)108, (uint8_t)189, (uint8_t)165, (uint8_t)114, (uint8_t)147, (uint8_t)152, (uint8_t)101, (uint8_t)8, (uint8_t)182, (uint8_t)248, (uint8_t)92, (uint8_t)36, (uint8_t)16, (uint8_t)90, (uint8_t)55, (uint8_t)204, (uint8_t)195, (uint8_t)92, (uint8_t)63, (uint8_t)238, (uint8_t)97, (uint8_t)68, (uint8_t)94, (uint8_t)87, (uint8_t)138, (uint8_t)56, (uint8_t)84, (uint8_t)162, (uint8_t)58, (uint8_t)96, (uint8_t)33, (uint8_t)22, (uint8_t)188, (uint8_t)159, (uint8_t)61, (uint8_t)62, (uint8_t)172, (uint8_t)189, (uint8_t)127, (uint8_t)85} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)37603);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)93);
    {
        uint8_t exemplary[] =  {(uint8_t)28, (uint8_t)37, (uint8_t)157, (uint8_t)200, (uint8_t)251, (uint8_t)159, (uint8_t)30, (uint8_t)34, (uint8_t)196, (uint8_t)20, (uint8_t)203, (uint8_t)38, (uint8_t)241, (uint8_t)240, (uint8_t)124, (uint8_t)211, (uint8_t)231, (uint8_t)15, (uint8_t)149, (uint8_t)44, (uint8_t)23, (uint8_t)29, (uint8_t)232, (uint8_t)176, (uint8_t)229, (uint8_t)35, (uint8_t)118, (uint8_t)110, (uint8_t)248, (uint8_t)55, (uint8_t)167, (uint8_t)123, (uint8_t)6, (uint8_t)44, (uint8_t)58, (uint8_t)2, (uint8_t)2, (uint8_t)173, (uint8_t)149, (uint8_t)172, (uint8_t)58, (uint8_t)220, (uint8_t)119, (uint8_t)145, (uint8_t)104, (uint8_t)211, (uint8_t)27, (uint8_t)99, (uint8_t)222, (uint8_t)107, (uint8_t)211, (uint8_t)45, (uint8_t)173, (uint8_t)80, (uint8_t)98, (uint8_t)77, (uint8_t)183, (uint8_t)72, (uint8_t)13, (uint8_t)92, (uint8_t)166, (uint8_t)166, (uint8_t)173, (uint8_t)5, (uint8_t)203, (uint8_t)109, (uint8_t)148, (uint8_t)202, (uint8_t)83, (uint8_t)101, (uint8_t)195, (uint8_t)78, (uint8_t)232, (uint8_t)201, (uint8_t)66, (uint8_t)194, (uint8_t)255, (uint8_t)2, (uint8_t)14, (uint8_t)85, (uint8_t)83, (uint8_t)13, (uint8_t)227, (uint8_t)215, (uint8_t)213, (uint8_t)155, (uint8_t)128, (uint8_t)107, (uint8_t)33, (uint8_t)69, (uint8_t)233, (uint8_t)124, (uint8_t)68, (uint8_t)24, (uint8_t)71, (uint8_t)95, (uint8_t)26, (uint8_t)246, (uint8_t)186, (uint8_t)18, (uint8_t)55, (uint8_t)103, (uint8_t)35, (uint8_t)186, (uint8_t)209, (uint8_t)175, (uint8_t)142, (uint8_t)213, (uint8_t)107, (uint8_t)69, (uint8_t)186, (uint8_t)202, (uint8_t)176, (uint8_t)75, (uint8_t)148, (uint8_t)101, (uint8_t)154, (uint8_t)109, (uint8_t)213, (uint8_t)62, (uint8_t)83, (uint8_t)209, (uint8_t)117, (uint8_t)255, (uint8_t)43, (uint8_t)154, (uint8_t)240, (uint8_t)227, (uint8_t)52, (uint8_t)38, (uint8_t)183, (uint8_t)115, (uint8_t)164, (uint8_t)212, (uint8_t)30, (uint8_t)74, (uint8_t)220, (uint8_t)41, (uint8_t)21, (uint8_t)119, (uint8_t)62, (uint8_t)103, (uint8_t)242, (uint8_t)50, (uint8_t)92, (uint8_t)183, (uint8_t)57, (uint8_t)53, (uint8_t)18, (uint8_t)253, (uint8_t)83, (uint8_t)203, (uint8_t)103, (uint8_t)243, (uint8_t)209, (uint8_t)117, (uint8_t)153, (uint8_t)28, (uint8_t)169, (uint8_t)123, (uint8_t)24, (uint8_t)231, (uint8_t)112, (uint8_t)1, (uint8_t)162, (uint8_t)73, (uint8_t)4, (uint8_t)16, (uint8_t)235, (uint8_t)131, (uint8_t)141, (uint8_t)32, (uint8_t)112, (uint8_t)228, (uint8_t)47, (uint8_t)122, (uint8_t)230, (uint8_t)194, (uint8_t)170, (uint8_t)53, (uint8_t)217, (uint8_t)214, (uint8_t)50, (uint8_t)14, (uint8_t)228, (uint8_t)75, (uint8_t)221, (uint8_t)251, (uint8_t)149, (uint8_t)14, (uint8_t)107, (uint8_t)240, (uint8_t)102, (uint8_t)76, (uint8_t)249, (uint8_t)235, (uint8_t)247, (uint8_t)250, (uint8_t)169, (uint8_t)150, (uint8_t)103, (uint8_t)245, (uint8_t)186, (uint8_t)82, (uint8_t)253, (uint8_t)156, (uint8_t)109, (uint8_t)71, (uint8_t)44, (uint8_t)97, (uint8_t)61, (uint8_t)173, (uint8_t)114, (uint8_t)26, (uint8_t)140, (uint8_t)43, (uint8_t)95, (uint8_t)88, (uint8_t)241, (uint8_t)80, (uint8_t)45, (uint8_t)214, (uint8_t)154, (uint8_t)230, (uint8_t)28, (uint8_t)146, (uint8_t)219, (uint8_t)96, (uint8_t)76, (uint8_t)43, (uint8_t)148, (uint8_t)238, (uint8_t)212, (uint8_t)156, (uint8_t)43, (uint8_t)119, (uint8_t)194, (uint8_t)71, (uint8_t)145, (uint8_t)2, (uint8_t)0, (uint8_t)191, (uint8_t)62, (uint8_t)107, (uint8_t)61, (uint8_t)147, (uint8_t)89, (uint8_t)74, (uint8_t)251} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)28922);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)78);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)63635);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)3582);
    assert(p269_framerate_GET(pack) == (float) -1.2616542E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)3567902515L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)13268);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)12429);
    assert(p269_uri_LEN(ph) == 34);
    {
        char16_t * exemplary = u"tmuozqjqkuNvbtvkozjjqdfijreadupfie";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 68);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)47373);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)32942);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)25551);
    assert(p270_uri_LEN(ph) == 59);
    {
        char16_t * exemplary = u"sktkwifpafozhzlifyahajlgaqlamekqdledsgoateonlqkwXpqqqxawnfv";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_framerate_GET(pack) == (float)1.0374664E38F);
    assert(p270_bitrate_GET(pack) == (uint32_t)1433860849L);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 21);
    {
        char16_t * exemplary = u"JstebgskaBkqUsYzrCiha";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 2);
    {
        char16_t * exemplary = u"ks";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)1385);
    {
        uint8_t exemplary[] =  {(uint8_t)86, (uint8_t)97, (uint8_t)160, (uint8_t)67, (uint8_t)47, (uint8_t)56, (uint8_t)210, (uint8_t)161} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)26443);
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)30, (uint8_t)141, (uint8_t)26, (uint8_t)105, (uint8_t)87, (uint8_t)74, (uint8_t)59} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)55042);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_time_usec_GET(pack) == (uint64_t)5931053939311141478L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)2636436347L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)21659);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1733437275L);
    assert(p311_time_usec_GET(pack) == (uint64_t)5288525481692905638L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p311_name_LEN(ph) == 78);
    {
        char16_t * exemplary = u"NbajrEchfedTrxbnfppmhavotqnxwfsniftUYfzvjZgifcLepddbqxtOacgaluXqcpxdwemqzfyfwP";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 156);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2437690819L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)225);
    {
        uint8_t exemplary[] =  {(uint8_t)173, (uint8_t)149, (uint8_t)19, (uint8_t)12, (uint8_t)100, (uint8_t)78, (uint8_t)249, (uint8_t)244, (uint8_t)248, (uint8_t)104, (uint8_t)211, (uint8_t)186, (uint8_t)36, (uint8_t)236, (uint8_t)148, (uint8_t)204} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p320_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"gxexzohxetbVqwzk";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)2576);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)89);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)38192);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p322_param_value_LEN(ph) == 44);
    {
        char16_t * exemplary = u"EzfhgheejjtOzaharxndxxbqshqzwebljepgdndTfUmm";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 88);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"ucsguflxssmdzc";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)46600);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p323_param_value_LEN(ph) == 59);
    {
        char16_t * exemplary = u"vttovfajjvoGwppbdcuvoogjjVjfzsCwytevyedgXIhaxbfxymjcaDGmPbL";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"Wdqsjmzmtddmnum";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED);
    assert(p324_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"jzsCfvindnivj";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 102);
    {
        char16_t * exemplary = u"nkufilpPaFcmekhkffnuwvDSyifqtejprekyckrqfPajjaiXoglrlAhjFalteprzAwqqzzvsadhdeWowvjlLgxkacxcteLtmrkjCgt";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 204);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)48610, (uint16_t)46399, (uint16_t)27883, (uint16_t)56765, (uint16_t)33709, (uint16_t)51424, (uint16_t)9627, (uint16_t)27362, (uint16_t)41025, (uint16_t)59867, (uint16_t)64750, (uint16_t)27499, (uint16_t)59769, (uint16_t)25643, (uint16_t)59788, (uint16_t)1762, (uint16_t)24012, (uint16_t)2303, (uint16_t)14927, (uint16_t)30360, (uint16_t)12278, (uint16_t)4461, (uint16_t)36551, (uint16_t)9652, (uint16_t)12655, (uint16_t)52836, (uint16_t)56631, (uint16_t)64899, (uint16_t)49042, (uint16_t)27291, (uint16_t)16198, (uint16_t)50806, (uint16_t)55572, (uint16_t)4518, (uint16_t)13485, (uint16_t)31607, (uint16_t)51667, (uint16_t)58526, (uint16_t)16343, (uint16_t)14197, (uint16_t)1068, (uint16_t)32416, (uint16_t)30396, (uint16_t)22742, (uint16_t)18019, (uint16_t)9221, (uint16_t)60089, (uint16_t)29901, (uint16_t)329, (uint16_t)31259, (uint16_t)50184, (uint16_t)63963, (uint16_t)20226, (uint16_t)14428, (uint16_t)40444, (uint16_t)9168, (uint16_t)15303, (uint16_t)21279, (uint16_t)60617, (uint16_t)46420, (uint16_t)27713, (uint16_t)31116, (uint16_t)45828, (uint16_t)57588, (uint16_t)60448, (uint16_t)53030, (uint16_t)5983, (uint16_t)27138, (uint16_t)55930, (uint16_t)49595, (uint16_t)45484, (uint16_t)21100} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)44897);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)24759);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p330_time_usec_GET(pack) == (uint64_t)5464788776190332383L);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)58);
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
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED3, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2111593741L, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)53389, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)24067, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)85, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)29809, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)55501, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)55396, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)30353, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)23331, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)28802, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -158, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)1306368784L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3149460314444622265L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vx_SET((float) -8.388056E37F, PH.base.pack) ;
        p3_vz_SET((float) -3.4025482E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)5.458907E37F, PH.base.pack) ;
        p3_afz_SET((float)1.482443E38F, PH.base.pack) ;
        p3_vy_SET((float) -1.2542174E38F, PH.base.pack) ;
        p3_x_SET((float) -1.736333E38F, PH.base.pack) ;
        p3_afy_SET((float) -2.9042804E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)15522, PH.base.pack) ;
        p3_yaw_SET((float)3.929907E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3370480666L, PH.base.pack) ;
        p3_afx_SET((float) -2.4853387E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p3_y_SET((float) -1.9548856E38F, PH.base.pack) ;
        p3_z_SET((float) -1.9552417E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)3275526724013108700L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p4_seq_SET((uint32_t)3731123607L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        {
            char16_t* passkey = u"ogeqziTpP";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"ybnzwjTvcniswSeDxseqboDknkhgxr";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)1668372511L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"nkywkcwza";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)21175, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"adxeblnf";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float) -1.4988212E38F, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)15328, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)52718, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        {
            char16_t* param_id = u"heecfzlsn";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, PH.base.pack) ;
        p23_param_value_SET((float) -1.3995217E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_alt_SET((int32_t)546502488, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1621055977L, &PH) ;
        p24_lon_SET((int32_t) -89674982, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)59763, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)10748, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)48752286L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -925238291, &PH) ;
        p24_h_acc_SET((uint32_t)1231474419L, &PH) ;
        p24_time_usec_SET((uint64_t)5686604473737045278L, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)58561, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)35696, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)3268060685L, &PH) ;
        p24_lat_SET((int32_t) -972758839, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)183, (uint8_t)123, (uint8_t)1, (uint8_t)195, (uint8_t)9, (uint8_t)71, (uint8_t)39, (uint8_t)127, (uint8_t)30, (uint8_t)99, (uint8_t)99, (uint8_t)140, (uint8_t)68, (uint8_t)38, (uint8_t)162, (uint8_t)240, (uint8_t)34, (uint8_t)236, (uint8_t)124, (uint8_t)33};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)189, (uint8_t)91, (uint8_t)211, (uint8_t)193, (uint8_t)7, (uint8_t)72, (uint8_t)251, (uint8_t)91, (uint8_t)57, (uint8_t)56, (uint8_t)13, (uint8_t)133, (uint8_t)27, (uint8_t)40, (uint8_t)70, (uint8_t)243, (uint8_t)123, (uint8_t)23, (uint8_t)104, (uint8_t)67};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)227, (uint8_t)33, (uint8_t)238, (uint8_t)164, (uint8_t)78, (uint8_t)56, (uint8_t)197, (uint8_t)35, (uint8_t)242, (uint8_t)163, (uint8_t)127, (uint8_t)74, (uint8_t)144, (uint8_t)93, (uint8_t)84, (uint8_t)231, (uint8_t)38, (uint8_t)54, (uint8_t)73, (uint8_t)30};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)189, (uint8_t)98, (uint8_t)195, (uint8_t)212, (uint8_t)39, (uint8_t)111, (uint8_t)113, (uint8_t)255, (uint8_t)199, (uint8_t)222, (uint8_t)1, (uint8_t)134, (uint8_t)234, (uint8_t)84, (uint8_t)100, (uint8_t)60, (uint8_t)135, (uint8_t)94, (uint8_t)169, (uint8_t)113};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)21, (uint8_t)170, (uint8_t)6, (uint8_t)167, (uint8_t)29, (uint8_t)145, (uint8_t)90, (uint8_t)187, (uint8_t)42, (uint8_t)211, (uint8_t)140, (uint8_t)71, (uint8_t)98, (uint8_t)192, (uint8_t)57, (uint8_t)141, (uint8_t)210, (uint8_t)157, (uint8_t)119, (uint8_t)14};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)3276058377L, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)13741, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -25490, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)1719, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)15673, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -18464, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)9952, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -22027, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -25882, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)1435, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_zmag_SET((int16_t)(int16_t) -1079, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)25140, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -26508, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -7531, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -31244, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)5515856878550614031L, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -414, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)3167, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -2500, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)1084, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -10744, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)7307129725482014966L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)32299, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)6763, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)12385, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float) -3.0272145E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)12360, PH.base.pack) ;
        p29_press_diff_SET((float) -1.809151E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1357191805L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_roll_SET((float)6.3257393E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)995463298L, PH.base.pack) ;
        p30_yaw_SET((float)3.0267474E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)5.470659E37F, PH.base.pack) ;
        p30_pitch_SET((float) -1.5683162E38F, PH.base.pack) ;
        p30_yawspeed_SET((float) -2.3486207E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)3.084712E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q3_SET((float) -4.690723E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2568461791L, PH.base.pack) ;
        p31_yawspeed_SET((float) -8.524665E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)1.4016034E37F, PH.base.pack) ;
        p31_q1_SET((float)8.809495E37F, PH.base.pack) ;
        p31_pitchspeed_SET((float)2.6504726E38F, PH.base.pack) ;
        p31_q2_SET((float)2.6901554E38F, PH.base.pack) ;
        p31_q4_SET((float) -1.64537E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)3546715808L, PH.base.pack) ;
        p32_vy_SET((float)2.5522618E38F, PH.base.pack) ;
        p32_y_SET((float) -2.4520746E38F, PH.base.pack) ;
        p32_vx_SET((float) -2.9863246E38F, PH.base.pack) ;
        p32_x_SET((float) -5.959607E36F, PH.base.pack) ;
        p32_z_SET((float)1.8166175E38F, PH.base.pack) ;
        p32_vz_SET((float) -2.3719651E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_vy_SET((int16_t)(int16_t)20977, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1176544429, PH.base.pack) ;
        p33_lon_SET((int32_t) -1994171876, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)7210, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -25073, PH.base.pack) ;
        p33_lat_SET((int32_t)1231645915, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1832901716L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)7649, PH.base.pack) ;
        p33_alt_SET((int32_t)2038312002, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan8_scaled_SET((int16_t)(int16_t)28234, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -891, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)3980, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -26358, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -29369, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)23600, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -26007, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3323129562L, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -23928, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_port_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)2604, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)14844, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)17703, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)6942, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)3664, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)29136, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)25759, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)1153, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1542424469L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_port_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)20042, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)61514, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)63031, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)21214, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)21618, &PH) ;
        p36_time_usec_SET((uint32_t)4047164298L, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)56665, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)30790, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)9501, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)52910, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)19113, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)2712, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)45711, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)60981, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)10623, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)47058, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)39805, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -13634, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -8459, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)10835, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)11294, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_param4_SET((float)1.829423E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)55122, PH.base.pack) ;
        p39_param1_SET((float)1.8376893E38F, PH.base.pack) ;
        p39_param2_SET((float)1.9824595E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_y_SET((float)2.7549504E38F, PH.base.pack) ;
        p39_x_SET((float)1.7626203E38F, PH.base.pack) ;
        p39_z_SET((float) -2.5373112E38F, PH.base.pack) ;
        p39_param3_SET((float) -8.863477E37F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_DO_FOLLOW, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)10895, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)24428, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)57364, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)54229, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)15528, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t)734026057, PH.base.pack) ;
        p48_altitude_SET((int32_t)1481644176, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1761394955046902823L, &PH) ;
        p48_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1998078549, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_altitude_SET((int32_t)1870943713, PH.base.pack) ;
        p49_longitude_SET((int32_t) -383806453, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)4225157407281284095L, &PH) ;
        p49_latitude_SET((int32_t) -699738576, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p50_param_value_min_SET((float) -2.8901838E37F, PH.base.pack) ;
        p50_param_value_max_SET((float)1.97105E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -21170, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p50_scale_SET((float)1.356137E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"wkxferonrbdpYiy";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value0_SET((float)5.0163115E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)25159, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p54_p2y_SET((float) -2.6476637E38F, PH.base.pack) ;
        p54_p1z_SET((float) -3.1185241E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p54_p1y_SET((float)1.6032298E38F, PH.base.pack) ;
        p54_p2x_SET((float)1.3893395E37F, PH.base.pack) ;
        p54_p1x_SET((float)9.897746E37F, PH.base.pack) ;
        p54_p2z_SET((float) -3.7206293E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float) -1.325486E38F, PH.base.pack) ;
        p55_p1y_SET((float) -1.5222091E38F, PH.base.pack) ;
        p55_p1z_SET((float) -6.3022446E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p55_p2y_SET((float)2.41154E38F, PH.base.pack) ;
        p55_p2x_SET((float)1.4757776E38F, PH.base.pack) ;
        p55_p2z_SET((float)2.452846E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_yawspeed_SET((float)3.0602625E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.776401E38F, PH.base.pack) ;
        p61_rollspeed_SET((float) -3.3046546E38F, PH.base.pack) ;
        {
            float q[] =  {-3.6274654E37F, 8.1353085E37F, -1.3520544E36F, -8.4541474E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float covariance[] =  {-1.8253769E37F, -3.1119325E38F, -3.0696287E38F, -1.1853452E38F, 2.9096948E38F, 1.1220174E37F, -1.0562288E38F, 1.6955624E38F, 2.1708222E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)8159047715814304118L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_alt_error_SET((float)5.2900033E37F, PH.base.pack) ;
        p62_aspd_error_SET((float) -3.0807496E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -1.2290909E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)1.548782E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)21972, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.9961066E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)20544, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -32404, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_time_usec_SET((uint64_t)2691314528630411946L, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1825919170, PH.base.pack) ;
        p63_vx_SET((float)1.1290167E38F, PH.base.pack) ;
        p63_vy_SET((float) -1.8683495E38F, PH.base.pack) ;
        p63_lat_SET((int32_t)1698118391, PH.base.pack) ;
        p63_lon_SET((int32_t)16359475, PH.base.pack) ;
        p63_alt_SET((int32_t)807412950, PH.base.pack) ;
        {
            float covariance[] =  {-8.4655735E37F, 4.4738077E37F, -2.625713E38F, -3.2142186E38F, 2.9044415E38F, -1.7398507E38F, 2.4776541E38F, -3.1327039E37F, 7.138031E37F, 2.2474403E38F, 2.5989545E38F, -2.5140975E37F, -1.0259047E38F, -1.0317322E38F, -1.8669331E38F, -3.2212012E38F, -2.195169E38F, -2.456672E38F, -2.6350844E38F, 2.1797502E38F, 1.9091181E38F, -1.1911467E37F, 1.8414002E38F, 9.809157E37F, 7.981286E37F, 1.4740694E37F, -1.6152132E38F, -1.057354E38F, -1.8586786E38F, -2.9146626E38F, -3.27326E37F, -2.3123095E38F, 1.7064236E38F, -1.169135E37F, -5.8219414E37F, 5.113582E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float)3.13737E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_x_SET((float)1.1427123E38F, PH.base.pack) ;
        p64_az_SET((float) -3.2923507E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.3068659E37F, -2.9641644E38F, -2.1893549E38F, 5.1765374E36F, -3.2726357E38F, -1.7337162E38F, 2.2496861E38F, -2.6517273E38F, -1.8319474E38F, -3.6809475E37F, -2.7261927E38F, -3.3598448E38F, 2.4617463E37F, 2.9085052E38F, 3.3689314E38F, 9.239743E37F, 6.736276E37F, -2.9155292E38F, 2.46671E38F, -2.1858507E38F, -3.6421273E37F, -1.2404782E38F, 3.2295127E38F, -3.5699896E37F, 2.1693366E38F, 2.7511085E38F, -4.2377953E37F, -2.4217298E38F, -1.2586465E38F, 3.9122725E37F, -2.1571422E38F, 2.1301163E38F, -1.569228E38F, -7.935842E37F, -6.862774E37F, 1.8901399E38F, 9.471448E37F, -2.1452788E38F, -1.4200475E38F, -1.6170903E38F, 7.5479466E37F, -3.1555537E38F, -2.4012541E38F, -1.4899933E38F, 8.760589E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float)1.1836205E38F, PH.base.pack) ;
        p64_vz_SET((float)1.8523227E38F, PH.base.pack) ;
        p64_z_SET((float) -1.5203018E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)6392751380170744104L, PH.base.pack) ;
        p64_vx_SET((float)2.3368538E38F, PH.base.pack) ;
        p64_ay_SET((float)2.824649E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_ax_SET((float) -1.37784E38F, PH.base.pack) ;
        p64_vy_SET((float)1.784613E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan2_raw_SET((uint16_t)(uint16_t)51366, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)29561, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)21594, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)4326, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)15218, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)53605, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)5518, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)33594, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)56914, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)6206, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)3019, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3679398204L, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)56799, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)13700, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)64536, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)10063, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)34057, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)41266, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)55673, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)32143, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)10439, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)1307, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)20076, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -8488, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -8919, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -15118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan6_raw_SET((uint16_t)(uint16_t)36618, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)26881, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)34569, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)41971, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)62687, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)55654, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)42956, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)58597, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_current_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)26616, PH.base.pack) ;
        p73_param3_SET((float)1.0575775E38F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_param4_SET((float) -1.5198803E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p73_param2_SET((float)6.378401E37F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p73_param1_SET((float)1.0663975E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p73_y_SET((int32_t) -613839222, PH.base.pack) ;
        p73_z_SET((float) -1.9534958E38F, PH.base.pack) ;
        p73_x_SET((int32_t)1680223369, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float)2.6539344E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -20234, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)31696, PH.base.pack) ;
        p74_climb_SET((float)1.7025929E38F, PH.base.pack) ;
        p74_alt_SET((float) -4.001697E37F, PH.base.pack) ;
        p74_airspeed_SET((float) -3.3661403E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_param2_SET((float) -9.25285E36F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p75_param3_SET((float) -2.3211405E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p75_z_SET((float)9.447587E37F, PH.base.pack) ;
        p75_param1_SET((float)1.9112052E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p75_y_SET((int32_t) -746616668, PH.base.pack) ;
        p75_param4_SET((float)4.2364262E37F, PH.base.pack) ;
        p75_x_SET((int32_t)257455975, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param4_SET((float)3.3825897E38F, PH.base.pack) ;
        p76_param1_SET((float) -1.745496E38F, PH.base.pack) ;
        p76_param3_SET((float) -9.293185E37F, PH.base.pack) ;
        p76_param2_SET((float) -2.6910358E38F, PH.base.pack) ;
        p76_param6_SET((float)1.0089471E38F, PH.base.pack) ;
        p76_param7_SET((float)1.3793901E38F, PH.base.pack) ;
        p76_param5_SET((float)2.5295745E37F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)144, &PH) ;
        p77_result_param2_SET((int32_t)127963839, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)91, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)108, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float)7.2413506E37F, PH.base.pack) ;
        p81_roll_SET((float)7.598536E37F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)139448512L, PH.base.pack) ;
        p81_pitch_SET((float) -2.1980891E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p81_yaw_SET((float)2.4722047E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_pitch_rate_SET((float) -2.4917303E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            float q[] =  {1.7556098E37F, -2.2365922E38F, -2.9810694E38F, 3.0277007E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_thrust_SET((float) -2.2886504E37F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.6738734E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -2.0500084E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)1754171402L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_time_boot_ms_SET((uint32_t)3644901033L, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)4.1139293E37F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)2.7647622E38F, PH.base.pack) ;
        {
            float q[] =  {2.3367745E38F, 3.1722638E38F, 3.556615E37F, 5.7753157E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p83_thrust_SET((float)2.2634974E38F, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -2.974885E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vy_SET((float) -1.7486741E38F, PH.base.pack) ;
        p84_vx_SET((float) -2.2088738E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3609167808L, PH.base.pack) ;
        p84_x_SET((float)7.6423486E37F, PH.base.pack) ;
        p84_afx_SET((float)7.8366647E37F, PH.base.pack) ;
        p84_afz_SET((float)3.2391716E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)41753, PH.base.pack) ;
        p84_yaw_SET((float)2.3241972E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p84_y_SET((float)2.1332055E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.1102756E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.6580732E38F, PH.base.pack) ;
        p84_vz_SET((float)4.8457486E37F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p84_z_SET((float)3.2596749E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_time_boot_ms_SET((uint32_t)2821341128L, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)54353, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p86_vz_SET((float)2.6043793E37F, PH.base.pack) ;
        p86_afx_SET((float) -2.7682771E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p86_alt_SET((float)5.0939343E37F, PH.base.pack) ;
        p86_vx_SET((float) -1.2514833E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)994588484, PH.base.pack) ;
        p86_lon_int_SET((int32_t)910175209, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p86_afz_SET((float)2.6407282E38F, PH.base.pack) ;
        p86_yaw_SET((float) -3.9198822E37F, PH.base.pack) ;
        p86_afy_SET((float)1.846277E38F, PH.base.pack) ;
        p86_vy_SET((float) -3.3223258E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float)1.1332725E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vy_SET((float) -1.6508696E38F, PH.base.pack) ;
        p87_yaw_SET((float)2.6642274E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p87_alt_SET((float) -3.0346E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)765720318, PH.base.pack) ;
        p87_vz_SET((float) -8.826181E37F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -2137100437, PH.base.pack) ;
        p87_afy_SET((float)2.1076547E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)5.903961E37F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2491022205L, PH.base.pack) ;
        p87_afz_SET((float)1.3478934E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)19978, PH.base.pack) ;
        p87_vx_SET((float)3.0807169E38F, PH.base.pack) ;
        p87_afx_SET((float)2.8235623E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_x_SET((float)3.3168562E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2406014081L, PH.base.pack) ;
        p89_roll_SET((float)3.9994349E37F, PH.base.pack) ;
        p89_y_SET((float)2.3236226E38F, PH.base.pack) ;
        p89_yaw_SET((float)9.315444E37F, PH.base.pack) ;
        p89_z_SET((float)2.6685195E38F, PH.base.pack) ;
        p89_pitch_SET((float)1.177011E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_rollspeed_SET((float) -1.3157117E38F, PH.base.pack) ;
        p90_roll_SET((float) -8.176157E37F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)8248, PH.base.pack) ;
        p90_yaw_SET((float) -2.7251764E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -11408, PH.base.pack) ;
        p90_lat_SET((int32_t)2051616150, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)6418, PH.base.pack) ;
        p90_lon_SET((int32_t) -1207336887, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -31165, PH.base.pack) ;
        p90_alt_SET((int32_t) -2147294216, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6605171793525838041L, PH.base.pack) ;
        p90_yawspeed_SET((float)3.5662355E36F, PH.base.pack) ;
        p90_pitchspeed_SET((float)7.3471766E37F, PH.base.pack) ;
        p90_pitch_SET((float) -3.2921388E37F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)6939, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)23037, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_roll_ailerons_SET((float)4.635186E37F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.7670175E38F, PH.base.pack) ;
        p91_aux2_SET((float)1.2676066E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -2.0384596E38F, PH.base.pack) ;
        p91_aux3_SET((float) -1.415079E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p91_aux4_SET((float) -3.0279104E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)7990636338854914622L, PH.base.pack) ;
        p91_aux1_SET((float) -1.5737381E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p91_throttle_SET((float) -1.6719508E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan11_raw_SET((uint16_t)(uint16_t)6086, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)53313, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)61107, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)27392, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)60544, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)10100, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)51688, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)32719, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)23534, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)8520, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)6519904937898176219L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)372, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)57210, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)5341938393403580195L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        {
            float controls[] =  {-5.8018517E37F, 6.7757907E37F, -2.0470067E38F, -8.552335E37F, 1.7536843E38F, -2.040806E37F, 4.4147666E37F, -1.9649811E38F, 3.2685669E38F, -3.4029573E37F, 3.3270595E38F, -3.3992513E38F, 1.0989142E38F, -2.2854641E38F, 2.7626157E38F, -1.0657792E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_flags_SET((uint64_t)3722358325685448759L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_ground_distance_SET((float)2.3035599E38F, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.6119547E38F, &PH) ;
        p100_time_usec_SET((uint64_t)2001902961552274345L, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)2.5560489E38F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)7856, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -3.1668652E38F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -26775, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -7.718696E37F, &PH) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)7267746115248919677L, PH.base.pack) ;
        p101_yaw_SET((float) -1.2284003E38F, PH.base.pack) ;
        p101_x_SET((float)1.8082127E38F, PH.base.pack) ;
        p101_pitch_SET((float)2.725869E38F, PH.base.pack) ;
        p101_y_SET((float) -1.6386792E38F, PH.base.pack) ;
        p101_roll_SET((float)2.0991178E38F, PH.base.pack) ;
        p101_z_SET((float) -7.8831545E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_usec_SET((uint64_t)9056670794392650258L, PH.base.pack) ;
        p102_yaw_SET((float) -1.8064238E38F, PH.base.pack) ;
        p102_y_SET((float) -2.7219866E38F, PH.base.pack) ;
        p102_roll_SET((float)8.1237206E37F, PH.base.pack) ;
        p102_x_SET((float) -6.236664E37F, PH.base.pack) ;
        p102_pitch_SET((float) -8.4355013E37F, PH.base.pack) ;
        p102_z_SET((float) -2.346737E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_y_SET((float) -2.2335217E38F, PH.base.pack) ;
        p103_z_SET((float) -1.946152E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)644584205284985019L, PH.base.pack) ;
        p103_x_SET((float)4.1197078E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)425501550247838189L, PH.base.pack) ;
        p104_roll_SET((float) -1.4355618E38F, PH.base.pack) ;
        p104_yaw_SET((float) -1.6583686E38F, PH.base.pack) ;
        p104_y_SET((float) -1.8226862E38F, PH.base.pack) ;
        p104_z_SET((float)8.726704E36F, PH.base.pack) ;
        p104_x_SET((float) -2.703523E38F, PH.base.pack) ;
        p104_pitch_SET((float)2.5118592E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_pressure_alt_SET((float)1.7266356E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.1910222E38F, PH.base.pack) ;
        p105_zgyro_SET((float)5.1701226E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float)9.925683E37F, PH.base.pack) ;
        p105_ymag_SET((float)1.7367915E37F, PH.base.pack) ;
        p105_xgyro_SET((float)1.2799663E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4809521626914798354L, PH.base.pack) ;
        p105_yacc_SET((float) -3.292288E38F, PH.base.pack) ;
        p105_xmag_SET((float) -1.6085343E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -2.640599E38F, PH.base.pack) ;
        p105_temperature_SET((float) -2.477878E38F, PH.base.pack) ;
        p105_xacc_SET((float)7.7781073E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)13779, PH.base.pack) ;
        p105_zmag_SET((float)6.1177305E37F, PH.base.pack) ;
        p105_zacc_SET((float) -2.1256785E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_usec_SET((uint64_t)5695183757555482773L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -16220, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -9.065592E37F, PH.base.pack) ;
        p106_integrated_y_SET((float) -6.525318E37F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.1623402E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -1.5864946E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3849740708L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.9101166E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)4001788444L, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p106_distance_SET((float) -1.635358E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_temperature_SET((float)1.7574907E38F, PH.base.pack) ;
        p107_xacc_SET((float)2.212779E38F, PH.base.pack) ;
        p107_yacc_SET((float) -9.520627E37F, PH.base.pack) ;
        p107_xmag_SET((float)2.6043011E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)1.8695968E37F, PH.base.pack) ;
        p107_zgyro_SET((float)2.4483112E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)2769187365011362530L, PH.base.pack) ;
        p107_ygyro_SET((float) -1.1353716E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)258807589L, PH.base.pack) ;
        p107_diff_pressure_SET((float)1.3088171E38F, PH.base.pack) ;
        p107_zacc_SET((float) -2.3373698E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.5689538E38F, PH.base.pack) ;
        p107_ymag_SET((float) -1.9952216E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.0448261E37F, PH.base.pack) ;
        p107_zmag_SET((float)1.0690842E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_yacc_SET((float) -1.0804414E38F, PH.base.pack) ;
        p108_xacc_SET((float)2.582188E38F, PH.base.pack) ;
        p108_q1_SET((float)1.8273782E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.7935064E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -3.3212016E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -4.4152793E37F, PH.base.pack) ;
        p108_xgyro_SET((float)1.636591E38F, PH.base.pack) ;
        p108_vn_SET((float)8.1131185E37F, PH.base.pack) ;
        p108_zacc_SET((float) -3.2358495E38F, PH.base.pack) ;
        p108_alt_SET((float)2.7758258E38F, PH.base.pack) ;
        p108_q3_SET((float)9.447934E37F, PH.base.pack) ;
        p108_vd_SET((float)1.9676312E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.8213584E38F, PH.base.pack) ;
        p108_ve_SET((float) -2.402552E38F, PH.base.pack) ;
        p108_yaw_SET((float) -3.1187428E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.5539957E38F, PH.base.pack) ;
        p108_q4_SET((float)2.645436E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -1.1826189E38F, PH.base.pack) ;
        p108_roll_SET((float) -2.7444471E38F, PH.base.pack) ;
        p108_lon_SET((float)2.3065787E37F, PH.base.pack) ;
        p108_lat_SET((float) -2.3966912E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_remrssi_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)2337, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)12141, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)46, (uint8_t)49, (uint8_t)56, (uint8_t)30, (uint8_t)4, (uint8_t)155, (uint8_t)98, (uint8_t)6, (uint8_t)195, (uint8_t)10, (uint8_t)127, (uint8_t)214, (uint8_t)195, (uint8_t)29, (uint8_t)118, (uint8_t)31, (uint8_t)154, (uint8_t)170, (uint8_t)121, (uint8_t)193, (uint8_t)239, (uint8_t)95, (uint8_t)178, (uint8_t)83, (uint8_t)4, (uint8_t)135, (uint8_t)50, (uint8_t)147, (uint8_t)173, (uint8_t)6, (uint8_t)33, (uint8_t)209, (uint8_t)74, (uint8_t)191, (uint8_t)32, (uint8_t)206, (uint8_t)245, (uint8_t)119, (uint8_t)247, (uint8_t)3, (uint8_t)148, (uint8_t)181, (uint8_t)60, (uint8_t)229, (uint8_t)152, (uint8_t)193, (uint8_t)194, (uint8_t)246, (uint8_t)171, (uint8_t)180, (uint8_t)158, (uint8_t)203, (uint8_t)182, (uint8_t)103, (uint8_t)91, (uint8_t)60, (uint8_t)222, (uint8_t)176, (uint8_t)198, (uint8_t)136, (uint8_t)242, (uint8_t)88, (uint8_t)243, (uint8_t)142, (uint8_t)159, (uint8_t)111, (uint8_t)171, (uint8_t)146, (uint8_t)25, (uint8_t)104, (uint8_t)194, (uint8_t)97, (uint8_t)143, (uint8_t)31, (uint8_t)255, (uint8_t)118, (uint8_t)159, (uint8_t)36, (uint8_t)215, (uint8_t)74, (uint8_t)208, (uint8_t)85, (uint8_t)145, (uint8_t)116, (uint8_t)119, (uint8_t)164, (uint8_t)201, (uint8_t)118, (uint8_t)132, (uint8_t)172, (uint8_t)163, (uint8_t)200, (uint8_t)165, (uint8_t)132, (uint8_t)57, (uint8_t)159, (uint8_t)32, (uint8_t)89, (uint8_t)177, (uint8_t)241, (uint8_t)31, (uint8_t)74, (uint8_t)229, (uint8_t)109, (uint8_t)95, (uint8_t)172, (uint8_t)118, (uint8_t)44, (uint8_t)23, (uint8_t)230, (uint8_t)178, (uint8_t)184, (uint8_t)236, (uint8_t)64, (uint8_t)211, (uint8_t)199, (uint8_t)178, (uint8_t)128, (uint8_t)119, (uint8_t)168, (uint8_t)200, (uint8_t)148, (uint8_t)230, (uint8_t)60, (uint8_t)51, (uint8_t)72, (uint8_t)103, (uint8_t)159, (uint8_t)96, (uint8_t)80, (uint8_t)113, (uint8_t)71, (uint8_t)88, (uint8_t)194, (uint8_t)123, (uint8_t)53, (uint8_t)116, (uint8_t)199, (uint8_t)197, (uint8_t)61, (uint8_t)165, (uint8_t)87, (uint8_t)101, (uint8_t)136, (uint8_t)164, (uint8_t)251, (uint8_t)87, (uint8_t)100, (uint8_t)24, (uint8_t)179, (uint8_t)118, (uint8_t)32, (uint8_t)77, (uint8_t)115, (uint8_t)79, (uint8_t)95, (uint8_t)141, (uint8_t)147, (uint8_t)83, (uint8_t)215, (uint8_t)104, (uint8_t)238, (uint8_t)151, (uint8_t)142, (uint8_t)191, (uint8_t)161, (uint8_t)93, (uint8_t)124, (uint8_t)19, (uint8_t)84, (uint8_t)244, (uint8_t)116, (uint8_t)15, (uint8_t)162, (uint8_t)199, (uint8_t)241, (uint8_t)139, (uint8_t)52, (uint8_t)145, (uint8_t)237, (uint8_t)138, (uint8_t)192, (uint8_t)178, (uint8_t)193, (uint8_t)8, (uint8_t)120, (uint8_t)86, (uint8_t)8, (uint8_t)122, (uint8_t)52, (uint8_t)110, (uint8_t)10, (uint8_t)50, (uint8_t)151, (uint8_t)192, (uint8_t)3, (uint8_t)130, (uint8_t)237, (uint8_t)244, (uint8_t)97, (uint8_t)166, (uint8_t)211, (uint8_t)122, (uint8_t)196, (uint8_t)178, (uint8_t)246, (uint8_t)75, (uint8_t)109, (uint8_t)150, (uint8_t)243, (uint8_t)241, (uint8_t)10, (uint8_t)192, (uint8_t)51, (uint8_t)118, (uint8_t)171, (uint8_t)167, (uint8_t)208, (uint8_t)200, (uint8_t)47, (uint8_t)116, (uint8_t)30, (uint8_t)59, (uint8_t)240, (uint8_t)120, (uint8_t)212, (uint8_t)197, (uint8_t)249, (uint8_t)14, (uint8_t)247, (uint8_t)218, (uint8_t)144, (uint8_t)50, (uint8_t)196, (uint8_t)115, (uint8_t)55, (uint8_t)89, (uint8_t)211, (uint8_t)116, (uint8_t)201, (uint8_t)127, (uint8_t)153, (uint8_t)228, (uint8_t)114, (uint8_t)123, (uint8_t)112, (uint8_t)100, (uint8_t)12, (uint8_t)75, (uint8_t)22, (uint8_t)255};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)1952061322550441358L, PH.base.pack) ;
        p111_tc1_SET((int64_t)984070289590459344L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)3452781018247207385L, PH.base.pack) ;
        p112_seq_SET((uint32_t)1753248257L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_lon_SET((int32_t)1127925067, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p113_lat_SET((int32_t)757436846, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)23776, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)49005, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)49520, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -1495, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -5803, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)5659, PH.base.pack) ;
        p113_alt_SET((int32_t) -1921574456, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)794400969753929107L, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)37615, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_xgyro_SET((float) -2.1636468E37F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.0557119E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p114_integrated_x_SET((float)3.0022706E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -3.2740553E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3515813319L, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)2588, PH.base.pack) ;
        p114_distance_SET((float) -2.3556648E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)6539942226715359330L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1524641126L, PH.base.pack) ;
        p114_integrated_y_SET((float)2.3279194E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        {
            float attitude_quaternion[] =  {1.6246718E38F, -9.145671E37F, 3.439822E37F, 3.2895733E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_true_airspeed_SET((uint16_t)(uint16_t)58304, PH.base.pack) ;
        p115_yawspeed_SET((float)3.356234E38F, PH.base.pack) ;
        p115_pitchspeed_SET((float) -3.1884869E38F, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)24512, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -10068, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)17337, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)1013835421286389586L, PH.base.pack) ;
        p115_lat_SET((int32_t)781813158, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)13747, PH.base.pack) ;
        p115_alt_SET((int32_t)30453367, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)15732, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -16983, PH.base.pack) ;
        p115_lon_SET((int32_t)1687209570, PH.base.pack) ;
        p115_rollspeed_SET((float) -7.0586705E37F, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)38268, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_time_boot_ms_SET((uint32_t)3288106365L, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -18258, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)8622, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -14830, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)12979, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)27634, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)20322, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -14649, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)17169, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -13732, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)54121, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)36426, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)4142, PH.base.pack) ;
        p118_size_SET((uint32_t)4144396705L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)38002, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)958348128L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)13527, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)31087, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1900353624L, PH.base.pack) ;
        p119_count_SET((uint32_t)3339152813L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)3276131960L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)47006, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)215, (uint8_t)123, (uint8_t)146, (uint8_t)188, (uint8_t)18, (uint8_t)127, (uint8_t)52, (uint8_t)177, (uint8_t)127, (uint8_t)240, (uint8_t)210, (uint8_t)40, (uint8_t)223, (uint8_t)62, (uint8_t)63, (uint8_t)248, (uint8_t)113, (uint8_t)22, (uint8_t)183, (uint8_t)211, (uint8_t)76, (uint8_t)222, (uint8_t)30, (uint8_t)91, (uint8_t)103, (uint8_t)186, (uint8_t)103, (uint8_t)67, (uint8_t)225, (uint8_t)202, (uint8_t)207, (uint8_t)161, (uint8_t)40, (uint8_t)158, (uint8_t)251, (uint8_t)35, (uint8_t)27, (uint8_t)159, (uint8_t)117, (uint8_t)178, (uint8_t)175, (uint8_t)8, (uint8_t)209, (uint8_t)104, (uint8_t)23, (uint8_t)176, (uint8_t)142, (uint8_t)240, (uint8_t)147, (uint8_t)254, (uint8_t)73, (uint8_t)17, (uint8_t)140, (uint8_t)55, (uint8_t)92, (uint8_t)242, (uint8_t)47, (uint8_t)53, (uint8_t)56, (uint8_t)214, (uint8_t)124, (uint8_t)213, (uint8_t)242, (uint8_t)126, (uint8_t)226, (uint8_t)222, (uint8_t)240, (uint8_t)236, (uint8_t)240, (uint8_t)198, (uint8_t)36, (uint8_t)58, (uint8_t)180, (uint8_t)241, (uint8_t)117, (uint8_t)46, (uint8_t)12, (uint8_t)102, (uint8_t)246, (uint8_t)123, (uint8_t)158, (uint8_t)150, (uint8_t)26, (uint8_t)32, (uint8_t)23, (uint8_t)175, (uint8_t)67, (uint8_t)189, (uint8_t)90, (uint8_t)247};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)88, (uint8_t)26, (uint8_t)138, (uint8_t)22, (uint8_t)137, (uint8_t)26, (uint8_t)42, (uint8_t)119, (uint8_t)97, (uint8_t)86, (uint8_t)102, (uint8_t)86, (uint8_t)73, (uint8_t)9, (uint8_t)6, (uint8_t)189, (uint8_t)52, (uint8_t)178, (uint8_t)70, (uint8_t)237, (uint8_t)38, (uint8_t)198, (uint8_t)110, (uint8_t)78, (uint8_t)224, (uint8_t)67, (uint8_t)253, (uint8_t)72, (uint8_t)216, (uint8_t)166, (uint8_t)147, (uint8_t)14, (uint8_t)187, (uint8_t)132, (uint8_t)177, (uint8_t)128, (uint8_t)211, (uint8_t)35, (uint8_t)184, (uint8_t)9, (uint8_t)91, (uint8_t)30, (uint8_t)97, (uint8_t)254, (uint8_t)190, (uint8_t)99, (uint8_t)198, (uint8_t)171, (uint8_t)17, (uint8_t)63, (uint8_t)206, (uint8_t)239, (uint8_t)112, (uint8_t)156, (uint8_t)116, (uint8_t)223, (uint8_t)2, (uint8_t)11, (uint8_t)190, (uint8_t)6, (uint8_t)104, (uint8_t)71, (uint8_t)85, (uint8_t)176, (uint8_t)191, (uint8_t)164, (uint8_t)206, (uint8_t)175, (uint8_t)33, (uint8_t)41, (uint8_t)202, (uint8_t)219, (uint8_t)122, (uint8_t)213, (uint8_t)72, (uint8_t)130, (uint8_t)81, (uint8_t)223, (uint8_t)212, (uint8_t)10, (uint8_t)55, (uint8_t)79, (uint8_t)17, (uint8_t)70, (uint8_t)143, (uint8_t)202, (uint8_t)128, (uint8_t)176, (uint8_t)109, (uint8_t)98, (uint8_t)52, (uint8_t)109, (uint8_t)33, (uint8_t)208, (uint8_t)204, (uint8_t)33, (uint8_t)41, (uint8_t)95, (uint8_t)154, (uint8_t)207, (uint8_t)32, (uint8_t)115, (uint8_t)218, (uint8_t)129, (uint8_t)235, (uint8_t)235, (uint8_t)66, (uint8_t)181, (uint8_t)117, (uint8_t)127};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_dgps_age_SET((uint32_t)3142296150L, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)14433, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)40086, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)63672, PH.base.pack) ;
        p124_alt_SET((int32_t) -973263044, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p124_lon_SET((int32_t)719808003, PH.base.pack) ;
        p124_lat_SET((int32_t) -701916125, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)5704, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)8271421492108452511L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)15023, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)55795, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)31567, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)205, (uint8_t)140, (uint8_t)73, (uint8_t)66, (uint8_t)157, (uint8_t)63, (uint8_t)148, (uint8_t)252, (uint8_t)161, (uint8_t)62, (uint8_t)43, (uint8_t)25, (uint8_t)225, (uint8_t)174, (uint8_t)134, (uint8_t)157, (uint8_t)171, (uint8_t)240, (uint8_t)107, (uint8_t)242, (uint8_t)67, (uint8_t)100, (uint8_t)88, (uint8_t)191, (uint8_t)191, (uint8_t)150, (uint8_t)81, (uint8_t)184, (uint8_t)152, (uint8_t)94, (uint8_t)119, (uint8_t)120, (uint8_t)169, (uint8_t)252, (uint8_t)108, (uint8_t)138, (uint8_t)230, (uint8_t)55, (uint8_t)141, (uint8_t)252, (uint8_t)15, (uint8_t)192, (uint8_t)15, (uint8_t)143, (uint8_t)189, (uint8_t)207, (uint8_t)170, (uint8_t)151, (uint8_t)156, (uint8_t)99, (uint8_t)47, (uint8_t)11, (uint8_t)81, (uint8_t)193, (uint8_t)229, (uint8_t)168, (uint8_t)144, (uint8_t)137, (uint8_t)137, (uint8_t)107, (uint8_t)161, (uint8_t)58, (uint8_t)230, (uint8_t)217, (uint8_t)221, (uint8_t)179, (uint8_t)118, (uint8_t)112, (uint8_t)131, (uint8_t)171};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_baudrate_SET((uint32_t)2060999350L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_nsats_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)346431268L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -788715736, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2646524226L, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -632227583, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)1391241703, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -2014927820, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p127_tow_SET((uint32_t)3738954107L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)39132, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_wn_SET((uint16_t)(uint16_t)3532, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)337913507, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4040283679L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -538954998, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -1707436860, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p128_tow_SET((uint32_t)3249469675L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2404709418L, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -676710899, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_zmag_SET((int16_t)(int16_t) -28043, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)4194656063L, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)136, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -7657, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)21092, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)12728, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)3978, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)1860, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)1801, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)18031, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_height_SET((uint16_t)(uint16_t)804, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)17385, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p130_size_SET((uint32_t)1035263706L, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)46626, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)246, (uint8_t)237, (uint8_t)75, (uint8_t)106, (uint8_t)105, (uint8_t)42, (uint8_t)73, (uint8_t)177, (uint8_t)25, (uint8_t)24, (uint8_t)113, (uint8_t)114, (uint8_t)30, (uint8_t)154, (uint8_t)176, (uint8_t)75, (uint8_t)67, (uint8_t)98, (uint8_t)190, (uint8_t)115, (uint8_t)130, (uint8_t)57, (uint8_t)143, (uint8_t)140, (uint8_t)156, (uint8_t)69, (uint8_t)229, (uint8_t)142, (uint8_t)223, (uint8_t)16, (uint8_t)252, (uint8_t)96, (uint8_t)140, (uint8_t)107, (uint8_t)128, (uint8_t)220, (uint8_t)131, (uint8_t)25, (uint8_t)66, (uint8_t)232, (uint8_t)1, (uint8_t)190, (uint8_t)152, (uint8_t)154, (uint8_t)157, (uint8_t)23, (uint8_t)74, (uint8_t)106, (uint8_t)29, (uint8_t)10, (uint8_t)255, (uint8_t)180, (uint8_t)147, (uint8_t)101, (uint8_t)16, (uint8_t)2, (uint8_t)73, (uint8_t)64, (uint8_t)88, (uint8_t)159, (uint8_t)175, (uint8_t)222, (uint8_t)75, (uint8_t)18, (uint8_t)23, (uint8_t)42, (uint8_t)206, (uint8_t)157, (uint8_t)178, (uint8_t)146, (uint8_t)96, (uint8_t)19, (uint8_t)160, (uint8_t)116, (uint8_t)33, (uint8_t)20, (uint8_t)110, (uint8_t)49, (uint8_t)121, (uint8_t)61, (uint8_t)250, (uint8_t)239, (uint8_t)30, (uint8_t)142, (uint8_t)138, (uint8_t)62, (uint8_t)76, (uint8_t)228, (uint8_t)181, (uint8_t)94, (uint8_t)2, (uint8_t)190, (uint8_t)126, (uint8_t)140, (uint8_t)55, (uint8_t)245, (uint8_t)98, (uint8_t)68, (uint8_t)99, (uint8_t)168, (uint8_t)250, (uint8_t)202, (uint8_t)96, (uint8_t)129, (uint8_t)192, (uint8_t)171, (uint8_t)11, (uint8_t)119, (uint8_t)203, (uint8_t)124, (uint8_t)84, (uint8_t)20, (uint8_t)37, (uint8_t)242, (uint8_t)222, (uint8_t)48, (uint8_t)153, (uint8_t)6, (uint8_t)154, (uint8_t)83, (uint8_t)65, (uint8_t)69, (uint8_t)26, (uint8_t)222, (uint8_t)37, (uint8_t)97, (uint8_t)193, (uint8_t)145, (uint8_t)14, (uint8_t)195, (uint8_t)226, (uint8_t)64, (uint8_t)69, (uint8_t)160, (uint8_t)27, (uint8_t)182, (uint8_t)107, (uint8_t)154, (uint8_t)218, (uint8_t)110, (uint8_t)60, (uint8_t)60, (uint8_t)136, (uint8_t)221, (uint8_t)177, (uint8_t)148, (uint8_t)188, (uint8_t)22, (uint8_t)97, (uint8_t)38, (uint8_t)208, (uint8_t)41, (uint8_t)144, (uint8_t)210, (uint8_t)176, (uint8_t)48, (uint8_t)31, (uint8_t)223, (uint8_t)80, (uint8_t)38, (uint8_t)66, (uint8_t)218, (uint8_t)111, (uint8_t)119, (uint8_t)241, (uint8_t)224, (uint8_t)178, (uint8_t)193, (uint8_t)63, (uint8_t)172, (uint8_t)76, (uint8_t)148, (uint8_t)49, (uint8_t)115, (uint8_t)86, (uint8_t)197, (uint8_t)13, (uint8_t)55, (uint8_t)91, (uint8_t)44, (uint8_t)225, (uint8_t)131, (uint8_t)82, (uint8_t)204, (uint8_t)156, (uint8_t)212, (uint8_t)95, (uint8_t)197, (uint8_t)182, (uint8_t)171, (uint8_t)6, (uint8_t)125, (uint8_t)165, (uint8_t)44, (uint8_t)207, (uint8_t)134, (uint8_t)87, (uint8_t)60, (uint8_t)132, (uint8_t)200, (uint8_t)195, (uint8_t)166, (uint8_t)59, (uint8_t)1, (uint8_t)35, (uint8_t)91, (uint8_t)167, (uint8_t)222, (uint8_t)68, (uint8_t)6, (uint8_t)55, (uint8_t)209, (uint8_t)26, (uint8_t)16, (uint8_t)167, (uint8_t)231, (uint8_t)164, (uint8_t)252, (uint8_t)197, (uint8_t)21, (uint8_t)194, (uint8_t)57, (uint8_t)54, (uint8_t)250, (uint8_t)68, (uint8_t)211, (uint8_t)103, (uint8_t)166, (uint8_t)193, (uint8_t)63, (uint8_t)145, (uint8_t)194, (uint8_t)41, (uint8_t)189, (uint8_t)8, (uint8_t)119, (uint8_t)71, (uint8_t)62, (uint8_t)140, (uint8_t)194, (uint8_t)96, (uint8_t)119, (uint8_t)20, (uint8_t)89, (uint8_t)33, (uint8_t)74, (uint8_t)44, (uint8_t)250, (uint8_t)178, (uint8_t)104, (uint8_t)138, (uint8_t)8, (uint8_t)33};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)26813, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_time_boot_ms_SET((uint32_t)4127518180L, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)63288, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)57434, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_225, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)63277, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)2539027389232551789L, PH.base.pack) ;
        p133_lon_SET((int32_t)1353225415, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)16561, PH.base.pack) ;
        p133_lat_SET((int32_t) -1240274137, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)29290, PH.base.pack) ;
        p134_lat_SET((int32_t) -1573945037, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -7152, (int16_t)31459, (int16_t) -30229, (int16_t)24298, (int16_t) -19626, (int16_t) -20170, (int16_t) -30970, (int16_t) -19882, (int16_t) -17857, (int16_t)12823, (int16_t) -15123, (int16_t)17850, (int16_t) -22127, (int16_t)31021, (int16_t)824, (int16_t) -24386};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lon_SET((int32_t) -1151468842, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -1268481740, PH.base.pack) ;
        p135_lon_SET((int32_t) -1976495672, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_terrain_height_SET((float) -2.5712383E37F, PH.base.pack) ;
        p136_lon_SET((int32_t) -1844185232, PH.base.pack) ;
        p136_lat_SET((int32_t) -1561717303, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)62352, PH.base.pack) ;
        p136_current_height_SET((float)2.120251E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)54033, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)11965, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float)1.3462372E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)8247, PH.base.pack) ;
        p137_press_abs_SET((float)2.6145425E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3459624371L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {-2.614686E38F, -2.6357888E38F, 2.6234964E38F, -2.8229224E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -1.1645493E38F, PH.base.pack) ;
        p138_y_SET((float)3.2069408E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)4492177325691604603L, PH.base.pack) ;
        p138_z_SET((float)1.7400366E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)7348825432026364559L, PH.base.pack) ;
        {
            float controls[] =  {-1.8311521E38F, 3.3853751E38F, -5.014698E37F, -3.1049724E38F, 2.12802E38F, -6.185365E37F, 2.7076203E38F, 2.4331797E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)2463457250857692133L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            float controls[] =  {-2.5450512E38F, 4.2444157E37F, 3.1426886E38F, -2.2601673E38F, -9.625697E37F, 1.0860842E38F, 2.1582467E38F, -6.143432E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_amsl_SET((float) -2.76371E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.8233982E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float)1.2489406E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -3.1094492E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.1744562E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)1007474590083344186L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)4.7013947E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t uri[] =  {(uint8_t)12, (uint8_t)58, (uint8_t)157, (uint8_t)75, (uint8_t)133, (uint8_t)244, (uint8_t)101, (uint8_t)114, (uint8_t)90, (uint8_t)178, (uint8_t)153, (uint8_t)177, (uint8_t)188, (uint8_t)197, (uint8_t)243, (uint8_t)81, (uint8_t)131, (uint8_t)238, (uint8_t)168, (uint8_t)6, (uint8_t)146, (uint8_t)20, (uint8_t)72, (uint8_t)215, (uint8_t)79, (uint8_t)9, (uint8_t)77, (uint8_t)201, (uint8_t)103, (uint8_t)198, (uint8_t)158, (uint8_t)23, (uint8_t)195, (uint8_t)242, (uint8_t)34, (uint8_t)17, (uint8_t)163, (uint8_t)21, (uint8_t)20, (uint8_t)204, (uint8_t)144, (uint8_t)143, (uint8_t)75, (uint8_t)217, (uint8_t)10, (uint8_t)16, (uint8_t)71, (uint8_t)27, (uint8_t)111, (uint8_t)234, (uint8_t)253, (uint8_t)137, (uint8_t)250, (uint8_t)225, (uint8_t)148, (uint8_t)142, (uint8_t)79, (uint8_t)149, (uint8_t)232, (uint8_t)20, (uint8_t)171, (uint8_t)102, (uint8_t)129, (uint8_t)71, (uint8_t)23, (uint8_t)46, (uint8_t)90, (uint8_t)116, (uint8_t)11, (uint8_t)91, (uint8_t)235, (uint8_t)144, (uint8_t)240, (uint8_t)93, (uint8_t)103, (uint8_t)40, (uint8_t)90, (uint8_t)120, (uint8_t)3, (uint8_t)56, (uint8_t)242, (uint8_t)25, (uint8_t)48, (uint8_t)45, (uint8_t)151, (uint8_t)50, (uint8_t)130, (uint8_t)70, (uint8_t)29, (uint8_t)135, (uint8_t)51, (uint8_t)121, (uint8_t)30, (uint8_t)129, (uint8_t)218, (uint8_t)148, (uint8_t)29, (uint8_t)248, (uint8_t)89, (uint8_t)165, (uint8_t)229, (uint8_t)107, (uint8_t)134, (uint8_t)184, (uint8_t)142, (uint8_t)36, (uint8_t)209, (uint8_t)178, (uint8_t)170, (uint8_t)240, (uint8_t)74, (uint8_t)54, (uint8_t)224, (uint8_t)109, (uint8_t)169, (uint8_t)94, (uint8_t)4, (uint8_t)232, (uint8_t)238, (uint8_t)207};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        {
            uint8_t storage[] =  {(uint8_t)171, (uint8_t)124, (uint8_t)56, (uint8_t)65, (uint8_t)162, (uint8_t)92, (uint8_t)0, (uint8_t)225, (uint8_t)185, (uint8_t)215, (uint8_t)58, (uint8_t)93, (uint8_t)89, (uint8_t)101, (uint8_t)226, (uint8_t)33, (uint8_t)8, (uint8_t)59, (uint8_t)221, (uint8_t)244, (uint8_t)153, (uint8_t)71, (uint8_t)30, (uint8_t)182, (uint8_t)54, (uint8_t)70, (uint8_t)124, (uint8_t)45, (uint8_t)50, (uint8_t)13, (uint8_t)129, (uint8_t)180, (uint8_t)25, (uint8_t)177, (uint8_t)112, (uint8_t)89, (uint8_t)218, (uint8_t)116, (uint8_t)149, (uint8_t)54, (uint8_t)248, (uint8_t)130, (uint8_t)220, (uint8_t)182, (uint8_t)103, (uint8_t)89, (uint8_t)63, (uint8_t)178, (uint8_t)219, (uint8_t)88, (uint8_t)89, (uint8_t)157, (uint8_t)74, (uint8_t)60, (uint8_t)121, (uint8_t)124, (uint8_t)27, (uint8_t)212, (uint8_t)40, (uint8_t)255, (uint8_t)118, (uint8_t)246, (uint8_t)245, (uint8_t)124, (uint8_t)83, (uint8_t)146, (uint8_t)207, (uint8_t)177, (uint8_t)155, (uint8_t)84, (uint8_t)94, (uint8_t)176, (uint8_t)54, (uint8_t)146, (uint8_t)13, (uint8_t)11, (uint8_t)223, (uint8_t)7, (uint8_t)148, (uint8_t)177, (uint8_t)130, (uint8_t)207, (uint8_t)209, (uint8_t)102, (uint8_t)47, (uint8_t)107, (uint8_t)227, (uint8_t)205, (uint8_t)101, (uint8_t)1, (uint8_t)94, (uint8_t)13, (uint8_t)236, (uint8_t)98, (uint8_t)84, (uint8_t)73, (uint8_t)161, (uint8_t)86, (uint8_t)186, (uint8_t)240, (uint8_t)157, (uint8_t)18, (uint8_t)86, (uint8_t)21, (uint8_t)84, (uint8_t)47, (uint8_t)163, (uint8_t)1, (uint8_t)64, (uint8_t)82, (uint8_t)78, (uint8_t)5, (uint8_t)161, (uint8_t)44, (uint8_t)137, (uint8_t)249, (uint8_t)22, (uint8_t)236, (uint8_t)58, (uint8_t)72};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)2.5559578E38F, PH.base.pack) ;
        p143_press_diff_SET((float)1.9553263E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)18118, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)2103703163L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_lat_SET((int32_t) -336251820, PH.base.pack) ;
        {
            float vel[] =  {-2.645347E38F, 2.884403E38F, -6.575374E36F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        {
            float position_cov[] =  {-8.1476934E37F, -2.2586126E38F, 4.7315236E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -5.4936884E37F, PH.base.pack) ;
        {
            float rates[] =  {2.5700304E38F, 1.6943772E38F, 4.977151E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)44688403, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)5401330020222628162L, PH.base.pack) ;
        {
            float attitude_q[] =  {2.9928008E38F, -3.4233067E37F, 9.03231E37F, -2.3800045E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)7798861023712896359L, PH.base.pack) ;
        {
            float acc[] =  {-3.604132E37F, 1.2674056E38F, 3.2318261E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_acc_SET((float) -1.8054283E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)1.6525003E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -3.0007509E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)4299980209147537323L, PH.base.pack) ;
        p146_y_pos_SET((float) -2.0396626E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {-8.701573E37F, 3.0642648E38F, 2.57798E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_z_pos_SET((float) -1.4343964E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-3.0934881E38F, -2.1641177E38F, 1.139701E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_x_vel_SET((float) -2.7880182E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.7742888E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -4.268993E37F, PH.base.pack) ;
        p146_z_acc_SET((float)2.4275101E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -9.710111E37F, PH.base.pack) ;
        p146_y_vel_SET((float)2.5466052E38F, PH.base.pack) ;
        p146_x_pos_SET((float)2.1059666E38F, PH.base.pack) ;
        {
            float q[] =  {-3.1731528E38F, 2.7952562E38F, -1.9558847E38F, -1.5322047E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float) -2.1923368E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t) -175299373, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)64, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -1997240039, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)13037, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -28115, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)33210, (uint16_t)29978, (uint16_t)44123, (uint16_t)52096, (uint16_t)11444, (uint16_t)53248, (uint16_t)34798, (uint16_t)14272, (uint16_t)40369, (uint16_t)28660};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_os_sw_version_SET((uint32_t)112435539L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3542781889L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)20029, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)162, (uint8_t)236, (uint8_t)237, (uint8_t)106, (uint8_t)21, (uint8_t)93, (uint8_t)187, (uint8_t)11};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)57, (uint8_t)50, (uint8_t)249, (uint8_t)208, (uint8_t)133, (uint8_t)213, (uint8_t)68, (uint8_t)198};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)2033622656L, PH.base.pack) ;
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)130, (uint8_t)203, (uint8_t)148, (uint8_t)66, (uint8_t)243, (uint8_t)178, (uint8_t)65, (uint8_t)54};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_middleware_sw_version_SET((uint32_t)1345676899L, PH.base.pack) ;
        p148_uid_SET((uint64_t)8917899589896546499L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)91, (uint8_t)7, (uint8_t)213, (uint8_t)141, (uint8_t)160, (uint8_t)181, (uint8_t)223, (uint8_t)214, (uint8_t)98, (uint8_t)167, (uint8_t)3, (uint8_t)247, (uint8_t)41, (uint8_t)208, (uint8_t)47, (uint8_t)96, (uint8_t)249, (uint8_t)77};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)37612, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_size_x_SET((float) -1.8422515E38F, PH.base.pack) ;
        p149_distance_SET((float)9.652033E37F, PH.base.pack) ;
        p149_angle_x_SET((float) -1.6949234E38F, PH.base.pack) ;
        {
            float q[] =  {1.6526461E38F, -4.1241215E37F, -8.176324E36F, -2.7780774E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_x_SET((float)1.4663395E38F, &PH) ;
        p149_time_usec_SET((uint64_t)7101268281404681725L, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)135, &PH) ;
        p149_angle_y_SET((float) -1.8193962E38F, PH.base.pack) ;
        p149_z_SET((float) -1.7675467E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p149_size_y_SET((float)2.3309452E38F, PH.base.pack) ;
        p149_y_SET((float)3.5069947E36F, &PH) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_mag_ratio_SET((float)3.0555533E38F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -1.3658147E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.2530092E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -1.5432947E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.6913839E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -1.991166E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)4124172350540905314L, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -2.849157E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -1.0746971E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_vert_accuracy_SET((float) -1.8510583E38F, PH.base.pack) ;
        p231_wind_z_SET((float)2.223529E38F, PH.base.pack) ;
        p231_wind_x_SET((float)9.929221E37F, PH.base.pack) ;
        p231_var_horiz_SET((float) -3.1989964E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -1.8538331E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -2.1744822E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)4.152779E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)5851683437562058257L, PH.base.pack) ;
        p231_wind_alt_SET((float)2.374925E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_alt_SET((float)2.9555517E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)3244619747L, PH.base.pack) ;
        p232_vd_SET((float) -8.76137E37F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)4954183652961731119L, PH.base.pack) ;
        p232_vn_SET((float) -5.972222E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)32191, PH.base.pack) ;
        p232_vdop_SET((float) -7.4021805E37F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -6.015086E37F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
        p232_speed_accuracy_SET((float)3.30279E37F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -1.7444286E38F, PH.base.pack) ;
        p232_ve_SET((float)2.5670925E38F, PH.base.pack) ;
        p232_lon_SET((int32_t)1181191409, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p232_lat_SET((int32_t)626231187, PH.base.pack) ;
        p232_hdop_SET((float)7.746753E37F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)174, (uint8_t)185, (uint8_t)45, (uint8_t)221, (uint8_t)178, (uint8_t)16, (uint8_t)132, (uint8_t)79, (uint8_t)73, (uint8_t)158, (uint8_t)51, (uint8_t)168, (uint8_t)45, (uint8_t)38, (uint8_t)30, (uint8_t)219, (uint8_t)150, (uint8_t)129, (uint8_t)226, (uint8_t)156, (uint8_t)140, (uint8_t)155, (uint8_t)106, (uint8_t)33, (uint8_t)5, (uint8_t)214, (uint8_t)73, (uint8_t)225, (uint8_t)116, (uint8_t)108, (uint8_t)227, (uint8_t)60, (uint8_t)108, (uint8_t)47, (uint8_t)205, (uint8_t)248, (uint8_t)98, (uint8_t)233, (uint8_t)83, (uint8_t)137, (uint8_t)145, (uint8_t)243, (uint8_t)56, (uint8_t)190, (uint8_t)114, (uint8_t)140, (uint8_t)201, (uint8_t)56, (uint8_t)196, (uint8_t)56, (uint8_t)210, (uint8_t)42, (uint8_t)16, (uint8_t)34, (uint8_t)95, (uint8_t)200, (uint8_t)185, (uint8_t)200, (uint8_t)204, (uint8_t)58, (uint8_t)222, (uint8_t)118, (uint8_t)78, (uint8_t)89, (uint8_t)125, (uint8_t)106, (uint8_t)173, (uint8_t)169, (uint8_t)70, (uint8_t)216, (uint8_t)57, (uint8_t)130, (uint8_t)76, (uint8_t)235, (uint8_t)64, (uint8_t)122, (uint8_t)37, (uint8_t)50, (uint8_t)217, (uint8_t)204, (uint8_t)237, (uint8_t)51, (uint8_t)17, (uint8_t)53, (uint8_t)218, (uint8_t)208, (uint8_t)48, (uint8_t)149, (uint8_t)191, (uint8_t)98, (uint8_t)254, (uint8_t)236, (uint8_t)139, (uint8_t)88, (uint8_t)159, (uint8_t)171, (uint8_t)21, (uint8_t)66, (uint8_t)30, (uint8_t)61, (uint8_t)197, (uint8_t)61, (uint8_t)37, (uint8_t)95, (uint8_t)19, (uint8_t)50, (uint8_t)42, (uint8_t)160, (uint8_t)174, (uint8_t)190, (uint8_t)36, (uint8_t)145, (uint8_t)71, (uint8_t)55, (uint8_t)92, (uint8_t)143, (uint8_t)124, (uint8_t)104, (uint8_t)89, (uint8_t)47, (uint8_t)199, (uint8_t)186, (uint8_t)223, (uint8_t)217, (uint8_t)50, (uint8_t)146, (uint8_t)106, (uint8_t)251, (uint8_t)219, (uint8_t)107, (uint8_t)112, (uint8_t)189, (uint8_t)171, (uint8_t)30, (uint8_t)96, (uint8_t)250, (uint8_t)31, (uint8_t)9, (uint8_t)27, (uint8_t)8, (uint8_t)79, (uint8_t)253, (uint8_t)31, (uint8_t)196, (uint8_t)100, (uint8_t)164, (uint8_t)1, (uint8_t)89, (uint8_t)51, (uint8_t)159, (uint8_t)147, (uint8_t)75, (uint8_t)46, (uint8_t)9, (uint8_t)208, (uint8_t)130, (uint8_t)43, (uint8_t)103, (uint8_t)1, (uint8_t)130, (uint8_t)67, (uint8_t)50, (uint8_t)82, (uint8_t)10, (uint8_t)107, (uint8_t)73, (uint8_t)228, (uint8_t)245, (uint8_t)19, (uint8_t)141, (uint8_t)166, (uint8_t)210, (uint8_t)153, (uint8_t)137, (uint8_t)101, (uint8_t)138, (uint8_t)9, (uint8_t)193, (uint8_t)245, (uint8_t)27};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_wp_num_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -25710, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)45256, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p234_latitude_SET((int32_t) -1734152714, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -10792, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -7553, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)78, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)42820, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)20217, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -16, PH.base.pack) ;
        p234_longitude_SET((int32_t) -380281959, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -7522, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)45, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -35, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1720033366L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_vibration_z_SET((float) -1.5970138E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)214989208L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)799830589L, PH.base.pack) ;
        p241_vibration_x_SET((float)1.3767085E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6560928051464057959L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1992743661L, PH.base.pack) ;
        p241_vibration_y_SET((float) -2.866285E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_longitude_SET((int32_t)988792597, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1154741668, PH.base.pack) ;
        p242_y_SET((float)2.0701268E38F, PH.base.pack) ;
        p242_z_SET((float) -1.0028086E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)575141900146189234L, &PH) ;
        p242_approach_z_SET((float)2.9670378E38F, PH.base.pack) ;
        p242_approach_x_SET((float)2.1065004E38F, PH.base.pack) ;
        {
            float q[] =  {1.3792931E38F, 3.3747617E38F, -1.3354088E38F, -1.9748911E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float)6.2852023E37F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1298436124, PH.base.pack) ;
        p242_x_SET((float) -2.5291796E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_z_SET((float)4.0756807E37F, PH.base.pack) ;
        p243_z_SET((float) -1.9945505E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)1821608584, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)8717836766971549615L, &PH) ;
        p243_approach_x_SET((float) -1.0705927E38F, PH.base.pack) ;
        {
            float q[] =  {5.222147E37F, -4.6233885E37F, -8.85017E37F, -1.2642371E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_y_SET((float)1.4497441E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t)1298592727, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1355640098, PH.base.pack) ;
        p243_y_SET((float)1.939387E38F, PH.base.pack) ;
        p243_x_SET((float) -2.9684404E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)39067, PH.base.pack) ;
        p244_interval_us_SET((int32_t)950098489, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, PH.base.pack) ;
        p246_lat_SET((int32_t)237967964, PH.base.pack) ;
        {
            char16_t* callsign = u"iRnnmteum";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_squawk_SET((uint16_t)(uint16_t)54717, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)45390, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)665356756L, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p246_altitude_SET((int32_t) -138872184, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -1387, PH.base.pack) ;
        p246_lon_SET((int32_t)491717234, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)34182, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)2.7013074E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -2.2626225E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_id_SET((uint32_t)2395526728L, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.7566149E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_message_type_SET((uint16_t)(uint16_t)30348, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)56, (uint8_t)61, (uint8_t)142, (uint8_t)34, (uint8_t)251, (uint8_t)178, (uint8_t)177, (uint8_t)153, (uint8_t)231, (uint8_t)87, (uint8_t)129, (uint8_t)178, (uint8_t)43, (uint8_t)61, (uint8_t)121, (uint8_t)200, (uint8_t)232, (uint8_t)136, (uint8_t)242, (uint8_t)55, (uint8_t)113, (uint8_t)32, (uint8_t)137, (uint8_t)238, (uint8_t)105, (uint8_t)237, (uint8_t)173, (uint8_t)190, (uint8_t)66, (uint8_t)241, (uint8_t)92, (uint8_t)122, (uint8_t)49, (uint8_t)157, (uint8_t)252, (uint8_t)17, (uint8_t)253, (uint8_t)126, (uint8_t)102, (uint8_t)12, (uint8_t)86, (uint8_t)85, (uint8_t)13, (uint8_t)175, (uint8_t)43, (uint8_t)92, (uint8_t)69, (uint8_t)182, (uint8_t)150, (uint8_t)204, (uint8_t)63, (uint8_t)130, (uint8_t)169, (uint8_t)8, (uint8_t)107, (uint8_t)87, (uint8_t)84, (uint8_t)75, (uint8_t)16, (uint8_t)118, (uint8_t)85, (uint8_t)89, (uint8_t)131, (uint8_t)96, (uint8_t)98, (uint8_t)218, (uint8_t)191, (uint8_t)233, (uint8_t)199, (uint8_t)43, (uint8_t)31, (uint8_t)168, (uint8_t)134, (uint8_t)209, (uint8_t)117, (uint8_t)63, (uint8_t)170, (uint8_t)41, (uint8_t)171, (uint8_t)119, (uint8_t)30, (uint8_t)54, (uint8_t)52, (uint8_t)46, (uint8_t)217, (uint8_t)179, (uint8_t)72, (uint8_t)140, (uint8_t)79, (uint8_t)200, (uint8_t)180, (uint8_t)35, (uint8_t)213, (uint8_t)240, (uint8_t)128, (uint8_t)185, (uint8_t)24, (uint8_t)30, (uint8_t)6, (uint8_t)242, (uint8_t)222, (uint8_t)128, (uint8_t)198, (uint8_t)22, (uint8_t)140, (uint8_t)200, (uint8_t)252, (uint8_t)73, (uint8_t)129, (uint8_t)223, (uint8_t)223, (uint8_t)31, (uint8_t)162, (uint8_t)15, (uint8_t)199, (uint8_t)202, (uint8_t)53, (uint8_t)29, (uint8_t)42, (uint8_t)67, (uint8_t)191, (uint8_t)115, (uint8_t)216, (uint8_t)182, (uint8_t)145, (uint8_t)140, (uint8_t)216, (uint8_t)109, (uint8_t)177, (uint8_t)147, (uint8_t)157, (uint8_t)46, (uint8_t)43, (uint8_t)186, (uint8_t)209, (uint8_t)231, (uint8_t)238, (uint8_t)109, (uint8_t)172, (uint8_t)160, (uint8_t)127, (uint8_t)209, (uint8_t)225, (uint8_t)207, (uint8_t)176, (uint8_t)87, (uint8_t)89, (uint8_t)82, (uint8_t)213, (uint8_t)236, (uint8_t)187, (uint8_t)194, (uint8_t)219, (uint8_t)222, (uint8_t)87, (uint8_t)133, (uint8_t)98, (uint8_t)76, (uint8_t)13, (uint8_t)111, (uint8_t)202, (uint8_t)88, (uint8_t)209, (uint8_t)81, (uint8_t)194, (uint8_t)7, (uint8_t)248, (uint8_t)214, (uint8_t)161, (uint8_t)187, (uint8_t)155, (uint8_t)155, (uint8_t)218, (uint8_t)86, (uint8_t)26, (uint8_t)221, (uint8_t)173, (uint8_t)68, (uint8_t)175, (uint8_t)80, (uint8_t)64, (uint8_t)166, (uint8_t)124, (uint8_t)1, (uint8_t)246, (uint8_t)46, (uint8_t)178, (uint8_t)74, (uint8_t)72, (uint8_t)41, (uint8_t)90, (uint8_t)156, (uint8_t)114, (uint8_t)209, (uint8_t)62, (uint8_t)41, (uint8_t)13, (uint8_t)90, (uint8_t)25, (uint8_t)0, (uint8_t)230, (uint8_t)245, (uint8_t)51, (uint8_t)177, (uint8_t)46, (uint8_t)81, (uint8_t)129, (uint8_t)148, (uint8_t)116, (uint8_t)58, (uint8_t)180, (uint8_t)120, (uint8_t)161, (uint8_t)208, (uint8_t)244, (uint8_t)65, (uint8_t)243, (uint8_t)29, (uint8_t)212, (uint8_t)76, (uint8_t)222, (uint8_t)38, (uint8_t)162, (uint8_t)5, (uint8_t)48, (uint8_t)98, (uint8_t)38, (uint8_t)216, (uint8_t)39, (uint8_t)159, (uint8_t)239, (uint8_t)117, (uint8_t)103, (uint8_t)190, (uint8_t)250, (uint8_t)76, (uint8_t)36, (uint8_t)95, (uint8_t)88, (uint8_t)120, (uint8_t)248, (uint8_t)52, (uint8_t)190, (uint8_t)32, (uint8_t)235, (uint8_t)150, (uint8_t)154, (uint8_t)207, (uint8_t)140};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t)4, (int8_t)73, (int8_t) -117, (int8_t) -103, (int8_t) -103, (int8_t)28, (int8_t) -19, (int8_t) -10, (int8_t) -112, (int8_t)46, (int8_t)7, (int8_t) -74, (int8_t)1, (int8_t) -91, (int8_t) -98, (int8_t) -17, (int8_t)93, (int8_t)56, (int8_t)93, (int8_t) -113, (int8_t)107, (int8_t)80, (int8_t) -20, (int8_t)21, (int8_t) -73, (int8_t) -53, (int8_t) -11, (int8_t) -112, (int8_t)6, (int8_t)64, (int8_t) -78, (int8_t)67};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)8403, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float)1.168707E38F, PH.base.pack) ;
        p250_x_SET((float)3.1071465E38F, PH.base.pack) ;
        p250_z_SET((float)1.6441413E38F, PH.base.pack) ;
        {
            char16_t* name = u"iczehvof";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)5023732387116550688L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -7.5580397E36F, PH.base.pack) ;
        {
            char16_t* name = u"dkzCwiJwzr";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)3644101452L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)1067250052, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)599121679L, PH.base.pack) ;
        {
            char16_t* name = u"mTddzrpddj";
            p252_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"ogtezpnevcgvhitcns";
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
        p254_ind_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)2356209675L, PH.base.pack) ;
        p254_value_SET((float) -2.9182597E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_target_component_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)184, (uint8_t)219, (uint8_t)217, (uint8_t)158, (uint8_t)254, (uint8_t)244, (uint8_t)204, (uint8_t)85, (uint8_t)77, (uint8_t)192, (uint8_t)151, (uint8_t)245, (uint8_t)255, (uint8_t)58, (uint8_t)147, (uint8_t)46, (uint8_t)217, (uint8_t)79, (uint8_t)204, (uint8_t)81, (uint8_t)161, (uint8_t)127, (uint8_t)228, (uint8_t)45, (uint8_t)9, (uint8_t)74, (uint8_t)144, (uint8_t)99, (uint8_t)67, (uint8_t)160, (uint8_t)250, (uint8_t)215};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)4759514120119289553L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)3052068457L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)33263741L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        {
            char16_t* tune = u"tozcg";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)586924252L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)15894, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)72, (uint8_t)183, (uint8_t)249, (uint8_t)185, (uint8_t)196, (uint8_t)19, (uint8_t)182, (uint8_t)240, (uint8_t)139, (uint8_t)77, (uint8_t)49, (uint8_t)103, (uint8_t)108, (uint8_t)81, (uint8_t)143, (uint8_t)30, (uint8_t)43, (uint8_t)14, (uint8_t)178, (uint8_t)241, (uint8_t)158, (uint8_t)159, (uint8_t)186, (uint8_t)225, (uint8_t)89, (uint8_t)226, (uint8_t)120, (uint8_t)68, (uint8_t)114, (uint8_t)147, (uint8_t)90, (uint8_t)68};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_time_boot_ms_SET((uint32_t)577165625L, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -8.4406495E37F, PH.base.pack) ;
        p259_sensor_size_h_SET((float)1.9747617E38F, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)35800, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)48, (uint8_t)250, (uint8_t)33, (uint8_t)88, (uint8_t)61, (uint8_t)167, (uint8_t)59, (uint8_t)7, (uint8_t)90, (uint8_t)225, (uint8_t)83, (uint8_t)99, (uint8_t)82, (uint8_t)74, (uint8_t)233, (uint8_t)215, (uint8_t)218, (uint8_t)86, (uint8_t)170, (uint8_t)174, (uint8_t)132, (uint8_t)7, (uint8_t)71, (uint8_t)5, (uint8_t)250, (uint8_t)161, (uint8_t)104, (uint8_t)3, (uint8_t)94, (uint8_t)109, (uint8_t)115, (uint8_t)105};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)44754, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"qmRdxzgiudepldoapmwaqtmVxcwlKvupgfOrmxzjmtnhklEpkeUebgvBsyUqitWpievInigmntcUdwvrkexedhKisdlxgdtyyofpcwahnpztnz";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_focal_length_SET((float) -9.893087E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)2108997621L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_time_boot_ms_SET((uint32_t)3302372977L, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p261_available_capacity_SET((float)1.1602998E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -1.5792938E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -1.5280247E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p261_write_speed_SET((float) -2.8285828E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)2.3382012E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float) -2.7720014E38F, PH.base.pack) ;
        p262_available_capacity_SET((float) -4.9887887E37F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2642282647L, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3070506308L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lat_SET((int32_t) -1078123946, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -24478503, PH.base.pack) ;
        {
            float q[] =  {8.2884533E37F, 2.5477144E38F, -7.694992E37F, 3.3994414E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_alt_SET((int32_t)109063020, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)3895740126L, PH.base.pack) ;
        p263_lon_SET((int32_t) -1275919850, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)8668743606971487687L, PH.base.pack) ;
        {
            char16_t* file_url = u"phQmNhomoctwnssknomxufhzqlkhyzchdrpcQTsrbggazxeslKwcrlcpQMgrbasfmzyhmNdwBqjYlwistsjxztesKeRnbywidbvvhhcjlinbrmddblwsZfjukinolgjQsaqzpOzwfYrmuxbpzovkydfqprYAmzsxeczevirlmzltrPeffwrUhubyxifoLpkbnmqnuu";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_capture_result_SET((int8_t)(int8_t)53, PH.base.pack) ;
        p263_image_index_SET((int32_t) -2000711644, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)1686345821L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)255503551503881297L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)4879600289714646429L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)874959079588848711L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float)5.8667184E37F, PH.base.pack) ;
        p265_roll_SET((float)3.2338298E38F, PH.base.pack) ;
        p265_pitch_SET((float)3.3692891E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1448016743L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_first_message_offset_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)60, (uint8_t)74, (uint8_t)129, (uint8_t)189, (uint8_t)248, (uint8_t)70, (uint8_t)91, (uint8_t)239, (uint8_t)35, (uint8_t)181, (uint8_t)138, (uint8_t)193, (uint8_t)173, (uint8_t)245, (uint8_t)161, (uint8_t)127, (uint8_t)177, (uint8_t)197, (uint8_t)159, (uint8_t)47, (uint8_t)83, (uint8_t)162, (uint8_t)7, (uint8_t)178, (uint8_t)242, (uint8_t)156, (uint8_t)14, (uint8_t)49, (uint8_t)100, (uint8_t)124, (uint8_t)42, (uint8_t)155, (uint8_t)100, (uint8_t)155, (uint8_t)244, (uint8_t)94, (uint8_t)152, (uint8_t)246, (uint8_t)249, (uint8_t)89, (uint8_t)199, (uint8_t)92, (uint8_t)76, (uint8_t)164, (uint8_t)130, (uint8_t)37, (uint8_t)110, (uint8_t)193, (uint8_t)28, (uint8_t)140, (uint8_t)91, (uint8_t)189, (uint8_t)226, (uint8_t)161, (uint8_t)42, (uint8_t)91, (uint8_t)163, (uint8_t)209, (uint8_t)131, (uint8_t)7, (uint8_t)31, (uint8_t)54, (uint8_t)158, (uint8_t)105, (uint8_t)186, (uint8_t)114, (uint8_t)20, (uint8_t)51, (uint8_t)254, (uint8_t)19, (uint8_t)54, (uint8_t)230, (uint8_t)236, (uint8_t)184, (uint8_t)205, (uint8_t)33, (uint8_t)101, (uint8_t)109, (uint8_t)133, (uint8_t)68, (uint8_t)224, (uint8_t)173, (uint8_t)156, (uint8_t)35, (uint8_t)171, (uint8_t)218, (uint8_t)235, (uint8_t)77, (uint8_t)238, (uint8_t)215, (uint8_t)164, (uint8_t)241, (uint8_t)38, (uint8_t)51, (uint8_t)43, (uint8_t)114, (uint8_t)221, (uint8_t)167, (uint8_t)74, (uint8_t)73, (uint8_t)57, (uint8_t)234, (uint8_t)119, (uint8_t)221, (uint8_t)69, (uint8_t)124, (uint8_t)72, (uint8_t)254, (uint8_t)36, (uint8_t)35, (uint8_t)32, (uint8_t)183, (uint8_t)28, (uint8_t)241, (uint8_t)24, (uint8_t)7, (uint8_t)22, (uint8_t)73, (uint8_t)200, (uint8_t)235, (uint8_t)224, (uint8_t)70, (uint8_t)103, (uint8_t)115, (uint8_t)29, (uint8_t)18, (uint8_t)209, (uint8_t)82, (uint8_t)119, (uint8_t)39, (uint8_t)76, (uint8_t)27, (uint8_t)79, (uint8_t)76, (uint8_t)35, (uint8_t)18, (uint8_t)204, (uint8_t)6, (uint8_t)18, (uint8_t)142, (uint8_t)251, (uint8_t)230, (uint8_t)250, (uint8_t)185, (uint8_t)97, (uint8_t)204, (uint8_t)83, (uint8_t)193, (uint8_t)99, (uint8_t)185, (uint8_t)184, (uint8_t)233, (uint8_t)108, (uint8_t)146, (uint8_t)188, (uint8_t)138, (uint8_t)96, (uint8_t)134, (uint8_t)4, (uint8_t)205, (uint8_t)116, (uint8_t)74, (uint8_t)56, (uint8_t)38, (uint8_t)145, (uint8_t)229, (uint8_t)158, (uint8_t)146, (uint8_t)243, (uint8_t)62, (uint8_t)119, (uint8_t)164, (uint8_t)61, (uint8_t)207, (uint8_t)29, (uint8_t)190, (uint8_t)211, (uint8_t)70, (uint8_t)132, (uint8_t)85, (uint8_t)49, (uint8_t)7, (uint8_t)112, (uint8_t)27, (uint8_t)102, (uint8_t)5, (uint8_t)193, (uint8_t)198, (uint8_t)20, (uint8_t)101, (uint8_t)55, (uint8_t)173, (uint8_t)120, (uint8_t)220, (uint8_t)145, (uint8_t)152, (uint8_t)170, (uint8_t)232, (uint8_t)148, (uint8_t)213, (uint8_t)190, (uint8_t)246, (uint8_t)49, (uint8_t)155, (uint8_t)209, (uint8_t)74, (uint8_t)39, (uint8_t)23, (uint8_t)54, (uint8_t)108, (uint8_t)189, (uint8_t)165, (uint8_t)114, (uint8_t)147, (uint8_t)152, (uint8_t)101, (uint8_t)8, (uint8_t)182, (uint8_t)248, (uint8_t)92, (uint8_t)36, (uint8_t)16, (uint8_t)90, (uint8_t)55, (uint8_t)204, (uint8_t)195, (uint8_t)92, (uint8_t)63, (uint8_t)238, (uint8_t)97, (uint8_t)68, (uint8_t)94, (uint8_t)87, (uint8_t)138, (uint8_t)56, (uint8_t)84, (uint8_t)162, (uint8_t)58, (uint8_t)96, (uint8_t)33, (uint8_t)22, (uint8_t)188, (uint8_t)159, (uint8_t)61, (uint8_t)62, (uint8_t)172, (uint8_t)189, (uint8_t)127, (uint8_t)85};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)37603, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)28922, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)28, (uint8_t)37, (uint8_t)157, (uint8_t)200, (uint8_t)251, (uint8_t)159, (uint8_t)30, (uint8_t)34, (uint8_t)196, (uint8_t)20, (uint8_t)203, (uint8_t)38, (uint8_t)241, (uint8_t)240, (uint8_t)124, (uint8_t)211, (uint8_t)231, (uint8_t)15, (uint8_t)149, (uint8_t)44, (uint8_t)23, (uint8_t)29, (uint8_t)232, (uint8_t)176, (uint8_t)229, (uint8_t)35, (uint8_t)118, (uint8_t)110, (uint8_t)248, (uint8_t)55, (uint8_t)167, (uint8_t)123, (uint8_t)6, (uint8_t)44, (uint8_t)58, (uint8_t)2, (uint8_t)2, (uint8_t)173, (uint8_t)149, (uint8_t)172, (uint8_t)58, (uint8_t)220, (uint8_t)119, (uint8_t)145, (uint8_t)104, (uint8_t)211, (uint8_t)27, (uint8_t)99, (uint8_t)222, (uint8_t)107, (uint8_t)211, (uint8_t)45, (uint8_t)173, (uint8_t)80, (uint8_t)98, (uint8_t)77, (uint8_t)183, (uint8_t)72, (uint8_t)13, (uint8_t)92, (uint8_t)166, (uint8_t)166, (uint8_t)173, (uint8_t)5, (uint8_t)203, (uint8_t)109, (uint8_t)148, (uint8_t)202, (uint8_t)83, (uint8_t)101, (uint8_t)195, (uint8_t)78, (uint8_t)232, (uint8_t)201, (uint8_t)66, (uint8_t)194, (uint8_t)255, (uint8_t)2, (uint8_t)14, (uint8_t)85, (uint8_t)83, (uint8_t)13, (uint8_t)227, (uint8_t)215, (uint8_t)213, (uint8_t)155, (uint8_t)128, (uint8_t)107, (uint8_t)33, (uint8_t)69, (uint8_t)233, (uint8_t)124, (uint8_t)68, (uint8_t)24, (uint8_t)71, (uint8_t)95, (uint8_t)26, (uint8_t)246, (uint8_t)186, (uint8_t)18, (uint8_t)55, (uint8_t)103, (uint8_t)35, (uint8_t)186, (uint8_t)209, (uint8_t)175, (uint8_t)142, (uint8_t)213, (uint8_t)107, (uint8_t)69, (uint8_t)186, (uint8_t)202, (uint8_t)176, (uint8_t)75, (uint8_t)148, (uint8_t)101, (uint8_t)154, (uint8_t)109, (uint8_t)213, (uint8_t)62, (uint8_t)83, (uint8_t)209, (uint8_t)117, (uint8_t)255, (uint8_t)43, (uint8_t)154, (uint8_t)240, (uint8_t)227, (uint8_t)52, (uint8_t)38, (uint8_t)183, (uint8_t)115, (uint8_t)164, (uint8_t)212, (uint8_t)30, (uint8_t)74, (uint8_t)220, (uint8_t)41, (uint8_t)21, (uint8_t)119, (uint8_t)62, (uint8_t)103, (uint8_t)242, (uint8_t)50, (uint8_t)92, (uint8_t)183, (uint8_t)57, (uint8_t)53, (uint8_t)18, (uint8_t)253, (uint8_t)83, (uint8_t)203, (uint8_t)103, (uint8_t)243, (uint8_t)209, (uint8_t)117, (uint8_t)153, (uint8_t)28, (uint8_t)169, (uint8_t)123, (uint8_t)24, (uint8_t)231, (uint8_t)112, (uint8_t)1, (uint8_t)162, (uint8_t)73, (uint8_t)4, (uint8_t)16, (uint8_t)235, (uint8_t)131, (uint8_t)141, (uint8_t)32, (uint8_t)112, (uint8_t)228, (uint8_t)47, (uint8_t)122, (uint8_t)230, (uint8_t)194, (uint8_t)170, (uint8_t)53, (uint8_t)217, (uint8_t)214, (uint8_t)50, (uint8_t)14, (uint8_t)228, (uint8_t)75, (uint8_t)221, (uint8_t)251, (uint8_t)149, (uint8_t)14, (uint8_t)107, (uint8_t)240, (uint8_t)102, (uint8_t)76, (uint8_t)249, (uint8_t)235, (uint8_t)247, (uint8_t)250, (uint8_t)169, (uint8_t)150, (uint8_t)103, (uint8_t)245, (uint8_t)186, (uint8_t)82, (uint8_t)253, (uint8_t)156, (uint8_t)109, (uint8_t)71, (uint8_t)44, (uint8_t)97, (uint8_t)61, (uint8_t)173, (uint8_t)114, (uint8_t)26, (uint8_t)140, (uint8_t)43, (uint8_t)95, (uint8_t)88, (uint8_t)241, (uint8_t)80, (uint8_t)45, (uint8_t)214, (uint8_t)154, (uint8_t)230, (uint8_t)28, (uint8_t)146, (uint8_t)219, (uint8_t)96, (uint8_t)76, (uint8_t)43, (uint8_t)148, (uint8_t)238, (uint8_t)212, (uint8_t)156, (uint8_t)43, (uint8_t)119, (uint8_t)194, (uint8_t)71, (uint8_t)145, (uint8_t)2, (uint8_t)0, (uint8_t)191, (uint8_t)62, (uint8_t)107, (uint8_t)61, (uint8_t)147, (uint8_t)89, (uint8_t)74, (uint8_t)251};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_system_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)63635, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_resolution_h_SET((uint16_t)(uint16_t)12429, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)13268, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)3567902515L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        {
            char16_t* uri = u"tmuozqjqkuNvbtvkozjjqdfijreadupfie";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_rotation_SET((uint16_t)(uint16_t)3582, PH.base.pack) ;
        p269_framerate_SET((float) -1.2616542E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)1433860849L, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)47373, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)32942, PH.base.pack) ;
        {
            char16_t* uri = u"sktkwifpafozhzlifyahajlgaqlamekqdledsgoateonlqkwXpqqqxawnfv";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_framerate_SET((float)1.0374664E38F, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)25551, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"JstebgskaBkqUsYzrCiha";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"ks";
            p299_password_SET_(password, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)1385, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)86, (uint8_t)97, (uint8_t)160, (uint8_t)67, (uint8_t)47, (uint8_t)56, (uint8_t)210, (uint8_t)161};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)55042, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)26443, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)215, (uint8_t)30, (uint8_t)141, (uint8_t)26, (uint8_t)105, (uint8_t)87, (uint8_t)74, (uint8_t)59};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_sub_mode_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)21659, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)5931053939311141478L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)2636436347L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_major_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)2437690819L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1733437275L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)5288525481692905638L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        {
            char16_t* name = u"NbajrEchfedTrxbnfppmhavotqnxwfsniftUYfzvjZgifcLepddbqxtOacgaluXqcpxdwemqzfyfwP";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_major_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)173, (uint8_t)149, (uint8_t)19, (uint8_t)12, (uint8_t)100, (uint8_t)78, (uint8_t)249, (uint8_t)244, (uint8_t)248, (uint8_t)104, (uint8_t)211, (uint8_t)186, (uint8_t)36, (uint8_t)236, (uint8_t)148, (uint8_t)204};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t)2576, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            char16_t* param_id = u"gxexzohxetbVqwzk";
            p320_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)38192, PH.base.pack) ;
        {
            char16_t* param_id = u"ucsguflxssmdzc";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"EzfhgheejjtOzaharxndxxbqshqzwebljepgdndTfUmm";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)46600, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        {
            char16_t* param_value = u"vttovfajjvoGwppbdcuvoogjjVjfzsCwytevyedgXIhaxbfxymjcaDGmPbL";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        {
            char16_t* param_id = u"Wdqsjmzmtddmnum";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"nkufilpPaFcmekhkffnuwvDSyifqtejprekyckrqfPajjaiXoglrlAhjFalteprzAwqqzzvsadhdeWowvjlLgxkacxcteLtmrkjCgt";
            p324_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"jzsCfvindnivj";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)44897, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)24759, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)5464788776190332383L, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)48610, (uint16_t)46399, (uint16_t)27883, (uint16_t)56765, (uint16_t)33709, (uint16_t)51424, (uint16_t)9627, (uint16_t)27362, (uint16_t)41025, (uint16_t)59867, (uint16_t)64750, (uint16_t)27499, (uint16_t)59769, (uint16_t)25643, (uint16_t)59788, (uint16_t)1762, (uint16_t)24012, (uint16_t)2303, (uint16_t)14927, (uint16_t)30360, (uint16_t)12278, (uint16_t)4461, (uint16_t)36551, (uint16_t)9652, (uint16_t)12655, (uint16_t)52836, (uint16_t)56631, (uint16_t)64899, (uint16_t)49042, (uint16_t)27291, (uint16_t)16198, (uint16_t)50806, (uint16_t)55572, (uint16_t)4518, (uint16_t)13485, (uint16_t)31607, (uint16_t)51667, (uint16_t)58526, (uint16_t)16343, (uint16_t)14197, (uint16_t)1068, (uint16_t)32416, (uint16_t)30396, (uint16_t)22742, (uint16_t)18019, (uint16_t)9221, (uint16_t)60089, (uint16_t)29901, (uint16_t)329, (uint16_t)31259, (uint16_t)50184, (uint16_t)63963, (uint16_t)20226, (uint16_t)14428, (uint16_t)40444, (uint16_t)9168, (uint16_t)15303, (uint16_t)21279, (uint16_t)60617, (uint16_t)46420, (uint16_t)27713, (uint16_t)31116, (uint16_t)45828, (uint16_t)57588, (uint16_t)60448, (uint16_t)53030, (uint16_t)5983, (uint16_t)27138, (uint16_t)55930, (uint16_t)49595, (uint16_t)45484, (uint16_t)21100};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}


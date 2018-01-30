
package org.noname;
import java.util.*;
import static org.unirail.BlackBox.BitUtils.*;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import java.io.IOException;

public class Test extends DemoDevice
{




    static class TestChannel extends Channel
    {
        static final TestChannel instance = new TestChannel(); //test channel

        public final java.io.InputStream  inputStream          = new InputStream();
        public final java.io.OutputStream outputStream         = new OutputStream();
        public final java.io.InputStream  inputStreamAdvanced  = new AdvancedInputStream();
        public final java.io.OutputStream outputStreamAdvanced = new AdvancedOutputStream();

        @Override protected void failure(String reason)
        {
            super.failure(reason);
            assert(false);
        }



        static Pack testing_pack; //one pack send/receive buffer

        void send(Pack pack) { testing_pack = pack; }

        final Bounds.Inside ph = new Bounds.Inside();

        @Override protected Pack process(Pack pack, int id)
        {
            switch(id)
            {
                case Channel.PROCESS_CHANNEL_REQEST:
                    if(pack == null)
                    {
                        pack = testing_pack;
                        testing_pack = null;
                    }
                    else testing_pack = pack;
                    return pack;
                case Channel.PROCESS_RECEIVED:
                    if(testing_pack == null) return null;
                    ph.setPack(pack = testing_pack);
                    testing_pack = null;
                    id = pack.meta.id;
            }
            switch(id)
            {
            }
            return null;
        }
        static final byte[] buff = new byte[1024];
        static void transmission(java.io.InputStream src, java.io.OutputStream dst, Channel dst_ch)
        {
            try
            {
                if(src instanceof AdvancedInputStream && !(dst instanceof AdvancedOutputStream))
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStreamAdvanced.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStream.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else if(!(src instanceof AdvancedInputStream) && dst instanceof AdvancedOutputStream)
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStream.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStreamAdvanced.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                processReceived(dst_ch);
            }
            catch(IOException e)
            {
                e.printStackTrace();
                assert(false);
            }
        }
    }

    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        LoopBackDemoChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_EMERGENCY);
            assert(pack.custom_mode_GET() == 3364154311L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER);
            assert(pack.mavlink_version_GET() == (char)106);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)106) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD) ;
        p0.custom_mode_SET(3364154311L) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_EMERGENCY) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)4966);
            assert(pack.errors_count3_GET() == (char)8816);
            assert(pack.errors_count2_GET() == (char)38222);
            assert(pack.drop_rate_comm_GET() == (char)57106);
            assert(pack.current_battery_GET() == (short) -16061);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
            assert(pack.errors_count4_GET() == (char)45182);
            assert(pack.load_GET() == (char)43940);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
            assert(pack.voltage_battery_GET() == (char)38862);
            assert(pack.battery_remaining_GET() == (byte) - 28);
            assert(pack.errors_count1_GET() == (char)8174);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)4966) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG) ;
        p1.load_SET((char)43940) ;
        p1.errors_count3_SET((char)8816) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION) ;
        p1.drop_rate_comm_SET((char)57106) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL) ;
        p1.errors_count2_SET((char)38222) ;
        p1.errors_count4_SET((char)45182) ;
        p1.voltage_battery_SET((char)38862) ;
        p1.errors_count1_SET((char)8174) ;
        p1.battery_remaining_SET((byte) - 28) ;
        p1.current_battery_SET((short) -16061) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 797142646506809519L);
            assert(pack.time_boot_ms_GET() == 2187601352L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2187601352L) ;
        p2.time_unix_usec_SET(797142646506809519L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == 5.700945E37F);
            assert(pack.time_boot_ms_GET() == 1044825468L);
            assert(pack.vz_GET() == 1.8832463E38F);
            assert(pack.type_mask_GET() == (char)50811);
            assert(pack.y_GET() == 1.3503946E38F);
            assert(pack.z_GET() == -1.6529292E38F);
            assert(pack.vy_GET() == -8.711896E37F);
            assert(pack.afz_GET() == 1.2360824E38F);
            assert(pack.afy_GET() == 2.8331222E38F);
            assert(pack.vx_GET() == 2.7129552E38F);
            assert(pack.yaw_GET() == -1.1214044E38F);
            assert(pack.x_GET() == 2.4888987E38F);
            assert(pack.yaw_rate_GET() == 3.153588E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.yaw_SET(-1.1214044E38F) ;
        p3.z_SET(-1.6529292E38F) ;
        p3.type_mask_SET((char)50811) ;
        p3.vz_SET(1.8832463E38F) ;
        p3.yaw_rate_SET(3.153588E38F) ;
        p3.afx_SET(5.700945E37F) ;
        p3.x_SET(2.4888987E38F) ;
        p3.vx_SET(2.7129552E38F) ;
        p3.afz_SET(1.2360824E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p3.time_boot_ms_SET(1044825468L) ;
        p3.y_SET(1.3503946E38F) ;
        p3.vy_SET(-8.711896E37F) ;
        p3.afy_SET(2.8331222E38F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 2028380744L);
            assert(pack.target_component_GET() == (char)101);
            assert(pack.time_usec_GET() == 5139728976133459337L);
            assert(pack.target_system_GET() == (char)252);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.target_system_SET((char)252) ;
        p4.seq_SET(2028380744L) ;
        p4.time_usec_SET(5139728976133459337L) ;
        p4.target_component_SET((char)101) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)34);
            assert(pack.passkey_LEN(ph) == 23);
            assert(pack.passkey_TRY(ph).equals("huzfsvaeclLJtfvuLulgtaj"));
            assert(pack.control_request_GET() == (char)88);
            assert(pack.version_GET() == (char)120);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.version_SET((char)120) ;
        p5.target_system_SET((char)34) ;
        p5.control_request_SET((char)88) ;
        p5.passkey_SET("huzfsvaeclLJtfvuLulgtaj", PH) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)118);
            assert(pack.control_request_GET() == (char)130);
            assert(pack.ack_GET() == (char)218);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)118) ;
        p6.control_request_SET((char)130) ;
        p6.ack_SET((char)218) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 30);
            assert(pack.key_TRY(ph).equals("tlxoCncekamglicmyldskoziMMceve"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("tlxoCncekamglicmyldskoziMMceve", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.target_system_GET() == (char)142);
            assert(pack.custom_mode_GET() == 3082935544L);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)142) ;
        p11.custom_mode_SET(3082935544L) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("aXdEe"));
            assert(pack.target_system_GET() == (char)29);
            assert(pack.param_index_GET() == (short)24213);
            assert(pack.target_component_GET() == (char)192);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)29) ;
        p20.param_id_SET("aXdEe", PH) ;
        p20.target_component_SET((char)192) ;
        p20.param_index_SET((short)24213) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)99);
            assert(pack.target_system_GET() == (char)118);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)118) ;
        p21.target_component_SET((char)99) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("wuybjvqtwQwtuA"));
            assert(pack.param_value_GET() == -3.0423955E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_count_GET() == (char)6303);
            assert(pack.param_index_GET() == (char)62821);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)62821) ;
        p22.param_count_SET((char)6303) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p22.param_value_SET(-3.0423955E38F) ;
        p22.param_id_SET("wuybjvqtwQwtuA", PH) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("qbelcjYumza"));
            assert(pack.target_component_GET() == (char)246);
            assert(pack.param_value_GET() == 2.1863963E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            assert(pack.target_system_GET() == (char)52);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("qbelcjYumza", PH) ;
        p23.target_component_SET((char)246) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        p23.param_value_SET(2.1863963E38F) ;
        p23.target_system_SET((char)52) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_acc_TRY(ph) == 3039839620L);
            assert(pack.satellites_visible_GET() == (char)88);
            assert(pack.v_acc_TRY(ph) == 3960633109L);
            assert(pack.cog_GET() == (char)63732);
            assert(pack.lon_GET() == 978166570);
            assert(pack.alt_GET() == 750062361);
            assert(pack.lat_GET() == -1904620995);
            assert(pack.vel_GET() == (char)12584);
            assert(pack.epv_GET() == (char)53873);
            assert(pack.hdg_acc_TRY(ph) == 454465964L);
            assert(pack.eph_GET() == (char)13417);
            assert(pack.alt_ellipsoid_TRY(ph) == -354499337);
            assert(pack.time_usec_GET() == 4387945816888140029L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.h_acc_TRY(ph) == 2335709679L);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.alt_ellipsoid_SET(-354499337, PH) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p24.vel_acc_SET(3039839620L, PH) ;
        p24.lat_SET(-1904620995) ;
        p24.epv_SET((char)53873) ;
        p24.alt_SET(750062361) ;
        p24.v_acc_SET(3960633109L, PH) ;
        p24.h_acc_SET(2335709679L, PH) ;
        p24.eph_SET((char)13417) ;
        p24.vel_SET((char)12584) ;
        p24.hdg_acc_SET(454465964L, PH) ;
        p24.lon_SET(978166570) ;
        p24.time_usec_SET(4387945816888140029L) ;
        p24.cog_SET((char)63732) ;
        p24.satellites_visible_SET((char)88) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)145);
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)56, (char)24, (char)116, (char)85, (char)193, (char)38, (char)171, (char)163, (char)113, (char)188, (char)69, (char)87, (char)197, (char)88, (char)116, (char)220, (char)218, (char)61, (char)41, (char)30}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)163, (char)242, (char)153, (char)123, (char)2, (char)214, (char)57, (char)190, (char)172, (char)65, (char)225, (char)71, (char)113, (char)189, (char)95, (char)87, (char)49, (char)164, (char)194, (char)137}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)122, (char)36, (char)15, (char)151, (char)149, (char)30, (char)102, (char)110, (char)72, (char)208, (char)227, (char)106, (char)39, (char)145, (char)179, (char)157, (char)4, (char)13, (char)45, (char)49}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)40, (char)3, (char)2, (char)123, (char)153, (char)38, (char)26, (char)50, (char)14, (char)164, (char)236, (char)65, (char)140, (char)195, (char)126, (char)29, (char)117, (char)174, (char)89, (char)1}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)111, (char)33, (char)141, (char)5, (char)193, (char)238, (char)47, (char)80, (char)219, (char)249, (char)42, (char)149, (char)238, (char)154, (char)192, (char)89, (char)186, (char)118, (char)113, (char)133}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)40, (char)3, (char)2, (char)123, (char)153, (char)38, (char)26, (char)50, (char)14, (char)164, (char)236, (char)65, (char)140, (char)195, (char)126, (char)29, (char)117, (char)174, (char)89, (char)1}, 0) ;
        p25.satellites_visible_SET((char)145) ;
        p25.satellite_used_SET(new char[] {(char)111, (char)33, (char)141, (char)5, (char)193, (char)238, (char)47, (char)80, (char)219, (char)249, (char)42, (char)149, (char)238, (char)154, (char)192, (char)89, (char)186, (char)118, (char)113, (char)133}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)56, (char)24, (char)116, (char)85, (char)193, (char)38, (char)171, (char)163, (char)113, (char)188, (char)69, (char)87, (char)197, (char)88, (char)116, (char)220, (char)218, (char)61, (char)41, (char)30}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)163, (char)242, (char)153, (char)123, (char)2, (char)214, (char)57, (char)190, (char)172, (char)65, (char)225, (char)71, (char)113, (char)189, (char)95, (char)87, (char)49, (char)164, (char)194, (char)137}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)122, (char)36, (char)15, (char)151, (char)149, (char)30, (char)102, (char)110, (char)72, (char)208, (char)227, (char)106, (char)39, (char)145, (char)179, (char)157, (char)4, (char)13, (char)45, (char)49}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -29734);
            assert(pack.xgyro_GET() == (short) -26877);
            assert(pack.yacc_GET() == (short)18095);
            assert(pack.ygyro_GET() == (short) -20925);
            assert(pack.ymag_GET() == (short)25787);
            assert(pack.xacc_GET() == (short)10461);
            assert(pack.xmag_GET() == (short)26677);
            assert(pack.zmag_GET() == (short) -15845);
            assert(pack.zacc_GET() == (short)11650);
            assert(pack.time_boot_ms_GET() == 247954304L);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.zacc_SET((short)11650) ;
        p26.xacc_SET((short)10461) ;
        p26.yacc_SET((short)18095) ;
        p26.ygyro_SET((short) -20925) ;
        p26.ymag_SET((short)25787) ;
        p26.zgyro_SET((short) -29734) ;
        p26.zmag_SET((short) -15845) ;
        p26.time_boot_ms_SET(247954304L) ;
        p26.xmag_SET((short)26677) ;
        p26.xgyro_SET((short) -26877) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)7403);
            assert(pack.ygyro_GET() == (short)4589);
            assert(pack.zgyro_GET() == (short) -29684);
            assert(pack.time_usec_GET() == 7397329234753607849L);
            assert(pack.zacc_GET() == (short)17688);
            assert(pack.zmag_GET() == (short)5537);
            assert(pack.yacc_GET() == (short)27029);
            assert(pack.xacc_GET() == (short)9874);
            assert(pack.xmag_GET() == (short)18089);
            assert(pack.xgyro_GET() == (short)13084);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.xmag_SET((short)18089) ;
        p27.zacc_SET((short)17688) ;
        p27.zgyro_SET((short) -29684) ;
        p27.xgyro_SET((short)13084) ;
        p27.ygyro_SET((short)4589) ;
        p27.xacc_SET((short)9874) ;
        p27.yacc_SET((short)27029) ;
        p27.ymag_SET((short)7403) ;
        p27.zmag_SET((short)5537) ;
        p27.time_usec_SET(7397329234753607849L) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short)25084);
            assert(pack.press_abs_GET() == (short)7110);
            assert(pack.press_diff2_GET() == (short) -17367);
            assert(pack.temperature_GET() == (short) -782);
            assert(pack.time_usec_GET() == 4442712938280937645L);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(4442712938280937645L) ;
        p28.temperature_SET((short) -782) ;
        p28.press_diff1_SET((short)25084) ;
        p28.press_abs_SET((short)7110) ;
        p28.press_diff2_SET((short) -17367) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 933503932L);
            assert(pack.temperature_GET() == (short)22207);
            assert(pack.press_diff_GET() == 3.3711377E38F);
            assert(pack.press_abs_GET() == -3.0820239E38F);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)22207) ;
        p29.time_boot_ms_SET(933503932L) ;
        p29.press_diff_SET(3.3711377E38F) ;
        p29.press_abs_SET(-3.0820239E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 6.2200917E37F);
            assert(pack.roll_GET() == -2.2044091E38F);
            assert(pack.pitchspeed_GET() == 1.6395327E38F);
            assert(pack.pitch_GET() == -1.3037706E38F);
            assert(pack.time_boot_ms_GET() == 3220558401L);
            assert(pack.yawspeed_GET() == 2.106714E38F);
            assert(pack.yaw_GET() == 1.5471933E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(3220558401L) ;
        p30.yawspeed_SET(2.106714E38F) ;
        p30.pitch_SET(-1.3037706E38F) ;
        p30.rollspeed_SET(6.2200917E37F) ;
        p30.yaw_SET(1.5471933E38F) ;
        p30.pitchspeed_SET(1.6395327E38F) ;
        p30.roll_SET(-2.2044091E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3433131204L);
            assert(pack.rollspeed_GET() == -3.2589242E38F);
            assert(pack.q3_GET() == -3.0977136E38F);
            assert(pack.q4_GET() == 1.7176937E38F);
            assert(pack.yawspeed_GET() == -8.073534E37F);
            assert(pack.pitchspeed_GET() == 2.9531821E38F);
            assert(pack.q2_GET() == -1.5549353E38F);
            assert(pack.q1_GET() == 2.645251E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q1_SET(2.645251E38F) ;
        p31.q3_SET(-3.0977136E38F) ;
        p31.pitchspeed_SET(2.9531821E38F) ;
        p31.q2_SET(-1.5549353E38F) ;
        p31.q4_SET(1.7176937E38F) ;
        p31.rollspeed_SET(-3.2589242E38F) ;
        p31.time_boot_ms_SET(3433131204L) ;
        p31.yawspeed_SET(-8.073534E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == -5.5747765E37F);
            assert(pack.z_GET() == 1.1364162E38F);
            assert(pack.vy_GET() == -2.2531777E37F);
            assert(pack.vx_GET() == 1.6859543E38F);
            assert(pack.x_GET() == -8.526177E37F);
            assert(pack.time_boot_ms_GET() == 1539226748L);
            assert(pack.y_GET() == 2.9021532E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(1539226748L) ;
        p32.vz_SET(-5.5747765E37F) ;
        p32.vx_SET(1.6859543E38F) ;
        p32.z_SET(1.1364162E38F) ;
        p32.y_SET(2.9021532E38F) ;
        p32.x_SET(-8.526177E37F) ;
        p32.vy_SET(-2.2531777E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 2103672512);
            assert(pack.relative_alt_GET() == 1046466546);
            assert(pack.time_boot_ms_GET() == 484926260L);
            assert(pack.alt_GET() == -1966408448);
            assert(pack.hdg_GET() == (char)3155);
            assert(pack.lat_GET() == -167625219);
            assert(pack.vx_GET() == (short) -4786);
            assert(pack.vz_GET() == (short) -23501);
            assert(pack.vy_GET() == (short) -6660);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vy_SET((short) -6660) ;
        p33.lat_SET(-167625219) ;
        p33.hdg_SET((char)3155) ;
        p33.relative_alt_SET(1046466546) ;
        p33.time_boot_ms_SET(484926260L) ;
        p33.vz_SET((short) -23501) ;
        p33.alt_SET(-1966408448) ;
        p33.lon_SET(2103672512) ;
        p33.vx_SET((short) -4786) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan2_scaled_GET() == (short)10008);
            assert(pack.chan1_scaled_GET() == (short) -24662);
            assert(pack.chan4_scaled_GET() == (short)30172);
            assert(pack.rssi_GET() == (char)198);
            assert(pack.time_boot_ms_GET() == 753142756L);
            assert(pack.chan6_scaled_GET() == (short) -26925);
            assert(pack.chan8_scaled_GET() == (short)13823);
            assert(pack.chan3_scaled_GET() == (short) -23538);
            assert(pack.chan5_scaled_GET() == (short)28229);
            assert(pack.port_GET() == (char)102);
            assert(pack.chan7_scaled_GET() == (short)29976);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan7_scaled_SET((short)29976) ;
        p34.chan3_scaled_SET((short) -23538) ;
        p34.chan8_scaled_SET((short)13823) ;
        p34.time_boot_ms_SET(753142756L) ;
        p34.chan1_scaled_SET((short) -24662) ;
        p34.port_SET((char)102) ;
        p34.chan5_scaled_SET((short)28229) ;
        p34.chan6_scaled_SET((short) -26925) ;
        p34.chan4_scaled_SET((short)30172) ;
        p34.rssi_SET((char)198) ;
        p34.chan2_scaled_SET((short)10008) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)27968);
            assert(pack.time_boot_ms_GET() == 491276325L);
            assert(pack.chan5_raw_GET() == (char)50426);
            assert(pack.chan6_raw_GET() == (char)3754);
            assert(pack.rssi_GET() == (char)189);
            assert(pack.port_GET() == (char)28);
            assert(pack.chan7_raw_GET() == (char)9205);
            assert(pack.chan4_raw_GET() == (char)14063);
            assert(pack.chan1_raw_GET() == (char)41044);
            assert(pack.chan8_raw_GET() == (char)19143);
            assert(pack.chan2_raw_GET() == (char)53577);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan1_raw_SET((char)41044) ;
        p35.rssi_SET((char)189) ;
        p35.chan4_raw_SET((char)14063) ;
        p35.port_SET((char)28) ;
        p35.chan6_raw_SET((char)3754) ;
        p35.chan8_raw_SET((char)19143) ;
        p35.chan3_raw_SET((char)27968) ;
        p35.chan5_raw_SET((char)50426) ;
        p35.chan2_raw_SET((char)53577) ;
        p35.time_boot_ms_SET(491276325L) ;
        p35.chan7_raw_SET((char)9205) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo3_raw_GET() == (char)59744);
            assert(pack.port_GET() == (char)182);
            assert(pack.servo11_raw_TRY(ph) == (char)2151);
            assert(pack.servo7_raw_GET() == (char)58078);
            assert(pack.servo6_raw_GET() == (char)49982);
            assert(pack.servo14_raw_TRY(ph) == (char)35216);
            assert(pack.servo13_raw_TRY(ph) == (char)35756);
            assert(pack.servo5_raw_GET() == (char)24711);
            assert(pack.servo9_raw_TRY(ph) == (char)41114);
            assert(pack.servo2_raw_GET() == (char)30433);
            assert(pack.servo1_raw_GET() == (char)65248);
            assert(pack.time_usec_GET() == 3507223298L);
            assert(pack.servo12_raw_TRY(ph) == (char)55794);
            assert(pack.servo16_raw_TRY(ph) == (char)27689);
            assert(pack.servo15_raw_TRY(ph) == (char)51752);
            assert(pack.servo8_raw_GET() == (char)8491);
            assert(pack.servo10_raw_TRY(ph) == (char)21221);
            assert(pack.servo4_raw_GET() == (char)47151);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo14_raw_SET((char)35216, PH) ;
        p36.servo1_raw_SET((char)65248) ;
        p36.servo5_raw_SET((char)24711) ;
        p36.servo8_raw_SET((char)8491) ;
        p36.servo12_raw_SET((char)55794, PH) ;
        p36.servo7_raw_SET((char)58078) ;
        p36.servo11_raw_SET((char)2151, PH) ;
        p36.servo16_raw_SET((char)27689, PH) ;
        p36.time_usec_SET(3507223298L) ;
        p36.servo2_raw_SET((char)30433) ;
        p36.port_SET((char)182) ;
        p36.servo9_raw_SET((char)41114, PH) ;
        p36.servo10_raw_SET((char)21221, PH) ;
        p36.servo3_raw_SET((char)59744) ;
        p36.servo13_raw_SET((char)35756, PH) ;
        p36.servo15_raw_SET((char)51752, PH) ;
        p36.servo4_raw_SET((char)47151) ;
        p36.servo6_raw_SET((char)49982) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)206);
            assert(pack.end_index_GET() == (short) -24457);
            assert(pack.target_system_GET() == (char)225);
            assert(pack.start_index_GET() == (short)29098);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.end_index_SET((short) -24457) ;
        p37.start_index_SET((short)29098) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.target_component_SET((char)206) ;
        p37.target_system_SET((char)225) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.start_index_GET() == (short) -23868);
            assert(pack.target_component_GET() == (char)231);
            assert(pack.target_system_GET() == (char)188);
            assert(pack.end_index_GET() == (short)1168);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)188) ;
        p38.end_index_SET((short)1168) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_component_SET((char)231) ;
        p38.start_index_SET((short) -23868) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION);
            assert(pack.target_system_GET() == (char)46);
            assert(pack.autocontinue_GET() == (char)157);
            assert(pack.param2_GET() == 1.2797489E38F);
            assert(pack.x_GET() == 4.1008144E35F);
            assert(pack.param1_GET() == 2.5870278E38F);
            assert(pack.z_GET() == 3.0691121E38F);
            assert(pack.param3_GET() == 2.5122216E38F);
            assert(pack.seq_GET() == (char)42930);
            assert(pack.param4_GET() == 2.4043488E38F);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.current_GET() == (char)124);
            assert(pack.y_GET() == 2.4730545E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.seq_SET((char)42930) ;
        p39.param3_SET(2.5122216E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION) ;
        p39.autocontinue_SET((char)157) ;
        p39.current_SET((char)124) ;
        p39.param2_SET(1.2797489E38F) ;
        p39.y_SET(2.4730545E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.param1_SET(2.5870278E38F) ;
        p39.target_system_SET((char)46) ;
        p39.target_component_SET((char)190) ;
        p39.param4_SET(2.4043488E38F) ;
        p39.z_SET(3.0691121E38F) ;
        p39.x_SET(4.1008144E35F) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)32593);
            assert(pack.target_component_GET() == (char)235);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)236);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)236) ;
        p40.target_component_SET((char)235) ;
        p40.seq_SET((char)32593) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)93);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.seq_GET() == (char)47232);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)47232) ;
        p41.target_component_SET((char)205) ;
        p41.target_system_SET((char)93) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)39353);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)39353) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)65);
            assert(pack.target_component_GET() == (char)118);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_component_SET((char)118) ;
        p43.target_system_SET((char)65) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.count_GET() == (char)29946);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.target_system_GET() == (char)149);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)46) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)29946) ;
        p44.target_system_SET((char)149) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)165);
            assert(pack.target_system_GET() == (char)22);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_component_SET((char)165) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_system_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)64773);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)64773) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)225);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)6);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p47.target_system_SET((char)225) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4) ;
        p47.target_component_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -1448218896);
            assert(pack.altitude_GET() == -1341685310);
            assert(pack.time_usec_TRY(ph) == 202945601902750384L);
            assert(pack.target_system_GET() == (char)236);
            assert(pack.latitude_GET() == -364811944);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.altitude_SET(-1341685310) ;
        p48.time_usec_SET(202945601902750384L, PH) ;
        p48.latitude_SET(-364811944) ;
        p48.target_system_SET((char)236) ;
        p48.longitude_SET(-1448218896) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -452742361);
            assert(pack.longitude_GET() == -879220881);
            assert(pack.time_usec_TRY(ph) == 6567242358394122598L);
            assert(pack.altitude_GET() == -1554104883);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.altitude_SET(-1554104883) ;
        p49.time_usec_SET(6567242358394122598L, PH) ;
        p49.latitude_SET(-452742361) ;
        p49.longitude_SET(-879220881) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("enchkSmv"));
            assert(pack.target_component_GET() == (char)68);
            assert(pack.param_value0_GET() == -1.0790343E38F);
            assert(pack.scale_GET() == 1.5021837E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)157);
            assert(pack.param_index_GET() == (short)13921);
            assert(pack.param_value_max_GET() == -3.2214474E38F);
            assert(pack.target_system_GET() == (char)217);
            assert(pack.param_value_min_GET() == -3.0895706E38F);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)68) ;
        p50.param_value_max_SET(-3.2214474E38F) ;
        p50.param_id_SET("enchkSmv", PH) ;
        p50.param_value0_SET(-1.0790343E38F) ;
        p50.target_system_SET((char)217) ;
        p50.scale_SET(1.5021837E38F) ;
        p50.param_value_min_SET(-3.0895706E38F) ;
        p50.param_index_SET((short)13921) ;
        p50.parameter_rc_channel_index_SET((char)157) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.seq_GET() == (char)58521);
            assert(pack.target_system_GET() == (char)148);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)18) ;
        p51.target_system_SET((char)148) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p51.seq_SET((char)58521) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)127);
            assert(pack.p1x_GET() == 2.767067E38F);
            assert(pack.target_system_GET() == (char)61);
            assert(pack.p2z_GET() == 1.2164756E38F);
            assert(pack.p2x_GET() == 2.6973994E38F);
            assert(pack.p1z_GET() == 2.1169335E38F);
            assert(pack.p2y_GET() == -2.6128593E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p1y_GET() == -8.2749153E37F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p54.target_component_SET((char)127) ;
        p54.p1z_SET(2.1169335E38F) ;
        p54.target_system_SET((char)61) ;
        p54.p2y_SET(-2.6128593E38F) ;
        p54.p2x_SET(2.6973994E38F) ;
        p54.p1x_SET(2.767067E38F) ;
        p54.p2z_SET(1.2164756E38F) ;
        p54.p1y_SET(-8.2749153E37F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == -2.4388062E38F);
            assert(pack.p1z_GET() == 1.9888241E38F);
            assert(pack.p1y_GET() == 2.6010787E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.p2x_GET() == 7.058757E37F);
            assert(pack.p1x_GET() == 2.0162534E38F);
            assert(pack.p2z_GET() == 2.2279381E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(2.2279381E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p55.p1x_SET(2.0162534E38F) ;
        p55.p1y_SET(2.6010787E38F) ;
        p55.p1z_SET(1.9888241E38F) ;
        p55.p2x_SET(7.058757E37F) ;
        p55.p2y_SET(-2.4388062E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.1620051E38F, 1.6666879E38F, 1.7018643E38F, -1.6643904E37F, 3.0202473E38F, -1.8724128E38F, -9.573807E37F, -3.3487814E38F, 9.2828596E36F}));
            assert(pack.time_usec_GET() == 5614948497319649725L);
            assert(pack.rollspeed_GET() == 1.2344089E38F);
            assert(pack.pitchspeed_GET() == 1.0620318E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.8191272E38F, 2.3227612E38F, -2.5904162E38F, 2.8695158E38F}));
            assert(pack.yawspeed_GET() == 2.558576E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(5614948497319649725L) ;
        p61.yawspeed_SET(2.558576E38F) ;
        p61.q_SET(new float[] {-2.8191272E38F, 2.3227612E38F, -2.5904162E38F, 2.8695158E38F}, 0) ;
        p61.pitchspeed_SET(1.0620318E38F) ;
        p61.rollspeed_SET(1.2344089E38F) ;
        p61.covariance_SET(new float[] {1.1620051E38F, 1.6666879E38F, 1.7018643E38F, -1.6643904E37F, 3.0202473E38F, -1.8724128E38F, -9.573807E37F, -3.3487814E38F, 9.2828596E36F}, 0) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.wp_dist_GET() == (char)27522);
            assert(pack.nav_bearing_GET() == (short) -30000);
            assert(pack.xtrack_error_GET() == -2.1151515E38F);
            assert(pack.target_bearing_GET() == (short)23962);
            assert(pack.nav_roll_GET() == 1.6072868E37F);
            assert(pack.nav_pitch_GET() == -1.2663983E38F);
            assert(pack.alt_error_GET() == -9.556709E37F);
            assert(pack.aspd_error_GET() == -1.9578618E38F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.aspd_error_SET(-1.9578618E38F) ;
        p62.wp_dist_SET((char)27522) ;
        p62.nav_bearing_SET((short) -30000) ;
        p62.target_bearing_SET((short)23962) ;
        p62.alt_error_SET(-9.556709E37F) ;
        p62.xtrack_error_SET(-2.1151515E38F) ;
        p62.nav_roll_SET(1.6072868E37F) ;
        p62.nav_pitch_SET(-1.2663983E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1707394791);
            assert(pack.vz_GET() == -1.9835217E38F);
            assert(pack.vy_GET() == 1.2051297E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {6.928553E37F, 1.0200642E37F, 1.0110879E38F, -7.242283E37F, 4.909863E37F, -3.146449E38F, -1.7771926E38F, 1.7434328E36F, 5.528796E37F, 9.846485E37F, -3.1631523E38F, -1.9627482E38F, -2.5053816E38F, 2.727451E38F, 1.696831E38F, -1.5339994E38F, -3.020971E38F, -8.1033784E37F, -6.446451E37F, 4.984628E37F, 2.6105794E38F, 2.0926119E38F, 2.6559179E38F, -3.3318234E37F, 1.5131913E38F, 2.2481303E38F, -4.315032E37F, -2.117617E38F, 9.970383E35F, -6.34918E37F, -4.8654063E37F, 2.8543382E38F, 9.825504E37F, 3.3084695E38F, 1.0577179E38F, -1.2667719E38F}));
            assert(pack.time_usec_GET() == 2729712459339268953L);
            assert(pack.relative_alt_GET() == 453805068);
            assert(pack.lon_GET() == -1324152340);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vx_GET() == 3.2354934E38F);
            assert(pack.lat_GET() == -1754730359);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lat_SET(-1754730359) ;
        p63.alt_SET(-1707394791) ;
        p63.lon_SET(-1324152340) ;
        p63.covariance_SET(new float[] {6.928553E37F, 1.0200642E37F, 1.0110879E38F, -7.242283E37F, 4.909863E37F, -3.146449E38F, -1.7771926E38F, 1.7434328E36F, 5.528796E37F, 9.846485E37F, -3.1631523E38F, -1.9627482E38F, -2.5053816E38F, 2.727451E38F, 1.696831E38F, -1.5339994E38F, -3.020971E38F, -8.1033784E37F, -6.446451E37F, 4.984628E37F, 2.6105794E38F, 2.0926119E38F, 2.6559179E38F, -3.3318234E37F, 1.5131913E38F, 2.2481303E38F, -4.315032E37F, -2.117617E38F, 9.970383E35F, -6.34918E37F, -4.8654063E37F, 2.8543382E38F, 9.825504E37F, 3.3084695E38F, 1.0577179E38F, -1.2667719E38F}, 0) ;
        p63.vx_SET(3.2354934E38F) ;
        p63.vy_SET(1.2051297E38F) ;
        p63.time_usec_SET(2729712459339268953L) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.relative_alt_SET(453805068) ;
        p63.vz_SET(-1.9835217E38F) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.ax_GET() == -2.3289396E38F);
            assert(pack.vx_GET() == -2.875459E38F);
            assert(pack.vy_GET() == -3.3960025E38F);
            assert(pack.z_GET() == -8.28699E37F);
            assert(pack.time_usec_GET() == 4677166975870155217L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            assert(pack.vz_GET() == 1.1732536E38F);
            assert(pack.ay_GET() == 2.2469435E38F);
            assert(pack.az_GET() == -1.9053133E37F);
            assert(pack.y_GET() == -3.3527884E38F);
            assert(pack.x_GET() == 5.8912617E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {6.909505E37F, -1.1737767E38F, -1.6884129E38F, -1.5924509E38F, -1.7449038E38F, 2.5454913E38F, 2.8898583E37F, -3.3272263E38F, 3.1754522E38F, -2.0894031E37F, -1.2926332E38F, -1.5076264E38F, 8.438833E37F, 3.115095E38F, -1.3670682E38F, -1.8287766E38F, 1.9256499E38F, 2.3419145E38F, -8.651411E37F, 2.1831661E38F, -2.0789809E38F, 2.1270982E38F, -1.6432141E38F, 2.7876515E38F, -1.0985225E38F, 4.394826E37F, 2.1326627E38F, -2.0107022E38F, 1.44731E38F, -1.0905718E38F, 8.17983E37F, -1.911386E38F, 1.8660127E38F, 3.3114676E38F, 1.1692476E37F, -5.3639882E34F, 4.837401E37F, -1.2909747E38F, -3.2926385E38F, 2.633525E38F, -8.653147E37F, 2.4504309E38F, -3.0010084E38F, -3.1603144E38F, -3.3369683E37F}));
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.z_SET(-8.28699E37F) ;
        p64.ay_SET(2.2469435E38F) ;
        p64.vy_SET(-3.3960025E38F) ;
        p64.y_SET(-3.3527884E38F) ;
        p64.time_usec_SET(4677166975870155217L) ;
        p64.covariance_SET(new float[] {6.909505E37F, -1.1737767E38F, -1.6884129E38F, -1.5924509E38F, -1.7449038E38F, 2.5454913E38F, 2.8898583E37F, -3.3272263E38F, 3.1754522E38F, -2.0894031E37F, -1.2926332E38F, -1.5076264E38F, 8.438833E37F, 3.115095E38F, -1.3670682E38F, -1.8287766E38F, 1.9256499E38F, 2.3419145E38F, -8.651411E37F, 2.1831661E38F, -2.0789809E38F, 2.1270982E38F, -1.6432141E38F, 2.7876515E38F, -1.0985225E38F, 4.394826E37F, 2.1326627E38F, -2.0107022E38F, 1.44731E38F, -1.0905718E38F, 8.17983E37F, -1.911386E38F, 1.8660127E38F, 3.3114676E38F, 1.1692476E37F, -5.3639882E34F, 4.837401E37F, -1.2909747E38F, -3.2926385E38F, 2.633525E38F, -8.653147E37F, 2.4504309E38F, -3.0010084E38F, -3.1603144E38F, -3.3369683E37F}, 0) ;
        p64.x_SET(5.8912617E37F) ;
        p64.az_SET(-1.9053133E37F) ;
        p64.vz_SET(1.1732536E38F) ;
        p64.vx_SET(-2.875459E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE) ;
        p64.ax_SET(-2.3289396E38F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)52920);
            assert(pack.chan5_raw_GET() == (char)524);
            assert(pack.chan15_raw_GET() == (char)60362);
            assert(pack.chancount_GET() == (char)0);
            assert(pack.chan10_raw_GET() == (char)23915);
            assert(pack.chan14_raw_GET() == (char)3511);
            assert(pack.rssi_GET() == (char)189);
            assert(pack.chan9_raw_GET() == (char)31529);
            assert(pack.chan2_raw_GET() == (char)15594);
            assert(pack.time_boot_ms_GET() == 2208132646L);
            assert(pack.chan17_raw_GET() == (char)30056);
            assert(pack.chan4_raw_GET() == (char)2424);
            assert(pack.chan13_raw_GET() == (char)35670);
            assert(pack.chan18_raw_GET() == (char)11015);
            assert(pack.chan3_raw_GET() == (char)58868);
            assert(pack.chan16_raw_GET() == (char)31151);
            assert(pack.chan12_raw_GET() == (char)35180);
            assert(pack.chan7_raw_GET() == (char)51334);
            assert(pack.chan8_raw_GET() == (char)3101);
            assert(pack.chan6_raw_GET() == (char)578);
            assert(pack.chan1_raw_GET() == (char)64799);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan14_raw_SET((char)3511) ;
        p65.chan17_raw_SET((char)30056) ;
        p65.chan7_raw_SET((char)51334) ;
        p65.chan12_raw_SET((char)35180) ;
        p65.chancount_SET((char)0) ;
        p65.chan16_raw_SET((char)31151) ;
        p65.chan9_raw_SET((char)31529) ;
        p65.chan6_raw_SET((char)578) ;
        p65.chan10_raw_SET((char)23915) ;
        p65.rssi_SET((char)189) ;
        p65.chan1_raw_SET((char)64799) ;
        p65.time_boot_ms_SET(2208132646L) ;
        p65.chan3_raw_SET((char)58868) ;
        p65.chan18_raw_SET((char)11015) ;
        p65.chan2_raw_SET((char)15594) ;
        p65.chan8_raw_SET((char)3101) ;
        p65.chan11_raw_SET((char)52920) ;
        p65.chan5_raw_SET((char)524) ;
        p65.chan13_raw_SET((char)35670) ;
        p65.chan15_raw_SET((char)60362) ;
        p65.chan4_raw_SET((char)2424) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)68);
            assert(pack.start_stop_GET() == (char)102);
            assert(pack.target_component_GET() == (char)26);
            assert(pack.req_stream_id_GET() == (char)133);
            assert(pack.req_message_rate_GET() == (char)43002);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.start_stop_SET((char)102) ;
        p66.target_system_SET((char)68) ;
        p66.req_stream_id_SET((char)133) ;
        p66.target_component_SET((char)26) ;
        p66.req_message_rate_SET((char)43002) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)10);
            assert(pack.on_off_GET() == (char)136);
            assert(pack.message_rate_GET() == (char)50470);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)50470) ;
        p67.stream_id_SET((char)10) ;
        p67.on_off_SET((char)136) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short) -25554);
            assert(pack.buttons_GET() == (char)17894);
            assert(pack.y_GET() == (short)1167);
            assert(pack.target_GET() == (char)128);
            assert(pack.z_GET() == (short)1981);
            assert(pack.r_GET() == (short)17466);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.x_SET((short) -25554) ;
        p69.target_SET((char)128) ;
        p69.y_SET((short)1167) ;
        p69.buttons_SET((char)17894) ;
        p69.r_SET((short)17466) ;
        p69.z_SET((short)1981) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)54660);
            assert(pack.target_system_GET() == (char)33);
            assert(pack.target_component_GET() == (char)241);
            assert(pack.chan4_raw_GET() == (char)14227);
            assert(pack.chan2_raw_GET() == (char)30968);
            assert(pack.chan8_raw_GET() == (char)12);
            assert(pack.chan3_raw_GET() == (char)60555);
            assert(pack.chan5_raw_GET() == (char)39005);
            assert(pack.chan7_raw_GET() == (char)17681);
            assert(pack.chan6_raw_GET() == (char)1565);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)14227) ;
        p70.chan5_raw_SET((char)39005) ;
        p70.chan3_raw_SET((char)60555) ;
        p70.target_component_SET((char)241) ;
        p70.chan8_raw_SET((char)12) ;
        p70.chan2_raw_SET((char)30968) ;
        p70.target_system_SET((char)33) ;
        p70.chan1_raw_SET((char)54660) ;
        p70.chan7_raw_SET((char)17681) ;
        p70.chan6_raw_SET((char)1565) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -370283623);
            assert(pack.param3_GET() == -3.236069E38F);
            assert(pack.autocontinue_GET() == (char)147);
            assert(pack.x_GET() == 363400383);
            assert(pack.param1_GET() == 3.503392E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.param2_GET() == 1.795431E38F);
            assert(pack.current_GET() == (char)172);
            assert(pack.z_GET() == 1.8954524E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE);
            assert(pack.target_component_GET() == (char)20);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)9501);
            assert(pack.param4_GET() == -7.0140005E37F);
            assert(pack.target_system_GET() == (char)182);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_component_SET((char)20) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.z_SET(1.8954524E38F) ;
        p73.y_SET(-370283623) ;
        p73.x_SET(363400383) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p73.param3_SET(-3.236069E38F) ;
        p73.param1_SET(3.503392E37F) ;
        p73.param4_SET(-7.0140005E37F) ;
        p73.autocontinue_SET((char)147) ;
        p73.current_SET((char)172) ;
        p73.target_system_SET((char)182) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE) ;
        p73.seq_SET((char)9501) ;
        p73.param2_SET(1.795431E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == 2.4465214E38F);
            assert(pack.alt_GET() == 1.8053229E38F);
            assert(pack.airspeed_GET() == 1.3582537E38F);
            assert(pack.climb_GET() == 2.6293457E38F);
            assert(pack.heading_GET() == (short) -24501);
            assert(pack.throttle_GET() == (char)20372);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.heading_SET((short) -24501) ;
        p74.climb_SET(2.6293457E38F) ;
        p74.throttle_SET((char)20372) ;
        p74.groundspeed_SET(2.4465214E38F) ;
        p74.airspeed_SET(1.3582537E38F) ;
        p74.alt_SET(1.8053229E38F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 7099567);
            assert(pack.current_GET() == (char)192);
            assert(pack.param2_GET() == 2.3735072E38F);
            assert(pack.x_GET() == 1969394204);
            assert(pack.target_system_GET() == (char)48);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_RALLY_LAND);
            assert(pack.autocontinue_GET() == (char)30);
            assert(pack.param3_GET() == 2.533073E38F);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.param1_GET() == -1.8528395E38F);
            assert(pack.z_GET() == -2.0408783E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.param4_GET() == 1.9204988E38F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_component_SET((char)103) ;
        p75.param1_SET(-1.8528395E38F) ;
        p75.param3_SET(2.533073E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND) ;
        p75.autocontinue_SET((char)30) ;
        p75.param4_SET(1.9204988E38F) ;
        p75.y_SET(7099567) ;
        p75.z_SET(-2.0408783E38F) ;
        p75.current_SET((char)192) ;
        p75.param2_SET(2.3735072E38F) ;
        p75.x_SET(1969394204) ;
        p75.target_system_SET((char)48) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
            assert(pack.param3_GET() == -2.2776844E37F);
            assert(pack.param6_GET() == -2.7246695E38F);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.confirmation_GET() == (char)252);
            assert(pack.param5_GET() == 2.2954512E38F);
            assert(pack.param2_GET() == 7.9398535E37F);
            assert(pack.param1_GET() == -2.4218988E38F);
            assert(pack.param7_GET() == -2.2391713E38F);
            assert(pack.param4_GET() == 7.5591917E37F);
            assert(pack.target_component_GET() == (char)102);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param3_SET(-2.2776844E37F) ;
        p76.param5_SET(2.2954512E38F) ;
        p76.target_component_SET((char)102) ;
        p76.param1_SET(-2.4218988E38F) ;
        p76.target_system_SET((char)139) ;
        p76.param4_SET(7.5591917E37F) ;
        p76.param2_SET(7.9398535E37F) ;
        p76.param7_SET(-2.2391713E38F) ;
        p76.confirmation_SET((char)252) ;
        p76.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE) ;
        p76.param6_SET(-2.7246695E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAND_START);
            assert(pack.target_system_TRY(ph) == (char)179);
            assert(pack.progress_TRY(ph) == (char)252);
            assert(pack.result_param2_TRY(ph) == -861269372);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_component_TRY(ph) == (char)231);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.progress_SET((char)252, PH) ;
        p77.result_param2_SET(-861269372, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.target_system_SET((char)179, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_LAND_START) ;
        p77.target_component_SET((char)231, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.9686291E38F);
            assert(pack.mode_switch_GET() == (char)62);
            assert(pack.pitch_GET() == 1.7787645E38F);
            assert(pack.manual_override_switch_GET() == (char)146);
            assert(pack.thrust_GET() == -2.3999171E38F);
            assert(pack.roll_GET() == -6.8119086E37F);
            assert(pack.time_boot_ms_GET() == 3876113809L);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(-6.8119086E37F) ;
        p81.pitch_SET(1.7787645E38F) ;
        p81.mode_switch_SET((char)62) ;
        p81.thrust_SET(-2.3999171E38F) ;
        p81.time_boot_ms_SET(3876113809L) ;
        p81.yaw_SET(1.9686291E38F) ;
        p81.manual_override_switch_SET((char)146) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_yaw_rate_GET() == 1.008228E38F);
            assert(pack.body_roll_rate_GET() == 3.1546071E38F);
            assert(pack.body_pitch_rate_GET() == -4.037294E37F);
            assert(pack.time_boot_ms_GET() == 2281323988L);
            assert(pack.target_component_GET() == (char)44);
            assert(pack.target_system_GET() == (char)190);
            assert(pack.type_mask_GET() == (char)251);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-5.1931705E37F, -2.722963E38F, -1.778818E38F, 1.9820934E38F}));
            assert(pack.thrust_GET() == -2.1318766E38F);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(1.008228E38F) ;
        p82.thrust_SET(-2.1318766E38F) ;
        p82.body_pitch_rate_SET(-4.037294E37F) ;
        p82.body_roll_rate_SET(3.1546071E38F) ;
        p82.target_system_SET((char)190) ;
        p82.time_boot_ms_SET(2281323988L) ;
        p82.q_SET(new float[] {-5.1931705E37F, -2.722963E38F, -1.778818E38F, 1.9820934E38F}, 0) ;
        p82.type_mask_SET((char)251) ;
        p82.target_component_SET((char)44) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -1.3113836E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5408793E38F, -2.0547653E38F, 1.2749614E38F, -1.1331615E38F}));
            assert(pack.type_mask_GET() == (char)182);
            assert(pack.body_yaw_rate_GET() == -3.8884026E37F);
            assert(pack.time_boot_ms_GET() == 522330119L);
            assert(pack.body_roll_rate_GET() == 6.143263E37F);
            assert(pack.body_pitch_rate_GET() == 1.5542977E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(1.5542977E38F) ;
        p83.time_boot_ms_SET(522330119L) ;
        p83.q_SET(new float[] {-1.5408793E38F, -2.0547653E38F, 1.2749614E38F, -1.1331615E38F}, 0) ;
        p83.body_roll_rate_SET(6.143263E37F) ;
        p83.thrust_SET(-1.3113836E38F) ;
        p83.type_mask_SET((char)182) ;
        p83.body_yaw_rate_SET(-3.8884026E37F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.5941961E38F);
            assert(pack.x_GET() == -1.6196024E38F);
            assert(pack.yaw_GET() == 3.1623903E38F);
            assert(pack.afy_GET() == 1.0688422E38F);
            assert(pack.afx_GET() == -1.6377428E38F);
            assert(pack.time_boot_ms_GET() == 3053146775L);
            assert(pack.vz_GET() == -1.954303E38F);
            assert(pack.target_component_GET() == (char)248);
            assert(pack.afz_GET() == 2.950081E38F);
            assert(pack.yaw_rate_GET() == 2.9961833E38F);
            assert(pack.target_system_GET() == (char)176);
            assert(pack.type_mask_GET() == (char)63793);
            assert(pack.vx_GET() == 2.4766787E38F);
            assert(pack.vy_GET() == 2.1055192E38F);
            assert(pack.y_GET() == 2.001843E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.type_mask_SET((char)63793) ;
        p84.yaw_SET(3.1623903E38F) ;
        p84.vz_SET(-1.954303E38F) ;
        p84.yaw_rate_SET(2.9961833E38F) ;
        p84.x_SET(-1.6196024E38F) ;
        p84.afy_SET(1.0688422E38F) ;
        p84.vx_SET(2.4766787E38F) ;
        p84.target_component_SET((char)248) ;
        p84.vy_SET(2.1055192E38F) ;
        p84.afz_SET(2.950081E38F) ;
        p84.time_boot_ms_SET(3053146775L) ;
        p84.afx_SET(-1.6377428E38F) ;
        p84.y_SET(2.001843E37F) ;
        p84.target_system_SET((char)176) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p84.z_SET(-1.5941961E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.1586516E38F);
            assert(pack.afz_GET() == 4.4975895E36F);
            assert(pack.vz_GET() == 2.6976805E38F);
            assert(pack.target_component_GET() == (char)21);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.lon_int_GET() == 1115550277);
            assert(pack.yaw_GET() == -3.0574412E38F);
            assert(pack.yaw_rate_GET() == -2.4081159E38F);
            assert(pack.alt_GET() == 2.0043243E38F);
            assert(pack.afx_GET() == -2.0330832E38F);
            assert(pack.vx_GET() == -1.6868899E38F);
            assert(pack.afy_GET() == -1.4094438E38F);
            assert(pack.type_mask_GET() == (char)42883);
            assert(pack.lat_int_GET() == 1570508282);
            assert(pack.target_system_GET() == (char)243);
            assert(pack.time_boot_ms_GET() == 3821564714L);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.alt_SET(2.0043243E38F) ;
        p86.vx_SET(-1.6868899E38F) ;
        p86.yaw_rate_SET(-2.4081159E38F) ;
        p86.afz_SET(4.4975895E36F) ;
        p86.type_mask_SET((char)42883) ;
        p86.afy_SET(-1.4094438E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p86.lon_int_SET(1115550277) ;
        p86.target_system_SET((char)243) ;
        p86.afx_SET(-2.0330832E38F) ;
        p86.target_component_SET((char)21) ;
        p86.yaw_SET(-3.0574412E38F) ;
        p86.time_boot_ms_SET(3821564714L) ;
        p86.lat_int_SET(1570508282) ;
        p86.vy_SET(2.1586516E38F) ;
        p86.vz_SET(2.6976805E38F) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 2.0400395E37F);
            assert(pack.afz_GET() == 2.4846673E36F);
            assert(pack.yaw_GET() == 9.202785E37F);
            assert(pack.vx_GET() == -1.2560832E38F);
            assert(pack.lat_int_GET() == 957610859);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.lon_int_GET() == -973564267);
            assert(pack.yaw_rate_GET() == -2.4601322E38F);
            assert(pack.alt_GET() == -1.3922363E37F);
            assert(pack.afx_GET() == -7.3918973E37F);
            assert(pack.vz_GET() == -1.5856343E36F);
            assert(pack.afy_GET() == 3.3135411E38F);
            assert(pack.type_mask_GET() == (char)37364);
            assert(pack.time_boot_ms_GET() == 1435039351L);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.yaw_rate_SET(-2.4601322E38F) ;
        p87.lon_int_SET(-973564267) ;
        p87.afy_SET(3.3135411E38F) ;
        p87.vx_SET(-1.2560832E38F) ;
        p87.lat_int_SET(957610859) ;
        p87.alt_SET(-1.3922363E37F) ;
        p87.afx_SET(-7.3918973E37F) ;
        p87.afz_SET(2.4846673E36F) ;
        p87.vy_SET(2.0400395E37F) ;
        p87.time_boot_ms_SET(1435039351L) ;
        p87.yaw_SET(9.202785E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p87.type_mask_SET((char)37364) ;
        p87.vz_SET(-1.5856343E36F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.388442E38F);
            assert(pack.roll_GET() == 3.3143857E38F);
            assert(pack.x_GET() == 1.1104843E38F);
            assert(pack.time_boot_ms_GET() == 2724471309L);
            assert(pack.yaw_GET() == 5.8000304E37F);
            assert(pack.y_GET() == -1.3681625E38F);
            assert(pack.z_GET() == -8.711266E37F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(2724471309L) ;
        p89.x_SET(1.1104843E38F) ;
        p89.y_SET(-1.3681625E38F) ;
        p89.pitch_SET(-2.388442E38F) ;
        p89.roll_SET(3.3143857E38F) ;
        p89.z_SET(-8.711266E37F) ;
        p89.yaw_SET(5.8000304E37F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -664397331);
            assert(pack.yaw_GET() == -1.0015328E38F);
            assert(pack.yawspeed_GET() == 4.4147443E37F);
            assert(pack.roll_GET() == -2.7831953E37F);
            assert(pack.vy_GET() == (short) -24970);
            assert(pack.lat_GET() == -1583816196);
            assert(pack.time_usec_GET() == 5214085859928203066L);
            assert(pack.vx_GET() == (short)8810);
            assert(pack.vz_GET() == (short) -12799);
            assert(pack.pitchspeed_GET() == 1.6882449E38F);
            assert(pack.yacc_GET() == (short) -5083);
            assert(pack.zacc_GET() == (short)15869);
            assert(pack.rollspeed_GET() == -4.6386E37F);
            assert(pack.pitch_GET() == -5.559471E37F);
            assert(pack.xacc_GET() == (short)32099);
            assert(pack.alt_GET() == 788359862);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lon_SET(-664397331) ;
        p90.pitchspeed_SET(1.6882449E38F) ;
        p90.roll_SET(-2.7831953E37F) ;
        p90.alt_SET(788359862) ;
        p90.time_usec_SET(5214085859928203066L) ;
        p90.xacc_SET((short)32099) ;
        p90.vy_SET((short) -24970) ;
        p90.vz_SET((short) -12799) ;
        p90.yawspeed_SET(4.4147443E37F) ;
        p90.lat_SET(-1583816196) ;
        p90.yacc_SET((short) -5083) ;
        p90.rollspeed_SET(-4.6386E37F) ;
        p90.yaw_SET(-1.0015328E38F) ;
        p90.zacc_SET((short)15869) ;
        p90.vx_SET((short)8810) ;
        p90.pitch_SET(-5.559471E37F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == -2.0359186E37F);
            assert(pack.yaw_rudder_GET() == 7.81559E37F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.aux1_GET() == -2.1844131E36F);
            assert(pack.aux4_GET() == -2.8388579E38F);
            assert(pack.nav_mode_GET() == (char)60);
            assert(pack.aux3_GET() == -2.9330833E38F);
            assert(pack.throttle_GET() == -2.0193512E37F);
            assert(pack.pitch_elevator_GET() == -1.002213E38F);
            assert(pack.roll_ailerons_GET() == 1.2333319E38F);
            assert(pack.time_usec_GET() == 5554338347061419750L);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux2_SET(-2.0359186E37F) ;
        p91.aux4_SET(-2.8388579E38F) ;
        p91.nav_mode_SET((char)60) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p91.yaw_rudder_SET(7.81559E37F) ;
        p91.roll_ailerons_SET(1.2333319E38F) ;
        p91.aux3_SET(-2.9330833E38F) ;
        p91.aux1_SET(-2.1844131E36F) ;
        p91.throttle_SET(-2.0193512E37F) ;
        p91.pitch_elevator_SET(-1.002213E38F) ;
        p91.time_usec_SET(5554338347061419750L) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 421430616187144436L);
            assert(pack.chan5_raw_GET() == (char)33594);
            assert(pack.chan7_raw_GET() == (char)38701);
            assert(pack.chan2_raw_GET() == (char)16056);
            assert(pack.chan10_raw_GET() == (char)28706);
            assert(pack.chan12_raw_GET() == (char)47800);
            assert(pack.chan9_raw_GET() == (char)32687);
            assert(pack.chan1_raw_GET() == (char)15434);
            assert(pack.chan3_raw_GET() == (char)64985);
            assert(pack.chan4_raw_GET() == (char)35407);
            assert(pack.chan8_raw_GET() == (char)9404);
            assert(pack.rssi_GET() == (char)47);
            assert(pack.chan11_raw_GET() == (char)53860);
            assert(pack.chan6_raw_GET() == (char)1866);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan8_raw_SET((char)9404) ;
        p92.chan9_raw_SET((char)32687) ;
        p92.chan10_raw_SET((char)28706) ;
        p92.rssi_SET((char)47) ;
        p92.chan12_raw_SET((char)47800) ;
        p92.chan5_raw_SET((char)33594) ;
        p92.chan2_raw_SET((char)16056) ;
        p92.time_usec_SET(421430616187144436L) ;
        p92.chan7_raw_SET((char)38701) ;
        p92.chan4_raw_SET((char)35407) ;
        p92.chan1_raw_SET((char)15434) ;
        p92.chan11_raw_SET((char)53860) ;
        p92.chan6_raw_SET((char)1866) ;
        p92.chan3_raw_SET((char)64985) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.8746768E38F, -3.0346612E38F, 5.9398877E37F, -1.122555E38F, 4.769814E37F, -2.3850728E38F, 2.8682404E38F, 2.5981193E38F, 1.5987366E38F, 1.8432603E38F, 2.177174E38F, -9.059028E37F, -2.0000995E38F, -1.3984992E36F, -1.9836687E38F, -2.4651665E38F}));
            assert(pack.flags_GET() == 1131110551864381137L);
            assert(pack.time_usec_GET() == 7385442823142817168L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {2.8746768E38F, -3.0346612E38F, 5.9398877E37F, -1.122555E38F, 4.769814E37F, -2.3850728E38F, 2.8682404E38F, 2.5981193E38F, 1.5987366E38F, 1.8432603E38F, 2.177174E38F, -9.059028E37F, -2.0000995E38F, -1.3984992E36F, -1.9836687E38F, -2.4651665E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p93.flags_SET(1131110551864381137L) ;
        p93.time_usec_SET(7385442823142817168L) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)164);
            assert(pack.flow_rate_y_TRY(ph) == 2.838496E38F);
            assert(pack.flow_comp_m_x_GET() == 3.1624824E38F);
            assert(pack.flow_rate_x_TRY(ph) == 6.655559E36F);
            assert(pack.flow_y_GET() == (short)2614);
            assert(pack.quality_GET() == (char)118);
            assert(pack.flow_x_GET() == (short)9923);
            assert(pack.flow_comp_m_y_GET() == -1.8493773E38F);
            assert(pack.time_usec_GET() == 7944063743768261455L);
            assert(pack.ground_distance_GET() == 5.601906E37F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_x_SET((short)9923) ;
        p100.quality_SET((char)118) ;
        p100.flow_comp_m_x_SET(3.1624824E38F) ;
        p100.ground_distance_SET(5.601906E37F) ;
        p100.flow_rate_x_SET(6.655559E36F, PH) ;
        p100.flow_rate_y_SET(2.838496E38F, PH) ;
        p100.time_usec_SET(7944063743768261455L) ;
        p100.flow_y_SET((short)2614) ;
        p100.flow_comp_m_y_SET(-1.8493773E38F) ;
        p100.sensor_id_SET((char)164) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8831944676909486320L);
            assert(pack.roll_GET() == -1.2157108E38F);
            assert(pack.yaw_GET() == -1.8210058E38F);
            assert(pack.pitch_GET() == 2.8575656E37F);
            assert(pack.z_GET() == -3.5867979E37F);
            assert(pack.y_GET() == -1.5814584E38F);
            assert(pack.x_GET() == -2.4859324E38F);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(8831944676909486320L) ;
        p101.pitch_SET(2.8575656E37F) ;
        p101.y_SET(-1.5814584E38F) ;
        p101.x_SET(-2.4859324E38F) ;
        p101.roll_SET(-1.2157108E38F) ;
        p101.yaw_SET(-1.8210058E38F) ;
        p101.z_SET(-3.5867979E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.1713215E38F);
            assert(pack.x_GET() == 1.7641252E38F);
            assert(pack.pitch_GET() == -3.305873E38F);
            assert(pack.z_GET() == -2.9128403E37F);
            assert(pack.roll_GET() == -1.9979145E38F);
            assert(pack.yaw_GET() == -3.1954308E38F);
            assert(pack.usec_GET() == 8515434614559520719L);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(8515434614559520719L) ;
        p102.yaw_SET(-3.1954308E38F) ;
        p102.z_SET(-2.9128403E37F) ;
        p102.x_SET(1.7641252E38F) ;
        p102.roll_SET(-1.9979145E38F) ;
        p102.pitch_SET(-3.305873E38F) ;
        p102.y_SET(-3.1713215E38F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.8792383E38F);
            assert(pack.y_GET() == 2.3709186E38F);
            assert(pack.x_GET() == -1.1322709E38F);
            assert(pack.usec_GET() == 5413786391431533561L);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.3709186E38F) ;
        p103.x_SET(-1.1322709E38F) ;
        p103.usec_SET(5413786391431533561L) ;
        p103.z_SET(-1.8792383E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.037166E38F);
            assert(pack.x_GET() == -7.3939443E37F);
            assert(pack.pitch_GET() == 3.080398E38F);
            assert(pack.z_GET() == 2.8210047E38F);
            assert(pack.y_GET() == 1.3607103E38F);
            assert(pack.usec_GET() == 1792094261629584586L);
            assert(pack.roll_GET() == 2.5655557E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.pitch_SET(3.080398E38F) ;
        p104.y_SET(1.3607103E38F) ;
        p104.yaw_SET(-3.037166E38F) ;
        p104.z_SET(2.8210047E38F) ;
        p104.x_SET(-7.3939443E37F) ;
        p104.roll_SET(2.5655557E38F) ;
        p104.usec_SET(1792094261629584586L) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == -3.1663717E38F);
            assert(pack.time_usec_GET() == 8916006271845644889L);
            assert(pack.yacc_GET() == 2.527469E37F);
            assert(pack.pressure_alt_GET() == 2.4584797E38F);
            assert(pack.abs_pressure_GET() == -3.3820502E38F);
            assert(pack.zgyro_GET() == -1.5538587E38F);
            assert(pack.fields_updated_GET() == (char)59599);
            assert(pack.diff_pressure_GET() == -2.8935764E38F);
            assert(pack.ymag_GET() == -2.9332293E38F);
            assert(pack.temperature_GET() == 3.189597E38F);
            assert(pack.xmag_GET() == 1.4201231E37F);
            assert(pack.zacc_GET() == 1.3316511E38F);
            assert(pack.zmag_GET() == 1.842375E38F);
            assert(pack.ygyro_GET() == -1.1729583E38F);
            assert(pack.xgyro_GET() == 1.7814444E38F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(1.842375E38F) ;
        p105.xmag_SET(1.4201231E37F) ;
        p105.zgyro_SET(-1.5538587E38F) ;
        p105.diff_pressure_SET(-2.8935764E38F) ;
        p105.ymag_SET(-2.9332293E38F) ;
        p105.xgyro_SET(1.7814444E38F) ;
        p105.ygyro_SET(-1.1729583E38F) ;
        p105.time_usec_SET(8916006271845644889L) ;
        p105.yacc_SET(2.527469E37F) ;
        p105.pressure_alt_SET(2.4584797E38F) ;
        p105.temperature_SET(3.189597E38F) ;
        p105.zacc_SET(1.3316511E38F) ;
        p105.xacc_SET(-3.1663717E38F) ;
        p105.fields_updated_SET((char)59599) ;
        p105.abs_pressure_SET(-3.3820502E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)99);
            assert(pack.distance_GET() == -2.7438928E38F);
            assert(pack.integrated_ygyro_GET() == -2.8358273E38F);
            assert(pack.integrated_zgyro_GET() == -3.8713446E37F);
            assert(pack.time_delta_distance_us_GET() == 2564305381L);
            assert(pack.temperature_GET() == (short) -16086);
            assert(pack.sensor_id_GET() == (char)213);
            assert(pack.integrated_xgyro_GET() == 1.3769016E38F);
            assert(pack.time_usec_GET() == 4840325640805756564L);
            assert(pack.integrated_x_GET() == 4.5393174E37F);
            assert(pack.integration_time_us_GET() == 3637612173L);
            assert(pack.integrated_y_GET() == -1.0928546E38F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_delta_distance_us_SET(2564305381L) ;
        p106.integration_time_us_SET(3637612173L) ;
        p106.integrated_xgyro_SET(1.3769016E38F) ;
        p106.integrated_zgyro_SET(-3.8713446E37F) ;
        p106.distance_SET(-2.7438928E38F) ;
        p106.integrated_y_SET(-1.0928546E38F) ;
        p106.quality_SET((char)99) ;
        p106.integrated_ygyro_SET(-2.8358273E38F) ;
        p106.sensor_id_SET((char)213) ;
        p106.integrated_x_SET(4.5393174E37F) ;
        p106.time_usec_SET(4840325640805756564L) ;
        p106.temperature_SET((short) -16086) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4649392169867202112L);
            assert(pack.xgyro_GET() == -2.865816E37F);
            assert(pack.abs_pressure_GET() == -2.9058795E38F);
            assert(pack.zmag_GET() == 2.4057345E38F);
            assert(pack.diff_pressure_GET() == -1.1360273E38F);
            assert(pack.xmag_GET() == 7.48239E37F);
            assert(pack.fields_updated_GET() == 3065945745L);
            assert(pack.zgyro_GET() == -1.971023E38F);
            assert(pack.ygyro_GET() == -2.556553E38F);
            assert(pack.ymag_GET() == 1.9791474E38F);
            assert(pack.temperature_GET() == 1.2963078E38F);
            assert(pack.xacc_GET() == -9.078537E37F);
            assert(pack.zacc_GET() == -7.787898E37F);
            assert(pack.yacc_GET() == 2.0561644E38F);
            assert(pack.pressure_alt_GET() == 2.8634168E38F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xmag_SET(7.48239E37F) ;
        p107.pressure_alt_SET(2.8634168E38F) ;
        p107.diff_pressure_SET(-1.1360273E38F) ;
        p107.yacc_SET(2.0561644E38F) ;
        p107.temperature_SET(1.2963078E38F) ;
        p107.ymag_SET(1.9791474E38F) ;
        p107.ygyro_SET(-2.556553E38F) ;
        p107.xacc_SET(-9.078537E37F) ;
        p107.abs_pressure_SET(-2.9058795E38F) ;
        p107.fields_updated_SET(3065945745L) ;
        p107.zacc_SET(-7.787898E37F) ;
        p107.xgyro_SET(-2.865816E37F) ;
        p107.zgyro_SET(-1.971023E38F) ;
        p107.zmag_SET(2.4057345E38F) ;
        p107.time_usec_SET(4649392169867202112L) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == -5.953031E37F);
            assert(pack.yacc_GET() == 4.11925E37F);
            assert(pack.alt_GET() == -2.7395213E38F);
            assert(pack.roll_GET() == -3.3776143E38F);
            assert(pack.yaw_GET() == 2.2559013E38F);
            assert(pack.xacc_GET() == 2.4890285E38F);
            assert(pack.std_dev_vert_GET() == -1.5240418E38F);
            assert(pack.ve_GET() == 2.247179E38F);
            assert(pack.q3_GET() == 1.6034681E38F);
            assert(pack.vn_GET() == -6.7878755E37F);
            assert(pack.zacc_GET() == 1.4816963E38F);
            assert(pack.xgyro_GET() == -5.064193E37F);
            assert(pack.zgyro_GET() == -1.4346021E38F);
            assert(pack.q2_GET() == 2.035511E38F);
            assert(pack.lat_GET() == -3.1158377E38F);
            assert(pack.q1_GET() == 1.1400011E38F);
            assert(pack.vd_GET() == -2.2148777E38F);
            assert(pack.q4_GET() == 4.2681877E37F);
            assert(pack.lon_GET() == 1.8046568E38F);
            assert(pack.ygyro_GET() == -1.8275771E38F);
            assert(pack.pitch_GET() == -1.7227749E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q4_SET(4.2681877E37F) ;
        p108.q3_SET(1.6034681E38F) ;
        p108.zgyro_SET(-1.4346021E38F) ;
        p108.ve_SET(2.247179E38F) ;
        p108.q2_SET(2.035511E38F) ;
        p108.vd_SET(-2.2148777E38F) ;
        p108.yaw_SET(2.2559013E38F) ;
        p108.std_dev_vert_SET(-1.5240418E38F) ;
        p108.pitch_SET(-1.7227749E38F) ;
        p108.std_dev_horz_SET(-5.953031E37F) ;
        p108.yacc_SET(4.11925E37F) ;
        p108.roll_SET(-3.3776143E38F) ;
        p108.xgyro_SET(-5.064193E37F) ;
        p108.lat_SET(-3.1158377E38F) ;
        p108.xacc_SET(2.4890285E38F) ;
        p108.vn_SET(-6.7878755E37F) ;
        p108.alt_SET(-2.7395213E38F) ;
        p108.ygyro_SET(-1.8275771E38F) ;
        p108.lon_SET(1.8046568E38F) ;
        p108.q1_SET(1.1400011E38F) ;
        p108.zacc_SET(1.4816963E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.txbuf_GET() == (char)236);
            assert(pack.rssi_GET() == (char)164);
            assert(pack.remrssi_GET() == (char)129);
            assert(pack.noise_GET() == (char)29);
            assert(pack.remnoise_GET() == (char)242);
            assert(pack.fixed__GET() == (char)63612);
            assert(pack.rxerrors_GET() == (char)54318);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)29) ;
        p109.fixed__SET((char)63612) ;
        p109.remnoise_SET((char)242) ;
        p109.rxerrors_SET((char)54318) ;
        p109.remrssi_SET((char)129) ;
        p109.txbuf_SET((char)236) ;
        p109.rssi_SET((char)164) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)131, (char)107, (char)196, (char)60, (char)156, (char)130, (char)177, (char)101, (char)104, (char)74, (char)189, (char)151, (char)74, (char)200, (char)76, (char)234, (char)185, (char)138, (char)183, (char)158, (char)53, (char)77, (char)190, (char)174, (char)57, (char)167, (char)62, (char)75, (char)180, (char)67, (char)159, (char)158, (char)93, (char)164, (char)184, (char)224, (char)100, (char)9, (char)190, (char)251, (char)58, (char)98, (char)82, (char)63, (char)199, (char)22, (char)231, (char)11, (char)42, (char)21, (char)114, (char)54, (char)1, (char)194, (char)249, (char)161, (char)239, (char)80, (char)153, (char)205, (char)206, (char)38, (char)149, (char)205, (char)200, (char)85, (char)13, (char)26, (char)89, (char)88, (char)234, (char)83, (char)132, (char)48, (char)63, (char)37, (char)74, (char)230, (char)63, (char)127, (char)118, (char)156, (char)59, (char)175, (char)185, (char)197, (char)199, (char)227, (char)3, (char)246, (char)101, (char)162, (char)147, (char)252, (char)144, (char)31, (char)67, (char)116, (char)185, (char)153, (char)218, (char)146, (char)228, (char)56, (char)6, (char)66, (char)17, (char)228, (char)85, (char)98, (char)179, (char)214, (char)166, (char)37, (char)101, (char)168, (char)210, (char)74, (char)0, (char)80, (char)117, (char)148, (char)99, (char)64, (char)216, (char)229, (char)196, (char)191, (char)180, (char)79, (char)24, (char)144, (char)77, (char)44, (char)151, (char)93, (char)5, (char)217, (char)196, (char)201, (char)245, (char)133, (char)146, (char)173, (char)228, (char)77, (char)146, (char)247, (char)255, (char)242, (char)79, (char)102, (char)239, (char)117, (char)219, (char)136, (char)116, (char)110, (char)11, (char)94, (char)40, (char)188, (char)104, (char)51, (char)131, (char)240, (char)167, (char)135, (char)100, (char)241, (char)168, (char)38, (char)143, (char)121, (char)67, (char)137, (char)19, (char)145, (char)41, (char)56, (char)245, (char)95, (char)99, (char)209, (char)117, (char)120, (char)69, (char)55, (char)39, (char)164, (char)141, (char)237, (char)148, (char)116, (char)229, (char)0, (char)89, (char)253, (char)111, (char)26, (char)15, (char)39, (char)86, (char)69, (char)209, (char)119, (char)134, (char)177, (char)93, (char)221, (char)42, (char)45, (char)59, (char)130, (char)86, (char)26, (char)208, (char)45, (char)16, (char)85, (char)10, (char)130, (char)86, (char)93, (char)22, (char)135, (char)101, (char)25, (char)14, (char)146, (char)250, (char)7, (char)136, (char)195, (char)220, (char)96, (char)131, (char)111, (char)190, (char)5, (char)182, (char)210, (char)3, (char)114, (char)7, (char)43, (char)81, (char)100, (char)212, (char)22, (char)62}));
            assert(pack.target_network_GET() == (char)86);
            assert(pack.target_component_GET() == (char)43);
            assert(pack.target_system_GET() == (char)117);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)131, (char)107, (char)196, (char)60, (char)156, (char)130, (char)177, (char)101, (char)104, (char)74, (char)189, (char)151, (char)74, (char)200, (char)76, (char)234, (char)185, (char)138, (char)183, (char)158, (char)53, (char)77, (char)190, (char)174, (char)57, (char)167, (char)62, (char)75, (char)180, (char)67, (char)159, (char)158, (char)93, (char)164, (char)184, (char)224, (char)100, (char)9, (char)190, (char)251, (char)58, (char)98, (char)82, (char)63, (char)199, (char)22, (char)231, (char)11, (char)42, (char)21, (char)114, (char)54, (char)1, (char)194, (char)249, (char)161, (char)239, (char)80, (char)153, (char)205, (char)206, (char)38, (char)149, (char)205, (char)200, (char)85, (char)13, (char)26, (char)89, (char)88, (char)234, (char)83, (char)132, (char)48, (char)63, (char)37, (char)74, (char)230, (char)63, (char)127, (char)118, (char)156, (char)59, (char)175, (char)185, (char)197, (char)199, (char)227, (char)3, (char)246, (char)101, (char)162, (char)147, (char)252, (char)144, (char)31, (char)67, (char)116, (char)185, (char)153, (char)218, (char)146, (char)228, (char)56, (char)6, (char)66, (char)17, (char)228, (char)85, (char)98, (char)179, (char)214, (char)166, (char)37, (char)101, (char)168, (char)210, (char)74, (char)0, (char)80, (char)117, (char)148, (char)99, (char)64, (char)216, (char)229, (char)196, (char)191, (char)180, (char)79, (char)24, (char)144, (char)77, (char)44, (char)151, (char)93, (char)5, (char)217, (char)196, (char)201, (char)245, (char)133, (char)146, (char)173, (char)228, (char)77, (char)146, (char)247, (char)255, (char)242, (char)79, (char)102, (char)239, (char)117, (char)219, (char)136, (char)116, (char)110, (char)11, (char)94, (char)40, (char)188, (char)104, (char)51, (char)131, (char)240, (char)167, (char)135, (char)100, (char)241, (char)168, (char)38, (char)143, (char)121, (char)67, (char)137, (char)19, (char)145, (char)41, (char)56, (char)245, (char)95, (char)99, (char)209, (char)117, (char)120, (char)69, (char)55, (char)39, (char)164, (char)141, (char)237, (char)148, (char)116, (char)229, (char)0, (char)89, (char)253, (char)111, (char)26, (char)15, (char)39, (char)86, (char)69, (char)209, (char)119, (char)134, (char)177, (char)93, (char)221, (char)42, (char)45, (char)59, (char)130, (char)86, (char)26, (char)208, (char)45, (char)16, (char)85, (char)10, (char)130, (char)86, (char)93, (char)22, (char)135, (char)101, (char)25, (char)14, (char)146, (char)250, (char)7, (char)136, (char)195, (char)220, (char)96, (char)131, (char)111, (char)190, (char)5, (char)182, (char)210, (char)3, (char)114, (char)7, (char)43, (char)81, (char)100, (char)212, (char)22, (char)62}, 0) ;
        p110.target_network_SET((char)86) ;
        p110.target_component_SET((char)43) ;
        p110.target_system_SET((char)117) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == 8265803394050568762L);
            assert(pack.ts1_GET() == 4747545236560807093L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(4747545236560807093L) ;
        p111.tc1_SET(8265803394050568762L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7071512771290093945L);
            assert(pack.seq_GET() == 1843287117L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(7071512771290093945L) ;
        p112.seq_SET(1843287117L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -538607938);
            assert(pack.time_usec_GET() == 1756175461617782035L);
            assert(pack.ve_GET() == (short) -19907);
            assert(pack.cog_GET() == (char)30089);
            assert(pack.epv_GET() == (char)47886);
            assert(pack.eph_GET() == (char)30914);
            assert(pack.vd_GET() == (short)20436);
            assert(pack.fix_type_GET() == (char)107);
            assert(pack.vel_GET() == (char)3527);
            assert(pack.vn_GET() == (short) -30310);
            assert(pack.alt_GET() == 1359043072);
            assert(pack.satellites_visible_GET() == (char)173);
            assert(pack.lat_GET() == -1481614041);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lon_SET(-538607938) ;
        p113.vel_SET((char)3527) ;
        p113.vd_SET((short)20436) ;
        p113.lat_SET(-1481614041) ;
        p113.cog_SET((char)30089) ;
        p113.epv_SET((char)47886) ;
        p113.time_usec_SET(1756175461617782035L) ;
        p113.ve_SET((short) -19907) ;
        p113.eph_SET((char)30914) ;
        p113.satellites_visible_SET((char)173) ;
        p113.vn_SET((short) -30310) ;
        p113.alt_SET(1359043072) ;
        p113.fix_type_SET((char)107) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == -2.0021704E38F);
            assert(pack.integrated_zgyro_GET() == 1.1852579E38F);
            assert(pack.temperature_GET() == (short) -24058);
            assert(pack.time_usec_GET() == 7743098376377975989L);
            assert(pack.sensor_id_GET() == (char)152);
            assert(pack.quality_GET() == (char)36);
            assert(pack.integrated_y_GET() == -2.2528788E38F);
            assert(pack.integration_time_us_GET() == 1793619040L);
            assert(pack.integrated_ygyro_GET() == -1.7039597E38F);
            assert(pack.integrated_x_GET() == -3.1451825E38F);
            assert(pack.distance_GET() == -2.2327854E38F);
            assert(pack.time_delta_distance_us_GET() == 2106961955L);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integration_time_us_SET(1793619040L) ;
        p114.quality_SET((char)36) ;
        p114.integrated_x_SET(-3.1451825E38F) ;
        p114.integrated_y_SET(-2.2528788E38F) ;
        p114.integrated_ygyro_SET(-1.7039597E38F) ;
        p114.sensor_id_SET((char)152) ;
        p114.time_delta_distance_us_SET(2106961955L) ;
        p114.integrated_zgyro_SET(1.1852579E38F) ;
        p114.temperature_SET((short) -24058) ;
        p114.time_usec_SET(7743098376377975989L) ;
        p114.distance_SET(-2.2327854E38F) ;
        p114.integrated_xgyro_SET(-2.0021704E38F) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == (short) -18876);
            assert(pack.yawspeed_GET() == -3.3851451E38F);
            assert(pack.yacc_GET() == (short) -7688);
            assert(pack.pitchspeed_GET() == -1.8729684E38F);
            assert(pack.rollspeed_GET() == -6.57541E37F);
            assert(pack.lat_GET() == 628858736);
            assert(pack.true_airspeed_GET() == (char)3563);
            assert(pack.zacc_GET() == (short) -6544);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-2.6549275E38F, 1.806712E38F, -2.559241E37F, 8.553317E37F}));
            assert(pack.lon_GET() == -655579862);
            assert(pack.ind_airspeed_GET() == (char)59662);
            assert(pack.alt_GET() == 548581390);
            assert(pack.vz_GET() == (short)5919);
            assert(pack.vx_GET() == (short) -21976);
            assert(pack.time_usec_GET() == 179446298193389099L);
            assert(pack.xacc_GET() == (short) -26200);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.lon_SET(-655579862) ;
        p115.vx_SET((short) -21976) ;
        p115.attitude_quaternion_SET(new float[] {-2.6549275E38F, 1.806712E38F, -2.559241E37F, 8.553317E37F}, 0) ;
        p115.time_usec_SET(179446298193389099L) ;
        p115.rollspeed_SET(-6.57541E37F) ;
        p115.yawspeed_SET(-3.3851451E38F) ;
        p115.true_airspeed_SET((char)3563) ;
        p115.xacc_SET((short) -26200) ;
        p115.lat_SET(628858736) ;
        p115.yacc_SET((short) -7688) ;
        p115.alt_SET(548581390) ;
        p115.pitchspeed_SET(-1.8729684E38F) ;
        p115.vy_SET((short) -18876) ;
        p115.vz_SET((short)5919) ;
        p115.ind_airspeed_SET((char)59662) ;
        p115.zacc_SET((short) -6544) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)18290);
            assert(pack.xmag_GET() == (short) -24011);
            assert(pack.zacc_GET() == (short) -10140);
            assert(pack.time_boot_ms_GET() == 2784264709L);
            assert(pack.zgyro_GET() == (short) -11909);
            assert(pack.ymag_GET() == (short)25254);
            assert(pack.xacc_GET() == (short)10126);
            assert(pack.ygyro_GET() == (short) -27656);
            assert(pack.xgyro_GET() == (short) -7947);
            assert(pack.zmag_GET() == (short) -26790);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.ygyro_SET((short) -27656) ;
        p116.zgyro_SET((short) -11909) ;
        p116.yacc_SET((short)18290) ;
        p116.zmag_SET((short) -26790) ;
        p116.zacc_SET((short) -10140) ;
        p116.xgyro_SET((short) -7947) ;
        p116.xacc_SET((short)10126) ;
        p116.time_boot_ms_SET(2784264709L) ;
        p116.ymag_SET((short)25254) ;
        p116.xmag_SET((short) -24011) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)134);
            assert(pack.start_GET() == (char)15315);
            assert(pack.target_component_GET() == (char)136);
            assert(pack.end_GET() == (char)54294);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)15315) ;
        p117.end_SET((char)54294) ;
        p117.target_component_SET((char)136) ;
        p117.target_system_SET((char)134) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)9637);
            assert(pack.time_utc_GET() == 655096851L);
            assert(pack.id_GET() == (char)47276);
            assert(pack.size_GET() == 1676901874L);
            assert(pack.num_logs_GET() == (char)25657);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)47276) ;
        p118.time_utc_SET(655096851L) ;
        p118.size_SET(1676901874L) ;
        p118.last_log_num_SET((char)9637) ;
        p118.num_logs_SET((char)25657) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)29375);
            assert(pack.target_component_GET() == (char)83);
            assert(pack.target_system_GET() == (char)169);
            assert(pack.ofs_GET() == 2579089122L);
            assert(pack.count_GET() == 3549198909L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_component_SET((char)83) ;
        p119.id_SET((char)29375) ;
        p119.target_system_SET((char)169) ;
        p119.count_SET(3549198909L) ;
        p119.ofs_SET(2579089122L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)109);
            assert(pack.ofs_GET() == 2565336402L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)113, (char)27, (char)113, (char)133, (char)67, (char)116, (char)96, (char)32, (char)214, (char)28, (char)214, (char)12, (char)205, (char)88, (char)101, (char)108, (char)168, (char)155, (char)111, (char)231, (char)205, (char)231, (char)111, (char)64, (char)124, (char)230, (char)154, (char)19, (char)172, (char)255, (char)134, (char)119, (char)244, (char)230, (char)148, (char)186, (char)221, (char)96, (char)210, (char)39, (char)71, (char)119, (char)179, (char)37, (char)31, (char)26, (char)44, (char)120, (char)194, (char)224, (char)131, (char)182, (char)69, (char)152, (char)129, (char)176, (char)35, (char)76, (char)66, (char)106, (char)232, (char)221, (char)9, (char)184, (char)125, (char)94, (char)47, (char)239, (char)142, (char)232, (char)81, (char)133, (char)198, (char)199, (char)245, (char)119, (char)168, (char)219, (char)90, (char)72, (char)248, (char)24, (char)247, (char)102, (char)59, (char)175, (char)168, (char)96, (char)117, (char)67}));
            assert(pack.id_GET() == (char)33788);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)113, (char)27, (char)113, (char)133, (char)67, (char)116, (char)96, (char)32, (char)214, (char)28, (char)214, (char)12, (char)205, (char)88, (char)101, (char)108, (char)168, (char)155, (char)111, (char)231, (char)205, (char)231, (char)111, (char)64, (char)124, (char)230, (char)154, (char)19, (char)172, (char)255, (char)134, (char)119, (char)244, (char)230, (char)148, (char)186, (char)221, (char)96, (char)210, (char)39, (char)71, (char)119, (char)179, (char)37, (char)31, (char)26, (char)44, (char)120, (char)194, (char)224, (char)131, (char)182, (char)69, (char)152, (char)129, (char)176, (char)35, (char)76, (char)66, (char)106, (char)232, (char)221, (char)9, (char)184, (char)125, (char)94, (char)47, (char)239, (char)142, (char)232, (char)81, (char)133, (char)198, (char)199, (char)245, (char)119, (char)168, (char)219, (char)90, (char)72, (char)248, (char)24, (char)247, (char)102, (char)59, (char)175, (char)168, (char)96, (char)117, (char)67}, 0) ;
        p120.ofs_SET(2565336402L) ;
        p120.id_SET((char)33788) ;
        p120.count_SET((char)109) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)208);
            assert(pack.target_component_GET() == (char)29);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)208) ;
        p121.target_component_SET((char)29) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)70);
            assert(pack.target_system_GET() == (char)77);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)77) ;
        p122.target_component_SET((char)70) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)57);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)102, (char)102, (char)19, (char)47, (char)203, (char)229, (char)147, (char)190, (char)235, (char)172, (char)97, (char)149, (char)219, (char)106, (char)0, (char)35, (char)125, (char)50, (char)111, (char)232, (char)219, (char)164, (char)162, (char)160, (char)88, (char)84, (char)75, (char)237, (char)13, (char)129, (char)17, (char)205, (char)254, (char)39, (char)221, (char)22, (char)92, (char)190, (char)225, (char)128, (char)220, (char)162, (char)165, (char)46, (char)205, (char)61, (char)172, (char)174, (char)120, (char)215, (char)154, (char)123, (char)120, (char)4, (char)151, (char)67, (char)49, (char)80, (char)181, (char)173, (char)39, (char)178, (char)21, (char)56, (char)193, (char)234, (char)172, (char)82, (char)25, (char)75, (char)57, (char)175, (char)136, (char)192, (char)187, (char)240, (char)188, (char)104, (char)223, (char)67, (char)182, (char)170, (char)158, (char)12, (char)191, (char)83, (char)102, (char)44, (char)112, (char)153, (char)65, (char)187, (char)83, (char)211, (char)19, (char)185, (char)180, (char)173, (char)128, (char)169, (char)150, (char)65, (char)182, (char)109, (char)209, (char)34, (char)211, (char)243, (char)195, (char)24}));
            assert(pack.len_GET() == (char)178);
            assert(pack.target_component_GET() == (char)61);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)57) ;
        p123.target_component_SET((char)61) ;
        p123.data__SET(new char[] {(char)102, (char)102, (char)19, (char)47, (char)203, (char)229, (char)147, (char)190, (char)235, (char)172, (char)97, (char)149, (char)219, (char)106, (char)0, (char)35, (char)125, (char)50, (char)111, (char)232, (char)219, (char)164, (char)162, (char)160, (char)88, (char)84, (char)75, (char)237, (char)13, (char)129, (char)17, (char)205, (char)254, (char)39, (char)221, (char)22, (char)92, (char)190, (char)225, (char)128, (char)220, (char)162, (char)165, (char)46, (char)205, (char)61, (char)172, (char)174, (char)120, (char)215, (char)154, (char)123, (char)120, (char)4, (char)151, (char)67, (char)49, (char)80, (char)181, (char)173, (char)39, (char)178, (char)21, (char)56, (char)193, (char)234, (char)172, (char)82, (char)25, (char)75, (char)57, (char)175, (char)136, (char)192, (char)187, (char)240, (char)188, (char)104, (char)223, (char)67, (char)182, (char)170, (char)158, (char)12, (char)191, (char)83, (char)102, (char)44, (char)112, (char)153, (char)65, (char)187, (char)83, (char)211, (char)19, (char)185, (char)180, (char)173, (char)128, (char)169, (char)150, (char)65, (char)182, (char)109, (char)209, (char)34, (char)211, (char)243, (char)195, (char)24}, 0) ;
        p123.len_SET((char)178) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.eph_GET() == (char)39894);
            assert(pack.vel_GET() == (char)22673);
            assert(pack.cog_GET() == (char)19682);
            assert(pack.dgps_numch_GET() == (char)18);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.time_usec_GET() == 3184757483741841049L);
            assert(pack.lat_GET() == -1272297122);
            assert(pack.lon_GET() == -637675696);
            assert(pack.satellites_visible_GET() == (char)190);
            assert(pack.alt_GET() == 10094641);
            assert(pack.epv_GET() == (char)22708);
            assert(pack.dgps_age_GET() == 1444316043L);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.eph_SET((char)39894) ;
        p124.alt_SET(10094641) ;
        p124.dgps_numch_SET((char)18) ;
        p124.epv_SET((char)22708) ;
        p124.satellites_visible_SET((char)190) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p124.dgps_age_SET(1444316043L) ;
        p124.cog_SET((char)19682) ;
        p124.time_usec_SET(3184757483741841049L) ;
        p124.lat_SET(-1272297122) ;
        p124.vel_SET((char)22673) ;
        p124.lon_SET(-637675696) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)17874);
            assert(pack.Vcc_GET() == (char)48848);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)48848) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID) ;
        p125.Vservo_SET((char)17874) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.timeout_GET() == (char)17317);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)17, (char)236, (char)90, (char)118, (char)233, (char)227, (char)26, (char)96, (char)207, (char)171, (char)219, (char)181, (char)53, (char)111, (char)138, (char)11, (char)50, (char)131, (char)182, (char)119, (char)107, (char)8, (char)218, (char)14, (char)241, (char)55, (char)42, (char)8, (char)116, (char)146, (char)135, (char)176, (char)19, (char)9, (char)222, (char)219, (char)239, (char)132, (char)98, (char)110, (char)83, (char)202, (char)245, (char)97, (char)101, (char)138, (char)134, (char)248, (char)221, (char)58, (char)216, (char)149, (char)233, (char)171, (char)178, (char)77, (char)18, (char)81, (char)227, (char)125, (char)247, (char)136, (char)83, (char)250, (char)117, (char)228, (char)66, (char)30, (char)189, (char)155}));
            assert(pack.baudrate_GET() == 4020395176L);
            assert(pack.count_GET() == (char)145);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)145) ;
        p126.baudrate_SET(4020395176L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.timeout_SET((char)17317) ;
        p126.data__SET(new char[] {(char)17, (char)236, (char)90, (char)118, (char)233, (char)227, (char)26, (char)96, (char)207, (char)171, (char)219, (char)181, (char)53, (char)111, (char)138, (char)11, (char)50, (char)131, (char)182, (char)119, (char)107, (char)8, (char)218, (char)14, (char)241, (char)55, (char)42, (char)8, (char)116, (char)146, (char)135, (char)176, (char)19, (char)9, (char)222, (char)219, (char)239, (char)132, (char)98, (char)110, (char)83, (char)202, (char)245, (char)97, (char)101, (char)138, (char)134, (char)248, (char)221, (char)58, (char)216, (char)149, (char)233, (char)171, (char)178, (char)77, (char)18, (char)81, (char)227, (char)125, (char)247, (char)136, (char)83, (char)250, (char)117, (char)228, (char)66, (char)30, (char)189, (char)155}, 0) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_coords_type_GET() == (char)210);
            assert(pack.rtk_health_GET() == (char)50);
            assert(pack.iar_num_hypotheses_GET() == 1902613772);
            assert(pack.wn_GET() == (char)30495);
            assert(pack.tow_GET() == 3878757675L);
            assert(pack.rtk_rate_GET() == (char)109);
            assert(pack.rtk_receiver_id_GET() == (char)25);
            assert(pack.nsats_GET() == (char)218);
            assert(pack.baseline_b_mm_GET() == -281661717);
            assert(pack.baseline_c_mm_GET() == -110979973);
            assert(pack.accuracy_GET() == 809034575L);
            assert(pack.time_last_baseline_ms_GET() == 1060825173L);
            assert(pack.baseline_a_mm_GET() == 2047455284);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.rtk_health_SET((char)50) ;
        p127.wn_SET((char)30495) ;
        p127.baseline_coords_type_SET((char)210) ;
        p127.baseline_b_mm_SET(-281661717) ;
        p127.baseline_c_mm_SET(-110979973) ;
        p127.accuracy_SET(809034575L) ;
        p127.tow_SET(3878757675L) ;
        p127.iar_num_hypotheses_SET(1902613772) ;
        p127.rtk_rate_SET((char)109) ;
        p127.baseline_a_mm_SET(2047455284) ;
        p127.nsats_SET((char)218) ;
        p127.time_last_baseline_ms_SET(1060825173L) ;
        p127.rtk_receiver_id_SET((char)25) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)161);
            assert(pack.baseline_c_mm_GET() == -1303971186);
            assert(pack.baseline_coords_type_GET() == (char)191);
            assert(pack.baseline_a_mm_GET() == -905444870);
            assert(pack.rtk_rate_GET() == (char)15);
            assert(pack.rtk_receiver_id_GET() == (char)181);
            assert(pack.tow_GET() == 714927703L);
            assert(pack.baseline_b_mm_GET() == 2136486904);
            assert(pack.accuracy_GET() == 146143312L);
            assert(pack.rtk_health_GET() == (char)189);
            assert(pack.wn_GET() == (char)16255);
            assert(pack.iar_num_hypotheses_GET() == 1415602638);
            assert(pack.time_last_baseline_ms_GET() == 4000615015L);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_coords_type_SET((char)191) ;
        p128.tow_SET(714927703L) ;
        p128.baseline_b_mm_SET(2136486904) ;
        p128.accuracy_SET(146143312L) ;
        p128.rtk_receiver_id_SET((char)181) ;
        p128.rtk_rate_SET((char)15) ;
        p128.baseline_a_mm_SET(-905444870) ;
        p128.baseline_c_mm_SET(-1303971186) ;
        p128.wn_SET((char)16255) ;
        p128.time_last_baseline_ms_SET(4000615015L) ;
        p128.nsats_SET((char)161) ;
        p128.iar_num_hypotheses_SET(1415602638) ;
        p128.rtk_health_SET((char)189) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)25505);
            assert(pack.time_boot_ms_GET() == 4024478256L);
            assert(pack.zmag_GET() == (short) -15694);
            assert(pack.zacc_GET() == (short) -18448);
            assert(pack.yacc_GET() == (short)32560);
            assert(pack.zgyro_GET() == (short)17251);
            assert(pack.xacc_GET() == (short) -2634);
            assert(pack.xmag_GET() == (short) -8023);
            assert(pack.xgyro_GET() == (short)8381);
            assert(pack.ymag_GET() == (short) -20985);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zacc_SET((short) -18448) ;
        p129.xgyro_SET((short)8381) ;
        p129.time_boot_ms_SET(4024478256L) ;
        p129.ygyro_SET((short)25505) ;
        p129.ymag_SET((short) -20985) ;
        p129.zgyro_SET((short)17251) ;
        p129.yacc_SET((short)32560) ;
        p129.xmag_SET((short) -8023) ;
        p129.xacc_SET((short) -2634) ;
        p129.zmag_SET((short) -15694) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.width_GET() == (char)32595);
            assert(pack.size_GET() == 4037464366L);
            assert(pack.height_GET() == (char)1984);
            assert(pack.jpg_quality_GET() == (char)233);
            assert(pack.type_GET() == (char)168);
            assert(pack.packets_GET() == (char)50985);
            assert(pack.payload_GET() == (char)117);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.jpg_quality_SET((char)233) ;
        p130.type_SET((char)168) ;
        p130.payload_SET((char)117) ;
        p130.size_SET(4037464366L) ;
        p130.packets_SET((char)50985) ;
        p130.width_SET((char)32595) ;
        p130.height_SET((char)1984) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)36586);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)117, (char)28, (char)71, (char)58, (char)176, (char)233, (char)123, (char)218, (char)210, (char)12, (char)109, (char)95, (char)153, (char)7, (char)96, (char)21, (char)59, (char)198, (char)119, (char)170, (char)94, (char)208, (char)208, (char)90, (char)214, (char)8, (char)212, (char)235, (char)146, (char)155, (char)67, (char)212, (char)170, (char)109, (char)23, (char)208, (char)200, (char)114, (char)125, (char)62, (char)57, (char)223, (char)175, (char)3, (char)39, (char)47, (char)255, (char)44, (char)115, (char)163, (char)245, (char)209, (char)248, (char)71, (char)71, (char)156, (char)158, (char)159, (char)79, (char)67, (char)32, (char)114, (char)96, (char)200, (char)146, (char)193, (char)138, (char)114, (char)78, (char)156, (char)215, (char)149, (char)74, (char)15, (char)120, (char)9, (char)134, (char)4, (char)121, (char)56, (char)38, (char)141, (char)43, (char)89, (char)231, (char)92, (char)198, (char)187, (char)159, (char)134, (char)201, (char)69, (char)197, (char)157, (char)219, (char)46, (char)223, (char)39, (char)149, (char)173, (char)184, (char)211, (char)67, (char)166, (char)14, (char)250, (char)8, (char)174, (char)145, (char)180, (char)173, (char)63, (char)172, (char)201, (char)201, (char)87, (char)127, (char)98, (char)78, (char)198, (char)149, (char)178, (char)238, (char)21, (char)130, (char)219, (char)181, (char)245, (char)15, (char)105, (char)181, (char)205, (char)162, (char)220, (char)239, (char)212, (char)136, (char)111, (char)157, (char)86, (char)49, (char)197, (char)162, (char)0, (char)238, (char)165, (char)44, (char)157, (char)31, (char)20, (char)36, (char)169, (char)114, (char)99, (char)67, (char)99, (char)100, (char)247, (char)122, (char)120, (char)169, (char)98, (char)167, (char)69, (char)24, (char)198, (char)213, (char)0, (char)42, (char)217, (char)179, (char)153, (char)215, (char)223, (char)69, (char)100, (char)190, (char)108, (char)106, (char)199, (char)89, (char)210, (char)162, (char)211, (char)30, (char)10, (char)156, (char)43, (char)2, (char)94, (char)197, (char)190, (char)219, (char)177, (char)248, (char)105, (char)65, (char)135, (char)178, (char)166, (char)139, (char)18, (char)50, (char)207, (char)67, (char)66, (char)25, (char)198, (char)237, (char)31, (char)12, (char)59, (char)84, (char)92, (char)70, (char)49, (char)86, (char)76, (char)150, (char)209, (char)213, (char)162, (char)37, (char)2, (char)27, (char)84, (char)133, (char)203, (char)219, (char)20, (char)219, (char)232, (char)78, (char)178, (char)138, (char)52, (char)192, (char)71, (char)24, (char)97, (char)124, (char)201, (char)229, (char)109, (char)16, (char)196, (char)194, (char)97, (char)100, (char)66, (char)189, (char)198, (char)7}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)36586) ;
        p131.data__SET(new char[] {(char)117, (char)28, (char)71, (char)58, (char)176, (char)233, (char)123, (char)218, (char)210, (char)12, (char)109, (char)95, (char)153, (char)7, (char)96, (char)21, (char)59, (char)198, (char)119, (char)170, (char)94, (char)208, (char)208, (char)90, (char)214, (char)8, (char)212, (char)235, (char)146, (char)155, (char)67, (char)212, (char)170, (char)109, (char)23, (char)208, (char)200, (char)114, (char)125, (char)62, (char)57, (char)223, (char)175, (char)3, (char)39, (char)47, (char)255, (char)44, (char)115, (char)163, (char)245, (char)209, (char)248, (char)71, (char)71, (char)156, (char)158, (char)159, (char)79, (char)67, (char)32, (char)114, (char)96, (char)200, (char)146, (char)193, (char)138, (char)114, (char)78, (char)156, (char)215, (char)149, (char)74, (char)15, (char)120, (char)9, (char)134, (char)4, (char)121, (char)56, (char)38, (char)141, (char)43, (char)89, (char)231, (char)92, (char)198, (char)187, (char)159, (char)134, (char)201, (char)69, (char)197, (char)157, (char)219, (char)46, (char)223, (char)39, (char)149, (char)173, (char)184, (char)211, (char)67, (char)166, (char)14, (char)250, (char)8, (char)174, (char)145, (char)180, (char)173, (char)63, (char)172, (char)201, (char)201, (char)87, (char)127, (char)98, (char)78, (char)198, (char)149, (char)178, (char)238, (char)21, (char)130, (char)219, (char)181, (char)245, (char)15, (char)105, (char)181, (char)205, (char)162, (char)220, (char)239, (char)212, (char)136, (char)111, (char)157, (char)86, (char)49, (char)197, (char)162, (char)0, (char)238, (char)165, (char)44, (char)157, (char)31, (char)20, (char)36, (char)169, (char)114, (char)99, (char)67, (char)99, (char)100, (char)247, (char)122, (char)120, (char)169, (char)98, (char)167, (char)69, (char)24, (char)198, (char)213, (char)0, (char)42, (char)217, (char)179, (char)153, (char)215, (char)223, (char)69, (char)100, (char)190, (char)108, (char)106, (char)199, (char)89, (char)210, (char)162, (char)211, (char)30, (char)10, (char)156, (char)43, (char)2, (char)94, (char)197, (char)190, (char)219, (char)177, (char)248, (char)105, (char)65, (char)135, (char)178, (char)166, (char)139, (char)18, (char)50, (char)207, (char)67, (char)66, (char)25, (char)198, (char)237, (char)31, (char)12, (char)59, (char)84, (char)92, (char)70, (char)49, (char)86, (char)76, (char)150, (char)209, (char)213, (char)162, (char)37, (char)2, (char)27, (char)84, (char)133, (char)203, (char)219, (char)20, (char)219, (char)232, (char)78, (char)178, (char)138, (char)52, (char)192, (char)71, (char)24, (char)97, (char)124, (char)201, (char)229, (char)109, (char)16, (char)196, (char)194, (char)97, (char)100, (char)66, (char)189, (char)198, (char)7}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3179137836L);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_90);
            assert(pack.current_distance_GET() == (char)40566);
            assert(pack.max_distance_GET() == (char)7942);
            assert(pack.min_distance_GET() == (char)3382);
            assert(pack.covariance_GET() == (char)28);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            assert(pack.id_GET() == (char)141);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.covariance_SET((char)28) ;
        p132.id_SET((char)141) ;
        p132.max_distance_SET((char)7942) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p132.current_distance_SET((char)40566) ;
        p132.min_distance_SET((char)3382) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_90) ;
        p132.time_boot_ms_SET(3179137836L) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 8694178546222249404L);
            assert(pack.lon_GET() == -1069178858);
            assert(pack.grid_spacing_GET() == (char)27507);
            assert(pack.lat_GET() == -283974493);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-1069178858) ;
        p133.mask_SET(8694178546222249404L) ;
        p133.grid_spacing_SET((char)27507) ;
        p133.lat_SET(-283974493) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)60037);
            assert(pack.gridbit_GET() == (char)33);
            assert(pack.lon_GET() == -2140112622);
            assert(pack.lat_GET() == 372921449);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -32073, (short)7620, (short) -16400, (short) -10245, (short) -1553, (short)433, (short) -6936, (short)4995, (short) -21255, (short)18971, (short)4375, (short) -6158, (short)5190, (short)9105, (short) -27119, (short)19771}));
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(372921449) ;
        p134.data__SET(new short[] {(short) -32073, (short)7620, (short) -16400, (short) -10245, (short) -1553, (short)433, (short) -6936, (short)4995, (short) -21255, (short)18971, (short)4375, (short) -6158, (short)5190, (short)9105, (short) -27119, (short)19771}, 0) ;
        p134.gridbit_SET((char)33) ;
        p134.lon_SET(-2140112622) ;
        p134.grid_spacing_SET((char)60037) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1499865637);
            assert(pack.lon_GET() == -1265604802);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1499865637) ;
        p135.lon_SET(-1265604802) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.spacing_GET() == (char)25720);
            assert(pack.lat_GET() == 1122945328);
            assert(pack.current_height_GET() == -3.2271988E37F);
            assert(pack.pending_GET() == (char)57570);
            assert(pack.terrain_height_GET() == 3.1848377E38F);
            assert(pack.loaded_GET() == (char)50288);
            assert(pack.lon_GET() == -455382419);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(-3.2271988E37F) ;
        p136.loaded_SET((char)50288) ;
        p136.lat_SET(1122945328) ;
        p136.spacing_SET((char)25720) ;
        p136.terrain_height_SET(3.1848377E38F) ;
        p136.pending_SET((char)57570) ;
        p136.lon_SET(-455382419) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.9876172E38F);
            assert(pack.time_boot_ms_GET() == 49996187L);
            assert(pack.press_diff_GET() == 2.7077197E38F);
            assert(pack.temperature_GET() == (short)302);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.temperature_SET((short)302) ;
        p137.time_boot_ms_SET(49996187L) ;
        p137.press_diff_SET(2.7077197E38F) ;
        p137.press_abs_SET(2.9876172E38F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.039614E38F);
            assert(pack.time_usec_GET() == 5463051589281330269L);
            assert(pack.z_GET() == -1.6462302E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4960437E37F, 1.0724741E37F, 3.166908E38F, -2.6894627E38F}));
            assert(pack.x_GET() == -2.322703E38F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(5463051589281330269L) ;
        p138.y_SET(2.039614E38F) ;
        p138.z_SET(-1.6462302E38F) ;
        p138.x_SET(-2.322703E38F) ;
        p138.q_SET(new float[] {1.4960437E37F, 1.0724741E37F, 3.166908E38F, -2.6894627E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)186);
            assert(pack.time_usec_GET() == 6567709216333540840L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.6761098E38F, 1.3591273E38F, -1.9336339E38F, 2.939048E38F, 1.292405E38F, 2.1522912E38F, 3.0683586E38F, 2.3410904E38F}));
            assert(pack.group_mlx_GET() == (char)100);
            assert(pack.target_component_GET() == (char)245);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {1.6761098E38F, 1.3591273E38F, -1.9336339E38F, 2.939048E38F, 1.292405E38F, 2.1522912E38F, 3.0683586E38F, 2.3410904E38F}, 0) ;
        p139.time_usec_SET(6567709216333540840L) ;
        p139.target_system_SET((char)186) ;
        p139.target_component_SET((char)245) ;
        p139.group_mlx_SET((char)100) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)180);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.4122757E38F, -1.851449E38F, -2.9778893E38F, -4.4285673E37F, 3.1846133E38F, -3.1701305E38F, -5.9740727E37F, 2.2562227E38F}));
            assert(pack.time_usec_GET() == 7804632109353212902L);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)180) ;
        p140.controls_SET(new float[] {1.4122757E38F, -1.851449E38F, -2.9778893E38F, -4.4285673E37F, 3.1846133E38F, -3.1701305E38F, -5.9740727E37F, 2.2562227E38F}, 0) ;
        p140.time_usec_SET(7804632109353212902L) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.bottom_clearance_GET() == -2.8478848E38F);
            assert(pack.altitude_relative_GET() == 1.4247196E38F);
            assert(pack.altitude_amsl_GET() == 3.2279185E38F);
            assert(pack.time_usec_GET() == 1738587131612152522L);
            assert(pack.altitude_monotonic_GET() == 3.24003E38F);
            assert(pack.altitude_terrain_GET() == -2.367531E37F);
            assert(pack.altitude_local_GET() == -8.5000506E37F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(-2.8478848E38F) ;
        p141.time_usec_SET(1738587131612152522L) ;
        p141.altitude_relative_SET(1.4247196E38F) ;
        p141.altitude_monotonic_SET(3.24003E38F) ;
        p141.altitude_amsl_SET(3.2279185E38F) ;
        p141.altitude_local_SET(-8.5000506E37F) ;
        p141.altitude_terrain_SET(-2.367531E37F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)255);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)250, (char)53, (char)139, (char)253, (char)35, (char)113, (char)251, (char)223, (char)99, (char)171, (char)210, (char)36, (char)202, (char)176, (char)215, (char)40, (char)207, (char)52, (char)152, (char)126, (char)95, (char)31, (char)34, (char)219, (char)63, (char)173, (char)114, (char)45, (char)100, (char)65, (char)150, (char)141, (char)128, (char)93, (char)224, (char)208, (char)133, (char)62, (char)81, (char)197, (char)117, (char)247, (char)106, (char)187, (char)230, (char)150, (char)129, (char)249, (char)248, (char)210, (char)55, (char)130, (char)53, (char)23, (char)199, (char)98, (char)14, (char)184, (char)110, (char)24, (char)90, (char)21, (char)250, (char)1, (char)221, (char)100, (char)8, (char)124, (char)10, (char)146, (char)255, (char)53, (char)200, (char)206, (char)188, (char)29, (char)41, (char)192, (char)60, (char)99, (char)170, (char)157, (char)66, (char)205, (char)112, (char)162, (char)56, (char)39, (char)201, (char)61, (char)96, (char)47, (char)186, (char)218, (char)117, (char)169, (char)36, (char)42, (char)33, (char)142, (char)249, (char)227, (char)120, (char)195, (char)197, (char)109, (char)159, (char)96, (char)210, (char)120, (char)171, (char)236, (char)116, (char)55, (char)65, (char)21, (char)172, (char)225, (char)196, (char)1}));
            assert(pack.transfer_type_GET() == (char)35);
            assert(pack.uri_type_GET() == (char)99);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)44, (char)23, (char)139, (char)23, (char)184, (char)36, (char)150, (char)155, (char)33, (char)208, (char)84, (char)78, (char)249, (char)48, (char)130, (char)137, (char)125, (char)98, (char)96, (char)252, (char)31, (char)26, (char)249, (char)33, (char)66, (char)36, (char)161, (char)225, (char)167, (char)105, (char)30, (char)36, (char)252, (char)28, (char)164, (char)221, (char)98, (char)14, (char)181, (char)115, (char)159, (char)82, (char)201, (char)197, (char)209, (char)5, (char)135, (char)247, (char)191, (char)238, (char)191, (char)34, (char)89, (char)142, (char)157, (char)25, (char)207, (char)206, (char)119, (char)196, (char)24, (char)162, (char)253, (char)185, (char)18, (char)32, (char)58, (char)11, (char)236, (char)128, (char)43, (char)198, (char)94, (char)31, (char)154, (char)237, (char)147, (char)207, (char)25, (char)66, (char)71, (char)40, (char)41, (char)7, (char)84, (char)164, (char)196, (char)85, (char)15, (char)224, (char)84, (char)102, (char)25, (char)193, (char)216, (char)199, (char)131, (char)11, (char)188, (char)239, (char)228, (char)143, (char)36, (char)191, (char)212, (char)225, (char)40, (char)127, (char)54, (char)82, (char)154, (char)4, (char)59, (char)83, (char)128, (char)212, (char)203, (char)161, (char)155, (char)148}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)44, (char)23, (char)139, (char)23, (char)184, (char)36, (char)150, (char)155, (char)33, (char)208, (char)84, (char)78, (char)249, (char)48, (char)130, (char)137, (char)125, (char)98, (char)96, (char)252, (char)31, (char)26, (char)249, (char)33, (char)66, (char)36, (char)161, (char)225, (char)167, (char)105, (char)30, (char)36, (char)252, (char)28, (char)164, (char)221, (char)98, (char)14, (char)181, (char)115, (char)159, (char)82, (char)201, (char)197, (char)209, (char)5, (char)135, (char)247, (char)191, (char)238, (char)191, (char)34, (char)89, (char)142, (char)157, (char)25, (char)207, (char)206, (char)119, (char)196, (char)24, (char)162, (char)253, (char)185, (char)18, (char)32, (char)58, (char)11, (char)236, (char)128, (char)43, (char)198, (char)94, (char)31, (char)154, (char)237, (char)147, (char)207, (char)25, (char)66, (char)71, (char)40, (char)41, (char)7, (char)84, (char)164, (char)196, (char)85, (char)15, (char)224, (char)84, (char)102, (char)25, (char)193, (char)216, (char)199, (char)131, (char)11, (char)188, (char)239, (char)228, (char)143, (char)36, (char)191, (char)212, (char)225, (char)40, (char)127, (char)54, (char)82, (char)154, (char)4, (char)59, (char)83, (char)128, (char)212, (char)203, (char)161, (char)155, (char)148}, 0) ;
        p142.uri_type_SET((char)99) ;
        p142.uri_SET(new char[] {(char)250, (char)53, (char)139, (char)253, (char)35, (char)113, (char)251, (char)223, (char)99, (char)171, (char)210, (char)36, (char)202, (char)176, (char)215, (char)40, (char)207, (char)52, (char)152, (char)126, (char)95, (char)31, (char)34, (char)219, (char)63, (char)173, (char)114, (char)45, (char)100, (char)65, (char)150, (char)141, (char)128, (char)93, (char)224, (char)208, (char)133, (char)62, (char)81, (char)197, (char)117, (char)247, (char)106, (char)187, (char)230, (char)150, (char)129, (char)249, (char)248, (char)210, (char)55, (char)130, (char)53, (char)23, (char)199, (char)98, (char)14, (char)184, (char)110, (char)24, (char)90, (char)21, (char)250, (char)1, (char)221, (char)100, (char)8, (char)124, (char)10, (char)146, (char)255, (char)53, (char)200, (char)206, (char)188, (char)29, (char)41, (char)192, (char)60, (char)99, (char)170, (char)157, (char)66, (char)205, (char)112, (char)162, (char)56, (char)39, (char)201, (char)61, (char)96, (char)47, (char)186, (char)218, (char)117, (char)169, (char)36, (char)42, (char)33, (char)142, (char)249, (char)227, (char)120, (char)195, (char)197, (char)109, (char)159, (char)96, (char)210, (char)120, (char)171, (char)236, (char)116, (char)55, (char)65, (char)21, (char)172, (char)225, (char)196, (char)1}, 0) ;
        p142.transfer_type_SET((char)35) ;
        p142.request_id_SET((char)255) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.7217884E38F);
            assert(pack.temperature_GET() == (short) -31764);
            assert(pack.time_boot_ms_GET() == 3481691482L);
            assert(pack.press_diff_GET() == -1.9803718E38F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short) -31764) ;
        p143.press_abs_SET(2.7217884E38F) ;
        p143.press_diff_SET(-1.9803718E38F) ;
        p143.time_boot_ms_SET(3481691482L) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.9744975E38F, -1.1064283E38F, 1.5976048E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-3.2571015E38F, 9.75238E37F, -1.4917778E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {6.373067E37F, 2.8604832E38F, -3.7471118E37F}));
            assert(pack.custom_state_GET() == 7259861527755437129L);
            assert(pack.lat_GET() == 658709872);
            assert(pack.est_capabilities_GET() == (char)79);
            assert(pack.alt_GET() == -1.7852407E38F);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-1.0704405E37F, -2.6322245E38F, -6.853433E36F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.6467643E38F, -2.664329E38F, 2.4490525E38F, 6.62551E37F}));
            assert(pack.timestamp_GET() == 5190323864603679296L);
            assert(pack.lon_GET() == -1313739808);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.est_capabilities_SET((char)79) ;
        p144.timestamp_SET(5190323864603679296L) ;
        p144.lat_SET(658709872) ;
        p144.position_cov_SET(new float[] {6.373067E37F, 2.8604832E38F, -3.7471118E37F}, 0) ;
        p144.acc_SET(new float[] {-1.0704405E37F, -2.6322245E38F, -6.853433E36F}, 0) ;
        p144.lon_SET(-1313739808) ;
        p144.attitude_q_SET(new float[] {-1.6467643E38F, -2.664329E38F, 2.4490525E38F, 6.62551E37F}, 0) ;
        p144.vel_SET(new float[] {-2.9744975E38F, -1.1064283E38F, 1.5976048E38F}, 0) ;
        p144.rates_SET(new float[] {-3.2571015E38F, 9.75238E37F, -1.4917778E38F}, 0) ;
        p144.alt_SET(-1.7852407E38F) ;
        p144.custom_state_SET(7259861527755437129L) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -2.8791327E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.3188012E38F, 4.096007E37F, -2.5955193E38F}));
            assert(pack.roll_rate_GET() == 3.1529766E38F);
            assert(pack.y_acc_GET() == 6.7101553E37F);
            assert(pack.z_acc_GET() == -1.4187518E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8127485E38F, -1.5655559E38F, -1.9487986E38F, 2.2781034E38F}));
            assert(pack.y_vel_GET() == 2.7873294E38F);
            assert(pack.z_vel_GET() == -3.4350155E37F);
            assert(pack.x_acc_GET() == -2.6365135E38F);
            assert(pack.airspeed_GET() == -2.6770661E38F);
            assert(pack.z_pos_GET() == 2.647583E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-9.565141E37F, -3.238972E38F, -1.4387652E38F}));
            assert(pack.y_pos_GET() == -8.0110295E37F);
            assert(pack.x_vel_GET() == 2.6330137E38F);
            assert(pack.time_usec_GET() == 4449135595521762404L);
            assert(pack.pitch_rate_GET() == -3.0530172E38F);
            assert(pack.x_pos_GET() == 2.3774442E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.roll_rate_SET(3.1529766E38F) ;
        p146.y_acc_SET(6.7101553E37F) ;
        p146.vel_variance_SET(new float[] {1.3188012E38F, 4.096007E37F, -2.5955193E38F}, 0) ;
        p146.x_vel_SET(2.6330137E38F) ;
        p146.yaw_rate_SET(-2.8791327E38F) ;
        p146.time_usec_SET(4449135595521762404L) ;
        p146.z_vel_SET(-3.4350155E37F) ;
        p146.pos_variance_SET(new float[] {-9.565141E37F, -3.238972E38F, -1.4387652E38F}, 0) ;
        p146.y_pos_SET(-8.0110295E37F) ;
        p146.z_acc_SET(-1.4187518E38F) ;
        p146.x_acc_SET(-2.6365135E38F) ;
        p146.x_pos_SET(2.3774442E38F) ;
        p146.y_vel_SET(2.7873294E38F) ;
        p146.z_pos_SET(2.647583E38F) ;
        p146.q_SET(new float[] {-1.8127485E38F, -1.5655559E38F, -1.9487986E38F, 2.2781034E38F}, 0) ;
        p146.airspeed_SET(-2.6770661E38F) ;
        p146.pitch_rate_SET(-3.0530172E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)232);
            assert(pack.current_battery_GET() == (short) -29148);
            assert(pack.battery_remaining_GET() == (byte) - 23);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.energy_consumed_GET() == 82462805);
            assert(pack.temperature_GET() == (short) -22441);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)64847, (char)3669, (char)4462, (char)40867, (char)61132, (char)28438, (char)20335, (char)39466, (char)25251, (char)5222}));
            assert(pack.current_consumed_GET() == -91570549);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_consumed_SET(-91570549) ;
        p147.current_battery_SET((short) -29148) ;
        p147.id_SET((char)232) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.energy_consumed_SET(82462805) ;
        p147.voltages_SET(new char[] {(char)64847, (char)3669, (char)4462, (char)40867, (char)61132, (char)28438, (char)20335, (char)39466, (char)25251, (char)5222}, 0) ;
        p147.temperature_SET((short) -22441) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.battery_remaining_SET((byte) - 23) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)13, (char)234, (char)105, (char)203, (char)240, (char)245, (char)39, (char)234}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)138, (char)113, (char)17, (char)39, (char)124, (char)4, (char)179, (char)160, (char)179, (char)21, (char)65, (char)161, (char)134, (char)132, (char)218, (char)88, (char)196, (char)131}));
            assert(pack.os_sw_version_GET() == 2480371588L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)144, (char)178, (char)194, (char)142, (char)58, (char)194, (char)8, (char)195}));
            assert(pack.middleware_sw_version_GET() == 2977744922L);
            assert(pack.flight_sw_version_GET() == 413736631L);
            assert(pack.product_id_GET() == (char)51501);
            assert(pack.uid_GET() == 7918125698834555888L);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT);
            assert(pack.vendor_id_GET() == (char)57402);
            assert(pack.board_version_GET() == 3882671824L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)204, (char)214, (char)70, (char)234, (char)174, (char)153, (char)175, (char)226}));
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)13, (char)234, (char)105, (char)203, (char)240, (char)245, (char)39, (char)234}, 0) ;
        p148.vendor_id_SET((char)57402) ;
        p148.uid_SET(7918125698834555888L) ;
        p148.middleware_sw_version_SET(2977744922L) ;
        p148.os_sw_version_SET(2480371588L) ;
        p148.uid2_SET(new char[] {(char)138, (char)113, (char)17, (char)39, (char)124, (char)4, (char)179, (char)160, (char)179, (char)21, (char)65, (char)161, (char)134, (char)132, (char)218, (char)88, (char)196, (char)131}, 0, PH) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT) ;
        p148.product_id_SET((char)51501) ;
        p148.flight_sw_version_SET(413736631L) ;
        p148.flight_custom_version_SET(new char[] {(char)144, (char)178, (char)194, (char)142, (char)58, (char)194, (char)8, (char)195}, 0) ;
        p148.board_version_SET(3882671824L) ;
        p148.os_custom_version_SET(new char[] {(char)204, (char)214, (char)70, (char)234, (char)174, (char)153, (char)175, (char)226}, 0) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.position_valid_TRY(ph) == (char)185);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.725865E38F, 1.9598744E38F, -2.4684506E37F, 2.8176332E38F}));
            assert(pack.x_TRY(ph) == -3.0979388E37F);
            assert(pack.z_TRY(ph) == 3.2635044E38F);
            assert(pack.size_y_GET() == -2.9928582E38F);
            assert(pack.angle_x_GET() == -2.8858145E38F);
            assert(pack.size_x_GET() == -3.3157953E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.y_TRY(ph) == -1.9461905E38F);
            assert(pack.target_num_GET() == (char)248);
            assert(pack.distance_GET() == -2.7397763E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.angle_y_GET() == -1.5697873E38F);
            assert(pack.time_usec_GET() == 414891939118823329L);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.target_num_SET((char)248) ;
        p149.x_SET(-3.0979388E37F, PH) ;
        p149.distance_SET(-2.7397763E38F) ;
        p149.position_valid_SET((char)185, PH) ;
        p149.size_y_SET(-2.9928582E38F) ;
        p149.z_SET(3.2635044E38F, PH) ;
        p149.angle_y_SET(-1.5697873E38F) ;
        p149.time_usec_SET(414891939118823329L) ;
        p149.size_x_SET(-3.3157953E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p149.q_SET(new float[] {2.725865E38F, 1.9598744E38F, -2.4684506E37F, 2.8176332E38F}, 0, PH) ;
        p149.angle_x_SET(-2.8858145E38F) ;
        p149.y_SET(-1.9461905E38F, PH) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)1);
            assert(pack.target_component_GET() == (char)162);
        });
        DemoDevice.FLEXIFUNCTION_SET p150 = LoopBackDemoChannel.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_component_SET((char)162) ;
        p150.target_system_SET((char)1) ;
        LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)109);
            assert(pack.read_req_type_GET() == (short) -16064);
            assert(pack.data_index_GET() == (short)10468);
            assert(pack.target_system_GET() == (char)198);
        });
        DemoDevice.FLEXIFUNCTION_READ_REQ p151 = LoopBackDemoChannel.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.data_index_SET((short)10468) ;
        p151.read_req_type_SET((short) -16064) ;
        p151.target_system_SET((char)198) ;
        p151.target_component_SET((char)109) ;
        LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)215);
            assert(pack.func_count_GET() == (char)1375);
            assert(pack.data_size_GET() == (char)60792);
            assert(Arrays.equals(pack.data__GET(),  new byte[] {(byte)126, (byte)72, (byte)31, (byte) - 110, (byte)126, (byte) - 83, (byte) - 112, (byte)95, (byte) - 27, (byte)48, (byte)1, (byte) - 54, (byte) - 47, (byte)36, (byte) - 75, (byte) - 51, (byte) - 56, (byte) - 5, (byte)107, (byte)82, (byte) - 68, (byte) - 5, (byte)102, (byte) - 8, (byte)67, (byte)89, (byte) - 109, (byte)2, (byte)71, (byte) - 26, (byte)119, (byte) - 6, (byte)24, (byte) - 24, (byte) - 103, (byte)24, (byte)75, (byte)97, (byte) - 7, (byte) - 90, (byte)38, (byte) - 35, (byte)111, (byte) - 59, (byte)61, (byte) - 74, (byte) - 111, (byte)99}));
            assert(pack.func_index_GET() == (char)11709);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.data_address_GET() == (char)20928);
        });
        DemoDevice.FLEXIFUNCTION_BUFFER_FUNCTION p152 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.data_address_SET((char)20928) ;
        p152.target_component_SET((char)215) ;
        p152.func_index_SET((char)11709) ;
        p152.data__SET(new byte[] {(byte)126, (byte)72, (byte)31, (byte) - 110, (byte)126, (byte) - 83, (byte) - 112, (byte)95, (byte) - 27, (byte)48, (byte)1, (byte) - 54, (byte) - 47, (byte)36, (byte) - 75, (byte) - 51, (byte) - 56, (byte) - 5, (byte)107, (byte)82, (byte) - 68, (byte) - 5, (byte)102, (byte) - 8, (byte)67, (byte)89, (byte) - 109, (byte)2, (byte)71, (byte) - 26, (byte)119, (byte) - 6, (byte)24, (byte) - 24, (byte) - 103, (byte)24, (byte)75, (byte)97, (byte) - 7, (byte) - 90, (byte)38, (byte) - 35, (byte)111, (byte) - 59, (byte)61, (byte) - 74, (byte) - 111, (byte)99}, 0) ;
        p152.data_size_SET((char)60792) ;
        p152.target_system_SET((char)124) ;
        p152.func_count_SET((char)1375) ;
        LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)68);
            assert(pack.func_index_GET() == (char)9426);
            assert(pack.result_GET() == (char)49952);
            assert(pack.target_system_GET() == (char)15);
        });
        DemoDevice.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.result_SET((char)49952) ;
        p153.func_index_SET((char)9426) ;
        p153.target_component_SET((char)68) ;
        p153.target_system_SET((char)15) ;
        LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (char)30);
            assert(Arrays.equals(pack.directory_data_GET(),  new byte[] {(byte)92, (byte)100, (byte) - 33, (byte)59, (byte)91, (byte) - 43, (byte)81, (byte) - 101, (byte)20, (byte) - 31, (byte) - 48, (byte) - 116, (byte) - 68, (byte)47, (byte)49, (byte) - 38, (byte)1, (byte)56, (byte)24, (byte) - 63, (byte) - 11, (byte)34, (byte)16, (byte) - 2, (byte)54, (byte) - 39, (byte)76, (byte) - 5, (byte)31, (byte)58, (byte) - 76, (byte) - 88, (byte) - 84, (byte) - 83, (byte)83, (byte)113, (byte) - 18, (byte) - 92, (byte)0, (byte)9, (byte) - 48, (byte)45, (byte) - 105, (byte) - 98, (byte)102, (byte) - 122, (byte) - 10, (byte)79}));
            assert(pack.count_GET() == (char)203);
            assert(pack.target_system_GET() == (char)86);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.directory_type_GET() == (char)213);
        });
        DemoDevice.FLEXIFUNCTION_DIRECTORY p155 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.start_index_SET((char)30) ;
        p155.directory_data_SET(new byte[] {(byte)92, (byte)100, (byte) - 33, (byte)59, (byte)91, (byte) - 43, (byte)81, (byte) - 101, (byte)20, (byte) - 31, (byte) - 48, (byte) - 116, (byte) - 68, (byte)47, (byte)49, (byte) - 38, (byte)1, (byte)56, (byte)24, (byte) - 63, (byte) - 11, (byte)34, (byte)16, (byte) - 2, (byte)54, (byte) - 39, (byte)76, (byte) - 5, (byte)31, (byte)58, (byte) - 76, (byte) - 88, (byte) - 84, (byte) - 83, (byte)83, (byte)113, (byte) - 18, (byte) - 92, (byte)0, (byte)9, (byte) - 48, (byte)45, (byte) - 105, (byte) - 98, (byte)102, (byte) - 122, (byte) - 10, (byte)79}, 0) ;
        p155.directory_type_SET((char)213) ;
        p155.target_component_SET((char)103) ;
        p155.count_SET((char)203) ;
        p155.target_system_SET((char)86) ;
        LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == (char)22027);
            assert(pack.start_index_GET() == (char)220);
            assert(pack.count_GET() == (char)73);
            assert(pack.directory_type_GET() == (char)17);
            assert(pack.target_component_GET() == (char)69);
            assert(pack.target_system_GET() == (char)19);
        });
        DemoDevice.FLEXIFUNCTION_DIRECTORY_ACK p156 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.count_SET((char)73) ;
        p156.target_component_SET((char)69) ;
        p156.result_SET((char)22027) ;
        p156.start_index_SET((char)220) ;
        p156.directory_type_SET((char)17) ;
        p156.target_system_SET((char)19) ;
        LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)13);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.command_type_GET() == (char)107);
        });
        DemoDevice.FLEXIFUNCTION_COMMAND p157 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)13) ;
        p157.command_type_SET((char)107) ;
        p157.target_component_SET((char)37) ;
        LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.command_type_GET() == (char)63020);
            assert(pack.result_GET() == (char)7211);
        });
        DemoDevice.FLEXIFUNCTION_COMMAND_ACK p158 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)63020) ;
        p158.result_SET((char)7211) ;
        LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
        {
            assert(pack.sue_sog_GET() == (short)10248);
            assert(pack.sue_longitude_GET() == -1314501814);
            assert(pack.sue_rmat1_GET() == (short)1871);
            assert(pack.sue_waypoint_index_GET() == (char)54105);
            assert(pack.sue_magFieldEarth0_GET() == (short)3378);
            assert(pack.sue_time_GET() == 3537599333L);
            assert(pack.sue_rmat8_GET() == (short)9199);
            assert(pack.sue_air_speed_3DIMU_GET() == (char)39529);
            assert(pack.sue_cpu_load_GET() == (char)45335);
            assert(pack.sue_cog_GET() == (char)20137);
            assert(pack.sue_estimated_wind_1_GET() == (short) -2710);
            assert(pack.sue_svs_GET() == (short) -19946);
            assert(pack.sue_rmat0_GET() == (short) -31814);
            assert(pack.sue_rmat5_GET() == (short) -7455);
            assert(pack.sue_rmat2_GET() == (short) -11038);
            assert(pack.sue_altitude_GET() == -615752754);
            assert(pack.sue_estimated_wind_0_GET() == (short)31536);
            assert(pack.sue_rmat6_GET() == (short) -11842);
            assert(pack.sue_rmat4_GET() == (short)23883);
            assert(pack.sue_latitude_GET() == -1916254025);
            assert(pack.sue_status_GET() == (char)20);
            assert(pack.sue_rmat3_GET() == (short) -25054);
            assert(pack.sue_magFieldEarth2_GET() == (short)24851);
            assert(pack.sue_rmat7_GET() == (short)18466);
            assert(pack.sue_hdop_GET() == (short) -1674);
            assert(pack.sue_estimated_wind_2_GET() == (short) -12984);
            assert(pack.sue_magFieldEarth1_GET() == (short) -30792);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F2_A p170 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_rmat7_SET((short)18466) ;
        p170.sue_sog_SET((short)10248) ;
        p170.sue_estimated_wind_2_SET((short) -12984) ;
        p170.sue_air_speed_3DIMU_SET((char)39529) ;
        p170.sue_latitude_SET(-1916254025) ;
        p170.sue_longitude_SET(-1314501814) ;
        p170.sue_cog_SET((char)20137) ;
        p170.sue_waypoint_index_SET((char)54105) ;
        p170.sue_rmat3_SET((short) -25054) ;
        p170.sue_magFieldEarth2_SET((short)24851) ;
        p170.sue_rmat4_SET((short)23883) ;
        p170.sue_estimated_wind_0_SET((short)31536) ;
        p170.sue_status_SET((char)20) ;
        p170.sue_magFieldEarth1_SET((short) -30792) ;
        p170.sue_svs_SET((short) -19946) ;
        p170.sue_time_SET(3537599333L) ;
        p170.sue_rmat6_SET((short) -11842) ;
        p170.sue_magFieldEarth0_SET((short)3378) ;
        p170.sue_rmat0_SET((short) -31814) ;
        p170.sue_estimated_wind_1_SET((short) -2710) ;
        p170.sue_rmat2_SET((short) -11038) ;
        p170.sue_rmat1_SET((short)1871) ;
        p170.sue_rmat5_SET((short) -7455) ;
        p170.sue_rmat8_SET((short)9199) ;
        p170.sue_altitude_SET(-615752754) ;
        p170.sue_hdop_SET((short) -1674) ;
        p170.sue_cpu_load_SET((char)45335) ;
        LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
        {
            assert(pack.sue_barom_alt_GET() == -1998677432);
            assert(pack.sue_pwm_output_5_GET() == (short)5501);
            assert(pack.sue_waypoint_goal_z_GET() == (short) -30823);
            assert(pack.sue_memory_stack_free_GET() == (short)3236);
            assert(pack.sue_pwm_input_6_GET() == (short)14039);
            assert(pack.sue_location_error_earth_x_GET() == (short)3091);
            assert(pack.sue_pwm_input_11_GET() == (short)21786);
            assert(pack.sue_bat_amp_GET() == (short) -23976);
            assert(pack.sue_pwm_input_4_GET() == (short)30326);
            assert(pack.sue_imu_velocity_y_GET() == (short)27929);
            assert(pack.sue_desired_height_GET() == (short) -14505);
            assert(pack.sue_pwm_output_3_GET() == (short) -31422);
            assert(pack.sue_imu_location_z_GET() == (short) -3892);
            assert(pack.sue_pwm_output_10_GET() == (short)21422);
            assert(pack.sue_pwm_output_12_GET() == (short) -29031);
            assert(pack.sue_imu_location_x_GET() == (short) -1523);
            assert(pack.sue_osc_fails_GET() == (short) -5645);
            assert(pack.sue_barom_press_GET() == -1200570714);
            assert(pack.sue_pwm_output_8_GET() == (short) -24885);
            assert(pack.sue_pwm_output_7_GET() == (short) -22629);
            assert(pack.sue_pwm_input_3_GET() == (short)2463);
            assert(pack.sue_pwm_input_1_GET() == (short)14653);
            assert(pack.sue_pwm_output_2_GET() == (short)9079);
            assert(pack.sue_location_error_earth_z_GET() == (short) -5668);
            assert(pack.sue_pwm_input_10_GET() == (short)10338);
            assert(pack.sue_pwm_input_12_GET() == (short)841);
            assert(pack.sue_pwm_output_6_GET() == (short) -27110);
            assert(pack.sue_waypoint_goal_y_GET() == (short) -29563);
            assert(pack.sue_pwm_input_7_GET() == (short) -1380);
            assert(pack.sue_pwm_input_5_GET() == (short) -4073);
            assert(pack.sue_imu_location_y_GET() == (short) -21931);
            assert(pack.sue_flags_GET() == 2872449926L);
            assert(pack.sue_aero_z_GET() == (short)20191);
            assert(pack.sue_pwm_input_8_GET() == (short) -12508);
            assert(pack.sue_barom_temp_GET() == (short)28764);
            assert(pack.sue_aero_x_GET() == (short)1753);
            assert(pack.sue_pwm_output_11_GET() == (short)9845);
            assert(pack.sue_bat_volt_GET() == (short)25394);
            assert(pack.sue_aero_y_GET() == (short)13668);
            assert(pack.sue_pwm_input_9_GET() == (short)20152);
            assert(pack.sue_pwm_output_1_GET() == (short) -8838);
            assert(pack.sue_imu_velocity_x_GET() == (short)30265);
            assert(pack.sue_pwm_output_9_GET() == (short) -32210);
            assert(pack.sue_waypoint_goal_x_GET() == (short)23675);
            assert(pack.sue_bat_amp_hours_GET() == (short)20945);
            assert(pack.sue_pwm_input_2_GET() == (short)5132);
            assert(pack.sue_imu_velocity_z_GET() == (short)5228);
            assert(pack.sue_pwm_output_4_GET() == (short) -3178);
            assert(pack.sue_time_GET() == 2562495191L);
            assert(pack.sue_location_error_earth_y_GET() == (short) -23387);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F2_B p171 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_waypoint_goal_y_SET((short) -29563) ;
        p171.sue_waypoint_goal_z_SET((short) -30823) ;
        p171.sue_pwm_input_7_SET((short) -1380) ;
        p171.sue_time_SET(2562495191L) ;
        p171.sue_desired_height_SET((short) -14505) ;
        p171.sue_imu_velocity_y_SET((short)27929) ;
        p171.sue_pwm_input_10_SET((short)10338) ;
        p171.sue_pwm_output_5_SET((short)5501) ;
        p171.sue_pwm_output_2_SET((short)9079) ;
        p171.sue_location_error_earth_y_SET((short) -23387) ;
        p171.sue_location_error_earth_z_SET((short) -5668) ;
        p171.sue_aero_z_SET((short)20191) ;
        p171.sue_bat_volt_SET((short)25394) ;
        p171.sue_pwm_input_3_SET((short)2463) ;
        p171.sue_pwm_input_9_SET((short)20152) ;
        p171.sue_pwm_input_5_SET((short) -4073) ;
        p171.sue_pwm_output_1_SET((short) -8838) ;
        p171.sue_pwm_output_10_SET((short)21422) ;
        p171.sue_pwm_output_11_SET((short)9845) ;
        p171.sue_barom_temp_SET((short)28764) ;
        p171.sue_pwm_output_6_SET((short) -27110) ;
        p171.sue_pwm_output_7_SET((short) -22629) ;
        p171.sue_pwm_output_12_SET((short) -29031) ;
        p171.sue_pwm_output_3_SET((short) -31422) ;
        p171.sue_pwm_input_6_SET((short)14039) ;
        p171.sue_aero_y_SET((short)13668) ;
        p171.sue_pwm_output_8_SET((short) -24885) ;
        p171.sue_pwm_input_12_SET((short)841) ;
        p171.sue_pwm_input_2_SET((short)5132) ;
        p171.sue_bat_amp_hours_SET((short)20945) ;
        p171.sue_pwm_input_1_SET((short)14653) ;
        p171.sue_imu_velocity_z_SET((short)5228) ;
        p171.sue_pwm_input_11_SET((short)21786) ;
        p171.sue_osc_fails_SET((short) -5645) ;
        p171.sue_imu_location_x_SET((short) -1523) ;
        p171.sue_imu_location_y_SET((short) -21931) ;
        p171.sue_location_error_earth_x_SET((short)3091) ;
        p171.sue_bat_amp_SET((short) -23976) ;
        p171.sue_pwm_input_4_SET((short)30326) ;
        p171.sue_imu_velocity_x_SET((short)30265) ;
        p171.sue_pwm_output_9_SET((short) -32210) ;
        p171.sue_flags_SET(2872449926L) ;
        p171.sue_pwm_input_8_SET((short) -12508) ;
        p171.sue_waypoint_goal_x_SET((short)23675) ;
        p171.sue_barom_press_SET(-1200570714) ;
        p171.sue_barom_alt_SET(-1998677432) ;
        p171.sue_memory_stack_free_SET((short)3236) ;
        p171.sue_imu_location_z_SET((short) -3892) ;
        p171.sue_aero_x_SET((short)1753) ;
        p171.sue_pwm_output_4_SET((short) -3178) ;
        LoopBackDemoChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
        {
            assert(pack.sue_YAW_STABILIZATION_AILERON_GET() == (char)29);
            assert(pack.sue_ALTITUDEHOLD_STABILIZED_GET() == (char)98);
            assert(pack.sue_PITCH_STABILIZATION_GET() == (char)225);
            assert(pack.sue_RUDDER_NAVIGATION_GET() == (char)207);
            assert(pack.sue_YAW_STABILIZATION_RUDDER_GET() == (char)25);
            assert(pack.sue_ALTITUDEHOLD_WAYPOINT_GET() == (char)41);
            assert(pack.sue_RACING_MODE_GET() == (char)87);
            assert(pack.sue_ROLL_STABILIZATION_RUDDER_GET() == (char)107);
            assert(pack.sue_ROLL_STABILIZATION_AILERONS_GET() == (char)115);
            assert(pack.sue_AILERON_NAVIGATION_GET() == (char)229);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F4 p172 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)41) ;
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)29) ;
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)115) ;
        p172.sue_RACING_MODE_SET((char)87) ;
        p172.sue_PITCH_STABILIZATION_SET((char)225) ;
        p172.sue_RUDDER_NAVIGATION_SET((char)207) ;
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)25) ;
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)98) ;
        p172.sue_AILERON_NAVIGATION_SET((char)229) ;
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)107) ;
        LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
        {
            assert(pack.sue_YAWKP_AILERON_GET() == -1.6328907E38F);
            assert(pack.sue_YAWKD_AILERON_GET() == 3.2701333E38F);
            assert(pack.sue_ROLLKD_GET() == 2.717976E38F);
            assert(pack.sue_ROLLKP_GET() == 1.6549326E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F5 p173 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_ROLLKP_SET(1.6549326E38F) ;
        p173.sue_ROLLKD_SET(2.717976E38F) ;
        p173.sue_YAWKD_AILERON_SET(3.2701333E38F) ;
        p173.sue_YAWKP_AILERON_SET(-1.6328907E38F) ;
        LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
        {
            assert(pack.sue_ROLL_ELEV_MIX_GET() == 3.1038084E38F);
            assert(pack.sue_PITCHGAIN_GET() == 2.797508E38F);
            assert(pack.sue_RUDDER_ELEV_MIX_GET() == 1.5595988E38F);
            assert(pack.sue_PITCHKD_GET() == 3.3407938E38F);
            assert(pack.sue_ELEVATOR_BOOST_GET() == -3.0989465E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F6 p174 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_PITCHKD_SET(3.3407938E38F) ;
        p174.sue_ROLL_ELEV_MIX_SET(3.1038084E38F) ;
        p174.sue_RUDDER_ELEV_MIX_SET(1.5595988E38F) ;
        p174.sue_PITCHGAIN_SET(2.797508E38F) ;
        p174.sue_ELEVATOR_BOOST_SET(-3.0989465E38F) ;
        LoopBackDemoChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
        {
            assert(pack.sue_YAWKD_RUDDER_GET() == -1.1882034E38F);
            assert(pack.sue_RUDDER_BOOST_GET() == -1.3951543E38F);
            assert(pack.sue_ROLLKP_RUDDER_GET() == 1.2120027E38F);
            assert(pack.sue_YAWKP_RUDDER_GET() == 1.0732196E38F);
            assert(pack.sue_RTL_PITCH_DOWN_GET() == 6.6398773E37F);
            assert(pack.sue_ROLLKD_RUDDER_GET() == -3.3911055E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F7 p175 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_ROLLKD_RUDDER_SET(-3.3911055E38F) ;
        p175.sue_RTL_PITCH_DOWN_SET(6.6398773E37F) ;
        p175.sue_ROLLKP_RUDDER_SET(1.2120027E38F) ;
        p175.sue_YAWKD_RUDDER_SET(-1.1882034E38F) ;
        p175.sue_RUDDER_BOOST_SET(-1.3951543E38F) ;
        p175.sue_YAWKP_RUDDER_SET(1.0732196E38F) ;
        LoopBackDemoChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
        {
            assert(pack.sue_ALT_HOLD_PITCH_HIGH_GET() == 1.1703569E38F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MAX_GET() == 7.533178E37F);
            assert(pack.sue_HEIGHT_TARGET_MIN_GET() == 1.5308141E38F);
            assert(pack.sue_HEIGHT_TARGET_MAX_GET() == -8.819522E37F);
            assert(pack.sue_ALT_HOLD_PITCH_MIN_GET() == -6.92264E37F);
            assert(pack.sue_ALT_HOLD_THROTTLE_MIN_GET() == -1.3150153E38F);
            assert(pack.sue_ALT_HOLD_PITCH_MAX_GET() == 2.2268402E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F8 p176 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_ALT_HOLD_PITCH_MAX_SET(2.2268402E38F) ;
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-1.3150153E38F) ;
        p176.sue_HEIGHT_TARGET_MIN_SET(1.5308141E38F) ;
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(1.1703569E38F) ;
        p176.sue_HEIGHT_TARGET_MAX_SET(-8.819522E37F) ;
        p176.sue_ALT_HOLD_PITCH_MIN_SET(-6.92264E37F) ;
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(7.533178E37F) ;
        LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
        {
            assert(pack.sue_alt_origin_GET() == -1735636130);
            assert(pack.sue_week_no_GET() == (short)22606);
            assert(pack.sue_lon_origin_GET() == -1499255025);
            assert(pack.sue_lat_origin_GET() == -189333821);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F13 p177 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_alt_origin_SET(-1735636130) ;
        p177.sue_week_no_SET((short)22606) ;
        p177.sue_lon_origin_SET(-1499255025) ;
        p177.sue_lat_origin_SET(-189333821) ;
        LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
        {
            assert(pack.sue_FLIGHT_PLAN_TYPE_GET() == (char)144);
            assert(pack.sue_TRAP_FLAGS_GET() == (short) -32390);
            assert(pack.sue_AIRFRAME_GET() == (char)26);
            assert(pack.sue_WIND_ESTIMATION_GET() == (char)133);
            assert(pack.sue_RCON_GET() == (short)19963);
            assert(pack.sue_CLOCK_CONFIG_GET() == (char)156);
            assert(pack.sue_osc_fail_count_GET() == (short)30199);
            assert(pack.sue_GPS_TYPE_GET() == (char)48);
            assert(pack.sue_TRAP_SOURCE_GET() == 2090968139L);
            assert(pack.sue_BOARD_TYPE_GET() == (char)41);
            assert(pack.sue_DR_GET() == (char)177);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F14 p178 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_osc_fail_count_SET((short)30199) ;
        p178.sue_TRAP_FLAGS_SET((short) -32390) ;
        p178.sue_WIND_ESTIMATION_SET((char)133) ;
        p178.sue_GPS_TYPE_SET((char)48) ;
        p178.sue_CLOCK_CONFIG_SET((char)156) ;
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)144) ;
        p178.sue_TRAP_SOURCE_SET(2090968139L) ;
        p178.sue_RCON_SET((short)19963) ;
        p178.sue_DR_SET((char)177) ;
        p178.sue_BOARD_TYPE_SET((char)41) ;
        p178.sue_AIRFRAME_SET((char)26) ;
        LoopBackDemoChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_VEHICLE_MODEL_NAME_GET(),  new char[] {(char)174, (char)48, (char)122, (char)69, (char)238, (char)202, (char)128, (char)27, (char)74, (char)88, (char)19, (char)176, (char)211, (char)217, (char)177, (char)50, (char)56, (char)134, (char)119, (char)137, (char)47, (char)3, (char)167, (char)157, (char)107, (char)131, (char)54, (char)26, (char)231, (char)251, (char)220, (char)24, (char)11, (char)50, (char)200, (char)43, (char)206, (char)134, (char)245, (char)113}));
            assert(Arrays.equals(pack.sue_ID_VEHICLE_REGISTRATION_GET(),  new char[] {(char)131, (char)157, (char)133, (char)254, (char)208, (char)82, (char)201, (char)150, (char)254, (char)119, (char)120, (char)234, (char)13, (char)72, (char)61, (char)186, (char)109, (char)113, (char)93, (char)43}));
        });
        DemoDevice.SERIAL_UDB_EXTRA_F15 p179 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[] {(char)174, (char)48, (char)122, (char)69, (char)238, (char)202, (char)128, (char)27, (char)74, (char)88, (char)19, (char)176, (char)211, (char)217, (char)177, (char)50, (char)56, (char)134, (char)119, (char)137, (char)47, (char)3, (char)167, (char)157, (char)107, (char)131, (char)54, (char)26, (char)231, (char)251, (char)220, (char)24, (char)11, (char)50, (char)200, (char)43, (char)206, (char)134, (char)245, (char)113}, 0) ;
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[] {(char)131, (char)157, (char)133, (char)254, (char)208, (char)82, (char)201, (char)150, (char)254, (char)119, (char)120, (char)234, (char)13, (char)72, (char)61, (char)186, (char)109, (char)113, (char)93, (char)43}, 0) ;
        LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.sue_ID_DIY_DRONES_URL_GET(),  new char[] {(char)254, (char)6, (char)196, (char)187, (char)123, (char)216, (char)173, (char)230, (char)159, (char)224, (char)214, (char)217, (char)34, (char)212, (char)235, (char)157, (char)7, (char)90, (char)20, (char)114, (char)99, (char)175, (char)41, (char)190, (char)186, (char)83, (char)25, (char)49, (char)201, (char)19, (char)187, (char)72, (char)194, (char)240, (char)213, (char)70, (char)202, (char)238, (char)48, (char)165, (char)72, (char)142, (char)124, (char)109, (char)80, (char)222, (char)60, (char)56, (char)251, (char)106, (char)124, (char)122, (char)164, (char)236, (char)200, (char)207, (char)45, (char)167, (char)169, (char)223, (char)119, (char)89, (char)240, (char)98, (char)89, (char)154, (char)66, (char)146, (char)50, (char)49}));
            assert(Arrays.equals(pack.sue_ID_LEAD_PILOT_GET(),  new char[] {(char)145, (char)29, (char)40, (char)205, (char)152, (char)53, (char)52, (char)133, (char)50, (char)183, (char)230, (char)140, (char)247, (char)183, (char)113, (char)164, (char)100, (char)36, (char)215, (char)69, (char)94, (char)2, (char)223, (char)145, (char)119, (char)87, (char)71, (char)142, (char)163, (char)183, (char)128, (char)67, (char)60, (char)231, (char)85, (char)55, (char)184, (char)22, (char)29, (char)170}));
        });
        DemoDevice.SERIAL_UDB_EXTRA_F16 p180 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[] {(char)145, (char)29, (char)40, (char)205, (char)152, (char)53, (char)52, (char)133, (char)50, (char)183, (char)230, (char)140, (char)247, (char)183, (char)113, (char)164, (char)100, (char)36, (char)215, (char)69, (char)94, (char)2, (char)223, (char)145, (char)119, (char)87, (char)71, (char)142, (char)163, (char)183, (char)128, (char)67, (char)60, (char)231, (char)85, (char)55, (char)184, (char)22, (char)29, (char)170}, 0) ;
        p180.sue_ID_DIY_DRONES_URL_SET(new char[] {(char)254, (char)6, (char)196, (char)187, (char)123, (char)216, (char)173, (char)230, (char)159, (char)224, (char)214, (char)217, (char)34, (char)212, (char)235, (char)157, (char)7, (char)90, (char)20, (char)114, (char)99, (char)175, (char)41, (char)190, (char)186, (char)83, (char)25, (char)49, (char)201, (char)19, (char)187, (char)72, (char)194, (char)240, (char)213, (char)70, (char)202, (char)238, (char)48, (char)165, (char)72, (char)142, (char)124, (char)109, (char)80, (char)222, (char)60, (char)56, (char)251, (char)106, (char)124, (char)122, (char)164, (char)236, (char)200, (char)207, (char)45, (char)167, (char)169, (char)223, (char)119, (char)89, (char)240, (char)98, (char)89, (char)154, (char)66, (char)146, (char)50, (char)49}, 0) ;
        LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
        {
            assert(pack.alt_gps_GET() == -355467088);
            assert(pack.alt_barometric_GET() == 898082319);
            assert(pack.time_boot_ms_GET() == 1075642823L);
            assert(pack.alt_range_finder_GET() == 1398933933);
            assert(pack.alt_extra_GET() == 792059714);
            assert(pack.alt_optical_flow_GET() == -1771309002);
            assert(pack.alt_imu_GET() == 1934783338);
        });
        DemoDevice.ALTITUDES p181 = LoopBackDemoChannel.new_ALTITUDES();
        PH.setPack(p181);
        p181.alt_extra_SET(792059714) ;
        p181.alt_gps_SET(-355467088) ;
        p181.alt_range_finder_SET(1398933933) ;
        p181.alt_optical_flow_SET(-1771309002) ;
        p181.alt_imu_SET(1934783338) ;
        p181.time_boot_ms_SET(1075642823L) ;
        p181.alt_barometric_SET(898082319) ;
        LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
        {
            assert(pack.aoa_GET() == (short)15040);
            assert(pack.airspeed_imu_GET() == (short) -969);
            assert(pack.airspeed_ultrasonic_GET() == (short) -25804);
            assert(pack.airspeed_pitot_GET() == (short)21987);
            assert(pack.airspeed_hot_wire_GET() == (short) -17462);
            assert(pack.time_boot_ms_GET() == 1393101511L);
            assert(pack.aoy_GET() == (short) -31100);
        });
        DemoDevice.AIRSPEEDS p182 = LoopBackDemoChannel.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.airspeed_imu_SET((short) -969) ;
        p182.airspeed_ultrasonic_SET((short) -25804) ;
        p182.time_boot_ms_SET(1393101511L) ;
        p182.aoa_SET((short)15040) ;
        p182.aoy_SET((short) -31100) ;
        p182.airspeed_pitot_SET((short)21987) ;
        p182.airspeed_hot_wire_SET((short) -17462) ;
        LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
        {
            assert(pack.sue_feed_forward_GET() == -5.186174E37F);
            assert(pack.sue_turn_rate_fbw_GET() == -2.4939663E38F);
            assert(pack.sue_turn_rate_nav_GET() == 3.1434015E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F17 p183 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_feed_forward_SET(-5.186174E37F) ;
        p183.sue_turn_rate_nav_SET(3.1434015E38F) ;
        p183.sue_turn_rate_fbw_SET(-2.4939663E38F) ;
        LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
        {
            assert(pack.elevator_trim_normal_GET() == 3.2889856E38F);
            assert(pack.elevator_trim_inverted_GET() == 2.0597815E38F);
            assert(pack.reference_speed_GET() == 2.8837579E38F);
            assert(pack.angle_of_attack_inverted_GET() == -3.3730412E38F);
            assert(pack.angle_of_attack_normal_GET() == 2.6262857E38F);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F18 p184 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.reference_speed_SET(2.8837579E38F) ;
        p184.elevator_trim_inverted_SET(2.0597815E38F) ;
        p184.angle_of_attack_inverted_SET(-3.3730412E38F) ;
        p184.elevator_trim_normal_SET(3.2889856E38F) ;
        p184.angle_of_attack_normal_SET(2.6262857E38F) ;
        LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
        {
            assert(pack.sue_elevator_output_channel_GET() == (char)51);
            assert(pack.sue_throttle_reversed_GET() == (char)65);
            assert(pack.sue_rudder_output_channel_GET() == (char)27);
            assert(pack.sue_throttle_output_channel_GET() == (char)63);
            assert(pack.sue_aileron_reversed_GET() == (char)234);
            assert(pack.sue_elevator_reversed_GET() == (char)138);
            assert(pack.sue_rudder_reversed_GET() == (char)217);
            assert(pack.sue_aileron_output_channel_GET() == (char)4);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F19 p185 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_throttle_output_channel_SET((char)63) ;
        p185.sue_elevator_output_channel_SET((char)51) ;
        p185.sue_rudder_reversed_SET((char)217) ;
        p185.sue_elevator_reversed_SET((char)138) ;
        p185.sue_throttle_reversed_SET((char)65) ;
        p185.sue_aileron_reversed_SET((char)234) ;
        p185.sue_rudder_output_channel_SET((char)27) ;
        p185.sue_aileron_output_channel_SET((char)4) ;
        LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
        {
            assert(pack.sue_trim_value_input_9_GET() == (short) -18622);
            assert(pack.sue_trim_value_input_3_GET() == (short)2157);
            assert(pack.sue_trim_value_input_1_GET() == (short)20295);
            assert(pack.sue_trim_value_input_8_GET() == (short)24558);
            assert(pack.sue_trim_value_input_12_GET() == (short) -17044);
            assert(pack.sue_trim_value_input_7_GET() == (short)4778);
            assert(pack.sue_trim_value_input_4_GET() == (short) -11628);
            assert(pack.sue_trim_value_input_6_GET() == (short)26886);
            assert(pack.sue_trim_value_input_11_GET() == (short) -16031);
            assert(pack.sue_trim_value_input_5_GET() == (short) -25734);
            assert(pack.sue_number_of_inputs_GET() == (char)162);
            assert(pack.sue_trim_value_input_2_GET() == (short) -24003);
            assert(pack.sue_trim_value_input_10_GET() == (short) -240);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F20 p186 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_trim_value_input_5_SET((short) -25734) ;
        p186.sue_trim_value_input_4_SET((short) -11628) ;
        p186.sue_trim_value_input_3_SET((short)2157) ;
        p186.sue_trim_value_input_10_SET((short) -240) ;
        p186.sue_number_of_inputs_SET((char)162) ;
        p186.sue_trim_value_input_2_SET((short) -24003) ;
        p186.sue_trim_value_input_11_SET((short) -16031) ;
        p186.sue_trim_value_input_12_SET((short) -17044) ;
        p186.sue_trim_value_input_7_SET((short)4778) ;
        p186.sue_trim_value_input_8_SET((short)24558) ;
        p186.sue_trim_value_input_1_SET((short)20295) ;
        p186.sue_trim_value_input_6_SET((short)26886) ;
        p186.sue_trim_value_input_9_SET((short) -18622) ;
        LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
        {
            assert(pack.sue_accel_y_offset_GET() == (short) -2250);
            assert(pack.sue_gyro_y_offset_GET() == (short) -25546);
            assert(pack.sue_accel_x_offset_GET() == (short)28489);
            assert(pack.sue_accel_z_offset_GET() == (short)19994);
            assert(pack.sue_gyro_z_offset_GET() == (short) -28042);
            assert(pack.sue_gyro_x_offset_GET() == (short)5079);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F21 p187 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_gyro_y_offset_SET((short) -25546) ;
        p187.sue_gyro_x_offset_SET((short)5079) ;
        p187.sue_gyro_z_offset_SET((short) -28042) ;
        p187.sue_accel_x_offset_SET((short)28489) ;
        p187.sue_accel_z_offset_SET((short)19994) ;
        p187.sue_accel_y_offset_SET((short) -2250) ;
        LoopBackDemoChannel.instance.send(p187);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
        {
            assert(pack.sue_accel_z_at_calibration_GET() == (short)23201);
            assert(pack.sue_accel_y_at_calibration_GET() == (short)7858);
            assert(pack.sue_gyro_z_at_calibration_GET() == (short) -28436);
            assert(pack.sue_accel_x_at_calibration_GET() == (short) -7610);
            assert(pack.sue_gyro_y_at_calibration_GET() == (short) -17459);
            assert(pack.sue_gyro_x_at_calibration_GET() == (short) -1960);
        });
        DemoDevice.SERIAL_UDB_EXTRA_F22 p188 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_y_at_calibration_SET((short)7858) ;
        p188.sue_accel_z_at_calibration_SET((short)23201) ;
        p188.sue_gyro_y_at_calibration_SET((short) -17459) ;
        p188.sue_gyro_x_at_calibration_SET((short) -1960) ;
        p188.sue_accel_x_at_calibration_SET((short) -7610) ;
        p188.sue_gyro_z_at_calibration_SET((short) -28436) ;
        LoopBackDemoChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.hagl_ratio_GET() == -2.3708165E38F);
            assert(pack.mag_ratio_GET() == 2.69182E38F);
            assert(pack.tas_ratio_GET() == -1.051607E38F);
            assert(pack.pos_horiz_ratio_GET() == 2.9356792E38F);
            assert(pack.time_usec_GET() == 2604197832121638418L);
            assert(pack.pos_horiz_accuracy_GET() == -4.6801708E36F);
            assert(pack.pos_vert_ratio_GET() == 3.337522E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS);
            assert(pack.pos_vert_accuracy_GET() == -2.4834955E38F);
            assert(pack.vel_ratio_GET() == 5.576722E37F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_ratio_SET(2.9356792E38F) ;
        p230.time_usec_SET(2604197832121638418L) ;
        p230.tas_ratio_SET(-1.051607E38F) ;
        p230.pos_vert_accuracy_SET(-2.4834955E38F) ;
        p230.pos_horiz_accuracy_SET(-4.6801708E36F) ;
        p230.vel_ratio_SET(5.576722E37F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS) ;
        p230.pos_vert_ratio_SET(3.337522E38F) ;
        p230.mag_ratio_SET(2.69182E38F) ;
        p230.hagl_ratio_SET(-2.3708165E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == -1.4381392E38F);
            assert(pack.wind_alt_GET() == -1.7388511E38F);
            assert(pack.vert_accuracy_GET() == -1.9145655E37F);
            assert(pack.wind_z_GET() == 9.300222E37F);
            assert(pack.time_usec_GET() == 1310064606308458594L);
            assert(pack.var_horiz_GET() == 2.32738E38F);
            assert(pack.var_vert_GET() == 2.2956006E38F);
            assert(pack.horiz_accuracy_GET() == -2.6139103E38F);
            assert(pack.wind_y_GET() == 1.5573439E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.horiz_accuracy_SET(-2.6139103E38F) ;
        p231.wind_alt_SET(-1.7388511E38F) ;
        p231.wind_x_SET(-1.4381392E38F) ;
        p231.var_horiz_SET(2.32738E38F) ;
        p231.wind_y_SET(1.5573439E38F) ;
        p231.var_vert_SET(2.2956006E38F) ;
        p231.vert_accuracy_SET(-1.9145655E37F) ;
        p231.wind_z_SET(9.300222E37F) ;
        p231.time_usec_SET(1310064606308458594L) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.time_week_ms_GET() == 4010784214L);
            assert(pack.lat_GET() == 600352737);
            assert(pack.gps_id_GET() == (char)153);
            assert(pack.vn_GET() == -1.0731438E38F);
            assert(pack.vert_accuracy_GET() == -2.4818185E38F);
            assert(pack.speed_accuracy_GET() == 1.5998156E38F);
            assert(pack.vd_GET() == -1.1210672E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
            assert(pack.hdop_GET() == 7.6948896E37F);
            assert(pack.fix_type_GET() == (char)85);
            assert(pack.horiz_accuracy_GET() == -2.5071452E38F);
            assert(pack.lon_GET() == 1830867512);
            assert(pack.vdop_GET() == -1.8460763E38F);
            assert(pack.alt_GET() == -2.2851952E38F);
            assert(pack.satellites_visible_GET() == (char)237);
            assert(pack.time_usec_GET() == 729618682094744301L);
            assert(pack.ve_GET() == -2.9438792E38F);
            assert(pack.time_week_GET() == (char)12630);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.satellites_visible_SET((char)237) ;
        p232.lat_SET(600352737) ;
        p232.gps_id_SET((char)153) ;
        p232.speed_accuracy_SET(1.5998156E38F) ;
        p232.alt_SET(-2.2851952E38F) ;
        p232.vn_SET(-1.0731438E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY) ;
        p232.vd_SET(-1.1210672E38F) ;
        p232.lon_SET(1830867512) ;
        p232.ve_SET(-2.9438792E38F) ;
        p232.time_week_ms_SET(4010784214L) ;
        p232.horiz_accuracy_SET(-2.5071452E38F) ;
        p232.hdop_SET(7.6948896E37F) ;
        p232.fix_type_SET((char)85) ;
        p232.time_usec_SET(729618682094744301L) ;
        p232.vert_accuracy_SET(-2.4818185E38F) ;
        p232.time_week_SET((char)12630) ;
        p232.vdop_SET(-1.8460763E38F) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)9);
            assert(pack.len_GET() == (char)26);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)220, (char)80, (char)21, (char)26, (char)240, (char)240, (char)187, (char)144, (char)168, (char)253, (char)142, (char)75, (char)8, (char)113, (char)1, (char)226, (char)158, (char)174, (char)226, (char)254, (char)217, (char)44, (char)191, (char)86, (char)185, (char)159, (char)135, (char)222, (char)217, (char)131, (char)219, (char)216, (char)212, (char)116, (char)69, (char)11, (char)233, (char)65, (char)150, (char)39, (char)195, (char)54, (char)6, (char)248, (char)148, (char)137, (char)80, (char)21, (char)48, (char)156, (char)171, (char)251, (char)40, (char)10, (char)226, (char)224, (char)112, (char)200, (char)137, (char)136, (char)165, (char)3, (char)148, (char)108, (char)114, (char)168, (char)132, (char)153, (char)190, (char)159, (char)72, (char)24, (char)156, (char)145, (char)151, (char)245, (char)52, (char)30, (char)51, (char)240, (char)205, (char)57, (char)38, (char)42, (char)175, (char)51, (char)217, (char)172, (char)156, (char)206, (char)91, (char)39, (char)120, (char)17, (char)212, (char)81, (char)140, (char)153, (char)203, (char)89, (char)127, (char)179, (char)219, (char)72, (char)251, (char)96, (char)122, (char)73, (char)103, (char)144, (char)159, (char)136, (char)129, (char)129, (char)203, (char)117, (char)153, (char)128, (char)252, (char)231, (char)153, (char)7, (char)111, (char)169, (char)233, (char)231, (char)49, (char)99, (char)62, (char)80, (char)58, (char)240, (char)44, (char)88, (char)89, (char)50, (char)24, (char)197, (char)206, (char)57, (char)245, (char)68, (char)253, (char)246, (char)209, (char)83, (char)150, (char)49, (char)189, (char)213, (char)13, (char)230, (char)152, (char)36, (char)9, (char)121, (char)96, (char)121, (char)46, (char)29, (char)6, (char)221, (char)80, (char)249, (char)249, (char)226, (char)57, (char)139, (char)254, (char)6, (char)158, (char)93, (char)187, (char)93, (char)198, (char)162, (char)65, (char)134, (char)104, (char)13}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)26) ;
        p233.flags_SET((char)9) ;
        p233.data__SET(new char[] {(char)220, (char)80, (char)21, (char)26, (char)240, (char)240, (char)187, (char)144, (char)168, (char)253, (char)142, (char)75, (char)8, (char)113, (char)1, (char)226, (char)158, (char)174, (char)226, (char)254, (char)217, (char)44, (char)191, (char)86, (char)185, (char)159, (char)135, (char)222, (char)217, (char)131, (char)219, (char)216, (char)212, (char)116, (char)69, (char)11, (char)233, (char)65, (char)150, (char)39, (char)195, (char)54, (char)6, (char)248, (char)148, (char)137, (char)80, (char)21, (char)48, (char)156, (char)171, (char)251, (char)40, (char)10, (char)226, (char)224, (char)112, (char)200, (char)137, (char)136, (char)165, (char)3, (char)148, (char)108, (char)114, (char)168, (char)132, (char)153, (char)190, (char)159, (char)72, (char)24, (char)156, (char)145, (char)151, (char)245, (char)52, (char)30, (char)51, (char)240, (char)205, (char)57, (char)38, (char)42, (char)175, (char)51, (char)217, (char)172, (char)156, (char)206, (char)91, (char)39, (char)120, (char)17, (char)212, (char)81, (char)140, (char)153, (char)203, (char)89, (char)127, (char)179, (char)219, (char)72, (char)251, (char)96, (char)122, (char)73, (char)103, (char)144, (char)159, (char)136, (char)129, (char)129, (char)203, (char)117, (char)153, (char)128, (char)252, (char)231, (char)153, (char)7, (char)111, (char)169, (char)233, (char)231, (char)49, (char)99, (char)62, (char)80, (char)58, (char)240, (char)44, (char)88, (char)89, (char)50, (char)24, (char)197, (char)206, (char)57, (char)245, (char)68, (char)253, (char)246, (char)209, (char)83, (char)150, (char)49, (char)189, (char)213, (char)13, (char)230, (char)152, (char)36, (char)9, (char)121, (char)96, (char)121, (char)46, (char)29, (char)6, (char)221, (char)80, (char)249, (char)249, (char)226, (char)57, (char)139, (char)254, (char)6, (char)158, (char)93, (char)187, (char)93, (char)198, (char)162, (char)65, (char)134, (char)104, (char)13}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == (short) -9776);
            assert(pack.custom_mode_GET() == 1751747668L);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            assert(pack.battery_remaining_GET() == (char)140);
            assert(pack.wp_num_GET() == (char)164);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.failsafe_GET() == (char)131);
            assert(pack.heading_GET() == (char)34176);
            assert(pack.throttle_GET() == (byte)91);
            assert(pack.temperature_GET() == (byte) - 23);
            assert(pack.climb_rate_GET() == (byte)67);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.gps_nsat_GET() == (char)243);
            assert(pack.altitude_amsl_GET() == (short) -25051);
            assert(pack.temperature_air_GET() == (byte) - 118);
            assert(pack.latitude_GET() == 1002534131);
            assert(pack.altitude_sp_GET() == (short) -2031);
            assert(pack.longitude_GET() == -1611561852);
            assert(pack.airspeed_sp_GET() == (char)99);
            assert(pack.airspeed_GET() == (char)221);
            assert(pack.heading_sp_GET() == (short) -29185);
            assert(pack.roll_GET() == (short)1641);
            assert(pack.wp_distance_GET() == (char)51279);
            assert(pack.groundspeed_GET() == (char)42);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.climb_rate_SET((byte)67) ;
        p234.custom_mode_SET(1751747668L) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p234.temperature_SET((byte) - 23) ;
        p234.heading_SET((char)34176) ;
        p234.gps_nsat_SET((char)243) ;
        p234.wp_distance_SET((char)51279) ;
        p234.pitch_SET((short) -9776) ;
        p234.battery_remaining_SET((char)140) ;
        p234.altitude_amsl_SET((short) -25051) ;
        p234.roll_SET((short)1641) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) ;
        p234.airspeed_sp_SET((char)99) ;
        p234.airspeed_SET((char)221) ;
        p234.wp_num_SET((char)164) ;
        p234.latitude_SET(1002534131) ;
        p234.throttle_SET((byte)91) ;
        p234.temperature_air_SET((byte) - 118) ;
        p234.heading_sp_SET((short) -29185) ;
        p234.longitude_SET(-1611561852) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.altitude_sp_SET((short) -2031) ;
        p234.failsafe_SET((char)131) ;
        p234.groundspeed_SET((char)42) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == 1.7323823E38F);
            assert(pack.clipping_0_GET() == 187335562L);
            assert(pack.vibration_y_GET() == -2.7690789E38F);
            assert(pack.clipping_1_GET() == 579335951L);
            assert(pack.vibration_x_GET() == 1.4416666E38F);
            assert(pack.clipping_2_GET() == 2546305635L);
            assert(pack.time_usec_GET() == 8295655205715749625L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(8295655205715749625L) ;
        p241.vibration_x_SET(1.4416666E38F) ;
        p241.clipping_0_SET(187335562L) ;
        p241.vibration_z_SET(1.7323823E38F) ;
        p241.clipping_2_SET(2546305635L) ;
        p241.clipping_1_SET(579335951L) ;
        p241.vibration_y_SET(-2.7690789E38F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -1519972934);
            assert(pack.time_usec_TRY(ph) == 6239676930070708964L);
            assert(pack.approach_x_GET() == 1.8898141E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.4798179E38F, 3.5742747E37F, -3.3098773E38F, -2.959453E37F}));
            assert(pack.approach_z_GET() == 1.4191505E38F);
            assert(pack.y_GET() == -2.3679665E38F);
            assert(pack.approach_y_GET() == -2.5254158E38F);
            assert(pack.latitude_GET() == -882797288);
            assert(pack.longitude_GET() == -848290549);
            assert(pack.z_GET() == 1.4149268E38F);
            assert(pack.x_GET() == -1.8678495E38F);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.altitude_SET(-1519972934) ;
        p242.approach_x_SET(1.8898141E38F) ;
        p242.time_usec_SET(6239676930070708964L, PH) ;
        p242.z_SET(1.4149268E38F) ;
        p242.approach_z_SET(1.4191505E38F) ;
        p242.y_SET(-2.3679665E38F) ;
        p242.q_SET(new float[] {2.4798179E38F, 3.5742747E37F, -3.3098773E38F, -2.959453E37F}, 0) ;
        p242.latitude_SET(-882797288) ;
        p242.approach_y_SET(-2.5254158E38F) ;
        p242.x_SET(-1.8678495E38F) ;
        p242.longitude_SET(-848290549) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -1505739518);
            assert(pack.longitude_GET() == 2021692590);
            assert(pack.y_GET() == -6.077358E37F);
            assert(pack.target_system_GET() == (char)158);
            assert(pack.approach_z_GET() == -1.1354583E38F);
            assert(pack.time_usec_TRY(ph) == 2649558920826070664L);
            assert(pack.altitude_GET() == 412551296);
            assert(pack.approach_y_GET() == 1.1769032E38F);
            assert(pack.x_GET() == 1.9205503E38F);
            assert(pack.z_GET() == -3.3648497E38F);
            assert(pack.approach_x_GET() == 1.7505918E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.2055544E38F, -2.5942934E38F, 1.4301399E38F, -1.0056363E38F}));
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.time_usec_SET(2649558920826070664L, PH) ;
        p243.q_SET(new float[] {1.2055544E38F, -2.5942934E38F, 1.4301399E38F, -1.0056363E38F}, 0) ;
        p243.altitude_SET(412551296) ;
        p243.y_SET(-6.077358E37F) ;
        p243.x_SET(1.9205503E38F) ;
        p243.longitude_SET(2021692590) ;
        p243.approach_y_SET(1.1769032E38F) ;
        p243.approach_x_SET(1.7505918E38F) ;
        p243.approach_z_SET(-1.1354583E38F) ;
        p243.target_system_SET((char)158) ;
        p243.latitude_SET(-1505739518) ;
        p243.z_SET(-3.3648497E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1635424883);
            assert(pack.message_id_GET() == (char)27919);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1635424883) ;
        p244.message_id_SET((char)27919) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.squawk_GET() == (char)1710);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            assert(pack.tslc_GET() == (char)4);
            assert(pack.hor_velocity_GET() == (char)22502);
            assert(pack.ver_velocity_GET() == (short) -6681);
            assert(pack.altitude_GET() == 1690716243);
            assert(pack.ICAO_address_GET() == 517217215L);
            assert(pack.heading_GET() == (char)62625);
            assert(pack.lat_GET() == 458609388);
            assert(pack.callsign_LEN(ph) == 3);
            assert(pack.callsign_TRY(ph).equals("ugi"));
            assert(pack.lon_GET() == -144730466);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.hor_velocity_SET((char)22502) ;
        p246.callsign_SET("ugi", PH) ;
        p246.tslc_SET((char)4) ;
        p246.lat_SET(458609388) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY) ;
        p246.ICAO_address_SET(517217215L) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN) ;
        p246.ver_velocity_SET((short) -6681) ;
        p246.heading_SET((char)62625) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.squawk_SET((char)1710) ;
        p246.altitude_SET(1690716243) ;
        p246.lon_SET(-144730466) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.altitude_minimum_delta_GET() == -7.1736784E37F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
            assert(pack.horizontal_minimum_delta_GET() == 2.042914E38F);
            assert(pack.id_GET() == 1271197789L);
            assert(pack.time_to_minimum_delta_GET() == 1.0586118E38F);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.altitude_minimum_delta_SET(-7.1736784E37F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.id_SET(1271197789L) ;
        p247.horizontal_minimum_delta_SET(2.042914E38F) ;
        p247.time_to_minimum_delta_SET(1.0586118E38F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.message_type_GET() == (char)13702);
            assert(pack.target_component_GET() == (char)214);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.target_network_GET() == (char)147);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)91, (char)208, (char)253, (char)1, (char)230, (char)133, (char)58, (char)83, (char)142, (char)178, (char)11, (char)126, (char)16, (char)24, (char)54, (char)107, (char)75, (char)109, (char)235, (char)235, (char)190, (char)255, (char)5, (char)120, (char)27, (char)139, (char)124, (char)252, (char)144, (char)49, (char)155, (char)38, (char)176, (char)138, (char)234, (char)94, (char)177, (char)2, (char)47, (char)10, (char)27, (char)97, (char)218, (char)27, (char)17, (char)181, (char)185, (char)24, (char)211, (char)79, (char)113, (char)33, (char)0, (char)189, (char)248, (char)35, (char)166, (char)158, (char)101, (char)20, (char)176, (char)212, (char)62, (char)56, (char)192, (char)67, (char)15, (char)155, (char)132, (char)251, (char)160, (char)172, (char)173, (char)128, (char)140, (char)11, (char)101, (char)75, (char)3, (char)47, (char)218, (char)65, (char)223, (char)37, (char)120, (char)140, (char)13, (char)116, (char)1, (char)61, (char)81, (char)15, (char)90, (char)141, (char)115, (char)175, (char)161, (char)251, (char)14, (char)150, (char)80, (char)41, (char)97, (char)158, (char)44, (char)104, (char)13, (char)83, (char)184, (char)41, (char)224, (char)223, (char)67, (char)102, (char)235, (char)142, (char)138, (char)72, (char)209, (char)38, (char)240, (char)227, (char)60, (char)60, (char)7, (char)97, (char)7, (char)252, (char)45, (char)53, (char)224, (char)200, (char)68, (char)250, (char)247, (char)7, (char)109, (char)31, (char)228, (char)55, (char)248, (char)238, (char)246, (char)174, (char)239, (char)48, (char)125, (char)42, (char)16, (char)4, (char)153, (char)42, (char)243, (char)60, (char)149, (char)61, (char)232, (char)241, (char)254, (char)133, (char)250, (char)240, (char)153, (char)125, (char)179, (char)227, (char)1, (char)184, (char)10, (char)127, (char)141, (char)103, (char)79, (char)80, (char)66, (char)230, (char)10, (char)98, (char)160, (char)243, (char)156, (char)7, (char)125, (char)31, (char)148, (char)27, (char)188, (char)61, (char)31, (char)118, (char)67, (char)176, (char)219, (char)181, (char)173, (char)164, (char)85, (char)227, (char)188, (char)247, (char)176, (char)210, (char)19, (char)143, (char)136, (char)162, (char)145, (char)74, (char)150, (char)141, (char)5, (char)177, (char)95, (char)200, (char)229, (char)6, (char)61, (char)70, (char)194, (char)192, (char)123, (char)147, (char)19, (char)127, (char)170, (char)84, (char)246, (char)106, (char)207, (char)118, (char)31, (char)127, (char)128, (char)17, (char)159, (char)84, (char)44, (char)200, (char)54, (char)181, (char)164, (char)99, (char)182, (char)5, (char)53, (char)86, (char)96, (char)25, (char)109}));
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.payload_SET(new char[] {(char)91, (char)208, (char)253, (char)1, (char)230, (char)133, (char)58, (char)83, (char)142, (char)178, (char)11, (char)126, (char)16, (char)24, (char)54, (char)107, (char)75, (char)109, (char)235, (char)235, (char)190, (char)255, (char)5, (char)120, (char)27, (char)139, (char)124, (char)252, (char)144, (char)49, (char)155, (char)38, (char)176, (char)138, (char)234, (char)94, (char)177, (char)2, (char)47, (char)10, (char)27, (char)97, (char)218, (char)27, (char)17, (char)181, (char)185, (char)24, (char)211, (char)79, (char)113, (char)33, (char)0, (char)189, (char)248, (char)35, (char)166, (char)158, (char)101, (char)20, (char)176, (char)212, (char)62, (char)56, (char)192, (char)67, (char)15, (char)155, (char)132, (char)251, (char)160, (char)172, (char)173, (char)128, (char)140, (char)11, (char)101, (char)75, (char)3, (char)47, (char)218, (char)65, (char)223, (char)37, (char)120, (char)140, (char)13, (char)116, (char)1, (char)61, (char)81, (char)15, (char)90, (char)141, (char)115, (char)175, (char)161, (char)251, (char)14, (char)150, (char)80, (char)41, (char)97, (char)158, (char)44, (char)104, (char)13, (char)83, (char)184, (char)41, (char)224, (char)223, (char)67, (char)102, (char)235, (char)142, (char)138, (char)72, (char)209, (char)38, (char)240, (char)227, (char)60, (char)60, (char)7, (char)97, (char)7, (char)252, (char)45, (char)53, (char)224, (char)200, (char)68, (char)250, (char)247, (char)7, (char)109, (char)31, (char)228, (char)55, (char)248, (char)238, (char)246, (char)174, (char)239, (char)48, (char)125, (char)42, (char)16, (char)4, (char)153, (char)42, (char)243, (char)60, (char)149, (char)61, (char)232, (char)241, (char)254, (char)133, (char)250, (char)240, (char)153, (char)125, (char)179, (char)227, (char)1, (char)184, (char)10, (char)127, (char)141, (char)103, (char)79, (char)80, (char)66, (char)230, (char)10, (char)98, (char)160, (char)243, (char)156, (char)7, (char)125, (char)31, (char)148, (char)27, (char)188, (char)61, (char)31, (char)118, (char)67, (char)176, (char)219, (char)181, (char)173, (char)164, (char)85, (char)227, (char)188, (char)247, (char)176, (char)210, (char)19, (char)143, (char)136, (char)162, (char)145, (char)74, (char)150, (char)141, (char)5, (char)177, (char)95, (char)200, (char)229, (char)6, (char)61, (char)70, (char)194, (char)192, (char)123, (char)147, (char)19, (char)127, (char)170, (char)84, (char)246, (char)106, (char)207, (char)118, (char)31, (char)127, (char)128, (char)17, (char)159, (char)84, (char)44, (char)200, (char)54, (char)181, (char)164, (char)99, (char)182, (char)5, (char)53, (char)86, (char)96, (char)25, (char)109}, 0) ;
        p248.message_type_SET((char)13702) ;
        p248.target_system_SET((char)22) ;
        p248.target_network_SET((char)147) ;
        p248.target_component_SET((char)214) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)50235);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 44, (byte)22, (byte)17, (byte)69, (byte)13, (byte)41, (byte)56, (byte) - 47, (byte)96, (byte)44, (byte)41, (byte) - 68, (byte) - 125, (byte) - 39, (byte) - 69, (byte)86, (byte) - 86, (byte) - 71, (byte) - 89, (byte) - 86, (byte) - 55, (byte) - 3, (byte)127, (byte)40, (byte) - 59, (byte) - 48, (byte) - 44, (byte)55, (byte) - 61, (byte)43, (byte)117, (byte)5}));
            assert(pack.ver_GET() == (char)36);
            assert(pack.type_GET() == (char)184);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.value_SET(new byte[] {(byte) - 44, (byte)22, (byte)17, (byte)69, (byte)13, (byte)41, (byte)56, (byte) - 47, (byte)96, (byte)44, (byte)41, (byte) - 68, (byte) - 125, (byte) - 39, (byte) - 69, (byte)86, (byte) - 86, (byte) - 71, (byte) - 89, (byte) - 86, (byte) - 55, (byte) - 3, (byte)127, (byte)40, (byte) - 59, (byte) - 48, (byte) - 44, (byte)55, (byte) - 61, (byte)43, (byte)117, (byte)5}, 0) ;
        p249.type_SET((char)184) ;
        p249.ver_SET((char)36) ;
        p249.address_SET((char)50235) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.082432E38F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("fdszrch"));
            assert(pack.y_GET() == -2.2439245E38F);
            assert(pack.time_usec_GET() == 5497118471052609769L);
            assert(pack.z_GET() == 2.7794422E38F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.x_SET(1.082432E38F) ;
        p250.z_SET(2.7794422E38F) ;
        p250.y_SET(-2.2439245E38F) ;
        p250.time_usec_SET(5497118471052609769L) ;
        p250.name_SET("fdszrch", PH) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 949092127L);
            assert(pack.value_GET() == 3.5150493E36F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("jwuOtq"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("jwuOtq", PH) ;
        p251.time_boot_ms_SET(949092127L) ;
        p251.value_SET(3.5150493E36F) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("Hxtdd"));
            assert(pack.time_boot_ms_GET() == 1890715393L);
            assert(pack.value_GET() == -623799715);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-623799715) ;
        p252.time_boot_ms_SET(1890715393L) ;
        p252.name_SET("Hxtdd", PH) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            assert(pack.text_LEN(ph) == 46);
            assert(pack.text_TRY(ph).equals("unujegmyxxXkdteedrVipjlgpekIivoWyabxfllwsAvadn"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY) ;
        p253.text_SET("unujegmyxxXkdteedrVipjlgpekIivoWyabxfllwsAvadn", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.3590575E38F);
            assert(pack.ind_GET() == (char)241);
            assert(pack.time_boot_ms_GET() == 3848086907L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)241) ;
        p254.time_boot_ms_SET(3848086907L) ;
        p254.value_SET(1.3590575E38F) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)238);
            assert(pack.target_system_GET() == (char)243);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)150, (char)78, (char)84, (char)193, (char)163, (char)119, (char)110, (char)151, (char)180, (char)22, (char)176, (char)252, (char)204, (char)74, (char)13, (char)163, (char)57, (char)22, (char)111, (char)85, (char)220, (char)159, (char)244, (char)78, (char)116, (char)199, (char)246, (char)11, (char)1, (char)54, (char)150, (char)8}));
            assert(pack.initial_timestamp_GET() == 3411043998348086952L);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(3411043998348086952L) ;
        p256.target_component_SET((char)238) ;
        p256.secret_key_SET(new char[] {(char)150, (char)78, (char)84, (char)193, (char)163, (char)119, (char)110, (char)151, (char)180, (char)22, (char)176, (char)252, (char)204, (char)74, (char)13, (char)163, (char)57, (char)22, (char)111, (char)85, (char)220, (char)159, (char)244, (char)78, (char)116, (char)199, (char)246, (char)11, (char)1, (char)54, (char)150, (char)8}, 0) ;
        p256.target_system_SET((char)243) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1616883481L);
            assert(pack.last_change_ms_GET() == 302508468L);
            assert(pack.state_GET() == (char)219);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.last_change_ms_SET(302508468L) ;
        p257.time_boot_ms_SET(1616883481L) ;
        p257.state_SET((char)219) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)11);
            assert(pack.target_system_GET() == (char)231);
            assert(pack.tune_LEN(ph) == 24);
            assert(pack.tune_TRY(ph).equals("nuKksnUhtudgmeembmtegAmg"));
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)231) ;
        p258.tune_SET("nuKksnUhtudgmeembmtegAmg", PH) ;
        p258.target_component_SET((char)11) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.lens_id_GET() == (char)255);
            assert(pack.cam_definition_uri_LEN(ph) == 24);
            assert(pack.cam_definition_uri_TRY(ph).equals("uKplkddzptcyeknTpucuClve"));
            assert(pack.sensor_size_v_GET() == 1.993754E38F);
            assert(pack.sensor_size_h_GET() == 2.6108296E38F);
            assert(pack.cam_definition_version_GET() == (char)27644);
            assert(pack.resolution_v_GET() == (char)14073);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)14, (char)202, (char)117, (char)198, (char)216, (char)155, (char)252, (char)227, (char)214, (char)198, (char)105, (char)16, (char)162, (char)203, (char)74, (char)162, (char)71, (char)135, (char)224, (char)144, (char)223, (char)54, (char)73, (char)70, (char)254, (char)239, (char)118, (char)20, (char)216, (char)29, (char)83, (char)187}));
            assert(pack.resolution_h_GET() == (char)7086);
            assert(pack.firmware_version_GET() == 2087643385L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)38, (char)221, (char)150, (char)40, (char)32, (char)14, (char)123, (char)152, (char)202, (char)14, (char)46, (char)223, (char)82, (char)34, (char)201, (char)88, (char)187, (char)145, (char)55, (char)212, (char)128, (char)119, (char)73, (char)246, (char)171, (char)156, (char)95, (char)201, (char)127, (char)174, (char)64, (char)214}));
            assert(pack.focal_length_GET() == 1.1532042E38F);
            assert(pack.time_boot_ms_GET() == 359151784L);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_h_SET(2.6108296E38F) ;
        p259.resolution_h_SET((char)7086) ;
        p259.model_name_SET(new char[] {(char)38, (char)221, (char)150, (char)40, (char)32, (char)14, (char)123, (char)152, (char)202, (char)14, (char)46, (char)223, (char)82, (char)34, (char)201, (char)88, (char)187, (char)145, (char)55, (char)212, (char)128, (char)119, (char)73, (char)246, (char)171, (char)156, (char)95, (char)201, (char)127, (char)174, (char)64, (char)214}, 0) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE) ;
        p259.focal_length_SET(1.1532042E38F) ;
        p259.lens_id_SET((char)255) ;
        p259.time_boot_ms_SET(359151784L) ;
        p259.cam_definition_version_SET((char)27644) ;
        p259.resolution_v_SET((char)14073) ;
        p259.firmware_version_SET(2087643385L) ;
        p259.cam_definition_uri_SET("uKplkddzptcyeknTpucuClve", PH) ;
        p259.sensor_size_v_SET(1.993754E38F) ;
        p259.vendor_name_SET(new char[] {(char)14, (char)202, (char)117, (char)198, (char)216, (char)155, (char)252, (char)227, (char)214, (char)198, (char)105, (char)16, (char)162, (char)203, (char)74, (char)162, (char)71, (char)135, (char)224, (char)144, (char)223, (char)54, (char)73, (char)70, (char)254, (char)239, (char)118, (char)20, (char)216, (char)29, (char)83, (char)187}, 0) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 19690838L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(19690838L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)4);
            assert(pack.total_capacity_GET() == -2.2562613E38F);
            assert(pack.write_speed_GET() == 1.1342686E38F);
            assert(pack.used_capacity_GET() == -2.2107518E38F);
            assert(pack.available_capacity_GET() == -5.8850715E37F);
            assert(pack.time_boot_ms_GET() == 585542087L);
            assert(pack.status_GET() == (char)48);
            assert(pack.read_speed_GET() == 1.8575434E38F);
            assert(pack.storage_id_GET() == (char)177);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(1.1342686E38F) ;
        p261.available_capacity_SET(-5.8850715E37F) ;
        p261.used_capacity_SET(-2.2107518E38F) ;
        p261.storage_id_SET((char)177) ;
        p261.time_boot_ms_SET(585542087L) ;
        p261.read_speed_SET(1.8575434E38F) ;
        p261.total_capacity_SET(-2.2562613E38F) ;
        p261.storage_count_SET((char)4) ;
        p261.status_SET((char)48) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_status_GET() == (char)120);
            assert(pack.image_interval_GET() == 3.0381725E38F);
            assert(pack.video_status_GET() == (char)58);
            assert(pack.time_boot_ms_GET() == 3718741881L);
            assert(pack.available_capacity_GET() == 2.9303245E38F);
            assert(pack.recording_time_ms_GET() == 3126354468L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(3.0381725E38F) ;
        p262.recording_time_ms_SET(3126354468L) ;
        p262.available_capacity_SET(2.9303245E38F) ;
        p262.time_boot_ms_SET(3718741881L) ;
        p262.video_status_SET((char)58) ;
        p262.image_status_SET((char)120) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 271817656);
            assert(pack.image_index_GET() == 1533367961);
            assert(pack.time_utc_GET() == 9121862444153387439L);
            assert(pack.alt_GET() == -769061586);
            assert(pack.capture_result_GET() == (byte)46);
            assert(pack.time_boot_ms_GET() == 3100817224L);
            assert(pack.camera_id_GET() == (char)169);
            assert(pack.relative_alt_GET() == 1170002329);
            assert(pack.lon_GET() == 1023986311);
            assert(pack.file_url_LEN(ph) == 26);
            assert(pack.file_url_TRY(ph).equals("edodowbpxfsAegpgfKhnwwivnl"));
            assert(Arrays.equals(pack.q_GET(),  new float[] {-6.4937273E37F, 4.321556E37F, 2.602842E38F, -3.0934332E38F}));
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lat_SET(271817656) ;
        p263.camera_id_SET((char)169) ;
        p263.time_utc_SET(9121862444153387439L) ;
        p263.file_url_SET("edodowbpxfsAegpgfKhnwwivnl", PH) ;
        p263.relative_alt_SET(1170002329) ;
        p263.alt_SET(-769061586) ;
        p263.lon_SET(1023986311) ;
        p263.image_index_SET(1533367961) ;
        p263.time_boot_ms_SET(3100817224L) ;
        p263.q_SET(new float[] {-6.4937273E37F, 4.321556E37F, 2.602842E38F, -3.0934332E38F}, 0) ;
        p263.capture_result_SET((byte)46) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 518680986L);
            assert(pack.flight_uuid_GET() == 7299107425213915826L);
            assert(pack.arming_time_utc_GET() == 7758356945179067271L);
            assert(pack.takeoff_time_utc_GET() == 3385082446809780537L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.flight_uuid_SET(7299107425213915826L) ;
        p264.time_boot_ms_SET(518680986L) ;
        p264.arming_time_utc_SET(7758356945179067271L) ;
        p264.takeoff_time_utc_SET(3385082446809780537L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.352599E37F);
            assert(pack.roll_GET() == 2.7519938E38F);
            assert(pack.yaw_GET() == -1.6697782E38F);
            assert(pack.time_boot_ms_GET() == 1403590425L);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1403590425L) ;
        p265.roll_SET(2.7519938E38F) ;
        p265.pitch_SET(-2.352599E37F) ;
        p265.yaw_SET(-1.6697782E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)30852);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)248, (char)208, (char)67, (char)244, (char)145, (char)161, (char)124, (char)42, (char)236, (char)8, (char)183, (char)105, (char)106, (char)137, (char)68, (char)67, (char)211, (char)3, (char)214, (char)248, (char)143, (char)38, (char)83, (char)187, (char)75, (char)121, (char)144, (char)138, (char)205, (char)106, (char)93, (char)120, (char)212, (char)113, (char)24, (char)190, (char)0, (char)112, (char)167, (char)159, (char)236, (char)43, (char)61, (char)30, (char)84, (char)52, (char)197, (char)105, (char)255, (char)21, (char)120, (char)25, (char)155, (char)116, (char)17, (char)238, (char)178, (char)0, (char)171, (char)127, (char)212, (char)81, (char)123, (char)251, (char)230, (char)1, (char)233, (char)109, (char)60, (char)109, (char)121, (char)62, (char)101, (char)46, (char)235, (char)221, (char)39, (char)91, (char)110, (char)212, (char)207, (char)120, (char)8, (char)125, (char)21, (char)114, (char)88, (char)14, (char)212, (char)248, (char)108, (char)88, (char)181, (char)47, (char)84, (char)169, (char)27, (char)28, (char)138, (char)128, (char)115, (char)124, (char)93, (char)62, (char)98, (char)231, (char)144, (char)36, (char)37, (char)225, (char)73, (char)2, (char)65, (char)84, (char)83, (char)118, (char)112, (char)20, (char)117, (char)251, (char)247, (char)130, (char)211, (char)241, (char)136, (char)156, (char)12, (char)112, (char)47, (char)67, (char)54, (char)175, (char)190, (char)85, (char)148, (char)0, (char)197, (char)109, (char)89, (char)116, (char)3, (char)25, (char)218, (char)216, (char)176, (char)151, (char)116, (char)78, (char)99, (char)71, (char)33, (char)57, (char)153, (char)234, (char)118, (char)86, (char)4, (char)232, (char)223, (char)195, (char)116, (char)149, (char)64, (char)76, (char)38, (char)81, (char)140, (char)144, (char)89, (char)156, (char)31, (char)50, (char)142, (char)15, (char)140, (char)32, (char)131, (char)152, (char)177, (char)21, (char)245, (char)153, (char)105, (char)205, (char)36, (char)105, (char)154, (char)180, (char)109, (char)230, (char)114, (char)30, (char)153, (char)157, (char)135, (char)60, (char)81, (char)107, (char)118, (char)152, (char)61, (char)32, (char)39, (char)85, (char)136, (char)138, (char)50, (char)42, (char)200, (char)124, (char)31, (char)192, (char)55, (char)225, (char)72, (char)128, (char)13, (char)9, (char)193, (char)101, (char)223, (char)194, (char)53, (char)78, (char)46, (char)37, (char)122, (char)155, (char)194, (char)12, (char)227, (char)63, (char)188, (char)156, (char)163, (char)133, (char)96, (char)222, (char)207, (char)241, (char)126, (char)74, (char)232, (char)199, (char)30, (char)129, (char)210, (char)190, (char)47}));
            assert(pack.first_message_offset_GET() == (char)172);
            assert(pack.length_GET() == (char)206);
            assert(pack.target_component_GET() == (char)229);
            assert(pack.target_system_GET() == (char)233);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.first_message_offset_SET((char)172) ;
        p266.data__SET(new char[] {(char)248, (char)208, (char)67, (char)244, (char)145, (char)161, (char)124, (char)42, (char)236, (char)8, (char)183, (char)105, (char)106, (char)137, (char)68, (char)67, (char)211, (char)3, (char)214, (char)248, (char)143, (char)38, (char)83, (char)187, (char)75, (char)121, (char)144, (char)138, (char)205, (char)106, (char)93, (char)120, (char)212, (char)113, (char)24, (char)190, (char)0, (char)112, (char)167, (char)159, (char)236, (char)43, (char)61, (char)30, (char)84, (char)52, (char)197, (char)105, (char)255, (char)21, (char)120, (char)25, (char)155, (char)116, (char)17, (char)238, (char)178, (char)0, (char)171, (char)127, (char)212, (char)81, (char)123, (char)251, (char)230, (char)1, (char)233, (char)109, (char)60, (char)109, (char)121, (char)62, (char)101, (char)46, (char)235, (char)221, (char)39, (char)91, (char)110, (char)212, (char)207, (char)120, (char)8, (char)125, (char)21, (char)114, (char)88, (char)14, (char)212, (char)248, (char)108, (char)88, (char)181, (char)47, (char)84, (char)169, (char)27, (char)28, (char)138, (char)128, (char)115, (char)124, (char)93, (char)62, (char)98, (char)231, (char)144, (char)36, (char)37, (char)225, (char)73, (char)2, (char)65, (char)84, (char)83, (char)118, (char)112, (char)20, (char)117, (char)251, (char)247, (char)130, (char)211, (char)241, (char)136, (char)156, (char)12, (char)112, (char)47, (char)67, (char)54, (char)175, (char)190, (char)85, (char)148, (char)0, (char)197, (char)109, (char)89, (char)116, (char)3, (char)25, (char)218, (char)216, (char)176, (char)151, (char)116, (char)78, (char)99, (char)71, (char)33, (char)57, (char)153, (char)234, (char)118, (char)86, (char)4, (char)232, (char)223, (char)195, (char)116, (char)149, (char)64, (char)76, (char)38, (char)81, (char)140, (char)144, (char)89, (char)156, (char)31, (char)50, (char)142, (char)15, (char)140, (char)32, (char)131, (char)152, (char)177, (char)21, (char)245, (char)153, (char)105, (char)205, (char)36, (char)105, (char)154, (char)180, (char)109, (char)230, (char)114, (char)30, (char)153, (char)157, (char)135, (char)60, (char)81, (char)107, (char)118, (char)152, (char)61, (char)32, (char)39, (char)85, (char)136, (char)138, (char)50, (char)42, (char)200, (char)124, (char)31, (char)192, (char)55, (char)225, (char)72, (char)128, (char)13, (char)9, (char)193, (char)101, (char)223, (char)194, (char)53, (char)78, (char)46, (char)37, (char)122, (char)155, (char)194, (char)12, (char)227, (char)63, (char)188, (char)156, (char)163, (char)133, (char)96, (char)222, (char)207, (char)241, (char)126, (char)74, (char)232, (char)199, (char)30, (char)129, (char)210, (char)190, (char)47}, 0) ;
        p266.sequence_SET((char)30852) ;
        p266.target_system_SET((char)233) ;
        p266.target_component_SET((char)229) ;
        p266.length_SET((char)206) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)143);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)59, (char)13, (char)185, (char)123, (char)1, (char)99, (char)125, (char)172, (char)177, (char)33, (char)177, (char)71, (char)120, (char)200, (char)115, (char)77, (char)175, (char)48, (char)211, (char)238, (char)27, (char)204, (char)147, (char)132, (char)200, (char)215, (char)175, (char)214, (char)153, (char)213, (char)15, (char)72, (char)221, (char)14, (char)191, (char)92, (char)167, (char)148, (char)163, (char)5, (char)251, (char)143, (char)251, (char)168, (char)92, (char)194, (char)76, (char)239, (char)113, (char)195, (char)252, (char)26, (char)194, (char)219, (char)85, (char)129, (char)131, (char)10, (char)51, (char)250, (char)8, (char)230, (char)99, (char)160, (char)111, (char)77, (char)42, (char)56, (char)126, (char)115, (char)124, (char)165, (char)56, (char)8, (char)70, (char)242, (char)167, (char)105, (char)164, (char)186, (char)175, (char)2, (char)69, (char)6, (char)254, (char)97, (char)9, (char)155, (char)222, (char)170, (char)243, (char)23, (char)148, (char)129, (char)15, (char)164, (char)191, (char)214, (char)57, (char)138, (char)118, (char)181, (char)159, (char)7, (char)193, (char)210, (char)11, (char)19, (char)196, (char)70, (char)69, (char)244, (char)160, (char)70, (char)204, (char)53, (char)169, (char)240, (char)36, (char)197, (char)163, (char)194, (char)126, (char)201, (char)225, (char)23, (char)210, (char)184, (char)221, (char)184, (char)206, (char)142, (char)38, (char)31, (char)124, (char)2, (char)163, (char)120, (char)133, (char)108, (char)54, (char)125, (char)47, (char)194, (char)172, (char)195, (char)151, (char)30, (char)10, (char)132, (char)218, (char)180, (char)217, (char)183, (char)96, (char)61, (char)174, (char)136, (char)45, (char)69, (char)47, (char)239, (char)175, (char)80, (char)3, (char)134, (char)207, (char)104, (char)43, (char)118, (char)191, (char)110, (char)186, (char)229, (char)27, (char)180, (char)141, (char)17, (char)242, (char)24, (char)164, (char)12, (char)249, (char)24, (char)183, (char)207, (char)36, (char)101, (char)250, (char)150, (char)240, (char)28, (char)30, (char)165, (char)226, (char)96, (char)228, (char)5, (char)215, (char)206, (char)111, (char)17, (char)47, (char)39, (char)178, (char)18, (char)126, (char)170, (char)97, (char)114, (char)156, (char)126, (char)64, (char)46, (char)50, (char)231, (char)42, (char)59, (char)9, (char)161, (char)64, (char)29, (char)61, (char)186, (char)59, (char)126, (char)213, (char)139, (char)128, (char)82, (char)166, (char)133, (char)110, (char)232, (char)16, (char)80, (char)62, (char)31, (char)77, (char)96, (char)67, (char)214, (char)48, (char)156, (char)74, (char)146, (char)4, (char)231, (char)65}));
            assert(pack.length_GET() == (char)146);
            assert(pack.sequence_GET() == (char)60816);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.target_system_GET() == (char)37);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)143) ;
        p267.length_SET((char)146) ;
        p267.target_system_SET((char)37) ;
        p267.target_component_SET((char)113) ;
        p267.sequence_SET((char)60816) ;
        p267.data__SET(new char[] {(char)59, (char)13, (char)185, (char)123, (char)1, (char)99, (char)125, (char)172, (char)177, (char)33, (char)177, (char)71, (char)120, (char)200, (char)115, (char)77, (char)175, (char)48, (char)211, (char)238, (char)27, (char)204, (char)147, (char)132, (char)200, (char)215, (char)175, (char)214, (char)153, (char)213, (char)15, (char)72, (char)221, (char)14, (char)191, (char)92, (char)167, (char)148, (char)163, (char)5, (char)251, (char)143, (char)251, (char)168, (char)92, (char)194, (char)76, (char)239, (char)113, (char)195, (char)252, (char)26, (char)194, (char)219, (char)85, (char)129, (char)131, (char)10, (char)51, (char)250, (char)8, (char)230, (char)99, (char)160, (char)111, (char)77, (char)42, (char)56, (char)126, (char)115, (char)124, (char)165, (char)56, (char)8, (char)70, (char)242, (char)167, (char)105, (char)164, (char)186, (char)175, (char)2, (char)69, (char)6, (char)254, (char)97, (char)9, (char)155, (char)222, (char)170, (char)243, (char)23, (char)148, (char)129, (char)15, (char)164, (char)191, (char)214, (char)57, (char)138, (char)118, (char)181, (char)159, (char)7, (char)193, (char)210, (char)11, (char)19, (char)196, (char)70, (char)69, (char)244, (char)160, (char)70, (char)204, (char)53, (char)169, (char)240, (char)36, (char)197, (char)163, (char)194, (char)126, (char)201, (char)225, (char)23, (char)210, (char)184, (char)221, (char)184, (char)206, (char)142, (char)38, (char)31, (char)124, (char)2, (char)163, (char)120, (char)133, (char)108, (char)54, (char)125, (char)47, (char)194, (char)172, (char)195, (char)151, (char)30, (char)10, (char)132, (char)218, (char)180, (char)217, (char)183, (char)96, (char)61, (char)174, (char)136, (char)45, (char)69, (char)47, (char)239, (char)175, (char)80, (char)3, (char)134, (char)207, (char)104, (char)43, (char)118, (char)191, (char)110, (char)186, (char)229, (char)27, (char)180, (char)141, (char)17, (char)242, (char)24, (char)164, (char)12, (char)249, (char)24, (char)183, (char)207, (char)36, (char)101, (char)250, (char)150, (char)240, (char)28, (char)30, (char)165, (char)226, (char)96, (char)228, (char)5, (char)215, (char)206, (char)111, (char)17, (char)47, (char)39, (char)178, (char)18, (char)126, (char)170, (char)97, (char)114, (char)156, (char)126, (char)64, (char)46, (char)50, (char)231, (char)42, (char)59, (char)9, (char)161, (char)64, (char)29, (char)61, (char)186, (char)59, (char)126, (char)213, (char)139, (char)128, (char)82, (char)166, (char)133, (char)110, (char)232, (char)16, (char)80, (char)62, (char)31, (char)77, (char)96, (char)67, (char)214, (char)48, (char)156, (char)74, (char)146, (char)4, (char)231, (char)65}, 0) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)24790);
            assert(pack.target_system_GET() == (char)217);
            assert(pack.target_component_GET() == (char)41);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)217) ;
        p268.target_component_SET((char)41) ;
        p268.sequence_SET((char)24790) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)174);
            assert(pack.resolution_h_GET() == (char)32539);
            assert(pack.resolution_v_GET() == (char)12387);
            assert(pack.uri_LEN(ph) == 27);
            assert(pack.uri_TRY(ph).equals("VeixfxEtqndtSSzVisdZiwxnslx"));
            assert(pack.framerate_GET() == -1.7926054E38F);
            assert(pack.camera_id_GET() == (char)226);
            assert(pack.rotation_GET() == (char)24485);
            assert(pack.bitrate_GET() == 4269498198L);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.framerate_SET(-1.7926054E38F) ;
        p269.bitrate_SET(4269498198L) ;
        p269.resolution_v_SET((char)12387) ;
        p269.status_SET((char)174) ;
        p269.camera_id_SET((char)226) ;
        p269.uri_SET("VeixfxEtqndtSSzVisdZiwxnslx", PH) ;
        p269.resolution_h_SET((char)32539) ;
        p269.rotation_SET((char)24485) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)70);
            assert(pack.framerate_GET() == -4.0774922E37F);
            assert(pack.bitrate_GET() == 4022577992L);
            assert(pack.rotation_GET() == (char)24825);
            assert(pack.target_component_GET() == (char)191);
            assert(pack.resolution_v_GET() == (char)31547);
            assert(pack.uri_LEN(ph) == 52);
            assert(pack.uri_TRY(ph).equals("xxsvbKzjyqHkwcbahalyhrjhksiQjkrdmdejpDbcvuvaBKokzOxH"));
            assert(pack.resolution_h_GET() == (char)13878);
            assert(pack.camera_id_GET() == (char)45);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_h_SET((char)13878) ;
        p270.framerate_SET(-4.0774922E37F) ;
        p270.bitrate_SET(4022577992L) ;
        p270.resolution_v_SET((char)31547) ;
        p270.camera_id_SET((char)45) ;
        p270.uri_SET("xxsvbKzjyqHkwcbahalyhrjhksiQjkrdmdejpDbcvuvaBKokzOxH", PH) ;
        p270.target_component_SET((char)191) ;
        p270.target_system_SET((char)70) ;
        p270.rotation_SET((char)24825) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 40);
            assert(pack.password_TRY(ph).equals("xynxlurxblxobthnnvmtkwcqvePauuTgtfGbgtuw"));
            assert(pack.ssid_LEN(ph) == 4);
            assert(pack.ssid_TRY(ph).equals("plia"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("plia", PH) ;
        p299.password_SET("xynxlurxblxobthnnvmtkwcqvePauuTgtfGbgtuw", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)7590);
            assert(pack.version_GET() == (char)54607);
            assert(pack.max_version_GET() == (char)39637);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)125, (char)109, (char)108, (char)12, (char)47, (char)234, (char)106, (char)123}));
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)180, (char)219, (char)23, (char)124, (char)210, (char)243, (char)109, (char)157}));
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)180, (char)219, (char)23, (char)124, (char)210, (char)243, (char)109, (char)157}, 0) ;
        p300.min_version_SET((char)7590) ;
        p300.spec_version_hash_SET(new char[] {(char)125, (char)109, (char)108, (char)12, (char)47, (char)234, (char)106, (char)123}, 0) ;
        p300.max_version_SET((char)39637) ;
        p300.version_SET((char)54607) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 940996351731524015L);
            assert(pack.sub_mode_GET() == (char)47);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            assert(pack.vendor_specific_status_code_GET() == (char)39936);
            assert(pack.uptime_sec_GET() == 4272973954L);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(4272973954L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE) ;
        p310.time_usec_SET(940996351731524015L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        p310.sub_mode_SET((char)47) ;
        p310.vendor_specific_status_code_SET((char)39936) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2424026256906609420L);
            assert(pack.hw_version_major_GET() == (char)206);
            assert(pack.sw_version_major_GET() == (char)117);
            assert(pack.hw_version_minor_GET() == (char)46);
            assert(pack.name_LEN(ph) == 45);
            assert(pack.name_TRY(ph).equals("MWlseAtdrqnfmijnvtfoctytzgfkhgjplRanoaKcddJyq"));
            assert(pack.uptime_sec_GET() == 1507155404L);
            assert(pack.sw_version_minor_GET() == (char)204);
            assert(pack.sw_vcs_commit_GET() == 1191139597L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)84, (char)43, (char)77, (char)17, (char)253, (char)57, (char)66, (char)36, (char)127, (char)87, (char)221, (char)18, (char)197, (char)96, (char)101, (char)193}));
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.sw_version_major_SET((char)117) ;
        p311.time_usec_SET(2424026256906609420L) ;
        p311.hw_version_minor_SET((char)46) ;
        p311.hw_unique_id_SET(new char[] {(char)84, (char)43, (char)77, (char)17, (char)253, (char)57, (char)66, (char)36, (char)127, (char)87, (char)221, (char)18, (char)197, (char)96, (char)101, (char)193}, 0) ;
        p311.uptime_sec_SET(1507155404L) ;
        p311.hw_version_major_SET((char)206) ;
        p311.name_SET("MWlseAtdrqnfmijnvtfoctytzgfkhgjplRanoaKcddJyq", PH) ;
        p311.sw_version_minor_SET((char)204) ;
        p311.sw_vcs_commit_SET(1191139597L) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)226);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("edvdjlWiWezRjboc"));
            assert(pack.param_index_GET() == (short) -22435);
            assert(pack.target_component_GET() == (char)237);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)226) ;
        p320.param_id_SET("edvdjlWiWezRjboc", PH) ;
        p320.param_index_SET((short) -22435) ;
        p320.target_component_SET((char)237) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)129);
            assert(pack.target_component_GET() == (char)145);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)145) ;
        p321.target_system_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)42846);
            assert(pack.param_value_LEN(ph) == 93);
            assert(pack.param_value_TRY(ph).equals("ibrEWrwirpomxlgqpGulxfnthoijbzrozscgqkwtnqyckkkdjstmadlkeqtgqbGsfumzlqrxknmrprqWxgkoIzaclleaf"));
            assert(pack.param_index_GET() == (char)22949);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("lupwoduthazoaRdl"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("ibrEWrwirpomxlgqpGulxfnthoijbzrozscgqkwtnqyckkkdjstmadlkeqtgqbGsfumzlqrxknmrprqWxgkoIzaclleaf", PH) ;
        p322.param_id_SET("lupwoduthazoaRdl", PH) ;
        p322.param_count_SET((char)42846) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p322.param_index_SET((char)22949) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)27);
            assert(pack.target_system_GET() == (char)47);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("fHkkjffFf"));
            assert(pack.param_value_LEN(ph) == 8);
            assert(pack.param_value_TRY(ph).equals("szxiqhzw"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_value_SET("szxiqhzw", PH) ;
        p323.target_system_SET((char)47) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p323.target_component_SET((char)27) ;
        p323.param_id_SET("fHkkjffFf", PH) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 37);
            assert(pack.param_value_TRY(ph).equals("kgpgkfynvfsmjejuupbHwaLljnlysguqojtna"));
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("ejkztabljmpv"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("kgpgkfynvfsmjejuupbHwaLljnlysguqojtna", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_id_SET("ejkztabljmpv", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)3);
            assert(pack.min_distance_GET() == (char)37694);
            assert(pack.time_usec_GET() == 1746528940166563458L);
            assert(pack.max_distance_GET() == (char)982);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)11993, (char)12623, (char)32084, (char)37776, (char)54053, (char)26140, (char)5147, (char)61278, (char)42508, (char)61760, (char)27339, (char)27856, (char)23483, (char)50510, (char)19388, (char)40494, (char)834, (char)21897, (char)37857, (char)42297, (char)51800, (char)30829, (char)5717, (char)40121, (char)48239, (char)38123, (char)24255, (char)18406, (char)17633, (char)44189, (char)39020, (char)49872, (char)28597, (char)55861, (char)62409, (char)33579, (char)16388, (char)5779, (char)33000, (char)14438, (char)40047, (char)52111, (char)35583, (char)20835, (char)7468, (char)49030, (char)48964, (char)33602, (char)51707, (char)11887, (char)51767, (char)26342, (char)37198, (char)21921, (char)52555, (char)3581, (char)16866, (char)36810, (char)21241, (char)42441, (char)60931, (char)50439, (char)35224, (char)37726, (char)22834, (char)17423, (char)41162, (char)25487, (char)54387, (char)61608, (char)63292, (char)14922}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(1746528940166563458L) ;
        p330.min_distance_SET((char)37694) ;
        p330.increment_SET((char)3) ;
        p330.distances_SET(new char[] {(char)11993, (char)12623, (char)32084, (char)37776, (char)54053, (char)26140, (char)5147, (char)61278, (char)42508, (char)61760, (char)27339, (char)27856, (char)23483, (char)50510, (char)19388, (char)40494, (char)834, (char)21897, (char)37857, (char)42297, (char)51800, (char)30829, (char)5717, (char)40121, (char)48239, (char)38123, (char)24255, (char)18406, (char)17633, (char)44189, (char)39020, (char)49872, (char)28597, (char)55861, (char)62409, (char)33579, (char)16388, (char)5779, (char)33000, (char)14438, (char)40047, (char)52111, (char)35583, (char)20835, (char)7468, (char)49030, (char)48964, (char)33602, (char)51707, (char)11887, (char)51767, (char)26342, (char)37198, (char)21921, (char)52555, (char)3581, (char)16866, (char)36810, (char)21241, (char)42441, (char)60931, (char)50439, (char)35224, (char)37726, (char)22834, (char)17423, (char)41162, (char)25487, (char)54387, (char)61608, (char)63292, (char)14922}, 0) ;
        p330.max_distance_SET((char)982) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
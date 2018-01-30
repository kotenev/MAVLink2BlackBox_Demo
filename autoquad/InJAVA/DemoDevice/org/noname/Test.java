
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_KITE);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            assert(pack.custom_mode_GET() == 4116849297L);
            assert(pack.mavlink_version_GET() == (char)108);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)108) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED) ;
        p0.custom_mode_SET(4116849297L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_KITE) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)43935);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
            assert(pack.errors_count4_GET() == (char)46764);
            assert(pack.voltage_battery_GET() == (char)58289);
            assert(pack.current_battery_GET() == (short) -32344);
            assert(pack.errors_count3_GET() == (char)23513);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
            assert(pack.errors_count2_GET() == (char)42886);
            assert(pack.battery_remaining_GET() == (byte) - 74);
            assert(pack.drop_rate_comm_GET() == (char)62189);
            assert(pack.load_GET() == (char)11266);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            assert(pack.errors_count1_GET() == (char)4960);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.drop_rate_comm_SET((char)62189) ;
        p1.errors_count1_SET((char)4960) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH) ;
        p1.load_SET((char)11266) ;
        p1.errors_count2_SET((char)42886) ;
        p1.voltage_battery_SET((char)58289) ;
        p1.current_battery_SET((short) -32344) ;
        p1.errors_count4_SET((char)46764) ;
        p1.battery_remaining_SET((byte) - 74) ;
        p1.errors_count3_SET((char)23513) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) ;
        p1.errors_comm_SET((char)43935) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_unix_usec_GET() == 2402321795286319868L);
            assert(pack.time_boot_ms_GET() == 1016725412L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(1016725412L) ;
        p2.time_unix_usec_SET(2402321795286319868L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.0247595E38F);
            assert(pack.afz_GET() == -3.4465828E37F);
            assert(pack.afx_GET() == 1.964928E38F);
            assert(pack.y_GET() == 4.84852E37F);
            assert(pack.vy_GET() == 3.152383E38F);
            assert(pack.z_GET() == 3.162438E38F);
            assert(pack.time_boot_ms_GET() == 3914796043L);
            assert(pack.vx_GET() == 2.2616986E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.vz_GET() == 3.235964E38F);
            assert(pack.afy_GET() == 1.206254E38F);
            assert(pack.yaw_GET() == -5.586568E37F);
            assert(pack.type_mask_GET() == (char)9832);
            assert(pack.yaw_rate_GET() == -2.8028651E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.z_SET(3.162438E38F) ;
        p3.time_boot_ms_SET(3914796043L) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p3.vy_SET(3.152383E38F) ;
        p3.vx_SET(2.2616986E38F) ;
        p3.x_SET(-2.0247595E38F) ;
        p3.yaw_rate_SET(-2.8028651E38F) ;
        p3.afx_SET(1.964928E38F) ;
        p3.vz_SET(3.235964E38F) ;
        p3.afz_SET(-3.4465828E37F) ;
        p3.afy_SET(1.206254E38F) ;
        p3.yaw_SET(-5.586568E37F) ;
        p3.type_mask_SET((char)9832) ;
        p3.y_SET(4.84852E37F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5946230806738039192L);
            assert(pack.target_component_GET() == (char)187);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.seq_GET() == 1904433847L);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.seq_SET(1904433847L) ;
        p4.target_system_SET((char)124) ;
        p4.target_component_SET((char)187) ;
        p4.time_usec_SET(5946230806738039192L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)135);
            assert(pack.target_system_GET() == (char)218);
            assert(pack.control_request_GET() == (char)128);
            assert(pack.passkey_LEN(ph) == 11);
            assert(pack.passkey_TRY(ph).equals("mjzhffrrZgt"));
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)218) ;
        p5.version_SET((char)135) ;
        p5.control_request_SET((char)128) ;
        p5.passkey_SET("mjzhffrrZgt", PH) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)246);
            assert(pack.ack_GET() == (char)34);
            assert(pack.gcs_system_id_GET() == (char)62);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)62) ;
        p6.ack_SET((char)34) ;
        p6.control_request_SET((char)246) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 19);
            assert(pack.key_TRY(ph).equals("evmdsyfoFvyeqelgHsj"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("evmdsyfoFvyeqelgHsj", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)106);
            assert(pack.custom_mode_GET() == 438996320L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p11.custom_mode_SET(438996320L) ;
        p11.target_system_SET((char)106) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)232);
            assert(pack.param_index_GET() == (short)9819);
            assert(pack.target_system_GET() == (char)96);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("uKkoeutwweyjxks"));
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("uKkoeutwweyjxks", PH) ;
        p20.target_component_SET((char)232) ;
        p20.target_system_SET((char)96) ;
        p20.param_index_SET((short)9819) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)105);
            assert(pack.target_component_GET() == (char)21);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)21) ;
        p21.target_system_SET((char)105) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)62784);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("gynRzdkLxvzweprw"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == -2.5325398E38F);
            assert(pack.param_index_GET() == (char)34901);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_index_SET((char)34901) ;
        p22.param_value_SET(-2.5325398E38F) ;
        p22.param_count_SET((char)62784) ;
        p22.param_id_SET("gynRzdkLxvzweprw", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            assert(pack.param_value_GET() == -2.6795682E38F);
            assert(pack.target_component_GET() == (char)223);
            assert(pack.target_system_GET() == (char)171);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("nekuuCumqwvwucpm"));
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)171) ;
        p23.target_component_SET((char)223) ;
        p23.param_value_SET(-2.6795682E38F) ;
        p23.param_id_SET("nekuuCumqwvwucpm", PH) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.hdg_acc_TRY(ph) == 2536405759L);
            assert(pack.h_acc_TRY(ph) == 2314513927L);
            assert(pack.eph_GET() == (char)10614);
            assert(pack.cog_GET() == (char)6543);
            assert(pack.alt_ellipsoid_TRY(ph) == -185940640);
            assert(pack.lon_GET() == -999844373);
            assert(pack.vel_acc_TRY(ph) == 3883822499L);
            assert(pack.v_acc_TRY(ph) == 1935549371L);
            assert(pack.satellites_visible_GET() == (char)43);
            assert(pack.alt_GET() == 439544952);
            assert(pack.vel_GET() == (char)65204);
            assert(pack.epv_GET() == (char)35329);
            assert(pack.time_usec_GET() == 6913869850775718315L);
            assert(pack.lat_GET() == -55295357);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.v_acc_SET(1935549371L, PH) ;
        p24.alt_ellipsoid_SET(-185940640, PH) ;
        p24.time_usec_SET(6913869850775718315L) ;
        p24.vel_SET((char)65204) ;
        p24.lat_SET(-55295357) ;
        p24.cog_SET((char)6543) ;
        p24.satellites_visible_SET((char)43) ;
        p24.lon_SET(-999844373) ;
        p24.alt_SET(439544952) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p24.epv_SET((char)35329) ;
        p24.vel_acc_SET(3883822499L, PH) ;
        p24.h_acc_SET(2314513927L, PH) ;
        p24.hdg_acc_SET(2536405759L, PH) ;
        p24.eph_SET((char)10614) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)127, (char)26, (char)214, (char)146, (char)67, (char)199, (char)57, (char)239, (char)169, (char)105, (char)59, (char)126, (char)154, (char)236, (char)238, (char)149, (char)188, (char)167, (char)150, (char)21}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)5, (char)192, (char)64, (char)201, (char)177, (char)94, (char)7, (char)64, (char)133, (char)13, (char)254, (char)174, (char)37, (char)186, (char)28, (char)98, (char)0, (char)156, (char)65, (char)5}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)227, (char)24, (char)13, (char)93, (char)164, (char)237, (char)175, (char)213, (char)228, (char)178, (char)54, (char)176, (char)122, (char)251, (char)162, (char)224, (char)50, (char)48, (char)67, (char)95}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)249, (char)128, (char)183, (char)26, (char)7, (char)50, (char)196, (char)63, (char)154, (char)186, (char)156, (char)199, (char)229, (char)47, (char)79, (char)47, (char)19, (char)134, (char)79, (char)174}));
            assert(pack.satellites_visible_GET() == (char)164);
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)141, (char)7, (char)81, (char)161, (char)71, (char)99, (char)64, (char)189, (char)200, (char)65, (char)94, (char)8, (char)3, (char)147, (char)202, (char)204, (char)188, (char)127, (char)238, (char)211}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)164) ;
        p25.satellite_prn_SET(new char[] {(char)249, (char)128, (char)183, (char)26, (char)7, (char)50, (char)196, (char)63, (char)154, (char)186, (char)156, (char)199, (char)229, (char)47, (char)79, (char)47, (char)19, (char)134, (char)79, (char)174}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)127, (char)26, (char)214, (char)146, (char)67, (char)199, (char)57, (char)239, (char)169, (char)105, (char)59, (char)126, (char)154, (char)236, (char)238, (char)149, (char)188, (char)167, (char)150, (char)21}, 0) ;
        p25.satellite_used_SET(new char[] {(char)5, (char)192, (char)64, (char)201, (char)177, (char)94, (char)7, (char)64, (char)133, (char)13, (char)254, (char)174, (char)37, (char)186, (char)28, (char)98, (char)0, (char)156, (char)65, (char)5}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)227, (char)24, (char)13, (char)93, (char)164, (char)237, (char)175, (char)213, (char)228, (char)178, (char)54, (char)176, (char)122, (char)251, (char)162, (char)224, (char)50, (char)48, (char)67, (char)95}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)141, (char)7, (char)81, (char)161, (char)71, (char)99, (char)64, (char)189, (char)200, (char)65, (char)94, (char)8, (char)3, (char)147, (char)202, (char)204, (char)188, (char)127, (char)238, (char)211}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)3157);
            assert(pack.xgyro_GET() == (short)17375);
            assert(pack.zgyro_GET() == (short)30687);
            assert(pack.xacc_GET() == (short) -26824);
            assert(pack.time_boot_ms_GET() == 3945055327L);
            assert(pack.zacc_GET() == (short) -11245);
            assert(pack.zmag_GET() == (short)11423);
            assert(pack.yacc_GET() == (short)7139);
            assert(pack.ymag_GET() == (short)19891);
            assert(pack.xmag_GET() == (short) -14780);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)7139) ;
        p26.xgyro_SET((short)17375) ;
        p26.zgyro_SET((short)30687) ;
        p26.zacc_SET((short) -11245) ;
        p26.xmag_SET((short) -14780) ;
        p26.zmag_SET((short)11423) ;
        p26.xacc_SET((short) -26824) ;
        p26.time_boot_ms_SET(3945055327L) ;
        p26.ymag_SET((short)19891) ;
        p26.ygyro_SET((short)3157) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)27896);
            assert(pack.xacc_GET() == (short)1096);
            assert(pack.yacc_GET() == (short) -9871);
            assert(pack.zmag_GET() == (short) -5290);
            assert(pack.zgyro_GET() == (short)19929);
            assert(pack.ymag_GET() == (short)2955);
            assert(pack.zacc_GET() == (short) -3715);
            assert(pack.xmag_GET() == (short) -27362);
            assert(pack.time_usec_GET() == 7415328713044713796L);
            assert(pack.ygyro_GET() == (short) -18356);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.zacc_SET((short) -3715) ;
        p27.yacc_SET((short) -9871) ;
        p27.ymag_SET((short)2955) ;
        p27.time_usec_SET(7415328713044713796L) ;
        p27.zgyro_SET((short)19929) ;
        p27.ygyro_SET((short) -18356) ;
        p27.zmag_SET((short) -5290) ;
        p27.xgyro_SET((short)27896) ;
        p27.xacc_SET((short)1096) ;
        p27.xmag_SET((short) -27362) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)4075);
            assert(pack.press_abs_GET() == (short)8211);
            assert(pack.temperature_GET() == (short) -29571);
            assert(pack.press_diff1_GET() == (short)9139);
            assert(pack.time_usec_GET() == 7780975951552254918L);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short)8211) ;
        p28.time_usec_SET(7780975951552254918L) ;
        p28.press_diff2_SET((short)4075) ;
        p28.temperature_SET((short) -29571) ;
        p28.press_diff1_SET((short)9139) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -19626);
            assert(pack.press_abs_GET() == -1.6725041E38F);
            assert(pack.press_diff_GET() == -2.920586E38F);
            assert(pack.time_boot_ms_GET() == 2528228881L);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short) -19626) ;
        p29.time_boot_ms_SET(2528228881L) ;
        p29.press_diff_SET(-2.920586E38F) ;
        p29.press_abs_SET(-1.6725041E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 508232950L);
            assert(pack.pitch_GET() == -7.138391E37F);
            assert(pack.rollspeed_GET() == 3.1816722E37F);
            assert(pack.yawspeed_GET() == 2.384262E38F);
            assert(pack.roll_GET() == 3.7168205E37F);
            assert(pack.pitchspeed_GET() == -3.3528363E38F);
            assert(pack.yaw_GET() == -2.3657954E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(-7.138391E37F) ;
        p30.yaw_SET(-2.3657954E38F) ;
        p30.roll_SET(3.7168205E37F) ;
        p30.pitchspeed_SET(-3.3528363E38F) ;
        p30.yawspeed_SET(2.384262E38F) ;
        p30.rollspeed_SET(3.1816722E37F) ;
        p30.time_boot_ms_SET(508232950L) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 1.3005733E38F);
            assert(pack.rollspeed_GET() == 8.2835257E37F);
            assert(pack.pitchspeed_GET() == 2.991628E38F);
            assert(pack.q4_GET() == -1.8866736E38F);
            assert(pack.yawspeed_GET() == -1.7765332E38F);
            assert(pack.q2_GET() == -1.1518231E37F);
            assert(pack.time_boot_ms_GET() == 24451860L);
            assert(pack.q1_GET() == 2.8528651E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(24451860L) ;
        p31.yawspeed_SET(-1.7765332E38F) ;
        p31.q3_SET(1.3005733E38F) ;
        p31.q1_SET(2.8528651E38F) ;
        p31.pitchspeed_SET(2.991628E38F) ;
        p31.rollspeed_SET(8.2835257E37F) ;
        p31.q4_SET(-1.8866736E38F) ;
        p31.q2_SET(-1.1518231E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 4.2338612E37F);
            assert(pack.vx_GET() == 1.0749742E38F);
            assert(pack.z_GET() == 2.9496203E38F);
            assert(pack.x_GET() == -1.6617417E38F);
            assert(pack.time_boot_ms_GET() == 837262000L);
            assert(pack.y_GET() == 1.1113019E37F);
            assert(pack.vz_GET() == -2.2188591E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-2.2188591E38F) ;
        p32.time_boot_ms_SET(837262000L) ;
        p32.vx_SET(1.0749742E38F) ;
        p32.x_SET(-1.6617417E38F) ;
        p32.vy_SET(4.2338612E37F) ;
        p32.z_SET(2.9496203E38F) ;
        p32.y_SET(1.1113019E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1403582645);
            assert(pack.hdg_GET() == (char)19181);
            assert(pack.alt_GET() == -1140581812);
            assert(pack.vy_GET() == (short) -3125);
            assert(pack.vz_GET() == (short) -3714);
            assert(pack.relative_alt_GET() == 750231270);
            assert(pack.vx_GET() == (short)22214);
            assert(pack.time_boot_ms_GET() == 225529022L);
            assert(pack.lon_GET() == -1344177959);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.alt_SET(-1140581812) ;
        p33.relative_alt_SET(750231270) ;
        p33.time_boot_ms_SET(225529022L) ;
        p33.vx_SET((short)22214) ;
        p33.vy_SET((short) -3125) ;
        p33.lat_SET(-1403582645) ;
        p33.hdg_SET((char)19181) ;
        p33.vz_SET((short) -3714) ;
        p33.lon_SET(-1344177959) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan4_scaled_GET() == (short)12067);
            assert(pack.chan7_scaled_GET() == (short)6438);
            assert(pack.chan5_scaled_GET() == (short)218);
            assert(pack.chan2_scaled_GET() == (short) -19280);
            assert(pack.port_GET() == (char)142);
            assert(pack.chan8_scaled_GET() == (short) -25625);
            assert(pack.chan1_scaled_GET() == (short) -16502);
            assert(pack.time_boot_ms_GET() == 1867482464L);
            assert(pack.chan3_scaled_GET() == (short)31276);
            assert(pack.chan6_scaled_GET() == (short)9708);
            assert(pack.rssi_GET() == (char)227);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan5_scaled_SET((short)218) ;
        p34.chan6_scaled_SET((short)9708) ;
        p34.port_SET((char)142) ;
        p34.chan2_scaled_SET((short) -19280) ;
        p34.chan7_scaled_SET((short)6438) ;
        p34.rssi_SET((char)227) ;
        p34.time_boot_ms_SET(1867482464L) ;
        p34.chan3_scaled_SET((short)31276) ;
        p34.chan4_scaled_SET((short)12067) ;
        p34.chan1_scaled_SET((short) -16502) ;
        p34.chan8_scaled_SET((short) -25625) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3908618604L);
            assert(pack.chan8_raw_GET() == (char)11408);
            assert(pack.chan3_raw_GET() == (char)53021);
            assert(pack.port_GET() == (char)157);
            assert(pack.chan4_raw_GET() == (char)41709);
            assert(pack.rssi_GET() == (char)112);
            assert(pack.chan7_raw_GET() == (char)18216);
            assert(pack.chan1_raw_GET() == (char)27968);
            assert(pack.chan6_raw_GET() == (char)42999);
            assert(pack.chan5_raw_GET() == (char)2968);
            assert(pack.chan2_raw_GET() == (char)33473);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan1_raw_SET((char)27968) ;
        p35.chan4_raw_SET((char)41709) ;
        p35.port_SET((char)157) ;
        p35.chan3_raw_SET((char)53021) ;
        p35.rssi_SET((char)112) ;
        p35.chan2_raw_SET((char)33473) ;
        p35.chan7_raw_SET((char)18216) ;
        p35.chan5_raw_SET((char)2968) ;
        p35.chan6_raw_SET((char)42999) ;
        p35.time_boot_ms_SET(3908618604L) ;
        p35.chan8_raw_SET((char)11408) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo11_raw_TRY(ph) == (char)46086);
            assert(pack.servo16_raw_TRY(ph) == (char)30160);
            assert(pack.servo4_raw_GET() == (char)61785);
            assert(pack.servo5_raw_GET() == (char)6569);
            assert(pack.servo14_raw_TRY(ph) == (char)1005);
            assert(pack.servo8_raw_GET() == (char)11077);
            assert(pack.servo13_raw_TRY(ph) == (char)27032);
            assert(pack.servo9_raw_TRY(ph) == (char)47938);
            assert(pack.servo1_raw_GET() == (char)53191);
            assert(pack.servo2_raw_GET() == (char)17816);
            assert(pack.servo7_raw_GET() == (char)14618);
            assert(pack.time_usec_GET() == 2753875143L);
            assert(pack.servo10_raw_TRY(ph) == (char)14607);
            assert(pack.servo12_raw_TRY(ph) == (char)37820);
            assert(pack.port_GET() == (char)98);
            assert(pack.servo6_raw_GET() == (char)55485);
            assert(pack.servo3_raw_GET() == (char)38476);
            assert(pack.servo15_raw_TRY(ph) == (char)41281);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo3_raw_SET((char)38476) ;
        p36.servo16_raw_SET((char)30160, PH) ;
        p36.servo15_raw_SET((char)41281, PH) ;
        p36.servo14_raw_SET((char)1005, PH) ;
        p36.servo12_raw_SET((char)37820, PH) ;
        p36.servo9_raw_SET((char)47938, PH) ;
        p36.servo2_raw_SET((char)17816) ;
        p36.time_usec_SET(2753875143L) ;
        p36.servo10_raw_SET((char)14607, PH) ;
        p36.servo4_raw_SET((char)61785) ;
        p36.servo5_raw_SET((char)6569) ;
        p36.servo1_raw_SET((char)53191) ;
        p36.servo11_raw_SET((char)46086, PH) ;
        p36.servo7_raw_SET((char)14618) ;
        p36.port_SET((char)98) ;
        p36.servo13_raw_SET((char)27032, PH) ;
        p36.servo6_raw_SET((char)55485) ;
        p36.servo8_raw_SET((char)11077) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.end_index_GET() == (short) -26735);
            assert(pack.target_system_GET() == (char)100);
            assert(pack.start_index_GET() == (short) -7851);
            assert(pack.target_component_GET() == (char)14);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short) -7851) ;
        p37.end_index_SET((short) -26735) ;
        p37.target_component_SET((char)14) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p37.target_system_SET((char)100) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)21987);
            assert(pack.target_system_GET() == (char)186);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)249);
            assert(pack.start_index_GET() == (short)20662);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)249) ;
        p38.start_index_SET((short)20662) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.end_index_SET((short)21987) ;
        p38.target_system_SET((char)186) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.current_GET() == (char)2);
            assert(pack.z_GET() == 2.6477296E38F);
            assert(pack.target_component_GET() == (char)39);
            assert(pack.autocontinue_GET() == (char)86);
            assert(pack.param1_GET() == -1.8354954E38F);
            assert(pack.seq_GET() == (char)7802);
            assert(pack.x_GET() == 2.8177086E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.y_GET() == 1.3549351E38F);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SPATIAL_USER_4);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.param3_GET() == -8.331541E37F);
            assert(pack.param2_GET() == 1.822429E38F);
            assert(pack.param4_GET() == -1.352599E38F);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.z_SET(2.6477296E38F) ;
        p39.autocontinue_SET((char)86) ;
        p39.param2_SET(1.822429E38F) ;
        p39.y_SET(1.3549351E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.param4_SET(-1.352599E38F) ;
        p39.seq_SET((char)7802) ;
        p39.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_4) ;
        p39.param1_SET(-1.8354954E38F) ;
        p39.target_system_SET((char)235) ;
        p39.target_component_SET((char)39) ;
        p39.param3_SET(-8.331541E37F) ;
        p39.x_SET(2.8177086E38F) ;
        p39.current_SET((char)2) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)117);
            assert(pack.seq_GET() == (char)16629);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)200);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)200) ;
        p40.seq_SET((char)16629) ;
        p40.target_system_SET((char)117) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)21004);
            assert(pack.target_system_GET() == (char)82);
            assert(pack.target_component_GET() == (char)50);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)82) ;
        p41.target_component_SET((char)50) ;
        p41.seq_SET((char)21004) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)34477);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)34477) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)49);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)94);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)49) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_component_SET((char)94) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)204);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.count_GET() == (char)14017);
            assert(pack.target_component_GET() == (char)252);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)14017) ;
        p44.target_component_SET((char)252) ;
        p44.target_system_SET((char)204) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)93);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)205);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)93) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_component_SET((char)205) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)61848);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)61848) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)187);
            assert(pack.target_system_GET() == (char)22);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE) ;
        p47.target_component_SET((char)187) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.target_system_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 52068655);
            assert(pack.longitude_GET() == -1944983504);
            assert(pack.target_system_GET() == (char)153);
            assert(pack.altitude_GET() == 88667226);
            assert(pack.time_usec_TRY(ph) == 7558384977387578557L);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(7558384977387578557L, PH) ;
        p48.longitude_SET(-1944983504) ;
        p48.latitude_SET(52068655) ;
        p48.target_system_SET((char)153) ;
        p48.altitude_SET(88667226) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 6857795943020702799L);
            assert(pack.latitude_GET() == -1974232828);
            assert(pack.longitude_GET() == 1476290588);
            assert(pack.altitude_GET() == 527995909);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(1476290588) ;
        p49.altitude_SET(527995909) ;
        p49.latitude_SET(-1974232828) ;
        p49.time_usec_SET(6857795943020702799L, PH) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)144);
            assert(pack.param_value0_GET() == 2.8672092E38F);
            assert(pack.param_value_max_GET() == 1.5489518E38F);
            assert(pack.param_value_min_GET() == 2.852075E38F);
            assert(pack.target_system_GET() == (char)45);
            assert(pack.parameter_rc_channel_index_GET() == (char)215);
            assert(pack.param_index_GET() == (short) -20136);
            assert(pack.scale_GET() == -9.9576515E36F);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("idevoubxumb"));
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_index_SET((short) -20136) ;
        p50.param_value0_SET(2.8672092E38F) ;
        p50.scale_SET(-9.9576515E36F) ;
        p50.parameter_rc_channel_index_SET((char)215) ;
        p50.param_id_SET("idevoubxumb", PH) ;
        p50.param_value_min_SET(2.852075E38F) ;
        p50.param_value_max_SET(1.5489518E38F) ;
        p50.target_component_SET((char)144) ;
        p50.target_system_SET((char)45) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)170);
            assert(pack.seq_GET() == (char)56726);
            assert(pack.target_system_GET() == (char)149);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)170) ;
        p51.target_system_SET((char)149) ;
        p51.seq_SET((char)56726) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 2.1881225E38F);
            assert(pack.p1z_GET() == 1.3439553E38F);
            assert(pack.p2z_GET() == 1.6689951E38F);
            assert(pack.target_system_GET() == (char)241);
            assert(pack.p2y_GET() == -6.5702214E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p2x_GET() == 2.2926248E38F);
            assert(pack.p1y_GET() == -3.208495E38F);
            assert(pack.target_component_GET() == (char)147);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p54.p1z_SET(1.3439553E38F) ;
        p54.p1x_SET(2.1881225E38F) ;
        p54.p2y_SET(-6.5702214E37F) ;
        p54.target_system_SET((char)241) ;
        p54.p1y_SET(-3.208495E38F) ;
        p54.p2z_SET(1.6689951E38F) ;
        p54.p2x_SET(2.2926248E38F) ;
        p54.target_component_SET((char)147) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1z_GET() == 3.1966004E38F);
            assert(pack.p2y_GET() == 8.3667247E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p1y_GET() == 1.9624176E38F);
            assert(pack.p2z_GET() == -2.53792E38F);
            assert(pack.p2x_GET() == -8.992643E37F);
            assert(pack.p1x_GET() == -2.736675E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1z_SET(3.1966004E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p55.p2x_SET(-8.992643E37F) ;
        p55.p2z_SET(-2.53792E38F) ;
        p55.p2y_SET(8.3667247E37F) ;
        p55.p1y_SET(1.9624176E38F) ;
        p55.p1x_SET(-2.736675E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4558939974306735204L);
            assert(pack.rollspeed_GET() == -2.861822E35F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.6250415E38F, -6.341909E37F, -1.5442509E38F, 2.0937091E38F, 2.3546442E38F, 2.2419881E38F, -3.3980127E38F, 2.0553255E38F, 2.9378922E38F}));
            assert(pack.pitchspeed_GET() == -2.7038705E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2180866E38F, 1.2058173E38F, -2.206336E38F, -2.8823809E38F}));
            assert(pack.yawspeed_GET() == 2.3341352E37F);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.yawspeed_SET(2.3341352E37F) ;
        p61.pitchspeed_SET(-2.7038705E38F) ;
        p61.time_usec_SET(4558939974306735204L) ;
        p61.q_SET(new float[] {-3.2180866E38F, 1.2058173E38F, -2.206336E38F, -2.8823809E38F}, 0) ;
        p61.covariance_SET(new float[] {2.6250415E38F, -6.341909E37F, -1.5442509E38F, 2.0937091E38F, 2.3546442E38F, 2.2419881E38F, -3.3980127E38F, 2.0553255E38F, 2.9378922E38F}, 0) ;
        p61.rollspeed_SET(-2.861822E35F) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.target_bearing_GET() == (short) -25527);
            assert(pack.nav_pitch_GET() == 2.0213351E36F);
            assert(pack.wp_dist_GET() == (char)57708);
            assert(pack.aspd_error_GET() == 7.384783E37F);
            assert(pack.xtrack_error_GET() == 2.2200823E38F);
            assert(pack.nav_roll_GET() == -3.0343241E38F);
            assert(pack.nav_bearing_GET() == (short)2352);
            assert(pack.alt_error_GET() == 4.0628772E37F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(4.0628772E37F) ;
        p62.nav_roll_SET(-3.0343241E38F) ;
        p62.nav_pitch_SET(2.0213351E36F) ;
        p62.target_bearing_SET((short) -25527) ;
        p62.wp_dist_SET((char)57708) ;
        p62.nav_bearing_SET((short)2352) ;
        p62.aspd_error_SET(7.384783E37F) ;
        p62.xtrack_error_SET(2.2200823E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1814220247);
            assert(pack.vz_GET() == -1.0490487E38F);
            assert(pack.vy_GET() == -4.1176123E37F);
            assert(pack.alt_GET() == -692580263);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.3839432E37F, 6.6951803E37F, 1.7035634E38F, -3.055027E38F, -1.8379062E38F, 1.7783381E38F, 7.4374724E37F, -2.4689702E38F, -3.9215365E37F, -1.5171798E38F, 1.881428E38F, 2.5890816E38F, -2.1353039E38F, 2.949805E38F, 1.7582584E38F, -2.9585626E38F, -7.140229E37F, -2.7877395E38F, 7.611321E37F, 2.9239558E38F, -2.9567482E38F, -3.200323E38F, 3.2776579E38F, 5.6639496E37F, 1.5167554E38F, -2.3499721E38F, 1.7652815E38F, -1.0139252E38F, 2.045159E38F, -3.0701895E38F, 7.401664E37F, -1.998464E38F, 3.1791057E37F, -7.5125736E37F, 2.7921536E38F, -2.3204003E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.time_usec_GET() == 1878204235866008871L);
            assert(pack.vx_GET() == -2.3080725E38F);
            assert(pack.lon_GET() == -1389962730);
            assert(pack.relative_alt_GET() == -2106795221);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vz_SET(-1.0490487E38F) ;
        p63.relative_alt_SET(-2106795221) ;
        p63.vy_SET(-4.1176123E37F) ;
        p63.lon_SET(-1389962730) ;
        p63.time_usec_SET(1878204235866008871L) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vx_SET(-2.3080725E38F) ;
        p63.alt_SET(-692580263) ;
        p63.covariance_SET(new float[] {1.3839432E37F, 6.6951803E37F, 1.7035634E38F, -3.055027E38F, -1.8379062E38F, 1.7783381E38F, 7.4374724E37F, -2.4689702E38F, -3.9215365E37F, -1.5171798E38F, 1.881428E38F, 2.5890816E38F, -2.1353039E38F, 2.949805E38F, 1.7582584E38F, -2.9585626E38F, -7.140229E37F, -2.7877395E38F, 7.611321E37F, 2.9239558E38F, -2.9567482E38F, -3.200323E38F, 3.2776579E38F, 5.6639496E37F, 1.5167554E38F, -2.3499721E38F, 1.7652815E38F, -1.0139252E38F, 2.045159E38F, -3.0701895E38F, 7.401664E37F, -1.998464E38F, 3.1791057E37F, -7.5125736E37F, 2.7921536E38F, -2.3204003E38F}, 0) ;
        p63.lat_SET(1814220247) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -3.2541086E38F);
            assert(pack.time_usec_GET() == 2111361978379003275L);
            assert(pack.ax_GET() == -1.3111442E38F);
            assert(pack.vy_GET() == 1.5420594E38F);
            assert(pack.vx_GET() == -2.3916875E38F);
            assert(pack.y_GET() == 7.1344623E37F);
            assert(pack.ay_GET() == 1.067873E38F);
            assert(pack.z_GET() == -2.6964447E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-9.7060064E36F, 1.869551E38F, 6.3180583E37F, -3.6972637E37F, -1.8009987E38F, 2.4172428E38F, 1.4088042E38F, -2.5640846E38F, -2.7889285E38F, -1.9743204E38F, -1.715468E38F, -2.7671395E38F, -1.1297685E38F, -2.1064495E38F, -5.6209696E37F, 3.065893E38F, 2.6985813E38F, 1.2802367E38F, 1.0163426E38F, 2.3434144E38F, -1.1756353E38F, 2.0592241E37F, 7.075988E37F, -1.683538E38F, -1.7790614E38F, -3.394693E38F, -2.2965902E38F, -3.2436473E38F, 2.1051433E38F, -2.0274274E38F, -1.9561062E38F, 6.207442E37F, 4.697704E37F, -2.7577936E38F, -3.2406374E38F, 3.213383E37F, -1.3897851E38F, 2.677098E38F, 2.8507265E38F, 7.823218E37F, 9.455288E37F, -1.0979294E38F, -2.2213877E38F, 3.0874281E38F, 3.097791E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vz_GET() == 2.8543415E38F);
            assert(pack.az_GET() == 1.307105E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(7.1344623E37F) ;
        p64.vz_SET(2.8543415E38F) ;
        p64.vy_SET(1.5420594E38F) ;
        p64.time_usec_SET(2111361978379003275L) ;
        p64.az_SET(1.307105E38F) ;
        p64.vx_SET(-2.3916875E38F) ;
        p64.ay_SET(1.067873E38F) ;
        p64.ax_SET(-1.3111442E38F) ;
        p64.x_SET(-3.2541086E38F) ;
        p64.z_SET(-2.6964447E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.covariance_SET(new float[] {-9.7060064E36F, 1.869551E38F, 6.3180583E37F, -3.6972637E37F, -1.8009987E38F, 2.4172428E38F, 1.4088042E38F, -2.5640846E38F, -2.7889285E38F, -1.9743204E38F, -1.715468E38F, -2.7671395E38F, -1.1297685E38F, -2.1064495E38F, -5.6209696E37F, 3.065893E38F, 2.6985813E38F, 1.2802367E38F, 1.0163426E38F, 2.3434144E38F, -1.1756353E38F, 2.0592241E37F, 7.075988E37F, -1.683538E38F, -1.7790614E38F, -3.394693E38F, -2.2965902E38F, -3.2436473E38F, 2.1051433E38F, -2.0274274E38F, -1.9561062E38F, 6.207442E37F, 4.697704E37F, -2.7577936E38F, -3.2406374E38F, 3.213383E37F, -1.3897851E38F, 2.677098E38F, 2.8507265E38F, 7.823218E37F, 9.455288E37F, -1.0979294E38F, -2.2213877E38F, 3.0874281E38F, 3.097791E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan16_raw_GET() == (char)60702);
            assert(pack.rssi_GET() == (char)67);
            assert(pack.chan11_raw_GET() == (char)5770);
            assert(pack.chan12_raw_GET() == (char)45426);
            assert(pack.chan17_raw_GET() == (char)6775);
            assert(pack.chan1_raw_GET() == (char)28180);
            assert(pack.chan8_raw_GET() == (char)22014);
            assert(pack.chan9_raw_GET() == (char)51376);
            assert(pack.chan10_raw_GET() == (char)41317);
            assert(pack.chan5_raw_GET() == (char)55631);
            assert(pack.chan2_raw_GET() == (char)10623);
            assert(pack.chan15_raw_GET() == (char)11583);
            assert(pack.time_boot_ms_GET() == 181463243L);
            assert(pack.chancount_GET() == (char)196);
            assert(pack.chan7_raw_GET() == (char)28841);
            assert(pack.chan3_raw_GET() == (char)42248);
            assert(pack.chan18_raw_GET() == (char)24538);
            assert(pack.chan4_raw_GET() == (char)53405);
            assert(pack.chan13_raw_GET() == (char)26815);
            assert(pack.chan6_raw_GET() == (char)30918);
            assert(pack.chan14_raw_GET() == (char)54306);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.rssi_SET((char)67) ;
        p65.chan13_raw_SET((char)26815) ;
        p65.chan15_raw_SET((char)11583) ;
        p65.chan4_raw_SET((char)53405) ;
        p65.time_boot_ms_SET(181463243L) ;
        p65.chancount_SET((char)196) ;
        p65.chan9_raw_SET((char)51376) ;
        p65.chan6_raw_SET((char)30918) ;
        p65.chan11_raw_SET((char)5770) ;
        p65.chan7_raw_SET((char)28841) ;
        p65.chan12_raw_SET((char)45426) ;
        p65.chan17_raw_SET((char)6775) ;
        p65.chan8_raw_SET((char)22014) ;
        p65.chan2_raw_SET((char)10623) ;
        p65.chan18_raw_SET((char)24538) ;
        p65.chan14_raw_SET((char)54306) ;
        p65.chan16_raw_SET((char)60702) ;
        p65.chan1_raw_SET((char)28180) ;
        p65.chan5_raw_SET((char)55631) ;
        p65.chan3_raw_SET((char)42248) ;
        p65.chan10_raw_SET((char)41317) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)210);
            assert(pack.req_message_rate_GET() == (char)24888);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.req_stream_id_GET() == (char)177);
            assert(pack.target_component_GET() == (char)153);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)221) ;
        p66.req_stream_id_SET((char)177) ;
        p66.target_component_SET((char)153) ;
        p66.req_message_rate_SET((char)24888) ;
        p66.start_stop_SET((char)210) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.message_rate_GET() == (char)44458);
            assert(pack.on_off_GET() == (char)150);
            assert(pack.stream_id_GET() == (char)55);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)44458) ;
        p67.stream_id_SET((char)55) ;
        p67.on_off_SET((char)150) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == (short)27928);
            assert(pack.target_GET() == (char)112);
            assert(pack.y_GET() == (short) -17840);
            assert(pack.buttons_GET() == (char)46313);
            assert(pack.x_GET() == (short) -10248);
            assert(pack.r_GET() == (short)12394);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short)27928) ;
        p69.r_SET((short)12394) ;
        p69.target_SET((char)112) ;
        p69.y_SET((short) -17840) ;
        p69.buttons_SET((char)46313) ;
        p69.x_SET((short) -10248) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)53291);
            assert(pack.chan1_raw_GET() == (char)58384);
            assert(pack.chan7_raw_GET() == (char)27574);
            assert(pack.chan5_raw_GET() == (char)27663);
            assert(pack.target_system_GET() == (char)223);
            assert(pack.chan4_raw_GET() == (char)57221);
            assert(pack.chan6_raw_GET() == (char)10637);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.chan8_raw_GET() == (char)55093);
            assert(pack.chan2_raw_GET() == (char)34203);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)57221) ;
        p70.chan2_raw_SET((char)34203) ;
        p70.chan6_raw_SET((char)10637) ;
        p70.chan1_raw_SET((char)58384) ;
        p70.chan3_raw_SET((char)53291) ;
        p70.chan7_raw_SET((char)27574) ;
        p70.target_system_SET((char)223) ;
        p70.chan5_raw_SET((char)27663) ;
        p70.chan8_raw_SET((char)55093) ;
        p70.target_component_SET((char)34) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 1.6925079E38F);
            assert(pack.z_GET() == -8.0320213E37F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL);
            assert(pack.autocontinue_GET() == (char)9);
            assert(pack.seq_GET() == (char)56555);
            assert(pack.current_GET() == (char)154);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_GET() == -873623211);
            assert(pack.target_system_GET() == (char)253);
            assert(pack.param3_GET() == -3.5182063E37F);
            assert(pack.target_component_GET() == (char)32);
            assert(pack.param4_GET() == -2.118093E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.y_GET() == -583009539);
            assert(pack.param2_GET() == -2.6106712E38F);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)253) ;
        p73.y_SET(-583009539) ;
        p73.autocontinue_SET((char)9) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p73.target_component_SET((char)32) ;
        p73.x_SET(-873623211) ;
        p73.z_SET(-8.0320213E37F) ;
        p73.param2_SET(-2.6106712E38F) ;
        p73.param4_SET(-2.118093E38F) ;
        p73.param1_SET(1.6925079E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p73.seq_SET((char)56555) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL) ;
        p73.param3_SET(-3.5182063E37F) ;
        p73.current_SET((char)154) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == 3.2606268E36F);
            assert(pack.alt_GET() == 2.9716373E38F);
            assert(pack.heading_GET() == (short) -5393);
            assert(pack.throttle_GET() == (char)50509);
            assert(pack.airspeed_GET() == -1.7052241E38F);
            assert(pack.climb_GET() == 2.089018E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(2.089018E38F) ;
        p74.alt_SET(2.9716373E38F) ;
        p74.heading_SET((short) -5393) ;
        p74.groundspeed_SET(3.2606268E36F) ;
        p74.airspeed_SET(-1.7052241E38F) ;
        p74.throttle_SET((char)50509) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)129);
            assert(pack.param1_GET() == 2.9683592E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.target_system_GET() == (char)13);
            assert(pack.param2_GET() == 9.33974E37F);
            assert(pack.param3_GET() == 2.8446223E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
            assert(pack.x_GET() == -937709851);
            assert(pack.param4_GET() == -1.1543218E38F);
            assert(pack.current_GET() == (char)79);
            assert(pack.z_GET() == 2.3797998E38F);
            assert(pack.y_GET() == 289141071);
            assert(pack.autocontinue_GET() == (char)204);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.command_SET(MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE) ;
        p75.target_component_SET((char)129) ;
        p75.target_system_SET((char)13) ;
        p75.param1_SET(2.9683592E38F) ;
        p75.current_SET((char)79) ;
        p75.param3_SET(2.8446223E38F) ;
        p75.y_SET(289141071) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p75.z_SET(2.3797998E38F) ;
        p75.x_SET(-937709851) ;
        p75.param4_SET(-1.1543218E38F) ;
        p75.param2_SET(9.33974E37F) ;
        p75.autocontinue_SET((char)204) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -4.077105E36F);
            assert(pack.param1_GET() == -3.539596E36F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
            assert(pack.param6_GET() == 1.3280625E38F);
            assert(pack.param5_GET() == 2.7205603E38F);
            assert(pack.confirmation_GET() == (char)33);
            assert(pack.param2_GET() == -6.1430186E37F);
            assert(pack.target_component_GET() == (char)36);
            assert(pack.target_system_GET() == (char)239);
            assert(pack.param4_GET() == 1.6932498E38F);
            assert(pack.param7_GET() == 2.938341E38F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(1.6932498E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL) ;
        p76.param3_SET(-4.077105E36F) ;
        p76.param5_SET(2.7205603E38F) ;
        p76.target_system_SET((char)239) ;
        p76.confirmation_SET((char)33) ;
        p76.param6_SET(1.3280625E38F) ;
        p76.param2_SET(-6.1430186E37F) ;
        p76.param1_SET(-3.539596E36F) ;
        p76.target_component_SET((char)36) ;
        p76.param7_SET(2.938341E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.target_system_TRY(ph) == (char)225);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_LAND_START);
            assert(pack.progress_TRY(ph) == (char)113);
            assert(pack.result_param2_TRY(ph) == -1981894941);
            assert(pack.target_component_TRY(ph) == (char)33);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_DO_LAND_START) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.progress_SET((char)113, PH) ;
        p77.target_system_SET((char)225, PH) ;
        p77.result_param2_SET(-1981894941, PH) ;
        p77.target_component_SET((char)33, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.manual_override_switch_GET() == (char)189);
            assert(pack.time_boot_ms_GET() == 656123716L);
            assert(pack.pitch_GET() == -1.2126577E38F);
            assert(pack.roll_GET() == 2.788056E36F);
            assert(pack.thrust_GET() == 2.3103967E37F);
            assert(pack.mode_switch_GET() == (char)170);
            assert(pack.yaw_GET() == -3.1464297E38F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(656123716L) ;
        p81.manual_override_switch_SET((char)189) ;
        p81.roll_SET(2.788056E36F) ;
        p81.thrust_SET(2.3103967E37F) ;
        p81.yaw_SET(-3.1464297E38F) ;
        p81.mode_switch_SET((char)170) ;
        p81.pitch_SET(-1.2126577E38F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == -4.3638867E37F);
            assert(pack.body_pitch_rate_GET() == -2.8811934E38F);
            assert(pack.body_roll_rate_GET() == -1.9644966E38F);
            assert(pack.body_yaw_rate_GET() == -2.867293E38F);
            assert(pack.time_boot_ms_GET() == 2818379349L);
            assert(pack.target_system_GET() == (char)7);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.1853525E38F, -2.7050427E38F, 2.680198E38F, -3.303377E38F}));
            assert(pack.target_component_GET() == (char)215);
            assert(pack.type_mask_GET() == (char)199);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_roll_rate_SET(-1.9644966E38F) ;
        p82.type_mask_SET((char)199) ;
        p82.body_pitch_rate_SET(-2.8811934E38F) ;
        p82.target_component_SET((char)215) ;
        p82.target_system_SET((char)7) ;
        p82.q_SET(new float[] {1.1853525E38F, -2.7050427E38F, 2.680198E38F, -3.303377E38F}, 0) ;
        p82.time_boot_ms_SET(2818379349L) ;
        p82.thrust_SET(-4.3638867E37F) ;
        p82.body_yaw_rate_SET(-2.867293E38F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 6.3349536E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2572338E38F, -2.52482E38F, -1.84367E38F, -2.0859456E38F}));
            assert(pack.thrust_GET() == -2.9217568E38F);
            assert(pack.type_mask_GET() == (char)6);
            assert(pack.body_pitch_rate_GET() == 9.024831E37F);
            assert(pack.time_boot_ms_GET() == 966043795L);
            assert(pack.body_yaw_rate_GET() == 1.7481946E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(966043795L) ;
        p83.type_mask_SET((char)6) ;
        p83.body_yaw_rate_SET(1.7481946E38F) ;
        p83.q_SET(new float[] {-2.2572338E38F, -2.52482E38F, -1.84367E38F, -2.0859456E38F}, 0) ;
        p83.thrust_SET(-2.9217568E38F) ;
        p83.body_pitch_rate_SET(9.024831E37F) ;
        p83.body_roll_rate_SET(6.3349536E37F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.1394185E37F);
            assert(pack.afz_GET() == 2.5983225E38F);
            assert(pack.vx_GET() == 3.2985315E38F);
            assert(pack.y_GET() == 1.6252895E38F);
            assert(pack.target_system_GET() == (char)220);
            assert(pack.yaw_GET() == -1.9674802E37F);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.afx_GET() == 3.4589451E37F);
            assert(pack.afy_GET() == -3.2088313E38F);
            assert(pack.time_boot_ms_GET() == 2518065314L);
            assert(pack.vy_GET() == 1.6995998E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.type_mask_GET() == (char)28329);
            assert(pack.vz_GET() == 3.144463E38F);
            assert(pack.yaw_rate_GET() == -1.1893471E38F);
            assert(pack.z_GET() == -2.7425779E38F);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_SET(-1.9674802E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p84.afz_SET(2.5983225E38F) ;
        p84.type_mask_SET((char)28329) ;
        p84.z_SET(-2.7425779E38F) ;
        p84.vz_SET(3.144463E38F) ;
        p84.x_SET(-2.1394185E37F) ;
        p84.time_boot_ms_SET(2518065314L) ;
        p84.target_system_SET((char)220) ;
        p84.yaw_rate_SET(-1.1893471E38F) ;
        p84.afy_SET(-3.2088313E38F) ;
        p84.target_component_SET((char)237) ;
        p84.vy_SET(1.6995998E38F) ;
        p84.vx_SET(3.2985315E38F) ;
        p84.y_SET(1.6252895E38F) ;
        p84.afx_SET(3.4589451E37F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_int_GET() == 289929795);
            assert(pack.yaw_rate_GET() == 2.7441175E38F);
            assert(pack.afy_GET() == 2.1533913E38F);
            assert(pack.lon_int_GET() == 1656738759);
            assert(pack.target_component_GET() == (char)172);
            assert(pack.vx_GET() == -1.0975086E38F);
            assert(pack.alt_GET() == -2.165771E38F);
            assert(pack.type_mask_GET() == (char)19573);
            assert(pack.target_system_GET() == (char)64);
            assert(pack.afx_GET() == -1.6189958E37F);
            assert(pack.vy_GET() == 1.2226283E38F);
            assert(pack.afz_GET() == -2.043867E38F);
            assert(pack.yaw_GET() == 1.7070023E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.time_boot_ms_GET() == 4140025490L);
            assert(pack.vz_GET() == 1.4714397E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.yaw_SET(1.7070023E38F) ;
        p86.afy_SET(2.1533913E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p86.afx_SET(-1.6189958E37F) ;
        p86.vx_SET(-1.0975086E38F) ;
        p86.time_boot_ms_SET(4140025490L) ;
        p86.target_component_SET((char)172) ;
        p86.vz_SET(1.4714397E38F) ;
        p86.vy_SET(1.2226283E38F) ;
        p86.afz_SET(-2.043867E38F) ;
        p86.alt_SET(-2.165771E38F) ;
        p86.target_system_SET((char)64) ;
        p86.lon_int_SET(1656738759) ;
        p86.yaw_rate_SET(2.7441175E38F) ;
        p86.lat_int_SET(289929795) ;
        p86.type_mask_SET((char)19573) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 7.983502E37F);
            assert(pack.yaw_rate_GET() == 2.7250642E38F);
            assert(pack.time_boot_ms_GET() == 1668794924L);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.alt_GET() == 1.1474387E38F);
            assert(pack.lat_int_GET() == -1150806299);
            assert(pack.vy_GET() == 3.1086809E38F);
            assert(pack.afx_GET() == 1.4227265E38F);
            assert(pack.lon_int_GET() == 1928684002);
            assert(pack.afy_GET() == -3.39474E38F);
            assert(pack.afz_GET() == -2.8352935E37F);
            assert(pack.vx_GET() == -1.7457295E38F);
            assert(pack.type_mask_GET() == (char)35819);
            assert(pack.vz_GET() == 1.1055045E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afy_SET(-3.39474E38F) ;
        p87.afx_SET(1.4227265E38F) ;
        p87.yaw_SET(7.983502E37F) ;
        p87.type_mask_SET((char)35819) ;
        p87.yaw_rate_SET(2.7250642E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p87.lat_int_SET(-1150806299) ;
        p87.lon_int_SET(1928684002) ;
        p87.vx_SET(-1.7457295E38F) ;
        p87.vy_SET(3.1086809E38F) ;
        p87.time_boot_ms_SET(1668794924L) ;
        p87.alt_SET(1.1474387E38F) ;
        p87.afz_SET(-2.8352935E37F) ;
        p87.vz_SET(1.1055045E38F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 1.5906759E38F);
            assert(pack.pitch_GET() == -2.2621537E38F);
            assert(pack.time_boot_ms_GET() == 371515961L);
            assert(pack.z_GET() == -3.3281505E38F);
            assert(pack.y_GET() == -2.1420669E38F);
            assert(pack.x_GET() == 1.4697895E38F);
            assert(pack.yaw_GET() == -1.723585E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.pitch_SET(-2.2621537E38F) ;
        p89.x_SET(1.4697895E38F) ;
        p89.roll_SET(1.5906759E38F) ;
        p89.y_SET(-2.1420669E38F) ;
        p89.z_SET(-3.3281505E38F) ;
        p89.yaw_SET(-1.723585E38F) ;
        p89.time_boot_ms_SET(371515961L) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -2002);
            assert(pack.lat_GET() == 259362183);
            assert(pack.vx_GET() == (short)28847);
            assert(pack.rollspeed_GET() == 2.72023E37F);
            assert(pack.pitchspeed_GET() == 8.719541E37F);
            assert(pack.yawspeed_GET() == 3.1487366E37F);
            assert(pack.vy_GET() == (short)18049);
            assert(pack.alt_GET() == 1084302778);
            assert(pack.time_usec_GET() == 8752892106254869676L);
            assert(pack.xacc_GET() == (short) -19005);
            assert(pack.pitch_GET() == -1.5369568E38F);
            assert(pack.vz_GET() == (short)17165);
            assert(pack.roll_GET() == 1.8339033E37F);
            assert(pack.yacc_GET() == (short)17992);
            assert(pack.lon_GET() == -2086723244);
            assert(pack.yaw_GET() == -7.6558085E37F);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.vz_SET((short)17165) ;
        p90.time_usec_SET(8752892106254869676L) ;
        p90.yaw_SET(-7.6558085E37F) ;
        p90.vy_SET((short)18049) ;
        p90.xacc_SET((short) -19005) ;
        p90.rollspeed_SET(2.72023E37F) ;
        p90.vx_SET((short)28847) ;
        p90.pitch_SET(-1.5369568E38F) ;
        p90.alt_SET(1084302778) ;
        p90.pitchspeed_SET(8.719541E37F) ;
        p90.lon_SET(-2086723244) ;
        p90.yawspeed_SET(3.1487366E37F) ;
        p90.roll_SET(1.8339033E37F) ;
        p90.zacc_SET((short) -2002) ;
        p90.yacc_SET((short)17992) ;
        p90.lat_SET(259362183) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.roll_ailerons_GET() == -2.119396E38F);
            assert(pack.aux4_GET() == -7.0733484E37F);
            assert(pack.aux1_GET() == 2.5375467E38F);
            assert(pack.nav_mode_GET() == (char)183);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.pitch_elevator_GET() == 1.5953538E38F);
            assert(pack.aux2_GET() == -2.7000418E38F);
            assert(pack.aux3_GET() == 9.2325516E36F);
            assert(pack.time_usec_GET() == 6003551735668289869L);
            assert(pack.throttle_GET() == 3.0844369E38F);
            assert(pack.yaw_rudder_GET() == 2.5670876E38F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux3_SET(9.2325516E36F) ;
        p91.aux4_SET(-7.0733484E37F) ;
        p91.yaw_rudder_SET(2.5670876E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.pitch_elevator_SET(1.5953538E38F) ;
        p91.roll_ailerons_SET(-2.119396E38F) ;
        p91.throttle_SET(3.0844369E38F) ;
        p91.aux1_SET(2.5375467E38F) ;
        p91.time_usec_SET(6003551735668289869L) ;
        p91.nav_mode_SET((char)183) ;
        p91.aux2_SET(-2.7000418E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)53735);
            assert(pack.chan6_raw_GET() == (char)25727);
            assert(pack.chan4_raw_GET() == (char)27221);
            assert(pack.chan12_raw_GET() == (char)44705);
            assert(pack.rssi_GET() == (char)96);
            assert(pack.chan2_raw_GET() == (char)65450);
            assert(pack.chan9_raw_GET() == (char)46720);
            assert(pack.chan10_raw_GET() == (char)14826);
            assert(pack.chan3_raw_GET() == (char)3083);
            assert(pack.chan11_raw_GET() == (char)36720);
            assert(pack.time_usec_GET() == 716432663992481926L);
            assert(pack.chan8_raw_GET() == (char)14753);
            assert(pack.chan1_raw_GET() == (char)33251);
            assert(pack.chan7_raw_GET() == (char)56861);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan11_raw_SET((char)36720) ;
        p92.chan3_raw_SET((char)3083) ;
        p92.chan12_raw_SET((char)44705) ;
        p92.chan2_raw_SET((char)65450) ;
        p92.chan4_raw_SET((char)27221) ;
        p92.chan10_raw_SET((char)14826) ;
        p92.chan1_raw_SET((char)33251) ;
        p92.rssi_SET((char)96) ;
        p92.time_usec_SET(716432663992481926L) ;
        p92.chan8_raw_SET((char)14753) ;
        p92.chan6_raw_SET((char)25727) ;
        p92.chan9_raw_SET((char)46720) ;
        p92.chan7_raw_SET((char)56861) ;
        p92.chan5_raw_SET((char)53735) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.2537609E38F, 2.4026048E38F, 3.5927505E37F, -2.3875899E38F, -3.1336323E38F, 2.2425688E38F, 3.3140295E38F, 7.7335905E37F, -1.0093847E38F, 1.4639767E38F, -3.1559659E38F, 4.6277177E37F, 6.8309066E37F, 1.2240185E38F, 1.7805005E38F, -1.8294001E38F}));
            assert(pack.flags_GET() == 2913323665455611217L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_ARMED);
            assert(pack.time_usec_GET() == 7959531140974412061L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED) ;
        p93.controls_SET(new float[] {2.2537609E38F, 2.4026048E38F, 3.5927505E37F, -2.3875899E38F, -3.1336323E38F, 2.2425688E38F, 3.3140295E38F, 7.7335905E37F, -1.0093847E38F, 1.4639767E38F, -3.1559659E38F, 4.6277177E37F, 6.8309066E37F, 1.2240185E38F, 1.7805005E38F, -1.8294001E38F}, 0) ;
        p93.time_usec_SET(7959531140974412061L) ;
        p93.flags_SET(2913323665455611217L) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_comp_m_x_GET() == -1.6763729E38F);
            assert(pack.quality_GET() == (char)41);
            assert(pack.flow_comp_m_y_GET() == 2.8936317E38F);
            assert(pack.ground_distance_GET() == 1.6567891E38F);
            assert(pack.sensor_id_GET() == (char)53);
            assert(pack.flow_rate_y_TRY(ph) == 3.095504E37F);
            assert(pack.flow_x_GET() == (short) -30536);
            assert(pack.flow_rate_x_TRY(ph) == -1.3529495E38F);
            assert(pack.flow_y_GET() == (short)2159);
            assert(pack.time_usec_GET() == 8637313076328996904L);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_comp_m_y_SET(2.8936317E38F) ;
        p100.quality_SET((char)41) ;
        p100.flow_y_SET((short)2159) ;
        p100.time_usec_SET(8637313076328996904L) ;
        p100.ground_distance_SET(1.6567891E38F) ;
        p100.flow_comp_m_x_SET(-1.6763729E38F) ;
        p100.flow_rate_x_SET(-1.3529495E38F, PH) ;
        p100.flow_rate_y_SET(3.095504E37F, PH) ;
        p100.flow_x_SET((short) -30536) ;
        p100.sensor_id_SET((char)53) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.5783922E38F);
            assert(pack.y_GET() == 7.051541E37F);
            assert(pack.pitch_GET() == 5.8207833E37F);
            assert(pack.x_GET() == 1.4746402E38F);
            assert(pack.usec_GET() == 6205471719575946784L);
            assert(pack.yaw_GET() == -2.59275E38F);
            assert(pack.roll_GET() == 1.3815754E38F);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.yaw_SET(-2.59275E38F) ;
        p101.pitch_SET(5.8207833E37F) ;
        p101.usec_SET(6205471719575946784L) ;
        p101.z_SET(-1.5783922E38F) ;
        p101.x_SET(1.4746402E38F) ;
        p101.y_SET(7.051541E37F) ;
        p101.roll_SET(1.3815754E38F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.8198658E38F);
            assert(pack.pitch_GET() == -2.5975804E38F);
            assert(pack.usec_GET() == 1405556858268049734L);
            assert(pack.z_GET() == -2.2955365E38F);
            assert(pack.x_GET() == -1.4234209E38F);
            assert(pack.roll_GET() == -2.6505511E38F);
            assert(pack.yaw_GET() == 1.3867156E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(1.3867156E38F) ;
        p102.pitch_SET(-2.5975804E38F) ;
        p102.z_SET(-2.2955365E38F) ;
        p102.roll_SET(-2.6505511E38F) ;
        p102.y_SET(2.8198658E38F) ;
        p102.usec_SET(1405556858268049734L) ;
        p102.x_SET(-1.4234209E38F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.4822753E38F);
            assert(pack.usec_GET() == 6245580081556187688L);
            assert(pack.y_GET() == 1.5758335E38F);
            assert(pack.x_GET() == -2.1705478E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.x_SET(-2.1705478E38F) ;
        p103.z_SET(-2.4822753E38F) ;
        p103.usec_SET(6245580081556187688L) ;
        p103.y_SET(1.5758335E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 836318160593943269L);
            assert(pack.pitch_GET() == 3.9640035E37F);
            assert(pack.roll_GET() == -1.0095768E38F);
            assert(pack.y_GET() == 1.9281633E38F);
            assert(pack.x_GET() == -4.4447035E37F);
            assert(pack.z_GET() == -3.149304E38F);
            assert(pack.yaw_GET() == 7.006379E37F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(7.006379E37F) ;
        p104.x_SET(-4.4447035E37F) ;
        p104.y_SET(1.9281633E38F) ;
        p104.pitch_SET(3.9640035E37F) ;
        p104.z_SET(-3.149304E38F) ;
        p104.usec_SET(836318160593943269L) ;
        p104.roll_SET(-1.0095768E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == 2.0399147E38F);
            assert(pack.time_usec_GET() == 8935669181896334242L);
            assert(pack.xgyro_GET() == -3.1250283E38F);
            assert(pack.xacc_GET() == -1.3378893E38F);
            assert(pack.temperature_GET() == 2.5336082E38F);
            assert(pack.zgyro_GET() == 2.2880213E38F);
            assert(pack.ygyro_GET() == 3.2793602E38F);
            assert(pack.abs_pressure_GET() == 2.1547127E38F);
            assert(pack.pressure_alt_GET() == 1.1775148E38F);
            assert(pack.zacc_GET() == 3.0658811E38F);
            assert(pack.yacc_GET() == -1.0761388E38F);
            assert(pack.diff_pressure_GET() == -1.7715068E38F);
            assert(pack.xmag_GET() == -1.9326885E38F);
            assert(pack.fields_updated_GET() == (char)49039);
            assert(pack.zmag_GET() == -3.2458277E37F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zmag_SET(-3.2458277E37F) ;
        p105.xgyro_SET(-3.1250283E38F) ;
        p105.temperature_SET(2.5336082E38F) ;
        p105.zacc_SET(3.0658811E38F) ;
        p105.time_usec_SET(8935669181896334242L) ;
        p105.xmag_SET(-1.9326885E38F) ;
        p105.xacc_SET(-1.3378893E38F) ;
        p105.zgyro_SET(2.2880213E38F) ;
        p105.pressure_alt_SET(1.1775148E38F) ;
        p105.ygyro_SET(3.2793602E38F) ;
        p105.ymag_SET(2.0399147E38F) ;
        p105.abs_pressure_SET(2.1547127E38F) ;
        p105.yacc_SET(-1.0761388E38F) ;
        p105.fields_updated_SET((char)49039) ;
        p105.diff_pressure_SET(-1.7715068E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == 1.407339E38F);
            assert(pack.integrated_x_GET() == 2.554461E38F);
            assert(pack.quality_GET() == (char)110);
            assert(pack.integration_time_us_GET() == 1635098764L);
            assert(pack.temperature_GET() == (short)329);
            assert(pack.distance_GET() == 6.2592185E37F);
            assert(pack.time_usec_GET() == 5688922048365792937L);
            assert(pack.integrated_xgyro_GET() == -2.171693E37F);
            assert(pack.integrated_zgyro_GET() == 8.882955E37F);
            assert(pack.sensor_id_GET() == (char)84);
            assert(pack.integrated_ygyro_GET() == -1.6250192E38F);
            assert(pack.time_delta_distance_us_GET() == 917121366L);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(-1.6250192E38F) ;
        p106.quality_SET((char)110) ;
        p106.sensor_id_SET((char)84) ;
        p106.temperature_SET((short)329) ;
        p106.distance_SET(6.2592185E37F) ;
        p106.integrated_zgyro_SET(8.882955E37F) ;
        p106.integrated_y_SET(1.407339E38F) ;
        p106.integrated_xgyro_SET(-2.171693E37F) ;
        p106.integrated_x_SET(2.554461E38F) ;
        p106.integration_time_us_SET(1635098764L) ;
        p106.time_usec_SET(5688922048365792937L) ;
        p106.time_delta_distance_us_SET(917121366L) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == -1.0579692E38F);
            assert(pack.ymag_GET() == -2.1304268E38F);
            assert(pack.xmag_GET() == 1.1481011E38F);
            assert(pack.diff_pressure_GET() == -1.5088778E38F);
            assert(pack.zgyro_GET() == 9.324232E36F);
            assert(pack.ygyro_GET() == -1.7821518E38F);
            assert(pack.xgyro_GET() == 1.1463206E38F);
            assert(pack.yacc_GET() == -5.268454E37F);
            assert(pack.zmag_GET() == -6.73845E37F);
            assert(pack.time_usec_GET() == 6558956336211660289L);
            assert(pack.zacc_GET() == 2.2679403E38F);
            assert(pack.xacc_GET() == -3.0810877E38F);
            assert(pack.fields_updated_GET() == 365784209L);
            assert(pack.temperature_GET() == -6.634793E37F);
            assert(pack.pressure_alt_GET() == 9.774492E37F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(365784209L) ;
        p107.ygyro_SET(-1.7821518E38F) ;
        p107.diff_pressure_SET(-1.5088778E38F) ;
        p107.pressure_alt_SET(9.774492E37F) ;
        p107.zacc_SET(2.2679403E38F) ;
        p107.abs_pressure_SET(-1.0579692E38F) ;
        p107.xacc_SET(-3.0810877E38F) ;
        p107.xgyro_SET(1.1463206E38F) ;
        p107.xmag_SET(1.1481011E38F) ;
        p107.ymag_SET(-2.1304268E38F) ;
        p107.zmag_SET(-6.73845E37F) ;
        p107.time_usec_SET(6558956336211660289L) ;
        p107.temperature_SET(-6.634793E37F) ;
        p107.yacc_SET(-5.268454E37F) ;
        p107.zgyro_SET(9.324232E36F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == -1.0323531E38F);
            assert(pack.roll_GET() == 1.6014966E38F);
            assert(pack.yaw_GET() == 2.9256425E38F);
            assert(pack.alt_GET() == 3.3286416E38F);
            assert(pack.yacc_GET() == 3.1397444E38F);
            assert(pack.xacc_GET() == -2.3167828E38F);
            assert(pack.ve_GET() == -8.441984E37F);
            assert(pack.lat_GET() == 9.25797E37F);
            assert(pack.q2_GET() == 1.6270265E38F);
            assert(pack.pitch_GET() == 1.6969637E38F);
            assert(pack.zgyro_GET() == 1.831032E38F);
            assert(pack.q4_GET() == 2.0621377E38F);
            assert(pack.std_dev_vert_GET() == 4.3255747E37F);
            assert(pack.xgyro_GET() == -9.481279E37F);
            assert(pack.ygyro_GET() == 7.7793775E37F);
            assert(pack.vd_GET() == 1.1080954E38F);
            assert(pack.std_dev_horz_GET() == 1.0744922E38F);
            assert(pack.q1_GET() == 2.6924211E38F);
            assert(pack.vn_GET() == 5.2858444E37F);
            assert(pack.q3_GET() == 1.0451789E36F);
            assert(pack.lon_GET() == -3.305027E36F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q3_SET(1.0451789E36F) ;
        p108.lon_SET(-3.305027E36F) ;
        p108.std_dev_horz_SET(1.0744922E38F) ;
        p108.roll_SET(1.6014966E38F) ;
        p108.ygyro_SET(7.7793775E37F) ;
        p108.yacc_SET(3.1397444E38F) ;
        p108.vd_SET(1.1080954E38F) ;
        p108.q4_SET(2.0621377E38F) ;
        p108.alt_SET(3.3286416E38F) ;
        p108.zacc_SET(-1.0323531E38F) ;
        p108.pitch_SET(1.6969637E38F) ;
        p108.q1_SET(2.6924211E38F) ;
        p108.yaw_SET(2.9256425E38F) ;
        p108.vn_SET(5.2858444E37F) ;
        p108.std_dev_vert_SET(4.3255747E37F) ;
        p108.zgyro_SET(1.831032E38F) ;
        p108.lat_SET(9.25797E37F) ;
        p108.xgyro_SET(-9.481279E37F) ;
        p108.q2_SET(1.6270265E38F) ;
        p108.xacc_SET(-2.3167828E38F) ;
        p108.ve_SET(-8.441984E37F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rxerrors_GET() == (char)61991);
            assert(pack.txbuf_GET() == (char)131);
            assert(pack.remnoise_GET() == (char)206);
            assert(pack.noise_GET() == (char)71);
            assert(pack.rssi_GET() == (char)66);
            assert(pack.fixed__GET() == (char)65492);
            assert(pack.remrssi_GET() == (char)4);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.fixed__SET((char)65492) ;
        p109.noise_SET((char)71) ;
        p109.rxerrors_SET((char)61991) ;
        p109.txbuf_SET((char)131) ;
        p109.remrssi_SET((char)4) ;
        p109.rssi_SET((char)66) ;
        p109.remnoise_SET((char)206) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)151);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)31, (char)112, (char)98, (char)158, (char)131, (char)5, (char)61, (char)55, (char)44, (char)57, (char)233, (char)144, (char)165, (char)175, (char)6, (char)228, (char)18, (char)100, (char)95, (char)134, (char)214, (char)15, (char)205, (char)18, (char)213, (char)69, (char)194, (char)140, (char)253, (char)39, (char)249, (char)119, (char)140, (char)80, (char)201, (char)130, (char)66, (char)23, (char)110, (char)147, (char)134, (char)169, (char)34, (char)158, (char)228, (char)20, (char)13, (char)69, (char)164, (char)177, (char)205, (char)104, (char)41, (char)136, (char)47, (char)15, (char)49, (char)225, (char)237, (char)69, (char)236, (char)45, (char)143, (char)188, (char)52, (char)229, (char)4, (char)233, (char)38, (char)217, (char)67, (char)231, (char)196, (char)93, (char)249, (char)151, (char)201, (char)232, (char)215, (char)32, (char)153, (char)235, (char)175, (char)71, (char)239, (char)205, (char)35, (char)8, (char)97, (char)78, (char)170, (char)47, (char)119, (char)223, (char)56, (char)219, (char)119, (char)193, (char)250, (char)180, (char)138, (char)160, (char)208, (char)9, (char)134, (char)135, (char)163, (char)11, (char)221, (char)74, (char)112, (char)4, (char)161, (char)166, (char)118, (char)54, (char)48, (char)98, (char)188, (char)218, (char)255, (char)128, (char)176, (char)186, (char)211, (char)253, (char)49, (char)115, (char)119, (char)35, (char)61, (char)113, (char)21, (char)163, (char)230, (char)93, (char)155, (char)181, (char)89, (char)126, (char)231, (char)185, (char)229, (char)45, (char)203, (char)34, (char)229, (char)251, (char)148, (char)138, (char)170, (char)100, (char)64, (char)190, (char)200, (char)109, (char)76, (char)45, (char)239, (char)151, (char)59, (char)29, (char)67, (char)0, (char)55, (char)131, (char)244, (char)97, (char)67, (char)172, (char)63, (char)89, (char)100, (char)190, (char)195, (char)78, (char)209, (char)85, (char)156, (char)113, (char)146, (char)247, (char)190, (char)41, (char)251, (char)173, (char)26, (char)143, (char)200, (char)87, (char)42, (char)232, (char)91, (char)11, (char)233, (char)10, (char)229, (char)47, (char)75, (char)139, (char)249, (char)217, (char)202, (char)107, (char)169, (char)94, (char)16, (char)143, (char)42, (char)11, (char)67, (char)17, (char)254, (char)6, (char)126, (char)153, (char)19, (char)150, (char)44, (char)88, (char)10, (char)133, (char)212, (char)90, (char)210, (char)17, (char)0, (char)192, (char)171, (char)108, (char)174, (char)172, (char)250, (char)13, (char)168, (char)192, (char)182, (char)172, (char)246, (char)228, (char)208, (char)181, (char)62, (char)117, (char)227, (char)251, (char)234, (char)126, (char)124, (char)155, (char)196}));
            assert(pack.target_network_GET() == (char)182);
            assert(pack.target_system_GET() == (char)200);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)151) ;
        p110.target_system_SET((char)200) ;
        p110.target_network_SET((char)182) ;
        p110.payload_SET(new char[] {(char)31, (char)112, (char)98, (char)158, (char)131, (char)5, (char)61, (char)55, (char)44, (char)57, (char)233, (char)144, (char)165, (char)175, (char)6, (char)228, (char)18, (char)100, (char)95, (char)134, (char)214, (char)15, (char)205, (char)18, (char)213, (char)69, (char)194, (char)140, (char)253, (char)39, (char)249, (char)119, (char)140, (char)80, (char)201, (char)130, (char)66, (char)23, (char)110, (char)147, (char)134, (char)169, (char)34, (char)158, (char)228, (char)20, (char)13, (char)69, (char)164, (char)177, (char)205, (char)104, (char)41, (char)136, (char)47, (char)15, (char)49, (char)225, (char)237, (char)69, (char)236, (char)45, (char)143, (char)188, (char)52, (char)229, (char)4, (char)233, (char)38, (char)217, (char)67, (char)231, (char)196, (char)93, (char)249, (char)151, (char)201, (char)232, (char)215, (char)32, (char)153, (char)235, (char)175, (char)71, (char)239, (char)205, (char)35, (char)8, (char)97, (char)78, (char)170, (char)47, (char)119, (char)223, (char)56, (char)219, (char)119, (char)193, (char)250, (char)180, (char)138, (char)160, (char)208, (char)9, (char)134, (char)135, (char)163, (char)11, (char)221, (char)74, (char)112, (char)4, (char)161, (char)166, (char)118, (char)54, (char)48, (char)98, (char)188, (char)218, (char)255, (char)128, (char)176, (char)186, (char)211, (char)253, (char)49, (char)115, (char)119, (char)35, (char)61, (char)113, (char)21, (char)163, (char)230, (char)93, (char)155, (char)181, (char)89, (char)126, (char)231, (char)185, (char)229, (char)45, (char)203, (char)34, (char)229, (char)251, (char)148, (char)138, (char)170, (char)100, (char)64, (char)190, (char)200, (char)109, (char)76, (char)45, (char)239, (char)151, (char)59, (char)29, (char)67, (char)0, (char)55, (char)131, (char)244, (char)97, (char)67, (char)172, (char)63, (char)89, (char)100, (char)190, (char)195, (char)78, (char)209, (char)85, (char)156, (char)113, (char)146, (char)247, (char)190, (char)41, (char)251, (char)173, (char)26, (char)143, (char)200, (char)87, (char)42, (char)232, (char)91, (char)11, (char)233, (char)10, (char)229, (char)47, (char)75, (char)139, (char)249, (char)217, (char)202, (char)107, (char)169, (char)94, (char)16, (char)143, (char)42, (char)11, (char)67, (char)17, (char)254, (char)6, (char)126, (char)153, (char)19, (char)150, (char)44, (char)88, (char)10, (char)133, (char)212, (char)90, (char)210, (char)17, (char)0, (char)192, (char)171, (char)108, (char)174, (char)172, (char)250, (char)13, (char)168, (char)192, (char)182, (char)172, (char)246, (char)228, (char)208, (char)181, (char)62, (char)117, (char)227, (char)251, (char)234, (char)126, (char)124, (char)155, (char)196}, 0) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == 8883134003107719518L);
            assert(pack.ts1_GET() == -3506538422247100948L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(8883134003107719518L) ;
        p111.ts1_SET(-3506538422247100948L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 4010057651L);
            assert(pack.time_usec_GET() == 4760717266411644954L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(4760717266411644954L) ;
        p112.seq_SET(4010057651L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1322308507895249940L);
            assert(pack.alt_GET() == 730971874);
            assert(pack.eph_GET() == (char)57404);
            assert(pack.satellites_visible_GET() == (char)140);
            assert(pack.ve_GET() == (short)12666);
            assert(pack.cog_GET() == (char)39759);
            assert(pack.lat_GET() == 1549348557);
            assert(pack.vd_GET() == (short) -25342);
            assert(pack.vn_GET() == (short)28589);
            assert(pack.lon_GET() == 487551286);
            assert(pack.epv_GET() == (char)15824);
            assert(pack.fix_type_GET() == (char)129);
            assert(pack.vel_GET() == (char)27165);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.ve_SET((short)12666) ;
        p113.vn_SET((short)28589) ;
        p113.time_usec_SET(1322308507895249940L) ;
        p113.lat_SET(1549348557) ;
        p113.satellites_visible_SET((char)140) ;
        p113.vd_SET((short) -25342) ;
        p113.eph_SET((char)57404) ;
        p113.vel_SET((char)27165) ;
        p113.epv_SET((char)15824) ;
        p113.fix_type_SET((char)129) ;
        p113.alt_SET(730971874) ;
        p113.cog_SET((char)39759) ;
        p113.lon_SET(487551286) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_zgyro_GET() == -2.2509802E38F);
            assert(pack.integration_time_us_GET() == 4225153682L);
            assert(pack.integrated_xgyro_GET() == -3.912218E37F);
            assert(pack.sensor_id_GET() == (char)161);
            assert(pack.temperature_GET() == (short) -21113);
            assert(pack.integrated_ygyro_GET() == -7.4805837E37F);
            assert(pack.integrated_x_GET() == -8.4343163E37F);
            assert(pack.time_usec_GET() == 4088984878915286056L);
            assert(pack.time_delta_distance_us_GET() == 2732773213L);
            assert(pack.distance_GET() == 2.2123387E38F);
            assert(pack.quality_GET() == (char)82);
            assert(pack.integrated_y_GET() == -1.4103528E37F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(-3.912218E37F) ;
        p114.sensor_id_SET((char)161) ;
        p114.temperature_SET((short) -21113) ;
        p114.distance_SET(2.2123387E38F) ;
        p114.time_usec_SET(4088984878915286056L) ;
        p114.integration_time_us_SET(4225153682L) ;
        p114.integrated_y_SET(-1.4103528E37F) ;
        p114.time_delta_distance_us_SET(2732773213L) ;
        p114.integrated_zgyro_SET(-2.2509802E38F) ;
        p114.integrated_ygyro_SET(-7.4805837E37F) ;
        p114.integrated_x_SET(-8.4343163E37F) ;
        p114.quality_SET((char)82) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -4.9066627E37F);
            assert(pack.lon_GET() == -691691104);
            assert(pack.vx_GET() == (short) -35);
            assert(pack.yawspeed_GET() == 2.0255322E38F);
            assert(pack.zacc_GET() == (short)28230);
            assert(pack.ind_airspeed_GET() == (char)27096);
            assert(pack.true_airspeed_GET() == (char)37621);
            assert(pack.alt_GET() == -1038329203);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-2.0794605E38F, 2.0308456E38F, -3.3754426E38F, -3.1305412E38F}));
            assert(pack.time_usec_GET() == 2837708936696572055L);
            assert(pack.vz_GET() == (short)9345);
            assert(pack.lat_GET() == 683741425);
            assert(pack.yacc_GET() == (short)4358);
            assert(pack.pitchspeed_GET() == 1.7570808E38F);
            assert(pack.vy_GET() == (short)15909);
            assert(pack.xacc_GET() == (short) -6736);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.yawspeed_SET(2.0255322E38F) ;
        p115.alt_SET(-1038329203) ;
        p115.ind_airspeed_SET((char)27096) ;
        p115.vz_SET((short)9345) ;
        p115.true_airspeed_SET((char)37621) ;
        p115.pitchspeed_SET(1.7570808E38F) ;
        p115.vx_SET((short) -35) ;
        p115.rollspeed_SET(-4.9066627E37F) ;
        p115.xacc_SET((short) -6736) ;
        p115.attitude_quaternion_SET(new float[] {-2.0794605E38F, 2.0308456E38F, -3.3754426E38F, -3.1305412E38F}, 0) ;
        p115.yacc_SET((short)4358) ;
        p115.lat_SET(683741425) ;
        p115.zacc_SET((short)28230) ;
        p115.time_usec_SET(2837708936696572055L) ;
        p115.lon_SET(-691691104) ;
        p115.vy_SET((short)15909) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)4226);
            assert(pack.yacc_GET() == (short) -4322);
            assert(pack.ymag_GET() == (short)5867);
            assert(pack.time_boot_ms_GET() == 2725703868L);
            assert(pack.xmag_GET() == (short) -5140);
            assert(pack.zgyro_GET() == (short)25694);
            assert(pack.xgyro_GET() == (short)18712);
            assert(pack.zmag_GET() == (short) -23426);
            assert(pack.zacc_GET() == (short) -351);
            assert(pack.xacc_GET() == (short) -13948);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short) -5140) ;
        p116.xacc_SET((short) -13948) ;
        p116.zgyro_SET((short)25694) ;
        p116.yacc_SET((short) -4322) ;
        p116.zacc_SET((short) -351) ;
        p116.ygyro_SET((short)4226) ;
        p116.xgyro_SET((short)18712) ;
        p116.time_boot_ms_SET(2725703868L) ;
        p116.ymag_SET((short)5867) ;
        p116.zmag_SET((short) -23426) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)6873);
            assert(pack.target_system_GET() == (char)96);
            assert(pack.target_component_GET() == (char)30);
            assert(pack.start_GET() == (char)55742);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)30) ;
        p117.target_system_SET((char)96) ;
        p117.end_SET((char)6873) ;
        p117.start_SET((char)55742) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.last_log_num_GET() == (char)9832);
            assert(pack.id_GET() == (char)18265);
            assert(pack.num_logs_GET() == (char)43355);
            assert(pack.size_GET() == 2440017224L);
            assert(pack.time_utc_GET() == 3525582269L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.num_logs_SET((char)43355) ;
        p118.size_SET(2440017224L) ;
        p118.id_SET((char)18265) ;
        p118.last_log_num_SET((char)9832) ;
        p118.time_utc_SET(3525582269L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)44538);
            assert(pack.count_GET() == 3455317950L);
            assert(pack.target_component_GET() == (char)31);
            assert(pack.target_system_GET() == (char)84);
            assert(pack.ofs_GET() == 160371199L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)84) ;
        p119.id_SET((char)44538) ;
        p119.target_component_SET((char)31) ;
        p119.count_SET(3455317950L) ;
        p119.ofs_SET(160371199L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)98, (char)111, (char)179, (char)166, (char)178, (char)180, (char)227, (char)163, (char)6, (char)38, (char)101, (char)212, (char)242, (char)59, (char)69, (char)117, (char)111, (char)231, (char)77, (char)84, (char)181, (char)88, (char)228, (char)88, (char)173, (char)230, (char)110, (char)231, (char)187, (char)167, (char)46, (char)68, (char)190, (char)95, (char)229, (char)242, (char)229, (char)236, (char)250, (char)145, (char)223, (char)183, (char)174, (char)12, (char)192, (char)57, (char)34, (char)165, (char)225, (char)55, (char)30, (char)184, (char)16, (char)76, (char)119, (char)47, (char)110, (char)77, (char)124, (char)196, (char)169, (char)0, (char)87, (char)73, (char)118, (char)70, (char)104, (char)216, (char)54, (char)206, (char)42, (char)221, (char)104, (char)40, (char)197, (char)224, (char)102, (char)129, (char)113, (char)231, (char)5, (char)17, (char)108, (char)136, (char)213, (char)2, (char)251, (char)43, (char)149, (char)248}));
            assert(pack.count_GET() == (char)252);
            assert(pack.id_GET() == (char)1340);
            assert(pack.ofs_GET() == 4068497416L);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(4068497416L) ;
        p120.id_SET((char)1340) ;
        p120.count_SET((char)252) ;
        p120.data__SET(new char[] {(char)98, (char)111, (char)179, (char)166, (char)178, (char)180, (char)227, (char)163, (char)6, (char)38, (char)101, (char)212, (char)242, (char)59, (char)69, (char)117, (char)111, (char)231, (char)77, (char)84, (char)181, (char)88, (char)228, (char)88, (char)173, (char)230, (char)110, (char)231, (char)187, (char)167, (char)46, (char)68, (char)190, (char)95, (char)229, (char)242, (char)229, (char)236, (char)250, (char)145, (char)223, (char)183, (char)174, (char)12, (char)192, (char)57, (char)34, (char)165, (char)225, (char)55, (char)30, (char)184, (char)16, (char)76, (char)119, (char)47, (char)110, (char)77, (char)124, (char)196, (char)169, (char)0, (char)87, (char)73, (char)118, (char)70, (char)104, (char)216, (char)54, (char)206, (char)42, (char)221, (char)104, (char)40, (char)197, (char)224, (char)102, (char)129, (char)113, (char)231, (char)5, (char)17, (char)108, (char)136, (char)213, (char)2, (char)251, (char)43, (char)149, (char)248}, 0) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)119);
            assert(pack.target_system_GET() == (char)16);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)16) ;
        p121.target_component_SET((char)119) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)254);
            assert(pack.target_component_GET() == (char)56);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)254) ;
        p122.target_component_SET((char)56) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)69, (char)46, (char)194, (char)240, (char)183, (char)41, (char)235, (char)80, (char)49, (char)222, (char)4, (char)125, (char)227, (char)122, (char)33, (char)233, (char)194, (char)239, (char)7, (char)201, (char)51, (char)241, (char)38, (char)71, (char)114, (char)112, (char)81, (char)77, (char)214, (char)47, (char)135, (char)143, (char)236, (char)215, (char)208, (char)149, (char)84, (char)81, (char)71, (char)40, (char)90, (char)29, (char)134, (char)67, (char)227, (char)47, (char)99, (char)136, (char)173, (char)134, (char)87, (char)45, (char)245, (char)143, (char)66, (char)139, (char)115, (char)83, (char)69, (char)236, (char)203, (char)110, (char)123, (char)63, (char)214, (char)218, (char)91, (char)177, (char)141, (char)8, (char)30, (char)18, (char)38, (char)198, (char)110, (char)73, (char)245, (char)219, (char)222, (char)225, (char)87, (char)169, (char)160, (char)95, (char)156, (char)68, (char)126, (char)48, (char)179, (char)11, (char)90, (char)19, (char)139, (char)242, (char)97, (char)244, (char)251, (char)57, (char)184, (char)239, (char)173, (char)119, (char)11, (char)246, (char)66, (char)62, (char)89, (char)133, (char)44, (char)235}));
            assert(pack.target_component_GET() == (char)189);
            assert(pack.len_GET() == (char)161);
            assert(pack.target_system_GET() == (char)196);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)196) ;
        p123.len_SET((char)161) ;
        p123.data__SET(new char[] {(char)69, (char)46, (char)194, (char)240, (char)183, (char)41, (char)235, (char)80, (char)49, (char)222, (char)4, (char)125, (char)227, (char)122, (char)33, (char)233, (char)194, (char)239, (char)7, (char)201, (char)51, (char)241, (char)38, (char)71, (char)114, (char)112, (char)81, (char)77, (char)214, (char)47, (char)135, (char)143, (char)236, (char)215, (char)208, (char)149, (char)84, (char)81, (char)71, (char)40, (char)90, (char)29, (char)134, (char)67, (char)227, (char)47, (char)99, (char)136, (char)173, (char)134, (char)87, (char)45, (char)245, (char)143, (char)66, (char)139, (char)115, (char)83, (char)69, (char)236, (char)203, (char)110, (char)123, (char)63, (char)214, (char)218, (char)91, (char)177, (char)141, (char)8, (char)30, (char)18, (char)38, (char)198, (char)110, (char)73, (char)245, (char)219, (char)222, (char)225, (char)87, (char)169, (char)160, (char)95, (char)156, (char)68, (char)126, (char)48, (char)179, (char)11, (char)90, (char)19, (char)139, (char)242, (char)97, (char)244, (char)251, (char)57, (char)184, (char)239, (char)173, (char)119, (char)11, (char)246, (char)66, (char)62, (char)89, (char)133, (char)44, (char)235}, 0) ;
        p123.target_component_SET((char)189) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)26343);
            assert(pack.alt_GET() == 1166836921);
            assert(pack.time_usec_GET() == 9151796055180425778L);
            assert(pack.epv_GET() == (char)38585);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.lat_GET() == -1508617299);
            assert(pack.lon_GET() == -1039783655);
            assert(pack.dgps_age_GET() == 2751712086L);
            assert(pack.dgps_numch_GET() == (char)141);
            assert(pack.eph_GET() == (char)50583);
            assert(pack.satellites_visible_GET() == (char)122);
            assert(pack.vel_GET() == (char)32124);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)141) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.time_usec_SET(9151796055180425778L) ;
        p124.epv_SET((char)38585) ;
        p124.lat_SET(-1508617299) ;
        p124.alt_SET(1166836921) ;
        p124.cog_SET((char)26343) ;
        p124.vel_SET((char)32124) ;
        p124.eph_SET((char)50583) ;
        p124.dgps_age_SET(2751712086L) ;
        p124.satellites_visible_SET((char)122) ;
        p124.lon_SET(-1039783655) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)17811);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            assert(pack.Vcc_GET() == (char)25987);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)25987) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED) ;
        p125.Vservo_SET((char)17811) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)92);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)221, (char)132, (char)182, (char)202, (char)106, (char)186, (char)78, (char)71, (char)57, (char)104, (char)21, (char)197, (char)232, (char)41, (char)145, (char)56, (char)150, (char)91, (char)179, (char)28, (char)133, (char)109, (char)104, (char)175, (char)36, (char)104, (char)230, (char)181, (char)72, (char)233, (char)13, (char)94, (char)191, (char)153, (char)37, (char)250, (char)83, (char)48, (char)43, (char)92, (char)171, (char)227, (char)217, (char)105, (char)101, (char)240, (char)54, (char)199, (char)145, (char)236, (char)22, (char)65, (char)5, (char)154, (char)191, (char)96, (char)64, (char)239, (char)176, (char)127, (char)2, (char)66, (char)1, (char)16, (char)232, (char)165, (char)12, (char)15, (char)224, (char)166}));
            assert(pack.baudrate_GET() == 1325179642L);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.timeout_GET() == (char)59055);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(1325179642L) ;
        p126.data__SET(new char[] {(char)221, (char)132, (char)182, (char)202, (char)106, (char)186, (char)78, (char)71, (char)57, (char)104, (char)21, (char)197, (char)232, (char)41, (char)145, (char)56, (char)150, (char)91, (char)179, (char)28, (char)133, (char)109, (char)104, (char)175, (char)36, (char)104, (char)230, (char)181, (char)72, (char)233, (char)13, (char)94, (char)191, (char)153, (char)37, (char)250, (char)83, (char)48, (char)43, (char)92, (char)171, (char)227, (char)217, (char)105, (char)101, (char)240, (char)54, (char)199, (char)145, (char)236, (char)22, (char)65, (char)5, (char)154, (char)191, (char)96, (char)64, (char)239, (char)176, (char)127, (char)2, (char)66, (char)1, (char)16, (char)232, (char)165, (char)12, (char)15, (char)224, (char)166}, 0) ;
        p126.count_SET((char)92) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI) ;
        p126.timeout_SET((char)59055) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_c_mm_GET() == -1846292785);
            assert(pack.accuracy_GET() == 2968508788L);
            assert(pack.time_last_baseline_ms_GET() == 247048099L);
            assert(pack.tow_GET() == 3438885686L);
            assert(pack.wn_GET() == (char)2361);
            assert(pack.nsats_GET() == (char)120);
            assert(pack.rtk_health_GET() == (char)108);
            assert(pack.iar_num_hypotheses_GET() == -791571740);
            assert(pack.rtk_rate_GET() == (char)132);
            assert(pack.baseline_coords_type_GET() == (char)125);
            assert(pack.rtk_receiver_id_GET() == (char)74);
            assert(pack.baseline_b_mm_GET() == -714795011);
            assert(pack.baseline_a_mm_GET() == -1171032995);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_coords_type_SET((char)125) ;
        p127.iar_num_hypotheses_SET(-791571740) ;
        p127.rtk_receiver_id_SET((char)74) ;
        p127.nsats_SET((char)120) ;
        p127.accuracy_SET(2968508788L) ;
        p127.baseline_b_mm_SET(-714795011) ;
        p127.rtk_health_SET((char)108) ;
        p127.rtk_rate_SET((char)132) ;
        p127.time_last_baseline_ms_SET(247048099L) ;
        p127.baseline_a_mm_SET(-1171032995) ;
        p127.baseline_c_mm_SET(-1846292785) ;
        p127.tow_SET(3438885686L) ;
        p127.wn_SET((char)2361) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_coords_type_GET() == (char)182);
            assert(pack.baseline_b_mm_GET() == -544389213);
            assert(pack.wn_GET() == (char)8423);
            assert(pack.time_last_baseline_ms_GET() == 2600118702L);
            assert(pack.rtk_health_GET() == (char)240);
            assert(pack.rtk_rate_GET() == (char)180);
            assert(pack.rtk_receiver_id_GET() == (char)128);
            assert(pack.iar_num_hypotheses_GET() == 1970874609);
            assert(pack.tow_GET() == 3595376005L);
            assert(pack.accuracy_GET() == 1760665713L);
            assert(pack.nsats_GET() == (char)71);
            assert(pack.baseline_a_mm_GET() == -1469481733);
            assert(pack.baseline_c_mm_GET() == -1549170284);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_a_mm_SET(-1469481733) ;
        p128.rtk_receiver_id_SET((char)128) ;
        p128.wn_SET((char)8423) ;
        p128.rtk_rate_SET((char)180) ;
        p128.nsats_SET((char)71) ;
        p128.baseline_b_mm_SET(-544389213) ;
        p128.rtk_health_SET((char)240) ;
        p128.accuracy_SET(1760665713L) ;
        p128.iar_num_hypotheses_SET(1970874609) ;
        p128.baseline_c_mm_SET(-1549170284) ;
        p128.tow_SET(3595376005L) ;
        p128.time_last_baseline_ms_SET(2600118702L) ;
        p128.baseline_coords_type_SET((char)182) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)9871);
            assert(pack.zgyro_GET() == (short) -18348);
            assert(pack.yacc_GET() == (short) -26179);
            assert(pack.xacc_GET() == (short)2867);
            assert(pack.xgyro_GET() == (short) -12789);
            assert(pack.zmag_GET() == (short) -25811);
            assert(pack.zacc_GET() == (short) -3479);
            assert(pack.time_boot_ms_GET() == 4281487296L);
            assert(pack.xmag_GET() == (short)20513);
            assert(pack.ymag_GET() == (short) -1722);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xmag_SET((short)20513) ;
        p129.ygyro_SET((short)9871) ;
        p129.ymag_SET((short) -1722) ;
        p129.time_boot_ms_SET(4281487296L) ;
        p129.xacc_SET((short)2867) ;
        p129.xgyro_SET((short) -12789) ;
        p129.zgyro_SET((short) -18348) ;
        p129.zmag_SET((short) -25811) ;
        p129.zacc_SET((short) -3479) ;
        p129.yacc_SET((short) -26179) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)25646);
            assert(pack.payload_GET() == (char)233);
            assert(pack.size_GET() == 4213417847L);
            assert(pack.packets_GET() == (char)34960);
            assert(pack.width_GET() == (char)35828);
            assert(pack.jpg_quality_GET() == (char)47);
            assert(pack.type_GET() == (char)251);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.payload_SET((char)233) ;
        p130.width_SET((char)35828) ;
        p130.type_SET((char)251) ;
        p130.height_SET((char)25646) ;
        p130.packets_SET((char)34960) ;
        p130.jpg_quality_SET((char)47) ;
        p130.size_SET(4213417847L) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)37861);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)60, (char)73, (char)135, (char)191, (char)205, (char)132, (char)94, (char)187, (char)63, (char)83, (char)213, (char)35, (char)25, (char)111, (char)0, (char)197, (char)111, (char)176, (char)241, (char)23, (char)15, (char)97, (char)255, (char)152, (char)44, (char)198, (char)232, (char)249, (char)204, (char)174, (char)132, (char)178, (char)223, (char)1, (char)126, (char)87, (char)239, (char)220, (char)214, (char)107, (char)203, (char)54, (char)93, (char)132, (char)246, (char)216, (char)230, (char)36, (char)98, (char)15, (char)144, (char)33, (char)102, (char)208, (char)76, (char)40, (char)55, (char)141, (char)105, (char)107, (char)181, (char)236, (char)249, (char)182, (char)255, (char)171, (char)146, (char)220, (char)6, (char)243, (char)117, (char)221, (char)137, (char)33, (char)77, (char)228, (char)206, (char)102, (char)176, (char)243, (char)157, (char)20, (char)1, (char)18, (char)150, (char)2, (char)230, (char)13, (char)209, (char)71, (char)56, (char)117, (char)49, (char)84, (char)141, (char)168, (char)215, (char)10, (char)184, (char)180, (char)59, (char)187, (char)144, (char)133, (char)126, (char)97, (char)178, (char)47, (char)47, (char)162, (char)227, (char)55, (char)104, (char)45, (char)68, (char)104, (char)72, (char)8, (char)165, (char)255, (char)76, (char)18, (char)78, (char)129, (char)102, (char)77, (char)75, (char)180, (char)149, (char)251, (char)73, (char)137, (char)134, (char)2, (char)19, (char)66, (char)163, (char)129, (char)53, (char)243, (char)178, (char)69, (char)134, (char)153, (char)248, (char)4, (char)161, (char)4, (char)31, (char)125, (char)114, (char)217, (char)156, (char)188, (char)116, (char)103, (char)239, (char)129, (char)170, (char)39, (char)113, (char)13, (char)128, (char)203, (char)202, (char)145, (char)126, (char)145, (char)10, (char)25, (char)37, (char)248, (char)73, (char)113, (char)126, (char)175, (char)149, (char)50, (char)11, (char)227, (char)1, (char)171, (char)0, (char)178, (char)90, (char)68, (char)43, (char)119, (char)159, (char)96, (char)117, (char)38, (char)171, (char)62, (char)243, (char)43, (char)104, (char)223, (char)248, (char)64, (char)237, (char)242, (char)221, (char)193, (char)239, (char)153, (char)178, (char)36, (char)37, (char)119, (char)92, (char)57, (char)157, (char)121, (char)29, (char)42, (char)129, (char)15, (char)23, (char)222, (char)120, (char)44, (char)72, (char)211, (char)8, (char)103, (char)165, (char)13, (char)251, (char)252, (char)152, (char)80, (char)223, (char)171, (char)27, (char)67, (char)136, (char)34, (char)98, (char)114, (char)140, (char)129, (char)229, (char)91, (char)224, (char)81, (char)219, (char)39, (char)216, (char)147, (char)191, (char)236, (char)232}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)37861) ;
        p131.data__SET(new char[] {(char)60, (char)73, (char)135, (char)191, (char)205, (char)132, (char)94, (char)187, (char)63, (char)83, (char)213, (char)35, (char)25, (char)111, (char)0, (char)197, (char)111, (char)176, (char)241, (char)23, (char)15, (char)97, (char)255, (char)152, (char)44, (char)198, (char)232, (char)249, (char)204, (char)174, (char)132, (char)178, (char)223, (char)1, (char)126, (char)87, (char)239, (char)220, (char)214, (char)107, (char)203, (char)54, (char)93, (char)132, (char)246, (char)216, (char)230, (char)36, (char)98, (char)15, (char)144, (char)33, (char)102, (char)208, (char)76, (char)40, (char)55, (char)141, (char)105, (char)107, (char)181, (char)236, (char)249, (char)182, (char)255, (char)171, (char)146, (char)220, (char)6, (char)243, (char)117, (char)221, (char)137, (char)33, (char)77, (char)228, (char)206, (char)102, (char)176, (char)243, (char)157, (char)20, (char)1, (char)18, (char)150, (char)2, (char)230, (char)13, (char)209, (char)71, (char)56, (char)117, (char)49, (char)84, (char)141, (char)168, (char)215, (char)10, (char)184, (char)180, (char)59, (char)187, (char)144, (char)133, (char)126, (char)97, (char)178, (char)47, (char)47, (char)162, (char)227, (char)55, (char)104, (char)45, (char)68, (char)104, (char)72, (char)8, (char)165, (char)255, (char)76, (char)18, (char)78, (char)129, (char)102, (char)77, (char)75, (char)180, (char)149, (char)251, (char)73, (char)137, (char)134, (char)2, (char)19, (char)66, (char)163, (char)129, (char)53, (char)243, (char)178, (char)69, (char)134, (char)153, (char)248, (char)4, (char)161, (char)4, (char)31, (char)125, (char)114, (char)217, (char)156, (char)188, (char)116, (char)103, (char)239, (char)129, (char)170, (char)39, (char)113, (char)13, (char)128, (char)203, (char)202, (char)145, (char)126, (char)145, (char)10, (char)25, (char)37, (char)248, (char)73, (char)113, (char)126, (char)175, (char)149, (char)50, (char)11, (char)227, (char)1, (char)171, (char)0, (char)178, (char)90, (char)68, (char)43, (char)119, (char)159, (char)96, (char)117, (char)38, (char)171, (char)62, (char)243, (char)43, (char)104, (char)223, (char)248, (char)64, (char)237, (char)242, (char)221, (char)193, (char)239, (char)153, (char)178, (char)36, (char)37, (char)119, (char)92, (char)57, (char)157, (char)121, (char)29, (char)42, (char)129, (char)15, (char)23, (char)222, (char)120, (char)44, (char)72, (char)211, (char)8, (char)103, (char)165, (char)13, (char)251, (char)252, (char)152, (char)80, (char)223, (char)171, (char)27, (char)67, (char)136, (char)34, (char)98, (char)114, (char)140, (char)129, (char)229, (char)91, (char)224, (char)81, (char)219, (char)39, (char)216, (char)147, (char)191, (char)236, (char)232}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)52212);
            assert(pack.current_distance_GET() == (char)60669);
            assert(pack.id_GET() == (char)95);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.min_distance_GET() == (char)19821);
            assert(pack.time_boot_ms_GET() == 3487567012L);
            assert(pack.covariance_GET() == (char)92);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p132.id_SET((char)95) ;
        p132.min_distance_SET((char)19821) ;
        p132.time_boot_ms_SET(3487567012L) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) ;
        p132.covariance_SET((char)92) ;
        p132.max_distance_SET((char)52212) ;
        p132.current_distance_SET((char)60669) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)51378);
            assert(pack.mask_GET() == 6786786165413157221L);
            assert(pack.lat_GET() == -790441798);
            assert(pack.lon_GET() == 379437978);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(379437978) ;
        p133.mask_SET(6786786165413157221L) ;
        p133.lat_SET(-790441798) ;
        p133.grid_spacing_SET((char)51378) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1691327973);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)29544, (short) -32680, (short) -32155, (short) -7095, (short)28667, (short) -2042, (short)25698, (short)8207, (short)32028, (short) -1274, (short) -4740, (short) -24600, (short)32351, (short) -26312, (short)17047, (short)3202}));
            assert(pack.lat_GET() == -132552474);
            assert(pack.grid_spacing_GET() == (char)26930);
            assert(pack.gridbit_GET() == (char)185);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)185) ;
        p134.lon_SET(1691327973) ;
        p134.grid_spacing_SET((char)26930) ;
        p134.data__SET(new short[] {(short)29544, (short) -32680, (short) -32155, (short) -7095, (short)28667, (short) -2042, (short)25698, (short)8207, (short)32028, (short) -1274, (short) -4740, (short) -24600, (short)32351, (short) -26312, (short)17047, (short)3202}, 0) ;
        p134.lat_SET(-132552474) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1675514396);
            assert(pack.lon_GET() == 1304446114);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1675514396) ;
        p135.lon_SET(1304446114) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == 2.146102E38F);
            assert(pack.lon_GET() == 1922683047);
            assert(pack.loaded_GET() == (char)43889);
            assert(pack.lat_GET() == -823543447);
            assert(pack.pending_GET() == (char)22455);
            assert(pack.current_height_GET() == -3.3144546E38F);
            assert(pack.spacing_GET() == (char)62681);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.loaded_SET((char)43889) ;
        p136.spacing_SET((char)62681) ;
        p136.current_height_SET(-3.3144546E38F) ;
        p136.lon_SET(1922683047) ;
        p136.terrain_height_SET(2.146102E38F) ;
        p136.lat_SET(-823543447) ;
        p136.pending_SET((char)22455) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == -2.0861629E38F);
            assert(pack.press_abs_GET() == 1.824575E38F);
            assert(pack.temperature_GET() == (short)9580);
            assert(pack.time_boot_ms_GET() == 302803215L);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(-2.0861629E38F) ;
        p137.press_abs_SET(1.824575E38F) ;
        p137.time_boot_ms_SET(302803215L) ;
        p137.temperature_SET((short)9580) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.0972018E37F);
            assert(pack.z_GET() == 1.8149432E38F);
            assert(pack.y_GET() == -8.16643E37F);
            assert(pack.time_usec_GET() == 6104972166406805067L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.6849268E38F, -2.4138933E38F, 3.2047065E38F, -3.0311065E38F}));
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {2.6849268E38F, -2.4138933E38F, 3.2047065E38F, -3.0311065E38F}, 0) ;
        p138.y_SET(-8.16643E37F) ;
        p138.z_SET(1.8149432E38F) ;
        p138.time_usec_SET(6104972166406805067L) ;
        p138.x_SET(2.0972018E37F) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7655490229420111131L);
            assert(pack.group_mlx_GET() == (char)221);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.7605764E37F, -3.252375E38F, 1.0048595E38F, 2.9091644E38F, 2.639998E38F, 1.5434589E37F, -2.9843318E38F, 4.642797E37F}));
            assert(pack.target_component_GET() == (char)228);
            assert(pack.target_system_GET() == (char)255);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_component_SET((char)228) ;
        p139.group_mlx_SET((char)221) ;
        p139.time_usec_SET(7655490229420111131L) ;
        p139.controls_SET(new float[] {-1.7605764E37F, -3.252375E38F, 1.0048595E38F, 2.9091644E38F, 2.639998E38F, 1.5434589E37F, -2.9843318E38F, 4.642797E37F}, 0) ;
        p139.target_system_SET((char)255) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)171);
            assert(pack.time_usec_GET() == 7103760816997080666L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.7895374E38F, -3.0511723E38F, 1.3807511E38F, -5.3131084E36F, 3.268229E38F, -5.7611126E36F, -2.2514245E38F, 3.46177E37F}));
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)171) ;
        p140.time_usec_SET(7103760816997080666L) ;
        p140.controls_SET(new float[] {2.7895374E38F, -3.0511723E38F, 1.3807511E38F, -5.3131084E36F, 3.268229E38F, -5.7611126E36F, -2.2514245E38F, 3.46177E37F}, 0) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_terrain_GET() == 2.384477E38F);
            assert(pack.bottom_clearance_GET() == 7.031831E37F);
            assert(pack.altitude_monotonic_GET() == 2.5915705E38F);
            assert(pack.altitude_amsl_GET() == -1.8251755E38F);
            assert(pack.altitude_local_GET() == 1.4861532E38F);
            assert(pack.time_usec_GET() == 8551009755851260109L);
            assert(pack.altitude_relative_GET() == -2.8589016E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(1.4861532E38F) ;
        p141.bottom_clearance_SET(7.031831E37F) ;
        p141.altitude_relative_SET(-2.8589016E38F) ;
        p141.altitude_monotonic_SET(2.5915705E38F) ;
        p141.altitude_terrain_SET(2.384477E38F) ;
        p141.time_usec_SET(8551009755851260109L) ;
        p141.altitude_amsl_SET(-1.8251755E38F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)254, (char)59, (char)74, (char)228, (char)124, (char)77, (char)113, (char)10, (char)112, (char)83, (char)151, (char)49, (char)198, (char)75, (char)252, (char)177, (char)179, (char)43, (char)207, (char)47, (char)92, (char)149, (char)162, (char)161, (char)59, (char)206, (char)110, (char)117, (char)83, (char)67, (char)107, (char)21, (char)70, (char)10, (char)111, (char)91, (char)84, (char)252, (char)121, (char)35, (char)56, (char)184, (char)50, (char)160, (char)42, (char)145, (char)100, (char)105, (char)234, (char)155, (char)227, (char)35, (char)94, (char)38, (char)38, (char)159, (char)214, (char)173, (char)114, (char)19, (char)13, (char)131, (char)186, (char)8, (char)255, (char)71, (char)21, (char)149, (char)31, (char)241, (char)75, (char)140, (char)31, (char)17, (char)33, (char)49, (char)137, (char)178, (char)115, (char)1, (char)65, (char)127, (char)180, (char)236, (char)112, (char)153, (char)58, (char)226, (char)20, (char)69, (char)106, (char)197, (char)161, (char)106, (char)32, (char)152, (char)228, (char)35, (char)252, (char)47, (char)107, (char)39, (char)72, (char)161, (char)103, (char)212, (char)17, (char)182, (char)116, (char)193, (char)45, (char)6, (char)23, (char)76, (char)85, (char)12, (char)99, (char)137, (char)35, (char)148}));
            assert(pack.uri_type_GET() == (char)65);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)102, (char)119, (char)141, (char)87, (char)209, (char)106, (char)204, (char)19, (char)34, (char)147, (char)160, (char)235, (char)198, (char)3, (char)5, (char)136, (char)92, (char)147, (char)61, (char)114, (char)195, (char)140, (char)252, (char)105, (char)222, (char)242, (char)196, (char)111, (char)86, (char)137, (char)151, (char)104, (char)21, (char)149, (char)161, (char)141, (char)197, (char)252, (char)89, (char)253, (char)159, (char)239, (char)196, (char)164, (char)249, (char)189, (char)169, (char)225, (char)237, (char)174, (char)235, (char)155, (char)57, (char)184, (char)114, (char)219, (char)100, (char)241, (char)200, (char)175, (char)47, (char)43, (char)16, (char)9, (char)16, (char)100, (char)169, (char)73, (char)149, (char)80, (char)196, (char)234, (char)250, (char)137, (char)141, (char)225, (char)198, (char)182, (char)175, (char)181, (char)29, (char)50, (char)208, (char)211, (char)125, (char)92, (char)205, (char)67, (char)178, (char)176, (char)102, (char)62, (char)207, (char)83, (char)183, (char)181, (char)218, (char)189, (char)222, (char)147, (char)68, (char)44, (char)33, (char)95, (char)176, (char)223, (char)238, (char)155, (char)116, (char)132, (char)252, (char)64, (char)4, (char)179, (char)224, (char)120, (char)204, (char)62, (char)149, (char)1}));
            assert(pack.transfer_type_GET() == (char)225);
            assert(pack.request_id_GET() == (char)86);
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)86) ;
        p142.storage_SET(new char[] {(char)102, (char)119, (char)141, (char)87, (char)209, (char)106, (char)204, (char)19, (char)34, (char)147, (char)160, (char)235, (char)198, (char)3, (char)5, (char)136, (char)92, (char)147, (char)61, (char)114, (char)195, (char)140, (char)252, (char)105, (char)222, (char)242, (char)196, (char)111, (char)86, (char)137, (char)151, (char)104, (char)21, (char)149, (char)161, (char)141, (char)197, (char)252, (char)89, (char)253, (char)159, (char)239, (char)196, (char)164, (char)249, (char)189, (char)169, (char)225, (char)237, (char)174, (char)235, (char)155, (char)57, (char)184, (char)114, (char)219, (char)100, (char)241, (char)200, (char)175, (char)47, (char)43, (char)16, (char)9, (char)16, (char)100, (char)169, (char)73, (char)149, (char)80, (char)196, (char)234, (char)250, (char)137, (char)141, (char)225, (char)198, (char)182, (char)175, (char)181, (char)29, (char)50, (char)208, (char)211, (char)125, (char)92, (char)205, (char)67, (char)178, (char)176, (char)102, (char)62, (char)207, (char)83, (char)183, (char)181, (char)218, (char)189, (char)222, (char)147, (char)68, (char)44, (char)33, (char)95, (char)176, (char)223, (char)238, (char)155, (char)116, (char)132, (char)252, (char)64, (char)4, (char)179, (char)224, (char)120, (char)204, (char)62, (char)149, (char)1}, 0) ;
        p142.transfer_type_SET((char)225) ;
        p142.uri_SET(new char[] {(char)254, (char)59, (char)74, (char)228, (char)124, (char)77, (char)113, (char)10, (char)112, (char)83, (char)151, (char)49, (char)198, (char)75, (char)252, (char)177, (char)179, (char)43, (char)207, (char)47, (char)92, (char)149, (char)162, (char)161, (char)59, (char)206, (char)110, (char)117, (char)83, (char)67, (char)107, (char)21, (char)70, (char)10, (char)111, (char)91, (char)84, (char)252, (char)121, (char)35, (char)56, (char)184, (char)50, (char)160, (char)42, (char)145, (char)100, (char)105, (char)234, (char)155, (char)227, (char)35, (char)94, (char)38, (char)38, (char)159, (char)214, (char)173, (char)114, (char)19, (char)13, (char)131, (char)186, (char)8, (char)255, (char)71, (char)21, (char)149, (char)31, (char)241, (char)75, (char)140, (char)31, (char)17, (char)33, (char)49, (char)137, (char)178, (char)115, (char)1, (char)65, (char)127, (char)180, (char)236, (char)112, (char)153, (char)58, (char)226, (char)20, (char)69, (char)106, (char)197, (char)161, (char)106, (char)32, (char)152, (char)228, (char)35, (char)252, (char)47, (char)107, (char)39, (char)72, (char)161, (char)103, (char)212, (char)17, (char)182, (char)116, (char)193, (char)45, (char)6, (char)23, (char)76, (char)85, (char)12, (char)99, (char)137, (char)35, (char)148}, 0) ;
        p142.uri_type_SET((char)65) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)20931);
            assert(pack.press_abs_GET() == -1.315634E38F);
            assert(pack.press_diff_GET() == 1.3179779E38F);
            assert(pack.time_boot_ms_GET() == 4149913909L);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(-1.315634E38F) ;
        p143.press_diff_SET(1.3179779E38F) ;
        p143.temperature_SET((short)20931) ;
        p143.time_boot_ms_SET(4149913909L) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-6.7594375E37F, -1.612182E38F, 3.5210324E37F}));
            assert(pack.custom_state_GET() == 4193973349471288037L);
            assert(pack.lat_GET() == -809004845);
            assert(pack.timestamp_GET() == 568107815624555524L);
            assert(pack.est_capabilities_GET() == (char)162);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-2.2941403E38F, 1.7894403E38F, -1.8470677E37F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-2.6119594E38F, 8.885831E37F, 2.5107848E38F}));
            assert(pack.alt_GET() == 1.6785124E38F);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.2766157E38F, 2.3815445E38F, -3.971515E36F, -6.158537E37F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-2.1007325E38F, -1.1360904E38F, -1.5416667E38F}));
            assert(pack.lon_GET() == 1695058180);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(568107815624555524L) ;
        p144.attitude_q_SET(new float[] {-2.2766157E38F, 2.3815445E38F, -3.971515E36F, -6.158537E37F}, 0) ;
        p144.est_capabilities_SET((char)162) ;
        p144.alt_SET(1.6785124E38F) ;
        p144.rates_SET(new float[] {-6.7594375E37F, -1.612182E38F, 3.5210324E37F}, 0) ;
        p144.vel_SET(new float[] {-2.2941403E38F, 1.7894403E38F, -1.8470677E37F}, 0) ;
        p144.lat_SET(-809004845) ;
        p144.custom_state_SET(4193973349471288037L) ;
        p144.position_cov_SET(new float[] {-2.1007325E38F, -1.1360904E38F, -1.5416667E38F}, 0) ;
        p144.lon_SET(1695058180) ;
        p144.acc_SET(new float[] {-2.6119594E38F, 8.885831E37F, 2.5107848E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.x_pos_GET() == 8.3853916E37F);
            assert(pack.z_pos_GET() == 2.0911742E38F);
            assert(pack.z_vel_GET() == -1.0334228E38F);
            assert(pack.x_acc_GET() == -4.960675E37F);
            assert(pack.airspeed_GET() == 8.636189E35F);
            assert(pack.roll_rate_GET() == -1.7555738E38F);
            assert(pack.x_vel_GET() == 1.6759987E38F);
            assert(pack.y_vel_GET() == -1.7788594E38F);
            assert(pack.z_acc_GET() == -1.6353534E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.7894994E38F, 1.5386623E38F, -2.6352868E38F}));
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {8.455687E37F, -3.087753E38F, -2.8377655E38F}));
            assert(pack.time_usec_GET() == 5158774581751672824L);
            assert(pack.pitch_rate_GET() == -1.8678053E38F);
            assert(pack.y_acc_GET() == -7.4541696E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.8280448E37F, 2.0514004E38F, 2.5892678E38F, 2.7468118E38F}));
            assert(pack.yaw_rate_GET() == 2.4626772E38F);
            assert(pack.y_pos_GET() == -2.8920327E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(-1.7788594E38F) ;
        p146.z_acc_SET(-1.6353534E38F) ;
        p146.time_usec_SET(5158774581751672824L) ;
        p146.z_pos_SET(2.0911742E38F) ;
        p146.airspeed_SET(8.636189E35F) ;
        p146.x_vel_SET(1.6759987E38F) ;
        p146.z_vel_SET(-1.0334228E38F) ;
        p146.x_acc_SET(-4.960675E37F) ;
        p146.vel_variance_SET(new float[] {-1.7894994E38F, 1.5386623E38F, -2.6352868E38F}, 0) ;
        p146.roll_rate_SET(-1.7555738E38F) ;
        p146.y_acc_SET(-7.4541696E36F) ;
        p146.pitch_rate_SET(-1.8678053E38F) ;
        p146.q_SET(new float[] {-2.8280448E37F, 2.0514004E38F, 2.5892678E38F, 2.7468118E38F}, 0) ;
        p146.yaw_rate_SET(2.4626772E38F) ;
        p146.x_pos_SET(8.3853916E37F) ;
        p146.pos_variance_SET(new float[] {8.455687E37F, -3.087753E38F, -2.8377655E38F}, 0) ;
        p146.y_pos_SET(-2.8920327E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == 189157933);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)15538, (char)604, (char)3680, (char)19099, (char)25345, (char)57857, (char)10357, (char)60066, (char)40777, (char)8977}));
            assert(pack.current_battery_GET() == (short)20650);
            assert(pack.temperature_GET() == (short) -3445);
            assert(pack.energy_consumed_GET() == 191540516);
            assert(pack.id_GET() == (char)97);
            assert(pack.battery_remaining_GET() == (byte)47);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.battery_remaining_SET((byte)47) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH) ;
        p147.energy_consumed_SET(191540516) ;
        p147.temperature_SET((short) -3445) ;
        p147.current_consumed_SET(189157933) ;
        p147.voltages_SET(new char[] {(char)15538, (char)604, (char)3680, (char)19099, (char)25345, (char)57857, (char)10357, (char)60066, (char)40777, (char)8977}, 0) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.current_battery_SET((short)20650) ;
        p147.id_SET((char)97) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 2311212431L);
            assert(pack.vendor_id_GET() == (char)64714);
            assert(pack.os_sw_version_GET() == 2781254603L);
            assert(pack.board_version_GET() == 623385598L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)185, (char)206, (char)58, (char)51, (char)11, (char)136, (char)170, (char)119}));
            assert(pack.uid_GET() == 1114253058313501449L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)41, (char)251, (char)152, (char)183, (char)68, (char)137, (char)209, (char)189}));
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN);
            assert(pack.middleware_sw_version_GET() == 666347641L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)29, (char)12, (char)228, (char)141, (char)48, (char)67, (char)174, (char)105, (char)97, (char)237, (char)77, (char)164, (char)33, (char)132, (char)239, (char)15, (char)44, (char)226}));
            assert(pack.product_id_GET() == (char)62749);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)31, (char)42, (char)222, (char)61, (char)177, (char)214, (char)95, (char)138}));
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.product_id_SET((char)62749) ;
        p148.flight_custom_version_SET(new char[] {(char)185, (char)206, (char)58, (char)51, (char)11, (char)136, (char)170, (char)119}, 0) ;
        p148.flight_sw_version_SET(2311212431L) ;
        p148.uid2_SET(new char[] {(char)29, (char)12, (char)228, (char)141, (char)48, (char)67, (char)174, (char)105, (char)97, (char)237, (char)77, (char)164, (char)33, (char)132, (char)239, (char)15, (char)44, (char)226}, 0, PH) ;
        p148.os_sw_version_SET(2781254603L) ;
        p148.vendor_id_SET((char)64714) ;
        p148.middleware_custom_version_SET(new char[] {(char)31, (char)42, (char)222, (char)61, (char)177, (char)214, (char)95, (char)138}, 0) ;
        p148.middleware_sw_version_SET(666347641L) ;
        p148.board_version_SET(623385598L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN) ;
        p148.uid_SET(1114253058313501449L) ;
        p148.os_custom_version_SET(new char[] {(char)41, (char)251, (char)152, (char)183, (char)68, (char)137, (char)209, (char)189}, 0) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_num_GET() == (char)70);
            assert(pack.size_x_GET() == 6.3298297E37F);
            assert(pack.time_usec_GET() == 7686315763138259532L);
            assert(pack.y_TRY(ph) == -1.1844684E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.909455E38F, 1.1815714E38F, 3.030259E38F, -4.7699456E37F}));
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_x_GET() == -1.0482028E38F);
            assert(pack.size_y_GET() == 4.646369E37F);
            assert(pack.distance_GET() == -7.931368E36F);
            assert(pack.z_TRY(ph) == -3.272424E38F);
            assert(pack.angle_y_GET() == -3.3076314E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.x_TRY(ph) == 1.9396703E38F);
            assert(pack.position_valid_TRY(ph) == (char)233);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.q_SET(new float[] {-1.909455E38F, 1.1815714E38F, 3.030259E38F, -4.7699456E37F}, 0, PH) ;
        p149.size_y_SET(4.646369E37F) ;
        p149.distance_SET(-7.931368E36F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p149.position_valid_SET((char)233, PH) ;
        p149.time_usec_SET(7686315763138259532L) ;
        p149.angle_x_SET(-1.0482028E38F) ;
        p149.angle_y_SET(-3.3076314E38F) ;
        p149.z_SET(-3.272424E38F, PH) ;
        p149.x_SET(1.9396703E38F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.target_num_SET((char)70) ;
        p149.y_SET(-1.1844684E38F, PH) ;
        p149.size_x_SET(6.3298297E37F) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AQ_TELEMETRY_F.add((src, ph, pack) ->
        {
            assert(pack.value1_GET() == -8.6134044E36F);
            assert(pack.value10_GET() == -6.43723E36F);
            assert(pack.value5_GET() == 5.783572E37F);
            assert(pack.value15_GET() == 1.6855868E38F);
            assert(pack.value11_GET() == 2.4819218E38F);
            assert(pack.value9_GET() == -2.0830838E38F);
            assert(pack.value2_GET() == -4.6864247E37F);
            assert(pack.value13_GET() == -2.2898824E38F);
            assert(pack.value16_GET() == -1.423204E38F);
            assert(pack.value14_GET() == 3.309204E38F);
            assert(pack.value6_GET() == -2.5736296E37F);
            assert(pack.value17_GET() == -1.4697421E38F);
            assert(pack.value19_GET() == 1.421211E38F);
            assert(pack.value18_GET() == -2.6573181E38F);
            assert(pack.value4_GET() == 2.4125302E38F);
            assert(pack.value3_GET() == 2.682446E38F);
            assert(pack.value8_GET() == -3.7368922E37F);
            assert(pack.Index_GET() == (char)2900);
            assert(pack.value7_GET() == 2.5468776E38F);
            assert(pack.value20_GET() == 3.235678E38F);
            assert(pack.value12_GET() == -6.4268445E37F);
        });
        DemoDevice.AQ_TELEMETRY_F p150 = LoopBackDemoChannel.new_AQ_TELEMETRY_F();
        PH.setPack(p150);
        p150.value12_SET(-6.4268445E37F) ;
        p150.value9_SET(-2.0830838E38F) ;
        p150.value1_SET(-8.6134044E36F) ;
        p150.value8_SET(-3.7368922E37F) ;
        p150.value15_SET(1.6855868E38F) ;
        p150.value20_SET(3.235678E38F) ;
        p150.value10_SET(-6.43723E36F) ;
        p150.value4_SET(2.4125302E38F) ;
        p150.value13_SET(-2.2898824E38F) ;
        p150.value6_SET(-2.5736296E37F) ;
        p150.value2_SET(-4.6864247E37F) ;
        p150.Index_SET((char)2900) ;
        p150.value3_SET(2.682446E38F) ;
        p150.value7_SET(2.5468776E38F) ;
        p150.value5_SET(5.783572E37F) ;
        p150.value19_SET(1.421211E38F) ;
        p150.value16_SET(-1.423204E38F) ;
        p150.value14_SET(3.309204E38F) ;
        p150.value11_SET(2.4819218E38F) ;
        p150.value18_SET(-2.6573181E38F) ;
        p150.value17_SET(-1.4697421E38F) ;
        LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AQ_ESC_TELEMETRY.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)130);
            assert(Arrays.equals(pack.data_version_GET(),  new char[] {(char)241, (char)155, (char)0, (char)77}));
            assert(pack.num_motors_GET() == (char)45);
            assert(Arrays.equals(pack.data1_GET(),  new long[] {4264250414L, 934377034L, 2701693111L, 3954399022L}));
            assert(pack.time_boot_ms_GET() == 479312499L);
            assert(Arrays.equals(pack.escid_GET(),  new char[] {(char)243, (char)165, (char)73, (char)57}));
            assert(pack.num_in_seq_GET() == (char)85);
            assert(Arrays.equals(pack.data0_GET(),  new long[] {258504628L, 776762L, 573988268L, 1640237032L}));
            assert(Arrays.equals(pack.status_age_GET(),  new char[] {(char)16308, (char)45711, (char)24869, (char)56396}));
        });
        DemoDevice.AQ_ESC_TELEMETRY p152 = LoopBackDemoChannel.new_AQ_ESC_TELEMETRY();
        PH.setPack(p152);
        p152.time_boot_ms_SET(479312499L) ;
        p152.num_in_seq_SET((char)85) ;
        p152.escid_SET(new char[] {(char)243, (char)165, (char)73, (char)57}, 0) ;
        p152.num_motors_SET((char)45) ;
        p152.seq_SET((char)130) ;
        p152.status_age_SET(new char[] {(char)16308, (char)45711, (char)24869, (char)56396}, 0) ;
        p152.data1_SET(new long[] {4264250414L, 934377034L, 2701693111L, 3954399022L}, 0) ;
        p152.data0_SET(new long[] {258504628L, 776762L, 573988268L, 1640237032L}, 0) ;
        p152.data_version_SET(new char[] {(char)241, (char)155, (char)0, (char)77}, 0) ;
        LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pos_vert_accuracy_GET() == -9.535433E37F);
            assert(pack.vel_ratio_GET() == 3.3645522E38F);
            assert(pack.mag_ratio_GET() == -2.8880592E38F);
            assert(pack.pos_vert_ratio_GET() == 6.5004104E37F);
            assert(pack.hagl_ratio_GET() == -1.4990425E38F);
            assert(pack.pos_horiz_ratio_GET() == 9.708695E37F);
            assert(pack.time_usec_GET() == 3930820613629717289L);
            assert(pack.tas_ratio_GET() == 9.138663E37F);
            assert(pack.pos_horiz_accuracy_GET() == 2.358581E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(6.5004104E37F) ;
        p230.pos_vert_accuracy_SET(-9.535433E37F) ;
        p230.hagl_ratio_SET(-1.4990425E38F) ;
        p230.pos_horiz_ratio_SET(9.708695E37F) ;
        p230.tas_ratio_SET(9.138663E37F) ;
        p230.vel_ratio_SET(3.3645522E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE) ;
        p230.pos_horiz_accuracy_SET(2.358581E38F) ;
        p230.mag_ratio_SET(-2.8880592E38F) ;
        p230.time_usec_SET(3930820613629717289L) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == 2.5292796E37F);
            assert(pack.wind_x_GET() == 1.02528766E37F);
            assert(pack.var_horiz_GET() == -6.018451E37F);
            assert(pack.wind_y_GET() == -2.2475074E38F);
            assert(pack.horiz_accuracy_GET() == 1.9536465E38F);
            assert(pack.time_usec_GET() == 2610753833534068933L);
            assert(pack.vert_accuracy_GET() == 1.709525E38F);
            assert(pack.wind_z_GET() == -2.2648403E38F);
            assert(pack.var_vert_GET() == 2.9065338E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.var_horiz_SET(-6.018451E37F) ;
        p231.wind_alt_SET(2.5292796E37F) ;
        p231.vert_accuracy_SET(1.709525E38F) ;
        p231.var_vert_SET(2.9065338E38F) ;
        p231.wind_y_SET(-2.2475074E38F) ;
        p231.horiz_accuracy_SET(1.9536465E38F) ;
        p231.time_usec_SET(2610753833534068933L) ;
        p231.wind_x_SET(1.02528766E37F) ;
        p231.wind_z_SET(-2.2648403E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.ve_GET() == 2.023519E38F);
            assert(pack.time_week_GET() == (char)15908);
            assert(pack.time_week_ms_GET() == 914141188L);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
            assert(pack.lon_GET() == 1724747626);
            assert(pack.time_usec_GET() == 4790238888688789368L);
            assert(pack.gps_id_GET() == (char)173);
            assert(pack.hdop_GET() == 1.318492E38F);
            assert(pack.vert_accuracy_GET() == 2.5554104E38F);
            assert(pack.speed_accuracy_GET() == -2.6974266E38F);
            assert(pack.fix_type_GET() == (char)63);
            assert(pack.vdop_GET() == 3.2839773E38F);
            assert(pack.satellites_visible_GET() == (char)127);
            assert(pack.lat_GET() == 1072703578);
            assert(pack.alt_GET() == -2.6008714E38F);
            assert(pack.horiz_accuracy_GET() == 7.2298393E37F);
            assert(pack.vn_GET() == -3.3560618E38F);
            assert(pack.vd_GET() == -1.406133E38F);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.lon_SET(1724747626) ;
        p232.ve_SET(2.023519E38F) ;
        p232.speed_accuracy_SET(-2.6974266E38F) ;
        p232.time_usec_SET(4790238888688789368L) ;
        p232.time_week_ms_SET(914141188L) ;
        p232.gps_id_SET((char)173) ;
        p232.fix_type_SET((char)63) ;
        p232.hdop_SET(1.318492E38F) ;
        p232.lat_SET(1072703578) ;
        p232.horiz_accuracy_SET(7.2298393E37F) ;
        p232.vdop_SET(3.2839773E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP) ;
        p232.vn_SET(-3.3560618E38F) ;
        p232.time_week_SET((char)15908) ;
        p232.vert_accuracy_SET(2.5554104E38F) ;
        p232.alt_SET(-2.6008714E38F) ;
        p232.vd_SET(-1.406133E38F) ;
        p232.satellites_visible_SET((char)127) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)163);
            assert(pack.len_GET() == (char)18);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)170, (char)224, (char)162, (char)51, (char)231, (char)131, (char)225, (char)209, (char)19, (char)56, (char)201, (char)64, (char)213, (char)127, (char)209, (char)53, (char)106, (char)81, (char)80, (char)99, (char)203, (char)79, (char)85, (char)139, (char)210, (char)165, (char)230, (char)27, (char)154, (char)241, (char)217, (char)5, (char)64, (char)86, (char)232, (char)214, (char)252, (char)109, (char)205, (char)69, (char)60, (char)84, (char)56, (char)161, (char)122, (char)148, (char)124, (char)79, (char)65, (char)42, (char)109, (char)87, (char)79, (char)213, (char)237, (char)185, (char)221, (char)149, (char)59, (char)31, (char)35, (char)202, (char)126, (char)18, (char)96, (char)166, (char)161, (char)13, (char)108, (char)119, (char)149, (char)190, (char)29, (char)232, (char)235, (char)15, (char)121, (char)92, (char)61, (char)154, (char)231, (char)87, (char)184, (char)188, (char)15, (char)178, (char)29, (char)48, (char)2, (char)201, (char)7, (char)84, (char)159, (char)115, (char)186, (char)152, (char)33, (char)247, (char)158, (char)10, (char)149, (char)86, (char)54, (char)154, (char)173, (char)238, (char)255, (char)63, (char)37, (char)172, (char)33, (char)54, (char)173, (char)153, (char)153, (char)116, (char)129, (char)202, (char)98, (char)27, (char)115, (char)221, (char)186, (char)100, (char)67, (char)154, (char)69, (char)178, (char)176, (char)47, (char)147, (char)156, (char)116, (char)244, (char)89, (char)236, (char)209, (char)131, (char)86, (char)8, (char)34, (char)74, (char)180, (char)43, (char)135, (char)33, (char)82, (char)83, (char)55, (char)107, (char)40, (char)66, (char)10, (char)181, (char)216, (char)83, (char)51, (char)91, (char)248, (char)188, (char)100, (char)45, (char)50, (char)43, (char)147, (char)9, (char)15, (char)68, (char)107, (char)233, (char)192, (char)147, (char)100, (char)237, (char)217, (char)51, (char)125, (char)240, (char)163, (char)243}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)170, (char)224, (char)162, (char)51, (char)231, (char)131, (char)225, (char)209, (char)19, (char)56, (char)201, (char)64, (char)213, (char)127, (char)209, (char)53, (char)106, (char)81, (char)80, (char)99, (char)203, (char)79, (char)85, (char)139, (char)210, (char)165, (char)230, (char)27, (char)154, (char)241, (char)217, (char)5, (char)64, (char)86, (char)232, (char)214, (char)252, (char)109, (char)205, (char)69, (char)60, (char)84, (char)56, (char)161, (char)122, (char)148, (char)124, (char)79, (char)65, (char)42, (char)109, (char)87, (char)79, (char)213, (char)237, (char)185, (char)221, (char)149, (char)59, (char)31, (char)35, (char)202, (char)126, (char)18, (char)96, (char)166, (char)161, (char)13, (char)108, (char)119, (char)149, (char)190, (char)29, (char)232, (char)235, (char)15, (char)121, (char)92, (char)61, (char)154, (char)231, (char)87, (char)184, (char)188, (char)15, (char)178, (char)29, (char)48, (char)2, (char)201, (char)7, (char)84, (char)159, (char)115, (char)186, (char)152, (char)33, (char)247, (char)158, (char)10, (char)149, (char)86, (char)54, (char)154, (char)173, (char)238, (char)255, (char)63, (char)37, (char)172, (char)33, (char)54, (char)173, (char)153, (char)153, (char)116, (char)129, (char)202, (char)98, (char)27, (char)115, (char)221, (char)186, (char)100, (char)67, (char)154, (char)69, (char)178, (char)176, (char)47, (char)147, (char)156, (char)116, (char)244, (char)89, (char)236, (char)209, (char)131, (char)86, (char)8, (char)34, (char)74, (char)180, (char)43, (char)135, (char)33, (char)82, (char)83, (char)55, (char)107, (char)40, (char)66, (char)10, (char)181, (char)216, (char)83, (char)51, (char)91, (char)248, (char)188, (char)100, (char)45, (char)50, (char)43, (char)147, (char)9, (char)15, (char)68, (char)107, (char)233, (char)192, (char)147, (char)100, (char)237, (char)217, (char)51, (char)125, (char)240, (char)163, (char)243}, 0) ;
        p233.len_SET((char)18) ;
        p233.flags_SET((char)163) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_distance_GET() == (char)9839);
            assert(pack.pitch_GET() == (short) -7827);
            assert(pack.temperature_air_GET() == (byte)31);
            assert(pack.altitude_amsl_GET() == (short) -6518);
            assert(pack.battery_remaining_GET() == (char)32);
            assert(pack.throttle_GET() == (byte)102);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.failsafe_GET() == (char)240);
            assert(pack.longitude_GET() == -917754395);
            assert(pack.latitude_GET() == -1242820024);
            assert(pack.custom_mode_GET() == 4161949877L);
            assert(pack.roll_GET() == (short) -14918);
            assert(pack.heading_sp_GET() == (short) -12011);
            assert(pack.temperature_GET() == (byte) - 11);
            assert(pack.airspeed_GET() == (char)230);
            assert(pack.wp_num_GET() == (char)82);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.climb_rate_GET() == (byte)110);
            assert(pack.groundspeed_GET() == (char)64);
            assert(pack.altitude_sp_GET() == (short) -6438);
            assert(pack.gps_nsat_GET() == (char)223);
            assert(pack.heading_GET() == (char)214);
            assert(pack.airspeed_sp_GET() == (char)195);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.roll_SET((short) -14918) ;
        p234.altitude_sp_SET((short) -6438) ;
        p234.pitch_SET((short) -7827) ;
        p234.throttle_SET((byte)102) ;
        p234.custom_mode_SET(4161949877L) ;
        p234.climb_rate_SET((byte)110) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p234.wp_distance_SET((char)9839) ;
        p234.altitude_amsl_SET((short) -6518) ;
        p234.groundspeed_SET((char)64) ;
        p234.battery_remaining_SET((char)32) ;
        p234.latitude_SET(-1242820024) ;
        p234.airspeed_sp_SET((char)195) ;
        p234.longitude_SET(-917754395) ;
        p234.temperature_air_SET((byte)31) ;
        p234.failsafe_SET((char)240) ;
        p234.wp_num_SET((char)82) ;
        p234.airspeed_SET((char)230) ;
        p234.gps_nsat_SET((char)223) ;
        p234.temperature_SET((byte) - 11) ;
        p234.heading_SET((char)214) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.heading_sp_SET((short) -12011) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_y_GET() == -4.5625E37F);
            assert(pack.time_usec_GET() == 407291782299912073L);
            assert(pack.clipping_2_GET() == 3431615051L);
            assert(pack.clipping_0_GET() == 4288034336L);
            assert(pack.vibration_z_GET() == -1.599447E38F);
            assert(pack.clipping_1_GET() == 1481838869L);
            assert(pack.vibration_x_GET() == 7.8994256E37F);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(1481838869L) ;
        p241.clipping_0_SET(4288034336L) ;
        p241.vibration_x_SET(7.8994256E37F) ;
        p241.vibration_y_SET(-4.5625E37F) ;
        p241.time_usec_SET(407291782299912073L) ;
        p241.clipping_2_SET(3431615051L) ;
        p241.vibration_z_SET(-1.599447E38F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1424898485);
            assert(pack.y_GET() == -3.0093064E37F);
            assert(pack.z_GET() == 2.0132312E38F);
            assert(pack.approach_y_GET() == 2.4722588E38F);
            assert(pack.time_usec_TRY(ph) == 4415859929596514765L);
            assert(pack.x_GET() == 2.058214E38F);
            assert(pack.altitude_GET() == -70741588);
            assert(pack.approach_x_GET() == 1.913373E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.9980384E38F, -1.3217816E38F, 1.4018683E37F, -9.327487E37F}));
            assert(pack.approach_z_GET() == -2.4203828E37F);
            assert(pack.latitude_GET() == -1190109132);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.altitude_SET(-70741588) ;
        p242.y_SET(-3.0093064E37F) ;
        p242.latitude_SET(-1190109132) ;
        p242.time_usec_SET(4415859929596514765L, PH) ;
        p242.q_SET(new float[] {-1.9980384E38F, -1.3217816E38F, 1.4018683E37F, -9.327487E37F}, 0) ;
        p242.approach_z_SET(-2.4203828E37F) ;
        p242.longitude_SET(1424898485) ;
        p242.x_SET(2.058214E38F) ;
        p242.approach_y_SET(2.4722588E38F) ;
        p242.approach_x_SET(1.913373E38F) ;
        p242.z_SET(2.0132312E38F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)190);
            assert(pack.latitude_GET() == 1400414832);
            assert(pack.altitude_GET() == 272912145);
            assert(pack.y_GET() == -1.9211794E38F);
            assert(pack.approach_y_GET() == -8.02436E37F);
            assert(pack.approach_x_GET() == -1.9297792E38F);
            assert(pack.time_usec_TRY(ph) == 5051250831912816237L);
            assert(pack.z_GET() == -2.2821108E38F);
            assert(pack.x_GET() == -1.3146916E38F);
            assert(pack.approach_z_GET() == 4.1899384E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.080833E37F, -3.0413092E38F, 3.1752475E38F, -4.705233E37F}));
            assert(pack.longitude_GET() == -323178942);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.longitude_SET(-323178942) ;
        p243.altitude_SET(272912145) ;
        p243.x_SET(-1.3146916E38F) ;
        p243.latitude_SET(1400414832) ;
        p243.approach_z_SET(4.1899384E37F) ;
        p243.z_SET(-2.2821108E38F) ;
        p243.target_system_SET((char)190) ;
        p243.q_SET(new float[] {-7.080833E37F, -3.0413092E38F, 3.1752475E38F, -4.705233E37F}, 0) ;
        p243.time_usec_SET(5051250831912816237L, PH) ;
        p243.y_SET(-1.9211794E38F) ;
        p243.approach_x_SET(-1.9297792E38F) ;
        p243.approach_y_SET(-8.02436E37F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)43399);
            assert(pack.interval_us_GET() == -25599812);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)43399) ;
        p244.interval_us_SET(-25599812) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.squawk_GET() == (char)63043);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("vsbmeNqav"));
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            assert(pack.ICAO_address_GET() == 2132352218L);
            assert(pack.lon_GET() == -889492374);
            assert(pack.altitude_GET() == 56224809);
            assert(pack.ver_velocity_GET() == (short)6202);
            assert(pack.hor_velocity_GET() == (char)58701);
            assert(pack.tslc_GET() == (char)180);
            assert(pack.heading_GET() == (char)10310);
            assert(pack.lat_GET() == -930703534);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY) ;
        p246.ICAO_address_SET(2132352218L) ;
        p246.ver_velocity_SET((short)6202) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.tslc_SET((char)180) ;
        p246.lon_SET(-889492374) ;
        p246.squawk_SET((char)63043) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV) ;
        p246.callsign_SET("vsbmeNqav", PH) ;
        p246.heading_SET((char)10310) ;
        p246.lat_SET(-930703534) ;
        p246.hor_velocity_SET((char)58701) ;
        p246.altitude_SET(56224809) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.horizontal_minimum_delta_GET() == 2.877905E37F);
            assert(pack.time_to_minimum_delta_GET() == 2.7207999E38F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.altitude_minimum_delta_GET() == -2.2666787E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.id_GET() == 3809387895L);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        p247.time_to_minimum_delta_SET(2.7207999E38F) ;
        p247.altitude_minimum_delta_SET(-2.2666787E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.id_SET(3809387895L) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.horizontal_minimum_delta_SET(2.877905E37F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)208);
            assert(pack.message_type_GET() == (char)32606);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.target_system_GET() == (char)46);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)147, (char)82, (char)1, (char)151, (char)230, (char)130, (char)105, (char)5, (char)34, (char)86, (char)74, (char)147, (char)102, (char)150, (char)121, (char)65, (char)146, (char)174, (char)140, (char)139, (char)30, (char)39, (char)155, (char)119, (char)155, (char)58, (char)51, (char)206, (char)212, (char)167, (char)55, (char)86, (char)237, (char)52, (char)55, (char)208, (char)252, (char)245, (char)22, (char)30, (char)57, (char)180, (char)92, (char)231, (char)186, (char)161, (char)63, (char)87, (char)131, (char)146, (char)123, (char)190, (char)228, (char)74, (char)215, (char)93, (char)243, (char)46, (char)111, (char)197, (char)199, (char)112, (char)122, (char)193, (char)42, (char)222, (char)197, (char)134, (char)209, (char)175, (char)41, (char)243, (char)57, (char)218, (char)185, (char)113, (char)113, (char)115, (char)71, (char)248, (char)177, (char)60, (char)175, (char)225, (char)30, (char)232, (char)138, (char)8, (char)252, (char)212, (char)99, (char)160, (char)99, (char)240, (char)214, (char)80, (char)57, (char)14, (char)119, (char)12, (char)198, (char)107, (char)61, (char)93, (char)80, (char)143, (char)208, (char)11, (char)69, (char)41, (char)161, (char)117, (char)61, (char)32, (char)200, (char)253, (char)201, (char)133, (char)96, (char)115, (char)141, (char)35, (char)204, (char)127, (char)235, (char)241, (char)14, (char)1, (char)94, (char)67, (char)249, (char)113, (char)19, (char)200, (char)159, (char)164, (char)205, (char)118, (char)239, (char)82, (char)139, (char)131, (char)21, (char)72, (char)1, (char)130, (char)1, (char)97, (char)30, (char)238, (char)195, (char)169, (char)192, (char)131, (char)87, (char)57, (char)23, (char)121, (char)250, (char)149, (char)248, (char)116, (char)127, (char)8, (char)184, (char)18, (char)197, (char)178, (char)220, (char)61, (char)161, (char)187, (char)5, (char)70, (char)160, (char)171, (char)186, (char)195, (char)168, (char)131, (char)132, (char)134, (char)255, (char)1, (char)27, (char)41, (char)28, (char)248, (char)21, (char)18, (char)132, (char)49, (char)44, (char)255, (char)195, (char)80, (char)251, (char)43, (char)231, (char)58, (char)247, (char)2, (char)159, (char)126, (char)134, (char)123, (char)39, (char)122, (char)44, (char)191, (char)165, (char)188, (char)24, (char)82, (char)76, (char)233, (char)155, (char)16, (char)140, (char)98, (char)65, (char)239, (char)34, (char)122, (char)158, (char)25, (char)227, (char)87, (char)226, (char)128, (char)106, (char)127, (char)158, (char)87, (char)2, (char)97, (char)38, (char)111, (char)24, (char)119, (char)134, (char)240, (char)157, (char)28, (char)26, (char)105, (char)99, (char)2, (char)41}));
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)208) ;
        p248.payload_SET(new char[] {(char)147, (char)82, (char)1, (char)151, (char)230, (char)130, (char)105, (char)5, (char)34, (char)86, (char)74, (char)147, (char)102, (char)150, (char)121, (char)65, (char)146, (char)174, (char)140, (char)139, (char)30, (char)39, (char)155, (char)119, (char)155, (char)58, (char)51, (char)206, (char)212, (char)167, (char)55, (char)86, (char)237, (char)52, (char)55, (char)208, (char)252, (char)245, (char)22, (char)30, (char)57, (char)180, (char)92, (char)231, (char)186, (char)161, (char)63, (char)87, (char)131, (char)146, (char)123, (char)190, (char)228, (char)74, (char)215, (char)93, (char)243, (char)46, (char)111, (char)197, (char)199, (char)112, (char)122, (char)193, (char)42, (char)222, (char)197, (char)134, (char)209, (char)175, (char)41, (char)243, (char)57, (char)218, (char)185, (char)113, (char)113, (char)115, (char)71, (char)248, (char)177, (char)60, (char)175, (char)225, (char)30, (char)232, (char)138, (char)8, (char)252, (char)212, (char)99, (char)160, (char)99, (char)240, (char)214, (char)80, (char)57, (char)14, (char)119, (char)12, (char)198, (char)107, (char)61, (char)93, (char)80, (char)143, (char)208, (char)11, (char)69, (char)41, (char)161, (char)117, (char)61, (char)32, (char)200, (char)253, (char)201, (char)133, (char)96, (char)115, (char)141, (char)35, (char)204, (char)127, (char)235, (char)241, (char)14, (char)1, (char)94, (char)67, (char)249, (char)113, (char)19, (char)200, (char)159, (char)164, (char)205, (char)118, (char)239, (char)82, (char)139, (char)131, (char)21, (char)72, (char)1, (char)130, (char)1, (char)97, (char)30, (char)238, (char)195, (char)169, (char)192, (char)131, (char)87, (char)57, (char)23, (char)121, (char)250, (char)149, (char)248, (char)116, (char)127, (char)8, (char)184, (char)18, (char)197, (char)178, (char)220, (char)61, (char)161, (char)187, (char)5, (char)70, (char)160, (char)171, (char)186, (char)195, (char)168, (char)131, (char)132, (char)134, (char)255, (char)1, (char)27, (char)41, (char)28, (char)248, (char)21, (char)18, (char)132, (char)49, (char)44, (char)255, (char)195, (char)80, (char)251, (char)43, (char)231, (char)58, (char)247, (char)2, (char)159, (char)126, (char)134, (char)123, (char)39, (char)122, (char)44, (char)191, (char)165, (char)188, (char)24, (char)82, (char)76, (char)233, (char)155, (char)16, (char)140, (char)98, (char)65, (char)239, (char)34, (char)122, (char)158, (char)25, (char)227, (char)87, (char)226, (char)128, (char)106, (char)127, (char)158, (char)87, (char)2, (char)97, (char)38, (char)111, (char)24, (char)119, (char)134, (char)240, (char)157, (char)28, (char)26, (char)105, (char)99, (char)2, (char)41}, 0) ;
        p248.message_type_SET((char)32606) ;
        p248.target_component_SET((char)203) ;
        p248.target_system_SET((char)46) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)40131);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)77, (byte) - 77, (byte)122, (byte) - 4, (byte) - 112, (byte)29, (byte)30, (byte) - 74, (byte)32, (byte) - 28, (byte) - 105, (byte)39, (byte) - 30, (byte) - 84, (byte)94, (byte) - 76, (byte)121, (byte)1, (byte)115, (byte) - 42, (byte)10, (byte)120, (byte)90, (byte) - 113, (byte)89, (byte) - 83, (byte) - 58, (byte) - 67, (byte) - 79, (byte) - 48, (byte)88, (byte) - 76}));
            assert(pack.ver_GET() == (char)167);
            assert(pack.type_GET() == (char)141);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)167) ;
        p249.type_SET((char)141) ;
        p249.value_SET(new byte[] {(byte)77, (byte) - 77, (byte)122, (byte) - 4, (byte) - 112, (byte)29, (byte)30, (byte) - 74, (byte)32, (byte) - 28, (byte) - 105, (byte)39, (byte) - 30, (byte) - 84, (byte)94, (byte) - 76, (byte)121, (byte)1, (byte)115, (byte) - 42, (byte)10, (byte)120, (byte)90, (byte) - 113, (byte)89, (byte) - 83, (byte) - 58, (byte) - 67, (byte) - 79, (byte) - 48, (byte)88, (byte) - 76}, 0) ;
        p249.address_SET((char)40131) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -3.393305E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("xZUnglsn"));
            assert(pack.x_GET() == 2.2595858E38F);
            assert(pack.y_GET() == -2.8597412E37F);
            assert(pack.time_usec_GET() == 8295835769357721884L);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-3.393305E38F) ;
        p250.name_SET("xZUnglsn", PH) ;
        p250.x_SET(2.2595858E38F) ;
        p250.y_SET(-2.8597412E37F) ;
        p250.time_usec_SET(8295835769357721884L) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("zhljalpftq"));
            assert(pack.time_boot_ms_GET() == 1656215842L);
            assert(pack.value_GET() == -3.9544153E37F);
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("zhljalpftq", PH) ;
        p251.time_boot_ms_SET(1656215842L) ;
        p251.value_SET(-3.9544153E37F) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2101568327);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("ptsjgpem"));
            assert(pack.time_boot_ms_GET() == 2163399494L);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("ptsjgpem", PH) ;
        p252.time_boot_ms_SET(2163399494L) ;
        p252.value_SET(-2101568327) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            assert(pack.text_LEN(ph) == 26);
            assert(pack.text_TRY(ph).equals("pjlrnvjnjwkxsyixhfyJxxjica"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY) ;
        p253.text_SET("pjlrnvjnjwkxsyixhfyJxxjica", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 861138793L);
            assert(pack.value_GET() == -1.2270048E38F);
            assert(pack.ind_GET() == (char)15);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)15) ;
        p254.value_SET(-1.2270048E38F) ;
        p254.time_boot_ms_SET(861138793L) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)236);
            assert(pack.initial_timestamp_GET() == 2369687471896004702L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)241, (char)226, (char)131, (char)92, (char)221, (char)247, (char)56, (char)242, (char)60, (char)201, (char)91, (char)214, (char)54, (char)31, (char)23, (char)134, (char)143, (char)71, (char)88, (char)91, (char)3, (char)111, (char)242, (char)116, (char)104, (char)253, (char)95, (char)70, (char)21, (char)86, (char)222, (char)188}));
            assert(pack.target_component_GET() == (char)25);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)236) ;
        p256.target_component_SET((char)25) ;
        p256.initial_timestamp_SET(2369687471896004702L) ;
        p256.secret_key_SET(new char[] {(char)241, (char)226, (char)131, (char)92, (char)221, (char)247, (char)56, (char)242, (char)60, (char)201, (char)91, (char)214, (char)54, (char)31, (char)23, (char)134, (char)143, (char)71, (char)88, (char)91, (char)3, (char)111, (char)242, (char)116, (char)104, (char)253, (char)95, (char)70, (char)21, (char)86, (char)222, (char)188}, 0) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 3450137164L);
            assert(pack.time_boot_ms_GET() == 2981332265L);
            assert(pack.state_GET() == (char)60);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2981332265L) ;
        p257.last_change_ms_SET(3450137164L) ;
        p257.state_SET((char)60) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.tune_LEN(ph) == 8);
            assert(pack.tune_TRY(ph).equals("sipivtrz"));
            assert(pack.target_component_GET() == (char)207);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)207) ;
        p258.target_system_SET((char)159) ;
        p258.tune_SET("sipivtrz", PH) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)19771);
            assert(pack.resolution_v_GET() == (char)23042);
            assert(pack.sensor_size_v_GET() == -2.7802356E38F);
            assert(pack.focal_length_GET() == 2.290361E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)224, (char)66, (char)138, (char)187, (char)200, (char)234, (char)179, (char)11, (char)225, (char)200, (char)56, (char)21, (char)159, (char)161, (char)51, (char)19, (char)247, (char)33, (char)98, (char)38, (char)65, (char)253, (char)236, (char)123, (char)12, (char)167, (char)234, (char)175, (char)220, (char)132, (char)0, (char)120}));
            assert(pack.cam_definition_version_GET() == (char)28705);
            assert(pack.lens_id_GET() == (char)254);
            assert(pack.sensor_size_h_GET() == 2.3957266E38F);
            assert(pack.cam_definition_uri_LEN(ph) == 59);
            assert(pack.cam_definition_uri_TRY(ph).equals("mkfowsoNnxwvxzGzqfuVhitfvttcpyHiroEdvAvgxqogrgwovcpncthmrnk"));
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            assert(pack.firmware_version_GET() == 1334069162L);
            assert(pack.time_boot_ms_GET() == 594672825L);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)104, (char)53, (char)181, (char)190, (char)183, (char)173, (char)201, (char)254, (char)176, (char)8, (char)26, (char)89, (char)201, (char)82, (char)123, (char)198, (char)97, (char)42, (char)103, (char)135, (char)38, (char)152, (char)177, (char)215, (char)212, (char)99, (char)228, (char)64, (char)5, (char)133, (char)98, (char)214}));
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_h_SET(2.3957266E38F) ;
        p259.cam_definition_uri_SET("mkfowsoNnxwvxzGzqfuVhitfvttcpyHiroEdvAvgxqogrgwovcpncthmrnk", PH) ;
        p259.model_name_SET(new char[] {(char)104, (char)53, (char)181, (char)190, (char)183, (char)173, (char)201, (char)254, (char)176, (char)8, (char)26, (char)89, (char)201, (char)82, (char)123, (char)198, (char)97, (char)42, (char)103, (char)135, (char)38, (char)152, (char)177, (char)215, (char)212, (char)99, (char)228, (char)64, (char)5, (char)133, (char)98, (char)214}, 0) ;
        p259.cam_definition_version_SET((char)28705) ;
        p259.focal_length_SET(2.290361E38F) ;
        p259.lens_id_SET((char)254) ;
        p259.resolution_h_SET((char)19771) ;
        p259.vendor_name_SET(new char[] {(char)224, (char)66, (char)138, (char)187, (char)200, (char)234, (char)179, (char)11, (char)225, (char)200, (char)56, (char)21, (char)159, (char)161, (char)51, (char)19, (char)247, (char)33, (char)98, (char)38, (char)65, (char)253, (char)236, (char)123, (char)12, (char)167, (char)234, (char)175, (char)220, (char)132, (char)0, (char)120}, 0) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO) ;
        p259.time_boot_ms_SET(594672825L) ;
        p259.resolution_v_SET((char)23042) ;
        p259.firmware_version_SET(1334069162L) ;
        p259.sensor_size_v_SET(-2.7802356E38F) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3536489740L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        p260.time_boot_ms_SET(3536489740L) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2094721289L);
            assert(pack.storage_count_GET() == (char)150);
            assert(pack.read_speed_GET() == -2.9793896E38F);
            assert(pack.used_capacity_GET() == -3.0525951E38F);
            assert(pack.storage_id_GET() == (char)246);
            assert(pack.write_speed_GET() == -9.0397356E36F);
            assert(pack.available_capacity_GET() == 1.4611514E38F);
            assert(pack.status_GET() == (char)218);
            assert(pack.total_capacity_GET() == -3.056327E38F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)218) ;
        p261.storage_count_SET((char)150) ;
        p261.used_capacity_SET(-3.0525951E38F) ;
        p261.read_speed_SET(-2.9793896E38F) ;
        p261.storage_id_SET((char)246) ;
        p261.time_boot_ms_SET(2094721289L) ;
        p261.write_speed_SET(-9.0397356E36F) ;
        p261.available_capacity_SET(1.4611514E38F) ;
        p261.total_capacity_SET(-3.056327E38F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)152);
            assert(pack.available_capacity_GET() == -1.2698367E38F);
            assert(pack.image_interval_GET() == -1.771719E38F);
            assert(pack.image_status_GET() == (char)194);
            assert(pack.recording_time_ms_GET() == 3321929365L);
            assert(pack.time_boot_ms_GET() == 3711283909L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.recording_time_ms_SET(3321929365L) ;
        p262.image_interval_SET(-1.771719E38F) ;
        p262.available_capacity_SET(-1.2698367E38F) ;
        p262.image_status_SET((char)194) ;
        p262.time_boot_ms_SET(3711283909L) ;
        p262.video_status_SET((char)152) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 705676413);
            assert(pack.file_url_LEN(ph) == 30);
            assert(pack.file_url_TRY(ph).equals("jbxXybcUfjifyhUxibxesdGepaYmmb"));
            assert(pack.time_boot_ms_GET() == 56774731L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.1509435E38F, 1.9496209E38F, -9.16288E37F, 2.1754483E38F}));
            assert(pack.alt_GET() == -161616874);
            assert(pack.image_index_GET() == -866906851);
            assert(pack.time_utc_GET() == 8813581817921855834L);
            assert(pack.capture_result_GET() == (byte) - 38);
            assert(pack.camera_id_GET() == (char)40);
            assert(pack.relative_alt_GET() == -233299740);
            assert(pack.lat_GET() == -630926349);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lon_SET(705676413) ;
        p263.alt_SET(-161616874) ;
        p263.image_index_SET(-866906851) ;
        p263.q_SET(new float[] {-3.1509435E38F, 1.9496209E38F, -9.16288E37F, 2.1754483E38F}, 0) ;
        p263.camera_id_SET((char)40) ;
        p263.file_url_SET("jbxXybcUfjifyhUxibxesdGepaYmmb", PH) ;
        p263.time_boot_ms_SET(56774731L) ;
        p263.relative_alt_SET(-233299740) ;
        p263.capture_result_SET((byte) - 38) ;
        p263.time_utc_SET(8813581817921855834L) ;
        p263.lat_SET(-630926349) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 5607371289695004624L);
            assert(pack.flight_uuid_GET() == 7424332670215028537L);
            assert(pack.time_boot_ms_GET() == 3095094707L);
            assert(pack.arming_time_utc_GET() == 8860326773108662639L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(5607371289695004624L) ;
        p264.flight_uuid_SET(7424332670215028537L) ;
        p264.time_boot_ms_SET(3095094707L) ;
        p264.arming_time_utc_SET(8860326773108662639L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 6.321328E37F);
            assert(pack.roll_GET() == 1.1661923E38F);
            assert(pack.yaw_GET() == -1.9414951E37F);
            assert(pack.time_boot_ms_GET() == 1772661672L);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(1.1661923E38F) ;
        p265.pitch_SET(6.321328E37F) ;
        p265.yaw_SET(-1.9414951E37F) ;
        p265.time_boot_ms_SET(1772661672L) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(pack.sequence_GET() == (char)62675);
            assert(pack.target_component_GET() == (char)180);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)242, (char)191, (char)218, (char)157, (char)90, (char)126, (char)237, (char)128, (char)120, (char)210, (char)130, (char)145, (char)132, (char)211, (char)24, (char)153, (char)242, (char)229, (char)162, (char)140, (char)249, (char)87, (char)233, (char)156, (char)206, (char)132, (char)111, (char)22, (char)253, (char)132, (char)148, (char)111, (char)235, (char)124, (char)250, (char)166, (char)173, (char)6, (char)203, (char)80, (char)11, (char)38, (char)44, (char)109, (char)222, (char)119, (char)200, (char)5, (char)55, (char)254, (char)231, (char)221, (char)91, (char)194, (char)211, (char)141, (char)247, (char)74, (char)228, (char)125, (char)197, (char)40, (char)55, (char)27, (char)61, (char)107, (char)63, (char)152, (char)115, (char)157, (char)124, (char)34, (char)214, (char)211, (char)191, (char)13, (char)120, (char)213, (char)254, (char)246, (char)94, (char)95, (char)146, (char)85, (char)12, (char)36, (char)49, (char)218, (char)97, (char)48, (char)227, (char)101, (char)99, (char)31, (char)137, (char)199, (char)4, (char)52, (char)41, (char)67, (char)120, (char)71, (char)127, (char)142, (char)31, (char)114, (char)27, (char)56, (char)186, (char)84, (char)24, (char)210, (char)84, (char)5, (char)3, (char)77, (char)213, (char)128, (char)173, (char)93, (char)47, (char)134, (char)117, (char)148, (char)128, (char)241, (char)175, (char)212, (char)227, (char)222, (char)19, (char)50, (char)147, (char)176, (char)253, (char)103, (char)29, (char)200, (char)161, (char)127, (char)104, (char)231, (char)234, (char)46, (char)6, (char)35, (char)25, (char)159, (char)116, (char)83, (char)106, (char)227, (char)197, (char)19, (char)27, (char)229, (char)154, (char)7, (char)104, (char)113, (char)249, (char)128, (char)151, (char)155, (char)236, (char)175, (char)113, (char)172, (char)81, (char)13, (char)245, (char)49, (char)228, (char)250, (char)54, (char)84, (char)31, (char)52, (char)141, (char)119, (char)236, (char)56, (char)211, (char)200, (char)91, (char)126, (char)222, (char)133, (char)43, (char)20, (char)189, (char)2, (char)251, (char)209, (char)54, (char)112, (char)5, (char)4, (char)179, (char)172, (char)79, (char)86, (char)30, (char)254, (char)74, (char)106, (char)2, (char)31, (char)87, (char)215, (char)58, (char)60, (char)186, (char)128, (char)125, (char)235, (char)3, (char)67, (char)140, (char)84, (char)144, (char)80, (char)250, (char)136, (char)20, (char)20, (char)246, (char)25, (char)59, (char)236, (char)44, (char)38, (char)96, (char)209, (char)188, (char)58, (char)212, (char)188, (char)62, (char)252, (char)166, (char)113, (char)143, (char)255, (char)180, (char)57, (char)135, (char)21, (char)72}));
            assert(pack.first_message_offset_GET() == (char)229);
            assert(pack.length_GET() == (char)97);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)62675) ;
        p266.data__SET(new char[] {(char)242, (char)191, (char)218, (char)157, (char)90, (char)126, (char)237, (char)128, (char)120, (char)210, (char)130, (char)145, (char)132, (char)211, (char)24, (char)153, (char)242, (char)229, (char)162, (char)140, (char)249, (char)87, (char)233, (char)156, (char)206, (char)132, (char)111, (char)22, (char)253, (char)132, (char)148, (char)111, (char)235, (char)124, (char)250, (char)166, (char)173, (char)6, (char)203, (char)80, (char)11, (char)38, (char)44, (char)109, (char)222, (char)119, (char)200, (char)5, (char)55, (char)254, (char)231, (char)221, (char)91, (char)194, (char)211, (char)141, (char)247, (char)74, (char)228, (char)125, (char)197, (char)40, (char)55, (char)27, (char)61, (char)107, (char)63, (char)152, (char)115, (char)157, (char)124, (char)34, (char)214, (char)211, (char)191, (char)13, (char)120, (char)213, (char)254, (char)246, (char)94, (char)95, (char)146, (char)85, (char)12, (char)36, (char)49, (char)218, (char)97, (char)48, (char)227, (char)101, (char)99, (char)31, (char)137, (char)199, (char)4, (char)52, (char)41, (char)67, (char)120, (char)71, (char)127, (char)142, (char)31, (char)114, (char)27, (char)56, (char)186, (char)84, (char)24, (char)210, (char)84, (char)5, (char)3, (char)77, (char)213, (char)128, (char)173, (char)93, (char)47, (char)134, (char)117, (char)148, (char)128, (char)241, (char)175, (char)212, (char)227, (char)222, (char)19, (char)50, (char)147, (char)176, (char)253, (char)103, (char)29, (char)200, (char)161, (char)127, (char)104, (char)231, (char)234, (char)46, (char)6, (char)35, (char)25, (char)159, (char)116, (char)83, (char)106, (char)227, (char)197, (char)19, (char)27, (char)229, (char)154, (char)7, (char)104, (char)113, (char)249, (char)128, (char)151, (char)155, (char)236, (char)175, (char)113, (char)172, (char)81, (char)13, (char)245, (char)49, (char)228, (char)250, (char)54, (char)84, (char)31, (char)52, (char)141, (char)119, (char)236, (char)56, (char)211, (char)200, (char)91, (char)126, (char)222, (char)133, (char)43, (char)20, (char)189, (char)2, (char)251, (char)209, (char)54, (char)112, (char)5, (char)4, (char)179, (char)172, (char)79, (char)86, (char)30, (char)254, (char)74, (char)106, (char)2, (char)31, (char)87, (char)215, (char)58, (char)60, (char)186, (char)128, (char)125, (char)235, (char)3, (char)67, (char)140, (char)84, (char)144, (char)80, (char)250, (char)136, (char)20, (char)20, (char)246, (char)25, (char)59, (char)236, (char)44, (char)38, (char)96, (char)209, (char)188, (char)58, (char)212, (char)188, (char)62, (char)252, (char)166, (char)113, (char)143, (char)255, (char)180, (char)57, (char)135, (char)21, (char)72}, 0) ;
        p266.target_system_SET((char)82) ;
        p266.length_SET((char)97) ;
        p266.first_message_offset_SET((char)229) ;
        p266.target_component_SET((char)180) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)26724);
            assert(pack.length_GET() == (char)247);
            assert(pack.target_component_GET() == (char)201);
            assert(pack.target_system_GET() == (char)42);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)218, (char)124, (char)22, (char)208, (char)165, (char)143, (char)124, (char)160, (char)77, (char)104, (char)92, (char)205, (char)144, (char)127, (char)174, (char)75, (char)190, (char)55, (char)186, (char)239, (char)35, (char)115, (char)11, (char)163, (char)90, (char)62, (char)64, (char)217, (char)225, (char)97, (char)69, (char)98, (char)226, (char)64, (char)99, (char)105, (char)102, (char)145, (char)181, (char)199, (char)200, (char)96, (char)243, (char)208, (char)148, (char)167, (char)138, (char)196, (char)79, (char)42, (char)206, (char)79, (char)161, (char)27, (char)162, (char)208, (char)170, (char)240, (char)78, (char)90, (char)253, (char)141, (char)88, (char)150, (char)142, (char)22, (char)163, (char)112, (char)204, (char)21, (char)178, (char)90, (char)205, (char)180, (char)231, (char)119, (char)248, (char)95, (char)236, (char)235, (char)210, (char)98, (char)62, (char)52, (char)82, (char)224, (char)73, (char)9, (char)190, (char)220, (char)12, (char)246, (char)137, (char)169, (char)100, (char)222, (char)6, (char)14, (char)30, (char)193, (char)70, (char)91, (char)108, (char)202, (char)48, (char)165, (char)125, (char)0, (char)180, (char)161, (char)112, (char)65, (char)6, (char)145, (char)62, (char)173, (char)111, (char)113, (char)253, (char)254, (char)246, (char)108, (char)52, (char)133, (char)146, (char)149, (char)199, (char)74, (char)122, (char)50, (char)33, (char)140, (char)3, (char)72, (char)35, (char)154, (char)206, (char)44, (char)133, (char)194, (char)230, (char)172, (char)64, (char)58, (char)121, (char)174, (char)189, (char)235, (char)0, (char)76, (char)200, (char)121, (char)92, (char)224, (char)243, (char)32, (char)154, (char)186, (char)168, (char)74, (char)22, (char)94, (char)70, (char)50, (char)190, (char)120, (char)24, (char)216, (char)200, (char)199, (char)56, (char)41, (char)34, (char)40, (char)223, (char)27, (char)34, (char)49, (char)70, (char)24, (char)107, (char)2, (char)221, (char)162, (char)135, (char)94, (char)111, (char)217, (char)201, (char)189, (char)155, (char)85, (char)213, (char)3, (char)206, (char)217, (char)92, (char)243, (char)172, (char)65, (char)201, (char)39, (char)91, (char)54, (char)67, (char)114, (char)63, (char)190, (char)238, (char)228, (char)191, (char)201, (char)105, (char)154, (char)102, (char)143, (char)243, (char)53, (char)68, (char)209, (char)59, (char)94, (char)81, (char)193, (char)220, (char)145, (char)187, (char)99, (char)22, (char)227, (char)247, (char)46, (char)114, (char)192, (char)136, (char)52, (char)20, (char)96, (char)198, (char)93, (char)82, (char)67, (char)41, (char)85, (char)65, (char)1, (char)75, (char)143, (char)99}));
            assert(pack.first_message_offset_GET() == (char)197);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)201) ;
        p267.first_message_offset_SET((char)197) ;
        p267.sequence_SET((char)26724) ;
        p267.data__SET(new char[] {(char)218, (char)124, (char)22, (char)208, (char)165, (char)143, (char)124, (char)160, (char)77, (char)104, (char)92, (char)205, (char)144, (char)127, (char)174, (char)75, (char)190, (char)55, (char)186, (char)239, (char)35, (char)115, (char)11, (char)163, (char)90, (char)62, (char)64, (char)217, (char)225, (char)97, (char)69, (char)98, (char)226, (char)64, (char)99, (char)105, (char)102, (char)145, (char)181, (char)199, (char)200, (char)96, (char)243, (char)208, (char)148, (char)167, (char)138, (char)196, (char)79, (char)42, (char)206, (char)79, (char)161, (char)27, (char)162, (char)208, (char)170, (char)240, (char)78, (char)90, (char)253, (char)141, (char)88, (char)150, (char)142, (char)22, (char)163, (char)112, (char)204, (char)21, (char)178, (char)90, (char)205, (char)180, (char)231, (char)119, (char)248, (char)95, (char)236, (char)235, (char)210, (char)98, (char)62, (char)52, (char)82, (char)224, (char)73, (char)9, (char)190, (char)220, (char)12, (char)246, (char)137, (char)169, (char)100, (char)222, (char)6, (char)14, (char)30, (char)193, (char)70, (char)91, (char)108, (char)202, (char)48, (char)165, (char)125, (char)0, (char)180, (char)161, (char)112, (char)65, (char)6, (char)145, (char)62, (char)173, (char)111, (char)113, (char)253, (char)254, (char)246, (char)108, (char)52, (char)133, (char)146, (char)149, (char)199, (char)74, (char)122, (char)50, (char)33, (char)140, (char)3, (char)72, (char)35, (char)154, (char)206, (char)44, (char)133, (char)194, (char)230, (char)172, (char)64, (char)58, (char)121, (char)174, (char)189, (char)235, (char)0, (char)76, (char)200, (char)121, (char)92, (char)224, (char)243, (char)32, (char)154, (char)186, (char)168, (char)74, (char)22, (char)94, (char)70, (char)50, (char)190, (char)120, (char)24, (char)216, (char)200, (char)199, (char)56, (char)41, (char)34, (char)40, (char)223, (char)27, (char)34, (char)49, (char)70, (char)24, (char)107, (char)2, (char)221, (char)162, (char)135, (char)94, (char)111, (char)217, (char)201, (char)189, (char)155, (char)85, (char)213, (char)3, (char)206, (char)217, (char)92, (char)243, (char)172, (char)65, (char)201, (char)39, (char)91, (char)54, (char)67, (char)114, (char)63, (char)190, (char)238, (char)228, (char)191, (char)201, (char)105, (char)154, (char)102, (char)143, (char)243, (char)53, (char)68, (char)209, (char)59, (char)94, (char)81, (char)193, (char)220, (char)145, (char)187, (char)99, (char)22, (char)227, (char)247, (char)46, (char)114, (char)192, (char)136, (char)52, (char)20, (char)96, (char)198, (char)93, (char)82, (char)67, (char)41, (char)85, (char)65, (char)1, (char)75, (char)143, (char)99}, 0) ;
        p267.target_system_SET((char)42) ;
        p267.length_SET((char)247) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)27010);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.target_component_GET() == (char)61);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)27010) ;
        p268.target_component_SET((char)61) ;
        p268.target_system_SET((char)175) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 2535327855L);
            assert(pack.status_GET() == (char)102);
            assert(pack.rotation_GET() == (char)6768);
            assert(pack.resolution_h_GET() == (char)60371);
            assert(pack.camera_id_GET() == (char)242);
            assert(pack.resolution_v_GET() == (char)35785);
            assert(pack.framerate_GET() == -1.7011154E38F);
            assert(pack.uri_LEN(ph) == 29);
            assert(pack.uri_TRY(ph).equals("ynbymmtmwtyfeumlpsiuwxmosxpTd"));
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("ynbymmtmwtyfeumlpsiuwxmosxpTd", PH) ;
        p269.resolution_h_SET((char)60371) ;
        p269.camera_id_SET((char)242) ;
        p269.rotation_SET((char)6768) ;
        p269.bitrate_SET(2535327855L) ;
        p269.framerate_SET(-1.7011154E38F) ;
        p269.status_SET((char)102) ;
        p269.resolution_v_SET((char)35785) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)57135);
            assert(pack.bitrate_GET() == 2895175698L);
            assert(pack.target_component_GET() == (char)94);
            assert(pack.camera_id_GET() == (char)219);
            assert(pack.framerate_GET() == 1.6928935E37F);
            assert(pack.resolution_v_GET() == (char)1547);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.uri_LEN(ph) == 102);
            assert(pack.uri_TRY(ph).equals("unxvThirzgjwbsrYjasvuzeshnJOqkoimiJyozpaqgzxldZgRMxusocvsGscvgrhsqghpfsunrfqonveigbojxbrjzivzrawfrkgGz"));
            assert(pack.resolution_h_GET() == (char)15824);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(2895175698L) ;
        p270.target_component_SET((char)94) ;
        p270.uri_SET("unxvThirzgjwbsrYjasvuzeshnJOqkoimiJyozpaqgzxldZgRMxusocvsGscvgrhsqghpfsunrfqonveigbojxbrjzivzrawfrkgGz", PH) ;
        p270.target_system_SET((char)240) ;
        p270.framerate_SET(1.6928935E37F) ;
        p270.rotation_SET((char)57135) ;
        p270.resolution_v_SET((char)1547) ;
        p270.resolution_h_SET((char)15824) ;
        p270.camera_id_SET((char)219) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 8);
            assert(pack.ssid_TRY(ph).equals("ayyLskng"));
            assert(pack.password_LEN(ph) == 22);
            assert(pack.password_TRY(ph).equals("nnbSarafwbVcvkzgctUqxN"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("ayyLskng", PH) ;
        p299.password_SET("nnbSarafwbVcvkzgctUqxN", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.min_version_GET() == (char)18650);
            assert(pack.max_version_GET() == (char)11043);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)195, (char)35, (char)161, (char)252, (char)118, (char)62, (char)166, (char)244}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)131, (char)77, (char)90, (char)216, (char)58, (char)134, (char)103, (char)236}));
            assert(pack.version_GET() == (char)63505);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.spec_version_hash_SET(new char[] {(char)131, (char)77, (char)90, (char)216, (char)58, (char)134, (char)103, (char)236}, 0) ;
        p300.min_version_SET((char)18650) ;
        p300.version_SET((char)63505) ;
        p300.library_version_hash_SET(new char[] {(char)195, (char)35, (char)161, (char)252, (char)118, (char)62, (char)166, (char)244}, 0) ;
        p300.max_version_SET((char)11043) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4196589802651483336L);
            assert(pack.vendor_specific_status_code_GET() == (char)3629);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.uptime_sec_GET() == 545290382L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.sub_mode_GET() == (char)161);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)161) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.vendor_specific_status_code_SET((char)3629) ;
        p310.uptime_sec_SET(545290382L) ;
        p310.time_usec_SET(4196589802651483336L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.hw_version_minor_GET() == (char)45);
            assert(pack.sw_version_major_GET() == (char)72);
            assert(pack.sw_version_minor_GET() == (char)89);
            assert(pack.time_usec_GET() == 5754004316362435361L);
            assert(pack.uptime_sec_GET() == 4170748443L);
            assert(pack.sw_vcs_commit_GET() == 388604416L);
            assert(pack.name_LEN(ph) == 74);
            assert(pack.name_TRY(ph).equals("khcrpfqtxdoabwvgnsymlbhdKqlfvfdqfkwkfygrfqcfxfKjjvtgcObxotRoofdlhpezpwgsVr"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)253, (char)251, (char)13, (char)202, (char)110, (char)152, (char)181, (char)23, (char)232, (char)132, (char)60, (char)197, (char)168, (char)3, (char)215, (char)3}));
            assert(pack.hw_version_major_GET() == (char)167);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.name_SET("khcrpfqtxdoabwvgnsymlbhdKqlfvfdqfkwkfygrfqcfxfKjjvtgcObxotRoofdlhpezpwgsVr", PH) ;
        p311.sw_version_minor_SET((char)89) ;
        p311.uptime_sec_SET(4170748443L) ;
        p311.hw_unique_id_SET(new char[] {(char)253, (char)251, (char)13, (char)202, (char)110, (char)152, (char)181, (char)23, (char)232, (char)132, (char)60, (char)197, (char)168, (char)3, (char)215, (char)3}, 0) ;
        p311.sw_version_major_SET((char)72) ;
        p311.hw_version_major_SET((char)167) ;
        p311.hw_version_minor_SET((char)45) ;
        p311.sw_vcs_commit_SET(388604416L) ;
        p311.time_usec_SET(5754004316362435361L) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)218);
            assert(pack.target_system_GET() == (char)251);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("psfWwurt"));
            assert(pack.param_index_GET() == (short) -6911);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)251) ;
        p320.param_id_SET("psfWwurt", PH) ;
        p320.target_component_SET((char)218) ;
        p320.param_index_SET((short) -6911) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)160);
            assert(pack.target_system_GET() == (char)107);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)107) ;
        p321.target_component_SET((char)160) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.param_index_GET() == (char)50888);
            assert(pack.param_value_LEN(ph) == 7);
            assert(pack.param_value_TRY(ph).equals("Sewoepe"));
            assert(pack.param_count_GET() == (char)5328);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("xSyKdfuxmiskdV"));
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("Sewoepe", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p322.param_count_SET((char)5328) ;
        p322.param_index_SET((char)50888) ;
        p322.param_id_SET("xSyKdfuxmiskdV", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)18);
            assert(pack.target_system_GET() == (char)140);
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("qHk"));
            assert(pack.param_value_LEN(ph) == 79);
            assert(pack.param_value_TRY(ph).equals("qmdyyqjnaLwliViGtzvrjnrvYximmEtaPmieosUziosMvdyfutlqFnyzsdpcwwkmcxlwjvvlwgRclUv"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("qHk", PH) ;
        p323.target_component_SET((char)18) ;
        p323.param_value_SET("qmdyyqjnaLwliViGtzvrjnrvYximmEtaPmieosUziosMvdyfutlqFnyzsdpcwwkmcxlwjvvlwgRclUv", PH) ;
        p323.target_system_SET((char)140) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_value_LEN(ph) == 5);
            assert(pack.param_value_TRY(ph).equals("fdvpj"));
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("BsqtipkUhmrhxNt"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_id_SET("BsqtipkUhmrhxNt", PH) ;
        p324.param_value_SET("fdvpj", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)199);
            assert(pack.max_distance_GET() == (char)27409);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)56400, (char)11611, (char)49025, (char)8507, (char)38073, (char)56139, (char)55611, (char)2805, (char)37903, (char)45634, (char)56410, (char)743, (char)16583, (char)18768, (char)37953, (char)60228, (char)23877, (char)16462, (char)29896, (char)22747, (char)25619, (char)944, (char)27052, (char)19712, (char)64628, (char)4562, (char)38171, (char)19327, (char)56966, (char)24502, (char)4151, (char)35468, (char)36883, (char)7551, (char)26155, (char)44468, (char)60836, (char)35393, (char)3280, (char)37340, (char)40355, (char)55679, (char)6686, (char)42512, (char)37824, (char)6159, (char)58803, (char)64187, (char)43592, (char)34064, (char)54763, (char)47646, (char)42901, (char)37174, (char)54548, (char)4579, (char)57820, (char)34358, (char)33715, (char)55810, (char)12253, (char)58532, (char)2845, (char)43651, (char)54800, (char)37879, (char)2059, (char)45858, (char)52221, (char)55273, (char)38088, (char)55374}));
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.min_distance_GET() == (char)18889);
            assert(pack.time_usec_GET() == 7584113380591080935L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)199) ;
        p330.max_distance_SET((char)27409) ;
        p330.min_distance_SET((char)18889) ;
        p330.distances_SET(new char[] {(char)56400, (char)11611, (char)49025, (char)8507, (char)38073, (char)56139, (char)55611, (char)2805, (char)37903, (char)45634, (char)56410, (char)743, (char)16583, (char)18768, (char)37953, (char)60228, (char)23877, (char)16462, (char)29896, (char)22747, (char)25619, (char)944, (char)27052, (char)19712, (char)64628, (char)4562, (char)38171, (char)19327, (char)56966, (char)24502, (char)4151, (char)35468, (char)36883, (char)7551, (char)26155, (char)44468, (char)60836, (char)35393, (char)3280, (char)37340, (char)40355, (char)55679, (char)6686, (char)42512, (char)37824, (char)6159, (char)58803, (char)64187, (char)43592, (char)34064, (char)54763, (char)47646, (char)42901, (char)37174, (char)54548, (char)4579, (char)57820, (char)34358, (char)33715, (char)55810, (char)12253, (char)58532, (char)2845, (char)43651, (char)54800, (char)37879, (char)2059, (char)45858, (char)52221, (char)55273, (char)38088, (char)55374}, 0) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p330.time_usec_SET(7584113380591080935L) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
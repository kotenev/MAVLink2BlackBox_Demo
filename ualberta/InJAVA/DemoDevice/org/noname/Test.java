
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
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.mavlink_version_GET() == (char)112);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED3);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
            assert(pack.custom_mode_GET() == 576117895L);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p0.custom_mode_SET(576117895L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_FLIGHT_TERMINATION) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED3) ;
        p0.mavlink_version_SET((char)112) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
            assert(pack.battery_remaining_GET() == (byte)123);
            assert(pack.errors_count2_GET() == (char)33600);
            assert(pack.errors_comm_GET() == (char)51727);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            assert(pack.errors_count3_GET() == (char)37792);
            assert(pack.current_battery_GET() == (short)2334);
            assert(pack.voltage_battery_GET() == (char)29883);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
            assert(pack.load_GET() == (char)8991);
            assert(pack.drop_rate_comm_GET() == (char)26307);
            assert(pack.errors_count4_GET() == (char)17219);
            assert(pack.errors_count1_GET() == (char)46228);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.current_battery_SET((short)2334) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) ;
        p1.load_SET((char)8991) ;
        p1.voltage_battery_SET((char)29883) ;
        p1.errors_count3_SET((char)37792) ;
        p1.battery_remaining_SET((byte)123) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) ;
        p1.errors_count4_SET((char)17219) ;
        p1.errors_count1_SET((char)46228) ;
        p1.errors_comm_SET((char)51727) ;
        p1.drop_rate_comm_SET((char)26307) ;
        p1.errors_count2_SET((char)33600) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2007642015L);
            assert(pack.time_unix_usec_GET() == 588321682913180021L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(588321682913180021L) ;
        p2.time_boot_ms_SET(2007642015L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 7.511396E37F);
            assert(pack.z_GET() == 3.9574394E37F);
            assert(pack.vx_GET() == -1.7597679E37F);
            assert(pack.afx_GET() == 2.1160839E38F);
            assert(pack.afz_GET() == -7.3783715E37F);
            assert(pack.y_GET() == -1.0175328E38F);
            assert(pack.vy_GET() == 1.3502673E38F);
            assert(pack.type_mask_GET() == (char)55552);
            assert(pack.time_boot_ms_GET() == 2951139925L);
            assert(pack.x_GET() == 2.170564E38F);
            assert(pack.yaw_rate_GET() == 3.889171E33F);
            assert(pack.yaw_GET() == 3.2588954E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.vz_GET() == 2.0597947E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.y_SET(-1.0175328E38F) ;
        p3.afz_SET(-7.3783715E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p3.vx_SET(-1.7597679E37F) ;
        p3.z_SET(3.9574394E37F) ;
        p3.yaw_rate_SET(3.889171E33F) ;
        p3.yaw_SET(3.2588954E37F) ;
        p3.time_boot_ms_SET(2951139925L) ;
        p3.vz_SET(2.0597947E38F) ;
        p3.vy_SET(1.3502673E38F) ;
        p3.afy_SET(7.511396E37F) ;
        p3.afx_SET(2.1160839E38F) ;
        p3.x_SET(2.170564E38F) ;
        p3.type_mask_SET((char)55552) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3997483576L);
            assert(pack.time_usec_GET() == 6791085229165747844L);
            assert(pack.target_system_GET() == (char)15);
            assert(pack.target_component_GET() == (char)130);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.target_system_SET((char)15) ;
        p4.target_component_SET((char)130) ;
        p4.seq_SET(3997483576L) ;
        p4.time_usec_SET(6791085229165747844L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)22);
            assert(pack.target_system_GET() == (char)222);
            assert(pack.passkey_LEN(ph) == 9);
            assert(pack.passkey_TRY(ph).equals("ndzpgsjfs"));
            assert(pack.control_request_GET() == (char)59);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)59) ;
        p5.target_system_SET((char)222) ;
        p5.passkey_SET("ndzpgsjfs", PH) ;
        p5.version_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)210);
            assert(pack.control_request_GET() == (char)200);
            assert(pack.gcs_system_id_GET() == (char)229);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)200) ;
        p6.ack_SET((char)210) ;
        p6.gcs_system_id_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 24);
            assert(pack.key_TRY(ph).equals("wNfomvlGEtjodgjtyihohsnp"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("wNfomvlGEtjodgjtyihohsnp", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)90);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_GUIDED_ARMED);
            assert(pack.custom_mode_GET() == 2156928422L);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED) ;
        p11.custom_mode_SET(2156928422L) ;
        p11.target_system_SET((char)90) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("cylfaUqahJ"));
            assert(pack.target_component_GET() == (char)247);
            assert(pack.param_index_GET() == (short)20609);
            assert(pack.target_system_GET() == (char)68);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_index_SET((short)20609) ;
        p20.target_component_SET((char)247) ;
        p20.target_system_SET((char)68) ;
        p20.param_id_SET("cylfaUqahJ", PH) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)121);
            assert(pack.target_system_GET() == (char)172);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)172) ;
        p21.target_component_SET((char)121) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
            assert(pack.param_count_GET() == (char)29598);
            assert(pack.param_index_GET() == (char)13413);
            assert(pack.param_value_GET() == -1.2474359E38F);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("pitzjvzkdhok"));
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("pitzjvzkdhok", PH) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        p22.param_value_SET(-1.2474359E38F) ;
        p22.param_index_SET((char)13413) ;
        p22.param_count_SET((char)29598) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("fempvGl"));
            assert(pack.target_system_GET() == (char)158);
            assert(pack.param_value_GET() == 1.3721141E38F);
            assert(pack.target_component_GET() == (char)171);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)171) ;
        p23.target_system_SET((char)158) ;
        p23.param_id_SET("fempvGl", PH) ;
        p23.param_value_SET(1.3721141E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)22347);
            assert(pack.v_acc_TRY(ph) == 949988436L);
            assert(pack.alt_GET() == -883969647);
            assert(pack.lat_GET() == 1077749544);
            assert(pack.vel_acc_TRY(ph) == 846569690L);
            assert(pack.satellites_visible_GET() == (char)171);
            assert(pack.alt_ellipsoid_TRY(ph) == 344483166);
            assert(pack.h_acc_TRY(ph) == 390210272L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.epv_GET() == (char)29672);
            assert(pack.lon_GET() == -125802827);
            assert(pack.time_usec_GET() == 7880125346063715496L);
            assert(pack.hdg_acc_TRY(ph) == 3844319181L);
            assert(pack.vel_GET() == (char)39931);
            assert(pack.eph_GET() == (char)36907);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(7880125346063715496L) ;
        p24.cog_SET((char)22347) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p24.lon_SET(-125802827) ;
        p24.lat_SET(1077749544) ;
        p24.vel_SET((char)39931) ;
        p24.epv_SET((char)29672) ;
        p24.h_acc_SET(390210272L, PH) ;
        p24.alt_ellipsoid_SET(344483166, PH) ;
        p24.vel_acc_SET(846569690L, PH) ;
        p24.satellites_visible_SET((char)171) ;
        p24.eph_SET((char)36907) ;
        p24.v_acc_SET(949988436L, PH) ;
        p24.alt_SET(-883969647) ;
        p24.hdg_acc_SET(3844319181L, PH) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)240, (char)102, (char)164, (char)28, (char)210, (char)211, (char)246, (char)172, (char)62, (char)12, (char)31, (char)21, (char)216, (char)90, (char)231, (char)214, (char)84, (char)91, (char)57, (char)197}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)135, (char)227, (char)146, (char)254, (char)228, (char)149, (char)35, (char)143, (char)58, (char)93, (char)91, (char)100, (char)89, (char)142, (char)130, (char)50, (char)236, (char)162, (char)35, (char)68}));
            assert(pack.satellites_visible_GET() == (char)3);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)54, (char)202, (char)75, (char)195, (char)152, (char)139, (char)216, (char)69, (char)152, (char)208, (char)228, (char)19, (char)62, (char)240, (char)84, (char)229, (char)44, (char)23, (char)203, (char)245}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)147, (char)69, (char)164, (char)83, (char)200, (char)26, (char)1, (char)213, (char)140, (char)53, (char)7, (char)5, (char)112, (char)249, (char)202, (char)74, (char)129, (char)20, (char)208, (char)23}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)81, (char)244, (char)81, (char)143, (char)42, (char)37, (char)0, (char)188, (char)245, (char)145, (char)93, (char)184, (char)236, (char)170, (char)61, (char)178, (char)102, (char)12, (char)196, (char)87}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)135, (char)227, (char)146, (char)254, (char)228, (char)149, (char)35, (char)143, (char)58, (char)93, (char)91, (char)100, (char)89, (char)142, (char)130, (char)50, (char)236, (char)162, (char)35, (char)68}, 0) ;
        p25.satellite_used_SET(new char[] {(char)240, (char)102, (char)164, (char)28, (char)210, (char)211, (char)246, (char)172, (char)62, (char)12, (char)31, (char)21, (char)216, (char)90, (char)231, (char)214, (char)84, (char)91, (char)57, (char)197}, 0) ;
        p25.satellites_visible_SET((char)3) ;
        p25.satellite_azimuth_SET(new char[] {(char)81, (char)244, (char)81, (char)143, (char)42, (char)37, (char)0, (char)188, (char)245, (char)145, (char)93, (char)184, (char)236, (char)170, (char)61, (char)178, (char)102, (char)12, (char)196, (char)87}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)54, (char)202, (char)75, (char)195, (char)152, (char)139, (char)216, (char)69, (char)152, (char)208, (char)228, (char)19, (char)62, (char)240, (char)84, (char)229, (char)44, (char)23, (char)203, (char)245}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)147, (char)69, (char)164, (char)83, (char)200, (char)26, (char)1, (char)213, (char)140, (char)53, (char)7, (char)5, (char)112, (char)249, (char)202, (char)74, (char)129, (char)20, (char)208, (char)23}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short) -32695);
            assert(pack.ygyro_GET() == (short) -31479);
            assert(pack.xgyro_GET() == (short) -9454);
            assert(pack.zgyro_GET() == (short)20360);
            assert(pack.xmag_GET() == (short) -21030);
            assert(pack.yacc_GET() == (short) -25550);
            assert(pack.zacc_GET() == (short)25353);
            assert(pack.xacc_GET() == (short)24593);
            assert(pack.ymag_GET() == (short)27046);
            assert(pack.time_boot_ms_GET() == 3746418125L);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.ygyro_SET((short) -31479) ;
        p26.ymag_SET((short)27046) ;
        p26.xacc_SET((short)24593) ;
        p26.zmag_SET((short) -32695) ;
        p26.xgyro_SET((short) -9454) ;
        p26.time_boot_ms_SET(3746418125L) ;
        p26.xmag_SET((short) -21030) ;
        p26.zgyro_SET((short)20360) ;
        p26.zacc_SET((short)25353) ;
        p26.yacc_SET((short) -25550) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -1342);
            assert(pack.time_usec_GET() == 8961111653001282423L);
            assert(pack.yacc_GET() == (short) -13115);
            assert(pack.xgyro_GET() == (short) -11836);
            assert(pack.xmag_GET() == (short)18541);
            assert(pack.ygyro_GET() == (short)12971);
            assert(pack.zacc_GET() == (short) -24139);
            assert(pack.zmag_GET() == (short)3940);
            assert(pack.ymag_GET() == (short)3941);
            assert(pack.zgyro_GET() == (short)12045);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.xgyro_SET((short) -11836) ;
        p27.zmag_SET((short)3940) ;
        p27.time_usec_SET(8961111653001282423L) ;
        p27.yacc_SET((short) -13115) ;
        p27.ygyro_SET((short)12971) ;
        p27.xacc_SET((short) -1342) ;
        p27.xmag_SET((short)18541) ;
        p27.ymag_SET((short)3941) ;
        p27.zacc_SET((short) -24139) ;
        p27.zgyro_SET((short)12045) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)21725);
            assert(pack.temperature_GET() == (short)2314);
            assert(pack.press_abs_GET() == (short)21981);
            assert(pack.time_usec_GET() == 5865958255537427047L);
            assert(pack.press_diff1_GET() == (short) -26597);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short)2314) ;
        p28.press_abs_SET((short)21981) ;
        p28.press_diff2_SET((short)21725) ;
        p28.time_usec_SET(5865958255537427047L) ;
        p28.press_diff1_SET((short) -26597) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -1.3752575E38F);
            assert(pack.time_boot_ms_GET() == 450969141L);
            assert(pack.temperature_GET() == (short)15306);
            assert(pack.press_diff_GET() == 2.787834E38F);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.temperature_SET((short)15306) ;
        p29.press_diff_SET(2.787834E38F) ;
        p29.time_boot_ms_SET(450969141L) ;
        p29.press_abs_SET(-1.3752575E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.4843605E38F);
            assert(pack.yaw_GET() == -3.1769373E38F);
            assert(pack.time_boot_ms_GET() == 3330657609L);
            assert(pack.roll_GET() == 2.4365508E38F);
            assert(pack.pitchspeed_GET() == -1.4401601E38F);
            assert(pack.yawspeed_GET() == -2.3040525E38F);
            assert(pack.rollspeed_GET() == 2.9231155E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.roll_SET(2.4365508E38F) ;
        p30.pitchspeed_SET(-1.4401601E38F) ;
        p30.pitch_SET(-2.4843605E38F) ;
        p30.yawspeed_SET(-2.3040525E38F) ;
        p30.time_boot_ms_SET(3330657609L) ;
        p30.yaw_SET(-3.1769373E38F) ;
        p30.rollspeed_SET(2.9231155E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == 1.7176905E38F);
            assert(pack.q4_GET() == -1.9449602E38F);
            assert(pack.yawspeed_GET() == 2.9620822E38F);
            assert(pack.time_boot_ms_GET() == 1719006743L);
            assert(pack.pitchspeed_GET() == -2.7484046E38F);
            assert(pack.q1_GET() == 2.8170874E38F);
            assert(pack.q3_GET() == -7.725494E37F);
            assert(pack.rollspeed_GET() == 1.7163809E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.pitchspeed_SET(-2.7484046E38F) ;
        p31.time_boot_ms_SET(1719006743L) ;
        p31.q1_SET(2.8170874E38F) ;
        p31.q2_SET(1.7176905E38F) ;
        p31.rollspeed_SET(1.7163809E38F) ;
        p31.yawspeed_SET(2.9620822E38F) ;
        p31.q4_SET(-1.9449602E38F) ;
        p31.q3_SET(-7.725494E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == -6.4968995E37F);
            assert(pack.x_GET() == -6.2433445E37F);
            assert(pack.z_GET() == 2.560081E37F);
            assert(pack.vy_GET() == 6.821624E37F);
            assert(pack.time_boot_ms_GET() == 2707240934L);
            assert(pack.y_GET() == 1.6369891E38F);
            assert(pack.vz_GET() == -2.615393E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.z_SET(2.560081E37F) ;
        p32.y_SET(1.6369891E38F) ;
        p32.time_boot_ms_SET(2707240934L) ;
        p32.vz_SET(-2.615393E38F) ;
        p32.vy_SET(6.821624E37F) ;
        p32.vx_SET(-6.4968995E37F) ;
        p32.x_SET(-6.2433445E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.hdg_GET() == (char)33490);
            assert(pack.alt_GET() == -826256278);
            assert(pack.lon_GET() == -444684999);
            assert(pack.lat_GET() == 55273945);
            assert(pack.vx_GET() == (short)25237);
            assert(pack.vz_GET() == (short)27142);
            assert(pack.vy_GET() == (short)18838);
            assert(pack.time_boot_ms_GET() == 816379023L);
            assert(pack.relative_alt_GET() == -87758414);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.relative_alt_SET(-87758414) ;
        p33.vz_SET((short)27142) ;
        p33.vx_SET((short)25237) ;
        p33.vy_SET((short)18838) ;
        p33.time_boot_ms_SET(816379023L) ;
        p33.lon_SET(-444684999) ;
        p33.lat_SET(55273945) ;
        p33.hdg_SET((char)33490) ;
        p33.alt_SET(-826256278) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4154189580L);
            assert(pack.chan6_scaled_GET() == (short) -16015);
            assert(pack.chan4_scaled_GET() == (short)25920);
            assert(pack.chan3_scaled_GET() == (short)32196);
            assert(pack.chan7_scaled_GET() == (short)27929);
            assert(pack.port_GET() == (char)149);
            assert(pack.chan5_scaled_GET() == (short)9496);
            assert(pack.rssi_GET() == (char)135);
            assert(pack.chan8_scaled_GET() == (short)16209);
            assert(pack.chan2_scaled_GET() == (short)25397);
            assert(pack.chan1_scaled_GET() == (short)25898);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(4154189580L) ;
        p34.chan5_scaled_SET((short)9496) ;
        p34.chan6_scaled_SET((short) -16015) ;
        p34.rssi_SET((char)135) ;
        p34.chan4_scaled_SET((short)25920) ;
        p34.chan8_scaled_SET((short)16209) ;
        p34.chan2_scaled_SET((short)25397) ;
        p34.chan1_scaled_SET((short)25898) ;
        p34.chan7_scaled_SET((short)27929) ;
        p34.chan3_scaled_SET((short)32196) ;
        p34.port_SET((char)149) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)57557);
            assert(pack.rssi_GET() == (char)209);
            assert(pack.chan4_raw_GET() == (char)39387);
            assert(pack.time_boot_ms_GET() == 232038571L);
            assert(pack.chan3_raw_GET() == (char)41030);
            assert(pack.port_GET() == (char)103);
            assert(pack.chan1_raw_GET() == (char)29443);
            assert(pack.chan5_raw_GET() == (char)29262);
            assert(pack.chan2_raw_GET() == (char)60490);
            assert(pack.chan8_raw_GET() == (char)647);
            assert(pack.chan7_raw_GET() == (char)21995);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan5_raw_SET((char)29262) ;
        p35.port_SET((char)103) ;
        p35.chan7_raw_SET((char)21995) ;
        p35.chan3_raw_SET((char)41030) ;
        p35.chan2_raw_SET((char)60490) ;
        p35.chan1_raw_SET((char)29443) ;
        p35.time_boot_ms_SET(232038571L) ;
        p35.rssi_SET((char)209) ;
        p35.chan4_raw_SET((char)39387) ;
        p35.chan8_raw_SET((char)647) ;
        p35.chan6_raw_SET((char)57557) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo13_raw_TRY(ph) == (char)3533);
            assert(pack.servo2_raw_GET() == (char)59936);
            assert(pack.servo15_raw_TRY(ph) == (char)47356);
            assert(pack.port_GET() == (char)141);
            assert(pack.time_usec_GET() == 2523073878L);
            assert(pack.servo3_raw_GET() == (char)23709);
            assert(pack.servo11_raw_TRY(ph) == (char)45132);
            assert(pack.servo6_raw_GET() == (char)47729);
            assert(pack.servo8_raw_GET() == (char)54464);
            assert(pack.servo14_raw_TRY(ph) == (char)26954);
            assert(pack.servo5_raw_GET() == (char)38285);
            assert(pack.servo16_raw_TRY(ph) == (char)40818);
            assert(pack.servo1_raw_GET() == (char)46782);
            assert(pack.servo7_raw_GET() == (char)52350);
            assert(pack.servo9_raw_TRY(ph) == (char)47119);
            assert(pack.servo12_raw_TRY(ph) == (char)16036);
            assert(pack.servo10_raw_TRY(ph) == (char)20122);
            assert(pack.servo4_raw_GET() == (char)62093);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo4_raw_SET((char)62093) ;
        p36.servo13_raw_SET((char)3533, PH) ;
        p36.servo7_raw_SET((char)52350) ;
        p36.servo10_raw_SET((char)20122, PH) ;
        p36.servo15_raw_SET((char)47356, PH) ;
        p36.time_usec_SET(2523073878L) ;
        p36.servo6_raw_SET((char)47729) ;
        p36.servo1_raw_SET((char)46782) ;
        p36.servo11_raw_SET((char)45132, PH) ;
        p36.servo8_raw_SET((char)54464) ;
        p36.servo2_raw_SET((char)59936) ;
        p36.servo5_raw_SET((char)38285) ;
        p36.servo3_raw_SET((char)23709) ;
        p36.servo12_raw_SET((char)16036, PH) ;
        p36.servo14_raw_SET((char)26954, PH) ;
        p36.servo9_raw_SET((char)47119, PH) ;
        p36.port_SET((char)141) ;
        p36.servo16_raw_SET((char)40818, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short) -14860);
            assert(pack.target_system_GET() == (char)82);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)128);
            assert(pack.start_index_GET() == (short) -30103);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short) -30103) ;
        p37.target_component_SET((char)128) ;
        p37.end_index_SET((short) -14860) ;
        p37.target_system_SET((char)82) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)45);
            assert(pack.start_index_GET() == (short)14373);
            assert(pack.end_index_GET() == (short)24441);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)135);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_component_SET((char)45) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.start_index_SET((short)14373) ;
        p38.end_index_SET((short)24441) ;
        p38.target_system_SET((char)135) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == -2.2024239E38F);
            assert(pack.current_GET() == (char)93);
            assert(pack.autocontinue_GET() == (char)38);
            assert(pack.param1_GET() == 2.6940908E38F);
            assert(pack.seq_GET() == (char)53142);
            assert(pack.y_GET() == -4.84512E37F);
            assert(pack.x_GET() == 1.7393428E38F);
            assert(pack.z_GET() == 2.037353E38F);
            assert(pack.param2_GET() == 3.1082695E38F);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
            assert(pack.target_system_GET() == (char)230);
            assert(pack.param3_GET() == 4.7388217E37F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.target_system_SET((char)230) ;
        p39.z_SET(2.037353E38F) ;
        p39.seq_SET((char)53142) ;
        p39.param4_SET(-2.2024239E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p39.x_SET(1.7393428E38F) ;
        p39.param1_SET(2.6940908E38F) ;
        p39.param2_SET(3.1082695E38F) ;
        p39.target_component_SET((char)14) ;
        p39.current_SET((char)93) ;
        p39.param3_SET(4.7388217E37F) ;
        p39.autocontinue_SET((char)38) ;
        p39.command_SET(MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION) ;
        p39.y_SET(-4.84512E37F) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)1126);
            assert(pack.target_system_GET() == (char)177);
            assert(pack.target_component_GET() == (char)12);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)12) ;
        p40.target_system_SET((char)177) ;
        p40.seq_SET((char)1126) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.seq_GET() == (char)43769);
            assert(pack.target_system_GET() == (char)195);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)195) ;
        p41.target_component_SET((char)54) ;
        p41.seq_SET((char)43769) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)27547);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)27547) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)243);
            assert(pack.target_component_GET() == (char)9);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p43.target_system_SET((char)243) ;
        p43.target_component_SET((char)9) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)18248);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.target_system_GET() == (char)138);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)106) ;
        p44.target_system_SET((char)138) ;
        p44.count_SET((char)18248) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)76);
            assert(pack.target_system_GET() == (char)145);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)145) ;
        p45.target_component_SET((char)76) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)35493);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)35493) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)42);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1);
            assert(pack.target_component_GET() == (char)241);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1) ;
        p47.target_system_SET((char)42) ;
        p47.target_component_SET((char)241) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1799366212);
            assert(pack.altitude_GET() == -1140204461);
            assert(pack.time_usec_TRY(ph) == 8771697968212459969L);
            assert(pack.longitude_GET() == -837576722);
            assert(pack.target_system_GET() == (char)176);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(1799366212) ;
        p48.time_usec_SET(8771697968212459969L, PH) ;
        p48.target_system_SET((char)176) ;
        p48.longitude_SET(-837576722) ;
        p48.altitude_SET(-1140204461) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 1729622306);
            assert(pack.longitude_GET() == -1533833155);
            assert(pack.time_usec_TRY(ph) == 628623266356860422L);
            assert(pack.latitude_GET() == 858127339);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-1533833155) ;
        p49.time_usec_SET(628623266356860422L, PH) ;
        p49.altitude_SET(1729622306) ;
        p49.latitude_SET(858127339) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == 3.0450002E38F);
            assert(pack.param_value0_GET() == -1.389614E37F);
            assert(pack.target_system_GET() == (char)187);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("hJvcm"));
            assert(pack.target_component_GET() == (char)9);
            assert(pack.param_index_GET() == (short)14308);
            assert(pack.parameter_rc_channel_index_GET() == (char)181);
            assert(pack.param_value_min_GET() == 2.5467555E38F);
            assert(pack.scale_GET() == -2.4623001E38F);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)181) ;
        p50.param_value_max_SET(3.0450002E38F) ;
        p50.param_index_SET((short)14308) ;
        p50.target_component_SET((char)9) ;
        p50.param_value_min_SET(2.5467555E38F) ;
        p50.scale_SET(-2.4623001E38F) ;
        p50.target_system_SET((char)187) ;
        p50.param_id_SET("hJvcm", PH) ;
        p50.param_value0_SET(-1.389614E37F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.seq_GET() == (char)52840);
            assert(pack.target_component_GET() == (char)200);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)127) ;
        p51.seq_SET((char)52840) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.target_component_SET((char)200) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1y_GET() == 1.1326913E38F);
            assert(pack.p2z_GET() == -1.4275108E38F);
            assert(pack.p1z_GET() == 2.084436E38F);
            assert(pack.target_component_GET() == (char)153);
            assert(pack.target_system_GET() == (char)154);
            assert(pack.p1x_GET() == 2.149352E38F);
            assert(pack.p2x_GET() == -2.047429E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p2y_GET() == 5.905813E34F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_component_SET((char)153) ;
        p54.p2y_SET(5.905813E34F) ;
        p54.p1z_SET(2.084436E38F) ;
        p54.p2x_SET(-2.047429E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p54.p1x_SET(2.149352E38F) ;
        p54.p1y_SET(1.1326913E38F) ;
        p54.p2z_SET(-1.4275108E38F) ;
        p54.target_system_SET((char)154) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2x_GET() == 2.9460754E38F);
            assert(pack.p1z_GET() == -3.4483717E37F);
            assert(pack.p2y_GET() == -1.4497831E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.p1x_GET() == 3.324627E38F);
            assert(pack.p2z_GET() == 1.6110893E38F);
            assert(pack.p1y_GET() == -2.6915762E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1z_SET(-3.4483717E37F) ;
        p55.p2x_SET(2.9460754E38F) ;
        p55.p2y_SET(-1.4497831E38F) ;
        p55.p2z_SET(1.6110893E38F) ;
        p55.p1y_SET(-2.6915762E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p55.p1x_SET(3.324627E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {3.3509972E38F, 4.4140086E37F, -6.4747663E37F, -8.633752E37F, 2.7964816E38F, -1.8195183E38F, -4.2882536E37F, -1.5697043E38F, 9.608472E37F}));
            assert(pack.pitchspeed_GET() == -7.075786E37F);
            assert(pack.rollspeed_GET() == 3.1172518E38F);
            assert(pack.yawspeed_GET() == 1.7939822E38F);
            assert(pack.time_usec_GET() == 1612576266285421154L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.12419E38F, -6.88736E36F, -1.1024334E38F, 2.1982901E38F}));
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {3.3509972E38F, 4.4140086E37F, -6.4747663E37F, -8.633752E37F, 2.7964816E38F, -1.8195183E38F, -4.2882536E37F, -1.5697043E38F, 9.608472E37F}, 0) ;
        p61.pitchspeed_SET(-7.075786E37F) ;
        p61.yawspeed_SET(1.7939822E38F) ;
        p61.rollspeed_SET(3.1172518E38F) ;
        p61.q_SET(new float[] {2.12419E38F, -6.88736E36F, -1.1024334E38F, 2.1982901E38F}, 0) ;
        p61.time_usec_SET(1612576266285421154L) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.nav_pitch_GET() == -3.1546191E38F);
            assert(pack.aspd_error_GET() == 2.4768254E38F);
            assert(pack.xtrack_error_GET() == -8.256491E37F);
            assert(pack.target_bearing_GET() == (short) -11314);
            assert(pack.nav_bearing_GET() == (short)24973);
            assert(pack.nav_roll_GET() == -1.3669087E38F);
            assert(pack.wp_dist_GET() == (char)61099);
            assert(pack.alt_error_GET() == -1.2490765E38F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.target_bearing_SET((short) -11314) ;
        p62.wp_dist_SET((char)61099) ;
        p62.xtrack_error_SET(-8.256491E37F) ;
        p62.aspd_error_SET(2.4768254E38F) ;
        p62.nav_bearing_SET((short)24973) ;
        p62.alt_error_SET(-1.2490765E38F) ;
        p62.nav_pitch_SET(-3.1546191E38F) ;
        p62.nav_roll_SET(-1.3669087E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3628263858815524423L);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vy_GET() == 3.3915355E38F);
            assert(pack.vx_GET() == 1.3935236E37F);
            assert(pack.relative_alt_GET() == -1708497068);
            assert(pack.vz_GET() == 1.4684679E38F);
            assert(pack.lat_GET() == -137217444);
            assert(pack.lon_GET() == 1191607913);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.2899535E38F, -1.08159E38F, 1.0060936E37F, 1.2825793E38F, 2.9751388E38F, 8.642479E37F, 5.4624393E37F, 2.077522E37F, -2.234706E38F, -2.004324E38F, -2.7456498E37F, -1.9752473E38F, 1.040767E38F, -2.5373988E38F, 1.0168337E38F, -8.718285E37F, 1.6788635E37F, 3.466205E37F, 5.866098E37F, 8.188671E37F, 2.1405002E38F, 1.078201E38F, -2.3950064E38F, -1.6916287E38F, -1.8460025E37F, 3.3671335E38F, 1.4423265E38F, 1.5860758E38F, -1.2768002E37F, 3.003152E38F, -1.5288201E37F, 1.5215532E38F, -6.5314424E37F, 1.6356719E38F, 2.8205842E38F, -6.039381E37F}));
            assert(pack.alt_GET() == -1338437726);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lon_SET(1191607913) ;
        p63.vz_SET(1.4684679E38F) ;
        p63.lat_SET(-137217444) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vx_SET(1.3935236E37F) ;
        p63.time_usec_SET(3628263858815524423L) ;
        p63.covariance_SET(new float[] {1.2899535E38F, -1.08159E38F, 1.0060936E37F, 1.2825793E38F, 2.9751388E38F, 8.642479E37F, 5.4624393E37F, 2.077522E37F, -2.234706E38F, -2.004324E38F, -2.7456498E37F, -1.9752473E38F, 1.040767E38F, -2.5373988E38F, 1.0168337E38F, -8.718285E37F, 1.6788635E37F, 3.466205E37F, 5.866098E37F, 8.188671E37F, 2.1405002E38F, 1.078201E38F, -2.3950064E38F, -1.6916287E38F, -1.8460025E37F, 3.3671335E38F, 1.4423265E38F, 1.5860758E38F, -1.2768002E37F, 3.003152E38F, -1.5288201E37F, 1.5215532E38F, -6.5314424E37F, 1.6356719E38F, 2.8205842E38F, -6.039381E37F}, 0) ;
        p63.relative_alt_SET(-1708497068) ;
        p63.vy_SET(3.3915355E38F) ;
        p63.alt_SET(-1338437726) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 3049466941579062176L);
            assert(pack.ax_GET() == 5.687981E37F);
            assert(pack.z_GET() == -1.4835284E38F);
            assert(pack.vx_GET() == 2.8263442E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.195286E38F, -7.7657903E37F, -1.3823132E38F, -5.3737783E37F, -2.3855203E38F, -1.0210059E38F, 2.9583807E38F, -2.1889868E38F, -8.744197E37F, -1.315483E38F, 9.439994E37F, -8.897271E37F, -2.5617444E37F, -3.2153442E38F, -8.888827E37F, 1.4251009E38F, -2.944925E38F, -5.4854544E36F, -1.2374695E38F, 1.9013323E38F, 8.238573E37F, 3.0187185E36F, -8.754071E37F, -3.0971308E38F, -2.6614393E38F, -2.0212892E38F, 6.9169086E37F, -2.0678018E38F, -7.6016955E37F, -3.5562236E37F, -3.0247926E38F, 1.501622E38F, -6.815732E37F, 1.5044129E38F, 5.896155E37F, -1.6351229E38F, -2.329709E38F, -1.756269E37F, -8.1896217E37F, 2.6444116E38F, -8.174432E37F, -6.760251E37F, -3.3447894E38F, 3.4690308E37F, 3.118402E38F}));
            assert(pack.y_GET() == -2.7738436E37F);
            assert(pack.vy_GET() == -1.3281701E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.az_GET() == 6.814928E37F);
            assert(pack.vz_GET() == -1.0598593E38F);
            assert(pack.ay_GET() == -2.1729748E38F);
            assert(pack.x_GET() == 3.266529E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.y_SET(-2.7738436E37F) ;
        p64.vz_SET(-1.0598593E38F) ;
        p64.vy_SET(-1.3281701E38F) ;
        p64.az_SET(6.814928E37F) ;
        p64.time_usec_SET(3049466941579062176L) ;
        p64.vx_SET(2.8263442E38F) ;
        p64.ax_SET(5.687981E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.x_SET(3.266529E38F) ;
        p64.ay_SET(-2.1729748E38F) ;
        p64.z_SET(-1.4835284E38F) ;
        p64.covariance_SET(new float[] {2.195286E38F, -7.7657903E37F, -1.3823132E38F, -5.3737783E37F, -2.3855203E38F, -1.0210059E38F, 2.9583807E38F, -2.1889868E38F, -8.744197E37F, -1.315483E38F, 9.439994E37F, -8.897271E37F, -2.5617444E37F, -3.2153442E38F, -8.888827E37F, 1.4251009E38F, -2.944925E38F, -5.4854544E36F, -1.2374695E38F, 1.9013323E38F, 8.238573E37F, 3.0187185E36F, -8.754071E37F, -3.0971308E38F, -2.6614393E38F, -2.0212892E38F, 6.9169086E37F, -2.0678018E38F, -7.6016955E37F, -3.5562236E37F, -3.0247926E38F, 1.501622E38F, -6.815732E37F, 1.5044129E38F, 5.896155E37F, -1.6351229E38F, -2.329709E38F, -1.756269E37F, -8.1896217E37F, 2.6444116E38F, -8.174432E37F, -6.760251E37F, -3.3447894E38F, 3.4690308E37F, 3.118402E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)22682);
            assert(pack.chan8_raw_GET() == (char)57719);
            assert(pack.chan4_raw_GET() == (char)22957);
            assert(pack.time_boot_ms_GET() == 828722406L);
            assert(pack.rssi_GET() == (char)0);
            assert(pack.chan16_raw_GET() == (char)51585);
            assert(pack.chan6_raw_GET() == (char)32912);
            assert(pack.chan7_raw_GET() == (char)59060);
            assert(pack.chan18_raw_GET() == (char)38999);
            assert(pack.chan14_raw_GET() == (char)15052);
            assert(pack.chan12_raw_GET() == (char)6940);
            assert(pack.chan10_raw_GET() == (char)4356);
            assert(pack.chan9_raw_GET() == (char)28678);
            assert(pack.chan15_raw_GET() == (char)3499);
            assert(pack.chancount_GET() == (char)227);
            assert(pack.chan1_raw_GET() == (char)19622);
            assert(pack.chan17_raw_GET() == (char)33364);
            assert(pack.chan5_raw_GET() == (char)3292);
            assert(pack.chan3_raw_GET() == (char)55457);
            assert(pack.chan13_raw_GET() == (char)4067);
            assert(pack.chan2_raw_GET() == (char)53871);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan1_raw_SET((char)19622) ;
        p65.rssi_SET((char)0) ;
        p65.chan6_raw_SET((char)32912) ;
        p65.chan3_raw_SET((char)55457) ;
        p65.chan14_raw_SET((char)15052) ;
        p65.chan7_raw_SET((char)59060) ;
        p65.chan5_raw_SET((char)3292) ;
        p65.chan17_raw_SET((char)33364) ;
        p65.chan4_raw_SET((char)22957) ;
        p65.chan13_raw_SET((char)4067) ;
        p65.chan9_raw_SET((char)28678) ;
        p65.chan8_raw_SET((char)57719) ;
        p65.chan15_raw_SET((char)3499) ;
        p65.time_boot_ms_SET(828722406L) ;
        p65.chan11_raw_SET((char)22682) ;
        p65.chan10_raw_SET((char)4356) ;
        p65.chan2_raw_SET((char)53871) ;
        p65.chan16_raw_SET((char)51585) ;
        p65.chancount_SET((char)227) ;
        p65.chan12_raw_SET((char)6940) ;
        p65.chan18_raw_SET((char)38999) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)211);
            assert(pack.req_message_rate_GET() == (char)52504);
            assert(pack.req_stream_id_GET() == (char)4);
            assert(pack.start_stop_GET() == (char)212);
            assert(pack.target_component_GET() == (char)123);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)211) ;
        p66.req_stream_id_SET((char)4) ;
        p66.req_message_rate_SET((char)52504) ;
        p66.start_stop_SET((char)212) ;
        p66.target_component_SET((char)123) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)186);
            assert(pack.stream_id_GET() == (char)14);
            assert(pack.message_rate_GET() == (char)6302);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)186) ;
        p67.stream_id_SET((char)14) ;
        p67.message_rate_SET((char)6302) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short) -3802);
            assert(pack.r_GET() == (short) -3446);
            assert(pack.z_GET() == (short) -21319);
            assert(pack.target_GET() == (char)93);
            assert(pack.y_GET() == (short) -67);
            assert(pack.buttons_GET() == (char)7580);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)93) ;
        p69.y_SET((short) -67) ;
        p69.x_SET((short) -3802) ;
        p69.r_SET((short) -3446) ;
        p69.buttons_SET((char)7580) ;
        p69.z_SET((short) -21319) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)61675);
            assert(pack.chan1_raw_GET() == (char)42020);
            assert(pack.chan6_raw_GET() == (char)14338);
            assert(pack.chan4_raw_GET() == (char)14805);
            assert(pack.chan8_raw_GET() == (char)63263);
            assert(pack.chan2_raw_GET() == (char)43642);
            assert(pack.target_component_GET() == (char)245);
            assert(pack.target_system_GET() == (char)124);
            assert(pack.chan7_raw_GET() == (char)28097);
            assert(pack.chan3_raw_GET() == (char)21057);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan3_raw_SET((char)21057) ;
        p70.chan4_raw_SET((char)14805) ;
        p70.chan8_raw_SET((char)63263) ;
        p70.chan6_raw_SET((char)14338) ;
        p70.chan5_raw_SET((char)61675) ;
        p70.chan2_raw_SET((char)43642) ;
        p70.target_component_SET((char)245) ;
        p70.target_system_SET((char)124) ;
        p70.chan1_raw_SET((char)42020) ;
        p70.chan7_raw_SET((char)28097) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 1.0199226E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_GET() == -1191833445);
            assert(pack.autocontinue_GET() == (char)150);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION);
            assert(pack.param4_GET() == -7.2835527E37F);
            assert(pack.y_GET() == 1314861445);
            assert(pack.current_GET() == (char)176);
            assert(pack.z_GET() == -3.0697778E38F);
            assert(pack.param1_GET() == 1.8451338E38F);
            assert(pack.target_system_GET() == (char)55);
            assert(pack.target_component_GET() == (char)244);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.param3_GET() == 3.0542778E38F);
            assert(pack.seq_GET() == (char)28365);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param1_SET(1.8451338E38F) ;
        p73.target_system_SET((char)55) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION) ;
        p73.param3_SET(3.0542778E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p73.autocontinue_SET((char)150) ;
        p73.x_SET(-1191833445) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.y_SET(1314861445) ;
        p73.current_SET((char)176) ;
        p73.target_component_SET((char)244) ;
        p73.z_SET(-3.0697778E38F) ;
        p73.param4_SET(-7.2835527E37F) ;
        p73.seq_SET((char)28365) ;
        p73.param2_SET(1.0199226E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short) -31789);
            assert(pack.alt_GET() == 1.9894752E38F);
            assert(pack.throttle_GET() == (char)20987);
            assert(pack.groundspeed_GET() == 2.5363315E37F);
            assert(pack.airspeed_GET() == -1.2846387E38F);
            assert(pack.climb_GET() == -1.0135732E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.groundspeed_SET(2.5363315E37F) ;
        p74.throttle_SET((char)20987) ;
        p74.alt_SET(1.9894752E38F) ;
        p74.airspeed_SET(-1.2846387E38F) ;
        p74.climb_SET(-1.0135732E38F) ;
        p74.heading_SET((short) -31789) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 2.0239983E38F);
            assert(pack.y_GET() == 1036510280);
            assert(pack.param2_GET() == -1.5823096E38F);
            assert(pack.target_component_GET() == (char)223);
            assert(pack.current_GET() == (char)121);
            assert(pack.autocontinue_GET() == (char)7);
            assert(pack.x_GET() == 85171430);
            assert(pack.param1_GET() == 5.644324E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE);
            assert(pack.z_GET() == 3.0774646E38F);
            assert(pack.param3_GET() == 2.9565314E38F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.z_SET(3.0774646E38F) ;
        p75.y_SET(1036510280) ;
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p75.x_SET(85171430) ;
        p75.param1_SET(5.644324E37F) ;
        p75.current_SET((char)121) ;
        p75.param3_SET(2.9565314E38F) ;
        p75.param2_SET(-1.5823096E38F) ;
        p75.param4_SET(2.0239983E38F) ;
        p75.target_component_SET((char)223) ;
        p75.target_system_SET((char)72) ;
        p75.autocontinue_SET((char)7) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param6_GET() == 9.910565E37F);
            assert(pack.param5_GET() == 3.0746397E38F);
            assert(pack.param1_GET() == -1.833328E38F);
            assert(pack.target_system_GET() == (char)13);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO);
            assert(pack.confirmation_GET() == (char)218);
            assert(pack.param3_GET() == 6.12343E37F);
            assert(pack.param2_GET() == 1.8807947E38F);
            assert(pack.param7_GET() == 1.3336139E38F);
            assert(pack.param4_GET() == -5.204991E37F);
            assert(pack.target_component_GET() == (char)8);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param3_SET(6.12343E37F) ;
        p76.confirmation_SET((char)218) ;
        p76.command_SET(MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO) ;
        p76.param1_SET(-1.833328E38F) ;
        p76.param5_SET(3.0746397E38F) ;
        p76.param6_SET(9.910565E37F) ;
        p76.target_component_SET((char)8) ;
        p76.param7_SET(1.3336139E38F) ;
        p76.target_system_SET((char)13) ;
        p76.param4_SET(-5.204991E37F) ;
        p76.param2_SET(1.8807947E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_TRY(ph) == (char)172);
            assert(pack.result_param2_TRY(ph) == -641378266);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
            assert(pack.progress_TRY(ph) == (char)148);
            assert(pack.target_system_TRY(ph) == (char)233);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL) ;
        p77.target_system_SET((char)233, PH) ;
        p77.progress_SET((char)148, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS) ;
        p77.target_component_SET((char)172, PH) ;
        p77.result_param2_SET(-641378266, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.3844524E38F);
            assert(pack.thrust_GET() == 3.2889795E38F);
            assert(pack.pitch_GET() == 3.0215916E38F);
            assert(pack.time_boot_ms_GET() == 3045009148L);
            assert(pack.yaw_GET() == 3.131346E37F);
            assert(pack.mode_switch_GET() == (char)41);
            assert(pack.manual_override_switch_GET() == (char)98);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.manual_override_switch_SET((char)98) ;
        p81.pitch_SET(3.0215916E38F) ;
        p81.time_boot_ms_SET(3045009148L) ;
        p81.thrust_SET(3.2889795E38F) ;
        p81.mode_switch_SET((char)41) ;
        p81.roll_SET(-1.3844524E38F) ;
        p81.yaw_SET(3.131346E37F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -1.1858525E38F);
            assert(pack.type_mask_GET() == (char)86);
            assert(pack.body_yaw_rate_GET() == -3.2656436E38F);
            assert(pack.time_boot_ms_GET() == 2896694876L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.0617877E38F, -2.5340163E38F, 1.829116E38F, 1.1916917E38F}));
            assert(pack.thrust_GET() == 1.7598921E38F);
            assert(pack.target_component_GET() == (char)178);
            assert(pack.target_system_GET() == (char)147);
            assert(pack.body_pitch_rate_GET() == 2.657482E38F);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.thrust_SET(1.7598921E38F) ;
        p82.body_pitch_rate_SET(2.657482E38F) ;
        p82.type_mask_SET((char)86) ;
        p82.body_yaw_rate_SET(-3.2656436E38F) ;
        p82.time_boot_ms_SET(2896694876L) ;
        p82.body_roll_rate_SET(-1.1858525E38F) ;
        p82.q_SET(new float[] {-3.0617877E38F, -2.5340163E38F, 1.829116E38F, 1.1916917E38F}, 0) ;
        p82.target_system_SET((char)147) ;
        p82.target_component_SET((char)178) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 2.1018188E38F);
            assert(pack.type_mask_GET() == (char)240);
            assert(pack.time_boot_ms_GET() == 651243321L);
            assert(pack.thrust_GET() == 1.426406E38F);
            assert(pack.body_pitch_rate_GET() == -6.6213534E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.772615E38F, -3.1689417E38F, 1.2375717E38F, 1.7899048E38F}));
            assert(pack.body_yaw_rate_GET() == 2.6533574E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)240) ;
        p83.time_boot_ms_SET(651243321L) ;
        p83.q_SET(new float[] {-1.772615E38F, -3.1689417E38F, 1.2375717E38F, 1.7899048E38F}, 0) ;
        p83.body_pitch_rate_SET(-6.6213534E37F) ;
        p83.body_roll_rate_SET(2.1018188E38F) ;
        p83.thrust_SET(1.426406E38F) ;
        p83.body_yaw_rate_SET(2.6533574E38F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)12728);
            assert(pack.yaw_GET() == -1.8320253E38F);
            assert(pack.y_GET() == -3.610382E37F);
            assert(pack.x_GET() == 2.3140688E38F);
            assert(pack.vy_GET() == -3.0476525E38F);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.afz_GET() == -1.9393785E38F);
            assert(pack.vz_GET() == 2.4598648E38F);
            assert(pack.z_GET() == 1.882739E38F);
            assert(pack.vx_GET() == 2.5554256E38F);
            assert(pack.afy_GET() == -3.2762971E38F);
            assert(pack.yaw_rate_GET() == 2.0221595E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afx_GET() == -1.3480791E38F);
            assert(pack.time_boot_ms_GET() == 3235205229L);
            assert(pack.target_component_GET() == (char)153);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.vy_SET(-3.0476525E38F) ;
        p84.afz_SET(-1.9393785E38F) ;
        p84.z_SET(1.882739E38F) ;
        p84.yaw_SET(-1.8320253E38F) ;
        p84.yaw_rate_SET(2.0221595E38F) ;
        p84.type_mask_SET((char)12728) ;
        p84.target_system_SET((char)246) ;
        p84.afy_SET(-3.2762971E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p84.target_component_SET((char)153) ;
        p84.y_SET(-3.610382E37F) ;
        p84.x_SET(2.3140688E38F) ;
        p84.vx_SET(2.5554256E38F) ;
        p84.afx_SET(-1.3480791E38F) ;
        p84.time_boot_ms_SET(3235205229L) ;
        p84.vz_SET(2.4598648E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 3.3986449E38F);
            assert(pack.yaw_rate_GET() == 1.5928883E38F);
            assert(pack.vx_GET() == 1.6605488E38F);
            assert(pack.afy_GET() == 7.41617E36F);
            assert(pack.time_boot_ms_GET() == 4229288690L);
            assert(pack.type_mask_GET() == (char)62968);
            assert(pack.target_system_GET() == (char)139);
            assert(pack.lat_int_GET() == -2059133516);
            assert(pack.yaw_GET() == -1.311133E38F);
            assert(pack.lon_int_GET() == -1131858706);
            assert(pack.vy_GET() == -1.0456474E38F);
            assert(pack.afx_GET() == 2.91381E38F);
            assert(pack.target_component_GET() == (char)55);
            assert(pack.afz_GET() == -1.7884885E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.vz_GET() == 1.8696788E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p86.yaw_rate_SET(1.5928883E38F) ;
        p86.yaw_SET(-1.311133E38F) ;
        p86.lon_int_SET(-1131858706) ;
        p86.afz_SET(-1.7884885E38F) ;
        p86.vx_SET(1.6605488E38F) ;
        p86.alt_SET(3.3986449E38F) ;
        p86.type_mask_SET((char)62968) ;
        p86.target_system_SET((char)139) ;
        p86.vz_SET(1.8696788E38F) ;
        p86.afx_SET(2.91381E38F) ;
        p86.time_boot_ms_SET(4229288690L) ;
        p86.target_component_SET((char)55) ;
        p86.afy_SET(7.41617E36F) ;
        p86.lat_int_SET(-2059133516) ;
        p86.vy_SET(-1.0456474E38F) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)28857);
            assert(pack.afx_GET() == 2.4052054E38F);
            assert(pack.vx_GET() == -2.330036E38F);
            assert(pack.alt_GET() == -8.947226E37F);
            assert(pack.yaw_rate_GET() == 2.3365855E38F);
            assert(pack.time_boot_ms_GET() == 2051508393L);
            assert(pack.afz_GET() == 2.5283602E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.lat_int_GET() == -2110510008);
            assert(pack.afy_GET() == 3.9814892E37F);
            assert(pack.vy_GET() == -7.093137E37F);
            assert(pack.yaw_GET() == -3.0000059E38F);
            assert(pack.vz_GET() == 2.3100764E38F);
            assert(pack.lon_int_GET() == -1173611232);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vy_SET(-7.093137E37F) ;
        p87.yaw_SET(-3.0000059E38F) ;
        p87.afx_SET(2.4052054E38F) ;
        p87.alt_SET(-8.947226E37F) ;
        p87.yaw_rate_SET(2.3365855E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p87.type_mask_SET((char)28857) ;
        p87.lon_int_SET(-1173611232) ;
        p87.time_boot_ms_SET(2051508393L) ;
        p87.afy_SET(3.9814892E37F) ;
        p87.afz_SET(2.5283602E38F) ;
        p87.vz_SET(2.3100764E38F) ;
        p87.vx_SET(-2.330036E38F) ;
        p87.lat_int_SET(-2110510008) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.66437E38F);
            assert(pack.y_GET() == -1.2863195E38F);
            assert(pack.time_boot_ms_GET() == 2096927332L);
            assert(pack.x_GET() == -3.003626E38F);
            assert(pack.z_GET() == -1.1349082E38F);
            assert(pack.roll_GET() == -1.1376779E38F);
            assert(pack.yaw_GET() == -2.0965077E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.y_SET(-1.2863195E38F) ;
        p89.time_boot_ms_SET(2096927332L) ;
        p89.pitch_SET(-2.66437E38F) ;
        p89.x_SET(-3.003626E38F) ;
        p89.yaw_SET(-2.0965077E38F) ;
        p89.z_SET(-1.1349082E38F) ;
        p89.roll_SET(-1.1376779E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -468922637);
            assert(pack.yacc_GET() == (short)27647);
            assert(pack.pitch_GET() == -8.368682E37F);
            assert(pack.vz_GET() == (short) -16428);
            assert(pack.roll_GET() == 2.8421122E38F);
            assert(pack.lat_GET() == 1200035059);
            assert(pack.pitchspeed_GET() == -3.3871229E38F);
            assert(pack.rollspeed_GET() == 2.8169085E38F);
            assert(pack.alt_GET() == -182818100);
            assert(pack.time_usec_GET() == 2929261269131114408L);
            assert(pack.vy_GET() == (short) -5728);
            assert(pack.zacc_GET() == (short)28059);
            assert(pack.yaw_GET() == 2.58462E38F);
            assert(pack.xacc_GET() == (short)6620);
            assert(pack.yawspeed_GET() == -1.6733888E38F);
            assert(pack.vx_GET() == (short) -24032);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.vx_SET((short) -24032) ;
        p90.yacc_SET((short)27647) ;
        p90.vz_SET((short) -16428) ;
        p90.yaw_SET(2.58462E38F) ;
        p90.pitchspeed_SET(-3.3871229E38F) ;
        p90.xacc_SET((short)6620) ;
        p90.alt_SET(-182818100) ;
        p90.lon_SET(-468922637) ;
        p90.zacc_SET((short)28059) ;
        p90.vy_SET((short) -5728) ;
        p90.rollspeed_SET(2.8169085E38F) ;
        p90.lat_SET(1200035059) ;
        p90.time_usec_SET(2929261269131114408L) ;
        p90.yawspeed_SET(-1.6733888E38F) ;
        p90.roll_SET(2.8421122E38F) ;
        p90.pitch_SET(-8.368682E37F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.roll_ailerons_GET() == 2.9567208E38F);
            assert(pack.aux4_GET() == 1.5105852E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
            assert(pack.nav_mode_GET() == (char)26);
            assert(pack.throttle_GET() == -2.39619E38F);
            assert(pack.aux3_GET() == 1.5608218E38F);
            assert(pack.time_usec_GET() == 933155240502242789L);
            assert(pack.aux1_GET() == 2.8658294E38F);
            assert(pack.aux2_GET() == -2.6501737E37F);
            assert(pack.pitch_elevator_GET() == -1.5984442E38F);
            assert(pack.yaw_rudder_GET() == -7.5745196E37F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED) ;
        p91.time_usec_SET(933155240502242789L) ;
        p91.nav_mode_SET((char)26) ;
        p91.pitch_elevator_SET(-1.5984442E38F) ;
        p91.aux1_SET(2.8658294E38F) ;
        p91.aux4_SET(1.5105852E38F) ;
        p91.throttle_SET(-2.39619E38F) ;
        p91.yaw_rudder_SET(-7.5745196E37F) ;
        p91.roll_ailerons_SET(2.9567208E38F) ;
        p91.aux3_SET(1.5608218E38F) ;
        p91.aux2_SET(-2.6501737E37F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)39129);
            assert(pack.chan12_raw_GET() == (char)24207);
            assert(pack.chan7_raw_GET() == (char)46142);
            assert(pack.chan8_raw_GET() == (char)1097);
            assert(pack.chan10_raw_GET() == (char)58334);
            assert(pack.rssi_GET() == (char)26);
            assert(pack.chan11_raw_GET() == (char)64160);
            assert(pack.chan5_raw_GET() == (char)34210);
            assert(pack.chan6_raw_GET() == (char)37588);
            assert(pack.chan4_raw_GET() == (char)49768);
            assert(pack.chan1_raw_GET() == (char)36765);
            assert(pack.time_usec_GET() == 3517164615823541365L);
            assert(pack.chan2_raw_GET() == (char)45753);
            assert(pack.chan9_raw_GET() == (char)30682);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(3517164615823541365L) ;
        p92.chan5_raw_SET((char)34210) ;
        p92.chan9_raw_SET((char)30682) ;
        p92.chan8_raw_SET((char)1097) ;
        p92.chan2_raw_SET((char)45753) ;
        p92.rssi_SET((char)26) ;
        p92.chan12_raw_SET((char)24207) ;
        p92.chan1_raw_SET((char)36765) ;
        p92.chan3_raw_SET((char)39129) ;
        p92.chan6_raw_SET((char)37588) ;
        p92.chan7_raw_SET((char)46142) ;
        p92.chan10_raw_SET((char)58334) ;
        p92.chan11_raw_SET((char)64160) ;
        p92.chan4_raw_SET((char)49768) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 4571847285522028832L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.3924249E38F, 8.027336E37F, -1.8492918E37F, -9.988044E36F, -3.241054E38F, -2.2619886E38F, -1.8027266E36F, 1.347101E38F, -7.3773695E37F, -1.5695795E38F, 2.9237876E38F, -8.1053873E37F, 8.575913E37F, 2.9502746E38F, -1.8191593E38F, -1.747619E38F}));
            assert(pack.time_usec_GET() == 5238781226734618462L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {3.3924249E38F, 8.027336E37F, -1.8492918E37F, -9.988044E36F, -3.241054E38F, -2.2619886E38F, -1.8027266E36F, 1.347101E38F, -7.3773695E37F, -1.5695795E38F, 2.9237876E38F, -8.1053873E37F, 8.575913E37F, 2.9502746E38F, -1.8191593E38F, -1.747619E38F}, 0) ;
        p93.time_usec_SET(5238781226734618462L) ;
        p93.flags_SET(4571847285522028832L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)230);
            assert(pack.flow_comp_m_x_GET() == -2.9448053E38F);
            assert(pack.time_usec_GET() == 6628058793309085707L);
            assert(pack.sensor_id_GET() == (char)205);
            assert(pack.flow_comp_m_y_GET() == -2.9726188E38F);
            assert(pack.ground_distance_GET() == -5.903038E37F);
            assert(pack.flow_rate_y_TRY(ph) == -3.0287534E38F);
            assert(pack.flow_rate_x_TRY(ph) == -2.0321857E38F);
            assert(pack.flow_x_GET() == (short) -26692);
            assert(pack.flow_y_GET() == (short) -12868);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_rate_x_SET(-2.0321857E38F, PH) ;
        p100.ground_distance_SET(-5.903038E37F) ;
        p100.flow_comp_m_x_SET(-2.9448053E38F) ;
        p100.sensor_id_SET((char)205) ;
        p100.flow_x_SET((short) -26692) ;
        p100.flow_comp_m_y_SET(-2.9726188E38F) ;
        p100.flow_y_SET((short) -12868) ;
        p100.flow_rate_y_SET(-3.0287534E38F, PH) ;
        p100.quality_SET((char)230) ;
        p100.time_usec_SET(6628058793309085707L) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1.4555174E37F);
            assert(pack.pitch_GET() == -6.983172E37F);
            assert(pack.y_GET() == -3.7729364E37F);
            assert(pack.yaw_GET() == 1.1039507E38F);
            assert(pack.roll_GET() == 1.406919E38F);
            assert(pack.usec_GET() == 8462636939896620379L);
            assert(pack.z_GET() == 1.0860649E38F);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.y_SET(-3.7729364E37F) ;
        p101.z_SET(1.0860649E38F) ;
        p101.yaw_SET(1.1039507E38F) ;
        p101.roll_SET(1.406919E38F) ;
        p101.pitch_SET(-6.983172E37F) ;
        p101.usec_SET(8462636939896620379L) ;
        p101.x_SET(1.4555174E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 959217004923253126L);
            assert(pack.pitch_GET() == 5.818788E37F);
            assert(pack.yaw_GET() == -8.2147395E35F);
            assert(pack.y_GET() == 2.7853342E38F);
            assert(pack.roll_GET() == -1.5568437E38F);
            assert(pack.z_GET() == -2.9171944E38F);
            assert(pack.x_GET() == 7.3185176E37F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.z_SET(-2.9171944E38F) ;
        p102.usec_SET(959217004923253126L) ;
        p102.roll_SET(-1.5568437E38F) ;
        p102.x_SET(7.3185176E37F) ;
        p102.y_SET(2.7853342E38F) ;
        p102.pitch_SET(5.818788E37F) ;
        p102.yaw_SET(-8.2147395E35F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 6324766374491692958L);
            assert(pack.y_GET() == 7.111139E37F);
            assert(pack.z_GET() == -3.2454119E38F);
            assert(pack.x_GET() == -2.9884822E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(6324766374491692958L) ;
        p103.z_SET(-3.2454119E38F) ;
        p103.y_SET(7.111139E37F) ;
        p103.x_SET(-2.9884822E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.564203E38F);
            assert(pack.x_GET() == -2.7584322E38F);
            assert(pack.z_GET() == 1.5620152E38F);
            assert(pack.usec_GET() == 3102341578815914291L);
            assert(pack.roll_GET() == -8.670021E36F);
            assert(pack.y_GET() == 3.5277973E37F);
            assert(pack.pitch_GET() == 3.2517933E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.z_SET(1.5620152E38F) ;
        p104.yaw_SET(2.564203E38F) ;
        p104.usec_SET(3102341578815914291L) ;
        p104.x_SET(-2.7584322E38F) ;
        p104.y_SET(3.5277973E37F) ;
        p104.roll_SET(-8.670021E36F) ;
        p104.pitch_SET(3.2517933E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == -1.863797E38F);
            assert(pack.time_usec_GET() == 4373780470065991446L);
            assert(pack.xacc_GET() == -1.0945974E38F);
            assert(pack.xgyro_GET() == 1.4903076E38F);
            assert(pack.zmag_GET() == -8.734747E37F);
            assert(pack.xmag_GET() == -1.5866566E38F);
            assert(pack.temperature_GET() == -1.8030115E38F);
            assert(pack.zacc_GET() == 2.3307592E38F);
            assert(pack.pressure_alt_GET() == -3.074034E38F);
            assert(pack.zgyro_GET() == 3.388686E38F);
            assert(pack.ygyro_GET() == -2.2824692E38F);
            assert(pack.fields_updated_GET() == (char)41581);
            assert(pack.ymag_GET() == -2.237614E38F);
            assert(pack.abs_pressure_GET() == 4.60888E37F);
            assert(pack.yacc_GET() == 2.5390001E38F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.ygyro_SET(-2.2824692E38F) ;
        p105.xgyro_SET(1.4903076E38F) ;
        p105.temperature_SET(-1.8030115E38F) ;
        p105.xmag_SET(-1.5866566E38F) ;
        p105.time_usec_SET(4373780470065991446L) ;
        p105.yacc_SET(2.5390001E38F) ;
        p105.zmag_SET(-8.734747E37F) ;
        p105.abs_pressure_SET(4.60888E37F) ;
        p105.ymag_SET(-2.237614E38F) ;
        p105.diff_pressure_SET(-1.863797E38F) ;
        p105.xacc_SET(-1.0945974E38F) ;
        p105.zgyro_SET(3.388686E38F) ;
        p105.fields_updated_SET((char)41581) ;
        p105.zacc_SET(2.3307592E38F) ;
        p105.pressure_alt_SET(-3.074034E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integrated_xgyro_GET() == -2.7240375E38F);
            assert(pack.time_usec_GET() == 235553650179021981L);
            assert(pack.integrated_x_GET() == -2.5757482E38F);
            assert(pack.integration_time_us_GET() == 870373087L);
            assert(pack.quality_GET() == (char)171);
            assert(pack.integrated_ygyro_GET() == 9.203221E37F);
            assert(pack.time_delta_distance_us_GET() == 57814305L);
            assert(pack.sensor_id_GET() == (char)144);
            assert(pack.temperature_GET() == (short)17175);
            assert(pack.distance_GET() == 1.97468E38F);
            assert(pack.integrated_zgyro_GET() == 1.2183246E38F);
            assert(pack.integrated_y_GET() == -1.6759074E38F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(235553650179021981L) ;
        p106.integrated_ygyro_SET(9.203221E37F) ;
        p106.integrated_xgyro_SET(-2.7240375E38F) ;
        p106.integrated_y_SET(-1.6759074E38F) ;
        p106.integration_time_us_SET(870373087L) ;
        p106.integrated_x_SET(-2.5757482E38F) ;
        p106.integrated_zgyro_SET(1.2183246E38F) ;
        p106.quality_SET((char)171) ;
        p106.sensor_id_SET((char)144) ;
        p106.temperature_SET((short)17175) ;
        p106.distance_SET(1.97468E38F) ;
        p106.time_delta_distance_us_SET(57814305L) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.abs_pressure_GET() == 1.6719109E38F);
            assert(pack.ygyro_GET() == 3.1362288E38F);
            assert(pack.zacc_GET() == 1.4295261E38F);
            assert(pack.fields_updated_GET() == 278554045L);
            assert(pack.zmag_GET() == 3.1642296E37F);
            assert(pack.ymag_GET() == 1.7740369E38F);
            assert(pack.time_usec_GET() == 7327732812016131862L);
            assert(pack.yacc_GET() == 3.2144488E38F);
            assert(pack.diff_pressure_GET() == 2.9873748E38F);
            assert(pack.xgyro_GET() == -9.422873E37F);
            assert(pack.xmag_GET() == 2.9484853E38F);
            assert(pack.pressure_alt_GET() == 2.0666725E38F);
            assert(pack.temperature_GET() == -3.1451494E38F);
            assert(pack.xacc_GET() == -9.551343E37F);
            assert(pack.zgyro_GET() == 1.0037668E38F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.pressure_alt_SET(2.0666725E38F) ;
        p107.ymag_SET(1.7740369E38F) ;
        p107.xgyro_SET(-9.422873E37F) ;
        p107.xacc_SET(-9.551343E37F) ;
        p107.yacc_SET(3.2144488E38F) ;
        p107.diff_pressure_SET(2.9873748E38F) ;
        p107.time_usec_SET(7327732812016131862L) ;
        p107.zgyro_SET(1.0037668E38F) ;
        p107.ygyro_SET(3.1362288E38F) ;
        p107.zacc_SET(1.4295261E38F) ;
        p107.fields_updated_SET(278554045L) ;
        p107.zmag_SET(3.1642296E37F) ;
        p107.temperature_SET(-3.1451494E38F) ;
        p107.abs_pressure_SET(1.6719109E38F) ;
        p107.xmag_SET(2.9484853E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.5024419E38F);
            assert(pack.yacc_GET() == -2.7457424E38F);
            assert(pack.q2_GET() == -2.7791572E38F);
            assert(pack.xacc_GET() == -7.93293E37F);
            assert(pack.vn_GET() == -3.1871448E38F);
            assert(pack.xgyro_GET() == -2.5322483E38F);
            assert(pack.ve_GET() == -1.0364426E38F);
            assert(pack.ygyro_GET() == -2.8062681E38F);
            assert(pack.lat_GET() == 3.3755056E37F);
            assert(pack.q3_GET() == 2.2152401E38F);
            assert(pack.pitch_GET() == 3.175851E38F);
            assert(pack.zacc_GET() == 2.9936737E38F);
            assert(pack.roll_GET() == -1.794795E38F);
            assert(pack.q1_GET() == -7.3952106E36F);
            assert(pack.vd_GET() == 2.571631E38F);
            assert(pack.lon_GET() == 8.584023E37F);
            assert(pack.q4_GET() == 2.2965943E38F);
            assert(pack.zgyro_GET() == 6.413893E37F);
            assert(pack.std_dev_vert_GET() == 8.141444E37F);
            assert(pack.std_dev_horz_GET() == -9.13904E36F);
            assert(pack.alt_GET() == -2.0915961E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.alt_SET(-2.0915961E38F) ;
        p108.lon_SET(8.584023E37F) ;
        p108.q1_SET(-7.3952106E36F) ;
        p108.xacc_SET(-7.93293E37F) ;
        p108.q3_SET(2.2152401E38F) ;
        p108.ygyro_SET(-2.8062681E38F) ;
        p108.pitch_SET(3.175851E38F) ;
        p108.ve_SET(-1.0364426E38F) ;
        p108.std_dev_vert_SET(8.141444E37F) ;
        p108.xgyro_SET(-2.5322483E38F) ;
        p108.std_dev_horz_SET(-9.13904E36F) ;
        p108.q2_SET(-2.7791572E38F) ;
        p108.zacc_SET(2.9936737E38F) ;
        p108.roll_SET(-1.794795E38F) ;
        p108.yacc_SET(-2.7457424E38F) ;
        p108.vn_SET(-3.1871448E38F) ;
        p108.yaw_SET(2.5024419E38F) ;
        p108.vd_SET(2.571631E38F) ;
        p108.zgyro_SET(6.413893E37F) ;
        p108.lat_SET(3.3755056E37F) ;
        p108.q4_SET(2.2965943E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)203);
            assert(pack.rxerrors_GET() == (char)47825);
            assert(pack.rssi_GET() == (char)136);
            assert(pack.fixed__GET() == (char)61753);
            assert(pack.txbuf_GET() == (char)54);
            assert(pack.noise_GET() == (char)68);
            assert(pack.remrssi_GET() == (char)34);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)136) ;
        p109.remrssi_SET((char)34) ;
        p109.fixed__SET((char)61753) ;
        p109.rxerrors_SET((char)47825) ;
        p109.noise_SET((char)68) ;
        p109.txbuf_SET((char)54) ;
        p109.remnoise_SET((char)203) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)105);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)167, (char)185, (char)232, (char)212, (char)237, (char)64, (char)166, (char)114, (char)110, (char)109, (char)167, (char)65, (char)212, (char)47, (char)120, (char)75, (char)117, (char)145, (char)225, (char)242, (char)191, (char)124, (char)135, (char)59, (char)38, (char)46, (char)78, (char)163, (char)126, (char)31, (char)39, (char)32, (char)195, (char)103, (char)15, (char)85, (char)241, (char)249, (char)243, (char)34, (char)239, (char)198, (char)190, (char)101, (char)105, (char)230, (char)158, (char)31, (char)27, (char)6, (char)134, (char)77, (char)149, (char)106, (char)164, (char)164, (char)105, (char)20, (char)51, (char)105, (char)91, (char)81, (char)104, (char)213, (char)128, (char)161, (char)146, (char)116, (char)254, (char)173, (char)215, (char)20, (char)22, (char)247, (char)253, (char)124, (char)153, (char)156, (char)104, (char)80, (char)172, (char)114, (char)78, (char)229, (char)57, (char)203, (char)216, (char)44, (char)151, (char)219, (char)49, (char)11, (char)136, (char)206, (char)16, (char)140, (char)197, (char)44, (char)182, (char)55, (char)159, (char)196, (char)84, (char)4, (char)30, (char)134, (char)100, (char)111, (char)246, (char)119, (char)151, (char)162, (char)82, (char)129, (char)57, (char)201, (char)40, (char)178, (char)189, (char)202, (char)77, (char)78, (char)69, (char)211, (char)69, (char)132, (char)114, (char)143, (char)125, (char)253, (char)65, (char)55, (char)158, (char)90, (char)231, (char)87, (char)2, (char)181, (char)39, (char)178, (char)248, (char)165, (char)212, (char)62, (char)67, (char)253, (char)34, (char)109, (char)136, (char)143, (char)200, (char)193, (char)92, (char)219, (char)9, (char)85, (char)250, (char)27, (char)246, (char)211, (char)208, (char)55, (char)131, (char)149, (char)131, (char)209, (char)245, (char)143, (char)232, (char)24, (char)172, (char)200, (char)198, (char)174, (char)162, (char)8, (char)173, (char)32, (char)29, (char)49, (char)55, (char)87, (char)31, (char)56, (char)59, (char)83, (char)1, (char)210, (char)12, (char)125, (char)145, (char)83, (char)194, (char)145, (char)164, (char)105, (char)90, (char)131, (char)228, (char)223, (char)218, (char)236, (char)101, (char)35, (char)126, (char)238, (char)9, (char)209, (char)10, (char)37, (char)33, (char)194, (char)5, (char)179, (char)107, (char)125, (char)204, (char)7, (char)95, (char)189, (char)92, (char)156, (char)193, (char)245, (char)30, (char)239, (char)17, (char)125, (char)172, (char)47, (char)89, (char)206, (char)142, (char)8, (char)48, (char)204, (char)85, (char)169, (char)161, (char)89, (char)11, (char)209, (char)43, (char)114, (char)200, (char)123, (char)83, (char)175, (char)237, (char)91, (char)34}));
            assert(pack.target_network_GET() == (char)15);
            assert(pack.target_system_GET() == (char)60);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)105) ;
        p110.target_system_SET((char)60) ;
        p110.payload_SET(new char[] {(char)167, (char)185, (char)232, (char)212, (char)237, (char)64, (char)166, (char)114, (char)110, (char)109, (char)167, (char)65, (char)212, (char)47, (char)120, (char)75, (char)117, (char)145, (char)225, (char)242, (char)191, (char)124, (char)135, (char)59, (char)38, (char)46, (char)78, (char)163, (char)126, (char)31, (char)39, (char)32, (char)195, (char)103, (char)15, (char)85, (char)241, (char)249, (char)243, (char)34, (char)239, (char)198, (char)190, (char)101, (char)105, (char)230, (char)158, (char)31, (char)27, (char)6, (char)134, (char)77, (char)149, (char)106, (char)164, (char)164, (char)105, (char)20, (char)51, (char)105, (char)91, (char)81, (char)104, (char)213, (char)128, (char)161, (char)146, (char)116, (char)254, (char)173, (char)215, (char)20, (char)22, (char)247, (char)253, (char)124, (char)153, (char)156, (char)104, (char)80, (char)172, (char)114, (char)78, (char)229, (char)57, (char)203, (char)216, (char)44, (char)151, (char)219, (char)49, (char)11, (char)136, (char)206, (char)16, (char)140, (char)197, (char)44, (char)182, (char)55, (char)159, (char)196, (char)84, (char)4, (char)30, (char)134, (char)100, (char)111, (char)246, (char)119, (char)151, (char)162, (char)82, (char)129, (char)57, (char)201, (char)40, (char)178, (char)189, (char)202, (char)77, (char)78, (char)69, (char)211, (char)69, (char)132, (char)114, (char)143, (char)125, (char)253, (char)65, (char)55, (char)158, (char)90, (char)231, (char)87, (char)2, (char)181, (char)39, (char)178, (char)248, (char)165, (char)212, (char)62, (char)67, (char)253, (char)34, (char)109, (char)136, (char)143, (char)200, (char)193, (char)92, (char)219, (char)9, (char)85, (char)250, (char)27, (char)246, (char)211, (char)208, (char)55, (char)131, (char)149, (char)131, (char)209, (char)245, (char)143, (char)232, (char)24, (char)172, (char)200, (char)198, (char)174, (char)162, (char)8, (char)173, (char)32, (char)29, (char)49, (char)55, (char)87, (char)31, (char)56, (char)59, (char)83, (char)1, (char)210, (char)12, (char)125, (char)145, (char)83, (char)194, (char)145, (char)164, (char)105, (char)90, (char)131, (char)228, (char)223, (char)218, (char)236, (char)101, (char)35, (char)126, (char)238, (char)9, (char)209, (char)10, (char)37, (char)33, (char)194, (char)5, (char)179, (char)107, (char)125, (char)204, (char)7, (char)95, (char)189, (char)92, (char)156, (char)193, (char)245, (char)30, (char)239, (char)17, (char)125, (char)172, (char)47, (char)89, (char)206, (char)142, (char)8, (char)48, (char)204, (char)85, (char)169, (char)161, (char)89, (char)11, (char)209, (char)43, (char)114, (char)200, (char)123, (char)83, (char)175, (char)237, (char)91, (char)34}, 0) ;
        p110.target_network_SET((char)15) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -3388188461854469705L);
            assert(pack.ts1_GET() == 3549330099624288088L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-3388188461854469705L) ;
        p111.ts1_SET(3549330099624288088L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 1259518608758007105L);
            assert(pack.seq_GET() == 730308037L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(1259518608758007105L) ;
        p112.seq_SET(730308037L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)59);
            assert(pack.ve_GET() == (short)23745);
            assert(pack.lat_GET() == 1702788158);
            assert(pack.vd_GET() == (short)22995);
            assert(pack.vel_GET() == (char)13974);
            assert(pack.time_usec_GET() == 8928340370479798665L);
            assert(pack.eph_GET() == (char)55908);
            assert(pack.lon_GET() == -1584015559);
            assert(pack.alt_GET() == 823263377);
            assert(pack.vn_GET() == (short)28777);
            assert(pack.cog_GET() == (char)16670);
            assert(pack.epv_GET() == (char)2623);
            assert(pack.fix_type_GET() == (char)55);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(1702788158) ;
        p113.vd_SET((short)22995) ;
        p113.fix_type_SET((char)55) ;
        p113.eph_SET((char)55908) ;
        p113.cog_SET((char)16670) ;
        p113.epv_SET((char)2623) ;
        p113.lon_SET(-1584015559) ;
        p113.vn_SET((short)28777) ;
        p113.ve_SET((short)23745) ;
        p113.alt_SET(823263377) ;
        p113.vel_SET((char)13974) ;
        p113.time_usec_SET(8928340370479798665L) ;
        p113.satellites_visible_SET((char)59) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_y_GET() == 1.0823203E38F);
            assert(pack.integrated_ygyro_GET() == 1.881621E38F);
            assert(pack.integrated_zgyro_GET() == -2.2060982E38F);
            assert(pack.integration_time_us_GET() == 1390973830L);
            assert(pack.integrated_x_GET() == 2.3935904E38F);
            assert(pack.time_delta_distance_us_GET() == 599983529L);
            assert(pack.sensor_id_GET() == (char)70);
            assert(pack.time_usec_GET() == 5755303963590878689L);
            assert(pack.distance_GET() == -1.0153548E38F);
            assert(pack.quality_GET() == (char)249);
            assert(pack.temperature_GET() == (short) -31226);
            assert(pack.integrated_xgyro_GET() == -1.588697E38F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short) -31226) ;
        p114.integrated_xgyro_SET(-1.588697E38F) ;
        p114.integrated_y_SET(1.0823203E38F) ;
        p114.integrated_zgyro_SET(-2.2060982E38F) ;
        p114.sensor_id_SET((char)70) ;
        p114.distance_SET(-1.0153548E38F) ;
        p114.quality_SET((char)249) ;
        p114.time_delta_distance_us_SET(599983529L) ;
        p114.integrated_ygyro_SET(1.881621E38F) ;
        p114.integrated_x_SET(2.3935904E38F) ;
        p114.integration_time_us_SET(1390973830L) ;
        p114.time_usec_SET(5755303963590878689L) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.5245112E38F, -2.0116493E38F, -2.3454775E38F, -1.7945235E38F}));
            assert(pack.vy_GET() == (short) -29923);
            assert(pack.vx_GET() == (short)25111);
            assert(pack.lon_GET() == 2055156948);
            assert(pack.yacc_GET() == (short) -10746);
            assert(pack.time_usec_GET() == 977749725405735510L);
            assert(pack.xacc_GET() == (short)22962);
            assert(pack.zacc_GET() == (short)21754);
            assert(pack.ind_airspeed_GET() == (char)39795);
            assert(pack.yawspeed_GET() == 2.2159932E38F);
            assert(pack.alt_GET() == -31346637);
            assert(pack.true_airspeed_GET() == (char)58712);
            assert(pack.rollspeed_GET() == -1.6188857E38F);
            assert(pack.lat_GET() == -588260103);
            assert(pack.vz_GET() == (short) -19054);
            assert(pack.pitchspeed_GET() == -9.346344E37F);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vz_SET((short) -19054) ;
        p115.lon_SET(2055156948) ;
        p115.xacc_SET((short)22962) ;
        p115.time_usec_SET(977749725405735510L) ;
        p115.lat_SET(-588260103) ;
        p115.vy_SET((short) -29923) ;
        p115.yacc_SET((short) -10746) ;
        p115.zacc_SET((short)21754) ;
        p115.alt_SET(-31346637) ;
        p115.vx_SET((short)25111) ;
        p115.pitchspeed_SET(-9.346344E37F) ;
        p115.yawspeed_SET(2.2159932E38F) ;
        p115.rollspeed_SET(-1.6188857E38F) ;
        p115.ind_airspeed_SET((char)39795) ;
        p115.attitude_quaternion_SET(new float[] {1.5245112E38F, -2.0116493E38F, -2.3454775E38F, -1.7945235E38F}, 0) ;
        p115.true_airspeed_SET((char)58712) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)20342);
            assert(pack.zacc_GET() == (short)27711);
            assert(pack.yacc_GET() == (short) -28240);
            assert(pack.xmag_GET() == (short)32499);
            assert(pack.zgyro_GET() == (short) -31940);
            assert(pack.time_boot_ms_GET() == 3787120845L);
            assert(pack.ymag_GET() == (short) -2856);
            assert(pack.ygyro_GET() == (short)3653);
            assert(pack.zmag_GET() == (short)17598);
            assert(pack.xgyro_GET() == (short)27811);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short)27811) ;
        p116.ygyro_SET((short)3653) ;
        p116.time_boot_ms_SET(3787120845L) ;
        p116.ymag_SET((short) -2856) ;
        p116.zgyro_SET((short) -31940) ;
        p116.zacc_SET((short)27711) ;
        p116.xmag_SET((short)32499) ;
        p116.xacc_SET((short)20342) ;
        p116.zmag_SET((short)17598) ;
        p116.yacc_SET((short) -28240) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)88);
            assert(pack.start_GET() == (char)13207);
            assert(pack.target_system_GET() == (char)52);
            assert(pack.end_GET() == (char)4997);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)13207) ;
        p117.target_component_SET((char)88) ;
        p117.target_system_SET((char)52) ;
        p117.end_SET((char)4997) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)39126);
            assert(pack.time_utc_GET() == 879716359L);
            assert(pack.size_GET() == 3337601917L);
            assert(pack.last_log_num_GET() == (char)21248);
            assert(pack.id_GET() == (char)12692);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.last_log_num_SET((char)21248) ;
        p118.num_logs_SET((char)39126) ;
        p118.size_SET(3337601917L) ;
        p118.id_SET((char)12692) ;
        p118.time_utc_SET(879716359L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)7);
            assert(pack.target_system_GET() == (char)246);
            assert(pack.count_GET() == 1083656248L);
            assert(pack.id_GET() == (char)49464);
            assert(pack.ofs_GET() == 2898028172L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(2898028172L) ;
        p119.target_component_SET((char)7) ;
        p119.target_system_SET((char)246) ;
        p119.id_SET((char)49464) ;
        p119.count_SET(1083656248L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 1839053064L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)181, (char)200, (char)158, (char)82, (char)165, (char)80, (char)163, (char)215, (char)51, (char)129, (char)176, (char)175, (char)98, (char)224, (char)3, (char)24, (char)34, (char)1, (char)149, (char)27, (char)43, (char)83, (char)39, (char)246, (char)161, (char)3, (char)38, (char)188, (char)189, (char)243, (char)166, (char)86, (char)209, (char)93, (char)175, (char)31, (char)81, (char)231, (char)219, (char)148, (char)168, (char)240, (char)223, (char)251, (char)3, (char)35, (char)208, (char)56, (char)158, (char)232, (char)209, (char)58, (char)33, (char)152, (char)158, (char)65, (char)82, (char)227, (char)125, (char)117, (char)44, (char)31, (char)218, (char)134, (char)198, (char)244, (char)48, (char)147, (char)5, (char)123, (char)18, (char)144, (char)47, (char)104, (char)22, (char)197, (char)76, (char)158, (char)169, (char)192, (char)199, (char)121, (char)214, (char)7, (char)43, (char)67, (char)101, (char)4, (char)122, (char)169}));
            assert(pack.count_GET() == (char)192);
            assert(pack.id_GET() == (char)2876);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)181, (char)200, (char)158, (char)82, (char)165, (char)80, (char)163, (char)215, (char)51, (char)129, (char)176, (char)175, (char)98, (char)224, (char)3, (char)24, (char)34, (char)1, (char)149, (char)27, (char)43, (char)83, (char)39, (char)246, (char)161, (char)3, (char)38, (char)188, (char)189, (char)243, (char)166, (char)86, (char)209, (char)93, (char)175, (char)31, (char)81, (char)231, (char)219, (char)148, (char)168, (char)240, (char)223, (char)251, (char)3, (char)35, (char)208, (char)56, (char)158, (char)232, (char)209, (char)58, (char)33, (char)152, (char)158, (char)65, (char)82, (char)227, (char)125, (char)117, (char)44, (char)31, (char)218, (char)134, (char)198, (char)244, (char)48, (char)147, (char)5, (char)123, (char)18, (char)144, (char)47, (char)104, (char)22, (char)197, (char)76, (char)158, (char)169, (char)192, (char)199, (char)121, (char)214, (char)7, (char)43, (char)67, (char)101, (char)4, (char)122, (char)169}, 0) ;
        p120.count_SET((char)192) ;
        p120.id_SET((char)2876) ;
        p120.ofs_SET(1839053064L) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)119);
            assert(pack.target_system_GET() == (char)149);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)149) ;
        p121.target_component_SET((char)119) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)201);
            assert(pack.target_component_GET() == (char)178);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)201) ;
        p122.target_component_SET((char)178) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)155);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)196, (char)119, (char)133, (char)29, (char)129, (char)155, (char)120, (char)213, (char)132, (char)75, (char)236, (char)6, (char)114, (char)50, (char)12, (char)217, (char)91, (char)153, (char)76, (char)40, (char)178, (char)149, (char)237, (char)118, (char)16, (char)90, (char)211, (char)224, (char)146, (char)201, (char)132, (char)101, (char)35, (char)236, (char)120, (char)189, (char)214, (char)156, (char)57, (char)119, (char)219, (char)166, (char)169, (char)82, (char)108, (char)73, (char)194, (char)68, (char)253, (char)54, (char)109, (char)145, (char)15, (char)78, (char)155, (char)88, (char)70, (char)59, (char)126, (char)205, (char)238, (char)156, (char)193, (char)206, (char)22, (char)202, (char)189, (char)48, (char)41, (char)13, (char)124, (char)191, (char)214, (char)175, (char)123, (char)62, (char)98, (char)120, (char)90, (char)122, (char)42, (char)176, (char)209, (char)113, (char)255, (char)93, (char)174, (char)243, (char)233, (char)194, (char)253, (char)136, (char)247, (char)232, (char)82, (char)240, (char)184, (char)71, (char)93, (char)228, (char)235, (char)116, (char)55, (char)219, (char)39, (char)137, (char)147, (char)60, (char)5, (char)170}));
            assert(pack.target_component_GET() == (char)80);
            assert(pack.target_system_GET() == (char)254);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.data__SET(new char[] {(char)196, (char)119, (char)133, (char)29, (char)129, (char)155, (char)120, (char)213, (char)132, (char)75, (char)236, (char)6, (char)114, (char)50, (char)12, (char)217, (char)91, (char)153, (char)76, (char)40, (char)178, (char)149, (char)237, (char)118, (char)16, (char)90, (char)211, (char)224, (char)146, (char)201, (char)132, (char)101, (char)35, (char)236, (char)120, (char)189, (char)214, (char)156, (char)57, (char)119, (char)219, (char)166, (char)169, (char)82, (char)108, (char)73, (char)194, (char)68, (char)253, (char)54, (char)109, (char)145, (char)15, (char)78, (char)155, (char)88, (char)70, (char)59, (char)126, (char)205, (char)238, (char)156, (char)193, (char)206, (char)22, (char)202, (char)189, (char)48, (char)41, (char)13, (char)124, (char)191, (char)214, (char)175, (char)123, (char)62, (char)98, (char)120, (char)90, (char)122, (char)42, (char)176, (char)209, (char)113, (char)255, (char)93, (char)174, (char)243, (char)233, (char)194, (char)253, (char)136, (char)247, (char)232, (char)82, (char)240, (char)184, (char)71, (char)93, (char)228, (char)235, (char)116, (char)55, (char)219, (char)39, (char)137, (char)147, (char)60, (char)5, (char)170}, 0) ;
        p123.target_system_SET((char)254) ;
        p123.target_component_SET((char)80) ;
        p123.len_SET((char)155) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -822876319);
            assert(pack.alt_GET() == 1934579325);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.epv_GET() == (char)5221);
            assert(pack.time_usec_GET() == 6399630134747672471L);
            assert(pack.dgps_numch_GET() == (char)175);
            assert(pack.eph_GET() == (char)22256);
            assert(pack.satellites_visible_GET() == (char)149);
            assert(pack.vel_GET() == (char)12263);
            assert(pack.cog_GET() == (char)55959);
            assert(pack.dgps_age_GET() == 1909015976L);
            assert(pack.lon_GET() == 1040772689);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.dgps_numch_SET((char)175) ;
        p124.time_usec_SET(6399630134747672471L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p124.lon_SET(1040772689) ;
        p124.satellites_visible_SET((char)149) ;
        p124.alt_SET(1934579325) ;
        p124.dgps_age_SET(1909015976L) ;
        p124.cog_SET((char)55959) ;
        p124.lat_SET(-822876319) ;
        p124.eph_SET((char)22256) ;
        p124.epv_SET((char)5221) ;
        p124.vel_SET((char)12263) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)26295);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            assert(pack.Vcc_GET() == (char)25286);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)25286) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID) ;
        p125.Vservo_SET((char)26295) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)9044);
            assert(pack.count_GET() == (char)218);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)97, (char)57, (char)222, (char)51, (char)97, (char)79, (char)131, (char)79, (char)1, (char)35, (char)35, (char)139, (char)129, (char)120, (char)130, (char)77, (char)140, (char)199, (char)49, (char)77, (char)0, (char)79, (char)209, (char)25, (char)157, (char)199, (char)220, (char)143, (char)36, (char)144, (char)254, (char)194, (char)237, (char)244, (char)61, (char)188, (char)128, (char)232, (char)73, (char)62, (char)46, (char)191, (char)216, (char)234, (char)233, (char)119, (char)250, (char)166, (char)17, (char)218, (char)95, (char)120, (char)166, (char)185, (char)36, (char)64, (char)35, (char)41, (char)246, (char)31, (char)219, (char)61, (char)84, (char)5, (char)239, (char)142, (char)143, (char)97, (char)205, (char)221}));
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            assert(pack.baudrate_GET() == 3496780258L);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)218) ;
        p126.timeout_SET((char)9044) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2) ;
        p126.data__SET(new char[] {(char)97, (char)57, (char)222, (char)51, (char)97, (char)79, (char)131, (char)79, (char)1, (char)35, (char)35, (char)139, (char)129, (char)120, (char)130, (char)77, (char)140, (char)199, (char)49, (char)77, (char)0, (char)79, (char)209, (char)25, (char)157, (char)199, (char)220, (char)143, (char)36, (char)144, (char)254, (char)194, (char)237, (char)244, (char)61, (char)188, (char)128, (char)232, (char)73, (char)62, (char)46, (char)191, (char)216, (char)234, (char)233, (char)119, (char)250, (char)166, (char)17, (char)218, (char)95, (char)120, (char)166, (char)185, (char)36, (char)64, (char)35, (char)41, (char)246, (char)31, (char)219, (char)61, (char)84, (char)5, (char)239, (char)142, (char)143, (char)97, (char)205, (char)221}, 0) ;
        p126.baudrate_SET(3496780258L) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)118);
            assert(pack.accuracy_GET() == 1217596428L);
            assert(pack.wn_GET() == (char)39551);
            assert(pack.baseline_b_mm_GET() == 2014454160);
            assert(pack.baseline_a_mm_GET() == 1110825398);
            assert(pack.baseline_coords_type_GET() == (char)86);
            assert(pack.iar_num_hypotheses_GET() == -564670546);
            assert(pack.rtk_health_GET() == (char)95);
            assert(pack.baseline_c_mm_GET() == -671675273);
            assert(pack.rtk_rate_GET() == (char)214);
            assert(pack.tow_GET() == 2311864461L);
            assert(pack.time_last_baseline_ms_GET() == 1793557743L);
            assert(pack.rtk_receiver_id_GET() == (char)248);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(1793557743L) ;
        p127.baseline_c_mm_SET(-671675273) ;
        p127.accuracy_SET(1217596428L) ;
        p127.iar_num_hypotheses_SET(-564670546) ;
        p127.rtk_health_SET((char)95) ;
        p127.baseline_a_mm_SET(1110825398) ;
        p127.tow_SET(2311864461L) ;
        p127.baseline_b_mm_SET(2014454160) ;
        p127.nsats_SET((char)118) ;
        p127.wn_SET((char)39551) ;
        p127.rtk_rate_SET((char)214) ;
        p127.baseline_coords_type_SET((char)86) ;
        p127.rtk_receiver_id_SET((char)248) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)96);
            assert(pack.rtk_receiver_id_GET() == (char)39);
            assert(pack.baseline_a_mm_GET() == -1956746916);
            assert(pack.tow_GET() == 805560073L);
            assert(pack.time_last_baseline_ms_GET() == 1121221582L);
            assert(pack.wn_GET() == (char)6237);
            assert(pack.baseline_b_mm_GET() == -1639118617);
            assert(pack.baseline_coords_type_GET() == (char)244);
            assert(pack.accuracy_GET() == 2804485864L);
            assert(pack.rtk_rate_GET() == (char)208);
            assert(pack.baseline_c_mm_GET() == 1194212944);
            assert(pack.iar_num_hypotheses_GET() == 887710643);
            assert(pack.rtk_health_GET() == (char)8);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_a_mm_SET(-1956746916) ;
        p128.tow_SET(805560073L) ;
        p128.rtk_rate_SET((char)208) ;
        p128.baseline_b_mm_SET(-1639118617) ;
        p128.baseline_c_mm_SET(1194212944) ;
        p128.rtk_health_SET((char)8) ;
        p128.accuracy_SET(2804485864L) ;
        p128.wn_SET((char)6237) ;
        p128.baseline_coords_type_SET((char)244) ;
        p128.iar_num_hypotheses_SET(887710643) ;
        p128.time_last_baseline_ms_SET(1121221582L) ;
        p128.nsats_SET((char)96) ;
        p128.rtk_receiver_id_SET((char)39) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short) -23415);
            assert(pack.yacc_GET() == (short) -14606);
            assert(pack.zgyro_GET() == (short) -6193);
            assert(pack.zacc_GET() == (short)9422);
            assert(pack.xacc_GET() == (short)16765);
            assert(pack.xmag_GET() == (short)20662);
            assert(pack.xgyro_GET() == (short)15342);
            assert(pack.ygyro_GET() == (short)26016);
            assert(pack.time_boot_ms_GET() == 3286292895L);
            assert(pack.zmag_GET() == (short) -31582);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zacc_SET((short)9422) ;
        p129.xacc_SET((short)16765) ;
        p129.time_boot_ms_SET(3286292895L) ;
        p129.ymag_SET((short) -23415) ;
        p129.zgyro_SET((short) -6193) ;
        p129.ygyro_SET((short)26016) ;
        p129.xmag_SET((short)20662) ;
        p129.zmag_SET((short) -31582) ;
        p129.xgyro_SET((short)15342) ;
        p129.yacc_SET((short) -14606) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 1238292910L);
            assert(pack.packets_GET() == (char)46172);
            assert(pack.height_GET() == (char)1571);
            assert(pack.type_GET() == (char)203);
            assert(pack.width_GET() == (char)31503);
            assert(pack.jpg_quality_GET() == (char)199);
            assert(pack.payload_GET() == (char)85);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)46172) ;
        p130.type_SET((char)203) ;
        p130.jpg_quality_SET((char)199) ;
        p130.payload_SET((char)85) ;
        p130.height_SET((char)1571) ;
        p130.width_SET((char)31503) ;
        p130.size_SET(1238292910L) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)34162);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)243, (char)208, (char)98, (char)23, (char)226, (char)146, (char)228, (char)228, (char)41, (char)18, (char)136, (char)95, (char)35, (char)153, (char)248, (char)237, (char)97, (char)59, (char)142, (char)18, (char)33, (char)34, (char)158, (char)109, (char)65, (char)232, (char)173, (char)102, (char)168, (char)53, (char)123, (char)136, (char)73, (char)183, (char)13, (char)70, (char)243, (char)108, (char)135, (char)61, (char)28, (char)247, (char)82, (char)142, (char)146, (char)242, (char)59, (char)137, (char)142, (char)19, (char)219, (char)39, (char)77, (char)29, (char)203, (char)43, (char)246, (char)21, (char)180, (char)198, (char)9, (char)14, (char)144, (char)41, (char)17, (char)180, (char)94, (char)28, (char)146, (char)96, (char)48, (char)19, (char)174, (char)49, (char)176, (char)147, (char)36, (char)64, (char)98, (char)153, (char)252, (char)180, (char)39, (char)176, (char)200, (char)233, (char)54, (char)51, (char)121, (char)200, (char)94, (char)234, (char)134, (char)139, (char)244, (char)121, (char)177, (char)13, (char)11, (char)16, (char)169, (char)135, (char)241, (char)19, (char)173, (char)78, (char)238, (char)208, (char)234, (char)192, (char)9, (char)55, (char)207, (char)90, (char)169, (char)6, (char)233, (char)199, (char)11, (char)81, (char)126, (char)82, (char)69, (char)99, (char)39, (char)187, (char)106, (char)180, (char)131, (char)180, (char)53, (char)118, (char)162, (char)230, (char)214, (char)41, (char)0, (char)60, (char)146, (char)101, (char)139, (char)137, (char)178, (char)143, (char)76, (char)209, (char)140, (char)120, (char)34, (char)140, (char)164, (char)87, (char)90, (char)13, (char)150, (char)77, (char)127, (char)189, (char)105, (char)252, (char)44, (char)234, (char)247, (char)110, (char)215, (char)86, (char)27, (char)41, (char)28, (char)186, (char)126, (char)218, (char)149, (char)89, (char)251, (char)174, (char)181, (char)75, (char)1, (char)178, (char)34, (char)105, (char)162, (char)236, (char)169, (char)144, (char)222, (char)196, (char)128, (char)17, (char)182, (char)177, (char)155, (char)84, (char)161, (char)71, (char)12, (char)204, (char)89, (char)198, (char)80, (char)167, (char)241, (char)205, (char)143, (char)63, (char)22, (char)15, (char)253, (char)144, (char)185, (char)174, (char)94, (char)211, (char)43, (char)102, (char)7, (char)108, (char)66, (char)171, (char)47, (char)86, (char)147, (char)93, (char)55, (char)130, (char)127, (char)9, (char)211, (char)2, (char)63, (char)29, (char)191, (char)84, (char)81, (char)122, (char)7, (char)20, (char)153, (char)15, (char)27, (char)79, (char)120, (char)140, (char)119, (char)216, (char)38, (char)149, (char)216, (char)169, (char)10, (char)126, (char)183}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)34162) ;
        p131.data__SET(new char[] {(char)243, (char)208, (char)98, (char)23, (char)226, (char)146, (char)228, (char)228, (char)41, (char)18, (char)136, (char)95, (char)35, (char)153, (char)248, (char)237, (char)97, (char)59, (char)142, (char)18, (char)33, (char)34, (char)158, (char)109, (char)65, (char)232, (char)173, (char)102, (char)168, (char)53, (char)123, (char)136, (char)73, (char)183, (char)13, (char)70, (char)243, (char)108, (char)135, (char)61, (char)28, (char)247, (char)82, (char)142, (char)146, (char)242, (char)59, (char)137, (char)142, (char)19, (char)219, (char)39, (char)77, (char)29, (char)203, (char)43, (char)246, (char)21, (char)180, (char)198, (char)9, (char)14, (char)144, (char)41, (char)17, (char)180, (char)94, (char)28, (char)146, (char)96, (char)48, (char)19, (char)174, (char)49, (char)176, (char)147, (char)36, (char)64, (char)98, (char)153, (char)252, (char)180, (char)39, (char)176, (char)200, (char)233, (char)54, (char)51, (char)121, (char)200, (char)94, (char)234, (char)134, (char)139, (char)244, (char)121, (char)177, (char)13, (char)11, (char)16, (char)169, (char)135, (char)241, (char)19, (char)173, (char)78, (char)238, (char)208, (char)234, (char)192, (char)9, (char)55, (char)207, (char)90, (char)169, (char)6, (char)233, (char)199, (char)11, (char)81, (char)126, (char)82, (char)69, (char)99, (char)39, (char)187, (char)106, (char)180, (char)131, (char)180, (char)53, (char)118, (char)162, (char)230, (char)214, (char)41, (char)0, (char)60, (char)146, (char)101, (char)139, (char)137, (char)178, (char)143, (char)76, (char)209, (char)140, (char)120, (char)34, (char)140, (char)164, (char)87, (char)90, (char)13, (char)150, (char)77, (char)127, (char)189, (char)105, (char)252, (char)44, (char)234, (char)247, (char)110, (char)215, (char)86, (char)27, (char)41, (char)28, (char)186, (char)126, (char)218, (char)149, (char)89, (char)251, (char)174, (char)181, (char)75, (char)1, (char)178, (char)34, (char)105, (char)162, (char)236, (char)169, (char)144, (char)222, (char)196, (char)128, (char)17, (char)182, (char)177, (char)155, (char)84, (char)161, (char)71, (char)12, (char)204, (char)89, (char)198, (char)80, (char)167, (char)241, (char)205, (char)143, (char)63, (char)22, (char)15, (char)253, (char)144, (char)185, (char)174, (char)94, (char)211, (char)43, (char)102, (char)7, (char)108, (char)66, (char)171, (char)47, (char)86, (char)147, (char)93, (char)55, (char)130, (char)127, (char)9, (char)211, (char)2, (char)63, (char)29, (char)191, (char)84, (char)81, (char)122, (char)7, (char)20, (char)153, (char)15, (char)27, (char)79, (char)120, (char)140, (char)119, (char)216, (char)38, (char)149, (char)216, (char)169, (char)10, (char)126, (char)183}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.current_distance_GET() == (char)27277);
            assert(pack.time_boot_ms_GET() == 2881074588L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
            assert(pack.min_distance_GET() == (char)45497);
            assert(pack.max_distance_GET() == (char)10405);
            assert(pack.id_GET() == (char)224);
            assert(pack.covariance_GET() == (char)187);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.covariance_SET((char)187) ;
        p132.id_SET((char)224) ;
        p132.current_distance_SET((char)27277) ;
        p132.min_distance_SET((char)45497) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.max_distance_SET((char)10405) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_270) ;
        p132.time_boot_ms_SET(2881074588L) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1203399557);
            assert(pack.lon_GET() == -1285207344);
            assert(pack.mask_GET() == 5459782380879874788L);
            assert(pack.grid_spacing_GET() == (char)36407);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-1285207344) ;
        p133.lat_SET(-1203399557) ;
        p133.grid_spacing_SET((char)36407) ;
        p133.mask_SET(5459782380879874788L) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -1901, (short) -14431, (short) -1676, (short) -25893, (short) -24741, (short)11049, (short)3817, (short) -2161, (short)9687, (short) -9855, (short) -30920, (short)11732, (short) -5233, (short)11377, (short) -14164, (short)10613}));
            assert(pack.lon_GET() == -599972919);
            assert(pack.grid_spacing_GET() == (char)14649);
            assert(pack.gridbit_GET() == (char)70);
            assert(pack.lat_GET() == 2009647190);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.data__SET(new short[] {(short) -1901, (short) -14431, (short) -1676, (short) -25893, (short) -24741, (short)11049, (short)3817, (short) -2161, (short)9687, (short) -9855, (short) -30920, (short)11732, (short) -5233, (short)11377, (short) -14164, (short)10613}, 0) ;
        p134.gridbit_SET((char)70) ;
        p134.lat_SET(2009647190) ;
        p134.lon_SET(-599972919) ;
        p134.grid_spacing_SET((char)14649) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -677168924);
            assert(pack.lon_GET() == -1806613095);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-1806613095) ;
        p135.lat_SET(-677168924) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == -3.9891066E36F);
            assert(pack.lat_GET() == -1858388098);
            assert(pack.lon_GET() == -1165862384);
            assert(pack.current_height_GET() == 6.2677067E37F);
            assert(pack.pending_GET() == (char)33650);
            assert(pack.loaded_GET() == (char)58925);
            assert(pack.spacing_GET() == (char)27927);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lon_SET(-1165862384) ;
        p136.loaded_SET((char)58925) ;
        p136.spacing_SET((char)27927) ;
        p136.lat_SET(-1858388098) ;
        p136.terrain_height_SET(-3.9891066E36F) ;
        p136.pending_SET((char)33650) ;
        p136.current_height_SET(6.2677067E37F) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -9454);
            assert(pack.time_boot_ms_GET() == 3166005210L);
            assert(pack.press_diff_GET() == -2.4945693E37F);
            assert(pack.press_abs_GET() == -2.6644024E38F);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3166005210L) ;
        p137.press_diff_SET(-2.4945693E37F) ;
        p137.temperature_SET((short) -9454) ;
        p137.press_abs_SET(-2.6644024E38F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 2.1138597E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.9167218E38F, 2.5702374E37F, -1.0430112E38F, -2.8491435E38F}));
            assert(pack.x_GET() == 7.175039E36F);
            assert(pack.time_usec_GET() == 6439095358270496407L);
            assert(pack.z_GET() == -2.7506608E37F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(7.175039E36F) ;
        p138.time_usec_SET(6439095358270496407L) ;
        p138.q_SET(new float[] {-1.9167218E38F, 2.5702374E37F, -1.0430112E38F, -2.8491435E38F}, 0) ;
        p138.y_SET(2.1138597E38F) ;
        p138.z_SET(-2.7506608E37F) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)115);
            assert(pack.time_usec_GET() == 7031799938249198820L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.6770067E37F, 2.6851377E38F, -2.5158084E38F, 2.7867376E38F, -1.0783189E37F, -3.2020014E38F, 1.0855068E38F, -1.7942417E37F}));
            assert(pack.target_component_GET() == (char)205);
            assert(pack.target_system_GET() == (char)54);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)115) ;
        p139.target_component_SET((char)205) ;
        p139.time_usec_SET(7031799938249198820L) ;
        p139.target_system_SET((char)54) ;
        p139.controls_SET(new float[] {3.6770067E37F, 2.6851377E38F, -2.5158084E38F, 2.7867376E38F, -1.0783189E37F, -3.2020014E38F, 1.0855068E38F, -1.7942417E37F}, 0) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-8.2032703E37F, 3.241619E38F, -1.3126964E38F, 4.6744494E37F, 5.0464156E37F, 2.6851362E37F, 1.894684E38F, -8.125698E37F}));
            assert(pack.group_mlx_GET() == (char)2);
            assert(pack.time_usec_GET() == 7328962065038814519L);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(7328962065038814519L) ;
        p140.controls_SET(new float[] {-8.2032703E37F, 3.241619E38F, -1.3126964E38F, 4.6744494E37F, 5.0464156E37F, 2.6851362E37F, 1.894684E38F, -8.125698E37F}, 0) ;
        p140.group_mlx_SET((char)2) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == 1.0880221E38F);
            assert(pack.bottom_clearance_GET() == -3.363318E38F);
            assert(pack.time_usec_GET() == 6064763846987536360L);
            assert(pack.altitude_monotonic_GET() == 1.909252E38F);
            assert(pack.altitude_terrain_GET() == -2.1042001E37F);
            assert(pack.altitude_relative_GET() == 1.4465317E38F);
            assert(pack.altitude_amsl_GET() == -3.4919404E36F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(1.909252E38F) ;
        p141.altitude_local_SET(1.0880221E38F) ;
        p141.altitude_relative_SET(1.4465317E38F) ;
        p141.time_usec_SET(6064763846987536360L) ;
        p141.bottom_clearance_SET(-3.363318E38F) ;
        p141.altitude_amsl_SET(-3.4919404E36F) ;
        p141.altitude_terrain_SET(-2.1042001E37F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)249);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)236, (char)40, (char)144, (char)178, (char)83, (char)209, (char)152, (char)30, (char)128, (char)233, (char)200, (char)100, (char)173, (char)48, (char)114, (char)129, (char)217, (char)37, (char)20, (char)179, (char)255, (char)107, (char)53, (char)120, (char)58, (char)68, (char)46, (char)188, (char)132, (char)69, (char)17, (char)24, (char)174, (char)177, (char)3, (char)193, (char)236, (char)74, (char)55, (char)235, (char)33, (char)228, (char)20, (char)67, (char)236, (char)120, (char)160, (char)197, (char)22, (char)112, (char)52, (char)243, (char)8, (char)146, (char)253, (char)20, (char)139, (char)52, (char)111, (char)254, (char)52, (char)118, (char)208, (char)202, (char)81, (char)66, (char)139, (char)27, (char)136, (char)153, (char)255, (char)49, (char)12, (char)222, (char)71, (char)203, (char)93, (char)120, (char)39, (char)69, (char)206, (char)25, (char)150, (char)182, (char)29, (char)19, (char)130, (char)112, (char)239, (char)45, (char)63, (char)16, (char)226, (char)254, (char)128, (char)234, (char)205, (char)153, (char)229, (char)2, (char)89, (char)125, (char)199, (char)11, (char)113, (char)197, (char)135, (char)140, (char)132, (char)47, (char)99, (char)119, (char)11, (char)155, (char)147, (char)246, (char)97, (char)200, (char)197, (char)164}));
            assert(pack.request_id_GET() == (char)102);
            assert(pack.transfer_type_GET() == (char)16);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)67, (char)106, (char)215, (char)232, (char)2, (char)126, (char)81, (char)158, (char)48, (char)159, (char)6, (char)81, (char)170, (char)148, (char)110, (char)115, (char)152, (char)210, (char)23, (char)57, (char)208, (char)243, (char)187, (char)37, (char)24, (char)80, (char)164, (char)154, (char)175, (char)88, (char)251, (char)5, (char)117, (char)73, (char)226, (char)206, (char)74, (char)65, (char)88, (char)55, (char)25, (char)122, (char)139, (char)25, (char)138, (char)142, (char)182, (char)124, (char)247, (char)109, (char)152, (char)135, (char)70, (char)146, (char)38, (char)126, (char)255, (char)11, (char)179, (char)236, (char)66, (char)58, (char)224, (char)59, (char)79, (char)72, (char)214, (char)114, (char)205, (char)161, (char)145, (char)36, (char)82, (char)136, (char)176, (char)73, (char)252, (char)115, (char)6, (char)165, (char)83, (char)107, (char)108, (char)128, (char)103, (char)222, (char)112, (char)178, (char)108, (char)95, (char)67, (char)30, (char)113, (char)213, (char)33, (char)19, (char)84, (char)1, (char)167, (char)33, (char)238, (char)154, (char)83, (char)255, (char)127, (char)122, (char)46, (char)245, (char)25, (char)222, (char)211, (char)233, (char)154, (char)111, (char)45, (char)144, (char)197, (char)174, (char)90, (char)251}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.uri_type_SET((char)249) ;
        p142.request_id_SET((char)102) ;
        p142.uri_SET(new char[] {(char)67, (char)106, (char)215, (char)232, (char)2, (char)126, (char)81, (char)158, (char)48, (char)159, (char)6, (char)81, (char)170, (char)148, (char)110, (char)115, (char)152, (char)210, (char)23, (char)57, (char)208, (char)243, (char)187, (char)37, (char)24, (char)80, (char)164, (char)154, (char)175, (char)88, (char)251, (char)5, (char)117, (char)73, (char)226, (char)206, (char)74, (char)65, (char)88, (char)55, (char)25, (char)122, (char)139, (char)25, (char)138, (char)142, (char)182, (char)124, (char)247, (char)109, (char)152, (char)135, (char)70, (char)146, (char)38, (char)126, (char)255, (char)11, (char)179, (char)236, (char)66, (char)58, (char)224, (char)59, (char)79, (char)72, (char)214, (char)114, (char)205, (char)161, (char)145, (char)36, (char)82, (char)136, (char)176, (char)73, (char)252, (char)115, (char)6, (char)165, (char)83, (char)107, (char)108, (char)128, (char)103, (char)222, (char)112, (char)178, (char)108, (char)95, (char)67, (char)30, (char)113, (char)213, (char)33, (char)19, (char)84, (char)1, (char)167, (char)33, (char)238, (char)154, (char)83, (char)255, (char)127, (char)122, (char)46, (char)245, (char)25, (char)222, (char)211, (char)233, (char)154, (char)111, (char)45, (char)144, (char)197, (char)174, (char)90, (char)251}, 0) ;
        p142.transfer_type_SET((char)16) ;
        p142.storage_SET(new char[] {(char)236, (char)40, (char)144, (char)178, (char)83, (char)209, (char)152, (char)30, (char)128, (char)233, (char)200, (char)100, (char)173, (char)48, (char)114, (char)129, (char)217, (char)37, (char)20, (char)179, (char)255, (char)107, (char)53, (char)120, (char)58, (char)68, (char)46, (char)188, (char)132, (char)69, (char)17, (char)24, (char)174, (char)177, (char)3, (char)193, (char)236, (char)74, (char)55, (char)235, (char)33, (char)228, (char)20, (char)67, (char)236, (char)120, (char)160, (char)197, (char)22, (char)112, (char)52, (char)243, (char)8, (char)146, (char)253, (char)20, (char)139, (char)52, (char)111, (char)254, (char)52, (char)118, (char)208, (char)202, (char)81, (char)66, (char)139, (char)27, (char)136, (char)153, (char)255, (char)49, (char)12, (char)222, (char)71, (char)203, (char)93, (char)120, (char)39, (char)69, (char)206, (char)25, (char)150, (char)182, (char)29, (char)19, (char)130, (char)112, (char)239, (char)45, (char)63, (char)16, (char)226, (char)254, (char)128, (char)234, (char)205, (char)153, (char)229, (char)2, (char)89, (char)125, (char)199, (char)11, (char)113, (char)197, (char)135, (char)140, (char)132, (char)47, (char)99, (char)119, (char)11, (char)155, (char)147, (char)246, (char)97, (char)200, (char)197, (char)164}, 0) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1904767425L);
            assert(pack.press_abs_GET() == -1.2588438E38F);
            assert(pack.press_diff_GET() == -2.171953E37F);
            assert(pack.temperature_GET() == (short) -615);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1904767425L) ;
        p143.press_abs_SET(-1.2588438E38F) ;
        p143.temperature_SET((short) -615) ;
        p143.press_diff_SET(-2.171953E37F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 953760609842093288L);
            assert(pack.lon_GET() == -1562232351);
            assert(pack.alt_GET() == 1.9798726E37F);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {6.7370417E37F, 7.086934E36F, -2.4250308E38F}));
            assert(pack.est_capabilities_GET() == (char)43);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {-3.1422797E37F, 2.0285793E38F, 3.2644076E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {2.791441E38F, 1.9432295E37F, -2.2485688E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.1681728E38F, 1.2245316E38F, -2.7743213E38F, -3.151141E38F}));
            assert(pack.lat_GET() == 1298308124);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {7.859878E37F, 2.0335004E38F, -8.190031E37F}));
            assert(pack.custom_state_GET() == 7313025987142100941L);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.acc_SET(new float[] {-3.1422797E37F, 2.0285793E38F, 3.2644076E38F}, 0) ;
        p144.position_cov_SET(new float[] {2.791441E38F, 1.9432295E37F, -2.2485688E38F}, 0) ;
        p144.lat_SET(1298308124) ;
        p144.est_capabilities_SET((char)43) ;
        p144.attitude_q_SET(new float[] {-1.1681728E38F, 1.2245316E38F, -2.7743213E38F, -3.151141E38F}, 0) ;
        p144.timestamp_SET(953760609842093288L) ;
        p144.custom_state_SET(7313025987142100941L) ;
        p144.vel_SET(new float[] {6.7370417E37F, 7.086934E36F, -2.4250308E38F}, 0) ;
        p144.lon_SET(-1562232351) ;
        p144.rates_SET(new float[] {7.859878E37F, 2.0335004E38F, -8.190031E37F}, 0) ;
        p144.alt_SET(1.9798726E37F) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.z_acc_GET() == -2.456832E38F);
            assert(pack.z_vel_GET() == -2.1992032E38F);
            assert(pack.x_vel_GET() == -2.744857E38F);
            assert(pack.airspeed_GET() == -2.3801615E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {1.58611E38F, -2.2129717E38F, 3.18976E38F}));
            assert(pack.pitch_rate_GET() == 2.1770205E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.558793E37F, 3.2611255E38F, -2.2530321E38F, -5.633762E36F}));
            assert(pack.roll_rate_GET() == -1.4946451E37F);
            assert(pack.y_vel_GET() == 1.5002137E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.4124683E38F, -1.9892337E37F, 8.931781E37F}));
            assert(pack.y_pos_GET() == 3.217629E38F);
            assert(pack.y_acc_GET() == 4.7808104E37F);
            assert(pack.x_acc_GET() == -1.6169347E38F);
            assert(pack.x_pos_GET() == -3.0553057E38F);
            assert(pack.z_pos_GET() == 1.1492006E38F);
            assert(pack.time_usec_GET() == 2388211587078326383L);
            assert(pack.yaw_rate_GET() == -1.4353372E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_acc_SET(-2.456832E38F) ;
        p146.pitch_rate_SET(2.1770205E38F) ;
        p146.z_pos_SET(1.1492006E38F) ;
        p146.x_pos_SET(-3.0553057E38F) ;
        p146.y_acc_SET(4.7808104E37F) ;
        p146.x_acc_SET(-1.6169347E38F) ;
        p146.airspeed_SET(-2.3801615E38F) ;
        p146.time_usec_SET(2388211587078326383L) ;
        p146.y_pos_SET(3.217629E38F) ;
        p146.roll_rate_SET(-1.4946451E37F) ;
        p146.y_vel_SET(1.5002137E38F) ;
        p146.yaw_rate_SET(-1.4353372E38F) ;
        p146.z_vel_SET(-2.1992032E38F) ;
        p146.vel_variance_SET(new float[] {1.58611E38F, -2.2129717E38F, 3.18976E38F}, 0) ;
        p146.q_SET(new float[] {-7.558793E37F, 3.2611255E38F, -2.2530321E38F, -5.633762E36F}, 0) ;
        p146.pos_variance_SET(new float[] {2.4124683E38F, -1.9892337E37F, 8.931781E37F}, 0) ;
        p146.x_vel_SET(-2.744857E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.energy_consumed_GET() == -1294056717);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.id_GET() == (char)21);
            assert(pack.temperature_GET() == (short)8395);
            assert(pack.current_battery_GET() == (short) -7257);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
            assert(pack.current_consumed_GET() == 785281671);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)26954, (char)48060, (char)10929, (char)45893, (char)62733, (char)9508, (char)62834, (char)62909, (char)14745, (char)56101}));
            assert(pack.battery_remaining_GET() == (byte)87);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.id_SET((char)21) ;
        p147.temperature_SET((short)8395) ;
        p147.energy_consumed_SET(-1294056717) ;
        p147.current_battery_SET((short) -7257) ;
        p147.voltages_SET(new char[] {(char)26954, (char)48060, (char)10929, (char)45893, (char)62733, (char)9508, (char)62834, (char)62909, (char)14745, (char)56101}, 0) ;
        p147.current_consumed_SET(785281671) ;
        p147.battery_remaining_SET((byte)87) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.flight_sw_version_GET() == 2940269516L);
            assert(pack.uid_GET() == 7989955430306557395L);
            assert(pack.product_id_GET() == (char)46121);
            assert(pack.middleware_sw_version_GET() == 3229232037L);
            assert(pack.vendor_id_GET() == (char)44005);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)25, (char)166, (char)114, (char)126, (char)170, (char)210, (char)170, (char)215}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)136, (char)223, (char)178, (char)41, (char)242, (char)198, (char)25, (char)81, (char)192, (char)11, (char)35, (char)191, (char)78, (char)176, (char)153, (char)62, (char)147, (char)10}));
            assert(pack.os_sw_version_GET() == 3434624367L);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)90, (char)176, (char)103, (char)225, (char)223, (char)91, (char)122, (char)157}));
            assert(pack.board_version_GET() == 1873624109L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)121, (char)203, (char)212, (char)253, (char)174, (char)19, (char)239, (char)212}));
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.os_custom_version_SET(new char[] {(char)90, (char)176, (char)103, (char)225, (char)223, (char)91, (char)122, (char)157}, 0) ;
        p148.flight_sw_version_SET(2940269516L) ;
        p148.flight_custom_version_SET(new char[] {(char)121, (char)203, (char)212, (char)253, (char)174, (char)19, (char)239, (char)212}, 0) ;
        p148.vendor_id_SET((char)44005) ;
        p148.uid_SET(7989955430306557395L) ;
        p148.middleware_custom_version_SET(new char[] {(char)25, (char)166, (char)114, (char)126, (char)170, (char)210, (char)170, (char)215}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION) ;
        p148.board_version_SET(1873624109L) ;
        p148.middleware_sw_version_SET(3229232037L) ;
        p148.os_sw_version_SET(3434624367L) ;
        p148.product_id_SET((char)46121) ;
        p148.uid2_SET(new char[] {(char)136, (char)223, (char)178, (char)41, (char)242, (char)198, (char)25, (char)81, (char)192, (char)11, (char)35, (char)191, (char)78, (char)176, (char)153, (char)62, (char)147, (char)10}, 0, PH) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            assert(pack.size_x_GET() == -3.0074694E38F);
            assert(pack.z_TRY(ph) == -2.157309E38F);
            assert(pack.time_usec_GET() == 8210859466172793323L);
            assert(pack.distance_GET() == -2.5599849E38F);
            assert(pack.x_TRY(ph) == 6.489184E37F);
            assert(pack.position_valid_TRY(ph) == (char)0);
            assert(pack.y_TRY(ph) == 2.1473626E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.angle_x_GET() == 1.9264263E38F);
            assert(pack.target_num_GET() == (char)166);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.5141054E38F, 9.323304E37F, 2.2778294E38F, 1.1413896E38F}));
            assert(pack.angle_y_GET() == 6.7419556E37F);
            assert(pack.size_y_GET() == 2.1612607E38F);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.position_valid_SET((char)0, PH) ;
        p149.z_SET(-2.157309E38F, PH) ;
        p149.y_SET(2.1473626E37F, PH) ;
        p149.q_SET(new float[] {1.5141054E38F, 9.323304E37F, 2.2778294E38F, 1.1413896E38F}, 0, PH) ;
        p149.size_y_SET(2.1612607E38F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER) ;
        p149.target_num_SET((char)166) ;
        p149.distance_SET(-2.5599849E38F) ;
        p149.size_x_SET(-3.0074694E38F) ;
        p149.time_usec_SET(8210859466172793323L) ;
        p149.x_SET(6.489184E37F, PH) ;
        p149.angle_y_SET(6.7419556E37F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.angle_x_SET(1.9264263E38F) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
        {
            assert(pack.accel_2_GET() == -1.70582E38F);
            assert(pack.accel_1_GET() == -2.2626634E38F);
            assert(pack.accel_0_GET() == -1.1865866E38F);
            assert(pack.usec_GET() == 5913467690607768437L);
            assert(pack.gyro_2_GET() == -1.5693631E38F);
            assert(pack.gyro_1_GET() == -9.46859E36F);
            assert(pack.gyro_0_GET() == 2.4424625E38F);
        });
        DemoDevice.NAV_FILTER_BIAS p220 = LoopBackDemoChannel.new_NAV_FILTER_BIAS();
        PH.setPack(p220);
        p220.accel_2_SET(-1.70582E38F) ;
        p220.gyro_1_SET(-9.46859E36F) ;
        p220.accel_1_SET(-2.2626634E38F) ;
        p220.accel_0_SET(-1.1865866E38F) ;
        p220.usec_SET(5913467690607768437L) ;
        p220.gyro_2_SET(-1.5693631E38F) ;
        p220.gyro_0_SET(2.4424625E38F) ;
        LoopBackDemoChannel.instance.send(p220);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.elevator_GET(),  new char[] {(char)47426, (char)39961, (char)3124}));
            assert(Arrays.equals(pack.gyro_GET(),  new char[] {(char)51132, (char)1466}));
            assert(Arrays.equals(pack.rudder_GET(),  new char[] {(char)57997, (char)57256, (char)54963}));
            assert(Arrays.equals(pack.aileron_GET(),  new char[] {(char)9565, (char)62132, (char)60512}));
            assert(Arrays.equals(pack.throttle_GET(),  new char[] {(char)3300, (char)30105, (char)42897, (char)44112, (char)11537}));
            assert(Arrays.equals(pack.pitch_GET(),  new char[] {(char)49378, (char)28182, (char)9498, (char)61611, (char)45180}));
        });
        DemoDevice.RADIO_CALIBRATION p221 = LoopBackDemoChannel.new_RADIO_CALIBRATION();
        PH.setPack(p221);
        p221.throttle_SET(new char[] {(char)3300, (char)30105, (char)42897, (char)44112, (char)11537}, 0) ;
        p221.gyro_SET(new char[] {(char)51132, (char)1466}, 0) ;
        p221.aileron_SET(new char[] {(char)9565, (char)62132, (char)60512}, 0) ;
        p221.pitch_SET(new char[] {(char)49378, (char)28182, (char)9498, (char)61611, (char)45180}, 0) ;
        p221.elevator_SET(new char[] {(char)47426, (char)39961, (char)3124}, 0) ;
        p221.rudder_SET(new char[] {(char)57997, (char)57256, (char)54963}, 0) ;
        LoopBackDemoChannel.instance.send(p221);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == (char)211);
            assert(pack.pilot_GET() == (char)131);
            assert(pack.nav_mode_GET() == (char)13);
        });
        DemoDevice.UALBERTA_SYS_STATUS p222 = LoopBackDemoChannel.new_UALBERTA_SYS_STATUS();
        PH.setPack(p222);
        p222.pilot_SET((char)131) ;
        p222.nav_mode_SET((char)13) ;
        p222.mode_SET((char)211) ;
        LoopBackDemoChannel.instance.send(p222);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mag_ratio_GET() == 2.9238093E38F);
            assert(pack.time_usec_GET() == 8662403594117300468L);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            assert(pack.pos_horiz_accuracy_GET() == 2.3381456E38F);
            assert(pack.pos_vert_ratio_GET() == -3.235441E38F);
            assert(pack.hagl_ratio_GET() == -5.5254243E37F);
            assert(pack.tas_ratio_GET() == -2.8797756E38F);
            assert(pack.pos_horiz_ratio_GET() == -6.807651E37F);
            assert(pack.pos_vert_accuracy_GET() == 2.171704E38F);
            assert(pack.vel_ratio_GET() == 1.8306035E38F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.tas_ratio_SET(-2.8797756E38F) ;
        p230.pos_vert_ratio_SET(-3.235441E38F) ;
        p230.pos_vert_accuracy_SET(2.171704E38F) ;
        p230.pos_horiz_accuracy_SET(2.3381456E38F) ;
        p230.hagl_ratio_SET(-5.5254243E37F) ;
        p230.mag_ratio_SET(2.9238093E38F) ;
        p230.pos_horiz_ratio_SET(-6.807651E37F) ;
        p230.vel_ratio_SET(1.8306035E38F) ;
        p230.time_usec_SET(8662403594117300468L) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 736879140780660805L);
            assert(pack.vert_accuracy_GET() == -2.6635124E38F);
            assert(pack.horiz_accuracy_GET() == 6.425052E37F);
            assert(pack.wind_z_GET() == 2.2444218E38F);
            assert(pack.wind_alt_GET() == -2.4407373E38F);
            assert(pack.wind_y_GET() == -5.991486E37F);
            assert(pack.wind_x_GET() == -9.438291E37F);
            assert(pack.var_vert_GET() == -3.2823346E38F);
            assert(pack.var_horiz_GET() == 2.4968547E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.vert_accuracy_SET(-2.6635124E38F) ;
        p231.wind_z_SET(2.2444218E38F) ;
        p231.horiz_accuracy_SET(6.425052E37F) ;
        p231.wind_alt_SET(-2.4407373E38F) ;
        p231.var_horiz_SET(2.4968547E38F) ;
        p231.time_usec_SET(736879140780660805L) ;
        p231.wind_x_SET(-9.438291E37F) ;
        p231.var_vert_SET(-3.2823346E38F) ;
        p231.wind_y_SET(-5.991486E37F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1612390337);
            assert(pack.time_usec_GET() == 2856953150571157503L);
            assert(pack.time_week_ms_GET() == 2221758861L);
            assert(pack.vdop_GET() == 4.4360383E37F);
            assert(pack.horiz_accuracy_GET() == -2.6207354E38F);
            assert(pack.vn_GET() == -2.0323922E38F);
            assert(pack.vert_accuracy_GET() == 2.7907054E38F);
            assert(pack.lat_GET() == -1777116822);
            assert(pack.gps_id_GET() == (char)37);
            assert(pack.vd_GET() == -3.2624905E38F);
            assert(pack.fix_type_GET() == (char)185);
            assert(pack.hdop_GET() == 6.607658E37F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
            assert(pack.time_week_GET() == (char)9912);
            assert(pack.ve_GET() == -4.2691446E35F);
            assert(pack.alt_GET() == 2.7872779E38F);
            assert(pack.speed_accuracy_GET() == 2.3044937E38F);
            assert(pack.satellites_visible_GET() == (char)109);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.horiz_accuracy_SET(-2.6207354E38F) ;
        p232.ve_SET(-4.2691446E35F) ;
        p232.vdop_SET(4.4360383E37F) ;
        p232.speed_accuracy_SET(2.3044937E38F) ;
        p232.time_week_ms_SET(2221758861L) ;
        p232.time_usec_SET(2856953150571157503L) ;
        p232.time_week_SET((char)9912) ;
        p232.satellites_visible_SET((char)109) ;
        p232.gps_id_SET((char)37) ;
        p232.lat_SET(-1777116822) ;
        p232.vert_accuracy_SET(2.7907054E38F) ;
        p232.vd_SET(-3.2624905E38F) ;
        p232.alt_SET(2.7872779E38F) ;
        p232.vn_SET(-2.0323922E38F) ;
        p232.fix_type_SET((char)185) ;
        p232.lon_SET(1612390337) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP) ;
        p232.hdop_SET(6.607658E37F) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)152, (char)198, (char)59, (char)107, (char)241, (char)198, (char)0, (char)32, (char)28, (char)120, (char)203, (char)145, (char)5, (char)5, (char)63, (char)28, (char)78, (char)88, (char)219, (char)85, (char)244, (char)197, (char)22, (char)25, (char)80, (char)125, (char)3, (char)82, (char)73, (char)226, (char)140, (char)103, (char)251, (char)85, (char)117, (char)239, (char)18, (char)204, (char)152, (char)166, (char)130, (char)192, (char)49, (char)30, (char)56, (char)10, (char)130, (char)113, (char)16, (char)79, (char)157, (char)181, (char)153, (char)50, (char)70, (char)206, (char)209, (char)171, (char)130, (char)114, (char)192, (char)142, (char)37, (char)80, (char)190, (char)63, (char)162, (char)125, (char)45, (char)150, (char)183, (char)233, (char)255, (char)64, (char)168, (char)188, (char)98, (char)163, (char)244, (char)79, (char)32, (char)53, (char)53, (char)50, (char)188, (char)255, (char)105, (char)205, (char)7, (char)170, (char)74, (char)135, (char)250, (char)184, (char)73, (char)249, (char)180, (char)245, (char)187, (char)126, (char)140, (char)143, (char)246, (char)209, (char)4, (char)30, (char)214, (char)82, (char)80, (char)73, (char)66, (char)163, (char)31, (char)215, (char)173, (char)16, (char)32, (char)116, (char)186, (char)58, (char)243, (char)197, (char)86, (char)16, (char)41, (char)144, (char)119, (char)174, (char)39, (char)97, (char)251, (char)254, (char)68, (char)50, (char)216, (char)214, (char)93, (char)146, (char)17, (char)79, (char)76, (char)115, (char)133, (char)166, (char)54, (char)129, (char)196, (char)126, (char)234, (char)9, (char)175, (char)164, (char)253, (char)41, (char)174, (char)135, (char)88, (char)121, (char)150, (char)153, (char)69, (char)213, (char)21, (char)218, (char)137, (char)118, (char)141, (char)237, (char)228, (char)174, (char)188, (char)25, (char)54, (char)121, (char)213, (char)143, (char)227, (char)240, (char)63, (char)250}));
            assert(pack.len_GET() == (char)135);
            assert(pack.flags_GET() == (char)35);
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)135) ;
        p233.flags_SET((char)35) ;
        p233.data__SET(new char[] {(char)152, (char)198, (char)59, (char)107, (char)241, (char)198, (char)0, (char)32, (char)28, (char)120, (char)203, (char)145, (char)5, (char)5, (char)63, (char)28, (char)78, (char)88, (char)219, (char)85, (char)244, (char)197, (char)22, (char)25, (char)80, (char)125, (char)3, (char)82, (char)73, (char)226, (char)140, (char)103, (char)251, (char)85, (char)117, (char)239, (char)18, (char)204, (char)152, (char)166, (char)130, (char)192, (char)49, (char)30, (char)56, (char)10, (char)130, (char)113, (char)16, (char)79, (char)157, (char)181, (char)153, (char)50, (char)70, (char)206, (char)209, (char)171, (char)130, (char)114, (char)192, (char)142, (char)37, (char)80, (char)190, (char)63, (char)162, (char)125, (char)45, (char)150, (char)183, (char)233, (char)255, (char)64, (char)168, (char)188, (char)98, (char)163, (char)244, (char)79, (char)32, (char)53, (char)53, (char)50, (char)188, (char)255, (char)105, (char)205, (char)7, (char)170, (char)74, (char)135, (char)250, (char)184, (char)73, (char)249, (char)180, (char)245, (char)187, (char)126, (char)140, (char)143, (char)246, (char)209, (char)4, (char)30, (char)214, (char)82, (char)80, (char)73, (char)66, (char)163, (char)31, (char)215, (char)173, (char)16, (char)32, (char)116, (char)186, (char)58, (char)243, (char)197, (char)86, (char)16, (char)41, (char)144, (char)119, (char)174, (char)39, (char)97, (char)251, (char)254, (char)68, (char)50, (char)216, (char)214, (char)93, (char)146, (char)17, (char)79, (char)76, (char)115, (char)133, (char)166, (char)54, (char)129, (char)196, (char)126, (char)234, (char)9, (char)175, (char)164, (char)253, (char)41, (char)174, (char)135, (char)88, (char)121, (char)150, (char)153, (char)69, (char)213, (char)21, (char)218, (char)137, (char)118, (char)141, (char)237, (char)228, (char)174, (char)188, (char)25, (char)54, (char)121, (char)213, (char)143, (char)227, (char)240, (char)63, (char)250}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.wp_num_GET() == (char)226);
            assert(pack.climb_rate_GET() == (byte)18);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.latitude_GET() == 1665804645);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
            assert(pack.roll_GET() == (short) -7539);
            assert(pack.airspeed_GET() == (char)99);
            assert(pack.longitude_GET() == -1971576106);
            assert(pack.altitude_sp_GET() == (short)31555);
            assert(pack.custom_mode_GET() == 2486523832L);
            assert(pack.battery_remaining_GET() == (char)220);
            assert(pack.wp_distance_GET() == (char)54882);
            assert(pack.airspeed_sp_GET() == (char)46);
            assert(pack.gps_nsat_GET() == (char)6);
            assert(pack.temperature_air_GET() == (byte)62);
            assert(pack.failsafe_GET() == (char)132);
            assert(pack.temperature_GET() == (byte)82);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            assert(pack.pitch_GET() == (short)31375);
            assert(pack.throttle_GET() == (byte)32);
            assert(pack.altitude_amsl_GET() == (short)5346);
            assert(pack.groundspeed_GET() == (char)92);
            assert(pack.heading_sp_GET() == (short) -12225);
            assert(pack.heading_GET() == (char)42969);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.latitude_SET(1665804645) ;
        p234.airspeed_sp_SET((char)46) ;
        p234.altitude_amsl_SET((short)5346) ;
        p234.wp_distance_SET((char)54882) ;
        p234.altitude_sp_SET((short)31555) ;
        p234.throttle_SET((byte)32) ;
        p234.climb_rate_SET((byte)18) ;
        p234.heading_SET((char)42969) ;
        p234.temperature_air_SET((byte)62) ;
        p234.pitch_SET((short)31375) ;
        p234.failsafe_SET((char)132) ;
        p234.heading_sp_SET((short) -12225) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX) ;
        p234.gps_nsat_SET((char)6) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        p234.temperature_SET((byte)82) ;
        p234.longitude_SET(-1971576106) ;
        p234.wp_num_SET((char)226) ;
        p234.roll_SET((short) -7539) ;
        p234.battery_remaining_SET((char)220) ;
        p234.airspeed_SET((char)99) ;
        p234.groundspeed_SET((char)92) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED) ;
        p234.custom_mode_SET(2486523832L) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 3405597881L);
            assert(pack.vibration_y_GET() == 1.727034E38F);
            assert(pack.vibration_x_GET() == -9.011362E37F);
            assert(pack.time_usec_GET() == 9140390992572264362L);
            assert(pack.clipping_2_GET() == 1444464218L);
            assert(pack.vibration_z_GET() == -2.985953E38F);
            assert(pack.clipping_1_GET() == 3777169293L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_z_SET(-2.985953E38F) ;
        p241.vibration_x_SET(-9.011362E37F) ;
        p241.clipping_1_SET(3777169293L) ;
        p241.clipping_2_SET(1444464218L) ;
        p241.vibration_y_SET(1.727034E38F) ;
        p241.clipping_0_SET(3405597881L) ;
        p241.time_usec_SET(9140390992572264362L) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 992053261);
            assert(pack.y_GET() == 3.9025517E36F);
            assert(pack.approach_x_GET() == 1.9126988E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.742971E38F, 2.6389758E38F, -4.5436274E37F, -3.2386542E38F}));
            assert(pack.altitude_GET() == 1994539157);
            assert(pack.time_usec_TRY(ph) == 6868348198552290785L);
            assert(pack.latitude_GET() == -1366407140);
            assert(pack.x_GET() == -3.388738E38F);
            assert(pack.z_GET() == 1.2145266E38F);
            assert(pack.approach_y_GET() == 3.1373732E38F);
            assert(pack.approach_z_GET() == 9.621345E37F);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.longitude_SET(992053261) ;
        p242.approach_y_SET(3.1373732E38F) ;
        p242.q_SET(new float[] {2.742971E38F, 2.6389758E38F, -4.5436274E37F, -3.2386542E38F}, 0) ;
        p242.latitude_SET(-1366407140) ;
        p242.time_usec_SET(6868348198552290785L, PH) ;
        p242.y_SET(3.9025517E36F) ;
        p242.approach_x_SET(1.9126988E38F) ;
        p242.z_SET(1.2145266E38F) ;
        p242.x_SET(-3.388738E38F) ;
        p242.altitude_SET(1994539157) ;
        p242.approach_z_SET(9.621345E37F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 825155889);
            assert(pack.y_GET() == -3.2038966E38F);
            assert(pack.approach_x_GET() == -2.4859523E38F);
            assert(pack.approach_y_GET() == 1.1758328E38F);
            assert(pack.longitude_GET() == -545057822);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.883347E38F, -1.4932902E38F, 1.6886781E38F, 9.321385E37F}));
            assert(pack.z_GET() == 2.2236924E38F);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.latitude_GET() == -1987266878);
            assert(pack.time_usec_TRY(ph) == 3437338802519211183L);
            assert(pack.approach_z_GET() == -2.742879E38F);
            assert(pack.x_GET() == 2.5004361E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.latitude_SET(-1987266878) ;
        p243.approach_z_SET(-2.742879E38F) ;
        p243.q_SET(new float[] {-1.883347E38F, -1.4932902E38F, 1.6886781E38F, 9.321385E37F}, 0) ;
        p243.approach_x_SET(-2.4859523E38F) ;
        p243.time_usec_SET(3437338802519211183L, PH) ;
        p243.altitude_SET(825155889) ;
        p243.target_system_SET((char)31) ;
        p243.approach_y_SET(1.1758328E38F) ;
        p243.x_SET(2.5004361E38F) ;
        p243.y_SET(-3.2038966E38F) ;
        p243.z_SET(2.2236924E38F) ;
        p243.longitude_SET(-545057822) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)51262);
            assert(pack.interval_us_GET() == -687018264);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-687018264) ;
        p244.message_id_SET((char)51262) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -801357717);
            assert(pack.heading_GET() == (char)58204);
            assert(pack.lat_GET() == -90295110);
            assert(pack.squawk_GET() == (char)41001);
            assert(pack.hor_velocity_GET() == (char)5720);
            assert(pack.ver_velocity_GET() == (short) -14176);
            assert(pack.tslc_GET() == (char)178);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.altitude_GET() == 858745822);
            assert(pack.ICAO_address_GET() == 2357870213L);
            assert(pack.callsign_LEN(ph) == 1);
            assert(pack.callsign_TRY(ph).equals("q"));
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.lon_SET(-801357717) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2) ;
        p246.ver_velocity_SET((short) -14176) ;
        p246.squawk_SET((char)41001) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN) ;
        p246.hor_velocity_SET((char)5720) ;
        p246.lat_SET(-90295110) ;
        p246.altitude_SET(858745822) ;
        p246.heading_SET((char)58204) ;
        p246.tslc_SET((char)178) ;
        p246.callsign_SET("q", PH) ;
        p246.ICAO_address_SET(2357870213L) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 3040941333L);
            assert(pack.horizontal_minimum_delta_GET() == -1.2018728E38F);
            assert(pack.time_to_minimum_delta_GET() == 1.1425471E38F);
            assert(pack.altitude_minimum_delta_GET() == -2.1830448E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(-2.1830448E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.time_to_minimum_delta_SET(1.1425471E38F) ;
        p247.id_SET(3040941333L) ;
        p247.horizontal_minimum_delta_SET(-1.2018728E38F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)217);
            assert(pack.target_network_GET() == (char)224);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)152, (char)203, (char)80, (char)3, (char)237, (char)211, (char)167, (char)132, (char)130, (char)217, (char)174, (char)226, (char)85, (char)25, (char)19, (char)32, (char)203, (char)182, (char)103, (char)83, (char)107, (char)34, (char)166, (char)39, (char)122, (char)161, (char)109, (char)203, (char)28, (char)90, (char)241, (char)132, (char)184, (char)98, (char)162, (char)42, (char)254, (char)125, (char)84, (char)164, (char)177, (char)173, (char)196, (char)205, (char)228, (char)137, (char)18, (char)79, (char)78, (char)123, (char)161, (char)251, (char)247, (char)209, (char)128, (char)71, (char)122, (char)206, (char)64, (char)124, (char)80, (char)222, (char)102, (char)175, (char)119, (char)226, (char)83, (char)138, (char)6, (char)127, (char)34, (char)34, (char)237, (char)69, (char)33, (char)209, (char)161, (char)3, (char)132, (char)6, (char)126, (char)209, (char)84, (char)208, (char)103, (char)95, (char)78, (char)66, (char)3, (char)128, (char)173, (char)245, (char)232, (char)144, (char)226, (char)87, (char)83, (char)220, (char)1, (char)249, (char)122, (char)173, (char)58, (char)81, (char)31, (char)136, (char)40, (char)181, (char)178, (char)251, (char)199, (char)41, (char)175, (char)12, (char)232, (char)103, (char)61, (char)71, (char)81, (char)152, (char)3, (char)122, (char)248, (char)33, (char)46, (char)86, (char)218, (char)228, (char)228, (char)107, (char)196, (char)74, (char)122, (char)49, (char)137, (char)181, (char)246, (char)64, (char)149, (char)38, (char)176, (char)61, (char)94, (char)54, (char)37, (char)176, (char)236, (char)211, (char)131, (char)36, (char)193, (char)2, (char)190, (char)93, (char)190, (char)181, (char)93, (char)88, (char)232, (char)175, (char)90, (char)189, (char)65, (char)159, (char)207, (char)85, (char)251, (char)207, (char)35, (char)33, (char)200, (char)37, (char)43, (char)216, (char)245, (char)7, (char)161, (char)2, (char)207, (char)139, (char)230, (char)11, (char)101, (char)93, (char)252, (char)15, (char)173, (char)202, (char)82, (char)239, (char)90, (char)238, (char)169, (char)67, (char)59, (char)10, (char)229, (char)127, (char)22, (char)238, (char)176, (char)246, (char)29, (char)153, (char)196, (char)33, (char)0, (char)230, (char)87, (char)153, (char)221, (char)103, (char)153, (char)14, (char)248, (char)208, (char)190, (char)47, (char)5, (char)220, (char)12, (char)42, (char)201, (char)107, (char)176, (char)233, (char)48, (char)14, (char)178, (char)22, (char)19, (char)73, (char)72, (char)40, (char)162, (char)69, (char)66, (char)154, (char)55, (char)189, (char)234, (char)216, (char)94, (char)166, (char)151, (char)216, (char)16, (char)103, (char)211}));
            assert(pack.message_type_GET() == (char)50080);
            assert(pack.target_component_GET() == (char)112);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)224) ;
        p248.target_system_SET((char)217) ;
        p248.message_type_SET((char)50080) ;
        p248.payload_SET(new char[] {(char)152, (char)203, (char)80, (char)3, (char)237, (char)211, (char)167, (char)132, (char)130, (char)217, (char)174, (char)226, (char)85, (char)25, (char)19, (char)32, (char)203, (char)182, (char)103, (char)83, (char)107, (char)34, (char)166, (char)39, (char)122, (char)161, (char)109, (char)203, (char)28, (char)90, (char)241, (char)132, (char)184, (char)98, (char)162, (char)42, (char)254, (char)125, (char)84, (char)164, (char)177, (char)173, (char)196, (char)205, (char)228, (char)137, (char)18, (char)79, (char)78, (char)123, (char)161, (char)251, (char)247, (char)209, (char)128, (char)71, (char)122, (char)206, (char)64, (char)124, (char)80, (char)222, (char)102, (char)175, (char)119, (char)226, (char)83, (char)138, (char)6, (char)127, (char)34, (char)34, (char)237, (char)69, (char)33, (char)209, (char)161, (char)3, (char)132, (char)6, (char)126, (char)209, (char)84, (char)208, (char)103, (char)95, (char)78, (char)66, (char)3, (char)128, (char)173, (char)245, (char)232, (char)144, (char)226, (char)87, (char)83, (char)220, (char)1, (char)249, (char)122, (char)173, (char)58, (char)81, (char)31, (char)136, (char)40, (char)181, (char)178, (char)251, (char)199, (char)41, (char)175, (char)12, (char)232, (char)103, (char)61, (char)71, (char)81, (char)152, (char)3, (char)122, (char)248, (char)33, (char)46, (char)86, (char)218, (char)228, (char)228, (char)107, (char)196, (char)74, (char)122, (char)49, (char)137, (char)181, (char)246, (char)64, (char)149, (char)38, (char)176, (char)61, (char)94, (char)54, (char)37, (char)176, (char)236, (char)211, (char)131, (char)36, (char)193, (char)2, (char)190, (char)93, (char)190, (char)181, (char)93, (char)88, (char)232, (char)175, (char)90, (char)189, (char)65, (char)159, (char)207, (char)85, (char)251, (char)207, (char)35, (char)33, (char)200, (char)37, (char)43, (char)216, (char)245, (char)7, (char)161, (char)2, (char)207, (char)139, (char)230, (char)11, (char)101, (char)93, (char)252, (char)15, (char)173, (char)202, (char)82, (char)239, (char)90, (char)238, (char)169, (char)67, (char)59, (char)10, (char)229, (char)127, (char)22, (char)238, (char)176, (char)246, (char)29, (char)153, (char)196, (char)33, (char)0, (char)230, (char)87, (char)153, (char)221, (char)103, (char)153, (char)14, (char)248, (char)208, (char)190, (char)47, (char)5, (char)220, (char)12, (char)42, (char)201, (char)107, (char)176, (char)233, (char)48, (char)14, (char)178, (char)22, (char)19, (char)73, (char)72, (char)40, (char)162, (char)69, (char)66, (char)154, (char)55, (char)189, (char)234, (char)216, (char)94, (char)166, (char)151, (char)216, (char)16, (char)103, (char)211}, 0) ;
        p248.target_component_SET((char)112) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)40, (byte) - 18, (byte)116, (byte) - 64, (byte)14, (byte)71, (byte) - 98, (byte)117, (byte) - 51, (byte) - 120, (byte) - 91, (byte) - 3, (byte) - 60, (byte)67, (byte)62, (byte)36, (byte)26, (byte)45, (byte) - 55, (byte) - 109, (byte) - 83, (byte) - 74, (byte) - 30, (byte)57, (byte) - 54, (byte) - 120, (byte) - 127, (byte) - 90, (byte)46, (byte) - 40, (byte)64, (byte)72}));
            assert(pack.ver_GET() == (char)91);
            assert(pack.address_GET() == (char)41193);
            assert(pack.type_GET() == (char)63);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)41193) ;
        p249.type_SET((char)63) ;
        p249.ver_SET((char)91) ;
        p249.value_SET(new byte[] {(byte)40, (byte) - 18, (byte)116, (byte) - 64, (byte)14, (byte)71, (byte) - 98, (byte)117, (byte) - 51, (byte) - 120, (byte) - 91, (byte) - 3, (byte) - 60, (byte)67, (byte)62, (byte)36, (byte)26, (byte)45, (byte) - 55, (byte) - 109, (byte) - 83, (byte) - 74, (byte) - 30, (byte)57, (byte) - 54, (byte) - 120, (byte) - 127, (byte) - 90, (byte)46, (byte) - 40, (byte)64, (byte)72}, 0) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 3.8957996E37F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("yotucpb"));
            assert(pack.x_GET() == 1.4211683E38F);
            assert(pack.time_usec_GET() == 6010402841699834110L);
            assert(pack.y_GET() == -1.0012711E38F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("yotucpb", PH) ;
        p250.y_SET(-1.0012711E38F) ;
        p250.time_usec_SET(6010402841699834110L) ;
        p250.x_SET(1.4211683E38F) ;
        p250.z_SET(3.8957996E37F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 2.4779854E37F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("udxLsbesae"));
            assert(pack.time_boot_ms_GET() == 4235164389L);
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(2.4779854E37F) ;
        p251.time_boot_ms_SET(4235164389L) ;
        p251.name_SET("udxLsbesae", PH) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("dlxv"));
            assert(pack.value_GET() == 1844592841);
            assert(pack.time_boot_ms_GET() == 735224442L);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(1844592841) ;
        p252.name_SET("dlxv", PH) ;
        p252.time_boot_ms_SET(735224442L) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            assert(pack.text_LEN(ph) == 29);
            assert(pack.text_TRY(ph).equals("frzvllfShmzjcimyqqdwXozsMgpcy"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("frzvllfShmzjcimyqqdwXozsMgpcy", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2437861606L);
            assert(pack.ind_GET() == (char)130);
            assert(pack.value_GET() == 2.6753325E37F);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)130) ;
        p254.value_SET(2.6753325E37F) ;
        p254.time_boot_ms_SET(2437861606L) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)170);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)22, (char)239, (char)239, (char)134, (char)253, (char)128, (char)74, (char)147, (char)51, (char)40, (char)24, (char)154, (char)82, (char)216, (char)26, (char)175, (char)239, (char)130, (char)105, (char)57, (char)47, (char)180, (char)141, (char)32, (char)107, (char)187, (char)232, (char)145, (char)93, (char)196, (char)26, (char)175}));
            assert(pack.initial_timestamp_GET() == 3803273633620475265L);
            assert(pack.target_component_GET() == (char)57);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)57) ;
        p256.target_system_SET((char)170) ;
        p256.secret_key_SET(new char[] {(char)22, (char)239, (char)239, (char)134, (char)253, (char)128, (char)74, (char)147, (char)51, (char)40, (char)24, (char)154, (char)82, (char)216, (char)26, (char)175, (char)239, (char)130, (char)105, (char)57, (char)47, (char)180, (char)141, (char)32, (char)107, (char)187, (char)232, (char)145, (char)93, (char)196, (char)26, (char)175}, 0) ;
        p256.initial_timestamp_SET(3803273633620475265L) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 1907969862L);
            assert(pack.state_GET() == (char)202);
            assert(pack.time_boot_ms_GET() == 3100675487L);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3100675487L) ;
        p257.state_SET((char)202) ;
        p257.last_change_ms_SET(1907969862L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)198);
            assert(pack.tune_LEN(ph) == 24);
            assert(pack.tune_TRY(ph).equals("mdbraxzxjqtgoRgyqvzgJutf"));
            assert(pack.target_system_GET() == (char)199);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("mdbraxzxjqtgoRgyqvzgJutf", PH) ;
        p258.target_system_SET((char)199) ;
        p258.target_component_SET((char)198) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.lens_id_GET() == (char)67);
            assert(pack.firmware_version_GET() == 2904989794L);
            assert(pack.cam_definition_version_GET() == (char)48285);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)113, (char)87, (char)3, (char)223, (char)167, (char)134, (char)70, (char)199, (char)42, (char)205, (char)181, (char)194, (char)232, (char)241, (char)214, (char)148, (char)215, (char)130, (char)47, (char)182, (char)75, (char)88, (char)212, (char)68, (char)135, (char)52, (char)188, (char)213, (char)162, (char)252, (char)152, (char)37}));
            assert(pack.sensor_size_v_GET() == 4.3453344E37F);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            assert(pack.time_boot_ms_GET() == 599484335L);
            assert(pack.cam_definition_uri_LEN(ph) == 78);
            assert(pack.cam_definition_uri_TRY(ph).equals("onxzmcwbjnwemrvrFclriittddnpkqiUvwmwhRzgxnqpGmwqxIfnXvaaidnwxwjwisyaefkjuoLawf"));
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)99, (char)128, (char)27, (char)123, (char)122, (char)45, (char)129, (char)132, (char)248, (char)121, (char)79, (char)131, (char)140, (char)126, (char)125, (char)22, (char)97, (char)144, (char)27, (char)40, (char)43, (char)19, (char)40, (char)106, (char)201, (char)2, (char)76, (char)58, (char)15, (char)162, (char)58, (char)103}));
            assert(pack.sensor_size_h_GET() == -9.913312E36F);
            assert(pack.resolution_h_GET() == (char)13728);
            assert(pack.focal_length_GET() == -2.8566562E37F);
            assert(pack.resolution_v_GET() == (char)60050);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.sensor_size_h_SET(-9.913312E36F) ;
        p259.resolution_v_SET((char)60050) ;
        p259.time_boot_ms_SET(599484335L) ;
        p259.sensor_size_v_SET(4.3453344E37F) ;
        p259.cam_definition_uri_SET("onxzmcwbjnwemrvrFclriittddnpkqiUvwmwhRzgxnqpGmwqxIfnXvaaidnwxwjwisyaefkjuoLawf", PH) ;
        p259.model_name_SET(new char[] {(char)113, (char)87, (char)3, (char)223, (char)167, (char)134, (char)70, (char)199, (char)42, (char)205, (char)181, (char)194, (char)232, (char)241, (char)214, (char)148, (char)215, (char)130, (char)47, (char)182, (char)75, (char)88, (char)212, (char)68, (char)135, (char)52, (char)188, (char)213, (char)162, (char)252, (char)152, (char)37}, 0) ;
        p259.cam_definition_version_SET((char)48285) ;
        p259.firmware_version_SET(2904989794L) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE) ;
        p259.lens_id_SET((char)67) ;
        p259.vendor_name_SET(new char[] {(char)99, (char)128, (char)27, (char)123, (char)122, (char)45, (char)129, (char)132, (char)248, (char)121, (char)79, (char)131, (char)140, (char)126, (char)125, (char)22, (char)97, (char)144, (char)27, (char)40, (char)43, (char)19, (char)40, (char)106, (char)201, (char)2, (char)76, (char)58, (char)15, (char)162, (char)58, (char)103}, 0) ;
        p259.focal_length_SET(-2.8566562E37F) ;
        p259.resolution_h_SET((char)13728) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4046458026L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(4046458026L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)76);
            assert(pack.storage_id_GET() == (char)138);
            assert(pack.total_capacity_GET() == 7.099919E37F);
            assert(pack.used_capacity_GET() == -2.1529358E38F);
            assert(pack.read_speed_GET() == -7.3369507E37F);
            assert(pack.storage_count_GET() == (char)36);
            assert(pack.available_capacity_GET() == 1.2514176E38F);
            assert(pack.time_boot_ms_GET() == 3515071018L);
            assert(pack.write_speed_GET() == 8.96292E34F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.read_speed_SET(-7.3369507E37F) ;
        p261.used_capacity_SET(-2.1529358E38F) ;
        p261.time_boot_ms_SET(3515071018L) ;
        p261.storage_id_SET((char)138) ;
        p261.available_capacity_SET(1.2514176E38F) ;
        p261.status_SET((char)76) ;
        p261.storage_count_SET((char)36) ;
        p261.write_speed_SET(8.96292E34F) ;
        p261.total_capacity_SET(7.099919E37F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.video_status_GET() == (char)13);
            assert(pack.time_boot_ms_GET() == 514673814L);
            assert(pack.image_interval_GET() == 2.3883018E38F);
            assert(pack.image_status_GET() == (char)223);
            assert(pack.available_capacity_GET() == -1.6749338E38F);
            assert(pack.recording_time_ms_GET() == 3293314502L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(514673814L) ;
        p262.image_status_SET((char)223) ;
        p262.video_status_SET((char)13) ;
        p262.recording_time_ms_SET(3293314502L) ;
        p262.available_capacity_SET(-1.6749338E38F) ;
        p262.image_interval_SET(2.3883018E38F) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.capture_result_GET() == (byte) - 18);
            assert(pack.camera_id_GET() == (char)196);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.4361057E38F, 2.2018034E38F, -1.2913522E38F, -2.0571909E38F}));
            assert(pack.time_utc_GET() == 5835492454419974982L);
            assert(pack.alt_GET() == -1231466755);
            assert(pack.file_url_LEN(ph) == 18);
            assert(pack.file_url_TRY(ph).equals("ppdfuasxekmpnmaGzh"));
            assert(pack.lon_GET() == -481080713);
            assert(pack.relative_alt_GET() == 249583116);
            assert(pack.image_index_GET() == 2097797886);
            assert(pack.lat_GET() == 809012139);
            assert(pack.time_boot_ms_GET() == 1956567988L);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.file_url_SET("ppdfuasxekmpnmaGzh", PH) ;
        p263.capture_result_SET((byte) - 18) ;
        p263.relative_alt_SET(249583116) ;
        p263.camera_id_SET((char)196) ;
        p263.alt_SET(-1231466755) ;
        p263.q_SET(new float[] {1.4361057E38F, 2.2018034E38F, -1.2913522E38F, -2.0571909E38F}, 0) ;
        p263.time_boot_ms_SET(1956567988L) ;
        p263.lon_SET(-481080713) ;
        p263.image_index_SET(2097797886) ;
        p263.lat_SET(809012139) ;
        p263.time_utc_SET(5835492454419974982L) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 2460460184827885926L);
            assert(pack.time_boot_ms_GET() == 873318923L);
            assert(pack.takeoff_time_utc_GET() == 2368171486018748977L);
            assert(pack.flight_uuid_GET() == 1301836091382591946L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(2368171486018748977L) ;
        p264.time_boot_ms_SET(873318923L) ;
        p264.arming_time_utc_SET(2460460184827885926L) ;
        p264.flight_uuid_SET(1301836091382591946L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.2518393E38F);
            assert(pack.roll_GET() == 1.8279986E38F);
            assert(pack.yaw_GET() == 3.0133909E38F);
            assert(pack.time_boot_ms_GET() == 2641855669L);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(1.8279986E38F) ;
        p265.pitch_SET(2.2518393E38F) ;
        p265.time_boot_ms_SET(2641855669L) ;
        p265.yaw_SET(3.0133909E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)89);
            assert(pack.target_system_GET() == (char)167);
            assert(pack.length_GET() == (char)136);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)116, (char)200, (char)44, (char)56, (char)228, (char)57, (char)100, (char)179, (char)218, (char)48, (char)50, (char)252, (char)229, (char)20, (char)245, (char)126, (char)105, (char)232, (char)64, (char)239, (char)60, (char)219, (char)55, (char)66, (char)204, (char)10, (char)206, (char)34, (char)188, (char)201, (char)246, (char)229, (char)250, (char)213, (char)98, (char)209, (char)109, (char)151, (char)245, (char)16, (char)248, (char)214, (char)113, (char)207, (char)244, (char)94, (char)96, (char)73, (char)20, (char)2, (char)115, (char)82, (char)244, (char)65, (char)85, (char)125, (char)224, (char)200, (char)247, (char)162, (char)234, (char)59, (char)108, (char)243, (char)48, (char)71, (char)38, (char)244, (char)28, (char)114, (char)12, (char)182, (char)122, (char)103, (char)29, (char)194, (char)200, (char)28, (char)30, (char)232, (char)64, (char)80, (char)148, (char)132, (char)123, (char)204, (char)109, (char)215, (char)20, (char)124, (char)4, (char)101, (char)80, (char)107, (char)71, (char)151, (char)112, (char)226, (char)171, (char)64, (char)123, (char)227, (char)52, (char)13, (char)107, (char)202, (char)202, (char)85, (char)62, (char)37, (char)215, (char)170, (char)165, (char)83, (char)140, (char)224, (char)112, (char)48, (char)238, (char)158, (char)136, (char)197, (char)12, (char)194, (char)216, (char)228, (char)50, (char)157, (char)240, (char)222, (char)227, (char)134, (char)165, (char)234, (char)61, (char)197, (char)51, (char)243, (char)166, (char)41, (char)155, (char)110, (char)62, (char)243, (char)17, (char)254, (char)121, (char)65, (char)112, (char)195, (char)166, (char)156, (char)190, (char)17, (char)133, (char)133, (char)66, (char)208, (char)20, (char)111, (char)20, (char)223, (char)29, (char)110, (char)207, (char)236, (char)9, (char)77, (char)234, (char)1, (char)29, (char)108, (char)24, (char)50, (char)214, (char)159, (char)43, (char)152, (char)29, (char)15, (char)88, (char)129, (char)34, (char)250, (char)228, (char)160, (char)36, (char)182, (char)40, (char)246, (char)174, (char)96, (char)220, (char)27, (char)232, (char)47, (char)255, (char)114, (char)173, (char)59, (char)147, (char)56, (char)176, (char)240, (char)173, (char)215, (char)70, (char)10, (char)59, (char)182, (char)188, (char)220, (char)58, (char)244, (char)46, (char)39, (char)152, (char)143, (char)94, (char)173, (char)157, (char)25, (char)225, (char)71, (char)47, (char)45, (char)218, (char)85, (char)190, (char)37, (char)243, (char)24, (char)217, (char)170, (char)145, (char)232, (char)217, (char)19, (char)117, (char)8, (char)215, (char)58, (char)64, (char)147, (char)60, (char)23, (char)87, (char)50, (char)116}));
            assert(pack.sequence_GET() == (char)48250);
            assert(pack.target_component_GET() == (char)214);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.length_SET((char)136) ;
        p266.data__SET(new char[] {(char)116, (char)200, (char)44, (char)56, (char)228, (char)57, (char)100, (char)179, (char)218, (char)48, (char)50, (char)252, (char)229, (char)20, (char)245, (char)126, (char)105, (char)232, (char)64, (char)239, (char)60, (char)219, (char)55, (char)66, (char)204, (char)10, (char)206, (char)34, (char)188, (char)201, (char)246, (char)229, (char)250, (char)213, (char)98, (char)209, (char)109, (char)151, (char)245, (char)16, (char)248, (char)214, (char)113, (char)207, (char)244, (char)94, (char)96, (char)73, (char)20, (char)2, (char)115, (char)82, (char)244, (char)65, (char)85, (char)125, (char)224, (char)200, (char)247, (char)162, (char)234, (char)59, (char)108, (char)243, (char)48, (char)71, (char)38, (char)244, (char)28, (char)114, (char)12, (char)182, (char)122, (char)103, (char)29, (char)194, (char)200, (char)28, (char)30, (char)232, (char)64, (char)80, (char)148, (char)132, (char)123, (char)204, (char)109, (char)215, (char)20, (char)124, (char)4, (char)101, (char)80, (char)107, (char)71, (char)151, (char)112, (char)226, (char)171, (char)64, (char)123, (char)227, (char)52, (char)13, (char)107, (char)202, (char)202, (char)85, (char)62, (char)37, (char)215, (char)170, (char)165, (char)83, (char)140, (char)224, (char)112, (char)48, (char)238, (char)158, (char)136, (char)197, (char)12, (char)194, (char)216, (char)228, (char)50, (char)157, (char)240, (char)222, (char)227, (char)134, (char)165, (char)234, (char)61, (char)197, (char)51, (char)243, (char)166, (char)41, (char)155, (char)110, (char)62, (char)243, (char)17, (char)254, (char)121, (char)65, (char)112, (char)195, (char)166, (char)156, (char)190, (char)17, (char)133, (char)133, (char)66, (char)208, (char)20, (char)111, (char)20, (char)223, (char)29, (char)110, (char)207, (char)236, (char)9, (char)77, (char)234, (char)1, (char)29, (char)108, (char)24, (char)50, (char)214, (char)159, (char)43, (char)152, (char)29, (char)15, (char)88, (char)129, (char)34, (char)250, (char)228, (char)160, (char)36, (char)182, (char)40, (char)246, (char)174, (char)96, (char)220, (char)27, (char)232, (char)47, (char)255, (char)114, (char)173, (char)59, (char)147, (char)56, (char)176, (char)240, (char)173, (char)215, (char)70, (char)10, (char)59, (char)182, (char)188, (char)220, (char)58, (char)244, (char)46, (char)39, (char)152, (char)143, (char)94, (char)173, (char)157, (char)25, (char)225, (char)71, (char)47, (char)45, (char)218, (char)85, (char)190, (char)37, (char)243, (char)24, (char)217, (char)170, (char)145, (char)232, (char)217, (char)19, (char)117, (char)8, (char)215, (char)58, (char)64, (char)147, (char)60, (char)23, (char)87, (char)50, (char)116}, 0) ;
        p266.target_component_SET((char)214) ;
        p266.first_message_offset_SET((char)89) ;
        p266.sequence_SET((char)48250) ;
        p266.target_system_SET((char)167) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)89);
            assert(pack.target_system_GET() == (char)216);
            assert(pack.length_GET() == (char)163);
            assert(pack.target_component_GET() == (char)248);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)76, (char)93, (char)207, (char)110, (char)228, (char)38, (char)30, (char)227, (char)156, (char)219, (char)17, (char)10, (char)28, (char)143, (char)81, (char)238, (char)1, (char)115, (char)253, (char)74, (char)1, (char)139, (char)99, (char)98, (char)8, (char)42, (char)31, (char)71, (char)171, (char)61, (char)65, (char)215, (char)172, (char)113, (char)32, (char)63, (char)29, (char)252, (char)179, (char)222, (char)62, (char)204, (char)62, (char)8, (char)69, (char)55, (char)221, (char)41, (char)186, (char)201, (char)46, (char)113, (char)26, (char)84, (char)41, (char)23, (char)211, (char)75, (char)228, (char)25, (char)129, (char)187, (char)204, (char)143, (char)194, (char)9, (char)107, (char)197, (char)165, (char)184, (char)204, (char)22, (char)229, (char)218, (char)38, (char)17, (char)201, (char)141, (char)90, (char)234, (char)36, (char)50, (char)68, (char)110, (char)151, (char)129, (char)140, (char)199, (char)240, (char)255, (char)134, (char)115, (char)33, (char)77, (char)181, (char)15, (char)109, (char)101, (char)93, (char)215, (char)205, (char)250, (char)159, (char)103, (char)173, (char)46, (char)35, (char)100, (char)9, (char)215, (char)68, (char)182, (char)115, (char)237, (char)56, (char)51, (char)56, (char)225, (char)47, (char)13, (char)200, (char)197, (char)237, (char)164, (char)126, (char)240, (char)77, (char)195, (char)247, (char)223, (char)36, (char)73, (char)180, (char)84, (char)35, (char)101, (char)255, (char)249, (char)236, (char)238, (char)220, (char)69, (char)111, (char)20, (char)194, (char)201, (char)54, (char)39, (char)76, (char)76, (char)37, (char)188, (char)227, (char)169, (char)2, (char)65, (char)177, (char)113, (char)68, (char)183, (char)223, (char)29, (char)101, (char)131, (char)131, (char)163, (char)72, (char)224, (char)104, (char)83, (char)160, (char)80, (char)130, (char)101, (char)17, (char)138, (char)19, (char)172, (char)196, (char)161, (char)30, (char)60, (char)12, (char)243, (char)8, (char)70, (char)44, (char)254, (char)242, (char)75, (char)91, (char)75, (char)87, (char)34, (char)149, (char)116, (char)126, (char)165, (char)205, (char)226, (char)178, (char)236, (char)224, (char)204, (char)84, (char)178, (char)109, (char)102, (char)82, (char)185, (char)98, (char)4, (char)109, (char)182, (char)141, (char)145, (char)143, (char)39, (char)18, (char)192, (char)42, (char)228, (char)88, (char)80, (char)2, (char)151, (char)30, (char)201, (char)5, (char)255, (char)2, (char)249, (char)136, (char)235, (char)99, (char)255, (char)58, (char)65, (char)96, (char)148, (char)103, (char)188, (char)83, (char)158, (char)172, (char)61, (char)145, (char)49, (char)163}));
            assert(pack.sequence_GET() == (char)14931);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.length_SET((char)163) ;
        p267.first_message_offset_SET((char)89) ;
        p267.sequence_SET((char)14931) ;
        p267.data__SET(new char[] {(char)76, (char)93, (char)207, (char)110, (char)228, (char)38, (char)30, (char)227, (char)156, (char)219, (char)17, (char)10, (char)28, (char)143, (char)81, (char)238, (char)1, (char)115, (char)253, (char)74, (char)1, (char)139, (char)99, (char)98, (char)8, (char)42, (char)31, (char)71, (char)171, (char)61, (char)65, (char)215, (char)172, (char)113, (char)32, (char)63, (char)29, (char)252, (char)179, (char)222, (char)62, (char)204, (char)62, (char)8, (char)69, (char)55, (char)221, (char)41, (char)186, (char)201, (char)46, (char)113, (char)26, (char)84, (char)41, (char)23, (char)211, (char)75, (char)228, (char)25, (char)129, (char)187, (char)204, (char)143, (char)194, (char)9, (char)107, (char)197, (char)165, (char)184, (char)204, (char)22, (char)229, (char)218, (char)38, (char)17, (char)201, (char)141, (char)90, (char)234, (char)36, (char)50, (char)68, (char)110, (char)151, (char)129, (char)140, (char)199, (char)240, (char)255, (char)134, (char)115, (char)33, (char)77, (char)181, (char)15, (char)109, (char)101, (char)93, (char)215, (char)205, (char)250, (char)159, (char)103, (char)173, (char)46, (char)35, (char)100, (char)9, (char)215, (char)68, (char)182, (char)115, (char)237, (char)56, (char)51, (char)56, (char)225, (char)47, (char)13, (char)200, (char)197, (char)237, (char)164, (char)126, (char)240, (char)77, (char)195, (char)247, (char)223, (char)36, (char)73, (char)180, (char)84, (char)35, (char)101, (char)255, (char)249, (char)236, (char)238, (char)220, (char)69, (char)111, (char)20, (char)194, (char)201, (char)54, (char)39, (char)76, (char)76, (char)37, (char)188, (char)227, (char)169, (char)2, (char)65, (char)177, (char)113, (char)68, (char)183, (char)223, (char)29, (char)101, (char)131, (char)131, (char)163, (char)72, (char)224, (char)104, (char)83, (char)160, (char)80, (char)130, (char)101, (char)17, (char)138, (char)19, (char)172, (char)196, (char)161, (char)30, (char)60, (char)12, (char)243, (char)8, (char)70, (char)44, (char)254, (char)242, (char)75, (char)91, (char)75, (char)87, (char)34, (char)149, (char)116, (char)126, (char)165, (char)205, (char)226, (char)178, (char)236, (char)224, (char)204, (char)84, (char)178, (char)109, (char)102, (char)82, (char)185, (char)98, (char)4, (char)109, (char)182, (char)141, (char)145, (char)143, (char)39, (char)18, (char)192, (char)42, (char)228, (char)88, (char)80, (char)2, (char)151, (char)30, (char)201, (char)5, (char)255, (char)2, (char)249, (char)136, (char)235, (char)99, (char)255, (char)58, (char)65, (char)96, (char)148, (char)103, (char)188, (char)83, (char)158, (char)172, (char)61, (char)145, (char)49, (char)163}, 0) ;
        p267.target_component_SET((char)248) ;
        p267.target_system_SET((char)216) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)60194);
            assert(pack.target_component_GET() == (char)19);
            assert(pack.target_system_GET() == (char)2);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.sequence_SET((char)60194) ;
        p268.target_system_SET((char)2) ;
        p268.target_component_SET((char)19) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -6.33976E37F);
            assert(pack.bitrate_GET() == 1000322499L);
            assert(pack.rotation_GET() == (char)55398);
            assert(pack.status_GET() == (char)249);
            assert(pack.uri_LEN(ph) == 98);
            assert(pack.uri_TRY(ph).equals("glgikmiypYgofhgibceouyoUCaksRprxuOwmvqxakmlinOiFgaacsixIyraolSeeqahwgqqambkbhkcdqrfunrgbunqdqnylnB"));
            assert(pack.camera_id_GET() == (char)7);
            assert(pack.resolution_v_GET() == (char)64227);
            assert(pack.resolution_h_GET() == (char)50957);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.status_SET((char)249) ;
        p269.camera_id_SET((char)7) ;
        p269.uri_SET("glgikmiypYgofhgibceouyoUCaksRprxuOwmvqxakmlinOiFgaacsixIyraolSeeqahwgqqambkbhkcdqrfunrgbunqdqnylnB", PH) ;
        p269.resolution_h_SET((char)50957) ;
        p269.framerate_SET(-6.33976E37F) ;
        p269.rotation_SET((char)55398) ;
        p269.resolution_v_SET((char)64227) ;
        p269.bitrate_SET(1000322499L) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 135);
            assert(pack.uri_TRY(ph).equals("sosvywtsbcfsnHGXceecjjswgvsvsepdixqrfccnxjlfEkpxhkuxfvkjtgkzhvbksEybvhrtelrjfvumrkyrvcpirlCaxwbsomzdqzldacpbluzmgEujpeotlheoFbbJlpynrpc"));
            assert(pack.target_system_GET() == (char)236);
            assert(pack.resolution_h_GET() == (char)62714);
            assert(pack.bitrate_GET() == 2129549910L);
            assert(pack.rotation_GET() == (char)35586);
            assert(pack.resolution_v_GET() == (char)35009);
            assert(pack.camera_id_GET() == (char)130);
            assert(pack.framerate_GET() == -2.8146823E38F);
            assert(pack.target_component_GET() == (char)134);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_v_SET((char)35009) ;
        p270.framerate_SET(-2.8146823E38F) ;
        p270.target_system_SET((char)236) ;
        p270.camera_id_SET((char)130) ;
        p270.target_component_SET((char)134) ;
        p270.rotation_SET((char)35586) ;
        p270.resolution_h_SET((char)62714) ;
        p270.uri_SET("sosvywtsbcfsnHGXceecjjswgvsvsepdixqrfccnxjlfEkpxhkuxfvkjtgkzhvbksEybvhrtelrjfvumrkyrvcpirlCaxwbsomzdqzldacpbluzmgEujpeotlheoFbbJlpynrpc", PH) ;
        p270.bitrate_SET(2129549910L) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 9);
            assert(pack.ssid_TRY(ph).equals("WcRpcgYlq"));
            assert(pack.password_LEN(ph) == 62);
            assert(pack.password_TRY(ph).equals("hcxISaeustaEizhozzcwgomRqYotnneSeoaxgzpcdlcoxtJrzxomnhvyzbyebk"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("hcxISaeustaEizhozzcwgomRqYotnneSeoaxgzpcdlcoxtJrzxomnhvyzbyebk", PH) ;
        p299.ssid_SET("WcRpcgYlq", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)66, (char)71, (char)225, (char)144, (char)51, (char)23, (char)164, (char)32}));
            assert(pack.min_version_GET() == (char)37114);
            assert(pack.max_version_GET() == (char)41637);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)15, (char)99, (char)83, (char)73, (char)66, (char)75, (char)252, (char)209}));
            assert(pack.version_GET() == (char)59159);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)66, (char)71, (char)225, (char)144, (char)51, (char)23, (char)164, (char)32}, 0) ;
        p300.version_SET((char)59159) ;
        p300.spec_version_hash_SET(new char[] {(char)15, (char)99, (char)83, (char)73, (char)66, (char)75, (char)252, (char)209}, 0) ;
        p300.min_version_SET((char)37114) ;
        p300.max_version_SET((char)41637) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.vendor_specific_status_code_GET() == (char)6112);
            assert(pack.sub_mode_GET() == (char)236);
            assert(pack.uptime_sec_GET() == 3006857683L);
            assert(pack.time_usec_GET() == 7315810974602713909L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        p310.vendor_specific_status_code_SET((char)6112) ;
        p310.time_usec_SET(7315810974602713909L) ;
        p310.uptime_sec_SET(3006857683L) ;
        p310.sub_mode_SET((char)236) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)50);
            assert(pack.sw_vcs_commit_GET() == 1092068069L);
            assert(pack.sw_version_major_GET() == (char)164);
            assert(pack.time_usec_GET() == 6362705579100616230L);
            assert(pack.uptime_sec_GET() == 2653023940L);
            assert(pack.hw_version_major_GET() == (char)228);
            assert(pack.hw_version_minor_GET() == (char)157);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)182, (char)190, (char)249, (char)240, (char)62, (char)101, (char)235, (char)108, (char)139, (char)211, (char)175, (char)152, (char)92, (char)51, (char)233, (char)11}));
            assert(pack.name_LEN(ph) == 15);
            assert(pack.name_TRY(ph).equals("qmwXanmwzywtbMp"));
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)228) ;
        p311.sw_version_minor_SET((char)50) ;
        p311.sw_version_major_SET((char)164) ;
        p311.uptime_sec_SET(2653023940L) ;
        p311.sw_vcs_commit_SET(1092068069L) ;
        p311.hw_unique_id_SET(new char[] {(char)182, (char)190, (char)249, (char)240, (char)62, (char)101, (char)235, (char)108, (char)139, (char)211, (char)175, (char)152, (char)92, (char)51, (char)233, (char)11}, 0) ;
        p311.hw_version_minor_SET((char)157) ;
        p311.time_usec_SET(6362705579100616230L) ;
        p311.name_SET("qmwXanmwzywtbMp", PH) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short)32049);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("C"));
            assert(pack.target_system_GET() == (char)193);
            assert(pack.target_component_GET() == (char)110);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("C", PH) ;
        p320.param_index_SET((short)32049) ;
        p320.target_component_SET((char)110) ;
        p320.target_system_SET((char)193) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)9);
            assert(pack.target_system_GET() == (char)68);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)9) ;
        p321.target_system_SET((char)68) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)58209);
            assert(pack.param_index_GET() == (char)21093);
            assert(pack.param_value_LEN(ph) == 11);
            assert(pack.param_value_TRY(ph).equals("Cnwxzawxcpr"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("ryupJznsd"));
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)21093) ;
        p322.param_count_SET((char)58209) ;
        p322.param_value_SET("Cnwxzawxcpr", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p322.param_id_SET("ryupJznsd", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)11);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.target_system_GET() == (char)109);
            assert(pack.param_value_LEN(ph) == 70);
            assert(pack.param_value_TRY(ph).equals("gvfjkywpeatwpphuZzfzujaNeqJgexsfggOqwuxbjhtvolcwsivnbrieynqoyivueUyihd"));
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("ypnbSprTpcnwb"));
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p323.target_system_SET((char)109) ;
        p323.param_value_SET("gvfjkywpeatwpphuZzfzujaNeqJgexsfggOqwuxbjhtvolcwsivnbrieynqoyivueUyihd", PH) ;
        p323.target_component_SET((char)11) ;
        p323.param_id_SET("ypnbSprTpcnwb", PH) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("wbuo"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_value_LEN(ph) == 79);
            assert(pack.param_value_TRY(ph).equals("hvlxdpvetVbofupTzabdtommmbjseYgqaqcnpgkvxvpbicgYOjxpczmnyjczAfjHsgkpdsbdstimeny"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p324.param_value_SET("hvlxdpvetVbofupTzabdtommmbjseYgqaqcnpgkvxvpbicgYOjxpczmnyjczAfjHsgkpdsbdstimeny", PH) ;
        p324.param_id_SET("wbuo", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.increment_GET() == (char)23);
            assert(pack.min_distance_GET() == (char)41862);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)5113, (char)13572, (char)62280, (char)34240, (char)41407, (char)31747, (char)30482, (char)51411, (char)16197, (char)42052, (char)58665, (char)38162, (char)16547, (char)8322, (char)43680, (char)53474, (char)31133, (char)13290, (char)34359, (char)63272, (char)6972, (char)32422, (char)32978, (char)1016, (char)12505, (char)21422, (char)37031, (char)35312, (char)29331, (char)51537, (char)47190, (char)26437, (char)33893, (char)32691, (char)6261, (char)63330, (char)43720, (char)39728, (char)3291, (char)12974, (char)41501, (char)43705, (char)47911, (char)6271, (char)20491, (char)8225, (char)14784, (char)27592, (char)10350, (char)51652, (char)35443, (char)6070, (char)35660, (char)19600, (char)48054, (char)56772, (char)11941, (char)28915, (char)43546, (char)42681, (char)52924, (char)62507, (char)48708, (char)18824, (char)3171, (char)13057, (char)41334, (char)58232, (char)7432, (char)21773, (char)7721, (char)27537}));
            assert(pack.max_distance_GET() == (char)3852);
            assert(pack.time_usec_GET() == 6939351331212088790L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.increment_SET((char)23) ;
        p330.max_distance_SET((char)3852) ;
        p330.min_distance_SET((char)41862) ;
        p330.time_usec_SET(6939351331212088790L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.distances_SET(new char[] {(char)5113, (char)13572, (char)62280, (char)34240, (char)41407, (char)31747, (char)30482, (char)51411, (char)16197, (char)42052, (char)58665, (char)38162, (char)16547, (char)8322, (char)43680, (char)53474, (char)31133, (char)13290, (char)34359, (char)63272, (char)6972, (char)32422, (char)32978, (char)1016, (char)12505, (char)21422, (char)37031, (char)35312, (char)29331, (char)51537, (char)47190, (char)26437, (char)33893, (char)32691, (char)6261, (char)63330, (char)43720, (char)39728, (char)3291, (char)12974, (char)41501, (char)43705, (char)47911, (char)6271, (char)20491, (char)8225, (char)14784, (char)27592, (char)10350, (char)51652, (char)35443, (char)6070, (char)35660, (char)19600, (char)48054, (char)56772, (char)11941, (char)28915, (char)43546, (char)42681, (char)52924, (char)62507, (char)48708, (char)18824, (char)3171, (char)13057, (char)41334, (char)58232, (char)7432, (char)21773, (char)7721, (char)27537}, 0) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
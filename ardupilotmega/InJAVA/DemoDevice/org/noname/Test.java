
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
            assert(pack.mavlink_version_GET() == (char)156);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            assert(pack.custom_mode_GET() == 3693799082L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_OCTOROTOR);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_ARMAZILA);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_STANDBY);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.mavlink_version_SET((char)156) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_ARMAZILA) ;
        p0.custom_mode_SET(3693799082L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_OCTOROTOR) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            assert(pack.current_battery_GET() == (short) -5618);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
            assert(pack.errors_count1_GET() == (char)63600);
            assert(pack.battery_remaining_GET() == (byte) - 74);
            assert(pack.voltage_battery_GET() == (char)4887);
            assert(pack.drop_rate_comm_GET() == (char)55376);
            assert(pack.errors_count4_GET() == (char)8199);
            assert(pack.errors_count2_GET() == (char)35700);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
            assert(pack.load_GET() == (char)32644);
            assert(pack.errors_comm_GET() == (char)20702);
            assert(pack.errors_count3_GET() == (char)16704);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)20702) ;
        p1.battery_remaining_SET((byte) - 74) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN) ;
        p1.errors_count3_SET((char)16704) ;
        p1.current_battery_SET((short) -5618) ;
        p1.drop_rate_comm_SET((char)55376) ;
        p1.voltage_battery_SET((char)4887) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION) ;
        p1.load_SET((char)32644) ;
        p1.errors_count4_SET((char)8199) ;
        p1.errors_count2_SET((char)35700) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO) ;
        p1.errors_count1_SET((char)63600) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 596123271L);
            assert(pack.time_unix_usec_GET() == 305347516249217063L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(305347516249217063L) ;
        p2.time_boot_ms_SET(596123271L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -1.8779106E38F);
            assert(pack.y_GET() == -2.3085398E38F);
            assert(pack.vx_GET() == -9.363118E37F);
            assert(pack.afx_GET() == 2.1627342E38F);
            assert(pack.vz_GET() == 4.4064584E37F);
            assert(pack.z_GET() == 3.0894736E38F);
            assert(pack.yaw_rate_GET() == 9.579737E37F);
            assert(pack.time_boot_ms_GET() == 4229435873L);
            assert(pack.x_GET() == -2.0490146E38F);
            assert(pack.type_mask_GET() == (char)55589);
            assert(pack.afy_GET() == -9.964964E37F);
            assert(pack.yaw_GET() == 2.326637E38F);
            assert(pack.afz_GET() == 5.070515E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afz_SET(5.070515E37F) ;
        p3.afx_SET(2.1627342E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.x_SET(-2.0490146E38F) ;
        p3.afy_SET(-9.964964E37F) ;
        p3.z_SET(3.0894736E38F) ;
        p3.yaw_SET(2.326637E38F) ;
        p3.type_mask_SET((char)55589) ;
        p3.vz_SET(4.4064584E37F) ;
        p3.vx_SET(-9.363118E37F) ;
        p3.time_boot_ms_SET(4229435873L) ;
        p3.vy_SET(-1.8779106E38F) ;
        p3.yaw_rate_SET(9.579737E37F) ;
        p3.y_SET(-2.3085398E38F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)164);
            assert(pack.seq_GET() == 251531371L);
            assert(pack.target_system_GET() == (char)117);
            assert(pack.time_usec_GET() == 8115391968254948822L);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(8115391968254948822L) ;
        p4.seq_SET(251531371L) ;
        p4.target_component_SET((char)164) ;
        p4.target_system_SET((char)117) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)53);
            assert(pack.passkey_LEN(ph) == 24);
            assert(pack.passkey_TRY(ph).equals("astxovnvizMavqgpiwAqpnhx"));
            assert(pack.version_GET() == (char)91);
            assert(pack.control_request_GET() == (char)119);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("astxovnvizMavqgpiwAqpnhx", PH) ;
        p5.version_SET((char)91) ;
        p5.target_system_SET((char)53) ;
        p5.control_request_SET((char)119) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)113);
            assert(pack.gcs_system_id_GET() == (char)5);
            assert(pack.ack_GET() == (char)161);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)5) ;
        p6.ack_SET((char)161) ;
        p6.control_request_SET((char)113) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 32);
            assert(pack.key_TRY(ph).equals("pqviumYmdfNcfrissgqcfiaorjkiSkyi"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("pqviumYmdfNcfrissgqcfiaorjkiSkyi", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 3794565086L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.target_system_GET() == (char)78);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p11.target_system_SET((char)78) ;
        p11.custom_mode_SET(3794565086L) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)126);
            assert(pack.target_system_GET() == (char)40);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("e"));
            assert(pack.param_index_GET() == (short)6539);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)126) ;
        p20.param_index_SET((short)6539) ;
        p20.target_system_SET((char)40) ;
        p20.param_id_SET("e", PH) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)167);
            assert(pack.target_component_GET() == (char)177);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)167) ;
        p21.target_component_SET((char)177) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)44283);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("jurekkgo"));
            assert(pack.param_value_GET() == -1.2984633E38F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            assert(pack.param_index_GET() == (char)9748);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32) ;
        p22.param_count_SET((char)44283) ;
        p22.param_id_SET("jurekkgo", PH) ;
        p22.param_value_SET(-1.2984633E38F) ;
        p22.param_index_SET((char)9748) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.param_value_GET() == -1.4125652E38F);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("ifkwgmkbo"));
            assert(pack.target_component_GET() == (char)95);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_component_SET((char)95) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.param_value_SET(-1.4125652E38F) ;
        p23.target_system_SET((char)175) ;
        p23.param_id_SET("ifkwgmkbo", PH) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.v_acc_TRY(ph) == 479297975L);
            assert(pack.satellites_visible_GET() == (char)132);
            assert(pack.alt_ellipsoid_TRY(ph) == 975786468);
            assert(pack.lat_GET() == -1052187683);
            assert(pack.hdg_acc_TRY(ph) == 937865191L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
            assert(pack.time_usec_GET() == 6495465995088043005L);
            assert(pack.epv_GET() == (char)44274);
            assert(pack.h_acc_TRY(ph) == 206894474L);
            assert(pack.eph_GET() == (char)33565);
            assert(pack.cog_GET() == (char)28262);
            assert(pack.vel_acc_TRY(ph) == 198004399L);
            assert(pack.vel_GET() == (char)39410);
            assert(pack.lon_GET() == -1999744744);
            assert(pack.alt_GET() == 1336402554);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.lon_SET(-1999744744) ;
        p24.vel_acc_SET(198004399L, PH) ;
        p24.alt_ellipsoid_SET(975786468, PH) ;
        p24.cog_SET((char)28262) ;
        p24.time_usec_SET(6495465995088043005L) ;
        p24.eph_SET((char)33565) ;
        p24.epv_SET((char)44274) ;
        p24.satellites_visible_SET((char)132) ;
        p24.lat_SET(-1052187683) ;
        p24.vel_SET((char)39410) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC) ;
        p24.v_acc_SET(479297975L, PH) ;
        p24.alt_SET(1336402554) ;
        p24.h_acc_SET(206894474L, PH) ;
        p24.hdg_acc_SET(937865191L, PH) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)89);
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)151, (char)121, (char)31, (char)69, (char)245, (char)135, (char)203, (char)192, (char)239, (char)201, (char)212, (char)122, (char)80, (char)169, (char)130, (char)25, (char)98, (char)228, (char)116, (char)106}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)255, (char)214, (char)230, (char)152, (char)19, (char)43, (char)9, (char)164, (char)63, (char)111, (char)136, (char)152, (char)128, (char)190, (char)25, (char)10, (char)202, (char)207, (char)220, (char)217}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)156, (char)187, (char)160, (char)71, (char)163, (char)188, (char)121, (char)142, (char)126, (char)228, (char)55, (char)20, (char)29, (char)1, (char)139, (char)119, (char)0, (char)201, (char)231, (char)57}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)56, (char)123, (char)141, (char)245, (char)204, (char)127, (char)49, (char)135, (char)147, (char)40, (char)186, (char)27, (char)72, (char)211, (char)37, (char)57, (char)75, (char)247, (char)190, (char)68}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)65, (char)39, (char)139, (char)159, (char)124, (char)94, (char)169, (char)28, (char)243, (char)172, (char)39, (char)151, (char)76, (char)100, (char)171, (char)137, (char)126, (char)9, (char)239, (char)198}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)56, (char)123, (char)141, (char)245, (char)204, (char)127, (char)49, (char)135, (char)147, (char)40, (char)186, (char)27, (char)72, (char)211, (char)37, (char)57, (char)75, (char)247, (char)190, (char)68}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)151, (char)121, (char)31, (char)69, (char)245, (char)135, (char)203, (char)192, (char)239, (char)201, (char)212, (char)122, (char)80, (char)169, (char)130, (char)25, (char)98, (char)228, (char)116, (char)106}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)156, (char)187, (char)160, (char)71, (char)163, (char)188, (char)121, (char)142, (char)126, (char)228, (char)55, (char)20, (char)29, (char)1, (char)139, (char)119, (char)0, (char)201, (char)231, (char)57}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)65, (char)39, (char)139, (char)159, (char)124, (char)94, (char)169, (char)28, (char)243, (char)172, (char)39, (char)151, (char)76, (char)100, (char)171, (char)137, (char)126, (char)9, (char)239, (char)198}, 0) ;
        p25.satellites_visible_SET((char)89) ;
        p25.satellite_used_SET(new char[] {(char)255, (char)214, (char)230, (char)152, (char)19, (char)43, (char)9, (char)164, (char)63, (char)111, (char)136, (char)152, (char)128, (char)190, (char)25, (char)10, (char)202, (char)207, (char)220, (char)217}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.ymag_GET() == (short)1276);
            assert(pack.yacc_GET() == (short)18125);
            assert(pack.xacc_GET() == (short) -6259);
            assert(pack.zacc_GET() == (short)19554);
            assert(pack.time_boot_ms_GET() == 3544021141L);
            assert(pack.zmag_GET() == (short) -27531);
            assert(pack.ygyro_GET() == (short) -12802);
            assert(pack.xmag_GET() == (short) -15030);
            assert(pack.zgyro_GET() == (short)1034);
            assert(pack.xgyro_GET() == (short)17881);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.zacc_SET((short)19554) ;
        p26.yacc_SET((short)18125) ;
        p26.time_boot_ms_SET(3544021141L) ;
        p26.ymag_SET((short)1276) ;
        p26.xacc_SET((short) -6259) ;
        p26.zgyro_SET((short)1034) ;
        p26.ygyro_SET((short) -12802) ;
        p26.zmag_SET((short) -27531) ;
        p26.xgyro_SET((short)17881) ;
        p26.xmag_SET((short) -15030) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -17294);
            assert(pack.xgyro_GET() == (short) -2719);
            assert(pack.zmag_GET() == (short)24737);
            assert(pack.xmag_GET() == (short)26940);
            assert(pack.ygyro_GET() == (short)7273);
            assert(pack.time_usec_GET() == 4386383714465368255L);
            assert(pack.xacc_GET() == (short)29681);
            assert(pack.yacc_GET() == (short)4009);
            assert(pack.zgyro_GET() == (short) -15560);
            assert(pack.ymag_GET() == (short)8639);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.zmag_SET((short)24737) ;
        p27.xgyro_SET((short) -2719) ;
        p27.xacc_SET((short)29681) ;
        p27.yacc_SET((short)4009) ;
        p27.time_usec_SET(4386383714465368255L) ;
        p27.zgyro_SET((short) -15560) ;
        p27.ymag_SET((short)8639) ;
        p27.ygyro_SET((short)7273) ;
        p27.xmag_SET((short)26940) ;
        p27.zacc_SET((short) -17294) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff1_GET() == (short) -805);
            assert(pack.time_usec_GET() == 2374797748528166294L);
            assert(pack.temperature_GET() == (short)6560);
            assert(pack.press_diff2_GET() == (short) -94);
            assert(pack.press_abs_GET() == (short) -2680);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff2_SET((short) -94) ;
        p28.time_usec_SET(2374797748528166294L) ;
        p28.press_diff1_SET((short) -805) ;
        p28.temperature_SET((short)6560) ;
        p28.press_abs_SET((short) -2680) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.859273E38F);
            assert(pack.temperature_GET() == (short)7386);
            assert(pack.press_diff_GET() == -3.326978E38F);
            assert(pack.time_boot_ms_GET() == 342654744L);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(342654744L) ;
        p29.press_abs_SET(-2.859273E38F) ;
        p29.temperature_SET((short)7386) ;
        p29.press_diff_SET(-3.326978E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -7.8426135E37F);
            assert(pack.rollspeed_GET() == 1.9012837E37F);
            assert(pack.yawspeed_GET() == 6.3843635E37F);
            assert(pack.pitchspeed_GET() == 5.2175647E37F);
            assert(pack.time_boot_ms_GET() == 2794676413L);
            assert(pack.yaw_GET() == 1.5350462E38F);
            assert(pack.roll_GET() == 2.8562904E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(-7.8426135E37F) ;
        p30.yawspeed_SET(6.3843635E37F) ;
        p30.yaw_SET(1.5350462E38F) ;
        p30.rollspeed_SET(1.9012837E37F) ;
        p30.time_boot_ms_SET(2794676413L) ;
        p30.pitchspeed_SET(5.2175647E37F) ;
        p30.roll_SET(2.8562904E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -1.4182093E38F);
            assert(pack.pitchspeed_GET() == -2.0057046E38F);
            assert(pack.time_boot_ms_GET() == 2297806011L);
            assert(pack.q3_GET() == -1.9728604E38F);
            assert(pack.q4_GET() == 3.9817514E37F);
            assert(pack.q1_GET() == -1.6115922E37F);
            assert(pack.yawspeed_GET() == -2.8413135E38F);
            assert(pack.q2_GET() == -4.438101E37F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q2_SET(-4.438101E37F) ;
        p31.q4_SET(3.9817514E37F) ;
        p31.pitchspeed_SET(-2.0057046E38F) ;
        p31.time_boot_ms_SET(2297806011L) ;
        p31.yawspeed_SET(-2.8413135E38F) ;
        p31.q1_SET(-1.6115922E37F) ;
        p31.rollspeed_SET(-1.4182093E38F) ;
        p31.q3_SET(-1.9728604E38F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 8.663364E37F);
            assert(pack.x_GET() == 9.692785E37F);
            assert(pack.vz_GET() == 2.9240888E38F);
            assert(pack.vx_GET() == -3.3227887E38F);
            assert(pack.z_GET() == -3.363904E38F);
            assert(pack.y_GET() == -2.9911374E38F);
            assert(pack.time_boot_ms_GET() == 3387235253L);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vx_SET(-3.3227887E38F) ;
        p32.z_SET(-3.363904E38F) ;
        p32.time_boot_ms_SET(3387235253L) ;
        p32.vz_SET(2.9240888E38F) ;
        p32.x_SET(9.692785E37F) ;
        p32.y_SET(-2.9911374E38F) ;
        p32.vy_SET(8.663364E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -519159200);
            assert(pack.hdg_GET() == (char)15569);
            assert(pack.alt_GET() == 57398850);
            assert(pack.vz_GET() == (short)27998);
            assert(pack.lon_GET() == 1354597254);
            assert(pack.vx_GET() == (short)10314);
            assert(pack.vy_GET() == (short) -28743);
            assert(pack.relative_alt_GET() == 1601398348);
            assert(pack.time_boot_ms_GET() == 1999482767L);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.alt_SET(57398850) ;
        p33.vy_SET((short) -28743) ;
        p33.vx_SET((short)10314) ;
        p33.lon_SET(1354597254) ;
        p33.time_boot_ms_SET(1999482767L) ;
        p33.vz_SET((short)27998) ;
        p33.lat_SET(-519159200) ;
        p33.hdg_SET((char)15569) ;
        p33.relative_alt_SET(1601398348) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan1_scaled_GET() == (short) -17791);
            assert(pack.chan4_scaled_GET() == (short)4838);
            assert(pack.chan8_scaled_GET() == (short)13664);
            assert(pack.chan2_scaled_GET() == (short)20263);
            assert(pack.chan7_scaled_GET() == (short)6122);
            assert(pack.chan5_scaled_GET() == (short) -11630);
            assert(pack.chan6_scaled_GET() == (short) -9776);
            assert(pack.rssi_GET() == (char)92);
            assert(pack.chan3_scaled_GET() == (short) -31885);
            assert(pack.time_boot_ms_GET() == 2491087059L);
            assert(pack.port_GET() == (char)34);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan3_scaled_SET((short) -31885) ;
        p34.chan2_scaled_SET((short)20263) ;
        p34.chan7_scaled_SET((short)6122) ;
        p34.chan1_scaled_SET((short) -17791) ;
        p34.chan6_scaled_SET((short) -9776) ;
        p34.chan4_scaled_SET((short)4838) ;
        p34.chan5_scaled_SET((short) -11630) ;
        p34.rssi_SET((char)92) ;
        p34.time_boot_ms_SET(2491087059L) ;
        p34.port_SET((char)34) ;
        p34.chan8_scaled_SET((short)13664) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)30606);
            assert(pack.chan6_raw_GET() == (char)5877);
            assert(pack.chan1_raw_GET() == (char)53788);
            assert(pack.chan7_raw_GET() == (char)62579);
            assert(pack.chan5_raw_GET() == (char)53229);
            assert(pack.rssi_GET() == (char)128);
            assert(pack.chan8_raw_GET() == (char)39087);
            assert(pack.chan3_raw_GET() == (char)51428);
            assert(pack.port_GET() == (char)115);
            assert(pack.chan4_raw_GET() == (char)47988);
            assert(pack.time_boot_ms_GET() == 4051607865L);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan7_raw_SET((char)62579) ;
        p35.chan6_raw_SET((char)5877) ;
        p35.chan8_raw_SET((char)39087) ;
        p35.chan2_raw_SET((char)30606) ;
        p35.rssi_SET((char)128) ;
        p35.time_boot_ms_SET(4051607865L) ;
        p35.chan1_raw_SET((char)53788) ;
        p35.chan5_raw_SET((char)53229) ;
        p35.chan4_raw_SET((char)47988) ;
        p35.port_SET((char)115) ;
        p35.chan3_raw_SET((char)51428) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo14_raw_TRY(ph) == (char)5319);
            assert(pack.servo13_raw_TRY(ph) == (char)64420);
            assert(pack.servo11_raw_TRY(ph) == (char)62341);
            assert(pack.servo5_raw_GET() == (char)59175);
            assert(pack.servo16_raw_TRY(ph) == (char)50088);
            assert(pack.time_usec_GET() == 2996628617L);
            assert(pack.servo12_raw_TRY(ph) == (char)24145);
            assert(pack.servo4_raw_GET() == (char)55374);
            assert(pack.port_GET() == (char)21);
            assert(pack.servo9_raw_TRY(ph) == (char)44239);
            assert(pack.servo15_raw_TRY(ph) == (char)50480);
            assert(pack.servo1_raw_GET() == (char)5681);
            assert(pack.servo3_raw_GET() == (char)7008);
            assert(pack.servo6_raw_GET() == (char)17671);
            assert(pack.servo10_raw_TRY(ph) == (char)9662);
            assert(pack.servo8_raw_GET() == (char)28306);
            assert(pack.servo2_raw_GET() == (char)53077);
            assert(pack.servo7_raw_GET() == (char)18711);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo1_raw_SET((char)5681) ;
        p36.servo13_raw_SET((char)64420, PH) ;
        p36.servo11_raw_SET((char)62341, PH) ;
        p36.port_SET((char)21) ;
        p36.servo12_raw_SET((char)24145, PH) ;
        p36.servo5_raw_SET((char)59175) ;
        p36.servo9_raw_SET((char)44239, PH) ;
        p36.servo4_raw_SET((char)55374) ;
        p36.servo2_raw_SET((char)53077) ;
        p36.servo16_raw_SET((char)50088, PH) ;
        p36.servo3_raw_SET((char)7008) ;
        p36.servo6_raw_SET((char)17671) ;
        p36.servo7_raw_SET((char)18711) ;
        p36.servo14_raw_SET((char)5319, PH) ;
        p36.servo15_raw_SET((char)50480, PH) ;
        p36.servo8_raw_SET((char)28306) ;
        p36.servo10_raw_SET((char)9662, PH) ;
        p36.time_usec_SET(2996628617L) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)24981);
            assert(pack.target_system_GET() == (char)253);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)198);
            assert(pack.end_index_GET() == (short)32295);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)253) ;
        p37.end_index_SET((short)32295) ;
        p37.target_component_SET((char)198) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p37.start_index_SET((short)24981) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)25014);
            assert(pack.target_component_GET() == (char)17);
            assert(pack.target_system_GET() == (char)38);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short) -12481);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)38) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p38.start_index_SET((short)25014) ;
        p38.target_component_SET((char)17) ;
        p38.end_index_SET((short) -12481) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == 1.5646473E38F);
            assert(pack.x_GET() == 5.1768766E37F);
            assert(pack.param3_GET() == 1.625321E38F);
            assert(pack.y_GET() == -3.1585038E38F);
            assert(pack.target_component_GET() == (char)50);
            assert(pack.target_system_GET() == (char)97);
            assert(pack.param4_GET() == 1.1227203E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)12878);
            assert(pack.autocontinue_GET() == (char)201);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_RALLY_LAND);
            assert(pack.current_GET() == (char)129);
            assert(pack.param2_GET() == -1.4756245E38F);
            assert(pack.z_GET() == 2.217255E37F);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND) ;
        p39.param1_SET(1.5646473E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.target_component_SET((char)50) ;
        p39.y_SET(-3.1585038E38F) ;
        p39.param4_SET(1.1227203E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p39.target_system_SET((char)97) ;
        p39.z_SET(2.217255E37F) ;
        p39.x_SET(5.1768766E37F) ;
        p39.autocontinue_SET((char)201) ;
        p39.param2_SET(-1.4756245E38F) ;
        p39.param3_SET(1.625321E38F) ;
        p39.current_SET((char)129) ;
        p39.seq_SET((char)12878) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)152);
            assert(pack.seq_GET() == (char)54674);
            assert(pack.target_system_GET() == (char)126);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)126) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.seq_SET((char)54674) ;
        p40.target_component_SET((char)152) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)125);
            assert(pack.target_component_GET() == (char)1);
            assert(pack.seq_GET() == (char)35204);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)125) ;
        p41.target_component_SET((char)1) ;
        p41.seq_SET((char)35204) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)29591);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)29591) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.target_component_GET() == (char)41);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)234) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_component_SET((char)41) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)76);
            assert(pack.count_GET() == (char)60940);
            assert(pack.target_component_GET() == (char)204);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)76) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p44.count_SET((char)60940) ;
        p44.target_component_SET((char)204) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)231);
            assert(pack.target_system_GET() == (char)158);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p45.target_system_SET((char)158) ;
        p45.target_component_SET((char)231) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)48668);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)48668) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)89);
            assert(pack.target_system_GET() == (char)172);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)89) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_system_SET((char)172) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)171);
            assert(pack.altitude_GET() == 1712052011);
            assert(pack.longitude_GET() == 468153961);
            assert(pack.time_usec_TRY(ph) == 1384998944160206650L);
            assert(pack.latitude_GET() == -1854845463);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(1384998944160206650L, PH) ;
        p48.longitude_SET(468153961) ;
        p48.latitude_SET(-1854845463) ;
        p48.altitude_SET(1712052011) ;
        p48.target_system_SET((char)171) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == -1541810334);
            assert(pack.time_usec_TRY(ph) == 8840499993035272702L);
            assert(pack.longitude_GET() == -495809820);
            assert(pack.latitude_GET() == 2076772156);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(2076772156) ;
        p49.time_usec_SET(8840499993035272702L, PH) ;
        p49.longitude_SET(-495809820) ;
        p49.altitude_SET(-1541810334) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_value_max_GET() == -2.2920303E38F);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.param_value_min_GET() == -6.418043E36F);
            assert(pack.param_value0_GET() == -2.262228E38F);
            assert(pack.param_index_GET() == (short)30856);
            assert(pack.scale_GET() == -3.2872723E38F);
            assert(pack.parameter_rc_channel_index_GET() == (char)178);
            assert(pack.target_system_GET() == (char)175);
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("zbAfzj"));
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_component_SET((char)203) ;
        p50.param_index_SET((short)30856) ;
        p50.param_value_min_SET(-6.418043E36F) ;
        p50.target_system_SET((char)175) ;
        p50.param_value0_SET(-2.262228E38F) ;
        p50.parameter_rc_channel_index_SET((char)178) ;
        p50.param_id_SET("zbAfzj", PH) ;
        p50.param_value_max_SET(-2.2920303E38F) ;
        p50.scale_SET(-3.2872723E38F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)44222);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)2);
            assert(pack.target_component_GET() == (char)106);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)106) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_system_SET((char)2) ;
        p51.seq_SET((char)44222) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.p1x_GET() == -2.0196067E38F);
            assert(pack.p2y_GET() == 2.498839E38F);
            assert(pack.p1y_GET() == 2.2825578E38F);
            assert(pack.target_system_GET() == (char)224);
            assert(pack.p2x_GET() == -8.2205316E37F);
            assert(pack.target_component_GET() == (char)126);
            assert(pack.p2z_GET() == 1.4143676E38F);
            assert(pack.p1z_GET() == 3.3330366E38F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p54.target_component_SET((char)126) ;
        p54.target_system_SET((char)224) ;
        p54.p2z_SET(1.4143676E38F) ;
        p54.p2x_SET(-8.2205316E37F) ;
        p54.p2y_SET(2.498839E38F) ;
        p54.p1y_SET(2.2825578E38F) ;
        p54.p1z_SET(3.3330366E38F) ;
        p54.p1x_SET(-2.0196067E38F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 1.2804689E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.p1y_GET() == -4.1604998E37F);
            assert(pack.p2x_GET() == -1.0530337E38F);
            assert(pack.p1z_GET() == 1.3610195E38F);
            assert(pack.p2y_GET() == 1.0253899E38F);
            assert(pack.p2z_GET() == -1.2789408E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p55.p1y_SET(-4.1604998E37F) ;
        p55.p2z_SET(-1.2789408E38F) ;
        p55.p1x_SET(1.2804689E38F) ;
        p55.p2y_SET(1.0253899E38F) ;
        p55.p1z_SET(1.3610195E38F) ;
        p55.p2x_SET(-1.0530337E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {4.0410868E37F, -1.1929078E38F, 1.0070059E38F, -1.6564407E38F, 2.7643841E38F, 3.283994E37F, -1.9691735E38F, 2.58533E38F, -6.6341287E37F}));
            assert(pack.time_usec_GET() == 6604236944356962794L);
            assert(pack.rollspeed_GET() == 6.6048835E37F);
            assert(pack.pitchspeed_GET() == -3.2914753E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3359605E38F, 2.1238441E38F, -1.9535668E38F, -1.9080502E38F}));
            assert(pack.yawspeed_GET() == 2.1005096E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {4.0410868E37F, -1.1929078E38F, 1.0070059E38F, -1.6564407E38F, 2.7643841E38F, 3.283994E37F, -1.9691735E38F, 2.58533E38F, -6.6341287E37F}, 0) ;
        p61.q_SET(new float[] {3.3359605E38F, 2.1238441E38F, -1.9535668E38F, -1.9080502E38F}, 0) ;
        p61.rollspeed_SET(6.6048835E37F) ;
        p61.yawspeed_SET(2.1005096E38F) ;
        p61.time_usec_SET(6604236944356962794L) ;
        p61.pitchspeed_SET(-3.2914753E38F) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.alt_error_GET() == 6.439608E37F);
            assert(pack.wp_dist_GET() == (char)50245);
            assert(pack.nav_bearing_GET() == (short) -16054);
            assert(pack.target_bearing_GET() == (short)24830);
            assert(pack.aspd_error_GET() == 1.3438638E38F);
            assert(pack.nav_pitch_GET() == -6.1513525E36F);
            assert(pack.xtrack_error_GET() == -2.2579614E38F);
            assert(pack.nav_roll_GET() == 5.7666856E37F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(5.7666856E37F) ;
        p62.wp_dist_SET((char)50245) ;
        p62.nav_bearing_SET((short) -16054) ;
        p62.alt_error_SET(6.439608E37F) ;
        p62.target_bearing_SET((short)24830) ;
        p62.nav_pitch_SET(-6.1513525E36F) ;
        p62.aspd_error_SET(1.3438638E38F) ;
        p62.xtrack_error_SET(-2.2579614E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.0411846E38F, 2.8726662E38F, -2.3809216E38F, -2.1977508E38F, 2.4755206E38F, -1.3749617E38F, 3.9458122E37F, 2.3364115E38F, -8.25835E37F, 2.4338756E38F, 2.2528835E38F, -9.803871E37F, -2.1847325E38F, -9.06667E37F, -2.79389E37F, 4.423506E37F, 1.5370378E37F, 3.1115107E38F, 3.0682465E38F, 4.7370835E37F, 2.1432345E38F, 9.42533E37F, 1.4573697E38F, -2.1433668E38F, -1.3943794E38F, 1.7283803E38F, -8.2258314E37F, -1.0217218E37F, -4.614186E37F, -2.2828673E38F, -2.6420443E38F, -2.3260357E37F, 9.534334E37F, 2.9667113E38F, 2.7127944E38F, -2.4102654E38F}));
            assert(pack.vx_GET() == -9.590454E37F);
            assert(pack.alt_GET() == -1969207289);
            assert(pack.vz_GET() == -9.400583E37F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.lon_GET() == 1458592882);
            assert(pack.time_usec_GET() == 8157759343277066858L);
            assert(pack.lat_GET() == -326608500);
            assert(pack.relative_alt_GET() == -1733704602);
            assert(pack.vy_GET() == -2.225122E37F);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.vy_SET(-2.225122E37F) ;
        p63.vz_SET(-9.400583E37F) ;
        p63.covariance_SET(new float[] {1.0411846E38F, 2.8726662E38F, -2.3809216E38F, -2.1977508E38F, 2.4755206E38F, -1.3749617E38F, 3.9458122E37F, 2.3364115E38F, -8.25835E37F, 2.4338756E38F, 2.2528835E38F, -9.803871E37F, -2.1847325E38F, -9.06667E37F, -2.79389E37F, 4.423506E37F, 1.5370378E37F, 3.1115107E38F, 3.0682465E38F, 4.7370835E37F, 2.1432345E38F, 9.42533E37F, 1.4573697E38F, -2.1433668E38F, -1.3943794E38F, 1.7283803E38F, -8.2258314E37F, -1.0217218E37F, -4.614186E37F, -2.2828673E38F, -2.6420443E38F, -2.3260357E37F, 9.534334E37F, 2.9667113E38F, 2.7127944E38F, -2.4102654E38F}, 0) ;
        p63.relative_alt_SET(-1733704602) ;
        p63.lat_SET(-326608500) ;
        p63.lon_SET(1458592882) ;
        p63.time_usec_SET(8157759343277066858L) ;
        p63.alt_SET(-1969207289) ;
        p63.vx_SET(-9.590454E37F) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2794957322054249684L);
            assert(pack.vy_GET() == 2.8553231E38F);
            assert(pack.x_GET() == -3.2220938E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vx_GET() == 1.456645E38F);
            assert(pack.vz_GET() == -2.239639E38F);
            assert(pack.y_GET() == -2.051123E38F);
            assert(pack.az_GET() == 2.7677528E38F);
            assert(pack.ax_GET() == 1.7393694E38F);
            assert(pack.z_GET() == 2.39138E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.3636118E38F, -5.563237E37F, -7.701452E37F, -3.2662819E38F, -1.4361187E38F, -2.1118982E38F, 9.997205E37F, 4.1186597E37F, 9.281401E37F, -3.0282527E37F, -1.6788045E38F, -3.0176907E38F, 7.3447605E37F, 5.0905735E37F, 2.6754608E38F, -7.1480515E37F, -2.3604263E37F, -2.7202528E38F, 2.773805E38F, 2.0013094E38F, 1.6725587E38F, -2.5328026E38F, 2.0286902E38F, 1.9836385E38F, -1.831496E38F, 8.1489134E37F, -1.5387356E38F, -2.323893E38F, -9.284402E35F, -2.664823E37F, -1.7905615E38F, -2.6252022E38F, -1.4418398E38F, -3.1962246E38F, -2.6891118E38F, 3.2613409E38F, -2.1855402E38F, -1.0772162E38F, 1.7276784E38F, -1.4393327E37F, 1.2457523E38F, -9.890394E37F, -1.0090652E38F, -7.116568E37F, -1.6996199E38F}));
            assert(pack.ay_GET() == 2.6073849E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ay_SET(2.6073849E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p64.ax_SET(1.7393694E38F) ;
        p64.z_SET(2.39138E38F) ;
        p64.y_SET(-2.051123E38F) ;
        p64.vx_SET(1.456645E38F) ;
        p64.vz_SET(-2.239639E38F) ;
        p64.covariance_SET(new float[] {-1.3636118E38F, -5.563237E37F, -7.701452E37F, -3.2662819E38F, -1.4361187E38F, -2.1118982E38F, 9.997205E37F, 4.1186597E37F, 9.281401E37F, -3.0282527E37F, -1.6788045E38F, -3.0176907E38F, 7.3447605E37F, 5.0905735E37F, 2.6754608E38F, -7.1480515E37F, -2.3604263E37F, -2.7202528E38F, 2.773805E38F, 2.0013094E38F, 1.6725587E38F, -2.5328026E38F, 2.0286902E38F, 1.9836385E38F, -1.831496E38F, 8.1489134E37F, -1.5387356E38F, -2.323893E38F, -9.284402E35F, -2.664823E37F, -1.7905615E38F, -2.6252022E38F, -1.4418398E38F, -3.1962246E38F, -2.6891118E38F, 3.2613409E38F, -2.1855402E38F, -1.0772162E38F, 1.7276784E38F, -1.4393327E37F, 1.2457523E38F, -9.890394E37F, -1.0090652E38F, -7.116568E37F, -1.6996199E38F}, 0) ;
        p64.vy_SET(2.8553231E38F) ;
        p64.az_SET(2.7677528E38F) ;
        p64.x_SET(-3.2220938E38F) ;
        p64.time_usec_SET(2794957322054249684L) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan18_raw_GET() == (char)4142);
            assert(pack.chan3_raw_GET() == (char)22801);
            assert(pack.chan2_raw_GET() == (char)8823);
            assert(pack.chancount_GET() == (char)1);
            assert(pack.chan17_raw_GET() == (char)12870);
            assert(pack.chan11_raw_GET() == (char)54931);
            assert(pack.chan15_raw_GET() == (char)36622);
            assert(pack.time_boot_ms_GET() == 4135748424L);
            assert(pack.chan1_raw_GET() == (char)4364);
            assert(pack.chan7_raw_GET() == (char)51566);
            assert(pack.chan4_raw_GET() == (char)46552);
            assert(pack.chan10_raw_GET() == (char)43492);
            assert(pack.chan5_raw_GET() == (char)17618);
            assert(pack.chan9_raw_GET() == (char)36114);
            assert(pack.chan8_raw_GET() == (char)59268);
            assert(pack.rssi_GET() == (char)250);
            assert(pack.chan14_raw_GET() == (char)19753);
            assert(pack.chan16_raw_GET() == (char)25015);
            assert(pack.chan13_raw_GET() == (char)8172);
            assert(pack.chan12_raw_GET() == (char)18836);
            assert(pack.chan6_raw_GET() == (char)26039);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan7_raw_SET((char)51566) ;
        p65.chan10_raw_SET((char)43492) ;
        p65.chan9_raw_SET((char)36114) ;
        p65.chan6_raw_SET((char)26039) ;
        p65.chan5_raw_SET((char)17618) ;
        p65.chancount_SET((char)1) ;
        p65.time_boot_ms_SET(4135748424L) ;
        p65.chan13_raw_SET((char)8172) ;
        p65.chan4_raw_SET((char)46552) ;
        p65.chan8_raw_SET((char)59268) ;
        p65.chan11_raw_SET((char)54931) ;
        p65.chan16_raw_SET((char)25015) ;
        p65.chan14_raw_SET((char)19753) ;
        p65.chan18_raw_SET((char)4142) ;
        p65.chan12_raw_SET((char)18836) ;
        p65.chan2_raw_SET((char)8823) ;
        p65.chan17_raw_SET((char)12870) ;
        p65.chan15_raw_SET((char)36622) ;
        p65.chan3_raw_SET((char)22801) ;
        p65.rssi_SET((char)250) ;
        p65.chan1_raw_SET((char)4364) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_message_rate_GET() == (char)21355);
            assert(pack.target_system_GET() == (char)250);
            assert(pack.req_stream_id_GET() == (char)173);
            assert(pack.start_stop_GET() == (char)75);
            assert(pack.target_component_GET() == (char)231);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)21355) ;
        p66.start_stop_SET((char)75) ;
        p66.target_component_SET((char)231) ;
        p66.target_system_SET((char)250) ;
        p66.req_stream_id_SET((char)173) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)73);
            assert(pack.message_rate_GET() == (char)61200);
            assert(pack.on_off_GET() == (char)158);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)73) ;
        p67.on_off_SET((char)158) ;
        p67.message_rate_SET((char)61200) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == (short) -15195);
            assert(pack.buttons_GET() == (char)5520);
            assert(pack.x_GET() == (short) -14915);
            assert(pack.z_GET() == (short)27812);
            assert(pack.target_GET() == (char)19);
            assert(pack.r_GET() == (short) -7693);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.x_SET((short) -14915) ;
        p69.target_SET((char)19) ;
        p69.z_SET((short)27812) ;
        p69.buttons_SET((char)5520) ;
        p69.y_SET((short) -15195) ;
        p69.r_SET((short) -7693) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan6_raw_GET() == (char)11365);
            assert(pack.chan8_raw_GET() == (char)62232);
            assert(pack.chan3_raw_GET() == (char)55752);
            assert(pack.chan4_raw_GET() == (char)61190);
            assert(pack.target_component_GET() == (char)74);
            assert(pack.chan7_raw_GET() == (char)7453);
            assert(pack.chan2_raw_GET() == (char)2350);
            assert(pack.chan5_raw_GET() == (char)39974);
            assert(pack.target_system_GET() == (char)90);
            assert(pack.chan1_raw_GET() == (char)49905);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan8_raw_SET((char)62232) ;
        p70.chan2_raw_SET((char)2350) ;
        p70.chan5_raw_SET((char)39974) ;
        p70.target_component_SET((char)74) ;
        p70.chan6_raw_SET((char)11365) ;
        p70.chan1_raw_SET((char)49905) ;
        p70.target_system_SET((char)90) ;
        p70.chan3_raw_SET((char)55752) ;
        p70.chan4_raw_SET((char)61190) ;
        p70.chan7_raw_SET((char)7453) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)173);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.autocontinue_GET() == (char)184);
            assert(pack.param3_GET() == 1.2038924E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)221);
            assert(pack.param2_GET() == -2.4278845E38F);
            assert(pack.x_GET() == 803557991);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
            assert(pack.y_GET() == -1232453431);
            assert(pack.seq_GET() == (char)14290);
            assert(pack.param1_GET() == -2.770634E38F);
            assert(pack.current_GET() == (char)18);
            assert(pack.param4_GET() == -2.5404793E38F);
            assert(pack.z_GET() == 1.8748972E38F);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)221) ;
        p73.seq_SET((char)14290) ;
        p73.y_SET(-1232453431) ;
        p73.current_SET((char)18) ;
        p73.z_SET(1.8748972E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p73.param2_SET(-2.4278845E38F) ;
        p73.x_SET(803557991) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.target_component_SET((char)173) ;
        p73.param4_SET(-2.5404793E38F) ;
        p73.param1_SET(-2.770634E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM) ;
        p73.autocontinue_SET((char)184) ;
        p73.param3_SET(1.2038924E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.groundspeed_GET() == 6.348336E37F);
            assert(pack.airspeed_GET() == -1.7917837E37F);
            assert(pack.throttle_GET() == (char)5315);
            assert(pack.heading_GET() == (short)8221);
            assert(pack.climb_GET() == -2.1606443E38F);
            assert(pack.alt_GET() == -1.9686103E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(-2.1606443E38F) ;
        p74.heading_SET((short)8221) ;
        p74.alt_SET(-1.9686103E38F) ;
        p74.groundspeed_SET(6.348336E37F) ;
        p74.throttle_SET((char)5315) ;
        p74.airspeed_SET(-1.7917837E37F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.autocontinue_GET() == (char)100);
            assert(pack.param1_GET() == -9.230021E37F);
            assert(pack.param3_GET() == -2.0907274E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.param2_GET() == -1.2057329E38F);
            assert(pack.target_system_GET() == (char)238);
            assert(pack.param4_GET() == 2.4178827E38F);
            assert(pack.x_GET() == -142804913);
            assert(pack.y_GET() == 200081346);
            assert(pack.z_GET() == -1.481658E38F);
            assert(pack.current_GET() == (char)2);
            assert(pack.target_component_GET() == (char)177);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)238) ;
        p75.y_SET(200081346) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p75.param1_SET(-9.230021E37F) ;
        p75.x_SET(-142804913) ;
        p75.z_SET(-1.481658E38F) ;
        p75.param4_SET(2.4178827E38F) ;
        p75.target_component_SET((char)177) ;
        p75.autocontinue_SET((char)100) ;
        p75.current_SET((char)2) ;
        p75.command_SET(MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST) ;
        p75.param2_SET(-1.2057329E38F) ;
        p75.param3_SET(-2.0907274E38F) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)28);
            assert(pack.param4_GET() == -1.8178787E38F);
            assert(pack.param6_GET() == -6.309032E37F);
            assert(pack.target_component_GET() == (char)215);
            assert(pack.confirmation_GET() == (char)200);
            assert(pack.param1_GET() == 3.1037074E38F);
            assert(pack.param7_GET() == 1.2251231E38F);
            assert(pack.param2_GET() == 2.042403E38F);
            assert(pack.param3_GET() == -2.6091136E38F);
            assert(pack.param5_GET() == -3.2094682E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_REPEAT_SERVO);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param4_SET(-1.8178787E38F) ;
        p76.param6_SET(-6.309032E37F) ;
        p76.target_system_SET((char)28) ;
        p76.param5_SET(-3.2094682E38F) ;
        p76.confirmation_SET((char)200) ;
        p76.param2_SET(2.042403E38F) ;
        p76.param1_SET(3.1037074E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_DO_REPEAT_SERVO) ;
        p76.target_component_SET((char)215) ;
        p76.param3_SET(-2.6091136E38F) ;
        p76.param7_SET(1.2251231E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)133);
            assert(pack.result_param2_TRY(ph) == 558428690);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            assert(pack.target_component_TRY(ph) == (char)34);
            assert(pack.target_system_TRY(ph) == (char)12);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)12, PH) ;
        p77.progress_SET((char)133, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED) ;
        p77.target_component_SET((char)34, PH) ;
        p77.result_param2_SET(558428690, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.2903516E38F);
            assert(pack.yaw_GET() == 2.6537663E38F);
            assert(pack.manual_override_switch_GET() == (char)156);
            assert(pack.thrust_GET() == 1.2009329E38F);
            assert(pack.pitch_GET() == 6.014465E37F);
            assert(pack.mode_switch_GET() == (char)4);
            assert(pack.time_boot_ms_GET() == 2338633273L);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.roll_SET(2.2903516E38F) ;
        p81.time_boot_ms_SET(2338633273L) ;
        p81.pitch_SET(6.014465E37F) ;
        p81.manual_override_switch_SET((char)156) ;
        p81.yaw_SET(2.6537663E38F) ;
        p81.thrust_SET(1.2009329E38F) ;
        p81.mode_switch_SET((char)4) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)251);
            assert(pack.body_roll_rate_GET() == 5.3467155E37F);
            assert(pack.body_yaw_rate_GET() == -3.2739137E38F);
            assert(pack.body_pitch_rate_GET() == -3.376112E38F);
            assert(pack.target_component_GET() == (char)197);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3219392E38F, -1.2178361E38F, 9.671695E37F, -4.1887815E37F}));
            assert(pack.target_system_GET() == (char)239);
            assert(pack.time_boot_ms_GET() == 3961857720L);
            assert(pack.thrust_GET() == 8.736949E37F);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)251) ;
        p82.body_roll_rate_SET(5.3467155E37F) ;
        p82.time_boot_ms_SET(3961857720L) ;
        p82.target_system_SET((char)239) ;
        p82.body_pitch_rate_SET(-3.376112E38F) ;
        p82.target_component_SET((char)197) ;
        p82.thrust_SET(8.736949E37F) ;
        p82.q_SET(new float[] {2.3219392E38F, -1.2178361E38F, 9.671695E37F, -4.1887815E37F}, 0) ;
        p82.body_yaw_rate_SET(-3.2739137E38F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 696702338L);
            assert(pack.body_yaw_rate_GET() == -2.5048539E38F);
            assert(pack.body_pitch_rate_GET() == -2.480005E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.5321809E37F, 1.4104708E38F, -4.690402E37F, -2.8959314E38F}));
            assert(pack.body_roll_rate_GET() == -4.4626514E37F);
            assert(pack.type_mask_GET() == (char)58);
            assert(pack.thrust_GET() == -2.51211E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(-2.480005E37F) ;
        p83.q_SET(new float[] {1.5321809E37F, 1.4104708E38F, -4.690402E37F, -2.8959314E38F}, 0) ;
        p83.type_mask_SET((char)58) ;
        p83.time_boot_ms_SET(696702338L) ;
        p83.body_roll_rate_SET(-4.4626514E37F) ;
        p83.thrust_SET(-2.51211E38F) ;
        p83.body_yaw_rate_SET(-2.5048539E38F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.414165E38F);
            assert(pack.vx_GET() == 1.0762884E38F);
            assert(pack.type_mask_GET() == (char)23873);
            assert(pack.vy_GET() == -3.3114818E38F);
            assert(pack.vz_GET() == -1.7443302E38F);
            assert(pack.yaw_rate_GET() == 1.0902941E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.target_system_GET() == (char)129);
            assert(pack.afz_GET() == 3.34375E38F);
            assert(pack.target_component_GET() == (char)239);
            assert(pack.time_boot_ms_GET() == 3003599450L);
            assert(pack.afy_GET() == -9.076283E37F);
            assert(pack.x_GET() == -8.81071E37F);
            assert(pack.afx_GET() == -3.874696E36F);
            assert(pack.y_GET() == -2.5919017E38F);
            assert(pack.yaw_GET() == -1.2734109E38F);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(1.0902941E38F) ;
        p84.target_component_SET((char)239) ;
        p84.type_mask_SET((char)23873) ;
        p84.afy_SET(-9.076283E37F) ;
        p84.x_SET(-8.81071E37F) ;
        p84.afx_SET(-3.874696E36F) ;
        p84.vx_SET(1.0762884E38F) ;
        p84.y_SET(-2.5919017E38F) ;
        p84.afz_SET(3.34375E38F) ;
        p84.yaw_SET(-1.2734109E38F) ;
        p84.target_system_SET((char)129) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p84.z_SET(-2.414165E38F) ;
        p84.time_boot_ms_SET(3003599450L) ;
        p84.vy_SET(-3.3114818E38F) ;
        p84.vz_SET(-1.7443302E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)95);
            assert(pack.lat_int_GET() == 499535291);
            assert(pack.alt_GET() == 2.9805964E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.yaw_GET() == 1.7090636E38F);
            assert(pack.afx_GET() == -6.191711E37F);
            assert(pack.vz_GET() == -3.1350072E38F);
            assert(pack.afy_GET() == 2.8367915E38F);
            assert(pack.type_mask_GET() == (char)45397);
            assert(pack.lon_int_GET() == 1697123332);
            assert(pack.yaw_rate_GET() == 1.8376873E38F);
            assert(pack.target_system_GET() == (char)91);
            assert(pack.afz_GET() == -5.431677E37F);
            assert(pack.vy_GET() == 1.438504E37F);
            assert(pack.time_boot_ms_GET() == 2736874451L);
            assert(pack.vx_GET() == 1.5047879E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.target_system_SET((char)91) ;
        p86.time_boot_ms_SET(2736874451L) ;
        p86.vx_SET(1.5047879E38F) ;
        p86.vz_SET(-3.1350072E38F) ;
        p86.afx_SET(-6.191711E37F) ;
        p86.alt_SET(2.9805964E37F) ;
        p86.yaw_SET(1.7090636E38F) ;
        p86.type_mask_SET((char)45397) ;
        p86.yaw_rate_SET(1.8376873E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p86.lat_int_SET(499535291) ;
        p86.vy_SET(1.438504E37F) ;
        p86.target_component_SET((char)95) ;
        p86.afy_SET(2.8367915E38F) ;
        p86.afz_SET(-5.431677E37F) ;
        p86.lon_int_SET(1697123332) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == 1.0613466E37F);
            assert(pack.afy_GET() == 2.4399662E38F);
            assert(pack.afx_GET() == -2.3156157E38F);
            assert(pack.alt_GET() == -2.3574535E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.vz_GET() == 4.0542083E37F);
            assert(pack.yaw_rate_GET() == 8.3964136E37F);
            assert(pack.yaw_GET() == 2.5878184E38F);
            assert(pack.lon_int_GET() == 773753766);
            assert(pack.vy_GET() == 1.0814543E38F);
            assert(pack.vx_GET() == 3.2485368E38F);
            assert(pack.type_mask_GET() == (char)55204);
            assert(pack.time_boot_ms_GET() == 4250541202L);
            assert(pack.lat_int_GET() == -25499960);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.afy_SET(2.4399662E38F) ;
        p87.time_boot_ms_SET(4250541202L) ;
        p87.afz_SET(1.0613466E37F) ;
        p87.yaw_SET(2.5878184E38F) ;
        p87.afx_SET(-2.3156157E38F) ;
        p87.vy_SET(1.0814543E38F) ;
        p87.vx_SET(3.2485368E38F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p87.vz_SET(4.0542083E37F) ;
        p87.alt_SET(-2.3574535E38F) ;
        p87.type_mask_SET((char)55204) ;
        p87.lat_int_SET(-25499960) ;
        p87.lon_int_SET(773753766) ;
        p87.yaw_rate_SET(8.3964136E37F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -8.165697E37F);
            assert(pack.x_GET() == 3.1719512E38F);
            assert(pack.pitch_GET() == -8.370795E37F);
            assert(pack.z_GET() == -1.2848198E38F);
            assert(pack.roll_GET() == -1.6255334E38F);
            assert(pack.y_GET() == 2.6864676E38F);
            assert(pack.time_boot_ms_GET() == 3507000813L);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(-8.165697E37F) ;
        p89.roll_SET(-1.6255334E38F) ;
        p89.z_SET(-1.2848198E38F) ;
        p89.x_SET(3.1719512E38F) ;
        p89.time_boot_ms_SET(3507000813L) ;
        p89.y_SET(2.6864676E38F) ;
        p89.pitch_SET(-8.370795E37F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2525518011003224768L);
            assert(pack.pitchspeed_GET() == -3.0770764E38F);
            assert(pack.zacc_GET() == (short)10897);
            assert(pack.lat_GET() == 665216868);
            assert(pack.lon_GET() == 1839774511);
            assert(pack.xacc_GET() == (short)17777);
            assert(pack.alt_GET() == -268062847);
            assert(pack.vz_GET() == (short)14380);
            assert(pack.vy_GET() == (short) -18621);
            assert(pack.pitch_GET() == -2.5807778E38F);
            assert(pack.vx_GET() == (short) -18579);
            assert(pack.yacc_GET() == (short)13855);
            assert(pack.roll_GET() == 8.7947E37F);
            assert(pack.yawspeed_GET() == 3.0783532E38F);
            assert(pack.rollspeed_GET() == 5.759191E37F);
            assert(pack.yaw_GET() == -7.998454E37F);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.alt_SET(-268062847) ;
        p90.zacc_SET((short)10897) ;
        p90.roll_SET(8.7947E37F) ;
        p90.lat_SET(665216868) ;
        p90.time_usec_SET(2525518011003224768L) ;
        p90.vx_SET((short) -18579) ;
        p90.vz_SET((short)14380) ;
        p90.vy_SET((short) -18621) ;
        p90.rollspeed_SET(5.759191E37F) ;
        p90.xacc_SET((short)17777) ;
        p90.yaw_SET(-7.998454E37F) ;
        p90.lon_SET(1839774511) ;
        p90.pitchspeed_SET(-3.0770764E38F) ;
        p90.pitch_SET(-2.5807778E38F) ;
        p90.yacc_SET((short)13855) ;
        p90.yawspeed_SET(3.0783532E38F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux3_GET() == 1.7179024E38F);
            assert(pack.aux4_GET() == -1.9777266E38F);
            assert(pack.throttle_GET() == 2.5726632E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.nav_mode_GET() == (char)4);
            assert(pack.aux1_GET() == 2.1540842E38F);
            assert(pack.yaw_rudder_GET() == -3.3991854E38F);
            assert(pack.roll_ailerons_GET() == -2.965772E38F);
            assert(pack.pitch_elevator_GET() == -1.3331024E37F);
            assert(pack.aux2_GET() == -1.702808E38F);
            assert(pack.time_usec_GET() == 4924923766909283084L);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.yaw_rudder_SET(-3.3991854E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.throttle_SET(2.5726632E38F) ;
        p91.roll_ailerons_SET(-2.965772E38F) ;
        p91.nav_mode_SET((char)4) ;
        p91.aux2_SET(-1.702808E38F) ;
        p91.time_usec_SET(4924923766909283084L) ;
        p91.aux3_SET(1.7179024E38F) ;
        p91.aux1_SET(2.1540842E38F) ;
        p91.aux4_SET(-1.9777266E38F) ;
        p91.pitch_elevator_SET(-1.3331024E37F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan5_raw_GET() == (char)17239);
            assert(pack.chan3_raw_GET() == (char)12575);
            assert(pack.chan6_raw_GET() == (char)16361);
            assert(pack.chan8_raw_GET() == (char)42318);
            assert(pack.chan9_raw_GET() == (char)25594);
            assert(pack.chan4_raw_GET() == (char)63078);
            assert(pack.chan11_raw_GET() == (char)62892);
            assert(pack.rssi_GET() == (char)47);
            assert(pack.chan2_raw_GET() == (char)49734);
            assert(pack.chan12_raw_GET() == (char)55180);
            assert(pack.chan1_raw_GET() == (char)28510);
            assert(pack.chan10_raw_GET() == (char)20239);
            assert(pack.time_usec_GET() == 8714929788033307295L);
            assert(pack.chan7_raw_GET() == (char)40688);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan12_raw_SET((char)55180) ;
        p92.rssi_SET((char)47) ;
        p92.chan6_raw_SET((char)16361) ;
        p92.chan1_raw_SET((char)28510) ;
        p92.chan5_raw_SET((char)17239) ;
        p92.chan7_raw_SET((char)40688) ;
        p92.chan8_raw_SET((char)42318) ;
        p92.chan3_raw_SET((char)12575) ;
        p92.chan11_raw_SET((char)62892) ;
        p92.time_usec_SET(8714929788033307295L) ;
        p92.chan2_raw_SET((char)49734) ;
        p92.chan10_raw_SET((char)20239) ;
        p92.chan4_raw_SET((char)63078) ;
        p92.chan9_raw_SET((char)25594) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-8.568138E37F, -1.9978849E38F, 2.9823616E38F, 2.1969993E38F, 3.1830902E37F, -1.702647E38F, -9.164101E37F, -1.6348373E37F, 3.0032363E38F, 5.033207E37F, -2.5517847E38F, 2.9908167E38F, 1.0496434E38F, 2.96665E38F, -3.2153258E38F, -2.9439461E38F}));
            assert(pack.time_usec_GET() == 8535835985744459182L);
            assert(pack.flags_GET() == 5784030555684214940L);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(8535835985744459182L) ;
        p93.flags_SET(5784030555684214940L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p93.controls_SET(new float[] {-8.568138E37F, -1.9978849E38F, 2.9823616E38F, 2.1969993E38F, 3.1830902E37F, -1.702647E38F, -9.164101E37F, -1.6348373E37F, 3.0032363E38F, 5.033207E37F, -2.5517847E38F, 2.9908167E38F, 1.0496434E38F, 2.96665E38F, -3.2153258E38F, -2.9439461E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)51);
            assert(pack.sensor_id_GET() == (char)182);
            assert(pack.flow_comp_m_x_GET() == 2.690769E38F);
            assert(pack.flow_y_GET() == (short) -26915);
            assert(pack.flow_x_GET() == (short)670);
            assert(pack.ground_distance_GET() == -2.4006183E38F);
            assert(pack.flow_rate_y_TRY(ph) == 1.4892489E38F);
            assert(pack.time_usec_GET() == 9011591718817290715L);
            assert(pack.flow_comp_m_y_GET() == 1.6544969E38F);
            assert(pack.flow_rate_x_TRY(ph) == -3.144063E37F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.ground_distance_SET(-2.4006183E38F) ;
        p100.sensor_id_SET((char)182) ;
        p100.flow_rate_y_SET(1.4892489E38F, PH) ;
        p100.time_usec_SET(9011591718817290715L) ;
        p100.quality_SET((char)51) ;
        p100.flow_comp_m_x_SET(2.690769E38F) ;
        p100.flow_x_SET((short)670) ;
        p100.flow_y_SET((short) -26915) ;
        p100.flow_comp_m_y_SET(1.6544969E38F) ;
        p100.flow_rate_x_SET(-3.144063E37F, PH) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.2133585E38F);
            assert(pack.pitch_GET() == -8.394877E36F);
            assert(pack.z_GET() == -1.0929042E38F);
            assert(pack.x_GET() == -2.3938697E38F);
            assert(pack.y_GET() == 9.324489E37F);
            assert(pack.roll_GET() == -3.9506957E37F);
            assert(pack.usec_GET() == 2768164002773830535L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.pitch_SET(-8.394877E36F) ;
        p101.z_SET(-1.0929042E38F) ;
        p101.usec_SET(2768164002773830535L) ;
        p101.roll_SET(-3.9506957E37F) ;
        p101.x_SET(-2.3938697E38F) ;
        p101.y_SET(9.324489E37F) ;
        p101.yaw_SET(-1.2133585E38F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.2543228E38F);
            assert(pack.pitch_GET() == -1.2961796E37F);
            assert(pack.z_GET() == 1.5966757E38F);
            assert(pack.roll_GET() == -1.0817685E38F);
            assert(pack.x_GET() == 3.072308E38F);
            assert(pack.usec_GET() == 129432825756697397L);
            assert(pack.yaw_GET() == -1.6099455E35F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(-1.0817685E38F) ;
        p102.usec_SET(129432825756697397L) ;
        p102.y_SET(-3.2543228E38F) ;
        p102.z_SET(1.5966757E38F) ;
        p102.yaw_SET(-1.6099455E35F) ;
        p102.x_SET(3.072308E38F) ;
        p102.pitch_SET(-1.2961796E37F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 8495242309646016166L);
            assert(pack.z_GET() == -1.458898E38F);
            assert(pack.y_GET() == -2.9115401E38F);
            assert(pack.x_GET() == 2.3959002E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(-2.9115401E38F) ;
        p103.z_SET(-1.458898E38F) ;
        p103.usec_SET(8495242309646016166L) ;
        p103.x_SET(2.3959002E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -1.3784349E38F);
            assert(pack.usec_GET() == 3380780903752221294L);
            assert(pack.pitch_GET() == -2.8984348E38F);
            assert(pack.y_GET() == -3.308695E38F);
            assert(pack.roll_GET() == 1.2759331E38F);
            assert(pack.x_GET() == -7.6528133E37F);
            assert(pack.z_GET() == -2.7881109E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.y_SET(-3.308695E38F) ;
        p104.x_SET(-7.6528133E37F) ;
        p104.usec_SET(3380780903752221294L) ;
        p104.roll_SET(1.2759331E38F) ;
        p104.yaw_SET(-1.3784349E38F) ;
        p104.z_SET(-2.7881109E38F) ;
        p104.pitch_SET(-2.8984348E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == 3.3125594E38F);
            assert(pack.xgyro_GET() == 1.9431078E38F);
            assert(pack.zacc_GET() == 1.2744663E38F);
            assert(pack.xmag_GET() == 3.1262517E38F);
            assert(pack.zgyro_GET() == -2.6866662E38F);
            assert(pack.xacc_GET() == -1.5827155E38F);
            assert(pack.pressure_alt_GET() == -3.2254867E38F);
            assert(pack.ygyro_GET() == 1.2088813E38F);
            assert(pack.zmag_GET() == -1.2806177E38F);
            assert(pack.time_usec_GET() == 2861525771091449686L);
            assert(pack.fields_updated_GET() == (char)10819);
            assert(pack.diff_pressure_GET() == -3.0758966E38F);
            assert(pack.abs_pressure_GET() == -2.5922631E38F);
            assert(pack.yacc_GET() == 1.0979409E38F);
            assert(pack.ymag_GET() == 2.2251152E38F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.xacc_SET(-1.5827155E38F) ;
        p105.time_usec_SET(2861525771091449686L) ;
        p105.xmag_SET(3.1262517E38F) ;
        p105.xgyro_SET(1.9431078E38F) ;
        p105.fields_updated_SET((char)10819) ;
        p105.ymag_SET(2.2251152E38F) ;
        p105.temperature_SET(3.3125594E38F) ;
        p105.zacc_SET(1.2744663E38F) ;
        p105.abs_pressure_SET(-2.5922631E38F) ;
        p105.ygyro_SET(1.2088813E38F) ;
        p105.diff_pressure_SET(-3.0758966E38F) ;
        p105.zmag_SET(-1.2806177E38F) ;
        p105.zgyro_SET(-2.6866662E38F) ;
        p105.yacc_SET(1.0979409E38F) ;
        p105.pressure_alt_SET(-3.2254867E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)98);
            assert(pack.time_usec_GET() == 499390984922934440L);
            assert(pack.integrated_x_GET() == -1.4578489E38F);
            assert(pack.integrated_xgyro_GET() == -1.7508695E38F);
            assert(pack.sensor_id_GET() == (char)11);
            assert(pack.integrated_y_GET() == -2.3983128E38F);
            assert(pack.integrated_ygyro_GET() == 3.0356336E38F);
            assert(pack.time_delta_distance_us_GET() == 1957973871L);
            assert(pack.integration_time_us_GET() == 1552294208L);
            assert(pack.distance_GET() == -1.8251893E38F);
            assert(pack.temperature_GET() == (short) -18101);
            assert(pack.integrated_zgyro_GET() == 6.361663E37F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_zgyro_SET(6.361663E37F) ;
        p106.integrated_y_SET(-2.3983128E38F) ;
        p106.integrated_x_SET(-1.4578489E38F) ;
        p106.time_usec_SET(499390984922934440L) ;
        p106.quality_SET((char)98) ;
        p106.integration_time_us_SET(1552294208L) ;
        p106.temperature_SET((short) -18101) ;
        p106.integrated_xgyro_SET(-1.7508695E38F) ;
        p106.integrated_ygyro_SET(3.0356336E38F) ;
        p106.distance_SET(-1.8251893E38F) ;
        p106.time_delta_distance_us_SET(1957973871L) ;
        p106.sensor_id_SET((char)11) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == 1.0375293E38F);
            assert(pack.xmag_GET() == 3.049132E38F);
            assert(pack.xacc_GET() == -2.790575E35F);
            assert(pack.fields_updated_GET() == 2647848173L);
            assert(pack.ymag_GET() == -4.895655E37F);
            assert(pack.time_usec_GET() == 4859179671682678270L);
            assert(pack.zgyro_GET() == -2.5656741E38F);
            assert(pack.xgyro_GET() == -1.401904E38F);
            assert(pack.temperature_GET() == -1.5349098E38F);
            assert(pack.diff_pressure_GET() == -1.4122724E38F);
            assert(pack.ygyro_GET() == 2.0711707E38F);
            assert(pack.abs_pressure_GET() == 3.1812093E38F);
            assert(pack.yacc_GET() == 1.8806992E38F);
            assert(pack.zacc_GET() == 2.585136E38F);
            assert(pack.zmag_GET() == -6.4891465E37F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.fields_updated_SET(2647848173L) ;
        p107.zgyro_SET(-2.5656741E38F) ;
        p107.xmag_SET(3.049132E38F) ;
        p107.zacc_SET(2.585136E38F) ;
        p107.pressure_alt_SET(1.0375293E38F) ;
        p107.yacc_SET(1.8806992E38F) ;
        p107.diff_pressure_SET(-1.4122724E38F) ;
        p107.ymag_SET(-4.895655E37F) ;
        p107.temperature_SET(-1.5349098E38F) ;
        p107.xgyro_SET(-1.401904E38F) ;
        p107.ygyro_SET(2.0711707E38F) ;
        p107.xacc_SET(-2.790575E35F) ;
        p107.time_usec_SET(4859179671682678270L) ;
        p107.zmag_SET(-6.4891465E37F) ;
        p107.abs_pressure_SET(3.1812093E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_horz_GET() == -1.9960173E38F);
            assert(pack.zacc_GET() == -1.9356605E38F);
            assert(pack.roll_GET() == -2.0629103E38F);
            assert(pack.q2_GET() == 4.8818416E37F);
            assert(pack.q1_GET() == 6.930849E37F);
            assert(pack.zgyro_GET() == -1.2033906E38F);
            assert(pack.std_dev_vert_GET() == -3.3357678E38F);
            assert(pack.ygyro_GET() == -1.7280388E38F);
            assert(pack.q3_GET() == 1.785243E38F);
            assert(pack.xgyro_GET() == -2.6575216E38F);
            assert(pack.lat_GET() == -2.2131798E38F);
            assert(pack.vn_GET() == -3.016943E38F);
            assert(pack.yaw_GET() == 3.670094E37F);
            assert(pack.vd_GET() == -2.8841586E38F);
            assert(pack.q4_GET() == 2.5597581E38F);
            assert(pack.xacc_GET() == 9.30141E37F);
            assert(pack.pitch_GET() == -1.221177E38F);
            assert(pack.yacc_GET() == 1.9804688E38F);
            assert(pack.ve_GET() == 2.4941186E38F);
            assert(pack.lon_GET() == -6.1436757E37F);
            assert(pack.alt_GET() == -1.0154756E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q2_SET(4.8818416E37F) ;
        p108.vn_SET(-3.016943E38F) ;
        p108.lon_SET(-6.1436757E37F) ;
        p108.alt_SET(-1.0154756E38F) ;
        p108.zgyro_SET(-1.2033906E38F) ;
        p108.q1_SET(6.930849E37F) ;
        p108.zacc_SET(-1.9356605E38F) ;
        p108.yacc_SET(1.9804688E38F) ;
        p108.xacc_SET(9.30141E37F) ;
        p108.vd_SET(-2.8841586E38F) ;
        p108.pitch_SET(-1.221177E38F) ;
        p108.lat_SET(-2.2131798E38F) ;
        p108.roll_SET(-2.0629103E38F) ;
        p108.std_dev_horz_SET(-1.9960173E38F) ;
        p108.ve_SET(2.4941186E38F) ;
        p108.yaw_SET(3.670094E37F) ;
        p108.q4_SET(2.5597581E38F) ;
        p108.xgyro_SET(-2.6575216E38F) ;
        p108.q3_SET(1.785243E38F) ;
        p108.ygyro_SET(-1.7280388E38F) ;
        p108.std_dev_vert_SET(-3.3357678E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)14);
            assert(pack.txbuf_GET() == (char)121);
            assert(pack.remnoise_GET() == (char)101);
            assert(pack.fixed__GET() == (char)24942);
            assert(pack.remrssi_GET() == (char)36);
            assert(pack.noise_GET() == (char)168);
            assert(pack.rxerrors_GET() == (char)30162);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remnoise_SET((char)101) ;
        p109.remrssi_SET((char)36) ;
        p109.noise_SET((char)168) ;
        p109.rxerrors_SET((char)30162) ;
        p109.txbuf_SET((char)121) ;
        p109.rssi_SET((char)14) ;
        p109.fixed__SET((char)24942) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)177);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)118, (char)53, (char)176, (char)109, (char)230, (char)248, (char)70, (char)177, (char)154, (char)114, (char)101, (char)249, (char)249, (char)4, (char)94, (char)82, (char)128, (char)37, (char)98, (char)9, (char)130, (char)47, (char)163, (char)30, (char)164, (char)68, (char)217, (char)22, (char)159, (char)88, (char)22, (char)244, (char)200, (char)52, (char)49, (char)184, (char)137, (char)251, (char)234, (char)65, (char)12, (char)222, (char)234, (char)64, (char)218, (char)196, (char)215, (char)198, (char)18, (char)164, (char)135, (char)193, (char)37, (char)158, (char)47, (char)219, (char)113, (char)135, (char)248, (char)109, (char)238, (char)84, (char)55, (char)140, (char)121, (char)26, (char)128, (char)247, (char)153, (char)26, (char)76, (char)33, (char)248, (char)81, (char)5, (char)64, (char)216, (char)29, (char)106, (char)182, (char)96, (char)181, (char)248, (char)212, (char)49, (char)202, (char)240, (char)35, (char)67, (char)116, (char)26, (char)250, (char)112, (char)183, (char)248, (char)79, (char)244, (char)122, (char)20, (char)55, (char)58, (char)252, (char)174, (char)125, (char)127, (char)213, (char)39, (char)200, (char)119, (char)195, (char)243, (char)41, (char)224, (char)66, (char)200, (char)114, (char)182, (char)153, (char)41, (char)42, (char)16, (char)43, (char)99, (char)66, (char)205, (char)139, (char)132, (char)188, (char)150, (char)144, (char)146, (char)154, (char)21, (char)80, (char)9, (char)190, (char)95, (char)160, (char)82, (char)156, (char)138, (char)23, (char)68, (char)195, (char)189, (char)244, (char)134, (char)43, (char)148, (char)88, (char)206, (char)194, (char)189, (char)175, (char)64, (char)9, (char)150, (char)49, (char)177, (char)156, (char)191, (char)24, (char)180, (char)110, (char)153, (char)128, (char)207, (char)120, (char)160, (char)175, (char)208, (char)195, (char)150, (char)46, (char)159, (char)239, (char)171, (char)77, (char)4, (char)207, (char)173, (char)100, (char)212, (char)118, (char)35, (char)222, (char)205, (char)120, (char)139, (char)225, (char)71, (char)64, (char)68, (char)71, (char)46, (char)74, (char)224, (char)249, (char)48, (char)80, (char)132, (char)32, (char)87, (char)129, (char)94, (char)49, (char)58, (char)145, (char)199, (char)25, (char)70, (char)136, (char)9, (char)232, (char)167, (char)120, (char)52, (char)137, (char)207, (char)132, (char)46, (char)144, (char)196, (char)104, (char)140, (char)48, (char)149, (char)88, (char)69, (char)170, (char)48, (char)62, (char)250, (char)246, (char)196, (char)232, (char)12, (char)161, (char)63, (char)16, (char)251, (char)18, (char)204, (char)35, (char)77, (char)210, (char)188, (char)162, (char)232, (char)149, (char)87}));
            assert(pack.target_network_GET() == (char)67);
            assert(pack.target_system_GET() == (char)120);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)118, (char)53, (char)176, (char)109, (char)230, (char)248, (char)70, (char)177, (char)154, (char)114, (char)101, (char)249, (char)249, (char)4, (char)94, (char)82, (char)128, (char)37, (char)98, (char)9, (char)130, (char)47, (char)163, (char)30, (char)164, (char)68, (char)217, (char)22, (char)159, (char)88, (char)22, (char)244, (char)200, (char)52, (char)49, (char)184, (char)137, (char)251, (char)234, (char)65, (char)12, (char)222, (char)234, (char)64, (char)218, (char)196, (char)215, (char)198, (char)18, (char)164, (char)135, (char)193, (char)37, (char)158, (char)47, (char)219, (char)113, (char)135, (char)248, (char)109, (char)238, (char)84, (char)55, (char)140, (char)121, (char)26, (char)128, (char)247, (char)153, (char)26, (char)76, (char)33, (char)248, (char)81, (char)5, (char)64, (char)216, (char)29, (char)106, (char)182, (char)96, (char)181, (char)248, (char)212, (char)49, (char)202, (char)240, (char)35, (char)67, (char)116, (char)26, (char)250, (char)112, (char)183, (char)248, (char)79, (char)244, (char)122, (char)20, (char)55, (char)58, (char)252, (char)174, (char)125, (char)127, (char)213, (char)39, (char)200, (char)119, (char)195, (char)243, (char)41, (char)224, (char)66, (char)200, (char)114, (char)182, (char)153, (char)41, (char)42, (char)16, (char)43, (char)99, (char)66, (char)205, (char)139, (char)132, (char)188, (char)150, (char)144, (char)146, (char)154, (char)21, (char)80, (char)9, (char)190, (char)95, (char)160, (char)82, (char)156, (char)138, (char)23, (char)68, (char)195, (char)189, (char)244, (char)134, (char)43, (char)148, (char)88, (char)206, (char)194, (char)189, (char)175, (char)64, (char)9, (char)150, (char)49, (char)177, (char)156, (char)191, (char)24, (char)180, (char)110, (char)153, (char)128, (char)207, (char)120, (char)160, (char)175, (char)208, (char)195, (char)150, (char)46, (char)159, (char)239, (char)171, (char)77, (char)4, (char)207, (char)173, (char)100, (char)212, (char)118, (char)35, (char)222, (char)205, (char)120, (char)139, (char)225, (char)71, (char)64, (char)68, (char)71, (char)46, (char)74, (char)224, (char)249, (char)48, (char)80, (char)132, (char)32, (char)87, (char)129, (char)94, (char)49, (char)58, (char)145, (char)199, (char)25, (char)70, (char)136, (char)9, (char)232, (char)167, (char)120, (char)52, (char)137, (char)207, (char)132, (char)46, (char)144, (char)196, (char)104, (char)140, (char)48, (char)149, (char)88, (char)69, (char)170, (char)48, (char)62, (char)250, (char)246, (char)196, (char)232, (char)12, (char)161, (char)63, (char)16, (char)251, (char)18, (char)204, (char)35, (char)77, (char)210, (char)188, (char)162, (char)232, (char)149, (char)87}, 0) ;
        p110.target_component_SET((char)177) ;
        p110.target_network_SET((char)67) ;
        p110.target_system_SET((char)120) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -4029052677671701697L);
            assert(pack.ts1_GET() == 3175902512287092215L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-4029052677671701697L) ;
        p111.ts1_SET(3175902512287092215L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1455805770L);
            assert(pack.time_usec_GET() == 354785170868958108L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(354785170868958108L) ;
        p112.seq_SET(1455805770L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1576659649);
            assert(pack.vd_GET() == (short)15330);
            assert(pack.time_usec_GET() == 3999097778540516483L);
            assert(pack.vn_GET() == (short)14148);
            assert(pack.alt_GET() == 514645627);
            assert(pack.satellites_visible_GET() == (char)111);
            assert(pack.epv_GET() == (char)812);
            assert(pack.eph_GET() == (char)36783);
            assert(pack.lat_GET() == -2089614165);
            assert(pack.ve_GET() == (short) -21287);
            assert(pack.fix_type_GET() == (char)81);
            assert(pack.cog_GET() == (char)14213);
            assert(pack.vel_GET() == (char)1475);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.epv_SET((char)812) ;
        p113.cog_SET((char)14213) ;
        p113.fix_type_SET((char)81) ;
        p113.vd_SET((short)15330) ;
        p113.vel_SET((char)1475) ;
        p113.lon_SET(-1576659649) ;
        p113.ve_SET((short) -21287) ;
        p113.eph_SET((char)36783) ;
        p113.satellites_visible_SET((char)111) ;
        p113.time_usec_SET(3999097778540516483L) ;
        p113.lat_SET(-2089614165) ;
        p113.alt_SET(514645627) ;
        p113.vn_SET((short)14148) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == 9.374588E37F);
            assert(pack.integrated_xgyro_GET() == -1.1385529E38F);
            assert(pack.sensor_id_GET() == (char)68);
            assert(pack.quality_GET() == (char)207);
            assert(pack.integrated_y_GET() == -2.887629E38F);
            assert(pack.integrated_x_GET() == 7.45943E37F);
            assert(pack.integration_time_us_GET() == 865429075L);
            assert(pack.temperature_GET() == (short) -32128);
            assert(pack.integrated_zgyro_GET() == 9.606145E37F);
            assert(pack.time_usec_GET() == 392025968570950208L);
            assert(pack.time_delta_distance_us_GET() == 2007037339L);
            assert(pack.integrated_ygyro_GET() == 1.1160516E38F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_xgyro_SET(-1.1385529E38F) ;
        p114.integrated_zgyro_SET(9.606145E37F) ;
        p114.integration_time_us_SET(865429075L) ;
        p114.integrated_y_SET(-2.887629E38F) ;
        p114.temperature_SET((short) -32128) ;
        p114.sensor_id_SET((char)68) ;
        p114.integrated_x_SET(7.45943E37F) ;
        p114.integrated_ygyro_SET(1.1160516E38F) ;
        p114.quality_SET((char)207) ;
        p114.time_delta_distance_us_SET(2007037339L) ;
        p114.distance_SET(9.374588E37F) ;
        p114.time_usec_SET(392025968570950208L) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.true_airspeed_GET() == (char)8994);
            assert(pack.time_usec_GET() == 294755719980694998L);
            assert(pack.ind_airspeed_GET() == (char)1846);
            assert(pack.alt_GET() == 477567354);
            assert(pack.lon_GET() == 999334729);
            assert(pack.xacc_GET() == (short) -16260);
            assert(pack.yawspeed_GET() == -3.1676881E35F);
            assert(pack.vx_GET() == (short)6815);
            assert(pack.pitchspeed_GET() == -2.2698458E38F);
            assert(pack.yacc_GET() == (short)9248);
            assert(pack.rollspeed_GET() == 1.8793048E37F);
            assert(pack.vy_GET() == (short) -10420);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {3.3983936E38F, -2.3644974E38F, 2.3945457E38F, 2.2572137E38F}));
            assert(pack.lat_GET() == -92806227);
            assert(pack.zacc_GET() == (short)10447);
            assert(pack.vz_GET() == (short) -27841);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.ind_airspeed_SET((char)1846) ;
        p115.alt_SET(477567354) ;
        p115.true_airspeed_SET((char)8994) ;
        p115.lat_SET(-92806227) ;
        p115.attitude_quaternion_SET(new float[] {3.3983936E38F, -2.3644974E38F, 2.3945457E38F, 2.2572137E38F}, 0) ;
        p115.vy_SET((short) -10420) ;
        p115.xacc_SET((short) -16260) ;
        p115.pitchspeed_SET(-2.2698458E38F) ;
        p115.time_usec_SET(294755719980694998L) ;
        p115.vz_SET((short) -27841) ;
        p115.rollspeed_SET(1.8793048E37F) ;
        p115.vx_SET((short)6815) ;
        p115.zacc_SET((short)10447) ;
        p115.yacc_SET((short)9248) ;
        p115.yawspeed_SET(-3.1676881E35F) ;
        p115.lon_SET(999334729) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)24675);
            assert(pack.ygyro_GET() == (short) -12713);
            assert(pack.zacc_GET() == (short) -32090);
            assert(pack.zgyro_GET() == (short) -4809);
            assert(pack.zmag_GET() == (short) -16612);
            assert(pack.ymag_GET() == (short) -23413);
            assert(pack.xacc_GET() == (short)12004);
            assert(pack.xmag_GET() == (short) -18708);
            assert(pack.xgyro_GET() == (short)30923);
            assert(pack.time_boot_ms_GET() == 3350211198L);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xmag_SET((short) -18708) ;
        p116.ymag_SET((short) -23413) ;
        p116.xgyro_SET((short)30923) ;
        p116.yacc_SET((short)24675) ;
        p116.zgyro_SET((short) -4809) ;
        p116.ygyro_SET((short) -12713) ;
        p116.zacc_SET((short) -32090) ;
        p116.xacc_SET((short)12004) ;
        p116.time_boot_ms_SET(3350211198L) ;
        p116.zmag_SET((short) -16612) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)58);
            assert(pack.target_component_GET() == (char)116);
            assert(pack.start_GET() == (char)32872);
            assert(pack.end_GET() == (char)28054);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)116) ;
        p117.target_system_SET((char)58) ;
        p117.end_SET((char)28054) ;
        p117.start_SET((char)32872) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 2771371059L);
            assert(pack.num_logs_GET() == (char)17702);
            assert(pack.id_GET() == (char)19409);
            assert(pack.last_log_num_GET() == (char)49363);
            assert(pack.time_utc_GET() == 2578137272L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.time_utc_SET(2578137272L) ;
        p118.last_log_num_SET((char)49363) ;
        p118.id_SET((char)19409) ;
        p118.size_SET(2771371059L) ;
        p118.num_logs_SET((char)17702) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 3446914302L);
            assert(pack.ofs_GET() == 2569807374L);
            assert(pack.target_component_GET() == (char)182);
            assert(pack.id_GET() == (char)40953);
            assert(pack.target_system_GET() == (char)155);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.ofs_SET(2569807374L) ;
        p119.id_SET((char)40953) ;
        p119.count_SET(3446914302L) ;
        p119.target_system_SET((char)155) ;
        p119.target_component_SET((char)182) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)76);
            assert(pack.id_GET() == (char)47814);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)254, (char)158, (char)27, (char)185, (char)242, (char)71, (char)241, (char)6, (char)237, (char)191, (char)4, (char)83, (char)88, (char)219, (char)241, (char)29, (char)28, (char)57, (char)89, (char)130, (char)183, (char)214, (char)183, (char)180, (char)90, (char)0, (char)24, (char)226, (char)144, (char)221, (char)14, (char)79, (char)106, (char)10, (char)237, (char)219, (char)146, (char)204, (char)163, (char)108, (char)127, (char)178, (char)90, (char)168, (char)10, (char)12, (char)127, (char)106, (char)174, (char)145, (char)232, (char)176, (char)91, (char)187, (char)186, (char)59, (char)119, (char)63, (char)190, (char)96, (char)116, (char)168, (char)243, (char)254, (char)80, (char)189, (char)44, (char)79, (char)137, (char)139, (char)236, (char)78, (char)2, (char)190, (char)69, (char)148, (char)9, (char)135, (char)61, (char)90, (char)115, (char)37, (char)78, (char)187, (char)80, (char)63, (char)46, (char)175, (char)138, (char)110}));
            assert(pack.ofs_GET() == 3992225530L);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)254, (char)158, (char)27, (char)185, (char)242, (char)71, (char)241, (char)6, (char)237, (char)191, (char)4, (char)83, (char)88, (char)219, (char)241, (char)29, (char)28, (char)57, (char)89, (char)130, (char)183, (char)214, (char)183, (char)180, (char)90, (char)0, (char)24, (char)226, (char)144, (char)221, (char)14, (char)79, (char)106, (char)10, (char)237, (char)219, (char)146, (char)204, (char)163, (char)108, (char)127, (char)178, (char)90, (char)168, (char)10, (char)12, (char)127, (char)106, (char)174, (char)145, (char)232, (char)176, (char)91, (char)187, (char)186, (char)59, (char)119, (char)63, (char)190, (char)96, (char)116, (char)168, (char)243, (char)254, (char)80, (char)189, (char)44, (char)79, (char)137, (char)139, (char)236, (char)78, (char)2, (char)190, (char)69, (char)148, (char)9, (char)135, (char)61, (char)90, (char)115, (char)37, (char)78, (char)187, (char)80, (char)63, (char)46, (char)175, (char)138, (char)110}, 0) ;
        p120.id_SET((char)47814) ;
        p120.ofs_SET(3992225530L) ;
        p120.count_SET((char)76) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)186);
            assert(pack.target_component_GET() == (char)63);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)186) ;
        p121.target_component_SET((char)63) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)184);
            assert(pack.target_component_GET() == (char)144);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)184) ;
        p122.target_component_SET((char)144) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)111);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)232, (char)8, (char)174, (char)238, (char)117, (char)96, (char)137, (char)227, (char)198, (char)195, (char)5, (char)99, (char)50, (char)71, (char)241, (char)5, (char)252, (char)199, (char)140, (char)147, (char)85, (char)110, (char)218, (char)153, (char)141, (char)251, (char)2, (char)125, (char)98, (char)161, (char)107, (char)92, (char)19, (char)253, (char)54, (char)197, (char)165, (char)94, (char)66, (char)199, (char)170, (char)235, (char)188, (char)175, (char)110, (char)93, (char)127, (char)236, (char)4, (char)100, (char)158, (char)210, (char)170, (char)64, (char)195, (char)42, (char)128, (char)75, (char)204, (char)74, (char)35, (char)66, (char)248, (char)245, (char)164, (char)220, (char)29, (char)199, (char)250, (char)105, (char)162, (char)243, (char)50, (char)231, (char)183, (char)35, (char)133, (char)87, (char)19, (char)47, (char)15, (char)176, (char)54, (char)203, (char)127, (char)156, (char)255, (char)168, (char)49, (char)118, (char)73, (char)59, (char)34, (char)83, (char)53, (char)187, (char)47, (char)54, (char)172, (char)97, (char)119, (char)84, (char)97, (char)153, (char)220, (char)193, (char)227, (char)172, (char)14, (char)62}));
            assert(pack.len_GET() == (char)122);
            assert(pack.target_component_GET() == (char)199);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)111) ;
        p123.data__SET(new char[] {(char)232, (char)8, (char)174, (char)238, (char)117, (char)96, (char)137, (char)227, (char)198, (char)195, (char)5, (char)99, (char)50, (char)71, (char)241, (char)5, (char)252, (char)199, (char)140, (char)147, (char)85, (char)110, (char)218, (char)153, (char)141, (char)251, (char)2, (char)125, (char)98, (char)161, (char)107, (char)92, (char)19, (char)253, (char)54, (char)197, (char)165, (char)94, (char)66, (char)199, (char)170, (char)235, (char)188, (char)175, (char)110, (char)93, (char)127, (char)236, (char)4, (char)100, (char)158, (char)210, (char)170, (char)64, (char)195, (char)42, (char)128, (char)75, (char)204, (char)74, (char)35, (char)66, (char)248, (char)245, (char)164, (char)220, (char)29, (char)199, (char)250, (char)105, (char)162, (char)243, (char)50, (char)231, (char)183, (char)35, (char)133, (char)87, (char)19, (char)47, (char)15, (char)176, (char)54, (char)203, (char)127, (char)156, (char)255, (char)168, (char)49, (char)118, (char)73, (char)59, (char)34, (char)83, (char)53, (char)187, (char)47, (char)54, (char)172, (char)97, (char)119, (char)84, (char)97, (char)153, (char)220, (char)193, (char)227, (char)172, (char)14, (char)62}, 0) ;
        p123.target_component_SET((char)199) ;
        p123.len_SET((char)122) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.dgps_numch_GET() == (char)99);
            assert(pack.time_usec_GET() == 3980366128378902521L);
            assert(pack.lat_GET() == -628013408);
            assert(pack.lon_GET() == -294220235);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
            assert(pack.vel_GET() == (char)54813);
            assert(pack.eph_GET() == (char)52859);
            assert(pack.alt_GET() == 1999018769);
            assert(pack.epv_GET() == (char)28996);
            assert(pack.dgps_age_GET() == 3336756821L);
            assert(pack.cog_GET() == (char)42456);
            assert(pack.satellites_visible_GET() == (char)37);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lat_SET(-628013408) ;
        p124.lon_SET(-294220235) ;
        p124.dgps_numch_SET((char)99) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP) ;
        p124.satellites_visible_SET((char)37) ;
        p124.vel_SET((char)54813) ;
        p124.eph_SET((char)52859) ;
        p124.dgps_age_SET(3336756821L) ;
        p124.time_usec_SET(3980366128378902521L) ;
        p124.epv_SET((char)28996) ;
        p124.cog_SET((char)42456) ;
        p124.alt_SET(1999018769) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            assert(pack.Vcc_GET() == (char)59221);
            assert(pack.Vservo_GET() == (char)42100);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)59221) ;
        p125.Vservo_SET((char)42100) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)42);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            assert(pack.timeout_GET() == (char)64302);
            assert(pack.baudrate_GET() == 2609574837L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)250, (char)112, (char)223, (char)78, (char)65, (char)237, (char)120, (char)54, (char)253, (char)71, (char)70, (char)89, (char)39, (char)106, (char)63, (char)53, (char)192, (char)149, (char)147, (char)190, (char)12, (char)22, (char)188, (char)242, (char)44, (char)181, (char)217, (char)2, (char)26, (char)36, (char)11, (char)161, (char)131, (char)232, (char)44, (char)24, (char)102, (char)193, (char)30, (char)165, (char)141, (char)37, (char)42, (char)75, (char)81, (char)206, (char)72, (char)150, (char)15, (char)147, (char)127, (char)7, (char)38, (char)57, (char)167, (char)49, (char)163, (char)167, (char)138, (char)118, (char)217, (char)6, (char)106, (char)119, (char)176, (char)205, (char)73, (char)107, (char)155, (char)10}));
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.timeout_SET((char)64302) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1) ;
        p126.count_SET((char)42) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING) ;
        p126.data__SET(new char[] {(char)250, (char)112, (char)223, (char)78, (char)65, (char)237, (char)120, (char)54, (char)253, (char)71, (char)70, (char)89, (char)39, (char)106, (char)63, (char)53, (char)192, (char)149, (char)147, (char)190, (char)12, (char)22, (char)188, (char)242, (char)44, (char)181, (char)217, (char)2, (char)26, (char)36, (char)11, (char)161, (char)131, (char)232, (char)44, (char)24, (char)102, (char)193, (char)30, (char)165, (char)141, (char)37, (char)42, (char)75, (char)81, (char)206, (char)72, (char)150, (char)15, (char)147, (char)127, (char)7, (char)38, (char)57, (char)167, (char)49, (char)163, (char)167, (char)138, (char)118, (char)217, (char)6, (char)106, (char)119, (char)176, (char)205, (char)73, (char)107, (char)155, (char)10}, 0) ;
        p126.baudrate_SET(2609574837L) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.wn_GET() == (char)62319);
            assert(pack.accuracy_GET() == 1596322540L);
            assert(pack.rtk_health_GET() == (char)163);
            assert(pack.time_last_baseline_ms_GET() == 3484863607L);
            assert(pack.tow_GET() == 2384929217L);
            assert(pack.baseline_b_mm_GET() == -1856994496);
            assert(pack.baseline_c_mm_GET() == 79673410);
            assert(pack.rtk_rate_GET() == (char)162);
            assert(pack.baseline_a_mm_GET() == -73367156);
            assert(pack.rtk_receiver_id_GET() == (char)131);
            assert(pack.baseline_coords_type_GET() == (char)14);
            assert(pack.iar_num_hypotheses_GET() == -712960672);
            assert(pack.nsats_GET() == (char)177);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(3484863607L) ;
        p127.rtk_rate_SET((char)162) ;
        p127.baseline_coords_type_SET((char)14) ;
        p127.nsats_SET((char)177) ;
        p127.rtk_health_SET((char)163) ;
        p127.wn_SET((char)62319) ;
        p127.baseline_a_mm_SET(-73367156) ;
        p127.iar_num_hypotheses_SET(-712960672) ;
        p127.rtk_receiver_id_SET((char)131) ;
        p127.baseline_b_mm_SET(-1856994496) ;
        p127.tow_SET(2384929217L) ;
        p127.baseline_c_mm_SET(79673410) ;
        p127.accuracy_SET(1596322540L) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.time_last_baseline_ms_GET() == 4108556458L);
            assert(pack.rtk_receiver_id_GET() == (char)92);
            assert(pack.rtk_rate_GET() == (char)117);
            assert(pack.baseline_c_mm_GET() == 597295712);
            assert(pack.accuracy_GET() == 3196315993L);
            assert(pack.baseline_coords_type_GET() == (char)152);
            assert(pack.baseline_b_mm_GET() == 2129121246);
            assert(pack.tow_GET() == 3995839928L);
            assert(pack.baseline_a_mm_GET() == 553420506);
            assert(pack.rtk_health_GET() == (char)102);
            assert(pack.wn_GET() == (char)56502);
            assert(pack.iar_num_hypotheses_GET() == -922440945);
            assert(pack.nsats_GET() == (char)247);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(4108556458L) ;
        p128.rtk_health_SET((char)102) ;
        p128.rtk_rate_SET((char)117) ;
        p128.iar_num_hypotheses_SET(-922440945) ;
        p128.accuracy_SET(3196315993L) ;
        p128.wn_SET((char)56502) ;
        p128.baseline_b_mm_SET(2129121246) ;
        p128.tow_SET(3995839928L) ;
        p128.rtk_receiver_id_SET((char)92) ;
        p128.baseline_a_mm_SET(553420506) ;
        p128.baseline_c_mm_SET(597295712) ;
        p128.nsats_SET((char)247) ;
        p128.baseline_coords_type_SET((char)152) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short) -23123);
            assert(pack.zmag_GET() == (short)23796);
            assert(pack.xgyro_GET() == (short)23498);
            assert(pack.zacc_GET() == (short) -9516);
            assert(pack.zgyro_GET() == (short) -20744);
            assert(pack.xmag_GET() == (short) -20852);
            assert(pack.yacc_GET() == (short) -6228);
            assert(pack.ygyro_GET() == (short)16747);
            assert(pack.time_boot_ms_GET() == 3595676765L);
            assert(pack.ymag_GET() == (short) -31352);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zmag_SET((short)23796) ;
        p129.zgyro_SET((short) -20744) ;
        p129.xmag_SET((short) -20852) ;
        p129.ygyro_SET((short)16747) ;
        p129.xgyro_SET((short)23498) ;
        p129.zacc_SET((short) -9516) ;
        p129.xacc_SET((short) -23123) ;
        p129.time_boot_ms_SET(3595676765L) ;
        p129.yacc_SET((short) -6228) ;
        p129.ymag_SET((short) -31352) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.width_GET() == (char)18109);
            assert(pack.payload_GET() == (char)130);
            assert(pack.packets_GET() == (char)46992);
            assert(pack.type_GET() == (char)163);
            assert(pack.size_GET() == 3579354573L);
            assert(pack.height_GET() == (char)21210);
            assert(pack.jpg_quality_GET() == (char)69);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.jpg_quality_SET((char)69) ;
        p130.payload_SET((char)130) ;
        p130.width_SET((char)18109) ;
        p130.height_SET((char)21210) ;
        p130.type_SET((char)163) ;
        p130.size_SET(3579354573L) ;
        p130.packets_SET((char)46992) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)42996);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)129, (char)41, (char)234, (char)216, (char)139, (char)212, (char)190, (char)214, (char)78, (char)172, (char)192, (char)135, (char)103, (char)24, (char)81, (char)117, (char)252, (char)252, (char)231, (char)232, (char)250, (char)167, (char)31, (char)199, (char)59, (char)170, (char)224, (char)236, (char)191, (char)118, (char)169, (char)220, (char)41, (char)125, (char)240, (char)121, (char)211, (char)27, (char)37, (char)188, (char)73, (char)226, (char)67, (char)35, (char)35, (char)123, (char)28, (char)75, (char)22, (char)208, (char)244, (char)213, (char)230, (char)4, (char)250, (char)217, (char)231, (char)111, (char)71, (char)82, (char)15, (char)159, (char)145, (char)174, (char)41, (char)115, (char)10, (char)211, (char)232, (char)130, (char)135, (char)39, (char)34, (char)56, (char)103, (char)154, (char)69, (char)240, (char)247, (char)50, (char)73, (char)133, (char)90, (char)5, (char)66, (char)0, (char)249, (char)198, (char)135, (char)246, (char)241, (char)148, (char)120, (char)116, (char)0, (char)154, (char)68, (char)189, (char)36, (char)105, (char)244, (char)129, (char)102, (char)130, (char)133, (char)118, (char)50, (char)63, (char)94, (char)216, (char)15, (char)33, (char)114, (char)180, (char)27, (char)114, (char)161, (char)67, (char)51, (char)5, (char)86, (char)176, (char)112, (char)13, (char)242, (char)37, (char)227, (char)92, (char)32, (char)125, (char)181, (char)133, (char)67, (char)108, (char)182, (char)213, (char)151, (char)38, (char)248, (char)5, (char)251, (char)130, (char)70, (char)140, (char)115, (char)46, (char)204, (char)14, (char)192, (char)192, (char)38, (char)32, (char)168, (char)118, (char)142, (char)129, (char)64, (char)220, (char)118, (char)78, (char)223, (char)172, (char)107, (char)226, (char)138, (char)3, (char)222, (char)115, (char)96, (char)154, (char)161, (char)236, (char)48, (char)19, (char)88, (char)193, (char)115, (char)127, (char)241, (char)23, (char)214, (char)122, (char)62, (char)9, (char)225, (char)88, (char)91, (char)6, (char)114, (char)186, (char)249, (char)44, (char)140, (char)67, (char)68, (char)151, (char)148, (char)134, (char)201, (char)98, (char)17, (char)157, (char)202, (char)117, (char)133, (char)186, (char)41, (char)86, (char)70, (char)14, (char)106, (char)163, (char)181, (char)153, (char)221, (char)25, (char)108, (char)96, (char)5, (char)73, (char)81, (char)36, (char)44, (char)24, (char)16, (char)6, (char)22, (char)143, (char)191, (char)151, (char)113, (char)108, (char)35, (char)157, (char)72, (char)249, (char)74, (char)223, (char)73, (char)123, (char)67, (char)52, (char)62, (char)165, (char)67, (char)56, (char)50, (char)237, (char)157, (char)27, (char)248, (char)175, (char)149}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)42996) ;
        p131.data__SET(new char[] {(char)129, (char)41, (char)234, (char)216, (char)139, (char)212, (char)190, (char)214, (char)78, (char)172, (char)192, (char)135, (char)103, (char)24, (char)81, (char)117, (char)252, (char)252, (char)231, (char)232, (char)250, (char)167, (char)31, (char)199, (char)59, (char)170, (char)224, (char)236, (char)191, (char)118, (char)169, (char)220, (char)41, (char)125, (char)240, (char)121, (char)211, (char)27, (char)37, (char)188, (char)73, (char)226, (char)67, (char)35, (char)35, (char)123, (char)28, (char)75, (char)22, (char)208, (char)244, (char)213, (char)230, (char)4, (char)250, (char)217, (char)231, (char)111, (char)71, (char)82, (char)15, (char)159, (char)145, (char)174, (char)41, (char)115, (char)10, (char)211, (char)232, (char)130, (char)135, (char)39, (char)34, (char)56, (char)103, (char)154, (char)69, (char)240, (char)247, (char)50, (char)73, (char)133, (char)90, (char)5, (char)66, (char)0, (char)249, (char)198, (char)135, (char)246, (char)241, (char)148, (char)120, (char)116, (char)0, (char)154, (char)68, (char)189, (char)36, (char)105, (char)244, (char)129, (char)102, (char)130, (char)133, (char)118, (char)50, (char)63, (char)94, (char)216, (char)15, (char)33, (char)114, (char)180, (char)27, (char)114, (char)161, (char)67, (char)51, (char)5, (char)86, (char)176, (char)112, (char)13, (char)242, (char)37, (char)227, (char)92, (char)32, (char)125, (char)181, (char)133, (char)67, (char)108, (char)182, (char)213, (char)151, (char)38, (char)248, (char)5, (char)251, (char)130, (char)70, (char)140, (char)115, (char)46, (char)204, (char)14, (char)192, (char)192, (char)38, (char)32, (char)168, (char)118, (char)142, (char)129, (char)64, (char)220, (char)118, (char)78, (char)223, (char)172, (char)107, (char)226, (char)138, (char)3, (char)222, (char)115, (char)96, (char)154, (char)161, (char)236, (char)48, (char)19, (char)88, (char)193, (char)115, (char)127, (char)241, (char)23, (char)214, (char)122, (char)62, (char)9, (char)225, (char)88, (char)91, (char)6, (char)114, (char)186, (char)249, (char)44, (char)140, (char)67, (char)68, (char)151, (char)148, (char)134, (char)201, (char)98, (char)17, (char)157, (char)202, (char)117, (char)133, (char)186, (char)41, (char)86, (char)70, (char)14, (char)106, (char)163, (char)181, (char)153, (char)221, (char)25, (char)108, (char)96, (char)5, (char)73, (char)81, (char)36, (char)44, (char)24, (char)16, (char)6, (char)22, (char)143, (char)191, (char)151, (char)113, (char)108, (char)35, (char)157, (char)72, (char)249, (char)74, (char)223, (char)73, (char)123, (char)67, (char)52, (char)62, (char)165, (char)67, (char)56, (char)50, (char)237, (char)157, (char)27, (char)248, (char)175, (char)149}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.id_GET() == (char)250);
            assert(pack.covariance_GET() == (char)74);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135);
            assert(pack.current_distance_GET() == (char)15778);
            assert(pack.max_distance_GET() == (char)41229);
            assert(pack.time_boot_ms_GET() == 119141776L);
            assert(pack.min_distance_GET() == (char)62406);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.id_SET((char)250) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p132.min_distance_SET((char)62406) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135) ;
        p132.current_distance_SET((char)15778) ;
        p132.time_boot_ms_SET(119141776L) ;
        p132.max_distance_SET((char)41229) ;
        p132.covariance_SET((char)74) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 7913273547578811438L);
            assert(pack.lat_GET() == 838676192);
            assert(pack.grid_spacing_GET() == (char)7377);
            assert(pack.lon_GET() == -69301389);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(838676192) ;
        p133.lon_SET(-69301389) ;
        p133.mask_SET(7913273547578811438L) ;
        p133.grid_spacing_SET((char)7377) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -504229123);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)7224, (short)3584, (short)15990, (short)14866, (short)20640, (short) -10426, (short)10273, (short)31786, (short) -17787, (short) -14031, (short) -5149, (short) -21683, (short)29760, (short)20296, (short) -16563, (short) -16783}));
            assert(pack.lat_GET() == 51617028);
            assert(pack.grid_spacing_GET() == (char)17220);
            assert(pack.gridbit_GET() == (char)84);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)84) ;
        p134.lon_SET(-504229123) ;
        p134.grid_spacing_SET((char)17220) ;
        p134.lat_SET(51617028) ;
        p134.data__SET(new short[] {(short)7224, (short)3584, (short)15990, (short)14866, (short)20640, (short) -10426, (short)10273, (short)31786, (short) -17787, (short) -14031, (short) -5149, (short) -21683, (short)29760, (short)20296, (short) -16563, (short) -16783}, 0) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1375989884);
            assert(pack.lat_GET() == 1010236308);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(-1375989884) ;
        p135.lat_SET(1010236308) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.terrain_height_GET() == -2.3928765E38F);
            assert(pack.lat_GET() == -1677634451);
            assert(pack.pending_GET() == (char)58980);
            assert(pack.current_height_GET() == -3.4819206E37F);
            assert(pack.spacing_GET() == (char)43440);
            assert(pack.lon_GET() == -422494970);
            assert(pack.loaded_GET() == (char)4636);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.loaded_SET((char)4636) ;
        p136.lon_SET(-422494970) ;
        p136.pending_SET((char)58980) ;
        p136.terrain_height_SET(-2.3928765E38F) ;
        p136.lat_SET(-1677634451) ;
        p136.current_height_SET(-3.4819206E37F) ;
        p136.spacing_SET((char)43440) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -2.6334814E38F);
            assert(pack.time_boot_ms_GET() == 952417529L);
            assert(pack.temperature_GET() == (short) -29777);
            assert(pack.press_diff_GET() == 2.1725255E37F);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(952417529L) ;
        p137.temperature_SET((short) -29777) ;
        p137.press_abs_SET(-2.6334814E38F) ;
        p137.press_diff_SET(2.1725255E37F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -2.0700845E37F);
            assert(pack.time_usec_GET() == 6717579313811407196L);
            assert(pack.z_GET() == -3.1673175E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.6354164E38F, 2.3703807E38F, -9.323255E37F, 1.2373419E38F}));
            assert(pack.y_GET() == 1.2256573E37F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.y_SET(1.2256573E37F) ;
        p138.z_SET(-3.1673175E38F) ;
        p138.time_usec_SET(6717579313811407196L) ;
        p138.x_SET(-2.0700845E37F) ;
        p138.q_SET(new float[] {-2.6354164E38F, 2.3703807E38F, -9.323255E37F, 1.2373419E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)117);
            assert(pack.time_usec_GET() == 5203271515515231008L);
            assert(pack.target_system_GET() == (char)33);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.7074688E38F, -2.094739E38F, -2.059535E38F, 1.9413339E38F, -2.2645992E38F, -6.999454E37F, -1.4878229E37F, -3.273228E38F}));
            assert(pack.group_mlx_GET() == (char)206);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.controls_SET(new float[] {-1.7074688E38F, -2.094739E38F, -2.059535E38F, 1.9413339E38F, -2.2645992E38F, -6.999454E37F, -1.4878229E37F, -3.273228E38F}, 0) ;
        p139.time_usec_SET(5203271515515231008L) ;
        p139.target_system_SET((char)33) ;
        p139.target_component_SET((char)117) ;
        p139.group_mlx_SET((char)206) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.5592665E38F, 2.1420265E38F, -1.0357757E38F, -1.3419552E38F, -2.4547815E38F, -2.7698212E38F, -2.4914754E38F, 1.2912334E38F}));
            assert(pack.time_usec_GET() == 4214696180164133375L);
            assert(pack.group_mlx_GET() == (char)143);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(4214696180164133375L) ;
        p140.controls_SET(new float[] {1.5592665E38F, 2.1420265E38F, -1.0357757E38F, -1.3419552E38F, -2.4547815E38F, -2.7698212E38F, -2.4914754E38F, 1.2912334E38F}, 0) ;
        p140.group_mlx_SET((char)143) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8343846070353456327L);
            assert(pack.altitude_relative_GET() == 7.256466E37F);
            assert(pack.altitude_terrain_GET() == -4.7521334E36F);
            assert(pack.altitude_monotonic_GET() == 2.2835797E38F);
            assert(pack.altitude_local_GET() == -2.5591251E38F);
            assert(pack.altitude_amsl_GET() == -2.759516E38F);
            assert(pack.bottom_clearance_GET() == 4.6883413E37F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_amsl_SET(-2.759516E38F) ;
        p141.altitude_relative_SET(7.256466E37F) ;
        p141.bottom_clearance_SET(4.6883413E37F) ;
        p141.time_usec_SET(8343846070353456327L) ;
        p141.altitude_monotonic_SET(2.2835797E38F) ;
        p141.altitude_local_SET(-2.5591251E38F) ;
        p141.altitude_terrain_SET(-4.7521334E36F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)67, (char)20, (char)240, (char)16, (char)245, (char)129, (char)160, (char)114, (char)53, (char)37, (char)234, (char)169, (char)177, (char)206, (char)67, (char)219, (char)135, (char)164, (char)129, (char)17, (char)131, (char)39, (char)72, (char)176, (char)106, (char)26, (char)41, (char)154, (char)102, (char)7, (char)251, (char)193, (char)63, (char)177, (char)55, (char)59, (char)50, (char)99, (char)201, (char)90, (char)165, (char)1, (char)199, (char)106, (char)87, (char)192, (char)140, (char)241, (char)105, (char)139, (char)27, (char)105, (char)251, (char)224, (char)159, (char)202, (char)133, (char)199, (char)143, (char)41, (char)113, (char)179, (char)150, (char)126, (char)45, (char)251, (char)249, (char)232, (char)66, (char)140, (char)119, (char)245, (char)21, (char)122, (char)248, (char)254, (char)129, (char)164, (char)146, (char)129, (char)89, (char)7, (char)129, (char)124, (char)242, (char)217, (char)11, (char)91, (char)58, (char)232, (char)254, (char)56, (char)156, (char)53, (char)73, (char)163, (char)199, (char)171, (char)222, (char)255, (char)161, (char)196, (char)58, (char)205, (char)191, (char)219, (char)104, (char)161, (char)107, (char)99, (char)104, (char)76, (char)46, (char)45, (char)32, (char)15, (char)72, (char)31, (char)16, (char)14}));
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)156, (char)254, (char)230, (char)173, (char)227, (char)66, (char)79, (char)234, (char)157, (char)244, (char)12, (char)164, (char)161, (char)60, (char)108, (char)145, (char)72, (char)146, (char)51, (char)94, (char)60, (char)11, (char)29, (char)0, (char)171, (char)76, (char)106, (char)152, (char)198, (char)144, (char)144, (char)200, (char)212, (char)82, (char)207, (char)237, (char)149, (char)152, (char)40, (char)190, (char)100, (char)184, (char)224, (char)4, (char)222, (char)111, (char)168, (char)198, (char)138, (char)158, (char)59, (char)100, (char)19, (char)62, (char)123, (char)170, (char)106, (char)73, (char)174, (char)164, (char)183, (char)61, (char)115, (char)39, (char)183, (char)92, (char)180, (char)215, (char)209, (char)186, (char)100, (char)252, (char)7, (char)63, (char)172, (char)136, (char)95, (char)204, (char)138, (char)205, (char)9, (char)189, (char)75, (char)78, (char)33, (char)227, (char)238, (char)118, (char)125, (char)239, (char)156, (char)27, (char)39, (char)91, (char)16, (char)233, (char)94, (char)127, (char)100, (char)222, (char)65, (char)138, (char)54, (char)65, (char)218, (char)26, (char)14, (char)152, (char)125, (char)191, (char)68, (char)218, (char)223, (char)71, (char)131, (char)72, (char)215, (char)4, (char)68, (char)241}));
            assert(pack.transfer_type_GET() == (char)137);
            assert(pack.uri_type_GET() == (char)118);
            assert(pack.request_id_GET() == (char)152);
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)67, (char)20, (char)240, (char)16, (char)245, (char)129, (char)160, (char)114, (char)53, (char)37, (char)234, (char)169, (char)177, (char)206, (char)67, (char)219, (char)135, (char)164, (char)129, (char)17, (char)131, (char)39, (char)72, (char)176, (char)106, (char)26, (char)41, (char)154, (char)102, (char)7, (char)251, (char)193, (char)63, (char)177, (char)55, (char)59, (char)50, (char)99, (char)201, (char)90, (char)165, (char)1, (char)199, (char)106, (char)87, (char)192, (char)140, (char)241, (char)105, (char)139, (char)27, (char)105, (char)251, (char)224, (char)159, (char)202, (char)133, (char)199, (char)143, (char)41, (char)113, (char)179, (char)150, (char)126, (char)45, (char)251, (char)249, (char)232, (char)66, (char)140, (char)119, (char)245, (char)21, (char)122, (char)248, (char)254, (char)129, (char)164, (char)146, (char)129, (char)89, (char)7, (char)129, (char)124, (char)242, (char)217, (char)11, (char)91, (char)58, (char)232, (char)254, (char)56, (char)156, (char)53, (char)73, (char)163, (char)199, (char)171, (char)222, (char)255, (char)161, (char)196, (char)58, (char)205, (char)191, (char)219, (char)104, (char)161, (char)107, (char)99, (char)104, (char)76, (char)46, (char)45, (char)32, (char)15, (char)72, (char)31, (char)16, (char)14}, 0) ;
        p142.uri_SET(new char[] {(char)156, (char)254, (char)230, (char)173, (char)227, (char)66, (char)79, (char)234, (char)157, (char)244, (char)12, (char)164, (char)161, (char)60, (char)108, (char)145, (char)72, (char)146, (char)51, (char)94, (char)60, (char)11, (char)29, (char)0, (char)171, (char)76, (char)106, (char)152, (char)198, (char)144, (char)144, (char)200, (char)212, (char)82, (char)207, (char)237, (char)149, (char)152, (char)40, (char)190, (char)100, (char)184, (char)224, (char)4, (char)222, (char)111, (char)168, (char)198, (char)138, (char)158, (char)59, (char)100, (char)19, (char)62, (char)123, (char)170, (char)106, (char)73, (char)174, (char)164, (char)183, (char)61, (char)115, (char)39, (char)183, (char)92, (char)180, (char)215, (char)209, (char)186, (char)100, (char)252, (char)7, (char)63, (char)172, (char)136, (char)95, (char)204, (char)138, (char)205, (char)9, (char)189, (char)75, (char)78, (char)33, (char)227, (char)238, (char)118, (char)125, (char)239, (char)156, (char)27, (char)39, (char)91, (char)16, (char)233, (char)94, (char)127, (char)100, (char)222, (char)65, (char)138, (char)54, (char)65, (char)218, (char)26, (char)14, (char)152, (char)125, (char)191, (char)68, (char)218, (char)223, (char)71, (char)131, (char)72, (char)215, (char)4, (char)68, (char)241}, 0) ;
        p142.transfer_type_SET((char)137) ;
        p142.uri_type_SET((char)118) ;
        p142.request_id_SET((char)152) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == -5.504491E37F);
            assert(pack.time_boot_ms_GET() == 143113602L);
            assert(pack.temperature_GET() == (short) -1091);
            assert(pack.press_diff_GET() == -1.5896415E38F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.press_abs_SET(-5.504491E37F) ;
        p143.time_boot_ms_SET(143113602L) ;
        p143.temperature_SET((short) -1091) ;
        p143.press_diff_SET(-1.5896415E38F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.est_capabilities_GET() == (char)243);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-8.789467E37F, -2.288216E38F, 2.9884613E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.7580363E38F, -2.5999915E38F, 7.9231823E37F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-2.1816415E38F, -3.0792523E38F, 2.7854563E38F, 1.835259E38F}));
            assert(pack.alt_GET() == -5.47785E37F);
            assert(pack.custom_state_GET() == 3858396314299340422L);
            assert(pack.timestamp_GET() == 367657945148752648L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-1.319203E38F, -5.4457895E37F, 1.7209939E38F}));
            assert(pack.lat_GET() == 887300313);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.7367522E38F, -1.7603004E38F, -2.1994261E38F}));
            assert(pack.lon_GET() == 1155681129);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.vel_SET(new float[] {-1.319203E38F, -5.4457895E37F, 1.7209939E38F}, 0) ;
        p144.position_cov_SET(new float[] {-8.789467E37F, -2.288216E38F, 2.9884613E38F}, 0) ;
        p144.attitude_q_SET(new float[] {-2.1816415E38F, -3.0792523E38F, 2.7854563E38F, 1.835259E38F}, 0) ;
        p144.timestamp_SET(367657945148752648L) ;
        p144.lon_SET(1155681129) ;
        p144.rates_SET(new float[] {2.7580363E38F, -2.5999915E38F, 7.9231823E37F}, 0) ;
        p144.acc_SET(new float[] {2.7367522E38F, -1.7603004E38F, -2.1994261E38F}, 0) ;
        p144.lat_SET(887300313) ;
        p144.custom_state_SET(3858396314299340422L) ;
        p144.alt_SET(-5.47785E37F) ;
        p144.est_capabilities_SET((char)243) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-1.2568601E38F, 1.4366939E38F, -1.8013005E38F}));
            assert(pack.z_pos_GET() == 2.5633812E38F);
            assert(pack.x_pos_GET() == -1.7307402E38F);
            assert(pack.z_vel_GET() == 3.3266705E38F);
            assert(pack.roll_rate_GET() == 7.437041E37F);
            assert(pack.y_pos_GET() == -2.0999143E38F);
            assert(pack.z_acc_GET() == -1.8301907E38F);
            assert(pack.airspeed_GET() == 2.5426582E38F);
            assert(pack.x_acc_GET() == -2.4690008E38F);
            assert(pack.pitch_rate_GET() == 1.0062896E38F);
            assert(pack.x_vel_GET() == 7.4084533E37F);
            assert(pack.yaw_rate_GET() == -1.1766459E38F);
            assert(pack.time_usec_GET() == 2798288116346680641L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.33504E38F, 2.354116E38F, 8.332454E37F, -1.5805068E38F}));
            assert(pack.y_acc_GET() == 6.3570386E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {3.3890033E37F, 9.03138E37F, -3.3649022E37F}));
            assert(pack.y_vel_GET() == 2.9537306E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.z_pos_SET(2.5633812E38F) ;
        p146.airspeed_SET(2.5426582E38F) ;
        p146.roll_rate_SET(7.437041E37F) ;
        p146.y_acc_SET(6.3570386E37F) ;
        p146.q_SET(new float[] {-2.33504E38F, 2.354116E38F, 8.332454E37F, -1.5805068E38F}, 0) ;
        p146.z_vel_SET(3.3266705E38F) ;
        p146.pitch_rate_SET(1.0062896E38F) ;
        p146.x_vel_SET(7.4084533E37F) ;
        p146.time_usec_SET(2798288116346680641L) ;
        p146.y_pos_SET(-2.0999143E38F) ;
        p146.x_acc_SET(-2.4690008E38F) ;
        p146.x_pos_SET(-1.7307402E38F) ;
        p146.pos_variance_SET(new float[] {3.3890033E37F, 9.03138E37F, -3.3649022E37F}, 0) ;
        p146.z_acc_SET(-1.8301907E38F) ;
        p146.yaw_rate_SET(-1.1766459E38F) ;
        p146.y_vel_SET(2.9537306E38F) ;
        p146.vel_variance_SET(new float[] {-1.2568601E38F, 1.4366939E38F, -1.8013005E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_consumed_GET() == -55122195);
            assert(pack.current_battery_GET() == (short) -7002);
            assert(pack.id_GET() == (char)111);
            assert(pack.energy_consumed_GET() == 209159188);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.temperature_GET() == (short)20314);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            assert(pack.battery_remaining_GET() == (byte) - 109);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)7437, (char)7998, (char)29356, (char)22206, (char)46992, (char)39978, (char)5323, (char)30862, (char)22779, (char)28003}));
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.temperature_SET((short)20314) ;
        p147.current_consumed_SET(-55122195) ;
        p147.current_battery_SET((short) -7002) ;
        p147.voltages_SET(new char[] {(char)7437, (char)7998, (char)29356, (char)22206, (char)46992, (char)39978, (char)5323, (char)30862, (char)22779, (char)28003}, 0) ;
        p147.battery_remaining_SET((byte) - 109) ;
        p147.energy_consumed_SET(209159188) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL) ;
        p147.id_SET((char)111) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION);
            assert(pack.board_version_GET() == 1559833879L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)0, (char)94, (char)226, (char)100, (char)59, (char)70, (char)240, (char)237, (char)184, (char)246, (char)226, (char)230, (char)245, (char)187, (char)188, (char)22, (char)224, (char)90}));
            assert(pack.flight_sw_version_GET() == 3730901575L);
            assert(pack.uid_GET() == 755403621032359299L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)16, (char)158, (char)160, (char)164, (char)233, (char)217, (char)252, (char)43}));
            assert(pack.os_sw_version_GET() == 3590308419L);
            assert(pack.vendor_id_GET() == (char)2737);
            assert(pack.middleware_sw_version_GET() == 10844968L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)41, (char)103, (char)60, (char)97, (char)210, (char)177, (char)49, (char)152}));
            assert(pack.product_id_GET() == (char)5050);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)9, (char)144, (char)93, (char)13, (char)156, (char)27, (char)86, (char)248}));
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.vendor_id_SET((char)2737) ;
        p148.uid2_SET(new char[] {(char)0, (char)94, (char)226, (char)100, (char)59, (char)70, (char)240, (char)237, (char)184, (char)246, (char)226, (char)230, (char)245, (char)187, (char)188, (char)22, (char)224, (char)90}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)41, (char)103, (char)60, (char)97, (char)210, (char)177, (char)49, (char)152}, 0) ;
        p148.product_id_SET((char)5050) ;
        p148.middleware_custom_version_SET(new char[] {(char)9, (char)144, (char)93, (char)13, (char)156, (char)27, (char)86, (char)248}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION) ;
        p148.flight_sw_version_SET(3730901575L) ;
        p148.board_version_SET(1559833879L) ;
        p148.uid_SET(755403621032359299L) ;
        p148.flight_custom_version_SET(new char[] {(char)16, (char)158, (char)160, (char)164, (char)233, (char)217, (char)252, (char)43}, 0) ;
        p148.middleware_sw_version_SET(10844968L) ;
        p148.os_sw_version_SET(3590308419L) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.x_TRY(ph) == -2.0717157E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.distance_GET() == -2.7676632E38F);
            assert(pack.position_valid_TRY(ph) == (char)221);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-9.546568E37F, -8.77175E37F, -2.5775507E38F, 1.5494175E38F}));
            assert(pack.size_x_GET() == 1.1105243E38F);
            assert(pack.time_usec_GET() == 6080297232198394630L);
            assert(pack.target_num_GET() == (char)45);
            assert(pack.y_TRY(ph) == -2.1889387E38F);
            assert(pack.angle_x_GET() == 1.4362136E38F);
            assert(pack.z_TRY(ph) == 6.4972265E37F);
            assert(pack.angle_y_GET() == -1.1560219E38F);
            assert(pack.size_y_GET() == 2.5051084E38F);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)45) ;
        p149.x_SET(-2.0717157E38F, PH) ;
        p149.size_y_SET(2.5051084E38F) ;
        p149.angle_x_SET(1.4362136E38F) ;
        p149.y_SET(-2.1889387E38F, PH) ;
        p149.distance_SET(-2.7676632E38F) ;
        p149.q_SET(new float[] {-9.546568E37F, -8.77175E37F, -2.5775507E38F, 1.5494175E38F}, 0, PH) ;
        p149.position_valid_SET((char)221, PH) ;
        p149.size_x_SET(1.1105243E38F) ;
        p149.z_SET(6.4972265E37F, PH) ;
        p149.time_usec_SET(6080297232198394630L) ;
        p149.angle_y_SET(-1.1560219E38F) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_declination_GET() == -2.997554E38F);
            assert(pack.mag_ofs_y_GET() == (short) -5061);
            assert(pack.raw_press_GET() == -2029538845);
            assert(pack.gyro_cal_z_GET() == 5.3279963E37F);
            assert(pack.mag_ofs_z_GET() == (short) -8690);
            assert(pack.accel_cal_x_GET() == 2.4700337E37F);
            assert(pack.mag_ofs_x_GET() == (short)25691);
            assert(pack.accel_cal_z_GET() == -1.0631437E38F);
            assert(pack.gyro_cal_y_GET() == 1.5355915E38F);
            assert(pack.gyro_cal_x_GET() == -3.0224603E38F);
            assert(pack.raw_temp_GET() == -421191473);
            assert(pack.accel_cal_y_GET() == -1.6314444E38F);
        });
        DemoDevice.SENSOR_OFFSETS p150 = LoopBackDemoChannel.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.gyro_cal_z_SET(5.3279963E37F) ;
        p150.accel_cal_y_SET(-1.6314444E38F) ;
        p150.raw_temp_SET(-421191473) ;
        p150.mag_ofs_x_SET((short)25691) ;
        p150.mag_declination_SET(-2.997554E38F) ;
        p150.raw_press_SET(-2029538845) ;
        p150.mag_ofs_z_SET((short) -8690) ;
        p150.accel_cal_x_SET(2.4700337E37F) ;
        p150.accel_cal_z_SET(-1.0631437E38F) ;
        p150.mag_ofs_y_SET((short) -5061) ;
        p150.gyro_cal_y_SET(1.5355915E38F) ;
        p150.gyro_cal_x_SET(-3.0224603E38F) ;
        LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            assert(pack.mag_ofs_x_GET() == (short) -20879);
            assert(pack.mag_ofs_z_GET() == (short)1067);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.mag_ofs_y_GET() == (short) -31058);
            assert(pack.target_system_GET() == (char)243);
        });
        DemoDevice.SET_MAG_OFFSETS p151 = LoopBackDemoChannel.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.mag_ofs_z_SET((short)1067) ;
        p151.target_component_SET((char)237) ;
        p151.mag_ofs_x_SET((short) -20879) ;
        p151.target_system_SET((char)243) ;
        p151.mag_ofs_y_SET((short) -31058) ;
        LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            assert(pack.freemem32_TRY(ph) == 4265525800L);
            assert(pack.freemem_GET() == (char)30238);
            assert(pack.brkval_GET() == (char)30381);
        });
        DemoDevice.MEMINFO p152 = LoopBackDemoChannel.new_MEMINFO();
        PH.setPack(p152);
        p152.freemem_SET((char)30238) ;
        p152.brkval_SET((char)30381) ;
        p152.freemem32_SET(4265525800L, PH) ;
        LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            assert(pack.adc4_GET() == (char)61027);
            assert(pack.adc1_GET() == (char)55391);
            assert(pack.adc3_GET() == (char)28107);
            assert(pack.adc6_GET() == (char)20733);
            assert(pack.adc5_GET() == (char)53726);
            assert(pack.adc2_GET() == (char)58881);
        });
        DemoDevice.AP_ADC p153 = LoopBackDemoChannel.new_AP_ADC();
        PH.setPack(p153);
        p153.adc3_SET((char)28107) ;
        p153.adc4_SET((char)61027) ;
        p153.adc6_SET((char)20733) ;
        p153.adc2_SET((char)58881) ;
        p153.adc5_SET((char)53726) ;
        p153.adc1_SET((char)55391) ;
        LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.engine_cut_off_GET() == (char)35);
            assert(pack.command_id_GET() == (char)239);
            assert(pack.iso_GET() == (char)211);
            assert(pack.exposure_type_GET() == (char)234);
            assert(pack.mode_GET() == (char)58);
            assert(pack.shutter_speed_GET() == (char)25934);
            assert(pack.extra_value_GET() == 2.204452E38F);
            assert(pack.aperture_GET() == (char)43);
            assert(pack.target_system_GET() == (char)159);
            assert(pack.extra_param_GET() == (char)134);
            assert(pack.target_component_GET() == (char)164);
        });
        DemoDevice.DIGICAM_CONFIGURE p154 = LoopBackDemoChannel.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_component_SET((char)164) ;
        p154.mode_SET((char)58) ;
        p154.extra_param_SET((char)134) ;
        p154.engine_cut_off_SET((char)35) ;
        p154.target_system_SET((char)159) ;
        p154.iso_SET((char)211) ;
        p154.extra_value_SET(2.204452E38F) ;
        p154.shutter_speed_SET((char)25934) ;
        p154.exposure_type_SET((char)234) ;
        p154.command_id_SET((char)239) ;
        p154.aperture_SET((char)43) ;
        LoopBackDemoChannel.instance.send(p154);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)180);
            assert(pack.command_id_GET() == (char)93);
            assert(pack.session_GET() == (char)247);
            assert(pack.shot_GET() == (char)125);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.zoom_pos_GET() == (char)41);
            assert(pack.extra_param_GET() == (char)6);
            assert(pack.focus_lock_GET() == (char)250);
            assert(pack.extra_value_GET() == -2.095281E38F);
            assert(pack.zoom_step_GET() == (byte) - 74);
        });
        DemoDevice.DIGICAM_CONTROL p155 = LoopBackDemoChannel.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.target_system_SET((char)180) ;
        p155.extra_value_SET(-2.095281E38F) ;
        p155.zoom_pos_SET((char)41) ;
        p155.zoom_step_SET((byte) - 74) ;
        p155.command_id_SET((char)93) ;
        p155.session_SET((char)247) ;
        p155.target_component_SET((char)156) ;
        p155.focus_lock_SET((char)250) ;
        p155.shot_SET((char)125) ;
        p155.extra_param_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            assert(pack.mount_mode_GET() == MAV_MOUNT_MODE.MAV_MOUNT_MODE_NEUTRAL);
            assert(pack.stab_roll_GET() == (char)22);
            assert(pack.stab_pitch_GET() == (char)76);
            assert(pack.target_system_GET() == (char)143);
            assert(pack.stab_yaw_GET() == (char)117);
            assert(pack.target_component_GET() == (char)232);
        });
        DemoDevice.MOUNT_CONFIGURE p156 = LoopBackDemoChannel.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.target_component_SET((char)232) ;
        p156.target_system_SET((char)143) ;
        p156.stab_yaw_SET((char)117) ;
        p156.stab_roll_SET((char)22) ;
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_NEUTRAL) ;
        p156.stab_pitch_SET((char)76) ;
        LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)168);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.input_a_GET() == 878331342);
            assert(pack.input_c_GET() == 648753160);
            assert(pack.save_position_GET() == (char)65);
            assert(pack.input_b_GET() == 1442138397);
        });
        DemoDevice.MOUNT_CONTROL p157 = LoopBackDemoChannel.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.input_b_SET(1442138397) ;
        p157.input_c_SET(648753160) ;
        p157.target_system_SET((char)168) ;
        p157.save_position_SET((char)65) ;
        p157.target_component_SET((char)113) ;
        p157.input_a_SET(878331342) ;
        LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.pointing_c_GET() == 1916990623);
            assert(pack.target_component_GET() == (char)184);
            assert(pack.pointing_a_GET() == -1873959504);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.pointing_b_GET() == 456059226);
        });
        DemoDevice.MOUNT_STATUS p158 = LoopBackDemoChannel.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_component_SET((char)184) ;
        p158.target_system_SET((char)240) ;
        p158.pointing_a_SET(-1873959504) ;
        p158.pointing_c_SET(1916990623) ;
        p158.pointing_b_SET(456059226) ;
        LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            assert(pack.lng_GET() == -2.8184814E38F);
            assert(pack.count_GET() == (char)239);
            assert(pack.target_component_GET() == (char)136);
            assert(pack.target_system_GET() == (char)234);
            assert(pack.lat_GET() == 7.5202434E37F);
            assert(pack.idx_GET() == (char)215);
        });
        DemoDevice.FENCE_POINT p160 = LoopBackDemoChannel.new_FENCE_POINT();
        PH.setPack(p160);
        p160.target_system_SET((char)234) ;
        p160.target_component_SET((char)136) ;
        p160.idx_SET((char)215) ;
        p160.lat_SET(7.5202434E37F) ;
        p160.count_SET((char)239) ;
        p160.lng_SET(-2.8184814E38F) ;
        LoopBackDemoChannel.instance.send(p160);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)0);
            assert(pack.idx_GET() == (char)26);
            assert(pack.target_component_GET() == (char)213);
        });
        DemoDevice.FENCE_FETCH_POINT p161 = LoopBackDemoChannel.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.target_component_SET((char)213) ;
        p161.target_system_SET((char)0) ;
        p161.idx_SET((char)26) ;
        LoopBackDemoChannel.instance.send(p161);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.breach_type_GET() == FENCE_BREACH.FENCE_BREACH_NONE);
            assert(pack.breach_count_GET() == (char)31516);
            assert(pack.breach_time_GET() == 288868676L);
            assert(pack.breach_status_GET() == (char)206);
        });
        DemoDevice.FENCE_STATUS p162 = LoopBackDemoChannel.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_NONE) ;
        p162.breach_status_SET((char)206) ;
        p162.breach_count_SET((char)31516) ;
        p162.breach_time_SET(288868676L) ;
        LoopBackDemoChannel.instance.send(p162);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            assert(pack.accel_weight_GET() == -1.4317987E38F);
            assert(pack.omegaIx_GET() == -8.2552465E37F);
            assert(pack.error_yaw_GET() == 1.6272722E37F);
            assert(pack.omegaIz_GET() == -9.506184E37F);
            assert(pack.omegaIy_GET() == -2.7045348E38F);
            assert(pack.error_rp_GET() == -2.3822174E37F);
            assert(pack.renorm_val_GET() == 1.4164918E38F);
        });
        DemoDevice.AHRS p163 = LoopBackDemoChannel.new_AHRS();
        PH.setPack(p163);
        p163.omegaIx_SET(-8.2552465E37F) ;
        p163.omegaIz_SET(-9.506184E37F) ;
        p163.error_yaw_SET(1.6272722E37F) ;
        p163.omegaIy_SET(-2.7045348E38F) ;
        p163.renorm_val_SET(1.4164918E38F) ;
        p163.error_rp_SET(-2.3822174E37F) ;
        p163.accel_weight_SET(-1.4317987E38F) ;
        LoopBackDemoChannel.instance.send(p163);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.5114095E38F);
            assert(pack.pitch_GET() == 1.8335582E38F);
            assert(pack.lat_GET() == -630272040);
            assert(pack.lng_GET() == 1226931119);
            assert(pack.xacc_GET() == 1.2761561E38F);
            assert(pack.ygyro_GET() == 3.17638E38F);
            assert(pack.yacc_GET() == -1.4246116E38F);
            assert(pack.yaw_GET() == -1.8474827E38F);
            assert(pack.xgyro_GET() == 2.5106786E38F);
            assert(pack.zgyro_GET() == 2.9021205E37F);
            assert(pack.zacc_GET() == -2.4921947E37F);
        });
        DemoDevice.SIMSTATE p164 = LoopBackDemoChannel.new_SIMSTATE();
        PH.setPack(p164);
        p164.yacc_SET(-1.4246116E38F) ;
        p164.xgyro_SET(2.5106786E38F) ;
        p164.lng_SET(1226931119) ;
        p164.ygyro_SET(3.17638E38F) ;
        p164.zgyro_SET(2.9021205E37F) ;
        p164.pitch_SET(1.8335582E38F) ;
        p164.xacc_SET(1.2761561E38F) ;
        p164.roll_SET(2.5114095E38F) ;
        p164.yaw_SET(-1.8474827E38F) ;
        p164.zacc_SET(-2.4921947E37F) ;
        p164.lat_SET(-630272040) ;
        LoopBackDemoChannel.instance.send(p164);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            assert(pack.I2Cerr_GET() == (char)32);
            assert(pack.Vcc_GET() == (char)56922);
        });
        DemoDevice.HWSTATUS p165 = LoopBackDemoChannel.new_HWSTATUS();
        PH.setPack(p165);
        p165.I2Cerr_SET((char)32) ;
        p165.Vcc_SET((char)56922) ;
        LoopBackDemoChannel.instance.send(p165);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)27202);
            assert(pack.remnoise_GET() == (char)2);
            assert(pack.remrssi_GET() == (char)123);
            assert(pack.rxerrors_GET() == (char)57120);
            assert(pack.rssi_GET() == (char)108);
            assert(pack.txbuf_GET() == (char)187);
            assert(pack.noise_GET() == (char)118);
        });
        DemoDevice.RADIO p166 = LoopBackDemoChannel.new_RADIO();
        PH.setPack(p166);
        p166.txbuf_SET((char)187) ;
        p166.rxerrors_SET((char)57120) ;
        p166.noise_SET((char)118) ;
        p166.remrssi_SET((char)123) ;
        p166.remnoise_SET((char)2) ;
        p166.fixed__SET((char)27202) ;
        p166.rssi_SET((char)108) ;
        LoopBackDemoChannel.instance.send(p166);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.last_clear_GET() == 969025317L);
            assert(pack.mods_triggered_GET() == LIMIT_MODULE.LIMIT_GPSLOCK);
            assert(pack.breach_count_GET() == (char)58532);
            assert(pack.mods_enabled_GET() == LIMIT_MODULE.LIMIT_GPSLOCK);
            assert(pack.last_recovery_GET() == 858682425L);
            assert(pack.mods_required_GET() == LIMIT_MODULE.LIMIT_ALTITUDE);
            assert(pack.last_action_GET() == 1483906041L);
            assert(pack.last_trigger_GET() == 2820424982L);
            assert(pack.limits_state_GET() == LIMITS_STATE.LIMITS_TRIGGERED);
        });
        DemoDevice.LIMITS_STATUS p167 = LoopBackDemoChannel.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.last_recovery_SET(858682425L) ;
        p167.last_clear_SET(969025317L) ;
        p167.breach_count_SET((char)58532) ;
        p167.last_trigger_SET(2820424982L) ;
        p167.mods_required_SET(LIMIT_MODULE.LIMIT_ALTITUDE) ;
        p167.mods_triggered_SET(LIMIT_MODULE.LIMIT_GPSLOCK) ;
        p167.last_action_SET(1483906041L) ;
        p167.mods_enabled_SET(LIMIT_MODULE.LIMIT_GPSLOCK) ;
        p167.limits_state_SET(LIMITS_STATE.LIMITS_TRIGGERED) ;
        LoopBackDemoChannel.instance.send(p167);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            assert(pack.speed_GET() == -2.793882E38F);
            assert(pack.speed_z_GET() == 2.2407218E37F);
            assert(pack.direction_GET() == 2.9670524E38F);
        });
        DemoDevice.WIND p168 = LoopBackDemoChannel.new_WIND();
        PH.setPack(p168);
        p168.speed_SET(-2.793882E38F) ;
        p168.direction_SET(2.9670524E38F) ;
        p168.speed_z_SET(2.2407218E37F) ;
        LoopBackDemoChannel.instance.send(p168);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)48);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)185, (char)172, (char)86, (char)206, (char)129, (char)70, (char)62, (char)245, (char)95, (char)159, (char)224, (char)129, (char)161, (char)162, (char)208, (char)45}));
            assert(pack.len_GET() == (char)207);
        });
        DemoDevice.DATA16 p169 = LoopBackDemoChannel.new_DATA16();
        PH.setPack(p169);
        p169.data__SET(new char[] {(char)185, (char)172, (char)86, (char)206, (char)129, (char)70, (char)62, (char)245, (char)95, (char)159, (char)224, (char)129, (char)161, (char)162, (char)208, (char)45}, 0) ;
        p169.type_SET((char)48) ;
        p169.len_SET((char)207) ;
        LoopBackDemoChannel.instance.send(p169);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)15);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)94, (char)176, (char)245, (char)176, (char)230, (char)251, (char)70, (char)138, (char)245, (char)250, (char)249, (char)81, (char)205, (char)167, (char)186, (char)163, (char)166, (char)12, (char)203, (char)230, (char)135, (char)200, (char)227, (char)236, (char)117, (char)27, (char)30, (char)174, (char)77, (char)33, (char)167, (char)214}));
            assert(pack.type_GET() == (char)125);
        });
        DemoDevice.DATA32 p170 = LoopBackDemoChannel.new_DATA32();
        PH.setPack(p170);
        p170.data__SET(new char[] {(char)94, (char)176, (char)245, (char)176, (char)230, (char)251, (char)70, (char)138, (char)245, (char)250, (char)249, (char)81, (char)205, (char)167, (char)186, (char)163, (char)166, (char)12, (char)203, (char)230, (char)135, (char)200, (char)227, (char)236, (char)117, (char)27, (char)30, (char)174, (char)77, (char)33, (char)167, (char)214}, 0) ;
        p170.len_SET((char)15) ;
        p170.type_SET((char)125) ;
        LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)240, (char)100, (char)193, (char)138, (char)219, (char)167, (char)6, (char)77, (char)183, (char)216, (char)132, (char)73, (char)130, (char)181, (char)49, (char)114, (char)123, (char)80, (char)33, (char)212, (char)95, (char)7, (char)216, (char)186, (char)178, (char)44, (char)101, (char)47, (char)40, (char)236, (char)51, (char)127, (char)121, (char)31, (char)104, (char)215, (char)233, (char)56, (char)6, (char)146, (char)45, (char)179, (char)238, (char)17, (char)11, (char)199, (char)51, (char)111, (char)200, (char)5, (char)164, (char)204, (char)113, (char)78, (char)43, (char)190, (char)134, (char)34, (char)235, (char)199, (char)220, (char)26, (char)142, (char)173}));
            assert(pack.len_GET() == (char)154);
            assert(pack.type_GET() == (char)208);
        });
        DemoDevice.DATA64 p171 = LoopBackDemoChannel.new_DATA64();
        PH.setPack(p171);
        p171.data__SET(new char[] {(char)240, (char)100, (char)193, (char)138, (char)219, (char)167, (char)6, (char)77, (char)183, (char)216, (char)132, (char)73, (char)130, (char)181, (char)49, (char)114, (char)123, (char)80, (char)33, (char)212, (char)95, (char)7, (char)216, (char)186, (char)178, (char)44, (char)101, (char)47, (char)40, (char)236, (char)51, (char)127, (char)121, (char)31, (char)104, (char)215, (char)233, (char)56, (char)6, (char)146, (char)45, (char)179, (char)238, (char)17, (char)11, (char)199, (char)51, (char)111, (char)200, (char)5, (char)164, (char)204, (char)113, (char)78, (char)43, (char)190, (char)134, (char)34, (char)235, (char)199, (char)220, (char)26, (char)142, (char)173}, 0) ;
        p171.len_SET((char)154) ;
        p171.type_SET((char)208) ;
        LoopBackDemoChannel.instance.send(p171);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)0);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)115, (char)153, (char)124, (char)66, (char)199, (char)239, (char)168, (char)123, (char)213, (char)86, (char)18, (char)218, (char)67, (char)182, (char)201, (char)241, (char)102, (char)171, (char)210, (char)86, (char)48, (char)94, (char)17, (char)207, (char)245, (char)36, (char)211, (char)179, (char)116, (char)234, (char)217, (char)172, (char)22, (char)161, (char)196, (char)99, (char)244, (char)177, (char)203, (char)182, (char)66, (char)105, (char)209, (char)111, (char)140, (char)138, (char)147, (char)249, (char)194, (char)138, (char)7, (char)197, (char)89, (char)217, (char)112, (char)171, (char)66, (char)12, (char)10, (char)155, (char)224, (char)85, (char)213, (char)78, (char)184, (char)105, (char)24, (char)136, (char)41, (char)12, (char)0, (char)196, (char)130, (char)48, (char)52, (char)62, (char)230, (char)240, (char)103, (char)99, (char)66, (char)176, (char)228, (char)136, (char)0, (char)22, (char)204, (char)102, (char)134, (char)105, (char)49, (char)145, (char)185, (char)156, (char)188, (char)87}));
            assert(pack.len_GET() == (char)175);
        });
        DemoDevice.DATA96 p172 = LoopBackDemoChannel.new_DATA96();
        PH.setPack(p172);
        p172.len_SET((char)175) ;
        p172.type_SET((char)0) ;
        p172.data__SET(new char[] {(char)115, (char)153, (char)124, (char)66, (char)199, (char)239, (char)168, (char)123, (char)213, (char)86, (char)18, (char)218, (char)67, (char)182, (char)201, (char)241, (char)102, (char)171, (char)210, (char)86, (char)48, (char)94, (char)17, (char)207, (char)245, (char)36, (char)211, (char)179, (char)116, (char)234, (char)217, (char)172, (char)22, (char)161, (char)196, (char)99, (char)244, (char)177, (char)203, (char)182, (char)66, (char)105, (char)209, (char)111, (char)140, (char)138, (char)147, (char)249, (char)194, (char)138, (char)7, (char)197, (char)89, (char)217, (char)112, (char)171, (char)66, (char)12, (char)10, (char)155, (char)224, (char)85, (char)213, (char)78, (char)184, (char)105, (char)24, (char)136, (char)41, (char)12, (char)0, (char)196, (char)130, (char)48, (char)52, (char)62, (char)230, (char)240, (char)103, (char)99, (char)66, (char)176, (char)228, (char)136, (char)0, (char)22, (char)204, (char)102, (char)134, (char)105, (char)49, (char)145, (char)185, (char)156, (char)188, (char)87}, 0) ;
        LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            assert(pack.distance_GET() == -2.6275742E38F);
            assert(pack.voltage_GET() == 1.5140529E38F);
        });
        DemoDevice.RANGEFINDER p173 = LoopBackDemoChannel.new_RANGEFINDER();
        PH.setPack(p173);
        p173.distance_SET(-2.6275742E38F) ;
        p173.voltage_SET(1.5140529E38F) ;
        LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            assert(pack.state_y_GET() == 9.466627E37F);
            assert(pack.Pby_GET() == -2.3288285E38F);
            assert(pack.ratio_GET() == 3.8269145E37F);
            assert(pack.diff_pressure_GET() == -2.9376902E38F);
            assert(pack.EAS2TAS_GET() == 1.096524E38F);
            assert(pack.state_x_GET() == -1.2755748E38F);
            assert(pack.vy_GET() == -2.2663893E38F);
            assert(pack.Pax_GET() == 2.6855287E38F);
            assert(pack.Pcz_GET() == -1.2138188E37F);
            assert(pack.vz_GET() == 1.0003171E37F);
            assert(pack.state_z_GET() == 2.1226408E38F);
            assert(pack.vx_GET() == -2.0353242E38F);
        });
        DemoDevice.AIRSPEED_AUTOCAL p174 = LoopBackDemoChannel.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.vx_SET(-2.0353242E38F) ;
        p174.state_z_SET(2.1226408E38F) ;
        p174.Pcz_SET(-1.2138188E37F) ;
        p174.EAS2TAS_SET(1.096524E38F) ;
        p174.vy_SET(-2.2663893E38F) ;
        p174.vz_SET(1.0003171E37F) ;
        p174.state_y_SET(9.466627E37F) ;
        p174.state_x_SET(-1.2755748E38F) ;
        p174.Pax_SET(2.6855287E38F) ;
        p174.Pby_SET(-2.3288285E38F) ;
        p174.diff_pressure_SET(-2.9376902E38F) ;
        p174.ratio_SET(3.8269145E37F) ;
        LoopBackDemoChannel.instance.send(p174);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)222);
            assert(pack.lat_GET() == -1065115558);
            assert(pack.lng_GET() == -1944513582);
            assert(pack.land_dir_GET() == (char)7717);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.break_alt_GET() == (short)8807);
            assert(pack.flags_GET() == RALLY_FLAGS.LAND_IMMEDIATELY);
            assert(pack.idx_GET() == (char)25);
            assert(pack.count_GET() == (char)7);
            assert(pack.alt_GET() == (short)13901);
        });
        DemoDevice.RALLY_POINT p175 = LoopBackDemoChannel.new_RALLY_POINT();
        PH.setPack(p175);
        p175.flags_SET(RALLY_FLAGS.LAND_IMMEDIATELY) ;
        p175.alt_SET((short)13901) ;
        p175.land_dir_SET((char)7717) ;
        p175.lat_SET(-1065115558) ;
        p175.count_SET((char)7) ;
        p175.break_alt_SET((short)8807) ;
        p175.target_component_SET((char)222) ;
        p175.target_system_SET((char)240) ;
        p175.lng_SET(-1944513582) ;
        p175.idx_SET((char)25) ;
        LoopBackDemoChannel.instance.send(p175);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)199);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.idx_GET() == (char)23);
        });
        DemoDevice.RALLY_FETCH_POINT p176 = LoopBackDemoChannel.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)199) ;
        p176.target_component_SET((char)199) ;
        p176.idx_SET((char)23) ;
        LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            assert(pack.CompensationX_GET() == -2.9147952E38F);
            assert(pack.current_GET() == -6.48507E37F);
            assert(pack.interference_GET() == (char)22214);
            assert(pack.CompensationZ_GET() == -4.7647746E37F);
            assert(pack.CompensationY_GET() == 1.8440779E38F);
            assert(pack.throttle_GET() == (char)18908);
        });
        DemoDevice.COMPASSMOT_STATUS p177 = LoopBackDemoChannel.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.current_SET(-6.48507E37F) ;
        p177.CompensationZ_SET(-4.7647746E37F) ;
        p177.CompensationY_SET(1.8440779E38F) ;
        p177.CompensationX_SET(-2.9147952E38F) ;
        p177.interference_SET((char)22214) ;
        p177.throttle_SET((char)18908) ;
        LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 2.4542616E38F);
            assert(pack.lng_GET() == 2012968414);
            assert(pack.lat_GET() == -381834035);
            assert(pack.roll_GET() == -1.6740041E38F);
            assert(pack.pitch_GET() == -5.0636364E37F);
            assert(pack.altitude_GET() == 7.2758687E37F);
        });
        DemoDevice.AHRS2 p178 = LoopBackDemoChannel.new_AHRS2();
        PH.setPack(p178);
        p178.lng_SET(2012968414) ;
        p178.lat_SET(-381834035) ;
        p178.yaw_SET(2.4542616E38F) ;
        p178.pitch_SET(-5.0636364E37F) ;
        p178.roll_SET(-1.6740041E38F) ;
        p178.altitude_SET(7.2758687E37F) ;
        LoopBackDemoChannel.instance.send(p178);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            assert(pack.p4_GET() == -1.7491797E38F);
            assert(pack.img_idx_GET() == (char)50670);
            assert(pack.target_system_GET() == (char)99);
            assert(pack.time_usec_GET() == 2810563702414826079L);
            assert(pack.p2_GET() == -2.6398607E38F);
            assert(pack.event_id_GET() == CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTOREV);
            assert(pack.p1_GET() == -1.9664968E38F);
            assert(pack.p3_GET() == -3.2425577E38F);
            assert(pack.cam_idx_GET() == (char)147);
        });
        DemoDevice.CAMERA_STATUS p179 = LoopBackDemoChannel.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTOREV) ;
        p179.p3_SET(-3.2425577E38F) ;
        p179.target_system_SET((char)99) ;
        p179.p4_SET(-1.7491797E38F) ;
        p179.img_idx_SET((char)50670) ;
        p179.cam_idx_SET((char)147) ;
        p179.time_usec_SET(2810563702414826079L) ;
        p179.p1_SET(-1.9664968E38F) ;
        p179.p2_SET(-2.6398607E38F) ;
        LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == -1.3156779E38F);
            assert(pack.cam_idx_GET() == (char)217);
            assert(pack.foc_len_GET() == 4.478157E37F);
            assert(pack.flags_GET() == CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_VIDEO);
            assert(pack.img_idx_GET() == (char)51398);
            assert(pack.yaw_GET() == -2.416389E38F);
            assert(pack.alt_rel_GET() == 2.1469635E37F);
            assert(pack.time_usec_GET() == 2322740339665438013L);
            assert(pack.pitch_GET() == 3.687937E37F);
            assert(pack.lng_GET() == -900338472);
            assert(pack.target_system_GET() == (char)158);
            assert(pack.lat_GET() == -626524562);
            assert(pack.alt_msl_GET() == 2.9496193E38F);
        });
        DemoDevice.CAMERA_FEEDBACK p180 = LoopBackDemoChannel.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.time_usec_SET(2322740339665438013L) ;
        p180.alt_rel_SET(2.1469635E37F) ;
        p180.cam_idx_SET((char)217) ;
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_VIDEO) ;
        p180.alt_msl_SET(2.9496193E38F) ;
        p180.yaw_SET(-2.416389E38F) ;
        p180.lng_SET(-900338472) ;
        p180.target_system_SET((char)158) ;
        p180.roll_SET(-1.3156779E38F) ;
        p180.pitch_SET(3.687937E37F) ;
        p180.img_idx_SET((char)51398) ;
        p180.foc_len_SET(4.478157E37F) ;
        p180.lat_SET(-626524562) ;
        LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            assert(pack.voltage_GET() == (char)60479);
            assert(pack.current_battery_GET() == (short) -21673);
        });
        DemoDevice.BATTERY2 p181 = LoopBackDemoChannel.new_BATTERY2();
        PH.setPack(p181);
        p181.voltage_SET((char)60479) ;
        p181.current_battery_SET((short) -21673) ;
        LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.058803E38F);
            assert(pack.v2_GET() == -1.9780278E37F);
            assert(pack.v3_GET() == -3.1173086E38F);
            assert(pack.v1_GET() == -3.1033955E38F);
            assert(pack.yaw_GET() == -3.2985936E38F);
            assert(pack.lat_GET() == 786490003);
            assert(pack.lng_GET() == -1554354156);
            assert(pack.v4_GET() == 1.8689492E38F);
            assert(pack.roll_GET() == -5.761618E37F);
            assert(pack.altitude_GET() == 1.6854808E38F);
        });
        DemoDevice.AHRS3 p182 = LoopBackDemoChannel.new_AHRS3();
        PH.setPack(p182);
        p182.yaw_SET(-3.2985936E38F) ;
        p182.pitch_SET(1.058803E38F) ;
        p182.altitude_SET(1.6854808E38F) ;
        p182.v4_SET(1.8689492E38F) ;
        p182.roll_SET(-5.761618E37F) ;
        p182.v3_SET(-3.1173086E38F) ;
        p182.v1_SET(-3.1033955E38F) ;
        p182.lat_SET(786490003) ;
        p182.v2_SET(-1.9780278E37F) ;
        p182.lng_SET(-1554354156) ;
        LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)141);
            assert(pack.target_component_GET() == (char)109);
        });
        DemoDevice.AUTOPILOT_VERSION_REQUEST p183 = LoopBackDemoChannel.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_component_SET((char)109) ;
        p183.target_system_SET((char)141) ;
        LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)64);
            assert(pack.seqno_GET() == MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            assert(pack.target_system_GET() == (char)73);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)221, (char)201, (char)68, (char)8, (char)7, (char)3, (char)236, (char)74, (char)109, (char)107, (char)247, (char)171, (char)171, (char)6, (char)166, (char)179, (char)199, (char)16, (char)112, (char)58, (char)13, (char)24, (char)60, (char)248, (char)105, (char)48, (char)137, (char)184, (char)16, (char)163, (char)94, (char)217, (char)8, (char)76, (char)178, (char)15, (char)73, (char)204, (char)213, (char)131, (char)144, (char)176, (char)211, (char)42, (char)44, (char)121, (char)26, (char)241, (char)138, (char)174, (char)40, (char)58, (char)193, (char)7, (char)6, (char)124, (char)123, (char)130, (char)59, (char)17, (char)156, (char)206, (char)77, (char)15, (char)134, (char)115, (char)128, (char)200, (char)16, (char)51, (char)225, (char)70, (char)117, (char)212, (char)185, (char)82, (char)184, (char)11, (char)224, (char)18, (char)61, (char)116, (char)119, (char)251, (char)142, (char)131, (char)90, (char)230, (char)243, (char)118, (char)9, (char)2, (char)89, (char)74, (char)167, (char)212, (char)27, (char)89, (char)91, (char)188, (char)216, (char)4, (char)56, (char)131, (char)221, (char)125, (char)219, (char)178, (char)122, (char)211, (char)152, (char)168, (char)182, (char)217, (char)225, (char)77, (char)85, (char)107, (char)99, (char)246, (char)143, (char)56, (char)64, (char)59, (char)32, (char)70, (char)115, (char)237, (char)254, (char)22, (char)213, (char)157, (char)204, (char)252, (char)176, (char)131, (char)98, (char)181, (char)166, (char)94, (char)134, (char)56, (char)219, (char)113, (char)151, (char)64, (char)153, (char)12, (char)238, (char)178, (char)121, (char)152, (char)23, (char)46, (char)126, (char)156, (char)192, (char)240, (char)11, (char)120, (char)220, (char)237, (char)149, (char)176, (char)160, (char)115, (char)16, (char)220, (char)159, (char)143, (char)253, (char)50, (char)126, (char)112, (char)155, (char)179, (char)44, (char)165, (char)209, (char)205, (char)118, (char)72, (char)1, (char)29, (char)228, (char)5, (char)30, (char)62, (char)207, (char)111, (char)224, (char)7, (char)1, (char)11, (char)75, (char)88, (char)175, (char)45, (char)171, (char)205}));
        });
        DemoDevice.REMOTE_LOG_DATA_BLOCK p184 = LoopBackDemoChannel.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.target_system_SET((char)73) ;
        p184.target_component_SET((char)64) ;
        p184.data__SET(new char[] {(char)221, (char)201, (char)68, (char)8, (char)7, (char)3, (char)236, (char)74, (char)109, (char)107, (char)247, (char)171, (char)171, (char)6, (char)166, (char)179, (char)199, (char)16, (char)112, (char)58, (char)13, (char)24, (char)60, (char)248, (char)105, (char)48, (char)137, (char)184, (char)16, (char)163, (char)94, (char)217, (char)8, (char)76, (char)178, (char)15, (char)73, (char)204, (char)213, (char)131, (char)144, (char)176, (char)211, (char)42, (char)44, (char)121, (char)26, (char)241, (char)138, (char)174, (char)40, (char)58, (char)193, (char)7, (char)6, (char)124, (char)123, (char)130, (char)59, (char)17, (char)156, (char)206, (char)77, (char)15, (char)134, (char)115, (char)128, (char)200, (char)16, (char)51, (char)225, (char)70, (char)117, (char)212, (char)185, (char)82, (char)184, (char)11, (char)224, (char)18, (char)61, (char)116, (char)119, (char)251, (char)142, (char)131, (char)90, (char)230, (char)243, (char)118, (char)9, (char)2, (char)89, (char)74, (char)167, (char)212, (char)27, (char)89, (char)91, (char)188, (char)216, (char)4, (char)56, (char)131, (char)221, (char)125, (char)219, (char)178, (char)122, (char)211, (char)152, (char)168, (char)182, (char)217, (char)225, (char)77, (char)85, (char)107, (char)99, (char)246, (char)143, (char)56, (char)64, (char)59, (char)32, (char)70, (char)115, (char)237, (char)254, (char)22, (char)213, (char)157, (char)204, (char)252, (char)176, (char)131, (char)98, (char)181, (char)166, (char)94, (char)134, (char)56, (char)219, (char)113, (char)151, (char)64, (char)153, (char)12, (char)238, (char)178, (char)121, (char)152, (char)23, (char)46, (char)126, (char)156, (char)192, (char)240, (char)11, (char)120, (char)220, (char)237, (char)149, (char)176, (char)160, (char)115, (char)16, (char)220, (char)159, (char)143, (char)253, (char)50, (char)126, (char)112, (char)155, (char)179, (char)44, (char)165, (char)209, (char)205, (char)118, (char)72, (char)1, (char)29, (char)228, (char)5, (char)30, (char)62, (char)207, (char)111, (char)224, (char)7, (char)1, (char)11, (char)75, (char)88, (char)175, (char)45, (char)171, (char)205}, 0) ;
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP) ;
        LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK);
            assert(pack.target_system_GET() == (char)127);
            assert(pack.seqno_GET() == 158082445L);
            assert(pack.target_component_GET() == (char)37);
        });
        DemoDevice.REMOTE_LOG_BLOCK_STATUS p185 = LoopBackDemoChannel.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_system_SET((char)127) ;
        p185.seqno_SET(158082445L) ;
        p185.target_component_SET((char)37) ;
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK) ;
        LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.instance_GET() == (char)179);
            assert(pack.target_component_GET() == (char)14);
            assert(pack.custom_len_GET() == (char)57);
            assert(Arrays.equals(pack.custom_bytes_GET(),  new char[] {(char)110, (char)85, (char)163, (char)157, (char)66, (char)176, (char)158, (char)174, (char)20, (char)104, (char)231, (char)63, (char)17, (char)120, (char)69, (char)141, (char)151, (char)176, (char)225, (char)51, (char)250, (char)206, (char)11, (char)223}));
            assert(pack.pattern_GET() == (char)89);
            assert(pack.target_system_GET() == (char)61);
        });
        DemoDevice.LED_CONTROL p186 = LoopBackDemoChannel.new_LED_CONTROL();
        PH.setPack(p186);
        p186.pattern_SET((char)89) ;
        p186.instance_SET((char)179) ;
        p186.custom_len_SET((char)57) ;
        p186.custom_bytes_SET(new char[] {(char)110, (char)85, (char)163, (char)157, (char)66, (char)176, (char)158, (char)174, (char)20, (char)104, (char)231, (char)63, (char)17, (char)120, (char)69, (char)141, (char)151, (char)176, (char)225, (char)51, (char)250, (char)206, (char)11, (char)223}, 0) ;
        p186.target_component_SET((char)14) ;
        p186.target_system_SET((char)61) ;
        LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.completion_mask_GET(),  new char[] {(char)94, (char)166, (char)212, (char)150, (char)235, (char)208, (char)126, (char)230, (char)95, (char)196}));
            assert(pack.direction_x_GET() == -2.500388E38F);
            assert(pack.attempt_GET() == (char)156);
            assert(pack.completion_pct_GET() == (char)8);
            assert(pack.cal_mask_GET() == (char)162);
            assert(pack.compass_id_GET() == (char)249);
            assert(pack.direction_y_GET() == -2.4351434E38F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_SUCCESS);
            assert(pack.direction_z_GET() == 2.149431E38F);
        });
        DemoDevice.MAG_CAL_PROGRESS p191 = LoopBackDemoChannel.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.direction_x_SET(-2.500388E38F) ;
        p191.compass_id_SET((char)249) ;
        p191.direction_z_SET(2.149431E38F) ;
        p191.completion_pct_SET((char)8) ;
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_SUCCESS) ;
        p191.direction_y_SET(-2.4351434E38F) ;
        p191.cal_mask_SET((char)162) ;
        p191.completion_mask_SET(new char[] {(char)94, (char)166, (char)212, (char)150, (char)235, (char)208, (char)126, (char)230, (char)95, (char)196}, 0) ;
        p191.attempt_SET((char)156) ;
        LoopBackDemoChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.offdiag_y_GET() == -4.7487966E37F);
            assert(pack.offdiag_z_GET() == -3.1462323E38F);
            assert(pack.cal_mask_GET() == (char)97);
            assert(pack.diag_y_GET() == 1.4026793E38F);
            assert(pack.autosaved_GET() == (char)97);
            assert(pack.compass_id_GET() == (char)208);
            assert(pack.diag_x_GET() == -2.825917E38F);
            assert(pack.ofs_z_GET() == -3.1911684E36F);
            assert(pack.fitness_GET() == -3.3982471E38F);
            assert(pack.ofs_y_GET() == -1.6349833E38F);
            assert(pack.diag_z_GET() == 2.2667875E38F);
            assert(pack.ofs_x_GET() == -1.6403211E38F);
            assert(pack.offdiag_x_GET() == -1.7179357E38F);
            assert(pack.cal_status_GET() == MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
        });
        DemoDevice.MAG_CAL_REPORT p192 = LoopBackDemoChannel.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.offdiag_y_SET(-4.7487966E37F) ;
        p192.compass_id_SET((char)208) ;
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_NOT_STARTED) ;
        p192.diag_z_SET(2.2667875E38F) ;
        p192.ofs_z_SET(-3.1911684E36F) ;
        p192.fitness_SET(-3.3982471E38F) ;
        p192.offdiag_x_SET(-1.7179357E38F) ;
        p192.cal_mask_SET((char)97) ;
        p192.diag_y_SET(1.4026793E38F) ;
        p192.ofs_x_SET(-1.6403211E38F) ;
        p192.ofs_y_SET(-1.6349833E38F) ;
        p192.offdiag_z_SET(-3.1462323E38F) ;
        p192.autosaved_SET((char)97) ;
        p192.diag_x_SET(-2.825917E38F) ;
        LoopBackDemoChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == EKF_STATUS_FLAGS.EKF_CONST_POS_MODE);
            assert(pack.velocity_variance_GET() == 2.1990509E38F);
            assert(pack.pos_vert_variance_GET() == -2.2603273E38F);
            assert(pack.terrain_alt_variance_GET() == -2.1532948E38F);
            assert(pack.pos_horiz_variance_GET() == -9.789709E37F);
            assert(pack.compass_variance_GET() == -2.3210508E38F);
        });
        DemoDevice.EKF_STATUS_REPORT p193 = LoopBackDemoChannel.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.pos_horiz_variance_SET(-9.789709E37F) ;
        p193.compass_variance_SET(-2.3210508E38F) ;
        p193.velocity_variance_SET(2.1990509E38F) ;
        p193.pos_vert_variance_SET(-2.2603273E38F) ;
        p193.terrain_alt_variance_SET(-2.1532948E38F) ;
        p193.flags_SET(EKF_STATUS_FLAGS.EKF_CONST_POS_MODE) ;
        LoopBackDemoChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            assert(pack.desired_GET() == 1.9100081E38F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_YAW);
            assert(pack.P_GET() == 2.3704146E37F);
            assert(pack.FF_GET() == 7.460185E37F);
            assert(pack.I_GET() == -1.4837876E37F);
            assert(pack.D_GET() == 1.584811E38F);
            assert(pack.achieved_GET() == -3.3109003E38F);
        });
        DemoDevice.PID_TUNING p194 = LoopBackDemoChannel.new_PID_TUNING();
        PH.setPack(p194);
        p194.FF_SET(7.460185E37F) ;
        p194.I_SET(-1.4837876E37F) ;
        p194.P_SET(2.3704146E37F) ;
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_YAW) ;
        p194.desired_SET(1.9100081E38F) ;
        p194.achieved_SET(-3.3109003E38F) ;
        p194.D_SET(1.584811E38F) ;
        LoopBackDemoChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            assert(pack.delta_angle_x_GET() == -2.1926185E38F);
            assert(pack.joint_az_GET() == 1.6590028E38F);
            assert(pack.target_component_GET() == (char)82);
            assert(pack.delta_velocity_y_GET() == 3.065807E37F);
            assert(pack.joint_roll_GET() == -2.9485415E38F);
            assert(pack.delta_velocity_z_GET() == -6.9107915E37F);
            assert(pack.delta_angle_z_GET() == 2.1349167E38F);
            assert(pack.target_system_GET() == (char)106);
            assert(pack.joint_el_GET() == -3.1039303E38F);
            assert(pack.delta_time_GET() == -6.945963E37F);
            assert(pack.delta_velocity_x_GET() == -1.5933264E38F);
            assert(pack.delta_angle_y_GET() == -1.1902384E38F);
        });
        DemoDevice.GIMBAL_REPORT p200 = LoopBackDemoChannel.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.joint_az_SET(1.6590028E38F) ;
        p200.delta_velocity_y_SET(3.065807E37F) ;
        p200.delta_angle_x_SET(-2.1926185E38F) ;
        p200.target_component_SET((char)82) ;
        p200.delta_angle_y_SET(-1.1902384E38F) ;
        p200.delta_velocity_z_SET(-6.9107915E37F) ;
        p200.delta_angle_z_SET(2.1349167E38F) ;
        p200.target_system_SET((char)106) ;
        p200.delta_velocity_x_SET(-1.5933264E38F) ;
        p200.delta_time_SET(-6.945963E37F) ;
        p200.joint_roll_SET(-2.9485415E38F) ;
        p200.joint_el_SET(-3.1039303E38F) ;
        LoopBackDemoChannel.instance.send(p200);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.demanded_rate_x_GET() == -1.0260685E38F);
            assert(pack.demanded_rate_z_GET() == -2.085687E38F);
            assert(pack.demanded_rate_y_GET() == 3.3682762E37F);
            assert(pack.target_component_GET() == (char)150);
            assert(pack.target_system_GET() == (char)6);
        });
        DemoDevice.GIMBAL_CONTROL p201 = LoopBackDemoChannel.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.demanded_rate_y_SET(3.3682762E37F) ;
        p201.target_component_SET((char)150) ;
        p201.target_system_SET((char)6) ;
        p201.demanded_rate_z_SET(-2.085687E38F) ;
        p201.demanded_rate_x_SET(-1.0260685E38F) ;
        LoopBackDemoChannel.instance.send(p201);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rl_torque_cmd_GET() == (short) -23855);
            assert(pack.target_component_GET() == (char)237);
            assert(pack.target_system_GET() == (char)179);
            assert(pack.el_torque_cmd_GET() == (short)13004);
            assert(pack.az_torque_cmd_GET() == (short) -30258);
        });
        DemoDevice.GIMBAL_TORQUE_CMD_REPORT p214 = LoopBackDemoChannel.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)179) ;
        p214.el_torque_cmd_SET((short)13004) ;
        p214.target_component_SET((char)237) ;
        p214.az_torque_cmd_SET((short) -30258) ;
        p214.rl_torque_cmd_SET((short) -23855) ;
        LoopBackDemoChannel.instance.send(p214);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR);
            assert(pack.capture_mode_GET() == GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP);
            assert(pack.flags_GET() == GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
        });
        DemoDevice.GOPRO_HEARTBEAT p215 = LoopBackDemoChannel.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP) ;
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR) ;
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING) ;
        LoopBackDemoChannel.instance.send(p215);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)18);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_POWER);
        });
        DemoDevice.GOPRO_GET_REQUEST p216 = LoopBackDemoChannel.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_POWER) ;
        p216.target_system_SET((char)18) ;
        p216.target_component_SET((char)220) ;
        LoopBackDemoChannel.instance.send(p216);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)169, (char)85, (char)76, (char)251}));
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_TIME);
        });
        DemoDevice.GOPRO_GET_RESPONSE p217 = LoopBackDemoChannel.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS) ;
        p217.value_SET(new char[] {(char)169, (char)85, (char)76, (char)251}, 0) ;
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_TIME) ;
        LoopBackDemoChannel.instance.send(p217);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)106);
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT);
            assert(Arrays.equals(pack.value_GET(),  new char[] {(char)154, (char)104, (char)160, (char)135}));
            assert(pack.target_component_GET() == (char)242);
        });
        DemoDevice.GOPRO_SET_REQUEST p218 = LoopBackDemoChannel.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.target_system_SET((char)106) ;
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT) ;
        p218.value_SET(new char[] {(char)154, (char)104, (char)160, (char)135}, 0) ;
        p218.target_component_SET((char)242) ;
        LoopBackDemoChannel.instance.send(p218);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            assert(pack.cmd_id_GET() == GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
            assert(pack.status_GET() == GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
        });
        DemoDevice.GOPRO_SET_RESPONSE p219 = LoopBackDemoChannel.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED) ;
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS) ;
        LoopBackDemoChannel.instance.send(p219);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            assert(pack.rpm2_GET() == 3.046875E38F);
            assert(pack.rpm1_GET() == -1.0334444E38F);
        });
        DemoDevice.RPM p226 = LoopBackDemoChannel.new_RPM();
        PH.setPack(p226);
        p226.rpm1_SET(-1.0334444E38F) ;
        p226.rpm2_SET(3.046875E38F) ;
        LoopBackDemoChannel.instance.send(p226);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.tas_ratio_GET() == -3.7306306E37F);
            assert(pack.pos_horiz_ratio_GET() == -1.1259846E38F);
            assert(pack.mag_ratio_GET() == 1.6689328E37F);
            assert(pack.vel_ratio_GET() == 2.5946613E38F);
            assert(pack.time_usec_GET() == 73767023344056786L);
            assert(pack.pos_vert_accuracy_GET() == -2.7197638E38F);
            assert(pack.pos_vert_ratio_GET() == 2.7937634E38F);
            assert(pack.pos_horiz_accuracy_GET() == -2.8576897E38F);
            assert(pack.hagl_ratio_GET() == 1.5811237E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.hagl_ratio_SET(1.5811237E38F) ;
        p230.pos_horiz_accuracy_SET(-2.8576897E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE) ;
        p230.mag_ratio_SET(1.6689328E37F) ;
        p230.pos_vert_ratio_SET(2.7937634E38F) ;
        p230.pos_horiz_ratio_SET(-1.1259846E38F) ;
        p230.pos_vert_accuracy_SET(-2.7197638E38F) ;
        p230.time_usec_SET(73767023344056786L) ;
        p230.tas_ratio_SET(-3.7306306E37F) ;
        p230.vel_ratio_SET(2.5946613E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_x_GET() == 2.443593E38F);
            assert(pack.horiz_accuracy_GET() == 1.2727168E36F);
            assert(pack.wind_z_GET() == -3.3782225E38F);
            assert(pack.var_horiz_GET() == -2.2357144E38F);
            assert(pack.wind_alt_GET() == 1.2071292E38F);
            assert(pack.time_usec_GET() == 3793347228381732994L);
            assert(pack.vert_accuracy_GET() == -1.53756E38F);
            assert(pack.wind_y_GET() == -7.7868105E37F);
            assert(pack.var_vert_GET() == -8.2691724E37F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(-7.7868105E37F) ;
        p231.horiz_accuracy_SET(1.2727168E36F) ;
        p231.var_horiz_SET(-2.2357144E38F) ;
        p231.vert_accuracy_SET(-1.53756E38F) ;
        p231.time_usec_SET(3793347228381732994L) ;
        p231.var_vert_SET(-8.2691724E37F) ;
        p231.wind_alt_SET(1.2071292E38F) ;
        p231.wind_z_SET(-3.3782225E38F) ;
        p231.wind_x_SET(2.443593E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.hdop_GET() == 2.6906561E38F);
            assert(pack.vd_GET() == 9.497964E37F);
            assert(pack.fix_type_GET() == (char)154);
            assert(pack.vdop_GET() == -3.283829E38F);
            assert(pack.vert_accuracy_GET() == -1.7526748E38F);
            assert(pack.alt_GET() == 1.6995948E38F);
            assert(pack.time_week_ms_GET() == 803258133L);
            assert(pack.lat_GET() == -884226697);
            assert(pack.time_usec_GET() == 787621920651939216L);
            assert(pack.lon_GET() == 467540694);
            assert(pack.vn_GET() == 2.1881108E38F);
            assert(pack.ve_GET() == 1.7481504E38F);
            assert(pack.satellites_visible_GET() == (char)133);
            assert(pack.gps_id_GET() == (char)111);
            assert(pack.time_week_GET() == (char)36657);
            assert(pack.horiz_accuracy_GET() == 2.8642332E38F);
            assert(pack.speed_accuracy_GET() == -1.06324796E37F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.hdop_SET(2.6906561E38F) ;
        p232.ve_SET(1.7481504E38F) ;
        p232.alt_SET(1.6995948E38F) ;
        p232.vert_accuracy_SET(-1.7526748E38F) ;
        p232.time_usec_SET(787621920651939216L) ;
        p232.speed_accuracy_SET(-1.06324796E37F) ;
        p232.horiz_accuracy_SET(2.8642332E38F) ;
        p232.satellites_visible_SET((char)133) ;
        p232.vn_SET(2.1881108E38F) ;
        p232.fix_type_SET((char)154) ;
        p232.vd_SET(9.497964E37F) ;
        p232.lon_SET(467540694) ;
        p232.time_week_ms_SET(803258133L) ;
        p232.vdop_SET(-3.283829E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY) ;
        p232.lat_SET(-884226697) ;
        p232.gps_id_SET((char)111) ;
        p232.time_week_SET((char)36657) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)44);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)192, (char)52, (char)105, (char)137, (char)229, (char)60, (char)49, (char)201, (char)45, (char)109, (char)186, (char)230, (char)186, (char)155, (char)24, (char)234, (char)41, (char)112, (char)1, (char)209, (char)193, (char)224, (char)158, (char)119, (char)178, (char)71, (char)217, (char)101, (char)164, (char)188, (char)120, (char)89, (char)189, (char)66, (char)151, (char)30, (char)40, (char)103, (char)216, (char)115, (char)52, (char)183, (char)25, (char)4, (char)122, (char)153, (char)56, (char)78, (char)18, (char)61, (char)159, (char)137, (char)215, (char)56, (char)237, (char)228, (char)136, (char)72, (char)65, (char)49, (char)71, (char)101, (char)47, (char)221, (char)3, (char)33, (char)202, (char)16, (char)154, (char)48, (char)216, (char)149, (char)243, (char)133, (char)158, (char)0, (char)241, (char)82, (char)98, (char)219, (char)169, (char)71, (char)222, (char)43, (char)53, (char)11, (char)234, (char)255, (char)179, (char)175, (char)10, (char)131, (char)64, (char)225, (char)70, (char)241, (char)60, (char)209, (char)44, (char)29, (char)38, (char)71, (char)250, (char)57, (char)251, (char)180, (char)216, (char)54, (char)200, (char)250, (char)194, (char)250, (char)103, (char)71, (char)140, (char)5, (char)116, (char)89, (char)249, (char)114, (char)156, (char)47, (char)88, (char)142, (char)33, (char)26, (char)83, (char)29, (char)175, (char)111, (char)168, (char)42, (char)72, (char)90, (char)83, (char)221, (char)161, (char)165, (char)138, (char)36, (char)29, (char)2, (char)245, (char)179, (char)124, (char)193, (char)158, (char)150, (char)49, (char)131, (char)37, (char)18, (char)172, (char)66, (char)110, (char)69, (char)209, (char)172, (char)254, (char)122, (char)185, (char)36, (char)47, (char)249, (char)202, (char)246, (char)178, (char)205, (char)59, (char)93, (char)62, (char)169, (char)178, (char)157, (char)218, (char)68, (char)60, (char)178, (char)55, (char)121}));
            assert(pack.flags_GET() == (char)136);
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)44) ;
        p233.flags_SET((char)136) ;
        p233.data__SET(new char[] {(char)192, (char)52, (char)105, (char)137, (char)229, (char)60, (char)49, (char)201, (char)45, (char)109, (char)186, (char)230, (char)186, (char)155, (char)24, (char)234, (char)41, (char)112, (char)1, (char)209, (char)193, (char)224, (char)158, (char)119, (char)178, (char)71, (char)217, (char)101, (char)164, (char)188, (char)120, (char)89, (char)189, (char)66, (char)151, (char)30, (char)40, (char)103, (char)216, (char)115, (char)52, (char)183, (char)25, (char)4, (char)122, (char)153, (char)56, (char)78, (char)18, (char)61, (char)159, (char)137, (char)215, (char)56, (char)237, (char)228, (char)136, (char)72, (char)65, (char)49, (char)71, (char)101, (char)47, (char)221, (char)3, (char)33, (char)202, (char)16, (char)154, (char)48, (char)216, (char)149, (char)243, (char)133, (char)158, (char)0, (char)241, (char)82, (char)98, (char)219, (char)169, (char)71, (char)222, (char)43, (char)53, (char)11, (char)234, (char)255, (char)179, (char)175, (char)10, (char)131, (char)64, (char)225, (char)70, (char)241, (char)60, (char)209, (char)44, (char)29, (char)38, (char)71, (char)250, (char)57, (char)251, (char)180, (char)216, (char)54, (char)200, (char)250, (char)194, (char)250, (char)103, (char)71, (char)140, (char)5, (char)116, (char)89, (char)249, (char)114, (char)156, (char)47, (char)88, (char)142, (char)33, (char)26, (char)83, (char)29, (char)175, (char)111, (char)168, (char)42, (char)72, (char)90, (char)83, (char)221, (char)161, (char)165, (char)138, (char)36, (char)29, (char)2, (char)245, (char)179, (char)124, (char)193, (char)158, (char)150, (char)49, (char)131, (char)37, (char)18, (char)172, (char)66, (char)110, (char)69, (char)209, (char)172, (char)254, (char)122, (char)185, (char)36, (char)47, (char)249, (char)202, (char)246, (char)178, (char)205, (char)59, (char)93, (char)62, (char)169, (char)178, (char)157, (char)218, (char)68, (char)60, (char)178, (char)55, (char)121}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.airspeed_sp_GET() == (char)167);
            assert(pack.pitch_GET() == (short)12579);
            assert(pack.battery_remaining_GET() == (char)239);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.altitude_sp_GET() == (short) -29617);
            assert(pack.latitude_GET() == -1140807590);
            assert(pack.roll_GET() == (short) -14508);
            assert(pack.longitude_GET() == 653107510);
            assert(pack.altitude_amsl_GET() == (short)24402);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            assert(pack.gps_nsat_GET() == (char)187);
            assert(pack.temperature_GET() == (byte)101);
            assert(pack.airspeed_GET() == (char)78);
            assert(pack.throttle_GET() == (byte) - 89);
            assert(pack.groundspeed_GET() == (char)2);
            assert(pack.custom_mode_GET() == 3760373769L);
            assert(pack.climb_rate_GET() == (byte)92);
            assert(pack.wp_num_GET() == (char)38);
            assert(pack.heading_sp_GET() == (short) -24976);
            assert(pack.failsafe_GET() == (char)96);
            assert(pack.heading_GET() == (char)55187);
            assert(pack.wp_distance_GET() == (char)47111);
            assert(pack.temperature_air_GET() == (byte) - 99);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.airspeed_sp_SET((char)167) ;
        p234.temperature_air_SET((byte) - 99) ;
        p234.altitude_sp_SET((short) -29617) ;
        p234.throttle_SET((byte) - 89) ;
        p234.failsafe_SET((char)96) ;
        p234.gps_nsat_SET((char)187) ;
        p234.latitude_SET(-1140807590) ;
        p234.wp_num_SET((char)38) ;
        p234.airspeed_SET((char)78) ;
        p234.wp_distance_SET((char)47111) ;
        p234.climb_rate_SET((byte)92) ;
        p234.temperature_SET((byte)101) ;
        p234.altitude_amsl_SET((short)24402) ;
        p234.longitude_SET(653107510) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p234.battery_remaining_SET((char)239) ;
        p234.pitch_SET((short)12579) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.roll_SET((short) -14508) ;
        p234.heading_sp_SET((short) -24976) ;
        p234.custom_mode_SET(3760373769L) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) ;
        p234.groundspeed_SET((char)2) ;
        p234.heading_SET((char)55187) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.clipping_0_GET() == 763080038L);
            assert(pack.vibration_y_GET() == -1.4485731E37F);
            assert(pack.vibration_z_GET() == 9.815512E37F);
            assert(pack.vibration_x_GET() == -3.2418568E38F);
            assert(pack.clipping_2_GET() == 1238830338L);
            assert(pack.time_usec_GET() == 2478520917019159421L);
            assert(pack.clipping_1_GET() == 135575451L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_z_SET(9.815512E37F) ;
        p241.time_usec_SET(2478520917019159421L) ;
        p241.vibration_y_SET(-1.4485731E37F) ;
        p241.clipping_0_SET(763080038L) ;
        p241.clipping_1_SET(135575451L) ;
        p241.vibration_x_SET(-3.2418568E38F) ;
        p241.clipping_2_SET(1238830338L) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_z_GET() == -1.0960226E38F);
            assert(pack.y_GET() == 1.40387E38F);
            assert(pack.time_usec_TRY(ph) == 4386992922912744317L);
            assert(pack.approach_x_GET() == 8.587014E37F);
            assert(pack.longitude_GET() == 640733416);
            assert(pack.z_GET() == -6.787337E37F);
            assert(pack.altitude_GET() == 903827819);
            assert(pack.latitude_GET() == -674844750);
            assert(pack.approach_y_GET() == -3.0867458E38F);
            assert(pack.x_GET() == -1.3504698E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.1403048E37F, 2.866526E38F, 4.380997E37F, -4.1006704E37F}));
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_z_SET(-1.0960226E38F) ;
        p242.y_SET(1.40387E38F) ;
        p242.time_usec_SET(4386992922912744317L, PH) ;
        p242.z_SET(-6.787337E37F) ;
        p242.latitude_SET(-674844750) ;
        p242.q_SET(new float[] {4.1403048E37F, 2.866526E38F, 4.380997E37F, -4.1006704E37F}, 0) ;
        p242.longitude_SET(640733416) ;
        p242.approach_x_SET(8.587014E37F) ;
        p242.approach_y_SET(-3.0867458E38F) ;
        p242.x_SET(-1.3504698E38F) ;
        p242.altitude_SET(903827819) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.2828531E38F, -9.914177E37F, 1.5282855E38F, -3.1367848E38F}));
            assert(pack.time_usec_TRY(ph) == 4163707133048643496L);
            assert(pack.longitude_GET() == -1636499486);
            assert(pack.target_system_GET() == (char)43);
            assert(pack.z_GET() == -1.4539613E37F);
            assert(pack.approach_y_GET() == 2.3844634E38F);
            assert(pack.latitude_GET() == -154325811);
            assert(pack.y_GET() == -3.0163194E38F);
            assert(pack.approach_z_GET() == 1.2068154E38F);
            assert(pack.x_GET() == 1.217432E38F);
            assert(pack.approach_x_GET() == 1.27787E38F);
            assert(pack.altitude_GET() == 1099718928);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.y_SET(-3.0163194E38F) ;
        p243.latitude_SET(-154325811) ;
        p243.target_system_SET((char)43) ;
        p243.x_SET(1.217432E38F) ;
        p243.time_usec_SET(4163707133048643496L, PH) ;
        p243.q_SET(new float[] {-2.2828531E38F, -9.914177E37F, 1.5282855E38F, -3.1367848E38F}, 0) ;
        p243.approach_y_SET(2.3844634E38F) ;
        p243.z_SET(-1.4539613E37F) ;
        p243.approach_z_SET(1.2068154E38F) ;
        p243.altitude_SET(1099718928) ;
        p243.approach_x_SET(1.27787E38F) ;
        p243.longitude_SET(-1636499486) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -1401622665);
            assert(pack.message_id_GET() == (char)26249);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1401622665) ;
        p244.message_id_SET((char)26249) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.ver_velocity_GET() == (short)7594);
            assert(pack.squawk_GET() == (char)10935);
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("flwoLdylw"));
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.hor_velocity_GET() == (char)25450);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED);
            assert(pack.ICAO_address_GET() == 3205292585L);
            assert(pack.altitude_GET() == -908678498);
            assert(pack.lat_GET() == 1269984130);
            assert(pack.tslc_GET() == (char)186);
            assert(pack.heading_GET() == (char)12553);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            assert(pack.lon_GET() == 2032218442);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ver_velocity_SET((short)7594) ;
        p246.altitude_SET(-908678498) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED) ;
        p246.heading_SET((char)12553) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.squawk_SET((char)10935) ;
        p246.callsign_SET("flwoLdylw", PH) ;
        p246.tslc_SET((char)186) ;
        p246.hor_velocity_SET((char)25450) ;
        p246.ICAO_address_SET(3205292585L) ;
        p246.lon_SET(2032218442) ;
        p246.lat_SET(1269984130) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.time_to_minimum_delta_GET() == 9.003138E37F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            assert(pack.id_GET() == 1710182132L);
            assert(pack.horizontal_minimum_delta_GET() == 3.2287546E38F);
            assert(pack.altitude_minimum_delta_GET() == 2.022122E38F);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(2.022122E38F) ;
        p247.time_to_minimum_delta_SET(9.003138E37F) ;
        p247.id_SET(1710182132L) ;
        p247.horizontal_minimum_delta_SET(3.2287546E38F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)62, (char)150, (char)139, (char)167, (char)246, (char)178, (char)226, (char)127, (char)77, (char)59, (char)29, (char)3, (char)224, (char)31, (char)95, (char)226, (char)199, (char)10, (char)16, (char)193, (char)62, (char)214, (char)150, (char)201, (char)110, (char)190, (char)231, (char)144, (char)233, (char)49, (char)209, (char)250, (char)152, (char)252, (char)6, (char)235, (char)186, (char)16, (char)80, (char)254, (char)55, (char)255, (char)113, (char)217, (char)8, (char)169, (char)107, (char)159, (char)247, (char)84, (char)4, (char)195, (char)166, (char)50, (char)15, (char)203, (char)23, (char)147, (char)139, (char)20, (char)178, (char)78, (char)222, (char)52, (char)41, (char)127, (char)190, (char)206, (char)13, (char)205, (char)244, (char)238, (char)185, (char)216, (char)185, (char)247, (char)28, (char)251, (char)69, (char)143, (char)102, (char)47, (char)85, (char)100, (char)15, (char)255, (char)227, (char)92, (char)16, (char)182, (char)125, (char)232, (char)190, (char)250, (char)214, (char)205, (char)25, (char)130, (char)145, (char)206, (char)100, (char)137, (char)137, (char)201, (char)104, (char)128, (char)94, (char)63, (char)174, (char)60, (char)217, (char)60, (char)187, (char)182, (char)118, (char)4, (char)51, (char)74, (char)255, (char)8, (char)89, (char)65, (char)210, (char)114, (char)239, (char)193, (char)137, (char)166, (char)97, (char)164, (char)148, (char)203, (char)82, (char)254, (char)55, (char)254, (char)165, (char)24, (char)37, (char)176, (char)185, (char)201, (char)117, (char)252, (char)49, (char)57, (char)9, (char)50, (char)74, (char)110, (char)167, (char)122, (char)188, (char)119, (char)184, (char)35, (char)41, (char)143, (char)197, (char)189, (char)68, (char)221, (char)108, (char)131, (char)252, (char)223, (char)0, (char)214, (char)133, (char)177, (char)248, (char)60, (char)182, (char)180, (char)186, (char)105, (char)38, (char)130, (char)144, (char)66, (char)198, (char)105, (char)190, (char)13, (char)225, (char)15, (char)121, (char)122, (char)58, (char)65, (char)195, (char)144, (char)190, (char)76, (char)17, (char)196, (char)204, (char)102, (char)33, (char)82, (char)49, (char)13, (char)60, (char)235, (char)86, (char)237, (char)183, (char)255, (char)157, (char)33, (char)98, (char)140, (char)186, (char)91, (char)34, (char)3, (char)161, (char)103, (char)173, (char)194, (char)33, (char)54, (char)138, (char)38, (char)63, (char)74, (char)194, (char)39, (char)39, (char)133, (char)30, (char)161, (char)239, (char)118, (char)130, (char)213, (char)199, (char)176, (char)209, (char)253, (char)98, (char)47, (char)192, (char)100, (char)55, (char)157, (char)2, (char)238, (char)110}));
            assert(pack.target_system_GET() == (char)4);
            assert(pack.message_type_GET() == (char)49323);
            assert(pack.target_network_GET() == (char)105);
            assert(pack.target_component_GET() == (char)197);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.message_type_SET((char)49323) ;
        p248.target_network_SET((char)105) ;
        p248.target_component_SET((char)197) ;
        p248.target_system_SET((char)4) ;
        p248.payload_SET(new char[] {(char)62, (char)150, (char)139, (char)167, (char)246, (char)178, (char)226, (char)127, (char)77, (char)59, (char)29, (char)3, (char)224, (char)31, (char)95, (char)226, (char)199, (char)10, (char)16, (char)193, (char)62, (char)214, (char)150, (char)201, (char)110, (char)190, (char)231, (char)144, (char)233, (char)49, (char)209, (char)250, (char)152, (char)252, (char)6, (char)235, (char)186, (char)16, (char)80, (char)254, (char)55, (char)255, (char)113, (char)217, (char)8, (char)169, (char)107, (char)159, (char)247, (char)84, (char)4, (char)195, (char)166, (char)50, (char)15, (char)203, (char)23, (char)147, (char)139, (char)20, (char)178, (char)78, (char)222, (char)52, (char)41, (char)127, (char)190, (char)206, (char)13, (char)205, (char)244, (char)238, (char)185, (char)216, (char)185, (char)247, (char)28, (char)251, (char)69, (char)143, (char)102, (char)47, (char)85, (char)100, (char)15, (char)255, (char)227, (char)92, (char)16, (char)182, (char)125, (char)232, (char)190, (char)250, (char)214, (char)205, (char)25, (char)130, (char)145, (char)206, (char)100, (char)137, (char)137, (char)201, (char)104, (char)128, (char)94, (char)63, (char)174, (char)60, (char)217, (char)60, (char)187, (char)182, (char)118, (char)4, (char)51, (char)74, (char)255, (char)8, (char)89, (char)65, (char)210, (char)114, (char)239, (char)193, (char)137, (char)166, (char)97, (char)164, (char)148, (char)203, (char)82, (char)254, (char)55, (char)254, (char)165, (char)24, (char)37, (char)176, (char)185, (char)201, (char)117, (char)252, (char)49, (char)57, (char)9, (char)50, (char)74, (char)110, (char)167, (char)122, (char)188, (char)119, (char)184, (char)35, (char)41, (char)143, (char)197, (char)189, (char)68, (char)221, (char)108, (char)131, (char)252, (char)223, (char)0, (char)214, (char)133, (char)177, (char)248, (char)60, (char)182, (char)180, (char)186, (char)105, (char)38, (char)130, (char)144, (char)66, (char)198, (char)105, (char)190, (char)13, (char)225, (char)15, (char)121, (char)122, (char)58, (char)65, (char)195, (char)144, (char)190, (char)76, (char)17, (char)196, (char)204, (char)102, (char)33, (char)82, (char)49, (char)13, (char)60, (char)235, (char)86, (char)237, (char)183, (char)255, (char)157, (char)33, (char)98, (char)140, (char)186, (char)91, (char)34, (char)3, (char)161, (char)103, (char)173, (char)194, (char)33, (char)54, (char)138, (char)38, (char)63, (char)74, (char)194, (char)39, (char)39, (char)133, (char)30, (char)161, (char)239, (char)118, (char)130, (char)213, (char)199, (char)176, (char)209, (char)253, (char)98, (char)47, (char)192, (char)100, (char)55, (char)157, (char)2, (char)238, (char)110}, 0) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)100, (byte)92, (byte)55, (byte) - 40, (byte) - 96, (byte)53, (byte) - 68, (byte)100, (byte)19, (byte) - 120, (byte) - 114, (byte)41, (byte)79, (byte) - 39, (byte) - 70, (byte)44, (byte) - 101, (byte) - 49, (byte)3, (byte) - 92, (byte)3, (byte)119, (byte) - 21, (byte)75, (byte) - 35, (byte)101, (byte) - 107, (byte)123, (byte) - 82, (byte) - 60, (byte)74, (byte) - 8}));
            assert(pack.ver_GET() == (char)194);
            assert(pack.type_GET() == (char)29);
            assert(pack.address_GET() == (char)12431);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)12431) ;
        p249.value_SET(new byte[] {(byte)100, (byte)92, (byte)55, (byte) - 40, (byte) - 96, (byte)53, (byte) - 68, (byte)100, (byte)19, (byte) - 120, (byte) - 114, (byte)41, (byte)79, (byte) - 39, (byte) - 70, (byte)44, (byte) - 101, (byte) - 49, (byte)3, (byte) - 92, (byte)3, (byte)119, (byte) - 21, (byte)75, (byte) - 35, (byte)101, (byte) - 107, (byte)123, (byte) - 82, (byte) - 60, (byte)74, (byte) - 8}, 0) ;
        p249.ver_SET((char)194) ;
        p249.type_SET((char)29) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 9.768486E37F);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("bhjgZk"));
            assert(pack.time_usec_GET() == 3976236555133052533L);
            assert(pack.z_GET() == -1.9222118E38F);
            assert(pack.y_GET() == -1.8362085E38F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(-1.8362085E38F) ;
        p250.x_SET(9.768486E37F) ;
        p250.time_usec_SET(3976236555133052533L) ;
        p250.name_SET("bhjgZk", PH) ;
        p250.z_SET(-1.9222118E38F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -1.9566026E37F);
            assert(pack.time_boot_ms_GET() == 718668351L);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("xn"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.value_SET(-1.9566026E37F) ;
        p251.name_SET("xn", PH) ;
        p251.time_boot_ms_SET(718668351L) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 3);
            assert(pack.name_TRY(ph).equals("pRa"));
            assert(pack.time_boot_ms_GET() == 1391039714L);
            assert(pack.value_GET() == -1528743356);
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-1528743356) ;
        p252.name_SET("pRa", PH) ;
        p252.time_boot_ms_SET(1391039714L) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            assert(pack.text_LEN(ph) == 30);
            assert(pack.text_TRY(ph).equals("omxaduzwcWrrkjnacWujreagcohwao"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        p253.text_SET("omxaduzwcWrrkjnacWujreagcohwao", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 1.8957656E38F);
            assert(pack.ind_GET() == (char)141);
            assert(pack.time_boot_ms_GET() == 1995758890L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)141) ;
        p254.value_SET(1.8957656E38F) ;
        p254.time_boot_ms_SET(1995758890L) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)109, (char)101, (char)34, (char)187, (char)43, (char)43, (char)5, (char)218, (char)145, (char)229, (char)74, (char)27, (char)109, (char)126, (char)149, (char)95, (char)71, (char)46, (char)12, (char)10, (char)110, (char)133, (char)147, (char)209, (char)225, (char)50, (char)143, (char)29, (char)26, (char)143, (char)211, (char)43}));
            assert(pack.target_system_GET() == (char)137);
            assert(pack.initial_timestamp_GET() == 3225762206328996859L);
            assert(pack.target_component_GET() == (char)236);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.secret_key_SET(new char[] {(char)109, (char)101, (char)34, (char)187, (char)43, (char)43, (char)5, (char)218, (char)145, (char)229, (char)74, (char)27, (char)109, (char)126, (char)149, (char)95, (char)71, (char)46, (char)12, (char)10, (char)110, (char)133, (char)147, (char)209, (char)225, (char)50, (char)143, (char)29, (char)26, (char)143, (char)211, (char)43}, 0) ;
        p256.target_system_SET((char)137) ;
        p256.initial_timestamp_SET(3225762206328996859L) ;
        p256.target_component_SET((char)236) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3285726311L);
            assert(pack.last_change_ms_GET() == 1860940453L);
            assert(pack.state_GET() == (char)132);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)132) ;
        p257.time_boot_ms_SET(3285726311L) ;
        p257.last_change_ms_SET(1860940453L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)30);
            assert(pack.tune_LEN(ph) == 6);
            assert(pack.tune_TRY(ph).equals("duetpl"));
            assert(pack.target_component_GET() == (char)197);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)197) ;
        p258.target_system_SET((char)30) ;
        p258.tune_SET("duetpl", PH) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)820);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)145, (char)138, (char)104, (char)28, (char)26, (char)78, (char)140, (char)196, (char)232, (char)79, (char)15, (char)75, (char)168, (char)17, (char)92, (char)58, (char)193, (char)34, (char)62, (char)169, (char)250, (char)176, (char)122, (char)253, (char)168, (char)183, (char)209, (char)33, (char)219, (char)30, (char)206, (char)137}));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)225, (char)211, (char)109, (char)186, (char)249, (char)140, (char)165, (char)193, (char)140, (char)42, (char)153, (char)24, (char)69, (char)8, (char)137, (char)120, (char)95, (char)222, (char)9, (char)0, (char)177, (char)93, (char)195, (char)239, (char)119, (char)183, (char)208, (char)228, (char)123, (char)114, (char)99, (char)75}));
            assert(pack.time_boot_ms_GET() == 539104044L);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            assert(pack.lens_id_GET() == (char)102);
            assert(pack.sensor_size_h_GET() == 9.864738E37F);
            assert(pack.focal_length_GET() == 2.404806E38F);
            assert(pack.cam_definition_version_GET() == (char)12265);
            assert(pack.sensor_size_v_GET() == -2.1587944E38F);
            assert(pack.cam_definition_uri_LEN(ph) == 16);
            assert(pack.cam_definition_uri_TRY(ph).equals("swfFncjyxanbyvig"));
            assert(pack.firmware_version_GET() == 597196756L);
            assert(pack.resolution_v_GET() == (char)7367);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(539104044L) ;
        p259.resolution_h_SET((char)820) ;
        p259.cam_definition_version_SET((char)12265) ;
        p259.vendor_name_SET(new char[] {(char)145, (char)138, (char)104, (char)28, (char)26, (char)78, (char)140, (char)196, (char)232, (char)79, (char)15, (char)75, (char)168, (char)17, (char)92, (char)58, (char)193, (char)34, (char)62, (char)169, (char)250, (char)176, (char)122, (char)253, (char)168, (char)183, (char)209, (char)33, (char)219, (char)30, (char)206, (char)137}, 0) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO) ;
        p259.focal_length_SET(2.404806E38F) ;
        p259.sensor_size_h_SET(9.864738E37F) ;
        p259.cam_definition_uri_SET("swfFncjyxanbyvig", PH) ;
        p259.sensor_size_v_SET(-2.1587944E38F) ;
        p259.model_name_SET(new char[] {(char)225, (char)211, (char)109, (char)186, (char)249, (char)140, (char)165, (char)193, (char)140, (char)42, (char)153, (char)24, (char)69, (char)8, (char)137, (char)120, (char)95, (char)222, (char)9, (char)0, (char)177, (char)93, (char)195, (char)239, (char)119, (char)183, (char)208, (char)228, (char)123, (char)114, (char)99, (char)75}, 0) ;
        p259.firmware_version_SET(597196756L) ;
        p259.resolution_v_SET((char)7367) ;
        p259.lens_id_SET((char)102) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2632004986L);
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2632004986L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == 2.0613329E37F);
            assert(pack.status_GET() == (char)53);
            assert(pack.write_speed_GET() == -3.045518E37F);
            assert(pack.available_capacity_GET() == -2.8152938E38F);
            assert(pack.read_speed_GET() == 2.9524457E38F);
            assert(pack.storage_count_GET() == (char)19);
            assert(pack.time_boot_ms_GET() == 2697084769L);
            assert(pack.storage_id_GET() == (char)149);
            assert(pack.used_capacity_GET() == -3.1943138E38F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.available_capacity_SET(-2.8152938E38F) ;
        p261.storage_count_SET((char)19) ;
        p261.used_capacity_SET(-3.1943138E38F) ;
        p261.status_SET((char)53) ;
        p261.write_speed_SET(-3.045518E37F) ;
        p261.storage_id_SET((char)149) ;
        p261.time_boot_ms_SET(2697084769L) ;
        p261.read_speed_SET(2.9524457E38F) ;
        p261.total_capacity_SET(2.0613329E37F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.recording_time_ms_GET() == 1511572233L);
            assert(pack.image_interval_GET() == -8.798672E37F);
            assert(pack.time_boot_ms_GET() == 3943550367L);
            assert(pack.available_capacity_GET() == -3.226683E38F);
            assert(pack.image_status_GET() == (char)159);
            assert(pack.video_status_GET() == (char)229);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.available_capacity_SET(-3.226683E38F) ;
        p262.image_interval_SET(-8.798672E37F) ;
        p262.time_boot_ms_SET(3943550367L) ;
        p262.image_status_SET((char)159) ;
        p262.recording_time_ms_SET(1511572233L) ;
        p262.video_status_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.relative_alt_GET() == -1724161191);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.3641402E38F, 2.6925708E38F, -2.7304766E38F, -1.7656075E38F}));
            assert(pack.image_index_GET() == 862672324);
            assert(pack.camera_id_GET() == (char)18);
            assert(pack.lon_GET() == 1155017593);
            assert(pack.time_utc_GET() == 5035093043636263778L);
            assert(pack.capture_result_GET() == (byte)71);
            assert(pack.alt_GET() == 1117296880);
            assert(pack.lat_GET() == 606472780);
            assert(pack.file_url_LEN(ph) == 196);
            assert(pack.file_url_TRY(ph).equals("ArhajdyjfwzjlwJcFrbbzjxskwBrjxeKZBVenpewekjmpiuDupprDfsevdpwfcbcstobqWwymciFcdxvflrAPpAzqsgvizfbvjegsdtguuLHujmrtignudxczfbsxzusAjxgIhmufbuyttmqymiilujwlhsxyfymdriivylpyvpcUemfwybKyfplldjfwnZinela"));
            assert(pack.time_boot_ms_GET() == 4264165950L);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.relative_alt_SET(-1724161191) ;
        p263.alt_SET(1117296880) ;
        p263.time_boot_ms_SET(4264165950L) ;
        p263.camera_id_SET((char)18) ;
        p263.file_url_SET("ArhajdyjfwzjlwJcFrbbzjxskwBrjxeKZBVenpewekjmpiuDupprDfsevdpwfcbcstobqWwymciFcdxvflrAPpAzqsgvizfbvjegsdtguuLHujmrtignudxczfbsxzusAjxgIhmufbuyttmqymiilujwlhsxyfymdriivylpyvpcUemfwybKyfplldjfwnZinela", PH) ;
        p263.q_SET(new float[] {3.3641402E38F, 2.6925708E38F, -2.7304766E38F, -1.7656075E38F}, 0) ;
        p263.lon_SET(1155017593) ;
        p263.lat_SET(606472780) ;
        p263.capture_result_SET((byte)71) ;
        p263.image_index_SET(862672324) ;
        p263.time_utc_SET(5035093043636263778L) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 8905813195973187809L);
            assert(pack.time_boot_ms_GET() == 377397931L);
            assert(pack.flight_uuid_GET() == 5134986918228501965L);
            assert(pack.arming_time_utc_GET() == 880565540406297445L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(8905813195973187809L) ;
        p264.arming_time_utc_SET(880565540406297445L) ;
        p264.time_boot_ms_SET(377397931L) ;
        p264.flight_uuid_SET(5134986918228501965L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.0025927E38F);
            assert(pack.time_boot_ms_GET() == 1008825130L);
            assert(pack.roll_GET() == 1.631655E38F);
            assert(pack.yaw_GET() == 2.0169201E38F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(2.0169201E38F) ;
        p265.time_boot_ms_SET(1008825130L) ;
        p265.roll_SET(1.631655E38F) ;
        p265.pitch_SET(-1.0025927E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)217);
            assert(pack.length_GET() == (char)190);
            assert(pack.target_component_GET() == (char)251);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)158, (char)119, (char)214, (char)55, (char)66, (char)26, (char)218, (char)158, (char)244, (char)254, (char)94, (char)132, (char)61, (char)170, (char)13, (char)181, (char)62, (char)130, (char)200, (char)221, (char)75, (char)100, (char)246, (char)111, (char)160, (char)66, (char)217, (char)115, (char)20, (char)144, (char)1, (char)184, (char)91, (char)211, (char)148, (char)194, (char)251, (char)245, (char)206, (char)36, (char)225, (char)52, (char)161, (char)109, (char)6, (char)232, (char)146, (char)96, (char)18, (char)49, (char)236, (char)95, (char)175, (char)81, (char)107, (char)110, (char)103, (char)49, (char)135, (char)64, (char)81, (char)250, (char)241, (char)44, (char)160, (char)62, (char)155, (char)24, (char)192, (char)38, (char)174, (char)142, (char)194, (char)238, (char)114, (char)230, (char)26, (char)12, (char)28, (char)74, (char)153, (char)140, (char)117, (char)163, (char)162, (char)132, (char)37, (char)247, (char)93, (char)96, (char)105, (char)222, (char)44, (char)67, (char)247, (char)140, (char)212, (char)150, (char)151, (char)209, (char)85, (char)163, (char)227, (char)231, (char)90, (char)51, (char)185, (char)15, (char)121, (char)105, (char)222, (char)250, (char)56, (char)194, (char)8, (char)184, (char)205, (char)154, (char)182, (char)25, (char)218, (char)185, (char)222, (char)63, (char)144, (char)116, (char)89, (char)57, (char)172, (char)155, (char)211, (char)213, (char)27, (char)232, (char)185, (char)113, (char)200, (char)100, (char)176, (char)22, (char)135, (char)120, (char)204, (char)116, (char)68, (char)39, (char)253, (char)109, (char)124, (char)208, (char)129, (char)200, (char)101, (char)86, (char)18, (char)12, (char)30, (char)64, (char)73, (char)77, (char)152, (char)215, (char)227, (char)183, (char)47, (char)38, (char)144, (char)80, (char)29, (char)135, (char)67, (char)180, (char)30, (char)30, (char)162, (char)220, (char)201, (char)216, (char)250, (char)231, (char)220, (char)41, (char)199, (char)235, (char)209, (char)162, (char)35, (char)9, (char)12, (char)64, (char)221, (char)36, (char)12, (char)92, (char)13, (char)219, (char)155, (char)178, (char)100, (char)222, (char)162, (char)77, (char)182, (char)111, (char)194, (char)196, (char)50, (char)60, (char)92, (char)100, (char)83, (char)158, (char)242, (char)161, (char)168, (char)74, (char)37, (char)130, (char)153, (char)156, (char)138, (char)101, (char)201, (char)144, (char)154, (char)73, (char)176, (char)91, (char)243, (char)132, (char)107, (char)82, (char)223, (char)210, (char)180, (char)48, (char)170, (char)104, (char)107, (char)199, (char)6, (char)192, (char)61, (char)53, (char)26, (char)8, (char)6, (char)147, (char)222}));
            assert(pack.sequence_GET() == (char)27928);
            assert(pack.first_message_offset_GET() == (char)192);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.length_SET((char)190) ;
        p266.target_system_SET((char)217) ;
        p266.first_message_offset_SET((char)192) ;
        p266.sequence_SET((char)27928) ;
        p266.target_component_SET((char)251) ;
        p266.data__SET(new char[] {(char)158, (char)119, (char)214, (char)55, (char)66, (char)26, (char)218, (char)158, (char)244, (char)254, (char)94, (char)132, (char)61, (char)170, (char)13, (char)181, (char)62, (char)130, (char)200, (char)221, (char)75, (char)100, (char)246, (char)111, (char)160, (char)66, (char)217, (char)115, (char)20, (char)144, (char)1, (char)184, (char)91, (char)211, (char)148, (char)194, (char)251, (char)245, (char)206, (char)36, (char)225, (char)52, (char)161, (char)109, (char)6, (char)232, (char)146, (char)96, (char)18, (char)49, (char)236, (char)95, (char)175, (char)81, (char)107, (char)110, (char)103, (char)49, (char)135, (char)64, (char)81, (char)250, (char)241, (char)44, (char)160, (char)62, (char)155, (char)24, (char)192, (char)38, (char)174, (char)142, (char)194, (char)238, (char)114, (char)230, (char)26, (char)12, (char)28, (char)74, (char)153, (char)140, (char)117, (char)163, (char)162, (char)132, (char)37, (char)247, (char)93, (char)96, (char)105, (char)222, (char)44, (char)67, (char)247, (char)140, (char)212, (char)150, (char)151, (char)209, (char)85, (char)163, (char)227, (char)231, (char)90, (char)51, (char)185, (char)15, (char)121, (char)105, (char)222, (char)250, (char)56, (char)194, (char)8, (char)184, (char)205, (char)154, (char)182, (char)25, (char)218, (char)185, (char)222, (char)63, (char)144, (char)116, (char)89, (char)57, (char)172, (char)155, (char)211, (char)213, (char)27, (char)232, (char)185, (char)113, (char)200, (char)100, (char)176, (char)22, (char)135, (char)120, (char)204, (char)116, (char)68, (char)39, (char)253, (char)109, (char)124, (char)208, (char)129, (char)200, (char)101, (char)86, (char)18, (char)12, (char)30, (char)64, (char)73, (char)77, (char)152, (char)215, (char)227, (char)183, (char)47, (char)38, (char)144, (char)80, (char)29, (char)135, (char)67, (char)180, (char)30, (char)30, (char)162, (char)220, (char)201, (char)216, (char)250, (char)231, (char)220, (char)41, (char)199, (char)235, (char)209, (char)162, (char)35, (char)9, (char)12, (char)64, (char)221, (char)36, (char)12, (char)92, (char)13, (char)219, (char)155, (char)178, (char)100, (char)222, (char)162, (char)77, (char)182, (char)111, (char)194, (char)196, (char)50, (char)60, (char)92, (char)100, (char)83, (char)158, (char)242, (char)161, (char)168, (char)74, (char)37, (char)130, (char)153, (char)156, (char)138, (char)101, (char)201, (char)144, (char)154, (char)73, (char)176, (char)91, (char)243, (char)132, (char)107, (char)82, (char)223, (char)210, (char)180, (char)48, (char)170, (char)104, (char)107, (char)199, (char)6, (char)192, (char)61, (char)53, (char)26, (char)8, (char)6, (char)147, (char)222}, 0) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)66);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)53, (char)189, (char)180, (char)99, (char)76, (char)164, (char)93, (char)41, (char)135, (char)213, (char)242, (char)39, (char)81, (char)77, (char)83, (char)80, (char)92, (char)60, (char)245, (char)35, (char)16, (char)99, (char)160, (char)244, (char)88, (char)19, (char)220, (char)253, (char)202, (char)54, (char)99, (char)57, (char)195, (char)127, (char)111, (char)76, (char)19, (char)20, (char)122, (char)84, (char)33, (char)243, (char)54, (char)12, (char)2, (char)170, (char)53, (char)228, (char)130, (char)86, (char)152, (char)248, (char)11, (char)230, (char)85, (char)241, (char)85, (char)20, (char)243, (char)245, (char)252, (char)121, (char)233, (char)119, (char)146, (char)17, (char)157, (char)249, (char)76, (char)126, (char)120, (char)215, (char)145, (char)205, (char)186, (char)33, (char)86, (char)228, (char)249, (char)238, (char)212, (char)50, (char)216, (char)15, (char)183, (char)26, (char)131, (char)134, (char)104, (char)173, (char)228, (char)220, (char)251, (char)98, (char)219, (char)132, (char)140, (char)118, (char)225, (char)159, (char)99, (char)30, (char)42, (char)126, (char)136, (char)196, (char)155, (char)55, (char)121, (char)226, (char)216, (char)166, (char)240, (char)251, (char)187, (char)209, (char)43, (char)234, (char)40, (char)121, (char)16, (char)193, (char)241, (char)87, (char)141, (char)88, (char)222, (char)29, (char)70, (char)225, (char)94, (char)210, (char)147, (char)248, (char)230, (char)9, (char)163, (char)209, (char)78, (char)128, (char)241, (char)112, (char)90, (char)10, (char)27, (char)19, (char)11, (char)206, (char)218, (char)167, (char)219, (char)63, (char)151, (char)181, (char)17, (char)34, (char)7, (char)62, (char)107, (char)156, (char)14, (char)16, (char)141, (char)229, (char)164, (char)2, (char)102, (char)248, (char)215, (char)16, (char)187, (char)38, (char)241, (char)155, (char)124, (char)39, (char)225, (char)247, (char)41, (char)171, (char)214, (char)150, (char)116, (char)170, (char)176, (char)39, (char)133, (char)2, (char)246, (char)138, (char)189, (char)12, (char)23, (char)86, (char)249, (char)214, (char)60, (char)146, (char)108, (char)212, (char)195, (char)157, (char)6, (char)61, (char)48, (char)187, (char)110, (char)155, (char)106, (char)225, (char)110, (char)133, (char)19, (char)198, (char)220, (char)191, (char)239, (char)6, (char)220, (char)173, (char)241, (char)222, (char)213, (char)16, (char)249, (char)5, (char)247, (char)48, (char)195, (char)23, (char)122, (char)157, (char)160, (char)60, (char)229, (char)212, (char)134, (char)9, (char)182, (char)230, (char)158, (char)187, (char)2, (char)239, (char)247, (char)229, (char)248, (char)216, (char)14}));
            assert(pack.target_component_GET() == (char)58);
            assert(pack.first_message_offset_GET() == (char)81);
            assert(pack.sequence_GET() == (char)34860);
            assert(pack.length_GET() == (char)163);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.sequence_SET((char)34860) ;
        p267.target_component_SET((char)58) ;
        p267.length_SET((char)163) ;
        p267.data__SET(new char[] {(char)53, (char)189, (char)180, (char)99, (char)76, (char)164, (char)93, (char)41, (char)135, (char)213, (char)242, (char)39, (char)81, (char)77, (char)83, (char)80, (char)92, (char)60, (char)245, (char)35, (char)16, (char)99, (char)160, (char)244, (char)88, (char)19, (char)220, (char)253, (char)202, (char)54, (char)99, (char)57, (char)195, (char)127, (char)111, (char)76, (char)19, (char)20, (char)122, (char)84, (char)33, (char)243, (char)54, (char)12, (char)2, (char)170, (char)53, (char)228, (char)130, (char)86, (char)152, (char)248, (char)11, (char)230, (char)85, (char)241, (char)85, (char)20, (char)243, (char)245, (char)252, (char)121, (char)233, (char)119, (char)146, (char)17, (char)157, (char)249, (char)76, (char)126, (char)120, (char)215, (char)145, (char)205, (char)186, (char)33, (char)86, (char)228, (char)249, (char)238, (char)212, (char)50, (char)216, (char)15, (char)183, (char)26, (char)131, (char)134, (char)104, (char)173, (char)228, (char)220, (char)251, (char)98, (char)219, (char)132, (char)140, (char)118, (char)225, (char)159, (char)99, (char)30, (char)42, (char)126, (char)136, (char)196, (char)155, (char)55, (char)121, (char)226, (char)216, (char)166, (char)240, (char)251, (char)187, (char)209, (char)43, (char)234, (char)40, (char)121, (char)16, (char)193, (char)241, (char)87, (char)141, (char)88, (char)222, (char)29, (char)70, (char)225, (char)94, (char)210, (char)147, (char)248, (char)230, (char)9, (char)163, (char)209, (char)78, (char)128, (char)241, (char)112, (char)90, (char)10, (char)27, (char)19, (char)11, (char)206, (char)218, (char)167, (char)219, (char)63, (char)151, (char)181, (char)17, (char)34, (char)7, (char)62, (char)107, (char)156, (char)14, (char)16, (char)141, (char)229, (char)164, (char)2, (char)102, (char)248, (char)215, (char)16, (char)187, (char)38, (char)241, (char)155, (char)124, (char)39, (char)225, (char)247, (char)41, (char)171, (char)214, (char)150, (char)116, (char)170, (char)176, (char)39, (char)133, (char)2, (char)246, (char)138, (char)189, (char)12, (char)23, (char)86, (char)249, (char)214, (char)60, (char)146, (char)108, (char)212, (char)195, (char)157, (char)6, (char)61, (char)48, (char)187, (char)110, (char)155, (char)106, (char)225, (char)110, (char)133, (char)19, (char)198, (char)220, (char)191, (char)239, (char)6, (char)220, (char)173, (char)241, (char)222, (char)213, (char)16, (char)249, (char)5, (char)247, (char)48, (char)195, (char)23, (char)122, (char)157, (char)160, (char)60, (char)229, (char)212, (char)134, (char)9, (char)182, (char)230, (char)158, (char)187, (char)2, (char)239, (char)247, (char)229, (char)248, (char)216, (char)14}, 0) ;
        p267.target_system_SET((char)66) ;
        p267.first_message_offset_SET((char)81) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)117);
            assert(pack.sequence_GET() == (char)30029);
            assert(pack.target_component_GET() == (char)142);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)142) ;
        p268.target_system_SET((char)117) ;
        p268.sequence_SET((char)30029) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.rotation_GET() == (char)42853);
            assert(pack.camera_id_GET() == (char)48);
            assert(pack.bitrate_GET() == 2046235600L);
            assert(pack.status_GET() == (char)54);
            assert(pack.resolution_h_GET() == (char)43540);
            assert(pack.framerate_GET() == -3.2554162E38F);
            assert(pack.uri_LEN(ph) == 45);
            assert(pack.uri_TRY(ph).equals("iFuQxcxkEpexXouYzqmkarlDcakxlyimcyCSnqkgPtjxj"));
            assert(pack.resolution_v_GET() == (char)7939);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_v_SET((char)7939) ;
        p269.framerate_SET(-3.2554162E38F) ;
        p269.uri_SET("iFuQxcxkEpexXouYzqmkarlDcakxlyimcyCSnqkgPtjxj", PH) ;
        p269.resolution_h_SET((char)43540) ;
        p269.status_SET((char)54) ;
        p269.rotation_SET((char)42853) ;
        p269.bitrate_SET(2046235600L) ;
        p269.camera_id_SET((char)48) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.framerate_GET() == -1.2212116E38F);
            assert(pack.target_system_GET() == (char)223);
            assert(pack.resolution_v_GET() == (char)58965);
            assert(pack.uri_LEN(ph) == 73);
            assert(pack.uri_TRY(ph).equals("ZagrDsybyuctlhcIMofyYxxugwrkgrzhotakfqkrxglgvQibtpboxbmsqazstbhsedqixzuaw"));
            assert(pack.resolution_h_GET() == (char)46702);
            assert(pack.bitrate_GET() == 2467799628L);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.camera_id_GET() == (char)110);
            assert(pack.rotation_GET() == (char)245);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(2467799628L) ;
        p270.resolution_h_SET((char)46702) ;
        p270.uri_SET("ZagrDsybyuctlhcIMofyYxxugwrkgrzhotakfqkrxglgvQibtpboxbmsqazstbhsedqixzuaw", PH) ;
        p270.resolution_v_SET((char)58965) ;
        p270.rotation_SET((char)245) ;
        p270.camera_id_SET((char)110) ;
        p270.framerate_SET(-1.2212116E38F) ;
        p270.target_component_SET((char)222) ;
        p270.target_system_SET((char)223) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 1);
            assert(pack.ssid_TRY(ph).equals("l"));
            assert(pack.password_LEN(ph) == 58);
            assert(pack.password_TRY(ph).equals("wbsklkirrnezuxwIspojnmdmidrfhuerzjekGowntyknqvwrgYqybxnjdc"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("wbsklkirrnezuxwIspojnmdmidrfhuerzjekGowntyknqvwrgYqybxnjdc", PH) ;
        p299.ssid_SET("l", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)214, (char)182, (char)58, (char)44, (char)64, (char)162, (char)15, (char)224}));
            assert(pack.version_GET() == (char)58550);
            assert(pack.min_version_GET() == (char)49969);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)70, (char)153, (char)181, (char)132, (char)167, (char)146, (char)143, (char)234}));
            assert(pack.max_version_GET() == (char)22944);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)58550) ;
        p300.spec_version_hash_SET(new char[] {(char)70, (char)153, (char)181, (char)132, (char)167, (char)146, (char)143, (char)234}, 0) ;
        p300.library_version_hash_SET(new char[] {(char)214, (char)182, (char)58, (char)44, (char)64, (char)162, (char)15, (char)224}, 0) ;
        p300.max_version_SET((char)22944) ;
        p300.min_version_SET((char)49969) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1264697765L);
            assert(pack.time_usec_GET() == 8833464726458897184L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            assert(pack.vendor_specific_status_code_GET() == (char)205);
            assert(pack.sub_mode_GET() == (char)60);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.vendor_specific_status_code_SET((char)205) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK) ;
        p310.time_usec_SET(8833464726458897184L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.sub_mode_SET((char)60) ;
        p310.uptime_sec_SET(1264697765L) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_major_GET() == (char)41);
            assert(pack.hw_version_minor_GET() == (char)183);
            assert(pack.sw_version_minor_GET() == (char)129);
            assert(pack.name_LEN(ph) == 74);
            assert(pack.name_TRY(ph).equals("wbtsbqkgmfiCKdndodfsynpbbkxejhxqixzjdeIfuhieKbfxkwiakqtbnOuaiccCYhiEiggvox"));
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)169, (char)24, (char)57, (char)130, (char)23, (char)73, (char)232, (char)74, (char)134, (char)62, (char)45, (char)123, (char)24, (char)5, (char)3, (char)126}));
            assert(pack.hw_version_major_GET() == (char)113);
            assert(pack.sw_vcs_commit_GET() == 3694700989L);
            assert(pack.uptime_sec_GET() == 3201989139L);
            assert(pack.time_usec_GET() == 9222326437262898520L);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_minor_SET((char)183) ;
        p311.sw_version_minor_SET((char)129) ;
        p311.name_SET("wbtsbqkgmfiCKdndodfsynpbbkxejhxqixzjdeIfuhieKbfxkwiakqtbnOuaiccCYhiEiggvox", PH) ;
        p311.hw_unique_id_SET(new char[] {(char)169, (char)24, (char)57, (char)130, (char)23, (char)73, (char)232, (char)74, (char)134, (char)62, (char)45, (char)123, (char)24, (char)5, (char)3, (char)126}, 0) ;
        p311.uptime_sec_SET(3201989139L) ;
        p311.sw_vcs_commit_SET(3694700989L) ;
        p311.hw_version_major_SET((char)113) ;
        p311.time_usec_SET(9222326437262898520L) ;
        p311.sw_version_major_SET((char)41) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)94);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.param_index_GET() == (short) -27391);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("uGqyquykiebqv"));
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_component_SET((char)156) ;
        p320.param_index_SET((short) -27391) ;
        p320.target_system_SET((char)94) ;
        p320.param_id_SET("uGqyquykiebqv", PH) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)212);
            assert(pack.target_system_GET() == (char)229);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)212) ;
        p321.target_system_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)17628);
            assert(pack.param_id_LEN(ph) == 8);
            assert(pack.param_id_TRY(ph).equals("lqhpifcw"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            assert(pack.param_value_LEN(ph) == 67);
            assert(pack.param_value_TRY(ph).equals("nrkBMQkhmjPqasphvawymlnfvfTefbtsaBgkyxnoqyvlzpalejtcvhfvfwgtncDywdm"));
            assert(pack.param_index_GET() == (char)56246);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)17628) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        p322.param_id_SET("lqhpifcw", PH) ;
        p322.param_value_SET("nrkBMQkhmjPqasphvawymlnfvfTefbtsaBgkyxnoqyvlzpalejtcvhfvfwgtncDywdm", PH) ;
        p322.param_index_SET((char)56246) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)15);
            assert(pack.target_component_GET() == (char)229);
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("dzoezlwOno"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            assert(pack.param_value_LEN(ph) == 121);
            assert(pack.param_value_TRY(ph).equals("jafgjdgapgQvdlcwauiEjdhdTfjmxorOehkxqfYwaayovgzvcCnTKrplzlkvcbsTjsBzvhrltaakiwbjimdixovfvwydivboPwozlPpolfqshnMeywpsyjbvz"));
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        p323.param_id_SET("dzoezlwOno", PH) ;
        p323.param_value_SET("jafgjdgapgQvdlcwauiEjdhdTfjmxorOehkxqfYwaayovgzvcCnTKrplzlkvcbsTjsBzvhrltaakiwbjimdixovfvwydivboPwozlPpolfqshnMeywpsyjbvz", PH) ;
        p323.target_system_SET((char)15) ;
        p323.target_component_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_value_LEN(ph) == 125);
            assert(pack.param_value_TRY(ph).equals("xgzzGnctVzehivhshrxxuwotxfqLzimciqnlxlCMgusgfcLSbvfywrruqvoyxpqjrVMxZebkpvvfCaqnnxwtbjmhiSlgjucvmzmJqkvjiadsjbaqoTfosfbyettoc"));
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("ksUE"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("xgzzGnctVzehivhshrxxuwotxfqLzimciqnlxlCMgusgfcLSbvfywrruqvoyxpqjrVMxZebkpvvfCaqnnxwtbjmhiSlgjucvmzmJqkvjiadsjbaqoTfosfbyettoc", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        p324.param_id_SET("ksUE", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.max_distance_GET() == (char)23016);
            assert(pack.increment_GET() == (char)94);
            assert(pack.time_usec_GET() == 3177035429033980342L);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)63704, (char)64524, (char)20948, (char)22595, (char)824, (char)55282, (char)59280, (char)18752, (char)57095, (char)48650, (char)44651, (char)16207, (char)55571, (char)35403, (char)57191, (char)18702, (char)17761, (char)16291, (char)42309, (char)56038, (char)22969, (char)40956, (char)26298, (char)38538, (char)58134, (char)20162, (char)5943, (char)1459, (char)17979, (char)18026, (char)42131, (char)21548, (char)11370, (char)47403, (char)32398, (char)52548, (char)56320, (char)31479, (char)14731, (char)15478, (char)18660, (char)65023, (char)8462, (char)55838, (char)16749, (char)19665, (char)53014, (char)39407, (char)10593, (char)16492, (char)5262, (char)18153, (char)35637, (char)61933, (char)45594, (char)33507, (char)28364, (char)51359, (char)57765, (char)24637, (char)34283, (char)30396, (char)26517, (char)30849, (char)52055, (char)55709, (char)5379, (char)6974, (char)42306, (char)18656, (char)49789, (char)61381}));
            assert(pack.min_distance_GET() == (char)14091);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.time_usec_SET(3177035429033980342L) ;
        p330.min_distance_SET((char)14091) ;
        p330.distances_SET(new char[] {(char)63704, (char)64524, (char)20948, (char)22595, (char)824, (char)55282, (char)59280, (char)18752, (char)57095, (char)48650, (char)44651, (char)16207, (char)55571, (char)35403, (char)57191, (char)18702, (char)17761, (char)16291, (char)42309, (char)56038, (char)22969, (char)40956, (char)26298, (char)38538, (char)58134, (char)20162, (char)5943, (char)1459, (char)17979, (char)18026, (char)42131, (char)21548, (char)11370, (char)47403, (char)32398, (char)52548, (char)56320, (char)31479, (char)14731, (char)15478, (char)18660, (char)65023, (char)8462, (char)55838, (char)16749, (char)19665, (char)53014, (char)39407, (char)10593, (char)16492, (char)5262, (char)18153, (char)35637, (char)61933, (char)45594, (char)33507, (char)28364, (char)51359, (char)57765, (char)24637, (char)34283, (char)30396, (char)26517, (char)30849, (char)52055, (char)55709, (char)5379, (char)6974, (char)42306, (char)18656, (char)49789, (char)61381}, 0) ;
        p330.increment_SET((char)94) ;
        p330.max_distance_SET((char)23016) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.ICAO_GET() == 470077687L);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA);
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M);
            assert(pack.callsign_LEN(ph) == 4);
            assert(pack.callsign_TRY(ph).equals("gwip"));
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
            assert(pack.stallSpeed_GET() == (char)21051);
            assert(pack.rfSelect_GET() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
        });
        DemoDevice.UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M) ;
        p10001.ICAO_SET(470077687L) ;
        p10001.callsign_SET("gwip", PH) ;
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA) ;
        p10001.stallSpeed_SET((char)21051) ;
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED) ;
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR) ;
        LoopBackDemoChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.accuracyHor_GET() == 671506513L);
            assert(pack.gpsAlt_GET() == -1517388356);
            assert(pack.baroAltMSL_GET() == 1512693465);
            assert(pack.squawk_GET() == (char)14024);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY);
            assert(pack.gpsLat_GET() == -740170661);
            assert(pack.gpsLon_GET() == 1953902689);
            assert(pack.state_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT);
            assert(pack.VelEW_GET() == (short)10546);
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
            assert(pack.utcTime_GET() == 3833332587L);
            assert(pack.accuracyVel_GET() == (char)18532);
            assert(pack.accuracyVert_GET() == (char)40081);
            assert(pack.velVert_GET() == (short)20199);
            assert(pack.velNS_GET() == (short)18254);
            assert(pack.numSats_GET() == (char)211);
        });
        DemoDevice.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.squawk_SET((char)14024) ;
        p10002.accuracyVel_SET((char)18532) ;
        p10002.VelEW_SET((short)10546) ;
        p10002.baroAltMSL_SET(1512693465) ;
        p10002.accuracyHor_SET(671506513L) ;
        p10002.velVert_SET((short)20199) ;
        p10002.gpsLon_SET(1953902689) ;
        p10002.velNS_SET((short)18254) ;
        p10002.utcTime_SET(3833332587L) ;
        p10002.gpsAlt_SET(-1517388356) ;
        p10002.gpsLat_SET(-740170661) ;
        p10002.numSats_SET((char)211) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY) ;
        p10002.accuracyVert_SET((char)40081) ;
        p10002.state_SET(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT) ;
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0) ;
        LoopBackDemoChannel.instance.send(p10002);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rfHealth_GET() == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
        });
        DemoDevice.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX) ;
        LoopBackDemoChannel.instance.send(p10003);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEVICE_OP_READ.add((src, ph, pack) ->
        {
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
            assert(pack.target_system_GET() == (char)193);
            assert(pack.address_GET() == (char)124);
            assert(pack.regstart_GET() == (char)2);
            assert(pack.count_GET() == (char)29);
            assert(pack.bus_GET() == (char)190);
            assert(pack.target_component_GET() == (char)3);
            assert(pack.request_id_GET() == 3266299565L);
            assert(pack.busname_LEN(ph) == 9);
            assert(pack.busname_TRY(ph).equals("uioujvkaG"));
        });
        DemoDevice.DEVICE_OP_READ p11000 = LoopBackDemoChannel.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.request_id_SET(3266299565L) ;
        p11000.target_system_SET((char)193) ;
        p11000.address_SET((char)124) ;
        p11000.regstart_SET((char)2) ;
        p11000.target_component_SET((char)3) ;
        p11000.busname_SET("uioujvkaG", PH) ;
        p11000.bus_SET((char)190) ;
        p11000.count_SET((char)29) ;
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C) ;
        LoopBackDemoChannel.instance.send(p11000);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == (char)149);
            assert(pack.regstart_GET() == (char)218);
            assert(pack.count_GET() == (char)155);
            assert(pack.request_id_GET() == 1181152341L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)49, (char)47, (char)17, (char)226, (char)120, (char)37, (char)26, (char)129, (char)45, (char)106, (char)62, (char)42, (char)113, (char)104, (char)200, (char)41, (char)3, (char)18, (char)96, (char)60, (char)230, (char)38, (char)80, (char)241, (char)9, (char)153, (char)146, (char)46, (char)212, (char)20, (char)251, (char)63, (char)38, (char)121, (char)50, (char)157, (char)196, (char)74, (char)219, (char)176, (char)138, (char)70, (char)234, (char)55, (char)76, (char)5, (char)254, (char)213, (char)150, (char)114, (char)56, (char)9, (char)162, (char)112, (char)38, (char)15, (char)208, (char)95, (char)105, (char)201, (char)184, (char)53, (char)207, (char)235, (char)58, (char)1, (char)81, (char)180, (char)136, (char)116, (char)100, (char)69, (char)99, (char)254, (char)51, (char)142, (char)34, (char)148, (char)141, (char)255, (char)51, (char)141, (char)76, (char)148, (char)116, (char)148, (char)29, (char)200, (char)27, (char)101, (char)210, (char)144, (char)57, (char)23, (char)102, (char)82, (char)104, (char)246, (char)35, (char)152, (char)105, (char)1, (char)151, (char)198, (char)19, (char)48, (char)88, (char)2, (char)49, (char)233, (char)201, (char)161, (char)134, (char)164, (char)196, (char)229, (char)254, (char)48, (char)14, (char)71, (char)203, (char)15, (char)188, (char)94, (char)199, (char)169, (char)171, (char)146}));
        });
        DemoDevice.DEVICE_OP_READ_REPLY p11001 = LoopBackDemoChannel.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.regstart_SET((char)218) ;
        p11001.result_SET((char)149) ;
        p11001.count_SET((char)155) ;
        p11001.request_id_SET(1181152341L) ;
        p11001.data__SET(new char[] {(char)49, (char)47, (char)17, (char)226, (char)120, (char)37, (char)26, (char)129, (char)45, (char)106, (char)62, (char)42, (char)113, (char)104, (char)200, (char)41, (char)3, (char)18, (char)96, (char)60, (char)230, (char)38, (char)80, (char)241, (char)9, (char)153, (char)146, (char)46, (char)212, (char)20, (char)251, (char)63, (char)38, (char)121, (char)50, (char)157, (char)196, (char)74, (char)219, (char)176, (char)138, (char)70, (char)234, (char)55, (char)76, (char)5, (char)254, (char)213, (char)150, (char)114, (char)56, (char)9, (char)162, (char)112, (char)38, (char)15, (char)208, (char)95, (char)105, (char)201, (char)184, (char)53, (char)207, (char)235, (char)58, (char)1, (char)81, (char)180, (char)136, (char)116, (char)100, (char)69, (char)99, (char)254, (char)51, (char)142, (char)34, (char)148, (char)141, (char)255, (char)51, (char)141, (char)76, (char)148, (char)116, (char)148, (char)29, (char)200, (char)27, (char)101, (char)210, (char)144, (char)57, (char)23, (char)102, (char)82, (char)104, (char)246, (char)35, (char)152, (char)105, (char)1, (char)151, (char)198, (char)19, (char)48, (char)88, (char)2, (char)49, (char)233, (char)201, (char)161, (char)134, (char)164, (char)196, (char)229, (char)254, (char)48, (char)14, (char)71, (char)203, (char)15, (char)188, (char)94, (char)199, (char)169, (char)171, (char)146}, 0) ;
        LoopBackDemoChannel.instance.send(p11001);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)126);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)171, (char)168, (char)177, (char)248, (char)5, (char)110, (char)147, (char)188, (char)252, (char)228, (char)139, (char)28, (char)5, (char)64, (char)14, (char)92, (char)113, (char)14, (char)44, (char)189, (char)234, (char)55, (char)248, (char)146, (char)2, (char)2, (char)169, (char)171, (char)252, (char)201, (char)123, (char)34, (char)148, (char)166, (char)52, (char)161, (char)65, (char)136, (char)111, (char)43, (char)57, (char)218, (char)126, (char)47, (char)73, (char)122, (char)162, (char)59, (char)168, (char)192, (char)207, (char)112, (char)40, (char)250, (char)39, (char)142, (char)140, (char)73, (char)247, (char)21, (char)113, (char)35, (char)221, (char)202, (char)125, (char)39, (char)241, (char)86, (char)48, (char)139, (char)148, (char)164, (char)184, (char)149, (char)11, (char)193, (char)80, (char)31, (char)202, (char)55, (char)134, (char)34, (char)245, (char)214, (char)183, (char)127, (char)177, (char)43, (char)22, (char)76, (char)1, (char)154, (char)196, (char)66, (char)225, (char)101, (char)44, (char)53, (char)199, (char)122, (char)145, (char)223, (char)108, (char)138, (char)211, (char)162, (char)129, (char)123, (char)215, (char)212, (char)194, (char)255, (char)106, (char)224, (char)187, (char)124, (char)146, (char)37, (char)31, (char)101, (char)165, (char)224, (char)37, (char)155, (char)167, (char)70, (char)62, (char)78}));
            assert(pack.bustype_GET() == DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
            assert(pack.target_component_GET() == (char)51);
            assert(pack.busname_LEN(ph) == 15);
            assert(pack.busname_TRY(ph).equals("fckoolcJtxcmgoj"));
            assert(pack.request_id_GET() == 3114010464L);
            assert(pack.regstart_GET() == (char)141);
            assert(pack.bus_GET() == (char)26);
            assert(pack.address_GET() == (char)4);
            assert(pack.count_GET() == (char)187);
        });
        DemoDevice.DEVICE_OP_WRITE p11002 = LoopBackDemoChannel.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.target_component_SET((char)51) ;
        p11002.address_SET((char)4) ;
        p11002.regstart_SET((char)141) ;
        p11002.data__SET(new char[] {(char)171, (char)168, (char)177, (char)248, (char)5, (char)110, (char)147, (char)188, (char)252, (char)228, (char)139, (char)28, (char)5, (char)64, (char)14, (char)92, (char)113, (char)14, (char)44, (char)189, (char)234, (char)55, (char)248, (char)146, (char)2, (char)2, (char)169, (char)171, (char)252, (char)201, (char)123, (char)34, (char)148, (char)166, (char)52, (char)161, (char)65, (char)136, (char)111, (char)43, (char)57, (char)218, (char)126, (char)47, (char)73, (char)122, (char)162, (char)59, (char)168, (char)192, (char)207, (char)112, (char)40, (char)250, (char)39, (char)142, (char)140, (char)73, (char)247, (char)21, (char)113, (char)35, (char)221, (char)202, (char)125, (char)39, (char)241, (char)86, (char)48, (char)139, (char)148, (char)164, (char)184, (char)149, (char)11, (char)193, (char)80, (char)31, (char)202, (char)55, (char)134, (char)34, (char)245, (char)214, (char)183, (char)127, (char)177, (char)43, (char)22, (char)76, (char)1, (char)154, (char)196, (char)66, (char)225, (char)101, (char)44, (char)53, (char)199, (char)122, (char)145, (char)223, (char)108, (char)138, (char)211, (char)162, (char)129, (char)123, (char)215, (char)212, (char)194, (char)255, (char)106, (char)224, (char)187, (char)124, (char)146, (char)37, (char)31, (char)101, (char)165, (char)224, (char)37, (char)155, (char)167, (char)70, (char)62, (char)78}, 0) ;
        p11002.target_system_SET((char)126) ;
        p11002.count_SET((char)187) ;
        p11002.bus_SET((char)26) ;
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI) ;
        p11002.busname_SET("fckoolcJtxcmgoj", PH) ;
        p11002.request_id_SET(3114010464L) ;
        LoopBackDemoChannel.instance.send(p11002);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            assert(pack.result_GET() == (char)205);
            assert(pack.request_id_GET() == 215024065L);
        });
        DemoDevice.DEVICE_OP_WRITE_REPLY p11003 = LoopBackDemoChannel.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.result_SET((char)205) ;
        p11003.request_id_SET(215024065L) ;
        LoopBackDemoChannel.instance.send(p11003);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            assert(pack.u_GET() == 1.3845791E38F);
            assert(pack.theta_GET() == 1.0072848E38F);
            assert(pack.omega_GET() == -2.7580785E38F);
            assert(pack.error_GET() == 1.6412651E38F);
            assert(pack.omega_dot_GET() == 1.5507253E38F);
            assert(pack.axis_GET() == PID_TUNING_AXIS.PID_TUNING_ACCZ);
            assert(pack.f_GET() == -3.205818E38F);
            assert(pack.f_dot_GET() == 7.8918993E37F);
            assert(pack.sigma_dot_GET() == -1.5103567E38F);
            assert(pack.theta_dot_GET() == 1.5551501E38F);
            assert(pack.sigma_GET() == 1.2858292E38F);
            assert(pack.desired_GET() == -2.268674E38F);
            assert(pack.achieved_GET() == 2.197295E37F);
        });
        DemoDevice.ADAP_TUNING p11010 = LoopBackDemoChannel.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.desired_SET(-2.268674E38F) ;
        p11010.achieved_SET(2.197295E37F) ;
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_ACCZ) ;
        p11010.omega_dot_SET(1.5507253E38F) ;
        p11010.sigma_dot_SET(-1.5103567E38F) ;
        p11010.theta_SET(1.0072848E38F) ;
        p11010.omega_SET(-2.7580785E38F) ;
        p11010.theta_dot_SET(1.5551501E38F) ;
        p11010.f_SET(-3.205818E38F) ;
        p11010.error_SET(1.6412651E38F) ;
        p11010.f_dot_SET(7.8918993E37F) ;
        p11010.u_SET(1.3845791E38F) ;
        p11010.sigma_SET(1.2858292E38F) ;
        LoopBackDemoChannel.instance.send(p11010);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            assert(pack.time_delta_usec_GET() == 7578213623855507825L);
            assert(pack.confidence_GET() == 2.549385E38F);
            assert(pack.time_usec_GET() == 1426109774266330975L);
            assert(Arrays.equals(pack.angle_delta_GET(),  new float[] {8.802744E37F, -6.305875E36F, 1.1623542E38F}));
            assert(Arrays.equals(pack.position_delta_GET(),  new float[] {-2.552605E38F, -1.5692183E38F, 1.5855644E38F}));
        });
        DemoDevice.VISION_POSITION_DELTA p11011 = LoopBackDemoChannel.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.angle_delta_SET(new float[] {8.802744E37F, -6.305875E36F, 1.1623542E38F}, 0) ;
        p11011.time_delta_usec_SET(7578213623855507825L) ;
        p11011.confidence_SET(2.549385E38F) ;
        p11011.position_delta_SET(new float[] {-2.552605E38F, -1.5692183E38F, 1.5855644E38F}, 0) ;
        p11011.time_usec_SET(1426109774266330975L) ;
        LoopBackDemoChannel.instance.send(p11011);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
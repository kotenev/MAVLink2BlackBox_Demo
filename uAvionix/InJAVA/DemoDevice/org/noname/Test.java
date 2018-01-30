
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
            assert(pack.mavlink_version_GET() == (char)244);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
            assert(pack.custom_mode_GET() == 3044132610L);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(3044132610L) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_FLIGHT_TERMINATION) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5) ;
        p0.mavlink_version_SET((char)244) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count1_GET() == (char)27680);
            assert(pack.voltage_battery_GET() == (char)64102);
            assert(pack.errors_count3_GET() == (char)24875);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
            assert(pack.battery_remaining_GET() == (byte)110);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
            assert(pack.errors_count4_GET() == (char)26418);
            assert(pack.load_GET() == (char)19586);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
            assert(pack.current_battery_GET() == (short)23077);
            assert(pack.drop_rate_comm_GET() == (char)30478);
            assert(pack.errors_count2_GET() == (char)44937);
            assert(pack.errors_comm_GET() == (char)22864);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count1_SET((char)27680) ;
        p1.voltage_battery_SET((char)64102) ;
        p1.errors_count3_SET((char)24875) ;
        p1.drop_rate_comm_SET((char)30478) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL) ;
        p1.load_SET((char)19586) ;
        p1.battery_remaining_SET((byte)110) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) ;
        p1.current_battery_SET((short)23077) ;
        p1.errors_comm_SET((char)22864) ;
        p1.errors_count2_SET((char)44937) ;
        p1.errors_count4_SET((char)26418) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2638825652L);
            assert(pack.time_unix_usec_GET() == 8896568855748926877L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2638825652L) ;
        p2.time_unix_usec_SET(8896568855748926877L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 1.1522589E38F);
            assert(pack.z_GET() == -7.6608203E37F);
            assert(pack.x_GET() == 2.3833638E38F);
            assert(pack.afz_GET() == -1.6853999E38F);
            assert(pack.vz_GET() == -3.1355912E38F);
            assert(pack.vx_GET() == 1.2969323E38F);
            assert(pack.afx_GET() == 4.0751486E37F);
            assert(pack.afy_GET() == -2.536454E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.time_boot_ms_GET() == 1658267830L);
            assert(pack.yaw_GET() == 2.8651552E38F);
            assert(pack.vy_GET() == 1.0950375E38F);
            assert(pack.y_GET() == 1.9111376E38F);
            assert(pack.type_mask_GET() == (char)21838);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.vy_SET(1.0950375E38F) ;
        p3.yaw_SET(2.8651552E38F) ;
        p3.y_SET(1.9111376E38F) ;
        p3.vx_SET(1.2969323E38F) ;
        p3.afy_SET(-2.536454E38F) ;
        p3.type_mask_SET((char)21838) ;
        p3.vz_SET(-3.1355912E38F) ;
        p3.afx_SET(4.0751486E37F) ;
        p3.yaw_rate_SET(1.1522589E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p3.x_SET(2.3833638E38F) ;
        p3.z_SET(-7.6608203E37F) ;
        p3.time_boot_ms_SET(1658267830L) ;
        p3.afz_SET(-1.6853999E38F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)151);
            assert(pack.time_usec_GET() == 1322187791609525140L);
            assert(pack.target_component_GET() == (char)242);
            assert(pack.seq_GET() == 527331023L);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.target_system_SET((char)151) ;
        p4.time_usec_SET(1322187791609525140L) ;
        p4.target_component_SET((char)242) ;
        p4.seq_SET(527331023L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)207);
            assert(pack.version_GET() == (char)6);
            assert(pack.control_request_GET() == (char)20);
            assert(pack.passkey_LEN(ph) == 10);
            assert(pack.passkey_TRY(ph).equals("reubxjqucv"));
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("reubxjqucv", PH) ;
        p5.target_system_SET((char)207) ;
        p5.control_request_SET((char)20) ;
        p5.version_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)75);
            assert(pack.control_request_GET() == (char)101);
            assert(pack.gcs_system_id_GET() == (char)73);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.control_request_SET((char)101) ;
        p6.ack_SET((char)75) ;
        p6.gcs_system_id_SET((char)73) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 5);
            assert(pack.key_TRY(ph).equals("grqpm"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("grqpm", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)8);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(pack.custom_mode_GET() == 1471767658L);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p11.custom_mode_SET(1471767658L) ;
        p11.target_system_SET((char)8) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)234);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("kxrl"));
            assert(pack.param_index_GET() == (short)30782);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)234) ;
        p20.target_component_SET((char)37) ;
        p20.param_index_SET((short)30782) ;
        p20.param_id_SET("kxrl", PH) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)101);
            assert(pack.target_component_GET() == (char)31);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)101) ;
        p21.target_component_SET((char)31) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("ihwo"));
            assert(pack.param_index_GET() == (char)42280);
            assert(pack.param_value_GET() == 9.144202E37F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_count_GET() == (char)23848);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_id_SET("ihwo", PH) ;
        p22.param_index_SET((char)42280) ;
        p22.param_count_SET((char)23848) ;
        p22.param_value_SET(9.144202E37F) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)88);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("yniejilmwrzbjtgm"));
            assert(pack.param_value_GET() == -9.326889E37F);
            assert(pack.target_component_GET() == (char)16);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("yniejilmwrzbjtgm", PH) ;
        p23.param_value_SET(-9.326889E37F) ;
        p23.target_system_SET((char)88) ;
        p23.target_component_SET((char)16) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.vel_acc_TRY(ph) == 3050447506L);
            assert(pack.cog_GET() == (char)60771);
            assert(pack.h_acc_TRY(ph) == 91354669L);
            assert(pack.epv_GET() == (char)33998);
            assert(pack.alt_GET() == -1246904614);
            assert(pack.vel_GET() == (char)42515);
            assert(pack.lat_GET() == 1329562238);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            assert(pack.satellites_visible_GET() == (char)16);
            assert(pack.time_usec_GET() == 2500162947139383775L);
            assert(pack.hdg_acc_TRY(ph) == 1061730705L);
            assert(pack.v_acc_TRY(ph) == 2675191568L);
            assert(pack.eph_GET() == (char)58007);
            assert(pack.lon_GET() == 1150668630);
            assert(pack.alt_ellipsoid_TRY(ph) == -625846300);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.lon_SET(1150668630) ;
        p24.lat_SET(1329562238) ;
        p24.satellites_visible_SET((char)16) ;
        p24.vel_SET((char)42515) ;
        p24.cog_SET((char)60771) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS) ;
        p24.h_acc_SET(91354669L, PH) ;
        p24.v_acc_SET(2675191568L, PH) ;
        p24.eph_SET((char)58007) ;
        p24.alt_ellipsoid_SET(-625846300, PH) ;
        p24.epv_SET((char)33998) ;
        p24.vel_acc_SET(3050447506L, PH) ;
        p24.time_usec_SET(2500162947139383775L) ;
        p24.alt_SET(-1246904614) ;
        p24.hdg_acc_SET(1061730705L, PH) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)210, (char)162, (char)182, (char)12, (char)83, (char)194, (char)133, (char)70, (char)95, (char)60, (char)184, (char)232, (char)53, (char)47, (char)42, (char)56, (char)173, (char)214, (char)139, (char)70}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)41, (char)136, (char)66, (char)108, (char)45, (char)34, (char)211, (char)195, (char)204, (char)168, (char)121, (char)121, (char)186, (char)99, (char)221, (char)13, (char)154, (char)115, (char)214, (char)185}));
            assert(pack.satellites_visible_GET() == (char)136);
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)24, (char)145, (char)3, (char)193, (char)35, (char)227, (char)197, (char)169, (char)111, (char)50, (char)182, (char)120, (char)159, (char)146, (char)85, (char)248, (char)78, (char)229, (char)144, (char)60}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)222, (char)185, (char)75, (char)22, (char)114, (char)216, (char)59, (char)222, (char)84, (char)7, (char)80, (char)100, (char)249, (char)93, (char)26, (char)102, (char)9, (char)27, (char)254, (char)66}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)98, (char)4, (char)122, (char)121, (char)112, (char)132, (char)250, (char)20, (char)158, (char)92, (char)176, (char)46, (char)139, (char)110, (char)94, (char)167, (char)37, (char)27, (char)156, (char)96}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_snr_SET(new char[] {(char)41, (char)136, (char)66, (char)108, (char)45, (char)34, (char)211, (char)195, (char)204, (char)168, (char)121, (char)121, (char)186, (char)99, (char)221, (char)13, (char)154, (char)115, (char)214, (char)185}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)222, (char)185, (char)75, (char)22, (char)114, (char)216, (char)59, (char)222, (char)84, (char)7, (char)80, (char)100, (char)249, (char)93, (char)26, (char)102, (char)9, (char)27, (char)254, (char)66}, 0) ;
        p25.satellites_visible_SET((char)136) ;
        p25.satellite_used_SET(new char[] {(char)24, (char)145, (char)3, (char)193, (char)35, (char)227, (char)197, (char)169, (char)111, (char)50, (char)182, (char)120, (char)159, (char)146, (char)85, (char)248, (char)78, (char)229, (char)144, (char)60}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)98, (char)4, (char)122, (char)121, (char)112, (char)132, (char)250, (char)20, (char)158, (char)92, (char)176, (char)46, (char)139, (char)110, (char)94, (char)167, (char)37, (char)27, (char)156, (char)96}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)210, (char)162, (char)182, (char)12, (char)83, (char)194, (char)133, (char)70, (char)95, (char)60, (char)184, (char)232, (char)53, (char)47, (char)42, (char)56, (char)173, (char)214, (char)139, (char)70}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short) -592);
            assert(pack.time_boot_ms_GET() == 3959731143L);
            assert(pack.zacc_GET() == (short)24300);
            assert(pack.xacc_GET() == (short)16469);
            assert(pack.xmag_GET() == (short)24547);
            assert(pack.ygyro_GET() == (short)19543);
            assert(pack.ymag_GET() == (short) -7863);
            assert(pack.zmag_GET() == (short)27277);
            assert(pack.xgyro_GET() == (short) -15894);
            assert(pack.yacc_GET() == (short)7295);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(3959731143L) ;
        p26.xacc_SET((short)16469) ;
        p26.zmag_SET((short)27277) ;
        p26.ymag_SET((short) -7863) ;
        p26.zgyro_SET((short) -592) ;
        p26.zacc_SET((short)24300) ;
        p26.xmag_SET((short)24547) ;
        p26.yacc_SET((short)7295) ;
        p26.ygyro_SET((short)19543) ;
        p26.xgyro_SET((short) -15894) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short) -9752);
            assert(pack.xmag_GET() == (short)4752);
            assert(pack.ygyro_GET() == (short) -11992);
            assert(pack.zgyro_GET() == (short)8079);
            assert(pack.zmag_GET() == (short) -25411);
            assert(pack.ymag_GET() == (short)2038);
            assert(pack.time_usec_GET() == 8291236803220497511L);
            assert(pack.xacc_GET() == (short)20633);
            assert(pack.yacc_GET() == (short)24863);
            assert(pack.xgyro_GET() == (short)11501);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.ymag_SET((short)2038) ;
        p27.ygyro_SET((short) -11992) ;
        p27.zmag_SET((short) -25411) ;
        p27.zgyro_SET((short)8079) ;
        p27.time_usec_SET(8291236803220497511L) ;
        p27.xgyro_SET((short)11501) ;
        p27.yacc_SET((short)24863) ;
        p27.zacc_SET((short) -9752) ;
        p27.xacc_SET((short)20633) ;
        p27.xmag_SET((short)4752) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2928608428323471911L);
            assert(pack.press_abs_GET() == (short)15832);
            assert(pack.temperature_GET() == (short)6501);
            assert(pack.press_diff2_GET() == (short)6536);
            assert(pack.press_diff1_GET() == (short) -17168);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_abs_SET((short)15832) ;
        p28.press_diff1_SET((short) -17168) ;
        p28.temperature_SET((short)6501) ;
        p28.time_usec_SET(2928608428323471911L) ;
        p28.press_diff2_SET((short)6536) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -5037);
            assert(pack.time_boot_ms_GET() == 3374928955L);
            assert(pack.press_diff_GET() == 6.0094716E37F);
            assert(pack.press_abs_GET() == -2.5643034E38F);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(6.0094716E37F) ;
        p29.time_boot_ms_SET(3374928955L) ;
        p29.press_abs_SET(-2.5643034E38F) ;
        p29.temperature_SET((short) -5037) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == -2.3960975E38F);
            assert(pack.time_boot_ms_GET() == 4131662727L);
            assert(pack.rollspeed_GET() == -2.3268254E37F);
            assert(pack.pitchspeed_GET() == 9.613237E37F);
            assert(pack.pitch_GET() == 2.442283E38F);
            assert(pack.yaw_GET() == -3.1194863E38F);
            assert(pack.roll_GET() == -2.8800235E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.pitch_SET(2.442283E38F) ;
        p30.pitchspeed_SET(9.613237E37F) ;
        p30.rollspeed_SET(-2.3268254E37F) ;
        p30.yawspeed_SET(-2.3960975E38F) ;
        p30.roll_SET(-2.8800235E38F) ;
        p30.time_boot_ms_SET(4131662727L) ;
        p30.yaw_SET(-3.1194863E38F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q3_GET() == 1.9658437E38F);
            assert(pack.q1_GET() == -1.8598951E38F);
            assert(pack.q4_GET() == -2.4768088E38F);
            assert(pack.q2_GET() == -3.3515753E38F);
            assert(pack.time_boot_ms_GET() == 1763520936L);
            assert(pack.rollspeed_GET() == -7.8183796E37F);
            assert(pack.pitchspeed_GET() == -3.6248568E37F);
            assert(pack.yawspeed_GET() == -3.0494086E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.q2_SET(-3.3515753E38F) ;
        p31.time_boot_ms_SET(1763520936L) ;
        p31.q4_SET(-2.4768088E38F) ;
        p31.yawspeed_SET(-3.0494086E38F) ;
        p31.pitchspeed_SET(-3.6248568E37F) ;
        p31.q3_SET(1.9658437E38F) ;
        p31.rollspeed_SET(-7.8183796E37F) ;
        p31.q1_SET(-1.8598951E38F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -1.8977979E38F);
            assert(pack.vx_GET() == 6.365695E37F);
            assert(pack.z_GET() == -1.6321807E38F);
            assert(pack.time_boot_ms_GET() == 3148935491L);
            assert(pack.vz_GET() == -2.8890877E38F);
            assert(pack.x_GET() == 2.7327703E38F);
            assert(pack.y_GET() == 7.0569323E37F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vz_SET(-2.8890877E38F) ;
        p32.time_boot_ms_SET(3148935491L) ;
        p32.z_SET(-1.6321807E38F) ;
        p32.vx_SET(6.365695E37F) ;
        p32.vy_SET(-1.8977979E38F) ;
        p32.x_SET(2.7327703E38F) ;
        p32.y_SET(7.0569323E37F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -1824611743);
            assert(pack.time_boot_ms_GET() == 1271966272L);
            assert(pack.vz_GET() == (short)8108);
            assert(pack.hdg_GET() == (char)39244);
            assert(pack.vx_GET() == (short) -8309);
            assert(pack.lon_GET() == -142858394);
            assert(pack.lat_GET() == -1048728021);
            assert(pack.relative_alt_GET() == 787847821);
            assert(pack.vy_GET() == (short)18997);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lon_SET(-142858394) ;
        p33.vy_SET((short)18997) ;
        p33.vx_SET((short) -8309) ;
        p33.relative_alt_SET(787847821) ;
        p33.lat_SET(-1048728021) ;
        p33.hdg_SET((char)39244) ;
        p33.time_boot_ms_SET(1271966272L) ;
        p33.vz_SET((short)8108) ;
        p33.alt_SET(-1824611743) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan7_scaled_GET() == (short) -28441);
            assert(pack.chan1_scaled_GET() == (short)15515);
            assert(pack.chan4_scaled_GET() == (short)22710);
            assert(pack.chan8_scaled_GET() == (short)31794);
            assert(pack.rssi_GET() == (char)142);
            assert(pack.time_boot_ms_GET() == 3244638184L);
            assert(pack.chan6_scaled_GET() == (short)22574);
            assert(pack.chan2_scaled_GET() == (short) -10494);
            assert(pack.port_GET() == (char)94);
            assert(pack.chan3_scaled_GET() == (short)10962);
            assert(pack.chan5_scaled_GET() == (short)20521);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan7_scaled_SET((short) -28441) ;
        p34.chan8_scaled_SET((short)31794) ;
        p34.port_SET((char)94) ;
        p34.rssi_SET((char)142) ;
        p34.chan6_scaled_SET((short)22574) ;
        p34.chan1_scaled_SET((short)15515) ;
        p34.time_boot_ms_SET(3244638184L) ;
        p34.chan4_scaled_SET((short)22710) ;
        p34.chan3_scaled_SET((short)10962) ;
        p34.chan5_scaled_SET((short)20521) ;
        p34.chan2_scaled_SET((short) -10494) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan7_raw_GET() == (char)45475);
            assert(pack.chan2_raw_GET() == (char)4733);
            assert(pack.chan6_raw_GET() == (char)18712);
            assert(pack.chan3_raw_GET() == (char)7667);
            assert(pack.rssi_GET() == (char)125);
            assert(pack.port_GET() == (char)206);
            assert(pack.chan4_raw_GET() == (char)32577);
            assert(pack.chan5_raw_GET() == (char)50485);
            assert(pack.chan8_raw_GET() == (char)3455);
            assert(pack.time_boot_ms_GET() == 1525178409L);
            assert(pack.chan1_raw_GET() == (char)60395);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan7_raw_SET((char)45475) ;
        p35.chan4_raw_SET((char)32577) ;
        p35.chan6_raw_SET((char)18712) ;
        p35.chan1_raw_SET((char)60395) ;
        p35.chan2_raw_SET((char)4733) ;
        p35.chan8_raw_SET((char)3455) ;
        p35.chan5_raw_SET((char)50485) ;
        p35.chan3_raw_SET((char)7667) ;
        p35.rssi_SET((char)125) ;
        p35.port_SET((char)206) ;
        p35.time_boot_ms_SET(1525178409L) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo8_raw_GET() == (char)14728);
            assert(pack.servo1_raw_GET() == (char)536);
            assert(pack.servo12_raw_TRY(ph) == (char)27426);
            assert(pack.servo6_raw_GET() == (char)56672);
            assert(pack.servo4_raw_GET() == (char)51629);
            assert(pack.servo3_raw_GET() == (char)29756);
            assert(pack.time_usec_GET() == 35323192L);
            assert(pack.servo7_raw_GET() == (char)41367);
            assert(pack.servo2_raw_GET() == (char)21493);
            assert(pack.servo5_raw_GET() == (char)55018);
            assert(pack.servo9_raw_TRY(ph) == (char)38945);
            assert(pack.servo14_raw_TRY(ph) == (char)47929);
            assert(pack.servo10_raw_TRY(ph) == (char)24473);
            assert(pack.port_GET() == (char)115);
            assert(pack.servo15_raw_TRY(ph) == (char)45275);
            assert(pack.servo16_raw_TRY(ph) == (char)42974);
            assert(pack.servo13_raw_TRY(ph) == (char)27830);
            assert(pack.servo11_raw_TRY(ph) == (char)13130);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo14_raw_SET((char)47929, PH) ;
        p36.servo13_raw_SET((char)27830, PH) ;
        p36.servo1_raw_SET((char)536) ;
        p36.servo2_raw_SET((char)21493) ;
        p36.port_SET((char)115) ;
        p36.servo10_raw_SET((char)24473, PH) ;
        p36.servo8_raw_SET((char)14728) ;
        p36.servo7_raw_SET((char)41367) ;
        p36.servo3_raw_SET((char)29756) ;
        p36.servo4_raw_SET((char)51629) ;
        p36.servo5_raw_SET((char)55018) ;
        p36.servo11_raw_SET((char)13130, PH) ;
        p36.servo6_raw_SET((char)56672) ;
        p36.servo12_raw_SET((char)27426, PH) ;
        p36.servo15_raw_SET((char)45275, PH) ;
        p36.servo16_raw_SET((char)42974, PH) ;
        p36.time_usec_SET(35323192L) ;
        p36.servo9_raw_SET((char)38945, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)24810);
            assert(pack.start_index_GET() == (short)5997);
            assert(pack.target_component_GET() == (char)35);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)225);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_component_SET((char)35) ;
        p37.end_index_SET((short)24810) ;
        p37.target_system_SET((char)225) ;
        p37.start_index_SET((short)5997) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)188);
            assert(pack.target_component_GET() == (char)93);
            assert(pack.end_index_GET() == (short)31319);
            assert(pack.start_index_GET() == (short) -6485);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.end_index_SET((short)31319) ;
        p38.target_system_SET((char)188) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.target_component_SET((char)93) ;
        p38.start_index_SET((short) -6485) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.param4_GET() == 2.0723221E38F);
            assert(pack.param2_GET() == 1.6606899E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.z_GET() == -8.505233E37F);
            assert(pack.current_GET() == (char)23);
            assert(pack.param3_GET() == 3.2695828E38F);
            assert(pack.x_GET() == 1.7341278E38F);
            assert(pack.target_component_GET() == (char)63);
            assert(pack.y_GET() == -1.3939777E38F);
            assert(pack.param1_GET() == 1.2388754E37F);
            assert(pack.target_system_GET() == (char)185);
            assert(pack.autocontinue_GET() == (char)111);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT);
            assert(pack.seq_GET() == (char)41510);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        p39.param3_SET(3.2695828E38F) ;
        p39.current_SET((char)23) ;
        p39.target_component_SET((char)63) ;
        p39.param1_SET(1.2388754E37F) ;
        p39.y_SET(-1.3939777E38F) ;
        p39.target_system_SET((char)185) ;
        p39.param4_SET(2.0723221E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.param2_SET(1.6606899E38F) ;
        p39.autocontinue_SET((char)111) ;
        p39.x_SET(1.7341278E38F) ;
        p39.command_SET(MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT) ;
        p39.z_SET(-8.505233E37F) ;
        p39.seq_SET((char)41510) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)146);
            assert(pack.seq_GET() == (char)56603);
            assert(pack.target_component_GET() == (char)180);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.target_system_SET((char)146) ;
        p40.seq_SET((char)56603) ;
        p40.target_component_SET((char)180) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)19712);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.target_component_GET() == (char)50);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)232) ;
        p41.seq_SET((char)19712) ;
        p41.target_component_SET((char)50) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)6617);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)6617) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)42);
            assert(pack.target_component_GET() == (char)229);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p43.target_component_SET((char)229) ;
        p43.target_system_SET((char)42) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)183);
            assert(pack.count_GET() == (char)10387);
            assert(pack.target_system_GET() == (char)120);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_component_SET((char)183) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p44.count_SET((char)10387) ;
        p44.target_system_SET((char)120) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)177);
            assert(pack.target_system_GET() == (char)119);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)119) ;
        p45.target_component_SET((char)177) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)59541);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)59541) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)33);
            assert(pack.target_component_GET() == (char)218);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p47.target_component_SET((char)218) ;
        p47.target_system_SET((char)33) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 1106745982);
            assert(pack.target_system_GET() == (char)36);
            assert(pack.time_usec_TRY(ph) == 6743044762881333802L);
            assert(pack.latitude_GET() == 150588997);
            assert(pack.altitude_GET() == 2116979586);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.longitude_SET(1106745982) ;
        p48.altitude_SET(2116979586) ;
        p48.time_usec_SET(6743044762881333802L, PH) ;
        p48.latitude_SET(150588997) ;
        p48.target_system_SET((char)36) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == 1281425829);
            assert(pack.longitude_GET() == 2120409770);
            assert(pack.altitude_GET() == 1324226509);
            assert(pack.time_usec_TRY(ph) == 635262285891806067L);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(1281425829) ;
        p49.longitude_SET(2120409770) ;
        p49.time_usec_SET(635262285891806067L, PH) ;
        p49.altitude_SET(1324226509) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)14);
            assert(pack.parameter_rc_channel_index_GET() == (char)94);
            assert(pack.target_component_GET() == (char)113);
            assert(pack.param_index_GET() == (short) -7342);
            assert(pack.param_id_LEN(ph) == 7);
            assert(pack.param_id_TRY(ph).equals("lcikxgf"));
            assert(pack.scale_GET() == 3.1006438E38F);
            assert(pack.param_value_max_GET() == -3.4251818E37F);
            assert(pack.param_value_min_GET() == -2.9034898E38F);
            assert(pack.param_value0_GET() == 1.4162653E38F);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)94) ;
        p50.param_value_min_SET(-2.9034898E38F) ;
        p50.target_component_SET((char)113) ;
        p50.param_id_SET("lcikxgf", PH) ;
        p50.param_value0_SET(1.4162653E38F) ;
        p50.param_value_max_SET(-3.4251818E37F) ;
        p50.scale_SET(3.1006438E38F) ;
        p50.target_system_SET((char)14) ;
        p50.param_index_SET((short) -7342) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)26335);
            assert(pack.target_component_GET() == (char)79);
            assert(pack.target_system_GET() == (char)117);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_component_SET((char)79) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.seq_SET((char)26335) ;
        p51.target_system_SET((char)117) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 3.1867888E38F);
            assert(pack.p1x_GET() == 4.760759E37F);
            assert(pack.target_component_GET() == (char)59);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.p2z_GET() == -3.3111584E37F);
            assert(pack.p2x_GET() == -2.656766E38F);
            assert(pack.p1y_GET() == 1.9868271E38F);
            assert(pack.p1z_GET() == -7.6097076E37F);
            assert(pack.target_system_GET() == (char)117);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2z_SET(-3.3111584E37F) ;
        p54.p2y_SET(3.1867888E38F) ;
        p54.p1x_SET(4.760759E37F) ;
        p54.p1z_SET(-7.6097076E37F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p54.target_system_SET((char)117) ;
        p54.p1y_SET(1.9868271E38F) ;
        p54.p2x_SET(-2.656766E38F) ;
        p54.target_component_SET((char)59) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.p1z_GET() == -2.435213E38F);
            assert(pack.p2z_GET() == 1.745383E38F);
            assert(pack.p2x_GET() == -7.2450937E37F);
            assert(pack.p2y_GET() == 1.796046E38F);
            assert(pack.p1y_GET() == 2.194819E38F);
            assert(pack.p1x_GET() == 8.1927726E37F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2z_SET(1.745383E38F) ;
        p55.p1y_SET(2.194819E38F) ;
        p55.p2x_SET(-7.2450937E37F) ;
        p55.p1z_SET(-2.435213E38F) ;
        p55.p1x_SET(8.1927726E37F) ;
        p55.p2y_SET(1.796046E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.166767E38F);
            assert(pack.pitchspeed_GET() == -3.5612258E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.9702035E38F, -2.512625E38F, 1.512107E38F, -6.282604E37F}));
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.4719588E38F, 6.701449E37F, -2.7305427E38F, 1.7916594E38F, 3.1152739E38F, 1.5302597E38F, 2.0667013E38F, 2.2165457E38F, 3.3246814E38F}));
            assert(pack.rollspeed_GET() == -1.0430437E38F);
            assert(pack.time_usec_GET() == 418048736757033879L);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.rollspeed_SET(-1.0430437E38F) ;
        p61.q_SET(new float[] {1.9702035E38F, -2.512625E38F, 1.512107E38F, -6.282604E37F}, 0) ;
        p61.covariance_SET(new float[] {2.4719588E38F, 6.701449E37F, -2.7305427E38F, 1.7916594E38F, 3.1152739E38F, 1.5302597E38F, 2.0667013E38F, 2.2165457E38F, 3.3246814E38F}, 0) ;
        p61.time_usec_SET(418048736757033879L) ;
        p61.pitchspeed_SET(-3.5612258E37F) ;
        p61.yawspeed_SET(2.166767E38F) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.aspd_error_GET() == 3.005151E38F);
            assert(pack.nav_pitch_GET() == 6.762325E37F);
            assert(pack.alt_error_GET() == 1.9121795E38F);
            assert(pack.nav_bearing_GET() == (short)24522);
            assert(pack.nav_roll_GET() == -1.5312483E38F);
            assert(pack.wp_dist_GET() == (char)31621);
            assert(pack.target_bearing_GET() == (short) -1219);
            assert(pack.xtrack_error_GET() == -1.5987479E38F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.xtrack_error_SET(-1.5987479E38F) ;
        p62.nav_bearing_SET((short)24522) ;
        p62.aspd_error_SET(3.005151E38F) ;
        p62.target_bearing_SET((short) -1219) ;
        p62.nav_pitch_SET(6.762325E37F) ;
        p62.alt_error_SET(1.9121795E38F) ;
        p62.wp_dist_SET((char)31621) ;
        p62.nav_roll_SET(-1.5312483E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -2036091264);
            assert(pack.vy_GET() == -7.5027564E37F);
            assert(pack.alt_GET() == -101416476);
            assert(pack.relative_alt_GET() == 829440865);
            assert(pack.vx_GET() == 2.2211883E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.0484167E38F, 2.9699005E38F, -1.24977E38F, -2.8617058E38F, 2.6793262E38F, 1.7830603E38F, -2.9640265E37F, 2.8621277E38F, -1.7929074E38F, -1.3103036E38F, 2.6705303E38F, 2.7878184E38F, 3.5274845E37F, 2.6724225E38F, -3.3148692E38F, 5.4642414E37F, 2.840719E38F, -3.3656543E38F, -2.5804886E38F, -1.4585793E38F, 1.3966224E38F, 9.57351E37F, 1.3446799E38F, -1.395675E38F, -3.146513E37F, -3.1945945E38F, -7.413622E37F, -1.2376041E38F, -1.5964497E38F, -3.080566E38F, 2.8641529E38F, -2.106826E38F, -1.4541207E38F, -2.7457705E38F, 7.093242E37F, -2.1542767E38F}));
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.vz_GET() == -1.3184541E38F);
            assert(pack.lat_GET() == 1195848577);
            assert(pack.time_usec_GET() == 2215717821350001154L);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p63.vy_SET(-7.5027564E37F) ;
        p63.time_usec_SET(2215717821350001154L) ;
        p63.lon_SET(-2036091264) ;
        p63.lat_SET(1195848577) ;
        p63.alt_SET(-101416476) ;
        p63.vx_SET(2.2211883E38F) ;
        p63.covariance_SET(new float[] {-1.0484167E38F, 2.9699005E38F, -1.24977E38F, -2.8617058E38F, 2.6793262E38F, 1.7830603E38F, -2.9640265E37F, 2.8621277E38F, -1.7929074E38F, -1.3103036E38F, 2.6705303E38F, 2.7878184E38F, 3.5274845E37F, 2.6724225E38F, -3.3148692E38F, 5.4642414E37F, 2.840719E38F, -3.3656543E38F, -2.5804886E38F, -1.4585793E38F, 1.3966224E38F, 9.57351E37F, 1.3446799E38F, -1.395675E38F, -3.146513E37F, -3.1945945E38F, -7.413622E37F, -1.2376041E38F, -1.5964497E38F, -3.080566E38F, 2.8641529E38F, -2.106826E38F, -1.4541207E38F, -2.7457705E38F, 7.093242E37F, -2.1542767E38F}, 0) ;
        p63.relative_alt_SET(829440865) ;
        p63.vz_SET(-1.3184541E38F) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 6.9763336E37F);
            assert(pack.vx_GET() == -3.0844195E38F);
            assert(pack.az_GET() == -5.8635675E37F);
            assert(pack.ay_GET() == 5.981932E37F);
            assert(pack.time_usec_GET() == 5186433996708691571L);
            assert(pack.y_GET() == 1.9827246E38F);
            assert(pack.z_GET() == 1.282672E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {3.0199628E38F, 3.095628E38F, -6.4425436E37F, -2.04147E38F, 2.7527589E38F, -2.4857823E38F, -3.1002355E38F, -1.3477279E38F, -1.860584E37F, 2.3877073E38F, -1.4339867E38F, 1.1293588E38F, -2.9960113E38F, -2.9695312E38F, -3.008943E38F, -1.8382835E37F, -2.0200633E38F, -3.1168001E38F, -3.3720248E38F, 1.0285136E38F, 3.2604779E38F, -2.6994021E38F, -2.7797095E38F, 3.9383458E37F, 1.7771328E38F, -1.4930953E37F, 3.3961368E38F, -3.36937E36F, 1.634854E38F, 2.6199626E38F, -2.6280496E38F, -1.4336416E38F, 2.968969E38F, -2.8442636E38F, -1.2778087E38F, -3.318012E38F, 2.8858573E38F, -2.73402E38F, -3.7449738E37F, -3.4137162E36F, -2.9945361E38F, 3.3877626E38F, -1.6261121E38F, -1.9739042E38F, -4.685539E37F}));
            assert(pack.vz_GET() == -3.0375726E38F);
            assert(pack.ax_GET() == -2.1558694E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            assert(pack.x_GET() == -5.361187E37F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(5186433996708691571L) ;
        p64.y_SET(1.9827246E38F) ;
        p64.ay_SET(5.981932E37F) ;
        p64.x_SET(-5.361187E37F) ;
        p64.ax_SET(-2.1558694E38F) ;
        p64.vy_SET(6.9763336E37F) ;
        p64.vx_SET(-3.0844195E38F) ;
        p64.covariance_SET(new float[] {3.0199628E38F, 3.095628E38F, -6.4425436E37F, -2.04147E38F, 2.7527589E38F, -2.4857823E38F, -3.1002355E38F, -1.3477279E38F, -1.860584E37F, 2.3877073E38F, -1.4339867E38F, 1.1293588E38F, -2.9960113E38F, -2.9695312E38F, -3.008943E38F, -1.8382835E37F, -2.0200633E38F, -3.1168001E38F, -3.3720248E38F, 1.0285136E38F, 3.2604779E38F, -2.6994021E38F, -2.7797095E38F, 3.9383458E37F, 1.7771328E38F, -1.4930953E37F, 3.3961368E38F, -3.36937E36F, 1.634854E38F, 2.6199626E38F, -2.6280496E38F, -1.4336416E38F, 2.968969E38F, -2.8442636E38F, -1.2778087E38F, -3.318012E38F, 2.8858573E38F, -2.73402E38F, -3.7449738E37F, -3.4137162E36F, -2.9945361E38F, 3.3877626E38F, -1.6261121E38F, -1.9739042E38F, -4.685539E37F}, 0) ;
        p64.z_SET(1.282672E38F) ;
        p64.vz_SET(-3.0375726E38F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO) ;
        p64.az_SET(-5.8635675E37F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan11_raw_GET() == (char)22320);
            assert(pack.chan8_raw_GET() == (char)24198);
            assert(pack.chan9_raw_GET() == (char)44626);
            assert(pack.time_boot_ms_GET() == 2951606710L);
            assert(pack.chan10_raw_GET() == (char)43286);
            assert(pack.chan5_raw_GET() == (char)22423);
            assert(pack.chan18_raw_GET() == (char)56642);
            assert(pack.chan13_raw_GET() == (char)8970);
            assert(pack.chan1_raw_GET() == (char)55029);
            assert(pack.chan17_raw_GET() == (char)54607);
            assert(pack.chancount_GET() == (char)162);
            assert(pack.chan16_raw_GET() == (char)35745);
            assert(pack.rssi_GET() == (char)194);
            assert(pack.chan14_raw_GET() == (char)51109);
            assert(pack.chan6_raw_GET() == (char)20021);
            assert(pack.chan4_raw_GET() == (char)56305);
            assert(pack.chan7_raw_GET() == (char)65433);
            assert(pack.chan12_raw_GET() == (char)33621);
            assert(pack.chan2_raw_GET() == (char)22483);
            assert(pack.chan15_raw_GET() == (char)2510);
            assert(pack.chan3_raw_GET() == (char)63248);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan15_raw_SET((char)2510) ;
        p65.chan3_raw_SET((char)63248) ;
        p65.chan10_raw_SET((char)43286) ;
        p65.chan2_raw_SET((char)22483) ;
        p65.chan8_raw_SET((char)24198) ;
        p65.chan1_raw_SET((char)55029) ;
        p65.chan17_raw_SET((char)54607) ;
        p65.chan7_raw_SET((char)65433) ;
        p65.chancount_SET((char)162) ;
        p65.chan14_raw_SET((char)51109) ;
        p65.chan9_raw_SET((char)44626) ;
        p65.chan4_raw_SET((char)56305) ;
        p65.time_boot_ms_SET(2951606710L) ;
        p65.chan13_raw_SET((char)8970) ;
        p65.chan18_raw_SET((char)56642) ;
        p65.chan12_raw_SET((char)33621) ;
        p65.chan6_raw_SET((char)20021) ;
        p65.chan11_raw_SET((char)22320) ;
        p65.chan16_raw_SET((char)35745) ;
        p65.rssi_SET((char)194) ;
        p65.chan5_raw_SET((char)22423) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.start_stop_GET() == (char)39);
            assert(pack.req_message_rate_GET() == (char)63844);
            assert(pack.req_stream_id_GET() == (char)177);
            assert(pack.target_component_GET() == (char)139);
            assert(pack.target_system_GET() == (char)200);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)177) ;
        p66.target_component_SET((char)139) ;
        p66.target_system_SET((char)200) ;
        p66.req_message_rate_SET((char)63844) ;
        p66.start_stop_SET((char)39) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)207);
            assert(pack.stream_id_GET() == (char)147);
            assert(pack.message_rate_GET() == (char)64011);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)147) ;
        p67.on_off_SET((char)207) ;
        p67.message_rate_SET((char)64011) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)129);
            assert(pack.z_GET() == (short)979);
            assert(pack.x_GET() == (short)1424);
            assert(pack.buttons_GET() == (char)64766);
            assert(pack.y_GET() == (short)28791);
            assert(pack.r_GET() == (short)17413);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.z_SET((short)979) ;
        p69.x_SET((short)1424) ;
        p69.r_SET((short)17413) ;
        p69.y_SET((short)28791) ;
        p69.buttons_SET((char)64766) ;
        p69.target_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)5594);
            assert(pack.target_component_GET() == (char)135);
            assert(pack.chan3_raw_GET() == (char)27001);
            assert(pack.chan4_raw_GET() == (char)15218);
            assert(pack.chan6_raw_GET() == (char)5955);
            assert(pack.chan2_raw_GET() == (char)11732);
            assert(pack.chan7_raw_GET() == (char)19282);
            assert(pack.chan5_raw_GET() == (char)43904);
            assert(pack.chan1_raw_GET() == (char)25183);
            assert(pack.target_system_GET() == (char)250);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan5_raw_SET((char)43904) ;
        p70.chan3_raw_SET((char)27001) ;
        p70.chan2_raw_SET((char)11732) ;
        p70.chan4_raw_SET((char)15218) ;
        p70.target_component_SET((char)135) ;
        p70.chan1_raw_SET((char)25183) ;
        p70.chan7_raw_SET((char)19282) ;
        p70.chan6_raw_SET((char)5955) ;
        p70.target_system_SET((char)250) ;
        p70.chan8_raw_SET((char)5594) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param1_GET() == -2.6064967E38F);
            assert(pack.autocontinue_GET() == (char)123);
            assert(pack.param2_GET() == 3.0655371E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)218);
            assert(pack.y_GET() == 500812297);
            assert(pack.current_GET() == (char)141);
            assert(pack.param4_GET() == -2.8164862E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.x_GET() == -1551994312);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_PARAMETER);
            assert(pack.z_GET() == -2.320676E38F);
            assert(pack.target_component_GET() == (char)118);
            assert(pack.param3_GET() == 6.6791177E37F);
            assert(pack.seq_GET() == (char)41760);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param4_SET(-2.8164862E38F) ;
        p73.target_system_SET((char)218) ;
        p73.current_SET((char)141) ;
        p73.seq_SET((char)41760) ;
        p73.param2_SET(3.0655371E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p73.param3_SET(6.6791177E37F) ;
        p73.z_SET(-2.320676E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_PARAMETER) ;
        p73.param1_SET(-2.6064967E38F) ;
        p73.autocontinue_SET((char)123) ;
        p73.x_SET(-1551994312) ;
        p73.y_SET(500812297) ;
        p73.target_component_SET((char)118) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 1.85309E38F);
            assert(pack.climb_GET() == -2.0590073E38F);
            assert(pack.groundspeed_GET() == -1.9531679E38F);
            assert(pack.airspeed_GET() == -2.1747029E38F);
            assert(pack.heading_GET() == (short) -13569);
            assert(pack.throttle_GET() == (char)42885);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(1.85309E38F) ;
        p74.throttle_SET((char)42885) ;
        p74.climb_SET(-2.0590073E38F) ;
        p74.groundspeed_SET(-1.9531679E38F) ;
        p74.airspeed_SET(-2.1747029E38F) ;
        p74.heading_SET((short) -13569) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -1420155306);
            assert(pack.z_GET() == -3.3719622E38F);
            assert(pack.target_component_GET() == (char)46);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE);
            assert(pack.autocontinue_GET() == (char)54);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.current_GET() == (char)80);
            assert(pack.x_GET() == -763052912);
            assert(pack.param2_GET() == -2.8651104E38F);
            assert(pack.param3_GET() == 1.0445602E38F);
            assert(pack.param1_GET() == 3.011566E38F);
            assert(pack.param4_GET() == 7.0686966E37F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.param1_SET(3.011566E38F) ;
        p75.param4_SET(7.0686966E37F) ;
        p75.z_SET(-3.3719622E38F) ;
        p75.command_SET(MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE) ;
        p75.target_component_SET((char)46) ;
        p75.x_SET(-763052912) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p75.param3_SET(1.0445602E38F) ;
        p75.current_SET((char)80) ;
        p75.param2_SET(-2.8651104E38F) ;
        p75.target_system_SET((char)27) ;
        p75.autocontinue_SET((char)54) ;
        p75.y_SET(-1420155306) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param6_GET() == -1.0750461E38F);
            assert(pack.param2_GET() == -1.4486003E38F);
            assert(pack.target_system_GET() == (char)191);
            assert(pack.param5_GET() == 2.7757532E37F);
            assert(pack.param7_GET() == -2.7589557E38F);
            assert(pack.param1_GET() == -2.5284829E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LAND_LOCAL);
            assert(pack.confirmation_GET() == (char)190);
            assert(pack.target_component_GET() == (char)119);
            assert(pack.param3_GET() == 1.2184453E38F);
            assert(pack.param4_GET() == 6.4749255E37F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.param7_SET(-2.7589557E38F) ;
        p76.param2_SET(-1.4486003E38F) ;
        p76.param5_SET(2.7757532E37F) ;
        p76.target_system_SET((char)191) ;
        p76.confirmation_SET((char)190) ;
        p76.param4_SET(6.4749255E37F) ;
        p76.param3_SET(1.2184453E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_LAND_LOCAL) ;
        p76.param6_SET(-1.0750461E38F) ;
        p76.target_component_SET((char)119) ;
        p76.param1_SET(-2.5284829E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)210);
            assert(pack.result_param2_TRY(ph) == -1567401936);
            assert(pack.progress_TRY(ph) == (char)141);
            assert(pack.target_component_TRY(ph) == (char)111);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_MODE);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)111, PH) ;
        p77.progress_SET((char)141, PH) ;
        p77.target_system_SET((char)210, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.command_SET(MAV_CMD.MAV_CMD_DO_SET_MODE) ;
        p77.result_param2_SET(-1567401936, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -2.4427763E38F);
            assert(pack.time_boot_ms_GET() == 461330315L);
            assert(pack.thrust_GET() == 1.0629331E38F);
            assert(pack.manual_override_switch_GET() == (char)159);
            assert(pack.roll_GET() == -4.428078E37F);
            assert(pack.yaw_GET() == -8.1524664E37F);
            assert(pack.mode_switch_GET() == (char)56);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(-8.1524664E37F) ;
        p81.thrust_SET(1.0629331E38F) ;
        p81.manual_override_switch_SET((char)159) ;
        p81.time_boot_ms_SET(461330315L) ;
        p81.roll_SET(-4.428078E37F) ;
        p81.mode_switch_SET((char)56) ;
        p81.pitch_SET(-2.4427763E38F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.240029E38F, 2.7281634E38F, -3.380519E38F, 8.791453E37F}));
            assert(pack.body_pitch_rate_GET() == -1.825675E38F);
            assert(pack.body_yaw_rate_GET() == -1.2356683E38F);
            assert(pack.time_boot_ms_GET() == 980955193L);
            assert(pack.body_roll_rate_GET() == 2.6100376E38F);
            assert(pack.target_component_GET() == (char)112);
            assert(pack.thrust_GET() == 2.7040993E38F);
            assert(pack.target_system_GET() == (char)146);
            assert(pack.type_mask_GET() == (char)22);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.target_system_SET((char)146) ;
        p82.thrust_SET(2.7040993E38F) ;
        p82.body_roll_rate_SET(2.6100376E38F) ;
        p82.time_boot_ms_SET(980955193L) ;
        p82.q_SET(new float[] {-2.240029E38F, 2.7281634E38F, -3.380519E38F, 8.791453E37F}, 0) ;
        p82.body_pitch_rate_SET(-1.825675E38F) ;
        p82.target_component_SET((char)112) ;
        p82.body_yaw_rate_SET(-1.2356683E38F) ;
        p82.type_mask_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.3567286E38F, -3.1382498E38F, -3.3946127E38F, 2.6123654E38F}));
            assert(pack.thrust_GET() == 1.4525889E37F);
            assert(pack.body_pitch_rate_GET() == -2.1961292E38F);
            assert(pack.body_roll_rate_GET() == -2.9451479E38F);
            assert(pack.type_mask_GET() == (char)159);
            assert(pack.time_boot_ms_GET() == 200834617L);
            assert(pack.body_yaw_rate_GET() == -2.6761102E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.body_pitch_rate_SET(-2.1961292E38F) ;
        p83.time_boot_ms_SET(200834617L) ;
        p83.q_SET(new float[] {-3.3567286E38F, -3.1382498E38F, -3.3946127E38F, 2.6123654E38F}, 0) ;
        p83.body_yaw_rate_SET(-2.6761102E38F) ;
        p83.type_mask_SET((char)159) ;
        p83.body_roll_rate_SET(-2.9451479E38F) ;
        p83.thrust_SET(1.4525889E37F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 2.01515E38F);
            assert(pack.z_GET() == -1.2214552E38F);
            assert(pack.target_component_GET() == (char)11);
            assert(pack.afx_GET() == -5.4160823E37F);
            assert(pack.time_boot_ms_GET() == 1453851710L);
            assert(pack.afy_GET() == -2.5903596E38F);
            assert(pack.afz_GET() == 1.8575854E38F);
            assert(pack.vx_GET() == -2.4968192E38F);
            assert(pack.target_system_GET() == (char)199);
            assert(pack.vy_GET() == -1.0046922E38F);
            assert(pack.vz_GET() == 7.870741E37F);
            assert(pack.y_GET() == -1.1623523E38F);
            assert(pack.yaw_GET() == 1.2661545E38F);
            assert(pack.x_GET() == 2.937423E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.type_mask_GET() == (char)10558);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.z_SET(-1.2214552E38F) ;
        p84.target_component_SET((char)11) ;
        p84.vz_SET(7.870741E37F) ;
        p84.vy_SET(-1.0046922E38F) ;
        p84.yaw_rate_SET(2.01515E38F) ;
        p84.afy_SET(-2.5903596E38F) ;
        p84.vx_SET(-2.4968192E38F) ;
        p84.afz_SET(1.8575854E38F) ;
        p84.x_SET(2.937423E38F) ;
        p84.time_boot_ms_SET(1453851710L) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p84.yaw_SET(1.2661545E38F) ;
        p84.target_system_SET((char)199) ;
        p84.type_mask_SET((char)10558) ;
        p84.y_SET(-1.1623523E38F) ;
        p84.afx_SET(-5.4160823E37F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vz_GET() == 3.2065704E37F);
            assert(pack.afz_GET() == -1.1338226E38F);
            assert(pack.time_boot_ms_GET() == 1070145430L);
            assert(pack.vy_GET() == -2.408333E38F);
            assert(pack.afx_GET() == 1.1570799E38F);
            assert(pack.type_mask_GET() == (char)12629);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.lon_int_GET() == 1415803345);
            assert(pack.target_system_GET() == (char)134);
            assert(pack.yaw_rate_GET() == -1.2618109E38F);
            assert(pack.alt_GET() == 1.5717729E38F);
            assert(pack.vx_GET() == -1.6972411E38F);
            assert(pack.yaw_GET() == -3.0012316E36F);
            assert(pack.lat_int_GET() == -153411342);
            assert(pack.target_component_GET() == (char)202);
            assert(pack.afy_GET() == -1.3236552E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.target_system_SET((char)134) ;
        p86.type_mask_SET((char)12629) ;
        p86.yaw_SET(-3.0012316E36F) ;
        p86.target_component_SET((char)202) ;
        p86.afx_SET(1.1570799E38F) ;
        p86.afz_SET(-1.1338226E38F) ;
        p86.alt_SET(1.5717729E38F) ;
        p86.lon_int_SET(1415803345) ;
        p86.vz_SET(3.2065704E37F) ;
        p86.vx_SET(-1.6972411E38F) ;
        p86.yaw_rate_SET(-1.2618109E38F) ;
        p86.vy_SET(-2.408333E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p86.afy_SET(-1.3236552E38F) ;
        p86.lat_int_SET(-153411342) ;
        p86.time_boot_ms_SET(1070145430L) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -3.3782937E38F);
            assert(pack.lon_int_GET() == 2082969114);
            assert(pack.lat_int_GET() == -1489417245);
            assert(pack.afz_GET() == 1.4144213E37F);
            assert(pack.afx_GET() == -1.4598528E38F);
            assert(pack.afy_GET() == 7.396958E37F);
            assert(pack.yaw_GET() == -7.639349E37F);
            assert(pack.alt_GET() == -2.6902844E38F);
            assert(pack.time_boot_ms_GET() == 1689811011L);
            assert(pack.vy_GET() == -1.3317151E36F);
            assert(pack.vx_GET() == -6.6915077E37F);
            assert(pack.type_mask_GET() == (char)30284);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.vz_GET() == -3.2660006E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vz_SET(-3.2660006E38F) ;
        p87.type_mask_SET((char)30284) ;
        p87.yaw_rate_SET(-3.3782937E38F) ;
        p87.afz_SET(1.4144213E37F) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p87.lat_int_SET(-1489417245) ;
        p87.afx_SET(-1.4598528E38F) ;
        p87.afy_SET(7.396958E37F) ;
        p87.time_boot_ms_SET(1689811011L) ;
        p87.vy_SET(-1.3317151E36F) ;
        p87.lon_int_SET(2082969114) ;
        p87.vx_SET(-6.6915077E37F) ;
        p87.alt_SET(-2.6902844E38F) ;
        p87.yaw_SET(-7.639349E37F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.1900848E38F);
            assert(pack.yaw_GET() == 2.3967046E38F);
            assert(pack.y_GET() == -2.2906578E38F);
            assert(pack.x_GET() == 3.1002732E38F);
            assert(pack.roll_GET() == -2.52222E38F);
            assert(pack.time_boot_ms_GET() == 3656203499L);
            assert(pack.z_GET() == -1.0216806E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.yaw_SET(2.3967046E38F) ;
        p89.y_SET(-2.2906578E38F) ;
        p89.pitch_SET(1.1900848E38F) ;
        p89.roll_SET(-2.52222E38F) ;
        p89.time_boot_ms_SET(3656203499L) ;
        p89.x_SET(3.1002732E38F) ;
        p89.z_SET(-1.0216806E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -3.20766E38F);
            assert(pack.yaw_GET() == -2.3282683E38F);
            assert(pack.yacc_GET() == (short) -30795);
            assert(pack.roll_GET() == -9.5567234E36F);
            assert(pack.vy_GET() == (short) -16167);
            assert(pack.pitchspeed_GET() == 2.565762E38F);
            assert(pack.rollspeed_GET() == -2.1757513E38F);
            assert(pack.yawspeed_GET() == -1.0412558E38F);
            assert(pack.lat_GET() == -540942336);
            assert(pack.zacc_GET() == (short)20872);
            assert(pack.time_usec_GET() == 34977098557735066L);
            assert(pack.alt_GET() == -484891358);
            assert(pack.vz_GET() == (short) -5366);
            assert(pack.xacc_GET() == (short) -7739);
            assert(pack.lon_GET() == 1443864781);
            assert(pack.vx_GET() == (short)4043);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lon_SET(1443864781) ;
        p90.rollspeed_SET(-2.1757513E38F) ;
        p90.pitch_SET(-3.20766E38F) ;
        p90.xacc_SET((short) -7739) ;
        p90.vx_SET((short)4043) ;
        p90.yacc_SET((short) -30795) ;
        p90.zacc_SET((short)20872) ;
        p90.yawspeed_SET(-1.0412558E38F) ;
        p90.vy_SET((short) -16167) ;
        p90.pitchspeed_SET(2.565762E38F) ;
        p90.vz_SET((short) -5366) ;
        p90.lat_SET(-540942336) ;
        p90.time_usec_SET(34977098557735066L) ;
        p90.alt_SET(-484891358) ;
        p90.yaw_SET(-2.3282683E38F) ;
        p90.roll_SET(-9.5567234E36F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux3_GET() == 7.104294E37F);
            assert(pack.aux2_GET() == 6.973685E37F);
            assert(pack.throttle_GET() == 1.0767172E38F);
            assert(pack.aux4_GET() == 3.0375393E38F);
            assert(pack.pitch_elevator_GET() == -1.6011885E38F);
            assert(pack.aux1_GET() == 3.3135748E38F);
            assert(pack.roll_ailerons_GET() == 1.581137E38F);
            assert(pack.yaw_rudder_GET() == -3.0963307E38F);
            assert(pack.time_usec_GET() == 5861664026584490219L);
            assert(pack.nav_mode_GET() == (char)40);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux3_SET(7.104294E37F) ;
        p91.aux2_SET(6.973685E37F) ;
        p91.aux1_SET(3.3135748E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        p91.pitch_elevator_SET(-1.6011885E38F) ;
        p91.nav_mode_SET((char)40) ;
        p91.throttle_SET(1.0767172E38F) ;
        p91.time_usec_SET(5861664026584490219L) ;
        p91.yaw_rudder_SET(-3.0963307E38F) ;
        p91.aux4_SET(3.0375393E38F) ;
        p91.roll_ailerons_SET(1.581137E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan12_raw_GET() == (char)3023);
            assert(pack.time_usec_GET() == 7006972822405281560L);
            assert(pack.chan7_raw_GET() == (char)16726);
            assert(pack.chan5_raw_GET() == (char)9638);
            assert(pack.chan10_raw_GET() == (char)15370);
            assert(pack.chan9_raw_GET() == (char)11099);
            assert(pack.chan4_raw_GET() == (char)52395);
            assert(pack.chan2_raw_GET() == (char)59131);
            assert(pack.chan11_raw_GET() == (char)10190);
            assert(pack.chan6_raw_GET() == (char)62182);
            assert(pack.chan3_raw_GET() == (char)14106);
            assert(pack.chan8_raw_GET() == (char)39957);
            assert(pack.rssi_GET() == (char)168);
            assert(pack.chan1_raw_GET() == (char)4364);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan12_raw_SET((char)3023) ;
        p92.rssi_SET((char)168) ;
        p92.chan11_raw_SET((char)10190) ;
        p92.time_usec_SET(7006972822405281560L) ;
        p92.chan10_raw_SET((char)15370) ;
        p92.chan9_raw_SET((char)11099) ;
        p92.chan8_raw_SET((char)39957) ;
        p92.chan5_raw_SET((char)9638) ;
        p92.chan7_raw_SET((char)16726) ;
        p92.chan1_raw_SET((char)4364) ;
        p92.chan3_raw_SET((char)14106) ;
        p92.chan4_raw_SET((char)52395) ;
        p92.chan2_raw_SET((char)59131) ;
        p92.chan6_raw_SET((char)62182) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_PREFLIGHT);
            assert(pack.time_usec_GET() == 3029913443001934968L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.7336562E38F, 1.3252699E37F, 2.9445459E38F, -3.1935926E38F, 1.809602E38F, 7.4701296E37F, -3.000068E37F, 2.1047998E38F, -1.0401988E38F, 2.7092376E37F, -2.1283695E38F, 1.1880322E38F, -2.8749575E38F, -1.825834E38F, -8.134685E37F, 1.6500013E38F}));
            assert(pack.flags_GET() == 5597624303933155016L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {1.7336562E38F, 1.3252699E37F, 2.9445459E38F, -3.1935926E38F, 1.809602E38F, 7.4701296E37F, -3.000068E37F, 2.1047998E38F, -1.0401988E38F, 2.7092376E37F, -2.1283695E38F, 1.1880322E38F, -2.8749575E38F, -1.825834E38F, -8.134685E37F, 1.6500013E38F}, 0) ;
        p93.flags_SET(5597624303933155016L) ;
        p93.time_usec_SET(3029913443001934968L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_y_TRY(ph) == -3.386154E38F);
            assert(pack.quality_GET() == (char)26);
            assert(pack.flow_y_GET() == (short) -31148);
            assert(pack.flow_comp_m_x_GET() == 2.5193807E38F);
            assert(pack.sensor_id_GET() == (char)53);
            assert(pack.flow_x_GET() == (short)23144);
            assert(pack.flow_rate_x_TRY(ph) == 1.8815135E38F);
            assert(pack.ground_distance_GET() == 1.7758452E38F);
            assert(pack.time_usec_GET() == 8415354935564951378L);
            assert(pack.flow_comp_m_y_GET() == 1.7057387E38F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.ground_distance_SET(1.7758452E38F) ;
        p100.flow_rate_y_SET(-3.386154E38F, PH) ;
        p100.flow_comp_m_x_SET(2.5193807E38F) ;
        p100.flow_y_SET((short) -31148) ;
        p100.quality_SET((char)26) ;
        p100.flow_x_SET((short)23144) ;
        p100.time_usec_SET(8415354935564951378L) ;
        p100.flow_rate_x_SET(1.8815135E38F, PH) ;
        p100.flow_comp_m_y_SET(1.7057387E38F) ;
        p100.sensor_id_SET((char)53) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == 2.462452E38F);
            assert(pack.yaw_GET() == 1.0227726E38F);
            assert(pack.y_GET() == 3.1691576E36F);
            assert(pack.roll_GET() == 2.3947822E38F);
            assert(pack.pitch_GET() == 8.4417625E37F);
            assert(pack.x_GET() == -3.0529906E38F);
            assert(pack.usec_GET() == 8953049614000559202L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(2.462452E38F) ;
        p101.pitch_SET(8.4417625E37F) ;
        p101.yaw_SET(1.0227726E38F) ;
        p101.usec_SET(8953049614000559202L) ;
        p101.x_SET(-3.0529906E38F) ;
        p101.y_SET(3.1691576E36F) ;
        p101.roll_SET(2.3947822E38F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -3.119736E38F);
            assert(pack.pitch_GET() == -7.5033324E37F);
            assert(pack.x_GET() == -2.9496493E38F);
            assert(pack.z_GET() == 1.2824406E38F);
            assert(pack.roll_GET() == -1.00843703E37F);
            assert(pack.usec_GET() == 3656034399412647265L);
            assert(pack.y_GET() == 1.8411911E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(3656034399412647265L) ;
        p102.z_SET(1.2824406E38F) ;
        p102.x_SET(-2.9496493E38F) ;
        p102.pitch_SET(-7.5033324E37F) ;
        p102.y_SET(1.8411911E38F) ;
        p102.yaw_SET(-3.119736E38F) ;
        p102.roll_SET(-1.00843703E37F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 380539224549781046L);
            assert(pack.z_GET() == -2.4356156E38F);
            assert(pack.y_GET() == 2.2883998E38F);
            assert(pack.x_GET() == 2.092108E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.2883998E38F) ;
        p103.x_SET(2.092108E38F) ;
        p103.usec_SET(380539224549781046L) ;
        p103.z_SET(-2.4356156E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 4919420064568882881L);
            assert(pack.x_GET() == 1.9283627E38F);
            assert(pack.yaw_GET() == 1.2228988E38F);
            assert(pack.y_GET() == 3.008697E38F);
            assert(pack.z_GET() == -6.5070615E37F);
            assert(pack.roll_GET() == 3.3416986E38F);
            assert(pack.pitch_GET() == 1.3780242E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(1.2228988E38F) ;
        p104.roll_SET(3.3416986E38F) ;
        p104.x_SET(1.9283627E38F) ;
        p104.z_SET(-6.5070615E37F) ;
        p104.pitch_SET(1.3780242E38F) ;
        p104.usec_SET(4919420064568882881L) ;
        p104.y_SET(3.008697E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == -6.5668707E37F);
            assert(pack.xacc_GET() == -4.4511447E37F);
            assert(pack.zgyro_GET() == 1.6387209E38F);
            assert(pack.temperature_GET() == 2.9689314E38F);
            assert(pack.ymag_GET() == -8.0006525E37F);
            assert(pack.time_usec_GET() == 6504944386141681023L);
            assert(pack.fields_updated_GET() == (char)45450);
            assert(pack.diff_pressure_GET() == 1.4967807E37F);
            assert(pack.abs_pressure_GET() == 1.3511287E38F);
            assert(pack.zmag_GET() == 2.6596364E37F);
            assert(pack.ygyro_GET() == -1.1748627E38F);
            assert(pack.yacc_GET() == 9.361915E37F);
            assert(pack.pressure_alt_GET() == -2.0693631E38F);
            assert(pack.zacc_GET() == -2.7397467E38F);
            assert(pack.xgyro_GET() == 5.31573E36F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(6504944386141681023L) ;
        p105.pressure_alt_SET(-2.0693631E38F) ;
        p105.temperature_SET(2.9689314E38F) ;
        p105.zacc_SET(-2.7397467E38F) ;
        p105.ymag_SET(-8.0006525E37F) ;
        p105.xgyro_SET(5.31573E36F) ;
        p105.xmag_SET(-6.5668707E37F) ;
        p105.zmag_SET(2.6596364E37F) ;
        p105.ygyro_SET(-1.1748627E38F) ;
        p105.zgyro_SET(1.6387209E38F) ;
        p105.abs_pressure_SET(1.3511287E38F) ;
        p105.fields_updated_SET((char)45450) ;
        p105.diff_pressure_SET(1.4967807E37F) ;
        p105.yacc_SET(9.361915E37F) ;
        p105.xacc_SET(-4.4511447E37F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.time_delta_distance_us_GET() == 3400093612L);
            assert(pack.distance_GET() == -1.0593151E38F);
            assert(pack.integrated_xgyro_GET() == -7.7949843E37F);
            assert(pack.integrated_zgyro_GET() == -2.5558599E38F);
            assert(pack.temperature_GET() == (short)5987);
            assert(pack.integrated_y_GET() == 1.9726783E38F);
            assert(pack.integration_time_us_GET() == 2783671494L);
            assert(pack.integrated_x_GET() == 2.9017034E37F);
            assert(pack.integrated_ygyro_GET() == -5.2183027E36F);
            assert(pack.time_usec_GET() == 8217837453483654245L);
            assert(pack.quality_GET() == (char)173);
            assert(pack.sensor_id_GET() == (char)166);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.temperature_SET((short)5987) ;
        p106.integrated_x_SET(2.9017034E37F) ;
        p106.integrated_y_SET(1.9726783E38F) ;
        p106.sensor_id_SET((char)166) ;
        p106.quality_SET((char)173) ;
        p106.integration_time_us_SET(2783671494L) ;
        p106.time_usec_SET(8217837453483654245L) ;
        p106.distance_SET(-1.0593151E38F) ;
        p106.time_delta_distance_us_SET(3400093612L) ;
        p106.integrated_ygyro_SET(-5.2183027E36F) ;
        p106.integrated_zgyro_SET(-2.5558599E38F) ;
        p106.integrated_xgyro_SET(-7.7949843E37F) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.pressure_alt_GET() == -7.894256E37F);
            assert(pack.ygyro_GET() == 3.0139815E38F);
            assert(pack.zacc_GET() == -3.390587E38F);
            assert(pack.ymag_GET() == -3.028143E38F);
            assert(pack.yacc_GET() == 2.1374684E38F);
            assert(pack.diff_pressure_GET() == -3.3337521E38F);
            assert(pack.abs_pressure_GET() == 6.1295414E37F);
            assert(pack.fields_updated_GET() == 2076940398L);
            assert(pack.zmag_GET() == -3.375397E38F);
            assert(pack.xacc_GET() == 3.0361493E38F);
            assert(pack.temperature_GET() == 2.7129826E38F);
            assert(pack.time_usec_GET() == 3721028740245083185L);
            assert(pack.xgyro_GET() == -1.7543704E38F);
            assert(pack.zgyro_GET() == 3.1309146E38F);
            assert(pack.xmag_GET() == 6.2471104E37F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.pressure_alt_SET(-7.894256E37F) ;
        p107.zacc_SET(-3.390587E38F) ;
        p107.ymag_SET(-3.028143E38F) ;
        p107.zgyro_SET(3.1309146E38F) ;
        p107.yacc_SET(2.1374684E38F) ;
        p107.zmag_SET(-3.375397E38F) ;
        p107.abs_pressure_SET(6.1295414E37F) ;
        p107.fields_updated_SET(2076940398L) ;
        p107.temperature_SET(2.7129826E38F) ;
        p107.xgyro_SET(-1.7543704E38F) ;
        p107.time_usec_SET(3721028740245083185L) ;
        p107.xmag_SET(6.2471104E37F) ;
        p107.xacc_SET(3.0361493E38F) ;
        p107.diff_pressure_SET(-3.3337521E38F) ;
        p107.ygyro_SET(3.0139815E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.q2_GET() == 1.9599769E38F);
            assert(pack.std_dev_vert_GET() == 9.373291E37F);
            assert(pack.lon_GET() == 1.5593965E38F);
            assert(pack.vd_GET() == -3.1666376E38F);
            assert(pack.ygyro_GET() == 8.520785E37F);
            assert(pack.q1_GET() == 1.4627598E38F);
            assert(pack.yacc_GET() == -1.9102555E38F);
            assert(pack.std_dev_horz_GET() == 9.789601E37F);
            assert(pack.zgyro_GET() == 3.254971E38F);
            assert(pack.vn_GET() == -1.9840527E38F);
            assert(pack.q3_GET() == -1.0168245E38F);
            assert(pack.pitch_GET() == -8.823852E37F);
            assert(pack.q4_GET() == 2.183561E38F);
            assert(pack.yaw_GET() == -2.415397E38F);
            assert(pack.lat_GET() == -1.323103E37F);
            assert(pack.ve_GET() == 6.4166384E37F);
            assert(pack.xgyro_GET() == -1.5880073E38F);
            assert(pack.roll_GET() == -2.6987E38F);
            assert(pack.alt_GET() == -3.013948E38F);
            assert(pack.xacc_GET() == 1.7501073E38F);
            assert(pack.zacc_GET() == 8.1763895E37F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.roll_SET(-2.6987E38F) ;
        p108.ve_SET(6.4166384E37F) ;
        p108.q2_SET(1.9599769E38F) ;
        p108.vn_SET(-1.9840527E38F) ;
        p108.ygyro_SET(8.520785E37F) ;
        p108.q1_SET(1.4627598E38F) ;
        p108.zacc_SET(8.1763895E37F) ;
        p108.pitch_SET(-8.823852E37F) ;
        p108.q3_SET(-1.0168245E38F) ;
        p108.q4_SET(2.183561E38F) ;
        p108.lat_SET(-1.323103E37F) ;
        p108.zgyro_SET(3.254971E38F) ;
        p108.yacc_SET(-1.9102555E38F) ;
        p108.yaw_SET(-2.415397E38F) ;
        p108.xgyro_SET(-1.5880073E38F) ;
        p108.xacc_SET(1.7501073E38F) ;
        p108.std_dev_vert_SET(9.373291E37F) ;
        p108.lon_SET(1.5593965E38F) ;
        p108.std_dev_horz_SET(9.789601E37F) ;
        p108.vd_SET(-3.1666376E38F) ;
        p108.alt_SET(-3.013948E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)51059);
            assert(pack.noise_GET() == (char)218);
            assert(pack.txbuf_GET() == (char)151);
            assert(pack.rxerrors_GET() == (char)36798);
            assert(pack.remrssi_GET() == (char)99);
            assert(pack.rssi_GET() == (char)202);
            assert(pack.remnoise_GET() == (char)248);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)99) ;
        p109.fixed__SET((char)51059) ;
        p109.txbuf_SET((char)151) ;
        p109.rssi_SET((char)202) ;
        p109.noise_SET((char)218) ;
        p109.rxerrors_SET((char)36798) ;
        p109.remnoise_SET((char)248) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)92);
            assert(pack.target_component_GET() == (char)229);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)56, (char)168, (char)117, (char)228, (char)220, (char)61, (char)123, (char)254, (char)121, (char)188, (char)226, (char)114, (char)175, (char)12, (char)77, (char)97, (char)75, (char)184, (char)151, (char)72, (char)112, (char)58, (char)161, (char)109, (char)226, (char)144, (char)196, (char)179, (char)133, (char)229, (char)75, (char)167, (char)10, (char)38, (char)198, (char)255, (char)4, (char)226, (char)190, (char)189, (char)141, (char)106, (char)175, (char)165, (char)111, (char)187, (char)172, (char)250, (char)3, (char)129, (char)207, (char)213, (char)69, (char)18, (char)45, (char)234, (char)106, (char)254, (char)208, (char)19, (char)219, (char)89, (char)92, (char)178, (char)66, (char)125, (char)178, (char)172, (char)11, (char)27, (char)56, (char)216, (char)142, (char)78, (char)91, (char)117, (char)251, (char)253, (char)40, (char)108, (char)209, (char)148, (char)197, (char)80, (char)182, (char)136, (char)241, (char)220, (char)133, (char)66, (char)29, (char)209, (char)89, (char)195, (char)74, (char)26, (char)241, (char)62, (char)171, (char)133, (char)104, (char)191, (char)95, (char)187, (char)46, (char)35, (char)96, (char)78, (char)248, (char)41, (char)179, (char)67, (char)59, (char)74, (char)147, (char)71, (char)235, (char)184, (char)87, (char)66, (char)121, (char)17, (char)74, (char)93, (char)134, (char)1, (char)71, (char)199, (char)122, (char)158, (char)80, (char)12, (char)254, (char)15, (char)2, (char)200, (char)158, (char)87, (char)57, (char)132, (char)173, (char)220, (char)154, (char)65, (char)139, (char)157, (char)196, (char)157, (char)97, (char)22, (char)35, (char)76, (char)111, (char)220, (char)5, (char)116, (char)42, (char)30, (char)127, (char)229, (char)189, (char)194, (char)12, (char)87, (char)221, (char)113, (char)135, (char)13, (char)2, (char)7, (char)106, (char)89, (char)76, (char)75, (char)59, (char)103, (char)208, (char)3, (char)220, (char)207, (char)143, (char)230, (char)25, (char)187, (char)184, (char)77, (char)118, (char)51, (char)71, (char)221, (char)6, (char)253, (char)75, (char)43, (char)76, (char)102, (char)164, (char)73, (char)198, (char)134, (char)31, (char)81, (char)136, (char)137, (char)153, (char)187, (char)12, (char)166, (char)176, (char)111, (char)231, (char)188, (char)198, (char)12, (char)199, (char)61, (char)180, (char)227, (char)178, (char)0, (char)0, (char)12, (char)136, (char)1, (char)208, (char)53, (char)90, (char)91, (char)30, (char)136, (char)162, (char)21, (char)118, (char)131, (char)144, (char)180, (char)143, (char)74, (char)178, (char)114, (char)75, (char)50, (char)192, (char)5, (char)193, (char)204, (char)177, (char)83, (char)76, (char)59, (char)219}));
            assert(pack.target_network_GET() == (char)109);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_system_SET((char)92) ;
        p110.target_network_SET((char)109) ;
        p110.payload_SET(new char[] {(char)56, (char)168, (char)117, (char)228, (char)220, (char)61, (char)123, (char)254, (char)121, (char)188, (char)226, (char)114, (char)175, (char)12, (char)77, (char)97, (char)75, (char)184, (char)151, (char)72, (char)112, (char)58, (char)161, (char)109, (char)226, (char)144, (char)196, (char)179, (char)133, (char)229, (char)75, (char)167, (char)10, (char)38, (char)198, (char)255, (char)4, (char)226, (char)190, (char)189, (char)141, (char)106, (char)175, (char)165, (char)111, (char)187, (char)172, (char)250, (char)3, (char)129, (char)207, (char)213, (char)69, (char)18, (char)45, (char)234, (char)106, (char)254, (char)208, (char)19, (char)219, (char)89, (char)92, (char)178, (char)66, (char)125, (char)178, (char)172, (char)11, (char)27, (char)56, (char)216, (char)142, (char)78, (char)91, (char)117, (char)251, (char)253, (char)40, (char)108, (char)209, (char)148, (char)197, (char)80, (char)182, (char)136, (char)241, (char)220, (char)133, (char)66, (char)29, (char)209, (char)89, (char)195, (char)74, (char)26, (char)241, (char)62, (char)171, (char)133, (char)104, (char)191, (char)95, (char)187, (char)46, (char)35, (char)96, (char)78, (char)248, (char)41, (char)179, (char)67, (char)59, (char)74, (char)147, (char)71, (char)235, (char)184, (char)87, (char)66, (char)121, (char)17, (char)74, (char)93, (char)134, (char)1, (char)71, (char)199, (char)122, (char)158, (char)80, (char)12, (char)254, (char)15, (char)2, (char)200, (char)158, (char)87, (char)57, (char)132, (char)173, (char)220, (char)154, (char)65, (char)139, (char)157, (char)196, (char)157, (char)97, (char)22, (char)35, (char)76, (char)111, (char)220, (char)5, (char)116, (char)42, (char)30, (char)127, (char)229, (char)189, (char)194, (char)12, (char)87, (char)221, (char)113, (char)135, (char)13, (char)2, (char)7, (char)106, (char)89, (char)76, (char)75, (char)59, (char)103, (char)208, (char)3, (char)220, (char)207, (char)143, (char)230, (char)25, (char)187, (char)184, (char)77, (char)118, (char)51, (char)71, (char)221, (char)6, (char)253, (char)75, (char)43, (char)76, (char)102, (char)164, (char)73, (char)198, (char)134, (char)31, (char)81, (char)136, (char)137, (char)153, (char)187, (char)12, (char)166, (char)176, (char)111, (char)231, (char)188, (char)198, (char)12, (char)199, (char)61, (char)180, (char)227, (char)178, (char)0, (char)0, (char)12, (char)136, (char)1, (char)208, (char)53, (char)90, (char)91, (char)30, (char)136, (char)162, (char)21, (char)118, (char)131, (char)144, (char)180, (char)143, (char)74, (char)178, (char)114, (char)75, (char)50, (char)192, (char)5, (char)193, (char)204, (char)177, (char)83, (char)76, (char)59, (char)219}, 0) ;
        p110.target_component_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -7415931211986366597L);
            assert(pack.tc1_GET() == -3805224711937477920L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-3805224711937477920L) ;
        p111.ts1_SET(-7415931211986366597L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 280252774L);
            assert(pack.time_usec_GET() == 7875556534868960097L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(7875556534868960097L) ;
        p112.seq_SET(280252774L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 457503726862423417L);
            assert(pack.alt_GET() == -1135944015);
            assert(pack.ve_GET() == (short)23414);
            assert(pack.eph_GET() == (char)11181);
            assert(pack.epv_GET() == (char)49661);
            assert(pack.vel_GET() == (char)39155);
            assert(pack.lat_GET() == -1690804071);
            assert(pack.vd_GET() == (short) -5357);
            assert(pack.vn_GET() == (short) -7040);
            assert(pack.cog_GET() == (char)40891);
            assert(pack.satellites_visible_GET() == (char)238);
            assert(pack.fix_type_GET() == (char)42);
            assert(pack.lon_GET() == 1307170356);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.alt_SET(-1135944015) ;
        p113.lat_SET(-1690804071) ;
        p113.cog_SET((char)40891) ;
        p113.time_usec_SET(457503726862423417L) ;
        p113.vel_SET((char)39155) ;
        p113.fix_type_SET((char)42) ;
        p113.ve_SET((short)23414) ;
        p113.eph_SET((char)11181) ;
        p113.vn_SET((short) -7040) ;
        p113.vd_SET((short) -5357) ;
        p113.epv_SET((char)49661) ;
        p113.lon_SET(1307170356) ;
        p113.satellites_visible_SET((char)238) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)5312);
            assert(pack.time_usec_GET() == 1409951116765075714L);
            assert(pack.distance_GET() == -8.0021463E37F);
            assert(pack.integrated_y_GET() == 2.1306681E37F);
            assert(pack.quality_GET() == (char)110);
            assert(pack.time_delta_distance_us_GET() == 1302477709L);
            assert(pack.integrated_ygyro_GET() == 1.7681426E38F);
            assert(pack.integrated_xgyro_GET() == -1.158835E38F);
            assert(pack.sensor_id_GET() == (char)212);
            assert(pack.integration_time_us_GET() == 3305914778L);
            assert(pack.integrated_zgyro_GET() == 2.5942532E38F);
            assert(pack.integrated_x_GET() == 2.5080934E38F);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(1409951116765075714L) ;
        p114.sensor_id_SET((char)212) ;
        p114.integrated_ygyro_SET(1.7681426E38F) ;
        p114.integrated_xgyro_SET(-1.158835E38F) ;
        p114.integrated_y_SET(2.1306681E37F) ;
        p114.integrated_zgyro_SET(2.5942532E38F) ;
        p114.temperature_SET((short)5312) ;
        p114.integrated_x_SET(2.5080934E38F) ;
        p114.quality_SET((char)110) ;
        p114.integration_time_us_SET(3305914778L) ;
        p114.time_delta_distance_us_SET(1302477709L) ;
        p114.distance_SET(-8.0021463E37F) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {1.6357668E38F, -1.3325661E37F, 5.71345E37F, -1.4109923E38F}));
            assert(pack.vx_GET() == (short)4084);
            assert(pack.lat_GET() == -923219527);
            assert(pack.vz_GET() == (short) -27423);
            assert(pack.rollspeed_GET() == -6.158947E37F);
            assert(pack.ind_airspeed_GET() == (char)1018);
            assert(pack.time_usec_GET() == 3782256654419855358L);
            assert(pack.yawspeed_GET() == -2.4144874E38F);
            assert(pack.zacc_GET() == (short)7615);
            assert(pack.vy_GET() == (short) -24490);
            assert(pack.alt_GET() == -959353642);
            assert(pack.lon_GET() == -606922502);
            assert(pack.true_airspeed_GET() == (char)53509);
            assert(pack.pitchspeed_GET() == 1.6061122E38F);
            assert(pack.xacc_GET() == (short) -13657);
            assert(pack.yacc_GET() == (short) -4428);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.xacc_SET((short) -13657) ;
        p115.vz_SET((short) -27423) ;
        p115.ind_airspeed_SET((char)1018) ;
        p115.vx_SET((short)4084) ;
        p115.alt_SET(-959353642) ;
        p115.attitude_quaternion_SET(new float[] {1.6357668E38F, -1.3325661E37F, 5.71345E37F, -1.4109923E38F}, 0) ;
        p115.yawspeed_SET(-2.4144874E38F) ;
        p115.true_airspeed_SET((char)53509) ;
        p115.time_usec_SET(3782256654419855358L) ;
        p115.yacc_SET((short) -4428) ;
        p115.lon_SET(-606922502) ;
        p115.pitchspeed_SET(1.6061122E38F) ;
        p115.lat_SET(-923219527) ;
        p115.rollspeed_SET(-6.158947E37F) ;
        p115.zacc_SET((short)7615) ;
        p115.vy_SET((short) -24490) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -17187);
            assert(pack.xmag_GET() == (short)13458);
            assert(pack.ymag_GET() == (short)22196);
            assert(pack.zacc_GET() == (short)17665);
            assert(pack.zmag_GET() == (short)25159);
            assert(pack.yacc_GET() == (short)11534);
            assert(pack.time_boot_ms_GET() == 3143705549L);
            assert(pack.xacc_GET() == (short) -32265);
            assert(pack.xgyro_GET() == (short)29381);
            assert(pack.zgyro_GET() == (short)29762);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short)29381) ;
        p116.xacc_SET((short) -32265) ;
        p116.ymag_SET((short)22196) ;
        p116.zacc_SET((short)17665) ;
        p116.time_boot_ms_SET(3143705549L) ;
        p116.yacc_SET((short)11534) ;
        p116.zgyro_SET((short)29762) ;
        p116.zmag_SET((short)25159) ;
        p116.ygyro_SET((short) -17187) ;
        p116.xmag_SET((short)13458) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_GET() == (char)8792);
            assert(pack.target_system_GET() == (char)241);
            assert(pack.end_GET() == (char)33497);
            assert(pack.target_component_GET() == (char)227);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)227) ;
        p117.start_SET((char)8792) ;
        p117.target_system_SET((char)241) ;
        p117.end_SET((char)33497) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.num_logs_GET() == (char)19089);
            assert(pack.last_log_num_GET() == (char)20740);
            assert(pack.size_GET() == 2229158583L);
            assert(pack.id_GET() == (char)18268);
            assert(pack.time_utc_GET() == 2781224686L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.num_logs_SET((char)19089) ;
        p118.last_log_num_SET((char)20740) ;
        p118.id_SET((char)18268) ;
        p118.time_utc_SET(2781224686L) ;
        p118.size_SET(2229158583L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 1673745943L);
            assert(pack.id_GET() == (char)13393);
            assert(pack.count_GET() == 86032528L);
            assert(pack.target_component_GET() == (char)107);
            assert(pack.target_system_GET() == (char)144);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.count_SET(86032528L) ;
        p119.ofs_SET(1673745943L) ;
        p119.id_SET((char)13393) ;
        p119.target_component_SET((char)107) ;
        p119.target_system_SET((char)144) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)206, (char)243, (char)104, (char)87, (char)89, (char)254, (char)175, (char)148, (char)165, (char)52, (char)9, (char)197, (char)81, (char)90, (char)97, (char)234, (char)55, (char)101, (char)27, (char)189, (char)75, (char)60, (char)237, (char)150, (char)101, (char)60, (char)198, (char)217, (char)43, (char)107, (char)221, (char)158, (char)89, (char)179, (char)81, (char)129, (char)154, (char)182, (char)230, (char)52, (char)102, (char)176, (char)56, (char)86, (char)154, (char)153, (char)47, (char)85, (char)252, (char)193, (char)236, (char)87, (char)38, (char)231, (char)180, (char)26, (char)161, (char)98, (char)111, (char)120, (char)239, (char)239, (char)207, (char)142, (char)6, (char)46, (char)93, (char)20, (char)180, (char)243, (char)152, (char)245, (char)241, (char)144, (char)61, (char)182, (char)66, (char)33, (char)102, (char)42, (char)189, (char)72, (char)52, (char)63, (char)50, (char)214, (char)44, (char)7, (char)129, (char)114}));
            assert(pack.ofs_GET() == 2093172834L);
            assert(pack.count_GET() == (char)60);
            assert(pack.id_GET() == (char)14143);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.data__SET(new char[] {(char)206, (char)243, (char)104, (char)87, (char)89, (char)254, (char)175, (char)148, (char)165, (char)52, (char)9, (char)197, (char)81, (char)90, (char)97, (char)234, (char)55, (char)101, (char)27, (char)189, (char)75, (char)60, (char)237, (char)150, (char)101, (char)60, (char)198, (char)217, (char)43, (char)107, (char)221, (char)158, (char)89, (char)179, (char)81, (char)129, (char)154, (char)182, (char)230, (char)52, (char)102, (char)176, (char)56, (char)86, (char)154, (char)153, (char)47, (char)85, (char)252, (char)193, (char)236, (char)87, (char)38, (char)231, (char)180, (char)26, (char)161, (char)98, (char)111, (char)120, (char)239, (char)239, (char)207, (char)142, (char)6, (char)46, (char)93, (char)20, (char)180, (char)243, (char)152, (char)245, (char)241, (char)144, (char)61, (char)182, (char)66, (char)33, (char)102, (char)42, (char)189, (char)72, (char)52, (char)63, (char)50, (char)214, (char)44, (char)7, (char)129, (char)114}, 0) ;
        p120.count_SET((char)60) ;
        p120.id_SET((char)14143) ;
        p120.ofs_SET(2093172834L) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)96);
            assert(pack.target_component_GET() == (char)39);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)96) ;
        p121.target_component_SET((char)39) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)201);
            assert(pack.target_system_GET() == (char)55);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)55) ;
        p122.target_component_SET((char)201) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)169);
            assert(pack.len_GET() == (char)212);
            assert(pack.target_component_GET() == (char)8);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)126, (char)209, (char)216, (char)56, (char)161, (char)91, (char)184, (char)254, (char)16, (char)198, (char)78, (char)224, (char)119, (char)194, (char)96, (char)223, (char)135, (char)150, (char)159, (char)182, (char)210, (char)56, (char)252, (char)3, (char)145, (char)109, (char)153, (char)84, (char)107, (char)7, (char)78, (char)237, (char)206, (char)246, (char)200, (char)55, (char)122, (char)45, (char)197, (char)155, (char)225, (char)247, (char)201, (char)20, (char)128, (char)193, (char)182, (char)153, (char)177, (char)255, (char)146, (char)204, (char)212, (char)246, (char)153, (char)120, (char)89, (char)233, (char)104, (char)108, (char)71, (char)59, (char)189, (char)167, (char)55, (char)210, (char)243, (char)218, (char)27, (char)213, (char)152, (char)172, (char)33, (char)36, (char)88, (char)233, (char)168, (char)113, (char)51, (char)39, (char)170, (char)157, (char)40, (char)171, (char)21, (char)175, (char)207, (char)12, (char)36, (char)252, (char)51, (char)227, (char)206, (char)170, (char)142, (char)159, (char)241, (char)82, (char)239, (char)127, (char)17, (char)24, (char)61, (char)94, (char)187, (char)37, (char)67, (char)216, (char)63, (char)254}));
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)169) ;
        p123.len_SET((char)212) ;
        p123.target_component_SET((char)8) ;
        p123.data__SET(new char[] {(char)126, (char)209, (char)216, (char)56, (char)161, (char)91, (char)184, (char)254, (char)16, (char)198, (char)78, (char)224, (char)119, (char)194, (char)96, (char)223, (char)135, (char)150, (char)159, (char)182, (char)210, (char)56, (char)252, (char)3, (char)145, (char)109, (char)153, (char)84, (char)107, (char)7, (char)78, (char)237, (char)206, (char)246, (char)200, (char)55, (char)122, (char)45, (char)197, (char)155, (char)225, (char)247, (char)201, (char)20, (char)128, (char)193, (char)182, (char)153, (char)177, (char)255, (char)146, (char)204, (char)212, (char)246, (char)153, (char)120, (char)89, (char)233, (char)104, (char)108, (char)71, (char)59, (char)189, (char)167, (char)55, (char)210, (char)243, (char)218, (char)27, (char)213, (char)152, (char)172, (char)33, (char)36, (char)88, (char)233, (char)168, (char)113, (char)51, (char)39, (char)170, (char)157, (char)40, (char)171, (char)21, (char)175, (char)207, (char)12, (char)36, (char)252, (char)51, (char)227, (char)206, (char)170, (char)142, (char)159, (char)241, (char)82, (char)239, (char)127, (char)17, (char)24, (char)61, (char)94, (char)187, (char)37, (char)67, (char)216, (char)63, (char)254}, 0) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.vel_GET() == (char)3801);
            assert(pack.epv_GET() == (char)27759);
            assert(pack.dgps_numch_GET() == (char)127);
            assert(pack.satellites_visible_GET() == (char)198);
            assert(pack.dgps_age_GET() == 1429458524L);
            assert(pack.time_usec_GET() == 6431132042536706382L);
            assert(pack.lat_GET() == -1961913093);
            assert(pack.lon_GET() == 755453967);
            assert(pack.alt_GET() == 921533434);
            assert(pack.eph_GET() == (char)39763);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.cog_GET() == (char)8347);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.lat_SET(-1961913093) ;
        p124.eph_SET((char)39763) ;
        p124.vel_SET((char)3801) ;
        p124.lon_SET(755453967) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p124.dgps_numch_SET((char)127) ;
        p124.alt_SET(921533434) ;
        p124.time_usec_SET(6431132042536706382L) ;
        p124.cog_SET((char)8347) ;
        p124.satellites_visible_SET((char)198) ;
        p124.epv_SET((char)27759) ;
        p124.dgps_age_SET(1429458524L) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            assert(pack.Vcc_GET() == (char)5504);
            assert(pack.Vservo_GET() == (char)63734);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)5504) ;
        p125.Vservo_SET((char)63734) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.timeout_GET() == (char)26598);
            assert(pack.count_GET() == (char)152);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)251, (char)128, (char)80, (char)60, (char)135, (char)54, (char)31, (char)178, (char)9, (char)10, (char)177, (char)104, (char)137, (char)55, (char)212, (char)181, (char)236, (char)15, (char)41, (char)231, (char)85, (char)188, (char)67, (char)149, (char)237, (char)21, (char)234, (char)180, (char)138, (char)182, (char)135, (char)40, (char)33, (char)228, (char)143, (char)248, (char)104, (char)232, (char)54, (char)31, (char)243, (char)228, (char)241, (char)239, (char)149, (char)166, (char)63, (char)189, (char)87, (char)18, (char)189, (char)69, (char)51, (char)44, (char)208, (char)48, (char)47, (char)252, (char)167, (char)142, (char)211, (char)113, (char)149, (char)132, (char)16, (char)88, (char)151, (char)141, (char)228, (char)109}));
            assert(pack.baudrate_GET() == 1846678609L);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)152) ;
        p126.baudrate_SET(1846678609L) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.data__SET(new char[] {(char)251, (char)128, (char)80, (char)60, (char)135, (char)54, (char)31, (char)178, (char)9, (char)10, (char)177, (char)104, (char)137, (char)55, (char)212, (char)181, (char)236, (char)15, (char)41, (char)231, (char)85, (char)188, (char)67, (char)149, (char)237, (char)21, (char)234, (char)180, (char)138, (char)182, (char)135, (char)40, (char)33, (char)228, (char)143, (char)248, (char)104, (char)232, (char)54, (char)31, (char)243, (char)228, (char)241, (char)239, (char)149, (char)166, (char)63, (char)189, (char)87, (char)18, (char)189, (char)69, (char)51, (char)44, (char)208, (char)48, (char)47, (char)252, (char)167, (char)142, (char)211, (char)113, (char)149, (char)132, (char)16, (char)88, (char)151, (char)141, (char)228, (char)109}, 0) ;
        p126.timeout_SET((char)26598) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 2568327506L);
            assert(pack.iar_num_hypotheses_GET() == -1412392651);
            assert(pack.rtk_rate_GET() == (char)130);
            assert(pack.tow_GET() == 3508640970L);
            assert(pack.rtk_receiver_id_GET() == (char)239);
            assert(pack.baseline_coords_type_GET() == (char)155);
            assert(pack.baseline_b_mm_GET() == -1778448623);
            assert(pack.rtk_health_GET() == (char)187);
            assert(pack.wn_GET() == (char)23315);
            assert(pack.baseline_c_mm_GET() == 960986520);
            assert(pack.time_last_baseline_ms_GET() == 3523233982L);
            assert(pack.nsats_GET() == (char)74);
            assert(pack.baseline_a_mm_GET() == 964536487);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_b_mm_SET(-1778448623) ;
        p127.rtk_receiver_id_SET((char)239) ;
        p127.wn_SET((char)23315) ;
        p127.time_last_baseline_ms_SET(3523233982L) ;
        p127.tow_SET(3508640970L) ;
        p127.accuracy_SET(2568327506L) ;
        p127.baseline_a_mm_SET(964536487) ;
        p127.baseline_c_mm_SET(960986520) ;
        p127.rtk_rate_SET((char)130) ;
        p127.iar_num_hypotheses_SET(-1412392651) ;
        p127.nsats_SET((char)74) ;
        p127.baseline_coords_type_SET((char)155) ;
        p127.rtk_health_SET((char)187) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)80);
            assert(pack.wn_GET() == (char)30331);
            assert(pack.rtk_health_GET() == (char)128);
            assert(pack.baseline_b_mm_GET() == 899292002);
            assert(pack.iar_num_hypotheses_GET() == -1647127045);
            assert(pack.time_last_baseline_ms_GET() == 2508852548L);
            assert(pack.nsats_GET() == (char)158);
            assert(pack.baseline_c_mm_GET() == -500757651);
            assert(pack.rtk_rate_GET() == (char)250);
            assert(pack.tow_GET() == 3530558774L);
            assert(pack.baseline_coords_type_GET() == (char)57);
            assert(pack.baseline_a_mm_GET() == 1736713167);
            assert(pack.accuracy_GET() == 2024046007L);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.tow_SET(3530558774L) ;
        p128.rtk_health_SET((char)128) ;
        p128.baseline_b_mm_SET(899292002) ;
        p128.baseline_coords_type_SET((char)57) ;
        p128.time_last_baseline_ms_SET(2508852548L) ;
        p128.iar_num_hypotheses_SET(-1647127045) ;
        p128.nsats_SET((char)158) ;
        p128.wn_SET((char)30331) ;
        p128.baseline_c_mm_SET(-500757651) ;
        p128.accuracy_SET(2024046007L) ;
        p128.rtk_receiver_id_SET((char)80) ;
        p128.baseline_a_mm_SET(1736713167) ;
        p128.rtk_rate_SET((char)250) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == (short)21595);
            assert(pack.zacc_GET() == (short)22703);
            assert(pack.ymag_GET() == (short) -14816);
            assert(pack.xmag_GET() == (short) -11359);
            assert(pack.xgyro_GET() == (short)8051);
            assert(pack.yacc_GET() == (short) -26174);
            assert(pack.zmag_GET() == (short) -30148);
            assert(pack.ygyro_GET() == (short)4487);
            assert(pack.time_boot_ms_GET() == 709979461L);
            assert(pack.xacc_GET() == (short) -25560);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.yacc_SET((short) -26174) ;
        p129.zmag_SET((short) -30148) ;
        p129.zgyro_SET((short)21595) ;
        p129.ygyro_SET((short)4487) ;
        p129.ymag_SET((short) -14816) ;
        p129.time_boot_ms_SET(709979461L) ;
        p129.xmag_SET((short) -11359) ;
        p129.xgyro_SET((short)8051) ;
        p129.zacc_SET((short)22703) ;
        p129.xacc_SET((short) -25560) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 3442083243L);
            assert(pack.payload_GET() == (char)218);
            assert(pack.height_GET() == (char)65458);
            assert(pack.width_GET() == (char)43187);
            assert(pack.packets_GET() == (char)24574);
            assert(pack.jpg_quality_GET() == (char)244);
            assert(pack.type_GET() == (char)179);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)179) ;
        p130.payload_SET((char)218) ;
        p130.height_SET((char)65458) ;
        p130.packets_SET((char)24574) ;
        p130.size_SET(3442083243L) ;
        p130.jpg_quality_SET((char)244) ;
        p130.width_SET((char)43187) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)61377);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)172, (char)18, (char)162, (char)104, (char)6, (char)160, (char)40, (char)229, (char)45, (char)177, (char)175, (char)135, (char)164, (char)179, (char)55, (char)118, (char)244, (char)156, (char)162, (char)179, (char)195, (char)200, (char)166, (char)120, (char)171, (char)49, (char)114, (char)84, (char)151, (char)38, (char)36, (char)164, (char)85, (char)165, (char)3, (char)66, (char)124, (char)212, (char)97, (char)170, (char)61, (char)113, (char)53, (char)233, (char)173, (char)55, (char)227, (char)254, (char)184, (char)246, (char)18, (char)19, (char)67, (char)185, (char)244, (char)79, (char)152, (char)100, (char)214, (char)75, (char)147, (char)195, (char)160, (char)164, (char)248, (char)251, (char)12, (char)134, (char)126, (char)205, (char)41, (char)161, (char)76, (char)149, (char)223, (char)134, (char)169, (char)39, (char)91, (char)224, (char)206, (char)80, (char)149, (char)71, (char)70, (char)23, (char)116, (char)241, (char)74, (char)11, (char)47, (char)20, (char)52, (char)129, (char)221, (char)19, (char)118, (char)80, (char)129, (char)136, (char)173, (char)14, (char)242, (char)181, (char)187, (char)164, (char)183, (char)190, (char)44, (char)176, (char)17, (char)246, (char)76, (char)181, (char)211, (char)83, (char)32, (char)227, (char)17, (char)62, (char)114, (char)84, (char)57, (char)61, (char)50, (char)225, (char)62, (char)137, (char)9, (char)92, (char)54, (char)20, (char)234, (char)31, (char)136, (char)175, (char)9, (char)158, (char)29, (char)169, (char)118, (char)106, (char)2, (char)144, (char)5, (char)143, (char)55, (char)113, (char)225, (char)97, (char)131, (char)162, (char)86, (char)157, (char)208, (char)54, (char)35, (char)159, (char)8, (char)33, (char)138, (char)124, (char)144, (char)121, (char)147, (char)59, (char)83, (char)154, (char)117, (char)12, (char)32, (char)139, (char)251, (char)32, (char)49, (char)95, (char)22, (char)13, (char)248, (char)83, (char)24, (char)8, (char)157, (char)204, (char)43, (char)228, (char)220, (char)211, (char)87, (char)226, (char)188, (char)126, (char)131, (char)21, (char)255, (char)218, (char)250, (char)71, (char)239, (char)126, (char)130, (char)225, (char)186, (char)128, (char)177, (char)205, (char)91, (char)150, (char)234, (char)229, (char)114, (char)0, (char)5, (char)59, (char)6, (char)85, (char)121, (char)33, (char)127, (char)247, (char)188, (char)57, (char)125, (char)20, (char)12, (char)24, (char)117, (char)147, (char)108, (char)203, (char)14, (char)38, (char)43, (char)43, (char)92, (char)75, (char)42, (char)37, (char)220, (char)29, (char)245, (char)55, (char)232, (char)31, (char)64, (char)78, (char)162, (char)142, (char)85, (char)217, (char)143, (char)168, (char)166}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)61377) ;
        p131.data__SET(new char[] {(char)172, (char)18, (char)162, (char)104, (char)6, (char)160, (char)40, (char)229, (char)45, (char)177, (char)175, (char)135, (char)164, (char)179, (char)55, (char)118, (char)244, (char)156, (char)162, (char)179, (char)195, (char)200, (char)166, (char)120, (char)171, (char)49, (char)114, (char)84, (char)151, (char)38, (char)36, (char)164, (char)85, (char)165, (char)3, (char)66, (char)124, (char)212, (char)97, (char)170, (char)61, (char)113, (char)53, (char)233, (char)173, (char)55, (char)227, (char)254, (char)184, (char)246, (char)18, (char)19, (char)67, (char)185, (char)244, (char)79, (char)152, (char)100, (char)214, (char)75, (char)147, (char)195, (char)160, (char)164, (char)248, (char)251, (char)12, (char)134, (char)126, (char)205, (char)41, (char)161, (char)76, (char)149, (char)223, (char)134, (char)169, (char)39, (char)91, (char)224, (char)206, (char)80, (char)149, (char)71, (char)70, (char)23, (char)116, (char)241, (char)74, (char)11, (char)47, (char)20, (char)52, (char)129, (char)221, (char)19, (char)118, (char)80, (char)129, (char)136, (char)173, (char)14, (char)242, (char)181, (char)187, (char)164, (char)183, (char)190, (char)44, (char)176, (char)17, (char)246, (char)76, (char)181, (char)211, (char)83, (char)32, (char)227, (char)17, (char)62, (char)114, (char)84, (char)57, (char)61, (char)50, (char)225, (char)62, (char)137, (char)9, (char)92, (char)54, (char)20, (char)234, (char)31, (char)136, (char)175, (char)9, (char)158, (char)29, (char)169, (char)118, (char)106, (char)2, (char)144, (char)5, (char)143, (char)55, (char)113, (char)225, (char)97, (char)131, (char)162, (char)86, (char)157, (char)208, (char)54, (char)35, (char)159, (char)8, (char)33, (char)138, (char)124, (char)144, (char)121, (char)147, (char)59, (char)83, (char)154, (char)117, (char)12, (char)32, (char)139, (char)251, (char)32, (char)49, (char)95, (char)22, (char)13, (char)248, (char)83, (char)24, (char)8, (char)157, (char)204, (char)43, (char)228, (char)220, (char)211, (char)87, (char)226, (char)188, (char)126, (char)131, (char)21, (char)255, (char)218, (char)250, (char)71, (char)239, (char)126, (char)130, (char)225, (char)186, (char)128, (char)177, (char)205, (char)91, (char)150, (char)234, (char)229, (char)114, (char)0, (char)5, (char)59, (char)6, (char)85, (char)121, (char)33, (char)127, (char)247, (char)188, (char)57, (char)125, (char)20, (char)12, (char)24, (char)117, (char)147, (char)108, (char)203, (char)14, (char)38, (char)43, (char)43, (char)92, (char)75, (char)42, (char)37, (char)220, (char)29, (char)245, (char)55, (char)232, (char)31, (char)64, (char)78, (char)162, (char)142, (char)85, (char)217, (char)143, (char)168, (char)166}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.current_distance_GET() == (char)37056);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.id_GET() == (char)84);
            assert(pack.min_distance_GET() == (char)23784);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90);
            assert(pack.covariance_GET() == (char)157);
            assert(pack.time_boot_ms_GET() == 2734097410L);
            assert(pack.max_distance_GET() == (char)23722);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.id_SET((char)84) ;
        p132.covariance_SET((char)157) ;
        p132.current_distance_SET((char)37056) ;
        p132.min_distance_SET((char)23784) ;
        p132.time_boot_ms_SET(2734097410L) ;
        p132.max_distance_SET((char)23722) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -936506358);
            assert(pack.grid_spacing_GET() == (char)37925);
            assert(pack.lat_GET() == 1227721995);
            assert(pack.mask_GET() == 8430501575351858705L);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.grid_spacing_SET((char)37925) ;
        p133.lat_SET(1227721995) ;
        p133.mask_SET(8430501575351858705L) ;
        p133.lon_SET(-936506358) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1462139312);
            assert(pack.lon_GET() == 1805579733);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)24203, (short)1824, (short)11849, (short) -4525, (short)21553, (short)7825, (short) -9503, (short) -13623, (short) -22980, (short) -3472, (short)10414, (short) -26989, (short) -13011, (short) -13216, (short)20515, (short) -29899}));
            assert(pack.grid_spacing_GET() == (char)37420);
            assert(pack.gridbit_GET() == (char)153);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.grid_spacing_SET((char)37420) ;
        p134.lon_SET(1805579733) ;
        p134.data__SET(new short[] {(short)24203, (short)1824, (short)11849, (short) -4525, (short)21553, (short)7825, (short) -9503, (short) -13623, (short) -22980, (short) -3472, (short)10414, (short) -26989, (short) -13011, (short) -13216, (short)20515, (short) -29899}, 0) ;
        p134.lat_SET(-1462139312) ;
        p134.gridbit_SET((char)153) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1181584518);
            assert(pack.lon_GET() == 449566387);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(449566387) ;
        p135.lat_SET(-1181584518) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.loaded_GET() == (char)23901);
            assert(pack.pending_GET() == (char)58686);
            assert(pack.current_height_GET() == 2.2604946E37F);
            assert(pack.lat_GET() == 602485947);
            assert(pack.lon_GET() == -399727917);
            assert(pack.spacing_GET() == (char)3827);
            assert(pack.terrain_height_GET() == 3.8516347E37F);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(2.2604946E37F) ;
        p136.lon_SET(-399727917) ;
        p136.loaded_SET((char)23901) ;
        p136.terrain_height_SET(3.8516347E37F) ;
        p136.lat_SET(602485947) ;
        p136.pending_SET((char)58686) ;
        p136.spacing_SET((char)3827) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 2.1490874E38F);
            assert(pack.temperature_GET() == (short) -3470);
            assert(pack.time_boot_ms_GET() == 374557380L);
            assert(pack.press_diff_GET() == 2.2072223E38F);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(2.2072223E38F) ;
        p137.press_abs_SET(2.1490874E38F) ;
        p137.temperature_SET((short) -3470) ;
        p137.time_boot_ms_SET(374557380L) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.0861042E38F, -2.1674126E38F, 8.521205E36F, -1.5107848E38F}));
            assert(pack.x_GET() == -1.007698E38F);
            assert(pack.time_usec_GET() == 8277159344159266448L);
            assert(pack.y_GET() == 3.3928476E38F);
            assert(pack.z_GET() == 2.6107688E38F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(2.6107688E38F) ;
        p138.time_usec_SET(8277159344159266448L) ;
        p138.x_SET(-1.007698E38F) ;
        p138.q_SET(new float[] {2.0861042E38F, -2.1674126E38F, 8.521205E36F, -1.5107848E38F}, 0) ;
        p138.y_SET(3.3928476E38F) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)177);
            assert(pack.target_system_GET() == (char)236);
            assert(pack.time_usec_GET() == 8233730353980291110L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {5.2570206E37F, 1.8032088E38F, -1.5180623E38F, 2.2434221E38F, 6.7806244E36F, 1.1082576E38F, 8.52016E37F, -1.3011055E38F}));
            assert(pack.target_component_GET() == (char)201);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(8233730353980291110L) ;
        p139.group_mlx_SET((char)177) ;
        p139.controls_SET(new float[] {5.2570206E37F, 1.8032088E38F, -1.5180623E38F, 2.2434221E38F, 6.7806244E36F, 1.1082576E38F, 8.52016E37F, -1.3011055E38F}, 0) ;
        p139.target_component_SET((char)201) ;
        p139.target_system_SET((char)236) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)28);
            assert(pack.time_usec_GET() == 9060234813817643332L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-1.149958E37F, -1.0288081E38F, -3.4027298E38F, -6.104838E37F, -2.5361157E38F, 2.521171E38F, -5.0540973E36F, -2.7849349E38F}));
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.controls_SET(new float[] {-1.149958E37F, -1.0288081E38F, -3.4027298E38F, -6.104838E37F, -2.5361157E38F, 2.521171E38F, -5.0540973E36F, -2.7849349E38F}, 0) ;
        p140.time_usec_SET(9060234813817643332L) ;
        p140.group_mlx_SET((char)28) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4243407892809575064L);
            assert(pack.altitude_relative_GET() == 1.2097661E38F);
            assert(pack.altitude_terrain_GET() == 2.7483026E38F);
            assert(pack.altitude_amsl_GET() == -2.962982E38F);
            assert(pack.altitude_monotonic_GET() == -3.2216198E38F);
            assert(pack.altitude_local_GET() == 1.9793746E38F);
            assert(pack.bottom_clearance_GET() == -1.8090993E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_monotonic_SET(-3.2216198E38F) ;
        p141.altitude_amsl_SET(-2.962982E38F) ;
        p141.time_usec_SET(4243407892809575064L) ;
        p141.altitude_terrain_SET(2.7483026E38F) ;
        p141.bottom_clearance_SET(-1.8090993E38F) ;
        p141.altitude_relative_SET(1.2097661E38F) ;
        p141.altitude_local_SET(1.9793746E38F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.uri_type_GET() == (char)120);
            assert(pack.transfer_type_GET() == (char)192);
            assert(pack.request_id_GET() == (char)164);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)39, (char)41, (char)154, (char)148, (char)192, (char)46, (char)137, (char)171, (char)8, (char)241, (char)177, (char)181, (char)84, (char)190, (char)10, (char)252, (char)241, (char)53, (char)176, (char)12, (char)40, (char)255, (char)154, (char)215, (char)244, (char)142, (char)62, (char)18, (char)130, (char)194, (char)167, (char)195, (char)45, (char)107, (char)87, (char)194, (char)34, (char)88, (char)142, (char)195, (char)56, (char)152, (char)52, (char)118, (char)194, (char)235, (char)56, (char)84, (char)240, (char)246, (char)20, (char)25, (char)241, (char)234, (char)112, (char)175, (char)195, (char)235, (char)117, (char)245, (char)33, (char)46, (char)228, (char)157, (char)66, (char)247, (char)79, (char)82, (char)136, (char)138, (char)167, (char)39, (char)172, (char)133, (char)128, (char)22, (char)0, (char)122, (char)93, (char)27, (char)145, (char)31, (char)191, (char)126, (char)51, (char)62, (char)255, (char)116, (char)120, (char)94, (char)10, (char)171, (char)192, (char)34, (char)115, (char)127, (char)161, (char)160, (char)51, (char)221, (char)40, (char)240, (char)91, (char)228, (char)95, (char)204, (char)195, (char)50, (char)18, (char)69, (char)215, (char)12, (char)131, (char)209, (char)37, (char)105, (char)185, (char)214, (char)8, (char)214}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)149, (char)83, (char)223, (char)195, (char)248, (char)159, (char)243, (char)229, (char)116, (char)111, (char)124, (char)23, (char)121, (char)34, (char)56, (char)157, (char)236, (char)35, (char)167, (char)71, (char)255, (char)86, (char)117, (char)108, (char)62, (char)23, (char)115, (char)249, (char)132, (char)202, (char)211, (char)7, (char)99, (char)109, (char)165, (char)42, (char)32, (char)216, (char)112, (char)27, (char)74, (char)2, (char)88, (char)93, (char)66, (char)91, (char)208, (char)21, (char)49, (char)61, (char)151, (char)225, (char)93, (char)25, (char)60, (char)228, (char)255, (char)138, (char)134, (char)120, (char)39, (char)87, (char)82, (char)179, (char)178, (char)99, (char)49, (char)145, (char)95, (char)33, (char)245, (char)194, (char)117, (char)189, (char)239, (char)223, (char)68, (char)141, (char)92, (char)140, (char)96, (char)92, (char)135, (char)144, (char)239, (char)118, (char)47, (char)135, (char)37, (char)213, (char)106, (char)185, (char)235, (char)156, (char)252, (char)0, (char)77, (char)43, (char)25, (char)41, (char)107, (char)123, (char)46, (char)146, (char)41, (char)65, (char)111, (char)114, (char)225, (char)50, (char)202, (char)184, (char)113, (char)147, (char)62, (char)126, (char)226, (char)88, (char)126, (char)218}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)192) ;
        p142.request_id_SET((char)164) ;
        p142.uri_type_SET((char)120) ;
        p142.storage_SET(new char[] {(char)149, (char)83, (char)223, (char)195, (char)248, (char)159, (char)243, (char)229, (char)116, (char)111, (char)124, (char)23, (char)121, (char)34, (char)56, (char)157, (char)236, (char)35, (char)167, (char)71, (char)255, (char)86, (char)117, (char)108, (char)62, (char)23, (char)115, (char)249, (char)132, (char)202, (char)211, (char)7, (char)99, (char)109, (char)165, (char)42, (char)32, (char)216, (char)112, (char)27, (char)74, (char)2, (char)88, (char)93, (char)66, (char)91, (char)208, (char)21, (char)49, (char)61, (char)151, (char)225, (char)93, (char)25, (char)60, (char)228, (char)255, (char)138, (char)134, (char)120, (char)39, (char)87, (char)82, (char)179, (char)178, (char)99, (char)49, (char)145, (char)95, (char)33, (char)245, (char)194, (char)117, (char)189, (char)239, (char)223, (char)68, (char)141, (char)92, (char)140, (char)96, (char)92, (char)135, (char)144, (char)239, (char)118, (char)47, (char)135, (char)37, (char)213, (char)106, (char)185, (char)235, (char)156, (char)252, (char)0, (char)77, (char)43, (char)25, (char)41, (char)107, (char)123, (char)46, (char)146, (char)41, (char)65, (char)111, (char)114, (char)225, (char)50, (char)202, (char)184, (char)113, (char)147, (char)62, (char)126, (char)226, (char)88, (char)126, (char)218}, 0) ;
        p142.uri_SET(new char[] {(char)39, (char)41, (char)154, (char)148, (char)192, (char)46, (char)137, (char)171, (char)8, (char)241, (char)177, (char)181, (char)84, (char)190, (char)10, (char)252, (char)241, (char)53, (char)176, (char)12, (char)40, (char)255, (char)154, (char)215, (char)244, (char)142, (char)62, (char)18, (char)130, (char)194, (char)167, (char)195, (char)45, (char)107, (char)87, (char)194, (char)34, (char)88, (char)142, (char)195, (char)56, (char)152, (char)52, (char)118, (char)194, (char)235, (char)56, (char)84, (char)240, (char)246, (char)20, (char)25, (char)241, (char)234, (char)112, (char)175, (char)195, (char)235, (char)117, (char)245, (char)33, (char)46, (char)228, (char)157, (char)66, (char)247, (char)79, (char)82, (char)136, (char)138, (char)167, (char)39, (char)172, (char)133, (char)128, (char)22, (char)0, (char)122, (char)93, (char)27, (char)145, (char)31, (char)191, (char)126, (char)51, (char)62, (char)255, (char)116, (char)120, (char)94, (char)10, (char)171, (char)192, (char)34, (char)115, (char)127, (char)161, (char)160, (char)51, (char)221, (char)40, (char)240, (char)91, (char)228, (char)95, (char)204, (char)195, (char)50, (char)18, (char)69, (char)215, (char)12, (char)131, (char)209, (char)37, (char)105, (char)185, (char)214, (char)8, (char)214}, 0) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3256124906L);
            assert(pack.press_diff_GET() == 2.7198693E38F);
            assert(pack.temperature_GET() == (short)27286);
            assert(pack.press_abs_GET() == -6.803447E37F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3256124906L) ;
        p143.press_abs_SET(-6.803447E37F) ;
        p143.press_diff_SET(2.7198693E38F) ;
        p143.temperature_SET((short)27286) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.8638342E38F, -3.3989627E38F, 3.196571E38F}));
            assert(pack.est_capabilities_GET() == (char)149);
            assert(pack.lon_GET() == -977030128);
            assert(pack.custom_state_GET() == 3861100708296989929L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-6.9795275E37F, 2.519834E38F, 1.0275003E38F}));
            assert(pack.lat_GET() == 1916549929);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.404006E37F, -4.199961E37F, 2.6378645E38F}));
            assert(pack.timestamp_GET() == 1241107689372691693L);
            assert(pack.alt_GET() == 3.1606067E38F);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {-1.0746558E38F, -1.6409178E38F, 1.4060351E38F, -1.6819359E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.7658662E38F, -2.6411638E38F, -1.8863894E38F}));
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.alt_SET(3.1606067E38F) ;
        p144.timestamp_SET(1241107689372691693L) ;
        p144.custom_state_SET(3861100708296989929L) ;
        p144.position_cov_SET(new float[] {-6.9795275E37F, 2.519834E38F, 1.0275003E38F}, 0) ;
        p144.acc_SET(new float[] {2.404006E37F, -4.199961E37F, 2.6378645E38F}, 0) ;
        p144.lon_SET(-977030128) ;
        p144.lat_SET(1916549929) ;
        p144.est_capabilities_SET((char)149) ;
        p144.attitude_q_SET(new float[] {-1.0746558E38F, -1.6409178E38F, 1.4060351E38F, -1.6819359E38F}, 0) ;
        p144.rates_SET(new float[] {-2.7658662E38F, -2.6411638E38F, -1.8863894E38F}, 0) ;
        p144.vel_SET(new float[] {1.8638342E38F, -3.3989627E38F, 3.196571E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.1947715E37F, -2.8093624E38F, 3.0449844E38F, 7.3526387E37F}));
            assert(pack.roll_rate_GET() == 1.3614598E38F);
            assert(pack.time_usec_GET() == 8172620197246681951L);
            assert(pack.y_acc_GET() == 3.065953E38F);
            assert(pack.pitch_rate_GET() == -1.0320876E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-6.323285E37F, 1.8466198E37F, -1.3207645E38F}));
            assert(pack.x_acc_GET() == -1.6341536E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {2.5188037E38F, 1.125047E38F, -2.4276722E38F}));
            assert(pack.airspeed_GET() == 2.157727E38F);
            assert(pack.z_pos_GET() == 1.7626323E38F);
            assert(pack.yaw_rate_GET() == 1.3739122E38F);
            assert(pack.z_vel_GET() == 1.4888371E38F);
            assert(pack.x_vel_GET() == -2.7867284E38F);
            assert(pack.x_pos_GET() == 1.4576551E38F);
            assert(pack.y_pos_GET() == 8.537325E37F);
            assert(pack.z_acc_GET() == 2.1829629E38F);
            assert(pack.y_vel_GET() == 2.9888621E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pitch_rate_SET(-1.0320876E38F) ;
        p146.roll_rate_SET(1.3614598E38F) ;
        p146.x_acc_SET(-1.6341536E37F) ;
        p146.z_vel_SET(1.4888371E38F) ;
        p146.q_SET(new float[] {-2.1947715E37F, -2.8093624E38F, 3.0449844E38F, 7.3526387E37F}, 0) ;
        p146.z_pos_SET(1.7626323E38F) ;
        p146.y_vel_SET(2.9888621E38F) ;
        p146.airspeed_SET(2.157727E38F) ;
        p146.time_usec_SET(8172620197246681951L) ;
        p146.pos_variance_SET(new float[] {2.5188037E38F, 1.125047E38F, -2.4276722E38F}, 0) ;
        p146.x_vel_SET(-2.7867284E38F) ;
        p146.x_pos_SET(1.4576551E38F) ;
        p146.y_pos_SET(8.537325E37F) ;
        p146.vel_variance_SET(new float[] {-6.323285E37F, 1.8466198E37F, -1.3207645E38F}, 0) ;
        p146.z_acc_SET(2.1829629E38F) ;
        p146.yaw_rate_SET(1.3739122E38F) ;
        p146.y_acc_SET(3.065953E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short) -31402);
            assert(pack.current_battery_GET() == (short)11092);
            assert(pack.id_GET() == (char)197);
            assert(pack.energy_consumed_GET() == -915669667);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
            assert(pack.battery_remaining_GET() == (byte)66);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
            assert(pack.current_consumed_GET() == 1127982984);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)30988, (char)35259, (char)9619, (char)64485, (char)57559, (char)42978, (char)44518, (char)29443, (char)1155, (char)28475}));
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO) ;
        p147.battery_remaining_SET((byte)66) ;
        p147.temperature_SET((short) -31402) ;
        p147.current_battery_SET((short)11092) ;
        p147.current_consumed_SET(1127982984) ;
        p147.energy_consumed_SET(-915669667) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN) ;
        p147.voltages_SET(new char[] {(char)30988, (char)35259, (char)9619, (char)64485, (char)57559, (char)42978, (char)44518, (char)29443, (char)1155, (char)28475}, 0) ;
        p147.id_SET((char)197) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)209, (char)83, (char)110, (char)51, (char)47, (char)79, (char)207, (char)152, (char)81, (char)124, (char)232, (char)140, (char)65, (char)77, (char)60, (char)102, (char)97, (char)122}));
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)128, (char)161, (char)63, (char)109, (char)105, (char)217, (char)123, (char)114}));
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)155, (char)21, (char)57, (char)192, (char)242, (char)93, (char)156, (char)77}));
            assert(pack.os_sw_version_GET() == 164695009L);
            assert(pack.product_id_GET() == (char)41679);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)9, (char)146, (char)59, (char)217, (char)206, (char)23, (char)75, (char)95}));
            assert(pack.flight_sw_version_GET() == 1636130677L);
            assert(pack.vendor_id_GET() == (char)35401);
            assert(pack.middleware_sw_version_GET() == 2774023034L);
            assert(pack.uid_GET() == 4271514700269165530L);
            assert(pack.board_version_GET() == 829847267L);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)155, (char)21, (char)57, (char)192, (char)242, (char)93, (char)156, (char)77}, 0) ;
        p148.middleware_sw_version_SET(2774023034L) ;
        p148.os_sw_version_SET(164695009L) ;
        p148.uid_SET(4271514700269165530L) ;
        p148.product_id_SET((char)41679) ;
        p148.uid2_SET(new char[] {(char)209, (char)83, (char)110, (char)51, (char)47, (char)79, (char)207, (char)152, (char)81, (char)124, (char)232, (char)140, (char)65, (char)77, (char)60, (char)102, (char)97, (char)122}, 0, PH) ;
        p148.vendor_id_SET((char)35401) ;
        p148.flight_sw_version_SET(1636130677L) ;
        p148.flight_custom_version_SET(new char[] {(char)9, (char)146, (char)59, (char)217, (char)206, (char)23, (char)75, (char)95}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET) ;
        p148.board_version_SET(829847267L) ;
        p148.os_custom_version_SET(new char[] {(char)128, (char)161, (char)63, (char)109, (char)105, (char)217, (char)123, (char)114}, 0) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_num_GET() == (char)16);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.size_x_GET() == 5.236849E37F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {1.9430494E38F, 6.646387E37F, -2.9756157E38F, -1.1754653E37F}));
            assert(pack.size_y_GET() == -1.2915204E38F);
            assert(pack.y_TRY(ph) == 1.8113618E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.time_usec_GET() == 1535649030794361036L);
            assert(pack.x_TRY(ph) == 2.6505388E38F);
            assert(pack.angle_x_GET() == -8.834811E37F);
            assert(pack.distance_GET() == -2.1415385E38F);
            assert(pack.position_valid_TRY(ph) == (char)164);
            assert(pack.angle_y_GET() == 1.1237896E38F);
            assert(pack.z_TRY(ph) == 5.4297405E37F);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.target_num_SET((char)16) ;
        p149.size_y_SET(-1.2915204E38F) ;
        p149.distance_SET(-2.1415385E38F) ;
        p149.q_SET(new float[] {1.9430494E38F, 6.646387E37F, -2.9756157E38F, -1.1754653E37F}, 0, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p149.position_valid_SET((char)164, PH) ;
        p149.z_SET(5.4297405E37F, PH) ;
        p149.angle_y_SET(1.1237896E38F) ;
        p149.size_x_SET(5.236849E37F) ;
        p149.time_usec_SET(1535649030794361036L) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.angle_x_SET(-8.834811E37F) ;
        p149.x_SET(2.6505388E38F, PH) ;
        p149.y_SET(1.8113618E38F, PH) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            assert(pack.pos_horiz_ratio_GET() == -6.883459E37F);
            assert(pack.time_usec_GET() == 2302194601037570374L);
            assert(pack.hagl_ratio_GET() == -2.1131013E38F);
            assert(pack.mag_ratio_GET() == -2.8858354E38F);
            assert(pack.vel_ratio_GET() == 2.7254857E38F);
            assert(pack.pos_horiz_accuracy_GET() == 1.9137577E38F);
            assert(pack.pos_vert_ratio_GET() == 2.1138834E38F);
            assert(pack.tas_ratio_GET() == -2.6002134E38F);
            assert(pack.pos_vert_accuracy_GET() == 1.4777771E37F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_accuracy_SET(1.4777771E37F) ;
        p230.mag_ratio_SET(-2.8858354E38F) ;
        p230.pos_horiz_accuracy_SET(1.9137577E38F) ;
        p230.hagl_ratio_SET(-2.1131013E38F) ;
        p230.pos_vert_ratio_SET(2.1138834E38F) ;
        p230.time_usec_SET(2302194601037570374L) ;
        p230.vel_ratio_SET(2.7254857E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT) ;
        p230.tas_ratio_SET(-2.6002134E38F) ;
        p230.pos_horiz_ratio_SET(-6.883459E37F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == -2.9289286E38F);
            assert(pack.horiz_accuracy_GET() == 1.6684925E38F);
            assert(pack.wind_y_GET() == 1.2602929E37F);
            assert(pack.wind_z_GET() == -2.172359E38F);
            assert(pack.wind_x_GET() == 2.1434231E38F);
            assert(pack.var_vert_GET() == -9.943845E37F);
            assert(pack.vert_accuracy_GET() == -1.3901122E38F);
            assert(pack.wind_alt_GET() == -9.767578E37F);
            assert(pack.time_usec_GET() == 6370621001501636893L);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(6370621001501636893L) ;
        p231.wind_alt_SET(-9.767578E37F) ;
        p231.wind_y_SET(1.2602929E37F) ;
        p231.horiz_accuracy_SET(1.6684925E38F) ;
        p231.var_vert_SET(-9.943845E37F) ;
        p231.var_horiz_SET(-2.9289286E38F) ;
        p231.vert_accuracy_SET(-1.3901122E38F) ;
        p231.wind_z_SET(-2.172359E38F) ;
        p231.wind_x_SET(2.1434231E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.speed_accuracy_GET() == -6.102271E37F);
            assert(pack.vd_GET() == -2.9963315E38F);
            assert(pack.fix_type_GET() == (char)141);
            assert(pack.ve_GET() == 1.0686473E38F);
            assert(pack.vn_GET() == 2.5364242E38F);
            assert(pack.hdop_GET() == -3.501502E37F);
            assert(pack.satellites_visible_GET() == (char)156);
            assert(pack.vdop_GET() == -8.0956205E36F);
            assert(pack.vert_accuracy_GET() == 1.1726694E38F);
            assert(pack.time_week_GET() == (char)51932);
            assert(pack.time_usec_GET() == 8106597499374707541L);
            assert(pack.lon_GET() == -140383474);
            assert(pack.gps_id_GET() == (char)68);
            assert(pack.horiz_accuracy_GET() == -2.0892304E38F);
            assert(pack.lat_GET() == -1506309304);
            assert(pack.alt_GET() == -2.2839587E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
            assert(pack.time_week_ms_GET() == 3534701976L);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.ve_SET(1.0686473E38F) ;
        p232.horiz_accuracy_SET(-2.0892304E38F) ;
        p232.satellites_visible_SET((char)156) ;
        p232.fix_type_SET((char)141) ;
        p232.vert_accuracy_SET(1.1726694E38F) ;
        p232.hdop_SET(-3.501502E37F) ;
        p232.vn_SET(2.5364242E38F) ;
        p232.lat_SET(-1506309304) ;
        p232.time_week_ms_SET(3534701976L) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP) ;
        p232.lon_SET(-140383474) ;
        p232.time_usec_SET(8106597499374707541L) ;
        p232.alt_SET(-2.2839587E38F) ;
        p232.vdop_SET(-8.0956205E36F) ;
        p232.time_week_SET((char)51932) ;
        p232.speed_accuracy_SET(-6.102271E37F) ;
        p232.vd_SET(-2.9963315E38F) ;
        p232.gps_id_SET((char)68) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)129);
            assert(pack.len_GET() == (char)254);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)208, (char)229, (char)101, (char)77, (char)51, (char)20, (char)177, (char)21, (char)241, (char)252, (char)220, (char)203, (char)228, (char)244, (char)45, (char)235, (char)46, (char)228, (char)127, (char)29, (char)114, (char)155, (char)108, (char)70, (char)64, (char)56, (char)85, (char)74, (char)42, (char)49, (char)187, (char)153, (char)204, (char)54, (char)170, (char)37, (char)163, (char)75, (char)44, (char)138, (char)13, (char)204, (char)23, (char)199, (char)201, (char)143, (char)214, (char)67, (char)24, (char)98, (char)157, (char)54, (char)63, (char)64, (char)46, (char)123, (char)33, (char)241, (char)91, (char)216, (char)106, (char)140, (char)86, (char)43, (char)32, (char)145, (char)8, (char)174, (char)115, (char)132, (char)68, (char)92, (char)134, (char)254, (char)233, (char)149, (char)184, (char)86, (char)221, (char)96, (char)197, (char)199, (char)142, (char)54, (char)178, (char)121, (char)235, (char)239, (char)241, (char)146, (char)234, (char)212, (char)89, (char)243, (char)220, (char)197, (char)171, (char)70, (char)125, (char)173, (char)5, (char)192, (char)192, (char)121, (char)92, (char)48, (char)238, (char)166, (char)142, (char)170, (char)36, (char)129, (char)99, (char)192, (char)191, (char)234, (char)251, (char)18, (char)252, (char)111, (char)19, (char)52, (char)0, (char)209, (char)134, (char)141, (char)50, (char)74, (char)178, (char)240, (char)101, (char)10, (char)157, (char)214, (char)70, (char)248, (char)240, (char)244, (char)66, (char)33, (char)190, (char)79, (char)74, (char)14, (char)93, (char)52, (char)117, (char)115, (char)48, (char)8, (char)70, (char)207, (char)93, (char)46, (char)164, (char)43, (char)165, (char)231, (char)63, (char)128, (char)136, (char)68, (char)76, (char)103, (char)153, (char)56, (char)38, (char)29, (char)240, (char)247, (char)65, (char)213, (char)131, (char)7, (char)59, (char)2, (char)97, (char)10, (char)24, (char)17}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)254) ;
        p233.data__SET(new char[] {(char)208, (char)229, (char)101, (char)77, (char)51, (char)20, (char)177, (char)21, (char)241, (char)252, (char)220, (char)203, (char)228, (char)244, (char)45, (char)235, (char)46, (char)228, (char)127, (char)29, (char)114, (char)155, (char)108, (char)70, (char)64, (char)56, (char)85, (char)74, (char)42, (char)49, (char)187, (char)153, (char)204, (char)54, (char)170, (char)37, (char)163, (char)75, (char)44, (char)138, (char)13, (char)204, (char)23, (char)199, (char)201, (char)143, (char)214, (char)67, (char)24, (char)98, (char)157, (char)54, (char)63, (char)64, (char)46, (char)123, (char)33, (char)241, (char)91, (char)216, (char)106, (char)140, (char)86, (char)43, (char)32, (char)145, (char)8, (char)174, (char)115, (char)132, (char)68, (char)92, (char)134, (char)254, (char)233, (char)149, (char)184, (char)86, (char)221, (char)96, (char)197, (char)199, (char)142, (char)54, (char)178, (char)121, (char)235, (char)239, (char)241, (char)146, (char)234, (char)212, (char)89, (char)243, (char)220, (char)197, (char)171, (char)70, (char)125, (char)173, (char)5, (char)192, (char)192, (char)121, (char)92, (char)48, (char)238, (char)166, (char)142, (char)170, (char)36, (char)129, (char)99, (char)192, (char)191, (char)234, (char)251, (char)18, (char)252, (char)111, (char)19, (char)52, (char)0, (char)209, (char)134, (char)141, (char)50, (char)74, (char)178, (char)240, (char)101, (char)10, (char)157, (char)214, (char)70, (char)248, (char)240, (char)244, (char)66, (char)33, (char)190, (char)79, (char)74, (char)14, (char)93, (char)52, (char)117, (char)115, (char)48, (char)8, (char)70, (char)207, (char)93, (char)46, (char)164, (char)43, (char)165, (char)231, (char)63, (char)128, (char)136, (char)68, (char)76, (char)103, (char)153, (char)56, (char)38, (char)29, (char)240, (char)247, (char)65, (char)213, (char)131, (char)7, (char)59, (char)2, (char)97, (char)10, (char)24, (char)17}, 0) ;
        p233.flags_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == (short) -12388);
            assert(pack.heading_GET() == (char)43480);
            assert(pack.latitude_GET() == 1587087998);
            assert(pack.wp_num_GET() == (char)223);
            assert(pack.altitude_sp_GET() == (short) -13372);
            assert(pack.throttle_GET() == (byte)93);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            assert(pack.airspeed_GET() == (char)4);
            assert(pack.roll_GET() == (short) -10037);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.failsafe_GET() == (char)113);
            assert(pack.climb_rate_GET() == (byte)126);
            assert(pack.groundspeed_GET() == (char)155);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
            assert(pack.battery_remaining_GET() == (char)34);
            assert(pack.temperature_GET() == (byte)100);
            assert(pack.longitude_GET() == 1667605143);
            assert(pack.wp_distance_GET() == (char)53574);
            assert(pack.temperature_air_GET() == (byte) - 122);
            assert(pack.heading_sp_GET() == (short)24774);
            assert(pack.airspeed_sp_GET() == (char)180);
            assert(pack.altitude_amsl_GET() == (short)11363);
            assert(pack.custom_mode_GET() == 3339569464L);
            assert(pack.gps_nsat_GET() == (char)190);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.altitude_amsl_SET((short)11363) ;
        p234.airspeed_SET((char)4) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX) ;
        p234.pitch_SET((short) -12388) ;
        p234.altitude_sp_SET((short) -13372) ;
        p234.throttle_SET((byte)93) ;
        p234.temperature_SET((byte)100) ;
        p234.roll_SET((short) -10037) ;
        p234.climb_rate_SET((byte)126) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED) ;
        p234.failsafe_SET((char)113) ;
        p234.custom_mode_SET(3339569464L) ;
        p234.groundspeed_SET((char)155) ;
        p234.heading_sp_SET((short)24774) ;
        p234.gps_nsat_SET((char)190) ;
        p234.temperature_air_SET((byte) - 122) ;
        p234.latitude_SET(1587087998) ;
        p234.airspeed_sp_SET((char)180) ;
        p234.wp_num_SET((char)223) ;
        p234.battery_remaining_SET((char)34) ;
        p234.heading_SET((char)43480) ;
        p234.wp_distance_SET((char)53574) ;
        p234.longitude_SET(1667605143) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5352032721022676971L);
            assert(pack.clipping_2_GET() == 1135804143L);
            assert(pack.clipping_0_GET() == 1066182844L);
            assert(pack.clipping_1_GET() == 2390790947L);
            assert(pack.vibration_z_GET() == -2.876847E37F);
            assert(pack.vibration_y_GET() == 2.0356264E38F);
            assert(pack.vibration_x_GET() == -1.2343612E38F);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(5352032721022676971L) ;
        p241.vibration_x_SET(-1.2343612E38F) ;
        p241.clipping_0_SET(1066182844L) ;
        p241.vibration_z_SET(-2.876847E37F) ;
        p241.clipping_1_SET(2390790947L) ;
        p241.clipping_2_SET(1135804143L) ;
        p241.vibration_y_SET(2.0356264E38F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == 2.9055929E38F);
            assert(pack.z_GET() == -3.2467708E38F);
            assert(pack.longitude_GET() == -557987080);
            assert(pack.x_GET() == 4.134439E37F);
            assert(pack.altitude_GET() == -193480214);
            assert(pack.approach_y_GET() == 5.884327E37F);
            assert(pack.y_GET() == 2.7991727E38F);
            assert(pack.latitude_GET() == 341492819);
            assert(pack.approach_z_GET() == 2.8501727E37F);
            assert(pack.time_usec_TRY(ph) == 1238078099770179666L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.0144195E38F, -2.149876E38F, -1.6578845E37F, -3.2138965E38F}));
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(5.884327E37F) ;
        p242.q_SET(new float[] {2.0144195E38F, -2.149876E38F, -1.6578845E37F, -3.2138965E38F}, 0) ;
        p242.y_SET(2.7991727E38F) ;
        p242.approach_x_SET(2.9055929E38F) ;
        p242.z_SET(-3.2467708E38F) ;
        p242.approach_z_SET(2.8501727E37F) ;
        p242.latitude_SET(341492819) ;
        p242.x_SET(4.134439E37F) ;
        p242.longitude_SET(-557987080) ;
        p242.altitude_SET(-193480214) ;
        p242.time_usec_SET(1238078099770179666L, PH) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_y_GET() == -4.5891446E37F);
            assert(pack.x_GET() == 5.9131717E37F);
            assert(pack.latitude_GET() == -505813923);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.5014304E38F, -3.0112845E38F, -4.747414E37F, 8.110028E37F}));
            assert(pack.approach_z_GET() == 2.8819953E38F);
            assert(pack.longitude_GET() == 1491116012);
            assert(pack.target_system_GET() == (char)15);
            assert(pack.altitude_GET() == -1179667452);
            assert(pack.approach_x_GET() == 5.6327334E37F);
            assert(pack.time_usec_TRY(ph) == 4413896063570956320L);
            assert(pack.z_GET() == 1.4510279E38F);
            assert(pack.y_GET() == -2.9461285E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_y_SET(-4.5891446E37F) ;
        p243.x_SET(5.9131717E37F) ;
        p243.target_system_SET((char)15) ;
        p243.z_SET(1.4510279E38F) ;
        p243.latitude_SET(-505813923) ;
        p243.time_usec_SET(4413896063570956320L, PH) ;
        p243.approach_x_SET(5.6327334E37F) ;
        p243.altitude_SET(-1179667452) ;
        p243.y_SET(-2.9461285E38F) ;
        p243.q_SET(new float[] {-2.5014304E38F, -3.0112845E38F, -4.747414E37F, 8.110028E37F}, 0) ;
        p243.approach_z_SET(2.8819953E38F) ;
        p243.longitude_SET(1491116012) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)18769);
            assert(pack.interval_us_GET() == -1903655269);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(-1903655269) ;
        p244.message_id_SET((char)18769) ;
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
            assert(pack.lon_GET() == -1232731560);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.squawk_GET() == (char)39942);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
            assert(pack.heading_GET() == (char)32844);
            assert(pack.hor_velocity_GET() == (char)36583);
            assert(pack.lat_GET() == 994872261);
            assert(pack.altitude_GET() == -883057984);
            assert(pack.callsign_LEN(ph) == 1);
            assert(pack.callsign_TRY(ph).equals("v"));
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            assert(pack.tslc_GET() == (char)21);
            assert(pack.ver_velocity_GET() == (short) -11838);
            assert(pack.ICAO_address_GET() == 4104354951L);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.altitude_SET(-883057984) ;
        p246.lat_SET(994872261) ;
        p246.tslc_SET((char)21) ;
        p246.hor_velocity_SET((char)36583) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.ICAO_address_SET(4104354951L) ;
        p246.heading_SET((char)32844) ;
        p246.lon_SET(-1232731560) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE) ;
        p246.callsign_SET("v", PH) ;
        p246.squawk_SET((char)39942) ;
        p246.ver_velocity_SET((short) -11838) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_SIMULATED) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.time_to_minimum_delta_GET() == -2.5613292E38F);
            assert(pack.id_GET() == 4018072373L);
            assert(pack.horizontal_minimum_delta_GET() == 2.523501E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            assert(pack.altitude_minimum_delta_GET() == -2.1970693E38F);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.id_SET(4018072373L) ;
        p247.time_to_minimum_delta_SET(-2.5613292E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH) ;
        p247.altitude_minimum_delta_SET(-2.1970693E38F) ;
        p247.horizontal_minimum_delta_SET(2.523501E37F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)22);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)98, (char)98, (char)82, (char)238, (char)225, (char)207, (char)202, (char)175, (char)113, (char)143, (char)96, (char)243, (char)158, (char)245, (char)35, (char)70, (char)118, (char)229, (char)22, (char)180, (char)137, (char)49, (char)228, (char)130, (char)118, (char)59, (char)72, (char)222, (char)100, (char)82, (char)211, (char)92, (char)75, (char)250, (char)68, (char)232, (char)115, (char)52, (char)28, (char)168, (char)163, (char)173, (char)43, (char)35, (char)200, (char)55, (char)98, (char)235, (char)127, (char)41, (char)11, (char)25, (char)52, (char)230, (char)156, (char)82, (char)220, (char)9, (char)101, (char)5, (char)125, (char)94, (char)169, (char)180, (char)78, (char)244, (char)150, (char)113, (char)75, (char)209, (char)213, (char)226, (char)167, (char)238, (char)135, (char)61, (char)179, (char)38, (char)227, (char)100, (char)90, (char)11, (char)92, (char)88, (char)155, (char)127, (char)19, (char)13, (char)171, (char)32, (char)156, (char)49, (char)183, (char)154, (char)70, (char)49, (char)190, (char)203, (char)19, (char)62, (char)85, (char)177, (char)123, (char)131, (char)99, (char)6, (char)115, (char)58, (char)26, (char)54, (char)33, (char)245, (char)237, (char)161, (char)143, (char)146, (char)211, (char)243, (char)166, (char)128, (char)95, (char)40, (char)49, (char)197, (char)122, (char)205, (char)214, (char)243, (char)206, (char)100, (char)231, (char)168, (char)196, (char)47, (char)207, (char)255, (char)135, (char)55, (char)66, (char)192, (char)22, (char)238, (char)114, (char)172, (char)112, (char)178, (char)201, (char)240, (char)78, (char)137, (char)29, (char)89, (char)211, (char)22, (char)21, (char)133, (char)193, (char)198, (char)237, (char)253, (char)196, (char)0, (char)87, (char)181, (char)69, (char)64, (char)78, (char)62, (char)71, (char)32, (char)166, (char)63, (char)251, (char)176, (char)35, (char)93, (char)122, (char)180, (char)173, (char)56, (char)159, (char)138, (char)120, (char)45, (char)94, (char)201, (char)3, (char)176, (char)224, (char)141, (char)225, (char)189, (char)135, (char)143, (char)0, (char)15, (char)7, (char)29, (char)244, (char)11, (char)16, (char)85, (char)136, (char)4, (char)143, (char)80, (char)45, (char)166, (char)123, (char)30, (char)130, (char)17, (char)159, (char)171, (char)224, (char)70, (char)253, (char)26, (char)78, (char)48, (char)1, (char)230, (char)75, (char)55, (char)210, (char)188, (char)211, (char)19, (char)193, (char)65, (char)237, (char)192, (char)131, (char)175, (char)21, (char)159, (char)150, (char)189, (char)83, (char)174, (char)92, (char)165, (char)51, (char)185, (char)49, (char)175, (char)154, (char)210, (char)155}));
            assert(pack.target_system_GET() == (char)242);
            assert(pack.message_type_GET() == (char)47936);
            assert(pack.target_network_GET() == (char)159);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)22) ;
        p248.message_type_SET((char)47936) ;
        p248.target_network_SET((char)159) ;
        p248.target_system_SET((char)242) ;
        p248.payload_SET(new char[] {(char)98, (char)98, (char)82, (char)238, (char)225, (char)207, (char)202, (char)175, (char)113, (char)143, (char)96, (char)243, (char)158, (char)245, (char)35, (char)70, (char)118, (char)229, (char)22, (char)180, (char)137, (char)49, (char)228, (char)130, (char)118, (char)59, (char)72, (char)222, (char)100, (char)82, (char)211, (char)92, (char)75, (char)250, (char)68, (char)232, (char)115, (char)52, (char)28, (char)168, (char)163, (char)173, (char)43, (char)35, (char)200, (char)55, (char)98, (char)235, (char)127, (char)41, (char)11, (char)25, (char)52, (char)230, (char)156, (char)82, (char)220, (char)9, (char)101, (char)5, (char)125, (char)94, (char)169, (char)180, (char)78, (char)244, (char)150, (char)113, (char)75, (char)209, (char)213, (char)226, (char)167, (char)238, (char)135, (char)61, (char)179, (char)38, (char)227, (char)100, (char)90, (char)11, (char)92, (char)88, (char)155, (char)127, (char)19, (char)13, (char)171, (char)32, (char)156, (char)49, (char)183, (char)154, (char)70, (char)49, (char)190, (char)203, (char)19, (char)62, (char)85, (char)177, (char)123, (char)131, (char)99, (char)6, (char)115, (char)58, (char)26, (char)54, (char)33, (char)245, (char)237, (char)161, (char)143, (char)146, (char)211, (char)243, (char)166, (char)128, (char)95, (char)40, (char)49, (char)197, (char)122, (char)205, (char)214, (char)243, (char)206, (char)100, (char)231, (char)168, (char)196, (char)47, (char)207, (char)255, (char)135, (char)55, (char)66, (char)192, (char)22, (char)238, (char)114, (char)172, (char)112, (char)178, (char)201, (char)240, (char)78, (char)137, (char)29, (char)89, (char)211, (char)22, (char)21, (char)133, (char)193, (char)198, (char)237, (char)253, (char)196, (char)0, (char)87, (char)181, (char)69, (char)64, (char)78, (char)62, (char)71, (char)32, (char)166, (char)63, (char)251, (char)176, (char)35, (char)93, (char)122, (char)180, (char)173, (char)56, (char)159, (char)138, (char)120, (char)45, (char)94, (char)201, (char)3, (char)176, (char)224, (char)141, (char)225, (char)189, (char)135, (char)143, (char)0, (char)15, (char)7, (char)29, (char)244, (char)11, (char)16, (char)85, (char)136, (char)4, (char)143, (char)80, (char)45, (char)166, (char)123, (char)30, (char)130, (char)17, (char)159, (char)171, (char)224, (char)70, (char)253, (char)26, (char)78, (char)48, (char)1, (char)230, (char)75, (char)55, (char)210, (char)188, (char)211, (char)19, (char)193, (char)65, (char)237, (char)192, (char)131, (char)175, (char)21, (char)159, (char)150, (char)189, (char)83, (char)174, (char)92, (char)165, (char)51, (char)185, (char)49, (char)175, (char)154, (char)210, (char)155}, 0) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)10989);
            assert(pack.type_GET() == (char)136);
            assert(pack.ver_GET() == (char)162);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 24, (byte) - 113, (byte)80, (byte) - 52, (byte) - 26, (byte) - 112, (byte) - 58, (byte) - 59, (byte) - 87, (byte) - 32, (byte)12, (byte)25, (byte) - 7, (byte) - 109, (byte)106, (byte) - 121, (byte)29, (byte)122, (byte)51, (byte) - 102, (byte) - 35, (byte) - 77, (byte) - 107, (byte) - 107, (byte)20, (byte)2, (byte)47, (byte) - 90, (byte) - 29, (byte)48, (byte) - 78, (byte)123}));
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)10989) ;
        p249.ver_SET((char)162) ;
        p249.value_SET(new byte[] {(byte) - 24, (byte) - 113, (byte)80, (byte) - 52, (byte) - 26, (byte) - 112, (byte) - 58, (byte) - 59, (byte) - 87, (byte) - 32, (byte)12, (byte)25, (byte) - 7, (byte) - 109, (byte)106, (byte) - 121, (byte)29, (byte)122, (byte)51, (byte) - 102, (byte) - 35, (byte) - 77, (byte) - 107, (byte) - 107, (byte)20, (byte)2, (byte)47, (byte) - 90, (byte) - 29, (byte)48, (byte) - 78, (byte)123}, 0) ;
        p249.type_SET((char)136) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.6543015E38F);
            assert(pack.y_GET() == 2.4974911E38F);
            assert(pack.x_GET() == 1.5185465E38F);
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("akbzsmob"));
            assert(pack.time_usec_GET() == 7591461093748959590L);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.z_SET(-2.6543015E38F) ;
        p250.x_SET(1.5185465E38F) ;
        p250.y_SET(2.4974911E38F) ;
        p250.name_SET("akbzsmob", PH) ;
        p250.time_usec_SET(7591461093748959590L) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 262630798L);
            assert(pack.value_GET() == 1.1240856E37F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("fqvfkoc"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("fqvfkoc", PH) ;
        p251.value_SET(1.1240856E37F) ;
        p251.time_boot_ms_SET(262630798L) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2203677666L);
            assert(pack.value_GET() == 156818005);
            assert(pack.name_LEN(ph) == 6);
            assert(pack.name_TRY(ph).equals("qnssji"));
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("qnssji", PH) ;
        p252.time_boot_ms_SET(2203677666L) ;
        p252.value_SET(156818005) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_ALERT);
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("xraibpjcYBSzqwldbsoiueolnCtkqnv"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("xraibpjcYBSzqwldbsoiueolnCtkqnv", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.8523307E38F);
            assert(pack.ind_GET() == (char)13);
            assert(pack.time_boot_ms_GET() == 508835880L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.value_SET(-2.8523307E38F) ;
        p254.ind_SET((char)13) ;
        p254.time_boot_ms_SET(508835880L) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)129);
            assert(pack.initial_timestamp_GET() == 3097700524908884885L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)137, (char)106, (char)165, (char)250, (char)123, (char)91, (char)34, (char)177, (char)239, (char)40, (char)61, (char)139, (char)116, (char)126, (char)241, (char)39, (char)146, (char)28, (char)159, (char)157, (char)190, (char)140, (char)78, (char)214, (char)102, (char)213, (char)11, (char)223, (char)119, (char)64, (char)159, (char)165}));
            assert(pack.target_component_GET() == (char)56);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(3097700524908884885L) ;
        p256.target_component_SET((char)56) ;
        p256.secret_key_SET(new char[] {(char)137, (char)106, (char)165, (char)250, (char)123, (char)91, (char)34, (char)177, (char)239, (char)40, (char)61, (char)139, (char)116, (char)126, (char)241, (char)39, (char)146, (char)28, (char)159, (char)157, (char)190, (char)140, (char)78, (char)214, (char)102, (char)213, (char)11, (char)223, (char)119, (char)64, (char)159, (char)165}, 0) ;
        p256.target_system_SET((char)129) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3469581531L);
            assert(pack.state_GET() == (char)121);
            assert(pack.last_change_ms_GET() == 233329153L);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3469581531L) ;
        p257.state_SET((char)121) ;
        p257.last_change_ms_SET(233329153L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.tune_LEN(ph) == 26);
            assert(pack.tune_TRY(ph).equals("uufskzmnorNflwiamuxcudbayh"));
            assert(pack.target_system_GET() == (char)170);
            assert(pack.target_component_GET() == (char)62);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)170) ;
        p258.target_component_SET((char)62) ;
        p258.tune_SET("uufskzmnorNflwiamuxcudbayh", PH) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.focal_length_GET() == 1.5481929E38F);
            assert(pack.cam_definition_version_GET() == (char)51052);
            assert(pack.resolution_v_GET() == (char)22950);
            assert(pack.lens_id_GET() == (char)251);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
            assert(pack.time_boot_ms_GET() == 3616559166L);
            assert(pack.sensor_size_h_GET() == 1.7926641E37F);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)221, (char)139, (char)5, (char)28, (char)179, (char)215, (char)42, (char)19, (char)34, (char)185, (char)126, (char)86, (char)4, (char)200, (char)9, (char)16, (char)137, (char)214, (char)243, (char)134, (char)199, (char)174, (char)157, (char)197, (char)61, (char)67, (char)142, (char)114, (char)239, (char)43, (char)29, (char)99}));
            assert(pack.resolution_h_GET() == (char)27786);
            assert(pack.cam_definition_uri_LEN(ph) == 19);
            assert(pack.cam_definition_uri_TRY(ph).equals("hepNoxejickbpplsxbd"));
            assert(pack.sensor_size_v_GET() == 1.0214062E38F);
            assert(pack.firmware_version_GET() == 2810917335L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)10, (char)57, (char)0, (char)29, (char)57, (char)128, (char)173, (char)163, (char)118, (char)117, (char)16, (char)40, (char)235, (char)149, (char)35, (char)38, (char)55, (char)237, (char)198, (char)63, (char)125, (char)127, (char)201, (char)11, (char)125, (char)70, (char)74, (char)214, (char)136, (char)61, (char)114, (char)113}));
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_uri_SET("hepNoxejickbpplsxbd", PH) ;
        p259.vendor_name_SET(new char[] {(char)10, (char)57, (char)0, (char)29, (char)57, (char)128, (char)173, (char)163, (char)118, (char)117, (char)16, (char)40, (char)235, (char)149, (char)35, (char)38, (char)55, (char)237, (char)198, (char)63, (char)125, (char)127, (char)201, (char)11, (char)125, (char)70, (char)74, (char)214, (char)136, (char)61, (char)114, (char)113}, 0) ;
        p259.model_name_SET(new char[] {(char)221, (char)139, (char)5, (char)28, (char)179, (char)215, (char)42, (char)19, (char)34, (char)185, (char)126, (char)86, (char)4, (char)200, (char)9, (char)16, (char)137, (char)214, (char)243, (char)134, (char)199, (char)174, (char)157, (char)197, (char)61, (char)67, (char)142, (char)114, (char)239, (char)43, (char)29, (char)99}, 0) ;
        p259.resolution_v_SET((char)22950) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES) ;
        p259.sensor_size_v_SET(1.0214062E38F) ;
        p259.cam_definition_version_SET((char)51052) ;
        p259.sensor_size_h_SET(1.7926641E37F) ;
        p259.focal_length_SET(1.5481929E38F) ;
        p259.lens_id_SET((char)251) ;
        p259.resolution_h_SET((char)27786) ;
        p259.time_boot_ms_SET(3616559166L) ;
        p259.firmware_version_SET(2810917335L) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 3166208265L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3166208265L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.available_capacity_GET() == -3.2545256E38F);
            assert(pack.status_GET() == (char)218);
            assert(pack.used_capacity_GET() == -2.8792262E38F);
            assert(pack.write_speed_GET() == 2.1929719E38F);
            assert(pack.total_capacity_GET() == -2.006036E38F);
            assert(pack.read_speed_GET() == -3.3377758E37F);
            assert(pack.storage_count_GET() == (char)197);
            assert(pack.time_boot_ms_GET() == 3599812270L);
            assert(pack.storage_id_GET() == (char)122);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.total_capacity_SET(-2.006036E38F) ;
        p261.read_speed_SET(-3.3377758E37F) ;
        p261.write_speed_SET(2.1929719E38F) ;
        p261.storage_id_SET((char)122) ;
        p261.used_capacity_SET(-2.8792262E38F) ;
        p261.available_capacity_SET(-3.2545256E38F) ;
        p261.storage_count_SET((char)197) ;
        p261.time_boot_ms_SET(3599812270L) ;
        p261.status_SET((char)218) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4267176493L);
            assert(pack.recording_time_ms_GET() == 1953327869L);
            assert(pack.video_status_GET() == (char)83);
            assert(pack.image_interval_GET() == 2.5916937E37F);
            assert(pack.available_capacity_GET() == -6.559923E37F);
            assert(pack.image_status_GET() == (char)149);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.video_status_SET((char)83) ;
        p262.time_boot_ms_SET(4267176493L) ;
        p262.image_status_SET((char)149) ;
        p262.available_capacity_SET(-6.559923E37F) ;
        p262.recording_time_ms_SET(1953327869L) ;
        p262.image_interval_SET(2.5916937E37F) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.2165634E38F, 9.803378E37F, -1.3578982E37F, -2.681129E37F}));
            assert(pack.alt_GET() == -486824160);
            assert(pack.lat_GET() == -261618868);
            assert(pack.relative_alt_GET() == -1470301333);
            assert(pack.camera_id_GET() == (char)228);
            assert(pack.file_url_LEN(ph) == 33);
            assert(pack.file_url_TRY(ph).equals("qdifaybeatpCklgdcCssibighIVWzNare"));
            assert(pack.time_utc_GET() == 4249640573426657066L);
            assert(pack.capture_result_GET() == (byte)85);
            assert(pack.lon_GET() == -1173826931);
            assert(pack.time_boot_ms_GET() == 233425236L);
            assert(pack.image_index_GET() == -789192298);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.q_SET(new float[] {-3.2165634E38F, 9.803378E37F, -1.3578982E37F, -2.681129E37F}, 0) ;
        p263.image_index_SET(-789192298) ;
        p263.lat_SET(-261618868) ;
        p263.lon_SET(-1173826931) ;
        p263.file_url_SET("qdifaybeatpCklgdcCssibighIVWzNare", PH) ;
        p263.camera_id_SET((char)228) ;
        p263.time_utc_SET(4249640573426657066L) ;
        p263.capture_result_SET((byte)85) ;
        p263.time_boot_ms_SET(233425236L) ;
        p263.alt_SET(-486824160) ;
        p263.relative_alt_SET(-1470301333) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.takeoff_time_utc_GET() == 4538820941556553728L);
            assert(pack.arming_time_utc_GET() == 5866662553947360720L);
            assert(pack.flight_uuid_GET() == 8857009014727880013L);
            assert(pack.time_boot_ms_GET() == 3105597825L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(5866662553947360720L) ;
        p264.flight_uuid_SET(8857009014727880013L) ;
        p264.takeoff_time_utc_SET(4538820941556553728L) ;
        p264.time_boot_ms_SET(3105597825L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.4167731E38F);
            assert(pack.roll_GET() == 1.7064273E38F);
            assert(pack.time_boot_ms_GET() == 3944160668L);
            assert(pack.yaw_GET() == 3.2584667E38F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.yaw_SET(3.2584667E38F) ;
        p265.roll_SET(1.7064273E38F) ;
        p265.pitch_SET(-1.4167731E38F) ;
        p265.time_boot_ms_SET(3944160668L) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)97);
            assert(pack.sequence_GET() == (char)41745);
            assert(pack.target_component_GET() == (char)251);
            assert(pack.first_message_offset_GET() == (char)79);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)153, (char)36, (char)22, (char)241, (char)154, (char)196, (char)204, (char)160, (char)203, (char)199, (char)188, (char)109, (char)138, (char)43, (char)130, (char)187, (char)45, (char)254, (char)0, (char)50, (char)214, (char)131, (char)65, (char)86, (char)11, (char)60, (char)8, (char)92, (char)122, (char)66, (char)199, (char)198, (char)247, (char)181, (char)226, (char)89, (char)105, (char)100, (char)246, (char)33, (char)145, (char)123, (char)4, (char)232, (char)2, (char)217, (char)165, (char)47, (char)68, (char)181, (char)213, (char)56, (char)67, (char)62, (char)122, (char)145, (char)30, (char)97, (char)133, (char)140, (char)134, (char)140, (char)114, (char)154, (char)103, (char)227, (char)67, (char)27, (char)237, (char)168, (char)245, (char)43, (char)15, (char)180, (char)161, (char)72, (char)32, (char)30, (char)66, (char)38, (char)179, (char)128, (char)240, (char)197, (char)145, (char)84, (char)234, (char)126, (char)50, (char)143, (char)54, (char)109, (char)61, (char)28, (char)104, (char)75, (char)22, (char)205, (char)180, (char)29, (char)51, (char)208, (char)125, (char)253, (char)153, (char)72, (char)1, (char)122, (char)72, (char)253, (char)69, (char)100, (char)155, (char)248, (char)151, (char)255, (char)176, (char)33, (char)150, (char)132, (char)9, (char)28, (char)132, (char)198, (char)180, (char)84, (char)229, (char)205, (char)105, (char)133, (char)73, (char)17, (char)252, (char)32, (char)38, (char)29, (char)155, (char)73, (char)56, (char)132, (char)128, (char)181, (char)2, (char)112, (char)96, (char)204, (char)210, (char)149, (char)6, (char)246, (char)170, (char)108, (char)219, (char)13, (char)10, (char)242, (char)177, (char)165, (char)197, (char)178, (char)204, (char)250, (char)191, (char)26, (char)144, (char)38, (char)223, (char)38, (char)119, (char)184, (char)15, (char)129, (char)59, (char)66, (char)125, (char)118, (char)173, (char)70, (char)12, (char)171, (char)14, (char)27, (char)51, (char)22, (char)254, (char)248, (char)30, (char)137, (char)205, (char)117, (char)208, (char)19, (char)159, (char)108, (char)42, (char)23, (char)141, (char)29, (char)74, (char)93, (char)48, (char)213, (char)18, (char)156, (char)12, (char)112, (char)128, (char)240, (char)202, (char)147, (char)10, (char)7, (char)170, (char)43, (char)23, (char)248, (char)185, (char)161, (char)17, (char)135, (char)228, (char)201, (char)118, (char)243, (char)22, (char)15, (char)58, (char)177, (char)5, (char)195, (char)248, (char)134, (char)12, (char)246, (char)155, (char)145, (char)99, (char)233, (char)206, (char)237, (char)206, (char)49, (char)43, (char)40, (char)12, (char)65, (char)60, (char)240, (char)220}));
            assert(pack.length_GET() == (char)53);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_component_SET((char)251) ;
        p266.sequence_SET((char)41745) ;
        p266.length_SET((char)53) ;
        p266.data__SET(new char[] {(char)153, (char)36, (char)22, (char)241, (char)154, (char)196, (char)204, (char)160, (char)203, (char)199, (char)188, (char)109, (char)138, (char)43, (char)130, (char)187, (char)45, (char)254, (char)0, (char)50, (char)214, (char)131, (char)65, (char)86, (char)11, (char)60, (char)8, (char)92, (char)122, (char)66, (char)199, (char)198, (char)247, (char)181, (char)226, (char)89, (char)105, (char)100, (char)246, (char)33, (char)145, (char)123, (char)4, (char)232, (char)2, (char)217, (char)165, (char)47, (char)68, (char)181, (char)213, (char)56, (char)67, (char)62, (char)122, (char)145, (char)30, (char)97, (char)133, (char)140, (char)134, (char)140, (char)114, (char)154, (char)103, (char)227, (char)67, (char)27, (char)237, (char)168, (char)245, (char)43, (char)15, (char)180, (char)161, (char)72, (char)32, (char)30, (char)66, (char)38, (char)179, (char)128, (char)240, (char)197, (char)145, (char)84, (char)234, (char)126, (char)50, (char)143, (char)54, (char)109, (char)61, (char)28, (char)104, (char)75, (char)22, (char)205, (char)180, (char)29, (char)51, (char)208, (char)125, (char)253, (char)153, (char)72, (char)1, (char)122, (char)72, (char)253, (char)69, (char)100, (char)155, (char)248, (char)151, (char)255, (char)176, (char)33, (char)150, (char)132, (char)9, (char)28, (char)132, (char)198, (char)180, (char)84, (char)229, (char)205, (char)105, (char)133, (char)73, (char)17, (char)252, (char)32, (char)38, (char)29, (char)155, (char)73, (char)56, (char)132, (char)128, (char)181, (char)2, (char)112, (char)96, (char)204, (char)210, (char)149, (char)6, (char)246, (char)170, (char)108, (char)219, (char)13, (char)10, (char)242, (char)177, (char)165, (char)197, (char)178, (char)204, (char)250, (char)191, (char)26, (char)144, (char)38, (char)223, (char)38, (char)119, (char)184, (char)15, (char)129, (char)59, (char)66, (char)125, (char)118, (char)173, (char)70, (char)12, (char)171, (char)14, (char)27, (char)51, (char)22, (char)254, (char)248, (char)30, (char)137, (char)205, (char)117, (char)208, (char)19, (char)159, (char)108, (char)42, (char)23, (char)141, (char)29, (char)74, (char)93, (char)48, (char)213, (char)18, (char)156, (char)12, (char)112, (char)128, (char)240, (char)202, (char)147, (char)10, (char)7, (char)170, (char)43, (char)23, (char)248, (char)185, (char)161, (char)17, (char)135, (char)228, (char)201, (char)118, (char)243, (char)22, (char)15, (char)58, (char)177, (char)5, (char)195, (char)248, (char)134, (char)12, (char)246, (char)155, (char)145, (char)99, (char)233, (char)206, (char)237, (char)206, (char)49, (char)43, (char)40, (char)12, (char)65, (char)60, (char)240, (char)220}, 0) ;
        p266.first_message_offset_SET((char)79) ;
        p266.target_system_SET((char)97) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)83, (char)118, (char)182, (char)92, (char)191, (char)97, (char)70, (char)191, (char)3, (char)67, (char)217, (char)119, (char)152, (char)14, (char)231, (char)48, (char)55, (char)221, (char)109, (char)198, (char)164, (char)34, (char)170, (char)37, (char)198, (char)208, (char)37, (char)23, (char)55, (char)50, (char)232, (char)227, (char)170, (char)149, (char)182, (char)84, (char)74, (char)91, (char)14, (char)64, (char)38, (char)62, (char)173, (char)184, (char)14, (char)238, (char)205, (char)239, (char)101, (char)79, (char)217, (char)228, (char)209, (char)160, (char)173, (char)117, (char)227, (char)17, (char)225, (char)93, (char)40, (char)205, (char)2, (char)203, (char)30, (char)169, (char)224, (char)104, (char)56, (char)188, (char)163, (char)38, (char)89, (char)30, (char)109, (char)186, (char)181, (char)216, (char)244, (char)238, (char)131, (char)180, (char)31, (char)25, (char)68, (char)149, (char)55, (char)110, (char)67, (char)224, (char)43, (char)92, (char)210, (char)241, (char)113, (char)88, (char)54, (char)80, (char)140, (char)174, (char)45, (char)59, (char)155, (char)245, (char)76, (char)4, (char)161, (char)118, (char)164, (char)150, (char)221, (char)69, (char)63, (char)179, (char)193, (char)170, (char)195, (char)239, (char)58, (char)85, (char)167, (char)60, (char)219, (char)175, (char)135, (char)33, (char)78, (char)249, (char)149, (char)188, (char)201, (char)133, (char)205, (char)104, (char)165, (char)132, (char)35, (char)193, (char)120, (char)227, (char)26, (char)175, (char)190, (char)252, (char)83, (char)104, (char)75, (char)103, (char)52, (char)185, (char)222, (char)187, (char)129, (char)220, (char)157, (char)206, (char)193, (char)2, (char)4, (char)115, (char)56, (char)211, (char)47, (char)151, (char)137, (char)29, (char)172, (char)80, (char)209, (char)253, (char)225, (char)237, (char)145, (char)19, (char)173, (char)93, (char)96, (char)78, (char)118, (char)202, (char)190, (char)249, (char)114, (char)172, (char)22, (char)238, (char)156, (char)54, (char)108, (char)221, (char)228, (char)241, (char)47, (char)242, (char)223, (char)46, (char)87, (char)147, (char)198, (char)65, (char)82, (char)216, (char)86, (char)109, (char)223, (char)12, (char)4, (char)145, (char)153, (char)215, (char)156, (char)38, (char)71, (char)190, (char)110, (char)50, (char)91, (char)25, (char)74, (char)74, (char)219, (char)4, (char)0, (char)181, (char)182, (char)78, (char)127, (char)14, (char)129, (char)244, (char)14, (char)26, (char)169, (char)168, (char)88, (char)5, (char)146, (char)221, (char)226, (char)242, (char)246, (char)173, (char)153, (char)66, (char)106, (char)231, (char)132, (char)163, (char)135}));
            assert(pack.target_component_GET() == (char)144);
            assert(pack.length_GET() == (char)239);
            assert(pack.first_message_offset_GET() == (char)214);
            assert(pack.target_system_GET() == (char)240);
            assert(pack.sequence_GET() == (char)7080);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)214) ;
        p267.length_SET((char)239) ;
        p267.data__SET(new char[] {(char)83, (char)118, (char)182, (char)92, (char)191, (char)97, (char)70, (char)191, (char)3, (char)67, (char)217, (char)119, (char)152, (char)14, (char)231, (char)48, (char)55, (char)221, (char)109, (char)198, (char)164, (char)34, (char)170, (char)37, (char)198, (char)208, (char)37, (char)23, (char)55, (char)50, (char)232, (char)227, (char)170, (char)149, (char)182, (char)84, (char)74, (char)91, (char)14, (char)64, (char)38, (char)62, (char)173, (char)184, (char)14, (char)238, (char)205, (char)239, (char)101, (char)79, (char)217, (char)228, (char)209, (char)160, (char)173, (char)117, (char)227, (char)17, (char)225, (char)93, (char)40, (char)205, (char)2, (char)203, (char)30, (char)169, (char)224, (char)104, (char)56, (char)188, (char)163, (char)38, (char)89, (char)30, (char)109, (char)186, (char)181, (char)216, (char)244, (char)238, (char)131, (char)180, (char)31, (char)25, (char)68, (char)149, (char)55, (char)110, (char)67, (char)224, (char)43, (char)92, (char)210, (char)241, (char)113, (char)88, (char)54, (char)80, (char)140, (char)174, (char)45, (char)59, (char)155, (char)245, (char)76, (char)4, (char)161, (char)118, (char)164, (char)150, (char)221, (char)69, (char)63, (char)179, (char)193, (char)170, (char)195, (char)239, (char)58, (char)85, (char)167, (char)60, (char)219, (char)175, (char)135, (char)33, (char)78, (char)249, (char)149, (char)188, (char)201, (char)133, (char)205, (char)104, (char)165, (char)132, (char)35, (char)193, (char)120, (char)227, (char)26, (char)175, (char)190, (char)252, (char)83, (char)104, (char)75, (char)103, (char)52, (char)185, (char)222, (char)187, (char)129, (char)220, (char)157, (char)206, (char)193, (char)2, (char)4, (char)115, (char)56, (char)211, (char)47, (char)151, (char)137, (char)29, (char)172, (char)80, (char)209, (char)253, (char)225, (char)237, (char)145, (char)19, (char)173, (char)93, (char)96, (char)78, (char)118, (char)202, (char)190, (char)249, (char)114, (char)172, (char)22, (char)238, (char)156, (char)54, (char)108, (char)221, (char)228, (char)241, (char)47, (char)242, (char)223, (char)46, (char)87, (char)147, (char)198, (char)65, (char)82, (char)216, (char)86, (char)109, (char)223, (char)12, (char)4, (char)145, (char)153, (char)215, (char)156, (char)38, (char)71, (char)190, (char)110, (char)50, (char)91, (char)25, (char)74, (char)74, (char)219, (char)4, (char)0, (char)181, (char)182, (char)78, (char)127, (char)14, (char)129, (char)244, (char)14, (char)26, (char)169, (char)168, (char)88, (char)5, (char)146, (char)221, (char)226, (char)242, (char)246, (char)173, (char)153, (char)66, (char)106, (char)231, (char)132, (char)163, (char)135}, 0) ;
        p267.target_system_SET((char)240) ;
        p267.target_component_SET((char)144) ;
        p267.sequence_SET((char)7080) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)153);
            assert(pack.sequence_GET() == (char)47253);
            assert(pack.target_system_GET() == (char)193);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_component_SET((char)153) ;
        p268.sequence_SET((char)47253) ;
        p268.target_system_SET((char)193) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)31105);
            assert(pack.bitrate_GET() == 2573013918L);
            assert(pack.uri_LEN(ph) == 32);
            assert(pack.uri_TRY(ph).equals("ugknsdhoqLsxhqETitipThfekpwDhvho"));
            assert(pack.framerate_GET() == 6.758868E37F);
            assert(pack.resolution_h_GET() == (char)64387);
            assert(pack.rotation_GET() == (char)64553);
            assert(pack.status_GET() == (char)89);
            assert(pack.camera_id_GET() == (char)150);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_h_SET((char)64387) ;
        p269.rotation_SET((char)64553) ;
        p269.uri_SET("ugknsdhoqLsxhqETitipThfekpwDhvho", PH) ;
        p269.bitrate_SET(2573013918L) ;
        p269.framerate_SET(6.758868E37F) ;
        p269.status_SET((char)89) ;
        p269.camera_id_SET((char)150) ;
        p269.resolution_v_SET((char)31105) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)64301);
            assert(pack.bitrate_GET() == 2296470424L);
            assert(pack.camera_id_GET() == (char)254);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.resolution_v_GET() == (char)58543);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.uri_LEN(ph) == 227);
            assert(pack.uri_TRY(ph).equals("pwrbtdwibuiwnBToriuyhvriscyodVYjynbphcweqqbzlraAyjxsikgozvzlaQlXaUdlysnOoaoeovkJeztivcgllvspfquSruurChkPgeejnNxizhqnzLzlgffqjaznlkawlsjgpqoaozciyxmnygmkkphzjfsbcwpedSlanhgzvtxnnuptmbkdgjczSyrzkzsaqmxnqtjVdpjzfyajsflyjqKbsvjbkgu"));
            assert(pack.rotation_GET() == (char)17493);
            assert(pack.framerate_GET() == -7.8239314E37F);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)17) ;
        p270.framerate_SET(-7.8239314E37F) ;
        p270.resolution_v_SET((char)58543) ;
        p270.bitrate_SET(2296470424L) ;
        p270.rotation_SET((char)17493) ;
        p270.uri_SET("pwrbtdwibuiwnBToriuyhvriscyodVYjynbphcweqqbzlraAyjxsikgozvzlaQlXaUdlysnOoaoeovkJeztivcgllvspfquSruurChkPgeejnNxizhqnzLzlgffqjaznlkawlsjgpqoaozciyxmnygmkkphzjfsbcwpedSlanhgzvtxnnuptmbkdgjczSyrzkzsaqmxnqtjVdpjzfyajsflyjqKbsvjbkgu", PH) ;
        p270.target_component_SET((char)222) ;
        p270.camera_id_SET((char)254) ;
        p270.resolution_h_SET((char)64301) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 6);
            assert(pack.ssid_TRY(ph).equals("kllrwe"));
            assert(pack.password_LEN(ph) == 64);
            assert(pack.password_TRY(ph).equals("qajqsrixxeansZgRcqsijcaVpXucNqjjjkpianrwynzljtixsXanjsNadfvahlqg"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("kllrwe", PH) ;
        p299.password_SET("qajqsrixxeansZgRcqsijcaVpXucNqjjjkpianrwynzljtixsXanjsNadfvahlqg", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)61, (char)47, (char)243, (char)67, (char)42, (char)73, (char)12, (char)49}));
            assert(pack.version_GET() == (char)5490);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)206, (char)202, (char)143, (char)93, (char)66, (char)4, (char)64, (char)143}));
            assert(pack.min_version_GET() == (char)22800);
            assert(pack.max_version_GET() == (char)63184);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)5490) ;
        p300.spec_version_hash_SET(new char[] {(char)206, (char)202, (char)143, (char)93, (char)66, (char)4, (char)64, (char)143}, 0) ;
        p300.library_version_hash_SET(new char[] {(char)61, (char)47, (char)243, (char)67, (char)42, (char)73, (char)12, (char)49}, 0) ;
        p300.min_version_SET((char)22800) ;
        p300.max_version_SET((char)63184) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4252441568117754008L);
            assert(pack.sub_mode_GET() == (char)252);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
            assert(pack.vendor_specific_status_code_GET() == (char)155);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.uptime_sec_GET() == 2630988343L);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.sub_mode_SET((char)252) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.time_usec_SET(4252441568117754008L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL) ;
        p310.vendor_specific_status_code_SET((char)155) ;
        p310.uptime_sec_SET(2630988343L) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 3746439242L);
            assert(pack.time_usec_GET() == 8977771470508096027L);
            assert(pack.sw_vcs_commit_GET() == 1931198476L);
            assert(pack.sw_version_major_GET() == (char)227);
            assert(pack.sw_version_minor_GET() == (char)233);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)54, (char)125, (char)77, (char)81, (char)225, (char)208, (char)207, (char)68, (char)94, (char)92, (char)105, (char)183, (char)115, (char)105, (char)220, (char)92}));
            assert(pack.name_LEN(ph) == 66);
            assert(pack.name_TRY(ph).equals("xhZJwGhhrditjksmtBhfrklbtCclcqhsyfEarrvbdbDmpkejjjhmsxgAfXnokpixiA"));
            assert(pack.hw_version_major_GET() == (char)130);
            assert(pack.hw_version_minor_GET() == (char)192);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_unique_id_SET(new char[] {(char)54, (char)125, (char)77, (char)81, (char)225, (char)208, (char)207, (char)68, (char)94, (char)92, (char)105, (char)183, (char)115, (char)105, (char)220, (char)92}, 0) ;
        p311.sw_version_major_SET((char)227) ;
        p311.sw_vcs_commit_SET(1931198476L) ;
        p311.hw_version_minor_SET((char)192) ;
        p311.sw_version_minor_SET((char)233) ;
        p311.time_usec_SET(8977771470508096027L) ;
        p311.hw_version_major_SET((char)130) ;
        p311.uptime_sec_SET(3746439242L) ;
        p311.name_SET("xhZJwGhhrditjksmtBhfrklbtCclcqhsyfEarrvbdbDmpkejjjhmsxgAfXnokpixiA", PH) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 10);
            assert(pack.param_id_TRY(ph).equals("drngrjuzgt"));
            assert(pack.param_index_GET() == (short)19869);
            assert(pack.target_system_GET() == (char)239);
            assert(pack.target_component_GET() == (char)75);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("drngrjuzgt", PH) ;
        p320.param_index_SET((short)19869) ;
        p320.target_component_SET((char)75) ;
        p320.target_system_SET((char)239) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)92);
            assert(pack.target_system_GET() == (char)29);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)92) ;
        p321.target_system_SET((char)29) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 90);
            assert(pack.param_value_TRY(ph).equals("gdfgFnmjqxistotmzlpXbmmvbpcwzbqqvtpbclkqcxppDVotlVdwvughsqWlgmzbiqPrfcbqqDfilfckyaBevfkXnd"));
            assert(pack.param_count_GET() == (char)31094);
            assert(pack.param_index_GET() == (char)53112);
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("yhdr"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_value_SET("gdfgFnmjqxistotmzlpXbmmvbpcwzbqqvtpbclkqcxppDVotlVdwvughsqWlgmzbiqPrfcbqqDfilfckyaBevfkXnd", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p322.param_count_SET((char)31094) ;
        p322.param_id_SET("yhdr", PH) ;
        p322.param_index_SET((char)53112) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)225);
            assert(pack.target_system_GET() == (char)112);
            assert(pack.param_value_LEN(ph) == 60);
            assert(pack.param_value_TRY(ph).equals("gcnxqgpinwbfetfpxzgODtydvgsqdcijwpgttgtaQipGzuRkfNrDxkzhaugn"));
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("notxyoubPefrdgjm"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.param_id_SET("notxyoubPefrdgjm", PH) ;
        p323.target_component_SET((char)225) ;
        p323.target_system_SET((char)112) ;
        p323.param_value_SET("gcnxqgpinwbfetfpxzgODtydvgsqdcijwpgttgtaQipGzuRkfNrDxkzhaugn", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 90);
            assert(pack.param_value_TRY(ph).equals("mebldkpfevjwkUkznmfeetbsyhoIxmnmxengzhsjfbvqQhdlqglpoxmyttbvcpOusjmmytndzZraIhUuniiXayvilc"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            assert(pack.param_id_LEN(ph) == 15);
            assert(pack.param_id_TRY(ph).equals("yhaurtkpgrwmwrs"));
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("yhaurtkpgrwmwrs", PH) ;
        p324.param_value_SET("mebldkpfevjwkUkznmfeetbsyhoIxmnmxengzhsjfbvqQhdlqglpoxmyttbvcpOusjmmytndzZraIhUuniiXayvilc", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)20351, (char)1211, (char)5379, (char)6097, (char)48959, (char)13234, (char)59792, (char)44175, (char)23484, (char)38204, (char)23571, (char)61563, (char)6507, (char)38005, (char)61020, (char)51931, (char)8220, (char)51621, (char)25062, (char)39729, (char)65141, (char)54806, (char)53663, (char)46488, (char)20663, (char)26510, (char)61130, (char)2678, (char)7617, (char)62445, (char)1022, (char)39806, (char)7291, (char)28713, (char)54510, (char)9758, (char)28995, (char)25999, (char)36127, (char)42721, (char)5240, (char)15238, (char)43456, (char)45632, (char)28091, (char)9896, (char)19600, (char)2516, (char)15546, (char)42477, (char)61360, (char)46535, (char)51356, (char)55019, (char)45245, (char)18985, (char)42505, (char)18956, (char)23919, (char)59902, (char)26987, (char)19506, (char)13309, (char)15337, (char)62356, (char)7352, (char)14912, (char)54850, (char)34899, (char)63463, (char)20220, (char)61483}));
            assert(pack.max_distance_GET() == (char)48011);
            assert(pack.min_distance_GET() == (char)15987);
            assert(pack.increment_GET() == (char)41);
            assert(pack.time_usec_GET() == 9174242792045672611L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)20351, (char)1211, (char)5379, (char)6097, (char)48959, (char)13234, (char)59792, (char)44175, (char)23484, (char)38204, (char)23571, (char)61563, (char)6507, (char)38005, (char)61020, (char)51931, (char)8220, (char)51621, (char)25062, (char)39729, (char)65141, (char)54806, (char)53663, (char)46488, (char)20663, (char)26510, (char)61130, (char)2678, (char)7617, (char)62445, (char)1022, (char)39806, (char)7291, (char)28713, (char)54510, (char)9758, (char)28995, (char)25999, (char)36127, (char)42721, (char)5240, (char)15238, (char)43456, (char)45632, (char)28091, (char)9896, (char)19600, (char)2516, (char)15546, (char)42477, (char)61360, (char)46535, (char)51356, (char)55019, (char)45245, (char)18985, (char)42505, (char)18956, (char)23919, (char)59902, (char)26987, (char)19506, (char)13309, (char)15337, (char)62356, (char)7352, (char)14912, (char)54850, (char)34899, (char)63463, (char)20220, (char)61483}, 0) ;
        p330.time_usec_SET(9174242792045672611L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED) ;
        p330.max_distance_SET((char)48011) ;
        p330.min_distance_SET((char)15987) ;
        p330.increment_SET((char)41) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            assert(pack.callsign_LEN(ph) == 9);
            assert(pack.callsign_TRY(ph).equals("isyOsvaie"));
            assert(pack.stallSpeed_GET() == (char)52348);
            assert(pack.ICAO_GET() == 3266661603L);
            assert(pack.rfSelect_GET() == UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
            assert(pack.emitterType_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
            assert(pack.gpsOffsetLat_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA);
            assert(pack.gpsOffsetLon_GET() == UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            assert(pack.aircraftSize_GET() == UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M);
        });
        DemoDevice.UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA) ;
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M) ;
        p10001.stallSpeed_SET((char)52348) ;
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED) ;
        p10001.callsign_SET("isyOsvaie", PH) ;
        p10001.ICAO_SET(3266661603L) ;
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL) ;
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR) ;
        LoopBackDemoChannel.instance.send(p10001);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            assert(pack.gpsFix_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0);
            assert(pack.emergencyStatus_GET() == UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
            assert(pack.accuracyHor_GET() == 3057915025L);
            assert(pack.accuracyVel_GET() == (char)15010);
            assert(pack.baroAltMSL_GET() == 481101978);
            assert(pack.gpsLat_GET() == 2051537711);
            assert(pack.velNS_GET() == (short) -21384);
            assert(pack.squawk_GET() == (char)41072);
            assert(pack.velVert_GET() == (short) -21413);
            assert(pack.numSats_GET() == (char)51);
            assert(pack.gpsLon_GET() == 2048685835);
            assert(pack.VelEW_GET() == (short)18689);
            assert(pack.gpsAlt_GET() == -2053643183);
            assert(pack.accuracyVert_GET() == (char)10008);
            assert(pack.utcTime_GET() == 2882126348L);
            assert(pack.state_GET() == UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT);
        });
        DemoDevice.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.state_SET(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT) ;
        p10002.squawk_SET((char)41072) ;
        p10002.accuracyHor_SET(3057915025L) ;
        p10002.accuracyVert_SET((char)10008) ;
        p10002.numSats_SET((char)51) ;
        p10002.velVert_SET((short) -21413) ;
        p10002.velNS_SET((short) -21384) ;
        p10002.accuracyVel_SET((char)15010) ;
        p10002.VelEW_SET((short)18689) ;
        p10002.utcTime_SET(2882126348L) ;
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY) ;
        p10002.gpsLat_SET(2051537711) ;
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0) ;
        p10002.baroAltMSL_SET(481101978) ;
        p10002.gpsLon_SET(2048685835) ;
        p10002.gpsAlt_SET(-2053643183) ;
        LoopBackDemoChannel.instance.send(p10002);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            assert(pack.rfHealth_GET() == UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK);
        });
        DemoDevice.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK) ;
        LoopBackDemoChannel.instance.send(p10003);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
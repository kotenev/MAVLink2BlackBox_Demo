
using System;
using System.Diagnostics;
using System.Linq;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Test : DemoDevice
    {





        class TestChannelAdvanced : Channel.Advanced
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(string reason)
            {
                base.failure(reason);
                Debug.Assert(false);
            }


            static Pack testing_pack;
            readonly  byte[]                        buf = new byte[4024];

            internal void send(Pack pack)
            {
                testing_pack = pack;
            }

            readonly Inside ph = new Inside();

            protected internal override Pack process(Pack pack, int id)
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
                        break;
                }
                switch(id)
                {
                }
                return null;
            }
            static readonly byte[] buff = new byte[1024];
            internal static void transmission(Channel src, Channel dst)
            {
                if(src is Channel.Advanced && !(dst is Channel.Advanced))
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) ADV_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = SMP_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else if(!(src is Channel.Advanced) && dst is Channel.Advanced)
                {
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) SMP_TEST_CH.Write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = ADV_TEST_CH.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.Read(buff, 0, buff.Length));) dst.Write(buff, 0, bytes);
                dst.process(null, Channel.PROCESS_RECEIVED);
            }
        }
        static readonly TestChannelAdvanced ADV_TEST_CH = new TestChannelAdvanced();

        public class TestChannelSimple : Channel
        {
            public override bool CanRead { get { return true; } }
            public override bool CanWrite { get { return true; } }

            public override void failure(String reason) { ADV_TEST_CH.failure(reason);}
            public void send(Pack pack) {ADV_TEST_CH.send(pack); }
            protected internal override Pack process(Pack pack, int id) { return ADV_TEST_CH.process(pack, id); }
        }
        static readonly TestChannelSimple SMP_TEST_CH = new TestChannelSimple();  //test channel with SimpleProtocol



        public static void Main(string[] args)
        {
            Inside PH = new Inside();
            LoopBackDemoChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_BOOT);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR);
                Debug.Assert(pack.mavlink_version == (byte)(byte)110);
                Debug.Assert(pack.custom_mode == (uint)1421882571U);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_BOOT;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR;
            p0.mavlink_version = (byte)(byte)110;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT;
            p0.custom_mode = (uint)1421882571U;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)2957);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)57620);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)32628);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)53796);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)58272);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 98);
                Debug.Assert(pack.current_battery == (short)(short)26321);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)29656);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)776);
                Debug.Assert(pack.load == (ushort)(ushort)63333);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.current_battery = (short)(short)26321;
            p1.voltage_battery = (ushort)(ushort)53796;
            p1.load = (ushort)(ushort)63333;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
            p1.errors_count3 = (ushort)(ushort)57620;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.battery_remaining = (sbyte)(sbyte) - 98;
            p1.errors_count2 = (ushort)(ushort)776;
            p1.errors_comm = (ushort)(ushort)58272;
            p1.errors_count4 = (ushort)(ushort)2957;
            p1.errors_count1 = (ushort)(ushort)32628;
            p1.drop_rate_comm = (ushort)(ushort)29656;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2686124857U);
                Debug.Assert(pack.time_unix_usec == (ulong)4895473435766578178L);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)2686124857U;
            p2.time_unix_usec = (ulong)4895473435766578178L;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -1.712666E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)13199);
                Debug.Assert(pack.vy == (float) -2.2228407E38F);
                Debug.Assert(pack.z == (float) -2.5606417E36F);
                Debug.Assert(pack.afz == (float)1.6245586E38F);
                Debug.Assert(pack.y == (float) -2.372076E38F);
                Debug.Assert(pack.yaw == (float)3.367261E38F);
                Debug.Assert(pack.vx == (float) -1.2049342E38F);
                Debug.Assert(pack.time_boot_ms == (uint)684272806U);
                Debug.Assert(pack.afy == (float) -1.5862809E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.vz == (float) -2.4844404E38F);
                Debug.Assert(pack.afx == (float) -2.960126E37F);
                Debug.Assert(pack.x == (float)1.4122604E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afy = (float) -1.5862809E38F;
            p3.x = (float)1.4122604E38F;
            p3.vx = (float) -1.2049342E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p3.y = (float) -2.372076E38F;
            p3.type_mask = (ushort)(ushort)13199;
            p3.vz = (float) -2.4844404E38F;
            p3.vy = (float) -2.2228407E38F;
            p3.time_boot_ms = (uint)684272806U;
            p3.yaw = (float)3.367261E38F;
            p3.afx = (float) -2.960126E37F;
            p3.afz = (float)1.6245586E38F;
            p3.yaw_rate = (float) -1.712666E38F;
            p3.z = (float) -2.5606417E36F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)145);
                Debug.Assert(pack.seq == (uint)3393649047U);
                Debug.Assert(pack.time_usec == (ulong)4012745957387137674L);
                Debug.Assert(pack.target_component == (byte)(byte)198);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)4012745957387137674L;
            p4.target_system = (byte)(byte)145;
            p4.seq = (uint)3393649047U;
            p4.target_component = (byte)(byte)198;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.passkey_LEN(ph) == 14);
                Debug.Assert(pack.passkey_TRY(ph).Equals("qqacmInpcslyhX"));
                Debug.Assert(pack.control_request == (byte)(byte)104);
                Debug.Assert(pack.version == (byte)(byte)56);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)104;
            p5.target_system = (byte)(byte)185;
            p5.version = (byte)(byte)56;
            p5.passkey_SET("qqacmInpcslyhX", PH) ;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)223);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)90);
                Debug.Assert(pack.ack == (byte)(byte)219);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)90;
            p6.control_request = (byte)(byte)223;
            p6.ack = (byte)(byte)219;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 6);
                Debug.Assert(pack.key_TRY(ph).Equals("OaKqfg"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("OaKqfg", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.custom_mode == (uint)3302686328U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3302686328U;
            p11.target_system = (byte)(byte)240;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)114);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("frutvk"));
                Debug.Assert(pack.param_index == (short)(short) -13256);
                Debug.Assert(pack.target_system == (byte)(byte)126);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -13256;
            p20.target_system = (byte)(byte)126;
            p20.param_id_SET("frutvk", PH) ;
            p20.target_component = (byte)(byte)114;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.target_component == (byte)(byte)153);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)151;
            p21.target_component = (byte)(byte)153;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nwufeontxag"));
                Debug.Assert(pack.param_index == (ushort)(ushort)56056);
                Debug.Assert(pack.param_count == (ushort)(ushort)40125);
                Debug.Assert(pack.param_value == (float) -1.4146354E38F);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("nwufeontxag", PH) ;
            p22.param_value = (float) -1.4146354E38F;
            p22.param_count = (ushort)(ushort)40125;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_index = (ushort)(ushort)56056;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kcmbmbqdnbxkcfb"));
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.target_system == (byte)(byte)76);
                Debug.Assert(pack.param_value == (float)1.3827598E38F);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("kcmbmbqdnbxkcfb", PH) ;
            p23.target_system = (byte)(byte)76;
            p23.target_component = (byte)(byte)59;
            p23.param_value = (float)1.3827598E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)62477);
                Debug.Assert(pack.alt == (int) -203615784);
                Debug.Assert(pack.lat == (int) -1847355303);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2874582305U);
                Debug.Assert(pack.epv == (ushort)(ushort)43519);
                Debug.Assert(pack.eph == (ushort)(ushort)53429);
                Debug.Assert(pack.cog == (ushort)(ushort)2532);
                Debug.Assert(pack.lon == (int)39492012);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)146888448U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)868031694U);
                Debug.Assert(pack.time_usec == (ulong)9190178624079466701L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)249);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1755405024);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3563187595U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.v_acc_SET((uint)146888448U, PH) ;
            p24.lon = (int)39492012;
            p24.vel_acc_SET((uint)3563187595U, PH) ;
            p24.cog = (ushort)(ushort)2532;
            p24.vel = (ushort)(ushort)62477;
            p24.epv = (ushort)(ushort)43519;
            p24.alt = (int) -203615784;
            p24.satellites_visible = (byte)(byte)249;
            p24.eph = (ushort)(ushort)53429;
            p24.hdg_acc_SET((uint)2874582305U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p24.alt_ellipsoid_SET((int) -1755405024, PH) ;
            p24.lat = (int) -1847355303;
            p24.h_acc_SET((uint)868031694U, PH) ;
            p24.time_usec = (ulong)9190178624079466701L;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)132, (byte)67, (byte)29, (byte)207, (byte)91, (byte)0, (byte)120, (byte)215, (byte)169, (byte)9, (byte)176, (byte)227, (byte)247, (byte)50, (byte)120, (byte)79, (byte)0, (byte)85, (byte)57, (byte)80}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)49, (byte)28, (byte)122, (byte)142, (byte)3, (byte)204, (byte)2, (byte)102, (byte)92, (byte)69, (byte)186, (byte)96, (byte)84, (byte)59, (byte)178, (byte)6, (byte)50, (byte)58, (byte)161, (byte)214}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)217, (byte)181, (byte)51, (byte)68, (byte)50, (byte)68, (byte)167, (byte)143, (byte)103, (byte)177, (byte)213, (byte)188, (byte)218, (byte)100, (byte)29, (byte)3, (byte)4, (byte)31, (byte)198, (byte)94}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)136, (byte)249, (byte)146, (byte)35, (byte)18, (byte)63, (byte)70, (byte)3, (byte)56, (byte)237, (byte)63, (byte)224, (byte)146, (byte)42, (byte)54, (byte)195, (byte)27, (byte)97, (byte)81, (byte)56}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)161);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)149, (byte)98, (byte)235, (byte)189, (byte)71, (byte)165, (byte)245, (byte)23, (byte)75, (byte)131, (byte)251, (byte)166, (byte)67, (byte)13, (byte)166, (byte)202, (byte)220, (byte)29, (byte)166, (byte)210}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)49, (byte)28, (byte)122, (byte)142, (byte)3, (byte)204, (byte)2, (byte)102, (byte)92, (byte)69, (byte)186, (byte)96, (byte)84, (byte)59, (byte)178, (byte)6, (byte)50, (byte)58, (byte)161, (byte)214}, 0) ;
            p25.satellites_visible = (byte)(byte)161;
            p25.satellite_prn_SET(new byte[] {(byte)136, (byte)249, (byte)146, (byte)35, (byte)18, (byte)63, (byte)70, (byte)3, (byte)56, (byte)237, (byte)63, (byte)224, (byte)146, (byte)42, (byte)54, (byte)195, (byte)27, (byte)97, (byte)81, (byte)56}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)149, (byte)98, (byte)235, (byte)189, (byte)71, (byte)165, (byte)245, (byte)23, (byte)75, (byte)131, (byte)251, (byte)166, (byte)67, (byte)13, (byte)166, (byte)202, (byte)220, (byte)29, (byte)166, (byte)210}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)217, (byte)181, (byte)51, (byte)68, (byte)50, (byte)68, (byte)167, (byte)143, (byte)103, (byte)177, (byte)213, (byte)188, (byte)218, (byte)100, (byte)29, (byte)3, (byte)4, (byte)31, (byte)198, (byte)94}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)132, (byte)67, (byte)29, (byte)207, (byte)91, (byte)0, (byte)120, (byte)215, (byte)169, (byte)9, (byte)176, (byte)227, (byte)247, (byte)50, (byte)120, (byte)79, (byte)0, (byte)85, (byte)57, (byte)80}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -26903);
                Debug.Assert(pack.zacc == (short)(short)20468);
                Debug.Assert(pack.ymag == (short)(short)12927);
                Debug.Assert(pack.time_boot_ms == (uint)2921552926U);
                Debug.Assert(pack.xmag == (short)(short)14068);
                Debug.Assert(pack.zmag == (short)(short) -24893);
                Debug.Assert(pack.yacc == (short)(short) -5590);
                Debug.Assert(pack.ygyro == (short)(short)20665);
                Debug.Assert(pack.xacc == (short)(short)19246);
                Debug.Assert(pack.xgyro == (short)(short)11177);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.zacc = (short)(short)20468;
            p26.ymag = (short)(short)12927;
            p26.time_boot_ms = (uint)2921552926U;
            p26.zmag = (short)(short) -24893;
            p26.xmag = (short)(short)14068;
            p26.yacc = (short)(short) -5590;
            p26.zgyro = (short)(short) -26903;
            p26.xgyro = (short)(short)11177;
            p26.xacc = (short)(short)19246;
            p26.ygyro = (short)(short)20665;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -1859);
                Debug.Assert(pack.yacc == (short)(short) -2750);
                Debug.Assert(pack.ygyro == (short)(short) -20987);
                Debug.Assert(pack.ymag == (short)(short) -664);
                Debug.Assert(pack.zmag == (short)(short) -31985);
                Debug.Assert(pack.zgyro == (short)(short)14767);
                Debug.Assert(pack.xacc == (short)(short) -11939);
                Debug.Assert(pack.xgyro == (short)(short)9749);
                Debug.Assert(pack.time_usec == (ulong)2309902743368890868L);
                Debug.Assert(pack.zacc == (short)(short)16658);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short)16658;
            p27.zmag = (short)(short) -31985;
            p27.xacc = (short)(short) -11939;
            p27.ymag = (short)(short) -664;
            p27.zgyro = (short)(short)14767;
            p27.xmag = (short)(short) -1859;
            p27.xgyro = (short)(short)9749;
            p27.ygyro = (short)(short) -20987;
            p27.time_usec = (ulong)2309902743368890868L;
            p27.yacc = (short)(short) -2750;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -27479);
                Debug.Assert(pack.press_diff1 == (short)(short) -30953);
                Debug.Assert(pack.press_diff2 == (short)(short) -18422);
                Debug.Assert(pack.time_usec == (ulong)6650620884973042245L);
                Debug.Assert(pack.press_abs == (short)(short)29947);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short) -18422;
            p28.press_abs = (short)(short)29947;
            p28.temperature = (short)(short) -27479;
            p28.time_usec = (ulong)6650620884973042245L;
            p28.press_diff1 = (short)(short) -30953;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)234629850U);
                Debug.Assert(pack.temperature == (short)(short)14333);
                Debug.Assert(pack.press_diff == (float) -1.2897218E38F);
                Debug.Assert(pack.press_abs == (float)3.716911E37F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)14333;
            p29.time_boot_ms = (uint)234629850U;
            p29.press_diff = (float) -1.2897218E38F;
            p29.press_abs = (float)3.716911E37F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)5.3912196E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3107272611U);
                Debug.Assert(pack.yaw == (float)3.378858E37F);
                Debug.Assert(pack.pitchspeed == (float) -1.3545144E38F);
                Debug.Assert(pack.roll == (float)2.4536858E38F);
                Debug.Assert(pack.pitch == (float)1.0493466E38F);
                Debug.Assert(pack.yawspeed == (float)2.8285613E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float)3.378858E37F;
            p30.pitchspeed = (float) -1.3545144E38F;
            p30.pitch = (float)1.0493466E38F;
            p30.time_boot_ms = (uint)3107272611U;
            p30.roll = (float)2.4536858E38F;
            p30.rollspeed = (float)5.3912196E37F;
            p30.yawspeed = (float)2.8285613E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -1.735754E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.0141047E38F);
                Debug.Assert(pack.q4 == (float)2.0171623E38F);
                Debug.Assert(pack.q1 == (float)1.3950658E38F);
                Debug.Assert(pack.q2 == (float)2.3593355E38F);
                Debug.Assert(pack.q3 == (float)4.612769E37F);
                Debug.Assert(pack.yawspeed == (float) -2.9363236E38F);
                Debug.Assert(pack.time_boot_ms == (uint)898057796U);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -2.0141047E38F;
            p31.yawspeed = (float) -2.9363236E38F;
            p31.time_boot_ms = (uint)898057796U;
            p31.q1 = (float)1.3950658E38F;
            p31.q4 = (float)2.0171623E38F;
            p31.q2 = (float)2.3593355E38F;
            p31.q3 = (float)4.612769E37F;
            p31.rollspeed = (float) -1.735754E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.8914834E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1723390898U);
                Debug.Assert(pack.vy == (float) -6.7782297E37F);
                Debug.Assert(pack.y == (float) -8.1773336E37F);
                Debug.Assert(pack.z == (float) -2.659012E38F);
                Debug.Assert(pack.vz == (float) -2.2907651E38F);
                Debug.Assert(pack.vx == (float)1.4658757E38F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float)1.4658757E38F;
            p32.time_boot_ms = (uint)1723390898U;
            p32.y = (float) -8.1773336E37F;
            p32.z = (float) -2.659012E38F;
            p32.x = (float) -1.8914834E38F;
            p32.vy = (float) -6.7782297E37F;
            p32.vz = (float) -2.2907651E38F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1967153858);
                Debug.Assert(pack.relative_alt == (int)995334264);
                Debug.Assert(pack.hdg == (ushort)(ushort)57094);
                Debug.Assert(pack.vy == (short)(short) -28602);
                Debug.Assert(pack.lat == (int)180503280);
                Debug.Assert(pack.time_boot_ms == (uint)2155528648U);
                Debug.Assert(pack.lon == (int)1851278413);
                Debug.Assert(pack.vx == (short)(short) -11931);
                Debug.Assert(pack.vz == (short)(short)31356);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vy = (short)(short) -28602;
            p33.lat = (int)180503280;
            p33.hdg = (ushort)(ushort)57094;
            p33.time_boot_ms = (uint)2155528648U;
            p33.vz = (short)(short)31356;
            p33.lon = (int)1851278413;
            p33.vx = (short)(short) -11931;
            p33.alt = (int) -1967153858;
            p33.relative_alt = (int)995334264;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_scaled == (short)(short)23239);
                Debug.Assert(pack.time_boot_ms == (uint)3281977211U);
                Debug.Assert(pack.rssi == (byte)(byte)104);
                Debug.Assert(pack.port == (byte)(byte)198);
                Debug.Assert(pack.chan4_scaled == (short)(short) -7223);
                Debug.Assert(pack.chan7_scaled == (short)(short) -18288);
                Debug.Assert(pack.chan1_scaled == (short)(short)5143);
                Debug.Assert(pack.chan2_scaled == (short)(short) -20729);
                Debug.Assert(pack.chan3_scaled == (short)(short)5628);
                Debug.Assert(pack.chan5_scaled == (short)(short) -14490);
                Debug.Assert(pack.chan6_scaled == (short)(short)9444);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan6_scaled = (short)(short)9444;
            p34.chan1_scaled = (short)(short)5143;
            p34.chan7_scaled = (short)(short) -18288;
            p34.port = (byte)(byte)198;
            p34.time_boot_ms = (uint)3281977211U;
            p34.chan2_scaled = (short)(short) -20729;
            p34.chan3_scaled = (short)(short)5628;
            p34.rssi = (byte)(byte)104;
            p34.chan8_scaled = (short)(short)23239;
            p34.chan4_scaled = (short)(short) -7223;
            p34.chan5_scaled = (short)(short) -14490;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)61331);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)44314);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)25966);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)40884);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)15665);
                Debug.Assert(pack.time_boot_ms == (uint)550691164U);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)12544);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)44983);
                Debug.Assert(pack.rssi == (byte)(byte)191);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)12296);
                Debug.Assert(pack.port == (byte)(byte)72);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)44314;
            p35.chan5_raw = (ushort)(ushort)15665;
            p35.chan8_raw = (ushort)(ushort)25966;
            p35.chan6_raw = (ushort)(ushort)61331;
            p35.chan4_raw = (ushort)(ushort)12296;
            p35.chan3_raw = (ushort)(ushort)12544;
            p35.chan1_raw = (ushort)(ushort)40884;
            p35.port = (byte)(byte)72;
            p35.time_boot_ms = (uint)550691164U;
            p35.rssi = (byte)(byte)191;
            p35.chan7_raw = (ushort)(ushort)44983;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)63743);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)23300);
                Debug.Assert(pack.time_usec == (uint)729060077U);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)22648);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)56412);
                Debug.Assert(pack.port == (byte)(byte)33);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)40194);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)12386);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)44707);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)1068);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)10029);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)3266);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)4687);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)46908);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)45854);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)53153);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)32306);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)26140);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)40194;
            p36.servo16_raw_SET((ushort)(ushort)22648, PH) ;
            p36.servo3_raw = (ushort)(ushort)32306;
            p36.servo4_raw = (ushort)(ushort)3266;
            p36.servo1_raw = (ushort)(ushort)44707;
            p36.servo8_raw = (ushort)(ushort)4687;
            p36.servo10_raw_SET((ushort)(ushort)45854, PH) ;
            p36.servo5_raw = (ushort)(ushort)63743;
            p36.servo13_raw_SET((ushort)(ushort)26140, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)12386, PH) ;
            p36.time_usec = (uint)729060077U;
            p36.port = (byte)(byte)33;
            p36.servo6_raw = (ushort)(ushort)56412;
            p36.servo12_raw_SET((ushort)(ushort)1068, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)46908, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)10029, PH) ;
            p36.servo7_raw = (ushort)(ushort)53153;
            p36.servo14_raw_SET((ushort)(ushort)23300, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.target_component == (byte)(byte)129);
                Debug.Assert(pack.start_index == (short)(short)9456);
                Debug.Assert(pack.end_index == (short)(short) -18878);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short)9456;
            p37.end_index = (short)(short) -18878;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.target_system = (byte)(byte)4;
            p37.target_component = (byte)(byte)129;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)9097);
                Debug.Assert(pack.target_component == (byte)(byte)193);
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.start_index == (short)(short)21788);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.target_component = (byte)(byte)193;
            p38.end_index = (short)(short)9097;
            p38.start_index = (short)(short)21788;
            p38.target_system = (byte)(byte)111;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.current == (byte)(byte)141);
                Debug.Assert(pack.x == (float)2.2314259E38F);
                Debug.Assert(pack.target_component == (byte)(byte)78);
                Debug.Assert(pack.z == (float) -9.501032E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)71);
                Debug.Assert(pack.seq == (ushort)(ushort)45495);
                Debug.Assert(pack.param2 == (float)1.1819058E38F);
                Debug.Assert(pack.param4 == (float) -1.4196326E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
                Debug.Assert(pack.y == (float)4.621875E37F);
                Debug.Assert(pack.param1 == (float) -1.1880204E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.param3 == (float) -2.2616256E38F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.param4 = (float) -1.4196326E38F;
            p39.z = (float) -9.501032E37F;
            p39.y = (float)4.621875E37F;
            p39.seq = (ushort)(ushort)45495;
            p39.target_component = (byte)(byte)78;
            p39.x = (float)2.2314259E38F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_VTOL_LAND;
            p39.param3 = (float) -2.2616256E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.target_system = (byte)(byte)133;
            p39.param2 = (float)1.1819058E38F;
            p39.param1 = (float) -1.1880204E38F;
            p39.current = (byte)(byte)141;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p39.autocontinue = (byte)(byte)71;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)60984);
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)0);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)15;
            p40.seq = (ushort)(ushort)60984;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_component = (byte)(byte)0;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.target_component == (byte)(byte)94);
                Debug.Assert(pack.seq == (ushort)(ushort)48235);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)31;
            p41.target_component = (byte)(byte)94;
            p41.seq = (ushort)(ushort)48235;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)29717);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)29717;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)221);
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)126;
            p43.target_system = (byte)(byte)221;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)206);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.count == (ushort)(ushort)15443);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)15443;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.target_system = (byte)(byte)150;
            p44.target_component = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)226);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)226;
            p45.target_system = (byte)(byte)13;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)27015);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)27015;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)255);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X);
                Debug.Assert(pack.target_component == (byte)(byte)91);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)255;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X;
            p47.target_component = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)120);
                Debug.Assert(pack.longitude == (int) -52080671);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5438467669336143779L);
                Debug.Assert(pack.altitude == (int) -1691917343);
                Debug.Assert(pack.latitude == (int)1451468776);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)5438467669336143779L, PH) ;
            p48.altitude = (int) -1691917343;
            p48.target_system = (byte)(byte)120;
            p48.longitude = (int) -52080671;
            p48.latitude = (int)1451468776;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -850025515);
                Debug.Assert(pack.longitude == (int) -1496316555);
                Debug.Assert(pack.altitude == (int) -255703829);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8464457586304988507L);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int) -1496316555;
            p49.latitude = (int) -850025515;
            p49.time_usec_SET((ulong)8464457586304988507L, PH) ;
            p49.altitude = (int) -255703829;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_min == (float) -1.1993812E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)180);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.param_index == (short)(short)10001);
                Debug.Assert(pack.scale == (float)1.9350567E38F);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.param_value_max == (float)1.4060066E38F);
                Debug.Assert(pack.param_value0 == (float)6.22581E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pop"));
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value0 = (float)6.22581E37F;
            p50.target_system = (byte)(byte)58;
            p50.param_id_SET("pop", PH) ;
            p50.param_value_min = (float) -1.1993812E38F;
            p50.parameter_rc_channel_index = (byte)(byte)180;
            p50.target_component = (byte)(byte)95;
            p50.param_value_max = (float)1.4060066E38F;
            p50.scale = (float)1.9350567E38F;
            p50.param_index = (short)(short)10001;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)18);
                Debug.Assert(pack.seq == (ushort)(ushort)35877);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)123);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)18;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.target_component = (byte)(byte)123;
            p51.seq = (ushort)(ushort)35877;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.p1y == (float)3.1144216E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.p1x == (float) -2.1913263E38F);
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.p2x == (float)3.3723538E38F);
                Debug.Assert(pack.p2z == (float) -1.9678276E38F);
                Debug.Assert(pack.p1z == (float) -1.0614458E38F);
                Debug.Assert(pack.p2y == (float) -1.1368435E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1y = (float)3.1144216E38F;
            p54.target_system = (byte)(byte)137;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p54.p2z = (float) -1.9678276E38F;
            p54.p1z = (float) -1.0614458E38F;
            p54.target_component = (byte)(byte)242;
            p54.p2x = (float)3.3723538E38F;
            p54.p2y = (float) -1.1368435E38F;
            p54.p1x = (float) -2.1913263E38F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float) -1.118611E38F);
                Debug.Assert(pack.p2x == (float) -1.7332017E38F);
                Debug.Assert(pack.p1z == (float)1.7782418E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.p2y == (float) -2.696926E38F);
                Debug.Assert(pack.p1x == (float) -3.1944097E38F);
                Debug.Assert(pack.p1y == (float) -1.863643E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float) -3.1944097E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p55.p1y = (float) -1.863643E38F;
            p55.p2y = (float) -2.696926E38F;
            p55.p1z = (float)1.7782418E38F;
            p55.p2z = (float) -1.118611E38F;
            p55.p2x = (float) -1.7332017E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -4.945977E37F);
                Debug.Assert(pack.yawspeed == (float)1.785862E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.7247415E38F, 1.0100757E38F, 2.5318E38F, -1.5655233E38F}));
                Debug.Assert(pack.pitchspeed == (float) -2.496435E37F);
                Debug.Assert(pack.time_usec == (ulong)3036127185616791733L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.8777498E38F, -1.1640145E38F, -2.8598723E38F, 6.8908054E36F, 1.2061644E38F, 2.9084E38F, -1.0840319E38F, 1.9162871E38F, -1.7754296E38F}));
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-2.7247415E38F, 1.0100757E38F, 2.5318E38F, -1.5655233E38F}, 0) ;
            p61.time_usec = (ulong)3036127185616791733L;
            p61.covariance_SET(new float[] {2.8777498E38F, -1.1640145E38F, -2.8598723E38F, 6.8908054E36F, 1.2061644E38F, 2.9084E38F, -1.0840319E38F, 1.9162871E38F, -1.7754296E38F}, 0) ;
            p61.pitchspeed = (float) -2.496435E37F;
            p61.rollspeed = (float) -4.945977E37F;
            p61.yawspeed = (float)1.785862E38F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aspd_error == (float) -2.9859013E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)32497);
                Debug.Assert(pack.xtrack_error == (float)2.191501E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)51734);
                Debug.Assert(pack.alt_error == (float)1.5859418E38F);
                Debug.Assert(pack.nav_pitch == (float) -3.030262E38F);
                Debug.Assert(pack.nav_roll == (float) -2.2319772E38F);
                Debug.Assert(pack.target_bearing == (short)(short)9929);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.xtrack_error = (float)2.191501E37F;
            p62.target_bearing = (short)(short)9929;
            p62.alt_error = (float)1.5859418E38F;
            p62.aspd_error = (float) -2.9859013E38F;
            p62.wp_dist = (ushort)(ushort)51734;
            p62.nav_pitch = (float) -3.030262E38F;
            p62.nav_bearing = (short)(short)32497;
            p62.nav_roll = (float) -2.2319772E38F;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.9870032E38F, 9.55272E37F, -2.4611607E38F, 4.5759023E37F, 7.3473034E37F, -6.3230995E37F, 6.494928E37F, 2.198703E37F, -2.7212522E38F, -1.2139783E38F, -3.0287753E38F, -1.0825861E38F, -2.9654255E36F, 2.9419158E38F, 2.003768E38F, 3.4005291E38F, 3.1442136E37F, -8.6279754E36F, 8.8638256E36F, 1.5926889E38F, 3.1592486E38F, 2.9452477E37F, -1.1728251E38F, -3.2301017E38F, 3.373039E38F, 2.466962E38F, 2.5495784E38F, -9.494097E37F, -1.5360268E37F, 1.8216346E38F, -1.1250079E38F, -2.5315064E38F, 1.959745E38F, -2.9651844E38F, 1.3089761E38F, -2.9120549E38F}));
                Debug.Assert(pack.time_usec == (ulong)3883086913046162845L);
                Debug.Assert(pack.lat == (int) -1702805118);
                Debug.Assert(pack.alt == (int) -2071101726);
                Debug.Assert(pack.vx == (float) -1.6727197E38F);
                Debug.Assert(pack.vz == (float)1.5258193E38F);
                Debug.Assert(pack.lon == (int)1945138851);
                Debug.Assert(pack.vy == (float)8.3099826E37F);
                Debug.Assert(pack.relative_alt == (int)2091571421);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vz = (float)1.5258193E38F;
            p63.relative_alt = (int)2091571421;
            p63.lon = (int)1945138851;
            p63.vy = (float)8.3099826E37F;
            p63.covariance_SET(new float[] {-2.9870032E38F, 9.55272E37F, -2.4611607E38F, 4.5759023E37F, 7.3473034E37F, -6.3230995E37F, 6.494928E37F, 2.198703E37F, -2.7212522E38F, -1.2139783E38F, -3.0287753E38F, -1.0825861E38F, -2.9654255E36F, 2.9419158E38F, 2.003768E38F, 3.4005291E38F, 3.1442136E37F, -8.6279754E36F, 8.8638256E36F, 1.5926889E38F, 3.1592486E38F, 2.9452477E37F, -1.1728251E38F, -3.2301017E38F, 3.373039E38F, 2.466962E38F, 2.5495784E38F, -9.494097E37F, -1.5360268E37F, 1.8216346E38F, -1.1250079E38F, -2.5315064E38F, 1.959745E38F, -2.9651844E38F, 1.3089761E38F, -2.9120549E38F}, 0) ;
            p63.time_usec = (ulong)3883086913046162845L;
            p63.vx = (float) -1.6727197E38F;
            p63.alt = (int) -2071101726;
            p63.lat = (int) -1702805118;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float)1.3833941E38F);
                Debug.Assert(pack.time_usec == (ulong)3875980977094359268L);
                Debug.Assert(pack.vz == (float) -1.7002078E38F);
                Debug.Assert(pack.vx == (float) -1.0431541E37F);
                Debug.Assert(pack.ax == (float)2.2323657E37F);
                Debug.Assert(pack.x == (float)7.899872E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.y == (float)2.6131262E38F);
                Debug.Assert(pack.vy == (float)2.1675612E38F);
                Debug.Assert(pack.ay == (float) -2.7891895E38F);
                Debug.Assert(pack.z == (float)3.3955648E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.8753021E38F, 1.2572278E38F, -7.145609E37F, -8.868728E37F, -1.3485516E38F, -2.7072794E38F, 2.625547E38F, 8.594288E37F, -1.4076239E38F, 3.01787E38F, 9.046E37F, -1.5447691E38F, 2.5329476E38F, 2.2793429E38F, 1.454979E38F, 2.6862094E38F, -3.7803633E37F, -1.3888201E38F, -2.0623339E38F, -1.9366898E38F, -1.1003351E38F, -2.561998E38F, 3.2689677E38F, -2.3901848E38F, -9.445621E37F, 2.8349103E38F, -3.395825E38F, -7.047398E37F, 6.9452644E37F, 2.4966614E38F, 2.2793485E38F, -2.7436494E38F, 2.4187212E38F, 4.10277E37F, -1.9765478E38F, -2.3798678E38F, 1.9270257E38F, -1.5952081E37F, -1.2178802E38F, -3.674442E37F, -3.0564535E38F, 1.3843273E38F, 9.1252443E36F, 2.615997E38F, 3.1289898E38F}));
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.z = (float)3.3955648E38F;
            p64.ay = (float) -2.7891895E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p64.x = (float)7.899872E37F;
            p64.vz = (float) -1.7002078E38F;
            p64.time_usec = (ulong)3875980977094359268L;
            p64.az = (float)1.3833941E38F;
            p64.vx = (float) -1.0431541E37F;
            p64.y = (float)2.6131262E38F;
            p64.covariance_SET(new float[] {2.8753021E38F, 1.2572278E38F, -7.145609E37F, -8.868728E37F, -1.3485516E38F, -2.7072794E38F, 2.625547E38F, 8.594288E37F, -1.4076239E38F, 3.01787E38F, 9.046E37F, -1.5447691E38F, 2.5329476E38F, 2.2793429E38F, 1.454979E38F, 2.6862094E38F, -3.7803633E37F, -1.3888201E38F, -2.0623339E38F, -1.9366898E38F, -1.1003351E38F, -2.561998E38F, 3.2689677E38F, -2.3901848E38F, -9.445621E37F, 2.8349103E38F, -3.395825E38F, -7.047398E37F, 6.9452644E37F, 2.4966614E38F, 2.2793485E38F, -2.7436494E38F, 2.4187212E38F, 4.10277E37F, -1.9765478E38F, -2.3798678E38F, 1.9270257E38F, -1.5952081E37F, -1.2178802E38F, -3.674442E37F, -3.0564535E38F, 1.3843273E38F, 9.1252443E36F, 2.615997E38F, 3.1289898E38F}, 0) ;
            p64.vy = (float)2.1675612E38F;
            p64.ax = (float)2.2323657E37F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)9281);
                Debug.Assert(pack.chancount == (byte)(byte)228);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)4520);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)58079);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)44639);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)42068);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)56310);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)38409);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)41665);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)484);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)1960);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)36050);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)11846);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)51652);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)738);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)48996);
                Debug.Assert(pack.time_boot_ms == (uint)86048515U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)40465);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)36545);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)10766);
                Debug.Assert(pack.rssi == (byte)(byte)222);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan5_raw = (ushort)(ushort)44639;
            p65.chan14_raw = (ushort)(ushort)51652;
            p65.chan11_raw = (ushort)(ushort)48996;
            p65.chan9_raw = (ushort)(ushort)9281;
            p65.chancount = (byte)(byte)228;
            p65.chan1_raw = (ushort)(ushort)4520;
            p65.chan13_raw = (ushort)(ushort)484;
            p65.chan17_raw = (ushort)(ushort)36545;
            p65.chan6_raw = (ushort)(ushort)40465;
            p65.chan8_raw = (ushort)(ushort)38409;
            p65.chan10_raw = (ushort)(ushort)10766;
            p65.chan4_raw = (ushort)(ushort)42068;
            p65.chan16_raw = (ushort)(ushort)1960;
            p65.chan18_raw = (ushort)(ushort)11846;
            p65.time_boot_ms = (uint)86048515U;
            p65.chan7_raw = (ushort)(ushort)738;
            p65.rssi = (byte)(byte)222;
            p65.chan15_raw = (ushort)(ushort)41665;
            p65.chan2_raw = (ushort)(ushort)58079;
            p65.chan3_raw = (ushort)(ushort)36050;
            p65.chan12_raw = (ushort)(ushort)56310;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)253);
                Debug.Assert(pack.start_stop == (byte)(byte)211);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)4631);
                Debug.Assert(pack.target_system == (byte)(byte)63);
                Debug.Assert(pack.target_component == (byte)(byte)233);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_stream_id = (byte)(byte)253;
            p66.req_message_rate = (ushort)(ushort)4631;
            p66.target_system = (byte)(byte)63;
            p66.target_component = (byte)(byte)233;
            p66.start_stop = (byte)(byte)211;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)11652);
                Debug.Assert(pack.on_off == (byte)(byte)249);
                Debug.Assert(pack.stream_id == (byte)(byte)245);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)249;
            p67.stream_id = (byte)(byte)245;
            p67.message_rate = (ushort)(ushort)11652;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short)11414);
                Debug.Assert(pack.x == (short)(short) -29009);
                Debug.Assert(pack.target == (byte)(byte)2);
                Debug.Assert(pack.r == (short)(short)14250);
                Debug.Assert(pack.buttons == (ushort)(ushort)7140);
                Debug.Assert(pack.y == (short)(short) -20183);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)2;
            p69.y = (short)(short) -20183;
            p69.z = (short)(short)11414;
            p69.r = (short)(short)14250;
            p69.x = (short)(short) -29009;
            p69.buttons = (ushort)(ushort)7140;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)28758);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)21033);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)9464);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43086);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)49472);
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)59599);
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22769);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)28283);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)250;
            p70.chan3_raw = (ushort)(ushort)9464;
            p70.chan2_raw = (ushort)(ushort)43086;
            p70.chan1_raw = (ushort)(ushort)21033;
            p70.chan7_raw = (ushort)(ushort)22769;
            p70.target_component = (byte)(byte)40;
            p70.chan8_raw = (ushort)(ushort)28758;
            p70.chan6_raw = (ushort)(ushort)59599;
            p70.chan4_raw = (ushort)(ushort)49472;
            p70.chan5_raw = (ushort)(ushort)28283;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)51816);
                Debug.Assert(pack.y == (int) -828342620);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.autocontinue == (byte)(byte)187);
                Debug.Assert(pack.z == (float)8.840363E37F);
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.current == (byte)(byte)247);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED);
                Debug.Assert(pack.param2 == (float) -9.941344E37F);
                Debug.Assert(pack.param4 == (float) -3.3309067E38F);
                Debug.Assert(pack.target_system == (byte)(byte)234);
                Debug.Assert(pack.param1 == (float) -3.2817286E38F);
                Debug.Assert(pack.param3 == (float) -3.246533E37F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.x == (int) -1952376126);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.z = (float)8.840363E37F;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p73.target_component = (byte)(byte)36;
            p73.target_system = (byte)(byte)234;
            p73.param2 = (float) -9.941344E37F;
            p73.param1 = (float) -3.2817286E38F;
            p73.x = (int) -1952376126;
            p73.seq = (ushort)(ushort)51816;
            p73.current = (byte)(byte)247;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
            p73.autocontinue = (byte)(byte)187;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.param3 = (float) -3.246533E37F;
            p73.y = (int) -828342620;
            p73.param4 = (float) -3.3309067E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float)6.004172E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)2938);
                Debug.Assert(pack.climb == (float)3.0187667E38F);
                Debug.Assert(pack.heading == (short)(short)7890);
                Debug.Assert(pack.alt == (float) -9.866689E37F);
                Debug.Assert(pack.groundspeed == (float) -2.5516413E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.throttle = (ushort)(ushort)2938;
            p74.heading = (short)(short)7890;
            p74.groundspeed = (float) -2.5516413E38F;
            p74.climb = (float)3.0187667E38F;
            p74.airspeed = (float)6.004172E37F;
            p74.alt = (float) -9.866689E37F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)2.0137985E38F);
                Debug.Assert(pack.param3 == (float) -2.3077977E38F);
                Debug.Assert(pack.param1 == (float) -3.029302E38F);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.param4 == (float)4.477624E37F);
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.z == (float)2.3330793E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)91);
                Debug.Assert(pack.x == (int)1882619639);
                Debug.Assert(pack.current == (byte)(byte)223);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.y == (int)1776140860);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param4 = (float)4.477624E37F;
            p75.param3 = (float) -2.3077977E38F;
            p75.z = (float)2.3330793E38F;
            p75.y = (int)1776140860;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p75.current = (byte)(byte)223;
            p75.target_system = (byte)(byte)101;
            p75.param1 = (float) -3.029302E38F;
            p75.autocontinue = (byte)(byte)91;
            p75.target_component = (byte)(byte)97;
            p75.param2 = (float)2.0137985E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT;
            p75.x = (int)1882619639;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.param4 == (float)1.7757327E38F);
                Debug.Assert(pack.param1 == (float)1.166366E38F);
                Debug.Assert(pack.param6 == (float)2.3918443E38F);
                Debug.Assert(pack.param2 == (float) -3.3143158E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.confirmation == (byte)(byte)255);
                Debug.Assert(pack.param3 == (float) -2.632616E37F);
                Debug.Assert(pack.param7 == (float) -1.3328914E38F);
                Debug.Assert(pack.param5 == (float) -2.148788E38F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
            p76.param7 = (float) -1.3328914E38F;
            p76.param6 = (float)2.3918443E38F;
            p76.param3 = (float) -2.632616E37F;
            p76.param5 = (float) -2.148788E38F;
            p76.target_component = (byte)(byte)236;
            p76.param1 = (float)1.166366E38F;
            p76.param4 = (float)1.7757327E38F;
            p76.target_system = (byte)(byte)49;
            p76.confirmation = (byte)(byte)255;
            p76.param2 = (float) -3.3143158E37F;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)88);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)245);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)137);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1194630637);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)88, PH) ;
            p77.progress_SET((byte)(byte)245, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.result_param2_SET((int)1194630637, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST;
            p77.target_component_SET((byte)(byte)137, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.manual_override_switch == (byte)(byte)37);
                Debug.Assert(pack.mode_switch == (byte)(byte)132);
                Debug.Assert(pack.thrust == (float) -1.6887524E36F);
                Debug.Assert(pack.time_boot_ms == (uint)3381311382U);
                Debug.Assert(pack.pitch == (float) -2.677413E38F);
                Debug.Assert(pack.yaw == (float)2.339826E38F);
                Debug.Assert(pack.roll == (float) -1.822848E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)2.339826E38F;
            p81.roll = (float) -1.822848E38F;
            p81.mode_switch = (byte)(byte)132;
            p81.thrust = (float) -1.6887524E36F;
            p81.manual_override_switch = (byte)(byte)37;
            p81.pitch = (float) -2.677413E38F;
            p81.time_boot_ms = (uint)3381311382U;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3740249113U);
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.body_yaw_rate == (float)1.1780297E38F);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.body_roll_rate == (float) -2.9601599E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -2.8630817E37F);
                Debug.Assert(pack.thrust == (float)6.754075E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.3980396E38F, 3.253998E38F, 3.0038445E38F, -3.3365998E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)185);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.type_mask = (byte)(byte)185;
            p82.time_boot_ms = (uint)3740249113U;
            p82.target_component = (byte)(byte)227;
            p82.q_SET(new float[] {2.3980396E38F, 3.253998E38F, 3.0038445E38F, -3.3365998E38F}, 0) ;
            p82.body_pitch_rate = (float) -2.8630817E37F;
            p82.body_roll_rate = (float) -2.9601599E38F;
            p82.thrust = (float)6.754075E37F;
            p82.target_system = (byte)(byte)73;
            p82.body_yaw_rate = (float)1.1780297E38F;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)1.4060761E38F);
                Debug.Assert(pack.thrust == (float)1.4940685E38F);
                Debug.Assert(pack.body_pitch_rate == (float)6.599112E37F);
                Debug.Assert(pack.time_boot_ms == (uint)225203461U);
                Debug.Assert(pack.type_mask == (byte)(byte)208);
                Debug.Assert(pack.body_yaw_rate == (float)2.241962E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.187683E38F, -1.4773219E38F, 2.9677414E38F, -1.8246782E38F}));
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {2.187683E38F, -1.4773219E38F, 2.9677414E38F, -1.8246782E38F}, 0) ;
            p83.thrust = (float)1.4940685E38F;
            p83.type_mask = (byte)(byte)208;
            p83.body_yaw_rate = (float)2.241962E38F;
            p83.body_pitch_rate = (float)6.599112E37F;
            p83.time_boot_ms = (uint)225203461U;
            p83.body_roll_rate = (float)1.4060761E38F;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)2.5296824E38F);
                Debug.Assert(pack.z == (float) -4.9183287E37F);
                Debug.Assert(pack.afy == (float)1.0664225E38F);
                Debug.Assert(pack.vz == (float) -1.6242391E37F);
                Debug.Assert(pack.vx == (float) -1.178842E38F);
                Debug.Assert(pack.x == (float)2.607567E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.time_boot_ms == (uint)1661370904U);
                Debug.Assert(pack.vy == (float)3.172979E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)13624);
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.yaw_rate == (float)2.51293E38F);
                Debug.Assert(pack.yaw == (float)2.6604094E38F);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.afz == (float)2.3652504E37F);
                Debug.Assert(pack.y == (float)2.3987413E36F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1661370904U;
            p84.vy = (float)3.172979E37F;
            p84.target_component = (byte)(byte)232;
            p84.yaw_rate = (float)2.51293E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p84.z = (float) -4.9183287E37F;
            p84.afz = (float)2.3652504E37F;
            p84.vz = (float) -1.6242391E37F;
            p84.target_system = (byte)(byte)247;
            p84.afx = (float)2.5296824E38F;
            p84.y = (float)2.3987413E36F;
            p84.yaw = (float)2.6604094E38F;
            p84.afy = (float)1.0664225E38F;
            p84.type_mask = (ushort)(ushort)13624;
            p84.x = (float)2.607567E38F;
            p84.vx = (float) -1.178842E38F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)6704);
                Debug.Assert(pack.lat_int == (int)549242476);
                Debug.Assert(pack.afx == (float)2.5395483E38F);
                Debug.Assert(pack.alt == (float)1.7738636E38F);
                Debug.Assert(pack.vx == (float)5.1738054E37F);
                Debug.Assert(pack.vy == (float) -3.3523199E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1598766785U);
                Debug.Assert(pack.lon_int == (int) -1628817183);
                Debug.Assert(pack.yaw == (float)4.392149E37F);
                Debug.Assert(pack.afy == (float) -2.6180974E38F);
                Debug.Assert(pack.afz == (float)1.1224569E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.yaw_rate == (float)1.4055492E38F);
                Debug.Assert(pack.target_component == (byte)(byte)198);
                Debug.Assert(pack.vz == (float)1.3360784E38F);
                Debug.Assert(pack.target_system == (byte)(byte)146);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.lon_int = (int) -1628817183;
            p86.alt = (float)1.7738636E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p86.yaw_rate = (float)1.4055492E38F;
            p86.lat_int = (int)549242476;
            p86.time_boot_ms = (uint)1598766785U;
            p86.afx = (float)2.5395483E38F;
            p86.yaw = (float)4.392149E37F;
            p86.target_system = (byte)(byte)146;
            p86.afz = (float)1.1224569E38F;
            p86.vy = (float) -3.3523199E38F;
            p86.target_component = (byte)(byte)198;
            p86.afy = (float) -2.6180974E38F;
            p86.type_mask = (ushort)(ushort)6704;
            p86.vx = (float)5.1738054E37F;
            p86.vz = (float)1.3360784E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -2.6973252E37F);
                Debug.Assert(pack.afx == (float)2.4457992E38F);
                Debug.Assert(pack.lon_int == (int) -375552915);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.vz == (float) -6.324875E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1604435223U);
                Debug.Assert(pack.alt == (float) -3.0109334E38F);
                Debug.Assert(pack.yaw_rate == (float)2.9645861E38F);
                Debug.Assert(pack.yaw == (float)2.4165218E37F);
                Debug.Assert(pack.afy == (float)2.507781E37F);
                Debug.Assert(pack.vy == (float)2.9762722E38F);
                Debug.Assert(pack.vx == (float) -1.62446E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)55823);
                Debug.Assert(pack.lat_int == (int) -1133427787);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.alt = (float) -3.0109334E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.afz = (float) -2.6973252E37F;
            p87.yaw_rate = (float)2.9645861E38F;
            p87.afx = (float)2.4457992E38F;
            p87.time_boot_ms = (uint)1604435223U;
            p87.vz = (float) -6.324875E37F;
            p87.lon_int = (int) -375552915;
            p87.type_mask = (ushort)(ushort)55823;
            p87.lat_int = (int) -1133427787;
            p87.afy = (float)2.507781E37F;
            p87.vy = (float)2.9762722E38F;
            p87.vx = (float) -1.62446E38F;
            p87.yaw = (float)2.4165218E37F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.7770835E38F);
                Debug.Assert(pack.z == (float) -3.0165576E38F);
                Debug.Assert(pack.yaw == (float)5.601882E37F);
                Debug.Assert(pack.pitch == (float)4.9857443E37F);
                Debug.Assert(pack.x == (float) -1.7234536E37F);
                Debug.Assert(pack.time_boot_ms == (uint)563119592U);
                Debug.Assert(pack.y == (float)3.1038723E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)563119592U;
            p89.z = (float) -3.0165576E38F;
            p89.roll = (float)2.7770835E38F;
            p89.yaw = (float)5.601882E37F;
            p89.x = (float) -1.7234536E37F;
            p89.y = (float)3.1038723E38F;
            p89.pitch = (float)4.9857443E37F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -24762);
                Debug.Assert(pack.lon == (int)839942506);
                Debug.Assert(pack.vy == (short)(short)12509);
                Debug.Assert(pack.pitch == (float) -1.5129448E38F);
                Debug.Assert(pack.yacc == (short)(short) -8612);
                Debug.Assert(pack.xacc == (short)(short) -29720);
                Debug.Assert(pack.roll == (float) -2.6032495E38F);
                Debug.Assert(pack.vx == (short)(short)12888);
                Debug.Assert(pack.yawspeed == (float)2.695468E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.161619E38F);
                Debug.Assert(pack.yaw == (float) -1.3886856E38F);
                Debug.Assert(pack.rollspeed == (float)2.9730384E38F);
                Debug.Assert(pack.lat == (int) -2096595049);
                Debug.Assert(pack.alt == (int) -94639170);
                Debug.Assert(pack.vz == (short)(short) -14272);
                Debug.Assert(pack.time_usec == (ulong)301478725766168529L);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vy = (short)(short)12509;
            p90.pitch = (float) -1.5129448E38F;
            p90.alt = (int) -94639170;
            p90.time_usec = (ulong)301478725766168529L;
            p90.vz = (short)(short) -14272;
            p90.yaw = (float) -1.3886856E38F;
            p90.lat = (int) -2096595049;
            p90.xacc = (short)(short) -29720;
            p90.yacc = (short)(short) -8612;
            p90.vx = (short)(short)12888;
            p90.zacc = (short)(short) -24762;
            p90.yawspeed = (float)2.695468E38F;
            p90.rollspeed = (float)2.9730384E38F;
            p90.pitchspeed = (float) -2.161619E38F;
            p90.roll = (float) -2.6032495E38F;
            p90.lon = (int)839942506;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux4 == (float) -4.3661086E37F);
                Debug.Assert(pack.aux1 == (float)7.794691E36F);
                Debug.Assert(pack.roll_ailerons == (float)2.3011844E37F);
                Debug.Assert(pack.aux3 == (float)3.065164E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.throttle == (float) -6.2858564E37F);
                Debug.Assert(pack.time_usec == (ulong)8116729587247021911L);
                Debug.Assert(pack.aux2 == (float)2.817953E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)213);
                Debug.Assert(pack.yaw_rudder == (float)1.3087792E38F);
                Debug.Assert(pack.pitch_elevator == (float)2.2286277E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux1 = (float)7.794691E36F;
            p91.aux4 = (float) -4.3661086E37F;
            p91.roll_ailerons = (float)2.3011844E37F;
            p91.pitch_elevator = (float)2.2286277E38F;
            p91.aux3 = (float)3.065164E38F;
            p91.aux2 = (float)2.817953E37F;
            p91.nav_mode = (byte)(byte)213;
            p91.time_usec = (ulong)8116729587247021911L;
            p91.throttle = (float) -6.2858564E37F;
            p91.yaw_rudder = (float)1.3087792E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)63060);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)34571);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)28110);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)39254);
                Debug.Assert(pack.rssi == (byte)(byte)182);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)5890);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)19096);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)23931);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)63500);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)41249);
                Debug.Assert(pack.time_usec == (ulong)3551722969451276362L);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)56888);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)39544);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)19404);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan6_raw = (ushort)(ushort)39544;
            p92.chan4_raw = (ushort)(ushort)28110;
            p92.chan3_raw = (ushort)(ushort)34571;
            p92.chan8_raw = (ushort)(ushort)19404;
            p92.chan7_raw = (ushort)(ushort)63060;
            p92.chan1_raw = (ushort)(ushort)56888;
            p92.time_usec = (ulong)3551722969451276362L;
            p92.chan2_raw = (ushort)(ushort)63500;
            p92.chan10_raw = (ushort)(ushort)19096;
            p92.rssi = (byte)(byte)182;
            p92.chan12_raw = (ushort)(ushort)23931;
            p92.chan11_raw = (ushort)(ushort)39254;
            p92.chan5_raw = (ushort)(ushort)41249;
            p92.chan9_raw = (ushort)(ushort)5890;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.8321373E38F, 1.856241E38F, 2.4541334E38F, 2.0907599E38F, 1.7382684E38F, -3.0562687E38F, 5.3793864E37F, -2.484619E38F, -5.2032554E37F, -2.471526E38F, 1.504734E38F, -1.5327269E38F, -2.2311308E38F, -2.5650715E38F, -1.5759704E38F, -1.2780432E38F}));
                Debug.Assert(pack.flags == (ulong)825065294145295486L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.time_usec == (ulong)1225738061431370530L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)825065294145295486L;
            p93.time_usec = (ulong)1225738061431370530L;
            p93.controls_SET(new float[] {2.8321373E38F, 1.856241E38F, 2.4541334E38F, 2.0907599E38F, 1.7382684E38F, -3.0562687E38F, 5.3793864E37F, -2.484619E38F, -5.2032554E37F, -2.471526E38F, 1.504734E38F, -1.5327269E38F, -2.2311308E38F, -2.5650715E38F, -1.5759704E38F, -1.2780432E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_comp_m_x == (float) -1.2744076E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)37);
                Debug.Assert(pack.quality == (byte)(byte)158);
                Debug.Assert(pack.ground_distance == (float)1.1123246E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.20054E38F);
                Debug.Assert(pack.time_usec == (ulong)2152783742987564484L);
                Debug.Assert(pack.flow_y == (short)(short)1843);
                Debug.Assert(pack.flow_x == (short)(short) -6844);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.8229315E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)3.108826E38F);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_y_SET((float)2.20054E38F, PH) ;
            p100.ground_distance = (float)1.1123246E38F;
            p100.quality = (byte)(byte)158;
            p100.time_usec = (ulong)2152783742987564484L;
            p100.flow_x = (short)(short) -6844;
            p100.flow_comp_m_y = (float)3.108826E38F;
            p100.sensor_id = (byte)(byte)37;
            p100.flow_comp_m_x = (float) -1.2744076E38F;
            p100.flow_y = (short)(short)1843;
            p100.flow_rate_x_SET((float) -2.8229315E38F, PH) ;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.1287263E38F);
                Debug.Assert(pack.z == (float) -2.1564172E37F);
                Debug.Assert(pack.x == (float)6.254663E37F);
                Debug.Assert(pack.roll == (float) -2.722144E38F);
                Debug.Assert(pack.usec == (ulong)7969835516154294622L);
                Debug.Assert(pack.y == (float)2.9195713E38F);
                Debug.Assert(pack.yaw == (float) -2.975146E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float) -2.975146E38F;
            p101.usec = (ulong)7969835516154294622L;
            p101.roll = (float) -2.722144E38F;
            p101.z = (float) -2.1564172E37F;
            p101.y = (float)2.9195713E38F;
            p101.x = (float)6.254663E37F;
            p101.pitch = (float) -2.1287263E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)4.956478E37F);
                Debug.Assert(pack.pitch == (float)6.518419E37F);
                Debug.Assert(pack.yaw == (float) -1.4222769E38F);
                Debug.Assert(pack.z == (float)3.3488763E38F);
                Debug.Assert(pack.usec == (ulong)8120855709269460611L);
                Debug.Assert(pack.roll == (float)2.7448954E38F);
                Debug.Assert(pack.y == (float)9.103547E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float)2.7448954E38F;
            p102.z = (float)3.3488763E38F;
            p102.pitch = (float)6.518419E37F;
            p102.y = (float)9.103547E37F;
            p102.usec = (ulong)8120855709269460611L;
            p102.yaw = (float) -1.4222769E38F;
            p102.x = (float)4.956478E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)8160251105803025282L);
                Debug.Assert(pack.y == (float) -1.6940555E38F);
                Debug.Assert(pack.z == (float)6.539376E37F);
                Debug.Assert(pack.x == (float) -4.98056E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)8160251105803025282L;
            p103.x = (float) -4.98056E37F;
            p103.z = (float)6.539376E37F;
            p103.y = (float) -1.6940555E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -8.519032E37F);
                Debug.Assert(pack.yaw == (float)1.4949709E38F);
                Debug.Assert(pack.z == (float)2.3510686E38F);
                Debug.Assert(pack.roll == (float) -1.0871689E38F);
                Debug.Assert(pack.usec == (ulong)4454880647516004591L);
                Debug.Assert(pack.pitch == (float) -2.5298967E37F);
                Debug.Assert(pack.x == (float)2.1925078E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float) -1.0871689E38F;
            p104.z = (float)2.3510686E38F;
            p104.yaw = (float)1.4949709E38F;
            p104.pitch = (float) -2.5298967E37F;
            p104.x = (float)2.1925078E38F;
            p104.y = (float) -8.519032E37F;
            p104.usec = (ulong)4454880647516004591L;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -3.1437051E38F);
                Debug.Assert(pack.zgyro == (float) -2.4927964E38F);
                Debug.Assert(pack.ymag == (float) -2.8741655E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)5946);
                Debug.Assert(pack.xgyro == (float)7.099431E37F);
                Debug.Assert(pack.yacc == (float)2.7643405E38F);
                Debug.Assert(pack.zacc == (float) -2.0917655E38F);
                Debug.Assert(pack.time_usec == (ulong)4395837716698249377L);
                Debug.Assert(pack.xmag == (float)2.392386E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.1341756E38F);
                Debug.Assert(pack.abs_pressure == (float)1.823887E38F);
                Debug.Assert(pack.pressure_alt == (float)1.7640555E38F);
                Debug.Assert(pack.temperature == (float) -3.5894163E37F);
                Debug.Assert(pack.xacc == (float) -2.1045483E38F);
                Debug.Assert(pack.zmag == (float) -3.1235292E38F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.temperature = (float) -3.5894163E37F;
            p105.yacc = (float)2.7643405E38F;
            p105.xmag = (float)2.392386E38F;
            p105.zmag = (float) -3.1235292E38F;
            p105.fields_updated = (ushort)(ushort)5946;
            p105.pressure_alt = (float)1.7640555E38F;
            p105.ymag = (float) -2.8741655E38F;
            p105.abs_pressure = (float)1.823887E38F;
            p105.time_usec = (ulong)4395837716698249377L;
            p105.ygyro = (float) -3.1437051E38F;
            p105.xacc = (float) -2.1045483E38F;
            p105.zacc = (float) -2.0917655E38F;
            p105.xgyro = (float)7.099431E37F;
            p105.diff_pressure = (float) -3.1341756E38F;
            p105.zgyro = (float) -2.4927964E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)666597510U);
                Debug.Assert(pack.integrated_xgyro == (float)2.0298057E38F);
                Debug.Assert(pack.integrated_ygyro == (float)2.4865758E38F);
                Debug.Assert(pack.integrated_y == (float)1.1000756E38F);
                Debug.Assert(pack.integration_time_us == (uint)640069585U);
                Debug.Assert(pack.quality == (byte)(byte)98);
                Debug.Assert(pack.distance == (float) -3.3585847E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)143);
                Debug.Assert(pack.integrated_zgyro == (float) -2.1242548E38F);
                Debug.Assert(pack.time_usec == (ulong)2408610640972989740L);
                Debug.Assert(pack.temperature == (short)(short)25282);
                Debug.Assert(pack.integrated_x == (float) -2.1095006E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.quality = (byte)(byte)98;
            p106.integrated_y = (float)1.1000756E38F;
            p106.sensor_id = (byte)(byte)143;
            p106.distance = (float) -3.3585847E38F;
            p106.time_delta_distance_us = (uint)666597510U;
            p106.time_usec = (ulong)2408610640972989740L;
            p106.integration_time_us = (uint)640069585U;
            p106.integrated_zgyro = (float) -2.1242548E38F;
            p106.temperature = (short)(short)25282;
            p106.integrated_ygyro = (float)2.4865758E38F;
            p106.integrated_xgyro = (float)2.0298057E38F;
            p106.integrated_x = (float) -2.1095006E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float) -3.1632401E38F);
                Debug.Assert(pack.xacc == (float)4.059668E37F);
                Debug.Assert(pack.xgyro == (float)2.8845832E38F);
                Debug.Assert(pack.time_usec == (ulong)958898788034884083L);
                Debug.Assert(pack.yacc == (float) -1.557537E38F);
                Debug.Assert(pack.diff_pressure == (float) -2.3061619E38F);
                Debug.Assert(pack.zmag == (float)2.5744588E38F);
                Debug.Assert(pack.temperature == (float)9.423252E37F);
                Debug.Assert(pack.fields_updated == (uint)3458175323U);
                Debug.Assert(pack.abs_pressure == (float) -3.7630872E37F);
                Debug.Assert(pack.pressure_alt == (float) -1.5615354E38F);
                Debug.Assert(pack.ygyro == (float) -1.8463266E38F);
                Debug.Assert(pack.zgyro == (float)9.625037E37F);
                Debug.Assert(pack.ymag == (float) -8.150866E37F);
                Debug.Assert(pack.zacc == (float)2.3403025E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.yacc = (float) -1.557537E38F;
            p107.diff_pressure = (float) -2.3061619E38F;
            p107.zacc = (float)2.3403025E38F;
            p107.time_usec = (ulong)958898788034884083L;
            p107.temperature = (float)9.423252E37F;
            p107.zmag = (float)2.5744588E38F;
            p107.pressure_alt = (float) -1.5615354E38F;
            p107.zgyro = (float)9.625037E37F;
            p107.ygyro = (float) -1.8463266E38F;
            p107.xacc = (float)4.059668E37F;
            p107.xmag = (float) -3.1632401E38F;
            p107.abs_pressure = (float) -3.7630872E37F;
            p107.ymag = (float) -8.150866E37F;
            p107.xgyro = (float)2.8845832E38F;
            p107.fields_updated = (uint)3458175323U;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)2.5589525E38F);
                Debug.Assert(pack.alt == (float)2.9654242E38F);
                Debug.Assert(pack.yaw == (float)2.2025425E38F);
                Debug.Assert(pack.zgyro == (float) -1.759108E38F);
                Debug.Assert(pack.vn == (float)8.297245E37F);
                Debug.Assert(pack.xgyro == (float)3.971519E37F);
                Debug.Assert(pack.yacc == (float)1.7424655E38F);
                Debug.Assert(pack.lon == (float)1.2614446E37F);
                Debug.Assert(pack.roll == (float)2.1011445E38F);
                Debug.Assert(pack.pitch == (float) -5.6795493E37F);
                Debug.Assert(pack.q1 == (float) -1.548892E38F);
                Debug.Assert(pack.lat == (float) -4.493617E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.666046E38F);
                Debug.Assert(pack.q2 == (float)2.793085E38F);
                Debug.Assert(pack.xacc == (float) -2.7894438E38F);
                Debug.Assert(pack.ygyro == (float)1.9812482E38F);
                Debug.Assert(pack.vd == (float) -2.574271E38F);
                Debug.Assert(pack.zacc == (float) -1.3849717E37F);
                Debug.Assert(pack.std_dev_vert == (float) -1.6688682E38F);
                Debug.Assert(pack.q4 == (float) -4.292902E37F);
                Debug.Assert(pack.q3 == (float) -3.102768E37F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q2 = (float)2.793085E38F;
            p108.yaw = (float)2.2025425E38F;
            p108.yacc = (float)1.7424655E38F;
            p108.ygyro = (float)1.9812482E38F;
            p108.ve = (float)2.5589525E38F;
            p108.alt = (float)2.9654242E38F;
            p108.xgyro = (float)3.971519E37F;
            p108.zacc = (float) -1.3849717E37F;
            p108.q4 = (float) -4.292902E37F;
            p108.q3 = (float) -3.102768E37F;
            p108.roll = (float)2.1011445E38F;
            p108.q1 = (float) -1.548892E38F;
            p108.lat = (float) -4.493617E37F;
            p108.pitch = (float) -5.6795493E37F;
            p108.zgyro = (float) -1.759108E38F;
            p108.xacc = (float) -2.7894438E38F;
            p108.std_dev_horz = (float)2.666046E38F;
            p108.std_dev_vert = (float) -1.6688682E38F;
            p108.lon = (float)1.2614446E37F;
            p108.vd = (float) -2.574271E38F;
            p108.vn = (float)8.297245E37F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)17);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)38789);
                Debug.Assert(pack.noise == (byte)(byte)6);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)48575);
                Debug.Assert(pack.remrssi == (byte)(byte)122);
                Debug.Assert(pack.remnoise == (byte)(byte)146);
                Debug.Assert(pack.rssi == (byte)(byte)222);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)222;
            p109.remnoise = (byte)(byte)146;
            p109.fixed_ = (ushort)(ushort)48575;
            p109.remrssi = (byte)(byte)122;
            p109.rxerrors = (ushort)(ushort)38789;
            p109.txbuf = (byte)(byte)17;
            p109.noise = (byte)(byte)6;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)26, (byte)146, (byte)16, (byte)20, (byte)75, (byte)190, (byte)237, (byte)71, (byte)55, (byte)254, (byte)201, (byte)240, (byte)45, (byte)202, (byte)50, (byte)255, (byte)46, (byte)159, (byte)130, (byte)19, (byte)17, (byte)190, (byte)103, (byte)235, (byte)125, (byte)61, (byte)41, (byte)235, (byte)26, (byte)14, (byte)186, (byte)110, (byte)166, (byte)143, (byte)201, (byte)138, (byte)204, (byte)55, (byte)175, (byte)154, (byte)13, (byte)30, (byte)130, (byte)178, (byte)18, (byte)126, (byte)254, (byte)108, (byte)210, (byte)75, (byte)97, (byte)195, (byte)24, (byte)39, (byte)126, (byte)237, (byte)251, (byte)76, (byte)129, (byte)131, (byte)223, (byte)52, (byte)87, (byte)231, (byte)26, (byte)168, (byte)65, (byte)196, (byte)22, (byte)227, (byte)54, (byte)133, (byte)12, (byte)96, (byte)139, (byte)100, (byte)205, (byte)105, (byte)221, (byte)14, (byte)12, (byte)226, (byte)199, (byte)12, (byte)136, (byte)119, (byte)161, (byte)72, (byte)186, (byte)131, (byte)206, (byte)138, (byte)213, (byte)161, (byte)3, (byte)53, (byte)128, (byte)52, (byte)205, (byte)163, (byte)179, (byte)63, (byte)70, (byte)248, (byte)127, (byte)135, (byte)73, (byte)8, (byte)67, (byte)110, (byte)69, (byte)20, (byte)239, (byte)62, (byte)34, (byte)72, (byte)172, (byte)212, (byte)80, (byte)48, (byte)88, (byte)29, (byte)100, (byte)130, (byte)6, (byte)142, (byte)123, (byte)224, (byte)224, (byte)207, (byte)80, (byte)163, (byte)243, (byte)144, (byte)155, (byte)106, (byte)47, (byte)165, (byte)180, (byte)6, (byte)80, (byte)157, (byte)50, (byte)255, (byte)137, (byte)70, (byte)196, (byte)172, (byte)185, (byte)25, (byte)167, (byte)199, (byte)122, (byte)40, (byte)159, (byte)184, (byte)150, (byte)80, (byte)247, (byte)160, (byte)25, (byte)63, (byte)36, (byte)202, (byte)97, (byte)142, (byte)152, (byte)73, (byte)207, (byte)231, (byte)169, (byte)57, (byte)151, (byte)52, (byte)173, (byte)228, (byte)85, (byte)229, (byte)31, (byte)189, (byte)42, (byte)47, (byte)167, (byte)78, (byte)67, (byte)171, (byte)35, (byte)248, (byte)137, (byte)172, (byte)160, (byte)193, (byte)164, (byte)141, (byte)0, (byte)176, (byte)58, (byte)227, (byte)28, (byte)7, (byte)154, (byte)206, (byte)78, (byte)210, (byte)154, (byte)133, (byte)158, (byte)86, (byte)172, (byte)38, (byte)198, (byte)186, (byte)173, (byte)161, (byte)144, (byte)145, (byte)31, (byte)61, (byte)56, (byte)130, (byte)181, (byte)250, (byte)217, (byte)144, (byte)82, (byte)35, (byte)136, (byte)103, (byte)148, (byte)215, (byte)73, (byte)40, (byte)9, (byte)94, (byte)210, (byte)103, (byte)184, (byte)240, (byte)190, (byte)76, (byte)104, (byte)72, (byte)39, (byte)122, (byte)235, (byte)121, (byte)166, (byte)2, (byte)90, (byte)128, (byte)70}));
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)215);
                Debug.Assert(pack.target_network == (byte)(byte)129);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)213;
            p110.target_network = (byte)(byte)129;
            p110.payload_SET(new byte[] {(byte)26, (byte)146, (byte)16, (byte)20, (byte)75, (byte)190, (byte)237, (byte)71, (byte)55, (byte)254, (byte)201, (byte)240, (byte)45, (byte)202, (byte)50, (byte)255, (byte)46, (byte)159, (byte)130, (byte)19, (byte)17, (byte)190, (byte)103, (byte)235, (byte)125, (byte)61, (byte)41, (byte)235, (byte)26, (byte)14, (byte)186, (byte)110, (byte)166, (byte)143, (byte)201, (byte)138, (byte)204, (byte)55, (byte)175, (byte)154, (byte)13, (byte)30, (byte)130, (byte)178, (byte)18, (byte)126, (byte)254, (byte)108, (byte)210, (byte)75, (byte)97, (byte)195, (byte)24, (byte)39, (byte)126, (byte)237, (byte)251, (byte)76, (byte)129, (byte)131, (byte)223, (byte)52, (byte)87, (byte)231, (byte)26, (byte)168, (byte)65, (byte)196, (byte)22, (byte)227, (byte)54, (byte)133, (byte)12, (byte)96, (byte)139, (byte)100, (byte)205, (byte)105, (byte)221, (byte)14, (byte)12, (byte)226, (byte)199, (byte)12, (byte)136, (byte)119, (byte)161, (byte)72, (byte)186, (byte)131, (byte)206, (byte)138, (byte)213, (byte)161, (byte)3, (byte)53, (byte)128, (byte)52, (byte)205, (byte)163, (byte)179, (byte)63, (byte)70, (byte)248, (byte)127, (byte)135, (byte)73, (byte)8, (byte)67, (byte)110, (byte)69, (byte)20, (byte)239, (byte)62, (byte)34, (byte)72, (byte)172, (byte)212, (byte)80, (byte)48, (byte)88, (byte)29, (byte)100, (byte)130, (byte)6, (byte)142, (byte)123, (byte)224, (byte)224, (byte)207, (byte)80, (byte)163, (byte)243, (byte)144, (byte)155, (byte)106, (byte)47, (byte)165, (byte)180, (byte)6, (byte)80, (byte)157, (byte)50, (byte)255, (byte)137, (byte)70, (byte)196, (byte)172, (byte)185, (byte)25, (byte)167, (byte)199, (byte)122, (byte)40, (byte)159, (byte)184, (byte)150, (byte)80, (byte)247, (byte)160, (byte)25, (byte)63, (byte)36, (byte)202, (byte)97, (byte)142, (byte)152, (byte)73, (byte)207, (byte)231, (byte)169, (byte)57, (byte)151, (byte)52, (byte)173, (byte)228, (byte)85, (byte)229, (byte)31, (byte)189, (byte)42, (byte)47, (byte)167, (byte)78, (byte)67, (byte)171, (byte)35, (byte)248, (byte)137, (byte)172, (byte)160, (byte)193, (byte)164, (byte)141, (byte)0, (byte)176, (byte)58, (byte)227, (byte)28, (byte)7, (byte)154, (byte)206, (byte)78, (byte)210, (byte)154, (byte)133, (byte)158, (byte)86, (byte)172, (byte)38, (byte)198, (byte)186, (byte)173, (byte)161, (byte)144, (byte)145, (byte)31, (byte)61, (byte)56, (byte)130, (byte)181, (byte)250, (byte)217, (byte)144, (byte)82, (byte)35, (byte)136, (byte)103, (byte)148, (byte)215, (byte)73, (byte)40, (byte)9, (byte)94, (byte)210, (byte)103, (byte)184, (byte)240, (byte)190, (byte)76, (byte)104, (byte)72, (byte)39, (byte)122, (byte)235, (byte)121, (byte)166, (byte)2, (byte)90, (byte)128, (byte)70}, 0) ;
            p110.target_system = (byte)(byte)215;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -7300135461039464896L);
                Debug.Assert(pack.tc1 == (long) -997353428767718140L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -7300135461039464896L;
            p111.tc1 = (long) -997353428767718140L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3622037181U);
                Debug.Assert(pack.time_usec == (ulong)4023302772762859268L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3622037181U;
            p112.time_usec = (ulong)4023302772762859268L;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)279690348);
                Debug.Assert(pack.fix_type == (byte)(byte)220);
                Debug.Assert(pack.satellites_visible == (byte)(byte)83);
                Debug.Assert(pack.lon == (int) -810742360);
                Debug.Assert(pack.vd == (short)(short)7276);
                Debug.Assert(pack.eph == (ushort)(ushort)62677);
                Debug.Assert(pack.vel == (ushort)(ushort)1588);
                Debug.Assert(pack.vn == (short)(short)23605);
                Debug.Assert(pack.cog == (ushort)(ushort)43679);
                Debug.Assert(pack.epv == (ushort)(ushort)8997);
                Debug.Assert(pack.alt == (int)1921778436);
                Debug.Assert(pack.ve == (short)(short)23212);
                Debug.Assert(pack.time_usec == (ulong)7927117823922975713L);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vel = (ushort)(ushort)1588;
            p113.epv = (ushort)(ushort)8997;
            p113.eph = (ushort)(ushort)62677;
            p113.vd = (short)(short)7276;
            p113.alt = (int)1921778436;
            p113.lat = (int)279690348;
            p113.vn = (short)(short)23605;
            p113.fix_type = (byte)(byte)220;
            p113.lon = (int) -810742360;
            p113.cog = (ushort)(ushort)43679;
            p113.ve = (short)(short)23212;
            p113.satellites_visible = (byte)(byte)83;
            p113.time_usec = (ulong)7927117823922975713L;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -16812);
                Debug.Assert(pack.quality == (byte)(byte)29);
                Debug.Assert(pack.integrated_ygyro == (float)2.1685727E38F);
                Debug.Assert(pack.integrated_y == (float)2.3092803E38F);
                Debug.Assert(pack.integrated_zgyro == (float)1.6768444E38F);
                Debug.Assert(pack.time_usec == (ulong)6526004496299679638L);
                Debug.Assert(pack.distance == (float) -3.567706E37F);
                Debug.Assert(pack.integrated_xgyro == (float)4.080525E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)18);
                Debug.Assert(pack.time_delta_distance_us == (uint)3058456818U);
                Debug.Assert(pack.integrated_x == (float)1.0243588E38F);
                Debug.Assert(pack.integration_time_us == (uint)2925565713U);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.quality = (byte)(byte)29;
            p114.integrated_x = (float)1.0243588E38F;
            p114.sensor_id = (byte)(byte)18;
            p114.integrated_zgyro = (float)1.6768444E38F;
            p114.time_usec = (ulong)6526004496299679638L;
            p114.integrated_ygyro = (float)2.1685727E38F;
            p114.integrated_y = (float)2.3092803E38F;
            p114.integrated_xgyro = (float)4.080525E37F;
            p114.time_delta_distance_us = (uint)3058456818U;
            p114.distance = (float) -3.567706E37F;
            p114.integration_time_us = (uint)2925565713U;
            p114.temperature = (short)(short) -16812;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1160671230);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)8029);
                Debug.Assert(pack.rollspeed == (float) -2.3435101E38F);
                Debug.Assert(pack.lat == (int) -602241673);
                Debug.Assert(pack.time_usec == (ulong)8097186695976945155L);
                Debug.Assert(pack.zacc == (short)(short) -23099);
                Debug.Assert(pack.vz == (short)(short)26997);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.9321917E38F, 1.6772778E36F, -2.2232095E38F, -1.6510034E38F}));
                Debug.Assert(pack.vx == (short)(short) -24442);
                Debug.Assert(pack.alt == (int)1607414025);
                Debug.Assert(pack.xacc == (short)(short) -26678);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)17348);
                Debug.Assert(pack.yacc == (short)(short) -5277);
                Debug.Assert(pack.yawspeed == (float) -2.593028E38F);
                Debug.Assert(pack.vy == (short)(short)8117);
                Debug.Assert(pack.pitchspeed == (float)2.0790022E38F);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.ind_airspeed = (ushort)(ushort)17348;
            p115.yawspeed = (float) -2.593028E38F;
            p115.lon = (int) -1160671230;
            p115.true_airspeed = (ushort)(ushort)8029;
            p115.vz = (short)(short)26997;
            p115.alt = (int)1607414025;
            p115.vy = (short)(short)8117;
            p115.time_usec = (ulong)8097186695976945155L;
            p115.lat = (int) -602241673;
            p115.xacc = (short)(short) -26678;
            p115.zacc = (short)(short) -23099;
            p115.rollspeed = (float) -2.3435101E38F;
            p115.pitchspeed = (float)2.0790022E38F;
            p115.yacc = (short)(short) -5277;
            p115.attitude_quaternion_SET(new float[] {2.9321917E38F, 1.6772778E36F, -2.2232095E38F, -1.6510034E38F}, 0) ;
            p115.vx = (short)(short) -24442;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -22821);
                Debug.Assert(pack.xgyro == (short)(short) -20473);
                Debug.Assert(pack.zacc == (short)(short)10156);
                Debug.Assert(pack.time_boot_ms == (uint)610903411U);
                Debug.Assert(pack.ymag == (short)(short) -6641);
                Debug.Assert(pack.yacc == (short)(short) -11267);
                Debug.Assert(pack.zgyro == (short)(short) -14553);
                Debug.Assert(pack.xmag == (short)(short) -571);
                Debug.Assert(pack.xacc == (short)(short)8615);
                Debug.Assert(pack.ygyro == (short)(short)4991);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zacc = (short)(short)10156;
            p116.xmag = (short)(short) -571;
            p116.ygyro = (short)(short)4991;
            p116.zmag = (short)(short) -22821;
            p116.zgyro = (short)(short) -14553;
            p116.time_boot_ms = (uint)610903411U;
            p116.yacc = (short)(short) -11267;
            p116.ymag = (short)(short) -6641;
            p116.xacc = (short)(short)8615;
            p116.xgyro = (short)(short) -20473;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)12);
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.start == (ushort)(ushort)7146);
                Debug.Assert(pack.end == (ushort)(ushort)3150);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)3150;
            p117.target_system = (byte)(byte)94;
            p117.target_component = (byte)(byte)12;
            p117.start = (ushort)(ushort)7146;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)64708);
                Debug.Assert(pack.time_utc == (uint)1468167280U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)11771);
                Debug.Assert(pack.size == (uint)2317671318U);
                Debug.Assert(pack.id == (ushort)(ushort)610);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)1468167280U;
            p118.size = (uint)2317671318U;
            p118.num_logs = (ushort)(ushort)11771;
            p118.id = (ushort)(ushort)610;
            p118.last_log_num = (ushort)(ushort)64708;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.count == (uint)1097367469U);
                Debug.Assert(pack.ofs == (uint)4078466877U);
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.id == (ushort)(ushort)60819);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)59;
            p119.id = (ushort)(ushort)60819;
            p119.count = (uint)1097367469U;
            p119.target_system = (byte)(byte)6;
            p119.ofs = (uint)4078466877U;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)153, (byte)158, (byte)67, (byte)1, (byte)193, (byte)176, (byte)48, (byte)200, (byte)184, (byte)14, (byte)61, (byte)127, (byte)27, (byte)193, (byte)170, (byte)60, (byte)243, (byte)42, (byte)219, (byte)107, (byte)46, (byte)224, (byte)229, (byte)221, (byte)1, (byte)26, (byte)103, (byte)92, (byte)94, (byte)239, (byte)210, (byte)184, (byte)109, (byte)206, (byte)227, (byte)162, (byte)122, (byte)203, (byte)139, (byte)30, (byte)155, (byte)203, (byte)28, (byte)59, (byte)137, (byte)139, (byte)69, (byte)70, (byte)78, (byte)16, (byte)100, (byte)205, (byte)150, (byte)188, (byte)136, (byte)185, (byte)60, (byte)36, (byte)158, (byte)40, (byte)195, (byte)225, (byte)236, (byte)105, (byte)138, (byte)1, (byte)198, (byte)217, (byte)42, (byte)107, (byte)132, (byte)114, (byte)173, (byte)84, (byte)100, (byte)73, (byte)78, (byte)64, (byte)126, (byte)108, (byte)195, (byte)157, (byte)44, (byte)45, (byte)93, (byte)224, (byte)109, (byte)41, (byte)249, (byte)211}));
                Debug.Assert(pack.id == (ushort)(ushort)28028);
                Debug.Assert(pack.ofs == (uint)1903567980U);
                Debug.Assert(pack.count == (byte)(byte)57);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)57;
            p120.ofs = (uint)1903567980U;
            p120.data__SET(new byte[] {(byte)153, (byte)158, (byte)67, (byte)1, (byte)193, (byte)176, (byte)48, (byte)200, (byte)184, (byte)14, (byte)61, (byte)127, (byte)27, (byte)193, (byte)170, (byte)60, (byte)243, (byte)42, (byte)219, (byte)107, (byte)46, (byte)224, (byte)229, (byte)221, (byte)1, (byte)26, (byte)103, (byte)92, (byte)94, (byte)239, (byte)210, (byte)184, (byte)109, (byte)206, (byte)227, (byte)162, (byte)122, (byte)203, (byte)139, (byte)30, (byte)155, (byte)203, (byte)28, (byte)59, (byte)137, (byte)139, (byte)69, (byte)70, (byte)78, (byte)16, (byte)100, (byte)205, (byte)150, (byte)188, (byte)136, (byte)185, (byte)60, (byte)36, (byte)158, (byte)40, (byte)195, (byte)225, (byte)236, (byte)105, (byte)138, (byte)1, (byte)198, (byte)217, (byte)42, (byte)107, (byte)132, (byte)114, (byte)173, (byte)84, (byte)100, (byte)73, (byte)78, (byte)64, (byte)126, (byte)108, (byte)195, (byte)157, (byte)44, (byte)45, (byte)93, (byte)224, (byte)109, (byte)41, (byte)249, (byte)211}, 0) ;
            p120.id = (ushort)(ushort)28028;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.target_system == (byte)(byte)33);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)69;
            p121.target_system = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.target_system == (byte)(byte)0);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)0;
            p122.target_component = (byte)(byte)195;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)240, (byte)2, (byte)211, (byte)229, (byte)164, (byte)172, (byte)161, (byte)175, (byte)19, (byte)63, (byte)170, (byte)109, (byte)248, (byte)83, (byte)58, (byte)251, (byte)246, (byte)3, (byte)14, (byte)206, (byte)57, (byte)5, (byte)228, (byte)141, (byte)37, (byte)34, (byte)117, (byte)133, (byte)141, (byte)89, (byte)213, (byte)41, (byte)11, (byte)231, (byte)85, (byte)162, (byte)150, (byte)208, (byte)128, (byte)244, (byte)225, (byte)251, (byte)190, (byte)186, (byte)132, (byte)249, (byte)155, (byte)19, (byte)77, (byte)191, (byte)241, (byte)25, (byte)166, (byte)42, (byte)120, (byte)59, (byte)79, (byte)51, (byte)222, (byte)61, (byte)126, (byte)61, (byte)101, (byte)221, (byte)68, (byte)99, (byte)169, (byte)179, (byte)100, (byte)170, (byte)222, (byte)4, (byte)217, (byte)44, (byte)68, (byte)120, (byte)211, (byte)165, (byte)114, (byte)22, (byte)233, (byte)111, (byte)236, (byte)215, (byte)25, (byte)232, (byte)167, (byte)30, (byte)194, (byte)94, (byte)200, (byte)164, (byte)107, (byte)99, (byte)229, (byte)167, (byte)4, (byte)165, (byte)86, (byte)133, (byte)229, (byte)224, (byte)61, (byte)225, (byte)109, (byte)204, (byte)186, (byte)153, (byte)151, (byte)227}));
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.len == (byte)(byte)236);
                Debug.Assert(pack.target_system == (byte)(byte)109);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_component = (byte)(byte)168;
            p123.data__SET(new byte[] {(byte)240, (byte)2, (byte)211, (byte)229, (byte)164, (byte)172, (byte)161, (byte)175, (byte)19, (byte)63, (byte)170, (byte)109, (byte)248, (byte)83, (byte)58, (byte)251, (byte)246, (byte)3, (byte)14, (byte)206, (byte)57, (byte)5, (byte)228, (byte)141, (byte)37, (byte)34, (byte)117, (byte)133, (byte)141, (byte)89, (byte)213, (byte)41, (byte)11, (byte)231, (byte)85, (byte)162, (byte)150, (byte)208, (byte)128, (byte)244, (byte)225, (byte)251, (byte)190, (byte)186, (byte)132, (byte)249, (byte)155, (byte)19, (byte)77, (byte)191, (byte)241, (byte)25, (byte)166, (byte)42, (byte)120, (byte)59, (byte)79, (byte)51, (byte)222, (byte)61, (byte)126, (byte)61, (byte)101, (byte)221, (byte)68, (byte)99, (byte)169, (byte)179, (byte)100, (byte)170, (byte)222, (byte)4, (byte)217, (byte)44, (byte)68, (byte)120, (byte)211, (byte)165, (byte)114, (byte)22, (byte)233, (byte)111, (byte)236, (byte)215, (byte)25, (byte)232, (byte)167, (byte)30, (byte)194, (byte)94, (byte)200, (byte)164, (byte)107, (byte)99, (byte)229, (byte)167, (byte)4, (byte)165, (byte)86, (byte)133, (byte)229, (byte)224, (byte)61, (byte)225, (byte)109, (byte)204, (byte)186, (byte)153, (byte)151, (byte)227}, 0) ;
            p123.len = (byte)(byte)236;
            p123.target_system = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)61607);
                Debug.Assert(pack.dgps_age == (uint)3564632675U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)85);
                Debug.Assert(pack.lon == (int)2050928892);
                Debug.Assert(pack.alt == (int) -2141469902);
                Debug.Assert(pack.cog == (ushort)(ushort)46857);
                Debug.Assert(pack.dgps_numch == (byte)(byte)239);
                Debug.Assert(pack.vel == (ushort)(ushort)61571);
                Debug.Assert(pack.eph == (ushort)(ushort)23811);
                Debug.Assert(pack.time_usec == (ulong)9011413204230603544L);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
                Debug.Assert(pack.lat == (int) -713882399);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.alt = (int) -2141469902;
            p124.epv = (ushort)(ushort)61607;
            p124.time_usec = (ulong)9011413204230603544L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.cog = (ushort)(ushort)46857;
            p124.lat = (int) -713882399;
            p124.dgps_numch = (byte)(byte)239;
            p124.lon = (int)2050928892;
            p124.eph = (ushort)(ushort)23811;
            p124.dgps_age = (uint)3564632675U;
            p124.vel = (ushort)(ushort)61571;
            p124.satellites_visible = (byte)(byte)85;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)36268);
                Debug.Assert(pack.Vcc == (ushort)(ushort)8112);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)8112;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            p125.Vservo = (ushort)(ushort)36268;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)88, (byte)85, (byte)243, (byte)124, (byte)118, (byte)107, (byte)104, (byte)195, (byte)167, (byte)163, (byte)174, (byte)89, (byte)7, (byte)253, (byte)72, (byte)31, (byte)89, (byte)20, (byte)89, (byte)71, (byte)92, (byte)176, (byte)72, (byte)123, (byte)82, (byte)143, (byte)124, (byte)102, (byte)115, (byte)100, (byte)164, (byte)237, (byte)188, (byte)252, (byte)45, (byte)30, (byte)246, (byte)64, (byte)224, (byte)75, (byte)79, (byte)204, (byte)39, (byte)56, (byte)54, (byte)164, (byte)209, (byte)75, (byte)100, (byte)81, (byte)151, (byte)139, (byte)110, (byte)152, (byte)18, (byte)235, (byte)247, (byte)133, (byte)230, (byte)9, (byte)170, (byte)142, (byte)163, (byte)163, (byte)19, (byte)143, (byte)170, (byte)131, (byte)12, (byte)65}));
                Debug.Assert(pack.count == (byte)(byte)76);
                Debug.Assert(pack.timeout == (ushort)(ushort)56962);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
                Debug.Assert(pack.baudrate == (uint)537546560U);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)76;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.timeout = (ushort)(ushort)56962;
            p126.baudrate = (uint)537546560U;
            p126.data__SET(new byte[] {(byte)88, (byte)85, (byte)243, (byte)124, (byte)118, (byte)107, (byte)104, (byte)195, (byte)167, (byte)163, (byte)174, (byte)89, (byte)7, (byte)253, (byte)72, (byte)31, (byte)89, (byte)20, (byte)89, (byte)71, (byte)92, (byte)176, (byte)72, (byte)123, (byte)82, (byte)143, (byte)124, (byte)102, (byte)115, (byte)100, (byte)164, (byte)237, (byte)188, (byte)252, (byte)45, (byte)30, (byte)246, (byte)64, (byte)224, (byte)75, (byte)79, (byte)204, (byte)39, (byte)56, (byte)54, (byte)164, (byte)209, (byte)75, (byte)100, (byte)81, (byte)151, (byte)139, (byte)110, (byte)152, (byte)18, (byte)235, (byte)247, (byte)133, (byte)230, (byte)9, (byte)170, (byte)142, (byte)163, (byte)163, (byte)19, (byte)143, (byte)170, (byte)131, (byte)12, (byte)65}, 0) ;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.iar_num_hypotheses == (int)78936929);
                Debug.Assert(pack.baseline_b_mm == (int)1881305672);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3453819886U);
                Debug.Assert(pack.baseline_a_mm == (int) -416278981);
                Debug.Assert(pack.tow == (uint)2165404748U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)61);
                Debug.Assert(pack.accuracy == (uint)2869587609U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)169);
                Debug.Assert(pack.wn == (ushort)(ushort)13129);
                Debug.Assert(pack.rtk_health == (byte)(byte)90);
                Debug.Assert(pack.baseline_c_mm == (int) -1349033003);
                Debug.Assert(pack.nsats == (byte)(byte)93);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)212);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)2869587609U;
            p127.rtk_health = (byte)(byte)90;
            p127.tow = (uint)2165404748U;
            p127.time_last_baseline_ms = (uint)3453819886U;
            p127.rtk_receiver_id = (byte)(byte)61;
            p127.nsats = (byte)(byte)93;
            p127.baseline_coords_type = (byte)(byte)212;
            p127.rtk_rate = (byte)(byte)169;
            p127.baseline_a_mm = (int) -416278981;
            p127.iar_num_hypotheses = (int)78936929;
            p127.wn = (ushort)(ushort)13129;
            p127.baseline_c_mm = (int) -1349033003;
            p127.baseline_b_mm = (int)1881305672;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)628635900);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)206);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)205);
                Debug.Assert(pack.nsats == (byte)(byte)111);
                Debug.Assert(pack.baseline_c_mm == (int) -1975340990);
                Debug.Assert(pack.rtk_rate == (byte)(byte)182);
                Debug.Assert(pack.baseline_b_mm == (int)1280229290);
                Debug.Assert(pack.accuracy == (uint)4261672752U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2916128023U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -986467417);
                Debug.Assert(pack.rtk_health == (byte)(byte)36);
                Debug.Assert(pack.tow == (uint)3376527629U);
                Debug.Assert(pack.wn == (ushort)(ushort)9982);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)4261672752U;
            p128.baseline_b_mm = (int)1280229290;
            p128.rtk_rate = (byte)(byte)182;
            p128.wn = (ushort)(ushort)9982;
            p128.time_last_baseline_ms = (uint)2916128023U;
            p128.baseline_coords_type = (byte)(byte)205;
            p128.tow = (uint)3376527629U;
            p128.nsats = (byte)(byte)111;
            p128.baseline_c_mm = (int) -1975340990;
            p128.iar_num_hypotheses = (int) -986467417;
            p128.rtk_health = (byte)(byte)36;
            p128.baseline_a_mm = (int)628635900;
            p128.rtk_receiver_id = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)26721);
                Debug.Assert(pack.yacc == (short)(short)7249);
                Debug.Assert(pack.zacc == (short)(short) -18954);
                Debug.Assert(pack.ymag == (short)(short) -19284);
                Debug.Assert(pack.xmag == (short)(short)26912);
                Debug.Assert(pack.zgyro == (short)(short) -10191);
                Debug.Assert(pack.ygyro == (short)(short) -6515);
                Debug.Assert(pack.xgyro == (short)(short) -29980);
                Debug.Assert(pack.time_boot_ms == (uint)1444442923U);
                Debug.Assert(pack.zmag == (short)(short) -3094);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short)26721;
            p129.zacc = (short)(short) -18954;
            p129.xgyro = (short)(short) -29980;
            p129.zgyro = (short)(short) -10191;
            p129.zmag = (short)(short) -3094;
            p129.time_boot_ms = (uint)1444442923U;
            p129.yacc = (short)(short)7249;
            p129.ygyro = (short)(short) -6515;
            p129.xmag = (short)(short)26912;
            p129.ymag = (short)(short) -19284;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)139);
                Debug.Assert(pack.jpg_quality == (byte)(byte)176);
                Debug.Assert(pack.width == (ushort)(ushort)22358);
                Debug.Assert(pack.height == (ushort)(ushort)44109);
                Debug.Assert(pack.size == (uint)3892504816U);
                Debug.Assert(pack.packets == (ushort)(ushort)39812);
                Debug.Assert(pack.payload == (byte)(byte)102);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.size = (uint)3892504816U;
            p130.payload = (byte)(byte)102;
            p130.width = (ushort)(ushort)22358;
            p130.height = (ushort)(ushort)44109;
            p130.jpg_quality = (byte)(byte)176;
            p130.packets = (ushort)(ushort)39812;
            p130.type = (byte)(byte)139;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)50427);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)112, (byte)106, (byte)159, (byte)29, (byte)12, (byte)44, (byte)237, (byte)15, (byte)132, (byte)134, (byte)86, (byte)174, (byte)210, (byte)189, (byte)245, (byte)95, (byte)130, (byte)97, (byte)75, (byte)3, (byte)183, (byte)47, (byte)27, (byte)188, (byte)232, (byte)196, (byte)119, (byte)238, (byte)47, (byte)15, (byte)50, (byte)133, (byte)180, (byte)59, (byte)46, (byte)214, (byte)169, (byte)153, (byte)31, (byte)69, (byte)84, (byte)172, (byte)108, (byte)62, (byte)219, (byte)110, (byte)27, (byte)146, (byte)115, (byte)161, (byte)20, (byte)143, (byte)169, (byte)224, (byte)40, (byte)43, (byte)79, (byte)58, (byte)157, (byte)209, (byte)166, (byte)170, (byte)217, (byte)72, (byte)235, (byte)85, (byte)22, (byte)23, (byte)249, (byte)168, (byte)165, (byte)153, (byte)207, (byte)187, (byte)189, (byte)253, (byte)10, (byte)212, (byte)166, (byte)143, (byte)142, (byte)165, (byte)252, (byte)209, (byte)233, (byte)167, (byte)207, (byte)137, (byte)169, (byte)164, (byte)168, (byte)48, (byte)77, (byte)118, (byte)201, (byte)41, (byte)153, (byte)163, (byte)177, (byte)193, (byte)48, (byte)211, (byte)178, (byte)141, (byte)231, (byte)70, (byte)123, (byte)158, (byte)112, (byte)143, (byte)93, (byte)100, (byte)73, (byte)185, (byte)224, (byte)72, (byte)3, (byte)75, (byte)233, (byte)199, (byte)26, (byte)88, (byte)102, (byte)94, (byte)19, (byte)38, (byte)123, (byte)147, (byte)39, (byte)137, (byte)250, (byte)39, (byte)102, (byte)218, (byte)56, (byte)82, (byte)69, (byte)206, (byte)170, (byte)58, (byte)41, (byte)105, (byte)205, (byte)161, (byte)63, (byte)10, (byte)145, (byte)166, (byte)67, (byte)33, (byte)211, (byte)42, (byte)216, (byte)246, (byte)49, (byte)248, (byte)239, (byte)247, (byte)58, (byte)82, (byte)45, (byte)163, (byte)170, (byte)213, (byte)237, (byte)23, (byte)149, (byte)173, (byte)171, (byte)138, (byte)15, (byte)123, (byte)178, (byte)238, (byte)127, (byte)236, (byte)87, (byte)173, (byte)94, (byte)21, (byte)229, (byte)242, (byte)192, (byte)56, (byte)180, (byte)23, (byte)81, (byte)175, (byte)163, (byte)157, (byte)11, (byte)125, (byte)61, (byte)55, (byte)48, (byte)45, (byte)151, (byte)26, (byte)230, (byte)157, (byte)62, (byte)190, (byte)82, (byte)218, (byte)86, (byte)201, (byte)247, (byte)41, (byte)235, (byte)169, (byte)110, (byte)88, (byte)113, (byte)115, (byte)7, (byte)43, (byte)197, (byte)29, (byte)245, (byte)122, (byte)228, (byte)158, (byte)124, (byte)102, (byte)168, (byte)229, (byte)8, (byte)21, (byte)29, (byte)116, (byte)182, (byte)244, (byte)133, (byte)183, (byte)105, (byte)112, (byte)88, (byte)69, (byte)137, (byte)143, (byte)22, (byte)228, (byte)54, (byte)89, (byte)167, (byte)136, (byte)34, (byte)236, (byte)167, (byte)190, (byte)3, (byte)98, (byte)144}));
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)112, (byte)106, (byte)159, (byte)29, (byte)12, (byte)44, (byte)237, (byte)15, (byte)132, (byte)134, (byte)86, (byte)174, (byte)210, (byte)189, (byte)245, (byte)95, (byte)130, (byte)97, (byte)75, (byte)3, (byte)183, (byte)47, (byte)27, (byte)188, (byte)232, (byte)196, (byte)119, (byte)238, (byte)47, (byte)15, (byte)50, (byte)133, (byte)180, (byte)59, (byte)46, (byte)214, (byte)169, (byte)153, (byte)31, (byte)69, (byte)84, (byte)172, (byte)108, (byte)62, (byte)219, (byte)110, (byte)27, (byte)146, (byte)115, (byte)161, (byte)20, (byte)143, (byte)169, (byte)224, (byte)40, (byte)43, (byte)79, (byte)58, (byte)157, (byte)209, (byte)166, (byte)170, (byte)217, (byte)72, (byte)235, (byte)85, (byte)22, (byte)23, (byte)249, (byte)168, (byte)165, (byte)153, (byte)207, (byte)187, (byte)189, (byte)253, (byte)10, (byte)212, (byte)166, (byte)143, (byte)142, (byte)165, (byte)252, (byte)209, (byte)233, (byte)167, (byte)207, (byte)137, (byte)169, (byte)164, (byte)168, (byte)48, (byte)77, (byte)118, (byte)201, (byte)41, (byte)153, (byte)163, (byte)177, (byte)193, (byte)48, (byte)211, (byte)178, (byte)141, (byte)231, (byte)70, (byte)123, (byte)158, (byte)112, (byte)143, (byte)93, (byte)100, (byte)73, (byte)185, (byte)224, (byte)72, (byte)3, (byte)75, (byte)233, (byte)199, (byte)26, (byte)88, (byte)102, (byte)94, (byte)19, (byte)38, (byte)123, (byte)147, (byte)39, (byte)137, (byte)250, (byte)39, (byte)102, (byte)218, (byte)56, (byte)82, (byte)69, (byte)206, (byte)170, (byte)58, (byte)41, (byte)105, (byte)205, (byte)161, (byte)63, (byte)10, (byte)145, (byte)166, (byte)67, (byte)33, (byte)211, (byte)42, (byte)216, (byte)246, (byte)49, (byte)248, (byte)239, (byte)247, (byte)58, (byte)82, (byte)45, (byte)163, (byte)170, (byte)213, (byte)237, (byte)23, (byte)149, (byte)173, (byte)171, (byte)138, (byte)15, (byte)123, (byte)178, (byte)238, (byte)127, (byte)236, (byte)87, (byte)173, (byte)94, (byte)21, (byte)229, (byte)242, (byte)192, (byte)56, (byte)180, (byte)23, (byte)81, (byte)175, (byte)163, (byte)157, (byte)11, (byte)125, (byte)61, (byte)55, (byte)48, (byte)45, (byte)151, (byte)26, (byte)230, (byte)157, (byte)62, (byte)190, (byte)82, (byte)218, (byte)86, (byte)201, (byte)247, (byte)41, (byte)235, (byte)169, (byte)110, (byte)88, (byte)113, (byte)115, (byte)7, (byte)43, (byte)197, (byte)29, (byte)245, (byte)122, (byte)228, (byte)158, (byte)124, (byte)102, (byte)168, (byte)229, (byte)8, (byte)21, (byte)29, (byte)116, (byte)182, (byte)244, (byte)133, (byte)183, (byte)105, (byte)112, (byte)88, (byte)69, (byte)137, (byte)143, (byte)22, (byte)228, (byte)54, (byte)89, (byte)167, (byte)136, (byte)34, (byte)236, (byte)167, (byte)190, (byte)3, (byte)98, (byte)144}, 0) ;
            p131.seqnr = (ushort)(ushort)50427;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55819);
                Debug.Assert(pack.min_distance == (ushort)(ushort)57473);
                Debug.Assert(pack.id == (byte)(byte)192);
                Debug.Assert(pack.covariance == (byte)(byte)254);
                Debug.Assert(pack.current_distance == (ushort)(ushort)24389);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.time_boot_ms == (uint)1081016382U);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)24389;
            p132.covariance = (byte)(byte)254;
            p132.time_boot_ms = (uint)1081016382U;
            p132.id = (byte)(byte)192;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_135;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.max_distance = (ushort)(ushort)55819;
            p132.min_distance = (ushort)(ushort)57473;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -156710594);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)18199);
                Debug.Assert(pack.mask == (ulong)6176325556374669875L);
                Debug.Assert(pack.lon == (int)1992474227);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -156710594;
            p133.grid_spacing = (ushort)(ushort)18199;
            p133.lon = (int)1992474227;
            p133.mask = (ulong)6176325556374669875L;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1309839414);
                Debug.Assert(pack.gridbit == (byte)(byte)165);
                Debug.Assert(pack.lat == (int) -1529219908);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)2457);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)15894, (short)1640, (short)30282, (short)6245, (short)24592, (short)9199, (short) -18239, (short)2947, (short)20609, (short)23689, (short)19187, (short)24725, (short)23652, (short) -32373, (short)22015, (short) -28908}));
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.gridbit = (byte)(byte)165;
            p134.lon = (int) -1309839414;
            p134.grid_spacing = (ushort)(ushort)2457;
            p134.data__SET(new short[] {(short)15894, (short)1640, (short)30282, (short)6245, (short)24592, (short)9199, (short) -18239, (short)2947, (short)20609, (short)23689, (short)19187, (short)24725, (short)23652, (short) -32373, (short)22015, (short) -28908}, 0) ;
            p134.lat = (int) -1529219908;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1954055878);
                Debug.Assert(pack.lon == (int)140202663);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)140202663;
            p135.lat = (int)1954055878;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)59474);
                Debug.Assert(pack.loaded == (ushort)(ushort)55052);
                Debug.Assert(pack.lat == (int) -1767112424);
                Debug.Assert(pack.terrain_height == (float) -2.724435E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)64866);
                Debug.Assert(pack.current_height == (float)8.3392106E37F);
                Debug.Assert(pack.lon == (int)1843136558);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)8.3392106E37F;
            p136.lon = (int)1843136558;
            p136.terrain_height = (float) -2.724435E38F;
            p136.lat = (int) -1767112424;
            p136.loaded = (ushort)(ushort)55052;
            p136.spacing = (ushort)(ushort)59474;
            p136.pending = (ushort)(ushort)64866;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2743359132U);
                Debug.Assert(pack.press_diff == (float) -5.692863E37F);
                Debug.Assert(pack.temperature == (short)(short)17676);
                Debug.Assert(pack.press_abs == (float) -1.7020572E38F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float) -5.692863E37F;
            p137.temperature = (short)(short)17676;
            p137.time_boot_ms = (uint)2743359132U;
            p137.press_abs = (float) -1.7020572E38F;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7490396082781515586L);
                Debug.Assert(pack.x == (float) -2.624407E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.8596643E38F, -1.5346006E38F, 3.0816016E38F, -1.2062079E38F}));
                Debug.Assert(pack.z == (float) -3.0909333E38F);
                Debug.Assert(pack.y == (float)3.0816414E38F);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float) -3.0909333E38F;
            p138.time_usec = (ulong)7490396082781515586L;
            p138.y = (float)3.0816414E38F;
            p138.x = (float) -2.624407E38F;
            p138.q_SET(new float[] {-1.8596643E38F, -1.5346006E38F, 3.0816016E38F, -1.2062079E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.group_mlx == (byte)(byte)161);
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.time_usec == (ulong)2147468012315987789L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {5.1096754E37F, -2.0983915E38F, 3.192459E38F, -2.0600036E38F, -2.2648495E38F, 1.3932816E38F, -1.2878657E38F, 2.0469134E38F}));
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)3;
            p139.group_mlx = (byte)(byte)161;
            p139.controls_SET(new float[] {5.1096754E37F, -2.0983915E38F, 3.192459E38F, -2.0600036E38F, -2.2648495E38F, 1.3932816E38F, -1.2878657E38F, 2.0469134E38F}, 0) ;
            p139.time_usec = (ulong)2147468012315987789L;
            p139.target_system = (byte)(byte)156;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6272559389388051321L);
                Debug.Assert(pack.group_mlx == (byte)(byte)143);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.097607E38F, 1.1065951E38F, 3.1774483E36F, -1.8046358E37F, 1.6235592E38F, 3.0224685E37F, -3.1485796E38F, -4.939434E37F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6272559389388051321L;
            p140.controls_SET(new float[] {-3.097607E38F, 1.1065951E38F, 3.1774483E36F, -1.8046358E37F, 1.6235592E38F, 3.0224685E37F, -3.1485796E38F, -4.939434E37F}, 0) ;
            p140.group_mlx = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float) -2.3779018E38F);
                Debug.Assert(pack.altitude_local == (float) -2.9731972E38F);
                Debug.Assert(pack.altitude_amsl == (float) -8.1111734E37F);
                Debug.Assert(pack.altitude_terrain == (float) -2.5690233E38F);
                Debug.Assert(pack.time_usec == (ulong)2698096942190961897L);
                Debug.Assert(pack.altitude_monotonic == (float) -7.1477306E37F);
                Debug.Assert(pack.bottom_clearance == (float)2.5031483E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -2.9731972E38F;
            p141.altitude_monotonic = (float) -7.1477306E37F;
            p141.altitude_terrain = (float) -2.5690233E38F;
            p141.altitude_relative = (float) -2.3779018E38F;
            p141.time_usec = (ulong)2698096942190961897L;
            p141.bottom_clearance = (float)2.5031483E38F;
            p141.altitude_amsl = (float) -8.1111734E37F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)203);
                Debug.Assert(pack.request_id == (byte)(byte)15);
                Debug.Assert(pack.transfer_type == (byte)(byte)160);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)167, (byte)240, (byte)124, (byte)82, (byte)35, (byte)196, (byte)176, (byte)80, (byte)194, (byte)111, (byte)19, (byte)105, (byte)68, (byte)78, (byte)114, (byte)42, (byte)105, (byte)37, (byte)123, (byte)23, (byte)106, (byte)189, (byte)27, (byte)235, (byte)81, (byte)199, (byte)145, (byte)161, (byte)255, (byte)2, (byte)167, (byte)91, (byte)45, (byte)38, (byte)213, (byte)199, (byte)213, (byte)61, (byte)165, (byte)247, (byte)79, (byte)239, (byte)47, (byte)121, (byte)252, (byte)216, (byte)228, (byte)124, (byte)175, (byte)244, (byte)74, (byte)190, (byte)237, (byte)104, (byte)183, (byte)13, (byte)113, (byte)187, (byte)27, (byte)159, (byte)250, (byte)95, (byte)223, (byte)159, (byte)131, (byte)209, (byte)43, (byte)77, (byte)185, (byte)83, (byte)4, (byte)186, (byte)131, (byte)90, (byte)89, (byte)232, (byte)85, (byte)69, (byte)86, (byte)219, (byte)116, (byte)246, (byte)156, (byte)185, (byte)169, (byte)166, (byte)96, (byte)175, (byte)119, (byte)72, (byte)133, (byte)239, (byte)207, (byte)87, (byte)84, (byte)47, (byte)40, (byte)15, (byte)41, (byte)70, (byte)234, (byte)114, (byte)102, (byte)65, (byte)31, (byte)82, (byte)116, (byte)227, (byte)97, (byte)131, (byte)151, (byte)28, (byte)105, (byte)34, (byte)163, (byte)57, (byte)188, (byte)26, (byte)59, (byte)81}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)225, (byte)94, (byte)91, (byte)55, (byte)87, (byte)232, (byte)29, (byte)172, (byte)174, (byte)135, (byte)62, (byte)4, (byte)6, (byte)96, (byte)132, (byte)47, (byte)4, (byte)98, (byte)205, (byte)66, (byte)33, (byte)48, (byte)226, (byte)62, (byte)72, (byte)144, (byte)39, (byte)193, (byte)41, (byte)14, (byte)60, (byte)138, (byte)72, (byte)212, (byte)31, (byte)199, (byte)121, (byte)187, (byte)31, (byte)105, (byte)5, (byte)179, (byte)161, (byte)133, (byte)13, (byte)7, (byte)129, (byte)82, (byte)134, (byte)172, (byte)21, (byte)192, (byte)38, (byte)216, (byte)193, (byte)225, (byte)123, (byte)91, (byte)204, (byte)50, (byte)51, (byte)87, (byte)157, (byte)188, (byte)57, (byte)9, (byte)208, (byte)22, (byte)162, (byte)251, (byte)230, (byte)59, (byte)229, (byte)72, (byte)240, (byte)235, (byte)155, (byte)141, (byte)215, (byte)244, (byte)13, (byte)106, (byte)24, (byte)108, (byte)68, (byte)212, (byte)44, (byte)228, (byte)145, (byte)157, (byte)218, (byte)121, (byte)208, (byte)217, (byte)40, (byte)3, (byte)129, (byte)4, (byte)252, (byte)224, (byte)247, (byte)237, (byte)8, (byte)49, (byte)33, (byte)85, (byte)214, (byte)120, (byte)182, (byte)153, (byte)140, (byte)65, (byte)245, (byte)157, (byte)7, (byte)101, (byte)37, (byte)34, (byte)26, (byte)232}));
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)203;
            p142.storage_SET(new byte[] {(byte)225, (byte)94, (byte)91, (byte)55, (byte)87, (byte)232, (byte)29, (byte)172, (byte)174, (byte)135, (byte)62, (byte)4, (byte)6, (byte)96, (byte)132, (byte)47, (byte)4, (byte)98, (byte)205, (byte)66, (byte)33, (byte)48, (byte)226, (byte)62, (byte)72, (byte)144, (byte)39, (byte)193, (byte)41, (byte)14, (byte)60, (byte)138, (byte)72, (byte)212, (byte)31, (byte)199, (byte)121, (byte)187, (byte)31, (byte)105, (byte)5, (byte)179, (byte)161, (byte)133, (byte)13, (byte)7, (byte)129, (byte)82, (byte)134, (byte)172, (byte)21, (byte)192, (byte)38, (byte)216, (byte)193, (byte)225, (byte)123, (byte)91, (byte)204, (byte)50, (byte)51, (byte)87, (byte)157, (byte)188, (byte)57, (byte)9, (byte)208, (byte)22, (byte)162, (byte)251, (byte)230, (byte)59, (byte)229, (byte)72, (byte)240, (byte)235, (byte)155, (byte)141, (byte)215, (byte)244, (byte)13, (byte)106, (byte)24, (byte)108, (byte)68, (byte)212, (byte)44, (byte)228, (byte)145, (byte)157, (byte)218, (byte)121, (byte)208, (byte)217, (byte)40, (byte)3, (byte)129, (byte)4, (byte)252, (byte)224, (byte)247, (byte)237, (byte)8, (byte)49, (byte)33, (byte)85, (byte)214, (byte)120, (byte)182, (byte)153, (byte)140, (byte)65, (byte)245, (byte)157, (byte)7, (byte)101, (byte)37, (byte)34, (byte)26, (byte)232}, 0) ;
            p142.transfer_type = (byte)(byte)160;
            p142.uri_SET(new byte[] {(byte)167, (byte)240, (byte)124, (byte)82, (byte)35, (byte)196, (byte)176, (byte)80, (byte)194, (byte)111, (byte)19, (byte)105, (byte)68, (byte)78, (byte)114, (byte)42, (byte)105, (byte)37, (byte)123, (byte)23, (byte)106, (byte)189, (byte)27, (byte)235, (byte)81, (byte)199, (byte)145, (byte)161, (byte)255, (byte)2, (byte)167, (byte)91, (byte)45, (byte)38, (byte)213, (byte)199, (byte)213, (byte)61, (byte)165, (byte)247, (byte)79, (byte)239, (byte)47, (byte)121, (byte)252, (byte)216, (byte)228, (byte)124, (byte)175, (byte)244, (byte)74, (byte)190, (byte)237, (byte)104, (byte)183, (byte)13, (byte)113, (byte)187, (byte)27, (byte)159, (byte)250, (byte)95, (byte)223, (byte)159, (byte)131, (byte)209, (byte)43, (byte)77, (byte)185, (byte)83, (byte)4, (byte)186, (byte)131, (byte)90, (byte)89, (byte)232, (byte)85, (byte)69, (byte)86, (byte)219, (byte)116, (byte)246, (byte)156, (byte)185, (byte)169, (byte)166, (byte)96, (byte)175, (byte)119, (byte)72, (byte)133, (byte)239, (byte)207, (byte)87, (byte)84, (byte)47, (byte)40, (byte)15, (byte)41, (byte)70, (byte)234, (byte)114, (byte)102, (byte)65, (byte)31, (byte)82, (byte)116, (byte)227, (byte)97, (byte)131, (byte)151, (byte)28, (byte)105, (byte)34, (byte)163, (byte)57, (byte)188, (byte)26, (byte)59, (byte)81}, 0) ;
            p142.request_id = (byte)(byte)15;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -3.468173E37F);
                Debug.Assert(pack.temperature == (short)(short) -3471);
                Debug.Assert(pack.press_abs == (float)2.0911775E38F);
                Debug.Assert(pack.time_boot_ms == (uint)10276760U);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -3471;
            p143.time_boot_ms = (uint)10276760U;
            p143.press_diff = (float) -3.468173E37F;
            p143.press_abs = (float)2.0911775E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.82313E38F, 3.0444469E38F, -1.1391881E38F}));
                Debug.Assert(pack.lon == (int)1412761735);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.1686887E38F, 2.9328336E37F, 2.2564672E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-6.1459834E37F, 3.130916E38F, 3.2579856E38F, -2.0619558E38F}));
                Debug.Assert(pack.alt == (float)2.999085E38F);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.3754132E38F, -3.1745758E38F, -1.6978005E38F}));
                Debug.Assert(pack.custom_state == (ulong)4112743406348677684L);
                Debug.Assert(pack.est_capabilities == (byte)(byte)107);
                Debug.Assert(pack.timestamp == (ulong)7025711539600585811L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.392674E38F, -2.7555572E38F, 3.699791E37F}));
                Debug.Assert(pack.lat == (int)318181324);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float)2.999085E38F;
            p144.position_cov_SET(new float[] {3.3754132E38F, -3.1745758E38F, -1.6978005E38F}, 0) ;
            p144.vel_SET(new float[] {2.1686887E38F, 2.9328336E37F, 2.2564672E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-6.1459834E37F, 3.130916E38F, 3.2579856E38F, -2.0619558E38F}, 0) ;
            p144.acc_SET(new float[] {1.82313E38F, 3.0444469E38F, -1.1391881E38F}, 0) ;
            p144.rates_SET(new float[] {-2.392674E38F, -2.7555572E38F, 3.699791E37F}, 0) ;
            p144.lon = (int)1412761735;
            p144.timestamp = (ulong)7025711539600585811L;
            p144.est_capabilities = (byte)(byte)107;
            p144.lat = (int)318181324;
            p144.custom_state = (ulong)4112743406348677684L;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_vel == (float)3.125825E38F);
                Debug.Assert(pack.z_pos == (float) -1.3510117E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.7748082E38F, -1.5366639E38F, -1.2436629E38F}));
                Debug.Assert(pack.time_usec == (ulong)3317322534735669470L);
                Debug.Assert(pack.z_acc == (float)2.4752561E38F);
                Debug.Assert(pack.x_acc == (float)3.3842553E38F);
                Debug.Assert(pack.y_vel == (float)2.1543193E38F);
                Debug.Assert(pack.roll_rate == (float) -2.9823352E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.7617974E38F, -2.6975887E38F, 1.1122698E38F}));
                Debug.Assert(pack.z_vel == (float) -2.2884896E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.6569E38F);
                Debug.Assert(pack.x_pos == (float)5.542091E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.8409428E37F, 2.130821E38F, -2.5679534E38F, 2.2663307E38F}));
                Debug.Assert(pack.y_pos == (float)2.2121897E37F);
                Debug.Assert(pack.y_acc == (float)2.3162076E38F);
                Debug.Assert(pack.pitch_rate == (float) -2.3947492E38F);
                Debug.Assert(pack.airspeed == (float)2.7488932E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.z_vel = (float) -2.2884896E38F;
            p146.y_pos = (float)2.2121897E37F;
            p146.pitch_rate = (float) -2.3947492E38F;
            p146.roll_rate = (float) -2.9823352E38F;
            p146.time_usec = (ulong)3317322534735669470L;
            p146.z_pos = (float) -1.3510117E38F;
            p146.yaw_rate = (float) -2.6569E38F;
            p146.x_pos = (float)5.542091E37F;
            p146.pos_variance_SET(new float[] {1.7617974E38F, -2.6975887E38F, 1.1122698E38F}, 0) ;
            p146.airspeed = (float)2.7488932E38F;
            p146.z_acc = (float)2.4752561E38F;
            p146.x_vel = (float)3.125825E38F;
            p146.q_SET(new float[] {1.8409428E37F, 2.130821E38F, -2.5679534E38F, 2.2663307E38F}, 0) ;
            p146.y_vel = (float)2.1543193E38F;
            p146.y_acc = (float)2.3162076E38F;
            p146.x_acc = (float)3.3842553E38F;
            p146.vel_variance_SET(new float[] {1.7748082E38F, -1.5366639E38F, -1.2436629E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.current_consumed == (int)256104380);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)36);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.temperature == (short)(short) -13707);
                Debug.Assert(pack.energy_consumed == (int) -1957585288);
                Debug.Assert(pack.current_battery == (short)(short)22857);
                Debug.Assert(pack.id == (byte)(byte)230);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)28853, (ushort)3071, (ushort)17210, (ushort)9492, (ushort)29480, (ushort)49560, (ushort)51156, (ushort)40491, (ushort)41609, (ushort)44513}));
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int)256104380;
            p147.temperature = (short)(short) -13707;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.voltages_SET(new ushort[] {(ushort)28853, (ushort)3071, (ushort)17210, (ushort)9492, (ushort)29480, (ushort)49560, (ushort)51156, (ushort)40491, (ushort)41609, (ushort)44513}, 0) ;
            p147.current_battery = (short)(short)22857;
            p147.energy_consumed = (int) -1957585288;
            p147.battery_remaining = (sbyte)(sbyte)36;
            p147.id = (byte)(byte)230;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_id == (ushort)(ushort)55968);
                Debug.Assert(pack.middleware_sw_version == (uint)3694530920U);
                Debug.Assert(pack.product_id == (ushort)(ushort)33021);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)223, (byte)107, (byte)252, (byte)41, (byte)156, (byte)225, (byte)97, (byte)40}));
                Debug.Assert(pack.flight_sw_version == (uint)3398246586U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)136, (byte)47, (byte)177, (byte)14, (byte)189, (byte)209, (byte)81, (byte)45}));
                Debug.Assert(pack.os_sw_version == (uint)3034523854U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)18, (byte)4, (byte)248, (byte)236, (byte)94, (byte)113, (byte)188, (byte)18}));
                Debug.Assert(pack.uid == (ulong)2476228925804198516L);
                Debug.Assert(pack.board_version == (uint)1789655634U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)248, (byte)133, (byte)228, (byte)210, (byte)111, (byte)42, (byte)181, (byte)204, (byte)60, (byte)150, (byte)91, (byte)13, (byte)138, (byte)63, (byte)91, (byte)75, (byte)91, (byte)161}));
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)3034523854U;
            p148.middleware_custom_version_SET(new byte[] {(byte)18, (byte)4, (byte)248, (byte)236, (byte)94, (byte)113, (byte)188, (byte)18}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)136, (byte)47, (byte)177, (byte)14, (byte)189, (byte)209, (byte)81, (byte)45}, 0) ;
            p148.product_id = (ushort)(ushort)33021;
            p148.uid = (ulong)2476228925804198516L;
            p148.uid2_SET(new byte[] {(byte)248, (byte)133, (byte)228, (byte)210, (byte)111, (byte)42, (byte)181, (byte)204, (byte)60, (byte)150, (byte)91, (byte)13, (byte)138, (byte)63, (byte)91, (byte)75, (byte)91, (byte)161}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)223, (byte)107, (byte)252, (byte)41, (byte)156, (byte)225, (byte)97, (byte)40}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
            p148.board_version = (uint)1789655634U;
            p148.middleware_sw_version = (uint)3694530920U;
            p148.flight_sw_version = (uint)3398246586U;
            p148.vendor_id = (ushort)(ushort)55968;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)189);
                Debug.Assert(pack.angle_y == (float) -1.5892671E38F);
                Debug.Assert(pack.distance == (float) -2.4788279E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.6589629E38F);
                Debug.Assert(pack.size_x == (float)2.787654E38F);
                Debug.Assert(pack.time_usec == (ulong)1008961073015020704L);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-1.8154111E38F, 4.722749E37F, -1.7092208E38F, 2.3452698E38F}));
                Debug.Assert(pack.size_y == (float)1.8008893E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.angle_x == (float) -1.6658978E38F);
                Debug.Assert(pack.target_num == (byte)(byte)11);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.1791652E37F);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.549462E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p149.target_num = (byte)(byte)11;
            p149.angle_x = (float) -1.6658978E38F;
            p149.z_SET((float) -1.6589629E38F, PH) ;
            p149.size_x = (float)2.787654E38F;
            p149.size_y = (float)1.8008893E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.distance = (float) -2.4788279E38F;
            p149.y_SET((float) -1.549462E38F, PH) ;
            p149.x_SET((float) -2.1791652E37F, PH) ;
            p149.position_valid_SET((byte)(byte)189, PH) ;
            p149.q_SET(new float[] {-1.8154111E38F, 4.722749E37F, -1.7092208E38F, 2.3452698E38F}, 0, PH) ;
            p149.angle_y = (float) -1.5892671E38F;
            p149.time_usec = (ulong)1008961073015020704L;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)175);
                Debug.Assert(pack.target_system == (byte)(byte)91);
            };
            DemoDevice.FLEXIFUNCTION_SET p150 = LoopBackDemoChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_component = (byte)(byte)175;
            p150.target_system = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.read_req_type == (short)(short)2569);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.data_index == (short)(short) -32046);
            };
            DemoDevice.FLEXIFUNCTION_READ_REQ p151 = LoopBackDemoChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)92;
            p151.read_req_type = (short)(short)2569;
            p151.data_index = (short)(short) -32046;
            p151.target_component = (byte)(byte)219;
            LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_address == (ushort)(ushort)19707);
                Debug.Assert(pack.func_count == (ushort)(ushort)44019);
                Debug.Assert(pack.data_size == (ushort)(ushort)49265);
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.func_index == (ushort)(ushort)5106);
                Debug.Assert(pack.data_.SequenceEqual(new sbyte[] {(sbyte) - 64, (sbyte)22, (sbyte)12, (sbyte)22, (sbyte) - 56, (sbyte) - 67, (sbyte)32, (sbyte)41, (sbyte) - 36, (sbyte) - 67, (sbyte)3, (sbyte)124, (sbyte) - 23, (sbyte) - 75, (sbyte) - 105, (sbyte)6, (sbyte) - 27, (sbyte) - 19, (sbyte) - 113, (sbyte)124, (sbyte) - 92, (sbyte)69, (sbyte)94, (sbyte) - 7, (sbyte) - 14, (sbyte) - 90, (sbyte)62, (sbyte) - 22, (sbyte) - 41, (sbyte)44, (sbyte) - 102, (sbyte) - 121, (sbyte) - 54, (sbyte)41, (sbyte) - 85, (sbyte)99, (sbyte)39, (sbyte)34, (sbyte) - 7, (sbyte) - 49, (sbyte) - 57, (sbyte) - 96, (sbyte) - 91, (sbyte) - 62, (sbyte)22, (sbyte) - 60, (sbyte) - 127, (sbyte)30}));
                Debug.Assert(pack.target_system == (byte)(byte)135);
            };
            DemoDevice.FLEXIFUNCTION_BUFFER_FUNCTION p152 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.func_count = (ushort)(ushort)44019;
            p152.func_index = (ushort)(ushort)5106;
            p152.data_size = (ushort)(ushort)49265;
            p152.data_address = (ushort)(ushort)19707;
            p152.target_component = (byte)(byte)174;
            p152.target_system = (byte)(byte)135;
            p152.data__SET(new sbyte[] {(sbyte) - 64, (sbyte)22, (sbyte)12, (sbyte)22, (sbyte) - 56, (sbyte) - 67, (sbyte)32, (sbyte)41, (sbyte) - 36, (sbyte) - 67, (sbyte)3, (sbyte)124, (sbyte) - 23, (sbyte) - 75, (sbyte) - 105, (sbyte)6, (sbyte) - 27, (sbyte) - 19, (sbyte) - 113, (sbyte)124, (sbyte) - 92, (sbyte)69, (sbyte)94, (sbyte) - 7, (sbyte) - 14, (sbyte) - 90, (sbyte)62, (sbyte) - 22, (sbyte) - 41, (sbyte)44, (sbyte) - 102, (sbyte) - 121, (sbyte) - 54, (sbyte)41, (sbyte) - 85, (sbyte)99, (sbyte)39, (sbyte)34, (sbyte) - 7, (sbyte) - 49, (sbyte) - 57, (sbyte) - 96, (sbyte) - 91, (sbyte) - 62, (sbyte)22, (sbyte) - 60, (sbyte) - 127, (sbyte)30}, 0) ;
            LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.result == (ushort)(ushort)35050);
                Debug.Assert(pack.func_index == (ushort)(ushort)32504);
                Debug.Assert(pack.target_component == (byte)(byte)149);
            };
            DemoDevice.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.func_index = (ushort)(ushort)32504;
            p153.result = (ushort)(ushort)35050;
            p153.target_component = (byte)(byte)149;
            p153.target_system = (byte)(byte)137;
            LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.target_component == (byte)(byte)125);
                Debug.Assert(pack.count == (byte)(byte)73);
                Debug.Assert(pack.directory_type == (byte)(byte)19);
                Debug.Assert(pack.start_index == (byte)(byte)193);
                Debug.Assert(pack.directory_data.SequenceEqual(new sbyte[] {(sbyte) - 57, (sbyte)7, (sbyte) - 47, (sbyte)70, (sbyte)47, (sbyte)25, (sbyte)21, (sbyte) - 113, (sbyte) - 49, (sbyte)77, (sbyte)95, (sbyte)63, (sbyte) - 119, (sbyte)108, (sbyte) - 5, (sbyte)3, (sbyte)81, (sbyte) - 119, (sbyte)20, (sbyte) - 33, (sbyte) - 118, (sbyte) - 32, (sbyte) - 105, (sbyte)125, (sbyte)24, (sbyte)106, (sbyte) - 122, (sbyte) - 54, (sbyte) - 39, (sbyte)123, (sbyte) - 46, (sbyte)106, (sbyte)11, (sbyte)29, (sbyte) - 68, (sbyte)21, (sbyte)38, (sbyte) - 103, (sbyte)55, (sbyte) - 106, (sbyte)5, (sbyte)105, (sbyte)14, (sbyte)73, (sbyte)62, (sbyte)91, (sbyte)17, (sbyte) - 86}));
            };
            DemoDevice.FLEXIFUNCTION_DIRECTORY p155 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.start_index = (byte)(byte)193;
            p155.directory_data_SET(new sbyte[] {(sbyte) - 57, (sbyte)7, (sbyte) - 47, (sbyte)70, (sbyte)47, (sbyte)25, (sbyte)21, (sbyte) - 113, (sbyte) - 49, (sbyte)77, (sbyte)95, (sbyte)63, (sbyte) - 119, (sbyte)108, (sbyte) - 5, (sbyte)3, (sbyte)81, (sbyte) - 119, (sbyte)20, (sbyte) - 33, (sbyte) - 118, (sbyte) - 32, (sbyte) - 105, (sbyte)125, (sbyte)24, (sbyte)106, (sbyte) - 122, (sbyte) - 54, (sbyte) - 39, (sbyte)123, (sbyte) - 46, (sbyte)106, (sbyte)11, (sbyte)29, (sbyte) - 68, (sbyte)21, (sbyte)38, (sbyte) - 103, (sbyte)55, (sbyte) - 106, (sbyte)5, (sbyte)105, (sbyte)14, (sbyte)73, (sbyte)62, (sbyte)91, (sbyte)17, (sbyte) - 86}, 0) ;
            p155.count = (byte)(byte)73;
            p155.target_component = (byte)(byte)125;
            p155.target_system = (byte)(byte)225;
            p155.directory_type = (byte)(byte)19;
            LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)25);
                Debug.Assert(pack.start_index == (byte)(byte)45);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.directory_type == (byte)(byte)143);
                Debug.Assert(pack.result == (ushort)(ushort)50361);
                Debug.Assert(pack.target_component == (byte)(byte)123);
            };
            DemoDevice.FLEXIFUNCTION_DIRECTORY_ACK p156 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.start_index = (byte)(byte)45;
            p156.target_system = (byte)(byte)203;
            p156.result = (ushort)(ushort)50361;
            p156.count = (byte)(byte)25;
            p156.directory_type = (byte)(byte)143;
            p156.target_component = (byte)(byte)123;
            LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_type == (byte)(byte)145);
                Debug.Assert(pack.target_component == (byte)(byte)35);
                Debug.Assert(pack.target_system == (byte)(byte)219);
            };
            DemoDevice.FLEXIFUNCTION_COMMAND p157 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_component = (byte)(byte)35;
            p157.target_system = (byte)(byte)219;
            p157.command_type = (byte)(byte)145;
            LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (ushort)(ushort)11862);
                Debug.Assert(pack.command_type == (ushort)(ushort)58180);
            };
            DemoDevice.FLEXIFUNCTION_COMMAND_ACK p158 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)58180;
            p158.result = (ushort)(ushort)11862;
            LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_magFieldEarth0 == (short)(short) -3208);
                Debug.Assert(pack.sue_svs == (short)(short)7868);
                Debug.Assert(pack.sue_longitude == (int) -1194659223);
                Debug.Assert(pack.sue_magFieldEarth1 == (short)(short)4586);
                Debug.Assert(pack.sue_estimated_wind_0 == (short)(short) -31901);
                Debug.Assert(pack.sue_rmat3 == (short)(short) -1431);
                Debug.Assert(pack.sue_status == (byte)(byte)51);
                Debug.Assert(pack.sue_sog == (short)(short) -16293);
                Debug.Assert(pack.sue_estimated_wind_1 == (short)(short) -25531);
                Debug.Assert(pack.sue_rmat1 == (short)(short)5107);
                Debug.Assert(pack.sue_air_speed_3DIMU == (ushort)(ushort)54202);
                Debug.Assert(pack.sue_rmat5 == (short)(short)22158);
                Debug.Assert(pack.sue_waypoint_index == (ushort)(ushort)58192);
                Debug.Assert(pack.sue_rmat4 == (short)(short)22765);
                Debug.Assert(pack.sue_rmat8 == (short)(short)17002);
                Debug.Assert(pack.sue_estimated_wind_2 == (short)(short)7995);
                Debug.Assert(pack.sue_rmat6 == (short)(short) -2943);
                Debug.Assert(pack.sue_cpu_load == (ushort)(ushort)51043);
                Debug.Assert(pack.sue_cog == (ushort)(ushort)8644);
                Debug.Assert(pack.sue_rmat0 == (short)(short) -675);
                Debug.Assert(pack.sue_rmat7 == (short)(short)24937);
                Debug.Assert(pack.sue_rmat2 == (short)(short) -28881);
                Debug.Assert(pack.sue_hdop == (short)(short) -13368);
                Debug.Assert(pack.sue_altitude == (int) -1857394016);
                Debug.Assert(pack.sue_time == (uint)2524067994U);
                Debug.Assert(pack.sue_latitude == (int) -1318192917);
                Debug.Assert(pack.sue_magFieldEarth2 == (short)(short)24375);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F2_A p170 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_rmat0 = (short)(short) -675;
            p170.sue_rmat4 = (short)(short)22765;
            p170.sue_time = (uint)2524067994U;
            p170.sue_sog = (short)(short) -16293;
            p170.sue_rmat7 = (short)(short)24937;
            p170.sue_svs = (short)(short)7868;
            p170.sue_estimated_wind_1 = (short)(short) -25531;
            p170.sue_rmat1 = (short)(short)5107;
            p170.sue_longitude = (int) -1194659223;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)54202;
            p170.sue_rmat8 = (short)(short)17002;
            p170.sue_estimated_wind_2 = (short)(short)7995;
            p170.sue_hdop = (short)(short) -13368;
            p170.sue_rmat6 = (short)(short) -2943;
            p170.sue_waypoint_index = (ushort)(ushort)58192;
            p170.sue_latitude = (int) -1318192917;
            p170.sue_cpu_load = (ushort)(ushort)51043;
            p170.sue_rmat2 = (short)(short) -28881;
            p170.sue_cog = (ushort)(ushort)8644;
            p170.sue_magFieldEarth0 = (short)(short) -3208;
            p170.sue_estimated_wind_0 = (short)(short) -31901;
            p170.sue_rmat3 = (short)(short) -1431;
            p170.sue_magFieldEarth1 = (short)(short)4586;
            p170.sue_rmat5 = (short)(short)22158;
            p170.sue_magFieldEarth2 = (short)(short)24375;
            p170.sue_altitude = (int) -1857394016;
            p170.sue_status = (byte)(byte)51;
            LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_pwm_output_3 == (short)(short) -16749);
                Debug.Assert(pack.sue_pwm_input_7 == (short)(short)21844);
                Debug.Assert(pack.sue_waypoint_goal_z == (short)(short)18455);
                Debug.Assert(pack.sue_location_error_earth_z == (short)(short) -30747);
                Debug.Assert(pack.sue_imu_location_x == (short)(short)25791);
                Debug.Assert(pack.sue_imu_velocity_x == (short)(short)23762);
                Debug.Assert(pack.sue_waypoint_goal_y == (short)(short)19043);
                Debug.Assert(pack.sue_pwm_input_1 == (short)(short) -32653);
                Debug.Assert(pack.sue_pwm_output_2 == (short)(short)20655);
                Debug.Assert(pack.sue_imu_velocity_y == (short)(short) -19511);
                Debug.Assert(pack.sue_pwm_output_6 == (short)(short)18867);
                Debug.Assert(pack.sue_pwm_input_5 == (short)(short)10389);
                Debug.Assert(pack.sue_pwm_output_8 == (short)(short) -24819);
                Debug.Assert(pack.sue_aero_x == (short)(short) -14533);
                Debug.Assert(pack.sue_pwm_input_6 == (short)(short) -7036);
                Debug.Assert(pack.sue_pwm_output_4 == (short)(short)6688);
                Debug.Assert(pack.sue_barom_temp == (short)(short)19243);
                Debug.Assert(pack.sue_pwm_output_9 == (short)(short) -11673);
                Debug.Assert(pack.sue_aero_y == (short)(short) -27923);
                Debug.Assert(pack.sue_flags == (uint)2644870123U);
                Debug.Assert(pack.sue_pwm_input_2 == (short)(short)11225);
                Debug.Assert(pack.sue_pwm_output_1 == (short)(short) -29042);
                Debug.Assert(pack.sue_bat_amp_hours == (short)(short) -14304);
                Debug.Assert(pack.sue_osc_fails == (short)(short)19965);
                Debug.Assert(pack.sue_pwm_input_4 == (short)(short) -5021);
                Debug.Assert(pack.sue_pwm_output_12 == (short)(short)23518);
                Debug.Assert(pack.sue_pwm_input_9 == (short)(short)16625);
                Debug.Assert(pack.sue_pwm_output_7 == (short)(short)6685);
                Debug.Assert(pack.sue_location_error_earth_y == (short)(short) -29589);
                Debug.Assert(pack.sue_desired_height == (short)(short)18657);
                Debug.Assert(pack.sue_pwm_input_3 == (short)(short) -20002);
                Debug.Assert(pack.sue_pwm_output_10 == (short)(short)11933);
                Debug.Assert(pack.sue_bat_volt == (short)(short) -16149);
                Debug.Assert(pack.sue_barom_alt == (int) -1414779755);
                Debug.Assert(pack.sue_imu_location_y == (short)(short)15023);
                Debug.Assert(pack.sue_pwm_input_12 == (short)(short)25437);
                Debug.Assert(pack.sue_pwm_input_8 == (short)(short)13693);
                Debug.Assert(pack.sue_pwm_input_11 == (short)(short) -16770);
                Debug.Assert(pack.sue_pwm_output_11 == (short)(short)30432);
                Debug.Assert(pack.sue_pwm_input_10 == (short)(short) -20518);
                Debug.Assert(pack.sue_aero_z == (short)(short)25961);
                Debug.Assert(pack.sue_barom_press == (int) -1823563594);
                Debug.Assert(pack.sue_bat_amp == (short)(short) -9781);
                Debug.Assert(pack.sue_imu_velocity_z == (short)(short)26204);
                Debug.Assert(pack.sue_time == (uint)3665500521U);
                Debug.Assert(pack.sue_imu_location_z == (short)(short) -1781);
                Debug.Assert(pack.sue_pwm_output_5 == (short)(short)6320);
                Debug.Assert(pack.sue_waypoint_goal_x == (short)(short)28343);
                Debug.Assert(pack.sue_location_error_earth_x == (short)(short) -4658);
                Debug.Assert(pack.sue_memory_stack_free == (short)(short)11369);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F2_B p171 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_imu_location_y = (short)(short)15023;
            p171.sue_pwm_input_7 = (short)(short)21844;
            p171.sue_pwm_output_7 = (short)(short)6685;
            p171.sue_pwm_input_9 = (short)(short)16625;
            p171.sue_aero_z = (short)(short)25961;
            p171.sue_pwm_input_11 = (short)(short) -16770;
            p171.sue_pwm_input_8 = (short)(short)13693;
            p171.sue_waypoint_goal_z = (short)(short)18455;
            p171.sue_aero_y = (short)(short) -27923;
            p171.sue_bat_amp_hours = (short)(short) -14304;
            p171.sue_pwm_input_10 = (short)(short) -20518;
            p171.sue_aero_x = (short)(short) -14533;
            p171.sue_flags = (uint)2644870123U;
            p171.sue_waypoint_goal_x = (short)(short)28343;
            p171.sue_pwm_input_6 = (short)(short) -7036;
            p171.sue_pwm_input_4 = (short)(short) -5021;
            p171.sue_pwm_input_12 = (short)(short)25437;
            p171.sue_pwm_output_10 = (short)(short)11933;
            p171.sue_memory_stack_free = (short)(short)11369;
            p171.sue_pwm_output_3 = (short)(short) -16749;
            p171.sue_time = (uint)3665500521U;
            p171.sue_pwm_output_11 = (short)(short)30432;
            p171.sue_imu_velocity_y = (short)(short) -19511;
            p171.sue_waypoint_goal_y = (short)(short)19043;
            p171.sue_barom_alt = (int) -1414779755;
            p171.sue_pwm_output_6 = (short)(short)18867;
            p171.sue_pwm_output_12 = (short)(short)23518;
            p171.sue_location_error_earth_z = (short)(short) -30747;
            p171.sue_pwm_input_1 = (short)(short) -32653;
            p171.sue_pwm_output_4 = (short)(short)6688;
            p171.sue_location_error_earth_x = (short)(short) -4658;
            p171.sue_desired_height = (short)(short)18657;
            p171.sue_pwm_output_2 = (short)(short)20655;
            p171.sue_bat_volt = (short)(short) -16149;
            p171.sue_pwm_input_2 = (short)(short)11225;
            p171.sue_barom_press = (int) -1823563594;
            p171.sue_pwm_output_5 = (short)(short)6320;
            p171.sue_pwm_output_1 = (short)(short) -29042;
            p171.sue_imu_velocity_z = (short)(short)26204;
            p171.sue_osc_fails = (short)(short)19965;
            p171.sue_pwm_input_3 = (short)(short) -20002;
            p171.sue_barom_temp = (short)(short)19243;
            p171.sue_location_error_earth_y = (short)(short) -29589;
            p171.sue_imu_velocity_x = (short)(short)23762;
            p171.sue_pwm_output_9 = (short)(short) -11673;
            p171.sue_pwm_output_8 = (short)(short) -24819;
            p171.sue_imu_location_z = (short)(short) -1781;
            p171.sue_imu_location_x = (short)(short)25791;
            p171.sue_bat_amp = (short)(short) -9781;
            p171.sue_pwm_input_5 = (short)(short)10389;
            LoopBackDemoChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLL_STABILIZATION_AILERONS == (byte)(byte)126);
                Debug.Assert(pack.sue_RUDDER_NAVIGATION == (byte)(byte)173);
                Debug.Assert(pack.sue_ALTITUDEHOLD_STABILIZED == (byte)(byte)234);
                Debug.Assert(pack.sue_RACING_MODE == (byte)(byte)178);
                Debug.Assert(pack.sue_YAW_STABILIZATION_RUDDER == (byte)(byte)6);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_RUDDER == (byte)(byte)18);
                Debug.Assert(pack.sue_PITCH_STABILIZATION == (byte)(byte)57);
                Debug.Assert(pack.sue_AILERON_NAVIGATION == (byte)(byte)112);
                Debug.Assert(pack.sue_ALTITUDEHOLD_WAYPOINT == (byte)(byte)23);
                Debug.Assert(pack.sue_YAW_STABILIZATION_AILERON == (byte)(byte)148);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F4 p172 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)148;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)23;
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)126;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)234;
            p172.sue_RACING_MODE = (byte)(byte)178;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)18;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)112;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)6;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)173;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)57;
            LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLLKP == (float) -1.9133551E38F);
                Debug.Assert(pack.sue_ROLLKD == (float)2.4965105E38F);
                Debug.Assert(pack.sue_YAWKD_AILERON == (float)1.5114145E37F);
                Debug.Assert(pack.sue_YAWKP_AILERON == (float) -1.1535424E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F5 p173 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_ROLLKP = (float) -1.9133551E38F;
            p173.sue_YAWKD_AILERON = (float)1.5114145E37F;
            p173.sue_YAWKP_AILERON = (float) -1.1535424E38F;
            p173.sue_ROLLKD = (float)2.4965105E38F;
            LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLL_ELEV_MIX == (float)3.118828E38F);
                Debug.Assert(pack.sue_RUDDER_ELEV_MIX == (float)3.2384061E38F);
                Debug.Assert(pack.sue_ELEVATOR_BOOST == (float) -4.853182E37F);
                Debug.Assert(pack.sue_PITCHKD == (float)1.3638936E38F);
                Debug.Assert(pack.sue_PITCHGAIN == (float) -2.3851276E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F6 p174 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHKD = (float)1.3638936E38F;
            p174.sue_PITCHGAIN = (float) -2.3851276E38F;
            p174.sue_ELEVATOR_BOOST = (float) -4.853182E37F;
            p174.sue_ROLL_ELEV_MIX = (float)3.118828E38F;
            p174.sue_RUDDER_ELEV_MIX = (float)3.2384061E38F;
            LoopBackDemoChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLLKP_RUDDER == (float)2.830135E38F);
                Debug.Assert(pack.sue_ROLLKD_RUDDER == (float)1.3482608E38F);
                Debug.Assert(pack.sue_YAWKD_RUDDER == (float)7.8030953E37F);
                Debug.Assert(pack.sue_RTL_PITCH_DOWN == (float) -2.9828105E38F);
                Debug.Assert(pack.sue_RUDDER_BOOST == (float)2.9678035E38F);
                Debug.Assert(pack.sue_YAWKP_RUDDER == (float)1.3187123E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F7 p175 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKP_RUDDER = (float)1.3187123E38F;
            p175.sue_YAWKD_RUDDER = (float)7.8030953E37F;
            p175.sue_RTL_PITCH_DOWN = (float) -2.9828105E38F;
            p175.sue_RUDDER_BOOST = (float)2.9678035E38F;
            p175.sue_ROLLKD_RUDDER = (float)1.3482608E38F;
            p175.sue_ROLLKP_RUDDER = (float)2.830135E38F;
            LoopBackDemoChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_HEIGHT_TARGET_MIN == (float) -7.1965807E37F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MIN == (float) -2.229434E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MAX == (float) -1.0953312E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MAX == (float)1.163899E38F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MAX == (float)2.6593369E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MIN == (float) -8.971438E37F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_HIGH == (float) -2.4012825E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F8 p176 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_ALT_HOLD_PITCH_MIN = (float) -2.229434E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -1.0953312E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float) -2.4012825E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float) -8.971438E37F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float)1.163899E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float) -7.1965807E37F;
            p176.sue_HEIGHT_TARGET_MAX = (float)2.6593369E38F;
            LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_lat_origin == (int)790066843);
                Debug.Assert(pack.sue_lon_origin == (int) -2091738185);
                Debug.Assert(pack.sue_alt_origin == (int)1599317661);
                Debug.Assert(pack.sue_week_no == (short)(short) -11219);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F13 p177 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_lat_origin = (int)790066843;
            p177.sue_week_no = (short)(short) -11219;
            p177.sue_alt_origin = (int)1599317661;
            p177.sue_lon_origin = (int) -2091738185;
            LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_WIND_ESTIMATION == (byte)(byte)59);
                Debug.Assert(pack.sue_BOARD_TYPE == (byte)(byte)170);
                Debug.Assert(pack.sue_DR == (byte)(byte)216);
                Debug.Assert(pack.sue_GPS_TYPE == (byte)(byte)113);
                Debug.Assert(pack.sue_FLIGHT_PLAN_TYPE == (byte)(byte)193);
                Debug.Assert(pack.sue_TRAP_FLAGS == (short)(short) -21702);
                Debug.Assert(pack.sue_AIRFRAME == (byte)(byte)35);
                Debug.Assert(pack.sue_TRAP_SOURCE == (uint)301274066U);
                Debug.Assert(pack.sue_RCON == (short)(short)2829);
                Debug.Assert(pack.sue_CLOCK_CONFIG == (byte)(byte)84);
                Debug.Assert(pack.sue_osc_fail_count == (short)(short)15685);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F14 p178 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_TRAP_SOURCE = (uint)301274066U;
            p178.sue_RCON = (short)(short)2829;
            p178.sue_WIND_ESTIMATION = (byte)(byte)59;
            p178.sue_DR = (byte)(byte)216;
            p178.sue_osc_fail_count = (short)(short)15685;
            p178.sue_AIRFRAME = (byte)(byte)35;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)193;
            p178.sue_TRAP_FLAGS = (short)(short) -21702;
            p178.sue_BOARD_TYPE = (byte)(byte)170;
            p178.sue_GPS_TYPE = (byte)(byte)113;
            p178.sue_CLOCK_CONFIG = (byte)(byte)84;
            LoopBackDemoChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_VEHICLE_MODEL_NAME.SequenceEqual(new byte[] {(byte)251, (byte)253, (byte)77, (byte)87, (byte)34, (byte)64, (byte)142, (byte)21, (byte)77, (byte)89, (byte)70, (byte)164, (byte)202, (byte)253, (byte)165, (byte)188, (byte)87, (byte)111, (byte)136, (byte)140, (byte)73, (byte)240, (byte)214, (byte)146, (byte)56, (byte)141, (byte)2, (byte)39, (byte)83, (byte)242, (byte)124, (byte)222, (byte)148, (byte)91, (byte)189, (byte)67, (byte)104, (byte)47, (byte)220, (byte)82}));
                Debug.Assert(pack.sue_ID_VEHICLE_REGISTRATION.SequenceEqual(new byte[] {(byte)193, (byte)196, (byte)65, (byte)206, (byte)246, (byte)133, (byte)162, (byte)82, (byte)171, (byte)90, (byte)142, (byte)58, (byte)61, (byte)110, (byte)12, (byte)8, (byte)197, (byte)112, (byte)4, (byte)87}));
            };
            DemoDevice.SERIAL_UDB_EXTRA_F15 p179 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[] {(byte)251, (byte)253, (byte)77, (byte)87, (byte)34, (byte)64, (byte)142, (byte)21, (byte)77, (byte)89, (byte)70, (byte)164, (byte)202, (byte)253, (byte)165, (byte)188, (byte)87, (byte)111, (byte)136, (byte)140, (byte)73, (byte)240, (byte)214, (byte)146, (byte)56, (byte)141, (byte)2, (byte)39, (byte)83, (byte)242, (byte)124, (byte)222, (byte)148, (byte)91, (byte)189, (byte)67, (byte)104, (byte)47, (byte)220, (byte)82}, 0) ;
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[] {(byte)193, (byte)196, (byte)65, (byte)206, (byte)246, (byte)133, (byte)162, (byte)82, (byte)171, (byte)90, (byte)142, (byte)58, (byte)61, (byte)110, (byte)12, (byte)8, (byte)197, (byte)112, (byte)4, (byte)87}, 0) ;
            LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_LEAD_PILOT.SequenceEqual(new byte[] {(byte)112, (byte)29, (byte)95, (byte)17, (byte)7, (byte)152, (byte)110, (byte)29, (byte)241, (byte)1, (byte)168, (byte)236, (byte)0, (byte)2, (byte)32, (byte)69, (byte)122, (byte)250, (byte)241, (byte)245, (byte)105, (byte)12, (byte)224, (byte)239, (byte)136, (byte)165, (byte)220, (byte)224, (byte)237, (byte)119, (byte)166, (byte)102, (byte)141, (byte)1, (byte)204, (byte)173, (byte)142, (byte)211, (byte)23, (byte)252}));
                Debug.Assert(pack.sue_ID_DIY_DRONES_URL.SequenceEqual(new byte[] {(byte)2, (byte)215, (byte)145, (byte)236, (byte)70, (byte)229, (byte)52, (byte)49, (byte)236, (byte)171, (byte)46, (byte)168, (byte)87, (byte)206, (byte)88, (byte)35, (byte)178, (byte)241, (byte)155, (byte)130, (byte)47, (byte)223, (byte)203, (byte)40, (byte)96, (byte)170, (byte)223, (byte)8, (byte)38, (byte)77, (byte)10, (byte)218, (byte)2, (byte)240, (byte)231, (byte)19, (byte)35, (byte)117, (byte)158, (byte)105, (byte)214, (byte)38, (byte)136, (byte)195, (byte)202, (byte)251, (byte)44, (byte)254, (byte)129, (byte)72, (byte)108, (byte)72, (byte)255, (byte)102, (byte)140, (byte)124, (byte)27, (byte)236, (byte)157, (byte)139, (byte)170, (byte)52, (byte)114, (byte)57, (byte)167, (byte)195, (byte)102, (byte)73, (byte)42, (byte)223}));
            };
            DemoDevice.SERIAL_UDB_EXTRA_F16 p180 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_LEAD_PILOT_SET(new byte[] {(byte)112, (byte)29, (byte)95, (byte)17, (byte)7, (byte)152, (byte)110, (byte)29, (byte)241, (byte)1, (byte)168, (byte)236, (byte)0, (byte)2, (byte)32, (byte)69, (byte)122, (byte)250, (byte)241, (byte)245, (byte)105, (byte)12, (byte)224, (byte)239, (byte)136, (byte)165, (byte)220, (byte)224, (byte)237, (byte)119, (byte)166, (byte)102, (byte)141, (byte)1, (byte)204, (byte)173, (byte)142, (byte)211, (byte)23, (byte)252}, 0) ;
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[] {(byte)2, (byte)215, (byte)145, (byte)236, (byte)70, (byte)229, (byte)52, (byte)49, (byte)236, (byte)171, (byte)46, (byte)168, (byte)87, (byte)206, (byte)88, (byte)35, (byte)178, (byte)241, (byte)155, (byte)130, (byte)47, (byte)223, (byte)203, (byte)40, (byte)96, (byte)170, (byte)223, (byte)8, (byte)38, (byte)77, (byte)10, (byte)218, (byte)2, (byte)240, (byte)231, (byte)19, (byte)35, (byte)117, (byte)158, (byte)105, (byte)214, (byte)38, (byte)136, (byte)195, (byte)202, (byte)251, (byte)44, (byte)254, (byte)129, (byte)72, (byte)108, (byte)72, (byte)255, (byte)102, (byte)140, (byte)124, (byte)27, (byte)236, (byte)157, (byte)139, (byte)170, (byte)52, (byte)114, (byte)57, (byte)167, (byte)195, (byte)102, (byte)73, (byte)42, (byte)223}, 0) ;
            LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_extra == (int)1847940410);
                Debug.Assert(pack.alt_range_finder == (int) -1357705748);
                Debug.Assert(pack.time_boot_ms == (uint)3029649921U);
                Debug.Assert(pack.alt_optical_flow == (int) -588086848);
                Debug.Assert(pack.alt_barometric == (int) -41987476);
                Debug.Assert(pack.alt_imu == (int) -222548605);
                Debug.Assert(pack.alt_gps == (int) -165377287);
            };
            DemoDevice.ALTITUDES p181 = LoopBackDemoChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.alt_extra = (int)1847940410;
            p181.alt_imu = (int) -222548605;
            p181.alt_gps = (int) -165377287;
            p181.alt_barometric = (int) -41987476;
            p181.time_boot_ms = (uint)3029649921U;
            p181.alt_range_finder = (int) -1357705748;
            p181.alt_optical_flow = (int) -588086848;
            LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed_hot_wire == (short)(short)936);
                Debug.Assert(pack.airspeed_ultrasonic == (short)(short)30488);
                Debug.Assert(pack.time_boot_ms == (uint)1028559781U);
                Debug.Assert(pack.aoa == (short)(short)26305);
                Debug.Assert(pack.aoy == (short)(short)30531);
                Debug.Assert(pack.airspeed_pitot == (short)(short)3017);
                Debug.Assert(pack.airspeed_imu == (short)(short)27071);
            };
            DemoDevice.AIRSPEEDS p182 = LoopBackDemoChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.aoa = (short)(short)26305;
            p182.airspeed_imu = (short)(short)27071;
            p182.airspeed_hot_wire = (short)(short)936;
            p182.airspeed_ultrasonic = (short)(short)30488;
            p182.airspeed_pitot = (short)(short)3017;
            p182.time_boot_ms = (uint)1028559781U;
            p182.aoy = (short)(short)30531;
            LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_feed_forward == (float) -3.3282554E38F);
                Debug.Assert(pack.sue_turn_rate_fbw == (float) -3.333984E38F);
                Debug.Assert(pack.sue_turn_rate_nav == (float) -2.2948906E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F17 p183 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float) -3.3282554E38F;
            p183.sue_turn_rate_fbw = (float) -3.333984E38F;
            p183.sue_turn_rate_nav = (float) -2.2948906E38F;
            LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.elevator_trim_inverted == (float) -6.035387E37F);
                Debug.Assert(pack.elevator_trim_normal == (float)2.294311E38F);
                Debug.Assert(pack.angle_of_attack_inverted == (float) -2.5261236E38F);
                Debug.Assert(pack.angle_of_attack_normal == (float)1.3995488E38F);
                Debug.Assert(pack.reference_speed == (float)2.935476E38F);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F18 p184 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_normal = (float)1.3995488E38F;
            p184.elevator_trim_inverted = (float) -6.035387E37F;
            p184.angle_of_attack_inverted = (float) -2.5261236E38F;
            p184.reference_speed = (float)2.935476E38F;
            p184.elevator_trim_normal = (float)2.294311E38F;
            LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_throttle_output_channel == (byte)(byte)108);
                Debug.Assert(pack.sue_aileron_output_channel == (byte)(byte)48);
                Debug.Assert(pack.sue_throttle_reversed == (byte)(byte)71);
                Debug.Assert(pack.sue_rudder_output_channel == (byte)(byte)64);
                Debug.Assert(pack.sue_elevator_reversed == (byte)(byte)91);
                Debug.Assert(pack.sue_elevator_output_channel == (byte)(byte)82);
                Debug.Assert(pack.sue_rudder_reversed == (byte)(byte)250);
                Debug.Assert(pack.sue_aileron_reversed == (byte)(byte)209);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F19 p185 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)48;
            p185.sue_throttle_output_channel = (byte)(byte)108;
            p185.sue_rudder_output_channel = (byte)(byte)64;
            p185.sue_elevator_reversed = (byte)(byte)91;
            p185.sue_rudder_reversed = (byte)(byte)250;
            p185.sue_aileron_reversed = (byte)(byte)209;
            p185.sue_throttle_reversed = (byte)(byte)71;
            p185.sue_elevator_output_channel = (byte)(byte)82;
            LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_trim_value_input_8 == (short)(short) -16140);
                Debug.Assert(pack.sue_trim_value_input_7 == (short)(short)31384);
                Debug.Assert(pack.sue_trim_value_input_11 == (short)(short)6255);
                Debug.Assert(pack.sue_trim_value_input_4 == (short)(short) -32319);
                Debug.Assert(pack.sue_trim_value_input_9 == (short)(short) -11006);
                Debug.Assert(pack.sue_trim_value_input_1 == (short)(short) -19077);
                Debug.Assert(pack.sue_trim_value_input_6 == (short)(short)11327);
                Debug.Assert(pack.sue_trim_value_input_2 == (short)(short)1429);
                Debug.Assert(pack.sue_trim_value_input_12 == (short)(short)19445);
                Debug.Assert(pack.sue_trim_value_input_10 == (short)(short)15735);
                Debug.Assert(pack.sue_trim_value_input_3 == (short)(short) -22462);
                Debug.Assert(pack.sue_trim_value_input_5 == (short)(short)11871);
                Debug.Assert(pack.sue_number_of_inputs == (byte)(byte)241);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F20 p186 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_trim_value_input_10 = (short)(short)15735;
            p186.sue_trim_value_input_1 = (short)(short) -19077;
            p186.sue_trim_value_input_8 = (short)(short) -16140;
            p186.sue_trim_value_input_4 = (short)(short) -32319;
            p186.sue_trim_value_input_2 = (short)(short)1429;
            p186.sue_trim_value_input_12 = (short)(short)19445;
            p186.sue_trim_value_input_6 = (short)(short)11327;
            p186.sue_trim_value_input_7 = (short)(short)31384;
            p186.sue_trim_value_input_11 = (short)(short)6255;
            p186.sue_trim_value_input_9 = (short)(short) -11006;
            p186.sue_trim_value_input_5 = (short)(short)11871;
            p186.sue_number_of_inputs = (byte)(byte)241;
            p186.sue_trim_value_input_3 = (short)(short) -22462;
            LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_gyro_y_offset == (short)(short) -458);
                Debug.Assert(pack.sue_gyro_z_offset == (short)(short)15449);
                Debug.Assert(pack.sue_accel_x_offset == (short)(short) -9277);
                Debug.Assert(pack.sue_gyro_x_offset == (short)(short)23478);
                Debug.Assert(pack.sue_accel_z_offset == (short)(short)13028);
                Debug.Assert(pack.sue_accel_y_offset == (short)(short)919);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F21 p187 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_gyro_z_offset = (short)(short)15449;
            p187.sue_accel_x_offset = (short)(short) -9277;
            p187.sue_accel_y_offset = (short)(short)919;
            p187.sue_gyro_y_offset = (short)(short) -458;
            p187.sue_gyro_x_offset = (short)(short)23478;
            p187.sue_accel_z_offset = (short)(short)13028;
            LoopBackDemoChannel.instance.send(p187);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_gyro_z_at_calibration == (short)(short) -19374);
                Debug.Assert(pack.sue_gyro_y_at_calibration == (short)(short) -20876);
                Debug.Assert(pack.sue_accel_x_at_calibration == (short)(short) -6809);
                Debug.Assert(pack.sue_gyro_x_at_calibration == (short)(short)12577);
                Debug.Assert(pack.sue_accel_z_at_calibration == (short)(short)28949);
                Debug.Assert(pack.sue_accel_y_at_calibration == (short)(short)13523);
            };
            DemoDevice.SERIAL_UDB_EXTRA_F22 p188 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_gyro_z_at_calibration = (short)(short) -19374;
            p188.sue_accel_z_at_calibration = (short)(short)28949;
            p188.sue_gyro_x_at_calibration = (short)(short)12577;
            p188.sue_accel_y_at_calibration = (short)(short)13523;
            p188.sue_accel_x_at_calibration = (short)(short) -6809;
            p188.sue_gyro_y_at_calibration = (short)(short) -20876;
            LoopBackDemoChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_horiz_ratio == (float)2.3898901E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.9378648E38F);
                Debug.Assert(pack.mag_ratio == (float)2.8159438E37F);
                Debug.Assert(pack.tas_ratio == (float) -3.4310857E37F);
                Debug.Assert(pack.time_usec == (ulong)5044594790436246591L);
                Debug.Assert(pack.pos_vert_ratio == (float) -5.154975E37F);
                Debug.Assert(pack.hagl_ratio == (float) -1.3483371E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.5232569E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.130946E38F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.vel_ratio = (float) -2.5232569E38F;
            p230.tas_ratio = (float) -3.4310857E37F;
            p230.time_usec = (ulong)5044594790436246591L;
            p230.pos_vert_ratio = (float) -5.154975E37F;
            p230.pos_horiz_accuracy = (float)1.130946E38F;
            p230.hagl_ratio = (float) -1.3483371E38F;
            p230.mag_ratio = (float)2.8159438E37F;
            p230.pos_horiz_ratio = (float)2.3898901E38F;
            p230.pos_vert_accuracy = (float) -1.9378648E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float)9.522061E37F);
                Debug.Assert(pack.wind_z == (float) -1.0046163E38F);
                Debug.Assert(pack.wind_alt == (float) -1.9150544E38F);
                Debug.Assert(pack.wind_y == (float) -3.0326157E38F);
                Debug.Assert(pack.time_usec == (ulong)5238444463879781715L);
                Debug.Assert(pack.var_horiz == (float) -1.14156E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.1317861E38F);
                Debug.Assert(pack.var_vert == (float)4.3712244E37F);
                Debug.Assert(pack.wind_x == (float)7.330997E37F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float) -1.14156E38F;
            p231.horiz_accuracy = (float)9.522061E37F;
            p231.var_vert = (float)4.3712244E37F;
            p231.wind_z = (float) -1.0046163E38F;
            p231.wind_alt = (float) -1.9150544E38F;
            p231.wind_x = (float)7.330997E37F;
            p231.time_usec = (ulong)5238444463879781715L;
            p231.vert_accuracy = (float)1.1317861E38F;
            p231.wind_y = (float) -3.0326157E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)188);
                Debug.Assert(pack.time_week == (ushort)(ushort)5829);
                Debug.Assert(pack.ve == (float) -1.9007415E38F);
                Debug.Assert(pack.vdop == (float)1.0579332E37F);
                Debug.Assert(pack.speed_accuracy == (float) -1.8195936E38F);
                Debug.Assert(pack.vert_accuracy == (float)3.0530282E38F);
                Debug.Assert(pack.hdop == (float)2.9089243E38F);
                Debug.Assert(pack.time_usec == (ulong)8135482691376081194L);
                Debug.Assert(pack.vn == (float)2.3275268E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)168);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
                Debug.Assert(pack.vd == (float) -2.5511339E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)48);
                Debug.Assert(pack.lat == (int)1710763828);
                Debug.Assert(pack.alt == (float)4.0275797E37F);
                Debug.Assert(pack.time_week_ms == (uint)4235300375U);
                Debug.Assert(pack.horiz_accuracy == (float)9.822049E37F);
                Debug.Assert(pack.lon == (int) -1340369611);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lat = (int)1710763828;
            p232.ve = (float) -1.9007415E38F;
            p232.time_usec = (ulong)8135482691376081194L;
            p232.satellites_visible = (byte)(byte)188;
            p232.vdop = (float)1.0579332E37F;
            p232.alt = (float)4.0275797E37F;
            p232.lon = (int) -1340369611;
            p232.time_week = (ushort)(ushort)5829;
            p232.vert_accuracy = (float)3.0530282E38F;
            p232.gps_id = (byte)(byte)48;
            p232.horiz_accuracy = (float)9.822049E37F;
            p232.fix_type = (byte)(byte)168;
            p232.speed_accuracy = (float) -1.8195936E38F;
            p232.hdop = (float)2.9089243E38F;
            p232.vd = (float) -2.5511339E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
            p232.time_week_ms = (uint)4235300375U;
            p232.vn = (float)2.3275268E38F;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)76);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)206, (byte)234, (byte)163, (byte)219, (byte)241, (byte)135, (byte)138, (byte)225, (byte)128, (byte)206, (byte)175, (byte)45, (byte)16, (byte)59, (byte)23, (byte)227, (byte)177, (byte)160, (byte)220, (byte)18, (byte)254, (byte)56, (byte)185, (byte)61, (byte)75, (byte)233, (byte)158, (byte)67, (byte)225, (byte)158, (byte)101, (byte)240, (byte)74, (byte)173, (byte)157, (byte)247, (byte)209, (byte)84, (byte)196, (byte)52, (byte)55, (byte)71, (byte)134, (byte)218, (byte)191, (byte)87, (byte)151, (byte)165, (byte)116, (byte)11, (byte)58, (byte)104, (byte)144, (byte)18, (byte)211, (byte)161, (byte)204, (byte)56, (byte)168, (byte)215, (byte)17, (byte)38, (byte)35, (byte)44, (byte)62, (byte)250, (byte)131, (byte)89, (byte)127, (byte)181, (byte)35, (byte)14, (byte)13, (byte)180, (byte)156, (byte)221, (byte)141, (byte)170, (byte)181, (byte)236, (byte)182, (byte)224, (byte)182, (byte)238, (byte)185, (byte)189, (byte)5, (byte)31, (byte)55, (byte)57, (byte)227, (byte)210, (byte)7, (byte)88, (byte)160, (byte)194, (byte)133, (byte)101, (byte)180, (byte)248, (byte)101, (byte)162, (byte)83, (byte)205, (byte)171, (byte)43, (byte)16, (byte)202, (byte)106, (byte)90, (byte)57, (byte)102, (byte)25, (byte)28, (byte)168, (byte)89, (byte)243, (byte)71, (byte)33, (byte)164, (byte)88, (byte)16, (byte)44, (byte)179, (byte)238, (byte)164, (byte)245, (byte)148, (byte)180, (byte)247, (byte)188, (byte)131, (byte)222, (byte)161, (byte)229, (byte)146, (byte)244, (byte)75, (byte)164, (byte)187, (byte)176, (byte)101, (byte)240, (byte)240, (byte)119, (byte)122, (byte)237, (byte)67, (byte)25, (byte)189, (byte)240, (byte)134, (byte)183, (byte)205, (byte)25, (byte)23, (byte)84, (byte)150, (byte)162, (byte)72, (byte)199, (byte)193, (byte)51, (byte)152, (byte)139, (byte)218, (byte)172, (byte)169, (byte)13, (byte)226, (byte)175, (byte)152, (byte)146, (byte)213, (byte)229, (byte)254, (byte)28, (byte)183, (byte)219, (byte)62}));
                Debug.Assert(pack.len == (byte)(byte)70);
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)76;
            p233.data__SET(new byte[] {(byte)206, (byte)234, (byte)163, (byte)219, (byte)241, (byte)135, (byte)138, (byte)225, (byte)128, (byte)206, (byte)175, (byte)45, (byte)16, (byte)59, (byte)23, (byte)227, (byte)177, (byte)160, (byte)220, (byte)18, (byte)254, (byte)56, (byte)185, (byte)61, (byte)75, (byte)233, (byte)158, (byte)67, (byte)225, (byte)158, (byte)101, (byte)240, (byte)74, (byte)173, (byte)157, (byte)247, (byte)209, (byte)84, (byte)196, (byte)52, (byte)55, (byte)71, (byte)134, (byte)218, (byte)191, (byte)87, (byte)151, (byte)165, (byte)116, (byte)11, (byte)58, (byte)104, (byte)144, (byte)18, (byte)211, (byte)161, (byte)204, (byte)56, (byte)168, (byte)215, (byte)17, (byte)38, (byte)35, (byte)44, (byte)62, (byte)250, (byte)131, (byte)89, (byte)127, (byte)181, (byte)35, (byte)14, (byte)13, (byte)180, (byte)156, (byte)221, (byte)141, (byte)170, (byte)181, (byte)236, (byte)182, (byte)224, (byte)182, (byte)238, (byte)185, (byte)189, (byte)5, (byte)31, (byte)55, (byte)57, (byte)227, (byte)210, (byte)7, (byte)88, (byte)160, (byte)194, (byte)133, (byte)101, (byte)180, (byte)248, (byte)101, (byte)162, (byte)83, (byte)205, (byte)171, (byte)43, (byte)16, (byte)202, (byte)106, (byte)90, (byte)57, (byte)102, (byte)25, (byte)28, (byte)168, (byte)89, (byte)243, (byte)71, (byte)33, (byte)164, (byte)88, (byte)16, (byte)44, (byte)179, (byte)238, (byte)164, (byte)245, (byte)148, (byte)180, (byte)247, (byte)188, (byte)131, (byte)222, (byte)161, (byte)229, (byte)146, (byte)244, (byte)75, (byte)164, (byte)187, (byte)176, (byte)101, (byte)240, (byte)240, (byte)119, (byte)122, (byte)237, (byte)67, (byte)25, (byte)189, (byte)240, (byte)134, (byte)183, (byte)205, (byte)25, (byte)23, (byte)84, (byte)150, (byte)162, (byte)72, (byte)199, (byte)193, (byte)51, (byte)152, (byte)139, (byte)218, (byte)172, (byte)169, (byte)13, (byte)226, (byte)175, (byte)152, (byte)146, (byte)213, (byte)229, (byte)254, (byte)28, (byte)183, (byte)219, (byte)62}, 0) ;
            p233.len = (byte)(byte)70;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.groundspeed == (byte)(byte)110);
                Debug.Assert(pack.pitch == (short)(short)14991);
                Debug.Assert(pack.wp_num == (byte)(byte)97);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)114);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)254);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 109);
                Debug.Assert(pack.altitude_amsl == (short)(short) -23604);
                Debug.Assert(pack.roll == (short)(short)14550);
                Debug.Assert(pack.custom_mode == (uint)3809746870U);
                Debug.Assert(pack.altitude_sp == (short)(short) -1725);
                Debug.Assert(pack.battery_remaining == (byte)(byte)94);
                Debug.Assert(pack.heading == (ushort)(ushort)36178);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 77);
                Debug.Assert(pack.gps_nsat == (byte)(byte)159);
                Debug.Assert(pack.heading_sp == (short)(short) -23638);
                Debug.Assert(pack.latitude == (int) -800663721);
                Debug.Assert(pack.failsafe == (byte)(byte)49);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)61);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
                Debug.Assert(pack.longitude == (int) -1412011989);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)35742);
                Debug.Assert(pack.airspeed == (byte)(byte)0);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.longitude = (int) -1412011989;
            p234.failsafe = (byte)(byte)49;
            p234.roll = (short)(short)14550;
            p234.temperature_air = (sbyte)(sbyte) - 77;
            p234.battery_remaining = (byte)(byte)94;
            p234.temperature = (sbyte)(sbyte)61;
            p234.wp_num = (byte)(byte)97;
            p234.heading_sp = (short)(short) -23638;
            p234.altitude_amsl = (short)(short) -23604;
            p234.altitude_sp = (short)(short) -1725;
            p234.throttle = (sbyte)(sbyte)114;
            p234.custom_mode = (uint)3809746870U;
            p234.groundspeed = (byte)(byte)110;
            p234.climb_rate = (sbyte)(sbyte) - 109;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.airspeed_sp = (byte)(byte)254;
            p234.pitch = (short)(short)14991;
            p234.wp_distance = (ushort)(ushort)35742;
            p234.heading = (ushort)(ushort)36178;
            p234.airspeed = (byte)(byte)0;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p234.latitude = (int) -800663721;
            p234.gps_nsat = (byte)(byte)159;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_1 == (uint)2857183843U);
                Debug.Assert(pack.vibration_z == (float)3.0015885E38F);
                Debug.Assert(pack.vibration_y == (float) -3.9448035E36F);
                Debug.Assert(pack.vibration_x == (float) -1.9999902E38F);
                Debug.Assert(pack.clipping_2 == (uint)3397168722U);
                Debug.Assert(pack.time_usec == (ulong)3932787353066818595L);
                Debug.Assert(pack.clipping_0 == (uint)2034703762U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)2034703762U;
            p241.vibration_y = (float) -3.9448035E36F;
            p241.clipping_1 = (uint)2857183843U;
            p241.vibration_x = (float) -1.9999902E38F;
            p241.time_usec = (ulong)3932787353066818595L;
            p241.clipping_2 = (uint)3397168722U;
            p241.vibration_z = (float)3.0015885E38F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float) -1.3099086E38F);
                Debug.Assert(pack.altitude == (int)220196132);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6839343E38F, -3.400425E37F, 1.913313E38F, 2.5370437E38F}));
                Debug.Assert(pack.x == (float)1.7062143E38F);
                Debug.Assert(pack.z == (float) -1.4524047E38F);
                Debug.Assert(pack.approach_z == (float)1.4561257E38F);
                Debug.Assert(pack.approach_x == (float) -3.0529146E38F);
                Debug.Assert(pack.y == (float)2.0896427E38F);
                Debug.Assert(pack.longitude == (int)1255635509);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)861301950741292229L);
                Debug.Assert(pack.latitude == (int)1226085417);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_x = (float) -3.0529146E38F;
            p242.q_SET(new float[] {2.6839343E38F, -3.400425E37F, 1.913313E38F, 2.5370437E38F}, 0) ;
            p242.z = (float) -1.4524047E38F;
            p242.y = (float)2.0896427E38F;
            p242.time_usec_SET((ulong)861301950741292229L, PH) ;
            p242.altitude = (int)220196132;
            p242.longitude = (int)1255635509;
            p242.latitude = (int)1226085417;
            p242.approach_y = (float) -1.3099086E38F;
            p242.x = (float)1.7062143E38F;
            p242.approach_z = (float)1.4561257E38F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -60648502);
                Debug.Assert(pack.latitude == (int) -1763383117);
                Debug.Assert(pack.approach_z == (float)1.252179E38F);
                Debug.Assert(pack.longitude == (int) -1026972520);
                Debug.Assert(pack.approach_y == (float)2.225703E38F);
                Debug.Assert(pack.x == (float)3.0280017E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0764575E38F, -1.7387296E37F, -6.0995666E37F, -2.740334E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1867747653026001573L);
                Debug.Assert(pack.target_system == (byte)(byte)217);
                Debug.Assert(pack.z == (float) -8.4654625E37F);
                Debug.Assert(pack.y == (float)2.9824916E38F);
                Debug.Assert(pack.approach_x == (float)2.292368E38F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.latitude = (int) -1763383117;
            p243.approach_z = (float)1.252179E38F;
            p243.q_SET(new float[] {2.0764575E38F, -1.7387296E37F, -6.0995666E37F, -2.740334E38F}, 0) ;
            p243.approach_x = (float)2.292368E38F;
            p243.x = (float)3.0280017E38F;
            p243.approach_y = (float)2.225703E38F;
            p243.y = (float)2.9824916E38F;
            p243.longitude = (int) -1026972520;
            p243.altitude = (int) -60648502;
            p243.target_system = (byte)(byte)217;
            p243.time_usec_SET((ulong)1867747653026001573L, PH) ;
            p243.z = (float) -8.4654625E37F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)1647111085);
                Debug.Assert(pack.message_id == (ushort)(ushort)24183);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1647111085;
            p244.message_id = (ushort)(ushort)24183;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("n"));
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)21737);
                Debug.Assert(pack.ICAO_address == (uint)2224535038U);
                Debug.Assert(pack.lon == (int)1788634622);
                Debug.Assert(pack.ver_velocity == (short)(short)5652);
                Debug.Assert(pack.tslc == (byte)(byte)180);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.altitude == (int) -650963929);
                Debug.Assert(pack.squawk == (ushort)(ushort)12877);
                Debug.Assert(pack.heading == (ushort)(ushort)43612);
                Debug.Assert(pack.lat == (int)1203666275);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -650963929;
            p246.heading = (ushort)(ushort)43612;
            p246.lat = (int)1203666275;
            p246.squawk = (ushort)(ushort)12877;
            p246.tslc = (byte)(byte)180;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT;
            p246.hor_velocity = (ushort)(ushort)21737;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
            p246.callsign_SET("n", PH) ;
            p246.ver_velocity = (short)(short)5652;
            p246.ICAO_address = (uint)2224535038U;
            p246.lon = (int)1788634622;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)2217256840U);
                Debug.Assert(pack.time_to_minimum_delta == (float) -7.7176255E36F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.8746396E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
                Debug.Assert(pack.altitude_minimum_delta == (float) -1.2269263E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.time_to_minimum_delta = (float) -7.7176255E36F;
            p247.id = (uint)2217256840U;
            p247.altitude_minimum_delta = (float) -1.2269263E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.horizontal_minimum_delta = (float)1.8746396E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)203, (byte)46, (byte)18, (byte)94, (byte)207, (byte)102, (byte)254, (byte)241, (byte)76, (byte)105, (byte)245, (byte)97, (byte)178, (byte)132, (byte)138, (byte)249, (byte)238, (byte)75, (byte)215, (byte)187, (byte)88, (byte)156, (byte)53, (byte)32, (byte)226, (byte)36, (byte)195, (byte)132, (byte)160, (byte)247, (byte)142, (byte)96, (byte)208, (byte)72, (byte)193, (byte)160, (byte)227, (byte)36, (byte)157, (byte)62, (byte)142, (byte)102, (byte)81, (byte)213, (byte)27, (byte)136, (byte)196, (byte)55, (byte)129, (byte)31, (byte)161, (byte)49, (byte)178, (byte)32, (byte)114, (byte)87, (byte)179, (byte)160, (byte)182, (byte)169, (byte)243, (byte)119, (byte)18, (byte)0, (byte)145, (byte)239, (byte)175, (byte)47, (byte)172, (byte)75, (byte)110, (byte)28, (byte)149, (byte)221, (byte)102, (byte)175, (byte)36, (byte)50, (byte)198, (byte)28, (byte)185, (byte)68, (byte)167, (byte)59, (byte)222, (byte)39, (byte)125, (byte)50, (byte)86, (byte)134, (byte)213, (byte)9, (byte)75, (byte)217, (byte)219, (byte)136, (byte)24, (byte)218, (byte)230, (byte)12, (byte)79, (byte)109, (byte)145, (byte)56, (byte)64, (byte)47, (byte)237, (byte)194, (byte)1, (byte)228, (byte)123, (byte)183, (byte)158, (byte)64, (byte)10, (byte)134, (byte)0, (byte)23, (byte)101, (byte)26, (byte)144, (byte)57, (byte)244, (byte)117, (byte)131, (byte)178, (byte)241, (byte)24, (byte)49, (byte)68, (byte)50, (byte)12, (byte)163, (byte)205, (byte)149, (byte)56, (byte)110, (byte)137, (byte)86, (byte)29, (byte)137, (byte)34, (byte)182, (byte)153, (byte)24, (byte)252, (byte)99, (byte)8, (byte)168, (byte)44, (byte)115, (byte)5, (byte)207, (byte)96, (byte)6, (byte)184, (byte)130, (byte)89, (byte)124, (byte)254, (byte)124, (byte)73, (byte)173, (byte)93, (byte)23, (byte)48, (byte)72, (byte)229, (byte)62, (byte)105, (byte)77, (byte)138, (byte)119, (byte)68, (byte)254, (byte)119, (byte)187, (byte)200, (byte)129, (byte)44, (byte)116, (byte)58, (byte)51, (byte)89, (byte)248, (byte)118, (byte)181, (byte)207, (byte)204, (byte)125, (byte)59, (byte)172, (byte)10, (byte)156, (byte)162, (byte)113, (byte)10, (byte)48, (byte)28, (byte)90, (byte)251, (byte)18, (byte)186, (byte)198, (byte)91, (byte)24, (byte)111, (byte)133, (byte)143, (byte)242, (byte)203, (byte)111, (byte)178, (byte)187, (byte)129, (byte)17, (byte)174, (byte)237, (byte)237, (byte)52, (byte)118, (byte)85, (byte)34, (byte)17, (byte)155, (byte)47, (byte)90, (byte)66, (byte)239, (byte)251, (byte)209, (byte)16, (byte)208, (byte)36, (byte)241, (byte)73, (byte)123, (byte)4, (byte)77, (byte)200, (byte)190, (byte)120, (byte)136, (byte)193, (byte)46, (byte)217, (byte)100, (byte)110, (byte)198}));
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.target_network == (byte)(byte)164);
                Debug.Assert(pack.message_type == (ushort)(ushort)12876);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)12876;
            p248.payload_SET(new byte[] {(byte)203, (byte)46, (byte)18, (byte)94, (byte)207, (byte)102, (byte)254, (byte)241, (byte)76, (byte)105, (byte)245, (byte)97, (byte)178, (byte)132, (byte)138, (byte)249, (byte)238, (byte)75, (byte)215, (byte)187, (byte)88, (byte)156, (byte)53, (byte)32, (byte)226, (byte)36, (byte)195, (byte)132, (byte)160, (byte)247, (byte)142, (byte)96, (byte)208, (byte)72, (byte)193, (byte)160, (byte)227, (byte)36, (byte)157, (byte)62, (byte)142, (byte)102, (byte)81, (byte)213, (byte)27, (byte)136, (byte)196, (byte)55, (byte)129, (byte)31, (byte)161, (byte)49, (byte)178, (byte)32, (byte)114, (byte)87, (byte)179, (byte)160, (byte)182, (byte)169, (byte)243, (byte)119, (byte)18, (byte)0, (byte)145, (byte)239, (byte)175, (byte)47, (byte)172, (byte)75, (byte)110, (byte)28, (byte)149, (byte)221, (byte)102, (byte)175, (byte)36, (byte)50, (byte)198, (byte)28, (byte)185, (byte)68, (byte)167, (byte)59, (byte)222, (byte)39, (byte)125, (byte)50, (byte)86, (byte)134, (byte)213, (byte)9, (byte)75, (byte)217, (byte)219, (byte)136, (byte)24, (byte)218, (byte)230, (byte)12, (byte)79, (byte)109, (byte)145, (byte)56, (byte)64, (byte)47, (byte)237, (byte)194, (byte)1, (byte)228, (byte)123, (byte)183, (byte)158, (byte)64, (byte)10, (byte)134, (byte)0, (byte)23, (byte)101, (byte)26, (byte)144, (byte)57, (byte)244, (byte)117, (byte)131, (byte)178, (byte)241, (byte)24, (byte)49, (byte)68, (byte)50, (byte)12, (byte)163, (byte)205, (byte)149, (byte)56, (byte)110, (byte)137, (byte)86, (byte)29, (byte)137, (byte)34, (byte)182, (byte)153, (byte)24, (byte)252, (byte)99, (byte)8, (byte)168, (byte)44, (byte)115, (byte)5, (byte)207, (byte)96, (byte)6, (byte)184, (byte)130, (byte)89, (byte)124, (byte)254, (byte)124, (byte)73, (byte)173, (byte)93, (byte)23, (byte)48, (byte)72, (byte)229, (byte)62, (byte)105, (byte)77, (byte)138, (byte)119, (byte)68, (byte)254, (byte)119, (byte)187, (byte)200, (byte)129, (byte)44, (byte)116, (byte)58, (byte)51, (byte)89, (byte)248, (byte)118, (byte)181, (byte)207, (byte)204, (byte)125, (byte)59, (byte)172, (byte)10, (byte)156, (byte)162, (byte)113, (byte)10, (byte)48, (byte)28, (byte)90, (byte)251, (byte)18, (byte)186, (byte)198, (byte)91, (byte)24, (byte)111, (byte)133, (byte)143, (byte)242, (byte)203, (byte)111, (byte)178, (byte)187, (byte)129, (byte)17, (byte)174, (byte)237, (byte)237, (byte)52, (byte)118, (byte)85, (byte)34, (byte)17, (byte)155, (byte)47, (byte)90, (byte)66, (byte)239, (byte)251, (byte)209, (byte)16, (byte)208, (byte)36, (byte)241, (byte)73, (byte)123, (byte)4, (byte)77, (byte)200, (byte)190, (byte)120, (byte)136, (byte)193, (byte)46, (byte)217, (byte)100, (byte)110, (byte)198}, 0) ;
            p248.target_network = (byte)(byte)164;
            p248.target_system = (byte)(byte)254;
            p248.target_component = (byte)(byte)229;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 77, (sbyte)4, (sbyte) - 115, (sbyte)50, (sbyte) - 76, (sbyte)120, (sbyte) - 122, (sbyte)18, (sbyte) - 79, (sbyte)58, (sbyte)2, (sbyte)25, (sbyte) - 95, (sbyte)126, (sbyte)21, (sbyte)25, (sbyte) - 116, (sbyte) - 71, (sbyte) - 48, (sbyte)47, (sbyte) - 90, (sbyte)28, (sbyte) - 70, (sbyte)20, (sbyte)69, (sbyte)99, (sbyte) - 106, (sbyte)45, (sbyte) - 10, (sbyte) - 19, (sbyte) - 42, (sbyte)124}));
                Debug.Assert(pack.type == (byte)(byte)83);
                Debug.Assert(pack.address == (ushort)(ushort)22828);
                Debug.Assert(pack.ver == (byte)(byte)233);
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)22828;
            p249.value_SET(new sbyte[] {(sbyte) - 77, (sbyte)4, (sbyte) - 115, (sbyte)50, (sbyte) - 76, (sbyte)120, (sbyte) - 122, (sbyte)18, (sbyte) - 79, (sbyte)58, (sbyte)2, (sbyte)25, (sbyte) - 95, (sbyte)126, (sbyte)21, (sbyte)25, (sbyte) - 116, (sbyte) - 71, (sbyte) - 48, (sbyte)47, (sbyte) - 90, (sbyte)28, (sbyte) - 70, (sbyte)20, (sbyte)69, (sbyte)99, (sbyte) - 106, (sbyte)45, (sbyte) - 10, (sbyte) - 19, (sbyte) - 42, (sbyte)124}, 0) ;
            p249.ver = (byte)(byte)233;
            p249.type = (byte)(byte)83;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("fkaVzumvvk"));
                Debug.Assert(pack.y == (float)8.971538E35F);
                Debug.Assert(pack.time_usec == (ulong)4050692665995953120L);
                Debug.Assert(pack.z == (float) -2.0047288E38F);
                Debug.Assert(pack.x == (float)1.830564E37F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("fkaVzumvvk", PH) ;
            p250.z = (float) -2.0047288E38F;
            p250.x = (float)1.830564E37F;
            p250.y = (float)8.971538E35F;
            p250.time_usec = (ulong)4050692665995953120L;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1615511493U);
                Debug.Assert(pack.value == (float) -2.0378372E38F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("qqyfWB"));
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1615511493U;
            p251.value = (float) -2.0378372E38F;
            p251.name_SET("qqyfWB", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("fkp"));
                Debug.Assert(pack.value == (int)2103369388);
                Debug.Assert(pack.time_boot_ms == (uint)1127350452U);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)1127350452U;
            p252.name_SET("fkp", PH) ;
            p252.value = (int)2103369388;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_DEBUG);
                Debug.Assert(pack.text_LEN(ph) == 22);
                Debug.Assert(pack.text_TRY(ph).Equals("fyufttCbrczfzlxlhfouqy"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("fyufttCbrczfzlxlhfouqy", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_DEBUG;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -9.406921E37F);
                Debug.Assert(pack.ind == (byte)(byte)250);
                Debug.Assert(pack.time_boot_ms == (uint)709984867U);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)250;
            p254.time_boot_ms = (uint)709984867U;
            p254.value = (float) -9.406921E37F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)154, (byte)177, (byte)25, (byte)147, (byte)9, (byte)193, (byte)146, (byte)208, (byte)159, (byte)26, (byte)175, (byte)57, (byte)220, (byte)65, (byte)219, (byte)154, (byte)27, (byte)51, (byte)2, (byte)224, (byte)223, (byte)63, (byte)19, (byte)7, (byte)84, (byte)174, (byte)28, (byte)217, (byte)222, (byte)51, (byte)150, (byte)132}));
                Debug.Assert(pack.initial_timestamp == (ulong)6886528926720425809L);
                Debug.Assert(pack.target_component == (byte)(byte)146);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)229;
            p256.initial_timestamp = (ulong)6886528926720425809L;
            p256.target_component = (byte)(byte)146;
            p256.secret_key_SET(new byte[] {(byte)154, (byte)177, (byte)25, (byte)147, (byte)9, (byte)193, (byte)146, (byte)208, (byte)159, (byte)26, (byte)175, (byte)57, (byte)220, (byte)65, (byte)219, (byte)154, (byte)27, (byte)51, (byte)2, (byte)224, (byte)223, (byte)63, (byte)19, (byte)7, (byte)84, (byte)174, (byte)28, (byte)217, (byte)222, (byte)51, (byte)150, (byte)132}, 0) ;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)111);
                Debug.Assert(pack.last_change_ms == (uint)2078532891U);
                Debug.Assert(pack.time_boot_ms == (uint)1678133774U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1678133774U;
            p257.last_change_ms = (uint)2078532891U;
            p257.state = (byte)(byte)111;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.tune_LEN(ph) == 10);
                Debug.Assert(pack.tune_TRY(ph).Equals("lwsjukmgoj"));
                Debug.Assert(pack.target_component == (byte)(byte)69);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)22;
            p258.target_component = (byte)(byte)69;
            p258.tune_SET("lwsjukmgoj", PH) ;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)25154);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)68, (byte)108, (byte)225, (byte)3, (byte)64, (byte)153, (byte)107, (byte)81, (byte)150, (byte)121, (byte)10, (byte)124, (byte)42, (byte)22, (byte)128, (byte)249, (byte)241, (byte)250, (byte)248, (byte)224, (byte)14, (byte)177, (byte)236, (byte)142, (byte)193, (byte)222, (byte)132, (byte)228, (byte)233, (byte)210, (byte)74, (byte)192}));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)198, (byte)201, (byte)150, (byte)145, (byte)116, (byte)174, (byte)140, (byte)206, (byte)160, (byte)34, (byte)143, (byte)247, (byte)34, (byte)136, (byte)115, (byte)81, (byte)165, (byte)185, (byte)252, (byte)213, (byte)203, (byte)141, (byte)195, (byte)215, (byte)104, (byte)136, (byte)162, (byte)163, (byte)72, (byte)135, (byte)92, (byte)152}));
                Debug.Assert(pack.sensor_size_v == (float)2.0096047E38F);
                Debug.Assert(pack.firmware_version == (uint)1709043309U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)6857);
                Debug.Assert(pack.time_boot_ms == (uint)2078616710U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)27634);
                Debug.Assert(pack.lens_id == (byte)(byte)204);
                Debug.Assert(pack.focal_length == (float)5.1784734E37F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 59);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("hzcsIobtvrnmLurIuzevmbwssZmsiuetQqjunhrqipvtckhbjpCxewbkPqi"));
                Debug.Assert(pack.sensor_size_h == (float) -2.0722743E38F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.lens_id = (byte)(byte)204;
            p259.resolution_h = (ushort)(ushort)25154;
            p259.resolution_v = (ushort)(ushort)27634;
            p259.model_name_SET(new byte[] {(byte)68, (byte)108, (byte)225, (byte)3, (byte)64, (byte)153, (byte)107, (byte)81, (byte)150, (byte)121, (byte)10, (byte)124, (byte)42, (byte)22, (byte)128, (byte)249, (byte)241, (byte)250, (byte)248, (byte)224, (byte)14, (byte)177, (byte)236, (byte)142, (byte)193, (byte)222, (byte)132, (byte)228, (byte)233, (byte)210, (byte)74, (byte)192}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)6857;
            p259.cam_definition_uri_SET("hzcsIobtvrnmLurIuzevmbwssZmsiuetQqjunhrqipvtckhbjpCxewbkPqi", PH) ;
            p259.focal_length = (float)5.1784734E37F;
            p259.vendor_name_SET(new byte[] {(byte)198, (byte)201, (byte)150, (byte)145, (byte)116, (byte)174, (byte)140, (byte)206, (byte)160, (byte)34, (byte)143, (byte)247, (byte)34, (byte)136, (byte)115, (byte)81, (byte)165, (byte)185, (byte)252, (byte)213, (byte)203, (byte)141, (byte)195, (byte)215, (byte)104, (byte)136, (byte)162, (byte)163, (byte)72, (byte)135, (byte)92, (byte)152}, 0) ;
            p259.time_boot_ms = (uint)2078616710U;
            p259.sensor_size_h = (float) -2.0722743E38F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
            p259.firmware_version = (uint)1709043309U;
            p259.sensor_size_v = (float)2.0096047E38F;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3551511418U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3551511418U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.used_capacity == (float) -4.743166E37F);
                Debug.Assert(pack.available_capacity == (float) -1.8205665E38F);
                Debug.Assert(pack.read_speed == (float) -1.903375E38F);
                Debug.Assert(pack.status == (byte)(byte)239);
                Debug.Assert(pack.storage_count == (byte)(byte)252);
                Debug.Assert(pack.total_capacity == (float) -1.7260738E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1395100615U);
                Debug.Assert(pack.write_speed == (float) -1.7975855E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)63);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.status = (byte)(byte)239;
            p261.storage_count = (byte)(byte)252;
            p261.storage_id = (byte)(byte)63;
            p261.write_speed = (float) -1.7975855E38F;
            p261.available_capacity = (float) -1.8205665E38F;
            p261.total_capacity = (float) -1.7260738E38F;
            p261.read_speed = (float) -1.903375E38F;
            p261.time_boot_ms = (uint)1395100615U;
            p261.used_capacity = (float) -4.743166E37F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2332253933U);
                Debug.Assert(pack.video_status == (byte)(byte)48);
                Debug.Assert(pack.available_capacity == (float)3.9308684E37F);
                Debug.Assert(pack.recording_time_ms == (uint)2112517237U);
                Debug.Assert(pack.image_status == (byte)(byte)151);
                Debug.Assert(pack.image_interval == (float)2.8873306E38F);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)48;
            p262.recording_time_ms = (uint)2112517237U;
            p262.time_boot_ms = (uint)2332253933U;
            p262.available_capacity = (float)3.9308684E37F;
            p262.image_interval = (float)2.8873306E38F;
            p262.image_status = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)303745899834353647L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7940467E38F, 9.715863E37F, -1.5797999E38F, 5.9945534E37F}));
                Debug.Assert(pack.image_index == (int)36720668);
                Debug.Assert(pack.relative_alt == (int) -228051020);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)65);
                Debug.Assert(pack.file_url_LEN(ph) == 34);
                Debug.Assert(pack.file_url_TRY(ph).Equals("bnmEkzhttpuerjwqbmvtiqgpphwfubogxf"));
                Debug.Assert(pack.lon == (int) -661666574);
                Debug.Assert(pack.lat == (int) -1689732302);
                Debug.Assert(pack.camera_id == (byte)(byte)181);
                Debug.Assert(pack.time_boot_ms == (uint)941953735U);
                Debug.Assert(pack.alt == (int)256022512);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.camera_id = (byte)(byte)181;
            p263.lon = (int) -661666574;
            p263.lat = (int) -1689732302;
            p263.time_utc = (ulong)303745899834353647L;
            p263.image_index = (int)36720668;
            p263.relative_alt = (int) -228051020;
            p263.time_boot_ms = (uint)941953735U;
            p263.file_url_SET("bnmEkzhttpuerjwqbmvtiqgpphwfubogxf", PH) ;
            p263.capture_result = (sbyte)(sbyte)65;
            p263.alt = (int)256022512;
            p263.q_SET(new float[] {-1.7940467E38F, 9.715863E37F, -1.5797999E38F, 5.9945534E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)7167202401308883063L);
                Debug.Assert(pack.flight_uuid == (ulong)4388222240045123601L);
                Debug.Assert(pack.time_boot_ms == (uint)2965435445U);
                Debug.Assert(pack.arming_time_utc == (ulong)1169599378851139882L);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2965435445U;
            p264.flight_uuid = (ulong)4388222240045123601L;
            p264.arming_time_utc = (ulong)1169599378851139882L;
            p264.takeoff_time_utc = (ulong)7167202401308883063L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.3349748E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1628845888U);
                Debug.Assert(pack.yaw == (float)3.3331226E38F);
                Debug.Assert(pack.pitch == (float) -2.7313372E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -2.7313372E38F;
            p265.yaw = (float)3.3331226E38F;
            p265.time_boot_ms = (uint)1628845888U;
            p265.roll = (float) -1.3349748E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)63);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)36, (byte)80, (byte)188, (byte)43, (byte)116, (byte)190, (byte)242, (byte)63, (byte)241, (byte)58, (byte)92, (byte)230, (byte)72, (byte)237, (byte)132, (byte)202, (byte)214, (byte)94, (byte)246, (byte)89, (byte)172, (byte)240, (byte)146, (byte)40, (byte)99, (byte)230, (byte)204, (byte)190, (byte)243, (byte)84, (byte)152, (byte)146, (byte)192, (byte)82, (byte)48, (byte)30, (byte)238, (byte)174, (byte)6, (byte)120, (byte)203, (byte)19, (byte)18, (byte)242, (byte)39, (byte)153, (byte)120, (byte)69, (byte)253, (byte)209, (byte)158, (byte)163, (byte)222, (byte)61, (byte)35, (byte)7, (byte)19, (byte)248, (byte)254, (byte)51, (byte)104, (byte)141, (byte)212, (byte)23, (byte)69, (byte)132, (byte)122, (byte)252, (byte)148, (byte)116, (byte)39, (byte)116, (byte)66, (byte)85, (byte)44, (byte)203, (byte)255, (byte)208, (byte)90, (byte)121, (byte)10, (byte)142, (byte)193, (byte)222, (byte)169, (byte)70, (byte)156, (byte)255, (byte)242, (byte)60, (byte)92, (byte)135, (byte)139, (byte)249, (byte)159, (byte)247, (byte)123, (byte)245, (byte)45, (byte)183, (byte)61, (byte)132, (byte)28, (byte)127, (byte)10, (byte)84, (byte)206, (byte)0, (byte)196, (byte)49, (byte)62, (byte)144, (byte)247, (byte)140, (byte)174, (byte)85, (byte)121, (byte)41, (byte)40, (byte)34, (byte)38, (byte)216, (byte)119, (byte)49, (byte)203, (byte)78, (byte)76, (byte)15, (byte)248, (byte)215, (byte)62, (byte)9, (byte)123, (byte)72, (byte)175, (byte)111, (byte)213, (byte)200, (byte)217, (byte)182, (byte)115, (byte)220, (byte)172, (byte)76, (byte)61, (byte)230, (byte)192, (byte)184, (byte)193, (byte)168, (byte)103, (byte)122, (byte)239, (byte)241, (byte)75, (byte)154, (byte)204, (byte)11, (byte)107, (byte)10, (byte)31, (byte)42, (byte)84, (byte)137, (byte)10, (byte)54, (byte)165, (byte)52, (byte)31, (byte)46, (byte)189, (byte)225, (byte)202, (byte)105, (byte)92, (byte)235, (byte)201, (byte)250, (byte)62, (byte)105, (byte)100, (byte)31, (byte)228, (byte)188, (byte)184, (byte)186, (byte)224, (byte)116, (byte)228, (byte)91, (byte)188, (byte)193, (byte)166, (byte)182, (byte)102, (byte)127, (byte)251, (byte)211, (byte)151, (byte)135, (byte)31, (byte)195, (byte)77, (byte)197, (byte)74, (byte)236, (byte)17, (byte)175, (byte)102, (byte)141, (byte)62, (byte)55, (byte)0, (byte)19, (byte)3, (byte)138, (byte)163, (byte)139, (byte)77, (byte)161, (byte)204, (byte)226, (byte)50, (byte)56, (byte)110, (byte)176, (byte)201, (byte)246, (byte)244, (byte)49, (byte)153, (byte)67, (byte)125, (byte)27, (byte)139, (byte)240, (byte)5, (byte)82, (byte)20, (byte)153, (byte)213, (byte)7, (byte)122, (byte)206, (byte)218, (byte)8, (byte)74, (byte)244, (byte)10}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)4);
                Debug.Assert(pack.sequence == (ushort)(ushort)11022);
                Debug.Assert(pack.target_system == (byte)(byte)167);
                Debug.Assert(pack.target_component == (byte)(byte)169);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)11022;
            p266.data__SET(new byte[] {(byte)36, (byte)80, (byte)188, (byte)43, (byte)116, (byte)190, (byte)242, (byte)63, (byte)241, (byte)58, (byte)92, (byte)230, (byte)72, (byte)237, (byte)132, (byte)202, (byte)214, (byte)94, (byte)246, (byte)89, (byte)172, (byte)240, (byte)146, (byte)40, (byte)99, (byte)230, (byte)204, (byte)190, (byte)243, (byte)84, (byte)152, (byte)146, (byte)192, (byte)82, (byte)48, (byte)30, (byte)238, (byte)174, (byte)6, (byte)120, (byte)203, (byte)19, (byte)18, (byte)242, (byte)39, (byte)153, (byte)120, (byte)69, (byte)253, (byte)209, (byte)158, (byte)163, (byte)222, (byte)61, (byte)35, (byte)7, (byte)19, (byte)248, (byte)254, (byte)51, (byte)104, (byte)141, (byte)212, (byte)23, (byte)69, (byte)132, (byte)122, (byte)252, (byte)148, (byte)116, (byte)39, (byte)116, (byte)66, (byte)85, (byte)44, (byte)203, (byte)255, (byte)208, (byte)90, (byte)121, (byte)10, (byte)142, (byte)193, (byte)222, (byte)169, (byte)70, (byte)156, (byte)255, (byte)242, (byte)60, (byte)92, (byte)135, (byte)139, (byte)249, (byte)159, (byte)247, (byte)123, (byte)245, (byte)45, (byte)183, (byte)61, (byte)132, (byte)28, (byte)127, (byte)10, (byte)84, (byte)206, (byte)0, (byte)196, (byte)49, (byte)62, (byte)144, (byte)247, (byte)140, (byte)174, (byte)85, (byte)121, (byte)41, (byte)40, (byte)34, (byte)38, (byte)216, (byte)119, (byte)49, (byte)203, (byte)78, (byte)76, (byte)15, (byte)248, (byte)215, (byte)62, (byte)9, (byte)123, (byte)72, (byte)175, (byte)111, (byte)213, (byte)200, (byte)217, (byte)182, (byte)115, (byte)220, (byte)172, (byte)76, (byte)61, (byte)230, (byte)192, (byte)184, (byte)193, (byte)168, (byte)103, (byte)122, (byte)239, (byte)241, (byte)75, (byte)154, (byte)204, (byte)11, (byte)107, (byte)10, (byte)31, (byte)42, (byte)84, (byte)137, (byte)10, (byte)54, (byte)165, (byte)52, (byte)31, (byte)46, (byte)189, (byte)225, (byte)202, (byte)105, (byte)92, (byte)235, (byte)201, (byte)250, (byte)62, (byte)105, (byte)100, (byte)31, (byte)228, (byte)188, (byte)184, (byte)186, (byte)224, (byte)116, (byte)228, (byte)91, (byte)188, (byte)193, (byte)166, (byte)182, (byte)102, (byte)127, (byte)251, (byte)211, (byte)151, (byte)135, (byte)31, (byte)195, (byte)77, (byte)197, (byte)74, (byte)236, (byte)17, (byte)175, (byte)102, (byte)141, (byte)62, (byte)55, (byte)0, (byte)19, (byte)3, (byte)138, (byte)163, (byte)139, (byte)77, (byte)161, (byte)204, (byte)226, (byte)50, (byte)56, (byte)110, (byte)176, (byte)201, (byte)246, (byte)244, (byte)49, (byte)153, (byte)67, (byte)125, (byte)27, (byte)139, (byte)240, (byte)5, (byte)82, (byte)20, (byte)153, (byte)213, (byte)7, (byte)122, (byte)206, (byte)218, (byte)8, (byte)74, (byte)244, (byte)10}, 0) ;
            p266.target_component = (byte)(byte)169;
            p266.length = (byte)(byte)63;
            p266.target_system = (byte)(byte)167;
            p266.first_message_offset = (byte)(byte)4;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)200);
                Debug.Assert(pack.first_message_offset == (byte)(byte)206);
                Debug.Assert(pack.sequence == (ushort)(ushort)60195);
                Debug.Assert(pack.length == (byte)(byte)108);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)120, (byte)64, (byte)164, (byte)131, (byte)102, (byte)172, (byte)111, (byte)172, (byte)227, (byte)89, (byte)237, (byte)125, (byte)51, (byte)44, (byte)14, (byte)22, (byte)239, (byte)233, (byte)38, (byte)189, (byte)24, (byte)210, (byte)23, (byte)84, (byte)193, (byte)198, (byte)97, (byte)2, (byte)113, (byte)101, (byte)92, (byte)210, (byte)206, (byte)223, (byte)16, (byte)83, (byte)173, (byte)98, (byte)113, (byte)39, (byte)163, (byte)22, (byte)37, (byte)152, (byte)225, (byte)162, (byte)234, (byte)222, (byte)163, (byte)208, (byte)66, (byte)197, (byte)129, (byte)251, (byte)172, (byte)133, (byte)67, (byte)180, (byte)132, (byte)200, (byte)138, (byte)136, (byte)180, (byte)96, (byte)40, (byte)38, (byte)12, (byte)161, (byte)19, (byte)187, (byte)248, (byte)78, (byte)237, (byte)11, (byte)75, (byte)56, (byte)248, (byte)118, (byte)46, (byte)57, (byte)10, (byte)247, (byte)83, (byte)64, (byte)67, (byte)53, (byte)84, (byte)80, (byte)45, (byte)164, (byte)69, (byte)9, (byte)252, (byte)226, (byte)166, (byte)216, (byte)2, (byte)242, (byte)241, (byte)163, (byte)164, (byte)42, (byte)16, (byte)87, (byte)204, (byte)68, (byte)205, (byte)155, (byte)30, (byte)221, (byte)203, (byte)21, (byte)98, (byte)95, (byte)42, (byte)23, (byte)119, (byte)240, (byte)47, (byte)234, (byte)81, (byte)254, (byte)121, (byte)143, (byte)152, (byte)163, (byte)23, (byte)242, (byte)69, (byte)222, (byte)211, (byte)137, (byte)47, (byte)16, (byte)243, (byte)173, (byte)95, (byte)250, (byte)166, (byte)51, (byte)87, (byte)251, (byte)245, (byte)94, (byte)209, (byte)90, (byte)82, (byte)44, (byte)46, (byte)9, (byte)77, (byte)150, (byte)13, (byte)74, (byte)86, (byte)215, (byte)176, (byte)25, (byte)196, (byte)184, (byte)204, (byte)130, (byte)31, (byte)233, (byte)3, (byte)250, (byte)128, (byte)8, (byte)252, (byte)155, (byte)170, (byte)100, (byte)168, (byte)196, (byte)110, (byte)233, (byte)124, (byte)131, (byte)71, (byte)1, (byte)222, (byte)195, (byte)241, (byte)33, (byte)167, (byte)86, (byte)36, (byte)137, (byte)221, (byte)143, (byte)246, (byte)222, (byte)63, (byte)238, (byte)87, (byte)152, (byte)152, (byte)6, (byte)239, (byte)67, (byte)128, (byte)248, (byte)214, (byte)128, (byte)200, (byte)212, (byte)209, (byte)210, (byte)31, (byte)153, (byte)116, (byte)16, (byte)132, (byte)131, (byte)186, (byte)239, (byte)149, (byte)240, (byte)108, (byte)218, (byte)83, (byte)221, (byte)145, (byte)90, (byte)34, (byte)27, (byte)164, (byte)41, (byte)132, (byte)88, (byte)226, (byte)209, (byte)13, (byte)34, (byte)104, (byte)159, (byte)142, (byte)172, (byte)204, (byte)28, (byte)57, (byte)107, (byte)45, (byte)235, (byte)241, (byte)208, (byte)62, (byte)163, (byte)94}));
                Debug.Assert(pack.target_system == (byte)(byte)198);
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)60195;
            p267.length = (byte)(byte)108;
            p267.data__SET(new byte[] {(byte)120, (byte)64, (byte)164, (byte)131, (byte)102, (byte)172, (byte)111, (byte)172, (byte)227, (byte)89, (byte)237, (byte)125, (byte)51, (byte)44, (byte)14, (byte)22, (byte)239, (byte)233, (byte)38, (byte)189, (byte)24, (byte)210, (byte)23, (byte)84, (byte)193, (byte)198, (byte)97, (byte)2, (byte)113, (byte)101, (byte)92, (byte)210, (byte)206, (byte)223, (byte)16, (byte)83, (byte)173, (byte)98, (byte)113, (byte)39, (byte)163, (byte)22, (byte)37, (byte)152, (byte)225, (byte)162, (byte)234, (byte)222, (byte)163, (byte)208, (byte)66, (byte)197, (byte)129, (byte)251, (byte)172, (byte)133, (byte)67, (byte)180, (byte)132, (byte)200, (byte)138, (byte)136, (byte)180, (byte)96, (byte)40, (byte)38, (byte)12, (byte)161, (byte)19, (byte)187, (byte)248, (byte)78, (byte)237, (byte)11, (byte)75, (byte)56, (byte)248, (byte)118, (byte)46, (byte)57, (byte)10, (byte)247, (byte)83, (byte)64, (byte)67, (byte)53, (byte)84, (byte)80, (byte)45, (byte)164, (byte)69, (byte)9, (byte)252, (byte)226, (byte)166, (byte)216, (byte)2, (byte)242, (byte)241, (byte)163, (byte)164, (byte)42, (byte)16, (byte)87, (byte)204, (byte)68, (byte)205, (byte)155, (byte)30, (byte)221, (byte)203, (byte)21, (byte)98, (byte)95, (byte)42, (byte)23, (byte)119, (byte)240, (byte)47, (byte)234, (byte)81, (byte)254, (byte)121, (byte)143, (byte)152, (byte)163, (byte)23, (byte)242, (byte)69, (byte)222, (byte)211, (byte)137, (byte)47, (byte)16, (byte)243, (byte)173, (byte)95, (byte)250, (byte)166, (byte)51, (byte)87, (byte)251, (byte)245, (byte)94, (byte)209, (byte)90, (byte)82, (byte)44, (byte)46, (byte)9, (byte)77, (byte)150, (byte)13, (byte)74, (byte)86, (byte)215, (byte)176, (byte)25, (byte)196, (byte)184, (byte)204, (byte)130, (byte)31, (byte)233, (byte)3, (byte)250, (byte)128, (byte)8, (byte)252, (byte)155, (byte)170, (byte)100, (byte)168, (byte)196, (byte)110, (byte)233, (byte)124, (byte)131, (byte)71, (byte)1, (byte)222, (byte)195, (byte)241, (byte)33, (byte)167, (byte)86, (byte)36, (byte)137, (byte)221, (byte)143, (byte)246, (byte)222, (byte)63, (byte)238, (byte)87, (byte)152, (byte)152, (byte)6, (byte)239, (byte)67, (byte)128, (byte)248, (byte)214, (byte)128, (byte)200, (byte)212, (byte)209, (byte)210, (byte)31, (byte)153, (byte)116, (byte)16, (byte)132, (byte)131, (byte)186, (byte)239, (byte)149, (byte)240, (byte)108, (byte)218, (byte)83, (byte)221, (byte)145, (byte)90, (byte)34, (byte)27, (byte)164, (byte)41, (byte)132, (byte)88, (byte)226, (byte)209, (byte)13, (byte)34, (byte)104, (byte)159, (byte)142, (byte)172, (byte)204, (byte)28, (byte)57, (byte)107, (byte)45, (byte)235, (byte)241, (byte)208, (byte)62, (byte)163, (byte)94}, 0) ;
            p267.target_system = (byte)(byte)198;
            p267.target_component = (byte)(byte)200;
            p267.first_message_offset = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.target_component == (byte)(byte)238);
                Debug.Assert(pack.sequence == (ushort)(ushort)16804);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)158;
            p268.sequence = (ushort)(ushort)16804;
            p268.target_component = (byte)(byte)238;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)47777);
                Debug.Assert(pack.bitrate == (uint)3816193345U);
                Debug.Assert(pack.uri_LEN(ph) == 129);
                Debug.Assert(pack.uri_TRY(ph).Equals("nXhnAwjnjojoyakcmrpfjsgkkrbzoayvafqdrfkfjqyzvghillophkFhOolggcqfDwAylsckbPkttYwmrshybquceawzghxuzzywayfqmefnJxzvbyoznmofYecsqivle"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)3730);
                Debug.Assert(pack.camera_id == (byte)(byte)235);
                Debug.Assert(pack.framerate == (float)7.3266016E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)12200);
                Debug.Assert(pack.status == (byte)(byte)167);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)47777;
            p269.rotation = (ushort)(ushort)12200;
            p269.bitrate = (uint)3816193345U;
            p269.resolution_v = (ushort)(ushort)3730;
            p269.framerate = (float)7.3266016E37F;
            p269.uri_SET("nXhnAwjnjojoyakcmrpfjsgkkrbzoayvafqdrfkfjqyzvghillophkFhOolggcqfDwAylsckbPkttYwmrshybquceawzghxuzzywayfqmefnJxzvbyoznmofYecsqivle", PH) ;
            p269.status = (byte)(byte)167;
            p269.camera_id = (byte)(byte)235;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rotation == (ushort)(ushort)56590);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)43129);
                Debug.Assert(pack.framerate == (float) -5.7723535E37F);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.bitrate == (uint)1531081192U);
                Debug.Assert(pack.uri_LEN(ph) == 70);
                Debug.Assert(pack.uri_TRY(ph).Equals("arvtNhgLcbgyrBigjnahjtmnherdmslbtXzkIvsgingoxpzoiwbzwojdlxludvmgspxcgs"));
                Debug.Assert(pack.camera_id == (byte)(byte)178);
                Debug.Assert(pack.target_component == (byte)(byte)9);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)55418);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.bitrate = (uint)1531081192U;
            p270.uri_SET("arvtNhgLcbgyrBigjnahjtmnherdmslbtXzkIvsgingoxpzoiwbzwojdlxludvmgspxcgs", PH) ;
            p270.resolution_v = (ushort)(ushort)55418;
            p270.target_component = (byte)(byte)9;
            p270.camera_id = (byte)(byte)178;
            p270.framerate = (float) -5.7723535E37F;
            p270.rotation = (ushort)(ushort)56590;
            p270.target_system = (byte)(byte)135;
            p270.resolution_h = (ushort)(ushort)43129;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 17);
                Debug.Assert(pack.ssid_TRY(ph).Equals("bldwmhtwrgyEkayid"));
                Debug.Assert(pack.password_LEN(ph) == 61);
                Debug.Assert(pack.password_TRY(ph).Equals("hlkeOcGXpzcayAiBcbfviqsvPfPfRRiuMocfwbxwjJnkchfibrjzstaiqyssm"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("bldwmhtwrgyEkayid", PH) ;
            p299.password_SET("hlkeOcGXpzcayAiBcbfviqsvPfPfRRiuMocfwbxwjJnkchfibrjzstaiqyssm", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)53167);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)160, (byte)176, (byte)18, (byte)248, (byte)194, (byte)40, (byte)63, (byte)214}));
                Debug.Assert(pack.version == (ushort)(ushort)63582);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)37, (byte)116, (byte)27, (byte)164, (byte)82, (byte)139, (byte)131, (byte)216}));
                Debug.Assert(pack.min_version == (ushort)(ushort)25279);
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)160, (byte)176, (byte)18, (byte)248, (byte)194, (byte)40, (byte)63, (byte)214}, 0) ;
            p300.version = (ushort)(ushort)63582;
            p300.max_version = (ushort)(ushort)53167;
            p300.min_version = (ushort)(ushort)25279;
            p300.spec_version_hash_SET(new byte[] {(byte)37, (byte)116, (byte)27, (byte)164, (byte)82, (byte)139, (byte)131, (byte)216}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5099442245606314216L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.sub_mode == (byte)(byte)190);
                Debug.Assert(pack.uptime_sec == (uint)1383938964U);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)60295);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)60295;
            p310.uptime_sec = (uint)1383938964U;
            p310.time_usec = (ulong)5099442245606314216L;
            p310.sub_mode = (byte)(byte)190;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)2701056863U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)234, (byte)134, (byte)52, (byte)8, (byte)36, (byte)247, (byte)239, (byte)93, (byte)231, (byte)81, (byte)221, (byte)45, (byte)121, (byte)102, (byte)11, (byte)42}));
                Debug.Assert(pack.time_usec == (ulong)6673235532207954532L);
                Debug.Assert(pack.uptime_sec == (uint)366570272U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)138);
                Debug.Assert(pack.hw_version_major == (byte)(byte)65);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)219);
                Debug.Assert(pack.name_LEN(ph) == 55);
                Debug.Assert(pack.name_TRY(ph).Equals("vnwdtvgRHfaynarimhPymiftldwqiwfbHpgeglpjaPPbhbkjbemaVoW"));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)33);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)219;
            p311.sw_version_major = (byte)(byte)138;
            p311.name_SET("vnwdtvgRHfaynarimhPymiftldwqiwfbHpgeglpjaPPbhbkjbemaVoW", PH) ;
            p311.hw_version_major = (byte)(byte)65;
            p311.sw_vcs_commit = (uint)2701056863U;
            p311.uptime_sec = (uint)366570272U;
            p311.hw_unique_id_SET(new byte[] {(byte)234, (byte)134, (byte)52, (byte)8, (byte)36, (byte)247, (byte)239, (byte)93, (byte)231, (byte)81, (byte)221, (byte)45, (byte)121, (byte)102, (byte)11, (byte)42}, 0) ;
            p311.time_usec = (ulong)6673235532207954532L;
            p311.sw_version_minor = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -16453);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("u"));
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.target_component == (byte)(byte)201);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("u", PH) ;
            p320.param_index = (short)(short) -16453;
            p320.target_system = (byte)(byte)108;
            p320.target_component = (byte)(byte)201;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.target_system == (byte)(byte)8);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)179;
            p321.target_system = (byte)(byte)8;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("FzByakSnWv"));
                Debug.Assert(pack.param_index == (ushort)(ushort)26955);
                Debug.Assert(pack.param_value_LEN(ph) == 23);
                Debug.Assert(pack.param_value_TRY(ph).Equals("Lntxvzdtzpihfeqyvqpgdni"));
                Debug.Assert(pack.param_count == (ushort)(ushort)31244);
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)31244;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_id_SET("FzByakSnWv", PH) ;
            p322.param_index = (ushort)(ushort)26955;
            p322.param_value_SET("Lntxvzdtzpihfeqyvqpgdni", PH) ;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("aRdfexwbwszezjj"));
                Debug.Assert(pack.param_value_LEN(ph) == 1);
                Debug.Assert(pack.param_value_TRY(ph).Equals("r"));
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("aRdfexwbwszezjj", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.target_component = (byte)(byte)27;
            p323.param_value_SET("r", PH) ;
            p323.target_system = (byte)(byte)197;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hpnD"));
                Debug.Assert(pack.param_value_LEN(ph) == 122);
                Debug.Assert(pack.param_value_TRY(ph).Equals("bdibcwKngrmzdbziakgyYVUOpzkzxalynooivihmqdviqyppglflnojdoxfapyinjckfjpywhukqalsaoykcyxuyxmunmhmvdbuoRYupzzgkIihmnnytuhrxpB"));
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p324.param_value_SET("bdibcwKngrmzdbziakgyYVUOpzkzxalynooivihmqdviqyppglflnojdoxfapyinjckfjpywhukqalsaoykcyxuyxmunmhmvdbuoRYupzzgkIihmnnytuhrxpB", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("hpnD", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)18);
                Debug.Assert(pack.time_usec == (ulong)2569715573367992721L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)20277, (ushort)19216, (ushort)42109, (ushort)9254, (ushort)21042, (ushort)35053, (ushort)54030, (ushort)32809, (ushort)57587, (ushort)18324, (ushort)44747, (ushort)46576, (ushort)7173, (ushort)35304, (ushort)1536, (ushort)2164, (ushort)47328, (ushort)55492, (ushort)21478, (ushort)10625, (ushort)23825, (ushort)2042, (ushort)45672, (ushort)21442, (ushort)44441, (ushort)58823, (ushort)64171, (ushort)7225, (ushort)52470, (ushort)17738, (ushort)35392, (ushort)56370, (ushort)55217, (ushort)15982, (ushort)32007, (ushort)55976, (ushort)18245, (ushort)28066, (ushort)58577, (ushort)39173, (ushort)20169, (ushort)8320, (ushort)59126, (ushort)57509, (ushort)65287, (ushort)50006, (ushort)42050, (ushort)51336, (ushort)23714, (ushort)49092, (ushort)17314, (ushort)44516, (ushort)46450, (ushort)60900, (ushort)25822, (ushort)54826, (ushort)41884, (ushort)8037, (ushort)29908, (ushort)15067, (ushort)41125, (ushort)6180, (ushort)14365, (ushort)56139, (ushort)29266, (ushort)41401, (ushort)2822, (ushort)10957, (ushort)53416, (ushort)21315, (ushort)40668, (ushort)62853}));
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55850);
                Debug.Assert(pack.min_distance == (ushort)(ushort)34970);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2569715573367992721L;
            p330.min_distance = (ushort)(ushort)34970;
            p330.max_distance = (ushort)(ushort)55850;
            p330.increment = (byte)(byte)18;
            p330.distances_SET(new ushort[] {(ushort)20277, (ushort)19216, (ushort)42109, (ushort)9254, (ushort)21042, (ushort)35053, (ushort)54030, (ushort)32809, (ushort)57587, (ushort)18324, (ushort)44747, (ushort)46576, (ushort)7173, (ushort)35304, (ushort)1536, (ushort)2164, (ushort)47328, (ushort)55492, (ushort)21478, (ushort)10625, (ushort)23825, (ushort)2042, (ushort)45672, (ushort)21442, (ushort)44441, (ushort)58823, (ushort)64171, (ushort)7225, (ushort)52470, (ushort)17738, (ushort)35392, (ushort)56370, (ushort)55217, (ushort)15982, (ushort)32007, (ushort)55976, (ushort)18245, (ushort)28066, (ushort)58577, (ushort)39173, (ushort)20169, (ushort)8320, (ushort)59126, (ushort)57509, (ushort)65287, (ushort)50006, (ushort)42050, (ushort)51336, (ushort)23714, (ushort)49092, (ushort)17314, (ushort)44516, (ushort)46450, (ushort)60900, (ushort)25822, (ushort)54826, (ushort)41884, (ushort)8037, (ushort)29908, (ushort)15067, (ushort)41125, (ushort)6180, (ushort)14365, (ushort)56139, (ushort)29266, (ushort)41401, (ushort)2822, (ushort)10957, (ushort)53416, (ushort)21315, (ushort)40668, (ushort)62853}, 0) ;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
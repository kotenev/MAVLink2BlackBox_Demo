
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
                Debug.Assert(pack.custom_mode == (uint)836365276U);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
                Debug.Assert(pack.mavlink_version == (byte)(byte)191);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)836365276U;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED5;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)191;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)45761);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
                Debug.Assert(pack.current_battery == (short)(short)11621);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)51704);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)49127);
                Debug.Assert(pack.load == (ushort)(ushort)29225);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)96);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)16762);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)46698);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)38475);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)50163);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)46698;
            p1.current_battery = (short)(short)11621;
            p1.voltage_battery = (ushort)(ushort)45761;
            p1.battery_remaining = (sbyte)(sbyte)96;
            p1.drop_rate_comm = (ushort)(ushort)51704;
            p1.errors_count3 = (ushort)(ushort)38475;
            p1.errors_count4 = (ushort)(ushort)16762;
            p1.errors_count1 = (ushort)(ushort)50163;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.load = (ushort)(ushort)29225;
            p1.errors_count2 = (ushort)(ushort)49127;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3163791257719966634L);
                Debug.Assert(pack.time_boot_ms == (uint)3800320124U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3800320124U;
            p2.time_unix_usec = (ulong)3163791257719966634L;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -9.259601E37F);
                Debug.Assert(pack.z == (float)3.2787452E38F);
                Debug.Assert(pack.vy == (float)1.2548325E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2201739599U);
                Debug.Assert(pack.vx == (float)8.530673E37F);
                Debug.Assert(pack.yaw == (float)5.2720884E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.type_mask == (ushort)(ushort)1321);
                Debug.Assert(pack.afz == (float) -1.7245191E37F);
                Debug.Assert(pack.y == (float)3.3574907E38F);
                Debug.Assert(pack.afx == (float) -1.8970637E38F);
                Debug.Assert(pack.vz == (float)1.4973953E38F);
                Debug.Assert(pack.yaw_rate == (float)1.5375682E38F);
                Debug.Assert(pack.x == (float) -6.10778E37F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vx = (float)8.530673E37F;
            p3.yaw = (float)5.2720884E37F;
            p3.time_boot_ms = (uint)2201739599U;
            p3.afx = (float) -1.8970637E38F;
            p3.y = (float)3.3574907E38F;
            p3.afy = (float) -9.259601E37F;
            p3.z = (float)3.2787452E38F;
            p3.afz = (float) -1.7245191E37F;
            p3.vz = (float)1.4973953E38F;
            p3.vy = (float)1.2548325E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p3.type_mask = (ushort)(ushort)1321;
            p3.yaw_rate = (float)1.5375682E38F;
            p3.x = (float) -6.10778E37F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.time_usec == (ulong)4043356851747122366L);
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.seq == (uint)417904051U);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)4043356851747122366L;
            p4.target_component = (byte)(byte)36;
            p4.seq = (uint)417904051U;
            p4.target_system = (byte)(byte)99;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)134);
                Debug.Assert(pack.control_request == (byte)(byte)28);
                Debug.Assert(pack.passkey_LEN(ph) == 16);
                Debug.Assert(pack.passkey_TRY(ph).Equals("erbjxkjhxtvtStuh"));
                Debug.Assert(pack.target_system == (byte)(byte)16);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)28;
            p5.passkey_SET("erbjxkjhxtvtStuh", PH) ;
            p5.target_system = (byte)(byte)16;
            p5.version = (byte)(byte)134;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)109);
                Debug.Assert(pack.control_request == (byte)(byte)10);
                Debug.Assert(pack.ack == (byte)(byte)164);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)164;
            p6.gcs_system_id = (byte)(byte)109;
            p6.control_request = (byte)(byte)10;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 28);
                Debug.Assert(pack.key_TRY(ph).Equals("fooahXukhehdicrvysbokanzlExr"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("fooahXukhehdicrvysbokanzlExr", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.custom_mode == (uint)195357142U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            p11.custom_mode = (uint)195357142U;
            p11.target_system = (byte)(byte)54;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -25828);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bydib"));
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)138);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)138;
            p20.param_index = (short)(short) -25828;
            p20.target_component = (byte)(byte)149;
            p20.param_id_SET("bydib", PH) ;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.target_component == (byte)(byte)175);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)216;
            p21.target_component = (byte)(byte)175;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vtugmiQirKXHvbc"));
                Debug.Assert(pack.param_index == (ushort)(ushort)57003);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.param_count == (ushort)(ushort)2340);
                Debug.Assert(pack.param_value == (float)2.994411E37F);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)57003;
            p22.param_id_SET("vtugmiQirKXHvbc", PH) ;
            p22.param_value = (float)2.994411E37F;
            p22.param_count = (ushort)(ushort)2340;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)119);
                Debug.Assert(pack.target_system == (byte)(byte)106);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("cnnxznmaupz"));
                Debug.Assert(pack.param_value == (float) -2.5391642E38F);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p23.target_system = (byte)(byte)106;
            p23.param_value = (float) -2.5391642E38F;
            p23.target_component = (byte)(byte)119;
            p23.param_id_SET("cnnxznmaupz", PH) ;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)205306385);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)4242522067U);
                Debug.Assert(pack.epv == (ushort)(ushort)56367);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -690181833);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3299971597U);
                Debug.Assert(pack.lat == (int)317524549);
                Debug.Assert(pack.time_usec == (ulong)8546331082488941581L);
                Debug.Assert(pack.vel == (ushort)(ushort)63822);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)458260832U);
                Debug.Assert(pack.eph == (ushort)(ushort)27439);
                Debug.Assert(pack.cog == (ushort)(ushort)16456);
                Debug.Assert(pack.satellites_visible == (byte)(byte)107);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1862795120U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.alt == (int) -1576142308);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.vel = (ushort)(ushort)63822;
            p24.satellites_visible = (byte)(byte)107;
            p24.lon = (int)205306385;
            p24.eph = (ushort)(ushort)27439;
            p24.cog = (ushort)(ushort)16456;
            p24.h_acc_SET((uint)3299971597U, PH) ;
            p24.hdg_acc_SET((uint)4242522067U, PH) ;
            p24.time_usec = (ulong)8546331082488941581L;
            p24.alt_ellipsoid_SET((int) -690181833, PH) ;
            p24.alt = (int) -1576142308;
            p24.vel_acc_SET((uint)458260832U, PH) ;
            p24.epv = (ushort)(ushort)56367;
            p24.v_acc_SET((uint)1862795120U, PH) ;
            p24.lat = (int)317524549;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)99);
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)16, (byte)89, (byte)199, (byte)106, (byte)66, (byte)108, (byte)43, (byte)162, (byte)61, (byte)201, (byte)105, (byte)23, (byte)227, (byte)202, (byte)165, (byte)105, (byte)56, (byte)94, (byte)64, (byte)206}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)37, (byte)3, (byte)239, (byte)160, (byte)52, (byte)96, (byte)172, (byte)69, (byte)68, (byte)72, (byte)38, (byte)109, (byte)63, (byte)237, (byte)55, (byte)56, (byte)179, (byte)253, (byte)154, (byte)180}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)19, (byte)231, (byte)188, (byte)61, (byte)246, (byte)18, (byte)150, (byte)227, (byte)172, (byte)94, (byte)29, (byte)46, (byte)215, (byte)51, (byte)89, (byte)237, (byte)195, (byte)164, (byte)2, (byte)148}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)239, (byte)229, (byte)64, (byte)247, (byte)174, (byte)140, (byte)31, (byte)220, (byte)134, (byte)92, (byte)160, (byte)76, (byte)163, (byte)0, (byte)2, (byte)185, (byte)188, (byte)150, (byte)195, (byte)124}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)127, (byte)104, (byte)243, (byte)232, (byte)129, (byte)230, (byte)101, (byte)152, (byte)181, (byte)231, (byte)34, (byte)215, (byte)145, (byte)110, (byte)84, (byte)167, (byte)52, (byte)39, (byte)35, (byte)95}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)99;
            p25.satellite_elevation_SET(new byte[] {(byte)127, (byte)104, (byte)243, (byte)232, (byte)129, (byte)230, (byte)101, (byte)152, (byte)181, (byte)231, (byte)34, (byte)215, (byte)145, (byte)110, (byte)84, (byte)167, (byte)52, (byte)39, (byte)35, (byte)95}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)37, (byte)3, (byte)239, (byte)160, (byte)52, (byte)96, (byte)172, (byte)69, (byte)68, (byte)72, (byte)38, (byte)109, (byte)63, (byte)237, (byte)55, (byte)56, (byte)179, (byte)253, (byte)154, (byte)180}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)19, (byte)231, (byte)188, (byte)61, (byte)246, (byte)18, (byte)150, (byte)227, (byte)172, (byte)94, (byte)29, (byte)46, (byte)215, (byte)51, (byte)89, (byte)237, (byte)195, (byte)164, (byte)2, (byte)148}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)16, (byte)89, (byte)199, (byte)106, (byte)66, (byte)108, (byte)43, (byte)162, (byte)61, (byte)201, (byte)105, (byte)23, (byte)227, (byte)202, (byte)165, (byte)105, (byte)56, (byte)94, (byte)64, (byte)206}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)239, (byte)229, (byte)64, (byte)247, (byte)174, (byte)140, (byte)31, (byte)220, (byte)134, (byte)92, (byte)160, (byte)76, (byte)163, (byte)0, (byte)2, (byte)185, (byte)188, (byte)150, (byte)195, (byte)124}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)12310);
                Debug.Assert(pack.xgyro == (short)(short) -25752);
                Debug.Assert(pack.zmag == (short)(short) -22586);
                Debug.Assert(pack.yacc == (short)(short)26100);
                Debug.Assert(pack.zacc == (short)(short) -20301);
                Debug.Assert(pack.ymag == (short)(short)32613);
                Debug.Assert(pack.xmag == (short)(short)15667);
                Debug.Assert(pack.zgyro == (short)(short)25770);
                Debug.Assert(pack.time_boot_ms == (uint)1510942421U);
                Debug.Assert(pack.xacc == (short)(short)20978);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)1510942421U;
            p26.ygyro = (short)(short)12310;
            p26.xgyro = (short)(short) -25752;
            p26.xacc = (short)(short)20978;
            p26.zmag = (short)(short) -22586;
            p26.zgyro = (short)(short)25770;
            p26.xmag = (short)(short)15667;
            p26.ymag = (short)(short)32613;
            p26.zacc = (short)(short) -20301;
            p26.yacc = (short)(short)26100;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)30269);
                Debug.Assert(pack.zacc == (short)(short) -26438);
                Debug.Assert(pack.time_usec == (ulong)3095762405701749087L);
                Debug.Assert(pack.zmag == (short)(short)18263);
                Debug.Assert(pack.zgyro == (short)(short) -32205);
                Debug.Assert(pack.xmag == (short)(short)15066);
                Debug.Assert(pack.ygyro == (short)(short) -3440);
                Debug.Assert(pack.yacc == (short)(short)25314);
                Debug.Assert(pack.ymag == (short)(short) -12708);
                Debug.Assert(pack.xgyro == (short)(short) -6350);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short)15066;
            p27.zgyro = (short)(short) -32205;
            p27.ymag = (short)(short) -12708;
            p27.ygyro = (short)(short) -3440;
            p27.xgyro = (short)(short) -6350;
            p27.zmag = (short)(short)18263;
            p27.zacc = (short)(short) -26438;
            p27.yacc = (short)(short)25314;
            p27.time_usec = (ulong)3095762405701749087L;
            p27.xacc = (short)(short)30269;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (short)(short) -17744);
                Debug.Assert(pack.time_usec == (ulong)2687263879617360063L);
                Debug.Assert(pack.press_diff2 == (short)(short) -18954);
                Debug.Assert(pack.press_diff1 == (short)(short)30486);
                Debug.Assert(pack.temperature == (short)(short) -6440);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short) -18954;
            p28.press_diff1 = (short)(short)30486;
            p28.press_abs = (short)(short) -17744;
            p28.time_usec = (ulong)2687263879617360063L;
            p28.temperature = (short)(short) -6440;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.9037608E38F);
                Debug.Assert(pack.press_diff == (float)1.4232793E38F);
                Debug.Assert(pack.temperature == (short)(short)31773);
                Debug.Assert(pack.time_boot_ms == (uint)314324361U);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)31773;
            p29.press_diff = (float)1.4232793E38F;
            p29.press_abs = (float) -2.9037608E38F;
            p29.time_boot_ms = (uint)314324361U;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)9.789581E37F);
                Debug.Assert(pack.rollspeed == (float)2.9571465E38F);
                Debug.Assert(pack.yawspeed == (float)3.1581588E38F);
                Debug.Assert(pack.pitchspeed == (float)1.939982E38F);
                Debug.Assert(pack.yaw == (float) -1.6340934E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3407826702U);
                Debug.Assert(pack.roll == (float) -2.3466695E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)3407826702U;
            p30.rollspeed = (float)2.9571465E38F;
            p30.yaw = (float) -1.6340934E38F;
            p30.roll = (float) -2.3466695E38F;
            p30.pitch = (float)9.789581E37F;
            p30.pitchspeed = (float)1.939982E38F;
            p30.yawspeed = (float)3.1581588E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float) -2.6924517E38F);
                Debug.Assert(pack.yawspeed == (float)3.1361698E38F);
                Debug.Assert(pack.q4 == (float)2.876202E38F);
                Debug.Assert(pack.q1 == (float) -3.133023E38F);
                Debug.Assert(pack.pitchspeed == (float)1.0752487E38F);
                Debug.Assert(pack.rollspeed == (float)4.77098E37F);
                Debug.Assert(pack.time_boot_ms == (uint)572490474U);
                Debug.Assert(pack.q3 == (float) -6.793343E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float)1.0752487E38F;
            p31.yawspeed = (float)3.1361698E38F;
            p31.q2 = (float) -2.6924517E38F;
            p31.q3 = (float) -6.793343E37F;
            p31.q4 = (float)2.876202E38F;
            p31.q1 = (float) -3.133023E38F;
            p31.rollspeed = (float)4.77098E37F;
            p31.time_boot_ms = (uint)572490474U;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -1.9646118E38F);
                Debug.Assert(pack.vy == (float) -2.7111223E37F);
                Debug.Assert(pack.y == (float) -1.8856674E38F);
                Debug.Assert(pack.x == (float)2.4806235E38F);
                Debug.Assert(pack.vx == (float)2.8599043E38F);
                Debug.Assert(pack.z == (float) -9.04593E37F);
                Debug.Assert(pack.time_boot_ms == (uint)639803333U);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.x = (float)2.4806235E38F;
            p32.vy = (float) -2.7111223E37F;
            p32.z = (float) -9.04593E37F;
            p32.time_boot_ms = (uint)639803333U;
            p32.vx = (float)2.8599043E38F;
            p32.y = (float) -1.8856674E38F;
            p32.vz = (float) -1.9646118E38F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)561063820U);
                Debug.Assert(pack.vy == (short)(short) -24727);
                Debug.Assert(pack.vz == (short)(short) -9011);
                Debug.Assert(pack.alt == (int)28536530);
                Debug.Assert(pack.vx == (short)(short)30209);
                Debug.Assert(pack.lon == (int)155040235);
                Debug.Assert(pack.lat == (int)1156049066);
                Debug.Assert(pack.relative_alt == (int)1003443111);
                Debug.Assert(pack.hdg == (ushort)(ushort)56373);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.alt = (int)28536530;
            p33.lat = (int)1156049066;
            p33.relative_alt = (int)1003443111;
            p33.vy = (short)(short) -24727;
            p33.vx = (short)(short)30209;
            p33.lon = (int)155040235;
            p33.time_boot_ms = (uint)561063820U;
            p33.vz = (short)(short) -9011;
            p33.hdg = (ushort)(ushort)56373;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_scaled == (short)(short)5332);
                Debug.Assert(pack.chan8_scaled == (short)(short)8526);
                Debug.Assert(pack.chan6_scaled == (short)(short) -3843);
                Debug.Assert(pack.rssi == (byte)(byte)210);
                Debug.Assert(pack.chan1_scaled == (short)(short) -18248);
                Debug.Assert(pack.chan5_scaled == (short)(short)20848);
                Debug.Assert(pack.chan3_scaled == (short)(short) -10381);
                Debug.Assert(pack.port == (byte)(byte)137);
                Debug.Assert(pack.time_boot_ms == (uint)3371359270U);
                Debug.Assert(pack.chan4_scaled == (short)(short)29757);
                Debug.Assert(pack.chan2_scaled == (short)(short)1950);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)210;
            p34.chan8_scaled = (short)(short)8526;
            p34.time_boot_ms = (uint)3371359270U;
            p34.chan4_scaled = (short)(short)29757;
            p34.chan7_scaled = (short)(short)5332;
            p34.chan3_scaled = (short)(short) -10381;
            p34.port = (byte)(byte)137;
            p34.chan5_scaled = (short)(short)20848;
            p34.chan6_scaled = (short)(short) -3843;
            p34.chan2_scaled = (short)(short)1950;
            p34.chan1_scaled = (short)(short) -18248;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)63960);
                Debug.Assert(pack.rssi == (byte)(byte)143);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)37600);
                Debug.Assert(pack.time_boot_ms == (uint)1665011932U);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)11081);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)17222);
                Debug.Assert(pack.port == (byte)(byte)170);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)56294);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)33090);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)26998);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)12967);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.port = (byte)(byte)170;
            p35.rssi = (byte)(byte)143;
            p35.chan3_raw = (ushort)(ushort)11081;
            p35.chan8_raw = (ushort)(ushort)33090;
            p35.chan2_raw = (ushort)(ushort)37600;
            p35.chan5_raw = (ushort)(ushort)56294;
            p35.chan4_raw = (ushort)(ushort)26998;
            p35.chan6_raw = (ushort)(ushort)63960;
            p35.chan1_raw = (ushort)(ushort)12967;
            p35.chan7_raw = (ushort)(ushort)17222;
            p35.time_boot_ms = (uint)1665011932U;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)4579);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)46679);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)819);
                Debug.Assert(pack.port == (byte)(byte)36);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)27091);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)13494);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)37232);
                Debug.Assert(pack.time_usec == (uint)3650016669U);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)62080);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)16110);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)57246);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)3243);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)58218);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)8186);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)5616);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)47874);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)35113);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)41747);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo12_raw_SET((ushort)(ushort)5616, PH) ;
            p36.servo1_raw = (ushort)(ushort)4579;
            p36.servo15_raw_SET((ushort)(ushort)37232, PH) ;
            p36.port = (byte)(byte)36;
            p36.servo8_raw = (ushort)(ushort)27091;
            p36.servo7_raw = (ushort)(ushort)13494;
            p36.servo14_raw_SET((ushort)(ushort)8186, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)46679, PH) ;
            p36.servo3_raw = (ushort)(ushort)62080;
            p36.time_usec = (uint)3650016669U;
            p36.servo9_raw_SET((ushort)(ushort)57246, PH) ;
            p36.servo6_raw = (ushort)(ushort)35113;
            p36.servo13_raw_SET((ushort)(ushort)3243, PH) ;
            p36.servo4_raw = (ushort)(ushort)41747;
            p36.servo2_raw = (ushort)(ushort)47874;
            p36.servo10_raw_SET((ushort)(ushort)58218, PH) ;
            p36.servo5_raw = (ushort)(ushort)819;
            p36.servo11_raw_SET((ushort)(ushort)16110, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)20216);
                Debug.Assert(pack.target_component == (byte)(byte)190);
                Debug.Assert(pack.start_index == (short)(short) -20196);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)41);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.target_component = (byte)(byte)190;
            p37.start_index = (short)(short) -20196;
            p37.target_system = (byte)(byte)41;
            p37.end_index = (short)(short)20216;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.start_index == (short)(short)11497);
                Debug.Assert(pack.end_index == (short)(short) -4980);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -4980;
            p38.target_component = (byte)(byte)174;
            p38.start_index = (short)(short)11497;
            p38.target_system = (byte)(byte)173;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.3371075E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)232);
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.current == (byte)(byte)10);
                Debug.Assert(pack.param2 == (float) -2.8724346E38F);
                Debug.Assert(pack.param1 == (float) -2.7935332E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.z == (float)2.9963037E38F);
                Debug.Assert(pack.y == (float)4.5794557E37F);
                Debug.Assert(pack.param4 == (float)1.9547984E38F);
                Debug.Assert(pack.param3 == (float) -2.7923629E38F);
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION);
                Debug.Assert(pack.seq == (ushort)(ushort)47787);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p39.param2 = (float) -2.8724346E38F;
            p39.param1 = (float) -2.7935332E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.target_system = (byte)(byte)103;
            p39.y = (float)4.5794557E37F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
            p39.target_component = (byte)(byte)191;
            p39.seq = (ushort)(ushort)47787;
            p39.param4 = (float)1.9547984E38F;
            p39.autocontinue = (byte)(byte)232;
            p39.z = (float)2.9963037E38F;
            p39.param3 = (float) -2.7923629E38F;
            p39.x = (float)1.3371075E38F;
            p39.current = (byte)(byte)10;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)201);
                Debug.Assert(pack.seq == (ushort)(ushort)57186);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)57186;
            p40.target_system = (byte)(byte)201;
            p40.target_component = (byte)(byte)213;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.seq == (ushort)(ushort)58843);
                Debug.Assert(pack.target_system == (byte)(byte)156);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)156;
            p41.seq = (ushort)(ushort)58843;
            p41.target_component = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)14154);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)14154;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)67);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)67;
            p43.target_system = (byte)(byte)224;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.count == (ushort)(ushort)65531);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_component = (byte)(byte)59;
            p44.count = (ushort)(ushort)65531;
            p44.target_system = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)166);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)117);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)117;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)166;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)25911);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)25911;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
                Debug.Assert(pack.target_system == (byte)(byte)44);
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE;
            p47.target_component = (byte)(byte)59;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_system = (byte)(byte)44;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -2030348321);
                Debug.Assert(pack.latitude == (int) -1346969066);
                Debug.Assert(pack.altitude == (int) -783146563);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9021514137585242713L);
                Debug.Assert(pack.target_system == (byte)(byte)190);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -783146563;
            p48.latitude = (int) -1346969066;
            p48.longitude = (int) -2030348321;
            p48.target_system = (byte)(byte)190;
            p48.time_usec_SET((ulong)9021514137585242713L, PH) ;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5100166288827822913L);
                Debug.Assert(pack.altitude == (int) -718502967);
                Debug.Assert(pack.longitude == (int) -712431359);
                Debug.Assert(pack.latitude == (int) -1296760251);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int) -1296760251;
            p49.longitude = (int) -712431359;
            p49.time_usec_SET((ulong)5100166288827822913L, PH) ;
            p49.altitude = (int) -718502967;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float)1.3045165E38F);
                Debug.Assert(pack.param_value_min == (float) -2.7198876E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)187);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wrYvkcqyv"));
                Debug.Assert(pack.scale == (float)1.0410975E38F);
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.param_index == (short)(short) -19078);
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.param_value0 == (float) -6.135494E37F);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_min = (float) -2.7198876E38F;
            p50.target_component = (byte)(byte)69;
            p50.param_value_max = (float)1.3045165E38F;
            p50.param_value0 = (float) -6.135494E37F;
            p50.scale = (float)1.0410975E38F;
            p50.parameter_rc_channel_index = (byte)(byte)187;
            p50.param_id_SET("wrYvkcqyv", PH) ;
            p50.target_system = (byte)(byte)137;
            p50.param_index = (short)(short) -19078;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)32049);
                Debug.Assert(pack.target_component == (byte)(byte)178);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)175;
            p51.target_component = (byte)(byte)178;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.seq = (ushort)(ushort)32049;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float) -2.7526073E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p2x == (float) -1.846367E38F);
                Debug.Assert(pack.p2y == (float)1.802894E38F);
                Debug.Assert(pack.p1z == (float)2.8231323E38F);
                Debug.Assert(pack.target_system == (byte)(byte)159);
                Debug.Assert(pack.p1y == (float) -1.0484135E38F);
                Debug.Assert(pack.p2z == (float) -8.3576544E37F);
                Debug.Assert(pack.target_component == (byte)(byte)132);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float)2.8231323E38F;
            p54.p1x = (float) -2.7526073E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p2x = (float) -1.846367E38F;
            p54.p1y = (float) -1.0484135E38F;
            p54.p2z = (float) -8.3576544E37F;
            p54.target_component = (byte)(byte)132;
            p54.p2y = (float)1.802894E38F;
            p54.target_system = (byte)(byte)159;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float) -9.347956E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p2x == (float)6.495817E35F);
                Debug.Assert(pack.p1x == (float) -1.2809651E38F);
                Debug.Assert(pack.p1z == (float) -4.274892E36F);
                Debug.Assert(pack.p1y == (float) -1.3408318E38F);
                Debug.Assert(pack.p2y == (float) -2.6583992E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float)6.495817E35F;
            p55.p2z = (float) -9.347956E37F;
            p55.p2y = (float) -2.6583992E38F;
            p55.p1x = (float) -1.2809651E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p1y = (float) -1.3408318E38F;
            p55.p1z = (float) -4.274892E36F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)7.1774473E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.9034835E38F, -2.6597277E38F, -1.2860917E38F, -1.7205913E38F, 8.955415E37F, 1.198664E38F, 1.1085397E38F, 2.2043975E38F, 5.9418323E37F}));
                Debug.Assert(pack.time_usec == (ulong)6783260265504704158L);
                Debug.Assert(pack.pitchspeed == (float) -1.1621736E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.8696139E38F, -7.725314E36F, -2.6739208E37F, -2.4343915E38F}));
                Debug.Assert(pack.yawspeed == (float) -2.046602E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {1.8696139E38F, -7.725314E36F, -2.6739208E37F, -2.4343915E38F}, 0) ;
            p61.rollspeed = (float)7.1774473E37F;
            p61.yawspeed = (float) -2.046602E38F;
            p61.covariance_SET(new float[] {1.9034835E38F, -2.6597277E38F, -1.2860917E38F, -1.7205913E38F, 8.955415E37F, 1.198664E38F, 1.1085397E38F, 2.2043975E38F, 5.9418323E37F}, 0) ;
            p61.time_usec = (ulong)6783260265504704158L;
            p61.pitchspeed = (float) -1.1621736E37F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aspd_error == (float)1.577253E38F);
                Debug.Assert(pack.nav_pitch == (float)2.1153876E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)37663);
                Debug.Assert(pack.xtrack_error == (float) -2.414204E38F);
                Debug.Assert(pack.alt_error == (float) -8.24943E37F);
                Debug.Assert(pack.nav_bearing == (short)(short)19766);
                Debug.Assert(pack.target_bearing == (short)(short) -12838);
                Debug.Assert(pack.nav_roll == (float) -2.3311946E38F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float)2.1153876E38F;
            p62.xtrack_error = (float) -2.414204E38F;
            p62.nav_bearing = (short)(short)19766;
            p62.alt_error = (float) -8.24943E37F;
            p62.aspd_error = (float)1.577253E38F;
            p62.wp_dist = (ushort)(ushort)37663;
            p62.nav_roll = (float) -2.3311946E38F;
            p62.target_bearing = (short)(short) -12838;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -9.689142E37F);
                Debug.Assert(pack.vz == (float)9.184701E37F);
                Debug.Assert(pack.relative_alt == (int)1762940338);
                Debug.Assert(pack.time_usec == (ulong)8616445454828573171L);
                Debug.Assert(pack.lat == (int)1668288461);
                Debug.Assert(pack.vy == (float)2.9679508E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.lon == (int)728469668);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.5570515E38F, 1.4164599E38F, -2.6305366E38F, -2.271491E38F, -2.2703024E38F, 1.4955886E38F, -2.145862E38F, -1.9444428E37F, -3.3563699E38F, -3.3037798E38F, -4.7937947E37F, 2.2108976E38F, -2.0062546E38F, -1.8431116E38F, -7.691613E37F, 1.1949191E38F, 2.8508219E38F, -1.2947312E38F, 2.12408E38F, 1.1669964E36F, 1.959698E38F, -3.4114832E36F, -1.2723422E38F, -3.3676836E38F, -1.2662215E38F, -1.9160344E38F, -2.275731E38F, 1.1506486E38F, -9.511278E37F, 2.4336977E37F, -2.3964064E38F, -2.3166205E38F, -1.6683089E38F, -3.0979874E38F, 2.6998627E38F, 2.06815E37F}));
                Debug.Assert(pack.alt == (int)974192477);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lat = (int)1668288461;
            p63.vz = (float)9.184701E37F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.alt = (int)974192477;
            p63.relative_alt = (int)1762940338;
            p63.time_usec = (ulong)8616445454828573171L;
            p63.lon = (int)728469668;
            p63.vx = (float) -9.689142E37F;
            p63.covariance_SET(new float[] {1.5570515E38F, 1.4164599E38F, -2.6305366E38F, -2.271491E38F, -2.2703024E38F, 1.4955886E38F, -2.145862E38F, -1.9444428E37F, -3.3563699E38F, -3.3037798E38F, -4.7937947E37F, 2.2108976E38F, -2.0062546E38F, -1.8431116E38F, -7.691613E37F, 1.1949191E38F, 2.8508219E38F, -1.2947312E38F, 2.12408E38F, 1.1669964E36F, 1.959698E38F, -3.4114832E36F, -1.2723422E38F, -3.3676836E38F, -1.2662215E38F, -1.9160344E38F, -2.275731E38F, 1.1506486E38F, -9.511278E37F, 2.4336977E37F, -2.3964064E38F, -2.3166205E38F, -1.6683089E38F, -3.0979874E38F, 2.6998627E38F, 2.06815E37F}, 0) ;
            p63.vy = (float)2.9679508E38F;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.829904E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.2919828E38F, -1.8154973E38F, 1.8146052E37F, 2.0211395E38F, 2.2981319E38F, 1.9880337E38F, 7.40814E37F, 1.7234951E38F, -1.5588062E38F, 2.9108272E38F, -3.8519833E37F, -2.1762232E37F, -1.1404007E38F, 2.4271781E38F, -1.2035636E38F, -3.2503774E38F, 5.7391674E37F, -2.3820694E38F, -2.4082447E38F, -2.9476698E38F, 7.265803E37F, 8.893442E37F, -3.1170291E38F, 1.04832036E37F, 2.5247982E38F, -9.162519E37F, -2.2154464E38F, -1.74379E38F, 1.3881849E38F, -1.3768957E37F, 2.7422903E38F, 1.2871709E38F, 1.0692575E38F, 3.0018118E37F, 1.8509325E38F, -1.8256394E38F, -1.0466444E38F, 7.977257E37F, 3.060603E38F, -1.3686857E38F, -5.0836714E37F, -7.544751E37F, 3.0397572E38F, 2.0005066E38F, -2.517169E38F}));
                Debug.Assert(pack.az == (float)3.3832493E38F);
                Debug.Assert(pack.ay == (float)3.6079286E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.vx == (float)2.4476522E38F);
                Debug.Assert(pack.time_usec == (ulong)3512697703616111922L);
                Debug.Assert(pack.ax == (float)1.1052248E38F);
                Debug.Assert(pack.vz == (float) -1.5710755E38F);
                Debug.Assert(pack.vy == (float) -1.762562E38F);
                Debug.Assert(pack.z == (float) -1.2782559E37F);
                Debug.Assert(pack.x == (float) -1.5326329E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p64.y = (float) -2.829904E38F;
            p64.vy = (float) -1.762562E38F;
            p64.covariance_SET(new float[] {-1.2919828E38F, -1.8154973E38F, 1.8146052E37F, 2.0211395E38F, 2.2981319E38F, 1.9880337E38F, 7.40814E37F, 1.7234951E38F, -1.5588062E38F, 2.9108272E38F, -3.8519833E37F, -2.1762232E37F, -1.1404007E38F, 2.4271781E38F, -1.2035636E38F, -3.2503774E38F, 5.7391674E37F, -2.3820694E38F, -2.4082447E38F, -2.9476698E38F, 7.265803E37F, 8.893442E37F, -3.1170291E38F, 1.04832036E37F, 2.5247982E38F, -9.162519E37F, -2.2154464E38F, -1.74379E38F, 1.3881849E38F, -1.3768957E37F, 2.7422903E38F, 1.2871709E38F, 1.0692575E38F, 3.0018118E37F, 1.8509325E38F, -1.8256394E38F, -1.0466444E38F, 7.977257E37F, 3.060603E38F, -1.3686857E38F, -5.0836714E37F, -7.544751E37F, 3.0397572E38F, 2.0005066E38F, -2.517169E38F}, 0) ;
            p64.x = (float) -1.5326329E38F;
            p64.time_usec = (ulong)3512697703616111922L;
            p64.ax = (float)1.1052248E38F;
            p64.az = (float)3.3832493E38F;
            p64.vx = (float)2.4476522E38F;
            p64.ay = (float)3.6079286E37F;
            p64.z = (float) -1.2782559E37F;
            p64.vz = (float) -1.5710755E38F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)11202);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)48102);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)32814);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)60762);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)41118);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)30260);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)36781);
                Debug.Assert(pack.time_boot_ms == (uint)1113342209U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18172);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)63025);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)27649);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)43664);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)16893);
                Debug.Assert(pack.rssi == (byte)(byte)237);
                Debug.Assert(pack.chancount == (byte)(byte)126);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)24436);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)9800);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)22984);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)46747);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)39937);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)15695);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan9_raw = (ushort)(ushort)48102;
            p65.chan4_raw = (ushort)(ushort)30260;
            p65.chan12_raw = (ushort)(ushort)27649;
            p65.chan11_raw = (ushort)(ushort)63025;
            p65.chan2_raw = (ushort)(ushort)36781;
            p65.chan13_raw = (ushort)(ushort)32814;
            p65.rssi = (byte)(byte)237;
            p65.chan17_raw = (ushort)(ushort)39937;
            p65.time_boot_ms = (uint)1113342209U;
            p65.chan18_raw = (ushort)(ushort)22984;
            p65.chan10_raw = (ushort)(ushort)46747;
            p65.chan7_raw = (ushort)(ushort)24436;
            p65.chancount = (byte)(byte)126;
            p65.chan16_raw = (ushort)(ushort)60762;
            p65.chan5_raw = (ushort)(ushort)16893;
            p65.chan14_raw = (ushort)(ushort)43664;
            p65.chan6_raw = (ushort)(ushort)11202;
            p65.chan1_raw = (ushort)(ushort)41118;
            p65.chan15_raw = (ushort)(ushort)15695;
            p65.chan8_raw = (ushort)(ushort)18172;
            p65.chan3_raw = (ushort)(ushort)9800;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)211);
                Debug.Assert(pack.target_component == (byte)(byte)45);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)29336);
                Debug.Assert(pack.req_stream_id == (byte)(byte)198);
                Debug.Assert(pack.start_stop == (byte)(byte)128);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)128;
            p66.target_component = (byte)(byte)45;
            p66.req_message_rate = (ushort)(ushort)29336;
            p66.req_stream_id = (byte)(byte)198;
            p66.target_system = (byte)(byte)211;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)44668);
                Debug.Assert(pack.on_off == (byte)(byte)178);
                Debug.Assert(pack.stream_id == (byte)(byte)243);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)243;
            p67.message_rate = (ushort)(ushort)44668;
            p67.on_off = (byte)(byte)178;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)63075);
                Debug.Assert(pack.target == (byte)(byte)240);
                Debug.Assert(pack.r == (short)(short) -23979);
                Debug.Assert(pack.z == (short)(short)6632);
                Debug.Assert(pack.y == (short)(short) -8047);
                Debug.Assert(pack.x == (short)(short)19677);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short) -8047;
            p69.z = (short)(short)6632;
            p69.buttons = (ushort)(ushort)63075;
            p69.r = (short)(short) -23979;
            p69.x = (short)(short)19677;
            p69.target = (byte)(byte)240;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)0);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)8107);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)7589);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)43772);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)44968);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)8810);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)41333);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)54290);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)4949);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan2_raw = (ushort)(ushort)8107;
            p70.target_system = (byte)(byte)0;
            p70.chan1_raw = (ushort)(ushort)8810;
            p70.chan7_raw = (ushort)(ushort)54290;
            p70.chan3_raw = (ushort)(ushort)4949;
            p70.chan6_raw = (ushort)(ushort)41333;
            p70.chan8_raw = (ushort)(ushort)7589;
            p70.target_component = (byte)(byte)143;
            p70.chan4_raw = (ushort)(ushort)43772;
            p70.chan5_raw = (ushort)(ushort)44968;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -6.113228E37F);
                Debug.Assert(pack.param4 == (float) -1.2990872E38F);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.z == (float)1.4091812E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param2 == (float)1.5120544E37F);
                Debug.Assert(pack.param1 == (float) -2.7179133E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)63592);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.x == (int)1119816513);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_START_RX_PAIR);
                Debug.Assert(pack.autocontinue == (byte)(byte)16);
                Debug.Assert(pack.y == (int) -1294196205);
                Debug.Assert(pack.current == (byte)(byte)57);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.y = (int) -1294196205;
            p73.param2 = (float)1.5120544E37F;
            p73.seq = (ushort)(ushort)63592;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p73.param4 = (float) -1.2990872E38F;
            p73.param3 = (float) -6.113228E37F;
            p73.target_component = (byte)(byte)26;
            p73.z = (float)1.4091812E38F;
            p73.target_system = (byte)(byte)228;
            p73.param1 = (float) -2.7179133E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_START_RX_PAIR;
            p73.autocontinue = (byte)(byte)16;
            p73.current = (byte)(byte)57;
            p73.x = (int)1119816513;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)2.4256415E38F);
                Debug.Assert(pack.airspeed == (float)4.182524E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)43344);
                Debug.Assert(pack.climb == (float) -1.6672834E38F);
                Debug.Assert(pack.heading == (short)(short) -30605);
                Debug.Assert(pack.groundspeed == (float) -2.0692773E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)2.4256415E38F;
            p74.climb = (float) -1.6672834E38F;
            p74.airspeed = (float)4.182524E37F;
            p74.heading = (short)(short) -30605;
            p74.throttle = (ushort)(ushort)43344;
            p74.groundspeed = (float) -2.0692773E38F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int)788845555);
                Debug.Assert(pack.param1 == (float)1.8678142E38F);
                Debug.Assert(pack.param4 == (float) -2.8962526E38F);
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.x == (int) -18658495);
                Debug.Assert(pack.autocontinue == (byte)(byte)103);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.param3 == (float)2.7193675E38F);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.current == (byte)(byte)189);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_DELAY);
                Debug.Assert(pack.param2 == (float) -3.0854147E38F);
                Debug.Assert(pack.z == (float)1.201425E37F);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param1 = (float)1.8678142E38F;
            p75.param3 = (float)2.7193675E38F;
            p75.param2 = (float) -3.0854147E38F;
            p75.autocontinue = (byte)(byte)103;
            p75.z = (float)1.201425E37F;
            p75.y = (int)788845555;
            p75.x = (int) -18658495;
            p75.target_component = (byte)(byte)213;
            p75.param4 = (float) -2.8962526E38F;
            p75.target_system = (byte)(byte)224;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_DELAY;
            p75.current = (byte)(byte)189;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param5 == (float) -1.4555822E38F);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.param3 == (float) -3.3711841E38F);
                Debug.Assert(pack.param7 == (float)1.0193383E38F);
                Debug.Assert(pack.target_component == (byte)(byte)75);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.param4 == (float)1.4947412E38F);
                Debug.Assert(pack.param1 == (float) -1.4551219E38F);
                Debug.Assert(pack.param6 == (float) -1.5617899E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)151);
                Debug.Assert(pack.param2 == (float) -2.887332E38F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)79;
            p76.param4 = (float)1.4947412E38F;
            p76.param1 = (float) -1.4551219E38F;
            p76.confirmation = (byte)(byte)151;
            p76.param6 = (float) -1.5617899E38F;
            p76.param3 = (float) -3.3711841E38F;
            p76.target_component = (byte)(byte)75;
            p76.param2 = (float) -2.887332E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_LAST;
            p76.param5 = (float) -1.4555822E38F;
            p76.param7 = (float)1.0193383E38F;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)75);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)54);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1538648711);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_LOGGING_START);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)120);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.target_component_SET((byte)(byte)54, PH) ;
            p77.progress_SET((byte)(byte)120, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_LOGGING_START;
            p77.target_system_SET((byte)(byte)75, PH) ;
            p77.result_param2_SET((int)1538648711, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)1.1261261E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)229);
                Debug.Assert(pack.yaw == (float) -1.1062516E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)118);
                Debug.Assert(pack.thrust == (float) -3.2275147E38F);
                Debug.Assert(pack.roll == (float)2.9379214E38F);
                Debug.Assert(pack.time_boot_ms == (uint)652421977U);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)652421977U;
            p81.pitch = (float)1.1261261E38F;
            p81.roll = (float)2.9379214E38F;
            p81.mode_switch = (byte)(byte)229;
            p81.yaw = (float) -1.1062516E38F;
            p81.thrust = (float) -3.2275147E38F;
            p81.manual_override_switch = (byte)(byte)118;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float) -2.9676356E38F);
                Debug.Assert(pack.body_yaw_rate == (float)2.8394094E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)176);
                Debug.Assert(pack.body_roll_rate == (float) -4.828432E37F);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.target_component == (byte)(byte)138);
                Debug.Assert(pack.time_boot_ms == (uint)1954325809U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9452442E38F, -3.265861E38F, -1.3562066E38F, -9.234825E37F}));
                Debug.Assert(pack.thrust == (float) -1.7106539E37F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_pitch_rate = (float) -2.9676356E38F;
            p82.thrust = (float) -1.7106539E37F;
            p82.body_roll_rate = (float) -4.828432E37F;
            p82.type_mask = (byte)(byte)176;
            p82.time_boot_ms = (uint)1954325809U;
            p82.target_system = (byte)(byte)85;
            p82.body_yaw_rate = (float)2.8394094E38F;
            p82.target_component = (byte)(byte)138;
            p82.q_SET(new float[] {2.9452442E38F, -3.265861E38F, -1.3562066E38F, -9.234825E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -3.5542408E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1976073766U);
                Debug.Assert(pack.body_yaw_rate == (float)1.4977877E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.3770126E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8598228E38F, -1.6139721E38F, 1.035211E38F, -2.211203E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)94);
                Debug.Assert(pack.body_roll_rate == (float)3.2252112E38F);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)3.2252112E38F;
            p83.thrust = (float) -3.5542408E37F;
            p83.time_boot_ms = (uint)1976073766U;
            p83.body_yaw_rate = (float)1.4977877E38F;
            p83.type_mask = (byte)(byte)94;
            p83.q_SET(new float[] {-2.8598228E38F, -1.6139721E38F, 1.035211E38F, -2.211203E38F}, 0) ;
            p83.body_pitch_rate = (float)2.3770126E38F;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)40011);
                Debug.Assert(pack.y == (float) -1.4737494E38F);
                Debug.Assert(pack.afz == (float)3.3970568E38F);
                Debug.Assert(pack.afy == (float) -2.0801317E38F);
                Debug.Assert(pack.afx == (float) -2.8538403E38F);
                Debug.Assert(pack.yaw == (float)1.7702586E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1521931821U);
                Debug.Assert(pack.z == (float)2.8792759E38F);
                Debug.Assert(pack.vz == (float)3.2417874E38F);
                Debug.Assert(pack.x == (float)2.5641422E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.7881005E38F);
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.vy == (float)3.3206085E36F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.vx == (float)1.8295245E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vy = (float)3.3206085E36F;
            p84.type_mask = (ushort)(ushort)40011;
            p84.time_boot_ms = (uint)1521931821U;
            p84.vz = (float)3.2417874E38F;
            p84.z = (float)2.8792759E38F;
            p84.yaw_rate = (float) -2.7881005E38F;
            p84.afx = (float) -2.8538403E38F;
            p84.x = (float)2.5641422E38F;
            p84.y = (float) -1.4737494E38F;
            p84.yaw = (float)1.7702586E38F;
            p84.vx = (float)1.8295245E38F;
            p84.afz = (float)3.3970568E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.target_component = (byte)(byte)7;
            p84.afy = (float) -2.0801317E38F;
            p84.target_system = (byte)(byte)242;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -3.6010506E37F);
                Debug.Assert(pack.vz == (float) -8.767076E37F);
                Debug.Assert(pack.target_component == (byte)(byte)146);
                Debug.Assert(pack.time_boot_ms == (uint)2893785548U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)63046);
                Debug.Assert(pack.yaw == (float)2.853659E38F);
                Debug.Assert(pack.afz == (float) -2.541381E38F);
                Debug.Assert(pack.lat_int == (int) -2077812690);
                Debug.Assert(pack.vy == (float)3.0464508E38F);
                Debug.Assert(pack.alt == (float)1.1750854E38F);
                Debug.Assert(pack.target_system == (byte)(byte)218);
                Debug.Assert(pack.yaw_rate == (float) -2.565026E38F);
                Debug.Assert(pack.afx == (float)2.0325047E38F);
                Debug.Assert(pack.lon_int == (int)1939412985);
                Debug.Assert(pack.afy == (float)1.7243626E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.alt = (float)1.1750854E38F;
            p86.yaw = (float)2.853659E38F;
            p86.vz = (float) -8.767076E37F;
            p86.yaw_rate = (float) -2.565026E38F;
            p86.target_component = (byte)(byte)146;
            p86.afz = (float) -2.541381E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p86.afx = (float)2.0325047E38F;
            p86.afy = (float)1.7243626E38F;
            p86.target_system = (byte)(byte)218;
            p86.vx = (float) -3.6010506E37F;
            p86.lon_int = (int)1939412985;
            p86.lat_int = (int) -2077812690;
            p86.time_boot_ms = (uint)2893785548U;
            p86.type_mask = (ushort)(ushort)63046;
            p86.vy = (float)3.0464508E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)1.8232329E38F);
                Debug.Assert(pack.alt == (float)2.4539673E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1139551096U);
                Debug.Assert(pack.lon_int == (int) -1961762023);
                Debug.Assert(pack.type_mask == (ushort)(ushort)12238);
                Debug.Assert(pack.vz == (float) -4.0784016E37F);
                Debug.Assert(pack.lat_int == (int)1242805204);
                Debug.Assert(pack.afy == (float)1.2155796E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afx == (float) -3.0219074E38F);
                Debug.Assert(pack.afz == (float) -5.4462187E36F);
                Debug.Assert(pack.vy == (float)9.4971413E36F);
                Debug.Assert(pack.yaw == (float) -6.964656E37F);
                Debug.Assert(pack.vx == (float) -2.441663E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1139551096U;
            p87.afy = (float)1.2155796E38F;
            p87.afz = (float) -5.4462187E36F;
            p87.type_mask = (ushort)(ushort)12238;
            p87.alt = (float)2.4539673E38F;
            p87.vz = (float) -4.0784016E37F;
            p87.afx = (float) -3.0219074E38F;
            p87.lat_int = (int)1242805204;
            p87.yaw = (float) -6.964656E37F;
            p87.lon_int = (int) -1961762023;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.vy = (float)9.4971413E36F;
            p87.yaw_rate = (float)1.8232329E38F;
            p87.vx = (float) -2.441663E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -7.5562107E37F);
                Debug.Assert(pack.time_boot_ms == (uint)828560540U);
                Debug.Assert(pack.x == (float)1.2467139E38F);
                Debug.Assert(pack.y == (float) -2.6113339E38F);
                Debug.Assert(pack.pitch == (float)2.724143E38F);
                Debug.Assert(pack.roll == (float)2.800022E38F);
                Debug.Assert(pack.yaw == (float)1.5337447E37F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float) -7.5562107E37F;
            p89.x = (float)1.2467139E38F;
            p89.yaw = (float)1.5337447E37F;
            p89.time_boot_ms = (uint)828560540U;
            p89.y = (float) -2.6113339E38F;
            p89.roll = (float)2.800022E38F;
            p89.pitch = (float)2.724143E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.0967368E38F);
                Debug.Assert(pack.xacc == (short)(short)12530);
                Debug.Assert(pack.yacc == (short)(short) -18610);
                Debug.Assert(pack.lon == (int)61902220);
                Debug.Assert(pack.vy == (short)(short)2049);
                Debug.Assert(pack.yawspeed == (float)3.0191604E38F);
                Debug.Assert(pack.zacc == (short)(short)8919);
                Debug.Assert(pack.vz == (short)(short) -28503);
                Debug.Assert(pack.pitch == (float)3.815757E36F);
                Debug.Assert(pack.rollspeed == (float) -4.996323E36F);
                Debug.Assert(pack.vx == (short)(short) -27553);
                Debug.Assert(pack.alt == (int)590413831);
                Debug.Assert(pack.pitchspeed == (float)1.3764687E38F);
                Debug.Assert(pack.yaw == (float)1.5470694E38F);
                Debug.Assert(pack.lat == (int)477788728);
                Debug.Assert(pack.time_usec == (ulong)6321975124326549258L);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitchspeed = (float)1.3764687E38F;
            p90.lat = (int)477788728;
            p90.yacc = (short)(short) -18610;
            p90.vx = (short)(short) -27553;
            p90.rollspeed = (float) -4.996323E36F;
            p90.vy = (short)(short)2049;
            p90.pitch = (float)3.815757E36F;
            p90.xacc = (short)(short)12530;
            p90.roll = (float)1.0967368E38F;
            p90.vz = (short)(short) -28503;
            p90.alt = (int)590413831;
            p90.yaw = (float)1.5470694E38F;
            p90.zacc = (short)(short)8919;
            p90.time_usec = (ulong)6321975124326549258L;
            p90.yawspeed = (float)3.0191604E38F;
            p90.lon = (int)61902220;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_ailerons == (float) -2.9993323E38F);
                Debug.Assert(pack.throttle == (float) -3.292657E38F);
                Debug.Assert(pack.yaw_rudder == (float)5.4286665E37F);
                Debug.Assert(pack.aux2 == (float) -3.0231806E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)104);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.time_usec == (ulong)7262946709653995087L);
                Debug.Assert(pack.aux3 == (float) -3.094351E38F);
                Debug.Assert(pack.pitch_elevator == (float) -3.0842675E38F);
                Debug.Assert(pack.aux1 == (float)1.216966E38F);
                Debug.Assert(pack.aux4 == (float)2.0346427E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.nav_mode = (byte)(byte)104;
            p91.aux2 = (float) -3.0231806E38F;
            p91.pitch_elevator = (float) -3.0842675E38F;
            p91.roll_ailerons = (float) -2.9993323E38F;
            p91.time_usec = (ulong)7262946709653995087L;
            p91.aux4 = (float)2.0346427E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p91.yaw_rudder = (float)5.4286665E37F;
            p91.aux1 = (float)1.216966E38F;
            p91.aux3 = (float) -3.094351E38F;
            p91.throttle = (float) -3.292657E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)56014);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)50035);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)54752);
                Debug.Assert(pack.time_usec == (ulong)3543405836996195407L);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)10401);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)43295);
                Debug.Assert(pack.rssi == (byte)(byte)242);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)8231);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)8342);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)58929);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32962);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)50931);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)3993);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41722);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan12_raw = (ushort)(ushort)58929;
            p92.time_usec = (ulong)3543405836996195407L;
            p92.chan3_raw = (ushort)(ushort)43295;
            p92.chan1_raw = (ushort)(ushort)32962;
            p92.chan10_raw = (ushort)(ushort)3993;
            p92.chan2_raw = (ushort)(ushort)56014;
            p92.rssi = (byte)(byte)242;
            p92.chan9_raw = (ushort)(ushort)8231;
            p92.chan8_raw = (ushort)(ushort)50931;
            p92.chan6_raw = (ushort)(ushort)10401;
            p92.chan5_raw = (ushort)(ushort)8342;
            p92.chan4_raw = (ushort)(ushort)54752;
            p92.chan11_raw = (ushort)(ushort)50035;
            p92.chan7_raw = (ushort)(ushort)41722;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2154072E38F, -1.3083045E38F, -6.193073E37F, 3.0651648E38F, -2.4881874E38F, 1.6462584E38F, -2.8481939E38F, -1.2974685E38F, 1.7265983E38F, 3.2250611E38F, -1.7955356E38F, 2.706867E38F, 8.1424053E37F, -1.2535196E38F, -1.1609584E38F, -2.8566547E38F}));
                Debug.Assert(pack.flags == (ulong)1314889773274271499L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.time_usec == (ulong)705068818035061821L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-1.2154072E38F, -1.3083045E38F, -6.193073E37F, 3.0651648E38F, -2.4881874E38F, 1.6462584E38F, -2.8481939E38F, -1.2974685E38F, 1.7265983E38F, 3.2250611E38F, -1.7955356E38F, 2.706867E38F, 8.1424053E37F, -1.2535196E38F, -1.1609584E38F, -2.8566547E38F}, 0) ;
            p93.flags = (ulong)1314889773274271499L;
            p93.time_usec = (ulong)705068818035061821L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)0);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.2259422E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.720034E38F);
                Debug.Assert(pack.flow_y == (short)(short) -9495);
                Debug.Assert(pack.time_usec == (ulong)6018789813544468687L);
                Debug.Assert(pack.flow_x == (short)(short)31339);
                Debug.Assert(pack.flow_comp_m_x == (float)1.6065723E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)99);
                Debug.Assert(pack.ground_distance == (float)3.3197354E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)7.933369E37F);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_y = (short)(short) -9495;
            p100.quality = (byte)(byte)0;
            p100.flow_rate_x_SET((float)2.720034E38F, PH) ;
            p100.flow_comp_m_x = (float)1.6065723E38F;
            p100.ground_distance = (float)3.3197354E38F;
            p100.flow_x = (short)(short)31339;
            p100.flow_rate_y_SET((float)7.933369E37F, PH) ;
            p100.sensor_id = (byte)(byte)99;
            p100.time_usec = (ulong)6018789813544468687L;
            p100.flow_comp_m_y = (float) -3.2259422E38F;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.043011E38F);
                Debug.Assert(pack.usec == (ulong)5006228194507012375L);
                Debug.Assert(pack.x == (float) -5.2894547E37F);
                Debug.Assert(pack.z == (float) -1.905256E38F);
                Debug.Assert(pack.roll == (float) -8.4544755E37F);
                Debug.Assert(pack.y == (float)3.2363027E38F);
                Debug.Assert(pack.yaw == (float)2.785788E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float)2.785788E38F;
            p101.z = (float) -1.905256E38F;
            p101.roll = (float) -8.4544755E37F;
            p101.usec = (ulong)5006228194507012375L;
            p101.y = (float)3.2363027E38F;
            p101.pitch = (float) -1.043011E38F;
            p101.x = (float) -5.2894547E37F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.3175666E38F);
                Debug.Assert(pack.z == (float) -2.66624E38F);
                Debug.Assert(pack.usec == (ulong)2752290274373781250L);
                Debug.Assert(pack.y == (float)5.7245935E36F);
                Debug.Assert(pack.yaw == (float)2.2386207E38F);
                Debug.Assert(pack.roll == (float) -1.0604612E38F);
                Debug.Assert(pack.pitch == (float) -8.0648935E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.y = (float)5.7245935E36F;
            p102.usec = (ulong)2752290274373781250L;
            p102.yaw = (float)2.2386207E38F;
            p102.pitch = (float) -8.0648935E37F;
            p102.x = (float) -1.3175666E38F;
            p102.roll = (float) -1.0604612E38F;
            p102.z = (float) -2.66624E38F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.1748799E35F);
                Debug.Assert(pack.z == (float)1.2373838E38F);
                Debug.Assert(pack.usec == (ulong)8787725347807006871L);
                Debug.Assert(pack.y == (float)3.329701E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)8787725347807006871L;
            p103.x = (float)3.1748799E35F;
            p103.z = (float)1.2373838E38F;
            p103.y = (float)3.329701E37F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)6.396749E37F);
                Debug.Assert(pack.y == (float)1.3553002E38F);
                Debug.Assert(pack.usec == (ulong)4413771976733877059L);
                Debug.Assert(pack.yaw == (float)1.9338032E38F);
                Debug.Assert(pack.roll == (float) -4.9533493E37F);
                Debug.Assert(pack.x == (float) -2.5195401E38F);
                Debug.Assert(pack.z == (float) -9.665348E37F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)4413771976733877059L;
            p104.pitch = (float)6.396749E37F;
            p104.roll = (float) -4.9533493E37F;
            p104.y = (float)1.3553002E38F;
            p104.z = (float) -9.665348E37F;
            p104.x = (float) -2.5195401E38F;
            p104.yaw = (float)1.9338032E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (float) -2.8402288E38F);
                Debug.Assert(pack.diff_pressure == (float)2.406612E38F);
                Debug.Assert(pack.zgyro == (float)2.8664014E38F);
                Debug.Assert(pack.abs_pressure == (float) -3.3848539E38F);
                Debug.Assert(pack.xgyro == (float) -3.360743E38F);
                Debug.Assert(pack.temperature == (float) -2.148925E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.2761591E38F);
                Debug.Assert(pack.time_usec == (ulong)1313071990376648603L);
                Debug.Assert(pack.zacc == (float)7.258778E37F);
                Debug.Assert(pack.ymag == (float) -9.318978E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)40774);
                Debug.Assert(pack.xmag == (float) -2.2416502E38F);
                Debug.Assert(pack.yacc == (float)1.2166867E38F);
                Debug.Assert(pack.ygyro == (float) -3.1862688E38F);
                Debug.Assert(pack.xacc == (float)2.7159296E38F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xgyro = (float) -3.360743E38F;
            p105.yacc = (float)1.2166867E38F;
            p105.xacc = (float)2.7159296E38F;
            p105.fields_updated = (ushort)(ushort)40774;
            p105.ygyro = (float) -3.1862688E38F;
            p105.diff_pressure = (float)2.406612E38F;
            p105.pressure_alt = (float) -2.2761591E38F;
            p105.temperature = (float) -2.148925E38F;
            p105.abs_pressure = (float) -3.3848539E38F;
            p105.xmag = (float) -2.2416502E38F;
            p105.ymag = (float) -9.318978E37F;
            p105.zgyro = (float)2.8664014E38F;
            p105.zacc = (float)7.258778E37F;
            p105.time_usec = (ulong)1313071990376648603L;
            p105.zmag = (float) -2.8402288E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)129);
                Debug.Assert(pack.integrated_y == (float) -3.0726275E38F);
                Debug.Assert(pack.integrated_x == (float)2.9978514E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1812821525U);
                Debug.Assert(pack.sensor_id == (byte)(byte)121);
                Debug.Assert(pack.integrated_xgyro == (float)4.880481E36F);
                Debug.Assert(pack.integration_time_us == (uint)328831427U);
                Debug.Assert(pack.integrated_zgyro == (float) -2.2535893E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -9.450682E37F);
                Debug.Assert(pack.temperature == (short)(short)21969);
                Debug.Assert(pack.time_usec == (ulong)6011850807767626813L);
                Debug.Assert(pack.distance == (float) -1.0374822E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_delta_distance_us = (uint)1812821525U;
            p106.temperature = (short)(short)21969;
            p106.integrated_x = (float)2.9978514E37F;
            p106.integration_time_us = (uint)328831427U;
            p106.quality = (byte)(byte)129;
            p106.integrated_y = (float) -3.0726275E38F;
            p106.sensor_id = (byte)(byte)121;
            p106.distance = (float) -1.0374822E38F;
            p106.integrated_ygyro = (float) -9.450682E37F;
            p106.integrated_xgyro = (float)4.880481E36F;
            p106.time_usec = (ulong)6011850807767626813L;
            p106.integrated_zgyro = (float) -2.2535893E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -3.0402797E38F);
                Debug.Assert(pack.xmag == (float)3.3027736E38F);
                Debug.Assert(pack.zacc == (float)9.71129E37F);
                Debug.Assert(pack.fields_updated == (uint)3570512766U);
                Debug.Assert(pack.yacc == (float)2.1771424E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.7999971E38F);
                Debug.Assert(pack.xacc == (float) -2.026879E38F);
                Debug.Assert(pack.zmag == (float)1.4534971E38F);
                Debug.Assert(pack.zgyro == (float) -1.925457E38F);
                Debug.Assert(pack.xgyro == (float)1.5673812E38F);
                Debug.Assert(pack.pressure_alt == (float)2.1615463E38F);
                Debug.Assert(pack.ymag == (float)6.1133414E37F);
                Debug.Assert(pack.diff_pressure == (float)4.7148946E37F);
                Debug.Assert(pack.temperature == (float)7.0065614E37F);
                Debug.Assert(pack.time_usec == (ulong)8849640681654090334L);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.fields_updated = (uint)3570512766U;
            p107.ymag = (float)6.1133414E37F;
            p107.time_usec = (ulong)8849640681654090334L;
            p107.xmag = (float)3.3027736E38F;
            p107.xgyro = (float)1.5673812E38F;
            p107.pressure_alt = (float)2.1615463E38F;
            p107.zmag = (float)1.4534971E38F;
            p107.zacc = (float)9.71129E37F;
            p107.yacc = (float)2.1771424E38F;
            p107.diff_pressure = (float)4.7148946E37F;
            p107.zgyro = (float) -1.925457E38F;
            p107.abs_pressure = (float) -1.7999971E38F;
            p107.ygyro = (float) -3.0402797E38F;
            p107.xacc = (float) -2.026879E38F;
            p107.temperature = (float)7.0065614E37F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)2.9903354E38F);
                Debug.Assert(pack.q2 == (float)2.3523212E37F);
                Debug.Assert(pack.pitch == (float) -7.290334E37F);
                Debug.Assert(pack.alt == (float)6.8369664E36F);
                Debug.Assert(pack.vd == (float)1.1321072E38F);
                Debug.Assert(pack.roll == (float)5.257832E37F);
                Debug.Assert(pack.yacc == (float)2.4228316E38F);
                Debug.Assert(pack.zgyro == (float) -2.4748562E38F);
                Debug.Assert(pack.q3 == (float) -1.1565548E38F);
                Debug.Assert(pack.std_dev_vert == (float) -5.1378644E37F);
                Debug.Assert(pack.std_dev_horz == (float) -1.209918E38F);
                Debug.Assert(pack.ygyro == (float) -1.6961302E38F);
                Debug.Assert(pack.q4 == (float) -4.0707032E37F);
                Debug.Assert(pack.vn == (float)3.2535064E38F);
                Debug.Assert(pack.zacc == (float)1.1962443E38F);
                Debug.Assert(pack.lon == (float)1.7087443E38F);
                Debug.Assert(pack.lat == (float)1.4612562E38F);
                Debug.Assert(pack.xgyro == (float)1.041307E38F);
                Debug.Assert(pack.ve == (float)2.883061E38F);
                Debug.Assert(pack.q1 == (float)1.4992831E38F);
                Debug.Assert(pack.yaw == (float)2.8850515E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zgyro = (float) -2.4748562E38F;
            p108.xgyro = (float)1.041307E38F;
            p108.pitch = (float) -7.290334E37F;
            p108.vn = (float)3.2535064E38F;
            p108.q3 = (float) -1.1565548E38F;
            p108.q4 = (float) -4.0707032E37F;
            p108.ygyro = (float) -1.6961302E38F;
            p108.ve = (float)2.883061E38F;
            p108.q1 = (float)1.4992831E38F;
            p108.q2 = (float)2.3523212E37F;
            p108.yacc = (float)2.4228316E38F;
            p108.std_dev_horz = (float) -1.209918E38F;
            p108.std_dev_vert = (float) -5.1378644E37F;
            p108.vd = (float)1.1321072E38F;
            p108.zacc = (float)1.1962443E38F;
            p108.roll = (float)5.257832E37F;
            p108.alt = (float)6.8369664E36F;
            p108.xacc = (float)2.9903354E38F;
            p108.lon = (float)1.7087443E38F;
            p108.yaw = (float)2.8850515E38F;
            p108.lat = (float)1.4612562E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)239);
                Debug.Assert(pack.noise == (byte)(byte)184);
                Debug.Assert(pack.remnoise == (byte)(byte)56);
                Debug.Assert(pack.rssi == (byte)(byte)209);
                Debug.Assert(pack.remrssi == (byte)(byte)238);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)7221);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)36338);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)7221;
            p109.txbuf = (byte)(byte)239;
            p109.remrssi = (byte)(byte)238;
            p109.noise = (byte)(byte)184;
            p109.fixed_ = (ushort)(ushort)36338;
            p109.remnoise = (byte)(byte)56;
            p109.rssi = (byte)(byte)209;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)147);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)3, (byte)187, (byte)33, (byte)49, (byte)69, (byte)27, (byte)229, (byte)239, (byte)199, (byte)64, (byte)151, (byte)11, (byte)153, (byte)255, (byte)142, (byte)5, (byte)200, (byte)228, (byte)137, (byte)57, (byte)236, (byte)146, (byte)135, (byte)215, (byte)178, (byte)29, (byte)159, (byte)31, (byte)44, (byte)159, (byte)197, (byte)91, (byte)192, (byte)6, (byte)85, (byte)33, (byte)75, (byte)131, (byte)231, (byte)13, (byte)27, (byte)130, (byte)21, (byte)191, (byte)97, (byte)234, (byte)180, (byte)249, (byte)177, (byte)254, (byte)95, (byte)51, (byte)202, (byte)95, (byte)124, (byte)149, (byte)98, (byte)202, (byte)198, (byte)176, (byte)212, (byte)8, (byte)232, (byte)12, (byte)133, (byte)141, (byte)156, (byte)141, (byte)139, (byte)21, (byte)183, (byte)218, (byte)86, (byte)249, (byte)68, (byte)151, (byte)250, (byte)87, (byte)99, (byte)103, (byte)26, (byte)68, (byte)182, (byte)189, (byte)21, (byte)106, (byte)124, (byte)161, (byte)4, (byte)53, (byte)78, (byte)139, (byte)186, (byte)208, (byte)26, (byte)25, (byte)201, (byte)47, (byte)44, (byte)147, (byte)37, (byte)105, (byte)165, (byte)117, (byte)72, (byte)189, (byte)13, (byte)96, (byte)252, (byte)87, (byte)27, (byte)118, (byte)165, (byte)178, (byte)45, (byte)19, (byte)164, (byte)141, (byte)17, (byte)78, (byte)31, (byte)38, (byte)238, (byte)10, (byte)253, (byte)63, (byte)146, (byte)254, (byte)50, (byte)91, (byte)82, (byte)227, (byte)93, (byte)18, (byte)160, (byte)18, (byte)31, (byte)240, (byte)37, (byte)70, (byte)2, (byte)51, (byte)203, (byte)28, (byte)182, (byte)203, (byte)241, (byte)116, (byte)152, (byte)134, (byte)253, (byte)94, (byte)238, (byte)109, (byte)51, (byte)245, (byte)147, (byte)20, (byte)135, (byte)62, (byte)116, (byte)83, (byte)32, (byte)8, (byte)55, (byte)143, (byte)90, (byte)125, (byte)24, (byte)88, (byte)137, (byte)221, (byte)0, (byte)175, (byte)231, (byte)168, (byte)19, (byte)0, (byte)134, (byte)70, (byte)41, (byte)162, (byte)9, (byte)161, (byte)86, (byte)201, (byte)10, (byte)217, (byte)255, (byte)6, (byte)151, (byte)159, (byte)157, (byte)193, (byte)179, (byte)102, (byte)130, (byte)133, (byte)110, (byte)198, (byte)237, (byte)226, (byte)107, (byte)248, (byte)158, (byte)48, (byte)181, (byte)93, (byte)174, (byte)25, (byte)148, (byte)124, (byte)235, (byte)15, (byte)138, (byte)28, (byte)151, (byte)185, (byte)75, (byte)4, (byte)45, (byte)83, (byte)67, (byte)121, (byte)242, (byte)197, (byte)216, (byte)135, (byte)94, (byte)157, (byte)231, (byte)29, (byte)63, (byte)12, (byte)99, (byte)238, (byte)197, (byte)13, (byte)170, (byte)241, (byte)136, (byte)202, (byte)151, (byte)255, (byte)129, (byte)26, (byte)75, (byte)248, (byte)181, (byte)203, (byte)177}));
                Debug.Assert(pack.target_system == (byte)(byte)148);
                Debug.Assert(pack.target_network == (byte)(byte)135);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)3, (byte)187, (byte)33, (byte)49, (byte)69, (byte)27, (byte)229, (byte)239, (byte)199, (byte)64, (byte)151, (byte)11, (byte)153, (byte)255, (byte)142, (byte)5, (byte)200, (byte)228, (byte)137, (byte)57, (byte)236, (byte)146, (byte)135, (byte)215, (byte)178, (byte)29, (byte)159, (byte)31, (byte)44, (byte)159, (byte)197, (byte)91, (byte)192, (byte)6, (byte)85, (byte)33, (byte)75, (byte)131, (byte)231, (byte)13, (byte)27, (byte)130, (byte)21, (byte)191, (byte)97, (byte)234, (byte)180, (byte)249, (byte)177, (byte)254, (byte)95, (byte)51, (byte)202, (byte)95, (byte)124, (byte)149, (byte)98, (byte)202, (byte)198, (byte)176, (byte)212, (byte)8, (byte)232, (byte)12, (byte)133, (byte)141, (byte)156, (byte)141, (byte)139, (byte)21, (byte)183, (byte)218, (byte)86, (byte)249, (byte)68, (byte)151, (byte)250, (byte)87, (byte)99, (byte)103, (byte)26, (byte)68, (byte)182, (byte)189, (byte)21, (byte)106, (byte)124, (byte)161, (byte)4, (byte)53, (byte)78, (byte)139, (byte)186, (byte)208, (byte)26, (byte)25, (byte)201, (byte)47, (byte)44, (byte)147, (byte)37, (byte)105, (byte)165, (byte)117, (byte)72, (byte)189, (byte)13, (byte)96, (byte)252, (byte)87, (byte)27, (byte)118, (byte)165, (byte)178, (byte)45, (byte)19, (byte)164, (byte)141, (byte)17, (byte)78, (byte)31, (byte)38, (byte)238, (byte)10, (byte)253, (byte)63, (byte)146, (byte)254, (byte)50, (byte)91, (byte)82, (byte)227, (byte)93, (byte)18, (byte)160, (byte)18, (byte)31, (byte)240, (byte)37, (byte)70, (byte)2, (byte)51, (byte)203, (byte)28, (byte)182, (byte)203, (byte)241, (byte)116, (byte)152, (byte)134, (byte)253, (byte)94, (byte)238, (byte)109, (byte)51, (byte)245, (byte)147, (byte)20, (byte)135, (byte)62, (byte)116, (byte)83, (byte)32, (byte)8, (byte)55, (byte)143, (byte)90, (byte)125, (byte)24, (byte)88, (byte)137, (byte)221, (byte)0, (byte)175, (byte)231, (byte)168, (byte)19, (byte)0, (byte)134, (byte)70, (byte)41, (byte)162, (byte)9, (byte)161, (byte)86, (byte)201, (byte)10, (byte)217, (byte)255, (byte)6, (byte)151, (byte)159, (byte)157, (byte)193, (byte)179, (byte)102, (byte)130, (byte)133, (byte)110, (byte)198, (byte)237, (byte)226, (byte)107, (byte)248, (byte)158, (byte)48, (byte)181, (byte)93, (byte)174, (byte)25, (byte)148, (byte)124, (byte)235, (byte)15, (byte)138, (byte)28, (byte)151, (byte)185, (byte)75, (byte)4, (byte)45, (byte)83, (byte)67, (byte)121, (byte)242, (byte)197, (byte)216, (byte)135, (byte)94, (byte)157, (byte)231, (byte)29, (byte)63, (byte)12, (byte)99, (byte)238, (byte)197, (byte)13, (byte)170, (byte)241, (byte)136, (byte)202, (byte)151, (byte)255, (byte)129, (byte)26, (byte)75, (byte)248, (byte)181, (byte)203, (byte)177}, 0) ;
            p110.target_component = (byte)(byte)147;
            p110.target_network = (byte)(byte)135;
            p110.target_system = (byte)(byte)148;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -780689777423868082L);
                Debug.Assert(pack.ts1 == (long) -8936830592744378547L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -780689777423868082L;
            p111.ts1 = (long) -8936830592744378547L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3025596062U);
                Debug.Assert(pack.time_usec == (ulong)8034894469854142593L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8034894469854142593L;
            p112.seq = (uint)3025596062U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vd == (short)(short)19405);
                Debug.Assert(pack.lat == (int)53983106);
                Debug.Assert(pack.cog == (ushort)(ushort)36409);
                Debug.Assert(pack.vel == (ushort)(ushort)34630);
                Debug.Assert(pack.alt == (int)1661858129);
                Debug.Assert(pack.fix_type == (byte)(byte)95);
                Debug.Assert(pack.lon == (int)934049610);
                Debug.Assert(pack.eph == (ushort)(ushort)52215);
                Debug.Assert(pack.time_usec == (ulong)7538116795289188490L);
                Debug.Assert(pack.vn == (short)(short)26734);
                Debug.Assert(pack.ve == (short)(short)3095);
                Debug.Assert(pack.epv == (ushort)(ushort)12136);
                Debug.Assert(pack.satellites_visible == (byte)(byte)47);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)7538116795289188490L;
            p113.vd = (short)(short)19405;
            p113.lat = (int)53983106;
            p113.satellites_visible = (byte)(byte)47;
            p113.ve = (short)(short)3095;
            p113.vn = (short)(short)26734;
            p113.vel = (ushort)(ushort)34630;
            p113.lon = (int)934049610;
            p113.epv = (ushort)(ushort)12136;
            p113.eph = (ushort)(ushort)52215;
            p113.alt = (int)1661858129;
            p113.fix_type = (byte)(byte)95;
            p113.cog = (ushort)(ushort)36409;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)8.1012675E37F);
                Debug.Assert(pack.integrated_ygyro == (float)1.9852605E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.060217E38F);
                Debug.Assert(pack.integrated_x == (float) -1.6147387E37F);
                Debug.Assert(pack.temperature == (short)(short)8413);
                Debug.Assert(pack.integrated_y == (float)3.1646816E38F);
                Debug.Assert(pack.time_usec == (ulong)4889769494260635797L);
                Debug.Assert(pack.integrated_xgyro == (float)1.5679726E38F);
                Debug.Assert(pack.quality == (byte)(byte)168);
                Debug.Assert(pack.time_delta_distance_us == (uint)1206976702U);
                Debug.Assert(pack.sensor_id == (byte)(byte)173);
                Debug.Assert(pack.integration_time_us == (uint)3777776824U);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_y = (float)3.1646816E38F;
            p114.sensor_id = (byte)(byte)173;
            p114.integrated_x = (float) -1.6147387E37F;
            p114.distance = (float)8.1012675E37F;
            p114.integrated_ygyro = (float)1.9852605E38F;
            p114.time_delta_distance_us = (uint)1206976702U;
            p114.temperature = (short)(short)8413;
            p114.quality = (byte)(byte)168;
            p114.time_usec = (ulong)4889769494260635797L;
            p114.integrated_xgyro = (float)1.5679726E38F;
            p114.integrated_zgyro = (float) -2.060217E38F;
            p114.integration_time_us = (uint)3777776824U;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)2.7810492E38F);
                Debug.Assert(pack.vz == (short)(short) -12013);
                Debug.Assert(pack.alt == (int)1771698343);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)34600);
                Debug.Assert(pack.pitchspeed == (float)5.3806713E37F);
                Debug.Assert(pack.lat == (int)915298822);
                Debug.Assert(pack.time_usec == (ulong)8929769234020060221L);
                Debug.Assert(pack.lon == (int)258390217);
                Debug.Assert(pack.vy == (short)(short) -5897);
                Debug.Assert(pack.zacc == (short)(short) -1596);
                Debug.Assert(pack.yawspeed == (float)2.2158143E38F);
                Debug.Assert(pack.yacc == (short)(short) -21766);
                Debug.Assert(pack.xacc == (short)(short) -27944);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.2019484E38F, -3.1238766E38F, 1.3960631E38F, 4.222137E37F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)30308);
                Debug.Assert(pack.vx == (short)(short)11436);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.true_airspeed = (ushort)(ushort)34600;
            p115.ind_airspeed = (ushort)(ushort)30308;
            p115.zacc = (short)(short) -1596;
            p115.yacc = (short)(short) -21766;
            p115.time_usec = (ulong)8929769234020060221L;
            p115.lat = (int)915298822;
            p115.attitude_quaternion_SET(new float[] {1.2019484E38F, -3.1238766E38F, 1.3960631E38F, 4.222137E37F}, 0) ;
            p115.vy = (short)(short) -5897;
            p115.yawspeed = (float)2.2158143E38F;
            p115.xacc = (short)(short) -27944;
            p115.pitchspeed = (float)5.3806713E37F;
            p115.alt = (int)1771698343;
            p115.lon = (int)258390217;
            p115.vx = (short)(short)11436;
            p115.rollspeed = (float)2.7810492E38F;
            p115.vz = (short)(short) -12013;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)28433);
                Debug.Assert(pack.xmag == (short)(short) -17015);
                Debug.Assert(pack.zacc == (short)(short) -304);
                Debug.Assert(pack.zmag == (short)(short)756);
                Debug.Assert(pack.ygyro == (short)(short)30544);
                Debug.Assert(pack.xgyro == (short)(short)30530);
                Debug.Assert(pack.ymag == (short)(short)16890);
                Debug.Assert(pack.zgyro == (short)(short)7609);
                Debug.Assert(pack.time_boot_ms == (uint)3325306345U);
                Debug.Assert(pack.xacc == (short)(short) -2673);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3325306345U;
            p116.xacc = (short)(short) -2673;
            p116.zgyro = (short)(short)7609;
            p116.xgyro = (short)(short)30530;
            p116.zmag = (short)(short)756;
            p116.yacc = (short)(short)28433;
            p116.xmag = (short)(short) -17015;
            p116.zacc = (short)(short) -304;
            p116.ymag = (short)(short)16890;
            p116.ygyro = (short)(short)30544;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)8246);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.end == (ushort)(ushort)36739);
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)8246;
            p117.target_system = (byte)(byte)3;
            p117.target_component = (byte)(byte)13;
            p117.end = (ushort)(ushort)36739;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)26206);
                Debug.Assert(pack.id == (ushort)(ushort)11460);
                Debug.Assert(pack.time_utc == (uint)2237717814U);
                Debug.Assert(pack.size == (uint)3152572006U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)32335);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)11460;
            p118.time_utc = (uint)2237717814U;
            p118.num_logs = (ushort)(ushort)26206;
            p118.last_log_num = (ushort)(ushort)32335;
            p118.size = (uint)3152572006U;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3232430230U);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.id == (ushort)(ushort)51220);
                Debug.Assert(pack.count == (uint)2484678109U);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)2484678109U;
            p119.target_system = (byte)(byte)77;
            p119.target_component = (byte)(byte)57;
            p119.ofs = (uint)3232430230U;
            p119.id = (ushort)(ushort)51220;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)78, (byte)233, (byte)218, (byte)22, (byte)84, (byte)194, (byte)188, (byte)104, (byte)56, (byte)132, (byte)136, (byte)22, (byte)50, (byte)202, (byte)146, (byte)51, (byte)95, (byte)39, (byte)255, (byte)252, (byte)58, (byte)184, (byte)239, (byte)253, (byte)33, (byte)112, (byte)192, (byte)134, (byte)67, (byte)238, (byte)111, (byte)214, (byte)249, (byte)242, (byte)56, (byte)117, (byte)180, (byte)199, (byte)159, (byte)119, (byte)158, (byte)117, (byte)194, (byte)121, (byte)20, (byte)13, (byte)142, (byte)118, (byte)192, (byte)124, (byte)140, (byte)48, (byte)18, (byte)103, (byte)197, (byte)152, (byte)139, (byte)208, (byte)194, (byte)76, (byte)31, (byte)44, (byte)246, (byte)21, (byte)124, (byte)99, (byte)108, (byte)217, (byte)88, (byte)38, (byte)214, (byte)56, (byte)247, (byte)61, (byte)99, (byte)75, (byte)34, (byte)84, (byte)160, (byte)69, (byte)189, (byte)189, (byte)25, (byte)69, (byte)114, (byte)162, (byte)101, (byte)251, (byte)207, (byte)89}));
                Debug.Assert(pack.count == (byte)(byte)29);
                Debug.Assert(pack.ofs == (uint)751684048U);
                Debug.Assert(pack.id == (ushort)(ushort)30251);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)751684048U;
            p120.data__SET(new byte[] {(byte)78, (byte)233, (byte)218, (byte)22, (byte)84, (byte)194, (byte)188, (byte)104, (byte)56, (byte)132, (byte)136, (byte)22, (byte)50, (byte)202, (byte)146, (byte)51, (byte)95, (byte)39, (byte)255, (byte)252, (byte)58, (byte)184, (byte)239, (byte)253, (byte)33, (byte)112, (byte)192, (byte)134, (byte)67, (byte)238, (byte)111, (byte)214, (byte)249, (byte)242, (byte)56, (byte)117, (byte)180, (byte)199, (byte)159, (byte)119, (byte)158, (byte)117, (byte)194, (byte)121, (byte)20, (byte)13, (byte)142, (byte)118, (byte)192, (byte)124, (byte)140, (byte)48, (byte)18, (byte)103, (byte)197, (byte)152, (byte)139, (byte)208, (byte)194, (byte)76, (byte)31, (byte)44, (byte)246, (byte)21, (byte)124, (byte)99, (byte)108, (byte)217, (byte)88, (byte)38, (byte)214, (byte)56, (byte)247, (byte)61, (byte)99, (byte)75, (byte)34, (byte)84, (byte)160, (byte)69, (byte)189, (byte)189, (byte)25, (byte)69, (byte)114, (byte)162, (byte)101, (byte)251, (byte)207, (byte)89}, 0) ;
            p120.count = (byte)(byte)29;
            p120.id = (ushort)(ushort)30251;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.target_component == (byte)(byte)197);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)197;
            p121.target_system = (byte)(byte)93;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.target_component == (byte)(byte)226);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)48;
            p122.target_component = (byte)(byte)226;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.len == (byte)(byte)117);
                Debug.Assert(pack.target_system == (byte)(byte)215);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)171, (byte)13, (byte)164, (byte)165, (byte)71, (byte)165, (byte)140, (byte)116, (byte)57, (byte)231, (byte)0, (byte)210, (byte)94, (byte)123, (byte)13, (byte)54, (byte)243, (byte)182, (byte)176, (byte)2, (byte)80, (byte)229, (byte)130, (byte)138, (byte)235, (byte)154, (byte)70, (byte)161, (byte)34, (byte)240, (byte)162, (byte)236, (byte)254, (byte)15, (byte)160, (byte)14, (byte)1, (byte)212, (byte)209, (byte)2, (byte)198, (byte)252, (byte)18, (byte)195, (byte)146, (byte)228, (byte)31, (byte)43, (byte)150, (byte)102, (byte)9, (byte)116, (byte)216, (byte)244, (byte)133, (byte)142, (byte)101, (byte)231, (byte)29, (byte)169, (byte)199, (byte)175, (byte)47, (byte)18, (byte)140, (byte)30, (byte)100, (byte)210, (byte)38, (byte)0, (byte)12, (byte)95, (byte)51, (byte)154, (byte)247, (byte)102, (byte)21, (byte)166, (byte)35, (byte)56, (byte)117, (byte)212, (byte)220, (byte)61, (byte)191, (byte)240, (byte)210, (byte)236, (byte)120, (byte)16, (byte)29, (byte)236, (byte)31, (byte)244, (byte)247, (byte)150, (byte)22, (byte)158, (byte)38, (byte)246, (byte)169, (byte)155, (byte)82, (byte)203, (byte)183, (byte)66, (byte)58, (byte)192, (byte)86, (byte)240}));
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)215;
            p123.data__SET(new byte[] {(byte)171, (byte)13, (byte)164, (byte)165, (byte)71, (byte)165, (byte)140, (byte)116, (byte)57, (byte)231, (byte)0, (byte)210, (byte)94, (byte)123, (byte)13, (byte)54, (byte)243, (byte)182, (byte)176, (byte)2, (byte)80, (byte)229, (byte)130, (byte)138, (byte)235, (byte)154, (byte)70, (byte)161, (byte)34, (byte)240, (byte)162, (byte)236, (byte)254, (byte)15, (byte)160, (byte)14, (byte)1, (byte)212, (byte)209, (byte)2, (byte)198, (byte)252, (byte)18, (byte)195, (byte)146, (byte)228, (byte)31, (byte)43, (byte)150, (byte)102, (byte)9, (byte)116, (byte)216, (byte)244, (byte)133, (byte)142, (byte)101, (byte)231, (byte)29, (byte)169, (byte)199, (byte)175, (byte)47, (byte)18, (byte)140, (byte)30, (byte)100, (byte)210, (byte)38, (byte)0, (byte)12, (byte)95, (byte)51, (byte)154, (byte)247, (byte)102, (byte)21, (byte)166, (byte)35, (byte)56, (byte)117, (byte)212, (byte)220, (byte)61, (byte)191, (byte)240, (byte)210, (byte)236, (byte)120, (byte)16, (byte)29, (byte)236, (byte)31, (byte)244, (byte)247, (byte)150, (byte)22, (byte)158, (byte)38, (byte)246, (byte)169, (byte)155, (byte)82, (byte)203, (byte)183, (byte)66, (byte)58, (byte)192, (byte)86, (byte)240}, 0) ;
            p123.len = (byte)(byte)117;
            p123.target_component = (byte)(byte)63;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)8827);
                Debug.Assert(pack.lon == (int) -1695435564);
                Debug.Assert(pack.cog == (ushort)(ushort)59163);
                Debug.Assert(pack.lat == (int) -131390877);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.satellites_visible == (byte)(byte)66);
                Debug.Assert(pack.dgps_age == (uint)3739159359U);
                Debug.Assert(pack.time_usec == (ulong)2526006822912318036L);
                Debug.Assert(pack.vel == (ushort)(ushort)46134);
                Debug.Assert(pack.dgps_numch == (byte)(byte)102);
                Debug.Assert(pack.alt == (int) -1861178576);
                Debug.Assert(pack.epv == (ushort)(ushort)29570);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2526006822912318036L;
            p124.dgps_age = (uint)3739159359U;
            p124.lon = (int) -1695435564;
            p124.cog = (ushort)(ushort)59163;
            p124.alt = (int) -1861178576;
            p124.eph = (ushort)(ushort)8827;
            p124.epv = (ushort)(ushort)29570;
            p124.vel = (ushort)(ushort)46134;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.satellites_visible = (byte)(byte)66;
            p124.dgps_numch = (byte)(byte)102;
            p124.lat = (int) -131390877;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
                Debug.Assert(pack.Vcc == (ushort)(ushort)19373);
                Debug.Assert(pack.Vservo == (ushort)(ushort)51623);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
            p125.Vservo = (ushort)(ushort)51623;
            p125.Vcc = (ushort)(ushort)19373;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)3149540485U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)127, (byte)35, (byte)232, (byte)190, (byte)138, (byte)106, (byte)228, (byte)170, (byte)3, (byte)13, (byte)176, (byte)203, (byte)95, (byte)216, (byte)226, (byte)236, (byte)15, (byte)164, (byte)11, (byte)6, (byte)83, (byte)220, (byte)79, (byte)51, (byte)204, (byte)179, (byte)101, (byte)226, (byte)30, (byte)180, (byte)89, (byte)175, (byte)212, (byte)151, (byte)160, (byte)226, (byte)124, (byte)46, (byte)188, (byte)154, (byte)121, (byte)44, (byte)164, (byte)151, (byte)166, (byte)27, (byte)60, (byte)56, (byte)86, (byte)247, (byte)99, (byte)182, (byte)124, (byte)222, (byte)213, (byte)34, (byte)69, (byte)140, (byte)223, (byte)29, (byte)149, (byte)169, (byte)52, (byte)126, (byte)110, (byte)60, (byte)136, (byte)99, (byte)247, (byte)161}));
                Debug.Assert(pack.count == (byte)(byte)197);
                Debug.Assert(pack.timeout == (ushort)(ushort)39572);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.data__SET(new byte[] {(byte)127, (byte)35, (byte)232, (byte)190, (byte)138, (byte)106, (byte)228, (byte)170, (byte)3, (byte)13, (byte)176, (byte)203, (byte)95, (byte)216, (byte)226, (byte)236, (byte)15, (byte)164, (byte)11, (byte)6, (byte)83, (byte)220, (byte)79, (byte)51, (byte)204, (byte)179, (byte)101, (byte)226, (byte)30, (byte)180, (byte)89, (byte)175, (byte)212, (byte)151, (byte)160, (byte)226, (byte)124, (byte)46, (byte)188, (byte)154, (byte)121, (byte)44, (byte)164, (byte)151, (byte)166, (byte)27, (byte)60, (byte)56, (byte)86, (byte)247, (byte)99, (byte)182, (byte)124, (byte)222, (byte)213, (byte)34, (byte)69, (byte)140, (byte)223, (byte)29, (byte)149, (byte)169, (byte)52, (byte)126, (byte)110, (byte)60, (byte)136, (byte)99, (byte)247, (byte)161}, 0) ;
            p126.baudrate = (uint)3149540485U;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING;
            p126.timeout = (ushort)(ushort)39572;
            p126.count = (byte)(byte)197;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int)1514671741);
                Debug.Assert(pack.baseline_b_mm == (int) -1893963372);
                Debug.Assert(pack.baseline_a_mm == (int)1502145144);
                Debug.Assert(pack.time_last_baseline_ms == (uint)530819312U);
                Debug.Assert(pack.rtk_health == (byte)(byte)156);
                Debug.Assert(pack.accuracy == (uint)3641295324U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)230);
                Debug.Assert(pack.wn == (ushort)(ushort)47527);
                Debug.Assert(pack.nsats == (byte)(byte)83);
                Debug.Assert(pack.rtk_rate == (byte)(byte)138);
                Debug.Assert(pack.tow == (uint)3274189593U);
                Debug.Assert(pack.iar_num_hypotheses == (int)390111215);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)229);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.tow = (uint)3274189593U;
            p127.baseline_b_mm = (int) -1893963372;
            p127.time_last_baseline_ms = (uint)530819312U;
            p127.iar_num_hypotheses = (int)390111215;
            p127.rtk_receiver_id = (byte)(byte)230;
            p127.rtk_health = (byte)(byte)156;
            p127.baseline_c_mm = (int)1514671741;
            p127.nsats = (byte)(byte)83;
            p127.rtk_rate = (byte)(byte)138;
            p127.baseline_a_mm = (int)1502145144;
            p127.accuracy = (uint)3641295324U;
            p127.wn = (ushort)(ushort)47527;
            p127.baseline_coords_type = (byte)(byte)229;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)2489200900U);
                Debug.Assert(pack.nsats == (byte)(byte)91);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)244);
                Debug.Assert(pack.wn == (ushort)(ushort)26520);
                Debug.Assert(pack.iar_num_hypotheses == (int)591295058);
                Debug.Assert(pack.rtk_health == (byte)(byte)111);
                Debug.Assert(pack.baseline_b_mm == (int) -452370871);
                Debug.Assert(pack.baseline_a_mm == (int)2134042108);
                Debug.Assert(pack.rtk_rate == (byte)(byte)15);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)91);
                Debug.Assert(pack.accuracy == (uint)1545590868U);
                Debug.Assert(pack.tow == (uint)1566840234U);
                Debug.Assert(pack.baseline_c_mm == (int)691346418);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_health = (byte)(byte)111;
            p128.baseline_b_mm = (int) -452370871;
            p128.time_last_baseline_ms = (uint)2489200900U;
            p128.baseline_a_mm = (int)2134042108;
            p128.iar_num_hypotheses = (int)591295058;
            p128.nsats = (byte)(byte)91;
            p128.wn = (ushort)(ushort)26520;
            p128.accuracy = (uint)1545590868U;
            p128.rtk_receiver_id = (byte)(byte)244;
            p128.rtk_rate = (byte)(byte)15;
            p128.tow = (uint)1566840234U;
            p128.baseline_c_mm = (int)691346418;
            p128.baseline_coords_type = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -17712);
                Debug.Assert(pack.xacc == (short)(short) -29992);
                Debug.Assert(pack.time_boot_ms == (uint)1917217987U);
                Debug.Assert(pack.yacc == (short)(short)20203);
                Debug.Assert(pack.zmag == (short)(short)29789);
                Debug.Assert(pack.ymag == (short)(short) -4590);
                Debug.Assert(pack.xmag == (short)(short)3163);
                Debug.Assert(pack.ygyro == (short)(short)5708);
                Debug.Assert(pack.zacc == (short)(short)22796);
                Debug.Assert(pack.zgyro == (short)(short)28139);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short)3163;
            p129.ymag = (short)(short) -4590;
            p129.xgyro = (short)(short) -17712;
            p129.time_boot_ms = (uint)1917217987U;
            p129.zmag = (short)(short)29789;
            p129.zacc = (short)(short)22796;
            p129.xacc = (short)(short) -29992;
            p129.zgyro = (short)(short)28139;
            p129.ygyro = (short)(short)5708;
            p129.yacc = (short)(short)20203;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)29274);
                Debug.Assert(pack.type == (byte)(byte)64);
                Debug.Assert(pack.payload == (byte)(byte)86);
                Debug.Assert(pack.jpg_quality == (byte)(byte)222);
                Debug.Assert(pack.size == (uint)3970262814U);
                Debug.Assert(pack.width == (ushort)(ushort)20569);
                Debug.Assert(pack.packets == (ushort)(ushort)3371);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)64;
            p130.size = (uint)3970262814U;
            p130.width = (ushort)(ushort)20569;
            p130.packets = (ushort)(ushort)3371;
            p130.jpg_quality = (byte)(byte)222;
            p130.payload = (byte)(byte)86;
            p130.height = (ushort)(ushort)29274;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)16913);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)185, (byte)93, (byte)251, (byte)196, (byte)242, (byte)100, (byte)71, (byte)182, (byte)14, (byte)54, (byte)77, (byte)43, (byte)228, (byte)166, (byte)51, (byte)84, (byte)140, (byte)242, (byte)23, (byte)57, (byte)67, (byte)23, (byte)200, (byte)129, (byte)139, (byte)8, (byte)139, (byte)165, (byte)13, (byte)12, (byte)75, (byte)10, (byte)58, (byte)123, (byte)67, (byte)136, (byte)97, (byte)137, (byte)185, (byte)111, (byte)26, (byte)128, (byte)116, (byte)79, (byte)85, (byte)105, (byte)219, (byte)251, (byte)101, (byte)30, (byte)97, (byte)57, (byte)223, (byte)153, (byte)95, (byte)17, (byte)92, (byte)207, (byte)119, (byte)208, (byte)93, (byte)187, (byte)3, (byte)205, (byte)8, (byte)131, (byte)168, (byte)57, (byte)174, (byte)205, (byte)22, (byte)250, (byte)113, (byte)226, (byte)149, (byte)145, (byte)165, (byte)136, (byte)138, (byte)203, (byte)196, (byte)232, (byte)236, (byte)175, (byte)36, (byte)203, (byte)127, (byte)29, (byte)77, (byte)188, (byte)232, (byte)34, (byte)170, (byte)152, (byte)60, (byte)116, (byte)162, (byte)114, (byte)149, (byte)243, (byte)239, (byte)83, (byte)9, (byte)77, (byte)79, (byte)190, (byte)15, (byte)228, (byte)118, (byte)74, (byte)131, (byte)54, (byte)90, (byte)253, (byte)14, (byte)251, (byte)222, (byte)185, (byte)62, (byte)73, (byte)38, (byte)166, (byte)224, (byte)255, (byte)141, (byte)129, (byte)17, (byte)36, (byte)107, (byte)214, (byte)66, (byte)90, (byte)224, (byte)3, (byte)76, (byte)190, (byte)167, (byte)231, (byte)17, (byte)222, (byte)143, (byte)210, (byte)70, (byte)19, (byte)81, (byte)249, (byte)137, (byte)170, (byte)89, (byte)6, (byte)207, (byte)110, (byte)238, (byte)134, (byte)16, (byte)17, (byte)234, (byte)98, (byte)162, (byte)193, (byte)90, (byte)239, (byte)5, (byte)15, (byte)156, (byte)53, (byte)97, (byte)180, (byte)96, (byte)76, (byte)143, (byte)115, (byte)86, (byte)54, (byte)172, (byte)30, (byte)9, (byte)85, (byte)236, (byte)169, (byte)246, (byte)193, (byte)247, (byte)197, (byte)105, (byte)185, (byte)139, (byte)118, (byte)137, (byte)112, (byte)43, (byte)62, (byte)180, (byte)183, (byte)34, (byte)218, (byte)82, (byte)153, (byte)195, (byte)120, (byte)62, (byte)67, (byte)152, (byte)79, (byte)228, (byte)11, (byte)28, (byte)94, (byte)48, (byte)99, (byte)34, (byte)60, (byte)37, (byte)161, (byte)35, (byte)55, (byte)67, (byte)235, (byte)118, (byte)59, (byte)191, (byte)139, (byte)45, (byte)153, (byte)215, (byte)202, (byte)62, (byte)45, (byte)252, (byte)229, (byte)16, (byte)144, (byte)119, (byte)199, (byte)130, (byte)42, (byte)106, (byte)42, (byte)27, (byte)93, (byte)79, (byte)227, (byte)10, (byte)127, (byte)206, (byte)228, (byte)49, (byte)112, (byte)214, (byte)79, (byte)38, (byte)212, (byte)245}));
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)16913;
            p131.data__SET(new byte[] {(byte)185, (byte)93, (byte)251, (byte)196, (byte)242, (byte)100, (byte)71, (byte)182, (byte)14, (byte)54, (byte)77, (byte)43, (byte)228, (byte)166, (byte)51, (byte)84, (byte)140, (byte)242, (byte)23, (byte)57, (byte)67, (byte)23, (byte)200, (byte)129, (byte)139, (byte)8, (byte)139, (byte)165, (byte)13, (byte)12, (byte)75, (byte)10, (byte)58, (byte)123, (byte)67, (byte)136, (byte)97, (byte)137, (byte)185, (byte)111, (byte)26, (byte)128, (byte)116, (byte)79, (byte)85, (byte)105, (byte)219, (byte)251, (byte)101, (byte)30, (byte)97, (byte)57, (byte)223, (byte)153, (byte)95, (byte)17, (byte)92, (byte)207, (byte)119, (byte)208, (byte)93, (byte)187, (byte)3, (byte)205, (byte)8, (byte)131, (byte)168, (byte)57, (byte)174, (byte)205, (byte)22, (byte)250, (byte)113, (byte)226, (byte)149, (byte)145, (byte)165, (byte)136, (byte)138, (byte)203, (byte)196, (byte)232, (byte)236, (byte)175, (byte)36, (byte)203, (byte)127, (byte)29, (byte)77, (byte)188, (byte)232, (byte)34, (byte)170, (byte)152, (byte)60, (byte)116, (byte)162, (byte)114, (byte)149, (byte)243, (byte)239, (byte)83, (byte)9, (byte)77, (byte)79, (byte)190, (byte)15, (byte)228, (byte)118, (byte)74, (byte)131, (byte)54, (byte)90, (byte)253, (byte)14, (byte)251, (byte)222, (byte)185, (byte)62, (byte)73, (byte)38, (byte)166, (byte)224, (byte)255, (byte)141, (byte)129, (byte)17, (byte)36, (byte)107, (byte)214, (byte)66, (byte)90, (byte)224, (byte)3, (byte)76, (byte)190, (byte)167, (byte)231, (byte)17, (byte)222, (byte)143, (byte)210, (byte)70, (byte)19, (byte)81, (byte)249, (byte)137, (byte)170, (byte)89, (byte)6, (byte)207, (byte)110, (byte)238, (byte)134, (byte)16, (byte)17, (byte)234, (byte)98, (byte)162, (byte)193, (byte)90, (byte)239, (byte)5, (byte)15, (byte)156, (byte)53, (byte)97, (byte)180, (byte)96, (byte)76, (byte)143, (byte)115, (byte)86, (byte)54, (byte)172, (byte)30, (byte)9, (byte)85, (byte)236, (byte)169, (byte)246, (byte)193, (byte)247, (byte)197, (byte)105, (byte)185, (byte)139, (byte)118, (byte)137, (byte)112, (byte)43, (byte)62, (byte)180, (byte)183, (byte)34, (byte)218, (byte)82, (byte)153, (byte)195, (byte)120, (byte)62, (byte)67, (byte)152, (byte)79, (byte)228, (byte)11, (byte)28, (byte)94, (byte)48, (byte)99, (byte)34, (byte)60, (byte)37, (byte)161, (byte)35, (byte)55, (byte)67, (byte)235, (byte)118, (byte)59, (byte)191, (byte)139, (byte)45, (byte)153, (byte)215, (byte)202, (byte)62, (byte)45, (byte)252, (byte)229, (byte)16, (byte)144, (byte)119, (byte)199, (byte)130, (byte)42, (byte)106, (byte)42, (byte)27, (byte)93, (byte)79, (byte)227, (byte)10, (byte)127, (byte)206, (byte)228, (byte)49, (byte)112, (byte)214, (byte)79, (byte)38, (byte)212, (byte)245}, 0) ;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)63018);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90);
                Debug.Assert(pack.time_boot_ms == (uint)1884048588U);
                Debug.Assert(pack.covariance == (byte)(byte)72);
                Debug.Assert(pack.id == (byte)(byte)144);
                Debug.Assert(pack.min_distance == (ushort)(ushort)4824);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.max_distance == (ushort)(ushort)7619);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)4824;
            p132.max_distance = (ushort)(ushort)7619;
            p132.time_boot_ms = (uint)1884048588U;
            p132.id = (byte)(byte)144;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_90;
            p132.covariance = (byte)(byte)72;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.current_distance = (ushort)(ushort)63018;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1042927873);
                Debug.Assert(pack.mask == (ulong)9124305915992183934L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)53620);
                Debug.Assert(pack.lat == (int) -304137576);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -304137576;
            p133.lon = (int)1042927873;
            p133.mask = (ulong)9124305915992183934L;
            p133.grid_spacing = (ushort)(ushort)53620;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)183);
                Debug.Assert(pack.lon == (int) -721381613);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)4736, (short) -19663, (short)14554, (short)26934, (short) -30217, (short)11814, (short) -18659, (short) -22653, (short) -28492, (short) -15949, (short)20593, (short)21726, (short)2867, (short)24308, (short)960, (short) -22463}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)2194);
                Debug.Assert(pack.lat == (int) -39206906);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int) -721381613;
            p134.grid_spacing = (ushort)(ushort)2194;
            p134.data__SET(new short[] {(short)4736, (short) -19663, (short)14554, (short)26934, (short) -30217, (short)11814, (short) -18659, (short) -22653, (short) -28492, (short) -15949, (short)20593, (short)21726, (short)2867, (short)24308, (short)960, (short) -22463}, 0) ;
            p134.lat = (int) -39206906;
            p134.gridbit = (byte)(byte)183;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)800195861);
                Debug.Assert(pack.lat == (int) -2013748744);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -2013748744;
            p135.lon = (int)800195861;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)33320);
                Debug.Assert(pack.lat == (int)1690752003);
                Debug.Assert(pack.lon == (int)1198239484);
                Debug.Assert(pack.current_height == (float)1.2363359E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)55115);
                Debug.Assert(pack.spacing == (ushort)(ushort)61559);
                Debug.Assert(pack.terrain_height == (float) -2.372515E38F);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.terrain_height = (float) -2.372515E38F;
            p136.current_height = (float)1.2363359E38F;
            p136.spacing = (ushort)(ushort)61559;
            p136.pending = (ushort)(ushort)33320;
            p136.loaded = (ushort)(ushort)55115;
            p136.lat = (int)1690752003;
            p136.lon = (int)1198239484;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -3.3122921E38F);
                Debug.Assert(pack.temperature == (short)(short) -7382);
                Debug.Assert(pack.time_boot_ms == (uint)254765245U);
                Debug.Assert(pack.press_diff == (float)2.6088095E38F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float) -3.3122921E38F;
            p137.temperature = (short)(short) -7382;
            p137.time_boot_ms = (uint)254765245U;
            p137.press_diff = (float)2.6088095E38F;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4596919E38F, -2.228175E38F, -7.8547115E37F, 5.8486174E37F}));
                Debug.Assert(pack.z == (float)4.429194E37F);
                Debug.Assert(pack.time_usec == (ulong)4772334522297035588L);
                Debug.Assert(pack.x == (float) -4.1902756E37F);
                Debug.Assert(pack.y == (float) -3.5194273E37F);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -3.5194273E37F;
            p138.time_usec = (ulong)4772334522297035588L;
            p138.z = (float)4.429194E37F;
            p138.x = (float) -4.1902756E37F;
            p138.q_SET(new float[] {1.4596919E38F, -2.228175E38F, -7.8547115E37F, 5.8486174E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)217);
                Debug.Assert(pack.group_mlx == (byte)(byte)25);
                Debug.Assert(pack.time_usec == (ulong)2629887565527259852L);
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.6845565E37F, -9.014947E37F, 8.1758276E37F, -3.0325736E38F, 1.5284283E38F, -2.5690544E38F, -4.8011704E37F, -2.2947653E37F}));
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)217;
            p139.target_component = (byte)(byte)236;
            p139.time_usec = (ulong)2629887565527259852L;
            p139.group_mlx = (byte)(byte)25;
            p139.controls_SET(new float[] {-3.6845565E37F, -9.014947E37F, 8.1758276E37F, -3.0325736E38F, 1.5284283E38F, -2.5690544E38F, -4.8011704E37F, -2.2947653E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.9003233E38F, -1.4390486E37F, -1.861085E38F, -1.2021015E38F, -4.127475E37F, -2.966158E38F, -1.8651012E38F, 1.0868963E38F}));
                Debug.Assert(pack.time_usec == (ulong)6381204318564018227L);
                Debug.Assert(pack.group_mlx == (byte)(byte)241);
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)241;
            p140.time_usec = (ulong)6381204318564018227L;
            p140.controls_SET(new float[] {2.9003233E38F, -1.4390486E37F, -1.861085E38F, -1.2021015E38F, -4.127475E37F, -2.966158E38F, -1.8651012E38F, 1.0868963E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float) -1.2123159E38F);
                Debug.Assert(pack.time_usec == (ulong)5141604111052011401L);
                Debug.Assert(pack.altitude_amsl == (float) -1.8619506E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.3311036E38F);
                Debug.Assert(pack.bottom_clearance == (float)1.197686E38F);
                Debug.Assert(pack.altitude_relative == (float) -3.1029215E38F);
                Debug.Assert(pack.altitude_local == (float) -1.8737103E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -1.8737103E38F;
            p141.altitude_amsl = (float) -1.8619506E38F;
            p141.bottom_clearance = (float)1.197686E38F;
            p141.altitude_monotonic = (float) -1.2123159E38F;
            p141.time_usec = (ulong)5141604111052011401L;
            p141.altitude_relative = (float) -3.1029215E38F;
            p141.altitude_terrain = (float)2.3311036E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)236, (byte)85, (byte)174, (byte)75, (byte)215, (byte)125, (byte)129, (byte)222, (byte)6, (byte)6, (byte)196, (byte)227, (byte)233, (byte)189, (byte)136, (byte)98, (byte)158, (byte)114, (byte)186, (byte)186, (byte)171, (byte)123, (byte)11, (byte)62, (byte)147, (byte)128, (byte)208, (byte)149, (byte)17, (byte)115, (byte)193, (byte)41, (byte)219, (byte)73, (byte)95, (byte)4, (byte)41, (byte)61, (byte)116, (byte)8, (byte)241, (byte)241, (byte)179, (byte)19, (byte)18, (byte)49, (byte)16, (byte)39, (byte)11, (byte)250, (byte)138, (byte)102, (byte)34, (byte)81, (byte)106, (byte)59, (byte)41, (byte)0, (byte)240, (byte)30, (byte)151, (byte)230, (byte)162, (byte)18, (byte)24, (byte)33, (byte)138, (byte)128, (byte)56, (byte)90, (byte)85, (byte)101, (byte)203, (byte)158, (byte)105, (byte)175, (byte)9, (byte)178, (byte)17, (byte)99, (byte)186, (byte)239, (byte)113, (byte)91, (byte)197, (byte)63, (byte)33, (byte)123, (byte)24, (byte)252, (byte)187, (byte)205, (byte)179, (byte)157, (byte)109, (byte)131, (byte)117, (byte)52, (byte)80, (byte)132, (byte)139, (byte)228, (byte)160, (byte)242, (byte)4, (byte)158, (byte)73, (byte)165, (byte)227, (byte)231, (byte)116, (byte)51, (byte)112, (byte)146, (byte)243, (byte)26, (byte)62, (byte)146, (byte)220, (byte)109}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)75, (byte)59, (byte)38, (byte)50, (byte)208, (byte)237, (byte)238, (byte)219, (byte)97, (byte)166, (byte)135, (byte)136, (byte)27, (byte)139, (byte)108, (byte)168, (byte)123, (byte)120, (byte)22, (byte)163, (byte)169, (byte)146, (byte)234, (byte)233, (byte)20, (byte)82, (byte)174, (byte)200, (byte)53, (byte)135, (byte)56, (byte)38, (byte)21, (byte)87, (byte)243, (byte)11, (byte)144, (byte)254, (byte)121, (byte)142, (byte)235, (byte)37, (byte)203, (byte)16, (byte)152, (byte)208, (byte)22, (byte)149, (byte)221, (byte)230, (byte)208, (byte)104, (byte)169, (byte)18, (byte)145, (byte)117, (byte)179, (byte)116, (byte)42, (byte)27, (byte)10, (byte)112, (byte)241, (byte)109, (byte)182, (byte)146, (byte)224, (byte)103, (byte)64, (byte)182, (byte)250, (byte)68, (byte)107, (byte)15, (byte)99, (byte)206, (byte)236, (byte)130, (byte)209, (byte)181, (byte)207, (byte)92, (byte)188, (byte)179, (byte)65, (byte)242, (byte)39, (byte)19, (byte)135, (byte)199, (byte)226, (byte)38, (byte)12, (byte)233, (byte)38, (byte)34, (byte)218, (byte)172, (byte)195, (byte)75, (byte)21, (byte)62, (byte)119, (byte)236, (byte)157, (byte)188, (byte)232, (byte)35, (byte)155, (byte)238, (byte)174, (byte)247, (byte)130, (byte)164, (byte)190, (byte)61, (byte)42, (byte)202, (byte)26, (byte)65}));
                Debug.Assert(pack.transfer_type == (byte)(byte)145);
                Debug.Assert(pack.uri_type == (byte)(byte)192);
                Debug.Assert(pack.request_id == (byte)(byte)105);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)105;
            p142.uri_type = (byte)(byte)192;
            p142.uri_SET(new byte[] {(byte)236, (byte)85, (byte)174, (byte)75, (byte)215, (byte)125, (byte)129, (byte)222, (byte)6, (byte)6, (byte)196, (byte)227, (byte)233, (byte)189, (byte)136, (byte)98, (byte)158, (byte)114, (byte)186, (byte)186, (byte)171, (byte)123, (byte)11, (byte)62, (byte)147, (byte)128, (byte)208, (byte)149, (byte)17, (byte)115, (byte)193, (byte)41, (byte)219, (byte)73, (byte)95, (byte)4, (byte)41, (byte)61, (byte)116, (byte)8, (byte)241, (byte)241, (byte)179, (byte)19, (byte)18, (byte)49, (byte)16, (byte)39, (byte)11, (byte)250, (byte)138, (byte)102, (byte)34, (byte)81, (byte)106, (byte)59, (byte)41, (byte)0, (byte)240, (byte)30, (byte)151, (byte)230, (byte)162, (byte)18, (byte)24, (byte)33, (byte)138, (byte)128, (byte)56, (byte)90, (byte)85, (byte)101, (byte)203, (byte)158, (byte)105, (byte)175, (byte)9, (byte)178, (byte)17, (byte)99, (byte)186, (byte)239, (byte)113, (byte)91, (byte)197, (byte)63, (byte)33, (byte)123, (byte)24, (byte)252, (byte)187, (byte)205, (byte)179, (byte)157, (byte)109, (byte)131, (byte)117, (byte)52, (byte)80, (byte)132, (byte)139, (byte)228, (byte)160, (byte)242, (byte)4, (byte)158, (byte)73, (byte)165, (byte)227, (byte)231, (byte)116, (byte)51, (byte)112, (byte)146, (byte)243, (byte)26, (byte)62, (byte)146, (byte)220, (byte)109}, 0) ;
            p142.transfer_type = (byte)(byte)145;
            p142.storage_SET(new byte[] {(byte)75, (byte)59, (byte)38, (byte)50, (byte)208, (byte)237, (byte)238, (byte)219, (byte)97, (byte)166, (byte)135, (byte)136, (byte)27, (byte)139, (byte)108, (byte)168, (byte)123, (byte)120, (byte)22, (byte)163, (byte)169, (byte)146, (byte)234, (byte)233, (byte)20, (byte)82, (byte)174, (byte)200, (byte)53, (byte)135, (byte)56, (byte)38, (byte)21, (byte)87, (byte)243, (byte)11, (byte)144, (byte)254, (byte)121, (byte)142, (byte)235, (byte)37, (byte)203, (byte)16, (byte)152, (byte)208, (byte)22, (byte)149, (byte)221, (byte)230, (byte)208, (byte)104, (byte)169, (byte)18, (byte)145, (byte)117, (byte)179, (byte)116, (byte)42, (byte)27, (byte)10, (byte)112, (byte)241, (byte)109, (byte)182, (byte)146, (byte)224, (byte)103, (byte)64, (byte)182, (byte)250, (byte)68, (byte)107, (byte)15, (byte)99, (byte)206, (byte)236, (byte)130, (byte)209, (byte)181, (byte)207, (byte)92, (byte)188, (byte)179, (byte)65, (byte)242, (byte)39, (byte)19, (byte)135, (byte)199, (byte)226, (byte)38, (byte)12, (byte)233, (byte)38, (byte)34, (byte)218, (byte)172, (byte)195, (byte)75, (byte)21, (byte)62, (byte)119, (byte)236, (byte)157, (byte)188, (byte)232, (byte)35, (byte)155, (byte)238, (byte)174, (byte)247, (byte)130, (byte)164, (byte)190, (byte)61, (byte)42, (byte)202, (byte)26, (byte)65}, 0) ;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)20279);
                Debug.Assert(pack.time_boot_ms == (uint)1901436362U);
                Debug.Assert(pack.press_diff == (float) -2.668595E38F);
                Debug.Assert(pack.press_abs == (float)2.2056181E38F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float) -2.668595E38F;
            p143.press_abs = (float)2.2056181E38F;
            p143.time_boot_ms = (uint)1901436362U;
            p143.temperature = (short)(short)20279;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)964746824);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.9472229E38F, -6.251799E37F, 4.709881E37F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.249663E38F, -2.3427865E38F, 2.7198302E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {7.321134E37F, -1.1693604E37F, -7.973622E37F, 1.5093504E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.4105808E38F, 1.6671207E38F, 1.6259689E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.0519479E38F, 9.726555E37F, -3.2795977E38F}));
                Debug.Assert(pack.lon == (int)1284523498);
                Debug.Assert(pack.alt == (float) -1.0616087E38F);
                Debug.Assert(pack.timestamp == (ulong)8807312590353677752L);
                Debug.Assert(pack.est_capabilities == (byte)(byte)140);
                Debug.Assert(pack.custom_state == (ulong)6852248094456320696L);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)8807312590353677752L;
            p144.position_cov_SET(new float[] {3.249663E38F, -2.3427865E38F, 2.7198302E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)140;
            p144.lat = (int)964746824;
            p144.custom_state = (ulong)6852248094456320696L;
            p144.vel_SET(new float[] {-2.0519479E38F, 9.726555E37F, -3.2795977E38F}, 0) ;
            p144.acc_SET(new float[] {1.4105808E38F, 1.6671207E38F, 1.6259689E38F}, 0) ;
            p144.lon = (int)1284523498;
            p144.attitude_q_SET(new float[] {7.321134E37F, -1.1693604E37F, -7.973622E37F, 1.5093504E38F}, 0) ;
            p144.rates_SET(new float[] {-1.9472229E38F, -6.251799E37F, 4.709881E37F}, 0) ;
            p144.alt = (float) -1.0616087E38F;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_acc == (float)2.4003926E38F);
                Debug.Assert(pack.y_acc == (float)2.9287588E35F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.9843915E38F, -2.5109481E38F, 1.910792E38F}));
                Debug.Assert(pack.y_vel == (float) -1.7067254E38F);
                Debug.Assert(pack.y_pos == (float) -1.3484107E38F);
                Debug.Assert(pack.x_vel == (float)9.7805715E36F);
                Debug.Assert(pack.x_pos == (float) -2.8683037E38F);
                Debug.Assert(pack.airspeed == (float) -6.200536E37F);
                Debug.Assert(pack.roll_rate == (float) -7.678177E37F);
                Debug.Assert(pack.time_usec == (ulong)7132369715651356963L);
                Debug.Assert(pack.z_vel == (float) -4.9388565E37F);
                Debug.Assert(pack.pitch_rate == (float) -2.6630857E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.605362E38F, -3.2624517E38F, -9.901005E37F, 2.7002012E37F}));
                Debug.Assert(pack.yaw_rate == (float)1.5800656E37F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.5375827E38F, 2.4990318E38F, 2.0942152E38F}));
                Debug.Assert(pack.x_acc == (float)2.750832E38F);
                Debug.Assert(pack.z_pos == (float) -1.0639915E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_vel = (float) -1.7067254E38F;
            p146.pos_variance_SET(new float[] {-1.5375827E38F, 2.4990318E38F, 2.0942152E38F}, 0) ;
            p146.airspeed = (float) -6.200536E37F;
            p146.z_vel = (float) -4.9388565E37F;
            p146.y_acc = (float)2.9287588E35F;
            p146.roll_rate = (float) -7.678177E37F;
            p146.yaw_rate = (float)1.5800656E37F;
            p146.y_pos = (float) -1.3484107E38F;
            p146.z_acc = (float)2.4003926E38F;
            p146.vel_variance_SET(new float[] {-2.9843915E38F, -2.5109481E38F, 1.910792E38F}, 0) ;
            p146.x_acc = (float)2.750832E38F;
            p146.time_usec = (ulong)7132369715651356963L;
            p146.pitch_rate = (float) -2.6630857E38F;
            p146.q_SET(new float[] {-1.605362E38F, -3.2624517E38F, -9.901005E37F, 2.7002012E37F}, 0) ;
            p146.x_vel = (float)9.7805715E36F;
            p146.z_pos = (float) -1.0639915E38F;
            p146.x_pos = (float) -2.8683037E38F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -4503);
                Debug.Assert(pack.current_consumed == (int)1385741754);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)21198, (ushort)57905, (ushort)53047, (ushort)46192, (ushort)42335, (ushort)28715, (ushort)30616, (ushort)11853, (ushort)19751, (ushort)36118}));
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
                Debug.Assert(pack.id == (byte)(byte)67);
                Debug.Assert(pack.energy_consumed == (int)535608964);
                Debug.Assert(pack.temperature == (short)(short)29320);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 75);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.energy_consumed = (int)535608964;
            p147.id = (byte)(byte)67;
            p147.current_battery = (short)(short) -4503;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.battery_remaining = (sbyte)(sbyte) - 75;
            p147.current_consumed = (int)1385741754;
            p147.temperature = (short)(short)29320;
            p147.voltages_SET(new ushort[] {(ushort)21198, (ushort)57905, (ushort)53047, (ushort)46192, (ushort)42335, (ushort)28715, (ushort)30616, (ushort)11853, (ushort)19751, (ushort)36118}, 0) ;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.middleware_sw_version == (uint)416472647U);
                Debug.Assert(pack.os_sw_version == (uint)2358607967U);
                Debug.Assert(pack.board_version == (uint)1632317494U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)17227);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)213, (byte)79, (byte)156, (byte)71, (byte)27, (byte)200, (byte)145, (byte)89}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)140, (byte)145, (byte)162, (byte)107, (byte)8, (byte)146, (byte)64, (byte)135}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)127, (byte)40, (byte)220, (byte)80, (byte)229, (byte)188, (byte)7, (byte)124}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)232, (byte)54, (byte)225, (byte)157, (byte)169, (byte)24, (byte)126, (byte)33, (byte)101, (byte)38, (byte)252, (byte)171, (byte)90, (byte)248, (byte)135, (byte)204, (byte)239, (byte)32}));
                Debug.Assert(pack.flight_sw_version == (uint)4000130865U);
                Debug.Assert(pack.product_id == (ushort)(ushort)49074);
                Debug.Assert(pack.uid == (ulong)6630627728088281790L);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.middleware_sw_version = (uint)416472647U;
            p148.flight_sw_version = (uint)4000130865U;
            p148.vendor_id = (ushort)(ushort)17227;
            p148.middleware_custom_version_SET(new byte[] {(byte)140, (byte)145, (byte)162, (byte)107, (byte)8, (byte)146, (byte)64, (byte)135}, 0) ;
            p148.board_version = (uint)1632317494U;
            p148.uid = (ulong)6630627728088281790L;
            p148.os_custom_version_SET(new byte[] {(byte)127, (byte)40, (byte)220, (byte)80, (byte)229, (byte)188, (byte)7, (byte)124}, 0) ;
            p148.product_id = (ushort)(ushort)49074;
            p148.flight_custom_version_SET(new byte[] {(byte)213, (byte)79, (byte)156, (byte)71, (byte)27, (byte)200, (byte)145, (byte)89}, 0) ;
            p148.os_sw_version = (uint)2358607967U;
            p148.uid2_SET(new byte[] {(byte)232, (byte)54, (byte)225, (byte)157, (byte)169, (byte)24, (byte)126, (byte)33, (byte)101, (byte)38, (byte)252, (byte)171, (byte)90, (byte)248, (byte)135, (byte)204, (byte)239, (byte)32}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size_x == (float) -1.8410416E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.8129658E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -7.4209716E37F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)227);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-1.9588615E38F, -2.2091913E38F, 1.8592318E36F, 1.1294633E38F}));
                Debug.Assert(pack.time_usec == (ulong)8282338354948128186L);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.angle_y == (float)2.8028936E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.angle_x == (float)1.4349711E38F);
                Debug.Assert(pack.size_y == (float)3.0449085E38F);
                Debug.Assert(pack.target_num == (byte)(byte)226);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.5121887E38F);
                Debug.Assert(pack.distance == (float) -3.0520305E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.x_SET((float) -2.8129658E38F, PH) ;
            p149.target_num = (byte)(byte)226;
            p149.size_y = (float)3.0449085E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p149.z_SET((float) -7.4209716E37F, PH) ;
            p149.y_SET((float) -1.5121887E38F, PH) ;
            p149.angle_x = (float)1.4349711E38F;
            p149.position_valid_SET((byte)(byte)227, PH) ;
            p149.size_x = (float) -1.8410416E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.angle_y = (float)2.8028936E37F;
            p149.q_SET(new float[] {-1.9588615E38F, -2.2091913E38F, 1.8592318E36F, 1.1294633E38F}, 0, PH) ;
            p149.distance = (float) -3.0520305E38F;
            p149.time_usec = (ulong)8282338354948128186L;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_ratio == (float)2.8796608E38F);
                Debug.Assert(pack.tas_ratio == (float) -2.6774208E37F);
                Debug.Assert(pack.mag_ratio == (float) -2.9160769E38F);
                Debug.Assert(pack.vel_ratio == (float) -3.3500513E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.0253284E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
                Debug.Assert(pack.hagl_ratio == (float) -2.2567333E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -1.8879518E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)7.723471E37F);
                Debug.Assert(pack.time_usec == (ulong)2629884122352480141L);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
            p230.vel_ratio = (float) -3.3500513E38F;
            p230.pos_vert_accuracy = (float)7.723471E37F;
            p230.pos_horiz_accuracy = (float) -1.8879518E38F;
            p230.hagl_ratio = (float) -2.2567333E38F;
            p230.mag_ratio = (float) -2.9160769E38F;
            p230.tas_ratio = (float) -2.6774208E37F;
            p230.pos_vert_ratio = (float)2.8796608E38F;
            p230.pos_horiz_ratio = (float) -2.0253284E38F;
            p230.time_usec = (ulong)2629884122352480141L;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5402156776228550752L);
                Debug.Assert(pack.vert_accuracy == (float)8.526513E37F);
                Debug.Assert(pack.wind_alt == (float)3.1484488E38F);
                Debug.Assert(pack.var_vert == (float) -2.4426676E38F);
                Debug.Assert(pack.wind_z == (float) -2.41185E38F);
                Debug.Assert(pack.wind_y == (float) -4.2665316E37F);
                Debug.Assert(pack.wind_x == (float) -2.4827418E38F);
                Debug.Assert(pack.var_horiz == (float) -2.5884987E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.6436614E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -2.4827418E38F;
            p231.vert_accuracy = (float)8.526513E37F;
            p231.time_usec = (ulong)5402156776228550752L;
            p231.wind_z = (float) -2.41185E38F;
            p231.var_horiz = (float) -2.5884987E37F;
            p231.horiz_accuracy = (float) -2.6436614E38F;
            p231.wind_alt = (float)3.1484488E38F;
            p231.wind_y = (float) -4.2665316E37F;
            p231.var_vert = (float) -2.4426676E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float) -2.764005E38F);
                Debug.Assert(pack.ve == (float)1.0584556E37F);
                Debug.Assert(pack.lon == (int)1690889304);
                Debug.Assert(pack.time_week_ms == (uint)3712124176U);
                Debug.Assert(pack.gps_id == (byte)(byte)250);
                Debug.Assert(pack.alt == (float)2.972057E38F);
                Debug.Assert(pack.time_usec == (ulong)7709849658780325360L);
                Debug.Assert(pack.lat == (int) -1165877498);
                Debug.Assert(pack.vert_accuracy == (float) -1.1516836E37F);
                Debug.Assert(pack.vn == (float)2.1791624E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)192);
                Debug.Assert(pack.vdop == (float) -8.56604E37F);
                Debug.Assert(pack.hdop == (float) -2.1677523E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)47683);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
                Debug.Assert(pack.vd == (float) -2.733233E38F);
                Debug.Assert(pack.speed_accuracy == (float)2.2912241E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)24);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lat = (int) -1165877498;
            p232.satellites_visible = (byte)(byte)24;
            p232.vd = (float) -2.733233E38F;
            p232.alt = (float)2.972057E38F;
            p232.lon = (int)1690889304;
            p232.time_week = (ushort)(ushort)47683;
            p232.vert_accuracy = (float) -1.1516836E37F;
            p232.speed_accuracy = (float)2.2912241E38F;
            p232.vdop = (float) -8.56604E37F;
            p232.time_usec = (ulong)7709849658780325360L;
            p232.hdop = (float) -2.1677523E38F;
            p232.vn = (float)2.1791624E38F;
            p232.ve = (float)1.0584556E37F;
            p232.fix_type = (byte)(byte)192;
            p232.horiz_accuracy = (float) -2.764005E38F;
            p232.gps_id = (byte)(byte)250;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
            p232.time_week_ms = (uint)3712124176U;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)131);
                Debug.Assert(pack.flags == (byte)(byte)141);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)13, (byte)135, (byte)118, (byte)69, (byte)59, (byte)171, (byte)112, (byte)152, (byte)204, (byte)206, (byte)121, (byte)64, (byte)23, (byte)9, (byte)163, (byte)247, (byte)38, (byte)209, (byte)16, (byte)73, (byte)109, (byte)38, (byte)41, (byte)242, (byte)193, (byte)179, (byte)169, (byte)239, (byte)21, (byte)133, (byte)47, (byte)163, (byte)186, (byte)146, (byte)47, (byte)180, (byte)226, (byte)249, (byte)123, (byte)218, (byte)209, (byte)84, (byte)197, (byte)140, (byte)252, (byte)214, (byte)12, (byte)155, (byte)70, (byte)182, (byte)46, (byte)104, (byte)7, (byte)48, (byte)154, (byte)24, (byte)134, (byte)246, (byte)45, (byte)153, (byte)77, (byte)39, (byte)36, (byte)221, (byte)162, (byte)39, (byte)25, (byte)104, (byte)55, (byte)19, (byte)110, (byte)114, (byte)47, (byte)25, (byte)149, (byte)159, (byte)58, (byte)27, (byte)85, (byte)13, (byte)16, (byte)5, (byte)226, (byte)150, (byte)103, (byte)221, (byte)232, (byte)191, (byte)226, (byte)227, (byte)43, (byte)42, (byte)246, (byte)248, (byte)224, (byte)130, (byte)185, (byte)254, (byte)173, (byte)170, (byte)83, (byte)104, (byte)92, (byte)208, (byte)1, (byte)255, (byte)71, (byte)17, (byte)136, (byte)209, (byte)77, (byte)206, (byte)85, (byte)194, (byte)155, (byte)224, (byte)33, (byte)236, (byte)220, (byte)9, (byte)86, (byte)42, (byte)31, (byte)227, (byte)115, (byte)170, (byte)55, (byte)67, (byte)228, (byte)87, (byte)110, (byte)98, (byte)173, (byte)228, (byte)240, (byte)237, (byte)159, (byte)141, (byte)95, (byte)93, (byte)244, (byte)170, (byte)198, (byte)167, (byte)3, (byte)63, (byte)8, (byte)201, (byte)228, (byte)27, (byte)133, (byte)206, (byte)133, (byte)109, (byte)109, (byte)193, (byte)82, (byte)223, (byte)18, (byte)239, (byte)132, (byte)40, (byte)183, (byte)91, (byte)85, (byte)132, (byte)49, (byte)198, (byte)108, (byte)52, (byte)174, (byte)11, (byte)99, (byte)201, (byte)88, (byte)99, (byte)33, (byte)186, (byte)126, (byte)210}));
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)13, (byte)135, (byte)118, (byte)69, (byte)59, (byte)171, (byte)112, (byte)152, (byte)204, (byte)206, (byte)121, (byte)64, (byte)23, (byte)9, (byte)163, (byte)247, (byte)38, (byte)209, (byte)16, (byte)73, (byte)109, (byte)38, (byte)41, (byte)242, (byte)193, (byte)179, (byte)169, (byte)239, (byte)21, (byte)133, (byte)47, (byte)163, (byte)186, (byte)146, (byte)47, (byte)180, (byte)226, (byte)249, (byte)123, (byte)218, (byte)209, (byte)84, (byte)197, (byte)140, (byte)252, (byte)214, (byte)12, (byte)155, (byte)70, (byte)182, (byte)46, (byte)104, (byte)7, (byte)48, (byte)154, (byte)24, (byte)134, (byte)246, (byte)45, (byte)153, (byte)77, (byte)39, (byte)36, (byte)221, (byte)162, (byte)39, (byte)25, (byte)104, (byte)55, (byte)19, (byte)110, (byte)114, (byte)47, (byte)25, (byte)149, (byte)159, (byte)58, (byte)27, (byte)85, (byte)13, (byte)16, (byte)5, (byte)226, (byte)150, (byte)103, (byte)221, (byte)232, (byte)191, (byte)226, (byte)227, (byte)43, (byte)42, (byte)246, (byte)248, (byte)224, (byte)130, (byte)185, (byte)254, (byte)173, (byte)170, (byte)83, (byte)104, (byte)92, (byte)208, (byte)1, (byte)255, (byte)71, (byte)17, (byte)136, (byte)209, (byte)77, (byte)206, (byte)85, (byte)194, (byte)155, (byte)224, (byte)33, (byte)236, (byte)220, (byte)9, (byte)86, (byte)42, (byte)31, (byte)227, (byte)115, (byte)170, (byte)55, (byte)67, (byte)228, (byte)87, (byte)110, (byte)98, (byte)173, (byte)228, (byte)240, (byte)237, (byte)159, (byte)141, (byte)95, (byte)93, (byte)244, (byte)170, (byte)198, (byte)167, (byte)3, (byte)63, (byte)8, (byte)201, (byte)228, (byte)27, (byte)133, (byte)206, (byte)133, (byte)109, (byte)109, (byte)193, (byte)82, (byte)223, (byte)18, (byte)239, (byte)132, (byte)40, (byte)183, (byte)91, (byte)85, (byte)132, (byte)49, (byte)198, (byte)108, (byte)52, (byte)174, (byte)11, (byte)99, (byte)201, (byte)88, (byte)99, (byte)33, (byte)186, (byte)126, (byte)210}, 0) ;
            p233.flags = (byte)(byte)141;
            p233.len = (byte)(byte)131;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (byte)(byte)161);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 12);
                Debug.Assert(pack.heading_sp == (short)(short) -6016);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
                Debug.Assert(pack.failsafe == (byte)(byte)157);
                Debug.Assert(pack.custom_mode == (uint)2061262562U);
                Debug.Assert(pack.airspeed == (byte)(byte)18);
                Debug.Assert(pack.heading == (ushort)(ushort)37097);
                Debug.Assert(pack.wp_num == (byte)(byte)67);
                Debug.Assert(pack.gps_nsat == (byte)(byte)250);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)90);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.altitude_amsl == (short)(short) -2733);
                Debug.Assert(pack.groundspeed == (byte)(byte)40);
                Debug.Assert(pack.roll == (short)(short)17478);
                Debug.Assert(pack.altitude_sp == (short)(short)30028);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)148);
                Debug.Assert(pack.pitch == (short)(short)17628);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)97);
                Debug.Assert(pack.latitude == (int)114928516);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)26);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)39566);
                Debug.Assert(pack.longitude == (int)1445233078);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.latitude = (int)114928516;
            p234.temperature = (sbyte)(sbyte)90;
            p234.airspeed = (byte)(byte)18;
            p234.altitude_sp = (short)(short)30028;
            p234.throttle = (sbyte)(sbyte) - 12;
            p234.roll = (short)(short)17478;
            p234.failsafe = (byte)(byte)157;
            p234.pitch = (short)(short)17628;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
            p234.wp_distance = (ushort)(ushort)39566;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.battery_remaining = (byte)(byte)161;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p234.temperature_air = (sbyte)(sbyte)97;
            p234.climb_rate = (sbyte)(sbyte)26;
            p234.gps_nsat = (byte)(byte)250;
            p234.wp_num = (byte)(byte)67;
            p234.heading = (ushort)(ushort)37097;
            p234.longitude = (int)1445233078;
            p234.custom_mode = (uint)2061262562U;
            p234.heading_sp = (short)(short) -6016;
            p234.groundspeed = (byte)(byte)40;
            p234.airspeed_sp = (byte)(byte)148;
            p234.altitude_amsl = (short)(short) -2733;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)5.787638E36F);
                Debug.Assert(pack.vibration_z == (float) -1.3190762E38F);
                Debug.Assert(pack.vibration_y == (float) -3.9932165E37F);
                Debug.Assert(pack.time_usec == (ulong)4698338706046674830L);
                Debug.Assert(pack.clipping_2 == (uint)3848071965U);
                Debug.Assert(pack.clipping_0 == (uint)2504382488U);
                Debug.Assert(pack.clipping_1 == (uint)4219278379U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_x = (float)5.787638E36F;
            p241.clipping_2 = (uint)3848071965U;
            p241.clipping_0 = (uint)2504382488U;
            p241.clipping_1 = (uint)4219278379U;
            p241.vibration_y = (float) -3.9932165E37F;
            p241.vibration_z = (float) -1.3190762E38F;
            p241.time_usec = (ulong)4698338706046674830L;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)1663301208);
                Debug.Assert(pack.approach_x == (float)2.004777E38F);
                Debug.Assert(pack.longitude == (int)853944736);
                Debug.Assert(pack.approach_y == (float) -5.826695E37F);
                Debug.Assert(pack.z == (float)2.8043528E38F);
                Debug.Assert(pack.y == (float)1.18597E38F);
                Debug.Assert(pack.approach_z == (float) -1.9810432E38F);
                Debug.Assert(pack.altitude == (int) -1502537328);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4472912E38F, 1.95896E38F, -9.167738E37F, -3.280693E38F}));
                Debug.Assert(pack.x == (float)8.470261E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)84739329909878850L);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.z = (float)2.8043528E38F;
            p242.time_usec_SET((ulong)84739329909878850L, PH) ;
            p242.approach_x = (float)2.004777E38F;
            p242.latitude = (int)1663301208;
            p242.altitude = (int) -1502537328;
            p242.longitude = (int)853944736;
            p242.approach_y = (float) -5.826695E37F;
            p242.y = (float)1.18597E38F;
            p242.approach_z = (float) -1.9810432E38F;
            p242.q_SET(new float[] {-1.4472912E38F, 1.95896E38F, -9.167738E37F, -3.280693E38F}, 0) ;
            p242.x = (float)8.470261E37F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1158358E38F, 2.2275454E38F, 3.3074961E38F, -1.5829019E37F}));
                Debug.Assert(pack.altitude == (int)58256780);
                Debug.Assert(pack.approach_z == (float) -2.0288724E38F);
                Debug.Assert(pack.latitude == (int) -488741674);
                Debug.Assert(pack.target_system == (byte)(byte)223);
                Debug.Assert(pack.y == (float) -3.3556775E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)533308742722564799L);
                Debug.Assert(pack.approach_y == (float)4.485778E37F);
                Debug.Assert(pack.z == (float)3.3401458E38F);
                Debug.Assert(pack.x == (float)9.332915E37F);
                Debug.Assert(pack.longitude == (int)834363411);
                Debug.Assert(pack.approach_x == (float)9.158812E37F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.longitude = (int)834363411;
            p243.approach_y = (float)4.485778E37F;
            p243.y = (float) -3.3556775E37F;
            p243.time_usec_SET((ulong)533308742722564799L, PH) ;
            p243.altitude = (int)58256780;
            p243.latitude = (int) -488741674;
            p243.q_SET(new float[] {-2.1158358E38F, 2.2275454E38F, 3.3074961E38F, -1.5829019E37F}, 0) ;
            p243.target_system = (byte)(byte)223;
            p243.z = (float)3.3401458E38F;
            p243.approach_z = (float) -2.0288724E38F;
            p243.x = (float)9.332915E37F;
            p243.approach_x = (float)9.158812E37F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)10904);
                Debug.Assert(pack.interval_us == (int)1252249922);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1252249922;
            p244.message_id = (ushort)(ushort)10904;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.lon == (int) -58045485);
                Debug.Assert(pack.tslc == (byte)(byte)24);
                Debug.Assert(pack.lat == (int) -994174738);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
                Debug.Assert(pack.ICAO_address == (uint)2914867779U);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)61200);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.altitude == (int) -905218631);
                Debug.Assert(pack.heading == (ushort)(ushort)47865);
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("urvt"));
                Debug.Assert(pack.ver_velocity == (short)(short)24983);
                Debug.Assert(pack.squawk == (ushort)(ushort)47258);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.heading = (ushort)(ushort)47865;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.ver_velocity = (short)(short)24983;
            p246.squawk = (ushort)(ushort)47258;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.hor_velocity = (ushort)(ushort)61200;
            p246.ICAO_address = (uint)2914867779U;
            p246.callsign_SET("urvt", PH) ;
            p246.tslc = (byte)(byte)24;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.lon = (int) -58045485;
            p246.lat = (int) -994174738;
            p246.altitude = (int) -905218631;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)1292557362U);
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.8751662E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -3.071153E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.5379785E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.time_to_minimum_delta = (float) -3.071153E38F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.altitude_minimum_delta = (float) -2.5379785E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.horizontal_minimum_delta = (float)1.8751662E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.id = (uint)1292557362U;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)24783);
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.target_network == (byte)(byte)149);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)60, (byte)238, (byte)120, (byte)97, (byte)247, (byte)169, (byte)239, (byte)226, (byte)45, (byte)93, (byte)227, (byte)224, (byte)76, (byte)191, (byte)230, (byte)123, (byte)170, (byte)149, (byte)219, (byte)86, (byte)143, (byte)111, (byte)221, (byte)150, (byte)25, (byte)152, (byte)67, (byte)90, (byte)71, (byte)220, (byte)246, (byte)55, (byte)47, (byte)203, (byte)64, (byte)54, (byte)45, (byte)127, (byte)142, (byte)35, (byte)2, (byte)239, (byte)39, (byte)225, (byte)229, (byte)214, (byte)255, (byte)56, (byte)32, (byte)15, (byte)171, (byte)38, (byte)230, (byte)229, (byte)216, (byte)55, (byte)55, (byte)201, (byte)222, (byte)232, (byte)199, (byte)29, (byte)111, (byte)122, (byte)90, (byte)21, (byte)246, (byte)164, (byte)47, (byte)150, (byte)46, (byte)99, (byte)243, (byte)211, (byte)89, (byte)106, (byte)69, (byte)92, (byte)209, (byte)116, (byte)120, (byte)245, (byte)150, (byte)12, (byte)138, (byte)247, (byte)118, (byte)2, (byte)96, (byte)218, (byte)204, (byte)41, (byte)109, (byte)209, (byte)193, (byte)245, (byte)135, (byte)23, (byte)166, (byte)90, (byte)98, (byte)120, (byte)230, (byte)45, (byte)16, (byte)232, (byte)234, (byte)0, (byte)249, (byte)24, (byte)127, (byte)220, (byte)184, (byte)192, (byte)36, (byte)153, (byte)20, (byte)122, (byte)153, (byte)110, (byte)94, (byte)218, (byte)62, (byte)68, (byte)43, (byte)81, (byte)154, (byte)159, (byte)206, (byte)152, (byte)209, (byte)132, (byte)53, (byte)219, (byte)123, (byte)149, (byte)67, (byte)155, (byte)85, (byte)0, (byte)3, (byte)19, (byte)142, (byte)6, (byte)50, (byte)166, (byte)125, (byte)13, (byte)163, (byte)238, (byte)117, (byte)116, (byte)253, (byte)218, (byte)175, (byte)202, (byte)183, (byte)218, (byte)42, (byte)17, (byte)130, (byte)44, (byte)189, (byte)235, (byte)101, (byte)212, (byte)61, (byte)142, (byte)61, (byte)205, (byte)70, (byte)22, (byte)209, (byte)204, (byte)56, (byte)82, (byte)216, (byte)62, (byte)191, (byte)153, (byte)229, (byte)118, (byte)202, (byte)245, (byte)122, (byte)184, (byte)240, (byte)254, (byte)62, (byte)18, (byte)166, (byte)66, (byte)255, (byte)100, (byte)229, (byte)30, (byte)43, (byte)124, (byte)145, (byte)66, (byte)93, (byte)38, (byte)78, (byte)143, (byte)165, (byte)115, (byte)161, (byte)241, (byte)162, (byte)108, (byte)118, (byte)179, (byte)146, (byte)96, (byte)53, (byte)186, (byte)160, (byte)67, (byte)176, (byte)236, (byte)227, (byte)183, (byte)98, (byte)250, (byte)137, (byte)176, (byte)79, (byte)110, (byte)75, (byte)198, (byte)36, (byte)44, (byte)233, (byte)208, (byte)48, (byte)187, (byte)239, (byte)151, (byte)111, (byte)174, (byte)210, (byte)67, (byte)229, (byte)33, (byte)6, (byte)213, (byte)57, (byte)1, (byte)209}));
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)24783;
            p248.target_component = (byte)(byte)97;
            p248.payload_SET(new byte[] {(byte)60, (byte)238, (byte)120, (byte)97, (byte)247, (byte)169, (byte)239, (byte)226, (byte)45, (byte)93, (byte)227, (byte)224, (byte)76, (byte)191, (byte)230, (byte)123, (byte)170, (byte)149, (byte)219, (byte)86, (byte)143, (byte)111, (byte)221, (byte)150, (byte)25, (byte)152, (byte)67, (byte)90, (byte)71, (byte)220, (byte)246, (byte)55, (byte)47, (byte)203, (byte)64, (byte)54, (byte)45, (byte)127, (byte)142, (byte)35, (byte)2, (byte)239, (byte)39, (byte)225, (byte)229, (byte)214, (byte)255, (byte)56, (byte)32, (byte)15, (byte)171, (byte)38, (byte)230, (byte)229, (byte)216, (byte)55, (byte)55, (byte)201, (byte)222, (byte)232, (byte)199, (byte)29, (byte)111, (byte)122, (byte)90, (byte)21, (byte)246, (byte)164, (byte)47, (byte)150, (byte)46, (byte)99, (byte)243, (byte)211, (byte)89, (byte)106, (byte)69, (byte)92, (byte)209, (byte)116, (byte)120, (byte)245, (byte)150, (byte)12, (byte)138, (byte)247, (byte)118, (byte)2, (byte)96, (byte)218, (byte)204, (byte)41, (byte)109, (byte)209, (byte)193, (byte)245, (byte)135, (byte)23, (byte)166, (byte)90, (byte)98, (byte)120, (byte)230, (byte)45, (byte)16, (byte)232, (byte)234, (byte)0, (byte)249, (byte)24, (byte)127, (byte)220, (byte)184, (byte)192, (byte)36, (byte)153, (byte)20, (byte)122, (byte)153, (byte)110, (byte)94, (byte)218, (byte)62, (byte)68, (byte)43, (byte)81, (byte)154, (byte)159, (byte)206, (byte)152, (byte)209, (byte)132, (byte)53, (byte)219, (byte)123, (byte)149, (byte)67, (byte)155, (byte)85, (byte)0, (byte)3, (byte)19, (byte)142, (byte)6, (byte)50, (byte)166, (byte)125, (byte)13, (byte)163, (byte)238, (byte)117, (byte)116, (byte)253, (byte)218, (byte)175, (byte)202, (byte)183, (byte)218, (byte)42, (byte)17, (byte)130, (byte)44, (byte)189, (byte)235, (byte)101, (byte)212, (byte)61, (byte)142, (byte)61, (byte)205, (byte)70, (byte)22, (byte)209, (byte)204, (byte)56, (byte)82, (byte)216, (byte)62, (byte)191, (byte)153, (byte)229, (byte)118, (byte)202, (byte)245, (byte)122, (byte)184, (byte)240, (byte)254, (byte)62, (byte)18, (byte)166, (byte)66, (byte)255, (byte)100, (byte)229, (byte)30, (byte)43, (byte)124, (byte)145, (byte)66, (byte)93, (byte)38, (byte)78, (byte)143, (byte)165, (byte)115, (byte)161, (byte)241, (byte)162, (byte)108, (byte)118, (byte)179, (byte)146, (byte)96, (byte)53, (byte)186, (byte)160, (byte)67, (byte)176, (byte)236, (byte)227, (byte)183, (byte)98, (byte)250, (byte)137, (byte)176, (byte)79, (byte)110, (byte)75, (byte)198, (byte)36, (byte)44, (byte)233, (byte)208, (byte)48, (byte)187, (byte)239, (byte)151, (byte)111, (byte)174, (byte)210, (byte)67, (byte)229, (byte)33, (byte)6, (byte)213, (byte)57, (byte)1, (byte)209}, 0) ;
            p248.target_network = (byte)(byte)149;
            p248.target_system = (byte)(byte)172;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)13315);
                Debug.Assert(pack.type == (byte)(byte)199);
                Debug.Assert(pack.ver == (byte)(byte)58);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 109, (sbyte)111, (sbyte)52, (sbyte)102, (sbyte)51, (sbyte)59, (sbyte)99, (sbyte) - 51, (sbyte) - 86, (sbyte) - 16, (sbyte) - 56, (sbyte)120, (sbyte) - 60, (sbyte) - 3, (sbyte) - 68, (sbyte) - 5, (sbyte)5, (sbyte) - 20, (sbyte) - 32, (sbyte)71, (sbyte) - 122, (sbyte)57, (sbyte)82, (sbyte)11, (sbyte)93, (sbyte) - 104, (sbyte)13, (sbyte)18, (sbyte) - 100, (sbyte) - 81, (sbyte)28, (sbyte) - 73}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)13315;
            p249.value_SET(new sbyte[] {(sbyte) - 109, (sbyte)111, (sbyte)52, (sbyte)102, (sbyte)51, (sbyte)59, (sbyte)99, (sbyte) - 51, (sbyte) - 86, (sbyte) - 16, (sbyte) - 56, (sbyte)120, (sbyte) - 60, (sbyte) - 3, (sbyte) - 68, (sbyte) - 5, (sbyte)5, (sbyte) - 20, (sbyte) - 32, (sbyte)71, (sbyte) - 122, (sbyte)57, (sbyte)82, (sbyte)11, (sbyte)93, (sbyte) - 104, (sbyte)13, (sbyte)18, (sbyte) - 100, (sbyte) - 81, (sbyte)28, (sbyte) - 73}, 0) ;
            p249.type = (byte)(byte)199;
            p249.ver = (byte)(byte)58;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.3441494E38F);
                Debug.Assert(pack.time_usec == (ulong)4879459199791636949L);
                Debug.Assert(pack.z == (float) -1.6878306E38F);
                Debug.Assert(pack.y == (float) -7.5100586E37F);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("wzhhlwvr"));
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)2.3441494E38F;
            p250.time_usec = (ulong)4879459199791636949L;
            p250.y = (float) -7.5100586E37F;
            p250.name_SET("wzhhlwvr", PH) ;
            p250.z = (float) -1.6878306E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1531176907U);
                Debug.Assert(pack.value == (float) -1.4118966E38F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("j"));
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1531176907U;
            p251.value = (float) -1.4118966E38F;
            p251.name_SET("j", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("e"));
                Debug.Assert(pack.time_boot_ms == (uint)783299080U);
                Debug.Assert(pack.value == (int)1071061688);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)1071061688;
            p252.name_SET("e", PH) ;
            p252.time_boot_ms = (uint)783299080U;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO);
                Debug.Assert(pack.text_LEN(ph) == 43);
                Debug.Assert(pack.text_TRY(ph).Equals("qljfaKexccmtjqkriyyuufqnbachpnzqBpwwxzrvuth"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("qljfaKexccmtjqkriyyuufqnbachpnzqBpwwxzrvuth", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3932752577U);
                Debug.Assert(pack.ind == (byte)(byte)76);
                Debug.Assert(pack.value == (float)2.9763783E38F);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)76;
            p254.value = (float)2.9763783E38F;
            p254.time_boot_ms = (uint)3932752577U;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)42);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)50, (byte)92, (byte)78, (byte)251, (byte)136, (byte)168, (byte)19, (byte)6, (byte)124, (byte)194, (byte)68, (byte)71, (byte)207, (byte)167, (byte)206, (byte)212, (byte)153, (byte)183, (byte)227, (byte)12, (byte)37, (byte)144, (byte)75, (byte)166, (byte)173, (byte)230, (byte)134, (byte)104, (byte)173, (byte)147, (byte)151, (byte)143}));
                Debug.Assert(pack.initial_timestamp == (ulong)1176887087658019433L);
                Debug.Assert(pack.target_system == (byte)(byte)83);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)83;
            p256.secret_key_SET(new byte[] {(byte)50, (byte)92, (byte)78, (byte)251, (byte)136, (byte)168, (byte)19, (byte)6, (byte)124, (byte)194, (byte)68, (byte)71, (byte)207, (byte)167, (byte)206, (byte)212, (byte)153, (byte)183, (byte)227, (byte)12, (byte)37, (byte)144, (byte)75, (byte)166, (byte)173, (byte)230, (byte)134, (byte)104, (byte)173, (byte)147, (byte)151, (byte)143}, 0) ;
            p256.target_component = (byte)(byte)42;
            p256.initial_timestamp = (ulong)1176887087658019433L;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3548448191U);
                Debug.Assert(pack.state == (byte)(byte)70);
                Debug.Assert(pack.last_change_ms == (uint)2939509999U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)70;
            p257.last_change_ms = (uint)2939509999U;
            p257.time_boot_ms = (uint)3548448191U;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.tune_LEN(ph) == 21);
                Debug.Assert(pack.tune_TRY(ph).Equals("nyPqrfjdprzahOtfvEazg"));
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)47;
            p258.target_system = (byte)(byte)155;
            p258.tune_SET("nyPqrfjdprzahOtfvEazg", PH) ;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_h == (float) -1.6674243E38F);
                Debug.Assert(pack.sensor_size_v == (float)2.3120355E38F);
                Debug.Assert(pack.focal_length == (float)1.5238451E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)85, (byte)37, (byte)20, (byte)134, (byte)54, (byte)25, (byte)86, (byte)242, (byte)122, (byte)204, (byte)196, (byte)216, (byte)61, (byte)10, (byte)185, (byte)170, (byte)186, (byte)225, (byte)213, (byte)254, (byte)226, (byte)220, (byte)90, (byte)29, (byte)154, (byte)82, (byte)237, (byte)128, (byte)81, (byte)207, (byte)84, (byte)253}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 49);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("gcFZgvYehzevvweJayfcmszvqtycnszZcKdOmeftbOudybkZa"));
                Debug.Assert(pack.lens_id == (byte)(byte)238);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)60499);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
                Debug.Assert(pack.time_boot_ms == (uint)3413212253U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)44300);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)1044);
                Debug.Assert(pack.firmware_version == (uint)1106154066U);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)174, (byte)86, (byte)114, (byte)6, (byte)93, (byte)110, (byte)89, (byte)67, (byte)251, (byte)2, (byte)13, (byte)83, (byte)245, (byte)249, (byte)225, (byte)253, (byte)138, (byte)209, (byte)188, (byte)126, (byte)95, (byte)187, (byte)24, (byte)82, (byte)243, (byte)151, (byte)89, (byte)73, (byte)94, (byte)214, (byte)1, (byte)81}));
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.sensor_size_v = (float)2.3120355E38F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.vendor_name_SET(new byte[] {(byte)174, (byte)86, (byte)114, (byte)6, (byte)93, (byte)110, (byte)89, (byte)67, (byte)251, (byte)2, (byte)13, (byte)83, (byte)245, (byte)249, (byte)225, (byte)253, (byte)138, (byte)209, (byte)188, (byte)126, (byte)95, (byte)187, (byte)24, (byte)82, (byte)243, (byte)151, (byte)89, (byte)73, (byte)94, (byte)214, (byte)1, (byte)81}, 0) ;
            p259.resolution_h = (ushort)(ushort)60499;
            p259.firmware_version = (uint)1106154066U;
            p259.cam_definition_version = (ushort)(ushort)1044;
            p259.cam_definition_uri_SET("gcFZgvYehzevvweJayfcmszvqtycnszZcKdOmeftbOudybkZa", PH) ;
            p259.sensor_size_h = (float) -1.6674243E38F;
            p259.focal_length = (float)1.5238451E38F;
            p259.time_boot_ms = (uint)3413212253U;
            p259.resolution_v = (ushort)(ushort)44300;
            p259.lens_id = (byte)(byte)238;
            p259.model_name_SET(new byte[] {(byte)85, (byte)37, (byte)20, (byte)134, (byte)54, (byte)25, (byte)86, (byte)242, (byte)122, (byte)204, (byte)196, (byte)216, (byte)61, (byte)10, (byte)185, (byte)170, (byte)186, (byte)225, (byte)213, (byte)254, (byte)226, (byte)220, (byte)90, (byte)29, (byte)154, (byte)82, (byte)237, (byte)128, (byte)81, (byte)207, (byte)84, (byte)253}, 0) ;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)667714274U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)667714274U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)166);
                Debug.Assert(pack.write_speed == (float) -2.6784839E38F);
                Debug.Assert(pack.read_speed == (float)2.3792398E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1432168389U);
                Debug.Assert(pack.storage_id == (byte)(byte)49);
                Debug.Assert(pack.status == (byte)(byte)129);
                Debug.Assert(pack.used_capacity == (float) -1.8623124E38F);
                Debug.Assert(pack.available_capacity == (float)2.988828E38F);
                Debug.Assert(pack.total_capacity == (float)2.6989164E38F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float)2.3792398E38F;
            p261.used_capacity = (float) -1.8623124E38F;
            p261.time_boot_ms = (uint)1432168389U;
            p261.storage_id = (byte)(byte)49;
            p261.storage_count = (byte)(byte)166;
            p261.available_capacity = (float)2.988828E38F;
            p261.status = (byte)(byte)129;
            p261.total_capacity = (float)2.6989164E38F;
            p261.write_speed = (float) -2.6784839E38F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4027681196U);
                Debug.Assert(pack.available_capacity == (float) -4.7313837E37F);
                Debug.Assert(pack.recording_time_ms == (uint)4241396044U);
                Debug.Assert(pack.image_interval == (float) -3.6337575E37F);
                Debug.Assert(pack.image_status == (byte)(byte)228);
                Debug.Assert(pack.video_status == (byte)(byte)104);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)104;
            p262.available_capacity = (float) -4.7313837E37F;
            p262.recording_time_ms = (uint)4241396044U;
            p262.image_status = (byte)(byte)228;
            p262.image_interval = (float) -3.6337575E37F;
            p262.time_boot_ms = (uint)4027681196U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)114);
                Debug.Assert(pack.image_index == (int) -304360116);
                Debug.Assert(pack.time_boot_ms == (uint)842168087U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3906482E37F, -1.0294687E38F, -3.2836055E38F, 2.4778062E38F}));
                Debug.Assert(pack.file_url_LEN(ph) == 102);
                Debug.Assert(pack.file_url_TRY(ph).Equals("sggjjjpozhrpudxpftzaygzyzxfdsekpMDtalGvijvkfFBupSirdeoavidngqxqyqTpYphLqyzlhtzNbdhqgooreyqbymwwfnopqqE"));
                Debug.Assert(pack.lon == (int)39024674);
                Debug.Assert(pack.alt == (int)8158048);
                Debug.Assert(pack.relative_alt == (int)138274693);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)48);
                Debug.Assert(pack.lat == (int)1864941609);
                Debug.Assert(pack.time_utc == (ulong)309722101486817075L);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.image_index = (int) -304360116;
            p263.time_utc = (ulong)309722101486817075L;
            p263.relative_alt = (int)138274693;
            p263.time_boot_ms = (uint)842168087U;
            p263.q_SET(new float[] {-3.3906482E37F, -1.0294687E38F, -3.2836055E38F, 2.4778062E38F}, 0) ;
            p263.lon = (int)39024674;
            p263.capture_result = (sbyte)(sbyte)48;
            p263.camera_id = (byte)(byte)114;
            p263.file_url_SET("sggjjjpozhrpudxpftzaygzyzxfdsekpMDtalGvijvkfFBupSirdeoavidngqxqyqTpYphLqyzlhtzNbdhqgooreyqbymwwfnopqqE", PH) ;
            p263.lat = (int)1864941609;
            p263.alt = (int)8158048;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)8205736943397244946L);
                Debug.Assert(pack.arming_time_utc == (ulong)4442697374962949089L);
                Debug.Assert(pack.flight_uuid == (ulong)2660134235467517314L);
                Debug.Assert(pack.time_boot_ms == (uint)2480578840U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)2660134235467517314L;
            p264.takeoff_time_utc = (ulong)8205736943397244946L;
            p264.time_boot_ms = (uint)2480578840U;
            p264.arming_time_utc = (ulong)4442697374962949089L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -8.285497E37F);
                Debug.Assert(pack.roll == (float)2.7690189E38F);
                Debug.Assert(pack.yaw == (float)1.1288398E38F);
                Debug.Assert(pack.time_boot_ms == (uint)309060448U);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)2.7690189E38F;
            p265.time_boot_ms = (uint)309060448U;
            p265.yaw = (float)1.1288398E38F;
            p265.pitch = (float) -8.285497E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)26367);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)222, (byte)132, (byte)141, (byte)107, (byte)38, (byte)82, (byte)150, (byte)170, (byte)41, (byte)142, (byte)251, (byte)151, (byte)251, (byte)115, (byte)179, (byte)152, (byte)59, (byte)55, (byte)113, (byte)167, (byte)6, (byte)20, (byte)151, (byte)73, (byte)89, (byte)102, (byte)164, (byte)28, (byte)246, (byte)50, (byte)149, (byte)169, (byte)135, (byte)186, (byte)173, (byte)29, (byte)88, (byte)35, (byte)22, (byte)200, (byte)119, (byte)73, (byte)227, (byte)180, (byte)188, (byte)233, (byte)211, (byte)76, (byte)139, (byte)201, (byte)173, (byte)66, (byte)255, (byte)51, (byte)111, (byte)211, (byte)47, (byte)211, (byte)51, (byte)191, (byte)226, (byte)124, (byte)112, (byte)49, (byte)154, (byte)160, (byte)86, (byte)136, (byte)25, (byte)125, (byte)68, (byte)80, (byte)25, (byte)41, (byte)105, (byte)117, (byte)105, (byte)152, (byte)186, (byte)192, (byte)200, (byte)112, (byte)51, (byte)217, (byte)7, (byte)56, (byte)237, (byte)70, (byte)185, (byte)246, (byte)238, (byte)136, (byte)65, (byte)187, (byte)239, (byte)110, (byte)73, (byte)250, (byte)236, (byte)78, (byte)12, (byte)54, (byte)76, (byte)230, (byte)201, (byte)239, (byte)223, (byte)251, (byte)37, (byte)166, (byte)183, (byte)191, (byte)222, (byte)86, (byte)250, (byte)53, (byte)81, (byte)102, (byte)85, (byte)38, (byte)81, (byte)43, (byte)152, (byte)180, (byte)99, (byte)108, (byte)168, (byte)36, (byte)238, (byte)36, (byte)254, (byte)35, (byte)119, (byte)91, (byte)210, (byte)1, (byte)118, (byte)244, (byte)26, (byte)145, (byte)2, (byte)186, (byte)237, (byte)178, (byte)40, (byte)132, (byte)170, (byte)43, (byte)193, (byte)228, (byte)108, (byte)223, (byte)45, (byte)177, (byte)83, (byte)176, (byte)31, (byte)244, (byte)53, (byte)235, (byte)100, (byte)136, (byte)253, (byte)200, (byte)37, (byte)31, (byte)61, (byte)16, (byte)244, (byte)11, (byte)88, (byte)231, (byte)93, (byte)122, (byte)226, (byte)71, (byte)240, (byte)87, (byte)218, (byte)92, (byte)191, (byte)130, (byte)21, (byte)176, (byte)112, (byte)14, (byte)155, (byte)50, (byte)70, (byte)118, (byte)214, (byte)141, (byte)82, (byte)201, (byte)26, (byte)130, (byte)157, (byte)221, (byte)211, (byte)194, (byte)188, (byte)112, (byte)11, (byte)139, (byte)231, (byte)132, (byte)153, (byte)244, (byte)137, (byte)9, (byte)30, (byte)127, (byte)140, (byte)40, (byte)148, (byte)209, (byte)38, (byte)81, (byte)225, (byte)142, (byte)113, (byte)134, (byte)216, (byte)108, (byte)83, (byte)12, (byte)135, (byte)213, (byte)181, (byte)166, (byte)196, (byte)93, (byte)36, (byte)216, (byte)13, (byte)3, (byte)176, (byte)76, (byte)89, (byte)235, (byte)1, (byte)79, (byte)208, (byte)101, (byte)96, (byte)110, (byte)85, (byte)175, (byte)1}));
                Debug.Assert(pack.length == (byte)(byte)160);
                Debug.Assert(pack.target_system == (byte)(byte)100);
                Debug.Assert(pack.first_message_offset == (byte)(byte)206);
                Debug.Assert(pack.target_component == (byte)(byte)29);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)222, (byte)132, (byte)141, (byte)107, (byte)38, (byte)82, (byte)150, (byte)170, (byte)41, (byte)142, (byte)251, (byte)151, (byte)251, (byte)115, (byte)179, (byte)152, (byte)59, (byte)55, (byte)113, (byte)167, (byte)6, (byte)20, (byte)151, (byte)73, (byte)89, (byte)102, (byte)164, (byte)28, (byte)246, (byte)50, (byte)149, (byte)169, (byte)135, (byte)186, (byte)173, (byte)29, (byte)88, (byte)35, (byte)22, (byte)200, (byte)119, (byte)73, (byte)227, (byte)180, (byte)188, (byte)233, (byte)211, (byte)76, (byte)139, (byte)201, (byte)173, (byte)66, (byte)255, (byte)51, (byte)111, (byte)211, (byte)47, (byte)211, (byte)51, (byte)191, (byte)226, (byte)124, (byte)112, (byte)49, (byte)154, (byte)160, (byte)86, (byte)136, (byte)25, (byte)125, (byte)68, (byte)80, (byte)25, (byte)41, (byte)105, (byte)117, (byte)105, (byte)152, (byte)186, (byte)192, (byte)200, (byte)112, (byte)51, (byte)217, (byte)7, (byte)56, (byte)237, (byte)70, (byte)185, (byte)246, (byte)238, (byte)136, (byte)65, (byte)187, (byte)239, (byte)110, (byte)73, (byte)250, (byte)236, (byte)78, (byte)12, (byte)54, (byte)76, (byte)230, (byte)201, (byte)239, (byte)223, (byte)251, (byte)37, (byte)166, (byte)183, (byte)191, (byte)222, (byte)86, (byte)250, (byte)53, (byte)81, (byte)102, (byte)85, (byte)38, (byte)81, (byte)43, (byte)152, (byte)180, (byte)99, (byte)108, (byte)168, (byte)36, (byte)238, (byte)36, (byte)254, (byte)35, (byte)119, (byte)91, (byte)210, (byte)1, (byte)118, (byte)244, (byte)26, (byte)145, (byte)2, (byte)186, (byte)237, (byte)178, (byte)40, (byte)132, (byte)170, (byte)43, (byte)193, (byte)228, (byte)108, (byte)223, (byte)45, (byte)177, (byte)83, (byte)176, (byte)31, (byte)244, (byte)53, (byte)235, (byte)100, (byte)136, (byte)253, (byte)200, (byte)37, (byte)31, (byte)61, (byte)16, (byte)244, (byte)11, (byte)88, (byte)231, (byte)93, (byte)122, (byte)226, (byte)71, (byte)240, (byte)87, (byte)218, (byte)92, (byte)191, (byte)130, (byte)21, (byte)176, (byte)112, (byte)14, (byte)155, (byte)50, (byte)70, (byte)118, (byte)214, (byte)141, (byte)82, (byte)201, (byte)26, (byte)130, (byte)157, (byte)221, (byte)211, (byte)194, (byte)188, (byte)112, (byte)11, (byte)139, (byte)231, (byte)132, (byte)153, (byte)244, (byte)137, (byte)9, (byte)30, (byte)127, (byte)140, (byte)40, (byte)148, (byte)209, (byte)38, (byte)81, (byte)225, (byte)142, (byte)113, (byte)134, (byte)216, (byte)108, (byte)83, (byte)12, (byte)135, (byte)213, (byte)181, (byte)166, (byte)196, (byte)93, (byte)36, (byte)216, (byte)13, (byte)3, (byte)176, (byte)76, (byte)89, (byte)235, (byte)1, (byte)79, (byte)208, (byte)101, (byte)96, (byte)110, (byte)85, (byte)175, (byte)1}, 0) ;
            p266.target_system = (byte)(byte)100;
            p266.first_message_offset = (byte)(byte)206;
            p266.length = (byte)(byte)160;
            p266.target_component = (byte)(byte)29;
            p266.sequence = (ushort)(ushort)26367;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)156);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)188, (byte)53, (byte)98, (byte)192, (byte)247, (byte)162, (byte)116, (byte)22, (byte)145, (byte)58, (byte)124, (byte)66, (byte)219, (byte)231, (byte)9, (byte)134, (byte)80, (byte)203, (byte)237, (byte)125, (byte)60, (byte)192, (byte)108, (byte)89, (byte)136, (byte)84, (byte)218, (byte)36, (byte)123, (byte)134, (byte)34, (byte)135, (byte)67, (byte)201, (byte)135, (byte)17, (byte)129, (byte)142, (byte)24, (byte)39, (byte)153, (byte)76, (byte)171, (byte)1, (byte)135, (byte)72, (byte)188, (byte)124, (byte)197, (byte)146, (byte)251, (byte)148, (byte)248, (byte)64, (byte)243, (byte)184, (byte)162, (byte)92, (byte)219, (byte)70, (byte)228, (byte)231, (byte)108, (byte)174, (byte)167, (byte)20, (byte)174, (byte)183, (byte)2, (byte)155, (byte)186, (byte)8, (byte)230, (byte)238, (byte)216, (byte)23, (byte)36, (byte)155, (byte)68, (byte)118, (byte)229, (byte)74, (byte)147, (byte)32, (byte)122, (byte)65, (byte)213, (byte)75, (byte)249, (byte)0, (byte)103, (byte)72, (byte)86, (byte)244, (byte)235, (byte)53, (byte)162, (byte)179, (byte)142, (byte)77, (byte)226, (byte)73, (byte)109, (byte)1, (byte)250, (byte)73, (byte)34, (byte)192, (byte)28, (byte)132, (byte)85, (byte)120, (byte)95, (byte)166, (byte)221, (byte)117, (byte)111, (byte)153, (byte)106, (byte)25, (byte)66, (byte)125, (byte)127, (byte)207, (byte)161, (byte)193, (byte)58, (byte)192, (byte)60, (byte)130, (byte)131, (byte)105, (byte)6, (byte)99, (byte)220, (byte)129, (byte)206, (byte)224, (byte)107, (byte)228, (byte)102, (byte)116, (byte)148, (byte)72, (byte)246, (byte)42, (byte)146, (byte)151, (byte)246, (byte)85, (byte)29, (byte)82, (byte)77, (byte)24, (byte)33, (byte)237, (byte)141, (byte)153, (byte)76, (byte)31, (byte)215, (byte)204, (byte)216, (byte)190, (byte)94, (byte)62, (byte)51, (byte)4, (byte)88, (byte)236, (byte)135, (byte)192, (byte)170, (byte)51, (byte)84, (byte)206, (byte)236, (byte)56, (byte)153, (byte)179, (byte)32, (byte)210, (byte)219, (byte)216, (byte)101, (byte)93, (byte)234, (byte)245, (byte)205, (byte)12, (byte)17, (byte)152, (byte)65, (byte)81, (byte)135, (byte)58, (byte)22, (byte)27, (byte)85, (byte)236, (byte)253, (byte)77, (byte)161, (byte)134, (byte)233, (byte)79, (byte)156, (byte)196, (byte)104, (byte)102, (byte)2, (byte)203, (byte)235, (byte)8, (byte)149, (byte)38, (byte)206, (byte)198, (byte)45, (byte)202, (byte)213, (byte)207, (byte)13, (byte)233, (byte)53, (byte)57, (byte)91, (byte)98, (byte)15, (byte)55, (byte)183, (byte)55, (byte)196, (byte)173, (byte)18, (byte)139, (byte)11, (byte)17, (byte)201, (byte)193, (byte)115, (byte)16, (byte)196, (byte)248, (byte)130, (byte)210, (byte)59, (byte)116, (byte)151}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)87);
                Debug.Assert(pack.sequence == (ushort)(ushort)49206);
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.target_component == (byte)(byte)172);
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)172;
            p267.length = (byte)(byte)156;
            p267.target_system = (byte)(byte)95;
            p267.data__SET(new byte[] {(byte)188, (byte)53, (byte)98, (byte)192, (byte)247, (byte)162, (byte)116, (byte)22, (byte)145, (byte)58, (byte)124, (byte)66, (byte)219, (byte)231, (byte)9, (byte)134, (byte)80, (byte)203, (byte)237, (byte)125, (byte)60, (byte)192, (byte)108, (byte)89, (byte)136, (byte)84, (byte)218, (byte)36, (byte)123, (byte)134, (byte)34, (byte)135, (byte)67, (byte)201, (byte)135, (byte)17, (byte)129, (byte)142, (byte)24, (byte)39, (byte)153, (byte)76, (byte)171, (byte)1, (byte)135, (byte)72, (byte)188, (byte)124, (byte)197, (byte)146, (byte)251, (byte)148, (byte)248, (byte)64, (byte)243, (byte)184, (byte)162, (byte)92, (byte)219, (byte)70, (byte)228, (byte)231, (byte)108, (byte)174, (byte)167, (byte)20, (byte)174, (byte)183, (byte)2, (byte)155, (byte)186, (byte)8, (byte)230, (byte)238, (byte)216, (byte)23, (byte)36, (byte)155, (byte)68, (byte)118, (byte)229, (byte)74, (byte)147, (byte)32, (byte)122, (byte)65, (byte)213, (byte)75, (byte)249, (byte)0, (byte)103, (byte)72, (byte)86, (byte)244, (byte)235, (byte)53, (byte)162, (byte)179, (byte)142, (byte)77, (byte)226, (byte)73, (byte)109, (byte)1, (byte)250, (byte)73, (byte)34, (byte)192, (byte)28, (byte)132, (byte)85, (byte)120, (byte)95, (byte)166, (byte)221, (byte)117, (byte)111, (byte)153, (byte)106, (byte)25, (byte)66, (byte)125, (byte)127, (byte)207, (byte)161, (byte)193, (byte)58, (byte)192, (byte)60, (byte)130, (byte)131, (byte)105, (byte)6, (byte)99, (byte)220, (byte)129, (byte)206, (byte)224, (byte)107, (byte)228, (byte)102, (byte)116, (byte)148, (byte)72, (byte)246, (byte)42, (byte)146, (byte)151, (byte)246, (byte)85, (byte)29, (byte)82, (byte)77, (byte)24, (byte)33, (byte)237, (byte)141, (byte)153, (byte)76, (byte)31, (byte)215, (byte)204, (byte)216, (byte)190, (byte)94, (byte)62, (byte)51, (byte)4, (byte)88, (byte)236, (byte)135, (byte)192, (byte)170, (byte)51, (byte)84, (byte)206, (byte)236, (byte)56, (byte)153, (byte)179, (byte)32, (byte)210, (byte)219, (byte)216, (byte)101, (byte)93, (byte)234, (byte)245, (byte)205, (byte)12, (byte)17, (byte)152, (byte)65, (byte)81, (byte)135, (byte)58, (byte)22, (byte)27, (byte)85, (byte)236, (byte)253, (byte)77, (byte)161, (byte)134, (byte)233, (byte)79, (byte)156, (byte)196, (byte)104, (byte)102, (byte)2, (byte)203, (byte)235, (byte)8, (byte)149, (byte)38, (byte)206, (byte)198, (byte)45, (byte)202, (byte)213, (byte)207, (byte)13, (byte)233, (byte)53, (byte)57, (byte)91, (byte)98, (byte)15, (byte)55, (byte)183, (byte)55, (byte)196, (byte)173, (byte)18, (byte)139, (byte)11, (byte)17, (byte)201, (byte)193, (byte)115, (byte)16, (byte)196, (byte)248, (byte)130, (byte)210, (byte)59, (byte)116, (byte)151}, 0) ;
            p267.first_message_offset = (byte)(byte)87;
            p267.sequence = (ushort)(ushort)49206;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)214);
                Debug.Assert(pack.sequence == (ushort)(ushort)64894);
                Debug.Assert(pack.target_component == (byte)(byte)231);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)214;
            p268.target_component = (byte)(byte)231;
            p268.sequence = (ushort)(ushort)64894;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)2288996974U);
                Debug.Assert(pack.framerate == (float)4.545882E37F);
                Debug.Assert(pack.uri_LEN(ph) == 159);
                Debug.Assert(pack.uri_TRY(ph).Equals("rrjmmqjruwdthbasNrtKfzIiLcpcwkxkQiqeuxjqbvcshnSIUgashmdHxpgyxhaaxkkaccvyiXjwljerXtULwcshlvgukkFgbctihqWrabtbhJucswjnwotgjmSCofdXludwkwtnpeUenphhaSsewXkbwmuLtqa"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)38075);
                Debug.Assert(pack.status == (byte)(byte)30);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)25844);
                Debug.Assert(pack.camera_id == (byte)(byte)13);
                Debug.Assert(pack.rotation == (ushort)(ushort)39569);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)39569;
            p269.framerate = (float)4.545882E37F;
            p269.resolution_v = (ushort)(ushort)38075;
            p269.camera_id = (byte)(byte)13;
            p269.status = (byte)(byte)30;
            p269.resolution_h = (ushort)(ushort)25844;
            p269.bitrate = (uint)2288996974U;
            p269.uri_SET("rrjmmqjruwdthbasNrtKfzIiLcpcwkxkQiqeuxjqbvcshnSIUgashmdHxpgyxhaaxkkaccvyiXjwljerXtULwcshlvgukkFgbctihqWrabtbhJucswjnwotgjmSCofdXludwkwtnpeUenphhaSsewXkbwmuLtqa", PH) ;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)55);
                Debug.Assert(pack.rotation == (ushort)(ushort)2493);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)29474);
                Debug.Assert(pack.target_component == (byte)(byte)62);
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)16201);
                Debug.Assert(pack.framerate == (float)1.0736426E38F);
                Debug.Assert(pack.bitrate == (uint)1693266988U);
                Debug.Assert(pack.uri_LEN(ph) == 139);
                Debug.Assert(pack.uri_TRY(ph).Equals("iotoafxayVeEdFkrmpeboedjbvUpjGgWjqfwhfBssWnatdKoxtofRfvsjhlckmJjcqsskhAztmoGaffzhwkrsjnnYzAundnhuepcFyutwcszLqiojigjixyngmkiftkpyhceofJcugn"));
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.rotation = (ushort)(ushort)2493;
            p270.target_system = (byte)(byte)155;
            p270.resolution_h = (ushort)(ushort)29474;
            p270.framerate = (float)1.0736426E38F;
            p270.bitrate = (uint)1693266988U;
            p270.camera_id = (byte)(byte)55;
            p270.uri_SET("iotoafxayVeEdFkrmpeboedjbvUpjGgWjqfwhfBssWnatdKoxtofRfvsjhlckmJjcqsskhAztmoGaffzhwkrsjnnYzAundnhuepcFyutwcszLqiojigjixyngmkiftkpyhceofJcugn", PH) ;
            p270.resolution_v = (ushort)(ushort)16201;
            p270.target_component = (byte)(byte)62;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 12);
                Debug.Assert(pack.password_TRY(ph).Equals("ArpaqpPrljli"));
                Debug.Assert(pack.ssid_LEN(ph) == 3);
                Debug.Assert(pack.ssid_TRY(ph).Equals("lrj"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("ArpaqpPrljli", PH) ;
            p299.ssid_SET("lrj", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)165, (byte)21, (byte)165, (byte)224, (byte)42, (byte)206, (byte)95, (byte)255}));
                Debug.Assert(pack.min_version == (ushort)(ushort)10246);
                Debug.Assert(pack.version == (ushort)(ushort)41283);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)232, (byte)207, (byte)222, (byte)190, (byte)101, (byte)24, (byte)209, (byte)208}));
                Debug.Assert(pack.max_version == (ushort)(ushort)53087);
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)165, (byte)21, (byte)165, (byte)224, (byte)42, (byte)206, (byte)95, (byte)255}, 0) ;
            p300.min_version = (ushort)(ushort)10246;
            p300.version = (ushort)(ushort)41283;
            p300.max_version = (ushort)(ushort)53087;
            p300.library_version_hash_SET(new byte[] {(byte)232, (byte)207, (byte)222, (byte)190, (byte)101, (byte)24, (byte)209, (byte)208}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)12915);
                Debug.Assert(pack.time_usec == (ulong)3805264660940289172L);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.uptime_sec == (uint)3418964402U);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.sub_mode == (byte)(byte)102);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)3418964402U;
            p310.vendor_specific_status_code = (ushort)(ushort)12915;
            p310.time_usec = (ulong)3805264660940289172L;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)102;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)17, (byte)221, (byte)124, (byte)193, (byte)49, (byte)82, (byte)141, (byte)59, (byte)239, (byte)129, (byte)34, (byte)121, (byte)137, (byte)250, (byte)119, (byte)4}));
                Debug.Assert(pack.name_LEN(ph) == 25);
                Debug.Assert(pack.name_TRY(ph).Equals("opesvyrflIzTvartvHwhvjprk"));
                Debug.Assert(pack.time_usec == (ulong)2501393826028768458L);
                Debug.Assert(pack.hw_version_major == (byte)(byte)255);
                Debug.Assert(pack.sw_version_major == (byte)(byte)204);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)193);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)179);
                Debug.Assert(pack.uptime_sec == (uint)4232615531U);
                Debug.Assert(pack.sw_vcs_commit == (uint)1899685666U);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.name_SET("opesvyrflIzTvartvHwhvjprk", PH) ;
            p311.time_usec = (ulong)2501393826028768458L;
            p311.sw_vcs_commit = (uint)1899685666U;
            p311.hw_version_major = (byte)(byte)255;
            p311.sw_version_major = (byte)(byte)204;
            p311.hw_version_minor = (byte)(byte)179;
            p311.sw_version_minor = (byte)(byte)193;
            p311.uptime_sec = (uint)4232615531U;
            p311.hw_unique_id_SET(new byte[] {(byte)17, (byte)221, (byte)124, (byte)193, (byte)49, (byte)82, (byte)141, (byte)59, (byte)239, (byte)129, (byte)34, (byte)121, (byte)137, (byte)250, (byte)119, (byte)4}, 0) ;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.target_system == (byte)(byte)24);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hBdpzfvdpncz"));
                Debug.Assert(pack.param_index == (short)(short)23130);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)24;
            p320.param_id_SET("hBdpzfvdpncz", PH) ;
            p320.target_component = (byte)(byte)236;
            p320.param_index = (short)(short)23130;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)196);
                Debug.Assert(pack.target_system == (byte)(byte)100);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)196;
            p321.target_system = (byte)(byte)100;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 40);
                Debug.Assert(pack.param_value_TRY(ph).Equals("rhdnRfjhpxeeTyzdixmofnqqvhYDkSquuktuYfgy"));
                Debug.Assert(pack.param_index == (ushort)(ushort)40830);
                Debug.Assert(pack.param_count == (ushort)(ushort)19896);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("NstpSmtgsyyl"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)40830;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_id_SET("NstpSmtgsyyl", PH) ;
            p322.param_value_SET("rhdnRfjhpxeeTyzdixmofnqqvhYDkSquuktuYfgy", PH) ;
            p322.param_count = (ushort)(ushort)19896;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)58);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ffm"));
                Debug.Assert(pack.param_value_LEN(ph) == 68);
                Debug.Assert(pack.param_value_TRY(ph).Equals("dAafkahYojtpntinghaFhqtufoaelnRpykQegvXstlepjndhemostoauirsmsTdjjvLi"));
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.param_id_SET("ffm", PH) ;
            p323.target_component = (byte)(byte)58;
            p323.target_system = (byte)(byte)225;
            p323.param_value_SET("dAafkahYojtpntinghaFhqtufoaelnRpykQegvXstlepjndhemostoauirsmsTdjjvLi", PH) ;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 40);
                Debug.Assert(pack.param_value_TRY(ph).Equals("rOhoFidubzpqhvtsvuQqcidiwkadbNKdqyjpvtho"));
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xjQmlafyolWjbnu"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_id_SET("xjQmlafyolWjbnu", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            p324.param_value_SET("rOhoFidubzpqhvtsvuQqcidiwkadbNKdqyjpvtho", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)244);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)29502, (ushort)59514, (ushort)24162, (ushort)36558, (ushort)40568, (ushort)49392, (ushort)16499, (ushort)5988, (ushort)29515, (ushort)45013, (ushort)5661, (ushort)46742, (ushort)50122, (ushort)11204, (ushort)1296, (ushort)49531, (ushort)31803, (ushort)18156, (ushort)48648, (ushort)997, (ushort)63907, (ushort)42780, (ushort)64002, (ushort)14989, (ushort)6295, (ushort)57747, (ushort)37639, (ushort)41950, (ushort)8221, (ushort)35429, (ushort)48963, (ushort)33662, (ushort)26348, (ushort)43979, (ushort)31269, (ushort)50955, (ushort)878, (ushort)28760, (ushort)43279, (ushort)49543, (ushort)48066, (ushort)14254, (ushort)26549, (ushort)14569, (ushort)30340, (ushort)24650, (ushort)54791, (ushort)11185, (ushort)49334, (ushort)52953, (ushort)62720, (ushort)5535, (ushort)57657, (ushort)19737, (ushort)24350, (ushort)59661, (ushort)56035, (ushort)13939, (ushort)33027, (ushort)16532, (ushort)35243, (ushort)46849, (ushort)49398, (ushort)38642, (ushort)52180, (ushort)14259, (ushort)24627, (ushort)37262, (ushort)62968, (ushort)51505, (ushort)45466, (ushort)45538}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)22637);
                Debug.Assert(pack.time_usec == (ulong)5061051868139948006L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)52041);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)29502, (ushort)59514, (ushort)24162, (ushort)36558, (ushort)40568, (ushort)49392, (ushort)16499, (ushort)5988, (ushort)29515, (ushort)45013, (ushort)5661, (ushort)46742, (ushort)50122, (ushort)11204, (ushort)1296, (ushort)49531, (ushort)31803, (ushort)18156, (ushort)48648, (ushort)997, (ushort)63907, (ushort)42780, (ushort)64002, (ushort)14989, (ushort)6295, (ushort)57747, (ushort)37639, (ushort)41950, (ushort)8221, (ushort)35429, (ushort)48963, (ushort)33662, (ushort)26348, (ushort)43979, (ushort)31269, (ushort)50955, (ushort)878, (ushort)28760, (ushort)43279, (ushort)49543, (ushort)48066, (ushort)14254, (ushort)26549, (ushort)14569, (ushort)30340, (ushort)24650, (ushort)54791, (ushort)11185, (ushort)49334, (ushort)52953, (ushort)62720, (ushort)5535, (ushort)57657, (ushort)19737, (ushort)24350, (ushort)59661, (ushort)56035, (ushort)13939, (ushort)33027, (ushort)16532, (ushort)35243, (ushort)46849, (ushort)49398, (ushort)38642, (ushort)52180, (ushort)14259, (ushort)24627, (ushort)37262, (ushort)62968, (ushort)51505, (ushort)45466, (ushort)45538}, 0) ;
            p330.time_usec = (ulong)5061051868139948006L;
            p330.max_distance = (ushort)(ushort)52041;
            p330.increment = (byte)(byte)244;
            p330.min_distance = (ushort)(ushort)22637;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
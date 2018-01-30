
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
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_KITE);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_STANDBY);
                Debug.Assert(pack.mavlink_version == (byte)(byte)119);
                Debug.Assert(pack.custom_mode == (uint)813621806U);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_KITE;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_STANDBY;
            p0.custom_mode = (uint)813621806U;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.mavlink_version = (byte)(byte)119;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_comm == (ushort)(ushort)6023);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)63980);
                Debug.Assert(pack.load == (ushort)(ushort)7617);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)90);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)29246);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)23514);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)28070);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)45370);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)51399);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
                Debug.Assert(pack.current_battery == (short)(short)21587);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)51399;
            p1.battery_remaining = (sbyte)(sbyte)90;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE;
            p1.drop_rate_comm = (ushort)(ushort)63980;
            p1.errors_comm = (ushort)(ushort)6023;
            p1.errors_count1 = (ushort)(ushort)28070;
            p1.errors_count4 = (ushort)(ushort)29246;
            p1.current_battery = (short)(short)21587;
            p1.voltage_battery = (ushort)(ushort)23514;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
            p1.load = (ushort)(ushort)7617;
            p1.errors_count2 = (ushort)(ushort)45370;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4022576405U);
                Debug.Assert(pack.time_unix_usec == (ulong)8716030003532926907L);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)8716030003532926907L;
            p2.time_boot_ms = (uint)4022576405U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)2.4891413E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)29497);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.afx == (float)4.0159174E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.5881995E37F);
                Debug.Assert(pack.yaw == (float) -2.8529365E38F);
                Debug.Assert(pack.afz == (float) -2.3620896E38F);
                Debug.Assert(pack.y == (float) -1.2501192E38F);
                Debug.Assert(pack.vy == (float)3.11565E38F);
                Debug.Assert(pack.z == (float) -4.8624456E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3135256593U);
                Debug.Assert(pack.vz == (float) -7.536606E36F);
                Debug.Assert(pack.x == (float)2.9918536E38F);
                Debug.Assert(pack.afy == (float) -1.3506984E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afx = (float)4.0159174E37F;
            p3.time_boot_ms = (uint)3135256593U;
            p3.y = (float) -1.2501192E38F;
            p3.yaw_rate = (float) -1.5881995E37F;
            p3.yaw = (float) -2.8529365E38F;
            p3.vy = (float)3.11565E38F;
            p3.vz = (float) -7.536606E36F;
            p3.type_mask = (ushort)(ushort)29497;
            p3.afz = (float) -2.3620896E38F;
            p3.afy = (float) -1.3506984E38F;
            p3.vx = (float)2.4891413E38F;
            p3.z = (float) -4.8624456E37F;
            p3.x = (float)2.9918536E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8839021940759665905L);
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.seq == (uint)1330979059U);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)163;
            p4.time_usec = (ulong)8839021940759665905L;
            p4.target_component = (byte)(byte)134;
            p4.seq = (uint)1330979059U;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 2);
                Debug.Assert(pack.passkey_TRY(ph).Equals("iy"));
                Debug.Assert(pack.version == (byte)(byte)38);
                Debug.Assert(pack.control_request == (byte)(byte)56);
                Debug.Assert(pack.target_system == (byte)(byte)27);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("iy", PH) ;
            p5.target_system = (byte)(byte)27;
            p5.version = (byte)(byte)38;
            p5.control_request = (byte)(byte)56;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)61);
                Debug.Assert(pack.control_request == (byte)(byte)138);
                Debug.Assert(pack.ack == (byte)(byte)125);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)61;
            p6.ack = (byte)(byte)125;
            p6.control_request = (byte)(byte)138;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 26);
                Debug.Assert(pack.key_TRY(ph).Equals("oemhqdqStXwxbihflktmiaslkw"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("oemhqdqStXwxbihflktmiaslkw", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.custom_mode == (uint)484874621U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)128;
            p11.custom_mode = (uint)484874621U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("l"));
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.target_component == (byte)(byte)32);
                Debug.Assert(pack.param_index == (short)(short)8283);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short)8283;
            p20.param_id_SET("l", PH) ;
            p20.target_component = (byte)(byte)32;
            p20.target_system = (byte)(byte)51;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.target_component == (byte)(byte)232);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)232;
            p21.target_system = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("HccxfZdkrgtPSddb"));
                Debug.Assert(pack.param_value == (float) -1.8654582E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)22883);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)4083);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)22883;
            p22.param_count = (ushort)(ushort)4083;
            p22.param_id_SET("HccxfZdkrgtPSddb", PH) ;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64;
            p22.param_value = (float) -1.8654582E38F;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rrywxCksflusaz"));
                Debug.Assert(pack.param_value == (float) -9.423497E37F);
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.target_system == (byte)(byte)78);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_component = (byte)(byte)165;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p23.target_system = (byte)(byte)78;
            p23.param_value = (float) -9.423497E37F;
            p23.param_id_SET("rrywxCksflusaz", PH) ;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1390745327U);
                Debug.Assert(pack.lat == (int)1431670057);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)171303879);
                Debug.Assert(pack.time_usec == (ulong)7481032917956389228L);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.cog == (ushort)(ushort)47755);
                Debug.Assert(pack.satellites_visible == (byte)(byte)112);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1212411669U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)295829474U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1599360538U);
                Debug.Assert(pack.lon == (int) -927014546);
                Debug.Assert(pack.vel == (ushort)(ushort)18206);
                Debug.Assert(pack.epv == (ushort)(ushort)15816);
                Debug.Assert(pack.eph == (ushort)(ushort)15220);
                Debug.Assert(pack.alt == (int)1143423585);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.lat = (int)1431670057;
            p24.v_acc_SET((uint)1599360538U, PH) ;
            p24.vel = (ushort)(ushort)18206;
            p24.satellites_visible = (byte)(byte)112;
            p24.alt = (int)1143423585;
            p24.vel_acc_SET((uint)1390745327U, PH) ;
            p24.time_usec = (ulong)7481032917956389228L;
            p24.alt_ellipsoid_SET((int)171303879, PH) ;
            p24.eph = (ushort)(ushort)15220;
            p24.h_acc_SET((uint)1212411669U, PH) ;
            p24.epv = (ushort)(ushort)15816;
            p24.lon = (int) -927014546;
            p24.cog = (ushort)(ushort)47755;
            p24.hdg_acc_SET((uint)295829474U, PH) ;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)137);
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)255, (byte)237, (byte)24, (byte)1, (byte)24, (byte)128, (byte)76, (byte)67, (byte)41, (byte)107, (byte)219, (byte)86, (byte)224, (byte)32, (byte)159, (byte)150, (byte)78, (byte)70, (byte)119, (byte)55}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)162, (byte)88, (byte)113, (byte)90, (byte)220, (byte)108, (byte)69, (byte)67, (byte)84, (byte)228, (byte)33, (byte)101, (byte)41, (byte)85, (byte)66, (byte)146, (byte)195, (byte)33, (byte)93, (byte)123}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)25, (byte)165, (byte)27, (byte)218, (byte)108, (byte)155, (byte)175, (byte)66, (byte)61, (byte)49, (byte)30, (byte)95, (byte)125, (byte)201, (byte)242, (byte)149, (byte)212, (byte)236, (byte)208, (byte)101}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)32, (byte)107, (byte)35, (byte)143, (byte)61, (byte)4, (byte)11, (byte)109, (byte)83, (byte)6, (byte)52, (byte)200, (byte)40, (byte)200, (byte)83, (byte)103, (byte)227, (byte)214, (byte)24, (byte)164}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)7, (byte)79, (byte)212, (byte)54, (byte)164, (byte)176, (byte)224, (byte)247, (byte)210, (byte)244, (byte)137, (byte)153, (byte)106, (byte)33, (byte)9, (byte)182, (byte)38, (byte)195, (byte)194, (byte)185}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)7, (byte)79, (byte)212, (byte)54, (byte)164, (byte)176, (byte)224, (byte)247, (byte)210, (byte)244, (byte)137, (byte)153, (byte)106, (byte)33, (byte)9, (byte)182, (byte)38, (byte)195, (byte)194, (byte)185}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)162, (byte)88, (byte)113, (byte)90, (byte)220, (byte)108, (byte)69, (byte)67, (byte)84, (byte)228, (byte)33, (byte)101, (byte)41, (byte)85, (byte)66, (byte)146, (byte)195, (byte)33, (byte)93, (byte)123}, 0) ;
            p25.satellites_visible = (byte)(byte)137;
            p25.satellite_snr_SET(new byte[] {(byte)255, (byte)237, (byte)24, (byte)1, (byte)24, (byte)128, (byte)76, (byte)67, (byte)41, (byte)107, (byte)219, (byte)86, (byte)224, (byte)32, (byte)159, (byte)150, (byte)78, (byte)70, (byte)119, (byte)55}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)25, (byte)165, (byte)27, (byte)218, (byte)108, (byte)155, (byte)175, (byte)66, (byte)61, (byte)49, (byte)30, (byte)95, (byte)125, (byte)201, (byte)242, (byte)149, (byte)212, (byte)236, (byte)208, (byte)101}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)32, (byte)107, (byte)35, (byte)143, (byte)61, (byte)4, (byte)11, (byte)109, (byte)83, (byte)6, (byte)52, (byte)200, (byte)40, (byte)200, (byte)83, (byte)103, (byte)227, (byte)214, (byte)24, (byte)164}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -10405);
                Debug.Assert(pack.ygyro == (short)(short) -28649);
                Debug.Assert(pack.xgyro == (short)(short)20247);
                Debug.Assert(pack.xmag == (short)(short)5256);
                Debug.Assert(pack.time_boot_ms == (uint)3165687470U);
                Debug.Assert(pack.yacc == (short)(short) -13987);
                Debug.Assert(pack.zacc == (short)(short)17466);
                Debug.Assert(pack.zgyro == (short)(short)3429);
                Debug.Assert(pack.xacc == (short)(short)24641);
                Debug.Assert(pack.zmag == (short)(short) -13788);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short) -13987;
            p26.ymag = (short)(short) -10405;
            p26.zacc = (short)(short)17466;
            p26.xacc = (short)(short)24641;
            p26.xgyro = (short)(short)20247;
            p26.zgyro = (short)(short)3429;
            p26.xmag = (short)(short)5256;
            p26.zmag = (short)(short) -13788;
            p26.ygyro = (short)(short) -28649;
            p26.time_boot_ms = (uint)3165687470U;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -24453);
                Debug.Assert(pack.time_usec == (ulong)7092809987655884270L);
                Debug.Assert(pack.xmag == (short)(short) -22157);
                Debug.Assert(pack.zmag == (short)(short) -7039);
                Debug.Assert(pack.xgyro == (short)(short) -13040);
                Debug.Assert(pack.ymag == (short)(short)21390);
                Debug.Assert(pack.ygyro == (short)(short) -7785);
                Debug.Assert(pack.zacc == (short)(short) -13039);
                Debug.Assert(pack.yacc == (short)(short)8013);
                Debug.Assert(pack.xacc == (short)(short)22190);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.ygyro = (short)(short) -7785;
            p27.yacc = (short)(short)8013;
            p27.ymag = (short)(short)21390;
            p27.xgyro = (short)(short) -13040;
            p27.zgyro = (short)(short) -24453;
            p27.xmag = (short)(short) -22157;
            p27.zacc = (short)(short) -13039;
            p27.time_usec = (ulong)7092809987655884270L;
            p27.xacc = (short)(short)22190;
            p27.zmag = (short)(short) -7039;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)13108);
                Debug.Assert(pack.time_usec == (ulong)3132801934663844862L);
                Debug.Assert(pack.press_diff1 == (short)(short)23377);
                Debug.Assert(pack.press_diff2 == (short)(short) -21571);
                Debug.Assert(pack.press_abs == (short)(short)5136);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short)13108;
            p28.press_diff2 = (short)(short) -21571;
            p28.time_usec = (ulong)3132801934663844862L;
            p28.press_diff1 = (short)(short)23377;
            p28.press_abs = (short)(short)5136;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1431585002U);
                Debug.Assert(pack.temperature == (short)(short)22366);
                Debug.Assert(pack.press_diff == (float)2.8597508E38F);
                Debug.Assert(pack.press_abs == (float)3.3027014E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)22366;
            p29.press_diff = (float)2.8597508E38F;
            p29.time_boot_ms = (uint)1431585002U;
            p29.press_abs = (float)3.3027014E38F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)1.9542252E38F);
                Debug.Assert(pack.pitchspeed == (float) -5.96481E37F);
                Debug.Assert(pack.rollspeed == (float)3.3142727E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3936607462U);
                Debug.Assert(pack.pitch == (float)9.808992E37F);
                Debug.Assert(pack.yaw == (float)1.2772965E38F);
                Debug.Assert(pack.roll == (float)4.1303641E37F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float)1.2772965E38F;
            p30.rollspeed = (float)3.3142727E37F;
            p30.pitch = (float)9.808992E37F;
            p30.yawspeed = (float)1.9542252E38F;
            p30.time_boot_ms = (uint)3936607462U;
            p30.roll = (float)4.1303641E37F;
            p30.pitchspeed = (float) -5.96481E37F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)573901264U);
                Debug.Assert(pack.q1 == (float)7.3753205E37F);
                Debug.Assert(pack.yawspeed == (float) -1.2505368E38F);
                Debug.Assert(pack.q2 == (float)3.0008216E38F);
                Debug.Assert(pack.rollspeed == (float) -2.8125133E38F);
                Debug.Assert(pack.q3 == (float)1.6204345E38F);
                Debug.Assert(pack.q4 == (float) -1.8330846E38F);
                Debug.Assert(pack.pitchspeed == (float)7.4136253E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float)7.4136253E37F;
            p31.time_boot_ms = (uint)573901264U;
            p31.q3 = (float)1.6204345E38F;
            p31.q4 = (float) -1.8330846E38F;
            p31.q2 = (float)3.0008216E38F;
            p31.yawspeed = (float) -1.2505368E38F;
            p31.q1 = (float)7.3753205E37F;
            p31.rollspeed = (float) -2.8125133E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1175242648U);
                Debug.Assert(pack.vz == (float) -2.1271477E38F);
                Debug.Assert(pack.vy == (float)5.545268E37F);
                Debug.Assert(pack.x == (float) -8.365082E37F);
                Debug.Assert(pack.vx == (float) -1.7270833E38F);
                Debug.Assert(pack.y == (float) -2.071067E38F);
                Debug.Assert(pack.z == (float)6.016129E37F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1175242648U;
            p32.vx = (float) -1.7270833E38F;
            p32.z = (float)6.016129E37F;
            p32.x = (float) -8.365082E37F;
            p32.vy = (float)5.545268E37F;
            p32.vz = (float) -2.1271477E38F;
            p32.y = (float) -2.071067E38F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -843638841);
                Debug.Assert(pack.vx == (short)(short)2100);
                Debug.Assert(pack.lat == (int) -1450481997);
                Debug.Assert(pack.vy == (short)(short)189);
                Debug.Assert(pack.lon == (int) -1210279165);
                Debug.Assert(pack.time_boot_ms == (uint)3722041163U);
                Debug.Assert(pack.relative_alt == (int) -18776429);
                Debug.Assert(pack.hdg == (ushort)(ushort)33223);
                Debug.Assert(pack.vz == (short)(short)2337);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int) -1210279165;
            p33.hdg = (ushort)(ushort)33223;
            p33.relative_alt = (int) -18776429;
            p33.time_boot_ms = (uint)3722041163U;
            p33.alt = (int) -843638841;
            p33.vx = (short)(short)2100;
            p33.lat = (int) -1450481997;
            p33.vy = (short)(short)189;
            p33.vz = (short)(short)2337;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_scaled == (short)(short)2724);
                Debug.Assert(pack.time_boot_ms == (uint)2226726893U);
                Debug.Assert(pack.chan8_scaled == (short)(short)30499);
                Debug.Assert(pack.chan1_scaled == (short)(short)31636);
                Debug.Assert(pack.rssi == (byte)(byte)134);
                Debug.Assert(pack.port == (byte)(byte)202);
                Debug.Assert(pack.chan5_scaled == (short)(short) -19351);
                Debug.Assert(pack.chan3_scaled == (short)(short)19661);
                Debug.Assert(pack.chan4_scaled == (short)(short)7687);
                Debug.Assert(pack.chan7_scaled == (short)(short) -13197);
                Debug.Assert(pack.chan6_scaled == (short)(short)21936);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan4_scaled = (short)(short)7687;
            p34.chan3_scaled = (short)(short)19661;
            p34.chan1_scaled = (short)(short)31636;
            p34.rssi = (byte)(byte)134;
            p34.port = (byte)(byte)202;
            p34.chan5_scaled = (short)(short) -19351;
            p34.chan7_scaled = (short)(short) -13197;
            p34.time_boot_ms = (uint)2226726893U;
            p34.chan6_scaled = (short)(short)21936;
            p34.chan2_scaled = (short)(short)2724;
            p34.chan8_scaled = (short)(short)30499;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)1539);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)29032);
                Debug.Assert(pack.rssi == (byte)(byte)4);
                Debug.Assert(pack.time_boot_ms == (uint)3457499870U);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)473);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)49481);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)35516);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)14858);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)36923);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)22238);
                Debug.Assert(pack.port == (byte)(byte)122);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.port = (byte)(byte)122;
            p35.chan5_raw = (ushort)(ushort)35516;
            p35.rssi = (byte)(byte)4;
            p35.chan6_raw = (ushort)(ushort)1539;
            p35.chan3_raw = (ushort)(ushort)49481;
            p35.chan8_raw = (ushort)(ushort)29032;
            p35.chan7_raw = (ushort)(ushort)473;
            p35.chan2_raw = (ushort)(ushort)36923;
            p35.chan4_raw = (ushort)(ushort)22238;
            p35.chan1_raw = (ushort)(ushort)14858;
            p35.time_boot_ms = (uint)3457499870U;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)375);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)58060);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)33929);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)16943);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)2700);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)45143);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)50404);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)37562);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)31529);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)50392);
                Debug.Assert(pack.time_usec == (uint)4233304630U);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)58244);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)15626);
                Debug.Assert(pack.port == (byte)(byte)53);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)10635);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)10055);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)8975);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)60704);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo1_raw = (ushort)(ushort)33929;
            p36.servo5_raw = (ushort)(ushort)58060;
            p36.servo15_raw_SET((ushort)(ushort)16943, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)45143, PH) ;
            p36.port = (byte)(byte)53;
            p36.servo7_raw = (ushort)(ushort)10055;
            p36.servo3_raw = (ushort)(ushort)15626;
            p36.servo2_raw = (ushort)(ushort)31529;
            p36.servo8_raw = (ushort)(ushort)50404;
            p36.servo9_raw_SET((ushort)(ushort)50392, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)10635, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)60704, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)375, PH) ;
            p36.servo6_raw = (ushort)(ushort)2700;
            p36.servo4_raw = (ushort)(ushort)37562;
            p36.servo11_raw_SET((ushort)(ushort)8975, PH) ;
            p36.time_usec = (uint)4233304630U;
            p36.servo12_raw_SET((ushort)(ushort)58244, PH) ;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)15735);
                Debug.Assert(pack.target_system == (byte)(byte)76);
                Debug.Assert(pack.end_index == (short)(short)12652);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)85);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)76;
            p37.end_index = (short)(short)12652;
            p37.start_index = (short)(short)15735;
            p37.target_component = (byte)(byte)85;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)74);
                Debug.Assert(pack.start_index == (short)(short)1093);
                Debug.Assert(pack.end_index == (short)(short)9686);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)251);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)251;
            p38.end_index = (short)(short)9686;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.start_index = (short)(short)1093;
            p38.target_component = (byte)(byte)74;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.5674582E37F);
                Debug.Assert(pack.y == (float)1.4709983E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)247);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPOSITION);
                Debug.Assert(pack.param3 == (float)2.8587845E38F);
                Debug.Assert(pack.x == (float)7.817118E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)49030);
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.current == (byte)(byte)92);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param1 == (float)5.9716616E37F);
                Debug.Assert(pack.target_component == (byte)(byte)90);
                Debug.Assert(pack.param4 == (float) -6.8120907E37F);
                Debug.Assert(pack.param2 == (float)3.2544071E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.seq = (ushort)(ushort)49030;
            p39.x = (float)7.817118E37F;
            p39.target_system = (byte)(byte)173;
            p39.z = (float) -2.5674582E37F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_REPOSITION;
            p39.param2 = (float)3.2544071E38F;
            p39.y = (float)1.4709983E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.param3 = (float)2.8587845E38F;
            p39.param4 = (float) -6.8120907E37F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p39.param1 = (float)5.9716616E37F;
            p39.autocontinue = (byte)(byte)247;
            p39.current = (byte)(byte)92;
            p39.target_component = (byte)(byte)90;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.seq == (ushort)(ushort)25851);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)248;
            p40.seq = (ushort)(ushort)25851;
            p40.target_system = (byte)(byte)228;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)23001);
                Debug.Assert(pack.target_component == (byte)(byte)240);
                Debug.Assert(pack.target_system == (byte)(byte)225);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)23001;
            p41.target_system = (byte)(byte)225;
            p41.target_component = (byte)(byte)240;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)9715);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)9715;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.target_system == (byte)(byte)14);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)208;
            p43.target_system = (byte)(byte)14;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)188);
                Debug.Assert(pack.target_component == (byte)(byte)151);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.count == (ushort)(ushort)23892);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)188;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)23892;
            p44.target_component = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)232);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)232;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_component = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)15002);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)15002;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
                Debug.Assert(pack.target_component == (byte)(byte)89);
                Debug.Assert(pack.target_system == (byte)(byte)15);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED;
            p47.target_system = (byte)(byte)15;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_component = (byte)(byte)89;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -1437018911);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4420630837405076068L);
                Debug.Assert(pack.latitude == (int)700562866);
                Debug.Assert(pack.altitude == (int) -1962933404);
                Debug.Assert(pack.target_system == (byte)(byte)133);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)133;
            p48.longitude = (int) -1437018911;
            p48.latitude = (int)700562866;
            p48.time_usec_SET((ulong)4420630837405076068L, PH) ;
            p48.altitude = (int) -1962933404;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -346086218);
                Debug.Assert(pack.longitude == (int) -218979454);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8807271217680847002L);
                Debug.Assert(pack.altitude == (int) -486063692);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int) -346086218;
            p49.longitude = (int) -218979454;
            p49.time_usec_SET((ulong)8807271217680847002L, PH) ;
            p49.altitude = (int) -486063692;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float)3.3453034E38F);
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.param_value_min == (float) -1.7291626E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)103);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fhXhTrHuis"));
                Debug.Assert(pack.param_value0 == (float) -1.7617636E38F);
                Debug.Assert(pack.scale == (float) -1.0802398E38F);
                Debug.Assert(pack.param_index == (short)(short) -26482);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value0 = (float) -1.7617636E38F;
            p50.target_component = (byte)(byte)213;
            p50.parameter_rc_channel_index = (byte)(byte)103;
            p50.param_value_min = (float) -1.7291626E38F;
            p50.param_id_SET("fhXhTrHuis", PH) ;
            p50.scale = (float) -1.0802398E38F;
            p50.param_value_max = (float)3.3453034E38F;
            p50.target_system = (byte)(byte)172;
            p50.param_index = (short)(short) -26482;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.seq == (ushort)(ushort)29400);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)92);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)29400;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_system = (byte)(byte)198;
            p51.target_component = (byte)(byte)92;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)3.1900198E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.p1x == (float) -1.726783E38F);
                Debug.Assert(pack.p2x == (float)1.1618999E38F);
                Debug.Assert(pack.p2z == (float) -2.0075507E38F);
                Debug.Assert(pack.p2y == (float) -3.0524266E38F);
                Debug.Assert(pack.p1y == (float) -2.2645974E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)34;
            p54.p2x = (float)1.1618999E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p54.p1y = (float) -2.2645974E38F;
            p54.p2z = (float) -2.0075507E38F;
            p54.p1z = (float)3.1900198E38F;
            p54.p1x = (float) -1.726783E38F;
            p54.target_component = (byte)(byte)247;
            p54.p2y = (float) -3.0524266E38F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)2.4472723E38F);
                Debug.Assert(pack.p1y == (float)3.1151645E38F);
                Debug.Assert(pack.p2z == (float) -1.2869118E38F);
                Debug.Assert(pack.p1x == (float)1.7618106E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p2y == (float)2.4318356E38F);
                Debug.Assert(pack.p2x == (float)1.4431812E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float)2.4318356E38F;
            p55.p1z = (float)2.4472723E38F;
            p55.p2z = (float) -1.2869118E38F;
            p55.p1y = (float)3.1151645E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p55.p1x = (float)1.7618106E38F;
            p55.p2x = (float)1.4431812E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.4907984E38F, 1.9952553E38F, 3.0829042E38F, 3.3899898E38F, 2.728458E38F, 1.7384481E38F, -2.080873E38F, 8.869318E37F, 2.5101758E38F}));
                Debug.Assert(pack.rollspeed == (float)1.0137769E38F);
                Debug.Assert(pack.pitchspeed == (float) -3.3876674E37F);
                Debug.Assert(pack.yawspeed == (float) -2.370456E38F);
                Debug.Assert(pack.time_usec == (ulong)2763923759765459460L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.0799825E38F, 2.2918808E38F, 2.2898E38F, -5.8742036E37F}));
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float) -3.3876674E37F;
            p61.covariance_SET(new float[] {-1.4907984E38F, 1.9952553E38F, 3.0829042E38F, 3.3899898E38F, 2.728458E38F, 1.7384481E38F, -2.080873E38F, 8.869318E37F, 2.5101758E38F}, 0) ;
            p61.time_usec = (ulong)2763923759765459460L;
            p61.yawspeed = (float) -2.370456E38F;
            p61.rollspeed = (float)1.0137769E38F;
            p61.q_SET(new float[] {1.0799825E38F, 2.2918808E38F, 2.2898E38F, -5.8742036E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float) -2.6732741E38F);
                Debug.Assert(pack.target_bearing == (short)(short)27245);
                Debug.Assert(pack.aspd_error == (float) -4.977466E36F);
                Debug.Assert(pack.xtrack_error == (float) -2.959157E38F);
                Debug.Assert(pack.nav_roll == (float) -2.1162153E38F);
                Debug.Assert(pack.alt_error == (float)1.7001099E37F);
                Debug.Assert(pack.nav_bearing == (short)(short)29734);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)32885);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short)29734;
            p62.aspd_error = (float) -4.977466E36F;
            p62.alt_error = (float)1.7001099E37F;
            p62.nav_roll = (float) -2.1162153E38F;
            p62.xtrack_error = (float) -2.959157E38F;
            p62.target_bearing = (short)(short)27245;
            p62.wp_dist = (ushort)(ushort)32885;
            p62.nav_pitch = (float) -2.6732741E38F;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-8.733379E37F, -2.4461841E38F, -2.2425748E37F, 2.275525E38F, -9.088184E37F, -2.0095734E38F, 4.373152E37F, -8.2703695E37F, 3.2683837E38F, 3.3965852E38F, -5.258523E37F, 1.5636446E38F, -2.924691E37F, -2.8561432E38F, 2.1312018E38F, 2.5847511E38F, 1.8804327E38F, 2.373081E38F, -1.6320181E38F, -1.6425697E38F, -1.7557038E38F, 3.0420807E38F, -5.0423322E36F, 2.0292293E38F, 1.8966728E38F, -2.9516267E38F, -1.1613875E38F, -1.473319E38F, 3.0035056E38F, -6.8971956E37F, 2.0421204E38F, -1.8740871E38F, 1.2487153E38F, -3.0232899E38F, 3.0940145E38F, 1.9230583E37F}));
                Debug.Assert(pack.alt == (int)1710139192);
                Debug.Assert(pack.time_usec == (ulong)2667465248092135317L);
                Debug.Assert(pack.vy == (float) -4.295904E37F);
                Debug.Assert(pack.lon == (int) -1011175159);
                Debug.Assert(pack.vz == (float) -1.713944E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vx == (float)2.9018275E38F);
                Debug.Assert(pack.relative_alt == (int)1441421903);
                Debug.Assert(pack.lat == (int) -941632069);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)2667465248092135317L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.alt = (int)1710139192;
            p63.covariance_SET(new float[] {-8.733379E37F, -2.4461841E38F, -2.2425748E37F, 2.275525E38F, -9.088184E37F, -2.0095734E38F, 4.373152E37F, -8.2703695E37F, 3.2683837E38F, 3.3965852E38F, -5.258523E37F, 1.5636446E38F, -2.924691E37F, -2.8561432E38F, 2.1312018E38F, 2.5847511E38F, 1.8804327E38F, 2.373081E38F, -1.6320181E38F, -1.6425697E38F, -1.7557038E38F, 3.0420807E38F, -5.0423322E36F, 2.0292293E38F, 1.8966728E38F, -2.9516267E38F, -1.1613875E38F, -1.473319E38F, 3.0035056E38F, -6.8971956E37F, 2.0421204E38F, -1.8740871E38F, 1.2487153E38F, -3.0232899E38F, 3.0940145E38F, 1.9230583E37F}, 0) ;
            p63.relative_alt = (int)1441421903;
            p63.vx = (float)2.9018275E38F;
            p63.lat = (int) -941632069;
            p63.vy = (float) -4.295904E37F;
            p63.lon = (int) -1011175159;
            p63.vz = (float) -1.713944E38F;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float)1.3471728E38F);
                Debug.Assert(pack.vx == (float) -1.0765609E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.y == (float)2.4115361E38F);
                Debug.Assert(pack.z == (float) -2.8242353E38F);
                Debug.Assert(pack.vy == (float) -8.961166E37F);
                Debug.Assert(pack.ax == (float)2.4578676E38F);
                Debug.Assert(pack.ay == (float) -3.370666E37F);
                Debug.Assert(pack.time_usec == (ulong)4866745775843072227L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.1397235E38F, -1.4423714E38F, -1.9940963E38F, 6.426449E37F, -2.4510675E38F, -1.146489E38F, 1.9588031E38F, -2.4179153E38F, -3.0130832E38F, 2.5496332E38F, 3.1142372E38F, 3.3587465E38F, -2.17786E38F, -3.2166346E38F, -1.8370825E38F, -2.2080893E38F, 5.944518E37F, 5.7156576E37F, -1.8422062E38F, -2.1810178E38F, 1.9061404E38F, -1.1026282E37F, 4.9005897E37F, 1.8693719E38F, 2.5918234E38F, -8.943978E37F, 2.66481E37F, -2.4957355E38F, 5.1849445E36F, -1.2692418E38F, -2.6859178E38F, -3.0282396E38F, -2.2117759E38F, -2.7109313E38F, 2.870444E38F, 7.4473753E37F, 1.922144E38F, -1.4897848E38F, -3.2408946E38F, 4.596805E37F, -6.383932E37F, 1.6067606E38F, -2.1373423E38F, 1.0692914E38F, 1.5839288E38F}));
                Debug.Assert(pack.vz == (float)3.2475144E38F);
                Debug.Assert(pack.x == (float)1.8947803E36F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float) -1.0765609E38F;
            p64.vz = (float)3.2475144E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.ay = (float) -3.370666E37F;
            p64.x = (float)1.8947803E36F;
            p64.y = (float)2.4115361E38F;
            p64.z = (float) -2.8242353E38F;
            p64.ax = (float)2.4578676E38F;
            p64.az = (float)1.3471728E38F;
            p64.covariance_SET(new float[] {-3.1397235E38F, -1.4423714E38F, -1.9940963E38F, 6.426449E37F, -2.4510675E38F, -1.146489E38F, 1.9588031E38F, -2.4179153E38F, -3.0130832E38F, 2.5496332E38F, 3.1142372E38F, 3.3587465E38F, -2.17786E38F, -3.2166346E38F, -1.8370825E38F, -2.2080893E38F, 5.944518E37F, 5.7156576E37F, -1.8422062E38F, -2.1810178E38F, 1.9061404E38F, -1.1026282E37F, 4.9005897E37F, 1.8693719E38F, 2.5918234E38F, -8.943978E37F, 2.66481E37F, -2.4957355E38F, 5.1849445E36F, -1.2692418E38F, -2.6859178E38F, -3.0282396E38F, -2.2117759E38F, -2.7109313E38F, 2.870444E38F, 7.4473753E37F, 1.922144E38F, -1.4897848E38F, -3.2408946E38F, 4.596805E37F, -6.383932E37F, 1.6067606E38F, -2.1373423E38F, 1.0692914E38F, 1.5839288E38F}, 0) ;
            p64.time_usec = (ulong)4866745775843072227L;
            p64.vy = (float) -8.961166E37F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)35560);
                Debug.Assert(pack.time_boot_ms == (uint)2332518866U);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)10647);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)43953);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)52977);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)11994);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)37521);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)20948);
                Debug.Assert(pack.rssi == (byte)(byte)198);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)775);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6346);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)52115);
                Debug.Assert(pack.chancount == (byte)(byte)65);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)17714);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)24908);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)15947);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)55266);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)59341);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)15615);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)64686);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)59971);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chancount = (byte)(byte)65;
            p65.chan5_raw = (ushort)(ushort)17714;
            p65.chan14_raw = (ushort)(ushort)11994;
            p65.chan16_raw = (ushort)(ushort)64686;
            p65.chan9_raw = (ushort)(ushort)20948;
            p65.chan12_raw = (ushort)(ushort)59971;
            p65.chan11_raw = (ushort)(ushort)15947;
            p65.chan15_raw = (ushort)(ushort)37521;
            p65.chan4_raw = (ushort)(ushort)52977;
            p65.chan10_raw = (ushort)(ushort)10647;
            p65.time_boot_ms = (uint)2332518866U;
            p65.chan7_raw = (ushort)(ushort)55266;
            p65.chan1_raw = (ushort)(ushort)15615;
            p65.rssi = (byte)(byte)198;
            p65.chan6_raw = (ushort)(ushort)24908;
            p65.chan3_raw = (ushort)(ushort)52115;
            p65.chan8_raw = (ushort)(ushort)775;
            p65.chan13_raw = (ushort)(ushort)43953;
            p65.chan17_raw = (ushort)(ushort)59341;
            p65.chan18_raw = (ushort)(ushort)35560;
            p65.chan2_raw = (ushort)(ushort)6346;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)63019);
                Debug.Assert(pack.req_stream_id == (byte)(byte)108);
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.start_stop == (byte)(byte)154);
                Debug.Assert(pack.target_system == (byte)(byte)167);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)225;
            p66.req_stream_id = (byte)(byte)108;
            p66.target_system = (byte)(byte)167;
            p66.req_message_rate = (ushort)(ushort)63019;
            p66.start_stop = (byte)(byte)154;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)57);
                Debug.Assert(pack.message_rate == (ushort)(ushort)48716);
                Debug.Assert(pack.stream_id == (byte)(byte)18);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)57;
            p67.message_rate = (ushort)(ushort)48716;
            p67.stream_id = (byte)(byte)18;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (short)(short) -10702);
                Debug.Assert(pack.r == (short)(short)16098);
                Debug.Assert(pack.target == (byte)(byte)34);
                Debug.Assert(pack.z == (short)(short) -17871);
                Debug.Assert(pack.buttons == (ushort)(ushort)51339);
                Debug.Assert(pack.y == (short)(short) -9260);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short) -9260;
            p69.buttons = (ushort)(ushort)51339;
            p69.r = (short)(short)16098;
            p69.x = (short)(short) -10702;
            p69.target = (byte)(byte)34;
            p69.z = (short)(short) -17871;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)57668);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)55470);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)58068);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)4434);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)56943);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)46280);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)36673);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)11358);
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.target_system == (byte)(byte)225);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)57668;
            p70.chan5_raw = (ushort)(ushort)46280;
            p70.chan1_raw = (ushort)(ushort)11358;
            p70.target_system = (byte)(byte)225;
            p70.target_component = (byte)(byte)132;
            p70.chan2_raw = (ushort)(ushort)56943;
            p70.chan7_raw = (ushort)(ushort)36673;
            p70.chan8_raw = (ushort)(ushort)58068;
            p70.chan3_raw = (ushort)(ushort)55470;
            p70.chan6_raw = (ushort)(ushort)4434;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)2047728604);
                Debug.Assert(pack.seq == (ushort)(ushort)36457);
                Debug.Assert(pack.y == (int) -1657746008);
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.current == (byte)(byte)121);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.autocontinue == (byte)(byte)231);
                Debug.Assert(pack.z == (float) -1.760898E38F);
                Debug.Assert(pack.param4 == (float)4.9941443E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_DELAY);
                Debug.Assert(pack.param2 == (float)1.8049142E38F);
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param3 == (float) -3.3217419E38F);
                Debug.Assert(pack.param1 == (float)2.0211119E38F);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.seq = (ushort)(ushort)36457;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_DELAY;
            p73.z = (float) -1.760898E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.autocontinue = (byte)(byte)231;
            p73.param3 = (float) -3.3217419E38F;
            p73.param2 = (float)1.8049142E38F;
            p73.current = (byte)(byte)121;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p73.target_system = (byte)(byte)90;
            p73.x = (int)2047728604;
            p73.param4 = (float)4.9941443E37F;
            p73.target_component = (byte)(byte)47;
            p73.param1 = (float)2.0211119E38F;
            p73.y = (int) -1657746008;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float) -2.0353593E38F);
                Debug.Assert(pack.heading == (short)(short)18554);
                Debug.Assert(pack.groundspeed == (float) -2.0008094E38F);
                Debug.Assert(pack.alt == (float) -3.3178432E37F);
                Debug.Assert(pack.airspeed == (float) -3.0197827E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)10609);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.throttle = (ushort)(ushort)10609;
            p74.airspeed = (float) -3.0197827E38F;
            p74.heading = (short)(short)18554;
            p74.climb = (float) -2.0353593E38F;
            p74.groundspeed = (float) -2.0008094E38F;
            p74.alt = (float) -3.3178432E37F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -1493849980);
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.param4 == (float)1.9906735E38F);
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS);
                Debug.Assert(pack.current == (byte)(byte)62);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.autocontinue == (byte)(byte)88);
                Debug.Assert(pack.z == (float) -7.153536E37F);
                Debug.Assert(pack.param1 == (float)2.2002646E38F);
                Debug.Assert(pack.x == (int) -1948622522);
                Debug.Assert(pack.param2 == (float) -3.0241257E38F);
                Debug.Assert(pack.param3 == (float) -2.194861E38F);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.current = (byte)(byte)62;
            p75.z = (float) -7.153536E37F;
            p75.x = (int) -1948622522;
            p75.param1 = (float)2.2002646E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p75.autocontinue = (byte)(byte)88;
            p75.target_component = (byte)(byte)208;
            p75.y = (int) -1493849980;
            p75.param4 = (float)1.9906735E38F;
            p75.param2 = (float) -3.0241257E38F;
            p75.target_system = (byte)(byte)245;
            p75.param3 = (float) -2.194861E38F;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float)1.88012E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)236);
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.param3 == (float) -2.870409E38F);
                Debug.Assert(pack.param5 == (float)4.749618E37F);
                Debug.Assert(pack.param1 == (float)2.3185857E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_4);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.param7 == (float) -1.3419498E38F);
                Debug.Assert(pack.param2 == (float) -1.0115462E38F);
                Debug.Assert(pack.param6 == (float)8.949546E37F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)172;
            p76.param7 = (float) -1.3419498E38F;
            p76.param2 = (float) -1.0115462E38F;
            p76.param6 = (float)8.949546E37F;
            p76.param4 = (float)1.88012E38F;
            p76.target_system = (byte)(byte)131;
            p76.param5 = (float)4.749618E37F;
            p76.confirmation = (byte)(byte)236;
            p76.param1 = (float)2.3185857E38F;
            p76.param3 = (float) -2.870409E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_4;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)196);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1459819083);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)230);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)104);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.result_param2_SET((int)1459819083, PH) ;
            p77.progress_SET((byte)(byte)230, PH) ;
            p77.target_component_SET((byte)(byte)196, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE;
            p77.target_system_SET((byte)(byte)104, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.7838116E38F);
                Debug.Assert(pack.roll == (float)2.1978753E38F);
                Debug.Assert(pack.thrust == (float) -2.9474864E38F);
                Debug.Assert(pack.pitch == (float) -1.268318E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3396584173U);
                Debug.Assert(pack.mode_switch == (byte)(byte)212);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)37);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)37;
            p81.time_boot_ms = (uint)3396584173U;
            p81.thrust = (float) -2.9474864E38F;
            p81.mode_switch = (byte)(byte)212;
            p81.pitch = (float) -1.268318E38F;
            p81.roll = (float)2.1978753E38F;
            p81.yaw = (float)2.7838116E38F;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)254);
                Debug.Assert(pack.body_pitch_rate == (float) -1.8316229E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4634102E38F, 3.1453829E38F, 5.129917E36F, 5.136123E37F}));
                Debug.Assert(pack.type_mask == (byte)(byte)161);
                Debug.Assert(pack.body_roll_rate == (float)1.0536555E37F);
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.thrust == (float)2.7014402E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3191834967U);
                Debug.Assert(pack.body_yaw_rate == (float) -1.1892996E38F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_pitch_rate = (float) -1.8316229E38F;
            p82.q_SET(new float[] {1.4634102E38F, 3.1453829E38F, 5.129917E36F, 5.136123E37F}, 0) ;
            p82.target_component = (byte)(byte)254;
            p82.time_boot_ms = (uint)3191834967U;
            p82.body_yaw_rate = (float) -1.1892996E38F;
            p82.body_roll_rate = (float)1.0536555E37F;
            p82.thrust = (float)2.7014402E37F;
            p82.target_system = (byte)(byte)199;
            p82.type_mask = (byte)(byte)161;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float) -8.941824E37F);
                Debug.Assert(pack.body_pitch_rate == (float)1.323081E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.0636269E37F, -5.7432416E37F, -2.4376534E38F, -3.0594313E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)146);
                Debug.Assert(pack.body_roll_rate == (float)2.5376177E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2636619363U);
                Debug.Assert(pack.thrust == (float) -2.2160987E37F);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float)1.323081E38F;
            p83.body_roll_rate = (float)2.5376177E38F;
            p83.body_yaw_rate = (float) -8.941824E37F;
            p83.q_SET(new float[] {-1.0636269E37F, -5.7432416E37F, -2.4376534E38F, -3.0594313E38F}, 0) ;
            p83.time_boot_ms = (uint)2636619363U;
            p83.thrust = (float) -2.2160987E37F;
            p83.type_mask = (byte)(byte)146;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3411168302U);
                Debug.Assert(pack.afy == (float) -1.0352241E38F);
                Debug.Assert(pack.x == (float)3.1283935E38F);
                Debug.Assert(pack.yaw_rate == (float) -9.911256E37F);
                Debug.Assert(pack.afz == (float)2.6702005E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.target_component == (byte)(byte)70);
                Debug.Assert(pack.vy == (float)2.641193E38F);
                Debug.Assert(pack.z == (float) -5.8829545E37F);
                Debug.Assert(pack.y == (float)3.2257759E38F);
                Debug.Assert(pack.target_system == (byte)(byte)167);
                Debug.Assert(pack.vz == (float)2.3186857E38F);
                Debug.Assert(pack.afx == (float) -5.5363366E36F);
                Debug.Assert(pack.yaw == (float)8.0971557E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)30495);
                Debug.Assert(pack.vx == (float) -1.9216275E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.target_system = (byte)(byte)167;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p84.y = (float)3.2257759E38F;
            p84.target_component = (byte)(byte)70;
            p84.time_boot_ms = (uint)3411168302U;
            p84.vz = (float)2.3186857E38F;
            p84.afx = (float) -5.5363366E36F;
            p84.yaw_rate = (float) -9.911256E37F;
            p84.afy = (float) -1.0352241E38F;
            p84.vx = (float) -1.9216275E38F;
            p84.x = (float)3.1283935E38F;
            p84.afz = (float)2.6702005E38F;
            p84.vy = (float)2.641193E38F;
            p84.z = (float) -5.8829545E37F;
            p84.type_mask = (ushort)(ushort)30495;
            p84.yaw = (float)8.0971557E37F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw == (float) -2.5536448E38F);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.time_boot_ms == (uint)2061163412U);
                Debug.Assert(pack.afz == (float) -1.3560486E38F);
                Debug.Assert(pack.vx == (float) -2.5719947E38F);
                Debug.Assert(pack.yaw_rate == (float)1.3161614E38F);
                Debug.Assert(pack.alt == (float) -1.3090797E38F);
                Debug.Assert(pack.vy == (float)2.0383623E38F);
                Debug.Assert(pack.lon_int == (int)1077443206);
                Debug.Assert(pack.lat_int == (int) -1243231246);
                Debug.Assert(pack.type_mask == (ushort)(ushort)49169);
                Debug.Assert(pack.target_system == (byte)(byte)76);
                Debug.Assert(pack.afx == (float) -2.8007008E38F);
                Debug.Assert(pack.vz == (float) -2.4282703E38F);
                Debug.Assert(pack.afy == (float)2.230704E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.target_system = (byte)(byte)76;
            p86.lon_int = (int)1077443206;
            p86.afz = (float) -1.3560486E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.yaw = (float) -2.5536448E38F;
            p86.afy = (float)2.230704E38F;
            p86.yaw_rate = (float)1.3161614E38F;
            p86.lat_int = (int) -1243231246;
            p86.type_mask = (ushort)(ushort)49169;
            p86.alt = (float) -1.3090797E38F;
            p86.target_component = (byte)(byte)100;
            p86.vz = (float) -2.4282703E38F;
            p86.vy = (float)2.0383623E38F;
            p86.vx = (float) -2.5719947E38F;
            p86.time_boot_ms = (uint)2061163412U;
            p86.afx = (float) -2.8007008E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)1.831477E38F);
                Debug.Assert(pack.afy == (float) -2.2191057E38F);
                Debug.Assert(pack.afx == (float)1.3569416E38F);
                Debug.Assert(pack.vz == (float)6.475944E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.yaw == (float)1.1260355E38F);
                Debug.Assert(pack.lat_int == (int) -720545534);
                Debug.Assert(pack.lon_int == (int)1056466510);
                Debug.Assert(pack.yaw_rate == (float) -4.8523424E37F);
                Debug.Assert(pack.afz == (float) -6.8018485E37F);
                Debug.Assert(pack.vy == (float) -3.4027502E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)51059);
                Debug.Assert(pack.time_boot_ms == (uint)2881285380U);
                Debug.Assert(pack.alt == (float)1.3436511E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.alt = (float)1.3436511E38F;
            p87.type_mask = (ushort)(ushort)51059;
            p87.vx = (float)1.831477E38F;
            p87.yaw_rate = (float) -4.8523424E37F;
            p87.afz = (float) -6.8018485E37F;
            p87.lon_int = (int)1056466510;
            p87.lat_int = (int) -720545534;
            p87.afx = (float)1.3569416E38F;
            p87.vy = (float) -3.4027502E38F;
            p87.vz = (float)6.475944E37F;
            p87.time_boot_ms = (uint)2881285380U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p87.yaw = (float)1.1260355E38F;
            p87.afy = (float) -2.2191057E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.865207E38F);
                Debug.Assert(pack.roll == (float)4.8569723E37F);
                Debug.Assert(pack.yaw == (float)3.540532E37F);
                Debug.Assert(pack.x == (float) -1.3343372E38F);
                Debug.Assert(pack.pitch == (float) -6.26178E37F);
                Debug.Assert(pack.y == (float)4.3000113E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1316028893U);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float) -1.3343372E38F;
            p89.y = (float)4.3000113E37F;
            p89.pitch = (float) -6.26178E37F;
            p89.roll = (float)4.8569723E37F;
            p89.yaw = (float)3.540532E37F;
            p89.time_boot_ms = (uint)1316028893U;
            p89.z = (float) -2.865207E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1248210981);
                Debug.Assert(pack.roll == (float) -3.3789428E38F);
                Debug.Assert(pack.vx == (short)(short) -32224);
                Debug.Assert(pack.rollspeed == (float)4.6828773E37F);
                Debug.Assert(pack.lat == (int)1643684064);
                Debug.Assert(pack.yaw == (float)3.2255534E38F);
                Debug.Assert(pack.vy == (short)(short) -17938);
                Debug.Assert(pack.lon == (int)1234538801);
                Debug.Assert(pack.yacc == (short)(short)19515);
                Debug.Assert(pack.time_usec == (ulong)508163741617358270L);
                Debug.Assert(pack.yawspeed == (float) -1.5210273E36F);
                Debug.Assert(pack.xacc == (short)(short)1723);
                Debug.Assert(pack.zacc == (short)(short)24902);
                Debug.Assert(pack.pitchspeed == (float) -2.192008E38F);
                Debug.Assert(pack.pitch == (float)3.381161E38F);
                Debug.Assert(pack.vz == (short)(short) -20040);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.lon = (int)1234538801;
            p90.zacc = (short)(short)24902;
            p90.vz = (short)(short) -20040;
            p90.pitchspeed = (float) -2.192008E38F;
            p90.time_usec = (ulong)508163741617358270L;
            p90.yacc = (short)(short)19515;
            p90.yawspeed = (float) -1.5210273E36F;
            p90.vx = (short)(short) -32224;
            p90.alt = (int) -1248210981;
            p90.roll = (float) -3.3789428E38F;
            p90.lat = (int)1643684064;
            p90.rollspeed = (float)4.6828773E37F;
            p90.yaw = (float)3.2255534E38F;
            p90.pitch = (float)3.381161E38F;
            p90.vy = (short)(short) -17938;
            p90.xacc = (short)(short)1723;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float) -3.131376E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)128);
                Debug.Assert(pack.roll_ailerons == (float)2.8294203E38F);
                Debug.Assert(pack.yaw_rudder == (float)2.5011296E38F);
                Debug.Assert(pack.aux3 == (float) -3.0840426E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.pitch_elevator == (float)2.261837E38F);
                Debug.Assert(pack.aux2 == (float) -2.7499662E38F);
                Debug.Assert(pack.time_usec == (ulong)4374828979945037144L);
                Debug.Assert(pack.aux1 == (float)1.412335E38F);
                Debug.Assert(pack.aux4 == (float)2.1722333E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.pitch_elevator = (float)2.261837E38F;
            p91.throttle = (float) -3.131376E38F;
            p91.aux1 = (float)1.412335E38F;
            p91.aux2 = (float) -2.7499662E38F;
            p91.aux4 = (float)2.1722333E38F;
            p91.nav_mode = (byte)(byte)128;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.yaw_rudder = (float)2.5011296E38F;
            p91.aux3 = (float) -3.0840426E38F;
            p91.time_usec = (ulong)4374828979945037144L;
            p91.roll_ailerons = (float)2.8294203E38F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)21832);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)65059);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)30534);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)5733);
                Debug.Assert(pack.time_usec == (ulong)6987570353626156428L);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)47131);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)64370);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)5663);
                Debug.Assert(pack.rssi == (byte)(byte)233);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)14727);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51724);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)62118);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)6871);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)40101);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan9_raw = (ushort)(ushort)62118;
            p92.chan5_raw = (ushort)(ushort)6871;
            p92.chan11_raw = (ushort)(ushort)5733;
            p92.chan10_raw = (ushort)(ushort)64370;
            p92.time_usec = (ulong)6987570353626156428L;
            p92.rssi = (byte)(byte)233;
            p92.chan7_raw = (ushort)(ushort)30534;
            p92.chan6_raw = (ushort)(ushort)47131;
            p92.chan4_raw = (ushort)(ushort)5663;
            p92.chan1_raw = (ushort)(ushort)14727;
            p92.chan8_raw = (ushort)(ushort)21832;
            p92.chan2_raw = (ushort)(ushort)51724;
            p92.chan3_raw = (ushort)(ushort)40101;
            p92.chan12_raw = (ushort)(ushort)65059;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.7728985E38F, -2.830984E38F, 3.1941696E38F, 1.7716385E38F, 3.246413E38F, -2.5837512E38F, 6.600938E37F, 7.11994E37F, -3.3922133E38F, 2.5373398E38F, -2.3770739E38F, -2.91446E37F, -2.4146507E38F, -2.4978357E38F, -1.7044124E38F, -2.4618055E38F}));
                Debug.Assert(pack.flags == (ulong)2946983894188204662L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)7724526615921008200L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p93.time_usec = (ulong)7724526615921008200L;
            p93.controls_SET(new float[] {2.7728985E38F, -2.830984E38F, 3.1941696E38F, 1.7716385E38F, 3.246413E38F, -2.5837512E38F, 6.600938E37F, 7.11994E37F, -3.3922133E38F, 2.5373398E38F, -2.3770739E38F, -2.91446E37F, -2.4146507E38F, -2.4978357E38F, -1.7044124E38F, -2.4618055E38F}, 0) ;
            p93.flags = (ulong)2946983894188204662L;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short) -2132);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.7972758E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.0429642E37F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.2608242E37F);
                Debug.Assert(pack.quality == (byte)(byte)203);
                Debug.Assert(pack.flow_comp_m_x == (float) -1.3704736E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)61);
                Debug.Assert(pack.time_usec == (ulong)7485880266588191064L);
                Debug.Assert(pack.ground_distance == (float)2.36137E38F);
                Debug.Assert(pack.flow_x == (short)(short)10212);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float) -1.7972758E38F;
            p100.sensor_id = (byte)(byte)61;
            p100.time_usec = (ulong)7485880266588191064L;
            p100.flow_rate_x_SET((float) -2.0429642E37F, PH) ;
            p100.flow_y = (short)(short) -2132;
            p100.flow_x = (short)(short)10212;
            p100.flow_rate_y_SET((float)1.2608242E37F, PH) ;
            p100.ground_distance = (float)2.36137E38F;
            p100.quality = (byte)(byte)203;
            p100.flow_comp_m_x = (float) -1.3704736E38F;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.5785295E38F);
                Debug.Assert(pack.x == (float) -2.953522E38F);
                Debug.Assert(pack.pitch == (float) -4.4815444E37F);
                Debug.Assert(pack.roll == (float)3.1467883E38F);
                Debug.Assert(pack.y == (float) -3.519529E37F);
                Debug.Assert(pack.usec == (ulong)1380374213802761123L);
                Debug.Assert(pack.z == (float)3.0716975E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float)3.0716975E38F;
            p101.x = (float) -2.953522E38F;
            p101.roll = (float)3.1467883E38F;
            p101.yaw = (float) -1.5785295E38F;
            p101.y = (float) -3.519529E37F;
            p101.usec = (ulong)1380374213802761123L;
            p101.pitch = (float) -4.4815444E37F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -2.7598476E38F);
                Debug.Assert(pack.y == (float) -3.6896832E37F);
                Debug.Assert(pack.usec == (ulong)6639740992561315645L);
                Debug.Assert(pack.yaw == (float)5.7256416E37F);
                Debug.Assert(pack.z == (float)8.808845E37F);
                Debug.Assert(pack.x == (float)3.329335E38F);
                Debug.Assert(pack.pitch == (float)2.578953E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.yaw = (float)5.7256416E37F;
            p102.x = (float)3.329335E38F;
            p102.roll = (float) -2.7598476E38F;
            p102.pitch = (float)2.578953E37F;
            p102.z = (float)8.808845E37F;
            p102.usec = (ulong)6639740992561315645L;
            p102.y = (float) -3.6896832E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)6933856647234218659L);
                Debug.Assert(pack.z == (float) -3.1113561E38F);
                Debug.Assert(pack.y == (float) -1.8304903E38F);
                Debug.Assert(pack.x == (float) -8.745194E37F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float) -8.745194E37F;
            p103.y = (float) -1.8304903E38F;
            p103.usec = (ulong)6933856647234218659L;
            p103.z = (float) -3.1113561E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.348187E38F);
                Debug.Assert(pack.roll == (float) -4.0844176E37F);
                Debug.Assert(pack.z == (float)1.5558169E38F);
                Debug.Assert(pack.y == (float)2.2451776E38F);
                Debug.Assert(pack.usec == (ulong)2884030743234201212L);
                Debug.Assert(pack.yaw == (float) -2.5562576E38F);
                Debug.Assert(pack.pitch == (float)1.5940367E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float) -2.5562576E38F;
            p104.pitch = (float)1.5940367E38F;
            p104.z = (float)1.5558169E38F;
            p104.roll = (float) -4.0844176E37F;
            p104.usec = (ulong)2884030743234201212L;
            p104.x = (float)3.348187E38F;
            p104.y = (float)2.2451776E38F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float)5.2139174E35F);
                Debug.Assert(pack.zmag == (float) -8.32597E36F);
                Debug.Assert(pack.yacc == (float)3.3503072E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.6309766E38F);
                Debug.Assert(pack.ygyro == (float) -3.0342087E38F);
                Debug.Assert(pack.xgyro == (float)1.1798312E38F);
                Debug.Assert(pack.abs_pressure == (float) -2.720553E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)59661);
                Debug.Assert(pack.ymag == (float)1.1625416E37F);
                Debug.Assert(pack.time_usec == (ulong)4171022905294337580L);
                Debug.Assert(pack.temperature == (float)2.100389E38F);
                Debug.Assert(pack.xmag == (float)4.4253186E37F);
                Debug.Assert(pack.zacc == (float) -6.519366E37F);
                Debug.Assert(pack.diff_pressure == (float)1.7567883E38F);
                Debug.Assert(pack.xacc == (float) -3.2089098E38F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.diff_pressure = (float)1.7567883E38F;
            p105.abs_pressure = (float) -2.720553E38F;
            p105.zgyro = (float)5.2139174E35F;
            p105.ymag = (float)1.1625416E37F;
            p105.fields_updated = (ushort)(ushort)59661;
            p105.xgyro = (float)1.1798312E38F;
            p105.yacc = (float)3.3503072E38F;
            p105.zmag = (float) -8.32597E36F;
            p105.zacc = (float) -6.519366E37F;
            p105.ygyro = (float) -3.0342087E38F;
            p105.xmag = (float)4.4253186E37F;
            p105.time_usec = (ulong)4171022905294337580L;
            p105.xacc = (float) -3.2089098E38F;
            p105.temperature = (float)2.100389E38F;
            p105.pressure_alt = (float) -2.6309766E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)3260138572U);
                Debug.Assert(pack.integrated_ygyro == (float)1.3526447E38F);
                Debug.Assert(pack.time_usec == (ulong)7852812476586019290L);
                Debug.Assert(pack.integrated_y == (float) -2.7408957E38F);
                Debug.Assert(pack.temperature == (short)(short)2784);
                Debug.Assert(pack.integrated_x == (float) -2.1733565E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)77);
                Debug.Assert(pack.quality == (byte)(byte)36);
                Debug.Assert(pack.distance == (float) -9.404623E37F);
                Debug.Assert(pack.integrated_xgyro == (float) -7.5719666E37F);
                Debug.Assert(pack.integration_time_us == (uint)781945075U);
                Debug.Assert(pack.integrated_zgyro == (float) -1.6531469E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.temperature = (short)(short)2784;
            p106.integrated_xgyro = (float) -7.5719666E37F;
            p106.integrated_zgyro = (float) -1.6531469E38F;
            p106.quality = (byte)(byte)36;
            p106.time_delta_distance_us = (uint)3260138572U;
            p106.time_usec = (ulong)7852812476586019290L;
            p106.integration_time_us = (uint)781945075U;
            p106.sensor_id = (byte)(byte)77;
            p106.integrated_y = (float) -2.7408957E38F;
            p106.integrated_ygyro = (float)1.3526447E38F;
            p106.integrated_x = (float) -2.1733565E38F;
            p106.distance = (float) -9.404623E37F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float) -1.5473744E38F);
                Debug.Assert(pack.zgyro == (float)3.0128909E38F);
                Debug.Assert(pack.xgyro == (float)8.588834E37F);
                Debug.Assert(pack.zacc == (float)2.5057554E38F);
                Debug.Assert(pack.diff_pressure == (float)2.182185E38F);
                Debug.Assert(pack.fields_updated == (uint)2716575285U);
                Debug.Assert(pack.abs_pressure == (float) -2.434953E38F);
                Debug.Assert(pack.ygyro == (float) -2.2552802E38F);
                Debug.Assert(pack.ymag == (float) -2.6814183E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.2825515E38F);
                Debug.Assert(pack.time_usec == (ulong)3245594478006657113L);
                Debug.Assert(pack.temperature == (float) -7.7495066E37F);
                Debug.Assert(pack.xmag == (float) -1.3523388E38F);
                Debug.Assert(pack.xacc == (float)1.9583119E38F);
                Debug.Assert(pack.zmag == (float) -1.460197E37F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float) -2.434953E38F;
            p107.ygyro = (float) -2.2552802E38F;
            p107.fields_updated = (uint)2716575285U;
            p107.xacc = (float)1.9583119E38F;
            p107.ymag = (float) -2.6814183E38F;
            p107.zacc = (float)2.5057554E38F;
            p107.time_usec = (ulong)3245594478006657113L;
            p107.xgyro = (float)8.588834E37F;
            p107.yacc = (float) -1.5473744E38F;
            p107.pressure_alt = (float) -2.2825515E38F;
            p107.temperature = (float) -7.7495066E37F;
            p107.xmag = (float) -1.3523388E38F;
            p107.zmag = (float) -1.460197E37F;
            p107.diff_pressure = (float)2.182185E38F;
            p107.zgyro = (float)3.0128909E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.0149817E38F);
                Debug.Assert(pack.yaw == (float)2.6998744E37F);
                Debug.Assert(pack.xacc == (float) -8.1394775E37F);
                Debug.Assert(pack.vn == (float)9.194431E37F);
                Debug.Assert(pack.ve == (float)2.4542841E38F);
                Debug.Assert(pack.q4 == (float)9.3728425E36F);
                Debug.Assert(pack.xgyro == (float) -2.2900938E38F);
                Debug.Assert(pack.alt == (float) -9.284938E37F);
                Debug.Assert(pack.lat == (float) -1.010678E38F);
                Debug.Assert(pack.q3 == (float)3.1613387E38F);
                Debug.Assert(pack.zgyro == (float)9.459346E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.7442597E38F);
                Debug.Assert(pack.std_dev_vert == (float) -3.173419E38F);
                Debug.Assert(pack.q2 == (float) -1.1771973E38F);
                Debug.Assert(pack.roll == (float)5.6374075E37F);
                Debug.Assert(pack.q1 == (float) -1.6135629E37F);
                Debug.Assert(pack.vd == (float) -2.686121E37F);
                Debug.Assert(pack.zacc == (float) -1.477476E38F);
                Debug.Assert(pack.yacc == (float) -3.1836517E38F);
                Debug.Assert(pack.ygyro == (float)3.1083803E38F);
                Debug.Assert(pack.lon == (float) -1.2744082E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.ve = (float)2.4542841E38F;
            p108.lon = (float) -1.2744082E38F;
            p108.pitch = (float) -2.0149817E38F;
            p108.xgyro = (float) -2.2900938E38F;
            p108.std_dev_horz = (float)2.7442597E38F;
            p108.alt = (float) -9.284938E37F;
            p108.vd = (float) -2.686121E37F;
            p108.vn = (float)9.194431E37F;
            p108.roll = (float)5.6374075E37F;
            p108.std_dev_vert = (float) -3.173419E38F;
            p108.xacc = (float) -8.1394775E37F;
            p108.q4 = (float)9.3728425E36F;
            p108.ygyro = (float)3.1083803E38F;
            p108.q1 = (float) -1.6135629E37F;
            p108.yaw = (float)2.6998744E37F;
            p108.zgyro = (float)9.459346E37F;
            p108.q3 = (float)3.1613387E38F;
            p108.q2 = (float) -1.1771973E38F;
            p108.lat = (float) -1.010678E38F;
            p108.yacc = (float) -3.1836517E38F;
            p108.zacc = (float) -1.477476E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)24932);
                Debug.Assert(pack.remrssi == (byte)(byte)39);
                Debug.Assert(pack.remnoise == (byte)(byte)96);
                Debug.Assert(pack.rssi == (byte)(byte)233);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)65199);
                Debug.Assert(pack.noise == (byte)(byte)105);
                Debug.Assert(pack.txbuf == (byte)(byte)44);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)105;
            p109.fixed_ = (ushort)(ushort)24932;
            p109.remrssi = (byte)(byte)39;
            p109.txbuf = (byte)(byte)44;
            p109.remnoise = (byte)(byte)96;
            p109.rssi = (byte)(byte)233;
            p109.rxerrors = (ushort)(ushort)65199;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.target_network == (byte)(byte)174);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)185, (byte)141, (byte)41, (byte)107, (byte)220, (byte)78, (byte)107, (byte)172, (byte)88, (byte)83, (byte)128, (byte)74, (byte)132, (byte)199, (byte)235, (byte)160, (byte)196, (byte)236, (byte)204, (byte)157, (byte)140, (byte)33, (byte)4, (byte)39, (byte)72, (byte)197, (byte)233, (byte)255, (byte)80, (byte)205, (byte)216, (byte)75, (byte)12, (byte)38, (byte)138, (byte)203, (byte)252, (byte)242, (byte)238, (byte)48, (byte)91, (byte)125, (byte)67, (byte)86, (byte)155, (byte)137, (byte)38, (byte)109, (byte)114, (byte)63, (byte)11, (byte)147, (byte)74, (byte)222, (byte)211, (byte)42, (byte)182, (byte)33, (byte)40, (byte)34, (byte)52, (byte)90, (byte)94, (byte)28, (byte)188, (byte)165, (byte)187, (byte)5, (byte)122, (byte)215, (byte)246, (byte)136, (byte)233, (byte)206, (byte)222, (byte)190, (byte)153, (byte)196, (byte)105, (byte)84, (byte)23, (byte)82, (byte)169, (byte)187, (byte)125, (byte)124, (byte)98, (byte)30, (byte)165, (byte)207, (byte)193, (byte)250, (byte)141, (byte)176, (byte)114, (byte)122, (byte)248, (byte)179, (byte)66, (byte)194, (byte)220, (byte)59, (byte)214, (byte)112, (byte)41, (byte)245, (byte)247, (byte)242, (byte)108, (byte)218, (byte)99, (byte)179, (byte)64, (byte)210, (byte)210, (byte)175, (byte)113, (byte)24, (byte)128, (byte)167, (byte)24, (byte)197, (byte)8, (byte)242, (byte)45, (byte)93, (byte)146, (byte)171, (byte)106, (byte)75, (byte)232, (byte)29, (byte)147, (byte)157, (byte)95, (byte)72, (byte)235, (byte)5, (byte)190, (byte)211, (byte)11, (byte)193, (byte)133, (byte)254, (byte)85, (byte)40, (byte)254, (byte)199, (byte)65, (byte)144, (byte)143, (byte)38, (byte)115, (byte)177, (byte)219, (byte)151, (byte)173, (byte)51, (byte)126, (byte)77, (byte)7, (byte)93, (byte)167, (byte)223, (byte)238, (byte)104, (byte)46, (byte)171, (byte)79, (byte)197, (byte)254, (byte)251, (byte)60, (byte)174, (byte)193, (byte)51, (byte)217, (byte)55, (byte)44, (byte)74, (byte)92, (byte)205, (byte)248, (byte)169, (byte)133, (byte)175, (byte)130, (byte)98, (byte)226, (byte)76, (byte)31, (byte)0, (byte)10, (byte)38, (byte)93, (byte)162, (byte)1, (byte)246, (byte)228, (byte)70, (byte)224, (byte)102, (byte)111, (byte)52, (byte)70, (byte)120, (byte)171, (byte)142, (byte)74, (byte)239, (byte)38, (byte)119, (byte)173, (byte)66, (byte)209, (byte)63, (byte)178, (byte)65, (byte)54, (byte)102, (byte)215, (byte)44, (byte)192, (byte)117, (byte)75, (byte)135, (byte)126, (byte)172, (byte)166, (byte)51, (byte)157, (byte)121, (byte)136, (byte)116, (byte)118, (byte)43, (byte)184, (byte)204, (byte)110, (byte)130, (byte)165, (byte)8, (byte)120, (byte)146, (byte)114, (byte)184, (byte)251, (byte)224, (byte)207, (byte)242, (byte)242}));
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)174;
            p110.payload_SET(new byte[] {(byte)185, (byte)141, (byte)41, (byte)107, (byte)220, (byte)78, (byte)107, (byte)172, (byte)88, (byte)83, (byte)128, (byte)74, (byte)132, (byte)199, (byte)235, (byte)160, (byte)196, (byte)236, (byte)204, (byte)157, (byte)140, (byte)33, (byte)4, (byte)39, (byte)72, (byte)197, (byte)233, (byte)255, (byte)80, (byte)205, (byte)216, (byte)75, (byte)12, (byte)38, (byte)138, (byte)203, (byte)252, (byte)242, (byte)238, (byte)48, (byte)91, (byte)125, (byte)67, (byte)86, (byte)155, (byte)137, (byte)38, (byte)109, (byte)114, (byte)63, (byte)11, (byte)147, (byte)74, (byte)222, (byte)211, (byte)42, (byte)182, (byte)33, (byte)40, (byte)34, (byte)52, (byte)90, (byte)94, (byte)28, (byte)188, (byte)165, (byte)187, (byte)5, (byte)122, (byte)215, (byte)246, (byte)136, (byte)233, (byte)206, (byte)222, (byte)190, (byte)153, (byte)196, (byte)105, (byte)84, (byte)23, (byte)82, (byte)169, (byte)187, (byte)125, (byte)124, (byte)98, (byte)30, (byte)165, (byte)207, (byte)193, (byte)250, (byte)141, (byte)176, (byte)114, (byte)122, (byte)248, (byte)179, (byte)66, (byte)194, (byte)220, (byte)59, (byte)214, (byte)112, (byte)41, (byte)245, (byte)247, (byte)242, (byte)108, (byte)218, (byte)99, (byte)179, (byte)64, (byte)210, (byte)210, (byte)175, (byte)113, (byte)24, (byte)128, (byte)167, (byte)24, (byte)197, (byte)8, (byte)242, (byte)45, (byte)93, (byte)146, (byte)171, (byte)106, (byte)75, (byte)232, (byte)29, (byte)147, (byte)157, (byte)95, (byte)72, (byte)235, (byte)5, (byte)190, (byte)211, (byte)11, (byte)193, (byte)133, (byte)254, (byte)85, (byte)40, (byte)254, (byte)199, (byte)65, (byte)144, (byte)143, (byte)38, (byte)115, (byte)177, (byte)219, (byte)151, (byte)173, (byte)51, (byte)126, (byte)77, (byte)7, (byte)93, (byte)167, (byte)223, (byte)238, (byte)104, (byte)46, (byte)171, (byte)79, (byte)197, (byte)254, (byte)251, (byte)60, (byte)174, (byte)193, (byte)51, (byte)217, (byte)55, (byte)44, (byte)74, (byte)92, (byte)205, (byte)248, (byte)169, (byte)133, (byte)175, (byte)130, (byte)98, (byte)226, (byte)76, (byte)31, (byte)0, (byte)10, (byte)38, (byte)93, (byte)162, (byte)1, (byte)246, (byte)228, (byte)70, (byte)224, (byte)102, (byte)111, (byte)52, (byte)70, (byte)120, (byte)171, (byte)142, (byte)74, (byte)239, (byte)38, (byte)119, (byte)173, (byte)66, (byte)209, (byte)63, (byte)178, (byte)65, (byte)54, (byte)102, (byte)215, (byte)44, (byte)192, (byte)117, (byte)75, (byte)135, (byte)126, (byte)172, (byte)166, (byte)51, (byte)157, (byte)121, (byte)136, (byte)116, (byte)118, (byte)43, (byte)184, (byte)204, (byte)110, (byte)130, (byte)165, (byte)8, (byte)120, (byte)146, (byte)114, (byte)184, (byte)251, (byte)224, (byte)207, (byte)242, (byte)242}, 0) ;
            p110.target_component = (byte)(byte)86;
            p110.target_system = (byte)(byte)222;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -1132611049793490602L);
                Debug.Assert(pack.tc1 == (long) -5759396219123903285L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -1132611049793490602L;
            p111.tc1 = (long) -5759396219123903285L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)282875899U);
                Debug.Assert(pack.time_usec == (ulong)8460176965505409203L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8460176965505409203L;
            p112.seq = (uint)282875899U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)29502);
                Debug.Assert(pack.lat == (int)1987925598);
                Debug.Assert(pack.time_usec == (ulong)8662508992214768419L);
                Debug.Assert(pack.cog == (ushort)(ushort)8982);
                Debug.Assert(pack.lon == (int)298024990);
                Debug.Assert(pack.ve == (short)(short) -1621);
                Debug.Assert(pack.vn == (short)(short) -14546);
                Debug.Assert(pack.eph == (ushort)(ushort)26075);
                Debug.Assert(pack.fix_type == (byte)(byte)193);
                Debug.Assert(pack.satellites_visible == (byte)(byte)80);
                Debug.Assert(pack.vel == (ushort)(ushort)2415);
                Debug.Assert(pack.alt == (int)1082358177);
                Debug.Assert(pack.vd == (short)(short)4187);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vel = (ushort)(ushort)2415;
            p113.eph = (ushort)(ushort)26075;
            p113.time_usec = (ulong)8662508992214768419L;
            p113.cog = (ushort)(ushort)8982;
            p113.alt = (int)1082358177;
            p113.satellites_visible = (byte)(byte)80;
            p113.lat = (int)1987925598;
            p113.fix_type = (byte)(byte)193;
            p113.vn = (short)(short) -14546;
            p113.epv = (ushort)(ushort)29502;
            p113.ve = (short)(short) -1621;
            p113.lon = (int)298024990;
            p113.vd = (short)(short)4187;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float)5.377213E37F);
                Debug.Assert(pack.temperature == (short)(short)3649);
                Debug.Assert(pack.integrated_x == (float) -1.3656168E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -5.367578E37F);
                Debug.Assert(pack.quality == (byte)(byte)170);
                Debug.Assert(pack.integrated_ygyro == (float) -1.2861151E38F);
                Debug.Assert(pack.time_usec == (ulong)8419587988332873716L);
                Debug.Assert(pack.distance == (float) -6.1014204E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2817874037U);
                Debug.Assert(pack.sensor_id == (byte)(byte)252);
                Debug.Assert(pack.integration_time_us == (uint)645821660U);
                Debug.Assert(pack.integrated_y == (float) -2.573547E37F);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_x = (float) -1.3656168E38F;
            p114.distance = (float) -6.1014204E37F;
            p114.temperature = (short)(short)3649;
            p114.quality = (byte)(byte)170;
            p114.integrated_xgyro = (float)5.377213E37F;
            p114.sensor_id = (byte)(byte)252;
            p114.integrated_y = (float) -2.573547E37F;
            p114.integrated_zgyro = (float) -5.367578E37F;
            p114.time_usec = (ulong)8419587988332873716L;
            p114.time_delta_distance_us = (uint)2817874037U;
            p114.integration_time_us = (uint)645821660U;
            p114.integrated_ygyro = (float) -1.2861151E38F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (short)(short) -13832);
                Debug.Assert(pack.yawspeed == (float)5.094315E37F);
                Debug.Assert(pack.yacc == (short)(short) -7930);
                Debug.Assert(pack.zacc == (short)(short)24814);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)24910);
                Debug.Assert(pack.xacc == (short)(short)16747);
                Debug.Assert(pack.time_usec == (ulong)1945567467112097419L);
                Debug.Assert(pack.pitchspeed == (float)2.535121E38F);
                Debug.Assert(pack.lon == (int)880102963);
                Debug.Assert(pack.lat == (int)269455759);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.0963172E38F, -2.266871E38F, 2.4600033E37F, -3.3071033E38F}));
                Debug.Assert(pack.vy == (short)(short) -28715);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)15505);
                Debug.Assert(pack.vx == (short)(short) -31002);
                Debug.Assert(pack.alt == (int)1574440984);
                Debug.Assert(pack.rollspeed == (float)1.8557458E37F);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.attitude_quaternion_SET(new float[] {1.0963172E38F, -2.266871E38F, 2.4600033E37F, -3.3071033E38F}, 0) ;
            p115.lat = (int)269455759;
            p115.alt = (int)1574440984;
            p115.time_usec = (ulong)1945567467112097419L;
            p115.lon = (int)880102963;
            p115.xacc = (short)(short)16747;
            p115.yawspeed = (float)5.094315E37F;
            p115.zacc = (short)(short)24814;
            p115.yacc = (short)(short) -7930;
            p115.pitchspeed = (float)2.535121E38F;
            p115.true_airspeed = (ushort)(ushort)24910;
            p115.vx = (short)(short) -31002;
            p115.vy = (short)(short) -28715;
            p115.vz = (short)(short) -13832;
            p115.ind_airspeed = (ushort)(ushort)15505;
            p115.rollspeed = (float)1.8557458E37F;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -8920);
                Debug.Assert(pack.xacc == (short)(short) -473);
                Debug.Assert(pack.ymag == (short)(short)17802);
                Debug.Assert(pack.xgyro == (short)(short)22191);
                Debug.Assert(pack.time_boot_ms == (uint)2509303039U);
                Debug.Assert(pack.yacc == (short)(short) -22566);
                Debug.Assert(pack.ygyro == (short)(short) -29829);
                Debug.Assert(pack.zmag == (short)(short)19595);
                Debug.Assert(pack.xmag == (short)(short) -6647);
                Debug.Assert(pack.zgyro == (short)(short) -10300);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xacc = (short)(short) -473;
            p116.yacc = (short)(short) -22566;
            p116.xmag = (short)(short) -6647;
            p116.ygyro = (short)(short) -29829;
            p116.zgyro = (short)(short) -10300;
            p116.time_boot_ms = (uint)2509303039U;
            p116.ymag = (short)(short)17802;
            p116.zacc = (short)(short) -8920;
            p116.xgyro = (short)(short)22191;
            p116.zmag = (short)(short)19595;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)22);
                Debug.Assert(pack.target_system == (byte)(byte)63);
                Debug.Assert(pack.start == (ushort)(ushort)17710);
                Debug.Assert(pack.end == (ushort)(ushort)16332);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)16332;
            p117.target_component = (byte)(byte)22;
            p117.start = (ushort)(ushort)17710;
            p117.target_system = (byte)(byte)63;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)41466);
                Debug.Assert(pack.id == (ushort)(ushort)24215);
                Debug.Assert(pack.size == (uint)981683297U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)51338);
                Debug.Assert(pack.time_utc == (uint)3352513423U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)24215;
            p118.size = (uint)981683297U;
            p118.last_log_num = (ushort)(ushort)41466;
            p118.time_utc = (uint)3352513423U;
            p118.num_logs = (ushort)(ushort)51338;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)1630253565U);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.count == (uint)1354583621U);
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.id == (ushort)(ushort)34349);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)1630253565U;
            p119.count = (uint)1354583621U;
            p119.target_component = (byte)(byte)95;
            p119.id = (ushort)(ushort)34349;
            p119.target_system = (byte)(byte)163;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3157185568U);
                Debug.Assert(pack.id == (ushort)(ushort)53635);
                Debug.Assert(pack.count == (byte)(byte)19);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)145, (byte)202, (byte)116, (byte)137, (byte)19, (byte)139, (byte)159, (byte)129, (byte)175, (byte)164, (byte)78, (byte)8, (byte)123, (byte)206, (byte)128, (byte)109, (byte)226, (byte)209, (byte)233, (byte)226, (byte)134, (byte)98, (byte)229, (byte)82, (byte)16, (byte)69, (byte)89, (byte)169, (byte)4, (byte)191, (byte)148, (byte)156, (byte)212, (byte)42, (byte)49, (byte)124, (byte)230, (byte)68, (byte)169, (byte)71, (byte)11, (byte)228, (byte)24, (byte)50, (byte)197, (byte)113, (byte)32, (byte)39, (byte)147, (byte)92, (byte)179, (byte)193, (byte)102, (byte)125, (byte)52, (byte)202, (byte)167, (byte)251, (byte)139, (byte)19, (byte)74, (byte)137, (byte)83, (byte)23, (byte)52, (byte)168, (byte)172, (byte)148, (byte)110, (byte)124, (byte)211, (byte)59, (byte)157, (byte)158, (byte)228, (byte)109, (byte)190, (byte)2, (byte)79, (byte)150, (byte)211, (byte)112, (byte)155, (byte)50, (byte)46, (byte)159, (byte)135, (byte)213, (byte)39, (byte)27}));
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)19;
            p120.ofs = (uint)3157185568U;
            p120.data__SET(new byte[] {(byte)145, (byte)202, (byte)116, (byte)137, (byte)19, (byte)139, (byte)159, (byte)129, (byte)175, (byte)164, (byte)78, (byte)8, (byte)123, (byte)206, (byte)128, (byte)109, (byte)226, (byte)209, (byte)233, (byte)226, (byte)134, (byte)98, (byte)229, (byte)82, (byte)16, (byte)69, (byte)89, (byte)169, (byte)4, (byte)191, (byte)148, (byte)156, (byte)212, (byte)42, (byte)49, (byte)124, (byte)230, (byte)68, (byte)169, (byte)71, (byte)11, (byte)228, (byte)24, (byte)50, (byte)197, (byte)113, (byte)32, (byte)39, (byte)147, (byte)92, (byte)179, (byte)193, (byte)102, (byte)125, (byte)52, (byte)202, (byte)167, (byte)251, (byte)139, (byte)19, (byte)74, (byte)137, (byte)83, (byte)23, (byte)52, (byte)168, (byte)172, (byte)148, (byte)110, (byte)124, (byte)211, (byte)59, (byte)157, (byte)158, (byte)228, (byte)109, (byte)190, (byte)2, (byte)79, (byte)150, (byte)211, (byte)112, (byte)155, (byte)50, (byte)46, (byte)159, (byte)135, (byte)213, (byte)39, (byte)27}, 0) ;
            p120.id = (ushort)(ushort)53635;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.target_component == (byte)(byte)188);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)188;
            p121.target_system = (byte)(byte)199;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.target_component == (byte)(byte)118);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)134;
            p122.target_component = (byte)(byte)118;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)36);
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)122, (byte)250, (byte)184, (byte)255, (byte)102, (byte)223, (byte)160, (byte)53, (byte)46, (byte)30, (byte)15, (byte)19, (byte)202, (byte)57, (byte)156, (byte)87, (byte)81, (byte)36, (byte)226, (byte)34, (byte)37, (byte)203, (byte)239, (byte)4, (byte)54, (byte)46, (byte)242, (byte)197, (byte)40, (byte)216, (byte)76, (byte)248, (byte)133, (byte)127, (byte)230, (byte)237, (byte)243, (byte)55, (byte)226, (byte)61, (byte)222, (byte)127, (byte)178, (byte)152, (byte)34, (byte)106, (byte)88, (byte)139, (byte)81, (byte)22, (byte)0, (byte)31, (byte)45, (byte)84, (byte)211, (byte)187, (byte)11, (byte)207, (byte)1, (byte)100, (byte)182, (byte)47, (byte)51, (byte)143, (byte)226, (byte)133, (byte)182, (byte)51, (byte)240, (byte)242, (byte)223, (byte)7, (byte)81, (byte)77, (byte)9, (byte)40, (byte)159, (byte)1, (byte)35, (byte)32, (byte)64, (byte)11, (byte)124, (byte)145, (byte)113, (byte)96, (byte)200, (byte)182, (byte)103, (byte)149, (byte)76, (byte)251, (byte)77, (byte)224, (byte)226, (byte)161, (byte)131, (byte)209, (byte)14, (byte)138, (byte)139, (byte)136, (byte)137, (byte)185, (byte)109, (byte)94, (byte)124, (byte)244, (byte)168, (byte)18}));
                Debug.Assert(pack.target_component == (byte)(byte)56);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)122, (byte)250, (byte)184, (byte)255, (byte)102, (byte)223, (byte)160, (byte)53, (byte)46, (byte)30, (byte)15, (byte)19, (byte)202, (byte)57, (byte)156, (byte)87, (byte)81, (byte)36, (byte)226, (byte)34, (byte)37, (byte)203, (byte)239, (byte)4, (byte)54, (byte)46, (byte)242, (byte)197, (byte)40, (byte)216, (byte)76, (byte)248, (byte)133, (byte)127, (byte)230, (byte)237, (byte)243, (byte)55, (byte)226, (byte)61, (byte)222, (byte)127, (byte)178, (byte)152, (byte)34, (byte)106, (byte)88, (byte)139, (byte)81, (byte)22, (byte)0, (byte)31, (byte)45, (byte)84, (byte)211, (byte)187, (byte)11, (byte)207, (byte)1, (byte)100, (byte)182, (byte)47, (byte)51, (byte)143, (byte)226, (byte)133, (byte)182, (byte)51, (byte)240, (byte)242, (byte)223, (byte)7, (byte)81, (byte)77, (byte)9, (byte)40, (byte)159, (byte)1, (byte)35, (byte)32, (byte)64, (byte)11, (byte)124, (byte)145, (byte)113, (byte)96, (byte)200, (byte)182, (byte)103, (byte)149, (byte)76, (byte)251, (byte)77, (byte)224, (byte)226, (byte)161, (byte)131, (byte)209, (byte)14, (byte)138, (byte)139, (byte)136, (byte)137, (byte)185, (byte)109, (byte)94, (byte)124, (byte)244, (byte)168, (byte)18}, 0) ;
            p123.len = (byte)(byte)36;
            p123.target_component = (byte)(byte)56;
            p123.target_system = (byte)(byte)155;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)29);
                Debug.Assert(pack.epv == (ushort)(ushort)34739);
                Debug.Assert(pack.cog == (ushort)(ushort)12832);
                Debug.Assert(pack.dgps_age == (uint)3608696384U);
                Debug.Assert(pack.lat == (int)1540065700);
                Debug.Assert(pack.eph == (ushort)(ushort)27858);
                Debug.Assert(pack.lon == (int) -1501084063);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.dgps_numch == (byte)(byte)144);
                Debug.Assert(pack.time_usec == (ulong)5426680113966727595L);
                Debug.Assert(pack.vel == (ushort)(ushort)46944);
                Debug.Assert(pack.alt == (int)1238330546);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.cog = (ushort)(ushort)12832;
            p124.alt = (int)1238330546;
            p124.lon = (int) -1501084063;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.lat = (int)1540065700;
            p124.vel = (ushort)(ushort)46944;
            p124.dgps_age = (uint)3608696384U;
            p124.time_usec = (ulong)5426680113966727595L;
            p124.eph = (ushort)(ushort)27858;
            p124.dgps_numch = (byte)(byte)144;
            p124.epv = (ushort)(ushort)34739;
            p124.satellites_visible = (byte)(byte)29;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)31653);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
                Debug.Assert(pack.Vservo == (ushort)(ushort)14411);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)31653;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT;
            p125.Vservo = (ushort)(ushort)14411;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)65, (byte)23, (byte)5, (byte)56, (byte)246, (byte)204, (byte)177, (byte)105, (byte)109, (byte)36, (byte)4, (byte)250, (byte)212, (byte)37, (byte)162, (byte)252, (byte)228, (byte)62, (byte)63, (byte)43, (byte)29, (byte)228, (byte)228, (byte)254, (byte)243, (byte)107, (byte)144, (byte)2, (byte)180, (byte)9, (byte)46, (byte)75, (byte)55, (byte)115, (byte)74, (byte)237, (byte)212, (byte)53, (byte)198, (byte)126, (byte)243, (byte)160, (byte)132, (byte)247, (byte)167, (byte)157, (byte)134, (byte)62, (byte)29, (byte)5, (byte)240, (byte)245, (byte)162, (byte)64, (byte)240, (byte)58, (byte)20, (byte)140, (byte)227, (byte)30, (byte)176, (byte)165, (byte)110, (byte)54, (byte)152, (byte)37, (byte)77, (byte)32, (byte)156, (byte)173}));
                Debug.Assert(pack.timeout == (ushort)(ushort)45569);
                Debug.Assert(pack.baudrate == (uint)3861078765U);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
                Debug.Assert(pack.count == (byte)(byte)246);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
            p126.baudrate = (uint)3861078765U;
            p126.timeout = (ushort)(ushort)45569;
            p126.data__SET(new byte[] {(byte)65, (byte)23, (byte)5, (byte)56, (byte)246, (byte)204, (byte)177, (byte)105, (byte)109, (byte)36, (byte)4, (byte)250, (byte)212, (byte)37, (byte)162, (byte)252, (byte)228, (byte)62, (byte)63, (byte)43, (byte)29, (byte)228, (byte)228, (byte)254, (byte)243, (byte)107, (byte)144, (byte)2, (byte)180, (byte)9, (byte)46, (byte)75, (byte)55, (byte)115, (byte)74, (byte)237, (byte)212, (byte)53, (byte)198, (byte)126, (byte)243, (byte)160, (byte)132, (byte)247, (byte)167, (byte)157, (byte)134, (byte)62, (byte)29, (byte)5, (byte)240, (byte)245, (byte)162, (byte)64, (byte)240, (byte)58, (byte)20, (byte)140, (byte)227, (byte)30, (byte)176, (byte)165, (byte)110, (byte)54, (byte)152, (byte)37, (byte)77, (byte)32, (byte)156, (byte)173}, 0) ;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.count = (byte)(byte)246;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)162488752);
                Debug.Assert(pack.time_last_baseline_ms == (uint)686828144U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)252);
                Debug.Assert(pack.rtk_health == (byte)(byte)147);
                Debug.Assert(pack.accuracy == (uint)2386938578U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)124);
                Debug.Assert(pack.tow == (uint)1807153506U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1578796302);
                Debug.Assert(pack.baseline_b_mm == (int) -1250886875);
                Debug.Assert(pack.nsats == (byte)(byte)111);
                Debug.Assert(pack.baseline_c_mm == (int)193554592);
                Debug.Assert(pack.wn == (ushort)(ushort)65533);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)38);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.nsats = (byte)(byte)111;
            p127.wn = (ushort)(ushort)65533;
            p127.baseline_c_mm = (int)193554592;
            p127.baseline_b_mm = (int) -1250886875;
            p127.rtk_health = (byte)(byte)147;
            p127.accuracy = (uint)2386938578U;
            p127.tow = (uint)1807153506U;
            p127.rtk_rate = (byte)(byte)252;
            p127.rtk_receiver_id = (byte)(byte)124;
            p127.time_last_baseline_ms = (uint)686828144U;
            p127.baseline_coords_type = (byte)(byte)38;
            p127.baseline_a_mm = (int)162488752;
            p127.iar_num_hypotheses = (int) -1578796302;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)35);
                Debug.Assert(pack.rtk_health == (byte)(byte)20);
                Debug.Assert(pack.wn == (ushort)(ushort)32147);
                Debug.Assert(pack.tow == (uint)1301634876U);
                Debug.Assert(pack.accuracy == (uint)1566984443U);
                Debug.Assert(pack.baseline_a_mm == (int) -2125502204);
                Debug.Assert(pack.nsats == (byte)(byte)219);
                Debug.Assert(pack.baseline_b_mm == (int)1367155575);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)113);
                Debug.Assert(pack.rtk_rate == (byte)(byte)210);
                Debug.Assert(pack.iar_num_hypotheses == (int) -805006826);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1561121062U);
                Debug.Assert(pack.baseline_c_mm == (int) -151687770);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_c_mm = (int) -151687770;
            p128.nsats = (byte)(byte)219;
            p128.accuracy = (uint)1566984443U;
            p128.baseline_coords_type = (byte)(byte)35;
            p128.baseline_a_mm = (int) -2125502204;
            p128.tow = (uint)1301634876U;
            p128.iar_num_hypotheses = (int) -805006826;
            p128.rtk_receiver_id = (byte)(byte)113;
            p128.baseline_b_mm = (int)1367155575;
            p128.rtk_rate = (byte)(byte)210;
            p128.rtk_health = (byte)(byte)20;
            p128.time_last_baseline_ms = (uint)1561121062U;
            p128.wn = (ushort)(ushort)32147;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -8345);
                Debug.Assert(pack.zgyro == (short)(short) -14022);
                Debug.Assert(pack.zacc == (short)(short)10988);
                Debug.Assert(pack.xmag == (short)(short) -6630);
                Debug.Assert(pack.yacc == (short)(short)8899);
                Debug.Assert(pack.ygyro == (short)(short) -32046);
                Debug.Assert(pack.time_boot_ms == (uint)338317520U);
                Debug.Assert(pack.xgyro == (short)(short)26460);
                Debug.Assert(pack.zmag == (short)(short) -30445);
                Debug.Assert(pack.ymag == (short)(short)13980);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zacc = (short)(short)10988;
            p129.time_boot_ms = (uint)338317520U;
            p129.xacc = (short)(short) -8345;
            p129.zgyro = (short)(short) -14022;
            p129.ymag = (short)(short)13980;
            p129.xmag = (short)(short) -6630;
            p129.zmag = (short)(short) -30445;
            p129.ygyro = (short)(short) -32046;
            p129.yacc = (short)(short)8899;
            p129.xgyro = (short)(short)26460;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload == (byte)(byte)93);
                Debug.Assert(pack.jpg_quality == (byte)(byte)161);
                Debug.Assert(pack.packets == (ushort)(ushort)54270);
                Debug.Assert(pack.width == (ushort)(ushort)4795);
                Debug.Assert(pack.height == (ushort)(ushort)32533);
                Debug.Assert(pack.size == (uint)4183120555U);
                Debug.Assert(pack.type == (byte)(byte)255);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.height = (ushort)(ushort)32533;
            p130.width = (ushort)(ushort)4795;
            p130.size = (uint)4183120555U;
            p130.jpg_quality = (byte)(byte)161;
            p130.packets = (ushort)(ushort)54270;
            p130.payload = (byte)(byte)93;
            p130.type = (byte)(byte)255;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)9078);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)96, (byte)243, (byte)185, (byte)48, (byte)115, (byte)11, (byte)48, (byte)191, (byte)186, (byte)75, (byte)32, (byte)239, (byte)1, (byte)145, (byte)46, (byte)7, (byte)229, (byte)144, (byte)153, (byte)66, (byte)157, (byte)82, (byte)109, (byte)211, (byte)144, (byte)205, (byte)86, (byte)145, (byte)67, (byte)29, (byte)7, (byte)9, (byte)65, (byte)177, (byte)113, (byte)26, (byte)70, (byte)184, (byte)75, (byte)198, (byte)42, (byte)68, (byte)22, (byte)253, (byte)204, (byte)229, (byte)15, (byte)187, (byte)38, (byte)32, (byte)140, (byte)69, (byte)118, (byte)176, (byte)102, (byte)136, (byte)138, (byte)225, (byte)146, (byte)214, (byte)86, (byte)118, (byte)196, (byte)223, (byte)72, (byte)182, (byte)164, (byte)85, (byte)226, (byte)234, (byte)118, (byte)35, (byte)242, (byte)157, (byte)62, (byte)76, (byte)161, (byte)68, (byte)172, (byte)20, (byte)151, (byte)128, (byte)222, (byte)125, (byte)246, (byte)30, (byte)214, (byte)194, (byte)30, (byte)146, (byte)116, (byte)72, (byte)2, (byte)127, (byte)56, (byte)106, (byte)6, (byte)232, (byte)193, (byte)31, (byte)213, (byte)50, (byte)20, (byte)100, (byte)221, (byte)2, (byte)181, (byte)76, (byte)241, (byte)51, (byte)172, (byte)66, (byte)30, (byte)199, (byte)108, (byte)223, (byte)132, (byte)123, (byte)133, (byte)150, (byte)194, (byte)81, (byte)116, (byte)205, (byte)18, (byte)178, (byte)6, (byte)75, (byte)172, (byte)22, (byte)112, (byte)7, (byte)48, (byte)103, (byte)201, (byte)248, (byte)36, (byte)189, (byte)66, (byte)218, (byte)152, (byte)159, (byte)247, (byte)64, (byte)190, (byte)152, (byte)4, (byte)246, (byte)183, (byte)234, (byte)27, (byte)4, (byte)140, (byte)218, (byte)35, (byte)190, (byte)155, (byte)183, (byte)234, (byte)54, (byte)142, (byte)17, (byte)49, (byte)3, (byte)141, (byte)47, (byte)78, (byte)52, (byte)59, (byte)183, (byte)149, (byte)57, (byte)222, (byte)88, (byte)88, (byte)222, (byte)242, (byte)54, (byte)121, (byte)161, (byte)226, (byte)65, (byte)165, (byte)199, (byte)61, (byte)221, (byte)38, (byte)129, (byte)184, (byte)92, (byte)49, (byte)164, (byte)236, (byte)90, (byte)184, (byte)216, (byte)175, (byte)158, (byte)35, (byte)72, (byte)164, (byte)104, (byte)234, (byte)215, (byte)152, (byte)86, (byte)172, (byte)47, (byte)136, (byte)97, (byte)239, (byte)140, (byte)229, (byte)73, (byte)49, (byte)101, (byte)192, (byte)47, (byte)74, (byte)156, (byte)192, (byte)215, (byte)111, (byte)123, (byte)253, (byte)198, (byte)140, (byte)101, (byte)205, (byte)228, (byte)59, (byte)23, (byte)47, (byte)74, (byte)20, (byte)4, (byte)137, (byte)42, (byte)209, (byte)108, (byte)105, (byte)8, (byte)33, (byte)119, (byte)241, (byte)218, (byte)8, (byte)179, (byte)55, (byte)58, (byte)48, (byte)191, (byte)44}));
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)9078;
            p131.data__SET(new byte[] {(byte)96, (byte)243, (byte)185, (byte)48, (byte)115, (byte)11, (byte)48, (byte)191, (byte)186, (byte)75, (byte)32, (byte)239, (byte)1, (byte)145, (byte)46, (byte)7, (byte)229, (byte)144, (byte)153, (byte)66, (byte)157, (byte)82, (byte)109, (byte)211, (byte)144, (byte)205, (byte)86, (byte)145, (byte)67, (byte)29, (byte)7, (byte)9, (byte)65, (byte)177, (byte)113, (byte)26, (byte)70, (byte)184, (byte)75, (byte)198, (byte)42, (byte)68, (byte)22, (byte)253, (byte)204, (byte)229, (byte)15, (byte)187, (byte)38, (byte)32, (byte)140, (byte)69, (byte)118, (byte)176, (byte)102, (byte)136, (byte)138, (byte)225, (byte)146, (byte)214, (byte)86, (byte)118, (byte)196, (byte)223, (byte)72, (byte)182, (byte)164, (byte)85, (byte)226, (byte)234, (byte)118, (byte)35, (byte)242, (byte)157, (byte)62, (byte)76, (byte)161, (byte)68, (byte)172, (byte)20, (byte)151, (byte)128, (byte)222, (byte)125, (byte)246, (byte)30, (byte)214, (byte)194, (byte)30, (byte)146, (byte)116, (byte)72, (byte)2, (byte)127, (byte)56, (byte)106, (byte)6, (byte)232, (byte)193, (byte)31, (byte)213, (byte)50, (byte)20, (byte)100, (byte)221, (byte)2, (byte)181, (byte)76, (byte)241, (byte)51, (byte)172, (byte)66, (byte)30, (byte)199, (byte)108, (byte)223, (byte)132, (byte)123, (byte)133, (byte)150, (byte)194, (byte)81, (byte)116, (byte)205, (byte)18, (byte)178, (byte)6, (byte)75, (byte)172, (byte)22, (byte)112, (byte)7, (byte)48, (byte)103, (byte)201, (byte)248, (byte)36, (byte)189, (byte)66, (byte)218, (byte)152, (byte)159, (byte)247, (byte)64, (byte)190, (byte)152, (byte)4, (byte)246, (byte)183, (byte)234, (byte)27, (byte)4, (byte)140, (byte)218, (byte)35, (byte)190, (byte)155, (byte)183, (byte)234, (byte)54, (byte)142, (byte)17, (byte)49, (byte)3, (byte)141, (byte)47, (byte)78, (byte)52, (byte)59, (byte)183, (byte)149, (byte)57, (byte)222, (byte)88, (byte)88, (byte)222, (byte)242, (byte)54, (byte)121, (byte)161, (byte)226, (byte)65, (byte)165, (byte)199, (byte)61, (byte)221, (byte)38, (byte)129, (byte)184, (byte)92, (byte)49, (byte)164, (byte)236, (byte)90, (byte)184, (byte)216, (byte)175, (byte)158, (byte)35, (byte)72, (byte)164, (byte)104, (byte)234, (byte)215, (byte)152, (byte)86, (byte)172, (byte)47, (byte)136, (byte)97, (byte)239, (byte)140, (byte)229, (byte)73, (byte)49, (byte)101, (byte)192, (byte)47, (byte)74, (byte)156, (byte)192, (byte)215, (byte)111, (byte)123, (byte)253, (byte)198, (byte)140, (byte)101, (byte)205, (byte)228, (byte)59, (byte)23, (byte)47, (byte)74, (byte)20, (byte)4, (byte)137, (byte)42, (byte)209, (byte)108, (byte)105, (byte)8, (byte)33, (byte)119, (byte)241, (byte)218, (byte)8, (byte)179, (byte)55, (byte)58, (byte)48, (byte)191, (byte)44}, 0) ;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)64);
                Debug.Assert(pack.current_distance == (ushort)(ushort)50467);
                Debug.Assert(pack.min_distance == (ushort)(ushort)14604);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.id == (byte)(byte)114);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55718);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.time_boot_ms == (uint)3589542863U);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)14604;
            p132.max_distance = (ushort)(ushort)55718;
            p132.current_distance = (ushort)(ushort)50467;
            p132.time_boot_ms = (uint)3589542863U;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.id = (byte)(byte)114;
            p132.covariance = (byte)(byte)64;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1798460333);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)30287);
                Debug.Assert(pack.lat == (int)1171349494);
                Debug.Assert(pack.mask == (ulong)7257214985784897741L);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)30287;
            p133.lon = (int)1798460333;
            p133.lat = (int)1171349494;
            p133.mask = (ulong)7257214985784897741L;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)18478);
                Debug.Assert(pack.lat == (int)1448064313);
                Debug.Assert(pack.lon == (int) -150595025);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)8714, (short)14311, (short)22563, (short) -18755, (short)26504, (short)27545, (short) -7559, (short) -20155, (short)1132, (short)2675, (short) -29053, (short) -3207, (short)14635, (short) -3341, (short)5619, (short)9822}));
                Debug.Assert(pack.gridbit == (byte)(byte)27);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)18478;
            p134.lon = (int) -150595025;
            p134.lat = (int)1448064313;
            p134.gridbit = (byte)(byte)27;
            p134.data__SET(new short[] {(short)8714, (short)14311, (short)22563, (short) -18755, (short)26504, (short)27545, (short) -7559, (short) -20155, (short)1132, (short)2675, (short) -29053, (short) -3207, (short)14635, (short) -3341, (short)5619, (short)9822}, 0) ;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1940007346);
                Debug.Assert(pack.lat == (int)1904627842);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)1940007346;
            p135.lat = (int)1904627842;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)2006534394);
                Debug.Assert(pack.terrain_height == (float)2.9239836E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)47447);
                Debug.Assert(pack.spacing == (ushort)(ushort)54513);
                Debug.Assert(pack.loaded == (ushort)(ushort)60504);
                Debug.Assert(pack.current_height == (float)2.9268324E38F);
                Debug.Assert(pack.lat == (int)1817993066);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)2.9268324E38F;
            p136.lon = (int)2006534394;
            p136.pending = (ushort)(ushort)47447;
            p136.lat = (int)1817993066;
            p136.spacing = (ushort)(ushort)54513;
            p136.terrain_height = (float)2.9239836E38F;
            p136.loaded = (ushort)(ushort)60504;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -7.71297E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1818368709U);
                Debug.Assert(pack.press_abs == (float)2.3092963E38F);
                Debug.Assert(pack.temperature == (short)(short)27610);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1818368709U;
            p137.press_diff = (float) -7.71297E37F;
            p137.press_abs = (float)2.3092963E38F;
            p137.temperature = (short)(short)27610;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)5.542659E36F);
                Debug.Assert(pack.time_usec == (ulong)275464220512391120L);
                Debug.Assert(pack.x == (float)1.1149063E38F);
                Debug.Assert(pack.z == (float) -2.3969652E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.4448715E38F, 1.410442E38F, 8.714168E37F, 1.7308057E38F}));
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float)1.1149063E38F;
            p138.q_SET(new float[] {2.4448715E38F, 1.410442E38F, 8.714168E37F, 1.7308057E38F}, 0) ;
            p138.time_usec = (ulong)275464220512391120L;
            p138.z = (float) -2.3969652E38F;
            p138.y = (float)5.542659E36F;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.group_mlx == (byte)(byte)148);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {8.935597E37F, 4.1103036E37F, 2.6519796E38F, 1.5831921E38F, 1.7733836E38F, 1.293405E38F, -5.545366E37F, -1.0538176E38F}));
                Debug.Assert(pack.time_usec == (ulong)8388668844729578527L);
                Debug.Assert(pack.target_component == (byte)(byte)60);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)148;
            p139.target_system = (byte)(byte)85;
            p139.controls_SET(new float[] {8.935597E37F, 4.1103036E37F, 2.6519796E38F, 1.5831921E38F, 1.7733836E38F, 1.293405E38F, -5.545366E37F, -1.0538176E38F}, 0) ;
            p139.time_usec = (ulong)8388668844729578527L;
            p139.target_component = (byte)(byte)60;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5793981729272744435L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.6409332E38F, 1.7739928E38F, 5.1441216E37F, -8.1810245E37F, 2.4217154E37F, 2.3023249E38F, -2.9328527E38F, -1.7032306E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)57);
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)5793981729272744435L;
            p140.controls_SET(new float[] {2.6409332E38F, 1.7739928E38F, 5.1441216E37F, -8.1810245E37F, 2.4217154E37F, 2.3023249E38F, -2.9328527E38F, -1.7032306E38F}, 0) ;
            p140.group_mlx = (byte)(byte)57;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)644344701588570215L);
                Debug.Assert(pack.bottom_clearance == (float) -2.9810218E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.60991E38F);
                Debug.Assert(pack.altitude_local == (float)1.5983705E38F);
                Debug.Assert(pack.altitude_relative == (float)7.7896176E37F);
                Debug.Assert(pack.altitude_amsl == (float)2.8600988E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -7.5586223E37F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)644344701588570215L;
            p141.bottom_clearance = (float) -2.9810218E38F;
            p141.altitude_terrain = (float)2.60991E38F;
            p141.altitude_local = (float)1.5983705E38F;
            p141.altitude_relative = (float)7.7896176E37F;
            p141.altitude_monotonic = (float) -7.5586223E37F;
            p141.altitude_amsl = (float)2.8600988E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)3);
                Debug.Assert(pack.request_id == (byte)(byte)32);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)218, (byte)84, (byte)253, (byte)65, (byte)124, (byte)6, (byte)217, (byte)90, (byte)235, (byte)191, (byte)16, (byte)1, (byte)185, (byte)109, (byte)240, (byte)139, (byte)208, (byte)120, (byte)107, (byte)32, (byte)84, (byte)129, (byte)86, (byte)121, (byte)117, (byte)18, (byte)2, (byte)178, (byte)17, (byte)225, (byte)220, (byte)142, (byte)124, (byte)68, (byte)99, (byte)204, (byte)242, (byte)78, (byte)46, (byte)140, (byte)118, (byte)106, (byte)50, (byte)243, (byte)97, (byte)111, (byte)8, (byte)69, (byte)60, (byte)141, (byte)104, (byte)2, (byte)220, (byte)186, (byte)183, (byte)233, (byte)204, (byte)84, (byte)66, (byte)250, (byte)80, (byte)100, (byte)170, (byte)93, (byte)83, (byte)225, (byte)212, (byte)187, (byte)106, (byte)195, (byte)213, (byte)43, (byte)252, (byte)160, (byte)57, (byte)71, (byte)21, (byte)57, (byte)114, (byte)215, (byte)248, (byte)91, (byte)68, (byte)54, (byte)72, (byte)188, (byte)20, (byte)80, (byte)30, (byte)165, (byte)41, (byte)242, (byte)11, (byte)58, (byte)237, (byte)143, (byte)109, (byte)53, (byte)244, (byte)174, (byte)179, (byte)157, (byte)20, (byte)106, (byte)103, (byte)132, (byte)86, (byte)9, (byte)178, (byte)191, (byte)101, (byte)247, (byte)5, (byte)157, (byte)200, (byte)32, (byte)175, (byte)84, (byte)176, (byte)124}));
                Debug.Assert(pack.transfer_type == (byte)(byte)108);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)32, (byte)47, (byte)156, (byte)61, (byte)243, (byte)210, (byte)233, (byte)246, (byte)197, (byte)81, (byte)244, (byte)159, (byte)249, (byte)216, (byte)44, (byte)81, (byte)179, (byte)126, (byte)63, (byte)160, (byte)246, (byte)220, (byte)79, (byte)73, (byte)12, (byte)104, (byte)115, (byte)77, (byte)235, (byte)197, (byte)244, (byte)119, (byte)19, (byte)240, (byte)232, (byte)87, (byte)247, (byte)230, (byte)188, (byte)205, (byte)99, (byte)245, (byte)113, (byte)143, (byte)47, (byte)192, (byte)58, (byte)217, (byte)243, (byte)176, (byte)6, (byte)80, (byte)199, (byte)196, (byte)23, (byte)168, (byte)157, (byte)176, (byte)18, (byte)14, (byte)46, (byte)239, (byte)46, (byte)72, (byte)251, (byte)127, (byte)84, (byte)227, (byte)196, (byte)28, (byte)132, (byte)249, (byte)229, (byte)255, (byte)136, (byte)73, (byte)72, (byte)18, (byte)25, (byte)110, (byte)10, (byte)51, (byte)119, (byte)22, (byte)114, (byte)159, (byte)250, (byte)224, (byte)117, (byte)115, (byte)84, (byte)46, (byte)208, (byte)255, (byte)222, (byte)150, (byte)122, (byte)9, (byte)248, (byte)128, (byte)20, (byte)87, (byte)17, (byte)73, (byte)167, (byte)240, (byte)191, (byte)178, (byte)197, (byte)34, (byte)78, (byte)36, (byte)15, (byte)99, (byte)35, (byte)250, (byte)191, (byte)178, (byte)208, (byte)103}));
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)32;
            p142.storage_SET(new byte[] {(byte)32, (byte)47, (byte)156, (byte)61, (byte)243, (byte)210, (byte)233, (byte)246, (byte)197, (byte)81, (byte)244, (byte)159, (byte)249, (byte)216, (byte)44, (byte)81, (byte)179, (byte)126, (byte)63, (byte)160, (byte)246, (byte)220, (byte)79, (byte)73, (byte)12, (byte)104, (byte)115, (byte)77, (byte)235, (byte)197, (byte)244, (byte)119, (byte)19, (byte)240, (byte)232, (byte)87, (byte)247, (byte)230, (byte)188, (byte)205, (byte)99, (byte)245, (byte)113, (byte)143, (byte)47, (byte)192, (byte)58, (byte)217, (byte)243, (byte)176, (byte)6, (byte)80, (byte)199, (byte)196, (byte)23, (byte)168, (byte)157, (byte)176, (byte)18, (byte)14, (byte)46, (byte)239, (byte)46, (byte)72, (byte)251, (byte)127, (byte)84, (byte)227, (byte)196, (byte)28, (byte)132, (byte)249, (byte)229, (byte)255, (byte)136, (byte)73, (byte)72, (byte)18, (byte)25, (byte)110, (byte)10, (byte)51, (byte)119, (byte)22, (byte)114, (byte)159, (byte)250, (byte)224, (byte)117, (byte)115, (byte)84, (byte)46, (byte)208, (byte)255, (byte)222, (byte)150, (byte)122, (byte)9, (byte)248, (byte)128, (byte)20, (byte)87, (byte)17, (byte)73, (byte)167, (byte)240, (byte)191, (byte)178, (byte)197, (byte)34, (byte)78, (byte)36, (byte)15, (byte)99, (byte)35, (byte)250, (byte)191, (byte)178, (byte)208, (byte)103}, 0) ;
            p142.uri_type = (byte)(byte)3;
            p142.uri_SET(new byte[] {(byte)218, (byte)84, (byte)253, (byte)65, (byte)124, (byte)6, (byte)217, (byte)90, (byte)235, (byte)191, (byte)16, (byte)1, (byte)185, (byte)109, (byte)240, (byte)139, (byte)208, (byte)120, (byte)107, (byte)32, (byte)84, (byte)129, (byte)86, (byte)121, (byte)117, (byte)18, (byte)2, (byte)178, (byte)17, (byte)225, (byte)220, (byte)142, (byte)124, (byte)68, (byte)99, (byte)204, (byte)242, (byte)78, (byte)46, (byte)140, (byte)118, (byte)106, (byte)50, (byte)243, (byte)97, (byte)111, (byte)8, (byte)69, (byte)60, (byte)141, (byte)104, (byte)2, (byte)220, (byte)186, (byte)183, (byte)233, (byte)204, (byte)84, (byte)66, (byte)250, (byte)80, (byte)100, (byte)170, (byte)93, (byte)83, (byte)225, (byte)212, (byte)187, (byte)106, (byte)195, (byte)213, (byte)43, (byte)252, (byte)160, (byte)57, (byte)71, (byte)21, (byte)57, (byte)114, (byte)215, (byte)248, (byte)91, (byte)68, (byte)54, (byte)72, (byte)188, (byte)20, (byte)80, (byte)30, (byte)165, (byte)41, (byte)242, (byte)11, (byte)58, (byte)237, (byte)143, (byte)109, (byte)53, (byte)244, (byte)174, (byte)179, (byte)157, (byte)20, (byte)106, (byte)103, (byte)132, (byte)86, (byte)9, (byte)178, (byte)191, (byte)101, (byte)247, (byte)5, (byte)157, (byte)200, (byte)32, (byte)175, (byte)84, (byte)176, (byte)124}, 0) ;
            p142.transfer_type = (byte)(byte)108;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.9547992E38F);
                Debug.Assert(pack.press_abs == (float)1.7244638E38F);
                Debug.Assert(pack.temperature == (short)(short)29644);
                Debug.Assert(pack.time_boot_ms == (uint)9570901U);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float)1.9547992E38F;
            p143.time_boot_ms = (uint)9570901U;
            p143.temperature = (short)(short)29644;
            p143.press_abs = (float)1.7244638E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.5256903E38F, -3.135267E38F, 3.2561906E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {3.597958E37F, -2.971764E38F, 1.9995397E38F}));
                Debug.Assert(pack.timestamp == (ulong)2418274300696440970L);
                Debug.Assert(pack.lat == (int) -714266132);
                Debug.Assert(pack.custom_state == (ulong)7398548313035315888L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.217018E38F, -1.7418258E38F, 2.0073046E38F, -6.037644E37F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)247);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {8.175405E37F, 3.3628562E38F, -1.3094195E38F}));
                Debug.Assert(pack.lon == (int) -374637912);
                Debug.Assert(pack.alt == (float)1.1403789E38F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {9.577678E36F, -6.287665E37F, -2.0875256E37F}));
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.attitude_q_SET(new float[] {-2.217018E38F, -1.7418258E38F, 2.0073046E38F, -6.037644E37F}, 0) ;
            p144.alt = (float)1.1403789E38F;
            p144.vel_SET(new float[] {3.597958E37F, -2.971764E38F, 1.9995397E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)247;
            p144.acc_SET(new float[] {8.175405E37F, 3.3628562E38F, -1.3094195E38F}, 0) ;
            p144.timestamp = (ulong)2418274300696440970L;
            p144.lat = (int) -714266132;
            p144.position_cov_SET(new float[] {1.5256903E38F, -3.135267E38F, 3.2561906E38F}, 0) ;
            p144.rates_SET(new float[] {9.577678E36F, -6.287665E37F, -2.0875256E37F}, 0) ;
            p144.custom_state = (ulong)7398548313035315888L;
            p144.lon = (int) -374637912;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_pos == (float)1.5799548E38F);
                Debug.Assert(pack.roll_rate == (float) -1.9908211E38F);
                Debug.Assert(pack.z_acc == (float) -2.5147705E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.4928576E38F, -2.602259E38F, -2.4405584E38F}));
                Debug.Assert(pack.y_pos == (float) -2.1694798E38F);
                Debug.Assert(pack.y_acc == (float) -3.8705078E36F);
                Debug.Assert(pack.time_usec == (ulong)4124777525273404047L);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.4125389E38F, 1.1086638E37F, -1.4439066E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.9587652E38F, 2.843422E38F, -5.1405037E37F, -2.2987552E38F}));
                Debug.Assert(pack.z_vel == (float)3.3702542E37F);
                Debug.Assert(pack.airspeed == (float)1.0687195E37F);
                Debug.Assert(pack.pitch_rate == (float)1.77708E38F);
                Debug.Assert(pack.x_acc == (float)4.1675743E37F);
                Debug.Assert(pack.z_pos == (float)1.8769658E38F);
                Debug.Assert(pack.x_vel == (float)1.8697278E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.0517822E38F);
                Debug.Assert(pack.y_vel == (float)2.7507929E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_acc = (float) -3.8705078E36F;
            p146.time_usec = (ulong)4124777525273404047L;
            p146.pos_variance_SET(new float[] {-1.4928576E38F, -2.602259E38F, -2.4405584E38F}, 0) ;
            p146.q_SET(new float[] {-1.9587652E38F, 2.843422E38F, -5.1405037E37F, -2.2987552E38F}, 0) ;
            p146.z_acc = (float) -2.5147705E38F;
            p146.x_acc = (float)4.1675743E37F;
            p146.roll_rate = (float) -1.9908211E38F;
            p146.z_vel = (float)3.3702542E37F;
            p146.z_pos = (float)1.8769658E38F;
            p146.y_pos = (float) -2.1694798E38F;
            p146.x_vel = (float)1.8697278E38F;
            p146.airspeed = (float)1.0687195E37F;
            p146.x_pos = (float)1.5799548E38F;
            p146.y_vel = (float)2.7507929E38F;
            p146.pitch_rate = (float)1.77708E38F;
            p146.vel_variance_SET(new float[] {-2.4125389E38F, 1.1086638E37F, -1.4439066E38F}, 0) ;
            p146.yaw_rate = (float) -3.0517822E38F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)201);
                Debug.Assert(pack.current_consumed == (int)504473107);
                Debug.Assert(pack.temperature == (short)(short) -5389);
                Debug.Assert(pack.current_battery == (short)(short) -20097);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)8);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.energy_consumed == (int) -1476970587);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)53150, (ushort)52196, (ushort)57613, (ushort)52805, (ushort)43562, (ushort)6448, (ushort)60902, (ushort)2071, (ushort)14408, (ushort)24450}));
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.current_consumed = (int)504473107;
            p147.energy_consumed = (int) -1476970587;
            p147.temperature = (short)(short) -5389;
            p147.id = (byte)(byte)201;
            p147.current_battery = (short)(short) -20097;
            p147.voltages_SET(new ushort[] {(ushort)53150, (ushort)52196, (ushort)57613, (ushort)52805, (ushort)43562, (ushort)6448, (ushort)60902, (ushort)2071, (ushort)14408, (ushort)24450}, 0) ;
            p147.battery_remaining = (sbyte)(sbyte)8;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)3970008341382251001L);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)71, (byte)254, (byte)241, (byte)120, (byte)189, (byte)70, (byte)75, (byte)134, (byte)246, (byte)222, (byte)231, (byte)48, (byte)125, (byte)23, (byte)85, (byte)61, (byte)49, (byte)208}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)124, (byte)41, (byte)9, (byte)73, (byte)104, (byte)82, (byte)52, (byte)68}));
                Debug.Assert(pack.product_id == (ushort)(ushort)63493);
                Debug.Assert(pack.middleware_sw_version == (uint)2732021925U);
                Debug.Assert(pack.board_version == (uint)564611082U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)214, (byte)53, (byte)182, (byte)236, (byte)29, (byte)227, (byte)174, (byte)190}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)49298);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
                Debug.Assert(pack.os_sw_version == (uint)423850573U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)185, (byte)64, (byte)126, (byte)48, (byte)53, (byte)0, (byte)13, (byte)238}));
                Debug.Assert(pack.flight_sw_version == (uint)879818902U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)214, (byte)53, (byte)182, (byte)236, (byte)29, (byte)227, (byte)174, (byte)190}, 0) ;
            p148.board_version = (uint)564611082U;
            p148.uid2_SET(new byte[] {(byte)71, (byte)254, (byte)241, (byte)120, (byte)189, (byte)70, (byte)75, (byte)134, (byte)246, (byte)222, (byte)231, (byte)48, (byte)125, (byte)23, (byte)85, (byte)61, (byte)49, (byte)208}, 0, PH) ;
            p148.vendor_id = (ushort)(ushort)49298;
            p148.middleware_custom_version_SET(new byte[] {(byte)124, (byte)41, (byte)9, (byte)73, (byte)104, (byte)82, (byte)52, (byte)68}, 0) ;
            p148.uid = (ulong)3970008341382251001L;
            p148.flight_sw_version = (uint)879818902U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
            p148.product_id = (ushort)(ushort)63493;
            p148.os_sw_version = (uint)423850573U;
            p148.os_custom_version_SET(new byte[] {(byte)185, (byte)64, (byte)126, (byte)48, (byte)53, (byte)0, (byte)13, (byte)238}, 0) ;
            p148.middleware_sw_version = (uint)2732021925U;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)1.3259766E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.z_TRY(ph) == (float) -3.925218E37F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)51);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-3.0515407E38F, -1.946063E38F, -2.6502992E38F, 5.3559054E37F}));
                Debug.Assert(pack.angle_x == (float)2.7749977E37F);
                Debug.Assert(pack.time_usec == (ulong)6518460322607399550L);
                Debug.Assert(pack.size_x == (float)3.0880092E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.angle_y == (float) -4.2000707E37F);
                Debug.Assert(pack.x_TRY(ph) == (float) -3.0519017E38F);
                Debug.Assert(pack.y_TRY(ph) == (float)3.179116E38F);
                Debug.Assert(pack.size_y == (float)3.244792E38F);
                Debug.Assert(pack.target_num == (byte)(byte)235);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)6518460322607399550L;
            p149.q_SET(new float[] {-3.0515407E38F, -1.946063E38F, -2.6502992E38F, 5.3559054E37F}, 0, PH) ;
            p149.x_SET((float) -3.0519017E38F, PH) ;
            p149.y_SET((float)3.179116E38F, PH) ;
            p149.angle_x = (float)2.7749977E37F;
            p149.size_y = (float)3.244792E38F;
            p149.angle_y = (float) -4.2000707E37F;
            p149.target_num = (byte)(byte)235;
            p149.distance = (float)1.3259766E38F;
            p149.size_x = (float)3.0880092E38F;
            p149.z_SET((float) -3.925218E37F, PH) ;
            p149.position_valid_SET((byte)(byte)51, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSENSOR_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gyro_cal_y == (float)1.6162062E38F);
                Debug.Assert(pack.accel_cal_x == (float) -2.6148108E37F);
                Debug.Assert(pack.accel_cal_y == (float) -2.7094492E38F);
                Debug.Assert(pack.mag_ofs_y == (short)(short) -1246);
                Debug.Assert(pack.accel_cal_z == (float)1.5979943E38F);
                Debug.Assert(pack.raw_press == (int) -780611878);
                Debug.Assert(pack.mag_declination == (float) -9.698396E37F);
                Debug.Assert(pack.mag_ofs_x == (short)(short) -26869);
                Debug.Assert(pack.gyro_cal_z == (float)2.7802665E38F);
                Debug.Assert(pack.gyro_cal_x == (float) -1.929937E38F);
                Debug.Assert(pack.raw_temp == (int) -1951223373);
                Debug.Assert(pack.mag_ofs_z == (short)(short) -3301);
            };
            DemoDevice.SENSOR_OFFSETS p150 = LoopBackDemoChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.gyro_cal_y = (float)1.6162062E38F;
            p150.mag_ofs_z = (short)(short) -3301;
            p150.raw_press = (int) -780611878;
            p150.gyro_cal_x = (float) -1.929937E38F;
            p150.mag_declination = (float) -9.698396E37F;
            p150.mag_ofs_x = (short)(short) -26869;
            p150.mag_ofs_y = (short)(short) -1246;
            p150.raw_temp = (int) -1951223373;
            p150.gyro_cal_z = (float)2.7802665E38F;
            p150.accel_cal_y = (float) -2.7094492E38F;
            p150.accel_cal_x = (float) -2.6148108E37F;
            p150.accel_cal_z = (float)1.5979943E38F;
            LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MAG_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)49);
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.mag_ofs_z == (short)(short) -20535);
                Debug.Assert(pack.mag_ofs_y == (short)(short)9429);
                Debug.Assert(pack.mag_ofs_x == (short)(short) -29995);
            };
            DemoDevice.SET_MAG_OFFSETS p151 = LoopBackDemoChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.mag_ofs_x = (short)(short) -29995;
            p151.mag_ofs_z = (short)(short) -20535;
            p151.target_system = (byte)(byte)65;
            p151.target_component = (byte)(byte)49;
            p151.mag_ofs_y = (short)(short)9429;
            LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMINFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.freemem32_TRY(ph) == (uint)4043081911U);
                Debug.Assert(pack.brkval == (ushort)(ushort)46698);
                Debug.Assert(pack.freemem == (ushort)(ushort)26270);
            };
            DemoDevice.MEMINFO p152 = LoopBackDemoChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.freemem32_SET((uint)4043081911U, PH) ;
            p152.brkval = (ushort)(ushort)46698;
            p152.freemem = (ushort)(ushort)26270;
            LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAP_ADCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc2 == (ushort)(ushort)33431);
                Debug.Assert(pack.adc3 == (ushort)(ushort)22177);
                Debug.Assert(pack.adc5 == (ushort)(ushort)6270);
                Debug.Assert(pack.adc4 == (ushort)(ushort)32483);
                Debug.Assert(pack.adc6 == (ushort)(ushort)12180);
                Debug.Assert(pack.adc1 == (ushort)(ushort)10905);
            };
            DemoDevice.AP_ADC p153 = LoopBackDemoChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc4 = (ushort)(ushort)32483;
            p153.adc1 = (ushort)(ushort)10905;
            p153.adc2 = (ushort)(ushort)33431;
            p153.adc5 = (ushort)(ushort)6270;
            p153.adc3 = (ushort)(ushort)22177;
            p153.adc6 = (ushort)(ushort)12180;
            LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDIGICAM_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.engine_cut_off == (byte)(byte)84);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.aperture == (byte)(byte)220);
                Debug.Assert(pack.extra_param == (byte)(byte)62);
                Debug.Assert(pack.extra_value == (float)9.253651E37F);
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.mode == (byte)(byte)67);
                Debug.Assert(pack.exposure_type == (byte)(byte)79);
                Debug.Assert(pack.iso == (byte)(byte)92);
                Debug.Assert(pack.command_id == (byte)(byte)158);
                Debug.Assert(pack.shutter_speed == (ushort)(ushort)60605);
            };
            DemoDevice.DIGICAM_CONFIGURE p154 = LoopBackDemoChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.engine_cut_off = (byte)(byte)84;
            p154.command_id = (byte)(byte)158;
            p154.iso = (byte)(byte)92;
            p154.target_system = (byte)(byte)54;
            p154.exposure_type = (byte)(byte)79;
            p154.mode = (byte)(byte)67;
            p154.shutter_speed = (ushort)(ushort)60605;
            p154.extra_value = (float)9.253651E37F;
            p154.extra_param = (byte)(byte)62;
            p154.aperture = (byte)(byte)220;
            p154.target_component = (byte)(byte)216;
            LoopBackDemoChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDIGICAM_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_id == (byte)(byte)209);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.zoom_pos == (byte)(byte)24);
                Debug.Assert(pack.zoom_step == (sbyte)(sbyte) - 92);
                Debug.Assert(pack.extra_value == (float) -7.8417216E37F);
                Debug.Assert(pack.session == (byte)(byte)205);
                Debug.Assert(pack.focus_lock == (byte)(byte)115);
                Debug.Assert(pack.shot == (byte)(byte)174);
                Debug.Assert(pack.extra_param == (byte)(byte)29);
                Debug.Assert(pack.target_system == (byte)(byte)120);
            };
            DemoDevice.DIGICAM_CONTROL p155 = LoopBackDemoChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.command_id = (byte)(byte)209;
            p155.target_system = (byte)(byte)120;
            p155.zoom_pos = (byte)(byte)24;
            p155.session = (byte)(byte)205;
            p155.zoom_step = (sbyte)(sbyte) - 92;
            p155.focus_lock = (byte)(byte)115;
            p155.shot = (byte)(byte)174;
            p155.extra_param = (byte)(byte)29;
            p155.extra_value = (float) -7.8417216E37F;
            p155.target_component = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.target_system == (byte)(byte)51);
                Debug.Assert(pack.stab_pitch == (byte)(byte)39);
                Debug.Assert(pack.stab_yaw == (byte)(byte)16);
                Debug.Assert(pack.mount_mode == (MAV_MOUNT_MODE)MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT);
                Debug.Assert(pack.stab_roll == (byte)(byte)141);
            };
            DemoDevice.MOUNT_CONFIGURE p156 = LoopBackDemoChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.mount_mode = (MAV_MOUNT_MODE)MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT;
            p156.stab_pitch = (byte)(byte)39;
            p156.stab_yaw = (byte)(byte)16;
            p156.stab_roll = (byte)(byte)141;
            p156.target_system = (byte)(byte)51;
            p156.target_component = (byte)(byte)91;
            LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.input_a == (int) -1908703684);
                Debug.Assert(pack.input_b == (int)1150410394);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.input_c == (int)849350437);
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.save_position == (byte)(byte)142);
            };
            DemoDevice.MOUNT_CONTROL p157 = LoopBackDemoChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.input_a = (int) -1908703684;
            p157.input_b = (int)1150410394;
            p157.target_system = (byte)(byte)58;
            p157.input_c = (int)849350437;
            p157.save_position = (byte)(byte)142;
            p157.target_component = (byte)(byte)39;
            LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pointing_b == (int) -1886094491);
                Debug.Assert(pack.pointing_a == (int) -1888920797);
                Debug.Assert(pack.target_component == (byte)(byte)4);
                Debug.Assert(pack.pointing_c == (int)1630190757);
                Debug.Assert(pack.target_system == (byte)(byte)37);
            };
            DemoDevice.MOUNT_STATUS p158 = LoopBackDemoChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.pointing_b = (int) -1886094491;
            p158.pointing_c = (int)1630190757;
            p158.pointing_a = (int) -1888920797;
            p158.target_component = (byte)(byte)4;
            p158.target_system = (byte)(byte)37;
            LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFENCE_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)106);
                Debug.Assert(pack.lat == (float)2.80147E38F);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.lng == (float)4.8539847E37F);
                Debug.Assert(pack.idx == (byte)(byte)238);
            };
            DemoDevice.FENCE_POINT p160 = LoopBackDemoChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.idx = (byte)(byte)238;
            p160.lng = (float)4.8539847E37F;
            p160.lat = (float)2.80147E38F;
            p160.target_component = (byte)(byte)57;
            p160.count = (byte)(byte)106;
            p160.target_system = (byte)(byte)17;
            LoopBackDemoChannel.instance.send(p160);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFENCE_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.idx == (byte)(byte)24);
                Debug.Assert(pack.target_system == (byte)(byte)108);
            };
            DemoDevice.FENCE_FETCH_POINT p161 = LoopBackDemoChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.idx = (byte)(byte)24;
            p161.target_component = (byte)(byte)150;
            p161.target_system = (byte)(byte)108;
            LoopBackDemoChannel.instance.send(p161);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFENCE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.breach_status == (byte)(byte)35);
                Debug.Assert(pack.breach_time == (uint)3726881189U);
                Debug.Assert(pack.breach_type == (FENCE_BREACH)FENCE_BREACH.FENCE_BREACH_BOUNDARY);
                Debug.Assert(pack.breach_count == (ushort)(ushort)44821);
            };
            DemoDevice.FENCE_STATUS p162 = LoopBackDemoChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_status = (byte)(byte)35;
            p162.breach_count = (ushort)(ushort)44821;
            p162.breach_type = (FENCE_BREACH)FENCE_BREACH.FENCE_BREACH_BOUNDARY;
            p162.breach_time = (uint)3726881189U;
            LoopBackDemoChannel.instance.send(p162);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAHRSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.error_yaw == (float) -3.1543414E38F);
                Debug.Assert(pack.omegaIx == (float) -2.7138383E38F);
                Debug.Assert(pack.renorm_val == (float) -1.0133167E38F);
                Debug.Assert(pack.omegaIz == (float)2.8708528E38F);
                Debug.Assert(pack.error_rp == (float) -6.1875465E36F);
                Debug.Assert(pack.omegaIy == (float) -2.0909505E38F);
                Debug.Assert(pack.accel_weight == (float) -4.0610554E37F);
            };
            DemoDevice.AHRS p163 = LoopBackDemoChannel.new_AHRS();
            PH.setPack(p163);
            p163.omegaIx = (float) -2.7138383E38F;
            p163.omegaIz = (float)2.8708528E38F;
            p163.accel_weight = (float) -4.0610554E37F;
            p163.omegaIy = (float) -2.0909505E38F;
            p163.renorm_val = (float) -1.0133167E38F;
            p163.error_rp = (float) -6.1875465E36F;
            p163.error_yaw = (float) -3.1543414E38F;
            LoopBackDemoChannel.instance.send(p163);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIMSTATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float) -3.2798833E38F);
                Debug.Assert(pack.xgyro == (float) -8.079906E37F);
                Debug.Assert(pack.roll == (float)2.5252789E38F);
                Debug.Assert(pack.pitch == (float)1.7126067E38F);
                Debug.Assert(pack.ygyro == (float)1.4970426E37F);
                Debug.Assert(pack.lng == (int) -1027693006);
                Debug.Assert(pack.yaw == (float) -6.017676E37F);
                Debug.Assert(pack.xacc == (float)4.6690903E37F);
                Debug.Assert(pack.zacc == (float)2.3020926E38F);
                Debug.Assert(pack.lat == (int) -363169971);
                Debug.Assert(pack.zgyro == (float) -1.4170463E38F);
            };
            DemoDevice.SIMSTATE p164 = LoopBackDemoChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.yacc = (float) -3.2798833E38F;
            p164.ygyro = (float)1.4970426E37F;
            p164.xgyro = (float) -8.079906E37F;
            p164.lat = (int) -363169971;
            p164.lng = (int) -1027693006;
            p164.xacc = (float)4.6690903E37F;
            p164.roll = (float)2.5252789E38F;
            p164.zacc = (float)2.3020926E38F;
            p164.zgyro = (float) -1.4170463E38F;
            p164.yaw = (float) -6.017676E37F;
            p164.pitch = (float)1.7126067E38F;
            LoopBackDemoChannel.instance.send(p164);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHWSTATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.I2Cerr == (byte)(byte)183);
                Debug.Assert(pack.Vcc == (ushort)(ushort)590);
            };
            DemoDevice.HWSTATUS p165 = LoopBackDemoChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.I2Cerr = (byte)(byte)183;
            p165.Vcc = (ushort)(ushort)590;
            LoopBackDemoChannel.instance.send(p165);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)30);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)13224);
                Debug.Assert(pack.remnoise == (byte)(byte)34);
                Debug.Assert(pack.rssi == (byte)(byte)20);
                Debug.Assert(pack.txbuf == (byte)(byte)0);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)26568);
                Debug.Assert(pack.noise == (byte)(byte)131);
            };
            DemoDevice.RADIO p166 = LoopBackDemoChannel.new_RADIO();
            PH.setPack(p166);
            p166.remrssi = (byte)(byte)30;
            p166.rssi = (byte)(byte)20;
            p166.noise = (byte)(byte)131;
            p166.fixed_ = (ushort)(ushort)26568;
            p166.remnoise = (byte)(byte)34;
            p166.rxerrors = (ushort)(ushort)13224;
            p166.txbuf = (byte)(byte)0;
            LoopBackDemoChannel.instance.send(p166);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLIMITS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mods_required == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK);
                Debug.Assert(pack.last_clear == (uint)905388932U);
                Debug.Assert(pack.breach_count == (ushort)(ushort)27430);
                Debug.Assert(pack.last_trigger == (uint)3998175450U);
                Debug.Assert(pack.last_recovery == (uint)1244177731U);
                Debug.Assert(pack.mods_enabled == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK);
                Debug.Assert(pack.mods_triggered == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GEOFENCE);
                Debug.Assert(pack.limits_state == (LIMITS_STATE)LIMITS_STATE.LIMITS_RECOVERING);
                Debug.Assert(pack.last_action == (uint)1153427469U);
            };
            DemoDevice.LIMITS_STATUS p167 = LoopBackDemoChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.breach_count = (ushort)(ushort)27430;
            p167.mods_enabled = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.last_trigger = (uint)3998175450U;
            p167.mods_triggered = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GEOFENCE;
            p167.last_action = (uint)1153427469U;
            p167.last_recovery = (uint)1244177731U;
            p167.last_clear = (uint)905388932U;
            p167.mods_required = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.limits_state = (LIMITS_STATE)LIMITS_STATE.LIMITS_RECOVERING;
            LoopBackDemoChannel.instance.send(p167);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWINDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed == (float) -3.3332065E38F);
                Debug.Assert(pack.speed_z == (float)3.3494915E38F);
                Debug.Assert(pack.direction == (float) -3.530747E37F);
            };
            DemoDevice.WIND p168 = LoopBackDemoChannel.new_WIND();
            PH.setPack(p168);
            p168.direction = (float) -3.530747E37F;
            p168.speed = (float) -3.3332065E38F;
            p168.speed_z = (float)3.3494915E38F;
            LoopBackDemoChannel.instance.send(p168);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)150, (byte)226, (byte)110, (byte)41, (byte)235, (byte)0, (byte)248, (byte)153, (byte)52, (byte)99, (byte)53, (byte)145, (byte)37, (byte)126, (byte)224, (byte)36}));
                Debug.Assert(pack.type == (byte)(byte)251);
                Debug.Assert(pack.len == (byte)(byte)142);
            };
            DemoDevice.DATA16 p169 = LoopBackDemoChannel.new_DATA16();
            PH.setPack(p169);
            p169.len = (byte)(byte)142;
            p169.data__SET(new byte[] {(byte)150, (byte)226, (byte)110, (byte)41, (byte)235, (byte)0, (byte)248, (byte)153, (byte)52, (byte)99, (byte)53, (byte)145, (byte)37, (byte)126, (byte)224, (byte)36}, 0) ;
            p169.type = (byte)(byte)251;
            LoopBackDemoChannel.instance.send(p169);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA32Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)203, (byte)228, (byte)221, (byte)151, (byte)39, (byte)241, (byte)152, (byte)217, (byte)195, (byte)150, (byte)48, (byte)193, (byte)85, (byte)119, (byte)30, (byte)56, (byte)206, (byte)244, (byte)87, (byte)243, (byte)20, (byte)98, (byte)35, (byte)154, (byte)85, (byte)195, (byte)179, (byte)227, (byte)34, (byte)244, (byte)87, (byte)157}));
                Debug.Assert(pack.type == (byte)(byte)130);
                Debug.Assert(pack.len == (byte)(byte)73);
            };
            DemoDevice.DATA32 p170 = LoopBackDemoChannel.new_DATA32();
            PH.setPack(p170);
            p170.type = (byte)(byte)130;
            p170.data__SET(new byte[] {(byte)203, (byte)228, (byte)221, (byte)151, (byte)39, (byte)241, (byte)152, (byte)217, (byte)195, (byte)150, (byte)48, (byte)193, (byte)85, (byte)119, (byte)30, (byte)56, (byte)206, (byte)244, (byte)87, (byte)243, (byte)20, (byte)98, (byte)35, (byte)154, (byte)85, (byte)195, (byte)179, (byte)227, (byte)34, (byte)244, (byte)87, (byte)157}, 0) ;
            p170.len = (byte)(byte)73;
            LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA64Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)71, (byte)57, (byte)183, (byte)180, (byte)107, (byte)88, (byte)152, (byte)26, (byte)163, (byte)2, (byte)82, (byte)248, (byte)68, (byte)225, (byte)140, (byte)151, (byte)218, (byte)238, (byte)118, (byte)41, (byte)136, (byte)76, (byte)98, (byte)158, (byte)43, (byte)212, (byte)99, (byte)24, (byte)63, (byte)244, (byte)134, (byte)185, (byte)113, (byte)215, (byte)174, (byte)221, (byte)158, (byte)183, (byte)245, (byte)32, (byte)59, (byte)115, (byte)125, (byte)40, (byte)162, (byte)80, (byte)75, (byte)74, (byte)210, (byte)80, (byte)20, (byte)124, (byte)253, (byte)235, (byte)104, (byte)73, (byte)53, (byte)119, (byte)5, (byte)112, (byte)147, (byte)140, (byte)34, (byte)214}));
                Debug.Assert(pack.len == (byte)(byte)218);
                Debug.Assert(pack.type == (byte)(byte)223);
            };
            DemoDevice.DATA64 p171 = LoopBackDemoChannel.new_DATA64();
            PH.setPack(p171);
            p171.len = (byte)(byte)218;
            p171.data__SET(new byte[] {(byte)71, (byte)57, (byte)183, (byte)180, (byte)107, (byte)88, (byte)152, (byte)26, (byte)163, (byte)2, (byte)82, (byte)248, (byte)68, (byte)225, (byte)140, (byte)151, (byte)218, (byte)238, (byte)118, (byte)41, (byte)136, (byte)76, (byte)98, (byte)158, (byte)43, (byte)212, (byte)99, (byte)24, (byte)63, (byte)244, (byte)134, (byte)185, (byte)113, (byte)215, (byte)174, (byte)221, (byte)158, (byte)183, (byte)245, (byte)32, (byte)59, (byte)115, (byte)125, (byte)40, (byte)162, (byte)80, (byte)75, (byte)74, (byte)210, (byte)80, (byte)20, (byte)124, (byte)253, (byte)235, (byte)104, (byte)73, (byte)53, (byte)119, (byte)5, (byte)112, (byte)147, (byte)140, (byte)34, (byte)214}, 0) ;
            p171.type = (byte)(byte)223;
            LoopBackDemoChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA96Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)94);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)0, (byte)136, (byte)36, (byte)4, (byte)78, (byte)37, (byte)239, (byte)247, (byte)65, (byte)37, (byte)21, (byte)181, (byte)159, (byte)100, (byte)161, (byte)116, (byte)5, (byte)220, (byte)69, (byte)138, (byte)208, (byte)54, (byte)28, (byte)219, (byte)4, (byte)108, (byte)94, (byte)10, (byte)82, (byte)154, (byte)206, (byte)10, (byte)153, (byte)46, (byte)123, (byte)108, (byte)179, (byte)149, (byte)8, (byte)66, (byte)242, (byte)166, (byte)213, (byte)31, (byte)106, (byte)236, (byte)241, (byte)189, (byte)87, (byte)23, (byte)209, (byte)131, (byte)42, (byte)151, (byte)88, (byte)109, (byte)116, (byte)190, (byte)195, (byte)213, (byte)165, (byte)169, (byte)40, (byte)117, (byte)7, (byte)195, (byte)156, (byte)164, (byte)220, (byte)1, (byte)81, (byte)255, (byte)105, (byte)212, (byte)150, (byte)46, (byte)25, (byte)52, (byte)112, (byte)149, (byte)11, (byte)35, (byte)252, (byte)96, (byte)45, (byte)206, (byte)46, (byte)173, (byte)92, (byte)28, (byte)110, (byte)220, (byte)195, (byte)235, (byte)149, (byte)181}));
                Debug.Assert(pack.type == (byte)(byte)163);
            };
            DemoDevice.DATA96 p172 = LoopBackDemoChannel.new_DATA96();
            PH.setPack(p172);
            p172.data__SET(new byte[] {(byte)0, (byte)136, (byte)36, (byte)4, (byte)78, (byte)37, (byte)239, (byte)247, (byte)65, (byte)37, (byte)21, (byte)181, (byte)159, (byte)100, (byte)161, (byte)116, (byte)5, (byte)220, (byte)69, (byte)138, (byte)208, (byte)54, (byte)28, (byte)219, (byte)4, (byte)108, (byte)94, (byte)10, (byte)82, (byte)154, (byte)206, (byte)10, (byte)153, (byte)46, (byte)123, (byte)108, (byte)179, (byte)149, (byte)8, (byte)66, (byte)242, (byte)166, (byte)213, (byte)31, (byte)106, (byte)236, (byte)241, (byte)189, (byte)87, (byte)23, (byte)209, (byte)131, (byte)42, (byte)151, (byte)88, (byte)109, (byte)116, (byte)190, (byte)195, (byte)213, (byte)165, (byte)169, (byte)40, (byte)117, (byte)7, (byte)195, (byte)156, (byte)164, (byte)220, (byte)1, (byte)81, (byte)255, (byte)105, (byte)212, (byte)150, (byte)46, (byte)25, (byte)52, (byte)112, (byte)149, (byte)11, (byte)35, (byte)252, (byte)96, (byte)45, (byte)206, (byte)46, (byte)173, (byte)92, (byte)28, (byte)110, (byte)220, (byte)195, (byte)235, (byte)149, (byte)181}, 0) ;
            p172.type = (byte)(byte)163;
            p172.len = (byte)(byte)94;
            LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRANGEFINDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)2.231794E38F);
                Debug.Assert(pack.voltage == (float) -2.508753E38F);
            };
            DemoDevice.RANGEFINDER p173 = LoopBackDemoChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.distance = (float)2.231794E38F;
            p173.voltage = (float) -2.508753E38F;
            LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAIRSPEED_AUTOCALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state_x == (float) -2.3573253E38F);
                Debug.Assert(pack.EAS2TAS == (float)2.0437396E38F);
                Debug.Assert(pack.state_y == (float) -6.175027E37F);
                Debug.Assert(pack.vz == (float) -1.5362432E38F);
                Debug.Assert(pack.Pcz == (float)1.763946E38F);
                Debug.Assert(pack.state_z == (float)1.6785141E38F);
                Debug.Assert(pack.diff_pressure == (float)2.3954071E38F);
                Debug.Assert(pack.vx == (float) -3.173986E38F);
                Debug.Assert(pack.vy == (float) -2.0188898E38F);
                Debug.Assert(pack.Pby == (float) -3.2923906E38F);
                Debug.Assert(pack.ratio == (float)4.6823646E37F);
                Debug.Assert(pack.Pax == (float) -4.8797703E36F);
            };
            DemoDevice.AIRSPEED_AUTOCAL p174 = LoopBackDemoChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.diff_pressure = (float)2.3954071E38F;
            p174.state_x = (float) -2.3573253E38F;
            p174.Pcz = (float)1.763946E38F;
            p174.Pby = (float) -3.2923906E38F;
            p174.state_z = (float)1.6785141E38F;
            p174.ratio = (float)4.6823646E37F;
            p174.vy = (float) -2.0188898E38F;
            p174.vx = (float) -3.173986E38F;
            p174.vz = (float) -1.5362432E38F;
            p174.EAS2TAS = (float)2.0437396E38F;
            p174.state_y = (float) -6.175027E37F;
            p174.Pax = (float) -4.8797703E36F;
            LoopBackDemoChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRALLY_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.land_dir == (ushort)(ushort)31209);
                Debug.Assert(pack.count == (byte)(byte)197);
                Debug.Assert(pack.idx == (byte)(byte)242);
                Debug.Assert(pack.lat == (int) -2014596683);
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.lng == (int)317271813);
                Debug.Assert(pack.break_alt == (short)(short) -7004);
                Debug.Assert(pack.flags == (RALLY_FLAGS)RALLY_FLAGS.FAVORABLE_WIND);
                Debug.Assert(pack.alt == (short)(short)19569);
                Debug.Assert(pack.target_component == (byte)(byte)163);
            };
            DemoDevice.RALLY_POINT p175 = LoopBackDemoChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.target_component = (byte)(byte)163;
            p175.alt = (short)(short)19569;
            p175.lng = (int)317271813;
            p175.break_alt = (short)(short) -7004;
            p175.flags = (RALLY_FLAGS)RALLY_FLAGS.FAVORABLE_WIND;
            p175.target_system = (byte)(byte)88;
            p175.idx = (byte)(byte)242;
            p175.count = (byte)(byte)197;
            p175.lat = (int) -2014596683;
            p175.land_dir = (ushort)(ushort)31209;
            LoopBackDemoChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRALLY_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)166);
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.idx == (byte)(byte)250);
            };
            DemoDevice.RALLY_FETCH_POINT p176 = LoopBackDemoChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.target_component = (byte)(byte)166;
            p176.target_system = (byte)(byte)192;
            p176.idx = (byte)(byte)250;
            LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMPASSMOT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.CompensationZ == (float) -4.618094E37F);
                Debug.Assert(pack.CompensationY == (float)5.2507934E37F);
                Debug.Assert(pack.current == (float)2.5970394E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)28042);
                Debug.Assert(pack.CompensationX == (float) -4.3742596E37F);
                Debug.Assert(pack.interference == (ushort)(ushort)1447);
            };
            DemoDevice.COMPASSMOT_STATUS p177 = LoopBackDemoChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.throttle = (ushort)(ushort)28042;
            p177.interference = (ushort)(ushort)1447;
            p177.CompensationX = (float) -4.3742596E37F;
            p177.CompensationZ = (float) -4.618094E37F;
            p177.current = (float)2.5970394E38F;
            p177.CompensationY = (float)5.2507934E37F;
            LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAHRS2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (float) -1.911922E38F);
                Debug.Assert(pack.yaw == (float)1.4143557E38F);
                Debug.Assert(pack.lng == (int)1779712660);
                Debug.Assert(pack.pitch == (float) -2.1377065E38F);
                Debug.Assert(pack.roll == (float) -2.1992691E38F);
                Debug.Assert(pack.lat == (int)601472508);
            };
            DemoDevice.AHRS2 p178 = LoopBackDemoChannel.new_AHRS2();
            PH.setPack(p178);
            p178.pitch = (float) -2.1377065E38F;
            p178.roll = (float) -2.1992691E38F;
            p178.altitude = (float) -1.911922E38F;
            p178.lat = (int)601472508;
            p178.lng = (int)1779712660;
            p178.yaw = (float)1.4143557E38F;
            LoopBackDemoChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.img_idx == (ushort)(ushort)7899);
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.p2 == (float) -2.6953517E37F);
                Debug.Assert(pack.time_usec == (ulong)1913268671211812288L);
                Debug.Assert(pack.p1 == (float)2.0642662E38F);
                Debug.Assert(pack.p4 == (float) -1.0958603E38F);
                Debug.Assert(pack.p3 == (float)1.3791913E38F);
                Debug.Assert(pack.cam_idx == (byte)(byte)234);
                Debug.Assert(pack.event_id == (CAMERA_STATUS_TYPES)CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_TRIGGER);
            };
            DemoDevice.CAMERA_STATUS p179 = LoopBackDemoChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.p2 = (float) -2.6953517E37F;
            p179.p4 = (float) -1.0958603E38F;
            p179.time_usec = (ulong)1913268671211812288L;
            p179.img_idx = (ushort)(ushort)7899;
            p179.cam_idx = (byte)(byte)234;
            p179.p3 = (float)1.3791913E38F;
            p179.p1 = (float)2.0642662E38F;
            p179.target_system = (byte)(byte)166;
            p179.event_id = (CAMERA_STATUS_TYPES)CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_TRIGGER;
            LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_FEEDBACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.img_idx == (ushort)(ushort)45942);
                Debug.Assert(pack.yaw == (float)2.3739554E38F);
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.pitch == (float)2.647226E38F);
                Debug.Assert(pack.lng == (int) -561559295);
                Debug.Assert(pack.roll == (float)2.0608293E38F);
                Debug.Assert(pack.foc_len == (float)2.020487E38F);
                Debug.Assert(pack.cam_idx == (byte)(byte)131);
                Debug.Assert(pack.alt_rel == (float) -9.21936E37F);
                Debug.Assert(pack.lat == (int)1873452540);
                Debug.Assert(pack.alt_msl == (float)2.1761093E38F);
                Debug.Assert(pack.time_usec == (ulong)1022761473749203595L);
                Debug.Assert(pack.flags == (CAMERA_FEEDBACK_FLAGS)CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_VIDEO);
            };
            DemoDevice.CAMERA_FEEDBACK p180 = LoopBackDemoChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.yaw = (float)2.3739554E38F;
            p180.alt_rel = (float) -9.21936E37F;
            p180.cam_idx = (byte)(byte)131;
            p180.flags = (CAMERA_FEEDBACK_FLAGS)CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_VIDEO;
            p180.roll = (float)2.0608293E38F;
            p180.pitch = (float)2.647226E38F;
            p180.alt_msl = (float)2.1761093E38F;
            p180.lat = (int)1873452540;
            p180.time_usec = (ulong)1022761473749203595L;
            p180.lng = (int) -561559295;
            p180.target_system = (byte)(byte)182;
            p180.foc_len = (float)2.020487E38F;
            p180.img_idx = (ushort)(ushort)45942;
            LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -1151);
                Debug.Assert(pack.voltage == (ushort)(ushort)23436);
            };
            DemoDevice.BATTERY2 p181 = LoopBackDemoChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.current_battery = (short)(short) -1151;
            p181.voltage = (ushort)(ushort)23436;
            LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAHRS3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.3106031E38F);
                Debug.Assert(pack.altitude == (float) -1.5403474E38F);
                Debug.Assert(pack.lat == (int)544412237);
                Debug.Assert(pack.v1 == (float) -5.646632E37F);
                Debug.Assert(pack.v3 == (float) -1.5160437E38F);
                Debug.Assert(pack.yaw == (float) -6.8181703E37F);
                Debug.Assert(pack.v4 == (float) -2.7244316E38F);
                Debug.Assert(pack.lng == (int)596488045);
                Debug.Assert(pack.v2 == (float) -3.2800192E38F);
                Debug.Assert(pack.pitch == (float) -1.0278814E38F);
            };
            DemoDevice.AHRS3 p182 = LoopBackDemoChannel.new_AHRS3();
            PH.setPack(p182);
            p182.altitude = (float) -1.5403474E38F;
            p182.pitch = (float) -1.0278814E38F;
            p182.lng = (int)596488045;
            p182.v2 = (float) -3.2800192E38F;
            p182.v4 = (float) -2.7244316E38F;
            p182.v1 = (float) -5.646632E37F;
            p182.roll = (float) -1.3106031E38F;
            p182.yaw = (float) -6.8181703E37F;
            p182.v3 = (float) -1.5160437E38F;
            p182.lat = (int)544412237;
            LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)170);
            };
            DemoDevice.AUTOPILOT_VERSION_REQUEST p183 = LoopBackDemoChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)118;
            p183.target_component = (byte)(byte)170;
            LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREMOTE_LOG_DATA_BLOCKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)42);
                Debug.Assert(pack.seqno == (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START);
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)28, (byte)222, (byte)70, (byte)141, (byte)78, (byte)171, (byte)155, (byte)233, (byte)174, (byte)135, (byte)115, (byte)191, (byte)55, (byte)41, (byte)139, (byte)162, (byte)134, (byte)34, (byte)232, (byte)7, (byte)67, (byte)4, (byte)124, (byte)236, (byte)169, (byte)45, (byte)184, (byte)193, (byte)239, (byte)240, (byte)210, (byte)233, (byte)230, (byte)229, (byte)205, (byte)11, (byte)69, (byte)95, (byte)15, (byte)48, (byte)211, (byte)111, (byte)5, (byte)236, (byte)163, (byte)38, (byte)205, (byte)85, (byte)113, (byte)179, (byte)158, (byte)178, (byte)243, (byte)63, (byte)227, (byte)43, (byte)108, (byte)154, (byte)193, (byte)106, (byte)34, (byte)219, (byte)155, (byte)66, (byte)139, (byte)250, (byte)143, (byte)53, (byte)135, (byte)102, (byte)86, (byte)84, (byte)239, (byte)228, (byte)54, (byte)200, (byte)184, (byte)204, (byte)29, (byte)51, (byte)117, (byte)243, (byte)65, (byte)50, (byte)0, (byte)99, (byte)131, (byte)111, (byte)160, (byte)11, (byte)202, (byte)93, (byte)143, (byte)21, (byte)16, (byte)85, (byte)231, (byte)192, (byte)121, (byte)137, (byte)215, (byte)88, (byte)223, (byte)18, (byte)250, (byte)28, (byte)212, (byte)44, (byte)208, (byte)58, (byte)134, (byte)83, (byte)124, (byte)170, (byte)126, (byte)133, (byte)201, (byte)162, (byte)71, (byte)153, (byte)103, (byte)114, (byte)220, (byte)71, (byte)97, (byte)21, (byte)241, (byte)173, (byte)75, (byte)193, (byte)32, (byte)74, (byte)75, (byte)168, (byte)138, (byte)1, (byte)31, (byte)234, (byte)72, (byte)108, (byte)133, (byte)127, (byte)8, (byte)91, (byte)216, (byte)194, (byte)207, (byte)192, (byte)251, (byte)160, (byte)0, (byte)110, (byte)77, (byte)112, (byte)44, (byte)95, (byte)150, (byte)206, (byte)154, (byte)124, (byte)206, (byte)164, (byte)41, (byte)168, (byte)189, (byte)31, (byte)246, (byte)163, (byte)109, (byte)145, (byte)24, (byte)177, (byte)80, (byte)168, (byte)37, (byte)161, (byte)161, (byte)152, (byte)63, (byte)67, (byte)41, (byte)227, (byte)22, (byte)21, (byte)191, (byte)222, (byte)170, (byte)92, (byte)222, (byte)234, (byte)8, (byte)9, (byte)216, (byte)88, (byte)221, (byte)216, (byte)146, (byte)204, (byte)88, (byte)84}));
            };
            DemoDevice.REMOTE_LOG_DATA_BLOCK p184 = LoopBackDemoChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.seqno = (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START;
            p184.target_system = (byte)(byte)143;
            p184.target_component = (byte)(byte)42;
            p184.data__SET(new byte[] {(byte)28, (byte)222, (byte)70, (byte)141, (byte)78, (byte)171, (byte)155, (byte)233, (byte)174, (byte)135, (byte)115, (byte)191, (byte)55, (byte)41, (byte)139, (byte)162, (byte)134, (byte)34, (byte)232, (byte)7, (byte)67, (byte)4, (byte)124, (byte)236, (byte)169, (byte)45, (byte)184, (byte)193, (byte)239, (byte)240, (byte)210, (byte)233, (byte)230, (byte)229, (byte)205, (byte)11, (byte)69, (byte)95, (byte)15, (byte)48, (byte)211, (byte)111, (byte)5, (byte)236, (byte)163, (byte)38, (byte)205, (byte)85, (byte)113, (byte)179, (byte)158, (byte)178, (byte)243, (byte)63, (byte)227, (byte)43, (byte)108, (byte)154, (byte)193, (byte)106, (byte)34, (byte)219, (byte)155, (byte)66, (byte)139, (byte)250, (byte)143, (byte)53, (byte)135, (byte)102, (byte)86, (byte)84, (byte)239, (byte)228, (byte)54, (byte)200, (byte)184, (byte)204, (byte)29, (byte)51, (byte)117, (byte)243, (byte)65, (byte)50, (byte)0, (byte)99, (byte)131, (byte)111, (byte)160, (byte)11, (byte)202, (byte)93, (byte)143, (byte)21, (byte)16, (byte)85, (byte)231, (byte)192, (byte)121, (byte)137, (byte)215, (byte)88, (byte)223, (byte)18, (byte)250, (byte)28, (byte)212, (byte)44, (byte)208, (byte)58, (byte)134, (byte)83, (byte)124, (byte)170, (byte)126, (byte)133, (byte)201, (byte)162, (byte)71, (byte)153, (byte)103, (byte)114, (byte)220, (byte)71, (byte)97, (byte)21, (byte)241, (byte)173, (byte)75, (byte)193, (byte)32, (byte)74, (byte)75, (byte)168, (byte)138, (byte)1, (byte)31, (byte)234, (byte)72, (byte)108, (byte)133, (byte)127, (byte)8, (byte)91, (byte)216, (byte)194, (byte)207, (byte)192, (byte)251, (byte)160, (byte)0, (byte)110, (byte)77, (byte)112, (byte)44, (byte)95, (byte)150, (byte)206, (byte)154, (byte)124, (byte)206, (byte)164, (byte)41, (byte)168, (byte)189, (byte)31, (byte)246, (byte)163, (byte)109, (byte)145, (byte)24, (byte)177, (byte)80, (byte)168, (byte)37, (byte)161, (byte)161, (byte)152, (byte)63, (byte)67, (byte)41, (byte)227, (byte)22, (byte)21, (byte)191, (byte)222, (byte)170, (byte)92, (byte)222, (byte)234, (byte)8, (byte)9, (byte)216, (byte)88, (byte)221, (byte)216, (byte)146, (byte)204, (byte)88, (byte)84}, 0) ;
            LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREMOTE_LOG_BLOCK_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.seqno == (uint)3452398250U);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.status == (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            };
            DemoDevice.REMOTE_LOG_BLOCK_STATUS p185 = LoopBackDemoChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.target_system = (byte)(byte)247;
            p185.status = (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK;
            p185.target_component = (byte)(byte)195;
            p185.seqno = (uint)3452398250U;
            LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLED_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.custom_len == (byte)(byte)76);
                Debug.Assert(pack.instance == (byte)(byte)124);
                Debug.Assert(pack.pattern == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.custom_bytes.SequenceEqual(new byte[] {(byte)98, (byte)86, (byte)2, (byte)185, (byte)204, (byte)248, (byte)49, (byte)29, (byte)192, (byte)229, (byte)9, (byte)140, (byte)87, (byte)184, (byte)106, (byte)151, (byte)252, (byte)147, (byte)149, (byte)108, (byte)234, (byte)39, (byte)85, (byte)93}));
            };
            DemoDevice.LED_CONTROL p186 = LoopBackDemoChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.target_component = (byte)(byte)87;
            p186.custom_bytes_SET(new byte[] {(byte)98, (byte)86, (byte)2, (byte)185, (byte)204, (byte)248, (byte)49, (byte)29, (byte)192, (byte)229, (byte)9, (byte)140, (byte)87, (byte)184, (byte)106, (byte)151, (byte)252, (byte)147, (byte)149, (byte)108, (byte)234, (byte)39, (byte)85, (byte)93}, 0) ;
            p186.target_system = (byte)(byte)8;
            p186.custom_len = (byte)(byte)76;
            p186.instance = (byte)(byte)124;
            p186.pattern = (byte)(byte)149;
            LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMAG_CAL_PROGRESSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.completion_pct == (byte)(byte)86);
                Debug.Assert(pack.completion_mask.SequenceEqual(new byte[] {(byte)24, (byte)132, (byte)185, (byte)73, (byte)237, (byte)98, (byte)135, (byte)81, (byte)39, (byte)84}));
                Debug.Assert(pack.direction_y == (float) -1.5015775E38F);
                Debug.Assert(pack.cal_mask == (byte)(byte)207);
                Debug.Assert(pack.direction_x == (float) -3.1367565E37F);
                Debug.Assert(pack.compass_id == (byte)(byte)60);
                Debug.Assert(pack.attempt == (byte)(byte)171);
                Debug.Assert(pack.cal_status == (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
                Debug.Assert(pack.direction_z == (float) -2.4033964E38F);
            };
            DemoDevice.MAG_CAL_PROGRESS p191 = LoopBackDemoChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.completion_mask_SET(new byte[] {(byte)24, (byte)132, (byte)185, (byte)73, (byte)237, (byte)98, (byte)135, (byte)81, (byte)39, (byte)84}, 0) ;
            p191.direction_z = (float) -2.4033964E38F;
            p191.completion_pct = (byte)(byte)86;
            p191.compass_id = (byte)(byte)60;
            p191.attempt = (byte)(byte)171;
            p191.direction_y = (float) -1.5015775E38F;
            p191.direction_x = (float) -3.1367565E37F;
            p191.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            p191.cal_mask = (byte)(byte)207;
            LoopBackDemoChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMAG_CAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs_z == (float) -1.3395257E38F);
                Debug.Assert(pack.diag_z == (float) -3.1062926E38F);
                Debug.Assert(pack.diag_x == (float) -8.907443E37F);
                Debug.Assert(pack.compass_id == (byte)(byte)88);
                Debug.Assert(pack.cal_mask == (byte)(byte)107);
                Debug.Assert(pack.offdiag_z == (float)1.6808993E38F);
                Debug.Assert(pack.cal_status == (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
                Debug.Assert(pack.ofs_y == (float)1.458261E38F);
                Debug.Assert(pack.offdiag_y == (float)1.1268635E38F);
                Debug.Assert(pack.fitness == (float)5.451206E37F);
                Debug.Assert(pack.ofs_x == (float)1.3367626E38F);
                Debug.Assert(pack.autosaved == (byte)(byte)250);
                Debug.Assert(pack.diag_y == (float) -1.1230312E38F);
                Debug.Assert(pack.offdiag_x == (float) -1.2396756E37F);
            };
            DemoDevice.MAG_CAL_REPORT p192 = LoopBackDemoChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.cal_mask = (byte)(byte)107;
            p192.offdiag_z = (float)1.6808993E38F;
            p192.diag_y = (float) -1.1230312E38F;
            p192.fitness = (float)5.451206E37F;
            p192.offdiag_x = (float) -1.2396756E37F;
            p192.offdiag_y = (float)1.1268635E38F;
            p192.compass_id = (byte)(byte)88;
            p192.diag_z = (float) -3.1062926E38F;
            p192.ofs_y = (float)1.458261E38F;
            p192.ofs_x = (float)1.3367626E38F;
            p192.ofs_z = (float) -1.3395257E38F;
            p192.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            p192.diag_x = (float) -8.907443E37F;
            p192.autosaved = (byte)(byte)250;
            LoopBackDemoChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEKF_STATUS_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.velocity_variance == (float)1.1105842E38F);
                Debug.Assert(pack.pos_horiz_variance == (float) -3.050985E38F);
                Debug.Assert(pack.flags == (EKF_STATUS_FLAGS)EKF_STATUS_FLAGS.EKF_POS_VERT_ABS);
                Debug.Assert(pack.terrain_alt_variance == (float) -1.920678E38F);
                Debug.Assert(pack.pos_vert_variance == (float) -8.63605E36F);
                Debug.Assert(pack.compass_variance == (float)1.6228015E38F);
            };
            DemoDevice.EKF_STATUS_REPORT p193 = LoopBackDemoChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.pos_horiz_variance = (float) -3.050985E38F;
            p193.compass_variance = (float)1.6228015E38F;
            p193.terrain_alt_variance = (float) -1.920678E38F;
            p193.pos_vert_variance = (float) -8.63605E36F;
            p193.flags = (EKF_STATUS_FLAGS)EKF_STATUS_FLAGS.EKF_POS_VERT_ABS;
            p193.velocity_variance = (float)1.1105842E38F;
            LoopBackDemoChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPID_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.FF == (float)2.2633658E38F);
                Debug.Assert(pack.D == (float)9.529253E37F);
                Debug.Assert(pack.P == (float)7.0334275E37F);
                Debug.Assert(pack.desired == (float) -8.4020125E37F);
                Debug.Assert(pack.I == (float)2.7487354E38F);
                Debug.Assert(pack.achieved == (float)2.4488703E38F);
                Debug.Assert(pack.axis == (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_LANDING);
            };
            DemoDevice.PID_TUNING p194 = LoopBackDemoChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.P = (float)7.0334275E37F;
            p194.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_LANDING;
            p194.I = (float)2.7487354E38F;
            p194.FF = (float)2.2633658E38F;
            p194.desired = (float) -8.4020125E37F;
            p194.achieved = (float)2.4488703E38F;
            p194.D = (float)9.529253E37F;
            LoopBackDemoChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGIMBAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.delta_angle_y == (float)2.4255754E38F);
                Debug.Assert(pack.delta_time == (float)2.7995187E38F);
                Debug.Assert(pack.delta_velocity_y == (float) -5.592318E37F);
                Debug.Assert(pack.joint_el == (float) -2.57316E38F);
                Debug.Assert(pack.delta_angle_z == (float) -1.0876814E38F);
                Debug.Assert(pack.delta_velocity_x == (float)1.3881511E38F);
                Debug.Assert(pack.target_component == (byte)(byte)130);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.delta_velocity_z == (float)2.408252E38F);
                Debug.Assert(pack.joint_az == (float) -3.3907374E38F);
                Debug.Assert(pack.delta_angle_x == (float) -2.9350115E38F);
                Debug.Assert(pack.joint_roll == (float) -2.1162248E38F);
            };
            DemoDevice.GIMBAL_REPORT p200 = LoopBackDemoChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.delta_angle_x = (float) -2.9350115E38F;
            p200.delta_velocity_y = (float) -5.592318E37F;
            p200.delta_velocity_z = (float)2.408252E38F;
            p200.joint_el = (float) -2.57316E38F;
            p200.joint_roll = (float) -2.1162248E38F;
            p200.delta_time = (float)2.7995187E38F;
            p200.delta_angle_z = (float) -1.0876814E38F;
            p200.delta_velocity_x = (float)1.3881511E38F;
            p200.delta_angle_y = (float)2.4255754E38F;
            p200.target_system = (byte)(byte)230;
            p200.joint_az = (float) -3.3907374E38F;
            p200.target_component = (byte)(byte)130;
            LoopBackDemoChannel.instance.send(p200);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGIMBAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.demanded_rate_x == (float) -4.767131E37F);
                Debug.Assert(pack.demanded_rate_y == (float)3.161091E38F);
                Debug.Assert(pack.demanded_rate_z == (float)3.2046169E38F);
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.target_component == (byte)(byte)75);
            };
            DemoDevice.GIMBAL_CONTROL p201 = LoopBackDemoChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.demanded_rate_z = (float)3.2046169E38F;
            p201.demanded_rate_y = (float)3.161091E38F;
            p201.target_component = (byte)(byte)75;
            p201.demanded_rate_x = (float) -4.767131E37F;
            p201.target_system = (byte)(byte)126;
            LoopBackDemoChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGIMBAL_TORQUE_CMD_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.el_torque_cmd == (short)(short)10519);
                Debug.Assert(pack.rl_torque_cmd == (short)(short)3727);
                Debug.Assert(pack.az_torque_cmd == (short)(short) -12291);
            };
            DemoDevice.GIMBAL_TORQUE_CMD_REPORT p214 = LoopBackDemoChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.az_torque_cmd = (short)(short) -12291;
            p214.rl_torque_cmd = (short)(short)3727;
            p214.el_torque_cmd = (short)(short)10519;
            p214.target_system = (byte)(byte)23;
            p214.target_component = (byte)(byte)39;
            LoopBackDemoChannel.instance.send(p214);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGOPRO_HEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (GOPRO_HEARTBEAT_FLAGS)GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
                Debug.Assert(pack.status == (GOPRO_HEARTBEAT_STATUS)GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE);
                Debug.Assert(pack.capture_mode == (GOPRO_CAPTURE_MODE)GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE);
            };
            DemoDevice.GOPRO_HEARTBEAT p215 = LoopBackDemoChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.capture_mode = (GOPRO_CAPTURE_MODE)GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE;
            p215.flags = (GOPRO_HEARTBEAT_FLAGS)GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            p215.status = (GOPRO_HEARTBEAT_STATUS)GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE;
            LoopBackDemoChannel.instance.send(p215);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGOPRO_GET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT);
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.target_system == (byte)(byte)220);
            };
            DemoDevice.GOPRO_GET_REQUEST p216 = LoopBackDemoChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.target_component = (byte)(byte)237;
            p216.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_LOW_LIGHT;
            p216.target_system = (byte)(byte)220;
            LoopBackDemoChannel.instance.send(p216);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGOPRO_GET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE);
                Debug.Assert(pack.status == (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)0, (byte)156, (byte)201, (byte)39}));
            };
            DemoDevice.GOPRO_GET_RESPONSE p217 = LoopBackDemoChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE;
            p217.value_SET(new byte[] {(byte)0, (byte)156, (byte)201, (byte)39}, 0) ;
            p217.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS;
            LoopBackDemoChannel.instance.send(p217);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGOPRO_SET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_BATTERY);
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)9, (byte)15, (byte)26, (byte)106}));
                Debug.Assert(pack.target_system == (byte)(byte)223);
                Debug.Assert(pack.target_component == (byte)(byte)57);
            };
            DemoDevice.GOPRO_SET_REQUEST p218 = LoopBackDemoChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.target_system = (byte)(byte)223;
            p218.target_component = (byte)(byte)57;
            p218.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_BATTERY;
            p218.value_SET(new byte[] {(byte)9, (byte)15, (byte)26, (byte)106}, 0) ;
            LoopBackDemoChannel.instance.send(p218);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGOPRO_SET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS);
                Debug.Assert(pack.status == (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
            };
            DemoDevice.GOPRO_SET_RESPONSE p219 = LoopBackDemoChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_VIDEO_SETTINGS;
            p219.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            LoopBackDemoChannel.instance.send(p219);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRPMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rpm1 == (float) -6.943034E37F);
                Debug.Assert(pack.rpm2 == (float) -2.8181429E38F);
            };
            DemoDevice.RPM p226 = LoopBackDemoChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm2 = (float) -2.8181429E38F;
            p226.rpm1 = (float) -6.943034E37F;
            LoopBackDemoChannel.instance.send(p226);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hagl_ratio == (float)1.3395645E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.215563E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.816379E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -4.811881E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -1.0051532E38F);
                Debug.Assert(pack.time_usec == (ulong)717926956144786046L);
                Debug.Assert(pack.pos_vert_ratio == (float)2.1932968E38F);
                Debug.Assert(pack.mag_ratio == (float)2.2396206E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
                Debug.Assert(pack.tas_ratio == (float)9.880022E37F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.mag_ratio = (float)2.2396206E38F;
            p230.pos_vert_accuracy = (float) -1.215563E38F;
            p230.pos_horiz_ratio = (float) -4.811881E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
            p230.time_usec = (ulong)717926956144786046L;
            p230.pos_vert_ratio = (float)2.1932968E38F;
            p230.tas_ratio = (float)9.880022E37F;
            p230.hagl_ratio = (float)1.3395645E38F;
            p230.vel_ratio = (float) -2.816379E37F;
            p230.pos_horiz_accuracy = (float) -1.0051532E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float) -3.0593946E38F);
                Debug.Assert(pack.time_usec == (ulong)8733101536277722973L);
                Debug.Assert(pack.wind_z == (float) -1.2707053E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.6868986E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.3198969E38F);
                Debug.Assert(pack.wind_x == (float) -1.9114275E38F);
                Debug.Assert(pack.wind_y == (float)2.4716424E38F);
                Debug.Assert(pack.var_vert == (float)2.8127704E38F);
                Debug.Assert(pack.wind_alt == (float) -8.400436E37F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float)1.6868986E38F;
            p231.time_usec = (ulong)8733101536277722973L;
            p231.var_vert = (float)2.8127704E38F;
            p231.wind_x = (float) -1.9114275E38F;
            p231.var_horiz = (float) -3.0593946E38F;
            p231.wind_z = (float) -1.2707053E38F;
            p231.wind_y = (float)2.4716424E38F;
            p231.horiz_accuracy = (float) -1.3198969E38F;
            p231.wind_alt = (float) -8.400436E37F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
                Debug.Assert(pack.horiz_accuracy == (float)2.9825168E38F);
                Debug.Assert(pack.vert_accuracy == (float)3.2119656E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)8522);
                Debug.Assert(pack.lat == (int)726299605);
                Debug.Assert(pack.time_week_ms == (uint)1433702094U);
                Debug.Assert(pack.speed_accuracy == (float) -2.0602229E38F);
                Debug.Assert(pack.hdop == (float)4.2560806E37F);
                Debug.Assert(pack.alt == (float) -3.1124587E37F);
                Debug.Assert(pack.lon == (int) -64465789);
                Debug.Assert(pack.time_usec == (ulong)4457888514094922481L);
                Debug.Assert(pack.gps_id == (byte)(byte)216);
                Debug.Assert(pack.vd == (float) -3.0715776E38F);
                Debug.Assert(pack.vdop == (float)3.024299E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)214);
                Debug.Assert(pack.satellites_visible == (byte)(byte)142);
                Debug.Assert(pack.ve == (float)2.9420807E38F);
                Debug.Assert(pack.vn == (float)3.6124914E37F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.hdop = (float)4.2560806E37F;
            p232.vert_accuracy = (float)3.2119656E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
            p232.time_week = (ushort)(ushort)8522;
            p232.lon = (int) -64465789;
            p232.lat = (int)726299605;
            p232.horiz_accuracy = (float)2.9825168E38F;
            p232.satellites_visible = (byte)(byte)142;
            p232.fix_type = (byte)(byte)214;
            p232.vdop = (float)3.024299E38F;
            p232.speed_accuracy = (float) -2.0602229E38F;
            p232.gps_id = (byte)(byte)216;
            p232.alt = (float) -3.1124587E37F;
            p232.vn = (float)3.6124914E37F;
            p232.time_week_ms = (uint)1433702094U;
            p232.time_usec = (ulong)4457888514094922481L;
            p232.vd = (float) -3.0715776E38F;
            p232.ve = (float)2.9420807E38F;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)129);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)222, (byte)118, (byte)238, (byte)178, (byte)108, (byte)226, (byte)112, (byte)64, (byte)127, (byte)188, (byte)77, (byte)96, (byte)123, (byte)229, (byte)48, (byte)235, (byte)143, (byte)65, (byte)95, (byte)67, (byte)164, (byte)25, (byte)14, (byte)222, (byte)194, (byte)204, (byte)168, (byte)238, (byte)207, (byte)177, (byte)37, (byte)91, (byte)153, (byte)18, (byte)77, (byte)150, (byte)161, (byte)214, (byte)227, (byte)234, (byte)97, (byte)227, (byte)153, (byte)236, (byte)108, (byte)173, (byte)72, (byte)199, (byte)118, (byte)101, (byte)84, (byte)21, (byte)190, (byte)103, (byte)125, (byte)112, (byte)92, (byte)22, (byte)163, (byte)179, (byte)176, (byte)136, (byte)62, (byte)180, (byte)209, (byte)26, (byte)44, (byte)93, (byte)62, (byte)124, (byte)191, (byte)51, (byte)17, (byte)230, (byte)174, (byte)214, (byte)55, (byte)43, (byte)188, (byte)210, (byte)152, (byte)209, (byte)206, (byte)178, (byte)179, (byte)28, (byte)103, (byte)200, (byte)131, (byte)253, (byte)123, (byte)133, (byte)182, (byte)51, (byte)117, (byte)128, (byte)61, (byte)122, (byte)252, (byte)204, (byte)8, (byte)213, (byte)171, (byte)239, (byte)92, (byte)118, (byte)204, (byte)118, (byte)200, (byte)32, (byte)234, (byte)253, (byte)69, (byte)244, (byte)216, (byte)30, (byte)166, (byte)34, (byte)83, (byte)73, (byte)252, (byte)137, (byte)5, (byte)78, (byte)201, (byte)16, (byte)33, (byte)75, (byte)255, (byte)90, (byte)178, (byte)149, (byte)83, (byte)149, (byte)82, (byte)127, (byte)242, (byte)23, (byte)139, (byte)220, (byte)239, (byte)208, (byte)207, (byte)85, (byte)66, (byte)10, (byte)242, (byte)40, (byte)190, (byte)217, (byte)73, (byte)244, (byte)199, (byte)24, (byte)251, (byte)213, (byte)221, (byte)49, (byte)47, (byte)18, (byte)117, (byte)114, (byte)48, (byte)39, (byte)169, (byte)209, (byte)64, (byte)220, (byte)192, (byte)184, (byte)244, (byte)208, (byte)74, (byte)63, (byte)158, (byte)228, (byte)253, (byte)211, (byte)37, (byte)156}));
                Debug.Assert(pack.len == (byte)(byte)0);
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)0;
            p233.flags = (byte)(byte)129;
            p233.data__SET(new byte[] {(byte)222, (byte)118, (byte)238, (byte)178, (byte)108, (byte)226, (byte)112, (byte)64, (byte)127, (byte)188, (byte)77, (byte)96, (byte)123, (byte)229, (byte)48, (byte)235, (byte)143, (byte)65, (byte)95, (byte)67, (byte)164, (byte)25, (byte)14, (byte)222, (byte)194, (byte)204, (byte)168, (byte)238, (byte)207, (byte)177, (byte)37, (byte)91, (byte)153, (byte)18, (byte)77, (byte)150, (byte)161, (byte)214, (byte)227, (byte)234, (byte)97, (byte)227, (byte)153, (byte)236, (byte)108, (byte)173, (byte)72, (byte)199, (byte)118, (byte)101, (byte)84, (byte)21, (byte)190, (byte)103, (byte)125, (byte)112, (byte)92, (byte)22, (byte)163, (byte)179, (byte)176, (byte)136, (byte)62, (byte)180, (byte)209, (byte)26, (byte)44, (byte)93, (byte)62, (byte)124, (byte)191, (byte)51, (byte)17, (byte)230, (byte)174, (byte)214, (byte)55, (byte)43, (byte)188, (byte)210, (byte)152, (byte)209, (byte)206, (byte)178, (byte)179, (byte)28, (byte)103, (byte)200, (byte)131, (byte)253, (byte)123, (byte)133, (byte)182, (byte)51, (byte)117, (byte)128, (byte)61, (byte)122, (byte)252, (byte)204, (byte)8, (byte)213, (byte)171, (byte)239, (byte)92, (byte)118, (byte)204, (byte)118, (byte)200, (byte)32, (byte)234, (byte)253, (byte)69, (byte)244, (byte)216, (byte)30, (byte)166, (byte)34, (byte)83, (byte)73, (byte)252, (byte)137, (byte)5, (byte)78, (byte)201, (byte)16, (byte)33, (byte)75, (byte)255, (byte)90, (byte)178, (byte)149, (byte)83, (byte)149, (byte)82, (byte)127, (byte)242, (byte)23, (byte)139, (byte)220, (byte)239, (byte)208, (byte)207, (byte)85, (byte)66, (byte)10, (byte)242, (byte)40, (byte)190, (byte)217, (byte)73, (byte)244, (byte)199, (byte)24, (byte)251, (byte)213, (byte)221, (byte)49, (byte)47, (byte)18, (byte)117, (byte)114, (byte)48, (byte)39, (byte)169, (byte)209, (byte)64, (byte)220, (byte)192, (byte)184, (byte)244, (byte)208, (byte)74, (byte)63, (byte)158, (byte)228, (byte)253, (byte)211, (byte)37, (byte)156}, 0) ;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (sbyte)(sbyte)1);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 46);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)47);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.wp_num == (byte)(byte)223);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)180);
                Debug.Assert(pack.pitch == (short)(short) -6100);
                Debug.Assert(pack.altitude_amsl == (short)(short) -3404);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.failsafe == (byte)(byte)22);
                Debug.Assert(pack.longitude == (int)377705255);
                Debug.Assert(pack.heading == (ushort)(ushort)149);
                Debug.Assert(pack.altitude_sp == (short)(short) -26686);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)30530);
                Debug.Assert(pack.gps_nsat == (byte)(byte)81);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)65);
                Debug.Assert(pack.airspeed == (byte)(byte)117);
                Debug.Assert(pack.groundspeed == (byte)(byte)131);
                Debug.Assert(pack.custom_mode == (uint)1223281225U);
                Debug.Assert(pack.roll == (short)(short) -5545);
                Debug.Assert(pack.latitude == (int) -1915211215);
                Debug.Assert(pack.heading_sp == (short)(short)14281);
                Debug.Assert(pack.battery_remaining == (byte)(byte)157);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short)14281;
            p234.airspeed = (byte)(byte)117;
            p234.altitude_amsl = (short)(short) -3404;
            p234.heading = (ushort)(ushort)149;
            p234.throttle = (sbyte)(sbyte)1;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p234.wp_distance = (ushort)(ushort)30530;
            p234.altitude_sp = (short)(short) -26686;
            p234.temperature_air = (sbyte)(sbyte) - 46;
            p234.gps_nsat = (byte)(byte)81;
            p234.wp_num = (byte)(byte)223;
            p234.failsafe = (byte)(byte)22;
            p234.roll = (short)(short) -5545;
            p234.battery_remaining = (byte)(byte)157;
            p234.pitch = (short)(short) -6100;
            p234.groundspeed = (byte)(byte)131;
            p234.climb_rate = (sbyte)(sbyte)47;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.temperature = (sbyte)(sbyte)65;
            p234.airspeed_sp = (byte)(byte)180;
            p234.longitude = (int)377705255;
            p234.latitude = (int) -1915211215;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p234.custom_mode = (uint)1223281225U;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_z == (float)1.6353657E38F);
                Debug.Assert(pack.clipping_0 == (uint)1042451458U);
                Debug.Assert(pack.time_usec == (ulong)2359774534062559529L);
                Debug.Assert(pack.vibration_y == (float)9.213641E37F);
                Debug.Assert(pack.clipping_1 == (uint)438171123U);
                Debug.Assert(pack.clipping_2 == (uint)107847376U);
                Debug.Assert(pack.vibration_x == (float)3.0488415E38F);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_z = (float)1.6353657E38F;
            p241.vibration_x = (float)3.0488415E38F;
            p241.clipping_0 = (uint)1042451458U;
            p241.vibration_y = (float)9.213641E37F;
            p241.time_usec = (ulong)2359774534062559529L;
            p241.clipping_2 = (uint)107847376U;
            p241.clipping_1 = (uint)438171123U;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.2222882E38F, -2.773425E38F, -1.6004246E38F, 2.1301844E38F}));
                Debug.Assert(pack.x == (float)9.707797E37F);
                Debug.Assert(pack.y == (float)6.582542E37F);
                Debug.Assert(pack.approach_x == (float) -6.490163E37F);
                Debug.Assert(pack.latitude == (int)1101344473);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3147453900870644736L);
                Debug.Assert(pack.approach_z == (float) -3.3130825E38F);
                Debug.Assert(pack.longitude == (int) -2029668693);
                Debug.Assert(pack.altitude == (int)469814705);
                Debug.Assert(pack.approach_y == (float) -1.3458317E37F);
                Debug.Assert(pack.z == (float)3.2836463E38F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.altitude = (int)469814705;
            p242.y = (float)6.582542E37F;
            p242.q_SET(new float[] {2.2222882E38F, -2.773425E38F, -1.6004246E38F, 2.1301844E38F}, 0) ;
            p242.latitude = (int)1101344473;
            p242.approach_z = (float) -3.3130825E38F;
            p242.approach_x = (float) -6.490163E37F;
            p242.longitude = (int) -2029668693;
            p242.time_usec_SET((ulong)3147453900870644736L, PH) ;
            p242.z = (float)3.2836463E38F;
            p242.x = (float)9.707797E37F;
            p242.approach_y = (float) -1.3458317E37F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -1.0298349E38F);
                Debug.Assert(pack.z == (float) -5.373923E37F);
                Debug.Assert(pack.latitude == (int) -952692946);
                Debug.Assert(pack.longitude == (int) -1465735516);
                Debug.Assert(pack.approach_x == (float) -2.5754066E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1938064E38F, 3.95059E37F, 1.2308667E38F, 3.1484218E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)763593663342089517L);
                Debug.Assert(pack.altitude == (int)1160887319);
                Debug.Assert(pack.y == (float) -1.6678861E38F);
                Debug.Assert(pack.x == (float)7.380018E37F);
                Debug.Assert(pack.approach_y == (float) -1.6143698E38F);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.longitude = (int) -1465735516;
            p243.approach_x = (float) -2.5754066E38F;
            p243.approach_z = (float) -1.0298349E38F;
            p243.approach_y = (float) -1.6143698E38F;
            p243.q_SET(new float[] {-1.1938064E38F, 3.95059E37F, 1.2308667E38F, 3.1484218E38F}, 0) ;
            p243.target_system = (byte)(byte)160;
            p243.latitude = (int) -952692946;
            p243.z = (float) -5.373923E37F;
            p243.y = (float) -1.6678861E38F;
            p243.altitude = (int)1160887319;
            p243.x = (float)7.380018E37F;
            p243.time_usec_SET((ulong)763593663342089517L, PH) ;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)18869);
                Debug.Assert(pack.interval_us == (int)1723685152);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)18869;
            p244.interval_us = (int)1723685152;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)2054129469);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)35389);
                Debug.Assert(pack.lat == (int) -184054821);
                Debug.Assert(pack.lon == (int) -788189826);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("az"));
                Debug.Assert(pack.squawk == (ushort)(ushort)46156);
                Debug.Assert(pack.tslc == (byte)(byte)123);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
                Debug.Assert(pack.ver_velocity == (short)(short) -12204);
                Debug.Assert(pack.ICAO_address == (uint)2531869808U);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
                Debug.Assert(pack.heading == (ushort)(ushort)49868);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lon = (int) -788189826;
            p246.ICAO_address = (uint)2531869808U;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
            p246.lat = (int) -184054821;
            p246.ver_velocity = (short)(short) -12204;
            p246.squawk = (ushort)(ushort)46156;
            p246.altitude = (int)2054129469;
            p246.hor_velocity = (ushort)(ushort)35389;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.tslc = (byte)(byte)123;
            p246.callsign_SET("az", PH) ;
            p246.heading = (ushort)(ushort)49868;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -2.4436622E38F);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.time_to_minimum_delta == (float)1.2645571E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float) -3.337768E38F);
                Debug.Assert(pack.id == (uint)2946316090U);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.id = (uint)2946316090U;
            p247.time_to_minimum_delta = (float)1.2645571E38F;
            p247.altitude_minimum_delta = (float) -3.337768E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.horizontal_minimum_delta = (float) -2.4436622E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.message_type == (ushort)(ushort)29981);
                Debug.Assert(pack.target_component == (byte)(byte)70);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)31, (byte)150, (byte)85, (byte)17, (byte)75, (byte)252, (byte)98, (byte)49, (byte)254, (byte)24, (byte)63, (byte)34, (byte)12, (byte)68, (byte)228, (byte)235, (byte)24, (byte)223, (byte)53, (byte)144, (byte)136, (byte)114, (byte)235, (byte)130, (byte)96, (byte)183, (byte)239, (byte)88, (byte)224, (byte)84, (byte)91, (byte)94, (byte)103, (byte)113, (byte)204, (byte)15, (byte)157, (byte)247, (byte)8, (byte)159, (byte)160, (byte)175, (byte)237, (byte)20, (byte)233, (byte)81, (byte)76, (byte)211, (byte)190, (byte)251, (byte)22, (byte)113, (byte)247, (byte)28, (byte)196, (byte)46, (byte)201, (byte)114, (byte)106, (byte)44, (byte)131, (byte)55, (byte)37, (byte)159, (byte)236, (byte)231, (byte)74, (byte)120, (byte)153, (byte)251, (byte)106, (byte)155, (byte)31, (byte)12, (byte)19, (byte)177, (byte)28, (byte)12, (byte)149, (byte)96, (byte)102, (byte)206, (byte)195, (byte)240, (byte)65, (byte)148, (byte)28, (byte)49, (byte)66, (byte)191, (byte)185, (byte)251, (byte)106, (byte)192, (byte)255, (byte)67, (byte)146, (byte)169, (byte)128, (byte)27, (byte)45, (byte)73, (byte)52, (byte)146, (byte)166, (byte)199, (byte)124, (byte)86, (byte)120, (byte)226, (byte)181, (byte)151, (byte)250, (byte)85, (byte)21, (byte)140, (byte)229, (byte)85, (byte)246, (byte)68, (byte)199, (byte)179, (byte)50, (byte)9, (byte)106, (byte)234, (byte)176, (byte)191, (byte)200, (byte)24, (byte)237, (byte)156, (byte)28, (byte)240, (byte)177, (byte)62, (byte)1, (byte)204, (byte)16, (byte)201, (byte)4, (byte)42, (byte)84, (byte)202, (byte)17, (byte)37, (byte)34, (byte)144, (byte)238, (byte)184, (byte)250, (byte)97, (byte)245, (byte)65, (byte)130, (byte)8, (byte)74, (byte)149, (byte)4, (byte)172, (byte)175, (byte)150, (byte)53, (byte)197, (byte)100, (byte)70, (byte)129, (byte)239, (byte)129, (byte)144, (byte)218, (byte)220, (byte)201, (byte)76, (byte)9, (byte)176, (byte)77, (byte)253, (byte)242, (byte)193, (byte)62, (byte)215, (byte)86, (byte)53, (byte)117, (byte)167, (byte)213, (byte)240, (byte)176, (byte)8, (byte)243, (byte)159, (byte)126, (byte)82, (byte)79, (byte)174, (byte)55, (byte)243, (byte)197, (byte)254, (byte)59, (byte)72, (byte)80, (byte)176, (byte)79, (byte)238, (byte)196, (byte)94, (byte)7, (byte)178, (byte)39, (byte)93, (byte)78, (byte)80, (byte)2, (byte)164, (byte)108, (byte)124, (byte)6, (byte)211, (byte)153, (byte)249, (byte)230, (byte)201, (byte)113, (byte)84, (byte)183, (byte)167, (byte)150, (byte)161, (byte)139, (byte)75, (byte)233, (byte)163, (byte)68, (byte)150, (byte)244, (byte)156, (byte)243, (byte)192, (byte)57, (byte)13, (byte)194, (byte)71, (byte)235, (byte)166, (byte)238, (byte)179, (byte)6}));
                Debug.Assert(pack.target_network == (byte)(byte)108);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)29981;
            p248.target_network = (byte)(byte)108;
            p248.payload_SET(new byte[] {(byte)31, (byte)150, (byte)85, (byte)17, (byte)75, (byte)252, (byte)98, (byte)49, (byte)254, (byte)24, (byte)63, (byte)34, (byte)12, (byte)68, (byte)228, (byte)235, (byte)24, (byte)223, (byte)53, (byte)144, (byte)136, (byte)114, (byte)235, (byte)130, (byte)96, (byte)183, (byte)239, (byte)88, (byte)224, (byte)84, (byte)91, (byte)94, (byte)103, (byte)113, (byte)204, (byte)15, (byte)157, (byte)247, (byte)8, (byte)159, (byte)160, (byte)175, (byte)237, (byte)20, (byte)233, (byte)81, (byte)76, (byte)211, (byte)190, (byte)251, (byte)22, (byte)113, (byte)247, (byte)28, (byte)196, (byte)46, (byte)201, (byte)114, (byte)106, (byte)44, (byte)131, (byte)55, (byte)37, (byte)159, (byte)236, (byte)231, (byte)74, (byte)120, (byte)153, (byte)251, (byte)106, (byte)155, (byte)31, (byte)12, (byte)19, (byte)177, (byte)28, (byte)12, (byte)149, (byte)96, (byte)102, (byte)206, (byte)195, (byte)240, (byte)65, (byte)148, (byte)28, (byte)49, (byte)66, (byte)191, (byte)185, (byte)251, (byte)106, (byte)192, (byte)255, (byte)67, (byte)146, (byte)169, (byte)128, (byte)27, (byte)45, (byte)73, (byte)52, (byte)146, (byte)166, (byte)199, (byte)124, (byte)86, (byte)120, (byte)226, (byte)181, (byte)151, (byte)250, (byte)85, (byte)21, (byte)140, (byte)229, (byte)85, (byte)246, (byte)68, (byte)199, (byte)179, (byte)50, (byte)9, (byte)106, (byte)234, (byte)176, (byte)191, (byte)200, (byte)24, (byte)237, (byte)156, (byte)28, (byte)240, (byte)177, (byte)62, (byte)1, (byte)204, (byte)16, (byte)201, (byte)4, (byte)42, (byte)84, (byte)202, (byte)17, (byte)37, (byte)34, (byte)144, (byte)238, (byte)184, (byte)250, (byte)97, (byte)245, (byte)65, (byte)130, (byte)8, (byte)74, (byte)149, (byte)4, (byte)172, (byte)175, (byte)150, (byte)53, (byte)197, (byte)100, (byte)70, (byte)129, (byte)239, (byte)129, (byte)144, (byte)218, (byte)220, (byte)201, (byte)76, (byte)9, (byte)176, (byte)77, (byte)253, (byte)242, (byte)193, (byte)62, (byte)215, (byte)86, (byte)53, (byte)117, (byte)167, (byte)213, (byte)240, (byte)176, (byte)8, (byte)243, (byte)159, (byte)126, (byte)82, (byte)79, (byte)174, (byte)55, (byte)243, (byte)197, (byte)254, (byte)59, (byte)72, (byte)80, (byte)176, (byte)79, (byte)238, (byte)196, (byte)94, (byte)7, (byte)178, (byte)39, (byte)93, (byte)78, (byte)80, (byte)2, (byte)164, (byte)108, (byte)124, (byte)6, (byte)211, (byte)153, (byte)249, (byte)230, (byte)201, (byte)113, (byte)84, (byte)183, (byte)167, (byte)150, (byte)161, (byte)139, (byte)75, (byte)233, (byte)163, (byte)68, (byte)150, (byte)244, (byte)156, (byte)243, (byte)192, (byte)57, (byte)13, (byte)194, (byte)71, (byte)235, (byte)166, (byte)238, (byte)179, (byte)6}, 0) ;
            p248.target_component = (byte)(byte)70;
            p248.target_system = (byte)(byte)17;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)174);
                Debug.Assert(pack.ver == (byte)(byte)164);
                Debug.Assert(pack.address == (ushort)(ushort)49759);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 6, (sbyte) - 99, (sbyte) - 13, (sbyte)5, (sbyte)122, (sbyte)21, (sbyte)32, (sbyte)115, (sbyte) - 93, (sbyte)30, (sbyte)60, (sbyte) - 100, (sbyte)13, (sbyte)127, (sbyte)58, (sbyte) - 48, (sbyte)67, (sbyte)61, (sbyte) - 73, (sbyte)71, (sbyte)118, (sbyte) - 74, (sbyte) - 123, (sbyte) - 36, (sbyte) - 46, (sbyte)68, (sbyte)58, (sbyte) - 1, (sbyte) - 30, (sbyte) - 96, (sbyte) - 16, (sbyte)102}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)49759;
            p249.type = (byte)(byte)174;
            p249.value_SET(new sbyte[] {(sbyte) - 6, (sbyte) - 99, (sbyte) - 13, (sbyte)5, (sbyte)122, (sbyte)21, (sbyte)32, (sbyte)115, (sbyte) - 93, (sbyte)30, (sbyte)60, (sbyte) - 100, (sbyte)13, (sbyte)127, (sbyte)58, (sbyte) - 48, (sbyte)67, (sbyte)61, (sbyte) - 73, (sbyte)71, (sbyte)118, (sbyte) - 74, (sbyte) - 123, (sbyte) - 36, (sbyte) - 46, (sbyte)68, (sbyte)58, (sbyte) - 1, (sbyte) - 30, (sbyte) - 96, (sbyte) - 16, (sbyte)102}, 0) ;
            p249.ver = (byte)(byte)164;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.7783954E38F);
                Debug.Assert(pack.time_usec == (ulong)2791363833731902660L);
                Debug.Assert(pack.y == (float)2.9953093E38F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("mxnuij"));
                Debug.Assert(pack.x == (float) -2.6277099E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("mxnuij", PH) ;
            p250.y = (float)2.9953093E38F;
            p250.x = (float) -2.6277099E38F;
            p250.z = (float)2.7783954E38F;
            p250.time_usec = (ulong)2791363833731902660L;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1189079565U);
                Debug.Assert(pack.value == (float) -5.623415E37F);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("qmfv"));
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -5.623415E37F;
            p251.time_boot_ms = (uint)1189079565U;
            p251.name_SET("qmfv", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("tp"));
                Debug.Assert(pack.value == (int)1132563219);
                Debug.Assert(pack.time_boot_ms == (uint)4158267373U);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)4158267373U;
            p252.value = (int)1132563219;
            p252.name_SET("tp", PH) ;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
                Debug.Assert(pack.text_LEN(ph) == 4);
                Debug.Assert(pack.text_TRY(ph).Equals("dlqg"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("dlqg", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)72);
                Debug.Assert(pack.value == (float)1.0894317E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1743882150U);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)72;
            p254.time_boot_ms = (uint)1743882150U;
            p254.value = (float)1.0894317E38F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)105, (byte)179, (byte)232, (byte)237, (byte)127, (byte)62, (byte)106, (byte)175, (byte)29, (byte)111, (byte)146, (byte)168, (byte)145, (byte)6, (byte)198, (byte)30, (byte)147, (byte)154, (byte)7, (byte)57, (byte)178, (byte)19, (byte)119, (byte)46, (byte)77, (byte)136, (byte)14, (byte)107, (byte)246, (byte)38, (byte)198, (byte)65}));
                Debug.Assert(pack.initial_timestamp == (ulong)7855004358647016184L);
                Debug.Assert(pack.target_system == (byte)(byte)233);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)233;
            p256.secret_key_SET(new byte[] {(byte)105, (byte)179, (byte)232, (byte)237, (byte)127, (byte)62, (byte)106, (byte)175, (byte)29, (byte)111, (byte)146, (byte)168, (byte)145, (byte)6, (byte)198, (byte)30, (byte)147, (byte)154, (byte)7, (byte)57, (byte)178, (byte)19, (byte)119, (byte)46, (byte)77, (byte)136, (byte)14, (byte)107, (byte)246, (byte)38, (byte)198, (byte)65}, 0) ;
            p256.initial_timestamp = (ulong)7855004358647016184L;
            p256.target_component = (byte)(byte)11;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)187);
                Debug.Assert(pack.last_change_ms == (uint)1483900943U);
                Debug.Assert(pack.time_boot_ms == (uint)405909885U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)1483900943U;
            p257.state = (byte)(byte)187;
            p257.time_boot_ms = (uint)405909885U;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.tune_LEN(ph) == 8);
                Debug.Assert(pack.tune_TRY(ph).Equals("bhlolcjb"));
                Debug.Assert(pack.target_system == (byte)(byte)213);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)171;
            p258.tune_SET("bhlolcjb", PH) ;
            p258.target_system = (byte)(byte)213;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)35239);
                Debug.Assert(pack.focal_length == (float)2.8015965E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)21900);
                Debug.Assert(pack.sensor_size_h == (float)1.7312129E37F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)209, (byte)111, (byte)223, (byte)140, (byte)210, (byte)43, (byte)1, (byte)58, (byte)13, (byte)106, (byte)50, (byte)48, (byte)73, (byte)79, (byte)199, (byte)178, (byte)168, (byte)229, (byte)39, (byte)130, (byte)51, (byte)127, (byte)178, (byte)239, (byte)3, (byte)230, (byte)150, (byte)253, (byte)217, (byte)175, (byte)16, (byte)66}));
                Debug.Assert(pack.lens_id == (byte)(byte)198);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 20);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("fgsxksriafdhybknbftq"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)72, (byte)126, (byte)191, (byte)40, (byte)51, (byte)122, (byte)26, (byte)173, (byte)226, (byte)174, (byte)50, (byte)11, (byte)91, (byte)28, (byte)125, (byte)158, (byte)85, (byte)103, (byte)80, (byte)82, (byte)205, (byte)111, (byte)115, (byte)143, (byte)240, (byte)84, (byte)238, (byte)230, (byte)167, (byte)215, (byte)161, (byte)16}));
                Debug.Assert(pack.time_boot_ms == (uint)225814273U);
                Debug.Assert(pack.firmware_version == (uint)1199582652U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)62206);
                Debug.Assert(pack.sensor_size_v == (float)1.2824183E38F);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_h = (ushort)(ushort)62206;
            p259.cam_definition_uri_SET("fgsxksriafdhybknbftq", PH) ;
            p259.model_name_SET(new byte[] {(byte)72, (byte)126, (byte)191, (byte)40, (byte)51, (byte)122, (byte)26, (byte)173, (byte)226, (byte)174, (byte)50, (byte)11, (byte)91, (byte)28, (byte)125, (byte)158, (byte)85, (byte)103, (byte)80, (byte)82, (byte)205, (byte)111, (byte)115, (byte)143, (byte)240, (byte)84, (byte)238, (byte)230, (byte)167, (byte)215, (byte)161, (byte)16}, 0) ;
            p259.focal_length = (float)2.8015965E38F;
            p259.firmware_version = (uint)1199582652U;
            p259.cam_definition_version = (ushort)(ushort)35239;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            p259.resolution_v = (ushort)(ushort)21900;
            p259.vendor_name_SET(new byte[] {(byte)209, (byte)111, (byte)223, (byte)140, (byte)210, (byte)43, (byte)1, (byte)58, (byte)13, (byte)106, (byte)50, (byte)48, (byte)73, (byte)79, (byte)199, (byte)178, (byte)168, (byte)229, (byte)39, (byte)130, (byte)51, (byte)127, (byte)178, (byte)239, (byte)3, (byte)230, (byte)150, (byte)253, (byte)217, (byte)175, (byte)16, (byte)66}, 0) ;
            p259.sensor_size_h = (float)1.7312129E37F;
            p259.time_boot_ms = (uint)225814273U;
            p259.sensor_size_v = (float)1.2824183E38F;
            p259.lens_id = (byte)(byte)198;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1091348060U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)1091348060U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.read_speed == (float)1.2551301E38F);
                Debug.Assert(pack.total_capacity == (float)1.9132818E37F);
                Debug.Assert(pack.storage_count == (byte)(byte)225);
                Debug.Assert(pack.write_speed == (float) -4.2381763E37F);
                Debug.Assert(pack.status == (byte)(byte)53);
                Debug.Assert(pack.time_boot_ms == (uint)309360569U);
                Debug.Assert(pack.storage_id == (byte)(byte)11);
                Debug.Assert(pack.available_capacity == (float)2.3058877E38F);
                Debug.Assert(pack.used_capacity == (float) -2.5183859E38F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float)1.2551301E38F;
            p261.available_capacity = (float)2.3058877E38F;
            p261.write_speed = (float) -4.2381763E37F;
            p261.storage_count = (byte)(byte)225;
            p261.storage_id = (byte)(byte)11;
            p261.time_boot_ms = (uint)309360569U;
            p261.used_capacity = (float) -2.5183859E38F;
            p261.status = (byte)(byte)53;
            p261.total_capacity = (float)1.9132818E37F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)278241519U);
                Debug.Assert(pack.video_status == (byte)(byte)89);
                Debug.Assert(pack.time_boot_ms == (uint)1697014282U);
                Debug.Assert(pack.available_capacity == (float)1.2064575E38F);
                Debug.Assert(pack.image_status == (byte)(byte)17);
                Debug.Assert(pack.image_interval == (float) -2.0615418E38F);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float) -2.0615418E38F;
            p262.time_boot_ms = (uint)1697014282U;
            p262.available_capacity = (float)1.2064575E38F;
            p262.video_status = (byte)(byte)89;
            p262.image_status = (byte)(byte)17;
            p262.recording_time_ms = (uint)278241519U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2095419050U);
                Debug.Assert(pack.image_index == (int) -1641825909);
                Debug.Assert(pack.alt == (int)1086756918);
                Debug.Assert(pack.camera_id == (byte)(byte)51);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-5.0507733E37F, 1.4652404E38F, -1.5341777E37F, -1.1608939E38F}));
                Debug.Assert(pack.lon == (int) -1318671947);
                Debug.Assert(pack.lat == (int) -2142822802);
                Debug.Assert(pack.file_url_LEN(ph) == 87);
                Debug.Assert(pack.file_url_TRY(ph).Equals("AsmeRyzpjpfjyuiffrEcsncpohgyzdvEqoOhsyhsibvantmtroellpBYqMlmdwLzaoddzvvccefUfsoozsFmjun"));
                Debug.Assert(pack.time_utc == (ulong)8724308497035684252L);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 64);
                Debug.Assert(pack.relative_alt == (int) -1418951171);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lat = (int) -2142822802;
            p263.relative_alt = (int) -1418951171;
            p263.image_index = (int) -1641825909;
            p263.camera_id = (byte)(byte)51;
            p263.time_utc = (ulong)8724308497035684252L;
            p263.q_SET(new float[] {-5.0507733E37F, 1.4652404E38F, -1.5341777E37F, -1.1608939E38F}, 0) ;
            p263.lon = (int) -1318671947;
            p263.file_url_SET("AsmeRyzpjpfjyuiffrEcsncpohgyzdvEqoOhsyhsibvantmtroellpBYqMlmdwLzaoddzvvccefUfsoozsFmjun", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 64;
            p263.time_boot_ms = (uint)2095419050U;
            p263.alt = (int)1086756918;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2760786559U);
                Debug.Assert(pack.arming_time_utc == (ulong)725998071790533207L);
                Debug.Assert(pack.flight_uuid == (ulong)3291736053213503058L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)8622859663437131454L);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)8622859663437131454L;
            p264.arming_time_utc = (ulong)725998071790533207L;
            p264.time_boot_ms = (uint)2760786559U;
            p264.flight_uuid = (ulong)3291736053213503058L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)3.2311592E38F);
                Debug.Assert(pack.yaw == (float)2.9263467E38F);
                Debug.Assert(pack.pitch == (float) -3.1368848E38F);
                Debug.Assert(pack.time_boot_ms == (uint)808607643U);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)808607643U;
            p265.yaw = (float)2.9263467E38F;
            p265.roll = (float)3.2311592E38F;
            p265.pitch = (float) -3.1368848E38F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)128, (byte)31, (byte)29, (byte)126, (byte)155, (byte)81, (byte)18, (byte)182, (byte)47, (byte)100, (byte)104, (byte)168, (byte)10, (byte)166, (byte)138, (byte)14, (byte)120, (byte)26, (byte)44, (byte)15, (byte)121, (byte)204, (byte)121, (byte)125, (byte)221, (byte)129, (byte)191, (byte)144, (byte)83, (byte)236, (byte)158, (byte)196, (byte)244, (byte)217, (byte)35, (byte)143, (byte)61, (byte)79, (byte)229, (byte)79, (byte)125, (byte)88, (byte)90, (byte)97, (byte)26, (byte)197, (byte)195, (byte)61, (byte)93, (byte)104, (byte)128, (byte)147, (byte)185, (byte)41, (byte)144, (byte)20, (byte)10, (byte)48, (byte)221, (byte)222, (byte)212, (byte)175, (byte)38, (byte)81, (byte)70, (byte)139, (byte)115, (byte)37, (byte)209, (byte)151, (byte)4, (byte)186, (byte)126, (byte)51, (byte)173, (byte)222, (byte)147, (byte)18, (byte)228, (byte)187, (byte)225, (byte)2, (byte)104, (byte)60, (byte)35, (byte)218, (byte)241, (byte)1, (byte)123, (byte)69, (byte)117, (byte)8, (byte)80, (byte)115, (byte)186, (byte)126, (byte)91, (byte)70, (byte)237, (byte)170, (byte)249, (byte)253, (byte)186, (byte)53, (byte)57, (byte)157, (byte)140, (byte)59, (byte)113, (byte)45, (byte)222, (byte)236, (byte)135, (byte)2, (byte)143, (byte)95, (byte)160, (byte)2, (byte)60, (byte)168, (byte)137, (byte)5, (byte)6, (byte)100, (byte)11, (byte)32, (byte)76, (byte)54, (byte)118, (byte)97, (byte)79, (byte)104, (byte)144, (byte)13, (byte)118, (byte)66, (byte)121, (byte)251, (byte)120, (byte)134, (byte)243, (byte)8, (byte)124, (byte)168, (byte)65, (byte)224, (byte)104, (byte)109, (byte)125, (byte)0, (byte)21, (byte)31, (byte)202, (byte)189, (byte)86, (byte)62, (byte)244, (byte)92, (byte)22, (byte)222, (byte)142, (byte)57, (byte)205, (byte)48, (byte)163, (byte)9, (byte)63, (byte)17, (byte)138, (byte)150, (byte)1, (byte)55, (byte)169, (byte)24, (byte)35, (byte)255, (byte)129, (byte)181, (byte)138, (byte)63, (byte)36, (byte)159, (byte)206, (byte)128, (byte)195, (byte)25, (byte)98, (byte)44, (byte)115, (byte)203, (byte)70, (byte)239, (byte)19, (byte)236, (byte)18, (byte)128, (byte)116, (byte)221, (byte)215, (byte)237, (byte)247, (byte)142, (byte)54, (byte)85, (byte)156, (byte)206, (byte)117, (byte)22, (byte)19, (byte)119, (byte)190, (byte)4, (byte)191, (byte)236, (byte)255, (byte)177, (byte)105, (byte)105, (byte)188, (byte)39, (byte)86, (byte)157, (byte)59, (byte)155, (byte)79, (byte)239, (byte)74, (byte)40, (byte)168, (byte)25, (byte)75, (byte)215, (byte)165, (byte)45, (byte)92, (byte)42, (byte)188, (byte)37, (byte)110, (byte)202, (byte)13, (byte)68, (byte)77, (byte)73, (byte)89, (byte)105, (byte)102, (byte)9, (byte)87}));
                Debug.Assert(pack.target_component == (byte)(byte)112);
                Debug.Assert(pack.target_system == (byte)(byte)211);
                Debug.Assert(pack.length == (byte)(byte)90);
                Debug.Assert(pack.first_message_offset == (byte)(byte)90);
                Debug.Assert(pack.sequence == (ushort)(ushort)34257);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.length = (byte)(byte)90;
            p266.target_system = (byte)(byte)211;
            p266.sequence = (ushort)(ushort)34257;
            p266.target_component = (byte)(byte)112;
            p266.first_message_offset = (byte)(byte)90;
            p266.data__SET(new byte[] {(byte)128, (byte)31, (byte)29, (byte)126, (byte)155, (byte)81, (byte)18, (byte)182, (byte)47, (byte)100, (byte)104, (byte)168, (byte)10, (byte)166, (byte)138, (byte)14, (byte)120, (byte)26, (byte)44, (byte)15, (byte)121, (byte)204, (byte)121, (byte)125, (byte)221, (byte)129, (byte)191, (byte)144, (byte)83, (byte)236, (byte)158, (byte)196, (byte)244, (byte)217, (byte)35, (byte)143, (byte)61, (byte)79, (byte)229, (byte)79, (byte)125, (byte)88, (byte)90, (byte)97, (byte)26, (byte)197, (byte)195, (byte)61, (byte)93, (byte)104, (byte)128, (byte)147, (byte)185, (byte)41, (byte)144, (byte)20, (byte)10, (byte)48, (byte)221, (byte)222, (byte)212, (byte)175, (byte)38, (byte)81, (byte)70, (byte)139, (byte)115, (byte)37, (byte)209, (byte)151, (byte)4, (byte)186, (byte)126, (byte)51, (byte)173, (byte)222, (byte)147, (byte)18, (byte)228, (byte)187, (byte)225, (byte)2, (byte)104, (byte)60, (byte)35, (byte)218, (byte)241, (byte)1, (byte)123, (byte)69, (byte)117, (byte)8, (byte)80, (byte)115, (byte)186, (byte)126, (byte)91, (byte)70, (byte)237, (byte)170, (byte)249, (byte)253, (byte)186, (byte)53, (byte)57, (byte)157, (byte)140, (byte)59, (byte)113, (byte)45, (byte)222, (byte)236, (byte)135, (byte)2, (byte)143, (byte)95, (byte)160, (byte)2, (byte)60, (byte)168, (byte)137, (byte)5, (byte)6, (byte)100, (byte)11, (byte)32, (byte)76, (byte)54, (byte)118, (byte)97, (byte)79, (byte)104, (byte)144, (byte)13, (byte)118, (byte)66, (byte)121, (byte)251, (byte)120, (byte)134, (byte)243, (byte)8, (byte)124, (byte)168, (byte)65, (byte)224, (byte)104, (byte)109, (byte)125, (byte)0, (byte)21, (byte)31, (byte)202, (byte)189, (byte)86, (byte)62, (byte)244, (byte)92, (byte)22, (byte)222, (byte)142, (byte)57, (byte)205, (byte)48, (byte)163, (byte)9, (byte)63, (byte)17, (byte)138, (byte)150, (byte)1, (byte)55, (byte)169, (byte)24, (byte)35, (byte)255, (byte)129, (byte)181, (byte)138, (byte)63, (byte)36, (byte)159, (byte)206, (byte)128, (byte)195, (byte)25, (byte)98, (byte)44, (byte)115, (byte)203, (byte)70, (byte)239, (byte)19, (byte)236, (byte)18, (byte)128, (byte)116, (byte)221, (byte)215, (byte)237, (byte)247, (byte)142, (byte)54, (byte)85, (byte)156, (byte)206, (byte)117, (byte)22, (byte)19, (byte)119, (byte)190, (byte)4, (byte)191, (byte)236, (byte)255, (byte)177, (byte)105, (byte)105, (byte)188, (byte)39, (byte)86, (byte)157, (byte)59, (byte)155, (byte)79, (byte)239, (byte)74, (byte)40, (byte)168, (byte)25, (byte)75, (byte)215, (byte)165, (byte)45, (byte)92, (byte)42, (byte)188, (byte)37, (byte)110, (byte)202, (byte)13, (byte)68, (byte)77, (byte)73, (byte)89, (byte)105, (byte)102, (byte)9, (byte)87}, 0) ;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)38);
                Debug.Assert(pack.first_message_offset == (byte)(byte)191);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.target_component == (byte)(byte)12);
                Debug.Assert(pack.sequence == (ushort)(ushort)14831);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)12, (byte)126, (byte)178, (byte)142, (byte)136, (byte)157, (byte)27, (byte)102, (byte)95, (byte)35, (byte)134, (byte)44, (byte)12, (byte)70, (byte)80, (byte)137, (byte)112, (byte)177, (byte)208, (byte)71, (byte)101, (byte)149, (byte)101, (byte)149, (byte)37, (byte)193, (byte)100, (byte)31, (byte)234, (byte)48, (byte)37, (byte)165, (byte)95, (byte)188, (byte)32, (byte)91, (byte)161, (byte)37, (byte)145, (byte)221, (byte)203, (byte)230, (byte)79, (byte)69, (byte)46, (byte)173, (byte)134, (byte)154, (byte)122, (byte)26, (byte)156, (byte)239, (byte)130, (byte)149, (byte)210, (byte)19, (byte)137, (byte)110, (byte)142, (byte)141, (byte)255, (byte)63, (byte)22, (byte)30, (byte)29, (byte)140, (byte)95, (byte)252, (byte)7, (byte)42, (byte)247, (byte)252, (byte)125, (byte)166, (byte)91, (byte)111, (byte)227, (byte)115, (byte)98, (byte)167, (byte)104, (byte)115, (byte)168, (byte)119, (byte)224, (byte)86, (byte)173, (byte)237, (byte)67, (byte)2, (byte)215, (byte)39, (byte)118, (byte)243, (byte)217, (byte)120, (byte)194, (byte)120, (byte)163, (byte)157, (byte)34, (byte)114, (byte)82, (byte)73, (byte)44, (byte)71, (byte)2, (byte)0, (byte)115, (byte)238, (byte)187, (byte)98, (byte)164, (byte)4, (byte)75, (byte)129, (byte)21, (byte)3, (byte)152, (byte)20, (byte)218, (byte)229, (byte)162, (byte)4, (byte)247, (byte)137, (byte)74, (byte)122, (byte)47, (byte)35, (byte)2, (byte)181, (byte)132, (byte)53, (byte)188, (byte)14, (byte)252, (byte)121, (byte)23, (byte)217, (byte)176, (byte)41, (byte)178, (byte)58, (byte)8, (byte)47, (byte)163, (byte)8, (byte)12, (byte)223, (byte)243, (byte)121, (byte)147, (byte)208, (byte)95, (byte)249, (byte)14, (byte)164, (byte)95, (byte)76, (byte)223, (byte)74, (byte)127, (byte)228, (byte)136, (byte)232, (byte)112, (byte)61, (byte)255, (byte)73, (byte)102, (byte)196, (byte)152, (byte)196, (byte)103, (byte)166, (byte)154, (byte)144, (byte)167, (byte)44, (byte)248, (byte)89, (byte)238, (byte)4, (byte)134, (byte)247, (byte)245, (byte)44, (byte)95, (byte)74, (byte)231, (byte)154, (byte)70, (byte)195, (byte)127, (byte)99, (byte)9, (byte)12, (byte)73, (byte)41, (byte)74, (byte)45, (byte)224, (byte)62, (byte)87, (byte)238, (byte)233, (byte)194, (byte)203, (byte)74, (byte)12, (byte)20, (byte)40, (byte)199, (byte)242, (byte)63, (byte)248, (byte)91, (byte)131, (byte)50, (byte)122, (byte)252, (byte)87, (byte)223, (byte)6, (byte)36, (byte)67, (byte)235, (byte)11, (byte)29, (byte)84, (byte)88, (byte)71, (byte)115, (byte)153, (byte)174, (byte)225, (byte)128, (byte)211, (byte)84, (byte)136, (byte)96, (byte)18, (byte)7, (byte)27, (byte)6, (byte)138, (byte)230, (byte)122}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)191;
            p267.length = (byte)(byte)38;
            p267.target_component = (byte)(byte)12;
            p267.sequence = (ushort)(ushort)14831;
            p267.target_system = (byte)(byte)203;
            p267.data__SET(new byte[] {(byte)12, (byte)126, (byte)178, (byte)142, (byte)136, (byte)157, (byte)27, (byte)102, (byte)95, (byte)35, (byte)134, (byte)44, (byte)12, (byte)70, (byte)80, (byte)137, (byte)112, (byte)177, (byte)208, (byte)71, (byte)101, (byte)149, (byte)101, (byte)149, (byte)37, (byte)193, (byte)100, (byte)31, (byte)234, (byte)48, (byte)37, (byte)165, (byte)95, (byte)188, (byte)32, (byte)91, (byte)161, (byte)37, (byte)145, (byte)221, (byte)203, (byte)230, (byte)79, (byte)69, (byte)46, (byte)173, (byte)134, (byte)154, (byte)122, (byte)26, (byte)156, (byte)239, (byte)130, (byte)149, (byte)210, (byte)19, (byte)137, (byte)110, (byte)142, (byte)141, (byte)255, (byte)63, (byte)22, (byte)30, (byte)29, (byte)140, (byte)95, (byte)252, (byte)7, (byte)42, (byte)247, (byte)252, (byte)125, (byte)166, (byte)91, (byte)111, (byte)227, (byte)115, (byte)98, (byte)167, (byte)104, (byte)115, (byte)168, (byte)119, (byte)224, (byte)86, (byte)173, (byte)237, (byte)67, (byte)2, (byte)215, (byte)39, (byte)118, (byte)243, (byte)217, (byte)120, (byte)194, (byte)120, (byte)163, (byte)157, (byte)34, (byte)114, (byte)82, (byte)73, (byte)44, (byte)71, (byte)2, (byte)0, (byte)115, (byte)238, (byte)187, (byte)98, (byte)164, (byte)4, (byte)75, (byte)129, (byte)21, (byte)3, (byte)152, (byte)20, (byte)218, (byte)229, (byte)162, (byte)4, (byte)247, (byte)137, (byte)74, (byte)122, (byte)47, (byte)35, (byte)2, (byte)181, (byte)132, (byte)53, (byte)188, (byte)14, (byte)252, (byte)121, (byte)23, (byte)217, (byte)176, (byte)41, (byte)178, (byte)58, (byte)8, (byte)47, (byte)163, (byte)8, (byte)12, (byte)223, (byte)243, (byte)121, (byte)147, (byte)208, (byte)95, (byte)249, (byte)14, (byte)164, (byte)95, (byte)76, (byte)223, (byte)74, (byte)127, (byte)228, (byte)136, (byte)232, (byte)112, (byte)61, (byte)255, (byte)73, (byte)102, (byte)196, (byte)152, (byte)196, (byte)103, (byte)166, (byte)154, (byte)144, (byte)167, (byte)44, (byte)248, (byte)89, (byte)238, (byte)4, (byte)134, (byte)247, (byte)245, (byte)44, (byte)95, (byte)74, (byte)231, (byte)154, (byte)70, (byte)195, (byte)127, (byte)99, (byte)9, (byte)12, (byte)73, (byte)41, (byte)74, (byte)45, (byte)224, (byte)62, (byte)87, (byte)238, (byte)233, (byte)194, (byte)203, (byte)74, (byte)12, (byte)20, (byte)40, (byte)199, (byte)242, (byte)63, (byte)248, (byte)91, (byte)131, (byte)50, (byte)122, (byte)252, (byte)87, (byte)223, (byte)6, (byte)36, (byte)67, (byte)235, (byte)11, (byte)29, (byte)84, (byte)88, (byte)71, (byte)115, (byte)153, (byte)174, (byte)225, (byte)128, (byte)211, (byte)84, (byte)136, (byte)96, (byte)18, (byte)7, (byte)27, (byte)6, (byte)138, (byte)230, (byte)122}, 0) ;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)57718);
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.target_component == (byte)(byte)239);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)57718;
            p268.target_system = (byte)(byte)43;
            p268.target_component = (byte)(byte)239;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rotation == (ushort)(ushort)9678);
                Debug.Assert(pack.camera_id == (byte)(byte)18);
                Debug.Assert(pack.uri_LEN(ph) == 80);
                Debug.Assert(pack.uri_TRY(ph).Equals("byktoxvhsdftnbcbpcsShcvtbeufrjcesnJonMpwlyxXimjrthwpSbmgydpmpqprqvwzqyetcwCrxefn"));
                Debug.Assert(pack.bitrate == (uint)998139776U);
                Debug.Assert(pack.framerate == (float) -5.047152E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)60292);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)27126);
                Debug.Assert(pack.status == (byte)(byte)90);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.uri_SET("byktoxvhsdftnbcbpcsShcvtbeufrjcesnJonMpwlyxXimjrthwpSbmgydpmpqprqvwzqyetcwCrxefn", PH) ;
            p269.status = (byte)(byte)90;
            p269.rotation = (ushort)(ushort)9678;
            p269.camera_id = (byte)(byte)18;
            p269.resolution_h = (ushort)(ushort)60292;
            p269.framerate = (float) -5.047152E37F;
            p269.bitrate = (uint)998139776U;
            p269.resolution_v = (ushort)(ushort)27126;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)9723);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.rotation == (ushort)(ushort)16634);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.camera_id == (byte)(byte)92);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)61050);
                Debug.Assert(pack.bitrate == (uint)3293134545U);
                Debug.Assert(pack.uri_LEN(ph) == 82);
                Debug.Assert(pack.uri_TRY(ph).Equals("uhqpmakCfnplxfsvNXlkatdwohujzBjngzytgjscndekaaxkGpmclbkyRSyayqgVocepCTuzikfMaXzmdi"));
                Debug.Assert(pack.framerate == (float)1.2446439E38F);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)9723;
            p270.resolution_h = (ushort)(ushort)61050;
            p270.rotation = (ushort)(ushort)16634;
            p270.framerate = (float)1.2446439E38F;
            p270.camera_id = (byte)(byte)92;
            p270.uri_SET("uhqpmakCfnplxfsvNXlkatdwohujzBjngzytgjscndekaaxkGpmclbkyRSyayqgVocepCTuzikfMaXzmdi", PH) ;
            p270.bitrate = (uint)3293134545U;
            p270.target_component = (byte)(byte)100;
            p270.target_system = (byte)(byte)140;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 25);
                Debug.Assert(pack.ssid_TRY(ph).Equals("qeofoevnzzccdwmztmvgzapdo"));
                Debug.Assert(pack.password_LEN(ph) == 34);
                Debug.Assert(pack.password_TRY(ph).Equals("fgjenakhbkbohfhngrfkgrWjtpaDwsIqkO"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("fgjenakhbkbohfhngrfkgrWjtpaDwsIqkO", PH) ;
            p299.ssid_SET("qeofoevnzzccdwmztmvgzapdo", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)48502);
                Debug.Assert(pack.version == (ushort)(ushort)49931);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)224, (byte)176, (byte)159, (byte)61, (byte)108, (byte)239, (byte)101, (byte)170}));
                Debug.Assert(pack.max_version == (ushort)(ushort)36118);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)193, (byte)76, (byte)204, (byte)230, (byte)146, (byte)240, (byte)177, (byte)188}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)48502;
            p300.max_version = (ushort)(ushort)36118;
            p300.library_version_hash_SET(new byte[] {(byte)193, (byte)76, (byte)204, (byte)230, (byte)146, (byte)240, (byte)177, (byte)188}, 0) ;
            p300.version = (ushort)(ushort)49931;
            p300.spec_version_hash_SET(new byte[] {(byte)224, (byte)176, (byte)159, (byte)61, (byte)108, (byte)239, (byte)101, (byte)170}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3987775782U);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)9893);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.time_usec == (ulong)7570086916843014004L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
                Debug.Assert(pack.sub_mode == (byte)(byte)81);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)9893;
            p310.time_usec = (ulong)7570086916843014004L;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)81;
            p310.uptime_sec = (uint)3987775782U;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_minor == (byte)(byte)180);
                Debug.Assert(pack.name_LEN(ph) == 57);
                Debug.Assert(pack.name_TRY(ph).Equals("VlkqzldRgptrsrhknsgatlrynrrxfCahspfvlstpvZwilaFicxadfkPhl"));
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)78, (byte)127, (byte)138, (byte)71, (byte)225, (byte)228, (byte)201, (byte)54, (byte)149, (byte)94, (byte)8, (byte)95, (byte)252, (byte)245, (byte)229, (byte)198}));
                Debug.Assert(pack.sw_vcs_commit == (uint)1539515709U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)33);
                Debug.Assert(pack.time_usec == (ulong)8174650852807472249L);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)59);
                Debug.Assert(pack.uptime_sec == (uint)2683155931U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)164);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)59;
            p311.hw_version_major = (byte)(byte)164;
            p311.uptime_sec = (uint)2683155931U;
            p311.time_usec = (ulong)8174650852807472249L;
            p311.name_SET("VlkqzldRgptrsrhknsgatlrynrrxfCahspfvlstpvZwilaFicxadfkPhl", PH) ;
            p311.sw_vcs_commit = (uint)1539515709U;
            p311.hw_version_minor = (byte)(byte)180;
            p311.sw_version_major = (byte)(byte)33;
            p311.hw_unique_id_SET(new byte[] {(byte)78, (byte)127, (byte)138, (byte)71, (byte)225, (byte)228, (byte)201, (byte)54, (byte)149, (byte)94, (byte)8, (byte)95, (byte)252, (byte)245, (byte)229, (byte)198}, 0) ;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("lcxgpibca"));
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.target_component == (byte)(byte)49);
                Debug.Assert(pack.param_index == (short)(short) -21134);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("lcxgpibca", PH) ;
            p320.target_system = (byte)(byte)33;
            p320.target_component = (byte)(byte)49;
            p320.param_index = (short)(short) -21134;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)50);
                Debug.Assert(pack.target_component == (byte)(byte)163);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)163;
            p321.target_system = (byte)(byte)50;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ivalcylvau"));
                Debug.Assert(pack.param_count == (ushort)(ushort)64990);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_index == (ushort)(ushort)36843);
                Debug.Assert(pack.param_value_LEN(ph) == 35);
                Debug.Assert(pack.param_value_TRY(ph).Equals("kEjxjufyatvluhbPhkcbjxdwhyhowbDgiyp"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("kEjxjufyatvluhbPhkcbjxdwhyhowbDgiyp", PH) ;
            p322.param_count = (ushort)(ushort)64990;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p322.param_id_SET("ivalcylvau", PH) ;
            p322.param_index = (ushort)(ushort)36843;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 67);
                Debug.Assert(pack.param_value_TRY(ph).Equals("yqxpvyzrytftowewzhThnodhhzgpxnabiTmkMwlfkExlsooafndhdonevceujoAyfpa"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("mc"));
                Debug.Assert(pack.target_component == (byte)(byte)188);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("mc", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.param_value_SET("yqxpvyzrytftowewzhThnodhhzgpxnabiTmkMwlfkExlsooafndhdonevceujoAyfpa", PH) ;
            p323.target_system = (byte)(byte)58;
            p323.target_component = (byte)(byte)188;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.param_value_LEN(ph) == 41);
                Debug.Assert(pack.param_value_TRY(ph).Equals("iMbpbIhessrthnkjlsbevDvnurpdivlhiKHgyfsig"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vDoqm"));
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("iMbpbIhessrthnkjlsbevDvnurpdivlhiKHgyfsig", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_id_SET("vDoqm", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)30449, (ushort)36198, (ushort)9264, (ushort)56698, (ushort)28797, (ushort)50783, (ushort)2868, (ushort)29017, (ushort)15293, (ushort)25945, (ushort)41819, (ushort)36865, (ushort)34500, (ushort)64000, (ushort)59276, (ushort)58265, (ushort)23639, (ushort)205, (ushort)49440, (ushort)47637, (ushort)49524, (ushort)32282, (ushort)43090, (ushort)9202, (ushort)25812, (ushort)62354, (ushort)64701, (ushort)39795, (ushort)20635, (ushort)64073, (ushort)23378, (ushort)63884, (ushort)16089, (ushort)3281, (ushort)15812, (ushort)19752, (ushort)51752, (ushort)7246, (ushort)29788, (ushort)26509, (ushort)47667, (ushort)13578, (ushort)62999, (ushort)43302, (ushort)43145, (ushort)37984, (ushort)53199, (ushort)29721, (ushort)58687, (ushort)50183, (ushort)1014, (ushort)58351, (ushort)1426, (ushort)54966, (ushort)44374, (ushort)23629, (ushort)29985, (ushort)52399, (ushort)9597, (ushort)45201, (ushort)64308, (ushort)8953, (ushort)49416, (ushort)48890, (ushort)42078, (ushort)10935, (ushort)59434, (ushort)6564, (ushort)59839, (ushort)48093, (ushort)51562, (ushort)44964}));
                Debug.Assert(pack.max_distance == (ushort)(ushort)10409);
                Debug.Assert(pack.time_usec == (ulong)7456743241539504354L);
                Debug.Assert(pack.increment == (byte)(byte)53);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.min_distance == (ushort)(ushort)24818);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)10409;
            p330.distances_SET(new ushort[] {(ushort)30449, (ushort)36198, (ushort)9264, (ushort)56698, (ushort)28797, (ushort)50783, (ushort)2868, (ushort)29017, (ushort)15293, (ushort)25945, (ushort)41819, (ushort)36865, (ushort)34500, (ushort)64000, (ushort)59276, (ushort)58265, (ushort)23639, (ushort)205, (ushort)49440, (ushort)47637, (ushort)49524, (ushort)32282, (ushort)43090, (ushort)9202, (ushort)25812, (ushort)62354, (ushort)64701, (ushort)39795, (ushort)20635, (ushort)64073, (ushort)23378, (ushort)63884, (ushort)16089, (ushort)3281, (ushort)15812, (ushort)19752, (ushort)51752, (ushort)7246, (ushort)29788, (ushort)26509, (ushort)47667, (ushort)13578, (ushort)62999, (ushort)43302, (ushort)43145, (ushort)37984, (ushort)53199, (ushort)29721, (ushort)58687, (ushort)50183, (ushort)1014, (ushort)58351, (ushort)1426, (ushort)54966, (ushort)44374, (ushort)23629, (ushort)29985, (ushort)52399, (ushort)9597, (ushort)45201, (ushort)64308, (ushort)8953, (ushort)49416, (ushort)48890, (ushort)42078, (ushort)10935, (ushort)59434, (ushort)6564, (ushort)59839, (ushort)48093, (ushort)51562, (ushort)44964}, 0) ;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.increment = (byte)(byte)53;
            p330.min_distance = (ushort)(ushort)24818;
            p330.time_usec = (ulong)7456743241539504354L;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stallSpeed == (ushort)(ushort)25364);
                Debug.Assert(pack.aircraftSize == (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M);
                Debug.Assert(pack.rfSelect == (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
                Debug.Assert(pack.emitterType == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR);
                Debug.Assert(pack.gpsOffsetLat == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M);
                Debug.Assert(pack.ICAO == (uint)2376493913U);
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("w"));
                Debug.Assert(pack.gpsOffsetLon == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
            };
            DemoDevice.UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.ICAO = (uint)2376493913U;
            p10001.rfSelect = (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY;
            p10001.stallSpeed = (ushort)(ushort)25364;
            p10001.gpsOffsetLon = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR;
            p10001.emitterType = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR;
            p10001.callsign_SET("w", PH) ;
            p10001.aircraftSize = (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M;
            p10001.gpsOffsetLat = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M;
            LoopBackDemoChannel.instance.send(p10001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.velVert == (short)(short)24841);
                Debug.Assert(pack.accuracyHor == (uint)4273864710U);
                Debug.Assert(pack.VelEW == (short)(short) -31800);
                Debug.Assert(pack.utcTime == (uint)114101651U);
                Debug.Assert(pack.state == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT);
                Debug.Assert(pack.emergencyStatus == (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
                Debug.Assert(pack.gpsLat == (int) -92517740);
                Debug.Assert(pack.numSats == (byte)(byte)13);
                Debug.Assert(pack.gpsAlt == (int)1884764221);
                Debug.Assert(pack.accuracyVert == (ushort)(ushort)56239);
                Debug.Assert(pack.squawk == (ushort)(ushort)13563);
                Debug.Assert(pack.baroAltMSL == (int)295615834);
                Debug.Assert(pack.velNS == (short)(short)5151);
                Debug.Assert(pack.gpsLon == (int)820195335);
                Debug.Assert(pack.gpsFix == (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS);
                Debug.Assert(pack.accuracyVel == (ushort)(ushort)52586);
            };
            DemoDevice.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.baroAltMSL = (int)295615834;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
            p10002.gpsFix = (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS;
            p10002.gpsLat = (int) -92517740;
            p10002.squawk = (ushort)(ushort)13563;
            p10002.accuracyHor = (uint)4273864710U;
            p10002.emergencyStatus = (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY;
            p10002.utcTime = (uint)114101651U;
            p10002.accuracyVel = (ushort)(ushort)52586;
            p10002.VelEW = (short)(short) -31800;
            p10002.accuracyVert = (ushort)(ushort)56239;
            p10002.numSats = (byte)(byte)13;
            p10002.gpsLon = (int)820195335;
            p10002.velNS = (short)(short)5151;
            p10002.velVert = (short)(short)24841;
            p10002.gpsAlt = (int)1884764221;
            LoopBackDemoChannel.instance.send(p10002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rfHealth == (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING);
            };
            DemoDevice.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
            LoopBackDemoChannel.instance.send(p10003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEVICE_OP_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)12);
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.address == (byte)(byte)160);
                Debug.Assert(pack.regstart == (byte)(byte)189);
                Debug.Assert(pack.request_id == (uint)3987086948U);
                Debug.Assert(pack.busname_LEN(ph) == 27);
                Debug.Assert(pack.busname_TRY(ph).Equals("eygGiuxwmklwjbEtlrijrrlmowi"));
                Debug.Assert(pack.bustype == (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
                Debug.Assert(pack.bus == (byte)(byte)116);
                Debug.Assert(pack.target_component == (byte)(byte)61);
            };
            DemoDevice.DEVICE_OP_READ p11000 = LoopBackDemoChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.busname_SET("eygGiuxwmklwjbEtlrijrrlmowi", PH) ;
            p11000.address = (byte)(byte)160;
            p11000.target_component = (byte)(byte)61;
            p11000.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11000.request_id = (uint)3987086948U;
            p11000.bus = (byte)(byte)116;
            p11000.target_system = (byte)(byte)53;
            p11000.regstart = (byte)(byte)189;
            p11000.count = (byte)(byte)12;
            LoopBackDemoChannel.instance.send(p11000);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEVICE_OP_READ_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.regstart == (byte)(byte)116);
                Debug.Assert(pack.count == (byte)(byte)5);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)43, (byte)110, (byte)245, (byte)6, (byte)216, (byte)251, (byte)36, (byte)80, (byte)8, (byte)46, (byte)232, (byte)189, (byte)225, (byte)204, (byte)175, (byte)6, (byte)55, (byte)110, (byte)205, (byte)230, (byte)208, (byte)12, (byte)160, (byte)56, (byte)102, (byte)91, (byte)142, (byte)165, (byte)109, (byte)235, (byte)58, (byte)152, (byte)9, (byte)189, (byte)25, (byte)209, (byte)8, (byte)204, (byte)237, (byte)1, (byte)74, (byte)3, (byte)173, (byte)58, (byte)199, (byte)214, (byte)241, (byte)46, (byte)74, (byte)88, (byte)49, (byte)239, (byte)101, (byte)79, (byte)192, (byte)74, (byte)104, (byte)210, (byte)53, (byte)123, (byte)119, (byte)40, (byte)209, (byte)48, (byte)91, (byte)178, (byte)118, (byte)1, (byte)116, (byte)234, (byte)108, (byte)33, (byte)61, (byte)133, (byte)142, (byte)242, (byte)110, (byte)138, (byte)227, (byte)236, (byte)176, (byte)5, (byte)180, (byte)48, (byte)99, (byte)35, (byte)170, (byte)29, (byte)20, (byte)238, (byte)219, (byte)178, (byte)10, (byte)19, (byte)159, (byte)43, (byte)46, (byte)207, (byte)247, (byte)119, (byte)39, (byte)221, (byte)229, (byte)15, (byte)37, (byte)195, (byte)229, (byte)244, (byte)25, (byte)173, (byte)50, (byte)156, (byte)75, (byte)22, (byte)171, (byte)251, (byte)58, (byte)239, (byte)100, (byte)159, (byte)72, (byte)98, (byte)138, (byte)99, (byte)151, (byte)193, (byte)217, (byte)188}));
                Debug.Assert(pack.request_id == (uint)4001814917U);
                Debug.Assert(pack.result == (byte)(byte)64);
            };
            DemoDevice.DEVICE_OP_READ_REPLY p11001 = LoopBackDemoChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.request_id = (uint)4001814917U;
            p11001.data__SET(new byte[] {(byte)43, (byte)110, (byte)245, (byte)6, (byte)216, (byte)251, (byte)36, (byte)80, (byte)8, (byte)46, (byte)232, (byte)189, (byte)225, (byte)204, (byte)175, (byte)6, (byte)55, (byte)110, (byte)205, (byte)230, (byte)208, (byte)12, (byte)160, (byte)56, (byte)102, (byte)91, (byte)142, (byte)165, (byte)109, (byte)235, (byte)58, (byte)152, (byte)9, (byte)189, (byte)25, (byte)209, (byte)8, (byte)204, (byte)237, (byte)1, (byte)74, (byte)3, (byte)173, (byte)58, (byte)199, (byte)214, (byte)241, (byte)46, (byte)74, (byte)88, (byte)49, (byte)239, (byte)101, (byte)79, (byte)192, (byte)74, (byte)104, (byte)210, (byte)53, (byte)123, (byte)119, (byte)40, (byte)209, (byte)48, (byte)91, (byte)178, (byte)118, (byte)1, (byte)116, (byte)234, (byte)108, (byte)33, (byte)61, (byte)133, (byte)142, (byte)242, (byte)110, (byte)138, (byte)227, (byte)236, (byte)176, (byte)5, (byte)180, (byte)48, (byte)99, (byte)35, (byte)170, (byte)29, (byte)20, (byte)238, (byte)219, (byte)178, (byte)10, (byte)19, (byte)159, (byte)43, (byte)46, (byte)207, (byte)247, (byte)119, (byte)39, (byte)221, (byte)229, (byte)15, (byte)37, (byte)195, (byte)229, (byte)244, (byte)25, (byte)173, (byte)50, (byte)156, (byte)75, (byte)22, (byte)171, (byte)251, (byte)58, (byte)239, (byte)100, (byte)159, (byte)72, (byte)98, (byte)138, (byte)99, (byte)151, (byte)193, (byte)217, (byte)188}, 0) ;
            p11001.result = (byte)(byte)64;
            p11001.regstart = (byte)(byte)116;
            p11001.count = (byte)(byte)5;
            LoopBackDemoChannel.instance.send(p11001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEVICE_OP_WRITEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)30);
                Debug.Assert(pack.busname_LEN(ph) == 29);
                Debug.Assert(pack.busname_TRY(ph).Equals("JznJrfshfNkzrxcaeiscfvjMhimxa"));
                Debug.Assert(pack.bustype == (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
                Debug.Assert(pack.bus == (byte)(byte)160);
                Debug.Assert(pack.request_id == (uint)1470636179U);
                Debug.Assert(pack.target_system == (byte)(byte)55);
                Debug.Assert(pack.target_component == (byte)(byte)96);
                Debug.Assert(pack.regstart == (byte)(byte)62);
                Debug.Assert(pack.address == (byte)(byte)144);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)203, (byte)190, (byte)10, (byte)196, (byte)36, (byte)84, (byte)76, (byte)166, (byte)166, (byte)249, (byte)149, (byte)182, (byte)221, (byte)99, (byte)189, (byte)171, (byte)92, (byte)183, (byte)112, (byte)13, (byte)169, (byte)114, (byte)166, (byte)120, (byte)253, (byte)7, (byte)12, (byte)1, (byte)122, (byte)121, (byte)35, (byte)12, (byte)94, (byte)88, (byte)82, (byte)162, (byte)41, (byte)216, (byte)177, (byte)94, (byte)147, (byte)76, (byte)46, (byte)27, (byte)185, (byte)89, (byte)149, (byte)16, (byte)182, (byte)135, (byte)224, (byte)44, (byte)220, (byte)112, (byte)255, (byte)44, (byte)150, (byte)155, (byte)9, (byte)212, (byte)124, (byte)247, (byte)11, (byte)110, (byte)104, (byte)36, (byte)68, (byte)162, (byte)183, (byte)95, (byte)85, (byte)129, (byte)64, (byte)221, (byte)14, (byte)175, (byte)240, (byte)226, (byte)26, (byte)31, (byte)189, (byte)41, (byte)236, (byte)205, (byte)84, (byte)36, (byte)175, (byte)108, (byte)49, (byte)114, (byte)101, (byte)69, (byte)70, (byte)196, (byte)157, (byte)135, (byte)156, (byte)187, (byte)110, (byte)88, (byte)168, (byte)48, (byte)248, (byte)126, (byte)139, (byte)18, (byte)64, (byte)60, (byte)58, (byte)162, (byte)59, (byte)207, (byte)97, (byte)119, (byte)98, (byte)126, (byte)130, (byte)71, (byte)165, (byte)58, (byte)139, (byte)210, (byte)167, (byte)171, (byte)220, (byte)83, (byte)220, (byte)212}));
            };
            DemoDevice.DEVICE_OP_WRITE p11002 = LoopBackDemoChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.data__SET(new byte[] {(byte)203, (byte)190, (byte)10, (byte)196, (byte)36, (byte)84, (byte)76, (byte)166, (byte)166, (byte)249, (byte)149, (byte)182, (byte)221, (byte)99, (byte)189, (byte)171, (byte)92, (byte)183, (byte)112, (byte)13, (byte)169, (byte)114, (byte)166, (byte)120, (byte)253, (byte)7, (byte)12, (byte)1, (byte)122, (byte)121, (byte)35, (byte)12, (byte)94, (byte)88, (byte)82, (byte)162, (byte)41, (byte)216, (byte)177, (byte)94, (byte)147, (byte)76, (byte)46, (byte)27, (byte)185, (byte)89, (byte)149, (byte)16, (byte)182, (byte)135, (byte)224, (byte)44, (byte)220, (byte)112, (byte)255, (byte)44, (byte)150, (byte)155, (byte)9, (byte)212, (byte)124, (byte)247, (byte)11, (byte)110, (byte)104, (byte)36, (byte)68, (byte)162, (byte)183, (byte)95, (byte)85, (byte)129, (byte)64, (byte)221, (byte)14, (byte)175, (byte)240, (byte)226, (byte)26, (byte)31, (byte)189, (byte)41, (byte)236, (byte)205, (byte)84, (byte)36, (byte)175, (byte)108, (byte)49, (byte)114, (byte)101, (byte)69, (byte)70, (byte)196, (byte)157, (byte)135, (byte)156, (byte)187, (byte)110, (byte)88, (byte)168, (byte)48, (byte)248, (byte)126, (byte)139, (byte)18, (byte)64, (byte)60, (byte)58, (byte)162, (byte)59, (byte)207, (byte)97, (byte)119, (byte)98, (byte)126, (byte)130, (byte)71, (byte)165, (byte)58, (byte)139, (byte)210, (byte)167, (byte)171, (byte)220, (byte)83, (byte)220, (byte)212}, 0) ;
            p11002.bus = (byte)(byte)160;
            p11002.target_system = (byte)(byte)55;
            p11002.address = (byte)(byte)144;
            p11002.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            p11002.regstart = (byte)(byte)62;
            p11002.request_id = (uint)1470636179U;
            p11002.busname_SET("JznJrfshfNkzrxcaeiscfvjMhimxa", PH) ;
            p11002.target_component = (byte)(byte)96;
            p11002.count = (byte)(byte)30;
            LoopBackDemoChannel.instance.send(p11002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEVICE_OP_WRITE_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (byte)(byte)197);
                Debug.Assert(pack.request_id == (uint)1297737434U);
            };
            DemoDevice.DEVICE_OP_WRITE_REPLY p11003 = LoopBackDemoChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.result = (byte)(byte)197;
            p11003.request_id = (uint)1297737434U;
            LoopBackDemoChannel.instance.send(p11003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADAP_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sigma == (float) -4.059327E37F);
                Debug.Assert(pack.theta == (float) -2.8197744E38F);
                Debug.Assert(pack.axis == (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_LANDING);
                Debug.Assert(pack.sigma_dot == (float) -1.1820934E38F);
                Debug.Assert(pack.achieved == (float) -1.8744332E38F);
                Debug.Assert(pack.f_dot == (float)2.5484992E38F);
                Debug.Assert(pack.omega_dot == (float)3.1744533E38F);
                Debug.Assert(pack.f == (float) -1.898421E38F);
                Debug.Assert(pack.u == (float) -2.2690341E38F);
                Debug.Assert(pack.desired == (float)1.6816295E38F);
                Debug.Assert(pack.omega == (float)2.492127E38F);
                Debug.Assert(pack.theta_dot == (float) -9.7592376E36F);
                Debug.Assert(pack.error == (float) -2.429136E38F);
            };
            DemoDevice.ADAP_TUNING p11010 = LoopBackDemoChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.achieved = (float) -1.8744332E38F;
            p11010.u = (float) -2.2690341E38F;
            p11010.sigma = (float) -4.059327E37F;
            p11010.sigma_dot = (float) -1.1820934E38F;
            p11010.theta = (float) -2.8197744E38F;
            p11010.error = (float) -2.429136E38F;
            p11010.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_LANDING;
            p11010.omega_dot = (float)3.1744533E38F;
            p11010.desired = (float)1.6816295E38F;
            p11010.omega = (float)2.492127E38F;
            p11010.f_dot = (float)2.5484992E38F;
            p11010.f = (float) -1.898421E38F;
            p11010.theta_dot = (float) -9.7592376E36F;
            LoopBackDemoChannel.instance.send(p11010);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_DELTAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)980238031564020112L);
                Debug.Assert(pack.angle_delta.SequenceEqual(new float[] {9.158147E37F, 2.2125624E38F, 1.4180798E37F}));
                Debug.Assert(pack.confidence == (float)3.3719202E38F);
                Debug.Assert(pack.time_delta_usec == (ulong)4338179973756388121L);
                Debug.Assert(pack.position_delta.SequenceEqual(new float[] {2.0510583E38F, 1.0060442E38F, 4.744803E37F}));
            };
            DemoDevice.VISION_POSITION_DELTA p11011 = LoopBackDemoChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.time_usec = (ulong)980238031564020112L;
            p11011.confidence = (float)3.3719202E38F;
            p11011.time_delta_usec = (ulong)4338179973756388121L;
            p11011.position_delta_SET(new float[] {2.0510583E38F, 1.0060442E38F, 4.744803E37F}, 0) ;
            p11011.angle_delta_SET(new float[] {9.158147E37F, 2.2125624E38F, 1.4180798E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p11011);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
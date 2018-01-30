
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
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_AIRSHIP);
                Debug.Assert(pack.mavlink_version == (byte)(byte)75);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
                Debug.Assert(pack.custom_mode == (uint)3151303442U);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_PX4);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_AIRSHIP;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_PX4;
            p0.custom_mode = (uint)3151303442U;
            p0.mavlink_version = (byte)(byte)75;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)40071);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)1759);
                Debug.Assert(pack.load == (ushort)(ushort)63795);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)64218);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)23);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.current_battery == (short)(short)12952);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)15735);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)11431);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)12382);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)23270);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.current_battery = (short)(short)12952;
            p1.drop_rate_comm = (ushort)(ushort)23270;
            p1.errors_count1 = (ushort)(ushort)11431;
            p1.errors_count2 = (ushort)(ushort)12382;
            p1.errors_count3 = (ushort)(ushort)1759;
            p1.voltage_battery = (ushort)(ushort)40071;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
            p1.errors_comm = (ushort)(ushort)64218;
            p1.errors_count4 = (ushort)(ushort)15735;
            p1.battery_remaining = (sbyte)(sbyte)23;
            p1.load = (ushort)(ushort)63795;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2154329593U);
                Debug.Assert(pack.time_unix_usec == (ulong)6953526059969241176L);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)6953526059969241176L;
            p2.time_boot_ms = (uint)2154329593U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.4110059E38F);
                Debug.Assert(pack.afy == (float)2.9689304E38F);
                Debug.Assert(pack.x == (float) -3.1148214E38F);
                Debug.Assert(pack.z == (float) -1.7906155E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1362215300U);
                Debug.Assert(pack.vz == (float)9.771152E37F);
                Debug.Assert(pack.afz == (float)5.9106663E37F);
                Debug.Assert(pack.y == (float)1.8888485E38F);
                Debug.Assert(pack.yaw == (float) -1.1663256E38F);
                Debug.Assert(pack.vy == (float) -1.7800983E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)58450);
                Debug.Assert(pack.yaw_rate == (float) -1.3990594E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.afx == (float) -1.5720534E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.x = (float) -3.1148214E38F;
            p3.time_boot_ms = (uint)1362215300U;
            p3.vy = (float) -1.7800983E38F;
            p3.yaw = (float) -1.1663256E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p3.y = (float)1.8888485E38F;
            p3.z = (float) -1.7906155E38F;
            p3.type_mask = (ushort)(ushort)58450;
            p3.afx = (float) -1.5720534E38F;
            p3.afz = (float)5.9106663E37F;
            p3.vx = (float) -1.4110059E38F;
            p3.vz = (float)9.771152E37F;
            p3.afy = (float)2.9689304E38F;
            p3.yaw_rate = (float) -1.3990594E37F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.seq == (uint)1844275445U);
                Debug.Assert(pack.time_usec == (ulong)7076861238346486622L);
                Debug.Assert(pack.target_component == (byte)(byte)16);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.seq = (uint)1844275445U;
            p4.target_component = (byte)(byte)16;
            p4.target_system = (byte)(byte)144;
            p4.time_usec = (ulong)7076861238346486622L;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)255);
                Debug.Assert(pack.passkey_LEN(ph) == 19);
                Debug.Assert(pack.passkey_TRY(ph).Equals("gyuzqoOousddsmtneax"));
                Debug.Assert(pack.version == (byte)(byte)169);
                Debug.Assert(pack.control_request == (byte)(byte)90);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)255;
            p5.control_request = (byte)(byte)90;
            p5.passkey_SET("gyuzqoOousddsmtneax", PH) ;
            p5.version = (byte)(byte)169;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)34);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)93);
                Debug.Assert(pack.control_request == (byte)(byte)243);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)243;
            p6.gcs_system_id = (byte)(byte)93;
            p6.ack = (byte)(byte)34;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 5);
                Debug.Assert(pack.key_TRY(ph).Equals("ycOez"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("ycOez", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)2706310046U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)152;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p11.custom_mode = (uint)2706310046U;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.param_index == (short)(short)21565);
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("uhqq"));
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short)21565;
            p20.target_system = (byte)(byte)191;
            p20.param_id_SET("uhqq", PH) ;
            p20.target_component = (byte)(byte)244;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)54);
                Debug.Assert(pack.target_system == (byte)(byte)204);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)204;
            p21.target_component = (byte)(byte)54;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_count == (ushort)(ushort)10559);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("owehkgib"));
                Debug.Assert(pack.param_index == (ushort)(ushort)47343);
                Debug.Assert(pack.param_value == (float)1.4561579E37F);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("owehkgib", PH) ;
            p22.param_count = (ushort)(ushort)10559;
            p22.param_value = (float)1.4561579E37F;
            p22.param_index = (ushort)(ushort)47343;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)235);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("tjo"));
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.param_value == (float) -1.6623771E38F);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_component = (byte)(byte)242;
            p23.param_value = (float) -1.6623771E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.param_id_SET("tjo", PH) ;
            p23.target_system = (byte)(byte)235;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.eph == (ushort)(ushort)8616);
                Debug.Assert(pack.vel == (ushort)(ushort)8903);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)391109463U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)5);
                Debug.Assert(pack.time_usec == (ulong)6399128461621584848L);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2527007457U);
                Debug.Assert(pack.lon == (int) -1607680459);
                Debug.Assert(pack.alt == (int)2047384049);
                Debug.Assert(pack.lat == (int) -1048645889);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)637173337U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)93569423);
                Debug.Assert(pack.cog == (ushort)(ushort)31780);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)736249589U);
                Debug.Assert(pack.epv == (ushort)(ushort)19625);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.eph = (ushort)(ushort)8616;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p24.lon = (int) -1607680459;
            p24.vel_acc_SET((uint)637173337U, PH) ;
            p24.alt = (int)2047384049;
            p24.epv = (ushort)(ushort)19625;
            p24.lat = (int) -1048645889;
            p24.v_acc_SET((uint)2527007457U, PH) ;
            p24.satellites_visible = (byte)(byte)5;
            p24.time_usec = (ulong)6399128461621584848L;
            p24.h_acc_SET((uint)391109463U, PH) ;
            p24.cog = (ushort)(ushort)31780;
            p24.vel = (ushort)(ushort)8903;
            p24.hdg_acc_SET((uint)736249589U, PH) ;
            p24.alt_ellipsoid_SET((int)93569423, PH) ;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)180, (byte)231, (byte)177, (byte)124, (byte)78, (byte)189, (byte)242, (byte)62, (byte)213, (byte)164, (byte)215, (byte)135, (byte)161, (byte)182, (byte)209, (byte)187, (byte)194, (byte)203, (byte)185, (byte)84}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)70, (byte)194, (byte)213, (byte)19, (byte)246, (byte)127, (byte)155, (byte)135, (byte)105, (byte)31, (byte)92, (byte)177, (byte)139, (byte)60, (byte)29, (byte)101, (byte)253, (byte)238, (byte)35, (byte)233}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)245, (byte)140, (byte)43, (byte)186, (byte)79, (byte)63, (byte)221, (byte)101, (byte)133, (byte)117, (byte)16, (byte)132, (byte)207, (byte)180, (byte)5, (byte)220, (byte)69, (byte)52, (byte)114, (byte)183}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)25, (byte)129, (byte)234, (byte)24, (byte)67, (byte)255, (byte)54, (byte)155, (byte)153, (byte)239, (byte)2, (byte)227, (byte)232, (byte)118, (byte)112, (byte)152, (byte)245, (byte)72, (byte)78, (byte)129}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)7);
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)237, (byte)46, (byte)82, (byte)200, (byte)10, (byte)114, (byte)198, (byte)132, (byte)232, (byte)150, (byte)254, (byte)190, (byte)161, (byte)56, (byte)131, (byte)180, (byte)235, (byte)206, (byte)245, (byte)249}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)180, (byte)231, (byte)177, (byte)124, (byte)78, (byte)189, (byte)242, (byte)62, (byte)213, (byte)164, (byte)215, (byte)135, (byte)161, (byte)182, (byte)209, (byte)187, (byte)194, (byte)203, (byte)185, (byte)84}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)70, (byte)194, (byte)213, (byte)19, (byte)246, (byte)127, (byte)155, (byte)135, (byte)105, (byte)31, (byte)92, (byte)177, (byte)139, (byte)60, (byte)29, (byte)101, (byte)253, (byte)238, (byte)35, (byte)233}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)25, (byte)129, (byte)234, (byte)24, (byte)67, (byte)255, (byte)54, (byte)155, (byte)153, (byte)239, (byte)2, (byte)227, (byte)232, (byte)118, (byte)112, (byte)152, (byte)245, (byte)72, (byte)78, (byte)129}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)245, (byte)140, (byte)43, (byte)186, (byte)79, (byte)63, (byte)221, (byte)101, (byte)133, (byte)117, (byte)16, (byte)132, (byte)207, (byte)180, (byte)5, (byte)220, (byte)69, (byte)52, (byte)114, (byte)183}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)237, (byte)46, (byte)82, (byte)200, (byte)10, (byte)114, (byte)198, (byte)132, (byte)232, (byte)150, (byte)254, (byte)190, (byte)161, (byte)56, (byte)131, (byte)180, (byte)235, (byte)206, (byte)245, (byte)249}, 0) ;
            p25.satellites_visible = (byte)(byte)7;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)3756);
                Debug.Assert(pack.ygyro == (short)(short) -8184);
                Debug.Assert(pack.zacc == (short)(short) -20638);
                Debug.Assert(pack.ymag == (short)(short) -20289);
                Debug.Assert(pack.yacc == (short)(short) -162);
                Debug.Assert(pack.xacc == (short)(short)19283);
                Debug.Assert(pack.xmag == (short)(short) -26571);
                Debug.Assert(pack.zgyro == (short)(short)17181);
                Debug.Assert(pack.time_boot_ms == (uint)1998339481U);
                Debug.Assert(pack.xgyro == (short)(short)15768);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.zmag = (short)(short)3756;
            p26.zacc = (short)(short) -20638;
            p26.xgyro = (short)(short)15768;
            p26.xacc = (short)(short)19283;
            p26.time_boot_ms = (uint)1998339481U;
            p26.ymag = (short)(short) -20289;
            p26.zgyro = (short)(short)17181;
            p26.yacc = (short)(short) -162;
            p26.xmag = (short)(short) -26571;
            p26.ygyro = (short)(short) -8184;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -3988);
                Debug.Assert(pack.time_usec == (ulong)3759240695226789553L);
                Debug.Assert(pack.xmag == (short)(short) -6319);
                Debug.Assert(pack.zacc == (short)(short) -8271);
                Debug.Assert(pack.zgyro == (short)(short) -17296);
                Debug.Assert(pack.yacc == (short)(short)26714);
                Debug.Assert(pack.xgyro == (short)(short)31408);
                Debug.Assert(pack.zmag == (short)(short)9313);
                Debug.Assert(pack.xacc == (short)(short) -4565);
                Debug.Assert(pack.ymag == (short)(short)475);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.xacc = (short)(short) -4565;
            p27.zacc = (short)(short) -8271;
            p27.time_usec = (ulong)3759240695226789553L;
            p27.ymag = (short)(short)475;
            p27.ygyro = (short)(short) -3988;
            p27.zmag = (short)(short)9313;
            p27.zgyro = (short)(short) -17296;
            p27.yacc = (short)(short)26714;
            p27.xgyro = (short)(short)31408;
            p27.xmag = (short)(short) -6319;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short) -23229);
                Debug.Assert(pack.press_abs == (short)(short) -13250);
                Debug.Assert(pack.temperature == (short)(short)32169);
                Debug.Assert(pack.time_usec == (ulong)3220195320283713396L);
                Debug.Assert(pack.press_diff1 == (short)(short) -30985);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short) -23229;
            p28.press_diff1 = (short)(short) -30985;
            p28.temperature = (short)(short)32169;
            p28.time_usec = (ulong)3220195320283713396L;
            p28.press_abs = (short)(short) -13250;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -3.562846E36F);
                Debug.Assert(pack.time_boot_ms == (uint)3072147585U);
                Debug.Assert(pack.temperature == (short)(short) -31862);
                Debug.Assert(pack.press_diff == (float) -2.3389328E38F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short) -31862;
            p29.press_abs = (float) -3.562846E36F;
            p29.time_boot_ms = (uint)3072147585U;
            p29.press_diff = (float) -2.3389328E38F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)668191990U);
                Debug.Assert(pack.rollspeed == (float)1.3358394E38F);
                Debug.Assert(pack.roll == (float) -2.7207984E38F);
                Debug.Assert(pack.pitch == (float) -6.876684E37F);
                Debug.Assert(pack.yaw == (float)1.6122179E38F);
                Debug.Assert(pack.yawspeed == (float) -5.325154E37F);
                Debug.Assert(pack.pitchspeed == (float) -2.5665824E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.rollspeed = (float)1.3358394E38F;
            p30.yawspeed = (float) -5.325154E37F;
            p30.yaw = (float)1.6122179E38F;
            p30.pitch = (float) -6.876684E37F;
            p30.roll = (float) -2.7207984E38F;
            p30.pitchspeed = (float) -2.5665824E38F;
            p30.time_boot_ms = (uint)668191990U;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)797897944U);
                Debug.Assert(pack.rollspeed == (float)1.8724254E38F);
                Debug.Assert(pack.q1 == (float)2.049766E38F);
                Debug.Assert(pack.yawspeed == (float) -1.797053E38F);
                Debug.Assert(pack.q3 == (float) -1.1154056E38F);
                Debug.Assert(pack.pitchspeed == (float) -3.1575412E38F);
                Debug.Assert(pack.q2 == (float)3.1129026E38F);
                Debug.Assert(pack.q4 == (float) -3.0875218E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -3.1575412E38F;
            p31.q1 = (float)2.049766E38F;
            p31.rollspeed = (float)1.8724254E38F;
            p31.yawspeed = (float) -1.797053E38F;
            p31.q2 = (float)3.1129026E38F;
            p31.time_boot_ms = (uint)797897944U;
            p31.q3 = (float) -1.1154056E38F;
            p31.q4 = (float) -3.0875218E38F;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.625099E37F);
                Debug.Assert(pack.vz == (float)2.969704E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1550835806U);
                Debug.Assert(pack.vy == (float)1.2845693E38F);
                Debug.Assert(pack.z == (float)1.7381302E37F);
                Debug.Assert(pack.y == (float)1.6269331E38F);
                Debug.Assert(pack.vx == (float)9.976764E37F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.z = (float)1.7381302E37F;
            p32.y = (float)1.6269331E38F;
            p32.vy = (float)1.2845693E38F;
            p32.vz = (float)2.969704E38F;
            p32.time_boot_ms = (uint)1550835806U;
            p32.x = (float)3.625099E37F;
            p32.vx = (float)9.976764E37F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int) -1079428295);
                Debug.Assert(pack.lon == (int) -569490621);
                Debug.Assert(pack.time_boot_ms == (uint)2809245472U);
                Debug.Assert(pack.alt == (int) -2107564436);
                Debug.Assert(pack.lat == (int) -2041576426);
                Debug.Assert(pack.vy == (short)(short) -13292);
                Debug.Assert(pack.vx == (short)(short)7035);
                Debug.Assert(pack.vz == (short)(short) -30087);
                Debug.Assert(pack.hdg == (ushort)(ushort)37971);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short)7035;
            p33.relative_alt = (int) -1079428295;
            p33.lat = (int) -2041576426;
            p33.vy = (short)(short) -13292;
            p33.hdg = (ushort)(ushort)37971;
            p33.lon = (int) -569490621;
            p33.alt = (int) -2107564436;
            p33.vz = (short)(short) -30087;
            p33.time_boot_ms = (uint)2809245472U;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_scaled == (short)(short) -7035);
                Debug.Assert(pack.chan4_scaled == (short)(short) -13891);
                Debug.Assert(pack.chan5_scaled == (short)(short) -15850);
                Debug.Assert(pack.rssi == (byte)(byte)119);
                Debug.Assert(pack.chan8_scaled == (short)(short) -29223);
                Debug.Assert(pack.time_boot_ms == (uint)1939733128U);
                Debug.Assert(pack.port == (byte)(byte)97);
                Debug.Assert(pack.chan6_scaled == (short)(short)11366);
                Debug.Assert(pack.chan1_scaled == (short)(short) -15794);
                Debug.Assert(pack.chan2_scaled == (short)(short) -14708);
                Debug.Assert(pack.chan3_scaled == (short)(short) -21382);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short) -29223;
            p34.port = (byte)(byte)97;
            p34.chan4_scaled = (short)(short) -13891;
            p34.chan6_scaled = (short)(short)11366;
            p34.chan7_scaled = (short)(short) -7035;
            p34.chan2_scaled = (short)(short) -14708;
            p34.chan3_scaled = (short)(short) -21382;
            p34.rssi = (byte)(byte)119;
            p34.chan1_scaled = (short)(short) -15794;
            p34.time_boot_ms = (uint)1939733128U;
            p34.chan5_scaled = (short)(short) -15850;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)20);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)27479);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)40022);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)44211);
                Debug.Assert(pack.port == (byte)(byte)231);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)44925);
                Debug.Assert(pack.time_boot_ms == (uint)2515079710U);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)45529);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)65040);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)55271);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)52859);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.rssi = (byte)(byte)20;
            p35.time_boot_ms = (uint)2515079710U;
            p35.chan6_raw = (ushort)(ushort)55271;
            p35.chan7_raw = (ushort)(ushort)40022;
            p35.port = (byte)(byte)231;
            p35.chan5_raw = (ushort)(ushort)45529;
            p35.chan8_raw = (ushort)(ushort)65040;
            p35.chan1_raw = (ushort)(ushort)27479;
            p35.chan4_raw = (ushort)(ushort)52859;
            p35.chan3_raw = (ushort)(ushort)44925;
            p35.chan2_raw = (ushort)(ushort)44211;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)49766);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)51144);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)57660);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)8805);
                Debug.Assert(pack.time_usec == (uint)1914970974U);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)39935);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)18735);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)14794);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)27339);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)17814);
                Debug.Assert(pack.port == (byte)(byte)109);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)44487);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)63172);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)60140);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)52977);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)32244);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)5597);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)9969);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo7_raw = (ushort)(ushort)49766;
            p36.servo14_raw_SET((ushort)(ushort)8805, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)39935, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)27339, PH) ;
            p36.port = (byte)(byte)109;
            p36.servo1_raw = (ushort)(ushort)32244;
            p36.servo2_raw = (ushort)(ushort)57660;
            p36.servo15_raw_SET((ushort)(ushort)51144, PH) ;
            p36.time_usec = (uint)1914970974U;
            p36.servo10_raw_SET((ushort)(ushort)18735, PH) ;
            p36.servo8_raw = (ushort)(ushort)14794;
            p36.servo3_raw = (ushort)(ushort)9969;
            p36.servo16_raw_SET((ushort)(ushort)63172, PH) ;
            p36.servo6_raw = (ushort)(ushort)17814;
            p36.servo11_raw_SET((ushort)(ushort)5597, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)44487, PH) ;
            p36.servo5_raw = (ushort)(ushort)60140;
            p36.servo4_raw = (ushort)(ushort)52977;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.end_index == (short)(short)8354);
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.start_index == (short)(short)25179);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short)25179;
            p37.end_index = (short)(short)8354;
            p37.target_system = (byte)(byte)25;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.target_component = (byte)(byte)21;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)12565);
                Debug.Assert(pack.target_component == (byte)(byte)204);
                Debug.Assert(pack.start_index == (short)(short) -14216);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)243);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.target_system = (byte)(byte)243;
            p38.start_index = (short)(short) -14216;
            p38.end_index = (short)(short)12565;
            p38.target_component = (byte)(byte)204;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.4027947E38F);
                Debug.Assert(pack.z == (float)7.291836E37F);
                Debug.Assert(pack.param4 == (float) -3.1067798E38F);
                Debug.Assert(pack.param2 == (float)2.1752292E38F);
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.x == (float)3.3479977E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SET_CAMERA_MODE);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.target_system == (byte)(byte)127);
                Debug.Assert(pack.autocontinue == (byte)(byte)86);
                Debug.Assert(pack.y == (float)1.4065931E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)3941);
                Debug.Assert(pack.current == (byte)(byte)249);
                Debug.Assert(pack.param3 == (float)2.5621965E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.param1 = (float)1.4027947E38F;
            p39.seq = (ushort)(ushort)3941;
            p39.target_component = (byte)(byte)132;
            p39.y = (float)1.4065931E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p39.target_system = (byte)(byte)127;
            p39.autocontinue = (byte)(byte)86;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_CAMERA_MODE;
            p39.param2 = (float)2.1752292E38F;
            p39.param3 = (float)2.5621965E38F;
            p39.param4 = (float) -3.1067798E38F;
            p39.z = (float)7.291836E37F;
            p39.current = (byte)(byte)249;
            p39.x = (float)3.3479977E38F;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.target_component == (byte)(byte)222);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)54712);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)222;
            p40.seq = (ushort)(ushort)54712;
            p40.target_system = (byte)(byte)183;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)202);
                Debug.Assert(pack.seq == (ushort)(ushort)54254);
                Debug.Assert(pack.target_component == (byte)(byte)0);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)54254;
            p41.target_system = (byte)(byte)202;
            p41.target_component = (byte)(byte)0;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)42012);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)42012;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)31);
                Debug.Assert(pack.target_system == (byte)(byte)176);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_system = (byte)(byte)176;
            p43.target_component = (byte)(byte)31;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)3745);
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_system = (byte)(byte)108;
            p44.count = (ushort)(ushort)3745;
            p44.target_component = (byte)(byte)229;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.target_system == (byte)(byte)208);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)208;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)132;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)21940);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)21940;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2);
                Debug.Assert(pack.target_system == (byte)(byte)106);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2;
            p47.target_component = (byte)(byte)121;
            p47.target_system = (byte)(byte)106;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.altitude == (int)992350735);
                Debug.Assert(pack.latitude == (int)277780078);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6804754685194228996L);
                Debug.Assert(pack.longitude == (int) -929593242);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)6804754685194228996L, PH) ;
            p48.longitude = (int) -929593242;
            p48.latitude = (int)277780078;
            p48.altitude = (int)992350735;
            p48.target_system = (byte)(byte)133;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5178424514263014447L);
                Debug.Assert(pack.altitude == (int)1265202563);
                Debug.Assert(pack.longitude == (int) -1018795516);
                Debug.Assert(pack.latitude == (int)1560692136);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)5178424514263014447L, PH) ;
            p49.latitude = (int)1560692136;
            p49.altitude = (int)1265202563;
            p49.longitude = (int) -1018795516;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)11879);
                Debug.Assert(pack.param_value_max == (float) -1.5382348E38F);
                Debug.Assert(pack.param_value0 == (float)3.419405E37F);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)166);
                Debug.Assert(pack.param_value_min == (float)5.0232004E37F);
                Debug.Assert(pack.scale == (float)2.5992358E38F);
                Debug.Assert(pack.target_component == (byte)(byte)209);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bmgdrw"));
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_min = (float)5.0232004E37F;
            p50.param_id_SET("bmgdrw", PH) ;
            p50.parameter_rc_channel_index = (byte)(byte)166;
            p50.param_value_max = (float) -1.5382348E38F;
            p50.param_value0 = (float)3.419405E37F;
            p50.scale = (float)2.5992358E38F;
            p50.target_component = (byte)(byte)209;
            p50.param_index = (short)(short)11879;
            p50.target_system = (byte)(byte)150;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)14596);
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.target_component == (byte)(byte)106);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)112;
            p51.seq = (ushort)(ushort)14596;
            p51.target_component = (byte)(byte)106;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.p2y == (float) -2.1687638E38F);
                Debug.Assert(pack.p2x == (float) -1.902638E38F);
                Debug.Assert(pack.p2z == (float)4.5046943E37F);
                Debug.Assert(pack.target_component == (byte)(byte)176);
                Debug.Assert(pack.p1y == (float)8.953964E37F);
                Debug.Assert(pack.p1x == (float) -1.3291844E37F);
                Debug.Assert(pack.p1z == (float)2.482781E38F);
                Debug.Assert(pack.target_system == (byte)(byte)151);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float)4.5046943E37F;
            p54.p1y = (float)8.953964E37F;
            p54.p1z = (float)2.482781E38F;
            p54.p2x = (float) -1.902638E38F;
            p54.target_system = (byte)(byte)151;
            p54.p2y = (float) -2.1687638E38F;
            p54.target_component = (byte)(byte)176;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p54.p1x = (float) -1.3291844E37F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p2x == (float) -2.0030433E38F);
                Debug.Assert(pack.p2y == (float)1.2444439E38F);
                Debug.Assert(pack.p1x == (float)2.8531024E38F);
                Debug.Assert(pack.p2z == (float) -1.3666821E37F);
                Debug.Assert(pack.p1y == (float)1.1881523E38F);
                Debug.Assert(pack.p1z == (float)3.2248595E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float)1.2444439E38F;
            p55.p2z = (float) -1.3666821E37F;
            p55.p1z = (float)3.2248595E38F;
            p55.p1x = (float)2.8531024E38F;
            p55.p2x = (float) -2.0030433E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p55.p1y = (float)1.1881523E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9683513E38F, 8.774697E37F, 3.2024412E38F, 6.9987623E37F}));
                Debug.Assert(pack.time_usec == (ulong)1743925086512505836L);
                Debug.Assert(pack.yawspeed == (float) -1.3539888E38F);
                Debug.Assert(pack.rollspeed == (float) -1.2047137E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {4.4405263E37F, -1.108501E38F, 1.8828245E37F, -2.8005728E38F, -8.697611E37F, -2.3955424E38F, 1.909163E38F, 4.9960103E37F, 1.9125304E38F}));
                Debug.Assert(pack.pitchspeed == (float)2.1649832E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)1743925086512505836L;
            p61.rollspeed = (float) -1.2047137E38F;
            p61.covariance_SET(new float[] {4.4405263E37F, -1.108501E38F, 1.8828245E37F, -2.8005728E38F, -8.697611E37F, -2.3955424E38F, 1.909163E38F, 4.9960103E37F, 1.9125304E38F}, 0) ;
            p61.q_SET(new float[] {2.9683513E38F, 8.774697E37F, 3.2024412E38F, 6.9987623E37F}, 0) ;
            p61.pitchspeed = (float)2.1649832E37F;
            p61.yawspeed = (float) -1.3539888E38F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float) -3.386368E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -5671);
                Debug.Assert(pack.nav_bearing == (short)(short)9562);
                Debug.Assert(pack.aspd_error == (float)2.6301251E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)7013);
                Debug.Assert(pack.nav_roll == (float)1.6253872E38F);
                Debug.Assert(pack.xtrack_error == (float) -3.2181285E37F);
                Debug.Assert(pack.alt_error == (float) -3.1746693E38F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.xtrack_error = (float) -3.2181285E37F;
            p62.nav_bearing = (short)(short)9562;
            p62.nav_roll = (float)1.6253872E38F;
            p62.wp_dist = (ushort)(ushort)7013;
            p62.nav_pitch = (float) -3.386368E38F;
            p62.alt_error = (float) -3.1746693E38F;
            p62.aspd_error = (float)2.6301251E38F;
            p62.target_bearing = (short)(short) -5671;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -521332453);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.696905E38F, -1.9919385E38F, -2.9739736E38F, 1.8487175E38F, 1.3224251E38F, -2.4794309E38F, -2.1977542E38F, -1.9744498E38F, -1.6751323E38F, -4.4636183E36F, -6.0737973E37F, 6.2895904E37F, 6.453685E37F, 1.4235283E37F, 3.0199832E38F, -2.0032267E38F, -3.3244163E38F, -1.3142356E38F, 1.2768631E38F, -2.5679563E38F, -3.0478945E38F, 2.3134903E38F, 2.3171028E38F, 1.6981118E38F, -1.2374588E38F, -7.991983E37F, 1.0739531E38F, 3.0435512E38F, 2.5499623E38F, -1.1336867E38F, -1.5391231E38F, 7.343723E37F, -2.1863182E38F, -3.1499216E37F, -2.217154E38F, -8.966559E37F}));
                Debug.Assert(pack.vx == (float) -6.2377633E37F);
                Debug.Assert(pack.vz == (float) -8.923755E37F);
                Debug.Assert(pack.alt == (int) -556280257);
                Debug.Assert(pack.vy == (float) -1.9649323E38F);
                Debug.Assert(pack.lon == (int) -439095220);
                Debug.Assert(pack.time_usec == (ulong)6005343752159054157L);
                Debug.Assert(pack.relative_alt == (int) -2046997715);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lat = (int) -521332453;
            p63.lon = (int) -439095220;
            p63.vx = (float) -6.2377633E37F;
            p63.vy = (float) -1.9649323E38F;
            p63.time_usec = (ulong)6005343752159054157L;
            p63.relative_alt = (int) -2046997715;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.alt = (int) -556280257;
            p63.vz = (float) -8.923755E37F;
            p63.covariance_SET(new float[] {2.696905E38F, -1.9919385E38F, -2.9739736E38F, 1.8487175E38F, 1.3224251E38F, -2.4794309E38F, -2.1977542E38F, -1.9744498E38F, -1.6751323E38F, -4.4636183E36F, -6.0737973E37F, 6.2895904E37F, 6.453685E37F, 1.4235283E37F, 3.0199832E38F, -2.0032267E38F, -3.3244163E38F, -1.3142356E38F, 1.2768631E38F, -2.5679563E38F, -3.0478945E38F, 2.3134903E38F, 2.3171028E38F, 1.6981118E38F, -1.2374588E38F, -7.991983E37F, 1.0739531E38F, 3.0435512E38F, 2.5499623E38F, -1.1336867E38F, -1.5391231E38F, 7.343723E37F, -2.1863182E38F, -3.1499216E37F, -2.217154E38F, -8.966559E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {8.1282816E37F, -9.466378E37F, -1.7711265E38F, -1.3134539E37F, -1.8475527E38F, -2.0880808E38F, -1.4961307E38F, 2.3671134E38F, -6.4827205E37F, -3.180916E38F, 2.1334018E38F, 2.0179292E38F, 1.1660171E38F, -2.2317896E37F, 1.7408279E38F, -2.6036967E38F, -1.9271183E38F, 1.7953127E38F, 1.0853442E38F, -1.7334801E38F, 1.9148974E38F, -5.345768E37F, -2.0224094E38F, 1.0657721E38F, -2.2638704E38F, -9.705453E37F, -2.9554166E38F, 7.2925695E36F, -1.8451896E38F, -7.2895325E37F, -8.922983E37F, 2.8030272E38F, -2.2315384E38F, -7.5558096E37F, 6.6914524E37F, -7.043921E37F, 2.3673046E38F, 3.362109E38F, -2.254537E37F, -2.82402E38F, -1.5386876E37F, -3.045768E38F, -1.798457E38F, -1.4463748E38F, -5.755894E37F}));
                Debug.Assert(pack.vy == (float) -1.3076652E38F);
                Debug.Assert(pack.vz == (float) -1.9211365E38F);
                Debug.Assert(pack.x == (float)1.4677694E38F);
                Debug.Assert(pack.ay == (float) -1.422334E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.time_usec == (ulong)7275492492603973640L);
                Debug.Assert(pack.vx == (float) -5.125771E37F);
                Debug.Assert(pack.az == (float)2.3849244E38F);
                Debug.Assert(pack.ax == (float) -2.378168E38F);
                Debug.Assert(pack.z == (float) -1.0511379E38F);
                Debug.Assert(pack.y == (float) -2.0168875E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vy = (float) -1.3076652E38F;
            p64.time_usec = (ulong)7275492492603973640L;
            p64.covariance_SET(new float[] {8.1282816E37F, -9.466378E37F, -1.7711265E38F, -1.3134539E37F, -1.8475527E38F, -2.0880808E38F, -1.4961307E38F, 2.3671134E38F, -6.4827205E37F, -3.180916E38F, 2.1334018E38F, 2.0179292E38F, 1.1660171E38F, -2.2317896E37F, 1.7408279E38F, -2.6036967E38F, -1.9271183E38F, 1.7953127E38F, 1.0853442E38F, -1.7334801E38F, 1.9148974E38F, -5.345768E37F, -2.0224094E38F, 1.0657721E38F, -2.2638704E38F, -9.705453E37F, -2.9554166E38F, 7.2925695E36F, -1.8451896E38F, -7.2895325E37F, -8.922983E37F, 2.8030272E38F, -2.2315384E38F, -7.5558096E37F, 6.6914524E37F, -7.043921E37F, 2.3673046E38F, 3.362109E38F, -2.254537E37F, -2.82402E38F, -1.5386876E37F, -3.045768E38F, -1.798457E38F, -1.4463748E38F, -5.755894E37F}, 0) ;
            p64.z = (float) -1.0511379E38F;
            p64.x = (float)1.4677694E38F;
            p64.az = (float)2.3849244E38F;
            p64.vz = (float) -1.9211365E38F;
            p64.ax = (float) -2.378168E38F;
            p64.y = (float) -2.0168875E38F;
            p64.ay = (float) -1.422334E38F;
            p64.vx = (float) -5.125771E37F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)46415);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)20911);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)34040);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)53432);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)44824);
                Debug.Assert(pack.time_boot_ms == (uint)1259366665U);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)39895);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)55713);
                Debug.Assert(pack.chancount == (byte)(byte)197);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)7157);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)11845);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)9169);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)40589);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)59317);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)56614);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)35375);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)24153);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)6125);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)50273);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51534);
                Debug.Assert(pack.rssi == (byte)(byte)217);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan18_raw = (ushort)(ushort)55713;
            p65.chan12_raw = (ushort)(ushort)6125;
            p65.chan4_raw = (ushort)(ushort)35375;
            p65.chan10_raw = (ushort)(ushort)39895;
            p65.time_boot_ms = (uint)1259366665U;
            p65.chan3_raw = (ushort)(ushort)59317;
            p65.chan16_raw = (ushort)(ushort)7157;
            p65.chan13_raw = (ushort)(ushort)24153;
            p65.chan9_raw = (ushort)(ushort)34040;
            p65.chan8_raw = (ushort)(ushort)40589;
            p65.chan5_raw = (ushort)(ushort)56614;
            p65.chan6_raw = (ushort)(ushort)53432;
            p65.chan14_raw = (ushort)(ushort)20911;
            p65.chan2_raw = (ushort)(ushort)51534;
            p65.rssi = (byte)(byte)217;
            p65.chan7_raw = (ushort)(ushort)11845;
            p65.chancount = (byte)(byte)197;
            p65.chan11_raw = (ushort)(ushort)9169;
            p65.chan17_raw = (ushort)(ushort)44824;
            p65.chan1_raw = (ushort)(ushort)50273;
            p65.chan15_raw = (ushort)(ushort)46415;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)97);
                Debug.Assert(pack.req_stream_id == (byte)(byte)245);
                Debug.Assert(pack.target_component == (byte)(byte)125);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)40761);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)136;
            p66.start_stop = (byte)(byte)97;
            p66.req_stream_id = (byte)(byte)245;
            p66.target_component = (byte)(byte)125;
            p66.req_message_rate = (ushort)(ushort)40761;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)93);
                Debug.Assert(pack.stream_id == (byte)(byte)215);
                Debug.Assert(pack.message_rate == (ushort)(ushort)9558);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)93;
            p67.message_rate = (ushort)(ushort)9558;
            p67.stream_id = (byte)(byte)215;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.r == (short)(short) -25137);
                Debug.Assert(pack.buttons == (ushort)(ushort)45125);
                Debug.Assert(pack.x == (short)(short) -9272);
                Debug.Assert(pack.y == (short)(short) -10704);
                Debug.Assert(pack.z == (short)(short)22963);
                Debug.Assert(pack.target == (byte)(byte)147);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)45125;
            p69.y = (short)(short) -10704;
            p69.x = (short)(short) -9272;
            p69.z = (short)(short)22963;
            p69.target = (byte)(byte)147;
            p69.r = (short)(short) -25137;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)24420);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)27888);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)29501);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)11549);
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)16329);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)58452);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)9280);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)37281);
                Debug.Assert(pack.target_system == (byte)(byte)114);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan5_raw = (ushort)(ushort)27888;
            p70.target_system = (byte)(byte)114;
            p70.chan3_raw = (ushort)(ushort)16329;
            p70.chan1_raw = (ushort)(ushort)37281;
            p70.chan6_raw = (ushort)(ushort)58452;
            p70.chan4_raw = (ushort)(ushort)11549;
            p70.chan2_raw = (ushort)(ushort)9280;
            p70.target_component = (byte)(byte)218;
            p70.chan8_raw = (ushort)(ushort)29501;
            p70.chan7_raw = (ushort)(ushort)24420;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -1.4522084E38F);
                Debug.Assert(pack.x == (int)1347978139);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM);
                Debug.Assert(pack.param1 == (float) -2.6544477E37F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.current == (byte)(byte)195);
                Debug.Assert(pack.seq == (ushort)(ushort)23083);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.param4 == (float) -1.3010897E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)8);
                Debug.Assert(pack.param2 == (float)2.5462478E38F);
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.y == (int)889488802);
                Debug.Assert(pack.z == (float) -1.6374908E38F);
                Debug.Assert(pack.target_component == (byte)(byte)54);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.seq = (ushort)(ushort)23083;
            p73.target_component = (byte)(byte)54;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.param1 = (float) -2.6544477E37F;
            p73.target_system = (byte)(byte)33;
            p73.param2 = (float)2.5462478E38F;
            p73.y = (int)889488802;
            p73.current = (byte)(byte)195;
            p73.param3 = (float) -1.4522084E38F;
            p73.x = (int)1347978139;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
            p73.param4 = (float) -1.3010897E38F;
            p73.autocontinue = (byte)(byte)8;
            p73.z = (float) -1.6374908E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float)1.7162973E38F);
                Debug.Assert(pack.heading == (short)(short) -15149);
                Debug.Assert(pack.throttle == (ushort)(ushort)7779);
                Debug.Assert(pack.alt == (float) -2.9818255E38F);
                Debug.Assert(pack.groundspeed == (float) -1.2756224E38F);
                Debug.Assert(pack.airspeed == (float)8.650253E37F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.groundspeed = (float) -1.2756224E38F;
            p74.throttle = (ushort)(ushort)7779;
            p74.airspeed = (float)8.650253E37F;
            p74.alt = (float) -2.9818255E38F;
            p74.climb = (float)1.7162973E38F;
            p74.heading = (short)(short) -15149;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.param2 == (float)3.2552016E38F);
                Debug.Assert(pack.param1 == (float) -2.6095588E38F);
                Debug.Assert(pack.z == (float) -1.4249819E38F);
                Debug.Assert(pack.param4 == (float)2.1984008E38F);
                Debug.Assert(pack.y == (int) -383912637);
                Debug.Assert(pack.param3 == (float)1.3924905E38F);
                Debug.Assert(pack.current == (byte)(byte)173);
                Debug.Assert(pack.x == (int) -1722159686);
                Debug.Assert(pack.autocontinue == (byte)(byte)35);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param2 = (float)3.2552016E38F;
            p75.param1 = (float) -2.6095588E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            p75.z = (float) -1.4249819E38F;
            p75.y = (int) -383912637;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p75.param3 = (float)1.3924905E38F;
            p75.x = (int) -1722159686;
            p75.target_component = (byte)(byte)107;
            p75.current = (byte)(byte)173;
            p75.param4 = (float)2.1984008E38F;
            p75.autocontinue = (byte)(byte)35;
            p75.target_system = (byte)(byte)45;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.confirmation == (byte)(byte)133);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
                Debug.Assert(pack.param1 == (float)2.7817739E38F);
                Debug.Assert(pack.param7 == (float) -3.965953E37F);
                Debug.Assert(pack.param3 == (float)2.8597524E38F);
                Debug.Assert(pack.param2 == (float) -1.8794243E38F);
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.target_system == (byte)(byte)100);
                Debug.Assert(pack.param6 == (float)6.872303E37F);
                Debug.Assert(pack.param4 == (float) -2.7078173E38F);
                Debug.Assert(pack.param5 == (float)6.3767566E37F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float)6.872303E37F;
            p76.confirmation = (byte)(byte)133;
            p76.target_system = (byte)(byte)100;
            p76.param7 = (float) -3.965953E37F;
            p76.param4 = (float) -2.7078173E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
            p76.target_component = (byte)(byte)36;
            p76.param5 = (float)6.3767566E37F;
            p76.param3 = (float)2.8597524E38F;
            p76.param1 = (float)2.7817739E38F;
            p76.param2 = (float) -1.8794243E38F;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1925649164);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)195);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)48);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)83);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SET_CAMERA_MODE);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)48, PH) ;
            p77.target_system_SET((byte)(byte)195, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_CAMERA_MODE;
            p77.progress_SET((byte)(byte)83, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.result_param2_SET((int)1925649164, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)274714478U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)33);
                Debug.Assert(pack.pitch == (float)2.8077155E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)243);
                Debug.Assert(pack.yaw == (float)7.504725E37F);
                Debug.Assert(pack.thrust == (float)2.6550776E38F);
                Debug.Assert(pack.roll == (float) -1.9496882E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)7.504725E37F;
            p81.pitch = (float)2.8077155E38F;
            p81.manual_override_switch = (byte)(byte)33;
            p81.thrust = (float)2.6550776E38F;
            p81.time_boot_ms = (uint)274714478U;
            p81.mode_switch = (byte)(byte)243;
            p81.roll = (float) -1.9496882E38F;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)1.1761502E38F);
                Debug.Assert(pack.thrust == (float) -1.7032478E38F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.1267582E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1238913391U);
                Debug.Assert(pack.target_system == (byte)(byte)2);
                Debug.Assert(pack.type_mask == (byte)(byte)184);
                Debug.Assert(pack.body_pitch_rate == (float)1.9928183E38F);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.2250423E38F, 2.3997587E38F, -1.876868E38F, 1.1477031E38F}));
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float) -1.1267582E38F;
            p82.body_roll_rate = (float)1.1761502E38F;
            p82.target_system = (byte)(byte)2;
            p82.q_SET(new float[] {-3.2250423E38F, 2.3997587E38F, -1.876868E38F, 1.1477031E38F}, 0) ;
            p82.body_pitch_rate = (float)1.9928183E38F;
            p82.thrust = (float) -1.7032478E38F;
            p82.target_component = (byte)(byte)135;
            p82.type_mask = (byte)(byte)184;
            p82.time_boot_ms = (uint)1238913391U;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float) -1.05909514E37F);
                Debug.Assert(pack.body_pitch_rate == (float)1.8384473E37F);
                Debug.Assert(pack.thrust == (float) -6.845223E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)203);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.9802185E37F, 2.9846053E38F, -2.833479E38F, -2.574105E38F}));
                Debug.Assert(pack.body_yaw_rate == (float) -2.4331385E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3791687867U);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.type_mask = (byte)(byte)203;
            p83.q_SET(new float[] {3.9802185E37F, 2.9846053E38F, -2.833479E38F, -2.574105E38F}, 0) ;
            p83.body_roll_rate = (float) -1.05909514E37F;
            p83.thrust = (float) -6.845223E37F;
            p83.body_yaw_rate = (float) -2.4331385E38F;
            p83.body_pitch_rate = (float)1.8384473E37F;
            p83.time_boot_ms = (uint)3791687867U;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.8416461E38F);
                Debug.Assert(pack.vz == (float) -2.786155E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.9253939E37F);
                Debug.Assert(pack.x == (float)2.9512153E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.y == (float) -3.0307366E37F);
                Debug.Assert(pack.vy == (float) -2.2525409E38F);
                Debug.Assert(pack.time_boot_ms == (uint)607136595U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)24887);
                Debug.Assert(pack.yaw == (float)1.5193304E38F);
                Debug.Assert(pack.afy == (float)1.2550672E38F);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.afx == (float) -2.0543982E38F);
                Debug.Assert(pack.vx == (float) -1.3183592E38F);
                Debug.Assert(pack.target_component == (byte)(byte)222);
                Debug.Assert(pack.afz == (float)7.750456E37F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afx = (float) -2.0543982E38F;
            p84.yaw_rate = (float) -1.9253939E37F;
            p84.type_mask = (ushort)(ushort)24887;
            p84.x = (float)2.9512153E37F;
            p84.vy = (float) -2.2525409E38F;
            p84.target_component = (byte)(byte)222;
            p84.afz = (float)7.750456E37F;
            p84.y = (float) -3.0307366E37F;
            p84.z = (float) -2.8416461E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p84.target_system = (byte)(byte)222;
            p84.yaw = (float)1.5193304E38F;
            p84.time_boot_ms = (uint)607136595U;
            p84.vz = (float) -2.786155E37F;
            p84.afy = (float)1.2550672E38F;
            p84.vx = (float) -1.3183592E38F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)55003);
                Debug.Assert(pack.yaw_rate == (float)1.0177136E38F);
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.yaw == (float) -2.8879156E38F);
                Debug.Assert(pack.vz == (float)1.9951362E38F);
                Debug.Assert(pack.alt == (float)5.6699075E37F);
                Debug.Assert(pack.vy == (float)1.7847871E38F);
                Debug.Assert(pack.lat_int == (int) -806989619);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.afy == (float) -2.663643E38F);
                Debug.Assert(pack.lon_int == (int) -1193041459);
                Debug.Assert(pack.time_boot_ms == (uint)2984599369U);
                Debug.Assert(pack.vx == (float) -3.3350054E38F);
                Debug.Assert(pack.afx == (float)2.4437336E38F);
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.afz == (float)1.8284103E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.target_system = (byte)(byte)21;
            p86.lat_int = (int) -806989619;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.alt = (float)5.6699075E37F;
            p86.vy = (float)1.7847871E38F;
            p86.afz = (float)1.8284103E38F;
            p86.vz = (float)1.9951362E38F;
            p86.lon_int = (int) -1193041459;
            p86.time_boot_ms = (uint)2984599369U;
            p86.target_component = (byte)(byte)134;
            p86.vx = (float) -3.3350054E38F;
            p86.type_mask = (ushort)(ushort)55003;
            p86.yaw = (float) -2.8879156E38F;
            p86.yaw_rate = (float)1.0177136E38F;
            p86.afy = (float) -2.663643E38F;
            p86.afx = (float)2.4437336E38F;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)2.9421402E38F);
                Debug.Assert(pack.vx == (float)1.0291673E38F);
                Debug.Assert(pack.afx == (float) -2.2592935E38F);
                Debug.Assert(pack.afz == (float)6.7045706E37F);
                Debug.Assert(pack.lon_int == (int)2004799866);
                Debug.Assert(pack.type_mask == (ushort)(ushort)4857);
                Debug.Assert(pack.afy == (float)2.7539285E38F);
                Debug.Assert(pack.vy == (float) -1.3929208E38F);
                Debug.Assert(pack.alt == (float) -2.1872236E38F);
                Debug.Assert(pack.yaw_rate == (float) -7.8377533E37F);
                Debug.Assert(pack.yaw == (float) -3.115553E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1918601849U);
                Debug.Assert(pack.lat_int == (int) -414350296);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afx = (float) -2.2592935E38F;
            p87.type_mask = (ushort)(ushort)4857;
            p87.lat_int = (int) -414350296;
            p87.yaw_rate = (float) -7.8377533E37F;
            p87.yaw = (float) -3.115553E38F;
            p87.afy = (float)2.7539285E38F;
            p87.vz = (float)2.9421402E38F;
            p87.vy = (float) -1.3929208E38F;
            p87.time_boot_ms = (uint)1918601849U;
            p87.lon_int = (int)2004799866;
            p87.afz = (float)6.7045706E37F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.alt = (float) -2.1872236E38F;
            p87.vx = (float)1.0291673E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.3707117E38F);
                Debug.Assert(pack.x == (float)1.4876046E38F);
                Debug.Assert(pack.roll == (float) -2.5568271E38F);
                Debug.Assert(pack.y == (float)3.1103057E38F);
                Debug.Assert(pack.yaw == (float)3.067816E38F);
                Debug.Assert(pack.z == (float)3.188526E37F);
                Debug.Assert(pack.time_boot_ms == (uint)968337655U);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.roll = (float) -2.5568271E38F;
            p89.yaw = (float)3.067816E38F;
            p89.y = (float)3.1103057E38F;
            p89.x = (float)1.4876046E38F;
            p89.z = (float)3.188526E37F;
            p89.time_boot_ms = (uint)968337655U;
            p89.pitch = (float)3.3707117E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)7414);
                Debug.Assert(pack.lat == (int)561182147);
                Debug.Assert(pack.pitch == (float) -3.2588956E38F);
                Debug.Assert(pack.yaw == (float) -3.0725853E38F);
                Debug.Assert(pack.vz == (short)(short) -24040);
                Debug.Assert(pack.rollspeed == (float) -1.8906471E38F);
                Debug.Assert(pack.lon == (int) -1193307781);
                Debug.Assert(pack.yawspeed == (float)2.4277442E38F);
                Debug.Assert(pack.vx == (short)(short) -11760);
                Debug.Assert(pack.zacc == (short)(short)30731);
                Debug.Assert(pack.vy == (short)(short) -20567);
                Debug.Assert(pack.xacc == (short)(short) -14453);
                Debug.Assert(pack.alt == (int) -1875116856);
                Debug.Assert(pack.time_usec == (ulong)1896077463254684796L);
                Debug.Assert(pack.roll == (float)7.8613053E37F);
                Debug.Assert(pack.pitchspeed == (float)2.1355536E38F);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yawspeed = (float)2.4277442E38F;
            p90.pitchspeed = (float)2.1355536E38F;
            p90.zacc = (short)(short)30731;
            p90.vx = (short)(short) -11760;
            p90.alt = (int) -1875116856;
            p90.pitch = (float) -3.2588956E38F;
            p90.lat = (int)561182147;
            p90.xacc = (short)(short) -14453;
            p90.roll = (float)7.8613053E37F;
            p90.rollspeed = (float) -1.8906471E38F;
            p90.lon = (int) -1193307781;
            p90.yaw = (float) -3.0725853E38F;
            p90.vy = (short)(short) -20567;
            p90.vz = (short)(short) -24040;
            p90.yacc = (short)(short)7414;
            p90.time_usec = (ulong)1896077463254684796L;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_ailerons == (float) -3.3384613E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.aux1 == (float)2.557955E38F);
                Debug.Assert(pack.aux4 == (float)3.3898683E38F);
                Debug.Assert(pack.time_usec == (ulong)9188881441188346040L);
                Debug.Assert(pack.yaw_rudder == (float)8.733749E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)38);
                Debug.Assert(pack.throttle == (float) -2.40624E38F);
                Debug.Assert(pack.aux2 == (float) -2.8773584E38F);
                Debug.Assert(pack.aux3 == (float)1.4034152E38F);
                Debug.Assert(pack.pitch_elevator == (float)4.8899465E37F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.nav_mode = (byte)(byte)38;
            p91.roll_ailerons = (float) -3.3384613E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.aux4 = (float)3.3898683E38F;
            p91.yaw_rudder = (float)8.733749E37F;
            p91.aux3 = (float)1.4034152E38F;
            p91.throttle = (float) -2.40624E38F;
            p91.aux1 = (float)2.557955E38F;
            p91.time_usec = (ulong)9188881441188346040L;
            p91.aux2 = (float) -2.8773584E38F;
            p91.pitch_elevator = (float)4.8899465E37F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)291);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)39391);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)42633);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)53010);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)57815);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)41917);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)38502);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)41080);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)39208);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)38587);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)37005);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)5869);
                Debug.Assert(pack.time_usec == (ulong)7595237184806393282L);
                Debug.Assert(pack.rssi == (byte)(byte)46);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)7595237184806393282L;
            p92.chan12_raw = (ushort)(ushort)39391;
            p92.chan3_raw = (ushort)(ushort)37005;
            p92.chan4_raw = (ushort)(ushort)42633;
            p92.chan7_raw = (ushort)(ushort)5869;
            p92.chan11_raw = (ushort)(ushort)291;
            p92.chan6_raw = (ushort)(ushort)53010;
            p92.rssi = (byte)(byte)46;
            p92.chan1_raw = (ushort)(ushort)39208;
            p92.chan10_raw = (ushort)(ushort)41917;
            p92.chan5_raw = (ushort)(ushort)57815;
            p92.chan9_raw = (ushort)(ushort)38587;
            p92.chan2_raw = (ushort)(ushort)38502;
            p92.chan8_raw = (ushort)(ushort)41080;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)7811131126840371365L);
                Debug.Assert(pack.flags == (ulong)3386808576511956516L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-6.0208287E37F, 3.9348718E36F, 1.5814296E38F, -8.400737E37F, -5.916502E37F, 2.616983E37F, -3.3922423E38F, 1.8061872E37F, -2.7861028E37F, -6.7346773E37F, -2.8925174E38F, 1.945088E38F, 1.6956918E38F, 1.8145617E38F, 2.155931E38F, 2.7414875E38F}));
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)3386808576511956516L;
            p93.time_usec = (ulong)7811131126840371365L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.controls_SET(new float[] {-6.0208287E37F, 3.9348718E36F, 1.5814296E38F, -8.400737E37F, -5.916502E37F, 2.616983E37F, -3.3922423E38F, 1.8061872E37F, -2.7861028E37F, -6.7346773E37F, -2.8925174E38F, 1.945088E38F, 1.6956918E38F, 1.8145617E38F, 2.155931E38F, 2.7414875E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.4383301E38F);
                Debug.Assert(pack.flow_x == (short)(short)14310);
                Debug.Assert(pack.ground_distance == (float) -2.3091692E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)166);
                Debug.Assert(pack.time_usec == (ulong)8303040880587123953L);
                Debug.Assert(pack.flow_y == (short)(short) -7626);
                Debug.Assert(pack.flow_comp_m_x == (float) -3.3006914E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.7326936E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.404625E38F);
                Debug.Assert(pack.quality == (byte)(byte)57);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_y = (short)(short) -7626;
            p100.quality = (byte)(byte)57;
            p100.flow_comp_m_x = (float) -3.3006914E38F;
            p100.sensor_id = (byte)(byte)166;
            p100.flow_comp_m_y = (float) -1.7326936E38F;
            p100.time_usec = (ulong)8303040880587123953L;
            p100.ground_distance = (float) -2.3091692E38F;
            p100.flow_rate_y_SET((float)1.4383301E38F, PH) ;
            p100.flow_rate_x_SET((float) -1.404625E38F, PH) ;
            p100.flow_x = (short)(short)14310;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.1096126E38F);
                Debug.Assert(pack.yaw == (float)1.3639114E38F);
                Debug.Assert(pack.usec == (ulong)6830521704484752734L);
                Debug.Assert(pack.x == (float)2.1521103E38F);
                Debug.Assert(pack.roll == (float)8.915278E37F);
                Debug.Assert(pack.pitch == (float) -1.9462469E38F);
                Debug.Assert(pack.y == (float)2.142485E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float) -1.1096126E38F;
            p101.pitch = (float) -1.9462469E38F;
            p101.yaw = (float)1.3639114E38F;
            p101.x = (float)2.1521103E38F;
            p101.usec = (ulong)6830521704484752734L;
            p101.roll = (float)8.915278E37F;
            p101.y = (float)2.142485E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.3203235E38F);
                Debug.Assert(pack.yaw == (float) -1.4067135E38F);
                Debug.Assert(pack.roll == (float)3.0176295E38F);
                Debug.Assert(pack.pitch == (float)3.578568E37F);
                Debug.Assert(pack.usec == (ulong)4424043249952159233L);
                Debug.Assert(pack.x == (float)4.3379206E37F);
                Debug.Assert(pack.z == (float)2.4097066E38F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)4424043249952159233L;
            p102.x = (float)4.3379206E37F;
            p102.yaw = (float) -1.4067135E38F;
            p102.z = (float)2.4097066E38F;
            p102.pitch = (float)3.578568E37F;
            p102.y = (float) -2.3203235E38F;
            p102.roll = (float)3.0176295E38F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.29122E38F);
                Debug.Assert(pack.z == (float)2.378108E37F);
                Debug.Assert(pack.usec == (ulong)8243826755849822392L);
                Debug.Assert(pack.x == (float)1.8689366E38F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)2.29122E38F;
            p103.x = (float)1.8689366E38F;
            p103.z = (float)2.378108E37F;
            p103.usec = (ulong)8243826755849822392L;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.2137443E38F);
                Debug.Assert(pack.y == (float)3.323354E38F);
                Debug.Assert(pack.yaw == (float) -4.051347E37F);
                Debug.Assert(pack.usec == (ulong)2324654369044652587L);
                Debug.Assert(pack.x == (float) -2.6113805E38F);
                Debug.Assert(pack.pitch == (float) -1.3323628E38F);
                Debug.Assert(pack.z == (float)3.4596076E37F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float)2.2137443E38F;
            p104.pitch = (float) -1.3323628E38F;
            p104.x = (float) -2.6113805E38F;
            p104.usec = (ulong)2324654369044652587L;
            p104.yaw = (float) -4.051347E37F;
            p104.y = (float)3.323354E38F;
            p104.z = (float)3.4596076E37F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pressure_alt == (float) -1.3620194E38F);
                Debug.Assert(pack.temperature == (float) -2.7775797E38F);
                Debug.Assert(pack.zgyro == (float)3.2802703E38F);
                Debug.Assert(pack.zacc == (float) -1.6972359E38F);
                Debug.Assert(pack.time_usec == (ulong)4421854775123454202L);
                Debug.Assert(pack.ygyro == (float)1.6807583E38F);
                Debug.Assert(pack.xacc == (float)1.6989764E38F);
                Debug.Assert(pack.xgyro == (float) -2.735021E38F);
                Debug.Assert(pack.zmag == (float)1.9037167E38F);
                Debug.Assert(pack.abs_pressure == (float) -3.188287E37F);
                Debug.Assert(pack.xmag == (float)1.2746436E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)38602);
                Debug.Assert(pack.ymag == (float) -2.7043287E38F);
                Debug.Assert(pack.diff_pressure == (float) -5.8986787E36F);
                Debug.Assert(pack.yacc == (float) -3.3939643E38F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float) -1.6972359E38F;
            p105.fields_updated = (ushort)(ushort)38602;
            p105.zmag = (float)1.9037167E38F;
            p105.diff_pressure = (float) -5.8986787E36F;
            p105.ygyro = (float)1.6807583E38F;
            p105.xgyro = (float) -2.735021E38F;
            p105.xmag = (float)1.2746436E38F;
            p105.pressure_alt = (float) -1.3620194E38F;
            p105.yacc = (float) -3.3939643E38F;
            p105.xacc = (float)1.6989764E38F;
            p105.temperature = (float) -2.7775797E38F;
            p105.time_usec = (ulong)4421854775123454202L;
            p105.abs_pressure = (float) -3.188287E37F;
            p105.ymag = (float) -2.7043287E38F;
            p105.zgyro = (float)3.2802703E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float) -3.213501E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)552926219U);
                Debug.Assert(pack.integrated_xgyro == (float)5.7971705E37F);
                Debug.Assert(pack.integrated_ygyro == (float)3.339392E38F);
                Debug.Assert(pack.time_usec == (ulong)9054327281609574316L);
                Debug.Assert(pack.sensor_id == (byte)(byte)122);
                Debug.Assert(pack.integrated_y == (float) -2.6445128E38F);
                Debug.Assert(pack.integration_time_us == (uint)1272195535U);
                Debug.Assert(pack.distance == (float)1.3713738E37F);
                Debug.Assert(pack.integrated_x == (float) -7.5265436E37F);
                Debug.Assert(pack.quality == (byte)(byte)230);
                Debug.Assert(pack.temperature == (short)(short)32694);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float) -2.6445128E38F;
            p106.time_usec = (ulong)9054327281609574316L;
            p106.integrated_zgyro = (float) -3.213501E38F;
            p106.integrated_x = (float) -7.5265436E37F;
            p106.integrated_xgyro = (float)5.7971705E37F;
            p106.distance = (float)1.3713738E37F;
            p106.integrated_ygyro = (float)3.339392E38F;
            p106.time_delta_distance_us = (uint)552926219U;
            p106.quality = (byte)(byte)230;
            p106.sensor_id = (byte)(byte)122;
            p106.temperature = (short)(short)32694;
            p106.integration_time_us = (uint)1272195535U;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float)6.190667E36F);
                Debug.Assert(pack.zgyro == (float)3.3401103E38F);
                Debug.Assert(pack.pressure_alt == (float)3.6563754E37F);
                Debug.Assert(pack.ymag == (float) -2.0294642E37F);
                Debug.Assert(pack.time_usec == (ulong)4508990145495596840L);
                Debug.Assert(pack.abs_pressure == (float)3.2944189E38F);
                Debug.Assert(pack.xacc == (float) -2.3489965E38F);
                Debug.Assert(pack.zacc == (float)2.0690849E38F);
                Debug.Assert(pack.fields_updated == (uint)1269079664U);
                Debug.Assert(pack.xgyro == (float) -6.8582686E37F);
                Debug.Assert(pack.zmag == (float) -8.252227E37F);
                Debug.Assert(pack.temperature == (float) -2.8182923E38F);
                Debug.Assert(pack.xmag == (float) -8.0107983E37F);
                Debug.Assert(pack.ygyro == (float) -1.0394123E38F);
                Debug.Assert(pack.diff_pressure == (float) -2.3315498E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.pressure_alt = (float)3.6563754E37F;
            p107.abs_pressure = (float)3.2944189E38F;
            p107.xmag = (float) -8.0107983E37F;
            p107.diff_pressure = (float) -2.3315498E38F;
            p107.xacc = (float) -2.3489965E38F;
            p107.fields_updated = (uint)1269079664U;
            p107.xgyro = (float) -6.8582686E37F;
            p107.zmag = (float) -8.252227E37F;
            p107.ygyro = (float) -1.0394123E38F;
            p107.yacc = (float)6.190667E36F;
            p107.zacc = (float)2.0690849E38F;
            p107.ymag = (float) -2.0294642E37F;
            p107.time_usec = (ulong)4508990145495596840L;
            p107.temperature = (float) -2.8182923E38F;
            p107.zgyro = (float)3.3401103E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.4022838E38F);
                Debug.Assert(pack.lon == (float)3.3167344E38F);
                Debug.Assert(pack.q4 == (float) -1.9149793E38F);
                Debug.Assert(pack.roll == (float) -9.489655E37F);
                Debug.Assert(pack.lat == (float) -1.2734161E38F);
                Debug.Assert(pack.q1 == (float) -2.213317E38F);
                Debug.Assert(pack.zgyro == (float)2.0732561E38F);
                Debug.Assert(pack.xgyro == (float) -1.1255387E38F);
                Debug.Assert(pack.q3 == (float)9.463501E36F);
                Debug.Assert(pack.q2 == (float) -2.9514727E38F);
                Debug.Assert(pack.ygyro == (float) -2.2654701E38F);
                Debug.Assert(pack.zacc == (float)2.3800355E38F);
                Debug.Assert(pack.pitch == (float)1.0233532E38F);
                Debug.Assert(pack.vn == (float)1.8990641E38F);
                Debug.Assert(pack.yaw == (float) -2.6649433E38F);
                Debug.Assert(pack.std_dev_vert == (float)2.3355878E38F);
                Debug.Assert(pack.vd == (float)4.853467E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.1786227E38F);
                Debug.Assert(pack.ve == (float) -3.3979845E38F);
                Debug.Assert(pack.yacc == (float)2.5540302E38F);
                Debug.Assert(pack.xacc == (float)2.9921807E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.std_dev_horz = (float)2.1786227E38F;
            p108.q1 = (float) -2.213317E38F;
            p108.ygyro = (float) -2.2654701E38F;
            p108.ve = (float) -3.3979845E38F;
            p108.xacc = (float)2.9921807E38F;
            p108.q2 = (float) -2.9514727E38F;
            p108.alt = (float)1.4022838E38F;
            p108.yaw = (float) -2.6649433E38F;
            p108.lon = (float)3.3167344E38F;
            p108.q3 = (float)9.463501E36F;
            p108.lat = (float) -1.2734161E38F;
            p108.zacc = (float)2.3800355E38F;
            p108.roll = (float) -9.489655E37F;
            p108.q4 = (float) -1.9149793E38F;
            p108.yacc = (float)2.5540302E38F;
            p108.zgyro = (float)2.0732561E38F;
            p108.vd = (float)4.853467E37F;
            p108.pitch = (float)1.0233532E38F;
            p108.vn = (float)1.8990641E38F;
            p108.xgyro = (float) -1.1255387E38F;
            p108.std_dev_vert = (float)2.3355878E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)126);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)21281);
                Debug.Assert(pack.txbuf == (byte)(byte)200);
                Debug.Assert(pack.noise == (byte)(byte)56);
                Debug.Assert(pack.rssi == (byte)(byte)47);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)55007);
                Debug.Assert(pack.remnoise == (byte)(byte)33);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)56;
            p109.fixed_ = (ushort)(ushort)21281;
            p109.txbuf = (byte)(byte)200;
            p109.remnoise = (byte)(byte)33;
            p109.remrssi = (byte)(byte)126;
            p109.rxerrors = (ushort)(ushort)55007;
            p109.rssi = (byte)(byte)47;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)204, (byte)24, (byte)99, (byte)105, (byte)4, (byte)45, (byte)67, (byte)240, (byte)73, (byte)20, (byte)248, (byte)218, (byte)127, (byte)153, (byte)50, (byte)212, (byte)24, (byte)43, (byte)250, (byte)8, (byte)131, (byte)103, (byte)208, (byte)150, (byte)202, (byte)105, (byte)139, (byte)29, (byte)19, (byte)39, (byte)130, (byte)133, (byte)218, (byte)104, (byte)163, (byte)23, (byte)162, (byte)184, (byte)23, (byte)245, (byte)130, (byte)35, (byte)31, (byte)25, (byte)120, (byte)64, (byte)63, (byte)128, (byte)242, (byte)201, (byte)139, (byte)68, (byte)68, (byte)156, (byte)250, (byte)255, (byte)71, (byte)8, (byte)135, (byte)37, (byte)125, (byte)132, (byte)167, (byte)108, (byte)110, (byte)247, (byte)244, (byte)216, (byte)62, (byte)120, (byte)152, (byte)77, (byte)126, (byte)48, (byte)59, (byte)170, (byte)201, (byte)175, (byte)223, (byte)14, (byte)115, (byte)66, (byte)174, (byte)205, (byte)129, (byte)150, (byte)230, (byte)135, (byte)69, (byte)127, (byte)211, (byte)64, (byte)8, (byte)91, (byte)86, (byte)42, (byte)113, (byte)154, (byte)227, (byte)154, (byte)76, (byte)198, (byte)195, (byte)84, (byte)167, (byte)142, (byte)224, (byte)236, (byte)155, (byte)98, (byte)51, (byte)150, (byte)95, (byte)175, (byte)119, (byte)154, (byte)145, (byte)118, (byte)155, (byte)11, (byte)101, (byte)63, (byte)194, (byte)28, (byte)200, (byte)230, (byte)119, (byte)166, (byte)13, (byte)36, (byte)143, (byte)34, (byte)113, (byte)102, (byte)53, (byte)29, (byte)78, (byte)243, (byte)153, (byte)71, (byte)215, (byte)2, (byte)107, (byte)153, (byte)225, (byte)156, (byte)36, (byte)34, (byte)148, (byte)55, (byte)156, (byte)184, (byte)94, (byte)87, (byte)236, (byte)25, (byte)43, (byte)153, (byte)48, (byte)230, (byte)253, (byte)205, (byte)248, (byte)104, (byte)138, (byte)59, (byte)76, (byte)28, (byte)130, (byte)76, (byte)153, (byte)172, (byte)195, (byte)180, (byte)71, (byte)223, (byte)118, (byte)111, (byte)203, (byte)106, (byte)190, (byte)253, (byte)200, (byte)54, (byte)78, (byte)14, (byte)188, (byte)7, (byte)171, (byte)62, (byte)90, (byte)227, (byte)90, (byte)73, (byte)222, (byte)95, (byte)132, (byte)45, (byte)175, (byte)164, (byte)20, (byte)192, (byte)58, (byte)245, (byte)249, (byte)69, (byte)55, (byte)115, (byte)201, (byte)215, (byte)58, (byte)13, (byte)153, (byte)66, (byte)221, (byte)214, (byte)216, (byte)83, (byte)223, (byte)29, (byte)34, (byte)50, (byte)224, (byte)133, (byte)53, (byte)218, (byte)120, (byte)14, (byte)46, (byte)125, (byte)117, (byte)143, (byte)98, (byte)240, (byte)53, (byte)63, (byte)51, (byte)220, (byte)143, (byte)235, (byte)93, (byte)176, (byte)118, (byte)37, (byte)63, (byte)237, (byte)101, (byte)53, (byte)210, (byte)86, (byte)34}));
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.target_network == (byte)(byte)100);
                Debug.Assert(pack.target_system == (byte)(byte)36);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)36;
            p110.payload_SET(new byte[] {(byte)204, (byte)24, (byte)99, (byte)105, (byte)4, (byte)45, (byte)67, (byte)240, (byte)73, (byte)20, (byte)248, (byte)218, (byte)127, (byte)153, (byte)50, (byte)212, (byte)24, (byte)43, (byte)250, (byte)8, (byte)131, (byte)103, (byte)208, (byte)150, (byte)202, (byte)105, (byte)139, (byte)29, (byte)19, (byte)39, (byte)130, (byte)133, (byte)218, (byte)104, (byte)163, (byte)23, (byte)162, (byte)184, (byte)23, (byte)245, (byte)130, (byte)35, (byte)31, (byte)25, (byte)120, (byte)64, (byte)63, (byte)128, (byte)242, (byte)201, (byte)139, (byte)68, (byte)68, (byte)156, (byte)250, (byte)255, (byte)71, (byte)8, (byte)135, (byte)37, (byte)125, (byte)132, (byte)167, (byte)108, (byte)110, (byte)247, (byte)244, (byte)216, (byte)62, (byte)120, (byte)152, (byte)77, (byte)126, (byte)48, (byte)59, (byte)170, (byte)201, (byte)175, (byte)223, (byte)14, (byte)115, (byte)66, (byte)174, (byte)205, (byte)129, (byte)150, (byte)230, (byte)135, (byte)69, (byte)127, (byte)211, (byte)64, (byte)8, (byte)91, (byte)86, (byte)42, (byte)113, (byte)154, (byte)227, (byte)154, (byte)76, (byte)198, (byte)195, (byte)84, (byte)167, (byte)142, (byte)224, (byte)236, (byte)155, (byte)98, (byte)51, (byte)150, (byte)95, (byte)175, (byte)119, (byte)154, (byte)145, (byte)118, (byte)155, (byte)11, (byte)101, (byte)63, (byte)194, (byte)28, (byte)200, (byte)230, (byte)119, (byte)166, (byte)13, (byte)36, (byte)143, (byte)34, (byte)113, (byte)102, (byte)53, (byte)29, (byte)78, (byte)243, (byte)153, (byte)71, (byte)215, (byte)2, (byte)107, (byte)153, (byte)225, (byte)156, (byte)36, (byte)34, (byte)148, (byte)55, (byte)156, (byte)184, (byte)94, (byte)87, (byte)236, (byte)25, (byte)43, (byte)153, (byte)48, (byte)230, (byte)253, (byte)205, (byte)248, (byte)104, (byte)138, (byte)59, (byte)76, (byte)28, (byte)130, (byte)76, (byte)153, (byte)172, (byte)195, (byte)180, (byte)71, (byte)223, (byte)118, (byte)111, (byte)203, (byte)106, (byte)190, (byte)253, (byte)200, (byte)54, (byte)78, (byte)14, (byte)188, (byte)7, (byte)171, (byte)62, (byte)90, (byte)227, (byte)90, (byte)73, (byte)222, (byte)95, (byte)132, (byte)45, (byte)175, (byte)164, (byte)20, (byte)192, (byte)58, (byte)245, (byte)249, (byte)69, (byte)55, (byte)115, (byte)201, (byte)215, (byte)58, (byte)13, (byte)153, (byte)66, (byte)221, (byte)214, (byte)216, (byte)83, (byte)223, (byte)29, (byte)34, (byte)50, (byte)224, (byte)133, (byte)53, (byte)218, (byte)120, (byte)14, (byte)46, (byte)125, (byte)117, (byte)143, (byte)98, (byte)240, (byte)53, (byte)63, (byte)51, (byte)220, (byte)143, (byte)235, (byte)93, (byte)176, (byte)118, (byte)37, (byte)63, (byte)237, (byte)101, (byte)53, (byte)210, (byte)86, (byte)34}, 0) ;
            p110.target_network = (byte)(byte)100;
            p110.target_component = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)133275357750579641L);
                Debug.Assert(pack.tc1 == (long)6293476949308785576L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)133275357750579641L;
            p111.tc1 = (long)6293476949308785576L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2199201528162361764L);
                Debug.Assert(pack.seq == (uint)4058044355U);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)2199201528162361764L;
            p112.seq = (uint)4058044355U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (short)(short) -4309);
                Debug.Assert(pack.eph == (ushort)(ushort)22734);
                Debug.Assert(pack.fix_type == (byte)(byte)218);
                Debug.Assert(pack.satellites_visible == (byte)(byte)220);
                Debug.Assert(pack.lat == (int) -2053970184);
                Debug.Assert(pack.epv == (ushort)(ushort)57384);
                Debug.Assert(pack.vel == (ushort)(ushort)53221);
                Debug.Assert(pack.vd == (short)(short)3213);
                Debug.Assert(pack.lon == (int)1455362983);
                Debug.Assert(pack.ve == (short)(short)14330);
                Debug.Assert(pack.cog == (ushort)(ushort)28868);
                Debug.Assert(pack.alt == (int) -1381490360);
                Debug.Assert(pack.time_usec == (ulong)2569561367937931927L);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)3213;
            p113.epv = (ushort)(ushort)57384;
            p113.ve = (short)(short)14330;
            p113.vn = (short)(short) -4309;
            p113.alt = (int) -1381490360;
            p113.time_usec = (ulong)2569561367937931927L;
            p113.lat = (int) -2053970184;
            p113.lon = (int)1455362983;
            p113.satellites_visible = (byte)(byte)220;
            p113.eph = (ushort)(ushort)22734;
            p113.cog = (ushort)(ushort)28868;
            p113.fix_type = (byte)(byte)218;
            p113.vel = (ushort)(ushort)53221;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)246523341U);
                Debug.Assert(pack.integrated_xgyro == (float) -1.9269944E38F);
                Debug.Assert(pack.integration_time_us == (uint)292737462U);
                Debug.Assert(pack.integrated_ygyro == (float) -1.2652544E38F);
                Debug.Assert(pack.temperature == (short)(short) -5868);
                Debug.Assert(pack.sensor_id == (byte)(byte)144);
                Debug.Assert(pack.time_usec == (ulong)3557363712760455394L);
                Debug.Assert(pack.integrated_x == (float) -1.2226688E38F);
                Debug.Assert(pack.distance == (float)3.6847081E37F);
                Debug.Assert(pack.quality == (byte)(byte)74);
                Debug.Assert(pack.integrated_y == (float)5.8470556E37F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.4908324E38F);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.temperature = (short)(short) -5868;
            p114.time_delta_distance_us = (uint)246523341U;
            p114.time_usec = (ulong)3557363712760455394L;
            p114.quality = (byte)(byte)74;
            p114.distance = (float)3.6847081E37F;
            p114.integrated_x = (float) -1.2226688E38F;
            p114.integrated_xgyro = (float) -1.9269944E38F;
            p114.integration_time_us = (uint)292737462U;
            p114.integrated_zgyro = (float) -2.4908324E38F;
            p114.sensor_id = (byte)(byte)144;
            p114.integrated_ygyro = (float) -1.2652544E38F;
            p114.integrated_y = (float)5.8470556E37F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)2.863028E38F);
                Debug.Assert(pack.yawspeed == (float)3.1350573E38F);
                Debug.Assert(pack.zacc == (short)(short)24130);
                Debug.Assert(pack.vx == (short)(short)10053);
                Debug.Assert(pack.alt == (int) -1580790501);
                Debug.Assert(pack.lon == (int) -1237517030);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-6.651081E37F, 4.917771E37F, 2.1212685E38F, 3.094843E38F}));
                Debug.Assert(pack.rollspeed == (float)9.184974E37F);
                Debug.Assert(pack.vz == (short)(short)2846);
                Debug.Assert(pack.yacc == (short)(short)20162);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)63852);
                Debug.Assert(pack.vy == (short)(short)28710);
                Debug.Assert(pack.lat == (int)1317757581);
                Debug.Assert(pack.xacc == (short)(short)29161);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)32759);
                Debug.Assert(pack.time_usec == (ulong)187296152370014186L);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)187296152370014186L;
            p115.vx = (short)(short)10053;
            p115.true_airspeed = (ushort)(ushort)32759;
            p115.rollspeed = (float)9.184974E37F;
            p115.xacc = (short)(short)29161;
            p115.attitude_quaternion_SET(new float[] {-6.651081E37F, 4.917771E37F, 2.1212685E38F, 3.094843E38F}, 0) ;
            p115.ind_airspeed = (ushort)(ushort)63852;
            p115.yawspeed = (float)3.1350573E38F;
            p115.alt = (int) -1580790501;
            p115.yacc = (short)(short)20162;
            p115.vy = (short)(short)28710;
            p115.zacc = (short)(short)24130;
            p115.vz = (short)(short)2846;
            p115.lon = (int) -1237517030;
            p115.lat = (int)1317757581;
            p115.pitchspeed = (float)2.863028E38F;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -12733);
                Debug.Assert(pack.xmag == (short)(short) -31115);
                Debug.Assert(pack.zmag == (short)(short) -8799);
                Debug.Assert(pack.zgyro == (short)(short) -16043);
                Debug.Assert(pack.yacc == (short)(short)26243);
                Debug.Assert(pack.zacc == (short)(short)22426);
                Debug.Assert(pack.time_boot_ms == (uint)1738326526U);
                Debug.Assert(pack.xgyro == (short)(short)6594);
                Debug.Assert(pack.ygyro == (short)(short) -32682);
                Debug.Assert(pack.xacc == (short)(short)4535);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xmag = (short)(short) -31115;
            p116.zacc = (short)(short)22426;
            p116.xgyro = (short)(short)6594;
            p116.xacc = (short)(short)4535;
            p116.yacc = (short)(short)26243;
            p116.ygyro = (short)(short) -32682;
            p116.time_boot_ms = (uint)1738326526U;
            p116.ymag = (short)(short) -12733;
            p116.zmag = (short)(short) -8799;
            p116.zgyro = (short)(short) -16043;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.start == (ushort)(ushort)12874);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.end == (ushort)(ushort)64948);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)64948;
            p117.target_system = (byte)(byte)183;
            p117.start = (ushort)(ushort)12874;
            p117.target_component = (byte)(byte)97;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)3176);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)46935);
                Debug.Assert(pack.num_logs == (ushort)(ushort)28135);
                Debug.Assert(pack.time_utc == (uint)1442072083U);
                Debug.Assert(pack.size == (uint)1737757069U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)3176;
            p118.num_logs = (ushort)(ushort)28135;
            p118.size = (uint)1737757069U;
            p118.time_utc = (uint)1442072083U;
            p118.last_log_num = (ushort)(ushort)46935;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.ofs == (uint)3934792731U);
                Debug.Assert(pack.count == (uint)492441694U);
                Debug.Assert(pack.target_system == (byte)(byte)52);
                Debug.Assert(pack.id == (ushort)(ushort)23794);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)492441694U;
            p119.ofs = (uint)3934792731U;
            p119.target_component = (byte)(byte)229;
            p119.target_system = (byte)(byte)52;
            p119.id = (ushort)(ushort)23794;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)61265);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)32, (byte)163, (byte)220, (byte)46, (byte)193, (byte)225, (byte)221, (byte)13, (byte)110, (byte)94, (byte)18, (byte)150, (byte)217, (byte)131, (byte)191, (byte)13, (byte)25, (byte)6, (byte)230, (byte)16, (byte)11, (byte)51, (byte)66, (byte)28, (byte)223, (byte)187, (byte)238, (byte)201, (byte)84, (byte)89, (byte)32, (byte)139, (byte)132, (byte)158, (byte)197, (byte)241, (byte)19, (byte)172, (byte)84, (byte)28, (byte)190, (byte)229, (byte)135, (byte)29, (byte)95, (byte)193, (byte)148, (byte)236, (byte)176, (byte)111, (byte)136, (byte)80, (byte)172, (byte)64, (byte)211, (byte)144, (byte)90, (byte)177, (byte)198, (byte)146, (byte)203, (byte)106, (byte)198, (byte)13, (byte)145, (byte)126, (byte)252, (byte)168, (byte)149, (byte)136, (byte)78, (byte)134, (byte)56, (byte)83, (byte)7, (byte)188, (byte)248, (byte)42, (byte)87, (byte)217, (byte)129, (byte)214, (byte)10, (byte)91, (byte)21, (byte)28, (byte)121, (byte)100, (byte)192, (byte)191}));
                Debug.Assert(pack.count == (byte)(byte)153);
                Debug.Assert(pack.ofs == (uint)2001882937U);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)61265;
            p120.count = (byte)(byte)153;
            p120.ofs = (uint)2001882937U;
            p120.data__SET(new byte[] {(byte)32, (byte)163, (byte)220, (byte)46, (byte)193, (byte)225, (byte)221, (byte)13, (byte)110, (byte)94, (byte)18, (byte)150, (byte)217, (byte)131, (byte)191, (byte)13, (byte)25, (byte)6, (byte)230, (byte)16, (byte)11, (byte)51, (byte)66, (byte)28, (byte)223, (byte)187, (byte)238, (byte)201, (byte)84, (byte)89, (byte)32, (byte)139, (byte)132, (byte)158, (byte)197, (byte)241, (byte)19, (byte)172, (byte)84, (byte)28, (byte)190, (byte)229, (byte)135, (byte)29, (byte)95, (byte)193, (byte)148, (byte)236, (byte)176, (byte)111, (byte)136, (byte)80, (byte)172, (byte)64, (byte)211, (byte)144, (byte)90, (byte)177, (byte)198, (byte)146, (byte)203, (byte)106, (byte)198, (byte)13, (byte)145, (byte)126, (byte)252, (byte)168, (byte)149, (byte)136, (byte)78, (byte)134, (byte)56, (byte)83, (byte)7, (byte)188, (byte)248, (byte)42, (byte)87, (byte)217, (byte)129, (byte)214, (byte)10, (byte)91, (byte)21, (byte)28, (byte)121, (byte)100, (byte)192, (byte)191}, 0) ;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.target_component == (byte)(byte)204);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)19;
            p121.target_component = (byte)(byte)204;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)15);
                Debug.Assert(pack.target_system == (byte)(byte)122);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)15;
            p122.target_system = (byte)(byte)122;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)55, (byte)129, (byte)149, (byte)10, (byte)166, (byte)85, (byte)30, (byte)35, (byte)107, (byte)50, (byte)104, (byte)179, (byte)197, (byte)126, (byte)187, (byte)143, (byte)49, (byte)43, (byte)179, (byte)173, (byte)42, (byte)236, (byte)94, (byte)85, (byte)76, (byte)145, (byte)227, (byte)201, (byte)98, (byte)56, (byte)96, (byte)249, (byte)210, (byte)20, (byte)170, (byte)130, (byte)169, (byte)213, (byte)129, (byte)145, (byte)42, (byte)213, (byte)207, (byte)109, (byte)209, (byte)149, (byte)172, (byte)123, (byte)115, (byte)3, (byte)171, (byte)192, (byte)17, (byte)201, (byte)37, (byte)201, (byte)8, (byte)1, (byte)90, (byte)55, (byte)53, (byte)28, (byte)131, (byte)57, (byte)23, (byte)50, (byte)94, (byte)145, (byte)83, (byte)73, (byte)229, (byte)70, (byte)58, (byte)250, (byte)63, (byte)47, (byte)76, (byte)8, (byte)135, (byte)74, (byte)136, (byte)231, (byte)38, (byte)96, (byte)28, (byte)207, (byte)44, (byte)71, (byte)93, (byte)154, (byte)174, (byte)254, (byte)150, (byte)241, (byte)146, (byte)32, (byte)74, (byte)23, (byte)28, (byte)117, (byte)209, (byte)34, (byte)220, (byte)44, (byte)227, (byte)173, (byte)181, (byte)55, (byte)245, (byte)76}));
                Debug.Assert(pack.len == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)177);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)118;
            p123.target_system = (byte)(byte)177;
            p123.target_component = (byte)(byte)57;
            p123.data__SET(new byte[] {(byte)55, (byte)129, (byte)149, (byte)10, (byte)166, (byte)85, (byte)30, (byte)35, (byte)107, (byte)50, (byte)104, (byte)179, (byte)197, (byte)126, (byte)187, (byte)143, (byte)49, (byte)43, (byte)179, (byte)173, (byte)42, (byte)236, (byte)94, (byte)85, (byte)76, (byte)145, (byte)227, (byte)201, (byte)98, (byte)56, (byte)96, (byte)249, (byte)210, (byte)20, (byte)170, (byte)130, (byte)169, (byte)213, (byte)129, (byte)145, (byte)42, (byte)213, (byte)207, (byte)109, (byte)209, (byte)149, (byte)172, (byte)123, (byte)115, (byte)3, (byte)171, (byte)192, (byte)17, (byte)201, (byte)37, (byte)201, (byte)8, (byte)1, (byte)90, (byte)55, (byte)53, (byte)28, (byte)131, (byte)57, (byte)23, (byte)50, (byte)94, (byte)145, (byte)83, (byte)73, (byte)229, (byte)70, (byte)58, (byte)250, (byte)63, (byte)47, (byte)76, (byte)8, (byte)135, (byte)74, (byte)136, (byte)231, (byte)38, (byte)96, (byte)28, (byte)207, (byte)44, (byte)71, (byte)93, (byte)154, (byte)174, (byte)254, (byte)150, (byte)241, (byte)146, (byte)32, (byte)74, (byte)23, (byte)28, (byte)117, (byte)209, (byte)34, (byte)220, (byte)44, (byte)227, (byte)173, (byte)181, (byte)55, (byte)245, (byte)76}, 0) ;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -2146211475);
                Debug.Assert(pack.lat == (int)718195450);
                Debug.Assert(pack.vel == (ushort)(ushort)38577);
                Debug.Assert(pack.dgps_numch == (byte)(byte)222);
                Debug.Assert(pack.cog == (ushort)(ushort)3030);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.satellites_visible == (byte)(byte)168);
                Debug.Assert(pack.epv == (ushort)(ushort)55731);
                Debug.Assert(pack.eph == (ushort)(ushort)576);
                Debug.Assert(pack.time_usec == (ulong)724415432203628327L);
                Debug.Assert(pack.dgps_age == (uint)2006597609U);
                Debug.Assert(pack.alt == (int) -666163425);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.lat = (int)718195450;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.epv = (ushort)(ushort)55731;
            p124.time_usec = (ulong)724415432203628327L;
            p124.alt = (int) -666163425;
            p124.dgps_age = (uint)2006597609U;
            p124.eph = (ushort)(ushort)576;
            p124.satellites_visible = (byte)(byte)168;
            p124.dgps_numch = (byte)(byte)222;
            p124.vel = (ushort)(ushort)38577;
            p124.lon = (int) -2146211475;
            p124.cog = (ushort)(ushort)3030;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)60364);
                Debug.Assert(pack.Vcc == (ushort)(ushort)44598);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)44598;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            p125.Vservo = (ushort)(ushort)60364;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)3968206402U);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
                Debug.Assert(pack.count == (byte)(byte)154);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)230, (byte)87, (byte)167, (byte)117, (byte)145, (byte)19, (byte)127, (byte)183, (byte)58, (byte)60, (byte)93, (byte)85, (byte)124, (byte)232, (byte)213, (byte)76, (byte)96, (byte)224, (byte)225, (byte)216, (byte)151, (byte)46, (byte)92, (byte)248, (byte)180, (byte)101, (byte)157, (byte)96, (byte)202, (byte)252, (byte)243, (byte)153, (byte)141, (byte)227, (byte)101, (byte)32, (byte)199, (byte)245, (byte)4, (byte)92, (byte)155, (byte)19, (byte)184, (byte)254, (byte)220, (byte)156, (byte)189, (byte)81, (byte)217, (byte)77, (byte)238, (byte)174, (byte)212, (byte)149, (byte)124, (byte)194, (byte)195, (byte)33, (byte)22, (byte)30, (byte)130, (byte)205, (byte)196, (byte)117, (byte)4, (byte)206, (byte)149, (byte)97, (byte)61, (byte)116}));
                Debug.Assert(pack.timeout == (ushort)(ushort)63501);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.baudrate = (uint)3968206402U;
            p126.timeout = (ushort)(ushort)63501;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.data__SET(new byte[] {(byte)230, (byte)87, (byte)167, (byte)117, (byte)145, (byte)19, (byte)127, (byte)183, (byte)58, (byte)60, (byte)93, (byte)85, (byte)124, (byte)232, (byte)213, (byte)76, (byte)96, (byte)224, (byte)225, (byte)216, (byte)151, (byte)46, (byte)92, (byte)248, (byte)180, (byte)101, (byte)157, (byte)96, (byte)202, (byte)252, (byte)243, (byte)153, (byte)141, (byte)227, (byte)101, (byte)32, (byte)199, (byte)245, (byte)4, (byte)92, (byte)155, (byte)19, (byte)184, (byte)254, (byte)220, (byte)156, (byte)189, (byte)81, (byte)217, (byte)77, (byte)238, (byte)174, (byte)212, (byte)149, (byte)124, (byte)194, (byte)195, (byte)33, (byte)22, (byte)30, (byte)130, (byte)205, (byte)196, (byte)117, (byte)4, (byte)206, (byte)149, (byte)97, (byte)61, (byte)116}, 0) ;
            p126.count = (byte)(byte)154;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)56212);
                Debug.Assert(pack.baseline_a_mm == (int) -48140176);
                Debug.Assert(pack.baseline_c_mm == (int)1246791021);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)188);
                Debug.Assert(pack.nsats == (byte)(byte)152);
                Debug.Assert(pack.rtk_rate == (byte)(byte)194);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)61);
                Debug.Assert(pack.tow == (uint)1875167410U);
                Debug.Assert(pack.accuracy == (uint)3912269248U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1982928085);
                Debug.Assert(pack.baseline_b_mm == (int) -558539038);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2981542006U);
                Debug.Assert(pack.rtk_health == (byte)(byte)64);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)3912269248U;
            p127.baseline_b_mm = (int) -558539038;
            p127.wn = (ushort)(ushort)56212;
            p127.baseline_coords_type = (byte)(byte)61;
            p127.baseline_c_mm = (int)1246791021;
            p127.rtk_receiver_id = (byte)(byte)188;
            p127.iar_num_hypotheses = (int)1982928085;
            p127.tow = (uint)1875167410U;
            p127.time_last_baseline_ms = (uint)2981542006U;
            p127.rtk_health = (byte)(byte)64;
            p127.rtk_rate = (byte)(byte)194;
            p127.baseline_a_mm = (int) -48140176;
            p127.nsats = (byte)(byte)152;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.iar_num_hypotheses == (int) -1207780226);
                Debug.Assert(pack.baseline_a_mm == (int) -1763185977);
                Debug.Assert(pack.rtk_health == (byte)(byte)19);
                Debug.Assert(pack.baseline_b_mm == (int) -709690694);
                Debug.Assert(pack.nsats == (byte)(byte)146);
                Debug.Assert(pack.tow == (uint)2327144306U);
                Debug.Assert(pack.wn == (ushort)(ushort)39109);
                Debug.Assert(pack.baseline_c_mm == (int)1698029675);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)111);
                Debug.Assert(pack.rtk_rate == (byte)(byte)143);
                Debug.Assert(pack.accuracy == (uint)867919701U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1333703679U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)95);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_health = (byte)(byte)19;
            p128.accuracy = (uint)867919701U;
            p128.baseline_b_mm = (int) -709690694;
            p128.time_last_baseline_ms = (uint)1333703679U;
            p128.tow = (uint)2327144306U;
            p128.baseline_c_mm = (int)1698029675;
            p128.iar_num_hypotheses = (int) -1207780226;
            p128.baseline_a_mm = (int) -1763185977;
            p128.wn = (ushort)(ushort)39109;
            p128.rtk_rate = (byte)(byte)143;
            p128.rtk_receiver_id = (byte)(byte)95;
            p128.nsats = (byte)(byte)146;
            p128.baseline_coords_type = (byte)(byte)111;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)5406);
                Debug.Assert(pack.yacc == (short)(short)13945);
                Debug.Assert(pack.xgyro == (short)(short)8859);
                Debug.Assert(pack.xacc == (short)(short) -23848);
                Debug.Assert(pack.zacc == (short)(short) -20130);
                Debug.Assert(pack.xmag == (short)(short) -9935);
                Debug.Assert(pack.time_boot_ms == (uint)2668489010U);
                Debug.Assert(pack.ymag == (short)(short) -12982);
                Debug.Assert(pack.zgyro == (short)(short)2172);
                Debug.Assert(pack.zmag == (short)(short)28763);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.ymag = (short)(short) -12982;
            p129.xacc = (short)(short) -23848;
            p129.zacc = (short)(short) -20130;
            p129.yacc = (short)(short)13945;
            p129.zmag = (short)(short)28763;
            p129.time_boot_ms = (uint)2668489010U;
            p129.ygyro = (short)(short)5406;
            p129.xgyro = (short)(short)8859;
            p129.xmag = (short)(short) -9935;
            p129.zgyro = (short)(short)2172;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)1732);
                Debug.Assert(pack.payload == (byte)(byte)131);
                Debug.Assert(pack.width == (ushort)(ushort)35118);
                Debug.Assert(pack.packets == (ushort)(ushort)50772);
                Debug.Assert(pack.type == (byte)(byte)178);
                Debug.Assert(pack.jpg_quality == (byte)(byte)12);
                Debug.Assert(pack.size == (uint)3613627598U);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)12;
            p130.width = (ushort)(ushort)35118;
            p130.size = (uint)3613627598U;
            p130.height = (ushort)(ushort)1732;
            p130.payload = (byte)(byte)131;
            p130.packets = (ushort)(ushort)50772;
            p130.type = (byte)(byte)178;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)166, (byte)47, (byte)24, (byte)137, (byte)10, (byte)240, (byte)219, (byte)250, (byte)146, (byte)72, (byte)175, (byte)154, (byte)108, (byte)132, (byte)45, (byte)50, (byte)181, (byte)146, (byte)105, (byte)25, (byte)17, (byte)148, (byte)164, (byte)48, (byte)241, (byte)227, (byte)193, (byte)34, (byte)36, (byte)11, (byte)102, (byte)87, (byte)127, (byte)8, (byte)40, (byte)38, (byte)45, (byte)211, (byte)39, (byte)25, (byte)217, (byte)76, (byte)174, (byte)144, (byte)51, (byte)168, (byte)152, (byte)153, (byte)30, (byte)7, (byte)6, (byte)226, (byte)41, (byte)222, (byte)60, (byte)129, (byte)26, (byte)204, (byte)191, (byte)81, (byte)187, (byte)229, (byte)76, (byte)254, (byte)186, (byte)208, (byte)65, (byte)103, (byte)244, (byte)131, (byte)100, (byte)174, (byte)219, (byte)201, (byte)145, (byte)7, (byte)246, (byte)116, (byte)248, (byte)23, (byte)238, (byte)50, (byte)115, (byte)194, (byte)44, (byte)70, (byte)142, (byte)215, (byte)117, (byte)151, (byte)175, (byte)214, (byte)227, (byte)117, (byte)239, (byte)68, (byte)109, (byte)103, (byte)232, (byte)211, (byte)87, (byte)177, (byte)55, (byte)157, (byte)156, (byte)123, (byte)90, (byte)183, (byte)169, (byte)119, (byte)117, (byte)204, (byte)161, (byte)151, (byte)102, (byte)49, (byte)146, (byte)24, (byte)222, (byte)1, (byte)99, (byte)182, (byte)97, (byte)76, (byte)109, (byte)113, (byte)55, (byte)125, (byte)130, (byte)201, (byte)28, (byte)221, (byte)145, (byte)33, (byte)15, (byte)58, (byte)45, (byte)73, (byte)227, (byte)38, (byte)144, (byte)217, (byte)92, (byte)132, (byte)73, (byte)186, (byte)181, (byte)232, (byte)64, (byte)238, (byte)3, (byte)165, (byte)128, (byte)174, (byte)202, (byte)61, (byte)37, (byte)58, (byte)204, (byte)141, (byte)154, (byte)46, (byte)195, (byte)200, (byte)136, (byte)231, (byte)41, (byte)225, (byte)224, (byte)223, (byte)206, (byte)239, (byte)129, (byte)76, (byte)47, (byte)87, (byte)249, (byte)235, (byte)74, (byte)87, (byte)74, (byte)6, (byte)246, (byte)86, (byte)145, (byte)78, (byte)127, (byte)189, (byte)192, (byte)108, (byte)179, (byte)181, (byte)209, (byte)136, (byte)45, (byte)214, (byte)137, (byte)228, (byte)65, (byte)110, (byte)24, (byte)129, (byte)197, (byte)157, (byte)90, (byte)91, (byte)232, (byte)153, (byte)237, (byte)251, (byte)141, (byte)171, (byte)176, (byte)197, (byte)135, (byte)214, (byte)217, (byte)120, (byte)126, (byte)208, (byte)95, (byte)221, (byte)133, (byte)115, (byte)192, (byte)63, (byte)120, (byte)3, (byte)233, (byte)20, (byte)244, (byte)102, (byte)207, (byte)67, (byte)218, (byte)167, (byte)128, (byte)100, (byte)125, (byte)237, (byte)210, (byte)14, (byte)2, (byte)219, (byte)113, (byte)44, (byte)44, (byte)182, (byte)235, (byte)116, (byte)61, (byte)46, (byte)216}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)41107);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)41107;
            p131.data__SET(new byte[] {(byte)166, (byte)47, (byte)24, (byte)137, (byte)10, (byte)240, (byte)219, (byte)250, (byte)146, (byte)72, (byte)175, (byte)154, (byte)108, (byte)132, (byte)45, (byte)50, (byte)181, (byte)146, (byte)105, (byte)25, (byte)17, (byte)148, (byte)164, (byte)48, (byte)241, (byte)227, (byte)193, (byte)34, (byte)36, (byte)11, (byte)102, (byte)87, (byte)127, (byte)8, (byte)40, (byte)38, (byte)45, (byte)211, (byte)39, (byte)25, (byte)217, (byte)76, (byte)174, (byte)144, (byte)51, (byte)168, (byte)152, (byte)153, (byte)30, (byte)7, (byte)6, (byte)226, (byte)41, (byte)222, (byte)60, (byte)129, (byte)26, (byte)204, (byte)191, (byte)81, (byte)187, (byte)229, (byte)76, (byte)254, (byte)186, (byte)208, (byte)65, (byte)103, (byte)244, (byte)131, (byte)100, (byte)174, (byte)219, (byte)201, (byte)145, (byte)7, (byte)246, (byte)116, (byte)248, (byte)23, (byte)238, (byte)50, (byte)115, (byte)194, (byte)44, (byte)70, (byte)142, (byte)215, (byte)117, (byte)151, (byte)175, (byte)214, (byte)227, (byte)117, (byte)239, (byte)68, (byte)109, (byte)103, (byte)232, (byte)211, (byte)87, (byte)177, (byte)55, (byte)157, (byte)156, (byte)123, (byte)90, (byte)183, (byte)169, (byte)119, (byte)117, (byte)204, (byte)161, (byte)151, (byte)102, (byte)49, (byte)146, (byte)24, (byte)222, (byte)1, (byte)99, (byte)182, (byte)97, (byte)76, (byte)109, (byte)113, (byte)55, (byte)125, (byte)130, (byte)201, (byte)28, (byte)221, (byte)145, (byte)33, (byte)15, (byte)58, (byte)45, (byte)73, (byte)227, (byte)38, (byte)144, (byte)217, (byte)92, (byte)132, (byte)73, (byte)186, (byte)181, (byte)232, (byte)64, (byte)238, (byte)3, (byte)165, (byte)128, (byte)174, (byte)202, (byte)61, (byte)37, (byte)58, (byte)204, (byte)141, (byte)154, (byte)46, (byte)195, (byte)200, (byte)136, (byte)231, (byte)41, (byte)225, (byte)224, (byte)223, (byte)206, (byte)239, (byte)129, (byte)76, (byte)47, (byte)87, (byte)249, (byte)235, (byte)74, (byte)87, (byte)74, (byte)6, (byte)246, (byte)86, (byte)145, (byte)78, (byte)127, (byte)189, (byte)192, (byte)108, (byte)179, (byte)181, (byte)209, (byte)136, (byte)45, (byte)214, (byte)137, (byte)228, (byte)65, (byte)110, (byte)24, (byte)129, (byte)197, (byte)157, (byte)90, (byte)91, (byte)232, (byte)153, (byte)237, (byte)251, (byte)141, (byte)171, (byte)176, (byte)197, (byte)135, (byte)214, (byte)217, (byte)120, (byte)126, (byte)208, (byte)95, (byte)221, (byte)133, (byte)115, (byte)192, (byte)63, (byte)120, (byte)3, (byte)233, (byte)20, (byte)244, (byte)102, (byte)207, (byte)67, (byte)218, (byte)167, (byte)128, (byte)100, (byte)125, (byte)237, (byte)210, (byte)14, (byte)2, (byte)219, (byte)113, (byte)44, (byte)44, (byte)182, (byte)235, (byte)116, (byte)61, (byte)46, (byte)216}, 0) ;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.current_distance == (ushort)(ushort)20628);
                Debug.Assert(pack.id == (byte)(byte)18);
                Debug.Assert(pack.time_boot_ms == (uint)3140499941U);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270);
                Debug.Assert(pack.min_distance == (ushort)(ushort)4001);
                Debug.Assert(pack.max_distance == (ushort)(ushort)53607);
                Debug.Assert(pack.covariance == (byte)(byte)169);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)4001;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270;
            p132.id = (byte)(byte)18;
            p132.covariance = (byte)(byte)169;
            p132.max_distance = (ushort)(ushort)53607;
            p132.current_distance = (ushort)(ushort)20628;
            p132.time_boot_ms = (uint)3140499941U;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1612002703);
                Debug.Assert(pack.lon == (int)1034119632);
                Debug.Assert(pack.mask == (ulong)6573223501712572951L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)56279);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)56279;
            p133.lat = (int)1612002703;
            p133.mask = (ulong)6573223501712572951L;
            p133.lon = (int)1034119632;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)44782);
                Debug.Assert(pack.gridbit == (byte)(byte)116);
                Debug.Assert(pack.lon == (int)1833953052);
                Debug.Assert(pack.lat == (int) -400019440);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)28361, (short)25924, (short)1608, (short) -23453, (short) -24151, (short) -32723, (short) -23374, (short)13957, (short)16136, (short) -28121, (short)9867, (short) -10590, (short) -5879, (short) -19950, (short) -25442, (short) -10031}));
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)44782;
            p134.gridbit = (byte)(byte)116;
            p134.lon = (int)1833953052;
            p134.lat = (int) -400019440;
            p134.data__SET(new short[] {(short)28361, (short)25924, (short)1608, (short) -23453, (short) -24151, (short) -32723, (short) -23374, (short)13957, (short)16136, (short) -28121, (short)9867, (short) -10590, (short) -5879, (short) -19950, (short) -25442, (short) -10031}, 0) ;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)672939354);
                Debug.Assert(pack.lon == (int) -430622985);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int) -430622985;
            p135.lat = (int)672939354;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)4258);
                Debug.Assert(pack.pending == (ushort)(ushort)53257);
                Debug.Assert(pack.lat == (int) -1183819105);
                Debug.Assert(pack.current_height == (float) -1.279902E38F);
                Debug.Assert(pack.terrain_height == (float)3.4261153E37F);
                Debug.Assert(pack.lon == (int)113170187);
                Debug.Assert(pack.loaded == (ushort)(ushort)61568);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)53257;
            p136.terrain_height = (float)3.4261153E37F;
            p136.lat = (int) -1183819105;
            p136.spacing = (ushort)(ushort)4258;
            p136.current_height = (float) -1.279902E38F;
            p136.lon = (int)113170187;
            p136.loaded = (ushort)(ushort)61568;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.9276952E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1357169998U);
                Debug.Assert(pack.temperature == (short)(short)8784);
                Debug.Assert(pack.press_abs == (float) -2.7300555E38F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)8784;
            p137.time_boot_ms = (uint)1357169998U;
            p137.press_diff = (float)2.9276952E38F;
            p137.press_abs = (float) -2.7300555E38F;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.4032135E38F);
                Debug.Assert(pack.time_usec == (ulong)4873464368631799617L);
                Debug.Assert(pack.x == (float)1.9515073E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.714777E37F, 1.559379E38F, -9.791194E37F, 1.6848522E38F}));
                Debug.Assert(pack.z == (float) -2.6031682E38F);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float) -2.6031682E38F;
            p138.q_SET(new float[] {8.714777E37F, 1.559379E38F, -9.791194E37F, 1.6848522E38F}, 0) ;
            p138.time_usec = (ulong)4873464368631799617L;
            p138.y = (float)1.4032135E38F;
            p138.x = (float)1.9515073E38F;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1446773175049602411L);
                Debug.Assert(pack.group_mlx == (byte)(byte)210);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.2765991E38F, -6.3713676E37F, 2.3369926E38F, -8.719069E37F, -1.2017103E38F, 3.2297737E38F, 2.5630486E38F, 1.7572422E38F}));
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {1.2765991E38F, -6.3713676E37F, 2.3369926E38F, -8.719069E37F, -1.2017103E38F, 3.2297737E38F, 2.5630486E38F, 1.7572422E38F}, 0) ;
            p139.target_component = (byte)(byte)127;
            p139.time_usec = (ulong)1446773175049602411L;
            p139.group_mlx = (byte)(byte)210;
            p139.target_system = (byte)(byte)58;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.334112E38F, 1.1272883E38F, 1.5154692E38F, -3.3802398E38F, -1.3907966E37F, 8.443551E37F, 2.3234327E37F, -2.5769822E38F}));
                Debug.Assert(pack.time_usec == (ulong)5650087122184530901L);
                Debug.Assert(pack.group_mlx == (byte)(byte)93);
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {3.334112E38F, 1.1272883E38F, 1.5154692E38F, -3.3802398E38F, -1.3907966E37F, 8.443551E37F, 2.3234327E37F, -2.5769822E38F}, 0) ;
            p140.group_mlx = (byte)(byte)93;
            p140.time_usec = (ulong)5650087122184530901L;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float)1.0099629E38F);
                Debug.Assert(pack.time_usec == (ulong)5549327943926081644L);
                Debug.Assert(pack.altitude_terrain == (float) -3.396349E37F);
                Debug.Assert(pack.altitude_amsl == (float)2.324869E38F);
                Debug.Assert(pack.altitude_local == (float)1.0718765E38F);
                Debug.Assert(pack.bottom_clearance == (float) -3.1586239E38F);
                Debug.Assert(pack.altitude_relative == (float)2.3147886E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_terrain = (float) -3.396349E37F;
            p141.time_usec = (ulong)5549327943926081644L;
            p141.altitude_amsl = (float)2.324869E38F;
            p141.altitude_local = (float)1.0718765E38F;
            p141.altitude_relative = (float)2.3147886E38F;
            p141.bottom_clearance = (float) -3.1586239E38F;
            p141.altitude_monotonic = (float)1.0099629E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)174);
                Debug.Assert(pack.uri_type == (byte)(byte)176);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)115, (byte)79, (byte)7, (byte)2, (byte)44, (byte)178, (byte)134, (byte)34, (byte)81, (byte)140, (byte)48, (byte)121, (byte)141, (byte)96, (byte)91, (byte)194, (byte)64, (byte)206, (byte)161, (byte)226, (byte)167, (byte)142, (byte)201, (byte)121, (byte)165, (byte)49, (byte)94, (byte)89, (byte)174, (byte)244, (byte)140, (byte)131, (byte)75, (byte)130, (byte)161, (byte)43, (byte)0, (byte)91, (byte)48, (byte)186, (byte)97, (byte)117, (byte)219, (byte)19, (byte)181, (byte)106, (byte)56, (byte)100, (byte)96, (byte)111, (byte)147, (byte)217, (byte)43, (byte)62, (byte)169, (byte)42, (byte)9, (byte)36, (byte)3, (byte)154, (byte)222, (byte)134, (byte)53, (byte)152, (byte)96, (byte)230, (byte)145, (byte)117, (byte)210, (byte)104, (byte)84, (byte)181, (byte)91, (byte)13, (byte)191, (byte)243, (byte)217, (byte)7, (byte)196, (byte)124, (byte)238, (byte)168, (byte)120, (byte)14, (byte)46, (byte)66, (byte)55, (byte)252, (byte)221, (byte)97, (byte)180, (byte)195, (byte)168, (byte)62, (byte)71, (byte)208, (byte)168, (byte)248, (byte)92, (byte)85, (byte)91, (byte)22, (byte)203, (byte)102, (byte)93, (byte)121, (byte)100, (byte)225, (byte)79, (byte)223, (byte)79, (byte)5, (byte)184, (byte)34, (byte)210, (byte)34, (byte)106, (byte)11, (byte)13, (byte)155}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)142, (byte)249, (byte)148, (byte)190, (byte)119, (byte)97, (byte)94, (byte)237, (byte)35, (byte)172, (byte)45, (byte)121, (byte)250, (byte)97, (byte)128, (byte)4, (byte)230, (byte)19, (byte)23, (byte)27, (byte)171, (byte)153, (byte)90, (byte)56, (byte)139, (byte)175, (byte)81, (byte)96, (byte)17, (byte)246, (byte)187, (byte)156, (byte)69, (byte)14, (byte)85, (byte)208, (byte)246, (byte)38, (byte)0, (byte)95, (byte)193, (byte)44, (byte)121, (byte)131, (byte)104, (byte)62, (byte)210, (byte)148, (byte)143, (byte)13, (byte)237, (byte)234, (byte)128, (byte)109, (byte)108, (byte)107, (byte)5, (byte)21, (byte)10, (byte)229, (byte)172, (byte)152, (byte)123, (byte)49, (byte)209, (byte)6, (byte)21, (byte)162, (byte)37, (byte)82, (byte)181, (byte)143, (byte)28, (byte)155, (byte)160, (byte)124, (byte)104, (byte)20, (byte)255, (byte)121, (byte)6, (byte)132, (byte)32, (byte)40, (byte)179, (byte)117, (byte)233, (byte)75, (byte)96, (byte)247, (byte)78, (byte)164, (byte)129, (byte)115, (byte)204, (byte)196, (byte)247, (byte)163, (byte)22, (byte)130, (byte)115, (byte)159, (byte)172, (byte)216, (byte)207, (byte)24, (byte)24, (byte)76, (byte)39, (byte)56, (byte)196, (byte)154, (byte)32, (byte)81, (byte)197, (byte)224, (byte)181, (byte)5, (byte)233, (byte)218}));
                Debug.Assert(pack.request_id == (byte)(byte)187);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)174;
            p142.request_id = (byte)(byte)187;
            p142.uri_SET(new byte[] {(byte)115, (byte)79, (byte)7, (byte)2, (byte)44, (byte)178, (byte)134, (byte)34, (byte)81, (byte)140, (byte)48, (byte)121, (byte)141, (byte)96, (byte)91, (byte)194, (byte)64, (byte)206, (byte)161, (byte)226, (byte)167, (byte)142, (byte)201, (byte)121, (byte)165, (byte)49, (byte)94, (byte)89, (byte)174, (byte)244, (byte)140, (byte)131, (byte)75, (byte)130, (byte)161, (byte)43, (byte)0, (byte)91, (byte)48, (byte)186, (byte)97, (byte)117, (byte)219, (byte)19, (byte)181, (byte)106, (byte)56, (byte)100, (byte)96, (byte)111, (byte)147, (byte)217, (byte)43, (byte)62, (byte)169, (byte)42, (byte)9, (byte)36, (byte)3, (byte)154, (byte)222, (byte)134, (byte)53, (byte)152, (byte)96, (byte)230, (byte)145, (byte)117, (byte)210, (byte)104, (byte)84, (byte)181, (byte)91, (byte)13, (byte)191, (byte)243, (byte)217, (byte)7, (byte)196, (byte)124, (byte)238, (byte)168, (byte)120, (byte)14, (byte)46, (byte)66, (byte)55, (byte)252, (byte)221, (byte)97, (byte)180, (byte)195, (byte)168, (byte)62, (byte)71, (byte)208, (byte)168, (byte)248, (byte)92, (byte)85, (byte)91, (byte)22, (byte)203, (byte)102, (byte)93, (byte)121, (byte)100, (byte)225, (byte)79, (byte)223, (byte)79, (byte)5, (byte)184, (byte)34, (byte)210, (byte)34, (byte)106, (byte)11, (byte)13, (byte)155}, 0) ;
            p142.uri_type = (byte)(byte)176;
            p142.storage_SET(new byte[] {(byte)142, (byte)249, (byte)148, (byte)190, (byte)119, (byte)97, (byte)94, (byte)237, (byte)35, (byte)172, (byte)45, (byte)121, (byte)250, (byte)97, (byte)128, (byte)4, (byte)230, (byte)19, (byte)23, (byte)27, (byte)171, (byte)153, (byte)90, (byte)56, (byte)139, (byte)175, (byte)81, (byte)96, (byte)17, (byte)246, (byte)187, (byte)156, (byte)69, (byte)14, (byte)85, (byte)208, (byte)246, (byte)38, (byte)0, (byte)95, (byte)193, (byte)44, (byte)121, (byte)131, (byte)104, (byte)62, (byte)210, (byte)148, (byte)143, (byte)13, (byte)237, (byte)234, (byte)128, (byte)109, (byte)108, (byte)107, (byte)5, (byte)21, (byte)10, (byte)229, (byte)172, (byte)152, (byte)123, (byte)49, (byte)209, (byte)6, (byte)21, (byte)162, (byte)37, (byte)82, (byte)181, (byte)143, (byte)28, (byte)155, (byte)160, (byte)124, (byte)104, (byte)20, (byte)255, (byte)121, (byte)6, (byte)132, (byte)32, (byte)40, (byte)179, (byte)117, (byte)233, (byte)75, (byte)96, (byte)247, (byte)78, (byte)164, (byte)129, (byte)115, (byte)204, (byte)196, (byte)247, (byte)163, (byte)22, (byte)130, (byte)115, (byte)159, (byte)172, (byte)216, (byte)207, (byte)24, (byte)24, (byte)76, (byte)39, (byte)56, (byte)196, (byte)154, (byte)32, (byte)81, (byte)197, (byte)224, (byte)181, (byte)5, (byte)233, (byte)218}, 0) ;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -32015);
                Debug.Assert(pack.press_abs == (float) -2.3810567E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2677272945U);
                Debug.Assert(pack.press_diff == (float)2.0594136E38F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -32015;
            p143.time_boot_ms = (uint)2677272945U;
            p143.press_abs = (float) -2.3810567E38F;
            p143.press_diff = (float)2.0594136E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_state == (ulong)4578161598032877343L);
                Debug.Assert(pack.lon == (int) -1799532233);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.7773184E38F, -1.4977365E38F, 3.2569834E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-3.0249616E38F, 3.374119E38F, -3.2244802E38F}));
                Debug.Assert(pack.lat == (int) -1200337326);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {2.7691687E38F, -2.8705794E38F, 2.220378E37F, -1.0653029E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {1.2984597E38F, -1.0795436E38F, 1.3027964E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.1433254E38F, -6.1383333E37F, -7.0336425E37F}));
                Debug.Assert(pack.alt == (float)1.0560121E38F);
                Debug.Assert(pack.est_capabilities == (byte)(byte)92);
                Debug.Assert(pack.timestamp == (ulong)2636213075811539846L);
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.vel_SET(new float[] {1.2984597E38F, -1.0795436E38F, 1.3027964E38F}, 0) ;
            p144.custom_state = (ulong)4578161598032877343L;
            p144.rates_SET(new float[] {-2.7773184E38F, -1.4977365E38F, 3.2569834E38F}, 0) ;
            p144.alt = (float)1.0560121E38F;
            p144.acc_SET(new float[] {-3.0249616E38F, 3.374119E38F, -3.2244802E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)92;
            p144.position_cov_SET(new float[] {2.1433254E38F, -6.1383333E37F, -7.0336425E37F}, 0) ;
            p144.timestamp = (ulong)2636213075811539846L;
            p144.lat = (int) -1200337326;
            p144.attitude_q_SET(new float[] {2.7691687E38F, -2.8705794E38F, 2.220378E37F, -1.0653029E38F}, 0) ;
            p144.lon = (int) -1799532233;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_acc == (float) -1.7075797E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.365883E37F, -2.3167319E38F, -8.573093E37F, -2.849964E38F}));
                Debug.Assert(pack.y_vel == (float) -3.3334363E38F);
                Debug.Assert(pack.pitch_rate == (float)9.487803E37F);
                Debug.Assert(pack.z_vel == (float) -2.007764E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.184079E38F, 1.605936E38F, 2.277377E37F}));
                Debug.Assert(pack.x_vel == (float) -1.1023598E38F);
                Debug.Assert(pack.yaw_rate == (float) -5.362939E36F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {4.005855E37F, -2.6795238E37F, -1.7818117E38F}));
                Debug.Assert(pack.y_acc == (float)4.2110763E36F);
                Debug.Assert(pack.z_pos == (float) -2.6660998E38F);
                Debug.Assert(pack.x_pos == (float)1.6192433E38F);
                Debug.Assert(pack.time_usec == (ulong)2784724363427252503L);
                Debug.Assert(pack.y_pos == (float) -2.1318758E38F);
                Debug.Assert(pack.z_acc == (float)3.0342252E38F);
                Debug.Assert(pack.airspeed == (float) -4.9005537E37F);
                Debug.Assert(pack.roll_rate == (float)1.5203592E38F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pitch_rate = (float)9.487803E37F;
            p146.z_acc = (float)3.0342252E38F;
            p146.y_acc = (float)4.2110763E36F;
            p146.z_vel = (float) -2.007764E38F;
            p146.x_vel = (float) -1.1023598E38F;
            p146.roll_rate = (float)1.5203592E38F;
            p146.vel_variance_SET(new float[] {2.184079E38F, 1.605936E38F, 2.277377E37F}, 0) ;
            p146.q_SET(new float[] {-9.365883E37F, -2.3167319E38F, -8.573093E37F, -2.849964E38F}, 0) ;
            p146.pos_variance_SET(new float[] {4.005855E37F, -2.6795238E37F, -1.7818117E38F}, 0) ;
            p146.time_usec = (ulong)2784724363427252503L;
            p146.z_pos = (float) -2.6660998E38F;
            p146.yaw_rate = (float) -5.362939E36F;
            p146.x_pos = (float)1.6192433E38F;
            p146.y_vel = (float) -3.3334363E38F;
            p146.y_pos = (float) -2.1318758E38F;
            p146.x_acc = (float) -1.7075797E38F;
            p146.airspeed = (float) -4.9005537E37F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int)1348175344);
                Debug.Assert(pack.energy_consumed == (int)227707423);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)56609, (ushort)7392, (ushort)7901, (ushort)1698, (ushort)50995, (ushort)41874, (ushort)60785, (ushort)20022, (ushort)20758, (ushort)29124}));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)69);
                Debug.Assert(pack.current_battery == (short)(short) -5961);
                Debug.Assert(pack.id == (byte)(byte)169);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.temperature == (short)(short)16353);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_battery = (short)(short) -5961;
            p147.battery_remaining = (sbyte)(sbyte)69;
            p147.voltages_SET(new ushort[] {(ushort)56609, (ushort)7392, (ushort)7901, (ushort)1698, (ushort)50995, (ushort)41874, (ushort)60785, (ushort)20022, (ushort)20758, (ushort)29124}, 0) ;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.current_consumed = (int)1348175344;
            p147.energy_consumed = (int)227707423;
            p147.temperature = (short)(short)16353;
            p147.id = (byte)(byte)169;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)209, (byte)213, (byte)77, (byte)202, (byte)140, (byte)207, (byte)38, (byte)61}));
                Debug.Assert(pack.flight_sw_version == (uint)1571515452U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)41, (byte)250, (byte)6, (byte)148, (byte)25, (byte)153, (byte)159, (byte)208}));
                Debug.Assert(pack.os_sw_version == (uint)3775533258U);
                Debug.Assert(pack.middleware_sw_version == (uint)2207367096U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
                Debug.Assert(pack.uid == (ulong)9198690891102673461L);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)164, (byte)244, (byte)158, (byte)110, (byte)73, (byte)72, (byte)128, (byte)168, (byte)225, (byte)96, (byte)48, (byte)204, (byte)208, (byte)199, (byte)44, (byte)154, (byte)17, (byte)23}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)53565);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)224, (byte)208, (byte)196, (byte)208, (byte)179, (byte)105, (byte)156, (byte)172}));
                Debug.Assert(pack.product_id == (ushort)(ushort)33919);
                Debug.Assert(pack.board_version == (uint)3948569365U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_custom_version_SET(new byte[] {(byte)41, (byte)250, (byte)6, (byte)148, (byte)25, (byte)153, (byte)159, (byte)208}, 0) ;
            p148.uid2_SET(new byte[] {(byte)164, (byte)244, (byte)158, (byte)110, (byte)73, (byte)72, (byte)128, (byte)168, (byte)225, (byte)96, (byte)48, (byte)204, (byte)208, (byte)199, (byte)44, (byte)154, (byte)17, (byte)23}, 0, PH) ;
            p148.uid = (ulong)9198690891102673461L;
            p148.vendor_id = (ushort)(ushort)53565;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
            p148.os_sw_version = (uint)3775533258U;
            p148.middleware_custom_version_SET(new byte[] {(byte)209, (byte)213, (byte)77, (byte)202, (byte)140, (byte)207, (byte)38, (byte)61}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)224, (byte)208, (byte)196, (byte)208, (byte)179, (byte)105, (byte)156, (byte)172}, 0) ;
            p148.flight_sw_version = (uint)1571515452U;
            p148.board_version = (uint)3948569365U;
            p148.product_id = (ushort)(ushort)33919;
            p148.middleware_sw_version = (uint)2207367096U;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float) -2.5480237E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.3297084E38F, 6.967276E37F, 8.82075E37F, 2.79056E38F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)79);
                Debug.Assert(pack.target_num == (byte)(byte)191);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.size_y == (float)1.0556856E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)2.9455507E38F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.distance == (float)1.5398226E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -1.970142E38F);
                Debug.Assert(pack.size_x == (float)1.3063955E38F);
                Debug.Assert(pack.time_usec == (ulong)576975965496085328L);
                Debug.Assert(pack.y_TRY(ph) == (float) -9.273098E37F);
                Debug.Assert(pack.angle_y == (float)1.9827708E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float)1.9827708E38F;
            p149.target_num = (byte)(byte)191;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.size_x = (float)1.3063955E38F;
            p149.y_SET((float) -9.273098E37F, PH) ;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.time_usec = (ulong)576975965496085328L;
            p149.x_SET((float) -1.970142E38F, PH) ;
            p149.size_y = (float)1.0556856E38F;
            p149.position_valid_SET((byte)(byte)79, PH) ;
            p149.q_SET(new float[] {-2.3297084E38F, 6.967276E37F, 8.82075E37F, 2.79056E38F}, 0, PH) ;
            p149.distance = (float)1.5398226E38F;
            p149.z_SET((float)2.9455507E38F, PH) ;
            p149.angle_x = (float) -2.5480237E38F;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAQ_TELEMETRY_FReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Index == (ushort)(ushort)41801);
                Debug.Assert(pack.value7 == (float)3.1165983E38F);
                Debug.Assert(pack.value15 == (float)3.2406516E38F);
                Debug.Assert(pack.value2 == (float)8.594305E36F);
                Debug.Assert(pack.value13 == (float) -1.1678271E38F);
                Debug.Assert(pack.value4 == (float)2.4111254E38F);
                Debug.Assert(pack.value8 == (float) -5.9624737E37F);
                Debug.Assert(pack.value20 == (float)1.6711088E38F);
                Debug.Assert(pack.value5 == (float)1.3143818E38F);
                Debug.Assert(pack.value16 == (float)2.7105071E38F);
                Debug.Assert(pack.value10 == (float) -5.9593867E37F);
                Debug.Assert(pack.value17 == (float) -3.279042E38F);
                Debug.Assert(pack.value12 == (float)3.0726043E38F);
                Debug.Assert(pack.value11 == (float)3.3479206E38F);
                Debug.Assert(pack.value18 == (float)2.7605862E38F);
                Debug.Assert(pack.value6 == (float) -9.709433E37F);
                Debug.Assert(pack.value14 == (float)3.3476705E38F);
                Debug.Assert(pack.value9 == (float) -2.6983505E37F);
                Debug.Assert(pack.value1 == (float) -2.1097219E38F);
                Debug.Assert(pack.value19 == (float) -1.0306486E38F);
                Debug.Assert(pack.value3 == (float)2.782603E38F);
            };
            DemoDevice.AQ_TELEMETRY_F p150 = LoopBackDemoChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.value7 = (float)3.1165983E38F;
            p150.value6 = (float) -9.709433E37F;
            p150.value5 = (float)1.3143818E38F;
            p150.Index = (ushort)(ushort)41801;
            p150.value4 = (float)2.4111254E38F;
            p150.value11 = (float)3.3479206E38F;
            p150.value10 = (float) -5.9593867E37F;
            p150.value9 = (float) -2.6983505E37F;
            p150.value1 = (float) -2.1097219E38F;
            p150.value19 = (float) -1.0306486E38F;
            p150.value16 = (float)2.7105071E38F;
            p150.value18 = (float)2.7605862E38F;
            p150.value15 = (float)3.2406516E38F;
            p150.value14 = (float)3.3476705E38F;
            p150.value12 = (float)3.0726043E38F;
            p150.value13 = (float) -1.1678271E38F;
            p150.value8 = (float) -5.9624737E37F;
            p150.value20 = (float)1.6711088E38F;
            p150.value17 = (float) -3.279042E38F;
            p150.value2 = (float)8.594305E36F;
            p150.value3 = (float)2.782603E38F;
            LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAQ_ESC_TELEMETRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_motors == (byte)(byte)63);
                Debug.Assert(pack.data0.SequenceEqual(new uint[] {3092163631U, 3017243768U, 4205445091U, 2565465422U}));
                Debug.Assert(pack.seq == (byte)(byte)22);
                Debug.Assert(pack.data1.SequenceEqual(new uint[] {3582644941U, 995915779U, 1714278818U, 639729349U}));
                Debug.Assert(pack.escid.SequenceEqual(new byte[] {(byte)107, (byte)226, (byte)126, (byte)9}));
                Debug.Assert(pack.time_boot_ms == (uint)3881357172U);
                Debug.Assert(pack.status_age.SequenceEqual(new ushort[] {(ushort)63853, (ushort)26105, (ushort)41277, (ushort)26316}));
                Debug.Assert(pack.data_version.SequenceEqual(new byte[] {(byte)194, (byte)93, (byte)85, (byte)188}));
                Debug.Assert(pack.num_in_seq == (byte)(byte)243);
            };
            DemoDevice.AQ_ESC_TELEMETRY p152 = LoopBackDemoChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.num_in_seq = (byte)(byte)243;
            p152.data1_SET(new uint[] {3582644941U, 995915779U, 1714278818U, 639729349U}, 0) ;
            p152.data_version_SET(new byte[] {(byte)194, (byte)93, (byte)85, (byte)188}, 0) ;
            p152.data0_SET(new uint[] {3092163631U, 3017243768U, 4205445091U, 2565465422U}, 0) ;
            p152.status_age_SET(new ushort[] {(ushort)63853, (ushort)26105, (ushort)41277, (ushort)26316}, 0) ;
            p152.seq = (byte)(byte)22;
            p152.escid_SET(new byte[] {(byte)107, (byte)226, (byte)126, (byte)9}, 0) ;
            p152.num_motors = (byte)(byte)63;
            p152.time_boot_ms = (uint)3881357172U;
            LoopBackDemoChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7905049773163542002L);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.822259E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.4512563E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)2.694011E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.2850106E38F);
                Debug.Assert(pack.tas_ratio == (float) -2.9939206E38F);
                Debug.Assert(pack.hagl_ratio == (float) -2.8633625E38F);
                Debug.Assert(pack.mag_ratio == (float) -2.9063472E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.9427888E37F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.tas_ratio = (float) -2.9939206E38F;
            p230.pos_vert_ratio = (float)2.694011E37F;
            p230.hagl_ratio = (float) -2.8633625E38F;
            p230.pos_horiz_accuracy = (float)1.2850106E38F;
            p230.pos_horiz_ratio = (float)1.822259E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
            p230.pos_vert_accuracy = (float)2.9427888E37F;
            p230.vel_ratio = (float) -1.4512563E38F;
            p230.time_usec = (ulong)7905049773163542002L;
            p230.mag_ratio = (float) -2.9063472E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_x == (float) -1.1373123E38F);
                Debug.Assert(pack.wind_alt == (float)5.1605143E37F);
                Debug.Assert(pack.vert_accuracy == (float)1.8869527E38F);
                Debug.Assert(pack.var_vert == (float) -1.3495688E38F);
                Debug.Assert(pack.wind_z == (float)1.1442866E38F);
                Debug.Assert(pack.time_usec == (ulong)7606238949843154024L);
                Debug.Assert(pack.wind_y == (float) -8.783267E37F);
                Debug.Assert(pack.horiz_accuracy == (float)7.9610126E37F);
                Debug.Assert(pack.var_horiz == (float)2.8714397E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float)2.8714397E38F;
            p231.vert_accuracy = (float)1.8869527E38F;
            p231.var_vert = (float) -1.3495688E38F;
            p231.wind_y = (float) -8.783267E37F;
            p231.wind_alt = (float)5.1605143E37F;
            p231.wind_z = (float)1.1442866E38F;
            p231.wind_x = (float) -1.1373123E38F;
            p231.horiz_accuracy = (float)7.9610126E37F;
            p231.time_usec = (ulong)7606238949843154024L;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_id == (byte)(byte)76);
                Debug.Assert(pack.vdop == (float)1.8007409E38F);
                Debug.Assert(pack.vn == (float)2.4275349E38F);
                Debug.Assert(pack.ve == (float) -2.1114242E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
                Debug.Assert(pack.vert_accuracy == (float) -7.089787E37F);
                Debug.Assert(pack.lat == (int) -849390925);
                Debug.Assert(pack.speed_accuracy == (float) -1.5378894E38F);
                Debug.Assert(pack.time_usec == (ulong)4402678809316726069L);
                Debug.Assert(pack.fix_type == (byte)(byte)78);
                Debug.Assert(pack.satellites_visible == (byte)(byte)22);
                Debug.Assert(pack.lon == (int)1891040102);
                Debug.Assert(pack.time_week_ms == (uint)409172874U);
                Debug.Assert(pack.horiz_accuracy == (float) -3.0858335E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)54832);
                Debug.Assert(pack.vd == (float)2.645283E38F);
                Debug.Assert(pack.hdop == (float)5.6878595E37F);
                Debug.Assert(pack.alt == (float) -2.474601E38F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_week = (ushort)(ushort)54832;
            p232.vdop = (float)1.8007409E38F;
            p232.lat = (int) -849390925;
            p232.vd = (float)2.645283E38F;
            p232.alt = (float) -2.474601E38F;
            p232.time_usec = (ulong)4402678809316726069L;
            p232.vert_accuracy = (float) -7.089787E37F;
            p232.horiz_accuracy = (float) -3.0858335E38F;
            p232.fix_type = (byte)(byte)78;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
            p232.hdop = (float)5.6878595E37F;
            p232.vn = (float)2.4275349E38F;
            p232.lon = (int)1891040102;
            p232.gps_id = (byte)(byte)76;
            p232.satellites_visible = (byte)(byte)22;
            p232.ve = (float) -2.1114242E38F;
            p232.time_week_ms = (uint)409172874U;
            p232.speed_accuracy = (float) -1.5378894E38F;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)7);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)22, (byte)41, (byte)168, (byte)247, (byte)103, (byte)170, (byte)153, (byte)30, (byte)173, (byte)2, (byte)210, (byte)65, (byte)62, (byte)6, (byte)185, (byte)127, (byte)221, (byte)155, (byte)181, (byte)142, (byte)130, (byte)25, (byte)76, (byte)151, (byte)79, (byte)90, (byte)78, (byte)173, (byte)246, (byte)57, (byte)232, (byte)175, (byte)240, (byte)0, (byte)130, (byte)8, (byte)185, (byte)139, (byte)199, (byte)97, (byte)86, (byte)33, (byte)30, (byte)236, (byte)208, (byte)62, (byte)222, (byte)103, (byte)241, (byte)231, (byte)44, (byte)240, (byte)67, (byte)155, (byte)74, (byte)32, (byte)158, (byte)140, (byte)45, (byte)153, (byte)97, (byte)16, (byte)144, (byte)1, (byte)190, (byte)46, (byte)93, (byte)76, (byte)91, (byte)36, (byte)168, (byte)160, (byte)80, (byte)254, (byte)10, (byte)193, (byte)190, (byte)72, (byte)175, (byte)125, (byte)254, (byte)176, (byte)224, (byte)92, (byte)33, (byte)144, (byte)251, (byte)141, (byte)45, (byte)76, (byte)30, (byte)40, (byte)46, (byte)210, (byte)169, (byte)139, (byte)63, (byte)208, (byte)133, (byte)210, (byte)39, (byte)229, (byte)164, (byte)116, (byte)143, (byte)158, (byte)175, (byte)183, (byte)38, (byte)242, (byte)112, (byte)37, (byte)151, (byte)14, (byte)2, (byte)127, (byte)109, (byte)231, (byte)53, (byte)116, (byte)5, (byte)194, (byte)110, (byte)255, (byte)230, (byte)10, (byte)198, (byte)99, (byte)233, (byte)200, (byte)71, (byte)138, (byte)100, (byte)166, (byte)234, (byte)32, (byte)53, (byte)103, (byte)32, (byte)97, (byte)217, (byte)202, (byte)239, (byte)4, (byte)209, (byte)164, (byte)65, (byte)76, (byte)116, (byte)75, (byte)159, (byte)112, (byte)200, (byte)154, (byte)138, (byte)16, (byte)157, (byte)191, (byte)224, (byte)76, (byte)96, (byte)24, (byte)19, (byte)186, (byte)181, (byte)185, (byte)108, (byte)180, (byte)241, (byte)52, (byte)95, (byte)52, (byte)86, (byte)64, (byte)90, (byte)189, (byte)164, (byte)195, (byte)202, (byte)206}));
                Debug.Assert(pack.flags == (byte)(byte)140);
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)7;
            p233.data__SET(new byte[] {(byte)22, (byte)41, (byte)168, (byte)247, (byte)103, (byte)170, (byte)153, (byte)30, (byte)173, (byte)2, (byte)210, (byte)65, (byte)62, (byte)6, (byte)185, (byte)127, (byte)221, (byte)155, (byte)181, (byte)142, (byte)130, (byte)25, (byte)76, (byte)151, (byte)79, (byte)90, (byte)78, (byte)173, (byte)246, (byte)57, (byte)232, (byte)175, (byte)240, (byte)0, (byte)130, (byte)8, (byte)185, (byte)139, (byte)199, (byte)97, (byte)86, (byte)33, (byte)30, (byte)236, (byte)208, (byte)62, (byte)222, (byte)103, (byte)241, (byte)231, (byte)44, (byte)240, (byte)67, (byte)155, (byte)74, (byte)32, (byte)158, (byte)140, (byte)45, (byte)153, (byte)97, (byte)16, (byte)144, (byte)1, (byte)190, (byte)46, (byte)93, (byte)76, (byte)91, (byte)36, (byte)168, (byte)160, (byte)80, (byte)254, (byte)10, (byte)193, (byte)190, (byte)72, (byte)175, (byte)125, (byte)254, (byte)176, (byte)224, (byte)92, (byte)33, (byte)144, (byte)251, (byte)141, (byte)45, (byte)76, (byte)30, (byte)40, (byte)46, (byte)210, (byte)169, (byte)139, (byte)63, (byte)208, (byte)133, (byte)210, (byte)39, (byte)229, (byte)164, (byte)116, (byte)143, (byte)158, (byte)175, (byte)183, (byte)38, (byte)242, (byte)112, (byte)37, (byte)151, (byte)14, (byte)2, (byte)127, (byte)109, (byte)231, (byte)53, (byte)116, (byte)5, (byte)194, (byte)110, (byte)255, (byte)230, (byte)10, (byte)198, (byte)99, (byte)233, (byte)200, (byte)71, (byte)138, (byte)100, (byte)166, (byte)234, (byte)32, (byte)53, (byte)103, (byte)32, (byte)97, (byte)217, (byte)202, (byte)239, (byte)4, (byte)209, (byte)164, (byte)65, (byte)76, (byte)116, (byte)75, (byte)159, (byte)112, (byte)200, (byte)154, (byte)138, (byte)16, (byte)157, (byte)191, (byte)224, (byte)76, (byte)96, (byte)24, (byte)19, (byte)186, (byte)181, (byte)185, (byte)108, (byte)180, (byte)241, (byte)52, (byte)95, (byte)52, (byte)86, (byte)64, (byte)90, (byte)189, (byte)164, (byte)195, (byte)202, (byte)206}, 0) ;
            p233.flags = (byte)(byte)140;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -1747505311);
                Debug.Assert(pack.altitude_sp == (short)(short)589);
                Debug.Assert(pack.gps_nsat == (byte)(byte)175);
                Debug.Assert(pack.custom_mode == (uint)86743887U);
                Debug.Assert(pack.groundspeed == (byte)(byte)22);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)93);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)52);
                Debug.Assert(pack.roll == (short)(short)17468);
                Debug.Assert(pack.failsafe == (byte)(byte)38);
                Debug.Assert(pack.longitude == (int) -1327058614);
                Debug.Assert(pack.battery_remaining == (byte)(byte)164);
                Debug.Assert(pack.heading == (ushort)(ushort)11074);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)6261);
                Debug.Assert(pack.airspeed == (byte)(byte)243);
                Debug.Assert(pack.heading_sp == (short)(short)29387);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.wp_num == (byte)(byte)89);
                Debug.Assert(pack.pitch == (short)(short) -19294);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)135);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 24);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.altitude_amsl == (short)(short)29790);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 113);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.airspeed_sp = (byte)(byte)135;
            p234.altitude_sp = (short)(short)589;
            p234.heading = (ushort)(ushort)11074;
            p234.climb_rate = (sbyte)(sbyte) - 24;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p234.wp_num = (byte)(byte)89;
            p234.battery_remaining = (byte)(byte)164;
            p234.wp_distance = (ushort)(ushort)6261;
            p234.longitude = (int) -1327058614;
            p234.airspeed = (byte)(byte)243;
            p234.temperature_air = (sbyte)(sbyte)93;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.failsafe = (byte)(byte)38;
            p234.altitude_amsl = (short)(short)29790;
            p234.latitude = (int) -1747505311;
            p234.throttle = (sbyte)(sbyte)52;
            p234.temperature = (sbyte)(sbyte) - 113;
            p234.custom_mode = (uint)86743887U;
            p234.roll = (short)(short)17468;
            p234.heading_sp = (short)(short)29387;
            p234.pitch = (short)(short) -19294;
            p234.groundspeed = (byte)(byte)22;
            p234.gps_nsat = (byte)(byte)175;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_z == (float) -2.0042822E38F);
                Debug.Assert(pack.clipping_1 == (uint)312395545U);
                Debug.Assert(pack.time_usec == (ulong)7616321991019492654L);
                Debug.Assert(pack.vibration_y == (float) -4.5156985E37F);
                Debug.Assert(pack.vibration_x == (float) -7.328272E37F);
                Debug.Assert(pack.clipping_2 == (uint)341050427U);
                Debug.Assert(pack.clipping_0 == (uint)3689207408U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)312395545U;
            p241.vibration_z = (float) -2.0042822E38F;
            p241.vibration_y = (float) -4.5156985E37F;
            p241.time_usec = (ulong)7616321991019492654L;
            p241.clipping_0 = (uint)3689207408U;
            p241.vibration_x = (float) -7.328272E37F;
            p241.clipping_2 = (uint)341050427U;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -274546240);
                Debug.Assert(pack.approach_x == (float) -2.464116E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.0892657E38F, -1.5389933E38F, -1.7568998E38F, -2.2013779E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1511550557960722654L);
                Debug.Assert(pack.approach_y == (float)1.6671571E38F);
                Debug.Assert(pack.z == (float)2.0184614E38F);
                Debug.Assert(pack.altitude == (int) -1142656949);
                Debug.Assert(pack.x == (float)5.873624E36F);
                Debug.Assert(pack.approach_z == (float)2.9625493E38F);
                Debug.Assert(pack.y == (float)1.4279388E38F);
                Debug.Assert(pack.longitude == (int) -1209591027);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.x = (float)5.873624E36F;
            p242.y = (float)1.4279388E38F;
            p242.altitude = (int) -1142656949;
            p242.time_usec_SET((ulong)1511550557960722654L, PH) ;
            p242.approach_y = (float)1.6671571E38F;
            p242.latitude = (int) -274546240;
            p242.approach_x = (float) -2.464116E38F;
            p242.approach_z = (float)2.9625493E38F;
            p242.q_SET(new float[] {-2.0892657E38F, -1.5389933E38F, -1.7568998E38F, -2.2013779E38F}, 0) ;
            p242.z = (float)2.0184614E38F;
            p242.longitude = (int) -1209591027;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -2.2862665E38F);
                Debug.Assert(pack.altitude == (int) -740399397);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4762390924314658947L);
                Debug.Assert(pack.longitude == (int)442727850);
                Debug.Assert(pack.approach_x == (float) -2.2800736E38F);
                Debug.Assert(pack.approach_y == (float)1.3325611E38F);
                Debug.Assert(pack.z == (float) -8.976706E36F);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.y == (float) -2.2279866E38F);
                Debug.Assert(pack.latitude == (int) -2004766925);
                Debug.Assert(pack.x == (float)1.2180578E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2628423E38F, 7.636909E37F, -9.99646E37F, 1.5288257E38F}));
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)60;
            p243.x = (float)1.2180578E38F;
            p243.approach_y = (float)1.3325611E38F;
            p243.q_SET(new float[] {-2.2628423E38F, 7.636909E37F, -9.99646E37F, 1.5288257E38F}, 0) ;
            p243.approach_z = (float) -2.2862665E38F;
            p243.z = (float) -8.976706E36F;
            p243.altitude = (int) -740399397;
            p243.latitude = (int) -2004766925;
            p243.y = (float) -2.2279866E38F;
            p243.time_usec_SET((ulong)4762390924314658947L, PH) ;
            p243.longitude = (int)442727850;
            p243.approach_x = (float) -2.2800736E38F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)53494);
                Debug.Assert(pack.interval_us == (int)597963882);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)597963882;
            p244.message_id = (ushort)(ushort)53494;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1359204939);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("gi"));
                Debug.Assert(pack.altitude == (int) -48925295);
                Debug.Assert(pack.lat == (int) -1328113808);
                Debug.Assert(pack.tslc == (byte)(byte)10);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)4908);
                Debug.Assert(pack.heading == (ushort)(ushort)42039);
                Debug.Assert(pack.squawk == (ushort)(ushort)38553);
                Debug.Assert(pack.ver_velocity == (short)(short) -4606);
                Debug.Assert(pack.ICAO_address == (uint)3504853525U);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)38553;
            p246.callsign_SET("gi", PH) ;
            p246.ver_velocity = (short)(short) -4606;
            p246.lat = (int) -1328113808;
            p246.tslc = (byte)(byte)10;
            p246.hor_velocity = (ushort)(ushort)4908;
            p246.heading = (ushort)(ushort)42039;
            p246.ICAO_address = (uint)3504853525U;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
            p246.altitude = (int) -48925295;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.lon = (int) -1359204939;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float)3.2270733E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.id == (uint)4143646329U);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.7333232E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)3.2336388E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.altitude_minimum_delta = (float)2.7333232E38F;
            p247.horizontal_minimum_delta = (float)3.2336388E38F;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)4143646329U;
            p247.time_to_minimum_delta = (float)3.2270733E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)27, (byte)165, (byte)194, (byte)37, (byte)82, (byte)0, (byte)69, (byte)128, (byte)237, (byte)195, (byte)153, (byte)209, (byte)80, (byte)49, (byte)239, (byte)200, (byte)87, (byte)126, (byte)112, (byte)118, (byte)187, (byte)137, (byte)73, (byte)31, (byte)95, (byte)209, (byte)74, (byte)191, (byte)150, (byte)247, (byte)79, (byte)7, (byte)140, (byte)239, (byte)157, (byte)171, (byte)250, (byte)160, (byte)213, (byte)206, (byte)246, (byte)184, (byte)206, (byte)234, (byte)143, (byte)171, (byte)21, (byte)205, (byte)201, (byte)96, (byte)145, (byte)190, (byte)53, (byte)35, (byte)28, (byte)166, (byte)77, (byte)206, (byte)196, (byte)230, (byte)56, (byte)198, (byte)131, (byte)181, (byte)220, (byte)53, (byte)34, (byte)155, (byte)164, (byte)183, (byte)95, (byte)243, (byte)30, (byte)60, (byte)97, (byte)188, (byte)171, (byte)238, (byte)208, (byte)94, (byte)99, (byte)93, (byte)209, (byte)245, (byte)114, (byte)41, (byte)54, (byte)173, (byte)234, (byte)214, (byte)131, (byte)101, (byte)136, (byte)188, (byte)159, (byte)127, (byte)228, (byte)74, (byte)206, (byte)229, (byte)78, (byte)154, (byte)73, (byte)133, (byte)50, (byte)221, (byte)215, (byte)80, (byte)162, (byte)98, (byte)177, (byte)223, (byte)67, (byte)86, (byte)236, (byte)250, (byte)77, (byte)235, (byte)212, (byte)165, (byte)20, (byte)142, (byte)222, (byte)193, (byte)16, (byte)27, (byte)214, (byte)201, (byte)167, (byte)29, (byte)91, (byte)117, (byte)77, (byte)59, (byte)4, (byte)94, (byte)207, (byte)43, (byte)86, (byte)225, (byte)222, (byte)223, (byte)247, (byte)239, (byte)240, (byte)179, (byte)162, (byte)73, (byte)174, (byte)229, (byte)219, (byte)195, (byte)56, (byte)212, (byte)8, (byte)186, (byte)157, (byte)139, (byte)249, (byte)144, (byte)245, (byte)119, (byte)158, (byte)110, (byte)58, (byte)116, (byte)128, (byte)150, (byte)11, (byte)59, (byte)166, (byte)96, (byte)103, (byte)27, (byte)49, (byte)234, (byte)192, (byte)75, (byte)147, (byte)10, (byte)108, (byte)241, (byte)129, (byte)223, (byte)83, (byte)148, (byte)255, (byte)227, (byte)146, (byte)65, (byte)17, (byte)133, (byte)55, (byte)57, (byte)247, (byte)53, (byte)105, (byte)55, (byte)7, (byte)87, (byte)60, (byte)237, (byte)212, (byte)4, (byte)112, (byte)249, (byte)174, (byte)140, (byte)252, (byte)2, (byte)227, (byte)62, (byte)86, (byte)144, (byte)56, (byte)19, (byte)29, (byte)129, (byte)71, (byte)46, (byte)208, (byte)126, (byte)52, (byte)50, (byte)75, (byte)204, (byte)35, (byte)79, (byte)139, (byte)64, (byte)209, (byte)24, (byte)8, (byte)249, (byte)32, (byte)203, (byte)163, (byte)33, (byte)79, (byte)83, (byte)36, (byte)26, (byte)58, (byte)163, (byte)106, (byte)7, (byte)196, (byte)129, (byte)146}));
                Debug.Assert(pack.target_network == (byte)(byte)234);
                Debug.Assert(pack.message_type == (ushort)(ushort)26366);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)227;
            p248.message_type = (ushort)(ushort)26366;
            p248.target_network = (byte)(byte)234;
            p248.target_system = (byte)(byte)150;
            p248.payload_SET(new byte[] {(byte)27, (byte)165, (byte)194, (byte)37, (byte)82, (byte)0, (byte)69, (byte)128, (byte)237, (byte)195, (byte)153, (byte)209, (byte)80, (byte)49, (byte)239, (byte)200, (byte)87, (byte)126, (byte)112, (byte)118, (byte)187, (byte)137, (byte)73, (byte)31, (byte)95, (byte)209, (byte)74, (byte)191, (byte)150, (byte)247, (byte)79, (byte)7, (byte)140, (byte)239, (byte)157, (byte)171, (byte)250, (byte)160, (byte)213, (byte)206, (byte)246, (byte)184, (byte)206, (byte)234, (byte)143, (byte)171, (byte)21, (byte)205, (byte)201, (byte)96, (byte)145, (byte)190, (byte)53, (byte)35, (byte)28, (byte)166, (byte)77, (byte)206, (byte)196, (byte)230, (byte)56, (byte)198, (byte)131, (byte)181, (byte)220, (byte)53, (byte)34, (byte)155, (byte)164, (byte)183, (byte)95, (byte)243, (byte)30, (byte)60, (byte)97, (byte)188, (byte)171, (byte)238, (byte)208, (byte)94, (byte)99, (byte)93, (byte)209, (byte)245, (byte)114, (byte)41, (byte)54, (byte)173, (byte)234, (byte)214, (byte)131, (byte)101, (byte)136, (byte)188, (byte)159, (byte)127, (byte)228, (byte)74, (byte)206, (byte)229, (byte)78, (byte)154, (byte)73, (byte)133, (byte)50, (byte)221, (byte)215, (byte)80, (byte)162, (byte)98, (byte)177, (byte)223, (byte)67, (byte)86, (byte)236, (byte)250, (byte)77, (byte)235, (byte)212, (byte)165, (byte)20, (byte)142, (byte)222, (byte)193, (byte)16, (byte)27, (byte)214, (byte)201, (byte)167, (byte)29, (byte)91, (byte)117, (byte)77, (byte)59, (byte)4, (byte)94, (byte)207, (byte)43, (byte)86, (byte)225, (byte)222, (byte)223, (byte)247, (byte)239, (byte)240, (byte)179, (byte)162, (byte)73, (byte)174, (byte)229, (byte)219, (byte)195, (byte)56, (byte)212, (byte)8, (byte)186, (byte)157, (byte)139, (byte)249, (byte)144, (byte)245, (byte)119, (byte)158, (byte)110, (byte)58, (byte)116, (byte)128, (byte)150, (byte)11, (byte)59, (byte)166, (byte)96, (byte)103, (byte)27, (byte)49, (byte)234, (byte)192, (byte)75, (byte)147, (byte)10, (byte)108, (byte)241, (byte)129, (byte)223, (byte)83, (byte)148, (byte)255, (byte)227, (byte)146, (byte)65, (byte)17, (byte)133, (byte)55, (byte)57, (byte)247, (byte)53, (byte)105, (byte)55, (byte)7, (byte)87, (byte)60, (byte)237, (byte)212, (byte)4, (byte)112, (byte)249, (byte)174, (byte)140, (byte)252, (byte)2, (byte)227, (byte)62, (byte)86, (byte)144, (byte)56, (byte)19, (byte)29, (byte)129, (byte)71, (byte)46, (byte)208, (byte)126, (byte)52, (byte)50, (byte)75, (byte)204, (byte)35, (byte)79, (byte)139, (byte)64, (byte)209, (byte)24, (byte)8, (byte)249, (byte)32, (byte)203, (byte)163, (byte)33, (byte)79, (byte)83, (byte)36, (byte)26, (byte)58, (byte)163, (byte)106, (byte)7, (byte)196, (byte)129, (byte)146}, 0) ;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)26256);
                Debug.Assert(pack.ver == (byte)(byte)93);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 113, (sbyte)125, (sbyte)108, (sbyte) - 102, (sbyte)61, (sbyte) - 21, (sbyte) - 115, (sbyte)112, (sbyte)13, (sbyte)41, (sbyte) - 99, (sbyte) - 125, (sbyte) - 66, (sbyte)54, (sbyte)124, (sbyte) - 42, (sbyte) - 107, (sbyte)30, (sbyte)19, (sbyte) - 113, (sbyte)9, (sbyte)19, (sbyte) - 62, (sbyte)45, (sbyte)125, (sbyte) - 89, (sbyte)0, (sbyte)82, (sbyte) - 64, (sbyte)98, (sbyte)33, (sbyte)50}));
                Debug.Assert(pack.type == (byte)(byte)134);
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte) - 113, (sbyte)125, (sbyte)108, (sbyte) - 102, (sbyte)61, (sbyte) - 21, (sbyte) - 115, (sbyte)112, (sbyte)13, (sbyte)41, (sbyte) - 99, (sbyte) - 125, (sbyte) - 66, (sbyte)54, (sbyte)124, (sbyte) - 42, (sbyte) - 107, (sbyte)30, (sbyte)19, (sbyte) - 113, (sbyte)9, (sbyte)19, (sbyte) - 62, (sbyte)45, (sbyte)125, (sbyte) - 89, (sbyte)0, (sbyte)82, (sbyte) - 64, (sbyte)98, (sbyte)33, (sbyte)50}, 0) ;
            p249.type = (byte)(byte)134;
            p249.address = (ushort)(ushort)26256;
            p249.ver = (byte)(byte)93;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("koAglhbza"));
                Debug.Assert(pack.x == (float) -1.524166E37F);
                Debug.Assert(pack.time_usec == (ulong)8194385513697123504L);
                Debug.Assert(pack.z == (float)1.0336614E38F);
                Debug.Assert(pack.y == (float) -2.2011623E38F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)8194385513697123504L;
            p250.name_SET("koAglhbza", PH) ;
            p250.x = (float) -1.524166E37F;
            p250.y = (float) -2.2011623E38F;
            p250.z = (float)1.0336614E38F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("nbclrNula"));
                Debug.Assert(pack.value == (float)6.575701E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4100688410U);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float)6.575701E37F;
            p251.time_boot_ms = (uint)4100688410U;
            p251.name_SET("nbclrNula", PH) ;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("Wowda"));
                Debug.Assert(pack.time_boot_ms == (uint)2227241214U);
                Debug.Assert(pack.value == (int) -1469569601);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2227241214U;
            p252.name_SET("Wowda", PH) ;
            p252.value = (int) -1469569601;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ALERT);
                Debug.Assert(pack.text_LEN(ph) == 13);
                Debug.Assert(pack.text_TRY(ph).Equals("csydukpvhzoqO"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ALERT;
            p253.text_SET("csydukpvhzoqO", PH) ;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1114538748U);
                Debug.Assert(pack.value == (float)1.8327956E38F);
                Debug.Assert(pack.ind == (byte)(byte)186);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)186;
            p254.time_boot_ms = (uint)1114538748U;
            p254.value = (float)1.8327956E38F;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)253);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)23, (byte)224, (byte)79, (byte)116, (byte)167, (byte)58, (byte)214, (byte)112, (byte)198, (byte)78, (byte)89, (byte)13, (byte)165, (byte)240, (byte)203, (byte)241, (byte)250, (byte)156, (byte)136, (byte)237, (byte)199, (byte)50, (byte)34, (byte)65, (byte)232, (byte)157, (byte)107, (byte)62, (byte)240, (byte)199, (byte)66, (byte)255}));
                Debug.Assert(pack.initial_timestamp == (ulong)8582216198040446428L);
                Debug.Assert(pack.target_system == (byte)(byte)102);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_component = (byte)(byte)253;
            p256.target_system = (byte)(byte)102;
            p256.secret_key_SET(new byte[] {(byte)23, (byte)224, (byte)79, (byte)116, (byte)167, (byte)58, (byte)214, (byte)112, (byte)198, (byte)78, (byte)89, (byte)13, (byte)165, (byte)240, (byte)203, (byte)241, (byte)250, (byte)156, (byte)136, (byte)237, (byte)199, (byte)50, (byte)34, (byte)65, (byte)232, (byte)157, (byte)107, (byte)62, (byte)240, (byte)199, (byte)66, (byte)255}, 0) ;
            p256.initial_timestamp = (ulong)8582216198040446428L;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)91937020U);
                Debug.Assert(pack.state == (byte)(byte)225);
                Debug.Assert(pack.last_change_ms == (uint)2543408084U);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2543408084U;
            p257.time_boot_ms = (uint)91937020U;
            p257.state = (byte)(byte)225;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)81);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.tune_LEN(ph) == 23);
                Debug.Assert(pack.tune_TRY(ph).Equals("HifcnvizdxYgziigondsyqm"));
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("HifcnvizdxYgziigondsyqm", PH) ;
            p258.target_component = (byte)(byte)81;
            p258.target_system = (byte)(byte)140;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2294062632U);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)113, (byte)118, (byte)93, (byte)170, (byte)205, (byte)211, (byte)222, (byte)39, (byte)180, (byte)4, (byte)105, (byte)172, (byte)129, (byte)144, (byte)179, (byte)44, (byte)4, (byte)24, (byte)9, (byte)160, (byte)135, (byte)79, (byte)193, (byte)186, (byte)125, (byte)48, (byte)184, (byte)105, (byte)69, (byte)181, (byte)129, (byte)11}));
                Debug.Assert(pack.sensor_size_v == (float)3.485169E37F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)47, (byte)247, (byte)123, (byte)233, (byte)165, (byte)184, (byte)50, (byte)255, (byte)6, (byte)159, (byte)122, (byte)78, (byte)93, (byte)46, (byte)30, (byte)13, (byte)60, (byte)151, (byte)200, (byte)185, (byte)179, (byte)132, (byte)77, (byte)116, (byte)124, (byte)202, (byte)22, (byte)163, (byte)176, (byte)159, (byte)105, (byte)157}));
                Debug.Assert(pack.firmware_version == (uint)994825365U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)14430);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)63619);
                Debug.Assert(pack.focal_length == (float)3.1680848E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)42927);
                Debug.Assert(pack.lens_id == (byte)(byte)51);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 61);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("gmxnnydzgpxnyhfOrtejlbxoqkomnwmrivpqgearmrGzydjbztcvjxdBxoyTg"));
                Debug.Assert(pack.sensor_size_h == (float) -2.6769256E38F);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_version = (ushort)(ushort)14430;
            p259.model_name_SET(new byte[] {(byte)113, (byte)118, (byte)93, (byte)170, (byte)205, (byte)211, (byte)222, (byte)39, (byte)180, (byte)4, (byte)105, (byte)172, (byte)129, (byte)144, (byte)179, (byte)44, (byte)4, (byte)24, (byte)9, (byte)160, (byte)135, (byte)79, (byte)193, (byte)186, (byte)125, (byte)48, (byte)184, (byte)105, (byte)69, (byte)181, (byte)129, (byte)11}, 0) ;
            p259.sensor_size_h = (float) -2.6769256E38F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.focal_length = (float)3.1680848E38F;
            p259.vendor_name_SET(new byte[] {(byte)47, (byte)247, (byte)123, (byte)233, (byte)165, (byte)184, (byte)50, (byte)255, (byte)6, (byte)159, (byte)122, (byte)78, (byte)93, (byte)46, (byte)30, (byte)13, (byte)60, (byte)151, (byte)200, (byte)185, (byte)179, (byte)132, (byte)77, (byte)116, (byte)124, (byte)202, (byte)22, (byte)163, (byte)176, (byte)159, (byte)105, (byte)157}, 0) ;
            p259.resolution_h = (ushort)(ushort)42927;
            p259.lens_id = (byte)(byte)51;
            p259.resolution_v = (ushort)(ushort)63619;
            p259.time_boot_ms = (uint)2294062632U;
            p259.firmware_version = (uint)994825365U;
            p259.sensor_size_v = (float)3.485169E37F;
            p259.cam_definition_uri_SET("gmxnnydzgpxnyhfOrtejlbxoqkomnwmrivpqgearmrGzydjbztcvjxdBxoyTg", PH) ;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO);
                Debug.Assert(pack.time_boot_ms == (uint)1942051728U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1942051728U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.write_speed == (float)2.591959E38F);
                Debug.Assert(pack.total_capacity == (float) -3.8964912E37F);
                Debug.Assert(pack.used_capacity == (float) -1.3831469E38F);
                Debug.Assert(pack.read_speed == (float)2.3676206E38F);
                Debug.Assert(pack.available_capacity == (float)2.5415102E37F);
                Debug.Assert(pack.storage_id == (byte)(byte)59);
                Debug.Assert(pack.status == (byte)(byte)41);
                Debug.Assert(pack.time_boot_ms == (uint)2564001872U);
                Debug.Assert(pack.storage_count == (byte)(byte)130);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -3.8964912E37F;
            p261.storage_count = (byte)(byte)130;
            p261.read_speed = (float)2.3676206E38F;
            p261.time_boot_ms = (uint)2564001872U;
            p261.write_speed = (float)2.591959E38F;
            p261.status = (byte)(byte)41;
            p261.available_capacity = (float)2.5415102E37F;
            p261.used_capacity = (float) -1.3831469E38F;
            p261.storage_id = (byte)(byte)59;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float) -2.1827376E38F);
                Debug.Assert(pack.image_status == (byte)(byte)158);
                Debug.Assert(pack.image_interval == (float)2.9846566E38F);
                Debug.Assert(pack.video_status == (byte)(byte)168);
                Debug.Assert(pack.recording_time_ms == (uint)2648667980U);
                Debug.Assert(pack.time_boot_ms == (uint)3661731596U);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)168;
            p262.image_interval = (float)2.9846566E38F;
            p262.image_status = (byte)(byte)158;
            p262.recording_time_ms = (uint)2648667980U;
            p262.available_capacity = (float) -2.1827376E38F;
            p262.time_boot_ms = (uint)3661731596U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1062894632);
                Debug.Assert(pack.image_index == (int)382829763);
                Debug.Assert(pack.relative_alt == (int) -2059538136);
                Debug.Assert(pack.file_url_LEN(ph) == 7);
                Debug.Assert(pack.file_url_TRY(ph).Equals("kqzbjxr"));
                Debug.Assert(pack.camera_id == (byte)(byte)197);
                Debug.Assert(pack.alt == (int)1322787485);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-4.400316E37F, -2.2372621E38F, 2.394314E38F, 1.7281502E37F}));
                Debug.Assert(pack.time_utc == (ulong)8212485800896440984L);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)67);
                Debug.Assert(pack.lon == (int)992127460);
                Debug.Assert(pack.time_boot_ms == (uint)1527671229U);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_utc = (ulong)8212485800896440984L;
            p263.q_SET(new float[] {-4.400316E37F, -2.2372621E38F, 2.394314E38F, 1.7281502E37F}, 0) ;
            p263.image_index = (int)382829763;
            p263.lat = (int)1062894632;
            p263.capture_result = (sbyte)(sbyte)67;
            p263.time_boot_ms = (uint)1527671229U;
            p263.lon = (int)992127460;
            p263.camera_id = (byte)(byte)197;
            p263.file_url_SET("kqzbjxr", PH) ;
            p263.alt = (int)1322787485;
            p263.relative_alt = (int) -2059538136;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)6815990270889063071L);
                Debug.Assert(pack.flight_uuid == (ulong)7481273173727274139L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)4060257554067699694L);
                Debug.Assert(pack.time_boot_ms == (uint)3163949121U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)3163949121U;
            p264.arming_time_utc = (ulong)6815990270889063071L;
            p264.flight_uuid = (ulong)7481273173727274139L;
            p264.takeoff_time_utc = (ulong)4060257554067699694L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -7.819467E37F);
                Debug.Assert(pack.yaw == (float)1.2289514E38F);
                Debug.Assert(pack.pitch == (float) -8.319235E37F);
                Debug.Assert(pack.time_boot_ms == (uint)476944314U);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)476944314U;
            p265.pitch = (float) -8.319235E37F;
            p265.roll = (float) -7.819467E37F;
            p265.yaw = (float)1.2289514E38F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)110);
                Debug.Assert(pack.first_message_offset == (byte)(byte)28);
                Debug.Assert(pack.sequence == (ushort)(ushort)26147);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)160, (byte)197, (byte)118, (byte)159, (byte)131, (byte)255, (byte)108, (byte)30, (byte)239, (byte)129, (byte)184, (byte)170, (byte)12, (byte)234, (byte)253, (byte)243, (byte)106, (byte)115, (byte)220, (byte)205, (byte)121, (byte)200, (byte)37, (byte)131, (byte)75, (byte)229, (byte)197, (byte)7, (byte)224, (byte)159, (byte)102, (byte)89, (byte)29, (byte)39, (byte)79, (byte)231, (byte)27, (byte)241, (byte)74, (byte)71, (byte)75, (byte)98, (byte)238, (byte)165, (byte)83, (byte)175, (byte)201, (byte)223, (byte)154, (byte)58, (byte)160, (byte)5, (byte)48, (byte)25, (byte)48, (byte)160, (byte)150, (byte)205, (byte)81, (byte)43, (byte)164, (byte)214, (byte)208, (byte)91, (byte)3, (byte)56, (byte)88, (byte)50, (byte)178, (byte)142, (byte)26, (byte)15, (byte)177, (byte)138, (byte)151, (byte)178, (byte)108, (byte)172, (byte)123, (byte)17, (byte)103, (byte)1, (byte)187, (byte)50, (byte)117, (byte)125, (byte)18, (byte)220, (byte)44, (byte)233, (byte)172, (byte)161, (byte)28, (byte)108, (byte)69, (byte)171, (byte)50, (byte)152, (byte)121, (byte)218, (byte)231, (byte)93, (byte)184, (byte)3, (byte)79, (byte)2, (byte)53, (byte)11, (byte)71, (byte)177, (byte)144, (byte)22, (byte)80, (byte)14, (byte)70, (byte)90, (byte)86, (byte)145, (byte)136, (byte)115, (byte)230, (byte)76, (byte)250, (byte)89, (byte)59, (byte)182, (byte)180, (byte)181, (byte)221, (byte)152, (byte)130, (byte)156, (byte)179, (byte)99, (byte)144, (byte)108, (byte)152, (byte)126, (byte)147, (byte)210, (byte)250, (byte)102, (byte)123, (byte)70, (byte)159, (byte)181, (byte)248, (byte)49, (byte)80, (byte)138, (byte)230, (byte)69, (byte)29, (byte)0, (byte)41, (byte)134, (byte)57, (byte)95, (byte)139, (byte)72, (byte)176, (byte)251, (byte)151, (byte)250, (byte)225, (byte)251, (byte)26, (byte)188, (byte)209, (byte)238, (byte)32, (byte)223, (byte)142, (byte)247, (byte)201, (byte)188, (byte)33, (byte)147, (byte)0, (byte)35, (byte)175, (byte)167, (byte)76, (byte)209, (byte)54, (byte)173, (byte)218, (byte)80, (byte)95, (byte)129, (byte)181, (byte)84, (byte)101, (byte)151, (byte)184, (byte)227, (byte)244, (byte)42, (byte)130, (byte)171, (byte)232, (byte)70, (byte)212, (byte)46, (byte)70, (byte)122, (byte)16, (byte)223, (byte)85, (byte)252, (byte)2, (byte)244, (byte)233, (byte)205, (byte)246, (byte)107, (byte)154, (byte)203, (byte)192, (byte)198, (byte)203, (byte)199, (byte)10, (byte)68, (byte)174, (byte)93, (byte)107, (byte)207, (byte)103, (byte)244, (byte)58, (byte)153, (byte)206, (byte)45, (byte)148, (byte)81, (byte)119, (byte)2, (byte)218, (byte)138, (byte)198, (byte)13, (byte)26, (byte)21, (byte)174, (byte)71, (byte)144, (byte)95, (byte)132}));
                Debug.Assert(pack.target_component == (byte)(byte)214);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)26147;
            p266.data__SET(new byte[] {(byte)160, (byte)197, (byte)118, (byte)159, (byte)131, (byte)255, (byte)108, (byte)30, (byte)239, (byte)129, (byte)184, (byte)170, (byte)12, (byte)234, (byte)253, (byte)243, (byte)106, (byte)115, (byte)220, (byte)205, (byte)121, (byte)200, (byte)37, (byte)131, (byte)75, (byte)229, (byte)197, (byte)7, (byte)224, (byte)159, (byte)102, (byte)89, (byte)29, (byte)39, (byte)79, (byte)231, (byte)27, (byte)241, (byte)74, (byte)71, (byte)75, (byte)98, (byte)238, (byte)165, (byte)83, (byte)175, (byte)201, (byte)223, (byte)154, (byte)58, (byte)160, (byte)5, (byte)48, (byte)25, (byte)48, (byte)160, (byte)150, (byte)205, (byte)81, (byte)43, (byte)164, (byte)214, (byte)208, (byte)91, (byte)3, (byte)56, (byte)88, (byte)50, (byte)178, (byte)142, (byte)26, (byte)15, (byte)177, (byte)138, (byte)151, (byte)178, (byte)108, (byte)172, (byte)123, (byte)17, (byte)103, (byte)1, (byte)187, (byte)50, (byte)117, (byte)125, (byte)18, (byte)220, (byte)44, (byte)233, (byte)172, (byte)161, (byte)28, (byte)108, (byte)69, (byte)171, (byte)50, (byte)152, (byte)121, (byte)218, (byte)231, (byte)93, (byte)184, (byte)3, (byte)79, (byte)2, (byte)53, (byte)11, (byte)71, (byte)177, (byte)144, (byte)22, (byte)80, (byte)14, (byte)70, (byte)90, (byte)86, (byte)145, (byte)136, (byte)115, (byte)230, (byte)76, (byte)250, (byte)89, (byte)59, (byte)182, (byte)180, (byte)181, (byte)221, (byte)152, (byte)130, (byte)156, (byte)179, (byte)99, (byte)144, (byte)108, (byte)152, (byte)126, (byte)147, (byte)210, (byte)250, (byte)102, (byte)123, (byte)70, (byte)159, (byte)181, (byte)248, (byte)49, (byte)80, (byte)138, (byte)230, (byte)69, (byte)29, (byte)0, (byte)41, (byte)134, (byte)57, (byte)95, (byte)139, (byte)72, (byte)176, (byte)251, (byte)151, (byte)250, (byte)225, (byte)251, (byte)26, (byte)188, (byte)209, (byte)238, (byte)32, (byte)223, (byte)142, (byte)247, (byte)201, (byte)188, (byte)33, (byte)147, (byte)0, (byte)35, (byte)175, (byte)167, (byte)76, (byte)209, (byte)54, (byte)173, (byte)218, (byte)80, (byte)95, (byte)129, (byte)181, (byte)84, (byte)101, (byte)151, (byte)184, (byte)227, (byte)244, (byte)42, (byte)130, (byte)171, (byte)232, (byte)70, (byte)212, (byte)46, (byte)70, (byte)122, (byte)16, (byte)223, (byte)85, (byte)252, (byte)2, (byte)244, (byte)233, (byte)205, (byte)246, (byte)107, (byte)154, (byte)203, (byte)192, (byte)198, (byte)203, (byte)199, (byte)10, (byte)68, (byte)174, (byte)93, (byte)107, (byte)207, (byte)103, (byte)244, (byte)58, (byte)153, (byte)206, (byte)45, (byte)148, (byte)81, (byte)119, (byte)2, (byte)218, (byte)138, (byte)198, (byte)13, (byte)26, (byte)21, (byte)174, (byte)71, (byte)144, (byte)95, (byte)132}, 0) ;
            p266.target_component = (byte)(byte)214;
            p266.first_message_offset = (byte)(byte)28;
            p266.length = (byte)(byte)110;
            p266.target_system = (byte)(byte)228;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)119, (byte)139, (byte)223, (byte)56, (byte)230, (byte)189, (byte)128, (byte)17, (byte)242, (byte)54, (byte)52, (byte)132, (byte)191, (byte)31, (byte)124, (byte)6, (byte)174, (byte)46, (byte)83, (byte)250, (byte)185, (byte)226, (byte)183, (byte)218, (byte)39, (byte)128, (byte)243, (byte)14, (byte)68, (byte)154, (byte)132, (byte)133, (byte)8, (byte)142, (byte)56, (byte)23, (byte)160, (byte)160, (byte)203, (byte)203, (byte)139, (byte)56, (byte)241, (byte)105, (byte)244, (byte)142, (byte)169, (byte)216, (byte)103, (byte)226, (byte)54, (byte)25, (byte)11, (byte)93, (byte)19, (byte)167, (byte)61, (byte)51, (byte)246, (byte)125, (byte)135, (byte)69, (byte)175, (byte)38, (byte)65, (byte)240, (byte)174, (byte)251, (byte)108, (byte)198, (byte)212, (byte)27, (byte)144, (byte)156, (byte)61, (byte)59, (byte)86, (byte)126, (byte)171, (byte)0, (byte)153, (byte)162, (byte)63, (byte)179, (byte)5, (byte)210, (byte)99, (byte)251, (byte)207, (byte)22, (byte)154, (byte)115, (byte)221, (byte)199, (byte)124, (byte)152, (byte)193, (byte)192, (byte)108, (byte)35, (byte)174, (byte)45, (byte)219, (byte)201, (byte)104, (byte)243, (byte)1, (byte)184, (byte)94, (byte)45, (byte)240, (byte)217, (byte)236, (byte)31, (byte)71, (byte)158, (byte)196, (byte)81, (byte)186, (byte)186, (byte)217, (byte)246, (byte)46, (byte)142, (byte)151, (byte)2, (byte)10, (byte)68, (byte)181, (byte)6, (byte)171, (byte)120, (byte)199, (byte)168, (byte)78, (byte)162, (byte)240, (byte)145, (byte)17, (byte)101, (byte)16, (byte)107, (byte)131, (byte)81, (byte)233, (byte)141, (byte)76, (byte)0, (byte)29, (byte)230, (byte)65, (byte)193, (byte)107, (byte)67, (byte)80, (byte)7, (byte)185, (byte)34, (byte)83, (byte)188, (byte)239, (byte)27, (byte)153, (byte)98, (byte)80, (byte)51, (byte)111, (byte)67, (byte)185, (byte)244, (byte)56, (byte)248, (byte)71, (byte)78, (byte)35, (byte)139, (byte)184, (byte)214, (byte)91, (byte)166, (byte)85, (byte)10, (byte)48, (byte)133, (byte)81, (byte)137, (byte)152, (byte)228, (byte)153, (byte)157, (byte)112, (byte)223, (byte)30, (byte)13, (byte)89, (byte)188, (byte)152, (byte)187, (byte)223, (byte)57, (byte)14, (byte)185, (byte)6, (byte)171, (byte)66, (byte)51, (byte)86, (byte)20, (byte)28, (byte)12, (byte)38, (byte)10, (byte)142, (byte)69, (byte)162, (byte)66, (byte)128, (byte)105, (byte)133, (byte)119, (byte)13, (byte)249, (byte)110, (byte)124, (byte)23, (byte)103, (byte)91, (byte)151, (byte)207, (byte)198, (byte)100, (byte)78, (byte)94, (byte)81, (byte)135, (byte)199, (byte)213, (byte)15, (byte)60, (byte)253, (byte)150, (byte)187, (byte)127, (byte)239, (byte)77, (byte)159, (byte)243, (byte)161, (byte)13}));
                Debug.Assert(pack.target_system == (byte)(byte)252);
                Debug.Assert(pack.length == (byte)(byte)90);
                Debug.Assert(pack.first_message_offset == (byte)(byte)149);
                Debug.Assert(pack.sequence == (ushort)(ushort)13386);
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)71;
            p267.first_message_offset = (byte)(byte)149;
            p267.length = (byte)(byte)90;
            p267.target_system = (byte)(byte)252;
            p267.data__SET(new byte[] {(byte)119, (byte)139, (byte)223, (byte)56, (byte)230, (byte)189, (byte)128, (byte)17, (byte)242, (byte)54, (byte)52, (byte)132, (byte)191, (byte)31, (byte)124, (byte)6, (byte)174, (byte)46, (byte)83, (byte)250, (byte)185, (byte)226, (byte)183, (byte)218, (byte)39, (byte)128, (byte)243, (byte)14, (byte)68, (byte)154, (byte)132, (byte)133, (byte)8, (byte)142, (byte)56, (byte)23, (byte)160, (byte)160, (byte)203, (byte)203, (byte)139, (byte)56, (byte)241, (byte)105, (byte)244, (byte)142, (byte)169, (byte)216, (byte)103, (byte)226, (byte)54, (byte)25, (byte)11, (byte)93, (byte)19, (byte)167, (byte)61, (byte)51, (byte)246, (byte)125, (byte)135, (byte)69, (byte)175, (byte)38, (byte)65, (byte)240, (byte)174, (byte)251, (byte)108, (byte)198, (byte)212, (byte)27, (byte)144, (byte)156, (byte)61, (byte)59, (byte)86, (byte)126, (byte)171, (byte)0, (byte)153, (byte)162, (byte)63, (byte)179, (byte)5, (byte)210, (byte)99, (byte)251, (byte)207, (byte)22, (byte)154, (byte)115, (byte)221, (byte)199, (byte)124, (byte)152, (byte)193, (byte)192, (byte)108, (byte)35, (byte)174, (byte)45, (byte)219, (byte)201, (byte)104, (byte)243, (byte)1, (byte)184, (byte)94, (byte)45, (byte)240, (byte)217, (byte)236, (byte)31, (byte)71, (byte)158, (byte)196, (byte)81, (byte)186, (byte)186, (byte)217, (byte)246, (byte)46, (byte)142, (byte)151, (byte)2, (byte)10, (byte)68, (byte)181, (byte)6, (byte)171, (byte)120, (byte)199, (byte)168, (byte)78, (byte)162, (byte)240, (byte)145, (byte)17, (byte)101, (byte)16, (byte)107, (byte)131, (byte)81, (byte)233, (byte)141, (byte)76, (byte)0, (byte)29, (byte)230, (byte)65, (byte)193, (byte)107, (byte)67, (byte)80, (byte)7, (byte)185, (byte)34, (byte)83, (byte)188, (byte)239, (byte)27, (byte)153, (byte)98, (byte)80, (byte)51, (byte)111, (byte)67, (byte)185, (byte)244, (byte)56, (byte)248, (byte)71, (byte)78, (byte)35, (byte)139, (byte)184, (byte)214, (byte)91, (byte)166, (byte)85, (byte)10, (byte)48, (byte)133, (byte)81, (byte)137, (byte)152, (byte)228, (byte)153, (byte)157, (byte)112, (byte)223, (byte)30, (byte)13, (byte)89, (byte)188, (byte)152, (byte)187, (byte)223, (byte)57, (byte)14, (byte)185, (byte)6, (byte)171, (byte)66, (byte)51, (byte)86, (byte)20, (byte)28, (byte)12, (byte)38, (byte)10, (byte)142, (byte)69, (byte)162, (byte)66, (byte)128, (byte)105, (byte)133, (byte)119, (byte)13, (byte)249, (byte)110, (byte)124, (byte)23, (byte)103, (byte)91, (byte)151, (byte)207, (byte)198, (byte)100, (byte)78, (byte)94, (byte)81, (byte)135, (byte)199, (byte)213, (byte)15, (byte)60, (byte)253, (byte)150, (byte)187, (byte)127, (byte)239, (byte)77, (byte)159, (byte)243, (byte)161, (byte)13}, 0) ;
            p267.sequence = (ushort)(ushort)13386;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.sequence == (ushort)(ushort)27389);
                Debug.Assert(pack.target_system == (byte)(byte)176);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)63;
            p268.sequence = (ushort)(ushort)27389;
            p268.target_system = (byte)(byte)176;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rotation == (ushort)(ushort)22882);
                Debug.Assert(pack.bitrate == (uint)3313136769U);
                Debug.Assert(pack.framerate == (float)3.117164E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)56462);
                Debug.Assert(pack.status == (byte)(byte)131);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)22530);
                Debug.Assert(pack.camera_id == (byte)(byte)8);
                Debug.Assert(pack.uri_LEN(ph) == 85);
                Debug.Assert(pack.uri_TRY(ph).Equals("wsvvlgbauSnScsmLdDgrbtwmqywjtkljxyuIszprhwutmbfrfvhKnCkznhhpabinzxwihxmxsoavbNxtgugEq"));
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)131;
            p269.resolution_v = (ushort)(ushort)22530;
            p269.bitrate = (uint)3313136769U;
            p269.resolution_h = (ushort)(ushort)56462;
            p269.uri_SET("wsvvlgbauSnScsmLdDgrbtwmqywjtkljxyuIszprhwutmbfrfvhKnCkznhhpabinzxwihxmxsoavbNxtgugEq", PH) ;
            p269.framerate = (float)3.117164E38F;
            p269.rotation = (ushort)(ushort)22882;
            p269.camera_id = (byte)(byte)8;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)6.39017E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)44097);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)27794);
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.rotation == (ushort)(ushort)28824);
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.uri_LEN(ph) == 111);
                Debug.Assert(pack.uri_TRY(ph).Equals("zPpmdsiclqwljmhjbaeqtYslsglwrgyzdmjcsojinrlevdyjhqncylpxjjychxteqnfmwhimshmfDUrrwebfXirqfmvyfctqhnSdykpwftNutku"));
                Debug.Assert(pack.bitrate == (uint)2851219201U);
                Debug.Assert(pack.camera_id == (byte)(byte)239);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.rotation = (ushort)(ushort)28824;
            p270.camera_id = (byte)(byte)239;
            p270.target_system = (byte)(byte)175;
            p270.resolution_h = (ushort)(ushort)44097;
            p270.uri_SET("zPpmdsiclqwljmhjbaeqtYslsglwrgyzdmjcsojinrlevdyjhqncylpxjjychxteqnfmwhimshmfDUrrwebfXirqfmvyfctqhnSdykpwftNutku", PH) ;
            p270.bitrate = (uint)2851219201U;
            p270.resolution_v = (ushort)(ushort)27794;
            p270.framerate = (float)6.39017E37F;
            p270.target_component = (byte)(byte)173;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 17);
                Debug.Assert(pack.password_TRY(ph).Equals("xubgMyHxjtLelkpjt"));
                Debug.Assert(pack.ssid_LEN(ph) == 16);
                Debug.Assert(pack.ssid_TRY(ph).Equals("vzyayocpoLlkvaea"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("xubgMyHxjtLelkpjt", PH) ;
            p299.ssid_SET("vzyayocpoLlkvaea", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)93, (byte)147, (byte)210, (byte)43, (byte)156, (byte)213, (byte)37, (byte)185}));
                Debug.Assert(pack.min_version == (ushort)(ushort)36964);
                Debug.Assert(pack.max_version == (ushort)(ushort)46227);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)177, (byte)30, (byte)34, (byte)130, (byte)143, (byte)162, (byte)119, (byte)150}));
                Debug.Assert(pack.version == (ushort)(ushort)48323);
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)36964;
            p300.version = (ushort)(ushort)48323;
            p300.spec_version_hash_SET(new byte[] {(byte)177, (byte)30, (byte)34, (byte)130, (byte)143, (byte)162, (byte)119, (byte)150}, 0) ;
            p300.max_version = (ushort)(ushort)46227;
            p300.library_version_hash_SET(new byte[] {(byte)93, (byte)147, (byte)210, (byte)43, (byte)156, (byte)213, (byte)37, (byte)185}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)23383);
                Debug.Assert(pack.uptime_sec == (uint)2678381938U);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.sub_mode == (byte)(byte)107);
                Debug.Assert(pack.time_usec == (ulong)3574471356816737185L);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.sub_mode = (byte)(byte)107;
            p310.vendor_specific_status_code = (ushort)(ushort)23383;
            p310.uptime_sec = (uint)2678381938U;
            p310.time_usec = (ulong)3574471356816737185L;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_minor == (byte)(byte)119);
                Debug.Assert(pack.uptime_sec == (uint)1483769576U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)195, (byte)174, (byte)233, (byte)56, (byte)206, (byte)182, (byte)103, (byte)135, (byte)243, (byte)243, (byte)119, (byte)128, (byte)145, (byte)24, (byte)118, (byte)84}));
                Debug.Assert(pack.sw_version_major == (byte)(byte)82);
                Debug.Assert(pack.time_usec == (ulong)7279837005794964044L);
                Debug.Assert(pack.sw_vcs_commit == (uint)1099166196U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)50);
                Debug.Assert(pack.hw_version_major == (byte)(byte)29);
                Debug.Assert(pack.name_LEN(ph) == 59);
                Debug.Assert(pack.name_TRY(ph).Equals("limxrkqbuzqkhtumateolqehmjreejinusadnbJwayMdohbtzvyGyfxjirr"));
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_unique_id_SET(new byte[] {(byte)195, (byte)174, (byte)233, (byte)56, (byte)206, (byte)182, (byte)103, (byte)135, (byte)243, (byte)243, (byte)119, (byte)128, (byte)145, (byte)24, (byte)118, (byte)84}, 0) ;
            p311.hw_version_minor = (byte)(byte)119;
            p311.name_SET("limxrkqbuzqkhtumateolqehmjreejinusadnbJwayMdohbtzvyGyfxjirr", PH) ;
            p311.sw_version_minor = (byte)(byte)50;
            p311.sw_version_major = (byte)(byte)82;
            p311.sw_vcs_commit = (uint)1099166196U;
            p311.time_usec = (ulong)7279837005794964044L;
            p311.uptime_sec = (uint)1483769576U;
            p311.hw_version_major = (byte)(byte)29;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)230);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("NedjdultdorZnd"));
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.param_index == (short)(short)4159);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)4159;
            p320.param_id_SET("NedjdultdorZnd", PH) ;
            p320.target_component = (byte)(byte)230;
            p320.target_system = (byte)(byte)73;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.target_system == (byte)(byte)241);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)241;
            p321.target_component = (byte)(byte)122;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)9270);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_value_LEN(ph) == 39);
                Debug.Assert(pack.param_value_TRY(ph).Equals("lnnjyYsyHlbqcsiNayfftamrucrzesdkzngJZuq"));
                Debug.Assert(pack.param_count == (ushort)(ushort)25130);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rygCgsztlJjq"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)9270;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_id_SET("rygCgsztlJjq", PH) ;
            p322.param_value_SET("lnnjyYsyHlbqcsiNayfftamrucrzesdkzngJZuq", PH) ;
            p322.param_count = (ushort)(ushort)25130;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("lnatyt"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_value_LEN(ph) == 44);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ydpvdaztpcrlguYeujxqpmubaDvIdwiufvYestfyhgxz"));
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("lnatyt", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p323.param_value_SET("ydpvdaztpcrlguYeujxqpmubaDvIdwiufvYestfyhgxz", PH) ;
            p323.target_system = (byte)(byte)32;
            p323.target_component = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bhttrywdlxde"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_value_LEN(ph) == 7);
                Debug.Assert(pack.param_value_TRY(ph).Equals("Zrgfwfi"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_id_SET("bhttrywdlxde", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_value_SET("Zrgfwfi", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)57986);
                Debug.Assert(pack.max_distance == (ushort)(ushort)59410);
                Debug.Assert(pack.time_usec == (ulong)1860281408968343904L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)57393, (ushort)11894, (ushort)40219, (ushort)45532, (ushort)34050, (ushort)14126, (ushort)53230, (ushort)4736, (ushort)6719, (ushort)30541, (ushort)5428, (ushort)64437, (ushort)42247, (ushort)46739, (ushort)6881, (ushort)53640, (ushort)33630, (ushort)38642, (ushort)10287, (ushort)13270, (ushort)10214, (ushort)35511, (ushort)51542, (ushort)34643, (ushort)47928, (ushort)47692, (ushort)6614, (ushort)35294, (ushort)5824, (ushort)28782, (ushort)64341, (ushort)44755, (ushort)27962, (ushort)30484, (ushort)28935, (ushort)16307, (ushort)19234, (ushort)55545, (ushort)47779, (ushort)40194, (ushort)45259, (ushort)48001, (ushort)30874, (ushort)43307, (ushort)58299, (ushort)32665, (ushort)28322, (ushort)39295, (ushort)38636, (ushort)34876, (ushort)9553, (ushort)16550, (ushort)26341, (ushort)4319, (ushort)24067, (ushort)60399, (ushort)23046, (ushort)11270, (ushort)9446, (ushort)8634, (ushort)18216, (ushort)24297, (ushort)34587, (ushort)35707, (ushort)5812, (ushort)7774, (ushort)41673, (ushort)2004, (ushort)23978, (ushort)50282, (ushort)29334, (ushort)31354}));
                Debug.Assert(pack.increment == (byte)(byte)175);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)57986;
            p330.max_distance = (ushort)(ushort)59410;
            p330.increment = (byte)(byte)175;
            p330.time_usec = (ulong)1860281408968343904L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[] {(ushort)57393, (ushort)11894, (ushort)40219, (ushort)45532, (ushort)34050, (ushort)14126, (ushort)53230, (ushort)4736, (ushort)6719, (ushort)30541, (ushort)5428, (ushort)64437, (ushort)42247, (ushort)46739, (ushort)6881, (ushort)53640, (ushort)33630, (ushort)38642, (ushort)10287, (ushort)13270, (ushort)10214, (ushort)35511, (ushort)51542, (ushort)34643, (ushort)47928, (ushort)47692, (ushort)6614, (ushort)35294, (ushort)5824, (ushort)28782, (ushort)64341, (ushort)44755, (ushort)27962, (ushort)30484, (ushort)28935, (ushort)16307, (ushort)19234, (ushort)55545, (ushort)47779, (ushort)40194, (ushort)45259, (ushort)48001, (ushort)30874, (ushort)43307, (ushort)58299, (ushort)32665, (ushort)28322, (ushort)39295, (ushort)38636, (ushort)34876, (ushort)9553, (ushort)16550, (ushort)26341, (ushort)4319, (ushort)24067, (ushort)60399, (ushort)23046, (ushort)11270, (ushort)9446, (ushort)8634, (ushort)18216, (ushort)24297, (ushort)34587, (ushort)35707, (ushort)5812, (ushort)7774, (ushort)41673, (ushort)2004, (ushort)23978, (ushort)50282, (ushort)29334, (ushort)31354}, 0) ;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
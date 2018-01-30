
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
                Debug.Assert(pack.custom_mode == (uint)40809497U);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
                Debug.Assert(pack.mavlink_version == (byte)(byte)195);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_FP);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)195;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_FP;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p0.custom_mode = (uint)40809497U;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)13386);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
                Debug.Assert(pack.current_battery == (short)(short)7468);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)40425);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)32645);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)41264);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 88);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)60897);
                Debug.Assert(pack.load == (ushort)(ushort)16471);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)50887);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)4713);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)40425;
            p1.drop_rate_comm = (ushort)(ushort)13386;
            p1.errors_count1 = (ushort)(ushort)41264;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.errors_count3 = (ushort)(ushort)32645;
            p1.battery_remaining = (sbyte)(sbyte) - 88;
            p1.load = (ushort)(ushort)16471;
            p1.voltage_battery = (ushort)(ushort)60897;
            p1.errors_count4 = (ushort)(ushort)4713;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
            p1.current_battery = (short)(short)7468;
            p1.errors_count2 = (ushort)(ushort)50887;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)602661730U);
                Debug.Assert(pack.time_unix_usec == (ulong)4734277037114787309L);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)4734277037114787309L;
            p2.time_boot_ms = (uint)602661730U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3298027431U);
                Debug.Assert(pack.vx == (float) -8.7994E37F);
                Debug.Assert(pack.afx == (float)1.5298786E38F);
                Debug.Assert(pack.yaw_rate == (float) -5.6254774E37F);
                Debug.Assert(pack.afy == (float)6.3259147E37F);
                Debug.Assert(pack.x == (float)2.711185E37F);
                Debug.Assert(pack.z == (float)3.3069226E38F);
                Debug.Assert(pack.vz == (float) -2.3780121E38F);
                Debug.Assert(pack.afz == (float) -2.3187137E38F);
                Debug.Assert(pack.yaw == (float) -3.2295024E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y == (float)4.2884914E37F);
                Debug.Assert(pack.vy == (float) -2.7476635E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)39681);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.time_boot_ms = (uint)3298027431U;
            p3.x = (float)2.711185E37F;
            p3.y = (float)4.2884914E37F;
            p3.vz = (float) -2.3780121E38F;
            p3.yaw_rate = (float) -5.6254774E37F;
            p3.z = (float)3.3069226E38F;
            p3.afy = (float)6.3259147E37F;
            p3.afx = (float)1.5298786E38F;
            p3.yaw = (float) -3.2295024E38F;
            p3.vy = (float) -2.7476635E38F;
            p3.afz = (float) -2.3187137E38F;
            p3.type_mask = (ushort)(ushort)39681;
            p3.vx = (float) -8.7994E37F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7123596165234965742L);
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.seq == (uint)991381089U);
                Debug.Assert(pack.target_system == (byte)(byte)213);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)213;
            p4.target_component = (byte)(byte)247;
            p4.time_usec = (ulong)7123596165234965742L;
            p4.seq = (uint)991381089U;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.control_request == (byte)(byte)72);
                Debug.Assert(pack.version == (byte)(byte)69);
                Debug.Assert(pack.passkey_LEN(ph) == 10);
                Debug.Assert(pack.passkey_TRY(ph).Equals("iklnDwbkmp"));
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)69;
            p5.control_request = (byte)(byte)72;
            p5.target_system = (byte)(byte)152;
            p5.passkey_SET("iklnDwbkmp", PH) ;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)71);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)221);
                Debug.Assert(pack.control_request == (byte)(byte)92);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)221;
            p6.control_request = (byte)(byte)92;
            p6.ack = (byte)(byte)71;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 6);
                Debug.Assert(pack.key_TRY(ph).Equals("htdrrm"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("htdrrm", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.custom_mode == (uint)3298391023U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3298391023U;
            p11.target_system = (byte)(byte)131;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.param_index == (short)(short) -28187);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("snxfqkfbcxem"));
                Debug.Assert(pack.target_system == (byte)(byte)139);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)139;
            p20.param_index = (short)(short) -28187;
            p20.param_id_SET("snxfqkfbcxem", PH) ;
            p20.target_component = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)161);
                Debug.Assert(pack.target_component == (byte)(byte)247);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)247;
            p21.target_system = (byte)(byte)161;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)57287);
                Debug.Assert(pack.param_value == (float)2.8356362E38F);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.param_count == (ushort)(ushort)28426);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Q"));
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)28426;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p22.param_index = (ushort)(ushort)57287;
            p22.param_id_SET("Q", PH) ;
            p22.param_value = (float)2.8356362E38F;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qsnmD"));
                Debug.Assert(pack.target_component == (byte)(byte)81);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.param_value == (float) -1.3432054E38F);
                Debug.Assert(pack.target_system == (byte)(byte)233);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("qsnmD", PH) ;
            p23.target_component = (byte)(byte)81;
            p23.target_system = (byte)(byte)233;
            p23.param_value = (float) -1.3432054E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)32962);
                Debug.Assert(pack.satellites_visible == (byte)(byte)67);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1367450492U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3394604738U);
                Debug.Assert(pack.lat == (int)1168806894);
                Debug.Assert(pack.lon == (int)788515661);
                Debug.Assert(pack.alt == (int)1884911654);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2554220088U);
                Debug.Assert(pack.eph == (ushort)(ushort)64763);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)471372293);
                Debug.Assert(pack.epv == (ushort)(ushort)37245);
                Debug.Assert(pack.cog == (ushort)(ushort)38721);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3317225405U);
                Debug.Assert(pack.time_usec == (ulong)3077726411650289587L);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.satellites_visible = (byte)(byte)67;
            p24.v_acc_SET((uint)1367450492U, PH) ;
            p24.alt = (int)1884911654;
            p24.cog = (ushort)(ushort)38721;
            p24.alt_ellipsoid_SET((int)471372293, PH) ;
            p24.h_acc_SET((uint)3317225405U, PH) ;
            p24.hdg_acc_SET((uint)3394604738U, PH) ;
            p24.lon = (int)788515661;
            p24.epv = (ushort)(ushort)37245;
            p24.lat = (int)1168806894;
            p24.vel_acc_SET((uint)2554220088U, PH) ;
            p24.time_usec = (ulong)3077726411650289587L;
            p24.eph = (ushort)(ushort)64763;
            p24.vel = (ushort)(ushort)32962;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)147, (byte)86, (byte)227, (byte)95, (byte)11, (byte)111, (byte)46, (byte)27, (byte)97, (byte)238, (byte)235, (byte)237, (byte)244, (byte)235, (byte)55, (byte)67, (byte)195, (byte)107, (byte)95, (byte)176}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)22, (byte)109, (byte)120, (byte)17, (byte)159, (byte)104, (byte)169, (byte)47, (byte)203, (byte)96, (byte)54, (byte)144, (byte)215, (byte)143, (byte)116, (byte)131, (byte)223, (byte)247, (byte)223, (byte)10}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)143, (byte)233, (byte)171, (byte)114, (byte)73, (byte)200, (byte)28, (byte)63, (byte)78, (byte)124, (byte)74, (byte)233, (byte)160, (byte)215, (byte)56, (byte)99, (byte)228, (byte)251, (byte)89, (byte)95}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)245, (byte)128, (byte)152, (byte)202, (byte)121, (byte)225, (byte)43, (byte)164, (byte)90, (byte)164, (byte)239, (byte)163, (byte)9, (byte)24, (byte)223, (byte)231, (byte)185, (byte)157, (byte)114, (byte)94}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)71, (byte)37, (byte)120, (byte)34, (byte)215, (byte)136, (byte)162, (byte)223, (byte)212, (byte)59, (byte)232, (byte)186, (byte)116, (byte)142, (byte)13, (byte)62, (byte)71, (byte)107, (byte)126, (byte)190}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)87);
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)22, (byte)109, (byte)120, (byte)17, (byte)159, (byte)104, (byte)169, (byte)47, (byte)203, (byte)96, (byte)54, (byte)144, (byte)215, (byte)143, (byte)116, (byte)131, (byte)223, (byte)247, (byte)223, (byte)10}, 0) ;
            p25.satellites_visible = (byte)(byte)87;
            p25.satellite_elevation_SET(new byte[] {(byte)71, (byte)37, (byte)120, (byte)34, (byte)215, (byte)136, (byte)162, (byte)223, (byte)212, (byte)59, (byte)232, (byte)186, (byte)116, (byte)142, (byte)13, (byte)62, (byte)71, (byte)107, (byte)126, (byte)190}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)147, (byte)86, (byte)227, (byte)95, (byte)11, (byte)111, (byte)46, (byte)27, (byte)97, (byte)238, (byte)235, (byte)237, (byte)244, (byte)235, (byte)55, (byte)67, (byte)195, (byte)107, (byte)95, (byte)176}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)143, (byte)233, (byte)171, (byte)114, (byte)73, (byte)200, (byte)28, (byte)63, (byte)78, (byte)124, (byte)74, (byte)233, (byte)160, (byte)215, (byte)56, (byte)99, (byte)228, (byte)251, (byte)89, (byte)95}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)245, (byte)128, (byte)152, (byte)202, (byte)121, (byte)225, (byte)43, (byte)164, (byte)90, (byte)164, (byte)239, (byte)163, (byte)9, (byte)24, (byte)223, (byte)231, (byte)185, (byte)157, (byte)114, (byte)94}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -18501);
                Debug.Assert(pack.xgyro == (short)(short) -32157);
                Debug.Assert(pack.zacc == (short)(short) -8922);
                Debug.Assert(pack.xmag == (short)(short)23341);
                Debug.Assert(pack.yacc == (short)(short) -14408);
                Debug.Assert(pack.ygyro == (short)(short) -2278);
                Debug.Assert(pack.xacc == (short)(short) -20763);
                Debug.Assert(pack.time_boot_ms == (uint)3472594589U);
                Debug.Assert(pack.zmag == (short)(short)2714);
                Debug.Assert(pack.zgyro == (short)(short) -4395);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.zacc = (short)(short) -8922;
            p26.xgyro = (short)(short) -32157;
            p26.zgyro = (short)(short) -4395;
            p26.xmag = (short)(short)23341;
            p26.zmag = (short)(short)2714;
            p26.ymag = (short)(short) -18501;
            p26.time_boot_ms = (uint)3472594589U;
            p26.yacc = (short)(short) -14408;
            p26.xacc = (short)(short) -20763;
            p26.ygyro = (short)(short) -2278;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)16307);
                Debug.Assert(pack.xacc == (short)(short) -21460);
                Debug.Assert(pack.ygyro == (short)(short)10558);
                Debug.Assert(pack.zmag == (short)(short)8921);
                Debug.Assert(pack.ymag == (short)(short)26366);
                Debug.Assert(pack.xmag == (short)(short) -396);
                Debug.Assert(pack.yacc == (short)(short) -2595);
                Debug.Assert(pack.zacc == (short)(short) -2416);
                Debug.Assert(pack.time_usec == (ulong)1879950979783716794L);
                Debug.Assert(pack.xgyro == (short)(short)17542);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short)16307;
            p27.ygyro = (short)(short)10558;
            p27.xacc = (short)(short) -21460;
            p27.zacc = (short)(short) -2416;
            p27.yacc = (short)(short) -2595;
            p27.ymag = (short)(short)26366;
            p27.xgyro = (short)(short)17542;
            p27.zmag = (short)(short)8921;
            p27.xmag = (short)(short) -396;
            p27.time_usec = (ulong)1879950979783716794L;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)127);
                Debug.Assert(pack.press_diff1 == (short)(short)16647);
                Debug.Assert(pack.time_usec == (ulong)3756002085077947901L);
                Debug.Assert(pack.press_diff2 == (short)(short)30393);
                Debug.Assert(pack.press_abs == (short)(short)27456);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)30393;
            p28.temperature = (short)(short)127;
            p28.time_usec = (ulong)3756002085077947901L;
            p28.press_abs = (short)(short)27456;
            p28.press_diff1 = (short)(short)16647;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -3.2774422E36F);
                Debug.Assert(pack.temperature == (short)(short)11008);
                Debug.Assert(pack.time_boot_ms == (uint)2089602271U);
                Debug.Assert(pack.press_diff == (float)6.5183076E37F);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)6.5183076E37F;
            p29.temperature = (short)(short)11008;
            p29.time_boot_ms = (uint)2089602271U;
            p29.press_abs = (float) -3.2774422E36F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.3619908E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2658639595U);
                Debug.Assert(pack.pitchspeed == (float)4.4647496E37F);
                Debug.Assert(pack.rollspeed == (float)2.5862285E38F);
                Debug.Assert(pack.yaw == (float) -8.314074E36F);
                Debug.Assert(pack.yawspeed == (float) -1.9565013E38F);
                Debug.Assert(pack.roll == (float)1.9766733E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float) -8.314074E36F;
            p30.pitch = (float) -1.3619908E38F;
            p30.pitchspeed = (float)4.4647496E37F;
            p30.rollspeed = (float)2.5862285E38F;
            p30.roll = (float)1.9766733E38F;
            p30.yawspeed = (float) -1.9565013E38F;
            p30.time_boot_ms = (uint)2658639595U;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float)2.4036243E38F);
                Debug.Assert(pack.q4 == (float) -3.273095E38F);
                Debug.Assert(pack.time_boot_ms == (uint)136264498U);
                Debug.Assert(pack.q2 == (float) -3.234551E38F);
                Debug.Assert(pack.q1 == (float) -8.507455E37F);
                Debug.Assert(pack.yawspeed == (float) -1.6730872E38F);
                Debug.Assert(pack.rollspeed == (float) -1.8134379E38F);
                Debug.Assert(pack.pitchspeed == (float)1.8495558E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q2 = (float) -3.234551E38F;
            p31.yawspeed = (float) -1.6730872E38F;
            p31.rollspeed = (float) -1.8134379E38F;
            p31.q1 = (float) -8.507455E37F;
            p31.pitchspeed = (float)1.8495558E38F;
            p31.q4 = (float) -3.273095E38F;
            p31.q3 = (float)2.4036243E38F;
            p31.time_boot_ms = (uint)136264498U;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1221327584U);
                Debug.Assert(pack.x == (float) -1.6698176E38F);
                Debug.Assert(pack.z == (float)9.413044E37F);
                Debug.Assert(pack.vx == (float)3.1605821E38F);
                Debug.Assert(pack.vy == (float)1.5770492E38F);
                Debug.Assert(pack.vz == (float)8.482118E37F);
                Debug.Assert(pack.y == (float) -2.066562E38F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float)8.482118E37F;
            p32.time_boot_ms = (uint)1221327584U;
            p32.x = (float) -1.6698176E38F;
            p32.y = (float) -2.066562E38F;
            p32.vy = (float)1.5770492E38F;
            p32.vx = (float)3.1605821E38F;
            p32.z = (float)9.413044E37F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -25106);
                Debug.Assert(pack.alt == (int)1150750119);
                Debug.Assert(pack.lon == (int)513976836);
                Debug.Assert(pack.vx == (short)(short)16793);
                Debug.Assert(pack.hdg == (ushort)(ushort)44832);
                Debug.Assert(pack.relative_alt == (int) -1338737874);
                Debug.Assert(pack.lat == (int) -1829822166);
                Debug.Assert(pack.time_boot_ms == (uint)1821574039U);
                Debug.Assert(pack.vz == (short)(short)4389);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)44832;
            p33.vz = (short)(short)4389;
            p33.vy = (short)(short) -25106;
            p33.vx = (short)(short)16793;
            p33.lon = (int)513976836;
            p33.relative_alt = (int) -1338737874;
            p33.lat = (int) -1829822166;
            p33.alt = (int)1150750119;
            p33.time_boot_ms = (uint)1821574039U;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_scaled == (short)(short) -1273);
                Debug.Assert(pack.port == (byte)(byte)52);
                Debug.Assert(pack.chan6_scaled == (short)(short)17208);
                Debug.Assert(pack.chan3_scaled == (short)(short)10811);
                Debug.Assert(pack.chan8_scaled == (short)(short)29885);
                Debug.Assert(pack.rssi == (byte)(byte)77);
                Debug.Assert(pack.chan7_scaled == (short)(short)27144);
                Debug.Assert(pack.chan5_scaled == (short)(short)6541);
                Debug.Assert(pack.time_boot_ms == (uint)2572978808U);
                Debug.Assert(pack.chan4_scaled == (short)(short) -12261);
                Debug.Assert(pack.chan1_scaled == (short)(short) -12202);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan1_scaled = (short)(short) -12202;
            p34.chan2_scaled = (short)(short) -1273;
            p34.chan5_scaled = (short)(short)6541;
            p34.chan7_scaled = (short)(short)27144;
            p34.time_boot_ms = (uint)2572978808U;
            p34.chan3_scaled = (short)(short)10811;
            p34.chan4_scaled = (short)(short) -12261;
            p34.port = (byte)(byte)52;
            p34.chan6_scaled = (short)(short)17208;
            p34.chan8_scaled = (short)(short)29885;
            p34.rssi = (byte)(byte)77;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)63201);
                Debug.Assert(pack.time_boot_ms == (uint)2724379751U);
                Debug.Assert(pack.rssi == (byte)(byte)18);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)29117);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43075);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)24967);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)4399);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)14053);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)25780);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)38048);
                Debug.Assert(pack.port == (byte)(byte)167);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)43075;
            p35.chan7_raw = (ushort)(ushort)24967;
            p35.chan3_raw = (ushort)(ushort)14053;
            p35.port = (byte)(byte)167;
            p35.chan8_raw = (ushort)(ushort)63201;
            p35.chan4_raw = (ushort)(ushort)25780;
            p35.chan5_raw = (ushort)(ushort)38048;
            p35.chan1_raw = (ushort)(ushort)29117;
            p35.rssi = (byte)(byte)18;
            p35.time_boot_ms = (uint)2724379751U;
            p35.chan6_raw = (ushort)(ushort)4399;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)578);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)3183);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)28191);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)21834);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)53707);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)29845);
                Debug.Assert(pack.port == (byte)(byte)172);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)13328);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)21648);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)57855);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)24895);
                Debug.Assert(pack.time_usec == (uint)2501414155U);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)23350);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)52282);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)37487);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)2728);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)46013);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)5384);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo10_raw_SET((ushort)(ushort)3183, PH) ;
            p36.servo3_raw = (ushort)(ushort)578;
            p36.servo6_raw = (ushort)(ushort)23350;
            p36.servo13_raw_SET((ushort)(ushort)21648, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)28191, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)21834, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)5384, PH) ;
            p36.time_usec = (uint)2501414155U;
            p36.servo8_raw = (ushort)(ushort)29845;
            p36.servo14_raw_SET((ushort)(ushort)37487, PH) ;
            p36.servo2_raw = (ushort)(ushort)2728;
            p36.servo1_raw = (ushort)(ushort)57855;
            p36.servo7_raw = (ushort)(ushort)53707;
            p36.servo16_raw_SET((ushort)(ushort)52282, PH) ;
            p36.port = (byte)(byte)172;
            p36.servo15_raw_SET((ushort)(ushort)13328, PH) ;
            p36.servo5_raw = (ushort)(ushort)46013;
            p36.servo4_raw = (ushort)(ushort)24895;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)19191);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.start_index == (short)(short) -14548);
                Debug.Assert(pack.target_component == (byte)(byte)143);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short) -14548;
            p37.target_component = (byte)(byte)143;
            p37.target_system = (byte)(byte)224;
            p37.end_index = (short)(short)19191;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.end_index == (short)(short)15392);
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.start_index == (short)(short)22030);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short)22030;
            p38.target_component = (byte)(byte)187;
            p38.target_system = (byte)(byte)90;
            p38.end_index = (short)(short)15392;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)1.3698929E38F);
                Debug.Assert(pack.target_system == (byte)(byte)252);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.y == (float) -3.1344316E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)253);
                Debug.Assert(pack.z == (float) -1.8164881E38F);
                Debug.Assert(pack.x == (float) -3.1536265E38F);
                Debug.Assert(pack.current == (byte)(byte)69);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param4 == (float)4.5688546E36F);
                Debug.Assert(pack.seq == (ushort)(ushort)56522);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_1);
                Debug.Assert(pack.param1 == (float) -1.0299853E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.param3 == (float)2.2835258E37F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.seq = (ushort)(ushort)56522;
            p39.param4 = (float)4.5688546E36F;
            p39.x = (float) -3.1536265E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p39.current = (byte)(byte)69;
            p39.z = (float) -1.8164881E38F;
            p39.autocontinue = (byte)(byte)253;
            p39.target_system = (byte)(byte)252;
            p39.target_component = (byte)(byte)143;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.y = (float) -3.1344316E38F;
            p39.param2 = (float)1.3698929E38F;
            p39.param1 = (float) -1.0299853E38F;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_1;
            p39.param3 = (float)2.2835258E37F;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)217);
                Debug.Assert(pack.seq == (ushort)(ushort)55870);
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)217;
            p40.seq = (ushort)(ushort)55870;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p40.target_component = (byte)(byte)133;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)55);
                Debug.Assert(pack.seq == (ushort)(ushort)3906);
                Debug.Assert(pack.target_system == (byte)(byte)238);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)55;
            p41.seq = (ushort)(ushort)3906;
            p41.target_system = (byte)(byte)238;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)39281);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)39281;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.target_system == (byte)(byte)233);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p43.target_system = (byte)(byte)233;
            p43.target_component = (byte)(byte)27;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.count == (ushort)(ushort)60510);
                Debug.Assert(pack.target_component == (byte)(byte)44);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)60510;
            p44.target_system = (byte)(byte)198;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_component = (byte)(byte)44;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)143;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)167;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)54897);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)54897;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)182;
            p47.target_component = (byte)(byte)134;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -1535762676);
                Debug.Assert(pack.longitude == (int)1214689046);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.altitude == (int) -1519322153);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7731560082157329960L);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int)1214689046;
            p48.target_system = (byte)(byte)179;
            p48.latitude = (int) -1535762676;
            p48.time_usec_SET((ulong)7731560082157329960L, PH) ;
            p48.altitude = (int) -1519322153;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)1070974416);
                Debug.Assert(pack.altitude == (int) -2084049941);
                Debug.Assert(pack.longitude == (int) -737573819);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3434635533932627801L);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)1070974416;
            p49.time_usec_SET((ulong)3434635533932627801L, PH) ;
            p49.altitude = (int) -2084049941;
            p49.longitude = (int) -737573819;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float) -2.2481873E38F);
                Debug.Assert(pack.param_index == (short)(short) -14138);
                Debug.Assert(pack.scale == (float) -8.78781E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("n"));
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.param_value_min == (float) -1.841605E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)66);
                Debug.Assert(pack.param_value_max == (float) -1.6862801E38F);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)66;
            p50.param_index = (short)(short) -14138;
            p50.scale = (float) -8.78781E37F;
            p50.parameter_rc_channel_index = (byte)(byte)41;
            p50.target_system = (byte)(byte)128;
            p50.param_value0 = (float) -2.2481873E38F;
            p50.param_id_SET("n", PH) ;
            p50.param_value_max = (float) -1.6862801E38F;
            p50.param_value_min = (float) -1.841605E38F;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)42384);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.target_system == (byte)(byte)71);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_system = (byte)(byte)71;
            p51.target_component = (byte)(byte)205;
            p51.seq = (ushort)(ushort)42384;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float)3.0008509E38F);
                Debug.Assert(pack.p1y == (float) -2.3147939E38F);
                Debug.Assert(pack.p2y == (float) -1.3474786E38F);
                Debug.Assert(pack.p1z == (float) -3.5987477E37F);
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p2z == (float) -6.56179E36F);
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.p2x == (float) -2.3029388E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float) -6.56179E36F;
            p54.p1x = (float)3.0008509E38F;
            p54.target_system = (byte)(byte)156;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p54.p1z = (float) -3.5987477E37F;
            p54.p2y = (float) -1.3474786E38F;
            p54.p1y = (float) -2.3147939E38F;
            p54.p2x = (float) -2.3029388E38F;
            p54.target_component = (byte)(byte)80;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p1x == (float) -2.1474705E38F);
                Debug.Assert(pack.p1y == (float) -2.5810973E38F);
                Debug.Assert(pack.p2z == (float)3.0911033E38F);
                Debug.Assert(pack.p2x == (float)5.096477E36F);
                Debug.Assert(pack.p1z == (float) -1.7261182E37F);
                Debug.Assert(pack.p2y == (float)2.2443983E38F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float)2.2443983E38F;
            p55.p2x = (float)5.096477E36F;
            p55.p2z = (float)3.0911033E38F;
            p55.p1z = (float) -1.7261182E37F;
            p55.p1x = (float) -2.1474705E38F;
            p55.p1y = (float) -2.5810973E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.1052109E38F, -2.9333603E38F, 9.654092E37F, 3.1973894E38F, 8.94461E37F, 2.0676367E38F, 4.6357025E36F, -2.4514983E38F, 6.959123E37F}));
                Debug.Assert(pack.time_usec == (ulong)3984087520805012752L);
                Debug.Assert(pack.pitchspeed == (float)2.8362808E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.288719E38F, -2.8435283E38F, 7.021605E37F, -9.113408E37F}));
                Debug.Assert(pack.rollspeed == (float)5.5182737E37F);
                Debug.Assert(pack.yawspeed == (float) -2.2719254E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {-2.1052109E38F, -2.9333603E38F, 9.654092E37F, 3.1973894E38F, 8.94461E37F, 2.0676367E38F, 4.6357025E36F, -2.4514983E38F, 6.959123E37F}, 0) ;
            p61.q_SET(new float[] {-1.288719E38F, -2.8435283E38F, 7.021605E37F, -9.113408E37F}, 0) ;
            p61.yawspeed = (float) -2.2719254E38F;
            p61.time_usec = (ulong)3984087520805012752L;
            p61.pitchspeed = (float)2.8362808E38F;
            p61.rollspeed = (float)5.5182737E37F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_dist == (ushort)(ushort)32067);
                Debug.Assert(pack.aspd_error == (float) -1.3534703E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -2188);
                Debug.Assert(pack.nav_bearing == (short)(short)30388);
                Debug.Assert(pack.xtrack_error == (float) -6.230287E37F);
                Debug.Assert(pack.alt_error == (float) -2.9133136E38F);
                Debug.Assert(pack.nav_roll == (float) -1.2398834E38F);
                Debug.Assert(pack.nav_pitch == (float) -2.1343092E38F);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float) -2.1343092E38F;
            p62.xtrack_error = (float) -6.230287E37F;
            p62.alt_error = (float) -2.9133136E38F;
            p62.aspd_error = (float) -1.3534703E38F;
            p62.nav_roll = (float) -1.2398834E38F;
            p62.target_bearing = (short)(short) -2188;
            p62.nav_bearing = (short)(short)30388;
            p62.wp_dist = (ushort)(ushort)32067;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.time_usec == (ulong)99822043470273480L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.6887729E38F, 2.320601E38F, -2.871498E38F, 3.2365091E38F, 2.9201175E38F, -1.2432192E38F, -1.9185131E38F, 2.1303958E38F, -1.1944444E38F, 2.7049997E38F, 8.738395E37F, -1.993904E38F, 1.3052198E38F, -6.276961E37F, 2.6205804E38F, 2.7974356E37F, -2.5191065E38F, 3.0024501E38F, 3.389745E38F, -2.4327671E38F, -1.4396066E38F, 1.9913544E38F, 1.2806711E38F, -1.7880092E38F, -8.642925E37F, -1.7138346E37F, 4.647108E37F, 1.7534054E38F, -2.8941323E38F, -2.4942048E38F, 1.6874476E38F, 2.0471967E38F, 1.6015254E38F, 1.3442932E38F, 2.9008886E38F, -7.0849945E37F}));
                Debug.Assert(pack.relative_alt == (int)353510648);
                Debug.Assert(pack.vy == (float)1.920268E38F);
                Debug.Assert(pack.vz == (float) -1.3541851E38F);
                Debug.Assert(pack.lon == (int) -123502844);
                Debug.Assert(pack.vx == (float)5.2346744E37F);
                Debug.Assert(pack.lat == (int)1769032701);
                Debug.Assert(pack.alt == (int) -1815973629);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vy = (float)1.920268E38F;
            p63.alt = (int) -1815973629;
            p63.covariance_SET(new float[] {-1.6887729E38F, 2.320601E38F, -2.871498E38F, 3.2365091E38F, 2.9201175E38F, -1.2432192E38F, -1.9185131E38F, 2.1303958E38F, -1.1944444E38F, 2.7049997E38F, 8.738395E37F, -1.993904E38F, 1.3052198E38F, -6.276961E37F, 2.6205804E38F, 2.7974356E37F, -2.5191065E38F, 3.0024501E38F, 3.389745E38F, -2.4327671E38F, -1.4396066E38F, 1.9913544E38F, 1.2806711E38F, -1.7880092E38F, -8.642925E37F, -1.7138346E37F, 4.647108E37F, 1.7534054E38F, -2.8941323E38F, -2.4942048E38F, 1.6874476E38F, 2.0471967E38F, 1.6015254E38F, 1.3442932E38F, 2.9008886E38F, -7.0849945E37F}, 0) ;
            p63.time_usec = (ulong)99822043470273480L;
            p63.vz = (float) -1.3541851E38F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.vx = (float)5.2346744E37F;
            p63.lon = (int) -123502844;
            p63.relative_alt = (int)353510648;
            p63.lat = (int)1769032701;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.z == (float) -3.1038613E37F);
                Debug.Assert(pack.ay == (float)2.5848361E38F);
                Debug.Assert(pack.ax == (float) -1.043092E38F);
                Debug.Assert(pack.y == (float) -1.0374496E38F);
                Debug.Assert(pack.vy == (float)2.3113268E38F);
                Debug.Assert(pack.time_usec == (ulong)701751180409669923L);
                Debug.Assert(pack.vz == (float) -2.7053098E38F);
                Debug.Assert(pack.az == (float)2.327502E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.0636634E38F, 2.519447E38F, 1.6221342E38F, 2.2737818E38F, -3.9277697E37F, -1.1964619E37F, 3.8722885E37F, -1.8524606E38F, -1.0043736E38F, 2.4646953E38F, -2.5895435E38F, -2.6904446E38F, 1.559883E38F, 2.836599E38F, -2.4073078E38F, 1.3227878E38F, 1.5569717E38F, -1.7564621E38F, -2.851708E38F, 2.3203843E38F, 2.8882788E38F, -1.879412E38F, 2.4735265E38F, -3.3594179E38F, 6.6144685E37F, 1.7454312E38F, 1.8241464E38F, 2.8265528E37F, -2.4168645E38F, 2.8266732E36F, 2.118577E38F, 1.9475798E37F, 2.829736E38F, -2.0680237E38F, -1.8105533E38F, 3.3138462E38F, 1.5872118E38F, 6.6762507E37F, 1.4171729E37F, 3.342148E38F, 2.4801367E38F, 7.428017E36F, 2.2418512E37F, -4.449539E37F, 3.038238E37F}));
                Debug.Assert(pack.vx == (float)2.4672878E38F);
                Debug.Assert(pack.x == (float) -3.2143242E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.x = (float) -3.2143242E38F;
            p64.z = (float) -3.1038613E37F;
            p64.ax = (float) -1.043092E38F;
            p64.ay = (float)2.5848361E38F;
            p64.covariance_SET(new float[] {1.0636634E38F, 2.519447E38F, 1.6221342E38F, 2.2737818E38F, -3.9277697E37F, -1.1964619E37F, 3.8722885E37F, -1.8524606E38F, -1.0043736E38F, 2.4646953E38F, -2.5895435E38F, -2.6904446E38F, 1.559883E38F, 2.836599E38F, -2.4073078E38F, 1.3227878E38F, 1.5569717E38F, -1.7564621E38F, -2.851708E38F, 2.3203843E38F, 2.8882788E38F, -1.879412E38F, 2.4735265E38F, -3.3594179E38F, 6.6144685E37F, 1.7454312E38F, 1.8241464E38F, 2.8265528E37F, -2.4168645E38F, 2.8266732E36F, 2.118577E38F, 1.9475798E37F, 2.829736E38F, -2.0680237E38F, -1.8105533E38F, 3.3138462E38F, 1.5872118E38F, 6.6762507E37F, 1.4171729E37F, 3.342148E38F, 2.4801367E38F, 7.428017E36F, 2.2418512E37F, -4.449539E37F, 3.038238E37F}, 0) ;
            p64.vz = (float) -2.7053098E38F;
            p64.vx = (float)2.4672878E38F;
            p64.time_usec = (ulong)701751180409669923L;
            p64.az = (float)2.327502E37F;
            p64.y = (float) -1.0374496E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.vy = (float)2.3113268E38F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)32200);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)12150);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)43580);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)36077);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)56854);
                Debug.Assert(pack.rssi == (byte)(byte)68);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)61588);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)22352);
                Debug.Assert(pack.time_boot_ms == (uint)2877157519U);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)30794);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)34839);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)27242);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)40224);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)44024);
                Debug.Assert(pack.chancount == (byte)(byte)61);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51997);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)46663);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)7444);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)59866);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)23413);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)4490);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan15_raw = (ushort)(ushort)46663;
            p65.chan9_raw = (ushort)(ushort)23413;
            p65.rssi = (byte)(byte)68;
            p65.chan2_raw = (ushort)(ushort)51997;
            p65.chan12_raw = (ushort)(ushort)7444;
            p65.chan8_raw = (ushort)(ushort)32200;
            p65.chan17_raw = (ushort)(ushort)56854;
            p65.chan16_raw = (ushort)(ushort)34839;
            p65.chan1_raw = (ushort)(ushort)43580;
            p65.chan6_raw = (ushort)(ushort)22352;
            p65.chan7_raw = (ushort)(ushort)61588;
            p65.chan13_raw = (ushort)(ushort)27242;
            p65.chan5_raw = (ushort)(ushort)40224;
            p65.chan11_raw = (ushort)(ushort)30794;
            p65.time_boot_ms = (uint)2877157519U;
            p65.chan18_raw = (ushort)(ushort)4490;
            p65.chan3_raw = (ushort)(ushort)59866;
            p65.chan10_raw = (ushort)(ushort)12150;
            p65.chan4_raw = (ushort)(ushort)36077;
            p65.chan14_raw = (ushort)(ushort)44024;
            p65.chancount = (byte)(byte)61;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)45);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)11661);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.start_stop == (byte)(byte)150);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)11661;
            p66.req_stream_id = (byte)(byte)45;
            p66.target_system = (byte)(byte)32;
            p66.target_component = (byte)(byte)117;
            p66.start_stop = (byte)(byte)150;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)215);
                Debug.Assert(pack.stream_id == (byte)(byte)120);
                Debug.Assert(pack.message_rate == (ushort)(ushort)7373);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)7373;
            p67.on_off = (byte)(byte)215;
            p67.stream_id = (byte)(byte)120;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short) -26489);
                Debug.Assert(pack.y == (short)(short)20373);
                Debug.Assert(pack.target == (byte)(byte)63);
                Debug.Assert(pack.r == (short)(short)30279);
                Debug.Assert(pack.buttons == (ushort)(ushort)14531);
                Debug.Assert(pack.x == (short)(short)7596);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)14531;
            p69.y = (short)(short)20373;
            p69.z = (short)(short) -26489;
            p69.target = (byte)(byte)63;
            p69.r = (short)(short)30279;
            p69.x = (short)(short)7596;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)5291);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)12952);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)43829);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)18481);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)57530);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)5848);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)13393);
                Debug.Assert(pack.target_component == (byte)(byte)140);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)15178);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan3_raw = (ushort)(ushort)13393;
            p70.chan2_raw = (ushort)(ushort)15178;
            p70.chan6_raw = (ushort)(ushort)43829;
            p70.chan7_raw = (ushort)(ushort)5291;
            p70.chan1_raw = (ushort)(ushort)57530;
            p70.target_system = (byte)(byte)179;
            p70.chan5_raw = (ushort)(ushort)18481;
            p70.chan4_raw = (ushort)(ushort)12952;
            p70.target_component = (byte)(byte)140;
            p70.chan8_raw = (ushort)(ushort)5848;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.z == (float) -1.2685048E38F);
                Debug.Assert(pack.x == (int) -319514737);
                Debug.Assert(pack.param1 == (float)2.2126054E38F);
                Debug.Assert(pack.param3 == (float)3.1248861E38F);
                Debug.Assert(pack.target_component == (byte)(byte)184);
                Debug.Assert(pack.current == (byte)(byte)96);
                Debug.Assert(pack.param2 == (float) -3.1260538E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_4);
                Debug.Assert(pack.param4 == (float) -1.3507045E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)47661);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.y == (int)1126199098);
                Debug.Assert(pack.autocontinue == (byte)(byte)169);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_component = (byte)(byte)184;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p73.y = (int)1126199098;
            p73.x = (int) -319514737;
            p73.param4 = (float) -1.3507045E38F;
            p73.param3 = (float)3.1248861E38F;
            p73.autocontinue = (byte)(byte)169;
            p73.target_system = (byte)(byte)254;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.z = (float) -1.2685048E38F;
            p73.param2 = (float) -3.1260538E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_4;
            p73.current = (byte)(byte)96;
            p73.seq = (ushort)(ushort)47661;
            p73.param1 = (float)2.2126054E38F;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float) -2.15412E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)1228);
                Debug.Assert(pack.heading == (short)(short) -17196);
                Debug.Assert(pack.climb == (float) -1.1338633E37F);
                Debug.Assert(pack.alt == (float)8.729688E37F);
                Debug.Assert(pack.airspeed == (float)1.2536951E38F);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float) -1.1338633E37F;
            p74.groundspeed = (float) -2.15412E38F;
            p74.throttle = (ushort)(ushort)1228;
            p74.heading = (short)(short) -17196;
            p74.airspeed = (float)1.2536951E38F;
            p74.alt = (float)8.729688E37F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)1600059908);
                Debug.Assert(pack.param4 == (float) -4.1867432E37F);
                Debug.Assert(pack.target_system == (byte)(byte)84);
                Debug.Assert(pack.z == (float)1.2189155E38F);
                Debug.Assert(pack.current == (byte)(byte)134);
                Debug.Assert(pack.y == (int) -1640477778);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION);
                Debug.Assert(pack.param2 == (float) -3.5375309E37F);
                Debug.Assert(pack.param3 == (float)1.6147321E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)139);
                Debug.Assert(pack.param1 == (float) -1.5267496E38F);
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.x = (int)1600059908;
            p75.param4 = (float) -4.1867432E37F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p75.param3 = (float)1.6147321E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
            p75.current = (byte)(byte)134;
            p75.y = (int) -1640477778;
            p75.target_system = (byte)(byte)84;
            p75.target_component = (byte)(byte)247;
            p75.param2 = (float) -3.5375309E37F;
            p75.autocontinue = (byte)(byte)139;
            p75.z = (float)1.2189155E38F;
            p75.param1 = (float) -1.5267496E38F;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float) -1.9139938E38F);
                Debug.Assert(pack.param4 == (float)2.8453162E38F);
                Debug.Assert(pack.param3 == (float)1.9247341E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)127);
                Debug.Assert(pack.param2 == (float) -2.3150697E38F);
                Debug.Assert(pack.param6 == (float)7.3257797E37F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FOLLOW);
                Debug.Assert(pack.param5 == (float) -2.0611901E38F);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.param1 == (float) -1.5864684E38F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param5 = (float) -2.0611901E38F;
            p76.target_system = (byte)(byte)242;
            p76.param2 = (float) -2.3150697E38F;
            p76.param4 = (float)2.8453162E38F;
            p76.confirmation = (byte)(byte)127;
            p76.target_component = (byte)(byte)179;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FOLLOW;
            p76.param6 = (float)7.3257797E37F;
            p76.param7 = (float) -1.9139938E38F;
            p76.param1 = (float) -1.5864684E38F;
            p76.param3 = (float)1.9247341E38F;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)234);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)186);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)1);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)621457279);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
            p77.progress_SET((byte)(byte)186, PH) ;
            p77.target_system_SET((byte)(byte)1, PH) ;
            p77.result_param2_SET((int)621457279, PH) ;
            p77.target_component_SET((byte)(byte)234, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.7917009E38F);
                Debug.Assert(pack.pitch == (float)1.3684302E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)145);
                Debug.Assert(pack.roll == (float)1.3724247E38F);
                Debug.Assert(pack.thrust == (float)1.3032578E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)254);
                Debug.Assert(pack.time_boot_ms == (uint)403351564U);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)254;
            p81.pitch = (float)1.3684302E38F;
            p81.thrust = (float)1.3032578E38F;
            p81.yaw = (float)2.7917009E38F;
            p81.roll = (float)1.3724247E38F;
            p81.time_boot_ms = (uint)403351564U;
            p81.manual_override_switch = (byte)(byte)145;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.0050032E38F, -2.38503E38F, 1.8103172E38F, 2.452403E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)37);
                Debug.Assert(pack.target_component == (byte)(byte)13);
                Debug.Assert(pack.body_pitch_rate == (float) -1.7620893E38F);
                Debug.Assert(pack.body_yaw_rate == (float)2.7051895E38F);
                Debug.Assert(pack.thrust == (float) -1.8814117E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2693120870U);
                Debug.Assert(pack.type_mask == (byte)(byte)215);
                Debug.Assert(pack.body_roll_rate == (float) -1.2662076E38F);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float)2.7051895E38F;
            p82.thrust = (float) -1.8814117E38F;
            p82.body_pitch_rate = (float) -1.7620893E38F;
            p82.time_boot_ms = (uint)2693120870U;
            p82.q_SET(new float[] {-1.0050032E38F, -2.38503E38F, 1.8103172E38F, 2.452403E38F}, 0) ;
            p82.target_system = (byte)(byte)37;
            p82.body_roll_rate = (float) -1.2662076E38F;
            p82.target_component = (byte)(byte)13;
            p82.type_mask = (byte)(byte)215;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2086307048U);
                Debug.Assert(pack.body_yaw_rate == (float)8.73327E37F);
                Debug.Assert(pack.body_roll_rate == (float)9.175737E36F);
                Debug.Assert(pack.body_pitch_rate == (float) -1.3016939E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.5205705E38F, 1.6593328E37F, -1.5147385E37F, 7.6663487E37F}));
                Debug.Assert(pack.thrust == (float) -1.4390794E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)95);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float) -1.3016939E38F;
            p83.body_yaw_rate = (float)8.73327E37F;
            p83.type_mask = (byte)(byte)95;
            p83.q_SET(new float[] {2.5205705E38F, 1.6593328E37F, -1.5147385E37F, 7.6663487E37F}, 0) ;
            p83.thrust = (float) -1.4390794E38F;
            p83.body_roll_rate = (float)9.175737E36F;
            p83.time_boot_ms = (uint)2086307048U;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3387278483U);
                Debug.Assert(pack.yaw_rate == (float)1.0536887E38F);
                Debug.Assert(pack.vz == (float) -2.192836E38F);
                Debug.Assert(pack.afy == (float) -1.3050087E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.z == (float)1.5133512E38F);
                Debug.Assert(pack.afx == (float) -4.6746837E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)63757);
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.target_component == (byte)(byte)235);
                Debug.Assert(pack.afz == (float)3.1487901E38F);
                Debug.Assert(pack.vx == (float) -2.9890094E38F);
                Debug.Assert(pack.vy == (float) -2.1662362E38F);
                Debug.Assert(pack.yaw == (float) -2.175913E38F);
                Debug.Assert(pack.y == (float) -3.115691E38F);
                Debug.Assert(pack.x == (float)2.5023378E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -3.115691E38F;
            p84.vy = (float) -2.1662362E38F;
            p84.afy = (float) -1.3050087E38F;
            p84.afz = (float)3.1487901E38F;
            p84.target_component = (byte)(byte)235;
            p84.type_mask = (ushort)(ushort)63757;
            p84.z = (float)1.5133512E38F;
            p84.x = (float)2.5023378E38F;
            p84.time_boot_ms = (uint)3387278483U;
            p84.yaw_rate = (float)1.0536887E38F;
            p84.afx = (float) -4.6746837E37F;
            p84.vz = (float) -2.192836E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p84.vx = (float) -2.9890094E38F;
            p84.target_system = (byte)(byte)108;
            p84.yaw = (float) -2.175913E38F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.1719778E38F);
                Debug.Assert(pack.yaw == (float)3.2350577E38F);
                Debug.Assert(pack.target_component == (byte)(byte)192);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.vz == (float) -1.2376124E38F);
                Debug.Assert(pack.lat_int == (int)1748436926);
                Debug.Assert(pack.afy == (float) -2.642489E38F);
                Debug.Assert(pack.lon_int == (int)251928585);
                Debug.Assert(pack.type_mask == (ushort)(ushort)52067);
                Debug.Assert(pack.vx == (float)1.3914135E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4154379817U);
                Debug.Assert(pack.afz == (float) -8.641791E37F);
                Debug.Assert(pack.vy == (float) -3.3573063E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.745938E38F);
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.afx == (float)6.2318834E37F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.target_component = (byte)(byte)192;
            p86.afy = (float) -2.642489E38F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p86.afx = (float)6.2318834E37F;
            p86.vx = (float)1.3914135E38F;
            p86.lon_int = (int)251928585;
            p86.vy = (float) -3.3573063E38F;
            p86.yaw = (float)3.2350577E38F;
            p86.target_system = (byte)(byte)101;
            p86.alt = (float)1.1719778E38F;
            p86.type_mask = (ushort)(ushort)52067;
            p86.vz = (float) -1.2376124E38F;
            p86.afz = (float) -8.641791E37F;
            p86.lat_int = (int)1748436926;
            p86.yaw_rate = (float) -1.745938E38F;
            p86.time_boot_ms = (uint)4154379817U;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.7660561E36F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.alt == (float)1.6136501E38F);
                Debug.Assert(pack.lat_int == (int)19090004);
                Debug.Assert(pack.type_mask == (ushort)(ushort)30277);
                Debug.Assert(pack.vz == (float) -4.781405E37F);
                Debug.Assert(pack.yaw_rate == (float)4.084736E37F);
                Debug.Assert(pack.vy == (float) -1.9293028E38F);
                Debug.Assert(pack.afx == (float)4.655296E37F);
                Debug.Assert(pack.yaw == (float) -1.0133158E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2787319434U);
                Debug.Assert(pack.afz == (float)2.4317139E38F);
                Debug.Assert(pack.afy == (float)3.6926687E36F);
                Debug.Assert(pack.lon_int == (int) -1920380583);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vy = (float) -1.9293028E38F;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.vx = (float) -1.7660561E36F;
            p87.lat_int = (int)19090004;
            p87.afz = (float)2.4317139E38F;
            p87.time_boot_ms = (uint)2787319434U;
            p87.vz = (float) -4.781405E37F;
            p87.afx = (float)4.655296E37F;
            p87.afy = (float)3.6926687E36F;
            p87.lon_int = (int) -1920380583;
            p87.alt = (float)1.6136501E38F;
            p87.yaw_rate = (float)4.084736E37F;
            p87.type_mask = (ushort)(ushort)30277;
            p87.yaw = (float) -1.0133158E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.181074E38F);
                Debug.Assert(pack.pitch == (float) -1.6859203E38F);
                Debug.Assert(pack.roll == (float) -1.8331045E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1634911372U);
                Debug.Assert(pack.y == (float) -3.1759955E38F);
                Debug.Assert(pack.yaw == (float) -3.005322E38F);
                Debug.Assert(pack.z == (float)3.3451113E36F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float) -3.1759955E38F;
            p89.yaw = (float) -3.005322E38F;
            p89.pitch = (float) -1.6859203E38F;
            p89.z = (float)3.3451113E36F;
            p89.x = (float) -2.181074E38F;
            p89.roll = (float) -1.8331045E38F;
            p89.time_boot_ms = (uint)1634911372U;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.2543498E37F);
                Debug.Assert(pack.vy == (short)(short)4439);
                Debug.Assert(pack.pitchspeed == (float) -8.0886985E37F);
                Debug.Assert(pack.vx == (short)(short) -6116);
                Debug.Assert(pack.time_usec == (ulong)6979389190419002912L);
                Debug.Assert(pack.lat == (int) -1050772229);
                Debug.Assert(pack.alt == (int) -330097824);
                Debug.Assert(pack.zacc == (short)(short) -23700);
                Debug.Assert(pack.yaw == (float)1.060296E38F);
                Debug.Assert(pack.yacc == (short)(short)22311);
                Debug.Assert(pack.xacc == (short)(short)377);
                Debug.Assert(pack.vz == (short)(short) -21879);
                Debug.Assert(pack.pitch == (float)2.659077E38F);
                Debug.Assert(pack.rollspeed == (float) -5.8589056E37F);
                Debug.Assert(pack.lon == (int)1230284968);
                Debug.Assert(pack.yawspeed == (float)2.527165E38F);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short) -21879;
            p90.roll = (float)2.2543498E37F;
            p90.yacc = (short)(short)22311;
            p90.vy = (short)(short)4439;
            p90.pitch = (float)2.659077E38F;
            p90.zacc = (short)(short) -23700;
            p90.pitchspeed = (float) -8.0886985E37F;
            p90.yawspeed = (float)2.527165E38F;
            p90.lon = (int)1230284968;
            p90.alt = (int) -330097824;
            p90.rollspeed = (float) -5.8589056E37F;
            p90.yaw = (float)1.060296E38F;
            p90.lat = (int) -1050772229;
            p90.xacc = (short)(short)377;
            p90.vx = (short)(short) -6116;
            p90.time_usec = (ulong)6979389190419002912L;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.throttle == (float)8.2220097E37F);
                Debug.Assert(pack.time_usec == (ulong)5221094885925006425L);
                Debug.Assert(pack.pitch_elevator == (float)2.879982E38F);
                Debug.Assert(pack.roll_ailerons == (float)1.6084266E38F);
                Debug.Assert(pack.aux3 == (float)2.4473135E38F);
                Debug.Assert(pack.aux1 == (float)1.704737E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)97);
                Debug.Assert(pack.yaw_rudder == (float)1.3991688E37F);
                Debug.Assert(pack.aux4 == (float)9.835475E37F);
                Debug.Assert(pack.aux2 == (float)1.4842133E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.yaw_rudder = (float)1.3991688E37F;
            p91.time_usec = (ulong)5221094885925006425L;
            p91.roll_ailerons = (float)1.6084266E38F;
            p91.nav_mode = (byte)(byte)97;
            p91.aux2 = (float)1.4842133E38F;
            p91.aux3 = (float)2.4473135E38F;
            p91.aux1 = (float)1.704737E38F;
            p91.aux4 = (float)9.835475E37F;
            p91.pitch_elevator = (float)2.879982E38F;
            p91.throttle = (float)8.2220097E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)47187);
                Debug.Assert(pack.time_usec == (ulong)3808743268858839149L);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)29252);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)47229);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)43311);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)38076);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)53442);
                Debug.Assert(pack.rssi == (byte)(byte)215);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)60178);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)5883);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)21216);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)14877);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)42927);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)12795);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan7_raw = (ushort)(ushort)42927;
            p92.chan6_raw = (ushort)(ushort)60178;
            p92.chan10_raw = (ushort)(ushort)53442;
            p92.chan12_raw = (ushort)(ushort)38076;
            p92.chan9_raw = (ushort)(ushort)29252;
            p92.chan4_raw = (ushort)(ushort)43311;
            p92.chan2_raw = (ushort)(ushort)12795;
            p92.chan3_raw = (ushort)(ushort)5883;
            p92.rssi = (byte)(byte)215;
            p92.time_usec = (ulong)3808743268858839149L;
            p92.chan5_raw = (ushort)(ushort)47187;
            p92.chan11_raw = (ushort)(ushort)21216;
            p92.chan1_raw = (ushort)(ushort)14877;
            p92.chan8_raw = (ushort)(ushort)47229;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)8885163541570318745L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.3611395E38F, -1.3963376E38F, 2.0512307E38F, 1.5834368E38F, -2.3264104E38F, -1.0838624E38F, 3.3368973E38F, 7.963937E37F, -3.3400902E38F, -1.1117426E38F, -2.6418423E38F, -2.3842163E38F, 2.8963005E38F, -2.7414232E37F, 2.3179334E38F, 3.197688E38F}));
                Debug.Assert(pack.time_usec == (ulong)7555346073727959512L);
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)8885163541570318745L;
            p93.time_usec = (ulong)7555346073727959512L;
            p93.controls_SET(new float[] {3.3611395E38F, -1.3963376E38F, 2.0512307E38F, 1.5834368E38F, -2.3264104E38F, -1.0838624E38F, 3.3368973E38F, 7.963937E37F, -3.3400902E38F, -1.1117426E38F, -2.6418423E38F, -2.3842163E38F, 2.8963005E38F, -2.7414232E37F, 2.3179334E38F, 3.197688E38F}, 0) ;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.8956366E37F);
                Debug.Assert(pack.ground_distance == (float)1.4332284E38F);
                Debug.Assert(pack.time_usec == (ulong)2485765594839771450L);
                Debug.Assert(pack.flow_y == (short)(short)32754);
                Debug.Assert(pack.quality == (byte)(byte)106);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.9919839E37F);
                Debug.Assert(pack.flow_comp_m_x == (float)3.131585E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)99);
                Debug.Assert(pack.flow_x == (short)(short)32278);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.7613067E37F);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_y_SET((float) -1.8956366E37F, PH) ;
            p100.flow_y = (short)(short)32754;
            p100.sensor_id = (byte)(byte)99;
            p100.time_usec = (ulong)2485765594839771450L;
            p100.flow_x = (short)(short)32278;
            p100.flow_comp_m_x = (float)3.131585E38F;
            p100.ground_distance = (float)1.4332284E38F;
            p100.quality = (byte)(byte)106;
            p100.flow_comp_m_y = (float) -3.9919839E37F;
            p100.flow_rate_x_SET((float) -1.7613067E37F, PH) ;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)4956299961579325672L);
                Debug.Assert(pack.z == (float)9.927372E37F);
                Debug.Assert(pack.roll == (float)2.3952155E38F);
                Debug.Assert(pack.x == (float)9.251335E35F);
                Debug.Assert(pack.y == (float) -1.92099E38F);
                Debug.Assert(pack.yaw == (float)3.0901675E38F);
                Debug.Assert(pack.pitch == (float)5.4407995E37F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float)9.927372E37F;
            p101.y = (float) -1.92099E38F;
            p101.yaw = (float)3.0901675E38F;
            p101.pitch = (float)5.4407995E37F;
            p101.usec = (ulong)4956299961579325672L;
            p101.roll = (float)2.3952155E38F;
            p101.x = (float)9.251335E35F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)7.891831E37F);
                Debug.Assert(pack.z == (float) -5.5924546E37F);
                Debug.Assert(pack.roll == (float)6.2647886E37F);
                Debug.Assert(pack.pitch == (float)3.083929E38F);
                Debug.Assert(pack.usec == (ulong)1128497092868924494L);
                Debug.Assert(pack.y == (float) -2.9520908E37F);
                Debug.Assert(pack.x == (float) -2.3418314E38F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float)3.083929E38F;
            p102.x = (float) -2.3418314E38F;
            p102.y = (float) -2.9520908E37F;
            p102.usec = (ulong)1128497092868924494L;
            p102.yaw = (float)7.891831E37F;
            p102.z = (float) -5.5924546E37F;
            p102.roll = (float)6.2647886E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3501608852183188329L);
                Debug.Assert(pack.z == (float)2.9532203E38F);
                Debug.Assert(pack.x == (float) -1.1005495E38F);
                Debug.Assert(pack.y == (float) -2.1492961E38F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float) -2.1492961E38F;
            p103.z = (float)2.9532203E38F;
            p103.x = (float) -1.1005495E38F;
            p103.usec = (ulong)3501608852183188329L;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.7648188E38F);
                Debug.Assert(pack.x == (float) -2.4920944E38F);
                Debug.Assert(pack.usec == (ulong)1449508588289773247L);
                Debug.Assert(pack.yaw == (float) -2.1032518E38F);
                Debug.Assert(pack.pitch == (float)1.4411323E38F);
                Debug.Assert(pack.roll == (float)1.2922716E38F);
                Debug.Assert(pack.y == (float) -1.524404E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float) -2.1032518E38F;
            p104.y = (float) -1.524404E38F;
            p104.z = (float) -2.7648188E38F;
            p104.pitch = (float)1.4411323E38F;
            p104.roll = (float)1.2922716E38F;
            p104.x = (float) -2.4920944E38F;
            p104.usec = (ulong)1449508588289773247L;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -8.3327663E37F);
                Debug.Assert(pack.zgyro == (float)5.779398E37F);
                Debug.Assert(pack.xgyro == (float)3.1937723E38F);
                Debug.Assert(pack.yacc == (float)1.2356936E38F);
                Debug.Assert(pack.diff_pressure == (float) -9.614247E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)40110);
                Debug.Assert(pack.abs_pressure == (float) -2.8373807E38F);
                Debug.Assert(pack.time_usec == (ulong)4937243899102967083L);
                Debug.Assert(pack.pressure_alt == (float)3.2424665E38F);
                Debug.Assert(pack.xmag == (float) -5.5694467E37F);
                Debug.Assert(pack.ymag == (float)5.298977E37F);
                Debug.Assert(pack.zacc == (float) -1.4034125E38F);
                Debug.Assert(pack.temperature == (float) -2.351501E38F);
                Debug.Assert(pack.zmag == (float) -3.3575594E38F);
                Debug.Assert(pack.xacc == (float) -2.796783E37F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float) -1.4034125E38F;
            p105.time_usec = (ulong)4937243899102967083L;
            p105.fields_updated = (ushort)(ushort)40110;
            p105.xmag = (float) -5.5694467E37F;
            p105.diff_pressure = (float) -9.614247E37F;
            p105.pressure_alt = (float)3.2424665E38F;
            p105.abs_pressure = (float) -2.8373807E38F;
            p105.xgyro = (float)3.1937723E38F;
            p105.zgyro = (float)5.779398E37F;
            p105.xacc = (float) -2.796783E37F;
            p105.ymag = (float)5.298977E37F;
            p105.temperature = (float) -2.351501E38F;
            p105.ygyro = (float) -8.3327663E37F;
            p105.yacc = (float)1.2356936E38F;
            p105.zmag = (float) -3.3575594E38F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)11303);
                Debug.Assert(pack.integrated_xgyro == (float) -5.065879E36F);
                Debug.Assert(pack.integrated_ygyro == (float) -2.8777792E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)361070411U);
                Debug.Assert(pack.sensor_id == (byte)(byte)43);
                Debug.Assert(pack.distance == (float)2.4919443E38F);
                Debug.Assert(pack.integrated_x == (float) -1.0298288E38F);
                Debug.Assert(pack.integration_time_us == (uint)3942164609U);
                Debug.Assert(pack.integrated_y == (float) -2.5955978E38F);
                Debug.Assert(pack.time_usec == (ulong)6450883176851881768L);
                Debug.Assert(pack.quality == (byte)(byte)204);
                Debug.Assert(pack.integrated_zgyro == (float) -1.0993275E38F);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float) -2.5955978E38F;
            p106.integrated_xgyro = (float) -5.065879E36F;
            p106.time_usec = (ulong)6450883176851881768L;
            p106.distance = (float)2.4919443E38F;
            p106.integrated_x = (float) -1.0298288E38F;
            p106.integrated_ygyro = (float) -2.8777792E38F;
            p106.integrated_zgyro = (float) -1.0993275E38F;
            p106.integration_time_us = (uint)3942164609U;
            p106.sensor_id = (byte)(byte)43;
            p106.quality = (byte)(byte)204;
            p106.time_delta_distance_us = (uint)361070411U;
            p106.temperature = (short)(short)11303;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diff_pressure == (float) -2.2056768E38F);
                Debug.Assert(pack.pressure_alt == (float)2.5490588E38F);
                Debug.Assert(pack.temperature == (float) -2.1374159E38F);
                Debug.Assert(pack.yacc == (float)2.8678875E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.2405856E38F);
                Debug.Assert(pack.xmag == (float)7.094498E37F);
                Debug.Assert(pack.time_usec == (ulong)4447493157642595709L);
                Debug.Assert(pack.zacc == (float)1.3258253E38F);
                Debug.Assert(pack.ymag == (float) -2.1897153E38F);
                Debug.Assert(pack.fields_updated == (uint)4215972386U);
                Debug.Assert(pack.ygyro == (float)1.812832E37F);
                Debug.Assert(pack.xacc == (float) -1.8814093E38F);
                Debug.Assert(pack.zmag == (float) -2.3192788E37F);
                Debug.Assert(pack.xgyro == (float)9.841613E37F);
                Debug.Assert(pack.zgyro == (float)1.8751528E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.ygyro = (float)1.812832E37F;
            p107.fields_updated = (uint)4215972386U;
            p107.ymag = (float) -2.1897153E38F;
            p107.temperature = (float) -2.1374159E38F;
            p107.yacc = (float)2.8678875E38F;
            p107.pressure_alt = (float)2.5490588E38F;
            p107.zmag = (float) -2.3192788E37F;
            p107.zgyro = (float)1.8751528E38F;
            p107.time_usec = (ulong)4447493157642595709L;
            p107.diff_pressure = (float) -2.2056768E38F;
            p107.xacc = (float) -1.8814093E38F;
            p107.abs_pressure = (float) -1.2405856E38F;
            p107.xmag = (float)7.094498E37F;
            p107.xgyro = (float)9.841613E37F;
            p107.zacc = (float)1.3258253E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (float) -1.87866E38F);
                Debug.Assert(pack.yaw == (float)2.6673697E38F);
                Debug.Assert(pack.std_dev_vert == (float) -1.6544676E38F);
                Debug.Assert(pack.zacc == (float)1.7167334E38F);
                Debug.Assert(pack.yacc == (float)2.966322E38F);
                Debug.Assert(pack.q2 == (float) -1.9705722E38F);
                Debug.Assert(pack.vd == (float) -3.2637884E38F);
                Debug.Assert(pack.vn == (float) -3.0706802E37F);
                Debug.Assert(pack.lon == (float) -1.2198194E38F);
                Debug.Assert(pack.std_dev_horz == (float)2.2533832E38F);
                Debug.Assert(pack.q4 == (float) -9.100171E37F);
                Debug.Assert(pack.q1 == (float) -2.0578578E37F);
                Debug.Assert(pack.roll == (float)9.750792E37F);
                Debug.Assert(pack.ygyro == (float)2.4755721E38F);
                Debug.Assert(pack.zgyro == (float)3.0955736E38F);
                Debug.Assert(pack.ve == (float) -2.2525048E38F);
                Debug.Assert(pack.xacc == (float)1.0294165E38F);
                Debug.Assert(pack.pitch == (float)2.6278586E38F);
                Debug.Assert(pack.q3 == (float)1.5809341E38F);
                Debug.Assert(pack.xgyro == (float) -2.5639055E37F);
                Debug.Assert(pack.alt == (float)2.896747E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float)2.896747E38F;
            p108.lat = (float) -1.87866E38F;
            p108.vd = (float) -3.2637884E38F;
            p108.q1 = (float) -2.0578578E37F;
            p108.std_dev_vert = (float) -1.6544676E38F;
            p108.ve = (float) -2.2525048E38F;
            p108.xgyro = (float) -2.5639055E37F;
            p108.zgyro = (float)3.0955736E38F;
            p108.roll = (float)9.750792E37F;
            p108.q2 = (float) -1.9705722E38F;
            p108.q3 = (float)1.5809341E38F;
            p108.ygyro = (float)2.4755721E38F;
            p108.vn = (float) -3.0706802E37F;
            p108.zacc = (float)1.7167334E38F;
            p108.xacc = (float)1.0294165E38F;
            p108.std_dev_horz = (float)2.2533832E38F;
            p108.yacc = (float)2.966322E38F;
            p108.q4 = (float) -9.100171E37F;
            p108.yaw = (float)2.6673697E38F;
            p108.pitch = (float)2.6278586E38F;
            p108.lon = (float) -1.2198194E38F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)43652);
                Debug.Assert(pack.noise == (byte)(byte)197);
                Debug.Assert(pack.remnoise == (byte)(byte)95);
                Debug.Assert(pack.remrssi == (byte)(byte)50);
                Debug.Assert(pack.rssi == (byte)(byte)175);
                Debug.Assert(pack.txbuf == (byte)(byte)7);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)42502);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remnoise = (byte)(byte)95;
            p109.rxerrors = (ushort)(ushort)42502;
            p109.noise = (byte)(byte)197;
            p109.rssi = (byte)(byte)175;
            p109.remrssi = (byte)(byte)50;
            p109.fixed_ = (ushort)(ushort)43652;
            p109.txbuf = (byte)(byte)7;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)50);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)189, (byte)42, (byte)118, (byte)248, (byte)189, (byte)79, (byte)7, (byte)101, (byte)204, (byte)150, (byte)144, (byte)202, (byte)19, (byte)5, (byte)196, (byte)168, (byte)31, (byte)207, (byte)6, (byte)65, (byte)2, (byte)101, (byte)10, (byte)91, (byte)146, (byte)246, (byte)105, (byte)4, (byte)120, (byte)203, (byte)79, (byte)81, (byte)21, (byte)217, (byte)187, (byte)48, (byte)196, (byte)7, (byte)217, (byte)149, (byte)161, (byte)124, (byte)94, (byte)41, (byte)212, (byte)198, (byte)251, (byte)102, (byte)166, (byte)53, (byte)167, (byte)178, (byte)169, (byte)51, (byte)69, (byte)243, (byte)23, (byte)121, (byte)247, (byte)173, (byte)250, (byte)125, (byte)44, (byte)141, (byte)71, (byte)159, (byte)205, (byte)18, (byte)61, (byte)32, (byte)212, (byte)242, (byte)136, (byte)94, (byte)101, (byte)0, (byte)174, (byte)212, (byte)195, (byte)173, (byte)147, (byte)43, (byte)70, (byte)84, (byte)77, (byte)4, (byte)19, (byte)85, (byte)41, (byte)39, (byte)37, (byte)45, (byte)20, (byte)198, (byte)147, (byte)205, (byte)213, (byte)163, (byte)239, (byte)240, (byte)198, (byte)175, (byte)10, (byte)36, (byte)211, (byte)145, (byte)117, (byte)151, (byte)15, (byte)195, (byte)211, (byte)109, (byte)190, (byte)5, (byte)199, (byte)237, (byte)47, (byte)16, (byte)49, (byte)99, (byte)157, (byte)145, (byte)3, (byte)223, (byte)179, (byte)71, (byte)201, (byte)200, (byte)244, (byte)105, (byte)208, (byte)75, (byte)181, (byte)74, (byte)67, (byte)165, (byte)201, (byte)224, (byte)107, (byte)4, (byte)116, (byte)100, (byte)86, (byte)228, (byte)70, (byte)173, (byte)253, (byte)157, (byte)26, (byte)57, (byte)78, (byte)10, (byte)189, (byte)148, (byte)22, (byte)86, (byte)0, (byte)174, (byte)139, (byte)28, (byte)102, (byte)159, (byte)50, (byte)74, (byte)228, (byte)148, (byte)148, (byte)165, (byte)20, (byte)26, (byte)165, (byte)195, (byte)151, (byte)43, (byte)94, (byte)23, (byte)188, (byte)164, (byte)249, (byte)34, (byte)27, (byte)121, (byte)214, (byte)110, (byte)252, (byte)116, (byte)100, (byte)147, (byte)46, (byte)147, (byte)146, (byte)19, (byte)91, (byte)90, (byte)106, (byte)229, (byte)240, (byte)139, (byte)214, (byte)196, (byte)174, (byte)8, (byte)226, (byte)96, (byte)180, (byte)211, (byte)153, (byte)241, (byte)162, (byte)200, (byte)136, (byte)224, (byte)245, (byte)18, (byte)78, (byte)186, (byte)243, (byte)26, (byte)32, (byte)210, (byte)175, (byte)214, (byte)112, (byte)21, (byte)33, (byte)201, (byte)243, (byte)174, (byte)213, (byte)65, (byte)29, (byte)142, (byte)134, (byte)48, (byte)9, (byte)235, (byte)125, (byte)100, (byte)180, (byte)216, (byte)24, (byte)137, (byte)100, (byte)183, (byte)115, (byte)139, (byte)37, (byte)138, (byte)142, (byte)10, (byte)105}));
                Debug.Assert(pack.target_system == (byte)(byte)235);
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)216;
            p110.target_system = (byte)(byte)235;
            p110.payload_SET(new byte[] {(byte)189, (byte)42, (byte)118, (byte)248, (byte)189, (byte)79, (byte)7, (byte)101, (byte)204, (byte)150, (byte)144, (byte)202, (byte)19, (byte)5, (byte)196, (byte)168, (byte)31, (byte)207, (byte)6, (byte)65, (byte)2, (byte)101, (byte)10, (byte)91, (byte)146, (byte)246, (byte)105, (byte)4, (byte)120, (byte)203, (byte)79, (byte)81, (byte)21, (byte)217, (byte)187, (byte)48, (byte)196, (byte)7, (byte)217, (byte)149, (byte)161, (byte)124, (byte)94, (byte)41, (byte)212, (byte)198, (byte)251, (byte)102, (byte)166, (byte)53, (byte)167, (byte)178, (byte)169, (byte)51, (byte)69, (byte)243, (byte)23, (byte)121, (byte)247, (byte)173, (byte)250, (byte)125, (byte)44, (byte)141, (byte)71, (byte)159, (byte)205, (byte)18, (byte)61, (byte)32, (byte)212, (byte)242, (byte)136, (byte)94, (byte)101, (byte)0, (byte)174, (byte)212, (byte)195, (byte)173, (byte)147, (byte)43, (byte)70, (byte)84, (byte)77, (byte)4, (byte)19, (byte)85, (byte)41, (byte)39, (byte)37, (byte)45, (byte)20, (byte)198, (byte)147, (byte)205, (byte)213, (byte)163, (byte)239, (byte)240, (byte)198, (byte)175, (byte)10, (byte)36, (byte)211, (byte)145, (byte)117, (byte)151, (byte)15, (byte)195, (byte)211, (byte)109, (byte)190, (byte)5, (byte)199, (byte)237, (byte)47, (byte)16, (byte)49, (byte)99, (byte)157, (byte)145, (byte)3, (byte)223, (byte)179, (byte)71, (byte)201, (byte)200, (byte)244, (byte)105, (byte)208, (byte)75, (byte)181, (byte)74, (byte)67, (byte)165, (byte)201, (byte)224, (byte)107, (byte)4, (byte)116, (byte)100, (byte)86, (byte)228, (byte)70, (byte)173, (byte)253, (byte)157, (byte)26, (byte)57, (byte)78, (byte)10, (byte)189, (byte)148, (byte)22, (byte)86, (byte)0, (byte)174, (byte)139, (byte)28, (byte)102, (byte)159, (byte)50, (byte)74, (byte)228, (byte)148, (byte)148, (byte)165, (byte)20, (byte)26, (byte)165, (byte)195, (byte)151, (byte)43, (byte)94, (byte)23, (byte)188, (byte)164, (byte)249, (byte)34, (byte)27, (byte)121, (byte)214, (byte)110, (byte)252, (byte)116, (byte)100, (byte)147, (byte)46, (byte)147, (byte)146, (byte)19, (byte)91, (byte)90, (byte)106, (byte)229, (byte)240, (byte)139, (byte)214, (byte)196, (byte)174, (byte)8, (byte)226, (byte)96, (byte)180, (byte)211, (byte)153, (byte)241, (byte)162, (byte)200, (byte)136, (byte)224, (byte)245, (byte)18, (byte)78, (byte)186, (byte)243, (byte)26, (byte)32, (byte)210, (byte)175, (byte)214, (byte)112, (byte)21, (byte)33, (byte)201, (byte)243, (byte)174, (byte)213, (byte)65, (byte)29, (byte)142, (byte)134, (byte)48, (byte)9, (byte)235, (byte)125, (byte)100, (byte)180, (byte)216, (byte)24, (byte)137, (byte)100, (byte)183, (byte)115, (byte)139, (byte)37, (byte)138, (byte)142, (byte)10, (byte)105}, 0) ;
            p110.target_network = (byte)(byte)50;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -3058313157017717915L);
                Debug.Assert(pack.ts1 == (long) -6521898215383038613L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -6521898215383038613L;
            p111.tc1 = (long) -3058313157017717915L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)379863822U);
                Debug.Assert(pack.time_usec == (ulong)8438156355358908606L);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8438156355358908606L;
            p112.seq = (uint)379863822U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -749395092);
                Debug.Assert(pack.eph == (ushort)(ushort)17835);
                Debug.Assert(pack.alt == (int) -288805119);
                Debug.Assert(pack.time_usec == (ulong)8798818002912507108L);
                Debug.Assert(pack.vd == (short)(short)6312);
                Debug.Assert(pack.satellites_visible == (byte)(byte)122);
                Debug.Assert(pack.vn == (short)(short) -16257);
                Debug.Assert(pack.lon == (int)179489174);
                Debug.Assert(pack.ve == (short)(short) -25538);
                Debug.Assert(pack.cog == (ushort)(ushort)50196);
                Debug.Assert(pack.fix_type == (byte)(byte)120);
                Debug.Assert(pack.vel == (ushort)(ushort)16504);
                Debug.Assert(pack.epv == (ushort)(ushort)36272);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)6312;
            p113.epv = (ushort)(ushort)36272;
            p113.time_usec = (ulong)8798818002912507108L;
            p113.lat = (int) -749395092;
            p113.ve = (short)(short) -25538;
            p113.satellites_visible = (byte)(byte)122;
            p113.vel = (ushort)(ushort)16504;
            p113.lon = (int)179489174;
            p113.eph = (ushort)(ushort)17835;
            p113.alt = (int) -288805119;
            p113.cog = (ushort)(ushort)50196;
            p113.fix_type = (byte)(byte)120;
            p113.vn = (short)(short) -16257;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -1822);
                Debug.Assert(pack.integrated_ygyro == (float)1.6715049E38F);
                Debug.Assert(pack.integrated_y == (float) -2.9505864E38F);
                Debug.Assert(pack.integrated_x == (float) -1.9200268E38F);
                Debug.Assert(pack.distance == (float) -2.1709549E38F);
                Debug.Assert(pack.integrated_xgyro == (float)1.6732638E38F);
                Debug.Assert(pack.time_usec == (ulong)8573300658579695800L);
                Debug.Assert(pack.integrated_zgyro == (float)2.201929E38F);
                Debug.Assert(pack.quality == (byte)(byte)12);
                Debug.Assert(pack.time_delta_distance_us == (uint)2207974451U);
                Debug.Assert(pack.integration_time_us == (uint)3705489045U);
                Debug.Assert(pack.sensor_id == (byte)(byte)21);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_x = (float) -1.9200268E38F;
            p114.integrated_ygyro = (float)1.6715049E38F;
            p114.quality = (byte)(byte)12;
            p114.integrated_y = (float) -2.9505864E38F;
            p114.sensor_id = (byte)(byte)21;
            p114.time_usec = (ulong)8573300658579695800L;
            p114.temperature = (short)(short) -1822;
            p114.distance = (float) -2.1709549E38F;
            p114.integration_time_us = (uint)3705489045U;
            p114.integrated_xgyro = (float)1.6732638E38F;
            p114.integrated_zgyro = (float)2.201929E38F;
            p114.time_delta_distance_us = (uint)2207974451U;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -5251);
                Debug.Assert(pack.lat == (int) -1731455114);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)6697);
                Debug.Assert(pack.vx == (short)(short) -15847);
                Debug.Assert(pack.lon == (int) -960365238);
                Debug.Assert(pack.time_usec == (ulong)7174422490116179370L);
                Debug.Assert(pack.xacc == (short)(short)31725);
                Debug.Assert(pack.yawspeed == (float) -6.2169104E37F);
                Debug.Assert(pack.yacc == (short)(short) -31324);
                Debug.Assert(pack.alt == (int)637236954);
                Debug.Assert(pack.rollspeed == (float) -2.2330988E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.0070262E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.6176348E38F, -7.4639693E37F, 2.7642955E38F, 1.699943E38F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)64592);
                Debug.Assert(pack.vy == (short)(short) -25266);
                Debug.Assert(pack.vz == (short)(short) -27677);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.yacc = (short)(short) -31324;
            p115.vz = (short)(short) -27677;
            p115.lon = (int) -960365238;
            p115.xacc = (short)(short)31725;
            p115.rollspeed = (float) -2.2330988E38F;
            p115.yawspeed = (float) -6.2169104E37F;
            p115.true_airspeed = (ushort)(ushort)64592;
            p115.vx = (short)(short) -15847;
            p115.alt = (int)637236954;
            p115.attitude_quaternion_SET(new float[] {-2.6176348E38F, -7.4639693E37F, 2.7642955E38F, 1.699943E38F}, 0) ;
            p115.time_usec = (ulong)7174422490116179370L;
            p115.vy = (short)(short) -25266;
            p115.zacc = (short)(short) -5251;
            p115.ind_airspeed = (ushort)(ushort)6697;
            p115.pitchspeed = (float) -2.0070262E38F;
            p115.lat = (int) -1731455114;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -31187);
                Debug.Assert(pack.ymag == (short)(short) -21572);
                Debug.Assert(pack.xmag == (short)(short) -12847);
                Debug.Assert(pack.xacc == (short)(short) -7795);
                Debug.Assert(pack.xgyro == (short)(short)28090);
                Debug.Assert(pack.time_boot_ms == (uint)2542514475U);
                Debug.Assert(pack.yacc == (short)(short)30586);
                Debug.Assert(pack.zacc == (short)(short)11879);
                Debug.Assert(pack.ygyro == (short)(short) -32539);
                Debug.Assert(pack.zgyro == (short)(short)27578);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xmag = (short)(short) -12847;
            p116.zgyro = (short)(short)27578;
            p116.ygyro = (short)(short) -32539;
            p116.xacc = (short)(short) -7795;
            p116.zacc = (short)(short)11879;
            p116.time_boot_ms = (uint)2542514475U;
            p116.ymag = (short)(short) -21572;
            p116.zmag = (short)(short) -31187;
            p116.xgyro = (short)(short)28090;
            p116.yacc = (short)(short)30586;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.end == (ushort)(ushort)2829);
                Debug.Assert(pack.start == (ushort)(ushort)36823);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)36823;
            p117.target_system = (byte)(byte)204;
            p117.target_component = (byte)(byte)40;
            p117.end = (ushort)(ushort)2829;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)4186520525U);
                Debug.Assert(pack.id == (ushort)(ushort)54464);
                Debug.Assert(pack.num_logs == (ushort)(ushort)42340);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)61413);
                Debug.Assert(pack.time_utc == (uint)2310469618U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.last_log_num = (ushort)(ushort)61413;
            p118.num_logs = (ushort)(ushort)42340;
            p118.time_utc = (uint)2310469618U;
            p118.size = (uint)4186520525U;
            p118.id = (ushort)(ushort)54464;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)3763619704U);
                Debug.Assert(pack.id == (ushort)(ushort)59490);
                Debug.Assert(pack.target_system == (byte)(byte)187);
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.ofs == (uint)3691754908U);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)3691754908U;
            p119.id = (ushort)(ushort)59490;
            p119.count = (uint)3763619704U;
            p119.target_system = (byte)(byte)187;
            p119.target_component = (byte)(byte)38;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)64373);
                Debug.Assert(pack.ofs == (uint)2709200220U);
                Debug.Assert(pack.count == (byte)(byte)194);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)154, (byte)87, (byte)125, (byte)37, (byte)53, (byte)205, (byte)86, (byte)232, (byte)160, (byte)150, (byte)181, (byte)64, (byte)166, (byte)1, (byte)175, (byte)206, (byte)182, (byte)233, (byte)83, (byte)166, (byte)23, (byte)174, (byte)44, (byte)170, (byte)207, (byte)133, (byte)251, (byte)148, (byte)152, (byte)217, (byte)178, (byte)42, (byte)127, (byte)127, (byte)232, (byte)238, (byte)145, (byte)248, (byte)27, (byte)44, (byte)167, (byte)163, (byte)213, (byte)89, (byte)46, (byte)30, (byte)148, (byte)110, (byte)112, (byte)57, (byte)242, (byte)235, (byte)185, (byte)167, (byte)59, (byte)207, (byte)232, (byte)103, (byte)175, (byte)121, (byte)250, (byte)50, (byte)61, (byte)140, (byte)120, (byte)232, (byte)42, (byte)57, (byte)66, (byte)74, (byte)108, (byte)113, (byte)153, (byte)149, (byte)6, (byte)124, (byte)117, (byte)56, (byte)71, (byte)100, (byte)52, (byte)166, (byte)50, (byte)138, (byte)253, (byte)188, (byte)157, (byte)156, (byte)188, (byte)19}));
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)154, (byte)87, (byte)125, (byte)37, (byte)53, (byte)205, (byte)86, (byte)232, (byte)160, (byte)150, (byte)181, (byte)64, (byte)166, (byte)1, (byte)175, (byte)206, (byte)182, (byte)233, (byte)83, (byte)166, (byte)23, (byte)174, (byte)44, (byte)170, (byte)207, (byte)133, (byte)251, (byte)148, (byte)152, (byte)217, (byte)178, (byte)42, (byte)127, (byte)127, (byte)232, (byte)238, (byte)145, (byte)248, (byte)27, (byte)44, (byte)167, (byte)163, (byte)213, (byte)89, (byte)46, (byte)30, (byte)148, (byte)110, (byte)112, (byte)57, (byte)242, (byte)235, (byte)185, (byte)167, (byte)59, (byte)207, (byte)232, (byte)103, (byte)175, (byte)121, (byte)250, (byte)50, (byte)61, (byte)140, (byte)120, (byte)232, (byte)42, (byte)57, (byte)66, (byte)74, (byte)108, (byte)113, (byte)153, (byte)149, (byte)6, (byte)124, (byte)117, (byte)56, (byte)71, (byte)100, (byte)52, (byte)166, (byte)50, (byte)138, (byte)253, (byte)188, (byte)157, (byte)156, (byte)188, (byte)19}, 0) ;
            p120.ofs = (uint)2709200220U;
            p120.id = (ushort)(ushort)64373;
            p120.count = (byte)(byte)194;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)188);
                Debug.Assert(pack.target_system == (byte)(byte)1);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)1;
            p121.target_component = (byte)(byte)188;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)45);
                Debug.Assert(pack.target_system == (byte)(byte)184);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)45;
            p122.target_system = (byte)(byte)184;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)74);
                Debug.Assert(pack.len == (byte)(byte)175);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)230, (byte)77, (byte)44, (byte)202, (byte)60, (byte)107, (byte)239, (byte)112, (byte)226, (byte)205, (byte)132, (byte)56, (byte)76, (byte)68, (byte)110, (byte)105, (byte)9, (byte)98, (byte)125, (byte)135, (byte)16, (byte)72, (byte)243, (byte)198, (byte)127, (byte)74, (byte)89, (byte)14, (byte)246, (byte)237, (byte)77, (byte)217, (byte)80, (byte)117, (byte)214, (byte)128, (byte)115, (byte)170, (byte)38, (byte)63, (byte)204, (byte)241, (byte)217, (byte)226, (byte)172, (byte)136, (byte)186, (byte)202, (byte)116, (byte)247, (byte)216, (byte)132, (byte)102, (byte)205, (byte)0, (byte)61, (byte)146, (byte)145, (byte)55, (byte)1, (byte)7, (byte)255, (byte)185, (byte)104, (byte)245, (byte)239, (byte)238, (byte)172, (byte)197, (byte)55, (byte)1, (byte)73, (byte)75, (byte)217, (byte)171, (byte)227, (byte)130, (byte)177, (byte)186, (byte)83, (byte)122, (byte)27, (byte)174, (byte)132, (byte)230, (byte)171, (byte)112, (byte)235, (byte)0, (byte)106, (byte)193, (byte)143, (byte)50, (byte)39, (byte)67, (byte)222, (byte)46, (byte)98, (byte)190, (byte)157, (byte)190, (byte)127, (byte)111, (byte)178, (byte)62, (byte)30, (byte)97, (byte)206, (byte)85, (byte)7}));
                Debug.Assert(pack.target_system == (byte)(byte)123);
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)230, (byte)77, (byte)44, (byte)202, (byte)60, (byte)107, (byte)239, (byte)112, (byte)226, (byte)205, (byte)132, (byte)56, (byte)76, (byte)68, (byte)110, (byte)105, (byte)9, (byte)98, (byte)125, (byte)135, (byte)16, (byte)72, (byte)243, (byte)198, (byte)127, (byte)74, (byte)89, (byte)14, (byte)246, (byte)237, (byte)77, (byte)217, (byte)80, (byte)117, (byte)214, (byte)128, (byte)115, (byte)170, (byte)38, (byte)63, (byte)204, (byte)241, (byte)217, (byte)226, (byte)172, (byte)136, (byte)186, (byte)202, (byte)116, (byte)247, (byte)216, (byte)132, (byte)102, (byte)205, (byte)0, (byte)61, (byte)146, (byte)145, (byte)55, (byte)1, (byte)7, (byte)255, (byte)185, (byte)104, (byte)245, (byte)239, (byte)238, (byte)172, (byte)197, (byte)55, (byte)1, (byte)73, (byte)75, (byte)217, (byte)171, (byte)227, (byte)130, (byte)177, (byte)186, (byte)83, (byte)122, (byte)27, (byte)174, (byte)132, (byte)230, (byte)171, (byte)112, (byte)235, (byte)0, (byte)106, (byte)193, (byte)143, (byte)50, (byte)39, (byte)67, (byte)222, (byte)46, (byte)98, (byte)190, (byte)157, (byte)190, (byte)127, (byte)111, (byte)178, (byte)62, (byte)30, (byte)97, (byte)206, (byte)85, (byte)7}, 0) ;
            p123.len = (byte)(byte)175;
            p123.target_component = (byte)(byte)74;
            p123.target_system = (byte)(byte)123;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)141);
                Debug.Assert(pack.time_usec == (ulong)5260556258154986709L);
                Debug.Assert(pack.dgps_age == (uint)4151730343U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.lat == (int) -1880540621);
                Debug.Assert(pack.lon == (int)1483958716);
                Debug.Assert(pack.alt == (int) -794781473);
                Debug.Assert(pack.eph == (ushort)(ushort)52649);
                Debug.Assert(pack.cog == (ushort)(ushort)18072);
                Debug.Assert(pack.vel == (ushort)(ushort)35466);
                Debug.Assert(pack.epv == (ushort)(ushort)26541);
                Debug.Assert(pack.dgps_numch == (byte)(byte)140);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_numch = (byte)(byte)140;
            p124.alt = (int) -794781473;
            p124.satellites_visible = (byte)(byte)141;
            p124.dgps_age = (uint)4151730343U;
            p124.epv = (ushort)(ushort)26541;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.vel = (ushort)(ushort)35466;
            p124.cog = (ushort)(ushort)18072;
            p124.time_usec = (ulong)5260556258154986709L;
            p124.lon = (int)1483958716;
            p124.lat = (int) -1880540621;
            p124.eph = (ushort)(ushort)52649;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)2462);
                Debug.Assert(pack.Vservo == (ushort)(ushort)6006);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)2462;
            p125.Vservo = (ushort)(ushort)6006;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
                Debug.Assert(pack.timeout == (ushort)(ushort)14716);
                Debug.Assert(pack.count == (byte)(byte)234);
                Debug.Assert(pack.baudrate == (uint)4240866180U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)174, (byte)239, (byte)34, (byte)239, (byte)64, (byte)180, (byte)223, (byte)199, (byte)6, (byte)119, (byte)130, (byte)215, (byte)9, (byte)221, (byte)189, (byte)136, (byte)64, (byte)118, (byte)112, (byte)49, (byte)13, (byte)120, (byte)127, (byte)215, (byte)38, (byte)50, (byte)3, (byte)87, (byte)250, (byte)160, (byte)87, (byte)24, (byte)44, (byte)1, (byte)16, (byte)153, (byte)145, (byte)101, (byte)254, (byte)243, (byte)232, (byte)160, (byte)72, (byte)104, (byte)202, (byte)189, (byte)133, (byte)151, (byte)184, (byte)192, (byte)208, (byte)42, (byte)145, (byte)18, (byte)13, (byte)128, (byte)57, (byte)33, (byte)55, (byte)244, (byte)175, (byte)10, (byte)90, (byte)94, (byte)162, (byte)203, (byte)107, (byte)26, (byte)38, (byte)66}));
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.timeout = (ushort)(ushort)14716;
            p126.data__SET(new byte[] {(byte)174, (byte)239, (byte)34, (byte)239, (byte)64, (byte)180, (byte)223, (byte)199, (byte)6, (byte)119, (byte)130, (byte)215, (byte)9, (byte)221, (byte)189, (byte)136, (byte)64, (byte)118, (byte)112, (byte)49, (byte)13, (byte)120, (byte)127, (byte)215, (byte)38, (byte)50, (byte)3, (byte)87, (byte)250, (byte)160, (byte)87, (byte)24, (byte)44, (byte)1, (byte)16, (byte)153, (byte)145, (byte)101, (byte)254, (byte)243, (byte)232, (byte)160, (byte)72, (byte)104, (byte)202, (byte)189, (byte)133, (byte)151, (byte)184, (byte)192, (byte)208, (byte)42, (byte)145, (byte)18, (byte)13, (byte)128, (byte)57, (byte)33, (byte)55, (byte)244, (byte)175, (byte)10, (byte)90, (byte)94, (byte)162, (byte)203, (byte)107, (byte)26, (byte)38, (byte)66}, 0) ;
            p126.baudrate = (uint)4240866180U;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.count = (byte)(byte)234;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)2938471568U);
                Debug.Assert(pack.tow == (uint)762196392U);
                Debug.Assert(pack.baseline_b_mm == (int) -852424631);
                Debug.Assert(pack.baseline_c_mm == (int) -99882596);
                Debug.Assert(pack.rtk_rate == (byte)(byte)203);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1230315784);
                Debug.Assert(pack.wn == (ushort)(ushort)34048);
                Debug.Assert(pack.nsats == (byte)(byte)139);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)84);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)113);
                Debug.Assert(pack.baseline_a_mm == (int)1755195777);
                Debug.Assert(pack.rtk_health == (byte)(byte)15);
                Debug.Assert(pack.accuracy == (uint)2005548291U);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_receiver_id = (byte)(byte)84;
            p127.accuracy = (uint)2005548291U;
            p127.baseline_a_mm = (int)1755195777;
            p127.rtk_rate = (byte)(byte)203;
            p127.rtk_health = (byte)(byte)15;
            p127.baseline_b_mm = (int) -852424631;
            p127.baseline_coords_type = (byte)(byte)113;
            p127.tow = (uint)762196392U;
            p127.iar_num_hypotheses = (int) -1230315784;
            p127.wn = (ushort)(ushort)34048;
            p127.nsats = (byte)(byte)139;
            p127.baseline_c_mm = (int) -99882596;
            p127.time_last_baseline_ms = (uint)2938471568U;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)1692699150U);
                Debug.Assert(pack.baseline_a_mm == (int) -39751109);
                Debug.Assert(pack.rtk_rate == (byte)(byte)241);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)204);
                Debug.Assert(pack.nsats == (byte)(byte)211);
                Debug.Assert(pack.iar_num_hypotheses == (int) -754257225);
                Debug.Assert(pack.baseline_b_mm == (int) -1686826663);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)82);
                Debug.Assert(pack.tow == (uint)4005489712U);
                Debug.Assert(pack.accuracy == (uint)815311460U);
                Debug.Assert(pack.baseline_c_mm == (int)1397272304);
                Debug.Assert(pack.wn == (ushort)(ushort)30048);
                Debug.Assert(pack.rtk_health == (byte)(byte)87);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.tow = (uint)4005489712U;
            p128.rtk_rate = (byte)(byte)241;
            p128.baseline_a_mm = (int) -39751109;
            p128.baseline_coords_type = (byte)(byte)82;
            p128.accuracy = (uint)815311460U;
            p128.iar_num_hypotheses = (int) -754257225;
            p128.baseline_c_mm = (int)1397272304;
            p128.rtk_health = (byte)(byte)87;
            p128.baseline_b_mm = (int) -1686826663;
            p128.time_last_baseline_ms = (uint)1692699150U;
            p128.rtk_receiver_id = (byte)(byte)204;
            p128.wn = (ushort)(ushort)30048;
            p128.nsats = (byte)(byte)211;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4049459494U);
                Debug.Assert(pack.zacc == (short)(short) -1319);
                Debug.Assert(pack.zgyro == (short)(short)30784);
                Debug.Assert(pack.zmag == (short)(short) -24871);
                Debug.Assert(pack.ymag == (short)(short) -2441);
                Debug.Assert(pack.xmag == (short)(short) -3991);
                Debug.Assert(pack.ygyro == (short)(short)20126);
                Debug.Assert(pack.yacc == (short)(short)3337);
                Debug.Assert(pack.xacc == (short)(short) -28421);
                Debug.Assert(pack.xgyro == (short)(short) -7938);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short) -3991;
            p129.yacc = (short)(short)3337;
            p129.zmag = (short)(short) -24871;
            p129.zacc = (short)(short) -1319;
            p129.ygyro = (short)(short)20126;
            p129.ymag = (short)(short) -2441;
            p129.zgyro = (short)(short)30784;
            p129.xgyro = (short)(short) -7938;
            p129.time_boot_ms = (uint)4049459494U;
            p129.xacc = (short)(short) -28421;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)64602);
                Debug.Assert(pack.type == (byte)(byte)238);
                Debug.Assert(pack.size == (uint)1605411734U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)203);
                Debug.Assert(pack.packets == (ushort)(ushort)58395);
                Debug.Assert(pack.width == (ushort)(ushort)42515);
                Debug.Assert(pack.payload == (byte)(byte)114);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)203;
            p130.type = (byte)(byte)238;
            p130.width = (ushort)(ushort)42515;
            p130.payload = (byte)(byte)114;
            p130.size = (uint)1605411734U;
            p130.packets = (ushort)(ushort)58395;
            p130.height = (ushort)(ushort)64602;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)236, (byte)176, (byte)117, (byte)50, (byte)94, (byte)117, (byte)221, (byte)210, (byte)29, (byte)118, (byte)93, (byte)99, (byte)189, (byte)4, (byte)57, (byte)223, (byte)234, (byte)42, (byte)83, (byte)63, (byte)169, (byte)48, (byte)104, (byte)105, (byte)186, (byte)135, (byte)74, (byte)151, (byte)102, (byte)103, (byte)216, (byte)137, (byte)49, (byte)91, (byte)110, (byte)133, (byte)70, (byte)147, (byte)73, (byte)205, (byte)90, (byte)31, (byte)126, (byte)186, (byte)63, (byte)59, (byte)37, (byte)178, (byte)101, (byte)26, (byte)20, (byte)51, (byte)50, (byte)136, (byte)73, (byte)76, (byte)175, (byte)102, (byte)193, (byte)247, (byte)124, (byte)115, (byte)207, (byte)152, (byte)83, (byte)222, (byte)104, (byte)20, (byte)154, (byte)188, (byte)223, (byte)237, (byte)87, (byte)19, (byte)104, (byte)160, (byte)64, (byte)159, (byte)180, (byte)219, (byte)111, (byte)54, (byte)72, (byte)226, (byte)190, (byte)241, (byte)160, (byte)94, (byte)59, (byte)11, (byte)245, (byte)26, (byte)227, (byte)4, (byte)173, (byte)39, (byte)33, (byte)49, (byte)118, (byte)46, (byte)235, (byte)245, (byte)204, (byte)48, (byte)155, (byte)114, (byte)30, (byte)243, (byte)66, (byte)13, (byte)11, (byte)50, (byte)93, (byte)250, (byte)25, (byte)162, (byte)209, (byte)12, (byte)164, (byte)67, (byte)93, (byte)175, (byte)141, (byte)186, (byte)47, (byte)14, (byte)193, (byte)23, (byte)83, (byte)7, (byte)22, (byte)158, (byte)96, (byte)116, (byte)130, (byte)33, (byte)3, (byte)122, (byte)163, (byte)211, (byte)191, (byte)220, (byte)58, (byte)218, (byte)232, (byte)68, (byte)90, (byte)25, (byte)181, (byte)217, (byte)79, (byte)107, (byte)38, (byte)254, (byte)28, (byte)173, (byte)87, (byte)243, (byte)91, (byte)31, (byte)71, (byte)83, (byte)149, (byte)49, (byte)198, (byte)160, (byte)252, (byte)53, (byte)59, (byte)148, (byte)47, (byte)61, (byte)45, (byte)207, (byte)82, (byte)89, (byte)23, (byte)63, (byte)169, (byte)93, (byte)16, (byte)43, (byte)47, (byte)115, (byte)17, (byte)103, (byte)42, (byte)117, (byte)39, (byte)234, (byte)100, (byte)227, (byte)159, (byte)141, (byte)79, (byte)71, (byte)223, (byte)40, (byte)246, (byte)208, (byte)49, (byte)140, (byte)137, (byte)122, (byte)118, (byte)137, (byte)245, (byte)124, (byte)55, (byte)30, (byte)158, (byte)234, (byte)133, (byte)119, (byte)161, (byte)12, (byte)153, (byte)133, (byte)79, (byte)129, (byte)183, (byte)26, (byte)34, (byte)250, (byte)3, (byte)165, (byte)153, (byte)94, (byte)232, (byte)234, (byte)216, (byte)109, (byte)17, (byte)94, (byte)128, (byte)171, (byte)193, (byte)4, (byte)66, (byte)198, (byte)65, (byte)69, (byte)23, (byte)57, (byte)82, (byte)197, (byte)62, (byte)50, (byte)15, (byte)101, (byte)239, (byte)60, (byte)141}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)23794);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)236, (byte)176, (byte)117, (byte)50, (byte)94, (byte)117, (byte)221, (byte)210, (byte)29, (byte)118, (byte)93, (byte)99, (byte)189, (byte)4, (byte)57, (byte)223, (byte)234, (byte)42, (byte)83, (byte)63, (byte)169, (byte)48, (byte)104, (byte)105, (byte)186, (byte)135, (byte)74, (byte)151, (byte)102, (byte)103, (byte)216, (byte)137, (byte)49, (byte)91, (byte)110, (byte)133, (byte)70, (byte)147, (byte)73, (byte)205, (byte)90, (byte)31, (byte)126, (byte)186, (byte)63, (byte)59, (byte)37, (byte)178, (byte)101, (byte)26, (byte)20, (byte)51, (byte)50, (byte)136, (byte)73, (byte)76, (byte)175, (byte)102, (byte)193, (byte)247, (byte)124, (byte)115, (byte)207, (byte)152, (byte)83, (byte)222, (byte)104, (byte)20, (byte)154, (byte)188, (byte)223, (byte)237, (byte)87, (byte)19, (byte)104, (byte)160, (byte)64, (byte)159, (byte)180, (byte)219, (byte)111, (byte)54, (byte)72, (byte)226, (byte)190, (byte)241, (byte)160, (byte)94, (byte)59, (byte)11, (byte)245, (byte)26, (byte)227, (byte)4, (byte)173, (byte)39, (byte)33, (byte)49, (byte)118, (byte)46, (byte)235, (byte)245, (byte)204, (byte)48, (byte)155, (byte)114, (byte)30, (byte)243, (byte)66, (byte)13, (byte)11, (byte)50, (byte)93, (byte)250, (byte)25, (byte)162, (byte)209, (byte)12, (byte)164, (byte)67, (byte)93, (byte)175, (byte)141, (byte)186, (byte)47, (byte)14, (byte)193, (byte)23, (byte)83, (byte)7, (byte)22, (byte)158, (byte)96, (byte)116, (byte)130, (byte)33, (byte)3, (byte)122, (byte)163, (byte)211, (byte)191, (byte)220, (byte)58, (byte)218, (byte)232, (byte)68, (byte)90, (byte)25, (byte)181, (byte)217, (byte)79, (byte)107, (byte)38, (byte)254, (byte)28, (byte)173, (byte)87, (byte)243, (byte)91, (byte)31, (byte)71, (byte)83, (byte)149, (byte)49, (byte)198, (byte)160, (byte)252, (byte)53, (byte)59, (byte)148, (byte)47, (byte)61, (byte)45, (byte)207, (byte)82, (byte)89, (byte)23, (byte)63, (byte)169, (byte)93, (byte)16, (byte)43, (byte)47, (byte)115, (byte)17, (byte)103, (byte)42, (byte)117, (byte)39, (byte)234, (byte)100, (byte)227, (byte)159, (byte)141, (byte)79, (byte)71, (byte)223, (byte)40, (byte)246, (byte)208, (byte)49, (byte)140, (byte)137, (byte)122, (byte)118, (byte)137, (byte)245, (byte)124, (byte)55, (byte)30, (byte)158, (byte)234, (byte)133, (byte)119, (byte)161, (byte)12, (byte)153, (byte)133, (byte)79, (byte)129, (byte)183, (byte)26, (byte)34, (byte)250, (byte)3, (byte)165, (byte)153, (byte)94, (byte)232, (byte)234, (byte)216, (byte)109, (byte)17, (byte)94, (byte)128, (byte)171, (byte)193, (byte)4, (byte)66, (byte)198, (byte)65, (byte)69, (byte)23, (byte)57, (byte)82, (byte)197, (byte)62, (byte)50, (byte)15, (byte)101, (byte)239, (byte)60, (byte)141}, 0) ;
            p131.seqnr = (ushort)(ushort)23794;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)169);
                Debug.Assert(pack.current_distance == (ushort)(ushort)55652);
                Debug.Assert(pack.min_distance == (ushort)(ushort)10216);
                Debug.Assert(pack.id == (byte)(byte)96);
                Debug.Assert(pack.max_distance == (ushort)(ushort)22522);
                Debug.Assert(pack.time_boot_ms == (uint)2732478076U);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)10216;
            p132.current_distance = (ushort)(ushort)55652;
            p132.id = (byte)(byte)96;
            p132.covariance = (byte)(byte)169;
            p132.max_distance = (ushort)(ushort)22522;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45;
            p132.time_boot_ms = (uint)2732478076U;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)998365194);
                Debug.Assert(pack.mask == (ulong)5086679194545801542L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)13099);
                Debug.Assert(pack.lat == (int)921558125);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int)998365194;
            p133.lat = (int)921558125;
            p133.grid_spacing = (ushort)(ushort)13099;
            p133.mask = (ulong)5086679194545801542L;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1814018305);
                Debug.Assert(pack.gridbit == (byte)(byte)36);
                Debug.Assert(pack.lon == (int) -129791683);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)5621, (short) -28196, (short)32062, (short)8666, (short) -12159, (short) -17841, (short)27927, (short)20776, (short)21967, (short)14573, (short) -1453, (short) -9900, (short)3348, (short)156, (short)27668, (short) -6688}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)59597);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1814018305;
            p134.lon = (int) -129791683;
            p134.grid_spacing = (ushort)(ushort)59597;
            p134.data__SET(new short[] {(short)5621, (short) -28196, (short)32062, (short)8666, (short) -12159, (short) -17841, (short)27927, (short)20776, (short)21967, (short)14573, (short) -1453, (short) -9900, (short)3348, (short)156, (short)27668, (short) -6688}, 0) ;
            p134.gridbit = (byte)(byte)36;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)689778867);
                Debug.Assert(pack.lon == (int) -12994152);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)689778867;
            p135.lon = (int) -12994152;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.loaded == (ushort)(ushort)36615);
                Debug.Assert(pack.lat == (int)400491082);
                Debug.Assert(pack.lon == (int) -1173179928);
                Debug.Assert(pack.pending == (ushort)(ushort)59549);
                Debug.Assert(pack.terrain_height == (float) -3.0206749E38F);
                Debug.Assert(pack.current_height == (float) -1.7874232E37F);
                Debug.Assert(pack.spacing == (ushort)(ushort)61781);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.terrain_height = (float) -3.0206749E38F;
            p136.lat = (int)400491082;
            p136.loaded = (ushort)(ushort)36615;
            p136.lon = (int) -1173179928;
            p136.current_height = (float) -1.7874232E37F;
            p136.spacing = (ushort)(ushort)61781;
            p136.pending = (ushort)(ushort)59549;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -1.0442628E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3566724736U);
                Debug.Assert(pack.press_abs == (float)1.3828608E37F);
                Debug.Assert(pack.temperature == (short)(short) -5995);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float) -1.0442628E38F;
            p137.temperature = (short)(short) -5995;
            p137.press_abs = (float)1.3828608E37F;
            p137.time_boot_ms = (uint)3566724736U;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.363773E38F, -1.8524635E38F, 7.9812113E37F, -7.637003E36F}));
                Debug.Assert(pack.x == (float) -1.6884683E38F);
                Debug.Assert(pack.z == (float)3.390022E38F);
                Debug.Assert(pack.y == (float) -1.6651312E38F);
                Debug.Assert(pack.time_usec == (ulong)6649546081883756675L);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {1.363773E38F, -1.8524635E38F, 7.9812113E37F, -7.637003E36F}, 0) ;
            p138.y = (float) -1.6651312E38F;
            p138.z = (float)3.390022E38F;
            p138.x = (float) -1.6884683E38F;
            p138.time_usec = (ulong)6649546081883756675L;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.target_component == (byte)(byte)66);
                Debug.Assert(pack.time_usec == (ulong)3363773704128474013L);
                Debug.Assert(pack.group_mlx == (byte)(byte)125);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2581692E38F, -2.573072E38F, 1.1255356E38F, -2.8956991E38F, -3.0584851E38F, 1.5148596E38F, 2.285497E38F, -1.552464E38F}));
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)125;
            p139.time_usec = (ulong)3363773704128474013L;
            p139.controls_SET(new float[] {-1.2581692E38F, -2.573072E38F, 1.1255356E38F, -2.8956991E38F, -3.0584851E38F, 1.5148596E38F, 2.285497E38F, -1.552464E38F}, 0) ;
            p139.target_component = (byte)(byte)66;
            p139.target_system = (byte)(byte)242;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)38);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.070977E38F, 2.0756915E38F, 1.9283255E37F, -2.0522457E37F, -1.6874994E38F, -1.2718752E38F, -2.6433612E38F, 2.5886815E38F}));
                Debug.Assert(pack.time_usec == (ulong)1585685812183327928L);
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1585685812183327928L;
            p140.controls_SET(new float[] {3.070977E38F, 2.0756915E38F, 1.9283255E37F, -2.0522457E37F, -1.6874994E38F, -1.2718752E38F, -2.6433612E38F, 2.5886815E38F}, 0) ;
            p140.group_mlx = (byte)(byte)38;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7923523582766381436L);
                Debug.Assert(pack.altitude_terrain == (float)1.7562413E38F);
                Debug.Assert(pack.altitude_local == (float)9.7386605E36F);
                Debug.Assert(pack.bottom_clearance == (float)1.4415823E37F);
                Debug.Assert(pack.altitude_amsl == (float) -1.2184207E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.4873804E38F);
                Debug.Assert(pack.altitude_relative == (float) -2.1281554E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float) -1.2184207E38F;
            p141.time_usec = (ulong)7923523582766381436L;
            p141.altitude_local = (float)9.7386605E36F;
            p141.bottom_clearance = (float)1.4415823E37F;
            p141.altitude_relative = (float) -2.1281554E38F;
            p141.altitude_monotonic = (float) -2.4873804E38F;
            p141.altitude_terrain = (float)1.7562413E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)193, (byte)91, (byte)24, (byte)126, (byte)94, (byte)175, (byte)174, (byte)73, (byte)0, (byte)180, (byte)216, (byte)68, (byte)63, (byte)158, (byte)136, (byte)196, (byte)225, (byte)138, (byte)203, (byte)197, (byte)59, (byte)234, (byte)236, (byte)84, (byte)218, (byte)60, (byte)43, (byte)82, (byte)79, (byte)81, (byte)209, (byte)12, (byte)54, (byte)200, (byte)8, (byte)217, (byte)18, (byte)230, (byte)29, (byte)227, (byte)116, (byte)135, (byte)80, (byte)195, (byte)109, (byte)167, (byte)89, (byte)203, (byte)198, (byte)250, (byte)92, (byte)111, (byte)101, (byte)90, (byte)180, (byte)54, (byte)108, (byte)47, (byte)74, (byte)254, (byte)242, (byte)66, (byte)15, (byte)26, (byte)121, (byte)233, (byte)0, (byte)218, (byte)251, (byte)149, (byte)130, (byte)176, (byte)157, (byte)92, (byte)110, (byte)17, (byte)1, (byte)73, (byte)130, (byte)55, (byte)158, (byte)231, (byte)210, (byte)60, (byte)150, (byte)214, (byte)147, (byte)134, (byte)8, (byte)95, (byte)54, (byte)128, (byte)210, (byte)139, (byte)14, (byte)190, (byte)44, (byte)129, (byte)48, (byte)233, (byte)137, (byte)174, (byte)159, (byte)77, (byte)47, (byte)229, (byte)110, (byte)91, (byte)54, (byte)38, (byte)119, (byte)43, (byte)52, (byte)47, (byte)165, (byte)248, (byte)0, (byte)7, (byte)237, (byte)211}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)112, (byte)203, (byte)187, (byte)131, (byte)127, (byte)230, (byte)134, (byte)118, (byte)203, (byte)112, (byte)5, (byte)163, (byte)62, (byte)252, (byte)22, (byte)15, (byte)56, (byte)57, (byte)84, (byte)243, (byte)37, (byte)253, (byte)145, (byte)253, (byte)226, (byte)52, (byte)105, (byte)139, (byte)220, (byte)126, (byte)71, (byte)18, (byte)127, (byte)243, (byte)55, (byte)41, (byte)142, (byte)36, (byte)54, (byte)158, (byte)171, (byte)200, (byte)76, (byte)41, (byte)181, (byte)239, (byte)253, (byte)74, (byte)178, (byte)86, (byte)77, (byte)184, (byte)38, (byte)141, (byte)135, (byte)74, (byte)254, (byte)158, (byte)135, (byte)196, (byte)173, (byte)196, (byte)254, (byte)47, (byte)180, (byte)123, (byte)172, (byte)51, (byte)205, (byte)3, (byte)175, (byte)47, (byte)209, (byte)11, (byte)2, (byte)52, (byte)227, (byte)29, (byte)186, (byte)77, (byte)192, (byte)210, (byte)209, (byte)185, (byte)103, (byte)120, (byte)65, (byte)182, (byte)211, (byte)153, (byte)126, (byte)168, (byte)26, (byte)32, (byte)122, (byte)72, (byte)212, (byte)219, (byte)186, (byte)155, (byte)99, (byte)232, (byte)229, (byte)152, (byte)231, (byte)151, (byte)96, (byte)173, (byte)140, (byte)152, (byte)123, (byte)168, (byte)62, (byte)104, (byte)127, (byte)132, (byte)129, (byte)222, (byte)243, (byte)125}));
                Debug.Assert(pack.request_id == (byte)(byte)114);
                Debug.Assert(pack.transfer_type == (byte)(byte)241);
                Debug.Assert(pack.uri_type == (byte)(byte)91);
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)241;
            p142.storage_SET(new byte[] {(byte)112, (byte)203, (byte)187, (byte)131, (byte)127, (byte)230, (byte)134, (byte)118, (byte)203, (byte)112, (byte)5, (byte)163, (byte)62, (byte)252, (byte)22, (byte)15, (byte)56, (byte)57, (byte)84, (byte)243, (byte)37, (byte)253, (byte)145, (byte)253, (byte)226, (byte)52, (byte)105, (byte)139, (byte)220, (byte)126, (byte)71, (byte)18, (byte)127, (byte)243, (byte)55, (byte)41, (byte)142, (byte)36, (byte)54, (byte)158, (byte)171, (byte)200, (byte)76, (byte)41, (byte)181, (byte)239, (byte)253, (byte)74, (byte)178, (byte)86, (byte)77, (byte)184, (byte)38, (byte)141, (byte)135, (byte)74, (byte)254, (byte)158, (byte)135, (byte)196, (byte)173, (byte)196, (byte)254, (byte)47, (byte)180, (byte)123, (byte)172, (byte)51, (byte)205, (byte)3, (byte)175, (byte)47, (byte)209, (byte)11, (byte)2, (byte)52, (byte)227, (byte)29, (byte)186, (byte)77, (byte)192, (byte)210, (byte)209, (byte)185, (byte)103, (byte)120, (byte)65, (byte)182, (byte)211, (byte)153, (byte)126, (byte)168, (byte)26, (byte)32, (byte)122, (byte)72, (byte)212, (byte)219, (byte)186, (byte)155, (byte)99, (byte)232, (byte)229, (byte)152, (byte)231, (byte)151, (byte)96, (byte)173, (byte)140, (byte)152, (byte)123, (byte)168, (byte)62, (byte)104, (byte)127, (byte)132, (byte)129, (byte)222, (byte)243, (byte)125}, 0) ;
            p142.uri_type = (byte)(byte)91;
            p142.request_id = (byte)(byte)114;
            p142.uri_SET(new byte[] {(byte)193, (byte)91, (byte)24, (byte)126, (byte)94, (byte)175, (byte)174, (byte)73, (byte)0, (byte)180, (byte)216, (byte)68, (byte)63, (byte)158, (byte)136, (byte)196, (byte)225, (byte)138, (byte)203, (byte)197, (byte)59, (byte)234, (byte)236, (byte)84, (byte)218, (byte)60, (byte)43, (byte)82, (byte)79, (byte)81, (byte)209, (byte)12, (byte)54, (byte)200, (byte)8, (byte)217, (byte)18, (byte)230, (byte)29, (byte)227, (byte)116, (byte)135, (byte)80, (byte)195, (byte)109, (byte)167, (byte)89, (byte)203, (byte)198, (byte)250, (byte)92, (byte)111, (byte)101, (byte)90, (byte)180, (byte)54, (byte)108, (byte)47, (byte)74, (byte)254, (byte)242, (byte)66, (byte)15, (byte)26, (byte)121, (byte)233, (byte)0, (byte)218, (byte)251, (byte)149, (byte)130, (byte)176, (byte)157, (byte)92, (byte)110, (byte)17, (byte)1, (byte)73, (byte)130, (byte)55, (byte)158, (byte)231, (byte)210, (byte)60, (byte)150, (byte)214, (byte)147, (byte)134, (byte)8, (byte)95, (byte)54, (byte)128, (byte)210, (byte)139, (byte)14, (byte)190, (byte)44, (byte)129, (byte)48, (byte)233, (byte)137, (byte)174, (byte)159, (byte)77, (byte)47, (byte)229, (byte)110, (byte)91, (byte)54, (byte)38, (byte)119, (byte)43, (byte)52, (byte)47, (byte)165, (byte)248, (byte)0, (byte)7, (byte)237, (byte)211}, 0) ;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)27035);
                Debug.Assert(pack.press_diff == (float)2.7354133E38F);
                Debug.Assert(pack.press_abs == (float) -2.286909E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1579210496U);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short)27035;
            p143.press_diff = (float)2.7354133E38F;
            p143.time_boot_ms = (uint)1579210496U;
            p143.press_abs = (float) -2.286909E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_state == (ulong)5079726945395383448L);
                Debug.Assert(pack.lon == (int)1679170181);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.4292745E38F, 2.594668E38F, 2.1745224E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)49);
                Debug.Assert(pack.alt == (float)1.8318243E38F);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {8.5048215E37F, 4.3806085E37F, -3.007245E38F}));
                Debug.Assert(pack.lat == (int)1324350860);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-2.621419E38F, -2.2883193E38F, -2.6574402E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.2578763E38F, -1.408758E38F, -4.358691E37F}));
                Debug.Assert(pack.timestamp == (ulong)834932896261939508L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {3.1209284E37F, -1.4938582E38F, -2.056886E38F, -2.6884127E38F}));
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)834932896261939508L;
            p144.est_capabilities = (byte)(byte)49;
            p144.position_cov_SET(new float[] {-2.4292745E38F, 2.594668E38F, 2.1745224E38F}, 0) ;
            p144.custom_state = (ulong)5079726945395383448L;
            p144.attitude_q_SET(new float[] {3.1209284E37F, -1.4938582E38F, -2.056886E38F, -2.6884127E38F}, 0) ;
            p144.lon = (int)1679170181;
            p144.alt = (float)1.8318243E38F;
            p144.vel_SET(new float[] {8.5048215E37F, 4.3806085E37F, -3.007245E38F}, 0) ;
            p144.lat = (int)1324350860;
            p144.rates_SET(new float[] {-2.621419E38F, -2.2883193E38F, -2.6574402E38F}, 0) ;
            p144.acc_SET(new float[] {1.2578763E38F, -1.408758E38F, -4.358691E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8569469838541732620L);
                Debug.Assert(pack.x_vel == (float) -2.6030552E38F);
                Debug.Assert(pack.airspeed == (float)3.2603634E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.7247983E38F, 1.5103471E38F, -1.1274527E38F}));
                Debug.Assert(pack.z_pos == (float) -2.4425066E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3740642E38F, -2.7149997E38F, -1.971153E38F, -1.0386921E38F}));
                Debug.Assert(pack.y_vel == (float)1.9912055E38F);
                Debug.Assert(pack.roll_rate == (float) -2.3728578E38F);
                Debug.Assert(pack.z_vel == (float)3.3905838E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.905263E38F, -7.0660127E37F, -2.390936E38F}));
                Debug.Assert(pack.x_acc == (float) -8.739564E37F);
                Debug.Assert(pack.y_pos == (float) -2.261976E38F);
                Debug.Assert(pack.z_acc == (float) -2.2005248E37F);
                Debug.Assert(pack.x_pos == (float)3.3917107E38F);
                Debug.Assert(pack.pitch_rate == (float) -1.8496677E38F);
                Debug.Assert(pack.yaw_rate == (float)1.0228817E38F);
                Debug.Assert(pack.y_acc == (float)9.101815E37F);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pos_variance_SET(new float[] {1.905263E38F, -7.0660127E37F, -2.390936E38F}, 0) ;
            p146.y_pos = (float) -2.261976E38F;
            p146.z_acc = (float) -2.2005248E37F;
            p146.yaw_rate = (float)1.0228817E38F;
            p146.z_pos = (float) -2.4425066E37F;
            p146.y_acc = (float)9.101815E37F;
            p146.x_pos = (float)3.3917107E38F;
            p146.x_vel = (float) -2.6030552E38F;
            p146.pitch_rate = (float) -1.8496677E38F;
            p146.z_vel = (float)3.3905838E38F;
            p146.time_usec = (ulong)8569469838541732620L;
            p146.roll_rate = (float) -2.3728578E38F;
            p146.q_SET(new float[] {-2.3740642E38F, -2.7149997E38F, -1.971153E38F, -1.0386921E38F}, 0) ;
            p146.vel_variance_SET(new float[] {1.7247983E38F, 1.5103471E38F, -1.1274527E38F}, 0) ;
            p146.y_vel = (float)1.9912055E38F;
            p146.x_acc = (float) -8.739564E37F;
            p146.airspeed = (float)3.2603634E37F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)86);
                Debug.Assert(pack.id == (byte)(byte)219);
                Debug.Assert(pack.current_consumed == (int)1999778172);
                Debug.Assert(pack.energy_consumed == (int) -136413299);
                Debug.Assert(pack.current_battery == (short)(short) -30227);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.temperature == (short)(short) -703);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)37933, (ushort)56286, (ushort)35359, (ushort)5358, (ushort)9323, (ushort)3827, (ushort)56266, (ushort)37121, (ushort)43499, (ushort)40489}));
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_remaining = (sbyte)(sbyte)86;
            p147.temperature = (short)(short) -703;
            p147.current_consumed = (int)1999778172;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.energy_consumed = (int) -136413299;
            p147.voltages_SET(new ushort[] {(ushort)37933, (ushort)56286, (ushort)35359, (ushort)5358, (ushort)9323, (ushort)3827, (ushort)56266, (ushort)37121, (ushort)43499, (ushort)40489}, 0) ;
            p147.id = (byte)(byte)219;
            p147.current_battery = (short)(short) -30227;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_sw_version == (uint)2852867659U);
                Debug.Assert(pack.flight_sw_version == (uint)1293635392U);
                Debug.Assert(pack.product_id == (ushort)(ushort)3773);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)204, (byte)12, (byte)27, (byte)43, (byte)67, (byte)3, (byte)167, (byte)75}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)96, (byte)148, (byte)42, (byte)57, (byte)3, (byte)115, (byte)234, (byte)237}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)4, (byte)46, (byte)9, (byte)60, (byte)133, (byte)107, (byte)103, (byte)44}));
                Debug.Assert(pack.uid == (ulong)1748948933527314600L);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)21809);
                Debug.Assert(pack.board_version == (uint)3257192106U);
                Debug.Assert(pack.middleware_sw_version == (uint)1655262042U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)241, (byte)220, (byte)100, (byte)30, (byte)170, (byte)80, (byte)133, (byte)193, (byte)26, (byte)93, (byte)156, (byte)236, (byte)53, (byte)157, (byte)56, (byte)3, (byte)120, (byte)101}));
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.board_version = (uint)3257192106U;
            p148.flight_sw_version = (uint)1293635392U;
            p148.middleware_custom_version_SET(new byte[] {(byte)96, (byte)148, (byte)42, (byte)57, (byte)3, (byte)115, (byte)234, (byte)237}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)4, (byte)46, (byte)9, (byte)60, (byte)133, (byte)107, (byte)103, (byte)44}, 0) ;
            p148.middleware_sw_version = (uint)1655262042U;
            p148.uid2_SET(new byte[] {(byte)241, (byte)220, (byte)100, (byte)30, (byte)170, (byte)80, (byte)133, (byte)193, (byte)26, (byte)93, (byte)156, (byte)236, (byte)53, (byte)157, (byte)56, (byte)3, (byte)120, (byte)101}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)204, (byte)12, (byte)27, (byte)43, (byte)67, (byte)3, (byte)167, (byte)75}, 0) ;
            p148.uid = (ulong)1748948933527314600L;
            p148.vendor_id = (ushort)(ushort)21809;
            p148.os_sw_version = (uint)2852867659U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
            p148.product_id = (ushort)(ushort)3773;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.8921211E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.3610777E38F);
                Debug.Assert(pack.size_x == (float)1.5673687E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.3947275E38F);
                Debug.Assert(pack.target_num == (byte)(byte)225);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.3907487E38F, 2.5683692E38F, -2.5631104E38F, -3.3293032E38F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)162);
                Debug.Assert(pack.angle_x == (float)8.393721E37F);
                Debug.Assert(pack.time_usec == (ulong)8022288352058108164L);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.distance == (float)1.1599811E38F);
                Debug.Assert(pack.angle_y == (float)8.654711E37F);
                Debug.Assert(pack.size_y == (float) -1.8897229E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)8022288352058108164L;
            p149.q_SET(new float[] {2.3907487E38F, 2.5683692E38F, -2.5631104E38F, -3.3293032E38F}, 0, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p149.size_x = (float)1.5673687E38F;
            p149.size_y = (float) -1.8897229E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)162, PH) ;
            p149.x_SET((float) -2.3947275E38F, PH) ;
            p149.angle_x = (float)8.393721E37F;
            p149.y_SET((float) -2.8921211E38F, PH) ;
            p149.angle_y = (float)8.654711E37F;
            p149.z_SET((float)1.3610777E38F, PH) ;
            p149.target_num = (byte)(byte)225;
            p149.distance = (float)1.1599811E38F;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_0Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte)96, (sbyte) - 24, (sbyte)95, (sbyte)10}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)32167, (ushort)7385, (ushort)6364, (ushort)27053}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2982030693U, 2443135946U, 3210940519U, 3527759376U}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)143, (byte)153, (byte)114, (byte)198}));
                Debug.Assert(pack.v1 == (byte)(byte)117);
            };
            DemoDevice.ARRAY_TEST_0 p150 = LoopBackDemoChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.ar_i8_SET(new sbyte[] {(sbyte)96, (sbyte) - 24, (sbyte)95, (sbyte)10}, 0) ;
            p150.ar_u32_SET(new uint[] {2982030693U, 2443135946U, 3210940519U, 3527759376U}, 0) ;
            p150.v1 = (byte)(byte)117;
            p150.ar_u16_SET(new ushort[] {(ushort)32167, (ushort)7385, (ushort)6364, (ushort)27053}, 0) ;
            p150.ar_u8_SET(new byte[] {(byte)143, (byte)153, (byte)114, (byte)198}, 0) ;
            LoopBackDemoChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_1Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2014269222U, 2900627641U, 1143040360U, 2322886110U}));
            };
            DemoDevice.ARRAY_TEST_1 p151 = LoopBackDemoChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[] {2014269222U, 2900627641U, 1143040360U, 2322886110U}, 0) ;
            LoopBackDemoChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {3406306976U, 3062688544U, 2571163245U, 417808365U}));
                Debug.Assert(pack.v == (byte)(byte)149);
            };
            DemoDevice.ARRAY_TEST_3 p153 = LoopBackDemoChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)149;
            p153.ar_u32_SET(new uint[] {3406306976U, 3062688544U, 2571163245U, 417808365U}, 0) ;
            LoopBackDemoChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {96587388U, 2545299066U, 1913142698U, 1833241932U}));
                Debug.Assert(pack.v == (byte)(byte)180);
            };
            DemoDevice.ARRAY_TEST_4 p154 = LoopBackDemoChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.v = (byte)(byte)180;
            p154.ar_u32_SET(new uint[] {96587388U, 2545299066U, 1913142698U, 1833241932U}, 0) ;
            LoopBackDemoChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.c2_LEN(ph) == 4);
                Debug.Assert(pack.c2_TRY(ph).Equals("ilrz"));
                Debug.Assert(pack.c1_LEN(ph) == 4);
                Debug.Assert(pack.c1_TRY(ph).Equals("ofdo"));
            };
            DemoDevice.ARRAY_TEST_5 p155 = LoopBackDemoChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("ofdo", PH) ;
            p155.c2_SET("ilrz", PH) ;
            LoopBackDemoChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_c_LEN(ph) == 12);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("kyypnmlvjpDg"));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)7782, (ushort)60742}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {2.2690297E38F, 3.20883E38F}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1934166535U, 848410223U}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 80, (sbyte) - 83}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-8.449359610104322E307, 4.005729695765801E307}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)51, (byte)80}));
                Debug.Assert(pack.v2 == (ushort)(ushort)76);
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -3174, (short)18943}));
                Debug.Assert(pack.v3 == (uint)1262896147U);
                Debug.Assert(pack.v1 == (byte)(byte)41);
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {329034727, 1998491448}));
            };
            DemoDevice.ARRAY_TEST_6 p156 = LoopBackDemoChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.ar_d_SET(new double[] {-8.449359610104322E307, 4.005729695765801E307}, 0) ;
            p156.ar_i32_SET(new int[] {329034727, 1998491448}, 0) ;
            p156.ar_u16_SET(new ushort[] {(ushort)7782, (ushort)60742}, 0) ;
            p156.ar_i8_SET(new sbyte[] {(sbyte) - 80, (sbyte) - 83}, 0) ;
            p156.ar_f_SET(new float[] {2.2690297E38F, 3.20883E38F}, 0) ;
            p156.ar_i16_SET(new short[] {(short) -3174, (short)18943}, 0) ;
            p156.v1 = (byte)(byte)41;
            p156.ar_c_SET("kyypnmlvjpDg", PH) ;
            p156.v2 = (ushort)(ushort)76;
            p156.ar_u32_SET(new uint[] {1934166535U, 848410223U}, 0) ;
            p156.ar_u8_SET(new byte[] {(byte)51, (byte)80}, 0) ;
            p156.v3 = (uint)1262896147U;
            LoopBackDemoChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 64, (sbyte) - 66}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {2.4111542E38F, 2.994886E38F}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)1943, (ushort)37769}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)164, (byte)62}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {1.465506902241631E307, -9.74795403001018E307}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {707450664U, 3265527853U}));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -23738, (short)21068}));
                Debug.Assert(pack.ar_c_LEN(ph) == 32);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("tdepzhgxeuKdfghYnulphdauzlzdnvjq"));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {914409022, 504978179}));
            };
            DemoDevice.ARRAY_TEST_7 p157 = LoopBackDemoChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_i32_SET(new int[] {914409022, 504978179}, 0) ;
            p157.ar_u32_SET(new uint[] {707450664U, 3265527853U}, 0) ;
            p157.ar_d_SET(new double[] {1.465506902241631E307, -9.74795403001018E307}, 0) ;
            p157.ar_u8_SET(new byte[] {(byte)164, (byte)62}, 0) ;
            p157.ar_i16_SET(new short[] {(short) -23738, (short)21068}, 0) ;
            p157.ar_u16_SET(new ushort[] {(ushort)1943, (ushort)37769}, 0) ;
            p157.ar_f_SET(new float[] {2.4111542E38F, 2.994886E38F}, 0) ;
            p157.ar_i8_SET(new sbyte[] {(sbyte) - 64, (sbyte) - 66}, 0) ;
            p157.ar_c_SET("tdepzhgxeuKdfghYnulphdauzlzdnvjq", PH) ;
            LoopBackDemoChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnARRAY_TEST_8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {1.0768730482552174E308, 1.1629694596550983E308}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)6746, (ushort)63049}));
                Debug.Assert(pack.v3 == (uint)4045948732U);
            };
            DemoDevice.ARRAY_TEST_8 p158 = LoopBackDemoChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.ar_d_SET(new double[] {1.0768730482552174E308, 1.1629694596550983E308}, 0) ;
            p158.ar_u16_SET(new ushort[] {(ushort)6746, (ushort)63049}, 0) ;
            p158.v3 = (uint)4045948732U;
            LoopBackDemoChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float)1.4864248E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.3918325E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.805094E38F);
                Debug.Assert(pack.mag_ratio == (float) -2.4081569E38F);
                Debug.Assert(pack.time_usec == (ulong)6271204669543615241L);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -5.549911E37F);
                Debug.Assert(pack.tas_ratio == (float) -3.3564145E38F);
                Debug.Assert(pack.hagl_ratio == (float)1.9198061E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -9.327458E37F);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_ratio = (float) -9.327458E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
            p230.time_usec = (ulong)6271204669543615241L;
            p230.pos_vert_accuracy = (float)1.4864248E38F;
            p230.mag_ratio = (float) -2.4081569E38F;
            p230.hagl_ratio = (float)1.9198061E38F;
            p230.tas_ratio = (float) -3.3564145E38F;
            p230.pos_horiz_accuracy = (float) -5.549911E37F;
            p230.pos_vert_ratio = (float) -2.805094E38F;
            p230.vel_ratio = (float) -1.3918325E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_x == (float)3.4149944E37F);
                Debug.Assert(pack.time_usec == (ulong)8607270212122202122L);
                Debug.Assert(pack.var_vert == (float) -1.4599459E38F);
                Debug.Assert(pack.var_horiz == (float)1.9928009E38F);
                Debug.Assert(pack.wind_y == (float) -2.758458E38F);
                Debug.Assert(pack.wind_z == (float) -9.673878E37F);
                Debug.Assert(pack.vert_accuracy == (float) -4.0080972E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.8278952E38F);
                Debug.Assert(pack.wind_alt == (float) -3.171392E38F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float) -1.4599459E38F;
            p231.horiz_accuracy = (float) -2.8278952E38F;
            p231.vert_accuracy = (float) -4.0080972E37F;
            p231.var_horiz = (float)1.9928009E38F;
            p231.wind_y = (float) -2.758458E38F;
            p231.time_usec = (ulong)8607270212122202122L;
            p231.wind_alt = (float) -3.171392E38F;
            p231.wind_z = (float) -9.673878E37F;
            p231.wind_x = (float)3.4149944E37F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float) -1.163148E37F);
                Debug.Assert(pack.vdop == (float)2.62045E38F);
                Debug.Assert(pack.vn == (float) -5.1642457E36F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)226);
                Debug.Assert(pack.fix_type == (byte)(byte)159);
                Debug.Assert(pack.vert_accuracy == (float) -3.2517384E38F);
                Debug.Assert(pack.time_usec == (ulong)4913748042690136646L);
                Debug.Assert(pack.lon == (int) -1832549712);
                Debug.Assert(pack.lat == (int)1965600105);
                Debug.Assert(pack.speed_accuracy == (float) -1.0307919E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)106);
                Debug.Assert(pack.vd == (float) -1.454814E37F);
                Debug.Assert(pack.horiz_accuracy == (float)4.1245357E37F);
                Debug.Assert(pack.time_week_ms == (uint)1443037275U);
                Debug.Assert(pack.hdop == (float) -1.365399E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
                Debug.Assert(pack.time_week == (ushort)(ushort)38526);
                Debug.Assert(pack.alt == (float)2.4578632E38F);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lat = (int)1965600105;
            p232.time_week_ms = (uint)1443037275U;
            p232.lon = (int) -1832549712;
            p232.fix_type = (byte)(byte)159;
            p232.time_usec = (ulong)4913748042690136646L;
            p232.hdop = (float) -1.365399E38F;
            p232.vn = (float) -5.1642457E36F;
            p232.vdop = (float)2.62045E38F;
            p232.vert_accuracy = (float) -3.2517384E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
            p232.speed_accuracy = (float) -1.0307919E38F;
            p232.ve = (float) -1.163148E37F;
            p232.time_week = (ushort)(ushort)38526;
            p232.horiz_accuracy = (float)4.1245357E37F;
            p232.alt = (float)2.4578632E38F;
            p232.vd = (float) -1.454814E37F;
            p232.satellites_visible = (byte)(byte)226;
            p232.gps_id = (byte)(byte)106;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)168);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)27, (byte)227, (byte)126, (byte)39, (byte)120, (byte)230, (byte)2, (byte)68, (byte)197, (byte)85, (byte)150, (byte)5, (byte)214, (byte)38, (byte)221, (byte)253, (byte)193, (byte)160, (byte)221, (byte)195, (byte)195, (byte)88, (byte)62, (byte)243, (byte)56, (byte)80, (byte)140, (byte)57, (byte)157, (byte)212, (byte)35, (byte)228, (byte)51, (byte)110, (byte)130, (byte)9, (byte)127, (byte)52, (byte)44, (byte)240, (byte)182, (byte)184, (byte)65, (byte)22, (byte)150, (byte)59, (byte)56, (byte)229, (byte)44, (byte)156, (byte)21, (byte)183, (byte)80, (byte)63, (byte)246, (byte)52, (byte)65, (byte)201, (byte)141, (byte)127, (byte)139, (byte)73, (byte)42, (byte)93, (byte)105, (byte)89, (byte)154, (byte)212, (byte)68, (byte)148, (byte)75, (byte)47, (byte)202, (byte)178, (byte)247, (byte)12, (byte)110, (byte)34, (byte)8, (byte)203, (byte)159, (byte)33, (byte)48, (byte)207, (byte)149, (byte)67, (byte)57, (byte)127, (byte)189, (byte)121, (byte)140, (byte)48, (byte)235, (byte)66, (byte)30, (byte)165, (byte)96, (byte)203, (byte)83, (byte)43, (byte)185, (byte)29, (byte)84, (byte)72, (byte)54, (byte)160, (byte)42, (byte)64, (byte)245, (byte)82, (byte)38, (byte)81, (byte)42, (byte)253, (byte)183, (byte)143, (byte)37, (byte)185, (byte)39, (byte)108, (byte)20, (byte)98, (byte)187, (byte)215, (byte)227, (byte)101, (byte)45, (byte)155, (byte)236, (byte)215, (byte)9, (byte)73, (byte)242, (byte)80, (byte)215, (byte)87, (byte)146, (byte)226, (byte)175, (byte)197, (byte)230, (byte)138, (byte)26, (byte)5, (byte)235, (byte)221, (byte)49, (byte)50, (byte)173, (byte)228, (byte)102, (byte)201, (byte)6, (byte)13, (byte)89, (byte)3, (byte)166, (byte)223, (byte)28, (byte)37, (byte)27, (byte)107, (byte)209, (byte)201, (byte)168, (byte)181, (byte)115, (byte)107, (byte)156, (byte)206, (byte)77, (byte)185, (byte)33, (byte)220, (byte)36, (byte)55, (byte)251, (byte)86, (byte)84, (byte)36}));
                Debug.Assert(pack.flags == (byte)(byte)221);
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)221;
            p233.len = (byte)(byte)168;
            p233.data__SET(new byte[] {(byte)27, (byte)227, (byte)126, (byte)39, (byte)120, (byte)230, (byte)2, (byte)68, (byte)197, (byte)85, (byte)150, (byte)5, (byte)214, (byte)38, (byte)221, (byte)253, (byte)193, (byte)160, (byte)221, (byte)195, (byte)195, (byte)88, (byte)62, (byte)243, (byte)56, (byte)80, (byte)140, (byte)57, (byte)157, (byte)212, (byte)35, (byte)228, (byte)51, (byte)110, (byte)130, (byte)9, (byte)127, (byte)52, (byte)44, (byte)240, (byte)182, (byte)184, (byte)65, (byte)22, (byte)150, (byte)59, (byte)56, (byte)229, (byte)44, (byte)156, (byte)21, (byte)183, (byte)80, (byte)63, (byte)246, (byte)52, (byte)65, (byte)201, (byte)141, (byte)127, (byte)139, (byte)73, (byte)42, (byte)93, (byte)105, (byte)89, (byte)154, (byte)212, (byte)68, (byte)148, (byte)75, (byte)47, (byte)202, (byte)178, (byte)247, (byte)12, (byte)110, (byte)34, (byte)8, (byte)203, (byte)159, (byte)33, (byte)48, (byte)207, (byte)149, (byte)67, (byte)57, (byte)127, (byte)189, (byte)121, (byte)140, (byte)48, (byte)235, (byte)66, (byte)30, (byte)165, (byte)96, (byte)203, (byte)83, (byte)43, (byte)185, (byte)29, (byte)84, (byte)72, (byte)54, (byte)160, (byte)42, (byte)64, (byte)245, (byte)82, (byte)38, (byte)81, (byte)42, (byte)253, (byte)183, (byte)143, (byte)37, (byte)185, (byte)39, (byte)108, (byte)20, (byte)98, (byte)187, (byte)215, (byte)227, (byte)101, (byte)45, (byte)155, (byte)236, (byte)215, (byte)9, (byte)73, (byte)242, (byte)80, (byte)215, (byte)87, (byte)146, (byte)226, (byte)175, (byte)197, (byte)230, (byte)138, (byte)26, (byte)5, (byte)235, (byte)221, (byte)49, (byte)50, (byte)173, (byte)228, (byte)102, (byte)201, (byte)6, (byte)13, (byte)89, (byte)3, (byte)166, (byte)223, (byte)28, (byte)37, (byte)27, (byte)107, (byte)209, (byte)201, (byte)168, (byte)181, (byte)115, (byte)107, (byte)156, (byte)206, (byte)77, (byte)185, (byte)33, (byte)220, (byte)36, (byte)55, (byte)251, (byte)86, (byte)84, (byte)36}, 0) ;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (byte)(byte)163);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.airspeed == (byte)(byte)147);
                Debug.Assert(pack.wp_num == (byte)(byte)67);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
                Debug.Assert(pack.altitude_amsl == (short)(short) -16887);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 85);
                Debug.Assert(pack.gps_nsat == (byte)(byte)182);
                Debug.Assert(pack.roll == (short)(short) -14053);
                Debug.Assert(pack.heading == (ushort)(ushort)51835);
                Debug.Assert(pack.custom_mode == (uint)226862264U);
                Debug.Assert(pack.longitude == (int)985276305);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)36);
                Debug.Assert(pack.altitude_sp == (short)(short) -6287);
                Debug.Assert(pack.pitch == (short)(short) -10397);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)68);
                Debug.Assert(pack.failsafe == (byte)(byte)133);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)9256);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 43);
                Debug.Assert(pack.heading_sp == (short)(short)8927);
                Debug.Assert(pack.battery_remaining == (byte)(byte)168);
                Debug.Assert(pack.latitude == (int)379787718);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)83);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.altitude_amsl = (short)(short) -16887;
            p234.gps_nsat = (byte)(byte)182;
            p234.pitch = (short)(short) -10397;
            p234.wp_num = (byte)(byte)67;
            p234.airspeed = (byte)(byte)147;
            p234.temperature = (sbyte)(sbyte) - 43;
            p234.battery_remaining = (byte)(byte)168;
            p234.failsafe = (byte)(byte)133;
            p234.latitude = (int)379787718;
            p234.roll = (short)(short) -14053;
            p234.temperature_air = (sbyte)(sbyte)36;
            p234.climb_rate = (sbyte)(sbyte) - 85;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.altitude_sp = (short)(short) -6287;
            p234.longitude = (int)985276305;
            p234.heading = (ushort)(ushort)51835;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.groundspeed = (byte)(byte)163;
            p234.heading_sp = (short)(short)8927;
            p234.throttle = (sbyte)(sbyte)68;
            p234.wp_distance = (ushort)(ushort)9256;
            p234.custom_mode = (uint)226862264U;
            p234.airspeed_sp = (byte)(byte)83;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1140642935298511925L);
                Debug.Assert(pack.clipping_0 == (uint)3257687885U);
                Debug.Assert(pack.vibration_x == (float) -8.754398E37F);
                Debug.Assert(pack.vibration_z == (float) -3.1297517E38F);
                Debug.Assert(pack.vibration_y == (float)1.3426523E38F);
                Debug.Assert(pack.clipping_2 == (uint)2371328680U);
                Debug.Assert(pack.clipping_1 == (uint)4203364354U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_2 = (uint)2371328680U;
            p241.time_usec = (ulong)1140642935298511925L;
            p241.vibration_y = (float)1.3426523E38F;
            p241.clipping_1 = (uint)4203364354U;
            p241.vibration_z = (float) -3.1297517E38F;
            p241.clipping_0 = (uint)3257687885U;
            p241.vibration_x = (float) -8.754398E37F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.5551289E38F, 1.3051155E38F, 2.6226967E38F, -2.289237E38F}));
                Debug.Assert(pack.x == (float) -1.2603677E38F);
                Debug.Assert(pack.y == (float) -7.734522E37F);
                Debug.Assert(pack.approach_x == (float)9.522987E37F);
                Debug.Assert(pack.z == (float) -1.929811E38F);
                Debug.Assert(pack.approach_z == (float) -3.1898488E38F);
                Debug.Assert(pack.longitude == (int) -168558742);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6123420384338166535L);
                Debug.Assert(pack.altitude == (int) -776104053);
                Debug.Assert(pack.approach_y == (float)3.0951681E38F);
                Debug.Assert(pack.latitude == (int) -1637950280);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_y = (float)3.0951681E38F;
            p242.q_SET(new float[] {2.5551289E38F, 1.3051155E38F, 2.6226967E38F, -2.289237E38F}, 0) ;
            p242.time_usec_SET((ulong)6123420384338166535L, PH) ;
            p242.y = (float) -7.734522E37F;
            p242.altitude = (int) -776104053;
            p242.approach_z = (float) -3.1898488E38F;
            p242.approach_x = (float)9.522987E37F;
            p242.latitude = (int) -1637950280;
            p242.x = (float) -1.2603677E38F;
            p242.longitude = (int) -168558742;
            p242.z = (float) -1.929811E38F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.9589836E38F);
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.approach_z == (float)1.2405795E38F);
                Debug.Assert(pack.approach_y == (float) -5.9580643E37F);
                Debug.Assert(pack.x == (float)9.445844E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3757483E38F, -3.205819E38F, 1.4435376E38F, -7.3619397E37F}));
                Debug.Assert(pack.latitude == (int)720639624);
                Debug.Assert(pack.approach_x == (float) -1.3688461E38F);
                Debug.Assert(pack.z == (float)1.7908887E38F);
                Debug.Assert(pack.altitude == (int)938317189);
                Debug.Assert(pack.longitude == (int)1658040021);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7790918991675133059L);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float)1.7908887E38F;
            p243.y = (float)1.9589836E38F;
            p243.latitude = (int)720639624;
            p243.approach_x = (float) -1.3688461E38F;
            p243.x = (float)9.445844E37F;
            p243.target_system = (byte)(byte)137;
            p243.time_usec_SET((ulong)7790918991675133059L, PH) ;
            p243.q_SET(new float[] {3.3757483E38F, -3.205819E38F, 1.4435376E38F, -7.3619397E37F}, 0) ;
            p243.altitude = (int)938317189;
            p243.approach_y = (float) -5.9580643E37F;
            p243.longitude = (int)1658040021;
            p243.approach_z = (float)1.2405795E38F;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1981868354);
                Debug.Assert(pack.message_id == (ushort)(ushort)62452);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)62452;
            p244.interval_us = (int) -1981868354;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (ushort)(ushort)53615);
                Debug.Assert(pack.tslc == (byte)(byte)41);
                Debug.Assert(pack.ICAO_address == (uint)431901431U);
                Debug.Assert(pack.ver_velocity == (short)(short)15493);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
                Debug.Assert(pack.lon == (int) -1409590614);
                Debug.Assert(pack.lat == (int) -83707967);
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("h"));
                Debug.Assert(pack.squawk == (ushort)(ushort)24053);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)44398);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.altitude == (int) -1583298652);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lat = (int) -83707967;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.ver_velocity = (short)(short)15493;
            p246.heading = (ushort)(ushort)53615;
            p246.ICAO_address = (uint)431901431U;
            p246.callsign_SET("h", PH) ;
            p246.tslc = (byte)(byte)41;
            p246.hor_velocity = (ushort)(ushort)44398;
            p246.squawk = (ushort)(ushort)24053;
            p246.altitude = (int) -1583298652;
            p246.lon = (int) -1409590614;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.435337E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.id == (uint)3091591542U);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.61428E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.257507E38F);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.id = (uint)3091591542U;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float) -1.435337E38F;
            p247.altitude_minimum_delta = (float) -2.61428E38F;
            p247.time_to_minimum_delta = (float) -2.257507E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)114);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)64);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)121, (byte)115, (byte)171, (byte)205, (byte)55, (byte)75, (byte)169, (byte)152, (byte)2, (byte)91, (byte)115, (byte)204, (byte)228, (byte)109, (byte)244, (byte)78, (byte)239, (byte)122, (byte)158, (byte)255, (byte)242, (byte)28, (byte)91, (byte)91, (byte)105, (byte)242, (byte)9, (byte)199, (byte)21, (byte)52, (byte)208, (byte)211, (byte)220, (byte)80, (byte)213, (byte)46, (byte)55, (byte)237, (byte)90, (byte)113, (byte)177, (byte)223, (byte)1, (byte)192, (byte)114, (byte)101, (byte)30, (byte)237, (byte)128, (byte)136, (byte)179, (byte)109, (byte)222, (byte)114, (byte)23, (byte)159, (byte)103, (byte)245, (byte)140, (byte)32, (byte)61, (byte)247, (byte)209, (byte)223, (byte)174, (byte)249, (byte)61, (byte)48, (byte)51, (byte)187, (byte)157, (byte)3, (byte)198, (byte)148, (byte)252, (byte)50, (byte)82, (byte)44, (byte)182, (byte)155, (byte)190, (byte)6, (byte)83, (byte)188, (byte)135, (byte)38, (byte)184, (byte)166, (byte)109, (byte)76, (byte)150, (byte)175, (byte)126, (byte)170, (byte)92, (byte)219, (byte)245, (byte)196, (byte)94, (byte)246, (byte)124, (byte)30, (byte)89, (byte)1, (byte)83, (byte)49, (byte)231, (byte)124, (byte)198, (byte)196, (byte)250, (byte)130, (byte)156, (byte)84, (byte)9, (byte)50, (byte)137, (byte)48, (byte)80, (byte)62, (byte)21, (byte)72, (byte)2, (byte)136, (byte)189, (byte)94, (byte)16, (byte)86, (byte)97, (byte)189, (byte)80, (byte)89, (byte)24, (byte)209, (byte)79, (byte)5, (byte)133, (byte)129, (byte)53, (byte)162, (byte)243, (byte)171, (byte)252, (byte)0, (byte)66, (byte)126, (byte)95, (byte)212, (byte)93, (byte)58, (byte)249, (byte)139, (byte)22, (byte)103, (byte)222, (byte)156, (byte)97, (byte)177, (byte)95, (byte)17, (byte)187, (byte)65, (byte)45, (byte)181, (byte)14, (byte)88, (byte)92, (byte)109, (byte)135, (byte)124, (byte)122, (byte)121, (byte)142, (byte)83, (byte)41, (byte)170, (byte)110, (byte)234, (byte)145, (byte)43, (byte)101, (byte)19, (byte)193, (byte)23, (byte)76, (byte)247, (byte)88, (byte)187, (byte)45, (byte)8, (byte)180, (byte)5, (byte)63, (byte)44, (byte)131, (byte)132, (byte)121, (byte)230, (byte)38, (byte)210, (byte)31, (byte)134, (byte)129, (byte)246, (byte)117, (byte)68, (byte)87, (byte)57, (byte)129, (byte)165, (byte)110, (byte)135, (byte)51, (byte)51, (byte)29, (byte)221, (byte)88, (byte)53, (byte)147, (byte)7, (byte)120, (byte)79, (byte)189, (byte)32, (byte)234, (byte)164, (byte)178, (byte)117, (byte)225, (byte)224, (byte)97, (byte)33, (byte)136, (byte)123, (byte)132, (byte)212, (byte)179, (byte)12, (byte)153, (byte)49, (byte)31, (byte)69, (byte)236, (byte)26, (byte)19, (byte)13, (byte)33, (byte)51, (byte)201}));
                Debug.Assert(pack.message_type == (ushort)(ushort)24082);
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)219;
            p248.target_system = (byte)(byte)64;
            p248.payload_SET(new byte[] {(byte)121, (byte)115, (byte)171, (byte)205, (byte)55, (byte)75, (byte)169, (byte)152, (byte)2, (byte)91, (byte)115, (byte)204, (byte)228, (byte)109, (byte)244, (byte)78, (byte)239, (byte)122, (byte)158, (byte)255, (byte)242, (byte)28, (byte)91, (byte)91, (byte)105, (byte)242, (byte)9, (byte)199, (byte)21, (byte)52, (byte)208, (byte)211, (byte)220, (byte)80, (byte)213, (byte)46, (byte)55, (byte)237, (byte)90, (byte)113, (byte)177, (byte)223, (byte)1, (byte)192, (byte)114, (byte)101, (byte)30, (byte)237, (byte)128, (byte)136, (byte)179, (byte)109, (byte)222, (byte)114, (byte)23, (byte)159, (byte)103, (byte)245, (byte)140, (byte)32, (byte)61, (byte)247, (byte)209, (byte)223, (byte)174, (byte)249, (byte)61, (byte)48, (byte)51, (byte)187, (byte)157, (byte)3, (byte)198, (byte)148, (byte)252, (byte)50, (byte)82, (byte)44, (byte)182, (byte)155, (byte)190, (byte)6, (byte)83, (byte)188, (byte)135, (byte)38, (byte)184, (byte)166, (byte)109, (byte)76, (byte)150, (byte)175, (byte)126, (byte)170, (byte)92, (byte)219, (byte)245, (byte)196, (byte)94, (byte)246, (byte)124, (byte)30, (byte)89, (byte)1, (byte)83, (byte)49, (byte)231, (byte)124, (byte)198, (byte)196, (byte)250, (byte)130, (byte)156, (byte)84, (byte)9, (byte)50, (byte)137, (byte)48, (byte)80, (byte)62, (byte)21, (byte)72, (byte)2, (byte)136, (byte)189, (byte)94, (byte)16, (byte)86, (byte)97, (byte)189, (byte)80, (byte)89, (byte)24, (byte)209, (byte)79, (byte)5, (byte)133, (byte)129, (byte)53, (byte)162, (byte)243, (byte)171, (byte)252, (byte)0, (byte)66, (byte)126, (byte)95, (byte)212, (byte)93, (byte)58, (byte)249, (byte)139, (byte)22, (byte)103, (byte)222, (byte)156, (byte)97, (byte)177, (byte)95, (byte)17, (byte)187, (byte)65, (byte)45, (byte)181, (byte)14, (byte)88, (byte)92, (byte)109, (byte)135, (byte)124, (byte)122, (byte)121, (byte)142, (byte)83, (byte)41, (byte)170, (byte)110, (byte)234, (byte)145, (byte)43, (byte)101, (byte)19, (byte)193, (byte)23, (byte)76, (byte)247, (byte)88, (byte)187, (byte)45, (byte)8, (byte)180, (byte)5, (byte)63, (byte)44, (byte)131, (byte)132, (byte)121, (byte)230, (byte)38, (byte)210, (byte)31, (byte)134, (byte)129, (byte)246, (byte)117, (byte)68, (byte)87, (byte)57, (byte)129, (byte)165, (byte)110, (byte)135, (byte)51, (byte)51, (byte)29, (byte)221, (byte)88, (byte)53, (byte)147, (byte)7, (byte)120, (byte)79, (byte)189, (byte)32, (byte)234, (byte)164, (byte)178, (byte)117, (byte)225, (byte)224, (byte)97, (byte)33, (byte)136, (byte)123, (byte)132, (byte)212, (byte)179, (byte)12, (byte)153, (byte)49, (byte)31, (byte)69, (byte)236, (byte)26, (byte)19, (byte)13, (byte)33, (byte)51, (byte)201}, 0) ;
            p248.message_type = (ushort)(ushort)24082;
            p248.target_network = (byte)(byte)114;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)214);
                Debug.Assert(pack.address == (ushort)(ushort)20575);
                Debug.Assert(pack.type == (byte)(byte)248);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)19, (sbyte)8, (sbyte)11, (sbyte) - 34, (sbyte)105, (sbyte)77, (sbyte) - 45, (sbyte) - 117, (sbyte) - 127, (sbyte)38, (sbyte) - 51, (sbyte)63, (sbyte) - 89, (sbyte)10, (sbyte) - 120, (sbyte) - 10, (sbyte) - 32, (sbyte)70, (sbyte) - 93, (sbyte) - 29, (sbyte)52, (sbyte) - 61, (sbyte)70, (sbyte) - 121, (sbyte) - 45, (sbyte)120, (sbyte) - 63, (sbyte)48, (sbyte) - 120, (sbyte) - 84, (sbyte)9, (sbyte)34}));
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)20575;
            p249.type = (byte)(byte)248;
            p249.ver = (byte)(byte)214;
            p249.value_SET(new sbyte[] {(sbyte)19, (sbyte)8, (sbyte)11, (sbyte) - 34, (sbyte)105, (sbyte)77, (sbyte) - 45, (sbyte) - 117, (sbyte) - 127, (sbyte)38, (sbyte) - 51, (sbyte)63, (sbyte) - 89, (sbyte)10, (sbyte) - 120, (sbyte) - 10, (sbyte) - 32, (sbyte)70, (sbyte) - 93, (sbyte) - 29, (sbyte)52, (sbyte) - 61, (sbyte)70, (sbyte) - 121, (sbyte) - 45, (sbyte)120, (sbyte) - 63, (sbyte)48, (sbyte) - 120, (sbyte) - 84, (sbyte)9, (sbyte)34}, 0) ;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("xsnlopvdk"));
                Debug.Assert(pack.z == (float) -1.4721966E38F);
                Debug.Assert(pack.y == (float)3.593379E36F);
                Debug.Assert(pack.time_usec == (ulong)6065728891744551741L);
                Debug.Assert(pack.x == (float)6.3193777E37F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)6.3193777E37F;
            p250.z = (float) -1.4721966E38F;
            p250.time_usec = (ulong)6065728891744551741L;
            p250.name_SET("xsnlopvdk", PH) ;
            p250.y = (float)3.593379E36F;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("Ykhht"));
                Debug.Assert(pack.time_boot_ms == (uint)1338799687U);
                Debug.Assert(pack.value == (float) -2.4205547E38F);
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1338799687U;
            p251.name_SET("Ykhht", PH) ;
            p251.value = (float) -2.4205547E38F;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("pH"));
                Debug.Assert(pack.value == (int) -1537741949);
                Debug.Assert(pack.time_boot_ms == (uint)4284980772U);
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -1537741949;
            p252.time_boot_ms = (uint)4284980772U;
            p252.name_SET("pH", PH) ;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 39);
                Debug.Assert(pack.text_TRY(ph).Equals("vvgfynoqummnEbpiqdrzdjOntsXbrkffuzzhtnz"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ALERT);
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("vvgfynoqummnEbpiqdrzdjOntsXbrkffuzzhtnz", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ALERT;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -3.9424727E37F);
                Debug.Assert(pack.ind == (byte)(byte)81);
                Debug.Assert(pack.time_boot_ms == (uint)113563131U);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float) -3.9424727E37F;
            p254.ind = (byte)(byte)81;
            p254.time_boot_ms = (uint)113563131U;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)131, (byte)98, (byte)174, (byte)200, (byte)143, (byte)232, (byte)160, (byte)13, (byte)25, (byte)108, (byte)99, (byte)42, (byte)213, (byte)221, (byte)93, (byte)186, (byte)8, (byte)179, (byte)184, (byte)31, (byte)183, (byte)173, (byte)24, (byte)120, (byte)207, (byte)113, (byte)36, (byte)37, (byte)80, (byte)245, (byte)199, (byte)129}));
                Debug.Assert(pack.initial_timestamp == (ulong)6950273303995261747L);
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)6950273303995261747L;
            p256.target_system = (byte)(byte)238;
            p256.secret_key_SET(new byte[] {(byte)131, (byte)98, (byte)174, (byte)200, (byte)143, (byte)232, (byte)160, (byte)13, (byte)25, (byte)108, (byte)99, (byte)42, (byte)213, (byte)221, (byte)93, (byte)186, (byte)8, (byte)179, (byte)184, (byte)31, (byte)183, (byte)173, (byte)24, (byte)120, (byte)207, (byte)113, (byte)36, (byte)37, (byte)80, (byte)245, (byte)199, (byte)129}, 0) ;
            p256.target_component = (byte)(byte)33;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)46712663U);
                Debug.Assert(pack.time_boot_ms == (uint)3072502319U);
                Debug.Assert(pack.state == (byte)(byte)3);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)46712663U;
            p257.state = (byte)(byte)3;
            p257.time_boot_ms = (uint)3072502319U;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 18);
                Debug.Assert(pack.tune_TRY(ph).Equals("OpGvmdMGvfctmyRbmz"));
                Debug.Assert(pack.target_component == (byte)(byte)221);
                Debug.Assert(pack.target_system == (byte)(byte)77);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)221;
            p258.tune_SET("OpGvmdMGvfctmyRbmz", PH) ;
            p258.target_system = (byte)(byte)77;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)115, (byte)188, (byte)58, (byte)103, (byte)52, (byte)167, (byte)150, (byte)174, (byte)27, (byte)18, (byte)65, (byte)162, (byte)240, (byte)191, (byte)204, (byte)119, (byte)29, (byte)125, (byte)72, (byte)114, (byte)50, (byte)212, (byte)21, (byte)170, (byte)187, (byte)84, (byte)196, (byte)152, (byte)66, (byte)235, (byte)1, (byte)76}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)23921);
                Debug.Assert(pack.sensor_size_v == (float)1.90232E38F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 42);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("gyqmsgnpcklkdklnfqlozomhoiajombszbbNmlqCrS"));
                Debug.Assert(pack.lens_id == (byte)(byte)75);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)9068);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
                Debug.Assert(pack.time_boot_ms == (uint)338876994U);
                Debug.Assert(pack.focal_length == (float)2.2118008E38F);
                Debug.Assert(pack.sensor_size_h == (float) -1.7614912E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)262);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)144, (byte)8, (byte)172, (byte)165, (byte)203, (byte)206, (byte)169, (byte)0, (byte)145, (byte)212, (byte)210, (byte)47, (byte)191, (byte)141, (byte)219, (byte)82, (byte)74, (byte)4, (byte)43, (byte)24, (byte)201, (byte)85, (byte)212, (byte)248, (byte)14, (byte)242, (byte)54, (byte)11, (byte)29, (byte)132, (byte)179, (byte)179}));
                Debug.Assert(pack.firmware_version == (uint)2805185484U);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.focal_length = (float)2.2118008E38F;
            p259.lens_id = (byte)(byte)75;
            p259.model_name_SET(new byte[] {(byte)115, (byte)188, (byte)58, (byte)103, (byte)52, (byte)167, (byte)150, (byte)174, (byte)27, (byte)18, (byte)65, (byte)162, (byte)240, (byte)191, (byte)204, (byte)119, (byte)29, (byte)125, (byte)72, (byte)114, (byte)50, (byte)212, (byte)21, (byte)170, (byte)187, (byte)84, (byte)196, (byte)152, (byte)66, (byte)235, (byte)1, (byte)76}, 0) ;
            p259.sensor_size_v = (float)1.90232E38F;
            p259.vendor_name_SET(new byte[] {(byte)144, (byte)8, (byte)172, (byte)165, (byte)203, (byte)206, (byte)169, (byte)0, (byte)145, (byte)212, (byte)210, (byte)47, (byte)191, (byte)141, (byte)219, (byte)82, (byte)74, (byte)4, (byte)43, (byte)24, (byte)201, (byte)85, (byte)212, (byte)248, (byte)14, (byte)242, (byte)54, (byte)11, (byte)29, (byte)132, (byte)179, (byte)179}, 0) ;
            p259.resolution_h = (ushort)(ushort)23921;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
            p259.time_boot_ms = (uint)338876994U;
            p259.cam_definition_version = (ushort)(ushort)262;
            p259.firmware_version = (uint)2805185484U;
            p259.sensor_size_h = (float) -1.7614912E38F;
            p259.resolution_v = (ushort)(ushort)9068;
            p259.cam_definition_uri_SET("gyqmsgnpcklkdklnfqlozomhoiajombszbbNmlqCrS", PH) ;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2751744925U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            p260.time_boot_ms = (uint)2751744925U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.read_speed == (float)2.7902445E37F);
                Debug.Assert(pack.total_capacity == (float)1.1176871E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1602654233U);
                Debug.Assert(pack.write_speed == (float)2.5886141E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)165);
                Debug.Assert(pack.storage_id == (byte)(byte)22);
                Debug.Assert(pack.status == (byte)(byte)120);
                Debug.Assert(pack.used_capacity == (float)5.8460623E37F);
                Debug.Assert(pack.available_capacity == (float) -7.0522684E37F);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.used_capacity = (float)5.8460623E37F;
            p261.time_boot_ms = (uint)1602654233U;
            p261.storage_count = (byte)(byte)165;
            p261.write_speed = (float)2.5886141E38F;
            p261.storage_id = (byte)(byte)22;
            p261.available_capacity = (float) -7.0522684E37F;
            p261.status = (byte)(byte)120;
            p261.total_capacity = (float)1.1176871E38F;
            p261.read_speed = (float)2.7902445E37F;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)3066573117U);
                Debug.Assert(pack.image_interval == (float)2.6766098E38F);
                Debug.Assert(pack.image_status == (byte)(byte)194);
                Debug.Assert(pack.time_boot_ms == (uint)2234430749U);
                Debug.Assert(pack.available_capacity == (float)1.2704132E38F);
                Debug.Assert(pack.video_status == (byte)(byte)204);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)3066573117U;
            p262.available_capacity = (float)1.2704132E38F;
            p262.time_boot_ms = (uint)2234430749U;
            p262.image_status = (byte)(byte)194;
            p262.video_status = (byte)(byte)204;
            p262.image_interval = (float)2.6766098E38F;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -2073165457);
                Debug.Assert(pack.image_index == (int) -578381927);
                Debug.Assert(pack.relative_alt == (int)575492916);
                Debug.Assert(pack.lon == (int) -254724069);
                Debug.Assert(pack.camera_id == (byte)(byte)104);
                Debug.Assert(pack.lat == (int)2008568976);
                Debug.Assert(pack.time_utc == (ulong)5694606210929539264L);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)42);
                Debug.Assert(pack.file_url_LEN(ph) == 179);
                Debug.Assert(pack.file_url_TRY(ph).Equals("juIpwqzmfxRxwpyvtzvbqncpltpwKzwLQKlmqeFeaxgvepdRrmyzVzyorsxfhqixjvaQpIxStzhDjpbcrxrwlfhuvUJlzfubnbzefycakxusirievvrwjfeuZtdsaxeozkhkzjocgHawoumymvjnjzvzdePOadXtlnLuAfsesTtatrYwGll"));
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6860048E38F, -1.6223437E38F, -1.0495954E38F, -1.3053845E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)3392937303U);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int) -254724069;
            p263.image_index = (int) -578381927;
            p263.camera_id = (byte)(byte)104;
            p263.file_url_SET("juIpwqzmfxRxwpyvtzvbqncpltpwKzwLQKlmqeFeaxgvepdRrmyzVzyorsxfhqixjvaQpIxStzhDjpbcrxrwlfhuvUJlzfubnbzefycakxusirievvrwjfeuZtdsaxeozkhkzjocgHawoumymvjnjzvzdePOadXtlnLuAfsesTtatrYwGll", PH) ;
            p263.alt = (int) -2073165457;
            p263.relative_alt = (int)575492916;
            p263.time_boot_ms = (uint)3392937303U;
            p263.capture_result = (sbyte)(sbyte)42;
            p263.time_utc = (ulong)5694606210929539264L;
            p263.lat = (int)2008568976;
            p263.q_SET(new float[] {2.6860048E38F, -1.6223437E38F, -1.0495954E38F, -1.3053845E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4135593923U);
                Debug.Assert(pack.arming_time_utc == (ulong)3779774450002392186L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5500009970184957568L);
                Debug.Assert(pack.flight_uuid == (ulong)3409552813294976150L);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)5500009970184957568L;
            p264.time_boot_ms = (uint)4135593923U;
            p264.arming_time_utc = (ulong)3779774450002392186L;
            p264.flight_uuid = (ulong)3409552813294976150L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2835425827U);
                Debug.Assert(pack.yaw == (float) -1.5498827E38F);
                Debug.Assert(pack.pitch == (float)1.7162064E38F);
                Debug.Assert(pack.roll == (float)3.2929161E38F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)3.2929161E38F;
            p265.pitch = (float)1.7162064E38F;
            p265.yaw = (float) -1.5498827E38F;
            p265.time_boot_ms = (uint)2835425827U;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)158);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)181, (byte)111, (byte)188, (byte)229, (byte)28, (byte)188, (byte)170, (byte)86, (byte)83, (byte)159, (byte)72, (byte)142, (byte)187, (byte)249, (byte)185, (byte)79, (byte)15, (byte)139, (byte)82, (byte)131, (byte)25, (byte)81, (byte)229, (byte)73, (byte)170, (byte)116, (byte)96, (byte)233, (byte)223, (byte)131, (byte)149, (byte)145, (byte)171, (byte)241, (byte)132, (byte)51, (byte)211, (byte)36, (byte)253, (byte)96, (byte)168, (byte)92, (byte)121, (byte)28, (byte)19, (byte)178, (byte)184, (byte)81, (byte)133, (byte)144, (byte)62, (byte)203, (byte)71, (byte)235, (byte)213, (byte)201, (byte)42, (byte)124, (byte)86, (byte)125, (byte)27, (byte)195, (byte)134, (byte)209, (byte)246, (byte)130, (byte)139, (byte)25, (byte)165, (byte)66, (byte)151, (byte)11, (byte)251, (byte)191, (byte)34, (byte)142, (byte)251, (byte)87, (byte)18, (byte)144, (byte)187, (byte)170, (byte)180, (byte)218, (byte)200, (byte)116, (byte)139, (byte)237, (byte)234, (byte)201, (byte)228, (byte)72, (byte)131, (byte)157, (byte)163, (byte)133, (byte)192, (byte)159, (byte)42, (byte)164, (byte)167, (byte)84, (byte)207, (byte)210, (byte)8, (byte)162, (byte)4, (byte)30, (byte)203, (byte)240, (byte)212, (byte)249, (byte)240, (byte)207, (byte)221, (byte)229, (byte)36, (byte)157, (byte)34, (byte)10, (byte)201, (byte)162, (byte)18, (byte)95, (byte)126, (byte)51, (byte)82, (byte)227, (byte)205, (byte)112, (byte)113, (byte)41, (byte)156, (byte)215, (byte)148, (byte)251, (byte)5, (byte)190, (byte)53, (byte)93, (byte)154, (byte)203, (byte)164, (byte)176, (byte)71, (byte)102, (byte)222, (byte)208, (byte)249, (byte)98, (byte)165, (byte)26, (byte)220, (byte)247, (byte)126, (byte)33, (byte)255, (byte)240, (byte)215, (byte)169, (byte)48, (byte)199, (byte)107, (byte)177, (byte)32, (byte)183, (byte)159, (byte)43, (byte)23, (byte)162, (byte)55, (byte)144, (byte)15, (byte)111, (byte)186, (byte)16, (byte)216, (byte)245, (byte)139, (byte)254, (byte)89, (byte)77, (byte)21, (byte)77, (byte)220, (byte)71, (byte)247, (byte)216, (byte)157, (byte)5, (byte)215, (byte)73, (byte)22, (byte)106, (byte)224, (byte)16, (byte)0, (byte)9, (byte)98, (byte)36, (byte)232, (byte)176, (byte)95, (byte)17, (byte)124, (byte)149, (byte)138, (byte)145, (byte)124, (byte)80, (byte)201, (byte)119, (byte)28, (byte)74, (byte)196, (byte)35, (byte)168, (byte)164, (byte)227, (byte)176, (byte)0, (byte)252, (byte)45, (byte)233, (byte)15, (byte)208, (byte)146, (byte)191, (byte)137, (byte)75, (byte)217, (byte)125, (byte)217, (byte)31, (byte)47, (byte)152, (byte)125, (byte)94, (byte)48, (byte)195, (byte)91, (byte)245, (byte)31, (byte)162, (byte)235, (byte)46, (byte)152, (byte)217}));
                Debug.Assert(pack.length == (byte)(byte)167);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.sequence == (ushort)(ushort)11229);
                Debug.Assert(pack.target_component == (byte)(byte)175);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.length = (byte)(byte)167;
            p266.target_component = (byte)(byte)175;
            p266.target_system = (byte)(byte)203;
            p266.sequence = (ushort)(ushort)11229;
            p266.data__SET(new byte[] {(byte)34, (byte)181, (byte)111, (byte)188, (byte)229, (byte)28, (byte)188, (byte)170, (byte)86, (byte)83, (byte)159, (byte)72, (byte)142, (byte)187, (byte)249, (byte)185, (byte)79, (byte)15, (byte)139, (byte)82, (byte)131, (byte)25, (byte)81, (byte)229, (byte)73, (byte)170, (byte)116, (byte)96, (byte)233, (byte)223, (byte)131, (byte)149, (byte)145, (byte)171, (byte)241, (byte)132, (byte)51, (byte)211, (byte)36, (byte)253, (byte)96, (byte)168, (byte)92, (byte)121, (byte)28, (byte)19, (byte)178, (byte)184, (byte)81, (byte)133, (byte)144, (byte)62, (byte)203, (byte)71, (byte)235, (byte)213, (byte)201, (byte)42, (byte)124, (byte)86, (byte)125, (byte)27, (byte)195, (byte)134, (byte)209, (byte)246, (byte)130, (byte)139, (byte)25, (byte)165, (byte)66, (byte)151, (byte)11, (byte)251, (byte)191, (byte)34, (byte)142, (byte)251, (byte)87, (byte)18, (byte)144, (byte)187, (byte)170, (byte)180, (byte)218, (byte)200, (byte)116, (byte)139, (byte)237, (byte)234, (byte)201, (byte)228, (byte)72, (byte)131, (byte)157, (byte)163, (byte)133, (byte)192, (byte)159, (byte)42, (byte)164, (byte)167, (byte)84, (byte)207, (byte)210, (byte)8, (byte)162, (byte)4, (byte)30, (byte)203, (byte)240, (byte)212, (byte)249, (byte)240, (byte)207, (byte)221, (byte)229, (byte)36, (byte)157, (byte)34, (byte)10, (byte)201, (byte)162, (byte)18, (byte)95, (byte)126, (byte)51, (byte)82, (byte)227, (byte)205, (byte)112, (byte)113, (byte)41, (byte)156, (byte)215, (byte)148, (byte)251, (byte)5, (byte)190, (byte)53, (byte)93, (byte)154, (byte)203, (byte)164, (byte)176, (byte)71, (byte)102, (byte)222, (byte)208, (byte)249, (byte)98, (byte)165, (byte)26, (byte)220, (byte)247, (byte)126, (byte)33, (byte)255, (byte)240, (byte)215, (byte)169, (byte)48, (byte)199, (byte)107, (byte)177, (byte)32, (byte)183, (byte)159, (byte)43, (byte)23, (byte)162, (byte)55, (byte)144, (byte)15, (byte)111, (byte)186, (byte)16, (byte)216, (byte)245, (byte)139, (byte)254, (byte)89, (byte)77, (byte)21, (byte)77, (byte)220, (byte)71, (byte)247, (byte)216, (byte)157, (byte)5, (byte)215, (byte)73, (byte)22, (byte)106, (byte)224, (byte)16, (byte)0, (byte)9, (byte)98, (byte)36, (byte)232, (byte)176, (byte)95, (byte)17, (byte)124, (byte)149, (byte)138, (byte)145, (byte)124, (byte)80, (byte)201, (byte)119, (byte)28, (byte)74, (byte)196, (byte)35, (byte)168, (byte)164, (byte)227, (byte)176, (byte)0, (byte)252, (byte)45, (byte)233, (byte)15, (byte)208, (byte)146, (byte)191, (byte)137, (byte)75, (byte)217, (byte)125, (byte)217, (byte)31, (byte)47, (byte)152, (byte)125, (byte)94, (byte)48, (byte)195, (byte)91, (byte)245, (byte)31, (byte)162, (byte)235, (byte)46, (byte)152, (byte)217}, 0) ;
            p266.first_message_offset = (byte)(byte)158;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.length == (byte)(byte)210);
                Debug.Assert(pack.first_message_offset == (byte)(byte)181);
                Debug.Assert(pack.target_component == (byte)(byte)45);
                Debug.Assert(pack.sequence == (ushort)(ushort)39184);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)33, (byte)202, (byte)81, (byte)16, (byte)120, (byte)131, (byte)67, (byte)54, (byte)131, (byte)158, (byte)192, (byte)148, (byte)248, (byte)195, (byte)206, (byte)7, (byte)212, (byte)27, (byte)252, (byte)96, (byte)131, (byte)235, (byte)157, (byte)204, (byte)49, (byte)102, (byte)247, (byte)101, (byte)101, (byte)177, (byte)45, (byte)38, (byte)115, (byte)0, (byte)27, (byte)175, (byte)30, (byte)219, (byte)55, (byte)249, (byte)2, (byte)115, (byte)93, (byte)68, (byte)189, (byte)21, (byte)74, (byte)236, (byte)110, (byte)161, (byte)244, (byte)255, (byte)137, (byte)181, (byte)15, (byte)116, (byte)50, (byte)199, (byte)221, (byte)214, (byte)4, (byte)66, (byte)103, (byte)232, (byte)4, (byte)67, (byte)28, (byte)31, (byte)228, (byte)163, (byte)234, (byte)248, (byte)40, (byte)99, (byte)20, (byte)230, (byte)41, (byte)60, (byte)131, (byte)42, (byte)2, (byte)11, (byte)220, (byte)201, (byte)53, (byte)188, (byte)53, (byte)157, (byte)50, (byte)151, (byte)236, (byte)204, (byte)89, (byte)148, (byte)98, (byte)231, (byte)201, (byte)44, (byte)82, (byte)48, (byte)139, (byte)219, (byte)188, (byte)215, (byte)84, (byte)84, (byte)5, (byte)188, (byte)129, (byte)243, (byte)119, (byte)254, (byte)83, (byte)190, (byte)153, (byte)171, (byte)82, (byte)159, (byte)230, (byte)252, (byte)171, (byte)85, (byte)72, (byte)44, (byte)22, (byte)194, (byte)147, (byte)32, (byte)24, (byte)226, (byte)2, (byte)118, (byte)99, (byte)188, (byte)62, (byte)29, (byte)183, (byte)196, (byte)13, (byte)5, (byte)236, (byte)207, (byte)16, (byte)169, (byte)43, (byte)212, (byte)110, (byte)83, (byte)172, (byte)172, (byte)23, (byte)19, (byte)42, (byte)123, (byte)244, (byte)113, (byte)170, (byte)73, (byte)239, (byte)249, (byte)162, (byte)107, (byte)105, (byte)198, (byte)175, (byte)101, (byte)144, (byte)90, (byte)223, (byte)199, (byte)71, (byte)198, (byte)69, (byte)80, (byte)226, (byte)205, (byte)21, (byte)80, (byte)110, (byte)205, (byte)192, (byte)181, (byte)232, (byte)168, (byte)65, (byte)246, (byte)242, (byte)11, (byte)106, (byte)29, (byte)238, (byte)90, (byte)73, (byte)0, (byte)213, (byte)161, (byte)103, (byte)184, (byte)168, (byte)241, (byte)101, (byte)94, (byte)147, (byte)14, (byte)236, (byte)234, (byte)55, (byte)192, (byte)17, (byte)43, (byte)174, (byte)96, (byte)11, (byte)217, (byte)4, (byte)168, (byte)64, (byte)21, (byte)222, (byte)216, (byte)32, (byte)145, (byte)243, (byte)90, (byte)18, (byte)105, (byte)220, (byte)150, (byte)152, (byte)156, (byte)120, (byte)57, (byte)200, (byte)231, (byte)128, (byte)143, (byte)246, (byte)205, (byte)197, (byte)14, (byte)46, (byte)16, (byte)200, (byte)65, (byte)8, (byte)3, (byte)191, (byte)202, (byte)170}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)33, (byte)202, (byte)81, (byte)16, (byte)120, (byte)131, (byte)67, (byte)54, (byte)131, (byte)158, (byte)192, (byte)148, (byte)248, (byte)195, (byte)206, (byte)7, (byte)212, (byte)27, (byte)252, (byte)96, (byte)131, (byte)235, (byte)157, (byte)204, (byte)49, (byte)102, (byte)247, (byte)101, (byte)101, (byte)177, (byte)45, (byte)38, (byte)115, (byte)0, (byte)27, (byte)175, (byte)30, (byte)219, (byte)55, (byte)249, (byte)2, (byte)115, (byte)93, (byte)68, (byte)189, (byte)21, (byte)74, (byte)236, (byte)110, (byte)161, (byte)244, (byte)255, (byte)137, (byte)181, (byte)15, (byte)116, (byte)50, (byte)199, (byte)221, (byte)214, (byte)4, (byte)66, (byte)103, (byte)232, (byte)4, (byte)67, (byte)28, (byte)31, (byte)228, (byte)163, (byte)234, (byte)248, (byte)40, (byte)99, (byte)20, (byte)230, (byte)41, (byte)60, (byte)131, (byte)42, (byte)2, (byte)11, (byte)220, (byte)201, (byte)53, (byte)188, (byte)53, (byte)157, (byte)50, (byte)151, (byte)236, (byte)204, (byte)89, (byte)148, (byte)98, (byte)231, (byte)201, (byte)44, (byte)82, (byte)48, (byte)139, (byte)219, (byte)188, (byte)215, (byte)84, (byte)84, (byte)5, (byte)188, (byte)129, (byte)243, (byte)119, (byte)254, (byte)83, (byte)190, (byte)153, (byte)171, (byte)82, (byte)159, (byte)230, (byte)252, (byte)171, (byte)85, (byte)72, (byte)44, (byte)22, (byte)194, (byte)147, (byte)32, (byte)24, (byte)226, (byte)2, (byte)118, (byte)99, (byte)188, (byte)62, (byte)29, (byte)183, (byte)196, (byte)13, (byte)5, (byte)236, (byte)207, (byte)16, (byte)169, (byte)43, (byte)212, (byte)110, (byte)83, (byte)172, (byte)172, (byte)23, (byte)19, (byte)42, (byte)123, (byte)244, (byte)113, (byte)170, (byte)73, (byte)239, (byte)249, (byte)162, (byte)107, (byte)105, (byte)198, (byte)175, (byte)101, (byte)144, (byte)90, (byte)223, (byte)199, (byte)71, (byte)198, (byte)69, (byte)80, (byte)226, (byte)205, (byte)21, (byte)80, (byte)110, (byte)205, (byte)192, (byte)181, (byte)232, (byte)168, (byte)65, (byte)246, (byte)242, (byte)11, (byte)106, (byte)29, (byte)238, (byte)90, (byte)73, (byte)0, (byte)213, (byte)161, (byte)103, (byte)184, (byte)168, (byte)241, (byte)101, (byte)94, (byte)147, (byte)14, (byte)236, (byte)234, (byte)55, (byte)192, (byte)17, (byte)43, (byte)174, (byte)96, (byte)11, (byte)217, (byte)4, (byte)168, (byte)64, (byte)21, (byte)222, (byte)216, (byte)32, (byte)145, (byte)243, (byte)90, (byte)18, (byte)105, (byte)220, (byte)150, (byte)152, (byte)156, (byte)120, (byte)57, (byte)200, (byte)231, (byte)128, (byte)143, (byte)246, (byte)205, (byte)197, (byte)14, (byte)46, (byte)16, (byte)200, (byte)65, (byte)8, (byte)3, (byte)191, (byte)202, (byte)170}, 0) ;
            p267.target_component = (byte)(byte)45;
            p267.first_message_offset = (byte)(byte)181;
            p267.target_system = (byte)(byte)9;
            p267.sequence = (ushort)(ushort)39184;
            p267.length = (byte)(byte)210;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)42366);
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.target_system == (byte)(byte)5);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)59;
            p268.target_system = (byte)(byte)5;
            p268.sequence = (ushort)(ushort)42366;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)203);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)54804);
                Debug.Assert(pack.bitrate == (uint)2819900675U);
                Debug.Assert(pack.uri_LEN(ph) == 122);
                Debug.Assert(pack.uri_TRY(ph).Equals("cmknwUYnvzounmnghqrbxkowupeoyjmqchnbdmlhttmjgbjpnefufPrnzgwOwcbgXbxsqufgxjlaunntthnbjupssjrtmskyrIkoysqWqaozxzcswyrqkGhdvb"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)71);
                Debug.Assert(pack.camera_id == (byte)(byte)103);
                Debug.Assert(pack.framerate == (float)1.0682725E38F);
                Debug.Assert(pack.rotation == (ushort)(ushort)41456);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)71;
            p269.rotation = (ushort)(ushort)41456;
            p269.status = (byte)(byte)203;
            p269.bitrate = (uint)2819900675U;
            p269.uri_SET("cmknwUYnvzounmnghqrbxkowupeoyjmqchnbdmlhttmjgbjpnefufPrnzgwOwcbgXbxsqufgxjlaunntthnbjupssjrtmskyrIkoysqWqaozxzcswyrqkGhdvb", PH) ;
            p269.camera_id = (byte)(byte)103;
            p269.framerate = (float)1.0682725E38F;
            p269.resolution_v = (ushort)(ushort)54804;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)177);
                Debug.Assert(pack.bitrate == (uint)2681764215U);
                Debug.Assert(pack.target_system == (byte)(byte)98);
                Debug.Assert(pack.framerate == (float)7.413919E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)37945);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)45467);
                Debug.Assert(pack.camera_id == (byte)(byte)103);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)9967);
                Debug.Assert(pack.uri_LEN(ph) == 156);
                Debug.Assert(pack.uri_TRY(ph).Equals("lmrojOwattqrbuyzxkbbyopkjixdarmtpsksatrngbzdptykrgfhzudpbhMhmuCvaqqXgdhtvfUimfrtpreohrEZdzakyunvmsnLfdtzuacbHfhloyptXjzhinghSjbbnbimxnbqamJzwblrBdtasprevnrc"));
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)98;
            p270.uri_SET("lmrojOwattqrbuyzxkbbyopkjixdarmtpsksatrngbzdptykrgfhzudpbhMhmuCvaqqXgdhtvfUimfrtpreohrEZdzakyunvmsnLfdtzuacbHfhloyptXjzhinghSjbbnbimxnbqamJzwblrBdtasprevnrc", PH) ;
            p270.bitrate = (uint)2681764215U;
            p270.camera_id = (byte)(byte)103;
            p270.resolution_h = (ushort)(ushort)45467;
            p270.rotation = (ushort)(ushort)37945;
            p270.target_component = (byte)(byte)177;
            p270.framerate = (float)7.413919E37F;
            p270.resolution_v = (ushort)(ushort)9967;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 32);
                Debug.Assert(pack.ssid_TRY(ph).Equals("cridipdabcxedcUbtchlcCwtkjwnuDmf"));
                Debug.Assert(pack.password_LEN(ph) == 57);
                Debug.Assert(pack.password_TRY(ph).Equals("pwsjDoRtzxlueyjMvhsGkcYfjpxasOegrthndsenuxcgpPMjghxKmffhu"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("cridipdabcxedcUbtchlcCwtkjwnuDmf", PH) ;
            p299.password_SET("pwsjDoRtzxlueyjMvhsGkcYfjpxasOegrthndsenuxcgpPMjghxKmffhu", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)2946);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)247, (byte)56, (byte)115, (byte)78, (byte)222, (byte)23, (byte)141, (byte)125}));
                Debug.Assert(pack.version == (ushort)(ushort)24186);
                Debug.Assert(pack.max_version == (ushort)(ushort)64099);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)198, (byte)4, (byte)41, (byte)92, (byte)164, (byte)234, (byte)87, (byte)230}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)24186;
            p300.library_version_hash_SET(new byte[] {(byte)198, (byte)4, (byte)41, (byte)92, (byte)164, (byte)234, (byte)87, (byte)230}, 0) ;
            p300.max_version = (ushort)(ushort)64099;
            p300.min_version = (ushort)(ushort)2946;
            p300.spec_version_hash_SET(new byte[] {(byte)247, (byte)56, (byte)115, (byte)78, (byte)222, (byte)23, (byte)141, (byte)125}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)7576);
                Debug.Assert(pack.uptime_sec == (uint)3870050872U);
                Debug.Assert(pack.time_usec == (ulong)3584328244300533540L);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.sub_mode == (byte)(byte)132);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)7576;
            p310.uptime_sec = (uint)3870050872U;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)132;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.time_usec = (ulong)3584328244300533540L;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 18);
                Debug.Assert(pack.name_TRY(ph).Equals("ooyhevdcoWdripqbtC"));
                Debug.Assert(pack.hw_version_major == (byte)(byte)148);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)37);
                Debug.Assert(pack.sw_vcs_commit == (uint)2086632829U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)67, (byte)140, (byte)95, (byte)146, (byte)28, (byte)59, (byte)175, (byte)15, (byte)241, (byte)200, (byte)57, (byte)52, (byte)9, (byte)111, (byte)131, (byte)250}));
                Debug.Assert(pack.sw_version_major == (byte)(byte)107);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)183);
                Debug.Assert(pack.time_usec == (ulong)8220535635124024924L);
                Debug.Assert(pack.uptime_sec == (uint)2378268102U);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)37;
            p311.sw_version_major = (byte)(byte)107;
            p311.hw_unique_id_SET(new byte[] {(byte)67, (byte)140, (byte)95, (byte)146, (byte)28, (byte)59, (byte)175, (byte)15, (byte)241, (byte)200, (byte)57, (byte)52, (byte)9, (byte)111, (byte)131, (byte)250}, 0) ;
            p311.time_usec = (ulong)8220535635124024924L;
            p311.sw_vcs_commit = (uint)2086632829U;
            p311.hw_version_minor = (byte)(byte)183;
            p311.name_SET("ooyhevdcoWdripqbtC", PH) ;
            p311.hw_version_major = (byte)(byte)148;
            p311.uptime_sec = (uint)2378268102U;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wgsurupjcixsmm"));
                Debug.Assert(pack.target_component == (byte)(byte)119);
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.param_index == (short)(short)30796);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)30796;
            p320.param_id_SET("wgsurupjcixsmm", PH) ;
            p320.target_component = (byte)(byte)119;
            p320.target_system = (byte)(byte)226;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)207);
                Debug.Assert(pack.target_component == (byte)(byte)143);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)207;
            p321.target_component = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xftfNU"));
                Debug.Assert(pack.param_value_LEN(ph) == 27);
                Debug.Assert(pack.param_value_TRY(ph).Equals("glaaljDeqaytnryywkfndCibsVw"));
                Debug.Assert(pack.param_index == (ushort)(ushort)59153);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)43183);
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("xftfNU", PH) ;
            p322.param_index = (ushort)(ushort)59153;
            p322.param_count = (ushort)(ushort)43183;
            p322.param_value_SET("glaaljDeqaytnryywkfndCibsVw", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.param_value_LEN(ph) == 35);
                Debug.Assert(pack.param_value_TRY(ph).Equals("hihcppooxtoubujjxinQfLuulhpcznBpyra"));
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xirYebdyOMFtTw"));
                Debug.Assert(pack.target_component == (byte)(byte)133);
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("hihcppooxtoubujjxinQfLuulhpcznBpyra", PH) ;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.target_system = (byte)(byte)136;
            p323.target_component = (byte)(byte)133;
            p323.param_id_SET("xirYebdyOMFtTw", PH) ;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jedjncvwtmhdmkj"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_value_LEN(ph) == 69);
                Debug.Assert(pack.param_value_TRY(ph).Equals("qSrxasfkysdtlwwnikpnJcgkWgzckrbeishrtjtzpjcilrjzwveuqlgIhuolmgqRnokqs"));
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("qSrxasfkysdtlwwnikpnJcgkWgzckrbeishrtjtzpjcilrjzwveuqlgIhuolmgqRnokqs", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_id_SET("jedjncvwtmhdmkj", PH) ;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)23840, (ushort)31537, (ushort)6335, (ushort)42116, (ushort)2162, (ushort)53450, (ushort)7945, (ushort)17802, (ushort)26964, (ushort)7365, (ushort)1389, (ushort)55319, (ushort)51918, (ushort)24813, (ushort)33949, (ushort)18945, (ushort)46271, (ushort)28363, (ushort)26282, (ushort)26244, (ushort)62153, (ushort)6105, (ushort)16734, (ushort)5867, (ushort)28372, (ushort)16649, (ushort)3210, (ushort)27229, (ushort)5685, (ushort)32047, (ushort)49489, (ushort)23034, (ushort)61336, (ushort)41584, (ushort)24529, (ushort)49853, (ushort)16018, (ushort)24147, (ushort)58970, (ushort)19920, (ushort)29382, (ushort)21361, (ushort)28149, (ushort)48613, (ushort)28785, (ushort)50763, (ushort)19929, (ushort)25218, (ushort)15544, (ushort)5692, (ushort)32220, (ushort)41137, (ushort)47697, (ushort)35479, (ushort)43498, (ushort)56071, (ushort)32325, (ushort)51758, (ushort)14707, (ushort)13588, (ushort)65500, (ushort)32416, (ushort)13395, (ushort)43255, (ushort)59646, (ushort)61479, (ushort)6104, (ushort)35854, (ushort)35792, (ushort)27086, (ushort)41099, (ushort)12746}));
                Debug.Assert(pack.time_usec == (ulong)2277152552636128967L);
                Debug.Assert(pack.min_distance == (ushort)(ushort)44292);
                Debug.Assert(pack.max_distance == (ushort)(ushort)51297);
                Debug.Assert(pack.increment == (byte)(byte)193);
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)51297;
            p330.distances_SET(new ushort[] {(ushort)23840, (ushort)31537, (ushort)6335, (ushort)42116, (ushort)2162, (ushort)53450, (ushort)7945, (ushort)17802, (ushort)26964, (ushort)7365, (ushort)1389, (ushort)55319, (ushort)51918, (ushort)24813, (ushort)33949, (ushort)18945, (ushort)46271, (ushort)28363, (ushort)26282, (ushort)26244, (ushort)62153, (ushort)6105, (ushort)16734, (ushort)5867, (ushort)28372, (ushort)16649, (ushort)3210, (ushort)27229, (ushort)5685, (ushort)32047, (ushort)49489, (ushort)23034, (ushort)61336, (ushort)41584, (ushort)24529, (ushort)49853, (ushort)16018, (ushort)24147, (ushort)58970, (ushort)19920, (ushort)29382, (ushort)21361, (ushort)28149, (ushort)48613, (ushort)28785, (ushort)50763, (ushort)19929, (ushort)25218, (ushort)15544, (ushort)5692, (ushort)32220, (ushort)41137, (ushort)47697, (ushort)35479, (ushort)43498, (ushort)56071, (ushort)32325, (ushort)51758, (ushort)14707, (ushort)13588, (ushort)65500, (ushort)32416, (ushort)13395, (ushort)43255, (ushort)59646, (ushort)61479, (ushort)6104, (ushort)35854, (ushort)35792, (ushort)27086, (ushort)41099, (ushort)12746}, 0) ;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.time_usec = (ulong)2277152552636128967L;
            p330.increment = (byte)(byte)193;
            p330.min_distance = (ushort)(ushort)44292;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
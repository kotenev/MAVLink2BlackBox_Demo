
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
                Debug.Assert(pack.custom_mode == (uint)3568996992U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_PARAFOIL);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.mavlink_version == (byte)(byte)235);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
            };
            DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_CALIBRATING;
            p0.mavlink_version = (byte)(byte)235;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_PARAFOIL;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS;
            p0.custom_mode = (uint)3568996992U;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)28526);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)47054);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 127);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
                Debug.Assert(pack.load == (ushort)(ushort)40783);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)3659);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)19950);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)6170);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)7197);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)35412);
                Debug.Assert(pack.current_battery == (short)(short)12757);
            };
            DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)47054;
            p1.battery_remaining = (sbyte)(sbyte) - 127;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
            p1.errors_comm = (ushort)(ushort)19950;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
            p1.errors_count1 = (ushort)(ushort)28526;
            p1.drop_rate_comm = (ushort)(ushort)3659;
            p1.load = (ushort)(ushort)40783;
            p1.voltage_battery = (ushort)(ushort)6170;
            p1.errors_count4 = (ushort)(ushort)35412;
            p1.current_battery = (short)(short)12757;
            p1.errors_count2 = (ushort)(ushort)7197;
            LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)7986426465353072534L);
                Debug.Assert(pack.time_boot_ms == (uint)3402515187U);
            };
            DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)7986426465353072534L;
            p2.time_boot_ms = (uint)3402515187U;
            LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)1.2572697E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.type_mask == (ushort)(ushort)38924);
                Debug.Assert(pack.yaw_rate == (float) -2.527213E38F);
                Debug.Assert(pack.x == (float) -1.233163E38F);
                Debug.Assert(pack.yaw == (float)2.9924742E38F);
                Debug.Assert(pack.vx == (float)2.4701746E38F);
                Debug.Assert(pack.z == (float) -5.8259234E37F);
                Debug.Assert(pack.afx == (float) -3.32177E37F);
                Debug.Assert(pack.y == (float) -2.3949828E38F);
                Debug.Assert(pack.afz == (float)2.2063134E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3474332255U);
                Debug.Assert(pack.afy == (float) -2.0920886E38F);
                Debug.Assert(pack.vz == (float)1.3765508E38F);
            };
            DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vx = (float)2.4701746E38F;
            p3.vy = (float)1.2572697E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p3.afy = (float) -2.0920886E38F;
            p3.yaw_rate = (float) -2.527213E38F;
            p3.vz = (float)1.3765508E38F;
            p3.x = (float) -1.233163E38F;
            p3.afz = (float)2.2063134E38F;
            p3.yaw = (float)2.9924742E38F;
            p3.time_boot_ms = (uint)3474332255U;
            p3.z = (float) -5.8259234E37F;
            p3.type_mask = (ushort)(ushort)38924;
            p3.y = (float) -2.3949828E38F;
            p3.afx = (float) -3.32177E37F;
            LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)44);
                Debug.Assert(pack.target_system == (byte)(byte)201);
                Debug.Assert(pack.time_usec == (ulong)2894831376940808535L);
                Debug.Assert(pack.seq == (uint)204458298U);
            };
            DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)44;
            p4.target_system = (byte)(byte)201;
            p4.time_usec = (ulong)2894831376940808535L;
            p4.seq = (uint)204458298U;
            LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.version == (byte)(byte)239);
                Debug.Assert(pack.control_request == (byte)(byte)30);
                Debug.Assert(pack.passkey_LEN(ph) == 6);
                Debug.Assert(pack.passkey_TRY(ph).Equals("gginlt"));
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)239;
            p5.target_system = (byte)(byte)40;
            p5.control_request = (byte)(byte)30;
            p5.passkey_SET("gginlt", PH) ;
            LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)170);
                Debug.Assert(pack.control_request == (byte)(byte)56);
                Debug.Assert(pack.ack == (byte)(byte)130);
            };
            DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)130;
            p6.gcs_system_id = (byte)(byte)170;
            p6.control_request = (byte)(byte)56;
            LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 25);
                Debug.Assert(pack.key_TRY(ph).Equals("fiplxtkiqgtlxqfLhfyOwDwgy"));
            };
            DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("fiplxtkiqgtlxqfLhfyOwDwgy", PH) ;
            LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.target_system == (byte)(byte)170);
                Debug.Assert(pack.custom_mode == (uint)2863355254U);
            };
            DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)170;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p11.custom_mode = (uint)2863355254U;
            LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)231);
                Debug.Assert(pack.param_index == (short)(short) -8406);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wrdoowbngj"));
                Debug.Assert(pack.target_system == (byte)(byte)97);
            };
            DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -8406;
            p20.target_component = (byte)(byte)231;
            p20.target_system = (byte)(byte)97;
            p20.param_id_SET("wrdoowbngj", PH) ;
            LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.target_component == (byte)(byte)151);
            };
            DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)151;
            p21.target_system = (byte)(byte)251;
            LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)48693);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pjyvos"));
                Debug.Assert(pack.param_value == (float)3.376494E38F);
                Debug.Assert(pack.param_count == (ushort)(ushort)61945);
            };
            DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float)3.376494E38F;
            p22.param_count = (ushort)(ushort)61945;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_id_SET("pjyvos", PH) ;
            p22.param_index = (ushort)(ushort)48693;
            LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("iqsDnTCrtjwhyhy"));
                Debug.Assert(pack.target_component == (byte)(byte)183);
                Debug.Assert(pack.param_value == (float) -1.4902034E38F);
            };
            DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("iqsDnTCrtjwhyhy", PH) ;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p23.target_system = (byte)(byte)237;
            p23.param_value = (float) -1.4902034E38F;
            p23.target_component = (byte)(byte)183;
            LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1663951794U);
                Debug.Assert(pack.lat == (int) -1465935631);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1328502793);
                Debug.Assert(pack.time_usec == (ulong)141221722562111848L);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1290371643U);
                Debug.Assert(pack.eph == (ushort)(ushort)32299);
                Debug.Assert(pack.cog == (ushort)(ushort)2951);
                Debug.Assert(pack.vel == (ushort)(ushort)14084);
                Debug.Assert(pack.satellites_visible == (byte)(byte)60);
                Debug.Assert(pack.alt == (int) -544341767);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)74157924U);
                Debug.Assert(pack.epv == (ushort)(ushort)39130);
                Debug.Assert(pack.lon == (int)839134800);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1181369373U);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            };
            DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.lat = (int) -1465935631;
            p24.lon = (int)839134800;
            p24.alt_ellipsoid_SET((int)1328502793, PH) ;
            p24.vel_acc_SET((uint)74157924U, PH) ;
            p24.eph = (ushort)(ushort)32299;
            p24.alt = (int) -544341767;
            p24.epv = (ushort)(ushort)39130;
            p24.cog = (ushort)(ushort)2951;
            p24.satellites_visible = (byte)(byte)60;
            p24.vel = (ushort)(ushort)14084;
            p24.h_acc_SET((uint)1663951794U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p24.hdg_acc_SET((uint)1181369373U, PH) ;
            p24.time_usec = (ulong)141221722562111848L;
            p24.v_acc_SET((uint)1290371643U, PH) ;
            LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)157, (byte)106, (byte)211, (byte)23, (byte)223, (byte)149, (byte)95, (byte)177, (byte)127, (byte)21, (byte)112, (byte)195, (byte)34, (byte)71, (byte)50, (byte)175, (byte)123, (byte)163, (byte)250, (byte)182}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)62);
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)29, (byte)226, (byte)12, (byte)113, (byte)239, (byte)68, (byte)21, (byte)144, (byte)211, (byte)104, (byte)36, (byte)123, (byte)177, (byte)76, (byte)39, (byte)91, (byte)38, (byte)59, (byte)176, (byte)121}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)86, (byte)91, (byte)63, (byte)207, (byte)111, (byte)86, (byte)157, (byte)216, (byte)240, (byte)124, (byte)142, (byte)174, (byte)173, (byte)121, (byte)90, (byte)196, (byte)160, (byte)3, (byte)208, (byte)93}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)105, (byte)119, (byte)54, (byte)172, (byte)105, (byte)30, (byte)139, (byte)84, (byte)104, (byte)70, (byte)89, (byte)232, (byte)40, (byte)225, (byte)201, (byte)220, (byte)158, (byte)82, (byte)67, (byte)50}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)75, (byte)73, (byte)108, (byte)64, (byte)132, (byte)129, (byte)75, (byte)167, (byte)10, (byte)91, (byte)27, (byte)141, (byte)110, (byte)146, (byte)13, (byte)126, (byte)161, (byte)168, (byte)145, (byte)64}));
            };
            DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)86, (byte)91, (byte)63, (byte)207, (byte)111, (byte)86, (byte)157, (byte)216, (byte)240, (byte)124, (byte)142, (byte)174, (byte)173, (byte)121, (byte)90, (byte)196, (byte)160, (byte)3, (byte)208, (byte)93}, 0) ;
            p25.satellites_visible = (byte)(byte)62;
            p25.satellite_azimuth_SET(new byte[] {(byte)29, (byte)226, (byte)12, (byte)113, (byte)239, (byte)68, (byte)21, (byte)144, (byte)211, (byte)104, (byte)36, (byte)123, (byte)177, (byte)76, (byte)39, (byte)91, (byte)38, (byte)59, (byte)176, (byte)121}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)75, (byte)73, (byte)108, (byte)64, (byte)132, (byte)129, (byte)75, (byte)167, (byte)10, (byte)91, (byte)27, (byte)141, (byte)110, (byte)146, (byte)13, (byte)126, (byte)161, (byte)168, (byte)145, (byte)64}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)105, (byte)119, (byte)54, (byte)172, (byte)105, (byte)30, (byte)139, (byte)84, (byte)104, (byte)70, (byte)89, (byte)232, (byte)40, (byte)225, (byte)201, (byte)220, (byte)158, (byte)82, (byte)67, (byte)50}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)157, (byte)106, (byte)211, (byte)23, (byte)223, (byte)149, (byte)95, (byte)177, (byte)127, (byte)21, (byte)112, (byte)195, (byte)34, (byte)71, (byte)50, (byte)175, (byte)123, (byte)163, (byte)250, (byte)182}, 0) ;
            LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -2807);
                Debug.Assert(pack.ygyro == (short)(short)835);
                Debug.Assert(pack.xgyro == (short)(short) -10960);
                Debug.Assert(pack.ymag == (short)(short)5085);
                Debug.Assert(pack.zacc == (short)(short)29579);
                Debug.Assert(pack.xmag == (short)(short)8017);
                Debug.Assert(pack.zmag == (short)(short) -25724);
                Debug.Assert(pack.xacc == (short)(short)27474);
                Debug.Assert(pack.time_boot_ms == (uint)5988923U);
                Debug.Assert(pack.zgyro == (short)(short) -831);
            };
            DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short) -831;
            p26.xmag = (short)(short)8017;
            p26.xacc = (short)(short)27474;
            p26.ymag = (short)(short)5085;
            p26.xgyro = (short)(short) -10960;
            p26.zmag = (short)(short) -25724;
            p26.ygyro = (short)(short)835;
            p26.yacc = (short)(short) -2807;
            p26.zacc = (short)(short)29579;
            p26.time_boot_ms = (uint)5988923U;
            LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -25703);
                Debug.Assert(pack.ymag == (short)(short)16296);
                Debug.Assert(pack.xmag == (short)(short)9521);
                Debug.Assert(pack.time_usec == (ulong)3200927225745335025L);
                Debug.Assert(pack.yacc == (short)(short)10852);
                Debug.Assert(pack.xacc == (short)(short)25745);
                Debug.Assert(pack.zacc == (short)(short)20707);
                Debug.Assert(pack.zgyro == (short)(short) -16035);
                Debug.Assert(pack.zmag == (short)(short)27179);
                Debug.Assert(pack.xgyro == (short)(short)20776);
            };
            DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.ygyro = (short)(short) -25703;
            p27.zmag = (short)(short)27179;
            p27.xacc = (short)(short)25745;
            p27.zacc = (short)(short)20707;
            p27.xmag = (short)(short)9521;
            p27.time_usec = (ulong)3200927225745335025L;
            p27.yacc = (short)(short)10852;
            p27.xgyro = (short)(short)20776;
            p27.ymag = (short)(short)16296;
            p27.zgyro = (short)(short) -16035;
            LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short) -94);
                Debug.Assert(pack.time_usec == (ulong)2524462514857951878L);
                Debug.Assert(pack.temperature == (short)(short) -17584);
                Debug.Assert(pack.press_diff1 == (short)(short)14557);
                Debug.Assert(pack.press_abs == (short)(short) -22641);
            };
            DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)2524462514857951878L;
            p28.temperature = (short)(short) -17584;
            p28.press_abs = (short)(short) -22641;
            p28.press_diff1 = (short)(short)14557;
            p28.press_diff2 = (short)(short) -94;
            LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)3.0502736E38F);
                Debug.Assert(pack.press_diff == (float)2.5032735E38F);
                Debug.Assert(pack.temperature == (short)(short)13166);
                Debug.Assert(pack.time_boot_ms == (uint)3894213424U);
            };
            DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)2.5032735E38F;
            p29.time_boot_ms = (uint)3894213424U;
            p29.temperature = (short)(short)13166;
            p29.press_abs = (float)3.0502736E38F;
            LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)2.334566E38F);
                Debug.Assert(pack.rollspeed == (float)2.8394724E38F);
                Debug.Assert(pack.yawspeed == (float) -2.1455297E38F);
                Debug.Assert(pack.pitch == (float)3.1850888E38F);
                Debug.Assert(pack.time_boot_ms == (uint)874306453U);
                Debug.Assert(pack.roll == (float)2.8310565E38F);
                Debug.Assert(pack.yaw == (float)1.4569158E38F);
            };
            DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.rollspeed = (float)2.8394724E38F;
            p30.yaw = (float)1.4569158E38F;
            p30.pitch = (float)3.1850888E38F;
            p30.roll = (float)2.8310565E38F;
            p30.time_boot_ms = (uint)874306453U;
            p30.pitchspeed = (float)2.334566E38F;
            p30.yawspeed = (float) -2.1455297E38F;
            LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)2.1293129E38F);
                Debug.Assert(pack.q3 == (float)2.569658E38F);
                Debug.Assert(pack.q1 == (float) -2.7929008E38F);
                Debug.Assert(pack.pitchspeed == (float) -3.9787093E37F);
                Debug.Assert(pack.q4 == (float)3.0876162E38F);
                Debug.Assert(pack.q2 == (float)1.5456836E38F);
                Debug.Assert(pack.time_boot_ms == (uint)319090860U);
                Debug.Assert(pack.rollspeed == (float) -3.6731796E37F);
            };
            DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float)3.0876162E38F;
            p31.q1 = (float) -2.7929008E38F;
            p31.rollspeed = (float) -3.6731796E37F;
            p31.q2 = (float)1.5456836E38F;
            p31.yawspeed = (float)2.1293129E38F;
            p31.q3 = (float)2.569658E38F;
            p31.pitchspeed = (float) -3.9787093E37F;
            p31.time_boot_ms = (uint)319090860U;
            LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.5216772E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3672775899U);
                Debug.Assert(pack.z == (float) -1.7798153E38F);
                Debug.Assert(pack.x == (float) -2.3433213E38F);
                Debug.Assert(pack.vy == (float)8.0378814E37F);
                Debug.Assert(pack.y == (float)2.1095941E38F);
                Debug.Assert(pack.vz == (float)5.557766E35F);
            };
            DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.x = (float) -2.3433213E38F;
            p32.vy = (float)8.0378814E37F;
            p32.z = (float) -1.7798153E38F;
            p32.y = (float)2.1095941E38F;
            p32.vz = (float)5.557766E35F;
            p32.time_boot_ms = (uint)3672775899U;
            p32.vx = (float) -1.5216772E38F;
            LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (short)(short)18502);
                Debug.Assert(pack.vx == (short)(short) -11220);
                Debug.Assert(pack.time_boot_ms == (uint)2057206913U);
                Debug.Assert(pack.lon == (int)1564303731);
                Debug.Assert(pack.lat == (int)2099015751);
                Debug.Assert(pack.vy == (short)(short)20479);
                Debug.Assert(pack.hdg == (ushort)(ushort)752);
                Debug.Assert(pack.relative_alt == (int)1727772283);
                Debug.Assert(pack.alt == (int) -660713120);
            };
            DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int)1564303731;
            p33.hdg = (ushort)(ushort)752;
            p33.vx = (short)(short) -11220;
            p33.vy = (short)(short)20479;
            p33.lat = (int)2099015751;
            p33.relative_alt = (int)1727772283;
            p33.vz = (short)(short)18502;
            p33.time_boot_ms = (uint)2057206913U;
            p33.alt = (int) -660713120;
            LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_scaled == (short)(short) -4199);
                Debug.Assert(pack.port == (byte)(byte)9);
                Debug.Assert(pack.chan3_scaled == (short)(short)20454);
                Debug.Assert(pack.chan4_scaled == (short)(short) -16761);
                Debug.Assert(pack.chan5_scaled == (short)(short) -13157);
                Debug.Assert(pack.chan2_scaled == (short)(short) -8025);
                Debug.Assert(pack.chan1_scaled == (short)(short) -22425);
                Debug.Assert(pack.rssi == (byte)(byte)140);
                Debug.Assert(pack.chan7_scaled == (short)(short)18992);
                Debug.Assert(pack.time_boot_ms == (uint)3631638188U);
                Debug.Assert(pack.chan8_scaled == (short)(short) -22549);
            };
            DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)140;
            p34.chan8_scaled = (short)(short) -22549;
            p34.chan4_scaled = (short)(short) -16761;
            p34.chan5_scaled = (short)(short) -13157;
            p34.chan2_scaled = (short)(short) -8025;
            p34.port = (byte)(byte)9;
            p34.chan6_scaled = (short)(short) -4199;
            p34.time_boot_ms = (uint)3631638188U;
            p34.chan7_scaled = (short)(short)18992;
            p34.chan3_scaled = (short)(short)20454;
            p34.chan1_scaled = (short)(short) -22425;
            LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)2375);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)14553);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)1910);
                Debug.Assert(pack.rssi == (byte)(byte)66);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)42556);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)31440);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)19745);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)30685);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)54049);
                Debug.Assert(pack.time_boot_ms == (uint)1306203614U);
                Debug.Assert(pack.port == (byte)(byte)36);
            };
            DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan7_raw = (ushort)(ushort)2375;
            p35.chan8_raw = (ushort)(ushort)54049;
            p35.rssi = (byte)(byte)66;
            p35.chan6_raw = (ushort)(ushort)14553;
            p35.chan4_raw = (ushort)(ushort)31440;
            p35.time_boot_ms = (uint)1306203614U;
            p35.chan1_raw = (ushort)(ushort)19745;
            p35.chan3_raw = (ushort)(ushort)42556;
            p35.chan2_raw = (ushort)(ushort)30685;
            p35.port = (byte)(byte)36;
            p35.chan5_raw = (ushort)(ushort)1910;
            LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)63439);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)27839);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)39342);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)42442);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)62678);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)8316);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)50147);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)53301);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)40451);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)40711);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)50203);
                Debug.Assert(pack.time_usec == (uint)2582748029U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)23381);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)29568);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)18329);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)37263);
                Debug.Assert(pack.port == (byte)(byte)92);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)9094);
            };
            DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo10_raw_SET((ushort)(ushort)23381, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)63439, PH) ;
            p36.servo5_raw = (ushort)(ushort)40451;
            p36.servo11_raw_SET((ushort)(ushort)42442, PH) ;
            p36.servo6_raw = (ushort)(ushort)53301;
            p36.servo3_raw = (ushort)(ushort)50147;
            p36.servo1_raw = (ushort)(ushort)29568;
            p36.servo14_raw_SET((ushort)(ushort)39342, PH) ;
            p36.time_usec = (uint)2582748029U;
            p36.servo16_raw_SET((ushort)(ushort)50203, PH) ;
            p36.servo4_raw = (ushort)(ushort)40711;
            p36.port = (byte)(byte)92;
            p36.servo13_raw_SET((ushort)(ushort)18329, PH) ;
            p36.servo8_raw = (ushort)(ushort)9094;
            p36.servo12_raw_SET((ushort)(ushort)8316, PH) ;
            p36.servo7_raw = (ushort)(ushort)62678;
            p36.servo9_raw_SET((ushort)(ushort)27839, PH) ;
            p36.servo2_raw = (ushort)(ushort)37263;
            LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)10);
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.start_index == (short)(short)28795);
                Debug.Assert(pack.end_index == (short)(short)23895);
            };
            DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short)28795;
            p37.target_system = (byte)(byte)246;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.end_index = (short)(short)23895;
            p37.target_component = (byte)(byte)10;
            LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)201);
                Debug.Assert(pack.start_index == (short)(short) -29873);
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.end_index == (short)(short) -23612);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)201;
            p38.target_component = (byte)(byte)88;
            p38.start_index = (short)(short) -29873;
            p38.end_index = (short)(short) -23612;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)218);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_RELAY);
                Debug.Assert(pack.param4 == (float) -2.3435085E38F);
                Debug.Assert(pack.target_component == (byte)(byte)245);
                Debug.Assert(pack.seq == (ushort)(ushort)47037);
                Debug.Assert(pack.param2 == (float) -9.601857E36F);
                Debug.Assert(pack.x == (float)1.9449441E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)1);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.param1 == (float) -2.8426414E38F);
                Debug.Assert(pack.z == (float) -3.0801455E38F);
                Debug.Assert(pack.y == (float)2.0605492E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param3 == (float) -1.3088771E37F);
            };
            DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.param3 = (float) -1.3088771E37F;
            p39.param1 = (float) -2.8426414E38F;
            p39.z = (float) -3.0801455E38F;
            p39.autocontinue = (byte)(byte)1;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_RELAY;
            p39.x = (float)1.9449441E38F;
            p39.y = (float)2.0605492E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p39.current = (byte)(byte)218;
            p39.target_component = (byte)(byte)245;
            p39.param4 = (float) -2.3435085E38F;
            p39.param2 = (float) -9.601857E36F;
            p39.target_system = (byte)(byte)3;
            p39.seq = (ushort)(ushort)47037;
            LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.seq == (ushort)(ushort)17461);
                Debug.Assert(pack.target_system == (byte)(byte)84);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)17461;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.target_system = (byte)(byte)84;
            p40.target_component = (byte)(byte)185;
            LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.seq == (ushort)(ushort)59430);
            };
            DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)59430;
            p41.target_system = (byte)(byte)129;
            p41.target_component = (byte)(byte)149;
            LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)21090);
            };
            DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)21090;
            LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)198);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)217);
            };
            DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)217;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)198;
            LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)39432);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)10);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)39432;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_system = (byte)(byte)10;
            p44.target_component = (byte)(byte)228;
            LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.target_component == (byte)(byte)142);
            };
            DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)142;
            p45.target_system = (byte)(byte)77;
            LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)42037);
            };
            DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)42037;
            LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.target_system == (byte)(byte)62);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID);
            };
            DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p47.target_system = (byte)(byte)62;
            p47.target_component = (byte)(byte)152;
            LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)792620459);
                Debug.Assert(pack.altitude == (int) -701789056);
                Debug.Assert(pack.latitude == (int) -654892816);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2679464279193552115L);
                Debug.Assert(pack.target_system == (byte)(byte)96);
            };
            DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int) -654892816;
            p48.target_system = (byte)(byte)96;
            p48.time_usec_SET((ulong)2679464279193552115L, PH) ;
            p48.altitude = (int) -701789056;
            p48.longitude = (int)792620459;
            LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1373883962859098544L);
                Debug.Assert(pack.latitude == (int) -1585470198);
                Debug.Assert(pack.longitude == (int) -1255533987);
                Debug.Assert(pack.altitude == (int) -1552867884);
            };
            DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int) -1585470198;
            p49.longitude = (int) -1255533987;
            p49.time_usec_SET((ulong)1373883962859098544L, PH) ;
            p49.altitude = (int) -1552867884;
            LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vctch"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)131);
                Debug.Assert(pack.param_value_max == (float) -1.7838844E38F);
                Debug.Assert(pack.scale == (float)1.6300743E37F);
                Debug.Assert(pack.param_index == (short)(short) -19206);
                Debug.Assert(pack.param_value_min == (float)1.5135544E38F);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.target_component == (byte)(byte)65);
                Debug.Assert(pack.param_value0 == (float) -3.8722028E37F);
            };
            DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float)1.6300743E37F;
            p50.param_value_max = (float) -1.7838844E38F;
            p50.parameter_rc_channel_index = (byte)(byte)131;
            p50.param_id_SET("vctch", PH) ;
            p50.param_value_min = (float)1.5135544E38F;
            p50.param_value0 = (float) -3.8722028E37F;
            p50.target_component = (byte)(byte)65;
            p50.param_index = (short)(short) -19206;
            p50.target_system = (byte)(byte)45;
            LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.seq == (ushort)(ushort)21969);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)2);
            };
            DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)2;
            p51.target_component = (byte)(byte)244;
            p51.seq = (ushort)(ushort)21969;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)232);
                Debug.Assert(pack.p2z == (float)7.9397794E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p2x == (float) -2.5713388E38F);
                Debug.Assert(pack.p1y == (float)2.992541E38F);
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.p1z == (float) -1.196915E38F);
                Debug.Assert(pack.p1x == (float)2.0563203E38F);
                Debug.Assert(pack.p2y == (float) -2.9206258E38F);
            };
            DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_component = (byte)(byte)169;
            p54.p2x = (float) -2.5713388E38F;
            p54.target_system = (byte)(byte)232;
            p54.p1x = (float)2.0563203E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p54.p2y = (float) -2.9206258E38F;
            p54.p1y = (float)2.992541E38F;
            p54.p1z = (float) -1.196915E38F;
            p54.p2z = (float)7.9397794E37F;
            LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float) -3.1724153E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.p1z == (float) -2.700169E38F);
                Debug.Assert(pack.p2z == (float) -3.3901762E38F);
                Debug.Assert(pack.p2x == (float)2.6330622E38F);
                Debug.Assert(pack.p2y == (float) -9.376109E37F);
                Debug.Assert(pack.p1y == (float)3.8601112E37F);
            };
            DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float)3.8601112E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p55.p1z = (float) -2.700169E38F;
            p55.p2z = (float) -3.3901762E38F;
            p55.p1x = (float) -3.1724153E38F;
            p55.p2y = (float) -9.376109E37F;
            p55.p2x = (float)2.6330622E38F;
            LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)2.234757E38F);
                Debug.Assert(pack.rollspeed == (float)2.551746E38F);
                Debug.Assert(pack.time_usec == (ulong)1208494154794246995L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0338566E38F, -2.6872235E37F, 2.293936E38F, 2.1841894E38F}));
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.6859129E37F, -2.5316321E38F, -1.1625874E38F, -6.475421E37F, -1.7105454E38F, 6.978702E37F, -4.5777627E37F, -6.8058173E37F, 2.840733E38F}));
                Debug.Assert(pack.yawspeed == (float)2.5766739E38F);
            };
            DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {1.6859129E37F, -2.5316321E38F, -1.1625874E38F, -6.475421E37F, -1.7105454E38F, 6.978702E37F, -4.5777627E37F, -6.8058173E37F, 2.840733E38F}, 0) ;
            p61.q_SET(new float[] {2.0338566E38F, -2.6872235E37F, 2.293936E38F, 2.1841894E38F}, 0) ;
            p61.yawspeed = (float)2.5766739E38F;
            p61.pitchspeed = (float)2.234757E38F;
            p61.time_usec = (ulong)1208494154794246995L;
            p61.rollspeed = (float)2.551746E38F;
            LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_error == (float) -3.2406549E38F);
                Debug.Assert(pack.nav_pitch == (float) -2.6157512E38F);
                Debug.Assert(pack.xtrack_error == (float)1.7641419E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -10032);
                Debug.Assert(pack.nav_roll == (float)1.9294253E38F);
                Debug.Assert(pack.aspd_error == (float)3.1431936E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)32352);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)18921);
            };
            DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float)3.1431936E38F;
            p62.nav_roll = (float)1.9294253E38F;
            p62.nav_pitch = (float) -2.6157512E38F;
            p62.wp_dist = (ushort)(ushort)18921;
            p62.target_bearing = (short)(short) -10032;
            p62.alt_error = (float) -3.2406549E38F;
            p62.xtrack_error = (float)1.7641419E38F;
            p62.nav_bearing = (short)(short)32352;
            LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4532644289781156233L);
                Debug.Assert(pack.lon == (int)803343475);
                Debug.Assert(pack.lat == (int) -568906891);
                Debug.Assert(pack.relative_alt == (int) -1621323445);
                Debug.Assert(pack.alt == (int)1513414468);
                Debug.Assert(pack.vz == (float) -4.1211337E36F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.7484423E38F, 6.111923E37F, -3.3373172E38F, 1.5029744E38F, -3.1527564E38F, -1.2934535E38F, 2.9700145E38F, -3.3508044E38F, 1.5619357E38F, 2.930271E38F, -1.6017441E38F, 5.3577207E37F, -1.6689627E38F, 1.2319054E38F, -2.9006251E38F, 2.7149553E38F, -2.5463385E38F, 2.5146897E37F, 4.0085556E37F, -1.7962743E38F, -2.8059292E38F, 2.4045E38F, 2.1240179E37F, 1.6028742E38F, 1.4290116E38F, -2.9346844E38F, 1.2328344E38F, 2.061254E38F, 1.708966E38F, -1.7129162E38F, 2.2935663E38F, -2.478384E38F, 1.5258161E37F, -2.2740745E38F, -7.640153E37F, 3.1908733E38F}));
                Debug.Assert(pack.vy == (float) -1.7478141E37F);
                Debug.Assert(pack.vx == (float) -1.9252345E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
            };
            DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vz = (float) -4.1211337E36F;
            p63.vy = (float) -1.7478141E37F;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.lat = (int) -568906891;
            p63.vx = (float) -1.9252345E38F;
            p63.time_usec = (ulong)4532644289781156233L;
            p63.lon = (int)803343475;
            p63.alt = (int)1513414468;
            p63.covariance_SET(new float[] {-2.7484423E38F, 6.111923E37F, -3.3373172E38F, 1.5029744E38F, -3.1527564E38F, -1.2934535E38F, 2.9700145E38F, -3.3508044E38F, 1.5619357E38F, 2.930271E38F, -1.6017441E38F, 5.3577207E37F, -1.6689627E38F, 1.2319054E38F, -2.9006251E38F, 2.7149553E38F, -2.5463385E38F, 2.5146897E37F, 4.0085556E37F, -1.7962743E38F, -2.8059292E38F, 2.4045E38F, 2.1240179E37F, 1.6028742E38F, 1.4290116E38F, -2.9346844E38F, 1.2328344E38F, 2.061254E38F, 1.708966E38F, -1.7129162E38F, 2.2935663E38F, -2.478384E38F, 1.5258161E37F, -2.2740745E38F, -7.640153E37F, 3.1908733E38F}, 0) ;
            p63.relative_alt = (int) -1621323445;
            LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.2253241E38F, -3.2781848E38F, 1.7919322E38F, 8.58978E37F, -1.0583036E38F, 6.294825E36F, -2.2623708E38F, 3.0998736E38F, 4.0418535E37F, -1.8705268E38F, 3.119794E38F, -8.293395E37F, 1.6283937E38F, -4.394559E37F, 2.4947148E37F, 1.2391665E38F, 2.6687848E38F, -3.2593656E38F, 1.560172E37F, -5.6687545E37F, -1.1136322E38F, -1.1252114E38F, 3.0505403E38F, -1.270802E37F, -2.9527953E38F, 3.3547698E38F, -1.6372443E38F, -1.724629E38F, -4.068888E37F, 2.3754754E38F, -2.152481E38F, 1.0533902E38F, 3.2186016E38F, 6.248753E37F, -2.2164727E38F, -2.8847339E38F, -1.0125361E38F, 6.1075573E37F, 3.0236856E38F, -3.2088565E38F, -2.6042596E37F, -3.5476277E37F, 3.0074877E38F, -1.8598179E38F, -2.4297358E37F}));
                Debug.Assert(pack.x == (float) -6.022017E37F);
                Debug.Assert(pack.ax == (float) -1.9691273E38F);
                Debug.Assert(pack.vy == (float)2.5396832E37F);
                Debug.Assert(pack.time_usec == (ulong)4836991957385663499L);
                Debug.Assert(pack.ay == (float) -1.4629754E38F);
                Debug.Assert(pack.vx == (float) -1.5268443E38F);
                Debug.Assert(pack.z == (float)5.0143197E37F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.vz == (float) -1.9172536E38F);
                Debug.Assert(pack.az == (float)3.3473198E38F);
                Debug.Assert(pack.y == (float) -2.9519465E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p64.vz = (float) -1.9172536E38F;
            p64.ax = (float) -1.9691273E38F;
            p64.az = (float)3.3473198E38F;
            p64.covariance_SET(new float[] {-1.2253241E38F, -3.2781848E38F, 1.7919322E38F, 8.58978E37F, -1.0583036E38F, 6.294825E36F, -2.2623708E38F, 3.0998736E38F, 4.0418535E37F, -1.8705268E38F, 3.119794E38F, -8.293395E37F, 1.6283937E38F, -4.394559E37F, 2.4947148E37F, 1.2391665E38F, 2.6687848E38F, -3.2593656E38F, 1.560172E37F, -5.6687545E37F, -1.1136322E38F, -1.1252114E38F, 3.0505403E38F, -1.270802E37F, -2.9527953E38F, 3.3547698E38F, -1.6372443E38F, -1.724629E38F, -4.068888E37F, 2.3754754E38F, -2.152481E38F, 1.0533902E38F, 3.2186016E38F, 6.248753E37F, -2.2164727E38F, -2.8847339E38F, -1.0125361E38F, 6.1075573E37F, 3.0236856E38F, -3.2088565E38F, -2.6042596E37F, -3.5476277E37F, 3.0074877E38F, -1.8598179E38F, -2.4297358E37F}, 0) ;
            p64.ay = (float) -1.4629754E38F;
            p64.time_usec = (ulong)4836991957385663499L;
            p64.y = (float) -2.9519465E38F;
            p64.z = (float)5.0143197E37F;
            p64.vy = (float)2.5396832E37F;
            p64.vx = (float) -1.5268443E38F;
            p64.x = (float) -6.022017E37F;
            LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)55982);
                Debug.Assert(pack.time_boot_ms == (uint)1856703537U);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)44780);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)7376);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)16282);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)50186);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)2044);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)42592);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)47638);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)8025);
                Debug.Assert(pack.rssi == (byte)(byte)40);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)59779);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)5339);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)28211);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)61737);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)24615);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)38506);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22354);
                Debug.Assert(pack.chancount == (byte)(byte)223);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)40601);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)47829);
            };
            DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.chan3_raw = (ushort)(ushort)38506;
            p65.chan1_raw = (ushort)(ushort)2044;
            p65.chan6_raw = (ushort)(ushort)24615;
            p65.chan2_raw = (ushort)(ushort)55982;
            p65.chan5_raw = (ushort)(ushort)16282;
            p65.chan16_raw = (ushort)(ushort)42592;
            p65.rssi = (byte)(byte)40;
            p65.chan8_raw = (ushort)(ushort)40601;
            p65.chan7_raw = (ushort)(ushort)22354;
            p65.chan9_raw = (ushort)(ushort)47829;
            p65.chan17_raw = (ushort)(ushort)47638;
            p65.chan13_raw = (ushort)(ushort)8025;
            p65.chan14_raw = (ushort)(ushort)5339;
            p65.chan15_raw = (ushort)(ushort)44780;
            p65.chan18_raw = (ushort)(ushort)28211;
            p65.time_boot_ms = (uint)1856703537U;
            p65.chan11_raw = (ushort)(ushort)59779;
            p65.chan12_raw = (ushort)(ushort)61737;
            p65.chan4_raw = (ushort)(ushort)7376;
            p65.chan10_raw = (ushort)(ushort)50186;
            p65.chancount = (byte)(byte)223;
            LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)73);
                Debug.Assert(pack.start_stop == (byte)(byte)217);
                Debug.Assert(pack.req_stream_id == (byte)(byte)12);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)2073);
                Debug.Assert(pack.target_system == (byte)(byte)111);
            };
            DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)217;
            p66.req_stream_id = (byte)(byte)12;
            p66.req_message_rate = (ushort)(ushort)2073;
            p66.target_component = (byte)(byte)73;
            p66.target_system = (byte)(byte)111;
            LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)166);
                Debug.Assert(pack.on_off == (byte)(byte)68);
                Debug.Assert(pack.message_rate == (ushort)(ushort)45982);
            };
            DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)166;
            p67.message_rate = (ushort)(ushort)45982;
            p67.on_off = (byte)(byte)68;
            LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short) -7734);
                Debug.Assert(pack.buttons == (ushort)(ushort)13310);
                Debug.Assert(pack.x == (short)(short) -20961);
                Debug.Assert(pack.z == (short)(short) -531);
                Debug.Assert(pack.target == (byte)(byte)99);
                Debug.Assert(pack.r == (short)(short)30688);
            };
            DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)13310;
            p69.z = (short)(short) -531;
            p69.y = (short)(short) -7734;
            p69.target = (byte)(byte)99;
            p69.r = (short)(short)30688;
            p69.x = (short)(short) -20961;
            LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)39816);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)6219);
                Debug.Assert(pack.target_system == (byte)(byte)14);
                Debug.Assert(pack.target_component == (byte)(byte)43);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7939);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)55266);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)5362);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)50318);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)25317);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)49190);
            };
            DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan1_raw = (ushort)(ushort)39816;
            p70.chan8_raw = (ushort)(ushort)49190;
            p70.chan3_raw = (ushort)(ushort)50318;
            p70.target_component = (byte)(byte)43;
            p70.chan5_raw = (ushort)(ushort)7939;
            p70.chan2_raw = (ushort)(ushort)5362;
            p70.target_system = (byte)(byte)14;
            p70.chan6_raw = (ushort)(ushort)6219;
            p70.chan7_raw = (ushort)(ushort)25317;
            p70.chan4_raw = (ushort)(ushort)55266;
            LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)1267379757);
                Debug.Assert(pack.seq == (ushort)(ushort)30769);
                Debug.Assert(pack.param2 == (float)2.140516E38F);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.current == (byte)(byte)31);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.z == (float) -1.0351194E38F);
                Debug.Assert(pack.param3 == (float) -3.07548E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)217);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
                Debug.Assert(pack.param4 == (float) -1.4017053E38F);
                Debug.Assert(pack.param1 == (float)1.5820782E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.y == (int) -1780636086);
                Debug.Assert(pack.target_system == (byte)(byte)219);
            };
            DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.z = (float) -1.0351194E38F;
            p73.x = (int)1267379757;
            p73.param2 = (float)2.140516E38F;
            p73.target_component = (byte)(byte)79;
            p73.param1 = (float)1.5820782E38F;
            p73.param4 = (float) -1.4017053E38F;
            p73.y = (int) -1780636086;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p73.current = (byte)(byte)31;
            p73.target_system = (byte)(byte)219;
            p73.param3 = (float) -3.07548E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            p73.autocontinue = (byte)(byte)217;
            p73.seq = (ushort)(ushort)30769;
            LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float) -5.207902E37F);
                Debug.Assert(pack.alt == (float) -4.739062E37F);
                Debug.Assert(pack.climb == (float)3.1995566E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)27740);
                Debug.Assert(pack.groundspeed == (float)1.1176254E38F);
                Debug.Assert(pack.heading == (short)(short) -21053);
            };
            DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -5.207902E37F;
            p74.climb = (float)3.1995566E38F;
            p74.throttle = (ushort)(ushort)27740;
            p74.heading = (short)(short) -21053;
            p74.groundspeed = (float)1.1176254E38F;
            p74.alt = (float) -4.739062E37F;
            LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.x == (int)300380090);
                Debug.Assert(pack.param1 == (float)8.425792E37F);
                Debug.Assert(pack.param3 == (float) -1.7075254E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_TIME);
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.param4 == (float) -2.0143794E38F);
                Debug.Assert(pack.param2 == (float) -2.2051198E38F);
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.autocontinue == (byte)(byte)26);
                Debug.Assert(pack.z == (float)8.4083183E37F);
                Debug.Assert(pack.y == (int) -540044192);
                Debug.Assert(pack.current == (byte)(byte)13);
            };
            DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.param3 = (float) -1.7075254E38F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.autocontinue = (byte)(byte)26;
            p75.z = (float)8.4083183E37F;
            p75.param1 = (float)8.425792E37F;
            p75.current = (byte)(byte)13;
            p75.x = (int)300380090;
            p75.target_system = (byte)(byte)128;
            p75.y = (int) -540044192;
            p75.param4 = (float) -2.0143794E38F;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LOITER_TIME;
            p75.target_component = (byte)(byte)248;
            p75.param2 = (float) -2.2051198E38F;
            LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)1.725837E38F);
                Debug.Assert(pack.param7 == (float) -2.9971361E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)55);
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.param6 == (float)2.826876E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_WAYPOINT);
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.param1 == (float)2.9415925E38F);
                Debug.Assert(pack.param5 == (float) -1.314874E37F);
                Debug.Assert(pack.param4 == (float)2.0517438E38F);
                Debug.Assert(pack.param2 == (float) -8.658875E37F);
            };
            DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param3 = (float)1.725837E38F;
            p76.param1 = (float)2.9415925E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_WAYPOINT;
            p76.param2 = (float) -8.658875E37F;
            p76.confirmation = (byte)(byte)55;
            p76.param6 = (float)2.826876E38F;
            p76.param7 = (float) -2.9971361E38F;
            p76.target_system = (byte)(byte)180;
            p76.param5 = (float) -1.314874E37F;
            p76.param4 = (float)2.0517438E38F;
            p76.target_component = (byte)(byte)187;
            LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int)88778637);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAND);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)240);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)143);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)170);
            };
            DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)240, PH) ;
            p77.target_system_SET((byte)(byte)170, PH) ;
            p77.target_component_SET((byte)(byte)143, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAND;
            p77.result_param2_SET((int)88778637, PH) ;
            LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.5276124E38F);
                Debug.Assert(pack.pitch == (float)3.0823963E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)143);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)152);
                Debug.Assert(pack.time_boot_ms == (uint)3476200463U);
                Debug.Assert(pack.yaw == (float)2.10601E38F);
                Debug.Assert(pack.thrust == (float)2.0229917E38F);
            };
            DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.thrust = (float)2.0229917E38F;
            p81.yaw = (float)2.10601E38F;
            p81.time_boot_ms = (uint)3476200463U;
            p81.roll = (float) -1.5276124E38F;
            p81.pitch = (float)3.0823963E38F;
            p81.manual_override_switch = (byte)(byte)152;
            p81.mode_switch = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)7.2981054E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8689517E38F, 9.894328E37F, -1.0558187E38F, 2.8711998E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.thrust == (float) -2.865032E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.8379307E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -2.5110739E38F);
                Debug.Assert(pack.target_component == (byte)(byte)125);
                Debug.Assert(pack.type_mask == (byte)(byte)127);
                Debug.Assert(pack.time_boot_ms == (uint)3504550407U);
            };
            DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.type_mask = (byte)(byte)127;
            p82.body_yaw_rate = (float)7.2981054E37F;
            p82.body_pitch_rate = (float) -2.5110739E38F;
            p82.thrust = (float) -2.865032E38F;
            p82.body_roll_rate = (float)1.8379307E38F;
            p82.q_SET(new float[] {-2.8689517E38F, 9.894328E37F, -1.0558187E38F, 2.8711998E38F}, 0) ;
            p82.target_system = (byte)(byte)192;
            p82.time_boot_ms = (uint)3504550407U;
            p82.target_component = (byte)(byte)125;
            LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float)2.581958E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1599096968U);
                Debug.Assert(pack.body_pitch_rate == (float)9.510742E37F);
                Debug.Assert(pack.body_roll_rate == (float)7.42279E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9376142E37F, 1.0507248E38F, -3.61555E37F, -7.744436E37F}));
                Debug.Assert(pack.body_yaw_rate == (float)1.3156563E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)248);
            };
            DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float)9.510742E37F;
            p83.q_SET(new float[] {2.9376142E37F, 1.0507248E38F, -3.61555E37F, -7.744436E37F}, 0) ;
            p83.body_roll_rate = (float)7.42279E37F;
            p83.time_boot_ms = (uint)1599096968U;
            p83.thrust = (float)2.581958E38F;
            p83.body_yaw_rate = (float)1.3156563E38F;
            p83.type_mask = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -4.690977E37F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.x == (float) -1.9465223E38F);
                Debug.Assert(pack.y == (float) -3.0347612E38F);
                Debug.Assert(pack.yaw == (float)3.0291939E38F);
                Debug.Assert(pack.afz == (float) -2.974393E38F);
                Debug.Assert(pack.z == (float) -2.967773E38F);
                Debug.Assert(pack.vy == (float) -2.4993596E38F);
                Debug.Assert(pack.afx == (float)3.265811E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2901727135U);
                Debug.Assert(pack.afy == (float) -2.1349546E38F);
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.type_mask == (ushort)(ushort)55095);
                Debug.Assert(pack.vx == (float)1.8562137E38F);
                Debug.Assert(pack.yaw_rate == (float)1.4948958E38F);
            };
            DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afx = (float)3.265811E38F;
            p84.y = (float) -3.0347612E38F;
            p84.z = (float) -2.967773E38F;
            p84.x = (float) -1.9465223E38F;
            p84.time_boot_ms = (uint)2901727135U;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p84.type_mask = (ushort)(ushort)55095;
            p84.yaw_rate = (float)1.4948958E38F;
            p84.afy = (float) -2.1349546E38F;
            p84.target_component = (byte)(byte)224;
            p84.afz = (float) -2.974393E38F;
            p84.target_system = (byte)(byte)240;
            p84.vy = (float) -2.4993596E38F;
            p84.vx = (float)1.8562137E38F;
            p84.yaw = (float)3.0291939E38F;
            p84.vz = (float) -4.690977E37F;
            LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)2.3007272E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.1680292E38F);
                Debug.Assert(pack.lat_int == (int) -805881724);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)176);
                Debug.Assert(pack.lon_int == (int)450762323);
                Debug.Assert(pack.vz == (float)2.3406199E38F);
                Debug.Assert(pack.vx == (float) -1.3753939E37F);
                Debug.Assert(pack.afz == (float) -1.3517947E38F);
                Debug.Assert(pack.yaw == (float) -3.2494927E38F);
                Debug.Assert(pack.vy == (float)2.521094E38F);
                Debug.Assert(pack.target_system == (byte)(byte)89);
                Debug.Assert(pack.time_boot_ms == (uint)3972908397U);
                Debug.Assert(pack.afy == (float) -2.0995314E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)56760);
                Debug.Assert(pack.alt == (float)3.2634454E38F);
            };
            DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afx = (float)2.3007272E38F;
            p86.alt = (float)3.2634454E38F;
            p86.afz = (float) -1.3517947E38F;
            p86.yaw = (float) -3.2494927E38F;
            p86.afy = (float) -2.0995314E38F;
            p86.lon_int = (int)450762323;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p86.vy = (float)2.521094E38F;
            p86.lat_int = (int) -805881724;
            p86.vz = (float)2.3406199E38F;
            p86.vx = (float) -1.3753939E37F;
            p86.target_system = (byte)(byte)89;
            p86.yaw_rate = (float) -2.1680292E38F;
            p86.target_component = (byte)(byte)176;
            p86.type_mask = (ushort)(ushort)56760;
            p86.time_boot_ms = (uint)3972908397U;
            LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -1.4787514E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1529951936U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.afz == (float)1.5750515E38F);
                Debug.Assert(pack.yaw_rate == (float)1.6131069E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)63485);
                Debug.Assert(pack.afy == (float) -1.0939239E38F);
                Debug.Assert(pack.alt == (float)2.3232825E38F);
                Debug.Assert(pack.vx == (float)1.2803464E38F);
                Debug.Assert(pack.vy == (float)2.8643938E38F);
                Debug.Assert(pack.yaw == (float)1.3198394E38F);
                Debug.Assert(pack.lat_int == (int)311714897);
                Debug.Assert(pack.lon_int == (int) -1313563138);
                Debug.Assert(pack.afx == (float) -2.8207842E38F);
            };
            DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float) -1.4787514E38F;
            p87.vx = (float)1.2803464E38F;
            p87.yaw = (float)1.3198394E38F;
            p87.lat_int = (int)311714897;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p87.afx = (float) -2.8207842E38F;
            p87.type_mask = (ushort)(ushort)63485;
            p87.lon_int = (int) -1313563138;
            p87.yaw_rate = (float)1.6131069E38F;
            p87.afy = (float) -1.0939239E38F;
            p87.alt = (float)2.3232825E38F;
            p87.time_boot_ms = (uint)1529951936U;
            p87.vy = (float)2.8643938E38F;
            p87.afz = (float)1.5750515E38F;
            LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.6762169E38F);
                Debug.Assert(pack.roll == (float) -2.257954E38F);
                Debug.Assert(pack.yaw == (float) -2.3449538E38F);
                Debug.Assert(pack.z == (float) -4.0125566E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2679487988U);
                Debug.Assert(pack.pitch == (float)9.647213E35F);
                Debug.Assert(pack.y == (float)2.6152313E38F);
            };
            DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float)2.6762169E38F;
            p89.roll = (float) -2.257954E38F;
            p89.z = (float) -4.0125566E37F;
            p89.y = (float)2.6152313E38F;
            p89.time_boot_ms = (uint)2679487988U;
            p89.pitch = (float)9.647213E35F;
            p89.yaw = (float) -2.3449538E38F;
            LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.4303048E38F);
                Debug.Assert(pack.xacc == (short)(short)21036);
                Debug.Assert(pack.roll == (float) -9.341951E37F);
                Debug.Assert(pack.alt == (int)1778908432);
                Debug.Assert(pack.rollspeed == (float)8.508674E37F);
                Debug.Assert(pack.vz == (short)(short) -4708);
                Debug.Assert(pack.vy == (short)(short) -10593);
                Debug.Assert(pack.lon == (int)579405982);
                Debug.Assert(pack.lat == (int) -1076690266);
                Debug.Assert(pack.zacc == (short)(short) -16137);
                Debug.Assert(pack.yawspeed == (float)1.5929392E38F);
                Debug.Assert(pack.pitch == (float)1.7230442E38F);
                Debug.Assert(pack.time_usec == (ulong)1621654384779040034L);
                Debug.Assert(pack.vx == (short)(short) -18566);
                Debug.Assert(pack.yacc == (short)(short)18615);
                Debug.Assert(pack.pitchspeed == (float) -1.4735974E38F);
            };
            DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yaw = (float)2.4303048E38F;
            p90.time_usec = (ulong)1621654384779040034L;
            p90.pitchspeed = (float) -1.4735974E38F;
            p90.xacc = (short)(short)21036;
            p90.zacc = (short)(short) -16137;
            p90.lon = (int)579405982;
            p90.vx = (short)(short) -18566;
            p90.rollspeed = (float)8.508674E37F;
            p90.vy = (short)(short) -10593;
            p90.yawspeed = (float)1.5929392E38F;
            p90.yacc = (short)(short)18615;
            p90.alt = (int)1778908432;
            p90.roll = (float) -9.341951E37F;
            p90.lat = (int) -1076690266;
            p90.vz = (short)(short) -4708;
            p90.pitch = (float)1.7230442E38F;
            LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rudder == (float) -1.3177967E37F);
                Debug.Assert(pack.throttle == (float) -4.1556888E37F);
                Debug.Assert(pack.aux2 == (float)9.540502E37F);
                Debug.Assert(pack.aux4 == (float)1.4202955E38F);
                Debug.Assert(pack.roll_ailerons == (float) -1.9320963E38F);
                Debug.Assert(pack.pitch_elevator == (float)1.3894469E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.nav_mode == (byte)(byte)43);
                Debug.Assert(pack.aux1 == (float)7.569202E37F);
                Debug.Assert(pack.time_usec == (ulong)4927371951021457760L);
                Debug.Assert(pack.aux3 == (float) -3.2366967E38F);
            };
            DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)4927371951021457760L;
            p91.pitch_elevator = (float)1.3894469E38F;
            p91.nav_mode = (byte)(byte)43;
            p91.aux2 = (float)9.540502E37F;
            p91.aux3 = (float) -3.2366967E38F;
            p91.throttle = (float) -4.1556888E37F;
            p91.aux4 = (float)1.4202955E38F;
            p91.roll_ailerons = (float) -1.9320963E38F;
            p91.yaw_rudder = (float) -1.3177967E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.aux1 = (float)7.569202E37F;
            LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)63854);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)57396);
                Debug.Assert(pack.rssi == (byte)(byte)29);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)18048);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)45052);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)46541);
                Debug.Assert(pack.time_usec == (ulong)6625853884991761031L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)49447);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)25627);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)25294);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)59324);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)13430);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)34108);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)48603);
            };
            DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan10_raw = (ushort)(ushort)57396;
            p92.chan2_raw = (ushort)(ushort)59324;
            p92.chan11_raw = (ushort)(ushort)34108;
            p92.chan9_raw = (ushort)(ushort)25627;
            p92.chan3_raw = (ushort)(ushort)63854;
            p92.rssi = (byte)(byte)29;
            p92.chan12_raw = (ushort)(ushort)46541;
            p92.time_usec = (ulong)6625853884991761031L;
            p92.chan4_raw = (ushort)(ushort)18048;
            p92.chan6_raw = (ushort)(ushort)25294;
            p92.chan5_raw = (ushort)(ushort)49447;
            p92.chan7_raw = (ushort)(ushort)48603;
            p92.chan8_raw = (ushort)(ushort)45052;
            p92.chan1_raw = (ushort)(ushort)13430;
            LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)7221820245517531414L);
                Debug.Assert(pack.time_usec == (ulong)615432803289663468L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.8982379E38F, 1.5671568E38F, 2.35309E38F, -1.6817057E38F, 3.1587151E38F, -1.5541156E38F, 3.4839942E37F, -2.1929702E38F, -1.869076E38F, -1.7091989E38F, -2.3442841E38F, -2.0835837E38F, -7.304575E37F, -5.119144E37F, 7.8124835E37F, -3.195153E38F}));
            };
            DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)7221820245517531414L;
            p93.controls_SET(new float[] {-2.8982379E38F, 1.5671568E38F, 2.35309E38F, -1.6817057E38F, 3.1587151E38F, -1.5541156E38F, 3.4839942E37F, -2.1929702E38F, -1.869076E38F, -1.7091989E38F, -2.3442841E38F, -2.0835837E38F, -7.304575E37F, -5.119144E37F, 7.8124835E37F, -3.195153E38F}, 0) ;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p93.time_usec = (ulong)615432803289663468L;
            LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)206);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.3471957E38F);
                Debug.Assert(pack.time_usec == (ulong)7479649751240469780L);
                Debug.Assert(pack.flow_x == (short)(short)12934);
                Debug.Assert(pack.sensor_id == (byte)(byte)21);
                Debug.Assert(pack.flow_comp_m_x == (float)1.4292284E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.2415825E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)1.4386809E38F);
                Debug.Assert(pack.ground_distance == (float)2.1445675E38F);
                Debug.Assert(pack.flow_y == (short)(short)23555);
            };
            DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)7479649751240469780L;
            p100.ground_distance = (float)2.1445675E38F;
            p100.flow_y = (short)(short)23555;
            p100.flow_rate_y_SET((float)1.2415825E38F, PH) ;
            p100.quality = (byte)(byte)206;
            p100.flow_rate_x_SET((float)1.3471957E38F, PH) ;
            p100.flow_comp_m_y = (float)1.4386809E38F;
            p100.flow_comp_m_x = (float)1.4292284E38F;
            p100.sensor_id = (byte)(byte)21;
            p100.flow_x = (short)(short)12934;
            LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.7800719E38F);
                Debug.Assert(pack.pitch == (float)7.3226364E37F);
                Debug.Assert(pack.y == (float) -2.274705E38F);
                Debug.Assert(pack.yaw == (float)1.71768E38F);
                Debug.Assert(pack.z == (float)3.3661105E38F);
                Debug.Assert(pack.usec == (ulong)8269292088783149775L);
                Debug.Assert(pack.roll == (float) -2.2114305E38F);
            };
            DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.pitch = (float)7.3226364E37F;
            p101.z = (float)3.3661105E38F;
            p101.usec = (ulong)8269292088783149775L;
            p101.x = (float)1.7800719E38F;
            p101.y = (float) -2.274705E38F;
            p101.yaw = (float)1.71768E38F;
            p101.roll = (float) -2.2114305E38F;
            LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)9207396841379981198L);
                Debug.Assert(pack.x == (float) -2.9028318E38F);
                Debug.Assert(pack.y == (float)9.118403E37F);
                Debug.Assert(pack.z == (float)1.1568544E38F);
                Debug.Assert(pack.roll == (float) -1.0014114E37F);
                Debug.Assert(pack.yaw == (float) -2.712624E38F);
                Debug.Assert(pack.pitch == (float)5.6713334E37F);
            };
            DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.z = (float)1.1568544E38F;
            p102.y = (float)9.118403E37F;
            p102.yaw = (float) -2.712624E38F;
            p102.usec = (ulong)9207396841379981198L;
            p102.x = (float) -2.9028318E38F;
            p102.pitch = (float)5.6713334E37F;
            p102.roll = (float) -1.0014114E37F;
            LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)8.394738E37F);
                Debug.Assert(pack.y == (float) -1.9599694E38F);
                Debug.Assert(pack.usec == (ulong)5983536746367687093L);
                Debug.Assert(pack.z == (float)2.7176622E38F);
            };
            DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)5983536746367687093L;
            p103.x = (float)8.394738E37F;
            p103.y = (float) -1.9599694E38F;
            p103.z = (float)2.7176622E38F;
            LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)7190017352614208059L);
                Debug.Assert(pack.y == (float) -1.2682308E38F);
                Debug.Assert(pack.x == (float)4.2977787E37F);
                Debug.Assert(pack.yaw == (float)2.0624451E37F);
                Debug.Assert(pack.pitch == (float)3.220877E38F);
                Debug.Assert(pack.roll == (float) -2.4549839E38F);
                Debug.Assert(pack.z == (float) -2.5445258E38F);
            };
            DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float) -2.5445258E38F;
            p104.pitch = (float)3.220877E38F;
            p104.roll = (float) -2.4549839E38F;
            p104.y = (float) -1.2682308E38F;
            p104.x = (float)4.2977787E37F;
            p104.usec = (ulong)7190017352614208059L;
            p104.yaw = (float)2.0624451E37F;
            LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)2.3048784E38F);
                Debug.Assert(pack.yacc == (float) -2.6494257E38F);
                Debug.Assert(pack.zmag == (float)2.5472007E38F);
                Debug.Assert(pack.time_usec == (ulong)6158329315805585593L);
                Debug.Assert(pack.temperature == (float)3.3223507E37F);
                Debug.Assert(pack.ymag == (float)8.4270293E37F);
                Debug.Assert(pack.xgyro == (float)2.1515225E38F);
                Debug.Assert(pack.xmag == (float) -2.0987921E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)6940);
                Debug.Assert(pack.diff_pressure == (float)6.827003E37F);
                Debug.Assert(pack.zgyro == (float)1.951483E38F);
                Debug.Assert(pack.pressure_alt == (float) -9.976688E37F);
                Debug.Assert(pack.ygyro == (float)2.9704862E37F);
                Debug.Assert(pack.zacc == (float) -8.120714E37F);
                Debug.Assert(pack.xacc == (float) -7.96464E37F);
            };
            DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xmag = (float) -2.0987921E38F;
            p105.fields_updated = (ushort)(ushort)6940;
            p105.time_usec = (ulong)6158329315805585593L;
            p105.zacc = (float) -8.120714E37F;
            p105.abs_pressure = (float)2.3048784E38F;
            p105.yacc = (float) -2.6494257E38F;
            p105.zgyro = (float)1.951483E38F;
            p105.pressure_alt = (float) -9.976688E37F;
            p105.ygyro = (float)2.9704862E37F;
            p105.diff_pressure = (float)6.827003E37F;
            p105.xgyro = (float)2.1515225E38F;
            p105.zmag = (float)2.5472007E38F;
            p105.xacc = (float) -7.96464E37F;
            p105.ymag = (float)8.4270293E37F;
            p105.temperature = (float)3.3223507E37F;
            LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float)8.3729047E37F);
                Debug.Assert(pack.integrated_x == (float)1.4282714E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)457737501U);
                Debug.Assert(pack.time_usec == (ulong)8832339699451200429L);
                Debug.Assert(pack.distance == (float)1.7872567E38F);
                Debug.Assert(pack.integrated_y == (float) -3.2693574E37F);
                Debug.Assert(pack.integrated_xgyro == (float)1.7382187E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.206125E38F);
                Debug.Assert(pack.integration_time_us == (uint)3533112473U);
                Debug.Assert(pack.sensor_id == (byte)(byte)238);
                Debug.Assert(pack.temperature == (short)(short)11617);
                Debug.Assert(pack.quality == (byte)(byte)230);
            };
            DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_ygyro = (float)8.3729047E37F;
            p106.temperature = (short)(short)11617;
            p106.time_delta_distance_us = (uint)457737501U;
            p106.integrated_zgyro = (float) -2.206125E38F;
            p106.time_usec = (ulong)8832339699451200429L;
            p106.integration_time_us = (uint)3533112473U;
            p106.distance = (float)1.7872567E38F;
            p106.sensor_id = (byte)(byte)238;
            p106.integrated_y = (float) -3.2693574E37F;
            p106.integrated_x = (float)1.4282714E38F;
            p106.quality = (byte)(byte)230;
            p106.integrated_xgyro = (float)1.7382187E38F;
            LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (float)8.4572684E37F);
                Debug.Assert(pack.zmag == (float)2.8330583E38F);
                Debug.Assert(pack.time_usec == (ulong)2629742666484002090L);
                Debug.Assert(pack.pressure_alt == (float)6.363537E37F);
                Debug.Assert(pack.zgyro == (float)1.6708013E38F);
                Debug.Assert(pack.xmag == (float) -8.769162E36F);
                Debug.Assert(pack.abs_pressure == (float)1.7275005E38F);
                Debug.Assert(pack.yacc == (float)8.706644E37F);
                Debug.Assert(pack.xgyro == (float) -1.900255E38F);
                Debug.Assert(pack.fields_updated == (uint)895233736U);
                Debug.Assert(pack.ygyro == (float)9.928546E37F);
                Debug.Assert(pack.diff_pressure == (float)1.86738E38F);
                Debug.Assert(pack.xacc == (float)2.2005932E38F);
                Debug.Assert(pack.zacc == (float) -4.4281865E37F);
                Debug.Assert(pack.ymag == (float)2.9273099E38F);
            };
            DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.fields_updated = (uint)895233736U;
            p107.zmag = (float)2.8330583E38F;
            p107.diff_pressure = (float)1.86738E38F;
            p107.time_usec = (ulong)2629742666484002090L;
            p107.xmag = (float) -8.769162E36F;
            p107.xacc = (float)2.2005932E38F;
            p107.xgyro = (float) -1.900255E38F;
            p107.pressure_alt = (float)6.363537E37F;
            p107.yacc = (float)8.706644E37F;
            p107.temperature = (float)8.4572684E37F;
            p107.ygyro = (float)9.928546E37F;
            p107.zacc = (float) -4.4281865E37F;
            p107.zgyro = (float)1.6708013E38F;
            p107.ymag = (float)2.9273099E38F;
            p107.abs_pressure = (float)1.7275005E38F;
            LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)2.0022582E38F);
                Debug.Assert(pack.std_dev_vert == (float)2.3290123E38F);
                Debug.Assert(pack.roll == (float) -2.7911668E38F);
                Debug.Assert(pack.vn == (float) -2.1455201E38F);
                Debug.Assert(pack.q4 == (float)9.400141E37F);
                Debug.Assert(pack.std_dev_horz == (float) -3.2592988E38F);
                Debug.Assert(pack.yacc == (float) -3.317843E38F);
                Debug.Assert(pack.lat == (float) -3.0814162E38F);
                Debug.Assert(pack.yaw == (float) -3.2680783E38F);
                Debug.Assert(pack.vd == (float) -2.1218267E37F);
                Debug.Assert(pack.xgyro == (float)3.1950578E38F);
                Debug.Assert(pack.q3 == (float)1.0828152E37F);
                Debug.Assert(pack.lon == (float) -1.433351E38F);
                Debug.Assert(pack.pitch == (float) -1.5221743E38F);
                Debug.Assert(pack.ve == (float) -3.2332224E38F);
                Debug.Assert(pack.alt == (float)1.8897038E38F);
                Debug.Assert(pack.q1 == (float) -2.2798071E38F);
                Debug.Assert(pack.zacc == (float)1.7695303E38F);
                Debug.Assert(pack.q2 == (float)1.033431E38F);
                Debug.Assert(pack.zgyro == (float)2.1291735E38F);
                Debug.Assert(pack.ygyro == (float) -1.7633239E38F);
            };
            DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.pitch = (float) -1.5221743E38F;
            p108.alt = (float)1.8897038E38F;
            p108.q3 = (float)1.0828152E37F;
            p108.q4 = (float)9.400141E37F;
            p108.std_dev_horz = (float) -3.2592988E38F;
            p108.yaw = (float) -3.2680783E38F;
            p108.lat = (float) -3.0814162E38F;
            p108.xgyro = (float)3.1950578E38F;
            p108.lon = (float) -1.433351E38F;
            p108.vn = (float) -2.1455201E38F;
            p108.q1 = (float) -2.2798071E38F;
            p108.xacc = (float)2.0022582E38F;
            p108.zacc = (float)1.7695303E38F;
            p108.yacc = (float) -3.317843E38F;
            p108.std_dev_vert = (float)2.3290123E38F;
            p108.zgyro = (float)2.1291735E38F;
            p108.ve = (float) -3.2332224E38F;
            p108.q2 = (float)1.033431E38F;
            p108.roll = (float) -2.7911668E38F;
            p108.ygyro = (float) -1.7633239E38F;
            p108.vd = (float) -2.1218267E37F;
            LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.noise == (byte)(byte)205);
                Debug.Assert(pack.remnoise == (byte)(byte)125);
                Debug.Assert(pack.remrssi == (byte)(byte)31);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)29804);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)31969);
                Debug.Assert(pack.rssi == (byte)(byte)252);
                Debug.Assert(pack.txbuf == (byte)(byte)138);
            };
            DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)31;
            p109.remnoise = (byte)(byte)125;
            p109.rssi = (byte)(byte)252;
            p109.txbuf = (byte)(byte)138;
            p109.noise = (byte)(byte)205;
            p109.fixed_ = (ushort)(ushort)31969;
            p109.rxerrors = (ushort)(ushort)29804;
            LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.target_component == (byte)(byte)110);
                Debug.Assert(pack.target_network == (byte)(byte)56);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)76, (byte)177, (byte)81, (byte)190, (byte)253, (byte)64, (byte)27, (byte)250, (byte)113, (byte)113, (byte)214, (byte)6, (byte)57, (byte)167, (byte)153, (byte)179, (byte)134, (byte)50, (byte)92, (byte)27, (byte)73, (byte)18, (byte)6, (byte)61, (byte)198, (byte)241, (byte)20, (byte)249, (byte)86, (byte)67, (byte)10, (byte)110, (byte)106, (byte)174, (byte)169, (byte)205, (byte)239, (byte)168, (byte)41, (byte)133, (byte)159, (byte)213, (byte)211, (byte)90, (byte)240, (byte)243, (byte)5, (byte)190, (byte)226, (byte)70, (byte)216, (byte)123, (byte)222, (byte)236, (byte)205, (byte)179, (byte)67, (byte)136, (byte)84, (byte)193, (byte)195, (byte)237, (byte)142, (byte)15, (byte)90, (byte)251, (byte)160, (byte)95, (byte)8, (byte)82, (byte)166, (byte)83, (byte)117, (byte)151, (byte)152, (byte)26, (byte)140, (byte)186, (byte)154, (byte)1, (byte)24, (byte)142, (byte)197, (byte)239, (byte)248, (byte)130, (byte)173, (byte)122, (byte)61, (byte)84, (byte)69, (byte)136, (byte)78, (byte)227, (byte)46, (byte)14, (byte)229, (byte)177, (byte)147, (byte)2, (byte)49, (byte)70, (byte)123, (byte)27, (byte)127, (byte)78, (byte)112, (byte)245, (byte)170, (byte)108, (byte)110, (byte)176, (byte)243, (byte)123, (byte)119, (byte)63, (byte)29, (byte)219, (byte)69, (byte)251, (byte)103, (byte)20, (byte)173, (byte)126, (byte)125, (byte)72, (byte)120, (byte)115, (byte)156, (byte)28, (byte)17, (byte)70, (byte)2, (byte)227, (byte)210, (byte)170, (byte)173, (byte)117, (byte)107, (byte)252, (byte)219, (byte)207, (byte)113, (byte)71, (byte)150, (byte)50, (byte)61, (byte)231, (byte)185, (byte)92, (byte)161, (byte)231, (byte)244, (byte)90, (byte)109, (byte)239, (byte)126, (byte)209, (byte)127, (byte)203, (byte)47, (byte)49, (byte)195, (byte)34, (byte)137, (byte)141, (byte)201, (byte)94, (byte)121, (byte)67, (byte)106, (byte)114, (byte)238, (byte)24, (byte)145, (byte)135, (byte)105, (byte)157, (byte)230, (byte)244, (byte)15, (byte)63, (byte)137, (byte)216, (byte)44, (byte)210, (byte)202, (byte)214, (byte)28, (byte)154, (byte)162, (byte)89, (byte)79, (byte)37, (byte)150, (byte)84, (byte)203, (byte)54, (byte)193, (byte)40, (byte)97, (byte)12, (byte)80, (byte)241, (byte)162, (byte)93, (byte)165, (byte)113, (byte)198, (byte)27, (byte)7, (byte)94, (byte)64, (byte)206, (byte)31, (byte)179, (byte)41, (byte)115, (byte)51, (byte)47, (byte)140, (byte)21, (byte)181, (byte)225, (byte)130, (byte)164, (byte)6, (byte)89, (byte)113, (byte)116, (byte)33, (byte)206, (byte)64, (byte)221, (byte)185, (byte)221, (byte)39, (byte)136, (byte)221, (byte)157, (byte)185, (byte)189, (byte)248, (byte)240, (byte)248, (byte)245, (byte)236, (byte)145, (byte)217, (byte)84, (byte)20}));
            };
            DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)56;
            p110.payload_SET(new byte[] {(byte)76, (byte)177, (byte)81, (byte)190, (byte)253, (byte)64, (byte)27, (byte)250, (byte)113, (byte)113, (byte)214, (byte)6, (byte)57, (byte)167, (byte)153, (byte)179, (byte)134, (byte)50, (byte)92, (byte)27, (byte)73, (byte)18, (byte)6, (byte)61, (byte)198, (byte)241, (byte)20, (byte)249, (byte)86, (byte)67, (byte)10, (byte)110, (byte)106, (byte)174, (byte)169, (byte)205, (byte)239, (byte)168, (byte)41, (byte)133, (byte)159, (byte)213, (byte)211, (byte)90, (byte)240, (byte)243, (byte)5, (byte)190, (byte)226, (byte)70, (byte)216, (byte)123, (byte)222, (byte)236, (byte)205, (byte)179, (byte)67, (byte)136, (byte)84, (byte)193, (byte)195, (byte)237, (byte)142, (byte)15, (byte)90, (byte)251, (byte)160, (byte)95, (byte)8, (byte)82, (byte)166, (byte)83, (byte)117, (byte)151, (byte)152, (byte)26, (byte)140, (byte)186, (byte)154, (byte)1, (byte)24, (byte)142, (byte)197, (byte)239, (byte)248, (byte)130, (byte)173, (byte)122, (byte)61, (byte)84, (byte)69, (byte)136, (byte)78, (byte)227, (byte)46, (byte)14, (byte)229, (byte)177, (byte)147, (byte)2, (byte)49, (byte)70, (byte)123, (byte)27, (byte)127, (byte)78, (byte)112, (byte)245, (byte)170, (byte)108, (byte)110, (byte)176, (byte)243, (byte)123, (byte)119, (byte)63, (byte)29, (byte)219, (byte)69, (byte)251, (byte)103, (byte)20, (byte)173, (byte)126, (byte)125, (byte)72, (byte)120, (byte)115, (byte)156, (byte)28, (byte)17, (byte)70, (byte)2, (byte)227, (byte)210, (byte)170, (byte)173, (byte)117, (byte)107, (byte)252, (byte)219, (byte)207, (byte)113, (byte)71, (byte)150, (byte)50, (byte)61, (byte)231, (byte)185, (byte)92, (byte)161, (byte)231, (byte)244, (byte)90, (byte)109, (byte)239, (byte)126, (byte)209, (byte)127, (byte)203, (byte)47, (byte)49, (byte)195, (byte)34, (byte)137, (byte)141, (byte)201, (byte)94, (byte)121, (byte)67, (byte)106, (byte)114, (byte)238, (byte)24, (byte)145, (byte)135, (byte)105, (byte)157, (byte)230, (byte)244, (byte)15, (byte)63, (byte)137, (byte)216, (byte)44, (byte)210, (byte)202, (byte)214, (byte)28, (byte)154, (byte)162, (byte)89, (byte)79, (byte)37, (byte)150, (byte)84, (byte)203, (byte)54, (byte)193, (byte)40, (byte)97, (byte)12, (byte)80, (byte)241, (byte)162, (byte)93, (byte)165, (byte)113, (byte)198, (byte)27, (byte)7, (byte)94, (byte)64, (byte)206, (byte)31, (byte)179, (byte)41, (byte)115, (byte)51, (byte)47, (byte)140, (byte)21, (byte)181, (byte)225, (byte)130, (byte)164, (byte)6, (byte)89, (byte)113, (byte)116, (byte)33, (byte)206, (byte)64, (byte)221, (byte)185, (byte)221, (byte)39, (byte)136, (byte)221, (byte)157, (byte)185, (byte)189, (byte)248, (byte)240, (byte)248, (byte)245, (byte)236, (byte)145, (byte)217, (byte)84, (byte)20}, 0) ;
            p110.target_system = (byte)(byte)147;
            p110.target_component = (byte)(byte)110;
            LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -7853069804201533550L);
                Debug.Assert(pack.tc1 == (long) -3732924466873143630L);
            };
            DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -3732924466873143630L;
            p111.ts1 = (long) -7853069804201533550L;
            LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3423149579663120845L);
                Debug.Assert(pack.seq == (uint)747999041U);
            };
            DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)3423149579663120845L;
            p112.seq = (uint)747999041U;
            LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)46219);
                Debug.Assert(pack.eph == (ushort)(ushort)14493);
                Debug.Assert(pack.time_usec == (ulong)8432284870992130126L);
                Debug.Assert(pack.lon == (int) -1195652882);
                Debug.Assert(pack.fix_type == (byte)(byte)42);
                Debug.Assert(pack.satellites_visible == (byte)(byte)177);
                Debug.Assert(pack.vd == (short)(short)14283);
                Debug.Assert(pack.vn == (short)(short) -16053);
                Debug.Assert(pack.lat == (int) -28540073);
                Debug.Assert(pack.alt == (int)597406764);
                Debug.Assert(pack.ve == (short)(short)9997);
                Debug.Assert(pack.vel == (ushort)(ushort)53326);
                Debug.Assert(pack.epv == (ushort)(ushort)50269);
            };
            DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.fix_type = (byte)(byte)42;
            p113.vn = (short)(short) -16053;
            p113.vel = (ushort)(ushort)53326;
            p113.satellites_visible = (byte)(byte)177;
            p113.cog = (ushort)(ushort)46219;
            p113.vd = (short)(short)14283;
            p113.lon = (int) -1195652882;
            p113.alt = (int)597406764;
            p113.epv = (ushort)(ushort)50269;
            p113.eph = (ushort)(ushort)14493;
            p113.lat = (int) -28540073;
            p113.ve = (short)(short)9997;
            p113.time_usec = (ulong)8432284870992130126L;
            LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.926706E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2485809551U);
                Debug.Assert(pack.integrated_y == (float) -1.583543E38F);
                Debug.Assert(pack.time_usec == (ulong)6185724503570809412L);
                Debug.Assert(pack.temperature == (short)(short)27209);
                Debug.Assert(pack.quality == (byte)(byte)0);
                Debug.Assert(pack.integrated_xgyro == (float)1.978225E38F);
                Debug.Assert(pack.integrated_ygyro == (float)3.150402E37F);
                Debug.Assert(pack.distance == (float)1.2563763E38F);
                Debug.Assert(pack.integration_time_us == (uint)203980660U);
                Debug.Assert(pack.integrated_x == (float)1.1743033E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)117);
            };
            DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_delta_distance_us = (uint)2485809551U;
            p114.time_usec = (ulong)6185724503570809412L;
            p114.temperature = (short)(short)27209;
            p114.integrated_ygyro = (float)3.150402E37F;
            p114.sensor_id = (byte)(byte)117;
            p114.integrated_x = (float)1.1743033E38F;
            p114.integrated_y = (float) -1.583543E38F;
            p114.integration_time_us = (uint)203980660U;
            p114.integrated_zgyro = (float)2.926706E38F;
            p114.integrated_xgyro = (float)1.978225E38F;
            p114.quality = (byte)(byte)0;
            p114.distance = (float)1.2563763E38F;
            LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)8570);
                Debug.Assert(pack.xacc == (short)(short)26865);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)13476);
                Debug.Assert(pack.yacc == (short)(short) -32044);
                Debug.Assert(pack.vx == (short)(short) -9206);
                Debug.Assert(pack.lat == (int) -1512549971);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)13608);
                Debug.Assert(pack.vz == (short)(short) -19905);
                Debug.Assert(pack.vy == (short)(short) -26081);
                Debug.Assert(pack.yawspeed == (float)1.4884583E38F);
                Debug.Assert(pack.alt == (int) -1663642455);
                Debug.Assert(pack.time_usec == (ulong)7010468306152894861L);
                Debug.Assert(pack.lon == (int)1395927953);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {3.9957848E37F, -1.6751375E38F, 3.3590707E38F, -3.0080643E38F}));
                Debug.Assert(pack.pitchspeed == (float)3.0523767E38F);
                Debug.Assert(pack.rollspeed == (float) -1.4476153E37F);
            };
            DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vz = (short)(short) -19905;
            p115.pitchspeed = (float)3.0523767E38F;
            p115.yacc = (short)(short) -32044;
            p115.lon = (int)1395927953;
            p115.time_usec = (ulong)7010468306152894861L;
            p115.xacc = (short)(short)26865;
            p115.alt = (int) -1663642455;
            p115.lat = (int) -1512549971;
            p115.attitude_quaternion_SET(new float[] {3.9957848E37F, -1.6751375E38F, 3.3590707E38F, -3.0080643E38F}, 0) ;
            p115.rollspeed = (float) -1.4476153E37F;
            p115.zacc = (short)(short)8570;
            p115.vx = (short)(short) -9206;
            p115.vy = (short)(short) -26081;
            p115.ind_airspeed = (ushort)(ushort)13608;
            p115.true_airspeed = (ushort)(ushort)13476;
            p115.yawspeed = (float)1.4884583E38F;
            LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -26666);
                Debug.Assert(pack.zgyro == (short)(short) -31233);
                Debug.Assert(pack.xacc == (short)(short) -15682);
                Debug.Assert(pack.ymag == (short)(short) -13964);
                Debug.Assert(pack.xgyro == (short)(short) -3982);
                Debug.Assert(pack.zacc == (short)(short)5229);
                Debug.Assert(pack.ygyro == (short)(short)23587);
                Debug.Assert(pack.zmag == (short)(short)27047);
                Debug.Assert(pack.yacc == (short)(short)25470);
                Debug.Assert(pack.time_boot_ms == (uint)1187363087U);
            };
            DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xgyro = (short)(short) -3982;
            p116.time_boot_ms = (uint)1187363087U;
            p116.zgyro = (short)(short) -31233;
            p116.zacc = (short)(short)5229;
            p116.zmag = (short)(short)27047;
            p116.xacc = (short)(short) -15682;
            p116.yacc = (short)(short)25470;
            p116.ygyro = (short)(short)23587;
            p116.xmag = (short)(short) -26666;
            p116.ymag = (short)(short) -13964;
            LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)53541);
                Debug.Assert(pack.target_component == (byte)(byte)233);
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.start == (ushort)(ushort)65510);
            };
            DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)169;
            p117.end = (ushort)(ushort)53541;
            p117.start = (ushort)(ushort)65510;
            p117.target_component = (byte)(byte)233;
            LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)3921);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)4546);
                Debug.Assert(pack.id == (ushort)(ushort)63566);
                Debug.Assert(pack.size == (uint)3394461993U);
                Debug.Assert(pack.time_utc == (uint)3746505533U);
            };
            DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)3746505533U;
            p118.size = (uint)3394461993U;
            p118.last_log_num = (ushort)(ushort)4546;
            p118.num_logs = (ushort)(ushort)3921;
            p118.id = (ushort)(ushort)63566;
            LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)39420);
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.count == (uint)894805396U);
                Debug.Assert(pack.ofs == (uint)3478036432U);
                Debug.Assert(pack.target_component == (byte)(byte)246);
            };
            DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)894805396U;
            p119.target_system = (byte)(byte)40;
            p119.id = (ushort)(ushort)39420;
            p119.target_component = (byte)(byte)246;
            p119.ofs = (uint)3478036432U;
            LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)151, (byte)167, (byte)58, (byte)25, (byte)217, (byte)242, (byte)180, (byte)150, (byte)100, (byte)7, (byte)229, (byte)255, (byte)104, (byte)230, (byte)192, (byte)74, (byte)163, (byte)103, (byte)211, (byte)219, (byte)4, (byte)73, (byte)235, (byte)111, (byte)26, (byte)211, (byte)128, (byte)224, (byte)202, (byte)8, (byte)133, (byte)18, (byte)212, (byte)163, (byte)14, (byte)254, (byte)116, (byte)32, (byte)42, (byte)66, (byte)229, (byte)56, (byte)4, (byte)118, (byte)71, (byte)12, (byte)127, (byte)243, (byte)70, (byte)212, (byte)84, (byte)182, (byte)5, (byte)71, (byte)88, (byte)238, (byte)152, (byte)212, (byte)158, (byte)146, (byte)46, (byte)177, (byte)238, (byte)80, (byte)227, (byte)200, (byte)155, (byte)29, (byte)78, (byte)60, (byte)155, (byte)74, (byte)177, (byte)141, (byte)112, (byte)12, (byte)168, (byte)244, (byte)82, (byte)26, (byte)196, (byte)21, (byte)234, (byte)254, (byte)221, (byte)84, (byte)111, (byte)202, (byte)163, (byte)176}));
                Debug.Assert(pack.count == (byte)(byte)22);
                Debug.Assert(pack.ofs == (uint)2002745700U);
                Debug.Assert(pack.id == (ushort)(ushort)52503);
            };
            DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)22;
            p120.id = (ushort)(ushort)52503;
            p120.data__SET(new byte[] {(byte)151, (byte)167, (byte)58, (byte)25, (byte)217, (byte)242, (byte)180, (byte)150, (byte)100, (byte)7, (byte)229, (byte)255, (byte)104, (byte)230, (byte)192, (byte)74, (byte)163, (byte)103, (byte)211, (byte)219, (byte)4, (byte)73, (byte)235, (byte)111, (byte)26, (byte)211, (byte)128, (byte)224, (byte)202, (byte)8, (byte)133, (byte)18, (byte)212, (byte)163, (byte)14, (byte)254, (byte)116, (byte)32, (byte)42, (byte)66, (byte)229, (byte)56, (byte)4, (byte)118, (byte)71, (byte)12, (byte)127, (byte)243, (byte)70, (byte)212, (byte)84, (byte)182, (byte)5, (byte)71, (byte)88, (byte)238, (byte)152, (byte)212, (byte)158, (byte)146, (byte)46, (byte)177, (byte)238, (byte)80, (byte)227, (byte)200, (byte)155, (byte)29, (byte)78, (byte)60, (byte)155, (byte)74, (byte)177, (byte)141, (byte)112, (byte)12, (byte)168, (byte)244, (byte)82, (byte)26, (byte)196, (byte)21, (byte)234, (byte)254, (byte)221, (byte)84, (byte)111, (byte)202, (byte)163, (byte)176}, 0) ;
            p120.ofs = (uint)2002745700U;
            LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)239);
                Debug.Assert(pack.target_system == (byte)(byte)27);
            };
            DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)27;
            p121.target_component = (byte)(byte)239;
            LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.target_component == (byte)(byte)218);
            };
            DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)175;
            p122.target_component = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)236);
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)228, (byte)207, (byte)97, (byte)65, (byte)61, (byte)123, (byte)72, (byte)6, (byte)4, (byte)180, (byte)117, (byte)76, (byte)132, (byte)230, (byte)161, (byte)15, (byte)183, (byte)56, (byte)136, (byte)59, (byte)155, (byte)48, (byte)62, (byte)94, (byte)111, (byte)74, (byte)32, (byte)218, (byte)72, (byte)14, (byte)29, (byte)113, (byte)161, (byte)84, (byte)193, (byte)100, (byte)49, (byte)66, (byte)86, (byte)70, (byte)204, (byte)107, (byte)144, (byte)177, (byte)78, (byte)251, (byte)40, (byte)78, (byte)180, (byte)102, (byte)72, (byte)162, (byte)221, (byte)166, (byte)251, (byte)225, (byte)174, (byte)142, (byte)235, (byte)59, (byte)122, (byte)86, (byte)167, (byte)81, (byte)158, (byte)49, (byte)127, (byte)186, (byte)125, (byte)64, (byte)154, (byte)84, (byte)31, (byte)234, (byte)230, (byte)10, (byte)228, (byte)166, (byte)30, (byte)35, (byte)161, (byte)147, (byte)183, (byte)230, (byte)234, (byte)162, (byte)238, (byte)255, (byte)224, (byte)149, (byte)191, (byte)251, (byte)136, (byte)28, (byte)232, (byte)144, (byte)69, (byte)52, (byte)117, (byte)213, (byte)213, (byte)183, (byte)5, (byte)15, (byte)139, (byte)204, (byte)219, (byte)38, (byte)126, (byte)84}));
            };
            DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)236;
            p123.target_system = (byte)(byte)171;
            p123.data__SET(new byte[] {(byte)228, (byte)207, (byte)97, (byte)65, (byte)61, (byte)123, (byte)72, (byte)6, (byte)4, (byte)180, (byte)117, (byte)76, (byte)132, (byte)230, (byte)161, (byte)15, (byte)183, (byte)56, (byte)136, (byte)59, (byte)155, (byte)48, (byte)62, (byte)94, (byte)111, (byte)74, (byte)32, (byte)218, (byte)72, (byte)14, (byte)29, (byte)113, (byte)161, (byte)84, (byte)193, (byte)100, (byte)49, (byte)66, (byte)86, (byte)70, (byte)204, (byte)107, (byte)144, (byte)177, (byte)78, (byte)251, (byte)40, (byte)78, (byte)180, (byte)102, (byte)72, (byte)162, (byte)221, (byte)166, (byte)251, (byte)225, (byte)174, (byte)142, (byte)235, (byte)59, (byte)122, (byte)86, (byte)167, (byte)81, (byte)158, (byte)49, (byte)127, (byte)186, (byte)125, (byte)64, (byte)154, (byte)84, (byte)31, (byte)234, (byte)230, (byte)10, (byte)228, (byte)166, (byte)30, (byte)35, (byte)161, (byte)147, (byte)183, (byte)230, (byte)234, (byte)162, (byte)238, (byte)255, (byte)224, (byte)149, (byte)191, (byte)251, (byte)136, (byte)28, (byte)232, (byte)144, (byte)69, (byte)52, (byte)117, (byte)213, (byte)213, (byte)183, (byte)5, (byte)15, (byte)139, (byte)204, (byte)219, (byte)38, (byte)126, (byte)84}, 0) ;
            p123.target_component = (byte)(byte)61;
            LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)176);
                Debug.Assert(pack.lat == (int) -776373821);
                Debug.Assert(pack.dgps_numch == (byte)(byte)177);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.epv == (ushort)(ushort)45196);
                Debug.Assert(pack.lon == (int) -1386974039);
                Debug.Assert(pack.alt == (int) -1540911475);
                Debug.Assert(pack.dgps_age == (uint)359631346U);
                Debug.Assert(pack.time_usec == (ulong)5953361206420051010L);
                Debug.Assert(pack.eph == (ushort)(ushort)31995);
                Debug.Assert(pack.vel == (ushort)(ushort)19060);
                Debug.Assert(pack.cog == (ushort)(ushort)56768);
            };
            DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.vel = (ushort)(ushort)19060;
            p124.lon = (int) -1386974039;
            p124.dgps_age = (uint)359631346U;
            p124.time_usec = (ulong)5953361206420051010L;
            p124.dgps_numch = (byte)(byte)177;
            p124.eph = (ushort)(ushort)31995;
            p124.alt = (int) -1540911475;
            p124.lat = (int) -776373821;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p124.epv = (ushort)(ushort)45196;
            p124.cog = (ushort)(ushort)56768;
            p124.satellites_visible = (byte)(byte)176;
            LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)33044);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
                Debug.Assert(pack.Vservo == (ushort)(ushort)3913);
            };
            DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)33044;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            p125.Vservo = (ushort)(ushort)3913;
            LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.baudrate == (uint)614003647U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)151, (byte)153, (byte)213, (byte)132, (byte)50, (byte)65, (byte)95, (byte)184, (byte)213, (byte)133, (byte)22, (byte)4, (byte)242, (byte)160, (byte)220, (byte)250, (byte)160, (byte)180, (byte)193, (byte)113, (byte)11, (byte)183, (byte)30, (byte)6, (byte)3, (byte)255, (byte)148, (byte)140, (byte)135, (byte)144, (byte)180, (byte)238, (byte)169, (byte)153, (byte)21, (byte)176, (byte)33, (byte)122, (byte)97, (byte)189, (byte)112, (byte)144, (byte)203, (byte)3, (byte)113, (byte)82, (byte)181, (byte)37, (byte)172, (byte)118, (byte)25, (byte)101, (byte)158, (byte)99, (byte)14, (byte)205, (byte)35, (byte)252, (byte)25, (byte)87, (byte)5, (byte)148, (byte)231, (byte)211, (byte)99, (byte)30, (byte)26, (byte)170, (byte)205, (byte)191}));
                Debug.Assert(pack.timeout == (ushort)(ushort)2861);
                Debug.Assert(pack.count == (byte)(byte)36);
            };
            DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)36;
            p126.timeout = (ushort)(ushort)2861;
            p126.baudrate = (uint)614003647U;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.data__SET(new byte[] {(byte)151, (byte)153, (byte)213, (byte)132, (byte)50, (byte)65, (byte)95, (byte)184, (byte)213, (byte)133, (byte)22, (byte)4, (byte)242, (byte)160, (byte)220, (byte)250, (byte)160, (byte)180, (byte)193, (byte)113, (byte)11, (byte)183, (byte)30, (byte)6, (byte)3, (byte)255, (byte)148, (byte)140, (byte)135, (byte)144, (byte)180, (byte)238, (byte)169, (byte)153, (byte)21, (byte)176, (byte)33, (byte)122, (byte)97, (byte)189, (byte)112, (byte)144, (byte)203, (byte)3, (byte)113, (byte)82, (byte)181, (byte)37, (byte)172, (byte)118, (byte)25, (byte)101, (byte)158, (byte)99, (byte)14, (byte)205, (byte)35, (byte)252, (byte)25, (byte)87, (byte)5, (byte)148, (byte)231, (byte)211, (byte)99, (byte)30, (byte)26, (byte)170, (byte)205, (byte)191}, 0) ;
            LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int)265444520);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)49);
                Debug.Assert(pack.tow == (uint)903752743U);
                Debug.Assert(pack.nsats == (byte)(byte)125);
                Debug.Assert(pack.accuracy == (uint)2803265027U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1659158288);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)130);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1069910271U);
                Debug.Assert(pack.rtk_health == (byte)(byte)26);
                Debug.Assert(pack.baseline_a_mm == (int)437666248);
                Debug.Assert(pack.baseline_c_mm == (int) -1054810781);
                Debug.Assert(pack.wn == (ushort)(ushort)7148);
                Debug.Assert(pack.rtk_rate == (byte)(byte)191);
            };
            DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_health = (byte)(byte)26;
            p127.rtk_receiver_id = (byte)(byte)130;
            p127.iar_num_hypotheses = (int) -1659158288;
            p127.accuracy = (uint)2803265027U;
            p127.baseline_b_mm = (int)265444520;
            p127.wn = (ushort)(ushort)7148;
            p127.nsats = (byte)(byte)125;
            p127.tow = (uint)903752743U;
            p127.baseline_coords_type = (byte)(byte)49;
            p127.time_last_baseline_ms = (uint)1069910271U;
            p127.baseline_a_mm = (int)437666248;
            p127.baseline_c_mm = (int) -1054810781;
            p127.rtk_rate = (byte)(byte)191;
            LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)3639628734U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)50);
                Debug.Assert(pack.baseline_b_mm == (int) -614964147);
                Debug.Assert(pack.rtk_rate == (byte)(byte)17);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1223196701U);
                Debug.Assert(pack.iar_num_hypotheses == (int)2111692727);
                Debug.Assert(pack.baseline_a_mm == (int)1686204480);
                Debug.Assert(pack.accuracy == (uint)1199632957U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)150);
                Debug.Assert(pack.wn == (ushort)(ushort)50919);
                Debug.Assert(pack.nsats == (byte)(byte)160);
                Debug.Assert(pack.rtk_health == (byte)(byte)47);
                Debug.Assert(pack.baseline_c_mm == (int)794763330);
            };
            DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.wn = (ushort)(ushort)50919;
            p128.baseline_b_mm = (int) -614964147;
            p128.rtk_rate = (byte)(byte)17;
            p128.baseline_a_mm = (int)1686204480;
            p128.rtk_receiver_id = (byte)(byte)50;
            p128.baseline_coords_type = (byte)(byte)150;
            p128.accuracy = (uint)1199632957U;
            p128.rtk_health = (byte)(byte)47;
            p128.iar_num_hypotheses = (int)2111692727;
            p128.nsats = (byte)(byte)160;
            p128.baseline_c_mm = (int)794763330;
            p128.time_last_baseline_ms = (uint)1223196701U;
            p128.tow = (uint)3639628734U;
            LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -15810);
                Debug.Assert(pack.xacc == (short)(short) -7882);
                Debug.Assert(pack.zacc == (short)(short) -28155);
                Debug.Assert(pack.zmag == (short)(short)22404);
                Debug.Assert(pack.ygyro == (short)(short) -16724);
                Debug.Assert(pack.yacc == (short)(short) -30914);
                Debug.Assert(pack.xmag == (short)(short)23785);
                Debug.Assert(pack.ymag == (short)(short) -27420);
                Debug.Assert(pack.xgyro == (short)(short)4319);
                Debug.Assert(pack.time_boot_ms == (uint)3277381017U);
            };
            DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.yacc = (short)(short) -30914;
            p129.zacc = (short)(short) -28155;
            p129.ygyro = (short)(short) -16724;
            p129.ymag = (short)(short) -27420;
            p129.zgyro = (short)(short) -15810;
            p129.xacc = (short)(short) -7882;
            p129.zmag = (short)(short)22404;
            p129.xmag = (short)(short)23785;
            p129.time_boot_ms = (uint)3277381017U;
            p129.xgyro = (short)(short)4319;
            LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)150);
                Debug.Assert(pack.packets == (ushort)(ushort)55632);
                Debug.Assert(pack.type == (byte)(byte)245);
                Debug.Assert(pack.width == (ushort)(ushort)51511);
                Debug.Assert(pack.height == (ushort)(ushort)36261);
                Debug.Assert(pack.payload == (byte)(byte)225);
                Debug.Assert(pack.size == (uint)2995510274U);
            };
            DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)150;
            p130.packets = (ushort)(ushort)55632;
            p130.height = (ushort)(ushort)36261;
            p130.payload = (byte)(byte)225;
            p130.width = (ushort)(ushort)51511;
            p130.size = (uint)2995510274U;
            p130.type = (byte)(byte)245;
            LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)96, (byte)211, (byte)26, (byte)241, (byte)77, (byte)179, (byte)108, (byte)246, (byte)75, (byte)136, (byte)202, (byte)180, (byte)165, (byte)60, (byte)137, (byte)87, (byte)179, (byte)38, (byte)196, (byte)242, (byte)68, (byte)155, (byte)163, (byte)46, (byte)191, (byte)104, (byte)74, (byte)110, (byte)119, (byte)24, (byte)100, (byte)236, (byte)1, (byte)151, (byte)40, (byte)242, (byte)37, (byte)183, (byte)9, (byte)232, (byte)92, (byte)77, (byte)63, (byte)136, (byte)155, (byte)62, (byte)34, (byte)121, (byte)201, (byte)243, (byte)83, (byte)212, (byte)186, (byte)221, (byte)146, (byte)127, (byte)9, (byte)25, (byte)122, (byte)169, (byte)93, (byte)135, (byte)94, (byte)9, (byte)155, (byte)140, (byte)54, (byte)83, (byte)181, (byte)69, (byte)192, (byte)80, (byte)97, (byte)16, (byte)8, (byte)32, (byte)90, (byte)81, (byte)28, (byte)146, (byte)234, (byte)224, (byte)215, (byte)228, (byte)193, (byte)163, (byte)132, (byte)173, (byte)108, (byte)229, (byte)9, (byte)249, (byte)136, (byte)198, (byte)122, (byte)40, (byte)222, (byte)3, (byte)204, (byte)117, (byte)189, (byte)209, (byte)57, (byte)12, (byte)207, (byte)67, (byte)20, (byte)178, (byte)248, (byte)220, (byte)209, (byte)142, (byte)225, (byte)213, (byte)230, (byte)148, (byte)95, (byte)75, (byte)1, (byte)225, (byte)9, (byte)160, (byte)159, (byte)188, (byte)143, (byte)243, (byte)127, (byte)250, (byte)74, (byte)135, (byte)111, (byte)246, (byte)124, (byte)174, (byte)55, (byte)208, (byte)155, (byte)71, (byte)213, (byte)95, (byte)209, (byte)228, (byte)216, (byte)213, (byte)124, (byte)62, (byte)179, (byte)86, (byte)178, (byte)204, (byte)111, (byte)42, (byte)228, (byte)51, (byte)173, (byte)235, (byte)78, (byte)242, (byte)70, (byte)212, (byte)106, (byte)229, (byte)112, (byte)65, (byte)98, (byte)95, (byte)216, (byte)71, (byte)90, (byte)200, (byte)217, (byte)64, (byte)253, (byte)175, (byte)13, (byte)1, (byte)126, (byte)204, (byte)160, (byte)81, (byte)96, (byte)201, (byte)144, (byte)116, (byte)16, (byte)252, (byte)4, (byte)61, (byte)45, (byte)39, (byte)172, (byte)138, (byte)89, (byte)25, (byte)232, (byte)200, (byte)170, (byte)60, (byte)87, (byte)183, (byte)239, (byte)130, (byte)190, (byte)16, (byte)77, (byte)129, (byte)148, (byte)40, (byte)1, (byte)153, (byte)173, (byte)241, (byte)44, (byte)209, (byte)38, (byte)166, (byte)184, (byte)253, (byte)156, (byte)170, (byte)128, (byte)12, (byte)242, (byte)79, (byte)205, (byte)17, (byte)40, (byte)107, (byte)230, (byte)145, (byte)8, (byte)231, (byte)110, (byte)169, (byte)164, (byte)121, (byte)184, (byte)118, (byte)187, (byte)200, (byte)131, (byte)243, (byte)113, (byte)174, (byte)173, (byte)192, (byte)107, (byte)50, (byte)20, (byte)91, (byte)83, (byte)135, (byte)12}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)39275);
            };
            DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)96, (byte)211, (byte)26, (byte)241, (byte)77, (byte)179, (byte)108, (byte)246, (byte)75, (byte)136, (byte)202, (byte)180, (byte)165, (byte)60, (byte)137, (byte)87, (byte)179, (byte)38, (byte)196, (byte)242, (byte)68, (byte)155, (byte)163, (byte)46, (byte)191, (byte)104, (byte)74, (byte)110, (byte)119, (byte)24, (byte)100, (byte)236, (byte)1, (byte)151, (byte)40, (byte)242, (byte)37, (byte)183, (byte)9, (byte)232, (byte)92, (byte)77, (byte)63, (byte)136, (byte)155, (byte)62, (byte)34, (byte)121, (byte)201, (byte)243, (byte)83, (byte)212, (byte)186, (byte)221, (byte)146, (byte)127, (byte)9, (byte)25, (byte)122, (byte)169, (byte)93, (byte)135, (byte)94, (byte)9, (byte)155, (byte)140, (byte)54, (byte)83, (byte)181, (byte)69, (byte)192, (byte)80, (byte)97, (byte)16, (byte)8, (byte)32, (byte)90, (byte)81, (byte)28, (byte)146, (byte)234, (byte)224, (byte)215, (byte)228, (byte)193, (byte)163, (byte)132, (byte)173, (byte)108, (byte)229, (byte)9, (byte)249, (byte)136, (byte)198, (byte)122, (byte)40, (byte)222, (byte)3, (byte)204, (byte)117, (byte)189, (byte)209, (byte)57, (byte)12, (byte)207, (byte)67, (byte)20, (byte)178, (byte)248, (byte)220, (byte)209, (byte)142, (byte)225, (byte)213, (byte)230, (byte)148, (byte)95, (byte)75, (byte)1, (byte)225, (byte)9, (byte)160, (byte)159, (byte)188, (byte)143, (byte)243, (byte)127, (byte)250, (byte)74, (byte)135, (byte)111, (byte)246, (byte)124, (byte)174, (byte)55, (byte)208, (byte)155, (byte)71, (byte)213, (byte)95, (byte)209, (byte)228, (byte)216, (byte)213, (byte)124, (byte)62, (byte)179, (byte)86, (byte)178, (byte)204, (byte)111, (byte)42, (byte)228, (byte)51, (byte)173, (byte)235, (byte)78, (byte)242, (byte)70, (byte)212, (byte)106, (byte)229, (byte)112, (byte)65, (byte)98, (byte)95, (byte)216, (byte)71, (byte)90, (byte)200, (byte)217, (byte)64, (byte)253, (byte)175, (byte)13, (byte)1, (byte)126, (byte)204, (byte)160, (byte)81, (byte)96, (byte)201, (byte)144, (byte)116, (byte)16, (byte)252, (byte)4, (byte)61, (byte)45, (byte)39, (byte)172, (byte)138, (byte)89, (byte)25, (byte)232, (byte)200, (byte)170, (byte)60, (byte)87, (byte)183, (byte)239, (byte)130, (byte)190, (byte)16, (byte)77, (byte)129, (byte)148, (byte)40, (byte)1, (byte)153, (byte)173, (byte)241, (byte)44, (byte)209, (byte)38, (byte)166, (byte)184, (byte)253, (byte)156, (byte)170, (byte)128, (byte)12, (byte)242, (byte)79, (byte)205, (byte)17, (byte)40, (byte)107, (byte)230, (byte)145, (byte)8, (byte)231, (byte)110, (byte)169, (byte)164, (byte)121, (byte)184, (byte)118, (byte)187, (byte)200, (byte)131, (byte)243, (byte)113, (byte)174, (byte)173, (byte)192, (byte)107, (byte)50, (byte)20, (byte)91, (byte)83, (byte)135, (byte)12}, 0) ;
            p131.seqnr = (ushort)(ushort)39275;
            LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)33962);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE);
                Debug.Assert(pack.covariance == (byte)(byte)241);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.current_distance == (ushort)(ushort)37821);
                Debug.Assert(pack.max_distance == (ushort)(ushort)41373);
                Debug.Assert(pack.time_boot_ms == (uint)1829303807U);
                Debug.Assert(pack.id == (byte)(byte)102);
            };
            DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.max_distance = (ushort)(ushort)41373;
            p132.current_distance = (ushort)(ushort)37821;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.min_distance = (ushort)(ushort)33962;
            p132.time_boot_ms = (uint)1829303807U;
            p132.id = (byte)(byte)102;
            p132.covariance = (byte)(byte)241;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE;
            LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)3043);
                Debug.Assert(pack.lat == (int) -378913440);
                Debug.Assert(pack.mask == (ulong)820121574521899358L);
                Debug.Assert(pack.lon == (int)1060904918);
            };
            DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -378913440;
            p133.mask = (ulong)820121574521899358L;
            p133.grid_spacing = (ushort)(ushort)3043;
            p133.lon = (int)1060904918;
            LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1091449549);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)30256, (short) -30218, (short)22553, (short)17176, (short) -18948, (short)16683, (short) -7728, (short) -11194, (short)30656, (short)24434, (short) -11747, (short)661, (short) -8465, (short) -28925, (short) -9291, (short) -24187}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)19807);
                Debug.Assert(pack.lon == (int)1934877937);
                Debug.Assert(pack.gridbit == (byte)(byte)179);
            };
            DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int)1934877937;
            p134.lat = (int) -1091449549;
            p134.grid_spacing = (ushort)(ushort)19807;
            p134.gridbit = (byte)(byte)179;
            p134.data__SET(new short[] {(short)30256, (short) -30218, (short)22553, (short)17176, (short) -18948, (short)16683, (short) -7728, (short) -11194, (short)30656, (short)24434, (short) -11747, (short)661, (short) -8465, (short) -28925, (short) -9291, (short) -24187}, 0) ;
            LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1664673902);
                Debug.Assert(pack.lon == (int) -1859492891);
            };
            DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1664673902;
            p135.lon = (int) -1859492891;
            LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)32774);
                Debug.Assert(pack.lon == (int) -1416738036);
                Debug.Assert(pack.lat == (int) -1545219768);
                Debug.Assert(pack.loaded == (ushort)(ushort)64212);
                Debug.Assert(pack.spacing == (ushort)(ushort)60284);
                Debug.Assert(pack.current_height == (float) -1.3879298E38F);
                Debug.Assert(pack.terrain_height == (float) -4.5664517E37F);
            };
            DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1545219768;
            p136.loaded = (ushort)(ushort)64212;
            p136.terrain_height = (float) -4.5664517E37F;
            p136.lon = (int) -1416738036;
            p136.current_height = (float) -1.3879298E38F;
            p136.pending = (ushort)(ushort)32774;
            p136.spacing = (ushort)(ushort)60284;
            LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3759638431U);
                Debug.Assert(pack.press_abs == (float) -4.7584135E37F);
                Debug.Assert(pack.temperature == (short)(short) -23460);
                Debug.Assert(pack.press_diff == (float) -7.9222134E37F);
            };
            DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float) -4.7584135E37F;
            p137.time_boot_ms = (uint)3759638431U;
            p137.press_diff = (float) -7.9222134E37F;
            p137.temperature = (short)(short) -23460;
            LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.9822093E38F);
                Debug.Assert(pack.time_usec == (ulong)4847961783136136733L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4986742E38F, -1.7745865E38F, -3.8167413E37F, -1.121268E38F}));
                Debug.Assert(pack.y == (float)2.7863905E38F);
                Debug.Assert(pack.x == (float) -2.0907668E38F);
            };
            DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float)2.7863905E38F;
            p138.x = (float) -2.0907668E38F;
            p138.z = (float)2.9822093E38F;
            p138.time_usec = (ulong)4847961783136136733L;
            p138.q_SET(new float[] {-1.4986742E38F, -1.7745865E38F, -3.8167413E37F, -1.121268E38F}, 0) ;
            LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8864807012194453591L);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.5688776E38F, -2.1511597E38F, 1.8626787E38F, 3.421936E37F, 3.2196407E38F, 8.066233E37F, 2.9679E37F, 1.702062E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.group_mlx == (byte)(byte)233);
            };
            DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)233;
            p139.target_system = (byte)(byte)133;
            p139.controls_SET(new float[] {1.5688776E38F, -2.1511597E38F, 1.8626787E38F, 3.421936E37F, 3.2196407E38F, 8.066233E37F, 2.9679E37F, 1.702062E38F}, 0) ;
            p139.time_usec = (ulong)8864807012194453591L;
            p139.target_component = (byte)(byte)25;
            LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)108);
                Debug.Assert(pack.time_usec == (ulong)7739346931840559620L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.404515E38F, -2.6266296E38F, -3.032107E38F, -2.3179156E38F, -9.447188E37F, -2.2545906E38F, -8.571644E37F, 1.914047E37F}));
            };
            DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)7739346931840559620L;
            p140.group_mlx = (byte)(byte)108;
            p140.controls_SET(new float[] {1.404515E38F, -2.6266296E38F, -3.032107E38F, -2.3179156E38F, -9.447188E37F, -2.2545906E38F, -8.571644E37F, 1.914047E37F}, 0) ;
            LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_terrain == (float) -1.7798358E38F);
                Debug.Assert(pack.altitude_amsl == (float)1.8109494E37F);
                Debug.Assert(pack.time_usec == (ulong)2714921872629978975L);
                Debug.Assert(pack.altitude_local == (float)5.0257935E37F);
                Debug.Assert(pack.altitude_monotonic == (float)1.8903007E38F);
                Debug.Assert(pack.bottom_clearance == (float) -2.4227862E38F);
                Debug.Assert(pack.altitude_relative == (float) -2.9911601E38F);
            };
            DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_relative = (float) -2.9911601E38F;
            p141.altitude_local = (float)5.0257935E37F;
            p141.altitude_amsl = (float)1.8109494E37F;
            p141.bottom_clearance = (float) -2.4227862E38F;
            p141.time_usec = (ulong)2714921872629978975L;
            p141.altitude_terrain = (float) -1.7798358E38F;
            p141.altitude_monotonic = (float)1.8903007E38F;
            LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)232);
                Debug.Assert(pack.transfer_type == (byte)(byte)183);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)90, (byte)232, (byte)13, (byte)170, (byte)226, (byte)30, (byte)50, (byte)246, (byte)60, (byte)175, (byte)204, (byte)212, (byte)6, (byte)224, (byte)77, (byte)38, (byte)205, (byte)163, (byte)229, (byte)26, (byte)48, (byte)232, (byte)149, (byte)46, (byte)156, (byte)45, (byte)22, (byte)96, (byte)60, (byte)12, (byte)56, (byte)216, (byte)101, (byte)250, (byte)80, (byte)115, (byte)244, (byte)254, (byte)211, (byte)188, (byte)205, (byte)44, (byte)200, (byte)50, (byte)116, (byte)251, (byte)54, (byte)179, (byte)34, (byte)64, (byte)118, (byte)126, (byte)209, (byte)218, (byte)217, (byte)88, (byte)163, (byte)50, (byte)72, (byte)28, (byte)168, (byte)101, (byte)143, (byte)147, (byte)89, (byte)111, (byte)165, (byte)34, (byte)41, (byte)230, (byte)231, (byte)216, (byte)24, (byte)195, (byte)180, (byte)118, (byte)111, (byte)95, (byte)189, (byte)190, (byte)246, (byte)225, (byte)134, (byte)39, (byte)205, (byte)253, (byte)209, (byte)184, (byte)21, (byte)135, (byte)85, (byte)136, (byte)217, (byte)72, (byte)233, (byte)61, (byte)121, (byte)83, (byte)138, (byte)132, (byte)115, (byte)0, (byte)115, (byte)14, (byte)40, (byte)92, (byte)149, (byte)16, (byte)104, (byte)156, (byte)130, (byte)13, (byte)233, (byte)9, (byte)35, (byte)171, (byte)164, (byte)225, (byte)243, (byte)45}));
                Debug.Assert(pack.request_id == (byte)(byte)126);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)79, (byte)79, (byte)91, (byte)34, (byte)77, (byte)113, (byte)233, (byte)31, (byte)144, (byte)74, (byte)89, (byte)176, (byte)176, (byte)69, (byte)111, (byte)247, (byte)89, (byte)214, (byte)190, (byte)246, (byte)119, (byte)17, (byte)245, (byte)75, (byte)64, (byte)79, (byte)227, (byte)209, (byte)117, (byte)138, (byte)105, (byte)113, (byte)142, (byte)94, (byte)129, (byte)200, (byte)146, (byte)153, (byte)249, (byte)79, (byte)61, (byte)170, (byte)126, (byte)161, (byte)232, (byte)187, (byte)74, (byte)32, (byte)0, (byte)99, (byte)165, (byte)229, (byte)65, (byte)157, (byte)139, (byte)126, (byte)99, (byte)123, (byte)240, (byte)56, (byte)127, (byte)34, (byte)244, (byte)239, (byte)102, (byte)79, (byte)203, (byte)57, (byte)94, (byte)99, (byte)179, (byte)52, (byte)194, (byte)165, (byte)210, (byte)185, (byte)16, (byte)179, (byte)186, (byte)204, (byte)52, (byte)90, (byte)147, (byte)60, (byte)175, (byte)138, (byte)218, (byte)129, (byte)5, (byte)75, (byte)42, (byte)97, (byte)97, (byte)83, (byte)251, (byte)231, (byte)78, (byte)254, (byte)88, (byte)174, (byte)3, (byte)35, (byte)173, (byte)38, (byte)206, (byte)247, (byte)139, (byte)238, (byte)84, (byte)210, (byte)17, (byte)206, (byte)99, (byte)181, (byte)235, (byte)112, (byte)214, (byte)205, (byte)250, (byte)217}));
            };
            DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)79, (byte)79, (byte)91, (byte)34, (byte)77, (byte)113, (byte)233, (byte)31, (byte)144, (byte)74, (byte)89, (byte)176, (byte)176, (byte)69, (byte)111, (byte)247, (byte)89, (byte)214, (byte)190, (byte)246, (byte)119, (byte)17, (byte)245, (byte)75, (byte)64, (byte)79, (byte)227, (byte)209, (byte)117, (byte)138, (byte)105, (byte)113, (byte)142, (byte)94, (byte)129, (byte)200, (byte)146, (byte)153, (byte)249, (byte)79, (byte)61, (byte)170, (byte)126, (byte)161, (byte)232, (byte)187, (byte)74, (byte)32, (byte)0, (byte)99, (byte)165, (byte)229, (byte)65, (byte)157, (byte)139, (byte)126, (byte)99, (byte)123, (byte)240, (byte)56, (byte)127, (byte)34, (byte)244, (byte)239, (byte)102, (byte)79, (byte)203, (byte)57, (byte)94, (byte)99, (byte)179, (byte)52, (byte)194, (byte)165, (byte)210, (byte)185, (byte)16, (byte)179, (byte)186, (byte)204, (byte)52, (byte)90, (byte)147, (byte)60, (byte)175, (byte)138, (byte)218, (byte)129, (byte)5, (byte)75, (byte)42, (byte)97, (byte)97, (byte)83, (byte)251, (byte)231, (byte)78, (byte)254, (byte)88, (byte)174, (byte)3, (byte)35, (byte)173, (byte)38, (byte)206, (byte)247, (byte)139, (byte)238, (byte)84, (byte)210, (byte)17, (byte)206, (byte)99, (byte)181, (byte)235, (byte)112, (byte)214, (byte)205, (byte)250, (byte)217}, 0) ;
            p142.uri_SET(new byte[] {(byte)90, (byte)232, (byte)13, (byte)170, (byte)226, (byte)30, (byte)50, (byte)246, (byte)60, (byte)175, (byte)204, (byte)212, (byte)6, (byte)224, (byte)77, (byte)38, (byte)205, (byte)163, (byte)229, (byte)26, (byte)48, (byte)232, (byte)149, (byte)46, (byte)156, (byte)45, (byte)22, (byte)96, (byte)60, (byte)12, (byte)56, (byte)216, (byte)101, (byte)250, (byte)80, (byte)115, (byte)244, (byte)254, (byte)211, (byte)188, (byte)205, (byte)44, (byte)200, (byte)50, (byte)116, (byte)251, (byte)54, (byte)179, (byte)34, (byte)64, (byte)118, (byte)126, (byte)209, (byte)218, (byte)217, (byte)88, (byte)163, (byte)50, (byte)72, (byte)28, (byte)168, (byte)101, (byte)143, (byte)147, (byte)89, (byte)111, (byte)165, (byte)34, (byte)41, (byte)230, (byte)231, (byte)216, (byte)24, (byte)195, (byte)180, (byte)118, (byte)111, (byte)95, (byte)189, (byte)190, (byte)246, (byte)225, (byte)134, (byte)39, (byte)205, (byte)253, (byte)209, (byte)184, (byte)21, (byte)135, (byte)85, (byte)136, (byte)217, (byte)72, (byte)233, (byte)61, (byte)121, (byte)83, (byte)138, (byte)132, (byte)115, (byte)0, (byte)115, (byte)14, (byte)40, (byte)92, (byte)149, (byte)16, (byte)104, (byte)156, (byte)130, (byte)13, (byte)233, (byte)9, (byte)35, (byte)171, (byte)164, (byte)225, (byte)243, (byte)45}, 0) ;
            p142.uri_type = (byte)(byte)232;
            p142.transfer_type = (byte)(byte)183;
            p142.request_id = (byte)(byte)126;
            LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4192223818U);
                Debug.Assert(pack.temperature == (short)(short) -3815);
                Debug.Assert(pack.press_abs == (float)1.7075957E38F);
                Debug.Assert(pack.press_diff == (float)2.1600659E38F);
            };
            DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)4192223818U;
            p143.temperature = (short)(short) -3815;
            p143.press_abs = (float)1.7075957E38F;
            p143.press_diff = (float)2.1600659E38F;
            LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rates.SequenceEqual(new float[] {1.5119066E38F, 1.5857091E37F, -1.5020893E38F}));
                Debug.Assert(pack.custom_state == (ulong)6538194127601057948L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-3.3236123E38F, -2.4209198E38F, -1.0351665E38F}));
                Debug.Assert(pack.lat == (int) -496553285);
                Debug.Assert(pack.alt == (float) -5.009128E37F);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.4201993E38F, -3.155603E38F, -2.536352E38F}));
                Debug.Assert(pack.lon == (int)342163673);
                Debug.Assert(pack.est_capabilities == (byte)(byte)79);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-8.596201E36F, -2.3614459E38F, -1.3727672E38F}));
                Debug.Assert(pack.timestamp == (ulong)3082121200281866562L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {2.1307957E38F, -7.496114E37F, -1.1405506E37F, -1.4393622E38F}));
            };
            DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lat = (int) -496553285;
            p144.lon = (int)342163673;
            p144.custom_state = (ulong)6538194127601057948L;
            p144.acc_SET(new float[] {2.4201993E38F, -3.155603E38F, -2.536352E38F}, 0) ;
            p144.rates_SET(new float[] {1.5119066E38F, 1.5857091E37F, -1.5020893E38F}, 0) ;
            p144.vel_SET(new float[] {-3.3236123E38F, -2.4209198E38F, -1.0351665E38F}, 0) ;
            p144.position_cov_SET(new float[] {-8.596201E36F, -2.3614459E38F, -1.3727672E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)79;
            p144.alt = (float) -5.009128E37F;
            p144.attitude_q_SET(new float[] {2.1307957E38F, -7.496114E37F, -1.1405506E37F, -1.4393622E38F}, 0) ;
            p144.timestamp = (ulong)3082121200281866562L;
            LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.979794E38F, 1.6412381E38F, 3.7701638E37F}));
                Debug.Assert(pack.pitch_rate == (float) -3.32351E37F);
                Debug.Assert(pack.z_pos == (float) -5.5252235E37F);
                Debug.Assert(pack.x_acc == (float) -1.8116094E38F);
                Debug.Assert(pack.z_vel == (float)2.174195E38F);
                Debug.Assert(pack.y_vel == (float) -5.504125E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.0056446E38F, 2.8235206E37F, 2.9736246E38F, -1.554331E38F}));
                Debug.Assert(pack.x_vel == (float)2.288706E38F);
                Debug.Assert(pack.yaw_rate == (float) -6.912665E37F);
                Debug.Assert(pack.x_pos == (float)2.9951462E38F);
                Debug.Assert(pack.roll_rate == (float)1.8731195E38F);
                Debug.Assert(pack.z_acc == (float) -3.0276022E37F);
                Debug.Assert(pack.y_acc == (float) -1.6888033E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-3.1232688E38F, 2.097259E38F, -1.3616568E38F}));
                Debug.Assert(pack.airspeed == (float) -2.2155618E38F);
                Debug.Assert(pack.y_pos == (float)2.8174178E38F);
                Debug.Assert(pack.time_usec == (ulong)1103396078342144039L);
            };
            DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_acc = (float) -1.6888033E38F;
            p146.y_pos = (float)2.8174178E38F;
            p146.x_pos = (float)2.9951462E38F;
            p146.pos_variance_SET(new float[] {-3.1232688E38F, 2.097259E38F, -1.3616568E38F}, 0) ;
            p146.q_SET(new float[] {-1.0056446E38F, 2.8235206E37F, 2.9736246E38F, -1.554331E38F}, 0) ;
            p146.x_acc = (float) -1.8116094E38F;
            p146.x_vel = (float)2.288706E38F;
            p146.time_usec = (ulong)1103396078342144039L;
            p146.pitch_rate = (float) -3.32351E37F;
            p146.roll_rate = (float)1.8731195E38F;
            p146.z_vel = (float)2.174195E38F;
            p146.yaw_rate = (float) -6.912665E37F;
            p146.vel_variance_SET(new float[] {-2.979794E38F, 1.6412381E38F, 3.7701638E37F}, 0) ;
            p146.airspeed = (float) -2.2155618E38F;
            p146.z_pos = (float) -5.5252235E37F;
            p146.y_vel = (float) -5.504125E37F;
            p146.z_acc = (float) -3.0276022E37F;
            LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 2);
                Debug.Assert(pack.current_consumed == (int)1734188712);
                Debug.Assert(pack.temperature == (short)(short)10372);
                Debug.Assert(pack.current_battery == (short)(short) -31410);
                Debug.Assert(pack.energy_consumed == (int)1550096243);
                Debug.Assert(pack.id == (byte)(byte)103);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)44995, (ushort)52209, (ushort)56803, (ushort)3929, (ushort)14578, (ushort)10628, (ushort)12262, (ushort)8303, (ushort)63459, (ushort)25214}));
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
            };
            DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_remaining = (sbyte)(sbyte) - 2;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.energy_consumed = (int)1550096243;
            p147.voltages_SET(new ushort[] {(ushort)44995, (ushort)52209, (ushort)56803, (ushort)3929, (ushort)14578, (ushort)10628, (ushort)12262, (ushort)8303, (ushort)63459, (ushort)25214}, 0) ;
            p147.temperature = (short)(short)10372;
            p147.id = (byte)(byte)103;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.current_battery = (short)(short) -31410;
            p147.current_consumed = (int)1734188712;
            LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_id == (ushort)(ushort)38461);
                Debug.Assert(pack.os_sw_version == (uint)1456464746U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)203, (byte)140, (byte)49, (byte)124, (byte)137, (byte)167, (byte)127, (byte)60}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)118, (byte)126, (byte)220, (byte)143, (byte)43, (byte)179, (byte)191, (byte)32}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)121, (byte)137, (byte)82, (byte)164, (byte)247, (byte)102, (byte)153, (byte)235}));
                Debug.Assert(pack.board_version == (uint)1745471302U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
                Debug.Assert(pack.uid == (ulong)5324752031742251716L);
                Debug.Assert(pack.flight_sw_version == (uint)3465143925U);
                Debug.Assert(pack.product_id == (ushort)(ushort)32344);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)139, (byte)114, (byte)51, (byte)54, (byte)47, (byte)51, (byte)125, (byte)155, (byte)228, (byte)253, (byte)238, (byte)172, (byte)220, (byte)42, (byte)59, (byte)38, (byte)228, (byte)112}));
                Debug.Assert(pack.middleware_sw_version == (uint)1248837182U);
            };
            DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.board_version = (uint)1745471302U;
            p148.flight_sw_version = (uint)3465143925U;
            p148.os_sw_version = (uint)1456464746U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
            p148.uid = (ulong)5324752031742251716L;
            p148.vendor_id = (ushort)(ushort)38461;
            p148.product_id = (ushort)(ushort)32344;
            p148.uid2_SET(new byte[] {(byte)139, (byte)114, (byte)51, (byte)54, (byte)47, (byte)51, (byte)125, (byte)155, (byte)228, (byte)253, (byte)238, (byte)172, (byte)220, (byte)42, (byte)59, (byte)38, (byte)228, (byte)112}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)118, (byte)126, (byte)220, (byte)143, (byte)43, (byte)179, (byte)191, (byte)32}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)121, (byte)137, (byte)82, (byte)164, (byte)247, (byte)102, (byte)153, (byte)235}, 0) ;
            p148.middleware_sw_version = (uint)1248837182U;
            p148.middleware_custom_version_SET(new byte[] {(byte)203, (byte)140, (byte)49, (byte)124, (byte)137, (byte)167, (byte)127, (byte)60}, 0) ;
            LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size_x == (float)1.677942E38F);
                Debug.Assert(pack.target_num == (byte)(byte)223);
                Debug.Assert(pack.distance == (float)1.2175213E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -3.030541E36F);
                Debug.Assert(pack.angle_x == (float) -3.8434464E37F);
                Debug.Assert(pack.time_usec == (ulong)1998627767895082616L);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.0848579E38F);
                Debug.Assert(pack.angle_y == (float) -9.979687E37F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)250);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.7108193E38F, 3.0006229E38F, -1.3613547E38F, -2.4533603E38F}));
                Debug.Assert(pack.y_TRY(ph) == (float)3.4008212E38F);
                Debug.Assert(pack.size_y == (float) -2.3880002E38F);
            };
            DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)1998627767895082616L;
            p149.size_x = (float)1.677942E38F;
            p149.distance = (float)1.2175213E38F;
            p149.position_valid_SET((byte)(byte)250, PH) ;
            p149.target_num = (byte)(byte)223;
            p149.size_y = (float) -2.3880002E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p149.q_SET(new float[] {-2.7108193E38F, 3.0006229E38F, -1.3613547E38F, -2.4533603E38F}, 0, PH) ;
            p149.y_SET((float)3.4008212E38F, PH) ;
            p149.x_SET((float) -3.030541E36F, PH) ;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.z_SET((float) -2.0848579E38F, PH) ;
            p149.angle_y = (float) -9.979687E37F;
            p149.angle_x = (float) -3.8434464E37F;
            LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.717338E38F);
                Debug.Assert(pack.mag_ratio == (float) -2.2670418E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.8576297E38F);
                Debug.Assert(pack.tas_ratio == (float)3.108093E38F);
                Debug.Assert(pack.time_usec == (ulong)7246863619186061033L);
                Debug.Assert(pack.hagl_ratio == (float) -1.0244713E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.5975727E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -4.7464588E36F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -1.0983378E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            };
            DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
            p230.vel_ratio = (float) -1.5975727E38F;
            p230.mag_ratio = (float) -2.2670418E38F;
            p230.pos_vert_ratio = (float) -2.8576297E38F;
            p230.pos_horiz_accuracy = (float) -1.0983378E38F;
            p230.pos_horiz_ratio = (float) -4.7464588E36F;
            p230.hagl_ratio = (float) -1.0244713E38F;
            p230.tas_ratio = (float)3.108093E38F;
            p230.time_usec = (ulong)7246863619186061033L;
            p230.pos_vert_accuracy = (float) -1.717338E38F;
            LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float) -8.812664E37F);
                Debug.Assert(pack.wind_y == (float)2.4309452E38F);
                Debug.Assert(pack.var_vert == (float)2.8860457E38F);
                Debug.Assert(pack.time_usec == (ulong)6998660351331565132L);
                Debug.Assert(pack.wind_z == (float)1.5370319E38F);
                Debug.Assert(pack.var_horiz == (float) -6.0769527E37F);
                Debug.Assert(pack.wind_alt == (float) -1.2919383E38F);
                Debug.Assert(pack.wind_x == (float) -3.0715472E38F);
                Debug.Assert(pack.vert_accuracy == (float)7.9881596E37F);
            };
            DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -3.0715472E38F;
            p231.vert_accuracy = (float)7.9881596E37F;
            p231.wind_y = (float)2.4309452E38F;
            p231.wind_alt = (float) -1.2919383E38F;
            p231.var_horiz = (float) -6.0769527E37F;
            p231.wind_z = (float)1.5370319E38F;
            p231.time_usec = (ulong)6998660351331565132L;
            p231.horiz_accuracy = (float) -8.812664E37F;
            p231.var_vert = (float)2.8860457E38F;
            LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float)9.962658E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)224);
                Debug.Assert(pack.hdop == (float) -8.1571283E37F);
                Debug.Assert(pack.vert_accuracy == (float) -4.7141477E37F);
                Debug.Assert(pack.vdop == (float)1.8463146E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
                Debug.Assert(pack.fix_type == (byte)(byte)31);
                Debug.Assert(pack.ve == (float)2.5628494E38F);
                Debug.Assert(pack.vn == (float)3.0987642E38F);
                Debug.Assert(pack.time_usec == (ulong)8964199213662516029L);
                Debug.Assert(pack.gps_id == (byte)(byte)59);
                Debug.Assert(pack.time_week_ms == (uint)2114855219U);
                Debug.Assert(pack.lat == (int) -1607990421);
                Debug.Assert(pack.alt == (float)2.5041476E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)44018);
                Debug.Assert(pack.speed_accuracy == (float) -1.5282153E38F);
                Debug.Assert(pack.vd == (float) -1.7653087E38F);
                Debug.Assert(pack.lon == (int) -1307564333);
            };
            DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.alt = (float)2.5041476E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
            p232.fix_type = (byte)(byte)31;
            p232.satellites_visible = (byte)(byte)224;
            p232.time_week_ms = (uint)2114855219U;
            p232.vd = (float) -1.7653087E38F;
            p232.gps_id = (byte)(byte)59;
            p232.vdop = (float)1.8463146E38F;
            p232.time_week = (ushort)(ushort)44018;
            p232.vert_accuracy = (float) -4.7141477E37F;
            p232.hdop = (float) -8.1571283E37F;
            p232.speed_accuracy = (float) -1.5282153E38F;
            p232.time_usec = (ulong)8964199213662516029L;
            p232.vn = (float)3.0987642E38F;
            p232.lat = (int) -1607990421;
            p232.horiz_accuracy = (float)9.962658E37F;
            p232.ve = (float)2.5628494E38F;
            p232.lon = (int) -1307564333;
            LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)83, (byte)255, (byte)60, (byte)237, (byte)48, (byte)100, (byte)91, (byte)26, (byte)118, (byte)205, (byte)120, (byte)116, (byte)183, (byte)156, (byte)5, (byte)208, (byte)16, (byte)159, (byte)34, (byte)180, (byte)120, (byte)204, (byte)147, (byte)194, (byte)212, (byte)186, (byte)102, (byte)89, (byte)10, (byte)16, (byte)89, (byte)130, (byte)11, (byte)253, (byte)30, (byte)244, (byte)76, (byte)164, (byte)223, (byte)196, (byte)14, (byte)208, (byte)20, (byte)75, (byte)28, (byte)252, (byte)30, (byte)91, (byte)219, (byte)220, (byte)159, (byte)37, (byte)211, (byte)30, (byte)144, (byte)144, (byte)216, (byte)109, (byte)172, (byte)189, (byte)215, (byte)146, (byte)93, (byte)16, (byte)207, (byte)234, (byte)12, (byte)126, (byte)175, (byte)205, (byte)238, (byte)233, (byte)194, (byte)244, (byte)10, (byte)35, (byte)190, (byte)38, (byte)211, (byte)67, (byte)151, (byte)20, (byte)186, (byte)154, (byte)161, (byte)173, (byte)129, (byte)22, (byte)181, (byte)43, (byte)26, (byte)139, (byte)201, (byte)229, (byte)220, (byte)127, (byte)73, (byte)89, (byte)247, (byte)217, (byte)88, (byte)199, (byte)249, (byte)78, (byte)23, (byte)66, (byte)106, (byte)246, (byte)241, (byte)15, (byte)193, (byte)97, (byte)178, (byte)115, (byte)221, (byte)254, (byte)206, (byte)43, (byte)33, (byte)226, (byte)191, (byte)126, (byte)33, (byte)3, (byte)93, (byte)23, (byte)111, (byte)143, (byte)34, (byte)233, (byte)158, (byte)97, (byte)233, (byte)136, (byte)156, (byte)150, (byte)211, (byte)147, (byte)177, (byte)137, (byte)153, (byte)172, (byte)231, (byte)0, (byte)116, (byte)69, (byte)47, (byte)194, (byte)144, (byte)94, (byte)213, (byte)135, (byte)234, (byte)245, (byte)70, (byte)147, (byte)193, (byte)139, (byte)64, (byte)229, (byte)113, (byte)219, (byte)3, (byte)232, (byte)168, (byte)73, (byte)92, (byte)161, (byte)245, (byte)159, (byte)27, (byte)223, (byte)100, (byte)216, (byte)215, (byte)100, (byte)129, (byte)93, (byte)64, (byte)121}));
                Debug.Assert(pack.len == (byte)(byte)31);
                Debug.Assert(pack.flags == (byte)(byte)45);
            };
            DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)45;
            p233.data__SET(new byte[] {(byte)83, (byte)255, (byte)60, (byte)237, (byte)48, (byte)100, (byte)91, (byte)26, (byte)118, (byte)205, (byte)120, (byte)116, (byte)183, (byte)156, (byte)5, (byte)208, (byte)16, (byte)159, (byte)34, (byte)180, (byte)120, (byte)204, (byte)147, (byte)194, (byte)212, (byte)186, (byte)102, (byte)89, (byte)10, (byte)16, (byte)89, (byte)130, (byte)11, (byte)253, (byte)30, (byte)244, (byte)76, (byte)164, (byte)223, (byte)196, (byte)14, (byte)208, (byte)20, (byte)75, (byte)28, (byte)252, (byte)30, (byte)91, (byte)219, (byte)220, (byte)159, (byte)37, (byte)211, (byte)30, (byte)144, (byte)144, (byte)216, (byte)109, (byte)172, (byte)189, (byte)215, (byte)146, (byte)93, (byte)16, (byte)207, (byte)234, (byte)12, (byte)126, (byte)175, (byte)205, (byte)238, (byte)233, (byte)194, (byte)244, (byte)10, (byte)35, (byte)190, (byte)38, (byte)211, (byte)67, (byte)151, (byte)20, (byte)186, (byte)154, (byte)161, (byte)173, (byte)129, (byte)22, (byte)181, (byte)43, (byte)26, (byte)139, (byte)201, (byte)229, (byte)220, (byte)127, (byte)73, (byte)89, (byte)247, (byte)217, (byte)88, (byte)199, (byte)249, (byte)78, (byte)23, (byte)66, (byte)106, (byte)246, (byte)241, (byte)15, (byte)193, (byte)97, (byte)178, (byte)115, (byte)221, (byte)254, (byte)206, (byte)43, (byte)33, (byte)226, (byte)191, (byte)126, (byte)33, (byte)3, (byte)93, (byte)23, (byte)111, (byte)143, (byte)34, (byte)233, (byte)158, (byte)97, (byte)233, (byte)136, (byte)156, (byte)150, (byte)211, (byte)147, (byte)177, (byte)137, (byte)153, (byte)172, (byte)231, (byte)0, (byte)116, (byte)69, (byte)47, (byte)194, (byte)144, (byte)94, (byte)213, (byte)135, (byte)234, (byte)245, (byte)70, (byte)147, (byte)193, (byte)139, (byte)64, (byte)229, (byte)113, (byte)219, (byte)3, (byte)232, (byte)168, (byte)73, (byte)92, (byte)161, (byte)245, (byte)159, (byte)27, (byte)223, (byte)100, (byte)216, (byte)215, (byte)100, (byte)129, (byte)93, (byte)64, (byte)121}, 0) ;
            p233.len = (byte)(byte)31;
            LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed_sp == (byte)(byte)73);
                Debug.Assert(pack.heading_sp == (short)(short)28367);
                Debug.Assert(pack.airspeed == (byte)(byte)100);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 35);
                Debug.Assert(pack.altitude_sp == (short)(short) -30461);
                Debug.Assert(pack.heading == (ushort)(ushort)55825);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)30);
                Debug.Assert(pack.altitude_amsl == (short)(short) -9021);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.failsafe == (byte)(byte)123);
                Debug.Assert(pack.groundspeed == (byte)(byte)101);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.custom_mode == (uint)586735593U);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)62842);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)103);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)73);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
                Debug.Assert(pack.longitude == (int)413774357);
                Debug.Assert(pack.gps_nsat == (byte)(byte)118);
                Debug.Assert(pack.roll == (short)(short) -3709);
                Debug.Assert(pack.latitude == (int)534033244);
                Debug.Assert(pack.pitch == (short)(short) -26906);
                Debug.Assert(pack.wp_num == (byte)(byte)133);
                Debug.Assert(pack.battery_remaining == (byte)(byte)64);
            };
            DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
            p234.battery_remaining = (byte)(byte)64;
            p234.pitch = (short)(short) -26906;
            p234.temperature = (sbyte)(sbyte) - 35;
            p234.wp_num = (byte)(byte)133;
            p234.gps_nsat = (byte)(byte)118;
            p234.climb_rate = (sbyte)(sbyte)103;
            p234.failsafe = (byte)(byte)123;
            p234.throttle = (sbyte)(sbyte)73;
            p234.airspeed_sp = (byte)(byte)73;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.temperature_air = (sbyte)(sbyte)30;
            p234.altitude_amsl = (short)(short) -9021;
            p234.altitude_sp = (short)(short) -30461;
            p234.roll = (short)(short) -3709;
            p234.airspeed = (byte)(byte)100;
            p234.groundspeed = (byte)(byte)101;
            p234.wp_distance = (ushort)(ushort)62842;
            p234.longitude = (int)413774357;
            p234.latitude = (int)534033244;
            p234.heading = (ushort)(ushort)55825;
            p234.custom_mode = (uint)586735593U;
            p234.heading_sp = (short)(short)28367;
            LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3846049309291496848L);
                Debug.Assert(pack.clipping_1 == (uint)608530283U);
                Debug.Assert(pack.clipping_0 == (uint)750178657U);
                Debug.Assert(pack.vibration_y == (float)3.0076597E38F);
                Debug.Assert(pack.vibration_z == (float) -1.767054E38F);
                Debug.Assert(pack.vibration_x == (float)3.3077298E38F);
                Debug.Assert(pack.clipping_2 == (uint)965223231U);
            };
            DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)750178657U;
            p241.vibration_z = (float) -1.767054E38F;
            p241.clipping_1 = (uint)608530283U;
            p241.vibration_x = (float)3.3077298E38F;
            p241.time_usec = (ulong)3846049309291496848L;
            p241.clipping_2 = (uint)965223231U;
            p241.vibration_y = (float)3.0076597E38F;
            LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -878583113);
                Debug.Assert(pack.longitude == (int) -438809914);
                Debug.Assert(pack.approach_y == (float)8.741679E37F);
                Debug.Assert(pack.approach_x == (float)3.2693601E38F);
                Debug.Assert(pack.altitude == (int) -56170648);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.3849506E38F, 2.9798198E38F, -2.105734E38F, 2.6449566E38F}));
                Debug.Assert(pack.x == (float)2.3334624E38F);
                Debug.Assert(pack.approach_z == (float) -3.0493518E38F);
                Debug.Assert(pack.y == (float) -1.1032287E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3153601236828219938L);
                Debug.Assert(pack.z == (float) -2.098418E38F);
            };
            DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.x = (float)2.3334624E38F;
            p242.q_SET(new float[] {-1.3849506E38F, 2.9798198E38F, -2.105734E38F, 2.6449566E38F}, 0) ;
            p242.approach_z = (float) -3.0493518E38F;
            p242.approach_x = (float)3.2693601E38F;
            p242.altitude = (int) -56170648;
            p242.latitude = (int) -878583113;
            p242.time_usec_SET((ulong)3153601236828219938L, PH) ;
            p242.longitude = (int) -438809914;
            p242.y = (float) -1.1032287E38F;
            p242.z = (float) -2.098418E38F;
            p242.approach_y = (float)8.741679E37F;
            LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float)2.7753313E38F);
                Debug.Assert(pack.x == (float) -2.1924532E38F);
                Debug.Assert(pack.latitude == (int)1222261166);
                Debug.Assert(pack.target_system == (byte)(byte)11);
                Debug.Assert(pack.altitude == (int) -694506555);
                Debug.Assert(pack.z == (float) -2.6153585E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.6373661E38F, -2.5905795E38F, -5.571544E37F, -1.7710198E38F}));
                Debug.Assert(pack.y == (float)2.773285E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2041175928137213223L);
                Debug.Assert(pack.approach_x == (float)5.114099E37F);
                Debug.Assert(pack.approach_z == (float)2.7876156E38F);
                Debug.Assert(pack.longitude == (int) -1301095076);
            };
            DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.latitude = (int)1222261166;
            p243.approach_y = (float)2.7753313E38F;
            p243.y = (float)2.773285E38F;
            p243.altitude = (int) -694506555;
            p243.q_SET(new float[] {1.6373661E38F, -2.5905795E38F, -5.571544E37F, -1.7710198E38F}, 0) ;
            p243.z = (float) -2.6153585E38F;
            p243.approach_x = (float)5.114099E37F;
            p243.time_usec_SET((ulong)2041175928137213223L, PH) ;
            p243.target_system = (byte)(byte)11;
            p243.x = (float) -2.1924532E38F;
            p243.approach_z = (float)2.7876156E38F;
            p243.longitude = (int) -1301095076;
            LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1324014884);
                Debug.Assert(pack.message_id == (ushort)(ushort)627);
            };
            DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1324014884;
            p244.message_id = (ushort)(ushort)627;
            LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.squawk == (ushort)(ushort)14265);
                Debug.Assert(pack.ICAO_address == (uint)1424715048U);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
                Debug.Assert(pack.lat == (int) -403982242);
                Debug.Assert(pack.ver_velocity == (short)(short)8175);
                Debug.Assert(pack.lon == (int)867076060);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)12462);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
                Debug.Assert(pack.tslc == (byte)(byte)45);
                Debug.Assert(pack.callsign_LEN(ph) == 6);
                Debug.Assert(pack.callsign_TRY(ph).Equals("xmljra"));
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.heading == (ushort)(ushort)61610);
                Debug.Assert(pack.altitude == (int)1310027860);
            };
            DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.callsign_SET("xmljra", PH) ;
            p246.ICAO_address = (uint)1424715048U;
            p246.lon = (int)867076060;
            p246.hor_velocity = (ushort)(ushort)12462;
            p246.altitude = (int)1310027860;
            p246.heading = (ushort)(ushort)61610;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p246.tslc = (byte)(byte)45;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.ver_velocity = (short)(short)8175;
            p246.lat = (int) -403982242;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)14265;
            LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float)1.0414753E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float) -3.147503E37F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.id == (uint)597230905U);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.476841E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            };
            DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)597230905U;
            p247.time_to_minimum_delta = (float)1.0414753E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.altitude_minimum_delta = (float) -3.147503E37F;
            p247.horizontal_minimum_delta = (float)2.476841E38F;
            LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.target_network == (byte)(byte)220);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.message_type == (ushort)(ushort)62273);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)52, (byte)12, (byte)37, (byte)17, (byte)127, (byte)215, (byte)111, (byte)127, (byte)7, (byte)18, (byte)75, (byte)39, (byte)56, (byte)110, (byte)213, (byte)21, (byte)152, (byte)15, (byte)192, (byte)70, (byte)227, (byte)237, (byte)184, (byte)224, (byte)78, (byte)223, (byte)201, (byte)55, (byte)35, (byte)10, (byte)153, (byte)96, (byte)188, (byte)252, (byte)80, (byte)90, (byte)104, (byte)178, (byte)154, (byte)238, (byte)192, (byte)89, (byte)8, (byte)165, (byte)157, (byte)83, (byte)254, (byte)36, (byte)190, (byte)243, (byte)138, (byte)42, (byte)20, (byte)233, (byte)72, (byte)196, (byte)89, (byte)172, (byte)245, (byte)201, (byte)154, (byte)55, (byte)130, (byte)167, (byte)93, (byte)222, (byte)175, (byte)203, (byte)171, (byte)243, (byte)85, (byte)59, (byte)123, (byte)134, (byte)48, (byte)118, (byte)252, (byte)150, (byte)159, (byte)54, (byte)33, (byte)123, (byte)170, (byte)232, (byte)189, (byte)68, (byte)101, (byte)97, (byte)141, (byte)136, (byte)178, (byte)255, (byte)12, (byte)251, (byte)142, (byte)124, (byte)146, (byte)87, (byte)66, (byte)173, (byte)232, (byte)159, (byte)72, (byte)124, (byte)20, (byte)54, (byte)126, (byte)159, (byte)78, (byte)160, (byte)175, (byte)80, (byte)66, (byte)81, (byte)139, (byte)123, (byte)118, (byte)254, (byte)166, (byte)22, (byte)0, (byte)157, (byte)28, (byte)42, (byte)81, (byte)85, (byte)242, (byte)136, (byte)121, (byte)253, (byte)241, (byte)74, (byte)119, (byte)159, (byte)200, (byte)7, (byte)11, (byte)228, (byte)137, (byte)92, (byte)132, (byte)85, (byte)159, (byte)116, (byte)4, (byte)124, (byte)96, (byte)194, (byte)63, (byte)138, (byte)204, (byte)109, (byte)5, (byte)137, (byte)63, (byte)74, (byte)189, (byte)29, (byte)51, (byte)83, (byte)251, (byte)22, (byte)172, (byte)230, (byte)203, (byte)172, (byte)241, (byte)31, (byte)220, (byte)31, (byte)224, (byte)79, (byte)88, (byte)1, (byte)163, (byte)210, (byte)209, (byte)48, (byte)127, (byte)112, (byte)207, (byte)82, (byte)91, (byte)63, (byte)64, (byte)239, (byte)203, (byte)69, (byte)93, (byte)160, (byte)198, (byte)204, (byte)177, (byte)224, (byte)75, (byte)115, (byte)74, (byte)41, (byte)112, (byte)7, (byte)61, (byte)232, (byte)21, (byte)47, (byte)89, (byte)5, (byte)30, (byte)158, (byte)104, (byte)100, (byte)72, (byte)98, (byte)14, (byte)149, (byte)46, (byte)19, (byte)204, (byte)30, (byte)105, (byte)58, (byte)99, (byte)58, (byte)236, (byte)160, (byte)228, (byte)57, (byte)68, (byte)165, (byte)84, (byte)236, (byte)1, (byte)27, (byte)34, (byte)130, (byte)64, (byte)130, (byte)252, (byte)29, (byte)173, (byte)210, (byte)135, (byte)1, (byte)55, (byte)145, (byte)126, (byte)89, (byte)154, (byte)106, (byte)102}));
            };
            DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)220;
            p248.message_type = (ushort)(ushort)62273;
            p248.target_component = (byte)(byte)226;
            p248.target_system = (byte)(byte)58;
            p248.payload_SET(new byte[] {(byte)52, (byte)12, (byte)37, (byte)17, (byte)127, (byte)215, (byte)111, (byte)127, (byte)7, (byte)18, (byte)75, (byte)39, (byte)56, (byte)110, (byte)213, (byte)21, (byte)152, (byte)15, (byte)192, (byte)70, (byte)227, (byte)237, (byte)184, (byte)224, (byte)78, (byte)223, (byte)201, (byte)55, (byte)35, (byte)10, (byte)153, (byte)96, (byte)188, (byte)252, (byte)80, (byte)90, (byte)104, (byte)178, (byte)154, (byte)238, (byte)192, (byte)89, (byte)8, (byte)165, (byte)157, (byte)83, (byte)254, (byte)36, (byte)190, (byte)243, (byte)138, (byte)42, (byte)20, (byte)233, (byte)72, (byte)196, (byte)89, (byte)172, (byte)245, (byte)201, (byte)154, (byte)55, (byte)130, (byte)167, (byte)93, (byte)222, (byte)175, (byte)203, (byte)171, (byte)243, (byte)85, (byte)59, (byte)123, (byte)134, (byte)48, (byte)118, (byte)252, (byte)150, (byte)159, (byte)54, (byte)33, (byte)123, (byte)170, (byte)232, (byte)189, (byte)68, (byte)101, (byte)97, (byte)141, (byte)136, (byte)178, (byte)255, (byte)12, (byte)251, (byte)142, (byte)124, (byte)146, (byte)87, (byte)66, (byte)173, (byte)232, (byte)159, (byte)72, (byte)124, (byte)20, (byte)54, (byte)126, (byte)159, (byte)78, (byte)160, (byte)175, (byte)80, (byte)66, (byte)81, (byte)139, (byte)123, (byte)118, (byte)254, (byte)166, (byte)22, (byte)0, (byte)157, (byte)28, (byte)42, (byte)81, (byte)85, (byte)242, (byte)136, (byte)121, (byte)253, (byte)241, (byte)74, (byte)119, (byte)159, (byte)200, (byte)7, (byte)11, (byte)228, (byte)137, (byte)92, (byte)132, (byte)85, (byte)159, (byte)116, (byte)4, (byte)124, (byte)96, (byte)194, (byte)63, (byte)138, (byte)204, (byte)109, (byte)5, (byte)137, (byte)63, (byte)74, (byte)189, (byte)29, (byte)51, (byte)83, (byte)251, (byte)22, (byte)172, (byte)230, (byte)203, (byte)172, (byte)241, (byte)31, (byte)220, (byte)31, (byte)224, (byte)79, (byte)88, (byte)1, (byte)163, (byte)210, (byte)209, (byte)48, (byte)127, (byte)112, (byte)207, (byte)82, (byte)91, (byte)63, (byte)64, (byte)239, (byte)203, (byte)69, (byte)93, (byte)160, (byte)198, (byte)204, (byte)177, (byte)224, (byte)75, (byte)115, (byte)74, (byte)41, (byte)112, (byte)7, (byte)61, (byte)232, (byte)21, (byte)47, (byte)89, (byte)5, (byte)30, (byte)158, (byte)104, (byte)100, (byte)72, (byte)98, (byte)14, (byte)149, (byte)46, (byte)19, (byte)204, (byte)30, (byte)105, (byte)58, (byte)99, (byte)58, (byte)236, (byte)160, (byte)228, (byte)57, (byte)68, (byte)165, (byte)84, (byte)236, (byte)1, (byte)27, (byte)34, (byte)130, (byte)64, (byte)130, (byte)252, (byte)29, (byte)173, (byte)210, (byte)135, (byte)1, (byte)55, (byte)145, (byte)126, (byte)89, (byte)154, (byte)106, (byte)102}, 0) ;
            LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)132);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 16, (sbyte) - 90, (sbyte)8, (sbyte)9, (sbyte) - 86, (sbyte)123, (sbyte) - 44, (sbyte) - 22, (sbyte) - 115, (sbyte) - 10, (sbyte) - 27, (sbyte) - 19, (sbyte) - 72, (sbyte)114, (sbyte)66, (sbyte) - 72, (sbyte)40, (sbyte)23, (sbyte) - 55, (sbyte)17, (sbyte)7, (sbyte) - 122, (sbyte)121, (sbyte) - 66, (sbyte)18, (sbyte) - 5, (sbyte)76, (sbyte)123, (sbyte)29, (sbyte) - 69, (sbyte) - 82, (sbyte) - 55}));
                Debug.Assert(pack.type == (byte)(byte)68);
                Debug.Assert(pack.address == (ushort)(ushort)39233);
            };
            DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)132;
            p249.address = (ushort)(ushort)39233;
            p249.value_SET(new sbyte[] {(sbyte) - 16, (sbyte) - 90, (sbyte)8, (sbyte)9, (sbyte) - 86, (sbyte)123, (sbyte) - 44, (sbyte) - 22, (sbyte) - 115, (sbyte) - 10, (sbyte) - 27, (sbyte) - 19, (sbyte) - 72, (sbyte)114, (sbyte)66, (sbyte) - 72, (sbyte)40, (sbyte)23, (sbyte) - 55, (sbyte)17, (sbyte)7, (sbyte) - 122, (sbyte)121, (sbyte) - 66, (sbyte)18, (sbyte) - 5, (sbyte)76, (sbyte)123, (sbyte)29, (sbyte) - 69, (sbyte) - 82, (sbyte) - 55}, 0) ;
            p249.type = (byte)(byte)68;
            LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3767208795415901466L);
                Debug.Assert(pack.z == (float)1.1512868E38F);
                Debug.Assert(pack.y == (float) -2.5685086E38F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("rivCnncjke"));
                Debug.Assert(pack.x == (float)1.4422527E37F);
            };
            DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float)1.1512868E38F;
            p250.y = (float) -2.5685086E38F;
            p250.time_usec = (ulong)3767208795415901466L;
            p250.x = (float)1.4422527E37F;
            p250.name_SET("rivCnncjke", PH) ;
            LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4212056206U);
                Debug.Assert(pack.value == (float)1.7893505E38F);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("srNX"));
            };
            DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("srNX", PH) ;
            p251.time_boot_ms = (uint)4212056206U;
            p251.value = (float)1.7893505E38F;
            LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)1524658939);
                Debug.Assert(pack.time_boot_ms == (uint)762898947U);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("tfmiijqi"));
            };
            DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)1524658939;
            p252.name_SET("tfmiijqi", PH) ;
            p252.time_boot_ms = (uint)762898947U;
            LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO);
                Debug.Assert(pack.text_LEN(ph) == 24);
                Debug.Assert(pack.text_TRY(ph).Equals("xazcymycyxsowbggcitncqio"));
            };
            DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("xazcymycyxsowbggcitncqio", PH) ;
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3553824121U);
                Debug.Assert(pack.value == (float)5.540176E37F);
                Debug.Assert(pack.ind == (byte)(byte)115);
            };
            DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)115;
            p254.value = (float)5.540176E37F;
            p254.time_boot_ms = (uint)3553824121U;
            LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)44);
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.initial_timestamp == (ulong)6065227304389985332L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)103, (byte)23, (byte)2, (byte)215, (byte)59, (byte)161, (byte)27, (byte)5, (byte)86, (byte)205, (byte)174, (byte)54, (byte)7, (byte)88, (byte)139, (byte)131, (byte)68, (byte)50, (byte)3, (byte)93, (byte)178, (byte)239, (byte)38, (byte)223, (byte)45, (byte)141, (byte)207, (byte)182, (byte)50, (byte)227, (byte)249, (byte)121}));
            };
            DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)6065227304389985332L;
            p256.target_component = (byte)(byte)21;
            p256.target_system = (byte)(byte)44;
            p256.secret_key_SET(new byte[] {(byte)103, (byte)23, (byte)2, (byte)215, (byte)59, (byte)161, (byte)27, (byte)5, (byte)86, (byte)205, (byte)174, (byte)54, (byte)7, (byte)88, (byte)139, (byte)131, (byte)68, (byte)50, (byte)3, (byte)93, (byte)178, (byte)239, (byte)38, (byte)223, (byte)45, (byte)141, (byte)207, (byte)182, (byte)50, (byte)227, (byte)249, (byte)121}, 0) ;
            LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2091596276U);
                Debug.Assert(pack.last_change_ms == (uint)2779124752U);
                Debug.Assert(pack.state == (byte)(byte)60);
            };
            DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2779124752U;
            p257.state = (byte)(byte)60;
            p257.time_boot_ms = (uint)2091596276U;
            LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)14);
                Debug.Assert(pack.tune_LEN(ph) == 28);
                Debug.Assert(pack.tune_TRY(ph).Equals("dBdyygjqyitvyomzqkkkcqvrtFCw"));
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)14;
            p258.target_component = (byte)(byte)13;
            p258.tune_SET("dBdyygjqyitvyomzqkkkcqvrtFCw", PH) ;
            LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)175, (byte)70, (byte)77, (byte)58, (byte)179, (byte)61, (byte)77, (byte)44, (byte)197, (byte)223, (byte)14, (byte)178, (byte)32, (byte)137, (byte)169, (byte)208, (byte)254, (byte)249, (byte)169, (byte)64, (byte)219, (byte)164, (byte)5, (byte)96, (byte)61, (byte)234, (byte)92, (byte)157, (byte)226, (byte)18, (byte)118, (byte)201}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)40898);
                Debug.Assert(pack.sensor_size_v == (float) -7.368203E37F);
                Debug.Assert(pack.time_boot_ms == (uint)476139190U);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 101);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("xrHnJanNoMrzxdnhtdetTmuvicqcJsgawlmncsipZbnrRwdydDomsBraecjUuNdasXbEciwscttkxauquggepksMchehywxfwfoxa"));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)35600);
                Debug.Assert(pack.sensor_size_h == (float) -1.6114041E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)150);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)149, (byte)99, (byte)194, (byte)26, (byte)96, (byte)187, (byte)76, (byte)194, (byte)145, (byte)136, (byte)236, (byte)165, (byte)100, (byte)197, (byte)32, (byte)24, (byte)110, (byte)58, (byte)97, (byte)68, (byte)141, (byte)32, (byte)174, (byte)81, (byte)47, (byte)171, (byte)29, (byte)8, (byte)114, (byte)29, (byte)12, (byte)167}));
                Debug.Assert(pack.firmware_version == (uint)3903434009U);
                Debug.Assert(pack.focal_length == (float) -1.5584554E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)63280);
            };
            DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)476139190U;
            p259.resolution_h = (ushort)(ushort)40898;
            p259.model_name_SET(new byte[] {(byte)175, (byte)70, (byte)77, (byte)58, (byte)179, (byte)61, (byte)77, (byte)44, (byte)197, (byte)223, (byte)14, (byte)178, (byte)32, (byte)137, (byte)169, (byte)208, (byte)254, (byte)249, (byte)169, (byte)64, (byte)219, (byte)164, (byte)5, (byte)96, (byte)61, (byte)234, (byte)92, (byte)157, (byte)226, (byte)18, (byte)118, (byte)201}, 0) ;
            p259.lens_id = (byte)(byte)150;
            p259.firmware_version = (uint)3903434009U;
            p259.resolution_v = (ushort)(ushort)63280;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
            p259.focal_length = (float) -1.5584554E38F;
            p259.sensor_size_h = (float) -1.6114041E38F;
            p259.cam_definition_version = (ushort)(ushort)35600;
            p259.cam_definition_uri_SET("xrHnJanNoMrzxdnhtdetTmuvicqcJsgawlmncsipZbnrRwdydDomsBraecjUuNdasXbEciwscttkxauquggepksMchehywxfwfoxa", PH) ;
            p259.sensor_size_v = (float) -7.368203E37F;
            p259.vendor_name_SET(new byte[] {(byte)149, (byte)99, (byte)194, (byte)26, (byte)96, (byte)187, (byte)76, (byte)194, (byte)145, (byte)136, (byte)236, (byte)165, (byte)100, (byte)197, (byte)32, (byte)24, (byte)110, (byte)58, (byte)97, (byte)68, (byte)141, (byte)32, (byte)174, (byte)81, (byte)47, (byte)171, (byte)29, (byte)8, (byte)114, (byte)29, (byte)12, (byte)167}, 0) ;
            LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)1161825764U);
            };
            DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            p260.time_boot_ms = (uint)1161825764U;
            LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.total_capacity == (float) -6.239497E37F);
                Debug.Assert(pack.storage_count == (byte)(byte)251);
                Debug.Assert(pack.time_boot_ms == (uint)621519505U);
                Debug.Assert(pack.write_speed == (float) -7.8982826E37F);
                Debug.Assert(pack.read_speed == (float)4.3361246E37F);
                Debug.Assert(pack.available_capacity == (float) -1.8838793E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)102);
                Debug.Assert(pack.used_capacity == (float)2.6373008E38F);
                Debug.Assert(pack.status == (byte)(byte)80);
            };
            DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -6.239497E37F;
            p261.available_capacity = (float) -1.8838793E38F;
            p261.write_speed = (float) -7.8982826E37F;
            p261.read_speed = (float)4.3361246E37F;
            p261.used_capacity = (float)2.6373008E38F;
            p261.status = (byte)(byte)80;
            p261.storage_count = (byte)(byte)251;
            p261.time_boot_ms = (uint)621519505U;
            p261.storage_id = (byte)(byte)102;
            LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)216);
                Debug.Assert(pack.recording_time_ms == (uint)408054317U);
                Debug.Assert(pack.image_interval == (float)2.9924785E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1099127098U);
                Debug.Assert(pack.image_status == (byte)(byte)202);
                Debug.Assert(pack.available_capacity == (float)1.2830727E38F);
            };
            DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)216;
            p262.image_status = (byte)(byte)202;
            p262.time_boot_ms = (uint)1099127098U;
            p262.image_interval = (float)2.9924785E38F;
            p262.available_capacity = (float)1.2830727E38F;
            p262.recording_time_ms = (uint)408054317U;
            LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)7936108013699693031L);
                Debug.Assert(pack.relative_alt == (int) -258684078);
                Debug.Assert(pack.image_index == (int) -1912095721);
                Debug.Assert(pack.file_url_LEN(ph) == 24);
                Debug.Assert(pack.file_url_TRY(ph).Equals("PKFsjyvxjczfrdtdevparsWf"));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 109);
                Debug.Assert(pack.lat == (int) -743024850);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.789503E38F, -2.001359E38F, -1.3746807E38F, -1.1621391E38F}));
                Debug.Assert(pack.lon == (int) -1144338539);
                Debug.Assert(pack.time_boot_ms == (uint)2592322332U);
                Debug.Assert(pack.camera_id == (byte)(byte)203);
                Debug.Assert(pack.alt == (int)61834189);
            };
            DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2592322332U;
            p263.lon = (int) -1144338539;
            p263.q_SET(new float[] {2.789503E38F, -2.001359E38F, -1.3746807E38F, -1.1621391E38F}, 0) ;
            p263.time_utc = (ulong)7936108013699693031L;
            p263.lat = (int) -743024850;
            p263.alt = (int)61834189;
            p263.camera_id = (byte)(byte)203;
            p263.image_index = (int) -1912095721;
            p263.file_url_SET("PKFsjyvxjczfrdtdevparsWf", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 109;
            p263.relative_alt = (int) -258684078;
            LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)7675474084238700842L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5792916248189761230L);
                Debug.Assert(pack.flight_uuid == (ulong)6411747983171792272L);
                Debug.Assert(pack.time_boot_ms == (uint)1903126336U);
            };
            DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)6411747983171792272L;
            p264.arming_time_utc = (ulong)7675474084238700842L;
            p264.time_boot_ms = (uint)1903126336U;
            p264.takeoff_time_utc = (ulong)5792916248189761230L;
            LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)9.37018E37F);
                Debug.Assert(pack.pitch == (float) -2.7140746E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1637954876U);
                Debug.Assert(pack.yaw == (float) -2.5257753E37F);
            };
            DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1637954876U;
            p265.yaw = (float) -2.5257753E37F;
            p265.pitch = (float) -2.7140746E38F;
            p265.roll = (float)9.37018E37F;
            LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)62, (byte)38, (byte)237, (byte)39, (byte)82, (byte)99, (byte)85, (byte)198, (byte)219, (byte)81, (byte)30, (byte)89, (byte)74, (byte)231, (byte)26, (byte)25, (byte)29, (byte)76, (byte)0, (byte)50, (byte)152, (byte)248, (byte)255, (byte)203, (byte)69, (byte)171, (byte)38, (byte)167, (byte)82, (byte)157, (byte)28, (byte)148, (byte)56, (byte)118, (byte)96, (byte)105, (byte)219, (byte)168, (byte)189, (byte)105, (byte)183, (byte)178, (byte)249, (byte)154, (byte)149, (byte)13, (byte)95, (byte)189, (byte)120, (byte)172, (byte)112, (byte)72, (byte)69, (byte)139, (byte)28, (byte)173, (byte)34, (byte)118, (byte)192, (byte)88, (byte)235, (byte)204, (byte)244, (byte)113, (byte)160, (byte)66, (byte)159, (byte)67, (byte)252, (byte)60, (byte)117, (byte)55, (byte)54, (byte)0, (byte)194, (byte)166, (byte)113, (byte)250, (byte)123, (byte)210, (byte)25, (byte)122, (byte)122, (byte)254, (byte)39, (byte)18, (byte)197, (byte)227, (byte)154, (byte)99, (byte)79, (byte)136, (byte)10, (byte)255, (byte)221, (byte)90, (byte)85, (byte)92, (byte)8, (byte)243, (byte)149, (byte)221, (byte)198, (byte)168, (byte)66, (byte)74, (byte)178, (byte)2, (byte)235, (byte)102, (byte)183, (byte)76, (byte)9, (byte)154, (byte)112, (byte)51, (byte)93, (byte)243, (byte)63, (byte)251, (byte)135, (byte)132, (byte)144, (byte)226, (byte)28, (byte)117, (byte)241, (byte)111, (byte)181, (byte)242, (byte)201, (byte)4, (byte)150, (byte)216, (byte)240, (byte)1, (byte)252, (byte)0, (byte)255, (byte)115, (byte)126, (byte)34, (byte)172, (byte)194, (byte)253, (byte)70, (byte)92, (byte)139, (byte)242, (byte)18, (byte)99, (byte)118, (byte)165, (byte)26, (byte)243, (byte)193, (byte)112, (byte)39, (byte)139, (byte)243, (byte)33, (byte)46, (byte)197, (byte)182, (byte)165, (byte)62, (byte)153, (byte)79, (byte)233, (byte)200, (byte)42, (byte)53, (byte)36, (byte)54, (byte)45, (byte)61, (byte)12, (byte)163, (byte)65, (byte)74, (byte)217, (byte)69, (byte)36, (byte)166, (byte)209, (byte)67, (byte)79, (byte)247, (byte)104, (byte)80, (byte)82, (byte)87, (byte)227, (byte)73, (byte)146, (byte)119, (byte)85, (byte)6, (byte)127, (byte)81, (byte)80, (byte)209, (byte)207, (byte)183, (byte)149, (byte)102, (byte)79, (byte)132, (byte)176, (byte)215, (byte)238, (byte)242, (byte)43, (byte)197, (byte)28, (byte)81, (byte)203, (byte)15, (byte)55, (byte)125, (byte)224, (byte)36, (byte)173, (byte)253, (byte)41, (byte)68, (byte)142, (byte)86, (byte)248, (byte)185, (byte)162, (byte)158, (byte)101, (byte)210, (byte)92, (byte)0, (byte)156, (byte)98, (byte)241, (byte)97, (byte)52, (byte)78, (byte)132, (byte)186, (byte)249, (byte)224, (byte)228, (byte)205, (byte)5}));
                Debug.Assert(pack.sequence == (ushort)(ushort)29949);
                Debug.Assert(pack.length == (byte)(byte)107);
                Debug.Assert(pack.target_component == (byte)(byte)90);
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.first_message_offset == (byte)(byte)127);
            };
            DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.length = (byte)(byte)107;
            p266.target_component = (byte)(byte)90;
            p266.target_system = (byte)(byte)91;
            p266.first_message_offset = (byte)(byte)127;
            p266.data__SET(new byte[] {(byte)62, (byte)38, (byte)237, (byte)39, (byte)82, (byte)99, (byte)85, (byte)198, (byte)219, (byte)81, (byte)30, (byte)89, (byte)74, (byte)231, (byte)26, (byte)25, (byte)29, (byte)76, (byte)0, (byte)50, (byte)152, (byte)248, (byte)255, (byte)203, (byte)69, (byte)171, (byte)38, (byte)167, (byte)82, (byte)157, (byte)28, (byte)148, (byte)56, (byte)118, (byte)96, (byte)105, (byte)219, (byte)168, (byte)189, (byte)105, (byte)183, (byte)178, (byte)249, (byte)154, (byte)149, (byte)13, (byte)95, (byte)189, (byte)120, (byte)172, (byte)112, (byte)72, (byte)69, (byte)139, (byte)28, (byte)173, (byte)34, (byte)118, (byte)192, (byte)88, (byte)235, (byte)204, (byte)244, (byte)113, (byte)160, (byte)66, (byte)159, (byte)67, (byte)252, (byte)60, (byte)117, (byte)55, (byte)54, (byte)0, (byte)194, (byte)166, (byte)113, (byte)250, (byte)123, (byte)210, (byte)25, (byte)122, (byte)122, (byte)254, (byte)39, (byte)18, (byte)197, (byte)227, (byte)154, (byte)99, (byte)79, (byte)136, (byte)10, (byte)255, (byte)221, (byte)90, (byte)85, (byte)92, (byte)8, (byte)243, (byte)149, (byte)221, (byte)198, (byte)168, (byte)66, (byte)74, (byte)178, (byte)2, (byte)235, (byte)102, (byte)183, (byte)76, (byte)9, (byte)154, (byte)112, (byte)51, (byte)93, (byte)243, (byte)63, (byte)251, (byte)135, (byte)132, (byte)144, (byte)226, (byte)28, (byte)117, (byte)241, (byte)111, (byte)181, (byte)242, (byte)201, (byte)4, (byte)150, (byte)216, (byte)240, (byte)1, (byte)252, (byte)0, (byte)255, (byte)115, (byte)126, (byte)34, (byte)172, (byte)194, (byte)253, (byte)70, (byte)92, (byte)139, (byte)242, (byte)18, (byte)99, (byte)118, (byte)165, (byte)26, (byte)243, (byte)193, (byte)112, (byte)39, (byte)139, (byte)243, (byte)33, (byte)46, (byte)197, (byte)182, (byte)165, (byte)62, (byte)153, (byte)79, (byte)233, (byte)200, (byte)42, (byte)53, (byte)36, (byte)54, (byte)45, (byte)61, (byte)12, (byte)163, (byte)65, (byte)74, (byte)217, (byte)69, (byte)36, (byte)166, (byte)209, (byte)67, (byte)79, (byte)247, (byte)104, (byte)80, (byte)82, (byte)87, (byte)227, (byte)73, (byte)146, (byte)119, (byte)85, (byte)6, (byte)127, (byte)81, (byte)80, (byte)209, (byte)207, (byte)183, (byte)149, (byte)102, (byte)79, (byte)132, (byte)176, (byte)215, (byte)238, (byte)242, (byte)43, (byte)197, (byte)28, (byte)81, (byte)203, (byte)15, (byte)55, (byte)125, (byte)224, (byte)36, (byte)173, (byte)253, (byte)41, (byte)68, (byte)142, (byte)86, (byte)248, (byte)185, (byte)162, (byte)158, (byte)101, (byte)210, (byte)92, (byte)0, (byte)156, (byte)98, (byte)241, (byte)97, (byte)52, (byte)78, (byte)132, (byte)186, (byte)249, (byte)224, (byte)228, (byte)205, (byte)5}, 0) ;
            p266.sequence = (ushort)(ushort)29949;
            LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.first_message_offset == (byte)(byte)90);
                Debug.Assert(pack.length == (byte)(byte)47);
                Debug.Assert(pack.sequence == (ushort)(ushort)6117);
                Debug.Assert(pack.target_system == (byte)(byte)66);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)134, (byte)64, (byte)160, (byte)226, (byte)13, (byte)7, (byte)81, (byte)231, (byte)14, (byte)103, (byte)115, (byte)148, (byte)218, (byte)1, (byte)168, (byte)50, (byte)124, (byte)95, (byte)165, (byte)198, (byte)165, (byte)72, (byte)58, (byte)167, (byte)222, (byte)70, (byte)26, (byte)6, (byte)125, (byte)95, (byte)177, (byte)226, (byte)225, (byte)50, (byte)147, (byte)197, (byte)227, (byte)8, (byte)121, (byte)11, (byte)65, (byte)22, (byte)152, (byte)121, (byte)73, (byte)86, (byte)99, (byte)69, (byte)139, (byte)75, (byte)110, (byte)183, (byte)90, (byte)192, (byte)135, (byte)246, (byte)179, (byte)140, (byte)183, (byte)56, (byte)163, (byte)29, (byte)250, (byte)73, (byte)113, (byte)43, (byte)203, (byte)4, (byte)76, (byte)207, (byte)60, (byte)178, (byte)214, (byte)21, (byte)226, (byte)73, (byte)240, (byte)71, (byte)33, (byte)111, (byte)85, (byte)199, (byte)148, (byte)145, (byte)59, (byte)135, (byte)47, (byte)180, (byte)145, (byte)60, (byte)18, (byte)184, (byte)242, (byte)33, (byte)93, (byte)92, (byte)116, (byte)179, (byte)246, (byte)188, (byte)134, (byte)209, (byte)157, (byte)185, (byte)176, (byte)162, (byte)32, (byte)48, (byte)219, (byte)105, (byte)212, (byte)71, (byte)205, (byte)27, (byte)98, (byte)227, (byte)5, (byte)241, (byte)136, (byte)206, (byte)7, (byte)200, (byte)255, (byte)81, (byte)24, (byte)161, (byte)105, (byte)133, (byte)16, (byte)40, (byte)150, (byte)243, (byte)119, (byte)62, (byte)127, (byte)153, (byte)232, (byte)161, (byte)235, (byte)13, (byte)195, (byte)236, (byte)133, (byte)175, (byte)79, (byte)222, (byte)129, (byte)95, (byte)255, (byte)196, (byte)211, (byte)148, (byte)251, (byte)16, (byte)128, (byte)29, (byte)173, (byte)229, (byte)78, (byte)234, (byte)113, (byte)166, (byte)243, (byte)6, (byte)54, (byte)137, (byte)228, (byte)55, (byte)165, (byte)205, (byte)179, (byte)5, (byte)70, (byte)66, (byte)153, (byte)89, (byte)137, (byte)180, (byte)227, (byte)190, (byte)53, (byte)79, (byte)147, (byte)116, (byte)22, (byte)185, (byte)227, (byte)228, (byte)184, (byte)159, (byte)9, (byte)83, (byte)95, (byte)80, (byte)79, (byte)134, (byte)125, (byte)255, (byte)209, (byte)15, (byte)139, (byte)116, (byte)198, (byte)15, (byte)102, (byte)193, (byte)236, (byte)1, (byte)96, (byte)27, (byte)139, (byte)252, (byte)218, (byte)239, (byte)33, (byte)170, (byte)241, (byte)186, (byte)109, (byte)144, (byte)161, (byte)0, (byte)249, (byte)155, (byte)214, (byte)92, (byte)89, (byte)20, (byte)32, (byte)71, (byte)32, (byte)196, (byte)149, (byte)180, (byte)242, (byte)20, (byte)169, (byte)61, (byte)26, (byte)165, (byte)246, (byte)50, (byte)44, (byte)115, (byte)167, (byte)50, (byte)225, (byte)93, (byte)167}));
            };
            DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)47;
            p267.target_component = (byte)(byte)102;
            p267.target_system = (byte)(byte)66;
            p267.first_message_offset = (byte)(byte)90;
            p267.data__SET(new byte[] {(byte)134, (byte)64, (byte)160, (byte)226, (byte)13, (byte)7, (byte)81, (byte)231, (byte)14, (byte)103, (byte)115, (byte)148, (byte)218, (byte)1, (byte)168, (byte)50, (byte)124, (byte)95, (byte)165, (byte)198, (byte)165, (byte)72, (byte)58, (byte)167, (byte)222, (byte)70, (byte)26, (byte)6, (byte)125, (byte)95, (byte)177, (byte)226, (byte)225, (byte)50, (byte)147, (byte)197, (byte)227, (byte)8, (byte)121, (byte)11, (byte)65, (byte)22, (byte)152, (byte)121, (byte)73, (byte)86, (byte)99, (byte)69, (byte)139, (byte)75, (byte)110, (byte)183, (byte)90, (byte)192, (byte)135, (byte)246, (byte)179, (byte)140, (byte)183, (byte)56, (byte)163, (byte)29, (byte)250, (byte)73, (byte)113, (byte)43, (byte)203, (byte)4, (byte)76, (byte)207, (byte)60, (byte)178, (byte)214, (byte)21, (byte)226, (byte)73, (byte)240, (byte)71, (byte)33, (byte)111, (byte)85, (byte)199, (byte)148, (byte)145, (byte)59, (byte)135, (byte)47, (byte)180, (byte)145, (byte)60, (byte)18, (byte)184, (byte)242, (byte)33, (byte)93, (byte)92, (byte)116, (byte)179, (byte)246, (byte)188, (byte)134, (byte)209, (byte)157, (byte)185, (byte)176, (byte)162, (byte)32, (byte)48, (byte)219, (byte)105, (byte)212, (byte)71, (byte)205, (byte)27, (byte)98, (byte)227, (byte)5, (byte)241, (byte)136, (byte)206, (byte)7, (byte)200, (byte)255, (byte)81, (byte)24, (byte)161, (byte)105, (byte)133, (byte)16, (byte)40, (byte)150, (byte)243, (byte)119, (byte)62, (byte)127, (byte)153, (byte)232, (byte)161, (byte)235, (byte)13, (byte)195, (byte)236, (byte)133, (byte)175, (byte)79, (byte)222, (byte)129, (byte)95, (byte)255, (byte)196, (byte)211, (byte)148, (byte)251, (byte)16, (byte)128, (byte)29, (byte)173, (byte)229, (byte)78, (byte)234, (byte)113, (byte)166, (byte)243, (byte)6, (byte)54, (byte)137, (byte)228, (byte)55, (byte)165, (byte)205, (byte)179, (byte)5, (byte)70, (byte)66, (byte)153, (byte)89, (byte)137, (byte)180, (byte)227, (byte)190, (byte)53, (byte)79, (byte)147, (byte)116, (byte)22, (byte)185, (byte)227, (byte)228, (byte)184, (byte)159, (byte)9, (byte)83, (byte)95, (byte)80, (byte)79, (byte)134, (byte)125, (byte)255, (byte)209, (byte)15, (byte)139, (byte)116, (byte)198, (byte)15, (byte)102, (byte)193, (byte)236, (byte)1, (byte)96, (byte)27, (byte)139, (byte)252, (byte)218, (byte)239, (byte)33, (byte)170, (byte)241, (byte)186, (byte)109, (byte)144, (byte)161, (byte)0, (byte)249, (byte)155, (byte)214, (byte)92, (byte)89, (byte)20, (byte)32, (byte)71, (byte)32, (byte)196, (byte)149, (byte)180, (byte)242, (byte)20, (byte)169, (byte)61, (byte)26, (byte)165, (byte)246, (byte)50, (byte)44, (byte)115, (byte)167, (byte)50, (byte)225, (byte)93, (byte)167}, 0) ;
            p267.sequence = (ushort)(ushort)6117;
            LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)55644);
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.target_system == (byte)(byte)19);
            };
            DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)19;
            p268.target_component = (byte)(byte)243;
            p268.sequence = (ushort)(ushort)55644;
            LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)8631);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)34455);
                Debug.Assert(pack.framerate == (float)3.3063758E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)63125);
                Debug.Assert(pack.camera_id == (byte)(byte)223);
                Debug.Assert(pack.uri_LEN(ph) == 194);
                Debug.Assert(pack.uri_TRY(ph).Equals("lbVruLxtlrbotosGqarfutmiptvptgzltUzfwufvrsfvshYruxfnoOagvifkyrxxmqAspsbkfrhkagasbtozsccjfeafvryyvtyrgbwxnejgfyxydqpgzpoEfzeleyxfiUrihmvpbfukocjdQtlwsdlllebmLzspebqqpdVpwisupacveulbdokitobsbKcmxz"));
                Debug.Assert(pack.bitrate == (uint)4010247400U);
                Debug.Assert(pack.status == (byte)(byte)204);
            };
            DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)63125;
            p269.framerate = (float)3.3063758E37F;
            p269.resolution_v = (ushort)(ushort)8631;
            p269.status = (byte)(byte)204;
            p269.uri_SET("lbVruLxtlrbotosGqarfutmiptvptgzltUzfwufvrsfvshYruxfnoOagvifkyrxxmqAspsbkfrhkagasbtozsccjfeafvryyvtyrgbwxnejgfyxydqpgzpoEfzeleyxfiUrihmvpbfukocjdQtlwsdlllebmLzspebqqpdVpwisupacveulbdokitobsbKcmxz", PH) ;
            p269.camera_id = (byte)(byte)223;
            p269.bitrate = (uint)4010247400U;
            p269.resolution_h = (ushort)(ushort)34455;
            LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)667436433U);
                Debug.Assert(pack.rotation == (ushort)(ushort)55258);
                Debug.Assert(pack.uri_LEN(ph) == 169);
                Debug.Assert(pack.uri_TRY(ph).Equals("jdvoMnjojlxkzmxEnjkifbrecxcxjwobdadspjgmqlkbzihKotlKxjymzzzhsdxBslRnbSbxkjajdoLlLimmlnwqgRxmclxhzrsiqakvxelwhdnexWiukiGbhlyfwmmBomwrutbitzhhiazrqcnhmwTvvEiiswdedqimyuokj"));
                Debug.Assert(pack.framerate == (float)3.7349794E37F);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)44735);
                Debug.Assert(pack.camera_id == (byte)(byte)78);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)46467);
                Debug.Assert(pack.target_component == (byte)(byte)203);
            };
            DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)44735;
            p270.camera_id = (byte)(byte)78;
            p270.target_component = (byte)(byte)203;
            p270.bitrate = (uint)667436433U;
            p270.framerate = (float)3.7349794E37F;
            p270.resolution_h = (ushort)(ushort)46467;
            p270.uri_SET("jdvoMnjojlxkzmxEnjkifbrecxcxjwobdadspjgmqlkbzihKotlKxjymzzzhsdxBslRnbSbxkjajdoLlLimmlnwqgRxmclxhzrsiqakvxelwhdnexWiukiGbhlyfwmmBomwrutbitzhhiazrqcnhmwTvvEiiswdedqimyuokj", PH) ;
            p270.rotation = (ushort)(ushort)55258;
            p270.target_system = (byte)(byte)140;
            LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 18);
                Debug.Assert(pack.ssid_TRY(ph).Equals("btuNeinphoysymrIeq"));
                Debug.Assert(pack.password_LEN(ph) == 35);
                Debug.Assert(pack.password_TRY(ph).Equals("qafxfRxuqfksbxoproswkjitywqdanCuuFb"));
            };
            DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("qafxfRxuqfksbxoproswkjitywqdanCuuFb", PH) ;
            p299.ssid_SET("btuNeinphoysymrIeq", PH) ;
            LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)21362);
                Debug.Assert(pack.min_version == (ushort)(ushort)12480);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)239, (byte)13, (byte)148, (byte)163, (byte)3, (byte)60, (byte)54, (byte)182}));
                Debug.Assert(pack.version == (ushort)(ushort)25994);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)11, (byte)253, (byte)27, (byte)50, (byte)53, (byte)63, (byte)239, (byte)105}));
            };
            DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)11, (byte)253, (byte)27, (byte)50, (byte)53, (byte)63, (byte)239, (byte)105}, 0) ;
            p300.version = (ushort)(ushort)25994;
            p300.min_version = (ushort)(ushort)12480;
            p300.max_version = (ushort)(ushort)21362;
            p300.spec_version_hash_SET(new byte[] {(byte)239, (byte)13, (byte)148, (byte)163, (byte)3, (byte)60, (byte)54, (byte)182}, 0) ;
            LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)12728);
                Debug.Assert(pack.time_usec == (ulong)4301600236041839942L);
                Debug.Assert(pack.sub_mode == (byte)(byte)16);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.uptime_sec == (uint)1615791469U);
            };
            DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.vendor_specific_status_code = (ushort)(ushort)12728;
            p310.time_usec = (ulong)4301600236041839942L;
            p310.uptime_sec = (uint)1615791469U;
            p310.sub_mode = (byte)(byte)16;
            LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 57);
                Debug.Assert(pack.name_TRY(ph).Equals("ubydbghQtqujjongaivkqctwhfauonDvyxNlFfihnltwaqcuwmtakresd"));
                Debug.Assert(pack.uptime_sec == (uint)49050933U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)100);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)68, (byte)160, (byte)142, (byte)190, (byte)130, (byte)7, (byte)172, (byte)15, (byte)234, (byte)81, (byte)105, (byte)131, (byte)131, (byte)231, (byte)237, (byte)120}));
                Debug.Assert(pack.time_usec == (ulong)7892804803177338531L);
                Debug.Assert(pack.sw_version_major == (byte)(byte)124);
                Debug.Assert(pack.hw_version_major == (byte)(byte)80);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)26);
                Debug.Assert(pack.sw_vcs_commit == (uint)2718167377U);
            };
            DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)7892804803177338531L;
            p311.hw_version_minor = (byte)(byte)100;
            p311.hw_unique_id_SET(new byte[] {(byte)68, (byte)160, (byte)142, (byte)190, (byte)130, (byte)7, (byte)172, (byte)15, (byte)234, (byte)81, (byte)105, (byte)131, (byte)131, (byte)231, (byte)237, (byte)120}, 0) ;
            p311.sw_version_minor = (byte)(byte)26;
            p311.name_SET("ubydbghQtqujjongaivkqctwhfauonDvyxNlFfihnltwaqcuwmtakresd", PH) ;
            p311.hw_version_major = (byte)(byte)80;
            p311.uptime_sec = (uint)49050933U;
            p311.sw_version_major = (byte)(byte)124;
            p311.sw_vcs_commit = (uint)2718167377U;
            LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bfbefeaha"));
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.param_index == (short)(short)31973);
            };
            DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("bfbefeaha", PH) ;
            p320.target_component = (byte)(byte)145;
            p320.target_system = (byte)(byte)82;
            p320.param_index = (short)(short)31973;
            LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.target_system == (byte)(byte)38);
            };
            DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)205;
            p321.target_system = (byte)(byte)38;
            LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)43748);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)40986);
                Debug.Assert(pack.param_value_LEN(ph) == 34);
                Debug.Assert(pack.param_value_TRY(ph).Equals("nciwzWcipftrxbxdzezKooghPjmccnnxyQ"));
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("iirasnkqux"));
            };
            DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("iirasnkqux", PH) ;
            p322.param_count = (ushort)(ushort)43748;
            p322.param_index = (ushort)(ushort)40986;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_value_SET("nciwzWcipftrxbxdzezKooghPjmccnnxyQ", PH) ;
            LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 11);
                Debug.Assert(pack.param_value_TRY(ph).Equals("lldgLazzarq"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.target_system == (byte)(byte)47);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("y"));
            };
            DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("lldgLazzarq", PH) ;
            p323.param_id_SET("y", PH) ;
            p323.target_component = (byte)(byte)28;
            p323.target_system = (byte)(byte)47;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_value_LEN(ph) == 55);
                Debug.Assert(pack.param_value_TRY(ph).Equals("iwazkhKhdsuscpfsfkuacwnTuxdojkydlvqwspbfbnAravmzrdrxgwh"));
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("BbBiobnzcTgwz"));
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED);
            };
            DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            p324.param_value_SET("iwazkhKhdsuscpfsfkuacwnTuxdojkydlvqwspbfbnAravmzrdrxgwh", PH) ;
            p324.param_id_SET("BbBiobnzcTgwz", PH) ;
            LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.time_usec == (ulong)70007637702031832L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)30227);
                Debug.Assert(pack.min_distance == (ushort)(ushort)40546);
                Debug.Assert(pack.increment == (byte)(byte)52);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)8744, (ushort)57524, (ushort)3500, (ushort)6623, (ushort)19956, (ushort)49869, (ushort)1446, (ushort)16599, (ushort)49856, (ushort)58473, (ushort)49819, (ushort)41988, (ushort)42279, (ushort)1143, (ushort)63498, (ushort)51225, (ushort)22698, (ushort)62814, (ushort)65158, (ushort)53355, (ushort)61448, (ushort)16261, (ushort)59767, (ushort)53164, (ushort)43519, (ushort)55766, (ushort)27668, (ushort)49210, (ushort)591, (ushort)45166, (ushort)61976, (ushort)28147, (ushort)43263, (ushort)40181, (ushort)29136, (ushort)50486, (ushort)22930, (ushort)59457, (ushort)4948, (ushort)3353, (ushort)32015, (ushort)35485, (ushort)12220, (ushort)15363, (ushort)28850, (ushort)46592, (ushort)24954, (ushort)39573, (ushort)1538, (ushort)11421, (ushort)42931, (ushort)49836, (ushort)32558, (ushort)52607, (ushort)58359, (ushort)1240, (ushort)21787, (ushort)52133, (ushort)34369, (ushort)48751, (ushort)30251, (ushort)28146, (ushort)59048, (ushort)60411, (ushort)36501, (ushort)58696, (ushort)20840, (ushort)47694, (ushort)27282, (ushort)25779, (ushort)27649, (ushort)58025}));
            };
            DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)40546;
            p330.max_distance = (ushort)(ushort)30227;
            p330.time_usec = (ulong)70007637702031832L;
            p330.increment = (byte)(byte)52;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.distances_SET(new ushort[] {(ushort)8744, (ushort)57524, (ushort)3500, (ushort)6623, (ushort)19956, (ushort)49869, (ushort)1446, (ushort)16599, (ushort)49856, (ushort)58473, (ushort)49819, (ushort)41988, (ushort)42279, (ushort)1143, (ushort)63498, (ushort)51225, (ushort)22698, (ushort)62814, (ushort)65158, (ushort)53355, (ushort)61448, (ushort)16261, (ushort)59767, (ushort)53164, (ushort)43519, (ushort)55766, (ushort)27668, (ushort)49210, (ushort)591, (ushort)45166, (ushort)61976, (ushort)28147, (ushort)43263, (ushort)40181, (ushort)29136, (ushort)50486, (ushort)22930, (ushort)59457, (ushort)4948, (ushort)3353, (ushort)32015, (ushort)35485, (ushort)12220, (ushort)15363, (ushort)28850, (ushort)46592, (ushort)24954, (ushort)39573, (ushort)1538, (ushort)11421, (ushort)42931, (ushort)49836, (ushort)32558, (ushort)52607, (ushort)58359, (ushort)1240, (ushort)21787, (ushort)52133, (ushort)34369, (ushort)48751, (ushort)30251, (ushort)28146, (ushort)59048, (ushort)60411, (ushort)36501, (ushort)58696, (ushort)20840, (ushort)47694, (ushort)27282, (ushort)25779, (ushort)27649, (ushort)58025}, 0) ;
            LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aircraftSize == (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M);
                Debug.Assert(pack.stallSpeed == (ushort)(ushort)14369);
                Debug.Assert(pack.gpsOffsetLat == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M);
                Debug.Assert(pack.emitterType == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.gpsOffsetLon == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
                Debug.Assert(pack.callsign_LEN(ph) == 6);
                Debug.Assert(pack.callsign_TRY(ph).Equals("cPNAoh"));
                Debug.Assert(pack.ICAO == (uint)129518704U);
                Debug.Assert(pack.rfSelect == (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
            };
            DemoDevice.UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.aircraftSize = (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M;
            p10001.callsign_SET("cPNAoh", PH) ;
            p10001.ICAO = (uint)129518704U;
            p10001.stallSpeed = (ushort)(ushort)14369;
            p10001.gpsOffsetLat = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M;
            p10001.rfSelect = (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED;
            p10001.gpsOffsetLon = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR;
            p10001.emitterType = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            LoopBackDemoChannel.instance.send(p10001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.VelEW == (short)(short) -13206);
                Debug.Assert(pack.accuracyVert == (ushort)(ushort)50966);
                Debug.Assert(pack.utcTime == (uint)4273240082U);
                Debug.Assert(pack.numSats == (byte)(byte)91);
                Debug.Assert(pack.velVert == (short)(short) -20150);
                Debug.Assert(pack.accuracyVel == (ushort)(ushort)21388);
                Debug.Assert(pack.emergencyStatus == (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY);
                Debug.Assert(pack.gpsAlt == (int)676217199);
                Debug.Assert(pack.accuracyHor == (uint)2331336026U);
                Debug.Assert(pack.gpsLon == (int) -1202535213);
                Debug.Assert(pack.squawk == (ushort)(ushort)25613);
                Debug.Assert(pack.velNS == (short)(short)28929);
                Debug.Assert(pack.state == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE);
                Debug.Assert(pack.gpsFix == (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS);
                Debug.Assert(pack.gpsLat == (int)1430623844);
                Debug.Assert(pack.baroAltMSL == (int)1626979591);
            };
            DemoDevice.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.emergencyStatus = (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY;
            p10002.baroAltMSL = (int)1626979591;
            p10002.velVert = (short)(short) -20150;
            p10002.gpsAlt = (int)676217199;
            p10002.utcTime = (uint)4273240082U;
            p10002.squawk = (ushort)(ushort)25613;
            p10002.numSats = (byte)(byte)91;
            p10002.velNS = (short)(short)28929;
            p10002.accuracyHor = (uint)2331336026U;
            p10002.gpsLat = (int)1430623844;
            p10002.accuracyVel = (ushort)(ushort)21388;
            p10002.VelEW = (short)(short) -13206;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE;
            p10002.gpsFix = (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS;
            p10002.accuracyVert = (ushort)(ushort)50966;
            p10002.gpsLon = (int) -1202535213;
            LoopBackDemoChannel.instance.send(p10002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rfHealth == (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX);
            };
            DemoDevice.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX;
            LoopBackDemoChannel.instance.send(p10003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}

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
            LoopBackDemoChannel.instance.OnTEST_TYPESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.u16 == (ushort)(ushort)39326);
                Debug.Assert(pack.u16_array.SequenceEqual(new ushort[] {(ushort)11332, (ushort)13366, (ushort)43813}));
                Debug.Assert(pack.c == (ushort)(ushort)14847);
                Debug.Assert(pack.d == (double) -5.373225915539415E307);
                Debug.Assert(pack.s8_array.SequenceEqual(new sbyte[] {(sbyte)62, (sbyte)110, (sbyte) - 86}));
                Debug.Assert(pack.s64 == (long)7125046053842312623L);
                Debug.Assert(pack.u32 == (uint)4089306715U);
                Debug.Assert(pack.s8 == (sbyte)(sbyte) - 89);
                Debug.Assert(pack.f == (float)4.5712185E37F);
                Debug.Assert(pack.u64_array.SequenceEqual(new ulong[] {7194004233661366529L, 5168933645823866254L, 1999201908225411589L}));
                Debug.Assert(pack.s64_array.SequenceEqual(new long[] {-497622854507314996L, 8388676822281522768L, 5351170554754052216L}));
                Debug.Assert(pack.s_LEN(ph) == 4);
                Debug.Assert(pack.s_TRY(ph).Equals("wjxw"));
                Debug.Assert(pack.u64 == (ulong)7928702084717961463L);
                Debug.Assert(pack.u8_array.SequenceEqual(new byte[] {(byte)121, (byte)174, (byte)39}));
                Debug.Assert(pack.s16_array.SequenceEqual(new short[] {(short) -20768, (short)31200, (short) -30618}));
                Debug.Assert(pack.u8 == (byte)(byte)84);
                Debug.Assert(pack.s16 == (short)(short) -7335);
                Debug.Assert(pack.s32 == (int)1764977703);
                Debug.Assert(pack.f_array.SequenceEqual(new float[] {3.40737E37F, 8.180184E37F, -8.270778E37F}));
                Debug.Assert(pack.u32_array.SequenceEqual(new uint[] {333102514U, 3803702871U, 3904066361U}));
                Debug.Assert(pack.d_array.SequenceEqual(new double[] {-1.1284547727947998E308, -3.79597733516474E307, 1.6051686058282311E308}));
                Debug.Assert(pack.s32_array.SequenceEqual(new int[] {-1008266253, 1685962533, 294459279}));
            };
            DemoDevice.TEST_TYPES p0 = LoopBackDemoChannel.new_TEST_TYPES();
            PH.setPack(p0);
            p0.u8_array_SET(new byte[] {(byte)121, (byte)174, (byte)39}, 0) ;
            p0.s64 = (long)7125046053842312623L;
            p0.d_array_SET(new double[] {-1.1284547727947998E308, -3.79597733516474E307, 1.6051686058282311E308}, 0) ;
            p0.f_array_SET(new float[] {3.40737E37F, 8.180184E37F, -8.270778E37F}, 0) ;
            p0.s_SET("wjxw", PH) ;
            p0.u32_array_SET(new uint[] {333102514U, 3803702871U, 3904066361U}, 0) ;
            p0.s32_array_SET(new int[] {-1008266253, 1685962533, 294459279}, 0) ;
            p0.s16 = (short)(short) -7335;
            p0.u16_array_SET(new ushort[] {(ushort)11332, (ushort)13366, (ushort)43813}, 0) ;
            p0.s8_array_SET(new sbyte[] {(sbyte)62, (sbyte)110, (sbyte) - 86}, 0) ;
            p0.f = (float)4.5712185E37F;
            p0.u32 = (uint)4089306715U;
            p0.s8 = (sbyte)(sbyte) - 89;
            p0.u16 = (ushort)(ushort)39326;
            p0.s32 = (int)1764977703;
            p0.c = (ushort)(ushort)14847;
            p0.s64_array_SET(new long[] {-497622854507314996L, 8388676822281522768L, 5351170554754052216L}, 0) ;
            p0.d = (double) -5.373225915539415E307;
            p0.u64_array_SET(new ulong[] {7194004233661366529L, 5168933645823866254L, 1999201908225411589L}, 0) ;
            p0.u64 = (ulong)7928702084717961463L;
            p0.u8 = (byte)(byte)84;
            p0.s16_array_SET(new short[] {(short) -20768, (short)31200, (short) -30618}, 0) ;
            LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(LoopBackDemoChannel.instance, LoopBackDemoChannel.instance);
        }
    }
}
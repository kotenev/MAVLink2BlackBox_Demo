
using System.Diagnostics;
using System.Threading;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Demo :  DemoDevice
    {
        static void Main_(string[] args)
        {
            Inside PH = new Inside();
            LoopBackDemoChannel.instance.OnTEST_TYPESReceive += (src, ph, pack) =>
            {
                ushort c = pack.c;
                string s = pack.s_TRY(ph);
                byte u8 = pack.u8;
                ushort u16 = pack.u16;
                uint u32 = pack.u32;
                ulong u64 = pack.u64;
                sbyte s8 = pack.s8;
                short s16 = pack.s16;
                int s32 = pack.s32;
                long s64 = pack.s64;
                float f = pack.f;
                double d = pack.d;
                byte[] u8_array = pack.u8_array;
                ushort[] u16_array = pack.u16_array;
                uint[] u32_array = pack.u32_array;
                ulong[] u64_array = pack.u64_array;
                sbyte[] s8_array = pack.s8_array;
                short[] s16_array = pack.s16_array;
                int[] s32_array = pack.s32_array;
                long[] s64_array = pack.s64_array;
                float[] f_array = pack.f_array;
                double[] d_array = pack.d_array;
            };
            TEST_TYPES p0 = LoopBackDemoChannel.new_TEST_TYPES();
            PH.setPack(p0);
            p0.c = (ushort)(ushort)57011;
            p0.s_SET("DEMO", PH);
            p0.u8 = (byte)(byte)101;
            p0.u16 = (ushort)(ushort)3102;
            p0.u32 = (uint)1661527502U;
            p0.u64 = (ulong)4149424448508277993L;
            p0.s8 = (sbyte)(sbyte) - 73;
            p0.s16 = (short)(short) -3142;
            p0.s32 = (int)1379948791;
            p0.s64 = (long)6692053499457886583L;
            p0.f = (float)2.174487E38F;
            p0.d = (double) -6.313699127006534E307;
            p0.u8_array_SET(new byte[3], 0);
            p0.u16_array_SET(new ushort[3], 0);
            p0.u32_array_SET(new uint[3], 0);
            p0.u64_array_SET(new ulong[3], 0);
            p0.s8_array_SET(new sbyte[3], 0);
            p0.s16_array_SET(new short[3], 0);
            p0.s32_array_SET(new int[3], 0);
            p0.s64_array_SET(new long[3], 0);
            p0.f_array_SET(new float[3], 0);
            p0.d_array_SET(new double[3], 0);
            LoopBackDemoChannel.instance.send(p0); //===============================
        }
    }
}

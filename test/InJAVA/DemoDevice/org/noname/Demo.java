package org.noname;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends  DemoDevice
{
    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        LoopBackDemoChannel.instance.on_TEST_TYPES.add((src, ph, pack) ->
        {
            char  c = pack.c_GET();
            String s = pack.s_TRY(ph);
            char  u8 = pack.u8_GET();
            char  u16 = pack.u16_GET();
            long  u32 = pack.u32_GET();
            long  u64 = pack.u64_GET();
            byte  s8 = pack.s8_GET();
            short  s16 = pack.s16_GET();
            int  s32 = pack.s32_GET();
            long  s64 = pack.s64_GET();
            float  f = pack.f_GET();
            double  d = pack.d_GET();
            char[]  u8_array = pack.u8_array_GET();
            char[]  u16_array = pack.u16_array_GET();
            long[]  u32_array = pack.u32_array_GET();
            long[]  u64_array = pack.u64_array_GET();
            byte[]  s8_array = pack.s8_array_GET();
            short[]  s16_array = pack.s16_array_GET();
            int[]  s32_array = pack.s32_array_GET();
            long[]  s64_array = pack.s64_array_GET();
            float[]  f_array = pack.f_array_GET();
            double[]  d_array = pack.d_array_GET();
        });
        TEST_TYPES p0 = LoopBackDemoChannel.instance.new_TEST_TYPES();
        PH.setPack(p0);
        p0.c_SET((char)14400);
        p0.s_SET("DEMO", PH);
        p0.u8_SET((char)196);
        p0.u16_SET((char)22558);
        p0.u32_SET(2583560675L);
        p0.u64_SET(2494553126248127495L);
        p0.s8_SET((byte)68);
        p0.s16_SET((short)32433);
        p0.s32_SET(-601165190);
        p0.s64_SET(-7635678727238898929L);
        p0.f_SET(-2.606212E38F);
        p0.d_SET(-1.004687109183497E308);
        p0.u8_array_SET(new char[3], 0);
        p0.u16_array_SET(new char[3], 0);
        p0.u32_array_SET(new long[3], 0);
        p0.u64_array_SET(new long[3], 0);
        p0.s8_array_SET(new byte[3], 0);
        p0.s16_array_SET(new short[3], 0);
        p0.s32_array_SET(new int[3], 0);
        p0.s64_array_SET(new long[3], 0);
        p0.f_array_SET(new float[3], 0);
        p0.d_array_SET(new double[3], 0);
        LoopBackDemoChannel.instance.send(p0); //===============================
    }
}

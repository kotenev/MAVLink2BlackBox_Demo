
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
        LoopBackDemoChannel.instance.on_TEST_TYPES.add((src, ph, pack) ->
        {
            assert(pack.s_LEN(ph) == 2);
            assert(pack.s_TRY(ph).equals("gt"));
            assert(Arrays.equals(pack.u64_array_GET(),  new long[] {4058240405631965179L, 7045184625845394498L, 7205245876484545422L}));
            assert(Arrays.equals(pack.u32_array_GET(),  new long[] {183470960L, 4225895476L, 511072338L}));
            assert(pack.u8_GET() == (char)128);
            assert(Arrays.equals(pack.f_array_GET(),  new float[] {-2.4910052E38F, 2.7877373E38F, -3.0423927E38F}));
            assert(Arrays.equals(pack.u8_array_GET(),  new char[] {(char)29, (char)165, (char)174}));
            assert(pack.u16_GET() == (char)23035);
            assert(pack.s16_GET() == (short) -31813);
            assert(Arrays.equals(pack.d_array_GET(),  new double[] {-4.0901308918897313E307, 1.1660809851002205E308, -8.07524758660438E307}));
            assert(pack.u32_GET() == 2611680089L);
            assert(pack.u64_GET() == 3258420538774021285L);
            assert(Arrays.equals(pack.u16_array_GET(),  new char[] {(char)32646, (char)31649, (char)51175}));
            assert(pack.s32_GET() == -806352833);
            assert(pack.c_GET() == (char)1580);
            assert(pack.d_GET() == -4.4504479588149845E307);
            assert(pack.s64_GET() == -228208318208944756L);
            assert(pack.s8_GET() == (byte) - 35);
            assert(Arrays.equals(pack.s16_array_GET(),  new short[] {(short) -10700, (short) -7908, (short)31005}));
            assert(Arrays.equals(pack.s32_array_GET(),  new int[] {1165466276, 476598825, 1542285418}));
            assert(Arrays.equals(pack.s64_array_GET(),  new long[] {1520252003708991056L, -3083723477264845548L, 7486510372433018513L}));
            assert(Arrays.equals(pack.s8_array_GET(),  new byte[] {(byte)112, (byte)81, (byte)13}));
            assert(pack.f_GET() == -9.776356E37F);
        });
        DemoDevice.TEST_TYPES p0 = LoopBackDemoChannel.new_TEST_TYPES();
        PH.setPack(p0);
        p0.u64_SET(3258420538774021285L) ;
        p0.s8_SET((byte) - 35) ;
        p0.u16_array_SET(new char[] {(char)32646, (char)31649, (char)51175}, 0) ;
        p0.s_SET("gt", PH) ;
        p0.d_array_SET(new double[] {-4.0901308918897313E307, 1.1660809851002205E308, -8.07524758660438E307}, 0) ;
        p0.u32_SET(2611680089L) ;
        p0.f_array_SET(new float[] {-2.4910052E38F, 2.7877373E38F, -3.0423927E38F}, 0) ;
        p0.d_SET(-4.4504479588149845E307) ;
        p0.s64_array_SET(new long[] {1520252003708991056L, -3083723477264845548L, 7486510372433018513L}, 0) ;
        p0.s32_SET(-806352833) ;
        p0.s64_SET(-228208318208944756L) ;
        p0.u64_array_SET(new long[] {4058240405631965179L, 7045184625845394498L, 7205245876484545422L}, 0) ;
        p0.u8_array_SET(new char[] {(char)29, (char)165, (char)174}, 0) ;
        p0.s32_array_SET(new int[] {1165466276, 476598825, 1542285418}, 0) ;
        p0.f_SET(-9.776356E37F) ;
        p0.u8_SET((char)128) ;
        p0.s8_array_SET(new byte[] {(byte)112, (byte)81, (byte)13}, 0) ;
        p0.u32_array_SET(new long[] {183470960L, 4225895476L, 511072338L}, 0) ;
        p0.s16_array_SET(new short[] {(short) -10700, (short) -7908, (short)31005}, 0) ;
        p0.s16_SET((short) -31813) ;
        p0.c_SET((char)1580) ;
        p0.u16_SET((char)23035) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}
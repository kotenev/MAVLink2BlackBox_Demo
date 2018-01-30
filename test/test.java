import org.unirail.BlackBox.*;
public class test {}
class LoopBackDemoChannel extends SimpleProtocol implements DemoDevice.MainInterface, DemoDevice.LoopBack {}
class LoopBackDemoChannel_ADV extends AdvancedProtocol implements DemoDevice.MainInterface, DemoDevice.LoopBack {}
class DemoDevice implements InC, InJAVA, InCS
{
    interface LoopBack extends MainInterface {} interface MainInterface
    {

        /*
        char*/
        @id(0) class TEST_TYPES
        {
            char  c;//char
            @A(10) String  s;//string
            @A byte  u8;//uint8_t
            @A short  u16;//uint16_t
            @A int  u32;//uint32_t
            @A long  u64;//uint64_t
            byte  s8;//int8_t
            short  s16;//int16_t
            int  s32;//int32_t
            long  s64;//int64_t
            float  f;//float
            double  d;//double
            @D(3)@A byte []  u8_array; //uint8_t_array
            @D(3)@A short []  u16_array; //uint16_t_array
            @D(3)@A int []  u32_array; //uint32_t_array
            @D(3)@A long []  u64_array; //uint64_t_array
            @D(3)byte []  s8_array; //int8_t_array
            @D(3)short []  s16_array; //int16_t_array
            @D(3)int []  s32_array; //int32_t_array
            @D(3)long []  s64_array; //int64_t_array
            @D(3)float []  f_array; //float_array
            @D(3)double []  d_array; //double_array
        }
    }
}

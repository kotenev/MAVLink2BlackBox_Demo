
#include "DemoDevice.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_LoopBackDemoChannel_on_TEST_TYPES_0(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  c = p0_c_GET(pack);
    char16_t *  s = p0_s_TRY_(ph);
    uint8_t  u8 = p0_u8_GET(pack);
    uint16_t  u16 = p0_u16_GET(pack);
    uint32_t  u32 = p0_u32_GET(pack);
    uint64_t  u64 = p0_u64_GET(pack);
    int8_t  s8 = p0_s8_GET(pack);
    int16_t  s16 = p0_s16_GET(pack);
    int32_t  s32 = p0_s32_GET(pack);
    int64_t  s64 = p0_s64_GET(pack);
    float  f = p0_f_GET(pack);
    double  d = p0_d_GET(pack);
    uint8_t*  u8_array = p0_u8_array_GET_(pack);
//process data in u8_array
    free(u8_array);//never forget to dispose
    uint16_t*  u16_array = p0_u16_array_GET_(pack);
//process data in u16_array
    free(u16_array);//never forget to dispose
    uint32_t*  u32_array = p0_u32_array_GET_(pack);
//process data in u32_array
    free(u32_array);//never forget to dispose
    uint64_t*  u64_array = p0_u64_array_GET_(pack);
//process data in u64_array
    free(u64_array);//never forget to dispose
    int8_t*  s8_array = p0_s8_array_GET_(pack);
//process data in s8_array
    free(s8_array);//never forget to dispose
    int16_t*  s16_array = p0_s16_array_GET_(pack);
//process data in s16_array
    free(s16_array);//never forget to dispose
    int32_t*  s32_array = p0_s32_array_GET_(pack);
//process data in s32_array
    free(s32_array);//never forget to dispose
    int64_t*  s64_array = p0_s64_array_GET_(pack);
//process data in s64_array
    free(s64_array);//never forget to dispose
    float*  f_array = p0_f_array_GET_(pack);
//process data in f_array
    free(f_array);//never forget to dispose
    double*  d_array = p0_d_array_GET_(pack);
//process data in d_array
    free(d_array);//never forget to dispose
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_LoopBackDemoChannel_new_TEST_TYPES_0(), &PH);
    p0_c_SET((uint16_t)(uint16_t)52428, PH.base.pack) ;
    {
        char16_t   s = "QMvsEfb";
        p0_s_SET(&s, 0,  sizeof(s), &PH) ;
    }
    p0_u8_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p0_u16_SET((uint16_t)(uint16_t)8327, PH.base.pack) ;
    p0_u32_SET((uint32_t)872312196L, PH.base.pack) ;
    p0_u64_SET((uint64_t)3982276214579140810L, PH.base.pack) ;
    p0_s8_SET((int8_t)(int8_t) -46, PH.base.pack) ;
    p0_s16_SET((int16_t)(int16_t) -29591, PH.base.pack) ;
    p0_s32_SET((int32_t)1881361059, PH.base.pack) ;
    p0_s64_SET((int64_t)2638931250083279036L, PH.base.pack) ;
    p0_f_SET((float)1.8226096E38F, PH.base.pack) ;
    p0_d_SET((double) -1.4647198710378237E308, PH.base.pack) ;
    {
        uint8_t  u8_array [] =  {(uint8_t)151, (uint8_t)46, (uint8_t)67};
        p0_u8_array_SET(&u8_array, 0, &PH.base.pack) ;
    }
    {
        uint16_t  u16_array [] =  {(uint16_t)41229, (uint16_t)33897, (uint16_t)29899};
        p0_u16_array_SET(&u16_array, 0, &PH.base.pack) ;
    }
    {
        uint32_t  u32_array [] =  {4157846222L, 3112649990L, 4090920131L};
        p0_u32_array_SET(&u32_array, 0, &PH.base.pack) ;
    }
    {
        uint64_t  u64_array [] =  {5529483577926240918L, 8817244388679788843L, 6167346620309662956L};
        p0_u64_array_SET(&u64_array, 0, &PH.base.pack) ;
    }
    {
        int8_t  s8_array [] =  {(int8_t) -81, (int8_t) -85, (int8_t)48};
        p0_s8_array_SET(&s8_array, 0, &PH.base.pack) ;
    }
    {
        int16_t  s16_array [] =  {(int16_t) -19694, (int16_t) -6657, (int16_t)30332};
        p0_s16_array_SET(&s16_array, 0, &PH.base.pack) ;
    }
    {
        int32_t  s32_array [] =  {1225100179, -1003424895, 1220005846};
        p0_s32_array_SET(&s32_array, 0, &PH.base.pack) ;
    }
    {
        int64_t  s64_array [] =  {2902295699110971744L, 8391243289001645369L, 5237387651803740823L};
        p0_s64_array_SET(&s64_array, 0, &PH.base.pack) ;
    }
    {
        float  f_array [] =  {1.165196E38F, 2.1456931E38F, 1.4724886E38F};
        p0_f_array_SET(&f_array, 0, &PH.base.pack) ;
    }
    {
        double  d_array [] =  {1.7677038173930673E308, -2.311272090288132E307, 6.699977115802134E307};
        p0_d_array_SET(&d_array, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

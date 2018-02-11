
#pragma once
#include "MicroAirVehicle.h"
const int pack_id_bytes = 2;
static const Field _t = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _T = {5, true, -6, 2, 1, 0, 0, 0, 1, 1 };
static const Field _E = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _d = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _l = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Sr = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _wr = {0, true, 1, 4, 1, 0, 0, 0, 0   };
static const Field _Ir = {0, true, 1, 4, 1, 0, 0, 0, 0   };
static const Field _xr = {0, true, 1, 4, 1, 0, 0, 0, 0   };
static const Field _Or = {0, true, 1, 4, 1, 0, 0, 0, 0   };
static const Field _os = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Ss = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _ws = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Is = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _xs = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Os = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Zs = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Ds = {0, true, 1, 2, 1, 0, 0, 0, 0   };
static const Field _Lm = {0, true, 1, 8, 1, 0, 0, 0, 0   };
static const Field _Am = {0, true, 1, 8, 1, 0, 0, 0, 0   };
static const Field _Fm = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _fh = {0, false, 1, 1, 1, 0, 0, 0, 0   };
static const Field _Fh = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _uh = {0, false, 1, 1, 1, 0, 0, 0, 0   };
static const Field _ch = {0, false, 1, 1, 1, 0, 0, 0, 0   };
static const Field _OS = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _ZS = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _qL = {0, false, 18, 1, 1, 0, 0, 0, 0   };
static const Field _OL = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _ZL = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _DL = {0, false, 1, 4, 1, 0, 0, 0, 0   };
static const Field _LL = {0, false, 4, 4, 1, 0, 0, 0, 0   };
static const Field _JL = {0, false, 1, 1, 1, 0, 0, 0, 0   };
static const Field _lH = {0, true, 1, 8, 1, 0, 0, 0, 0   };
static const Field _oJ = {0, true, 1, 8, 1, 0, 0, 0, 0   };
static const Field _vJ = {5, true, -4, 2, 1, 0, 0, 0, 1, 1 };
static const Field _XJ = {5, true, -4, 2, 1, 0, 0, 0, 1, 1 };
static const Field _lJ = {5, true, -4, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Wn = {5, true, -4, 2, 1, 0, 0, 0, 1, 1 };
static const Field _sn = {5, true, -6, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Dn = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Cn = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _qA = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _kA = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _RA = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _GA = {5, true, -6, 2, 1, 0, 0, 0, 1, 1 };
static const Field _MA = {5, true, -7, 2, 1, 0, 0, 0, 1, 1 };
static const Field _ov = {5, true, -7, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Hv = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _vv = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _fv = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Cv = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _jv = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Tv = {5, true, -5, 2, 1, 0, 0, 0, 1, 1 };
static const Field _Nv = {5, true, -8, 2, 1, 0, 0, 0, 1, 1 };

static const Meta meta0 = {0, 0, 1, 0, 0, 62,  8, 0, NULL};


Pack * c_CommunicationChannel_new_HEARTBEAT_0()
{
    Pack * pack = calloc(sizeof(Pack) + meta0.packMinBytes, 1);
    pack->meta = (Meta *)&meta0;
    return pack;
};
static const Meta meta1 = {1, 8, 0, 0, 0, 230,  29, 0, NULL};


Pack * c_CommunicationChannel_new_SYS_STATUS_1()
{
    Pack * pack = calloc(sizeof(Pack) + meta1.packMinBytes, 1);
    pack->meta = (Meta *)&meta1;
    return pack;
};
static const Meta meta2 = {2, 0, 1, 1, 0, 96,  12, 0, NULL};


Pack * c_CommunicationChannel_new_SYSTEM_TIME_2()
{
    Pack * pack = calloc(sizeof(Pack) + meta2.packMinBytes, 1);
    pack->meta = (Meta *)&meta2;
    return pack;
};
static const Meta meta3 = {3, 1, 1, 0, 0, 404,  51, 0, NULL};


Pack * c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3()
{
    Pack * pack = calloc(sizeof(Pack) + meta3.packMinBytes, 1);
    pack->meta = (Meta *)&meta3;
    return pack;
};
static const Meta meta4 = {4, 0, 1, 1, 0, 112,  14, 0, NULL};


Pack * c_CommunicationChannel_new_PING_4()
{
    Pack * pack = calloc(sizeof(Pack) + meta4.packMinBytes, 1);
    pack->meta = (Meta *)&meta4;
    return pack;
};
static const Field * meta5fields[] = {&_t};
static const Meta meta5 = {5, 0, 0, 0, 0, 24,  4, 1, (Field**)&meta5fields};


Pack * c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5()
{
    Pack * pack = calloc(sizeof(Pack) + meta5.packMinBytes, 1);
    pack->meta = (Meta *)&meta5;
    return pack;
};
static const Meta meta6 = {6, 0, 0, 0, 0, 24,  3, 0, NULL};


Pack * c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6()
{
    Pack * pack = calloc(sizeof(Pack) + meta6.packMinBytes, 1);
    pack->meta = (Meta *)&meta6;
    return pack;
};
static const Field * meta7fields[] = {&_T};
static const Meta meta7 = {7, 0, 0, 0, 0, 0,  1, 1, (Field**)&meta7fields};


Pack * c_CommunicationChannel_new_AUTH_KEY_7()
{
    Pack * pack = calloc(sizeof(Pack) + meta7.packMinBytes, 1);
    pack->meta = (Meta *)&meta7;
    return pack;
};
static const Meta meta11 = {11, 0, 1, 0, 0, 44,  6, 0, NULL};


Pack * c_CommunicationChannel_new_SET_MODE_11()
{
    Pack * pack = calloc(sizeof(Pack) + meta11.packMinBytes, 1);
    pack->meta = (Meta *)&meta11;
    return pack;
};
static const Field * meta20fields[] = {&_E};
static const Meta meta20 = {20, 0, 0, 0, 0, 32,  5, 1, (Field**)&meta20fields};


Pack * c_CommunicationChannel_new_PARAM_REQUEST_READ_20()
{
    Pack * pack = calloc(sizeof(Pack) + meta20.packMinBytes, 1);
    pack->meta = (Meta *)&meta20;
    return pack;
};
static const Meta meta21 = {21, 0, 0, 0, 0, 16,  2, 0, NULL};


Pack * c_CommunicationChannel_new_PARAM_REQUEST_LIST_21()
{
    Pack * pack = calloc(sizeof(Pack) + meta21.packMinBytes, 1);
    pack->meta = (Meta *)&meta21;
    return pack;
};
static const Field * meta22fields[] = {&_d};
static const Meta meta22 = {22, 2, 0, 0, 0, 68,  10, 1, (Field**)&meta22fields};


Pack * c_CommunicationChannel_new_PARAM_VALUE_22()
{
    Pack * pack = calloc(sizeof(Pack) + meta22.packMinBytes, 1);
    pack->meta = (Meta *)&meta22;
    return pack;
};
static const Field * meta23fields[] = {&_l};
static const Meta meta23 = {23, 0, 0, 0, 0, 52,  8, 1, (Field**)&meta23fields};


Pack * c_CommunicationChannel_new_PARAM_SET_23()
{
    Pack * pack = calloc(sizeof(Pack) + meta23.packMinBytes, 1);
    pack->meta = (Meta *)&meta23;
    return pack;
};
static const Field * meta24fields[] = {&_Sr, &_wr, &_Ir, &_xr, &_Or};
static const Meta meta24 = {24, 4, 0, 1, 0, 236,  31, 5, (Field**)&meta24fields};


Pack * c_CommunicationChannel_new_GPS_RAW_INT_24()
{
    Pack * pack = calloc(sizeof(Pack) + meta24.packMinBytes, 1);
    pack->meta = (Meta *)&meta24;
    return pack;
};
static const Meta meta25 = {25, 0, 0, 0, 0, 808,  101, 0, NULL};


Pack * c_CommunicationChannel_new_GPS_STATUS_25()
{
    Pack * pack = calloc(sizeof(Pack) + meta25.packMinBytes, 1);
    pack->meta = (Meta *)&meta25;
    return pack;
};
static const Meta meta26 = {26, 0, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_SCALED_IMU_26()
{
    Pack * pack = calloc(sizeof(Pack) + meta26.packMinBytes, 1);
    pack->meta = (Meta *)&meta26;
    return pack;
};
static const Meta meta27 = {27, 0, 0, 1, 0, 208,  26, 0, NULL};


Pack * c_CommunicationChannel_new_RAW_IMU_27()
{
    Pack * pack = calloc(sizeof(Pack) + meta27.packMinBytes, 1);
    pack->meta = (Meta *)&meta27;
    return pack;
};
static const Meta meta28 = {28, 0, 0, 1, 0, 128,  16, 0, NULL};


Pack * c_CommunicationChannel_new_RAW_PRESSURE_28()
{
    Pack * pack = calloc(sizeof(Pack) + meta28.packMinBytes, 1);
    pack->meta = (Meta *)&meta28;
    return pack;
};
static const Meta meta29 = {29, 0, 1, 0, 0, 112,  14, 0, NULL};


Pack * c_CommunicationChannel_new_SCALED_PRESSURE_29()
{
    Pack * pack = calloc(sizeof(Pack) + meta29.packMinBytes, 1);
    pack->meta = (Meta *)&meta29;
    return pack;
};
static const Meta meta30 = {30, 0, 1, 0, 0, 224,  28, 0, NULL};


Pack * c_CommunicationChannel_new_ATTITUDE_30()
{
    Pack * pack = calloc(sizeof(Pack) + meta30.packMinBytes, 1);
    pack->meta = (Meta *)&meta30;
    return pack;
};
static const Meta meta31 = {31, 0, 1, 0, 0, 256,  32, 0, NULL};


Pack * c_CommunicationChannel_new_ATTITUDE_QUATERNION_31()
{
    Pack * pack = calloc(sizeof(Pack) + meta31.packMinBytes, 1);
    pack->meta = (Meta *)&meta31;
    return pack;
};
static const Meta meta32 = {32, 0, 1, 0, 0, 224,  28, 0, NULL};


Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_32()
{
    Pack * pack = calloc(sizeof(Pack) + meta32.packMinBytes, 1);
    pack->meta = (Meta *)&meta32;
    return pack;
};
static const Meta meta33 = {33, 1, 1, 0, 0, 224,  28, 0, NULL};


Pack * c_CommunicationChannel_new_GLOBAL_POSITION_INT_33()
{
    Pack * pack = calloc(sizeof(Pack) + meta33.packMinBytes, 1);
    pack->meta = (Meta *)&meta33;
    return pack;
};
static const Meta meta34 = {34, 0, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_RC_CHANNELS_SCALED_34()
{
    Pack * pack = calloc(sizeof(Pack) + meta34.packMinBytes, 1);
    pack->meta = (Meta *)&meta34;
    return pack;
};
static const Meta meta35 = {35, 8, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_RC_CHANNELS_RAW_35()
{
    Pack * pack = calloc(sizeof(Pack) + meta35.packMinBytes, 1);
    pack->meta = (Meta *)&meta35;
    return pack;
};
static const Field * meta36fields[] = {&_os, &_Ss, &_ws, &_Is, &_xs, &_Os, &_Zs, &_Ds};
static const Meta meta36 = {36, 8, 1, 0, 0, 168,  22, 8, (Field**)&meta36fields};


Pack * c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36()
{
    Pack * pack = calloc(sizeof(Pack) + meta36.packMinBytes, 1);
    pack->meta = (Meta *)&meta36;
    return pack;
};
static const Meta meta37 = {37, 0, 0, 0, 0, 51,  7, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37()
{
    Pack * pack = calloc(sizeof(Pack) + meta37.packMinBytes, 1);
    pack->meta = (Meta *)&meta37;
    return pack;
};
static const Meta meta38 = {38, 0, 0, 0, 0, 51,  7, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38()
{
    Pack * pack = calloc(sizeof(Pack) + meta38.packMinBytes, 1);
    pack->meta = (Meta *)&meta38;
    return pack;
};
static const Meta meta39 = {39, 1, 0, 0, 0, 286,  36, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_ITEM_39()
{
    Pack * pack = calloc(sizeof(Pack) + meta39.packMinBytes, 1);
    pack->meta = (Meta *)&meta39;
    return pack;
};
static const Meta meta40 = {40, 1, 0, 0, 0, 35,  5, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_REQUEST_40()
{
    Pack * pack = calloc(sizeof(Pack) + meta40.packMinBytes, 1);
    pack->meta = (Meta *)&meta40;
    return pack;
};
static const Meta meta41 = {41, 1, 0, 0, 0, 32,  4, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_SET_CURRENT_41()
{
    Pack * pack = calloc(sizeof(Pack) + meta41.packMinBytes, 1);
    pack->meta = (Meta *)&meta41;
    return pack;
};
static const Meta meta42 = {42, 1, 0, 0, 0, 16,  2, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_CURRENT_42()
{
    Pack * pack = calloc(sizeof(Pack) + meta42.packMinBytes, 1);
    pack->meta = (Meta *)&meta42;
    return pack;
};
static const Meta meta43 = {43, 0, 0, 0, 0, 19,  3, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_REQUEST_LIST_43()
{
    Pack * pack = calloc(sizeof(Pack) + meta43.packMinBytes, 1);
    pack->meta = (Meta *)&meta43;
    return pack;
};
static const Meta meta44 = {44, 1, 0, 0, 0, 35,  5, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_COUNT_44()
{
    Pack * pack = calloc(sizeof(Pack) + meta44.packMinBytes, 1);
    pack->meta = (Meta *)&meta44;
    return pack;
};
static const Meta meta45 = {45, 0, 0, 0, 0, 19,  3, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_CLEAR_ALL_45()
{
    Pack * pack = calloc(sizeof(Pack) + meta45.packMinBytes, 1);
    pack->meta = (Meta *)&meta45;
    return pack;
};
static const Meta meta46 = {46, 1, 0, 0, 0, 16,  2, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_ITEM_REACHED_46()
{
    Pack * pack = calloc(sizeof(Pack) + meta46.packMinBytes, 1);
    pack->meta = (Meta *)&meta46;
    return pack;
};
static const Meta meta47 = {47, 0, 0, 0, 0, 23,  3, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_ACK_47()
{
    Pack * pack = calloc(sizeof(Pack) + meta47.packMinBytes, 1);
    pack->meta = (Meta *)&meta47;
    return pack;
};
static const Field * meta48fields[] = {&_Lm};
static const Meta meta48 = {48, 0, 0, 0, 0, 104,  14, 1, (Field**)&meta48fields};


Pack * c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48()
{
    Pack * pack = calloc(sizeof(Pack) + meta48.packMinBytes, 1);
    pack->meta = (Meta *)&meta48;
    return pack;
};
static const Field * meta49fields[] = {&_Am};
static const Meta meta49 = {49, 0, 0, 0, 0, 96,  13, 1, (Field**)&meta49fields};


Pack * c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49()
{
    Pack * pack = calloc(sizeof(Pack) + meta49.packMinBytes, 1);
    pack->meta = (Meta *)&meta49;
    return pack;
};
static const Field * meta50fields[] = {&_Fm};
static const Meta meta50 = {50, 0, 0, 0, 0, 168,  22, 1, (Field**)&meta50fields};


Pack * c_CommunicationChannel_new_PARAM_MAP_RC_50()
{
    Pack * pack = calloc(sizeof(Pack) + meta50.packMinBytes, 1);
    pack->meta = (Meta *)&meta50;
    return pack;
};
static const Meta meta51 = {51, 1, 0, 0, 0, 35,  5, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_REQUEST_INT_51()
{
    Pack * pack = calloc(sizeof(Pack) + meta51.packMinBytes, 1);
    pack->meta = (Meta *)&meta51;
    return pack;
};
static const Meta meta54 = {54, 0, 0, 0, 0, 212,  27, 0, NULL};


Pack * c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54()
{
    Pack * pack = calloc(sizeof(Pack) + meta54.packMinBytes, 1);
    pack->meta = (Meta *)&meta54;
    return pack;
};
static const Meta meta55 = {55, 0, 0, 0, 0, 196,  25, 0, NULL};


Pack * c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55()
{
    Pack * pack = calloc(sizeof(Pack) + meta55.packMinBytes, 1);
    pack->meta = (Meta *)&meta55;
    return pack;
};
static const Meta meta61 = {61, 0, 0, 1, 0, 576,  72, 0, NULL};


Pack * c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61()
{
    Pack * pack = calloc(sizeof(Pack) + meta61.packMinBytes, 1);
    pack->meta = (Meta *)&meta61;
    return pack;
};
static const Meta meta62 = {62, 1, 0, 0, 0, 208,  26, 0, NULL};


Pack * c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62()
{
    Pack * pack = calloc(sizeof(Pack) + meta62.packMinBytes, 1);
    pack->meta = (Meta *)&meta62;
    return pack;
};
static const Meta meta63 = {63, 0, 0, 1, 0, 1443,  181, 0, NULL};


Pack * c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63()
{
    Pack * pack = calloc(sizeof(Pack) + meta63.packMinBytes, 1);
    pack->meta = (Meta *)&meta63;
    return pack;
};
static const Meta meta64 = {64, 0, 0, 1, 0, 1795,  225, 0, NULL};


Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64()
{
    Pack * pack = calloc(sizeof(Pack) + meta64.packMinBytes, 1);
    pack->meta = (Meta *)&meta64;
    return pack;
};
static const Meta meta65 = {65, 18, 1, 0, 0, 336,  42, 0, NULL};


Pack * c_CommunicationChannel_new_RC_CHANNELS_65()
{
    Pack * pack = calloc(sizeof(Pack) + meta65.packMinBytes, 1);
    pack->meta = (Meta *)&meta65;
    return pack;
};
static const Meta meta66 = {66, 1, 0, 0, 0, 48,  6, 0, NULL};


Pack * c_CommunicationChannel_new_REQUEST_DATA_STREAM_66()
{
    Pack * pack = calloc(sizeof(Pack) + meta66.packMinBytes, 1);
    pack->meta = (Meta *)&meta66;
    return pack;
};
static const Meta meta67 = {67, 1, 0, 0, 0, 32,  4, 0, NULL};


Pack * c_CommunicationChannel_new_DATA_STREAM_67()
{
    Pack * pack = calloc(sizeof(Pack) + meta67.packMinBytes, 1);
    pack->meta = (Meta *)&meta67;
    return pack;
};
static const Meta meta69 = {69, 1, 0, 0, 0, 88,  11, 0, NULL};


Pack * c_CommunicationChannel_new_MANUAL_CONTROL_69()
{
    Pack * pack = calloc(sizeof(Pack) + meta69.packMinBytes, 1);
    pack->meta = (Meta *)&meta69;
    return pack;
};
static const Meta meta70 = {70, 8, 0, 0, 0, 144,  18, 0, NULL};


Pack * c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70()
{
    Pack * pack = calloc(sizeof(Pack) + meta70.packMinBytes, 1);
    pack->meta = (Meta *)&meta70;
    return pack;
};
static const Meta meta73 = {73, 1, 0, 0, 0, 286,  36, 0, NULL};


Pack * c_CommunicationChannel_new_MISSION_ITEM_INT_73()
{
    Pack * pack = calloc(sizeof(Pack) + meta73.packMinBytes, 1);
    pack->meta = (Meta *)&meta73;
    return pack;
};
static const Meta meta74 = {74, 1, 0, 0, 0, 160,  20, 0, NULL};


Pack * c_CommunicationChannel_new_VFR_HUD_74()
{
    Pack * pack = calloc(sizeof(Pack) + meta74.packMinBytes, 1);
    pack->meta = (Meta *)&meta74;
    return pack;
};
static const Meta meta75 = {75, 0, 0, 0, 0, 267,  34, 0, NULL};


Pack * c_CommunicationChannel_new_COMMAND_INT_75()
{
    Pack * pack = calloc(sizeof(Pack) + meta75.packMinBytes, 1);
    pack->meta = (Meta *)&meta75;
    return pack;
};
static const Meta meta76 = {76, 0, 0, 0, 0, 255,  32, 0, NULL};


Pack * c_CommunicationChannel_new_COMMAND_LONG_76()
{
    Pack * pack = calloc(sizeof(Pack) + meta76.packMinBytes, 1);
    pack->meta = (Meta *)&meta76;
    return pack;
};
static const Field * meta77fields[] = {&_fh, &_Fh, &_uh, &_ch};
static const Meta meta77 = {77, 0, 0, 0, 0, 10,  3, 4, (Field**)&meta77fields};


Pack * c_CommunicationChannel_new_COMMAND_ACK_77()
{
    Pack * pack = calloc(sizeof(Pack) + meta77.packMinBytes, 1);
    pack->meta = (Meta *)&meta77;
    return pack;
};
static const Meta meta81 = {81, 0, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_MANUAL_SETPOINT_81()
{
    Pack * pack = calloc(sizeof(Pack) + meta81.packMinBytes, 1);
    pack->meta = (Meta *)&meta81;
    return pack;
};
static const Meta meta82 = {82, 0, 1, 0, 0, 312,  39, 0, NULL};


Pack * c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82()
{
    Pack * pack = calloc(sizeof(Pack) + meta82.packMinBytes, 1);
    pack->meta = (Meta *)&meta82;
    return pack;
};
static const Meta meta83 = {83, 0, 1, 0, 0, 296,  37, 0, NULL};


Pack * c_CommunicationChannel_new_ATTITUDE_TARGET_83()
{
    Pack * pack = calloc(sizeof(Pack) + meta83.packMinBytes, 1);
    pack->meta = (Meta *)&meta83;
    return pack;
};
static const Meta meta84 = {84, 1, 1, 0, 0, 420,  53, 0, NULL};


Pack * c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84()
{
    Pack * pack = calloc(sizeof(Pack) + meta84.packMinBytes, 1);
    pack->meta = (Meta *)&meta84;
    return pack;
};
static const Meta meta86 = {86, 1, 1, 0, 0, 420,  53, 0, NULL};


Pack * c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86()
{
    Pack * pack = calloc(sizeof(Pack) + meta86.packMinBytes, 1);
    pack->meta = (Meta *)&meta86;
    return pack;
};
static const Meta meta87 = {87, 1, 1, 0, 0, 404,  51, 0, NULL};


Pack * c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87()
{
    Pack * pack = calloc(sizeof(Pack) + meta87.packMinBytes, 1);
    pack->meta = (Meta *)&meta87;
    return pack;
};
static const Meta meta89 = {89, 0, 1, 0, 0, 224,  28, 0, NULL};


Pack * c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89()
{
    Pack * pack = calloc(sizeof(Pack) + meta89.packMinBytes, 1);
    pack->meta = (Meta *)&meta89;
    return pack;
};
static const Meta meta90 = {90, 0, 0, 1, 0, 448,  56, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_STATE_90()
{
    Pack * pack = calloc(sizeof(Pack) + meta90.packMinBytes, 1);
    pack->meta = (Meta *)&meta90;
    return pack;
};
static const Meta meta91 = {91, 0, 0, 1, 0, 332,  42, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_CONTROLS_91()
{
    Pack * pack = calloc(sizeof(Pack) + meta91.packMinBytes, 1);
    pack->meta = (Meta *)&meta91;
    return pack;
};
static const Meta meta92 = {92, 12, 0, 1, 0, 264,  33, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92()
{
    Pack * pack = calloc(sizeof(Pack) + meta92.packMinBytes, 1);
    pack->meta = (Meta *)&meta92;
    return pack;
};
static const Meta meta93 = {93, 0, 0, 2, 0, 644,  81, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93()
{
    Pack * pack = calloc(sizeof(Pack) + meta93.packMinBytes, 1);
    pack->meta = (Meta *)&meta93;
    return pack;
};
static const Field * meta100fields[] = {&_OS, &_ZS};
static const Meta meta100 = {100, 0, 0, 1, 0, 208,  27, 2, (Field**)&meta100fields};


Pack * c_CommunicationChannel_new_OPTICAL_FLOW_100()
{
    Pack * pack = calloc(sizeof(Pack) + meta100.packMinBytes, 1);
    pack->meta = (Meta *)&meta100;
    return pack;
};
static const Meta meta101 = {101, 0, 0, 1, 0, 256,  32, 0, NULL};


Pack * c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101()
{
    Pack * pack = calloc(sizeof(Pack) + meta101.packMinBytes, 1);
    pack->meta = (Meta *)&meta101;
    return pack;
};
static const Meta meta102 = {102, 0, 0, 1, 0, 256,  32, 0, NULL};


Pack * c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102()
{
    Pack * pack = calloc(sizeof(Pack) + meta102.packMinBytes, 1);
    pack->meta = (Meta *)&meta102;
    return pack;
};
static const Meta meta103 = {103, 0, 0, 1, 0, 160,  20, 0, NULL};


Pack * c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103()
{
    Pack * pack = calloc(sizeof(Pack) + meta103.packMinBytes, 1);
    pack->meta = (Meta *)&meta103;
    return pack;
};
static const Meta meta104 = {104, 0, 0, 1, 0, 256,  32, 0, NULL};


Pack * c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104()
{
    Pack * pack = calloc(sizeof(Pack) + meta104.packMinBytes, 1);
    pack->meta = (Meta *)&meta104;
    return pack;
};
static const Meta meta105 = {105, 1, 0, 1, 0, 496,  62, 0, NULL};


Pack * c_CommunicationChannel_new_HIGHRES_IMU_105()
{
    Pack * pack = calloc(sizeof(Pack) + meta105.packMinBytes, 1);
    pack->meta = (Meta *)&meta105;
    return pack;
};
static const Meta meta106 = {106, 0, 2, 1, 0, 352,  44, 0, NULL};


Pack * c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106()
{
    Pack * pack = calloc(sizeof(Pack) + meta106.packMinBytes, 1);
    pack->meta = (Meta *)&meta106;
    return pack;
};
static const Meta meta107 = {107, 0, 1, 1, 0, 512,  64, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_SENSOR_107()
{
    Pack * pack = calloc(sizeof(Pack) + meta107.packMinBytes, 1);
    pack->meta = (Meta *)&meta107;
    return pack;
};
static const Meta meta108 = {108, 0, 0, 0, 0, 672,  84, 0, NULL};


Pack * c_CommunicationChannel_new_SIM_STATE_108()
{
    Pack * pack = calloc(sizeof(Pack) + meta108.packMinBytes, 1);
    pack->meta = (Meta *)&meta108;
    return pack;
};
static const Meta meta109 = {109, 2, 0, 0, 0, 72,  9, 0, NULL};


Pack * c_CommunicationChannel_new_RADIO_STATUS_109()
{
    Pack * pack = calloc(sizeof(Pack) + meta109.packMinBytes, 1);
    pack->meta = (Meta *)&meta109;
    return pack;
};
static const Meta meta110 = {110, 0, 0, 0, 0, 2032,  254, 0, NULL};


Pack * c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110()
{
    Pack * pack = calloc(sizeof(Pack) + meta110.packMinBytes, 1);
    pack->meta = (Meta *)&meta110;
    return pack;
};
static const Meta meta111 = {111, 0, 0, 0, 0, 128,  16, 0, NULL};


Pack * c_CommunicationChannel_new_TIMESYNC_111()
{
    Pack * pack = calloc(sizeof(Pack) + meta111.packMinBytes, 1);
    pack->meta = (Meta *)&meta111;
    return pack;
};
static const Meta meta112 = {112, 0, 1, 1, 0, 96,  12, 0, NULL};


Pack * c_CommunicationChannel_new_CAMERA_TRIGGER_112()
{
    Pack * pack = calloc(sizeof(Pack) + meta112.packMinBytes, 1);
    pack->meta = (Meta *)&meta112;
    return pack;
};
static const Meta meta113 = {113, 4, 0, 1, 0, 288,  36, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_GPS_113()
{
    Pack * pack = calloc(sizeof(Pack) + meta113.packMinBytes, 1);
    pack->meta = (Meta *)&meta113;
    return pack;
};
static const Meta meta114 = {114, 0, 2, 1, 0, 352,  44, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114()
{
    Pack * pack = calloc(sizeof(Pack) + meta114.packMinBytes, 1);
    pack->meta = (Meta *)&meta114;
    return pack;
};
static const Meta meta115 = {115, 2, 0, 1, 0, 512,  64, 0, NULL};


Pack * c_CommunicationChannel_new_HIL_STATE_QUATERNION_115()
{
    Pack * pack = calloc(sizeof(Pack) + meta115.packMinBytes, 1);
    pack->meta = (Meta *)&meta115;
    return pack;
};
static const Meta meta116 = {116, 0, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_SCALED_IMU2_116()
{
    Pack * pack = calloc(sizeof(Pack) + meta116.packMinBytes, 1);
    pack->meta = (Meta *)&meta116;
    return pack;
};
static const Meta meta117 = {117, 2, 0, 0, 0, 48,  6, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_REQUEST_LIST_117()
{
    Pack * pack = calloc(sizeof(Pack) + meta117.packMinBytes, 1);
    pack->meta = (Meta *)&meta117;
    return pack;
};
static const Meta meta118 = {118, 3, 2, 0, 0, 112,  14, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_ENTRY_118()
{
    Pack * pack = calloc(sizeof(Pack) + meta118.packMinBytes, 1);
    pack->meta = (Meta *)&meta118;
    return pack;
};
static const Meta meta119 = {119, 1, 2, 0, 0, 96,  12, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_REQUEST_DATA_119()
{
    Pack * pack = calloc(sizeof(Pack) + meta119.packMinBytes, 1);
    pack->meta = (Meta *)&meta119;
    return pack;
};
static const Meta meta120 = {120, 1, 1, 0, 0, 776,  97, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_DATA_120()
{
    Pack * pack = calloc(sizeof(Pack) + meta120.packMinBytes, 1);
    pack->meta = (Meta *)&meta120;
    return pack;
};
static const Meta meta121 = {121, 0, 0, 0, 0, 16,  2, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_ERASE_121()
{
    Pack * pack = calloc(sizeof(Pack) + meta121.packMinBytes, 1);
    pack->meta = (Meta *)&meta121;
    return pack;
};
static const Meta meta122 = {122, 0, 0, 0, 0, 16,  2, 0, NULL};


Pack * c_CommunicationChannel_new_LOG_REQUEST_END_122()
{
    Pack * pack = calloc(sizeof(Pack) + meta122.packMinBytes, 1);
    pack->meta = (Meta *)&meta122;
    return pack;
};
static const Meta meta123 = {123, 0, 0, 0, 0, 904,  113, 0, NULL};


Pack * c_CommunicationChannel_new_GPS_INJECT_DATA_123()
{
    Pack * pack = calloc(sizeof(Pack) + meta123.packMinBytes, 1);
    pack->meta = (Meta *)&meta123;
    return pack;
};
static const Meta meta124 = {124, 4, 1, 1, 0, 276,  35, 0, NULL};


Pack * c_CommunicationChannel_new_GPS2_RAW_124()
{
    Pack * pack = calloc(sizeof(Pack) + meta124.packMinBytes, 1);
    pack->meta = (Meta *)&meta124;
    return pack;
};
static const Meta meta125 = {125, 2, 0, 0, 0, 38,  5, 0, NULL};


Pack * c_CommunicationChannel_new_POWER_STATUS_125()
{
    Pack * pack = calloc(sizeof(Pack) + meta125.packMinBytes, 1);
    pack->meta = (Meta *)&meta125;
    return pack;
};
static const Meta meta126 = {126, 1, 1, 0, 0, 624,  78, 0, NULL};


Pack * c_CommunicationChannel_new_SERIAL_CONTROL_126()
{
    Pack * pack = calloc(sizeof(Pack) + meta126.packMinBytes, 1);
    pack->meta = (Meta *)&meta126;
    return pack;
};
static const Meta meta127 = {127, 1, 3, 0, 0, 280,  35, 0, NULL};


Pack * c_CommunicationChannel_new_GPS_RTK_127()
{
    Pack * pack = calloc(sizeof(Pack) + meta127.packMinBytes, 1);
    pack->meta = (Meta *)&meta127;
    return pack;
};
static const Meta meta128 = {128, 1, 3, 0, 0, 280,  35, 0, NULL};


Pack * c_CommunicationChannel_new_GPS2_RTK_128()
{
    Pack * pack = calloc(sizeof(Pack) + meta128.packMinBytes, 1);
    pack->meta = (Meta *)&meta128;
    return pack;
};
static const Meta meta129 = {129, 0, 1, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_SCALED_IMU3_129()
{
    Pack * pack = calloc(sizeof(Pack) + meta129.packMinBytes, 1);
    pack->meta = (Meta *)&meta129;
    return pack;
};
static const Meta meta130 = {130, 3, 1, 0, 0, 104,  13, 0, NULL};


Pack * c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130()
{
    Pack * pack = calloc(sizeof(Pack) + meta130.packMinBytes, 1);
    pack->meta = (Meta *)&meta130;
    return pack;
};
static const Meta meta131 = {131, 1, 0, 0, 0, 2040,  255, 0, NULL};


Pack * c_CommunicationChannel_new_ENCAPSULATED_DATA_131()
{
    Pack * pack = calloc(sizeof(Pack) + meta131.packMinBytes, 1);
    pack->meta = (Meta *)&meta131;
    return pack;
};
static const Meta meta132 = {132, 3, 1, 0, 0, 105,  14, 0, NULL};


Pack * c_CommunicationChannel_new_DISTANCE_SENSOR_132()
{
    Pack * pack = calloc(sizeof(Pack) + meta132.packMinBytes, 1);
    pack->meta = (Meta *)&meta132;
    return pack;
};
static const Meta meta133 = {133, 1, 0, 1, 0, 144,  18, 0, NULL};


Pack * c_CommunicationChannel_new_TERRAIN_REQUEST_133()
{
    Pack * pack = calloc(sizeof(Pack) + meta133.packMinBytes, 1);
    pack->meta = (Meta *)&meta133;
    return pack;
};
static const Meta meta134 = {134, 1, 0, 0, 0, 344,  43, 0, NULL};


Pack * c_CommunicationChannel_new_TERRAIN_DATA_134()
{
    Pack * pack = calloc(sizeof(Pack) + meta134.packMinBytes, 1);
    pack->meta = (Meta *)&meta134;
    return pack;
};
static const Meta meta135 = {135, 0, 0, 0, 0, 64,  8, 0, NULL};


Pack * c_CommunicationChannel_new_TERRAIN_CHECK_135()
{
    Pack * pack = calloc(sizeof(Pack) + meta135.packMinBytes, 1);
    pack->meta = (Meta *)&meta135;
    return pack;
};
static const Meta meta136 = {136, 3, 0, 0, 0, 176,  22, 0, NULL};


Pack * c_CommunicationChannel_new_TERRAIN_REPORT_136()
{
    Pack * pack = calloc(sizeof(Pack) + meta136.packMinBytes, 1);
    pack->meta = (Meta *)&meta136;
    return pack;
};
static const Meta meta137 = {137, 0, 1, 0, 0, 112,  14, 0, NULL};


Pack * c_CommunicationChannel_new_SCALED_PRESSURE2_137()
{
    Pack * pack = calloc(sizeof(Pack) + meta137.packMinBytes, 1);
    pack->meta = (Meta *)&meta137;
    return pack;
};
static const Meta meta138 = {138, 0, 0, 1, 0, 288,  36, 0, NULL};


Pack * c_CommunicationChannel_new_ATT_POS_MOCAP_138()
{
    Pack * pack = calloc(sizeof(Pack) + meta138.packMinBytes, 1);
    pack->meta = (Meta *)&meta138;
    return pack;
};
static const Meta meta139 = {139, 0, 0, 1, 0, 344,  43, 0, NULL};


Pack * c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139()
{
    Pack * pack = calloc(sizeof(Pack) + meta139.packMinBytes, 1);
    pack->meta = (Meta *)&meta139;
    return pack;
};
static const Meta meta140 = {140, 0, 0, 1, 0, 328,  41, 0, NULL};


Pack * c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140()
{
    Pack * pack = calloc(sizeof(Pack) + meta140.packMinBytes, 1);
    pack->meta = (Meta *)&meta140;
    return pack;
};
static const Meta meta141 = {141, 0, 0, 1, 0, 256,  32, 0, NULL};
static const Meta meta142 = {142, 0, 0, 0, 0, 1944,  243, 0, NULL};
static const Meta meta143 = {143, 0, 1, 0, 0, 112,  14, 0, NULL};
static const Meta meta144 = {144, 0, 0, 2, 0, 744,  93, 0, NULL};
static const Meta meta146 = {146, 0, 0, 1, 0, 800,  100, 0, NULL};
static const Meta meta147 = {147, 10, 0, 0, 0, 278,  35, 0, NULL};
static const Field * meta148fields[] = {&_qL};
static const Meta meta148 = {148, 2, 4, 1, 0, 433,  56, 1, (Field**)&meta148fields};
static const Field * meta149fields[] = {&_OL, &_ZL, &_DL, &_LL, &_JL};
static const Meta meta149 = {149, 0, 0, 1, 0, 238,  31, 5, (Field**)&meta149fields};
static const Meta meta220 = {220, 0, 0, 1, 0, 256,  32, 0, NULL};
static const Meta meta221 = {221, 21, 0, 0, 0, 336,  42, 0, NULL};
static const Meta meta222 = {222, 0, 0, 0, 0, 24,  3, 0, NULL};
static const Meta meta230 = {230, 0, 0, 1, 0, 331,  42, 0, NULL};
static const Meta meta231 = {231, 0, 0, 1, 0, 320,  40, 0, NULL};
static const Meta meta232 = {232, 1, 1, 1, 0, 496,  62, 0, NULL};
static const Meta meta233 = {233, 0, 0, 0, 0, 1456,  182, 0, NULL};
static const Meta meta234 = {234, 2, 1, 0, 0, 311,  39, 0, NULL};
static const Meta meta241 = {241, 0, 3, 1, 0, 256,  32, 0, NULL};
static const Field * meta242fields[] = {&_lH};
static const Meta meta242 = {242, 0, 0, 0, 0, 416,  53, 1, (Field**)&meta242fields};
static const Field * meta243fields[] = {&_oJ};
static const Meta meta243 = {243, 0, 0, 0, 0, 424,  54, 1, (Field**)&meta243fields};
static const Meta meta244 = {244, 1, 0, 0, 0, 48,  6, 0, NULL};
static const Meta meta245 = {245, 0, 0, 0, 0, 6,  1, 0, NULL};
static const Field * meta246fields[] = {&_vJ};
static const Meta meta246 = {246, 3, 1, 0, 0, 213,  28, 1, (Field**)&meta246fields};
static const Meta meta247 = {247, 0, 1, 0, 0, 135,  17, 0, NULL};
static const Meta meta248 = {248, 1, 0, 0, 0, 2032,  254, 0, NULL};
static const Meta meta249 = {249, 1, 0, 0, 0, 288,  36, 0, NULL};
static const Field * meta250fields[] = {&_XJ};
static const Meta meta250 = {250, 0, 0, 1, 0, 160,  21, 1, (Field**)&meta250fields};
static const Field * meta251fields[] = {&_lJ};
static const Meta meta251 = {251, 0, 1, 0, 0, 64,  9, 1, (Field**)&meta251fields};
static const Field * meta252fields[] = {&_Wn};
static const Meta meta252 = {252, 0, 1, 0, 0, 64,  9, 1, (Field**)&meta252fields};
static const Field * meta253fields[] = {&_sn};
static const Meta meta253 = {253, 0, 0, 0, 0, 3,  2, 1, (Field**)&meta253fields};
static const Meta meta254 = {254, 0, 1, 0, 0, 72,  9, 0, NULL};
static const Meta meta256 = {256, 0, 0, 1, 0, 336,  42, 0, NULL};
static const Meta meta257 = {257, 0, 2, 0, 0, 72,  9, 0, NULL};
static const Field * meta258fields[] = {&_Dn};
static const Meta meta258 = {258, 0, 0, 0, 0, 16,  3, 1, (Field**)&meta258fields};
static const Field * meta259fields[] = {&_Cn};
static const Meta meta259 = {259, 3, 2, 0, 0, 734,  93, 1, (Field**)&meta259fields};
static const Meta meta260 = {260, 0, 1, 0, 0, 35,  5, 0, NULL};
static const Meta meta261 = {261, 0, 1, 0, 0, 216,  27, 0, NULL};
static const Meta meta262 = {262, 0, 2, 0, 0, 144,  18, 0, NULL};
static const Field * meta263fields[] = {&_qA};
static const Meta meta263 = {263, 0, 1, 1, 2, 402,  51, 1, (Field**)&meta263fields};
static const Meta meta264 = {264, 0, 1, 3, 0, 224,  28, 0, NULL};
static const Meta meta265 = {265, 0, 1, 0, 0, 128,  16, 0, NULL};
static const Meta meta266 = {266, 1, 0, 0, 0, 2040,  255, 0, NULL};
static const Meta meta267 = {267, 1, 0, 0, 0, 2040,  255, 0, NULL};
static const Meta meta268 = {268, 1, 0, 0, 0, 32,  4, 0, NULL};
static const Field * meta269fields[] = {&_kA};
static const Meta meta269 = {269, 3, 1, 0, 2, 130,  17, 1, (Field**)&meta269fields};
static const Field * meta270fields[] = {&_RA};
static const Meta meta270 = {270, 3, 1, 0, 2, 138,  18, 1, (Field**)&meta270fields};
static const Field * meta299fields[] = {&_GA, &_MA};
static const Meta meta299 = {299, 0, 0, 0, 2, 2,  1, 2, (Field**)&meta299fields};
static const Meta meta300 = {300, 3, 0, 0, 0, 176,  22, 0, NULL};
static const Meta meta310 = {310, 1, 1, 1, 0, 125,  16, 0, NULL};
static const Field * meta311fields[] = {&_ov};
static const Meta meta311 = {311, 0, 2, 1, 0, 288,  37, 1, (Field**)&meta311fields};
static const Field * meta320fields[] = {&_Hv};
static const Meta meta320 = {320, 0, 0, 0, 0, 32,  5, 1, (Field**)&meta320fields};
static const Meta meta321 = {321, 0, 0, 0, 0, 16,  2, 0, NULL};
static const Field * meta322fields[] = {&_vv, &_fv};
static const Meta meta322 = {322, 2, 0, 0, 2, 38,  5, 2, (Field**)&meta322fields};
static const Field * meta323fields[] = {&_Cv, &_jv};
static const Meta meta323 = {323, 0, 0, 0, 2, 22,  3, 2, (Field**)&meta323fields};
static const Field * meta324fields[] = {&_Tv, &_Nv};
static const Meta meta324 = {324, 0, 0, 0, 2, 8,  1, 2, (Field**)&meta324fields};
static const Meta meta330 = {330, 74, 0, 1, 0, 1259,  158, 0, NULL};

static Pack *  x__W(Pack* pack, int32_t id);
Channel c_CommunicationChannel = {.process =  x__W }; //initialized channel instance


static Pack * x__W(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    Meta * meta = NULL;
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id)
        {
            default:
                //assert(0);
                return NULL;
            case 3:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta3;
                    goto new_pack;
                }
                c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&ph, pack); //handle received pack
                break;
            case 76:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta76;
                    goto new_pack;
                }
                c_CommunicationChannel_on_COMMAND_LONG_76(&ph, pack); //handle received pack
                break;
            case 77:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta77;
                    goto new_pack;
                }
                c_CommunicationChannel_on_COMMAND_ACK_77(&ph, pack); //handle received pack
                break;
            case 81:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta81;
                    goto new_pack;
                }
                c_CommunicationChannel_on_MANUAL_SETPOINT_81(&ph, pack); //handle received pack
                break;
            case 82:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta82;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&ph, pack); //handle received pack
                break;
            case 83:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta83;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ATTITUDE_TARGET_83(&ph, pack); //handle received pack
                break;
            case 84:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta84;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&ph, pack); //handle received pack
                break;
            case 86:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta86;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&ph, pack); //handle received pack
                break;
            case 87:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta87;
                    goto new_pack;
                }
                c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&ph, pack); //handle received pack
                break;
            case 89:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta89;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&ph, pack); //handle received pack
                break;
            case 90:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta90;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_STATE_90(&ph, pack); //handle received pack
                break;
            case 91:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta91;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_CONTROLS_91(&ph, pack); //handle received pack
                break;
            case 92:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta92;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&ph, pack); //handle received pack
                break;
            case 93:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta93;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&ph, pack); //handle received pack
                break;
            case 100:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta100;
                    goto new_pack;
                }
                c_CommunicationChannel_on_OPTICAL_FLOW_100(&ph, pack); //handle received pack
                break;
            case 101:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta101;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&ph, pack); //handle received pack
                break;
            case 102:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta102;
                    goto new_pack;
                }
                c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&ph, pack); //handle received pack
                break;
            case 103:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta103;
                    goto new_pack;
                }
                c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&ph, pack); //handle received pack
                break;
            case 104:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta104;
                    goto new_pack;
                }
                c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&ph, pack); //handle received pack
                break;
            case 105:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta105;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIGHRES_IMU_105(&ph, pack); //handle received pack
                break;
            case 106:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta106;
                    goto new_pack;
                }
                c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&ph, pack); //handle received pack
                break;
            case 107:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta107;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_SENSOR_107(&ph, pack); //handle received pack
                break;
            case 108:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta108;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SIM_STATE_108(&ph, pack); //handle received pack
                break;
            case 109:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta109;
                    goto new_pack;
                }
                c_CommunicationChannel_on_RADIO_STATUS_109(&ph, pack); //handle received pack
                break;
            case 110:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta110;
                    goto new_pack;
                }
                c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&ph, pack); //handle received pack
                break;
            case 111:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta111;
                    goto new_pack;
                }
                c_CommunicationChannel_on_TIMESYNC_111(&ph, pack); //handle received pack
                break;
            case 112:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta112;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CAMERA_TRIGGER_112(&ph, pack); //handle received pack
                break;
            case 113:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta113;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_GPS_113(&ph, pack); //handle received pack
                break;
            case 114:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta114;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&ph, pack); //handle received pack
                break;
            case 115:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta115;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&ph, pack); //handle received pack
                break;
            case 116:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta116;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SCALED_IMU2_116(&ph, pack); //handle received pack
                break;
            case 117:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta117;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&ph, pack); //handle received pack
                break;
            case 118:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta118;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_ENTRY_118(&ph, pack); //handle received pack
                break;
            case 119:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta119;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&ph, pack); //handle received pack
                break;
            case 120:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta120;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_DATA_120(&ph, pack); //handle received pack
                break;
            case 121:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta121;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_ERASE_121(&ph, pack); //handle received pack
                break;
            case 122:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta122;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOG_REQUEST_END_122(&ph, pack); //handle received pack
                break;
            case 123:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta123;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS_INJECT_DATA_123(&ph, pack); //handle received pack
                break;
            case 124:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta124;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS2_RAW_124(&ph, pack); //handle received pack
                break;
            case 125:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta125;
                    goto new_pack;
                }
                c_CommunicationChannel_on_POWER_STATUS_125(&ph, pack); //handle received pack
                break;
            case 126:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta126;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SERIAL_CONTROL_126(&ph, pack); //handle received pack
                break;
            case 127:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta127;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS_RTK_127(&ph, pack); //handle received pack
                break;
            case 128:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta128;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS2_RTK_128(&ph, pack); //handle received pack
                break;
            case 129:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta129;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SCALED_IMU3_129(&ph, pack); //handle received pack
                break;
            case 130:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta130;
                    goto new_pack;
                }
                c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&ph, pack); //handle received pack
                break;
            case 131:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta131;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&ph, pack); //handle received pack
                break;
            case 132:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta132;
                    goto new_pack;
                }
                c_CommunicationChannel_on_DISTANCE_SENSOR_132(&ph, pack); //handle received pack
                break;
            case 133:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta133;
                    goto new_pack;
                }
                c_CommunicationChannel_on_TERRAIN_REQUEST_133(&ph, pack); //handle received pack
                break;
            case 134:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta134;
                    goto new_pack;
                }
                c_CommunicationChannel_on_TERRAIN_DATA_134(&ph, pack); //handle received pack
                break;
            case 135:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta135;
                    goto new_pack;
                }
                c_CommunicationChannel_on_TERRAIN_CHECK_135(&ph, pack); //handle received pack
                break;
            case 136:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta136;
                    goto new_pack;
                }
                c_CommunicationChannel_on_TERRAIN_REPORT_136(&ph, pack); //handle received pack
                break;
            case 137:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta137;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SCALED_PRESSURE2_137(&ph, pack); //handle received pack
                break;
            case 138:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta138;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ATT_POS_MOCAP_138(&ph, pack); //handle received pack
                break;
            case 139:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta139;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&ph, pack); //handle received pack
                break;
            case 140:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta140;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&ph, pack); //handle received pack
                break;
            case 141:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta141;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ALTITUDE_141(&ph, pack); //handle received pack
                break;
            case 142:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta142;
                    goto new_pack;
                }
                c_CommunicationChannel_on_RESOURCE_REQUEST_142(&ph, pack); //handle received pack
                break;
            case 143:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta143;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SCALED_PRESSURE3_143(&ph, pack); //handle received pack
                break;
            case 144:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta144;
                    goto new_pack;
                }
                c_CommunicationChannel_on_FOLLOW_TARGET_144(&ph, pack); //handle received pack
                break;
            case 146:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta146;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&ph, pack); //handle received pack
                break;
            case 147:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta147;
                    goto new_pack;
                }
                c_CommunicationChannel_on_BATTERY_STATUS_147(&ph, pack); //handle received pack
                break;
            case 148:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta148;
                    goto new_pack;
                }
                c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&ph, pack); //handle received pack
                break;
            case 149:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta149;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LANDING_TARGET_149(&ph, pack); //handle received pack
                break;
            case 220:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta220;
                    goto new_pack;
                }
                c_CommunicationChannel_on_NAV_FILTER_BIAS_220(&ph, pack); //handle received pack
                break;
            case 221:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta221;
                    goto new_pack;
                }
                c_CommunicationChannel_on_RADIO_CALIBRATION_221(&ph, pack); //handle received pack
                break;
            case 222:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta222;
                    goto new_pack;
                }
                c_CommunicationChannel_on_UALBERTA_SYS_STATUS_222(&ph, pack); //handle received pack
                break;
            case 230:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta230;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&ph, pack); //handle received pack
                break;
            case 231:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta231;
                    goto new_pack;
                }
                c_CommunicationChannel_on_WIND_COV_231(&ph, pack); //handle received pack
                break;
            case 232:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta232;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS_INPUT_232(&ph, pack); //handle received pack
                break;
            case 233:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta233;
                    goto new_pack;
                }
                c_CommunicationChannel_on_GPS_RTCM_DATA_233(&ph, pack); //handle received pack
                break;
            case 234:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta234;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HIGH_LATENCY_234(&ph, pack); //handle received pack
                break;
            case 241:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta241;
                    goto new_pack;
                }
                c_CommunicationChannel_on_VIBRATION_241(&ph, pack); //handle received pack
                break;
            case 242:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta242;
                    goto new_pack;
                }
                c_CommunicationChannel_on_HOME_POSITION_242(&ph, pack); //handle received pack
                break;
            case 243:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta243;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_HOME_POSITION_243(&ph, pack); //handle received pack
                break;
            case 244:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta244;
                    goto new_pack;
                }
                c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&ph, pack); //handle received pack
                break;
            case 245:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta245;
                    goto new_pack;
                }
                c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&ph, pack); //handle received pack
                break;
            case 246:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta246;
                    goto new_pack;
                }
                c_CommunicationChannel_on_ADSB_VEHICLE_246(&ph, pack); //handle received pack
                break;
            case 247:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta247;
                    goto new_pack;
                }
                c_CommunicationChannel_on_COLLISION_247(&ph, pack); //handle received pack
                break;
            case 248:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta248;
                    goto new_pack;
                }
                c_CommunicationChannel_on_V2_EXTENSION_248(&ph, pack); //handle received pack
                break;
            case 249:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta249;
                    goto new_pack;
                }
                c_CommunicationChannel_on_MEMORY_VECT_249(&ph, pack); //handle received pack
                break;
            case 250:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta250;
                    goto new_pack;
                }
                c_CommunicationChannel_on_DEBUG_VECT_250(&ph, pack); //handle received pack
                break;
            case 251:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta251;
                    goto new_pack;
                }
                c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&ph, pack); //handle received pack
                break;
            case 252:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta252;
                    goto new_pack;
                }
                c_CommunicationChannel_on_NAMED_VALUE_INT_252(&ph, pack); //handle received pack
                break;
            case 253:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta253;
                    goto new_pack;
                }
                c_CommunicationChannel_on_STATUSTEXT_253(&ph, pack); //handle received pack
                break;
            case 254:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta254;
                    goto new_pack;
                }
                c_CommunicationChannel_on_DEBUG_254(&ph, pack); //handle received pack
                break;
            case 256:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta256;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SETUP_SIGNING_256(&ph, pack); //handle received pack
                break;
            case 257:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta257;
                    goto new_pack;
                }
                c_CommunicationChannel_on_BUTTON_CHANGE_257(&ph, pack); //handle received pack
                break;
            case 258:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta258;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PLAY_TUNE_258(&ph, pack); //handle received pack
                break;
            case 259:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta259;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CAMERA_INFORMATION_259(&ph, pack); //handle received pack
                break;
            case 260:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta260;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CAMERA_SETTINGS_260(&ph, pack); //handle received pack
                break;
            case 261:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta261;
                    goto new_pack;
                }
                c_CommunicationChannel_on_STORAGE_INFORMATION_261(&ph, pack); //handle received pack
                break;
            case 262:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta262;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&ph, pack); //handle received pack
                break;
            case 263:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta263;
                    goto new_pack;
                }
                c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&ph, pack); //handle received pack
                break;
            case 264:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta264;
                    goto new_pack;
                }
                c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&ph, pack); //handle received pack
                break;
            case 265:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta265;
                    goto new_pack;
                }
                c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&ph, pack); //handle received pack
                break;
            case 266:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta266;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOGGING_DATA_266(&ph, pack); //handle received pack
                break;
            case 267:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta267;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&ph, pack); //handle received pack
                break;
            case 268:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta268;
                    goto new_pack;
                }
                c_CommunicationChannel_on_LOGGING_ACK_268(&ph, pack); //handle received pack
                break;
            case 269:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta269;
                    goto new_pack;
                }
                c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&ph, pack); //handle received pack
                break;
            case 270:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta270;
                    goto new_pack;
                }
                c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&ph, pack); //handle received pack
                break;
            case 299:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta299;
                    goto new_pack;
                }
                c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&ph, pack); //handle received pack
                break;
            case 300:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta300;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PROTOCOL_VERSION_300(&ph, pack); //handle received pack
                break;
            case 310:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta310;
                    goto new_pack;
                }
                c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&ph, pack); //handle received pack
                break;
            case 311:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta311;
                    goto new_pack;
                }
                c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&ph, pack); //handle received pack
                break;
            case 320:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta320;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&ph, pack); //handle received pack
                break;
            case 321:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta321;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&ph, pack); //handle received pack
                break;
            case 322:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta322;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&ph, pack); //handle received pack
                break;
            case 323:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta323;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PARAM_EXT_SET_323(&ph, pack); //handle received pack
                break;
            case 324:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta324;
                    goto new_pack;
                }
                c_CommunicationChannel_on_PARAM_EXT_ACK_324(&ph, pack); //handle received pack
                break;
            case 330:
                if(pack == NULL) //request to create new empty pack
                {
                    meta = (Meta *)&meta330;
                    goto new_pack;
                }
                c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&ph, pack); //handle received pack
                break;
            case PROCESS_CHANNEL_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
                if(RBUF_ISFULL(received_packs)) return pack;
                RBUF_PUT(received_packs, pack)
                return NULL;
            case PROCESS_HOST_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(received_packs)) ? NULL : RBUF_GET(received_packs);
                if(RBUF_ISFULL(sendout_packs)) return pack;
                RBUF_PUT(sendout_packs, pack)
                return NULL;
            case PROCESS_RECEIVED_PACKS:
                LOOP = true;
                goto next_pack;
        }
        free(pack); // dispose pack
        if(!LOOP) return NULL;
next_pack:
        if(RBUF_ISEMPTY(received_packs)) return NULL;
        pack = RBUF_GET(received_packs);
        id = pack->meta->id;
        setPack(pack, &ph);
    }
new_pack:
    pack = calloc(sizeof(Pack) + (meta->fields_count ? 0 : meta->packMinBytes), 1);
    pack->meta = meta;
    return pack;
}


import org.unirail.BlackBox.*;
public class minimal {}
class LoopBackDemoChannel extends SimpleProtocol implements DemoDevice.MainInterface, DemoDevice.LoopBack {}
class LoopBackDemoChannel_ADV extends AdvancedProtocol implements DemoDevice.MainInterface, DemoDevice.LoopBack {}
class DemoDevice implements InC, InJAVA, InCS
{
    interface LoopBack extends MainInterface {} interface MainInterface
    {

        /*
        Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
        @id(0) class HEARTBEAT
        {
            MAV_TYPE type;//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
            MAV_AUTOPILOT autopilot;//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
            @A byte  base_mode;//System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
            @A int  custom_mode;//A bitfield for use for autopilot-specific flags.
            MAV_STATE system_status;//System status flag, see MAV_STATE ENUM
            @A byte  mavlink_version;//MAVLink version
        }
    }
}

/*
Generic autopilot, full support for everything*/
enum MAV_AUTOPILOT
{
    ;

    final int
    MAV_AUTOPILOT_GENERIC = 0, //Generic autopilot, full support for everything
    MAV_AUTOPILOT_PIXHAWK = 1, //PIXHAWK autopilot, http://pixhawk.ethz.ch
    MAV_AUTOPILOT_SLUGS = 2, //SLUGS autopilot, http://slugsuav.soe.ucsc.edu
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3, //ArduPilotMega / ArduCopter, http://diydrones.com
    MAV_AUTOPILOT_OPENPILOT = 4, //OpenPilot, http://openpilot.org
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5, //Generic autopilot only supporting simple waypoints
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, //Generic autopilot supporting waypoints and other simple navigation commands
    MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7, //Generic autopilot supporting the full mission command set
    MAV_AUTOPILOT_INVALID = 8, //No valid autopilot, e.g. a GCS or other MAVLink component
    MAV_AUTOPILOT_PPZ = 9, //PPZ UAV - http://nongnu.org/paparazzi
    MAV_AUTOPILOT_UDB = 10, //UAV Dev Board
    MAV_AUTOPILOT_FP = 11;  //FlexiPilot
}

/*
Generic micro air vehicle.*/
enum MAV_TYPE
{
    ;

    final int
    MAV_TYPE_GENERIC = 0, //Generic micro air vehicle.
    MAV_TYPE_FIXED_WING = 1, //Fixed wing aircraft.
    MAV_TYPE_QUADROTOR = 2, //Quadrotor
    MAV_TYPE_COAXIAL = 3, //Coaxial helicopter
    MAV_TYPE_HELICOPTER = 4, //Normal helicopter with tail rotor.
    MAV_TYPE_ANTENNA_TRACKER = 5, //Ground installation
    MAV_TYPE_GCS = 6, //Operator control unit / ground control station
    MAV_TYPE_AIRSHIP = 7, //Airship, controlled
    MAV_TYPE_FREE_BALLOON = 8, //Free balloon, uncontrolled
    MAV_TYPE_ROCKET = 9, //Rocket
    MAV_TYPE_GROUND_ROVER = 10, //Ground rover
    MAV_TYPE_SURFACE_BOAT = 11, //Surface vessel, boat, ship
    MAV_TYPE_SUBMARINE = 12, //Submarine
    MAV_TYPE_HEXAROTOR = 13, //Hexarotor
    MAV_TYPE_OCTOROTOR = 14, //Octorotor
    MAV_TYPE_TRICOPTER = 15, //Octorotor
    MAV_TYPE_FLAPPING_WING = 16;  //Flapping wing
}

/*
0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.*/
enum MAV_MODE_FLAG
{
    ;

    final int
    MAV_MODE_FLAG_SAFETY_ARMED = 128, //0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, //0b01000000 remote control input is enabled.
    /*
    0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software
    is full operational*/
    MAV_MODE_FLAG_HIL_ENABLED = 32,
    /*
    0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further
    control inputs to move around*/
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    MAV_MODE_FLAG_GUIDED_ENABLED = 8, //0b00001000 guided mode enabled, system flies waypoints / mission items.
    /*
    0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not,
    depends on the actual implementation*/
    MAV_MODE_FLAG_AUTO_ENABLED = 4,
    /*
    0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should
    not be used for stable implementations*/
    MAV_MODE_FLAG_TEST_ENABLED = 2,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;  //0b00000001 Reserved for future use.
}

/*
First bit:  10000000*/
enum MAV_MODE_FLAG_DECODE_POSITION
{
    ;

    final int
    MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128, //First bit:  10000000
    MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64, //Second bit: 01000000
    MAV_MODE_FLAG_DECODE_POSITION_HIL = 32, //Third bit:  00100000
    MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16, //Fourth bit: 00010000
    MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8, //Fifth bit:  00001000
    MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4, //Sixt bit:   00000100
    MAV_MODE_FLAG_DECODE_POSITION_TEST = 2, //Seventh bit: 00000010
    MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1;  //Eighth bit: 00000001
}

/*
Uninitialized system, state is unknown.*/
enum MAV_STATE
{
    MAV_STATE_BOOT, //System is booting up.
    MAV_STATE_CALIBRATING, //System is calibrating and not flight-ready.
    MAV_STATE_STANDBY, //System is grounded and on standby. It can be launched any time.
    MAV_STATE_ACTIVE, //System is active and might be already airborne. Motors are engaged.
    MAV_STATE_CRITICAL, //System is in a non-normal flight mode. It can however still navigate.
    /*
    System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in
    mayday and going down*/
    MAV_STATE_EMERGENCY,
    MAV_STATE_POWEROFF;  //System just initialized its power-down sequence, will shut down now.
    final int
    MAV_STATE_UNINIT = 0;  //Uninitialized system, state is unknown.
}

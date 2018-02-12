This project is designed to demonstrate the capabilities of [**BlackBox**](https://github.com/cheblin/BlackBox). As the model we have chosen the [**MAVLink**](http://qgroundcontrol.org/mavlink/start), a well-known protocol for managing a variety of unmanned devices. In general, **BlackBox**, has significantly more variety of data types, in comparison with **MAVLink**, and this demonstration shows only some of the **BlackBox** features. 

The project repository has the following structure.
Each directory corresponds to one of the [MAVLink’s](https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0) [XML protocol description files](https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0). 
In the directory, in addition to a copy of the original [XML](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.xml) [file](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.xml) itself, there is its image in the **BlackBox** format, a [JAVA](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.java) [file](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.java), as the result of the converter program [**MavLink2BlackBox.java**](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/MavLink2BlackBox.java)

**MAVLink**, unlike **BlackBox** does not describe the network topology. Therefore, for a more complete demonstration of the **BlackBox** possibilities, **MAVLink** packages are divided into three parts.

1. **MicroAirVehicleHandledPacks** - packets that can be received and processed only on the unmanned device side **MicroAirVehicle**
2. **GroundControlHandledPacks** - packages that can be received and processed only by the control panel **GroundControl**
3. **CommonPacks** - packages that are sent and accepted by either party.

And the following demonstration network topology is written to the resulting converter file.

![](http://www.unirail.org/wp-content/uploads/2018/02/Scheme.png)

In a simplified form, the description file looks like this:
```java
class CommunicationChannel extends SimpleProtocol implements GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface {}

class GroundControl implements InJAVA, InCS {
   interface CommunicationInterface extends GroundControlHandledPacks, CommonPacks {}
   
   interface GroundControlHandledPacks {
      /*
      Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
      @id(0) class HEARTBEAT {
         MAV_TYPE      type;//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
         MAV_AUTOPILOT autopilot;//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
         MAV_MODE_FLAG base_mode;//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
         @A int custom_mode;//A bitfield for use for autopilot-specific flags.
         MAV_STATE system_status;//System status flag, see MAV_STATE ENUM
         @A byte mavlink_version;//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
      }
     	. . . .
	 . . . .
   }
   
   interface CommonPacks {
      /*
      Current airspeed in m/s*/
      @id(74) class VFR_HUD {
         float airspeed;//Current airspeed in m/s
         float groundspeed;//Current ground speed in m/s
         short heading;//Current heading in degrees, in compass units (0..360, 0=north)
         @A short throttle;//Current throttle setting in integer percent, 0 to 100
         float alt;//Current altitude (MSL), in meters
         float climb;//Current climb rate in meters/second
      }
      
   }
}

class MicroAirVehicle implements InC {
   interface CommunicationInterface extends MicroAirVehicleHandledPacks, GroundControl.CommonPacks {}
   
   interface MicroAirVehicleHandledPacks {
      
      /*
      Timestamp (micros since boot or Unix epoch)*/
      @id(139) class SET_ACTUATOR_CONTROL_TARGET {
         @A    long    time_usec;//Timestamp (micros since boot or Unix epoch)
         /*
         Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
         this field to difference between instances*/
         @A    byte    group_mlx;
         @A    byte    target_system;//System ID
         @A    byte    target_component;//Component ID
         /*
         Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
         motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
         (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
         mixer to repurpose them as generic outputs*/
         @D(8) float[] controls;
      }
	. . . . .
      	 . . . . . 
   }
}
 . . . . .
. . . . . 

/**
 * additional active features/modifiers/constraints.
 */
@BitFlags enum AUTOQUAD_NAV_STATUS {
   ;
   
   final int
         AQ_NAV_STATUS_INIT            = 0, //System is initializing
         AQ_NAV_STATUS_STANDBY         = 0x00000001, //System is *armed* and standing by, with no throttle input and no autonomous mode
         AQ_NAV_STATUS_MANUAL          = 0x00000002, //Flying (throttle input detected), assumed under manual control unless other mode bits are set
         AQ_NAV_STATUS_ALTHOLD         = 0x00000004, //Altitude hold engaged
         AQ_NAV_STATUS_POSHOLD         = 0x00000008, //Position hold engaged
         AQ_NAV_STATUS_GUIDED          = 0x00000010, //Externally-guided (eg. GCS) navigation mode
         AQ_NAV_STATUS_MISSION         = 0x00000020, //Autonomous mission execution mode
         AQ_NAV_STATUS_READY           = 0x00000100, //Ready but *not armed*
         AQ_NAV_STATUS_CALIBRATING     = 0x00000200, //Calibration mode active
         AQ_NAV_STATUS_NO_RC           = 0x00001000, //No valid control input (eg. no radio link)
         AQ_NAV_STATUS_FUEL_LOW        = 0x00002000, //Battery is low (stage 1 warning)
         AQ_NAV_STATUS_FUEL_CRITICAL   = 0x00004000, //Battery is depleted (stage 2 warning)
         AQ_NAV_STATUS_DVH             = 0x01000000, //Dynamic Velocity Hold is active (PH with proportional manual direction override)
         AQ_NAV_STATUS_DAO             = 0x02000000, //ynamic Altitude Override is active (AH with proportional manual adjustment)
         AQ_NAV_STATUS_CEILING_REACHED = 0x04000000, //Craft is at ceiling altitude
         AQ_NAV_STATUS_CEILING         = 0x08000000, //Ceiling altitude is set
         AQ_NAV_STATUS_HF_DYNAMIC      = 0x10000000, //Heading-Free dynamic mode active
         AQ_NAV_STATUS_HF_LOCKED       = 0x20000000, //Heading-Free locked mode active
         AQ_NAV_STATUS_RTH             = 0x40000000, //Automatic Return to Home is active
         AQ_NAV_STATUS_FAILSAFE        = 0x80000000;  //System is in failsafe recovery mode
}
```
Each of the project directories also contains the generated **BlackBox** source code, distributed over the folders of the specified programming languages.

For example, according to the above descriptor, the connections between the **GroundControl** and **MicroAirVehicle** hosts will occur via the **SimpleProtocol** channel. For **GroundControl**, the source code will be generated in the [**JAVA**](https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InJAVA/GroundControl/org/noname) and [**C#**](https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/IncS/GroundControl) languages, and for **MicroAirVehicle** in [**C**](https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InC/MicroAirVehicle) only.

Each of the programming language folder, **JAVA** for example, contains:

- [**API** source file](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/GroundControl.java) - means of creating, filling, sending, receiving and parsing packages.
- [file **of one of the** tests](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/Test.java), where this **API** has been successfully tested
- [file with a demonstration](https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/Demo.java) using the generated  **API**
- [**API** support libraries](https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InJAVA/GroundControl/org/unirail/BlackBox)

If you have a project using **MAVLink** as an exchange protocol, then you should try **BlackBox**, which would allow you to benefit additional features such as:
- [Base 128 Varint](https://developers.google.com/protocol-buffers/docs/encoding) data compression
- fields with the built-in structure of multidimensional arrays,
- fields of sparse multidimensional arrays
- bit fields,
- fields with the built-in structure of multidimensional arrays of bits.
 
To do this, you need to do the following:

1. Download and install any **JAVA** code editor convenient for you. This can be an [IntelliJ IDEA](https://www.jetbrains.com/idea/), or [Android Studio](https://developer.android.com/studio/index.html), if you plan to program for the Android platform.
2. Create the simplest empty JAVA project in the installed development environment.
3. Download and connect to the JAVA project [annotations](https://github.com/cheblin/BlackBox/tree/master/org/unirail/BlackBox), needed to describe the data exchanged between the devices. For **Android Studio** you can download the [ready-made template of a similar project](https://github.com/cheblin/BlackBox/raw/master/BlackBoxDescriptionEditor.zip).
4. Create a file in the project with the source code **JAVA**, and [following the rules](http://www.unirail.org/) to describe the data of the protocol of exchange in the format [**BlackBox**]( https://github.com/cheblin/BlackBox/raw/master/BlackBoxDescriptionEditor.zip).
5. Verify that the resulting **JAVA** file is compiled without errors, and send it in an attachment to the mailbox [**OneBlackBoxPlease@outlook.com**](mailto:OneBlackBoxPlease@outlook.com). In subject field of the letter you can specify the name of the project.
6. If the file you have sent was received and does not contain errors, then you will receive a confirmation letter, otherwise you will get a notification of refusal with pointed out reason.
7. If the system has accepted the file for processing, after some time, depending on the server load, an archive containing the generated and already tested sources codes in the specified programming languages will be sent back.

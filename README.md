# MAVLink2BlackBox_Demo
This project is designed to demonstrate the capabilities of [** BlackBox **] (https://github.com/cheblin/BlackBox). As the model e have chosen the [** MAVLink **] (http://qgroundcontrol.org/mavlink/start), a well-known protocol for managing a variety of unmanned devices. In general, ** BlackBox **, has significantly more variety of data types, in comparison with ** MAVLink **, and this demonstration shows only some of the ** BlackBox ** features. 

The project repository has the following structure.
Each directory corresponds to one of the [MAVLink]’s (https://github.com/mavlink/mavlink/ tree / master / message_definitions / v1.0) [XML protocol description files] (https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0). 
In the directory, in addition to a copy of the original [XML] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.xml) [file] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ ASLUAV / ASLUAV.xml) itself, there is its image in the ** BlackBox ** format, a [JAVA] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/ASLUAV.java) [file] (https://github.com/cheblin/ MAVLink2BlackBox_Demo / blob / master / ASLUAV / ASLUAV.java), as the result of the converter program [** MavLink2BlackBox.java **] (https://github.com/cheblin/MAVLink2BlackBox_Demo /blob/master/MavLink2BlackBox.java)

** MAVLink **, unlike ** BlackBox ** does not describe the network topology. Therefore, for a more complete demonstration of the ** BlackBox **possibilities, ** MAVLink ** packages are divided into three parts.

1. ** MicroAirVehicleHandledPacks ** - packets that can be received and processed only on the unmanned device side ** MicroAirVehicle **
2. ** GroundControlHandledPacks ** - packages that can be received and processed only by the control panel ** GroundControl **
3. ** CommonPacks - ** packages that are sent and accepted by either party.

And the following demonstration network topology is written to the resulting converter file.

! [] (http://www.unirail.org/wp-content/uploads/2018/02/Scheme.png)

In a simplified form, the description file looks like this:



Each of the project directories also contains the generated ** BlackBox ** source code, distributed over the folders of the specified programming languages.

For example, according to the above descriptor, the connections between the ** GroundControl ** and ** MicroAirVehicle ** hosts will occur via the ** SimpleProtocol ** channel. For ** GroundControl **, the source code will be generated in the languages [** JAVA **] (https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InJAVA/GroundControl/org/noname) and [** C **] (https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/IncS/GroundControl) [** # **] (https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master / ASLUAV / InCS / GroundControl), and for ** MicroAirVehicle ** in [** C **] (https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InC/MicroAirVehicle) only **. * *

Each of the programming language folder, such as, for example ** JAVA **, contains:

- [source file] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/GroundControl.java) [** API **] (https: // github. com / cheblin / MAVLink2BlackBox_Demo / blob / master / ASLUAV / InJAVA / GroundControl / org / noname / GroundControl.java) - a tool for creating, filling, sending, receiving and parsing packages.
- [file ** of one of the ** tests] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/Test.java), where this ** API * * has been successfully tested
- [file with a demonstration] (https://github.com/cheblin/MAVLink2BlackBox_Demo/blob/master/ASLUAV/InJAVA/GroundControl/org/noname/Demo.java) using the generated ** API **
- [support libraries] (https://github.com/cheblin/MAVLink2BlackBox_Demo/tree/master/ASLUAV/InJAVA/GroundControl/org/unirail/BlackBox) [** API **] (https://github.com/ cheblin / MAVLink2BlackBox_Demo / tree / master / ASLUAV / InJAVA / GroundControl / org / unirail / BlackBox)

If you have a project using ** MAVLink ** as an exchange protocol, then you should try ** BlackBox **, which would allow you to benefit additional features such as:
- [Base 128 Varint] (https://developers.google.com/protocol-buffers/docs/encoding) data compression
- fields with the built-in structure of multidimensional arrays,
- fields of sparse multidimensional arrays
- bit fields,
- fields with the built-in structure of multidimensional arrays of bits.
 
To do this, you need to do the following:

1. Download and install any JAVA code editor convenient for you. This can be an [IntelliJ] (https://www.jetbrains.com/idea/) [IDEA] (https://www.jetbrains.com/idea/), or [Android] (https: // developer .android.com / studio / index.html) [Studio] (https://developer.android.com/studio/index.html), if you plan to program for the Android platform.
2. Create the simplest empty JAVA project in the installed development environment.
3. Download and connect to the JAVA project [annotations] (https://github.com/cheblin/BlackBox/tree/master/org/unirail/BlackBox), needed to describe the data exchanged between the devices. For ** Android Studio ** you can download the [ready-made template of a similar project] (https://github.com/cheblin/BlackBox/raw/master/BlackBoxDescriptionEditor.zip).
4. Create a file in the project with the source code ** JAVA **, and [following the rules] (http://www.unirail.org/?lang=en) to describe the data of the protocol of exchange in the format [** BlackBox **] ( https://github.com/cheblin/BlackBox/raw/master/BlackBoxDescriptionEditor.zip).
5. Verify that the resulting ** JAVA ** file is compiled without errors, and send it in an attachment to the mailbox [** OneBlackBoxPlease@outlook.com **] (mailto: OneBlackBoxPlease@outlook.com) **. ** In subject field of the letter you can specify the name of the project.
6. If the file you have sent was received and does not contain errors, then you will receive a confirmation letter, otherwise you will get a notification of refusal with pointed out reason.
7. If the system has accepted the file for processing, after some time, depending on the server load, an archive containing the generated and already tested sources codes in the specified programming languages will be sent back.

# 0220

## 1.rosserial_arduino/Tutorial 중 
```rosrun rosserial_python serial_node.py /dev/ttyUSB0```

실행시 다음과 같은 에러 발생

```ESP32:Protocol version of client is unrecognized, expected Rev 1 (rosserial 0.5+)```
## Sol)
~/Arduino/libraries/ros_lib에서 ros.h 편집
```
#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP) 
  #include "ArduinoTcpHardware.h" 
#else 
  #include "ArduinoHardware.h" 
#endif
```
을 다음과 같이 수정
```
// #if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP) 
#if defined(ROSSERIAL_ARDUINO_TCP) 
  #include "ArduinoTcpHardware.h" 
#else 
  #include "ArduinoHardware.h" 
#endif
```
Arduino IDE 재시작 후 하면 해결


## 2. 
```
class ArduinoHardware' has no member named 'setConnection' nh.getHardware()->setConnection(server, serverPort); ^ exit status 1 'class ArduinoHardware' has no member named 'setConnection'
```
에러 발생

## Sol)
 make sure you add ```#define ROSSERIAL_ARDUINO_TCP``` before including the file "ros.h".
 ```
 #define ROSSERIAL_ARDUINO_TCP
#include "ros.h"
IPAddress server(192, 168, 0, 100);
```
***
# 0413
##  arduino ros custom message 쓰는법
https://m.blog.naver.com/nswve/222040768394 

* custom message를 위한 패키지를 생성할때는 항상
catkin_create_pkg uwb_msgs message_generation std_msgs roscpp message_runtime
'message_generation' 이랑 'message_runtime' 모두 추가해줘야 함. 아니면 catkin build에러발생함

* "ros.h" 를 arduino UNO에 include하니 변수가 많아져서 SRAM 2KB를 초과하여 upload가 안됨
SRAM이 더 높은 MEGA나 ESP32보드로 해봐야할듯
***
# 0419
LinkTrack parser를 사용하여 AOA센서의 거리 및 각도 데이터를 rostopic으로 발행하였다. 거리 및 각도 데이터는 array형태로 들어오는데 이를 subscriber를 통해 읽으려고 callback함수를 실행시키는데 잠깐 동안 subscribe하다가
`"segmentation fault(core dumped)"` 가 뜨면서 subscrube가 중단되는 현상이 빈번하게 발생하였다.
`"segmentation fault(core dumped)"`에러는 비어있는 포인터를 dereference할 때 발생한다고 한다. 즉, AOA센서는 중간중간 가끔 데이터를 못받아 와서 빈 array가 생기는데 이 포인터를 subscribe하려고 하니까 발생하는 것이다.

* 이를 해결하기 위해 subscrube할 때 array가 비어있는지 확인하는 작업이 필요하다. array.empty() 함수를 사용하여 빈 포인터가 아닐 때에만 subscribe하니 잘된다.
***
# 0415
## 1. Adding Pozyx arduino file to turtlebot_core arduino file
- upload done
- when `roslaunch turtlebot3_bringup turtlebot3_robot.launch`, there is an buffer error that

`[ERROR] [1521650797.920229]: Message from device dropped: message larger than buffer`
## Sol) 
The solution is to fix `~/Arduino/libraries/ros_lib/ros/node_handle.h`

increase the `OUTPUT_SIZE` to 1024
(https://github.com/ROBOTIS-GIT/turtlebot3/issues/160)

## 2. 
I Edited turtlebot_core arduino file to publish the uwb msgs to ROS and uploading successed.

but when I typed `roslaunch turtlebot3_bringup turtlebot3_robot.launch`, the error occured that

`[ERROR] [1681545839.060578]: Cannot import package : uwb_msg`

Which means it could't find uwb_msg package.

## Sol)
 The solution is to edit the `package.xml`, `CMakeLists.txt` files in `tuetlebot3_bringup` package.
(http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages#Dependencies)

I added these two lines in 'package.xml'file
```
<build_depend>name_of_package_containing_custom_msg</build_depend>
<exec_depend>name_of_package_containing_custom_msg</exec_depend>
```
***
# 0420

#1
```
	0x1179(-300,943,227)			door
			
					0x1166(173, 710,245)









0x116F(-435,51,209)
			        0x113B(0,0,253)
```
* Measured localization with 4 anchors which have little different height
The result is that x, y coordinate is good but z coordinate is very bad.
So, I'll add another 5th anchor with low height.

#2
```
	0x1179(-300,943,245)			door
			
					0x1166(173,710,50)




					0x1102(173,590,110)




0x116F(-435,51,209)
			        0x113B(0,0,253)
```

#3
```
0x1179(-300,943,245)			door
			
					0x1166(173,710,50)



0x116F(-505,497,220)
					0x1102(173,590,110)





			        0x113B(0,0,253)
```

#4
```
	0x1179(-300,943,245)			door
			
					0x1166(173,710,50)



0x116F(-473,467,32)
					0x1102(173,590,110)





			        0x113B(0,0,253)
```

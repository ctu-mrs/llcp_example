## llcp_example

This package shows how to use the [mrs_llcp](https://github.com/ctu-mrs/mrs_llcp) library to communicate with low level devices.
Llcp_example consists of two parts:
* Arduino code as an example for a low level device
* A ROS node which communicates with the Arduino through [mrs_llcp_ros](https://github.com/ctu-mrs/mrs_llcp_ros)
To upload the Arduino code and test it, you need to have the [mrs_llcp](https://github.com/ctu-mrs/mrs_llcp) library in the Arduino libraries folder.

To run the example:
* Upload the sketch from the arduino_example folder to your arduino
* Launch the example.launch file, it will start two ROS nodes:
  * [mrs_llcp_ros](https://github.com/ctu-mrs/mrs_llcp_ros) which handles the serial port on the ROS side
  * llcp_example, which is the user ROS node


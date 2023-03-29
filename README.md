ROS driver for VL53MKAC laser.

User Guide:
	1. move FIT-XXX file into ros workspace
	2. run catkin_make in terminal in ros workspace
	3. modify host_ip and port num for your laser.
	4. set your Ethernet IPv4 address to 192.168.1.xxx (different than your host_ip and Netmask to 255.255.255.0
	5. roslaunch vl53mkac VL53MKAC.launch or roslaunch vl53mkac VL53MKAC_display.launch to start vl53mkac ros node.


ROS topic: sensor_msgs/LaserScan -> "/scan"

TF frame: /laser (default)

ROS Service:
	1. "/vl53mkac/disconnect_laser_srv"
		* disconnect laser, require restart the node to reconnect laser.

	2. "/vl53mkac/start_laser_srv"
		* start receiving continous sensor_msgs/LaserScan.

	3. "/vl53mkac/stop_laser_srv"
		* stop receiving continous sensor_msgs/LaserScan.
		* example: rosservice call /vl53mkac/stop_laser_srv
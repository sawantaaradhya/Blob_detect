THIS IS THE DOCUMENTARY FOR THIS PACKAGE

1. HOW TO USE THIS PACKAGE?

	Pre-requisites: OpenCV and ROS must be installed in the system.
	STEPS:
		1. Open four terminals in the directory where this package is extracted.
		2. In first terminal, input the following commands:
			1. source /opt/ros/kinetic/setup.bash
		`	2. catkin_make
			3. source devel/setup.bash
			4. roscore
		
		3. In second terminal, input the following commands:
			1. source /opt/ros/kinetic/setup.bash
			2. source devel/setup.bash
			3. rosrun image_transport_tutorial my_publisher 0

		4. In third terminal, input the following commands:
			1. source /opt/ros/kinetic/setup.bash
			2. source devel/setup.bash
			3. rosrun image_transport_tutorial my_subscriber

		5. In fourth terminal, input the following commands:
			1. source /opt/ros/kinetic/setup.bash
			2. source devel/setup.bash
			3. rosrun image_transport_tutorial my_subscriber2 





2. How it works?
	
	1. src/image_transport_tutorial/src/my_publisher.cpp 
		
		This file publishes video feed from webcam on a topic named "camera/image".
		Used libraries(apart from OpenCV) : image_transport, cv_bridge
		
		References: 
			http://wiki.ros.org/image_transport
			http://wiki.ros.org/cv_bridge
			
		I have used http://wiki.ros.org/image_transport/Tutorials/PublishingImages as a major part of my code.
		ERROR: Whenever I use VideoCapture Cap("Video_location") instead of VideoCapture Cap(0), it doesn't work. 

	2. src/image_transport_tutorial/src/my_subscriber.c
		
		This file subscribes to the topic "camera/image" and thereby fetches the published image.
		The image is then worked upon to fetch the center of mass and radius and publish it on the topic "position".
		Steps:
			1. Gaussian filter
			2. Conversion from BGR to HSV for thresholding.
			3. Thresholding points having 0<=H<=15, 100<=H<=179, S<=40 and V<=20.
			4. Finding the largest contour and fitting a circle around it which has smallest possible radius(minEnclosingCircle). This function returns the center and radius.
			5. This center and radius is then published on the topic "position".
			
		FOR BETTER RESULTS, ADAPTIVE THRESHOLDING COULD BE USED, WHICH I MAY IMPLEMENT IN THE FUTURE.
		Used libraries : geometry_msgs::Point.h used for publishing 3 parameters(center.x, center.y, radius in our case).
		References: 
			http://wiki.ros.org/geometry_msgs
		
	
	3. src/image_transport_tutorial/src/my_subscriber2.c

		This file subscribes to the topic "position", applies the Kalman Filter on the received COM and radius, and publishes the measured COM/radius on the topic. 
		Note that Noise Covariance Matrix is the property of sensor itself. For correct setup, we need to observe the error sensor does when we already know its position accurately.
		To apply Kalman filter, we need to have a correct understanding of how it works.
		References : Youtube tutorials(see as many as you can, then you will get it).
		
  

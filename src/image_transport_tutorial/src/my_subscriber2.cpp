#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
using namespace cv;
using namespace std;

Point2f center;
float radius=0;

void Callback(const geometry_msgs::Point::ConstPtr& msg)
{
  center.x=msg->x;
  center.y=msg->y;
  radius=msg->z;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "position_listener");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Point>("kposition",1000);
	//ros::Publisher pub = nh.advertise<geometry_msgs::Point>("kposition",1000);
	ros::Subscriber sub = nh.subscribe("position", 1000, Callback);
        unsigned int type = CV_32F;
    	KalmanFilter KF(5, 3, 0);//x,y,vx,vy,r;z_x,z_y,r
    	Mat state(5, 1, type);
    	Mat meas(3, 1, type);
  

    //  KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 ]
    // [ 0 1 0  dT 0 ]
    // [ 0 0 1  0  0 ]
    // [ 0 0 0  1  0 ]
    // [ 0 0 0  0  1 ]
   	cv::setIdentity(KF.transitionMatrix);
    

    // Measure Matrix H
    // [ 1 0 0 0 0 ]
    // [ 0 1 0 0 0 ]
    // [ 0 0 0 0 1 ]
	KF.measurementMatrix = cv::Mat::zeros(3, 5, type);
	KF.measurementMatrix.at<float>(0) = 1.0f;
	KF.measurementMatrix.at<float>(6) = 1.0f;
	KF.measurementMatrix.at<float>(14) = 1.0f;
    

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    ]
    // [ 0    Ey  0     0     0    ]
    // [ 0    0   Evx   0     0    ]
    // [ 0    0   0     Evy   0    ]
    // [ 0    0   0     0     Er   ]
       KF.processNoiseCov.at<float>(0) = 1e-2;
       KF.processNoiseCov.at<float>(6) = 1e-2;
       KF.processNoiseCov.at<float>(12) = 5.0f;
       KF.processNoiseCov.at<float>(18) = 5.0f;
       KF.processNoiseCov.at<float>(24) = 1e-2;

       // Measures Noise Covariance Matrix R
       cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(1e-1));

	double ticks =0;
	int notFoundCount=0;
	bool found=false;
	ros::Rate rate(10);
	while(ros::ok())
	{
		cout<<"Received center "<<center;
		cout<<" Recieved radius: "<<radius<<endl;
		double precTick = ticks;
                ticks = (double) cv::getTickCount();

                double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

                KF.transitionMatrix.at<float>(2) = dT;
                KF.transitionMatrix.at<float>(8) = dT;
		state = KF.predict();
      		//cout << "State post:" << endl << state << endl;
      		Point centerS;
      		float radiusS;
      		centerS.x = state.at<float>(0);
      		centerS.y = state.at<float>(1);
      		radiusS=state.at<float>(4);
		if (center.x==0 || center.y==0 || radius==0  )
          	{
            		notFoundCount++;
            		cout << "notFoundCount:" << notFoundCount << endl;
            		if( notFoundCount >= 100 )
            		{
               			found = false;
            		}
           	}
	        else
          	{
            		notFoundCount = 0;

            		meas.at<float>(0) = center.x;
            		meas.at<float>(1) = center.y;
            		meas.at<float>(2) = radius;
		

            		if (!found) // First detection!
            		{
            			// >>>> Initialization
                		KF.errorCovPre.at<float>(0) = 1;
                		KF.errorCovPre.at<float>(6) = 1;
                		KF.errorCovPre.at<float>(12) = 1;
                		KF.errorCovPre.at<float>(18) = 1;
                		KF.errorCovPre.at<float>(24) = 1;

                		state.at<float>(0) = meas.at<float>(0);
                		state.at<float>(1) = meas.at<float>(1);
                		state.at<float>(2) = 0;
                		state.at<float>(3) = 0;
                		state.at<float>(4) = meas.at<float>(2);
                		// <<<< Initialization

                		KF.statePost = state;

                		found = true;
            		}
            		else KF.correct(meas); // Kalman Correction
            		cout << "k_center [" <<meas.at<float>(0)<<", "<<meas.at<float>(1)<<"]";
			cout << " k_radius: "<<meas.at<float>(2)<<endl;
		}
		geometry_msgs::Point msg;
   		msg.x = meas.at<float>(0);     
		msg.y = meas.at<float>(1); 
		msg.z=meas.at<float>(2);
		pub.publish(msg);
		
		ros::spinOnce();
		rate.sleep();
	}
       	return 0;
}































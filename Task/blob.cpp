#include "opencv2/opencv.hpp"
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/video/video.hpp>
#include <iostream>
//#include <vector>
using namespace cv;
using namespace std;



/*


  Mat cntr_rds(Mat frame,bool found, Point2f centerS,float radiusS)
  {
  Mat res;
  frame.copyTo(res);
  circle(res, centerS, radiusS,CV_RGB(0,255), -1);
  Mat res;
  frame.copyTo(res);

  Mat blur;
  GaussianBlur(frame, blur,Size(5,5),3,3);
  Mat frmHsv;
  cvtColor(blur, frmHsv, CV_BGR2HSV);
  Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
  Mat lower_red_hue_range;
  Mat upper_red_hue_range;
  inRange(frmHsv, cv::Scalar(0, 0, 0), cv::Scalar(30, 255, 255), lower_red_hue_range);
  inRange(frmHsv, cv::Scalar(160, 0, 0), cv::Scalar(179, 255, 255), upper_red_hue_range);
  //Mat red_hue_image;
  addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, rangeRes);
  cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours(rangeRes, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  int largest_contour_index;
  if(contours.size()!=0)
    {
      vector<vector<Point> > contours_poly( contours.size() );
      //cout<<contours.size()<<endl;
      int i;
      float largest_area=100;
      for(int i = 0; i< contours.size(); i++ ) // iterate through each contour.
        {
          double a=contourArea( contours[i],false);  //  Find the area of contour
          if(a>largest_area)
          {
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
          }
        }
      }
      Point2f center;
      float radius;
      vector<vector<cv::Point> > balls;
      if (contourArea(contours[largest_contour_index], false)>10)
      {
        minEnclosingCircle( (Mat)contours[largest_contour_index], center, radius);
        balls.push_back(contours[largest_contour_index]);
      }
      //cout<<"HERE"<<endl;
      circle( res, center, radius, Scalar(100,0,0), 10, 8, 0 );
      cout<<"Sensor center Value:"<<center;
      cout<<"Sensor radius value:"<<radius;




*/


int main(int, char**)
{
  #define drawCross( center, color, d )                                 \
    line(frame2, Point(center.x - d, center.y - d), Point(center.x + d, center.y + d), color, 2, CV_AA, 0); \
    line(frame2, Point(center.x + d, center.y - d), Point(center.x - d, center.y + d), color, 2, CV_AA, 0)

    Mat frame;
    unsigned int type = CV_32F;
    KalmanFilter KF(6, 4, 0);//x,y,vx,vy,r,r;z_x,z_y,r,r
    Mat state(6, 1, type);
    Mat meas(4, 1, type);
    //  KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(KF.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    KF.measurementMatrix = cv::Mat::zeros(4, 6, type);
    KF.measurementMatrix.at<float>(0) = 1.0f;
    KF.measurementMatrix.at<float>(7) = 1.0f;
    KF.measurementMatrix.at<float>(16) = 1.0f;
    KF.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Evx  0     0    0  ]
    // [ 0    0   0     Evy  0    0  ]
    // [ 0    0   0     0     Er   0  ]
    // [ 0    0   0     0     0    Er ]
    //cv::setIdentity(KF.processNoiseCov, cv::Scalar(1e-2));
    KF.processNoiseCov.at<float>(0) = 1e-2;
    KF.processNoiseCov.at<float>(7) = 1e-2;
    KF.processNoiseCov.at<float>(14) = 5.0f;
    KF.processNoiseCov.at<float>(21) = 5.0f;
    KF.processNoiseCov.at<float>(28) = 1e-2;
    KF.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(1e-1));
    VideoCapture cap("bouyvideo_ml.avi"); // open the file
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    //setIdentity(KF.errorCovPost, Scalar::all(.1));

    cout << "\nHit 'q' to exit...\n";
    char ch=0;
    double ticks =0;
    bool found=false;
    int notFoundCount=0;

    while(ch!='q' && ch!='Q')
    {
      double precTick = ticks;
      ticks = (double) cv::getTickCount();

      double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

      // Frame acquisition
      // >>>> Matrix A
      KF.transitionMatrix.at<float>(2) = dT;
      KF.transitionMatrix.at<float>(9) = dT;
      state = KF.predict();
      cout << "State post:" << endl << state << endl;
      Point centerS;
      float radiusS;
      centerS.x = state.at<float>(0);
      centerS.y = state.at<float>(1);
      radiusS=state.at<float>(5);


      cap >> frame;
      Mat res;
      frame.copyTo(res);

      Mat blur;
      GaussianBlur(frame, blur,Size(5,5),3,3);
      Mat frmHsv;
      cvtColor(blur, frmHsv, CV_BGR2HSV);
      Mat red(frame.rows,frame.cols,CV_8UC1,Scalar(0));
      // for(int i=0;i<red.rows;i++)
      //   for(int j=0;j<red.cols;j++)
      //   {
      //     red.at<uchar>(i,j)=(int)(3*res.at<Vec3b>(i,j)[2]-(2*res.at<Vec3b>(i,j)[1]+res.at<Vec3b>(i,j)[0]));
      //   }
      //   imshow("color conversion",red);
      Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
      Mat lower_red_hue_range;
			Mat upper_red_hue_range;
      int x=20;
      int y=20;
      int lhue=15;
      int hhue=100;
			inRange(frmHsv, cv::Scalar(0, x, y), cv::Scalar(lhue, 255, 255), lower_red_hue_range);
			inRange(frmHsv, cv::Scalar(hhue, x,y), cv::Scalar(179, 255, 255), upper_red_hue_range);
			//Mat red_hue_image;
			addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, rangeRes);
      cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours(rangeRes, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      int largest_contour_index=-1;
      double a=0;
      float largest_area=-1;
      if(contours.size()!=0)
        {
          //vector<vector<Point> > contours_poly( contours.size() );
          //cout<<contours.size()<<endl;
          int i;
          for(int i = 0; i< contours.size(); i++ ) // iterate through each contour.
            {
              if(1) a=contourArea( contours[i],false);  //  Find the area of contour
              if(a>largest_area)
              {
                largest_area=a;
                largest_contour_index=i;                //Store the index of largest contour
              }
            }
            if(largest_area<1) largest_contour_index=-1;
        }

          Point2f center;
          float radius=0;
          vector<vector<cv::Point> > balls;
          //cout<<largest_contour_index;

          if (largest_contour_index!=-1)
          {
            minEnclosingCircle( (Mat)contours[largest_contour_index], center, radius);
            balls.push_back(contours[largest_contour_index]);
          }
          //cout<<"HERE"<<endl;
          circle( res, center, radius, Scalar(0,0,255), 10, 8, 0 );
          cout<<"Sensor center Value:"<<center;
          cout<<"Sensor radius value:"<<radius;
          if (found)
            {

               cout << "dT:" << endl << dT << endl;



               circle(res, centerS, radiusS,Scalar(255,0,0),3,8,0);
               //This is the state

            }
          // <<<< Matrix A

          if (balls.size() == 0)
          {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                KF.statePost = state;*/
          }
          else
          {
            notFoundCount = 0;

            meas.at<float>(0) = center.x;
            meas.at<float>(1) = center.y;
            meas.at<float>(2) = radius;
            meas.at<float>(3) = radius;

            if (!found) // First detection!
            {
                // >>>> Initialization
                KF.errorCovPre.at<float>(0) = 1; // px
                KF.errorCovPre.at<float>(7) = 1; // px
                KF.errorCovPre.at<float>(14) = 1;
                KF.errorCovPre.at<float>(21) = 1;
                KF.errorCovPre.at<float>(28) = 1; // px
                KF.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization

                KF.statePost = state;

                found = true;
            }
            else
                KF.correct(meas); // Kalman Correction

            cout << "Measure matrix:" << endl << meas << endl;
          }
          cv::imshow("Tracking", res);
          //imshow("See",frmHsv);
          ch=cv::waitKey(10);
        }
        return EXIT_SUCCESS;
      }


        /*
        Mat frame;
        cap >> frame;
				GaussianBlur(, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT );
				Mat hsv_image;
				cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);
				Mat lower_red_hue_range;
				Mat upper_red_hue_range;
				inRange(hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(30, 255, 255), lower_red_hue_range);
				inRange(hsv_image, cv::Scalar(160, 0, 0), cv::Scalar(179, 255, 255), upper_red_hue_range);
				Mat red_hue_image;
				addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
				GaussianBlur(red_hue_image, red_hue_image, cv::Size(4, 4), 2, 2);
        imshow("Image", red_hue_image);
	      imshow("Original",hsv_image);
	      imshow("Original",frame);
        */
        /*
        Mat frame;
        cap >> frame;
        Mat frame2;
        GaussianBlur(frame, frame2, cv::Size(9, 9), 2, 2);
        Mat hsv_image;
        cvtColor(frame2, hsv_image, cv::COLOR_BGR2HSV);
        Mat lower_red_hue_range;
				Mat upper_red_hue_range;
        inRange(hsv_image, cv::Scalar(0, 30, 30), cv::Scalar(15, 255, 255), lower_red_hue_range);
				inRange(hsv_image, cv::Scalar(160, 30, 30), cv::Scalar(179, 255, 255), upper_red_hue_range);
        Mat red_hue_image;
        addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
        /*
        vector<Vec3f> circles;
        HoughCircles( red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/16, 200, 100, 0,  red_hue_image.rows/1.5);
        cout<<circles.size();
        cout<<"Here"<<endl;
        //Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( frame, center, 15, Scalar(255,0,0), 10, 8, 0 );
          // circle outline
          circle( frame, center, radius, Scalar(255,0,0), 10, 8, 0 );
        }
        imshow("Original2",hsv_image);
        imshow("Original",frame2);
        imshow("Image", red_hue_image);
        waitKey(0);
        */
        /*
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Point2f center;
        float radius;

        findContours( red_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        if(contours.size()!=0)
        {
          vector<vector<Point> > contours_poly( contours.size() );
          //cout<<contours.size()<<endl;
          int i;
          float largest_area=50;
          int largest_contour_index;
          for(int i = 0; i< contours.size(); i++ ) // iterate through each contour.
          {
            double a=contourArea( contours[i],false);  //  Find the area of contour
            if(a>largest_area)
            {
              largest_area=a;
              largest_contour_index=i;                //Store the index of largest contour
            }
          }
          //c = max(contours, key = cv2.contourArea);
          for( int j = 0; j < contours.size(); j++ )
          {
            approxPolyDP( Mat(contours[j]), contours_poly[j], 1, true );
          }

          if (contourArea(contours_poly[largest_contour_index], false)>10) minEnclosingCircle( (Mat)contours_poly[largest_contour_index], center, radius);
          //cout<<"HERE"<<endl;
          circle( frame2, center, radius, Scalar(100,0,0), 10, 8, 0 );
          	char key = 0;
            KF.statePre.at<float>(0) = center.x;
            KF.statePre.at<float>(1) = center.y;
            KF.statePre.at<float>(2) = 0;
            KF.statePre.at<float>(3) = 0;
            Mat prediction = KF.predict();
        		Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
            measurement(0) = center.x;
        		measurement(1) = center.y;
            Point measPt(measurement(0), measurement(1));
            Mat estimated = KF.correct(measurement);
            Point statePt(estimated.at<float>(0), estimated.at<float>(1));
            drawCross(statePt, Scalar(255, 255, 255), 5);
        		drawCross(measPt, Scalar(0, 0, 255), 5);
            cout<<statePt<<measPt<<endl;


        }
          imshow("Original2",red_hue_image);
          imshow("Original",frame2);
        waitKey(1000);
        */


    // the camera will be deinitialized automatically in VideoCapture destructor

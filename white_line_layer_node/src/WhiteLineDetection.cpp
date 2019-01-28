#include "WhiteLineDetection.h"
//#include "ros/ros.h"
bool DetectWhiteLines::initCamera()
{
   initParameters.camera_resolution = RESOLUTION_HD720;
   initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
   initParameters.coordinate_system = COORDINATE_SYSTEM_IMAGE;
   initParameters.coordinate_units = UNIT_METER;

   ERROR_CODE err = zed.open(initParameters);
   if(err != SUCCESS)
   {
     cerr << err << "\n";
     zed.close();
     return false;
   }
   //cout << "CAMERA INITIALIZED\n";
   return true;
}

bool DetectWhiteLines::loadPointCloud()
{
  //cout << "LOADING POINT CLOUD\n";
  ERROR_CODE erg = zed.grab();
  if(erg == SUCCESS)
  {
    zed.retrieveMeasure(point_cloud,MEASURE_XYZRGBA);
    return true;
  }
  return false;
}


double DetectWhiteLines::findMinX()
{
  sl::float4 currPoint;
  double minX = MAX_X_VALUE;
   for(int i = 0; i < HEIGHT; ++i)
   {
	point_cloud.getValue(i,0,&currPoint);
	if(isValidPoint(currPoint.x,true))	
	   minX = currPoint.x;
   }
   if(minX < 0) minX *= -1;
   return minX;
}


bool DetectWhiteLines::isValidPoint(float & currVal, bool isX)
{
  if(isX)
    return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_X_VALUE \
	    && currVal > MIN_X_VALUE);
  else
    return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_Z_VALUE && \
	    currVal > MIN_Z_VALUE);  
}


 void DetectWhiteLines::convertXZ()
 {
   //cout << boolalpha;
   //cout << "CONVERTING\n";
   sl::float4 currPoint;
   for(size_t i = 0; i < HEIGHT; ++i)
   {
      for(size_t j = 0; j < WIDTH; ++j)
      {
          point_cloud.getValue(i,j,&currPoint);
          size_t x,z;
          int print = 0;
          if(isValidPoint(currPoint.x,true))
          {
	     x = static_cast<int>((currPoint.x + SHIFTVAL ) / XDIVISOR);
	     print++;
          }
          if(isValidPoint(currPoint.z,false))
          {
	     z = static_cast<int>(currPoint.z / ZDIVISOR);
	     print++;
          }

	  if(print == 2)
	  {
	    //cout << x << " " << z << endl; 
	     Rgba color = unpack_float(currPoint.w);
	     uchar r = uchar(color.r);
	     uchar b = uchar(color.b);
	     uchar g = uchar(color.g);

	     cv::Vec3b val = xzMat.at<cv::Vec3b>(cv::Point(x,z));
	     val[0] = b;
	     val[1] = g;
	     val[2] = r;
	     xzMat.at<cv::Vec3b>(cv::Point(x,z)) = {b,g,r};
	  }
	  print = 0;
      }
   }
 }


void DetectWhiteLines::whiteLineDetection()
{
      //whiteLineDetect = xzMat;
      cv::Size s = xzMat.size();
      outputImage = cv::Mat(s, CV_8UC3, cv::Scalar(0,0,0));
      cv::cvtColor(xzMat,xzMat,CV_BGR2GRAY);
      //cv::resize(whiteLineDetect,whiteLineDetect,cv::Size(134,200),cv::INTER_LANCZOS4);
      cv::threshold(xzMat,xzMat,170,255,cv::THRESH_BINARY);
      vector<cv::Vec4i> lines;
      cv::HoughLinesP(xzMat,lines,1,CV_PI/180,20,10,1);
      for( const auto& i : lines)
      {
        line(outputImage,cv::Point(i[0],i[1]),cv::Point(i[2],i[3]),cv::Scalar(255,255,255),2);
      }
      //cv::resize(outputImage,outputImage,s,cv::INTER_LANCZOS4);
}

void DetectWhiteLines::imuTransform(const std_msgs::String::ConstPtr& msg)
{
  
  //cout << "IMU GOES HERE\n";
}

DetectWhiteLines::DetectWhiteLines( const DetectWhiteLines & other) : it{other.node}
{
  pub = other.pub;
  xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
  outputImage = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
}

void DetectWhiteLines::detect(const ros::TimerEvent&)
{
  //ROS_DEBUG_STREAM("WORKING");
  bool test = initCamera();
  if(test)
  {
    bool value = loadPointCloud();
    if(value)
    {
      double val = findMinX();
      cout << val << endl;
      convertXZ();
      displayXZ(10);
    
      whiteLineDetection();
      displayWL(10);
    
      
       sensor_msgs::ImagePtr im = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
       pub.publish(im);
    
      zed.close();
      clearXZ();
      //ROS_DEBUG_STREAM(" -------------------------");
    }
  }
}

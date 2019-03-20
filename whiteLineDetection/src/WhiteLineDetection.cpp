#include "WhiteLineDetection.h"
#include <tf2/convert.h>
//#include "ros/ros.h"
bool DetectWhiteLines::initCamera()
{
   initParameters.camera_resolution = RESOLUTION_HD720;
   initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
   initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD; 
   initParameters.coordinate_units = UNIT_METER;

   ERROR_CODE err = zed.open(initParameters);
   if(err != SUCCESS)
   {
     cerr << err << "\n";
     zed.close();
     return false;
   }
   return true;
}

bool DetectWhiteLines::loadPointCloud()
{
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


bool DetectWhiteLines::isValidPoint(float currVal, bool isX)
{
  //true means X-coord, false means Z-coord
  if(isX)
    return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_X_VALUE \
	    && currVal > MIN_X_VALUE);
  else
    return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_Z_VALUE && \
	    currVal > MIN_Z_VALUE);  
}


void DetectWhiteLines::convertXZ()
{
   sl::float4 currPoint;
   tf2::Vector3  xyz;
   for(size_t i = 0; i < HEIGHT; ++i)
   {
      for(size_t j = 0; j < WIDTH; ++j)
      {
          point_cloud.getValue(i,j,&currPoint);
	  xyz[0] = currPoint.x;
	  xyz[1] = currPoint.y;
	  xyz[2] = currPoint.z;
	  tf2::Transform transform(quat);
	  //cout << xyz[0] << " " << xyz[1] << " " << xyz[2] << endl;
      
	  
	  tf2::Stamped<tf2::Transform> bodyTransform;
	  tf2::fromMsg(bodyFrame,bodyTransform);
	  //xyz = bodyTransform * xyz;

	  xyz = transform * xyz;
	  
          int print = 0;
          if(isValidPoint(xyz[1],true))
          {
	     xyz[1] = static_cast<int>((xyz[1] + SHIFTVAL ) / XDIVISOR);
	     print++;
          }
          if(isValidPoint(xyz[0],false))
          {
	     xyz[0] = static_cast<int>((xyz[0]) / ZDIVISOR);
	     print++;
          }

	  xyz[2] = 0;
	  
	  if(print == 2)
	  {
	     Rgba color = unpack_float(currPoint.w);
	     uchar r = uchar(color.r);
	     uchar b = uchar(color.b);
	     uchar g = uchar(color.g);

	     cv::Vec3b val = xzMat.at<cv::Vec3b>(cv::Point(xyz[1],xyz[0]));
	     val[0] = b;
	     val[1] = g;
	     val[2] = r;
	     xzMat.at<cv::Vec3b>(cv::Point(xyz[1],xyz[0])) = {b,g,r};
	  }
	  print = 0;
      }
   }
 }


void DetectWhiteLines::whiteLineDetection()
{
      cv::Size s = xzMat.size();
      outputImage = cv::Mat(s, CV_8UC3, cv::Scalar(0,0,0));
      cv::cvtColor(xzMat,xzMat,CV_BGR2GRAY);
      //cv::resize(xzMat,xzMat,cv::Size(134,200),cv::INTER_LANCZOS4);
      cv::threshold(xzMat,xzMat,170,255,cv::THRESH_BINARY);
      vector<cv::Vec4i> lines;
      cv::HoughLinesP(xzMat,lines,1,CV_PI/180,20,10,1);
      for( const auto& i : lines)
      {
        line(outputImage,cv::Point(i[0],i[1]),cv::Point(i[2],i[3]),cv::Scalar(111),2);
      }
      //cv::resize(outputImage,outputImage,s,cv::INTER_LANCZOS4);
}

void DetectWhiteLines::imuTransform(const sensor_msgs::ImuConstPtr &imu)
{
  tf2Scalar arr[4] = {imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w};
  quat = tf2::Quaternion(arr[0],arr[2],arr[2],arr[3]);
  
  try{
     bodyFrame = buffer->lookupTransform("zed_center","imu",ros::Time(0));
  }
  catch(tf2::TransformException &ex){
     ROS_WARN("%s",ex.what());
  }
  
}

DetectWhiteLines::DetectWhiteLines(const DetectWhiteLines & other )
{
  xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
  outputImage = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0));
}

void DetectWhiteLines::publish()
{
    nav_msgs::OccupancyGrid occ;
    occ.header.seq = seqId++;
    occ.header.frame_id = "test";
    occ.header.stamp = ros::Time().now();
    //MetaData
    occ.info.map_load_time = ros::Time().now();
    occ.info.resolution = XDIVISOR;
    occ.info.width = WIDTH;
    occ.info.height = HEIGHT;
    //occ.info.origin = geometry_msgs::Pose(p, q);
    occ.data.resize(xzMat.total());
    memcpy(&(occ.data.front()), outputImage.data, xzMat.elemSize() * xzMat.total());
    occ_pub.publish(occ);
    
}


void DetectWhiteLines::detect(const ros::TimerEvent&)
{
  bool test = initCamera();
  if(test)
  {
    bool value = loadPointCloud();
    if(value)
    {
      convertXZ();
      displayXZ(7);
    
      whiteLineDetection();
      displayWL(7);

      publish();
      
    
      zed.close();
      clearXZ();
    }
  }
}

#include "WhiteLineDetection.hpp"
#include <tf2/convert.h>

#include <simt_tf/simt_tf.h>


#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/cudaimgproc.hpp"



//SWITCH FROM BGR TO BGRA
/*
        const int64 start = getTickCount();
        //process
        const double timeSec = (getTickCount() - start) / getTickFrequency();
        cout << "CPU Time : " << timeSec * 1000 << " ms" << endl;
*/

using namespace cv;

bool DetectWhiteLines::initCamera()
{
	initParameters.camera_resolution = sl::RESOLUTION_HD720;
	initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	initParameters.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
	initParameters.coordinate_units = sl::UNIT_METER;

	sl::ERROR_CODE err = zed.open(initParameters);
	if(err != sl::SUCCESS)
	{
		cerr << "Opening camera error: " << err << "\n";
		zed.close();
		return false;
	}
	return true;
}

bool DetectWhiteLines::loadPointCloud()
{
	sl::ERROR_CODE erg = zed.grab();
	if(erg == sl::SUCCESS)
	{
		zed.retrieveMeasure(point_cloud,sl::MEASURE_XYZRGBA, sl::MEM_GPU);
		return true;
	}
	return false;
}



bool DetectWhiteLines::isValidPoint(float currVal, bool isX)
{
	if(isX)
		return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_Y_VALUE \
			&& currVal > MIN_Y_VALUE);
	else
		return (!isnan(currVal) && !isinf(currVal) && currVal < MAX_X_VALUE && \
			currVal > MIN_X_VALUE);  
}


void DetectWhiteLines::convertXZ()
{
	valid = false;
	sl::float4 currPoint;
	tf2::Vector3 xyz;
	for(size_t i = 0; i < HEIGHT; i++)
	{
		for(size_t j = 0; j < WIDTH; j++)
		{
			point_cloud.getValue(i,j,&currPoint);
			xyz[0] = currPoint.x;
			xyz[1] = currPoint.y;
			xyz[2] = currPoint.z;
			/*
			tf2::Transform transform(imuQuat);
         
			tf2::Stamped<tf2::Transform> bodyTransform;
			tf2::fromMsg(bodyFrame,bodyTransform);

			xyz = transform * xyz;
 
	  
			newXYZ = bodyTransform * newXYZ;
*/
			tf2::Vector3 newXYZ = {xyz[2],xyz[0],0};
			
			if(isValidPoint(newXYZ[1],Y))
			{
				newXYZ[1] = static_cast<int>((newXYZ[1] + SHIFTVAL ) / YDIVISOR);
			}
			else
			{
				continue;
			}
			
			if(isValidPoint(newXYZ[0],X))
			{
				 newXYZ[0] = static_cast<int>((newXYZ[0]) / XDIVISOR);
			}
			else
			{
				continue;
			}
	  
			Rgba color = unpack_float(currPoint.w);
			uchar r = uchar(color.r);
			uchar b = uchar(color.b);
			uchar g = uchar(color.g);

			cv::Vec3b val = xzMat.at<cv::Vec3b>(cv::Point(newXYZ[1],newXYZ[0]));
			xzMat.at<cv::Vec3b>(cv::Point(newXYZ[1],newXYZ[0])) = val;
		}
	}
	valid = true;
 }


void DetectWhiteLines::whiteLineDetection()
{
      cv::Size s = xzMat.size();
      outputImage = cv::Mat(s, CV_8UC3, cv::Scalar(0,0,0));
      cv::cvtColor(xzMat,xzMat,CV_BGR2GRAY);
      //cv::resize(xzMat,xzMat,cv::Size(134,200),cv::INTER_LANCZOS4);
      cv::threshold(xzMat,xzMat,170,255,cv::THRESH_BINARY);
      vector<cv::Vec4i> lines;
      
      
      //cv::Ptr<cv::cuda::HoughSegmentDetector> hough = cv::cuda::createHoughSegmentDetector(1.0f, (float) (CV_PI / 180.0f), 20, 10,1);
      //hough->detect(xzMat, lines);
       
      cv::HoughLinesP(xzMat,lines,1,CV_PI/180,20,10,1);
      for( const auto& i : lines)
      {
        line(outputImage,cv::Point(i[0],i[1]),cv::Point(i[2],i[3]),cv::Scalar(111),2);
      }
      //cv::resize(outputImage,outputImage,s,cv::INTER_LANCZOS4);
}

void DetectWhiteLines::imuTransform(const sensor_msgs::ImuConstPtr &imu)
{
	if(valid)
	{
		tf2Scalar arr[4] = {imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w};
		imuQuat = tf2::Quaternion(arr[0],arr[1],arr[2],arr[3]);
	}

	try{
		bodyFrame = buffer->lookupTransform("zed_center","base_link",ros::Time(0));
	} catch(tf2::TransformException &ex){
		ROS_WARN("%s",ex.what());
	}
  
}

DetectWhiteLines::DetectWhiteLines(const DetectWhiteLines & other )
{
	xzMat = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
	outputImage = cv::Mat(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(0));
	initCamera();
}

void DetectWhiteLines::publish()
{
  /*
	VERY CONFUSED HOW TO SET THIS UP
   */
	sensor_msgs::PointCloud2 occ;
	occ.height = HEIGHT;
	occ.width = WIDTH;
	
	//occ.fields[];
	
	//occ.point_step = ;
	//occ.row_step = ;
	//occ.data =;
	occ.is_dense = true;
	
	occ_pub.publish(occ);

}


void DetectWhiteLines::detect(const ros::TimerEvent&)
{
	const int64 start = getTickCount();
    //process
    
	bool value = loadPointCloud();
	if(value)
	{	
		convertXZGPU();
		displayXZ(2);
		const double timeSec = (getTickCount() - start) / getTickFrequency();
		cout << "CPU Time : " << timeSec * 1000 << " ms" << endl;

		whiteLineDetection();
		displayWL(2);
		

		publish();

		clearXZ();
	}
	
}



void DetectWhiteLines::convertXZGPU()
{
	tf2::Transform transform(imuQuat);
         
	tf2::Stamped<tf2::Transform> bodyTransform;
	tf2::fromMsg(bodyFrame,bodyTransform);
	
	transform *= bodyTransform;
	
	const simt_tf::Transform host_tf = {
		{
			float(transform.getBasis()[0][0]), float(transform.getBasis()[0][1]), float(transform.getBasis()[0][2]),
			float(transform.getBasis()[1][0]), float(transform.getBasis()[1][1]), float(transform.getBasis()[1][2]),
			float(transform.getBasis()[2][0]), float(transform.getBasis()[2][1]), float(transform.getBasis()[2][2])
		},	
		{ float(transform.getOrigin()[0]),float(transform.getOrigin()[1]),float(transform.getOrigin()[2]) }
	};

    cv::cuda::GpuMat output(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 255));

	simt_tf::pointcloud_birdseye(host_tf, zed, point_cloud,output, 0.01);

    xzMat = cv::Mat(output);
}




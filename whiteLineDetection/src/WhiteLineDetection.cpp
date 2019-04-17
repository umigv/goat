#include "WhiteLineDetection.hpp"
#include <tf2/convert.h>

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
  */
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


template <typename T>
using CudaVector = std::vector<T, CudaAllocator<T>>;

template <typename T, std::enable_if_t<!std::is_array<T>::value, int> = 0>
std::unique_ptr<T, CudaDeleter> to_device(const T &host) {
    static_assert(std::is_trivially_copyable<T>::value, "");

    T *device_ptr;

    const std::error_code malloc_ec = cudaMalloc(&device_ptr, sizeof(T));

    if (malloc_ec) {
        throw std::bad_alloc();
    }

    const std::error_code memcpy_ec = cudaMemcpy(device_ptr, std::addressof(host), sizeof(T), cudaMemcpyHostToDevice);

    if (memcpy_ec) {
        throw std::system_error(memcpy_ec);
    }

    return {device_ptr, CudaDeleter()};
}

template <typename T>
CudaVector<T> to_device(const T host[], std::size_t n) {
    static_assert(
        std::is_trivially_default_constructible<T>::value && std::is_trivially_copyable<T>::value,
        "T must be trivially default constructible and copyable"
    );

    CudaVector<T> device(n);

    const std::error_code ec = cudaMemcpy(
        device.data(),
        host,
        n * sizeof(T),
        cudaMemcpyHostToDevice
    );

    if (ec) {
        throw std::system_error(ec);
    }

    return device;
}

template <typename T, typename A>
CudaVector<T> to_device(const std::vector<T, A> &host) {
    return to_device(host.data(), host.size());
}

template <typename T, std::size_t N>
CudaVector<T> to_device(const std::array<T, N> &host) {
    return to_device(host.data(), N);
}

template <typename T, std::size_t N>
CudaVector<T> to_device(const T (&host)[N]) {
    return to_device(host, N);
}

template <typename T, typename A = std::allocator<T>>
std::vector<T, A> to_host(const std::vector<T, CudaAllocator<T>> &device) {
    std::vector<T, A> host(device.size());

    const std::error_code ec = cudaMemcpy(
        host.data(),
        device.data(),
        device.size() * sizeof(T),
        cudaMemcpyDeviceToHost
    );

    if (ec) {
        throw std::system_error(ec);
    }

    return host;
}

template <typename T>
T to_host(const std::unique_ptr<T, CudaDeleter> &ptr) {
    T host;

    const std::error_code ec = cudaMemcpy(
        std::addressof(host),
        std::addressof(*ptr),
        sizeof(T),
        cudaMemcpyDeviceToHost
    );

    if (ec) {
        throw std::system_error(ec);
    }

    return host;
}

constexpr std::size_t div_to_inf(std::size_t x, std::size_t y) noexcept {
    const std::size_t res = x / y;

    if (x % y != 0) {
        return res + 1;
    }

    return res;
}

__host__ __device__ sl::uchar4 from_packed(float x) {
    union Converter {
        float scalar;
        sl::uchar4 vector;
    };

    return Converter{x}.vector;
}

__host__ __device__ float pack(sl::uchar4 x) noexcept {
    union Converter {
        sl::uchar4 vector;
        float scalar;
    };

    return Converter{x}.scalar;
}

sl::Mat random_xyzrgba(std::size_t width, std::size_t height) {
    const std::size_t numel = width * height;
    sl::Mat m(width, height, sl::MAT_TYPE_32F_C4);

    const auto gen_ptr = std::make_unique<std::mt19937>();
    std::uniform_real_distribution<float> pos_dist(-10, 10);
    std::uniform_int_distribution<std::uint8_t> color_dist;

    const auto gen_pos = [&gen_ptr, &pos_dist] {
        return pos_dist(*gen_ptr);
    };

    const auto gen_col = [&gen_ptr, &color_dist] {
        return color_dist(*gen_ptr);
    };

    const auto arr = m.getPtr<sl::float4>();
    for (std::size_t i = 0; i < numel; ++i) {
        arr[i][0] = gen_pos();
        arr[i][1] = gen_pos();
        arr[i][2] = gen_pos();
        arr[i][3] = pack({255, 255, 255, 127});
    }

    m.updateGPUfromCPU();

    return m;
}

__host__ __device__ std::uint32_t pack_bgra(
    std::uint8_t b, std::uint8_t g,
    std::uint8_t r, std::uint8_t a
)  noexcept {
    union Converter {
        std::uint8_t arr[4];
        std::uint32_t scalar;
    };

    return Converter{{b, g, r, a}}.scalar;
}

/**
 *  @param input Points to an array of length n. Each element must be
 *               a 4-tuple (X, Y, Z, RGBA) corresponding to a depth
 *               map. The RGBA component is packed into a 32-bit float,
 *               with each element being an 8-bit unsigned integer.
 *  @param output Points to a matrix with m rows and p columns. Each
 *                element must be a 4-tuple (B, G, R, A). The elements
 *                should be stored contiguously in row-major order.
 *  @param resolution The resolution (in meters) of each pixel in
 *                    output, such that each pixel represents a
 *                    (resolution x resolution) square.
 *  @param x_offset The offset (in meters) between the center of the
 *                  matrix and its leftmost edge, such that the pixel
 *                  at (0, 0) is located in free space at (-x_offset,
 *                  -y_offset).
 *  @param y_offset The offset (in meters) between the center of the
 *                  matrix and its topmost edge, such that the pixel
 *                  at (0, 0) is located in free space at (-x_offset,
 *                  -y_offset).
 */
__global__ void transform(const Transform &tf, const sl::float4 *input, std::uint32_t n,
                          std::uint32_t *output, std::uint32_t m, std::uint32_t p,
                          float resolution, float x_offset, float y_offset) {
    const std::uint32_t pixel_idx = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (pixel_idx >= n) {
        return;
    }

    const sl::float4 &elem = input[pixel_idx];
    const Vector transformed = tf({elem[0], elem[1], elem[2]});

    const float pixel_x = (transformed.x() + x_offset) / resolution;
    const float pixel_y = (transformed.y() + y_offset) / resolution;

    if (pixel_x < 0 || pixel_y < 0) {
        return;
    }

    const auto i = static_cast<std::uint32_t>(pixel_y); // row idx
    const auto j = static_cast<std::uint32_t>(pixel_x); // col idx

    if (i >= m || j >= p) {
        return;
    }

    const sl::uchar4 rgba = from_packed(elem[3]);
    const std::uint32_t output_idx = i * p + j;
    output[output_idx] = pack_bgra(rgba[2], rgba[1], rgba[0], rgba[3]);
}



void DetectWhiteLines::convertXZGPU()
{
	tf2::Transform transform(imuQuat);
         
	tf2::Stamped<tf2::Transform> bodyTransform;
	tf2::fromMsg(bodyFrame,bodyTransform);
	
	transform *= bodyTransform;
	
	const Transform host_tf = {
		{
			float(transform.getBasis()[0][0]), float(transform.getBasis()[0][1]), float(transform.getBasis()[0][2]),
			float(transform.getBasis()[1][0]), float(transform.getBasis()[1][1]), float(transform.getBasis()[1][2]),
			float(transform.getBasis()[2][0]), float(transform.getBasis()[2][1]), float(transform.getBasis()[2][2])
		},	
		{float(transform.getOrigin()[0]),float(transform.getOrigin()[1]),float(transform.getOrigin()[2])}
	};
	const auto device_tf_ptr = to_device(host_tf);

	const float X_RANGE = MAX_X_VALUE - MIN_X_VALUE;
	const float Y_RANGE = MAX_Y_VALUE - MIN_Y_VALUE;
	const float OUTPUT_ROWS = Y_RANGE / YDIVISOR;
	const float OUTPUT_COLS = X_RANGE / XDIVISOR;
	const float RESOLUTION = XDIVISOR;
	

    cv::cuda::GpuMat output(OUTPUT_COLS, OUTPUT_ROWS, CV_8UC4, cv::Scalar(0, 0, 0, 255));

	const auto numel = static_cast<std::uint32_t>(point_cloud.getResolution().area());
    constexpr std::uint32_t BLOCKSIZE = 256;
    const std::uint32_t num_blocks = div_to_inf(numel, BLOCKSIZE);

	
	transform<<<num_blocks, BLOCKSIZE>>>(
            *device_tf_ptr,
            point_cloud.getPtr<sl::float4>(sl::MEM_GPU),
            numel,
            output.ptr<std::uint32_t>(),
            OUTPUT_ROWS,
            OUTPUT_COLS,
            RESOLUTION,
            X_RANGE / 2,
            Y_RANGE / 2
        );
        
     xzMat = cv::Mat(output);
}




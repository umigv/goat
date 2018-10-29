#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

struct TemporalFilter {
public:
    explicit TemporalFilter(ros::NodeHandle &node)
    : last_stamp_{ 0, 0 },
      publisher_{ node.advertise<sensor_msgs::Imu>("imu/data_filtered", 10) }
    { }

    void filter(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
        const auto now = ros::Time::now();
        const auto stamp = imu_ptr->header.stamp;

        if (stamp <= last_stamp_ || stamp > now) {
            return;
        }

        last_stamp_ = stamp;
        publisher_.publish(imu_ptr);
    }

private:
    ros::Time last_stamp_;
    ros::Publisher publisher_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "imu_temporal_filter_node");
    ros::NodeHandle node;

    TemporalFilter filter{ node };
    const auto subscriber =
        node.subscribe<sensor_msgs::Imu>("imu/data", 10,
                                         &TemporalFilter::filter, &filter);

    ros::spin();
}

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticFramePublisher : public rclcpp::Node
{
public:
    explicit StaticFramePublisher(char * tfs[])
        : Node("static_turtle_tf2_broadcaster")
    {
        tf_static_broad_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->make_transforms(tfs);
    }
private:
    void make_transforms(char * tfs[])
    {
        geometry_msgs::msg::TransformStamped transf;

        transf.header.stamp = this->get_clock()->now();
        transf.header.frame_id = "world";
        transf.child_frame_id = tfs[1];

        transf.transform.translation.x = atof(tfs[2]);
        transf.transform.translation.y = atof(tfs[3]);
        transf.transform.translation.z = atof(tfs[4]);
        
        tf2::Quaternion qua;
        qua.setRPY( atof(tfs[5]), atof(tfs[6]), atof(tfs[7]));

        transf.transform.rotation.x = qua.x();
        transf.transform.rotation.y = qua.y();
        transf.transform.rotation.z = qua.z();
        transf.transform.rotation.w = qua.w();

        tf_static_broad_->sendTransform(transf);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broad_;
};

int main(int argc, char * argv[])
{
    auto logger = rclcpp::get_logger("logger");

    if (argc != 8)
    {
        RCLCPP_INFO( logger, "Invalid usage, use form: "
            "$ ros2 run m03_tf2_with_cpp static_broadcaster"
            "child_name x y z roll pitch yaw");
        return 1;
    }

    if (strcmp(argv[1], "world") == 0)
    {
        RCLCPP_INFO(logger, "Cannot name child as parent, with name 'world'");
        return 1;
    }
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
    rclcpp::shutdown();

    return 0;
}
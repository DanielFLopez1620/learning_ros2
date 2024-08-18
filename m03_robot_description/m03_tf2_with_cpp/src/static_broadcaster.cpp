// ------------------ STANDARD DEPENDENCIES REQUIRED --------------------------
#include <memory>   // For dynamic memory mangament

// ------------------ ROS2 REQUIRED DEPENDENCIES ------------------------------
#include "rclcpp/rclcpp.hpp"                       // ROS2 Client Lib for C++
#include "tf2/LinearMath/Quaternion.h"             // Quaternion def
#include "tf2_ros/static_transform_broadcaster.h"  // Tf broadcaster

// ------------------ ROS2 MSG DEPENDENCIES -----------------------------------
#include "geometry_msgs/msg/transform_stamped.hpp" // Tf with stamped time


// ------------------ STATIC FRAME PUBLISHER IMPLEMENTATION -------------------

/**
 * Class oriented to generate a static frame that publishes via tf2 according to
 * the world (of the turtlesim).
 */
class StaticFramePublisher : public rclcpp::Node
{
public:
    /**
     * Explicit constructor that considers some arguments to create a static
     * publisher of a tf2 that considers position, rotation and child frame name.
     * Create the node with the name "static_turtle_tf2_broadcaster".
     * 
     * @param tfs Char array that considers the child frame name, x, y, z, r, 
     *            p, y positions and rotations separated by spaces as they are 
     *            passed from the terminal call or the launch file.
     */
    explicit StaticFramePublisher(char * tfs[])
        : Node("static_turtle_tf2_broadcaster")
    {
        // Define static broadcaster
        tf_static_broad_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Call function to create and broadcast the transform
        this->make_transforms(tfs);

    } // explicit StaticFramePublisher()

private:
    /**
     * Create static transform and publish it.
     * 
     * @param tfs Char array that considers the child frame name, x, y, z, r,
     *            p, y positions and rotations separated by spaces as they are
     *            passed from the terminal call or the launch file.
     */
    void make_transforms(char * tfs[])
    {
        // Declare a transform that counts with a stamp
        geometry_msgs::msg::TransformStamped transf;

        // Add header info related with time, parent frame and child frame
        transf.header.stamp = this->get_clock()->now();
        transf.header.frame_id = "world";
        transf.child_frame_id = tfs[1];

        // Update the traslational params of the tf
        transf.transform.translation.x = atof(tfs[2]);
        transf.transform.translation.y = atof(tfs[3]);
        transf.transform.translation.z = atof(tfs[4]);
        
        // Transforms work with quaternos, so the conversion from the rpy
        // rotation to the four components of the quaternion.
        tf2::Quaternion qua;
        qua.setRPY( atof(tfs[5]), atof(tfs[6]), atof(tfs[7]));

        // Update the rotatioanl params of the tf
        transf.transform.rotation.x = qua.x();
        transf.transform.rotation.y = qua.y();
        transf.transform.rotation.z = qua.z();
        transf.transform.rotation.w = qua.w();

        // Broadcast the transform
        tf_static_broad_->sendTransform(transf);

    } // make_transforms()

    // Declare static broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broad_;

}; // class StaticFramePUblisher()

// ----------------------- MAIN IMPLEMENTATION --------------------------------
int main(int argc, char * argv[])
{
    // Define logger
    auto logger = rclcpp::get_logger("logger");

    // Check that the number if parameters is valid to create the tf
    if (argc != 8)
    {
        RCLCPP_INFO( logger, "Invalid usage, use form: "
            "$ ros2 run m03_tf2_with_cpp static_broadcaster"
            "child_name x y z roll pitch yaw");
        return 1;
    }

    // Avoid duplication of world frame
    if (strcmp(argv[1], "world") == 0)
    {
        RCLCPP_INFO(logger, "Cannot name child as parent, with name 'world'");
        return 1;
    }
    
    // Initialize ROS2 Client Library for C++
    rclcpp::init(argc, argv);

    // Spin node via shared ptr
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));

    // Shutdown and return when spin is interrupted
    rclcpp::shutdown();
    return 0;

} // main()
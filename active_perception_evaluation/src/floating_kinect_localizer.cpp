#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>

#include <string>
#include <algorithm>

class FloatingKinectLocalizer {

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    tf::TransformBroadcaster tf_br;

    const tf::Transform sim_map_tf;

public:

    FloatingKinectLocalizer()
        : nh()
        , pose_sub(nh.subscribe("gazebo/link_states", 1, &FloatingKinectLocalizer::poseCb, this))
        , tf_br()
        , sim_map_tf(tf::Quaternion::getIdentity(), tf::Point(12.28, 10.20, 0.0))
    {}

    void poseCb(const gazebo_msgs::LinkStates & link_states)
    {
        size_t pos = std::find(link_states.name.begin(), link_states.name.end(), "floating_kinect::head_plate_frame") -
                     link_states.name.begin();

        if(pos == link_states.name.size()) {
            ROS_ERROR_ONCE("Could not find floating_kinect in link states");
            return;
        }

        tf::Transform cam_in_sim;
        tf::poseMsgToTF(link_states.pose[pos], cam_in_sim);

        // project cam pose to ground
        tf::Vector3 projected_position = cam_in_sim.getOrigin();
        projected_position.setZ(0);

        // we do not know the base orientation from the cam orientation, but let's assume that the base is always
        // oriented the same way as the camera in the xy-plane.
        tf::Quaternion base_rotation;
        tf::Vector3 xr = tf::Transform(cam_in_sim.getRotation(), tf::Vector3(0,0,0))(tf::Vector3(1,0,0));
        base_rotation.setRPY(0, 0, std::atan2(xr.getY(), xr.getX()));

        // finally add an offset of 7 cm in x direction.
        tf::Transform base_footprint(base_rotation, projected_position);
        tf::Transform base_offset(tf::Quaternion::getIdentity(), tf::Point(0.07, 0.0, 0.0));
        base_footprint = base_footprint * base_offset;

        // broadcast all transforms
        ros::Time now = ros::Time::now();
        tf_br.sendTransform(tf::StampedTransform(sim_map_tf, now, "/map", "/odom_combined"));
        tf_br.sendTransform(tf::StampedTransform(base_footprint, now, "/odom_combined", "/base_footprint"));
        tf_br.sendTransform(tf::StampedTransform(cam_in_sim, now, "/odom_combined", "/head_plate_frame"));
        tf_br.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), now, "/base_footprint", "/base_link"));
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "floating_kinect_localizer");
    FloatingKinectLocalizer node;
    ROS_INFO("floating_kinect_localizer: Initialized!");
    ros::spin();
}

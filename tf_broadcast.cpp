
#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

geometry_msgs::TransformStamped get_tf(){
    geometry_msgs::TransformStamped get_tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while(1) {
        try
        {
            get_tf = tfBuffer.lookupTransform("world","camera_depth_optical_frame", ros::Time(0));
            std::cout<<get_tf<<std::endl;
            ROS_INFO_ONCE("I got a transform");
            break;
        }
        catch(tf2::TransformException &e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
        
    }
    return get_tf;

}


void tf_callback(std_msgs::Float32MultiArray msg){
    geometry_msgs::TransformStamped original_tf;
    original_tf = get_tf();
    geometry_msgs::TransformStamped dynamic_tf_;
    tf2_ros::StaticTransformBroadcaster tf_;
    
    dynamic_tf_.header.frame_id = "camera_depth_optical_frame";
    dynamic_tf_.child_frame_id = "picking_point";
    double x, y, z;
    x = msg.data[0];
    y = msg.data[1];
    z = msg.data[2];

    
    
    double x_1, y_1, z_1;

    tf2::Quaternion q_moto(x, y, z, 0), quat, quat_after, q_zero(0, 0, 1, 0);
    tf2::convert(original_tf.transform.rotation, quat);
    quat_after = quat * q_zero * quat.inverse();
    tf2::Vector3 kaiten_jiku(quat_after[0], quat_after[1], quat_after[2]);
    tf2::Quaternion quat_1;
    quat_1.setRotation(kaiten_jiku, 0);
    
    quat_after = quat_1 * q_moto * quat_1.inverse();
    x_1 = quat_after[0];
    y_1 = quat_after[1];
    z_1 = quat_after[2];
    quat.setRPY(0.0, 0, 0);
    // dynamic_tf_.transform.rotation.x = quat.x();
    // dynamic_tf_.transform.rotation.y = quat.y();
    // dynamic_tf_.transform.rotation.z = quat.z();
    // dynamic_tf_.transform.rotation.w = quat.w();
    dynamic_tf_.transform.translation.x = x_1;
    dynamic_tf_.transform.translation.y = y_1;
    dynamic_tf_.transform.translation.z = z_1;
    dynamic_tf_.transform.rotation.x = quat[0];
    dynamic_tf_.transform.rotation.y = quat[1];
    dynamic_tf_.transform.rotation.z = quat[2];
    dynamic_tf_.transform.rotation.w = quat[3];
    ros::Rate lop(10);
    int count = 0;
    while (count <= 10){
        dynamic_tf_.header.stamp = ros::Time::now();
        tf_.sendTransform(dynamic_tf_);
        lop.sleep();
        count ++ ;
    }
    // tf_.sendTransform(dynamic_tf_);
    ROS_INFO_STREAM("callback done");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcast");
    ros::NodeHandle nh;

    ros::Subscriber sub;
    sub = nh.subscribe("/ggcnn/out/command", 10, tf_callback);
    ROS_INFO_STREAM("callback come");

    ros::spin();
    return 0;

}
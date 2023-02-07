#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::TransformStamped Picking_Point(){
    geometry_msgs::TransformStamped get_tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (1){
        try
        {
            get_tf = tfBuffer.lookupTransform("picking_point", "world", ros::Time(0));
            break;
        }
        catch(tf2::TransformException & e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    return get_tf;
}

// void tf_callback()
// {
//     geometry_msgs::TransformStamped original_tf;
//     original_tf = Picking_Point();

//     std_msgs::Float32MultiArray topic;
//     topic.data[0] = original_tf.transform.translation.x;
//     topic.data[1] = original_tf.transform.translation.y;
//     topic.data[2] = original_tf.transform.translation.z;

//     while (ros::ok())
//     {
//         ros::NodeHandle nh;
//         ros::Publisher pub;
//         pub = nh.advertise<std_msgs::Float32MultiArray>("Picking_Point", 10);
//     }

// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_listen");
    ros::NodeHandle nh;

    // ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("Picking_Point", 10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::TransformStamped original_tf;
        original_tf = Picking_Point();

        // std_msgs::Float32MultiArray msg;
        // msg.data[0] = original_tf.transform.translation.x;
        // msg.data[1] = original_tf.transform.translation.y;
        // msg.data[2] = original_tf.transform.translation.z;

        // pub.publish(msg);
        float x, y, z;
        x = original_tf.transform.translation.x;
        y = original_tf.transform.translation.y;
        z = original_tf.transform.translation.z;

        ROS_INFO_STREAM("x:" << x << "y:" << y << "z:" << z);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
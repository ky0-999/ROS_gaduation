


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

geometry_msgs::TransformStamped get_tf(std::string target_frame){
    geometry_msgs::TransformStamped get_tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while(1) {
        try
        {
            get_tf = tfBuffer.lookupTransform("world", target_frame, ros::Time(0));
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

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

void tf_callback(){
    ros::Rate loop(1);
    while(ros::ok()){
        geometry_msgs::TransformStamped goal_tf, object_tf, gripper_tf;
        tf2_ros::StaticTransformBroadcaster tf_;
        object_tf = get_tf("picking_point");
        // gripper_tf = get_tf("body_link");
        gripper_tf = get_tf("ee_link");
        // goal_tf.header.stamp = ros::Time::now();
        goal_tf.header.frame_id = "world";
        goal_tf.child_frame_id = "goal_tf";

        tf2::Quaternion q_zero(0, 0, -0.2175, 0), q_convert, q_pose_convert, q_trans_ato, q_pose_ato;
        tf2::convert(object_tf.transform.rotation, q_convert);
        q_trans_ato = q_convert  * q_zero * q_convert.inverse();
        goal_tf.transform.translation.x = q_trans_ato[0] + object_tf.transform.translation.x;
        goal_tf.transform.translation.y = q_trans_ato[1] + object_tf.transform.translation.y;
        goal_tf.transform.translation.z = q_trans_ato[2] + object_tf.transform.translation.z;
        // goal_tf.transform.translation.x = object_tf.transform.translation.x;
        // goal_tf.transform.translation.y = object_tf.transform.translation.y;
        // goal_tf.transform.translation.z = object_tf.transform.translation.z;
        goal_tf.transform.rotation.x = gripper_tf.transform.rotation.x;
        goal_tf.transform.rotation.y = gripper_tf.transform.rotation.y;
        goal_tf.transform.rotation.z = gripper_tf.transform.rotation.z;
        goal_tf.transform.rotation.w = gripper_tf.transform.rotation.w;
        tf2::Quaternion q_z(0, 0, 1, 0), q_y(0, 1, 0, 0);
        tf2::Quaternion quat = convert_quat(q_z, q_convert, M_PI_2)*q_convert;
        tf2::convert(convert_quat(q_y, quat, -M_PI_2)*quat,  goal_tf.transform.rotation);
        ros::Rate lop(10);
        int count = 0;
        while(count <= 5){
            goal_tf.header.stamp = ros::Time::now();
            tf_.sendTransform(goal_tf);
            // std::cout<<goal_tf<<std::endl;
            lop.sleep();
            count ++ ;
        }
    loop.sleep();
    }
    
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_goal");
    ros::NodeHandle nh;

    tf_callback();
    ROS_INFO_STREAM("callback come");

    ros::spin();
    return 0;

}
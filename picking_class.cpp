#include <picking_class.hpp>

ggcnn_picking::ggcnn_picking(ros::NodeHandle& node_handle)
    :   manipulator_("manipulator"),
        gripper_group_("gripper"),
        gripper(new RobotiqActionClient("/command_robotiq_action", true)){
            ros::param::param<std::string>(
                "~task_frame",
                scene_task_frame_,
                "base_link"
            );
            // manipulator_.setPoseReferenceFrame("base_link");
            manipulator_.setPoseReferenceFrame("world");
            manipulator_.setMaxVelocityScalingFactor(1.0);
            // manipulator_.setEndEffectorLink("body_link");
            manipulator_.setEndEffectorLink("ee_link");
            std::cout << manipulator_.getEndEffectorLink() << std::endl;

            SetupPlanningScene();
            // move_home();
            move_camera();
            sub_ = node_handle.subscribe("ggcnn/out/command", 10, &ggcnn_picking::PickAndPlace, this);
        }

void ggcnn_picking::SetupPlanningScene()
{
    ROS_INFO_STREAM("Setting up plnning scene");
    //Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()){
        objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()){
        objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.001;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    std_msgs::ColorRGBA colour;
    colour.b = 0.5;
    colour.a = 1;
    scene_.applyCollisionObject(table, colour);

    //Let the planner know that this is the surface supporting we will
    //be picking and placing, so collisions are allowed
    manipulator_.setSupportSurfaceName("table");
}

bool ggcnn_picking::move_home()
{
    ROS_INFO_STREAM("Move to home pose");
    manipulator_.setNamedTarget("home");
    if(!manipulator_.move()){
        ROS_WARN_STREAM("move_home() failed");
        ros::Duration(0.5).sleep();
        return false;
    }
    ROS_INFO_STREAM("move_home() complete");
    ros::Duration(0.5).sleep();
    return true;
}

void ggcnn_picking::PickAndPlace(std_msgs::Float32MultiArray::ConstPtr const &msg)
{
    // DoPick_M1(msg);
    // Gripper_Open();
    
    // move_home();
    geometry_msgs::TransformStamped a = Picking_point();
    move_end_effector_set_tf(a, 0.01);
    Rotate_Wrist_3(msg);
    Gripper_Move(msg);
    ros::Duration(0.5).sleep();
    Move_Down();
    Gripper_Close();
    Move_Up();
    move_home();
    Rotate_Base();
    Gripper_Open();
    move_camera();
    ros::Duration(1.5).sleep();
}

bool ggcnn_picking::Move_Point(std_msgs::Float32MultiArray::ConstPtr const &msg)
{
    ROS_INFO_STREAM("Move to, x:" << msg->data[0] << " y:" << msg->data[1] << " z:" << msg->data[2]);
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
    wpose.position.x = msg->data[0] * 0.001;
    wpose.position.y = msg->data[1] * 0.001;
    wpose.position.z = msg->data[2] * 0.01 + 0.15;
    // wpose.position.z = 0.2;
    // wpose.orientation.x = sin(M_PI);
    // wpose.orientation.y = sin(2*M_PI);
    // wpose.orientation.z = sin(2*M_PI);
    // wpose.orientation.w = cos(2*M_PI);
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    
    if(!manipulator_.execute(trajectory)){
        ROS_WARN_STREAM("Move_Point() failed");
        return false;
    }
    ROS_INFO_STREAM("Move_Point() complete");
    return true;
}

void ggcnn_picking::Gripper_Move(std_msgs::Float32MultiArray::ConstPtr const &msg)
{
    ROS_INFO_STREAM("Gripper width is" << msg->data[4]);
    gripper->goToPosition(msg->data[4]);
}

bool ggcnn_picking::Move_Down()
{
    ROS_INFO_STREAM("Move_Down() start");
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
    wpose.position.z -= 0.05;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.001;
    double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);

    if(!manipulator_.execute(trajectory)){
        ROS_WARN_STREAM("Move_Down() failed");
        ros::Duration(0.5).sleep();
        return false;
    }
    ROS_INFO_STREAM("Move_Down() complete");
    ros::Duration(0.5).sleep();
    return true;
}

bool ggcnn_picking::Move_Up()
{
    ROS_INFO_STREAM("Move_Up() start");
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
    wpose.position.z += 0.15;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.001;
    double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);

    if(!manipulator_.execute(trajectory)){
        ROS_WARN_STREAM("Move_Up() failed");
        ros::Duration(0.5).sleep();
        return false;
    }
    ROS_INFO_STREAM("Move_Up() complete");
    ros::Duration(0.5).sleep();
    return true;
}

bool ggcnn_picking::Gripper_Open()
{
    gripper->open();
    return true;
}

bool ggcnn_picking::Gripper_Close()
{
    gripper->close();
    return true;
}

// bool ggcnn_picking::DoPick_M1(std_msgs::Float32MultiArray::ConstPtr const &msg)
// {
//     while(1){
//         move_point:
//             if(Move_Point(msg)){
//                 move_down:
//                     if(Move_Down()){
//                         Rotate_Wrist_3(msg);
//                         gripper_close:
//                             if(Gripper_Close()){
//                                 Picked:
//                                     if(Move_Up()){
//                                         break;
//                                     }else{
//                                         Gripper_Open();
//                                         Gripper_Close();
//                                         goto Picked;
//                                     }
//                             }else{
//                                 Gripper_Open();
//                                 Move_Up();
//                                 goto move_down;
//                             }
//                     }else{
//                         goto move_point;
//                     }
//             }else{
//                 move_home();
//                 goto move_point;
//             }
//     }
// }

void ggcnn_picking::Rotate_Wrist_3(std_msgs::Float32MultiArray::ConstPtr const &msg)
{
    const robot_state::JointModelGroup* joint_model_group =
        manipulator_.getCurrentState()->getJointModelGroup("manipulator");
    moveit::core::RobotStatePtr current_state = manipulator_.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, start_joint_position);
    
    start_joint_position[5] = start_joint_position[5] - msg->data[3];
    manipulator_.setJointValueTarget(start_joint_position);
    manipulator_.move();
    ros::Duration(0.5).sleep();
}

// void ggcnn_picking::Cartesian_Move(std_msgs::Float32MultiArray::ConstPtr const &msg)
// {
//     ROS_INFO_STREAM("Cartesian_Move() started");
//     std::vector<geometry_msgs::Pose> waypoints;
//     geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
//     wpose.position.x += msg->data[0] - wpose.position.x;
//     wpose.position.y += msg->data[1] - wpose.position.y;
//     waypoints.push_back(wpose);
//     moveit_msgs::RobotTrajectory trajectory;

//     const double jump_thresold = 0.0;
//     const double eef_step = 0.01;
//     double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
//     manipulator_.execute(trajectory);
    
// }

void ggcnn_picking::Cartesian_Move(std_msgs::Float32MultiArray::ConstPtr const &msg)
{
    geometry_msgs::TransformStamped original_tf;
    original_tf = Picking_point();
    ROS_INFO_STREAM("Cartesian_Move() started");
    ROS_INFO_STREAM("x:" << original_tf.transform.translation.x << "y:" << original_tf.transform.translation.y);
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
    // wpose.position.x += original_tf.transform.translation.x - wpose.position.x;
    // wpose.position.y += original_tf.transform.translation.y - wpose.position.y;
    wpose.position.x += original_tf.transform.translation.x;
    wpose.position.y += original_tf.transform.translation.y;
    // wpose.position.z += original_tf.transform.translation.z;
    
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;

    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    manipulator_.execute(trajectory);
    
}

geometry_msgs::TransformStamped ggcnn_picking::Picking_point()
{
    geometry_msgs::TransformStamped get_tf;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    while (1){
        try
        {
            get_tf = tfBuffer.lookupTransform("world", "goal_tf", ros::Time(0));
            // std::cout<<get_tf<<std::endl;
            // get_tf = tfBuffer.lookupTransform("world", "picking_point", ros::Time(0));
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

bool ggcnn_picking::move_camera()
{
    ROS_INFO_STREAM("Move to camera pose");
    manipulator_.setNamedTarget("camera");
    if(!manipulator_.move()){
        ROS_WARN_STREAM("move_camera() failed");
        ros::Duration(0.5).sleep();
        return false;
    }
    ROS_INFO_STREAM("move_camera() complete");
    ros::Duration(0.5).sleep();
    return true;
}

void ggcnn_picking::move_end_effector_set_tf(geometry_msgs::TransformStamped goal_tf, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = manipulator_.getCurrentPose().pose;
    wpose.position.x = goal_tf.transform.translation.x ;
    wpose.position.y = goal_tf.transform.translation.y;
    wpose.position.z = goal_tf.transform.translation.z ;
    std::cout<< "wpose= " << wpose <<std::endl; 
    tf2::convert(goal_tf.transform.rotation, wpose.orientation);
    waypoints.push_back(wpose);

    moveit_msgs::RobotTrajectory trajectory;
    double jump_thresh = 0.0;
    double fraction = manipulator_.computeCartesianPath(waypoints, eef_step, jump_thresh, trajectory);
    manipulator_.execute(trajectory);

    wpose = manipulator_.getCurrentPose().pose;
    ros::Duration(1.0).sleep();
    

}

void ggcnn_picking::Rotate_Base()
{
    const robot_state::JointModelGroup* joint_model_group =
        manipulator_.getCurrentState()->getJointModelGroup("manipulator");
    moveit::core::RobotStatePtr current_state = manipulator_.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, start_joint_position);
    
    start_joint_position[0] = start_joint_position[5] + 2*M_PI/3;
    manipulator_.setJointValueTarget(start_joint_position);
    manipulator_.move();
    ros::Duration(0.5).sleep();
}
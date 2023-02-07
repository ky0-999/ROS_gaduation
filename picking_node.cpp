#include <picking_class.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "picking_node");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    ggcnn_picking gp(nh);

    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
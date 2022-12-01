#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller");

    // Istantiate controller
    UR5Controller controller(1000.0, 0.005, 10.0);

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();
    
    JointStateVector joints;
    double x, y, z;
    Coordinates pos;
    RotationMatrix rot = RotationMatrix::Identity();

    while (ros::ok())
    {
        printf("\nInsert x y z (1903 to exit): ");
        cin >> x >> y >> z;

        if (x == 1903) exit(0);

        pos << x, y, z;

        cout << "Selected position: " << pos.transpose() << endl;

        cout << "\n========= movement output ==========" << endl;
        controller.ur5_move_to(pos, rot, true);
        cout << "========= movement output ==========\n" << endl;
        
        controller.ur5_get_joint_states(joints);

        ur5_direct(joints, pos, rot);

        cout << "Final position: " << pos.transpose() << endl;
        cout << "Joints states: " << joints.transpose() << endl;
    }


    return 0;
}
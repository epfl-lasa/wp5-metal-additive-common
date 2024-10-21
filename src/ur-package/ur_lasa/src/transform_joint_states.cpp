#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


using namespace std;


class JointStateHandlerUR5 {
    public:
        vector<double> jointPosition;
        vector<double> jointSpeed;
        bool init_joint;
        JointStateHandlerUR5(ros::NodeHandle& nh) : nh_(nh) {
            jointPosition = {0,0,0,0,0,0};
            jointSpeed    = {0,0,0,0,0,0};
            init_joint = true;
        
            sub_ = nh_.subscribe("/ur5/joint_states", 10, &JointStateHandlerUR5::jointStateCallback, this);
        }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (!msg->position.empty()) {
                jointPosition = msg->position; // Update the position vector with received positions
                jointSpeed = msg->velocity; // Update the position vector with received positions
                swap(jointPosition[0], jointPosition[2]);
                swap(jointSpeed[0], jointSpeed[2]);

                init_joint = false;
            } else {
                ROS_WARN("Received joint positions are empty.");
            }
    }
    private:
        ros::NodeHandle nh_; // Member field to store the NodeHandle
        ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_republisher");

    ros::NodeHandle nh;
    ros::spin(); // Keep the node running and continue listening for messages
   
    JointStateHandlerUR5 JsHandlerUR5(nh);

    return 0;
}




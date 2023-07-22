/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <towr/terrain/height_map.h>
#include <xpp_states/convert.h>

#include <towr_ros/TowrCommand.h>  // listen to goal state
#include <towr_ros/topic_names.h>
#include <towr_ros/goal_pose_publisher.h>
#include <towr_ros/towr_ros_interface.h>

// extern double goal_pos_x;
// extern double goal_pos_y;
// extern double goal_pos_z;
// extern double goal_q_x;
// extern double goal_q_y;
// extern double goal_q_z;
// extern double goal_q_w;
// static ros::Publisher rviz_pub;
// void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
// {
//     // get which terrain
//     auto terrain_id = static_cast<towr::HeightMap::TerrainID>(msg_in.terrain);
//     auto terrain_ = towr::HeightMap::MakeTerrain(terrain_id);

//     geometry_msgs::PoseStamped goal_msg;
//     goal_msg.header.frame_id = "world";

//     // visualize goal z state on terrain.
//     double x = msg_in.goal_lin.pos.x;
//     double y = msg_in.goal_lin.pos.y;
//     goal_msg.pose.position.x = x;
//     goal_msg.pose.position.y = y;
//     goal_msg.pose.position.z = terrain_->GetHeight(x, y);

//     // orientation according to message
//     Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(msg_in.goal_ang.pos.z,
//                                                           msg_in.goal_ang.pos.y,
//                                                           msg_in.goal_ang.pos.x);
//     goal_msg.pose.orientation = xpp::Convert::ToRos(q);

//     // goal_pos_x = goal_msg.pose.position.x;
//     // goal_pos_y = goal_msg.pose.position.y;
//     // goal_pos_z = goal_msg.pose.position.z;
//     // goal_q_x = q.x();
//     // goal_q_y = q.y();
//     // goal_q_z = q.z();
//     // goal_q_w = q.w();
//     SetGoal(goal_pos_x, goal_pos_y, goal_pos_z, goal_q_x, goal_q_y, goal_q_z, goal_q_w,
//             goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z, q.x(), q.y(), q.z(), q.w());

//     printf("###### %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", goal_pos_x, goal_pos_y, goal_pos_z, goal_q_x, goal_q_y, goal_q_z, goal_q_w);

//     rviz_pub.publish(goal_msg);
// }


// void SetGoal(double& goal_pos_x,double& goal_pos_y,double& goal_pos_z,double& goal_q_x,double& goal_q_y,double& goal_q_z,double& goal_q_w,
//              double goal_pos_x_set, double goal_pos_y_set, double goal_pos_z_set, double goal_q_x_set, double goal_q_y_set, double goal_q_z_set, 
//              double goal_q_w_set) {
//     goal_pos_x = goal_pos_x_set;
//     goal_pos_y = goal_pos_y_set;
//     goal_pos_z = goal_pos_z_set;
//     goal_q_x = goal_q_x_set;
//     goal_q_y = goal_q_y_set;
//     goal_q_z = goal_q_z_set;
//     goal_q_w = goal_q_w_set;
// }

 namespace towr {

// double goal_pos_x = 1.0;
// double goal_pos_y = 0.0;
// double goal_pos_z = 0.0;
// double goal_q_x = 0.0;
// double goal_q_y = 0.0;
// double goal_q_z = 0.0;
// double goal_q_w = 0.0;


static ros::Publisher rviz_pub;

void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
{
    // get which terrain
    auto terrain_id = static_cast<HeightMap::TerrainID>(msg_in.terrain);
    auto terrain_ = HeightMap::MakeTerrain(terrain_id);

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "world";

    // visualize goal z state on terrain.
    double x = msg_in.goal_lin.pos.x;
    double y = msg_in.goal_lin.pos.y;
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = terrain_->GetHeight(x, y);

    // orientation according to message
    Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(msg_in.goal_ang.pos.z,
                                                          msg_in.goal_ang.pos.y,
                                                          msg_in.goal_ang.pos.x);
    goal_msg.pose.orientation = xpp::Convert::ToRos(q);

    // goal_pos_x = goal_msg.pose.position.x;
    // goal_pos_y = goal_msg.pose.position.y;
    // goal_pos_z = goal_msg.pose.position.z;
    // goal_q_x = q.x();
    // goal_q_y = q.y();
    // goal_q_z = q.z();
    // goal_q_w = q.w();
    // printf("###### %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", goal_pos_x, goal_pos_y, goal_pos_z, goal_q_x, goal_q_y, goal_q_z, goal_q_w);

    // goal_publisher.SetGoal(goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z, q.x(), q.y(), q.z(), q.w());
    // printf("###### %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", goal_publisher.goal_set[0], goal_publisher.goal_set[1], goal_publisher.goal_set[2], 
    //                          goal_publisher.goal_set[3], goal_publisher.goal_set[4], goal_publisher.goal_set[5], goal_publisher.goal_set[6]);

    rviz_pub.publish(goal_msg);
}

 } // namespace towr

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "goal_pose_publisher");

    ros::NodeHandle n;

    ros::Subscriber goal_sub;
      goal_sub       = n.subscribe(towr_msgs::user_command, 1, towr::UserCommandCallback);
    // goal_sub       = n.subscribe(towr_msgs::user_command, 1, UserCommandCallback);
    towr::rviz_pub = n.advertise<geometry_msgs::PoseStamped>("xpp/goal", 1);
    // rviz_pub = n.advertise<geometry_msgs::PoseStamped>("xpp/goal", 1);

    ros::spin();

    return 1;
}

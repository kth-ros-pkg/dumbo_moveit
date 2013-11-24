/*
 *  dumbo_moveit_example.cpp
 *
 *  Created on: Nov 19, 2013
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>


int main(int argc, char **argv)
{

	ros::init(argc, argv, "dumbo_move_group_example");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup l_group("left_arm");
	move_group_interface::MoveGroup r_group("right_arm");


	// set position and orientation of left_arm_7_link with respect to arm_base_link
	geometry_msgs::Pose p;

	p.position.x = -0.486;
	p.position.y = 0.367;
	p.position.z = 0.0;

	double roll = 1.571;
	double pitch = 1.571;
	double yaw = 3.13;
	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	tf::quaternionTFToMsg(q, p.orientation);

	geometry_msgs::PoseStamped p_stamped;
	p_stamped.pose = p;
	p_stamped.header.frame_id = "arm_base_link";
	p_stamped.header.stamp = ros::Time::now();


	l_group.setPoseTarget(p_stamped);

	l_group.move();

	ros::Duration(3.0).sleep();

	l_group.setNamedTarget("l_default");
	r_group.setNamedTarget("r_default");

	l_group.move();

	r_group.move();

	ros::shutdown();
	return 0;


}

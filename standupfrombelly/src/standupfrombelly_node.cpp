#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nao_msgs/JointAnglesWithSpeed.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "standupfrombelly_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	ros::Publisher pub = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
	
	float fast = 0.9;
	nao_msgs::JointAnglesWithSpeed mhp, mhy, mler, mrer, mley, mrey, mlwy, mrwy, mrsr, mlsr, mrsp, mlsp,
	mrhp, mlhp, mrhyp, mlhyp, mrkp, mlkp, mrar, mlar, mrap, mlap, mlhr, mrhr;

	mhp.joint_names.push_back("HeadPitch"); mhy.joint_names.push_back("HeadYaw"); mler.joint_names.push_back("LElbowRoll");
	mlwy.joint_names.push_back("LWristYaw"); mrwy.joint_names.push_back("RWristYaw"); mley.joint_names.push_back("LElbowYaw");
	mrey.joint_names.push_back("RElbowYaw"); mlsr.joint_names.push_back("LShoulderRoll"); mlsp.joint_names.push_back("LShoulderPitch");
	mrsr.joint_names.push_back("RShoulderRoll"); mrsp.joint_names.push_back("RShoulderPitch"); mrer.joint_names.push_back("RElbowRoll");
	mrhp.joint_names.push_back("RHipPitch"); mlhp.joint_names.push_back("LHipPitch"); mrhyp.joint_names.push_back("RHipYawPitch");
	mlhyp.joint_names.push_back("LHipYawPitch"); mrkp.joint_names.push_back("RKneePitch"); mlkp.joint_names.push_back("LKneePitch");
	mrar.joint_names.push_back("RAnkleRoll"); mlar.joint_names.push_back("LankleRoll"); mrap.joint_names.push_back("RAnklePitch");
	mlap.joint_names.push_back("LAnklePitch"); mlhr.joint_names.push_back("LHipRoll"); mrhr.joint_names.push_back("RHipRoll");
	mhp.joint_angles.push_back(0); mhy.joint_angles.push_back(0); mler.joint_angles.push_back(0);
	mlwy.joint_angles.push_back(0); mrwy.joint_angles.push_back(0); mley.joint_angles.push_back(0);
	mrey.joint_angles.push_back(0); mlsr.joint_angles.push_back(0); mlsp.joint_angles.push_back(0);
	mrsr.joint_angles.push_back(0); mrsp.joint_angles.push_back(0); mrer.joint_angles.push_back(0);
	mrhp.joint_angles.push_back(0); mlhp.joint_angles.push_back(0); mrhyp.joint_angles.push_back(0);
	mlhyp.joint_angles.push_back(0); mrkp.joint_angles.push_back(0); mlkp.joint_angles.push_back(0);
	mrar.joint_angles.push_back(0); mlar.joint_angles.push_back(0); mrap.joint_angles.push_back(0);
	mlap.joint_angles.push_back(0); mlhr.joint_angles.push_back(0); mrhr.joint_angles.push_back(0);
	pub.publish(mhp); pub.publish(mhy); pub.publish(mler);
	pub.publish(mlwy); pub.publish(mrwy); pub.publish(mley);
	pub.publish(mrey); pub.publish(mlsr); pub.publish(mlsp);
	pub.publish(mrsr); pub.publish(mrsp); pub.publish(mrer);
	pub.publish(mrhp); pub.publish(mlhp); pub.publish(mrhyp);
	pub.publish(mlhyp); pub.publish(mrkp); pub.publish(mlkp); 
	pub.publish(mrar); pub.publish(mlar); pub.publish(mrap);
	pub.publish(mlap); pub.publish(mlhr); pub.publish(mrhr);
	mhp.speed = fast; mhy.speed = fast; mler.speed = fast;
	mlwy.speed = fast; mrwy.speed = fast; mley.speed = fast;
	mrey.speed = fast; mlsr.speed = fast; mlsp.speed = fast;
	mrsr.speed = fast; mrsp.speed = fast; mrer.speed = fast;
	mrhp.speed = fast; mlhp.speed = fast; mrhyp.speed = fast;
	mlhyp.speed = fast; mrkp.speed = fast; mlkp.speed = fast;
	mrar.speed = fast; mlar.speed = fast; mrap.speed = fast;
	mlap.speed = fast; mlhr.speed = fast; mrhr.speed = fast;
	
	while(ros::ok()){
		ROS_INFO("ROBOT IS ON ITS BACK");
		ros::Duration(2).sleep();
		ROS_INFO("SETTING ROBOT UPRIGHT\n");
		ros::Duration(2).sleep();
			
		/* slightly moves head pitch so it is no resting on it */
		ROS_INFO("CHANGING HEAD PITCH\n");
		mhp.joint_angles[0] = 1;
		pub.publish(mhp);
		//loop_rate.sleep();

		/* moves the arms so that they are close the the nao's hips */
		ROS_INFO("CHANGING ARMS\n");
		mrsr.joint_angles[0] = -1; mlsr.joint_angles[0] = 1;
		pub.publish(mrsr); pub.publish(mlsr);
		loop_rate.sleep();
		mrer.joint_angles[0] = 1.5; mler.joint_angles[0] = -1.5;
		pub.publish(mrer); pub.publish(mler);
		loop_rate.sleep();
		mrsp.joint_angles[0] = 1.5; mlsp.joint_angles[0] = 1.5;
		pub.publish(mrsp); pub.publish(mlsp);
		loop_rate.sleep();
		mley.joint_angles[0] = 0.5; mrey.joint_angles[0] = -0.5;
		pub.publish(mley); pub.publish(mrey);
		//loop_rate.sleep();

		/* slightly moves the hip pitch so that the nao can move its arms behind its back */
		ROS_INFO("MOVING HIP PITCH\n");
		mrhp.joint_angles[0] = 0.5; mlhp.joint_angles[0] = 0.5;
		pub.publish(mrhp); pub.publish(mlhp);
		loop_rate.sleep();
		
		/* moves arms behind nao */
		ROS_INFO("MOVING ARMS\n");
		mrsr.joint_angles[0] = -0.5; mlsr.joint_angles[0] = 0.5;
		pub.publish(mrsr); pub.publish(mlsr);
		//loop_rate.sleep();
		
		/* moves the knee pitch amd ankle pitch to support the nao better */
		ROS_INFO("MOVING KNEE PITCH AND ANKLE PITCH\n");
		mrkp.joint_angles[0] = 0.5; mlkp.joint_angles[0] = 0.5;
		mrap.joint_angles[0] = 0.5; mlap.joint_angles[0] = 0.5;
		pub.publish(mrkp); pub.publish(mlkp);
		pub.publish(mrap); pub.publish(mlap);
		loop_rate.sleep();	
	
		/* moving arms further behind nao */
		ROS_INFO("MOVING ARMS\n");
		mrsr.joint_angles[0] = 0.3142; mlsr.joint_angles[0] = -0.3142;
		pub.publish(mrsr); pub.publish(mlsr);
		loop_rate.sleep();
		
		/* moves hip pitch to fall back on the arms */
		ROS_INFO("MOVING HIP PITCH\n");
		mrhp.joint_angles[0] = -0.5; mlhp.joint_angles[0] = -0.5;
		pub.publish(mrhp); pub.publish(mlhp);
		loop_rate.sleep();
	
		/* moves ankle pitch so that the robot can sit up */
		mrap.joint_angles[0] = 0.5; mlap.joint_angles[0] = 0.5;
		pub.publish(mrap); pub.publish(mlap);
		//loop_rate.sleep();

		/* moves knee pitch so that they are fully bent */
		mrkp.joint_angles[0] = 2; mlkp.joint_angles[0] = 2;
		pub.publish(mrkp); pub.publish(mlkp);
		//loop_rate.sleep();

		/* moves hip pitch so that robot is completely sitting up */
		mrhp.joint_angles[0] = -1; mlhp.joint_angles[0] = -1;		
		pub.publish(mrhp); pub.publish(mlhp);
		loop_rate.sleep();

		ros::Duration(20).sleep();
	}
	return 0;
}

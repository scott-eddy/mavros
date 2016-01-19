/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavplugin {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public MavRosPlugin {
public:
	LocalPositionPlugin() :
		lp_nh("~local_position"),
		uas(nullptr),
		tf_send(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		lp_nh.param<std::string>("frame_id", frame_id, "fcu");
		// tf subsection
		lp_nh.param("tf/send", tf_send, true);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "local_origin");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "fcu");
		// nav_msgs/Odometry info
		lp_nh.param<std::string>("tf/base_link_frame_id",tf_base_link_frame_id,"base_link_aircraft");

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		local_velocity = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);
		local_odom = lp_nh.advertise<nav_msgs::Odometry>("odom",10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &LocalPositionPlugin::handle_local_position_ned)
		};
	}

private:
	ros::NodeHandle lp_nh;
	UAS *uas;

	ros::Publisher local_position;
	ros::Publisher local_velocity;
	ros::Publisher local_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	std::string tf_base_link_frame_id; //!< frame for TF (specifically Odometry message)
	bool tf_send;

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

		auto enu_position = UAS::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
		auto enu_velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

		auto orientation = uas->get_attitude_orientation();
		auto angular_velocity = uas->get_attitude_angular_velocity();

		// Here the orientation quaternion is describing the rotation from the body frame 
		// to the ENU frame.  We also have the velocity expressed in the ENU frame.  We
		// Therefore need to calculate the inverse quaternion to find the transformation
		// from the ENU frame to the body frame.  Then we can apply this rotation to the 
		// velocity expressed in the ENU frame to get the body frame velocity.  
		auto enu_to_body_quad = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).inverse();
		auto enu_angular_velocity = Eigen::Vector3d(angular_velocity.x, angular_velocity.y, angular_velocity.z);
		Eigen::Transform<double, 3, Eigen::Affine> enu_to_body_rot(enu_to_body_quad);
		auto body_linear_velocity = enu_to_body_rot * enu_velocity;
		auto body_angular_velocity = enu_to_body_rot * enu_angular_velocity;

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
		auto odom = boost::make_shared<nav_msgs::Odometry>();

		pose->header = uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		twist->header = pose->header;

		tf::pointEigenToMsg(enu_position, pose->pose.position);
		pose->pose.orientation = orientation;

		tf::vectorEigenToMsg(enu_velocity,twist->twist.linear);
		twist->twist.angular = angular_velocity;
		
		local_position.publish(pose);
		local_velocity.publish(twist);

		odom->header.stamp = pose->header.stamp;
		odom->header.frame_id = tf_frame_id;
		odom->child_frame_id = tf_base_link_frame_id;
		tf::vectorEigenToMsg(body_linear_velocity,odom->twist.twist.linear);
		tf::vectorEigenToMsg(body_angular_velocity,odom->twist.twist.angular);
		odom->pose.pose = pose->pose;

		local_odom.publish(odom);

		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = pose->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = orientation;
			tf::vectorEigenToMsg(enu_position, transform.transform.translation);

			uas->tf2_broadcaster.sendTransform(transform);
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionPlugin, mavplugin::MavRosPlugin)



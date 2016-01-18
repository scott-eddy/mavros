/**
 * @brief Frame conversions helper functions
 * @file uas_frame_conversions.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <array>
#include <mavros/mavros_uas.h>

using namespace mavros;

// Eigen based functions

//! +PI rotation around X (Roll) axis give us ROS or FCU representation
static Eigen::Quaterniond FRAME_ROTATE_Q;

//! Transform for vector3
//static const Eigen::Transform<double, 3, Eigen::Affine> FRAME_TRANSFORM_VECTOR3(FRAME_ROTATE_Q);


Eigen::Quaterniond UAS::transform_frame(const Eigen::Quaterniond &q, const UAS::TRANSFORM_TYPE &transform)
{
	switch(transform){
		case BODY_TO_ENU:{
			FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
		default:{
			FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
	}

	return FRAME_ROTATE_Q * q * FRAME_ROTATE_Q.inverse();
}

Eigen::Vector3d UAS::transform_frame(const Eigen::Vector3d &vec, const UAS::TRANSFORM_TYPE &transform)
{
	switch(transform){
		case BODY_TO_ENU:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
		default:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
	}

	Eigen::Transform<double, 3, Eigen::Affine> FRAME_TRANSFORM_VECTOR3(FRAME_ROTATE_Q);
	return FRAME_TRANSFORM_VECTOR3 * vec;
}

UAS::Covariance3d UAS::transform_frame(const Covariance3d &cov, const UAS::TRANSFORM_TYPE &transform)
{
	switch(transform){
		case BODY_TO_ENU:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
		default:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
	}
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	// code from imu_transformer tf2_sensor_msgs.h
	//cov_out = FRAME_ROTATE_Q * cov_in * FRAME_ROTATE_Q.inverse();
	// from comments on github about tf2_sensor_msgs.h
	cov_out = cov_in * FRAME_ROTATE_Q;
	return cov_out_;
}

UAS::Covariance6d UAS::transform_frame(const Covariance6d &cov, const UAS::TRANSFORM_TYPE &transform)
{
	switch(transform){
		case BODY_TO_ENU:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
		default:{
			Eigen::Quaterniond FRAME_ROTATE_Q = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
			break;
		}
	}
	Covariance6d cov_out_;
	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	//! @todo implement me!!!
	ROS_ASSERT(false);
	return cov_out_;
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Move link along Cartesian direction
*/

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

MoveRelative::MoveRelative(const std::string& name, const solvers::PlannerInterfacePtr& planner)
   : PropagatingEitherWay(name)
   , planner_(planner)
{
	auto& p = properties();
	p.declare<double>("timeout", 10.0, "planning timeout");
	p.declare<std::string>("marker_ns", "", "marker namespace");
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("link", "", "link to move (default is tip of jmg)");
	p.declare<double>("min_distance", -1.0, "minimum distance to move");
	p.declare<double>("max_distance", 0.0, "maximum distance to move");

	p.declare<geometry_msgs::TwistStamped>("twist", "Cartesian twist transform");
	p.declare<geometry_msgs::Vector3Stamped>("direction", "Cartesian translation direction");
	p.declare<std::map<std::string, double>>("joints", "Relative joint space goal");

	p.declare<moveit_msgs::Constraints>("path_constraints", moveit_msgs::Constraints(),
	                                    "constraints to maintain during trajectory");
}

void MoveRelative::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	PropagatingEitherWay::init(robot_model);
	planner_->init(robot_model);
}

bool MoveRelative::compute(const InterfaceState &state, planning_scene::PlanningScenePtr& scene,
                           SubTrajectory &trajectory, Direction dir) {
	scene = state.scene()->diff();
	assert(scene->getRobotModel());

	const auto& props = properties();
	double timeout = props.get<double>("timeout");
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getJointModelGroup(group);
	if (!jmg) {
		ROS_WARN_STREAM_NAMED("MoveRelative", "Invalid joint model group: " << group);
		return false;
	}

	// only allow single target
	size_t count_goals = props.countDefined({"twist", "direction", "joints"});
	if (count_goals != 1) {
		if (count_goals == 0) ROS_WARN_NAMED("MoveRelative", "No goal defined");
		else ROS_WARN_NAMED("MoveRelative", "Cannot plan to multiple goals");
		return false;
	}

	double max_distance = props.get<double>("max_distance");
	double min_distance = props.get<double>("min_distance");
	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");

	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;

	boost::any goal = props.get("joints");
	if (!goal.empty()) {
		const auto& accepted = jmg->getJointModels();
		auto& robot_state = scene->getCurrentStateNonConst();
		const auto& joints = boost::any_cast<std::map<std::string, double>>(goal);
		for (auto j : joints) {
			int index = robot_state.getRobotModel()->getVariableIndex(j.first);
			auto jm = scene->getRobotModel()->getJointModel(index);
			if (std::find(accepted.begin(), accepted.end(), jm) == accepted.end()) {
				ROS_WARN_STREAM_NAMED("MoveRelative", "Ignoring joint " << jm->getName() << " that is not part of group " << group);
				continue;
			}
			robot_state.setVariablePosition(index, robot_state.getVariablePosition(index) + j.second);
			robot_state.enforceBounds(jm);
		}
		robot_state.update();
		success = planner_->plan(state.scene(), scene, jmg, timeout, robot_trajectory, path_constraints);
	} else {
		// Cartesian targets require the link name
		// TODO: use ik_frame property as in ComputeIK
		std::string link_name = props.get<std::string>("link");
		const moveit::core::LinkModel* link;
		if (link_name.empty())
			link_name = solvers::getEndEffectorLink(jmg);
		if (link_name.empty() || !(link = scene->getRobotModel()->getLinkModel(link_name))) {
			ROS_WARN_STREAM_NAMED("MoveRelative", "No or invalid link name specified: " << link_name);
			return false;
		}

		bool use_rotation_distance = false;  // measure achieved distance as rotation?
		Eigen::Vector3d linear;  // linear translation
		Eigen::Vector3d angular;  // angular rotation
		double linear_norm = 0.0, angular_norm = 0.0;

		Eigen::Affine3d target_eigen;
		Eigen::Affine3d link_pose = scene->getFrameTransform(link_name);  // take a copy here, pose will change on success

		boost::any goal = props.get("twist");
		if (!goal.empty()) {
			const geometry_msgs::TwistStamped& target = boost::any_cast<geometry_msgs::TwistStamped>(goal);
			const Eigen::Affine3d& frame_pose = scene->getFrameTransform(target.header.frame_id);
			tf::vectorMsgToEigen(target.twist.linear, linear);
			tf::vectorMsgToEigen(target.twist.angular, angular);

			linear_norm = linear.norm();
			angular_norm = angular.norm();
			if (angular_norm > std::numeric_limits<double>::epsilon())
				angular /= angular_norm;  // normalize angular
			use_rotation_distance = linear_norm < std::numeric_limits<double>::epsilon();

			// use max distance?
			if (max_distance > 0.0) {
				double scale;
				if (!use_rotation_distance)  // non-zero linear motion defines distance
					scale = max_distance / linear_norm;
				else if (angular_norm > std::numeric_limits<double>::epsilon())
					scale = max_distance / angular_norm;
				linear *= scale;
				linear_norm *= scale;
				angular_norm *= scale;
			}

			// invert direction?
			if (dir == BACKWARD) {
				linear *= -1.0;
				angular *= -1.0;
			}

			// compute absolute transform for link
			linear = frame_pose.linear() * linear;
			angular = frame_pose.linear() * angular;
			target_eigen = link_pose;
			target_eigen.linear() = target_eigen.linear() * Eigen::AngleAxisd(angular_norm, link_pose.linear().transpose() * angular);
			target_eigen.translation() += linear;
		}

		goal = props.get("direction");
		if (!goal.empty()) {
			const geometry_msgs::Vector3Stamped& target = boost::any_cast<geometry_msgs::Vector3Stamped>(goal);
			const Eigen::Affine3d& frame_pose = scene->getFrameTransform(target.header.frame_id);
			tf::vectorMsgToEigen(target.vector, linear);

			// use max distance?
			if (max_distance > 0.0) {
				linear.normalize();
				linear *= max_distance;
			}
			linear_norm = linear.norm();

			// invert direction?
			if (dir == BACKWARD)
				linear *= -1.0;

			// compute absolute transform for link
			linear = frame_pose.linear() * linear;
			target_eigen = link_pose;
			target_eigen.translation() += linear;
		}

		success = planner_->plan(state.scene(), *link, target_eigen, jmg, timeout, robot_trajectory, path_constraints);

		// min_distance reached?
		if (min_distance > 0.0) {
			double distance = 0.0;
			if (robot_trajectory && robot_trajectory->getWayPointCount() > 0) {
				const robot_state::RobotState& reached_state = robot_trajectory->getLastWayPoint();
				Eigen::Affine3d reached_pose = reached_state.getGlobalLinkTransform(link);
				if (use_rotation_distance) {
					Eigen::AngleAxisd rotation(reached_pose.linear() * link_pose.linear().transpose());
					distance = rotation.angle();
				} else
					distance = (reached_pose.translation() - link_pose.translation()).norm();
			}
			success = distance >= min_distance;
			if (!success) {
				char msg[100];
				snprintf(msg, sizeof(msg), "min_distance not reached (%.3g < %.3g)", distance, min_distance);
				trajectory.setName(msg);
			}
		} else if (min_distance == 0.0) { // if min_distance is zero, we succeed in any case
			success = true;
		}

		// add an arrow marker
		visualization_msgs::Marker m;
		// +1 TODO: make "marker" a common property of all stages. However, I would stick with "marker_ns"
		m.ns = props.get<std::string>("marker_ns");
		if (!m.ns.empty()) {
			m.header.frame_id = scene->getPlanningFrame();
			if (linear_norm > 1e-3) {
				// +1 TODO: arrow could be split into "valid" and "invalid" part (as red cylinder)
				rviz_marker_tools::setColor(m.color, success ? rviz_marker_tools::LIME_GREEN
				                                             : rviz_marker_tools::RED);
				rviz_marker_tools::makeArrow(m, linear_norm);
				auto quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), linear);
				Eigen::Vector3d pos(link_pose.translation());
				if (dir == BACKWARD)  {
					// flip arrow direction
					quat = quat * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
					// arrow tip at goal_pose
					pos += quat * Eigen::Vector3d(-linear_norm, 0, 0);
				}
				tf::pointEigenToMsg(pos, m.pose.position);
				tf::quaternionEigenToMsg(quat, m.pose.orientation);
				trajectory.markers().push_back(m);
			}
		}
	}

	// store result
	if (robot_trajectory && (success || storeFailures())) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == BACKWARD) robot_trajectory->reverse();
		trajectory.setTrajectory(robot_trajectory);
	}

	return success;
}

bool MoveRelative::computeForward(const InterfaceState &from)
{
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	bool success = compute(from, to, trajectory, FORWARD);
	if (!success) trajectory.setCost(std::numeric_limits<double>::infinity());
	sendForward(from, InterfaceState(to), std::move(trajectory));
	return success;
}

bool MoveRelative::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from;
	SubTrajectory trajectory;

	bool success = compute(to, from, trajectory, BACKWARD);
	if (!success) trajectory.setCost(std::numeric_limits<double>::infinity());
	sendBackward(InterfaceState(from), to, std::move(trajectory));
	return success;
}

} } }

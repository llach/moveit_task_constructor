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

/* Authors: Luca Lach
   Desc:    Set controller name
*/
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/stages/set_controller.h>
#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
namespace stages {

SetController::SetController(const std::string& name) : PropagatingEitherWay(name) {}

void SetController::addControllerName(const std::string& controller_name) {
	std_msgs::String s;
	s.data = controller_name;
	controller_names_.push_back(s);
}

void SetController::computeForward(const InterfaceState& from) {
	sendForward(from, InterfaceState(from.scene()->diff()), compute());
}

void SetController::computeBackward(const InterfaceState& to) {
	sendBackward(InterfaceState(to.scene()->diff()), to, compute());
}

SubTrajectory SetController::compute() const {
	SubTrajectory result;
	result.setControllerNames(controller_names_);

	return result;
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit

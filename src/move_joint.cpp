//
// Created by think on 2026/7/2.
//

#include "move_joint.hpp"

namespace rocos {
MoveJoint::~MoveJoint() {}
Result MoveJoint::GenerateRef(Reference& ref_out) {}
bool MoveJoint::Reset() {}
bool MoveJoint::supportsPause() const {}
bool MoveJoint::supportsResume() const {}
bool MoveJoint::supportsStop() const {}
Result MoveJoint::Pause() {}
Result MoveJoint::Resume() {}
Result MoveJoint::Stop() {}
Result MoveJoint::Update() {}
} // rocos
// openfield_scene_graph.h — the DSR side of openfield_concept: the "room" node and the robot->room RT.
//
// ★ THE CONTRACT IS room_concept's, BYTE FOR BYTE, and on purpose: controller, residual_concept and
// every object agent find the world frame as the node of TYPE "room" NAMED "room" and read the robot
// pose through the RT edge robot->room (active_inference/field_concept.md §1.2). Matching it exactly
// is what lets openfield_concept replace room_concept with a config change in the consumers and no code
// change. The edge writer is a port of RoomSceneGraph::write_robot_room_rt
// (room_concept/src/room_scene_graph.cpp); read the comments there before changing any index or sign —
// each one records a bug that already cost a measurement to find:
//   * the edge is parent=ROBOT, child=ROOM, so it stores T_robot<-room (the INVERSE of the pose), and the
//     covariance goes through the inversion Jacobian;
//   * rt_covariance is a 6x6 row-major SE(3) block [x,y,z,rx,ry,rz]: SE(2) goes to indices 0, 1, 5;
//   * the ring twist is the CHILD's (the room's) motion in the child's axes = -Ad(v_body, w), AXIS order;
//   * the legacy rt_translation_velocity pair is the robot's body twist in ARRAY order [adv, side, 0],
//     and it is written BEFORE the timestamped RT block ("ORDER IS LOAD-BEARING").
#pragma once

#include <genericworker.h>   // DSR API + generated node/attr type tags

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <vector>

namespace rc::openfield
{

struct PoseSample
{
    double x = 0, y = 0, theta = 0;                       // field<-robot, see openfield_ekf.h
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();    // over [x, y, theta]
    float adv = 0.f, side = 0.f, rot = 0.f;               // body twist (m/s, m/s, rad/s)
    Eigen::Vector3f vel_var{0.f, 0.f, 0.f};               // variances of [adv, side, rot]
    std::uint64_t timestamp_ms = 0;                       // WALL epoch ms of validity
};

class OpenFieldSceneGraph
{
public:
    OpenFieldSceneGraph(std::shared_ptr<DSR::DSRGraph> G, DSR::RT_API *rt_api);

    // Create the room node, or ADOPT an existing one (persisted graph / previous run) and make sure it
    // carries the polygon. Idempotent. Returns true once the room exists.
    bool ensure_room(const std::vector<Eigen::Vector2f> &polygon, float room_height);
    [[nodiscard]] bool has_room() const { return room_id_ != 0; }
    [[nodiscard]] std::uint64_t robot_id();

    // Write the robot->room RT (pose, covariance, twist). Refuses non-finite input.
    bool publish(const PoseSample &s);

private:
    std::shared_ptr<DSR::DSRGraph> G_;
    DSR::RT_API *rt_api_ = nullptr;
    std::uint64_t robot_id_ = 0;
    std::uint64_t room_id_ = 0;
};

} // namespace rc::openfield

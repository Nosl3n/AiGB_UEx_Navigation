/*
 *    Copyright (C) 2026 by RoboLab at the University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

// openfield_concept — robot localization in an OPEN FIELD; the room_concept replacement for
// agricultural worlds. GPS (bridge, Ice) + IMU (media plane) + wheel odometry (robot node) are fused
// by an SE(2) EKF (openfield_ekf.h), and the result is published with room_concept's exact DSR
// contract: a "room" node carrying the field polygon and the robot->room RT edge.

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include "openfield_ekf.h"
#include "openfield_scene_graph.h"

#include "../../common/agent_presence_coordinator/agent_presence_coordinator.h"
#include "../../common/concept_presence/concept_presence.h"
#include "../../common/media_transport/media_transport.h"

#include <atomic>
#include <deque>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <fps/fps.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();

public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

private:
	bool startup_check_flag = false;

	// ── Config ───────────────────────────────────────────────────────────────────────────────────
	struct Config
	{
		rc::openfield::EkfParams ekf;
		std::vector<double> poly_x, poly_y;        // field boundary, local metres (x east, y north)
		std::vector<double> poly_lat, poly_lon;    // ...or as GPS coordinates (degrees)
		float room_height = 5.0f;                  // obstacle-cloud ceiling published on the room node
		int yaw_update_period_ms = 100;            // IMU absolute-yaw correction rate (sim ms)
		bool use_imu_yaw = true;
		int gps_poll_period_ms = 50;
		int imu_stall_timeout_ms = 2000;           // primary-input gate (rc::stream)
		std::string imu_node = "imu";
		std::string csv_path = "tmp/openfield_pose.csv";
	} cfg_;
	void load_config();

	// ── Presence (shared protocol, common/concept_presence) ──────────────────────────────────────
	AgentPresenceCoordinator presence_coordinator_;
	rc::presence::ConceptProtocol presence_protocol_;
	std::atomic<bool> shutting_down_{false};
	bool owned_nodes_cleaned_ = false;
	void waiting_enter()   { presence_coordinator_.waiting_enter(); }
	void waiting_loop()    { presence_coordinator_.waiting_loop(); }
	void operating_enter() { presence_coordinator_.operating_enter(); }
	void operating_loop()  { presence_coordinator_.operating_loop(); }
	void degraded_enter()  { presence_coordinator_.degraded_enter(); }
	void degraded_loop()   { presence_coordinator_.degraded_loop(); }
	void request_shutdown();
	void terminal_shutdown();

	// ── Inputs ───────────────────────────────────────────────────────────────────────────────────
	// IMU (media plane): the primary input, and the clock the filter is propagated on.
	std::unique_ptr<rc::media::ImuSubscriber> imu_sub_;
	std::int64_t last_imu_try_ms_ = 0;
	std::int64_t last_imu_recv_ms_ = -1;       // wall ms of the last frame received (for the gate)
	std::int64_t last_imu_clock_ms_ = 0;       // sim ms (or wall ms on hardware) of the last propagated frame
	std::int64_t last_imu_stamp_ms_ = 0;       // wall epoch ms of the last frame (the RT edge timestamp)
	std::int64_t last_yaw_update_clock_ms_ = 0;
	double last_gyro_z_ = 0.0;
	std::optional<double> last_imu_yaw_;
	void pump_imu();
	[[nodiscard]] std::int64_t imu_age_ms() const;

	// Wheel odometry (robot node attributes written by robot_concept).
	std::uint64_t last_odom_ts_ = 0;
	float odom_adv_ = 0.f, odom_side_ = 0.f, odom_rot_ = 0.f;
	Eigen::Vector3f odom_var_{0.f, 0.f, 0.f};
	void read_odometry();

	// GPS (bridge, Ice).
	std::int64_t last_gps_poll_ms_ = 0;
	std::optional<Eigen::Vector2d> last_gps_xy_;
	std::uint64_t gps_fixes_ = 0;
	void poll_gps();

	// ── Estimation and publication ───────────────────────────────────────────────────────────────
	rc::openfield::OpenFieldEkf ekf_;
	std::unique_ptr<DSR::RT_API> rt_api_;
	std::unique_ptr<rc::openfield::OpenFieldSceneGraph> scene_graph_;
	std::vector<Eigen::Vector2f> field_polygon_;
	bool resolve_field_polygon();              // needs a GPS fix when the boundary is given in lat/lon
	void publish_pose();

	// ── Diagnostics ──────────────────────────────────────────────────────────────────────────────
	std::ofstream csv_;
	// Published poses by timestamp, so the ground truth is graded at ITS OWN instant. It arrives at the
	// base's 10 Hz; comparing it with the newest estimate instead charged up to 100 ms of motion to the
	// filter (2.9 deg at 0.5 rad/s), which is the comparison's error, not the localizer's.
	std::deque<std::pair<std::int64_t, Eigen::Vector3d>> pose_history_;
	[[nodiscard]] std::optional<Eigen::Vector3d> pose_at(std::int64_t stamp_ms) const;
	std::int64_t last_status_ms_ = 0;
	std::uint64_t published_ = 0;
	void log_status();
	FPSCounter fps_;

signals:
	void presenceReady();
	void presenceLost();
};

#endif

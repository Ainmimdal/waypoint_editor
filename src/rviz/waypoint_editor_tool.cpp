#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_common/display_context.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QString>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <future>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "waypoint_editor/io/waypoint_csv.hpp"
#include "waypoint_editor/io/waypoint_yaml.hpp"
#include "waypoint_editor/rviz/waypoint_editor_tool.hpp"

using namespace std::placeholders;

namespace waypoint_editor
{

namespace
{

std::string ExpandUserPath(std::string path)
{
    if (!path.empty() && path[0] == '~') {
        const char *home = std::getenv("HOME");
        if (home != nullptr) {
            path.replace(0, 1, home);
        }
    }
    return path;
}

double YawFromPose(const geometry_msgs::msg::Pose &pose)
{
    const auto &q = pose.orientation;
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string Trim(const std::string &input)
{
    const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) { return std::isspace(ch); });
    const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) { return std::isspace(ch); }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

}  // namespace

WaypointEditorTool::WaypointEditorTool() : rviz_default_plugins::tools::PoseTool() {}
WaypointEditorTool::~WaypointEditorTool()
{
    if (backend_executor_) {
        backend_executor_->cancel();
    }
    if (backend_thread_.joinable()) {
        backend_thread_.join();
    }
}

void WaypointEditorTool::onInitialize()
{
    PoseTool::onInitialize();

    setName("Add Waypoint");

    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    waypoint_file_ = ExpandUserPath(nh_->declare_parameter<std::string>("waypoint_file", ""));
    maps_directory_ = ExpandUserPath(nh_->declare_parameter<std::string>("maps_directory", "~/sahabat_ws/maps"));
    map_id_ = nh_->declare_parameter<std::string>("map_id", "rdlfront");
    const auto waypoint_sets_directory = ExpandUserPath(nh_->declare_parameter<std::string>("waypoint_sets_directory", ""));
    backend_mode_ = nh_->declare_parameter<std::string>("waypoint_backend", "local");
    map_directory_ = std::filesystem::path(maps_directory_);
    if (waypoint_file_.empty()) {
        waypoint_file_ = (map_directory_ / (map_id_ + "_waypoints.yaml")).string();
    }
    waypoint_sets_directory_ = waypoint_sets_directory.empty()
        ? map_directory_ / "waypoint_sets" / map_id_
        : std::filesystem::path(waypoint_sets_directory);
    legacy_waypoint_sets_directory_ = map_directory_ / map_id_ / "waypoint_sets";
    waypoint_index_path_ = waypoint_sets_directory_ / "index.yaml";
    auto_pose_topic_ = nh_->declare_parameter<std::string>("auto_pose_topic", "amcl_pose");
    auto_pose_type_  = nh_->declare_parameter<std::string>("auto_pose_type", "geometry_msgs/msg/PoseWithCovarianceStamped");
    auto_min_distance_m_ = nh_->declare_parameter<double>("auto_min_distance", 1.0);
    marker_size_ = nh_->declare_parameter<double>("waypoint_marker_size", 0.25);
    param_cb_handle_ = nh_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            std::string requested_map_id;
            for (const auto &p : params) {
                if (p.get_name() == "auto_pose_topic") {
                    auto_pose_topic_ = p.as_string();
                } else if (p.get_name() == "auto_pose_type") {
                    const auto type = p.as_string();
                    if (type == "geometry_msgs/msg/PoseWithCovarianceStamped" || type == "PoseWithCovarianceStamped" ||
                        type == "geometry_msgs/msg/PoseStamped" || type == "PoseStamped") {
                        auto_pose_type_ = type;
                    } else {
                        result.successful = false;
                        result.reason = "Unsupported auto_pose_type";
                    }
                } else if (p.get_name() == "waypoint_marker_size") {
                    const double value = p.as_double();
                    if (value >= 0.10 && value <= 1.00) {
                        marker_size_ = value;
                    } else {
                        result.successful = false;
                        result.reason = "Waypoint marker size must be between 0.10 and 1.00 m";
                    }
                } else if (p.get_name() == "map_id") {
                    requested_map_id = Trim(p.as_string());
                    if (requested_map_id.empty()) {
                        result.successful = false;
                        result.reason = "Map id cannot be empty";
                    }
                }
            }
            if (result.successful &&
                !requested_map_id.empty() &&
                requested_map_id != map_id_)
            {
                map_id_ = requested_map_id;
                waypoint_file_ =
                    (map_directory_ / (map_id_ + "_waypoints.yaml")).string();
                waypoint_sets_directory_ =
                    map_directory_ / "waypoint_sets" / map_id_;
                legacy_waypoint_sets_directory_ =
                    map_directory_ / map_id_ / "waypoint_sets";
                waypoint_index_path_ = waypoint_sets_directory_ / "index.yaml";
                active_set_id_.clear();
                active_set_name_ = "Default";
                active_revision_ = 0;
                std::string error;
                const bool loaded = usingOperatorBackend()
                    ? loadOperatorWaypointSet("", error)
                    : ensureWaypointSets(error) &&
                      loadWaypointSet(activeSetFromIndex(), error);
                if (!loaded) {
                    result.successful = false;
                    result.reason = "Failed to load waypoints for map '" +
                        map_id_ + "': " + error;
                }
            }
            if (result.successful) {
                refreshAutoPoseSubscription();
                updateWaypointMarker();
                publishRangeMetrics();
            }
            return result;
        }
    );

    if (usingOperatorBackend()) {
        backend_node_ = std::make_shared<rclcpp::Node>("waypoint_editor_operator_client");
        backend_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        backend_executor_->add_node(backend_node_);
        backend_thread_ = std::thread([this]() { backend_executor_->spin(); });
        operator_lease_client_ = backend_node_->create_client<sahabat_interfaces::srv::ControlLease>("/operator/control_lease");
        operator_get_waypoints_client_ = backend_node_->create_client<sahabat_interfaces::srv::GetWaypoints>("/operator/waypoints/get");
        operator_save_waypoints_client_ = backend_node_->create_client<sahabat_interfaces::srv::SaveWaypoints>("/operator/waypoints/save");
        operator_get_graph_client_ = backend_node_->create_client<sahabat_interfaces::srv::GetWaypointGraph>("/operator/waypoint_graph/get");
        operator_save_graph_client_ = backend_node_->create_client<sahabat_interfaces::srv::SaveWaypointGraph>("/operator/waypoint_graph/save");
        operator_list_sets_client_ = backend_node_->create_client<sahabat_interfaces::srv::ListWaypointSets>("/operator/waypoint_sets/list");
        operator_manage_set_client_ = backend_node_->create_client<sahabat_interfaces::srv::ManageWaypointSet>("/operator/waypoint_sets/manage");
    }

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server",
        nh_,
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );
    save_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "save_waypoints",
        std::bind(&WaypointEditorTool::handleSaveWaypoints, this, _1, _2)
    );
    undo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "undo_waypoints",
        std::bind(&WaypointEditorTool::handleUndoWaypoints, this, _1, _2)
    );
    redo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "redo_waypoints",
        std::bind(&WaypointEditorTool::handleRedoWaypoints, this, _1, _2)
    );
    clear_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "clear_waypoints",
        std::bind(&WaypointEditorTool::handleClearWaypoints, this, _1, _2)
    );
    load_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "load_waypoints",
        std::bind(&WaypointEditorTool::handleLoadWaypoints, this, _1, _2)
    );
    list_sets_service_ = nh_->create_service<sahabat_interfaces::srv::ListWaypointSets>(
        "waypoint_editor/list_sets",
        std::bind(&WaypointEditorTool::handleListWaypointSets, this, _1, _2)
    );
    manage_set_service_ = nh_->create_service<sahabat_interfaces::srv::ManageWaypointSet>(
        "waypoint_editor/manage_set",
        std::bind(&WaypointEditorTool::handleManageWaypointSet, this, _1, _2)
    );
    get_waypoints_service_ = nh_->create_service<sahabat_interfaces::srv::GetWaypoints>(
        "waypoint_editor/get_waypoints",
        std::bind(&WaypointEditorTool::handleGetWaypoints, this, _1, _2)
    );
    get_graph_service_ = nh_->create_service<sahabat_interfaces::srv::GetWaypointGraph>(
        "waypoint_editor/get_graph",
        std::bind(&WaypointEditorTool::handleGetWaypointGraph, this, _1, _2)
    );
    edit_route_service_ = nh_->create_service<sahabat_interfaces::srv::EditRoute>(
        "waypoint_editor/edit_route",
        std::bind(&WaypointEditorTool::handleEditRoute, this, _1, _2)
    );
    auto_start_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "start_auto_waypoints",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            auto_enabled_ = true;
            res->success = true;
            res->message = "Auto waypoint capture started\n(topic: " + auto_pose_topic_ + ")";
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint capture started");
        }
    );
    auto_stop_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "stop_auto_waypoints",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            auto_enabled_ = false;
            res->success = true;
            res->message = "Auto waypoint capture stopped";
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint capture stopped");
        }
    );

    line_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("waypoint_line", 10);
    total_wp_dist_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("total_wp_dist", 10);
    last_wp_dist_pub_  = nh_->create_publisher<std_msgs::msg::Float64>("last_wp_dist", 10);
    graph_changed_pub_ = nh_->create_publisher<std_msgs::msg::Empty>("waypoint_editor/graph_changed", 10);
    route_point_selected_pub_ = nh_->create_publisher<std_msgs::msg::String>(
        "waypoint_editor/route_point_selected", rclcpp::QoS(10).reliable());
    auto_distance_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "auto_waypoint_min_distance", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            auto_min_distance_m_ = std::max(0.0, msg->data);
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint min distance set to %.3f m", auto_min_distance_m_);
        }
    );
    refreshAutoPoseSubscription();

    std::string error;
    waypoint_sequence_.clear();
    pose_dirty_ = false;
    if (usingOperatorBackend()) {
        if (!loadOperatorWaypointSet("", error)) {
            RCLCPP_WARN(nh_->get_logger(), "Failed to initialize operator waypoint backend: %s", error.c_str());
        }
    } else if (!ensureWaypointSets(error) || !loadWaypointSet(activeSetFromIndex(), error)) {
        RCLCPP_WARN(nh_->get_logger(), "Failed to initialize waypoint sets: %s", error.c_str());
    }
    updateLastDistanceFromWaypoint(0);
    publishRangeMetrics();
}

void WaypointEditorTool::onPoseSet(double x, double y, double theta)
{
    Waypoint wp;
    wp.id = makeWaypointId(add_route_point_armed_ ? "rp" : "wp");
    wp.pose.header.frame_id = "map";
    wp.pose.header.stamp = nh_->now();
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = y;
    wp.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    wp.pose.pose.orientation.x = 0.0;
    wp.pose.pose.orientation.y = 0.0;
    wp.pose.pose.orientation.z = q.z();
    wp.pose.pose.orientation.w = q.w();

    wp.function_command.clear();

    if (add_route_point_armed_) {
        const int segment_index = segmentIndex(active_segment_id_);
        if (segment_index < 0) {
            add_route_point_armed_ = false;
            RCLCPP_WARN(nh_->get_logger(), "No route selected for the new route point");
            deactivate();
            return;
        }
        auto &segment = route_segments_.at(static_cast<std::size_t>(segment_index));
        sahabat_interfaces::msg::Waypoint via;
        via.id = wp.id;
        via.name = via.id;
        via.pose.x = x;
        via.pose.y = y;
        via.pose.theta = theta;
        via.dwell_seconds = 0.0;
        via.enabled = true;
        segment.via_points.push_back(std::move(via));
        add_route_point_armed_ = false;
        updateWaypointMarker();
        publishRangeMetrics();
        graph_changed_pub_->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(
            nh_->get_logger(), "Added route point to segment '%s'",
            active_segment_id_.c_str());
    } else {
        const int new_id = appendWaypointAndRefresh(std::move(wp));
        RCLCPP_INFO(nh_->get_logger(), "Added waypoint %d", new_id);
    }

    deactivate();
}

void WaypointEditorTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    }
    std::vector<std::size_t> route_order;
    const int selected_index = segmentIndex(active_segment_id_);
    if (selected_index >= 0) {
        route_order.push_back(static_cast<std::size_t>(selected_index));
    }
    for (std::size_t i = 0; i < route_segments_.size(); ++i) {
        if (static_cast<int>(i) != selected_index) {
            route_order.push_back(i);
        }
    }
    std::set<std::string> rendered_point_ids;
    for (const std::size_t segment_index : route_order) {
        const auto &segment = route_segments_[segment_index];
        for (std::size_t point_index = 0; point_index < segment.via_points.size(); ++point_index) {
            if (!rendered_point_ids.insert(segment.via_points[point_index].id).second) {
                continue;
            }
            auto marker = createRoutePointMarker(segment_index, point_index);
            server_->insert(marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
        }
    }

    server_->applyChanges();
}

int WaypointEditorTool::appendWaypointAndRefresh(Waypoint wp)
{
    if (wp.id.empty()) {
        wp.id = makeWaypointId("wp");
    }
    if (wp.pose.header.frame_id.empty()) {
        wp.pose.header.frame_id = "map";
    }
    if (wp.pose.header.stamp.sec == 0 && wp.pose.header.stamp.nanosec == 0) {
        wp.pose.header.stamp = nh_->now();
    }

    const int new_id = waypoint_sequence_.appendWaypoint(std::move(wp));
    auto int_marker = createWaypointMarker(new_id);
    server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    server_->applyChanges();
    commitWaypointChanges(new_id);
    return new_id;
}

void WaypointEditorTool::handleAutoPose(const geometry_msgs::msg::PoseStamped &pose_in)
{
    if (!auto_enabled_) {
        return;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    if (!transformToMapFrame(pose_in, pose_map)) {
        return;
    }

    const auto &waypoints = waypoint_sequence_.waypoints();
    if (!waypoints.empty()) {
        const auto &last_pose = waypoints.back().pose.pose.position;
        const double dist = std::hypot(
            pose_map.pose.position.x - last_pose.x,
            pose_map.pose.position.y - last_pose.y
        );
        if (dist < auto_min_distance_m_) {
            return;
        }
    }

    Waypoint wp;
    wp.pose = pose_map;
    wp.function_command.clear();
    appendWaypointAndRefresh(std::move(wp));
    RCLCPP_INFO(nh_->get_logger(), "Auto-added waypoint at (%.2f, %.2f)", pose_map.pose.position.x, pose_map.pose.position.y);
}

bool WaypointEditorTool::transformToMapFrame(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output) const
{
    if (!tf_buffer_) {
        return false;
    }

    output = input;
    if (output.header.frame_id.empty()) {
        output.header.frame_id = "map";
    }
    if (output.header.frame_id == "map") {
        return true;
    }

    try {
        const auto tf = tf_buffer_->lookupTransform("map", output.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(input, output, tf);
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            nh_->get_logger(),
            *nh_->get_clock(),
            5000,
            "Failed to transform from %s to map: %s",
            output.header.frame_id.c_str(),
            ex.what());
        return false;
    }
}

void WaypointEditorTool::refreshAutoPoseSubscription()
{
    auto_pose_sub_.reset();
    const std::string type = auto_pose_type_;

    if (type == "geometry_msgs/msg/PoseWithCovarianceStamped" || type == "PoseWithCovarianceStamped") {
        auto_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            auto_pose_topic_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped pose_in;
                pose_in.header = msg->header;
                pose_in.pose = msg->pose.pose;
                handleAutoPose(pose_in);
            }
        );
        RCLCPP_INFO(nh_->get_logger(), "Auto pose subscription: %s (PoseWithCovarianceStamped)", auto_pose_topic_.c_str());
    } else if (type == "geometry_msgs/msg/PoseStamped" || type == "PoseStamped") {
        auto_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            auto_pose_topic_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                handleAutoPose(*msg);
            }
        );
        RCLCPP_INFO(nh_->get_logger(), "Auto pose subscription: %s (PoseStamped)", auto_pose_topic_.c_str());
    } else {
        RCLCPP_WARN(nh_->get_logger(), "Unsupported auto_pose_type: %s", type.c_str());
    }
}

visualization_msgs::msg::InteractiveMarker WaypointEditorTool::createWaypointMarker(const int id)
{
    const auto & wp = waypoint_sequence_.at(id);
    const double size = std::max(0.10, marker_size_);

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id;
    int_marker.name = std::to_string(id);
    int_marker.description = waypoint_sequence_.at(id).function_command;
    int_marker.scale = std::max(0.30, size * 2.5);
    int_marker.pose.position = wp.pose.pose.position;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = wp.pose.pose.orientation.z;
    int_marker.pose.orientation.w = wp.pose.pose.orientation.w;

    // Position control (sphere)
    visualization_msgs::msg::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker sphere;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.scale.x = size;
        sphere.scale.y = size;
        sphere.scale.z = size;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        pos_control.markers.push_back(sphere);
    }
    int_marker.controls.push_back(pos_control);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_default;
    rot_control_default.name = "rotate_yaw_default";
    rot_control_default.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rot_control_default.orientation.w = 0.7071;
    rot_control_default.orientation.x = 0.0;
    rot_control_default.orientation.y = -0.7071;
    rot_control_default.orientation.z = 0.0;
    rot_control_default.always_visible = true;
    int_marker.controls.push_back(rot_control_default);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_arrow;
    rot_control_arrow.name = "rotate_yaw_arrow";
    rot_control_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    rot_control_arrow.orientation.w = 0.7071;
    rot_control_arrow.orientation.x = 0.0;
    rot_control_arrow.orientation.y = -0.7071;
    rot_control_arrow.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker arrow;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = size;
        arrow.scale.y = size * 0.25;
        arrow.scale.z = size * 0.25;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        rot_control_arrow.markers.push_back(arrow);
    }
    rot_control_arrow.always_visible = true;
    int_marker.controls.push_back(rot_control_arrow);

    // Text control
    visualization_msgs::msg::InteractiveMarkerControl text_control;
    text_control.name = "display_text";
    text_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    {
        visualization_msgs::msg::Marker text;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.scale.z = std::max(0.10, size * 0.55);
        text.color.r = 0.0;
        text.color.g = 0.0;
        text.color.b = 0.0;
        text.color.a = 1.0;
        std::string id_text = "ID:" + std::to_string(id) + "\n" + waypoint_sequence_.at(id).function_command;
        text.text = id_text;
        text.pose.position.x = -size * 0.75;
        text.pose.position.y = -size * 0.75;
        text.pose.position.z = size * 0.75;
        text_control.markers.push_back(text);
    }
    int_marker.controls.push_back(text_control);

    visualization_msgs::msg::InteractiveMarkerControl menu_control;
    menu_control.name              = "menu";
    menu_control.always_visible    = true;
    menu_control.interaction_mode  = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    int_marker.controls.push_back(menu_control);
    
    visualization_msgs::msg::MenuEntry delete_entry;
    delete_entry.id = 1;
    delete_entry.parent_id = 0;
    delete_entry.title = "Delete Waypoint";
    int_marker.menu_entries.push_back(delete_entry);

    visualization_msgs::msg::MenuEntry change_id_entry;
    change_id_entry.id = 2;
    change_id_entry.parent_id = 0;
    change_id_entry.title = "Reorder Waypoint";
    int_marker.menu_entries.push_back(change_id_entry);
    
    visualization_msgs::msg::MenuEntry add_function_command_entry;
    add_function_command_entry.id = 3;
    add_function_command_entry.parent_id = 0;
    add_function_command_entry.title = "Rename Waypoint";
    int_marker.menu_entries.push_back(add_function_command_entry);

    return int_marker;
}

visualization_msgs::msg::InteractiveMarker WaypointEditorTool::createRoutePointMarker(
    std::size_t segment_index, std::size_t point_index)
{
    const auto &segment = route_segments_.at(segment_index);
    const auto &point = segment.via_points.at(point_index);
    const double size = std::max(0.10, marker_size_ * 0.85);
    const bool selected = segment.id == active_segment_id_;

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.name = "route:" + segment.id + ":" + std::to_string(point_index);
    marker.description = point.name.empty() || point.name == point.id
        ? routePointTag(point.id)
        : routePointTag(point.id) + " · " + point.name;
    marker.scale = selected ? std::max(0.60, size * 3.0) : std::max(0.30, size * 2.0);
    marker.pose.position.x = point.pose.x;
    marker.pose.position.y = point.pose.y;
    marker.pose.orientation.z = std::sin(point.pose.theta / 2.0);
    marker.pose.orientation.w = std::cos(point.pose.theta / 2.0);

    visualization_msgs::msg::InteractiveMarkerControl move;
    move.name = "move_route_point";
    move.interaction_mode = selected
        ? visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE
        : visualization_msgs::msg::InteractiveMarkerControl::NONE;
    move.always_visible = true;
    move.orientation.w = 0.7071;
    move.orientation.y = 0.7071;
    if (selected) {
        visualization_msgs::msg::Marker halo;
        halo.type = visualization_msgs::msg::Marker::CYLINDER;
        halo.scale.x = std::max(0.50, size * 2.4);
        halo.scale.y = std::max(0.50, size * 2.4);
        halo.scale.z = 0.035;
        halo.pose.position.z = -0.02;
        halo.color.r = 0.10;
        halo.color.g = 0.85;
        halo.color.b = 1.0;
        halo.color.a = 0.28;
        move.markers.push_back(halo);
    }
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    const double sphere_size = selected ? std::max(0.32, size * 1.45) : size;
    sphere.scale.x = sphere_size;
    sphere.scale.y = sphere_size;
    sphere.scale.z = sphere_size;
    sphere.color.r = 0.15;
    sphere.color.g = 0.80;
    sphere.color.b = 1.0;
    sphere.color.a = selected ? 1.0 : 0.14;
    move.markers.push_back(sphere);
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.scale.z = selected ? std::max(0.30, size * 1.35) : std::max(0.13, size * 0.60);
    text.color.r = 0.0;
    text.color.g = selected ? 0.12 : 0.25;
    text.color.b = selected ? 0.18 : 0.40;
    text.color.a = selected ? 1.0 : 0.10;
    text.text = routePointTag(point.id);
    text.pose.position.z = selected ? std::max(0.30, size * 1.45) : size;
    move.markers.push_back(text);
    marker.controls.push_back(move);

    if (selected) {
        visualization_msgs::msg::InteractiveMarkerControl menu_control;
        menu_control.name = "route_point_menu";
        menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
        menu_control.always_visible = true;
        marker.controls.push_back(menu_control);

        visualization_msgs::msg::MenuEntry delete_entry;
        delete_entry.id = 101;
        delete_entry.title = "Remove Point from This Route";
        marker.menu_entries.push_back(delete_entry);
        visualization_msgs::msg::MenuEntry rename_entry;
        rename_entry.id = 102;
        rename_entry.title = "Rename Shared Route Point";
        marker.menu_entries.push_back(rename_entry);
        visualization_msgs::msg::MenuEntry earlier_entry;
        earlier_entry.id = 103;
        earlier_entry.title = "Move Earlier in This Route";
        marker.menu_entries.push_back(earlier_entry);
        visualization_msgs::msg::MenuEntry later_entry;
        later_entry.id = 104;
        later_entry.title = "Move Later in This Route";
        marker.menu_entries.push_back(later_entry);
    }
    return marker;
}

void WaypointEditorTool::processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb)
{
    if (fb->marker_name.rfind("route:", 0) == 0) {
        const auto last_separator = fb->marker_name.rfind(':');
        if (last_separator == std::string::npos || last_separator <= 6) {
            return;
        }
        const std::string segment_id = fb->marker_name.substr(6, last_separator - 6);
        const int segment_index = segmentIndex(segment_id);
        int point_index = -1;
        try {
            point_index = std::stoi(fb->marker_name.substr(last_separator + 1));
        } catch (const std::exception &) {
            return;
        }
        if (segment_index < 0 || point_index < 0) {
            return;
        }
        auto &segment = route_segments_.at(static_cast<std::size_t>(segment_index));
        if (point_index >= static_cast<int>(segment.via_points.size())) {
            return;
        }
        auto &point = segment.via_points.at(static_cast<std::size_t>(point_index));
        if (
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN ||
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            std_msgs::msg::String selected_point;
            selected_point.data = point.id;
            route_point_selected_pub_->publish(selected_point);
        }
        if (fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            const std::string shared_id = point.id;
            for (auto &route : route_segments_) {
                for (auto &shared_point : route.via_points) {
                    if (shared_point.id == shared_id) {
                        shared_point.pose.x = fb->pose.position.x;
                        shared_point.pose.y = fb->pose.position.y;
                        shared_point.pose.theta = YawFromPose(fb->pose);
                    }
                }
            }
            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            publishRangeMetrics();
        } else if (
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
            fb->menu_entry_id == 101)
        {
            segment.via_points.erase(
                segment.via_points.begin() + static_cast<std::ptrdiff_t>(point_index));
            updateWaypointMarker();
            publishRangeMetrics();
            graph_changed_pub_->publish(std_msgs::msg::Empty());
        } else if (
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
            fb->menu_entry_id == 102)
        {
            bool ok = false;
            const QString name = QInputDialog::getText(
                nullptr,
                tr("Rename Route Point"),
                tr("Route point name:"),
                QLineEdit::Normal,
                QString::fromStdString(point.name),
                &ok);
            if (ok && !name.trimmed().isEmpty()) {
                const std::string shared_id = point.id;
                for (auto &route : route_segments_) {
                    for (auto &shared_point : route.via_points) {
                        if (shared_point.id == shared_id) {
                            shared_point.name = name.trimmed().toStdString();
                        }
                    }
                }
                updateWaypointMarker();
                graph_changed_pub_->publish(std_msgs::msg::Empty());
            }
        } else if (
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
            fb->menu_entry_id == 103 && point_index > 0)
        {
            std::swap(
                segment.via_points[static_cast<std::size_t>(point_index)],
                segment.via_points[static_cast<std::size_t>(point_index - 1)]);
            updateWaypointMarker();
            publishRangeMetrics();
            graph_changed_pub_->publish(std_msgs::msg::Empty());
        } else if (
            fb->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT &&
            fb->menu_entry_id == 104 &&
            point_index + 1 < static_cast<int>(segment.via_points.size()))
        {
            std::swap(
                segment.via_points[static_cast<std::size_t>(point_index)],
                segment.via_points[static_cast<std::size_t>(point_index + 1)]);
            updateWaypointMarker();
            publishRangeMetrics();
            graph_changed_pub_->publish(std_msgs::msg::Empty());
        }
        return;
    }

    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) {
        return;
    }

    switch (fb->event_type)
    {
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            geometry_msgs::msg::Pose new_pose = fb->pose;
            new_pose.orientation.x = 0.0;
            new_pose.orientation.y = 0.0;
            new_pose.orientation.z = fb->pose.orientation.z;
            new_pose.orientation.w = fb->pose.orientation.w;
            waypoint_sequence_.updatePose(id, new_pose);
            pose_dirty_ = true;

            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            updateLastDistanceFromWaypoint(id);
            publishRangeMetrics();
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
        {
            if (pose_dirty_) {
                commitWaypointChanges(id);
            }
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        {
            processMenuControl(fb);
            break;
        }

        default:
            break;
    }
}

void WaypointEditorTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) { return; }

    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) { return; }

    switch (fb->menu_entry_id) {
      
        // Delete Waypoint
        case 1:
            removeSegmentsForWaypoint(
                waypoint_sequence_.at(static_cast<std::size_t>(id)).id);
            waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
            updateWaypointMarker();
            commitWaypointChanges(id);
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", id);
            break;

        // Reorder waypoint within the tour.
        case 2:
        {
         bool ok = false;
            QString current = QString::fromStdString(std::to_string(id));
            QString text = QInputDialog::getText(
                nullptr,
                tr("Reorder Waypoint"),
                tr("Move waypoint %1 to index:").arg(id),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                int insert_id = text.toInt();
                if (0 <= insert_id && insert_id < static_cast<int>(waypoint_sequence_.size())) {
                    Waypoint waypoint = waypoint_sequence_.at(static_cast<std::size_t>(id));
                    waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
                    waypoint_sequence_.insertWaypoint(static_cast<std::size_t>(insert_id), std::move(waypoint));
                    updateWaypointMarker();
                    commitWaypointChanges(insert_id);
                    RCLCPP_INFO(nh_->get_logger(), "Changed waypoint id %d to %d", id, insert_id);
                } else {
                    const int max_index = std::max(0, static_cast<int>(waypoint_sequence_.size()) - 1);
                    QMessageBox::warning(
                        nullptr,
                        tr("Invalid Waypoint ID"),
                        tr("Waypoint ID %1 is out of range.\n"
                        "Please enter a value between %2 and %3.")
                        .arg(insert_id)
                        .arg(0)
                        .arg(max_index)
                    );
                }
            }
        }
            break;

        // Rename waypoint.
        case 3:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Rename Waypoint"),
                tr("Waypoint name:"),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command = text.toStdString();
                updateWaypointMarker();
                commitWaypointChanges(id);
                RCLCPP_INFO(nh_->get_logger(), "Updated command of waypoint %d to '%s'", id, waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command.c_str());
            }
        }
            break;

      default:
            break;
    }
}

double WaypointEditorTool::computeSegmentDistance(std::size_t first, std::size_t second) const
{
    const auto &waypoints = waypoint_sequence_.waypoints();
    if (first >= waypoints.size() || second >= waypoints.size()) {
        return 0.0;
    }
    const auto &p0 = waypoints[first].pose.pose.position;
    const auto &p1 = waypoints[second].pose.pose.position;
    return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

void WaypointEditorTool::updateLastDistanceFromWaypoint(int waypoint_index)
{
    const auto size = waypoint_sequence_.size();
    if (size < 2) {
        last_displayed_distance_ = 0.0;
        return;
    }

    const int max_index = static_cast<int>(size) - 1;
    const int clamped = std::max(0, std::min(waypoint_index, max_index));
    const std::size_t idx = static_cast<std::size_t>(clamped);

    if (idx > 0) {
        last_displayed_distance_ = computeSegmentDistance(idx - 1, idx);
    } else if (idx + 1 < size) {
        last_displayed_distance_ = computeSegmentDistance(idx, idx + 1);
    } else {
        last_displayed_distance_ = computeSegmentDistance(size - 2, size - 1);
    }
}

void WaypointEditorTool::publishLineMarker()
{
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = nh_->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    line_pub_->publish(clear);
    const auto &waypoints = waypoint_sequence_.waypoints();
    if (route_segments_.empty()) {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = nh_->now();
        line.ns = "waypoint_lines";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.025f;
        line.color.g = 1.0f;
        line.color.a = 1.0f;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            line.points.push_back(waypoints[i - 1].pose.pose.position);
            line.points.push_back(waypoints[i].pose.pose.position);
        }
        line_pub_->publish(line);
        return;
    }

    std::vector<std::size_t> order;
    order.reserve(route_segments_.size());
    for (std::size_t i = 0; i < route_segments_.size(); ++i) {
        if (route_segments_[i].id != active_segment_id_) {
            order.push_back(i);
        }
    }
    const int selected_index = segmentIndex(active_segment_id_);
    if (selected_index >= 0) {
        order.push_back(static_cast<std::size_t>(selected_index));
    }

    for (const std::size_t segment_index : order) {
        const auto &segment = route_segments_[segment_index];
        const int from_index = waypointIndexById(segment.from_waypoint_id);
        const int to_index = waypointIndexById(segment.to_waypoint_id);
        if (from_index < 0 || to_index < 0 || !segment.enabled) {
            continue;
        }
        const bool selected = segment.id == active_segment_id_;
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = nh_->now();
        line.ns = "waypoint_lines";
        line.id = static_cast<int>(segment_index) + 1;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = selected ? 0.14f : 0.018f;
        line.color.r = selected ? 0.05f : 0.45f;
        line.color.g = selected ? 0.82f : 0.65f;
        line.color.b = 1.0f;
        line.color.a = selected ? 1.0f : 0.06f;
        line.points.push_back(
            waypoints.at(static_cast<std::size_t>(from_index)).pose.pose.position);
        for (const auto &via : segment.via_points) {
            geometry_msgs::msg::Point point;
            point.x = via.pose.x;
            point.y = via.pose.y;
            point.z = 0.03;
            line.points.push_back(point);
        }
        line.points.push_back(
            waypoints.at(static_cast<std::size_t>(to_index)).pose.pose.position);
        line_pub_->publish(line);
    }
}

void WaypointEditorTool::publishTotalWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = waypoint_sequence_.totalDistance();
    total_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishLastWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = last_displayed_distance_;
    last_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishRangeMetrics()
{
    publishLineMarker();
    publishTotalWpsDist();
    publishLastWpsDist();
}

void WaypointEditorTool::commitWaypointChanges(int waypoint_index, bool snapshot_history)
{
    if (snapshot_history) {
        waypoint_sequence_.snapshotHistory();
    }
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(waypoint_index);
    publishRangeMetrics();
}

bool WaypointEditorTool::isValidWaypointId(int id) const
{
    return id >= 0 && id < static_cast<int>(waypoint_sequence_.size());
}

std::string WaypointEditorTool::makeWaypointId(const std::string &prefix) const
{
    int suffix = 1;
    while (true) {
        std::string suffix_text = std::to_string(suffix++);
        if (prefix == "rp" && suffix_text.size() < 3) {
            suffix_text.insert(0, 3 - suffix_text.size(), '0');
        }
        const std::string candidate = prefix + "-" + suffix_text;
        bool used = std::any_of(
            waypoint_sequence_.waypoints().begin(),
            waypoint_sequence_.waypoints().end(),
            [&candidate](const Waypoint &waypoint) {
                return waypoint.id == candidate;
            });
        if (!used) {
            for (const auto &segment : route_segments_) {
                used = used || segment.id == candidate;
                used = used || std::any_of(
                    segment.via_points.begin(), segment.via_points.end(),
                    [&candidate](const sahabat_interfaces::msg::Waypoint &point) {
                        return point.id == candidate;
                    });
            }
        }
        if (!used) {
            return candidate;
        }
    }
}

int WaypointEditorTool::segmentIndex(const std::string &segment_id) const
{
    for (std::size_t i = 0; i < route_segments_.size(); ++i) {
        if (route_segments_[i].id == segment_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::string WaypointEditorTool::routePointTag(const std::string &point_id) const
{
    return point_id.empty() ? "rp-???" : point_id;
}

int WaypointEditorTool::waypointIndexById(const std::string &waypoint_id) const
{
    for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        if (waypoint_sequence_.at(i).id == waypoint_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void WaypointEditorTool::removeSegmentsForWaypoint(const std::string &waypoint_id)
{
    route_segments_.erase(
        std::remove_if(
            route_segments_.begin(), route_segments_.end(),
            [&waypoint_id](const sahabat_interfaces::msg::RouteSegment &segment) {
                return segment.from_waypoint_id == waypoint_id ||
                       segment.to_waypoint_id == waypoint_id;
            }),
        route_segments_.end());
    if (segmentIndex(active_segment_id_) < 0) {
        active_segment_id_.clear();
        add_route_point_armed_ = false;
    }
}

void WaypointEditorTool::refreshRouteVisualization()
{
    updateWaypointMarker();
    publishRangeMetrics();
}

bool WaypointEditorTool::usingOperatorBackend() const
{
    return backend_mode_ == "operator" || backend_mode_ == "live" || backend_mode_ == "backend";
}

bool WaypointEditorTool::acquireOperatorLease(std::string &lease_id, std::string &error)
{
    if (!operator_lease_client_ || !operator_lease_client_->wait_for_service(std::chrono::seconds(2))) {
        error = "Operator control lease service unavailable";
        return false;
    }
    auto req = std::make_shared<sahabat_interfaces::srv::ControlLease::Request>();
    req->action = sahabat_interfaces::srv::ControlLease::Request::ACQUIRE;
    req->client_id = "rviz_waypoint_editor";
    auto future = operator_lease_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        error = "Timed out acquiring operator control lease";
        return false;
    }
    auto response = future.get();
    if (!response->success) {
        error = response->message;
        return false;
    }
    lease_id = response->lease_id;
    return true;
}

void WaypointEditorTool::releaseOperatorLease(const std::string &lease_id)
{
    if (lease_id.empty() || !operator_lease_client_) {
        return;
    }
    auto req = std::make_shared<sahabat_interfaces::srv::ControlLease::Request>();
    req->action = sahabat_interfaces::srv::ControlLease::Request::RELEASE;
    req->client_id = "rviz_waypoint_editor";
    req->lease_id = lease_id;
    operator_lease_client_->async_send_request(req);
}

bool WaypointEditorTool::loadOperatorWaypointSet(const std::string &set_id, std::string &error)
{
    if (!operator_get_graph_client_ || !operator_get_graph_client_->wait_for_service(std::chrono::seconds(2))) {
        error = "Operator waypoint graph load service unavailable";
        return false;
    }
    auto req = std::make_shared<sahabat_interfaces::srv::GetWaypointGraph::Request>();
    req->map_id = map_id_;
    req->set_id = set_id;
    auto future = operator_get_graph_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        error = "Timed out loading operator waypoint graph";
        return false;
    }
    auto response = future.get();
    if (response->set_id.empty()) {
        error = response->message.empty() ? "Operator waypoint load failed" : response->message;
        return false;
    }

    std::vector<Waypoint> loaded;
    for (const auto &item : response->waypoints) {
        Waypoint waypoint;
        waypoint.id = item.id;
        waypoint.pose.header.frame_id = "map";
        waypoint.pose.header.stamp = nh_->now();
        waypoint.pose.pose.position.x = item.pose.x;
        waypoint.pose.pose.position.y = item.pose.y;
        waypoint.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, item.pose.theta);
        waypoint.pose.pose.orientation.x = 0.0;
        waypoint.pose.pose.orientation.y = 0.0;
        waypoint.pose.pose.orientation.z = q.z();
        waypoint.pose.pose.orientation.w = q.w();
        waypoint.function_command = item.name;
        loaded.emplace_back(std::move(waypoint));
    }
    for (std::size_t i = 0; i < loaded.size(); ++i) {
        if (loaded[i].id.empty()) {
            loaded[i].id = std::to_string(i);
        }
    }
    route_segments_ = response->segments;
    active_segment_id_ = route_segments_.empty() ? "" : route_segments_.front().id;
    add_route_point_armed_ = false;

    active_set_id_ = response->set_id;
    active_set_name_ = response->set_id;
    active_revision_ = static_cast<int>(response->revision);

    if (operator_list_sets_client_ && operator_list_sets_client_->wait_for_service(std::chrono::milliseconds(500))) {
        auto list_req = std::make_shared<sahabat_interfaces::srv::ListWaypointSets::Request>();
        list_req->map_id = map_id_;
        auto list_future = operator_list_sets_client_->async_send_request(list_req);
        if (list_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto list_response = list_future.get();
            for (const auto &info : list_response->sets) {
                if (info.id == active_set_id_) {
                    active_set_name_ = info.name;
                    break;
                }
            }
        }
    }

    waypoint_sequence_.assign(std::move(loaded));
    waypoint_sequence_.resetHistory();
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    error.clear();
    return true;
}

bool WaypointEditorTool::saveOperatorWaypointSet(std::string &error)
{
    std::string lease_id;
    if (!acquireOperatorLease(lease_id, error)) {
        return false;
    }
    auto release = [this, &lease_id]() { releaseOperatorLease(lease_id); };
    if (!operator_save_graph_client_ || !operator_save_graph_client_->wait_for_service(std::chrono::seconds(2))) {
        error = "Operator waypoint graph save service unavailable";
        release();
        return false;
    }
    auto make_request = [this, &lease_id](uint64_t expected_revision) {
        auto req = std::make_shared<sahabat_interfaces::srv::SaveWaypointGraph::Request>();
        req->map_id = map_id_;
        req->set_id = active_set_id_;
        req->expected_revision = expected_revision;
        req->lease_id = lease_id;
        for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
            const auto &wp = waypoint_sequence_.at(i);
            sahabat_interfaces::msg::Waypoint item;
            item.id = wp.id.empty() ? std::to_string(i) : wp.id;
            item.name = wp.function_command.empty() ? "waypoint_" + std::to_string(i + 1) : wp.function_command;
            item.pose.x = wp.pose.pose.position.x;
            item.pose.y = wp.pose.pose.position.y;
            item.pose.theta = YawFromPose(wp.pose.pose);
            item.dwell_seconds = 0.0;
            item.enabled = true;
            req->waypoints.push_back(item);
        }
        req->segments = route_segments_;
        return req;
    };

    auto req = make_request(static_cast<uint64_t>(std::max(0, active_revision_)));
    auto future = operator_save_graph_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        error = "Timed out saving operator waypoints";
        release();
        return false;
    }
    auto response = future.get();
    if (!response->success && response->revision != static_cast<uint64_t>(std::max(0, active_revision_))) {
        active_revision_ = static_cast<int>(response->revision);
        req = make_request(response->revision);
        future = operator_save_graph_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            error = "Timed out retrying operator waypoint save";
            release();
            return false;
        }
        response = future.get();
    }

    release();
    if (!response->success) {
        active_revision_ = static_cast<int>(response->revision);
        error = response->message;
        return false;
    }
    active_set_id_ = response->set_id;
    active_revision_ = static_cast<int>(response->revision);
    error.clear();
    return true;
}

std::vector<std::filesystem::path> WaypointEditorTool::waypointSetFiles() const
{
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::exists(waypoint_sets_directory_)) {
        return files;
    }
    for (const auto &entry : std::filesystem::directory_iterator(waypoint_sets_directory_)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".yaml" || entry.path().filename() == "index.yaml") {
            continue;
        }
        files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

bool WaypointEditorTool::validSetId(const std::string &set_id) const
{
    static const std::regex valid("^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$");
    return std::regex_match(set_id, valid);
}

std::string WaypointEditorTool::makeSetId(const std::string &name) const
{
    std::string result;
    for (const auto ch : Trim(name)) {
        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-') {
            result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        } else if (!result.empty() && result.back() != '-') {
            result.push_back('-');
        }
    }
    while (!result.empty() && (result.back() == '-' || result.back() == '_')) {
        result.pop_back();
    }
    if (result.empty()) {
        result = "set";
    }
    if (result.size() > 64) {
        result.resize(64);
    }
    std::string candidate = result;
    int suffix = 2;
    while (std::filesystem::exists(waypoint_sets_directory_ / (candidate + ".yaml"))) {
        const auto suffix_text = "-" + std::to_string(suffix++);
        candidate = result.substr(0, std::min<std::size_t>(result.size(), 64 - suffix_text.size())) + suffix_text;
    }
    return candidate;
}

bool WaypointEditorTool::writeActiveSetIndex(const std::string &set_id, std::string &error) const
{
    try {
        std::filesystem::create_directories(waypoint_sets_directory_);
    } catch (const std::exception &ex) {
        error = ex.what();
        return false;
    }
    std::ofstream ofs(waypoint_index_path_);
    if (!ofs.is_open()) {
        error = "Failed to write " + waypoint_index_path_.string();
        return false;
    }
    ofs << "active_set_id: " << set_id << "\n";
    return true;
}

std::string WaypointEditorTool::activeSetFromIndex() const
{
    std::string selected;
    try {
        if (std::filesystem::exists(waypoint_index_path_)) {
            YAML::Node index = YAML::LoadFile(waypoint_index_path_.string());
            if (auto node = index["active_set_id"]; node && node.IsScalar()) {
                selected = node.as<std::string>();
            }
        }
    } catch (const YAML::Exception &) {
        selected.clear();
    }
    if (validSetId(selected) && std::filesystem::exists(waypoint_sets_directory_ / (selected + ".yaml"))) {
        return selected;
    }
    auto files = waypointSetFiles();
    if (!files.empty()) {
        return files.front().stem().string();
    }
    return "default";
}

bool WaypointEditorTool::writeWaypointSetFile(const std::string &set_id, const std::string &name, int revision, const std::vector<Waypoint> &waypoints, std::string &error) const
{
    if (!validSetId(set_id)) {
        error = "Invalid waypoint set id: " + set_id;
        return false;
    }
    try {
        std::filesystem::create_directories(waypoint_sets_directory_);
    } catch (const std::exception &ex) {
        error = ex.what();
        return false;
    }
    const auto path = waypoint_sets_directory_ / (set_id + ".yaml");
    YAML::Node root;
    root["name"] = Trim(name).empty() ? set_id : Trim(name);
    root["revision"] = revision;
    YAML::Node waypoint_nodes(YAML::NodeType::Sequence);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];
        const auto waypoint_name = wp.function_command.empty() ? "waypoint_" + std::to_string(i + 1) : wp.function_command;
        YAML::Node item;
        item["id"] = wp.id.empty() ? std::to_string(i) : wp.id;
        item["name"] = waypoint_name;
        item["x"] = wp.pose.pose.position.x;
        item["y"] = wp.pose.pose.position.y;
        item["yaw"] = YawFromPose(wp.pose.pose);
        item["dwell_seconds"] = 0.0;
        item["enabled"] = true;
        waypoint_nodes.push_back(item);
    }
    root["waypoints"] = waypoint_nodes;

    if (!route_segments_.empty()) {
        YAML::Node segment_nodes(YAML::NodeType::Sequence);
        for (const auto &segment : route_segments_) {
            YAML::Node item;
            item["id"] = segment.id;
            item["name"] = segment.name;
            item["from_waypoint_id"] = segment.from_waypoint_id;
            item["to_waypoint_id"] = segment.to_waypoint_id;
            item["bidirectional"] = segment.bidirectional;
            item["enabled"] = segment.enabled;
            YAML::Node via_nodes(YAML::NodeType::Sequence);
            for (const auto &via : segment.via_points) {
                YAML::Node via_node;
                via_node["id"] = via.id;
                via_node["name"] = via.name;
                via_node["x"] = via.pose.x;
                via_node["y"] = via.pose.y;
                via_node["yaw"] = via.pose.theta;
                via_node["dwell_seconds"] = 0.0;
                via_node["enabled"] = via.enabled;
                via_nodes.push_back(via_node);
            }
            item["via_points"] = via_nodes;
            segment_nodes.push_back(item);
        }
        root["segments"] = segment_nodes;
    }
    if (route_settings_ && route_settings_.IsMap()) {
        root["settings"] = route_settings_;
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "Failed to write " + path.string();
        return false;
    }
    ofs << root << "\n";
    error.clear();
    return true;
}

bool WaypointEditorTool::ensureWaypointSets(std::string &error)
{
    try {
        std::filesystem::create_directories(waypoint_sets_directory_);
    } catch (const std::exception &ex) {
        error = ex.what();
        return false;
    }
    if (!waypointSetFiles().empty()) {
        return true;
    }

    if (std::filesystem::exists(legacy_waypoint_sets_directory_)) {
        try {
            for (const auto &entry : std::filesystem::directory_iterator(legacy_waypoint_sets_directory_)) {
                if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
                    continue;
                }
                const auto destination = waypoint_sets_directory_ / entry.path().filename();
                if (!std::filesystem::exists(destination)) {
                    std::filesystem::copy_file(entry.path(), destination);
                }
            }
        } catch (const std::exception &ex) {
            error = ex.what();
            return false;
        }
        if (!waypointSetFiles().empty()) {
            return true;
        }
    }

    std::vector<Waypoint> migrated;
    if (!waypoint_file_.empty() && std::filesystem::exists(waypoint_file_)) {
        std::string load_error;
        if (!io::WaypointYaml::Load(waypoint_file_, migrated, load_error)) {
            RCLCPP_WARN(nh_->get_logger(), "Could not migrate legacy waypoints: %s", load_error.c_str());
        }
    }
    const auto accidental_legacy_file = map_directory_ / map_id_ / "waypoints.yaml";
    if (migrated.empty() && std::filesystem::exists(accidental_legacy_file)) {
        std::string load_error;
        if (!io::WaypointYaml::Load(accidental_legacy_file.string(), migrated, load_error)) {
            RCLCPP_WARN(nh_->get_logger(), "Could not migrate %s: %s", accidental_legacy_file.string().c_str(), load_error.c_str());
        }
    }
    if (!writeWaypointSetFile("default", "Default", 0, migrated, error)) {
        return false;
    }
    return writeActiveSetIndex("default", error);
}

bool WaypointEditorTool::loadGraphFromYaml(
    const std::filesystem::path &path, std::string &error)
{
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        route_segments_.clear();
        const YAML::Node segments = root["segments"];
        if (segments && segments.IsSequence()) {
            for (const auto &node : segments) {
                if (!node.IsMap()) {
                    continue;
                }
                sahabat_interfaces::msg::RouteSegment segment;
                segment.id = node["id"] ? node["id"].as<std::string>() : makeWaypointId("route");
                segment.name = node["name"] ? node["name"].as<std::string>() : segment.id;
                segment.from_waypoint_id = node["from_waypoint_id"]
                    ? node["from_waypoint_id"].as<std::string>() : "";
                segment.to_waypoint_id = node["to_waypoint_id"]
                    ? node["to_waypoint_id"].as<std::string>() : "";
                segment.bidirectional = node["bidirectional"]
                    ? node["bidirectional"].as<bool>() : true;
                segment.enabled = node["enabled"] ? node["enabled"].as<bool>() : true;
                const YAML::Node via_points = node["via_points"];
                if (via_points && via_points.IsSequence()) {
                    for (const auto &via_node : via_points) {
                        sahabat_interfaces::msg::Waypoint via;
                        via.id = via_node["id"]
                            ? via_node["id"].as<std::string>() : makeWaypointId("via");
                        via.name = via_node["name"]
                            ? via_node["name"].as<std::string>() : "route_point";
                        via.pose.x = via_node["x"] ? via_node["x"].as<double>() : 0.0;
                        via.pose.y = via_node["y"] ? via_node["y"].as<double>() : 0.0;
                        via.pose.theta = via_node["yaw"] ? via_node["yaw"].as<double>() : 0.0;
                        via.dwell_seconds = 0.0;
                        via.enabled = via_node["enabled"] ? via_node["enabled"].as<bool>() : true;
                        segment.via_points.push_back(std::move(via));
                    }
                }
                if (!segment.id.empty()) {
                    route_segments_.push_back(std::move(segment));
                }
            }
        }
        const YAML::Node settings = root["settings"];
        route_settings_ = settings && settings.IsMap()
            ? YAML::Clone(settings)
            : YAML::Node(YAML::NodeType::Map);
        active_segment_id_ = route_segments_.empty() ? "" : route_segments_.front().id;
        add_route_point_armed_ = false;
        error.clear();
        return true;
    } catch (const YAML::Exception &ex) {
        error = "Failed to parse route graph: " + std::string(ex.what());
        return false;
    }
}

bool WaypointEditorTool::loadWaypointSet(const std::string &set_id, std::string &error)
{
    if (!validSetId(set_id)) {
        error = "Invalid waypoint set id: " + set_id;
        return false;
    }
    const auto path = waypoint_sets_directory_ / (set_id + ".yaml");
    if (!std::filesystem::exists(path)) {
        error = "Waypoint set does not exist: " + set_id;
        return false;
    }
    std::vector<Waypoint> loaded;
    if (!io::WaypointYaml::Load(path.string(), loaded, error)) {
        return false;
    }
    for (std::size_t i = 0; i < loaded.size(); ++i) {
        if (loaded[i].id.empty()) {
            loaded[i].id = std::to_string(i);
        }
    }
    if (!loadGraphFromYaml(path, error)) {
        return false;
    }
    try {
        YAML::Node root = YAML::LoadFile(path.string());
        active_revision_ = root["revision"] ? root["revision"].as<int>() : 0;
        active_set_name_ = root["name"] ? root["name"].as<std::string>() : set_id;
    } catch (const YAML::Exception &) {
        active_revision_ = 0;
        active_set_name_ = set_id;
    }
    for (auto &wp : loaded) {
        if (wp.pose.header.frame_id.empty()) {
            wp.pose.header.frame_id = "map";
        }
        wp.pose.header.stamp = nh_->now();
    }
    active_set_id_ = set_id;
    waypoint_sequence_.assign(std::move(loaded));
    waypoint_sequence_.resetHistory();
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    if (!writeActiveSetIndex(set_id, error)) {
        return false;
    }
    error.clear();
    return true;
}

bool WaypointEditorTool::saveActiveWaypointSet(std::string &error)
{
    if (active_set_id_.empty()) {
        active_set_id_ = activeSetFromIndex();
    }
    const int next_revision = active_revision_ + 1;
    if (!writeWaypointSetFile(active_set_id_, active_set_name_, next_revision, waypoint_sequence_.waypoints(), error)) {
        return false;
    }
    active_revision_ = next_revision;
    return true;
}

bool WaypointEditorTool::requestFilePathForSaving(std::string &path, bool &save_as_yaml)
{
    if (!waypoint_file_.empty()) {
        path = waypoint_file_;
        save_as_yaml = true;
        return true;
    }

    QString selected_filter;
    QString qpath = QFileDialog::getSaveFileName(
        nullptr,
        tr("Save Waypoints As"),
        "",
        tr("CSV Files (*.csv);;YAML Files (*.yaml)"),
        &selected_filter
    );

    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        save_as_yaml = true;
    } else if (lower.endsWith(".csv")) {
        save_as_yaml = false;
    } else {
        // Fallback to selected filter when no extension given.
        if (selected_filter.contains("yaml", Qt::CaseInsensitive)) {
            qpath += ".yaml";
            save_as_yaml = true;
        } else {
            qpath += ".csv";
            save_as_yaml = false;
        }
    }

    path = qpath.toStdString();
    return true;
}

bool WaypointEditorTool::requestFilePathForLoading(std::string &path, bool &load_yaml)
{
    if (!waypoint_file_.empty()) {
        path = waypoint_file_;
        load_yaml = true;
        return true;
    }

    QString selected_filter;
    QString qpath = QFileDialog::getOpenFileName(
        nullptr,
        tr("Open Waypoints"),
        "",
        tr("CSV Files (*.csv);;YAML Files (*.yaml)"),
        &selected_filter
    );
    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        load_yaml = true;
    } else if (lower.endsWith(".csv")) {
        load_yaml = false;
    } else {
        load_yaml = selected_filter.contains("yaml", Qt::CaseInsensitive);
    }

    path = qpath.toStdString();
    return true;
}

void WaypointEditorTool::handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string error;
    if (usingOperatorBackend()) {
        if (saveOperatorWaypointSet(error)) {
            res->success = true;
            res->message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to live set '" + active_set_name_ + "'";
        } else {
            res->success = false;
            res->message = error;
        }
        return;
    }

    if (ensureWaypointSets(error) && saveActiveWaypointSet(error)) {
        res->success = true;
        res->message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to set '" + active_set_name_ + "'";
        return;
    }

    std::string path;
    bool save_as_yaml = false;
    if (!requestFilePathForSaving(path, save_as_yaml)) {
        res->success = false;
        res->message = "Save canceled by user";
        return;
    }

    bool ok = false;
    if (save_as_yaml) {
        ok = io::WaypointYaml::Save(waypoint_sequence_.waypoints(), path, error);
    } else {
        ok = io::WaypointCsv::Save(waypoint_sequence_.waypoints(), path, error);
    }

    if (!ok) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot open file:\n%1").arg(QString::fromStdString(path)));
        res->success = false;
        res->message = error;
        return;
    }

    res->success = true;
    res->message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to " + path;
}

void WaypointEditorTool::handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string error;
    if (usingOperatorBackend()) {
        if (loadOperatorWaypointSet(active_set_id_, error)) {
            res->success = true;
            res->message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from live set '" + active_set_name_ + "'";
        } else {
            res->success = false;
            res->message = error;
        }
        return;
    }

    if (ensureWaypointSets(error) && loadWaypointSet(active_set_id_.empty() ? activeSetFromIndex() : active_set_id_, error)) {
        res->success = true;
        res->message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from set '" + active_set_name_ + "'";
        return;
    }

    std::string path;
    bool load_yaml = false;
    if (!requestFilePathForLoading(path, load_yaml)) {
        res->success = false;
        res->message = "Load canceled by user";
        return;
    }

    if (!loadWaypointsFromPath(path, load_yaml, error)) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot open file:\n%1").arg(QString::fromStdString(path)));
        res->success = false;
        res->message = error;
        return;
    }

    res->success = true;
    res->message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from " + path;
}

void WaypointEditorTool::handleListWaypointSets(
    const std::shared_ptr<sahabat_interfaces::srv::ListWaypointSets::Request> /*req*/,
    std::shared_ptr<sahabat_interfaces::srv::ListWaypointSets::Response> res)
{
    if (usingOperatorBackend()) {
        if (!operator_list_sets_client_ || !operator_list_sets_client_->wait_for_service(std::chrono::seconds(2))) {
            res->message = "Operator waypoint-set list service unavailable";
            return;
        }
        auto req = std::make_shared<sahabat_interfaces::srv::ListWaypointSets::Request>();
        req->map_id = map_id_;
        auto future = operator_list_sets_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            res->message = "Timed out listing operator waypoint sets";
            return;
        }
        auto response = future.get();
        res->sets = response->sets;
        res->active_set_id = response->active_set_id;
        res->has_dock = response->has_dock;
        res->dock = response->dock;
        res->message = response->message;
        return;
    }

    std::string error;
    if (!ensureWaypointSets(error)) {
        res->message = error;
        return;
    }
    const auto active = active_set_id_.empty() ? activeSetFromIndex() : active_set_id_;
    res->active_set_id = active;
    for (const auto &path : waypointSetFiles()) {
        YAML::Node root;
        try {
            root = YAML::LoadFile(path.string());
        } catch (const YAML::Exception &) {
            continue;
        }
        sahabat_interfaces::msg::WaypointSetInfo info;
        info.id = path.stem().string();
        info.name = root["name"] ? root["name"].as<std::string>() : info.id;
        info.revision = root["revision"] ? static_cast<uint64_t>(std::max(0, root["revision"].as<int>())) : 0;
        info.waypoint_count = root["waypoints"] && root["waypoints"].IsSequence()
            ? static_cast<uint32_t>(root["waypoints"].size())
            : 0;
        res->sets.push_back(info);
    }
    res->message = std::to_string(res->sets.size()) + " waypoint set(s) loaded";
}

void WaypointEditorTool::handleManageWaypointSet(
    const std::shared_ptr<sahabat_interfaces::srv::ManageWaypointSet::Request> req,
    std::shared_ptr<sahabat_interfaces::srv::ManageWaypointSet::Response> res)
{
    if (usingOperatorBackend()) {
        std::string error;
        std::string lease_id;
        if (!acquireOperatorLease(lease_id, error)) {
            res->message = error;
            return;
        }
        auto release = [this, &lease_id]() { releaseOperatorLease(lease_id); };
        if (!operator_manage_set_client_ || !operator_manage_set_client_->wait_for_service(std::chrono::seconds(2))) {
            res->message = "Operator waypoint-set manage service unavailable";
            release();
            return;
        }
        auto manage_req = std::make_shared<sahabat_interfaces::srv::ManageWaypointSet::Request>();
        manage_req->action = req->action;
        manage_req->map_id = map_id_;
        manage_req->set_id = req->set_id;
        manage_req->name = req->name;
        manage_req->lease_id = lease_id;
        auto future = operator_manage_set_client_->async_send_request(manage_req);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            res->message = "Timed out managing operator waypoint set";
            release();
            return;
        }
        auto response = future.get();
        res->success = response->success;
        res->set_id = response->set_id;
        res->active_set_id = response->active_set_id;
        res->message = response->message;
        if (!response->success) {
            release();
            return;
        }

        if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::CREATE) {
            const std::string previous_set = active_set_id_;
            const int previous_revision = active_revision_;
            active_set_id_ = response->set_id;
            active_set_name_ = Trim(req->name).empty() ? response->set_id : Trim(req->name);
            active_revision_ = 0;
            if (!operator_save_graph_client_ || !operator_save_graph_client_->wait_for_service(std::chrono::seconds(2))) {
                active_set_id_ = previous_set;
                active_revision_ = previous_revision;
                res->success = false;
                res->message = "Operator waypoint graph save service unavailable";
                release();
                return;
            }
            auto save_req = std::make_shared<sahabat_interfaces::srv::SaveWaypointGraph::Request>();
            save_req->map_id = map_id_;
            save_req->set_id = active_set_id_;
            save_req->expected_revision = 0;
            save_req->lease_id = lease_id;
            for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
                const auto &wp = waypoint_sequence_.at(i);
                sahabat_interfaces::msg::Waypoint item;
                item.id = wp.id.empty() ? std::to_string(i) : wp.id;
                item.name = wp.function_command.empty() ? "waypoint_" + std::to_string(i + 1) : wp.function_command;
                item.pose.x = wp.pose.pose.position.x;
                item.pose.y = wp.pose.pose.position.y;
                item.pose.theta = YawFromPose(wp.pose.pose);
                item.dwell_seconds = 0.0;
                item.enabled = true;
                save_req->waypoints.push_back(item);
            }
            save_req->segments = route_segments_;
            auto save_future = operator_save_graph_client_->async_send_request(save_req);
            if (save_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
                active_set_id_ = previous_set;
                active_revision_ = previous_revision;
                res->success = false;
                res->message = "Timed out saving current waypoints to new live set";
                release();
                return;
            }
            auto save_response = save_future.get();
            if (!save_response->success) {
                active_set_id_ = previous_set;
                active_revision_ = previous_revision;
                res->success = false;
                res->message = save_response->message;
                release();
                return;
            }
            active_revision_ = static_cast<int>(save_response->revision);
            res->message = "Saved current waypoints as live set '" + active_set_name_ + "'";
        } else if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::SELECT ||
                   req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::DELETE) {
            std::string load_error;
            if (!loadOperatorWaypointSet(response->active_set_id.empty() ? response->set_id : response->active_set_id, load_error)) {
                res->message = response->message + "; reload failed: " + load_error;
            }
        } else if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::RENAME) {
            if (response->set_id == active_set_id_) {
                active_set_name_ = Trim(req->name).empty() ? active_set_id_ : Trim(req->name);
            }
        }
        release();
        return;
    }

    std::string error;
    if (!ensureWaypointSets(error)) {
        res->message = error;
        return;
    }

    try {
        if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::CREATE) {
            const std::string name = Trim(req->name).empty() ? "Untitled set" : Trim(req->name);
            const auto set_id = makeSetId(name);
            if (!writeWaypointSetFile(set_id, name, 0, waypoint_sequence_.waypoints(), error) || !loadWaypointSet(set_id, error)) {
                res->message = error;
                return;
            }
            res->set_id = set_id;
            res->message = "Saved current waypoints as set '" + name + "'";
        } else if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::RENAME) {
            const auto set_id = req->set_id.empty() ? active_set_id_ : req->set_id;
            if (!validSetId(set_id)) {
                res->message = "Invalid waypoint set id";
                return;
            }
            const auto path = waypoint_sets_directory_ / (set_id + ".yaml");
            if (!std::filesystem::exists(path)) {
                res->message = "Waypoint set does not exist: " + set_id;
                return;
            }
            std::vector<Waypoint> loaded;
            if (!io::WaypointYaml::Load(path.string(), loaded, error)) {
                res->message = error;
                return;
            }
            int revision = 0;
            try {
                YAML::Node root = YAML::LoadFile(path.string());
                revision = root["revision"] ? root["revision"].as<int>() : 0;
            } catch (const YAML::Exception &) {
                revision = 0;
            }
            const std::string name = Trim(req->name).empty() ? set_id : Trim(req->name);
            if (!writeWaypointSetFile(set_id, name, revision, loaded, error)) {
                res->message = error;
                return;
            }
            if (set_id == active_set_id_) {
                active_set_name_ = name;
            }
            res->set_id = set_id;
            res->message = "Renamed waypoint set";
        } else if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::DELETE) {
            const auto files = waypointSetFiles();
            if (files.size() <= 1) {
                res->message = "A map must keep at least one waypoint set";
                return;
            }
            const auto set_id = req->set_id.empty() ? active_set_id_ : req->set_id;
            if (!validSetId(set_id)) {
                res->message = "Invalid waypoint set id";
                return;
            }
            const auto path = waypoint_sets_directory_ / (set_id + ".yaml");
            if (!std::filesystem::exists(path)) {
                res->message = "Waypoint set does not exist: " + set_id;
                return;
            }
            const auto archive = waypoint_sets_directory_ / "archive";
            std::filesystem::create_directories(archive);
            auto destination = archive / (set_id + "-archived.yaml");
            int archive_suffix = 2;
            while (std::filesystem::exists(destination)) {
                destination = archive / (set_id + "-archived-" + std::to_string(archive_suffix++) + ".yaml");
            }
            std::filesystem::rename(path, destination);
            auto remaining = waypointSetFiles();
            const auto next_set = remaining.empty() ? "default" : remaining.front().stem().string();
            if (!loadWaypointSet(next_set, error)) {
                res->message = error;
                return;
            }
            res->set_id = set_id;
            res->message = "Archived waypoint set";
        } else if (req->action == sahabat_interfaces::srv::ManageWaypointSet::Request::SELECT) {
            const auto set_id = req->set_id.empty() ? activeSetFromIndex() : req->set_id;
            if (!loadWaypointSet(set_id, error)) {
                res->message = error;
                return;
            }
            res->set_id = set_id;
            res->message = "Loaded waypoint set '" + active_set_name_ + "'";
        } else {
            res->message = "Unknown waypoint-set action";
            return;
        }
    } catch (const std::exception &ex) {
        res->message = ex.what();
        return;
    }

    res->active_set_id = active_set_id_;
    res->success = true;
}

void WaypointEditorTool::handleGetWaypoints(
    const std::shared_ptr<sahabat_interfaces::srv::GetWaypoints::Request> req,
    std::shared_ptr<sahabat_interfaces::srv::GetWaypoints::Response> res)
{
    if (!req->set_id.empty() && req->set_id != active_set_id_) {
        std::string error;
        const bool loaded = usingOperatorBackend()
            ? loadOperatorWaypointSet(req->set_id, error)
            : loadWaypointSet(req->set_id, error);
        if (!loaded) {
            res->message = error;
            return;
        }
    }

    res->revision = static_cast<uint64_t>(std::max(0, active_revision_));
    res->set_id = active_set_id_;
    for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        const auto &wp = waypoint_sequence_.at(i);
        sahabat_interfaces::msg::Waypoint item;
        item.id = wp.id.empty() ? std::to_string(i) : wp.id;
        item.name = wp.function_command.empty()
            ? "waypoint_" + std::to_string(i + 1)
            : wp.function_command;
        item.pose.x = wp.pose.pose.position.x;
        item.pose.y = wp.pose.pose.position.y;
        item.pose.theta = YawFromPose(wp.pose.pose);
        item.dwell_seconds = 0.0;
        item.enabled = true;
        res->waypoints.push_back(item);
    }
    res->message = std::to_string(res->waypoints.size())
        + " waypoint(s) in current editor set";
}

void WaypointEditorTool::handleGetWaypointGraph(
    const std::shared_ptr<sahabat_interfaces::srv::GetWaypointGraph::Request> req,
    std::shared_ptr<sahabat_interfaces::srv::GetWaypointGraph::Response> res)
{
    if (!req->set_id.empty() && req->set_id != active_set_id_) {
        std::string error;
        const bool loaded = usingOperatorBackend()
            ? loadOperatorWaypointSet(req->set_id, error)
            : loadWaypointSet(req->set_id, error);
        if (!loaded) {
            res->message = error;
            return;
        }
    }
    res->revision = static_cast<uint64_t>(std::max(0, active_revision_));
    res->set_id = active_set_id_;
    res->active_segment_id = active_segment_id_;
    for (std::size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        const auto &wp = waypoint_sequence_.at(i);
        sahabat_interfaces::msg::Waypoint item;
        item.id = wp.id.empty() ? std::to_string(i) : wp.id;
        item.name = wp.function_command.empty()
            ? "waypoint_" + std::to_string(i + 1)
            : wp.function_command;
        item.pose.x = wp.pose.pose.position.x;
        item.pose.y = wp.pose.pose.position.y;
        item.pose.theta = YawFromPose(wp.pose.pose);
        item.dwell_seconds = 0.0;
        item.enabled = true;
        res->waypoints.push_back(std::move(item));
    }
    res->segments = route_segments_;
    res->message = std::to_string(res->waypoints.size()) + " waypoint(s), " +
        std::to_string(res->segments.size()) + " route segment(s) in editor";
}

void WaypointEditorTool::handleEditRoute(
    const std::shared_ptr<sahabat_interfaces::srv::EditRoute::Request> req,
    std::shared_ptr<sahabat_interfaces::srv::EditRoute::Response> res)
{
    using Service = sahabat_interfaces::srv::EditRoute;
    if (req->action == Service::Request::CREATE) {
        const int from_index = waypointIndexById(req->from_waypoint_id);
        const int to_index = waypointIndexById(req->to_waypoint_id);
        if (from_index < 0 || to_index < 0) {
            res->message = "Select two valid main waypoints";
            return;
        }
        if (from_index == to_index) {
            res->message = "Route endpoints must be different";
            return;
        }
        const auto duplicate = std::find_if(
            route_segments_.begin(), route_segments_.end(),
            [&req](const sahabat_interfaces::msg::RouteSegment &segment) {
                return segment.from_waypoint_id == req->from_waypoint_id &&
                       segment.to_waypoint_id == req->to_waypoint_id;
            });
        if (duplicate != route_segments_.end()) {
            active_segment_id_ = duplicate->id;
            refreshRouteVisualization();
            res->active_segment_id = active_segment_id_;
            res->message = "That route already exists; selected it";
            res->success = true;
            return;
        }
        sahabat_interfaces::msg::RouteSegment segment;
        segment.id = makeWaypointId("route");
        const auto &from = waypoint_sequence_.at(static_cast<std::size_t>(from_index));
        const auto &to = waypoint_sequence_.at(static_cast<std::size_t>(to_index));
        const std::string from_name = from.function_command.empty() ? from.id : from.function_command;
        const std::string to_name = to.function_command.empty() ? to.id : to.function_command;
        segment.name = from_name + " to " + to_name;
        segment.from_waypoint_id = req->from_waypoint_id;
        segment.to_waypoint_id = req->to_waypoint_id;
        segment.bidirectional = req->bidirectional;
        segment.enabled = true;
        active_segment_id_ = segment.id;
        route_segments_.push_back(std::move(segment));
        refreshRouteVisualization();
        res->message = "Route created; add route points where you want the robot to pass";
    } else if (req->action == Service::Request::DELETE) {
        const int index = segmentIndex(req->segment_id);
        if (index < 0) {
            res->message = "Select a route first";
            return;
        }
        route_segments_.erase(route_segments_.begin() + index);
        active_segment_id_ = route_segments_.empty() ? "" : route_segments_.front().id;
        add_route_point_armed_ = false;
        refreshRouteVisualization();
        res->message = "Route deleted";
    } else if (req->action == Service::Request::SELECT) {
        if (segmentIndex(req->segment_id) < 0) {
            res->message = "Selected route no longer exists";
            return;
        }
        active_segment_id_ = req->segment_id;
        add_route_point_armed_ = false;
        refreshRouteVisualization();
        res->message = "Route selected";
    } else if (req->action == Service::Request::ARM_ADD_POINT) {
        if (segmentIndex(req->segment_id.empty() ? active_segment_id_ : req->segment_id) < 0) {
            res->message = "Create or select a route first";
            return;
        }
        if (!req->segment_id.empty()) {
            active_segment_id_ = req->segment_id;
        }
        add_route_point_armed_ = true;
        res->message = "Next Add Waypoint map click will create a route point";
    } else if (req->action == Service::Request::CANCEL_ADD_POINT) {
        add_route_point_armed_ = false;
        res->message = "Route-point placement canceled";
    } else if (req->action == Service::Request::SET_BIDIRECTIONAL) {
        const int index = segmentIndex(req->segment_id);
        if (index < 0) {
            res->message = "Select a route first";
            return;
        }
        route_segments_.at(static_cast<std::size_t>(index)).bidirectional =
            req->bidirectional;
        res->message = req->bidirectional
            ? "Route is usable in both directions"
            : "Route is one-way from From to To";
    } else if (req->action == Service::Request::ATTACH_EXISTING_POINTS) {
        const int target_index = segmentIndex(req->segment_id);
        if (target_index < 0) {
            res->message = "Select a route first";
            return;
        }
        auto &target = route_segments_.at(static_cast<std::size_t>(target_index));
        std::set<std::string> requested_ids(
            req->route_point_ids.begin(), req->route_point_ids.end());
        if (!req->route_point_id.empty()) {
            requested_ids.insert(req->route_point_id);
        }
        int added = 0;
        for (const auto &requested_id : requested_ids) {
            if (std::any_of(
                    target.via_points.begin(), target.via_points.end(),
                    [&requested_id](const sahabat_interfaces::msg::Waypoint &point) {
                        return point.id == requested_id;
                    }))
            {
                continue;
            }
            sahabat_interfaces::msg::Waypoint existing;
            bool found_existing = false;
            for (const auto &route : route_segments_) {
                const auto found = std::find_if(
                    route.via_points.begin(), route.via_points.end(),
                    [&requested_id](const sahabat_interfaces::msg::Waypoint &point) {
                        return point.id == requested_id;
                    });
                if (found != route.via_points.end()) {
                    existing = *found;
                    found_existing = true;
                    break;
                }
            }
            if (found_existing) {
                target.via_points.push_back(std::move(existing));
                ++added;
            }
        }
        if (added == 0) {
            res->message = "No new points added; selected points are already in this route";
            return;
        }
        active_segment_id_ = target.id;
        refreshRouteVisualization();
        res->message = std::to_string(added) +
            " existing point(s) added to this route; shared poses stay synchronized";
    } else if (req->action == Service::Request::REMOVE_POINT_FROM_ROUTE) {
        const int target_index = segmentIndex(req->segment_id);
        if (target_index < 0) {
            res->message = "Select a route first";
            return;
        }
        auto &points = route_segments_.at(static_cast<std::size_t>(target_index)).via_points;
        const auto point = std::find_if(
            points.begin(), points.end(),
            [&req](const sahabat_interfaces::msg::Waypoint &candidate) {
                return candidate.id == req->route_point_id;
            });
        if (point == points.end()) {
            res->message = "Selected point is not in this route";
            return;
        }
        points.erase(point);
        refreshRouteVisualization();
        res->message = "Point removed from this route only";
    } else if (
        req->action == Service::Request::MOVE_POINT_EARLIER ||
        req->action == Service::Request::MOVE_POINT_LATER)
    {
        const int target_index = segmentIndex(req->segment_id);
        if (target_index < 0) {
            res->message = "Select a route first";
            return;
        }
        auto &points = route_segments_.at(static_cast<std::size_t>(target_index)).via_points;
        const auto point = std::find_if(
            points.begin(), points.end(),
            [&req](const sahabat_interfaces::msg::Waypoint &candidate) {
                return candidate.id == req->route_point_id;
            });
        if (point == points.end()) {
            res->message = "Selected point is not in this route";
            return;
        }
        const auto index = static_cast<std::size_t>(std::distance(points.begin(), point));
        if (req->action == Service::Request::MOVE_POINT_EARLIER) {
            if (index == 0) {
                res->message = "Point is already first in this route";
                return;
            }
            std::swap(points[index], points[index - 1]);
            res->message = "Point moved earlier in this route";
        } else {
            if (index + 1 >= points.size()) {
                res->message = "Point is already last in this route";
                return;
            }
            std::swap(points[index], points[index + 1]);
            res->message = "Point moved later in this route";
        }
        refreshRouteVisualization();
    } else if (req->action == Service::Request::DELETE_POINTS_EVERYWHERE) {
        std::set<std::string> requested_ids(
            req->route_point_ids.begin(), req->route_point_ids.end());
        if (!req->route_point_id.empty()) {
            requested_ids.insert(req->route_point_id);
        }
        if (requested_ids.empty()) {
            res->message = "Select one or more map route points";
            return;
        }
        int removed = 0;
        for (auto &route : route_segments_) {
            auto &points = route.via_points;
            const auto new_end = std::remove_if(
                points.begin(), points.end(),
                [&requested_ids](const sahabat_interfaces::msg::Waypoint &point) {
                    return requested_ids.count(point.id) > 0;
                });
            removed += static_cast<int>(std::distance(new_end, points.end()));
            points.erase(new_end, points.end());
        }
        if (removed == 0) {
            res->message = "Selected map route points no longer exist";
            return;
        }
        refreshRouteVisualization();
        res->message = std::to_string(requested_ids.size()) +
            " point(s) deleted from every route";
    } else {
        res->message = "Unknown route edit action";
        return;
    }

    res->success = true;
    res->active_segment_id = active_segment_id_;
}

bool WaypointEditorTool::loadWaypointsFromPath(const std::string &path, bool load_yaml, std::string &error)
{
    std::vector<Waypoint> loaded;
    bool ok = false;
    if (load_yaml) {
        ok = io::WaypointYaml::Load(path, loaded, error);
    } else {
        ok = io::WaypointCsv::Load(path, loaded, error);
    }
    if (!ok) {
        return false;
    }
    for (std::size_t i = 0; i < loaded.size(); ++i) {
        auto &wp = loaded[i];
        if (wp.id.empty()) {
            wp.id = std::to_string(i);
        }
        if (wp.pose.header.frame_id.empty()) {
            wp.pose.header.frame_id = "map";
        }
        wp.pose.header.stamp = nh_->now();
    }
    if (load_yaml) {
        if (!loadGraphFromYaml(path, error)) {
            return false;
        }
    } else {
        route_segments_.clear();
        route_settings_ = YAML::Node(YAML::NodeType::Map);
        active_segment_id_.clear();
        add_route_point_armed_ = false;
    }
    waypoint_sequence_.assign(std::move(loaded));
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1);
    return true;
}

void WaypointEditorTool::handleUndoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.undo()) {
        res->success = false;
        res->message = "No more actions to undo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res->success = true;
    res->message = "Undid waypoint change";
}

void WaypointEditorTool::handleRedoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.redo()) {
        res->success = false;
        res->message = "No more actions to redo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res->success = true;
    res->message = "Redid waypoint change";
}

void WaypointEditorTool::handleClearWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    waypoint_sequence_.clear();
    route_segments_.clear();
    route_settings_ = YAML::Node(YAML::NodeType::Map);
    active_segment_id_.clear();
    add_route_point_armed_ = false;
    pose_dirty_ = false;
    server_->clear();
    server_->applyChanges();
    commitWaypointChanges(0, false);
    res->success = true;
    res->message = "Cleared all waypoints";
}

void WaypointEditorTool::activate() {}
void WaypointEditorTool::deactivate()
{
    PoseTool::deactivate();
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorTool, rviz_common::Tool)

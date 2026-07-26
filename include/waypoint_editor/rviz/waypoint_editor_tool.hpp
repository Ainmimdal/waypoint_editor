#ifndef WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_
#define WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sahabat_interfaces/srv/control_lease.hpp>
#include <sahabat_interfaces/srv/get_waypoints.hpp>
#include <sahabat_interfaces/srv/list_waypoint_sets.hpp>
#include <sahabat_interfaces/srv/manage_waypoint_set.hpp>
#include <sahabat_interfaces/srv/save_waypoints.hpp>

#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "waypoint_editor/core/waypoint_sequence.hpp"

namespace waypoint_editor
{

class WaypointEditorTool : public rviz_default_plugins::tools::PoseTool
{
public:
    WaypointEditorTool();
    ~WaypointEditorTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    void onPoseSet(double x, double y, double theta) override;
    void updateWaypointMarker();
    visualization_msgs::msg::InteractiveMarker createWaypointMarker(int id);
    void processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb);
    void processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb);
    void handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleUndoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleRedoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleClearWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleListWaypointSets(const std::shared_ptr<sahabat_interfaces::srv::ListWaypointSets::Request> req, std::shared_ptr<sahabat_interfaces::srv::ListWaypointSets::Response> res);
    void handleManageWaypointSet(const std::shared_ptr<sahabat_interfaces::srv::ManageWaypointSet::Request> req, std::shared_ptr<sahabat_interfaces::srv::ManageWaypointSet::Response> res);
    void publishLineMarker();
    void publishTotalWpsDist();
    void publishLastWpsDist();
    void publishRangeMetrics();
    bool requestFilePathForSaving(std::string &path, bool &save_as_yaml);
    bool requestFilePathForLoading(std::string &path, bool &load_yaml);

private:
    int appendWaypointAndRefresh(Waypoint wp);
    bool loadWaypointsFromPath(const std::string &path, bool load_yaml, std::string &error);
    bool ensureWaypointSets(std::string &error);
    bool loadWaypointSet(const std::string &set_id, std::string &error);
    bool saveActiveWaypointSet(std::string &error);
    bool loadOperatorWaypointSet(const std::string &set_id, std::string &error);
    bool saveOperatorWaypointSet(std::string &error);
    bool acquireOperatorLease(std::string &lease_id, std::string &error);
    void releaseOperatorLease(const std::string &lease_id);
    bool usingOperatorBackend() const;
    std::vector<std::filesystem::path> waypointSetFiles() const;
    std::string activeSetFromIndex() const;
    bool writeActiveSetIndex(const std::string &set_id, std::string &error) const;
    bool writeWaypointSetFile(const std::string &set_id, const std::string &name, int revision, const std::vector<Waypoint> &waypoints, std::string &error) const;
    std::string makeSetId(const std::string &name) const;
    bool validSetId(const std::string &set_id) const;
    bool transformToMapFrame(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output) const;
    void refreshAutoPoseSubscription();
    void handleAutoPose(const geometry_msgs::msg::PoseStamped &pose);

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Node::SharedPtr backend_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> backend_executor_;
    std::thread backend_thread_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_wp_dist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr last_wp_dist_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_stop_service_;
    rclcpp::Service<sahabat_interfaces::srv::ListWaypointSets>::SharedPtr list_sets_service_;
    rclcpp::Service<sahabat_interfaces::srv::ManageWaypointSet>::SharedPtr manage_set_service_;
    rclcpp::Client<sahabat_interfaces::srv::ControlLease>::SharedPtr operator_lease_client_;
    rclcpp::Client<sahabat_interfaces::srv::GetWaypoints>::SharedPtr operator_get_waypoints_client_;
    rclcpp::Client<sahabat_interfaces::srv::SaveWaypoints>::SharedPtr operator_save_waypoints_client_;
    rclcpp::Client<sahabat_interfaces::srv::ListWaypointSets>::SharedPtr operator_list_sets_client_;
    rclcpp::Client<sahabat_interfaces::srv::ManageWaypointSet>::SharedPtr operator_manage_set_client_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr auto_distance_sub_;
    std::shared_ptr<rclcpp::SubscriptionBase> auto_pose_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string auto_pose_topic_;
    std::string auto_pose_type_;
    std::string waypoint_file_;
    std::string maps_directory_;
    std::string map_id_;
    std::string backend_mode_{"local"};
    std::filesystem::path map_directory_;
    std::filesystem::path waypoint_sets_directory_;
    std::filesystem::path legacy_waypoint_sets_directory_;
    std::filesystem::path waypoint_index_path_;
    std::string active_set_id_;
    std::string active_set_name_{"Default"};
    int active_revision_{0};
    double marker_size_{0.25};
    double auto_min_distance_m_{1.0};
    bool auto_enabled_{false};

    WaypointSequence waypoint_sequence_;
    bool pose_dirty_{false};
    double last_displayed_distance_{0.0};

    void updateLastDistanceFromWaypoint(int waypoint_index);
    double computeSegmentDistance(std::size_t first, std::size_t second) const;
    void commitWaypointChanges(int waypoint_index, bool snapshot_history = true);
    bool isValidWaypointId(int id) const;
};

} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_TOOL_HPP_

#ifndef WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_
#define WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QString>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sahabat_interfaces/srv/control_lease.hpp>
#include <sahabat_interfaces/srv/get_waypoints.hpp>
#include <sahabat_interfaces/srv/load_map.hpp>
#include <sahabat_interfaces/srv/list_waypoint_sets.hpp>
#include <sahabat_interfaces/srv/manage_waypoint_set.hpp>
#include <sahabat_interfaces/srv/patrol_command.hpp>

namespace waypoint_editor
{

class WaypointEditorPanel : public rviz_common::Panel
{
    Q_OBJECT
    
public:
    explicit WaypointEditorPanel(QWidget *parent = nullptr);
    ~WaypointEditorPanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config &config) override;
    void save(rviz_common::Config config) const override;

protected Q_SLOTS:
    void onLoad2DMapButtonClick();
    void onLoad3DMapButtonClick();
    void onLoadWaypointsButtonClick();
    void onSaveWaypointsButtonClick();
    void onWaypointSetSelected(int index);
    void onRefreshSetsButtonClick();
    void onNewSetButtonClick();
    void onRenameSetButtonClick();
    void onDeleteSetButtonClick();
    void onRefreshWaypointsButtonClick();
    void onGoToWaypointButtonClick();
    void onStopNavigationButtonClick();
    void onMarkerSizeChanged(double value);
    void onUndoWaypointsButtonClick();
    void onRedoWaypointsButtonClick();
    void onClearWaypointsButtonClick();
    void onAutoToggle(bool checked);

private:
    void setAutoControlsEnabled(bool enabled);
    void postStatusMessage(const QString &msg);
    void refreshWaypointSets();
    void refreshWaypoints();
    void manageWaypointSet(uint8_t action, const QString &set_id, const QString &name);
    void sendPatrolCommand(uint8_t command, const QString &waypoint_id);
    void releaseControlLease(const std::string &lease_id);

    QLabel *status_text_label_;
    QLabel *status_value_label_;
    QLabel *last_wp_dist_text_label_;
    QLabel *last_wp_dist_value_label_;
    QLabel *total_wp_dist_text_label_;
    QLabel *total_wp_dist_value_label_;
    QVBoxLayout *layout_;
    QHBoxLayout *logo_layout_;
    QHBoxLayout *button_layout_;
    QPushButton *load_2d_map_button_;
    QPushButton *load_3d_map_button_;
    QPushButton *load_waypoints_button_;
    QPushButton *save_waypoints_button_;
    QComboBox *waypoint_set_combo_;
    QPushButton *refresh_sets_button_;
    QPushButton *new_set_button_;
    QPushButton *rename_set_button_;
    QPushButton *delete_set_button_;
    QComboBox *waypoint_combo_;
    QPushButton *refresh_waypoints_button_;
    QPushButton *go_waypoint_button_;
    QPushButton *stop_navigation_button_;
    QDoubleSpinBox *marker_size_spin_;
    QPushButton *undo_button_;
    QPushButton *redo_button_;
    QPushButton *clear_button_;
    QPushButton *auto_toggle_button_;
    QDoubleSpinBox *auto_distance_spin_;
    QLineEdit *auto_topic_edit_;
    QComboBox *auto_type_combo_;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<sahabat_interfaces::srv::LoadMap>::SharedPtr load_map_client_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr nav2_load_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr redo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_stop_client_;
    rclcpp::Client<sahabat_interfaces::srv::ListWaypointSets>::SharedPtr list_sets_client_;
    rclcpp::Client<sahabat_interfaces::srv::ManageWaypointSet>::SharedPtr manage_set_client_;
    rclcpp::Client<sahabat_interfaces::srv::GetWaypoints>::SharedPtr get_waypoints_client_;
    rclcpp::Client<sahabat_interfaces::srv::ControlLease>::SharedPtr control_lease_client_;
    rclcpp::Client<sahabat_interfaces::srv::PatrolCommand>::SharedPtr patrol_client_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr auto_distance_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr last_wp_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr total_wp_dist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    QString map_dialog_directory_;
    QString map_id_;
};
    
} // namespace waypoint_editor

#endif // WAYPOINT_EDITOR__RVIZ__WAYPOINT_EDITOR_PANEL_HPP_

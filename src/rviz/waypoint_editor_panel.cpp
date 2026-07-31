#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QPixmap>
#include <QFont>
#include <QFontMetrics>
#include <QSizePolicy>
#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QInputDialog>
#include <QMessageBox>
#include <QVariant>
#include <QColor>
#include <QDir>
#include <QFileInfo>
#include <QScrollArea>
#include <QFrame>
#include <QWidget>
#include <QAbstractItemView>

#include <chrono>
#include <map>
#include <set>

#include "waypoint_editor/rviz/waypoint_editor_panel.hpp"

namespace waypoint_editor
{

namespace
{

QString expandUserPath(QString path)
{
    if (path.startsWith("~/")) {
        return QDir::homePath() + path.mid(1);
    }
    return path;
}

}  // namespace

WaypointEditorPanel::WaypointEditorPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    // Fonts
    QFont smallFont = this->font();
    smallFont.setPointSize(10);
    QFont boldFont = smallFont;
    boldFont.setBold(true);

    // Text labels (Bold Font)
    status_text_label_       = new QLabel("Status :", this);
    last_wp_dist_text_label_ = new QLabel("Last WP Distance :", this);
    total_wp_dist_text_label_= new QLabel("Total WP Distance :", this);

    for (auto *lbl : {status_text_label_, last_wp_dist_text_label_, total_wp_dist_text_label_}) {
        lbl->setFont(boldFont);
        lbl->setContentsMargins(0, 0, 0, 0);
    }

    // Value labels (Normal Font)
    status_value_label_       = new QLabel("Start Waypoint Editor!", this);
    last_wp_dist_value_label_ = new QLabel("0.0", this);
    total_wp_dist_value_label_= new QLabel("0.0", this);

    for (auto *lbl : {status_value_label_, last_wp_dist_value_label_, total_wp_dist_value_label_}) {
        lbl->setFont(smallFont);
        lbl->setContentsMargins(0, 0, 0, 0);
    }
    status_value_label_->setWordWrap(true);
    status_value_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // Row helper
    auto makeRow = [&](QLabel *text, QLabel *value) {
        QHBoxLayout *row = new QHBoxLayout;
        row->setContentsMargins(0, 8, 0, 0);
        row->setSpacing(4);
        row->addWidget(text);
        row->addWidget(value);
        row->addStretch();
        return row;
    };

    // Build rows
    QHBoxLayout *status_row        = new QHBoxLayout;
    status_row->setContentsMargins(0, 0, 0, 0);
    status_row->setSpacing(4);
    status_text_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    status_value_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    status_row->addWidget(status_text_label_);
    status_row->addWidget(status_value_label_, /*stretch=*/1);
    status_row->addStretch();
    QHBoxLayout *last_wp_dist_row  = makeRow(last_wp_dist_text_label_,  last_wp_dist_value_label_);
    QHBoxLayout *total_wp_dist_row = makeRow(total_wp_dist_text_label_, total_wp_dist_value_label_);

    // Combine rows into container
    QVBoxLayout *status_container = new QVBoxLayout;
    status_container->setContentsMargins(6, 6, 6, 6);
    status_container->setSpacing(6);
    status_container->addLayout(status_row);

    // Setup logo
    QLabel *logo_label = new QLabel(this);
    QPixmap logo = rviz_common::loadPixmap("package://waypoint_editor/icons/classes/waypoint_editor_logo.png");
    QPixmap small_logo = logo.scaledToWidth(120, Qt::SmoothTransformation);
    logo_label->setPixmap(small_logo);
    logo_label->setFixedSize(small_logo.size());
    logo_label->setAlignment(Qt::AlignCenter);
    logo_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // Logo layout (centered vertically)
    QVBoxLayout *logo_layout = new QVBoxLayout;
    logo_layout->setContentsMargins(0, 0, 0, 0);
    logo_layout->setSpacing(0);
    logo_layout->addStretch();
    logo_layout->addWidget(logo_label);
    logo_layout->addStretch();

    // Top layout: status on left, logo on right
    QHBoxLayout *top_layout = new QHBoxLayout;
    top_layout->setContentsMargins(0, 0, 0, 0);
    top_layout->setSpacing(6);
    // Status group
    QGroupBox *status_group = new QGroupBox(tr("Status"), this);
    status_group->setFont(smallFont);
    status_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    status_group->setLayout(status_container);
    // Nudge logo down to align visually with group box height (account for title bar)
    logo_layout->setContentsMargins(0, status_group->fontMetrics().height(), 0, 0);
    top_layout->addWidget(status_group);
    logo_label->hide();
    top_layout->addLayout(logo_layout);
    top_layout->setAlignment(status_group, Qt::AlignVCenter);
    top_layout->setAlignment(logo_layout,  Qt::AlignVCenter);

    // Buttons
    load_2d_map_button_    = new QPushButton("Load 2D Map", this);
    load_3d_map_button_    = new QPushButton("Load 3D Map", this);
    load_waypoints_button_ = new QPushButton("Load WPs + Routes", this);
    save_waypoints_button_ = new QPushButton("Save WPs + Routes", this);
    waypoint_set_combo_    = new QComboBox(this);
    refresh_sets_button_   = new QPushButton("Refresh", this);
    new_set_button_        = new QPushButton("Save As Set", this);
    rename_set_button_     = new QPushButton("Rename Set", this);
    delete_set_button_     = new QPushButton("Delete Set", this);
    waypoint_combo_        = new QComboBox(this);
    refresh_waypoints_button_ = new QPushButton("Refresh", this);
    go_waypoint_button_    = new QPushButton("Go", this);
    stop_navigation_button_= new QPushButton("Stop", this);
    marker_size_spin_      = new QDoubleSpinBox(this);
    undo_button_           = new QPushButton("Undo", this);
    redo_button_           = new QPushButton("Redo", this);
    clear_button_          = new QPushButton("Clear All", this);
    route_from_combo_      = new QComboBox(this);
    route_to_combo_        = new QComboBox(this);
    route_combo_           = new QComboBox(this);
    route_points_list_     = new QListWidget(this);
    available_route_points_list_ = new QListWidget(this);
    selected_route_label_  = new QLabel("SELECTED ROUTE\nNone", this);
    create_route_button_   = new QPushButton("Create Route", this);
    delete_route_button_   = new QPushButton("Delete Route", this);
    add_route_point_button_= new QPushButton("Add Route Point", this);
    attach_route_point_button_ = new QPushButton("Use Existing Point", this);
    detach_route_point_button_ = new QPushButton("Remove", this);
    detach_route_point_button_->setToolTip("Detach the selected point from this route only");
    move_route_point_earlier_button_ = new QPushButton("Move Up", this);
    move_route_point_later_button_ = new QPushButton("Move Down", this);
    delete_route_points_everywhere_button_ = new QPushButton("Delete from Map", this);
    delete_route_points_everywhere_button_->setToolTip(
        "Permanently delete selected points from every route");
    route_bidirectional_check_ = new QCheckBox("Bidirectional", this);

    // Button sizing/styling to break monotony
    for (auto *btn : {load_2d_map_button_, load_3d_map_button_, load_waypoints_button_, save_waypoints_button_}) {
        btn->setMinimumWidth(90);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    }
    for (auto *btn : {undo_button_, redo_button_}) {
        btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        btn->setMinimumWidth(60);
    }
    clear_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    clear_button_->setMinimumWidth(80);
    clear_button_->setStyleSheet(""); // match other buttons

    waypoint_set_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    waypoint_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    marker_size_spin_->setDecimals(2);
    marker_size_spin_->setRange(0.10, 1.00);
    marker_size_spin_->setSingleStep(0.05);
    marker_size_spin_->setValue(0.25);
    marker_size_spin_->setSuffix(" m");
    marker_size_spin_->setMaximumWidth(90);
    for (auto *btn : {refresh_sets_button_, new_set_button_, rename_set_button_, delete_set_button_}) {
        btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        btn->setMinimumWidth(76);
    }
    for (auto *btn : {refresh_waypoints_button_, go_waypoint_button_, stop_navigation_button_}) {
        btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        btn->setMinimumWidth(64);
    }
    go_waypoint_button_->setStyleSheet("font-weight: bold;");
    route_from_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    route_to_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    route_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    route_points_list_->setSelectionMode(QAbstractItemView::SingleSelection);
    route_points_list_->setFixedHeight(76);
    route_points_list_->setAlternatingRowColors(true);
    available_route_points_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    available_route_points_list_->setFixedHeight(76);
    available_route_points_list_->setAlternatingRowColors(true);
    selected_route_label_->setWordWrap(true);
    QFont selectedRouteFont = smallFont;
    selectedRouteFont.setPointSize(12);
    selectedRouteFont.setBold(true);
    selected_route_label_->setFont(selectedRouteFont);
    selected_route_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    selected_route_label_->setMinimumHeight(54);
    selected_route_label_->setStyleSheet(
        "QLabel { background: #073846; color: #f2fcff; "
        "border: 2px solid #19c9ef; border-radius: 5px; padding: 5px 8px; }");
    add_route_point_button_->setStyleSheet("font-weight: bold;");

    // Map section
    QHBoxLayout *map_row = new QHBoxLayout;
    map_row->setContentsMargins(6, 6, 6, 6);
    map_row->setSpacing(8);
    map_row->addWidget(load_2d_map_button_, /*stretch=*/1);
    map_row->addWidget(load_3d_map_button_, /*stretch=*/1);
    QGroupBox *map_group = new QGroupBox(tr("Map"), this);
    map_group->setFont(smallFont);
    map_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    map_group->setLayout(map_row);
    map_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    // Waypoint set section
    QHBoxLayout *set_row = new QHBoxLayout;
    set_row->setContentsMargins(6, 6, 6, 0);
    set_row->setSpacing(8);
    set_row->addWidget(new QLabel("Set", this));
    set_row->addWidget(waypoint_set_combo_, /*stretch=*/1);
    set_row->addWidget(refresh_sets_button_, /*stretch=*/0);

    QHBoxLayout *set_actions_row = new QHBoxLayout;
    set_actions_row->setContentsMargins(6, 0, 6, 0);
    set_actions_row->setSpacing(8);
    set_actions_row->addWidget(new_set_button_, /*stretch=*/1);
    set_actions_row->addWidget(rename_set_button_, /*stretch=*/1);
    set_actions_row->addWidget(delete_set_button_, /*stretch=*/1);

    QVBoxLayout *set_layout = new QVBoxLayout;
    set_layout->setContentsMargins(0, 0, 0, 0);
    set_layout->setSpacing(4);
    set_layout->addLayout(set_row);
    set_layout->addLayout(set_actions_row);

    QGroupBox *set_group = new QGroupBox(tr("Waypoint Set"), this);
    set_group->setFont(smallFont);
    set_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    set_group->setLayout(set_layout);
    set_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    // Live navigation section
    QHBoxLayout *goto_row = new QHBoxLayout;
    goto_row->setContentsMargins(6, 6, 6, 0);
    goto_row->setSpacing(8);
    goto_row->addWidget(new QLabel("Waypoint", this));
    goto_row->addWidget(waypoint_combo_, /*stretch=*/1);
    goto_row->addWidget(refresh_waypoints_button_, /*stretch=*/0);

    QHBoxLayout *goto_actions_row = new QHBoxLayout;
    goto_actions_row->setContentsMargins(6, 0, 6, 6);
    goto_actions_row->setSpacing(8);
    goto_actions_row->addWidget(go_waypoint_button_, /*stretch=*/1);
    goto_actions_row->addWidget(stop_navigation_button_, /*stretch=*/1);

    QVBoxLayout *goto_layout = new QVBoxLayout;
    goto_layout->setContentsMargins(0, 0, 0, 0);
    goto_layout->setSpacing(4);
    goto_layout->addLayout(goto_row);
    goto_layout->addLayout(goto_actions_row);

    QGroupBox *goto_group = new QGroupBox(tr("Live Navigation"), this);
    goto_group->setFont(smallFont);
    goto_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    goto_group->setLayout(goto_layout);
    goto_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    QHBoxLayout *marker_size_row = new QHBoxLayout;
    marker_size_row->setContentsMargins(6, 0, 6, 0);
    marker_size_row->setSpacing(8);
    marker_size_row->addWidget(new QLabel("Marker size", this));
    marker_size_row->addWidget(marker_size_spin_, /*stretch=*/0);
    marker_size_row->addStretch();

    QHBoxLayout *wp_row1 = new QHBoxLayout;
    wp_row1->setContentsMargins(6, 0, 6, 0);
    wp_row1->setSpacing(8);
    wp_row1->addWidget(undo_button_, /*stretch=*/1);
    wp_row1->addWidget(redo_button_, /*stretch=*/1);
    wp_row1->addWidget(clear_button_, /*stretch=*/1);

    QHBoxLayout *wp_row2 = new QHBoxLayout;
    wp_row2->setContentsMargins(6, 0, 6, 6);
    wp_row2->setSpacing(8);
    wp_row2->addWidget(load_waypoints_button_, /*stretch=*/1);
    wp_row2->addWidget(save_waypoints_button_, /*stretch=*/1);

    QVBoxLayout *wp_layout = new QVBoxLayout;
    wp_layout->setContentsMargins(0, 0, 0, 0);
    wp_layout->setSpacing(4);
    wp_layout->addLayout(wp_row2);
    wp_layout->addLayout(wp_row1);
    wp_layout->addLayout(marker_size_row);

    QGroupBox *action_group = new QGroupBox(tr("Edit Waypoints"), this);
    action_group->setFont(smallFont);
    action_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    action_group->setLayout(wp_layout);
    action_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    QHBoxLayout *route_select_row = new QHBoxLayout;
    route_select_row->setContentsMargins(6, 6, 6, 0);
    route_select_row->setSpacing(6);
    route_select_row->addWidget(new QLabel("1. Select route", this));

    QHBoxLayout *route_select_combo_row = new QHBoxLayout;
    route_select_combo_row->setContentsMargins(6, 0, 6, 0);
    route_select_combo_row->addWidget(route_combo_, 1);

    QHBoxLayout *route_from_row = new QHBoxLayout;
    route_from_row->setContentsMargins(6, 4, 6, 0);
    route_from_row->setSpacing(6);
    route_from_row->addWidget(new QLabel("From", this));
    route_from_row->addWidget(route_from_combo_, 1);

    QHBoxLayout *route_to_row = new QHBoxLayout;
    route_to_row->setContentsMargins(6, 0, 6, 0);
    route_to_row->setSpacing(6);
    route_to_row->addWidget(new QLabel("To", this));
    route_to_row->addWidget(route_to_combo_, 1);

    QHBoxLayout *route_create_row = new QHBoxLayout;
    route_create_row->setContentsMargins(6, 0, 6, 0);
    route_create_row->setSpacing(6);
    route_create_row->addWidget(route_bidirectional_check_);
    route_create_row->addStretch();
    route_bidirectional_check_->setChecked(true);

    QHBoxLayout *route_create_button_row = new QHBoxLayout;
    route_create_button_row->setContentsMargins(6, 0, 6, 0);
    route_create_button_row->addWidget(create_route_button_, 1);

    QLabel *route_points_label = new QLabel("2. Points in selected route (travel order)", this);
    route_points_label->setStyleSheet("font-weight: bold;");
    route_points_label->setContentsMargins(6, 6, 6, 0);

    QHBoxLayout *route_points_row = new QHBoxLayout;
    route_points_row->setContentsMargins(6, 0, 6, 0);
    route_points_row->addWidget(route_points_list_, 1);

    QHBoxLayout *route_point_actions_row = new QHBoxLayout;
    route_point_actions_row->setContentsMargins(6, 0, 6, 0);
    route_point_actions_row->setSpacing(6);
    route_point_actions_row->addWidget(move_route_point_earlier_button_, 1);
    route_point_actions_row->addWidget(move_route_point_later_button_, 1);
    route_point_actions_row->addWidget(detach_route_point_button_, 1);

    QLabel *available_points_label = new QLabel("3. All route points on this map", this);
    available_points_label->setStyleSheet("font-weight: bold;");
    available_points_label->setContentsMargins(6, 6, 6, 0);

    QHBoxLayout *available_points_row = new QHBoxLayout;
    available_points_row->setContentsMargins(6, 0, 6, 0);
    available_points_row->addWidget(available_route_points_list_, 1);

    QHBoxLayout *available_point_actions_row = new QHBoxLayout;
    available_point_actions_row->setContentsMargins(6, 0, 6, 0);
    available_point_actions_row->setSpacing(6);
    attach_route_point_button_->setText("Add to Route");
    attach_route_point_button_->setToolTip(
        "Add all selected map points to the end of this route");
    available_point_actions_row->addWidget(attach_route_point_button_, 1);
    available_point_actions_row->addWidget(delete_route_points_everywhere_button_, 1);

    QHBoxLayout *route_actions_row = new QHBoxLayout;
    route_actions_row->setContentsMargins(6, 0, 6, 0);
    route_actions_row->setSpacing(6);
    add_route_point_button_->setText("Add New Point to Selected Route");
    route_actions_row->addWidget(add_route_point_button_, 1);

    QHBoxLayout *route_delete_row = new QHBoxLayout;
    route_delete_row->setContentsMargins(6, 0, 6, 6);
    route_delete_row->addWidget(delete_route_button_, 1);

    QLabel *route_help = new QLabel(
        "Select a route above. Only that route stays bright on the map. "
        "Add New Point switches to the map tool; click the map to place it. "
        "The lower list can add existing points to this route or permanently "
        "delete them from every route.", this);
    route_help->setWordWrap(true);
    route_help->setStyleSheet("color: #666;");
    route_help->setContentsMargins(6, 4, 6, 6);

    QVBoxLayout *route_layout = new QVBoxLayout;
    route_layout->setContentsMargins(0, 0, 0, 0);
    route_layout->setSpacing(4);
    route_layout->addLayout(route_select_row);
    route_layout->addLayout(route_select_combo_row);
    route_layout->addWidget(route_points_label);
    route_layout->addLayout(route_points_row);
    route_layout->addLayout(route_point_actions_row);
    route_layout->addLayout(route_actions_row);
    route_layout->addWidget(available_points_label);
    route_layout->addLayout(available_points_row);
    route_layout->addLayout(available_point_actions_row);
    route_layout->addLayout(route_delete_row);
    route_layout->addWidget(route_help);

    QGroupBox *route_group = new QGroupBox(tr("Preferred Routes"), this);
    route_group->setFont(smallFont);
    route_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    route_group->setLayout(route_layout);
    route_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    QLabel *new_route_help = new QLabel(
        "Create the connection first, then return to Routes to add its ordered points.",
        this);
    new_route_help->setWordWrap(true);
    new_route_help->setStyleSheet("color: #666;");
    new_route_help->setContentsMargins(6, 4, 6, 6);

    QVBoxLayout *new_route_layout = new QVBoxLayout;
    new_route_layout->setContentsMargins(0, 0, 0, 0);
    new_route_layout->setSpacing(4);
    new_route_layout->addLayout(route_from_row);
    new_route_layout->addLayout(route_to_row);
    new_route_layout->addLayout(route_create_row);
    new_route_layout->addLayout(route_create_button_row);
    new_route_layout->addWidget(new_route_help);

    QGroupBox *new_route_group = new QGroupBox(tr("Create Preferred Route"), this);
    new_route_group->setFont(smallFont);
    new_route_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    new_route_group->setLayout(new_route_layout);
    new_route_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    QLabel *auto_label = new QLabel("Auto Δd :", this);
    auto_label->setFont(boldFont);
    auto_label->setContentsMargins(0, 0, 0, 0);

    auto_distance_spin_ = new QDoubleSpinBox(this);
    auto_distance_spin_->setDecimals(2);
    auto_distance_spin_->setRange(0.01, 1000.0);
    auto_distance_spin_->setSingleStep(0.10);
    auto_distance_spin_->setValue(1.00);
    auto_distance_spin_->setSuffix(" m");
    auto_distance_spin_->setFont(smallFont);
    auto_distance_spin_->setMaximumWidth(90);
    auto_distance_spin_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    QLabel *auto_topic_label = new QLabel("Topic", this);
    auto_topic_label->setFont(boldFont);
    auto_topic_label->setContentsMargins(0, 0, 0, 0);
    auto_topic_edit_ = new QLineEdit("/amcl_pose", this);
    auto_topic_edit_->setFont(smallFont);
    auto_topic_edit_->setMinimumWidth(140);

    QLabel *auto_type_label = new QLabel("Type", this);
    auto_type_label->setFont(boldFont);
    auto_type_label->setContentsMargins(0, 0, 0, 0);
    auto_type_combo_ = new QComboBox(this);
    auto_type_combo_->setFont(smallFont);
    auto_type_combo_->addItem("PoseWithCovarianceStamped");
    auto_type_combo_->addItem("PoseStamped");

    auto_toggle_button_ = new QPushButton("Start Auto Capture", this);
    auto_toggle_button_->setCheckable(true);
    auto_toggle_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QHBoxLayout *auto_row1 = new QHBoxLayout;
    auto_row1->setContentsMargins(0, 0, 0, 0);
    auto_row1->setSpacing(6);
    auto_row1->addWidget(auto_label);
    auto_row1->addWidget(auto_distance_spin_, /*stretch=*/0);
    auto_row1->addStretch();
    auto_row1->addWidget(auto_toggle_button_, /*stretch=*/0);

    QHBoxLayout *auto_row2 = new QHBoxLayout;
    auto_row2->setContentsMargins(0, 0, 0, 0);
    auto_row2->setSpacing(6);
    auto_row2->addWidget(auto_topic_label);
    auto_row2->addWidget(auto_topic_edit_, /*stretch=*/1);
    auto_row2->addWidget(auto_type_label);
    auto_row2->addWidget(auto_type_combo_, /*stretch=*/0);

    QVBoxLayout *auto_layout = new QVBoxLayout;
    auto_layout->setContentsMargins(6, 6, 6, 6);
    auto_layout->setSpacing(6);
    auto_layout->addLayout(auto_row1);
    auto_layout->addLayout(auto_row2);
    auto_layout->addLayout(last_wp_dist_row);
    auto_layout->addLayout(total_wp_dist_row);

    QGroupBox *auto_group = new QGroupBox(tr("Auto Capture"), this);
    auto_group->setFont(smallFont);
    auto_group->setStyleSheet("QGroupBox::title { font-weight: bold; }");
    auto_group->setLayout(auto_layout);
    auto_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    // Main layout
    layout_ = new QVBoxLayout;
    layout_->setContentsMargins(6, 6, 6, 6);
    layout_->setSpacing(8);
    layout_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    layout_->setSizeConstraint(QLayout::SetMinimumSize);

    layout_->addLayout(top_layout);
    layout_->addSpacing(8);
    QTabWidget *tabs = new QTabWidget(this);

    QWidget *waypoints_page = new QWidget(tabs);
    QVBoxLayout *waypoints_layout = new QVBoxLayout(waypoints_page);
    waypoints_layout->setContentsMargins(4, 6, 4, 4);
    waypoints_layout->addWidget(map_group);
    waypoints_layout->addWidget(set_group);
    waypoints_layout->addWidget(action_group);
    waypoints_layout->addStretch();
    tabs->addTab(waypoints_page, tr("Waypoints"));

    QWidget *routes_page = new QWidget(tabs);
    QVBoxLayout *routes_layout = new QVBoxLayout(routes_page);
    routes_layout->setContentsMargins(4, 6, 4, 4);
    routes_layout->addWidget(route_group);
    routes_layout->addStretch();
    tabs->addTab(routes_page, tr("Routes"));

    QWidget *new_route_page = new QWidget(tabs);
    QVBoxLayout *new_route_page_layout = new QVBoxLayout(new_route_page);
    new_route_page_layout->setContentsMargins(4, 6, 4, 4);
    new_route_page_layout->addWidget(new_route_group);
    new_route_page_layout->addStretch();
    tabs->addTab(new_route_page, tr("New Route"));

    QWidget *navigation_page = new QWidget(tabs);
    QVBoxLayout *navigation_layout = new QVBoxLayout(navigation_page);
    navigation_layout->setContentsMargins(4, 6, 4, 4);
    navigation_layout->addWidget(goto_group);
    navigation_layout->addStretch();
    tabs->addTab(navigation_page, tr("Navigate"));

    QWidget *auto_page = new QWidget(tabs);
    QVBoxLayout *auto_page_layout = new QVBoxLayout(auto_page);
    auto_page_layout->setContentsMargins(4, 6, 4, 4);
    auto_page_layout->addWidget(auto_group);
    auto_page_layout->addStretch();
    tabs->addTab(auto_page, tr("Auto"));

    layout_->addWidget(tabs, 1);
    layout_->addStretch();

    QWidget *content_widget = new QWidget(this);
    content_widget->setLayout(layout_);

    QScrollArea *scroll_area = new QScrollArea(this);
    scroll_area->setWidget(content_widget);
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QVBoxLayout *root_layout = new QVBoxLayout;
    root_layout->setContentsMargins(0, 0, 0, 0);
    root_layout->setSpacing(4);
    root_layout->addWidget(selected_route_label_);
    root_layout->addWidget(scroll_area);
    setLayout(root_layout);
    setMinimumWidth(320);
    setMaximumWidth(560);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    // Connect signals
    connect(load_2d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad2DMapButtonClick);
    connect(load_3d_map_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoad3DMapButtonClick);
    connect(load_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onLoadWaypointsButtonClick);
    connect(save_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onSaveWaypointsButtonClick);
    connect(waypoint_set_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WaypointEditorPanel::onWaypointSetSelected);
    connect(refresh_sets_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRefreshSetsButtonClick);
    connect(new_set_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onNewSetButtonClick);
    connect(rename_set_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRenameSetButtonClick);
    connect(delete_set_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onDeleteSetButtonClick);
    connect(refresh_waypoints_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRefreshWaypointsButtonClick);
    connect(go_waypoint_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onGoToWaypointButtonClick);
    connect(stop_navigation_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onStopNavigationButtonClick);
    connect(marker_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &WaypointEditorPanel::onMarkerSizeChanged);
    connect(undo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onUndoWaypointsButtonClick);
    connect(redo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onRedoWaypointsButtonClick);
    connect(clear_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onClearWaypointsButtonClick);
    connect(create_route_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onCreateRouteButtonClick);
    connect(delete_route_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onDeleteRouteButtonClick);
    connect(add_route_point_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onAddRoutePointButtonClick);
    connect(attach_route_point_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onAttachRoutePointButtonClick);
    connect(detach_route_point_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onDetachRoutePointButtonClick);
    connect(move_route_point_earlier_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onMoveRoutePointEarlierButtonClick);
    connect(move_route_point_later_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onMoveRoutePointLaterButtonClick);
    connect(delete_route_points_everywhere_button_, &QPushButton::clicked, this, &WaypointEditorPanel::onDeleteRoutePointsEverywhereButtonClick);
    connect(route_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WaypointEditorPanel::onRouteSelected);
    connect(route_bidirectional_check_, &QCheckBox::toggled, this, &WaypointEditorPanel::onRouteBidirectionalChanged);
    connect(auto_toggle_button_, &QPushButton::toggled, this, &WaypointEditorPanel::onAutoToggle);
}

WaypointEditorPanel::~WaypointEditorPanel() {}

void WaypointEditorPanel::onInitialize()
{
    nh_               = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    load_map_client_  = nh_->create_client<sahabat_interfaces::srv::LoadMap>("/operator/maps/load");
    nav2_load_map_client_ = nh_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    load_client_      = nh_->create_client<std_srvs::srv::Trigger>("load_waypoints");
    save_client_      = nh_->create_client<std_srvs::srv::Trigger>("save_waypoints");
    undo_client_      = nh_->create_client<std_srvs::srv::Trigger>("undo_waypoints");
    redo_client_      = nh_->create_client<std_srvs::srv::Trigger>("redo_waypoints");
    clear_client_     = nh_->create_client<std_srvs::srv::Trigger>("clear_waypoints");
    auto_start_client_= nh_->create_client<std_srvs::srv::Trigger>("start_auto_waypoints");
    auto_stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("stop_auto_waypoints");
    list_sets_client_ = nh_->create_client<sahabat_interfaces::srv::ListWaypointSets>("waypoint_editor/list_sets");
    manage_set_client_= nh_->create_client<sahabat_interfaces::srv::ManageWaypointSet>("waypoint_editor/manage_set");
    get_waypoints_client_ = nh_->create_client<sahabat_interfaces::srv::GetWaypoints>("waypoint_editor/get_waypoints");
    get_graph_client_ = nh_->create_client<sahabat_interfaces::srv::GetWaypointGraph>("waypoint_editor/get_graph");
    edit_route_client_ = nh_->create_client<sahabat_interfaces::srv::EditRoute>("waypoint_editor/edit_route");
    control_lease_client_ = nh_->create_client<sahabat_interfaces::srv::ControlLease>("/operator/control_lease");
    patrol_client_ = nh_->create_client<sahabat_interfaces::srv::PatrolCommand>("/operator/patrol");
    auto_distance_pub_= nh_->create_publisher<std_msgs::msg::Float64>("auto_waypoint_min_distance", rclcpp::QoS(1).transient_local());

    last_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "last_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            QMetaObject::invokeMethod(last_wp_dist_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m"));
        }
    );

    total_wp_dist_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "total_wp_dist", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            QMetaObject::invokeMethod(total_wp_dist_value_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(msg->data, 'f', 3) + " m"));
        }
    );

    graph_changed_sub_ = nh_->create_subscription<std_msgs::msg::Empty>(
        "waypoint_editor/graph_changed", 10,
        [this](std_msgs::msg::Empty::SharedPtr) {
            QMetaObject::invokeMethod(
                this,
                [this]() { refreshWaypoints(); },
                Qt::QueuedConnection);
        }
    );
    route_point_selected_sub_ = nh_->create_subscription<std_msgs::msg::String>(
        "waypoint_editor/route_point_selected", rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            const QString point_id = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(
                this,
                [this, point_id]() {
                    for (int row = 0; row < route_points_list_->count(); ++row) {
                        auto *item = route_points_list_->item(row);
                        if (item->data(Qt::UserRole).toString() == point_id) {
                            route_points_list_->setCurrentItem(item);
                            route_points_list_->scrollToItem(
                                item, QAbstractItemView::PositionAtCenter);
                            postStatusMessage(tr("Selected route point %1").arg(item->text()));
                            break;
                        }
                    }
                },
                Qt::QueuedConnection);
        });
    
    cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    std::string maps_directory = "~/sahabat_ws/maps";
    std::string map_id = "rdlfront";
    nh_->get_parameter("maps_directory", maps_directory);
    nh_->get_parameter("map_id", map_id);
    map_id_ = QString::fromStdString(map_id);
    map_dialog_directory_ = expandUserPath(QString::fromStdString(maps_directory));
    double marker_size = marker_size_spin_->value();
    if (nh_->get_parameter("waypoint_marker_size", marker_size)) {
        marker_size_spin_->setValue(marker_size);
    }
    refreshWaypointSets();
    refreshWaypoints();
}

void WaypointEditorPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void WaypointEditorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void WaypointEditorPanel::refreshWaypointSets()
{
    if (!list_sets_client_ || !list_sets_client_->wait_for_service(std::chrono::milliseconds(500))) {
        postStatusMessage(tr("Waypoint-set service unavailable"));
        return;
    }

    auto req = std::make_shared<sahabat_interfaces::srv::ListWaypointSets::Request>();
    list_sets_client_->async_send_request(req,
        [this](rclcpp::Client<sahabat_interfaces::srv::ListWaypointSets>::SharedFuture future) {
            try {
                auto response = future.get();
                QMetaObject::invokeMethod(this, [this, response]() {
                    QSignalBlocker blocker(waypoint_set_combo_);
                    waypoint_set_combo_->clear();
                    int active_index = -1;
                    for (int i = 0; i < static_cast<int>(response->sets.size()); ++i) {
                        const auto &set = response->sets[static_cast<std::size_t>(i)];
                        const QString id = QString::fromStdString(set.id);
                        const QString label = QString::fromStdString(set.name);
                        waypoint_set_combo_->addItem(label, id);
                        if (set.id == response->active_set_id) {
                            active_index = i;
                        }
                    }
                    if (active_index >= 0) {
                        waypoint_set_combo_->setCurrentIndex(active_index);
                    }
                    postStatusMessage(QString::fromStdString(response->message));
                    refreshWaypoints();
                }, Qt::QueuedConnection);
            } catch (...) {
                postStatusMessage(tr("Failed to refresh waypoint sets"));
            }
        });
}

void WaypointEditorPanel::manageWaypointSet(uint8_t action, const QString &set_id, const QString &name)
{
    if (!manage_set_client_ || !manage_set_client_->wait_for_service(std::chrono::seconds(1))) {
        postStatusMessage(tr("Waypoint-set service unavailable"));
        return;
    }

    auto req = std::make_shared<sahabat_interfaces::srv::ManageWaypointSet::Request>();
    req->action = action;
    req->set_id = set_id.toStdString();
    req->name = name.toStdString();
    manage_set_client_->async_send_request(req,
        [this, action](rclcpp::Client<sahabat_interfaces::srv::ManageWaypointSet>::SharedFuture future) {
            try {
                auto response = future.get();
                postStatusMessage(QString::fromStdString(response->message));
                if (response->success) {
                    refreshWaypointSets();
                    if (action == sahabat_interfaces::srv::ManageWaypointSet::Request::SELECT) {
                        refreshWaypoints();
                    }
                }
            } catch (...) {
                postStatusMessage(tr("Waypoint-set action failed"));
            }
        });
}

void WaypointEditorPanel::onWaypointSetSelected(int index)
{
    if (index < 0) {
        return;
    }
    manageWaypointSet(
        sahabat_interfaces::srv::ManageWaypointSet::Request::SELECT,
        waypoint_set_combo_->itemData(index).toString(),
        QString());
}

void WaypointEditorPanel::onRefreshSetsButtonClick()
{
    refreshWaypointSets();
}

void WaypointEditorPanel::refreshWaypoints()
{
    if (!get_graph_client_ || !get_graph_client_->wait_for_service(std::chrono::milliseconds(300))) {
        QSignalBlocker waypoint_blocker(waypoint_combo_);
        QSignalBlocker route_blocker(route_combo_);
        QSignalBlocker route_points_blocker(route_points_list_);
        QSignalBlocker available_points_blocker(available_route_points_list_);
        waypoint_combo_->clear();
        route_combo_->clear();
        route_points_list_->clear();
        available_route_points_list_->clear();
        return;
    }

    auto req = std::make_shared<sahabat_interfaces::srv::GetWaypointGraph::Request>();
    req->map_id = map_id_.toStdString();
    req->set_id = waypoint_set_combo_->currentData().toString().toStdString();
    get_graph_client_->async_send_request(req,
        [this](rclcpp::Client<sahabat_interfaces::srv::GetWaypointGraph>::SharedFuture future) {
            try {
                auto response = future.get();
                QMetaObject::invokeMethod(this, [this, response]() {
                    const QString previous_route = route_combo_->currentData().toString();
                    const QString previous_route_point = route_points_list_->currentItem()
                        ? route_points_list_->currentItem()->data(Qt::UserRole).toString()
                        : QString();
                    QSignalBlocker waypoint_blocker(waypoint_combo_);
                    QSignalBlocker from_blocker(route_from_combo_);
                    QSignalBlocker to_blocker(route_to_combo_);
                    QSignalBlocker route_blocker(route_combo_);
                    QSignalBlocker route_points_blocker(route_points_list_);
                    QSignalBlocker available_points_blocker(available_route_points_list_);
                    QSignalBlocker bidirectional_blocker(route_bidirectional_check_);
                    waypoint_combo_->clear();
                    route_from_combo_->clear();
                    route_to_combo_->clear();
                    route_combo_->clear();
                    route_points_list_->clear();
                    available_route_points_list_->clear();
                    if (response->waypoints.empty()) {
                        postStatusMessage(QString::fromStdString(
                            response->message.empty()
                                ? "No waypoints in current set"
                                : response->message));
                        return;
                    }
                    for (const auto &waypoint : response->waypoints) {
                        const QString label = QString::fromStdString(waypoint.name.empty() ? waypoint.id : waypoint.name);
                        const QString id = QString::fromStdString(waypoint.id);
                        waypoint_combo_->addItem(label, id);
                        route_from_combo_->addItem(label, id);
                        route_to_combo_->addItem(label, id);
                    }
                    if (route_to_combo_->count() > 1) {
                        route_to_combo_->setCurrentIndex(1);
                    }
                    int selected_route = -1;
                    const QString selected_route_id = response->active_segment_id.empty()
                        ? previous_route
                        : QString::fromStdString(response->active_segment_id);
                    std::map<std::string, std::pair<std::string, int>> shared_points;
                    for (int i = 0; i < static_cast<int>(response->segments.size()); ++i) {
                        const auto &segment = response->segments.at(static_cast<std::size_t>(i));
                        const QString id = QString::fromStdString(segment.id);
                        const QString direction = segment.bidirectional ? " ↔ " : " → ";
                        const QString label = QString::fromStdString(
                            segment.name.empty() ? segment.id : segment.name) +
                            direction + QString::number(segment.via_points.size()) + " point(s)";
                        route_combo_->addItem(label, id);
                        route_combo_->setItemData(i, segment.bidirectional, Qt::UserRole + 1);
                        route_combo_->setItemData(
                            i,
                            QString::fromStdString(segment.name.empty() ? segment.id : segment.name),
                            Qt::UserRole + 2);
                        route_combo_->setItemData(
                            i,
                            static_cast<int>(segment.via_points.size()),
                            Qt::UserRole + 3);
                        for (const auto &point : segment.via_points) {
                            auto &entry = shared_points[point.id];
                            entry.first = point.name.empty() ? point.id : point.name;
                            entry.second += 1;
                        }
                        if (id == selected_route_id) {
                            selected_route = i;
                        }
                    }
                    if (selected_route >= 0) {
                        route_combo_->setCurrentIndex(selected_route);
                    }
                    if (route_combo_->currentIndex() >= 0) {
                        route_bidirectional_check_->setChecked(
                            route_combo_->currentData(Qt::UserRole + 1).toBool());
                        updateSelectedRouteLabel(route_combo_->currentIndex());
                    } else {
                        updateSelectedRouteLabel(-1);
                    }

                    std::set<std::string> points_in_selected_route;
                    const std::string active_route_id =
                        route_combo_->currentData().toString().toStdString();
                    const auto active_route = std::find_if(
                        response->segments.begin(), response->segments.end(),
                        [&active_route_id](const sahabat_interfaces::msg::RouteSegment &segment) {
                            return segment.id == active_route_id;
                        });
                    if (active_route != response->segments.end()) {
                        for (std::size_t index = 0; index < active_route->via_points.size(); ++index) {
                            const auto &point = active_route->via_points[index];
                            points_in_selected_route.insert(point.id);
                            const auto details = shared_points.find(point.id);
                            const int usage_count = details == shared_points.end()
                                ? 1
                                : details->second.second;
                            const QString shared_suffix = usage_count > 1
                                ? tr("  [shared by %1 routes]").arg(usage_count)
                                : QString();
                            const QString point_id = QString::fromStdString(point.id);
                            const QString point_name = QString::fromStdString(point.name);
                            const QString point_label =
                                point_name.isEmpty() || point_name == point_id
                                ? point_id
                                : point_id + " · " + point_name;
                            auto *item = new QListWidgetItem(
                                tr("%1. %2%3")
                                    .arg(index + 1)
                                    .arg(point_label)
                                    .arg(shared_suffix),
                                route_points_list_);
                            item->setData(Qt::UserRole, point_id);
                            if (point_id == previous_route_point) {
                                route_points_list_->setCurrentItem(item);
                            }
                        }
                    }
                    for (const auto &[id, details] : shared_points) {
                        const bool in_selected_route = points_in_selected_route.count(id) > 0;
                        const QString usage = details.second == 1
                            ? tr("1 route")
                            : tr("%1 routes").arg(details.second);
                        const QString membership = in_selected_route
                            ? tr("  [in selected]")
                            : QString();
                        const QString point_id = QString::fromStdString(id);
                        const QString point_name = QString::fromStdString(details.first);
                        const QString point_label =
                            point_name.isEmpty() || point_name == point_id
                            ? point_id
                            : point_id + " · " + point_name;
                        auto *item = new QListWidgetItem(
                            point_label + "  -  " + usage + membership,
                            available_route_points_list_);
                        item->setData(Qt::UserRole, point_id);
                        if (in_selected_route) {
                            item->setForeground(QColor(110, 110, 110));
                        }
                    }
                    postStatusMessage(QString::fromStdString(response->message));
                }, Qt::QueuedConnection);
            } catch (...) {
                postStatusMessage(tr("Failed to refresh waypoints"));
            }
        });
}

void WaypointEditorPanel::onRefreshWaypointsButtonClick()
{
    refreshWaypoints();
}

void WaypointEditorPanel::updateSelectedRouteLabel(int index)
{
    if (index < 0 || index >= route_combo_->count()) {
        selected_route_label_->setText(tr("SELECTED ROUTE\nNone"));
        return;
    }
    const QString name = route_combo_->itemData(index, Qt::UserRole + 2).toString();
    const int point_count = route_combo_->itemData(index, Qt::UserRole + 3).toInt();
    const QString direction = route_combo_->itemData(index, Qt::UserRole + 1).toBool()
        ? tr("two-way")
        : tr("one-way");
    selected_route_label_->setText(
        tr("SELECTED ROUTE\n%1 · %2 pts · %3")
            .arg(name)
            .arg(point_count)
            .arg(direction));
}

void WaypointEditorPanel::sendRouteEdit(
    uint8_t action,
    const QString &segment_id,
    const QString &from_waypoint_id,
    const QString &to_waypoint_id,
    bool bidirectional,
    const QString &route_point_id,
    const QStringList &route_point_ids)
{
    if (!edit_route_client_ ||
        !edit_route_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
        postStatusMessage(tr("Route editor service unavailable"));
        return;
    }
    auto req = std::make_shared<sahabat_interfaces::srv::EditRoute::Request>();
    req->action = action;
    req->segment_id = segment_id.toStdString();
    req->from_waypoint_id = from_waypoint_id.toStdString();
    req->to_waypoint_id = to_waypoint_id.toStdString();
    req->bidirectional = bidirectional;
    req->route_point_id = route_point_id.toStdString();
    for (const auto &point_id : route_point_ids) {
        req->route_point_ids.push_back(point_id.toStdString());
    }
    edit_route_client_->async_send_request(
        req,
        [this, action](rclcpp::Client<sahabat_interfaces::srv::EditRoute>::SharedFuture future) {
            try {
                const auto response = future.get();
                postStatusMessage(QString::fromStdString(response->message));
                if (response->success &&
                    action != sahabat_interfaces::srv::EditRoute::Request::ARM_ADD_POINT &&
                    action != sahabat_interfaces::srv::EditRoute::Request::CANCEL_ADD_POINT)
                {
                    refreshWaypoints();
                }
            } catch (...) {
                postStatusMessage(tr("Route edit failed"));
            }
        });
}

void WaypointEditorPanel::onCreateRouteButtonClick()
{
    if (route_from_combo_->currentIndex() < 0 ||
        route_to_combo_->currentIndex() < 0)
    {
        postStatusMessage(tr("Select route endpoints first"));
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::CREATE,
        QString(),
        route_from_combo_->currentData().toString(),
        route_to_combo_->currentData().toString(),
        route_bidirectional_check_->isChecked());
}

void WaypointEditorPanel::onDeleteRouteButtonClick()
{
    if (route_combo_->currentIndex() < 0) {
        postStatusMessage(tr("Select a route first"));
        return;
    }
    const auto answer = QMessageBox::question(
        this,
        tr("Delete Preferred Route"),
        tr("Delete route '%1' and all of its route points?")
            .arg(route_combo_->currentText()));
    if (answer == QMessageBox::Yes) {
        sendRouteEdit(
            sahabat_interfaces::srv::EditRoute::Request::DELETE,
            route_combo_->currentData().toString());
    }
}

void WaypointEditorPanel::onAddRoutePointButtonClick()
{
    if (route_combo_->currentIndex() < 0) {
        postStatusMessage(tr("Create or select a route first"));
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::ARM_ADD_POINT,
        route_combo_->currentData().toString());

    auto *tool_manager = getDisplayContext()->getToolManager();
    for (int index = 0; index < tool_manager->numTools(); ++index) {
        auto *tool = tool_manager->getTool(index);
        if (tool && tool->getClassId() == "waypoint_editor/WaypointEditorTool") {
            tool_manager->setCurrentTool(tool);
            break;
        }
    }
}

void WaypointEditorPanel::onAttachRoutePointButtonClick()
{
    if (route_combo_->currentIndex() < 0) {
        postStatusMessage(tr("Select a route first"));
        return;
    }
    const auto selected_items = available_route_points_list_->selectedItems();
    if (selected_items.isEmpty()) {
        postStatusMessage(tr("Select one or more points from the map list"));
        return;
    }
    QStringList point_ids;
    for (const auto *item : selected_items) {
        point_ids.push_back(item->data(Qt::UserRole).toString());
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::ATTACH_EXISTING_POINTS,
        route_combo_->currentData().toString(),
        QString(),
        QString(),
        true,
        QString(),
        point_ids);
}

void WaypointEditorPanel::onDetachRoutePointButtonClick()
{
    auto *item = route_points_list_->currentItem();
    if (route_combo_->currentIndex() < 0 || item == nullptr) {
        postStatusMessage(tr("Select a point in the current route list"));
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::REMOVE_POINT_FROM_ROUTE,
        route_combo_->currentData().toString(),
        QString(),
        QString(),
        true,
        item->data(Qt::UserRole).toString());
}

void WaypointEditorPanel::onMoveRoutePointEarlierButtonClick()
{
    auto *item = route_points_list_->currentItem();
    if (route_combo_->currentIndex() < 0 || item == nullptr) {
        postStatusMessage(tr("Select a point in the current route list"));
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::MOVE_POINT_EARLIER,
        route_combo_->currentData().toString(),
        QString(),
        QString(),
        true,
        item->data(Qt::UserRole).toString());
}

void WaypointEditorPanel::onMoveRoutePointLaterButtonClick()
{
    auto *item = route_points_list_->currentItem();
    if (route_combo_->currentIndex() < 0 || item == nullptr) {
        postStatusMessage(tr("Select a point in the current route list"));
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::MOVE_POINT_LATER,
        route_combo_->currentData().toString(),
        QString(),
        QString(),
        true,
        item->data(Qt::UserRole).toString());
}

void WaypointEditorPanel::onDeleteRoutePointsEverywhereButtonClick()
{
    const auto selected_items = available_route_points_list_->selectedItems();
    if (selected_items.isEmpty()) {
        postStatusMessage(tr("Select one or more points from the map list"));
        return;
    }
    const auto answer = QMessageBox::warning(
        this,
        tr("Delete Route Points from Map"),
        tr("Delete %1 selected point(s) from every route?\n\n"
           "This is global and cannot be undone after Save WPs + Routes.")
            .arg(selected_items.size()),
        QMessageBox::Cancel | QMessageBox::Yes,
        QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
        return;
    }
    QStringList point_ids;
    for (const auto *item : selected_items) {
        point_ids.push_back(item->data(Qt::UserRole).toString());
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::DELETE_POINTS_EVERYWHERE,
        QString(),
        QString(),
        QString(),
        true,
        QString(),
        point_ids);
}

void WaypointEditorPanel::onRouteSelected(int index)
{
    if (index < 0) {
        return;
    }
    {
        QSignalBlocker blocker(route_bidirectional_check_);
        route_bidirectional_check_->setChecked(
            route_combo_->itemData(index, Qt::UserRole + 1).toBool());
    }
    updateSelectedRouteLabel(index);
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::SELECT,
        route_combo_->itemData(index).toString());
}

void WaypointEditorPanel::onRouteBidirectionalChanged(bool checked)
{
    if (route_combo_->currentIndex() < 0) {
        return;
    }
    sendRouteEdit(
        sahabat_interfaces::srv::EditRoute::Request::SET_BIDIRECTIONAL,
        route_combo_->currentData().toString(),
        QString(),
        QString(),
        checked);
}

void WaypointEditorPanel::releaseControlLease(const std::string &lease_id)
{
    if (lease_id.empty() || !control_lease_client_) {
        return;
    }
    auto req = std::make_shared<sahabat_interfaces::srv::ControlLease::Request>();
    req->action = sahabat_interfaces::srv::ControlLease::Request::RELEASE;
    req->client_id = "rviz_waypoint_editor_panel";
    req->lease_id = lease_id;
    control_lease_client_->async_send_request(req);
}

void WaypointEditorPanel::sendPatrolCommand(uint8_t command, const QString &waypoint_id)
{
    if (!control_lease_client_ || !patrol_client_) {
        postStatusMessage(tr("Operator services unavailable"));
        return;
    }
    if (!control_lease_client_->wait_for_service(std::chrono::milliseconds(500)) ||
        !patrol_client_->wait_for_service(std::chrono::milliseconds(500))) {
        postStatusMessage(tr("Operator services unavailable"));
        return;
    }

    auto lease_req = std::make_shared<sahabat_interfaces::srv::ControlLease::Request>();
    lease_req->action = sahabat_interfaces::srv::ControlLease::Request::ACQUIRE;
    lease_req->client_id = "rviz_waypoint_editor_panel";
    control_lease_client_->async_send_request(lease_req,
        [this, command, waypoint_id](rclcpp::Client<sahabat_interfaces::srv::ControlLease>::SharedFuture lease_future) {
            auto lease = lease_future.get();
            if (!lease->success) {
                postStatusMessage(QString::fromStdString(lease->message));
                return;
            }

            auto req = std::make_shared<sahabat_interfaces::srv::PatrolCommand::Request>();
            req->command = command;
            req->set_id = waypoint_set_combo_->currentData().toString().toStdString();
            req->waypoint_id = waypoint_id.toStdString();
            req->loop = false;
            req->lease_id = lease->lease_id;
            patrol_client_->async_send_request(req,
                [this, lease_id = lease->lease_id](rclcpp::Client<sahabat_interfaces::srv::PatrolCommand>::SharedFuture patrol_future) {
                    auto response = patrol_future.get();
                    postStatusMessage(QString::fromStdString(response->message));
                    releaseControlLease(lease_id);
                });
        });
}

void WaypointEditorPanel::onGoToWaypointButtonClick()
{
    if (waypoint_combo_->currentIndex() < 0) {
        postStatusMessage(tr("Select a waypoint first"));
        return;
    }
    sendPatrolCommand(
        sahabat_interfaces::srv::PatrolCommand::Request::NAVIGATE,
        waypoint_combo_->currentData().toString());
}

void WaypointEditorPanel::onStopNavigationButtonClick()
{
    sendPatrolCommand(sahabat_interfaces::srv::PatrolCommand::Request::STOP, QString());
}

void WaypointEditorPanel::onNewSetButtonClick()
{
    bool ok = false;
    const QString name = QInputDialog::getText(
        this,
        tr("Save As Waypoint Set"),
        tr("Waypoint set name:"),
        QLineEdit::Normal,
        tr("New Tour"),
        &ok);
    if (ok && !name.trimmed().isEmpty()) {
        manageWaypointSet(sahabat_interfaces::srv::ManageWaypointSet::Request::CREATE, QString(), name.trimmed());
    }
}

void WaypointEditorPanel::onRenameSetButtonClick()
{
    const int index = waypoint_set_combo_->currentIndex();
    if (index < 0) {
        return;
    }
    bool ok = false;
    QString current = waypoint_set_combo_->currentText();
    const QString name = QInputDialog::getText(
        this,
        tr("Rename Waypoint Set"),
        tr("Waypoint set name:"),
        QLineEdit::Normal,
        current,
        &ok);
    if (ok && !name.trimmed().isEmpty()) {
        manageWaypointSet(
            sahabat_interfaces::srv::ManageWaypointSet::Request::RENAME,
            waypoint_set_combo_->itemData(index).toString(),
            name.trimmed());
    }
}

void WaypointEditorPanel::onDeleteSetButtonClick()
{
    const int index = waypoint_set_combo_->currentIndex();
    if (index < 0) {
        return;
    }
    const auto answer = QMessageBox::question(
        this,
        tr("Delete Waypoint Set"),
        tr("Archive waypoint set '%1'? This keeps the YAML under waypoint_sets/archive.").arg(waypoint_set_combo_->currentText()));
    if (answer == QMessageBox::Yes) {
        manageWaypointSet(
            sahabat_interfaces::srv::ManageWaypointSet::Request::DELETE,
            waypoint_set_combo_->itemData(index).toString(),
            QString());
    }
}

void WaypointEditorPanel::onMarkerSizeChanged(double value)
{
    if (!nh_) {
        return;
    }
    auto results = nh_->set_parameters({rclcpp::Parameter("waypoint_marker_size", value)});
    if (!results.empty() && !results.front().successful) {
        postStatusMessage(QString::fromStdString(results.front().reason));
    }
}

void WaypointEditorPanel::onAutoToggle(bool checked)
{
    if (!auto_start_client_ || !auto_stop_client_) {
        return;
    }

    // Push latest threshold before (re)starting.
    std_msgs::msg::Float64 msg;
    msg.data = auto_distance_spin_->value();
    auto_distance_pub_->publish(msg);

    // Push topic/type parameters.
    std::string topic = auto_topic_edit_->text().toStdString();
    std::string type = auto_type_combo_->currentText().toStdString();
    std::vector<rclcpp::Parameter> params;
    params.emplace_back("auto_pose_topic", topic);
    params.emplace_back("auto_pose_type", type);
    auto results = nh_->set_parameters(params);
    for (const auto &res : results) {
        if (!res.successful) {
        postStatusMessage(QString::fromStdString(res.reason));
        QSignalBlocker blocker(auto_toggle_button_);
        auto_toggle_button_->setChecked(false);
        auto_toggle_button_->setText(tr("Start Auto"));
        setAutoControlsEnabled(true);
        return;
        }
    }

    auto client = checked ? auto_start_client_ : auto_stop_client_;
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
        postStatusMessage(tr("Auto service unavailable"));
        QSignalBlocker blocker(auto_toggle_button_);
        auto_toggle_button_->setChecked(false);
        auto_toggle_button_->setText(tr("Start Auto"));
        setAutoControlsEnabled(true);
        return;
    }

    auto_toggle_button_->setText(checked ? tr("Stop Auto Capture") : tr("Start Auto Capture"));
    if (checked) {
        setAutoControlsEnabled(false);
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(req,
        [this, checked](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            std::string message = checked ? "Auto capture start failed" : "Auto capture stop failed";
            try {
                auto response = future.get();
                ok = response->success;
                message = response->message;
            } catch (...) {
                ok = false;
            }

            postStatusMessage(QString::fromStdString(message));
            if (!ok) {
                QMetaObject::invokeMethod(auto_toggle_button_, [this]() {
                    QSignalBlocker blocker(auto_toggle_button_);
                    auto_toggle_button_->setChecked(false);
                    auto_toggle_button_->setText(tr("Start Auto Capture"));
                }, Qt::QueuedConnection);
                QMetaObject::invokeMethod(this, [this]() {
                    setAutoControlsEnabled(true);
                }, Qt::QueuedConnection);
            } else {
                QMetaObject::invokeMethod(this, [this, checked]() {
                    setAutoControlsEnabled(!checked);
                }, Qt::QueuedConnection);
            }
        }
    );
}

void WaypointEditorPanel::setAutoControlsEnabled(bool enabled)
{
    auto_distance_spin_->setEnabled(enabled);
    auto_topic_edit_->setEnabled(enabled);
    auto_type_combo_->setEnabled(enabled);
}

void WaypointEditorPanel::postStatusMessage(const QString &msg)
{
    QMetaObject::invokeMethod(
        status_value_label_,
        "setText",
        Qt::QueuedConnection,
        Q_ARG(QString, msg));
}

void WaypointEditorPanel::onLoad2DMapButtonClick()
{
    QString qpath = QFileDialog::getOpenFileName(
        this,
        tr("Open 2D Map YAML"),
        map_dialog_directory_,
        tr("YAML Files (*.yaml)"));

    if (qpath.isEmpty()) {
        return;
    }

    const QFileInfo map_file(qpath);
    const QString map_id = map_file.fileName() == QStringLiteral("map.yaml")
        ? map_file.dir().dirName()
        : map_file.completeBaseName();

    const bool operator_available =
        control_lease_client_ && load_map_client_ &&
        control_lease_client_->wait_for_service(std::chrono::milliseconds(500)) &&
        load_map_client_->wait_for_service(std::chrono::milliseconds(500));

    if (!operator_available) {
        if (!nav2_load_map_client_ ||
            !nav2_load_map_client_->wait_for_service(std::chrono::seconds(2))) {
            postStatusMessage(tr("Map load service unavailable"));
            return;
        }
        auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        req->map_url = qpath.toStdString();
        try {
            nav2_load_map_client_->async_send_request(req,
                [this, map_id](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
                    QString message = tr("Failed to load 2D Map");
                    bool ok = false;
                    try {
                        auto response = future.get();
                        ok = response && response->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
                    } catch (const std::exception &e) {
                        message = QString("Failed to load 2D Map: %1").arg(e.what());
                    } catch (...) {
                        message = tr("Failed to load 2D Map");
                    }
                    if (ok) {
                        map_id_ = map_id;
                        const auto results = nh_->set_parameters({
                            rclcpp::Parameter("map_id", map_id.toStdString())});
                        if (!results.empty() && !results.front().successful) {
                            ok = false;
                            message = QString::fromStdString(results.front().reason);
                        } else {
                            refreshWaypointSets();
                            refreshWaypoints();
                        }
                    }
                    postStatusMessage(ok ? tr("Loaded 2D Map") : message);
                });
        } catch (const std::exception &e) {
            postStatusMessage(QString("Failed to request 2D map load: %1").arg(e.what()));
        } catch (...) {
            postStatusMessage(tr("Failed to request 2D map load"));
        }
        return;
    }

    auto lease_req = std::make_shared<sahabat_interfaces::srv::ControlLease::Request>();
    lease_req->action = sahabat_interfaces::srv::ControlLease::Request::ACQUIRE;
    lease_req->client_id = "rviz_waypoint_editor_panel";
    control_lease_client_->async_send_request(lease_req,
        [this, map_id](rclcpp::Client<sahabat_interfaces::srv::ControlLease>::SharedFuture lease_future) {
            sahabat_interfaces::srv::ControlLease::Response::SharedPtr lease;
            try {
                lease = lease_future.get();
            } catch (const std::exception &e) {
                postStatusMessage(QString("Failed to acquire map-load control: %1").arg(e.what()));
                return;
            } catch (...) {
                postStatusMessage(tr("Failed to acquire map-load control"));
                return;
            }
            if (!lease) {
                postStatusMessage(tr("Failed to acquire map-load control"));
                return;
            }
            if (!lease->success) {
                postStatusMessage(QString::fromStdString(lease->message));
                return;
            }

            auto req = std::make_shared<sahabat_interfaces::srv::LoadMap::Request>();
            req->map_id = map_id.toStdString();
            req->lease_id = lease->lease_id;
            try {
                load_map_client_->async_send_request(req,
                    [this, map_id, lease_id = lease->lease_id](rclcpp::Client<sahabat_interfaces::srv::LoadMap>::SharedFuture future) {
                        bool ok = false;
                        QString message = tr("Failed to load 2D Map");
                        try {
                            auto response = future.get();
                            if (response) {
                                ok = response->success;
                                message = QString::fromStdString(response->message);
                            } else {
                                message = tr("Failed to load 2D Map: empty service response");
                            }
                        } catch (const std::exception &e) {
                            message = QString("Failed to load 2D Map: %1").arg(e.what());
                        } catch (...) {
                            message = tr("Failed to load 2D Map");
                        }
                        if (ok) {
                            map_id_ = map_id;
                            const auto results = nh_->set_parameters({
                                rclcpp::Parameter("map_id", map_id.toStdString())});
                            if (!results.empty() && !results.front().successful) {
                                ok = false;
                                message = QString::fromStdString(results.front().reason);
                            } else {
                                refreshWaypointSets();
                                refreshWaypoints();
                            }
                        }
                        postStatusMessage(ok ? tr("Loaded 2D Map") : message);
                        releaseControlLease(lease_id);
                    });
            } catch (const std::exception &e) {
                postStatusMessage(QString("Failed to request 2D map load: %1").arg(e.what()));
                releaseControlLease(lease->lease_id);
            } catch (...) {
                postStatusMessage(tr("Failed to request 2D map load"));
                releaseControlLease(lease->lease_id);
            }
        });
}

void WaypointEditorPanel::onLoad3DMapButtonClick()
{
    QString qpath = QFileDialog::getOpenFileName(
        this,
        tr("Open 3D Map PCD"),
        map_dialog_directory_,
        tr("PCD Files (*.pcd)"));

    if (qpath.isEmpty()) {
        return;
    }

    const auto path = qpath.toStdString();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int ret = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    if (ret != 0 || cloud->empty()) {
        postStatusMessage(tr("Failed to load 3D Map"));
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = "map";
    cloud_pub_->publish(msg);
    postStatusMessage(tr("Loaded 3D Map"));
}

void WaypointEditorPanel::onLoadWaypointsButtonClick()
{
    if (!load_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    load_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            std::string message = "Failed to load WPs";
            try {
                auto response = future.get();
                ok = response->success;
                message = response->message;
            } catch (...) {
                ok = false;
            }
            if (ok) {
                refreshWaypointSets();
            }
            postStatusMessage(ok ? QString::fromStdString(message) : QString::fromStdString(message));
        });
}

void WaypointEditorPanel::onSaveWaypointsButtonClick()
{
    if (!save_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    save_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            std::string message = "Failed to save WPs";
            try {
                auto response = future.get();
                ok = response->success;
                message = response->message;
            } catch (...) {
                ok = false;
            }
            if (ok) {
                refreshWaypoints();
            }
            postStatusMessage(ok ? QString::fromStdString(message) : QString::fromStdString(message));
        });
}

void WaypointEditorPanel::onUndoWaypointsButtonClick()
{
    if (!undo_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    undo_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Reverted change") : tr("Nothing to undo"));
        });
}

void WaypointEditorPanel::onRedoWaypointsButtonClick()
{
    if (!redo_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    redo_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Reapplied change") : tr("Nothing to redo"));
        });
}

void WaypointEditorPanel::onClearWaypointsButtonClick()
{
    if (!clear_client_->wait_for_service(std::chrono::seconds(1))) {
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    clear_client_->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool ok = false;
            try {
                auto response = future.get();
                ok = response->success;
            } catch (...) {
                ok = false;
            }
            postStatusMessage(ok ? tr("Cleared all waypoints") : tr("Failed to clear waypoints"));
        });
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorPanel, rviz_common::Panel)

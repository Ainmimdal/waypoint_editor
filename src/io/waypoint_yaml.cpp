#include "waypoint_editor/io/waypoint_yaml.hpp"

#include <fstream>
#include <cmath>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace waypoint_editor::io
{

namespace
{

std::string EscapeYamlString(const std::string &input)
{
    std::ostringstream oss;
    for (const auto ch : input) {
        if (ch == '"') {
            oss << "\\\"";
        } else {
            oss << ch;
        }
    }
    return oss.str();
}

double YawFromPose(const geometry_msgs::msg::Pose &pose)
{
    const auto &q = pose.orientation;
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

}  // namespace

bool WaypointYaml::Save(const std::vector<Waypoint> &waypoints, const std::string &path, std::string &error)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "Failed to open file for writing: " + path;
        return false;
    }

    ofs << "revision: 0\n";
    ofs << "waypoints:\n";
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];
        const auto name = wp.function_command.empty()
            ? "waypoint_" + std::to_string(i + 1)
            : wp.function_command;
        ofs << "- id: " << i << "\n";
        ofs << "  name: \"" << EscapeYamlString(name) << "\"\n";
        ofs << "  x: " << wp.pose.pose.position.x << "\n";
        ofs << "  y: " << wp.pose.pose.position.y << "\n";
        ofs << "  yaw: " << YawFromPose(wp.pose.pose) << "\n";
        ofs << "  dwell_seconds: 0.0\n";
        ofs << "  enabled: true\n";
    }

    error.clear();
    return true;
}

namespace
{

bool ReadPoseComponent(const YAML::Node &parent, const char *key, double &value)
{
    const auto node = parent[key];
    if (!node) {
        return false;
    }
    try {
        value = node.as<double>();
        return true;
    } catch (const YAML::Exception &) {
        return false;
    }
}

bool PopulatePose(const YAML::Node &pose_node, geometry_msgs::msg::Pose &pose)
{
    const auto position = pose_node["position"];
    const auto orientation = pose_node["orientation"];
    if (!position || !orientation) {
        return false;
    }

    return ReadPoseComponent(position, "x", pose.position.x) &&
           ReadPoseComponent(position, "y", pose.position.y) &&
           ReadPoseComponent(position, "z", pose.position.z) &&
           ReadPoseComponent(orientation, "x", pose.orientation.x) &&
           ReadPoseComponent(orientation, "y", pose.orientation.y) &&
           ReadPoseComponent(orientation, "z", pose.orientation.z) &&
           ReadPoseComponent(orientation, "w", pose.orientation.w);
}

bool PopulateSahabatPose(const YAML::Node &entry, Waypoint &waypoint)
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!ReadPoseComponent(entry, "x", x) || !ReadPoseComponent(entry, "y", y)) {
        return false;
    }
    if (auto yaw_node = entry["yaw"]; yaw_node) {
        try {
            yaw = yaw_node.as<double>();
        } catch (const YAML::Exception &) {
            yaw = 0.0;
        }
    }
    waypoint.pose.header.frame_id = "map";
    waypoint.pose.pose.position.x = x;
    waypoint.pose.pose.position.y = y;
    waypoint.pose.pose.position.z = 0.0;
    waypoint.pose.pose.orientation = QuaternionFromYaw(yaw);
    if (auto name_node = entry["name"]; name_node && name_node.IsScalar()) {
        waypoint.function_command = name_node.as<std::string>();
    } else {
        waypoint.function_command.clear();
    }
    return true;
}

}  // namespace

bool WaypointYaml::Load(const std::string &path, std::vector<Waypoint> &waypoints, std::string &error)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception &ex) {
        error = "Failed to parse yaml: " + std::string(ex.what());
        return false;
    }

    const auto node = root["waypoints"];
    if (!node || !node.IsSequence()) {
        error = "Missing 'waypoints' list in yaml";
        return false;
    }

    std::vector<Waypoint> parsed;
    for (const auto &entry : node) {
        if (!entry.IsMap()) {
            continue;
        }

        Waypoint waypoint;
        const auto pose_node = entry["pose"];
        if (pose_node && pose_node.IsMap()) {
            if (!PopulatePose(pose_node, waypoint.pose.pose)) {
                continue;
            }
            if (auto frame_node = entry["frame_id"]; frame_node && frame_node.IsScalar()) {
                waypoint.pose.header.frame_id = frame_node.as<std::string>();
            } else {
                waypoint.pose.header.frame_id = "map";
            }
            if (auto command_node = entry["command"]; command_node && command_node.IsScalar()) {
                waypoint.function_command = command_node.as<std::string>();
            } else if (auto name_node = entry["name"]; name_node && name_node.IsScalar()) {
                waypoint.function_command = name_node.as<std::string>();
            } else {
                waypoint.function_command.clear();
            }
        } else {
            if (!PopulateSahabatPose(entry, waypoint)) {
                continue;
            }
        }

        parsed.emplace_back(std::move(waypoint));
    }

    waypoints = std::move(parsed);
    error.clear();
    return true;
}

}  // namespace waypoint_editor::io

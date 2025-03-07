#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_visualization/srv/save_trajectory.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisherSaver : public rclcpp::Node
{
public:
  TrajectoryPublisherSaver()
  : Node("trajectory_publisher_saver")
  {
    // Subscribe to odometry to record the robot's pose
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrajectoryPublisherSaver::odom_callback, this, std::placeholders::_1));

    // Publisher for visualization markers (for RViz)
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

    // Service to save the trajectory data to a file
    save_service_ = this->create_service<trajectory_visualization::srv::SaveTrajectory>(
      "save_trajectory",
      std::bind(&TrajectoryPublisherSaver::save_trajectory_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Timer to periodically publish the MarkerArray (every 500 ms)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TrajectoryPublisherSaver::publish_markers, this));
  }

private:
  // A simple struct to store a pose and its timestamp.
  struct PoseRecord
  {
    rclcpp::Time timestamp;
    geometry_msgs::msg::Pose pose;
  };

  // Callback to record each odometry message.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    PoseRecord record;
    record.timestamp = this->now();
    record.pose = msg->pose.pose;
    trajectory_.push_back(record);
  }

  // Timer callback to publish the trajectory as a MarkerArray.
  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto & record : trajectory_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "trajectory";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = record.pose.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
    }
    marker_publisher_->publish(marker_array);
  }

  // Service callback: saves only the trajectory data for the specified duration.
  void save_trajectory_callback(
    const std::shared_ptr<trajectory_visualization::srv::SaveTrajectory::Request> request,
    std::shared_ptr<trajectory_visualization::srv::SaveTrajectory::Response> response)
  {
    // Determine the time threshold (current time minus duration).
    rclcpp::Time now = this->now();
    int32_t secs = static_cast<int32_t>(request->duration);
    uint32_t nsecs = static_cast<uint32_t>((request->duration - secs) * 1e9);
    rclcpp::Duration duration_sec(secs, nsecs);

    rclcpp::Time threshold = now - duration_sec;

    // Filter the recorded trajectory.
    std::vector<PoseRecord> filtered;
    for (const auto & record : trajectory_) {
      if (record.timestamp >= threshold) {
        filtered.push_back(record);
      }
    }

    // Save the filtered trajectory based on file extension.
    std::string filename = request->filename;
    bool success = false;
    std::string message;
    if (filename.size() >= 4) {
      std::string ext = filename.substr(filename.size() - 4);
      if (ext == ".csv") {
        success = saveCSV(filename, filtered, message);
      } else {
        message = "Unsupported file extension. Use .csv";
      }
    } else {
      message = "Filename too short.";
    }
    response->success = success;
    response->message = message;
  }

  // Save trajectory data as CSV.
  bool saveCSV(const std::string & filename, const std::vector<PoseRecord> & data, std::string & msg)
  {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
      msg = "Failed to open file for writing.";
      return false;
    }
    // Write CSV header.
    ofs << "time,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w\n";
    for (const auto & record : data) {
      ofs << record.timestamp.seconds() << ","
          << record.pose.position.x << ","
          << record.pose.position.y << ","
          << record.pose.position.z << ","
          << record.pose.orientation.x << ","
          << record.pose.orientation.y << ","
          << record.pose.orientation.z << ","
          << record.pose.orientation.w << "\n";
    }
    ofs.close();
    msg = "CSV file saved successfully.";
    return true;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Service<trajectory_visualization::srv::SaveTrajectory>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<PoseRecord> trajectory_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisherSaver>());
  rclcpp::shutdown();
  return 0;
}

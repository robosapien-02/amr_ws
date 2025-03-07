#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

class TrajectoryReaderPublisher : public rclcpp::Node
{
public:
  TrajectoryReaderPublisher()
  : Node("trajectory_reader_publisher")
  {
    // Declare and get the parameter for the file name.
    this->declare_parameter<std::string>("trajectory_file", "");
    trajectory_file_ = this->get_parameter("trajectory_file").as_string();
    if (trajectory_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'trajectory_file' not set. Exiting.");
      rclcpp::shutdown();
      return;
    }

    // Parse the CSV file.
    if (!parseCSV(trajectory_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse trajectory file.");
      rclcpp::shutdown();
      return;
    }

    // Publisher for the MarkerArray.
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("read_trajectory_markers", 10);
    // Timer to periodically publish markers.
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TrajectoryReaderPublisher::publishMarkers, this));
  }

private:
  // Structure to store each parsed record.
  struct PoseRecord
  {
    double time;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
  };

  // Parse a CSV file formatted similarly to the one saved by the publisher node.
  bool parseCSV(const std::string & filename)
  {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return false;
    }

    std::string line;
    // Skip header line.
    if (!std::getline(ifs, line)) {
      return false;
    }

    while (std::getline(ifs, line)) {
      if (line.empty()) {
        continue;
      }
      std::istringstream ss(line);
      std::string token;
      PoseRecord rec;
      try {
        // time
        std::getline(ss, token, ',');
        rec.time = std::stod(token);
        // position.x
        std::getline(ss, token, ',');
        rec.position.x = std::stod(token);
        // position.y
        std::getline(ss, token, ',');
        rec.position.y = std::stod(token);
        // position.z
        std::getline(ss, token, ',');
        rec.position.z = std::stod(token);
        // orientation.x
        std::getline(ss, token, ',');
        rec.orientation.x = std::stod(token);
        // orientation.y
        std::getline(ss, token, ',');
        rec.orientation.y = std::stod(token);
        // orientation.z
        std::getline(ss, token, ',');
        rec.orientation.z = std::stod(token);
        // orientation.w
        std::getline(ss, token, ',');
        rec.orientation.w = std::stod(token);
        trajectory_.push_back(rec);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing line: %s", line.c_str());
      }
    }
    return true;
  }

  // Publishes the trajectory as a MarkerArray in the "odom" frame.
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto & rec : trajectory_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "odom";  // Transforming to odom frame
      marker.header.stamp = this->now();
      marker.ns = "read_trajectory";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = rec.position;
      marker.pose.orientation = rec.orientation;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
    }
    marker_publisher_->publish(marker_array);
  }

  std::string trajectory_file_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<PoseRecord> trajectory_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryReaderPublisher>());
  rclcpp::shutdown();
  return 0;
}

#ifndef SEAPATH_DRIVER_H
#define SEAPATH_DRIVER_H

#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "seapath_socket.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <vortex_msgs/msg/km_binary.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct KMBinaryData
{
   char start_id[4];
   uint16_t dgm_length;
   uint16_t dgm_version;
   uint32_t utc_seconds;
   uint32_t utc_nanoseconds;
   uint32_t status;
   double latitude;
   double longitude;
   float ellipsoid_height;
   float roll;
   float pitch;
   float heading;
   float heave;
   float roll_rate;
   float pitch_rate;
   float yaw_rate;
   float north_velocity;
   float east_velocity;
   float down_velocity;
   float latitude_error;
   float longitude_error;
   float height_error;
   float roll_error;
   float pitch_error;
   float heading_error;
   float heave_error;
   float north_acceleration;
   float east_acceleration;
   float down_acceleration;
   uint32_t delayed_heave_utc_seconds;
   uint32_t delayed_heave_utc_nanoseconds;
   float delayed_heave;
};
namespace seapath
{

   class Driver : public rclcpp::Node
   {
   public:
      /**
       * @brief Construct a new Seapath Driver object
       *
       */
      Driver();
      ~Driver() = default;

      /**
       * @brief Sets up the socket for communication.
       *
       * This function initializes the socket for UDP communication with the specified IP address and port number.
       *
       * @param UDP_IP The IP address to bind the socket to.
       * @param UDP_PORT The port number to bind the socket to.
       */
      void SetupSocket(std::string UDP_IP, uint16_t UDP_PORT);

      /**
       * @brief Timer callback function that retrieves KMBinaryData, then publishes it.
       */
      void timer_callback();

      /**
       * @brief Publish method that calls various publishers.
       *
       * @param data The KMBinaryData object to be published.
       */
      void publish(KMBinaryData data);

      /**
       * @brief Process the new data received from the socket
       *
       * @param data The new data
       */
      void process_kmbinary_data(std::vector<uint8_t> data);

      /**
       * @brief Parses the new data received from the socket
       *
       * @param data Variable for storing the new data
       * @return KMBinaryData - the new data. Is only updated with new data IF the socket is connected. That is done by checking the socket.connected boolean.
       */
      KMBinaryData parse_kmbinary_data(std::vector<uint8_t> data);

      /**
       * @brief Get the diagnostic array object message.
       *
       * @param diagnostic_msg The diagnostic message used for creating the diagnostic array.
       * @return diagnostic_msgs::msg::DiagnosticArray - the diagnostic array. Used for visualizing diagnostic status in programs like foxglove studio.
       */
      diagnostic_msgs::msg::DiagnosticArray get_diagnostic_array(const KMBinaryData &data);

      /**
       * @brief Get the odometry message object
       *
       * @param data The KMBinaryData object.
       * @return nav_msgs::msg::Odometry - the message
       */
      nav_msgs::msg::Odometry get_odometry_message(const KMBinaryData &data);

      /**
       * @brief Get the origin message object
       *
       * @return geometry_msgs::msg::Point. Contains the location of the origin.
       */
      geometry_msgs::msg::Point get_origin_message();

      /**
       * @brief Get the navsatfix message object. Contains the position (longitude, latitude and height), and the covariance of the position. The dimensions of the covariance matrix to the navsatfix message and the position covariance message is not the same.
       * Therefore a for-loop is used to only get the first 4 elements from the position covariance message and copy them over to the navsatfix message. Very small covariance information is lost.
       *
       * @param data The KMBinary object.
       * @return sensor_msgs::msg::NavSatFix - the message
       */
      sensor_msgs::msg::NavSatFix get_navsatfix_message(const KMBinaryData &data);

      /**
       * @brief Used for creating a message containing all the KMBinary data.
       *
       * @param data The KMBinary data.
       * @return vortex_msgs::msg::KMBinary - the message
       */
      vortex_msgs::msg::KMBinary get_kmbinary_message(const KMBinaryData &data);

      /**
       * @brief Calculates displacement in north and east directions from a reference position in the WGS84 coordinate system.
       *
       * @param north Northern coordinate in the WGS84 coordinate system.
       * @param east Eastern coordinate in the WGS84 coordinate system.
       * @return A std::pair<double, double> object containing the displacement in north and east directions.
       */
      std::pair<double, double> displacement_wgs84(double north, double east);
      /**
       * @brief Converts a value from degrees, minutes, and seconds (DMS) format to decimal degrees (DD).
       *
       * @param dms Value in degrees, minutes, and seconds format.
       * @return The converted value in decimal degrees.
       */
      double convert_dms_to_dd(double dms);
      /**
       * @brief Resets the global origin to the specified coordinates and height.
       *
       * @param data The KMBinaryData object containing latitude, longitude, and ellipsoid height.
       */
      void reset_origin(const KMBinaryData &data);

      /**
       * @brief Generates a TransformStamped message from the given KMBinaryData.
       *
       * This function takes a KMBinaryData object and converts it into a TransformStamped message
       * from the geometry_msgs package. The generated message represents a transformation in 3D space.
       *
       * @param data The KMBinaryData object to convert.
       * @return The generated TransformStamped message.
       */
      geometry_msgs::msg::TransformStamped get_transform_message(const KMBinaryData &data);
      /**
       * @brief Timer callback function that retrieves KMBinaryData, then publishes it.
       */
      rclcpp::TimerBase::SharedPtr timer_;

      KMBinaryData data_;
      std::mutex mutex_;
      std::vector<uint8_t> shared_vector_;
      bool packet_ready_;
      bool socket_connected_;
      std::string UDP_IP_;
      uint16_t UDP_PORT_;

      double orientation_error_diagnostic_;
      double position_error_diagnostic_;

      double ORIGIN_N = -100;
      double ORIGIN_E = -100;
      double ORIGIN_H = -100;

   private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr origin_pub_;
      rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticArray_pub_;
      rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub_;
      rclcpp::Publisher<vortex_msgs::msg::KMBinary>::SharedPtr kmbinary_pub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   };

   std::ostream &operator<<(std::ostream &os, const KMBinaryData &data);
   void printKMBinaryData(const KMBinaryData &data);

} // namespace seapath

#endif // SEAPATH_DRIVER_H
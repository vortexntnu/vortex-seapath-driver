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
#include <std_srvs/srv/trigger.hpp>


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
#include <tf2/LinearMath/Transform.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


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
       * This function initializes the socket for UDP communication with the specified IP address and port number from ROS params.
       *
       */
      void setup_socket();



      /**
       * @brief Sets the initial origin from the world frame and initiates the timer callback once origin is set.
       *
       *
       */
      void initial_setup();


      /**
       * @brief Timer callback function that retrieves KMBinaryData, then publishes it.
       */
      void timer_callback(); 

      /**
       * @brief Publishes static transform from world NED to world SEU to use for foxglove visualization.
       *
       */
      void publish_static_tf(const rclcpp::Time& time) const;

      /**
       * @brief Get the diagnostic array object message.
       *
       * @return diagnostic_msgs::msg::DiagnosticArray - the diagnostic array. Used for visualizing diagnostic status in programs like foxglove studio.
       */
      diagnostic_msgs::msg::DiagnosticArray get_diagnostic_array(const KMBinaryData& data, const rclcpp::Time& time) const;

      /**
       * @brief Get the odometry message object
       *
       * @return nav_msgs::msg::Odometry - the message
       */
      nav_msgs::msg::Odometry get_odometry_message(const KMBinaryData& data, const rclcpp::Time& time) const;

      /**
       * @brief Get the navsatfix message object. Contains the position (longitude, latitude and height), and the covariance of the position. 
       *
       * @param data The KMBinary object.
       * @return sensor_msgs::msg::NavSatFix - the message
       */
      sensor_msgs::msg::NavSatFix get_navsatfix_message(const KMBinaryData& data, const rclcpp::Time& time) const;

      /**
       * @brief Used for creating a ROS message defined in vortex-msgs containing all the KMBinary data.
       *
       * @return vortex_msgs::msg::KMBinary - the message
       */
      vortex_msgs::msg::KMBinary get_kmbinary_message(const KMBinaryData& data) const;
   
      /**
       * @brief sets the global origin to the specified coordinates and height in data_.
       *
       */
      void set_origin(const KMBinaryData& data);


      /**
       * @brief Service callback for resetting the origin. Service call example:
       * 
       * ros2 service call /reset_origin std_srvs/srv/Trigger "{}"
       *
       * @param request The request object.
       * @param response The response object.
       */
      void reset_origin_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);


      /**
       * @brief Loops until valid data is received from the socket and then calls set_origin().
       * 
       */
      void reset_origin();

      /**
       * @brief Converts degrees to radians.
       *
       * @param degrees The angle in degrees.
       * @return double - the angle in radians.
       */
      constexpr double deg2rad(double degrees) const;

      /**
       * @brief estimates the flat earth coordinates from the latitude, longitude and altitude of data_ from reference point set by origin.
       * 
       * @param lat The latitude.
       * @param lon The longitude.
       * @param alt The altitude.
       * @return std::array<double, 3> - the flat earth coordinates.
       */
      std::array<double, 3> lla2flat(double lat, double lon, double alt) const;

      
      /**
       * @brief Publishes the dynamic transforms based on data_. 
       *
       */
      void publish_dyn_tf(const KMBinaryData& data, const rclcpp::Time& time) const;

    

   private:

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
      rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub_;
      rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
      rclcpp::Publisher<vortex_msgs::msg::KMBinary>::SharedPtr kmbinary_pub_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
      std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_origin_service_;
      rclcpp::TimerBase::SharedPtr timer_;
      Socket socket_;
      std::string UDP_IP_;
      uint16_t UDP_PORT_;


      bool origin_set_ = false;
      double origin_lat_ = -100;
      double origin_lon_ = -100;
      float origin_h_ = -100;
   };

   std::ostream &operator<<(std::ostream &os, const KMBinaryData &data);
   void printKMBinaryData(const KMBinaryData &data);

} // namespace seapath

#endif // SEAPATH_DRIVER_H
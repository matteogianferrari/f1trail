/**
 * @file    longitudinal_pid.hpp
 * 
 * @author  Matteo Baccilieri
 *
 * @date    2024-10-22
 * 
 * @brief   Header file for Longitudinal PID Controller.
 */

#ifndef LONGITUDINAL_PID_HPP
#define LONGITUDINAL_PID_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <memory>
#include <cmath>

/**
 * @class LongitudinalPIDNode
 * 
 * @brief Implements a PID controller for longitudinal control of a vehicle.
 *
 * @details This node subscribes to odometry and target position topics, calculates the 
 *          control effort using a PID controller to move the vehicle towards the target position, 
 *          and publishes throttle commands to control the vehicle's speed.
 */
class LongitudinalPIDNode : public rclcpp::Node
{
public:
    /**
     * @fn LongitudinalPIDNode
     *
     * @brief Constructs a new LongitudinalPIDNode object
     */
    LongitudinalPIDNode();

private:
    /**
     * @struct Position
     * 
     * @brief A simple struct to represent the x and y coordinates of the vehicle and target positions.
     */
    struct Position
    {
        float x; ///< X coordinate
        float y; ///< Y coordinate
    };

    /**
     * @fn void odom_callback
     * 
     * @brief Callback function for the odometry data.
     *
     * @details Whenever new odometry data is received, it updates the current position and speed of the vehicle.
     * 
     * @param[in] msg A shared pointer to the received Odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @fn void target_position_callback
     * 
     * @brief Callback function for the target position.
     *
     * @details When a new target position is received, it updates the target position that the vehicle 
     *          should move towards.
     * 
     * @param[in] msg_loc A shared pointer to the received Point message with target coordinates.
     */
    void target_position_callback(const geometry_msgs::msg::Point::SharedPtr msg_loc);

    /**
     * @fn void control_loop()
     * 
     * @brief Control loop for periodically updating throttle commands.
     *
     * @details It calculates the distance to the target, updates the PID controller, and publishes 
     *          a throttle command based on the control effort. 
     */
    void control_loop();

    /**
     * @fn void init_pid()
     * @brief Initializes the PID controller variables.
     *
     * @details Resets the integrator, previous error, and differentiator to zero to start fresh 
     *          with the PID control process.
     */
    void init_pid();

    /**
     * @fn float update_pid(float distance_to_target, float current_speed)
     * @brief Updates the PID controller output.
     *
     * @details Calculates the PID output (throttle) based on the distance to the target position and 
     *          current speed of the vehicle.
     *
     * @param[in] distance_to_target The current distance to the target position.
     * @param[in] current_speed The current speed of the vehicle.
     * @return The PID-calculated throttle value.
     */
    float update_pid(float distance_to_target, float current_speed);
    
    float Kp; ///< Proportional gain for the PID controller.
    float Ki; ///< Integral gain for the PID controller.
    float Kd; ///< Derivative gain for the PID controller.
    float tau; ///< Time constant for the PID controller filter.
    float sample_time; ///< Time interval for updating the PID controller.
    float integrator_min; ///< Minimum value for the integrator to avoid windup.
    float integrator_max; ///< Maximum value for the integrator to avoid windup.

    float integrator; ///< Integrator state for the PID controller.
    float prev_error; ///< The previous error value for calculating derivative.
    float differentiator; ///< Differentiator state for the PID controller.
    float prev_measurement; ///< Previous measurement for filtering.
    float pid_output; ///< Output from the PID controller.

    Position current_position; ///< Current position of the vehicle.
    Position target_position; ///< Target position the vehicle should reach.
    float current_speed; ///< Current speed of the vehicle.

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_; ///< Subscriber for odometry data.
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_position_sub; ///< Subscriber for target position data.
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr throttle_pub; ///< Publisher for throttle commands.

    rclcpp::TimerBase::SharedPtr control_timer; ///< Timer for periodically running the control loop.
};


#endif // LONGITUDINAL_PID_HPP_
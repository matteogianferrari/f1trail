#include "longitudinal_pid.hpp"

// Used for parameter binding in callbacks to be registered with subscribers.
using std::placeholders::_1;

LongitudinalPIDNode::LongitudinalPIDNode()
    : Node("longitudinal_ctr"),
      Kp(0.3f), Ki(0.02f), Kd(0.05f), // PID coefficients
      tau(0.02f), sample_time(0.1f),  // Time constants for PID control
      integrator(0.0f), integrator_min(-0.4f),
      integrator_max(5.0f), prev_error(0.0f),
      differentiator(0.0f), prev_measurement(0.0f),
      pid_output(0.0f), current_speed(0.0f), // Initialize current speed
      current_position{0.0f, 0.0f},          // Initialize Position struct for current position
      target_position{0.0f, 0.0f}            // Initialize Position struct for target position
{
    // Initialize PID controller
    init_pid();

    // Subscribe to odometry to track car's position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&LongitudinalPIDNode::odom_callback, this, std::placeholders::_1));

    // Subscribe to target position messages
    target_position_sub = this->create_subscription<geometry_msgs::msg::Point>(
        "/target_loc", 10, std::bind(&LongitudinalPIDNode::target_position_callback, this, std::placeholders::_1));

    // Publisher for throttle command
    throttle_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    // Timer for control loop (runs every 0.1 seconds, i.e., 10 Hz)
    control_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(sample_time * 1000)),
        std::bind(&LongitudinalPIDNode::control_loop, this));
}

// Callback for odometry data to update current position
void LongitudinalPIDNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_position.x = msg->pose.pose.position.x; // Update current x position
    current_position.y = msg->pose.pose.position.y; // Update current y position
    current_speed = msg->twist.twist.linear.x;      // Update current speed
}

// Callback for target position
void LongitudinalPIDNode::target_position_callback(const geometry_msgs::msg::Point::SharedPtr msg_loc)
{
    target_position.x = msg_loc->x; // Set target x position
    target_position.y = msg_loc->y; // Set target y position
}

// Control loop method that runs periodically to update the throttle command
void LongitudinalPIDNode::control_loop()
{
    // Calculate distance to the target position
    float distance = sqrt(pow((target_position.x - current_position.x), 2) +
                          pow((target_position.y - current_position.y), 2));

    // Update PID controller with the current distance to the target
    float throttle = update_pid(distance, current_speed);

    // Create and publish throttle command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = throttle; // Set throttle based on PID output
    throttle_pub->publish(drive_msg); // Publish the drive command
}

// Initialize PID variables
void LongitudinalPIDNode::init_pid()
{
    integrator = 0.0f;
    prev_error = 0.0f;
    differentiator = 0.0f;
    prev_measurement = 0.0f;
    pid_output = 0.0f;
}

// Update PID output based on distance to target and current speed
float LongitudinalPIDNode::update_pid(float distance_to_target, float current_speed)
{
    // Threshold for stopping
    const float stop_threshold = 2.0f; // Distance threshold to stop (2 meters)

    // If the car is near the target, set throttle to zero and reset integrator
    if (distance_to_target < stop_threshold)
    {
        RCLCPP_INFO(this->get_logger(), "Car is near the target, stopping %f", distance_to_target);
        pid_output = 0.0f; // Stop the car (throttle = 0)
        integrator = 0.0f; // Reset the integrator to avoid windup
        return pid_output; // Return the output immediately
    }

    // Calculate the error (distance to target)
    float error = distance_to_target;

    // Proportional term
    float proportional = Kp * error;

    // Integral term with anti-windup using the trapezoidal rule
    integrator += 0.5f * Ki * sample_time * (error + prev_error);

    // Clamp integrator to its min and max values
    if (integrator > integrator_max)
    {
        integrator = integrator_max;
    }
    else if (integrator < integrator_min)
    {
        integrator = integrator_min;
    }

    // Derivative term (based on change in error over time)
    differentiator = Kd * (error - prev_error) / sample_time;

    // Compute final PID output (throttle) and apply limits
    pid_output = proportional + integrator + differentiator - current_speed; // Subtracting current speed for smooth control

    // Store the current error for the next update
    prev_error = error;

    return pid_output; // Return the calculated throttle
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LongitudinalPIDNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
/*
 * Pose Reinitializer Node
 * Periodically republishes AMCL pose to /initialpose to correct odometry drift
 *
 * Author: 이성민 (roboticsmaster@naver.com)
 * Company: TerraNox
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PoseReinitializer : public rclcpp::Node
{
public:
    PoseReinitializer() : Node("pose_reinitializer")
    {
        // Declare parameters
        this->declare_parameter("reinit_period", 30.0);  // seconds
        this->declare_parameter("covariance_threshold", 0.1);  // only reinit if covariance is low

        reinit_period_ = this->get_parameter("reinit_period").as_double();
        covariance_threshold_ = this->get_parameter("covariance_threshold").as_double();

        // Subscribe to AMCL pose
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&PoseReinitializer::amclPoseCallback, this, std::placeholders::_1));

        // Publisher for initial pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Timer for periodic reinitialization
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(reinit_period_),
            std::bind(&PoseReinitializer::reinitCallback, this));

        RCLCPP_INFO(this->get_logger(), "Pose Reinitializer started");
        RCLCPP_INFO(this->get_logger(), "  Reinit period: %.1f seconds", reinit_period_);
        RCLCPP_INFO(this->get_logger(), "  Covariance threshold: %.3f", covariance_threshold_);
    }

private:
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_amcl_pose_ = *msg;
        has_pose_ = true;
    }

    void reinitCallback()
    {
        if (!has_pose_) {
            RCLCPP_WARN(this->get_logger(), "No AMCL pose received yet, skipping reinit");
            return;
        }

        // Check if covariance is low enough (good localization)
        double cov_x = last_amcl_pose_.pose.covariance[0];
        double cov_y = last_amcl_pose_.pose.covariance[7];
        double cov_yaw = last_amcl_pose_.pose.covariance[35];

        if (cov_x > covariance_threshold_ || cov_y > covariance_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                "Covariance too high (x: %.4f, y: %.4f), skipping reinit", cov_x, cov_y);
            return;
        }

        // Create initial pose message with current AMCL pose
        // Use AMCL's original timestamp to avoid TF extrapolation issues
        auto init_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        init_pose.header.stamp = last_amcl_pose_.header.stamp;
        init_pose.header.frame_id = "map";
        init_pose.pose = last_amcl_pose_.pose;

        // Set small initial covariance for reinit
        init_pose.pose.covariance[0] = 0.01;   // x
        init_pose.pose.covariance[7] = 0.01;   // y
        init_pose.pose.covariance[35] = 0.005; // yaw

        initial_pose_pub_->publish(init_pose);

        RCLCPP_INFO(this->get_logger(),
            "Reinitializing pose at (%.2f, %.2f) with cov (%.6f, %.6f, %.6f)",
            last_amcl_pose_.pose.pose.position.x,
            last_amcl_pose_.pose.pose.position.y,
            cov_x, cov_y, cov_yaw);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
    bool has_pose_ = false;
    double reinit_period_;
    double covariance_threshold_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseReinitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

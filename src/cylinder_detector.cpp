#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class CylinderDetector : public rclcpp::Node {
public:
    CylinderDetector() : Node("cylinder_detector") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scan_callback, this, _1));
        cylinder_pub_ = this->create_publisher<std_msgs::msg::Int32>("cylinder", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cylinder_pub_;

    struct Point2D {
        float x, y;
    };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<Point2D> points;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            points.push_back({r * std::cos(angle), r * std::sin(angle)});
        }

        auto clusters = euclidean_clustering(points, 0.05);

        int cylinder_count = 0;

        for (const auto& cluster : clusters) {
            if (cluster.size() < 6) continue;
            auto [cx, cy, r] = fit_circle(cluster);
            if (r > 0.02 && r < 0.15) {
                RCLCPP_INFO(this->get_logger(), "Cylinder at (%.2f, %.2f), r=%.2f, %d", cx, cy, r, cylinder_count);
                cylinder_count++;
            }
        }

        std_msgs::msg::Int32 count_msg;
        count_msg.data = cylinder_count;
        cylinder_pub_->publish(count_msg);
    }

    std::vector<std::vector<Point2D>> euclidean_clustering(
        const std::vector<Point2D>& points, float threshold) {
        std::vector<std::vector<Point2D>> clusters;
        std::vector<bool> visited(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;

            std::vector<Point2D> cluster;
            cluster.push_back(points[i]);
            visited[i] = true;

            for (size_t j = 0; j < points.size(); ++j) {
                if (!visited[j] && distance(points[i], points[j]) < threshold) {
                    cluster.push_back(points[j]);
                    visited[j] = true;
                }
            }

            clusters.push_back(cluster);
        }

        return clusters;
    }

    float distance(const Point2D& p1, const Point2D& p2) {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }

    std::tuple<float, float, float> fit_circle(const std::vector<Point2D>& cluster) {
        Eigen::MatrixXd A(cluster.size(), 3);
        Eigen::VectorXd b(cluster.size());

        for (size_t i = 0; i < cluster.size(); ++i) {
            float x = cluster[i].x;
            float y = cluster[i].y;
            A(i, 0) = 2 * x;
            A(i, 1) = 2 * y;
            A(i, 2) = 1;
            b(i) = x*x + y*y;
        }

        Eigen::VectorXd sol = A.colPivHouseholderQr().solve(b);
        float cx = sol(0);
        float cy = sol(1);
        float r = std::sqrt(sol(2) + cx*cx + cy*cy);
        return {cx, cy, r};
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

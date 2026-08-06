#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

// intensity 없음

class LiDARTransformer : public rclcpp::Node
{
public:
    LiDARTransformer()
    : Node("lidar_transformer_13_cpp")
    {
        // LiDAR 토픽 구독
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nonground/lidar_13", 10,
            std::bind(&LiDARTransformer::pointCloudCallback, this, _1));

        // 변환된 LiDAR 토픽 발행
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/TFnonground/lidar_13", 10);

        // 변환값 설정 (translation, rotation)
        // translation_ = Eigen::Vector3f(-2.0f, 0.0f, 0.0f);  // g70 calib
        // translation_ = Eigen::Vector3f(-1.8f, 0.0f, 0.0f);  // fusion1
        // translation_ = Eigen::Vector3f(-2.0f, 0.0f, 0.0f);  // fusion2
        // translation_ = Eigen::Vector3f(-2.0f, 0.3f, 0.0f);  // fusion22
        translation_ = Eigen::Vector3f(-2.0f, 0.0f, 0.0f);  // 260623 해주

        // quaternion → rotation matrix
        // tf2::Quaternion q(0.0, 0.0, -0.972329, 0.233617);
        // tf2::Quaternion q(0.0, 0.0, -0.960835, 0.277121); // g70 calib
        // tf2::Quaternion q(0.0, 0.0, -0.960835, 0.277121); // fusion1
        // tf2::Quaternion q(0.0, 0.0, -0.964884, 0.262678); // fusion2
        // tf2::Quaternion q(0.0, 0.0, -0.963558, 0.267499);  // fusion22
        tf2::Quaternion q(0.0, 0.0, -0.963558, 0.267499);  // 260623 해주

        tf2::Matrix3x3 rot_tf(q);
        rotation_ << rot_tf[0][0], rot_tf[0][1], rot_tf[0][2],
                     rot_tf[1][0], rot_tf[1][1], rot_tf[1][2],
                     rot_tf[2][0], rot_tf[2][1], rot_tf[2][2];

        RCLCPP_INFO(this->get_logger(), "Translate LiDAR_13 points initialized");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto transformed_msg = transformPointCloud(msg);
        if (transformed_msg)
            pub_->publish(*transformed_msg);
    }

    sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &input)
    {
        // 입력 포인트 접근용 iterator
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input, "z");

        std::vector<Eigen::Vector3f> points;
        points.reserve(input->width * input->height);

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // NaN 필터
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                // RCLCPP_ERROR(this->get_logger(), "Publishing NaN");
                continue;
            }

            Eigen::Vector3f p(x, y, z);
            Eigen::Vector3f p_rot = rotation_ * p + translation_;

            points.emplace_back(p_rot.x(), p_rot.y(), p_rot.z());
        }

        // 출력 메시지 생성
        auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
        output->header.stamp = this->get_clock()->now();
        output->header.frame_id = "vehicle";
        output->height = 1;
        output->width = points.size();

        output->fields.resize(3);
        output->fields[0].name = "x";
        output->fields[0].offset = 0;
        output->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[0].count = 1;

        output->fields[1].name = "y";
        output->fields[1].offset = 4;
        output->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[1].count = 1;

        output->fields[2].name = "z";
        output->fields[2].offset = 8;
        output->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[2].count = 1;

        output->is_bigendian = false;
        output->point_step = 12;
        output->row_step = output->point_step * points.size();
        output->is_dense = true;

        // 데이터를 byte로 변환
        output->data.resize(output->row_step);
        uint8_t *ptr = output->data.data();
        for (const auto &p : points) {
            memcpy(ptr, p.data(), 12);
            ptr += 12;
        }

        // ✅ NaN 카운트 디버깅
        // int nan_count = 0;
        // {
        //     sensor_msgs::PointCloud2ConstIterator<float> check_x(*output, "x");
        //     sensor_msgs::PointCloud2ConstIterator<float> check_y(*output, "y");
        //     sensor_msgs::PointCloud2ConstIterator<float> check_z(*output, "z");
        //     for (; check_x != check_x.end(); ++check_x, ++check_y, ++check_z) {
        //         if (!std::isfinite(*check_x) || !std::isfinite(*check_y) || !std::isfinite(*check_z))
        //             nan_count++;
        //     }
        // }
        // if (nan_count > 0)
        //     RCLCPP_ERROR(this->get_logger(), "Publishing %d NaN points!", nan_count);

        return output;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    Eigen::Matrix3f rotation_;
    Eigen::Vector3f translation_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

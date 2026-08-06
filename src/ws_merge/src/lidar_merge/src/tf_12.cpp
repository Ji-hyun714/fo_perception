#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class LiDARTransformer : public rclcpp::Node
{
public:
    LiDARTransformer()
    : Node("lidar_transformer_12_cpp")
    {
        // LiDAR 토픽 구독
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_12_201/points", 10,
            std::bind(&LiDARTransformer::pointCloudCallback, this, _1));

        // 변환된 LiDAR 토픽 발행
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_12_201/TFpoints_new", 10);

        // 변환값 설정 (translation, rotation)
        translation_ = Eigen::Vector3f(2.2f, -1.0f, 0.0f);

        // quaternion → rotation matrix
        tf2::Quaternion q(0.0, 0.0, -0.543835, 0.839192);
        tf2::Matrix3x3 rot_tf(q);
        rotation_ << rot_tf[0][0], rot_tf[0][1], rot_tf[0][2],
                     rot_tf[1][0], rot_tf[1][1], rot_tf[1][2],
                     rot_tf[2][0], rot_tf[2][1], rot_tf[2][2];

        RCLCPP_INFO(this->get_logger(), "Translate LiDAR_12 points initialized");
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
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*input, "intensity");

        std::vector<Eigen::Vector4f> points;
        points.reserve(input->width * input->height);

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // NaN 필터
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            Eigen::Vector3f p(x, y, z);
            Eigen::Vector3f p_rot = rotation_ * p + translation_;

            // transform 후 NaN 체크
            if (!std::isfinite(p_rot.x()) || !std::isfinite(p_rot.y()) || !std::isfinite(p_rot.z()))
                continue;

            float intensity = 0.0f;
            if (iter_i != iter_i.end())
                intensity = *iter_i;

            points.emplace_back(p_rot.x(), p_rot.y(), p_rot.z(), intensity);
        }

        // 출력 메시지 생성
        auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
        output->header.stamp = this->get_clock()->now();
        output->header.frame_id = "vehicle";
        output->height = 1;
        output->width = points.size();

        output->fields.resize(4);
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

        output->fields[3].name = "intensity";
        output->fields[3].offset = 12;
        output->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output->fields[3].count = 1;

        output->is_bigendian = false;
        output->point_step = 16;
        output->row_step = output->point_step * points.size();
        output->is_dense = true;

        // 데이터를 byte로 변환
        output->data.resize(output->row_step);
        uint8_t *ptr = output->data.data();
        for (const auto &p : points) {
            memcpy(ptr, p.data(), 16);
            ptr += 16;
        }

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

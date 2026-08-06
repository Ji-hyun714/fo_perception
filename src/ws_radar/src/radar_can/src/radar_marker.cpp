#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fo_msgs/msg/radar_tr_array.hpp>
#include <cmath>
#include <cstdio>

using std::placeholders::_1;

// /radar_tr (fo_msgs/RadarTrArray) 를 구독해서
// 각 트랙의 (track_pos_x, track_pos_y) 위치에 마커를 생성/발행하는 노드
class RadarMarker : public rclcpp::Node
{
public:
    RadarMarker() : Node("radar_marker_node")
    {
        // RViz Fixed Frame 과 맞춰야 마커가 보임 (기본값: radar)
        frame_id_ = this->declare_parameter<std::string>("frame_id", "vehicle");
        marker_size_ = this->declare_parameter<double>("marker_size", 1.0);
        vel_arrow_scale_ = this->declare_parameter<double>("vel_arrow_scale", 0.1);  // vel 화살표 길이 스케일

        sub_ = this->create_subscription<fo_msgs::msg::RadarTrArray>(
            "/radar_tr", 10, std::bind(&RadarMarker::callback, this, _1));

        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar/markers", 10);
        pub_status_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar/status", 10);
        pub_pos_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar/pos_xy", 10);
        pub_vel_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar/vel_xy", 10);

        RCLCPP_INFO(this->get_logger(),
            "RadarMarker Node started. (frame_id: %s)", frame_id_.c_str());
    }

private:
    void callback(const fo_msgs::msg::RadarTrArray::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;   // marker
        visualization_msgs::msg::MarkerArray status_array;   // status
        visualization_msgs::msg::MarkerArray pos_array;      // pos_x, pos_y
        visualization_msgs::msg::MarkerArray vel_array;      // vel_x, vel_y

        // 이전 프레임 마커 전부 삭제 (잔상 방지)
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = frame_id_;
        clear_marker.header.stamp = msg->header.stamp;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        status_array.markers.push_back(clear_marker);
        pos_array.markers.push_back(clear_marker);
        vel_array.markers.push_back(clear_marker);

        int id = 0;
        for (const auto & tr : msg->tracks) {
            // 유효하지 않은 트랙은 건너뜀
            // if (tr.track_valid == 0) continue;
            if (tr.track_pos_x < 0.5f && tr.track_pos_x > -0.5f) continue;  // 중앙 마커 건너뜀

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = msg->header.stamp;
            marker.ns = "radar_tr";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = tr.track_pos_x + 2.2f;
            marker.pose.position.y = tr.track_pos_y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = marker_size_;
            marker.scale.y = marker_size_;
            marker.scale.z = marker_size_;

            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            // marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            marker_array.markers.push_back(marker);

            // 트랙 속도(vel) 텍스트 마커
            visualization_msgs::msg::Marker text = marker;
            text.ns = "radar_tr_vel";
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.pose.position.z = marker_size_;
            text.scale.z = marker_size_;
            text.color.r = 1.0f;
            text.color.g = 1.0f;
            text.color.b = 1.0f;

            float speed = 3.6f * std::sqrt(tr.track_vel_x * tr.track_vel_x + tr.track_vel_y * tr.track_vel_y);  // [km/h]
            // float speed = std::sqrt(tr.track_vel_x * tr.track_vel_x + tr.track_vel_y * tr.track_vel_y);  // [km/h]
            if (tr.track_vel_x < 0) speed = -speed;
            char speed_buf[24];
            std::snprintf(speed_buf, sizeof(speed_buf), "vel: %.1f", speed);
            text.text = speed_buf;

            vel_array.markers.push_back(text);

            // 트랙 속도(vel) 화살표 마커 (방향 = 속도 벡터, 길이 = 속력 비례)
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = msg->header.stamp;
            arrow.ns = "radar_tr_vel_arrow";
            arrow.id = marker.id;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point start, end;
            start.x = tr.track_pos_x + 2.2f;
            start.y = tr.track_pos_y;
            start.z = 0.0;
            end.x = start.x + tr.track_vel_x * 3.6f * vel_arrow_scale_;   // km/h 기준
            end.y = start.y + tr.track_vel_y * 3.6f * vel_arrow_scale_;   // km/h 기준
            // end.x = start.x + tr.track_vel_x * vel_arrow_scale_;   // km/h 기준 (bag)
            // end.y = start.y + tr.track_vel_y * vel_arrow_scale_;   // km/h 기준 (bag)
            end.z = 0.0;
            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.2;   // shaft 지름
            arrow.scale.y = 0.4;   // head 지름
            arrow.scale.z = 0.4;   // head 길이

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 1.0f;
            arrow.color.a = 1.0f;

            vel_array.markers.push_back(arrow);

            // 트랙 ID 텍스트 마커
            visualization_msgs::msg::Marker id_text = text;
            id_text.ns = "radar_tr_id";
            id_text.pose.position.z = marker_size_ * 2.0;   // 속도 텍스트 위에 표시
            char id_buf[24];
            std::snprintf(id_buf, sizeof(id_buf), "id: %d", tr.track_id);
            id_text.text = id_buf;
            marker_array.markers.push_back(id_text);

            // 트랙 위치(pos_x, pos_y) 텍스트 마커
            visualization_msgs::msg::Marker pos_text = text;
            pos_text.ns = "radar_tr_pos";
            pos_text.pose.position.z = marker_size_ * 3.0;   // id 텍스트 위에 표시
            char pos_buf[40];
            std::snprintf(pos_buf, sizeof(pos_buf), "pos: (%.1f, %.1f)",
                          tr.track_pos_x + 2.2f, tr.track_pos_y);
            pos_text.text = pos_buf;
            pos_array.markers.push_back(pos_text);

            // 트랙 상태(track_status) 텍스트 마커
            visualization_msgs::msg::Marker status_text = text;
            status_text.ns = "radar_tr_status";
            status_text.pose.position.z = marker_size_ * 4.0;   // pos 텍스트 위에 표시
            status_text.text = "status: " + std::to_string(tr.track_status);
            status_array.markers.push_back(status_text);
        }

        pub_marker_->publish(marker_array);
        pub_status_->publish(status_array);
        pub_pos_->publish(pos_array);
        pub_vel_->publish(vel_array);
    }

    rclcpp::Subscription<fo_msgs::msg::RadarTrArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_status_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pos_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vel_;
    std::string frame_id_;
    double marker_size_;
    double vel_arrow_scale_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarMarker>());
    rclcpp::shutdown();
    return 0;
}

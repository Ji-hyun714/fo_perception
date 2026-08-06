#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "fo_msgs/msg/lidar2_data_for_sf2.hpp"
#include "fo_msgs/msg/lidarobj_lists_for_sf2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <queue>
#include <limits>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <pcl/filters/filter.h>

using namespace std;

// ===================== Track =====================
struct Track {
    int id;

    Eigen::Vector4f x;      // [rx, ry, vx, vy]
    Eigen::Matrix4f P;

    float cx, cy, cz;
    float lx, ly, lz;

    rclcpp::Time last_time;
    int missed;
    float score;
    int tracked;
    bool updated;
    float vel;
    float cost;
};

// ===================== Hungarian =====================
class Hungarian {
public:
    static std::vector<int> Solve(const std::vector<std::vector<float>>& cost_matrix) {
        size_t n = cost_matrix.size();
        if (n == 0) return {};
        size_t m = cost_matrix[0].size();
        size_t dim = std::max(n, m);

        std::vector<std::vector<float>> cost(dim, std::vector<float>(dim, 1e6));
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < m; j++)
                cost[i][j] = cost_matrix[i][j];

        std::vector<float> u(dim + 1), v(dim + 1);
        std::vector<int> p(dim + 1), way(dim + 1);

        for (size_t i = 1; i <= dim; i++) {
            p[0] = i;
            int j0 = 0;
            std::vector<float> minv(dim + 1, 1e9);
            std::vector<char> used(dim + 1, false);

            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                float delta = 1e9;

                for (size_t j = 1; j <= dim; j++) {
                    if (!used[j]) {
                        float cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                for (size_t j = 0; j <= dim; j++) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);

            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }

        std::vector<int> assignment(n, -1);
        for (size_t j = 1; j <= dim; j++) {
            if (p[j] <= (int)n && j <= m)
                assignment[p[j] - 1] = j - 1;
        }
        return assignment;
    }
};

// ===================== Node =====================
class LidarKalmanNode : public rclcpp::Node {
public:
    LidarKalmanNode() : Node("lidar_kalman_tracking"), next_id_(0) {

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/TFnonground/merged_points", 1,
            std::bind(&LidarKalmanNode::Callback, this, std::placeholders::_1));

        pub_bbox_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_markers/bbox", 10);
        pub_bbox_vehicle_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/bbox", 10);
        pub_id_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_markers/id", 10);
        pub_z_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_markers/maxz", 10);
        pub_lz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_markers/lz", 10);
        pub_point_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_markers/point", 10);
        pub_sf_ = this->create_publisher<fo_msgs::msg::LidarobjListsForSF2>("/lidar/sf_objs", 10);
        pub_cluster_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster_points", 10);

        RCLCPP_INFO(this->get_logger(), "Kalman Tracking Node Started");
    }

private:
    // ---------------- Detection ----------------
    struct Detection {
        Eigen::Vector4f x;
        float cx, cy, cz;
        float lx, ly, lz;
    };

    // ---------------- ID Allocation ----------------
    int AllocateTrackID() {
        unordered_set<int> used;
        for (auto& t : tracks_) used.insert(t.id);

        for (int i = 0; i < 256; i++) {
            int candidate = (next_id_ + i) % 256;
            if (!used.count(candidate)) {
                next_id_ = (candidate + 1) % 256;
                return candidate;
            }
        }
        return -1;
    }

    // ---------------- Preprocess ----------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocess(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
    {
        // rclcpp::Time sub = this->get_clock()->now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return cloud;

        // ---------------- 자차 주변 point 제거 ----------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        valid_cloud->points.reserve(cloud->points.size());
        for (const auto& pt : cloud->points) {
            if (pt.x > -2.0f && pt.x < 3.0f && pt.y > -2.0f && pt.y < 2.0f) continue;
            valid_cloud->points.push_back(pt);
        }
        valid_cloud->width = static_cast<uint32_t>(valid_cloud->points.size());
        valid_cloud->height = 1;
        valid_cloud->is_dense = false;
        // ---------------------------------------------------

        pcl::CropBox<pcl::PointXYZ> crop;
        // crop.setInputCloud(cloud);
        crop.setInputCloud(valid_cloud);
        crop.setMin(Eigen::Vector4f(-100, -100, -2, 1));
        crop.setMax(Eigen::Vector4f(100, 100, 4, 1));

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*roi);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(roi);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*ds);
        
        // rclcpp::Time preprocess = this->get_clock()->now();
        // double preprocess_ms = (preprocess - sub).seconds() * 1000.0;
        // std::cout << "Preprocess: " << preprocess_ms << std::endl;

        // return cloud;
        return ds;
    }

    // ---------------- Clustering ----------------
    vector<Detection> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // rclcpp::Time start = this->get_clock()->now();

        vector<Detection> detections;
        if (!cloud || cloud->empty()) return detections;

        // =========================================================
        // Clustering 계산용 scaled cloud 생성
        pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        scaled_cloud->points.reserve(cloud->points.size());

        for (const auto& pt : cloud->points) {
            pcl::PointXYZ q;

            // 수직 간격 보정
            float r = std::sqrt((pt.x*pt.x) + (pt.y*pt.y) + (pt.z*pt.z));
            float z_scale = (1/r) * 10.0f;
            z_scale = std::clamp(z_scale, 0.5f, 1.0f);

            q.x = pt.x;
            q.y = pt.y;
            q.z = pt.z * z_scale;

            scaled_cloud->points.push_back(q);
        }

        scaled_cloud->width = static_cast<uint32_t>(scaled_cloud->points.size());
        scaled_cloud->height = 1;
        scaled_cloud->is_dense = cloud->is_dense;
        // =========================================================

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // tree->setInputCloud(cloud);
        tree->setInputCloud(scaled_cloud);

        vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // ec.setClusterTolerance(1.7);
        ec.setClusterTolerance(1.1);
        ec.setMinClusterSize(7);
        ec.setMaxClusterSize(6000);
        ec.setSearchMethod(tree);
        // ec.setInputCloud(cloud);
        ec.setInputCloud(scaled_cloud);
        ec.extract(clusters);

        for (auto& indices : clusters) {
            if (indices.indices.empty()) continue;

            pcl::PointXYZ min_pt, max_pt;
            min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
            max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

            for (int idx : indices.indices) {
                const auto& pt = cloud->points[idx];

                min_pt.x = std::min(min_pt.x, pt.x);
                min_pt.y = std::min(min_pt.y, pt.y);
                min_pt.z = std::min(min_pt.z, pt.z);

                max_pt.x = std::max(max_pt.x, pt.x);
                max_pt.y = std::max(max_pt.y, pt.y);
                max_pt.z = std::max(max_pt.z, pt.z);
            }

            float cx = (min_pt.x + max_pt.x) / 2.0f;
            float cy = (min_pt.y + max_pt.y) / 2.0f;
            float cz = (min_pt.z + max_pt.z) / 2.0f;

            float lx = max_pt.x - min_pt.x;
            float ly = max_pt.y - min_pt.y;
            float lz = max_pt.z - min_pt.z;

            // ================= 필터 조건 =================
            // if (-10.0f < cx && cx < 10.0f && max_pt.z < -0.3f) continue;
            // if (lz <= 0.25f && min_pt.z > 0.7f) continue;
            // // if (min_pt.z > 1.5f) continue;
            // if (lz > 4.0f || lz < 0.1f) continue;
            // if (lx > 13.0f || ly > 13.0f) continue;

            if (lz <= 0.1f && min_pt.z > 0.5f) continue;  
            if (lx > 13.0f || ly > 13.0f) continue;
            if ((lx > 13.0f || ly > 13.0f) || (lx > 2.8f && ly > 2.8f)) continue;
            // ===========================================

            float rx = min_pt.x;
            float ry = min_pt.y + (ly * 0.5f);

            Detection d;
            d.x << rx, ry, 0.0f, 0.0f;
            d.cx = cx; d.cy = cy; d.cz = cz;
            d.lx = lx; d.ly = ly; d.lz = lz;

            detections.push_back(d);
        }

        // rclcpp::Time end = this->get_clock()->now();
        // double clustering_ms = (end - start).seconds() * 1000.0;
        // std::cout << "Clustering: " << clustering_ms << std::endl;

        return detections;
    }

    // ---------------- Prediction ----------------
    void Prediction(Track& tr, double dt)
    {
        Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;

        Eigen::Matrix4f Q = Eigen::Matrix4f::Identity() * 0.05f;

        tr.x = F * tr.x;
        tr.P = F * tr.P * F.transpose() + Q;
    }

    // ---------------- Kalman Update ----------------
    void KalmanUpdate(Track& tr, const Detection& det)
    {
        Eigen::Matrix<float, 2, 4> H;
        H << 1, 0, 0, 0,
             0, 1, 0, 0;

        Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.2f;
        Eigen::Vector2f z(det.x(0), det.x(1));

        Eigen::Vector2f y = z - H * tr.x;
        Eigen::Matrix2f S = H * tr.P * H.transpose() + R;
        Eigen::Matrix<float, 4, 2> K = tr.P * H.transpose() * S.inverse();

        float prev_x = tr.x(0);
        float prev_y = tr.x(1);

        tr.x = tr.x + K * y;
        tr.P = (Eigen::Matrix4f::Identity() - K * H) * tr.P;

        double dt = (this->get_clock()->now() - tr.last_time).seconds();
        if (dt <= 0) dt = 0.01;

        float vx = (tr.x(0) - prev_x) / dt;
        float vy = (tr.x(1) - prev_y) / dt;
        float vel_ = std::sqrt((vx * vx) + (vy * vy));

        tr.x(2) = 0.7f * tr.x(2) + 0.3f * vx;
        tr.x(3) = 0.7f * tr.x(3) + 0.3f * vy;

        tr.cx = det.cx; tr.cy = det.cy; tr.cz = det.cz;
        tr.lx = det.lx; tr.ly = det.ly; tr.lz = det.lz;

        tr.score = 0.0f;
        tr.vel = vel_;
    }

    // ---------------- Marker ----------------
    visualization_msgs::msg::Marker createTextMarker(
        const std_msgs::msg::Header& header,
        const std::string& ns,
        int id,
        float x, float y, float z, float lx, float offset,
        const std::string& text)
    {
        visualization_msgs::msg::Marker marker;

        marker.header = header;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x + (lx * offset);
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.text = text;

        return marker;
    }

    void IDColor(int id, uint8_t &r, uint8_t &g, uint8_t &b)
    {
        r = (id * 53) % 255;
        g = (id * 97) % 255;
        b = (id * 223) % 255;
    }

    // ---------------- Callback ----------------
    void Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto cloud = Preprocess(msg);
        auto detections = Clustering(cloud);

        rclcpp::Time now = this->get_clock()->now();
        const float MAX_ASSOC_DIST = 2.0f;
        const int MAX_MISSED = 10;

        for (auto& tr : tracks_) {
            double dt = (now - tr.last_time).seconds();
            if (dt <= 0) dt = 0.01;
            Prediction(tr, dt);
            tr.missed++;
            tr.updated = false;
        }

        size_t N = detections.size();
        size_t M = tracks_.size();
        vector<vector<float>> cost(N, vector<float>(M, 1e6));

        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < M; j++) {
                float d = (detections[i].x.head<2>() - tracks_[j].x.head<2>()).norm();
                if (d < MAX_ASSOC_DIST)
                    cost[i][j] = d;
            }
        }

        vector<int> assignment =
            (N > 0 && M > 0) ? Hungarian::Solve(cost)
                             : vector<int>(N, -1);

        for (auto& tr : tracks_)
            tr.cost = 1e6;

        for (size_t i = 0; i < N; i++) {
            int j = assignment[i];
            if (j != -1 && j < (int)M && cost[i][j] < MAX_ASSOC_DIST) {
                tracks_[j].cost = cost[i][j];
            }
        }

        vector<bool> matched(M, false);

        for (size_t i = 0; i < N; i++) {
            int track_id = -1;
            int j = assignment[i];

            if (i < assignment.size() && j != -1 && cost[i][j] < MAX_ASSOC_DIST) {
                track_id = tracks_[j].id;
                Track& tr = tracks_[j];
                KalmanUpdate(tr, detections[i]);
                tr.last_time = now;
                tr.missed = 0;
                tr.tracked++;
                tr.updated = true;
                matched[j] = true;
            }
            else {
                Track tr{};
                tr.id = AllocateTrackID();
                if (tr.id < 0) continue;

                track_id = tr.id;

                tr.x << detections[i].x(0), detections[i].x(1), 0.0f, 0.0f;
                tr.P = Eigen::Matrix4f::Identity() * 100.0f;

                tr.cx = detections[i].cx;
                tr.cy = detections[i].cy;
                tr.cz = detections[i].cz;
                tr.lx = detections[i].lx;
                tr.ly = detections[i].ly;
                tr.lz = detections[i].lz;

                tr.last_time = now;
                tr.missed = 0;
                tr.tracked = 1;
                tr.updated = true;
                tr.score = 0.0f;
                tr.vel = 0.0f;
                tr.cost = 1e6;

                tracks_.push_back(tr);
            }

            uint8_t r, g, b;
            IDColor(track_id, r, g, b);
        }

        tracks_.erase(
            remove_if(tracks_.begin(), tracks_.end(),
                [&](const Track& t){ return t.missed > MAX_MISSED; }),
            tracks_.end()
        );

        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::MarkerArray arr_vehicle;
        visualization_msgs::msg::MarkerArray id_markers;
        visualization_msgs::msg::MarkerArray z_markers;
        visualization_msgs::msg::MarkerArray lz_markers;
        visualization_msgs::msg::MarkerArray point_markers;
        fo_msgs::msg::LidarobjListsForSF2 sf_msgs;
        sf_msgs.header = msg->header;

        visualization_msgs::msg::Marker del;
        del.header = msg->header;
        del.action = visualization_msgs::msg::Marker::DELETEALL;

        arr.markers.push_back(del);
        arr_vehicle.markers.push_back(del);
        id_markers.markers.push_back(del);
        z_markers.markers.push_back(del);
        lz_markers.markers.push_back(del);
        point_markers.markers.push_back(del);

        for (auto& tr : tracks_) {
            if (tr.missed > 0 && tr.cost > 10) continue;
            // if (tr.missed > MAX_MISSED && tr.cost > 10) continue;

            visualization_msgs::msg::Marker mk;
            mk.header = msg->header;
            mk.header.frame_id = "vehicle";
            mk.ns = "tracks";
            mk.id = tr.id;
            mk.type = visualization_msgs::msg::Marker::CUBE;

            mk.pose.position.x = tr.cx;
            mk.pose.position.y = tr.cy;
            mk.pose.position.z = tr.cz;

            mk.scale.x = tr.lx;
            mk.scale.y = tr.ly;
            mk.scale.z = tr.lz;

            mk.color.g = (tr.missed == 0) ? 1.0f : 0.0f;
            mk.color.b = (tr.missed == 0) ? 0.0f : 1.0f;
            mk.color.a = 0.4f;
            arr.markers.push_back(mk);

            mk.header.frame_id = "vehicle";
            arr_vehicle.markers.push_back(mk);

            auto id_marker = createTextMarker(msg->header, "text_id", tr.id, tr.cx, tr.cy, tr.cz, tr.lx, 0.5, std::to_string(tr.id));
            id_markers.markers.push_back(id_marker);

            float calib_cx = tr.cx;
            float calib_cy = tr.cy;
            float pos_rx = calib_cx - (tr.lx * 0.5f);
            float pos_ry = calib_cy;

            std::ostringstream oss_z;
            oss_z << std::fixed << std::setprecision(2) << tr.cz + (tr.lz*0.5);
            auto z_marker = createTextMarker(msg->header, "text_pos_z", tr.id, tr.cx, tr.cy, tr.cz, tr.lx, 0, oss_z.str());
            z_markers.markers.push_back(z_marker);

            std::ostringstream oss_lz;
            oss_lz << std::fixed << std::setprecision(2) << tr.lz;
            auto lz_marker = createTextMarker(msg->header, "text_lz", tr.id, tr.cx, tr.cy, tr.cz, tr.lx, 0, oss_lz.str());
            lz_markers.markers.push_back(lz_marker);

            visualization_msgs::msg::Marker point_marker;
            point_marker.header = msg->header;
            point_marker.ns = "point_pos";
            point_marker.id = tr.id;
            point_marker.type = visualization_msgs::msg::Marker::SPHERE;
            point_marker.action = visualization_msgs::msg::Marker::ADD;
            point_marker.pose.position.x = pos_rx;
            point_marker.pose.position.y = pos_ry;
            point_marker.pose.position.z = tr.cz - (tr.lz * 0.5f);
            point_marker.pose.orientation.w = 1.0;
            point_marker.scale.x = 0.5;
            point_marker.scale.y = 0.5;
            point_marker.scale.z = 0.5;
            point_marker.color.a = 1.0;
            point_marker.color.r = 1.0;
            point_marker.color.g = 1.0;
            point_marker.color.b = 1.0;
            point_markers.markers.push_back(point_marker);

            fo_msgs::msg::Lidar2DataForSf2 sf_msg;
            sf_msg.lidar_track_id = tr.id;
            sf_msg.pos_x = pos_rx;
            sf_msg.pos_y = pos_ry;
            sf_msg.length = tr.lx;
            sf_msg.width = tr.ly;
            sf_msg.height = tr.lz;
            sf_msg.cz = tr.cz;

            sf_msgs.objs.push_back(sf_msg);

            // std::cout << std::fixed << std::setprecision(2)
            //           << " id: " << tr.id << " / "
            //           << "x: " << pos_rx << " / "
            //           << "y: " << pos_ry << " / "
            //           << "min_z: " << tr.cz - (tr.lz * 0.5) << " / "
            //           << "lz: " << tr.lz << " / "
            //         //   << "vel_x: " << tr.x(2) << " / "
            //         //   << "vel_y: " << tr.x(3) << " / "
            //         //   << "vel: " << tr.vel << " / "
            //           << "cost: " << tr.cost << std::endl;
        }
        // std::cout << "" << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*empty_cloud, cloud_msg);
        cloud_msg.header = msg->header;

        pub_bbox_->publish(arr);
        pub_bbox_vehicle_->publish(arr_vehicle);
        pub_id_->publish(id_markers);
        pub_z_->publish(z_markers);
        pub_lz_->publish(lz_markers);
        pub_point_->publish(point_markers);
        pub_sf_->publish(sf_msgs);
        pub_cluster_->publish(cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bbox_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bbox_vehicle_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_id_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_z_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lz_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_point_;
    rclcpp::Publisher<fo_msgs::msg::LidarobjListsForSF2>::SharedPtr pub_sf_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_;

    vector<Track> tracks_;
    int next_id_;
};

// ===================== main =====================
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarKalmanNode>());
    rclcpp::shutdown();
    return 0;
}
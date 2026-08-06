#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include "fo_msgs/msg/lidarobj_lists_for_sf2.hpp"

#include <vector>
#include <limits>
#include <array>

// ------------------- Track 구조체 -------------------
struct Track {
    int id;
    Eigen::Vector2f pos;   // 중심 (cx, cy)
    Eigen::Vector2f vel;   // 속도 (m/s)
    rclcpp::Time last_time;
    float lx, ly, lz;      // bbox 크기
};

// ------------------- Hungarian 알고리즘 -------------------
class Hungarian {
public:
    static std::vector<int> Solve(const std::vector<std::vector<float>>& cost_matrix) {
        size_t n = cost_matrix.size();
        size_t m = cost_matrix[0].size();
        size_t dim = std::max(n, m);

        // 정방행렬로 확장
        std::vector<std::vector<float>> cost(dim, std::vector<float>(dim, 1e6));
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < m; j++)
                cost[i][j] = cost_matrix[i][j];

        // Hungarian 구현 (Munkres)
        std::vector<float> u(dim+1), v(dim+1);
        std::vector<int> p(dim+1), way(dim+1);

        for (size_t i = 1; i <= dim; i++) {
            p[0] = i;
            int j0 = 0;
            std::vector<float> minv(dim+1, 1e9);
            std::vector<char> used(dim+1, false);
            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                float delta = 1e9;
                for (size_t j = 1; j <= dim; j++) {
                    if (!used[j]) {
                        float cur = cost[i0-1][j-1] - u[i0] - v[j];
                        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                    }
                }
                for (size_t j = 0; j <= dim; j++) {
                    if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                    else { minv[j] -= delta; }
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
                assignment[p[j]-1] = j-1;
        }
        return assignment;
    }
};

// ------------------- IoU 계산 -------------------
float computeIoU(const Eigen::Vector2f& c1, float lx1, float ly1,
                   const Eigen::Vector2f& c2, float lx2, float ly2)
{
    float x1_min = c1.x() - lx1/2, x1_max = c1.x() + lx1/2;
    float y1_min = c1.y() - ly1/2, y1_max = c1.y() + ly1/2;

    float x2_min = c2.x() - lx2/2, x2_max = c2.x() + lx2/2;
    float y2_min = c2.y() - ly2/2, y2_max = c2.y() + ly2/2;

    float inter_x = std::max(0.0f, std::min(x1_max, x2_max) - std::max(x1_min, x2_min));
    float inter_y = std::max(0.0f, std::min(y1_max, y2_max) - std::max(y1_min, y2_min));
    float inter_area = inter_x * inter_y;

    float area1 = lx1 * ly1;
    float area2 = lx2 * ly2;

    float union_area = area1 + area2 - inter_area;

    if (union_area <= 0) return 0.0f;
    return inter_area / union_area;
}

// ------------------- NMS 수행 -------------------
void applyNMS(std::vector<Eigen::Vector2f>& dets,
              std::vector<std::array<float,4>>& sizes,
              float iou_threshold = 0.4f)
{
    std::vector<int> keep;
    std::vector<bool> removed(dets.size(), false);

    for (size_t i = 0; i < dets.size(); i++) {
        if (removed[i]) continue;
        keep.push_back(i);

        for (size_t j = i + 1; j < dets.size(); j++) {
            if (removed[j]) continue;

            float iou = computeIoU(
                dets[i], sizes[i][0], sizes[i][1],
                dets[j], sizes[j][0], sizes[j][1]
            );

            if (iou > iou_threshold) {
                removed[j] = true;   // 삭제
            }
        }
    }

    // keep 목록 기준으로 다시 구성
    std::vector<Eigen::Vector2f> dets_new;
    std::vector<std::array<float,4>> sizes_new;

    for (int idx : keep) {
        dets_new.push_back(dets[idx]);
        sizes_new.push_back(sizes[idx]);
    }

    dets = dets_new;
    sizes = sizes_new;
}

// ------------------- LidarClusteringNode -------------------
class LidarClusteringNode : public rclcpp::Node {
public:
    LidarClusteringNode() : Node("lidar_clustering_node"), next_id_(0) {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/merge_201/points", 10,
            std::bind(&LidarClusteringNode::cloud_callback, this, std::placeholders::_1));

        marker_pub_        = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trackers_marker", 10);
        marker_points_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/marker_points", 10);
        lidar_sf_pub_      = this->create_publisher<fo_msgs::msg::LidarobjListsForSF2>("/lidar/sf_objs", 10);
        nonground_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/nonground", 10);

        init_colors();
        RCLCPP_INFO(this->get_logger(), "✅ Lidar Clustering + Hungarian Tracking Node started.");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // ROS → PCL 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 1. 다운샘플링
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*cloud_ds);

        // 2. ROI Crop
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud_ds);
        crop.setMin(Eigen::Vector4f(-100, -100, -1, 1));
        crop.setMax(Eigen::Vector4f(100, 100, 4, 1));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*cloud_roi);

        // 3. RANSAC 바닥 제거
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.33);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        seg.setInputCloud(cloud_roi);
        seg.segment(*inliers, *coeff);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_roi);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noground(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_noground);
        if (cloud_noground->empty()) return;
        
        // 비지면 포인트 퍼블리시 (디버깅용)
        // sensor_msgs::msg::PointCloud2 nonground_msg;
        // pcl::toROSMsg(*cloud_noground, nonground_msg);
        // nonground_msg.header = msg->header;
        // nonground_pub_->publish(nonground_msg);

        // 4. 클러스터링
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_noground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.4);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(6000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_noground);
        ec.extract(cluster_indices);

        // Detection candidates
        std::vector<Eigen::Vector2f> detections;
        std::vector<std::array<float,4>> sizes; // lx, ly, lz, cz

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices)
                cluster->points.push_back(cloud_noground->points[idx]);
            if (cluster->empty()) continue;

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            float cx = (min_pt.x + max_pt.x) / 2.0f;
            float cy = (min_pt.y + max_pt.y) / 2.0f;
            float cz = (min_pt.z + max_pt.z) / 2.0f;
            float lx = (max_pt.x - min_pt.x);
            float ly = (max_pt.y - min_pt.y);
            float lz = (max_pt.z - min_pt.z);
            float low_z = min_pt.z;

            if (low_z > 2.0f) continue;  // 지면제거된 장거리에 있는 차량이 있을 수도?
            if (lz < 0.5f | lz >= 3.5f) continue;
            if (lx >= 2.5f && ly >= 2.5f) continue;
            if (lx >= 12.0f | ly >= 12.0f) continue;
            // if (!((lx >= 0.5f && lx <= 5.0f) || (ly >= 0.5f && ly <= 5.0f))) continue;
            if (lx * ly > 13.5f) continue;

            detections.push_back(Eigen::Vector2f(cx, cy));
            sizes.push_back({lx, ly, lz, cz});
        }

        applyNMS(detections, sizes, 0.4f);   // NMS 적용
        if (detections.empty()) return;

        // 5. Hungarian 매칭
        size_t N = detections.size();
        size_t M = tracks_.size();
        std::vector<std::vector<float>> cost(N, std::vector<float>(M, 1e6));

        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < M; j++) {
                cost[i][j] = (detections[i] - tracks_[j].pos).norm();
            }
        }

        std::vector<int> assignment;
        if (!cost.empty() && !cost[0].empty())
            assignment = Hungarian::Solve(cost);
        else
            assignment.assign(N, -1);

        // 6. 트랙 업데이트
        rclcpp::Time now = this->get_clock()->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker delete_markers;
        delete_markers.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_markers);

        // vision_msgs::msg::Detection3DArray detections_msg;
        // detections_msg.header = msg->header;

        // int fl_car = 0;
        // int fl_pedestrian = 0;
        // int fr_car = 0;
        // int fr_pedestrian = 0;
        // int r_car = 0;
        // int r_pedestrian = 0;

        // Custom message
        fo_msgs::msg::LidarobjListsForSF2 obj_list_msg;
        obj_list_msg.header = msg->header;

        for (size_t i = 0; i < N; i++) {
            int tid = -1;
            Eigen::Vector2f pos = detections[i];
            float lx = sizes[i][0], ly = sizes[i][1], lz = sizes[i][2], cz = sizes[i][3];

            if (i < assignment.size() && assignment[i] != -1 && assignment[i] < (int)M) {
                Track& tr = tracks_[assignment[i]];
                double dt = (now - tr.last_time).seconds();
                if (dt > 0.01) tr.vel = (pos - tr.pos) / dt;
                tr.pos = pos;
                tr.lx = lx; tr.ly = ly; tr.lz = lz;
                tr.last_time = now;
                tid = tr.id;
            } else {
                if (tracks_.size() < 32) {
                    Track tr;
                    tr.id = next_id_++ % 32;
                    tr.pos = pos;
                    tr.vel = Eigen::Vector2f(0,0);
                    tr.lx = lx; tr.ly = ly; tr.lz = lz;
                    tr.last_time = now;
                    tracks_.push_back(tr);
                    tid = tr.id;
                }
            }
            if (tid == -1) continue;

            // classification
            int obj_type = 0;                         // unknown (yellow)
            // if (cond_ped) {obj_type = 4;}          // pedestrian (red)
            // else if (cond_bus) {obj_type = 1;}     // car (green)
            // else if (cond_car) {obj_type = 2;}     // bus, truck (blue)

            if (lx <0.95f && ly < 0.95f && lz < 2.0f) {
                obj_type = 4;
            }
            else {
                obj_type = 1;
            }

            // Marker
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "tracks";
            marker.id = tid;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.pose.position.x = pos.x();
            marker.pose.position.y = pos.y();
            marker.pose.position.z = cz;
            marker.scale.x = lx;
            marker.scale.y = ly;
            marker.scale.z = lz;
            marker.color.a = 0.8;
            if (obj_type == 1) {
                // car → green
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            }
            else if (obj_type == 2) {
                // bus/truck → blue
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
            }
            else if (obj_type == 4) {
                // pedestrian → red
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
            }
            else if (obj_type == 0) {
                // unknown → yellow
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            }
            marker_array.markers.push_back(marker);

            // Custom message
            fo_msgs::msg::Lidar2DataForSf2 obj;
            obj.lidar_track_id = tid;    // cluster ID
            obj.object_type = obj_type;  // class

            // 중심 좌표
            float cx = pos.x();
            float cy = pos.y();

            // 후방 중심
            obj.pos_y = cy;
            obj.pos_x = cx - (lx / 2.0f) - 2.45f;  // 라이다원점~카메라원점
            obj.velocity_x = tracks_[tid].vel.x();
            obj.velocity_y = tracks_[tid].vel.y();
            obj.confidence = 0.0f;

            obj.length = lx;
            obj.width  = ly;
            obj.height = lz;

            // push
            obj_list_msg.objs.push_back(obj);

            // // 디버깅용
            // if (obj_type == 1 && cx >= 0 && cy > 0) {
            //     fl_car += 1;
            // }
            // else if (obj_type == 4 && cx >= 0 && cy > 0) {
            //     fl_pedestrian += 1;
            // }
            // else if (obj_type == 1 && cx >= 0 && cy <= 0) {
            //     fr_car += 1;
            // }
            // else if (obj_type == 4 && cx >= 0 && cy <= 0) {
            //     fr_pedestrian += 1;
            // }
            // else if (obj_type == 1 && cx < 0) {
            //     r_car += 1;
            // }
            // else if (obj_type == 4 && cx < 0) {
            //     r_pedestrian += 1;
            // }
            // std::cout << "[FL] car: " << static_cast<int>(fl_car) << " / pedestrian: " << static_cast<int>(fl_pedestrian) << std::endl;
            // std::cout << "[FR] car: " << static_cast<int>(fr_car) << " / pedestrian: " << static_cast<int>(fr_pedestrian) << std::endl;
            // std::cout << "[R] car: " << static_cast<int>(r_car) << " / pedestrian: " << static_cast<int>(r_pedestrian) << std::endl;
            // std::cout << "" << std::endl;
        }

        //     // Detection3D
        //     vision_msgs::msg::Detection3D detection;
        //     detection.bbox.center.position.x = pos.x();
        //     detection.bbox.center.position.y = pos.y();
        //     detection.bbox.center.position.z = cz;
        //     detection.bbox.size.x = lx;
        //     detection.bbox.size.y = ly;
        //     detection.bbox.size.z = lz;

        //     vision_msgs::msg::ObjectHypothesisWithPose hyp;
        //     hyp.hypothesis.class_id = std::to_string(tid);  // 트래킹 id
        //     hyp.hypothesis.score = 1.0;
        //     detection.results.push_back(hyp);

        //     // 속도 추가 (pose.pose.position.x/y에 저장)
        //     detection.results[0].pose.pose.position.x = tracks_[tid].vel.x();
        //     detection.results[0].pose.pose.position.y = tracks_[tid].vel.y();

        //     detections_msg.detections.push_back(detection);
        // }

        marker_pub_->publish(marker_array);
        lidar_sf_pub_->publish(obj_list_msg);
        // marker_points_pub_->publish(detections_msg);
    }

    void init_colors() {
        colors_ = {
            {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1},
            {0.5,0.5,0.5},{1,0.5,0},{0.5,0,1},{0,0.5,1},
            {0.3,0.7,0.2},{0.8,0.3,0.2},{0.2,0.8,0.8},{0.7,0.2,0.7},
            {0.9,0.6,0.1},{0.4,0.9,0.1},{0.1,0.4,0.9},{0.9,0.1,0.4},
            {0.6,0.1,0.9},{0.1,0.9,0.6},{0.9,0.9,0.3},{0.3,0.9,0.9},
            {0.9,0.3,0.9},{0.5,0.2,0.7},{0.7,0.5,0.2},{0.2,0.7,0.5},
            {0.6,0.4,0.3},{0.3,0.6,0.4},{0.4,0.3,0.6},{0.2,0.2,0.2},
            {0.8,0.8,0.8},{0.1,0.1,0.1}
        };
    }

    // ROS 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr marker_points_pub_;
    rclcpp::Publisher<fo_msgs::msg::LidarobjListsForSF2>::SharedPtr lidar_sf_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;

    std::vector<Track> tracks_;
    int next_id_;
    std::vector<std::array<float,3>> colors_;
};

// ------------------- main -------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClusteringNode>());
    rclcpp::shutdown();
    return 0;
}

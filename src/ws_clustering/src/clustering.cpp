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

// Detection 메시지
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

class LidarClusteringNode : public rclcpp::Node {
public:
    LidarClusteringNode() : Node("lidar_clustering_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/merge_201/points", 10,
            std::bind(&LidarClusteringNode::cloud_callback, this, std::placeholders::_1));

        // ransac_pub_        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ransac_points", 10);
        marker_pub_        = this->create_publisher<visualization_msgs::msg::MarkerArray>("/clusters_marker", 10);
        marker_points_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/marker_points", 10);
        RCLCPP_INFO(this->get_logger(), "Lidar Clustering Node started.");
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
        crop.setMin(Eigen::Vector4f(0, -7, -1, 1));
        crop.setMax(Eigen::Vector4f(50, 7, 4, 1));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*cloud_roi);

        // 3. RANSAC 바닥 제거
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.3);

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

        // // 지면 제거된 점군 퍼블리시 (디버깅용)
        // sensor_msgs::msg::PointCloud2 noground_msg;
        // pcl::toROSMsg(*cloud_noground, noground_msg);
        // noground_msg.header = msg->header;
        // ransac_pub_->publish(noground_msg);

        // 4. 클러스터링
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_noground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_noground);
        ec.extract(cluster_indices);

        // 5. 결과 퍼블리시 (Marker + Detection)
        visualization_msgs::msg::MarkerArray marker_array;
        auto detections_msg = vision_msgs::msg::Detection3DArray();
        detections_msg.header = msg->header;

        int cluster_id = 0;
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

            // 필터링 조건
            if (low_z > 0.5f) continue;
            if (lz < 0.5f || lz >= 2.5f) continue;
            if (!((lx >= 0.5f && lx <= 5.0f) || (ly >= 0.5f && ly <= 5.0f))) continue;
            if (lx * ly > 13.5f) continue;

            // RViz marker
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;
            marker.scale.x = lx;
            marker.scale.y = ly;
            marker.scale.z = lz;
            marker.color.a = 0.6;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker_array.markers.push_back(marker);

            // 값 저장
            // heights.push_back(lz);
            // cxs.push_back(cx);
            // cys.push_back(cy);
            // low_zs.push_back(low_z);
            // lxs.push_back(lx);
            // lys.push_back(ly);

            // Detection3D 메시지 생성
            vision_msgs::msg::Detection3D detection;
            detection.bbox.center.position.x = cx;
            detection.bbox.center.position.y = cy;
            detection.bbox.center.position.z = cz;
            detection.bbox.size.x = lx;
            detection.bbox.size.y = ly;
            detection.bbox.size.z = lz;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = "1"; // 일단 차량으로 가정
            hyp.hypothesis.score = 1.0;
            detection.results.push_back(hyp);

            detections_msg.detections.push_back(detection);
        }

        // // (디버깅) 필터링된 클러스터들만 출력
        // if (!cxs.empty()) {
        //     printf("center_x:");
        //     for (float v : cxs) printf(" %.2f,", v);
        //     printf("\n");

        //     printf("center_y:");
        //     for (float v : cys) printf(" %.2f,", v);
        //     printf("\n");

        //     printf("low_z:");
        //     for (float v : low_zs) printf(" %.2f,", v);
        //     printf("\n");

        //     printf("length_x:");
        //     for (float v : lxs) printf(" %.2f,", v);
        //     printf("\n");

        //     printf("length_y:");
        //     for (float v : lys) printf(" %.2f,", v);
        //     printf("\n");

        //     printf("heights:");
        //     for (float v : heights) printf(" %.2f,", v);
        //     printf("\n");
        //     printf("\n");
        // }

        marker_pub_->publish(marker_array);
        marker_points_pub_->publish(detections_msg);
    }

    // ROS 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransac_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr marker_points_pub_;
};

// ------------------- main -------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClusteringNode>());
    rclcpp::shutdown();
    return 0;
}

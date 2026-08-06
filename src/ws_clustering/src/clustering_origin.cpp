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
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <cstdlib>

class LidarClusteringNode : public rclcpp::Node {
public:
    LidarClusteringNode() : Node("lidar_clustering_node") {
        // subscribe
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/merge_201/points", 10,
            std::bind(&LidarClusteringNode::cloud_callback, this, std::placeholders::_1));
        
        // publish
        noground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_noground", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_clusters", 10);

        RCLCPP_INFO(this->get_logger(), "✅ Lidar Clustering Node started.");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // ROS → PCL 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // 1. 다운샘플링 (Voxel Grid)
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*cloud_ds);

        // 2. ROI Crop (전방 50m, 좌우 7m, 높이 -1~4m)
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud_ds);
        crop.setMin(Eigen::Vector4f(-10, -7, -1, 1));
        crop.setMax(Eigen::Vector4f(50, 7, 4, 1));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*cloud_roi);

        // 3. RANSAC 기반 바닥 제거
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
        extract.setNegative(true); // 바닥 제외
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noground(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_noground);

        if (cloud_noground->empty()) return;

        // 중간 publish (지면 제거)
        sensor_msgs::msg::PointCloud2 noground_msg;
        pcl::toROSMsg(*cloud_noground, noground_msg);
        noground_msg.header = msg->header; // 시간/좌표계 유지
        noground_pub_->publish(noground_msg);

        // 4. 클러스터링
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_noground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);   // 같은 객체로 묶을 거리 (m)
        ec.setMinClusterSize(20);      // 최소 포인트 개수
        ec.setMaxClusterSize(5000);    // 최대 포인트 개수
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_noground);
        ec.extract(cluster_indices);

        // 5. 클러스터별 Bounding Box → MarkerArray 퍼블리시
        visualization_msgs::msg::MarkerArray marker_array;
        int cluster_id = 0;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) {
                cluster->points.push_back(cloud_noground->points[idx]);
            }

            // 중심점
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            // Bounding Box (min-max)
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
            marker.scale.x = (max_pt.x - min_pt.x);
            marker.scale.y = (max_pt.y - min_pt.y);
            marker.scale.z = (max_pt.z - min_pt.z);
            marker.color.a = 0.6;

            // 🎨 클러스터별 랜덤 색상
            marker.color.r = static_cast<float>(rand()) / RAND_MAX;
            marker.color.g = static_cast<float>(rand()) / RAND_MAX;
            marker.color.b = static_cast<float>(rand()) / RAND_MAX;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noground_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClusteringNode>());
    rclcpp::shutdown();
    return 0;
}

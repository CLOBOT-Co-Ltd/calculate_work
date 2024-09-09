#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h> // 다각형 프리즘 필터링
#include <pcl/filters/extract_indices.h>                   // 인덱스 필터링
#include <pcl/filters/crop_hull.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
        : Node("pointcloud_filter_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // PointCloud2 토픽 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // 필터링된 PointCloud2를 퍼블리시
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
        line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/trapezoid_line_marker", qos);
        text_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/trapezoid_text_marker", qos);


        // 초기 파라미터 설정 (각 꼭지점 좌표)
        this->declare_parameter("P1_x", 1.0);
        this->declare_parameter("P1_y", -0.7);
        this->declare_parameter("P1_z", -1.0);

        this->declare_parameter("P2_x", 2.6);
        this->declare_parameter("P2_y", -0.7);
        this->declare_parameter("P2_z", -1.0);

        this->declare_parameter("P3_x", 2.6);
        this->declare_parameter("P3_y", 0.7);
        this->declare_parameter("P3_z", -1.0);

        this->declare_parameter("P4_x", 1.0);
        this->declare_parameter("P4_y", 0.7);
        this->declare_parameter("P4_z", -1.0);

        this->declare_parameter("P5_x", 1.0);
        this->declare_parameter("P5_y", -0.7);
        this->declare_parameter("P5_z", 3.0);

        this->declare_parameter("P6_x", 2.6);
        this->declare_parameter("P6_y", -0.7);
        this->declare_parameter("P6_z", 3.0);

        this->declare_parameter("P7_x", 2.6);
        this->declare_parameter("P7_y", 0.7);
        this->declare_parameter("P7_z", 3.0);

        this->declare_parameter("P8_x", 1.0);
        this->declare_parameter("P8_y", 0.7);
        this->declare_parameter("P8_z", 3.0);

        // 초기 파라미터 값을 가져오기
        this->get_parameter("P1_x", P1_.x);
        this->get_parameter("P1_y", P1_.y);
        this->get_parameter("P1_z", P1_.z);

        this->get_parameter("P2_x", P2_.x);
        this->get_parameter("P2_y", P2_.y);
        this->get_parameter("P2_z", P2_.z);

        this->get_parameter("P3_x", P3_.x);
        this->get_parameter("P3_y", P3_.y);
        this->get_parameter("P3_z", P3_.z);

        this->get_parameter("P4_x", P4_.x);
        this->get_parameter("P4_y", P4_.y);
        this->get_parameter("P4_z", P4_.z);

        this->get_parameter("P5_x", P5_.x);
        this->get_parameter("P5_y", P5_.y);
        this->get_parameter("P5_z", P5_.z);

        this->get_parameter("P6_x", P6_.x);
        this->get_parameter("P6_y", P6_.y);
        this->get_parameter("P6_z", P6_.z);

        this->get_parameter("P7_x", P7_.x);
        this->get_parameter("P7_y", P7_.y);
        this->get_parameter("P7_z", P7_.z);

        this->get_parameter("P8_x", P8_.x);
        this->get_parameter("P8_y", P8_.y);
        this->get_parameter("P8_z", P8_.z);

        // 파라미터 업데이트 콜백 설정
        parameter_event_sub_ = this->add_on_set_parameters_callback(
            std::bind(&PointCloudFilterNode::parametersCallback, this, std::placeholders::_1));

        // 초기 마커 업데이트
        updateMarker();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 사다리꼴 기둥의 각 꼭지점 좌표를 저장할 변수
    geometry_msgs::msg::Point P1_, P2_, P3_, P4_, P5_, P6_, P7_, P8_;

    // 파라미터 업데이트 콜백 함수
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "P1_x")
                P1_.x = parameter.as_double();
            else if (parameter.get_name() == "P1_y")
                P1_.y = parameter.as_double();
            else if (parameter.get_name() == "P1_z")
                P1_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P2_x")
                P2_.x = parameter.as_double();
            else if (parameter.get_name() == "P2_y")
                P2_.y = parameter.as_double();
            else if (parameter.get_name() == "P2_z")
                P2_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P3_x")
                P3_.x = parameter.as_double();
            else if (parameter.get_name() == "P3_y")
                P3_.y = parameter.as_double();
            else if (parameter.get_name() == "P3_z")
                P3_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P4_x")
                P4_.x = parameter.as_double();
            else if (parameter.get_name() == "P4_y")
                P4_.y = parameter.as_double();
            else if (parameter.get_name() == "P4_z")
                P4_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P5_x")
                P5_.x = parameter.as_double();
            else if (parameter.get_name() == "P5_y")
                P5_.y = parameter.as_double();
            else if (parameter.get_name() == "P5_z")
                P5_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P6_x")
                P6_.x = parameter.as_double();
            else if (parameter.get_name() == "P6_y")
                P6_.y = parameter.as_double();
            else if (parameter.get_name() == "P6_z")
                P6_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P7_x")
                P7_.x = parameter.as_double();
            else if (parameter.get_name() == "P7_y")
                P7_.y = parameter.as_double();
            else if (parameter.get_name() == "P7_z")
                P7_.z = parameter.as_double();
            
            else if (parameter.get_name() == "P8_x")
                P8_.x = parameter.as_double();
            else if (parameter.get_name() == "P8_y")
                P8_.y = parameter.as_double();
            else if (parameter.get_name() == "P8_z")
                P8_.z = parameter.as_double();
        }
        RCLCPP_INFO(this->get_logger(), "Updated trapezoidal prism parameters.");
        updateMarker(); // 파라미터 변경 시 마커 업데이트
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 좌표 변환 수행: laser_frame -> base_link
        sensor_msgs::msg::PointCloud2 cloud_in_base_link;
        try
        {
            cloud_in_base_link = tf_buffer_.transform(*msg, "base_link", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        // ROS2 메시지를 PCL 메시지로 변환
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud_in_base_link, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // 사다리꼴 기둥의 점들을 정의하기 위한 포인트 클라우드 생성
        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon(new pcl::PointCloud<pcl::PointXYZ>);
        polygon->points.push_back(pcl::PointXYZ(P1_.x, P1_.y, P1_.z));
        polygon->points.push_back(pcl::PointXYZ(P2_.x, P2_.y, P2_.z));
        polygon->points.push_back(pcl::PointXYZ(P3_.x, P3_.y, P3_.z));
        polygon->points.push_back(pcl::PointXYZ(P4_.x, P4_.y, P4_.z));
        polygon->points.push_back(pcl::PointXYZ(P5_.x, P5_.y, P5_.z));
        polygon->points.push_back(pcl::PointXYZ(P6_.x, P6_.y, P6_.z));
        polygon->points.push_back(pcl::PointXYZ(P7_.x, P7_.y, P7_.z));
        polygon->points.push_back(pcl::PointXYZ(P8_.x, P8_.y, P8_.z));

        // 다각형을 삼각형으로 분할하여 정의
        std::vector<pcl::Vertices> hull_polygons;

        // 밑면 삼각형 분할 (P1, P2, P3, P4)
        pcl::Vertices v1, v2;
        v1.vertices = {0, 1, 2}; // 삼각형 1
        v2.vertices = {0, 2, 3}; // 삼각형 2
        hull_polygons.push_back(v1);
        hull_polygons.push_back(v2);

        // 윗면 삼각형 분할 (P5, P6, P7, P8)
        pcl::Vertices v3, v4;
        v3.vertices = {4, 5, 6}; // 삼각형 3
        v4.vertices = {4, 6, 7}; // 삼각형 4
        hull_polygons.push_back(v3);
        hull_polygons.push_back(v4);

        // 각 측면을 삼각형으로 정의 (총 8개의 삼각형이 필요)
        pcl::Vertices v5, v6, v7, v8;
        v5.vertices = {0, 1, 5}; // 삼각형 5 (P1, P2, P6)
        v6.vertices = {0, 5, 4}; // 삼각형 6 (P1, P6, P5)
        v7.vertices = {1, 2, 6}; // 삼각형 7 (P2, P3, P7)
        v8.vertices = {1, 6, 5}; // 삼각형 8 (P2, P7, P6)
        pcl::Vertices v9, v10, v11, v12;
        v9.vertices = {2, 3, 7}; // 삼각형 9 (P3, P4, P8)
        v10.vertices = {2, 7, 6}; // 삼각형 10 (P3, P8, P7)
        v11.vertices = {3, 0, 4}; // 삼각형 11 (P4, P1, P5)
        v12.vertices = {3, 4, 7}; // 삼각형 12 (P4, P5, P8)

        hull_polygons.push_back(v5);
        hull_polygons.push_back(v6);
        hull_polygons.push_back(v7);
        hull_polygons.push_back(v8);
        hull_polygons.push_back(v9);
        hull_polygons.push_back(v10);
        hull_polygons.push_back(v11);
        hull_polygons.push_back(v12);

        // CropHull 필터 설정
        pcl::CropHull<pcl::PointXYZ> cropHullFilter;
        cropHullFilter.setInputCloud(cloud);
        cropHullFilter.setHullCloud(polygon);
        cropHullFilter.setHullIndices(hull_polygons);
        cropHullFilter.setDim(3); // 3D 필터링

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        cropHullFilter.filter(*cloud_filtered);

        // 필터링된 PCL 메시지를 ROS2 메시지로 변환
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = cloud_in_base_link.header;

        // 필터링된 메시지 퍼블리시
        publisher_->publish(output);
    }

    void updateMarker()
    {
        // 선 마커 생성 및 퍼블리시
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "base_link"; // base_link 프레임으로 변경
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "trapezoid";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.07; // 선의 두께
        line_marker.color.a = 1.0;  // 알파 (투명도)
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;

        // 마커에 점 추가
        line_marker.points.push_back(P1_);
        line_marker.points.push_back(P2_);
        line_marker.points.push_back(P2_);
        line_marker.points.push_back(P3_);
        line_marker.points.push_back(P3_);
        line_marker.points.push_back(P4_);
        line_marker.points.push_back(P4_);
        line_marker.points.push_back(P1_);

        line_marker.points.push_back(P5_);
        line_marker.points.push_back(P6_);
        line_marker.points.push_back(P6_);
        line_marker.points.push_back(P7_);
        line_marker.points.push_back(P7_);
        line_marker.points.push_back(P8_);
        line_marker.points.push_back(P8_);
        line_marker.points.push_back(P5_);

        line_marker.points.push_back(P1_);
        line_marker.points.push_back(P5_);
        line_marker.points.push_back(P2_);
        line_marker.points.push_back(P6_);
        line_marker.points.push_back(P3_);
        line_marker.points.push_back(P7_);
        line_marker.points.push_back(P4_);
        line_marker.points.push_back(P8_);

        // 선 마커 퍼블리시
        line_marker_publisher_->publish(line_marker);

        // 각 꼭지점에 이름을 표시하는 텍스트 마커 생성 및 퍼블리시
        createTextMarker(P1_, "P1", 1);

        createTextMarker(P2_, "P2", 2);

        createTextMarker(P3_, "P3", 3);

        createTextMarker(P4_, "P4", 4);

        createTextMarker(P5_, "P5", 5);

        createTextMarker(P6_, "P6", 6);

        createTextMarker(P7_, "P7", 7);

        createTextMarker(P8_, "P8", 8);
    }

    void createTextMarker(const geometry_msgs::msg::Point &position, const std::string &text, int id)
    {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "base_link";
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "trapezoid_text";
        text_marker.id = id;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.scale.z = 0.5;  // 텍스트 크기
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.pose.position = position;

        // P1, P2, P3, P4의 경우 z 좌표보다 -0.5 아래에 표시
        if (text == "P1" || text == "P2" || text == "P3" || text == "P4") {
            text_marker.pose.position.z -= 0.5;
        } else {
            text_marker.pose.position.z += 0.5; // 나머지 점은 위로 표시
        }
        text_marker.text = text;

        // 텍스트 마커 퍼블리시
        text_marker_publisher_->publish(text_marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

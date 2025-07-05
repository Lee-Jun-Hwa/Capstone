// // Copyright 2023 Intel Corporation. All Rights Reserved.
// // Licensed under the Apache License, Version 2.0 (the "License");
// // You may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //     http://www.apache.org/licenses/LICENSE-2.0
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#include <named_filter.h>
#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace realsense2_camera;

NamedFilter::NamedFilter(std::shared_ptr<rs2::filter> filter, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled, bool is_set_parameters) : _filter(filter), _is_enabled(is_enabled), _params(parameters, logger), _logger(logger)
{
    if (is_set_parameters)
        setParameters();
}

void NamedFilter::setParameters(std::function<void(const rclcpp::Parameter &)> enable_param_func)
{
    std::stringstream module_name_str;
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    module_name_str << module_name;
    _params.registerDynamicOptions(*(_filter.get()), module_name_str.str());
    module_name_str << ".enable";

    _params.getParameters()->setParamT(module_name_str.str(), _is_enabled, enable_param_func);
    _parameters_names.push_back(module_name_str.str());
}

void NamedFilter::clearParameters()
{
    while (!_parameters_names.empty())
    {
        auto name = _parameters_names.back();
        _params.getParameters()->removeParam(name);
        _parameters_names.pop_back();
    }
}

rs2::frameset NamedFilter::Process(rs2::frameset frameset)
{
    return _is_enabled ? _filter->process(frameset) : frameset;
}

rs2::frame NamedFilter::Process(rs2::frame frame)
{
    return _is_enabled ? _filter->process(frame) : frame;
}

PointcloudFilter::PointcloudFilter(std::shared_ptr<rs2::filter> filter, rclcpp::Node &node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled) : NamedFilter(filter, parameters, logger, is_enabled, false),
                                                                                                                                                                              _node(node),
                                                                                                                                                                              _allow_no_texture_points(ALLOW_NO_TEXTURE_POINTS),
                                                                                                                                                                              _ordered_pc(ORDERED_PC)
{
    setParameters();
}

void PointcloudFilter::setParameters()
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));

    std::string param_name = module_name + ".allow_no_texture_points";
    _params.getParameters()->setParamT(param_name, _allow_no_texture_points);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".ordered_pc";
    _params.getParameters()->setParamT(param_name, _ordered_pc);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".pointcloud_qos";
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Available options are:\n" + list_available_qos_strings();

    _pointcloud_qos = _params.getParameters()->setParam<std::string>(param_name, DEFAULT_QOS, [this](const rclcpp::Parameter &parameter)
                                                                     {
        try {
            qos_string_to_qos(parameter.get_value<std::string>());
            _pointcloud_qos = parameter.get_value<std::string>();
            ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Invalid QoS value: " << parameter.get_value<std::string>());
            _params.getParameters()->queueSetRosValue(parameter.get_name(), _pointcloud_qos);
        } }, descriptor);
    _parameters_names.push_back(param_name);

    NamedFilter::setParameters([this](const rclcpp::Parameter &)
                               { setPublisher(); });
}

void PointcloudFilter::setPublisher()
{
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
    if ((_is_enabled) && (!_pointcloud_publisher))
    {
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/color/points",
                                                                                      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_string_to_qos(_pointcloud_qos)),
                                                                                                  qos_string_to_qos(_pointcloud_qos)));
    }
    else if ((!_is_enabled) && (_pointcloud_publisher))
    {
        _pointcloud_publisher.reset();
    }
}

void reverse_memcpy(unsigned char *dst, const unsigned char *src, size_t n)
{
    for (size_t i = 0; i < n; ++i)
        dst[n - 1 - i] = src[i];
}

void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time &t, const rs2::frameset &, const std::string &frame_id)
{
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (!_pointcloud_publisher || !_pointcloud_publisher->get_subscription_count())
            return;
    }

    const rs2::vertex *vertex = pc.get_vertices();
    rs2_intrinsics intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::msg::PointCloud2::UniquePtr msg_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.size());

    if (_ordered_pc)
    {
        msg_pointcloud->width = intrin.width;
        msg_pointcloud->height = intrin.height;
        msg_pointcloud->is_dense = false;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg_pointcloud, "z");

    size_t valid_count = 0;
    for (size_t i = 0; i < pc.size(); ++i, ++vertex)
    {
        if (vertex->z > 0 && vertex->z < 5 && vertex->y < 2)             //
        {
            *iter_x = vertex->x;
            *iter_y = vertex->y;
            *iter_z = vertex->z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++valid_count;
        }
    }

    msg_pointcloud->header.stamp = t;
    msg_pointcloud->header.frame_id = frame_id;
    if (!_ordered_pc)
    {
        msg_pointcloud->width = valid_count;
        msg_pointcloud->height = 1;
        msg_pointcloud->is_dense = true;
        modifier.resize(valid_count);
    }

    // ===== âœ… VoxelGrid í•„í„° ì ìš© =====
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg_pointcloud, *pcl_cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.025f, 0.025f, 0.025f); // voxel í¬ê¸° ì¡°ì •

    pcl::PCLPointCloud2::Ptr pcl_filtered(new pcl::PCLPointCloud2());
    voxel_filter.filter(*pcl_filtered);

    sensor_msgs::msg::PointCloud2::UniquePtr filtered_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::fromPCL(*pcl_filtered, *filtered_msg);

    filtered_msg->header.stamp = t;
    filtered_msg->header.frame_id = frame_id;

    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_pointcloud_publisher)
            _pointcloud_publisher->publish(std::move(filtered_msg));
    }
}

AlignDepthFilter::AlignDepthFilter(std::shared_ptr<rs2::filter> filter,
                                   std::function<void(const rclcpp::Parameter &)> update_align_depth_func,
                                   std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled) : NamedFilter(filter, parameters, logger, is_enabled, false)
{
    _params.registerDynamicOptions(*(_filter.get()), "align_depth");
    _params.getParameters()->setParamT("align_depth.enable", _is_enabled, update_align_depth_func);
    _parameters_names.push_back("align_depth.enable");
}

/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/output_pose.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <fstream>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/ros_map_writing_points_processor.h"
#include "cartographer_ros/split_string.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer_ros
{
namespace
{

constexpr char kTfStaticTopic[] = "/tf_static";
namespace carto = ::cartographer;

typedef Eigen::Matrix<float, 4, 3> Matrix43f;

std::unique_ptr<carto::common::LuaParameterDictionary> LoadLuaDictionary(
    const std::string &configuration_directory,
    const std::string &configuration_basename)
{
    auto file_resolver =
        carto::common::make_unique<carto::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});

    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    auto lua_parameter_dictionary =
        carto::common::make_unique<carto::common::LuaParameterDictionary>(
            code, std::move(file_resolver));
    return lua_parameter_dictionary;
}

template <typename T>
Matrix43f HandleMessage(
    const T &message, const std::string &tracking_frame,
    const tf2_ros::Buffer &tf_buffer,
    const carto::transform::TransformInterpolationBuffer &
        transform_interpolation_buffer)
{
    const carto::common::Time start_time = FromRos(message.header.stamp);

    carto::sensor::PointCloudWithIntensities point_cloud;
    carto::common::Time point_cloud_time;
    std::tie(point_cloud, point_cloud_time) =
        ToPointCloudWithIntensities(message);
    CHECK_EQ(point_cloud.intensities.size(), point_cloud.points.size());
    Matrix43f pose;

    for (size_t i = 0; i < point_cloud.points.size(); ++i)
    {
        const carto::common::Time time =
            point_cloud_time + carto::common::FromSeconds(point_cloud.points[i][3]);
        if (!transform_interpolation_buffer.Has(time))
        {
            continue;
        }
        const carto::transform::Rigid3d tracking_to_map =
            transform_interpolation_buffer.Lookup(time);
        const carto::transform::Rigid3d sensor_to_tracking =
            ToRigid3d(tf_buffer.lookupTransform(
                tracking_frame, message.header.frame_id, ToRos(time)));
        const carto::transform::Rigid3f sensor_to_map =
            (tracking_to_map * sensor_to_tracking).cast<float>();

        Eigen::Matrix3f rotation(sensor_to_map.rotation());

        pose.block<3, 3>(0, 0) = rotation;
        pose.block<1, 3>(3, 0) = sensor_to_map.translation();

        break;
    }

    return pose;
}

} // namespace

OutputPose::OutputPose(const std::string &pose_graph_filename,
                       const std::vector<std::string> &bag_filenames,
                       const std::string &output_file_prefix)
    : bag_filenames_(bag_filenames),
      pose_graph_(
          carto::io::DeserializePoseGraphFromFile(pose_graph_filename))
{
    CHECK_EQ(pose_graph_.trajectory_size(), bag_filenames_.size())
        << "Pose graphs contains " << pose_graph_.trajectory_size()
        << " trajectories while " << bag_filenames_.size()
        << " bags were provided. This tool requires one bag for each "
           "trajectory in the same order as the correponding trajectories in the "
           "pose graph proto.";

    // This vector must outlive the pipeline.
    all_trajectories_ = std::vector<::cartographer::mapping::proto::Trajectory>(
        pose_graph_.trajectory().begin(), pose_graph_.trajectory().end());

    const std::string file_prefix = !output_file_prefix.empty()
                                        ? output_file_prefix
                                        : bag_filenames_.front() + "_";
    output_path = file_prefix + "pose.txt";
}

void OutputPose::Run(const std::string &configuration_directory,
                     const std::string &configuration_basename,
                     const std::string &urdf_filename,
                     const bool use_bag_transforms)
{
    const auto lua_parameter_dictionary =
        LoadLuaDictionary(configuration_directory, configuration_basename);

    const std::string tracking_frame =
        lua_parameter_dictionary->GetString("tracking_frame");
    const std::string topic_name =
        lua_parameter_dictionary->GetString("topic_name");

    std::ofstream pose_txt;
    pose_txt.open(output_path);
    int message_number = 0;
    for (size_t trajectory_id = 0; trajectory_id < bag_filenames_.size();
         ++trajectory_id)
    {
        const carto::mapping::proto::Trajectory &trajectory_proto =
            pose_graph_.trajectory(trajectory_id);
        const std::string &bag_filename = bag_filenames_[trajectory_id];
        LOG(INFO) << "Processing " << bag_filename << "...";
        if (trajectory_proto.node_size() == 0)
        {
            continue;
        }
        tf2_ros::Buffer tf_buffer;
        if (!urdf_filename.empty())
        {
            ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
        }

        const carto::transform::TransformInterpolationBuffer
            transform_interpolation_buffer(trajectory_proto);
        rosbag::Bag bag;
        bag.open(bag_filename, rosbag::bagmode::Read);
        rosbag::View view(bag);
        const ::ros::Time begin_time = view.getBeginTime();
        const double duration_in_seconds =
            (view.getEndTime() - begin_time).toSec();

        // We need to keep 'tf_buffer' small because it becomes very inefficient
        // otherwise. We make sure that tf_messages are published before any data
        // messages, so that tf lookups always work.
        std::deque<rosbag::MessageInstance> delayed_messages;
        // We publish tf messages one second earlier than other messages. Under
        // the assumption of higher frequency tf this should ensure that tf can
        // always interpolate.
        const ::ros::Duration kDelay(0.1);
        bool first_line = true;

        for (const rosbag::MessageInstance &message : view)
        {
            if (use_bag_transforms && message.isType<tf2_msgs::TFMessage>())
            {
                auto tf_message = message.instantiate<tf2_msgs::TFMessage>();
                for (const auto &transform : tf_message->transforms)
                {
                    try
                    {
                        tf_buffer.setTransform(transform, "unused_authority",
                                               message.getTopic() == kTfStaticTopic);
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        LOG(WARNING) << ex.what();
                    }
                }
            }

            while (!delayed_messages.empty() && delayed_messages.front().getTime() <
                                                    message.getTime() - kDelay)
            {
                const rosbag::MessageInstance &delayed_message =
                    delayed_messages.front();

                Matrix43f pose;
                bool pt_flag = false;
                if (delayed_message.isType<sensor_msgs::PointCloud2>())
                {  
                    pose = HandleMessage(
                        *delayed_message.instantiate<sensor_msgs::PointCloud2>(),
                        tracking_frame, tf_buffer, transform_interpolation_buffer);
                    message_number++;
                    pt_flag = true;
                }
                else if (delayed_message
                             .isType<sensor_msgs::MultiEchoLaserScan>())
                {
                    pose = HandleMessage(
                        *delayed_message.instantiate<sensor_msgs::MultiEchoLaserScan>(),
                        tracking_frame, tf_buffer, transform_interpolation_buffer);
                    message_number++;
                    pt_flag = true;
                }
                else if (delayed_message.isType<sensor_msgs::LaserScan>())
                {
                    pose = HandleMessage(
                        *delayed_message.instantiate<sensor_msgs::LaserScan>(),
                        tracking_frame, tf_buffer, transform_interpolation_buffer);
                    message_number++;
                    pt_flag = true;
                }

                if (pt_flag)
                {
                    if (!first_line)
                    {
                        pose_txt << "\n";
                    }
                    pose_txt << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(3,0) << " "
                             << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(3,1) << " "
                             << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(3,2);
                    first_line = false;
                }
                delayed_messages.pop_front();
            }
            if (message.getTopic()==topic_name)
            {
                delayed_messages.push_back(message);
            }
            LOG_EVERY_N(INFO, 100000)
                << "Processed " << (message.getTime() - begin_time).toSec()
                << " of " << duration_in_seconds << " bag time seconds...";
        }
        bag.close();
    }
    pose_txt.close();
    LOG(INFO) << "Message Number " << message_number << "...";
}

} // namespace cartographer_ros

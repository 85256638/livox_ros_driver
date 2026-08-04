//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"

#include <chrono>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <mutex>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include "lds_lidar.h"
#include "lds_lvx.h"

namespace livox_ros {

/** Cap on zero-point packets back-filled into a single published cloud. The
 *  publisher inserts zero packets to bridge timestamp gaps left by lost
 *  packets so timestamps stay continuous; without a cap, a long dropout or
 *  heavy loss would flood downstream with whole clouds of empty points for the
 *  entire gap duration. Once the cap is reached we resync to the next real
 *  packet instead of synthesizing more zeros. */
static const uint32_t kMaxZeroFillPacketPerMsg = 10;

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      enable_lidar_bag_(lidar_bag),
      enable_imu_bag_(imu_bag) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  cur_node_ = nullptr;
  bag_ = nullptr;
  max_distance_ = 0.0f;
};

Lddc::~Lddc() {
  if (global_pub_) {
    delete global_pub_;
  }

  if (global_imu_pub_) {
    delete global_imu_pub_;
  }

  if (lds_) {
    lds_->PrepareExit();
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i];
    }
  }
}

int32_t Lddc::GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                                  uint64_t *start_time,
                                  StoragePacket *storage_packet) {
  QueuePrePop(queue, storage_packet);
  LivoxEthPacket *raw_packet =
      reinterpret_cast<LivoxEthPacket *>(storage_packet->raw_data);
  const uint8_t alignment_data_type = raw_packet->data_type;
  uint32_t packet_interval =
      GetPacketInterval(lidar->info.type, raw_packet->data_type);
  uint32_t packet_interval_max =
      static_cast<uint32_t>(packet_interval * 1.8f);
  uint64_t timestamp =
      GetStoragePacketTimestamp(storage_packet, lidar->data_src);
  /** A return/coordinate type boundary has a different cadence. Start from
   *  the real packet timestamp instead of aligning/skipping it with the new
   *  cadence as though it were a loss gap. */
  if (lidar->last_published_data_type != 0xFF &&
      lidar->last_published_data_type != raw_packet->data_type) {
    *start_time = timestamp;
    return 0;
  }
  uint32_t remaining_time = timestamp % publish_period_ns_;
  uint32_t diff_time = publish_period_ns_ - remaining_time;
  /** Get start time, down to the period boundary */
  if (diff_time > (publish_period_ns_ / 4)) {
    // ROS_INFO("0 : %u", diff_time);
    *start_time = timestamp - remaining_time;
    return 0;
  } else if (diff_time <= packet_interval_max) {
    *start_time = timestamp;
    return 0;
  } else {
    /** Skip some packets up to the period boundary*/
    // ROS_INFO("2 : %u", diff_time);
    do {
      if (QueueIsEmpty(queue)) {
        break;
      }
      QueuePopUpdate(queue); /* skip packet */
      if (QueueIsEmpty(queue)) {
        break;
      }
      QueuePrePop(queue, storage_packet);
      raw_packet =
          reinterpret_cast<LivoxEthPacket *>(storage_packet->raw_data);
      /** Do not align through a return/coordinate-mode boundary. The boundary
       *  packet is only being peeked here (it has not been popped), so leave it
       *  for the next publish batch, which will start at its real timestamp. */
      if (raw_packet->data_type != alignment_data_type) {
        break;
      }
      packet_interval =
          GetPacketInterval(lidar->info.type, raw_packet->data_type);
      uint32_t last_remaning_time = remaining_time;
      timestamp = GetStoragePacketTimestamp(storage_packet, lidar->data_src);
      remaining_time = timestamp % publish_period_ns_;
      /** Flip to another period */
      if (last_remaning_time > remaining_time) {
        // ROS_INFO("Flip to another period, exit");
        break;
      }
      diff_time = publish_period_ns_ - remaining_time;
    } while (diff_time > packet_interval);

    /* the remaning packets in queue maybe not enough after skip */
    return -1;
  }
}

void Lddc::InitPointcloud2MsgHeader(sensor_msgs::PointCloud2& cloud) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(6);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::PointField::UINT8;
  cloud.point_step = sizeof(LivoxPointXyzrtl);
}

uint32_t Lddc::PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                                  uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  sensor_msgs::PointCloud2 cloud;
  InitPointcloud2MsgHeader(cloud);
  cloud.data.resize(packet_num * kMaxPointPerEthPacket *
                    sizeof(LivoxPointXyzrtl));
  cloud.point_step = sizeof(LivoxPointXyzrtl);

  uint8_t *point_base = cloud.data.data();
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t is_zero_packet = 0;
  uint32_t zero_packet_count = 0;
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    /** Parse each packet by its OWN data_type. The device-level
     *  lidar->raw_data_type tracks the newest packet; after a return-mode or
     *  coordinate reconfigure the queue can still hold packets of the previous
     *  type, and decoding those with the new type's converter reads/writes out
     *  of bounds (point_num was already stored per-packet at ingest). */
    uint32_t echo_num = GetEchoNumPerPoint(raw_packet->data_type);
    uint32_t packet_interval =
        GetPacketInterval(lidar->info.type, raw_packet->data_type);
    uint32_t packet_interval_max =
        static_cast<uint32_t>(packet_interval * 1.8f);
    bool data_type_changed =
        (lidar->last_published_data_type != 0xFF &&
         lidar->last_published_data_type != raw_packet->data_type);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if (!data_type_changed && (packet_gap > packet_interval_max) &&
        lidar->data_is_pubulished &&
        (zero_packet_count < kMaxZeroFillPacketPerMsg)) {
      // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
      //     packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
        ++zero_packet_count;
      }
    }
    /** Use the first packet timestamp as pointcloud2 msg timestamp */
    if (!published_packet) {
      cloud.header.stamp = ros::Time(timestamp / 1000000000.0);
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(raw_packet->data_type);
      if (pf_point_convert) {
        point_base = pf_point_convert(point_base, raw_packet,
            lidar->extrinsic_parameter, line_num);
      } else {
        /** Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      point_base = LivoxPointToPxyzrtl(point_base, raw_packet,
          lidar->extrinsic_parameter, line_num);
    }

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
      lidar->last_published_data_type = raw_packet->data_type;
      /** real packet consumed & converted (holder: DistributeLidarData lock) */
      ++lidar->statistic_info.publish_packet_count;
    } else {
      is_zero_packet = 0;
    }
    cloud.width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }
  cloud.row_step     = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense     = true;
  cloud.data.resize(cloud.row_step); /** Adjust to the real size */

  if (max_distance_ > 0.0f) {
    LivoxPointXyzrtl *all_points = reinterpret_cast<LivoxPointXyzrtl *>(cloud.data.data());
    size_t keep_count = 0;
    for (size_t i = 0; i < cloud.width; ++i) {
      float x = all_points[i].x, y = all_points[i].y, z = all_points[i].z;
      if (x*x + y*y + z*z <= max_distance_ * max_distance_) {
        if (keep_count != i) all_points[keep_count] = all_points[i];
        keep_count++;
      }
    }
    cloud.width    = static_cast<uint32_t>(keep_count);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step);
  }

  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
          cloud);
    }
  }
  if (published_packet != 0) {
    lds_->RecordPointCloudPublished(handle);
  }
  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

void Lddc::FillPointsToPclMsg(PointCloud::Ptr& pcl_msg, \
    LivoxPointXyzrtl* src_point, uint32_t num) {
  LivoxPointXyzrtl* point_xyzrtl = (LivoxPointXyzrtl*)src_point;
  for (uint32_t i = 0; i < num; i++) {
    pcl::PointXYZI point;
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.intensity = point_xyzrtl->reflectivity;
    ++point_xyzrtl;
    pcl_msg->points.push_back(point);
  }
}

/* for pcl::pxyzi */
uint32_t Lddc::PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                     uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id.assign(frame_id_);
  cloud->height = 1;
  cloud->width = 0;

  uint8_t point_buf[2048];
  uint32_t is_zero_packet = 0;
  uint32_t zero_packet_count = 0;
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    /** Per-packet data_type: see PublishPointcloud2 -- queued packets can
     *  predate a return-mode/coordinate reconfigure. */
    uint32_t echo_num = GetEchoNumPerPoint(raw_packet->data_type);
    uint32_t packet_interval =
        GetPacketInterval(lidar->info.type, raw_packet->data_type);
    uint32_t packet_interval_max =
        static_cast<uint32_t>(packet_interval * 1.8f);
    bool data_type_changed =
        (lidar->last_published_data_type != 0xFF &&
         lidar->last_published_data_type != raw_packet->data_type);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if (!data_type_changed && (packet_gap > packet_interval_max) &&
        lidar->data_is_pubulished &&
        (zero_packet_count < kMaxZeroFillPacketPerMsg)) {
      //ROS_INFO("Lidar[%d] packet time interval is %ldns", handle, packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
        ++zero_packet_count;
      }
    }
    if (!published_packet) {
      cloud->header.stamp = timestamp / 1000.0;  // to pcl ros time stamp
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(raw_packet->data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter, \
            line_num);
      } else {
        /* Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter, \
          line_num);
    }
    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToPclMsg(cloud, dst_point, single_point_num);
    if (!is_zero_packet) {
      QueuePopUpdate(queue);
      lidar->last_published_data_type = raw_packet->data_type;
      /** real packet consumed & converted (holder: DistributeLidarData lock) */
      ++lidar->statistic_info.publish_packet_count;
    } else {
      is_zero_packet = 0;
    }
    cloud->width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }

  if (max_distance_ > 0.0f) {
    size_t keep_count = 0;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto &pt = cloud->points[i];
      if (pt.x*pt.x + pt.y*pt.y + pt.z*pt.z <= max_distance_ * max_distance_) {
        if (keep_count != i) cloud->points[keep_count] = cloud->points[i];
        keep_count++;
      }
    }
    cloud->points.resize(keep_count);
    cloud->width = static_cast<uint32_t>(keep_count);
  }

  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
          cloud);
    }
  }
  if (published_packet != 0) {
    lds_->RecordPointCloudPublished(handle);
  }
  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

void Lddc::FillPointsToCustomMsg(livox_ros_driver::CustomMsg& livox_msg, \
    LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time, \
    uint32_t point_interval, uint32_t echo_num) {
  LivoxPointXyzrtl* point_xyzrtl = (LivoxPointXyzrtl*)src_point;
  for (uint32_t i = 0; i < num; i++) {
    livox_ros_driver::CustomPoint point;
    if (echo_num > 1) { /** dual return mode */
      point.offset_time = offset_time + (i / echo_num) * point_interval;
    } else {
      point.offset_time = offset_time + i * point_interval;
    }
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.reflectivity = point_xyzrtl->reflectivity;
    point.tag = point_xyzrtl->tag;
    point.line = point_xyzrtl->line;
    ++point_xyzrtl;
    livox_msg.points.push_back(point);
  }
}

uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue *queue,
                                       uint32_t packet_num, uint8_t handle) {
  static uint32_t msg_seq = 0;
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  livox_ros_driver::CustomMsg livox_msg;
  livox_msg.header.frame_id.assign(frame_id_);
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
  livox_msg.timebase = 0;
  livox_msg.point_num = 0;
  livox_msg.lidar_id = handle;

  uint8_t point_buf[2048];
  uint8_t data_source = lds_->lidars_[handle].data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t point_interval = GetPointInterval(lidar->info.type);
  uint32_t published_packet = 0;
  uint32_t packet_offset_time = 0;  /** uint:ns */
  uint32_t is_zero_packet = 0;
  uint32_t zero_packet_count = 0;
  while (published_packet < packet_num) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    /** Per-packet data_type: see PublishPointcloud2 -- queued packets can
     *  predate a return-mode/coordinate reconfigure. */
    uint32_t echo_num = GetEchoNumPerPoint(raw_packet->data_type);
    uint32_t packet_interval =
        GetPacketInterval(lidar->info.type, raw_packet->data_type);
    uint32_t packet_interval_max =
        static_cast<uint32_t>(packet_interval * 1.8f);
    bool data_type_changed =
        (lidar->last_published_data_type != 0xFF &&
         lidar->last_published_data_type != raw_packet->data_type);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if (!data_type_changed && (packet_gap > packet_interval_max) &&
        lidar->data_is_pubulished &&
        (zero_packet_count < kMaxZeroFillPacketPerMsg)) {
      // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
      // packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
        ++zero_packet_count;
      }
    }
    /** first packet */
    if (!published_packet) {
      livox_msg.timebase = timestamp;
      packet_offset_time = 0;
      /** convert to ros time stamp */
      livox_msg.header.stamp = ros::Time(timestamp / 1000000000.0);
    } else {
      packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(raw_packet->data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter, \
            line_num);
      } else {
        /* Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter, \
          line_num);
    }
    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToCustomMsg(livox_msg, dst_point, single_point_num, \
        packet_offset_time, point_interval, echo_num);

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
      lidar->last_published_data_type = raw_packet->data_type;
      /** real packet consumed & converted (holder: DistributeLidarData lock) */
      ++lidar->statistic_info.publish_packet_count;
    } else {
      is_zero_packet = 0;
    }

    livox_msg.point_num += single_point_num;
    last_timestamp = timestamp;
    ++published_packet;
  }

  if (max_distance_ > 0.0f) {
    size_t keep_count = 0;
    for (size_t i = 0; i < livox_msg.points.size(); ++i) {
      const auto &pt = livox_msg.points[i];
      if (pt.x*pt.x + pt.y*pt.y + pt.z*pt.z <= max_distance_ * max_distance_) {
        if (keep_count != i) livox_msg.points[keep_count] = livox_msg.points[i];
        keep_count++;
      }
    }
    livox_msg.points.resize(keep_count);
    livox_msg.point_num = static_cast<uint32_t>(keep_count);
  }

  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(livox_msg);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
          livox_msg);
    }
  }

  if (published_packet != 0) {
    lds_->RecordPointCloudPublished(handle);
  }

  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

uint32_t Lddc::PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle) {
  uint64_t timestamp = 0;
  uint32_t published_packet = 0;

  sensor_msgs::Imu imu_data;
  imu_data.header.frame_id = "livox_frame";

  uint8_t data_source = lds_->lidars_[handle].data_src;
  StoragePacket storage_packet;
  QueuePrePop(queue, &storage_packet);
  LivoxEthPacket *raw_packet =
      reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
  timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
  if (timestamp >= 0) {
    imu_data.header.stamp =
        ros::Time(timestamp / 1000000000.0);  // to ros time stamp
  }

  uint8_t point_buf[2048];
  LivoxImuDataProcess(point_buf, raw_packet);

  LivoxImuPoint *imu = (LivoxImuPoint *)point_buf;
  imu_data.angular_velocity.x = imu->gyro_x;
  imu_data.angular_velocity.y = imu->gyro_y;
  imu_data.angular_velocity.z = imu->gyro_z;
  imu_data.linear_acceleration.x = imu->acc_x;
  imu_data.linear_acceleration.y = imu->acc_y;
  imu_data.linear_acceleration.z = imu->acc_z;

  QueuePopUpdate(queue);
  ++published_packet;

  ros::Publisher *p_publisher = Lddc::GetCurrentImuPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(imu_data);
  } else {
    if (bag_ && enable_imu_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
          imu_data);
    }
  }
  return published_packet;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  while (!QueueIsEmpty(p_queue)) {
    uint32_t used_size = QueueUsedSize(p_queue);
    uint32_t onetime_publish_packets = lidar->onetime_publish_packets;
    if (used_size < onetime_publish_packets) {
      break;
    }

    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, onetime_publish_packets, handle);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, onetime_publish_packets, handle);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPointcloudData(p_queue, onetime_publish_packets, handle);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->imu_data;
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  while (!QueueIsEmpty(p_queue)) {
    PublishImuData(p_queue, 1, handle);
  }
}

void Lddc::DistributeLidarData(void) {
  if (lds_ == nullptr) {
    return;
  }
  if (!lds_->semaphore_.WaitFor(std::chrono::milliseconds(100))) {
    if (lds_->IsRequestExit()) {
      PrepareExit();
    }
    return;
  }
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    /** Hold the per-lidar lock across the whole poll so a concurrent
     *  ResetLidar (disconnect, runs on the SDK callback thread) cannot free
     *  the queue / clear lidar fields while we read them. Even the state
     *  eligibility check belongs inside the lock; an unlocked pre-check is a
     *  C++ data race and does not make the later re-check safe. */
    std::lock_guard<std::mutex> lk(lds_->data_lock_[lidar_id]);
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    if (kConnectStateSampling != lidar->connect_state) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);
    PollingLidarImuData(lidar_id, lidar);
  }

  if (lds_->IsRequestExit()) {
    PrepareExit();
  }
}

ros::Publisher *Lddc::GetCurrentPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               lds_->lidars_[handle].info.broadcast_code);
      ROS_INFO("Support multi topics.");
    } else {
      ROS_INFO("Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
      ROS_INFO(
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->advertise<livox_ros_driver::CustomMsg>(name_str,
                                                                queue_size);
      ROS_INFO(
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->advertise<PointCloud>(name_str, queue_size);
      ROS_INFO(
          "%s publish use pcl PointXYZI format, set ROS publisher queue "
          "size %d",
          name_str, queue_size);
    }
  }

  return *pub;
}

ros::Publisher *Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_imu_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      ROS_INFO("Support multi topics.");
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               lds_->lidars_[handle].info.broadcast_code);
    } else {
      ROS_INFO("Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->advertise<sensor_msgs::Imu>(name_str, queue_size);
    ROS_INFO("%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}

void Lddc::CreateBagFile(const std::string &file_name) {
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    ROS_INFO("Create bag file :%s!", file_name.c_str());
  }
}

void Lddc::PrepareExit(void) {
  if (bag_) {
    ROS_INFO("Waiting to save the bag file!");
    bag_->close();
    ROS_INFO("Save the bag file successfully!");
    delete bag_;
    bag_ = nullptr;
  }
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

}  // namespace livox_ros

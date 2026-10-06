// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_CUVSLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_
#define  ISAAC_ROS_CUVSLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_

#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>

#include "isaac_common/messaging/message_buffer.hpp"

// 学习入口：两条时间流的交织模板；同步多相机图像另由 MessageStreamSynchronizer 完成。
namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// This class is designed to generate an interleaved sequence of messages from two separate streams.
// Streams are named stream1 and stream2 and assumption is stream1 is faster than stream2.

template<class T1, class T2>
// 合并快流与慢流：一次回调携带一批 T1 和一个 T2；cuVSLAM 调用方把它们用于 IMU 与同步图像组。
class MessageStreamSequencer
{
public:
  using Callback = std::function<void (const std::vector<T1> &, const T2 &)>;

  // 声明两流缓存容量和超时阈值；模板本身只比较传入的整数时间戳与 epsilon，不转换时间单位。
  explicit MessageStreamSequencer(
    uint8_t size_stream1, int64_t eps_stream1, uint8_t size_stream2, int64_t eps_stream2);
  // 快流入口，把时间戳消息入队并尝试推进慢流批次。
  void CallbackStream1(const int64_t & timestamp, const T1 & msg);
  // 慢流入口，等待快流赶上或超时后发出批次。
  void CallbackStream2(const int64_t & timestamp, const T2 & msg);
  // 查询快流下一个缓存时间戳，具体空队列语义由 MessageBuffer 定义。
  int64_t GetNextTimeStampStream1();
  // 查询慢流下一个缓存时间戳，具体空队列语义由 MessageBuffer 定义。
  int64_t GetNextTimeStampStream2();
  // 查询快流当前缓存条数。
  size_t GetSizeStream1();
  // 查询慢流当前缓存条数。
  size_t GetSizeStream2();
  // 声明输出回调注册接口；注册前默认回调为空操作。
  void RegisterCallback(Callback callback);

private:
  isaac_common::messaging::MessageBuffer<T1> stream1_buff_;
  int64_t epsilon_stream1_;
  isaac_common::messaging::MessageBuffer<T2> stream2_buff_;
  int64_t epsilon_stream2_;
  Callback registered_callback_ = [](const std::vector<T1> &, const T2 &) {};
};

template<class T1, class T2>
// 按调用方提供的容量与 epsilon 初始化两条 MessageBuffer；不会自动把毫秒变为纳秒。
MessageStreamSequencer<T1, T2>::MessageStreamSequencer(
  uint8_t size_stream1, int64_t eps_stream1,
  uint8_t size_stream2, int64_t eps_stream2)
: stream1_buff_{size_stream1}, epsilon_stream1_{eps_stream1},
  stream2_buff_{size_stream2}, epsilon_stream2_{eps_stream2}
{
}

template<class T1, class T2>
// 快流新消息触发时间比较；过晚消息被丢弃，满足推进或超时条件时按慢流时间弹出快流批次。
void MessageStreamSequencer<T1, T2>::CallbackStream1(const int64_t & timestamp, const T1 & msg)
{
  stream1_buff_.Push(timestamp, msg);
  std::pair<std::vector<T1>, T2> result;
  auto curr_msg_ts = stream1_buff_.GetCurrentTimeStamp();
  auto last_msg_ts = stream1_buff_.GetLastTimeStamp();
  auto next_msg_ts = stream1_buff_.GetNextTimeStamp();
  auto stream2_curr_ts = stream2_buff_.GetCurrentTimeStamp();
  auto stream2_next_ts = stream2_buff_.GetNextTimeStamp();
  auto stream2_last_ts = stream2_buff_.GetLastTimeStamp();

  // Ignoring late stream1 msg
  if (curr_msg_ts < stream2_last_ts) {
    stream1_buff_.Pop();
    return;
  }

  // Timeouts w.r.t self and other stream
  // 所有比较都使用传入时间戳与 epsilon 的原数值；核对调用处单位是学习时必须做的接口检查。
  bool self_esp_timeout = std::abs(curr_msg_ts - last_msg_ts) > epsilon_stream1_;
  bool other_esp_timeout = std::abs(curr_msg_ts - stream2_curr_ts) > epsilon_stream2_;

  // Adding msgs till stream1 catches up with stream2
  if (curr_msg_ts < stream2_curr_ts && curr_msg_ts < stream2_next_ts &&
    (!other_esp_timeout && !self_esp_timeout))
  {
    return;
  } else {
    // Either timeout or stream1 has caught up and
    // all the msgs needed by stream2 for the current timestamp are present

    // Process all the stored msgs from stream2 till current time of stream1
    while ((curr_msg_ts >= stream2_next_ts || other_esp_timeout || self_esp_timeout) &&
      !stream2_buff_.IsEmpty())
    {
      // Process all the stored msgs from stream1 till current time of stream1
      while (!stream1_buff_.IsEmpty() && stream2_next_ts >= next_msg_ts) {
        result.first.emplace_back(stream1_buff_.Pop());
        next_msg_ts = stream1_buff_.GetNextTimeStamp();
      }
      result.second = stream2_buff_.Pop();
      // 每个慢流项消费对应快流批次；清空 result.first 后再处理下一个慢流项。
      registered_callback_(result.first, result.second);
      // Clearing stream1 msgs for the subsequent runs
      result.first.clear();
      stream2_next_ts = stream2_buff_.GetNextTimeStamp();
    }
  }
}

template<class T1, class T2>
// 慢流新消息触发排序；快流尚未赶上且未超时则等待，否则把可用快流消息与慢流消息组合输出。
void MessageStreamSequencer<T1, T2>::CallbackStream2(const int64_t & timestamp, const T2 & msg)
{
  stream2_buff_.Push(timestamp, msg);

  std::pair<std::vector<T1>, T2> result;
  auto curr_msg_ts = stream2_buff_.GetCurrentTimeStamp();
  auto last_msg_ts = stream2_buff_.GetLastTimeStamp();
  auto stream1_curr_ts = stream1_buff_.GetCurrentTimeStamp();
  auto stream1_next_ts = stream1_buff_.GetNextTimeStamp();

  // Ignoring late stream2 msg
  if (curr_msg_ts < stream1_buff_.GetLastTimeStamp()) {
    stream2_buff_.Pop();
    return;
  }

  // Timeouts w.r.t self and other stream
  bool self_timeout = std::abs(curr_msg_ts - last_msg_ts) > epsilon_stream2_;
  bool other_timeout = std::abs(curr_msg_ts - stream1_curr_ts) > epsilon_stream1_;

  // Adding msg and wait for other stream to catch-up
  if (curr_msg_ts > stream1_curr_ts && (!other_timeout && !self_timeout)) {
    return;
  } else {
    // Either timeout or stream1 has caught up and
    // all the msgs needed by stream2 for the current timestamp are present

    // Process all the stored msgs from stream2 till current time of stream1
    while (((curr_msg_ts >= stream1_next_ts && curr_msg_ts <= stream1_curr_ts) ||
      self_timeout || other_timeout) && !stream2_buff_.IsEmpty())
    {
      // Process all the stored msgs from stream1 till current time of stream1
      while (curr_msg_ts >= stream1_next_ts && !stream1_buff_.IsEmpty()) {
        result.first.emplace_back(stream1_buff_.Pop());
        stream1_next_ts = stream1_buff_.GetNextTimeStamp();
      }
      result.second = stream2_buff_.Pop();
      // 每个慢流项消费对应快流批次；清空 result.first 后再处理下一个慢流项。
      registered_callback_(result.first, result.second);
      // Clearing stream1 msgs for the subsequent runs
      result.first.clear();
    }
  }
}

template<class T1, class T2>
// 转发快流缓存的下一个时间戳查询，不修改缓存。
int64_t MessageStreamSequencer<T1, T2>::GetNextTimeStampStream1()
{
  return stream1_buff_.GetNextTimeStamp();
}

template<class T1, class T2>
// 转发慢流缓存的下一个时间戳查询，不修改缓存。
int64_t MessageStreamSequencer<T1, T2>::GetNextTimeStampStream2()
{
  return stream2_buff_.GetNextTimeStamp();
}

template<class T1, class T2>
// 转发快流缓存长度查询。
size_t MessageStreamSequencer<T1, T2>::GetSizeStream1() {return stream1_buff_.Size();}

template<class T1, class T2>
// 转发慢流缓存长度查询。
size_t MessageStreamSequencer<T1, T2>::GetSizeStream2() {return stream2_buff_.Size();}

template<class T1, class T2>
// 保存调用方回调，后续各批次在当前处理路径中直接调用它。
void MessageStreamSequencer<T1, T2>::RegisterCallback(Callback callback)
{
  registered_callback_ = callback;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CUVSLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_

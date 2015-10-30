/**
 * Plugin for creating CachedRawIMUData messages.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 10/16/2015
 * Copyright (c) 2015, Virginia Tech
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "kvh1750_trooper/kvh1750_trooper.h"

#include <pluginlib/class_list_macros.h>

namespace
{
  const size_t ImuCacheSize = 15;
}

namespace kvh1750
{

TrooperProcessor::TrooperProcessor() :
  _cache_size(TrooperProcessor::DefaultCacheSize),
  _counter(0)
{
  ros::NodeHandle nh = ros::NodeHandle("~");
  _pub = nh.advertise<trooper_mlc_msgs::CachedRawIMUData>("cached", 1);
}

TrooperProcessor::~TrooperProcessor()
{
}

/**
 * Adds a single IMU reading to the cached value. If the cache is full, this
 * resets the counter and returns true.
 */
void TrooperProcessor::process_message(const kvh::Message& msg)
{
  trooper_mlc_msgs::RawIMUData imu;
  uint32_t secs = 0;
  uint32_t nsecs = 0;
  msg.time(secs, nsecs);
  imu.imu_timestamp = static_cast<uint64_t>(secs * 1.0E6) +
    static_cast<uint64_t>(nsecs * 1.0E-3);
  imu.packet_count = _counter++;
  imu.dax = msg.gyro_x();
  imu.day = msg.gyro_y();
  imu.daz = msg.gyro_z();
  imu.ddx = msg.accel_x();
  imu.ddy = msg.accel_y();
  imu.ddz = msg.accel_z();

  _cached_msg.data[(_counter - 1) % _cache_size] = imu;

  if(_counter >= _cache_size)
  {
    msg.time(_cached_msg.header.stamp.sec, _cached_msg.header.stamp.nsec);
    _pub.publish(_cached_msg);
  }
}

void TrooperProcessor::set_link_name(const std::string& link)
{
  _cached_msg.header.frame_id = link;
}

}

PLUGINLIB_EXPORT_CLASS(kvh1750::TrooperProcessor, kvh::MessageProcessorBase)
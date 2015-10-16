/**
 * Plugin to process KVH messages into a trooper_mlc_msgs::CachedImuMessage
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 09/29/2015
 */
#ifndef _KVH1750_TROOPER_H_
#define _KVH1750_TROOPER_H_

#include "kvh1750/kvh_plugin.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#include <trooper_mlc_msgs/CachedRawIMUData.h>
#pragma GCC diagnostic pop

namespace kvh1750
{

class TrooperProcessor : public kvh::MessageProcessorBase
{
public:
  TrooperProcessor();
  virtual ~TrooperProcessor();

  virtual void process_message(const kvh::Message& msg);

  static const size_t DefaultCacheSize = 15;

protected:
  size_t _cache_size;
  size_t _counter;
  trooper_mlc_msgs::CachedRawIMUData _cached_msg;
  ros::Publisher _pub;
};

}

#endif

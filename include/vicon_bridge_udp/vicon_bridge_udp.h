/*
 *      Author: Vladimir Ivan
 *
 * Copyright (c) 2018, University Of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <vector>
#include <map>

#define BUFLEN 1024

#pragma pack(push, 1)
struct Header
{
    uint32_t id;
    uint8_t size;
};

struct TrackerObject
{
    uint8_t id;
    uint16_t size;
    char name[24];
    double tx;
    double ty;
    double tz;
    double rx;
    double ry;
    double rz;
};
#pragma pack(pop)

class Client
{
  public:
    Client();
    virtual ~Client(){};
    void bindPort(int port);
    void receiveTransforms(std::vector<geometry_msgs::TransformStamped> &data, const std::string &parent_frame, int block_size = 512);

  private:
    int socket_;
};

class ViconReceiver
{
  public:
    ViconReceiver();
    virtual ~ViconReceiver(){};
    void start();

  private:
    Client client_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    int port_;
    int block_size_;
    std::string tf_ref_frame_id_;
    std::string tracked_frame_suffix_;

    tf::TransformBroadcaster tf_broadcaster_;
    std::map<std::string, ros::Publisher> publishers;

    bool broadcast_tf_, publish_tf_;
};
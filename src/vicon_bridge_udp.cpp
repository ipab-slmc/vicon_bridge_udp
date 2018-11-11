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

#include <vicon_bridge_udp/vicon_bridge_udp.h>
#include <tf2/LinearMath/Quaternion.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <exception>

Client::Client()
{
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == -1)
    {
        throw std::runtime_error("Can't open socket!");
    }
}

void Client::bindPort(int port)
{
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        throw std::runtime_error(std::string("Can't bind port ") + std::to_string(port));
    }
}

void Client::receiveTransforms(std::vector<geometry_msgs::TransformStamped> &data, const std::string &parent_frame, int block_size)
{
    char buf[BUFLEN];
    int recv_len;
    if ((recv_len = recvfrom(socket_, buf, block_size, 0, nullptr, 0)) == -1)
    {
        throw std::runtime_error("Can't receive data!");
    }
    if (recv_len > 0)
    {
        ros::Time now = ros::Time::now();
        Header *head = (Header *)&buf[0];
        data.resize(head->size);
        for (int i = 0; i < head->size; i++)
        {
            TrackerObject *object = (TrackerObject *)&buf[sizeof(Header) + sizeof(TrackerObject) * i];
            if (object->size != sizeof(TrackerObject) - 3)
                throw std::runtime_error(std::string("Invalid data object size! Got ") + std::to_string(object->size) + std::string(" expecting ") + std::to_string(sizeof(TrackerObject) - 3));
            data[i].header.seq = head->id;
            data[i].header.stamp = now;
            data[i].header.frame_id = parent_frame;
            data[i].child_frame_id = std::string(object->name);
            data[i].transform.translation.x = object->tx * 1e-3;
            data[i].transform.translation.y = object->ty * 1e-3;
            data[i].transform.translation.z = object->tz * 1e-3;
            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY(object->ry, -object->rx, object->rz);
            data[i].transform.rotation.x = myQuaternion.x();
            data[i].transform.rotation.y = myQuaternion.y();
            data[i].transform.rotation.z = myQuaternion.z();
            data[i].transform.rotation.w = myQuaternion.w();
        }
    }
}

ViconReceiver::ViconReceiver() : nh_priv_("~"), tf_ref_frame_id_("world"), tracked_frame_suffix_("vicon"), port_(51001), block_size_(512)
{
    nh_priv_.param("datastream_port", port_, port_);
    nh_priv_.param("block_size", block_size_, block_size_);
    nh_priv_.param("tf_ref_frame_id", tf_ref_frame_id_, tf_ref_frame_id_);
    nh_priv_.param("broadcast_transform", broadcast_tf_, true);
    nh_priv_.param("publish_transform", publish_tf_, true);
    nh_priv_.param("tracked_frame_suffix", tracked_frame_suffix_, tracked_frame_suffix_);
    client_.bindPort(port_);
}

void ViconReceiver::start()
{
    std::vector<geometry_msgs::TransformStamped> trans;
    while (ros::ok())
    {
        client_.receiveTransforms(trans, tf_ref_frame_id_, block_size_);
        if (broadcast_tf_)
            tf_broadcaster_.sendTransform(trans);
        if (publish_tf_)
        {
            for (int i = 0; i < trans.size(); i++)
            {
                auto it = publishers.find(trans[i].child_frame_id);
                ros::Publisher *pub;
                if (it == publishers.end())
                {
                    ros::Publisher tmp = nh_priv_.advertise<geometry_msgs::TransformStamped>(trans[i].child_frame_id, 1000);
                    pub = &tmp;
                    publishers[trans[i].child_frame_id] = tmp;
                }
                else
                {
                    pub = &it->second;
                }
                pub->publish(trans[i]);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon");

    ros::AsyncSpinner aspin(1);
    aspin.start();
    ViconReceiver vr;
    vr.start();
    aspin.stop();
    return 0;
}
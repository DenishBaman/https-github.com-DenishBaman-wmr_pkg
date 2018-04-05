/**
 * @brief SubscribeTopic header
 * @file SubscribeTopic.h
 */

#ifndef _wmr_pkg_SUBSCRIBETOPIC_H_
#define _wmr_pkg_SUBSCRIBETOPIC_H_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <memory>

namespace wmr_pkg{

template<typename MsgType>
class SubscribeTopic{
    std::unique_ptr<MsgType> data_;
    ros::Subscriber topic_sub_;

    public:
        SubscribeTopic(ros::NodeHandle&, 
                        const std::string& ,int = 20);
        ~SubscribeTopic();

        void topic_callback(const typename MsgType::ConstPtr& );
        MsgType* get_data();
};

template<typename MsgType>
SubscribeTopic<MsgType>::SubscribeTopic(ros::NodeHandle& nh,
                                const std::string& topic_name,
                                int queue_size)
{
   topic_sub_ = nh.subscribe(   topic_name,
                                queue_size,
                                &SubscribeTopic<MsgType>::topic_callback,
                                this);
   data_.reset(new MsgType());
}


template<typename MsgType>
SubscribeTopic<MsgType>::~SubscribeTopic()
{ data_.reset(); }

template<typename MsgType>
void SubscribeTopic<MsgType>::topic_callback(const typename MsgType::ConstPtr& topic_data)
{
    data_.reset(new MsgType(*topic_data) ) ;
}

template<typename MsgType>
inline MsgType* SubscribeTopic<MsgType>::get_data()
{ return data_.get(); }

} //end namespace {wmr_pkg}


#endif

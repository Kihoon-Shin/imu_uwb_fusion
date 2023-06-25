#ifndef _ROS_hri_msgs_Gaze_h
#define _ROS_hri_msgs_Gaze_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class Gaze : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _sender_type;
      _sender_type sender;
      typedef const char* _receiver_type;
      _receiver_type receiver;

    Gaze():
      header(),
      sender(""),
      receiver("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_sender = strlen(this->sender);
      varToArr(outbuffer + offset, length_sender);
      offset += 4;
      memcpy(outbuffer + offset, this->sender, length_sender);
      offset += length_sender;
      uint32_t length_receiver = strlen(this->receiver);
      varToArr(outbuffer + offset, length_receiver);
      offset += 4;
      memcpy(outbuffer + offset, this->receiver, length_receiver);
      offset += length_receiver;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_sender;
      arrToVar(length_sender, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sender; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sender-1]=0;
      this->sender = (char *)(inbuffer + offset-1);
      offset += length_sender;
      uint32_t length_receiver;
      arrToVar(length_receiver, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_receiver; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_receiver-1]=0;
      this->receiver = (char *)(inbuffer + offset-1);
      offset += length_receiver;
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/Gaze"; };
    virtual const char * getMD5() override { return "1408dc110169ebd2a0cd704f3af52beb"; };

  };

}
#endif

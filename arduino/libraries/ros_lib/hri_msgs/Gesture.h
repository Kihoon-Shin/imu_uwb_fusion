#ifndef _ROS_hri_msgs_Gesture_h
#define _ROS_hri_msgs_Gesture_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class Gesture : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _gesture_type;
      _gesture_type gesture;
      enum { HANDS_ON_FACE =  1 };
      enum { ARMS_CROSSED =  2 };
      enum { LEFT_HAND_RAISED =  3 };
      enum { RIGHT_HAND_RAISED =  4 };
      enum { BOTH_HANDS_RAISED =  5 };
      enum { WAVING =  6  };
      enum { OTHER =  0 };

    Gesture():
      header(),
      gesture(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->gesture >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gesture);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->gesture =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gesture);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/Gesture"; };
    virtual const char * getMD5() override { return "c64cc745a3c18d0a7abe6aed5be4f345"; };

  };

}
#endif

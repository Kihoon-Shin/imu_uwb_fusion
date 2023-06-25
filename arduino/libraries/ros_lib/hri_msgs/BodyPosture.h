#ifndef _ROS_hri_msgs_BodyPosture_h
#define _ROS_hri_msgs_BodyPosture_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class BodyPosture : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _posture_type;
      _posture_type posture;
      enum { STANDING =  1 };
      enum { SITTING =  2 };
      enum { CROUCHING =  3 };
      enum { LAYING =  4 };
      enum { OTHER =  0 };

    BodyPosture():
      header(),
      posture(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->posture >> (8 * 0)) & 0xFF;
      offset += sizeof(this->posture);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->posture =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->posture);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/BodyPosture"; };
    virtual const char * getMD5() override { return "52f95070a71954a985e0ab92dd4d4eb9"; };

  };

}
#endif

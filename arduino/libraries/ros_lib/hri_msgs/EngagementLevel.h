#ifndef _ROS_hri_msgs_EngagementLevel_h
#define _ROS_hri_msgs_EngagementLevel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class EngagementLevel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _level_type;
      _level_type level;
      enum { UNKNOWN = 0 };
      enum { DISENGAGED = 1 };
      enum { ENGAGING = 2 };
      enum { ENGAGED = 3 };
      enum { DISENGAGING = 4 };

    EngagementLevel():
      header(),
      level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->level =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/EngagementLevel"; };
    virtual const char * getMD5() override { return "98693df082bea7da40fa598b187373d9"; };

  };

}
#endif

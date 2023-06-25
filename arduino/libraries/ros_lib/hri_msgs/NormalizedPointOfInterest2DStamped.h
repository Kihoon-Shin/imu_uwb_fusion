#ifndef _ROS_hri_msgs_NormalizedPointOfInterest2DStamped_h
#define _ROS_hri_msgs_NormalizedPointOfInterest2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class NormalizedPointOfInterest2DStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _c_type;
      _c_type c;

    NormalizedPointOfInterest2DStamped():
      header(),
      x(0),
      y(0),
      c(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_c;
      u_c.real = this->c;
      *(outbuffer + offset + 0) = (u_c.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->c);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_c;
      u_c.base = 0;
      u_c.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->c = u_c.real;
      offset += sizeof(this->c);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/NormalizedPointOfInterest2DStamped"; };
    virtual const char * getMD5() override { return "f00d620d5791659f1cba63fdcb50f444"; };

  };

}
#endif

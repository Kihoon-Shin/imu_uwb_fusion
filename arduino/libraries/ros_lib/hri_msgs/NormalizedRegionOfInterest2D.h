#ifndef _ROS_hri_msgs_NormalizedRegionOfInterest2D_h
#define _ROS_hri_msgs_NormalizedRegionOfInterest2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class NormalizedRegionOfInterest2D : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _xmin_type;
      _xmin_type xmin;
      typedef float _ymin_type;
      _ymin_type ymin;
      typedef float _xmax_type;
      _xmax_type xmax;
      typedef float _ymax_type;
      _ymax_type ymax;
      typedef float _c_type;
      _c_type c;

    NormalizedRegionOfInterest2D():
      header(),
      xmin(0),
      ymin(0),
      xmax(0),
      ymax(0),
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
      } u_xmin;
      u_xmin.real = this->xmin;
      *(outbuffer + offset + 0) = (u_xmin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xmin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xmin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xmin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xmin);
      union {
        float real;
        uint32_t base;
      } u_ymin;
      u_ymin.real = this->ymin;
      *(outbuffer + offset + 0) = (u_ymin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ymin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ymin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ymin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ymin);
      union {
        float real;
        uint32_t base;
      } u_xmax;
      u_xmax.real = this->xmax;
      *(outbuffer + offset + 0) = (u_xmax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xmax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xmax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xmax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xmax);
      union {
        float real;
        uint32_t base;
      } u_ymax;
      u_ymax.real = this->ymax;
      *(outbuffer + offset + 0) = (u_ymax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ymax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ymax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ymax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ymax);
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
      } u_xmin;
      u_xmin.base = 0;
      u_xmin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xmin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xmin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xmin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xmin = u_xmin.real;
      offset += sizeof(this->xmin);
      union {
        float real;
        uint32_t base;
      } u_ymin;
      u_ymin.base = 0;
      u_ymin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ymin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ymin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ymin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ymin = u_ymin.real;
      offset += sizeof(this->ymin);
      union {
        float real;
        uint32_t base;
      } u_xmax;
      u_xmax.base = 0;
      u_xmax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xmax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xmax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xmax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xmax = u_xmax.real;
      offset += sizeof(this->xmax);
      union {
        float real;
        uint32_t base;
      } u_ymax;
      u_ymax.base = 0;
      u_ymax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ymax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ymax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ymax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ymax = u_ymax.real;
      offset += sizeof(this->ymax);
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

    virtual const char * getType() override { return "hri_msgs/NormalizedRegionOfInterest2D"; };
    virtual const char * getMD5() override { return "33eb96af02d4b1dd1457132b5c2149c2"; };

  };

}
#endif

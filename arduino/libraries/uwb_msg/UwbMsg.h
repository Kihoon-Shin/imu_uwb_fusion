#ifndef _ROS_uwb_msg_UwbMsg_h
#define _ROS_uwb_msg_UwbMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace uwb_msg
{

  class UwbMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _pos_x_type;
      _pos_x_type pos_x;
      typedef float _pos_y_type;
      _pos_y_type pos_y;
      typedef float _pos_z_type;
      _pos_z_type pos_z;

    UwbMsg():
      header(),
      pos_x(0),
      pos_y(0),
      pos_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_pos_x;
      u_pos_x.real = this->pos_x;
      *(outbuffer + offset + 0) = (u_pos_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_x);
      union {
        float real;
        uint32_t base;
      } u_pos_y;
      u_pos_y.real = this->pos_y;
      *(outbuffer + offset + 0) = (u_pos_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_y);
      union {
        float real;
        uint32_t base;
      } u_pos_z;
      u_pos_z.real = this->pos_z;
      *(outbuffer + offset + 0) = (u_pos_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_pos_x;
      u_pos_x.base = 0;
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_x = u_pos_x.real;
      offset += sizeof(this->pos_x);
      union {
        float real;
        uint32_t base;
      } u_pos_y;
      u_pos_y.base = 0;
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_y = u_pos_y.real;
      offset += sizeof(this->pos_y);
      union {
        float real;
        uint32_t base;
      } u_pos_z;
      u_pos_z.base = 0;
      u_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_z = u_pos_z.real;
      offset += sizeof(this->pos_z);
     return offset;
    }

    virtual const char * getType() override { return "uwb_msg/UwbMsg"; };
    virtual const char * getMD5() override { return "9a165508c7865f10d0a4f133512d2f74"; };

  };

}
#endif

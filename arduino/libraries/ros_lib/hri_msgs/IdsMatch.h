#ifndef _ROS_hri_msgs_IdsMatch_h
#define _ROS_hri_msgs_IdsMatch_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hri_msgs
{

  class IdsMatch : public ros::Msg
  {
    public:
      typedef const char* _id1_type;
      _id1_type id1;
      typedef int8_t _id1_type_type;
      _id1_type_type id1_type;
      typedef const char* _id2_type;
      _id2_type id2;
      typedef int8_t _id2_type_type;
      _id2_type_type id2_type;
      typedef float _confidence_type;
      _confidence_type confidence;
      enum { UNSET = 0 };
      enum { PERSON = 1 };
      enum { FACE = 2 };
      enum { BODY = 3 };
      enum { VOICE = 4 };

    IdsMatch():
      id1(""),
      id1_type(0),
      id2(""),
      id2_type(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id1 = strlen(this->id1);
      varToArr(outbuffer + offset, length_id1);
      offset += 4;
      memcpy(outbuffer + offset, this->id1, length_id1);
      offset += length_id1;
      union {
        int8_t real;
        uint8_t base;
      } u_id1_type;
      u_id1_type.real = this->id1_type;
      *(outbuffer + offset + 0) = (u_id1_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id1_type);
      uint32_t length_id2 = strlen(this->id2);
      varToArr(outbuffer + offset, length_id2);
      offset += 4;
      memcpy(outbuffer + offset, this->id2, length_id2);
      offset += length_id2;
      union {
        int8_t real;
        uint8_t base;
      } u_id2_type;
      u_id2_type.real = this->id2_type;
      *(outbuffer + offset + 0) = (u_id2_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id2_type);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_id1;
      arrToVar(length_id1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id1-1]=0;
      this->id1 = (char *)(inbuffer + offset-1);
      offset += length_id1;
      union {
        int8_t real;
        uint8_t base;
      } u_id1_type;
      u_id1_type.base = 0;
      u_id1_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->id1_type = u_id1_type.real;
      offset += sizeof(this->id1_type);
      uint32_t length_id2;
      arrToVar(length_id2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id2-1]=0;
      this->id2 = (char *)(inbuffer + offset-1);
      offset += length_id2;
      union {
        int8_t real;
        uint8_t base;
      } u_id2_type;
      u_id2_type.base = 0;
      u_id2_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->id2_type = u_id2_type.real;
      offset += sizeof(this->id2_type);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/IdsMatch"; };
    virtual const char * getMD5() override { return "47ee5557c84afd004bec4ac7f5fa56f7"; };

  };

}
#endif

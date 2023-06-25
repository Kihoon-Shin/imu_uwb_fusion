#ifndef _ROS_hri_msgs_SoftBiometrics_h
#define _ROS_hri_msgs_SoftBiometrics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class SoftBiometrics : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _age_type;
      _age_type age;
      typedef float _age_confidence_type;
      _age_confidence_type age_confidence;
      typedef uint8_t _gender_type;
      _gender_type gender;
      typedef float _gender_confidence_type;
      _gender_confidence_type gender_confidence;
      enum { UNDEFINED = 0 };
      enum { FEMALE = 1 };
      enum { MALE = 2 };
      enum { OTHER = 3 };

    SoftBiometrics():
      header(),
      age(0),
      age_confidence(0),
      gender(0),
      gender_confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->age >> (8 * 0)) & 0xFF;
      offset += sizeof(this->age);
      union {
        float real;
        uint32_t base;
      } u_age_confidence;
      u_age_confidence.real = this->age_confidence;
      *(outbuffer + offset + 0) = (u_age_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_age_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_age_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_age_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->age_confidence);
      *(outbuffer + offset + 0) = (this->gender >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gender);
      union {
        float real;
        uint32_t base;
      } u_gender_confidence;
      u_gender_confidence.real = this->gender_confidence;
      *(outbuffer + offset + 0) = (u_gender_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gender_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gender_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gender_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gender_confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->age =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->age);
      union {
        float real;
        uint32_t base;
      } u_age_confidence;
      u_age_confidence.base = 0;
      u_age_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_age_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_age_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_age_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->age_confidence = u_age_confidence.real;
      offset += sizeof(this->age_confidence);
      this->gender =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gender);
      union {
        float real;
        uint32_t base;
      } u_gender_confidence;
      u_gender_confidence.base = 0;
      u_gender_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gender_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gender_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gender_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gender_confidence = u_gender_confidence.real;
      offset += sizeof(this->gender_confidence);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/SoftBiometrics"; };
    virtual const char * getMD5() override { return "791877398420c10707c4d9a832b0e6ad"; };

  };

}
#endif

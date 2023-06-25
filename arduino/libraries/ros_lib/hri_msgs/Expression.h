#ifndef _ROS_hri_msgs_Expression_h
#define _ROS_hri_msgs_Expression_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class Expression : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _expression_type;
      _expression_type expression;
      typedef float _valence_type;
      _valence_type valence;
      typedef float _arousal_type;
      _arousal_type arousal;
      typedef float _confidence_type;
      _confidence_type confidence;
      enum { NEUTRAL = "neutral" };
      enum { ANGRY = "angry" };
      enum { SAD = "sad" };
      enum { HAPPY = "happy" };
      enum { SURPRISED = "surprised" };
      enum { DISGUSTED = "disgusted" };
      enum { SCARED = "scared" };
      enum { PLEADING = "pleading" };
      enum { VULNERABLE = "vulnerable" };
      enum { DESPAIRED = "despaired" };
      enum { GUILTY = "guilty" };
      enum { DISAPPOINTED = "disappointed" };
      enum { EMBARRASSED = "embarrassed" };
      enum { HORRIFIED = "horrified" };
      enum { SKEPTICAL = "skeptical" };
      enum { ANNOYED = "annoyed" };
      enum { FURIOUS = "furious" };
      enum { SUSPICIOUS = "suspicious" };
      enum { REJECTED = "rejected" };
      enum { BORED = "bored" };
      enum { TIRED = "tired" };
      enum { ASLEEP = "asleep" };
      enum { CONFUSED = "confused" };
      enum { AMAZED = "amazed" };
      enum { EXCITED = "excited" };

    Expression():
      header(),
      expression(""),
      valence(0),
      arousal(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_expression = strlen(this->expression);
      varToArr(outbuffer + offset, length_expression);
      offset += 4;
      memcpy(outbuffer + offset, this->expression, length_expression);
      offset += length_expression;
      union {
        float real;
        uint32_t base;
      } u_valence;
      u_valence.real = this->valence;
      *(outbuffer + offset + 0) = (u_valence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->valence);
      union {
        float real;
        uint32_t base;
      } u_arousal;
      u_arousal.real = this->arousal;
      *(outbuffer + offset + 0) = (u_arousal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arousal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arousal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arousal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arousal);
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
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_expression;
      arrToVar(length_expression, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_expression; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_expression-1]=0;
      this->expression = (char *)(inbuffer + offset-1);
      offset += length_expression;
      union {
        float real;
        uint32_t base;
      } u_valence;
      u_valence.base = 0;
      u_valence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_valence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_valence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_valence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->valence = u_valence.real;
      offset += sizeof(this->valence);
      union {
        float real;
        uint32_t base;
      } u_arousal;
      u_arousal.base = 0;
      u_arousal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arousal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arousal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arousal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arousal = u_arousal.real;
      offset += sizeof(this->arousal);
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

    virtual const char * getType() override { return "hri_msgs/Expression"; };
    virtual const char * getMD5() override { return "c0f698742672d3238cdc31c07c7d835b"; };

  };

}
#endif

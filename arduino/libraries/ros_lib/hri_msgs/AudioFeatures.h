#ifndef _ROS_hri_msgs_AudioFeatures_h
#define _ROS_hri_msgs_AudioFeatures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hri_msgs
{

  class AudioFeatures : public ros::Msg
  {
    public:
      typedef float _ZCR_type;
      _ZCR_type ZCR;
      typedef float _RMS_type;
      _RMS_type RMS;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _HNR_type;
      _HNR_type HNR;
      uint32_t MFCC_length;
      typedef float _MFCC_type;
      _MFCC_type st_MFCC;
      _MFCC_type * MFCC;

    AudioFeatures():
      ZCR(0),
      RMS(0),
      pitch(0),
      HNR(0),
      MFCC_length(0), st_MFCC(), MFCC(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ZCR;
      u_ZCR.real = this->ZCR;
      *(outbuffer + offset + 0) = (u_ZCR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ZCR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ZCR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ZCR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ZCR);
      union {
        float real;
        uint32_t base;
      } u_RMS;
      u_RMS.real = this->RMS;
      *(outbuffer + offset + 0) = (u_RMS.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RMS.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RMS.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RMS.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RMS);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_HNR;
      u_HNR.real = this->HNR;
      *(outbuffer + offset + 0) = (u_HNR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HNR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HNR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HNR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->HNR);
      *(outbuffer + offset + 0) = (this->MFCC_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->MFCC_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->MFCC_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->MFCC_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MFCC_length);
      for( uint32_t i = 0; i < MFCC_length; i++){
      union {
        float real;
        uint32_t base;
      } u_MFCCi;
      u_MFCCi.real = this->MFCC[i];
      *(outbuffer + offset + 0) = (u_MFCCi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MFCCi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MFCCi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MFCCi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MFCC[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ZCR;
      u_ZCR.base = 0;
      u_ZCR.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ZCR.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ZCR.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ZCR.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ZCR = u_ZCR.real;
      offset += sizeof(this->ZCR);
      union {
        float real;
        uint32_t base;
      } u_RMS;
      u_RMS.base = 0;
      u_RMS.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RMS.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RMS.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RMS.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RMS = u_RMS.real;
      offset += sizeof(this->RMS);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_HNR;
      u_HNR.base = 0;
      u_HNR.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HNR.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HNR.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HNR.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->HNR = u_HNR.real;
      offset += sizeof(this->HNR);
      uint32_t MFCC_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      MFCC_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      MFCC_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      MFCC_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->MFCC_length);
      if(MFCC_lengthT > MFCC_length)
        this->MFCC = (float*)realloc(this->MFCC, MFCC_lengthT * sizeof(float));
      MFCC_length = MFCC_lengthT;
      for( uint32_t i = 0; i < MFCC_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_MFCC;
      u_st_MFCC.base = 0;
      u_st_MFCC.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_MFCC.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_MFCC.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_MFCC.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_MFCC = u_st_MFCC.real;
      offset += sizeof(this->st_MFCC);
        memcpy( &(this->MFCC[i]), &(this->st_MFCC), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/AudioFeatures"; };
    virtual const char * getMD5() override { return "1942f4cd8b6bb147f1ccb9aded9b6535"; };

  };

}
#endif

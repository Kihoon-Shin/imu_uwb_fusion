#ifndef _ROS_hri_msgs_FacialActionUnits_h
#define _ROS_hri_msgs_FacialActionUnits_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class FacialActionUnits : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t FAU_length;
      typedef float _FAU_type;
      _FAU_type st_FAU;
      _FAU_type * FAU;
      uint32_t intensity_length;
      typedef float _intensity_type;
      _intensity_type st_intensity;
      _intensity_type * intensity;
      uint32_t confidence_length;
      typedef float _confidence_type;
      _confidence_type st_confidence;
      _confidence_type * confidence;
      enum { NEUTRAL_FACE = 0   };
      enum { INNER_BROW_RAISER = 1   };
      enum { OUTER_BROW_RAISER = 2   };
      enum { BROW_LOWERER = 4   };
      enum { UPPER_LID_RAISER = 5   };
      enum { CHEEK_RAISER = 6   };
      enum { LID_TIGHTENER = 7   };
      enum { LIPS_TOWARD_EACH_OTHER = 8   };
      enum { NOSE_WRINKLER = 9   };
      enum { UPPER_LIP_RAISER = 10   };
      enum { NASOLABIAL_DEEPENER = 11   };
      enum { LIP_CORNER_PULLER = 12   };
      enum { SHARP_LIP_PULLER = 13   };
      enum { DIMPLER = 14   };
      enum { LIP_CORNER_DEPRESSOR = 15   };
      enum { LOWER_LIP_DEPRESSOR = 16   };
      enum { CHIN_RAISER = 17   };
      enum { LIP_PUCKER = 18   };
      enum { TONGUE_SHOW = 19   };
      enum { LIP_STRETCHER = 20   };
      enum { NECK_TIGHTENER = 21   };
      enum { LIP_FUNNELER = 22   };
      enum { LIP_TIGHTENER = 23   };
      enum { LIP_PRESSOR = 24   };
      enum { LIPS_PART = 25   };
      enum { JAW_DROP = 26   };
      enum { MOUTH_STRETCH = 27   };
      enum { LIP_SUCK = 28   };
      enum { HEAD_TURN_LEFT = 51   };
      enum { HEAD_TURN_RIGHT = 52   };
      enum { HEAD_UP = 53   };
      enum { HEAD_DOWN = 54   };
      enum { HEAD_TILT_LEFT = 55   };
      enum { HEAD_TILT_RIGHT = 56   };
      enum { HEAD_FORWARD = 57   };
      enum { HEAD_BACK = 58   };
      enum { EYES_TURN_LEFT = 61   };
      enum { EYES_TURN_RIGHT = 62   };
      enum { EYES_UP = 63   };
      enum { EYES_DOWN = 64   };
      enum { WALLEYE = 65   };
      enum { CROSS_EYE = 66   };
      enum { EYES_POSITIONED_TO_LOOK_AT_OTHER_PERSON = 69   };
      enum { BROWS_AND_FOREHEAD_NOT_VISIBLE = 70   };
      enum { EYES_NOT_VISIBLE = 71   };
      enum { LOWER_FACE_NOT_VISIBLE = 72   };
      enum { ENTIRE_FACE_NOT_VISIBLE = 73   };
      enum { UNSOCIABLE = 74   };
      enum { JAW_THRUST = 29   };
      enum { JAW_SIDEWAYS = 30   };
      enum { JAW_CLENCHER = 31   };
      enum { LIP_BITE = 32   };
      enum { CHEEK_BLOW = 33   };
      enum { CHEEK_PUFF = 34   };
      enum { CHEEK_SUCK = 35   };
      enum { TONGUE_BULGE = 36   };
      enum { LIP_WIPE = 37   };
      enum { NOSTRIL_DILATOR = 38   };
      enum { NOSTRIL_COMPRESSOR = 39   };
      enum { SNIFF = 40   };
      enum { LID_DROOP = 41   };
      enum { SLIT = 42   };
      enum { EYES_CLOSED = 43   };
      enum { SQUINT = 44   };
      enum { BLINK = 45   };
      enum { WINK = 46   };
      enum { SPEECH = 50   };
      enum { SWALLOW = 80   };
      enum { CHEWING = 81   };
      enum { SHOULDER_SHRUG = 82   };
      enum { HEAD_SHAKE_BACK_AND_FORTH = 84   };
      enum { HEAD_NOD_UP_AND_DOWN = 85   };
      enum { FLASH = 91   };
      enum { PARTIAL_FLASH = 92   };
      enum { SHIVER_TREMBLE = 97   };
      enum { FAST_UP_DOWN_LOOK = 98   };

    FacialActionUnits():
      header(),
      FAU_length(0), st_FAU(), FAU(nullptr),
      intensity_length(0), st_intensity(), intensity(nullptr),
      confidence_length(0), st_confidence(), confidence(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->FAU_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->FAU_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->FAU_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->FAU_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->FAU_length);
      for( uint32_t i = 0; i < FAU_length; i++){
      union {
        float real;
        uint32_t base;
      } u_FAUi;
      u_FAUi.real = this->FAU[i];
      *(outbuffer + offset + 0) = (u_FAUi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FAUi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FAUi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FAUi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->FAU[i]);
      }
      *(outbuffer + offset + 0) = (this->intensity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->intensity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->intensity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->intensity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensity_length);
      for( uint32_t i = 0; i < intensity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_intensityi;
      u_intensityi.real = this->intensity[i];
      *(outbuffer + offset + 0) = (u_intensityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intensityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intensityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intensityi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensity[i]);
      }
      *(outbuffer + offset + 0) = (this->confidence_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->confidence_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->confidence_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->confidence_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_length);
      for( uint32_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_confidencei;
      u_confidencei.real = this->confidence[i];
      *(outbuffer + offset + 0) = (u_confidencei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidencei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidencei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidencei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t FAU_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      FAU_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      FAU_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      FAU_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->FAU_length);
      if(FAU_lengthT > FAU_length)
        this->FAU = (float*)realloc(this->FAU, FAU_lengthT * sizeof(float));
      FAU_length = FAU_lengthT;
      for( uint32_t i = 0; i < FAU_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_FAU;
      u_st_FAU.base = 0;
      u_st_FAU.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_FAU.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_FAU.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_FAU.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_FAU = u_st_FAU.real;
      offset += sizeof(this->st_FAU);
        memcpy( &(this->FAU[i]), &(this->st_FAU), sizeof(float));
      }
      uint32_t intensity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      intensity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      intensity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      intensity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->intensity_length);
      if(intensity_lengthT > intensity_length)
        this->intensity = (float*)realloc(this->intensity, intensity_lengthT * sizeof(float));
      intensity_length = intensity_lengthT;
      for( uint32_t i = 0; i < intensity_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_intensity;
      u_st_intensity.base = 0;
      u_st_intensity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_intensity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_intensity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_intensity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_intensity = u_st_intensity.real;
      offset += sizeof(this->st_intensity);
        memcpy( &(this->intensity[i]), &(this->st_intensity), sizeof(float));
      }
      uint32_t confidence_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->confidence_length);
      if(confidence_lengthT > confidence_length)
        this->confidence = (float*)realloc(this->confidence, confidence_lengthT * sizeof(float));
      confidence_length = confidence_lengthT;
      for( uint32_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_confidence;
      u_st_confidence.base = 0;
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_confidence = u_st_confidence.real;
      offset += sizeof(this->st_confidence);
        memcpy( &(this->confidence[i]), &(this->st_confidence), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/FacialActionUnits"; };
    virtual const char * getMD5() override { return "7872c889685bdf665183ddf7f24a1dd9"; };

  };

}
#endif

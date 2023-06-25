#ifndef _ROS_hri_msgs_FacialLandmarks_h
#define _ROS_hri_msgs_FacialLandmarks_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "hri_msgs/NormalizedPointOfInterest2D.h"

namespace hri_msgs
{

  class FacialLandmarks : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t landmarks_length;
      typedef hri_msgs::NormalizedPointOfInterest2D _landmarks_type;
      _landmarks_type st_landmarks;
      _landmarks_type * landmarks;
      typedef uint32_t _height_type;
      _height_type height;
      typedef uint32_t _width_type;
      _width_type width;
      enum { RIGHT_EAR = 0 };
      enum { RIGHT_PROFILE_1 = 1 };
      enum { RIGHT_PROFILE_2 = 2 };
      enum { RIGHT_PROFILE_3 = 3 };
      enum { RIGHT_PROFILE_4 = 4 };
      enum { RIGHT_PROFILE_5 = 5 };
      enum { RIGHT_PROFILE_6 = 6 };
      enum { RIGHT_PROFILE_7 = 7 };
      enum { MENTON = 8 };
      enum { LEFT_EAR = 16 };
      enum { LEFT_PROFILE_1 = 15 };
      enum { LEFT_PROFILE_2 = 14 };
      enum { LEFT_PROFILE_3 = 13 };
      enum { LEFT_PROFILE_4 = 12 };
      enum { LEFT_PROFILE_5 = 11 };
      enum { LEFT_PROFILE_6 = 10 };
      enum { LEFT_PROFILE_7 = 9 };
      enum { RIGHT_EYEBROW_OUTSIDE = 17 };
      enum { RIGHT_EYEBROW_1 = 18 };
      enum { RIGHT_EYEBROW_2 = 19 };
      enum { RIGHT_EYEBROW_3 = 20 };
      enum { RIGHT_EYEBROW_INSIDE = 21 };
      enum { RIGHT_EYE_OUTSIDE = 36 };
      enum { RIGHT_EYE_TOP_1 = 37 };
      enum { RIGHT_EYE_TOP_2 = 38 };
      enum { RIGHT_EYE_INSIDE = 39 };
      enum { RIGHT_EYE_BOTTOM_1 = 41 };
      enum { RIGHT_EYE_BOTTOM_2 = 40 };
      enum { RIGHT_PUPIL = 68 };
      enum { LEFT_EYEBROW_OUTSIDE = 26 };
      enum { LEFT_EYEBROW_1 = 25 };
      enum { LEFT_EYEBROW_2 = 24 };
      enum { LEFT_EYEBROW_3 = 23 };
      enum { LEFT_EYEBROW_INSIDE = 22 };
      enum { LEFT_EYE_OUTSIDE = 45 };
      enum { LEFT_EYE_TOP_1 = 44 };
      enum { LEFT_EYE_TOP_2 = 43 };
      enum { LEFT_EYE_INSIDE = 42 };
      enum { LEFT_EYE_BOTTOM_1 = 46 };
      enum { LEFT_EYE_BOTTOM_2 = 47 };
      enum { LEFT_PUPIL = 69 };
      enum { SELLION = 27 };
      enum { NOSE_1 = 28 };
      enum { NOSE_2 = 29 };
      enum { NOSE = 30 };
      enum { NOSTRIL_1 = 31 };
      enum { NOSTRIL_2 = 32 };
      enum { NOSTRIL_3 = 33 };
      enum { NOSTRIL_4 = 34 };
      enum { NOSTRIL_5 = 35 };
      enum { MOUTH_OUTER_RIGHT = 48 };
      enum { MOUTH_OUTER_TOP_1 = 49 };
      enum { MOUTH_OUTER_TOP_2 = 50 };
      enum { MOUTH_OUTER_TOP_3 = 51 };
      enum { MOUTH_OUTER_TOP_4 = 52 };
      enum { MOUTH_OUTER_TOP_5 = 53 };
      enum { MOUTH_OUTER_LEFT = 54 };
      enum { MOUTH_OUTER_BOTTOM_1 = 59 };
      enum { MOUTH_OUTER_BOTTOM_2 = 58 };
      enum { MOUTH_OUTER_BOTTOM_3 = 57 };
      enum { MOUTH_OUTER_BOTTOM_4 = 56 };
      enum { MOUTH_OUTER_BOTTOM_5 = 55 };
      enum { MOUTH_INNER_RIGHT = 60 };
      enum { MOUTH_INNER_TOP_1 = 61 };
      enum { MOUTH_INNER_TOP_2 = 62 };
      enum { MOUTH_INNER_TOP_3 = 63 };
      enum { MOUTH_INNER_LEFT = 64 };
      enum { MOUTH_INNER_BOTTOM_1 = 67 };
      enum { MOUTH_INNER_BOTTOM_2 = 66 };
      enum { MOUTH_INNER_BOTTOM_3 = 65 };

    FacialLandmarks():
      header(),
      landmarks_length(0), st_landmarks(), landmarks(nullptr),
      height(0),
      width(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->landmarks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->landmarks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->landmarks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->landmarks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->landmarks_length);
      for( uint32_t i = 0; i < landmarks_length; i++){
      offset += this->landmarks[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t landmarks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      landmarks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->landmarks_length);
      if(landmarks_lengthT > landmarks_length)
        this->landmarks = (hri_msgs::NormalizedPointOfInterest2D*)realloc(this->landmarks, landmarks_lengthT * sizeof(hri_msgs::NormalizedPointOfInterest2D));
      landmarks_length = landmarks_lengthT;
      for( uint32_t i = 0; i < landmarks_length; i++){
      offset += this->st_landmarks.deserialize(inbuffer + offset);
        memcpy( &(this->landmarks[i]), &(this->st_landmarks), sizeof(hri_msgs::NormalizedPointOfInterest2D));
      }
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/FacialLandmarks"; };
    virtual const char * getMD5() override { return "c779d9fd02c1af8ab6a1712921ab9da5"; };

  };

}
#endif

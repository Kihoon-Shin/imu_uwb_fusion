#ifndef _ROS_hri_msgs_Skeleton2D_h
#define _ROS_hri_msgs_Skeleton2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "hri_msgs/NormalizedPointOfInterest2D.h"

namespace hri_msgs
{

  class Skeleton2D : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t skeleton_length;
      typedef hri_msgs::NormalizedPointOfInterest2D _skeleton_type;
      _skeleton_type st_skeleton;
      _skeleton_type * skeleton;
      enum { NOSE =  0 };
      enum { NECK =  1 };
      enum { RIGHT_SHOULDER =  2 };
      enum { RIGHT_ELBOW =  3 };
      enum { RIGHT_WRIST =  4 };
      enum { LEFT_SHOULDER =  5 };
      enum { LEFT_ELBOW =  6 };
      enum { LEFT_WRIST =  7 };
      enum { RIGHT_HIP =  8 };
      enum { RIGHT_KNEE =  9 };
      enum { RIGHT_ANKLE =  10 };
      enum { LEFT_HIP =  11 };
      enum { LEFT_KNEE =  12 };
      enum { LEFT_ANKLE =  13 };
      enum { LEFT_EYE =  14 };
      enum { RIGHT_EYE =  15 };
      enum { LEFT_EAR =  16 };
      enum { RIGHT_EAR =  17 };

    Skeleton2D():
      header(),
      skeleton_length(0), st_skeleton(), skeleton(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->skeleton_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->skeleton_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->skeleton_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->skeleton_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->skeleton_length);
      for( uint32_t i = 0; i < skeleton_length; i++){
      offset += this->skeleton[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t skeleton_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      skeleton_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->skeleton_length);
      if(skeleton_lengthT > skeleton_length)
        this->skeleton = (hri_msgs::NormalizedPointOfInterest2D*)realloc(this->skeleton, skeleton_lengthT * sizeof(hri_msgs::NormalizedPointOfInterest2D));
      skeleton_length = skeleton_lengthT;
      for( uint32_t i = 0; i < skeleton_length; i++){
      offset += this->st_skeleton.deserialize(inbuffer + offset);
        memcpy( &(this->skeleton[i]), &(this->st_skeleton), sizeof(hri_msgs::NormalizedPointOfInterest2D));
      }
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/Skeleton2D"; };
    virtual const char * getMD5() override { return "efedc2dc59671380a1d9b497f0740be4"; };

  };

}
#endif

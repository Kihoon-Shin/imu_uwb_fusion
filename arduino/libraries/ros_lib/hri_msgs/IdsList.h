#ifndef _ROS_hri_msgs_IdsList_h
#define _ROS_hri_msgs_IdsList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class IdsList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t ids_length;
      typedef char* _ids_type;
      _ids_type st_ids;
      _ids_type * ids;

    IdsList():
      header(),
      ids_length(0), st_ids(), ids(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids_length);
      for( uint32_t i = 0; i < ids_length; i++){
      uint32_t length_idsi = strlen(this->ids[i]);
      varToArr(outbuffer + offset, length_idsi);
      offset += 4;
      memcpy(outbuffer + offset, this->ids[i], length_idsi);
      offset += length_idsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ids_length);
      if(ids_lengthT > ids_length)
        this->ids = (char**)realloc(this->ids, ids_lengthT * sizeof(char*));
      ids_length = ids_lengthT;
      for( uint32_t i = 0; i < ids_length; i++){
      uint32_t length_st_ids;
      arrToVar(length_st_ids, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_ids; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_ids-1]=0;
      this->st_ids = (char *)(inbuffer + offset-1);
      offset += length_st_ids;
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/IdsList"; };
    virtual const char * getMD5() override { return "84a63f55b5676f78b625e8a8bb809fe5"; };

  };

}
#endif

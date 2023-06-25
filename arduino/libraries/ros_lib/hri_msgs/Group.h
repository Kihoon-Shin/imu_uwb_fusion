#ifndef _ROS_hri_msgs_Group_h
#define _ROS_hri_msgs_Group_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class Group : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _group_id_type;
      _group_id_type group_id;
      uint32_t members_length;
      typedef char* _members_type;
      _members_type st_members;
      _members_type * members;

    Group():
      header(),
      group_id(""),
      members_length(0), st_members(), members(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_group_id = strlen(this->group_id);
      varToArr(outbuffer + offset, length_group_id);
      offset += 4;
      memcpy(outbuffer + offset, this->group_id, length_group_id);
      offset += length_group_id;
      *(outbuffer + offset + 0) = (this->members_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->members_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->members_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->members_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->members_length);
      for( uint32_t i = 0; i < members_length; i++){
      uint32_t length_membersi = strlen(this->members[i]);
      varToArr(outbuffer + offset, length_membersi);
      offset += 4;
      memcpy(outbuffer + offset, this->members[i], length_membersi);
      offset += length_membersi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_group_id;
      arrToVar(length_group_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group_id-1]=0;
      this->group_id = (char *)(inbuffer + offset-1);
      offset += length_group_id;
      uint32_t members_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      members_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      members_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      members_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->members_length);
      if(members_lengthT > members_length)
        this->members = (char**)realloc(this->members, members_lengthT * sizeof(char*));
      members_length = members_lengthT;
      for( uint32_t i = 0; i < members_length; i++){
      uint32_t length_st_members;
      arrToVar(length_st_members, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_members; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_members-1]=0;
      this->st_members = (char *)(inbuffer + offset-1);
      offset += length_st_members;
        memcpy( &(this->members[i]), &(this->st_members), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/Group"; };
    virtual const char * getMD5() override { return "76be953e6ddf78f60b879c220d0a3c32"; };

  };

}
#endif

#ifndef _ROS_basestation_gui_python_RadioMsg_h
#define _ROS_basestation_gui_python_RadioMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace basestation_gui_python
{

  class RadioMsg : public ros::Msg
  {
    public:
      typedef uint8_t _message_type_type;
      _message_type_type message_type;
      typedef uint8_t _recipient_robot_id_type;
      _recipient_robot_id_type recipient_robot_id;
      typedef const char* _data_type;
      _data_type data;
      enum { MESSAGE_TYPE_ESTOP = 1 };
      enum { ESTOP_RESUME = 0 };
      enum { ESTOP_PAUSE = 1 };
      enum { ESTOP_SOFT = 2 };
      enum { ESTOP_HARD = 3 };

    RadioMsg():
      message_type(0),
      recipient_robot_id(0),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->message_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->message_type);
      *(outbuffer + offset + 0) = (this->recipient_robot_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->recipient_robot_id);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->message_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->message_type);
      this->recipient_robot_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->recipient_robot_id);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    const char * getType(){ return "basestation_gui_python/RadioMsg"; };
    const char * getMD5(){ return "811b7d3397a5bf455c94ff49dd59699b"; };

  };

}
#endif

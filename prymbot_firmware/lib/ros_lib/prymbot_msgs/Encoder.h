#ifndef _ROS_prymbot_msgs_Encoder_h
#define _ROS_prymbot_msgs_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace prymbot_msgs
{

  class Encoder : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int16_t _left_ticks_type;
      _left_ticks_type left_ticks;
      typedef int16_t _right_ticks_type;
      _right_ticks_type right_ticks;
      typedef float _left_speed_type;
      _left_speed_type left_speed;
      typedef float _right_speed_type;
      _right_speed_type right_speed;
      typedef float _left_ang_vel_type;
      _left_ang_vel_type left_ang_vel;
      typedef float _right_ang_vel_type;
      _right_ang_vel_type right_ang_vel;

    Encoder():
      header(),
      left_ticks(0),
      right_ticks(0),
      left_speed(0),
      right_speed(0),
      left_ang_vel(0),
      right_ang_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_left_ticks;
      u_left_ticks.real = this->left_ticks;
      *(outbuffer + offset + 0) = (u_left_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_ticks.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_ticks);
      union {
        int16_t real;
        uint16_t base;
      } u_right_ticks;
      u_right_ticks.real = this->right_ticks;
      *(outbuffer + offset + 0) = (u_right_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_ticks.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_ticks);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_ang_vel);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_ang_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_left_ticks;
      u_left_ticks.base = 0;
      u_left_ticks.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_ticks.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_ticks = u_left_ticks.real;
      offset += sizeof(this->left_ticks);
      union {
        int16_t real;
        uint16_t base;
      } u_right_ticks;
      u_right_ticks.base = 0;
      u_right_ticks.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_ticks.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_ticks = u_right_ticks.real;
      offset += sizeof(this->right_ticks);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_ang_vel));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_ang_vel));
     return offset;
    }

    virtual const char * getType() override { return "prymbot_msgs/Encoder"; };
    virtual const char * getMD5() override { return "ac723460252648cf185dce68af006a8f"; };

  };

}
#endif

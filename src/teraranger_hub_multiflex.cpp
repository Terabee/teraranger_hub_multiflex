/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name Teraranger_hub nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <string>

#include "teraranger_hub_multiflex/teraranger_hub_multiflex.h"

#include <ros/console.h>

namespace teraranger_hub_multiflex
{

Teraranger_hub_multiflex::Teraranger_hub_multiflex()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyACM0"));

  // Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_hub_multiflex", 8);

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&Teraranger_hub_multiflex::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
  {
    ros::shutdown();
    return;
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  std::string str = ns_.c_str();
  ROS_INFO("node namespace: [%s]", ns_.c_str());


  // Set operation Mode
 setMode(BINARY_MODE);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&Teraranger_hub_multiflex::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

Teraranger_hub_multiflex::~Teraranger_hub_multiflex()
{
}

uint8_t Teraranger_hub_multiflex::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void Teraranger_hub_multiflex::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  sensor_msgs::Range range_msg0;
  range_msg0.field_of_view = 0.0593;
  range_msg0.max_range = 3.0;
  range_msg0.min_range = 0.06;
  range_msg0.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg0.header.frame_id = ros::names::append(ns_, std::string("base_range_0"));

  sensor_msgs::Range range_msg1;
  range_msg1.field_of_view = 0.0593;
  range_msg1.max_range = 3.0;
  range_msg1.min_range = 0.06;
  range_msg1.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg1.header.frame_id = ros::names::append(ns_, std::string("base_range_1"));

  sensor_msgs::Range range_msg2;
  range_msg2.field_of_view = 0.0593;
  range_msg2.max_range = 3.0;
  range_msg2.min_range = 0.06;
  range_msg2.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg2.header.frame_id= ros::names::append(ns_, std::string("base_range_2"));

  sensor_msgs::Range range_msg3;
  range_msg3.field_of_view = 0.0593;
  range_msg3.max_range = 3.0;
  range_msg3.min_range = 0.06;
  range_msg3.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg3.header.frame_id= ros::names::append(ns_, std::string("base_range_3"));

  sensor_msgs::Range range_msg4;
  range_msg4.field_of_view = 0.0593;
  range_msg4.max_range = 3.0;
  range_msg4.min_range = 0.06;
  range_msg4.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg4.header.frame_id= ros::names::append(ns_, std::string("base_range_4"));

  sensor_msgs::Range range_msg5;
  range_msg5.field_of_view = 0.0593;
  range_msg5.max_range = 3.0;
  range_msg5.min_range = 0.06;
  range_msg5.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg5.header.frame_id= ros::names::append(ns_, std::string("base_range_5"));

  sensor_msgs::Range range_msg6;
  range_msg6.field_of_view = 0.0593;
  range_msg6.max_range = 3.0;
  range_msg6.min_range = 0.06;
  range_msg6.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg6.header.frame_id= ros::names::append(ns_, std::string("base_range_6"));

  sensor_msgs::Range range_msg7;
  range_msg7.field_of_view = 0.0593;
  range_msg7.max_range = 3.0;
  range_msg7.min_range = 0.06;
  range_msg7.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg7.header.frame_id= ros::names::append(ns_, std::string("base_range_7"));


 if (single_character != 'M' && buffer_ctr < 19)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }


  else if (single_character == 'M')
  {

    if (buffer_ctr == 19)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 18);

      if (crc == input_buffer[18])
      {
        int16_t range0 = input_buffer[2] << 8;
        range0 |= input_buffer[3];
        int16_t range1 = input_buffer[4] << 8;
        range1 |= input_buffer[5];
        int16_t range2 = input_buffer[6] << 8;
        range2 |= input_buffer[7];
        int16_t range3 = input_buffer[8] << 8;
        range3 |= input_buffer[9];
        int16_t range4 = input_buffer[10] << 8;
        range4 |= input_buffer[11];
        int16_t range5 = input_buffer[12] << 8;
        range5 |= input_buffer[13];
        int16_t range6 = input_buffer[14] << 8;
        range6 |= input_buffer[15];
        int16_t range7 = input_buffer[16] << 8;
        range7 |= input_buffer[17];

        if (range0 < 3000 && range0 > 60)
        {
          range_msg0.header.stamp = ros::Time::now();
          range_msg0.header.seq = seq_ctr++;
          range_msg0.range = range0 * 0.001; // convert to m
	  range_publisher_.publish(range_msg0);
        }
	else{

          range_msg0.header.stamp = ros::Time::now();
          range_msg0.header.seq = seq_ctr++;
          range_msg0.range = 0.0 ;
	  range_publisher_.publish(range_msg0);
}
        if (range1 < 3000 && range1 > 60)
        {
          range_msg1.header.stamp = ros::Time::now();
          range_msg1.header.seq = seq_ctr++;
          range_msg1.range = range1 * 0.001;
          range_publisher_.publish(range_msg1);
	}
        else{

          range_msg1.header.stamp = ros::Time::now();
          range_msg1.header.seq = seq_ctr++;
          range_msg1.range = 0.0 ;
	  range_publisher_.publish(range_msg1);
}
        if (range2 < 3000 && range2 > 60)
        {
          range_msg2.header.stamp = ros::Time::now();
          range_msg2.header.seq = seq_ctr++;
          range_msg2.range = range2 * 0.001;
          range_publisher_.publish(range_msg2);
	}
	else{

          range_msg2.header.stamp = ros::Time::now();
          range_msg2.header.seq = seq_ctr++;
          range_msg2.range = 0.0 ;
          range_publisher_.publish(range_msg2);
}
        if (range3 < 3000 && range3 > 60)
        {
          range_msg3.header.stamp = ros::Time::now();
          range_msg3.header.seq = seq_ctr++;
          range_msg3.range = range3 * 0.001;
          range_publisher_.publish(range_msg3);
	}
	else{

          range_msg3.header.stamp = ros::Time::now();
          range_msg3.header.seq = seq_ctr++;
          range_msg3.range = 0.0 ;
          range_publisher_.publish(range_msg3);
}
        if (range4 < 3000 && range4 > 60)
        {
          range_msg4.header.stamp = ros::Time::now();
          range_msg4.header.seq = seq_ctr++;
          range_msg4.range = range4 * 0.001;
          range_publisher_.publish(range_msg4);
	}
	else{

          range_msg4.header.stamp = ros::Time::now();
          range_msg4.header.seq = seq_ctr++;
          range_msg4.range = 0.0;
          range_publisher_.publish(range_msg4);
}
        if (range5 < 3000 && range5 > 60)
        {
          range_msg5.header.stamp = ros::Time::now();
          range_msg5.header.seq = seq_ctr++;
          range_msg5.range = range5 * 0.001;
          range_publisher_.publish(range_msg5);
	}
	else{

          range_msg5.header.stamp = ros::Time::now();
          range_msg5.header.seq = seq_ctr++;
          range_msg5.range = 0.0;
          range_publisher_.publish(range_msg5);
}
        if (range6 < 3000 && range6 > 60)
        {
          range_msg6.header.stamp = ros::Time::now();
          range_msg6.header.seq = seq_ctr++;
          range_msg6.range = range6 * 0.001;
          range_publisher_.publish(range_msg6);
	}
	else{

          range_msg6.header.stamp = ros::Time::now();
          range_msg6.header.seq = seq_ctr++;
          range_msg6.range = 0.0;
          range_publisher_.publish(range_msg6);
}
        if (range7 < 3000 && range7 > 60)
        {
          range_msg7.header.stamp = ros::Time::now();
          range_msg7.header.seq = seq_ctr++;
          range_msg7.range = range7 * 0.001;
          range_publisher_.publish(range_msg7);
	}
	else{

          range_msg7.header.stamp = ros::Time::now();
          range_msg7.header.seq = seq_ctr++;
          range_msg7.range = 0.0;
          range_publisher_.publish(range_msg7);
	  }



      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] reveived M but did not expect it, reset buffer without evaluating data",
               ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving M, reset input_buffer", ros::this_node::getName().c_str());
  }

  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'M';
}  //


//set mode original


void Teraranger_hub_multiflex::setMode(const char *c)
{
 serial_port_->sendChar(c);
}

void Teraranger_hub_multiflex::dynParamCallback(const teraranger_hub_multiflex::Teraranger_hub_multiflexConfig &config, uint32_t level)
{
  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_Fast)
  {
    setMode(FAST_MODE);
	ROS_INFO("Fast mode set");
  }

  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_Precise)
  {
    setMode(PRECISE_MODE);
	ROS_INFO("Precise mode set");
  }

  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_LongRange)
  {
    setMode(LONG_RANGE_MODE);
	ROS_INFO("Long range mode set");
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Teraranger_hub_multiflex");
  teraranger_hub_multiflex::Teraranger_hub_multiflex tera_bee;
  ros::spin();

  return 0;
}

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
#include <sstream>

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

  // Initialize all active sensors
  for(int i=0; i<8; i++)
  {
  	sensor_bit_mask[i] = 1;
  }

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
  static float min_range = 0.05;
  static float max_range = 2.0;
  static float field_of_view = 0.0593;
  static int int_min_range = (int)min_range*1000;
  static int int_max_range = (int)max_range*1000;


  sensor_msgs::Range sensors[8];
  for(int i=0; i<8; i++)
  	{
		sensors[i].field_of_view = field_of_view;
		sensors[i].max_range = max_range;
		sensors[i].min_range = min_range;
		sensors[i].radiation_type = sensor_msgs::Range::INFRARED;

		std::string frame="base_range_";
		std::string frame_id = frame + IntToString(i);
		sensors[i].header.frame_id = ros::names::append(ns_, frame_id);
	}

 if (single_character != 'M' && buffer_ctr < 20)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }


  else if (single_character == 'M')
  {

    if (buffer_ctr == 20)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 19);

      if (crc == input_buffer[19])
      {

		for(int i=0;i<20;i++)
		{
			ROS_DEBUG("%d %x \t",i,input_buffer[i]);
		}

		int16_t ranges[8];
		for(int i=0; i<8; i++)
		{
			ranges[i] = input_buffer[i*2 + 2] << 8;
	        ranges[i] |= input_buffer[i*2 + 3];
		}

		uint8_t bitmask = input_buffer[18];
		ROS_DEBUG("bit_mask %d", bitmask);
		uint8_t bit_compare = 1;

		for(int i=0; i<8; i++)
		{
			if ((bitmask & bit_compare) == bit_compare)
			{
				if (ranges[i] < int_max_range && ranges[i] > int_min_range)
				{
				sensors[i].header.stamp = ros::Time::now();
				sensors[i].header.seq = seq_ctr++;
				sensors[i].range = ranges[i] * 0.001; // convert to m
				range_publisher_.publish(sensors[i]);
				}
				else
				{
					sensors[i].header.stamp = ros::Time::now();
					sensors[i].header.seq = seq_ctr++;
					sensors[i].range = ranges[i];
				  	range_publisher_.publish(sensors[i]);
				}
			}
			else
			{
				ROS_WARN_ONCE("Not all sensors activated set proper bitmask using rosrun rqt_reconfigure rqt_reconfigure");
			}
			bit_compare <<= 1;
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
 serial_port_->sendChar(c, 4);
}

void Teraranger_hub_multiflex::setSensorBitMask(int *sensor_bit_mask_ptr)
{

 char bit_mask_hex = 0x00;
 for (int i=0; i<8; i++)
 {
	 bit_mask_hex |= *(sensor_bit_mask_ptr +7-i) << (7-i);
 }

 // calculate crc
 uint8_t command[4] = {0x00, 0x52, 0x03, bit_mask_hex};
 int8_t crc = crc8(command, 4);

 //send command
 char full_command[5] = {0x00, 0x52, 0x03, bit_mask_hex, crc};
 serial_port_->sendChar(full_command, 5);
}

void Teraranger_hub_multiflex::dynParamCallback(const teraranger_hub_multiflex::Teraranger_hub_multiflexConfig &config, uint32_t level)
{

  sensor_bit_mask[0] = config.Sensor_0?1:0;
  sensor_bit_mask[1] = config.Sensor_1?1:0;
  sensor_bit_mask[2] = config.Sensor_2?1:0;
  sensor_bit_mask[3] = config.Sensor_3?1:0;
  sensor_bit_mask[4] = config.Sensor_4?1:0;
  sensor_bit_mask[5] = config.Sensor_5?1:0;
  sensor_bit_mask[6] = config.Sensor_6?1:0;
  sensor_bit_mask[7] = config.Sensor_7?1:0;

  sensor_bit_mask_ptr = sensor_bit_mask;

  setSensorBitMask(sensor_bit_mask_ptr);

  for(int i = 0; i<8; i++)
  {
	  ROS_INFO("Sensor %d is set to %d",i,sensor_bit_mask[i]);
  }

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

std::string Teraranger_hub_multiflex::IntToString( int number )
{
	std::ostringstream oss;
	oss << number;
	return oss.str();
}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Teraranger_hub_multiflex");
  teraranger_hub_multiflex::Teraranger_hub_multiflex tera_bee;
  ros::spin();

  return 0;
}

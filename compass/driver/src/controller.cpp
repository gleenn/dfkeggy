/**
Software License Agreement (BSD)

\file      controller.cpp

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "dfcompass_driver/controller.h"

#include "dfcompass_msgs/status.h"
#include "serial/serial.h"

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <stdlib.h>
#include <unistd.h>

namespace dfcompass {

//const std::string eol("\n");
//const size_t max_line_length(1024);

Controller::Controller(const char *port, int baud)
  : nh_("~"), port_(port), baud_(baud), connected_(false),
    serial_(new serial::Serial()) {
  pub_status_ = nh_.advertise<dfcompass_msgs::status>("status", 1);

  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  serial_->setTimeout(to);
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);
  serial_->setParity(serial::parity_none);
  serial_->setStopbits(serial::stopbits_one);
  serial_->setBytesize(serial::eightbits);
  serial_->setFlowcontrol(serial::flowcontrol_software);
}

Controller::~Controller() {}

void Controller::connect() {
  for (int tries = 0; tries < 5; tries++) {
    try {
      serial_->open();
    } catch (serial::IOException) {
    }

    if (serial_->isOpen()) {
      connected_ = true;
      return;
    } else {
      connected_ = false;
      ROS_INFO("Bad Connection with serial port Error %s",port_);
    }
  }

  ROS_INFO("Compass controller not responding.");
}

void Controller::read() {
  while (serial_->available()) {
    std::string str = serial_->read();
    if (str.empty()) {
      ROS_WARN_NAMED("serial", "Serial::read() returned no data.");
      return;
    }
    if (str[0] == 0) {
      // ROS_WARN_NAMED("serial", "Serial::read() returned zero char.");
      continue;
    }
    if (str[0] == '\r' || str[0] == '\n') {
      if (!buffer_.empty()) {
	processStatus(buffer_);
      }
      buffer_ = "";
      continue;
    }
    buffer_ += str;
  }
}

void Controller::processStatus(const std::string& msg) {
  // Example message: "H:  354 R:   -2 P:    6 Calib: 030"
  ROS_DEBUG_STREAM_NAMED("serial", "Compass RX: " << msg);
  char last_marker = 0;
  std::string header_str;
  std::vector<std::string> strs;
  boost::split(strs, msg, boost::is_any_of(" "));
  for (size_t i = 0; i < strs.size(); ++i) {
    const std::string& str = strs[i];
    if (str.empty()) continue;
    if (str == "H:" || str == "R:" || str == "P:" || str == "Calib:") {
      last_marker = str[0];
      continue;
    }
    if (last_marker == 'H') {
      header_str = str;
      last_marker = 0;
      continue;
    }
  }

  if (header_str.empty()) {
    ROS_WARN_STREAM_NAMED("serial", "No heading in RX data: '" << msg << "'");
    return;
  }

  int heading = strtol(header_str.c_str(), NULL, 10);
  if (heading == LONG_MIN || heading == LONG_MAX) {
    ROS_WARN_STREAM_NAMED(
	"serial", "Incorrect heading in RX data: '" << msg << "'");
    return;
  }

  if (heading == 0) {
    ROS_WARN_STREAM_NAMED("serial", "Compass RX: " << msg);
  }

  dfcompass_msgs::status status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.heading_degrees = heading;
  pub_status_.publish(status_msg);
}

}  // namespace dfcompass

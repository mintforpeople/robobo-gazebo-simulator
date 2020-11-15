/*This work has been funded by rosin.org (contract agreement 732287) 
 through EP project "Robobo AI"*/

/*******************************************************************************
 *
 *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L.
 *   <http://www.mintforpeople.com>
 *
 *   Redistribution, modification and use of this software are permitted under
 *   terms of the Apache 2.0 License.
 *
 *   This software is distributed in the hope that it will be useful,
 *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   Apache 2.0 License for more details.
 *
 *   You should have received a copy of the Apache 2.0 License along with
 *   this software. If not, see <http://www.apache.org/licenses/>.
 *
 ******************************************************************************/

/* Derived from the work :
/* MIT License

 *Copyright (c) 2018 SMART Lab at Purdue University

 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:

 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.

 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.*/

#ifndef GAZEBO_ROS_LIGHT_SENSOR_HH
#define GAZEBO_ROS_LIGHT_SENSOR_HH
   
#include <string>
    
// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
    
#include <gazebo_plugins/gazebo_ros_camera_utils.h>


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "gazebo_msgs/LinkStates.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
    
namespace gazebo
{
  class GazeboRosLight : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosLight();
 
    /// \brief Destructor
    public: ~GazeboRosLight();
  
    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);
   
    //brief topic name
    std::string topic_name_;

    //brief for setting ROS name space
    std::string robot_namespace_;

    //std::unique_ptr<ros::NodeHandle> _nh;
    ros::NodeHandle* _nh;
    ros::Publisher _sensorPublisher;
    
    double _fov;
    double _range;
  };
}
#endif

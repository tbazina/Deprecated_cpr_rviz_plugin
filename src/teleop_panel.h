/*
* Copyright (c) 2014, Commonplace Robotics GmbH
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

class QLabel;
class QLineEdit;
class QPushButton;
namespace cpr_rviz_plugin
{


class TeleopPanel: public rviz::Panel
{
	Q_OBJECT
public:
	TeleopPanel( QWidget* parent = 0 );

public Q_SLOTS:
    void setJogVel();
    void initGUI();
    void initROS();

protected Q_SLOTS:
    void sendVel();
    void btPressedConnect();
    void btPressedReset();
    void btPressedEnable();
    void btPressedGripperOpen();
    void btPressedGripperClose();
    void btPressedOverrideMinus();
    void btPressedOverridePlus();
    void sendCommand();


protected:
	

    bool flagMover4;	// which robot to operate?
    bool flagMover6;

    int override_value;

    QTimer* output_timer;

    QLabel* labelOverride;
    QLabel* labelStatus;

    QLabel* labelJ0;
    QLabel* labelJ1;
    QLabel* labelJ2;
    QLabel* labelJ3;
    QLabel* labelJ4;
    QLabel* labelJ5;


    QPushButton* buttonConnect;
    QPushButton* buttonReset;
    QPushButton* buttonEnable;

    QPushButton* buttonOverridePlus;
    QPushButton* buttonOverrideMinus;

    QPushButton* buttonGripperOpen;
    QPushButton* buttonGripperClose;

    QPushButton* buttonJog0Plus;
    QPushButton* buttonJog0Minus;
    QPushButton* buttonJog1Plus;
    QPushButton* buttonJog1Minus;
    QPushButton* buttonJog2Plus;
    QPushButton* buttonJog2Minus;
    QPushButton* buttonJog3Plus;
    QPushButton* buttonJog3Minus;

    QPushButton* buttonJog4Plus;
    QPushButton* buttonJog4Minus;
    QPushButton* buttonJog5Plus;
    QPushButton* buttonJog5Minus;


    // The ROS publisher/subscriber
    ros::Publisher velocity_publisher_;
    ros::Publisher commands_publisher_;
    sensor_msgs::JointState velMsg;
    ros::Subscriber subErrorState_;
    ros::Subscriber subJointState_;
    void errorStateCallback(const std_msgs::String::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // The ROS node handle.
    ros::NodeHandle nh_;
	

    float jointVelocities[6];
    QString lastError;

	
};
} // end namespace rviz_plugin_tutorials
#endif // TELEOP_PANEL_H


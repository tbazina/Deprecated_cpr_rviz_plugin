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

// based on the RViz teleop panel tutorial.

// First version: November 1st, 2014
// Current versin: December 1st, 2014
// 
// RViz plugin to operate the Mover4 or Mover6 robot arms
// Allows to push commands like connect or enable, and to jog the joints
// Displays the joint values


#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QString>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "teleop_panel.h"


namespace cpr_rviz_plugin
{


TeleopPanel::TeleopPanel( QWidget* parent )
: rviz::Panel( parent )
, linear_velocity_( 0 )
, angular_velocity_( 0 )
{

    QHBoxLayout* hboxStatus = new QHBoxLayout;              // Status fields on top
    hboxStatus->addWidget( new QLabel( "Status: " ));
    labelStatus = new QLabel;
    labelStatus->setText("not connected");
    hboxStatus->addWidget(labelStatus);


    buttonConnect = new QPushButton;                        // buttons to connect. ..
    buttonConnect -> setText("Connect");
    buttonReset = new QPushButton;
    buttonReset -> setText("Reset");
    buttonEnable = new QPushButton;
    buttonEnable -> setText("Enable");

    buttonGripper = new QPushButton;
    buttonGripper -> setText("Gripper O/C");

    buttonJog0Plus = new QPushButton;                        // Jog Buttons
    buttonJog0Plus -> setText("J1 Plus");
    buttonJog0Minus = new QPushButton;
    buttonJog0Minus -> setText("J1 Minus");
    labelJ0 = new QLabel;
    labelJ0 -> setText("0.0");

    buttonJog1Plus = new QPushButton;
    buttonJog1Plus -> setText("J2 Plus");
    buttonJog1Minus = new QPushButton;
    buttonJog1Minus -> setText("J2 Minus");
    labelJ1 = new QLabel;
    labelJ1 -> setText("0.0");

    buttonJog2Plus = new QPushButton;
    buttonJog2Plus -> setText("J3 Plus");
    buttonJog2Minus = new QPushButton;
    buttonJog2Minus -> setText("J3 Minus");
    labelJ2 = new QLabel;
    labelJ2 -> setText("0.0");

    buttonJog3Plus = new QPushButton;
    buttonJog3Plus -> setText("J4 Plus");
    buttonJog3Minus = new QPushButton;
    buttonJog3Minus -> setText("J4 Minus");
    labelJ3 = new QLabel;
    labelJ3 -> setText("0.0");


    QVBoxLayout* layout = new QVBoxLayout;                  // the main container

    QHBoxLayout * hbox1 = new QHBoxLayout;                  // Compile a simple layout
    hbox1->addWidget(buttonConnect);
    hbox1->addWidget(buttonReset);
    hbox1->addWidget(buttonEnable);

    layout->addLayout(hboxStatus);
    layout->addLayout(hbox1);
    layout->addWidget( buttonGripper );

    QHBoxLayout * hboxJ0 = new QHBoxLayout;
    hboxJ0->addWidget(buttonJog0Minus);
    hboxJ0->addWidget(labelJ0);
    hboxJ0->addWidget(buttonJog0Plus);
    layout->addLayout(hboxJ0);

    QHBoxLayout * hboxJ1 = new QHBoxLayout;
    hboxJ1->addWidget(buttonJog1Minus);
    hboxJ1->addWidget(labelJ1);
    hboxJ1->addWidget(buttonJog1Plus);
    layout->addLayout(hboxJ1);

    QHBoxLayout * hboxJ2 = new QHBoxLayout;
    hboxJ2->addWidget(buttonJog2Minus);
    hboxJ2->addWidget(labelJ2);
    hboxJ2->addWidget(buttonJog2Plus);
    layout->addLayout(hboxJ2);

    QHBoxLayout * hboxJ3 = new QHBoxLayout;
    hboxJ3->addWidget(buttonJog3Minus);
    hboxJ3->addWidget(labelJ3);
    hboxJ3->addWidget(buttonJog3Plus);
    layout->addLayout(hboxJ3);


    setLayout( layout );


    // Create a timer for sending the output.
    QTimer* output_timer = new QTimer( this );

    connect( buttonConnect, SIGNAL( clicked() ), this, SLOT( btPressedConnect() ));
    connect( buttonReset, SIGNAL( clicked() ), this, SLOT( btPressedReset() ));
    connect( buttonEnable, SIGNAL( clicked() ), this, SLOT( btPressedEnable() ));
    connect( buttonGripper, SIGNAL( clicked() ), this, SLOT( btPressedGripper() ));

    connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
    // Start the timer.

    init();

    output_timer->start( 100 );


}




//********************************************************
void TeleopPanel::init()
{
    gripperState = false;

    velocity_publisher_ = nh_.advertise<sensor_msgs::JointState>( "CPRMoverJointVel", 1 );

    velMsg.name.resize(6);      // prepare the Message
    velMsg.velocity.resize(6);
    velMsg.position.resize(6);

    commands_publisher_ = nh_.advertise<std_msgs::String>("CPRMoverCommands", 1);


    subErrorState_ = nh_.subscribe<std_msgs::String>("/CPRMoverErrorCodes", 1, &TeleopPanel::errorStateCallback, this);
    subJointState_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &TeleopPanel::jointStateCallback, this);

}


//*************************************************************************************
// receive joint velocity commands
void TeleopPanel::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    float r2d = 180.0 / 3.141;
    labelJ0->setText(QString::number( (int)(r2d * msg->position[0]) ));
    labelJ1->setText(QString::number( (int)(r2d * msg->position[1]) ));
    labelJ2->setText(QString::number( (int)(r2d * msg->position[2]) ));
    labelJ3->setText(QString::number( (int)(r2d * msg->position[3]) ));
}


//*************************************************************************************
// Here we receive the discrete commands like Connect, Reset, Enable
// the commands are forwarded to the interface class
void TeleopPanel::errorStateCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("ErrorState: %s ", msg->data.c_str()) ;
    QString rec = msg->data.c_str();
    labelStatus->setText(rec);
}

//********************************************************
void TeleopPanel::btPressedConnect(){
    std_msgs::String msgCommands;
    msgCommands.data = "Connect";
    commands_publisher_.publish(msgCommands);
}

//********************************************************
void TeleopPanel::btPressedReset(){
    std_msgs::String msgCommands;
    msgCommands.data = "Reset";
    commands_publisher_.publish(msgCommands);

}

//********************************************************
void TeleopPanel::btPressedEnable(){
    std_msgs::String msgCommands;
    msgCommands.data = "Enable";
    commands_publisher_.publish(msgCommands);
}
//********************************************************
void TeleopPanel::btPressedGripper(){
    std_msgs::String msgCommands;

    if(gripperState == false){
        gripperState = true;
        msgCommands.data = "GripperOpen";
    }else{
        gripperState = false;
        msgCommands.data = "GripperClose";
    }
    commands_publisher_.publish(msgCommands);
}



//********************************************************
// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void TeleopPanel::sendVel()
{
    if( ros::ok() && velocity_publisher_ )
    {
        velMsg.header.stamp = ros::Time::now();

        if(buttonJog0Plus->isDown()) jointVelocities[0] = 50.0;
        else if(buttonJog0Minus->isDown()) jointVelocities[0] = -50.0;
        else jointVelocities[0] = 0.0;
        if(buttonJog1Plus->isDown()) jointVelocities[1] = 50.0;
        else if(buttonJog1Minus->isDown()) jointVelocities[1] = -50.0;
        else jointVelocities[1] = 0.0;
        if(buttonJog2Plus->isDown()) jointVelocities[2] = 50.0;
        else if(buttonJog2Minus->isDown()) jointVelocities[2] = -50.0;
        else jointVelocities[2] = 0.0;
        if(buttonJog3Plus->isDown()) jointVelocities[3] = 50.0;
        else if(buttonJog3Minus->isDown()) jointVelocities[3] = -50.0;
        else jointVelocities[3] = 0.0;

        for(int i=0; i<6; i++)
            velMsg.velocity[i] = jointVelocities[i];
        velocity_publisher_.publish( velMsg );
    }
}




} // end namespace

// Tell pluginlib about this class. 
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cpr_rviz_plugin::TeleopPanel,rviz::Panel )
// END_TUTORIAL

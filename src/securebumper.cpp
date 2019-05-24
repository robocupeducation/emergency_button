/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Daniel Arévalo Rodrigo   */

/* Mantainer: Daniel Arévalo Rodrigo sirl.daniel.arevalo@hotmail.com */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <semaphore.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <ros/ros.h>
#include <string>
#include "kobuki_msgs/BumperEvent.h"
#include <iostream>
#include <fstream>
using namespace std;

namespace button
{
class Emergency
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber bumperSubscriber;
        int events = 0;
    public:
        Emergency(): nh_("~"){
            bumperSubscriber = nh_.subscribe("/mobile_base/events/bumper", 1, &Emergency::checkEvent, this);
        }

        void checkEvent(const kobuki_msgs::BumperEvent::ConstPtr& msg ){
            events ++;
            printf("Events detected: %i\n", events );
            if(events >= 1){
                printf("[Emergency] Shutting down\n");
                killer();
            }
        }

        void killer(){
            system("ps aux | grep -i 'rosmaster' | awk '{print  $2}'>> pids");
            ifstream ficheroEntrada;
            string pid;
            ficheroEntrada.open ("pids");
            while(ficheroEntrada.peek()!=EOF){
                getline(ficheroEntrada, pid);
                string command = "kill -9 " + pid;
                printf("Command : %s\n",command.c_str() );
                system(command.c_str());
                printf("[Emergency]The proccess %s has been killed\n",pid.c_str() );
            }
            ficheroEntrada.close();
        }
};
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "emergencyBumper");

    button::Emergency watcher;
    ros::Rate loop_rate(2);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

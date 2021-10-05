#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include "math.h"

ros::Publisher velocity_pub;
geometry_msgs::Twist VelMsg;

float forceModule = 0;
float forceAngle = 0;
bool nearImpact = false;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& RecVelMsg){
  //Per evitare di ricevere, elaborare e pubblicare dei topic creati dal nodo stesso, si effettuano dei controlli tramite il VelMsg
  if(RecVelMsg->linear.x != 0 && (RecVelMsg->linear.x != VelMsg.linear.x || RecVelMsg->angular.z != VelMsg.angular.z)){

    //Il modulo generato è in genere più grande della velocità data in input, quindi lo ridimensiono di conseguenza e prendo la componente che incide sul movimento lineare
    forceModule *= cos(forceAngle)*RecVelMsg->linear.x/1000;

    //Lo scanner non da informazioni su cosa si trova esattamente dietro il robot, quindi si prova ad emarginare il più possibile una collisione sul retro
    if(RecVelMsg->linear.x < 0)
      forceModule *= -10;

    //Per non deviare la traiettoria, si va a modificare solo la componente lineare
    VelMsg.linear.x = RecVelMsg->linear.x + forceModule;  
    VelMsg.angular.z = RecVelMsg->angular.z;

    if(VelMsg.linear.x < 0) VelMsg.linear.x /= 10;
    //Se il robot si trova in prossimità di un ostacolo, si modifica la traiettoria per evitarlo
    else if(nearImpact) {
      VelMsg.angular.z += forceAngle;
      nearImpact = false;
      ROS_INFO("Near impact!");
    }

    ROS_INFO("Input: X=%f Z=%f \nDeflection: X=%f Z=%f",RecVelMsg->linear.x ,RecVelMsg->angular.z ,forceModule, forceAngle);

    velocity_pub.publish(VelMsg);

  } else {
    //Vengono reimpostati a 0 i valori da controllare, non pubblicando sul topic in questa eventualità, non si creano problemi coi messaggi generati dal nodo
    VelMsg.linear.x = 0;
    VelMsg.angular.z = 0;
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& LaserMsg){
    int LaserSize = LaserMsg->ranges.size();
    float totalForce[2];
    totalForce[0] = 0;
    totalForce[1] = 0;

    //Trovo le componenti delle forze generate dalla distanza del robot dagli ostacoli, la forza è inversamente proporzionale alla distanza
    for(int i=0; i < LaserSize; i++){
        if(LaserMsg->ranges[i] > LaserMsg->range_min && LaserMsg->ranges[i] < LaserMsg->range_max){
            float theta = LaserMsg->angle_min + i*LaserMsg->angle_increment;
            float deflectionForce = 1 / (LaserMsg->ranges[i]);
            if(((theta>=M_PI/4 && theta<=M_PI*7/4) || (theta<=-M_PI/4 && theta>=-M_PI*7/4)) && LaserMsg->ranges[i] < 0.2)
              nearImpact = true;
            theta += M_PI;
            totalForce[0] += deflectionForce * cos (theta);
            totalForce[1] += deflectionForce * sin (theta);            
        }
    }
    forceModule = sqrt(pow(totalForce[0], 2) + pow(totalForce[1], 2));
    forceAngle = atan2(totalForce[1], totalForce[0]);   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_avoidance");

  ros::NodeHandle n;

  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber subScan = n.subscribe("base_scan", 1, laserCallback);
  ros::Subscriber subVel = n.subscribe("cmd_vel", 2, velocityCallback);

  VelMsg.linear.x = 0;
  VelMsg.angular.z = 0;

  ros::spin();

  return 0;
}
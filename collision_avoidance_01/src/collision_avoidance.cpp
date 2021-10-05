#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include "math.h"

ros::Publisher velocity_pub;
geometry_msgs::Twist VelMsg;

int sensitivity = 800;
float forceModule = 0;
float forceAngle = 0;
bool nearImpact = false;
bool backCamera = false;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& RecVelMsg){
  //Per evitare di ricevere, elaborare e pubblicare dei topic creati dal nodo stesso, si effettuano dei controlli tramite il VelMsg
  if(RecVelMsg->linear.x != 0 && (RecVelMsg->linear.x != VelMsg.linear.x || RecVelMsg->angular.z != VelMsg.angular.z)){

    //Il modulo generato è in genere più grande della velocità data in input, quindi lo ridimensiono di conseguenza e prendo la componente che incide sul movimento lineare
    forceModule *= cos(forceAngle)*RecVelMsg->linear.x/sensitivity;

    //Per non deviare la traiettoria, inizialmente si va a modificare solo la componente lineare
    VelMsg.linear.x = RecVelMsg->linear.x + forceModule;  
    VelMsg.angular.z = RecVelMsg->angular.z;

    //Se lo scanner non da informazioni su cosa si trova esattamente dietro il robot e si desidera evitare movimenti pericolosi, limito lo spostamento all'indietro
    if(VelMsg.linear.x < 0 && !backCamera) VelMsg.linear.x /= 10;

    //Se il robot si trova in prossimità di un ostacolo, si modifica la traiettoria per evitarlo
    if(nearImpact) {
      VelMsg.angular.z += forceAngle;
      nearImpact = false;
      ROS_INFO("Near impact!");
    }

    ROS_INFO("Input: X=%f Z=%f \nDeflection: X=%f Z=%f",RecVelMsg->linear.x ,RecVelMsg->angular.z ,VelMsg.linear.x, VelMsg.angular.z);

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

            //La forza respingente è inversamente proporzionale all'ostacolo
            float deflectionForce = 1 / (LaserMsg->ranges[i]);
            
            //Modifico la traiettoria angolare solo se vi è il rischio di urtare su un lato
            if(((theta>=M_PI/5 && theta<=M_PI*4/5) || (theta<=-M_PI/5 && theta>=-M_PI*4/5)) && LaserMsg->ranges[i] < 0.25)
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

  //Il nodo collision_avoidance pubblica sul topic cmd_vel ed è iscritto ai topic cmd_vel e base_scan
  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber subScan = n.subscribe("base_scan", 1, laserCallback);
  ros::Subscriber subVel = n.subscribe("cmd_vel", 2, velocityCallback);

  VelMsg.linear.x = 0;
  VelMsg.angular.z = 0;

  //Il nodo entra nel ciclo di ascolto-publicazione
  ros::spin();

  return 0;
}
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <cmath>

//----------------------------------------------------- Funciones Auxiliares ----------------------------------------------------

// Función para mapear un rango de valores a otro. 
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Función convertir radianes a grados. 
float radianesAGrados(float rad) {
  return rad * (180.0 / M_PI);
}

//---------------------------- Variables que guardarán el valor del tópico convertido a grados ---------------------------------

float shoulderR;
float shoulderP;
float elbowR;
float elbowP;
float wristR;
float wristX;
float wristY;
float grabber;

//------------------------------------------------ Define los publicadores ---------------------------------------------------
ros::Publisher node_pub;
ros::Publisher node_pub2;

//Se definen los array que se publicarán
std_msgs::Float64MultiArray array_msg;

std_msgs::Float64MultiArray array_msg2;

//------------------------------------- Función que se ejecutará una vez suscrito al tópico ------------------------------------
void nodeCallback(const sensor_msgs::JointState& msg){
  //Establecer el tamaño de los array que se usarán
  array_msg.data.resize(5);

  array_msg2.data.resize(3);

  //Asignamos los valores a las variables creadas anteriormente:
  shoulderR = radianesAGrados(msg.position[0]);
  shoulderP = radianesAGrados(msg.position[1]);
  elbowR = radianesAGrados(msg.position[2]);
  elbowP = radianesAGrados(msg.position[3]);
  wristR = radianesAGrados(msg.position[4]);
  wristX = radianesAGrados(msg.position[5]);
  wristY= radianesAGrados(msg.position[6]);
  //grabber = radianesAGrados(msg.position[7]);
  grabber = mapFloat(msg.position[7], -0.008, 0.008, 0, 180);

  //Asignamos los datos a publicar: 
  //NOTA: Mapearemos con la función mapFloat el valor en grados. Esto para convertirlo a su valor equivalente en pulsos (valor con el
  //que opera la placa SSC-32), cuyo rango es de 500(0°) a 2500(180°)

  array_msg.data[0] = mapFloat(shoulderR, 0, 180, 500, 2500);
  array_msg.data[1] = mapFloat(shoulderP, 0, 180, 500, 2500);
  array_msg.data[2] = mapFloat(elbowR, 0, 180, 500, 2500);
  array_msg.data[3] = mapFloat(elbowP, 0, 180, 500, 2500);
  array_msg.data[4] = mapFloat(wristR, 0, 180, 500, 2500);
  array_msg2.data[0] = mapFloat(wristX, 0, 180, 500, 2500);
  array_msg2.data[1] = mapFloat(wristY, 0, 180, 500, 2500);
  array_msg2.data[2] = mapFloat(grabber, 0, 180, 500, 2500);
  
  node_pub.publish(array_msg);
  node_pub2.publish(array_msg2);


}


int main(int argc, char **argv){
  ros::init(argc, argv, "nodeFloatArray");

  ros::NodeHandle n;

  // Define el suscriptor para el tópico "joint_states"
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, nodeCallback);

  //Asignamos los publicadores
  node_pub = n.advertise<std_msgs::Float64MultiArray>("nodeFloatArray", 1000);
  node_pub2 = n.advertise<std_msgs::Float64MultiArray>("nodeFloatArray2", 1000);

  //Establecemos la velocidad en la que se publicarán los datos (10 veces por seg)
  ros::Rate loop_rate(10);

  // Ciclo para continuar publicando datos
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

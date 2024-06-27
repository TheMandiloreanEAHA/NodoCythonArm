/*
 ##----------------------------- Mover servomotores en físico con los valores recibidos de un tópico -----------------------------##
  El siguiente código es utilizado para mover los sevomotores de un brazo cyton, con respecto a los datos publicados en un 
  tópic de tipo "jointState". Este pograma se suscribe al tópico y esta en constante escucha de los datos publicados del tópico. 
  Los datos recibidos están en radianes, por lo que estos datos de conviertes a grados ya que así es como funciona el servomotor.
  También cuenta con una función que puede mapear un rango de valores dados a otro que sea deseado (De preferencia un rango 
  de grados entre 0-180).

  En esta ocasión, los servos no están conectados a arduino, si no que están conectados a una tarjeta SSC-32, 
  la placa arduino es un puente de comunicación entre ROS y la SSC-32.

 NOTA:El contenido de este archivo debe ser un archivo de tipo arduino (.ino).
 */
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
 
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle  nh;

// Se declaran las variables que almacenarán los datos de cada motor, públicados en el tópico /nodeFloatArray
float shoulderR, shoulderP, elbowR, elbowP, wristR, wristX, wristY, grabber;

//Función que se ejecutará una vez suscrito en el tópico
void arm_pose(const std_msgs::Float64MultiArray& msg){
  //asignamos los valores a la variables creadas previamente
  shoulderR = msg.data[0];
  shoulderP = msg.data[1];
  elbowR = msg.data[2];
  elbowP = msg.data[3];
  wristR = msg.data[4];

  //Movemos los respectivos motores
  move(0, shoulderR, 500);
  move(1, shoulderP, 500);
  move(2, elbowR, 500);
  move(3, elbowP, 500);
  move(4, wristR, 500);
}

void arm_pose2(const std_msgs::Float64MultiArray& msg){
  //asignamos los valores a la variables creadas previamente
  wristX = msg.data[0];
  wristY = msg.data[1];
  grabber = msg.data[2];

  //Movemos los respectivos motores
  move(5, wristX, 500);
  move(6, wristY, 500);
  move(7, grabber, 500);
  
}

//Función para comunicarse con la SSC-32 mediante el puerto Serial 2
void move(int servo, float pulse, int time) {
   Serial2.print("#");
   Serial2.print(servo);
   Serial2.print(" P");
   Serial2.print(pulse);
   Serial2.print(" T");
   Serial2.print(time);
   Serial2.print("\r\n");
}

//Nos suscribimos al tópico
ros::Subscriber<std_msgs::Float64MultiArray> sub("/nodeFloatArray", arm_pose);
ros::Subscriber<std_msgs::Float64MultiArray> sub2("/nodeFloatArray2", arm_pose2);

void setup(){
  // Inicializa el puerto serial para la comunicación de depuración, es importante que esté a la misma velocidad que la SSC-32
  Serial2.begin(115200); 
  
  //Iniciamos el nodo (serialNode) y nos suscribimos a los tópicos
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
}

void loop(){
  nh.spinOnce();
  delay(50);  
}

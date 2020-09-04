/* Nodo LineController */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include <vector>
#include <numeric>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <math.h>

#define PI 3.14159265
#define SATURATION 20 // Ángulo de saturación en grados
 
/* Clase del nodo del LineController */
class NodoController
{
	public:
	
/* Constructor de la clase */	
		NodoController()
		{		
			this->n=ros::NodeHandle("~"); // Inicialización del manejador de nodo
			this->sub=n.subscribe("/next_pos",1,&NodoController::data_Callback,this); // Inicialización del subscriptor a al topic dónde se publica 'el punto medio de la línea media' 
			this->twist_pub = n.advertise<geometry_msgs::Twist>("/teleop_keyboard/cmd_vel", 1); // Inicialización del publicador de velocidad
	
/* Incialización de la estructura de la velocidad */
			twist_msg.linear.x=0; // Velocidad lineal del vehículo en nuestro caso
			twist_msg.linear.y=0;
			twist_msg.linear.z=0;
			twist_msg.angular.x=0;
			twist_msg.angular.y=0;
			twist_msg.angular.z=0; // Velocidad angular del vehículo en nuestro caso
						
/* Inicialización del resto de variables */	
			lin = 0; // Velocidad lineal del vehículo
			ang = 0; // Velocidad angular del vehículo
			x_Next=330; // Posición en el eje x de 'el punto medio de la línea media', inicializado para un movimiento recto
			y_Next=0; // Posición en el eje y de 'el punto medio de la línea media'
			angle=0; // Ángulo de error
			x_coche=320; // Posición en el eje horizontal del coche respecto el mismo sistema de referencia que rige al 'punto medio de la línea media'
			y_coche=320; // Posición en el eje vertical del coche respecto el mismo sistema de referencia que rige al 'punto medio de la línea media'
		}
		
/* Función de callback del topic /next_pos */	
		void data_Callback(const geometry_msgs::Point::ConstPtr& msg)
		{
			if(std::abs(msg->x)<1000) // Comprobación de que el punto recibido es coherente
			{	
				x_Next=msg->x; // Actualizamos la posición en x
				y_Next=msg->y; // Actualizamos la posición en y
			}
		}
		
/* Función de llamada en el main() para su ejecución cíclica */
		void loop();
		
	private:
	
/* Declaración de las variables privadas de la clase*/
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher  twist_pub;
		geometry_msgs::Twist twist_msg;
		double lin, ang;
		int x_Next, y_Next;
		int x_coche, y_coche;
		double angle;
};

/* Función de llamada en el main() para su ejecución cíclica */
void NodoController::loop()
{
	angle =atan((double)(x_Next-x_coche)/(double)(y_coche-y_Next))*180/PI;
	ROS_INFO("Angle = %f grad \r\n",angle);
	
/* Ley de control */
	if(angle>-5 && angle<5) // Etapa de error 0
	{
		lin=0.1;
		ang=0;
		ROS_INFO("Recto");
	}
	else if(std::abs(angle)<SATURATION) // Etapa de control proporcional
	{	
		lin=0.1;
		ang=-0.6*angle/SATURATION;
	}	
	else if(angle>5) // Etapa de saturación - giro derecha
	{
		lin=0.1;
		ang=-0.6;
		ROS_INFO("Giro derecha");
	}
	else if(angle<-5) // Etapa de saturación - giro izquierda
	{
		lin=0.1;
		ang=0.6;
		ROS_INFO("Giro izquierda");
	}
	
/* Publicamos la velocidad actualizada */
	twist_msg.linear.x=lin;
	twist_msg.angular.z=ang;
	this->twist_pub.publish(this->twist_msg);
}

int main(int argc, char **argv)
{
/* Iniciamos el nodo */
	ros::init(argc, argv, "LineController");  
	NodoController nodo;
	
/* Definimos la frecuencia de ejecución y lanzamos el bucle */
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		nodo.loop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

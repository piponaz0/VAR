#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> 
#include <ctime> 
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class Wander {
	protected:
	ros::Publisher commandPub; 
	ros::Subscriber laserSub;
    ros::Subscriber speedSub; 

	double forwardVel;
	double rotateVel;
	double closestRange;

	string filename;
	fstream file_out;

	const double rango = 1;

	vector<double> pesos = {3.96633, -2.26918, 1.29999, 3.08029, -2.97882, -1.89187, 4.68018, -1.7888, -1.26946, -2.30283, -4.46275, -3.77767, -0.951535, -3.68175, -1.59274, 4.15091, -4.83802, -2.46562, 2.40288, -2.49546, 4.94031, -2.58008, -1.16119, -0.431453, -2.74648, 4.73788, 3.67763, 4.6468, 4.94876, 4.00276, -1.28655, 1.85612, -2.66806, -4.90032, -4.49729, 2.95184, 2.12123, 3.33763};

	public:
	Wander(ros::NodeHandle& nh) { //: rotateStartTime(ros::Time::now()), rotateDuration(0.f) {
		forwardVel=0.5; // despacito y con buena letra
//		rotateVel=0.0;
		closestRange=0.0;
		srand(time(NULL));
		// Este método nos permite indicar al sistema que vamos a publicar mensajes de cmd_vel
		// El valor de 1 indica que si acumulamos varios mensajes, solo el último será enviado.
		// El método devuelve el Publisher que recibirá los mensajes.
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
		laserSub = nh.subscribe("scan", 1, &Wander::commandCallback, this);
//        speedSub = nh.subscribe("cmd_vel", 1, &Wander::commandCallback2, this);
	};

/*    void commandCallback2(const geometry_msgs::Twist::ConstPtr& msg) {
        forwardVel = msg->linear.x;
        rotateVel = msg->angular.z;
    }
*/
	// Procesa los datos de láser
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
/*		ROS_INFO_STREAM("AngleMin: " << msg->angle_min); // Mínimo valor angular del láser
		ROS_INFO_STREAM("AngleMax: " << msg->angle_max); // Máximo valor angular del láser
		ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); // Incremento angular entre dos beams
		ROS_INFO_STREAM("RangeMin: " << msg->range_min); // Mínimo valor que devuelve el láser
		ROS_INFO_STREAM("RangeMax: " << msg->range_max); // Máximo valor que devuelve el láser. Valores por debajo y por encima de estos rangos no deben ser tenidos en cuenta.
		int totalValues = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment); // Total de valores que devuelve el láser
		for (int i=0; i< totalValues; i++) {
			ROS_INFO_STREAM("Values[" << i << "]:" << msg->ranges[i]); // Acceso a los valores de rango
		}
*/
		// TODO: a partir de los datos del láser se tiene que modificar las variables forwardVel y rotateVel para hacer que el robot no choque.
		// Bucles de escritura en fichero: 
		//		-> Nos interesan los cuadrantes primero (0º - 90º) y cuarto (270º - 360º)

		// Primer cuadrante
		int indice = 0;
		double resultado = pesos[38];
		for(int i = 0; i <= 90; i++){
			if(i % 5 == 0){
				double lecturaSensor;
				if(isinf(msg->ranges[i]) || msg->ranges[i] < 0.15 ) {
					lecturaSensor = 3.6;
				} else {
					lecturaSensor = msg->ranges[i];
				}
				
				resultado += lecturaSensor * pesos[indice];
				ROS_INFO_STREAM("Sensor = " << indice << ". Lectura = " << msg->ranges[i] << ". Resultado final: " << resultado);
				indice++;
			}
		}

		// Cuarto cuadrante
		for(int i = 270; i < 360; i++){
			if(i % 5 == 0){
				double lecturaSensor;
				if(isinf(msg->ranges[i]) || msg->ranges[i] < 0.15 ) {
					lecturaSensor = 3.6;
				} else {
					lecturaSensor = msg->ranges[i];
				}
				resultado += lecturaSensor * pesos[indice];
				ROS_INFO_STREAM("Sensor = " << indice << ". Lectura = " << msg->ranges[i] << ". Resultado final: " << resultado);
				indice++;
			}
		}

		ROS_INFO_STREAM("Resultado final: " << resultado);
		if (resultado > rango) {
			rotateVel = -1;
		} else {
			if (resultado < -1*rango) {
				rotateVel = 1;
			} else {
				rotateVel = 0;
			}
		}
	};
	
	// Bucle principal
	void bucle() {
		ros::Rate rate(5); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).

		file_out.open(filename, std::ios_base::out);
		while (ros::ok()) { // Bucle que estaremos ejecutando hasta que paremos este nodo o el roscore pare.
			geometry_msgs::Twist msg; // Este mensaje está compuesto por dos componentes: linear y angular. Permite especificar dichas velocidades
						  //  Cada componente tiene tres posibles valores: x, y, z, para cada componente de la velocidad. En el caso de
						  // robots que reciben velocidad linear y angular, debemos especificar la x linear y la z angular.
			msg.linear.x = forwardVel;
			msg.angular.z = rotateVel;

			commandPub.publish(msg);		
			ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
			
			rate.sleep(); // Espera a que finalice el ciclo
		}
	};
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "wander"); // Inicializa un nuevo nodo llamado wander
	ros::NodeHandle nh;
	Wander wand(nh); // Crea un objeto de esta clase y lo asocia con roscore
	wand.bucle(); // Ejecuta el bucle
	return 0;
};
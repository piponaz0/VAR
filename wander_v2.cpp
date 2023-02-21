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

	const double rango = 0.75;
	const double velGiro = 0.95;
	int cuentaChoque = 0;
	bool darMarchaAtras = false;

	vector<double> pesos = {3.96633, -2.26918, 1.29999, 3.08029, -2.97882, -1.89187, 4.68018, -1.7888, -1.26946, -2.30283, -4.46275,
							 -3.77767, -0.951535, -3.68175, -1.59274, 4.15091, -4.83802, -2.46562, 2.40288, -2.49546, 4.94031, -2.58008, 
							 -1.16119, -0.431453, -2.74648, 4.73788, 3.67763, 4.6468, 4.94876, 4.00276, -1.28655, 1.85612, -2.66806, -4.90032, 
							 -4.49729, 2.95184, 2.12123, 3.33763};

	public:
	Wander(ros::NodeHandle& nh) {
		closestRange=0.0;
		srand(time(NULL));
		// Este método nos permite indicar al sistema que vamos a publicar mensajes de cmd_vel
		// El valor de 1 indica que si acumulamos varios mensajes, solo el último será enviado.
		// El método devuelve el Publisher que recibirá los mensajes.
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
		laserSub = nh.subscribe("scan", 1, &Wander::commandCallback, this);
	};

	// Procesa los datos de láser
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		// Primer cuadrante
		int indice = 1;
		double resultado = pesos[0];

		//SUMA DE LOS TRES SENSORES MÁS FRONTALES PARA VER SI SE HA CHOCADO FRONTALMENTE CONTRA LA PARED
		double choqueFrontal = msg->ranges[0] + msg->ranges[1] + msg->ranges[359];
		if (choqueFrontal < 0.42) {
			cuentaChoque++;
			if (cuentaChoque >= 3) {
				darMarchaAtras = true;
			}
		} else {
			cuentaChoque = 0;
		}

		//Si se detecta que los sensores han dado choque durante 3 ejecuciones seguidas, movemos para atrás
		if (darMarchaAtras) {
			if (choqueFrontal < 1){
				forwardVel = -0.5;
				rotateVel = 1;
			} else {
				darMarchaAtras = false;
				forwardVel = 0.6;
				rotateVel = 0.0;
				cuentaChoque = 0;
			}
		}
		else {
			//ZONA DE MOVIMIENTO USANDO LA RED NEURONAL
			for(int i = 0; i <= 90; i++){
				if(i % 5 == 0){
					double lecturaSensor;
					if(isinf(msg->ranges[i]) || msg->ranges[i] < 0.15 ) {
						lecturaSensor = 3.6;
					} else {
						lecturaSensor = msg->ranges[i];
					}
					resultado += lecturaSensor * pesos[indice];
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
					indice++;
				}
			}

			//Determinamos la dirección del giro, segun el resultado devuelto por la red
			forwardVel = 0.6;
			if (resultado > rango) {
				rotateVel = -velGiro;
			} else {
				if (resultado < -rango) {
					rotateVel = velGiro;
				} else {
					rotateVel = 0;
				}
			}
		}
	};
	
	// Bucle principal
	void bucle() {
		ros::Rate rate(10); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).

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
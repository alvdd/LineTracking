/* Nodo LineDetector */

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

#define HALFRAIL 60 // mitad del ancho del carril 
#define DMAX 3.0/2.0*HALFRAIL // distancia máxima para que dos puntos pertenezcan a un mismo conjunto
#define MIDLANE 320 // posición en píxeles respecto al eje horizontal del eje vertical medio de la imagen en perspectiva de vista de pájaro
#define PI 3.141592
#define anglemax PI/3

/* Clase del nodo del LineDetector */
class NodoLinea
{
	public:
		cv::Mat imageINPUT;
		
/* Constructor de la clase */			
		NodoLinea()
		{
			this->n=ros::NodeHandle("~"); // Inicialización del manejador de nodo
			this->sub=n.subscribe("/camera/rgb/image_raw",1,&NodoLinea::data_Callback,this); // Inicialización del subscriptor al topic dónde se publica la imagen de la cámara rgb
			this->point_pub=n.advertise<geometry_msgs::Point>("/next_pos", 1); // Inicialización del publicador del 'punto medio de la línea media'
			av_x=-1; // Valor inicial en el eje x del 'punto medio de la línea media'
			av_y=-1; // Valor inicial en el eje y del 'punto medio de la línea media'
		}
			
/* Función de callback del topic /camera/rgb/image_raw */
		void data_Callback(const sensor_msgs::ImageConstPtr& msg)
		{
			//ROS_INFO("Entro en callback");
			cv_bridge::CvImagePtr cv_ptr; // Declaramos la estructura de cv_bridge necesaria para convertir la imagen a formato OpenCV
			cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8); // Guardamos la imagen y la codificamos en BGR (blue,green,red) de 8 bits
			imageINPUT=cv_ptr->image; // Extremos la imagen y la guardamos en una variable de OpenCV
		}

/* Función auxiliar para mostrar un vector bidimensional por terminal */
		void displayVector2d(std::vector< std::vector<int> > V);
		
/* Función que devuelve el número de elementos no nulos de un vector bidimensional */		
		int sizeVector2dnoNULL(std::vector<std::vector<int> > &V);
		
/* Función que genera la ROI y hace el cálculo de la imagen en perspectiva vista de pájaro a partir de la perspectiva cámara */		
		void dTransform(cv::Mat &M_in, cv::Mat &M_out);
		
/* Función que elimina la distorsión de la cámara según los parámetros de la lente */		
		void QuitDistorsion(cv::Mat &M_in, cv::Mat &M_out);
		
/* Función que imprime una línea sobre una imagen a partir de un vector bidimensional */		
		void dLines2d(std::vector< std::vector<int> > x_val, std::vector< std::vector<int> > y_val,cv::Mat &IM);
		
/* Función que imprime por terminal un vector unidimensional */				
		void displayVector1d(std::vector<int> V);
		
/* Función que imprime una línea sobre una imagen a partir de un vector unidimensional */		
		void dLines1d(std::vector< int > x_val, std::vector< int > y_val,cv::Mat &IM);
		
/* Función que calcula la imagen perspectiva cámara a partir de la imagen perspectiva vista de pájaro */		
		void iTransform(cv::Mat &M_in, cv::Mat &M_out);
		
/* Cálculo de la línea media y del punto medio de ésta*/
		int lmedia(std::vector<int> &xleft, std::vector<int> &yleft, std::vector<int> &xright, std::vector<int> &yright, std::vector<int> &x_medio, std::vector<int> &y_medio, cv::Mat &pintar);
		
/* Función que calcula la pendiente de una recta */		
		double pendiente(std::vector<int>Vx,std::vector<int>Vy);
		
/* Función de llamada en el main() para su ejecución cíclica */		
		void loop();
	private:
	
/* Declaración de las variables privadas de la clase*/	
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher point_pub;
		geometry_msgs::Point point_msg; // El 'punto medio de la línea media' se publica cómo variable de tipo punto
		int av_x; // Posición en el eje x del 'punto medio de la línea media'
		int av_y; // Posición en el eje y del 'punto medio de la línea media'
		cv::Mat src,src_gray,src_binary,src_binary_ND,src_binary_NDT,image_ROI,MDISPLAY;
		cv::Mat image_Label,stats,centroids;
};

/* Función de llamada en el main() para su ejecución cíclica */		
void NodoLinea::loop()
{
	src=imageINPUT.clone(); // clonamos la imagen convertida a formato Mat en el callback para no operar sobre la misma
	
	if(!src.empty()) // En caso de que se haya ejecutado el callback src no estará vacío y podremos ejecutar el programa
	{
		cv::cvtColor(src,src_gray,cv::COLOR_BGR2GRAY); //Convertimos la imagen de entrada a escala de grises
		cv::threshold(src_gray,src_binary,50,255,CV_THRESH_BINARY); // Binarizamos la imagen convertida a escala de grises
		
/* Declaramos las variables referidas a la ventana deslizante */
		int paso=5; // Ancho en píxeles entre las ROIs de detección
		int ngrupos=0; // Número de grupos de puntos, inicializado a 0
		cv::Point punto; // Variable auxiliar de tipo punto
		bool newgroup=true;
		int vA=20; // Área máxima de las regiones detectadas para evitar elementos que no pertenecen al carril
		std::vector<std::vector<int> > x_val(10); // Vector bidimensional dónde se almacenan las componentes x de los elementos de los grupos tal que x_val[grupo_al_que_pertenece][número_de_punto]
		std::vector<std::vector<int> > y_val(10); // Vector bidimensional dónde se almacenan las componentes y de los elementos de los grupos tal que y_val[grupo_al_que_pertenece][número_de_punto]
		x_val[0].push_back(1); // Inicialización
		y_val[0].push_back(1); // Inicialización
			
/* En caso de tener una cámara real, si se desea eliminar la distorsión descomentar las dos líneas que siguen y comentar la línea dTransform(src_binary, src_binary_NDT); */			
		//QuitDistorsion(src_binary, src_binary_ND);
		//dTransform(src_binary_ND, src_binary_NDT);
		
		dTransform(src_binary, src_binary_NDT); 
		dTransform(src, MDISPLAY); 

/* Agrupación por distancia */
		for(int y=198;y>100;y=y-paso) // Bucle que genera las ventanas deslizantes, se hace a lo largo de 100 píxeles aproximadamente ya que si 60 píxeles son la mitad del carril, 100 píxeles es menos del ancho del carril y teniendo en cuenta que el vehículo se encuentra dentro del carril no no es necesario ir en búsqueda de líneas más lejanas.
		{
			cv::Rect ROI=cv::Rect(0,y,639,1); // Generación de la ROI
			image_ROI=src_binary_NDT(ROI); // Aplicación de la ROI a la imagen
			cv::connectedComponentsWithStats(image_ROI,image_Label,stats,centroids); // Aplicado del etiquetado de regiones conectadas, que devuelve la imagen etiquetada, las estadísticas y los centroides
			cv::rectangle(src_binary_NDT,ROI,cv::Scalar(255,0,0),1,8,0); // Dibujado de la ROI sobre la imagen
										
/* Recorrido por todas las regiones conectadas obtenidas y aplicamos la clasificación por distancia*/										
			for(int ii=1;ii<stats.rows;ii++)
			{
				int32_t Area=stats.at<int32_t>(ii,cv::CC_STAT_AREA); // Extracción del área de la ii-ésima región
				if(0<Area && Area<vA) // La región es suficientemente pequeña para considerarse una línea
				{
					punto=cv::Point(int(centroids.at<double>(cv::Point(0,ii))),y); // Extraemos el centroide de la región y hacemos la conversión de sistema de referencia: el eje horizontal es idéntico entre la ventana deslizante y la imagen src_binary_NDT pero hay que hacer un desplazamiento de 'y' en el eje vertical
					cv::circle(src_binary_NDT,punto,1,cv::Scalar(0,255,0),CV_FILLED); // Dibujamos los centroides
					if(ngrupos>0) // Si existe al menos un grupo ejecutamos la comparación en base a distancias
					{
						int x1=punto.x;
						int y1=punto.y;
						
/* Se mide la distancia entre el punto actual y los últimos puntos de los grupos existentes, si cumple la condición de distancia se añade al grupo correspondiente y se sale del bucle */						
						for(int i=1;i<=ngrupos;i++)
						{
							int d=sqrt((x1-x_val[i][x_val[i].size()-1])*(x1-x_val[i][x_val[i].size()-1])+(y1-y_val[i][y_val[i].size()-1])*(y1-y_val[i][y_val[i].size()-1]));
							if(d<DMAX)
							{
								x_val[i].push_back(punto.x);
								y_val[i].push_back(punto.y);
								newgroup=false;
								break;
							}
						}
					}
/* En caso de que no existan grupos o no se haya podido añarir el punto a ninguno de los exitentes se crea un nuevo grupo cuyo primer punto es ii-
esimo centroide */
					if(newgroup || ngrupos==0)
					{
						ngrupos=ngrupos+1;
						x_val[ngrupos].push_back(punto.x);
						y_val[ngrupos].push_back(punto.y);
					}
					newgroup=true;
				}

			}
		}
		ROS_INFO("Numero de grupos %d",ngrupos);
		
/* Detección de las líneas derecha e izquierda (en caso de que se haya detectado algún grupo de puntos) */
		if(ngrupos>0)
		{
		
/* Declaración de variables */
			int idx_left,idx_right; // Índices de los grupos de línea izquierda y línea derecha (respecto el vehículo)
			int index_min1,min1;
			int index_min2,min2;
			index_min1=-1;
			index_min2=-1;
			min1=1000;
			min2=800;
			std::vector<int> prim;
			
/* Cálculo de dos conjuntos más próximos a derecha e izquierda del punto medio del carril*/			
			for(int kk=1;kk<(ngrupos+1);kk=kk+1)
			{

				if(x_val[kk].size()>=2 && x_val[kk][0]>200) // Dado que un carril tiene da ancho 120, detectase un conjunto por debajo de 320-120=200 no podría ser la línea central discontinua, sino que probablemente se trate de la línea continua exterior del carril contrario así que hay que establecer un mínimo para la primera posición del conjunto central
				{
					int et2=x_val[kk][0]-320;
					ROS_INFO("et2 = %d",et2);
					if(et2<min1 && et2>=0)
					{
						min1=et2;
						index_min1=kk;
					}
					else if(abs(et2)<abs(min2) && et2<0)
					{
						min2=et2;
						index_min2=kk;
					}
				}
				
			}
			
			if(index_min1!=-1 && index_min2!=-1) // Ejecutamos el código que sigue en caso de que se hayan detectado los dos carriles 
			{
/* Mediante una comparativa de valores, asignamos los índices derecho e izquierdo */
				if(x_val[index_min1][0]<x_val[index_min2][0])
				{
					idx_left=index_min1;
					idx_right=index_min2;
				}
				else
				{
					idx_left=index_min2;
					idx_right=index_min1;
				}
				ROS_INFO("Origen x de linea izq = %d",x_val[idx_left][0]);
				ROS_INFO("Origen x de linea der = %d",x_val[idx_right][0]);	

/* Declaración y asignación del valor según los índices a los vectores left y right de las líneas correspondientes */
				std::vector<int> xleft,yleft,xright,yright;
				xleft=x_val[idx_left];
				yleft=y_val[idx_left];
				xright=x_val[idx_right];
				yright=y_val[idx_right];
				
/* Inicialización de las penientes a valores muy altos, por si no puede calcularlas al no haber más de un punto en los conjuntos */				
				double m_left=100;
				double m_right=100;
				if(xleft.size()>1 && xright.size()>1) // En caso de que haya más de un punto en cada conjunto y se puedan calcular las pendientes ejecutamos el código que sigue
				{
				
/* Cálculo de las pendientes de los vectores*/				
					m_left=pendiente(xleft,yleft);
					m_right=pendiente(xright,yright);
					ROS_INFO("m izq = %f",pendiente(xleft,yleft));
					ROS_INFO("m der = %f",pendiente(xright,yright));
				}
				std::vector<int> x_medio,y_medio; // Declaración de los ectores que almacenarán la línea media
				
/* Análisis de la pendiente para evaluar si la información recibida es representativa de la realidad*/				
				if(m_left>-1 && m_left<0)	
				{
					ROS_INFO("Entro en 1");
					x_medio.push_back(xleft[xleft.size()-1]+HALFRAIL);
					y_medio.push_back(yleft[yleft.size()-1]);
				}
				else if(m_right>-1 && m_right<0 )
				{
					ROS_INFO("Entro en 2");
					x_medio.push_back(xright[xright.size()-1]+HALFRAIL);
					y_medio.push_back(yright[yright.size()-1]);

				}
				else if(m_left<1 && m_left>0 && xleft[0]>150)
				{
					ROS_INFO("Entro en 3");
					av_x=322;
					av_y=190; 
				}
				else if(m_right<1 && m_right>0 && xleft[0]<150)
				{
					ROS_INFO("Entro en 4");
					x_medio.push_back(xright[xright.size()-1]-HALFRAIL);
					y_medio.push_back(yright[yright.size()-1]); 
				}
				else
				{
					ROS_INFO("Entro en 5");
					lmedia(xleft,yleft,xright,yright,x_medio,y_medio,MDISPLAY);
				}
			}
			else // Se ejecuta el código que sigue si no se han detectado los dos carriles
			{
/* Declaración de variables */			
				std::vector<int> xleft,yleft,xright,yright;
				std::vector<int> x_medio,y_medio;
				
/* Se comprueba qué carril se ha detectado y se asigna el grupo a xleft e yleft */				
				if(index_min1!=-1)
				{
					xleft=x_val[index_min1];
					yleft=y_val[index_min1];
				}
				else if(index_min2!=-1)
				{
					xleft=x_val[index_min2];
					yleft=y_val[index_min2];
				}
				else // En caso de que sólo haya un grupo tanto index_min1 como index_min2 valen -1, por lo que se añade directamente el grupo a xleft e yleft
				{
					xleft=x_val[1];
					yleft=y_val[1];
				}
/* Asignamos un 0 a xright e yright con lo que le decimo a la función lmedia que sólo se ha detectado una línea y que se encargue de averiguar si es la derecha o la izquierda */				
				xright.push_back(0);
				yright.push_back(0);
				
				ROS_INFO("xleft = %d",xleft[1]);
				ROS_INFO("yleft = %d",yleft[1]);
				lmedia(xleft,yleft,xright,yright,x_medio,y_medio,MDISPLAY);
				ROS_INFO("Solo detecta 1 grupo");
			}
/* Actualizamos las variables del 'punto medio de la línea media' y publicamos en el topic sus coordenadas*/			
			point_msg.x=av_x;
			point_msg.y=av_y;
			this->point_pub.publish(this->point_msg);
			
/* Hacemos la transformada inversa y mostramos cómo transcurre la detección: tanto desde la perspectiva de la cámara como desde la perspectiva vista de pájaro */			
			iTransform(MDISPLAY,MDISPLAY);
			imshow("Bird view",src_binary_NDT);
			imshow("Lines normal perspective",MDISPLAY);
			cv::waitKey(1);
		}
		else // En caso de que se hayan detectado 0 grupos estamos en una situación desconocida, por lo que se avisa de ello
		{
			ROS_INFO("Situación desconocida");
		}
	}
	
	
}

/* Cálculo de la línea media y del punto medio de ésta*/
int NodoLinea::lmedia(std::vector<int>& xleft, std::vector<int>& yleft, std::vector<int>& xright, std::vector<int>& yright, std::vector<int>& x_medio, std::vector<int>& y_medio, cv::Mat& pintar)
{
	int ii=0;
	int jj=0;
	ROS_INFO("yleft[0] = %d, yright[0] = %d",yleft[0],yright[0]);
	ROS_INFO("xleft[0] = %d, xright[0] = %d",xleft[0],xright[0]);
		
	if(xright[0]!=0 && yright[0]!=0) // Ejecutamos el código que sigue en caso de que se haya detectado la línea derecha e izquierda
	{
/* Cálculo de la línea media según el procedimiento descrito en la memoria */	
		while(yleft[jj]>130 && yright[ii]>130)
		{
			x_medio.push_back((int)(xleft[jj]+xright[ii])/2);
			y_medio.push_back((int)(yleft[jj]+yright[ii])/2);
			cv::circle(pintar,cv::Point(xleft[jj],yleft[jj]),1,cv::Scalar(255,0,0),CV_FILLED);	  
			cv::circle(pintar,cv::Point(xright[jj],yright[jj]),1,cv::Scalar(255,255,0),CV_FILLED);	  
			cv::circle(pintar,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),1,cv::Scalar(255,0,0),CV_FILLED);	  
			if(yright[ii]<yleft[jj])
			{
				jj=jj+1;
			}
			else
			{
				ii=ii+1;
			}
		}
		if(x_medio.empty()) // En caso de que lo anterior no se haya podido ejecutar, analizamos la situación cómo sigue
		{
			if(yleft[jj]>130) // El problema está en la derecha
			{
				ROS_INFO("Solo se detecta la línea izquierda");
				while(yleft[jj]>130)
				{
					if(yleft[jj]<320)
					{
						x_medio.push_back((int)(xleft[jj]+HALFRAIL)); 
						y_medio.push_back((int)(yleft[jj]));
						cv::circle(src_binary_NDT,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),5,cv::Scalar(255,255,255),CV_FILLED);	  
						cv::circle(MDISPLAY,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),3,cv::Scalar(255,255,255),CV_FILLED);  
						
					}
					jj=jj+1;
					
				}
			}
			else if(yright[jj]>130) // El problema está en la izquierda
			{
				ROS_INFO("Solo se detecta la línea derecha");
				while(yright[jj]>130)
				{
					x_medio.push_back(xleft[jj]-HALFRAIL); // 60 es la mitad de la distancia aproximada entre las líneas para una y dada 
					y_medio.push_back(yleft[jj]);
					cv::circle(pintar,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),5,cv::Scalar(255,255,255),CV_FILLED);	  
					jj=jj+1;
				}
			}
			else // En otro caso no se detecta ninguna línea y se avisa por terminal
			{ 
				ROS_INFO("No se detecta ningua línea");
			}
		}
		else
		{
			ROS_INFO("Se detectan las dos líneas");
		}
	}
	else // Entramos en el else si sólo se ha detectado una línea
	{
		ROS_INFO("yleft[0] = %d",yleft[0]);
		ROS_INFO("xleft[0] = %d",xleft[0]);
		if(xleft.size()>1) // Comprobamos el tamaño del vector para ver si podemos calcular la pendiente     
		{
		
/* Calculamos la pendiente comprobando que los valores en el eje x no sean demasiado similares para evitar indeterminaciones */		
			double m = 10;
			if(abs(xleft[0]-xleft[xleft.size()-1])>1)
			{
				m=double(yleft[0]-yleft[yleft.size()-1])/double(xleft[0]-xleft[xleft.size()-1]);
				ROS_INFO("m = %f",m);
			}
			
/* En función de la pendiente y del primer punto del vector en el eje x se determina primero si se trata de una curva y el sentido de la misma, en tal caso se define la línea media como el último punto del vector desplazado la mitad del carril en el eje horizontal o de si se trata de una línea recta, en tal caso la línea media la conforman todos los puntos de la línea izquierda o derecha desplazados la mitad del carril*/			
			if(m<3 && m>3 && xleft[0]>320)
			{
				x_medio.push_back((int)(xleft[xleft.size()-1]-HALFRAIL));
				y_medio.push_back((int)(yleft[xleft.size()-1]));
			}
			else if(m>-2.5 && m<2.5 && xleft[0]<350)
			{
				x_medio.push_back((int)(xleft[xleft.size()-1]+HALFRAIL));
				y_medio.push_back((int)(yleft[xleft.size()-1]));
			}
			else if(xleft[0]<320)
			{
				for(int jj=0;jj<yleft.size();jj++)
				{
					ROS_INFO("Se detecta la linea izquierda");
					x_medio.push_back((int)(xleft[jj]+HALFRAIL));
					y_medio.push_back((int)(yleft[jj]));
					cv::circle(src_binary_NDT,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),5,cv::Scalar(255,255,255),CV_FILLED);	  
					cv::circle(MDISPLAY,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),3,cv::Scalar(255,255,255),CV_FILLED); 
				}
			}
			else
			{
				for(int jj=0;jj<yleft.size();jj++)
				{
					ROS_INFO("Se detecta la linea derecha");
					x_medio.push_back((int)(xleft[jj]-HALFRAIL));
					y_medio.push_back((int)(yleft[jj]));
					cv::circle(src_binary_NDT,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),5,cv::Scalar(255,255,255),CV_FILLED);	  
					cv::circle(MDISPLAY,cv::Point(x_medio[x_medio.size()-1],y_medio[y_medio.size()-1]),3,cv::Scalar(255,255,255),CV_FILLED); 
				}
			}				
		}
		else if(xleft[0]<320)
		{
			x_medio.push_back((int)(xleft[0]+HALFRAIL));
			y_medio.push_back((int)(yleft[0]));
		}
		else
		{
			x_medio.push_back((int)(xleft[0]-HALFRAIL));
			y_medio.push_back((int)(yleft[0]));
		} 
	}	
	
/* Si el vector x_medio no está vacío se calcula el nuevo punto central del carril y se actualiza su valor */	
	if(!x_medio.empty())
	{
		int av_x1,av_y1;
		av_x1=std::accumulate(x_medio.begin(),x_medio.end(),0)/x_medio.size();
		av_y1=std::accumulate(y_medio.begin(),y_medio.end(),0)/y_medio.size();
		
		av_x=av_x1;
		av_y=av_y1;
		
	}
	ROS_INFO("Media central x= %d  y=%d",av_x,av_y);
	cv::circle(pintar,cv::Point(av_x,av_y),3,cv::Scalar(255,255,0),CV_FILLED);
	return 0;
	
}

/* Función que imprime una línea sobre una imagen a partir de un vector bidimensional */		
void NodoLinea::dLines2d(std::vector< std::vector<int> > x_val, std::vector< std::vector<int> > y_val,cv::Mat &IM)
{
	for(int j=1;j<sizeVector2dnoNULL(x_val);j++)
	{
		for(int i=0;i<x_val[j].size()-1;i++)
		{
			cv::line(IM,cv::Point(x_val[j][i], y_val[j][i]), cv::Point(x_val[j][i+1],y_val[j][i+1]), cv::Scalar(j+50,0,255), 1);
		}
	}
	
}

/* Función que imprime una línea sobre una imagen a partir de un vector unidimensional */		
void NodoLinea::dLines1d(std::vector< int > x_val, std::vector< int > y_val,cv::Mat &IM)
{
	for(int i=0;i<x_val.size()-1;i++)
	{
		cv::line(IM,cv::Point(x_val[i], y_val[i]), cv::Point(x_val[i+1],y_val[i+1]), cv::Scalar(50,0,255), 1);
	}
}

/* Función que elimina la distorsión de la cámara según los parámetros de la lente */		
void NodoLinea::QuitDistorsion(cv::Mat &M_in, cv::Mat &M_out)
{
/* Los parámetros se pueden consultar con rostopic echo /camera/rgb/camera_info */
	cv::Mat K=(cv::Mat_<float>(3,3)<<
	726.3009328586187,0,-320.5,
	0,726.3009328586187,-240.5,
	0,0,1);
	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F); 
	
/* Función que corrige la distorsión */	
	undistort(M_in,M_out,K,distCoeffs);
}

/* Función que genera la ROI y hace el cálculo de la imagen en perspectiva vista de pájaro a partir de la perspectiva cámara */	
void NodoLinea::dTransform(cv::Mat &M_in, cv::Mat &M_out)
{

	int corte=280;
	int IMAGE_H,IMAGE_W;
	IMAGE_H=corte-1;
	IMAGE_W=639;
	
/* Segmentación de la ROI */	
	cv::Rect ROI2=cv::Rect(0,corte,639,479-corte);
	cv::Mat src_3ROI=M_in(ROI2);

/* Definición de los puntos en los que basarse para hacer la transformación */	
	std::vector<cv::Point> p_cal;
	p_cal.push_back(cv::Point(0,IMAGE_H));
	p_cal.push_back(cv::Point(IMAGE_W,IMAGE_H));
	p_cal.push_back(cv::Point(0,0));
	p_cal.push_back(cv::Point(IMAGE_W,0));
	
	std::vector<cv::Point> p_cald;
	p_cald.push_back(cv::Point(260,IMAGE_H-80));
	p_cald.push_back(cv::Point(IMAGE_W-260,IMAGE_H-80));
	p_cald.push_back(cv::Point(0,0));
	p_cald.push_back(cv::Point(IMAGE_W,0));
	
	
	cv::Point2f src_vertices2[4];
	src_vertices2[0]=p_cal[0];
	src_vertices2[1]=p_cal[1];
	src_vertices2[2]=p_cal[2];
	src_vertices2[3]=p_cal[3];
	
	cv::Point2f dst_vertices2[4];
	dst_vertices2[0]=p_cald[0];
	dst_vertices2[1]=p_cald[1];	
	dst_vertices2[2]=p_cald[2];  
	dst_vertices2[3]=p_cald[3];
	
/* Cálculo de la matriz de mapeo y obtención de la imagen en perspectiva vista de pájaro */	
	cv::Mat warpMatrix2 = getPerspectiveTransform(src_vertices2, dst_vertices2);
	warpPerspective(src_3ROI,M_out,warpMatrix2,src_3ROI.size());
}

/* Función que calcula la imagen perspectiva cámara a partir de la imagen perspectiva vista de pájaro */	
void NodoLinea::iTransform(cv::Mat &M_in, cv::Mat &M_out)
{


	int corte=280;
	int IMAGE_H,IMAGE_W;
	IMAGE_H=corte-1;
	IMAGE_W=639; 
	
/* Definición de los puntos en los que basarse para hacer la transformación */    	
	std::vector<cv::Point> p_cal;
	p_cal.push_back(cv::Point(0,IMAGE_H));
	p_cal.push_back(cv::Point(IMAGE_W,IMAGE_H));
	p_cal.push_back(cv::Point(0,0));
	p_cal.push_back(cv::Point(IMAGE_W,0));
	
	std::vector<cv::Point> p_cald;
	p_cald.push_back(cv::Point(260,IMAGE_H-80));
	p_cald.push_back(cv::Point(IMAGE_W-260,IMAGE_H-80));
	p_cald.push_back(cv::Point(0,0));
	p_cald.push_back(cv::Point(IMAGE_W,0));
	
	
	cv::Point2f src_vertices2[4];
	src_vertices2[0]=p_cal[0];
	src_vertices2[1]=p_cal[1];
	src_vertices2[2]=p_cal[2];
	src_vertices2[3]=p_cal[3];
	
	cv::Point2f dst_vertices2[4];
	dst_vertices2[0]=p_cald[0];
	dst_vertices2[1]=p_cald[1];	
	dst_vertices2[2]=p_cald[2];  
	dst_vertices2[3]=p_cald[3];

/* Cálculo de la matriz de mapeo y obtención de la imagen en la perspectiva de la cámara */    	
	cv::Mat warpMatrix2 = getPerspectiveTransform(dst_vertices2, src_vertices2);
	warpPerspective(M_in,M_out,warpMatrix2,M_out.size());
}

/* Función auxiliar para mostrar un vector bidimensional por terminal */
void NodoLinea::displayVector2d(std::vector< std::vector<int> > V)
{
	std::cout<<"Vector de posiciones: "<<std::endl;
	for(int i=0;i<sizeVector2dnoNULL(V);i++)
	{
		for(int j=0;j<V[i].size();j++)
		{
			std::cout<<V[i][j]<<" ";
		}
		std::cout<<std::endl;
	}  
}

/* Función que imprime por terminal un vector unidimensional */	
void NodoLinea::displayVector1d(std::vector<int> V)
{
	for(int j=0;j<V.size();j++)
	{
		std::cout<<V[j]<<" ";
	}
	std::cout<<std::endl;  
}

/* Función que devuelve el número de elementos no nulos de un vector bidimensional */		
int NodoLinea::sizeVector2dnoNULL(std::vector<std::vector<int> > &V)
{
	for(int i=0;i<V.size();i++)
	{
		if(V[i].empty())
		{
			return i;
			break;
		}
	}
	return V.size();  
}

/* Función que calcula la pendiente de una recta */		
double NodoLinea::pendiente(std::vector<int>Vx,std::vector<int>Vy)
{
	double x0,x1,y0,y1;
	x0=(double)Vx[0];
	y0=(double)Vy[0];
	x1=(double)Vx[Vx.size()-1],
	y1=(double)Vy[Vy.size()-1];

	return (y1-y0)/(x1-x0);
}


int main(int argc, char **argv)
{
/* Iniciamos el nodo */
	ros::init(argc, argv, "LineDetector");  
	NodoLinea nodo;
	
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

	

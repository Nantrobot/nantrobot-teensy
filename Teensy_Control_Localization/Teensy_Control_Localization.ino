/*
Programme asservissement position et localisation pour robot mobile (2,0)
Xavier Dauptain 
Copyright ©.All right reserved.

Dernière modification : 01/03/2016 : Modification de commentaires pour rendu académique


-Principe de l'asservissement:
On peut commuter entre deux asservissements complémentaires selon le type de tache à effectuer
Mode 0:
Asservissement en distance parcourue (L) et en orientation absolue (Theta) dit "polaire" ou classique.
Mode 1:
Asservissement en deux étage :
Haut niveau : consignes cinématiques intermédiaires Vcons et Omegacons générées par découplage statique
Bas niveau :asservissement en vitesse des roues par mode glissant pour amener V et Omega
sur Vcons et Omegacons.

-Principe de l'algorithme de localisation : fusion des données odométriques et de celles issues
d'un télémètre laser (Hokuyo) pouvant détecter trois balises

Modèle :

X_k+1=f(X_k,U_k)  avec X_k=(x_k,y_k,theta_k)  
Y_k=g(X_k) où Y_k est le vecteur de mesures

Algorithme:

U=(distance curviligne parcourue,theta)
On a P_0=Covariance initiale sur l'estimation de la pose
Et X_0=Pose initiale connue 
Qbeta matrice de covariance fixe sur l'odométrie à définir par test(à priori diagonale) -> paramètre
Qgamma covariance du bruit de mesure à évaluer -> paramètre



////////In the  µC (Teensy)
//State Prediction
delta_d=U(1)-U_prec(1);
delta_theta=U(2)-U_prec(2);
x+=delta_d*cos(theta+delta_theta/2);
y+=delta_d*sin(theta+delta_theta/2);
theta+=delta_theta;
X_k=(x,y,theta)
U_prec=U

//Error Propagation
A =[1,0,-delta_d*sin(theta)]
    [0,1, delta_d*cos(theta)] 
    [0,     0,         1]
B =[cos(theta),0]
   [sin(theta),0]
   [0        ,1]
P = A*P*A.'+B*Qbeta*B.'  ;


/////////In the embedded computer (Raspberry Pi)

Dès que nouvelle mesure de l'hokuyo :

Requête des positions estimées de chaque balise à la Teensy et de paramètres utiles au calcul de la distance de Mahalanobis de chaque point du cluster
à ces positions estimées.

//Cloud Processing

-Calcul des coordonnées de chaque point du nuage et la distance de mahalanobis au coordonnées estimées des balises Y_hat
-On stocke dans trois vecteurs les clusters associés aux balises 1,2 et 3.
-On estime les coordonnées des balises à partir de chaque cluster.


On envoie pour chaque balise i les coordonnées calculées.


////////In the  µC (Teensy)

Sur requête del'ordinateur , pour chaque balise dans la liste faire:

//Expected Measurements :
Yhat=(sTo(X_k)*[alpha_i;beta_i;1])(1:2) pour la balise i ,alpha et beta étant les coordonnées cartésiennes (connues)dans le repère du terrain
On envoie l'estimée des positions des balises dans le repère capteur
Y =g(X_k)=
 
 [alpha_i*cos(theta_k) - a + beta_i*sin(theta_k) - x_k*cos(theta_k) - y_k*sin(theta_k)]
 [beta_i*cos(theta_k) - b - alpha_i*sin(theta_k) - y_k*cos(theta) + x_k*sin(theta_k)  ]
 
//Jacobian Matrix of measurement equations
C=(dg/dx)(X_k)

On utilise jacobian(Y,[x,y,theta]) sur Matlab et on obtient :

[ -cos(theta), -sin(theta), beta_i*cos(theta) - alpha_i*sin(theta) - y*cos(theta) + x*sin(theta)]
[  sin(theta), -cos(theta), x*cos(theta) - beta_i*sin(theta) - alpha_i*cos(theta) + y*sin(theta)]


à évaluer pour x=x_k,y=y_k et theta=theta_k

//Innovation 
d_maha²=transpose((y_point_cluster_candidat-Yhat))*inv(C*P*C'+Qgamma)*(y_point_cluster_candidat-Yhat)
d_maha²=transpose((y_point_cluster_candidat-Yhat))*SIGMA_MAHA*(y_point_cluster_candidat-Yhat)
->actualiser et transmettre SIGMA_MAHA pour que l'ordinateur embarqué puisse déterminer les points du cluster associés à la
balise i
K=P*C.'*inv(C*P*C.'+Qgamma)

fin pour chaque balise


//State Estimation and new error estimation
Sur réception de mesures,pour chaque mesure (dans l'idéal une par balise)faire
X_k=X_k+K*(Y-Yhat) ou Y est la mesure
P = (eye(3)-K*C)*P ;

fin pour chaque mesure





-Matériel:
Teensy 3.1
Motoréducteurs C.C Faulhaber 024CR
Encodeurs Kubler 05.2400.1122.1024
Pont H : L298 (fréquence max 40kHz)



*/

//-----------------------------DEBUG--------------------------------
//#define DEBUG_MODE

//-----------------------------Names--------------------------------
#define __MK20DX256__
#define motord 5
#define motorg 6
#define enabled 3  //PWM : Timer 1
#define enableg 4  //PWM : Timer 1

#define encoderRA 23 //Roue droite quand on regarde le robot de face
#define encoderRB 22
#define encoderLA 20 //Roue gauche quand on regarde le robot de face
#define encoderLB 21
#define LED 13
//---------------------------Fixed Parameters-------------------------

#define ASSER_FREQUENCY 500 //En Hz
#define X_start 850
#define Y_start 100
#define Theta_start Pi/2
#define FIR_LENGTH 10

//---------------------------ROS libraries----------------------------

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>

//---------------------------Libraries-----------------------

#include <Encoder.h>
#include <TimerThree.h>
#include "loc_utilities.h"
#include "ringbuff.h"
//#include "vector.h";

//-------------------------Variables and objects---------------------

Encoder RightEnc(encoderRA,encoderRB);
Encoder LeftEnc(encoderLA,encoderLB);


ring_buffer<FIR_LENGTH> derivr_buff;
ring_buffer<FIR_LENGTH> derivl_buff;

unsigned long periode=(long)(1000000/ASSER_FREQUENCY);
const float periode_asserv_seconde=(1.0/ASSER_FREQUENCY);

long tickR_odo=0;
long tickRprec_odo=0;
volatile long tickR=0;
volatile long tickRprec=0;

long tickL_odo=0;
long tickLprec_odo=0;
volatile long tickL=0;
volatile long tickLprec=0;

volatile int cnt=0;
int wait_counter=0;
volatile boolean mode=true;// 0: en x,y pour un point de l'axe de symétrie hors axe des roues -1: classique en incrément L (incrément distance parcourue ou abscisse curviligne),Theta  
volatile boolean mode_prec=true;

//---------------------------Dimensions------------------------------

const double L=117.5;//demi-empattement en mm
const double e=2*L;
const double Rcg=50.5;//rayon roue codeuse gauche en mm
const double Rcd=50.5;//rayon roue codeuse droite en mm
const int npas=4096;//nombre d'impulsions par tours encodeurs en quadruple précision
const double a=100.0;//distance point asservi axe des roues (sur l'axe de symétrie) en mm


//-----------------------Variables pose & Localization -----------------------------

double X=X_start;
double Y=Y_start;
double theta=Theta_start;
double dist;

mat3 P;
mat2 Q_wheels,Q_beta,Q_gamma,JointToCartesian;
beacon_data beacon[3];//liste des balises

//TO BE TUNED
const double sigmaX=30;//+/- 30 en mm.Incertitude sur la position initiale
const double sigmaY=25;//en mm
const double sigmaTheta=2*Pi/180;//+/- 2 °
double sigma_wheels=0.003;//à définir par test
const double sigma_mesure=15;//en mm NE PAS définir par test : vérifier la variance de la mesure
const double telemeter_decX=-30;//telemeter decX in robot frame 
const double telemeter_decY= -5;//telemeter decY in robot frame
const double longueur=3000;//longueur du terrain en mm
const double largeur=2000;//largeur du terrain en mm 

//-------------------------Variables consignes-----------------------

//Mode 0
double Xcons=X+a*cos(theta);
double Ycons=Y+a*sin(theta);
volatile double omegacd=0.0;
volatile double omegacg=0.0;


//Mode 1
double cons_dist=0;//en mm
double cons_distprec=0;//en mm
double cons_or=0;//en degrés
volatile int consR=0;
volatile int consL=0;



//-----------------------Control parameters--------------------------

//Mode 0
double kpe=10.0;//gain proportionnels retour d'état statique
const double epsilon=2.0;//"amortissement" sigmoide sliding mode 
const double max_move_radius_security=100;//en mm

//Mode 1 (PID en L, Theta)
double kp=10.0;//gain proportionnel
double ki=10.0;//gain intégral
double kd=1.0;//gain dérivé

volatile int sumerreurR=0;//somme intégrale des erreurs 
volatile int sumerreurL=0;


//-----------------ROS instanciation and subscriber function----------


ros::NodeHandle  nh;
std_msgs::Float32MultiArray Estimation_Array;
ros::Publisher estimation_pub("Loc_Estimation",&Estimation_Array);

void actualiserCons(const geometry_msgs::Point32& cons_msg){
  cli();//On désactive les interruptions içi
  mode=(int(cons_msg.z) & 1);
  boolean reset=((int(cons_msg.z)>>1) & 1);
  if(reset){
    //Same mode
    mode=mode_prec;
    //Odometry reset
    RightEnc.write(0);
    LeftEnc.write(0);
    tickRprec=0;
    tickLprec=0;
    tickRprec_odo=0;
    tickLprec_odo=0;
    X=cons_msg.x;
    Y=cons_msg.y;
    theta=Theta_start;
    dist=0;
    P << sigmaX*sigmaX,     0 ,        0,
          0    , sigmaY*sigmaY,        0,
          0    ,    0  ,  sigmaTheta*sigmaTheta;      
    // mode 1 reset
    cons_distprec=dist;
    cons_dist=dist;
    cons_or=theta;
    derivl_buff.clear();
    derivr_buff.clear();
    sumerreurR=0;
    sumerreurL=0;
    //mode 0 reset
    Xcons=X+a*cos(theta);
    Ycons=Y+a*sin(theta);  
    isrh();   
  }else if(mode==0){
    double Xdef=X+a*cos(theta);
    double Ydef=Y+a*sin(theta);
    Xcons=cons_msg.x;
    Ycons=cons_msg.y;
    if(sq(Xcons-Xdef)+sq(Ycons-Ydef)>sq(max_move_radius_security)){//Sécurité limitant les mouvements en X,Y aux petits déplacements
      Xcons=Xdef;
      Ycons=Ydef;
    }
    if(mode_prec!=mode){
       isrh();//compute Vcons & OmegaCons now if mode_prec was 1 (L,Theta)
       derivl_buff.clear();//réinitialisation du buffer des deltas (vitesses) roues
       derivr_buff.clear();
    }
  }else if(mode==1){
    if(mode_prec!=mode){
      cons_distprec=dist;
      derivl_buff.clear();//réinitialisation des erreurs dérivées
      derivr_buff.clear();
      sumerreurR=0;//réinitialisation des erreurs intégrales
      sumerreurL=0;
    }
    cons_dist=cons_msg.x+cons_distprec;
    cons_distprec=cons_dist;
    cons_or=cons_msg.y;
  } 
  mode_prec=mode;
  sei();
}

void actualiserGains(const geometry_msgs::Point32& gains_msg){
  kp=gains_msg.x;
  ki=gains_msg.y;
  kd=gains_msg.z;
  sumerreurR=0;
  sumerreurL=0;
}


void sendEstimation(const std_msgs::Empty& request){
  float tab[18];//3*(2+4) 3 beacons,2 scalar for Yhat and 4 for sigma_maha
  int j=0;
  for (int i=0;i<3;i++){//Pour chaque balise
    //Calculs
    double alpha_i= beacon[i].map_frame_coords(0,0);
    double beta_i= beacon[i].map_frame_coords(1,0);
    beacon[i].y_hat << (alpha_i-X)*cos(theta) - telemeter_decX + (beta_i-Y)*sin(theta),
                       (beta_i-Y)*cos(theta) - telemeter_decY + (X-alpha_i)*sin(theta);
    beacon[i].C << -cos(theta), -sin(theta), (beta_i-Y)*cos(theta) + (X-alpha_i)*sin(theta),
                    sin(theta), -cos(theta), (X-alpha_i)*cos(theta) + (Y-beta_i)*sin(theta);
    mat2 temp;
    temp=beacon[i].C*P*beacon[i].C.transpose()+Q_gamma;
    beacon[i].sigma_maha=temp.inverse();
    beacon[i].K=P*beacon[i].C.transpose()*beacon[i].sigma_maha;
   //Fill data_vector 
   for(int n=0;n<2;n++){
     tab[j]=(float)beacon[i].y_hat(n,0);
     j++;
   }
   //Test distance euclidienne
   //float temp2[4]={1.0,0.0,0.0,1.0};
   //int k=0;
   for(int n=0;n<2;n++){
     for(int m=0;m<2;m++){
       tab[j]=(float)beacon[i].sigma_maha(n,m);
       //tab[j]=temp2[k];
       j++;
       //k++;
     }
   }
  }
  Estimation_Array.data=tab;
  estimation_pub.publish(&Estimation_Array);
}
    
  
void recalerPose(const geometry_msgs::Point32& mesure){
  vect3 correction;
  mat3 eye3;
  int i=int(mesure.z);//numéro de la balise
  beacon[i].y(0,0)=(double)mesure.x;
  beacon[i].y(1,0)=(double)mesure.y;
  correction=beacon[i].K*(beacon[i].y-beacon[i].y_hat);
  eye3 << 1,0,0,
          0,1,0,
          0,0,1;
  X+=correction(0,0);
  Y+=correction(1,0);
  theta+=correction(2,0);
  P=(eye3-beacon[i].K*beacon[i].C)*P;
}

void actualisersigmawheel(const std_msgs::Float32& newsigma){
  sigma_wheels=newsigma.data;
  Q_wheels << sigma_wheels*sigma_wheels,              0          ,
                       0               , sigma_wheels*sigma_wheels;
                   
  JointToCartesian << Rcd/2   , Rcg/2,
                      -Rcd/e  , Rcg/e;
                      
  Q_beta=JointToCartesian*Q_wheels*JointToCartesian.transpose();
}
  
  
geometry_msgs::Pose2D pose;
ros::Publisher posepub("Pose",&pose);
ros::Subscriber<geometry_msgs::Point32>subc("PointCons",&actualiserCons);
ros::Subscriber<geometry_msgs::Point32>subg("Gains",&actualiserGains);
ros::Subscriber<geometry_msgs::Point32>subm("Mesure_Kalman",&recalerPose);
ros::Subscriber<std_msgs::Empty>subr("Request_Estimation",&sendEstimation);
ros::Subscriber<std_msgs::Float32>subsw("Sigma_Wheels",&actualisersigmawheel);


//------------------------------SETUP------------------------------

void setup(){
  //Localisation Variables Initialization
  
  Q_wheels << sigma_wheels*sigma_wheels,              0          ,
                       0               , sigma_wheels*sigma_wheels;
                   
  JointToCartesian << Rcd/2   , Rcg/2,
                      -Rcd/e  , Rcg/e;
                      
  Q_beta=JointToCartesian*Q_wheels*JointToCartesian.transpose();
  
  Q_gamma << sigma_mesure*sigma_mesure,               0         ,
                  0                   ,sigma_mesure*sigma_mesure;
                  
  P << sigmaX*sigmaX,     0 ,        0,
        0    , sigmaY*sigmaY,        0,
        0    ,    0  ,  sigmaTheta*sigmaTheta;     
    
  beacon[0].map_frame_coords << largeur/2,0;
  beacon[1].map_frame_coords << 0,longueur;
  beacon[2].map_frame_coords << largeur,longueur;

  /*
  beacon[0].map_frame_coords << 0,longueur/4;
  beacon[1].map_frame_coords << largeur/2,longueur/2;
  beacon[2].map_frame_coords << largeur,longueur/2;
  */

  Estimation_Array.layout.dim_length=0;
  Estimation_Array.data_length=18;
  
  //Set Pins
  pinMode(motord,OUTPUT);
  pinMode(motorg,OUTPUT);
  pinMode(enabled,OUTPUT);
  pinMode(enableg,OUTPUT);
  pinMode(LED,OUTPUT);
  //PWM Configuration
  analogWriteFrequency(3,23437);//PWM (MLI) à 23kHz.cf www.pjrc.com. Attention ! L298 (fréquence max H-bridge: 40kHz).
  analogWriteResolution(11);//11 bits resolution : 0 to 2047
  //Encoder Initialization
  RightEnc.write(0);
  LeftEnc.write(0);
  #ifdef DEBUG_MODE
  //Classic Serial Initialization
  Serial.begin(9600);
  #else
  //ROS Interface Initialization
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(subc);
  nh.subscribe(subg);
  nh.subscribe(subm);
  nh.subscribe(subr);
  nh.subscribe(subsw);
  nh.advertise(estimation_pub);
  nh.advertise(posepub);
  #endif
  //Timer Initialization
  Timer3.initialize(periode);//in µs
  Timer3.attachInterrupt(isr);
  //Status LED
  digitalWrite(LED,HIGH);
}


//--------------------------------LOOP----------------------------------

void loop(){
  odometry();
  #ifndef DEBUG_MODE
  if(wait_counter>10){//on publie la pose tous les 50ms donc à 20Hz
    pose.x=X;
    pose.y=Y;
    pose.theta=theta;
    posepub.publish(&pose);
    wait_counter=0;
  }
  wait_counter++;
  nh.spinOnce();
  #endif
  //Fréquence odométrie : 200Hz
  delay(5);//Ne pas utiliser le Timer 0 : ça modifierait les temporisations (delay(),ROS,etc..)
}

//--------------------FONCTIONS AUXILIAIRES ET ROUTINES INTERRUPTIONS-----------------------


//Routine d'interruption 
void isr(void){
  tickR=RightEnc.read();
  tickL=LeftEnc.read();
  if(mode==1){
    isr1();
  }else{
    isr0();
  }
  tickRprec=tickR;
  tickLprec=tickL;
}

//Asservissement haut niveau pour le mode 0 -tourne 10 fois moins vite que le haut niveau
void isrh(void){
  double erreurX=Xcons-(X+a*cos(theta));
  double erreurY=Ycons-(Y+a*sin(theta));
  double v1=kpe*erreurX;
  double v2=kpe*erreurY;
  double Vcons=cos(theta)*v1+sin(theta)*v2;
  double Omegacons=(-sin(theta)*v1+cos(theta)*v2)/a;
  omegacg=(Vcons+L*Omegacons)/Rcg;
  omegacd=(Vcons-L*Omegacons)/Rcd;
}


//Asservissement bas niveau pour le mode 0
void isr0(void){//
  if(cnt>=10){
    isrh();
    cnt=0;
  }else{
    cnt++;
  }
  derivr_buff.push(tickR-tickRprec);
  derivl_buff.push(tickL-tickLprec);
  float filt_speedR=derivr_buff.sliding_mean();
  float filt_speedL=derivl_buff.sliding_mean();
  double omegad=1000000*((double)filt_speedR)*2*PI/(npas*periode);
  double omegag=1000000*((double)filt_speedL)*2*PI/(npas*periode);
  //double omegad=1000000*(tickR-tickRprec)*2*PI/(npas*periode);
  //double omegag=1000000*(tickL-tickLprec)*2*PI/(npas*periode);
  double sigmaR=omegad-omegacd;//Sliding mode variables
  double sigmaL=omegag-omegacg;
  int cmdR=int(2047.0*sigmaR/(abs(sigmaR)+epsilon));//Sigmoide function ->avoid chattering
  int cmdL=int(2047.0*sigmaL/(abs(sigmaL)+epsilon));
  if(cmdR>=0){//Normalisation
    if(cmdR>2047) cmdR=2047;
    digitalWrite(motord,HIGH);
    analogWrite(enabled,cmdR);
  }else{
    if(cmdR<-2047) cmdR=-2047;
    digitalWrite(motord,LOW);
    analogWrite(enabled,-cmdR);
  }
  if(cmdL>=0){
    if(cmdL>2047) cmdL=2047;
    digitalWrite(motorg,HIGH);
    analogWrite(enableg,cmdL);
  }else{
    if(cmdL<-2047) cmdL=-2047;
    digitalWrite(motorg,LOW);
    analogWrite(enableg,-cmdL);
  }
}


//Asservissement pour le mode 1. Les consignes sont un incrément sur la distance parcourue et l'orientation
void isr1(void){
  //Calcul des consignes pour chaque roue
  double posd = 0.5*(2.0*cons_dist - e*cons_or*PI/180.0);
  consR = int(posd*npas/(Rcd*2*PI));//consigne en points codeurs roue droite
  double posg= 0.5*(2.0*cons_dist + e*cons_or*PI/180.0);
  consL = int(posg*npas/(Rcg*2*PI));//consigne en points codeurs roue gauche
  //Asservissement PID Polaire
  int erreurR=consR-tickR;
  int erreurL=consL-tickL;
  int delta_erreurR=(-tickR+tickRprec);
  int delta_erreurL=(-tickL+tickLprec);
  derivl_buff.push(delta_erreurL);
  derivr_buff.push(delta_erreurR);
  float filt_delta_erreurR=derivr_buff.sliding_mean();
  float filt_delta_erreurL=derivl_buff.sliding_mean();
  int cmdR= (int)(kp*erreurR + kd*filt_delta_erreurR/periode_asserv_seconde+ ki*sumerreurR*periode_asserv_seconde);
  int cmdL= (int)(kp*erreurL + kd*filt_delta_erreurL/periode_asserv_seconde+ ki*sumerreurL*periode_asserv_seconde);
  sumerreurR+=erreurR;
  sumerreurL+=erreurL;
  if(cmdR>=0){//Normalisation
    if(cmdR>2047) cmdR=2047;
    digitalWrite(motord,LOW);
    analogWrite(enabled,cmdR);
  }else{
    if(cmdR<-2047) cmdR=-2047;
    digitalWrite(motord,HIGH);
    analogWrite(enabled,-cmdR);
  }
  if(cmdL>=0){
    if(cmdL>2047) cmdL=2047;
    digitalWrite(motorg,LOW);
    analogWrite(enableg,cmdL);
  }else{
    if(cmdL<-2047) cmdL=-2047;
    digitalWrite(motorg,HIGH);
    analogWrite(enableg,-cmdL);
  }
}

void odometry(void){
  tickR_odo=RightEnc.read();
  tickL_odo=LeftEnc.read();
  double delta_dist=0.5*((tickR_odo-tickRprec_odo)*Rcd +(tickL_odo-tickLprec_odo)*Rcg)*2*PI/npas;
  double delta_theta=-((tickR_odo-tickRprec_odo)*Rcd -(tickL_odo-tickLprec_odo)*Rcg)*2*PI/(npas*e);
  double dX=delta_dist*cos(theta);
  double dY=delta_dist*sin(theta);
  dist += delta_dist;
  theta += delta_theta;
  X += dX;
  Y += dY;
  //Refresh Covariances Matrix :error propagation
  mat3 A;
  matrix<3,2> B;
  A << 1, 0 , -delta_dist*sin(theta),
       0, 1 , delta_dist*cos(theta) ,
       0, 0 ,          1            ;
  B << cos(theta),0,
       sin(theta),0,
           0     ,1;
  P=A*P*A.transpose();
  P=P+B*Q_beta*B.transpose();  
  #ifdef DEBUG_MODE
  disp(P);
  #endif
  tickRprec_odo=tickR_odo;
  tickLprec_odo=tickL_odo;
}


  

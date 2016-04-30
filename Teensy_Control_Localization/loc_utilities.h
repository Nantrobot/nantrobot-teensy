#ifndef LOC_UTILITIES_H
#define LOC_UTILITIES_H
#ifndef Pi
#define Pi 3.1415927
#endif
#include"matrix.h"

typedef matrix<2, 2> mat2;
typedef matrix<3, 3> mat3;
typedef matrix<2, 1> vect2;
typedef matrix<3, 1> vect3;

typedef struct beacon_data { //Container for the data related to a beacon
  vect2 map_frame_coords;//known from the beginning,fixed
  vect2 y_hat;//estimated coordinates in the range finder frame
  vect2 y;//known from an incoming mesure,coordinates in the range finder frame
  matrix <2, 3> C;
  mat2 sigma_maha;//refreshed after an incoming message and a kalman iteration
  matrix<3, 2> K; //Gain Matrix
} beacon_data;

/*
  Algorithme-Méthode
  Remarque:
  Triangulation: estimation de la position et de l'angle du robot avec trois balises et des mesures d'angles.
  Trilatération : estimation de la position seulement du robot avec deux balises et des mesures de distance (trois redondantes)

  Si on a des mesures d'angles et de distance dans le repère capteur (cas implémenté sur le robot) on peut estimer la pose 2D avec deux balises seulement.
  Le télémètre nous permets de trouver des "clusters" de points dans le plan que l'on associera au balises. On a accès aux données d'angle et de distance cad
  que les coordonnées des balises dans le repère capteur peuvent être estimées précisément.
  -On commence par estimer la position des balises dans le plan par les données de l'odométrie.
  -Quand un message contenant des données laser arrive, on calcule les coordonnées de chaque point du nuage et la distance de mahalanobis au coordonnées estimées d
  des balises.
  -On stocke dans trois vecteurs les clusters associés aux balises 1,2 et 3.
  -On estime les coordonnées des balises à partir de chaque cluster, par exemple en prenant le barycentre.

  Recalage direct : Il s'agit alors de calculer la pose du robot,connaissant les coordonnées des balises dans le repère du terrain (fixe) et dans le repère mobile du capteur
  Les notations suivantes sont en format Matlab (calculs symboliques effectués sur Matlab)
  -On a 0Tm=
  [ cos(theta), -sin(theta), x]
  [ sin(theta),  cos(theta), y]
  [          0,           0, 1]
  -On a mTs=
  [ 1, 0, a]
  [ 0, 1, b]
  [ 0, 0, 1]
  En notant (a,b) les coordonnées de l'Hokuyo dans le repère robot dont l'origine est le milieu de l'axe des roues
  -Soit oTs=
  [ cos(theta), -sin(theta), x + a*cos(theta) - b*sin(theta)]
  [ sin(theta),  cos(theta), y + b*cos(theta) + a*sin(theta)]
  [          0,           0,                               1]
  -Donc en notant (alpha_i;beta_i) les coordonnées connues des balises dans le repère map (fixe) et (Xi;Yi) celles calculées dans le repère Hokuyo, on a :
  sTo*(alpha_i;beta_i;1)=(Xi;Yi;1)
  Avec sTo=
  [  cos(theta), sin(theta), - a - x*cos(theta) - y*sin(theta)]
  [ -sin(theta), cos(theta),   x*sin(theta) - y*cos(theta) - b]
  [           0,          0,                                 1]
  -On obtient donc deux équations :
  alpha_i*cos(theta) - a + beta_i*sin(theta) - x*cos(theta) - y*sin(theta)=Xi
  beta_i*cos(theta) - b - alpha_i*sin(theta) - y*cos(theta) + x*sin(theta)=Yi
  Que l'on peut reformuler en (forme finale)
  alpha_i*cos(theta)+beta_i*sin(theta)-Xi = a+x*cos(theta)+y*sin(theta)
  beta_i*cos(theta)-alpha_i*sin(theta)-Yi = b+y*cos(theta)-x*sin(theta)
  On remarque que les termes de droites ne dépendent pas de la balise i.Il s'agit de déterminer trois inconnues x,y et theta.
  Il suffit donc d'une deuxième balise pour résoudre le système.
  En effet:
  alpha_i*cos(theta)+beta_i*sin(theta)-Xi = alpha_j*cos(theta)+beta_j*sin(theta)-Xj
  beta_i*cos(theta)-alpha_i*sin(theta)-Yi = beta_j*cos(theta)-alpha_j*sin(theta)-Yj
  On résoud le système de deux équations en cos(theta) et sin (theta) et on a theta avec atan2:

  [(alpha_i-alpha_j)     (beta_i-beta_j)]  *  [cos(theta):sin(theta)] = [Xi-Xj;Yi-Yj]
  [(beta_i-beta_j)    -(alpha_i-alpha_j)]

  Déterminant égal à -((alpha_i-alpha_j)^2 + (beta_i-beta_j)^2) jamais nul si on a pris deux balises différentes !

  -Par la suite, connaissant theta :
  A= x*cos(theta)+y*sin(theta)
  B= y*cos(theta)-x*sin(theta)
  avec A= alpha_i*cos(theta)+beta_i*sin(theta)-Xi -a
      B=beta_i*cos(theta)-alpha_i*sin(theta)-Yi -b
  soit [cos(theta)   sin(theta] * [x;y]  = [A;B]
      [-sin(theta) cos(theta)]

  Déterminant égal à 1 -> toujours inversible.

  -On a x,y,theta et on peut dans un premier temps s'arrêter là,faire tourner cet algorithme en boucle rapidement et recaler directement l'odométrie avec
  ces nouvelles valeurs sans filtre de Kalman.
  La matrice de covariance pour la distance de Mahalanobis  reste à sa valeur d'initialisation (diagonale)

  Filtre de Kalman: cf .ino
*/

#endif

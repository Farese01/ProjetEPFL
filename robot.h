#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdio.h>

#include "utilitaire.h"


typedef struct robot ROBOT;


// ensemble des fonctions exportées

void 	robot_p_new(double vrot, double DELTA_D);

void 	robot_move_struct(double vrot, double DELTA_D, C2D struc);

void 	robot_tesst_collisionR_R();

int 	robot_live_collisionR_R(double DELTA_D, int j, double vrot);

void	robot_decode_ligne(int i,char* tab);

int 	robot_lecture_fichier (FILE *fentree);

long int 	robot_get_curseur(FILE* fichier);

void 	robot_new_coo(int i, double vrot, double DELTA_D);

void 	robot_new_coo2(int i, double x, double y, double alpha);

void	robot_dessine_robot();

C2D 	robot_retour_struct_robot(int i);

int 	robot_retour_nb_robot();

void    robot_save_current_state(FILE* save);

void 	robot_free();

int 	robot_get_line_number();

double 	robot_retour_alpha ();

void	robot_remplace_robot(int i,int j);

void 	robot_change_etat1(int i);

int 	robot_etat_robot (int i ) ;

void 	robot_change_etat0(int i );

void 	robot_change_p(int i);

C2D 	robot_return_p();

double 	robot_return_alpha();

void 	robot_change_selec1(int i);

void 	robot_change_selec0(int i);

int	 	robot_retour_selec(int i );


#endif

#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdio.h>
#include <stdlib.h>
#include "utilitaire.h"

// ensemble des fonctions exportées 

int 	simulation_live_collision_R_P(int i, double DELTA_D, 
														   double vrot);

int 	simulation_test_collision_R_P();

int 	simulation_lecture_fichier(FILE* fichier);

void 	simulation_dessine();

void 	simulation_init_decomposition();

void	simulation_move_robot();

void 	simulation_save_current_state(FILE* save);

void 	simulation_free_and_clear();

double  simulation_decontamination(FILE* fichier);

void 	simulation_init_selec();

void 	simulation_verif_souris(double x,double y );

void 	simulation_robot_new_coo(double vrota,double vtrans);

#endif


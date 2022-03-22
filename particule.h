#ifndef PARTICULE_H
#define PARTICULE_H

#include <stdio.h>
#include <stdlib.h>
#include "utilitaire.h"


typedef struct particule PART;

void    particule_tri_particules();

void 	particule_test_collisionP_P();

void 	particule_decode_ligne(int i, char* tab, int line_number);

int 	particule_lecture_fichier(FILE* fichier,long int curseur,
														int line_robot);

void 	particule_dessine_initiale();

void	particule_decomposition1();

double  particule_decontamination();

void 	particule_detruit(int i);

C2D 	particule_retour_struct_particules(int i);

int 	particule_retour_nb_particules();

int		particule_retour_max();

double  particule_get_rayon( int i);

void 	particule_save_current_state(FILE* save);

int 	particule_existe(int i);

void 	particule_free();

#endif

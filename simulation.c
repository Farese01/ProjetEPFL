
/**
 * \file		simulation.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
 //				Inclusion de fichiers en-tête
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glu.h>

#include "error.h"
#include "constantes.h"
#include "utilitaire.h"
#include "robot.h"
#include "graphic.h"
#include "particule.h"

static int truth = 1;
static int truth2 = 1;

//*********************************************************************
//				Ensemble des fonctions du module simulation

int simulation_live_collision_R_P2(C2D struct_par, double DELTA_D,
								  double L, int i, int j, int collision)
{
	double p_dist, la_new, alignement, vrot2;
	int h;
	if(util_collision_cercle(struct_par, robot_return_p(), 
		&p_dist) && !util_inner_triangle(fabs(DELTA_D), p_dist, L,
		(R_ROBOT+struct_par.rayon), &la_new))
	{
		util_ecart_angle(robot_retour_struct_robot(i).centre,
		robot_retour_alpha(i),struct_par.centre, &alignement);
		h = util_alignement(robot_retour_struct_robot(i).centre, 
		robot_retour_alpha(i), struct_par.centre);
		if(h)
		{
			particule_detruit(j);
			particule_tri_particules();
			robot_change_etat0(i);
		}
		else
		{
			vrot2= (alignement/DELTA_T);
			if (vrot2 > VROT_MAX)
				vrot2 = VROT_MAX;
			else if (vrot2 < -VROT_MAX)
				vrot2 = -VROT_MAX; 
			robot_new_coo(i, vrot2, 0);
			collision = 1;
		}
	}
	else if(util_collision_cercle(struct_par, 
			robot_retour_struct_robot(i), &p_dist))
	{
		util_ecart_angle(robot_retour_struct_robot(i).centre,
		robot_retour_alpha(i),struct_par.centre, &alignement);
		h = util_alignement(robot_retour_struct_robot(i).centre, 
		robot_retour_alpha(i), struct_par.centre);
		if(h)
		{
			particule_detruit(j);
			particule_tri_particules();
			robot_change_etat0(i);
		}
		else
		{
			vrot2= (alignement/DELTA_T);
			if (vrot2 > VROT_MAX)
			vrot2 = VROT_MAX;
			else if (vrot2 < -VROT_MAX)
			vrot2 = -VROT_MAX; 
			robot_new_coo(i, vrot2, 0);
			collision = 1;
		}
	}
	return collision;
}

int simulation_live_collision_R_P(int i, double DELTA_D, double vrot)
{
	int collision = 0;
	unsigned int j ;
	double p_dist;
	int nb_particules_sim;
	C2D struct_par;
	double alignement;
	double vrot2, L, la_new, vtran;
	int h;
	nb_particules_sim = particule_retour_nb_particules();
	robot_change_p(i);
	robot_p_new(vrot, DELTA_D);
	for( j = 0; j < nb_particules_sim; j++)
	{
		if(particule_existe(j) && !collision)
		{
			struct_par = particule_retour_struct_particules(j);
			L = util_distance(robot_retour_struct_robot(i).centre, 
			struct_par.centre);
			if(util_collision_cercle(struct_par, robot_return_p(), 
			   &p_dist) && util_inner_triangle(fabs(DELTA_D), p_dist, L,
			   (R_ROBOT+struct_par.rayon), &la_new))
			{
				vtran = (la_new/DELTA_T);
				if (vtran > VTRAN_MAX )	
					vtran = VTRAN_MAX;	
				else if (vtran < (-VTRAN_MAX) )
					vtran = -VTRAN_MAX;
				la_new = vtran*DELTA_T;
				robot_new_coo(i, 0, la_new);
				util_ecart_angle(robot_retour_struct_robot(i).centre,
				robot_retour_alpha(i),struct_par.centre, &alignement);
				h = util_alignement(robot_retour_struct_robot(i).centre, 
						robot_retour_alpha(i), struct_par.centre);
				if(h)
				{
					particule_detruit(j);
					robot_change_etat0(i);
					particule_tri_particules();
				}
				else
				{
					vrot2 = (alignement/DELTA_T);
					if (vrot2 > VROT_MAX)
						vrot2 = VROT_MAX;
					else if (vrot2 < -VROT_MAX)
						vrot2 = -VROT_MAX; 
					robot_new_coo(i, vrot2, 0);
					collision = 1;
				}
			}
			collision = 
			simulation_live_collision_R_P2(struct_par, DELTA_D, L, i,
			j, collision);
		}
	}
	return collision;
}			

void simulation_init_robot()
{	
	int i;
	for (i=0 ;i<robot_retour_nb_robot(); i++)
		robot_change_etat0(i);
}
void simulation_init_selec()
{
	int i ;
	for (i=0 ;i<robot_retour_nb_robot(); i++)
		robot_change_selec0(i);
	
}

int simulation_test_collision_R_P()
{
	unsigned int i,j ;
	double p_dist;
	int nb_robot_sim;
	int nb_particules_sim;
	C2D struct_rob ;
	C2D struct_par ;
	
	nb_robot_sim = robot_retour_nb_robot();
	nb_particules_sim = particule_retour_nb_particules();
	
	for( i = 0; i < nb_robot_sim ; i++)
	{		
		struct_rob = robot_retour_struct_robot(i);
		for( j = 0; j < nb_particules_sim; j++)
		{
			struct_par = particule_retour_struct_particules(j);
			if(j != i && 
			   util_collision_cercle(struct_par, struct_rob, &p_dist))
			{
				error_collision(ROBOT_PARTICULE, (j+1), (i+1));
				return 0;
			}
		}
	}
	return 1;
}

int simulation_lecture_fichier(FILE* fichier)
{
	int line_number;
	long int curseur = 0;
	if(robot_lecture_fichier( fichier))
	{
		curseur = robot_get_curseur(fichier);
		line_number = robot_get_line_number();
		if(particule_lecture_fichier(fichier, curseur, line_number))
		{	
			if(simulation_test_collision_R_P())
				{
					error_no_error_in_this_file();
					return 1;
				}
			else
				return 0;
		}
		else
			return 0;
	}	
	else
		return 0;
}


void simulation_dessine()
{	int i ;
	glColor3f(0.5, 0.5, 0.5);
	particule_dessine_initiale();
	for (i=0 ; i< robot_retour_nb_robot();i++)
	{
		if (robot_retour_selec(i)==0)
		{
			glColor3f(0.0,0.0,0.0);
			robot_dessine_robot(i);
		}
		else if(robot_retour_selec(i)==1)
		{
			glColor3f(1.0,0.0,0.0);
			robot_dessine_robot(i);
		}
	}
}


void simulation_init_decomposition()
{
	particule_decomposition1();
}

void simulation_move_robot()
{
	double vrot, vtran, DELTA_D, D;
	int i;
	double *ecart_angle = NULL;
	double ka;
	ecart_angle = &ka;
	int nb_robot = robot_retour_nb_robot() ;
	if(truth)
	{
		for(i = 0; i < nb_robot; i++)
			simulation_init_robot(i);
	}
	truth = 0;
	truth2 = 0;
	for(i = 0; i < robot_retour_nb_robot(); i++)
	{
		//simulation_tri_robot(i);
		if(particule_existe(i) && robot_retour_selec(i)==0)
		{
			util_ecart_angle(robot_retour_struct_robot(i).centre,
			robot_retour_alpha(i), 
			particule_retour_struct_particules(i).centre, ecart_angle);
			if (fabs(*ecart_angle) >  (M_PI/2)) 
				vtran = 0;
			else if (fabs(*ecart_angle)<(M_PI/2)) 
			{
				D = util_distance(robot_retour_struct_robot(i).centre,
				particule_retour_struct_particules(i).centre);
				if( D < (R_ROBOT + 
				particule_retour_struct_particules(i).rayon 
				- EPSIL_ZERO)) 
					D = 0;
				vtran = (D/DELTA_T);
				if (vtran >VTRAN_MAX )	
					vtran = VTRAN_MAX;	
				else if (vtran < -VTRAN_MAX )
					vtran = -VTRAN_MAX;
			}
			DELTA_D = vtran * DELTA_T;
			vrot= *ecart_angle/DELTA_T;
			if (vrot > VROT_MAX)
				vrot = VROT_MAX;
			else if (vrot < -VROT_MAX)
				vrot = -VROT_MAX;
			if(!robot_live_collisionR_R(DELTA_D, i, vrot)
				&&!simulation_live_collision_R_P(i, DELTA_D, vrot))
					robot_new_coo(i,vrot,DELTA_D);			
		}
	}
}

void simulation_save_current_state(FILE* save)
{
	robot_save_current_state(save);
	particule_save_current_state(save);
}

void simulation_free_and_clear()
{
	particule_free();
	robot_free();
}

double simulation_decontamination(FILE* fichier)
{
	return particule_decontamination(fichier);
}
void simulation_verif_souris(double x,double y )
{
	int i  ;
	
	S2D souris;
	souris.x = x; 
	souris.y = y;
	simulation_init_selec();
	for (i=0 ; i<robot_retour_nb_robot() ; i++)
	{
		if (util_point_dans_cercle(souris , 
		robot_retour_struct_robot(i))==1)
		{
			robot_change_selec1(i);
		}
		robot_change_etat0(i);	
	}	
}

void simulation_robot_new_coo(double vrota,double vtrans)
{
	double DELTA_D;
	int i;
	for (i=0;i<robot_retour_nb_robot();i++)
	{
	if (robot_retour_selec(i)==1)
	{
		DELTA_D= vtrans * DELTA_T;
		if(!robot_live_collisionR_R(DELTA_D, i, vrota)
				&&!simulation_live_collision_R_P(i, DELTA_D, vrota))
		robot_new_coo(i,vrota,DELTA_D);
	}
	}
}



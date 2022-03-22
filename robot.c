/**
 * \file		robot.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
//				Inclusion de fichiers en-tête
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "error.h"
#include "utilitaire.h"
#include "constantes.h"
#include "robot.h"
#include "tolerance.h"

//
// Types opaques du module robot
//

struct robot 
{		
		C2D cercle;
		double alpha ;
		int etat;
		int selec;
};
//*********************************************************************
//				Définition des constantes symboliques

#define NB_ARG_ROBOT 	 3
#define LINE_NB_ROBOT 	-1
#define ERROR			 0
#define NO_ERROR		 1
#define COLLISION		 1
#define NO_COLLISION     0
#define OCCUPE			 1
#define NON_OCCUPE		 0
#define SELECT			 1
#define NO_SELECT		 0

// Déclaration des variables static

static int line_number =0 ;
static int taille = 0 ;
static ROBOT* robots = NULL;
static int nb_robot;
static int verif;
static ROBOT p;

//*********************************************************************
//				Ensemble des fonctions du module robot

void robot_p_new(double vrot, double DELTA_D)
{ 
	p.alpha +=  (vrot*DELTA_T);
	util_range_angle(&p.alpha);
	p.cercle.centre = util_deplacement(p.cercle.centre ,
	p.alpha,DELTA_D);		
}

void robot_tesst_collisionR_R()
{	
	unsigned int j,l;
	double p_dist;
	
	for( l = 0; l < nb_robot; l++)
	{	
		for( j = 0; j < nb_robot; j++)
		{
			if(j != l && util_collision_cercle(robots[l].cercle,
										robots[j].cercle, &p_dist))
			{
				error_collision(ROBOT_ROBOT, (j+1), (l+1));
				verif = ERROR;
			}	
		}
	}
}

int robot_live_collisionR_R(double DELTA_D, int j, double vrot)
{	
	int collision = NO_COLLISION;
	unsigned int l;
	double p_dist, L, la_new, vtran;
	p.alpha = robots[j].alpha;
	p.cercle.centre.x = robots[j].cercle.centre.x;
	p.cercle.centre.y = robots[j].cercle.centre.y;
	p.cercle.rayon = R_ROBOT;
	robot_p_new(vrot, DELTA_D);	
	for( l = 0; l < nb_robot; l++)
	{	
		if(collision)
			robot_new_coo(j, 0, 0);
		else
		{
			L = util_distance(robots[j].cercle.centre,
			robots[l].cercle.centre);
			if(j != l && util_collision_cercle(p.cercle,
			robots[l].cercle, &p_dist) && util_inner_triangle
			(fabs(DELTA_D), p_dist, L, (R_ROBOT*2), &la_new) )
			{
				vtran = (la_new/DELTA_T);
				if (vtran >VTRAN_MAX )	
					vtran = VTRAN_MAX;	
				else if (vtran < -VTRAN_MAX )
					vtran = -VTRAN_MAX;
				la_new = vtran*DELTA_T;
				robot_new_coo(j, 0, la_new);
				collision = COLLISION;
			}
			else if(j != l && util_collision_cercle(p.cercle,
			robots[l].cercle, &p_dist) && !util_inner_triangle
			(fabs(DELTA_D), p_dist, L, (R_ROBOT*2), 
			&la_new))
			{
				robot_new_coo(j, (VROT_MAX/3), 0);
				collision = COLLISION;
			}	
		}
	}
	return collision;
}

void robot_decode_ligne(int i,char* tab)
{	
	char* test = tab;
	char** test2 = &tab ;
	 
	if (i == LINE_NB_ROBOT)
	{
		sscanf(tab,"%d", &nb_robot);
		robots = malloc((sizeof(ROBOT)*nb_robot));		
	}
	else if (verif)
	{	
		do
		{
			if(taille >= (NB_ARG_ROBOT*nb_robot))
			{	
				error_missing_fin_liste_robots (line_number);
				verif = ERROR;
			}
			else
			{	
				test = *test2;
				robots[i].cercle.centre.x = strtod(test, test2);
				test = *test2;
				robots[i].cercle.centre.y = strtod(test, test2);
				test = *test2;
				robots[i].alpha = strtod(test, test2);
				robots[i].cercle.rayon = R_ROBOT;	
				if (util_alpha_dehors(robots[i].alpha))
				{
					error_invalid_robot_angle(robots[i].alpha);
					verif = ERROR;
				}
				i++;
				taille += NB_ARG_ROBOT;
			}
		}while( test == *test2 && verif && i < nb_robot);
	}
}		

int robot_lecture_fichier (FILE *fentree)
{	
	taille = 0;
	line_number = 0;
	nb_robot = 0;
	verif = NO_ERROR;
	
	fseek(fentree, 0, 0);
	int i = LINE_NB_ROBOT;
	char* pos = NULL ;
	char tab [MAX_LINE];
	char *fin = "FIN_LISTE\n";
	while (strcmp((fgets(tab, MAX_LINE,fentree)) , fin) && verif)
	{ 	
		line_number++;
		pos = strchr(tab ,'#');
		if ( pos != NULL || tab[0] == '\n' || tab[0]=='\r' 
										   || tab[0]=='#')
		{
			continue ;
		}
		else
		{
			robot_decode_ligne(i,tab);
			i++; 
		}
	}
	if(verif)
	{
		if(taille < (NB_ARG_ROBOT*nb_robot))
		{
			error_fin_liste_robots(line_number);
			verif = ERROR;
		}
		else if(taille >= (NB_ARG_ROBOT*nb_robot) )
			robot_tesst_collisionR_R();
		else
			verif = ERROR;
	}
    return verif;
}


long int robot_get_curseur(FILE* fichier)
{
	return ftell(fichier);
}

void robot_new_coo(int i, double vrot, double DELTA_D)
{ 
	robots[i].alpha +=  (vrot*DELTA_T);
	util_range_angle(&robots[i].alpha);
	robots[i].cercle.centre = 
	util_deplacement(robots[i].cercle.centre ,
	robots[i].alpha,DELTA_D);		
}

void robot_new_coo2(int i, double x, double y, double alpha)
{
	robots[i].cercle.centre.x = x;
	robots[i].cercle.centre.y = y;
	robots[i].alpha = alpha;
}

void robot_dessine_robot(int i)
{
	if(robots != NULL)
	{			
		util_dessine_robot(robots[i].cercle.centre.x,
		robots[i].cercle.centre.y, robots[i].alpha);		
	}
}

C2D robot_retour_struct_robot(int i)
{
	return robots[i].cercle;
}

int robot_retour_nb_robot()
{
	return nb_robot;
}

void robot_save_current_state(FILE* save)
{
	if (robots == NULL)
		printf("Aucun état à sauvegarder\n");
	else
	{
		int i;
		fprintf(save, "%d\n", nb_robot);
		for ( i = 0; i < nb_robot ; i++)
		{
			fprintf(save, "%lf %lf %lf\n", robots[i].cercle.centre.x,
							robots[i].cercle.centre.y, robots[i].alpha);
		}
		fprintf(save, "FIN_LISTE\n");
	}
}

void robot_free()
{
	if(robots != NULL)
		free(robots);
	robots = NULL;
}

int robot_get_line_number()
{
	return line_number;
}
double robot_retour_alpha(int i)
{
	return robots[i].alpha;
}

void robot_remplace_robot(int i,int j)	
{	
	ROBOT tmp;
	tmp = robots[i];
	robots[i] = robots[j];
	robots[j] = tmp;
}	
int robot_etat_robot (int i ) 
{	
	return robots[i].etat;
}
void robot_change_etat1(int i )
{
	robots[i].etat = OCCUPE ;
}
void robot_change_etat0(int i )
{ 
	robots[i].etat = NON_OCCUPE ;
}

void robot_change_p(int i)
{
	p.alpha = robots[i].alpha;
	p.cercle.centre.x = robots[i].cercle.centre.x;
	p.cercle.centre.y = robots[i].cercle.centre.y;
	p.cercle.rayon = R_ROBOT;
}

C2D robot_return_p()
{
	return p.cercle;
}

double robot_return_alpha()
{
	return p.alpha;
}
void robot_change_selec1(int i)
{
	robots[i].selec = SELECT ;
}
void robot_change_selec0(int i)
{
	robots[i].selec = NO_SELECT ;
}
int robot_retour_selec(int i )
{
	return robots[i].selec ;
}



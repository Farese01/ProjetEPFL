/**
 * \file		particule.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
//						Inclusion de fichiers en-tête

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "constantes.h"
#include "utilitaire.h"
#include "error.h"
#include "particule.h"
#include <time.h>

//
// Types opaques du module particule
//

struct particule
{
	double energie;
	C2D cercle;
	int etat;
};

//*********************************************************************
//				Définition des constantes symboliques

#define NB_ARG_PARTICULE 	 4
#define LINE_NB_PARTICULE   -1
#define ERROR			 	 0
#define NO_ERROR		 	 1
#define E_PARTICULE_MIN		 0
#define PARTICULE_DECOMP     1
#define PARTICULE_NON_DECOMP 0
#define PARTICULE_DETRUITE   3
#define NO_PARTICULE         2
#define NB_SS_PARTICULE      4
#define LAST_SS_PARTICULE	 3

// Déclaration des variables static

static int nb_particules;
static PART* particules = NULL;
static int taille = 0;
static int verif;
static int truth = 0;
static int all_parts = 0;
static double energie_i = 0;
static double taux = 0;
static double energie_d = 0;

	
//*********************************************************************
//				Ensemble des fonctions du module particule

void particule_tri_particules()
{
	int i, j;
	PART tmp;
	int max_parts = nb_particules*21;
	for(i = 0; i < (max_parts); i++)
	{
		for(j = 0; j < (max_parts); j++)
		{
			if(particules[j].cercle.rayon >= particules[i].cercle.rayon
							&& j > i )
			{
				tmp = particules[i];
				particules[i] = particules[j];
				particules[j] = tmp;
			}
		}
	}
}

void particule_test_collisionP_P()
{
	double p_dist;
	unsigned int i,j;
	for( i = 0; i< nb_particules; i++)
	{
		for( j = 0; j < nb_particules; j++)
		{
			if(j != i && util_collision_cercle(particules[j].cercle,
										particules[i].cercle, &p_dist))
			{
				error_collision(PARTICULE_PARTICULE, (j+1), (i+1));
				verif = ERROR;
			}
		}
	}
}

void particule_init_tableau()
{
	int i;
	int max_parts = nb_particules*21;
	for(i = 0; i < (max_parts); i++)
	{
		particules[i].energie = 0;
		particules[i].cercle.rayon = 0;
		particules[i].etat = NO_PARTICULE;
	}
}
	
void particule_decode_ligne(int i, char* tab, int line_number)
{
	char* test = tab;
	char** test2 = &tab ;
	int max_parts;
	if (i == -1)
	{
		sscanf(tab, "%d", &nb_particules);
		max_parts = nb_particules*21;
		particules = malloc((sizeof(PART)*max_parts));
		particule_init_tableau();
	}
	else if(verif)
	{
		do
		{
			if(taille >= (NB_ARG_PARTICULE*nb_particules))
			{
				error_missing_fin_liste_particules(line_number);
				verif = ERROR;
			}
			else
			{
				test = *test2;
				particules[i].energie = strtod(test, test2);
				energie_i += particules[i].energie;
				test = *test2;
				particules[i].cercle.rayon = strtod(test, test2);
				test = *test2;
				particules[i].cercle.centre.x = strtod(test, test2);
				test = *test2;
				particules[i].cercle.centre.y =  strtod(test, test2);
				particules[i].etat = PARTICULE_NON_DECOMP;
				if( particules[i].energie > E_PARTICULE_MAX ||
					particules[i].energie < E_PARTICULE_MIN ||
					particules[i].cercle.rayon < R_PARTICULE_MIN ||
					particules[i].cercle.rayon > R_PARTICULE_MAX ||
					fabs(particules[i].cercle.centre.x) > DMAX ||
					 fabs(particules[i].cercle.centre.y) > DMAX)
				{
					error_invalid_particule_value(particules[i].energie,
					particules[i].cercle.rayon,
					particules[i].cercle.centre.x,
					particules[i].cercle.centre.y);
					verif = ERROR;
				}
				i++;
				taille += 4;
				all_parts++;
			}
		}while( test == *test2 && verif && i < nb_particules);
	}
}

int particule_lecture_fichier(FILE* fichier, long int curseur,
														int line_robot)
{
	fseek(fichier, curseur, 0);
	nb_particules=0;
	energie_d = 0;
	taille = 0;
	all_parts = 0;
	energie_i = 0;
	verif = NO_ERROR;
	
	int i = LINE_NB_PARTICULE;
	int line_number = line_robot;
	char* pos = NULL;
	char tab[MAX_LINE];
	char *fin = "FIN_LISTE\n";
	while(strcmp((fgets(tab, MAX_LINE, fichier)), fin) && verif)
	{
		line_number++;
		pos = strchr(tab, '#');
		if ( pos != NULL || tab[0] == '\n')
			continue;
		else
		{
			particule_decode_ligne(i, tab, line_number);
			++i;
		}
	
	}
	if(verif)
	{
		if ( taille < NB_ARG_PARTICULE*nb_particules)
		{
			error_fin_liste_particules(line_number);
			verif = ERROR;
		}
		else if(taille >= (NB_ARG_PARTICULE*nb_particules))
			particule_test_collisionP_P();
		else
			verif = ERROR;
	}
	return verif;
}


void particule_dessine_initiale()
{
	if( particules != NULL)
	{
		int i;
		int max_parts = nb_particules*21;
		for (i=0; i < (max_parts); i++)
		{
			if( particules[i].etat == PARTICULE_NON_DECOMP)
			{
				util_dessine_particule(particules[i].cercle.centre.x,
				particules[i].cercle.centre.y,
				particules[i].cercle.rayon);
			}
		}
	}
}

void particule_decomposition1()
{
	int i, j, k;
	double new_R;
	int tab1[NB_SS_PARTICULE] = {1,-1,-1,1};
	int tab2[NB_SS_PARTICULE] = {1, 1, -1, -1};
	if( !truth)
	{
		srand(time(NULL));
		truth = 1;
	}
	j = 0;
	for(i = 0; i < all_parts; i++)
	{
		double p = ((double)rand()/RAND_MAX);
		new_R = particules[i].cercle.rayon*R_PARTICULE_FACTOR;
		for(k = 0; k < NB_SS_PARTICULE; k++)
		{
			if(new_R >= R_PARTICULE_MIN && p <= DECOMPOSITION_RATE &&
					 particules[i].etat == PARTICULE_NON_DECOMP)
				{
					while(particules[j].etat == PARTICULE_NON_DECOMP)
					{
						j++;
					}
					particules[j].cercle.centre.x =
					particules[i].cercle.centre.x + new_R*(tab1[k]);
					particules[j].cercle.centre.y = 
					particules[i].cercle.centre.y + new_R*(tab2[k]);
					particules[j].cercle.rayon = new_R;
					particules[j].energie = 
					particules[i].energie*E_PARTICULE_FACTOR;
					particules[j].etat = PARTICULE_NON_DECOMP;
					all_parts++;
					if( k == LAST_SS_PARTICULE)
					{
						particules[i].etat = PARTICULE_DECOMP;
						particules[i].cercle.rayon = 0;
						all_parts--;
						particule_tri_particules();
					}
				}
			j++;
		}
	}
}

double particule_decontamination(FILE *fichier)
{
	if(fichier == NULL)
		taux = 0;
	else
		taux = ((double)100*(energie_d/energie_i));
	if (taux > 99.9999)
		taux = 0;
	return taux;

}

void particule_detruit(int i)
{
	all_parts--;
	particules[i].etat = PARTICULE_DETRUITE;
	energie_d += particules[i].energie;
	particules[i].cercle.rayon = 0;
	particule_tri_particules();
}

C2D particule_retour_struct_particules(int i)
{
	return particules[i].cercle;
}

int particule_retour_nb_particules()
{
	return all_parts;
}

int particule_retour_max()
{
	int max_parts = nb_particules*21;
	return max_parts;
}

double particule_get_rayon( int i)
{
	return particules[i].cercle.rayon;
}

void particule_save_current_state(FILE* save)
{
	int max_parts = nb_particules*21;
	if( particules == NULL)
		printf( "Aucun état à sauvegarder \n");
	else
	{
		int i;
		fprintf(save, "%d\n", all_parts);
		for ( i = 0; i < max_parts ; i++)
		{
			if( particules[i].etat == PARTICULE_NON_DECOMP)
			{
				fprintf(save, "%lf %lf %lf %lf\n", 
				particules[i].energie,
				particules[i].cercle.rayon,  
				particules[i].cercle.centre.x,
				particules[i].cercle.centre.y);
			}
		}
		fprintf(save, "FIN_LISTE\n");
	}
}

int particule_existe(int i)
{
	if(particules[i].etat == PARTICULE_NON_DECOMP)
		return 1;
	else
		return 0;
}

void particule_free()
{
	if( particules != NULL)
		free(particules);
	particules = NULL;
}

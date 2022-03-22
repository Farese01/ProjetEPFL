/**
 * \file		utilitaire.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
 //				Inclusion de fichiers en-tête

#include "utilitaire.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glu.h>
#include "tolerance.h"
#include "graphic.h"

//*********************************************************************
//				Définition des constantes symboliques

#define MAX_PI		2*M_PI
#define MIN_LB		0

//*********************************************************************
//				Ensemble des fonctions du module utilitaire

	int util_min_tab(double* tab, int taille)
	{
		int i, proche;
		proche = 0;
		for(i=0; i < taille; i++)
		{
			if(tab[i] >= tab[proche])
				proche = i;
		}
		return proche;
	}

	double	util_distance(S2D a, S2D b)
	{
		return (sqrt(((b.x-a.x)*(b.x-a.x))+((b.y-a.y)*(b.y-a.y))));
	}
	
	double	util_angle(S2D a, S2D b)
	{
		return (atan2((b.y-a.y),(b.x-a.x)));
	}
	
	void	util_range_angle(double * p_angle)
	{	
		double n = *p_angle;
		while (n > M_PI)
		{
			n -= (MAX_PI);
		}
		while (n <= (-M_PI))
		{
			n += (MAX_PI);
		}
		*p_angle = n;
	}
	
	bool	util_point_dehors(S2D a, double max)
	{ 	
		if (fabs(a.x) > max || fabs(a.y) > max)
			return true;
		else
			return false;
    }
    
	bool	util_alpha_dehors(double alpha)
	{ 
		if (fabs(alpha) > M_PI)
			return true;
		else
			return false;
	}
	
	bool	util_point_dans_cercle(S2D a, C2D c)
	{      
		double l =  util_distance( a, c.centre );
		if (l < c.rayon-EPSIL_ZERO)
			return true ;
		else
			return false;
	}
	
	bool	util_collision_cercle(C2D a, C2D b, double * p_dist)
	{ 
		*p_dist = util_distance(a.centre, b.centre);
		if (*p_dist < ((a.rayon+b.rayon)-EPSIL_ZERO))
			return true;
		else 
			return false ;
	}
	
	S2D		util_deplacement( S2D p, double alpha, double dist)
	{	
		double dep_x = (dist *  cos( alpha));
		double dep_y = (dist *  sin( alpha));
		(p.x) = (p.x)+dep_x;
		(p.y) = (p.y)+dep_y;
		return p ;
	}

	bool	util_ecart_angle(S2D a, double alpha, S2D b,
							 double *p_ecart_angle)
	{ 
		double d = util_distance(a,b);
		if (d > EPSIL_ZERO)
		{
			*p_ecart_angle = util_angle(a,b) - alpha;
			util_range_angle(p_ecart_angle);
			return true ;
		}
		else 
			return false ;
	}
	
	bool	util_alignement(S2D a, double alpha, S2D b)
	{
		double p_ecart_angle ;
		bool h = util_ecart_angle(a, alpha, b,  &p_ecart_angle );
		if  ( h &&  fabs (p_ecart_angle) < EPSIL_ALIGNEMENT) 
			return true ;
		else 
			return false ;
	}
	
	bool	util_inner_triangle(double la, double lb, double lc, 
	                            double lb_new, double * p_la_new)
	{	
		if (la > EPSIL_ZERO && lb > EPSIL_ZERO && lc > EPSIL_ZERO 
		    && lb <= lb_new && lb_new <= lc)
			{
				double cosb;
				cosb = (la*la + lc*lc -lb*lb)/(2*la*lc);
				*p_la_new = lc*cosb-sqrt(lc*lc*cosb*cosb+
				                         lb_new*lb_new-lc*lc);
				return true;
			}
		else
			return false;
	}

void util_dessine_particule(double xp, double yp, double rayon)
{
		graphic_dessine_cercle(xp, yp, rayon, GRAPHIC_FILLED);
}

void util_dessine_robot(double xr, double yr, double beta)
{
	glLineWidth(2.);
	graphic_dessine_cercle(xr, yr, R_ROBOT, GRAPHIC_EMPTY);
	graphic_dessine_ligne(xr, yr, beta);
	graphic_dessine_rectangle(xr, yr);
}


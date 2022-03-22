/**
 * \file		graphic.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
//						Inclusion de fichiers en-tête

#include <stdio.h>
#include <stdlib.h>
#include <GL/glu.h>
#include "constantes.h"
#include <math.h>

//*********************************************************************
//				Définition des constantes symboliques

#define NB_COTES	50
#define GRAPHIC_EMPTY  0
#define GRAPHIC_FILLED 1
	
//*********************************************************************
//				Ensemble des fonctions du module graphic

void graphic_dessine_rectangle( double xr, double yr)
{
	glColor3f(1.0, 0, 0);
    glBegin (GL_POLYGON);
    
    glVertex2f (xr+0.4/2, yr+0.4/2);
    glVertex2f (xr-0.4/2, yr+0.4/2);
    glVertex2f (xr-0.4/2, yr+0.4/2);
    glVertex2f (xr+0.4/2, yr+0.4/2);

    glEnd ();
    glColor3f(0.0,0.0,0.0);
}

void graphic_dessine_ligne(double xr, double yr, double beta)
{
	double c = xr + R_ROBOT*cos(beta);
	double d = yr + R_ROBOT*sin(beta);
	
	glBegin (GL_LINES);
	
    glVertex2f (xr, yr);
    glVertex2f (c, d);

    glEnd ();
	
}

void graphic_dessine_cercle(double xp,double yp,double rayon,int filled)
{
	int i;
	double alpha;
	
	if (filled == GRAPHIC_FILLED)
		glBegin (GL_POLYGON);
    else
		glBegin (GL_LINE_LOOP);
		
	for (i=0; i < NB_COTES; i++)
    {
      double alpha = i * 2. * M_PI / NB_COTES;
      double a = xp+rayon*cos(alpha);
      double b = yp + rayon*sin(alpha);
      glVertex2f(a, b);
    }
    
    glEnd();	
}





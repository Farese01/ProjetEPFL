#ifndef GRAPHIC_H
#define GRAPHIC_H

#include <stdbool.h>
#include "constantes.h"

#define GRAPHIC_EMPTY  0
#define GRAPHIC_FILLED 1

void 	graphic_dessine_rectangle( double xr, double yr);

void	graphic_dessine_ligne(double xr, double yr, double beta);

void	graphic_dessine_cercle(double xp, double yp, double rayon, int filled);

#endif

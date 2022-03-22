# Compilateur a utiliser :
##########################

CC     = gcc

# Options de compilation : (-g = pour deverminer)
#################################################

CFLAGS = -g


# Librairies a utiliser :
# LIBS: toutes les librairies qu'il faut intégrer
#########################

LIBS   = -lstdc++ -lglut -lGL -lGLU -lm -Llib -lglui -L/usr/X11R6/lib -lX11 -lXext -lXmu -lXi

# Liste de fichiers source (.c) a compiler :
############################################

CFILES = main.cpp simulation.c particule.c robot.c utilitaire.c graphic.c error.c

# Liste de modules objets (generee a partir de CFILES) :
# Les fichiers objets (.o) ont le même nom que les fichiers source (.c)
# Seulement le suffixe change.
########################################################

OFILES = $(CFILES:.c=.o)

# Nom du fichier executable :
#############################

CIBLE  = projet.x

# edition de liens (rassembler tous les objets -> executable)
#############################################################

glui.x: $(OFILES) 
	$(CC) $(OFILES) -o $(CIBLE) ${LIBS}


# compilation separee (production du module objet seulement)

 main.o: main.cpp 
	$(CC) -c main.cpp $(CFLAGS)

simulation.o: simulation.c
	$(CC) -c simulation.c $(CFLAGS)

robot.o : robot.c
	$(CC) -c robot.c $(CFLAGS)

particule.o : particule.c
	$(CC) -c particule.c $(CFLAGS)

utilitaire.o : utilitaire.c
	$(CC) -c utilitaire.c $(CFLAGS)

error.o: error.c
	$(CC) -c error.c $(CFLAGS)

graphic.o : graphic.c
	$(CC) -c graphic.c $(CFLAGS)




# Definitions de cibles particulieres :
#
# "make depend" : genere la liste des dependances
# "make clean"  : efface les fichiers .o et .x
#################################################

depend:
	@echo " *** MISE A JOUR DES DEPENDANCES ***"
	@(sed '/^# DO NOT DELETE THIS LINE/q' Makefile && \
	  $(CC) -MM $(CFLAGS) $(CFILES) | \
	  egrep -v "/usr/include" \
	 ) >Makefile.new
	@mv Makefile.new Makefile

clean:
	@echo " *** EFFACE MODULES OBJET ET EXECUTABLE ***"
	@/bin/rm -f *.o *.x *.c~ *.h~ *.h.gch

#
# -- Regles de dependances generees par "make depend"
#####################################################
# DO NOT DELETE THIS LINE
graphic.o : graphic.c graphic.h
error.o: error.c error.h constantes.h
utilitaire.o : utilitaire.c utilitaire.h graphic.h constantes.h tolerance.h
particule.o : particule.c particule.h utilitaire.h constantes.h error.h
robot.o : robot.c robot.h utilitaire.h error.h constantes.h
simulation.o : simulation.h simulation.c robot.h particule.h utilitaire.h error.h constantes.h
main.o : main.cpp simulation.h graphic.h constantes.h

/**
 * \file		main.cpp
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
//						Inclusion de fichiers en-tête

#include <stdlib.h>
#include <string.h>
#include <GL/glut.h>
#include <GL/glui.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 

 extern   "C"  {
    #include	"graphic.h"
    #include	"simulation.h"
}

// Déclaration des variables static

static int verif;
static FILE* fichier;
static int start;
static double rate = 0;
static int turn = 0;
static FILE* plot;
static  char vitran[MAX_LINE];
static  char vitrot[MAX_LINE];
static  float vtrans ;
static float vrota;
static	char tab[MAX_LINE];
static	char tab2[MAX_LINE];

namespace 
{
	int text[MAX_LINE];
	GLUI_Button *button1;
	GLUI_EditText *edittext1;
	GLUI_RadioGroup *radio;
	GLUI_EditText *edittext2;
	GLUI_StaticText *statictext4;
	GLUI_StaticText *statictext5;
	GLUI_StaticText *statictext3 ;
	GLUI_StaticText *statictext6 ;
	GLUI_Checkbox *checkbox_record;
	int main_window;
	GLfloat aspect_ratio;
	GLfloat x_min, x_max, y_min, y_max;
	int width, height;
	}
	
//*********************************************************************
//				Définition des constantes symboliques	

#define CHECKBOX1_ID   11 
#define EDITTEXT1_ID   12 
#define EDITTEXT2_ID   21
#define STATICTEXT4_ID 22
#define STATICTEXT5_ID 23  
#define RADIOBUTTON_ID 31
#define BUTTON1_ID	   41
#define BUTTON2_ID     42
#define ERROR			0
#define NO_ERROR		1
#define RATIO_NORMAL	1
#define MAX_SIZE_WIN_X 600
#define MAX_SIZE_WIN_Y 600
#define OFF				0
#define ON				1

//*********************************************************************
//				Ensemble des fonctions du fichier main

void reshape(int w, int h)
{
	glViewport(0,0, w, h);
	aspect_ratio = (GLfloat) w / (GLfloat) h;
	width = w;
	height= h;
	if(aspect_ratio <= RATIO_NORMAL)
	{
		x_min =-DMAX ; x_max =  DMAX;
		y_min =-DMAX/aspect_ratio;
		y_max = DMAX/aspect_ratio;
	}
	else
	{
		x_min = -DMAX*aspect_ratio;
		x_max = DMAX * aspect_ratio;
		y_min = -DMAX; y_max = DMAX;
	}
	printf("reshape\n");
}

void affichage()
{
	glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear ( GL_COLOR_BUFFER_BIT);
	
	glLoadIdentity();
	glOrtho(x_min, x_max, y_min, y_max, -1, 1);
	if (radio->get_int_val()==OFF)
		simulation_init_selec();
	if (start==ON &&radio->get_int_val()==1  )
		simulation_robot_new_coo(vrota,vtrans);
		
	if(start == ON && verif)
	{
		turn++;
		simulation_init_decomposition();
		simulation_move_robot();
	}
	if (verif)
		simulation_dessine();
	
	glutSwapBuffers();
	glutPostRedisplay();
}
void set_point_cb (int button, int state, int x, int y )
{ 	float new_x ,new_y ;
	new_x = (((float)x/width)*(2*DMAX)) - DMAX;
	new_y = (((float)(height - y)/height)*(2*DMAX)) - DMAX ;
	if (state == GLUT_DOWN)
	{
		if (button == GLUT_LEFT_BUTTON)
		{		
			if (radio->get_int_val()== ON && start== OFF)
			{
				simulation_verif_souris(new_x,new_y);
			}
		}	
	}		
	glutPostRedisplay();
}
void processSpecialKeys(int key, int x, int y)
{	switch(key)
   {	   
		case GLUT_KEY_UP :
		{
			vtrans += 0.250;
			if(vtrans >VTRAN_MAX)
				vtrans=VTRAN_MAX;
			break;
		}	
		case GLUT_KEY_DOWN :
		{
			vtrans -= 0.250;
			if(vtrans < -VTRAN_MAX)
				vtrans= -VTRAN_MAX;
			break;
		}
		case GLUT_KEY_LEFT :
		{
			vrota += 0.125;
			if(vrota >VROT_MAX)
				vrota=VROT_MAX;
			
			break;
		}
		case GLUT_KEY_RIGHT :
		{
			vrota -= 0.125;
			if(vrota < -VROT_MAX)
				vrota= -VROT_MAX;
			break;
		}
	}
   glutPostRedisplay();
}


void glut_initialisation(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE);
	glutInitWindowSize(MAX_SIZE_WIN_X, MAX_SIZE_WIN_Y);
	aspect_ratio = (GLfloat)MAX_SIZE_WIN_X/(GLfloat)MAX_SIZE_WIN_Y;
	main_window = glutCreateWindow("decontaminators-display");
	
	glutDisplayFunc(affichage);
	glutReshapeFunc(reshape);
	glutMouseFunc( set_point_cb );
	glutSpecialFunc(processSpecialKeys);
}

void time_CB(int value)
{	
	rate = simulation_decontamination(fichier);
	snprintf(tab, MAX_LINE, "Rate : %0.4lf", rate);
	snprintf(tab2, MAX_LINE, "Turn : %d", turn);
	statictext3 ->set_text(tab);
	statictext4 ->set_text(tab2);
	if(checkbox_record && start)
	{
		if(plot !=NULL)
			fprintf(plot, "%d %lf\n", turn, rate);
	}
	snprintf(vitran, MAX_LINE, "Translation:%0.4lf", vtrans);
    snprintf(vitrot, MAX_LINE, "Rotation : %0.4lf", vrota);
    statictext5->set_text(vitran);
    statictext6->set_text(vitrot);
	glutPostRedisplay();
	 	if(rate > 99.9999)
	{
		rate = 0;
		turn = 0;
		return;
	}
	glutTimerFunc(250, &time_CB, 0);
}

void cb_open()
{
	verif = NO_ERROR;
	rate = 0.0;
	turn = 0.0;
	printf("edit text1 : %s\n" , edittext1->get_text() );
	if ( fichier != NULL)
	{
		fclose(fichier);
		simulation_free_and_clear();
	}	
	fichier = fopen(edittext1->get_text(), "r");
	if(start == ON)
	{
		start = OFF;
		button1->set_name("Start");
	}
	if(fichier != NULL)
	{
		verif = simulation_lecture_fichier(fichier);
		if(verif == ERROR)
		{
			simulation_free_and_clear();
			glClearColor(1.0, 1.0, 1.0, 0.0);
			glClear ( GL_COLOR_BUFFER_BIT);
		}
		else 
			affichage();
	}
	edittext1->set_text(".txt");
}

void cb_save()
{
	printf("edit text2 : %s\n" , edittext2->get_text() );
	if ( verif == ERROR)
		printf("Le fichier initial comporte une erreur\n");
	else
	{
		FILE *save = fopen(edittext2->get_text(), "w");
		if ( save == NULL)
			printf("Impossible de créer le fichier\n");
		else
		{
			printf("Sauvegarde effectué\n");
			simulation_save_current_state(save);
			edittext2->set_text(".txt");
			fclose(save);
		}
	}
}

void cb_checkbox()
{
	printf("checkbox_record: %d\n" ,
	checkbox_record->get_int_val() );
	plot = fopen("out.dat", "w+");
	if(checkbox_record)
	{
		if(plot == NULL)
			printf("Impossible de créer le fichier\n");
		else
			printf("Record en cours\n");
	}
	else
	{
		if(plot != NULL)
			fclose(plot);
		plot = NULL;
	}
}

void control_cb(int control)
{
	printf( "callback: %d\n", control );
	switch (control)
	{ 
		case (BUTTON1_ID) :
		{
			printf(" Start\n");
			if( start == OFF)
			{
				start = ON;
				button1->set_name("Stop");
			}
			else
			{
				start = OFF;
				button1->set_name("Start");
			}
			break;
		}
		case (BUTTON2_ID) :
		{
			printf(" Step \n");
			start = ON;
			affichage();
			start = OFF;
			break;
		}
		case (CHECKBOX1_ID):
		{
			cb_checkbox();
			break;
		}
		case (EDITTEXT1_ID):
		{
			cb_open();
			break;
		}
		case (EDITTEXT2_ID):
		{
			cb_save();
			break;
		}	
		case (RADIOBUTTON_ID):
			printf("radio group: %d\n", radio->get_int_val() );
			break;
		default: 
			printf("\n Unknown command\n");   
			break;
	}
}


void glui_init(int argc, char* argv[]) 
{ 
	glut_initialisation(argc, argv);
  //initialisation
	GLUI *glui = GLUI_Master.create_glui
	("decontaminators - control",0,700,100); 
  //texte
    GLUI_Panel *panel_txt = glui->add_panel( "Opening");
    edittext1 = glui->add_edittext_to_panel( panel_txt,(char*)
    "File name:", GLUI_EDITTEXT_TEXT, text,EDITTEXT1_ID, control_cb);
    edittext1->set_text(".txt"); 
    glui->add_button_to_panel( panel_txt, "Open"); 
  //saving
    GLUI_Panel *panel_saving = glui->add_panel( "Saving");
    edittext2 = glui->add_edittext_to_panel( panel_saving,(char*)
    "File name:", GLUI_EDITTEXT_TEXT, text,EDITTEXT2_ID, control_cb);
	edittext2->set_text(".txt");
    glui->add_button_to_panel( panel_saving, "Save");
    glui->add_column(true);
   //simulation
    GLUI_Panel *panel_simulation = glui->add_panel( "Simulation");
    button1 = glui->add_button_to_panel( panel_simulation, "Start", 
										 BUTTON1_ID, control_cb);
    glui->add_button_to_panel( panel_simulation, "Step", BUTTON2_ID,
															control_cb);
   //recording
    GLUI_Panel *panel_recording = glui->add_panel( "Recording");
    checkbox_record = glui->add_checkbox_to_panel( panel_recording,
	(char*)"Record",NULL, CHECKBOX1_ID , control_cb);
    statictext3 = glui->add_statictext_to_panel( panel_recording,
    tab);
    statictext4 = glui->add_statictext_to_panel( panel_recording,
    tab2);
    glui->add_column(true);
   //control mode
    GLUI_Panel *panel_control_mode = glui->add_panel( "Control mode");
    radio = glui->add_radiogroup_to_panel(panel_control_mode, NULL,
    RADIOBUTTON_ID,control_cb);
    glui->add_radiobutton_to_group( radio,(char*)   "Automatic" );  
	glui->add_radiobutton_to_group( radio,(char*)   "Manual" );
    
   //robot controle
    GLUI_Panel *panel_robot_control = glui->add_panel( "Robot control");
    statictext5 = glui->add_statictext_to_panel( panel_robot_control,
    vitran);	
    statictext6 = glui->add_statictext_to_panel( panel_robot_control,
    vitrot); 
    
    glui->add_button((char*)"Quit", 0, (GLUI_Update_CB)exit);
    glutTimerFunc(250, &time_CB, 0);
    
    glui->set_main_gfx_window( main_window );
	
	glutMainLoop();

}


int main(int argc, char **argv)
{
	verif = NO_ERROR;
	turn = 0;
	start = OFF;
	fichier = fopen(argv[2], "r");
	if (fichier == NULL)
	{
		glui_init(argc, argv);
	}
	else if( strspn(argv[1], "Error"))
	{
		verif = simulation_lecture_fichier(fichier);
		if(verif == ERROR)
			exit(EXIT_FAILURE);
	}
	else if ( strspn(argv[1], "Draw"))
	{
		verif = simulation_lecture_fichier(fichier);
		if(verif == ERROR)
		{
			glClearColor(1.0, 1.0, 1.0, 0.0);
			glClear ( GL_COLOR_BUFFER_BIT);
			fclose(fichier);
		}
		else
		{
			glui_init(argc, argv);
			fclose(fichier);
		}
	}
	return 0;
}

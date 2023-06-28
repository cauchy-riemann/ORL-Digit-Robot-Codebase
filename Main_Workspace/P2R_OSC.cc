#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include <iostream>
#include <fstream>
#include <cstdlib>
#include <limits>
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <Eigen/QR>
USING_NAMESPACE_QPOASES
#define PI 3.141592653

//simulation end time
double inf = std::numeric_limits<double>::infinity();
double simend = 20;

#define ndof 2

//related to writing data to a file
std::ofstream datafile,qpfile;
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file


char xmlpath[] = "Data/planar_2r.xml";
char filename[] = "Data/Results/data.csv";
char qp_data[] = "Data/qp_data.csv";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
    datafile.open(filename);
	qpfile.open(qp_data);
    if (datafile.is_open()) datafile << "t,tau1,tau2,ddq1,ddq2,q1,q2,X,Y,\n";
    else
        {
            std::cerr << "Unable to open file";
            std::abort;
        }

	if (qpfile.is_open()) qpfile << "t,Q1,Q2,Q3,Q4,M1,M2,M3,M4,f1,f2,C1,C2\n";
    else
        {
            std::cerr << "Unable to open file";
            std::abort;
        }
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate da by a space %f followed by space
    datafile << d->time << "," << d->qfrc_applied[0] << "," << d->qfrc_applied[1] << "," << d->qacc[0] << "," << d->qacc[1] << "," << d->qpos[0] << "," << d->qpos[1] << "," << *(d->site_xpos) << "," << *(d->site_xpos+2) << "," << std::endl;
}
void QP_data(real_t* H, real_t* A, real_t* g,real_t* q,const mjData*d)
{
	qpfile << d->time << "," << H[10] << "," << H[11] << "," << H[14] << ","<< H[15] << "," << A[2] << "," << A[3] << "," << A[6] << "," << A[7] << "," <<g[2] << "," <<g[3] << "," << q[0] << "," << q[1] << "," << std::endl; 
}
	

//**************************

void OSC_controller(const mjModel* m,mjData* d)
{
	Eigen::MatrixXd J(2,2),J_t(2,2),Jh(2,2),dJdt(2,2);
	Eigen::VectorXd q(2),x(2),xref(2),
		            dq(2),dx(2),ddx_des(2),
	            	tau(2),C(2),b(4),beq(2);

	double M[ndof*ndof]; // Mass Matrix
	double dt = 1e-6;    //Differential to calculate the derivative of the Jacobian
	mjtNum jacp[6];      //Linear motion Jacobian
	double Kp = 80, Kv = 10;  //Gains for the PID controller for the acceleration
	
	mj_energyPos(m,d); //Calculate the Potential and Kinetic Energies of the system
	mj_energyVel(m,d);
	mj_jacSite(m,d,jacp,NULL,0); //Calculate the Jacobian at the end-effctor
	
	J << jacp[0],jacp[1], //Get the controllable parts of the Jacobian and store it in a matrix
		 jacp[4],jacp[5];
	
	J_t = J.transpose(); //Jacobian transpose

	q << d->qpos[0], //Generalized position
		 d->qpos[1];

	mj_integratePos(m,d->qpos,d->qvel,dt); //Calculate the Jacobian derivative
	mj_kinematics(m,d);
	mj_jacSite(m,d,jacp,NULL,0);
	Jh << jacp[0],jacp[1],
		 jacp[4],jacp[5];
	dJdt = (Jh-J)/dt;   //Jacobian derivative
	dq << d->qvel[0], //Generalized Velocity
		 d->qvel[1]; 
	
	x << *(d->site_xpos), //Task Space Velocity 
		*(d->site_xpos+2);
	//xref << -1,-1;
	//xref <<  2*cos(d->time),sin(3*d->time); //Reference point
	xref <<  cos(d->time),sin(d->time); //Reference poin
	for (int i = 0; i < m->nq;++i)
		d->qpos[i] = q(i); //Restore the old generalized position to the simulation

	SQProblem dyn_qp(4,2,HST_SEMIDEF); //Set up a quadratic program with 4 variables and 2 constraints 
	Options options; 
	options.printLevel = PL_NONE; //Don't print the output
	dyn_qp.setOptions(options);
	auto Q = J_t*J; //Hessian for the generalized accelerations' quadratic program
	mj_fullM(m,M,d->qM); //Get the mass matrix from the simulation

	
	dx = J*dq; //Task space velocity calculation
	C << d->qfrc_bias[0], //Coriolis and gravitational forces
		 d->qfrc_bias[1];	
	ddx_des = Kp*(xref-x)-Kv*dx; //PID law for task space acceleration
	//ddx_des << 0,0;
	auto g_tau = -J_t*(ddx_des-dJdt*dq); //Linear terms for the generalized accelerations' quadratic program
	real_t H[4*4] = {0.001,  0,  0,  0,  //Hessian for the entire quadratic program
	             	 0,  0.001,  0,  0,
	                 0,  0, Q(0),Q(1),
	                 0,  0, Q(2),Q(3)};
	
	real_t g[4] = {0,0,g_tau(0),g_tau(1)}; //Linear terms for the entire quadratic program

	real_t A[2*4] = {-1,1,M[0],M[1],    //Dynamics encoded into the constraint matrix
		             0,-1,M[2],M[3]};	
	real_t lb[4] = {-100,-100,-inf,-inf}; //Torque/Generalized acceleration lower bounds
	real_t ub[4] = {100,100,inf,inf};     //Torque/Generalized acceleration upper bounds
	real_t lbA[2] = {-C(0),-C(1)};      //Coriolis and gravitational terms
	real_t ubA[2] = {-C(0),-C(1)};
	int_t nWSR = 20;                    //Upper bound of 10 iterations

	dyn_qp.init(H,g,A,lb,ub,lbA,ubA,nWSR,0); //Run the quadratic program
	real_t zstar[4];

	dyn_qp.getPrimalSolution(zstar);      //Store solution to the quadratic program and set the appropriate accelerations and torques in the simulation

	for (int i = 0; i < 2;i++) d->qfrc_applied[i] = zstar[i];
	if ( loop_index%data_frequency==0) save_data(m,d);
			//			QP_data(H,A,g,lb,d);
    loop_index = loop_index + 1;
}
//************************
// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    init_save_data();
    // install control callback
    mjcb_control = OSC_controller;


	d->qpos[0] = 0;
    d->qpos[1] =0;
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        if (d->time>=simend)
        {
            datafile.close();
            break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 0;
}

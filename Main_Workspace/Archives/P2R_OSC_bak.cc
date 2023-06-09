

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
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solver_interface.h"
#include <Eigen/Dense>
#include <Eigen/QR>



//simulation end time
double inf = std::numeric_limits<double>::infinity();
double simend = 20;

#define ndof 2

//related to writing data to a file
std::ofstream datafile;
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file


char xmlpath[] = "planar_2r.xml";
char filename[] = "Data/data.csv";


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
    if (datafile.is_open()) datafile << "t,tau1,tau2,q1,q2,X,IG,Y, \n";
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
  //seperate data by a space %f followed by space
    datafile << d->time << ", " << d->qfrc_applied[0] << ", " << d->qfrc_applied[1] << ", " << d->qpos[0] << ", " << d->qpos[1] << ", " << *(d->site_xpos)
             <<", " << *(d->site_xpos+1) <<", " << *(d->site_xpos+2) <<"," << std::endl;
}

//**************************

/*Eigen::MatrixXd Jdot_func(const mjModel* m,mjData* d,const Eigen::MatrixXd& Jold)
{
    Eigen::MatrixXd J(2,2),Jdot_func(2,2);
    mjtNum jacp[6];
    mj_jacSite(m,d,jacp,NULL,0);
    J << jacp[0],jacp[1],jacp[4],jacp[5];
    double dt = 0.0001;
    Jdot_func = 1/dt*(J-Jold);
    // std::cout << "Dt" << dt << std::endl;
    return Jdot_func;
}*/

void OSC_controller(const mjModel* m,mjData* d)
{
    Eigen::MatrixXd J(2,2),J_t(2,2),M(2,2),M_xinv(2,2),M_x(2,2),Q(2,2);
    Eigen::VectorXd q(2),x(2),dq(2),dx(2),ddx(2),
                    tau(2),xref(2),C(2);

    double M_vec[ndof*ndof]; ///Stores the Inertia matrix
    double m1,m2,a1,a2,g;
    m1 = m->body_mass[1]; //Mass of the first link
    m2 = m->body_mass[2]; //Mass of the second link
    a1 = 2*(*(m->geom_size+4)); //Length of the first link (ties to the geometry of the link)
    a2 = 2*(*(m->geom_size+7)); //"           " second "                                    "
    g = 9.81;
    mjtNum jacp[6]; //The Linear End Effector Jacobian
    double Kp = 50, Kv = 10;
    mj_energyPos(m,d); //Calculates the Potential and Kinetic Energy respectively
    mj_energyVel(m,d);
    Q << 1,0,0,1;
    mj_fullM(m,M_vec,d->qM); //Stores the full interial matrix in M_vec
    M << M_vec[0],M_vec[1],M_vec[2],M_vec[3]; //Converts the Inertial matrix to a typical matrix form
    mj_jacSite(m,d,jacp,NULL,0); //Calculates the linear end effector Jacobian
    J << jacp[0],jacp[1],jacp[4],jacp[5];  //Stores the controllable parts in a 2x2 matrix
   // std::cout << "Jdot: " << Jdot_func(m,d,J) << std::endl;
    J_t = J.transpose();


    //Stores various kinematic quantities in vector form for calculations later
    q << d->qpos[0],d->qpos[1];
    dq << d->qvel[0],d->qvel[1];
    x <<  *(d->site_xpos), *(d->site_xpos+2);
    xref << sin(d->time),0.5*cos(d->time)+2;

    //Calculates task space velocity
    dx = J*dq;
    //Grabs the Coriolis and gravitational terms
    C << d->qfrc_bias[0],d->qfrc_bias[1];


    drake::solvers::MathematicalProgram  prog;
    auto z = prog.NewContinuousVariables(4,"z");
    Eigen::VectorXd z_val(4),beq(2);
    Eigen::MatrixXd Aeq(2,4);
    z_val << 50,50,inf,inf;
    //beq <<
    auto cost = prog.AddQuadraticErrorCost(Q,xref,z);
    prog.AddBoundingBoxConstraint(-z_val,z_val,z);
    Aeq << -1,1,((m1+m2)*a1*a1)+(m2*a2*a2)+(2*m2*a1*a2*cos(q(1))),(m2*a2*a2)+(m2*a1*a2*cos(q(1))),0,-1,(m2*a2*a2)+(m2*a1*a2*cos(q(1))),m2*a2*a2;
    beq <<m2*a1*a2*sin(q(1))*(2*dq(0)*dq(1)+dq(1)*dq(1)) + (m1+m2)*g*a1*cos(q(0)) + m2*g*a2*cos(q(0)+q(1)),m2*a1*a2*sin(q(1))*dq(0)*dq(1)+m2*g*a2*sin(q(1));
    prog.LinearEqualityConstraint(Aeq,beq);
    auto result = drake::solvers::Solve(prog);
    std::cout << "Success? " <<  result.is_success() << std::endl;
    //Print the solution to the decision variables.
    std::cout << "x* = " << result.GetSolution(x)) << std::endl;
    // Print the optimal cost.
    std::cout << "optimal cost = " <<  result.get_optimal_cost() <<std::endl;
             //         // Print the name of the solver that was called.
    std::endl << "solver is: ", result.get_solver_id().name() << std::endl;



    //Task space inertial matrix inverse
    M_xinv = J*M*J_t;
    //If the task space inertial matrix not is singular, calculate the inverse in the typical method.
    //Otherwise use the Moore-Penrose inverse to find the best fit for the inverse.
     if (M_xinv.determinant() > 0.005)
         M_x = M_xinv.inverse();
     else
         M_x = M_xinv.completeOrthogonalDecomposition().pseudoInverse();

     //PD control law
    ddx = Kp*(xref-x)-Kv*(dx);
    //Calculate the neccessary torques to generate perform OSC
    tau = J_t*M_x*ddx+C;
    for (int i = 0; i < 2;i++) d->qfrc_applied[i] = tau(i);
    if ( loop_index%data_frequency==0) save_data(m,d);
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

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 2.000000};
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
    d->qpos[1] = 1;
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

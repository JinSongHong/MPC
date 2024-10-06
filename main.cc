#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "Sub_func.hpp"
#include "globals.hpp"
#include "Mathtool.hpp"
// #include <eigen-master/Eigen/Core>


using namespace ROPTLIB;
using namespace Eigen;
using namespace std;



double t;


const int leg_FL_no = 0;
const int leg_FR_no = 3;
const int leg_RL_no = 6;
const int leg_RR_no = 9;



Kinematics K_FL(0);
Kinematics K_FR(1);
Kinematics K_RL(2);
Kinematics K_RR(3);

Controller C_FL(0);
Controller C_FR(1);
Controller C_RL(2);
Controller C_RR(3);

Trajectory Traj_FL(0);
Trajectory Traj_FR(1);
Trajectory Traj_RL(2);
Trajectory Traj_RR(3);

Vector3d FL_ctrl_input = Vector3d::Zero();
Vector3d FR_ctrl_input = Vector3d::Zero();
Vector3d RL_ctrl_input = Vector3d::Zero();
Vector3d RR_ctrl_input = Vector3d::Zero();

Vector3d FL_Joint_input = Vector3d::Zero();
Vector3d FR_Joint_input = Vector3d::Zero();
Vector3d RL_Joint_input = Vector3d::Zero();
Vector3d RR_Joint_input = Vector3d::Zero();

Vector3d joint_input[4];

// MyProblem Prob(1,3);
// ProdManifold ProdProb(1,4,0,3);

double sampling_time = 0.03;
double optimization_t = sampling_time;
int horizon_time = 3;

MPC MPC_class(sampling_time, horizon_time);
Euclidean u(12 * horizon_time);
MyProblem MPC_optu(0 ,12 * horizon_time, MPC_class);
Variable P;

// Vector A(12,1);

void JScontroller(const mjModel* m, mjData* d) {

    t = d->time;
    
    // u.CheckParams();
    // cout << "x:" << d->xipos[0] << "  y:" << d->xipos[1] << "  z:" << d->xipos[2] << endl;
    // cout << "w x: " << d->subtree_com[0] << "  w y: " << d->subtree_com[1] << "  w z: " << d->subtree_com[2] << endl;
    // cout << foot_pos[0] << endl;
    if (t >= optimization_t)
    {   
        // Optimization loop
        MPC_class.updateState(t);
        MPC_class.Cal_Parameter(K_FL, K_FR, K_RL, K_RR);
        // MPC_class.Cost_function(t);
        MPC_optu.get_MPC(MPC_class);

        // MPC_optu.get_f(MPC_class.Cost_function(P, t));       

        optimization_t += sampling_time;

        RBFGS *solver3 = new RBFGS(&MPC_optu, &P);
        solver3->Verbose = FINALRESULT;
        // // // solver->LineSearch_LS = LSSM_INPUTFUN;
        // // // solver->LinesearchInput = &LinesearchInput;
        // // solver->IsPureLSInput = false;
        // // // solver->StopPtr = &Stop;
        solver3->Max_Iteration = 100; // Define the maximum number of iterations
        solver3->OutputGap = 1; // 1번째 반복마다 결과를 출력
        // solver3->CheckParams();
        // u.CheckParams();
        // Crtl.CheckGradHessian(f.statespace(K_FL, K_FR, K_RL, K_RR, P, t));
        
        solver3->Run();
        
        
        // for(int i = 0; i < 12; i ++)
        // {
        //     d->ctrl[i] = solver3->GetXopt().ObtainReadData()[i];
        // }


        Vector3d ctrl_FL;
        Vector3d ctrl_FR;
        Vector3d ctrl_RL;
        Vector3d ctrl_RR;
        
        

        VectorXd uk = VectorXd::Zero(12) ;
        VectorXd up = VectorXd::Zero(12) ;
        
        for(int i = 0; i < 12; i ++)
        {
            uk[i] = solver3->GetXopt().ObtainReadData()[i];
        }        
        MPC_optu.get_up(uk);

        // cout << uk << endl;

        for(int i = 0; i < 3; i ++)
        {
            ctrl_FL[i] = solver3->GetXopt().ObtainReadData()[i];
            ctrl_FR[i] = solver3->GetXopt().ObtainReadData()[i + 3];
            ctrl_RL[i] = solver3->GetXopt().ObtainReadData()[i + 6];
            ctrl_RR[i] = solver3->GetXopt().ObtainReadData()[i + 9];
        }
            ctrl_FL[0] = -ctrl_FL[0];
            ctrl_FR[0] = -ctrl_FR[0];
            ctrl_RL[0] = -ctrl_RL[0];
            ctrl_RR[0] = -ctrl_RR[0];
        

        joint_input[0] = K_FL.get_Jacbtrans() * K_FL.get_R_trunk().transpose() * ctrl_FL;
        joint_input[1] = K_FR.get_Jacbtrans() * K_FR.get_R_trunk().transpose() * ctrl_FR;
        joint_input[2] = K_RL.get_Jacbtrans() * K_RL.get_R_trunk().transpose() * ctrl_RL;
        joint_input[3] = K_RR.get_Jacbtrans() * K_RR.get_R_trunk().transpose() * ctrl_RR;

        // cout << "joint torque\n" << K_FL.get_Jacbtrans() * K_FL.get_R_trunk().transpose() * ctrl_FL << endl;
        // cout << "HAA: " << joint_input[0][0] << "Hip: " << joint_input[0][1] << "   Knee: " << joint_input[0][2] << endl;
        // cout << "joint input[1] = "<< joint_input[0][1] << endl;
        for(int i = 0; i < 4; i++)
        joint_input[i][1] += joint_input[i][2];    
        // cout << "joint torque_FL\n" << K_FL.get_Jacbtrans() * K_FL.get_R_trunk().transpose() * ctrl_FL << endl;
        // cout << "joint torque_FR\n" << K_FR.get_Jacbtrans() * K_FR.get_R_trunk().transpose() * ctrl_FR << endl;
        // cout << "joint torque_RL\n" << K_RL.get_Jacbtrans() * K_RL.get_R_trunk().transpose() * ctrl_RL << endl;
        // cout << "joint torque_RR\n" << K_RR.get_Jacbtrans() * K_RR.get_R_trunk().transpose() * ctrl_RR << endl;

 
        delete solver3;
    }
    


    
    // cout << A << endl;
    // ProductManifold Traj(2,&Ori,1,&Translation,1);  // Num of Type, manifold1, num of mani1, manifold2, num of mani2
    // Traj.CheckParams();

    // d->qpos[0] = 0;
    // d->qpos[1] = 0;
    // d->qpos[2] = 0.4;   // qpos[0,1,2] : trunk pos                                                                                                                 
    // d->qpos[3] = -0.73;                    // qpos[3,4,5.6] : trunk orientation quaternian
    // d->qpos[4] = 0.73;
    

    
    // cout << d->subtree_angmom[0] << endl;

    if(t < 0.00000000000000001)
    {
        // cout << "w x: " << d->subtree_com[0] << "  w y: " << d->subtree_com[1] << "  w z: " << d->subtree_com[2] << endl;
        d->qpos[5] = 0;
        d->qpos[6] = 0;
        d->qpos[7] = 0; //FLHAA         
        d->qpos[8] = PI/4; //FLHIP       
        d->qpos[9] = PI/2; //FLKNEE        
        d->qpos[10] = 0; //FRHAA        
        d->qpos[11] = PI/4; //FRHIP        
        d->qpos[12] = PI/2; //FRKNEE       
        d->qpos[13] = 0; //RLHAA        
        d->qpos[14] = PI/4; //RLHIP        
        d->qpos[15] = PI/2; //RLKNEE       
        d->qpos[16] = 0; //RRHAA        
        d->qpos[17] = PI/4; //RRHIP        
        d->qpos[18] = PI/2; //2.2+0.4*sin(t); //RRKNEE       
    }


            for(int i = 0; i < 3; i ++)
        {
            d->ctrl[i] = joint_input[0][i];
            // cout << joint_input[0][i] << endl;
            d->ctrl[i + 3] = joint_input[1][i];
            d->ctrl[i + 6] = joint_input[2][i];
            d->ctrl[i + 9] = joint_input[3][i];
        }

        // d->ctrl[0] = joint_input[0][0];
        // d->ctrl[2] = joint_input[0][1];
        // d->ctrl[1] = joint_input[0][2];
        // d->ctrl[3] = joint_input[1][0];
        // d->ctrl[5] = joint_input[1][1];
        // d->ctrl[4] = joint_input[1][2];
        // d->ctrl[6] = joint_input[2][0];
        // d->ctrl[8] = joint_input[2][1];
        // d->ctrl[7] = joint_input[2][2];
        // d->ctrl[9] = joint_input[3][0];
        // d->ctrl[11] = joint_input[3][1];
        // d->ctrl[10] = joint_input[3][2];

        

    

    K_FL.sensor_measure(m, d, leg_FL_no);
    K_FR.sensor_measure(m, d, leg_FR_no);
    K_RL.sensor_measure(m, d, leg_RL_no);
    K_RR.sensor_measure(m, d, leg_RR_no);

    K_FL.Cal_Kinematics();
    K_FR.Cal_Kinematics();
    K_RL.Cal_Kinematics();
    K_RR.Cal_Kinematics();
    
    // FL_ctrl_input = C_FL.FB_controller(K_FL.get_vel_error(Traj_FL.Leg_vel_trajectory(t)), K_FL.get_vel_error_old());        
    // FR_ctrl_input = C_FR.FB_controller(K_FR.get_vel_error(Traj_FR.Leg_vel_trajectory(t)), K_FR.get_vel_error_old());
    // RL_ctrl_input = C_RL.FB_controller(K_RL.get_vel_error(Traj_RL.Leg_vel_trajectory(t)), K_RL.get_vel_error_old());
    // RR_ctrl_input = C_RR.FB_controller(K_RR.get_vel_error(Traj_RR.Leg_vel_trajectory(t)), K_RR.get_vel_error_old());
    
    // FL_Joint_input =  K_FL.get_Jacbtrans() * FL_ctrl_input;
    // FR_Joint_input =  K_FR.get_Jacbtrans() * FR_ctrl_input;
    // RL_Joint_input =  K_RL.get_Jacbtrans() * RL_ctrl_input;
    // RR_Joint_input =  K_RR.get_Jacbtrans() * RR_ctrl_input;
    
    // d->ctrl[0] = FL_Joint_input[0]; // FLHAA
    // d->ctrl[1] = FL_Joint_input[1]; // FLHIP
    // d->ctrl[2] = FL_Joint_input[2]; // FLKNEE
    
    // d->ctrl[3] = FR_Joint_input[0]; // FRHAA
    // d->ctrl[4] = FR_Joint_input[1]; // FRHIP
    // d->ctrl[5] = FR_Joint_input[2]; // FRKNEE

    // d->ctrl[6] = RL_ctrl_input[0]; // RLHAA
    // d->ctrl[7] = RL_ctrl_input[1]; // RLHIP
    // d->ctrl[8] = RL_ctrl_input[2]; // RLKNEE
    // d->ctrl[9] = RR_ctrl_input[0]; // RRHAA
    // d->ctrl[10] = RR_ctrl_input[1]; // RRHIP
    // d->ctrl[11] = RR_ctrl_input[2]; // RRKNEE

    // d->ctrl[0] = FL_Joint_input[0]; // FLHAA
    // d->ctrl[1] = FL_Joint_input[1]; // FLHIP
    // d->ctrl[2] = FL_Joint_input[2]; // FLKNEE
    
    // d->ctrl[3] = FR_Joint_input[0]; // FRHAA
    // d->ctrl[4] = FR_Joint_input[1]; // FRHIP
    // d->ctrl[5] = FR_Joint_input[2]; // FRKNEE

//     d->ctrl[6] = RL_ctrl_input[0]; // RLHAA
//     d->ctrl[7] = RL_ctrl_input[1]; // RLHIP
//     d->ctrl[8] = RL_ctrl_input[2]; // RLKNEE
//     d->ctrl[9] = RR_ctrl_input[0]; // RRHAA
//     d->ctrl[10] = RR_ctrl_input[1]; // RRHIP
//     d->ctrl[11] = RR_ctrl_input[2]; // RRKNEE

    


//////////////////////////////////////////////////////////////////////////////////////



    // SPDManifold Domain(3);

    // // Domain.ChooseParamsSet1();
    // Variable X = Domain.RandominManifold(); // Varaible is a point on manifold
    // Vector K = Domain.RandominManifold();   // Vector is vector on tangent space
    
    // double x_data[] = {1, 0, 0, 0};
    // double trans_data[] = {10, 0, 0, 0, 20, 0, 0, 0, 30};
    // copy(x_data, x_data + 9, X.ObtainWriteEntireData());

    // RBFGS *solver = new RBFGS(&Prob, &X);
    // solver->Verbose = ITERRESULT;
    // // // solver->LineSearch_LS = LSSM_INPUTFUN;
    // // // solver->LinesearchInput = &LinesearchInput;
    // // solver->IsPureLSInput = false;
    // // // solver->StopPtr = &Stop;
    // // solver->Max_Iteration = 100; // Define the maximum number of iterations
    // // solver->OutputGap = 1; // 1번째 반복마다 결과를 출력
    // // solver->CheckParams();
    // solver->Run();

    // // Prob.CheckGradHessian(solver->GetXopt());

    // delete solver;

/////////////////////////////////////////////////////////////////////////////////////////////
    // Sphere Ori(4);
    // Euclidean Translation(3);
    // ProductManifold Traj(2,&Ori,1,&Translation,1);  // Num of Type, manifold1, num of mani1, manifold2, num of mani2
    // // Traj.CheckParams();
    // Variable P = Traj.RandominManifold();
    
    // double *Ori_ptr = P.GetElement(0).ObtainWriteEntireData();
    // double *Trans_ptr = P.GetElement(1).ObtainWriteEntireData();// copy(Translation, Translation + 3, P.GetElement(1).ObtainWriteEntireData());
    // Ori_ptr[0] = 0;
    // Ori_ptr[1] = 1;
    // Ori_ptr[2] = 0;
    // Ori_ptr[3] = 0;

    // Trans_ptr[0] = 1;
    // Trans_ptr[1] = 5;
    // Trans_ptr[2] = 4;
    
    // // cout << P.GetElement(0).ObtainReadData()[0] << P.GetElement(0).ObtainReadData()[1] << P.GetElement(0).ObtainReadData()[2] << P.GetElement(0).ObtainReadData()[3] << endl;
    // // cout << P.GetElement(1).ObtainReadData()[0] << P.GetElement(1).ObtainReadData()[1] << P.GetElement(1).ObtainReadData()[2] << endl;


    // RBFGS *solver2 = new RBFGS(&ProdProb, &P);
    // solver2->Verbose = ITERRESULT;
    // // // // solver->LineSearch_LS = LSSM_INPUTFUN;
    // // // // solver->LinesearchInput = &LinesearchInput;
    // // // solver->IsPureLSInput = false;
    // // // // solver->StopPtr = &Stop;
    // solver2->Max_Iteration = 100; // Define the maximum number of iterations
    // solver2->OutputGap = 1; // 1번째 반복마다 결과를 출력
    // // solver2->CheckParams();
    // // solver2->Run();
    // // cout << solver2->GetXopt() << endl;
    // // Prob.CheckGradHessian(solver2->GetXopt());

    // delete solver2;

    K_FL.update_old_state();
    K_FR.update_old_state();
    K_RL.update_old_state();
    K_RR.update_old_state();

}




// main function
int main(int argc, const char** argv)
{

    // MPC_optu.Init_ROPT();
    P = u.RandominManifold();
    // cout << P << endl;



















    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

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

    // double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    // cam.azimuth = arr_view[0];
    // cam.elevation = arr_view[1];
    // cam.distance = arr_view[2];
    // cam.lookat[0] = arr_view[3];
    // cam.lookat[1] = arr_view[4];
    // cam.lookat[2] = arr_view[5];

    mjcb_control = JScontroller;

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

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

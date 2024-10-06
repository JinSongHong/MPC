#ifndef MYPROBLEM_H
#define MYPROBLEM_H

#include "iostream"
#include "globals.hpp"
#include "Problem.h"
#include "eigen-master/Eigen/Core"
#include "eigen-master/Eigen/Dense"


using namespace std;
using namespace ROPTLIB;
using namespace Eigen;

class MPC;

class MyProblem : public Problem {

private:

    VectorXd df;
    MatrixXd Q;
    MatrixXd R;
    VectorXd ek;
    MatrixXd F;
    MatrixXd phi;
    MatrixXd xk;
    VectorXd xref;
    MatrixXd Lp;
    VectorXd up;


    int horizon_time;

    Vector Q_;
    Vector R_;
    Vector ek_;
    Vector F_;
    Vector Fxk_;
    Vector phi_;
    Vector xref_;
    Vector xk_;
    
    Vector eT_Q;
    Vector eT_Q_e;

    VectorXd g; 
    

    

public:
    MyProblem(int Manifold, int demension, MPC MPC_);
    void get_MPC(MPC MPC_);
    void get_up(VectorXd u) {up = u;}
    void Init_ROPT();
    Vector VectorXd_to_ROPT(VectorXd V, Vector V_ROPT);
    Vector MatrixXd_to_ROPT(MatrixXd M, Vector V_ROPT);
    
    virtual realdp f(const Variable &x) const override;   // Problem.h file의 f 함수를 상속받아서 쓰기 때문에 x변수를 동일하게 하나만 받아올 수 있음
    // virtual void Grad(Variable *x, Vector *gf) const override;
    virtual Vector &EucGrad(const Variable &x, Vector *egf) const override;
    virtual Vector &RieGrad(const Variable &x, Vector *gf) const override;

    
};

#endif // MYPROBLEM_H

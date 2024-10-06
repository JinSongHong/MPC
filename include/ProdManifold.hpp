// File: ProdManifold.hpp

#ifndef PRODMANIFOLD_HPP
#define PRODMANIFOLD_HPP

#include "iostream"
#include "Problems/Problem.h"
#include "globals.hpp"
#include "eigen-master/Eigen/Core"
#include "eigen-master/Eigen/Dense"

using namespace std;
using namespace ROPTLIB;
using namespace Eigen;

class ProdManifold : public Problem {
private:
    
    int Man1;
    int Man2;    
    int dimension1;
    int dimension2;
public:
    ProdManifold(int Manifold1, int dim1, int Manifold2, int dim2);
    virtual realdp f(const Variable &x) const override; // 목적 함수
    virtual Vector &EucGrad(const Variable &x, Vector *egf) const override; // 유클리디안 기울기
    virtual Vector &RieGrad(const Variable &x, Vector *gf) const override; // 리만 기하학적 기울기
    

};

#endif // PRODMANIFOLD_HPP

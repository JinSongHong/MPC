#include "MyProblem.hpp"


MyProblem::MyProblem(int Manifold, int demension, MPC MPC_)
{
    // Set the manifold
    if(Manifold == 0)
    Domain = new Euclidean(demension);
    else if(Manifold == 1)
    Domain = new Sphere(demension);
    else if(Manifold == 2)
    Domain = new SPDManifold(demension);


    g = VectorXd::Zero(MPC_.get_horizon_time() * MPC_.get_state_size());

    for(int i = 0; i < MPC_.get_horizon_time(); i++)
    g[(i+1) * MPC_.get_state_size()-1] = -9.81;
    
    up = VectorXd::Zero(MPC_.get_state_size());
 
}
void MyProblem::Init_ROPT()
{
    // Vector Q_(Q.rows(), Q.cols());
    // Vector R_(R.rows(), R.cols());
    // Vector ek_(ek.rows(), ek.cols());
    // Vector F_(F.rows(), F.cols());
    // Vector phi_(phi.rows(), phi.cols());
    // Vector Fxk_(Fxk.rows(), Fxk.cols());
}

Vector MyProblem::VectorXd_to_ROPT(VectorXd V, Vector V_ROPT)
{
    double* V_p = V_ROPT.ObtainWriteEntireData();

    for(int i = 0; i < V.size(); i++)
    {   
        V_p[i] = V[i];    
    }

    return V_ROPT;
}

Vector MyProblem::MatrixXd_to_ROPT(MatrixXd V, Vector V_ROPT)
{
    
    double* V_p = V_ROPT.ObtainWriteEntireData();
    for(int i = 0; i < V.cols(); i++) // 12
    {
        for(int j = 0; j < V.rows(); j++) // 24
        {
            V_p[i*V.rows() + j] = V(j,i);    
        }
    }


    return V_ROPT;
}

void MyProblem:: get_MPC(MPC MPC_)
{
    Q = MPC_.get_Q();
    R = MPC_.get_R();
    df = MPC_.get_df();
    ek = MPC_.get_ek();
    F = MPC_.get_F();
    xk = MPC_.get_xk();
    phi = MPC_.get_phi();    
    horizon_time = MPC_.get_horizon_time();
    xref = MPC_.get_xref();
    Lp = MPC_.get_Lp();

    Vector Q_MPC(Q.rows(), Q.cols());
    Vector R_MPC(R.rows(), R.cols());
    Vector ek_MPC(ek.rows(), ek.cols());
    Vector F_MPC(F.rows(), F.cols());
    Vector phi_MPC(phi.rows(), phi.cols());
    Vector xk_MPC(xk.rows(), xk.cols());
    Vector xref_MPC(xref.rows(), xref.cols());

    Q_ = MatrixXd_to_ROPT(Q, Q_MPC);
    R_ = MatrixXd_to_ROPT(R, R_MPC);
    ek_ = VectorXd_to_ROPT(ek, ek_MPC);
    F_ = MatrixXd_to_ROPT(F, F_MPC);
    
    phi_ = MatrixXd_to_ROPT(phi, phi_MPC);
    xref_ = VectorXd_to_ROPT(xref, xref_MPC);
    xk_ = VectorXd_to_ROPT(xk, xk_MPC);



}

realdp MyProblem::f(const Variable &x) const {    // x = Point on Manifold


    const double *uptr = x.ObtainReadData();    
     
    // Vector Ru(R.rows(), 1);
    //     Ru = Ru.AlphaABaddBetaThis(1, R_, GLOBAL::N, x, GLOBAL::N, 0);
    // x.AddToFields("Ru", Ru);

    VectorXd u = VectorXd::Zero(x.Getlength());
    for(int i = 0; i< x.Getlength(); i++)
    {u[i] = uptr[i];}
    
    // cout << << endl;
    // cout << "Bu\n" << Lp * up << endl;
    VectorXd y = F*xk + phi * u + g + Lp * up ;
    VectorXd e = xref - y;
    // cout << "x error:" << e[3] << "  y error:" << e[4] << "  z error:" << e[5] << endl;
    realdp result = u.transpose() * R * u;
    // realdp result = x.DotProduct(Ru);
        // cout << result << "  " << u.transpose() * R * u << endl;
    // e = xref - (F * xk - phi * u)
    // Vector phiu(phi.rows(), 1);
    //     phiu = phiu.AlphaABaddBetaThis(1, phi_, GLOBAL::N, x, GLOBAL::N, 0);
    
    // Vector e(ek.rows(), ek.cols());
    //     e = phiu.AlphaABaddBetaThis(1, F_, GLOBAL::N, xk_, GLOBAL::N, -1);

    // Vector e(ek.rows(), ek.cols());
    //     e = e.AlphaABaddBetaThis(1, xref_, GLOBAL::N, 1, GLOBAL::N, -1);

    // cout << "e_ROPT\n" << e << endl;
    // cout << "e\n" <<  xref - F*xk + phi* u << endl;    
    
    // Vector Qe(Q.rows(), 1);
    // Qe = Qe.AlphaABaddBetaThis(1, Q_, GLOBAL::N, e, GLOBAL::N, 0);
    // x.AddToFields("Qe", Qe);

    // ek = xref - F*xk + phi* u;

    // cout << "e^TQe: " << (xref - F*xk + phi* u + g).transpose()  * Q * (xref - F*xk + phi* u + g) << endl;
    
    // cout << "u^TRu: "  << u.transpose() * R * u << endl;
    
    // result += (xref - F*xk - phi* u - g).transpose()  * Q * (xref - F*xk - phi* u - g);
    result += e.transpose()  * Q * e;

    // cout << up.size() << "  " << u.size() << endl;
    // cout << up << endl;
    return result;  
    
}

Vector &MyProblem::EucGrad(const Variable &x, Vector *egf) const{

    const double *uptr = x.ObtainReadData();    
    VectorXd u = VectorXd::Zero(x.Getlength());
    for(int i = 0; i< x.Getlength(); i++)
    {u[i] = uptr[i];        }
    
    egf->NewMemoryOnWrite();
    
    double *egfPtr = egf->ObtainWriteEntireData();
    for(int i = 0; i< x.Getlength(); i++)
    {
        egfPtr[i] = (2*phi.transpose()*Q*(-xref + F*xk + phi* u + g + Lp * up) + 2*R*u)[i];
        // cout <<     (2*phi.transpose()*Q*(xref - F*xk + phi* u)).[i] << endl;
    }
    
    // cout << "State\n" << xk << endl;
    // cout << "********error********\n" << xref - F*xk + phi* u << endl;
    // cout << "input\n" << u << endl;
    
    // *egf = x.Field("Ru").AlphaABaddBetaThis(2, phi_, GLOBAL::T, x.Field("Qe"), GLOBAL::N, 2);
    
    return *egf;



}

Vector &MyProblem::RieGrad(const Variable &x, Vector *gf) const {
    // 먼저 유클리디안 기울기 계산

    EucGrad(x, gf);
    // cout << "Euclidean gradient" << endl;
    // cout << gf->ObtainReadData()[0] << "    " << gf->ObtainReadData()[1] << "    " << gf->ObtainReadData()[2] << endl;
    // cout << gf->ObtainReadData()[3] << "    " << gf->ObtainReadData()[4] << "    " << gf->ObtainReadData()[5]<< endl;
    // cout << gf->ObtainReadData()[6] << "    " << gf->ObtainReadData()[7] << "    " << gf->ObtainReadData()[8]<< endl;
    
    // 리만 기하학적 기울기로 변환 (도메인 다양체에 따라 변환)
    // Domain->EucGradToGrad(x, *gf, this, gf);
    // cout << "Riemannian gradient" << endl;
    // cout << gf->ObtainReadData()[0] << "    " << gf->ObtainReadData()[1] << "    " << gf->ObtainReadData()[2] << endl;
    // cout << gf->ObtainReadData()[2] << "    " << gf->ObtainReadData()[4] << "    " << gf->ObtainReadData()[5]<< endl;
    // cout << gf->ObtainReadData()[6] << "    " << gf->ObtainReadData()[7] << "    " << gf->ObtainReadData()[8]<< endl;
    


}

// void MyProblem::Grad(Variable *x, Vector *gf) const {
//     const double *Xptr = x->ObtainReadData(); // Cost
//     double *Gfptr = gf->ObtainWriteEntireData(); // gf = dynamic array
//     // Euclidean gradient of cost //  
//     Gfptr[0] = 1; Gfptr[1] = 0; Gfptr[2] = 0;
//     Gfptr[3] = 0; Gfptr[4] = 1; Gfptr[5] = 0;
//     // x.Getsize();
//     // Gfptr[6] = 0; Gfptr[7] = 0; Gfptr[8] = 2;
    
//     // for (int i = 0; i < 4; i++)
//     // {
//     //     Gfptr[i] = 2*(Xd[i] - Xptr[i]);
//     // }
    
// }


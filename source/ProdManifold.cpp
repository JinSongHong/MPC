// File: ProdManifold.cpp
#include"ProdManifold.hpp"

ProdManifold::ProdManifold(int Manifold1, int dim1, int Manifold2, int dim2) {
    // 두 개의 다양체를 결합하여 곱 다양체 생성
    Manifold *Mani1, *Mani2;

    switch (Manifold1) {
        case 0:
            Mani1 = new Euclidean(dim1);
            break;
        case 1:
            Mani1 = new Sphere(dim1);
            break;
        case 2:
            Mani1 = new SPDManifold(dim1);
    }

    switch (Manifold2) {
        case 0:
            Mani2 = new Euclidean(dim2);
            break;
        case 1:
            Mani2 = new Sphere(dim2);
            break;
        case 2:
            Mani2 = new SPDManifold(dim2);
    }

    Domain = new ProductManifold(2, Mani1, 1, Mani2, 1);
}

realdp ProdManifold::f(const Variable &x) const {
    
    const double *manifold1 = x.GetElement(0).ObtainReadData();
    const double *manifold2 = x.GetElement(1).ObtainReadData();
    Vector4d Man1;
    Vector3d Man2;
    for(int i = 0; i < 4; i++){Man1[i] = manifold1[i];}
    for(int i = 0; i < 3; i++){Man2[i] = manifold2[i];}

    double f_man1 = Man1.transpose() * Man1;
    double f_man2 = Man2.transpose() * Man2;
    double result = f_man1 + f_man2;
    // cout << "result: " << result << endl;
    // cout << "Man2\n " << Man2 << endl;
    return result;
}

Vector &ProdManifold::EucGrad(const Variable &x, Vector *egf) const {

    // cout << "EucGrad" << endl;
    egf->NewMemoryOnWrite();

    double *Man1_Grad = egf->GetElement(0).ObtainWriteEntireData();
    double *Man2_Grad = egf->GetElement(1).ObtainWriteEntireData();

    const double *Man1_variable = x.GetElement(0).ObtainReadData();
    const double *Man2_variable = x.GetElement(1).ObtainReadData();

    Man1_Grad[0] = 2 * Man1_variable[0];
    Man1_Grad[1] = 2 * Man1_variable[1];
    Man1_Grad[2] = 2 * Man1_variable[2];
    Man1_Grad[3] = 2 * Man1_variable[3];
    
    Man2_Grad[0] = 2 * Man2_variable[0];
    Man2_Grad[1] = 2 * Man2_variable[1];
    Man2_Grad[2] = 2 * Man2_variable[2];
    
    std::cout << "Gradient calculated successfully for each manifold." << std::endl;
    // cout << egf->GetElement(0) << endl;
    return *egf;
}



Vector &ProdManifold::RieGrad(const Variable &x, Vector *gf) const {
    
    EucGrad(x, gf);
    // cout << gf->GetElement(1) << endl;
    cout << "Euclidean gradient calculated" << endl;
    // cout << x.Getlength() << endl;
    // cout << gf->Getlength() << endl;

    // cout << "Euclidean gradient" << endl;
    // cout << gf->ObtainReadData()[0] << endl;
    // cout << gf->ObtainReadData()[1] << endl;
    // cout << gf->ObtainReadData()[2] << endl;
    // cout << gf->ObtainReadData()[3] << endl;
    // cout << gf->ObtainReadData()[4] << endl;
    // cout << gf->ObtainReadData()[5] << endl;
    // cout << gf->ObtainReadData()[6] << endl;
    
    // cout << gf->GetElement(0) << endl;
    // // 리만 기하학적 기울기로 변환 (도메인 다양체에 따라 변환)
    Domain->EucGradToGrad(x, *gf, this, gf);
    // cout << "Riemannian gradient" << endl;
    // cout << gf->ObtainReadData()[0] << endl;
    // cout << gf->ObtainReadData()[1] << endl;
    // cout << gf->ObtainReadData()[2] << endl;
    // cout << gf->ObtainReadData()[3] << endl;
    // cout << gf->ObtainReadData()[4] << endl;
    // cout << gf->ObtainReadData()[5] << endl;
    // cout << gf->ObtainReadData()[6] << endl;

    return *gf;
    
}


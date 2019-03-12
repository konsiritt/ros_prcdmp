//
// Created by Elie Aljalbout on 18.11.18.
//

#include "DMP/DiscreteDMP.hpp"


DiscreteDMP::DiscreteDMP(int nDMPs, int nBFs, double dt, std::vector<double> &y0, std::vector<double> &goal,
                         std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern)
    : DMP(nDMPs, nBFs, dt, y0, goal, w, gainA, gainB, pattern)
{
    genCenters();

    //Set variance of gaussian basis functions
    double var = nBFs*sqrt(nBFs) / cs.getGainA();
    for (int b=0; b<nBFs; b++)
    {
        vars.push_back(var/centers[b]);
    }

    checkOffset();
}

DiscreteDMP::DiscreteDMP(int nDMPs, double dt, std::vector<double> &y0, std::vector<double> &goal,
                         std::vector<double> &gainA, std::vector<double> &gainB, std::string pattern)
    : DMP(nDMPs, dt, y0, goal, gainA, gainB, pattern)
{
    checkOffset();
}

void DiscreteDMP::setInitialPosition (std::vector<double> &y_0)
{
  if (y0.size()==y_0.size()){
    this->y0 = y_0;
    this->resettState();
  }
  else {
    std::cerr<<"setInitialPosition: wrong vector size specified!"<<std::endl;
  }
}

void DiscreteDMP::setEndThreshold(double thrsh)
{
  this->endThreshold = thrsh;
}

std::vector<double> DiscreteDMP::step( std::vector<double> &externalForce, double tau, double error)
{
    if (w.size()==0)
    {
        throw "ATTENTION: Weights Matrix not initialized";
    }

    double errorCoupling = 1.0/(1.0+error);

    double x    = cs.step(tau, errorCoupling);
    if (x < endThreshold) {trajFinished = true;}
    std::vector<double> psi;
    genPSI(x, psi);

    double f;
    int maxDur=0;
    for (int i=0; i<nDMPs; i++)
    {
        double in, acc;
        in = inner_product(&psi[0], &w[i][0], psi.size());
        acc = accumulate(&psi[0], psi.size() );
        f = x * in / acc;

        this->ddy[i] = tau*tau*(gainA[i]* (gainB[i]*(goal[i]-this->y[i] -(goal[i] - y0[i])*x +f ) -this->dy[i]/tau) + couplingTerm[i]); //2009

        if(externalForce.size()==nDMPs)
        {
            this->ddy[i] += externalForce[i];
        }
        this->dy[i] += this->ddy[i]*dt*errorCoupling;
        this->y[i]  += this->dy[i]*dt*errorCoupling;
    }
    return this->dy;
}

std::vector<double> DiscreteDMP::simpleStep( std::vector<double> &externalForce, double tau, double error)
{
    double errorCoupling = 1.0/(1.0+error);

    double x    = cs.step(tau, errorCoupling);
    if (x < endThreshold) {trajFinished = true;}

    for (int i=0; i<nDMPs; i++)
    {
        this->ddy[i] = tau*tau*(gainA[i]* (gainB[i]*(goal[i]-this->y[i] -(goal[i] - y0[i])*x ) -this->dy[i]/tau)); //2009

        if(externalForce.size()==nDMPs)
        {
            this->ddy[i] += externalForce[i];
        }
        this->dy[i] += this->ddy[i]*dt*errorCoupling;
        this->y[i]  += this->dy[i]*dt*errorCoupling;
    }
    return this->dy;
}

void DiscreteDMP::genPSI(const double &x, std::vector<double> &psi)
{
    psi.clear();
    for (int i=0; i<nBFs; i++)
    {
        psi.push_back(exp( -vars[i] * pow((x-centers[i]),2)));
    }
}

void DiscreteDMP::genCenters()
{
    std::vector<double> desCenters = UTILS::linspace<double>(0,  cs.getRunTime(), nBFs);

    for (int i=0; i<desCenters.size(); i++)
    {
        centers.push_back(exp(-cs.getGainA()*desCenters[i]));
    }
}

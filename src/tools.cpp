#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if (estimations.size() != ground_truth.size() || estimations.size() == 0 || ground_truth.size() == 0)
    {
        std::cout << "RMSE Calculation Error:Invalid estimations or ground_truth input data." << std::endl;
    }
    
    for (unsigned int n = 0; n < estimations.size(); ++n)
    {
        VectorXd residual = estimations[n] - ground_truth[n];
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    rmse = rmse/estimations.size();
    
    rmse = sqrt(rmse.array());
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    
    MatrixXd Hj(3,4);
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float c1 = px*px + py*py;
    
    
    if (fabs(c1) < 0.0001)
    {
        px = px + 0.001;
        py = py + 0.001;
        c1 = px*px + py*py;
    }
    
    float c2 = sqrt(c1);
    float c3 = c1*c2;
    float c4 = vx*py-vy*px;
    float c5 = vy*px-vx*py;
    
    Hj << px/c2,   py/c2,    0,     0,
         -py/c1,   px/c1,    0,     0,
          (py*c4)/c3,(px*c5)/c3,px/c2,py/c2;
    
    return Hj;
    
}

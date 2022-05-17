// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "cov_se_ard_noisy_inp.h"
#include <cmath>

namespace libgp
{
  
  CovSEardNoisyInp::CovSEardNoisyInp() {}
  
  CovSEardNoisyInp::~CovSEardNoisyInp() {}
  
  bool CovSEardNoisyInp::init(int n)
  {
    input_dim = n;
    param_dim = n+2;
    ell.resize(input_dim);
    loghyper.resize(param_dim);
    return true;
  }
  
  double CovSEardNoisyInp::get(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2)
  {
    double z = (x1-x2).cwiseQuotient((ell.diagonal().array()+sx2).matrix()).squaredNorm();
    double det = (ell.diagonal().prod())/(ell.diagonal().array()+sx2).prod();
    return sf2*sqrt(det)*exp(-0.5*z);
  }
  
  void CovSEardNoisyInp::grad(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, Eigen::VectorXd &grad)
  {
    Eigen::VectorXd z = (x1-x2).cwiseQuotient(ell).array().square();  
    double k = sf2*exp(-0.5*z.sum());
    grad.head(input_dim) = z * k;
    grad(input_dim) = 2.0 * k;
  }
  
  void CovSEardNoisyInp::set_loghyper(const Eigen::VectorXd &p)
  {
    CovarianceFunction::set_loghyper(p);
    for(size_t i = 0; i < input_dim; ++i) ell(i) = exp(loghyper(i));
    sf2 = exp(2*loghyper(input_dim));
    sx2 = exp(2*loghyper(input_dim+1));
//    std::cout<<"ell = "<<ell<<", sf2 = "<<sf2<<"\n";
  }
  
  std::string CovSEardNoisyInp::to_string()
  {
    return "CovSEardNoisyInp";
  }
}


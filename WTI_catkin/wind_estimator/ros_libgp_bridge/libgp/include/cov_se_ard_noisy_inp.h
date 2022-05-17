// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __COV_SE_ARD_NOISY_INP_H__
#define __COV_SE_ARD_NOISY_INP_H__

#include "cov.h"

namespace libgp
{
  
  /** Squared exponential covariance function with automatic relevance detection.
   *  Computes the squared exponential covariance considering noisy inputs
   *  \f$k_{SE}(x, y) := \sigma_f^2 \exp(-\frac{1}{2}(x-y)^T(\Lambda+\Sigma_x_star)^{-1}(x-y))\f$,
   *  with \f$\Lambda = diag(l_1^2, \dots, l_n^2)\f$ being the characteristic
   *  length scales, \f$\sigma_f\f$ describing the variability of the latent
   *  function, and Sigma_x_star=diag(sx. The parameters \f$l_1^2, \dots, l_n^2, \sigma_f, \sigma_x \f$ are expected
   *  in this order in the parameter array.
   *  @ingroup cov_group
   *  @author Mohit
   */
  class CovSEardNoisyInp : public CovarianceFunction
  {
  public:
    CovSEardNoisyInp ();
    virtual ~CovSEardNoisyInp ();
    bool init(int n);
    double get(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2);
    void grad(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, Eigen::VectorXd &grad);
    void set_loghyper(const Eigen::VectorXd &p);
    virtual std::string to_string();
  private:
    Eigen::VectorXd ell;
    double sx2;
    double sf2;
  };
  
}

#endif /* __COV_SE_ARD_NOISY_INP_H__ */


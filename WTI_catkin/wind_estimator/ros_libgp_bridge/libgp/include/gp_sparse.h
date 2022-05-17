// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2011, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __GP_SPARSE_H__
#define __GP_SPARSE_H__

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "gp.h"

namespace libgp {
  
  /** Sparse Gaussian process regression.
   *  @author Manuel Blum 
   *  Modified by Mohit Mehndiratta */
  class SparseGaussianProcess : public GaussianProcess
  {
  public:
    
    /** Create an instance of SparseGaussianProcess with given input dimensionality and covariance function. */
    SparseGaussianProcess (size_t input_dim, std::string covf_def, size_t _max_points);
    
    /** Create an instance of SparseGaussianProcess from file. */
    SparseGaussianProcess (const char * filename);
    
    /** Create an instance of SparseGaussianProcess from Copy constructor */
    SparseGaussianProcess (size_t _max_points, const GaussianProcess& gp);

    virtual ~SparseGaussianProcess ();
    
    virtual void sparsify();

    /** remove from particular location. */
    virtual void sparsify(size_t i);

    /** Predict variance of prediction for given input.
     *  @param x input vector
     *  @return predicted variance */
    virtual double var(const double x[]);

    /** Obtain mean and variance together to save computation */
    virtual std::pair<double, double> f_var(const double x[]);

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
      compute_KmmKuuKmu(SampleSet * sampleset_m, SampleSet * sampleset_u);

    virtual void add_pattern(const double x[], double y);

    size_t get_max_points();
    void set_max_points(size_t _max_points);

  protected:

    /** Prior mean for induced points */
    Eigen::VectorXd muu;
    
    /** Prior covariance matrix for induced points */
    Eigen::MatrixXd Suu;

    virtual void update_alpha();

    virtual void prior_meanVar(SampleSet * _sampleset);

    double minimize_fun(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);

    /** get the densest point in a list of samples
     * D is the dimensionality of the samples
     * N is the number of samples
     * distances is an NxN matrix where element (i,j) contains
         the (pre)computed distance between the ith and the jth samples */
    size_t get_most_dense_point(size_t D, size_t N, const Eigen::MatrixXd& distances);

  private:
    
    size_t max_points;

    /** Covariance function hyperparameters **/
    Eigen::VectorXd ell2;
    double sf2;
    double sqrt_det_ell2_by_det_ell2_p_sx2;
    double sqrt_det_ell2_by_det_ell2_p_2sx2;
    bool hyperpar_needs_update;

  };
}

#endif /* __GP_SPARSE_H__ */

// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "gp.h"
#include "cov_factory.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <ctime>

namespace libgp {
  
  const double log2pi = log(2*M_PI);
  const double initial_L_size = 1000;

  GaussianProcess::GaussianProcess ()
  {
      sampleset = NULL;
      cf = NULL;
  }

  GaussianProcess::GaussianProcess (size_t input_dim, std::string covf_def)
  {
    // set input dimensionality
    this->input_dim = input_dim;
    // create covariance function
    CovFactory factory;
    cf = factory.create(input_dim, covf_def);
    cf->loghyper_changed = 0;
    hyperpar_needs_update = true;
    sampleset = new SampleSet(input_dim);
    L.resize(initial_L_size, initial_L_size);
    mean_x = new Eigen::VectorXd(input_dim);
    mean_x->setZero(input_dim);
    std_dev_x = new Eigen::VectorXd(input_dim);
    std_dev_x->setOnes(input_dim);
    mean_y = 0;
    std_dev_y = 1;
  }
  
  GaussianProcess::GaussianProcess (const char * filename) 
  {
    sampleset = NULL;
    cf = NULL;
    int stage = 0;
    std::ifstream infile;
    double y;
    infile.open(filename);
    std::string s;
    double * x = NULL;
    L.resize(initial_L_size, initial_L_size);
    while (infile.good()) {
      getline(infile, s);
      // ignore empty lines and comments
      if (s.length() != 0 && s.at(0) != '#') {
        std::stringstream ss(s);
        if (stage > 5) {
          ss >> y;
          for(size_t j = 0; j < input_dim; ++j) {
            ss >> x[j];
          }
          add_pattern(x, y);
        } else if (stage == 0) {
          ss >> input_dim;
          sampleset = new SampleSet(input_dim);
          x = new double[input_dim];
        } else if (stage == 1) {
          CovFactory factory;
          cf = factory.create(input_dim, s);
          cf->loghyper_changed = 0;
          hyperpar_needs_update = true;     // Always compute the first time
        } else if (stage == 2) {
          Eigen::VectorXd params(cf->get_param_dim());
          for (size_t j = 0; j<cf->get_param_dim(); ++j) {
            ss >> params[j];
          }
          cf->set_loghyper(params);
        }
        // Added by Mohit
        else if (stage == 3)
        {
          mean_x = new Eigen::VectorXd(input_dim);
          for(size_t j = 0; j < input_dim; ++j)
            ss >> (*mean_x)(j);
        }
        else if (stage == 4)
        {
          std_dev_x = new Eigen::VectorXd(input_dim);
          for(size_t j = 0; j < input_dim; ++j)
            ss >> (*std_dev_x)(j);
          sampleset->standardize_variables_x(mean_x, std_dev_x);
        }
        else if (stage == 5)
        {
          ss >> mean_y;
          ss >> std_dev_y;
          sampleset->standardize_variables_y(mean_y, std_dev_y);
        }
        stage++;
      }
    }
    infile.close();
    if (stage < 6) {
      std::cerr << "fatal error while reading " << filename << std::endl;
      exit(EXIT_FAILURE);
    }
    delete [] x;
  }
  
  GaussianProcess::GaussianProcess(const GaussianProcess& gp)
  {
    this->input_dim = gp.input_dim;
    sampleset = new SampleSet(*(gp.sampleset));
    alpha = gp.alpha;
    k_star = gp.k_star;
    alpha_needs_update = gp.alpha_needs_update;
    L = gp.L;
    
    // copy covariance function
    CovFactory factory;
    cf = factory.create(gp.input_dim, gp.cf->to_string());
    cf->loghyper_changed = gp.cf->loghyper_changed;
    cf->set_loghyper(gp.cf->get_loghyper());
    hyperpar_needs_update = gp.hyperpar_needs_update;
    ell2 = gp.ell2;
    sf2 = gp.sf2;
    sqrt_det_ell2_by_det_ell2_p_sx2 = gp.sqrt_det_ell2_by_det_ell2_p_sx2;
    sqrt_det_ell2_by_det_ell2_p_2sx2 = gp.sqrt_det_ell2_by_det_ell2_p_2sx2;

    // copy standardized data
    this->mean_x = gp.mean_x;
    this->std_dev_x = gp.std_dev_x;
    this->mean_y = gp.mean_y;
    this->std_dev_y = gp.std_dev_y;
  }

  GaussianProcess::~GaussianProcess ()
  {
    // free memory
    if (sampleset != NULL) delete sampleset;
    if (cf != NULL) delete cf;
  }  
  
  double GaussianProcess::f(const double x[])
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
    // expected_f_star = k_star'*alpha
    return k_star.dot(alpha);    
  }

  // Added by Mohit
  double GaussianProcess::f_noisy_inp(const double x[], Eigen::VectorXd &sx2)
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star_noisy_inp(x_star, sx2);
    // expected_f_star = k_star'*alpha
    return k_star.dot(alpha);
  }
  
  double GaussianProcess::var(const double x[])
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // var_f_star = k_star_star - v'*v
    return cf->get(x_star, x_star) - v.dot(v);	
  }

  // Added by Mohit
  std::pair<double, double> GaussianProcess::f_var(const double x[])
  {
    if (sampleset->empty()) return std::make_pair(0, 0);
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // var_f_star = k_star_star - v'*v

    return std::make_pair(k_star.dot(alpha), cf->get(x_star, x_star) - v.dot(v));
  }

  // Added by Mohit: always to be called after f_noisy_inp() function
  double GaussianProcess::var_noisy_inp(const double x[], double mu_star, Eigen::VectorXd &sx2)
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    int n = sampleset->size();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);

    // TO BE: try to compute this loop faster by storing some matrices.
    // compute kernel matrix inverse: Q = inv(L'*L). It implies (Kmm + Sfm)^-1
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solveInPlace(Q);
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(Q);
    Q = Q - alpha * alpha.transpose();

    // making sx2 non-zero to avoid negative values for the vaiance
    if (sx2.isZero(0))
    {
      sx2.setConstant(input_dim,1e-6);
      if (cf->get_param_dim()-2 > 1)
        sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2.prod())/(ell2+2*sx2).prod());
      else
        sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2(0))/(ell2(0)+2*sx2(0)));
    }

    double sum = 0;
    for(size_t i = 0; i < sampleset->size(); ++i)
    {
//      for(size_t j = 0; j < sampleset->size(); ++j)
      for(size_t j = 0; j <= i; ++j)
      {
        if (i==j)
          sum = sum + Q(i,j)*exp(-0.5*(sampleset->x(i)-sampleset->x(j))
                                     .cwiseQuotient(2*ell2)
                                     .dot(sampleset->x(i)-sampleset->x(j)))
                           *exp(-0.5*(0.5*(sampleset->x(i)+sampleset->x(j))-x_star)
                                     .cwiseQuotient(0.5*ell2+sx2)
                                     .dot(0.5*(sampleset->x(i)+sampleset->x(j))-x_star));
        else
          sum = sum + 2*Q(i,j)*exp(-0.5*(sampleset->x(i)-sampleset->x(j))
                                      .cwiseQuotient(2*ell2)
                                      .dot(sampleset->x(i)-sampleset->x(j)))
                            *exp(-0.5*(0.5*(sampleset->x(i)+sampleset->x(j))-x_star)
                                      .cwiseQuotient(0.5*ell2+sx2)
                                      .dot(0.5*(sampleset->x(i)+sampleset->x(j))-x_star));
      }
    }
    return sf2 - sf2*sf2*sqrt_det_ell2_by_det_ell2_p_2sx2*sum - mu_star*mu_star;
  }



  void GaussianProcess::compute()
  {
    // can previously computed values be used?
    if (!cf->loghyper_changed) return;
    cf->loghyper_changed = false;
    int n = sampleset->size();
    // resize L if necessary
    if (n > L.rows()) L.resize(n + initial_L_size, n + initial_L_size);
    // compute kernel matrix (lower triangle)
    for(size_t i = 0; i < sampleset->size(); ++i) {
      for(size_t j = 0; j <= i; ++j) {
        L(i, j) = cf->get(sampleset->x(i), sampleset->x(j));
      }
    }
    // perform cholesky factorization: L = chol(L)
    //solver.compute(K.selfadjointView<Eigen::Lower>());
    L.topLeftCorner(n, n) = L.topLeftCorner(n, n).selfadjointView<Eigen::Lower>().llt().matrixL();
    alpha_needs_update = true;
  }
  
  void GaussianProcess::update_k_star(const Eigen::VectorXd &x_star)
  {
    k_star.resize(sampleset->size());
    for(size_t i = 0; i < sampleset->size(); ++i) {
      k_star(i) = cf->get(x_star, sampleset->x(i));
    }
  }

  // Added by Mohit
  void GaussianProcess::update_params_noisy_inp(Eigen::VectorXd &sx2)
  {
    ell2.setZero(input_dim);
    if (cf->get_param_dim()-2 > 1)
    {
      for(size_t i = 0; i < input_dim; ++i)
        ell2(i) = exp((cf->get_loghyper())(i))*exp((cf->get_loghyper())(i));
      sqrt_det_ell2_by_det_ell2_p_sx2 = sqrt((ell2.prod())/(ell2+sx2).prod());
      sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2.prod())/(ell2+2*sx2).prod());
    }
    else
    {
      for(size_t i = 0; i < input_dim; ++i)
        ell2(i) = exp((cf->get_loghyper())(0))*exp((cf->get_loghyper())(0));
      sqrt_det_ell2_by_det_ell2_p_sx2 = sqrt((ell2(0))/(ell2(0)+sx2(0)));
      sqrt_det_ell2_by_det_ell2_p_2sx2 = sqrt((ell2(0))/(ell2(0)+2*sx2(0)));
    }
    sf2 = exp(2*(cf->get_loghyper())(cf->get_param_dim()-2));
    hyperpar_needs_update = false;
//    std::cout<<"updated!!!\n";
//    std::cout<<"ell.array() = "<<sqrt(ell2.array())<<"\n";
//    std::cout<<"sf2 = "<<sf2<<"\n";
//    std::cout<<"sqrt_det_ell2_by_det_ell2_p_sx2 = "<<sqrt_det_ell2_by_det_ell2_p_sx2<<"\n";
//    std::cout<<"sqrt_det_ell2_by_det_ell2_p_2sx2 = "<<sqrt_det_ell2_by_det_ell2_p_2sx2<<"\n";
  }

  // Added by Mohit
  void GaussianProcess::update_k_star_noisy_inp(const Eigen::VectorXd &x_star, Eigen::VectorXd &sx2)
  {
    if (hyperpar_needs_update || cf->loghyper_changed)
    {
      update_params_noisy_inp(sx2);
    }

    k_star.resize(sampleset->size());
    for(size_t i = 0; i < sampleset->size(); ++i)
    {
      double z = (x_star-sampleset->x(i)).cwiseQuotient((ell2+sx2).matrix())
                                         .dot(x_star-sampleset->x(i));
      k_star(i) = sf2*sqrt_det_ell2_by_det_ell2_p_sx2*exp(-0.5*z);
    }
  }

  void GaussianProcess::update_alpha()
  {
    // can previously computed values be used?
    if (!alpha_needs_update) return;
    alpha_needs_update = false;
    alpha.resize(sampleset->size());
    // Map target values to VectorXd
    const std::vector<double>& targets = sampleset->y();
    Eigen::Map<const Eigen::VectorXd> y(&targets[0], sampleset->size());
    int n = sampleset->size();
    // alpha = inv(L)*y
    alpha = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(y);
    // alpha = inv(L')*alpha
    // TO BE: check which one is correct? Top one is the original one
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha); // Seems wrong!!!
//    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(alpha);
  }
  
  void GaussianProcess::add_pattern(const double x[], double y)
  {
    //std::cout<< L.rows() << std::endl;
#if 0
    sampleset->add(x, y);
    cf->loghyper_changed = true;
    alpha_needs_update = true;
    cached_x_star = NULL;
    return;
#else
    int n = sampleset->size();
    sampleset->add(x, y);
    // create kernel matrix if sampleset is empty
    if (n == 0) {
      L(0,0) = sqrt(cf->get(sampleset->x(0), sampleset->x(0)));
      cf->loghyper_changed = false;
    // recompute kernel matrix if necessary
    } else if (cf->loghyper_changed) {
      compute();
    // update kernel matrix 
    } else {
      Eigen::VectorXd k(n);
      for (int i = 0; i<n; ++i) {
        k(i) = cf->get(sampleset->x(i), sampleset->x(n));
      }
      double kappa = cf->get(sampleset->x(n), sampleset->x(n));
      // resize L if necessary
      if (sampleset->size() > static_cast<std::size_t>(L.rows())) {
        L.conservativeResize(n + initial_L_size, n + initial_L_size);
      }
      L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solveInPlace(k);
      L.block(n,0,1,n) = k.transpose();
      L(n,n) = sqrt(kappa - k.dot(k));
    }
    alpha_needs_update = true;
#endif
  }

  // Added by Mohit
  void GaussianProcess::remove_sample(size_t k)
  {
//    unsigned int n = sampleset->size();
    // remove data from sampleset
    sampleset->remove(k);
//    std::cout<<"sampleset->size() = "<<sampleset->size()<<"\n";

//    std::cout<<"K = \n"<<L.topLeftCorner(n, n)<<"\n";
//    cf->loghyper_changed = true;
//    compute();
//    std::cout<<"L.topLeftCorner(n-1, n-1) = \n"<<L.topLeftCorner(n-1, n-1)<<"\n";
//    std::cout<<"L.topLeftCorner(n, n) = \n"<<L.topLeftCorner(n, n)<<"\n";

//    // remove respective rows and columns from L matrix
//    Eigen::MatrixXd K = L.topLeftCorner(n, n);
//    remove_row(K, k);
//    remove_column(K, k);
//    std::cout<<"K = "<<K<<"\n";
//    L.topLeftCorner(n-1, n-1).triangularView<Eigen::Lower>() = K.topLeftCorner(n-1, n-1).triangularView<Eigen::Lower>();
//    std::cout<<"L.topLeftCorner(n, n) = \n"<<L.topLeftCorner(n, n)<<"\n";
//    K.resize(n-k-1, n-k-1);
//    std::cout<<"K = "<<K<<"\n";
//    std::cout<<"k = "<<k<<"\n";
//    std::cout<<"n-k-1 = "<<n-k-1<<"\n";
//    for(size_t i = k; i < sampleset->size(); ++i) {
//      for(size_t j = k; j <= i; ++j) {
//        K(i-k, j-k) = cf->get(sampleset->x(i), sampleset->x(j));
//        std::cout<<"(i,j) = ("<<i<<","<<j<<")\n";
//      }
//    }
//    std::cout<<"K = "<<K<<"\n";
//    // perform cholesky factorization: K = chol(K)
//    K.topLeftCorner(n-k-1, n-k-1) = K.topLeftCorner(n-k-1, n-k-1).selfadjointView<Eigen::Lower>().llt().matrixL();
//    std::cout<<"K = "<<K<<"\n";
////    L.bottomRightCorner(n-k-1,n-k-1) = K.topLeftCorner(n-k-1, n-k-1);
//    L.block(k,k,n-k-1,n-k-1) = K;


////    std::cout<<"K = "<<K<<"\n";
////    remove_row(K, i);
////    remove_column(K, i);
////    std::cout<<"K = "<<K<<"\n";
////    L.topLeftCorner(n-1, n-1).triangularView<Eigen::Lower>() = K.topLeftCorner(n-1, n-1).triangularView<Eigen::Lower>();
////    std::cout<<"L.topLeftCorner(n, n) = \n"<<L.topLeftCorner(n, n)<<"\n";
////    L.block(n-1,0,1,n) = Eigen::VectorXd::Zero(n).transpose();
//    std::cout<<"L.topLeftCorner(n, n) = \n"<<L.topLeftCorner(n, n)<<"\n";

    cf->loghyper_changed = true;
    alpha_needs_update = true;
  }


  // Added by Mohit
  void GaussianProcess::remove_row(Eigen::MatrixXd& matrix, size_t rowToRemove)
  {
      size_t numRows = matrix.rows() - 1;
      size_t numCols = matrix.cols();

      if (rowToRemove < numRows)
          matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

      matrix.conservativeResize(numRows, numCols);
  }

  // Added by Mohit
  void GaussianProcess::remove_column(Eigen::MatrixXd& matrix, size_t colToRemove)
  {
      size_t numRows = matrix.rows();
      size_t numCols = matrix.cols() - 1;

      if (colToRemove < numCols)
          matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

      matrix.conservativeResize(numRows, numCols);
  }

  bool GaussianProcess::set_y(size_t i, double y) 
  {
    if(sampleset->set_y(i,y)) {
      alpha_needs_update = true;
      return 1;
    }
    return false;
  }

  // Added by Mohit
  void GaussianProcess::standardize_sampleset(Eigen::VectorXd& _mean_x, Eigen::VectorXd& _std_dev_x)
  {
    // Standardize inputs
    sampleset->standardize_x();
    _mean_x = sampleset->mean_x();
    _std_dev_x = sampleset->std_dev_x();

    // Recompute the corresponding matrices
    cf->loghyper_changed = true;
    compute();
    update_alpha();

    // Storing values in gp object
    *mean_x = _mean_x;
    *std_dev_x = _std_dev_x;
  }

  // Added by Mohit
  void GaussianProcess::standardize_sampleset(double& _mean_y, double& _std_dev_y)
  {
    // Standardize targets
    sampleset->standardize_y();
    _mean_y = sampleset->mean_y();
    _std_dev_y = sampleset->std_dev_y();

    // Recompute the corresponding matrices
    cf->loghyper_changed = true;
    compute();
    update_alpha();

    // Storing values in gp object
    mean_y = _mean_y;
    std_dev_y = _std_dev_y;
  }

  // Added by Mohit
  void GaussianProcess::standardize_sampleset(Eigen::VectorXd& _mean_x, Eigen::VectorXd& _std_dev_x,
                                              double& _mean_y, double& _std_dev_y)
  {
    // Standardize inputs
    standardize_sampleset(_mean_x, _std_dev_x);

    // Standardize targets
    standardize_sampleset(_mean_y, _std_dev_y);
  }

  size_t GaussianProcess::get_sampleset_size()
  {
    return sampleset->size();
  }
  
  void GaussianProcess::clear_sampleset()
  {
    sampleset->clear();
  }

  // Added by Mohit
  SampleSet* GaussianProcess::get_sampleset()
  {
    return sampleset;
  }

  // Added by Mohit
  void GaussianProcess::set_sampleset(SampleSet& _sample_set)
  {
    this->clear_sampleset();
    sampleset = new SampleSet(_sample_set);

    // Recompute the corresponding matrices
    cf->loghyper_changed = true;
    compute();
    update_alpha();
  }
  
  // Added by Mohit
  bool GaussianProcess::get_hyperpar_needs_update()
  {
    return hyperpar_needs_update;
  }

  void GaussianProcess::write(const char * filename)
  {
    std::cout<<"writing to: "<<filename<<"\n";
    // output
    std::ofstream outfile;
    outfile.open(filename);
    time_t curtime = time(0);
    tm now=*localtime(&curtime);
    char dest[BUFSIZ]= {0};
    strftime(dest, sizeof(dest)-1, "%c", &now);
    outfile << "# " << dest << std::endl << std::endl
    << "# input dimensionality" << std::endl << input_dim << std::endl 
    << std::endl << "# covariance function" << std::endl 
    << cf->to_string() << std::endl << std::endl
    << "# log-hyperparameter" << std::endl;
    Eigen::VectorXd param = cf->get_loghyper();
    for (size_t i = 0; i< cf->get_param_dim(); i++) {
      outfile << std::setprecision(10) << param(i) << " ";
    }
    outfile << std::endl<< std::endl
    // Added by Mohit
    << "# mean for input values" << std::endl;
    for (size_t j=0; j<input_dim; ++j)
      outfile << std::setprecision(10) << (*mean_x)(j) << " ";
    outfile << std::endl<< "# std_dev for input values" << std::endl;
    for (size_t j=0; j<input_dim; ++j)
      outfile << std::setprecision(10) << (*std_dev_x)(j) << " ";
    outfile << std::endl<< std::endl<< "# mean and std_dev for target values" << std::endl
    << mean_y <<" "<< std_dev_y<< std::endl<< std::endl
    // End Mohit
    << "# data (target value in first column)" << std::endl;
    for (size_t i=0; i<sampleset->size(); ++i) {
      outfile << std::setprecision(10) << sampleset->y(i) << " ";
      for(size_t j = 0; j < input_dim; ++j) {
        outfile << std::setprecision(10) << sampleset->x(i)(j) << " ";
      }
      outfile << std::endl;
    }
    outfile.close();
  }
  
  CovarianceFunction & GaussianProcess::covf()
  {
    return *cf;
  }
  
  size_t GaussianProcess::get_input_dim()
  {
    return input_dim;
  }

  double GaussianProcess::log_likelihood()
  {
    compute();
    update_alpha();
    int n = sampleset->size();
    const std::vector<double>& targets = sampleset->y();
    Eigen::Map<const Eigen::VectorXd> y(&targets[0], sampleset->size());
    double det = 2 * L.diagonal().head(n).array().log().sum();
    return -0.5*y.dot(alpha) - 0.5*det - 0.5*n*log2pi;
  }

  Eigen::VectorXd GaussianProcess::log_likelihood_gradient() 
  {
    compute();
    update_alpha();
    size_t n = sampleset->size();
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(cf->get_param_dim());
    Eigen::VectorXd g(grad.size());
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n, n);

    // compute kernel matrix inverse: W = inv(L'*L). It implies (Kmm + Sfm)^-1
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solveInPlace(W);
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(W);

    W = alpha * alpha.transpose() - W;

    for(size_t i = 0; i < n; ++i) {
      for(size_t j = 0; j <= i; ++j) {
        cf->grad(sampleset->x(i), sampleset->x(j), g);
        if (i==j) grad += W(i,j) * g * 0.5;
        else      grad += W(i,j) * g;
      }
    }

    return grad;
  }
}

// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

// Modified by Mohit Mehndiratta <mohit005@e.ntu.edu.sg> 2019

#include "gp_sparse.h"

namespace libgp {
  
  const double log2pi = log(2*M_PI);

  SparseGaussianProcess::SparseGaussianProcess (size_t input_dim, std::string covf_def, size_t _max_points) : GaussianProcess(input_dim, covf_def)
  {
    max_points = _max_points;
  }
  
  SparseGaussianProcess::SparseGaussianProcess (const char * filename) : GaussianProcess(filename)
  {
    max_points = this->sampleset->size();
  }
  
  SparseGaussianProcess::SparseGaussianProcess (size_t _max_points, const GaussianProcess& gp) : GaussianProcess(gp)
  {
    max_points = _max_points;
  }

  SparseGaussianProcess::~SparseGaussianProcess () {}  
  
  void SparseGaussianProcess::sparsify()
  {
    SampleSet * sampleset_indpts = new SampleSet(*sampleset);

    /// compute big distance matrix to avoid re-computations
    Eigen::MatrixXd distances = Eigen::MatrixXd::Zero(sampleset->size(), sampleset->size());
    for(size_t i = 0; i < sampleset->size(); ++i) {
      for(size_t j = 0; j < sampleset->size(); ++j) {
        distances(i, j) = (sampleset->x(i) - sampleset->x(j)).norm();
      }
    }

    while(sampleset_indpts->size() > max_points) {
      int k = get_most_dense_point(input_dim, sampleset_indpts->size(), distances);
      /// sanity check
      if (k < 0)
          break;
      sampleset_indpts->remove(k);

      remove_column(distances, k);
      remove_row(distances, k);
    }

    // compute prior mean and covariance for induced points
    prior_meanVar(sampleset_indpts);

    // replace measurements with induced points
    delete sampleset;
    sampleset = sampleset_indpts;

    cf->loghyper_changed = true;
  }

  void SparseGaussianProcess::sparsify(size_t i)
  {
    SampleSet * sampleset_indpts = new SampleSet(*sampleset);

    sampleset_indpts->remove(i);

    // compute prior mean and covariance for induced points
    prior_meanVar(sampleset_indpts);

    // replace measurements with induced points
    delete sampleset;
    sampleset = sampleset_indpts;

    cf->loghyper_changed = true;
  }

  size_t SparseGaussianProcess::get_most_dense_point(size_t D, size_t N,
                                                     const Eigen::MatrixXd& distances)
  {
    double min_dist = std::numeric_limits<double>::max();
    size_t denser = -1;

    for(size_t i = 0; i < N; ++i) {
      double dist = 0.;
      std::vector<double> neighbors(N);
      Eigen::VectorXd::Map(neighbors.data(), neighbors.size()) = distances.row(i);
      /// remove self distance
      neighbors.erase(neighbors.begin() + i);

      std::partial_sort(neighbors.begin(), neighbors.begin() + D, neighbors.end());

      for (size_t j = 0; j < D; j++) {
          dist += neighbors[j];
      }
      if (dist < min_dist) {
          min_dist = dist;
          denser = i;
      }
    }
    return denser;
  }

  void SparseGaussianProcess::prior_meanVar(SampleSet * _sampleset)
  {
    size_t n = sampleset->size();
    size_t nu = _sampleset->size();

    Eigen::MatrixXd Kmm(n, n), Kuu(nu, nu), Kmu(n, nu);
    std::tie(Kmm, Kuu, Kmu) = compute_KmmKuuKmu(sampleset, _sampleset);

    // muu = Kmu'*inv(Kmm + Sfm)*fmh
    muu.resize(nu);
    // Update alpha using target values
    this->GaussianProcess::update_alpha();
    muu = Kmu.transpose()*alpha;
    // alpha to be updated again with muu for prediction
    alpha_needs_update = true;

    Eigen::MatrixXd Q(nu, nu);
    // Q = Kmu'inv(L'L)*Kmu
    Q = Kmu.transpose()*(Kmm.selfadjointView<Eigen::Lower>().llt().solve(Kmu));

    Suu.setZero(nu, nu);
//    Suu = Kuu - Q.topLeftCorner(n, n);
    // Suu = Q, to be helpful later
    Suu = Q;

  }

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    SparseGaussianProcess::compute_KmmKuuKmu(SampleSet * sampleset_m, SampleSet * sampleset_u)
  {
    size_t n = sampleset_m->size();
    size_t nu = sampleset_u->size();

    // compute kernel matrix (lower triangle)
    Eigen::MatrixXd Kmm(n, n);
    Kmm.setZero(n, n);
    Kmm = (L.topLeftCorner(sampleset->size(), sampleset->size())*L.topLeftCorner(sampleset->size(), sampleset->size()).transpose()).triangularView<Eigen::Lower>();

    // compute kernel matrix (lower triangle)
    Eigen::MatrixXd Kuu(nu, nu);
    Kuu.setZero(nu, nu);
    for(size_t i = 0; i < nu; ++i) {
      for(size_t j = 0; j <= i; ++j) {
        (Kuu)(i, j) = cf->get(sampleset_u->x(i), sampleset_u->x(j));
      }
    }

    // compute kernel matrix
    Eigen::MatrixXd Kmu(n, nu);
    Kmu.setZero(n, nu);
    for(size_t i = 0; i < n; ++i) {
      for(size_t j = 0; j < nu; ++j) {
        // TO BE: check if the values for noise covariance is coming up in the below evaluation
        (Kmu)(i, j) = cf->get(sampleset_m->x(i), sampleset_u->x(j));
      }
    }

    return std::make_tuple(Kmm, Kuu, Kmu);
  }

  void SparseGaussianProcess::update_alpha()
  {
    // can previously computed values be used?
    if (!alpha_needs_update) return;
    alpha_needs_update = false;
    alpha.resize(sampleset->size());
    int n = sampleset->size();

    // alpha = inv(L)*muu
    alpha = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(muu);
    // alpha = inv(L')*alpha
    // TO BE: check which one is correct? Top one is the original one
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha); // Seems wrong!!!
//    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(alpha);
  }

  double SparseGaussianProcess::var(const double x[])
  {
    if (sampleset->empty()) return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);

    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // v = inv(L')*v
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(v);
    return cf->get(x_star, x_star) - v.dot(Suu*v);
  }
  
  std::pair<double, double> SparseGaussianProcess::f_var(const double x[])
  {
    if (sampleset->empty()) return std::make_pair(0, 0);
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha();
    update_k_star(x_star);
    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // v = inv(L')*v
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(v);

    return std::make_pair(k_star.dot(alpha), cf->get(x_star, x_star) - v.dot(Suu*v));
  }

  /** add sample and update the GP. If the number of samples is bigger than
   *  the desired maximum points, we re-sparsify and re-compute the GP */
  void SparseGaussianProcess::add_pattern(const double x[], double y)
  {
    this->GaussianProcess::add_pattern(x, y);

    // if we surpassed the maximum points, re-sparsify and recompute
    if (sampleset->size() > max_points){
      sparsify();
    }
  }

  size_t SparseGaussianProcess::get_max_points()
  {
    return max_points;
  }

  void SparseGaussianProcess::set_max_points(size_t _max_points)
  {
    max_points = _max_points;
  }

}

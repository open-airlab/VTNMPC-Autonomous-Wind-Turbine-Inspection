// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __SAMPLESET_H__
#define __SAMPLESET_H__

#include <Eigen/Dense>
#include <vector>
// Added by Mohit
#include <iostream>

namespace libgp {
  
  /** Container holding training patterns.
   *  @author Manuel Blum */
  class SampleSet
  {
  public:
    /** Constructor.
     *  @param input_dim dimensionality of input vectors */
    SampleSet (int input_dim);

    /** Copy constructor */
    SampleSet ( const SampleSet& ss );

    /** Destructor. */    
    virtual ~SampleSet();
    
    /** Add input-output pattern to sample set.
     *  @param x input array
     *  @param y target value */
    void add(const double x[], double y);
    void add(const Eigen::VectorXd x, double y);
    
    /** Get input vector at index k. */
    const Eigen::VectorXd & x (size_t k);

    /** Get target value at index k. */
    double y (size_t k);

    /** Set target value at index i. */
    bool set_y(size_t i, double y);

    /** Get reference to vector of target values. */
    const std::vector<double>& y();
    
    /** Get number of samples. */
    size_t size();
    
    /** Clear sample set. */
    void clear();
    
    /** Check if sample set is empty. */
    bool empty ();

    // Added by Mohit
    /** Remove input-output data at i. */
    void remove(size_t i);

    // Added by Mohit
    /** Remove input-output data within range r. */
    void remove(std::vector<int> r);

    // Added by Mohit
    /** Standardize the target values. */
    void standardize_y();

    // Added by Mohit
    /** Standardize the input values. */
    void standardize_x();

    // Added by Mohit
    /** Get target values mean. */
    const double mean_y();

    // Added by Mohit
    /** Get target values standard deviation. */
    const double std_dev_y();

    // Added by Mohit
    /** Get input values mean at i. */
    const double mean_x(size_t i);

    // Added by Mohit
    /** Get input values standard deviation at i. */
    const double std_dev_x(size_t i);

    // Added by Mohit
    /** Get input values mean. */
    const Eigen::VectorXd mean_x();

    // Added by Mohit
    /** Get input values standard deviation. */
    const Eigen::VectorXd std_dev_x();

    // Added by Mohit
    /** Set target values mean and std_dev. */
    void standardize_variables_y(double& mean, double& std_dev);

    // Added by Mohit
    /** Set input values mean and std_dev. */
    void standardize_variables_x(Eigen::VectorXd *& mean, Eigen::VectorXd *& std_dev);

  private:

    /** Container holding input vectors. */
    std::vector<Eigen::VectorXd *> inputs;
    
    /** Container holding target values. */
    std::vector<double> targets;
    
    // Added by Mohit
    /** Variable holding the standardizing constant for the target values.*/
    double mean_targets;
    double std_dev_targets;

    // Added by Mohit
    /** Variable holding the standardizing constant for the input values.*/
    Eigen::VectorXd * mean_inputs;
    Eigen::VectorXd * std_dev_inputs;

    /** Dimensionality of input vectors. */
    size_t input_dim;
    
    /** Number of samples. */
    size_t n;
  };
}

#endif /* __SAMPLESET_H__ */

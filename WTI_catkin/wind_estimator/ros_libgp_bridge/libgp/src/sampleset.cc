// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "sampleset.h"
#include <Eigen/StdVector>
#include <numeric>

namespace libgp {
  
  SampleSet::SampleSet (int input_dim)
  {
    this->input_dim = input_dim;
    n = 0;
  }
  
  SampleSet::SampleSet ( const SampleSet& ss )
  {
    // shallow copies
    n = ss.n;
    input_dim = ss.input_dim;
    targets = ss.targets;

    // deep copy needed for vector of pointers
    for (size_t i=0; i<ss.inputs.size(); ++i)
    {
      Eigen::VectorXd * sample_to_store = new Eigen::VectorXd(input_dim);
      *sample_to_store = *ss.inputs.at(i);
      inputs.push_back(sample_to_store);
    }
  }

  SampleSet::~SampleSet() 
  {
    clear();
  }
  
  void SampleSet::add(const double x[], double y)
  {
    Eigen::VectorXd * v = new Eigen::VectorXd(input_dim);
    for (size_t i=0; i<input_dim; ++i) (*v)(i) = x[i];
    inputs.push_back(v);
    targets.push_back(y);
    assert(inputs.size()==targets.size());
    n = inputs.size();
  }
  
  void SampleSet::add(const Eigen::VectorXd x, double y)
  {
    Eigen::VectorXd * v = new Eigen::VectorXd(x);
    inputs.push_back(v);
    targets.push_back(y);
    assert(inputs.size()==targets.size());
    n = inputs.size();
  }

  // Added by Mohit
  void SampleSet::remove(size_t i)
  {
    inputs.erase(inputs.begin() + i);
    targets.erase(targets.begin() + i);
    assert(inputs.size()==targets.size());
    n = inputs.size();
//    std::cout<<"----------------\n";
//    std::cout<<"size = "<<n<<"\n";
//    std::cout<<"removed element "<<i<<" from sampleset!\n";
//    std::cout<<"----------------\n";
  }
  // Added by Mohit
  void SampleSet::remove(std::vector<int> r)
  {
    for (size_t i=0; i<r.size(); i++)
      SampleSet::remove( static_cast <size_t>(r.at(i)) -i );
  }
  
  // Added by Mohit
  void SampleSet::standardize_y()
  {
    mean_targets = accumulate( targets.begin(), targets.end(), 0.0)/targets.size();
    std::vector<double> v = targets;
    for(auto& element : v)
    {
      element -= mean_targets;
      element = element*element;
    }
    std_dev_targets = std::sqrt(accumulate( v.begin(), v.end(), 0.0)/v.size());

    for(auto& element : targets)
      element = (element - mean_targets)/std_dev_targets;
  }

  // Added by Mohit
  void SampleSet::standardize_x()
  {

    mean_inputs = new Eigen::VectorXd(input_dim);
    std_dev_inputs = new Eigen::VectorXd(input_dim);
    std::vector<double> x;
    for (size_t i=0; i<input_dim; ++i)
    {
      x.resize(inputs.size());
      for (size_t j=0; j<inputs.size(); j++)
        x[j] = (*inputs.at(j))(i);
      (*mean_inputs)(i) = accumulate( x.begin(), x.end(), 0.0)/x.size();
      std::vector<double> v = x;
      for(auto& element : v)
      {
        element -= (*mean_inputs)(i);
        element = element*element;
      }
      (*std_dev_inputs)(i) = std::sqrt(accumulate( v.begin(), v.end(), 0.0)/v.size());

      for (size_t j=0; j<inputs.size(); j++)
        (*inputs.at(j))(i) = ((*inputs.at(j))(i) - (*mean_inputs)(i))/(*std_dev_inputs)(i);
    }
  }

  const Eigen::VectorXd & SampleSet::x(size_t k)
  {
    return *inputs.at(k);
  }

  double SampleSet::y(size_t k)
  {
    return targets.at(k);
  }

  const std::vector<double>& SampleSet::y() 
  {
    return targets;
  }

  bool SampleSet::set_y(size_t i, double y)
  {
    if (i>=n) return false;
    targets[i] = y;
    return true;
  }

  size_t SampleSet::size()
  {
    return n;
  }
  
  void SampleSet::clear()
  {
    while (!inputs.empty()) {
      delete inputs.back();
      inputs.pop_back();
    }    
    n = 0;
    targets.clear();
  }
  
  bool SampleSet::empty ()
  {
    return n==0;
  }

  // Added by Mohit
  const double SampleSet::mean_y()
  {
    return mean_targets;
  }

  // Added by Mohit
  const double SampleSet::std_dev_y()
  {
    return std_dev_targets;
  }

  // Added by Mohit
  const double SampleSet::mean_x(size_t i)
  {
    return (*mean_inputs)(i);
  }

  // Added by Mohit
  const double SampleSet::std_dev_x(size_t i)
  {
    return (*std_dev_inputs)(i);
  }

  // Added by Mohit
  const Eigen::VectorXd SampleSet::mean_x()
  {
    return *mean_inputs;
  }

  // Added by Mohit
  const Eigen::VectorXd SampleSet::std_dev_x()
  {
    return *std_dev_inputs;
  }

  // Added by Mohit
  void SampleSet::standardize_variables_y(double& mean, double& std_dev)
  {
    mean_targets = mean;
    std_dev_targets = std_dev;
  }

  // Added by Mohit
  void SampleSet::standardize_variables_x(Eigen::VectorXd *& mean, Eigen::VectorXd *& std_dev)
  {
    mean_inputs = mean;
    std_dev_inputs = std_dev;
  }
}

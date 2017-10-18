#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // calculate root mean squared error
  VectorXd rsmes(4);
  rsmes << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rsmes;
  }

  for (int i = 0; i < estimations.size(); i++) {
    VectorXd item = (estimations[i] - ground_truth[i]);
    VectorXd me = item.array() * item.array();
    rsmes << rsmes + me;
  }

  // mean
  rsmes = rsmes / estimations.size();

  // square root
  rsmes = rsmes.array().sqrt();

  return rsmes;
}

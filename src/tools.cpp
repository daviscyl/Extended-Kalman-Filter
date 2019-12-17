#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() == 0 || ground_truth.size() != estimations.size()) {
		return rmse;
	}

	// accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	// calculate the mean
	rmse = rmse / ground_truth.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
	/**
   * TODO:
   * Calculate a Jacobian here.
   */
	int n = x_state.size();
	MatrixXd jacoian(n, n);

	return jacoian;
}

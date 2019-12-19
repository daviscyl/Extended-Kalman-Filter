#include "FusionEKF.h"
#include <math.h>
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cos;
using std::cout;
using std::endl;
using std::sin;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
	    0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
	    0, 0.0009, 0,
	    0, 0, 0.09;

	H_laser_ << 1, 0, 0, 0,
	    0, 1, 0, 0;

	// the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
	    0, 1, 0, 1,
	    0, 0, 1, 0,
	    0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
	/**
	 * Initialization
	*/
	if (!is_initialized_) {
		// initialize state vector x
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// Convert radar from polar to cartesian coordinates and initialize state.
			float rho = measurement_pack.raw_measurements_[0];
			float theta = measurement_pack.raw_measurements_[1];

			ekf_.x_ << rho * sin(theta),
			    rho * cos(theta),
			    0,
			    0;
		} else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			ekf_.x_ << measurement_pack.raw_measurements_[0],
			    measurement_pack.raw_measurements_[1],
			    0,
			    0;
		}

		// initialize state covariance matrix P
		ekf_.P_ = MatrixXd(4, 4);
		ekf_.P_ << 1, 0, 0, 0,
		    0, 1, 0, 0,
		    0, 0, 1000, 0,
		    0, 0, 0, 1000;

		// initialize time stamp
		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/**
	 * Prediction
	*/

	/**
	 * 	Update the state transition matrix F according to the new elapsed time.
	 * Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	float noise_ax = 9;
	float noise_ay = 9;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << pow(dt, 4) * noise_ax / 4, 0, pow(dt, 3) * noise_ax / 2, 0,
	    0, pow(dt, 4) * noise_ay / 4, 0, pow(dt, 3) * noise_ay / 2,
	    pow(dt, 3) * noise_ax / 2, 0, pow(dt, 2) * noise_ax, 0,
	    0, pow(dt, 3) * noise_ay / 2, 0, pow(dt, 2) * noise_ay;

	ekf_.Predict();

	/**
	 * Update
	*/

	/**
	 * -Use the sensor type to perform the update step.
	 * - Update the state and covariance matrices.
	*/

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
	} else {
		// Laser updates
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
	}

	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

	// print the output
	cout << "\nx_:\n" << ekf_.x_ << endl;
	cout << "\nP_:\n" << ekf_.P_ << endl;
	cout << "\n------------------------------------------" << endl;
}

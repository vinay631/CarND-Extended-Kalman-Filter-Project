#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	//the initial transition matrix F_
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
	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
		/**
    TODO:
		 * Initialize the state ekf_.x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement

		ekf_.x_ = VectorXd(4);
		float px = 0;
		float py = 0;
		float vx = 0;
		float vy = 0;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
      Convert radar from polar to cartesian coordinates and initialize state.
			 */

			float ro = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float ro_dot = measurement_pack.raw_measurements_(2);
			px = ro * cos(phi);
			py = ro * sin(phi);
			vx = ro_dot * cos(phi);
			vy = ro_dot * sin(phi);
			//state covariance matrix P initialization
			ekf_.P_ << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1000, 0,
					0, 0, 0, 1000;

		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
      Initialize state.
			 */
			px = measurement_pack.raw_measurements_[0];
			py = measurement_pack.raw_measurements_[1];
			//state covariance matrix P initialization
			ekf_.P_ << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1000, 0,
					0, 0, 0, 1000;
		}
		previous_timestamp_ = measurement_pack.timestamp_;
		ekf_.x_ << px, py, vx, vy;

		// done initializing, no need to predict or update
		if(px == 0){
			//will start true kalman state initialization till records whose px is not zero arrives
			return;
		}
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	/**
   TODO:
	 * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 */

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	float noise_ax = 9;
	float noise_ay = 9;


	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;

	//Set the process covariance matrix Q
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
   TODO:
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		R_radar_ << 0.09, 0, 0,
				0, 0.0009,0,
				0,0,0.09;
		Tools tools;
		try{
			Hj_ = tools.CalculateJacobian(ekf_.x_);
		}catch(StringException & caught){
			cout<<"Got "<<caught.what()<<std::endl;
			return;
		}


		ekf_.R_ = R_radar_;
		ekf_.H_ = Hj_;
		//Call the Kalman Filter update() function
		// with the most recent raw measurements_
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
		// Laser updates
		R_laser_ << 0.0225, 0,
				0, 0.0225;
		H_laser_<< 1, 0, 0, 0,
				0, 1, 0, 0;

		ekf_.R_ = R_laser_;
		ekf_.H_ = H_laser_;
		//Call the Kalman Filter update() function
		// with the most recent raw measurements_
		ekf_.Update(measurement_pack.raw_measurements_);
	}
}

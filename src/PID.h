#ifndef PID_H
#define PID_H

class PID {
private:
	// variable for cumulative Squared Error
	double squared_errs;
	// initialize best Mean Squared Error
	double best_mse;
	// number of iterations between coefficient updates
	unsigned int num_iter;
	// counter for iterations between coefficient updates
	unsigned int counter;
	// index for coefficient updates
	unsigned int index;
	// state for coefficient updates
	bool incr_added;
	// increments for coefficient updates
	double incr[3];

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  // initial values
  double Kp0;
  double Ki0;
  double Kd0;
  // updates
  double coeffs[3];
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, unsigned int num_iter);

  /*
  * Update the PID coefficients.
  */
  void UpdateCoeffs(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle PID coefficients.
  */
  void Twiddle();

  /*
  * Ziegler-Nichols method to approximate PID coefficients.
  */
  void ZieglerNichols();
};

#endif /* PID_H */

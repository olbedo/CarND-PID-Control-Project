#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class Twiddle {
private:
	// max absolute error
	double max_abs_error;
	// best max absolute error
	double best_max_error;
	// variable for cumulative Squared Error
	double squared_errs;
	// best Mean Squared Error
	double best_mse;
	// max speed
	double max_speed;
	// number of iterations between coefficient updates
	unsigned int num_iter;
	// counter for iterations between coefficient updates
	unsigned int counter;
	// index for coefficient updates
	unsigned int index;
	// state for coefficient updates
	bool incr_added;

public:
  /*
  * Coefficients
  */ 
  // updates
  std::vector<double> coeffs;
  // increments for coefficient updates
  std::vector<double> incr;
  // number of coefficients
  unsigned int num_coeffs;
  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize PID.
  */
  void Init(std::vector<double> coeffs, std::vector<double> incr, unsigned int num_coeffs, unsigned int num_iter);

  /*
  * Twiddle PID coefficients.
  */
  void Update(double error, double speed);

};

#endif /* TWIDDLE_H */

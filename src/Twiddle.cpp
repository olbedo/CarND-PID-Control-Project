#include <iostream>
#include <math.h>
#include "Twiddle.h"

using namespace std;

/*
* Twiddle class.
*/

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(std::vector<double> coeffs, std::vector<double> incr, unsigned int num_coeffs, unsigned int num_iter) {
	// save initial PID coefficients
    Twiddle::coeffs = coeffs;

	// initialize increments for coefficient updates
	Twiddle::incr = incr;

	// number of coefficients
	Twiddle::num_coeffs = num_coeffs;
	// initialize max absolute error
	max_abs_error = -1.;
	// initialize best max absolute error
	best_max_error = 0.;
	// initialize variable for cumulative Squared Error
	squared_errs = 0.;
	// initialize best Mean Squared Error
	best_mse = -1.;
	// max speed
	max_speed = 0;
	// number of iterations between coefficient updates
	Twiddle::num_iter = num_iter;
	// initialize counter for iterations between coefficient updates
	counter = 0;
	// initialize index for coefficient updates
	index = 0;
	// initialize state for coefficient updates
	incr_added = true;

}

void Twiddle::Update(double error, double speed) {
	if (fabs(error) > max_abs_error) max_abs_error = fabs(error);
	if (speed > max_speed) max_speed = speed;
    if (best_mse == -1.) {
        if (counter == 0){
        	for (unsigned int i = 0; i < num_coeffs; i++) {
                cout << "  " << coeffs[i];
        	}
        	cout << "  ";
        	for (unsigned int i = 0; i < num_coeffs; i++) {
                cout << "  " << incr[i];
        	}
        	//cout << endl;
        }
        // iterate num_iter times and accumulate squared errors
        if (counter < num_iter+1000){
            if (counter >= 1000) {
				squared_errs += error * error;
			}
            ++counter;
            return;
        }
        // initialize best Mean Squared Error
        best_mse = squared_errs / num_iter;
		best_max_error = max_abs_error;
		cout << "    " << best_mse << "  "  << max_abs_error <<  "  "  << max_speed << endl;
        cout << "Best Max Error: " << best_max_error << "    (Best MSE: " << best_mse << ")" << endl;
        // add increment to coefficient
        coeffs[index] += incr[index];
    	for (unsigned int i = 0; i < num_coeffs; i++) {
            cout << "  " << coeffs[i];
    	}
		cout << "  ";
		for (unsigned int i = 0; i < num_coeffs; i++) {
			cout << "  " << incr[i];
		}
    	//cout << endl;
        // reset counter and squared errors
        counter = 0;
        squared_errs = 0.;
        return;
    }
    else if (counter < num_iter+100){
        if (counter >= 100) squared_errs += error * error;
        ++counter;
        return;
    }
    double mse = squared_errs / num_iter;
	cout << "    " << mse << "  "  << max_abs_error <<  "  "  << max_speed << endl;

    // if MSE improved, save it
	if (mse < best_mse) best_mse = mse;
	
    // if max error improved, keep update and increase increment
    if (max_abs_error < best_max_error) {
		best_max_error = max_abs_error;
        cout << "Best Max Error: " << best_max_error << "    (Best MSE: " << best_mse << ")" << endl;
        incr[index] *= 1.1;
        index = (index + 1) % num_coeffs;
		if (incr[index] == 0.0) index = (index + 1) % num_coeffs;
        // add increment to coefficient
        coeffs[index] += incr[index];
        incr_added = true;
    // if max error worsened ...
    } else {
        if (incr_added == true) {
            // ... undo update and subtract increment from coefficient
            coeffs[index] -= 2 * incr[index];
            incr_added = false;
        } else {
            // ... undo update and decrease increment
            coeffs[index] += incr[index];
            incr[index] *= 0.9;
            index = (index + 1) % num_coeffs;
			if (incr[index] == 0.0) index = (index + 1) % num_coeffs;
            // add increment to coefficient
            coeffs[index] += incr[index];
            incr_added = true;
        }
    }
	for (unsigned int i = 0; i < num_coeffs; i++) {
        cout << "  " << coeffs[i];
	}
	cout << "  ";
	for (unsigned int i = 0; i < num_coeffs; i++) {
		cout << "  " << incr[i];
	}
	//cout << endl;
    // reset counter and squared errors
    counter = 0;
	max_abs_error = 0.;
    squared_errs = 0.;
	max_speed = 0;
}


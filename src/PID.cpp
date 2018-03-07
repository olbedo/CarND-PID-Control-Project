#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, unsigned int num_iter) {
	// save initial PID coefficients
	Kp0 = Kp;
	Ki0 = Ki;
	Kd0 = Kd;
    coeffs[0] = Kp;
    coeffs[1] = Ki;
    coeffs[2] = Kd;
	// initialize errors
	p_error = 0.;
	i_error = 0.;
	d_error = 0.;

	// initialize increments for coefficient updates
	incr[0] = 0.05;
	incr[1] = 0.001;
	incr[2] = 1.0;
	// initialize variable for cumulative Squared Error
	squared_errs = 0.;
	// initialize best Mean Squared Error
	best_mse = -1.;
	// number of iterations between coefficient updates
	PID::num_iter = num_iter;
	// initialize counter for iterations between coefficient updates
	counter = 0;
	// initialize index for coefficient updates
	index = 0;
	// initialize state for coefficient updates
	incr_added = true;

}

void PID::UpdateCoeffs(double Kp, double Ki, double Kd) {
	// update differential error
    coeffs[0] = Kp;
    coeffs[1] = Ki;
    coeffs[2] = Kd;
}

void PID::UpdateError(double cte) {
	// update differential error
	if (i_error != 0.) {  // for the first iteration d_error = 0
		d_error = cte - p_error;
	}
	// update proportional error
	p_error = cte;
	// update integral error
	i_error += cte;
}

double PID::TotalError() {
	return -coeffs[0] * p_error - coeffs[1] * i_error - coeffs[2] * d_error;
}

void PID::Twiddle() {
    if (best_mse == -1.) {
        if (counter == 0){
            cout << "Kp: " << coeffs[0] << "  Ki: " << coeffs[1] << "  Kd: " << coeffs[2] << endl;
        }
        // iterate num_iter times and accumulate squared errors
        if (counter < num_iter+1000){
            if (counter >= 1000) squared_errs += p_error * p_error;
            ++counter;
            return;
        }
        // initialize best Mean Squared Error
        best_mse = squared_errs / num_iter;
        cout << "Best MSE: " << best_mse << endl;
        // add increment to coefficient
        coeffs[index] += incr[index];
        cout << "Kp: " << coeffs[0] << "  Ki: " << coeffs[1] << "  Kd: " << coeffs[2] << endl;
        // reset counter and squared errors
        counter = 0;
        squared_errs = 0.;
        return;
    }
    else if (counter < num_iter+100){
        if (counter >= 100) squared_errs += p_error * p_error;
        ++counter;
        return;
    }
    double mse = squared_errs / num_iter;
    // if MSE improved, keep update and increase increment
    if (mse < best_mse) {
        best_mse = mse;
        cout << "Best MSE: " << best_mse << endl;
        incr[index] *= 1.25;
        index = (index + 1) % 3;
        // add increment to coefficient
        coeffs[index] += incr[index];
        incr_added = true;
    // if MSE worsened ...
    } else {
        if (incr_added == true) {
            // ... undo update and subtract increment from coefficient
            coeffs[index] -= 2 * incr[index];
            incr_added = false;
        } else {
            // ... undo update and decrease increment
            coeffs[index] += incr[index];
            incr[index] *= 0.8;
            index = (index + 1) % 3;
            // add increment to coefficient
            coeffs[index] += incr[index];
            incr_added = true;
        }
    }
    cout << "Kp: " << coeffs[0] << "  Ki: " << coeffs[1] << "  Kd: " << coeffs[2] << endl;
    // reset counter and squared errors
    counter = 0;
    squared_errs = 0.;
}

void PID::ZieglerNichols() {
    cout << counter << "  Kp: " << coeffs[0] << "  CTE: " << p_error << endl;
    if ( (counter != 0) && (counter % 1000 == 0) )
        coeffs[0] += 0.1;
    ++counter;
    
}

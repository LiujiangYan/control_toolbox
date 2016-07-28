#ifndef CONTROL_TOOLBOX__DITHER_H
#define CONTROL_TOOLBOX__DITHER_H

#include <cstdlib>
#include <ctime>
#include <math.h>
#include <iostream>

using namespace std;

class Dither{
public:
    Dither() : amplitude_(0), has_saved_value_(false){
        
    }

	/*!
	* \brief Destructor.
	*/
	~Dither(){}

	/*!
	* \brief Get next Gaussian white noise point. Called in RT loop.
	*\return White noise of given amplitude.
	*/
	double update(){
		if (has_saved_value_){
			has_saved_value_ = false;
			return saved_value_;
		}

		double v1, v2, r;
		for (int i=0; i<100; i++){
			v1 = 2.0*erand48(seed_) - 1.0; //[-1,1]
			v2 = 2.0*erand48(seed_) - 1.0; 
			r = v1*v1 + v2*v2;
			if (r<=1.0) 
				break;
		}
		if (r>1.0) 
			r=1.0;

		double f = sqrt(-2.0 * log(r) / r);
		double current = amplitude_ * f * v1;
		saved_value_ = amplitude_ * f * v2;
		has_saved_value_ = true;

		return current;
	}

	/*
	*\brief Dither gets an amplitude, must be >0 to initialize
	*
	*\param amplitude Amplitude of white noise output
	*\param seed Random seed for white noise
	*/
	bool init(const double &amplitude, const double &seed){
		if (amplitude < 0.0){
		  cout << "Dither amplitude not set properly. Amplitude must be >0.";
		  return false;
		}

		amplitude_ = amplitude;

		// Somewhat mimics the old code.  A given seed gives a
		// reproducible sequence of random numbers, and that's what
		// matters.
		unsigned long s = (unsigned long)seed;
		seed_[0] = (s) ^ 45213;
		seed_[1] = (s >> 16) ^ 39204;
		seed_[2] = (s >> 32) ^ 5598;

		return true;
	}


private:
	double amplitude_;   /**< Amplitude of the sweep. */
	double saved_value_;
	bool has_saved_value_;
	double s_;
	double x_;
	unsigned short seed_[3];
};

#endif
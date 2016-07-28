#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

class SineSweep{
public:
	double K_;
	double L_;
	double amplitude_;
	double duration_;
	double start_angular_freq_;
	double end_angular_freq_;
	double cmd_;

	SineSweep(){
		K_ = 0.0;
		L_ = 0.0;
		amplitude_ = 0.0;
		duration_ = 0.0;
		cmd_ = 0.0;
		start_angular_freq_ = 0.0;
		end_angular_freq_ = 0.0;
	}

	~SineSweep(){}

	bool init(double start_freq, double end_freq, double duration, double amplitude){
		if (start_freq > end_freq) return false;
		if (duration < 0) return false;
		if (amplitude < 0) return false;

		amplitude_ = amplitude;
		duration_ = duration;

		start_angular_freq_ = 2*M_PI*start_freq;
		end_angular_freq_ = 2*M_PI*end_freq;

		K_ = (start_angular_freq_*duration)/log(end_angular_freq_/start_angular_freq_);
		L_ = (duration)/log(end_angular_freq_/start_angular_freq_);

		cmd_ = 0.0;

		return true;
	}

	double update(double dt){
		if (dt <= duration_){
			cmd_ = amplitude_*sin(K_*(exp((dt)/(L_))-1));
		}
		else{
			cmd_ = 0.0;
		}
		return cmd_;
	}
};

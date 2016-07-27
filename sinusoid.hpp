#ifndef CONTROL_TOOLBOX__SINUSOID_H_
#define CONTROL_TOOLBOX__SINUSOID_H_

#include <iostream>
#include <cmath>
//#include <tinyxml.h>

/**
 * \class Sinusoid
 * \brief A basic sine class
 *
 * This class calculates the output for a sine wave and its derivatives, given the amplitude,
 * phase, frequency and offset.<br>
 *
 */
using namespace std;

class Sinusoid{
public:
    /**
    * \brief Constructor
    */
    //Sinusoid()

    /**
    * \brief Constructor which intializes values
    *
    * \param offset A DC offset to be added to the sine wave
    * \param amplitude Amplitude of the sine wave
    * \param frequency Frequency of the sine wave
    * \param phase Phase (in radians) of the sine wave at t=0
    */
    Sinusoid(double offset, double amplitude, double frequency, double phase):
        offset_(offset),amplitude_(amplitude),frequency_(frequency),phase_(phase){
    }

    /**
    * Destructor
    */
    ~Sinusoid(){}

    /**
    * \brief Initializes the parameters of the sine wave from the given xml element
    *
    * \param ti_xml_element This XML element needs to contain the following attributes: offset, amplitude, frequency, phase
    * \return true if successful, false if not
    */
    // bool initXml(){
    //     return true;
    // }

    /**
    * Prints the parameters of the sine wave to stdout (for debugging)
    */
    void debug(){
        cout << "offset=" << offset_ << " amplitude=" << amplitude_ << " phase=" << phase_ << " frequency=" << frequency_ << endl;
    }

    /**
    * \brief Gets the value and derivatives of the sinusoid at a given time
    *
    * \param time Time at which to sample the sine wave
    * \param qd (output) The derivative of the sine wave
    * \param qdd (output) Second derivative of the sine wave
    * \return The sampled value of the sine wave
    */
    double update(double time, double& qd, double& qdd){
        double angular_frequency = 2.0*M_PI*frequency_;
        double p = phase_ + angular_frequency*time;
        double sin_p = sin(p);
        double cos_p = cos(p);
        double q = offset_ + amplitude_*sin_p;
        qd = angular_frequency*amplitude_*cos_p;
        qdd = -angular_frequency*angular_frequency*amplitude_*sin_p;
        return q;
    }

private:
    double offset_;               /**< DC offset of the sine wave. */
    double amplitude_;            /**< Amplitude of the sine wave. */
    double frequency_;            /**< Frequency of the sine wave. */
    double phase_;                /**< Phase of the sine wave at t=0. */
};

#endif
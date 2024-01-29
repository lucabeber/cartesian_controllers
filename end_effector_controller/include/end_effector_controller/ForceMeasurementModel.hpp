#ifndef KALMAN_VISCOELASTIC_ESTIMATION_FORCEMEASUREMENTMODEL_HPP_
#define KALMAN_VISCOELASTIC_ESTIMATION_FORCEMEASUREMENTMODEL_HPP_

#include <end_effector_controller/kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples2
{
namespace Estimation
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class VelocityMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(VelocityMeasurement, T, 1)
    
    //! Distance to landmark 1
    static constexpr size_t V = 0;
    
    T v()       const { return (*this)[ V ]; }
    
    T& v()      { return (*this)[ V ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class ForceMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VelocityMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExamples2::Estimation::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples2::Estimation::VelocityMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    ForceMeasurementModel()
    {
        this->V.setIdentity();
        this->V *= 0.000005;
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        measurement.v() = x.x2(); // pow(abs(x.x1()),1.35) * x.x3() + pow(abs(x.x1()),1.35) * x.x2() * x.x4();
        
        return measurement;
    }

protected:
    void updateJacobians( const S& x )
    {
    // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    this->H.setZero();
    
    // partial derivative of meas.d1() w.r.t. x.x()
    // this->H( M::V, S::POSITION ) = 0.0;
    // partial derivative of meas.d1() w.r.t. x.y()
    this->H( M::V, S::VELOCITY ) = 1.0;
    // partial derivative of meas.d1() w.r.t. x.y()
    // this->H( M::V, S::ELASTICITY ) = 0.0;
    // partial derivative of force w.r.t. el
    // this->H( M::V, S::VISCOSITY ) = 0.0;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif
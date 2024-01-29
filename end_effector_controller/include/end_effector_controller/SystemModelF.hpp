#ifndef KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_
#define KALMAN_VISCOELASTIC_ESTIMATION_SYSTEMMODEL_HPP_

#include <end_effector_controller/kalman/LinearizedSystemModel.hpp>

namespace KalmanExamples2
{
namespace Estimation
{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)
    
    //! X-position
    static constexpr size_t POSITION = 0;
    //! X-position
    static constexpr size_t VELOCITY = 1;
    //! Y-Position
    static constexpr size_t ELASTICITY = 2;
    //! Orientation
    static constexpr size_t VISCOSITY = 3;
    
    T x1()      const { return (*this)[ POSITION ]; }
    T x2()      const { return (*this)[ VELOCITY ]; }
    T x3()      const { return (*this)[ ELASTICITY ]; }
    T x4()      const { return (*this)[ VISCOSITY ]; }
    
    T& x1()     { return (*this)[ POSITION ]; }
    T& x2()     { return (*this)[ VELOCITY ]; }
    T& x3()     { return (*this)[ ELASTICITY ]; }
    T& x4()     { return (*this)[ VISCOSITY ]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples2::Estimation::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples2::Estimation::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C&  c) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // New x-position given by old x-position plus change in x-direction
        // Change in x-direction is given by the cosine of the (new) orientation
        // times the velocity
        x_.x1() = x.x1() + this->dT * x.x2();
        x_.x2() = x.x2() + this->dT / this->mass * ( this->force_z - pow(abs(x.x1()),this->n) * x.x3() - pow(abs(x.x1()),this->n) * x.x2() * x.x4() );
        x_.x3() = x.x3();
        x_.x4() = x.x4();
        
        // Return transitioned state vector
        return x_;
    }

    void setModelData( double mass, double timeStep, int n)
    {
        this->mass  =   mass;
        this->dT    =   timeStep;
        this->n     =   n;
    }

    void setForce( double force_z ) 
    {
        this->force_z = force_z;
    }

protected:
    double mass;
    double dT;    
    double force_z;
    double n;

    void updateJacobians( const S& x, const C& c)
    {   
        // partial derivative of x.theta() w.r.t. x.theta()
        this->F.setIdentity();
        this->F( S::POSITION, S::VELOCITY ) = this->dT;
        this->F( S::VELOCITY, S::POSITION ) = (this->dT * (-x.x4() * this->n * x.x2() * pow(abs(x.x1()), this->n - 1) - x.x3() * this->n * pow(abs(x.x1()), this->n - 1))) / this->mass;
        this->F( S::VELOCITY, S::VELOCITY ) = 1 - (x.x4() * this->dT * pow(abs(x.x1()), this->n)) / this->mass;
        this->F( S::VELOCITY, S::ELASTICITY ) = -this->dT * pow(abs(x.x1()), this->n) / this->mass;
        this->F( S::VELOCITY, S::VISCOSITY ) = -this->dT * x.x2() * pow(abs(x.x1()), this->n) / this->mass; 
        
        // set W to zero
        this->W.setZero();
        this->W( S::VELOCITY , S::VELOCITY ) = 1e-5;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif
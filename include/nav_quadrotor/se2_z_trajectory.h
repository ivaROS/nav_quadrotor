
#ifndef SE2_Z_TRAJECTORY_H
#define SE2_Z_TRAJECTORY_H

#include <traj_generator.h>
#include <nav_quadrotor/se2_z_state.h>
#include <Eigen/Eigen>


namespace nav_quadrotor
{
  namespace se2_z_trajectory 
  {
    
    
    
    struct ni_params
    {
      ni_params() {}
      ni_params( double c_p, double c_d, double c_lambda, double epsilon ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon) { }
      ni_params( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon), v_max_(v_max), w_max_(w_max), a_max_(a_max), w_dot_max_(w_dot_max) { }
      
      double c_p_;
      double c_d_;
      double c_lambda_;
      double epsilon_;
      
      double v_max_ = std::numeric_limits<double>::infinity();
      double w_max_ = std::numeric_limits<double>::infinity();
      double a_max_ = std::numeric_limits<double>::infinity();
      double w_dot_max_ = std::numeric_limits<double>::infinity();
      
      bool verify()
      {
        //TODO: Add ROS_ASSERTs for each of these to enable explicit errors about these at run time and not in Release builds
        return c_p_>=0 && c_d_>=0 && c_lambda_>=0 && epsilon_>0
        && v_max_>0 && w_max_>0 && a_max_>0 && w_dot_max_>0;
      }
    };

    class near_identity {

        ni_params params_;

    //     double c_p_;
    //     double c_d_;
    //     double c_lambda_;
    //     double epsilon_;
    //     
    //     double v_max_ = std::numeric_limits<double>::infinity();
    //     double w_max_ = std::numeric_limits<double>::infinity();
    //     double a_max_ = std::numeric_limits<double>::infinity();
    //     double w_dot_max_ = std::numeric_limits<double>::infinity();

    public:

        
        near_identity( double c_p, double c_d, double c_lambda, double epsilon ) : params_(c_p, c_d, c_lambda, epsilon) { }

        near_identity( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : params_(c_p, c_d, c_lambda, epsilon, v_max, w_max, a_max, w_dot_max) { }
        
        near_identity( ni_params params) : params_(params) {}

        void operator() ( const se2_z_state &state , se2_z_state &state_dot, const double t )
        {
            //load state variables
            double x = state[se2_z_state::X_IND];
            double y = state[se2_z_state::Y_IND];
            double z = state[se2_z_state::Z_IND];
            double theta = state[se2_z_state::THETA_IND];
            double v = state[se2_z_state::V_IND];
            double vz = state[se2_z_state::VZ_IND];
            double w = state[se2_z_state::W_IND];
            double lambda = state[se2_z_state::LAMBDA_IND];
            double x_d = state[se2_z_state::XD_IND];
            double y_d = state[se2_z_state::YD_IND];
            double z_d = state[se2_z_state::ZD_IND];
            double x_d_dot = state_dot[se2_z_state::XD_IND];
            double y_d_dot = state_dot[se2_z_state::YD_IND];
            double z_d_dot = state_dot[se2_z_state::ZD_IND];

            Eigen::Matrix2d R;
            R << cos(theta), -sin(theta), 
                sin(theta), cos(theta);
                
            double lambda_dot = -params_.c_lambda_*(lambda - params_.epsilon_);
            

            //Now find derivatives of state variables
            //x_dot = (R*e1)*v1;
            double x_dot = R(0,0)*v;
            double y_dot = R(1,0)*v;
            double z_dot = vz;
            double theta_dot = w;
            
            //Now find tau
            Eigen::Vector3d tau = getTau(x,y,z,theta,v,vz,w,lambda,x_d,y_d,z_d,x_d_dot,y_d_dot,z_d_dot,R,lambda_dot);

            double v_dot = limitV(tau, v);
            double w_dot = limitW(tau, w);
            double vz_dot = limitVZ(tau, vz);


            state_dot[se2_z_state::X_IND] = x_dot;
            state_dot[se2_z_state::Y_IND] = y_dot;
            state_dot[se2_z_state::Z_IND] = z_dot;
            state_dot[se2_z_state::THETA_IND] = theta_dot;
            state_dot[se2_z_state::V_IND] = v_dot;
            state_dot[se2_z_state::VZ_IND] = vz_dot;
            state_dot[se2_z_state::W_IND] = w_dot;
            state_dot[se2_z_state::LAMBDA_IND] = lambda_dot;
            //std::vector<double> state_dot = {x_dot, y_dot, theta_dot, v_dot, w_dot, lambda_dot};

        }
        
    private:
        
        inline
        __attribute__((always_inline))
        Eigen::Vector3d getTau( double x, double y, double z, double theta, double v, double vz, double w, double lambda, double x_d, double y_d, double z_d, double x_d_dot, double y_d_dot, double z_d_dot, const Eigen::Matrix2d& R, double lambda_dot )
        {
            Eigen::Vector2d xy;
            xy << x,
                  y;
                              
            Eigen::Vector2d vw;
            vw << v,
                  w;

            Eigen::Vector2d xy_d;
            xy_d << x_d, 
                    y_d;
                    
            Eigen::Vector2d xy_d_dot;
            xy_d_dot << x_d_dot,
                        y_d_dot;

            //R_lambda = [R*e1 lambda*R*e2];
            Eigen::Matrix2d R_lambda;
            R_lambda << R(0,0), lambda*R(0,1), 
                        R(1,0), lambda*R(1,1);

            //R_lambda_inv = [R*e1 R*e2/lambda]';
            Eigen::Matrix2d R_lambda_inv;
            R_lambda_inv << R(0,0),        R(1,0), 
                            R(0,1)/lambda, R(1,1)/lambda;

            Eigen::Matrix2d w_hat;
            w_hat << 0,             -lambda*w, 
                    (1.0/lambda)*w, lambda_dot/lambda;

            //q = xy + lambda*R*e1;
        /*    Eigen::Vector2d q(x + lambda*R(0,0), 
                              y + lambda*R(1,0)); */
            Eigen::Vector2d q = xy + lambda*R.col(0);     
                              
        //   std::cout << "q: " << q(0) << ", " << q(1) << std::endl;
            
            //p = R_lambda*v + lambda_dot*R_lambda*e1;
        /*   Eigen::Vector2d p(v + lambda_dot*R_lambda(0,0), 
                              w + lambda_dot*R_lambda(1,0));*/

            Eigen::Vector2d p = R_lambda*vw + lambda_dot*R_lambda.col(0);

        //   std::cout << "p: " << p(0) << ", " << p(1) << std::endl;
            
            
            //u can be any expression that will feedback stabilize a linear system
            //here, using 2nd order feedback controller   ; + x_d_dot_dot
            Eigen::Vector2d u = -params_.c_p_*(q - xy_d) - params_.c_d_*(p - xy_d_dot);

      //     std::cout << "u: " << u(0) << ", " << u(1) << std::endl;

            //Now find tau
            Eigen::Vector2d tau = R_lambda_inv*u - w_hat*vw - lambda_dot*(w_hat - params_.c_lambda_*Eigen::Matrix2d::Identity())*Eigen::Matrix<double, 2, 1>::Identity();
            
            double uz = -params_.c_p_*(z - z_d) - params_.c_d_*(vz - z_d_dot);
            
            Eigen::Vector3d tauz;
            tauz << tau(0), tau(1), uz;
            
            return tauz;
      
        }
        

        /* Generic saturation function for variable X. */
        inline
        static double saturate(double X, double minX, double maxX)
        {
            if(X >= maxX)
            {
                return maxX;
            }
            else if(X <= minX)
            {
                return minX;
            }
            else
            {
                return X;
            }
        }
        
        /* Generic saturation function for variable X_dot given limits on X_dot and X. */
        inline
        static double applyLimits(double X_dot, double X, double minX, double maxX, double minX_dot, double maxX_dot)
        {
            if(X >= maxX)
            {
                return saturate(X_dot, minX_dot, 0);
            }
            else if(X <= minX)
            {
                return saturate(X_dot, 0, maxX_dot);
            }
            else
            {
                return saturate(X_dot, minX_dot, maxX_dot);
            }   
        }
        
        inline
        double limitV(const Eigen::Vector3d& tau, double v)
        {
            return applyLimits(tau[0], v, -params_.v_max_, params_.v_max_, -params_.a_max_, params_.a_max_);
        }
        
        inline
        double limitVZ(const Eigen::Vector3d& tau, double vz)
        {
          return applyLimits(tau[2], vz, -params_.v_max_, params_.v_max_, -params_.a_max_, params_.a_max_);
        }
        
        inline
        double limitW(const Eigen::Vector3d& tau, double w)
        {
            return applyLimits(tau[1], w, -params_.w_max_, params_.w_max_, -params_.w_dot_max_, params_.w_dot_max_);
        }
    };
    
    
    //[ rhs_class
    /* The rhs of x' = f(x) defined as a class */
    class desired_traj_func {
      
    public:
      //TODO: was there ever a reason to have this?
      virtual void init ( const se2_z_state &x0 )
      {
        //std::cout << "This should only print if an init function is not defined" << std::endl;
      }
      
      //This approach only allows write access to the variables that should be altered
      virtual void dState ( const se2_z_state &x, se2_z_state::Desired &des_dot, const double  t  )=0;
      
      
      typedef std::shared_ptr<desired_traj_func> Ptr;
      
    };
    //]
    
    
    
    //[ rhs_class
    /* The rhs of x' = f(x) defined as a class */
    template <typename C, typename D>
    class se2_z_system : public trajectory_generator::traj_func<se2_z_system<C,D>, se2_z_state>
    {
      C controller_;
      D desired_traj_;
      
      
    public:
      se2_z_system(C controller, D desired_traj) :  
      controller_(controller),
      desired_traj_(desired_traj)
      { }
      
      void operator_impl ( const se2_z_state &x , se2_z_state &dxdt , const double  t  )
      {
        
        desired_traj_.dState(x, dxdt.desired, t);
        controller_(x, dxdt, t); //Currently, includes the dynamics
        //Dynamics(x.actual, actual_command, dxdt.actual);
        //Dynamics(x.desired, desired_command, dxdt.desired);
        //TODO: reincorporate the 'dynamics' into separate class and more explicitly limit what each part of the system can access
      }
      
      typedef std::shared_ptr<se2_z_system> Ptr;
      
    };
    
    using traj_func_type = trajectory_generator::TypedTrajectoryGeneratorObject<se2_z_state>;
    
    template <typename F>
    using TrajGenObjectInterface = trajectory_generator::FuncTypedTrajectoryGeneratorObject<se2_z_state, F>;
    using trajectory_generator::traj_params;
    using Params = ni_params;
    
    
    template <typename C, typename F>
    class SpecializedTrajGenObject : public TrajGenObjectInterface<se2_z_system<C,F> >
    {
    public:
      
      // Template N allows either a controller or parameters to be passed in
      template <typename N>
      SpecializedTrajGenObject(F desired_traj, N controller, state_type x0, traj_params tparams) : 
      TrajGenObjectInterface<se2_z_system<C,F> >(se2_z_system<C, F>(C(controller), desired_traj), x0, tparams)
      {}
      
    };
    
    
    
    template<typename C, typename D, typename N>
    SpecializedTrajGenObject<C, D> getTrajGenObject(D desired_traj, N controller, state_type x0=state_type(), traj_params tparams=traj_params())  
    {
      return SpecializedTrajGenObject<C, D>(desired_traj, controller, x0, tparams);
    }
    
    template<typename D>
    SpecializedTrajGenObject<near_identity,D> getTrajGenObject(D desired_traj, Params params, state_type x0=state_type(), traj_params tparams=traj_params())  
    {
      return getTrajGenObject<near_identity>(desired_traj, params, x0, tparams);
    }
    
    //typedef trajectory_generator::traj_generator<state_type, traj_func_type> TrajectoryGenerator;
    typedef desired_traj_func::Ptr des_traj_func_ptr;

  }
  
}
#endif //SE2_Z_TRAJECTORY_H

#ifndef NAV_QUADROTOR_COMMITTED_GLOBAL_GOAL_SCORING_H
#define NAV_QUADROTOR_COMMITTED_GLOBAL_GOAL_SCORING_H

#include <nav_quadrotor/global_goal_scoring.h>

#include <trajectory_based_nav/default_implementations.h>  //Only need BasicTrajectoryCosts
#include <string> 

namespace trajectory_based_nav
{
 
  class TrajectoryCostsContainer : public BasicTrajectoryCosts
  {
  protected:
    
    struct CostComponent
    {
      CostComponent(double cost, const std::string& name): cost(cost), name(name) {}
      
      operator double () {return cost;}

      double cost;
      std::string name;
    };
    
    std::vector<CostComponent> costs_;
    
  public:
    
    TrajectoryCostsContainer():
      BasicTrajectoryCosts()
      {}
      
    void addCost(double cost, std::string name="") 
    {
      costs_.emplace_back(cost, name); 
      BasicTrajectoryCosts::single_cost += cost;
    }
    
    virtual double cost() const override
    {
      //std::accumulate(costs.begin(), costs.end(), 0)
      return BasicTrajectoryCosts::cost();
    }
    
    friend std::ostream& operator<< (std::ostream&, const TrajectoryCostsContainer::CostComponent&);

    virtual std::ostream& print(std::ostream& stream) const override;
    
    friend std::ostream& operator<< (std::ostream&, const TrajectoryCostsContainer&);
    
    using Ptr=std::shared_ptr<TrajectoryCostsContainer>;
  };
  
//   inline
//   std::ostream& costToStream(std::ostream& stream, double cost)
//   {
//     if(cost < std::numeric_limits<double>::max())
//     {
//       stream << cost;
//     }
//     else
//     {
//       stream << "FATAL";
//     }
//     return stream;
//   }
  
  inline
  std::string costToString(double cost)
  {
    if(cost < std::numeric_limits<double>::max())
    {
      return std::to_string(cost);
    }
    else
    {
      return "FATAL";
    }
  }

  
  inline
  std::ostream& operator<< (std::ostream& stream, const TrajectoryCostsContainer::CostComponent& cost)
  {
    stream << cost.name << "=" << costToString(cost.cost);
   
    return stream;
  }
  
  inline
  std::ostream& TrajectoryCostsContainer::print(std::ostream& stream) const
  {
    //stream << BasicTrajectoryCosts::stream(stream);

    stream << "Total Cost=" << costToString(cost());
    
    stream << " [";
    for(const auto& c : costs_)
    {
      stream << c << ", ";
    }
    stream << "]";
    return stream;
  }
  
  inline
  std::ostream& operator<< (std::ostream& stream, const TrajectoryCostsContainer& costs)
  {
    return costs.print(stream);
  }
  
//   inline
//   std::ostream& operator<< (std::ostream& stream, const TrajectoryCostsContainer& costs)
//   {
//     auto& t = costs.print(stream);
//     return t;
//   }

//   template <class T, typename std::enable_if<std::is_base_of<TrajectoryCosts, T>::value>::type* = nullptr>
//   std::ostream& operator<< (std::ostream& stream, const T* costs)
//   {
//     return costs->print(stream);
//   }
// 
//   template <class T, typename std::enable_if<std::is_base_of<trajectory_based_nav::TrajectoryCosts, T>::value>::type* = nullptr>
//   std::ostream& operator<< (std::ostream& stream, const T& costs)
//   {
//     return costs.print(stream);
//   }
  
} //end namespace trajectory_based_nav



namespace nav_quadrotor
{
  double getDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
  
  class CommittedGlobalGoalTrajectoryScoring : public GlobalGoalTrajectoryScoring
  {
  protected:
    trajectory_based_nav::GlobalGoalState::Ptr ggs_;
    double current_traj_cost_factor_;
    
  public:
    CommittedGlobalGoalTrajectoryScoring(GlobalPlanner::Ptr global_planner, const std::vector<std::shared_ptr<pips_egocylindrical::FreeSpaceChecker> >& fscs, trajectory_based_nav::GlobalGoalState::Ptr ggs);

    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf2_utils::TransformManager tfm) override;
    
    virtual bool scoreTrajectory(trajectory_based_nav::TrajectoryWrapper::Ptr traj) override;
  
  };

} //end namespace nav_quadrotor

#endif //NAV_QUADROTOR_COMMITTED_GLOBAL_GOAL_SCORING_H

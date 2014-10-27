/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ryan Luna */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/config.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>
#include <vector>
#include <math.h>

#include "RGRRT.h"
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

const int CHOICES = 2;
const int PLANNERS = 3;
const double epsilon = 0.01;
const double radius = 0.1;
const double square = 0.125;
const double g = 9.81;

// Pendulum Limits
const double torque_limit = M_PI;
const double pen_velocity_limit = 2*M_PI;

// Car Limits
const double turning_limit = M_PI / 2;
const double acceleration_limit = 0.25;
const double car_velocity_limit = 1;

typedef std::pair<double, double> Point2D;
typedef std::vector<Point2D> Rect;

namespace ob = ompl::base;
namespace oc = ompl::control;

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

bool lineIntersection(Point2D ours0, Point2D ours1, Point2D theirs0, Point2D theirs1)
{
    double theirs_lowerX = std::min(theirs0.first, theirs1.first);
    double theirs_upperX = std::max(theirs0.first, theirs1.first);
    double ours_lowerX = std::min(ours0.first, ours1.first);
    double ours_upperX = std::max(ours0.first, ours1.first);

    // Check if Y range of the lines overlap
    double theirs_lowerY = std::min(theirs0.second, theirs1.second);
    double theirs_upperY = std::max(theirs0.second, theirs1.second);
    double ours_lowerY = std::min(ours0.second, ours1.second);
    double ours_upperY = std::max(ours0.second, ours1.second);

    bool y0_overlap = (ours_lowerY >= theirs_lowerY) && (ours_lowerY <= theirs_upperY);
    bool y1_overlap = (ours_upperY >= theirs_lowerY) && (ours_lowerY <= theirs_upperY);
    if(!(y0_overlap || y1_overlap))
        return false;

    double ours_m = (ours1.second - ours0.second) / (ours1.first - ours0.first);
    double ours_b = ours0.second - ours_m * ours0.first;

    double theirs_m = (theirs1.second - theirs0.second) / (theirs1.first - theirs0.first);
    double theirs_b = theirs0.second - theirs_m * theirs0.first;

    if(isinf(ours_m))
    {
        // Check if X range of the lines overlap
        bool x_overlap = (theirs_lowerX < ours0.first) && (theirs_upperX > ours0.first);
        if(!x_overlap)
            return false;

        double theirs_value = theirs_m * ours0.first + theirs_b;
        return (theirs_value >= ours_lowerY) && (theirs_value <= ours_upperY);
    }

    if(isinf(theirs_m))
    {
        // Check if X range of the lines overlap
        bool x_overlap = (ours_lowerX < theirs0.first) && (ours_upperX > theirs0.first);
        if(!x_overlap)
            return false; 

        double ours_value = ours_m * theirs0.first + ours_b;
        return (ours_value >= theirs_lowerY) && (ours_value <= theirs_upperY);
    }

    // Brute-Force
    for(double pos = ours0.first; pos < ours1.first; pos+=epsilon)
    {
        if(pos >= theirs_lowerX && pos <= theirs_upperX)
        {
            double ours_value = ours_m * pos + ours_b;
            double theirs_value = theirs_m * pos + theirs_b;
            double diff = ours_value - theirs_value;
            if(abs(diff) < epsilon)
                return true;
        }
    }
    return false;
}

// Projections for KPIECE1 planner
class PenProjection : public ob::ProjectionEvaluator
{
    public:
        PenProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space) {}

        virtual unsigned int getDimension(void) const
        {
            return 2;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(2);
            cellSizes_[0] = 0.1;
            cellSizes_[1] = 0.25;
        }

        // Trivial projection
        virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
        {
            const ompl::base::CompoundState* cstate;
            cstate = state->as<ompl::base::CompoundState>();
            const ob::SO2StateSpace::StateType *rot = cstate->as<ob::SO2StateSpace::StateType>(0);
            const ob::RealVectorStateSpace::StateType *vel = cstate->as<ob::RealVectorStateSpace::StateType>(1);
            projection(0) = rot->value;
        }
};

class CarProjection : public ob::ProjectionEvaluator
{
    public:
        CarProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space) {}

        virtual unsigned int getDimension(void) const
        {
            return 2;
        }
        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(2);
            cellSizes_[0] = 0.1;
            cellSizes_[1] = 0.25;
        }
        // Average positions, average angle and angular velocity
        virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
        {
            const ompl::base::CompoundState* cstate;
            cstate = state->as<ompl::base::CompoundState>();
            const ob::RealVectorStateSpace::StateType *pos = cstate->as<ob::RealVectorStateSpace::StateType>(0);
            const ob::SO2StateSpace::StateType *rot = cstate->as<ob::SO2StateSpace::StateType>(1);
            const ob::RealVectorStateSpace::StateType *vel = cstate->as<ob::RealVectorStateSpace::StateType>(2);
            projection(0) = pos->values[0];
            projection(1) = pos->values[1];
        }
};

// Definition of the ODE for the pendulum
void PendulumODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[0];
    const double omega = q[1];

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = omega;
    qdot[1] = -g*cos(theta) + u[0];
}

// Definition of the ODE for the kinematic car.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    const double v = q[3];

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = u[0];
    qdot[3] = u[1];
}

// This is a callback method invoked after numerical integration.
void PendulumPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between -pi and pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0));
}

// This is a callback method invoked after numerical integration.
void CarPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between -pi and pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0));
}

bool isStateValidPen(const ob::State *state)
{
    /// cast the abstract state type to the type we expect
    const ompl::base::CompoundState* cstate;
    cstate = state->as<ompl::base::CompoundState>();

    /// extract the first component of the state and cast it to what we expect - angle orientation
    const ob::SO2StateSpace::StateType *rot = cstate->as<ob::SO2StateSpace::StateType>(0);

    /// extract the second component of the state and cast it to what we expect - angular velocity
    const ob::RealVectorStateSpace::StateType *vel = cstate->as<ob::RealVectorStateSpace::StateType>(1);

    if(vel->values[0] > pen_velocity_limit)
        return false;
    else if(vel->values[0] < -pen_velocity_limit)
        return false;
    else
        return true;
}

bool isStateValidCar(const ob::State *state, const double minBound, const double maxBound, const std::vector<Rect> obstacles)
{
    const ompl::base::CompoundState* cstate;
    cstate = state->as<ompl::base::CompoundState>();
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const ompl::base::SO2StateSpace::StateType* so2state;
    so2state = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    const ompl::base::RealVectorStateSpace::StateType* rstate;
    rstate = cstate->as<ompl::base::RealVectorStateSpace::StateType>(2);

    double x = r2state->values[0];
    double y = r2state->values[1];
    double theta = so2state->value;    
    double v = rstate->values[0];

    // Check car velocity bounds
    if(v > car_velocity_limit)
        return false;
    else if(v < -car_velocity_limit)
        return false;

    // Initial Square Robot Points
    std::vector<Point2D> pts;
    pts.push_back(std::make_pair(-square, -square));
    pts.push_back(std::make_pair(square, -square));
    pts.push_back(std::make_pair(square, square));
    pts.push_back(std::make_pair(-square, square));

    // Transform Square Robot Points to state position
    for(int i = 0; i < pts.size(); ++i)
    {
        double newX = pts[i].first * cos(theta) - pts[i].second * sin(theta) + x;
        double newY = pts[i].first * sin(theta) + pts[i].second * cos(theta) + y;

        if(newX < minBound || newY < minBound || newX > maxBound || newY > maxBound)
        {
            return false;
        }

        pts[i] = std::make_pair(newX, newY);
    }

    for(const Rect& r : obstacles)
    {
        // None of the points of square robot are contained inside of the obstacle 
        /*
           for(int j = 0; j < pts.size(); ++j)
           {
           if(pts[j].first >= r[0].first && pts[j].first <= r[2].first && pts[j].second >= r[0].second && pts[j].second <= r[2].second)
           return false;
           }
         */

        // Edge of rectangle r
        for(int i = 0; i < r.size(); ++i)
        {
            // Edge of square robot
            for(int j = 0; j < pts.size(); ++j)
            {
                bool intersection = lineIntersection(pts[j], pts[(j+1) % pts.size()], r[i], r[(i+1) % r.size()]);
                if(intersection)
                {
                    return false;
                }
            }
        }
    }

    return true;

}

/// @cond IGNORE
// torque
class PenControlSpace : public oc::RealVectorControlSpace
{
    public:
        PenControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 1) {}
};
// turning angle, acceleration
class CarControlSpace : public oc::RealVectorControlSpace
{
    public:
        CarControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2) {}
};
/// @endcond

void planWithSimpleSetupPen(int plannertype, std::vector<Rect> obstacles, std::string title = "Default", bool benchmark = false)
{
    // theta, angular veloctiy
    ompl::base::StateSpacePtr space;
    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
    ompl::base::StateSpacePtr r(new ompl::base::RealVectorStateSpace(1));

    ompl::base::RealVectorBounds velocity_limit(1);
    velocity_limit.setLow(-pen_velocity_limit);
    velocity_limit.setHigh(pen_velocity_limit);
    r->as<ompl::base::RealVectorStateSpace>()->setBounds(velocity_limit);

    space = so2 + r;

    // create a control space
    oc::ControlSpacePtr cspace(new PenControlSpace(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(0);
    cbounds.setHigh(torque_limit);
    cspace->as<PenControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValidPen);

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    /// create a start state
    ob::ScopedState<> start(space);
    start[0] = -M_PI/2;
    start[1] = 0.0;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<> goal(space);
    goal[0] = M_PI/2;
    goal[1] = 0.0;

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.15);

    if(benchmark)
    {
        // Benchmark Code - Project 4
        ompl::tools::Benchmark b(ss, title);
        ss.getSpaceInformation()->setPropagationStepSize(0.01);
        b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss.getSpaceInformation())));
        b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss.getSpaceInformation())));

        ss.getSpaceInformation()->setPropagationStepSize(0.05);
        ompl::base::PlannerPtr kp1(new ompl::control::KPIECE1(ss.getSpaceInformation()));
        space->registerProjection("PenProjection", ob::ProjectionEvaluatorPtr(new PenProjection(space)));
        kp1->as<ompl::control::KPIECE1>()->setProjectionEvaluator("PenProjection");
        b.addPlanner(kp1);

        ompl::tools::Benchmark::Request req;
        req.maxTime = 30.0;
        req.maxMem = 1000.0;
        req.runCount = 20;
        req.displayProgress = true;
        b.benchmark(req);
        std::string logfile = title + ".log";
        b.saveResultsToFile(logfile.c_str());
    }
    else
    {
        if (plannertype == 0) 
        {
            // RRT
            ss.getSpaceInformation()->setPropagationStepSize(0.01);
            ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
            ss.setPlanner(planner);
        } 
        else if (plannertype == 1) 
        {
            // KPIECE1
            ss.getSpaceInformation()->setPropagationStepSize(0.05);
            ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));
            space->registerProjection("PenProjection", ob::ProjectionEvaluatorPtr(new PenProjection(space)));
            planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("PenProjection");
            ss.setPlanner(planner);

        } else if (plannertype == 2) {
            // RG-RRT
            ss.getSpaceInformation()->setPropagationStepSize(0.01);
            ompl::base::PlannerPtr planner(new ompl::control::RGRRT(ss.getSpaceInformation()));
            ss.setPlanner(planner);
        }

        ss.setup();

        /// attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(20.0);

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            ompl::control::PathControl& path = ss.getSolutionPath();
            path.printAsMatrix(std::cout);

            // print path to file
            std::ofstream fout("path.txt");
            fout << "Pen" << std::endl;
            path.printAsMatrix(fout);
            fout.close();
        }
        else
            std::cout << "No solution found" << std::endl;
    }
}

void planWithSimpleSetupCar(int plannertype, std::vector<Rect> obstacles, std::string title = "Default", bool benchmark = false)
{
    // x, y, theta, v
    ompl::base::StateSpacePtr space;

    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

    ompl::base::StateSpacePtr r(new ompl::base::RealVectorStateSpace(1));
    ompl::base::RealVectorBounds velocity_limit(1);
    velocity_limit.setLow(-car_velocity_limit);
    velocity_limit.setHigh(car_velocity_limit);
    r->as<ompl::base::RealVectorStateSpace>()->setBounds(velocity_limit);

    space = r2 + so2 + r;

    // Create a control space
    oc::ControlSpacePtr cspace(new CarControlSpace(space));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -turning_limit);
    cbounds.setHigh(0, turning_limit);
    cbounds.setLow(1, -acceleration_limit);
    cbounds.setHigh(1, acceleration_limit);
    cspace->as<CarControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(isStateValidCar, _1, -1, 1, obstacles));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, CarPostIntegration));
    ss.getSpaceInformation()->setPropagationStepSize(0.05);

    /// create a start state
    ob::ScopedState<> start(space);
    start[0] = -0.75;
    start[1] = -0.75;
    start[2] = 0.0;
    start[3] = 0.0;

    /// create a  goal state; use the hard way to set the elements
    ob::ScopedState<> goal(space);
    goal[0] = 0.75;
    goal[1] = 0.75;
    goal[2] = 0.0;
    goal[3] = 0.0;

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.15);

    if(benchmark)
    {
        // Benchmark Code - Project 4
        ompl::tools::Benchmark b(ss, title);
        b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss.getSpaceInformation())));
        b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss.getSpaceInformation())));

        ompl::base::PlannerPtr kp1(new ompl::control::KPIECE1(ss.getSpaceInformation()));
        space->registerProjection("PenProjection", ob::ProjectionEvaluatorPtr(new PenProjection(space)));
        kp1->as<ompl::control::KPIECE1>()->setProjectionEvaluator("PenProjection");
        b.addPlanner(kp1);

        ompl::tools::Benchmark::Request req;
        req.maxTime = 30.0;
        req.maxMem = 1000.0;
        req.runCount = 20;
        req.displayProgress = true;
        b.benchmark(req);
        std::string logfile = title + ".log";
        b.saveResultsToFile(logfile.c_str());
    }
    else
    {
        if (plannertype == 0) 
        {
            // RRT
            ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
            ss.setPlanner(planner);
        } 
        else if (plannertype == 1) 
        {
            // KPIECE1
            ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));
            space->registerProjection("CarProjection", ob::ProjectionEvaluatorPtr(new CarProjection(space)));
            planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("CarProjection");
            ss.setPlanner(planner);
        } 
        else if (plannertype == 2) 
        {
            // RG-RRT
            ompl::base::PlannerPtr planner(new ompl::control::RGRRT(ss.getSpaceInformation()));
            ss.setPlanner(planner);
        }

        ss.setup();

        /// attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(20.0);

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            ompl::control::PathControl& path = ss.getSolutionPath();
            path.printAsMatrix(std::cout);

            // print path to file
            std::ofstream fout("path.txt");
            fout << "Car" << std::endl;
            path.printAsMatrix(fout);
            fout.close();
        }
        else
            std::cout << "No solution found" << std::endl;
    }
}

int main(int, char **)
{

    std::vector<std::vector<Rect>> obstacles;
    std::vector<Rect> narrow_obstacles;
    std::vector<Point2D> rect3;
    std::vector<Point2D> rect4;

    // Narrow Passage in-between two obstacles
    rect3.push_back(std::make_pair(-1.0, -0.5));
    rect3.push_back(std::make_pair(-0.25, -0.5));
    rect3.push_back(std::make_pair(-0.25, 0.5));
    rect3.push_back(std::make_pair(-1.0, 0.5));

    rect4.push_back(std::make_pair(0.25, -0.5));
    rect4.push_back(std::make_pair(1.0, -0.5));
    rect4.push_back(std::make_pair(1.0, 0.5));
    rect4.push_back(std::make_pair(0.25, 0.5));

    narrow_obstacles.push_back(rect3);
    narrow_obstacles.push_back(rect4);
    obstacles.push_back(narrow_obstacles);

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    int choice;
    do
    {
        std::cout << "Plan for: "<< std::endl;
        std::cout << " (0) A Torque-controlled Pendulum System" << std::endl;
        std::cout << " (1) A Car-like System" << std::endl;
        std::cout << " (2) Benchmark RG-RRT Planner" << std::endl;

        std::cin >> choice;
    } while (choice < 0 || choice > CHOICES);

    if(choice == CHOICES)
    {
        planWithSimpleSetupPen(0, obstacles[0], "RRT", true);
        planWithSimpleSetupPen(1, obstacles[0], "KPIECE", true);
        planWithSimpleSetupPen(2, obstacles[0], "RG-RRT", true);
        planWithSimpleSetupCar(0, obstacles[0], "RRT", true);
        planWithSimpleSetupCar(1, obstacles[0], "KPIECE", true);
        planWithSimpleSetupCar(2, obstacles[0], "RG-RRT", true);
        return 0;
    }

    int plannertype;
    do
    {
        std::cout << "Pick Planner: "<< std::endl;
        std::cout << " (0) RRT" << std::endl;
        std::cout << " (1) KPIECE" << std::endl;
        std::cout << " (2) RG-RRT" << std::endl;

        std::cin >> plannertype;
    } while (plannertype < 0 || plannertype > PLANNERS);

    switch(choice)
    {
        case 0:
            planWithSimpleSetupPen(plannertype,obstacles[0], "UserPlanner");
            break;
        case 1:
            planWithSimpleSetupCar(plannertype,obstacles[0], "UserPlanner");
            break;
    }
    return 0;
}

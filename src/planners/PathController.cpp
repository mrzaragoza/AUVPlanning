#include "planners/PathController.h"
#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <numeric>
#include <cmath>

namespace
{
    unsigned int getNumberOfDiscreteControls(const ompl::control::ControlSpace *cs)
    {
        if (cs->isCompound())
        {
            const ompl::control::CompoundControlSpace *ccs
                = cs->as<ompl::control::CompoundControlSpace>();
            unsigned int num = 0;
            for (unsigned int i = 0; i < ccs->getSubspaceCount(); ++i)
                num += getNumberOfDiscreteControls(ccs->getSubspace(i).get());

            return num;
        }
        else
            if (dynamic_cast<const ompl::control::DiscreteControlSpace*>(cs))
                return 1;
        return 0;
    }

    void printDiscreteControls(std::ostream &out, const ompl::control::ControlSpace *cs,
        const ompl::control::Control *c)
    {
        if (cs->isCompound())
        {
            const ompl::control::CompoundControlSpace *ccs
                = cs->as<ompl::control::CompoundControlSpace>();
            for (unsigned int i = 0; i < ccs->getSubspaceCount(); ++i)
                printDiscreteControls(out, ccs->getSubspace(i).get(),
                    c->as<ompl::control::CompoundControl>()->components[i]);
        }
        else if (dynamic_cast<const ompl::control::DiscreteControlSpace*>(cs))
            out << c->as<ompl::control::DiscreteControlSpace::ControlType>()->value << ' ';
    }
}

ompl::auvplanning::PathController::PathController(const base::SpaceInformationPtr &si) : base::Path(si)
{
    if (!dynamic_cast<const control::SpaceInformation*>(si_.get()))
        throw Exception("Cannot create a path with controls from a space that does not support controls");
}

ompl::auvplanning::PathController::PathController(const PathController &path) : base::Path(path.si_)
{
    copyFrom(path);
}

ompl::geometric::PathGeometric ompl::auvplanning::PathController::asGeometric() const
{
    PathController pc(*this);
    pc.interpolate();
    geometric::PathGeometric pg(si_);
    pg.getStates().swap(pc.currentStates_);
    return pg;
}

ompl::auvplanning::PathController& ompl::auvplanning::PathController::operator=(const PathController &other)
{
    freeMemory();
    si_ = other.si_;
    copyFrom(other);
    return *this;
}

void ompl::auvplanning::PathController::copyFrom(const PathController &other)
{
    currentStates_.resize(other.currentStates_.size());
    referenceStates_.resize(other.referenceStates_.size());

    for (unsigned int i = 0 ; i < currentStates_.size() ; ++i)
        currentStates_[i] = si_->cloneState(other.currentStates_[i]);

    for (unsigned int i = 0 ; i < referenceStates_.size() ; ++i)
        referenceStates_[i] = si_->cloneState(other.referenceStates_[i]);

    controlDurations_ = other.controlDurations_;
}

ompl::base::Cost ompl::auvplanning::PathController::cost(const base::OptimizationObjectivePtr &opt) const
{
    OMPL_ERROR("Error: Cost computation is only implemented for paths of type PathGeometric.");
    return opt->identityCost();
}

double ompl::auvplanning::PathController::length() const
{
    return std::accumulate(controlDurations_.begin(), controlDurations_.end(), 0.0);
}

void ompl::auvplanning::PathController::print(std::ostream &out) const
{
    const control::SpaceInformation *si = static_cast<const control::SpaceInformation*>(si_.get());
    double res = si->getPropagationStepSize();
    out << "Controller path with " << currentStates_.size() << " states" << std::endl;
    for (unsigned int i = 0 ; i < referenceStates_.size() ; ++i)
    {
        out << "At state ";
        si_->printState(currentStates_[i], out);
        out << "  apply controller PID with the reference ";
        si->printState(referenceStates_[i], out);
        out << "  for " << (int)floor(0.5 + controlDurations_[i]/res) << " steps" << std::endl;
    }
    out << "Arrive at state ";
    si_->printState(currentStates_[referenceStates_.size()], out);
    out << std::endl;
}

void ompl::auvplanning::PathController::printAsMatrix(std::ostream &out) const
{
    if (currentStates_.empty())
        return;
    const base::StateSpace* space(si_->getStateSpace().get());
    std::vector<double> reals, reals_ref;

    space->copyToReals(reals, currentStates_[0]);
    std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(out, " "));
    if (referenceStates_.empty())
        return;

    for (unsigned int i = 0 ; i < space->getDimension(); ++i)
        out << "0 ";
    out << '0' << std::endl;

    for (unsigned int i = 0 ; i < referenceStates_.size(); ++i)
    {
        space->copyToReals(reals, currentStates_[i + 1]);
        std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(out, " "));

        space->copyToReals(reals_ref, referenceStates_[i]);
        std::copy(reals_ref.begin(), reals_ref.end(), std::ostream_iterator<double>(out, " "));
        out << controlDurations_[i] << std::endl;
    }
}

void ompl::auvplanning::PathController::interpolate()
{
    if (currentStates_.size() <= referenceStates_.size())
    {
        OMPL_ERROR("Interpolation not performed.  Number of states in the path should be strictly greater than the number of references.");
        return;
    }

    /*
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    std::vector<base::State*> newStates;
    std::vector<Control*> newControls;
    std::vector<double> newControlDurations;

    double res = si->getPropagationStepSize();
    for (unsigned int  i = 0 ; i < controls_.size() ; ++i)
    {
        int steps = (int)floor(0.5 + controlDurations_[i] / res);
        assert(steps >= 0);
        if (steps <= 1)
        {
            newStates.push_back(states_[i]);
            newControls.push_back(controls_[i]);
            newControlDurations.push_back(controlDurations_[i]);
            continue;
        }
        std::vector<base::State*> istates;
        si->propagate(states_[i], controls_[i], steps, istates, true);
        // last state is already in the non-interpolated path
        if (!istates.empty())
        {
            si_->freeState(istates.back());
            istates.pop_back();
        }
        newStates.push_back(states_[i]);
        newStates.insert(newStates.end(), istates.begin(), istates.end());
        newControls.push_back(controls_[i]);
        newControlDurations.push_back(res);
        for (int j = 1 ; j < steps; ++j)
        {
            newControls.push_back(si->cloneControl(controls_[i]));
            newControlDurations.push_back(res);
        }
    }
    newStates.push_back(states_[controls_.size()]);
    states_.swap(newStates);
    controls_.swap(newControls);
    controlDurations_.swap(newControlDurations);*/
}

bool ompl::auvplanning::PathController::check() const
{
    if (referenceStates_.empty())
    {
        if (currentStates_.size() == 1)
            return si_->isValid(currentStates_[0]);
        else
            return false;
    }

    bool valid = true;
    /*const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    double res = si->getPropagationStepSize();
    base::State *next = si_->allocState();

    control::DirectedControlSamplerPtr controlSampler_ = siC_->allocDirectedControlSampler();

    for (unsigned int  i = 0 ; valid && i < controls_.size() ; ++i)
    {
        unsigned int steps = (unsigned int)floor(0.5 + controlDurations_[i] / res);
        if (!si->isValid(states_[i]) || si->propagateWhileValid(states_[i], controls_[i], steps, next) != steps ||
            si->distance(next, states_[i + 1]) > std::numeric_limits<float>::epsilon())
            valid = false;
    }
    si_->freeState(next);
*/
    return valid;
}

void ompl::auvplanning::PathController::append(const base::State *state)
{
    currentStates_.push_back(si_->cloneState(state));
}

void ompl::auvplanning::PathController::append(const base::State *state, const base::State *reference, double duration)
{
    const control::SpaceInformation *si = static_cast<const control::SpaceInformation*>(si_.get());
    currentStates_.push_back(si->cloneState(state));
    referenceStates_.push_back(si->cloneState(reference));
    controlDurations_.push_back(duration);
}

void ompl::auvplanning::PathController::random()
{
    /*freeMemory();
    states_.resize(2);
    controlDurations_.resize(1);
    controls_.resize(1);

    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    states_[0] = si->allocState();
    states_[1] = si->allocState();
    controls_[0] = si->allocControl();

    base::StateSamplerPtr ss = si->allocStateSampler();
    ss->sampleUniform(states_[0]);
    ControlSamplerPtr cs = si->allocControlSampler();
    cs->sample(controls_[0], states_[0]);
    unsigned int steps = cs->sampleStepCount(si->getMinControlDuration(), si->getMaxControlDuration());
    controlDurations_[0] = steps * si->getPropagationStepSize();
    si->propagate(states_[0], controls_[0], steps, states_[1]);*/
}

bool ompl::auvplanning::PathController::randomValid(unsigned int attempts)
{
    /*freeMemory();
    states_.resize(2);
    controlDurations_.resize(1);
    controls_.resize(1);

    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    states_[0] = si->allocState();
    states_[1] = si->allocState();
    controls_[0] = si->allocControl();

    ControlSamplerPtr cs = si->allocControlSampler();
    base::UniformValidStateSampler *uvss = new base::UniformValidStateSampler(si);
    uvss->setNrAttempts(attempts);
    bool ok = false;
    for (unsigned int i = 0 ; i < attempts ; ++i)
        if (uvss->sample(states_[0]))
        {
            cs->sample(controls_[0], states_[0]);
            unsigned int steps = cs->sampleStepCount(si->getMinControlDuration(), si->getMaxControlDuration());
            controlDurations_[0] = steps * si->getPropagationStepSize();
            if (si->propagateWhileValid(states_[0], controls_[0], steps, states_[1]) == steps)
            {
                ok = true;
                break;
            }
        }
    delete uvss;

    if (!ok)
    {
        freeMemory();
        states_.clear();
        controls_.clear();
        controlDurations_.clear();
    }
    return ok;*/ return false;
}

void ompl::auvplanning::PathController::freeMemory()
{
    for (unsigned int i = 0 ; i < currentStates_.size() ; ++i)
        si_->freeState(currentStates_[i]);
    for (unsigned int i = 0 ; i < referenceStates_.size() ; ++i)
        si_->freeState(referenceStates_[i]);
}

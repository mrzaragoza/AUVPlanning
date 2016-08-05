#ifndef PLANIFICADOR_CONTROL_H_
#define PLANIFICADOR_CONTROL_H_

#include <ompl/base/Planner.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
namespace ompl
{
    class PlanificadorControl : public base::Planner
    {
    public:
        PlanificadorControl(const base::SpaceInformationPtr &si);
        virtual ~PlanificadorControl(void);
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
        virtual void clear(void);
        virtual void setup(void);
        virtual void getPlannerData(base::PlannerData &data) const;
    };
}

#endif /* PLANIFICADOR_CONTROL_H_ */
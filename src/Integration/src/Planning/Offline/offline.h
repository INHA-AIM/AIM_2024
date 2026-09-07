#ifndef OFFLINE_H
#define OFFLINE_H

#include "Global/global.h"
#include <Eigen/Dense>


void CalcSplineCourse(PLANNING_DATA_t *pst_PlanningData, float32_t f32_DS, int32_t s32_Line);

void LoadPath(PLANNING_DATA_t *pst_PlanningData);

void CalcPathYaw(PLANNING_DATA_t *pst_PlanningData);


#endif

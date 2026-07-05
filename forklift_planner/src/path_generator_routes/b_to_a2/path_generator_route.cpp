#include "forklift_planner/path_generator.h"

RoughPath PathGenerator::generateRouteBToA2(const Slot& src, const Slot& tgt,
                                            PathGenerationInfo* info) const {
    return generateRouteCommon(src, tgt, info);
}

#include "wrapper.h"
#include "../inverse_kinematics.h"

namespace External {
slider_positions inverse_kinematics(External::approach_definition def,
                                    External::NewTransform tf,
                                    External::Robot &robot) {
  return ::inverse_kinematics(def, tf, robot);
}
} // namespace External

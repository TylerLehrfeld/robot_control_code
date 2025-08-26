#pragma once


namespace External {
struct approach_definition;
class NewTransform;
class Robot;
struct slider_positions;


slider_positions inverse_kinematics(approach_definition def, NewTransform tf,
                                    Robot &robot);
} // namespace External

#include "Matrix.h"
#include "Transform.h"
/**
 * @brief pivot class with p_tip and p_post 
 * 
 */
class Pivot {
    public:
    Pivot(vector<Transform> FrameTransformationList);
    Matrix p_t;
    Matrix p_post;
};
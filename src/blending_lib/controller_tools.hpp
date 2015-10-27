#ifndef IBL_CONTROLLER_TOOLS_HPP__
#define IBL_CONTROLLER_TOOLS_HPP__

// =============================================================================
namespace IBL {
// =============================================================================

float sigpos(float x, float slope1);

float signeg(float x, float slope0);

float dsig(float x, float slope);

}
// END IBL =====================================================================

#endif // IBL_CONTROLLER_TOOLS_HPP__

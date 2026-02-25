#include "models/FlatEarthGravity.hpp"

namespace sim::models
{
    FlatEarthGravity::FlatEarthGravity(double g)
        : g_(g) {}
    
    sim::math::Vec3d FlatEarthGravity::compute(
        const sim::math::Vec3d& /*position_ned*/) const
    {
        // NED Frame: +Z axis down, gravity acts downwards
        return {0.0, 0.0, g_};
    }

} // namespace sim::models
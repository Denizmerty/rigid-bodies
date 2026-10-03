#pragma once

#include <rigidbodies/physics/collider.hpp>

#include <vector>

namespace rigidbodies::physics
{

    // Integrates material mass, centre, and polar inertia in the body frame. Colliders are slabs
    // centred on z = 0 with their stated depths: an overlapping 3D volume is counted once, using
    // the first collider's density. A later, deeper part still contributes outside earlier slabs;
    // hollow regions contain no material and can be filled by later parts. Zero-density/depth
    // parts and segments contain no material and do not displace another part.
    //
    // Uses a planar boundary arrangement with analytic straight-edge and circular-arc moments;
    // circles are never tessellated. Predicates use 64 double epsilons after normalizing the full
    // assembly extent to one, so boundaries indistinguishable at that scale are treated as
    // coincident. As with all floating-point geometry, details below this tolerance are not
    // resolved. No grid, adaptive quadrature, stochastic sampling, or external package is used.
    //
    // Collision geometry remains the union of the original convex envelopes, even for shells.
    // A single solid custom Shape uses its virtual mass properties, transformed into the body
    // frame. Custom geometry in a multi-collider body, or with a nonzero hollow shell, is rejected:
    // the support interface does not expose the exact boundaries required for material subtraction.
    [[nodiscard]] MassProperties compute_compound_mass_properties(const std::vector<Collider>& colliders);

} // namespace rigidbodies::physics

/********************************************************************************
 * Copyright (c) 2025 Contributors
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <cmath>
#include <optional>
#include <algorithm>
#include <vector>
#include <map>

#include "adore_math/point.h"

namespace adore
{
namespace math
{

struct SlPoint
{
  double s = 0.0;
  double l = 0.0;
};

// Helper for dot product
template<typename T>
inline T dot2( T ax, T ay, T bx, T by )
{
  return ax * bx + ay * by;
}

// Clamp helper
template<typename T>
inline T clamp01( T u )
{
  return std::max( static_cast<T>(0.0), std::min( static_cast<T>(1.0), u ) );
}

/**
 * @brief Projects point P onto line segment AB.
 * @return pair {s_offset_from_A, lateral_distance_signed}
 *         s_offset is along vector AB.
 *         l is signed: positive is left of vector AB (assuming right-handed 2D: x forward, y left)
 */
template<typename PointT1, typename PointT2, typename PointT3>
std::pair<double, double>
project_point_to_segment( const PointT1& A, const PointT2& B, const PointT3& P )
{
    const double abx = B.x - A.x;
    const double aby = B.y - A.y;
    const double apx = P.x - A.x;
    const double apy = P.y - A.y;

    const double tab = dot2( abx, aby, abx, aby );
    if( tab < 1e-9 )
    {
        // Segment is a point
        return { 0.0, std::hypot( apx, apy ) };
    }

    const double u = clamp01( dot2( apx, apy, abx, aby ) / tab );
    
    // Projected point on line
    const double px = A.x + u * abx;
    const double py = A.y + u * aby;
    
    // Lateral vector
    const double dx = P.x - px;
    const double dy = P.y - py;
    
    // Cross product 2D (AB x AP) determines sign. 
    // AB = (abx, aby), AP = (apx, apy). Cross z = abx*apy - aby*apx
    // If z > 0, P is to the left of AB.
    const double cross_z = abx * apy - aby * apx;
    
    // Lateral distance. Sign matches cross product.
    // Length of P-p is hypot(dx, dy).
    // If cross_z > 0, l is positive (Left).
    // If cross_z < 0, l is negative (Right). 
    
    double l = std::hypot( dx, dy );
    if (cross_z < 0) l = -l;
    
    // Longitudinal offset from A
    double s = u * std::sqrt(tab);
    
    return { s, l };
}

/**
 * @brief Find the closest point on a path (sequence of points with 's', 'x', 'y') to a query point P.
 *        The path is assumed to be sorted by 's'.
 */
template<typename PointT, typename PathContainerT>
std::optional<SlPoint>
project_to_path( const PathContainerT& path, const PointT& p, std::optional<double> s_seed = std::nullopt, double window = 100.0 )
{
    if (path.empty()) return std::nullopt;
    if (path.size() == 1) {
        // Degenerate path
        double dx = p.x - path.begin()->second.x;
        double dy = p.y - path.begin()->second.y;
        return SlPoint{ path.begin()->second.s, std::hypot(dx, dy) }; 
        // Note: s is just the point's s. l is distance.
    }

    // Iterator windowing logic can be complex for generic containers.
    // For std::map<double, Point>, we can utilize s_seed.
    
    auto begin_it = path.begin();
    auto end_it = path.end();

    if (s_seed.has_value()) {
        double s_center = *s_seed;
        // Find lower bound for window
        auto lb = path.lower_bound(s_center - window);
        if (lb != path.end()) begin_it = lb;
        
        // Find upper bound (roughly)
        auto ub = path.upper_bound(s_center + window);
        end_it = ub;
    }

    // Find closest segment
    double min_dist_sq = std::numeric_limits<double>::infinity();
    SlPoint best_sl;
    bool found = false;

    // Iterate segments
    auto it = begin_it;
    auto next = std::next(it);
    
    // Safety check just in case begin_it was end
    if (it == path.end()) return std::nullopt;
    
    while( next != end_it && next != path.end() )
    {
        const auto& ptA = it->second;
        const auto& ptB = next->second;

        auto [ds, l] = project_point_to_segment(ptA, ptB, p);
        
        // Check if this projection is valid/closer
        // Actually, we want the global closest point. The segment projection gives us a local candidate.
        // We compare 3D distances (x,y) to find best.
        
        // Reconstruct point on line to get distance
        // Or trust |l| IF the point projects WITHIN the segment.
        // If it clamped to endpoints, distance calculation is implicitly handled by segment camping in the helper?
        // Wait, helper clamps u. So (px, py) is strictly on segment. 
        // Distance is hypot(P - proj).
        double d_sq = l*l; // rough approximation? No, l is exact distance to segment.
        
        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            best_sl.s = ptA.s + ds;
            best_sl.l = l;
            found = true;
        }

        it = next;
        next++;
    }

    if (!found) return std::nullopt;
    return best_sl;
}

} // namespace math
} // namespace adore

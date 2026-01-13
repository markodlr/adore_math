/********************************************************************************
 * Copyright (c) 2025 Contributors
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

namespace adore
{
namespace math
{

template<typename T>
struct Interval
{
  T lo = std::numeric_limits<T>::infinity();
  T hi = -std::numeric_limits<T>::infinity();

  Interval() = default;
  Interval( T l, T h ) : lo( l ), hi( h ) {}

  bool is_empty() const { return lo > hi; }
  
  T length() const { return is_empty() ? static_cast<T>(0) : (hi - lo); }

  bool contains( T val ) const { return val >= lo && val <= hi; }
  
  bool overlaps( const Interval<T>& other ) const
  {
    return !( lo > other.hi || hi < other.lo );
  }

  void expand( T val )
  {
    if (val < lo) lo = val;
    if (val > hi) hi = val;
  }
  
  void expand( const Interval<T>& other )
  {
      if (other.is_empty()) return;
      if (other.lo < lo) lo = other.lo;
      if (other.hi > hi) hi = other.hi;
  }
};

template<typename T>
void
merge_intervals_in_place( std::vector<Interval<T>>& intervals )
{
  if( intervals.empty() )
  {
    return;
  }

  // Sort by low start
  std::sort( intervals.begin(), intervals.end(), []( const Interval<T>& a, const Interval<T>& b ) {
    return a.lo < b.lo;
  } );

  size_t out = 0;
  for( size_t i = 1; i < intervals.size(); ++i )
  {
    if( intervals[out].overlaps( intervals[i] ) || std::abs(intervals[out].hi - intervals[i].lo) < 1e-9 )
    {
      intervals[out].hi = std::max( intervals[out].hi, intervals[i].hi );
    }
    else
    {
      out++;
      intervals[out] = intervals[i];
    }
  }
  intervals.resize( out + 1 );
}

} // namespace math
} // namespace adore

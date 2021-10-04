/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "BKE_spline.hh"
#include "UI_interface.h"
#include "UI_resources.h"
#include "node_geometry_util.hh"
#include <iostream>
#include <list>

/**
 * This file implements the Curve 2D Boolean geometry node.
 *
 * BUGS:
 *
 * TODO:
 * Line Segment intersection library
 * Splines to bezier conversion.
 * Optional hard corners so attributes like radius stick. Or maybe an enum to define which curve should provide attributes for intersection points.
 * Conformity slider : If 1, map intersection points to geometry, if 0, map to bezier curve. Interpolate between.
 * If either of the curves self intersect, show a warning.
 */

static bNodeSocketTemplate geo_node_curve_bool_in[] = {
    {SOCK_GEOMETRY, N_("Curve B")},
    {SOCK_GEOMETRY, N_("Curve A")},
    {-1, ""},
};

static bNodeSocketTemplate geo_node_curve_bool_out[] = {
    {SOCK_GEOMETRY, N_("Curve")},
    {-1, ""},
};

static void geo_node_curve_bool_layout(uiLayout *layout, bContext *UNUSED(C), PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "domain", 0, "", ICON_NONE);
}

/**
 * DEBUG CODE START
 */

static void print(std::string s)
{
  std::cout << s << std::endl << std::flush;
}

static std::chrono::_V2::system_clock::time_point __time;

/**
 * See stop_clock()
 */
static void start_clock()
{
  __time = std::chrono::high_resolution_clock::now();
}

/**
 * Print the time since the last start_clock call in ns.
 */
static void stop_clock(std::string label)
{
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - __time);
  std::cout << "Time taken by function " << label << " : " << duration.count() << " microseconds" << std::endl << std::flush;
}

/**
 * DEBUG CODE END
 */

namespace blender::nodes
{
/**
 * Does the curve contain the point?
 * This is always false if the curve is open.
 */
static bool curve_contains(Spline *spline, blender::float3 point)
{
  if (!spline->is_cyclic())  // ignore if not cyclic
    return false;

  blender::Span<blender::float3> points = spline->evaluated_positions();
  int points_length = points.size();
  int i, j, c = 0;
  for (i = 0, j = points_length - 1; i < points_length; j = i++)
  {
    if (((points[i][1] > point[1]) != (points[j][1] > point[1])) && (point[0] < (points[j][0] - points[i][0]) * (point[1] - points[i][1]) / (points[j][1] - points[i][1]) + points[i][0]))
      c = !c;
  }
  return c;  // if even, point is outside ( return false )
}

/**
 * Count the number of splines that contain the given point.
 * Ignore up to one particular spline, usually the one the point is located on
 */
static int all_curve_contains(Span<Spline *> splines, float3 point, const Spline *except = nullptr)
{
  int counter = 0;
  for (Spline *spline : splines)
  {
    if ( (spline != except) && curve_contains(spline, point))
      counter++;
  }
  return counter;
}

/**
 * Find intersection between 2 2d line segments, each defined by a start and end point. Z
 * Coordinates are ignored. If either of the 2 floats returned is negative, no collision occured.
 * If they are not negative, they are in [0,1]
 */
static blender::float2 linesIntersect(blender::float3 A, blender::float3 B, blender::float3 C, blender::float3 D)
{
  float CmA_x = C.x - A.x;
  float CmA_y = C.y - A.y;
  float BmA_x = B.x - A.x;
  float BmA_y = B.y - A.y;
  float DmC_x = D.x - C.x;
  float DmC_y = D.y - C.y;

  float CmAxBmA = CmA_x * BmA_y - CmA_y * BmA_x;
  float CmAxDmC = CmA_x * DmC_y - CmA_y * DmC_x;
  float BmAxDmC = BmA_x * DmC_y - BmA_y * DmC_x;

  if (CmAxBmA == 0.f)  // Lines are collinear. They intersect if they have any overlap. But we don't want that because we need discrete intersections.
    return blender::float2(-1, -1);

  float rxsr = 1.f / BmAxDmC;
  float t = CmAxDmC * rxsr;
  float u = CmAxBmA * rxsr;

  bool result = (t >= 0.f) && (t <= 1.f) && (u >= 0.f) && (u <= 1.f);
  if (!result)
    return blender::float2(-1, -1);

  return blender::float2(t, u);
}

struct CurveHull;

struct Intersection
{
  float v;               // distance between the current point and the next, in [0,1)
  Intersection *target;  // segment this segment intersects with
  int _target_i;         // index within target's "intersections" vector
  CurveHull *hull;       // self
  int pass = -1;
};

// Generate COMPLETE map of all connected points and intersections as a double linked list.
struct CurveHull
{
  int curve_id;  // used to identify which curve we're on, which is useful if self-intersection is disabled.
  Spline *spline;
  int curve_i;  // index of evaluated point on curve
  float v;      // distance along curve, in [0,1)
  blender::float3 position;
  CurveHull *next = nullptr;
  CurveHull *prev = nullptr;
  Intersection *intersection = nullptr;       // If not nullptr, this is a virtual CurveHull point, that denotes an intersection.
  std::vector<Intersection *> intersections;  // Used to collect intersections without creating additional segments. Only control points have anything in here.
  int pass = -1;
  int is_inside = -1;  // used when tracing. If true, the next intersection is passed right through. 0 means false, 1 means true, -1 means undefined

  // Find the next control point that HAS intersections
  CurveHull *next_intersection_control(bool include_self = false, CurveHull *__start = nullptr)
  {
    if (include_self && ((intersections.size() > 0)))
      return this;

    if (next == nullptr)
      return nullptr;

    if (__start == this)  // don't loop
      return nullptr;

    if (__start == nullptr)
      __start = this;

    return next->next_intersection_control(true, __start);
  }

  // Find the next point that IS an intersection
  CurveHull *next_intersection(bool include_self = false, CurveHull *__start = nullptr)
  {
    if (include_self && intersection != nullptr)
      return this;

    if (next == nullptr)
      return nullptr;

    if (__start == this)  // don't loop
      return nullptr;

    if (__start == nullptr)
      __start = this;

    return next->next_intersection(true, __start);
  }

  /**
   * Removes this and all following points as well as their intersection structs.
   * If a control point gets removed, none of the intersections along its segment get removed. You must remove the actual intersection CurveHullPoints for that.
   */
  void remove()
  {
    if (prev != nullptr)
      prev->next = nullptr;

    if (next != nullptr)
    {
      next->prev = nullptr;
      next->remove();
    }
    if (intersection != nullptr)
      delete intersection;

    delete this;
  }
};

/**
 * Convert a path into a CurveHull form, which is a double linked list with extra variables for tracing along intersections lateron.
 */
static CurveHull *points_to_hull(Spline *spline, int curve_id)
{
  blender::Span<blender::float3> points = spline->evaluated_positions();
  bool cycle = spline->is_cyclic();

  CurveHull *start = new CurveHull{.curve_id = curve_id, .spline = spline, .curve_i = 0, .v = 0, .position = points[0], .next = nullptr, .prev = nullptr};
  CurveHull *prev = start;

  int size = points.size();
  int real_size = size + (cycle ? 0 : -1);
  for (int i = 1; i < size; i++)
  {
    CurveHull *current = new CurveHull{.curve_id = curve_id, .spline = spline, .curve_i = i, .v = i / (float)real_size, .position = points[i], .prev = prev};
    prev->next = current;
    prev = current;
  }

  if (cycle)
  {
    prev->next = start;
    start->prev = prev;
  }

  return start;
}

/**
 * Check if 2 Bounds intersect
 * Each bounds must consist of 2 float2s, first min point, then max point
 */
static bool bounds_intersect(float2 bounds_a_min, float2 bounds_a_max, float2 bounds_b_min, float2 bounds_b_max)
{
  return (bounds_a_min.x < bounds_b_max.x) && (bounds_a_max.x > bounds_b_min.x) &&(bounds_a_min.y < bounds_b_max.y) && (bounds_a_max.y > bounds_b_min.y) ;
}

/**
 * Creates a representation of all curves as linked lists between evaluated points.
 * Finds all intersections between curves and adds them as new points to the curves. These new points contain references to the matching new point on the other curve.
 * We're essentially turning the splines into linked lists of points into linked meshes of points connected by common pairs of intersection points.
 */
static std::list<CurveHull *> bezierIntersectAll2(std::vector<std::vector<Spline *>> splines)
{
  // TODO : Option to allow self intersection in exchange for brute force collision algorithm
  // TODO : Option for convex shapes ( a lot more optimizable )

  std::list<CurveHull *> results;
  std::list<CurveHull *> frontier;

  // TODO : Create Binary search tree from bounds (Needs some kind of clumping heuristics.)

  std::vector<float3> bounds;

  int c = splines.size();
  for (int i = 0; i < c; i++)
  {
    for (Spline *_spline : splines[i])
    {
      float3 min = float3(INFINITY,INFINITY,INFINITY);
      float3 max = float3(-INFINITY,-INFINITY,-INFINITY);
      _spline->bounds_min_max(min, max, true);
      bounds.push_back(min);
      bounds.push_back(max);

      frontier.push_back(points_to_hull(_spline, i));
      CurveHull *last = frontier.back();
      int counter = all_curve_contains(last->curve_id == 0 ? splines[1] : splines[0], last->position);
      last->is_inside = counter % 2 == 1;
    }
  }

  c = frontier.size();
  int pass = 0;
  int i = -1;  // refers to bounds array
  while (frontier.size() > 0)
  {
    i++;
    CurveHull *primary_curve = frontier.front();
    frontier.pop_front();

    int j = i;  // refers to bounds array
    for (CurveHull *curve_hull_b : frontier)
    {
      pass++;
      j++;

      if(primary_curve->curve_id == curve_hull_b->curve_id || !bounds_intersect(bounds[i * 2], bounds[i * 2 + 1], bounds[j * 2], bounds[j * 2 + 1]))//
        continue;

      CurveHull *current_curve_hull_a = primary_curve;
      CurveHull *next_curve_hull_a = primary_curve->next;
      while (current_curve_hull_a != nullptr && next_curve_hull_a != nullptr && current_curve_hull_a->pass != pass)
      {
        current_curve_hull_a->pass = pass;

        CurveHull *current_curve_hull_b = curve_hull_b;
        CurveHull *next_curve_hull_b = curve_hull_b->next;
        do  // b loops from start to start, and won't loop new segments because they aren't cyclic.
        {
          blender::float2 intersection = linesIntersect(current_curve_hull_a->position, next_curve_hull_a->position, current_curve_hull_b->position, next_curve_hull_b->position);

          if (intersection[0] > 0)  // Found intersection
          {
            auto intersection_a = new Intersection{v : intersection[0], hull : current_curve_hull_a};
            auto intersection_b = new Intersection{v : intersection[1], target : intersection_a, hull : current_curve_hull_b};
            intersection_a->target = intersection_b;
            current_curve_hull_a->intersections.push_back(intersection_a);
            current_curve_hull_b->intersections.push_back(intersection_b);
          }

          current_curve_hull_b = current_curve_hull_b->next;
          next_curve_hull_b = current_curve_hull_b->next;
        } while (next_curve_hull_b != nullptr && current_curve_hull_b->curve_i != 0);

        current_curve_hull_a = next_curve_hull_a;
        next_curve_hull_a = current_curve_hull_a->next;
      }
    }

    results.push_back(primary_curve);
    pass++;
  }

  // Tell the intersection point on the opposite curve about the final index.
  // Doing this here makes it run in O(n) as opposed to O(nlogn) if we were to search by v lateron.
  for (CurveHull *current_point : results)
  {
    CurveHull *start = current_point;
    float curve_length = current_point->spline->evaluated_points_size() + (current_point->spline->is_cyclic() ? 0 : -1);
    do // Then create all intersection points as actual points. We didn't do this earlier to avoid extra intersection calculations with the new line segments.
    {
      std::vector<Intersection *> &intersections = current_point->intersections;
      // Sort the intersections by progress along line / v.
      std::sort(intersections.begin(), intersections.end(), [](Intersection *a, Intersection *b) { return a->v < b->v; });

      CurveHull *original_current_point = current_point;
      CurveHull *original_next_point = current_point->next;
      for (Intersection *intersection : intersections)
      {
        CurveHull *new_point = new CurveHull{.curve_id = current_point->curve_id, .spline = current_point->spline, .curve_i = current_point->curve_i, .v = original_current_point->v + intersection->v / curve_length, .position = float3::interpolate(original_current_point->position, original_next_point->position, intersection->v), .next = current_point->next, .prev = current_point, .intersection = intersection};
        intersection->hull = new_point;
        current_point->next->prev = new_point;
        current_point->next = new_point;
        current_point = new_point;
      }

      int i = 0;
      for (auto intersection : current_point->intersections)
      {
        intersection->_target_i = i++;
      }

      current_point = current_point->next;
    } while (current_point != nullptr && current_point != start);
  }

  return results;
}

/**
 * Switch the direction of the spline.
 */
static int switch_direction(Spline *spline)
{
  Spline::Type type = spline->type();
  MutableSpan<float3> points = spline->positions();
  MutableSpan<float> tilts = spline->tilts();
  MutableSpan<float> radii = spline->radii();

  int length = points.size();

  if (type == Spline::Type::Bezier)
  {
    MutableSpan<float3> handles_l = ((BezierSpline *)spline)->handle_positions_left();
    MutableSpan<float3> handles_r = ((BezierSpline *)spline)->handle_positions_right();

    for (int i = 0; i < length / 2; i++)
    {
      std::swap(points[i], points[length - i - 1]);
      std::swap(handles_l[i], handles_r[length - i - 1]);
      std::swap(handles_r[i], handles_l[length - i - 1]);
      std::swap(tilts[i], tilts[length - i - 1]);
      std::swap(radii[i], radii[length - i - 1]);
    }

    if (length % 2)
      std::swap(handles_l[length / 2], handles_r[length / 2]);
  }
  else  // Not Bezier
  {
    for (int i = 0; i < length / 2; i++)
    {
      std::swap(points[i], points[length - i - 1]);
      std::swap(tilts[i], tilts[length - i - 1]);
      std::swap(radii[i], radii[length - i - 1]);
    }
  }

  spline->mark_cache_invalid();

  return 0;
}

/**
 * Same ids as "curve_bool_items" in NOD_static_types.h
 */
typedef enum BoolDomainType
{
  OR = 0,
  AND = 1,
  SUB = 2,
} BoolDomainType;

/**
 * A virtual Spline point. Contains all the data to create a real (bezier-)spline point from, but is all in one place (real splines have separate arrays for each individual attribute)
 */
struct SplinePoint
{
  float3 position;
  float3 handle_position_left;
  float3 handle_position_right;
  float radius;
  float tilt;

  SplinePoint(BezierSpline *curve, int index)
  {
    position = curve->positions()[index];
    handle_position_left = curve->handle_positions_left()[index];
    handle_position_right = curve->handle_positions_right()[index];
    radius = curve->radii()[index];
    tilt = curve->tilts()[index];
  }

  SplinePoint(Spline *curve, int index)
  {
    position = curve->positions()[index];
    radius = curve->radii()[index];
    tilt = curve->tilts()[index];
  }

  SplinePoint(float3 position, float3 handle_position_left, float3 handle_position_right, float radius, float tilt) : position(position), handle_position_left(handle_position_left), handle_position_right(handle_position_right), radius(radius), tilt(tilt)
  {
  }

  SplinePoint(float3 position, float radius, float tilt) : position(position), radius(radius), tilt(tilt)
  {
  }
};

/**
 * Given a bezier curve segment described by 2 points and 2 handles, generate an extra point at v along the curve and return
 * both the new point's position and handles, as well as the modified handles of the initial points.
 */
static BezierSpline::InsertResult calculate_bezier_segment_insertion(float3 pos_prev, float3 handle_prev, float3 pos_next, float3 handle_next, float v)
{
  const float3 center_point = float3::interpolate(handle_prev, handle_next, v);

  BezierSpline::InsertResult result;
  result.handle_prev = float3::interpolate(pos_prev, handle_prev, v);
  result.handle_next = float3::interpolate(handle_next, pos_next, v);
  result.left_handle = float3::interpolate(result.handle_prev, center_point, v);
  result.right_handle = float3::interpolate(center_point, result.handle_next, v);
  result.position = float3::interpolate(result.left_handle, result.right_handle, v);
  return result;
}

/**
 * Checks if the current point is a control point. If so, add it to the result and set last_intersection_offset to 0.
 */
static float process_control_point(CurveHull *current_point, std::vector<SplinePoint> &result, Spline::Type type, float last_intersection_offset = 0)
{
  int segment_count = current_point->spline->segments_size();
  int resolution = current_point->spline->evaluated_points_size() / segment_count;

  float current_absolute_v = current_point->curve_i / (float)(resolution);  // control point index + v along segment
  int current_control_point = (int)(current_absolute_v);

  if (type == Spline::Type::Bezier)
  {
    if ((current_point->curve_i % resolution) != 0)
      return last_intersection_offset;
    BezierSpline *current_spline = (BezierSpline *)current_point->spline;

    result.push_back(SplinePoint(current_spline->evaluated_positions()[current_point->curve_i], float3::interpolate(current_spline->handle_positions_left()[current_control_point], current_spline->positions()[current_control_point], last_intersection_offset), current_spline->handle_positions_right()[current_control_point], current_spline->radii()[current_control_point], current_spline->tilts()[current_control_point]));
  }
  else
  {
    Spline *current_spline = current_point->spline;

    int current_segment_end = (current_control_point + 1) % current_spline->positions().size();
    float current_relative_v = fmod(current_absolute_v, 1);
    result.push_back(SplinePoint(current_spline->evaluated_positions()[current_point->curve_i], (1 - current_relative_v) * current_spline->radii()[current_control_point] + current_relative_v * current_spline->radii()[current_segment_end], (1 - current_relative_v) * current_spline->tilts()[current_control_point] + current_relative_v * current_spline->tilts()[current_segment_end]));
  }

  return 0;
}

/**
 * Process a CurveHull point that is an intersection.
 * Update frontier with paths not taken, and create BezierSplinePoints with interpolated bezier handles.
 */
static CurveHull *process_intersection(CurveHull *current_point, std::vector<SplinePoint> &result, float &last_intersection_offset, std::list<CurveHull *> &frontier, Spline::Type type, bool pass_through = false)
{
  bool first = result.size() == 0;
  float v_next = 0;
  Intersection *current_intersection = current_point->intersection;

  CurveHull *other_point = current_intersection->target->hull;

  int segment_count = current_point->spline->segments_size();
  int segment_count_next = other_point->spline->segments_size();

  if (type == Spline::Type::Bezier)
  {
    BezierSpline *current_spline = (BezierSpline *)current_point->spline;
    BezierSpline *other_spline = (BezierSpline *)other_point->spline;

    int current_resolution = current_spline->evaluated_points_size() / segment_count;
    int next_resolution = other_spline->evaluated_points_size() / segment_count_next;

    float current_absolute_v = current_point->curve_i / (float)(current_resolution);  // TODO : How do we derive the same from current_point->v? current_point->v * segment_count
    int current_control_point = (int)(current_absolute_v);
    float current_relative_v = fmod(current_absolute_v, 1);

    float next_absolute_v = other_point->curve_i / (float)(next_resolution);
    int next_control_point = (int)(next_absolute_v);
    float next_relative_v = fmod(next_absolute_v, 1);

    int current_segment_end = (current_control_point + 1) % current_spline->positions().size();

    float v_cur = (current_relative_v + current_intersection->v / current_resolution - last_intersection_offset) / (1 - last_intersection_offset);

    blender::float3 last_pos = first ? current_spline->positions()[current_control_point] : result.back().position;
    blender::float3 last_handle = first ? current_spline->handle_positions_right()[current_control_point] : result.back().handle_position_right;
    BezierSpline::InsertResult insert_current = calculate_bezier_segment_insertion(last_pos, last_handle, current_spline->positions()[current_segment_end], float3::interpolate(current_spline->handle_positions_left()[current_segment_end], current_spline->positions()[current_segment_end], last_intersection_offset), v_cur);

    int next_segment_end = (next_control_point + 1) % other_spline->handle_positions_left().size();
    v_next = next_relative_v + current_intersection->target->v / next_resolution;

    BezierSpline::InsertResult insert_next = calculate_bezier_segment_insertion(other_spline->positions()[next_control_point], other_spline->handle_positions_right()[next_control_point], other_spline->positions()[next_segment_end], other_spline->handle_positions_left()[next_segment_end], v_next);

    if (result.size() > 0)
      result.back().handle_position_right = insert_current.handle_prev;

    float3 real_pos = current_intersection->hull->position;  // the intersection point on the EVALUATED geometry

    result.push_back(SplinePoint(real_pos,
                                 insert_current.left_handle,  //+ real_pos - bezier_pos_current
                                 insert_next.right_handle,    // + real_pos - bezier_pos_next
                                 (1.0 - next_relative_v) * current_spline->radii()[current_control_point] + next_relative_v * current_spline->radii()[current_segment_end],
                                 (1.0 - next_relative_v) * current_spline->tilts()[current_control_point] + next_relative_v * current_spline->tilts()[current_segment_end]));
  }
  else
  {
    Spline *current_spline = current_point->spline;

    int current_resolution = current_spline->evaluated_points_size() / segment_count;
    float current_absolute_v = current_point->curve_i / (float)(current_resolution);  // TODO : How do we derive the same from current_point->v? current_point->v * segment_count
    int current_control_point = (int)(current_absolute_v);
    int current_segment_end = (current_control_point + 1) % current_spline->positions().size();

    auto radii = current_spline->radii(); // BUG : Why does radius sometimes spazz out? This is absurd... (Happens only with PolyLine)
    float3 real_pos = current_intersection->hull->position;
    result.push_back(SplinePoint(
      real_pos,
      (1.0 - current_intersection->v) * radii[current_control_point] + current_intersection->v * radii[current_segment_end],
      (1.0 - current_intersection->v) * current_spline->tilts()[current_control_point] + current_intersection->v * current_spline->tilts()[current_segment_end]));
  }

  // if an intersection is the first point to be processed, keep following the current curve and add the other curve to the frontier instead
  if (first && pass_through)
  {
    last_intersection_offset = v_next;
    other_point->pass = -2;

    CurveHull *next_possible_frontier = current_point->next_intersection();
    if (next_possible_frontier != nullptr)
      frontier.push_back(next_possible_frontier->intersection->target->hull);

    return other_point->next;
  }
  else
  {
    last_intersection_offset = v_next;
    other_point->pass = -2;

    CurveHull *next_possible_frontier = current_point->next_intersection();
    if (next_possible_frontier != nullptr)
      frontier.push_back(next_possible_frontier->intersection->target->hull);

    return other_point->next;
  }
}

/**
 * Turn a path WITH NO INTERSECTION into an actual bezier path
 */
static void copy_curve(std::vector<std::vector<SplinePoint>> &results, CurveHull *path, Spline::Type type)
{
  std::vector<SplinePoint> result;

  CurveHull *start_point = path;
  CurveHull *current_point = start_point;

  while (current_point != nullptr)
  {
    process_control_point(current_point, result, type);  // if the current point is a control point, processes it. Otherwise, skips it.

    current_point = current_point->next;

    if (current_point == start_point)
      break;
  };

  results.push_back(result);
}

/**
 * Walk along a CurveHull and turn it into virtual BezierSplinePoints.
 * These will contain all the data needed to turn them into real BezierSplines, but can be tossed more easily if they don't form desired topologies.
 */
static void trace_hull(std::vector<std::vector<SplinePoint>> &results, CurveHull *path, BoolDomainType type, Spline::Type return_type)
{
  std::list<CurveHull *> frontier;
  frontier.push_back(path);

  while (frontier.size() > 0)
  {
    std::vector<SplinePoint> result;
    CurveHull *start_point = frontier.front();
    frontier.pop_front();

    int is_inside = start_point->is_inside;
    bool first = true;

    if (type == BoolDomainType::SUB || type == BoolDomainType::OR)
    {
      if (is_inside == 1)  // For intersection only! How to do this for others as well?
      {
        start_point = start_point->next_intersection();
        if (start_point == nullptr)
          continue;
        start_point = start_point->intersection->target->hull;
      }
    }

    if (type == BoolDomainType::AND)
    {
      first = is_inside == -1;

      if (is_inside == 0)  // For intersection only! How to do this for others as well?
      {
        start_point = start_point->next_intersection();
        if (start_point == nullptr)
          continue;
        start_point = start_point->intersection->target->hull;
      }
    }

    if (start_point->pass == -2)
      continue;

    CurveHull *current_point = start_point;
    CurveHull *next_point = current_point->next;
    if (next_point == nullptr)
      continue;

    float last_intersection_offset = 0;
    do
    {
      current_point->pass = -2;

      // current point is a virtual intersection point
      if (current_point->intersection != nullptr)
      {
        current_point = process_intersection(current_point, result, last_intersection_offset, frontier, return_type, first);
      }
      else
      {
        last_intersection_offset = process_control_point(current_point, result, return_type, last_intersection_offset);  // if the current point is a control point, processes it. Otherwise, skips it.
        current_point = current_point->next;
      }
      is_inside = false;

    } while (current_point != nullptr && (current_point->pass != -2));

    if (current_point != nullptr)  // if cyclical result
    {
      if ((current_point != start_point) && (current_point->intersection == nullptr || current_point->intersection->target->hull != start_point))
        continue;  // skip, didn't reach a valid target.

      if (return_type == Spline::Type::Bezier)
      {
        // if cyclic, make sure the initial point's left handle scales with any previous intersection. (If not cyclic, those handles aren't used)
        if (current_point->intersection != nullptr)
        {
          int segment_count = current_point->spline->segments_size();

          float current_relative_v = fmod((start_point->v * segment_count), 1.0f);

          // both handles are already resized to the following sizes
          float expected_distance_right = 1 - last_intersection_offset;
          float expected_distance_left = current_relative_v;  // should this not be the start's point instead?

          // but because of this new intersection, they will need to be this size instead
          float proportional_distance_right = (1 - current_relative_v) / expected_distance_right;
          float proportional_distance_left = last_intersection_offset / expected_distance_left;

          result.front().handle_position_left = float3::interpolate(result.front().handle_position_left, result.front().position, proportional_distance_left);
          result.back().handle_position_right = float3::interpolate(result.back().handle_position_right, result.back().position, proportional_distance_right);
        }
        else
        {
          result.front().handle_position_left = float3::interpolate(result.front().handle_position_left, result.front().position, last_intersection_offset);
        }
      }
    }
    results.push_back(result);
  }
}

/**
 * Turn a list of virtual BezierSplinePoints into a real BezierSpline
 */
static void trace_to_bezier(std::vector<SplinePoint> trace, BezierSpline *result)
{
  int size = trace.size();
  for (int i = 0; i < size; i++)
  {
    SplinePoint point = trace[i];
    result->add_point(point.position, BezierSpline::HandleType::Free, point.handle_position_left, BezierSpline::HandleType::Free, point.handle_position_right, point.radius, point.tilt);
  }
}

/**
 * Turn a list of virtual SplinePoints into a real PolySpline
 */
static void trace_to_poly(std::vector<SplinePoint> trace, PolySpline *result)
{
  int size = trace.size();
  for (int i = 0; i < size; i++)
  {
    SplinePoint point = trace[i];
    result->add_point(point.position, point.radius, point.tilt);
  }
}

static bool is_curve_clockwise(Spline *spline)
{
  Span<float3> points = spline->evaluated_positions();

  float dotsum = 0;  // calculates 2x enclosed area. If negative, curve is counter clockwise
  for (int i = 0; i < points.size() - 1; i++)
  {
    dotsum += (points[i + 1].x - points[i].x) * (points[i + 1].y + points[i].y);
  }

  if (spline->is_cyclic())
    dotsum += (points[0].x - points[points.size() - 1].x) * (points[0].y + points[points.size() - 1].y);

  return dotsum >= 0;
}

/**
 * I don't know how to work with blender's Spans or with the unique pointer replacements. So we'll convert them to a vector of regular pointers for the time being. We're not violating the unique pointer anyways.
 */
static std::vector<Spline *> _unrwap_splines(blender::Span<SplinePtr> s)
{
  std::vector<Spline *> result = {};
  for (const SplinePtr &spline : s)
  {
    result.push_back(spline.get());
  }
  return result;
}

/**
 * Generate one or more new curves from 2 existing sets of curves
 * These curves must not self intersect.
 * The general idea is to follow one of the curves and copy the control points until an
 * intersection is found. At which point we know the other curve is bigger, so we switch to the
 * other one. We do this for each intersection until we reach our initial position.
 */
static std::unique_ptr<CurveEval> generate_boolean_shapes(const CurveEval *a, const CurveEval *b, BoolDomainType type)
{
  const int resolution = 12;
  std::unique_ptr<CurveEval> result = std::make_unique<CurveEval>();

  std::vector<Spline *> splines_a = _unrwap_splines(a->splines());  // SplinePtr is a unique ptr. We're not modifying those, we just want their data. So we cast to Spline* from the get-go.
  std::vector<Spline *> splines_b = _unrwap_splines(b->splines());
  std::vector<std::vector<Spline *>> splines = {splines_a, splines_b};

  Spline::Type result_type = Spline::Type::Bezier;

  for (Span<Spline *> _splines : splines)
  {
    for (Spline *spline : _splines)
    {
      if (spline->type() != Spline::Type::Bezier)
        result_type = Spline::Type::Poly;
    }
  }

  for (Span<Spline *> _splines : splines)
  {
    for (Spline *spline : _splines)
    {
      // All bezier handles must be free or else updating the curve will "correct" them.
      if (spline->type() == Spline::Type::Bezier)
      {
        // TODO : If result_type == Poly, convert to bezier.

        BezierSpline *bezier_spline = ((BezierSpline *)spline);
        MutableSpan<BezierSpline::HandleType> handles_left_t = bezier_spline->handle_types_left();
        MutableSpan<BezierSpline::HandleType> handles_right_t = bezier_spline->handle_types_right();
        bezier_spline->handle_positions_left();  // Bug : Don't remove ! This needs to be called or else the auto->free handle change loses the handle data. (It presumably flips some dirty flag that the handle_types alone don't but should)
        bezier_spline->handle_positions_right();
        int size = handles_left_t.size();
        for (int i = 0; i < size; i++)
        {
          handles_left_t[i] = BezierSpline::HandleType::Free;
          handles_right_t[i] = BezierSpline::HandleType::Free;
        }

        spline->mark_cache_invalid();
      }

      // All curves must be clockwise, unless the algorithm turns them explicitly.
      bool is_clockwise = is_curve_clockwise(spline);

      // Outside curves must go clockwise
      // Inside curves must go counterclockwise
      int counter = all_curve_contains(_splines, spline->positions()[0], spline);
      if (counter % 2 == (is_clockwise ? 1 : 0))  // curve represents negative space, must go counterclockwise
        switch_direction(spline);
    }
  }

  // Some operations need one curve to go clockwise, and the other counter-clockwise.
  if (type == BoolDomainType::AND)
  {
    for (Spline *spline : splines_a)
    {
      switch_direction(spline);
    }
    for (Spline *spline : splines_b)
    {
      switch_direction(spline);
    }
  }
  else if (type == BoolDomainType::SUB)
  {
    for (Spline *spline : splines_b)
    {
      switch_direction(spline);
    }
  }

  /// FIRST, FIND ALL INTERSECTIONS.
  std::list<CurveHull *> paths = bezierIntersectAll2(splines);

  int length = 0;

  for (CurveHull *path : paths)
  {
    if (path == nullptr)
      print("Loop is nullptr");
    std::vector<std::vector<SplinePoint>> results;
    if (path->next_intersection() == nullptr)  // Path does not intersect with anything
    {
      // count the number of curves that contain this one. Depending on the mode, and if it's contained in the other curve, toss it, or keep it as-is.
      int counter = all_curve_contains(path->curve_id == 0 ? splines_b : splines_a, path->position);
      if (type == BoolDomainType::SUB)
      {
        if (counter % 2 == (path->curve_id == 0 ? 0 : 1))
          copy_curve(results, path, result_type);
      }
      else if (type == BoolDomainType::OR)
      {
        if (counter % 2 == 0)
          copy_curve(results, path, result_type);
      }
      else if (type == BoolDomainType::AND)
      {
        if (counter % 2 == 1)
          copy_curve(results, path, result_type);
      }
    }
    else
      trace_hull(results, path, type, result_type);

    // Now turn our virtual control points into a real bezier curve
    if (result_type == Spline::Type::Bezier)
    {
      for (std::vector<SplinePoint> trace : results)
      {
        std::unique_ptr<BezierSpline> result_spline = std::make_unique<BezierSpline>();
        result_spline->set_resolution(resolution);
        trace_to_bezier(trace, result_spline.get());
        result_spline->set_cyclic(true);
        result->add_spline(std::move(result_spline));
        length += result->splines().size();
      }
    }
    else
    {
      for (std::vector<SplinePoint> trace : results)
      {
        std::unique_ptr<PolySpline> result_spline = std::make_unique<PolySpline>();
        trace_to_poly(trace, result_spline.get());
        result_spline->set_cyclic(true);
        result->add_spline(std::move(result_spline));
        length += result->splines().size();
      }
    }
  }

  // Remove all CurveHull and Intersection helper structs. These were created with new().
  for (CurveHull *path : paths)
  {
    path->remove();
  }

  result->attributes.reallocate(length);

  return result;
}

/**
 * This is called whenever the node needs to update, such as when inputs change
 */
static void geo_node_curve_bool_exec(GeoNodeExecParams params)
{
  start_clock();
  GeometrySet curve_set_a = params.extract_input<GeometrySet>("Curve A");
  curve_set_a = bke::geometry_set_realize_instances(curve_set_a);

  GeometrySet curve_set_b = params.extract_input<GeometrySet>("Curve B");
  curve_set_b = bke::geometry_set_realize_instances(curve_set_b);

  if (!curve_set_a.has_curve() || !curve_set_b.has_curve())
  {
    params.set_output("Curve", curve_set_a);
    return;
  }

  const bNode &node = params.node();
  const BoolDomainType data_type = static_cast<BoolDomainType>(node.custom2);

  std::unique_ptr<CurveEval> curve = generate_boolean_shapes(curve_set_a.get_curve_for_read(), curve_set_b.get_curve_for_read(),
                                                             data_type);  //,curve.get()
  params.set_output("Curve", GeometrySet::create_with_curve(curve.release()));

  stop_clock("Bezier Intersect");
}

}  // namespace blender::nodes

void register_node_type_geo_curve_bool()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_CURVE_BOOL, "Curve Bool", NODE_CLASS_GEOMETRY, 0);
  node_type_socket_templates(&ntype, geo_node_curve_bool_in, geo_node_curve_bool_out);
  ntype.geometry_node_execute = blender::nodes::geo_node_curve_bool_exec;
  ntype.draw_buttons = geo_node_curve_bool_layout;
  nodeRegisterType(&ntype);
}

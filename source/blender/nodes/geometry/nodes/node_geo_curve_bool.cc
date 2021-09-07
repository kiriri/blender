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

#include "UI_interface.h"
#include "UI_resources.h"
#include "BKE_spline.hh"
#include "node_geometry_util.hh"
#include <iostream>
#include <list>

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

namespace blender::nodes
{

// Step 1 :
// Find function that finds intersections
// Iterate shape A points, find intersections for each segment, add them as another point, then
// switch to shape B from that point onwards until the next intersection is found, then switch back
// again etc etc

/**
 * Find intersection between 2 2d line segments, each defined by a start and end point. Z
 * Coordinates are ignored. If either of the 2 floats returned is negative, no collision occured.
 * If they are not negative, they are in [0,1]
 */
static blender::float2 linesIntersect(blender::float3 A,
                                      blender::float3 B,
                                      blender::float3 C,
                                      blender::float3 D)
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

  if (CmAxBmA == 0.f)  // Lines are collinear, and so intersect if they have any overlap
    if (((C.x - A.x < 0.f) != (C.x - B.x < 0.f)) || ((C.y - A.y < 0.f) != (C.y - B.y < 0.f)))
      return blender::float2(0, 0);
    else
      return blender::float2(-1, -1);

  if (BmAxDmC == 0.f)  // Lines are parallel.
    return blender::float2(-1, -1);

  float rxsr = 1.f / BmAxDmC;
  float t = CmAxDmC * rxsr;
  float u = CmAxBmA * rxsr;

  bool result = (t >= 0.f) && (t <= 1.f) && (u >= 0.f) && (u <= 1.f);
  if (result)
    std::cout << t << " " << u << std::endl
              << std::flush;  // intersection at t on curve A and u on curve B
  else
    return blender::float2(-1, -1);

  return blender::float2(t, u);
}

/**
 * Find the first intersection between 2 2d bezier curves. Z Coordinates are ignored.
 * If either of the 2 floats returned is negative, no collision occured. If they are not negative,
 * they are in [0,1]
 */
static blender::float2 bezierIntersect(blender::Span<blender::float3> points_a,
                                       blender::Span<blender::float3> points_b,
                                       int start_a = 0,
                                       bool ignore_zero = false)
{
  for (int i_a = start_a; i_a < points_a.size() - 1; i_a++)
  {
    for (int i_b = 0; i_b < points_b.size() - 1; i_b++)
    {
      blender::float2 intersection = linesIntersect(
          points_a[i_a], points_a[i_a + 1], points_b[i_b], points_b[i_b + 1]);
      if (ignore_zero ? i_a != start_a && (intersection[0] > (0.001) && intersection[1] > 0.001) :
                        intersection[0] >= 0)
      {
        std::cout << "Int " << intersection[0] << " < "
                  << (0.001 + start_a / (float)points_a.size()) << std::endl
                  << std::flush;
        ;
        intersection[0] = (intersection[0] + i_a) / (points_a.size() - 1);
        intersection[1] = (intersection[1] + i_b) / (points_b.size() - 1);
        return intersection;
      }

      if (ignore_zero && ((intersection[0] >= 0) ||
                          (intersection[1] >= 0) &&
                              (intersection[0] <= (0.0001 + start_a / (float)points_a.size()) ||
                               intersection[1] <= 0.0001)))
        std::cout << "Ignored Duplicate " << intersection[0] << std::endl << std::flush;
    }
  }
  return blender::float2(-1, -1);
}


/**
 * Find the intersections between 2 2d bezier curves. Z Coordinates are ignored.
 * After each intersection, switch to scan along the other curve from the intersection point onwards,
 * zigzagging along until the end of either curve is reached.
 * Results are in [0,1] and represent the intersection point along the curve. Keep in mind that at uneven indices, the first and second curve are flipped, and so are the results.
 */
static std::list<blender::float2> bezierIntersectAll(blender::Span<blender::float3> points_a,
                                       blender::Span<blender::float3> points_b)
{
  std::list<blender::float2> results;

  int loop_terminator = 0;

  float a_line_offset = 0; // Whenever an intersection occurs, points_a and points_b get swapped. Remember how far along the line segment the intersection occured to avoid repeating it right after.

  // TODO : How to terminate? Reach same point (by index)
  bool* passed_a = new bool[points_a.size()]{false};
  bool* passed_b = new bool[points_b.size()]{false};

  for (int i_a = 0; true ; i_a++  )
  {
    if((i_a + 1) >= points_a.size())
      i_a = 0;
    loop_terminator++;
    if(loop_terminator > 1000)
      break;
    if(passed_a[i_a]) // Already processed this point -> Did a full loop.
      return results;
    passed_a[i_a] = true;

    blender::float2 earliest_intersection = blender::float2(2.0,0);
    int earliest_segment_i_b = 0;

    for (int i_b = 0; i_b < points_b.size() - 1; i_b++)
    {
      blender::float2 intersection = linesIntersect(
          a_line_offset * points_a[i_a + 1] + (1 - a_line_offset) * points_a[i_a], points_a[i_a + 1], points_b[i_b], points_b[i_b + 1]);

      if(intersection[0] >= 0)
      {
        intersection[0] = (intersection[0] + a_line_offset + i_a) / (points_a.size() );
        intersection[1] = (intersection[1] + i_b) / (points_b.size() );



        if(intersection[0] <  earliest_intersection[0])
        {
          earliest_intersection = intersection;
          earliest_segment_i_b = i_b;
        }
      }
    }
    if (earliest_intersection[0] <= 1.0)
    {

      results.push_back(earliest_intersection);

      // TODO : a_line_offset and passed_a[i_a] = false

      i_a = earliest_intersection[1] * points_b.size();
      std::cout << "I_A " << i_a << std::endl << std::flush;
      std::swap(points_a,points_b);
      std::swap(passed_b,passed_a);
      // passed_a[i_a] = false; // make sure looping back into the middle of a line segment does not cause a loop if we originally came from that line segment
    }
  }
  return results;
}



void print_a(std::list<blender::float2> const &list)
{
    for (auto const &i: list) {
        std::cout << i << std::endl << std::flush;
    }
}


void print(std::string s)
{
	std::cout << s << std::endl << std::flush;
}

static bool curve_contains(Spline* spline, blender::float3 point)
{
  blender::Span<blender::float3> points = spline->evaluated_positions();
  int points_length = points.size();
  int i, j, c = 0;
  for (i = 0, j = points_length-1; i < points_length; j = i++) {
    if ( ((points[i][1]>point[1]) != (points[j][1]>point[1])) &&
     (point[0] < (points[j][0]-points[i][0]) * (point[1]-points[i][1]) / (points[j][1]-points[i][1]) + points[i][0]) )
       c = !c;
  }
  return c; // if even, point is outside ( return false )
}




/**
 * Copy the segments from one curve to the end of another
 * Do until, but not including, target segment id.
 * Loop if cyclic.
 */
static int copy_segments(int end_a, int current_source_segment, BezierSpline* target_spline, BezierSpline* source_spline, int source_segment_count, int source_control_point_count, int offset = 1)
{
  print("COPYING SEGMENTS FROM " + std::to_string(current_source_segment) + " TO " + std::to_string(end_a) );
  int result = 0;
  for(; (current_source_segment != end_a ); current_source_segment = mod_i(current_source_segment + 1, source_segment_count) )
  {
    int next_control_point = mod_i(current_source_segment + offset, source_control_point_count);

    print("NEXT SEGMENT2 " + std::to_string(next_control_point));
    target_spline->add_point( source_spline->positions()[next_control_point ],
                              BezierSpline::HandleType::Free,
                              source_spline->handle_positions_left()[next_control_point ],
                              BezierSpline::HandleType::Free,
                              source_spline->handle_positions_right()[next_control_point ],
                              1.0f,
                              0.0f);

    result++;

  }

  return result;
}

/**
 * Process an intersection between two curves A/B.
 * Generates additional points along the result spline.
 *
 */
static int process_intersection(bool first, blender::float2 intersection, BezierSpline* result_spline, BezierSpline* spline_a,BezierSpline* spline_b, blender::float2 &last_intersection_offset, int last_a_segment)
{
  bool is_cyclic_a = spline_a->is_cyclic();
  bool is_cyclic_b = spline_b->is_cyclic();

  int control_point_count_a = spline_a->positions().size();
  int control_point_count_b = spline_b->positions().size();

  int segment_count_a = control_point_count_a + (is_cyclic_a ? 0 : -1);
  int segment_count_b = control_point_count_b + (is_cyclic_b ? 0 : -1);

  float a_control_point_v = intersection[0] * segment_count_a;
  float b_control_point_v = intersection[1] * segment_count_b;

  print("AV " + std::to_string(a_control_point_v) + " from " + std::to_string(intersection[0]));

  int a_segment = (int)a_control_point_v;
  int b_segment = (int)b_control_point_v;



  int _next_segment_a = mod_i(a_segment + 1, segment_count_a);


  float intersection_offset_a = fmod(a_control_point_v,1.0f);
  float intersection_offset_b = fmod(b_control_point_v,1.0f);


  bool force_point = (last_intersection_offset[0] >= intersection_offset_a) && (last_a_segment == _next_segment_a); // if next intersection occurs in same segment, but earlier, do a full loop

  print("LAST " + std::to_string(last_intersection_offset[0]) + " NEXT " + std::to_string(intersection_offset_a));


  // add all control points that did not have any intersections in their segments
  for(; (last_a_segment != _next_segment_a) || force_point; last_a_segment = mod_i(last_a_segment + 1, segment_count_a) )
  {
    force_point = false;
    int next_control_point = mod_i(last_a_segment + 1, control_point_count_a);

    print("NEXT SEGMENT " + std::to_string(last_a_segment));
    result_spline->add_point(spline_a->positions()[next_control_point ],
                          BezierSpline::HandleType::Free,
                          spline_a->handle_positions_left()[next_control_point ],
                          BezierSpline::HandleType::Free,
                          spline_a->handle_positions_right()[next_control_point ],
                          1.0f,
                          0.0f);
    print("NEW SEGMENT " + std::to_string(last_a_segment));
    //"NEW SEGMENT " << last_a_control_pointstd::cout << "NEW SEGMENT " << last_a_control_point << std::endl << std::flush;
    last_intersection_offset[0] = 0;

  }


/// Now add the next intersection
  int result_segment = result_spline->positions().size() - 2; // calculate the control index along the result
  float result_v = (intersection_offset_a - last_intersection_offset[0]) / (1-last_intersection_offset[0]); // calculate the coordinate along the result (keep in mind the result curve can contain additional "collision" control points)

  std::cout << "GOING IN WITH  " << result_segment << std::endl << std::flush;

  int next_control_point_b = mod_i(b_segment + 1,control_point_count_b);

  BezierSpline::InsertResult insert_a = first ? spline_a->calculate_segment_insertion(a_segment , a_segment + 1, intersection_offset_a) : result_spline->calculate_segment_insertion(result_segment , result_segment + 1, result_v);
  BezierSpline::InsertResult insert_b = spline_b->calculate_segment_insertion(
      b_segment , next_control_point_b , fmod(b_control_point_v,1.0f));

  std::cout << "B Intersection at : " << intersection[1] << " " << intersection_offset_a << ":" << last_intersection_offset[0] << " >> " << result_v << std::endl <<  std::flush;

  std::cout << "A Intersection at : " << intersection[0] << "/" << intersection[1] << " " << intersection_offset_a << ":" << last_intersection_offset[0] << " >> " << result_v << std::endl <<  std::flush;

  last_intersection_offset[0] = intersection_offset_a;
  last_intersection_offset[1] = intersection_offset_b;

  if(first) // if we're only starting at first intersection.
  {
    result_spline->handle_positions_left()[result_spline->positions().size() - 1] =
      insert_a.left_handle;
    result_spline->positions()[result_spline->positions().size() - 1] = insert_b.position;
    result_spline->handle_positions_right()[result_spline->positions().size() - 1] =
        insert_b.right_handle;

  }
  else
  {
    result_spline->handle_positions_right()[result_spline->positions().size() - 2] =
          insert_a.handle_prev;
    result_spline->handle_positions_left()[result_spline->positions().size() - 1] =
        insert_a.left_handle;
    result_spline->positions()[result_spline->positions().size() - 1] = insert_b.position;
    result_spline->handle_positions_right()[result_spline->positions().size() - 1] =
        insert_b.right_handle;
  }



  result_spline->add_point(spline_b->positions()[next_control_point_b],
                          BezierSpline::HandleType::Free,
                          insert_b.handle_next,
                          BezierSpline::HandleType::Free,
                          spline_b->handle_positions_right()[next_control_point_b],
                          1.0f,
                          0.0f);



  print("Set control point to ");
  print(std::to_string(b_segment));

  last_a_segment = mod_i(b_segment+1  , segment_count_b); // assign next point to from b to a knowing they are now switched

  return last_a_segment;
}




typedef enum BoolDomainType {
  OR = 0,
  AND = 1,
} BoolDomainType;

/**
 * Find intersection between 2 curves using subdivision.
 * Then generate a union shape.
 * The general idea is to follow one of the curves and copy the control points until an
 * intersection is found. At which point we know the other curve is bigger, so we switch to the
 * other one. We do this for each intersection until we reach our initial position.
 */
static std::unique_ptr<CurveEval> intersect(const CurveEval *a, const CurveEval *b, BoolDomainType type)
{
  const int resolution = 100;
  std::unique_ptr<CurveEval> result = std::make_unique<CurveEval>();
  std::unique_ptr<BezierSpline> result_spline = std::make_unique<BezierSpline>();
  result_spline->set_resolution(resolution);

  const char *types[] = {"Bezier", "NURBS", "Poly"};
  Span<SplinePtr> splines_a =
      a->splines();  // each CurveEval object can contain several unconnected curves.
  Span<SplinePtr> splines_b = b->splines();

  std::cout << "Wargh" << std::endl << std::flush;

  int test = 0;

  bool has_next_handle = false;
  blender::float3 next_handle;

  for (const SplinePtr &_spline_a : splines_a)
  {
    std::cout << types[(int)_spline_a->type()] << std::endl << std::flush;
    if (_spline_a->type() != Spline::Type::Bezier)
      continue;

    BezierSpline *spline_a = (BezierSpline *)_spline_a.get();

    for (const SplinePtr &_spline_b : splines_b)
    {
      std::cout << types[(int)_spline_b->type()] << std::endl << std::flush;
      if (_spline_b->type() != Spline::Type::Bezier)
        continue;

      BezierSpline *spline_b = (BezierSpline *)_spline_b.get();

      /// FIRST, FIND ALL INTERSECTIONS.
      std::list<blender::float2> path = bezierIntersectAll(spline_a->evaluated_positions(),spline_b->evaluated_positions());


      if(path.size() == 0 ) // No Intersections or uneven intersections (TODO : Reenable uneven intersections)
      {
        if(type == BoolDomainType::AND)
        {
          // If one curve is fully inside the other, use the smaller one. Otherwise return empty curve.
          std::cout << curve_contains(spline_a,spline_b->positions()[0]) << curve_contains(spline_b,spline_a->positions()[0]) << std::endl <<  std::flush;
          if(spline_a->size() > 0 && spline_b->size() > 0)
          {
            if(curve_contains(spline_a,spline_b->positions()[0]))
            {
              std::cout << "B in A" << std::endl <<  std::flush;
              auto c = *(BezierSpline*)(spline_b->copy().get());
              result_spline = std::make_unique<BezierSpline>(c);
            }
            else if(curve_contains(spline_b,spline_a->positions()[0]))
            {
              std::cout << "A in B" << std::endl <<  std::flush;
              auto c = *(BezierSpline*)(spline_a->copy().get());
              result_spline = std::make_unique<BezierSpline>(c);
            }
          }
        }
        else if (type == BoolDomainType::OR) // TODO : If one inside the other, use bigger one. Otherwise return both as separate Curves inside the same CurveEval
        {
          if(spline_a->size() > 0 && spline_b->size() > 0)
          {
            if(curve_contains(spline_a,spline_b->positions()[0]))
            {
              std::cout << "B in A" << std::endl <<  std::flush;
              auto c = *(BezierSpline*)(spline_a->copy().get());
              result_spline = std::make_unique<BezierSpline>(c);
            }
            else if(curve_contains(spline_b,spline_a->positions()[0]))
            {
              std::cout << "A in B" << std::endl <<  std::flush;

              auto c = *(BezierSpline*)(spline_b->copy().get());
              result_spline = std::make_unique<BezierSpline>(c);
            }
          }
        }
        break;
      }

      std::cout << "Intersections are : " << std::endl <<  std::flush;
      print_a(path);

      // Look at first intersection. If AND, follow outwards, if OR, follow inwards. If OR, ignore all control points that lie inside B ( anything up to the first intersection point )
      bool contains_start = curve_contains(spline_b,spline_a->positions()[0]);

      if( (contains_start && (type == BoolDomainType::OR)) || (!contains_start && (type == BoolDomainType::AND)) )
      {
        std::cout << "SWAP " << std::endl <<  std::flush;
        std::swap(spline_a, spline_b);
        for (blender::float2 &intersection : path)
        {
          std::swap(intersection[0], intersection[1]);
        }
      }

      // result_spline->add_point( spline_a->positions()[0 ],
      //                           BezierSpline::HandleType::Free,
      //                           spline_a->handle_positions_left()[0 ],
      //                           BezierSpline::HandleType::Free,
      //                           spline_a->handle_positions_right()[0 ],
      //                           1.0f,
      //                           0.0f);



      bool is_cyclic_a = spline_a->is_cyclic();
      int control_point_count_a = spline_a->positions().size();
      int segment_count_a = control_point_count_a + (is_cyclic_a ? 0 : -1);

      auto intersection = path.front();

      float a_control_point_v = intersection[0] * segment_count_a;
      int a_segment = (int)a_control_point_v;
      int _next_segment_a = mod_i(a_segment + 1, segment_count_a);
      float intersection_offset_a = fmod(a_control_point_v,1.0f);

      int start_a = a_segment;
      float start_a_offset = intersection_offset_a;
      int last_a_segment = start_a;


      blender::float2 last_intersection_offset = blender::float2(0,0); // offset along the curve from the last control point to the next. Used to translate between result and A/B coordinate

      print("+++++++++++++++++++++++++++++");


      bool first = true;
      for (const blender::float2 &intersection : path)
      {
        last_a_segment = process_intersection(first,intersection,result_spline.get(),spline_a,spline_b,last_intersection_offset,last_a_segment);
        first = false;

        std::swap(spline_a, spline_b);
        std::swap(last_intersection_offset[0], last_intersection_offset[1]);
      }



      //start_a = mod_i(start_a + 1, segment_count_a);
      copy_segments(start_a, last_a_segment, result_spline.get(), spline_a, segment_count_a, control_point_count_a);
      //start_a = mod_i(start_a + 1, segment_count_a);
      BezierSpline::InsertResult insert_a = spline_a->calculate_segment_insertion(start_a ,  mod_i(start_a  + 1, spline_a->positions().size()), intersection_offset_a);

      result_spline->handle_positions_right()[result_spline->size()-1] = insert_a.handle_prev;
      result_spline->handle_positions_left()[0] = insert_a.left_handle;
      std::cout << "EXITED WITH " << last_a_segment << std::endl << std::flush;

      result_spline->set_cyclic(spline_a->is_cyclic());

    }
  }
a:


  result_spline->set_resolution(resolution);
  result_spline->attributes.reallocate(result_spline->size());
  result->add_spline(std::move(result_spline));
  result->attributes.reallocate(result->splines().size());

  return result;
}

static std::unique_ptr<CurveEval> create_bezier_segment_curve(
    const float3 start,
    const float3 start_handle_right,
    const float3 end,
    const float3 end_handle_left,
    const int resolution,
    const GeometryNodeCurvePrimitiveBezierSegmentMode mode)
{
  std::unique_ptr<CurveEval> curve = std::make_unique<CurveEval>();
  std::unique_ptr<BezierSpline> spline = std::make_unique<BezierSpline>();

  if (mode == GEO_NODE_CURVE_PRIMITIVE_BEZIER_SEGMENT_POSITION)
  {
    spline->add_point(start,
                      BezierSpline::HandleType::Align,
                      2.0f * start - start_handle_right,
                      BezierSpline::HandleType::Align,
                      start_handle_right,
                      1.0f,
                      0.0f);
    spline->add_point(end,
                      BezierSpline::HandleType::Align,
                      end_handle_left,
                      BezierSpline::HandleType::Align,
                      2.0f * end - end_handle_left,
                      1.0f,
                      0.0f);
  }
  else
  {
    spline->add_point(start,
                      BezierSpline::HandleType::Align,
                      start - start_handle_right,
                      BezierSpline::HandleType::Align,
                      start + start_handle_right,
                      1.0f,
                      0.0f);
    spline->add_point(end,
                      BezierSpline::HandleType::Align,
                      end + end_handle_left,
                      BezierSpline::HandleType::Align,
                      end - end_handle_left,
                      1.0f,
                      0.0f);
  }

  spline->set_resolution(resolution);
  spline->attributes.reallocate(spline->size());
  curve->add_spline(std::move(spline));
  curve->attributes.reallocate(curve->splines().size());
  return curve;
}

static void geo_node_curve_bool_exec(GeoNodeExecParams params)
{
  GeometrySet curve_set_a = params.extract_input<GeometrySet>("Curve A");
  curve_set_a = bke::geometry_set_realize_instances(curve_set_a);

  GeometrySet curve_set_b = params.extract_input<GeometrySet>("Curve B");
  curve_set_b = bke::geometry_set_realize_instances(curve_set_b);

  if (!curve_set_a.has_curve() || !curve_set_b.has_curve())
  {
    params.set_output("Curve", curve_set_a);
    return;
  }

  // std::unique_ptr<CurveEval> curve = create_bezier_segment_curve(
  //   float3(0,0,0),
  //   float3(1,0,0),
  //   float3(3,1,0),
  //   float3(-1,0,0),
  //   16,
  //   GEO_NODE_CURVE_PRIMITIVE_BEZIER_SEGMENT_OFFSET // are handles defined relative to their
  //   point, or absolutely in local space?
  // );
  const bNode &node = params.node();
  const BoolDomainType data_type = static_cast<BoolDomainType>(node.custom2);

  std::unique_ptr<CurveEval> curve = intersect(curve_set_a.get_curve_for_read(),
                                               curve_set_b.get_curve_for_read(),data_type);  //,curve.get()
  params.set_output("Curve", GeometrySet::create_with_curve(curve.release()));

  // return;

  // const CurveEval &curve = *curve_set.get_curve_for_read();
  // float length = 0.0f;
  // for (const SplinePtr &spline : curve.splines()) {
  //   length += spline->length();
  // }
  // params.set_output("Length", length);
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

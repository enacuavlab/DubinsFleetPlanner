// Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
// 
// This file is part of DubinsFleetPlanner.
// 
// DubinsFleetPlanner is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// DubinsFleetPlanner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

#include "ConflictDetection.hpp"
#include "Primitives.hpp"

// ---------- Misc util functions ---------- //

[[gnu::const]]
inline bool angle_in_arc(double start_angle, double stop_angle, double candidate)
{
    if (start_angle > stop_angle)
    {
        double tmp = start_angle;
        start_angle = stop_angle;
        stop_angle = tmp;
    }
    
    while(candidate < start_angle)
    {
        candidate += 2*M_PI;
    }

    while(candidate > stop_angle)
    {
        candidate -= 2*M_PI;
    }

    return (start_angle <= candidate) && (candidate <= stop_angle);
}

[[gnu::const]]
inline double put_in_arc(double start_angle, double stop_angle, double candidate)
{
    if (start_angle > stop_angle)
    {
        double tmp = start_angle;
        start_angle = stop_angle;
        stop_angle = tmp;
    }
    
    while(candidate < start_angle)
    {
        candidate += 2*M_PI;
    }

    while(candidate > stop_angle)
    {
        candidate -= 2*M_PI;
    }

    if ((start_angle <= candidate) && (candidate <= stop_angle))
    {
        return candidate;
    }
    else
    {
        return NAN;
    }
}

[[gnu::const]]
inline Eigen::Vector2d angle_vector(double angle)
{
    return Eigen::Vector2d(std::cos(angle),std::sin(angle));
}

// -------------------- 3D Euclidean distance -------------------- //

/**
 * @brief Given two shapes descritions and a duration, compute the *geometric* 3D euclidean distance between the two
 * 
 * @tparam m1 Type of first shape   (a STRAIGHT line; or a turn, LEFT or RIGHT)
 * @tparam m2 Type of second shape  (a STRAIGHT line; or a turn, LEFT or RIGHT)
 * @param s1 First shape description
 * @param s2 Second shape description
 * @param duration Duration of execution, to draw the shapes
 * @return double Minimal geometric distance between the shapes
 */
template<DubinsMove m1, DubinsMove m2>
double geometric_distance(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

template<> [[gnu::pure]]
double geometric_distance(const PathShape<STRAIGHT> &s1, const PathShape<STRAIGHT> &s2, double duration)
{
    Eigen::Vector3d v1(s1.p1,s1.p2,s1.p3);
    Eigen::Vector3d v2(s2.p1,s2.p2,s2.p3);
    Eigen::Vector3d delta(s2.x-s1.x, s2.y-s1.y, s2.z-s1.z);

    double v1_n2 = v1.squaredNorm();
    double v2_n2 = v2.squaredNorm();

    Eigen::Vector3d p1_s(s1.x,s1.y,s1.z);
    Eigen::Vector3d p2_s(s2.x,s2.y,s2.z);

    Eigen::Vector3d p1_e = p1_s + duration*v1;
    Eigen::Vector3d p2_e = p2_s + duration*v2;


    // Function to check if a parameter is in the allowed range
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration);};


    // -- Line-line optimum
    // This matrix is symmetric positive semidefinite (one can show it is equivalent to a distance function)
    Eigen::Matrix2d obj_matrix;
    obj_matrix(0,0) =  v1.dot(v1);
    obj_matrix(0,1) = -v1.dot(v2);
    obj_matrix(1,0) = obj_matrix(0,1);
    obj_matrix(1,1) =  v2.dot(v2);

    Eigen::Vector2d obj_vector(delta.dot(v1), delta.dot(v2));

    Eigen::Vector2d closest_params = obj_matrix.ldlt().solve(obj_vector);

    // Since it is the global optimum, if it is valid, it is the best
    if (valid_param(closest_params.x()) && valid_param(closest_params.y()))
    {
        return ((p1_s + closest_params.x()*v1) - (p2_s + closest_params.y()*v2)).norm();
    }

    // -- Endpoints distances

    double min_dist = std::min({
        (p1_s-p2_s).norm(),
        (p1_e-p2_e).norm(),
        (p1_s-p2_e).norm(),
        (p1_e-p2_s).norm(),
    });

    // -- Projected distances

    double t1_s = (p1_s-p2_s).dot(v2)/v2_n2;
    if (valid_param(t1_s))
    {
        min_dist = std::min(min_dist,(p1_s-(t1_s*v2 + p2_s)).norm());
    }

    double t1_e = (p1_e-p2_s).dot(v2)/v2_n2;
    if (valid_param(t1_e))
    {
        min_dist = std::min(min_dist,(p1_e-(t1_e*v2 + p2_s)).norm());
    }

    double t2_s = (p2_s-p1_s).dot(v1)/v1_n2;
    if (valid_param(t2_s))
    {
        min_dist = std::min(min_dist,(p2_s-(t2_s*v1 + p1_s)).norm());
    }

    double t2_e = (p2_e-p1_s).dot(v1)/v1_n2;
    if (valid_param(t2_e))
    {
        min_dist = std::min(min_dist,(p2_e-(t2_e*v1 + p1_s)).norm());
    }

    return min_dist;
}

// -------------------- 2D+1D Euclidean distance -------------------- //
// We split computation between the Z (vertical) distance and the XY plane

// ----- XY distances ----- //

template<> [[gnu::pure]]
double geometric_XY_dist(const PathShape<STRAIGHT> &s1, const PathShape<STRAIGHT> &s2, double duration)
{
    Eigen::Vector2d v1(s1.p1,s1.p2);
    Eigen::Vector2d v2(s2.p1,s2.p2);
    Eigen::Vector2d delta(s2.x-s1.x, s2.y-s1.y);

    double v1_n2 = v1.squaredNorm();
    double v2_n2 = v2.squaredNorm();

    Eigen::Vector2d p1_s(s1.x,s1.y);
    Eigen::Vector2d p2_s(s2.x,s2.y);

    Eigen::Vector2d p1_e = p1_s + duration*v1;
    Eigen::Vector2d p2_e = p2_s + duration*v2;


    // Function to check if a parameter is in the allowed range
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration);};


    // -- Line-line optimum
    // This matrix is symmetric positive semidefinite (one can show it is equivalent to a distance function)
    Eigen::Matrix2d obj_matrix;
    obj_matrix(0,0) =  v1.dot(v1);
    obj_matrix(0,1) = -v1.dot(v2);
    obj_matrix(1,0) = obj_matrix(0,1);
    obj_matrix(1,1) =  v2.dot(v2);

    Eigen::Vector2d obj_vector(delta.dot(v1), delta.dot(v2));

    Eigen::Vector2d closest_params = obj_matrix.ldlt().solve(obj_vector);

    // Since it is the global optimum, if it is valid, it is the best
    if (valid_param(closest_params.x()) && valid_param(closest_params.y()))
    {
        return ((p1_s + closest_params.x()*v1) - (p2_s + closest_params.y()*v2)).norm();
    }

    // -- Endpoints distances

    double min_dist = std::min({
        (p1_s-p2_s).norm(),
        (p1_e-p2_e).norm(),
        (p1_s-p2_e).norm(),
        (p1_e-p2_s).norm(),
    });

    // -- Projected distances

    double t1_s = (p1_s-p2_s).dot(v2)/v2_n2;
    if (valid_param(t1_s))
    {
        min_dist = std::min(min_dist,(p1_s-(t1_s*v2 + p2_s)).norm());
    }

    double t1_e = (p1_e-p2_s).dot(v2)/v2_n2;
    if (valid_param(t1_e))
    {
        min_dist = std::min(min_dist,(p1_e-(t1_e*v2 + p2_s)).norm());
    }

    double t2_s = (p2_s-p1_s).dot(v1)/v1_n2;
    if (valid_param(t2_s))
    {
        min_dist = std::min(min_dist,(p2_s-(t2_s*v1 + p1_s)).norm());
    }

    double t2_e = (p2_e-p1_s).dot(v1)/v1_n2;
    if (valid_param(t2_e))
    {
        min_dist = std::min(min_dist,(p2_e-(t2_e*v1 + p1_s)).norm());
    }

    return min_dist;
}

template<DubinsMove m1, DubinsMove m2> [[gnu::pure]]
double geometric_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{
    static_assert(!(m1==STRAIGHT && m2==STRAIGHT));

    if (m2==STRAIGHT)
    {
        return geometric_XY_dist(s2,s1,duration);
    }

    // Function to check if a parameter is in the allowed range
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration);};
    double min_dist = INFINITY;

    if (m1==STRAIGHT)
    {
        // ---- Line-Circ case ---- //

        Eigen::Vector2d p1_s(s1.x,s1.y);
        Eigen::Vector2d v1(s1.p1,s1.p2);
        double v1_n2 = v1.squaredNorm();
        Eigen::Vector2d p1_e = p1_s + duration*v1;
        
        Eigen::Vector2d c2(s2.x,s2.y);

        double r        = s2.p1;
        double omega    = s2.p2;
        double phi      = s2.p4;
        double end_phi  = phi + omega*duration;

        
        // -- Line-Circle intersection
        // We solve te problem in the circle referential (shift by -other.center)
        
        // Line represented as ax + by + c = 0
        double a = -v1[1];
        double b = v1[0];
        double c = -(p1_s-c2).dot(Eigen::Vector2d(a,b));

        double ab2   = a*a + b*b;
        double discr = ab2*r*r - c*c;

        if (discr > 0)
        {
            double sqrt_discr = std::sqrt(discr);
                
            double x1 = -(a*c + sqrt_discr*b)/ab2;
            double y1 = -(b*c - sqrt_discr*a)/ab2;

            Eigen::Vector2d pc1(x1,y1); 
            
            double x2 = -(a*c - sqrt_discr*b)/ab2;
            double y2 = -(b*c + sqrt_discr*a)/ab2;

            Eigen::Vector2d pc2(x2,y2); 

            double phi1 = std::atan2(y1,x1);
            if (angle_in_arc(phi,end_phi,phi1))
            {
                double t1 = (pc1 + c2 - p1_s).dot(v1)/v1_n2;
                if (valid_param(t1))
                {
                    return 0.;
                }
            }

            double phi2 = std::atan2(y2,x2);
            if (angle_in_arc(phi,end_phi,phi2))
            {
                double t2 = (pc2 + c2 - p1_s).dot(v1)/v1_n2;
                if (valid_param(t2))
                {
                    return 0.;
                }
            }
        }


        // -- Project circle center on line
        double t_line = (c2-p1_s).dot(v1)/v1_n2;
        if (valid_param(t_line))
        {
            Eigen::Vector2d c2_proj = p1_s+t_line*v1 -c2;
            double phi_line = std::atan2(c2_proj.y(),c2_proj.x());
            if (angle_in_arc(phi,end_phi,phi_line))
            {
                min_dist = std::min(min_dist,std::abs(r-c2_proj.norm()));
            }
        }

        // -- Project arc endpoints on line

        Eigen::Vector2d c2_s = c2 + r*angle_vector(phi);
        Eigen::Vector2d c2_e = c2 + r*angle_vector(end_phi);

        double t2_s = (c2_s - p1_s).dot(v1)/v1_n2;
        if (valid_param(t2_s))
        {
            min_dist = std::min(min_dist,(c2_s-(t2_s*v1 + p1_s)).norm());
        }

        double t2_e = (c2_e - p1_s).dot(v1)/v1_n2;
        if (valid_param(t2_e))
        {
            min_dist = std::min(min_dist,(c2_e-(t2_e*v1 + p1_s)).norm());
        }
        
        // -- Project line endpoints on circle

        Eigen::Vector2d p_phi_s = p1_s - c2;
        double phi_s = std::atan2(p_phi_s.y(), p_phi_s.x());

        if (angle_in_arc(phi,end_phi,phi_s))
        {
            min_dist = std::min(min_dist,(p_phi_s-r*angle_vector(phi_s)).norm());
        }


        Eigen::Vector2d p_phi_e = p1_e - c2;
        double phi_e = std::atan2(p_phi_e.y(), p_phi_e.x());

        if (angle_in_arc(phi,end_phi,phi_e))
        {
            min_dist = std::min(min_dist,(p_phi_e-r*angle_vector(phi_e)).norm());
        }

        // -- Endpoints distances

        min_dist = std::min({min_dist,
            (p1_s-c2_s).norm(),
            (p1_e-c2_e).norm(),
            (p1_s-c2_e).norm(),
            (p1_e-c2_s).norm(),
        });

        return min_dist;
    }
    else
    {
        // ---- Circ-Circ case ---- //

        Eigen::Vector2d c1(s1.x,s1.y);

        double r1        = s1.p1;
        double omega1    = s1.p2;
        double phi1      = s1.p4;
        double end_phi1  = phi1 + omega1*duration;


        Eigen::Vector2d c2(s2.x,s2.y);

        double r2        = s2.p1;
        double omega2    = s2.p2;
        double phi2      = s2.p4;
        double end_phi2  = phi2 + omega2*duration;

        Eigen::Vector2d dC = c2 - c1;
        double dist2 = dC.squaredNorm();
            

        // -- Circle intersection check
        double delta_r2 = r1*r1 - r2*r2;
        double sum_r2 = r1*r1 + r2*r2;
            
        double discr = -delta_r2*delta_r2 - dist2*dist2 + 2*sum_r2*dist2;
            
        if (discr >= 0)
        {            
            double sqrt_discr = std::sqrt(discr);
            double x_2 = dC.x();
            double y_2 = dC.y();
            
            double sol1x = (x_2*x_2*x_2 + x_2*y_2*y_2 + delta_r2*x_2 - sqrt_discr*y_2)/(2*dist2);
            double sol1y = (y_2*y_2*y_2 + x_2*x_2*y_2 + delta_r2*y_2 + sqrt_discr*x_2)/(2*dist2);

            // Floating-point troubles...
            // assert((x_2*x_2*x_2 + x_2*y_2*y_2 + delta_r2*x_2 - sqrt_discr*y_2)/(2*dist2) == (1/2)*(x_2*x_2*x_2 + x_2*y_2*y_2 + delta_r2*x_2 - sqrt_discr*y_2)/(dist2));
                
            double t1 = std::atan2(sol1y,sol1x);
            double t2 = std::atan2(sol1y-y_2,sol1x-x_2);
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                return 0;
            }
            
            double sol2x = (x_2*x_2*x_2 + x_2*y_2*y_2 + delta_r2*x_2 + sqrt_discr*y_2)/(2*dist2);
            double sol2y = (y_2*y_2*y_2 + x_2*x_2*y_2 + delta_r2*y_2 - sqrt_discr*x_2)/(2*dist2);
            
            t1 = std::atan2(sol2y,sol2x);
            t2 = std::atan2(sol2y-y_2,sol2x-x_2);
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                return 0;
            }
        }
    
        // -- No intersection case
        
        double d = sqrt(dist2);
        
        // Separated circles 
        
        if (d > r1 + r2)
        {
            double t1 = std::atan2(dC.y(),dC.x());
            double t2 = t1 + M_PI;
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                min_dist = std::min(min_dist, d - r1 - r2);
            }
        }
        // Inscribed circles
        
        /// s1 in s2
        if (d + r1 < r2)
        {
            double t1 = std::atan2(dC.y(),dC.x());
            double t2 = t1 + M_PI;
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                min_dist = std::min(min_dist, r2 - d - r1);
            }
        }

        /// s2 in s1
        if (d + r2 < r1)
        {
            double t1 = std::atan2(dC.y(),dC.x());
            double t2 = t1 + M_PI;
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                min_dist = std::min(min_dist, r1 - d - r2);
            }
        }

        // -- Point to circle distances

        Eigen::Vector2d p1_s = c1 + r1*angle_vector(phi1);
        Eigen::Vector2d p1_e = c1 + r1*angle_vector(end_phi1);
        Eigen::Vector2d p2_s = c2 + r2*angle_vector(phi2);
        Eigen::Vector2d p2_e = c2 + r2*angle_vector(end_phi2);

        Eigen::Vector2d p1_s_in_2 = p1_s - c2;
        double phi1_s = std::atan2(p1_s_in_2.y(),p1_s_in_2.x());
        if (angle_in_arc(phi2,end_phi2,phi1_s))
        {
            min_dist = std::min(min_dist,std::abs(r2 - (p1_s - c2).norm()));
        }

        Eigen::Vector2d p1_e_in_2 = p1_e - c2;
        double phi1_e = std::atan2(p1_e_in_2.y(),p1_e_in_2.x());
        if (angle_in_arc(phi2,end_phi2,phi1_e))
        {
            min_dist = std::min(min_dist,std::abs(r2 - (p1_e - c2).norm()));
        }



        Eigen::Vector2d p2_s_in_1 = p2_s - c1;
        double phi2_s = std::atan2(p2_s_in_1.y(),p2_s_in_1.x());
        if (angle_in_arc(phi1,end_phi1,phi2_s))
        {
            min_dist = std::min(min_dist,std::abs(r1 - (p2_s - c1).norm()));
        }

        Eigen::Vector2d p2_e_in_1 = p2_e - c1;
        double phi2_e = std::atan2(p2_e_in_1.y(),p2_e_in_1.x());
        if (angle_in_arc(phi1,end_phi1,phi2_e))
        {
            min_dist = std::min(min_dist,std::abs(r1 - (p2_e - c1).norm()));
        }

        // -- Point to point distances

        min_dist = std::min({min_dist,
            (p1_s-p2_s).norm(),
            (p1_e-p2_e).norm(),
            (p1_s-p2_e).norm(),
            (p1_e-p2_s).norm(),
        });

        return min_dist;
    }
}

// Explicitely declare all possible specializations
template double geometric_XY_dist<STRAIGHT,STRAIGHT>(const PathShape<STRAIGHT>& , const PathShape<STRAIGHT>&, double);
template double geometric_XY_dist<STRAIGHT,LEFT>    (const PathShape<STRAIGHT>& , const PathShape<LEFT>&    , double);
template double geometric_XY_dist<STRAIGHT,RIGHT>   (const PathShape<STRAIGHT>& , const PathShape<RIGHT>&   , double);
template double geometric_XY_dist<LEFT,STRAIGHT>    (const PathShape<LEFT>&     , const PathShape<STRAIGHT>&, double);
template double geometric_XY_dist<RIGHT,STRAIGHT>   (const PathShape<RIGHT>&    , const PathShape<STRAIGHT>&, double);
template double geometric_XY_dist<LEFT,LEFT>        (const PathShape<LEFT>&     , const PathShape<LEFT>&    , double);
template double geometric_XY_dist<RIGHT,LEFT>       (const PathShape<RIGHT>&    , const PathShape<LEFT>&    , double);
template double geometric_XY_dist<LEFT,RIGHT>       (const PathShape<LEFT>&     , const PathShape<RIGHT>&   , double);
template double geometric_XY_dist<RIGHT,RIGHT>      (const PathShape<RIGHT>&    , const PathShape<RIGHT>&   , double);

// ----- Z distances ----- //

template<DubinsMove m1, DubinsMove m2> [[gnu::pure]]
double geometric_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{

    double z1_s = s1.z;
    double z2_s = s2.z;

    double v1 = s1.p3;
    double v2 = s2.p3;

    double z1_e = z1_s + v1*duration;
    double z2_e = z2_s + v2*duration;

    double dz = z2_s - z1_s;

    // Function to check if a parameter is in the allowed range
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration);};

    // -- Line-line optimum
    // This matrix is symmetric positive semidefinite (one can show it is equivalent to a distance function)
    Eigen::Matrix2d obj_matrix;
    obj_matrix(0,0) =  v1*v1;
    obj_matrix(0,1) = -v1*v2;
    obj_matrix(1,0) = obj_matrix(0,1);
    obj_matrix(1,1) =  v2*v2;

    Eigen::Vector2d obj_vector(dz*v1, dz*v2);

    Eigen::Vector2d closest_params = obj_matrix.ldlt().solve(obj_vector);

    // Since it is the global optimum, if it is valid, it is the best
    if (valid_param(closest_params.x()) && valid_param(closest_params.y()))
    {
        return std::abs((z1_s + closest_params.x()*v1) - (z2_s + closest_params.y()*v2));
    }

    // -- Endpoints distances

    double min_dist = std::min({
        std::abs(z1_s-z2_s),
        std::abs(z1_e-z2_e),
        std::abs(z1_s-z2_e),
        std::abs(z1_e-z2_s),
    });

    // -- Projected distances

    double t1_s = (z1_s-z2_s)*v2;
    if (valid_param(t1_s))
    {
        min_dist = std::min(min_dist,std::abs(z1_s-(t1_s*v2 + z2_s)));
    }

    double t1_e = (z1_e-z2_s)*v2;
    if (valid_param(t1_e))
    {
        min_dist = std::min(min_dist,std::abs(z1_e-(t1_e*v2 + z2_s)));
    }

    double t2_s = (z2_s-z1_s)*v1;
    if (valid_param(t2_s))
    {
        min_dist = std::min(min_dist,std::abs(z2_s-(t2_s*v1 + z1_s)));
    }

    double t2_e = (z2_e-z1_s)*v1;
    if (valid_param(t2_e))
    {
        min_dist = std::min(min_dist,std::abs(z2_e-(t2_e*v1 + z1_s)));
    }

    return min_dist;
}

// -------------------- 3D Euclidean distance -------------------- //

template<DubinsMove m1, DubinsMove m2>
std::pair<double,double> temporal_3D_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);


template<bool use_derivatives>
std::pair<double,double> temporal_XY_dist(const PathShape<STRAIGHT> &s1, const PathShape<STRAIGHT> &s2, double duration)
{
    Eigen::Vector2d dp(s1.x - s2.x, s1.y - s2.y);
    Eigen::Vector2d v1(s1.p1, s1.p2);
    Eigen::Vector2d v2(s2.p1, s2.p2);

    Eigen::Vector2d dv = v1 - v2;
    double sc = dv.dot(dp);
    double nv2 = dv.squaredNorm();

    double min_loc = 0.;
    double min_dist = dp.norm();
    
    double dist = (dp + duration*dv).norm();
    if (dist < min_dist)
    {
        min_dist = dist;
        min_loc  = duration;
    }

    if (nv2 > DubinsFleetPlanner_PRECISION)
    {
        double tmin = -sc/nv2;
        if ((0 <= tmin) && (tmin <= duration))
        {
            dist = std::sqrt(dp.squaredNorm()-sc*sc/nv2);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_loc  = tmin;
            }
        }
    }

    return {min_loc,min_dist};
}

template<DubinsMove m, bool use_derivatives> requires (m != STRAIGHT)
std::pair<double,double> temporal_XY_dist(const PathShape<STRAIGHT> &s1, const PathShape<m> &s2, double duration)
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    LineCircle problem(s1,s2);

    auto fbind = [problem](double x){return problem.f(x);};
    auto Fbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F(I);};

    double min_loc;

    if (use_derivatives)
    {
        auto f_dbind = [problem](double x){return problem.f_d(x);};
        auto F_dbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F_d(I);};
        auto f_ddbind = [problem](double x){return problem.f_dd(x);};
        auto F_ddbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F_dd(I);};

        min_loc = IntervalSolver::minimize_interval_method<double,Policies>(
            0.,duration,
            fbind,Fbind,
            f_dbind,F_dbind,
            f_ddbind,F_ddbind,
            DubinsFleetPlanner_PRECISION, 0.
        );
    }
    else
    {
        min_loc = IntervalSolver::minimize_interval_method<double,Policies>(
            0.,duration,
            fbind,Fbind,
            DubinsFleetPlanner_PRECISION, 0.
        );
    }

    double min_val = std::sqrt(problem.f(min_loc));

    return {min_loc,min_val};
}

template<DubinsMove m, bool use_derivatives> requires (m != STRAIGHT)
std::pair<double,double> temporal_XY_dist(const PathShape<m> &s1, const PathShape<STRAIGHT> &s2, double duration)
{
    return temporal_XY_dist<m,use_derivatives>(s2,s1,duration);
}


template<DubinsMove m1, DubinsMove m2, bool use_derivatives> requires ((m1!=STRAIGHT) && (m2!=STRAIGHT))
std::pair<double,double> temporal_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{
    typedef OptimizationRounding::fast_rounding<double> Policies;

    CircleCircle problem(s1,s2);

    auto fbind = [problem](double x){return problem.f(x);};
    auto Fbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F(I);};
    double min_loc;

    if (use_derivatives)
    {
        auto f_dbind = [problem](double x){return problem.f_d(x);};
        auto F_dbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F_d(I);};
        auto f_ddbind = [problem](double x){return problem.f_dd(x);};
        auto F_ddbind = [problem](const IntervalSolver::interval<double,Policies> &I){return problem.F_dd(I);};

        min_loc = IntervalSolver::minimize_interval_method<double,Policies>(
            0.,duration,
            fbind,Fbind,
            f_dbind,F_dbind,
            f_ddbind,F_ddbind,
            DubinsFleetPlanner_PRECISION, 0.
        );
    }
    else
    {
        min_loc = IntervalSolver::minimize_interval_method<double,Policies>(
            0.,duration,
            fbind,Fbind,
            DubinsFleetPlanner_PRECISION, 0.
        );
    }


    double min_val = std::sqrt(problem.f(min_loc));

    return {min_loc,min_val};
}

// Explicitely declare all possible specializations
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,STRAIGHT,false>(const PathShape<STRAIGHT>& s1, const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<false>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,STRAIGHT,true> (const PathShape<STRAIGHT>& s1, const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<true>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,LEFT,true>     (const PathShape<STRAIGHT>& s1, const PathShape<LEFT>& s2   , double d)
{
    return temporal_XY_dist<LEFT,true>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,LEFT,false>    (const PathShape<STRAIGHT>& s1, const PathShape<LEFT>& s2   , double d)
{
    return temporal_XY_dist<LEFT,false>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,RIGHT,true>    (const PathShape<STRAIGHT>& s1, const PathShape<RIGHT>& s2  , double d)
{
    return temporal_XY_dist<RIGHT,true>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<STRAIGHT,RIGHT,false>   (const PathShape<STRAIGHT>& s1, const PathShape<RIGHT>& s2  , double d)
{
    return temporal_XY_dist<RIGHT,false>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<LEFT,STRAIGHT,true>     (const PathShape<LEFT>& s1    , const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<LEFT,true>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<LEFT,STRAIGHT,false>    (const PathShape<LEFT>& s1    , const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<LEFT,false>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<RIGHT,STRAIGHT,true>    (const PathShape<RIGHT>& s1   , const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<RIGHT,true>(s1,s2,d);
}
template<> std::pair<double,double> temporal_XY_dist<RIGHT,STRAIGHT,false>   (const PathShape<RIGHT>& s1   , const PathShape<STRAIGHT>& s2, double d)
{
    return temporal_XY_dist<RIGHT,false>(s1,s2,d);
}


template std::pair<double,double> temporal_XY_dist<LEFT,LEFT,true>         (const PathShape<LEFT>&     , const PathShape<LEFT>&    , double);
template std::pair<double,double> temporal_XY_dist<LEFT,LEFT,false>        (const PathShape<LEFT>&     , const PathShape<LEFT>&    , double);
template std::pair<double,double> temporal_XY_dist<RIGHT,LEFT,true>        (const PathShape<RIGHT>&    , const PathShape<LEFT>&    , double);
template std::pair<double,double> temporal_XY_dist<RIGHT,LEFT,false>       (const PathShape<RIGHT>&    , const PathShape<LEFT>&    , double);
template std::pair<double,double> temporal_XY_dist<LEFT,RIGHT,true>        (const PathShape<LEFT>&     , const PathShape<RIGHT>&   , double);
template std::pair<double,double> temporal_XY_dist<LEFT,RIGHT,false>       (const PathShape<LEFT>&     , const PathShape<RIGHT>&   , double);
template std::pair<double,double> temporal_XY_dist<RIGHT,RIGHT,true>       (const PathShape<RIGHT>&    , const PathShape<RIGHT>&   , double);
template std::pair<double,double> temporal_XY_dist<RIGHT,RIGHT,false>      (const PathShape<RIGHT>&    , const PathShape<RIGHT>&   , double);


template<DubinsMove m1, DubinsMove m2>
std::pair<double,double> temporal_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{
    double dz = s1.z - s2.z;
    double v1 = s1.p3;
    double v2 = s2.p3;
    double dv = v1 - v2;
    double dv2 = dv*dv;

    double min_loc = 0.;
    double min_dist = std::abs(dz);
    
    if (std::abs(dz + duration*dv) < min_dist)
    {
        min_dist    = std::abs(dz + duration*dv);
        min_loc     = duration;
    }

    if (abs(dv) > DubinsFleetPlanner_PRECISION)
    {
        double tmin = (v1*v2)/dv2;
        if ((0 <= tmin) && (tmin <= duration))
        {
            double dist = std::sqrt(dz*dz-v1*v1*v2*v2/dv2); 
            if (dist < min_dist)
            {
                min_dist = dist;
                min_loc  = tmin;
            }
        }
    }

    return {min_loc,min_dist};

}

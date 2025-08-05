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
inline constexpr Eigen::Vector2d angle_vector(double angle)
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

    double t1_s = (p1_s-p2_s).dot(v2);
    if (valid_param(t1_s))
    {
        min_dist = std::min(min_dist,(p1_s-(t1_s*v2 + p2_s)).norm());
    }

    double t1_e = (p1_e-p2_s).dot(v2);
    if (valid_param(t1_e))
    {
        min_dist = std::min(min_dist,(p1_e-(t1_e*v2 + p2_s)).norm());
    }

    double t2_s = (p2_s-p1_s).dot(v1);
    if (valid_param(t2_s))
    {
        min_dist = std::min(min_dist,(p2_s-(t2_s*v1 + p1_s)).norm());
    }

    double t2_e = (p2_e-p1_s).dot(v1);
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

    double t1_s = (p1_s-p2_s).dot(v2);
    if (valid_param(t1_s))
    {
        min_dist = std::min(min_dist,(p1_s-(t1_s*v2 + p2_s)).norm());
    }

    double t1_e = (p1_e-p2_s).dot(v2);
    if (valid_param(t1_e))
    {
        min_dist = std::min(min_dist,(p1_e-(t1_e*v2 + p2_s)).norm());
    }

    double t2_s = (p2_s-p1_s).dot(v1);
    if (valid_param(t2_s))
    {
        min_dist = std::min(min_dist,(p2_s-(t2_s*v1 + p1_s)).norm());
    }

    double t2_e = (p2_e-p1_s).dot(v1);
    if (valid_param(t2_e))
    {
        min_dist = std::min(min_dist,(p2_e-(t2_e*v1 + p1_s)).norm());
    }

    return min_dist;
}

template<DubinsMove m1, DubinsMove m2> [[gnu::pure]]
double geometric_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration)
{
    static_assert(!(std::is_same_v<m1,STRAIGHT> && std::is_same_v<m2,STRAIGHT>));

    if (std::is_same_v<m2,STRAIGHT>)
    {
        return geometric_XY_dist(s2,s1,double);
    }

    // Function to check if a parameter is in the allowed range
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration);};
    double min_dist = INFINITY;

    if (std::is_same_v<m1,STRAIGHT>)
    {
        // ---- Line-Circ case ---- //

        Eigen::Vector2d p1_s(s1.x,s1.y);
        Eigen::Vector2d v1(s1.p1,s1.p2);
        Eigen::Vector2d p1_e = p1_s + duration*v1;
        
        Eigen::Vector2d c2(s2.x,s2.y);

        double r        = s2.p1;
        double omega    = s2.p2;
        double phi      = s2.p4;
        double end_phi  = phi + omega*duration;

        
        // -- Line-Circle intersection
        // We solve te problem in the circle referential (shift by -other.center)
        
        // Line represented as ax + by + c = 0
        double a = -self.direction[1];
        double b = self.direction[0];
        double c = -(self.point-other.center).dot(np.asarray([a,b]));

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
                double t1 = (pc1 + c2 - p1_s).dot(v1);
                if (valid_param(t1))
                {
                    return 0.;
                }
            }

            double phi2 = std::atan2(y2,x2);
            if (angle_in_arc(phi,end_phi,phi2))
            {
                double t2 = (pc2 + c2 - p1_s).dot(v1);
                if (valid_param(t2))
                {
                    return 0.;
                }
            }
        }


        // -- Project circle center on line
        double t_line = (c2-p1_s).dot(v1);
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

        double t2_s = (c2_s - p1_s).dot(v1);
        if (valid_param(t2_s))
        {
            min_dist = std::min(min_dist,(c2_s-(t2_s*v1 + p1_s)).norm());
        }

        double t2_e = (c2_e - p1_s).dot(v1);
        if (valid_param(t2_e))
        {
            min_dist = std::min(min_dist,(c2_e-(t2_e*v1 + p1_s)).norm());
        }
        
        // -- Project line endpoints on circle

        Eigen::Vector2d p_phi_s = p1_s - c2;
        double phi_s = std::atan2(p_phi_s.y(), p_phi_s.x());

        if (angle_in_arc(phi,end_phi,phi_s))
        {
            min_dist = std::min(min_dist,(p_phi_s-angle_vector(phi_s)).norm());
        }


        Eigen::Vector2d p_phi_e = p1_e - c2;
        double phi_e = std::atan2(p_phi_e.y(), p_phi_e.x());

        if (angle_in_arc(phi,end_phi,phi_e))
        {
            min_dist = std::min(min_dist,(p_phi_e-angle_vector(phi_e)).norm());
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
            
        double discr = -delta_r2*delta_r2 - d2*d2 + 2*sum_r2*d2;
            
        if (discr >= 0)
        {            
            double sqrt_discr = std::sqrt(discr);
            double x_2 = dC.x();
            double y_2 = dC.y();
            
            double sol1x = 1/2*(x_2**3 + x_2*y_2**2 + delta_r2*x_2 - sqrt_discr*y_2)/d2;
            double sol1y = 1/2*(y_2**3 + x_2**2*y_2 + delta_r2*y_2 + sqrt_discr*x_2)/d2;
                
            double t1 = std::atan2(sol1y,sol1x);
            double t2 = std::atan2(sol1y-y_2,sol1x-x_2);
            if (angle_in_arc(phi1,end_phi1,t1) && angle_in_arc(phi2,end_phi2,t2))
            {
                return 0;
            }
            
            double sol2x = 1/2*(x_2**3 + x_2*y_2**2 + delta_r2*x_2 + sqrt_discr*y_2)/d2;
            double sol2y = 1/2*(y_2**3 + x_2**2*y_2 + delta_r2*y_2 - sqrt_discr*x_2)/d2;
            
            t1 = arctan2(sol2y,sol2x);
            t2 = arctan2(sol2y-y_2,sol2x-x_2);
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
            min_dist = std::min(min_dist, d - r1 - r2);
        }
        // Inscribed circles
        
        /// s1 in s2
        if (d + r1 < r2)
        {
            min_dist = std::min(min_dist, r2 - d - r1);
        }

        /// s2 in s1
        if (d + r2 < r1)
        {
            min_dist = std::min(min_dist, r1 - d - r2);
        }

        // -- Point to circle distances

        Eigen::Vector2d p1_s = c1 + r1*angle_vector(phi1);
        Eigen::Vector2d p1_e = c1 + r1*angle_vector(end_phi1);
        Eigen::Vector2d p2_s = c2 + r2*angle_vector(phi2);
        Eigen::Vector2d p2_e = c2 + r2*angle_vector(end_phi2);

        Eigen::Vector2d p1_s_in_2 = p1_s - c2;
        double phi1_s = std::atan2(p1_s_in_2.y(),p1_s_in_2.x());
        if (angle_in_arc(phi1,end_phi1,phi1_s))
        {
            min_dist = std::min(min_dist,std::abs(r2 - (p1_s - c2).norm()));
        }

        Eigen::Vector2d p1_e_in_2 = p1_e - c2;
        double phi1_e = std::atan2(p1_e_in_2.y(),p1_e_in_2.x());
        if (angle_in_arc(phi1,end_phi1,phi1_e))
        {
            min_dist = std::min(min_dist,std::abs(r2 - (p1_e - c2).norm()));
        }



        Eigen::Vector2d p2_s_in_1 = p2_s - c1;
        double phi2_s = std::atan2(p2_s_in_1.y(),p2_s_in_1.x());
        if (angle_in_arc(phi2,end_phi2,phi2_s))
        {
            min_dist = std::min(min_dist,std::abs(r1 - (p2_s - c1).norm()));
        }

        Eigen::Vector2d p2_e_in_1 = p2_e - c1;
        double phi2_e = std::atan2(p2_e_in_1.y(),p2_e_in_1.x());
        if (angle_in_arc(phi2,end_phi2,phi2_e))
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
    auto valid_param = [=](double t) -> bool {return (0. <= t) && (t <= duration)};

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



template<DubinsMove m1, DubinsMove m2>
double temporal_3D_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

template<DubinsMove m1, DubinsMove m2>
double temporal_XY_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);

template<DubinsMove m1, DubinsMove m2>
double temporal_Z_dist(const PathShape<m1> &s1, const PathShape<m2> &s2, double duration);
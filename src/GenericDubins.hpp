#pragma once

#include <format>

#include "Dubins.hpp"
#include "BaseDubins.hpp"


class GenericDubins : public Dubins
/**
 * @brief A class representing a dynamically loaded path made of Dubins primitives (straight and circle arcs)
 * 
 */
{
private:
    std::vector<DynamicPathShape> sections;
    std::vector<double> junctions_locs;
    std::vector<Pose3D> junctions;

    void recompute()
    {
        compute_length();
        junctions_locs.clear();
        junctions.clear();

        double curr_len = 0;
        Pose3D p = start;

        uint N = sections.size();

        for(uint i = 0; i < N-1; i++)
        {
            const DynamicPathShape &s = sections[i];

            curr_len += s.length;
            junctions_locs.push_back(curr_len);
            if (s.m == STRAIGHT)
            {
                p = follow_dubins<STRAIGHT>(p,s.length,1.,s.p3/path_planar_speed(s),s.p1);
            }
            else if (s.m == LEFT)
            {
                p = follow_dubins<LEFT>(p,s.length,1.,s.p3/path_planar_speed(s),s.p1);
            }
            else if (s.m == RIGHT)
            {
                p = follow_dubins<RIGHT>(p,s.length,1.,s.p3/path_planar_speed(s),s.p1);
            }
            else
            {
                throw std::runtime_error("Unknown DubinsMove");
            }

            junctions.push_back(p);
        }
    }

    double _compute_length()
    {
        length = 0.;
        for(const DynamicPathShape &s: sections)
        {
            length += s.length;
        }
        return length;
    }

public:
    GenericDubins(double _climb, double _turn_radius, const Pose3D& _start, const Pose3D& _end,std::vector<DynamicPathShape> _sections)
    : Dubins(_climb, _turn_radius, _start, _end),sections(_sections) {recompute();}

    virtual const std::string get_type_abbr([[maybe_unused]] bool b) const
    {
        std::string output = "";
        for(const DynamicPathShape &s : sections)
        {
            output += get_DubinsMove_name(s.m)[0];
        }
        return output;
    }

    virtual const std::vector<DubinsMove> get_all_sections() const
    {
        std::vector<DubinsMove> output;
        for(const DynamicPathShape &s : sections)
        {
            output.push_back(s.m);
        }
        return output;
    }

    size_t type_hash() const
    {
        size_t seed = 0;
        std::vector<DubinsMove> sections = get_all_sections();
        for(DubinsMove m : sections)
        {
            boost::hash_combine<short>(seed,m);
        }

        return seed;
    }

    Pose3D get_position(double len) const
    {
        uint i = 0;
        uint N = sections.size();

        while(i < N-1 && len > junctions_locs[i])
        {
            i++;
        }

        Pose3D p;
        if (i == 0)
        {
            p = start;
        }
        else
        {
            p = junctions[i-1];
            len -= junctions_locs[i-1];
        } 

        DynamicPathShape s = sections[i];
        if (s.m == STRAIGHT)
        {
            return follow_dubins<STRAIGHT>(p,len,1.,s.p3/path_planar_speed(s),s.p1);
        }
        else if (s.m == LEFT)
        {
            return follow_dubins<LEFT>(p,len,1.,s.p3/path_planar_speed(s),s.p1);
        }
        else if (s.m == RIGHT)
        {
            return follow_dubins<RIGHT>(p,len,1.,s.p3/path_planar_speed(s),s.p1);
        }
        else
        {
            throw std::runtime_error("Unknown DubinsMove");
        }
    }

    std::vector<double> get_junction_locs() const 
    {
        std::vector<double> output(junctions_locs);
        return output;
    }

    std::vector<Pose3D> get_junction_points() const
    {
        std::vector<Pose3D> output(junctions);
        return output;
    }

    DubinsMove get_section_type(double len) const
    {
        uint i = 0;
        uint N = sections.size();

        while(i < N-1 && len > junctions_locs[i])
        {
            i++;
        }

        Pose3D p;
        if (i == 0)
        {
            p = start;
        }
        else
        {
            p = junctions[i-1];
            len -= junctions_locs[i-1];
        } 

        DynamicPathShape s = sections[i];
        return s.m;
    }
};
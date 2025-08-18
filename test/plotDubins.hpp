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


#pragma once

#include <sciplot/sciplot.hpp>

#include <tuple>
#include <valarray>
#include <map>

#include "utils.hpp"
#include "Primitives.hpp"
#include "Dubins.hpp"
#include "BaseDubins.hpp"


// Colormap from https://tsitsul.in/blog/coloropt/
// xgfs_normal12 = [(235, 172, 35), (184, 0, 88), (0, 140, 249), (0, 110, 0), (0, 187, 173), (209, 99, 230), (178, 69, 2), (255, 146, 135), (89, 84, 214), (0, 198, 248), (135, 133, 0), (0, 167, 108), (189, 189, 189)]
// xgfs_normal12_hex = {#ebac23,#b80058,#008cf9,#006e00,#00bbad,#d163e6,#b24502,#ff9287,#5954d6,#00c6f8,#878500,#00a76c,#bdbdbd}

namespace Visualisation
{

    const std::vector<std::string> colorlist{
        "#EBAC23",
        "#B80058",
        "#008CF9",
        "#006E00",
        "#00BBAD",
        "#D163E6",
        "#B24502",
        "#FF9287",
        "#5954d6",
        "#00c6f8",
        "#878500",
        "#00a76c",
        "#bdbdbd"
    };


    using namespace sciplot;

    DrawSpecs& plot_pose(Plot2D& plot, const Pose3D& pose)
    {
        return plot.drawWithVecs("vectors",
            std::valarray<double>{pose.x},
            std::valarray<double>{pose.y},
            std::valarray<double>{std::cos(pose.theta)},
            std::valarray<double>{std::sin(pose.theta)});
    }

    template<unsigned samples>
    DrawSpecs& plot_path(Plot2D& plot, Dubins& path)
    {
        static_assert(samples > 1);
        
        std::valarray<double> xs(samples),ys(samples);
        double len = path.get_length();
        auto lens = linspace(0,len,samples);
        std::vector<double> lens_vec{std::begin(lens),std::end(lens)};
        std::vector<Pose3D> poses = path.get_positions(lens_vec,true);

        for(unsigned i = 0; i < samples; i++)
        {
            xs[i] = poses[i].x;
            ys[i] = poses[i].y;
        }

        return plot.drawCurve(xs,ys);
    }

    template<unsigned samples, DubinsMove m>
    DrawSpecs& plot_path(Plot2D& plot, const PathShape<m>& path, double duration)
    {
        static_assert(samples > 1);

        std::valarray<double> xs(samples),ys(samples);

        for(unsigned i = 0; i < samples; i++)
        {
            Pose3D pose = follow_dubins(path,i*duration/(samples-1));
            xs[i] = pose.x;
            ys[i] = pose.y;
        }

        return plot.drawCurve(xs,ys);
    }

    DrawSpecs& plot_junctions(Plot2D& plot, Dubins& path)
    {
        std::vector<Pose3D> locs = path.get_junction_points();
        size_t num = locs.size();
        std::valarray<double> xs(num), ys(num);
        for(unsigned i = 0; i < num; i++)
        {
            xs[i] = locs[i].x;
            ys[i] = locs[i].y;
        }

        return plot.drawPoints(xs,ys);
    }

    Plot2D init_plot()
    {
        // Create a Plot object
        Plot2D plot;

        // Set the x and y labels
        plot.xlabel("x");
        plot.ylabel("y");

        // // Set x and y ranges
        // plot.xrange(xmin,xmax); 
        // plot.yrange("","*"); // Autorange
        plot.gnuplot("set size ratio -1"); // Equal aspect ratio

        // Set the legend to be on the bottom along the horizontal
        plot.legend()
            .atOutsideBottom()
            .displayHorizontal();

        return plot;
    }

}
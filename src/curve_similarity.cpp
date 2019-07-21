#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

int main() {
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::linestring<point_type> linestring_type;

    linestring_type ls1, ls2;
    boost::geometry::read_wkt("LINESTRING(0 0,1 1,2 4,3 9,4 16)", ls1);
    boost::geometry::read_wkt("LINESTRING(0 1,1 2,2 5,3 10,4 17)", ls2);

    double res = boost::geometry::discrete_frechet_distance(ls1, ls2);

    std::cout << "Discrete Frechet Distance: " << res << std::endl;

    return 0;
}

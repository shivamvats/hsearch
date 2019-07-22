#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::linestring<point_type> linestring_type;

linestring_type getParabola( const double a, const double b, const int n ){
    linestring_type ls;
    for( int i=0; i<n; i++ ){
        ls.push_back( point_type( i, a*i*i + b ) );
    }
    return ls;
}

int main() {

    //linestring_type ls1, ls2;
    //boost::geometry::read_wkt("LINESTRING(0 0,1 1,2 4,3 9,4 16)", ls1);
    //boost::geometry::read_wkt("LINESTRING(0 1,1 2,2 5,3 10,4 17)", ls2);
    auto ls1 = getParabola( 1, 0, 100 );
    auto ls2 = getParabola( 1, 2, 100 );

    double res = boost::geometry::discrete_frechet_distance(ls1, ls2);

    std::cout << "Discrete Frechet Distance: " << res << std::endl;

    return 0;
}

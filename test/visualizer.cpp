#include <hsearch/types.h>
#include <hsearch/viz/visualizer.h>

using namespace std;
using namespace hsearch;

int main(){
    Visualizer viz( 100, 100 );
    Color color = {255, 255, 255};
    std::vector<std::pair<int, int>> points;
    for( int i=0; i<50; i++ ){
        points.push_back( std::make_pair(100-i, 100-i) );
    }
    viz.drawLine( std::make_pair(0, 0), std::make_pair(50, 50), 5, color );
    viz.drawCurve( points, 2, color );
    viz.imshow();
}

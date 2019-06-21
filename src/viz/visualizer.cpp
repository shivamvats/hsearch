#include <hsearch/viz/visualizer.h>

namespace hsearch {

    Visualizer::Visualizer( int width, int height ){
        m_map_physical = cv::Mat::zeros( width, height, CV_8UC3 );
        m_map_visual = cv::Mat::zeros( width, height, CV_8UC3 );
    }

    void Visualizer::imshow( std::string window_name, int delay ){
        cv::imshow( window_name, m_map_visual );
        cv::waitKey( delay );
    }

    bool Visualizer::markPoint( int x, int y, int size, const Color& color ){

    }
}

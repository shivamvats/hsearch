#include <hsearch/viz/visualizer.h>

namespace hsearch {

    Visualizer::Visualizer( int width, int height ){
        m_map_physical = cv::Mat::zeros( width, height, CV_8UC3 );
        m_map_visual = cv::Mat::zeros( width, height, CV_8UC3 );
    }

    Visualizer::Visualizer( cv::Mat img_ ){
        m_map_physical = img_;
        m_map_visual = img_;
    }

    void Visualizer::imshow( std::string window_name, int delay ){
        cv::imshow( window_name, m_map_visual );
        cv::waitKey( delay );
    }

    bool Visualizer::markPoint( int x, int y, int size, const Color& color ){
        cv::circle( m_map_visual, cv::Point(x, y), size,
                    cv::Scalar(color[0], color[1], color[2]), cv::FILLED );
        return true;
    }

    bool Visualizer::drawLine( std::pair<int, int> start, std::pair<int, int> end, int thickness, const Color& color ){
        cv::line( m_map_visual, cv::Point(start.first, start.second),
                  cv::Point(end.first, end.second), cv::Scalar(color[0],
                  color[1], color[2]), thickness, cv::LINE_8 );
        return true;
    }

    bool Visualizer::drawCurve( std::vector<std::pair<int, int>> points, int size, const Color color ){
        for( auto& point: points ){
            markPoint( point.first, point.second, size, color );
        }
        return true;
    }
}

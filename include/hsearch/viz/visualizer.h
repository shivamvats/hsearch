#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <hsearch/types.h>

namespace hsearch {

    /** Uses OpenCV as its backend.
     */
    class Visualizer {
        public:
        Visualizer( int width, int height );
        void imshow( std::string window_name="map", int delay=0 );
        bool markPoint( int x, int y, int size=1, const Color& color=m_default_color );
        bool drawLine( std::pair<int, int> start, std::pair<int, int> end, int thickness, const Color& color=m_default_color );
        bool drawCurve( std::vector<std::pair<int, int>> points, int size=1, const Color color=m_default_color );

        public:
        cv::Mat m_map_physical;
        cv::Mat m_map_visual;

        static constexpr Color m_default_color={0, 0, 0};
    };

} //namespace hsearch

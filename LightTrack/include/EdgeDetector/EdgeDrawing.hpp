/**
 * @brief: interface of EdgeDrawing
 * @author: hongxinliu <github.com/hongxinliu> <hongxinliu.com>
 * @date: Jul. 15, 2018
 */

#ifndef _ED_ED_HPP
#define _ED_ED_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>
#include <memory>


#define GAUSS_SIZE	(5)
#define GAUSS_SIGMA	(1.0)
#define SOBEL_ORDER	(1)
#define SOBEL_SIZE	(3)

/**
* @brief: direction of edge
* @brief: if |Gx|>|Gy|, it is a proposal of vertical edge(EDGE_VER), or it is horizontal edge(EDGE_HOR)
*/
enum EDGE_DIR
{
	EDGE_HOR,
	EDGE_VER
};

/**
* @brief: a sign for recording status of each pixel in tracing
*/
enum STATUS
{
	STATUS_UNKNOWN = 0,
	STATUS_BACKGROUND = 1,
	STATUS_EDGE = 255
};

/**
* @brief: Trace direction
*/
enum TRACE_DIR
{
	TRACE_LEFT,
	TRACE_RIGHT,
	TRACE_UP,
	TRACE_DOWN
};

class EdgeDrawing
{
public:
	static std::vector<std::list<cv::Point>> detectEdges(const cv::Mat &image,
													     const int proposal_thresh = 36,
													     const int anchor_interval = 4,
													     const int anchor_thresh = 8);

	static std::vector<std::list<cv::Point>> detectEllipseEdges(const cv::Mat &image,
																const int proposal_thresh = 36,
																const int anchor_interval = 4,
																const int anchor_thresh = 8);
private:

	static void getGradient(const cv::Mat &gray,
							cv::Mat &M,
							cv::Mat &O);


	static void getAnchors(const cv::Mat &M,
						   const cv::Mat &O,
						   const int proposal_thresh,
						   const int anchor_interval,
						   const int anchor_thresh,
						   std::vector<cv::Point> &anchors);


	static void traceFromAnchor(const cv::Mat &M,
								const cv::Mat &O,
								const int proposal_thresh,
								const cv::Point &anchor,
								cv::Mat &status,
								std::vector<std::list<cv::Point>> &edges);


	static void trace(const cv::Mat &M,
					  const cv::Mat &O,
					  const int proposal_thresh,
					  cv::Point pt_last,
					  cv::Point pt_cur,
					  TRACE_DIR dir_last,
					  bool emplace_back,
					  cv::Mat &status,
					  std::list<cv::Point> &edge);


	static bool closedTraceFromAnchor(const cv::Mat &M,
									  const cv::Mat &O,
									  const int proposal_thresh,
									  const cv::Point &anchor,
									  cv::Mat &status,
									  std::vector<std::vector<cv::Point>> &edges);



	static bool traced(const cv::Mat &M,
					   const cv::Mat &O,
					   const int proposal_thresh,
					   const TRACE_DIR curDir,
					   const cv::Point curPoint,
					   cv::Point& nextPoint);
};




#endif

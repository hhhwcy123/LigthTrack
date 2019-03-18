#ifndef DRAWER_H
#define DRAWER_H

#include <pangolin/pangolin.h>
#include "GlobalMap.h"


namespace SLAM
{


class Drawer
{
friend class Viewer;

public:
	Drawer() :
		_frameSize(5),
		_frameLineWidth(1),
		_graphLineWidth(0.9),
		_pointSize(3),
		_cameraSize(15),
		_cameraLineWidth(3),
		_lineWidth(1),
		_fontSize(10),
		_globalMap(nullptr),
		_loopCloser(nullptr),
		_viewer(nullptr)
	{}
	~Drawer() {}

	void setGlobalMap(GlobalMap* map) { _globalMap = map; }
	GlobalMap* getGlobalMap() { return _globalMap; }

	void setViewer(Viewer* viewer) { _viewer = viewer; }
	Viewer* getViewer() { return _viewer; }

	void setLoopCloser(LoopClosing* loopCloser) { _loopCloser = loopCloser; }
	LoopClosing* getLoopCloser() { return _loopCloser; }

	void drawMptIds();

	void drawMapPoints();

	void drawFrames();

	void drawKeyFrames();

	void drawLaserPtCloud();

	void drawLoopClosing();

	void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);

	void setCurrentCameraPose(const cv::Mat &Tcw)
	{ 
		std::unique_lock<std::mutex> lockCamera(_cameraMutex);
		_cameraPose = Tcw.clone();
	}

	void getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

	double					_frameSize;
	double					_frameLineWidth;
	double					_graphLineWidth;
	double					_pointSize;
	int						_fontSize;
	double					_cameraSize;
	double					_cameraLineWidth;
	double					_lineWidth;

	cv::Mat					_cameraPose;

	GlobalMap*				_globalMap;
	LoopClosing*			_loopCloser;
	Viewer*					_viewer;

	static std::mutex       _cameraMutex;
};





}
#endif // DRAWER_H

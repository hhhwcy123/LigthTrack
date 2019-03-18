#include "Viewer.h"
#include "System.h"


namespace SLAM
{
bool HandlerPick3D::_isFistP3d = true;

void Viewer::run()
{
	pangolin::CreateWindowAndBind("DotMarker Tracking: Map Viewer", 1024, 768);

	// 3D Mouse handler requires depth testing to be enabled
	glEnable(GL_DEPTH_TEST);

	// Issue specific OpenGl we might need
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
	pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
	pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
	pangolin::Var<bool> menuShowFrames("menu.Show Frames", true, true);
	pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
	pangolin::Var<bool> menuShowLoopClosing("menu.Show Loop Closing", true, true);
	pangolin::Var<bool> menuShowLaserPts("menu.Show Laser", true, true);
	pangolin::Var<bool> menuShowMptIds("menu.Show id", false, true);
	pangolin::Var<int> menuFrameScale("menu.Frame Scale", _drawer->_frameSize,5,100,true);
	pangolin::Var<int> menuMptScale("menu.MapPoint Scale", _drawer->_pointSize, 1, 5, true);
	pangolin::Var<int> menuMptIdScale("menu.Id Scale", _drawer->_fontSize, 1, 20, true);
	//pangolin::Var<bool> menu3DPick("menu.Pick mode", false, true);

	//////////////// Define Camera Render Object (for view / scene browsing)////////////////
	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024, 768, _viewpointF, _viewpointF, 512, 389, 0.1, 10000),
		pangolin::ModelViewLookAt(_viewpointX, _viewpointY, _viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
	);

	////////////////Add named OpenGL viewport to window and provide 3D Handler////////////////

	//HandlerPick3D handlerPick3D(s_cam, &_system->getGlobalMap());
	pangolin::Handler3D hander3D(s_cam);
	setHandler(&hander3D);
	pangolin::View& d_cam = pangolin::CreateDisplay()
		.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
		.SetHandler(&hander3D);
		//.SetHandler(&handlerPick3D);


	pangolin::OpenGlMatrix Twc;
	Twc.SetIdentity();

	bool bFollow = true;

	while (1)
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		_drawer->getCurrentOpenGLCameraMatrix(Twc);


		if (menuFollowCamera && bFollow)
		{
			s_cam.Follow(Twc);
		}
		else if (menuFollowCamera && !bFollow)
		{
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(_viewpointX, _viewpointY, _viewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
			s_cam.Follow(Twc);
			bFollow = true;
		}
		else if (!menuFollowCamera && bFollow)
		{
			bFollow = false;
		}

		d_cam.Activate(s_cam);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		_drawer->drawCurrentCamera(Twc);

		_drawer->_frameSize = (int)menuFrameScale;
		_drawer->_pointSize= (int)menuMptScale;
		_drawer->_fontSize = (int)menuMptIdScale;
		
		/*if (menu3DPick)
		{
			setHandler(&handlerPick3D);
			d_cam.SetHandler(&handlerPick3D);
		}
		else
		{
			setHandler(&hander3D);
			d_cam.SetHandler(&hander3D);
		}*/
		bool bDrawKFs = true;
		if (menuShowFrames)
			_drawer->drawFrames();
		if (menuShowKeyFrames)
			_drawer->drawKeyFrames();
		if (menuShowPoints)
			_drawer->drawMapPoints();
		if (menuShowMptIds)
			_drawer->drawMptIds();
		if (menuShowLaserPts)
			_drawer->drawLaserPtCloud();
		if (menuShowLoopClosing)
			_drawer->drawLoopClosing();
		
		pangolin::FinishFrame();


		if (_system->isOutputStereoImg())
		{
			cv::namedWindow("DotMarker Tracker: Current Frame", cv::WINDOW_NORMAL);
			//if(!_tracker->_canvasImg.empty())
			if (_tracker->_currentFrame)
			{
				if (!_tracker->_currentFrame->_stereoImg.empty())
				{
					cv::Mat stereoImg;
					cv::resize(_tracker->_currentFrame->_stereoImg, stereoImg, cv::Size(1280, 720), 0, 0, CV_INTER_LINEAR);			
					cv::imshow("DotMarker Tracker: Current Frame", stereoImg);
					_tracker->_currentFrame->_stereoImg.release();
					//cv::imshow("DotMarker Tracker: Current Frame", _tracker->_currentFrame->_stereoImg);
					double fps = 100;
					double T = 1000.0 / (double)fps;
					cv::waitKey(T);
				}
			}
		}
			

		if (stop())
		{
			while (isStopped())
			{
				usleep(3000);
			}
		}

		if (checkFinish())
			break;
	}

	setFinish();
	


}


void HandlerPick3D::XYZProjectUV(cv::Mat& projMat, cv::Mat& modelViewMat, cv::Mat& viewMat, const cv::Point3d& p3D, cv::Point2d& p2D)
{
	cv::Mat worldPos = (cv::Mat_<double>(4, 1) << p3D.x, p3D.y, p3D.z,1.0);

	cv::Mat x4D = projMat*(modelViewMat*worldPos);

	const double invzc = 1.0 / x4D.at<double>(3);
	double x = x4D.at<double>(0)*invzc;
	double y = x4D.at<double>(1)*invzc;


	// Map x, y and z to range 0-1
	x = x * 0.5f + 0.5f;
	y = y * 0.5f + 0.5f;


	// Map x,y to viewport
	p2D.x = x * (double)viewMat.at<int>(2) + (double)viewMat.at<int>(0);
	p2D.y = y * (double)viewMat.at<int>(3) + (double)viewMat.at<int>(1);
}




bool HandlerPick3D::pick3DPoint(View& display, int x, int y)
{
	pangolin::Viewport viewPort = display.GetBounds();
	OpenGlMatrix& modelView = cam_state->GetModelViewMatrix();
	OpenGlMatrix& proj = cam_state->GetProjectionMatrix();
	GLint viewport[4] = { display.v.l,display.v.b,display.v.w,display.v.h }; //{ display.vp.l,display.vp.b,display.vp.w,display.vp.h }; //{ viewPort.l,viewPort.b,viewPort.w,viewPort.h };
	//const double Identity4d[] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	//glGetIntegerv(GL_VIEWPORT, viewport);
	cv::Mat projMat = cv::Mat(4, 4, CV_64FC1, proj.m);
	cv::Mat modelViewMat = cv::Mat(4, 4, CV_64FC1, modelView.m);
	cv::Mat viewMat = cv::Mat(1, 4, CV_32SC1, viewport);

	std::unique_lock<std::mutex> lockMpt(GlobalMap::_mptMutex);
	double minDiff = DBL_MAX;
	MapPoint3d* mptBest = nullptr;
	MptList& mptList = _globalMap->getMapPointList();
	for (MptList::iterator itr = mptList.begin(); itr != mptList.end(); itr++)
	{
		cv::Point2d p2d;
		cv::Point3d p3d;
		//XYZProjectUV(projMat, modelViewMat, viewMat, **itr, p2d);
		pangolin::glProject((*itr)->x, (*itr)->y, (*itr)->z, modelView.m, proj.m, viewport, &p3d.x, &p3d.y, &p3d.z);
		cv::Point2d err2D = cv::Point2d(p3d.x - last_pos[0], p3d.y - last_pos[1]);
		double distDiff = cv::norm(err2D);
		if (distDiff<UV_MINDIFF)
		{
			if (distDiff < minDiff)
			{
				minDiff = distDiff;
				mptBest = *itr;
			}
		}
	}
	if (mptBest)
	{
		if (_isFistP3d)
			_mptSelPair.first = mptBest;
		else
		{
			_mptSelPair.second = mptBest;
		}
		_mptSelPairList.emplace_back(_mptSelPair);
		
		
		return true;
	}
	return false;
}



void HandlerPick3D::Mouse(View& display, MouseButton button, int x, int y, bool pressed, int button_state)
{ // mouse down
	last_pos[0] = (float)x;
	last_pos[1] = (float)y;
	GLprecision T_nc[3 * 4];
	LieSetIdentity(T_nc);
	if (pressed)
	{
		if (pick3DPoint(display, x, y))
		{
			_isFistP3d = !_isFistP3d;
		}
		
		GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
		if (ValidWinDepth(p[2]))
		{
			last_z = p[2];
			std::copy(Pc, Pc + 3, rot_center);
		}

		if (button == MouseWheelUp || button == MouseWheelDown)
		{
			LieSetIdentity(T_nc);
			const GLprecision t[3] = { 0,0,(button == MouseWheelUp ? 1 : -1) * 100 * tf };
			LieSetTranslation<>(T_nc, t);
			if (!(button_state & MouseButtonRight) && !(rot_center[0] == 0 && rot_center[1] == 0 && rot_center[2] == 0))
			{
				LieSetTranslation<>(T_nc, rot_center);
				const GLprecision s = (button == MouseWheelUp ? -1.0 : 1.0) * zf;
				MatMul<3, 1>(T_nc + (3 * 3), s);
			}
			OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
			LieMul4x4bySE3<>(spec.m, T_nc, spec.m);
		}
		
		
					
	}
}

}

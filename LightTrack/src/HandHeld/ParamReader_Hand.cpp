#include <Eigen\Dense>
#include "HandHeld\ParamReader_Hand.h"
#include<iostream>
#include<fstream>
#include "HandHeld\GeoBase.h"

using namespace Eigen;
using namespace std;
namespace HandHeld
{
	void transCameraByCam(Camera& c1, const Camera cc)
	{
		Matrix3d R;
		Vector3d T;
		R = c1.pos.R * cc.pos.R.transpose();
		T = c1.pos.T - R*cc.pos.T;
		c1 = Camera(c1.K, c1.J, R, T);
	}
	void readCameraPara(Camera& c1, Camera& c2, string file) {
		ifstream in(file);
		if (!in) {
			cout << "file open fail...  path: " << file << endl;
			return;
		}
		char line[1000];
		in.getline(line, 1000);
		in.getline(line, 1000);
		in.getline(line, 1000);
		int no;
		Matrix3d R;
		Vector3d T, t;
		in >> no
			>> R(0, 0) >> R(0, 1) >> R(0, 2)
			>> R(1, 0) >> R(1, 1) >> R(1, 2)
			>> R(2, 0) >> R(2, 1) >> R(2, 2)
			>> T[0] >> T[1] >> T[2]
			>> t[0] >> t[1] >> t[2];
		c1.pos = Pose(R, T);

		in >> no
			>> R(0, 0) >> R(0, 1) >> R(0, 2)
			>> R(1, 0) >> R(1, 1) >> R(1, 2)
			>> R(2, 0) >> R(2, 1) >> R(2, 2)
			>> T[0] >> T[1] >> T[2]
			>> t[0] >> t[1] >> t[2];
		c2.pos = Pose(R, T);



		in.getline(line, 1000);
		in.getline(line, 1000);
		Matrix3d K = Matrix3d::Identity();
		Matrix<double, 5, 1> J;
		in >> no
			>> K(0, 0) >> K(1, 1) >> K(0, 2) >> K(1, 2)
			>> J(0, 0) >> J(1, 0) >> J(4, 0) >> J(2, 0) >> J(3, 0);
		c1.K = K; c1.J = J; c1.init();

		in >> no
			>> K(0, 0) >> K(1, 1) >> K(0, 2) >> K(1, 2)
			>> J(0, 0) >> J(1, 0) >> J(4, 0) >> J(2, 0) >> J(3, 0);
		c2.K = K; c2.J = J; c2.init();


	}
	void transCamPara(StrCamPos& camPosL, StrCamPos& camPosR,Camera camL,Camera camR) 
	{
		Camera c[2] = { Camera() }, cc;
		cc = camL;
		transCameraByCam(camL, cc);
		transCameraByCam(camR, cc);

		//cout << "dist 1-2: " << (camL.center - camR.center).norm() << endl;

		/*
		ofstream out(filePath + "CamPara.ini");
		if (!out) {
			cout << "file open fail...  file: " << filePath + "CamPara.ini" << endl;
			return;
		}

		for (int i = 0; i < 2; ++i) {
			out << i << endl;
			out << c[i].K(0, 0) << " " << c[i].K(1, 1) << " " << c[i].K(0, 2) << " " << c[i].K(1, 2) << endl;
			out << c[i].J(0, 0) << " " << c[i].J(1, 0) << " " << c[i].J(4, 0) << " " << c[i].J(2, 0) << " " << c[i].J(3, 0) << endl;
			out << c[i].pos.R(0, 0) << " " << c[i].pos.R(0, 1) << " " << c[i].pos.R(0, 2) << " " << c[i].pos.T(0, 0) << endl;
			out << c[i].pos.R(1, 0) << " " << c[i].pos.R(1, 1) << " " << c[i].pos.R(1, 2) << " " << c[i].pos.T(1, 0) << endl;
			out << c[i].pos.R(2, 0) << " " << c[i].pos.R(2, 1) << " " << c[i].pos.R(2, 2) << " " << c[i].pos.T(2, 0) << endl;
		}

		out.close();*/
		Camera2StrCamPos(camL, camPosL);
		Camera2StrCamPos(camR, camPosR);
	}


	void Camera2StrCamPos(Camera c, StrCamPos& cam)
	{
		StrPos pos;
		pos.R[0] = c.pos.R(0, 0);
		pos.R[1] = c.pos.R(0, 1);
		pos.R[2] = c.pos.R(0, 2);
		pos.R[3] = c.pos.R(1, 0);
		pos.R[4] = c.pos.R(1, 1);
		pos.R[5] = c.pos.R(1, 2);
		pos.R[6] = c.pos.R(2, 0);
		pos.R[7] = c.pos.R(2, 1);
		pos.R[8] = c.pos.R(2, 2);

		pos.T[0] = c.pos.T(0, 0);
		pos.T[1] = c.pos.T(1, 0);
		pos.T[2] = c.pos.T(2, 0);


		cam.f[0] = c.K(0, 0);
		cam.f[1] = c.K(1, 1);
		cam.C[0] = c.K(0, 2);
		cam.C[1] = c.K(1, 2);
		cam.k[0] = c.J(0, 0);
		cam.k[1] = c.J(1, 0);
		cam.k[2] = c.J(4, 0);
		cam.p[0] = c.J(2, 0);
		cam.p[1] = c.J(3, 0);
		cam.pos = pos;

	}
}
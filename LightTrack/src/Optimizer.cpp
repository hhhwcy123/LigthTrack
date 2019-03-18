#include "Optimizer.h"
#include "Matcher.h"
#include "Point.h"
#include "impl\Point.hpp"
#include "Camera.h"
#include "Converter.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

namespace SLAM
{
#ifdef STAGE
double Optimizer::MAX_EDGE_RMSE = 0.3;
double Optimizer::MAX_EDGE_ERROR = 0.5;
double Optimizer::MAX_BA_EDGE_ERROR = 0.3;// 0.5;
#else
double Optimizer::MAX_EDGE_RMSE = 0.08;
double Optimizer::MAX_EDGE_ERROR = 0.10;
double Optimizer::MAX_BA_EDGE_ERROR = 0.2;// 0.5;
#endif // STAGE


class Optimizer::VertexSim3ExpmapRGBD : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		VertexSim3ExpmapRGBD() : BaseVertex<7, g2o::Sim3>()
	{
		_marginalized = false;
		_fix_scale = false;
	}
	virtual bool read(std::istream& is) { return true; };
	virtual bool write(std::ostream& os) const { return true; };

	virtual void setToOriginImpl() {
		_estimate = g2o::Sim3();
	}

	virtual void oplusImpl(const double* update_)
	{
		Eigen::Map< Eigen::Matrix <double, 7, 1>> update(const_cast<double*>(update_));

		if (_fix_scale)
			update[6] = 0;

		g2o::Sim3 s(update);
		setEstimate(s*estimate());
	}

	bool _fix_scale;

};



class Optimizer::EdgeSim3ProjectXYZRGBD : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexSim3ExpmapRGBD>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSim3ProjectXYZRGBD() :
		BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexSim3ExpmapRGBD>() {}
	virtual bool read(std::istream& is) { return true; };
	virtual bool write(std::ostream& os) const { return true; };
	void computeError()
	{
		const VertexSim3ExpmapRGBD* v1 = static_cast<const VertexSim3ExpmapRGBD*>(_vertices[1]);
		const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

		Eigen::Vector3d obs(_measurement);
		_error = obs - v1->estimate().map(v2->estimate());
	}

	double error2() const
	{
		return _error.dot(_error);
	}
};



class Optimizer::EdgeInverseSim3ProjectXYZRGBD : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexSim3ExpmapRGBD>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeInverseSim3ProjectXYZRGBD() :
		BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexSim3ExpmapRGBD>() {}

	virtual bool read(std::istream& is) { return true; };
	virtual bool write(std::ostream& os) const { return true; };

	void computeError()
	{
		const VertexSim3ExpmapRGBD* v1 = static_cast<const VertexSim3ExpmapRGBD*>(_vertices[1]);
		const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
		Eigen::Vector3d obs(_measurement);
		_error = obs - v1->estimate().inverse().map(v2->estimate());
	}
	double error2() const
	{
		return _error.dot(_error);
	}
	// virtual void linearizeOplus();

};



class Optimizer::EdgeProjectXYZRGBDOnlyPose : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
	friend class Optimizer;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeProjectXYZRGBDOnlyPose() {}

	EdgeProjectXYZRGBDOnlyPose(const Eigen::Vector3d& x3D) : _x3D(x3D) {}

	double error2() const
	{
		return _error.dot(_error);
	}

	void computeError()
	{
		const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
		_error = _measurement - pose->estimate().map(_x3D);
	}
	Eigen::Vector2d project2d(const Eigen::Vector3d& v)
	{
		Eigen::Vector2d res;
		res(0) = v(0) / v(2);
		res(1) = v(1) / v(2);
		return res;
	}
	Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz)
	{
		Eigen::Vector2d proj = project2d(trans_xyz);
		Eigen::Vector2d res;
		res[0] = proj[0] * fx + cx;
		res[1] = proj[1] * fy + cy;
		return res;
	}
	virtual void linearizeOplus();

	bool read(std::istream& in) { return true; }
	bool write(std::ostream& out) const { return true; }
protected:
	Eigen::Vector3d _x3D;
	double fx, fy, cx, cy;
};



class  Optimizer::EdgeProjectXYZRGBD : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
	friend class Optimizer;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeProjectXYZRGBD() {};

	bool read(std::istream& in) { return true; }
	bool write(std::ostream& out) const { return true; }

	double error2() const
	{
		return _error.dot(_error);
	}

	void computeError()
	{
		const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*> (_vertices[1]);
		const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

		_error = _measurement - v1->estimate().map(v2->estimate());

		//Eigen::Vector2d obs(_measurement);
		//_error = obs - cam_project(v1->estimate().map(v2->estimate()));
	}
	//virtual void linearizeOplus();

};



class Optimizer::EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge < 2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
	friend class Optimizer;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeSE3ProjectXYZOnlyPose() {}

	bool read(std::istream& in) { return true; }
	bool write(std::ostream& out) const { return true; }

	void computeError() {
		const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
		Eigen::Vector2d obs(_measurement);
		_error = obs - cam_project(v1->estimate().map(Xw));
		//_error = obs - v1->estimate().map(Xw);
	}

	bool isDepthPositive() {
		const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
		return (v1->estimate().map(Xw))(2) > 0.0;
	}

	//virtual void linearizeOplus();

	Eigen::Vector2d project2d(const Eigen::Vector3d& v)
	{
		Eigen::Vector2d res;
		res(0) = v(0) / v(2);
		res(1) = v(1) / v(2);
		return res;
	}

	Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz)
	{
		Eigen::Vector2d proj = project2d(trans_xyz);
		Eigen::Vector2d res;
		res[0] = proj[0] * fx + cx;
		res[1] = proj[1] * fy + cy;
		return res;
	}

	Eigen::Vector3d Xw;
	double fx, fy, cx, cy;
};



class Optimizer::EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge < 2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap >
{
	friend class Optimizer;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeSE3ProjectXYZ() {};

	bool read(std::istream& in) { return true; }
	bool write(std::ostream& out) const { return true; }

	void computeError() {
		const g2o::VertexSE3Expmap *v1 = static_cast<const  g2o::VertexSE3Expmap *>(_vertices[1]);
		const g2o::VertexSBAPointXYZ *v2 = static_cast<const  g2o::VertexSBAPointXYZ *>(_vertices[0]);
		Eigen::Vector2d obs(_measurement);
		_error = obs - cam_project(v1->estimate().map(v2->estimate()));
	}

	bool isDepthPositive() {
		const  g2o::VertexSE3Expmap *v1 = static_cast<const  g2o::VertexSE3Expmap *>(_vertices[1]);
		const  g2o::VertexSBAPointXYZ *v2 = static_cast<const  g2o::VertexSBAPointXYZ *>(_vertices[0]);
		return (v1->estimate().map(v2->estimate()))(2) > 0.0;
	}

	//virtual void linearizeOplus();

	//Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz) const;

	Eigen::Vector2d project2d(const Eigen::Vector3d& v)
	{
		Eigen::Vector2d res;
		res(0) = v(0) / v(2);
		res(1) = v(1) / v(2);
		return res;
	}

	Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz)
	{
		Eigen::Vector2d proj = project2d(trans_xyz);
		Eigen::Vector2d res;
		res[0] = proj[0] * fx + cx;
		res[1] = proj[1] * fy + cy;
		return res;
	}

	double fx, fy, cx, cy;
};



void Optimizer::EdgeProjectXYZRGBDOnlyPose::linearizeOplus()
{
	g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
	g2o::SE3Quat T(pose->estimate());
	Eigen::Vector3d xyz_trans = T.map(_x3D);
	double x = xyz_trans[0];
	double y = xyz_trans[1];
	double z = xyz_trans[2];

	_jacobianOplusXi(0, 0) = 0;
	_jacobianOplusXi(0, 1) = -z;
	_jacobianOplusXi(0, 2) = y;
	_jacobianOplusXi(0, 3) = -1;
	_jacobianOplusXi(0, 4) = 0;
	_jacobianOplusXi(0, 5) = 0;

	_jacobianOplusXi(1, 0) = z;
	_jacobianOplusXi(1, 1) = 0;
	_jacobianOplusXi(1, 2) = -x;
	_jacobianOplusXi(1, 3) = 0;
	_jacobianOplusXi(1, 4) = -1;
	_jacobianOplusXi(1, 5) = 0;

	_jacobianOplusXi(2, 0) = -y;
	_jacobianOplusXi(2, 1) = x;
	_jacobianOplusXi(2, 2) = 0;
	_jacobianOplusXi(2, 3) = 0;
	_jacobianOplusXi(2, 4) = 0;
	_jacobianOplusXi(2, 5) = -1;
}



int Optimizer::optimizeRGBDSim3(Frame *pF1, Frame *pF2,
								P3DPairList& matchedP3DList,cv::Mat &T21,
								const int& iterCount,const bool& bFixScale,const bool& bDispErrorStats)
{
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	// Set Sim3 vertex
	VertexSim3ExpmapRGBD * vSim3 = new VertexSim3ExpmapRGBD();
	vSim3->_fix_scale = bFixScale;
	g2o::Sim3 sim21(Converter::toMatrix3d(T21.colRange(0,3).rowRange(0,3)), Converter::toVector3d(T21.col(3).rowRange(0,3)),1);
	vSim3->setEstimate(sim21);//pF1->pF2的转换矩阵
	vSim3->setId(0);
	vSim3->setFixed(false);
	optimizer.addVertex(vSim3);

	const int N = matchedP3DList.size();
	std::vector<EdgeSim3ProjectXYZRGBD*> vpEdges21;
	std::vector<EdgeInverseSim3ProjectXYZRGBD*> vpEdges12;
	vpEdges21.reserve(N);
	vpEdges12.reserve(N);

	int nCorrespondences = 0;
	int validMathcedCount = 0;
	int i = 0;
	for (auto itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++,i++)
	{
		const int id1 = 2 * i + 1;
		const int id2 = 2 * (i + 1);

		g2o::VertexSBAPointXYZ* p3D1Vertex = new g2o::VertexSBAPointXYZ();
		p3D1Vertex->setEstimate(Converter::toVector3d(*itr->second));//pF1的3d点
		p3D1Vertex->setId(id1);
		p3D1Vertex->setFixed(true);
		optimizer.addVertex(p3D1Vertex);

		g2o::VertexSBAPointXYZ* p3D2Vertex = new g2o::VertexSBAPointXYZ();
		p3D2Vertex->setEstimate(Converter::toVector3d(*itr->first));//pF2的3d点
		p3D2Vertex->setId(id2);
		p3D2Vertex->setFixed(true);
		optimizer.addVertex(p3D2Vertex);

		nCorrespondences++;
		///////////////pF2的3d点->pF1//////////////////
		Eigen::Matrix<double, 3, 1> obs1;
		obs1 << itr->second->x, itr->second->y, itr->second->z;

		EdgeInverseSim3ProjectXYZRGBD* e12 = new EdgeInverseSim3ProjectXYZRGBD();
		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		KeyPointPair kptPair1;
		pF1->getKeyPairFromP3D(itr->second, kptPair1);
		double w_sigmaInv1 = 1.0;// kptPair1.first->_weight*kptPair1.second->_weight;// 1.0;
		e12->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv1*w_sigmaInv1);//信息矩阵

		g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
		e12->setRobustKernel(rk1);
		rk1->setDelta(MAX_EDGE_ERROR);
		optimizer.addEdge(e12);

		///////////////pF1的3d点->pF2//////////////////
		Eigen::Matrix<double, 3, 1> obs2;
		obs2 << itr->first->x, itr->first->y, itr->first->z;

		EdgeSim3ProjectXYZRGBD* e21 = new EdgeSim3ProjectXYZRGBD();
		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);

		KeyPointPair kptPair2;
		pF2->getKeyPairFromP3D(itr->first, kptPair2);
		double w_sigmaInv2 = 1.0; //kptPair2.first->_weight*kptPair2.second->_weight;// 1.0; 
		e21->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv2*w_sigmaInv2);//信息矩阵

		g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
		e21->setRobustKernel(rk2);
		rk2->setDelta(MAX_EDGE_ERROR);
		optimizer.addEdge(e21);

		vpEdges12.emplace_back(e12);
		vpEdges21.emplace_back(e21);

		e12->computeError();//计算边的误差_error：相机坐标系的3d点误差
		const double& error12 = sqrt(e12->error2());
		e21->computeError();
		const double& error21 = sqrt(e21->error2());

		if (bDispErrorStats)
		{
			//Eigen::Vector2d& p2DObs = e->cam_project(obs);
			std::cout << "Edge 3D error " << i 
				      << " prev:" << error12 << " "
				      << " next:" << error21 << std::endl;			
		}
	}

	validMathcedCount = nCorrespondences;
	double RMSE = 0;
	double maxError = DBL_MIN;
	double totalError2 = 0.;
	EdgeInverseSim3ProjectXYZRGBD* worstEdge12 = nullptr;
	EdgeSim3ProjectXYZRGBD* worstEdge21 = nullptr;
	while (validMathcedCount > 3)//最少4个匹配对                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
	{
		RMSE = 0;
		maxError = DBL_MIN;
		totalError2 = 0.;
		worstEdge12 = nullptr;
		worstEdge21 = nullptr;

		std::cout << "Sim3 optimizing..." << std::endl;
		vSim3->setEstimate(g2o::Sim3(Converter::toMatrix3d(T21.colRange(0, 3).rowRange(0, 3)), Converter::toVector3d(T21.col(3).rowRange(0, 3)), 1));//每轮优化之前重置相对运动
	
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//开始优化：优化迭代次数为its[it]

		for (int i=0;i<nCorrespondences;i++)
		{
			EdgeInverseSim3ProjectXYZRGBD* e12 = vpEdges12[i];
			EdgeSim3ProjectXYZRGBD* e21 = vpEdges21[i];

			const double& error12 = sqrt(e12->error2());
			const double& error21 = sqrt(e21->error2());
			/*else
			{不还原
				e->setLevel(0);
			}*/
			if ((e12->level() == 0))
			{
				totalError2 += e12->error2();
				totalError2 += e21->error2();
			}
			
			if ((e12->level() == 0 && error12 > maxError))
			{
				maxError = error12;
				worstEdge12 = e12;
				worstEdge21 = e21;
			}
			if ((e21->level() == 0 && error21 > maxError))
			{
				maxError = error21;
				worstEdge12 = e12;
				worstEdge21 = e21;
			}
			if (bDispErrorStats)
			{
				if (e12->level() == 0|| e21->level()==0)
				{
					std::cout << "Edge 3D error " << i 
							  << " prev:" << error12 
							  << " next:" << error21 << std::endl;
				}
			}
			e21->setRobustKernel(0);
			e12->setRobustKernel(0);
		}
		validMathcedCount = optimizer.activeEdges().size()/2;
		//if (validMathcedCount< 3)
		//	break;//直接return出bug(未查明)

		RMSE = sqrt(totalError2 / optimizer.activeEdges().size());
		if (RMSE > MAX_EDGE_RMSE)
		{//如果边的RMSE过大，屏蔽最大误差边，重新优化
			worstEdge12->setLevel(1);//可以尝试设置较小的信息矩阵
			worstEdge21->setLevel(1);
			validMathcedCount--;
			std::cout << "Edges size: " << optimizer.edges().size()
					  << ", Pose Optimizing error: " << RMSE
					  << std::endl
					  << "Try Again!" << std::endl;
		}
		else
			break;
	}

	if (bDispErrorStats)
	{
		for (int i = 0; i<nCorrespondences; i++)
		{
			EdgeInverseSim3ProjectXYZRGBD* e12 = vpEdges12[i];
			EdgeSim3ProjectXYZRGBD* e21 = vpEdges21[i];
			if (e12->level() != 0)
			{
				e12->computeError();//计算未参与优化的边误差_error，参与优化的边误差不需要显式调用计算函数
				std::cout << "Outlier " << i
					      << " prev error: " << sqrt(e12->error2())
						  << " next error: " << sqrt(e21->error2()) 
						  <<std::endl;
			}
		}

		std::cout << "Inliers size:" << validMathcedCount
			      << ", Pose Optimizing error:" << RMSE
			      << std::endl;
	}
	// 恢复位姿
	VertexSim3ExpmapRGBD* vSim3_recov = static_cast<VertexSim3ExpmapRGBD*>(optimizer.vertex(0));
	T21 = Converter::toCvMat(vSim3_recov->estimate());

	return validMathcedCount;//返回内点数量=pFrame帧地图点数量-离群点数量
}




int Optimizer::poseOptimization3D(Frame* pFrame, const int& iterCount, const bool& bDispErrorStats)
{
	const double errorMax = MAX_EDGE_ERROR*MAX_EDGE_ERROR;
	typedef std::vector<EdgeProjectXYZRGBDOnlyPose*> EdgeList;

	g2o::SparseOptimizer optimizer;
	//g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

	//std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
	//g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(*solver_ptr);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);
	//optimizer.setVerbose(true);

	// Set Frame vertex
	g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3); //向优化器添加顶点：pFrame帧的位姿

	MptKeyPairMap matchedMptList = pFrame->_matchedMptMap;
	const int mPtMatchedCount = matchedMptList.size();

	EdgeList vpEdges;
	vpEdges.reserve(mPtMatchedCount);
	MptList mptMatchedList;
	mptMatchedList.reserve(mPtMatchedCount);

	int validMathcedCount = 0;
	int nInitialCorrespondences = 0;
	int nBad = 0;

	if (mPtMatchedCount < 3)
		return -1;
	for (MptKeyPairMap::iterator itr = matchedMptList.begin(); itr != matchedMptList.end(); itr++)
	{//遍历frame的匹配地图点
		MapPoint3d* mapPoint = itr->first;//frame匹配地图点	
		if (mapPoint->isBad())
			continue;//跳过在局部BA子线程确定的错误地图点
		cv::Point3d* p3d = itr->second.first->_matchedP3D;
		if (!p3d)
		{
			std::cout << "Can't get corresponding 3d point from map point in optimization!" << std::endl;
			return 0;
		}

		Eigen::Matrix<double, 3, 1> obs;
		obs << p3d->x, p3d->y, p3d->z;

		nInitialCorrespondences++;

		EdgeProjectXYZRGBDOnlyPose* e = new EdgeProjectXYZRGBDOnlyPose();

		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e->setMeasurement(obs);

		//const double w_sigma = 1.0;
		double w_sigmaInv = itr->second.first->_weight*itr->second.second->_weight;// 1.0; itr->second.first->_weight * itr->second.second->_weight;
		e->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv*w_sigmaInv);//信息矩阵

		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		e->setRobustKernel(rk);
		rk->setDelta(MAX_EDGE_ERROR);//设置鲁棒核函数，限制误差过大边的误差

		//设置地图点世界坐标
		e->_x3D[0] = mapPoint->x;
		e->_x3D[1] = mapPoint->y;
		e->_x3D[2] = mapPoint->z;

		//设置当前帧的相机内参
		e->fx = pFrame->getCamera()->Kl().at<double>(0, 0);
		e->fy = pFrame->getCamera()->Kl().at<double>(1, 1);
		e->cx = pFrame->getCamera()->Kl().at<double>(0, 2);
		e->cy = pFrame->getCamera()->Kl().at<double>(1, 2);



		e->computeError();//计算边的误差_error：相机坐标系的3d点误差
		const double& error2 = e->error2();
		//if (error2> errorMax)
		//{//优化前去除误差较大边
		//	nBad++;
		//	/*e->setLevel(1);		
		//}

		if (bDispErrorStats)
		{
			//Eigen::Vector2d& p2DObs = e->cam_project(obs);
			KeyPoint* keyPointLeft = itr->second.first;
			Eigen::Vector2d p2DObs;
			p2DObs << keyPointLeft->_undistortX, keyPointLeft->_undistortY;
			Eigen::Matrix<double, 3, 1> x3D(mapPoint->x, mapPoint->y, mapPoint->z);
			g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
			Eigen::Vector2d& p2DEstimate = e->cam_project(pose->estimate().map(x3D));
			Eigen::Vector2d err2D = p2DObs - p2DEstimate;
			double errLeft2D = sqrt(err2D.dot(err2D));

			std::cout << "Edge 3D " << mapPoint->getID() << " error:" << sqrt(error2)
				      << " Left 2D error:" << errLeft2D << std::endl;
		}
		optimizer.addEdge(e);//向优化器添加边
		vpEdges.emplace_back(e);
		mptMatchedList.emplace_back(mapPoint);
	}
	validMathcedCount = nInitialCorrespondences - nBad;

	double RMSE = 0;
	double maxError2 = DBL_MIN;
	EdgeProjectXYZRGBDOnlyPose* worstEdge = nullptr;
	MapPoint3d* worstMpt= nullptr;
	while (validMathcedCount > 2)
	{
		RMSE = 0;
		maxError2 = DBL_MIN;
		worstEdge = nullptr;

		std::cout << "Pose optimizing..." << std::endl;
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));//每轮优化之前重置pFrame帧位姿
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//开始优化：优化迭代次数为its[it]

		MptList::iterator mPItr = mptMatchedList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeProjectXYZRGBDOnlyPose* e = *itr;
			MapPoint3d* mapPoint = *mPItr;

			const double& error2 = e->error2();		
			/*else
			{不还原
			e->setLevel(0);
			}*/
			//if (error2 > errorMax || mapPoint->isBad())
			//{//mapPoint可能在子线程局部BA被判断为bad
			//	e->setLevel(1);
			//}
			if ((e->level() == 0 && error2 > maxError2))
			{
				maxError2 = error2;
				worstEdge = e;
				worstMpt = mapPoint;
			}
			if (bDispErrorStats)
			{
				if (e->level() == 0)
				{
					//e->computeError();//计算边的误差_error：重投影误差
					Eigen::Vector2d& p2DObs = e->cam_project(e->_measurement);
					g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
					Eigen::Vector2d& p2DEstimate = e->cam_project(pose->estimate().map(e->_x3D));
					Eigen::Vector2d err2D = p2DObs - p2DEstimate;
					double errLeft2D = sqrt(err2D.dot(err2D));
					std::cout << "Edge 3D " << mapPoint->getID() << " error:" << sqrt(error2)
						      << " Left 2D error:" << errLeft2D << std::endl;
				}
			}
			e->setRobustKernel(0);
		}
		validMathcedCount = optimizer.activeEdges().size();
		//if (validMathcedCount< 3)
		//	break;//直接return出bug(未查明)

		double totalError2 = 0.0;
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
		{
			if ((*itr)->level() == 0)
				totalError2 += (*itr)->error2();
		}
		RMSE = sqrt(totalError2 / validMathcedCount);
		if (RMSE > MAX_EDGE_RMSE)
		{//如果边的RMSE过大，屏蔽最大误差边，重新优化
			worstEdge->setLevel(1);//可以尝试设置较小的信息矩阵
			pFrame->insertOutLier(worstMpt);
			validMathcedCount--;
			std::cout << "Edges size: " << optimizer.edges().size()
					  << ", Pose Optimizing error: " << RMSE
					  << std::endl
					  << "Try Again!" << std::endl;
		}
		else
			break;
	} 

	if (bDispErrorStats)
	{
		MptKeyPairMap::iterator mPItr = matchedMptList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeProjectXYZRGBDOnlyPose* e = *itr;
			if (e->level() != 0)
			{
				e->computeError();//计算未参与优化的边误差_error，参与优化的边误差不需要显式调用计算函数
				std::cout << "Outlier " << mPItr->first->getID()
						  << " error:" << sqrt(e->error2()) << std::endl;
			}
		}
	}
	std::cout << "Inliers size:" << validMathcedCount
			  << ", Pose Optimizing error:" << RMSE
			  << std::endl;
	////////丢弃优化精度不高的帧(一般为错误点导致)////////////////
	//if (RMSE > 0.15)
	//{
	//	validMathcedCount = 0;
	//	//return validMathcedCount;//直接return出bug(未查明)
	//}
	//if (validMathcedCount > 3)
	//{
	//	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//获取优化过的顶点：pFrame帧的位姿
	//	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	//	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	//	pFrame->setPose(pose);//设置pFrame帧的位姿
	//}
	
	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//获取优化过的顶点：pFrame帧的位姿
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->setPose(pose);//设置pFrame帧的位姿

	return validMathcedCount;//返回内点数量=pFrame帧地图点数量-离群点数量
}





int Optimizer::poseOptimization2D(Frame *pFrame, const int& iterCount, const bool& bDispErrorStats)
{
	const double errorMax = MAX_EDGE_ERROR*MAX_EDGE_ERROR;
	typedef std::vector<EdgeSE3ProjectXYZOnlyPose*> EdgeList;

	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);

	// Set Frame vertex
	g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));
	vSE3->setId(0);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3); //向优化器添加顶点：pFrame帧的位姿

	MptKeyPairMap& matchedMptList = pFrame->_matchedMptMap;
	const int mPtMatchedCount = matchedMptList.size();

	EdgeList vpEdges;
	vpEdges.reserve(mPtMatchedCount);
	MptList mptMatchedList;
	mptMatchedList.reserve(mPtMatchedCount);

	int validMathcedCount = 0;
	int nInitialCorrespondences = 0;
	int nBad = 0;

	for (MptKeyPairMap::iterator itr = matchedMptList.begin(); itr != matchedMptList.end(); itr++)
	{//遍历frame的匹配地图点
		MapPoint3d* mapPoint = itr->first;//frame匹配地图点
		if (mapPoint->isBad())
			continue;//跳过在局部BA子线程确定的错误地图点
		KeyPoint* keyPointleft = itr->second.first; //frame匹配地图点所对应的图像特征点

		nInitialCorrespondences++;

		Eigen::Matrix<double, 2, 1> obs;
		obs << keyPointleft->_undistortX, keyPointleft->_undistortY;//设置观测值为pFrame地图点所对应的图像特征点的像素坐标

		EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e->setMeasurement(obs);

		//const double w_sigmaInv = 1.0;
		double w_sigmaInv = itr->second.first->_weight*itr->second.second->_weight;// 1.0; itr->second.first->_weight * itr->second.second->_weight;
		e->setInformation(Eigen::Matrix2d::Identity()*w_sigmaInv*w_sigmaInv);


		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		e->setRobustKernel(rk);
		rk->setDelta(MAX_EDGE_ERROR);//设置鲁棒核函数，限制误差过大边的误差

		//设置地图点世界坐标
		e->Xw[0] = mapPoint->x;
		e->Xw[1] = mapPoint->y;
		e->Xw[2] = mapPoint->z;

		//设置当前帧的相机内参
		e->fx = pFrame->getCamera()->Kl().at<double>(0, 0);
		e->fy = pFrame->getCamera()->Kl().at<double>(1, 1);
		e->cx = pFrame->getCamera()->Kl().at<double>(0, 2);
		e->cy = pFrame->getCamera()->Kl().at<double>(1, 2);


		e->computeError();//计算边的误差_error：相机坐标系的3d点误差
		const double& error2 = e->error().dot(e->error());
		//if (error2> errorMax)
		//{//优化前去除误差较大边
		//	nBad++;
		//	e->setLevel(1);		
		//}
		cv::Point3d* p3d = itr->second.first->_matchedP3D;
		if (!p3d)
		{
			std::cout << "Can't get corresponding 3d point from map point in optimiazation!" << std::endl;
			return 0;
		}

		Eigen::Vector3d p3DObs;
		p3DObs << p3d->x, p3d->y, p3d->z;
		g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
		Eigen::Vector3d& p3DEstimate = pose->estimate().map(e->Xw);
		Eigen::Vector3d err3D = p3DObs - p3DEstimate;
		double errLeft3D = sqrt(err3D.dot(err3D));

		if (bDispErrorStats)
		{
			e->computeError();//计算边的误差_error：重投影误差
			std::cout << "Edge 2D " << mapPoint->getID() << " error:" << sqrt(e->error().dot(e->error()))
				      << " 3D error:" << errLeft3D << std::endl;
		}
		optimizer.addEdge(e);//向优化器添加边
		vpEdges.emplace_back(e);
		mptMatchedList.emplace_back(mapPoint);
	}
	validMathcedCount = nInitialCorrespondences - nBad;

	double RMSE = 0;
	double maxError2 = DBL_MIN;
	EdgeSE3ProjectXYZOnlyPose* worstEdge = nullptr;
	MapPoint3d* worstMpt = nullptr;
	while (validMathcedCount > 2)
	{
		RMSE = 0;
		maxError2 = DBL_MIN;
		worstEdge = nullptr;

		std::cout << "Pose optimizing..." << std::endl;
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));//每轮优化之前重置pFrame帧位姿
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//开始优化：优化迭代次数为its[it]

		MptList::iterator mPItr = mptMatchedList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeSE3ProjectXYZOnlyPose* e = *itr;

			MapPoint3d* mapPoint = *mPItr;

			const double& error2 = e->error().dot(e->error());
			/*else
			{不还原
			e->setLevel(0);
			}*/
			//if (error2 > errorMax || mapPoint->isBad())
			//{//mapPoint可能在子线程局部BA被判断为bad
			//	e->setLevel(1);
			//}
			if ((e->level() == 0 && error2 > maxError2))
			{
				maxError2 = error2;
				worstEdge = e;
				worstMpt = mapPoint;
			}
			if (bDispErrorStats)
			{
				if (e->level() == 0)
				{
					Eigen::Vector3d p3DObs;
					cv::Point3d* p3d = pFrame->getP3DFromMpt(mapPoint);
					p3DObs << p3d->x, p3d->y, p3d->z;
					g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
					Eigen::Vector3d& p3DEstimate = pose->estimate().map(e->Xw);
					Eigen::Vector3d err3D = p3DObs - p3DEstimate;
					double errLeft3D = sqrt(err3D.dot(err3D));
					std::cout << "Edge 2D " << mapPoint->getID() << " error:" << sqrt(error2)
						      << " 3D error:" << errLeft3D << std::endl;
				}
			}
			e->setRobustKernel(0);
		}
		validMathcedCount = optimizer.activeEdges().size();
		//if (validMathcedCount< 3)
		//	break;//直接return出bug(未查明)

		double totalError2 = 0.0;
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
		{
			if ((*itr)->level() == 0)
				totalError2 += (*itr)->error().dot((*itr)->error());
		}
		RMSE = sqrt(totalError2 / validMathcedCount);
		if (RMSE > MAX_EDGE_RMSE)
		{//如果边的RMSE过大，屏蔽最大误差边，重新优化
			worstEdge->setLevel(1);//可以尝试设置较小的信息矩阵
			validMathcedCount--;
			std::cout << "Edges size: " << optimizer.edges().size()
					  << ", Pose Optimizing error: " << RMSE
					  << std::endl
					  << "Try Again!" << std::endl;
		}
		else
			break;
	}

	if (bDispErrorStats)
	{
		MptKeyPairMap::iterator mPItr = matchedMptList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeSE3ProjectXYZOnlyPose* e = *itr;
			if (e->level() != 0)
				e->computeError();//计算未参与优化的边误差_error，参与优化的边误差不需要显式调用计算函数
			std::cout << "Outlier " << mPItr->first->getID()
				      << " error:" << sqrt(e->error().dot(e->error())) << std::endl;
		}		
	}
	std::cout << "Inliers size:" << validMathcedCount
			  << ", Pose Optimizing error:" << RMSE
			  << std::endl;
	////////丢弃优化精度不高的帧(一般为错误点导致)////////////////
	//if (RMSE > 0.15)
	//{
	//	validMathcedCount = 0;
	//	//return validMathcedCount;//直接return出bug(未查明)
	//}

	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//获取优化过的顶点：pFrame帧的位姿
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->setPose(pose);//设置pFrame帧的位姿

	return validMathcedCount;//返回内点数量=pFrame帧地图点数量-离群点数量
}





void Optimizer::localKFBundleAdjustment3D(KeyFrame* pCurKF, GlobalMap* pMap, bool& bAbort)
{

	KeyFrameList localKFList;
	MptList localMptList;

	typedef std::vector<EdgeProjectXYZRGBD*> EdgeList;

	localKFList.emplace_back(pCurKF);//向局部关键帧集合lLocalKeyFrames添加当前关键帧
	pCurKF->_localForKF = pCurKF->getID();

	const KeyFrameList& neighborList = pCurKF->getBestCovisibleKeyFrames(5);
	//const KeyFrameList& neighborList = pCurKF->_orderedConnectedKFs;
	for (KeyFrameList::const_iterator itr = neighborList.begin(); itr != neighborList.end(); itr++)
	{
		localKFList.emplace_back(*itr);//localKFList：局部关键帧集合，与当前帧共点数量最多的前5帧
		(*itr)->_localForKF = pCurKF->getID();
	}

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//上锁，避免主线程删除地图点

	// 局部关键帧观察的所有地图点
	//std::list<MapPoint3d*> localMapPoints;
	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (pMP)
				if (!pMP->isBad())
					if (pMP->_localForKF != pCurKF->getID())
					{
						localMptList.emplace_back(pMP);//localMapPoints：局部地图点集合，局部关键帧观察的所有地图点
						pMP->_localForKF = pCurKF->getID();
					}
		}
	}

	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//锁住地图点，避免主线程更新地图点的观察帧

	std::vector<KeyFrame*> fixedCameras;
	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		KFSet& observationKFs = (*pit)->getObservationKFs();
		for (KFSet::iterator fit = observationKFs.begin(); fit != observationKFs.end(); fit++)
		{
			if ((*fit)->_localForKF != pCurKF->getID() && (*fit)->_fixedForKF != pCurKF->getID())//根据_localForKF标志跳过局部共点关键帧（避免重复添加顶点导致优化器错误）
			{
				(*fit)->_fixedForKF = pCurKF->getID();

				fixedCameras.emplace_back((*fit));//lFixedCameras：观察到局部地图点但不属于局部关键帧的关键帧集合，优化中保持位姿不变
			}
		}
	}	
	// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);
	optimizer.setForceStopFlag(&bAbort);

	// 设置地图点顶点
	unsigned long maxKFid = 0;
	for (KeyFrameList::iterator fit = localKFList.begin(); fit != localKFList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		vSE3->setId((*fit)->getID());
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//添加局部关键帧的位姿作为顶点
		if ((*fit)->getID()>maxKFid)
			maxKFid = (*fit)->getID();
	}

	// Set Fixed Frame vertices
	for (KeyFrameList::iterator cit = fixedCameras.begin(); cit != fixedCameras.end(); cit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*cit)->getPose()));
		vSE3->setId((*cit)->getID());
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);//添加lFixedCameras关键帧位姿作为顶点
		if ((*cit)->getID()>maxKFid)
			maxKFid = (*cit)->getID();
	}

	// Set MapPoint3d vertices
	const int nExpectedSize = (localKFList.size() + fixedCameras.size())*localMptList.size();

	std::vector<EdgeProjectXYZRGBD*> vpEdges;
	vpEdges.reserve(nExpectedSize);

	std::vector<KeyFrame*> vpEdgeKFs;
	vpEdgeKFs.reserve(nExpectedSize);

	std::vector<MapPoint3d*> vpMapPointEdge;
	vpMapPointEdge.reserve(nExpectedSize);

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;

		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		int id = pMP->getID() + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//添加局部地图点pMP作为顶点

		const KFSet& observationsKFs = pMP->getObservationKFs();//主线程会改变观察帧集合，需加锁
		

		//Set edges
		for (KFSet::const_iterator kFit = observationsKFs.begin(); kFit != observationsKFs.end(); kFit++)
		{//遍历局部地图点pMP的观察关键帧
			KeyFrame* pKF = *kFit;//获取局部地图点pMP的观察帧obsKF
			if (pKF->isBad())
				continue;
			cv::Point3d* p3d = pKF->getP3DFromMpt(pMP);
			if (!p3d)
			{
				std::cout << "Can't get corresponding 3d point from map point in optimiazation!" << std::endl;
				return;
			}

			Eigen::Matrix<double, 3, 1> obs;
			obs << p3d->x, p3d->y, p3d->z;

			EdgeProjectXYZRGBD* e = new EdgeProjectXYZRGBD();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//设置边的顶点为局部地图点pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//设置边的顶点为pMP的观察帧obsKF位姿

			e->setMeasurement(obs);//设置边的测量值
			//const double &invSigma2 = 1.0;
			KeyPointPair kptPair;
			if (!pKF->getKeyPairFromMpt(pMP, kptPair))
				return;
			const double wSigmaInv = kptPair.first->_weight* kptPair.second->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix3d::Identity()*wSigmaInv*wSigmaInv);//信息矩阵

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//设置鲁棒核函数，限制误差过大边的误差
			//离散边不参与局部BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//误差过大边不参与优化

			optimizer.addEdge(e);//添加边
			vpEdges.emplace_back(e);
			vpEdgeKFs.emplace_back(pKF);
			vpMapPointEdge.emplace_back(pMP);
		}
	}
	lockMptObs.unlock();
	lockEraseMpt.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//避免主线程poseOptimization3D和子线程localBundleAdjustment3D冲突
	

	//////////////第一轮局部BA///////////////
	if (bAbort)
		return;
	optimizer.initializeOptimization(0);
	optimizer.optimize(5);

	for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	{//	去除离散边
		EdgeProjectXYZRGBD* e = vpEdges[i];
		MapPoint3d* pMP = vpMapPointEdge[i];
			
		if (pMP->isBad())
			continue;
		if(e->level()!=0)
			e->computeError();
			
		if (e->error2() > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			e->setLevel(1);
		else
			e->setLevel(0);
		e->setRobustKernel(0);
	}


	
	/////////////////////重新第二轮局部BA//////////////////////	
	if (bAbort)
		return;
	optimizer.initializeOptimization(0);
	optimizer.optimize(10);
	
	double totalError2 = 0.0;
	for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
	{
		if ((*itr)->level() == 0)
			totalError2 += (*itr)->error2();
	}
	double RMSE = sqrt(totalError2 / optimizer.activeEdges().size());
	//std::unique_lock<std::mutex> lockStd(pMap->_mutex);
	std::cout << "Edge size:" << optimizer.edges().size() 
			  << ", Local optimization square error:" << RMSE 
			  << ", Frame id: " << pCurKF->getID()
			  << std::endl << std::endl;
	//lockStd.unlock();

	std::set<std::pair<KeyFrame*, MapPoint3d*> > vToErase;


	// 去除离散地图点和离散观察帧
	//for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	//{
	//	EdgeProjectXYZRGBD* e = vpEdges[i];
	//	MapPoint3d* pMP = vpMapPointEdge[i];
	//	KeyFrame* obsKF = vpEdgeKFs[i];
	//	if (pMP->isBad())
	//		continue;
	//
	//	if (e->error2()>MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
	//	{
	//		pMP->_outLierCount++;
	//		vToErase.insert(std::make_pair(obsKF, pMP));
	//	}
	//}
	//for (size_t i = 0, iend = vpMapPointEdge.size(); i < iend; i++)
	//{//BA后通过地图点离散观察帧数判断局部地图点是否是坏点
	//	MapPoint3d* pMP = vpMapPointEdge[i];
	//	if (pMP->isBad())
	//		continue;
	//	if ((float)pMP->_outLierCount / (float)pMP->obsKF() >= 0.5)
	//{
	//      pMP->setBad(true);
	//      pMap->insertBadMpt((*g_mItr));
	//}
	//		
	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//冗余帧删除和局部BA在同一线程
	//if (!vToErase.empty())
	//{
	//	for (auto itr= vToErase.begin(); itr!=vToErase.end(); itr++)
	//	{
	//		KeyFrame* obsKF = itr->first;
	//		MapPoint3d* pMPi = itr->second;
	//		obsKF->eraseMptMatchedWithP3D(pMPi);//对应观察关键帧帧删除该匹配地图点，避免下一次参与局部BA
	//		obsKF->getRef()->eraseMptMatchedWithP3D(pMPi); //对应观察帧删除该匹配地图点
	//		//obsKF->eraseMptMatched(pMPi);//对应观察关键帧帧删除该匹配地图点，避免下一次参与局部BA
	//		//obsKF->getRef()->eraseMptMatched(pMPi); //对应观察帧删除该匹配地图点
	//	}
	//}

	///////////////////重新计算局部地图点(迭代优化时主线程会删除地图点)///////////////////


	//更新局部地图点坐标

	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{//修正局部地图点位姿
		MapPoint3d* pMP = *lit;
		if (pMP->isBad())
			continue;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID() + maxKFid + 1));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);		

		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}

	//////////////更新局部关键帧、邻接帧和地图点位姿//////////////
	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());

	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{//修正局部关键帧位姿
		KeyFrame* pKF = *lit;
		////////////////////////////矫正局部关键帧及邻接帧位姿////////////////////////////
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->getID()));
		cv::Mat TcwCorrected = Converter::toCvMat(vSE3->estimate());
		pKF->setPose(TcwCorrected);//更新pKF位姿	
		pKF->updateLocalPoses();//更新pKF邻接帧位姿
	}
	
	//矫正完帧位姿及地图点坐标后检查观察帧是否为离散匹配
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		if (pMP->isBad())
			continue;
		double matchedError = 0.;
		auto& obs =  pMP->getObservations();
		for (auto obsItr = obs.begin(); obsItr != obs.end(); obsItr++)
		{
			matchedError = (*obsItr)->calcMptMatchedError(pMP);
			if (matchedError >= Matcher_<>::MAX_PTBIAS_ERROR)
				(*obsItr)->insertOutLier(pMP);
			else
				(*obsItr)->eraseOutlier(pMP);
		}
	}
	lockTrack.unlock();

	//锁保证：子线程调整完当前关键帧和局部帧位姿后存在两种情况：
	//1、主线程最新帧还未根据前帧估计出位姿，当跟踪完位姿一定是根据子线程中已修改过的前帧位姿为参考
	//2、主线程最新帧已根据前帧原始位姿估计出位姿且加入全局地图，并在子线程BA参与了局部帧的位姿调整
	//3、主线程最新帧已估计位姿且创建为关键帧，随关联帧入队并加入全局地图
	//4、主线程最新帧已估计位姿且创建为关键帧，随关联帧入队并加入全局地图,LocalMapping已建立该关键帧和pKF的共点关系

}




void Optimizer::localKFBundleAdjustment2D(KeyFrame* keyFrame, GlobalMap* pMap, bool& bAbort)
{
	KeyFrameList localKFList;
	MptList localMptList;
	typedef std::vector<EdgeSE3ProjectXYZ*> EdgeList;

	localKFList.emplace_back(keyFrame);//向局部关键帧集合lLocalKeyFrames添加当前关键帧
	keyFrame->_localForKF = keyFrame->getID();

	const KeyFrameList& neighborList = keyFrame->getBestCovisibleKeyFrames(5);
	//const KeyFrameList& neighborList = keyFrame->_orderedConnectedKFs;
	for (KeyFrameList::const_iterator itr = neighborList.begin(); itr != neighborList.end(); itr++)
	{
		localKFList.emplace_back(*itr);//localKFList：局部关键帧集合，与当前帧共点数量最多的前5帧
		(*itr)->_localForKF = keyFrame->getID();
	}

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//上锁，避免主线程删除地图点

	// 局部关键帧观察的所有地图点
	//std::list<MapPoint3d*> localMapPoints;
	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (pMP)
				if (!pMP->isBad())
					if (pMP->_localForKF != keyFrame->getID())
					{
						localMptList.emplace_back(pMP);//localMapPoints：局部地图点集合，局部关键帧观察的所有地图点
						pMP->_localForKF = keyFrame->getID();
					}
		}
	}

	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//锁住地图点，避免主线程更新地图点的观察帧

	std::vector<KeyFrame*> fixedCameras;
	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		KFSet& observations = (*pit)->getObservationKFs();
		for (KFSet::iterator fit = observations.begin(); fit != observations.end(); fit++)
		{
			if ((*fit)->_localForKF != keyFrame->getID() && (*fit)->_fixedForKF != keyFrame->getID())//根据_localForKF标志跳过局部共点关键帧（避免重复添加顶点导致优化器错误）
			{
				(*fit)->_fixedForKF = keyFrame->getID();

				fixedCameras.emplace_back((*fit));//fixedCameras：观察到局部地图点但不属于局部关键帧的关键帧集合，优化中保持位姿不变
			}
		}
	}

	// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);

	optimizer.setForceStopFlag(&bAbort);

	// 设置地图点顶点
	unsigned long maxKFid = 0;
	for (KeyFrameList::iterator fit = localKFList.begin(); fit != localKFList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		vSE3->setId((*fit)->getID());
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//添加局部关键帧的位姿作为顶点
		if ((*fit)->getID()>maxKFid)
			maxKFid = (*fit)->getID();
	}

	// Set Fixed Frame vertices
	for (KeyFrameList::iterator cit = fixedCameras.begin(); cit != fixedCameras.end(); cit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*cit)->getPose()));
		vSE3->setId((*cit)->getID());
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);//添加lFixedCameras关键帧位姿作为顶点
		if ((*cit)->getID()>maxKFid)
			maxKFid = (*cit)->getID();
	}

	// Set MapPoint3d vertices
	const int nExpectedSize = (localKFList.size() + fixedCameras.size())*localMptList.size();

	EdgeList vpEdges;
	vpEdges.reserve(nExpectedSize);

	std::vector<KeyFrame*> vpEdgeKFs;
	vpEdgeKFs.reserve(nExpectedSize);

	std::vector<MapPoint3d*> vpMapPointEdge;
	vpMapPointEdge.reserve(nExpectedSize);

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;

		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		int id = pMP->getID() + maxKFid + 1;
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//添加局部地图点pMP作为顶点

		const KFSet& observationsKF = pMP->getObservationKFs();//主线程会改变观察帧集合，需加锁

		//Set edges
		for (KFSet::const_iterator kFit = observationsKF.begin(); kFit != observationsKF.end(); kFit++)
		{//遍历局部地图点pMP的观察关键帧
			KeyFrame* pKF = *kFit;//获取局部地图点pMP的观察帧obsKF
			const KeyPoint* keyPointLeft = pKF->_matchedMptMap.find(pMP)->second.first;//获取局部地图点pMP在pKF上的特征点
			const KeyPoint* keyPointRight = pKF->_matchedMptMap.find(pMP)->second.second;//获取局部地图点pMP在pKF上的特征点								


			Eigen::Matrix<double, 2, 1> obs;
			obs << keyPointLeft->_undistortX, keyPointLeft->_undistortY;//设置观察值为地图点pMP在obsKF帧的投影特征点

			EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//设置边的顶点为局部地图点pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//设置边的顶点为pMP的观察帧obsKF位姿

			e->setMeasurement(obs);//设置边的测量值
								   //const double &invSigma2 = 1.0;

			const double wSigmaInv = keyPointLeft->_weight* keyPointRight->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix2d::Identity()*wSigmaInv*wSigmaInv);//信息矩阵

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//设置鲁棒核函数，限制误差过大边的误差
											//离散边不参与局部BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//误差过大边不参与优化

			////////////////设置pKFi帧的相机内参///////////////
			e->fx = pKF->getCamera()->Kl().at<double>(0, 0);
			e->fy = pKF->getCamera()->Kl().at<double>(1, 1);
			e->cx = pKF->getCamera()->Kl().at<double>(0, 2);
			e->cy = pKF->getCamera()->Kl().at<double>(1, 2);

			optimizer.addEdge(e);//添加边
			vpEdges.emplace_back(e);
			vpEdgeKFs.emplace_back(pKF);
			vpMapPointEdge.emplace_back(pMP);

		}
	}
	lockMptObs.unlock();
	lockEraseMpt.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//避免主线程poseOptimization3D和子线程localBundleAdjustment3D冲突


	//////////////第一轮局部BA///////////////
	if (bAbort)
		return;
	optimizer.initializeOptimization(0);
	optimizer.optimize(5);

	bool bDoMore = true;

	if (bDoMore)
	{

		for (size_t i = 0, iend = vpEdges.size(); i < iend; i++)
		{
			EdgeSE3ProjectXYZ* e = vpEdges[i];
			MapPoint3d* pMP = vpMapPointEdge[i];
			if (pMP->isBad())
				continue;
			if (e->level() != 0)
				e->computeError();
			if (e->chi2() > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR || !e->isDepthPositive())
			{
				//pMP->_fOutLier = true;
				e->setLevel(1);
			}
			else
				e->setLevel(0);
			e->setRobustKernel(0);
		}


		/////////////////////重新第二轮局部BA//////////////////////	
		if (bAbort)
			return;
		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

	}
	double totalError2 = 0.0;
	for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
	{
		if ((*itr)->level() == 0)
			totalError2 += (*itr)->chi2();
	}
	double RMSE = sqrt(totalError2 / optimizer.activeEdges().size());
	//std::unique_lock<std::mutex> lockStd(pMap->_mutex);
	std::cout << "Edge size:" << optimizer.edges().size() <<
		", Local optimization square error:" << RMSE <<
		", Frame id: " << keyFrame->getID()
		<< std::endl << std::endl;;
	//lockStd.unlock();

	std::set<std::pair<KeyFrame*, MapPoint3d*> > vToErase;

	for (size_t i = 0, iend = vpMapPointEdge.size(); i < iend; i++)
	{//重新计算每个局部地图点的离散观察帧数
		MapPoint3d* pMP = vpMapPointEdge[i];
		pMP->_outLierCount = 0;
	}
	// 去除离散地图点和离散观察帧
	for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	{
		EdgeSE3ProjectXYZ* e = vpEdges[i];
		MapPoint3d* pMP = vpMapPointEdge[i];
		KeyFrame* obsKF = vpEdgeKFs[i];
		if (pMP->isBad())
			continue;

		if (e->chi2()>MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR || !e->isDepthPositive())
		{
			pMP->_outLierCount++;
			vToErase.insert(std::make_pair(obsKF, pMP));
		}
	}
	for (size_t i = 0, iend = vpMapPointEdge.size(); i < iend; i++)
	{//BA后通过地图点离散观察帧数判断局部地图点是否是坏点
		MapPoint3d* pMP = vpMapPointEdge[i];
		if ((float)pMP->_outLierCount / (float)pMP->obsKF() >= 0.5)
		{
			pMP->setBad(true);
			pMap->insertBadMpt(pMP);
		}
			
	}

	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//冗余帧删除和局部BA在同一线程

	if (!vToErase.empty())
	{
		for (auto itr = vToErase.begin(); itr != vToErase.end(); itr++)
		{
			KeyFrame* obsKF = itr->first;
			MapPoint3d* pMPi = itr->second;
			obsKF->eraseMptMatchedWithP3D(pMPi);//对应观察关键帧帧删除该匹配地图点，避免下一次参与局部BA
		}
	}

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());
	lockEraseMpt.lock();//避免主线程删除地图点

	//更新局部关键帧、帧和地图点位姿
	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{
		KeyFrame* pKF = *lit;

		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->getID()));
		cv::Mat TcwCorrected = Converter::toCvMat(vSE3->estimate());
		pKF->setPose(TcwCorrected); //更新pKF位姿

		pKF->updateLocalPoses();//更新pKF邻接帧位姿


	}

	lockTrack.unlock();
			 

    ///////////////////重新计算局部地图点(迭代优化时主线程可能删除地图点)///////////////////
	localMptList.clear();
	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (!pMP->isBad())
				if (pMP->_localForKF != -1)
				{
					localMptList.emplace_back(pMP);//向局部地图点集合localMapPoints添加局部关键帧的匹配地图点
					pMP->_localForKF = -1;
				}

		}
	}
	//更新局部地图点坐标
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID() + maxKFid + 1));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);

		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}

}




void Optimizer::localBundleAdjustment2D(Frame* currentFrame, GlobalMap* pMap,FrameList& localFrameList, MptList& localMptList)
{
	localFrameList.emplace_back(currentFrame);//向局部关键帧集合lLocalKeyFrames添加当前关键帧
	currentFrame->_localForKF = currentFrame->getID();

	const FrameList& neighborMap = currentFrame->getBestCovisibles(5);
	for (FrameList::const_reverse_iterator itr = neighborMap.rbegin(); itr != neighborMap.rend(); itr++)
	{
		localFrameList.emplace_back(*itr);
		(*itr)->_localForKF = currentFrame->getID();
	}


	FrameList& frameList = pMap->getFrameList();


	// Local MapPoints seen in Local KeyFrames
	//std::list<MapPoint3d*> localMapPoints;
	for (FrameList::iterator lit = localFrameList.begin(), lend = localFrameList.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (pMP)
				if (!pMP->isBad())
					if (pMP->_localForKF != currentFrame->getID())
					{
						localMptList.emplace_back(pMP);//向局部地图点集合localMapPoints添加局部关键帧的匹配地图点
						pMP->_localForKF = currentFrame->getID();
					}
		}
	}
	// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);

	// 设置地图点顶点
	unsigned long maxMptId = 0;

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		vPoint->setId(pMP->getID());
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//添加局部地图点pMP作为顶点
		if (pMP->getID() > maxMptId)
			maxMptId = pMP->getID();
	}

	// 设置帧位姿顶点
	const int nExpectedSize = localFrameList.size()*localMptList.size();

	std::vector<EdgeSE3ProjectXYZ*> vpEdges;
	vpEdges.reserve(nExpectedSize);

	std::vector<Frame*> vpEdgeFrames;
	vpEdgeFrames.reserve(nExpectedSize);

	std::vector<MapPoint3d*> vpMapPointEdge;
	vpMapPointEdge.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);


	for (FrameList::iterator fit = localFrameList.begin(); fit != localFrameList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		int id = (*fit)->getID() + maxMptId + 1;
		vSE3->setId(id);
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//添加局部关键帧的位姿作为顶点


		MptKeyPairMap& mapPointList = (*fit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mItr = mapPointList.begin(); mItr != mapPointList.end(); mItr++)
		{
			MapPoint3d* pMP = mItr->first;
			const KeyPoint* keyPointLeft = (*fit)->_matchedMptMap.find(pMP)->second.first;//获取局部地图点pMP在obsFrame上的特征点
																							 //获取关键帧pKFi的矫正特征点kpUn


			Eigen::Matrix<double, 2, 1> obs;
			obs << keyPointLeft->_undistortX, keyPointLeft->_undistortY;//设置观察值为地图点pMP在obsKF帧的投影特征点

			EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pMP->getID())));//设置边的顶点为局部地图点pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//设置边的顶点为pMP的观察帧obsFrame位姿
			e->setMeasurement(obs);//设置边的测量值
			const double &invSigma2 = 1.0;
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);

			////////////////设置pKFi帧的相机内参///////////////
			e->fx = (*fit)->getCamera()->Kl().at<double>(0, 0);
			e->fy = (*fit)->getCamera()->Kl().at<double>(1, 1);
			e->cx = (*fit)->getCamera()->Kl().at<double>(0, 2);
			e->cy = (*fit)->getCamera()->Kl().at<double>(1, 2);

			optimizer.addEdge(e);//添加边
			vpEdges.emplace_back(e);
			vpEdgeFrames.emplace_back((*fit));
			vpMapPointEdge.emplace_back(pMP);
		}
		
	}
	/*
	// Set Fixed Frame vertices
	for (std::list<Frame*>::iterator cit = fixedCameras.begin(); cit != fixedCameras.end(); cit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*cit)->getPose()));
		vSE3->setId((*cit)->getID());
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);//添加lFixedCameras关键帧位姿作为顶点
		if ((*cit)->getID()>maxFrameId)
			maxFrameId = (*cit)->getID();
	}*/


	optimizer.initializeOptimization();
	optimizer.optimize(5);

	bool bDoMore = true;

	if (bDoMore)
	{

		for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
		{
			EdgeSE3ProjectXYZ* e = vpEdges[i];
			MapPoint3d* pMP = vpMapPointEdge[i];
			if (pMP->isBad())
				continue;
			

			if (e->chi2()>5.991 || !e->isDepthPositive())
			{
				e->setLevel(1);
			}

			e->setRobustKernel(0);
		}


		optimizer.initializeOptimization(0);
		optimizer.optimize(10);

	}
	std::cout << "Edge size:" << optimizer.edges().size() << ", Local optimization square error:" << sqrt(optimizer.activeChi2() / optimizer.edges().size()) << std::endl;

	std::vector<std::pair<Frame*, MapPoint3d*> > vToErase;
	vToErase.reserve(vpEdges.size());

	// 去除离散地图点和离散观察帧    
	for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	{
		EdgeSE3ProjectXYZ* e = vpEdges[i];
		MapPoint3d* pMP = vpMapPointEdge[i];
		if (pMP->isBad())
			continue;

		if (e->chi2()>5.991 || !e->isDepthPositive())
		{
			Frame* obsKF = vpEdgeFrames[i];
			vToErase.emplace_back(std::make_pair(obsKF, pMP));
		}
	}


	if (!vToErase.empty())
	{
		for (size_t i = 0; i<vToErase.size(); i++)
		{
			Frame* obsFrame = vToErase[i].first;
			MapPoint3d* pMPi = vToErase[i].second;
			//obsKF->eraseMptMatched(pMPi);
			//pMPi->eraseObservation(obsKF);
		}
	}

	//更新局部地图点坐标
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID()));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);

		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>1.0)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}

	//更新局部帧位姿
	for (FrameList::iterator lit = localFrameList.begin(), lend = localFrameList.end(); lit != lend; lit++)
	{
		Frame* frame = *lit;
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(frame->getID()+maxMptId+1));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		frame->setPose(Converter::toCvMat(SE3quat));
		KeyFrame* pKF = dynamic_cast<KeyFrame*>(frame); 
		if (pKF)
			pKF->setPose(Converter::toCvMat(SE3quat));

	}

}




void Optimizer::localBundleAdjustment3D(Frame* currentFrame, GlobalMap* pMap, FrameList& localFrameList, MptList& localMptList)
{

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//上锁，避免主线程删除地图点

	// Local MapPoints seen in Local KeyFrames
	//std::list<MapPoint3d*> localMapPoints;
	for (FrameList::iterator lit = localFrameList.begin(), lend = localFrameList.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (pMP)
				if (!pMP->isBad())
					//if (pMP->_localForKF != keyFrame->getID())
					if (pMP->_localForKF != currentFrame->getID())
					{
						localMptList.emplace_back(pMP);//向局部地图点集合localMapPoints添加局部关键帧的匹配地图点
						//pMP->_localForKF = keyFrame->getID();
						pMP->_localForKF = currentFrame->getID();
					}
		}
	}

	// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);


	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//锁住地图点，避免主线程更新地图点的观察帧

	// 设置地图点顶点
	unsigned long maxMptId = 0;

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		vPoint->setId(pMP->getID());
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//添加局部地图点pMP作为顶点
		if (pMP->getID() > maxMptId)
			maxMptId = pMP->getID();
	}

	// 设置帧位姿顶点
	const int nExpectedSize = localFrameList.size()*localMptList.size();

	std::vector<EdgeProjectXYZRGBD*> vpEdges;
	vpEdges.reserve(nExpectedSize);

	std::vector<Frame*> vpEdgeFrames;
	vpEdgeFrames.reserve(nExpectedSize);

	std::vector<MapPoint3d*> vpMapPointEdge;
	vpMapPointEdge.reserve(nExpectedSize);

	const float thHuberMono = sqrt(5.991);


	for (FrameList::iterator fit = localFrameList.begin(); fit != localFrameList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		int id = (*fit)->getID() + maxMptId + 1;
		vSE3->setId(id);
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//添加局部关键帧的位姿作为顶点


	    MptKeyPairMap& mapPointList = (*fit)->_matchedMptMap;//主线程会改变该帧的匹配地图点集合，需加锁
		for (MptKeyPairMap::iterator mItr = mapPointList.begin(); mItr != mapPointList.end(); mItr++)
		{
			MapPoint3d* pMP = mItr->first;
			cv::Point3d* p3d = mItr->second.first->_matchedP3D;
			if (!p3d)
			{
				std::cout << "Can't get corresponding 3d point from map point in optimiazation!" << std::endl;
				return;
			}

			Eigen::Matrix<double, 3, 1> obs;
			obs << p3d->x, p3d->y, p3d->z;

			EdgeProjectXYZRGBD* e = new EdgeProjectXYZRGBD();

			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//设置边的顶点为pMP的观察帧obsFrame位姿
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pMP->getID())));//设置边的顶点为局部地图点pMP
			
			e->setMeasurement(obs);//设置边的测量值
			const double &invSigma2 = 1.0;
			e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);

			optimizer.addEdge(e);//添加边
			vpEdges.emplace_back(e);
			vpEdgeFrames.emplace_back((*fit));
			vpMapPointEdge.emplace_back(pMP);
		}

	}

	lockMptObs.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//避免主线程poseOptimization3D和子线程localBundleAdjustment3D冲突


	optimizer.initializeOptimization();
	optimizer.optimize(1);//此值要调试

	bool bDoMore = true;

	double errorMax = 16.0;
	if (bDoMore)
	{
		for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
		{
			EdgeProjectXYZRGBD* e = vpEdges[i];
			MapPoint3d* pMP = vpMapPointEdge[i];
			if (pMP->isBad())
				continue;


			if (e->chi2()>errorMax)
			{
				e->setLevel(1);
			}

			e->setRobustKernel(0);
		}


		optimizer.initializeOptimization(0);
		optimizer.optimize(10);//不能过小

	}

	std::cout << "Edge size:" << optimizer.edges().size() << ", Local optimization square error:" << sqrt(optimizer.activeChi2() / optimizer.edges().size()) << std::endl;
	

	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);
	
	std::vector<std::pair<Frame*, MapPoint3d*> > vToErase;
	vToErase.reserve(vpEdges.size());

	// 去除离散地图点和离散观察帧    
	for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	{
		EdgeProjectXYZRGBD* e = vpEdges[i];
		MapPoint3d* pMP = vpMapPointEdge[i];
		if (pMP->isBad())
			continue;

		if (e->chi2()>errorMax)
		{
			Frame* obsKF = vpEdgeFrames[i];
			vToErase.emplace_back(std::make_pair(obsKF, pMP));
		}
	}

	if (!vToErase.empty())
	{
		for (size_t i = 0; i<vToErase.size(); i++)
		{
			Frame* obsFrame = vToErase[i].first;
			MapPoint3d* pMPi = vToErase[i].second;
			//obsKF->eraseMptMatched(pMPi);
			//pMPi->eraseObservation(obsKF);
		}
	}

	//更新局部地图点坐标
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID()));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast= *dynamic_cast<cv::Point3d*>(pMP);
		
		//if(cv::norm(pMP->_coordLast-pMP->_coordOri)>1.0)
		//	pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);
		
	}

	//更新局部帧位姿
	for (FrameList::iterator lit = localFrameList.begin(), lend = localFrameList.end(); lit != lend; lit++)
	{
		Frame* frame = *lit;
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(frame->getID() + maxMptId + 1));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		frame->setPose(Converter::toCvMat(SE3quat));
		KeyFrame* pKF = dynamic_cast<KeyFrame*>(frame);
		if (pKF)
			pKF->setPose(Converter::toCvMat(SE3quat));

	}
}




/****************************************************************************
pCurKF:尾部关键帧
pLoopKF:首部匹配关键帧
correctedSim3:尾部邻域共点帧矫正后的位姿
nonCorrectedSim3:尾部邻域共点帧的原始位姿
loopConnections:键--值 = 尾部邻域共点帧--该关键帧的首部共点关键帧集合
*****************************************************************************/
//void Optimizer::optimizeEssentialGraph(GlobalMap* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
//									   const LoopClosing::KeyFrameAndPose &nonCorrectedSim3,
//									   const LoopClosing::KeyFrameAndPose &correctedSim3,
//									   const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections,
//									   const bool& bFixScale)
//{
//	
//	g2o::SparseOptimizer optimizer;
//	optimizer.setVerbose(false);
//	g2o::BlockSolver_7_3::LinearSolverType * linearSolver =new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
//	g2o::BlockSolver_7_3 * solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
//	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//
//	solver->setUserLambdaInit(1e-16);
//	optimizer.setAlgorithm(solver);
//
//	const KeyFrameList keyFrameList = pMap->getKeyFrameList();//LocalMapping线程局部BA及关键帧删除已停止
//
//	const unsigned int maxKFid = keyFrameList.back()->_id;
//	std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > Sim3s(maxKFid + 1);
//
//
//	for (size_t i = 0, iend = keyFrameList.size(); i<iend; i++)
//	{
//		KeyFrame* pKF = keyFrameList[i];
//
//		g2o::VertexSim3Expmap* vSim3 = new g2o::VertexSim3Expmap();
//
//		LoopClosing::KeyFrameAndPose::const_iterator itr = correctedSim3.find(pKF);
//		if (itr != correctedSim3.end())
//		{//Sim3s保存尾部邻域帧已矫正位姿
//			cv::Mat Tcw = itr->second;
//			g2o::Sim3 Sim= g2o::Sim3(Converter::toMatrix3d(Tcw.colRange(0, 3).rowRange(0, 3)),
//								     Converter::toVector3d(Tcw.col(3).rowRange(0, 3)),
//								     1.0);
//			Sim3s[pKF->_id] = Sim;
//			vSim3->setEstimate(Sim);
//		}
//		else
//		{//Sim3s保存头部邻域帧原始位姿
//			g2o::Sim3 Sim(Converter::toMatrix3d(pKF->_Tcw.colRange(0, 3).rowRange(0, 3)),
//						  Converter::toVector3d(pKF->_Tcw.col(3).rowRange(0, 3)),
//						  1.0);
//			Sim3s[pKF->_id] = Sim;
//			vSim3->setEstimate(Sim);
//		}
//
//		if (pKF == pLoopKF)
//			vSim3->setFixed(true);
//
//		vSim3->setId(pKF->_id);
//		vSim3->setMarginalized(false);
//		vSim3->_fix_scale = true;
//
//		optimizer.addVertex(vSim3);
//	}
//
//
//	std::set<std::pair<long unsigned int, long unsigned int> > insertedEdges;
//
//	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();
//
//	for (std::map<KeyFrame*, std::set<KeyFrame*>>::const_iterator mit = loopConnections.begin(), mend = loopConnections.end(); mit != mend; mit++)
//	{//遍历尾部邻域共点关键帧pBackKF
//		KeyFrame* pBackKF = mit->first;
//		const std::set<KeyFrame*> &frontConnections = mit->second;
//		const g2o::Sim3 Sbw = Sim3s[pBackKF->_id];//尾部pBackKF位姿（已矫正）
//		const g2o::Sim3 Swb = Sbw.inverse();
//
//		for (std::set<KeyFrame*>::const_iterator frontItr = frontConnections.begin(), send = frontConnections.end(); frontItr != send; frontItr++)
//		{//遍历尾部pBackKF的首部共点关键帧
//			KeyFrame* pFrontKF = *frontItr;
//			if ((pBackKF->_id != pCurKF->_id || pFrontKF->_id != pLoopKF->_id)&& pBackKF->getKFCovisibleCount(pFrontKF)<3)
//				continue;
//
//			const g2o::Sim3 Sfw = Sim3s[pFrontKF->_id];//pBackKF首部共点帧位姿（原始）
//			const g2o::Sim3 Sfb = Sfw * Swb;//Sji：尾部pBackKF(已矫正)->首部共点帧pFrontKF(原始)
//
//			g2o::EdgeSim3* e = new g2o::EdgeSim3();
//			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pFrontKF->_id)));//顶点1：pBackKF首部共点关键帧（未矫正）
//			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pBackKF->_id)));//顶点0：尾部邻域关键帧pBackKF（已矫正）
//			e->setMeasurement(Sfb);
//
//			e->information() = matLambda;
//
//			optimizer.addEdge(e);
//
//			insertedEdges.insert(std::make_pair(std::min(pBackKF->_id, pFrontKF->_id), std::max(pBackKF->_id, pFrontKF->_id)));
//		}
//	}
//
//	for (size_t i = 0, iend = keyFrameList.size(); i<iend; i++)
//	{//遍历全局关键帧pKF
//		KeyFrame* pKF = keyFrameList[i];
//
//		//Swi：全局pKF位姿(原始)
//		g2o::Sim3 Swi;		
//		LoopClosing::KeyFrameAndPose::const_iterator itr = nonCorrectedSim3.find(pKF);
//		if (itr != nonCorrectedSim3.end())
//		{//pKF属于尾部共点帧（原始）
//			cv::Mat Tiw = itr->second;
//			g2o::Sim3 Siw = g2o::Sim3(Converter::toMatrix3d(Tiw.colRange(0, 3).rowRange(0, 3)),
//								      Converter::toVector3d(Tiw.col(3).rowRange(0, 3)),
//								      1.0);	
//			Swi = Siw.inverse();
//		}	
//		else//pKF非尾部共点帧（原始）
//			Swi = Sim3s[pKF->_id].inverse();
//
//		
//		if (pKF->_maxPrevConnKF)
//		{
//			KeyFrame* pKFj = pKF->_maxPrevConnKF;//pKFj：前向最大共点帧	
//			if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id))))
//				continue;//若已添加该边（尾部pKF->pKF头部共点回环帧），跳过
//
//			g2o::Sim3 Sjw;  //Sjw：pKF前向最大共点帧pMaxPrevKF位姿（原始）
//			LoopClosing::KeyFrameAndPose::const_iterator itj = nonCorrectedSim3.find(pKFj);
//			
//			if (itj != nonCorrectedSim3.end())
//			{
//				cv::Mat Tjw = itj->second;
//				Sjw = g2o::Sim3(Converter::toMatrix3d(Tjw.colRange(0, 3).rowRange(0, 3)),
//										  Converter::toVector3d(Tjw.col(3).rowRange(0, 3)),
//										  1.0);
//			}
//			else
//				Sjw = Sim3s[pKFj->_id];
//			
//			g2o::Sim3 Sji = Sjw * Swi;//pKF（原始）->pKF前向最大共点帧（原始）
//
//			g2o::EdgeSim3* e = new g2o::EdgeSim3();
//			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFj->_id))); //pKF前向最大共点帧（原始）
//			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF（原始）
//			e->setMeasurement(Sji);
//
//			e->information() = matLambda;
//			optimizer.addEdge(e);
//		}
//
//		// Loop edges
//		const  KFSet& loopKFs = pKF->getLoopKFs();
//		if (!loopKFs.empty())
//		{//如果pKF有历史匹配回环帧
//			for (auto sit = loopKFs.begin(), send = loopKFs.end(); sit != send; sit++)
//			{//遍历pKF历史回环匹配帧（若存在）
//				KeyFrame* pLoopKF = *sit;
//				if (pLoopKF->_id<pKF->_id)
//				{
//					//Slw：pKF历史回环匹配帧位姿(原始)
//					g2o::Sim3 Slw;
//					LoopClosing::KeyFrameAndPose::const_iterator itl = nonCorrectedSim3.find(pLoopKF);					
//					if (itl != nonCorrectedSim3.end())
//					{
//						cv::Mat Tlw = itl->second;
//						Slw = g2o::Sim3(Converter::toMatrix3d(Tlw.colRange(0, 3).rowRange(0, 3)),
//												  Converter::toVector3d(Tlw.col(3).rowRange(0, 3)),
//												  1.0);
//					}
//					else
//						Slw = Sim3s[pLoopKF->_id];
//
//					g2o::Sim3 Sli = Slw * Swi;//pKF（原始）->pKF匹配回环帧（原始）
//					g2o::EdgeSim3* el = new g2o::EdgeSim3();
//					el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLoopKF->_id)));//pKF匹配回环帧(原始)
//					el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKF（原始）
//					el->setMeasurement(Sli);
//					el->information() = matLambda;
//					optimizer.addEdge(el);
//				}
//			}
//		}
//		
//
//		const std::vector<KeyFrame*> covisibleKFs = pKF->getCovisibleKFs();
//		for (auto itr = covisibleKFs.begin(); itr != covisibleKFs.end(); itr++)
//		{//遍历pKF共点帧pKFn
//			KeyFrame* pKFn = *itr;
//			if (pKFn && pKFn != pKF->_maxPrevConnKF && !pKF->hasNextMaxKF(pKFn)&& !loopKFs.count(pKFn))
//			{//跳过pKF、pKF的父节点、pKF的孩子节点、回环匹配帧
//				if (! pKFn->_id<pKF->_id)
//				{//pKFn在pKF之前
//					if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id))))
//						continue;//若已添加该边（尾部pKF->pKF头部共点回环帧），跳过
//
//					//Snw：pKF剩余共点关键帧位姿（原始）
//					g2o::Sim3 Snw;
//					LoopClosing::KeyFrameAndPose::const_iterator itn = nonCorrectedSim3.find(pKFn);
//					if (itn != nonCorrectedSim3.end())
//					{
//						cv::Mat Tnw = itn->second;
//						Snw = g2o::Sim3(Converter::toMatrix3d(Tnw.colRange(0, 3).rowRange(0, 3)),
//												  Converter::toVector3d(Tnw.col(3).rowRange(0, 3)),
//												  1.0);
//					}
//					else
//						Snw = Sim3s[pKFn->_id];
//
//					g2o::Sim3 Sni = Snw * Swi;//Sni：pKF（原始）->pKF共点帧pKFn（原始）相对位姿
//
//					g2o::EdgeSim3* en = new g2o::EdgeSim3();
//					en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->_id)));//pKF（原始）
//					en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKF剩余共点帧pKFn(原始)
//					en->setMeasurement(Sni);
//					en->information() = matLambda;
//					optimizer.addEdge(en);
//				}
//			}
//		}
//	}
//
//	optimizer.initializeOptimization();
//	optimizer.optimize(20);
//
//	// Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
//
//	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//暂停主线程跟踪最新帧
//	std::unique_lock<std::mutex> lockEraseFrame(GlobalMap::_frameMutex);//暂停删除关键帧
//	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//暂停删除地图点
//	KeyFrameList& KFList = pMap->getKeyFrameList();//优化后的关键帧集合（优化中可能新增或减少）
//	const GlobalMap::MptList mapPointList = pMap->getMapPointList();//非引用，主线程后续可能增加地图点
//
//	int KFsize = KFList.size();
//	for (size_t i = 0; i<KFsize; i++)
//	{
//		KeyFrame* pKFi = KFList[i];
//		if (pKFi->_id <= maxKFid)
//		{//优化前关键帧
//			g2o::VertexSim3Expmap* vSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->_id));
//			g2o::Sim3 correctedSiw = vSim3->estimate();
//			Eigen::Matrix3d eigR = correctedSiw.rotation().toRotationMatrix();
//			Eigen::Vector3d eigt = correctedSiw.translation();
//			double s = correctedSiw.scale();
//
//			eigt *= (1. / s); //[R t/s;0 1]
//
//			cv::Mat TiwCorrected = Converter::toCvSE3(eigR, eigt);
//			pKFi->_TcwGBA = TiwCorrected;
//			/*pKFi->setPose(TiwCorrected);
//			pKFi->updateLocalPoses();*/
//		}
//		else
//		{//新增关键帧
//			pKFi->_TcwGBA = pKFi->getPose()*pKFi->_parentKF->getPoseInverse()*pKFi->_parentKF->_TcwGBA;
//		}
//	}
//
//	for (size_t i = 0, iend = mapPointList.size(); i<iend; i++)
//	{
//		MapPoint3d* pMP = mapPointList[i];
//
//		if (pMP->isBad())
//			continue;
//	
//	/*		g2o::Sim3 Sim3 = Sim3s[pMP->_refKF->_id];
//
//
//			cv::Mat x3Dw = pMP->getWorldPos();
//			Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(x3Dw);
//			Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSim3.inverse().map(Sim3.map(eigP3Dw));
//
//			cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
//			pMP->setWorldPos(cvCorrectedP3Dw);
//			pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);
//
//			if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
//				pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
//		cv::Mat x3Dw = cv::Mat(dynamic_cast<cv::Point3d&>(*pMP));
//		cv::Mat x3Dc = pMP->_refKF->_Rcw*x3Dw + pMP->_refKF->_tcw;
//
//		cv::Mat RwcCorrected = pMP->_refKF->_TcwGBA.colRange(0, 3).rowRange(0, 3).t();
//		cv::Mat	OwCorrected = -RwcCorrected * pMP->_refKF->_TcwGBA.col(3).rowRange(0, 3);
//
//		cv::Mat x3DwCorrected = RwcCorrected*x3Dc + OwCorrected;
//		pMP->setWorldPos(x3DwCorrected);
//		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);
//
//		if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
//			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);
//			
//	}
//	lockEraseMpt.unlock();
//
//	for (size_t i = 0; i < KFsize; i++)
//	{
//		/*if (KFList[i]->_id == maxKFid)
//			KFList[i]->updateChildrenPoses(KFList[i]->_TcwGBA);
//
//		else if (KFList[i]->_id > maxKFid)
//		{
//			KFList[i]->setPose(KFList[i]->getPose()*KFList[i]->_parentKF->getPoseInverse() * KFList[i]->_parentKF->_TcwGBA);
//		}*/
//
//		KFList[i]->setPose(KFList[i]->_TcwGBA);
//		KFList[i]->updateLocalPoses();
//		KFList[i]->_TcwGBA.release();
//	}
//}



void Optimizer::optimizeEssentialGraph(GlobalMap* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
									   const LoopClosing::KeyFrameAndPose &nonCorrectedSim3,
									   const LoopClosing::KeyFrameAndPose &correctedSim3,
									   const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections,
									   const bool& bFixScale)
{
	// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	const KeyFrameList keyFrameList = pMap->getKeyFrameList();//LocalMapping线程局部BA及关键帧删除已停止

	const unsigned int maxKFid = keyFrameList.back()->_id;
	std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > Sim3s(maxKFid + 1);


	for (size_t i = 0, iend = keyFrameList.size(); i<iend; i++)
	{
		KeyFrame* pKF = keyFrameList[i];
		if (pKF->isBad())
			continue;
		g2o::VertexSim3Expmap* vSim3 = new g2o::VertexSim3Expmap();

		LoopClosing::KeyFrameAndPose::const_iterator itr = correctedSim3.find(pKF);
		if (itr != correctedSim3.end())
		{//Sim3s保存尾部邻域帧已矫正位姿
			cv::Mat Tcw = itr->second;
			g2o::Sim3 Sim= g2o::Sim3(Converter::toMatrix3d(Tcw.colRange(0, 3).rowRange(0, 3)),
								     Converter::toVector3d(Tcw.col(3).rowRange(0, 3)),
								     1.0);
			Sim3s[pKF->_id] = Sim;
			vSim3->setEstimate(Sim);
		}
		else
		{//Sim3s保存头部邻域帧原始位姿
			g2o::Sim3 Sim(Converter::toMatrix3d(pKF->_Tcw.colRange(0, 3).rowRange(0, 3)),
						  Converter::toVector3d(pKF->_Tcw.col(3).rowRange(0, 3)),
						  1.0);
			Sim3s[pKF->_id] = Sim;
			vSim3->setEstimate(Sim);
		}

		if (pKF == pLoopKF)
			vSim3->setFixed(true);

		vSim3->setId(pKF->_id);
		vSim3->setMarginalized(false);
		vSim3->_fix_scale = true;

		optimizer.addVertex(vSim3);
	}


	std::set<std::pair<long unsigned int, long unsigned int> > insertedEdges;

	const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

	for (std::map<KeyFrame*, std::set<KeyFrame*>>::const_iterator mit = loopConnections.begin(), mend = loopConnections.end(); mit != mend; mit++)
	{//遍历尾部邻域共点关键帧pBackKF
		KeyFrame* pBackKF = mit->first;
		const std::set<KeyFrame*> &frontConnections = mit->second;
		const g2o::Sim3 Sbw = Sim3s[pBackKF->_id];//尾部pBackKF位姿（已矫正）
		const g2o::Sim3 Swb = Sbw.inverse();

		for (std::set<KeyFrame*>::const_iterator frontItr = frontConnections.begin(), send = frontConnections.end(); frontItr != send; frontItr++)
		{//遍历尾部pBackKF的首部共点关键帧
			KeyFrame* pFrontKF = *frontItr;
			if ((pBackKF->_id != pCurKF->_id || pFrontKF->_id != pLoopKF->_id) && pBackKF->getKFCovisibleCount(pFrontKF)<3)
				continue;

			const g2o::Sim3 Sfw = Sim3s[pFrontKF->_id];//pBackKF首部共点帧位姿（原始）
			const g2o::Sim3 Sfb = Sfw * Swb;//Sji：尾部pBackKF(已矫正)->首部共点帧pFrontKF(原始)

			g2o::EdgeSim3* e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pFrontKF->_id)));//顶点1：pBackKF首部共点关键帧（未矫正）
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pBackKF->_id)));//顶点0：尾部邻域关键帧pBackKF（已矫正）
			e->setMeasurement(Sfb);

			e->information() = matLambda;

			optimizer.addEdge(e);

			insertedEdges.insert(std::make_pair(std::min(pBackKF->_id, pFrontKF->_id), std::max(pBackKF->_id, pFrontKF->_id)));
		}
	}

	for (size_t i = 0, iend = keyFrameList.size(); i<iend; i++)
	{//遍历全局关键帧pKF
		KeyFrame* pKF = keyFrameList[i];

		//Swi：全局pKF位姿(原始)
		g2o::Sim3 Swi;		
		LoopClosing::KeyFrameAndPose::const_iterator itr = nonCorrectedSim3.find(pKF);
		if (itr != nonCorrectedSim3.end())
		{//pKF属于尾部共点帧（原始）
			cv::Mat Tiw = itr->second;
			g2o::Sim3 Siw = g2o::Sim3(Converter::toMatrix3d(Tiw.colRange(0, 3).rowRange(0, 3)),
								      Converter::toVector3d(Tiw.col(3).rowRange(0, 3)),
								      1.0);	
			Swi = Siw.inverse();
		}	
		else//pKF非尾部共点帧（原始）
			Swi = Sim3s[pKF->_id].inverse();

		//if (pKF->_maxPrevConnKF)
		//{
		//	KeyFrame* pKFj = pKF->_maxPrevConnKF;//pKFj：前向最大共点帧	
		//	if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id))))
		//		continue;//若已添加该边（尾部pKF->pKF头部共点回环帧），跳过
		//	
		//	g2o::Sim3 Sjw;  //Sjw：pKF前向最大共点帧pMaxPrevKF位姿（原始）
		//	LoopClosing::KeyFrameAndPose::const_iterator itj = nonCorrectedSim3.find(pKFj);
		//				
		//	if (itj != nonCorrectedSim3.end())
		//	{
		//		cv::Mat Tjw = itj->second;
		//		Sjw = g2o::Sim3(Converter::toMatrix3d(Tjw.colRange(0, 3).rowRange(0, 3)),
		//									Converter::toVector3d(Tjw.col(3).rowRange(0, 3)),
		//									1.0);
		//	}
		//	else
		//		Sjw = Sim3s[pKFj->_id];
		//				
		//	g2o::Sim3 Sji = Sjw * Swi;//pKF（原始）->pKF前向最大共点帧（原始）
		//	
		//	g2o::EdgeSim3* e = new g2o::EdgeSim3();
		//	e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFj->_id))); //pKF前向最大共点帧（原始）
		//	e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF（原始）
		//	e->setMeasurement(Sji);
		//	
		//	e->information() = matLambda;
		//	optimizer.addEdge(e);
		//	insertedEdges.insert(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id)));
		//}



		const  KFSet& loopKFs = pKF->getLoopKFs();
		if (!loopKFs.empty())
		{//如果pKF有历史匹配回环帧
			for (auto sit = loopKFs.begin(), send = loopKFs.end(); sit != send; sit++)
			{//遍历pKF历史回环匹配帧（若存在）
				KeyFrame* pLoopKF = *sit;
				if (pLoopKF->_id<pKF->_id)
				{
					//Slw：pKF历史回环匹配帧位姿(原始)
					g2o::Sim3 Slw;
					LoopClosing::KeyFrameAndPose::const_iterator itl = nonCorrectedSim3.find(pLoopKF);
					if (itl != nonCorrectedSim3.end())
					{
						cv::Mat Tlw = itl->second;
						Slw = g2o::Sim3(Converter::toMatrix3d(Tlw.colRange(0, 3).rowRange(0, 3)),
							Converter::toVector3d(Tlw.col(3).rowRange(0, 3)),
							1.0);
					}
					else
						Slw = Sim3s[pLoopKF->_id];

					g2o::Sim3 Sli = Slw * Swi;//pKF（原始）->pKF匹配回环帧（原始）
					g2o::EdgeSim3* el = new g2o::EdgeSim3();
					el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLoopKF->_id)));//pKF匹配回环帧(原始)
					el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKF（原始）
					el->setMeasurement(Sli);
					el->information() = matLambda;
					optimizer.addEdge(el);
				}
			}
		}
		if (pKF->_parentKF)
		{
			KeyFrame* pKFn = pKF->_parentKF;//pKFj：前向最大共点帧	
			if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id))))
				continue;//若已添加该边（尾部pKF->pKF头部共点回环帧），跳过

			g2o::Sim3 Snw;  //Snw：pKF前向共点关键帧位姿（原始）
			LoopClosing::KeyFrameAndPose::const_iterator itj = nonCorrectedSim3.find(pKFn);

			if (itj != nonCorrectedSim3.end())
			{
				cv::Mat Tjw = itj->second;
				Snw = g2o::Sim3(Converter::toMatrix3d(Tjw.colRange(0, 3).rowRange(0, 3)),
					Converter::toVector3d(Tjw.col(3).rowRange(0, 3)),
					1.0);
			}
			else
				Snw = Sim3s[pKFn->_id];

			g2o::Sim3 Sni = Snw * Swi;//pKF（原始）->pKF前向最大共点帧（原始）

			g2o::EdgeSim3* e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->_id))); //pKF前向最大共点帧（原始）
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF（原始）
			e->setMeasurement(Sni);

			e->information() = matLambda;
			optimizer.addEdge(e);

			insertedEdges.insert(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id)));
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(20);

	// Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//暂停主线程跟踪最新帧
	std::unique_lock<std::mutex> lockEraseFrame(GlobalMap::_frameMutex);//暂停删除关键帧
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//暂停删除地图点
	KeyFrameList& KFList = pMap->getKeyFrameList();//优化后的关键帧集合（优化中可能新增或减少）
	const MptList mapPointList = pMap->getMapPointList();//非引用，主线程后续可能增加地图点

	int KFsize = KFList.size();
	for (size_t i = 0; i<KFsize; i++)
	{
		KeyFrame* pKFi = KFList[i];
		if (pKFi->_id <= maxKFid)
		{//优化前关键帧
			g2o::VertexSim3Expmap* vSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->_id));
			g2o::Sim3 correctedSiw = vSim3->estimate();
			Eigen::Matrix3d eigR = correctedSiw.rotation().toRotationMatrix();
			Eigen::Vector3d eigt = correctedSiw.translation();
			double s = correctedSiw.scale();

			eigt *= (1. / s); //[R t/s;0 1]

			cv::Mat TiwCorrected = Converter::toCvSE3(eigR, eigt);
			pKFi->_TcwGBA = TiwCorrected;
			/*pKFi->setPose(TiwCorrected);
			pKFi->updateLocalPoses();*/
		}
		else
		{//新增关键帧
			pKFi->_TcwGBA = pKFi->getPose()*pKFi->_parentKF->getPoseInverse()*pKFi->_parentKF->_TcwGBA;
		}
	}

	for (size_t i = 0, iend = mapPointList.size(); i<iend; i++)
	{
		MapPoint3d* pMP = mapPointList[i];

		if (pMP->isBad())
			continue;

		/*		g2o::Sim3 Sim3 = Sim3s[pMP->_refKF->_id];


		cv::Mat x3Dw = pMP->getWorldPos();
		Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(x3Dw);
		Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSim3.inverse().map(Sim3.map(eigP3Dw));

		cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
		pMP->setWorldPos(cvCorrectedP3Dw);
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);

		if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
		pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
		cv::Mat x3Dw = cv::Mat(dynamic_cast<cv::Point3d&>(*pMP));
		cv::Mat x3Dc = pMP->_refKF->_Rcw*x3Dw + pMP->_refKF->_tcw;

		cv::Mat RwcCorrected = pMP->_refKF->_TcwGBA.colRange(0, 3).rowRange(0, 3).t();
		cv::Mat	OwCorrected = -RwcCorrected * pMP->_refKF->_TcwGBA.col(3).rowRange(0, 3);

		cv::Mat x3DwCorrected = RwcCorrected*x3Dc + OwCorrected;
		pMP->setWorldPos(x3DwCorrected);
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);

	/*	if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/

	}
	lockEraseMpt.unlock();

	for (size_t i = 0; i < KFsize; i++)
	{
		/*if (KFList[i]->_id == maxKFid)
		KFList[i]->updateChildrenPoses(KFList[i]->_TcwGBA);

		else if (KFList[i]->_id > maxKFid)
		{
		KFList[i]->setPose(KFList[i]->getPose()*KFList[i]->_parentKF->getPoseInverse() * KFList[i]->_parentKF->_TcwGBA);
		}*/

		KFList[i]->setPose(KFList[i]->_TcwGBA);
		KFList[i]->updateLocalPoses();
		KFList[i]->_TcwGBA.release();
	}
}






void Optimizer::globalBundleAdjustment(KeyFrameList& KFList, MptList& mptList,
									   KeyFrame* loopKF, KeyFrame::KFCompSet& backKFSet,
									   const int& nIterations)
{// 设置优化器
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	long unsigned int maxKFid = 0;

	int KFsize = KFList.size();
	for (size_t i = 0; i < KFsize; i++)
	{
		KeyFrame* pKF = KFList[i];

		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();

		vSE3->setEstimate(Converter::toSE3Quat(pKF->getPose()));
		vSE3->setId(pKF->_id);
		if (pKF == loopKF|| backKFSet.count(pKF))
			vSE3->setFixed(true);
		//vSE3->setFixed(pKF->_id == 0);
		optimizer.addVertex(vSE3);//添加顶点为关键帧位姿
		if (pKF->_id>maxKFid)
			maxKFid = pKF->_id;//maxKFid：传入的vpKFs关键帧集合中的最大关键帧id
	}

	int mptSize = mptList.size();
	for (size_t i = 0; i<mptSize; i++)
	{
		MapPoint3d* pMP = mptList[i];
		if (pMP->isBad())
			continue;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		const int id = pMP->_id + maxKFid + 1;//id在已有顶点id基础上递增
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//添加顶点为地图点

		const KFSet observationsKF = pMP->getObservationKFs();//获取观测到该地图点的所有关键帧集合observationsKF

		for (KFSet::const_iterator kFit = observationsKF.begin(); kFit != observationsKF.end(); kFit++)
		{//遍历局部地图点pMP的观察关键帧
			KeyFrame* pKF = *kFit;//获取局部地图点pMP的观察帧obsKF
			if (pKF->isBad())
				continue;
			if (pKF->_id>maxKFid)
				continue;//跳过主线程在全局BA时新建的关键帧
			if (pKF->_outLiers.count(pMP))
				continue;//避免离散匹配地点约束
			cv::Point3d* p3d = pKF->getP3DFromMpt(pMP);
			if (!p3d)
			{
				std::cout << "Can't get corresponding 3d point from map point in optimization!" << std::endl;
				return;
			}

			Eigen::Matrix<double, 3, 1> obs;
			obs << p3d->x, p3d->y, p3d->z;

			EdgeProjectXYZRGBD* e = new EdgeProjectXYZRGBD();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//设置边的顶点为局部地图点pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//设置边的顶点为pMP的观察帧obsKF位姿

			e->setMeasurement(obs);//设置边的测量值
								   //const double &invSigma2 = 1.0;
			KeyPointPair kptPair;
			if (!pKF->getKeyPairFromMpt(pMP, kptPair))
				return;
			const double wSigmaInv = kptPair.first->_weight* kptPair.second->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix3d::Identity()*wSigmaInv*wSigmaInv);//信息矩阵

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//设置鲁棒核函数，限制误差过大边的误差
											//离散边不参与局部BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//误差过大边不参与优化

			optimizer.addEdge(e);//添加边
		}

	}

	// 开始优化
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);//开始优化，迭代次数为nIterations


	for (size_t i = 0; i<KFsize; i++)
	{
		KeyFrame* pKF = KFList[i];
		if (pKF->_id <= maxKFid)
		{//优化前关键帧
			g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->_id));
			g2o::SE3Quat SE3quat = vSE3->estimate();

			pKF->_TcwGBA.create(4, 4, CV_32F);
			Converter::toCvMat(SE3quat).copyTo(pKF->_TcwGBA);
		}	
	}

	for (size_t i = 0; i<mptSize; i++)
	{
		MapPoint3d* pMP = mptList[i];

		if (pMP->isBad())
			continue;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->_id + maxKFid + 1));
		if (vPoint)
		{//优化前地图点
			//pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->_posGBA = Converter::toCvMat(vPoint->estimate());
		}		
		
	}
}

}

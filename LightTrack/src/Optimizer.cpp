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
	vSim3->setEstimate(sim21);//pF1->pF2��ת������
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
		p3D1Vertex->setEstimate(Converter::toVector3d(*itr->second));//pF1��3d��
		p3D1Vertex->setId(id1);
		p3D1Vertex->setFixed(true);
		optimizer.addVertex(p3D1Vertex);

		g2o::VertexSBAPointXYZ* p3D2Vertex = new g2o::VertexSBAPointXYZ();
		p3D2Vertex->setEstimate(Converter::toVector3d(*itr->first));//pF2��3d��
		p3D2Vertex->setId(id2);
		p3D2Vertex->setFixed(true);
		optimizer.addVertex(p3D2Vertex);

		nCorrespondences++;
		///////////////pF2��3d��->pF1//////////////////
		Eigen::Matrix<double, 3, 1> obs1;
		obs1 << itr->second->x, itr->second->y, itr->second->z;

		EdgeInverseSim3ProjectXYZRGBD* e12 = new EdgeInverseSim3ProjectXYZRGBD();
		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		KeyPointPair kptPair1;
		pF1->getKeyPairFromP3D(itr->second, kptPair1);
		double w_sigmaInv1 = 1.0;// kptPair1.first->_weight*kptPair1.second->_weight;// 1.0;
		e12->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv1*w_sigmaInv1);//��Ϣ����

		g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
		e12->setRobustKernel(rk1);
		rk1->setDelta(MAX_EDGE_ERROR);
		optimizer.addEdge(e12);

		///////////////pF1��3d��->pF2//////////////////
		Eigen::Matrix<double, 3, 1> obs2;
		obs2 << itr->first->x, itr->first->y, itr->first->z;

		EdgeSim3ProjectXYZRGBD* e21 = new EdgeSim3ProjectXYZRGBD();
		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);

		KeyPointPair kptPair2;
		pF2->getKeyPairFromP3D(itr->first, kptPair2);
		double w_sigmaInv2 = 1.0; //kptPair2.first->_weight*kptPair2.second->_weight;// 1.0; 
		e21->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv2*w_sigmaInv2);//��Ϣ����

		g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
		e21->setRobustKernel(rk2);
		rk2->setDelta(MAX_EDGE_ERROR);
		optimizer.addEdge(e21);

		vpEdges12.emplace_back(e12);
		vpEdges21.emplace_back(e21);

		e12->computeError();//����ߵ����_error���������ϵ��3d�����
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
	while (validMathcedCount > 3)//����4��ƥ���                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
	{
		RMSE = 0;
		maxError = DBL_MIN;
		totalError2 = 0.;
		worstEdge12 = nullptr;
		worstEdge21 = nullptr;

		std::cout << "Sim3 optimizing..." << std::endl;
		vSim3->setEstimate(g2o::Sim3(Converter::toMatrix3d(T21.colRange(0, 3).rowRange(0, 3)), Converter::toVector3d(T21.col(3).rowRange(0, 3)), 1));//ÿ���Ż�֮ǰ��������˶�
	
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//��ʼ�Ż����Ż���������Ϊits[it]

		for (int i=0;i<nCorrespondences;i++)
		{
			EdgeInverseSim3ProjectXYZRGBD* e12 = vpEdges12[i];
			EdgeSim3ProjectXYZRGBD* e21 = vpEdges21[i];

			const double& error12 = sqrt(e12->error2());
			const double& error21 = sqrt(e21->error2());
			/*else
			{����ԭ
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
		//	break;//ֱ��return��bug(δ����)

		RMSE = sqrt(totalError2 / optimizer.activeEdges().size());
		if (RMSE > MAX_EDGE_RMSE)
		{//����ߵ�RMSE��������������ߣ������Ż�
			worstEdge12->setLevel(1);//���Գ������ý�С����Ϣ����
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
				e12->computeError();//����δ�����Ż��ı����_error�������Ż��ı�����Ҫ��ʽ���ü��㺯��
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
	// �ָ�λ��
	VertexSim3ExpmapRGBD* vSim3_recov = static_cast<VertexSim3ExpmapRGBD*>(optimizer.vertex(0));
	T21 = Converter::toCvMat(vSim3_recov->estimate());

	return validMathcedCount;//�����ڵ�����=pFrame֡��ͼ������-��Ⱥ������
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
	optimizer.addVertex(vSE3); //���Ż�����Ӷ��㣺pFrame֡��λ��

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
	{//����frame��ƥ���ͼ��
		MapPoint3d* mapPoint = itr->first;//frameƥ���ͼ��	
		if (mapPoint->isBad())
			continue;//�����ھֲ�BA���߳�ȷ���Ĵ����ͼ��
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
		e->setInformation(Eigen::Matrix3d::Identity()*w_sigmaInv*w_sigmaInv);//��Ϣ����

		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		e->setRobustKernel(rk);
		rk->setDelta(MAX_EDGE_ERROR);//����³���˺���������������ߵ����

		//���õ�ͼ����������
		e->_x3D[0] = mapPoint->x;
		e->_x3D[1] = mapPoint->y;
		e->_x3D[2] = mapPoint->z;

		//���õ�ǰ֡������ڲ�
		e->fx = pFrame->getCamera()->Kl().at<double>(0, 0);
		e->fy = pFrame->getCamera()->Kl().at<double>(1, 1);
		e->cx = pFrame->getCamera()->Kl().at<double>(0, 2);
		e->cy = pFrame->getCamera()->Kl().at<double>(1, 2);



		e->computeError();//����ߵ����_error���������ϵ��3d�����
		const double& error2 = e->error2();
		//if (error2> errorMax)
		//{//�Ż�ǰȥ�����ϴ��
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
		optimizer.addEdge(e);//���Ż�����ӱ�
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
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));//ÿ���Ż�֮ǰ����pFrame֡λ��
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//��ʼ�Ż����Ż���������Ϊits[it]

		MptList::iterator mPItr = mptMatchedList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeProjectXYZRGBDOnlyPose* e = *itr;
			MapPoint3d* mapPoint = *mPItr;

			const double& error2 = e->error2();		
			/*else
			{����ԭ
			e->setLevel(0);
			}*/
			//if (error2 > errorMax || mapPoint->isBad())
			//{//mapPoint���������ֲ߳̾�BA���ж�Ϊbad
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
					//e->computeError();//����ߵ����_error����ͶӰ���
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
		//	break;//ֱ��return��bug(δ����)

		double totalError2 = 0.0;
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
		{
			if ((*itr)->level() == 0)
				totalError2 += (*itr)->error2();
		}
		RMSE = sqrt(totalError2 / validMathcedCount);
		if (RMSE > MAX_EDGE_RMSE)
		{//����ߵ�RMSE��������������ߣ������Ż�
			worstEdge->setLevel(1);//���Գ������ý�С����Ϣ����
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
				e->computeError();//����δ�����Ż��ı����_error�������Ż��ı�����Ҫ��ʽ���ü��㺯��
				std::cout << "Outlier " << mPItr->first->getID()
						  << " error:" << sqrt(e->error2()) << std::endl;
			}
		}
	}
	std::cout << "Inliers size:" << validMathcedCount
			  << ", Pose Optimizing error:" << RMSE
			  << std::endl;
	////////�����Ż����Ȳ��ߵ�֡(һ��Ϊ����㵼��)////////////////
	//if (RMSE > 0.15)
	//{
	//	validMathcedCount = 0;
	//	//return validMathcedCount;//ֱ��return��bug(δ����)
	//}
	//if (validMathcedCount > 3)
	//{
	//	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//��ȡ�Ż����Ķ��㣺pFrame֡��λ��
	//	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	//	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	//	pFrame->setPose(pose);//����pFrame֡��λ��
	//}
	
	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//��ȡ�Ż����Ķ��㣺pFrame֡��λ��
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->setPose(pose);//����pFrame֡��λ��

	return validMathcedCount;//�����ڵ�����=pFrame֡��ͼ������-��Ⱥ������
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
	optimizer.addVertex(vSE3); //���Ż�����Ӷ��㣺pFrame֡��λ��

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
	{//����frame��ƥ���ͼ��
		MapPoint3d* mapPoint = itr->first;//frameƥ���ͼ��
		if (mapPoint->isBad())
			continue;//�����ھֲ�BA���߳�ȷ���Ĵ����ͼ��
		KeyPoint* keyPointleft = itr->second.first; //frameƥ���ͼ������Ӧ��ͼ��������

		nInitialCorrespondences++;

		Eigen::Matrix<double, 2, 1> obs;
		obs << keyPointleft->_undistortX, keyPointleft->_undistortY;//���ù۲�ֵΪpFrame��ͼ������Ӧ��ͼ�����������������

		EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e->setMeasurement(obs);

		//const double w_sigmaInv = 1.0;
		double w_sigmaInv = itr->second.first->_weight*itr->second.second->_weight;// 1.0; itr->second.first->_weight * itr->second.second->_weight;
		e->setInformation(Eigen::Matrix2d::Identity()*w_sigmaInv*w_sigmaInv);


		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		e->setRobustKernel(rk);
		rk->setDelta(MAX_EDGE_ERROR);//����³���˺���������������ߵ����

		//���õ�ͼ����������
		e->Xw[0] = mapPoint->x;
		e->Xw[1] = mapPoint->y;
		e->Xw[2] = mapPoint->z;

		//���õ�ǰ֡������ڲ�
		e->fx = pFrame->getCamera()->Kl().at<double>(0, 0);
		e->fy = pFrame->getCamera()->Kl().at<double>(1, 1);
		e->cx = pFrame->getCamera()->Kl().at<double>(0, 2);
		e->cy = pFrame->getCamera()->Kl().at<double>(1, 2);


		e->computeError();//����ߵ����_error���������ϵ��3d�����
		const double& error2 = e->error().dot(e->error());
		//if (error2> errorMax)
		//{//�Ż�ǰȥ�����ϴ��
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
			e->computeError();//����ߵ����_error����ͶӰ���
			std::cout << "Edge 2D " << mapPoint->getID() << " error:" << sqrt(e->error().dot(e->error()))
				      << " 3D error:" << errLeft3D << std::endl;
		}
		optimizer.addEdge(e);//���Ż�����ӱ�
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
		vSE3->setEstimate(Converter::toSE3Quat(pFrame->_Tcw));//ÿ���Ż�֮ǰ����pFrame֡λ��
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterCount);//��ʼ�Ż����Ż���������Ϊits[it]

		MptList::iterator mPItr = mptMatchedList.begin();
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++, mPItr++)
		{
			EdgeSE3ProjectXYZOnlyPose* e = *itr;

			MapPoint3d* mapPoint = *mPItr;

			const double& error2 = e->error().dot(e->error());
			/*else
			{����ԭ
			e->setLevel(0);
			}*/
			//if (error2 > errorMax || mapPoint->isBad())
			//{//mapPoint���������ֲ߳̾�BA���ж�Ϊbad
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
		//	break;//ֱ��return��bug(δ����)

		double totalError2 = 0.0;
		for (EdgeList::iterator itr = vpEdges.begin(); itr != vpEdges.end(); itr++)
		{
			if ((*itr)->level() == 0)
				totalError2 += (*itr)->error().dot((*itr)->error());
		}
		RMSE = sqrt(totalError2 / validMathcedCount);
		if (RMSE > MAX_EDGE_RMSE)
		{//����ߵ�RMSE��������������ߣ������Ż�
			worstEdge->setLevel(1);//���Գ������ý�С����Ϣ����
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
				e->computeError();//����δ�����Ż��ı����_error�������Ż��ı�����Ҫ��ʽ���ü��㺯��
			std::cout << "Outlier " << mPItr->first->getID()
				      << " error:" << sqrt(e->error().dot(e->error())) << std::endl;
		}		
	}
	std::cout << "Inliers size:" << validMathcedCount
			  << ", Pose Optimizing error:" << RMSE
			  << std::endl;
	////////�����Ż����Ȳ��ߵ�֡(һ��Ϊ����㵼��)////////////////
	//if (RMSE > 0.15)
	//{
	//	validMathcedCount = 0;
	//	//return validMathcedCount;//ֱ��return��bug(δ����)
	//}

	g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));//��ȡ�Ż����Ķ��㣺pFrame֡��λ��
	g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
	cv::Mat pose = Converter::toCvMat(SE3quat_recov);
	pFrame->setPose(pose);//����pFrame֡��λ��

	return validMathcedCount;//�����ڵ�����=pFrame֡��ͼ������-��Ⱥ������
}





void Optimizer::localKFBundleAdjustment3D(KeyFrame* pCurKF, GlobalMap* pMap, bool& bAbort)
{

	KeyFrameList localKFList;
	MptList localMptList;

	typedef std::vector<EdgeProjectXYZRGBD*> EdgeList;

	localKFList.emplace_back(pCurKF);//��ֲ��ؼ�֡����lLocalKeyFrames��ӵ�ǰ�ؼ�֡
	pCurKF->_localForKF = pCurKF->getID();

	const KeyFrameList& neighborList = pCurKF->getBestCovisibleKeyFrames(5);
	//const KeyFrameList& neighborList = pCurKF->_orderedConnectedKFs;
	for (KeyFrameList::const_iterator itr = neighborList.begin(); itr != neighborList.end(); itr++)
	{
		localKFList.emplace_back(*itr);//localKFList���ֲ��ؼ�֡���ϣ��뵱ǰ֡������������ǰ5֡
		(*itr)->_localForKF = pCurKF->getID();
	}

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�������������߳�ɾ����ͼ��

	// �ֲ��ؼ�֡�۲�����е�ͼ��
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
						localMptList.emplace_back(pMP);//localMapPoints���ֲ���ͼ�㼯�ϣ��ֲ��ؼ�֡�۲�����е�ͼ��
						pMP->_localForKF = pCurKF->getID();
					}
		}
	}

	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//��ס��ͼ�㣬�������̸߳��µ�ͼ��Ĺ۲�֡

	std::vector<KeyFrame*> fixedCameras;
	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		KFSet& observationKFs = (*pit)->getObservationKFs();
		for (KFSet::iterator fit = observationKFs.begin(); fit != observationKFs.end(); fit++)
		{
			if ((*fit)->_localForKF != pCurKF->getID() && (*fit)->_fixedForKF != pCurKF->getID())//����_localForKF��־�����ֲ�����ؼ�֡�������ظ���Ӷ��㵼���Ż�������
			{
				(*fit)->_fixedForKF = pCurKF->getID();

				fixedCameras.emplace_back((*fit));//lFixedCameras���۲쵽�ֲ���ͼ�㵫�����ھֲ��ؼ�֡�Ĺؼ�֡���ϣ��Ż��б���λ�˲���
			}
		}
	}	
	// �����Ż���
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);
	optimizer.setForceStopFlag(&bAbort);

	// ���õ�ͼ�㶥��
	unsigned long maxKFid = 0;
	for (KeyFrameList::iterator fit = localKFList.begin(); fit != localKFList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		vSE3->setId((*fit)->getID());
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//��Ӿֲ��ؼ�֡��λ����Ϊ����
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
		optimizer.addVertex(vSE3);//���lFixedCameras�ؼ�֡λ����Ϊ����
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
		optimizer.addVertex(vPoint);//��Ӿֲ���ͼ��pMP��Ϊ����

		const KFSet& observationsKFs = pMP->getObservationKFs();//���̻߳�ı�۲�֡���ϣ������
		

		//Set edges
		for (KFSet::const_iterator kFit = observationsKFs.begin(); kFit != observationsKFs.end(); kFit++)
		{//�����ֲ���ͼ��pMP�Ĺ۲�ؼ�֡
			KeyFrame* pKF = *kFit;//��ȡ�ֲ���ͼ��pMP�Ĺ۲�֡obsKF
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

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ñߵĶ���Ϊ�ֲ���ͼ��pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//���ñߵĶ���ΪpMP�Ĺ۲�֡obsKFλ��

			e->setMeasurement(obs);//���ñߵĲ���ֵ
			//const double &invSigma2 = 1.0;
			KeyPointPair kptPair;
			if (!pKF->getKeyPairFromMpt(pMP, kptPair))
				return;
			const double wSigmaInv = kptPair.first->_weight* kptPair.second->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix3d::Identity()*wSigmaInv*wSigmaInv);//��Ϣ����

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//����³���˺���������������ߵ����
			//��ɢ�߲�����ֲ�BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//������߲������Ż�

			optimizer.addEdge(e);//��ӱ�
			vpEdges.emplace_back(e);
			vpEdgeKFs.emplace_back(pKF);
			vpMapPointEdge.emplace_back(pMP);
		}
	}
	lockMptObs.unlock();
	lockEraseMpt.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//�������߳�poseOptimization3D�����߳�localBundleAdjustment3D��ͻ
	

	//////////////��һ�־ֲ�BA///////////////
	if (bAbort)
		return;
	optimizer.initializeOptimization(0);
	optimizer.optimize(5);

	for (size_t i = 0, iend = vpEdges.size(); i<iend; i++)
	{//	ȥ����ɢ��
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


	
	/////////////////////���µڶ��־ֲ�BA//////////////////////	
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


	// ȥ����ɢ��ͼ�����ɢ�۲�֡
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
	//{//BA��ͨ����ͼ����ɢ�۲�֡���жϾֲ���ͼ���Ƿ��ǻ���
	//	MapPoint3d* pMP = vpMapPointEdge[i];
	//	if (pMP->isBad())
	//		continue;
	//	if ((float)pMP->_outLierCount / (float)pMP->obsKF() >= 0.5)
	//{
	//      pMP->setBad(true);
	//      pMap->insertBadMpt((*g_mItr));
	//}
	//		
	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//����֡ɾ���;ֲ�BA��ͬһ�߳�
	//if (!vToErase.empty())
	//{
	//	for (auto itr= vToErase.begin(); itr!=vToErase.end(); itr++)
	//	{
	//		KeyFrame* obsKF = itr->first;
	//		MapPoint3d* pMPi = itr->second;
	//		obsKF->eraseMptMatchedWithP3D(pMPi);//��Ӧ�۲�ؼ�֡֡ɾ����ƥ���ͼ�㣬������һ�β���ֲ�BA
	//		obsKF->getRef()->eraseMptMatchedWithP3D(pMPi); //��Ӧ�۲�֡ɾ����ƥ���ͼ��
	//		//obsKF->eraseMptMatched(pMPi);//��Ӧ�۲�ؼ�֡֡ɾ����ƥ���ͼ�㣬������һ�β���ֲ�BA
	//		//obsKF->getRef()->eraseMptMatched(pMPi); //��Ӧ�۲�֡ɾ����ƥ���ͼ��
	//	}
	//}

	///////////////////���¼���ֲ���ͼ��(�����Ż�ʱ���̻߳�ɾ����ͼ��)///////////////////


	//���¾ֲ���ͼ������

	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{//�����ֲ���ͼ��λ��
		MapPoint3d* pMP = *lit;
		if (pMP->isBad())
			continue;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID() + maxKFid + 1));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);		

		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}

	//////////////���¾ֲ��ؼ�֡���ڽ�֡�͵�ͼ��λ��//////////////
	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());

	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{//�����ֲ��ؼ�֡λ��
		KeyFrame* pKF = *lit;
		////////////////////////////�����ֲ��ؼ�֡���ڽ�֡λ��////////////////////////////
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->getID()));
		cv::Mat TcwCorrected = Converter::toCvMat(vSE3->estimate());
		pKF->setPose(TcwCorrected);//����pKFλ��	
		pKF->updateLocalPoses();//����pKF�ڽ�֡λ��
	}
	
	//������֡λ�˼���ͼ���������۲�֡�Ƿ�Ϊ��ɢƥ��
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

	//����֤�����̵߳����굱ǰ�ؼ�֡�;ֲ�֡λ�˺�������������
	//1�����߳�����֡��δ����ǰ֡���Ƴ�λ�ˣ���������λ��һ���Ǹ������߳������޸Ĺ���ǰ֡λ��Ϊ�ο�
	//2�����߳�����֡�Ѹ���ǰ֡ԭʼλ�˹��Ƴ�λ���Ҽ���ȫ�ֵ�ͼ���������߳�BA�����˾ֲ�֡��λ�˵���
	//3�����߳�����֡�ѹ���λ���Ҵ���Ϊ�ؼ�֡�������֡��Ӳ�����ȫ�ֵ�ͼ
	//4�����߳�����֡�ѹ���λ���Ҵ���Ϊ�ؼ�֡�������֡��Ӳ�����ȫ�ֵ�ͼ,LocalMapping�ѽ����ùؼ�֡��pKF�Ĺ����ϵ

}




void Optimizer::localKFBundleAdjustment2D(KeyFrame* keyFrame, GlobalMap* pMap, bool& bAbort)
{
	KeyFrameList localKFList;
	MptList localMptList;
	typedef std::vector<EdgeSE3ProjectXYZ*> EdgeList;

	localKFList.emplace_back(keyFrame);//��ֲ��ؼ�֡����lLocalKeyFrames��ӵ�ǰ�ؼ�֡
	keyFrame->_localForKF = keyFrame->getID();

	const KeyFrameList& neighborList = keyFrame->getBestCovisibleKeyFrames(5);
	//const KeyFrameList& neighborList = keyFrame->_orderedConnectedKFs;
	for (KeyFrameList::const_iterator itr = neighborList.begin(); itr != neighborList.end(); itr++)
	{
		localKFList.emplace_back(*itr);//localKFList���ֲ��ؼ�֡���ϣ��뵱ǰ֡������������ǰ5֡
		(*itr)->_localForKF = keyFrame->getID();
	}

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�������������߳�ɾ����ͼ��

	// �ֲ��ؼ�֡�۲�����е�ͼ��
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
						localMptList.emplace_back(pMP);//localMapPoints���ֲ���ͼ�㼯�ϣ��ֲ��ؼ�֡�۲�����е�ͼ��
						pMP->_localForKF = keyFrame->getID();
					}
		}
	}

	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//��ס��ͼ�㣬�������̸߳��µ�ͼ��Ĺ۲�֡

	std::vector<KeyFrame*> fixedCameras;
	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		KFSet& observations = (*pit)->getObservationKFs();
		for (KFSet::iterator fit = observations.begin(); fit != observations.end(); fit++)
		{
			if ((*fit)->_localForKF != keyFrame->getID() && (*fit)->_fixedForKF != keyFrame->getID())//����_localForKF��־�����ֲ�����ؼ�֡�������ظ���Ӷ��㵼���Ż�������
			{
				(*fit)->_fixedForKF = keyFrame->getID();

				fixedCameras.emplace_back((*fit));//fixedCameras���۲쵽�ֲ���ͼ�㵫�����ھֲ��ؼ�֡�Ĺؼ�֡���ϣ��Ż��б���λ�˲���
			}
		}
	}

	// �����Ż���
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);

	optimizer.setForceStopFlag(&bAbort);

	// ���õ�ͼ�㶥��
	unsigned long maxKFid = 0;
	for (KeyFrameList::iterator fit = localKFList.begin(); fit != localKFList.end(); fit++)
	{
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat((*fit)->getPose()));
		vSE3->setId((*fit)->getID());
		vSE3->setFixed((*fit)->getID() == 0);
		optimizer.addVertex(vSE3);//��Ӿֲ��ؼ�֡��λ����Ϊ����
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
		optimizer.addVertex(vSE3);//���lFixedCameras�ؼ�֡λ����Ϊ����
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
		optimizer.addVertex(vPoint);//��Ӿֲ���ͼ��pMP��Ϊ����

		const KFSet& observationsKF = pMP->getObservationKFs();//���̻߳�ı�۲�֡���ϣ������

		//Set edges
		for (KFSet::const_iterator kFit = observationsKF.begin(); kFit != observationsKF.end(); kFit++)
		{//�����ֲ���ͼ��pMP�Ĺ۲�ؼ�֡
			KeyFrame* pKF = *kFit;//��ȡ�ֲ���ͼ��pMP�Ĺ۲�֡obsKF
			const KeyPoint* keyPointLeft = pKF->_matchedMptMap.find(pMP)->second.first;//��ȡ�ֲ���ͼ��pMP��pKF�ϵ�������
			const KeyPoint* keyPointRight = pKF->_matchedMptMap.find(pMP)->second.second;//��ȡ�ֲ���ͼ��pMP��pKF�ϵ�������								


			Eigen::Matrix<double, 2, 1> obs;
			obs << keyPointLeft->_undistortX, keyPointLeft->_undistortY;//���ù۲�ֵΪ��ͼ��pMP��obsKF֡��ͶӰ������

			EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ñߵĶ���Ϊ�ֲ���ͼ��pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//���ñߵĶ���ΪpMP�Ĺ۲�֡obsKFλ��

			e->setMeasurement(obs);//���ñߵĲ���ֵ
								   //const double &invSigma2 = 1.0;

			const double wSigmaInv = keyPointLeft->_weight* keyPointRight->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix2d::Identity()*wSigmaInv*wSigmaInv);//��Ϣ����

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//����³���˺���������������ߵ����
											//��ɢ�߲�����ֲ�BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//������߲������Ż�

			////////////////����pKFi֡������ڲ�///////////////
			e->fx = pKF->getCamera()->Kl().at<double>(0, 0);
			e->fy = pKF->getCamera()->Kl().at<double>(1, 1);
			e->cx = pKF->getCamera()->Kl().at<double>(0, 2);
			e->cy = pKF->getCamera()->Kl().at<double>(1, 2);

			optimizer.addEdge(e);//��ӱ�
			vpEdges.emplace_back(e);
			vpEdgeKFs.emplace_back(pKF);
			vpMapPointEdge.emplace_back(pMP);

		}
	}
	lockMptObs.unlock();
	lockEraseMpt.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//�������߳�poseOptimization3D�����߳�localBundleAdjustment3D��ͻ


	//////////////��һ�־ֲ�BA///////////////
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


		/////////////////////���µڶ��־ֲ�BA//////////////////////	
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
	{//���¼���ÿ���ֲ���ͼ�����ɢ�۲�֡��
		MapPoint3d* pMP = vpMapPointEdge[i];
		pMP->_outLierCount = 0;
	}
	// ȥ����ɢ��ͼ�����ɢ�۲�֡
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
	{//BA��ͨ����ͼ����ɢ�۲�֡���жϾֲ���ͼ���Ƿ��ǻ���
		MapPoint3d* pMP = vpMapPointEdge[i];
		if ((float)pMP->_outLierCount / (float)pMP->obsKF() >= 0.5)
		{
			pMP->setBad(true);
			pMap->insertBadMpt(pMP);
		}
			
	}

	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//����֡ɾ���;ֲ�BA��ͬһ�߳�

	if (!vToErase.empty())
	{
		for (auto itr = vToErase.begin(); itr != vToErase.end(); itr++)
		{
			KeyFrame* obsKF = itr->first;
			MapPoint3d* pMPi = itr->second;
			obsKF->eraseMptMatchedWithP3D(pMPi);//��Ӧ�۲�ؼ�֡֡ɾ����ƥ���ͼ�㣬������һ�β���ֲ�BA
		}
	}

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());
	lockEraseMpt.lock();//�������߳�ɾ����ͼ��

	//���¾ֲ��ؼ�֡��֡�͵�ͼ��λ��
	for (KeyFrameList::iterator lit = localKFList.begin(), lend = localKFList.end(); lit != lend; lit++)
	{
		KeyFrame* pKF = *lit;

		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->getID()));
		cv::Mat TcwCorrected = Converter::toCvMat(vSE3->estimate());
		pKF->setPose(TcwCorrected); //����pKFλ��

		pKF->updateLocalPoses();//����pKF�ڽ�֡λ��


	}

	lockTrack.unlock();
			 

    ///////////////////���¼���ֲ���ͼ��(�����Ż�ʱ���߳̿���ɾ����ͼ��)///////////////////
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
					localMptList.emplace_back(pMP);//��ֲ���ͼ�㼯��localMapPoints��Ӿֲ��ؼ�֡��ƥ���ͼ��
					pMP->_localForKF = -1;
				}

		}
	}
	//���¾ֲ���ͼ������
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
	localFrameList.emplace_back(currentFrame);//��ֲ��ؼ�֡����lLocalKeyFrames��ӵ�ǰ�ؼ�֡
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
						localMptList.emplace_back(pMP);//��ֲ���ͼ�㼯��localMapPoints��Ӿֲ��ؼ�֡��ƥ���ͼ��
						pMP->_localForKF = currentFrame->getID();
					}
		}
	}
	// �����Ż���
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);

	// ���õ�ͼ�㶥��
	unsigned long maxMptId = 0;

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		vPoint->setId(pMP->getID());
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//��Ӿֲ���ͼ��pMP��Ϊ����
		if (pMP->getID() > maxMptId)
			maxMptId = pMP->getID();
	}

	// ����֡λ�˶���
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
		optimizer.addVertex(vSE3);//��Ӿֲ��ؼ�֡��λ����Ϊ����


		MptKeyPairMap& mapPointList = (*fit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mItr = mapPointList.begin(); mItr != mapPointList.end(); mItr++)
		{
			MapPoint3d* pMP = mItr->first;
			const KeyPoint* keyPointLeft = (*fit)->_matchedMptMap.find(pMP)->second.first;//��ȡ�ֲ���ͼ��pMP��obsFrame�ϵ�������
																							 //��ȡ�ؼ�֡pKFi�Ľ���������kpUn


			Eigen::Matrix<double, 2, 1> obs;
			obs << keyPointLeft->_undistortX, keyPointLeft->_undistortY;//���ù۲�ֵΪ��ͼ��pMP��obsKF֡��ͶӰ������

			EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pMP->getID())));//���ñߵĶ���Ϊ�ֲ���ͼ��pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ñߵĶ���ΪpMP�Ĺ۲�֡obsFrameλ��
			e->setMeasurement(obs);//���ñߵĲ���ֵ
			const double &invSigma2 = 1.0;
			e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);

			////////////////����pKFi֡������ڲ�///////////////
			e->fx = (*fit)->getCamera()->Kl().at<double>(0, 0);
			e->fy = (*fit)->getCamera()->Kl().at<double>(1, 1);
			e->cx = (*fit)->getCamera()->Kl().at<double>(0, 2);
			e->cy = (*fit)->getCamera()->Kl().at<double>(1, 2);

			optimizer.addEdge(e);//��ӱ�
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
		optimizer.addVertex(vSE3);//���lFixedCameras�ؼ�֡λ����Ϊ����
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

	// ȥ����ɢ��ͼ�����ɢ�۲�֡    
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

	//���¾ֲ���ͼ������
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID()));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);

		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>1.0)
			pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}

	//���¾ֲ�֡λ��
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

	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�������������߳�ɾ����ͼ��

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
						localMptList.emplace_back(pMP);//��ֲ���ͼ�㼯��localMapPoints��Ӿֲ��ؼ�֡��ƥ���ͼ��
						//pMP->_localForKF = keyFrame->getID();
						pMP->_localForKF = currentFrame->getID();
					}
		}
	}

	// �����Ż���
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	//optimizer.setVerbose(true);


	std::unique_lock<std::mutex> lockMptObs(MapPoint3d::_obsMutex);//��ס��ͼ�㣬�������̸߳��µ�ͼ��Ĺ۲�֡

	// ���õ�ͼ�㶥��
	unsigned long maxMptId = 0;

	for (MptList::iterator pit = localMptList.begin(); pit != localMptList.end(); pit++)
	{
		MapPoint3d* pMP = *pit;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		vPoint->setId(pMP->getID());
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//��Ӿֲ���ͼ��pMP��Ϊ����
		if (pMP->getID() > maxMptId)
			maxMptId = pMP->getID();
	}

	// ����֡λ�˶���
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
		optimizer.addVertex(vSE3);//��Ӿֲ��ؼ�֡��λ����Ϊ����


	    MptKeyPairMap& mapPointList = (*fit)->_matchedMptMap;//���̻߳�ı��֡��ƥ���ͼ�㼯�ϣ������
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

			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ñߵĶ���ΪpMP�Ĺ۲�֡obsFrameλ��
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pMP->getID())));//���ñߵĶ���Ϊ�ֲ���ͼ��pMP
			
			e->setMeasurement(obs);//���ñߵĲ���ֵ
			const double &invSigma2 = 1.0;
			e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuberMono);

			optimizer.addEdge(e);//��ӱ�
			vpEdges.emplace_back(e);
			vpEdgeFrames.emplace_back((*fit));
			vpMapPointEdge.emplace_back(pMP);
		}

	}

	lockMptObs.unlock();
	//std::unique_lock<std::mutex> lockOptimize(_localOptimizeMutex);//�������߳�poseOptimization3D�����߳�localBundleAdjustment3D��ͻ


	optimizer.initializeOptimization();
	optimizer.optimize(1);//��ֵҪ����

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
		optimizer.optimize(10);//���ܹ�С

	}

	std::cout << "Edge size:" << optimizer.edges().size() << ", Local optimization square error:" << sqrt(optimizer.activeChi2() / optimizer.edges().size()) << std::endl;
	

	//std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);
	
	std::vector<std::pair<Frame*, MapPoint3d*> > vToErase;
	vToErase.reserve(vpEdges.size());

	// ȥ����ɢ��ͼ�����ɢ�۲�֡    
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

	//���¾ֲ���ͼ������
	for (MptList::iterator lit = localMptList.begin(), lend = localMptList.end(); lit != lend; lit++)
	{
		MapPoint3d* pMP = *lit;
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->getID()));
		pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
		pMP->_coordLast= *dynamic_cast<cv::Point3d*>(pMP);
		
		//if(cv::norm(pMP->_coordLast-pMP->_coordOri)>1.0)
		//	pMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);
		
	}

	//���¾ֲ�֡λ��
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
pCurKF:β���ؼ�֡
pLoopKF:�ײ�ƥ��ؼ�֡
correctedSim3:β�����򹲵�֡�������λ��
nonCorrectedSim3:β�����򹲵�֡��ԭʼλ��
loopConnections:��--ֵ = β�����򹲵�֡--�ùؼ�֡���ײ�����ؼ�֡����
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
//	const KeyFrameList keyFrameList = pMap->getKeyFrameList();//LocalMapping�ֲ߳̾�BA���ؼ�֡ɾ����ֹͣ
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
//		{//Sim3s����β������֡�ѽ���λ��
//			cv::Mat Tcw = itr->second;
//			g2o::Sim3 Sim= g2o::Sim3(Converter::toMatrix3d(Tcw.colRange(0, 3).rowRange(0, 3)),
//								     Converter::toVector3d(Tcw.col(3).rowRange(0, 3)),
//								     1.0);
//			Sim3s[pKF->_id] = Sim;
//			vSim3->setEstimate(Sim);
//		}
//		else
//		{//Sim3s����ͷ������֡ԭʼλ��
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
//	{//����β�����򹲵�ؼ�֡pBackKF
//		KeyFrame* pBackKF = mit->first;
//		const std::set<KeyFrame*> &frontConnections = mit->second;
//		const g2o::Sim3 Sbw = Sim3s[pBackKF->_id];//β��pBackKFλ�ˣ��ѽ�����
//		const g2o::Sim3 Swb = Sbw.inverse();
//
//		for (std::set<KeyFrame*>::const_iterator frontItr = frontConnections.begin(), send = frontConnections.end(); frontItr != send; frontItr++)
//		{//����β��pBackKF���ײ�����ؼ�֡
//			KeyFrame* pFrontKF = *frontItr;
//			if ((pBackKF->_id != pCurKF->_id || pFrontKF->_id != pLoopKF->_id)&& pBackKF->getKFCovisibleCount(pFrontKF)<3)
//				continue;
//
//			const g2o::Sim3 Sfw = Sim3s[pFrontKF->_id];//pBackKF�ײ�����֡λ�ˣ�ԭʼ��
//			const g2o::Sim3 Sfb = Sfw * Swb;//Sji��β��pBackKF(�ѽ���)->�ײ�����֡pFrontKF(ԭʼ)
//
//			g2o::EdgeSim3* e = new g2o::EdgeSim3();
//			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pFrontKF->_id)));//����1��pBackKF�ײ�����ؼ�֡��δ������
//			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pBackKF->_id)));//����0��β������ؼ�֡pBackKF���ѽ�����
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
//	{//����ȫ�ֹؼ�֡pKF
//		KeyFrame* pKF = keyFrameList[i];
//
//		//Swi��ȫ��pKFλ��(ԭʼ)
//		g2o::Sim3 Swi;		
//		LoopClosing::KeyFrameAndPose::const_iterator itr = nonCorrectedSim3.find(pKF);
//		if (itr != nonCorrectedSim3.end())
//		{//pKF����β������֡��ԭʼ��
//			cv::Mat Tiw = itr->second;
//			g2o::Sim3 Siw = g2o::Sim3(Converter::toMatrix3d(Tiw.colRange(0, 3).rowRange(0, 3)),
//								      Converter::toVector3d(Tiw.col(3).rowRange(0, 3)),
//								      1.0);	
//			Swi = Siw.inverse();
//		}	
//		else//pKF��β������֡��ԭʼ��
//			Swi = Sim3s[pKF->_id].inverse();
//
//		
//		if (pKF->_maxPrevConnKF)
//		{
//			KeyFrame* pKFj = pKF->_maxPrevConnKF;//pKFj��ǰ����󹲵�֡	
//			if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id))))
//				continue;//������Ӹñߣ�β��pKF->pKFͷ������ػ�֡��������
//
//			g2o::Sim3 Sjw;  //Sjw��pKFǰ����󹲵�֡pMaxPrevKFλ�ˣ�ԭʼ��
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
//			g2o::Sim3 Sji = Sjw * Swi;//pKF��ԭʼ��->pKFǰ����󹲵�֡��ԭʼ��
//
//			g2o::EdgeSim3* e = new g2o::EdgeSim3();
//			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFj->_id))); //pKFǰ����󹲵�֡��ԭʼ��
//			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF��ԭʼ��
//			e->setMeasurement(Sji);
//
//			e->information() = matLambda;
//			optimizer.addEdge(e);
//		}
//
//		// Loop edges
//		const  KFSet& loopKFs = pKF->getLoopKFs();
//		if (!loopKFs.empty())
//		{//���pKF����ʷƥ��ػ�֡
//			for (auto sit = loopKFs.begin(), send = loopKFs.end(); sit != send; sit++)
//			{//����pKF��ʷ�ػ�ƥ��֡�������ڣ�
//				KeyFrame* pLoopKF = *sit;
//				if (pLoopKF->_id<pKF->_id)
//				{
//					//Slw��pKF��ʷ�ػ�ƥ��֡λ��(ԭʼ)
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
//					g2o::Sim3 Sli = Slw * Swi;//pKF��ԭʼ��->pKFƥ��ػ�֡��ԭʼ��
//					g2o::EdgeSim3* el = new g2o::EdgeSim3();
//					el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLoopKF->_id)));//pKFƥ��ػ�֡(ԭʼ)
//					el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKF��ԭʼ��
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
//		{//����pKF����֡pKFn
//			KeyFrame* pKFn = *itr;
//			if (pKFn && pKFn != pKF->_maxPrevConnKF && !pKF->hasNextMaxKF(pKFn)&& !loopKFs.count(pKFn))
//			{//����pKF��pKF�ĸ��ڵ㡢pKF�ĺ��ӽڵ㡢�ػ�ƥ��֡
//				if (! pKFn->_id<pKF->_id)
//				{//pKFn��pKF֮ǰ
//					if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id))))
//						continue;//������Ӹñߣ�β��pKF->pKFͷ������ػ�֡��������
//
//					//Snw��pKFʣ�๲��ؼ�֡λ�ˣ�ԭʼ��
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
//					g2o::Sim3 Sni = Snw * Swi;//Sni��pKF��ԭʼ��->pKF����֡pKFn��ԭʼ�����λ��
//
//					g2o::EdgeSim3* en = new g2o::EdgeSim3();
//					en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->_id)));//pKF��ԭʼ��
//					en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKFʣ�๲��֡pKFn(ԭʼ)
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
//	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//��ͣ���̸߳�������֡
//	std::unique_lock<std::mutex> lockEraseFrame(GlobalMap::_frameMutex);//��ͣɾ���ؼ�֡
//	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//��ͣɾ����ͼ��
//	KeyFrameList& KFList = pMap->getKeyFrameList();//�Ż���Ĺؼ�֡���ϣ��Ż��п�����������٣�
//	const GlobalMap::MptList mapPointList = pMap->getMapPointList();//�����ã����̺߳����������ӵ�ͼ��
//
//	int KFsize = KFList.size();
//	for (size_t i = 0; i<KFsize; i++)
//	{
//		KeyFrame* pKFi = KFList[i];
//		if (pKFi->_id <= maxKFid)
//		{//�Ż�ǰ�ؼ�֡
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
//		{//�����ؼ�֡
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
	// �����Ż���
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> linearSolver = std::make_unique< g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	solver->setUserLambdaInit(1e-16);
	optimizer.setAlgorithm(solver);

	const KeyFrameList keyFrameList = pMap->getKeyFrameList();//LocalMapping�ֲ߳̾�BA���ؼ�֡ɾ����ֹͣ

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
		{//Sim3s����β������֡�ѽ���λ��
			cv::Mat Tcw = itr->second;
			g2o::Sim3 Sim= g2o::Sim3(Converter::toMatrix3d(Tcw.colRange(0, 3).rowRange(0, 3)),
								     Converter::toVector3d(Tcw.col(3).rowRange(0, 3)),
								     1.0);
			Sim3s[pKF->_id] = Sim;
			vSim3->setEstimate(Sim);
		}
		else
		{//Sim3s����ͷ������֡ԭʼλ��
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
	{//����β�����򹲵�ؼ�֡pBackKF
		KeyFrame* pBackKF = mit->first;
		const std::set<KeyFrame*> &frontConnections = mit->second;
		const g2o::Sim3 Sbw = Sim3s[pBackKF->_id];//β��pBackKFλ�ˣ��ѽ�����
		const g2o::Sim3 Swb = Sbw.inverse();

		for (std::set<KeyFrame*>::const_iterator frontItr = frontConnections.begin(), send = frontConnections.end(); frontItr != send; frontItr++)
		{//����β��pBackKF���ײ�����ؼ�֡
			KeyFrame* pFrontKF = *frontItr;
			if ((pBackKF->_id != pCurKF->_id || pFrontKF->_id != pLoopKF->_id) && pBackKF->getKFCovisibleCount(pFrontKF)<3)
				continue;

			const g2o::Sim3 Sfw = Sim3s[pFrontKF->_id];//pBackKF�ײ�����֡λ�ˣ�ԭʼ��
			const g2o::Sim3 Sfb = Sfw * Swb;//Sji��β��pBackKF(�ѽ���)->�ײ�����֡pFrontKF(ԭʼ)

			g2o::EdgeSim3* e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pFrontKF->_id)));//����1��pBackKF�ײ�����ؼ�֡��δ������
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pBackKF->_id)));//����0��β������ؼ�֡pBackKF���ѽ�����
			e->setMeasurement(Sfb);

			e->information() = matLambda;

			optimizer.addEdge(e);

			insertedEdges.insert(std::make_pair(std::min(pBackKF->_id, pFrontKF->_id), std::max(pBackKF->_id, pFrontKF->_id)));
		}
	}

	for (size_t i = 0, iend = keyFrameList.size(); i<iend; i++)
	{//����ȫ�ֹؼ�֡pKF
		KeyFrame* pKF = keyFrameList[i];

		//Swi��ȫ��pKFλ��(ԭʼ)
		g2o::Sim3 Swi;		
		LoopClosing::KeyFrameAndPose::const_iterator itr = nonCorrectedSim3.find(pKF);
		if (itr != nonCorrectedSim3.end())
		{//pKF����β������֡��ԭʼ��
			cv::Mat Tiw = itr->second;
			g2o::Sim3 Siw = g2o::Sim3(Converter::toMatrix3d(Tiw.colRange(0, 3).rowRange(0, 3)),
								      Converter::toVector3d(Tiw.col(3).rowRange(0, 3)),
								      1.0);	
			Swi = Siw.inverse();
		}	
		else//pKF��β������֡��ԭʼ��
			Swi = Sim3s[pKF->_id].inverse();

		//if (pKF->_maxPrevConnKF)
		//{
		//	KeyFrame* pKFj = pKF->_maxPrevConnKF;//pKFj��ǰ����󹲵�֡	
		//	if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id))))
		//		continue;//������Ӹñߣ�β��pKF->pKFͷ������ػ�֡��������
		//	
		//	g2o::Sim3 Sjw;  //Sjw��pKFǰ����󹲵�֡pMaxPrevKFλ�ˣ�ԭʼ��
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
		//	g2o::Sim3 Sji = Sjw * Swi;//pKF��ԭʼ��->pKFǰ����󹲵�֡��ԭʼ��
		//	
		//	g2o::EdgeSim3* e = new g2o::EdgeSim3();
		//	e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFj->_id))); //pKFǰ����󹲵�֡��ԭʼ��
		//	e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF��ԭʼ��
		//	e->setMeasurement(Sji);
		//	
		//	e->information() = matLambda;
		//	optimizer.addEdge(e);
		//	insertedEdges.insert(std::make_pair(std::min(pKF->_id, pKFj->_id), std::max(pKF->_id, pKFj->_id)));
		//}



		const  KFSet& loopKFs = pKF->getLoopKFs();
		if (!loopKFs.empty())
		{//���pKF����ʷƥ��ػ�֡
			for (auto sit = loopKFs.begin(), send = loopKFs.end(); sit != send; sit++)
			{//����pKF��ʷ�ػ�ƥ��֡�������ڣ�
				KeyFrame* pLoopKF = *sit;
				if (pLoopKF->_id<pKF->_id)
				{
					//Slw��pKF��ʷ�ػ�ƥ��֡λ��(ԭʼ)
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

					g2o::Sim3 Sli = Slw * Swi;//pKF��ԭʼ��->pKFƥ��ػ�֡��ԭʼ��
					g2o::EdgeSim3* el = new g2o::EdgeSim3();
					el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLoopKF->_id)));//pKFƥ��ػ�֡(ԭʼ)
					el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id)));//pKF��ԭʼ��
					el->setMeasurement(Sli);
					el->information() = matLambda;
					optimizer.addEdge(el);
				}
			}
		}
		if (pKF->_parentKF)
		{
			KeyFrame* pKFn = pKF->_parentKF;//pKFj��ǰ����󹲵�֡	
			if (insertedEdges.count(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id))))
				continue;//������Ӹñߣ�β��pKF->pKFͷ������ػ�֡��������

			g2o::Sim3 Snw;  //Snw��pKFǰ�򹲵�ؼ�֡λ�ˣ�ԭʼ��
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

			g2o::Sim3 Sni = Snw * Swi;//pKF��ԭʼ��->pKFǰ����󹲵�֡��ԭʼ��

			g2o::EdgeSim3* e = new g2o::EdgeSim3();
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->_id))); //pKFǰ����󹲵�֡��ԭʼ��
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->_id))); //pKF��ԭʼ��
			e->setMeasurement(Sni);

			e->information() = matLambda;
			optimizer.addEdge(e);

			insertedEdges.insert(std::make_pair(std::min(pKF->_id, pKFn->_id), std::max(pKF->_id, pKFn->_id)));
		}
	}

	optimizer.initializeOptimization();
	optimizer.optimize(20);

	// Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//��ͣ���̸߳�������֡
	std::unique_lock<std::mutex> lockEraseFrame(GlobalMap::_frameMutex);//��ͣɾ���ؼ�֡
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//��ͣɾ����ͼ��
	KeyFrameList& KFList = pMap->getKeyFrameList();//�Ż���Ĺؼ�֡���ϣ��Ż��п�����������٣�
	const MptList mapPointList = pMap->getMapPointList();//�����ã����̺߳����������ӵ�ͼ��

	int KFsize = KFList.size();
	for (size_t i = 0; i<KFsize; i++)
	{
		KeyFrame* pKFi = KFList[i];
		if (pKFi->_id <= maxKFid)
		{//�Ż�ǰ�ؼ�֡
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
		{//�����ؼ�֡
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
{// �����Ż���
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
		optimizer.addVertex(vSE3);//��Ӷ���Ϊ�ؼ�֡λ��
		if (pKF->_id>maxKFid)
			maxKFid = pKF->_id;//maxKFid�������vpKFs�ؼ�֡�����е����ؼ�֡id
	}

	int mptSize = mptList.size();
	for (size_t i = 0; i<mptSize; i++)
	{
		MapPoint3d* pMP = mptList[i];
		if (pMP->isBad())
			continue;
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->getWorldPos()));
		const int id = pMP->_id + maxKFid + 1;//id�����ж���id�����ϵ���
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);//��Ӷ���Ϊ��ͼ��

		const KFSet observationsKF = pMP->getObservationKFs();//��ȡ�۲⵽�õ�ͼ������йؼ�֡����observationsKF

		for (KFSet::const_iterator kFit = observationsKF.begin(); kFit != observationsKF.end(); kFit++)
		{//�����ֲ���ͼ��pMP�Ĺ۲�ؼ�֡
			KeyFrame* pKF = *kFit;//��ȡ�ֲ���ͼ��pMP�Ĺ۲�֡obsKF
			if (pKF->isBad())
				continue;
			if (pKF->_id>maxKFid)
				continue;//�������߳���ȫ��BAʱ�½��Ĺؼ�֡
			if (pKF->_outLiers.count(pMP))
				continue;//������ɢƥ��ص�Լ��
			cv::Point3d* p3d = pKF->getP3DFromMpt(pMP);
			if (!p3d)
			{
				std::cout << "Can't get corresponding 3d point from map point in optimization!" << std::endl;
				return;
			}

			Eigen::Matrix<double, 3, 1> obs;
			obs << p3d->x, p3d->y, p3d->z;

			EdgeProjectXYZRGBD* e = new EdgeProjectXYZRGBD();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ñߵĶ���Ϊ�ֲ���ͼ��pMP
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->getID())));//���ñߵĶ���ΪpMP�Ĺ۲�֡obsKFλ��

			e->setMeasurement(obs);//���ñߵĲ���ֵ
								   //const double &invSigma2 = 1.0;
			KeyPointPair kptPair;
			if (!pKF->getKeyPairFromMpt(pMP, kptPair))
				return;
			const double wSigmaInv = kptPair.first->_weight* kptPair.second->_weight;//kptPair.first->_weight * kptPair.second->_weight; // kptPair.first->_weight * kptPair.second->_weight;
			e->setInformation(Eigen::Matrix3d::Identity()*wSigmaInv*wSigmaInv);//��Ϣ����

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(MAX_BA_EDGE_ERROR);//����³���˺���������������ߵ����
											//��ɢ�߲�����ֲ�BA
			e->computeError();
			//const double& error2 = e->error2();
			//if (error2 > MAX_BA_EDGE_ERROR*MAX_BA_EDGE_ERROR)
			//	e->setLevel(1);//������߲������Ż�

			optimizer.addEdge(e);//��ӱ�
		}

	}

	// ��ʼ�Ż�
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);//��ʼ�Ż�����������ΪnIterations


	for (size_t i = 0; i<KFsize; i++)
	{
		KeyFrame* pKF = KFList[i];
		if (pKF->_id <= maxKFid)
		{//�Ż�ǰ�ؼ�֡
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
		{//�Ż�ǰ��ͼ��
			//pMP->setWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->_posGBA = Converter::toCvMat(vPoint->estimate());
		}		
		
	}
}

}

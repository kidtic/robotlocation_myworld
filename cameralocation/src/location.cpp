#include <cameralocation/location.h>


Location::Location(int casheSec,int camNum,std::string configpath)
{

    imuodom=Imuodom(120*casheSec);
    for (size_t i = 0; i < camNum; i++)
    {
        /* code */
        camera acam(configpath,"camera"+std::to_string(i));
        camInfo.push_back(acam);
    }
     
    
}

Location::~Location()
{
}

Eigen::Vector3d Location::MultiCameraLocation(std::vector<Eigen::Vector2d> Pc)
{
    std::vector<Eigen::Vector3d> K;
    for (size_t i = 0; i < camInfo.size(); i++)
    {
        Eigen::Vector3d ppc;
        ppc<<Pc[i],1.0;
        K.push_back(camInfo[i].RM*ppc);
    }
    //根据IPC个数，定义HQ向量
    Eigen::MatrixXd H; H.setZero(3 * camInfo.size(), 3 + camInfo.size());
    Eigen::VectorXd Q(3 * camInfo.size());
    Eigen::MatrixXd E; E.setIdentity(3,3);
    //赋值HQ向量
    for (size_t n = 0; n <camInfo.size(); n++)
    {
        H.block(n * 3, 0, 3, 3) = -E;
        H.block(n * 3, 3+n, 3, 1) = K[n];
    }

    for (size_t n = 0; n <camInfo.size(); n++)
    {
        Q.block(n * 3, 0, 3, 1) = camInfo[n].RT;
    }

    //广义逆矩阵计算
    Eigen::VectorXd Pw = H.colPivHouseholderQr().solve(Q);

    //
    Eigen::Vector3d ret;
    ret<<Pw[0],Pw[1],Pw[2];
    return ret;
    
}


//定位算法
g2o::SE3Quat Location::imuCameraLocation(const cameralocation::cameraKeyPointConstPtr msg)
{
    
    //解析msg
    std::vector< Eigen::Matrix<double,6,1> > camKP;
    for (size_t i = 0; i < camInfo.size(); i++)
    {
        Eigen::Matrix<double,6,1> pa;
        pa <<   msg->org[2*i],  msg->org[1+2*i], msg->xaxis[2*i], msg->xaxis[1+2*i], msg->yaxis[2*i], msg->yaxis[1+2*i];
        camKP.push_back(pa);  
    }
    

    // -----初始化g2o
   // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,6> > Block;  // pose 维度为 6, landmark 维度为 3
    //Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverCSparse<Block::PoseMatrixType>());

    std::unique_ptr<Block> solver_ptr (new Block( std::move(linearSolver) ));     // 矩阵块求解器

    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // -----vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( imuodom.PQV_to_SE3(imuodom.robotPQV[0]) );
    optimizer.addVertex( pose );

    // -----edges
    int index = 1;
    std::vector<EdgeIMUCameraLocationPoseOnly*> edges;
    for ( size_t i=0; i<camInfo.size(); i++ )
    {
        EdgeIMUCameraLocationPoseOnly* edge = new EdgeIMUCameraLocationPoseOnly(
            camInfo[i],imuodom);
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        //设置测
        edge->setMeasurement( camKP[i] );
        edge->setInformation( Eigen::Matrix<double,6,6>::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    //开始计算
    
    optimizer.setVerbose( false );
    optimizer.initializeOptimization();
    optimizer.optimize(1);
    

    
    return pose->estimate();

}




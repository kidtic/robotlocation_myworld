#include <cameralocation/location.h>


Location::Location(int casheSec,int camNum,std::string configpath)
{

    //imuodom=Imuodom(120*casheSec);
    for (size_t i = 0; i < camNum; i++)
    {
        /* code */
        camera acam(configpath,"camera"+std::to_string(i));
        camInfo.push_back(acam);
    }
    cameraCashe_init();
    
}

Location::~Location()
{
}

void Location::cameraCashe_init()
{
    //比较相机延时大小，选出延时最大的哪一个
    int maxDelay=0,maxDelayIndex;
    for (size_t i = 0; i < camInfo.size(); i++)
    {
        if(camInfo[i].delay_size()>maxDelay)
        {   
            maxDelay=camInfo[i].delay_size();
            maxDelayIndex=i;
        }
    }
    printf("maxDelay:%d \n",maxDelay);

    //初始化缓存
    for (size_t i = 0; i < camInfo.size(); i++)
    {
        int cashesize=maxDelay-camInfo[i].delay_size()+1;
        camInfo[i].robotPixSynch.clear();
        for (size_t j = 0; j < cashesize; j++)
        {
            Eigen::Matrix<double,2,3> newmat;
            camInfo[i].robotPixSynch.push_back(newmat);
        }
        
    }
    //根据最大的相机延时大小来确定imu的缓存大小，确保imu[0]与延时最大的那个相机是同步的。
    
    imuodom=Imuodom(maxDelay*3);

    
}
/*
* 将相机的robot像素点数据push进对应相机的同步缓存。
* 
*/ 
void Location::camInfoSynchCashe_push(int cameraIndex,Eigen::Matrix<double,2,3> input)
{
    camInfo[cameraIndex].robotPixSynch.push_back(input);
    std::vector<Eigen::Matrix<double,2,3>>::iterator e=camInfo[cameraIndex].robotPixSynch.begin();
    camInfo[cameraIndex].robotPixSynch.erase(e);
}
void Location::camInfoSynchCashe_push( cameralocation::cameraKeyPointConstPtr msg)
{
    std::vector<Eigen::Matrix<double,2,3>> input=camera::msg2mat(msg);
    for (size_t i = 0; i < camInfo.size(); i++)
    {
        camInfoSynchCashe_push(i,input[i]);
    }
    
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

g2o::SE3Quat Location::MultiCameraLocation(std::vector<Eigen::Matrix<double ,2,3>> camKP)
{
    std::vector<Eigen::Vector3d> oxy;//robot的坐标轴
    for (size_t i = 0; i < 3; i++)
    {
        std::vector<Eigen::Vector2d> Pc;
        for (size_t j = 0; j < camKP.size(); j++)
        {
            Pc.push_back(Eigen::Vector2d(camKP[j](0,i),camKP[j](1,i)));
        }

        oxy.push_back(MultiCameraLocation(Pc)); 
    }
    //--------进行转化计算，得到SE3
    //计算ox oy向量
    Eigen::Vector3d ox=oxy[1]-oxy[0];
    Eigen::Vector3d oy=oxy[2]-oxy[0];

    double ang_z=atan2(ox[1],ox[0]);
    double c_xynorm=sqrt(ox[0]*ox[0]+ox[1]*ox[1]);
    double ang_y=atan2(ox[2],c_xynorm);
    double ang_x=acos( (-ox[1]*oy[0]+ox[0]*oy[1]) / (c_xynorm* oy.norm()) );

    Eigen::Quaterniond tmpQ;
    tmpQ = Eigen::AngleAxisd(ang_z, Eigen::Vector3d::UnitZ()) * 
            Eigen::AngleAxisd(ang_y, Eigen::Vector3d::UnitY()) * 
            Eigen::AngleAxisd(ang_x, Eigen::Vector3d::UnitX());
    
    g2o::SE3Quat ret=g2o::SE3Quat(tmpQ,oxy[0]);
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




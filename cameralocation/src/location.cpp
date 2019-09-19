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




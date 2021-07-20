#include "walking_controller/pose_optimizer.h"

typedef QuadrupedDescription::LimbList LimbList;
typedef std::unordered_map<LimbList, kindr::Position3D, EnumClassHash> Stance;

void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<kindr::Position3D>& footholds)
{
    footholds.reserve(stance.size());
    std::map<LimbList, kindr::Position3D, CompareByCounterClockwiseOrder> stanceOrdered;
    for (const auto& limb : stance) stanceOrdered.insert(limb);
    for (const auto& limb : stanceOrdered) footholds.push_back(limb.second);
};

PoseOptimizer::PoseOptimizer() {
}


// TODO: 1. footPosition, contactfoot
Eigen::VectorXd PoseOptimizer::optimize(std::array<Eigen::Matrix4f, 4> foot_positions, bool foot_contacts[4], float roll, float pitch, float yaw) {
    A1Adapter adapter;

    // change by paramter
    int nFeet = 4;

    int nDimensions_ = 2;
    int nStates_ = 3;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nDimensions_ * nFeet, nStates_);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(nDimensions_ * nFeet);

    // define pose?
    kindr::HomTransformQuatD pose = kindr::HomTransformQuatD(kindr::Position3D(0.0, 0.0, 0.0), kindr::RotationQuaternionD());
    Eigen::Matrix3d R = kindr::RotationMatrixD(pose.getRotation()).matrix();

    double nominal_height = 0.0;

    // change

    double base_height = 0.28;
    kindr::Position3D base_position(0, 0, base_height);
    Stance nominalStanceInBaseFrame_, footPositions;

    kindr::EulerAnglesZyxPD rotation(roll, pitch, 0);


    Eigen::Matrix<float, 3, 4> leg_positions;
    // leg_positions << foot_positions[0](0, 3)+0.1805, foot_positions[1](0, 3)+0.1805, foot_positions[2](0, 3)-0.1805, foot_positions[3](0, 3)-0.1805,
    //                  foot_positions[0](1, 3)+0.047, foot_positions[1](1, 3)-0.047, foot_positions[2](1, 3)+0.047, foot_positions[3](1, 3)-0.047,
    //                  foot_positions[0](2, 3), foot_positions[1](2, 3), foot_positions[2](2, 3), foot_positions[3](2, 3);

    leg_positions << foot_positions[0](0, 3), foot_positions[1](0, 3), foot_positions[2](0, 3), foot_positions[3](0, 3),
                     foot_positions[0](1, 3), foot_positions[1](1, 3), foot_positions[2](1, 3), foot_positions[3](1, 3),
                     foot_positions[0](2, 3), foot_positions[1](2, 3), foot_positions[2](2, 3), foot_positions[3](2, 3);


    Eigen::Matrix3f rot = rotxyz(roll, pitch, 0);
    
    // for(int i=0; i<4; i++) {
    //     std::cout << "Before " << i << " : " << std::endl << leg_positions.col(i) << std::endl;
    //     leg_positions.col(i) = rot * leg_positions.col(i);
    //     std::cout << "After " << i << " : " << std::endl << leg_positions.col(i) << std::endl;
    // }

    std::cout << "%%" << roll << ", " << pitch << ", " << yaw << std::endl;
    footPositions[LimbList::LF_LEG] = base_position + (kindr::Position3D(leg_positions(0, 0)+0.1805, leg_positions(1, 0)+0.047, leg_positions(2, 0)));
    footPositions[LimbList::RF_LEG] = base_position + (kindr::Position3D(leg_positions(0, 1)+0.1805, leg_positions(1, 1)-0.047, leg_positions(2, 1)));
    footPositions[LimbList::LH_LEG] = base_position + (kindr::Position3D(leg_positions(0, 2)-0.1805, leg_positions(1, 2)+0.047, leg_positions(2, 2)));
    footPositions[LimbList::RH_LEG] = base_position + (kindr::Position3D(leg_positions(0, 3)-0.1805, leg_positions(1, 3)-0.047, leg_positions(2, 3)));
    std::cout << "@@" << footPositions[LimbList::LF_LEG] << std::endl;
    std::cout << "@@" << footPositions[LimbList::RF_LEG] << std::endl;
    std::cout << "@@" << footPositions[LimbList::LH_LEG] << std::endl;
    std::cout << "@@" << footPositions[LimbList::RH_LEG] << std::endl;

    // footPositions[LimbList::LF_LEG] = base_position + rotation.rotate(kindr::Position3D(foot_positions[0](0, 3)+0.1805, foot_positions[0](1, 3)+0.047, foot_positions[0](2, 3)));
    // footPositions[LimbList::RF_LEG] = base_position + rotation.rotate(kindr::Position3D(foot_positions[1](0, 3)+0.1805, foot_positions[1](1, 3)-0.047, foot_positions[1](2, 3)));
    // footPositions[LimbList::LH_LEG] = base_position + rotation.rotate(kindr::Position3D(foot_positions[2](0, 3)-0.1805, foot_positions[2](1, 3)+0.047, foot_positions[2](2, 3)));
    // footPositions[LimbList::RH_LEG] = base_position + rotation.rotate(kindr::Position3D(foot_positions[3](0, 3)-0.1805, foot_positions[3](1, 3)-0.047, foot_positions[3](2, 3)));
    // std::cout << "&&" << footPositions[LimbList::LF_LEG] << std::endl;


    for(const auto& foot:footPositions) {
        nominal_height += foot.second(2);
    }
    nominal_height = nominal_height/nFeet + base_height;
    std::cout << "nominal_height: " << nominal_height << std::endl;

    for(const auto& limb:adapter.getLimbs()) {
        nominalStanceInBaseFrame_[limb] = kindr::Position3D(adapter.getPositionbaseToHipInBaseFrame(limb)(0),
                                                            adapter.getPositionbaseToHipInBaseFrame(limb)(1),
                                                            -nominal_height);
    }

    // std::cout << nominalStanceInBaseFrame_[LimbList::LF_LEG] << std::endl;

    Stance stance;
    stance = footPositions;
    unsigned int i = 0;
    Eigen::Matrix2d r_star;
    r_star << 0.0, -1.0,
              1.0, 0.0;

    Eigen::Matrix2d R_0;
    R_0 << cosf(yaw), -sinf(yaw),
           sin(yaw),  cos(yaw); 
    
    Eigen::Matrix3d R_0_0;
    R_0_0 << cosf(yaw), -sinf(yaw), 0,
             sin(yaw),  cos(yaw),   0,
             0,         0,          1;

    Eigen::Vector2d t;
    t << nominalStanceInBaseFrame_[LimbList::LF_LEG].vector()[0], nominalStanceInBaseFrame_[LimbList::LF_LEG].vector()[1];
    A.block(0, 0, nStates_, A.cols()) << Eigen::Matrix2d::Identity(), r_star * t; // R_0 * r_star * t;
    b(0, 1) = (footPositions[LimbList::LF_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::LF_LEG].vector())[0];
    b(0 + 1, 1) = (footPositions[LimbList::LF_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::LF_LEG].vector())[1];

    t << nominalStanceInBaseFrame_[LimbList::RF_LEG].vector()[0], nominalStanceInBaseFrame_[LimbList::RF_LEG].vector()[1];
    A.block(2, 0, nStates_, A.cols()) << Eigen::Matrix2d::Identity(), r_star * t; // R_0 * r_star * t;
    b(2, 1) = (footPositions[LimbList::RF_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::RF_LEG].vector())[0];
    b(2 + 1, 1) = (footPositions[LimbList::RF_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::RF_LEG].vector())[1];

    t << nominalStanceInBaseFrame_[LimbList::LH_LEG].vector()[0], nominalStanceInBaseFrame_[LimbList::LH_LEG].vector()[1];
    A.block(4, 0, nStates_, A.cols()) << Eigen::Matrix2d::Identity(), r_star * t; // R_0 * r_star * t;
    b(4, 1) = (footPositions[LimbList::LH_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::LH_LEG].vector())[0];
    b(4 + 1, 1) = (footPositions[LimbList::LH_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::LH_LEG].vector())[1];

    t << nominalStanceInBaseFrame_[LimbList::RH_LEG].vector()[0], nominalStanceInBaseFrame_[LimbList::RH_LEG].vector()[1];
    A.block(6, 0, nStates_, A.cols()) << Eigen::Matrix2d::Identity(), r_star * t; // R_0 * r_star * t;
    b(6, 1) = (footPositions[LimbList::RH_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::RH_LEG].vector())[0];
    b(6 + 1, 1) = (footPositions[LimbList::RH_LEG].vector() - R * nominalStanceInBaseFrame_[LimbList::RH_LEG].vector())[1];
    std::cout << "A--" << std::endl << A << std::endl;
 
    // for (const auto& footPosition : stance) {
        // Eigen::Vector2d t;
        // t << nominalStanceInBaseFrame_[footPosition.first].vector()[0], nominalStanceInBaseFrame_[footPosition.first].vector()[1];
        // A.block(6 - nDimensions_ * i, 0, nStates_, A.cols()) << Eigen::Matrix2d::Identity(), r_star * t;
        // std::cout << nominalStanceInBaseFrame_[footPosition.first].vector()[0] << ", " << nominalStanceInBaseFrame_[footPosition.first].vector()[1] << std::endl;
        // std::cout << "A--" << std::endl << A << std::endl;

        // b(nDimensions_ * i, 1) = (footPosition.second.vector() - R * nominalStanceInBaseFrame_[footPosition.first].vector())[0];
        // b(nDimensions_ * i + 1, 1) = (footPosition.second.vector() - R * nominalStanceInBaseFrame_[footPosition.first].vector())[1];

        // A.block(nDimensions_ * i, 0, nStates_, A.cols()) << Eigen::Matrix3d::Identity();
        // b.segment(nDimensions_ * i, nDimensions_) << footPosition.second.vector() - R * nominalStanceInBaseFrame_[footPosition.first].vector();

        // std::cout << footPosition.second.vector() << std::endl;
        // std::cout << R * nominalStanceInBaseFrame_[footPosition.first].vector() << std::endl;
        // ++i;
    // }

    // std::cout << "R: " << std::endl << R << std::endl;
    // std::cout << "A: " << std::endl << A << std::endl;
    // std::cout << "b: " << std::endl << b << std::endl;

    Eigen::MatrixXd G;
    Eigen::VectorXd hp;

    Stance support_stance;
    grid_map::Polygon supportRegion;
    // support_stance[LimbList::LF_LEG] = base_position + rotation.rotate(kindr::Position3D(0.2805, 0.047, -0.2));
    // support_stance[LimbList::RF_LEG] = base_position + rotation.rotate(kindr::Position3D(0.1805, -0.047, -base_height)); // footPositions[LimbList::RF_LEG];
    // support_stance[LimbList::LH_LEG] = base_position + rotation.rotate(kindr::Position3D(-0.1805, 0.047, -base_height)); // footPositions[LimbList::LH_LEG];
    // support_stance[LimbList::RH_LEG] = base_position + rotation.rotate(kindr::Position3D(-0.1805, -0.047, -base_height)); // footPositions[LimbList::RH_LEG];

    // FL, FR, RL, RR
    for(unsigned int i = 0; i < 4; i++) {
        if(foot_contacts[i]) {
            switch (i) {
                case 0:
                    support_stance[LimbList::LF_LEG] = footPositions[LimbList::LF_LEG];
                    break;
                case 1:
                    support_stance[LimbList::RF_LEG] = footPositions[LimbList::RF_LEG];
                    break;
                case 2:
                    support_stance[LimbList::LH_LEG] = footPositions[LimbList::LH_LEG];
                    break;
                case 3:
                    support_stance[LimbList::RH_LEG] = footPositions[LimbList::RH_LEG];
                    break;
            };
        }
    }

    std::vector<kindr::Position3D> footholdsOrdered;
    getFootholdsCounterClockwiseOrdered(support_stance, footholdsOrdered);
    for (auto foothold : footholdsOrdered) {
        supportRegion.addVertex(foothold.vector().head<2>());
    }


    bool isLinePolygon = false;
    // if there are only 2 stance leg, treat as a line with 0.001m witdth
    if (supportRegion.nVertices() == 2) {
        supportRegion.thickenLine(0.001);
        isLinePolygon = true;
    }
    // offsets the support region
    double supportMargin_ = 0.01;
    if (!isLinePolygon) supportRegion.offsetInward(supportMargin_);


    supportRegion.convertToInequalityConstraints(G, hp);
    const kindr::Position3D centerOfMassInBaseFrame =
        kindr::RotationQuaternionD(1, 0, 0, 0).inverseRotate(kindr::Position3D(0, 0, 0) - kindr::Position3D(0, 0, 0));
    Eigen::VectorXd h = hp - G * (R * centerOfMassInBaseFrame.vector()).head(2);
    G.conservativeResize(Eigen::NoChange, 3);
    G.col(2).setZero();

    // std::cout << "G: " << std::endl << G << std::endl;
    // std::cout << "hp: " << std::endl << hp << std::endl;
    // std::cout << "h: " << std::endl << h << std::endl;
 

    // Formulation as QP:
    // min 1/2 x'Px + q'x + r
    Eigen::MatrixXd P = 2 * A.transpose() * A;
    Eigen::VectorXd q = -2 * A.transpose() * b;

    Eigen::MatrixXd G_sparse = G.sparseView();
    Eigen::MatrixXd Aeq(P.cols(), 1);
    Eigen::VectorXd beq(1);

    Eigen::VectorXd params(P.cols());

    Eigen::MatrixXd P_sparse = P.sparseView();

    setGlobalHessian(P_sparse); // G
    setLinearTerm(q); // g0

    setGlobalInequalityConstraintJacobian(G_sparse);  // CI
    setInequalityConstraintMaxValues(h);  // CI0

    setGlobalEqualityConstraintJacobian(Aeq.setZero()); // CE
    setEqualityConstraintMaxValues(beq.setZero()); // CE0

    minimize(params);

    kindr::HomTransformQuatD pp;
    pp.getPosition().vector() = params;
    kindr::EulerAnglesZyxD eular_zyx(pp.getRotation());
    std::cout<<"SQP optimization result:"<<std::endl<<pp.getPosition()<<std::endl<<
                "Rotation: "<<std::endl<<"Roll: "<<eular_zyx.roll()<<std::endl<<"Pitch: "<<
                eular_zyx.pitch()<<std::endl<<"Yaw: "<<eular_zyx.yaw()<<std::endl;
    // std::cout << "params: " << std::endl << params << std::endl;

    return params;
}

void PoseOptimizer::minimize(Eigen::VectorXd& params) {

    int n = static_cast<int>(params.size());
    quadprogpp::Vector<double> x;
    x.resize(n);
    for(int i = 0; i < n; i++) {
        x[i] = params(i);
    }

    quadprogpp::solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x);

    for(int i = 0;i<n;i++){
        params(i) = x[i];
    }
}

void PoseOptimizer::setGlobalHessian(Eigen::MatrixXd& hessian) {
    int n = static_cast<int>(hessian.cols());
    quadprogpp::Matrix<double> G;
    G.resize(n, n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            G[i][j] = hessian(i,j);

    G_ = G;
}

void PoseOptimizer::setLinearTerm(Eigen::VectorXd& jacobian) {
    int n = static_cast<int>(jacobian.size());
    quadprogpp::Vector<double> g0;
    g0.resize(n);
    for (int i = 0; i < n; i++){
        g0[i] = jacobian(i);
    }

    g0_ = g0;
}

void PoseOptimizer::setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A)
{
    Eigen::MatrixXd A_T = -A.transpose();
    int m = static_cast<int>(A_T.rows());
    int n = static_cast<int>(A_T.cols());
    quadprogpp::Matrix<double> CI;
    CI.resize(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            CI[i][j] = A_T(i, j);

    CI_ = CI;
}

    
void PoseOptimizer::setInequalityConstraintMaxValues(Eigen::VectorXd& b)
{
    int n = static_cast<int>(b.size());
    quadprogpp::Vector<double> ci0;
    ci0.resize(n);
    for (int j = 0; j < n; j++)
        ci0[j] = b(j); 

    ci0_ = ci0;
}

void PoseOptimizer::setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq)
{
    int m = static_cast<int>(Aeq.rows());
    int n = static_cast<int>(Aeq.cols());
    quadprogpp::Matrix<double> CE;
    CE.resize(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            CE[i][j] = Aeq(i, j);

    CE_ = CE;
}

void PoseOptimizer::setEqualityConstraintMaxValues(Eigen::VectorXd& beq)
{
    int n = static_cast<int>(beq.size());
    quadprogpp::Vector<double> ce0;
    ce0.resize(n);
    for (int j = 0; j < n; j++)
        ce0[j] = beq(j);

    ce0_ = ce0;
}
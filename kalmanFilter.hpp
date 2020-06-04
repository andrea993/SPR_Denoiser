#include <eigen3/Eigen/Dense>

using A_t = Eigen::Matrix<double, 2, 2>;
using B_t = Eigen::Matrix<double, 0, 0>;
using C_t = Eigen::Matrix<double, 3, 2>;
using R_t = Eigen::Matrix<double, 3, 3>;
using Q_t = Eigen::Matrix<double, 2, 2>;
using P_t = Eigen::Matrix<double, 2, 2>;
using Xhat_t = Eigen::Vector2d;
using Yhat_t = Eigen::Vector3d;
using Y_t = Eigen::Vector3d;


struct StateSpace {
    StateSpace(const A_t &A = A_t::Zero(), const B_t &B = B_t::Zero(), const C_t &C = C_t::Zero()) :
        A(A), B(B), C(C)
    {}

    A_t A;
    B_t B;
    C_t C;
};

struct KalmanOut {
    KalmanOut(const Xhat_t &Xhat = Xhat_t::Zero(), const Yhat_t &Yhat = Yhat_t::Zero()) :
        Xhat(Xhat), Yhat(Yhat)
    {}

    Xhat_t Xhat;
    Yhat_t Yhat;
};


class KalmanFilter {
    public:      
        KalmanFilter(const StateSpace &ss = StateSpace(), const Q_t &Q = Q_t::Zero(), const P_t &P = P_t::Identity()*10) : 
            A(ss.A), B(ss.B), C(ss.C), Q(Q), P(P), Xhat(Xhat_t::Zero())
        {}

        KalmanOut GetSmp(const Y_t &Y, const R_t &R) {
            Yhat_t Yhat{ C*Xhat }; 
            auto epsilon{ Y - Yhat };
            auto S{ C*P*C.transpose() + R };
            auto K{ P*C.transpose()*S.inverse() };
            auto Xf{ Xhat + K*epsilon };
            auto Pf{ P - K*S*K.transpose() };

            Xhat = A*Xf;
            P = A*Pf*A.transpose() + Q;
            
            return KalmanOut(Xhat, Yhat);
        }

    private:
        A_t A;
        B_t B;
        C_t C;
        Q_t Q;
        P_t P;
        Xhat_t Xhat;

};
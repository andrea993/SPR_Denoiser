#include <eigen3/Eigen/Dense>

#include "envDetector.hpp"
#include "kalmanFilter.hpp"

constexpr double tauC = .005;
constexpr double ESP = 1e-3;
constexpr double k1_ed = .99;
constexpr double k2_ed = .8;
constexpr double K_R = 100.0;

class Denoiser {
    public:
        const double Ts;

        Denoiser(double Ts = 0.01) : Ts(Ts), ed0(k1_ed, k2_ed, Ts), ed1(k1_ed, k2_ed, Ts), ed2(k1_ed, k2_ed, Ts)
        {}

        Eigen::Vector2d GetSmp(Eigen::Vector2d u) {
            // Kalman
            Eigen::Vector3d kf_in;
            kf_in << u, 1;
            auto kf_out = kf.GetSmp(kf_in, R0 + R);
            // Update R
            auto err_pow = (kf_in - kf_out.Yhat).array().pow(2);
            R.diagonal() << ed0.GetSmp(err_pow(0)), ed1.GetSmp(err_pow(1)), ed2.GetSmp(err_pow(2));
            //Eigen::DiagonalMatrix<double, 3> diagR{ ed0.GetSmp(err_pow(0)), ed1.GetSmp(err_pow(1)), ed2.GetSmp(err_pow(2)) };
            //R = diagR;
            return kf_out.Xhat;
        }

    private:
        const Eigen::Matrix3d R0 = (Eigen::MatrixXd(3,3) << 10,   0,   0,
                                                             0,  10,   0,
                                                             0,   0, .05).finished();
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        const Eigen::Matrix2d Q = (Eigen::MatrixXd(2,2) <<   0,   0,
                                                             0, 100).finished();

        EnvDetector ed0;
        EnvDetector ed1;
        EnvDetector ed2;

        StateSpace ss {
            (A_t() << -1/tauC, 1/tauC, // A
                        0,       -ESP).finished(),
            B_t(), // B
            (C_t() << 1,   0, // C
                    1,   0,
                    1,  -1).finished()
        };
        KalmanFilter kf{ ss, Q };

};
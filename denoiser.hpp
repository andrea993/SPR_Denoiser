#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include "envDetector.hpp"
#include "kalmanFilter.hpp"

class Denoiser {
    public:
        const double Ts;
        
        Denoiser(double Ts = 0.01) : Ts(Ts), kf(StepInvariant(ss, Ts), Q), ed{EnvDetector(k1_ed, k2_ed, Ts), 
                                                                            EnvDetector(k1_ed, k2_ed, Ts),
                                                                            EnvDetector(k1_ed, k2_ed, Ts)}
                                                                            
        {}

        Eigen::Vector2d GetSmp(Eigen::Vector2d u) {
            // Kalman
            Eigen::Vector3d kf_in{ (Eigen::VectorXd(3) << u, 0).finished() };
            auto kf_out{ kf.GetSmp(kf_in, R0 + k_R * R) };
            // Update R
            auto err_pow{ (kf_in - kf_out.Yhat).array().pow(2) };
            R.diagonal() << ed[0].GetSmp(err_pow(0)), ed[1].GetSmp(err_pow(1)), ed[2].GetSmp(err_pow(2));

            
            //std::cout << kf_out.Xhat.transpose() << ' ' << R.diagonal().transpose() << '\n';

            return kf_out.Xhat;
        }

    private:
        const double k1_ed = .99;
        const double k2_ed = .8;
        const double k_R = 100.0;
        const double tauC = .005;
        const double EPS = 1e-3;

        static StateSpace StepInvariant (const StateSpace &ss, double Ts) { 
            auto ssd{ ss };
            //Eigen::SelfAdjointEigenSolver<A_t> eigensolver(ss.A*Ts);
            Eigen::EigenSolver<A_t> es(ss.A*Ts);

            auto eigvals{ es.eigenvalues().real()  };
            auto eigvecs{ es.eigenvectors().real() };
            
            A_t eigvmatd{ eigvals.array().exp().matrix().asDiagonal() };

            ssd.A = (eigvecs*eigvmatd*eigvecs.inverse());         
        
            return ssd;
        } 

        const R_t R0{ (R_t() << 10,   0,   0,
								0,  10,   0,
								0,   0, .05).finished() };
        R_t R{ R_t::Zero() };
        const Q_t Q{ (Q_t() <<  0,   0,
								0, 100).finished() };

        EnvDetector ed[3];

        StateSpace ss {
            (A_t() <<  -1/tauC,   1/tauC, 
                        0,        -EPS).finished(),
            B_t(), 
            (C_t() <<   1,   0, 
                        1,   0,
                        1,  -1).finished()
        };
        KalmanFilter kf;

};

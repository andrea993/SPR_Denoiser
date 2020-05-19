#include <eigen3/Eigen/Dense>

#include "envDetector.hpp"

class Denoiser {
    public:
        const double Ts;

        Denoiser(double Ts = 0.01) : Ts(Ts), ed(k1_ed, k2_ed, Ts) {
            
        }

    private:
        const double k1_ed = .99;
        const double k2_ed = .8;
        const double K_R = 100.0;
        const Eigen::Matrix3d R0 = (Eigen::MatrixXd(3,3) << 10, 0, 0, 
                                                            0, 10, 0,
                                                            0,  0, .05).finished();
        const Eigen::Matrix2d Q = (Eigen::MatrixXd(2,2) <<  0, 0, 
                                                            0, 100).finished();
                                    

        

        EnvDetector ed;




};
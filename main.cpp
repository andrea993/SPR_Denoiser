#include <iostream>
#include <cstdlib>

#include "denoiser.hpp"

#include <eigen3/Eigen/Dense>

std::istream& operator>>(std::istream& is, Eigen::VectorXd& v) {
    int size;
    is >> size;
    v.resize(size);
    for(int i = 0; i < size; ++i) {
        is >> v(i);
    }
    return is;
}

int main() {
    Denoiser den;
 
    while(true) {
        Eigen::Vector2d in_sample;
        std::cin >> in_sample(0) >> in_sample(1);
        if(std::cin.fail()) {
            break;
        }

        auto out_sample = den.GetSmp(in_sample);

        std::cout << out_sample(0) << ' ' << out_sample(1) << std::endl;
    };

    return EXIT_SUCCESS;
}
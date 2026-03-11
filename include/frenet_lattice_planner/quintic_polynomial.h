#ifndef FRENET_LATTICE_PLANNER__QUINTIC_POLYNOMIAL_H
#define FRENET_LATTICE_PLANNER__QUINTIC_POLYNOMIAL_H

#include <eigen3/Eigen/Dense>
#include <cmath>

class QuinticPolynomial{
public:
    double a0, a1, a2, a3, a4, a5;

    // Solve matrix using Eigen on object creation to derive quintic coefficients
    // xs = start pos
    // vxs = start vel
    // axs = start acc
    // xe = end pos
    // vxe = end vel
    // axe = end acc
    // T = total time for maneuver
    QuinticPolynomial(double xs, double vxs, double axs,
                        double xe, double vxe, double axe, double T)
    {
        // Start conditions give a0, a1, a2
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;

        // Setup matrix
        Eigen::Matrix3d A;
        A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
            3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
            6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

        // Setup B
        Eigen::Vector3d B;
        B << xe - a0 - a1 * T - a2 * std::pow(T, 2),
            vxe - a1 - 2 * a2 * T,
            axe - 2 * a2;

        // Solve for x to get a3, a4, a5
        Eigen::Vector3d x = A.colPivHouseholderQr().solve(B);

        a3 = x[0];
        a4 = x[1];
        a5 = x[2];
    }


    // Evaluate polynomial at time t to get position
    double calc_point(double t){
        return a0 + 
                a1 * t + 
                a2 * std::pow(t, 2) + 
                a3 * std::pow(t, 3) + 
                a4 * std::pow(t, 4) + 
                a5 * std::pow(t, 5);
    }

    double calc_first_derivative(double t){
        return  a1 + 
                2 * a2 * t + 
                3 * a3 * std::pow(t, 2) + 
                4 * a4 * std::pow(t, 3) + 
                5 * a5 * std::pow(t, 4);
    }

    double calc_second_derivative(double t){
        return  2 * a2 + 
                6 * a3 * t + 
                12 * a4 * std::pow(t, 2) + 
                20 * a5 * std::pow(t, 3);
    }
};


#endif // QUINTIC_POLYNOMIAL
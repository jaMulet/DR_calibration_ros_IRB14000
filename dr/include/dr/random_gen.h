#ifndef RANDOMISER_H_
#define RANDOMISER_H_

#include <iostream>
#include <random>

namespace dr {

    class Randomiser {

            std::normal_distribution<double> distribution;
            std::random_device rd;
            std::mt19937 mt;

            double min, max;

        public:

            /**
             * @brief The constructor
             * @param min Minimum value of the randomised range
             * @param max Maximum value of the randomised range
             */
            Randomiser(double min, double max);
            
            /**
             * @brief Randomiser operator
             *
             */
            double operator ()();

        private:
            
    };
}// namespace yumi_dr
#endif // RANDOMISER_H_
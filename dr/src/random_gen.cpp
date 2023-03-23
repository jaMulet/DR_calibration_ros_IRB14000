#include "random_gen.h"

#include <random>
#include <iostream>

namespace dr {

    /***********************************************************************************************************************
     * Class definitions: Randomiser
     */
    /***********************************************************************************************************************
     * Constructor
     **************************************************************************/
    Randomiser::Randomiser(double min, double max):
        distribution((min+max)/2, (max-min)/6), mt(rd()), min(min), max(max)
        {}

    /***********************************************************************************************************************
     * Operator of the randomisee class
     **************************************************************************/
    double Randomiser::operator ()(){
        while (true)
        {
            double number = this->distribution(this->mt);
            if (number >= this->min && number <= this->max)
                return number;
        }
        
    }

}// end yumi_dr namespace
#include "target_manipulator.h"

#include <string>
#include <sstream>
#include <vector>
#include <iterator>


namespace dr {

    /***********************************************************************************************************************
     * Class definitions: TargetManipulator
     */
    /***********************************************************************************************************************
     * Constructor
     **************************************************************************/
    TargetManipulator::TargetManipulator(){}

    /**************************************************************************
     * removeBrackets method
     **************************************************************************/
    std::string TargetManipulator::removeBrackets(std::string input)
    {
        int i= 0;
        while (i < input.size())
        {
            if (input[i] == '[' || input[i] == ']')
            {
            input.erase(i,1);
            }
            else
            {
            i++;
            }
        }
        return input;
    }

    /**************************************************************************
     * split method
     **************************************************************************/
    std::vector<std::string> TargetManipulator::split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }

    /**************************************************************************
     * split method (private)
     **************************************************************************/
    template <typename Out>
    void TargetManipulator::split(const std::string &s, char delim, Out result)
      {
        std::istringstream iss(s);
        std::string item;
        while(std::getline(iss, item, delim))
        {
            *result++ = item;
        }
      }

}// end dr namespace
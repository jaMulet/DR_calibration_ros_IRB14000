#ifndef TARGET_MANIPULATOR_H_
#define TARGET_MANIPULATOR_H_

#include <string>
#include <vector>

namespace dr {

    class TargetManipulator {

        public:         
            
            /**
             * @brief The constructor
             * @param min Minimum value of the randomised range
             * @param max Maximum value of the randomised range
             */
            TargetManipulator();

            /**
             * @brief Remove squared brackets
             * @param input Input string which contains initial and ending brackets
             */
            std::string removeBrackets(std::string input);

            /**
             * @brief Splits the string separated by a delimiter
             * @param s Input string
             * @param delim Delimiter of values in the string
             */
            std::vector<std::string> split(const std::string &s, char delim);

        private:

            /**
             * @brief Splits the string separated by a delimiter
             * @param s Input string
             * @param delim Delimiter of values in the string
             * @param result Output of the function
             */
            template <typename Out>
            void split(const std::string &s, char delim, Out result);
    };

}// end dr namespace

#endif // TARGET_MANIPULATOR_H
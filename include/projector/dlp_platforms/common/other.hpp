/** @file       other.hpp
 *  @ingroup    group_Common
 *  @brief      Contains common functions relating to strings, numbers, and time
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */


#ifndef DLP_SDK_OTHER_HPP
#define DLP_SDK_OTHER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <limits>   // for std::numeric_limits
#include <iomanip>  // for setprecision()

#define NUM_TO_STRING_PRECISION     16
#define FILE_DOES_NOT_EXIST         "FILE_DOES_NOT_EXIST"


/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/* DISALLOW_COPY_AND_ASSIGN Macro from google
 * http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml?showone=Copy_Constructors#Copy_Constructors
 *
 */
// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

#define DLP_STD_CIN_GET(value) std::cin >> value; \
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

void PressEnterToContinue(std::string msg = "Press ENTER to continue...");


namespace CmdLine{

template <typename Type = std::string>
void Print(const std::string &msg = "", const Type &value = "", const std::string &units = ""){
    std::cout << msg << value << units << std::endl;
}

template <typename Type>
bool Get(Type &value, const std::string &msg = ""){
    bool ret = true;

    std::cout << msg;
    std::cin >> value;

    if(std::cin.fail()){
        std::cin.clear();
        ret = false;
    }
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    return ret;
}

void PressEnterToContinue(const std::string &msg = "Press ENTER to continue...");

}

/** @brief  Contains sleep functions and time tracking \ref dlp::Time::Chronograph class
 *  @ingroup group_Common
 */
namespace Time{

    /** @brief  Contains methods to pause program or thread execution */
    namespace Sleep{
        void Microseconds(unsigned int time);
        void Milliseconds(unsigned int time);
        void Seconds(unsigned int time);
    }

    /** @class Chronograph
     *  @brief  Measures time between laps and total time in milliseconds
     *  @ingroup group_Common
     */
    class Chronograph{
    public:
        Chronograph();
        Chronograph(bool start);

        unsigned long long Reset();
        unsigned long long Lap();

        std::vector<unsigned long long> GetLapTimes();
        unsigned long long              GetTotalTime();

    private:
        unsigned long long start_;
        unsigned long long last_lap_;
        std::vector<unsigned long long> laps_;
    };
}

/** @brief  Contains common functions related to files
 *  @ingroup group_Common
 */
namespace File{
    bool Exists( std::string filename );
    unsigned long long GetSize(std::string filename);
    std::vector<std::string> ReadLines(std::string filename);
}

/** @brief  Contains common functions related to string manipulation
 *  @ingroup group_Common
 */
namespace String{

    std::string Trim(const std::string &string);
    std::string ToUpperCase(const std::string &string);
    std::string ToLowerCase(const std::string &string);

    std::vector<std::string> SeparateDelimited(const std::string &string, const char &delimiter);

    /** @brief Converts an ASCII string number to a signed or unsigned numerical variable
     *
     *  For example, to convert a string to an int perform the following:
     *  int value = dlp::String::ToNumber<int>("123");
     */
    template <typename T>
    T ToNumber( const std::string &text, unsigned int base = 10){
        std::string trimmed = dlp::String::Trim(text);

        std::size_t hex_0x = trimmed.find("0x");
        std::size_t hex_x  = trimmed.find("x");

        if(hex_0x != std::string::npos){
            base = 16;
            trimmed = trimmed.substr(hex_0x + 2);
        }
        else if(hex_x != std::string::npos){
            base = 16;
            trimmed = trimmed.substr(hex_x + 1);
        }

        std::istringstream ss(trimmed);
        T result;
        long double number;
        long long number_int;

        switch(base){
        case 8:
            ss >> std::oct >> number_int;          // Convert string to number
            result = (T) number_int;
            break;
        case 16:
            ss >> std::hex >> number_int;          // Convert string to number
            result = (T) number_int;
            break;
        case 10:
        default:
            ss >> std::dec >> number;          // Convert string to number
            result = (T) number;
        }

        return result;
    }


    template <> std::string ToNumber( const std::string &string, unsigned int base);

//    /** @brief Converts an ASCII string hexadecimal number to an unsigned numerical variable
//     *
//     *  For example, to convert a hexadecimal string to an int perform the following:
//     *  unsigned int value = dlp::String::HEXtoNumber_unsigned<unsigned int>("0x1A");
//     */
//    template <typename T>
//    T HEXtoNumber_unsigned(const std::string &text){
//        unsigned long long temp = strtoull(text, nullptr, 16);//std::stoull(text, nullptr, 16);
//        T result = (T) temp;
//        return result;
//    }

//    /** @brief Converts an ASCII string hexadecimal number to a signed numerical variable
//     *
//     * For example, to convert a hexadecimal string to an int perform the following:
//     *
//     * @code{.cpp}
//     * #include <stdlib.h>
//     * #include <stdio.h>
//     * int value = dlp::String::HEXtoNumber_unsigned<int>("0x1A");
//     * @endcode
//     *
//     * The expected output would be as follows.
//     *
//     * @verbatim
//     * user@ti.com : i = 23
//     * @endverbatim
//     *
//     */
//    template <typename T>
//    T HEXtoNumber_signed(const std::string &text){
//        long long temp =  strtoll(text, nullptr, 16); // std::stoll(text, nullptr, 16);
//        T result =  (T) temp;
//        return result;
//    }

}

/** @brief  Contains common functions to convert numbers to strings
 *  @ingroup group_Common
 */
namespace Number{

    /** @brief Converts a numerical variable to its ASCII string equivalent */
    template <typename T>
    std::string ToString( T number ){
        std::ostringstream ss;
        ss << std::setprecision(NUM_TO_STRING_PRECISION);
        ss << number;
        return ss.str();
    }

    template <> std::string ToString<std::string>( std::string string );
    template <> std::string ToString<char>( char number );
    template <> std::string ToString<unsigned char>( unsigned char number );

}

class GLFW_Library{
public:
    static bool Init();
    static void Terminate();
    static unsigned int GetMonitorCount();

private:
    static unsigned int open_connections_;
};

}

#endif // DLP_SDK_OTHER_HPP

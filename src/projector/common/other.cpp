/** @file   other.cpp
 *  @brief  Contains the dlp::Time::Chronograph class and also common file,
 *          number, and string functions
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <common/debug.hpp>
#include <common/other.hpp>

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <iterator>
#include <algorithm>
#include <sys/stat.h>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

void PressEnterToContinue(std::string msg){
    std::cout << msg;
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}

void CmdLine::PressEnterToContinue(const std::string &msg){
    std::cout << msg;
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}

/** @brief Returns true if the file exists
 *  @ingroup Common
 */
bool File::Exists( std::string filename ){
    bool ret;
    std::ifstream file(filename.c_str(),std::ifstream::in);
    if(file){
        ret = true;
        file.close();
    }
    else{
        ret = false;
    }
    return ret;
}

/** @brief Returns the file size in bytes
 *  @ingroup Common
 */
unsigned long long File::GetSize(std::string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

/** @brief Opens and reads all lines into a vector of strings. One string per line.
 *  \note If the file does not exist or open correctly the returned vector will be empty.
 *  @ingroup Common
 */
std::vector<std::string> File::ReadLines(std::string filename){
    std::vector<std::string>  ret;
    std::string line;

    // Clear return vector
    ret.clear();

    // Open file as input
    std::ifstream file(filename.c_str(),std::ifstream::in);

    // Check that file opened successfully
    if(file){

        // Read all lines in the file
        while ( getline(file,line) )
        {
            // Save the line to the vector
            ret.push_back(line);
        }

        // Close the file
        file.close();
    }
    return ret;
}

/** @brief      Pauses program or thread execution
 *  @param[in]  time Number of microseconds to pause
 *  @ingroup    Common
 */
void Time::Sleep::Microseconds(unsigned int time){
    std::chrono::microseconds timespan(time);
    std::this_thread::sleep_for(timespan);
}

/** @brief      Pauses program or thread execution
 *  @param[in]  time Number of milliseconds to pause
 *  @ingroup    Common
 */
void Time::Sleep::Milliseconds(unsigned int time){
    std::chrono::milliseconds timespan(time);
    std::this_thread::sleep_for(timespan);
}


/** @brief      Pauses program or thread execution
 *  @param[in]  time Number of seconds to pause
 *  @ingroup    Common
 */
void Time::Sleep::Seconds(unsigned int time){
    std::chrono::seconds timespan(time);
    std::this_thread::sleep_for(timespan);
}


/** @brief Constructs object in a NON-started state */
Time::Chronograph::Chronograph(){
    this->start_    = 0;
    this->last_lap_ = 0;
    this->laps_.clear();
}

/** @brief Constructs object
 *  @param[in] start If true the object is automatically reset (started) \ref Time::Chronograph::Reset()
 */
Time::Chronograph::Chronograph(bool start){
    if(start){
        this->Reset();
    }
    else{
        this->start_    = 0;
        this->last_lap_ = 0;
        this->laps_.clear();
    }
}

/** @brief Saves the current time and clears all previously stored lap times */
unsigned long long Time::Chronograph::Reset(){
    this->laps_.clear();
	auto time_now = std::chrono::high_resolution_clock::now().time_since_epoch();
	this->start_ =  std::chrono::duration_cast<std::chrono::milliseconds>(time_now).count();// std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    this->last_lap_ = this->start_;
    return this->start_;
}

/** @brief  Returns the elapsed time from the most previous Lap() or Reset()
 *          call in milliseconds
 */
unsigned long long Time::Chronograph::Lap(){
    unsigned long long lap_time = 0;

	if (this->start_ != 0){
		auto time_now = std::chrono::high_resolution_clock::now().time_since_epoch();
		unsigned long long current = std::chrono::duration_cast<std::chrono::milliseconds>(time_now).count();
        this->laps_.push_back(current);

        lap_time = current - this->last_lap_;
        this->last_lap_ = current;
    }

    return lap_time;
}

/** @brief Returns a vector of all lap times in milliseconds */
std::vector<unsigned long long> Time::Chronograph::GetLapTimes(){
    return this->laps_;
}

/** @brief Returns the time in milliseconds since the object has been Reset() */
unsigned long long Time::Chronograph::GetTotalTime(){
    unsigned long long current = 0;

	if (this->start_ != 0){
		auto time_now = std::chrono::high_resolution_clock::now().time_since_epoch();
		current = std::chrono::duration_cast<std::chrono::milliseconds>(time_now).count();
        //current = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
    }

    return current - this->start_;
}

/** @brief   Returns the supplied string with all leading and trailing whitespace removed
 *  \note    Both tabs and spaces are considered whitespace
 *  @ingroup Common
 */
std::string String::Trim(const std::string &string){
    std::string ret = "";
    std::string whitespace   = " \t";
    std::size_t string_begin = string.find_first_not_of(whitespace);
    std::size_t string_end   = string.find_last_not_of(whitespace);
    std::size_t string_range = string_end - string_begin + 1;

    // Check for carriage return and remove if present
    if(string.at(string_end) == 13){
        string_range--;
    }

    // Check that string is NOT empty
    if (string_begin != std::string::npos){
        ret =string.substr(string_begin, string_range);
    }

    return ret;
}

/** @brief   Returns the supplied string in all upper case ASCII characters
 *  \note    Only alphabetical characters are converted
 *  @ingroup Common
 */
std::string String::ToUpperCase(const std::string &string){
    std::string ret = string;

    // Convert all lower case characters in string to upper case
    std::transform(ret.begin(), ret.end(), ret.begin(), ::toupper);

    return ret;
}

/** @brief   Returns the supplied string in all lower case ASCII characters
 *  \note    Only alphabetical characters are converted
 *  @ingroup Common
 */
std::string String::ToLowerCase(const std::string &string){
    std::string ret = string;

    // Convert all lower case characters in string to upper case
    std::transform(ret.begin(), ret.end(), ret.begin(), ::tolower);

    return ret;
}

/**
 * @brief       Returns vector of strings where each string is a value
 *              separated by the delimiter
 * @param[in]   string      Input string with delimited values
 * @param[in]   delimiter   Character which separates the values in the input string
 * @ingroup Common
 */
std::vector<std::string> String::SeparateDelimited(const std::string &string, const char &delimiter){
    std::string string_temp;
    std::vector<std::string> separated_line;

    std::istringstream string_stream;

    // Remove leading and trailing whitespace
    string_temp = dlp::String::Trim(string);

    // Put the line into stream
    string_stream.str(string_temp);

    // Seperate the comma separated values into individual strings
    while(string_stream){
        std::string separated_string;

        // Get individual comma separated values into separated_string
        if(!std::getline(string_stream,separated_string, delimiter)) break;

        // Add value to vector
        separated_line.push_back(dlp::String::Trim(separated_string));
    }

    return separated_line;
}

template <> std::string String::ToNumber(const std::string &string, unsigned int base){
    (void)base; // Remove unused variable warning
    return string;
}


/** @brief In the case that this function is called with a string the same string is returned
 *  @ingroup Common
 */
template <> std::string Number::ToString<std::string>( std::string string ){
    return string;
}


/** @brief Converts a signed char variable to its integer ASCII string equivalent
 *  @ingroup Common
 */
template <> std::string Number::ToString<char>( char number ){
    std::ostringstream ss;
    ss << (int) number;
    return ss.str();
}

/** @brief   Converts an unsigned char variable to its integer ASCII string equivalent
 *  @ingroup Common
 */
template <> std::string Number::ToString<unsigned char>( unsigned char number ){
    std::ostringstream ss;
    ss << (unsigned int) number;
    return ss.str();
}

}

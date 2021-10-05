/** @file   debug.cpp
 *  @brief  Contains methods for Debug class
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/other.hpp>

#include <iostream>
#include <fstream>
#include <string>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @brief  Constructs object that is disabled and outputs to std::cout */
Debug::Debug(){
    this->enable_   = false;
    this->level_    = 0;
    this->name_     = "DLP_DEBUG_";
    this->output_   = &std::cout;
}

/** @brief      Specifies the name of the debug instance
 *  @param[in]  name Name of the debug instance
 */
void Debug::SetName(const std::string &name){
    this->name_ = name;
    return;
}

/** @brief Enables or disables the debug message output */
void Debug::SetEnable(const bool &enable){
    this->enable_ = enable;
    return;
}

/** @brief     Sets the maximum level of debug messages to output.
 *  @param[in] level Maximum level of debug messages to output
 *
 *  Any debug message with a level equal to or less than the
 *  SetLevel() parameter will be sent to the output stream.
 */
void Debug::SetLevel(const unsigned int &level){
    this->level_ = level;
    return;
}

/** @brief     Sets output stream for the debug instance to send messages to.
 *  @param[in] output Pointer to the output stream
 *  \note      The Debug class does NOT open, close, or control its output stream. If
 *             the stream closes, the messages will automatically go to std::cerr instead.
 */
void Debug::SetOutput(std::ostream *output){
    if(output) this->output_ = output;
    return;
}

/** @brief Returns true if the debug instance is enabled */
bool Debug::GetEnable() const{
    return this->enable_;
}

/** @brief Returns the maximum level of messages to display for the debug instance */
unsigned int Debug::GetLevel()const{
    return this->level_;
}

/** @brief Returns the name of the debug instance */
std::string Debug::GetName() const{
    return this->name_;
}

/** @brief Returns a pointer to the output stream of the debug instance */
std::ostream* Debug::GetOutput()const{
    return this->output_;
}

/** @brief     Displays debug message
 *  @param[in] msg Message
 *  \note If the set output stream closes, the message will be sent to std::cerr.
 *
 *  Debug messages output in the following format: {%Debug instance name} + {message} + std::endl
 */
void Debug::Msg(const std::string &msg) const{
    // If this debug object is enabled output the debug message
    if(this->enable_){
        if(*this->output_){
            (*this->output_) << this->name_ + msg << std::endl;
            (*this->output_).flush();
            dlp::Time::Sleep::Milliseconds(10);
        }
        else{
            std::cerr << "<<< DEBUG_OBJECT_FAILURE >>> message = " + msg << std::endl;
        }
    }
    return;
}

/** @brief      Displays debug message
 *  @param[in]  msg Message
 *  \note       If the set output stream closes, the message will be sent to std::cerr.
 *
 *  Debug messages output in the following format: {%Debug instance name} + {message} + std::endl
 */
void Debug::Msg(const std::stringstream &msg) const{
    this->Msg(msg.str());
}

/** @brief      Displays debug message if debug instance level allows
 *  @param[in]  level    Minimum level the debug instance must be set to for the message to display
 *  @param[in]  msg      Message
 *  \note       If the set output stream closes, the message will be sent to std::cerr.
 *
 *  Debug messages output in the following format: {%Debug instance name} + {message} + std::endl
 */
void Debug::Msg(const unsigned int &level, const std::stringstream &msg) const{
    if(level <= this->level_) this->Msg(msg);
    return;
}

/** @brief      Displays debug message if the level is less than or equal to the level threshold
 *  @param[in]  level    Minimum level threshold the debug instance must be set to for the message to display
 *  @param[in]  msg      Message
 *  \note       If the set output stream closes, the message will be sent to std::cerr.
 *
 *  Debug messages output in the following format: {%Debug instance name} + {message} + std::endl
 */
void Debug::Msg(const unsigned int &level, const std::string &msg) const{
    if(level <= this->level_) this->Msg(msg);
    return;
}

}

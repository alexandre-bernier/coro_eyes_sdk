/** @file       debug.hpp
 *  @ingroup    Common
 *  @brief      Defines the Debug class for terminal output messages
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_DEBUG_HPP
#define DLP_SDK_DEBUG_HPP

#include <iostream>
#include <fstream>
#include <string>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @class      Debug
 *  @ingroup    Common
 *  @brief      Runtime debug message interface with enable, level, and output controls.
 *
 *  Debug messages can be enabled or disabled during runtime and also
 *  have levels associated to them. Levels allow the user to specify
 *  which messages are displayed. The output stream can also be determined
 *  by the user.
 *
 *  Debug messages print in the following format: {Debug instance name} + {message} + std::endl
 *
 *  @warning The Debug class does NOT open, close, or control its output stream. If
 *           the stream closes, the messages will automatically go to std::cerr instead.
 *
 */
class Debug{
public:
    Debug();

    void SetEnable(const bool &enable);
    void SetLevel(const unsigned int &level);
    void SetName(const std::string &name);
    void SetOutput(std::ostream *output);

    bool          GetEnable() const;
    unsigned int  GetLevel()  const;
    std::string   GetName()   const;
    std::ostream* GetOutput() const;

    void Msg(const std::string       &msg) const;
    void Msg(const std::stringstream &msg) const;
    void Msg(const unsigned int &level, const std::string       &msg) const ;
    void Msg(const unsigned int &level, const std::stringstream &msg) const;
private:
    bool          enable_;
    unsigned int  level_;
    std::string   name_;
    std::ostream  *output_;
};
}

#endif

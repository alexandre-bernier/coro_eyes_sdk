/*! @file       returncode.hpp
 *  @ingroup    Common
 *  @brief      Defines ReturnCode class for all SDK modules for errors and warning messages
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_RETURNCODE_HPP
#define DLP_SDK_RETURNCODE_HPP

#include <string>
#include <vector>

#include <projector/dlp/common/other.hpp>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @class      ReturnCode
 *  @ingroup    Common
 *  @brief      Return type for most DLP SDK methods.
 *  @warning    NOT functional with switch() statements
 *  @example    returncodes_example.cpp
 */
class ReturnCode{
public:
    void Clear();

    ReturnCode& AddError(   const std::string &msg );
    ReturnCode& AddWarning( const std::string &msg );
    ReturnCode& Add(const ReturnCode &source);

    bool hasErrors() const;
    bool hasWarnings() const;

    bool ContainsError(   std::string msg ) const;
    bool ContainsWarning( std::string msg ) const;

    std::vector<std::string> GetErrors() const;
    std::vector<std::string> GetWarnings() const;
    unsigned int GetErrorCount() const;
    unsigned int GetWarningCount() const;

    std::string ToString() const;

    /** @brief Returns true if object has errors
     *
     *  Example usage:
     *
     *  @code
     *  dlp::ReturnCode retval;
     *
     *  // ...
     *
     *  if(retval){
     *      // retval has errors
     *      // retval may or may nor have warnings, use retval.hasWarnings() to check
     *  }
     *  else{
     *      // retval does NOT have errors
     *      // retval may or may nor have warnings, use retval.hasWarnings() to check
     *  }
     *  @endcode
     */
    operator bool() const{
        if(this->errors_.size()>0)  return false;
        else                        return true;
    }


private:
    std::vector<std::string> errors_;
    std::vector<std::string> warnings_;
};

}

#endif //#ifndef DLP_SDK_RETURNCODE_HPP












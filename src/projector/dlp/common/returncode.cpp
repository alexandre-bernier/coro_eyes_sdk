/** @file   returncode.cpp
 *  @brief  Contains methods for \ref dlp::ReturnCode
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/other.hpp>

#include <vector>
#include <string>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{


void ReturnCode::Clear(){
    this->warnings_.clear();
    this->errors_.clear();
}

/** @brief  Adds error message to object and returns a reference to itself
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  std::string     error_msg = "ERROR";
 *
 *  retval.AddError(error_msg);
 *  @endcode
 */
ReturnCode& ReturnCode::AddError(const std::string &msg ){
    if(!msg.empty()){
        this->errors_.push_back(msg);
    }
    return *this;
}

/** @brief  Adds warning message to object and returns a reference to itself
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  std::string     warning_msg = "WARNING";
 *
 *  retval.AddWarning(warning_msg);
 *  @endcode
 */
ReturnCode& ReturnCode::AddWarning(const std::string &msg ){
    if(!msg.empty()){
        this->warnings_.push_back(msg);
    }
    return *this;
}

/** @brief  Copies all error and warning messages from source
 *          and returns a reference to itself
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval_a;
 *  dlp::ReturnCode retval_b;
 *  std::string     error_msg = "ERROR";
 *
 *  retval_a.AddError(error_msg);
 *  retval_b.Add(retval_a);
 *  @endcode
 */
ReturnCode& ReturnCode::Add(const ReturnCode &source){
    if(this != &source){
        for(unsigned int iError = 0; iError < source.errors_.size();iError++){
            this->errors_.push_back(source.errors_.at(iError));
        }
        for(unsigned int iWarning = 0; iWarning < source.warnings_.size();iWarning++){
            this->warnings_.push_back(source.warnings_.at(iWarning));
        }
    }
    return *this;
}

/** @brief Returns true if object has errors
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *
 *  // ...
 *
 *  if(retval.hasErrors()){
 *      // retval has errors
 *  }
 *  else{
 *      // retval does NOT have errors
 *  }
 *  @endcode
 */
bool ReturnCode::hasErrors() const{
    if(this->errors_.size() > 0)    return true;
    else                            return false;
}

/** @brief Returns true if object has warnings
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *
 *  // ...
 *
 *  if(retval.hasWarnings()){
 *      // retval has warnings
 *  }
 *  else{
 *      // retval does NOT have warnings
 *  }
 *  @endcode
 */
bool ReturnCode::hasWarnings() const{
    if(this->warnings_.size() > 0)    return true;
    else                            return false;
}

/** @brief Returns true if object has the exact supplied string as error
 *
 *  Example usage:
 *
 *  @code
 *  #define ERROR_MESSAGE "ERROR_MESSAGE"
 *
 *  dlp::ReturnCode retval;
 *
 *  // ...
 *
 *  if(retval.ContainsError(ERROR_MESSAGE)){
 *      // retval has ERROR_MESSAGE
 *  }
 *  else{
 *      // retval does NOT have ERROR_MESSAGE
 *  }
 *  @endcode
 */
bool ReturnCode::ContainsError( std::string msg) const{
    bool contains_error = false;
    for(unsigned int iError = 0; (iError < this->errors_.size()) && (!contains_error); iError++){
        if(msg.compare(this->errors_.at(iError)) == 0) contains_error = true;
    }
    return contains_error;
}

/** @brief Returns true if object has the exact supplied string as warning
 *
 *  Example usage:
 *
 *  @code
 *  #define WARNING_MESSAGE "WARNING_MESSAGE"
 *
 *  dlp::ReturnCode retval;
 *
 *  // ...
 *
 *  if(retval.ContainsWarning(WARNING_MESSAGE)){
 *      // retval has WARNING_MESSAGE
 *  }
 *  else{
 *      // retval does NOT have WARNING_MESSAGE
 *  }
 *  @endcode
 */
bool ReturnCode::ContainsWarning( std::string msg) const{
    bool contains_warning = false;
    for(unsigned int iWarning = 0; (iWarning < this->warnings_.size()) && (!contains_warning); iWarning++){
        if(msg.compare(this->warnings_.at(iWarning)) == 0) contains_warning = true;
    }
    return contains_warning;
}

/** @brief Returns string vector of all object errors
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  std::vector<std::string> errors;
 *
 *  // ...
 *
 *  errors = retval.GetErrors();
 *  @endcode
 */
std::vector<std::string> ReturnCode::GetErrors() const{
    return this->errors_;
}

/** @brief Returns string vector of all object warnings
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  std::vector<std::string> warnings;
 *
 *  // ...
 *
 *  warnings = retval.GetWarnings();
 *  @endcode
 */
std::vector<std::string> ReturnCode::GetWarnings() const{
    return this->warnings_;
}

/** @brief Returns number of errors
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  unsigned int error_count;
 *
 *  // ...
 *
 *  error_count = retval.GetErrorCount();
 *  @endcode
 */
unsigned int ReturnCode::GetErrorCount() const{
    return this->errors_.size();
}

/** @brief Returns number of warnings
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *  unsigned int warning_count;
 *
 *  // ...
 *
 *  warning_count = retval.GetWarningCount();
 *  @endcode
 */
unsigned int ReturnCode::GetWarningCount() const{
    return this->warnings_.size();
}

/** @brief Returns multiline string where the first line lists the quantity of
 *         errors and warnings while all following lines list the individual
 *         errors and warnings
 *
 *  Example usage:
 *
 *  @code
 *  dlp::ReturnCode retval;
 *
 *  retval.AddError("ERROR_PARAMETER_INVALID");
 *  retval.AddWarning("WARNING_PARAMTER_VALUE_VERY_LARGE")
 *
 *  std::cout << retval.ToString() << std::endl;
 *  @endcode
 *
 *  Example output:
 *
 *  @verbatim
 *  ERRORS: 1 and WARNINGS: 1
 *  ERROR: ERROR_PARAMETER_INVALID
 *  WARNING: WARNING_PARAMTER_VALUE_VERY_LARGE
 *  @endverbatim
 *
 */
std::string ReturnCode::ToString() const{
    std::string ret;

    unsigned int errors   = this->GetErrorCount();
    unsigned int warnings = this->GetWarningCount();

    ret = "ERRORS: " + dlp::Number::ToString(errors) + " and WARNINGS: " + dlp::Number::ToString(warnings);

    if(errors > 0){
        for(unsigned int iError = 0; iError < errors; iError++){
            ret += "\nERROR: " + this->errors_.at(iError);
        }
    }

    if(warnings > 0){
        for(unsigned int iWarning = 0; iWarning < warnings; iWarning++){
            ret += "\nWARNING: " + this->errors_.at(iWarning);
        }
    }

    return ret;
}

}

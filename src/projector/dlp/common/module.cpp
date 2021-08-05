/** @file       module.cpp
 *  @ingroup    Common
 *  @brief      Contains methods for \ref dlp::Module debug interface
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <common/debug.hpp>
#include <common/module.hpp>
#include <common/other.hpp>

namespace dlp{

/** @brief      Turns on debug output messages DLP SDK module
 *  @param[in]  enable  Turns on debug messages if true
 */
void Module::SetDebugEnable(const bool &enable){
    this->debug_.SetEnable(enable);
}

/** @brief      Sets the level for debug output messages for the DLP SDK object
 *  @param[in]  level  Sets the maximum level for debug messages to
 */
void Module::SetDebugLevel(const unsigned int &level){
    this->debug_.SetLevel(level);
}

/** @brief      Sets the stream for debug output messages for the DLP SDK object
 *  @param[out] output  Pointer to return output stream
 */
void Module::SetDebugOutput(std::ostream* output){
    this->debug_.SetOutput(output);
}

/** @brief Returns true if DLP SDK module has been setup */
bool Module::isSetup()const{
    return this->is_setup_;
}

ReturnCode Module::Setup(Module &module, std::string parameters_file, bool output_cmdline){
    dlp::ReturnCode ret;
    dlp::Parameters settings;

    // Load the parameters from a file
    ret = settings.Load(parameters_file);
    if(ret.hasErrors()){
        if(output_cmdline){
            dlp::CmdLine::Print("Loading parameters file FAILED!");
            dlp::CmdLine::Print(ret.ToString());
        }
        return ret;
    }

    return Setup(module, settings, output_cmdline);
}


ReturnCode Module::Setup(Module &module, const dlp::Parameters &settings, bool output_cmdline){
    dlp::ReturnCode ret;

    // Setup the module file
    ret = module.Setup(settings);
    if(ret.hasErrors()){
        if(output_cmdline){
            dlp::CmdLine::Print("Module setup FAILED!");
            dlp::CmdLine::Print(ret.ToString());
        }
    }
   return ret;
}

}

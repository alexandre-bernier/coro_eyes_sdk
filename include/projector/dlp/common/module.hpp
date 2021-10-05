/** @file       module.hpp
 *  @ingroup    Common
 *  @brief      Defines dlp::Module base class for minimum DLP SDK module requirements
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_MODULE_HPP
#define DLP_SDK_MODULE_HPP

#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/parameters.hpp>

namespace dlp{

/** @class  Module
 *  @brief  Base class for DLP SDK modules which includes Setup() and GetSetup()
 *          methods for use with \ref dlp::Parameters and include a \ref dlp::Debug interface
 */
class Module{
public:
    virtual ReturnCode Setup(const dlp::Parameters &settings) = 0;
    virtual ReturnCode GetSetup(dlp::Parameters *settings) const = 0;

    bool isSetup() const;
    void SetDebugEnable(const bool &enable);
    void SetDebugLevel(const unsigned int &level);
    void SetDebugOutput(std::ostream* output);

    static ReturnCode Setup(Module &module, std::string parameters_file, bool output_cmdline = false);
    static ReturnCode Setup(Module &module, const dlp::Parameters &settings, bool output_cmdline = false);
protected:
    Debug   debug_;
    bool    is_setup_;
};

}

#endif // DLP_SDK_MODULE_HPP

/** @file       dlp_platform.hpp
 *  @brief      This header file declares the DLP_Platform base class.
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_DLP_PLATFORM_HPP
#define DLP_SDK_DLP_PLATFORM_HPP

#include <common/returncode.hpp>
#include <common/debug.hpp>
#include <common/other.hpp>
#include <common/image/image.hpp>
#include <common/pattern/pattern.hpp>
#include <common/parameters.hpp>
#include <common/module.hpp>
#include <common/pixel_array_configuration.hpp>

#include <iostream>

#define DLP_PLATFORM_NOT_SETUP              "DLP_PLATFORM_NOT_SETUP"
#define DLP_PLATFORM_NULL_INPUT_ARGUMENT    "DLP_PLATFORM_NULL_INPUT_ARGUMENT"
#define DLP_PLATFORM_PATTERN_SEQUENCE_NOT_PREPARED  "DLP_PLATFORM_PATTERN_SEQUENCE_NOT_PREPARED"

/** @brief dlp is the namespace for all DLP SDK classes, enumerations, functions, etc. */
namespace dlp{


/** @class DLP_Platform
 *  @defgroup group_DLP_Platforms DLP_Platforms
    @brief Base class definition for interfacing with DLP based platforms

    DLP_Platform has several pure virtual functions so that all subclasses
    have the same basic interface and functionality.
*/
class DLP_Platform: public dlp::Module{
public:
    enum class Mirror{
        ORTHOGONAL,         /*!< The DLP Platform DMD has an orthogonal mirror array                            */
        DIAMOND,            /*!< The DLP Platform DMD has a diamond mirror array                                */
        INVALID             /*!< The DLP Platform has not been selected and the mirror array type is unknown    */
    };

    enum class Platform{
        LIGHTCRAFTER_3000,       /*!< The DLP Platform is based on the LightCrafter which features the DLPC300 controller, DLP3000 DMD, DM365, and FPGA */
        LIGHTCRAFTER_4500,  /*!< The DLP Platform is based on the LightCrafter 4500 which features the DLPC350 controller and DLP4500 DMD */
        LIGHTCRAFTER_6500,  /*!< The DLP Platform is based on the LightCrafter 6500 which features the DLPC900 controller and DLP6500 DMD */
        INVALID             /*!< The DLP Platform has not been selected */
    };


    class Parameters{
    public:
        DLP_NEW_PARAMETERS_ENTRY(Mirror,           "DLP_PLATFORM_PARAMETERS_MIRROR",   dlp::DLP_Platform::Mirror,dlp::DLP_Platform::Mirror::INVALID);
        DLP_NEW_PARAMETERS_ENTRY(Platform,         "DLP_PLATFORM_PARAMETERS_PLATFORM", dlp::DLP_Platform::Platform,dlp::DLP_Platform::Platform::INVALID);
        DLP_NEW_PARAMETERS_ENTRY(SequencePrepared, "DLP_PLATFORM_PARAMETERS_SEQUENCE_PREPARED",            bool,   false);
        DLP_NEW_PARAMETERS_ENTRY(SequenceExposure, "DLP_PLATFORM_PARAMETERS_SEQUENCE_EXPOSURE_US", unsigned int,       0);
        DLP_NEW_PARAMETERS_ENTRY(SequencePeriod,   "DLP_PLATFORM_PARAMETERS_SEQUENCE_PERIOD_US",   unsigned int,       0);
    };

    DLP_Platform();

    virtual ReturnCode Connect(std::string id) = 0;
    virtual ReturnCode Disconnect() = 0;
    virtual bool       isConnected() const = 0;

    virtual ReturnCode ProjectSolidWhitePattern() = 0;
    virtual ReturnCode ProjectSolidBlackPattern() = 0;

    virtual ReturnCode PreparePatternSequence(const dlp::Pattern::Sequence &pattern_sequence) = 0;
    virtual ReturnCode StartPatternSequence(const unsigned int &start, const unsigned int &patterns, const bool &repeat) = 0;
    virtual ReturnCode DisplayPatternInSequence(const unsigned int &pattern_index, const bool &repeat) = 0;
    virtual ReturnCode StopPatternSequence() = 0;

    bool isPlatformSetup() const;
    bool isSetup() const;

    bool ImageResolutionCorrect(const std::string &image_filename)const;
    bool ImageResolutionCorrect(const Image &image)const;

    // Used for scanner validation
    ReturnCode GetPlatform( Platform *platform) const;
    ReturnCode GetMirrorType( Mirror *mirror) const;
    ReturnCode GetEffectiveMirrorSize( float *mirror_size) const;
    ReturnCode GetPixelArrayConfiguration( PixelArrayConfiguration *mirror_array_config) const;

    ReturnCode GetRows( unsigned int *rows) const;
    ReturnCode GetColumns( unsigned int *columns) const;

    ReturnCode GetID(std::string *id ) const;

    static ReturnCode ConnectSetup(dlp::DLP_Platform &projector, std::string id, std::string parameters_file,     bool output_cmdline = false);
    static ReturnCode ConnectSetup(dlp::DLP_Platform &projector, std::string id, const dlp::Parameters &settings, bool output_cmdline = false);

protected:
    // Methods that subclasses call during setup
    void SetID(std::string id);

    // Should be called in object construction
    ReturnCode  SetPlatform(Platform arg_platform);

    Parameters::SequencePrepared sequence_prepared_;
    Parameters::SequenceExposure sequence_exposure_;
    Parameters::SequencePeriod   sequence_period_;

private:
    bool            is_platform_set_;
    std::string     id_;
    unsigned int    rows_;
    unsigned int    columns_;
    float           mirror_effective_size_um_;

    PixelArrayConfiguration mirror_array_config_;

    Parameters::Platform    platform_;
    Parameters::Mirror      mirror_;
};

namespace Number{
template <> std::string ToString<dlp::DLP_Platform::Mirror>( dlp::DLP_Platform::Mirror mirror );
template <> std::string ToString<dlp::DLP_Platform::Platform>( dlp::DLP_Platform::Platform platform );
}

namespace String{
template <> dlp::DLP_Platform::Mirror ToNumber( const std::string &text, unsigned int base );
template <> dlp::DLP_Platform::Platform ToNumber( const std::string &text, unsigned int base );
}



}

#endif

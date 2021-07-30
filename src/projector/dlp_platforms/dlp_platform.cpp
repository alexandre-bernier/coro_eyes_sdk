/** @file dlp_platform.cpp
 *  @brief Contains methods for DLP_Platform class
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <iostream>

#include <common/debug.hpp>
#include <common/returncode.hpp>
#include <common/image/image.hpp>
#include <common/other.hpp>
#include <common/parameters.hpp>

#include <dlp_platforms/dlp_platform.hpp>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{


/** @brief Constructs object */
DLP_Platform::DLP_Platform(){
    this->id_   = "";

    this->rows_     = 0;
    this->columns_  = 0;


    this->mirror_array_config_.columns = 0;
    this->mirror_array_config_.rows = 0;
    this->mirror_array_config_.compress_columns = 1.0;
    this->mirror_array_config_.compress_rows = 1.0;
    this->mirror_array_config_.shift_odd_columns = false;
    this->mirror_array_config_.shift_odd_rows = false;
    this->mirror_array_config_.shift_column_distance = 0.0;
    this->mirror_array_config_.shift_row_distance = 0.0;


    this->platform_.Set(Platform::INVALID);
    this->mirror_.Set(Mirror::INVALID);
    this->mirror_effective_size_um_ = 0.0;

    this->is_platform_set_ = false;
}


/**
 * @brief   Sets the object's \ref DLP_Platform::Platform
 * @retval  DLP_PLATFORM_NOT_SETUP  An invalid \ref DLP_Platform::Platform was supplied or has NOT been implemented in SDK
 */
ReturnCode DLP_Platform::SetPlatform(Platform arg_platform){
    ReturnCode ret;

    switch(arg_platform){
    case Platform::LIGHTCRAFTER_3000:
        this->platform_.Set(arg_platform);
        this->mirror_.Set(Mirror::DIAMOND);
        this->mirror_effective_size_um_ = 10.8;
        this->rows_     = 684;
        this->columns_  = 608;
        this->mirror_array_config_.columns = 608;
        this->mirror_array_config_.rows = 684;
        this->mirror_array_config_.compress_columns = 1.0;
        this->mirror_array_config_.compress_rows = 0.5;
        this->mirror_array_config_.shift_odd_columns = false;
        this->mirror_array_config_.shift_odd_rows = true;
        this->mirror_array_config_.shift_column_distance = 0.0;
        this->mirror_array_config_.shift_row_distance = 0.5;
        this->is_platform_set_    = true;
        break;
    case Platform::LIGHTCRAFTER_4500:
        this->platform_.Set(arg_platform);
        this->mirror_.Set(Mirror::DIAMOND);
        this->mirror_effective_size_um_ = 10.8;
        this->rows_     = 1140;
        this->columns_  = 912;
        this->mirror_array_config_.columns = 912;
        this->mirror_array_config_.rows = 1140;
        this->mirror_array_config_.compress_columns = 1.0;
        this->mirror_array_config_.compress_rows = 0.5;
        this->mirror_array_config_.shift_odd_columns = false;
        this->mirror_array_config_.shift_odd_rows = true;
        this->mirror_array_config_.shift_column_distance = 0.0;
        this->mirror_array_config_.shift_row_distance = 0.5;
        this->is_platform_set_    = true;
        break;
    case Platform::LIGHTCRAFTER_6500:
        this->platform_.Set(arg_platform);
        this->mirror_.Set(Mirror::ORTHOGONAL);
        this->mirror_effective_size_um_ = 7.56;
        this->rows_     = 1080;
        this->columns_  = 1920;
        this->mirror_array_config_.columns = 1920;
        this->mirror_array_config_.rows = 1080;
        this->mirror_array_config_.compress_columns = 1.0;
        this->mirror_array_config_.compress_rows = 1.0;
        this->mirror_array_config_.shift_odd_columns = false;
        this->mirror_array_config_.shift_odd_rows = false;
        this->mirror_array_config_.shift_column_distance = 0.0;
        this->mirror_array_config_.shift_row_distance = 0.0;
        this->is_platform_set_    = true;
        break;
    default:
        this->is_platform_set_ = false;
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}

/** @brief Saves the device identifier */
void DLP_Platform::SetID(std::string id){
    this->id_ = id;
}

/** @brief  Returns the object's identifier
 *  @retval  DLP_PLATFORM_NOT_SETUP  An invalid \ref DLP_Platform::Platform was supplied or has NOT been implemented in SDK
 */
ReturnCode DLP_Platform::GetID(std::string *id) const{
    ReturnCode ret;

    if(!this->isPlatformSetup()){
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    (*id) = this->id_;

    return ret;
}

/** @brief Returns true if SetPlatform() was previously called successfully */
bool DLP_Platform::isPlatformSetup() const{
    return this->is_platform_set_;
}

/** @brief Returns true if SetPlatform() and Setup() were previously called successfully */
bool DLP_Platform::isSetup() const{
    return (this->is_setup_ & this->is_platform_set_);
}

/** @brief Returns true if the supplied image file has the same resolution as the DLP_Platform object */
bool DLP_Platform::ImageResolutionCorrect(const std::string &image_filename) const{
    bool ret;
    Image test_image;

    // Load the image and check it it loaded correctly
    if(!test_image.Load(image_filename).hasErrors()){

        // Check the resolution
        this->debug_.Msg(1,"Image loaded, comparing DMD and Image resolution...");
        ret = this->ImageResolutionCorrect(test_image);

        // Clear the image
        test_image.Clear();
    }
    else{
        // Image did NOT load properly
        this->debug_.Msg(1,"Image failed to load, cannot compare DMD and Image resolution");
        ret = false;
    }
    // Return the value
    return ret;
}

/** @brief Returns true if the supplied \ref dlp::Image has the same resolution as the DLP_Platform object */
bool DLP_Platform::ImageResolutionCorrect(const Image &image)const{
    bool ret;
    unsigned int image_rows = 0;
    unsigned int image_cols = 0;
    unsigned int dmd_rows = 0;
    unsigned int dmd_cols = 0;

    // Get DMD columns and rows
    this->GetColumns(&dmd_cols);
    this->GetRows(&dmd_rows);

    this->debug_.Msg(1, "Compare DMD and Image resolution...");

    this->debug_.Msg(1, "DMD rows    = " + dlp::Number::ToString(dmd_rows));
    this->debug_.Msg(1, "DMD columns = " + dlp::Number::ToString(dmd_cols));

    // Get Flash Image columns and rows
    image.GetRows(&image_rows);
    image.GetColumns(&image_cols);


    this->debug_.Msg(1, "Image rows    = " + dlp::Number::ToString(image_rows));
    this->debug_.Msg(1, "Image columns = " + dlp::Number::ToString(image_cols));

    // Detmine if resoluations match
    ret = (image_rows == dmd_rows) && (image_cols == dmd_cols);

    if(ret){
        this->debug_.Msg(1, "DMD and Image resolutions match");
    }
    else{
        this->debug_.Msg(1, "DMD and Image resolutions do NOT match");
    }

    // Return value
    return ret;
}

/** @brief  Returns the object's \ref dlp::DLP_Platform::Platform
 *  @retval DLP_PLATFORM_NULL_INPUT_ARGUMENT    Input argument NULL
 *  @retval DLP_PLATFORM_NOT_SETUP              Object has NOT been setup
 */
ReturnCode DLP_Platform::GetPlatform(Platform *platform) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!platform)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);

    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*platform) = this->platform_.Get();
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}

/** @brief  Returns the object's \ref dlp::DLP_Platform::Mirror
 *  @retval DLP_PLATFORM_NULL_INPUT_ARGUMENT    Input argument NULL
 *  @retval DLP_PLATFORM_NOT_SETUP              Object has NOT been setup
 */
ReturnCode DLP_Platform::GetMirrorType(Mirror *mirror) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!mirror)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);

    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*mirror) = this->mirror_.Get();
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}


/** @brief  Returns the object's effective mirror size in micrometers (othogonal distance from pixel to pixel)
 *  @retval DLP_PLATFORM_NULL_INPUT_ARGUMENT    Input argument NULL
 *  @retval DLP_PLATFORM_NOT_SETUP              Object has NOT been setup
 */
ReturnCode DLP_Platform::GetEffectiveMirrorSize(float *mirror_size) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!mirror_size)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);

    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*mirror_size) = this->mirror_effective_size_um_;
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}


ReturnCode DLP_Platform::GetPixelArrayConfiguration( PixelArrayConfiguration *mirror_array_config) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!mirror_array_config)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);

    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*mirror_array_config) = this->mirror_array_config_;
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}

/** @brief  Returns the number of pixel rows (height)
 *  @retval DLP_PLATFORM_NULL_INPUT_ARGUMENT    Input argument NULL
 *  @retval DLP_PLATFORM_NOT_SETUP              Object has NOT been setup
 */
ReturnCode DLP_Platform::GetRows(unsigned int *rows) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!rows)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);

    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*rows) = this->rows_;
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}

/** @brief  Returns the number of pixel columns (width)
 *  @retval DLP_PLATFORM_NULL_INPUT_ARGUMENT    Input argument NULL
 *  @retval DLP_PLATFORM_NOT_SETUP              Object has NOT been setup
 */
ReturnCode DLP_Platform::GetColumns(unsigned int *columns) const{
    ReturnCode ret;

    // Check for NULL pointer
    if(!columns)
        return ret.AddError(DLP_PLATFORM_NULL_INPUT_ARGUMENT);


    // Check that DLP Platform has been setup
    if(this->isPlatformSetup()){
        (*columns) = this->columns_;
    }
    else{
        // Platform has NOT been setup
        ret.AddError(DLP_PLATFORM_NOT_SETUP);
    }

    return ret;
}

ReturnCode DLP_Platform::ConnectSetup(dlp::DLP_Platform &projector, std::string id, std::string parameters_file, bool output_cmdline){
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

    return ConnectSetup(projector, id, settings, output_cmdline);
}

ReturnCode DLP_Platform::ConnectSetup(dlp::DLP_Platform &projector, std::string id, const dlp::Parameters &settings, bool output_cmdline){
    dlp::ReturnCode ret;

    // Connect the projector
    dlp::CmdLine::Print("Connecting to projector...");
    ret = projector.Connect(id);
    if(ret.hasErrors()){
        if(output_cmdline){
            dlp::CmdLine::Print("Connecting to projector FAILED!");
            dlp::CmdLine::Print(ret.ToString());
        }
        return ret;
    }

    // Setup the projector
    dlp::CmdLine::Print("Configuring projector...");
    ret = projector.Setup(settings);
    if(ret.hasErrors()){
        if(output_cmdline){
            dlp::CmdLine::Print("Projector setup FAILED!");
            dlp::CmdLine::Print(ret.ToString());
        }
    }

    return ret;
}


namespace Number{
template <> std::string ToString<dlp::DLP_Platform::Mirror>( dlp::DLP_Platform::Mirror mirror ){
    switch(mirror){
    case dlp::DLP_Platform::Mirror::ORTHOGONAL: return "ORTHOGONAL";
    case dlp::DLP_Platform::Mirror::DIAMOND:    return "DIAMOND";
    case dlp::DLP_Platform::Mirror::INVALID:    return "INVALID";
    }
    return "INVALID";
}

template <> std::string ToString<dlp::DLP_Platform::Platform>( dlp::DLP_Platform::Platform platform ){
    switch(platform){
    case dlp::DLP_Platform::Platform::LIGHTCRAFTER_3000:    return "LIGHTCRAFTER_3000";
    case dlp::DLP_Platform::Platform::LIGHTCRAFTER_4500:    return "LIGHTCRAFTER_4500";
    case dlp::DLP_Platform::Platform::LIGHTCRAFTER_6500:    return "LIGHTCRAFTER_6500";
    case dlp::DLP_Platform::Platform::INVALID:              return "INVALID";
    }
    return "INVALID";
}
}

namespace String{

template <> dlp::DLP_Platform::Mirror ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("ORTHOGONAL") == 0){
        return dlp::DLP_Platform::Mirror::ORTHOGONAL;
    }
    else if (text.compare("DIAMOND") == 0){
        return dlp::DLP_Platform::Mirror::DIAMOND;
    }
    else{
        return dlp::DLP_Platform::Mirror::INVALID;
    }
}

template <> dlp::DLP_Platform::Platform ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("LIGHTCRAFTER_3000") == 0){
        return dlp::DLP_Platform::Platform::LIGHTCRAFTER_3000;
    }
    else if (text.compare("LIGHTCRAFTER_4500") == 0){
        return dlp::DLP_Platform::Platform::LIGHTCRAFTER_4500;
    }
    else if (text.compare("LIGHTCRAFTER_6500") == 0){
        return dlp::DLP_Platform::Platform::LIGHTCRAFTER_6500;
    }
    else{
        return dlp::DLP_Platform::Platform::INVALID;
    }
}
}


}

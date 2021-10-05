/** @file   pattern_sequence.cpp
 *  @brief  Contains \ref dlp::Pattern::Sequence methods
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <string>
#include <vector>
#include <iostream>

#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/image/image.hpp>
#include <projector/dlp/common/other.hpp>
#include <projector/dlp/common/parameters.hpp>
#include <projector/dlp/common/pattern/pattern.hpp>           // Adds dlp::Pattern

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @brief  Constructs empty sequence */
Pattern::Sequence::Sequence(){
    this->Clear();
}

/** @brief  Destroys object and deallocates any memory from the stored \ref dlp::Pattern objects */
Pattern::Sequence::~Sequence(){
    this->Clear();
}

/** @brief  Constructs sequence and adds the supplied \ref dlp::Pattern */
Pattern::Sequence::Sequence(const Pattern &pattern){
    this->Clear();
    this->Add(pattern);
}

/** @brief  Constructs sequence and copies all data from the supplied \ref dlp::Pattern::Sequence */
Pattern::Sequence::Sequence(const Sequence &pattern_seq){
    this->Clear();
    this->Add(pattern_seq);
}

/** @brief  Copies all \ref dlp::Pattern objects from the supplied sequence */
Pattern::Sequence& Pattern::Sequence::operator=(const Sequence& pattern_seq){
    this->Clear();
    this->Add(pattern_seq);
    return *this;
}

/** @brief  Returns the number of \ref dlp::Pattern objects stored in sequence */
unsigned int Pattern::Sequence::GetCount() const{
    return this->patterns_.size();
}

/** @brief  Removes all stored \ref dlp::Pattern objects from sequence */
void Pattern::Sequence::Clear(){
    this->patterns_.clear();
    this->parameters.Clear();
}

/** @brief  Stores a deep copy of the supplied pattern in the sequence
 *  @param[in] new_pattern  New \ref dlp::Pattern to add to sequence
 *  @retval PATTERN_BITDEPTH_INVALID    Input pattern does NOT have a set \ref dlp::Pattern::Bitdepth
 *  @retval PATTERN_COLOR_INVALID       Input pattern does NOT have a set \ref dlp::Pattern::color
 *  @retval PATTERN_DATA_TYPE_INVALID   Input pattern does NOT have a set \ref dlp::Pattern::data_type
 *  @retval FILE_DOES_NOT_EXIST         Input pattern set as \ref dlp::Pattern::image_file, but the image file does NOT exist.
 *  @retval PATTERN_IMAGE_DATA_EMPTY    Input pattern set as \ref dlp::Pattern::image_data, but the image data is empty
 *  @retval PATTERN_PARAMETERS_EMPTY    Input pattern set as \ref dlp::Pattern::parameters, but the parameter list is empty
 */
ReturnCode Pattern::Sequence::Add( const Pattern &new_pattern){
    ReturnCode ret;

    // Check that the pattern bitdepth is valid
    if(new_pattern.bitdepth == Bitdepth::INVALID)
        return ret.AddError(PATTERN_BITDEPTH_INVALID);

    // Check that the pattern color is valid
    if(new_pattern.color == Color::INVALID)
        return ret.AddError(PATTERN_COLOR_INVALID);

    // Check that the pattern type is valid
    if(new_pattern.data_type == DataType::INVALID)
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);

    // Check the type to ensure the data, filename, or parameters are present
    switch(new_pattern.data_type){
    case DataType::IMAGE_FILE:
        // Check that the file exists
        if(dlp::File::Exists(new_pattern.image_file) == false)
            return ret.AddError(FILE_DOES_NOT_EXIST);
        break;
    case DataType::IMAGE_DATA:
        // Check that the image is not empty
        if(new_pattern.image_data.isEmpty())
            return ret.AddError(PATTERN_IMAGE_DATA_EMPTY);
        break;
    case DataType::PARAMETERS:
        // Check that there are parameters
        if(new_pattern.parameters.GetCount() == 0)
            return ret.AddError(PATTERN_PARAMETERS_EMPTY);
        break;
    case DataType::INVALID:
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);
    }

    // Add pattern to the sequence
    this->patterns_.push_back(new_pattern);

    return ret;
}

/** @brief  Stores a deep copy of the supplied \ref dlp::Pattern::Sequence in the sequence */
void Pattern::Sequence::Add( const Sequence &sequence){
    for(unsigned int iPattern = 0; iPattern < sequence.GetCount(); iPattern++){
        Pattern temp;

        // Get a pattern
        sequence.Get(iPattern,&temp);

        // Save the pattern
        this->Add(temp);
    }

    // Copy parameters
    this->parameters = sequence.parameters;

    return;
}

/** @brief  Retrieves pattern from sequence and returns it with supplied pointer
 *  @retval PATTERN_SEQUENCE_NULL_POINTER_ARGUMENT    Input argument NULL
 *  @retval PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE       Requested index does NOT exist in sequence
 */
ReturnCode Pattern::Sequence::Get( const unsigned int &index, Pattern* ret_pattern) const{
    ReturnCode ret;

    // Check the pointer
    if(!ret_pattern)
        return ret.AddError(PATTERN_SEQUENCE_NULL_POINTER_ARGUMENT);

    // Check the index
    if( index >= this->patterns_.size())
        return ret.AddError(PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE);

    // Return the pattern
    (*ret_pattern) = this->patterns_.at(index);

    return ret;
}

/** @brief  Updates the \ref dlp::Pattern at the supplied index
 *  @retval PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE       Requested index does NOT exist in object
 *  @retval PATTERN_BITDEPTH_INVALID    Pattern NOT updated. Input pattern does NOT have a set \ref dlp::Pattern::Bitdepth
 *  @retval PATTERN_COLOR_INVALID       Pattern NOT updated. Input pattern does NOT have a set \ref dlp::Pattern::Color
 *  @retval PATTERN_DATA_TYPE_INVALID   Pattern NOT updated. Input pattern does NOT have a set \ref dlp::Pattern::DataType
 *  @retval FILE_DOES_NOT_EXIST         Pattern NOT updated. Input pattern set as \ref std::string, but the image file does NOT exist.
 *  @retval PATTERN_IMAGE_DATA_EMPTY    Pattern NOT updated. Input pattern set as \ref dlp::Image, but the image data is empty
 *  @retval PATTERN_PARAMETERS_EMPTY    Pattern NOT updated. Input pattern set as \ref dlp::Parameters, but the parameter list is empty
 */
ReturnCode Pattern::Sequence::Set(const unsigned int &index, Pattern &arg_pattern){
    ReturnCode ret;

    // Check the index
    if( index >= this->patterns_.size())
        return ret.AddError(PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE);

    // Check that the pattern bitdepth is valid
    if(arg_pattern.bitdepth == Bitdepth::INVALID)
        return ret.AddError(PATTERN_BITDEPTH_INVALID);

    // Check that the pattern color is valid
    if(arg_pattern.color == Color::INVALID)
        return ret.AddError(PATTERN_COLOR_INVALID);

    // Check that the pattern type is valid
    if(arg_pattern.data_type == DataType::INVALID)
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);

    // Check the type to ensure the data, filename, or parameters are present
    switch(arg_pattern.data_type){
    case DataType::IMAGE_FILE:
        // Check that the file exists
        if(dlp::File::Exists(arg_pattern.image_file) == false)
            return ret.AddError(FILE_DOES_NOT_EXIST);
        break;
    case DataType::IMAGE_DATA:
        // Check that the image is not empty
        if(arg_pattern.image_data.isEmpty())
            return ret.AddError(PATTERN_IMAGE_DATA_EMPTY);
        break;
    case DataType::PARAMETERS:
        // Check that there are parameters
        if(arg_pattern.parameters.GetCount() == 0)
            return ret.AddError(PATTERN_PARAMETERS_EMPTY);
        break;
    case DataType::INVALID:
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);
    }

    // Save the new pattern settings to the sequence
    this->patterns_.at(index) = arg_pattern;

    return ret;
}

/** @brief  Removes the pattern at the supplied index location
 *  @retval PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE     Requested index does NOT exist in sequence
 */
ReturnCode Pattern::Sequence::Remove(const unsigned int &index){
    ReturnCode ret;

    // Check the index
    if( index >= this->patterns_.size())
        return ret.AddError(PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE);

    // Remove the pattern
    this->patterns_.erase(this->patterns_.begin()+index);
    return ret;
}

/** @brief  Sets all patterns within the sequence to the supplied \ref dlp::Pattern::Bitdepth
 *  @retval PATTERN_BITDEPTH_INVALID    Sequence NOT updated. Input argument is NOT valid.
 */
ReturnCode Pattern::Sequence::SetBitDepths(const dlp::Pattern::Bitdepth &bitdepth){
    ReturnCode ret;

    if(bitdepth == Bitdepth::INVALID)
        return ret.AddError(PATTERN_BITDEPTH_INVALID);

    for(unsigned int iPat = 0; iPat < this->patterns_.size(); iPat++){
        // Set the new value
        this->patterns_.at(iPat).bitdepth = bitdepth;
    }

    return ret;
}

/** @brief  Sets all patterns within the sequence to the supplied \ref dlp::Pattern::Color
 *  @retval PATTERN_COLOR_INVALID    Sequence NOT updated. Input argument is NOT valid.
 */
ReturnCode Pattern::Sequence::SetColors(const dlp::Pattern::Color &color){
    ReturnCode ret;

    if(color == Color::INVALID)
        return ret.AddError(PATTERN_COLOR_INVALID);

    for(unsigned int iPat = 0; iPat < this->patterns_.size(); iPat++){
        // Set the new value
        this->patterns_.at(iPat).color = color;
    }

    return ret;
}

/** @brief  Sets all patterns within the sequence to the supplied pattern exposure
 *  @retval PATTERN_EXPOSURE_TOO_SHORT    Sequence NOT updated. Input argument is equal to 0.
 */
ReturnCode Pattern::Sequence::SetExposures(const unsigned int &exposure){
    ReturnCode ret;

    if(exposure == 0)
        ret.AddError(PATTERN_EXPOSURE_TOO_SHORT);

    for(unsigned int iPat = 0; iPat < this->patterns_.size(); iPat++){
        // Set the new value
        this->patterns_.at(iPat).exposure = exposure;
    }

    return ret;
}

/** @brief  Sets all patterns within the sequence to the supplied pattern period
 *  @retval PATTERN_PERIOD_TOO_SHORT    Sequence NOT updated. Input argument is equal to 0.
 */
ReturnCode Pattern::Sequence::SetPeriods(const unsigned int &period){
    ReturnCode ret;

    if(period == 0)
        ret.AddError(PATTERN_PERIOD_TOO_SHORT);

    for(unsigned int iPat = 0; iPat < this->patterns_.size(); iPat++){
        // Set the new value
        this->patterns_.at(iPat).period = period;
    }

    return ret;
}

/** Returns true if all patterns in sequence have identical \ref dlp::Pattern::Bitdepth */
bool Pattern::Sequence::EqualBitDepths() const{
    bool ret = true;
    Pattern::Bitdepth prev_bitdepth;

    // Check that sequence has at least one pattern
    if(this->patterns_.size() > 0){
        // Get first value
        prev_bitdepth = this->patterns_.at(0).bitdepth;

        for(unsigned int iPat = 0; (ret == true) && (iPat < this->patterns_.size()); iPat++){
            if(this->patterns_.at(iPat).bitdepth != prev_bitdepth)
                ret = false;
        }
    }
    return ret;
}

/** Returns true if all patterns in sequence have identical \ref dlp::Pattern::Color */
bool Pattern::Sequence::EqualColors() const{
    bool ret = true;
    Pattern::Color prev_color;

    // Check that sequence has at least one pattern
    if(this->patterns_.size() > 0){
        // Get first value
        prev_color = this->patterns_.at(0).color;

        for(unsigned int iPat = 0; (ret == true) && (iPat < this->patterns_.size()); iPat++){
            if(this->patterns_.at(iPat).color != prev_color)
                ret = false;
        }
    }
    return ret;
}

/** Returns true if all patterns in sequence have identical exposure time */
bool Pattern::Sequence::EqualExposures() const{
    bool ret = true;
    unsigned long prev_exposure;

    // Check that sequence has at least one pattern
    if(this->patterns_.size() > 0){
        // Get first value
        prev_exposure = this->patterns_.at(0).exposure;

        for(unsigned int iPat = 0; (ret == true) && (iPat < this->patterns_.size()); iPat++){
            if(this->patterns_.at(iPat).exposure != prev_exposure)
                ret = false;
        }
    }
    return ret;
}

/** Returns true if all patterns in sequence have identical period time */
bool Pattern::Sequence::EqualPeriods() const{
    bool ret = true;
    unsigned long prev_period;

    // Check that sequence has at least one pattern
    if(this->patterns_.size() > 0){
        // Get first value
        prev_period = this->patterns_.at(0).period;

        for(unsigned int iPat = 0; (ret == true) && (iPat < this->patterns_.size()); iPat++){
            if(this->patterns_.at(iPat).period != prev_period)
                ret = false;
        }
    }
    return ret;
}

/** Returns true if all patterns in sequence have identical \ref dlp::Pattern::DataType */
bool Pattern::Sequence::EqualDataTypes() const{
    bool ret = true;
    Pattern::DataType prev_type;

    // Check that sequence has at least one pattern
    if(this->patterns_.size() > 0){
        // Get first value
        prev_type = this->patterns_.at(0).data_type;

        for(unsigned int iPat = 0; (ret == true) && (iPat < this->patterns_.size()); iPat++){
            if(this->patterns_.at(iPat).data_type != prev_type)
                ret = false;
        }
    }
    return ret;
}

}


/** @file   pattern.cpp
 *  @brief  Contains dlp::Pattern methods
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <string>
#include <vector>
#include <iostream>

#include <common/debug.hpp>
#include <common/returncode.hpp>
#include <common/image/image.hpp>
#include <common/other.hpp>
#include <common/parameters.hpp>
#include <common/pattern/pattern.hpp>           // Adds dlp::Pattern

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @brief  Object constructor */
Pattern::Pattern(){
    this->id        = 0;
    this->exposure  = 0;
    this->period    = 0;
    this->bitdepth  = Bitdepth::INVALID;
    this->color     = Color::INVALID;
    this->data_type = DataType::INVALID;
    this->image_file = "";
    this->image_data.Clear();
    this->parameters.Clear();
}

/** @brief  Destroys object and releases image memory */
Pattern::~Pattern(){
    image_data.Clear();
    parameters.Clear();
}

/** @brief  Constructs and initializes object with data copied from supplied Pattern */
Pattern::Pattern(const Pattern &pattern){
    this->id         = pattern.id;
    this->exposure   = pattern.exposure;
    this->period     = pattern.period;
    this->bitdepth   = pattern.bitdepth;
    this->color      = pattern.color;
    this->data_type  = pattern.data_type;
    this->image_file = pattern.image_file;
    this->image_data.Clear();
    this->image_data.Create(pattern.image_data);
    this->parameters = pattern.parameters;
}

/** Copies all data (deep) from supplied Pattern */
Pattern& Pattern::operator=(const Pattern& pattern){
    this->id         = pattern.id;
    this->exposure   = pattern.exposure;
    this->period     = pattern.period;
    this->bitdepth   = pattern.bitdepth;
    this->color      = pattern.color;
    this->data_type  = pattern.data_type;
    this->image_file = pattern.image_file;
    this->image_data.Clear();
    this->image_data.Create(pattern.image_data);
    this->parameters = pattern.parameters;
    return *this;
}

namespace Number{
template <> std::string ToString<dlp::Pattern::Bitdepth>( dlp::Pattern::Bitdepth bitdepth ){
    switch(bitdepth){
    case dlp::Pattern::Bitdepth::INVALID:   return "INVALID";
    case dlp::Pattern::Bitdepth::MONO_1BPP: return "MONO_1BPP";
    case dlp::Pattern::Bitdepth::MONO_2BPP: return "MONO_2BPP";
    case dlp::Pattern::Bitdepth::MONO_3BPP: return "MONO_3BPP";
    case dlp::Pattern::Bitdepth::MONO_4BPP: return "MONO_4BPP";
    case dlp::Pattern::Bitdepth::MONO_5BPP: return "MONO_5BPP";
    case dlp::Pattern::Bitdepth::MONO_6BPP: return "MONO_6BPP";
    case dlp::Pattern::Bitdepth::MONO_7BPP: return "MONO_7BPP";
    case dlp::Pattern::Bitdepth::MONO_8BPP: return "MONO_8BPP";
    case dlp::Pattern::Bitdepth::RGB_3BPP:  return "RGB_3BPP";
    case dlp::Pattern::Bitdepth::RGB_6BPP:  return "RGB_6BPP";
    case dlp::Pattern::Bitdepth::RGB_9BPP:  return "RGB_9BPP";
    case dlp::Pattern::Bitdepth::RGB_12BPP: return "RGB_12BPP";
    case dlp::Pattern::Bitdepth::RGB_15BPP: return "RGB_15BPP";
    case dlp::Pattern::Bitdepth::RGB_18BPP: return "RGB_18BPP";
    case dlp::Pattern::Bitdepth::RGB_21BPP: return "RGB_21BPP";
    case dlp::Pattern::Bitdepth::RGB_24BPP: return "RGB_24BPP";
    }
    return "INVALID";
}

template <> std::string ToString<dlp::Pattern::Color>( dlp::Pattern::Color color ){
    switch(color){
    case dlp::Pattern::Color::INVALID:  return "INVALID";
    case dlp::Pattern::Color::NONE:     return "NONE";
    case dlp::Pattern::Color::BLACK:    return "BLACK";
    case dlp::Pattern::Color::RED:      return "RED";
    case dlp::Pattern::Color::GREEN:    return "GREEN";
    case dlp::Pattern::Color::BLUE:     return "BLUE";
    case dlp::Pattern::Color::CYAN:     return "CYAN";
    case dlp::Pattern::Color::YELLOW:   return "YELLOW";
    case dlp::Pattern::Color::MAGENTA:  return "MAGENTA";
    case dlp::Pattern::Color::WHITE:    return "WHITE";
    case dlp::Pattern::Color::RGB:      return "RGB";
    }
    return "INVALID";
}
template <> std::string ToString<dlp::Pattern::DataType>( dlp::Pattern::DataType data_type ){
    switch(data_type){
    case dlp::Pattern::DataType::INVALID:       return "INVALID";
    case dlp::Pattern::DataType::IMAGE_FILE:    return "IMAGE_FILE";
    case dlp::Pattern::DataType::IMAGE_DATA:    return "IMAGE_DATA";
    case dlp::Pattern::DataType::PARAMETERS:    return "PARAMETERS";
    }
    return "INVALID";
}

template <> std::string ToString<dlp::Pattern::Orientation>( dlp::Pattern::Orientation orientation ){
    switch(orientation){
    case dlp::Pattern::Orientation::VERTICAL:           return "VERTICAL";
    case dlp::Pattern::Orientation::HORIZONTAL:         return "HORIZONTAL";
    case dlp::Pattern::Orientation::DIAMOND_ANGLE_1:    return "DIAMOND_ANGLE_1";
    case dlp::Pattern::Orientation::DIAMOND_ANGLE_2:    return "DIAMOND_ANGLE_2";
    case dlp::Pattern::Orientation::INVALID:            return "INVALID";
    }
    return "INVALID";
}

}

namespace String{
template <> dlp::Pattern::Bitdepth ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("MONO_1BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_1BPP;
    }
    else if (text.compare("MONO_2BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_2BPP;
    }
    else if (text.compare("MONO_3BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_3BPP;
    }
    else if (text.compare("MONO_4BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_4BPP;
    }
    else if (text.compare("MONO_5BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_5BPP;
    }
    else if (text.compare("MONO_6BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_6BPP;
    }
    else if (text.compare("MONO_7BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_7BPP;
    }
    else if (text.compare("MONO_8BPP") == 0){
        return dlp::Pattern::Bitdepth::MONO_8BPP;
    }
    else if (text.compare("RGB_3BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_3BPP;
    }
    else if (text.compare("RGB_6BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_6BPP;
    }
    else if (text.compare("RGB_9BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_9BPP;
    }
    else if (text.compare("RGB_12BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_12BPP;
    }
    else if (text.compare("RGB_15BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_15BPP;
    }
    else if (text.compare("RGB_18BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_18BPP;
    }
    else if (text.compare("RGB_21BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_21BPP;
    }
    else if (text.compare("RGB_24BPP") == 0){
        return dlp::Pattern::Bitdepth::RGB_24BPP;
    }
    else{
        return dlp::Pattern::Bitdepth::INVALID;
    }
}

template <> dlp::Pattern::Color ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("NONE") == 0){
        return dlp::Pattern::Color::NONE;
    }
    else if (text.compare("BLACK") == 0){
        return dlp::Pattern::Color::BLACK;
    }
    else if (text.compare("RED") == 0){
        return dlp::Pattern::Color::RED;
    }
    else if (text.compare("GREEN") == 0){
        return dlp::Pattern::Color::GREEN;
    }
    else if (text.compare("BLUE") == 0){
        return dlp::Pattern::Color::BLUE;
    }
    else if (text.compare("CYAN") == 0){
        return dlp::Pattern::Color::CYAN;
    }
    else if (text.compare("YELLOW") == 0){
        return dlp::Pattern::Color::YELLOW;
    }
    else if (text.compare("MAGENTA") == 0){
        return dlp::Pattern::Color::MAGENTA;
    }
    else if (text.compare("WHITE") == 0){
        return dlp::Pattern::Color::WHITE;
    }
    else if (text.compare("RGB") == 0){
        return dlp::Pattern::Color::RGB;
    }
    else{
        return dlp::Pattern::Color::INVALID;
    }
}

template <> dlp::Pattern::DataType ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("IMAGE_FILE") == 0){
        return dlp::Pattern::DataType::IMAGE_FILE;
    }
    else if (text.compare("IMAGE_DATA") == 0){
        return dlp::Pattern::DataType::IMAGE_DATA;
    }
    else if (text.compare("PARAMETERS") == 0){
        return dlp::Pattern::DataType::PARAMETERS;
    }
    else{
        return dlp::Pattern::DataType::INVALID;
    }
}

template <> dlp::Pattern::Orientation ToNumber( const std::string &text, unsigned int base ){
    // Ignore base variable
    if (text.compare("VERTICAL") == 0){
        return dlp::Pattern::Orientation::VERTICAL;
    }
    else if (text.compare("HORIZONTAL") == 0){
        return dlp::Pattern::Orientation::HORIZONTAL;
    }
    else if (text.compare("DIAMOND_ANGLE_1") == 0){
        return dlp::Pattern::Orientation::DIAMOND_ANGLE_1;
    }
    else if (text.compare("DIAMOND_ANGLE_2") == 0){
        return dlp::Pattern::Orientation::DIAMOND_ANGLE_2;
    }
    else{
        return dlp::Pattern::Orientation::INVALID;
    }
}

}





}


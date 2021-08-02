/** @file       pattern.hpp
 *  @ingroup    Common
 *  @brief      Contains \ref dlp::Pattern and \ref dlp::Pattern::Sequence
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_PATTERN_HPP
#define DLP_SDK_PATTERN_HPP

#include <common/debug.hpp>
#include <common/returncode.hpp>
#include <common/image/image.hpp>
#include <common/other.hpp>
#include <common/parameters.hpp>

#include <string>
#include <vector>
#include <iostream>

#define PATTERN_BITDEPTH_INVALID        "PATTERN_BITDEPTH_INVALID"
#define PATTERN_COLOR_INVALID           "PATTERN_COLOR_INVALID"
#define PATTERN_DATA_TYPE_INVALID       "PATTERN_DATA_TYPE_INVALID"
#define PATTERN_EXPOSURE_INVALID        "PATTERN_EXPOSURE_INVALID"
#define PATTERN_EXPOSURE_TOO_SHORT      "PATTERN_EXPOSURE_TOO_SHORT"
#define PATTERN_EXPOSURE_TOO_LONG       "PATTERN_EXPOSURE_TOO_LONG"
#define PATTERN_PERIOD_TOO_SHORT        "PATTERN_PERIOD_TOO_SHORT"
#define PATTERN_PERIOD_TOO_LONG         "PATTERN_PERIOD_TOO_LONG"
#define PATTERN_PARAMETERS_EMPTY        "PATTERN_PARAMETERS_EMPTY"
#define PATTERN_IMAGE_DATA_EMPTY        "PATTERN_IMAGE_DATA_EMPTY"
#define PATTERN_IMAGE_FILE_EMPTY        "PATTERN_IMAGE_FILE_EMPTY"

#define PATTERN_SEQUENCE_EMPTY                      "PATTERN_SEQUENCE_EMPTY"
#define PATTERN_SEQUENCE_TOO_LONG                   "PATTERN_SEQUENCE_TOO_LONG"
#define PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE         "PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE"
#define PATTERN_SEQUENCE_BITDEPTHS_NOT_EQUAL        "PATTERN_SEQUENCE_BITDEPTHS_NOT_EQUAL"
#define PATTERN_SEQUENCE_COLORS_NOT_EQUAL           "PATTERN_SEQUENCE_COLORS_NOT_EQUAL"
#define PATTERN_SEQUENCE_EXPOSURES_NOT_EQUAL        "PATTERN_SEQUENCE_EXPOSURES_NOT_EQUAL"
#define PATTERN_SEQUENCE_PERIODS_NOT_EQUAL          "PATTERN_SEQUENCE_PERIODS_NOT_EQUAL"
#define PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL    "PATTERN_SEQUENCE_bPATTERN_TYPES_NOT_EQUAL"
#define PATTERN_SEQUENCE_NULL_POINTER_ARGUMENT      "PATTERN_SEQUENCE_NULL_POINTER_ARGUMENT"


/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{


/** @class      Pattern
 *  @ingroup    Common
 *  @brief      Container class for \ref dlp::Image data, image file name, or
 *              \ref dlp::Parameters transfer between \ref dlp::DLP_Platform,
 *              and \ref dlp::StructuredLight classes.
 */
class Pattern{
public:

    /** @class Bitdepth
     *  @brief This object sets the pattern bit depth (i.e. pixel value range)
     */
    enum class Bitdepth{
        MONO_1BPP,      /**< Monochrome pattern with pixel values from 0 to 1 (also known as a binary pattern) */
        MONO_2BPP,      /**< Monochrome pattern with pixel values from 0 to 3   */
        MONO_3BPP,      /**< Monochrome pattern with pixel values from 0 to 7   */
        MONO_4BPP,      /**< Monochrome pattern with pixel values from 0 to 15  */
        MONO_5BPP,      /**< Monochrome pattern with pixel values from 0 to 31  */
        MONO_6BPP,      /**< Monochrome pattern with pixel values from 0 to 63  */
        MONO_7BPP,      /**< Monochrome pattern with pixel values from 0 to 127 */
        MONO_8BPP,      /**< Monochrome pattern with pixel values from 0 to 255 */
        RGB_3BPP,       /**< Color pattern created from three sequential MONO_1BPP patterns (red, green, and blue) */
        RGB_6BPP,       /**< Color pattern created from three sequential MONO_2BPP patterns (red, green, and blue) */
        RGB_9BPP,       /**< Color pattern created from three sequential MONO_3BPP patterns (red, green, and blue) */
        RGB_12BPP,      /**< Color pattern created from three sequential MONO_4BPP patterns (red, green, and blue) */
        RGB_15BPP,      /**< Color pattern created from three sequential MONO_5BPP patterns (red, green, and blue) */
        RGB_18BPP,      /**< Color pattern created from three sequential MONO_6BPP patterns (red, green, and blue) */
        RGB_21BPP,      /**< Color pattern created from three sequential MONO_7BPP patterns (red, green, and blue) */
        RGB_24BPP,      /**< Color pattern created from three sequential MONO_8BPP patterns (red, green, and blue) */
        INVALID
    };

    /** @class Color
     *  @brief This object sets which color LEDs are used for the pattern
     */
    enum class Color{
        NONE,           /**< No LED on      */
        BLACK,          /**< No LED on      */
        RED,            /**< Red LED on     */
        GREEN,          /**< Green LED on   */
        BLUE,           /**< Blue LED on    */
        CYAN,           /**< Green and Blue LEDs on simultaneously      */
        YELLOW,         /**< Red and Green LEDs on simultaneously       */
        MAGENTA,        /**< Red and Blue LEDs on simultaneously        */
        WHITE,          /**< Red, Green and Blue LEDs on simultaneously */
        RGB,            /**< Red, Green and Blue LEDs on sequentially   */
        INVALID
    };


    /** @class DataType
     *  @brief Dictates if the Pattern uses image_data, image_file, or
     *         parameters are used
     */
    enum class DataType{
        IMAGE_FILE,     /**< Pattern data stored in \ref dlp::Image with \ref image_data */
        IMAGE_DATA,     /**< Pattern data stored in image file with \ref image_file      */
        PARAMETERS,     /**< Pattern information stored in \ref parameters               */
        INVALID
    };


    /** @class Orientation
     *  @brief Determines if the pattern data is vertical or horizontal
     *         for disparity decoding
     * */
    enum class Orientation{
        VERTICAL,
        HORIZONTAL,
        DIAMOND_ANGLE_1,
        DIAMOND_ANGLE_2,
        INVALID
    };


    /** @class      Sequence
     *  @ingroup    Common
     *  @brief      Container class used to group multiple \ref dlp::Pattern objects
     */
    class Sequence{
    public:
        Sequence();
        ~Sequence();
        Sequence(const Pattern &pattern);
        Sequence(const Sequence &pattern_seq);
        Sequence& operator=(const Sequence& pattern_seq);

        unsigned int GetCount() const;
        void Clear();

        ReturnCode Add( const Pattern &new_pattern);
        void Add( const Sequence &sequence);

        ReturnCode Get(const unsigned int &index, Pattern* ret_pattern) const;
        ReturnCode Set(      const unsigned int &index, Pattern &arg_pattern);
        ReturnCode Remove(   const unsigned int &index);

        ReturnCode SetBitDepths(const dlp::Pattern::Bitdepth &bitdepth);
        ReturnCode SetColors(const dlp::Pattern::Color &color);
        ReturnCode SetExposures(const unsigned int &exposure);
        ReturnCode SetPeriods(const unsigned int &period);

        bool EqualBitDepths() const;
        bool EqualColors() const;
        bool EqualDataTypes() const;
        bool EqualExposures() const;
        bool EqualPeriods() const;

        Parameters parameters;

    private:
        std::vector<dlp::Pattern> patterns_;
    };


    Pattern();
    ~Pattern();
    Pattern(const Pattern &pattern);
    Pattern& operator=(const Pattern& pattern);

//    operator Sequence(){
//        dlp::Pattern::Sequence sequence;

//        sequence.Add(this);

//        return sequence;
//    }

    // Pattern information
    int                 id;                 /*!< None required pattern indentifier value                */
    unsigned long long  exposure;           /*!< Exposure time in microseconds (pattern display time)   */
    unsigned long long  period;             /*!< Period time in microseconds (time between patterns)    */
    Bitdepth            bitdepth;
    Color               color;
    DataType            data_type;
    Orientation         orientation;

    // Pattern data
    dlp::Parameters     parameters;
    dlp::Image          image_data;
    std::string         image_file;

};

namespace Number{
template <> std::string ToString<dlp::Pattern::Bitdepth>( dlp::Pattern::Bitdepth bitdepth );
template <> std::string ToString<dlp::Pattern::Color>(    dlp::Pattern::Color    color );
template <> std::string ToString<dlp::Pattern::DataType>( dlp::Pattern::DataType data_type );
template <> std::string ToString<dlp::Pattern::Orientation>( dlp::Pattern::Orientation orientation );
}

namespace String{
template <> dlp::Pattern::Bitdepth  ToNumber( const std::string &text, unsigned int base );
template <> dlp::Pattern::Color     ToNumber( const std::string &text, unsigned int base );
template <> dlp::Pattern::DataType  ToNumber( const std::string &text, unsigned int base );
template <> dlp::Pattern::Orientation ToNumber( const std::string &text, unsigned int base );
}

}


#endif // DLP_SDK_PATTERN_HPP

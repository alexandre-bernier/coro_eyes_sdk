/*! @file       pixel_array_configuration.hpp
 *  @ingroup    Common
 *  @brief      Defines ReturnCode class for all SDK modules for errors and warning messages
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_PIXEL_ARRAY_CONFIGURATION_HPP
#define DLP_SDK_PIXEL_ARRAY_CONFIGURATION_HPP

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

struct PixelArrayConfiguration{
    unsigned int rows;
    unsigned int columns;
    double compress_rows;           // in pixels
    double compress_columns;        // in pixels
    bool shift_odd_rows;
    bool shift_odd_columns;
    double shift_row_distance;      // in pixels
    double shift_column_distance;   // in pixels
};

}

#endif //#ifndef DLP_SDK_PIXEL_ARRAY_CONFIGURATION_HPP












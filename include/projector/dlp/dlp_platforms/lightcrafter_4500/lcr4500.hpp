/** @file       lcr4500.hpp
 *  @brief      Contains definitions for the DLP SDK LightCrafter 4500 class
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_LCR4500_HPP
#define DLP_LCR4500_HPP

#include <atomic>
#include <string>

#include <common/returncode.hpp>
#include <common/other.hpp>
#include <common/parameters.hpp>
#include <common/pattern/pattern.hpp>
#include <common/image/image.hpp>

#include <dlp_platforms/dlp_platform.hpp>
#include <dlp_platforms/lightcrafter_4500/flashdevice.hpp>

#define LCR4500_FLASH_FW_VERSION_ADDRESS    0xF902C000

#define LCR4500_TRUE            1
#define LCR4500_FALSE           0

#define LCR4500_TRIGGER_OUT_1   1
#define LCR4500_TRIGGER_OUT_2   2

#define LCR4500_COMMAND_FAILED  -1

#define LCR4500_PATTERN_SEQUENCE_VALIDATION_FAILED          "LCR4500_PATTERN_SEQUENCE_VALIDATION_FAILED"
#define LCR4500_PATTERN_SEQUENCE_START_FAILED               "LCR4500_PATTERN_SEQUENCE_START_FAILED"

#define LCR4500_CONNECTION_FAILED                           "LCR4500_CONNECTION_FAILED"
#define LCR4500_NOT_CONNECTED                               "LCR4500_NOT_CONNECTED"
#define LCR4500_SETUP_FAILURE                               "LCR4500_SETUP_FAILURE"

#define LCR4500_SETUP_POWER_STANDBY_FAILED                  "LCR4500_SETUP_POWER_STANDBY_FAILED"
#define LCR4500_SETUP_SHORT_AXIS_FLIP_FAILED                "LCR4500_SETUP_SHORT_AXIS_FLIP_FAILED"
#define LCR4500_SETUP_LONG_AXIS_FLIP_FAILED                 "LCR4500_SETUP_LONG_AXIS_FLIP_FAILED"
#define LCR4500_SETUP_LED_SEQUENCE_AND_ENABLES_FAILED       "LCR4500_SETUP_LED_SEQUENCE_AND_ENABLES_FAILED"
#define LCR4500_SETUP_INVERT_LED_PWM_FAILED                 "LCR4500_SETUP_INVERT_LED_PWM_FAILED"
#define LCR4500_SETUP_LED_CURRENTS_FAILED                   "LCR4500_SETUP_LED_CURRENTS"
#define LCR4500_SETUP_LED_RED_EDGE_DELAYS_FAILED            "LCR4500_SETUP_LED_RED_EDGE_DELAYS_FAILED"
#define LCR4500_SETUP_LED_GREEN_EDGE_DELAYS_FAILED          "LCR4500_SETUP_LED_GREEN_EDGE_DELAYS_FAILED"
#define LCR4500_SETUP_LED_BLUE_EDGE_DELAYS_FAILED           "LCR4500_SETUP_LED_BLUE_EDGE_DELAYS_FAILED"
#define LCR4500_SETUP_INPUT_SOURCE_FAILED                   "LCR4500_SETUP_INPUT_SOURCE_FAILED"
#define LCR4500_SETUP_PARALLEL_PORT_CLOCK_FAILED            "LCR4500_SETUP_PARALLEL_PORT_CLOCK_FAILED"
#define LCR4500_SETUP_DATA_SWAP_FAILED                      "LCR4500_SETUP_DATA_SWAP_FAILED"
#define LCR4500_SETUP_INVERT_DATA_FAILED                    "LCR4500_SETUP_INVERT_DATA_FAILED"
#define LCR4500_SETUP_DISPLAY_MODE_FAILED                   "LCR4500_SETUP_DISPLAY_MODE_FAILED"
#define LCR4500_SETUP_TEST_PATTERN_COLOR_FAILED             "LCR4500_SETUP_TEST_PATTERN_COLOR_FAILED"
#define LCR4500_SETUP_TEST_PATTERN_FAILED                   "LCR4500_SETUP_TEST_PATTERN_FAILED"
#define LCR4500_SETUP_FLASH_IMAGE_FAILED                    "LCR4500_SETUP_FLASH_IMAGE_FAILED"
#define LCR4500_SETUP_TRIGGER_INPUT_1_DELAY_FAILED          "LCR4500_SETUP_TRIGGER_INPUT_1_DELAY_FAILED"
#define LCR4500_SETUP_TRIGGER_OUTPUT_1_FAILED               "LCR4500_SETUP_TRIGGER_OUTPUT_1_FAILED"
#define LCR4500_SETUP_TRIGGER_OUTPUT_2_FAILED               "LCR4500_SETUP_TRIGGER_OUTPUT_2_FAILED"

#define LCR4500_PATTERN_SEQUENCE_NOT_PREPARED               "LCR4500_PATTERN_SEQUENCE_NOT_PREPARED"

#define LCR4500_IMAGE_RESOLUTION_INVALID                    "LCR4500_IMAGE_RESOLUTION_INVALID"
#define LCR4500_IMAGE_FORMAT_INVALID                        "LCR4500_IMAGE_FORMAT_INVALID"

#define LCR4500_PATTERN_NUMBER_PARAMETER_MISSING            "LCR4500_PATTERN_NUMBER_PARAMETER_MISSING"
#define LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING       "LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING"


#define LCR4500_PATTERN_SEQUENCE_BUFFERSWAP_TIME_ERROR      "LCR4500_PATTERN_SEQUENCE_BUFFERSWAP_TIME_ERROR"

#define LCR4500_IMAGE_FILE_FORMAT_INVALID                   "LCR4500_IMAGE_FILE_FORMAT_INVALID"
#define LCR4500_IMAGE_LIST_TOO_LONG                         "LCR4500_IMAGE_LIST_TOO_LONG"
#define LCR4500_IMAGE_MEMORY_ALLOCATION_FAILED              "LCR4500_IMAGE_MEMORY_ALLOCATION_FAILED"
#define LCR4500_FLASH_IMAGE_INDEX_INVALID                   "LCR4500_FLASH_IMAGE_INDEX_INVALID"
#define LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS                 "LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS"
#define LCR4500_FIRMWARE_FILE_INVALID                       "LCR4500_FIRMWARE_FILE_INVALID"
#define LCR4500_FIRMWARE_FILE_NOT_FOUND                     "LCR4500_FIRMWARE_FILE_NOT_FOUND"
#define LCR4500_FIRMWARE_FILE_NAME_INVALID                  "LCR4500_FIRMWARE_FILE_NAME_INVALID"
#define LCR4500_DLPC350_FIRMWARE_FILE_NOT_FOUND             "LCR4500_DLPC350_FIRMWARE_FILE_NOT_FOUND"
#define LCR4500_FIRMWARE_FLASH_PARAMETERS_FILE_NOT_FOUND    "LCR4500_FIRMWARE_FLASH_PARAMETERS_FILE_NOT_FOUND"
#define LCR4500_UNABLE_TO_ENTER_PROGRAMMING_MODE            "LCR4500_UNABLE_TO_ENTER_PROGRAMMING_MODE"
#define LCR4500_GET_FLASH_MANUFACTURER_ID_FAILED            "LCR4500_GET_FLASH_MANUFACTURER_ID_FAILED"
#define LCR4500_GET_FLASH_DEVICE_ID_FAILED                  "LCR4500_GET_FLASH_DEVICE_ID_FAILED"
#define LCR4500_FLASHDEVICE_PARAMETERS_NOT_FOUND            "LCR4500_FLASHDEVICE_PARAMETERS_NOT_FOUND"
#define LCR4500_FIRMWARE_FLASH_ERASE_FAILED                 "LCR4500_FIRMWARE_FLASH_ERASE_FAILED"
#define LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED           "LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED"
#define LCR4500_FIRMWARE_NOT_ENOUGH_MEMORY                  "LCR4500_FIRMWARE_NOT_ENOUGH_MEMORY"
#define LCR4500_FIRMWARE_UPLOAD_FAILED                      "LCR4500_FIRMWARE_UPLOAD_FAILED"
#define LCR4500_FIRMWARE_CHECKSUM_VERIFICATION_FAILED       "LCR4500_FIRMWARE_CHECKSUM_VERIFICATION_FAILED"
#define LCR4500_FIRMWARE_CHECKSUM_MISMATCH                  "LCR4500_FIRMWARE_CHECKSUM_MISMATCH"
#define LCR4500_FIRMWARE_IMAGE_BASENAME_EMPTY               "LCR4500_FIRMWARE_IMAGE_BASENAME_EMPTY"
#define LCR4500_NULL_POINT_ARGUMENT_PARAMETERS              "LCR4500_NULL_POINT_ARGUMENT_PARAMETERS"
#define LCR4500_NULL_POINT_ARGUMENT_MINIMUM_EXPOSURE        "LCR4500_NULL_POINT_ARGUMENT_MINIMUM_EXPOSURE"
#define LCR4500_MEASURE_FLASH_LOAD_TIMING_FAILED            "LCR4500_MEASURE_FLASH_LOAD_TIMING_FAILED"
#define LCR4500_READ_FLASH_LOAD_TIMING_FAILED               "LCR4500_READ_FLASH_LOAD_TIMING_FAILED"

#define LCR4500_GET_STATUS_FAILED                           "LCR4500_GET_STATUS_FAILED"
#define LCR4500_GET_OPERATING_MODE_FAILED                   "LCR4500_GET_OPERATING_MODE_FAILED"

#define LCR4500_SEQUENCE_VALIDATION_FAILED                      "LCR4500_SEQUENCE_VALIDATION_FAILED"
#define LCR4500_SEQUENCE_VALIDATION_EXP_OR_PERIOD_OOO           "LCR4500_SEQUENCE_VALIDATION_EXP_OR_PERIOD_OOO"
#define LCR4500_SEQUENCE_VALIDATION_PATTERN_NUMBER_INVALID      "LCR4500_SEQUENCE_VALIDATION_PATTERN_NUMBER_INVALID"
#define LCR4500_SEQUENCE_VALIDATION_OVERLAP_BLACK               "LCR4500_SEQUENCE_VALIDATION_OVERLAP_BLACK"
#define LCR4500_SEQUENCE_VALIDATION_BLACK_MISSING               "LCR4500_SEQUENCE_VALIDATION_BLACK_MISSING"
#define LCR4500_SEQUENCE_VALIDATION_EXP_PERIOD_DELTA_INVALID    "LCR4500_SEQUENCE_VALIDATION_EXP_PERIOD_DELTA_INVALID"

#define LCR4500_PATTERN_DISPLAY_FAILED                      "LCR4500_PATTERN_DISPLAY_FAILED"

#define LCR4500_ADD_EXP_LUT_ENTRY_FAILED                    "LCR4500_ADD_EXP_LUT_ENTRY_FAILED"

#define LCR4500_SEND_EXP_PATTERN_LUT_FAILED                 "LCR4500_SEND_EXP_PATTERN_LUT_FAILED"
#define LCR4500_SEND_EXP_IMAGE_LUT_FAILED                   "LCR4500_SEND_EXP_IMAGE_LUT_FAILED"

#define LCR4500_SET_POWER_MODE_FAILED                       "LCR4500_SET_POWER_MODE_FAILED"
#define LCR4500_SET_OPERATING_MODE_FAILED                   "LCR4500_SET_OPERATING_MODE_FAILED"
#define LCR4500_SET_TRIGGER_OUTPUT_CONFIG_FAILED            "LCR4500_SET_TRIGGER_OUTPUT_CONFIG_FAILED"
#define LCR4500_SET_PATTERN_DISPLAY_MODE_FAILED             "LCR4500_SET_PATTERN_DISPLAY_MODE_FAILED"
#define LCR4500_SET_PATTERN_TRIGGER_MODE_FAILED             "LCR4500_SET_PATTERN_TRIGGER_MODE_FAILED"
#define LCR4500_SET_VAR_EXP_PATTERN_CONFIG_FAILED           "LCR4500_SET_VAR_EXP_PATTERN_CONFIG_FAILED"


#define LCR4500_INVALID_ID  "LCR4500_INVALID_ID"


/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

struct LCR4500_LUT_Entry{
    int     trigger_type;
    int     pattern_number;
    int     bit_depth;
    int     LED_select;
    bool    invert_pattern;
    bool    insert_black;
    bool    buffer_swap;
    bool    trigger_out_share_prev;
    unsigned int exposure;
    unsigned int period;
};

/** @class  LCr4500
 *  @ingroup group_DLP_Platforms
 *  @brief  Contains classes for LightCrafter 4500 projector settings and operations.
 *
 *  The LCr4500 class contains several classes used for setting projector parameters,
 *  and controlling projector operation.
 *
 *  The LCr4500 class is specific to the DLP LightCrafter 4500 and should not be used
 *  with other DLP platforms.
 *
 */
class LCr4500 : public DLP_Platform{
public:

    // Add LUT options

    static const unsigned int PATTERN_LUT_SIZE;
    static const unsigned int IMAGE_LUT_SIZE;
    static const unsigned int BUFFER_IMAGE_SIZE;

    class PowerStandbyMode{
    public:
        enum Enum{
            STANDBY = 1,    /**< Power standby mode */
            NORMAL  = 0     /**< Normal power mode */
        };
    };

    class ImageFlip{
    public:
        enum Enum{
            FLIP   = 1,     /**< Flip image enabled  */
            NORMAL = 0      /**< Flip image disabled */
        };
    };

    class Video{
    public:

        class InputSource{
        public:
            enum Enum{
                PARALLEL_INTERFACE      = 0,    /**< Parallel port input source set */
                INTERNAL_TEST_PATTERNS  = 1,    /**< Internal test pattern source set */
                FLASH_IMAGES            = 2,    /**< Flash images source set */
                FPD_LINK                = 3,    /**< FPD input source set */
                INVALID
            };
        };

        class ParallelPortWidth{
        public:
            enum Enum{
                BITS_30 = 0,    /**< Parallel port width of 30 bits */
                BITS_24 = 1,    /**< Parallel port width of 24 bits */
                BITS_20 = 2,    /**< Parallel port width of 20 bits */
                BITS_16 = 3,    /**< Parallel port width of 16 bits */
                BITS_10 = 4,    /**< Parallel port width of 10 bits */
                BITS_8  = 5,    /**< Parallel port width of 8 bits */
                INVALID
            };
        };

        class ParallelClockPort{
        public:
            enum Enum{
                PORT_1_CLOCK_A = 0, /**< Set clock A as parallel clock source */
                PORT_1_CLOCK_B = 1, /**< Set clock B as parallel clock source */
                PORT_1_CLOCK_C = 2, /**< Set clock C as parallel clock source */
                INVALID
            };
        };

        class DataSwap{
        public:
            // The DLPC350 interprets Channel A as Green, Channel B as Red, and Channel C as Blue.
            class SubChannels{
            public:
                enum Enum{
                    ABC_TO_ABC = 0, /**< No swapping of data subchannels */
                    ABC_TO_CAB = 1, /**< Data subchannels are right shifted and circularly rotated  */
                    ABC_TO_BCA = 2, /**< Data subchannels are left shifted and circularly rotated   */
                    ABC_TO_ACB = 3, /**< Data subchannels B and C are swapped x4 wr                 */
                    ABC_TO_BAC = 4, /**< Data subchannels A and B are swapped                       */
                    ABC_TO_CBA = 5, /**< Data subchannels A and C are swapped                       */
                    INVALID
                };
            };
            class Port{
            public:
                enum Enum{
                    PARALLEL_INTERFACE = 0, /**< Sets the input port to be swapped to parallel port */
                    FPD_LINK           = 1, /**< Sets the input port to be swapped to FPD link      */
                    INVALID
                };
            };
        };

        class TestPattern{
        public:
            static const unsigned int COLOR_MAXIMUM;
            enum Enum{
                SOLID_FIELD       = 0,  /**< Sets the test pattern to a single color for every pixel in the field   */
                HORIZONTAL_RAMP   = 1,  /**< Sets the test pattern to a horizontal ramp of intensities              */
                VERTICAL_RAMP     = 2,  /**< Sets the test pattern to a vertical ramp of intensities                */
                HORIZONTAL_LINES  = 3,  /**< Sets the test pattern to display a series of horizontal lines          */
                DIAGONAL_LINES    = 4,  /**< Sets the test pattern to display a series of diagonal lines            */
                VERTICAL_LINES    = 5,  /**< Sets the test pattern to display a series of vertical lines            */
                GRID              = 6,  /**< Sets the test pattern to display a grid                                */
                CHECKERBOARD      = 7,  /**< Sets the test pattern to display a checkerboard definable by the user  */
                RGB_RAMP          = 8,  /**< Sets the test pattern to display an RGB ramp                           */
                COLOR_BARS        = 9,  /**< Sets the test pattern to display colored bars                          */
                STEP_BARS         = 10, /**< Sets the test pattern to display stepped bars                          */
                INVALID
            };
        };
    };

    class OperatingMode{
    public:
        enum Enum{
            PATTERN_SEQUENCE = 1,    /**< Sets the LightCrafter 4500 to pattern sequence mode */
            VIDEO            = 0    /**< Sets the LightCrafter 4500 to video mode            */
        };
    };

    class InvertData{
    public:
        enum Enum{
            INVERT = 1,    /**< Inverted operation. A data value of 0 will flip the mirror to output light */
            NORMAL = 0    /**< Normal operation. A data value of 1 will flip the mirror to output light   */
        };
    };

    class Led{
    public:
        static const unsigned char MAXIMUM_CURRENT;

    };

    class ImageIndex{
    public:
        static const unsigned int  MAXIMUM_INDEX;
    };


    /** @class Pattern
     *  @brief Class containing objects for the settings of individual patterns.
     */
    class Pattern{
    public:

        /** @class Source
         *  @brief Object for setting the input source of the pattern in the sequence.
         *
         *  DLP SDK does not currently support using a source other than flash images.
         *  A setting of true uses video port input, false sets flash image input.
         */
        class Source{
        public:
            enum Enum{
                FLASH_IMAGES = 0,   /**< Setting for flash image input */
                VIDEO_PORT   = 1     /**< Setting for video port input */
            };
        };

        class TriggerMode{
        public:
            enum Enum{
                MODE_0_VSYNC            = 0,  /**< Mode 0 triggering, uses Vsync signal         */
                MODE_1_INT_OR_EXT       = 1,  /**< Mode 1 internal or external trigger          */
                MODE_2                  = 2,  /**< Mode 2 triggering                            */
                MODE_3_EXP_INT_OR_EXT   = 3,  /**< Mode 3 expanded internal or external trigger */
                MODE_4_EXP_VSYNC        = 4,  /**< Mode 4 expanded triggering with VSYNC        */
                INVALID                 = 5
            };
        };

        class TriggerSource{
        public:
            enum Enum{
                INTERNAL          = 0,  /**< Internally triggered pattern */
                EXTERNAL_POSITIVE = 1,  /**< Externally triggered pattern by a low to high transition */
                EXTERNAL_NEGATIVE = 2,  /**< Externally triggered pattern by a high to low transition */
                NONE              = 3,  /**< No trigger */
                INVALID
            };
        };

        class Led{
        public:
            enum Enum{
                NONE     = 0,   /**< No LED on                                  */
                RED      = 1,   /**< Red LED on                                 */
                GREEN    = 2,   /**< Green LED on                               */
                YELLOW   = 3,   /**< Red and Green LEDs on simultaneously       */
                BLUE     = 4,   /**< Blue LED on simultaneously                 */
                MAGENTA  = 5,   /**< Red and Blue LEDs on simultaneously        */
                CYAN     = 6,   /**< Green and Blue LEDs on simultaneously      */
                WHITE    = 7,   /**< Red, Green and Blue LEDs on simultaneously */
                INVALID  = 8
            };
        };

        class Bitdepth{
        public:
            enum Enum{
                MONO_1BPP = 1,  /**< 1 bit per pixel, 1 color */
                MONO_2BPP = 2,  /**< 2 bit per pixel, 1 color */
                MONO_3BPP = 3,  /**< 3 bit per pixel, 1 color */
                MONO_4BPP = 4,  /**< 4 bit per pixel, 1 color */
                MONO_5BPP = 5,  /**< 5 bit per pixel, 1 color */
                MONO_6BPP = 6,  /**< 6 bit per pixel, 1 color */
                MONO_7BPP = 7,  /**< 7 bit per pixel, 1 color */
                MONO_8BPP = 8,  /**< 8 bit per pixel, 1 color */
                INVALID   = 9
            };
        };

        class Number{
        public:
            /** @class Mono_1BPP
             *  @brief This object sets the pattern to a bit depth of 1.
             */
            class Mono_1BPP{
            public:
                enum Enum{
                    BITPLANE_G0 = 0,     /**< Bitplane G0 */
                    BITPLANE_G1 = 1,     /**< Bitplane G1 */
                    BITPLANE_G2 = 2,     /**< Bitplane G2 */
                    BITPLANE_G3 = 3,     /**< Bitplane G3 */
                    BITPLANE_G4 = 4,     /**< Bitplane G4 */
                    BITPLANE_G5 = 5,     /**< Bitplane G5 */
                    BITPLANE_G6 = 6,     /**< Bitplane G6 */
                    BITPLANE_G7 = 7,     /**< Bitplane G7 */
                    BITPLANE_R0 = 8,     /**< Bitplane R0 */
                    BITPLANE_R1 = 9,     /**< Bitplane R1 */
                    BITPLANE_R2 = 10,    /**< Bitplane R2 */
                    BITPLANE_R3 = 11,    /**< Bitplane R3 */
                    BITPLANE_R4 = 12,    /**< Bitplane R4 */
                    BITPLANE_R5 = 13,    /**< Bitplane R5 */
                    BITPLANE_R6 = 14,    /**< Bitplane R6 */
                    BITPLANE_R7 = 15,    /**< Bitplane R7 */
                    BITPLANE_B0 = 16,    /**< Bitplane B0 */
                    BITPLANE_B1 = 17,    /**< Bitplane B1 */
                    BITPLANE_B2 = 18,    /**< Bitplane B2 */
                    BITPLANE_B3 = 19,    /**< Bitplane B3 */
                    BITPLANE_B4 = 20,    /**< Bitplane B4 */
                    BITPLANE_B5 = 21,    /**< Bitplane B5 */
                    BITPLANE_B6 = 22,    /**< Bitplane B6 */
                    BITPLANE_B7 = 23,    /**< Bitplane B7 */
                    BLACK = 24  /**< All bitplanes */
                };
            };

            /** @class Mono_2BPP
             *  @brief This object sets the pattern to a bit depth of 2.
             */
            class Mono_2BPP{
            public:
                enum Enum{
                    BITPLANES_G1_G0 = 0,  /**< Bitplane G1 to G0 */
                    BITPLANES_G3_G2 = 1,  /**< Bitplane G3 to G2 */
                    BITPLANES_G5_G4 = 2,  /**< Bitplane G5 to G4 */
                    BITPLANES_G7_G6 = 3,  /**< Bitplane G7 to G6 */
                    BITPLANES_R1_R0 = 4,  /**< Bitplane R1 to R0 */
                    BITPLANES_R3_R2 = 5,  /**< Bitplane R3 to R2 */
                    BITPLANES_R5_R4 = 6,  /**< Bitplane R5 to R4 */
                    BITPLANES_R7_R6 = 7,  /**< Bitplane R7 to R6 */
                    BITPLANES_B1_B0 = 8,  /**< Bitplane B1 to B0 */
                    BITPLANES_B3_B2 = 9,  /**< Bitplane B3 to B2 */
                    BITPLANES_B5_B4 = 10, /**< Bitplane B5 to B4 */
                    BITPLANES_B7_B6 = 11  /**< Bitplane B7 to B6 */
                };
            };

            /** @class Mono_3BPP
             *  @brief This object sets the pattern to a bit depth of 3.
             */
            class Mono_3BPP{
            public:
                enum Enum{
                    BITPLANES_G2_G1_G0 = 0,   /**< Bitplane G2 to G0 */
                    BITPLANES_G5_G4_G3 = 1,   /**< Bitplane G5 to G3 */
                    BITPLANES_R0_G7_G6 = 2,   /**< Bitplane R0 to G6 */
                    BITPLANES_R3_R2_R1 = 3,   /**< Bitplane R3 to R1 */
                    BITPLANES_R6_R5_R4 = 4,   /**< Bitplane R6 to R4 */
                    BITPLANES_B1_B0_R7 = 5,   /**< Bitplane B1 to R7 */
                    BITPLANES_B4_B3_B2 = 6,   /**< Bitplane B4 to B2 */
                    BITPLANES_B7_B6_B5 = 7    /**< Bitplane B7 to B5 */
                };
            };

            /** @class Mono_4BPP
             *  @brief This object sets the pattern to a bit depth of 4.
             */
            class Mono_4BPP{
            public:
                enum Enum{
                    BITPLANES_G3_G2_G1_G0 = 0,    /**< Bitplane G3 to G0 */
                    BITPLANES_G7_G6_G5_G4 = 1,    /**< Bitplane G7 to G4 */
                    BITPLANES_R3_R2_R1_R0 = 2,    /**< Bitplane R3 to R0 */
                    BITPLANES_R7_R6_R5_R4 = 3,    /**< Bitplane R7 to G4 */
                    BITPLANES_B3_B2_B1_B0 = 4,    /**< Bitplane B3 to B0 */
                    BITPLANES_B7_B6_B5_B4 = 5   /**< Bitplane B7 to B4 */
                };
            };

            /** @class Mono_5BPP
             *  @brief This object sets the pattern to a bit depth of 5.
             */
            class Mono_5BPP{
            public:
                enum Enum{
                    BITPLANES_G5_G4_G3_G2_G1 = 0, /**< Bitplane G5 to G1 */
                    BITPLANES_R3_R2_R1_R0_G7 = 1, /**< Bitplane R3 to G7 */
                    BITPLANES_B1_B0_R7_R6_R5 = 2, /**< Bitplane B1 to R5 */
                    BITPLANES_B7_B6_B5_B4_B3 = 3  /**< Bitplane B7 to B3 */
                };
            };

            /** @class Mono_6BPP
             *  @brief This object sets the pattern to a bit depth of 6.
             */
            class Mono_6BPP{
            public:
                enum Enum{
                    BITPLANES_G5_G4_G3_G2_G1_G0 = 0,  /**< Bitplane G5 to G0 */
                    BITPLANES_R3_R2_R1_R0_G7_G6 = 1,  /**< Bitplane R3 to G6 */
                    BITPLANES_B1_B0_R7_R6_R5_R4 = 2,  /**< Bitplane B1 to R4 */
                    BITPLANES_B7_B6_B5_B4_B3_B2 = 3   /**< Bitplane B7 to B2 */
                };
            };

            /** @class Mono_7BPP
             *  @brief This object sets the pattern to a bit depth of 7.
             */
            class Mono_7BPP{
            public:
                enum Enum{
                    BITPLANES_G7_G6_G5_G4_G3_G2_G1 = 0,   /**< Bitplane G7 to G1 */
                    BITPLANES_R7_R6_R5_R4_R3_R2_R1 = 1,   /**< Bitplane R7 to R1 */
                    BITPLANES_B7_B6_B5_B4_B3_B2_B1 = 2    /**< Bitplane B7 to B1 */
                };
            };

            /** @class Mono_8BPP
             *  @brief This object sets the pattern to a bit depth of 8.
             */
            class Mono_8BPP{
            public:
                enum Enum{
                    BITPLANES_G7_G6_G5_G4_G3_G2_G1_G0 = 0,    /**< Bitplane G7 to G0 */
                    BITPLANES_R7_R6_R5_R4_R3_R2_R1_R0 = 1,    /**< Bitplane R7 to R0 */
                    BITPLANES_B7_B6_B5_B4_B3_B2_B1_B0 = 2   /**< Bitplane B7 to B0 */
                };
            };
        };



        /** @class DisplayControl
         *  @brief Class containing settings that control the display of
         *         pattern sequences by the LightCrafter 4500.
         */
        class DisplayControl{
        public:
            enum Enum{
                STOP  = 0,  /**< Stops the display of pattern sequence */
                PAUSE = 1,  /**< Pauses the display of pattern sequence */
                START = 2,  /**< Starts the display of pattern sequence */
                INVALID
            };
        };

        /** @class Exposure
         *  @brief Sets the exposure time of the pattern.  Exposure time is set in ns.
         */
        class Exposure{
        public:
            static unsigned long int MININUM(const dlp::Pattern::Bitdepth &bitdepth);

            static const unsigned long int MAXIMUM;
            static const unsigned long int PERIOD_DIFFERENCE_MINIMUM;
        };

    };

    enum class ImageCompression{
        NONE,
        RLE,
        FOUR_LINE,
        UNSPECIFIED
    };


    class Parameters{
    public:
        DLP_NEW_PARAMETERS_ENTRY(DLPC350_Firmware,          "LCR4500_PARAMETERS_DLPC350_FIRMWARE",          std::string,    "");
        DLP_NEW_PARAMETERS_ENTRY(DLPC350_FlashParameters,   "LCR4500_PARAMETERS_DLPC350_FLASH_PARAMETERS",  std::string,    "resources/lcr4500/DLPC350_FlashDeviceParameters.txt");
        DLP_NEW_PARAMETERS_ENTRY(DLPC350_PreparedFirmware,  "LCR4500_PARAMETERS_DLPC350_FIRMWARE_PREPARED", std::string,    "dlp_sdk_lcr4500_dlpc350_prepared.bin");

        DLP_NEW_PARAMETERS_ENTRY(DLPC350_ImageCompression,  "LCR4500_PARAMETERS_DLPC350_IMAGE_COMPRESSION", ImageCompression, ImageCompression::UNSPECIFIED);

        DLP_NEW_PARAMETERS_ENTRY(FlagUseDefault,            "LCR4500_PARAMETERS_USE_DEFAULT", bool, false);

        DLP_NEW_PARAMETERS_ENTRY(OperatingMode,             "LCR4500_PARAMETERS_OPERATING_MODE",        dlp::LCr4500::OperatingMode::Enum,      dlp::LCr4500::OperatingMode::VIDEO);
        DLP_NEW_PARAMETERS_ENTRY(InvertData,                "LCR4500_PARAMETERS_INVERT_DATA",           dlp::LCr4500::InvertData::Enum,         dlp::LCr4500::InvertData::NORMAL);
        DLP_NEW_PARAMETERS_ENTRY(PowerStandyMode,           "LCR4500_PARAMETERS_POWER_MODE_STANDBY",    dlp::LCr4500::PowerStandbyMode::Enum,   dlp::LCr4500::PowerStandbyMode::NORMAL);

        DLP_NEW_PARAMETERS_ENTRY(ImageFlipShortAxis,        "LCR4500_PARAMETERS_IMAGE_FLIP_SHORT", dlp::LCr4500::ImageFlip::Enum, dlp::LCr4500::ImageFlip::NORMAL);
        DLP_NEW_PARAMETERS_ENTRY(ImageFlipLongAxis,         "LCR4500_PARAMETERS_IMAGE_FLIP_LONG",  dlp::LCr4500::ImageFlip::Enum, dlp::LCr4500::ImageFlip::NORMAL);

        DLP_NEW_PARAMETERS_ENTRY(VideoInputSource,          "LCR4500_PARAMETERS_VIDEO_INPUT_SOURCE",        dlp::LCr4500::Video::InputSource::Enum,       dlp::LCr4500::Video::InputSource::PARALLEL_INTERFACE);
        DLP_NEW_PARAMETERS_ENTRY(VideoParallelPortWidth,    "LCR4500_PARAMETERS_VIDEO_PARALLEL_PORT_WIDTH", dlp::LCr4500::Video::ParallelPortWidth::Enum,  dlp::LCr4500::Video::ParallelPortWidth::BITS_24);
        DLP_NEW_PARAMETERS_ENTRY(VideoParallelClockPort,    "LCR4500_PARAMETERS_VIDEO_PARALLEL_CLOCK_PORT", dlp::LCr4500::Video::ParallelClockPort::Enum,   dlp::LCr4500::Video::ParallelClockPort::PORT_1_CLOCK_A);
        DLP_NEW_PARAMETERS_ENTRY(VideoSwapDataParallel,     "LCR4500_PARAMETERS_VIDEO_SWAP_DATA_PARALLEL",  dlp::LCr4500::Video::DataSwap::SubChannels::Enum,     dlp::LCr4500::Video::DataSwap::SubChannels::ABC_TO_BAC);
        DLP_NEW_PARAMETERS_ENTRY(VideoSwapDataFpd,          "LCR4500_PARAMETERS_VIDEO_SWAP_DATA_FPD",       dlp::LCr4500::Video::DataSwap::SubChannels::Enum,     dlp::LCr4500::Video::DataSwap::SubChannels::ABC_TO_ABC);

        DLP_NEW_PARAMETERS_ENTRY(VideoTestPattern,                  "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN",                    dlp::LCr4500::Video::TestPattern::Enum,      dlp::LCr4500::Video::TestPattern::CHECKERBOARD);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternForegroundRed,     "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_FOREGROUND_RED",     unsigned int, 1023);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternForegroundGreen,   "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_FOREGROUND_GREEN",   unsigned int, 1023);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternForegroundBlue,    "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_FOREGROUND_BLUE",    unsigned int, 1023);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternBackgroundRed,     "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_BACKGROUND_RED",     unsigned int,    0);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternBackgroundGreen,   "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_BACKGROUND_GREEN",   unsigned int,    0);
        DLP_NEW_PARAMETERS_ENTRY(VideoTestPatternBackgroundBlue,    "LCR4500_PARAMETERS_VIDEO_TEST_PATTERN_BACKGROUND_BLUE",    unsigned int,    0);

        DLP_NEW_PARAMETERS_ENTRY(VideoFlashImage,       "LCR4500_PARAMETERS_VIDEO_FLASH_IMAGE", unsigned char, 0);


        DLP_NEW_PARAMETERS_ENTRY(LED_SequenceAutomatic, "LCR4500_PARAMETERS_LED_SEQUENCE_AUTOMATIC", bool,  true);
        DLP_NEW_PARAMETERS_ENTRY(LED_EnableRed,         "LCR4500_PARAMETERS_LED_ENABLE_RED",         bool, false);
        DLP_NEW_PARAMETERS_ENTRY(LED_EnableGreen,       "LCR4500_PARAMETERS_LED_ENABLE_GREEN",       bool, false);
        DLP_NEW_PARAMETERS_ENTRY(LED_EnableBlue,        "LCR4500_PARAMETERS_LED_ENABLE_BLUE",        bool, false);

        DLP_NEW_PARAMETERS_ENTRY(LED_InvertPWM,         "LCR4500_PARAMETERS_LED_INVERT_PWM",                   bool, false);
        DLP_NEW_PARAMETERS_ENTRY(LED_CurrentRed,        "LCR4500_PARAMETERS_LED_CURRENT_RED",         unsigned char,   105);
        DLP_NEW_PARAMETERS_ENTRY(LED_CurrentGreen,      "LCR4500_PARAMETERS_LED_CURRENT_GREEN",       unsigned char,   134);
        DLP_NEW_PARAMETERS_ENTRY(LED_CurrentBlue,       "LCR4500_PARAMETERS_LED_CURRENT_BLUE",        unsigned char,   135);

        DLP_NEW_PARAMETERS_ENTRY(LED_DelayRisingRed,    "LCR4500_PARAMETERS_LED_DELAY_RISING_RED",    unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(LED_DelayRisingGreen,  "LCR4500_PARAMETERS_LED_DELAY_RISING_GREEN",  unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(LED_DelayRisingBlue,   "LCR4500_PARAMETERS_LED_DELAY_RISING_BLUE",   unsigned char, 187);

        DLP_NEW_PARAMETERS_ENTRY(LED_DelayFallingRed,   "LCR4500_PARAMETERS_LED_DELAY_FALLING_RED",   unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(LED_DelayFallingGreen, "LCR4500_PARAMETERS_LED_DELAY_FALLING_GREEN", unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(LED_DelayFallingBlue,  "LCR4500_PARAMETERS_LED_DELAY_FALLING_BLUE",  unsigned char, 187);

        DLP_NEW_PARAMETERS_ENTRY(TriggerSource,             "LCR4500_PARAMETERS_TRIGGER_SOURCE", dlp::LCr4500::Pattern::TriggerSource::Enum, dlp::LCr4500::Pattern::TriggerSource::INTERNAL);

        DLP_NEW_PARAMETERS_ENTRY(TriggerIn1Delay,           "LCR4500_PARAMETERS_TRIGGER_IN_1_DELAY",    unsigned int,     0);

        DLP_NEW_PARAMETERS_ENTRY(TriggerOut1Invert,         "LCR4500_PARAMETERS_TRIGGER_OUT_1_INVERT",          bool, false);
        DLP_NEW_PARAMETERS_ENTRY(TriggerOut2Invert,         "LCR4500_PARAMETERS_TRIGGER_OUT_2_INVERT",          bool, false);

        DLP_NEW_PARAMETERS_ENTRY(TriggerOut1DelayRising,    "LCR4500_PARAMETERS_TRIGGER_OUT_1_DELAY_RISING",  unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(TriggerOut1DelayFalling,   "LCR4500_PARAMETERS_TRIGGER_OUT_1_DELAY_FALLING", unsigned char, 187);
        DLP_NEW_PARAMETERS_ENTRY(TriggerOut2DelayRising,    "LCR4500_PARAMETERS_TRIGGER_OUT_2_DELAY_RISING",  unsigned char, 187);

        DLP_NEW_PARAMETERS_ENTRY(PatternNumber,             "LCR4500_PARAMETERS_PATTERN_NUMBER",            unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternNumberRed,          "LCR4500_PARAMETERS_PATTERN_NUMBER_RED",        unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternNumberGreen,        "LCR4500_PARAMETERS_PATTERN_NUMBER_GREEN",      unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternNumberBlue,         "LCR4500_PARAMETERS_PATTERN_NUMBER_BLEU",       unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternImageIndex,         "LCR4500_PARAMETERS_PATTERN_IMAGE_INDEX",       unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternImageIndexRed,      "LCR4500_PARAMETERS_PATTERN_IMAGE_INDEX_RED",   unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternImageIndexGreen,    "LCR4500_PARAMETERS_PATTERN_IMAGE_INDEX_GREEN", unsigned int, 0);
        DLP_NEW_PARAMETERS_ENTRY(PatternImageIndexBlue,     "LCR4500_PARAMETERS_PATTERN_IMAGE_INDEX_BLEU",  unsigned int, 0);
        //DLP_NEW_PARAMETERS_ENTRY(PatternTrigger,            "LCR4500_PARAMETERS_PATTERN_TRIGGER",  dlp::LCr4500::Pattern::TriggerSource::Enum, dlp::LCr4500::Pattern::TriggerSource::INTERNAL);
        DLP_NEW_PARAMETERS_ENTRY(PatternInvert,             "LCR4500_PARAMETERS_PATTERN_INVERT",            bool,   false);
        DLP_NEW_PARAMETERS_ENTRY(PatternShareExposure,      "LCR4500_PARAMETERS_PATTERN_SHARE_EXPOSURE",    bool,   false);
        DLP_NEW_PARAMETERS_ENTRY(PatternSequenceRepeat,     "LCR4500_PARAMETERS_PATTERN_SEQUENCE_REPEAT",   bool,   false);

        DLP_NEW_PARAMETERS_ENTRY(VerifyImageLoadTimeCount,  "LCR4500_PARAMETERS_VERIFY_IMAGE_LOAD_COUNT", unsigned int, 1);
    };

    LCr4500();

    // DLP_Platform Pure Virtual Functions
    ReturnCode Connect(std::string id);
    ReturnCode Disconnect();
    bool       isConnected() const;

    ReturnCode Setup(const dlp::Parameters &settings);
    ReturnCode GetSetup(dlp::Parameters* settings)const;

    ReturnCode ProjectSolidWhitePattern();
    ReturnCode ProjectSolidBlackPattern();

    ReturnCode PreparePatternSequence(const dlp::Pattern::Sequence &pattern_sequence);
    ReturnCode StartPatternSequence(const unsigned int &start, const unsigned int &patterns, const bool &repeat);
    ReturnCode DisplayPatternInSequence(const unsigned int &pattern_index, const bool &repeat);
    ReturnCode StopPatternSequence();


    // LightCrafter 4500 Specific methods
    ReturnCode PatternSettingsValid(dlp::Pattern &arg_pattern);

    ReturnCode UploadFirmware(std::string firmware_filename);
    ReturnCode CreateFirmware(const std::string              &new_firmware_filename,
                             const std::vector<std::string> &image_filenames);

    ReturnCode CreateFirmwareImages(const dlp::Pattern::Sequence  &arg_pattern_sequence,
                                   const std::string        &arg_image_filename_base,
                                   dlp::Pattern::Sequence          &ret_pattern_sequence,
                                   std::vector<std::string> &ret_image_filename_list );

    ReturnCode GetImageLoadTime(const unsigned int &index, const unsigned int &load_count, double *max_microseconds);

    bool StartPatternImageStorage(const unsigned char &bitplane_position,
                                  const unsigned char &mono_bpp,
                                        unsigned char &ret_pattern_number);

    bool FirmwareUploadInProgress();

    long long GetFirmwareUploadPercentComplete();
    long long GetFirmwareFlashEraseComplete();

private:

    unsigned int previous_sequence_start_;
    unsigned int previous_sequence_patterns_;
    bool         previous_sequence_repeat_;

    // Pattern Sequence related methods
    static int DlpPatternColorToLCr4500Led(const dlp::Pattern::Color &color);
    static int DlpPatternBitdepthToLCr4500Bitdepth(const dlp::Pattern::Bitdepth &depth);

    // Firmware upload related methods
    bool ProcessFlashParamsLine(const std::string &line);
    int  GetSectorNum(unsigned int Addr);

    ReturnCode SavePatternIntImageAsRGBfile(Image &image_int, const std::string &filename);
    ReturnCode CreateSendStartSequenceLut(const dlp::Pattern::Sequence &arg_pattern_sequence);

    // Setting members
    Parameters::DLPC350_Firmware            dlpc350_firmware_;
    Parameters::DLPC350_FlashParameters     dlpc350_flash_parameters_;
    Parameters::DLPC350_PreparedFirmware    pattern_sequence_firmware_;
    Parameters::DLPC350_ImageCompression    dlpc350_image_compression_;

    Parameters::FlagUseDefault      use_default_;
    Parameters::PowerStandyMode     power_standby_;
    Parameters::OperatingMode       display_mode_;
    Parameters::InvertData          invert_data_;
    Parameters::ImageFlipShortAxis  short_axis_flip_;
    Parameters::ImageFlipLongAxis   long_axis_flip_;

    Parameters::VideoInputSource        input_source_;
    Parameters::VideoParallelPortWidth  parallel_port_width_;
    Parameters::VideoParallelClockPort  parallel_port_clock_;
    Parameters::VideoSwapDataParallel   parallel_data_swap_;

    Parameters::VideoTestPatternForegroundRed   test_pattern_foreground_red;
    Parameters::VideoTestPatternForegroundGreen test_pattern_foreground_green;
    Parameters::VideoTestPatternForegroundBlue  test_pattern_foreground_blue;
    Parameters::VideoTestPatternBackgroundRed   test_pattern_background_red;
    Parameters::VideoTestPatternBackgroundGreen test_pattern_background_green;
    Parameters::VideoTestPatternBackgroundBlue  test_pattern_background_blue;


    Parameters::LED_SequenceAutomatic   led_sequence_mode_;
    Parameters::LED_EnableRed           led_red_enable_;
    Parameters::LED_EnableGreen         led_green_enable_;
    Parameters::LED_EnableBlue          led_blue_enable_;

    Parameters::LED_InvertPWM           led_invert_pwm_;
    Parameters::LED_CurrentRed          led_red_current_;
    Parameters::LED_CurrentGreen        led_green_current_;
    Parameters::LED_CurrentBlue         led_blue_current_;

    Parameters::LED_DelayRisingRed      led_red_edge_delay_rising_;
    Parameters::LED_DelayRisingGreen    led_green_edge_delay_rising_;
    Parameters::LED_DelayRisingBlue     led_blue_edge_delay_rising_;

    Parameters::LED_DelayFallingRed     led_red_edge_delay_falling_;
    Parameters::LED_DelayFallingGreen   led_green_edge_delay_falling_;
    Parameters::LED_DelayFallingBlue    led_blue_edge_delay_falling_;

    Parameters::TriggerSource           trigger_source_;
    Parameters::TriggerIn1Delay         trigger_in_1_delay_;
    Parameters::TriggerOut1Invert       trigger_out_1_invert_;
    Parameters::TriggerOut2Invert       trigger_out_2_invert_;
    Parameters::TriggerOut1DelayRising  trigger_out_1_rising_;
    Parameters::TriggerOut1DelayFalling trigger_out_1_falling_;
    Parameters::TriggerOut2DelayRising  trigger_out_2_rising_;

    Parameters::VerifyImageLoadTimeCount    verify_image_load_;

    FlashDevice myFlashDevice;
    std::string firmwarePath;

    bool        previous_command_in_progress;

    bool        firmware_upload_restart_needed;


    std::atomic_flag        firmware_upload_in_progress;// = ATOMIC_FLAG_INIT;
    std::atomic <long long> firmware_upload_percent_erased_;
    std::atomic <long long> firmware_upload_percent_complete_;

    unsigned char status_hw_;
    unsigned char status_sys_;
    unsigned char status_main_;

    bool pattern_sequence_prepared_;
    dlp::Pattern::Sequence   pattern_sequence_;
};

namespace String{
template <> dlp::LCr4500::ImageCompression ToNumber(const std::string &text, unsigned int base );
template <> dlp::LCr4500::PowerStandbyMode::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::ImageFlip::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::InputSource::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::ParallelPortWidth::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::ParallelClockPort::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::DataSwap::SubChannels::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::DataSwap::Port::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Video::TestPattern::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::OperatingMode::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::InvertData::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Source::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::TriggerMode::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::TriggerSource::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Led::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Bitdepth::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum ToNumber( const std::string &text, unsigned int base );
template <> dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum ToNumber( const std::string &text, unsigned int base );
}

namespace Number{
template <> std::string ToString<dlp::LCr4500::ImageCompression>(dlp::LCr4500::ImageCompression value );
template <> std::string ToString<dlp::LCr4500::PowerStandbyMode::Enum>( dlp::LCr4500::PowerStandbyMode::Enum value );
template <> std::string ToString<dlp::LCr4500::ImageFlip::Enum>( dlp::LCr4500::ImageFlip::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::InputSource::Enum>( dlp::LCr4500::Video::InputSource::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::ParallelPortWidth::Enum>( dlp::LCr4500::Video::ParallelPortWidth::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::ParallelClockPort::Enum>( dlp::LCr4500::Video::ParallelClockPort::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::DataSwap::SubChannels::Enum>( dlp::LCr4500::Video::DataSwap::SubChannels::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::DataSwap::Port::Enum>( dlp::LCr4500::Video::DataSwap::Port::Enum value );
template <> std::string ToString<dlp::LCr4500::Video::TestPattern::Enum>( dlp::LCr4500::Video::TestPattern::Enum value );
template <> std::string ToString<dlp::LCr4500::OperatingMode::Enum>( dlp::LCr4500::OperatingMode::Enum value );
template <> std::string ToString<dlp::LCr4500::InvertData::Enum>( dlp::LCr4500::InvertData::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Source::Enum>( dlp::LCr4500::Pattern::Source::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::TriggerMode::Enum>( dlp::LCr4500::Pattern::TriggerMode::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::TriggerSource::Enum>( dlp::LCr4500::Pattern::TriggerSource::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Led::Enum>( dlp::LCr4500::Pattern::Led::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Bitdepth::Enum>( dlp::LCr4500::Pattern::Bitdepth::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum value );
template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum value );
}


}

#endif // DLP_SDK_LCR4500_HPP

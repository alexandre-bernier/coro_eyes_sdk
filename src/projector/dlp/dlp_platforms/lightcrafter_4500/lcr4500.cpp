/** @file   lcr4500.cpp
 *  @brief  Contains methods to interface with DLP LightCrafter 4500 EVM
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/other.hpp>
#include <projector/dlp/common/image/image.hpp>
#include <projector/dlp/common/parameters.hpp>
#include <projector/dlp/common/pattern/pattern.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <atomic>

#include <ctime>

#include <projector/dlp/dlp_platforms/lightcrafter_4500/common.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/error.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/dlpc350_usb.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/dlpc350_firmware.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/flashdevice.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/dlpc350_api.hpp>
#include <projector/dlp/dlp_platforms/dlp_platform.hpp>
#include <projector/dlp/dlp_platforms/lightcrafter_4500/lcr4500.hpp>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

const unsigned char LCr4500::Led::MAXIMUM_CURRENT               = 255;
const unsigned int  LCr4500::ImageIndex::MAXIMUM_INDEX          = MAX_SPLASH_IMAGES;
const unsigned int  LCr4500::Video::TestPattern::COLOR_MAXIMUM  = 1023;

const unsigned long int LCr4500::Pattern::Exposure::MAXIMUM = 200000000;
const unsigned long int LCr4500::Pattern::Exposure::PERIOD_DIFFERENCE_MINIMUM = 230;

const unsigned int LCr4500::PATTERN_LUT_SIZE  = MAX_VAR_EXP_PAT_LUT_ENTRIES;
const unsigned int LCr4500::IMAGE_LUT_SIZE    = MAX_VAR_EXP_IMAGE_LUT_ENTRIES;
const unsigned int LCr4500::BUFFER_IMAGE_SIZE =   2;

unsigned long int LCr4500::Pattern::Exposure::MININUM(const dlp::Pattern::Bitdepth &bitdepth){
    switch(bitdepth){
    case dlp::Pattern::Bitdepth::MONO_1BPP:   return     235;
    case dlp::Pattern::Bitdepth::MONO_2BPP:   return     700;
    case dlp::Pattern::Bitdepth::MONO_3BPP:   return    1570;
    case dlp::Pattern::Bitdepth::MONO_4BPP:   return    1700;
    case dlp::Pattern::Bitdepth::MONO_5BPP:   return    2000;
    case dlp::Pattern::Bitdepth::MONO_6BPP:   return    2500;
    case dlp::Pattern::Bitdepth::MONO_7BPP:   return    4500;
    case dlp::Pattern::Bitdepth::MONO_8BPP:   return    8333;
    case dlp::Pattern::Bitdepth::RGB_3BPP:    return     705;
    case dlp::Pattern::Bitdepth::RGB_6BPP:    return    2100;
    case dlp::Pattern::Bitdepth::RGB_9BPP:    return    4710;
    case dlp::Pattern::Bitdepth::RGB_12BPP:   return    5100;
    case dlp::Pattern::Bitdepth::RGB_15BPP:   return    6000;
    case dlp::Pattern::Bitdepth::RGB_18BPP:   return    7500;
    case dlp::Pattern::Bitdepth::RGB_21BPP:   return   13500;
    case dlp::Pattern::Bitdepth::RGB_24BPP:   return   24999;
    case dlp::Pattern::Bitdepth::INVALID:
    default:
        return MAXIMUM;
    }
}


/** @brief  Constructs empty object */
LCr4500::LCr4500()
{
    // Set the debug name
    this->debug_.SetName("LCR4500_DEBUG(" + dlp::Number::ToString(this)+ "): ");

    // Set the platform
    this->SetPlatform(Platform::LIGHTCRAFTER_4500);

    // Set default values
    this->previous_command_in_progress      = false;
    this->firmware_upload_restart_needed    = false;
    this->pattern_sequence_prepared_        = false;

    this->firmware_upload_percent_erased_   = 0;
    this->firmware_upload_percent_complete_ = 0;


    this->previous_sequence_start_ = 0;
    this->previous_sequence_patterns_ = 0;
    this->previous_sequence_repeat_ = false;

    this->debug_.Msg(1,"Object constructed");
}

/** @brief  Connects to a DLP LightCrafter 4500 EVM 
 *  @param[in]  device  assigned ID of individual projector
 *
 *  Functionality of multiple projectors
 *  NOT yet implemented but will allow multiple
 *  projectors to be connected to the DLP SDK.
 */
ReturnCode LCr4500::Connect(std::string id){
    ReturnCode ret;

    // If A firmware upload is in progress, do NOT send any commands until upload is complete! and does not
    // need a restard return and error
    if( this->FirmwareUploadInProgress() && !this->firmware_upload_restart_needed){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }


    // First initialize the HID USB interface
    if(DLPC350_USB_IsConnected()){
        DLPC350_USB_Close();
        DLPC350_USB_Exit();
    }

    // Initialize the USB interface
    this->debug_.Msg("Initializing USB HID interface...");
    DLPC350_USB_Init();

    // If the HID Interface has been connected already close it
    if(DLPC350_USB_IsConnected()) DLPC350_USB_Close();

    // Attempt to open the LCr4500 USB interface
    this->debug_.Msg("Opening USB HID interface...");
    DLPC350_USB_Open();

    // Workaround for DLPC350 error
    DLPC350_USB_Close();
    DLPC350_USB_Open();

    // Check if the connection succeeded
    if(!DLPC350_USB_IsConnected()){
        // Connection failed
        this->debug_.Msg("USB connection failed");
        DLPC350_USB_Close();
        DLPC350_USB_Exit();
        ret.AddError(LCR4500_CONNECTION_FAILED);
    }
    else{
        // Connection was successful
        this->debug_.Msg("USB connected successfully");
        this->SetID(id);
    }

    return ret;
}

/** @brief  Disconnects from the DLP LightCrafter 4500 EVM
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED               A LightCrafter 4500 EVM has NOT enumerated on the USB
 */
ReturnCode LCr4500::Disconnect(){
    ReturnCode ret;

    // If A firmware upload is in progress and does not
    // need a restard return and error
    if( this->FirmwareUploadInProgress() && !this->firmware_upload_restart_needed){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // If the HID Interface has been connected already close it
    if(!DLPC350_USB_IsConnected()){
        // The device wasn't connected
        this->debug_.Msg("Could NOT disconnect USB because it was NOT connected");
        ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Close the USB interface
    this->debug_.Msg("Closing USB HID interface");
    DLPC350_USB_Close();
    DLPC350_USB_Exit();

    return ret;
}

/** @brief  Returns true if the projector object is connected via USB. */
bool LCr4500::isConnected() const{
    bool ret = DLPC350_USB_IsConnected();

    if(ret){
        this->debug_.Msg("USB is connected");
    }
    else{
        this->debug_.Msg("USB is NOT connected");
    }

    return ret;
}

/** @brief  Sends a parameters object containing LightCrafter 4500 options
 * to a LightCrafter 4500 device.
 * @param[in]   arg_parameters  Takes an input object of dlp::Parameters type to set LightCrafter 4500 EVM
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS     A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED                   A LightCrafter 4500 EVM has NOT been enumerated on the USB
 * @retval  LCR4500_SETUP_POWER_STANDBY_FAILED      The power mode could NOT be set on the LightCrafter 4500
 * @retval  LCR4500_SETUP_SHORT_AXIS_FLIP_FAILED    The short axis flip could NOT be set
 * @retval  LCR4500_SETUP_LONG_AXIS_FLIP_FAILED     The long axis flip could NOT be set
 * @retval  LCR4500_SETUP_LED_SEQUENCE_AND_ENABLES_FAILED   LED sequence and enables could NOT be set
 * @retval  LCR4500_SETUP_INVERT_LED_PWM_FAILED     The inversion of the LED PWM signal could NOT be set
 * @retval  LCR4500_SETUP_LED_CURRENTS_FAILED       The LED current values could NOT be set
 * @retval  LCR4500_SETUP_LED_RED_EDGE_DELAYS_FAILED        The red LED edge delay could NOT be set
 * @retval  LCR4500_SETUP_LED_GREEN_EDGE_DELAYS_FAILED      The green LED edge delay could NOT be set
 * @retval  LCR4500_SETUP_LED_BLUE_EDGE_DELAYS_FAILED       The blue LED edge delay could NOT be set
 * @retval  LCR4500_SETUP_INPUT_SOURCE_FAILED           The input source could NOT be set
 * @retval  LCR4500_SETUP_PARALLEL_PORT_CLOCK_FAILED    The parallel port clock could NOT be set
 * @retval  LCR4500_SETUP_DATA_SWAP_FAILED          The color data swap could NOT be set
 * @retval  LCR4500_SETUP_INVERT_DATA_FAILED        The inverted color image could NOT be set
 * @retval  LCR4500_SETUP_DISPLAY_MODE_FAILED       The display mode could NOT be set
 * @retval  LCR4500_SETUP_TEST_PATTERN_COLOR_FAILED The test pattern color could NOT be set
 * @retval  LCR4500_SETUP_POWER_STANDBY_FAILED      The power standby could NOT be set
 * @retval  LCR4500_SETUP_DISPLAY_MODE_FAILED       The display mode could NOT be set
 * @retval  LCR4500_SETUP_INPUT_SOURCE_FAILED       The input source could NOT be set
 * @retval  LCR4500_SETUP_TEST_PATTERN_FAILED       The test pattern could NOT be set
 * @retval  LCR4500_SETUP_FLASH_IMAGE_FAILED        The flash image could NOT be set
 * @retval  LCR4500_SETUP_TRIGGER_INPUT_1_DELAY_FAILED  The input delay for trigger 1 could NOT be set
 * @retval  LCR4500_SETUP_TRIGGER_OUTPUT_1_FAILED   The trigger 1 output could NOT be set
 * @retval  LCR4500_SETUP_TRIGGER_OUTPUT_2_FAILED   The trigger 2 output could NOT be set
 */
ReturnCode LCr4500::Setup(const dlp::Parameters &settings){
    ReturnCode ret;

    bool setup_default = false;

    // If A firmware upload is in progress return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that LCr4500 is connected
    if(!this->isConnected()){
        this->debug_.Msg("Device is NOT connected");
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Stop the display
    this->debug_.Msg("Stopping pattern display");
    if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP)<0)
        return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);


    // Should default values be set?
    settings.Get(&this->use_default_);
    if(this->use_default_.Get()){
        this->debug_.Msg("Setting the default values...");
        setup_default = this->use_default_.Get();
    }

    // Load DLPC350 related file locations
    if(settings.Contains(this->dlpc350_firmware_))
        settings.Get(&this->dlpc350_firmware_);

    if(settings.Contains(this->dlpc350_flash_parameters_))
        settings.Get(&this->dlpc350_flash_parameters_);

    settings.Get(&this->dlpc350_image_compression_);

    if(settings.Contains(this->pattern_sequence_firmware_))
        settings.Get(&this->pattern_sequence_firmware_);

    if(settings.Contains(this->verify_image_load_))
        settings.Get(&this->verify_image_load_);


    // Check if parameters contains setup instructions
    if(setup_default || settings.Contains(this->power_standby_)){
        settings.Get(&this->power_standby_);

        // Send command and check if it succeeded
        this->debug_.Msg("Setting power mode = " + this->power_standby_.GetEntryValue());
        if(DLPC350_SetPowerMode(this->power_standby_.Get()) < 0){
            this->debug_.Msg("Setting power mode FAILED");
            return ret.AddError(LCR4500_SETUP_POWER_STANDBY_FAILED);
        }
    }


    if(setup_default || settings.Contains(this->short_axis_flip_)){
        // Get the value
        settings.Get(&this->short_axis_flip_);
        // Send command and check if it succeeded
        this->debug_.Msg("Setting short axis image flip = " + this->short_axis_flip_.GetEntryValue());
        if(DLPC350_SetShortAxisImageFlip(this->short_axis_flip_.Get()) < 0 ){
            this->debug_.Msg("Setting short axis image flip FAILED");
            return ret.AddError(LCR4500_SETUP_SHORT_AXIS_FLIP_FAILED);
        }
    }

    if(setup_default || settings.Contains(this->long_axis_flip_)){
        // Get the value
        settings.Get(&this->long_axis_flip_);

        // Send command and check if it succeeded
        this->debug_.Msg("Setting long axis image flip = " + this->long_axis_flip_.GetEntryValue());
        if(DLPC350_SetLongAxisImageFlip(this->long_axis_flip_.Get()) < 0 ){
            this->debug_.Msg("Setting long axis image flip FAILED");
            return ret.AddError(LCR4500_SETUP_LONG_AXIS_FLIP_FAILED);
        }
    }

    if(setup_default || settings.Contains(this->led_sequence_mode_)
                     || settings.Contains(this->led_red_enable_)
                     || settings.Contains(this->led_green_enable_)
                     || settings.Contains(this->led_blue_enable_)){
        // Get the values
        settings.Get(&this->led_sequence_mode_);
        settings.Get(&this->led_red_enable_);
        settings.Get(&this->led_green_enable_);
        settings.Get(&this->led_blue_enable_);

        // Send command and check if it succeeded
        this->debug_.Msg("Setting LED auto sequence = " + this->led_sequence_mode_.GetEntryValue());
        this->debug_.Msg("Setting LED enable red    = " + this->led_red_enable_.GetEntryValue());
        this->debug_.Msg("Setting LED enable green  = " + this->led_green_enable_.GetEntryValue());
        this->debug_.Msg("Setting LED enable blue   = " + this->led_blue_enable_.GetEntryValue());
        if(DLPC350_SetLedEnables( this->led_sequence_mode_.Get(),
                                  this->led_red_enable_.Get(),
                                  this->led_green_enable_.Get(),
                                  this->led_blue_enable_.Get()) < 0){
            this->debug_.Msg("Setting LED sequence and enables FAILED");
            return ret.AddError(LCR4500_SETUP_LED_SEQUENCE_AND_ENABLES_FAILED);
        }
    }

    if(setup_default || settings.Contains(this->led_invert_pwm_)){
        // Get the value
        settings.Get(&this->led_invert_pwm_);
        // Send command and check if it succeeded
        if(DLPC350_SetLEDPWMInvert(this->led_invert_pwm_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_INVERT_LED_PWM_FAILED);
    }

    if(setup_default || settings.Contains(this->led_red_current_)
                     || settings.Contains(this->led_green_current_)
                     || settings.Contains(this->led_blue_current_)){
        // Get the value
        settings.Get(&this->led_red_current_);
        settings.Get(&this->led_green_current_);
        settings.Get(&this->led_blue_current_);

        // Send command and check if it succeeded
        // Invert the currents for LightCrafter 4500 circuitry
        if(DLPC350_SetLedCurrents( Led::MAXIMUM_CURRENT - this->led_red_current_.Get(),
                                   Led::MAXIMUM_CURRENT - this->led_green_current_.Get(),
                                   Led::MAXIMUM_CURRENT - this->led_blue_current_.Get())
                                   < 0)
            return ret.AddError(LCR4500_SETUP_LED_CURRENTS_FAILED);

    }

    if(setup_default || settings.Contains(this->led_red_edge_delay_falling_)
                     || settings.Contains(this->led_red_edge_delay_rising_)){
        // Get the settings
        settings.Get(&this->led_red_edge_delay_rising_);
        settings.Get(&this->led_red_edge_delay_falling_);

        // Send command and check if it succeeded
        if(DLPC350_SetRedLEDStrobeDelay( this->led_red_edge_delay_rising_.Get(),
                                         this->led_red_edge_delay_falling_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_LED_RED_EDGE_DELAYS_FAILED);

    }

    if(setup_default || settings.Contains(this->led_green_edge_delay_falling_)
                     || settings.Contains(this->led_green_edge_delay_rising_)){
        // Get the settings
        settings.Get(&this->led_green_edge_delay_rising_);
        settings.Get(&this->led_green_edge_delay_falling_);

        // Send command and check if it succeeded
        if(DLPC350_SetGreenLEDStrobeDelay( this->led_green_edge_delay_rising_.Get(),
                                         this->led_green_edge_delay_falling_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_LED_GREEN_EDGE_DELAYS_FAILED);

    }

    if(setup_default || settings.Contains(this->led_blue_edge_delay_falling_)
                     || settings.Contains(this->led_blue_edge_delay_rising_)){
        // Get the settings
        settings.Get(&this->led_blue_edge_delay_rising_);
        settings.Get(&this->led_blue_edge_delay_falling_);

        // Send command and check if it succeeded
        if(DLPC350_SetBlueLEDStrobeDelay( this->led_blue_edge_delay_rising_.Get(),
                                         this->led_blue_edge_delay_falling_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_LED_BLUE_EDGE_DELAYS_FAILED);

    }

    if(setup_default || settings.Contains(this->input_source_)
                     || settings.Contains(this->parallel_port_width_)){
        // Get the value
        settings.Get(&this->input_source_);
        settings.Get(&this->parallel_port_width_);
        if(DLPC350_SetInputSource(this->input_source_.Get(),
                                  this->parallel_port_width_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_INPUT_SOURCE_FAILED);
    }

    if(setup_default || settings.Contains(this->parallel_port_clock_)){
        // Get the value
        settings.Get(&this->parallel_port_clock_);
        if(DLPC350_SetPortClock(this->parallel_port_clock_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_PARALLEL_PORT_CLOCK_FAILED);
    }

    /*
	if(setup_default || settings.Contains(this->parallel_data_swap_)){
        // Get the value
        settings.Get(&this->parallel_data_swap_);
        if(DLPC350_SetDataChannelSwap(LCr4500::Video::DataSwap::Port::PARALLEL_INTERFACE,
                                      this->parallel_data_swap_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_DATA_SWAP_FAILED);
    }
	*/

    if(setup_default || settings.Contains(this->invert_data_)){
        // Get the value and check that it existed
        settings.Get(&this->invert_data_);

        // Send command and check if it succeeded
        if(DLPC350_SetInvertData(this->invert_data_.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_INVERT_DATA_FAILED);
    }

    if(setup_default || settings.Contains(this->display_mode_)){
        bool mode_previous;

        // Set power mode to normal
        if(DLPC350_SetPowerMode(PowerStandbyMode::NORMAL) < 0)
            return ret.AddError(LCR4500_SETUP_POWER_STANDBY_FAILED);

        // Get the pattern
        settings.Get(&this->display_mode_);

        // Check the current mode
        if(DLPC350_GetMode(&mode_previous)<0)
            return ret.AddError(LCR4500_GET_OPERATING_MODE_FAILED);

        // The current mode is different from the requested setting change it
        if(mode_previous != this->display_mode_.Get()){
            if(DLPC350_SetMode(this->display_mode_.Get()) < 0)
                return ret.AddError(LCR4500_SETUP_DISPLAY_MODE_FAILED);
        }
    }

    if(setup_default || settings.Contains(this->test_pattern_foreground_red)
                     || settings.Contains(this->test_pattern_foreground_green)
                     || settings.Contains(this->test_pattern_foreground_blue)
                     || settings.Contains(this->test_pattern_background_red)
                     || settings.Contains(this->test_pattern_background_green)
                     || settings.Contains(this->test_pattern_background_blue))
    {

        // Get the test pattern foreground and background colors
        settings.Get(&this->test_pattern_foreground_red);
        settings.Get(&this->test_pattern_foreground_green);
        settings.Get(&this->test_pattern_foreground_blue);
        settings.Get(&this->test_pattern_background_red);
        settings.Get(&this->test_pattern_background_green);
        settings.Get(&this->test_pattern_background_blue);

        // Send command to LCr4500
        if(DLPC350_SetTPGColor(this->test_pattern_foreground_red.Get(),
                               this->test_pattern_foreground_green.Get(),
                               this->test_pattern_foreground_blue.Get(),
                               this->test_pattern_background_red.Get(),
                               this->test_pattern_background_green.Get(),
                               this->test_pattern_background_blue.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_TEST_PATTERN_COLOR_FAILED);
    }

    if(settings.Contains(Parameters::VideoTestPattern())){
        // Get the value
        Parameters::VideoTestPattern test_pattern;

        // Get the test pattern
        settings.Get(&test_pattern);

        // Set the current power mode
        if(DLPC350_SetPowerMode(PowerStandbyMode::NORMAL) < 0)
            return ret.AddError(LCR4500_SETUP_POWER_STANDBY_FAILED);


        // Check the display mode
        bool mode;
        if(DLPC350_GetMode(&mode)<0)
            return ret.AddError(LCR4500_GET_OPERATING_MODE_FAILED);

        // If the display mode is NOT video switch it
        if(mode != OperatingMode::VIDEO){
            if(DLPC350_SetMode(OperatingMode::VIDEO) < 0)
                return ret.AddError(LCR4500_SETUP_DISPLAY_MODE_FAILED);
        }

        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_SetInputSource(Video::InputSource::INTERNAL_TEST_PATTERNS,
                                  Video::ParallelPortWidth::BITS_30) < 0)
            return ret.AddError(LCR4500_SETUP_INPUT_SOURCE_FAILED);

        dlp::Time::Sleep::Milliseconds(10);

        if(DLPC350_SetTPGColor(0,0,0,1023,1023,1023) < 0)
            return ret.AddError(LCR4500_SETUP_TEST_PATTERN_COLOR_FAILED);

        dlp::Time::Sleep::Milliseconds(10);

        if(DLPC350_SetTPGSelect(test_pattern.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_TEST_PATTERN_FAILED);

    }

    if(settings.Contains(Parameters::VideoFlashImage())){
        // Get the value
        Parameters::VideoFlashImage flash_image;
        bool         mode;
        unsigned int source;
        unsigned int portWidth;

        settings.Get(&flash_image);

        if(DLPC350_SetPowerMode(PowerStandbyMode::NORMAL) < 0)
            return ret.AddError(LCR4500_SETUP_POWER_STANDBY_FAILED);


        // Check the display mode
        if(DLPC350_GetMode(&mode)<0)
            return ret.AddError(LCR4500_GET_OPERATING_MODE_FAILED);

        // If the display mode is NOT video switch it
        if(mode != OperatingMode::VIDEO){
            if(DLPC350_SetMode(OperatingMode::VIDEO) < 0)
                return ret.AddError(LCR4500_SETUP_DISPLAY_MODE_FAILED);
        }

        dlp::Time::Sleep::Milliseconds(10);
        DLPC350_GetInputSource(&source, &portWidth);
        if (source != Video::InputSource::FLASH_IMAGES){
            if(DLPC350_SetInputSource(Video::InputSource::FLASH_IMAGES,
                                      Video::ParallelPortWidth::BITS_30) < 0)
                return ret.AddError(LCR4500_SETUP_INPUT_SOURCE_FAILED);
        }

        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_LoadImageIndex(flash_image.Get()) < 0)
            return ret.AddError(LCR4500_SETUP_FLASH_IMAGE_FAILED);

    }

    if(settings.Contains(this->trigger_source_)){

        // Get the value and check that it existed
        settings.Get(&this->trigger_source_);
    }

    if(settings.Contains(this->sequence_prepared_)){

        // Get the value and check that it existed
        settings.Get(&this->sequence_prepared_);
    }

    if(settings.Contains(this->sequence_exposure_)){

        // Get the value and check that it existed
        settings.Get(&this->sequence_exposure_);
    }

    if(settings.Contains(this->sequence_period_)){

        // Get the value and check that it existed
        settings.Get(&this->sequence_period_);
    }

    if(settings.Contains(this->trigger_in_1_delay_)){

        // Get the value and check that it existed
        settings.Get(&this->trigger_in_1_delay_);

        if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
            return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

        // Send command and check if it succeeded
        if(DLPC350_SetTrigIn1Delay(this->trigger_in_1_delay_.Get()) == -1 )
            return ret.AddError(LCR4500_SETUP_TRIGGER_INPUT_1_DELAY_FAILED);
    }

    if(   settings.Contains(this->trigger_out_1_invert_)
       || settings.Contains(this->trigger_out_1_rising_)
       || settings.Contains(this->trigger_out_1_falling_)){

        // Get the value and check that it existed
        settings.Get(&this->trigger_out_1_invert_);
        settings.Get(&this->trigger_out_1_rising_);
        settings.Get(&this->trigger_out_1_falling_);

        if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
            return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

        // Send command and check if it succeeded
        if(DLPC350_SetTrigOutConfig( LCR4500_TRIGGER_OUT_1,
                                     this->trigger_out_1_invert_.Get(),
                                     this->trigger_out_1_rising_.Get(),
                                     this->trigger_out_1_falling_.Get()) == -1 )
            return ret.AddError(LCR4500_SETUP_TRIGGER_OUTPUT_1_FAILED);
    }

    if(   settings.Contains(this->trigger_out_2_invert_)
       || settings.Contains(this->trigger_out_2_rising_)){

        // Get the value and check that it existed
        settings.Get(&this->trigger_out_2_invert_);
        settings.Get(&this->trigger_out_2_rising_);

        if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
            return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

        // Send command and check if it succeeded
        if(DLPC350_SetTrigOutConfig( LCR4500_TRIGGER_OUT_2,
                                     this->trigger_out_2_invert_.Get(),
                                     this->trigger_out_2_rising_.Get(),
                                     0) == -1 )
            return ret.AddError(LCR4500_SETUP_TRIGGER_OUTPUT_2_FAILED);
    }

    this->is_setup_ = true;

    return ret;
}


/** @brief      Retrieves object settings
 * @param[out]  ret_parameters  Pointer to return \ref dlp::Parameters object containing setup for LightCrafter 4500
 *
 * @retval  LCR4500_NULL_POINT_ARGUMENT_PARAMETERS  The pointer is null
 * @retval  DLP_PLATFORM_NOT_SETUP                  The LightCrafter 4500 has NOT been set up
 */
ReturnCode LCr4500::GetSetup(dlp::Parameters* settings) const{
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!settings)
        return ret.AddError(LCR4500_NULL_POINT_ARGUMENT_PARAMETERS);

    if(!this->isPlatformSetup())
        return ret.AddError(DLP_PLATFORM_NOT_SETUP);

    // Clear the parameters list
    settings->Clear();

    settings->Set(this->dlpc350_firmware_);
    settings->Set(this->dlpc350_flash_parameters_);
    settings->Set(this->dlpc350_image_compression_);
    settings->Set(this->pattern_sequence_firmware_);
    settings->Set(this->use_default_);
    settings->Set(this->power_standby_);
    settings->Set(this->display_mode_);
    settings->Set(this->input_source_);
    settings->Set(this->parallel_port_width_);
    settings->Set(this->parallel_port_clock_);
    settings->Set(this->parallel_data_swap_);
    settings->Set(this->invert_data_);
    settings->Set(this->short_axis_flip_);
    settings->Set(this->long_axis_flip_);
    settings->Set(this->led_sequence_mode_);
    settings->Set(this->led_invert_pwm_);
    settings->Set(this->led_red_enable_);
    settings->Set(this->led_red_current_);
    settings->Set(this->led_red_edge_delay_rising_);
    settings->Set(this->led_red_edge_delay_falling_);
    settings->Set(this->led_green_enable_);
    settings->Set(this->led_green_current_);
    settings->Set(this->led_green_edge_delay_rising_);
    settings->Set(this->led_green_edge_delay_falling_);
    settings->Set(this->led_blue_enable_);
    settings->Set(this->led_blue_current_);
    settings->Set(this->led_blue_edge_delay_rising_);
    settings->Set(this->led_blue_edge_delay_falling_);
    settings->Set(this->trigger_in_1_delay_);
    settings->Set(this->trigger_out_1_invert_);
    settings->Set(this->trigger_out_2_invert_);
    settings->Set(this->trigger_out_1_rising_);
    settings->Set(this->trigger_out_1_falling_);
    settings->Set(this->trigger_out_2_rising_);
    settings->Set(this->verify_image_load_);

    return ret;
}


/** @brief  Projects a continuous solid white pattern from a connected LightCrafter 4500
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS     A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED                   A LightCrafter 4500 EVM has NOT enumerated on the USB
 */
ReturnCode LCr4500::ProjectSolidWhitePattern(){
    ReturnCode ret;

    // If A firmware upload is in progress return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that LCr4500 is connected
    if(this->isConnected()){
        dlp::Pattern::Sequence sequence;
        dlp::Pattern  white_pattern;

        // Setup Pattern
        white_pattern.id        = 0;
        white_pattern.exposure  = this->sequence_exposure_.Get();
        white_pattern.period    = this->sequence_period_.Get();
        white_pattern.bitdepth  = dlp::Pattern::Bitdepth::MONO_1BPP;
        white_pattern.color     = dlp::Pattern::Color::WHITE;
        white_pattern.data_type = dlp::Pattern::DataType::PARAMETERS;
        white_pattern.parameters.Set(Parameters::PatternNumber(LCr4500::Pattern::Number::Mono_1BPP::BLACK));
        white_pattern.parameters.Set(Parameters::PatternImageIndex(0));
        white_pattern.parameters.Set(Parameters::PatternInvert(true));
        white_pattern.parameters.Set(Parameters::PatternShareExposure(false));
        white_pattern.parameters.Set(this->trigger_source_);

        // Add pattern to sequence
        sequence.Add(white_pattern);
        sequence.parameters.Set(Parameters::PatternSequenceRepeat(true));

        // Pattern LUT will be updated so reset these values
        this->previous_sequence_start_    = 0;
        this->previous_sequence_patterns_ = 0;
        this->previous_sequence_repeat_   = false;

        // Send the LUT and start the sequence
        ret = this->CreateSendStartSequenceLut(sequence);
    }
    else{
        // Device NOT connected
        ret.AddError(LCR4500_NOT_CONNECTED);
    }

    return ret;
}

/** @brief  Projects a continuous solid black pattern from a connected LightCrafter 4500
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS     A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED                   A LightCrafter 4500 EVM has NOT enumerated on the USB
 */
ReturnCode LCr4500::ProjectSolidBlackPattern(){
    ReturnCode ret;

    // If A firmware upload is in progress return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that LCr4500 is connected
    if(this->isConnected()){
        dlp::Pattern::Sequence sequence;
        dlp::Pattern  black_pattern;

        // Setup Pattern
        black_pattern.id        = 0;
        black_pattern.exposure  = this->sequence_exposure_.Get();
        black_pattern.period    = this->sequence_period_.Get();
        black_pattern.bitdepth  = dlp::Pattern::Bitdepth::MONO_1BPP;
        black_pattern.color     = dlp::Pattern::Color::WHITE;
        black_pattern.data_type = dlp::Pattern::DataType::PARAMETERS;
        black_pattern.parameters.Set(Parameters::PatternNumber(LCr4500::Pattern::Number::Mono_1BPP::BLACK));
        black_pattern.parameters.Set(Parameters::PatternImageIndex(0));
        black_pattern.parameters.Set(Parameters::PatternInvert(false));
        black_pattern.parameters.Set(Parameters::PatternShareExposure(false));
        black_pattern.parameters.Set(this->trigger_source_);

        // Add pattern to sequence
        sequence.Add(black_pattern);

        sequence.parameters.Set(Parameters::PatternSequenceRepeat(true));

        // Pattern LUT will be updated so reset these values
        this->previous_sequence_start_    = 0;
        this->previous_sequence_patterns_ = 0;
        this->previous_sequence_repeat_   = false;

        // Send the LUT and start the sequence
        ret = this->CreateSendStartSequenceLut(sequence);
    }
    else{
        // Device NOT connected
        ret.AddError(LCR4500_NOT_CONNECTED);
    }

    return ret;
}

/** @brief      Checks for valid pattern settings in \ref dlp::Pattern object
 * @param[in]   arg_pattern \ref dlp::Pattern object to verify
 *
 * @retval  PATTERN_EXPOSURE_TOO_SHORT          Pattern exposure time is set too short
 * @retval  PATTERN_EXPOSURE_TOO_LONG           Pattern exposure time is set too long
 * @retval  PATTERN_BITDEPTH_INVALID            The pattern's bit depth is an invalid value
 * @retval  PATTERN_EXPOSURE_INVALID            Pattern exposure period is an invalid value
 * @retval  FILE_DOES_NOT_EXIST                 The pattern's image file could NOT be found
 * @retval  LCR4500_IMAGE_RESOLUTION_INVALID    The image is of an invalid resolution. Please use the native LCr4500 resolution 912x1140
 * @retval  LCR4500_IMAGE_FORMAT_INVALID        The image format does NOT match the pattern settings color type
 * @retval  LCR4500_PATTERN_NUMBER_PARAMETER_MISSING        The pattern is missing the number parameter
 * @retval  LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING   The pattern is missing the flash image index
 * @retval  PATTERN_DATA_TYPE_INVALID           The pattern isn't of a valid data type
 */
ReturnCode LCr4500::PatternSettingsValid(dlp::Pattern &arg_pattern){
    ReturnCode ret;

    unsigned int exposure   = arg_pattern.exposure;
    unsigned int period     = arg_pattern.period;

    // Check that exposure time is equal to period time or at meets delta requirement
    if((exposure == period) ||
       (exposure <= (period - dlp::LCr4500::Pattern::Exposure::PERIOD_DIFFERENCE_MINIMUM))){


        if(arg_pattern.bitdepth != dlp::Pattern::Bitdepth::INVALID){
            if(exposure <  dlp::LCr4500::Pattern::Exposure::MININUM(arg_pattern.bitdepth ))
                ret.AddError(PATTERN_EXPOSURE_TOO_SHORT);
            if(exposure > dlp::LCr4500::Pattern::Exposure::MAXIMUM)
                ret.AddError(PATTERN_EXPOSURE_TOO_LONG);
        }
        else{
            ret.AddError(PATTERN_BITDEPTH_INVALID);
        }
    }
    else{
        // Exposure period was invalid
        ret.AddError(PATTERN_EXPOSURE_INVALID);
    }

    // Check image resolution if image data or image file supplied
    if(!ret.hasErrors()){
        Image::Format temp_format;

        if(arg_pattern.data_type == dlp::Pattern::DataType::IMAGE_FILE){
            Image temp_image;

            // Check if file exists
            if(!dlp::File::Exists(arg_pattern.image_file))
                return ret.AddError(FILE_DOES_NOT_EXIST);

            // Load the image file
            temp_image.Load(arg_pattern.image_file);

            // Check image resolution
            if(!this->ImageResolutionCorrect(temp_image)){
                return ret.AddError(LCR4500_IMAGE_RESOLUTION_INVALID);
            }

            // Get it's data format
            temp_image.GetDataFormat(&temp_format);

            // If RGB pattern make sure image is RGB also
            if(arg_pattern.color == dlp::Pattern::Color::RGB){
                if(temp_format != Image::Format::RGB_UCHAR)
                    return ret.AddError(LCR4500_IMAGE_FORMAT_INVALID);
            }

            // Clear the image
            temp_image.Clear();
        }
        else if(arg_pattern.data_type == dlp::Pattern::DataType::IMAGE_DATA){
            // Check image resolution
            if(!this->ImageResolutionCorrect(arg_pattern.image_data)){
                return ret.AddError(LCR4500_IMAGE_RESOLUTION_INVALID);
            }

            // Get the image data format
            arg_pattern.image_data.GetDataFormat(&temp_format);

            // If RGB pattern make sure image is also RGB
            if(arg_pattern.color == dlp::Pattern::Color::RGB){
                if(temp_format != Image::Format::RGB_UCHAR)
                    return ret.AddError(LCR4500_IMAGE_FORMAT_INVALID);
            }
        }
        else if(arg_pattern.data_type == dlp::Pattern::DataType::PARAMETERS){
            // No Image resolution check required

            // Check if pattern is RGB or monochrome
            if(arg_pattern.color != dlp::Pattern::Color::RGB){
                // Patern is monochrome

                // Check for pattern number
                if(!arg_pattern.parameters.Contains(Parameters::PatternNumber()))
                    return ret.AddError(LCR4500_PATTERN_NUMBER_PARAMETER_MISSING);

                // Check for image number
                if(!arg_pattern.parameters.Contains(Parameters::PatternImageIndex()))
                    return ret.AddError(LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING);
            }
            else{
                // Pattern is RGB

                // Check for red channel parameters

                    // Check for pattern number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternNumberRed()))
                        return ret.AddError(LCR4500_PATTERN_NUMBER_PARAMETER_MISSING);

                    // Check for image number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternImageIndexRed()))
                        return ret.AddError(LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING);

                // Check for green channel paramaters

                    // Check for pattern number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternNumberGreen()))
                        return ret.AddError(LCR4500_PATTERN_NUMBER_PARAMETER_MISSING);

                    // Check for image number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternImageIndexGreen()))
                        return ret.AddError(LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING);

                // Check for blue channel parameters

                    // Check for pattern number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternNumberBlue()))
                        return ret.AddError(LCR4500_PATTERN_NUMBER_PARAMETER_MISSING);

                    // Check for image number
                    if(!arg_pattern.parameters.Contains(Parameters::PatternImageIndexBlue()))
                        return ret.AddError(LCR4500_PATTERN_FLASH_INDEX_PARAMETER_MISSING);
            }
        }
        else{
            return ret.AddError(PATTERN_DATA_TYPE_INVALID);
        }
    }
    return ret;
}

/** @brief  Creates, sends, and starts a sequence of patterns using the
 *          LightCrafter 4500 look up table format
 *  \note   This function should only be used by experienced DLPC350 programmers
 *  \note   The supplied \ref dlp::Pattern::Sequence MUST use LCr4500 parameters
 * @param[in]   arg_pattern_sequence    object of \ref dlp::Pattern::Sequence type containing sequence
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS             A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED                           The LightCrafter 4500 EVM is NOT connected
 * @retval  PATTERN_SEQUENCE_EMPTY                          The pattern sequence sent contains no patterns
 * @retval  PATTERN_SEQUENCE_EXPOSURES_NOT_EQUAL            The exposure times are NOT equal for each pattern in the sequence
 * @retval  PATTERN_SEQUENCE_PERIODS_NOT_EQUAL              The periods are NOT equal for each pattern in the sequence
 * @retval  PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL        The pattern types are NOT equal for each pattern in the sequence
 * @retval  PATTERN_DATA_TYPE_INVALID                       The data type of the pattern is invalid
 * @retval  PATTERN_SEQUENCE_TOO_LONG                       The pattern sequence excedes the supported number of patterns
 * @retval  LCR4500_FLASH_IMAGE_INDEX_INVALID               A flash image with the specified index is NOT valid
 * @retval  LCR4500_IMAGE_LIST_TOO_LONG                     Too many images have been created to fit in LightCrafter 4500 flash
 * @retval  LCR4500_PATTERN_SEQUENCE_BUFFERSWAP_TIME_ERROR  Buffer swap has occured prematurely
 */
ReturnCode LCr4500::CreateSendStartSequenceLut(const dlp::Pattern::Sequence &arg_pattern_sequence){
    ReturnCode ret;
    unsigned int sequence_count = arg_pattern_sequence.GetCount();

    dlp::Pattern::DataType sequence_type = dlp::Pattern::DataType::INVALID;
    unsigned int    sequence_exposure       = 0;
    unsigned int    sequence_period         = 0;
    unsigned int    sequence_validation     = 0;


    std::vector<LCR4500_LUT_Entry> sequence_LUT;
    std::vector<unsigned char> sequence_image_LUT;

    Parameters::PatternImageIndex flash_image;
    Parameters::PatternImageIndex flash_image_previous;

    dlp::Pattern      temp_pattern;

    // If A firmware upload is in progress return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that LCr4500 is connected
    if(!this->isConnected()){
        // Device NOT connected
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Check that the sequence has entries
    if(sequence_count == 0)
        return ret.AddError(PATTERN_SEQUENCE_EMPTY);

    // Check that all pattern types are equal
    if(!arg_pattern_sequence.EqualDataTypes())
        return ret.AddError(PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL);

    // Get the sequence type
    ret = arg_pattern_sequence.Get(0,&temp_pattern);
    sequence_type = temp_pattern.data_type;

    // Create the new pattern sequence that the LCr4500 can use
    dlp::Pattern::Sequence   sequence_to_project;

    switch(sequence_type){
    case dlp::Pattern::DataType::INVALID:
    case dlp::Pattern::DataType::IMAGE_DATA:
    case dlp::Pattern::DataType::IMAGE_FILE:
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);
    case dlp::Pattern::DataType::PARAMETERS:
        sequence_to_project.Add(arg_pattern_sequence);
        break;
    }

    // Stop the display
    if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
        return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError("LCR4500_GET_STATUS_FAILED - Pattern display stop");

    // Get and set the exposure and trigger time
    sequence_exposure   = this->sequence_exposure_.Get();
    sequence_period     = this->sequence_period_.Get();

    // If the first pattern does not have a set exposure or period
    // assume the entire sequence should use the timing parameters
    // defined during the LCr4500::Setup() method

    // Check if exposure and period have been entered
    if(temp_pattern.exposure == 0 || temp_pattern.period == 0){
        sequence_to_project.SetExposures(sequence_exposure);
        sequence_to_project.SetPeriods(sequence_period);
    }

    // Check pattern sequence timing and create LUT
    for(unsigned int iPat = 0; iPat < sequence_count; iPat++){

        // Check that maximum sequence has NOT been exceeded
        if( sequence_LUT.size() > LCr4500::PATTERN_LUT_SIZE )
            return ret.AddError(PATTERN_SEQUENCE_TOO_LONG);

        // Get the pattern
        temp_pattern.parameters.Clear();
        ret = sequence_to_project.Get(iPat,&temp_pattern);
        if( ret.hasErrors()) return ret;

        // Check the pattern timing
        ret = PatternSettingsValid(temp_pattern);
        if( ret.hasErrors()) return ret;


        // Increment sequence pattern count
        if(temp_pattern.color != dlp::Pattern::Color::RGB){
            // Setup the LUT entry
            LCR4500_LUT_Entry pattern_entry;

            // Copy the pattern timings
            pattern_entry.exposure  = temp_pattern.exposure;
            pattern_entry.period    = temp_pattern.period;

            // Determine trigger type
            Parameters::TriggerSource pattern_trigger;
            temp_pattern.parameters.Get(&pattern_trigger);
            pattern_entry.trigger_type = pattern_trigger.Get();

            // Determine bit depth
            pattern_entry.bit_depth = DlpPatternBitdepthToLCr4500Bitdepth(temp_pattern.bitdepth);

            // Determine LED select
            pattern_entry.LED_select = DlpPatternColorToLCr4500Led(temp_pattern.color);

            // Determine pattern should be inverted
            Parameters::PatternInvert pattern_invert;
            temp_pattern.parameters.Get(&pattern_invert);
            pattern_entry.invert_pattern = pattern_invert.Get();

            // Insert black fill should be used if period and exposure are NOT equal
            // or external triggers are used
            if((temp_pattern.exposure      != temp_pattern.period) ||
               (pattern_entry.trigger_type != LCr4500::Pattern::TriggerSource::INTERNAL)){
                pattern_entry.insert_black = true;
            }
            else{
                pattern_entry.insert_black = false;
            }

            // Determine pattern number
            Parameters::PatternNumber pattern_number;
            temp_pattern.parameters.Get(&pattern_number);
            pattern_entry.pattern_number = pattern_number.Get();

            // Determine pattern image
            temp_pattern.parameters.Get(&flash_image);

            // Check that pattern image is valid
            if(flash_image.Get() >= LCr4500::IMAGE_LUT_SIZE )
                return ret.AddError(LCR4500_FLASH_IMAGE_INDEX_INVALID);

            // Does this pattern have the same image as the previous one?
            if((flash_image.Get() == flash_image_previous.Get()) &&
               (iPat != 0)){    // The first pattern should always have a buffer swap
                // Same image used so buffer swap is NOT needed
                pattern_entry.buffer_swap = false;
            }
            else{
                // Different image so buffer swap is needed
                pattern_entry.buffer_swap = true;

                // Add the image number to the Image LUT
                sequence_image_LUT.push_back(flash_image.Get());
                flash_image_previous.Set(flash_image.Get());
            }

            // Determine share previous trigger out (exposure sharing)
            Parameters::PatternShareExposure share_exposure;
            temp_pattern.parameters.Get(&share_exposure);
            pattern_entry.trigger_out_share_prev = share_exposure.Get();

            // If exposure is shared disable black post-fill
            if( pattern_entry.trigger_out_share_prev ){
                pattern_entry.insert_black = false;
            }

            // If using external trigger and NOT the first pattern, set the
            // previous pattern to clear the DMD after its exposure
            if( (sequence_LUT.size() != 0) &&
               ((pattern_entry.trigger_type == LCr4500::Pattern::TriggerSource::EXTERNAL_NEGATIVE) ||
                (pattern_entry.trigger_type == LCr4500::Pattern::TriggerSource::EXTERNAL_POSITIVE))){
                sequence_LUT.at(sequence_LUT.size()-1).insert_black = true;
            }

            // Non RGB patterns only use one entry slot in LUT
            sequence_LUT.push_back(pattern_entry);
        }
        else{
            // Setup the LUT entry
            LCR4500_LUT_Entry pattern_entry_red;
            LCR4500_LUT_Entry pattern_entry_green;
            LCR4500_LUT_Entry pattern_entry_blue;

            // Copy the pattern timings

            // For the red channel use a third of the pattern exposure
            // time and set the period equal to its channel exposure time
            // so that the green channel displays immediately afterwards
            pattern_entry_red.exposure   = temp_pattern.exposure / 3;
            pattern_entry_red.period     = temp_pattern.exposure / 3;

            // For the green channel use a third of the pattern exposure
            // time and set the period equal to its channel exposure time
            // so that the blue channel displays immediately afterwards
            pattern_entry_green.exposure = temp_pattern.exposure / 3;
            pattern_entry_green.period   = temp_pattern.exposure / 3;

            // For the blue channel use a third of the pattern exposure
            // time and set the blue channels period equal such that the
            // total RGB pattern period will terminate at the same moment
            pattern_entry_blue.exposure =  temp_pattern.exposure / 3;
            pattern_entry_blue.period   = (temp_pattern.period - temp_pattern.exposure) +
                                          (temp_pattern.exposure / 3);

            // Red pattern determines the overall RGB trigger type
            Parameters::TriggerSource pattern_trigger_red;
            temp_pattern.parameters.Get(&pattern_trigger_red);
            pattern_entry_red.trigger_type = pattern_trigger_red.Get();

            // Green and blue patterns don't have a trigger because
            // they are a continuation of the RGB pattern
            pattern_entry_green.trigger_type = LCr4500::Pattern::TriggerSource::NONE;
            pattern_entry_blue.trigger_type  = LCr4500::Pattern::TriggerSource::NONE;

            // Determine bit depth. SDK only enables equal color channel bitdepths for RGB
            pattern_entry_red.bit_depth     = DlpPatternBitdepthToLCr4500Bitdepth(temp_pattern.bitdepth);
            pattern_entry_green.bit_depth   = pattern_entry_red.bit_depth;
            pattern_entry_blue.bit_depth    = pattern_entry_red.bit_depth;

            // Determine LED select
            pattern_entry_red.LED_select   = LCr4500::Pattern::Led::RED;
            pattern_entry_green.LED_select = LCr4500::Pattern::Led::GREEN;
            pattern_entry_blue.LED_select  = LCr4500::Pattern::Led::BLUE;

            // Determine invert pattern
            Parameters::PatternInvert rgb_pattern_invert;
            temp_pattern.parameters.Get(&rgb_pattern_invert);

            pattern_entry_red.invert_pattern    = rgb_pattern_invert.Get();
            pattern_entry_green.invert_pattern  = pattern_entry_red.invert_pattern;
            pattern_entry_blue.invert_pattern   = pattern_entry_red.invert_pattern;

            // Determine pattern number
            Parameters::PatternNumberRed   pattern_number_red;
            Parameters::PatternNumberGreen pattern_number_green;
            Parameters::PatternNumberBlue  pattern_number_blue;

            temp_pattern.parameters.Get(&pattern_number_red);
            pattern_entry_red.pattern_number = pattern_number_red.Get();

            temp_pattern.parameters.Get(&pattern_number_green);
            pattern_entry_green.pattern_number = pattern_number_green.Get();

            temp_pattern.parameters.Get(&pattern_number_blue);
            pattern_entry_blue.pattern_number = pattern_number_blue.Get();

            // Determine buffer swap by if the image changed or not
            Parameters::PatternImageIndexRed   flash_image_red;
            Parameters::PatternImageIndexGreen flash_image_green;
            Parameters::PatternImageIndexBlue  flash_image_blue;

            temp_pattern.parameters.Get(&flash_image_red);

            // Check that pattern image is valid
            if(flash_image_red.Get() >= LCr4500::IMAGE_LUT_SIZE )
                return ret.AddError(LCR4500_FLASH_IMAGE_INDEX_INVALID);

            // Does this pattern have the same image as the previous one?
            if((flash_image_previous.Get() == flash_image_red.Get()) &&
               (iPat != 0)){    // If this is the first pattern add a buffer swap
                // Same image used so buffer swap is NOT needed
                pattern_entry_red.buffer_swap = false;
            }
            else{
                // Different image so buffer swap is needed
                pattern_entry_red.buffer_swap = true;

                // Add the image number to the Image LUT
                sequence_image_LUT.push_back(flash_image_red.Get());
                flash_image_previous.Set(flash_image_red.Get());
            }

            // Determine buffer swap by if the image changed or not
            temp_pattern.parameters.Get(&flash_image_green);

            // Check that pattern image is valid
            if(flash_image_green.Get() >= LCr4500::IMAGE_LUT_SIZE )
                return ret.AddError(LCR4500_FLASH_IMAGE_INDEX_INVALID);

            // Does this pattern have the same image as the previous one?
            // No need to check if first pattern because red portion already
            // would add required buffer swap
            if(flash_image_previous.Get() == flash_image_green.Get() ){
                // Same image used so buffer swap is NOT needed
                pattern_entry_green.buffer_swap = false;
            }
            else{
                // Different image so buffer swap is needed
                pattern_entry_green.buffer_swap = true;

                // Add the image number to the Image LUT
                sequence_image_LUT.push_back(flash_image_green.Get());
                flash_image_previous.Set(flash_image_green.Get());
            }

            // Determine buffer swap by if the image changed or not
            temp_pattern.parameters.Get(&flash_image_blue);

            // Check that pattern image is valid
            if(flash_image_blue.Get() >= LCr4500::IMAGE_LUT_SIZE )
                return ret.AddError(LCR4500_FLASH_IMAGE_INDEX_INVALID);


            // Does this pattern have the same image as the previous one?
            if(flash_image_previous.Get() == flash_image_blue.Get() ){
                // Same image used so buffer swap is NOT needed
                pattern_entry_blue.buffer_swap = false;
            }
            else{
                // Different image so buffer swap is needed
                pattern_entry_blue.buffer_swap = true;

                // Add the image number to the Image LUT
                sequence_image_LUT.push_back(flash_image_blue.Get());
                flash_image_previous.Set(flash_image_blue.Get());
            }

            // Determine share previous trigger out (exposure sharing)
            Parameters::PatternShareExposure share_exposure_rgb;
            temp_pattern.parameters.Get(&share_exposure_rgb);
            pattern_entry_red.trigger_out_share_prev    = share_exposure_rgb.Get();
            pattern_entry_green.trigger_out_share_prev  = true;
            pattern_entry_blue.trigger_out_share_prev   = true;

            // Insert black fill should be used if period and exposure are NOT equal
            if((temp_pattern.exposure          != temp_pattern.period) ||
               (pattern_entry_red.trigger_type != LCr4500::Pattern::TriggerSource::INTERNAL)){
                pattern_entry_red.insert_black      = false;
                pattern_entry_green.insert_black    = false;
                pattern_entry_blue.insert_black     = true;
            }
            else{
                pattern_entry_red.insert_black      = false;
                pattern_entry_green.insert_black    = false;
                pattern_entry_blue.insert_black     = false;
            }

            // If using external trigger and NOT the first pattern, set the
            // previous pattern to clear the DMD after its exposure
            if( (sequence_LUT.size() != 0) &&
               ((pattern_entry_red.trigger_type == LCr4500::Pattern::TriggerSource::EXTERNAL_NEGATIVE) ||
                (pattern_entry_red.trigger_type == LCr4500::Pattern::TriggerSource::EXTERNAL_POSITIVE))){
                sequence_LUT.at(sequence_LUT.size()-1).insert_black = true;
            }

            // Add each single color pattern to create the RGB pattern
            sequence_LUT.push_back(pattern_entry_red);
            sequence_LUT.push_back(pattern_entry_green);
            sequence_LUT.push_back(pattern_entry_blue);
        }
    }


    // Check number of patterns is NOT greater than the LUT size
    if( sequence_LUT.size() > LCr4500::PATTERN_LUT_SIZE )
        return ret.AddError(PATTERN_SEQUENCE_TOO_LONG);

    // Check that the number of images is NOT greater than the image LUT size
    if( sequence_image_LUT.size() > LCr4500::IMAGE_LUT_SIZE)
        return ret.AddError(LCR4500_IMAGE_LIST_TOO_LONG);

    // Check the image load times if there are more than two images
    // If there are more images than the buffer can hold, only one image is preloaded
    if( (sequence_image_LUT.size() > LCr4500::BUFFER_IMAGE_SIZE) &&
        (this->verify_image_load_.Get() > 0)){

        unsigned long long time_since_buffer_swap = 0;  // in microseconds
        unsigned int jImage = 0;

        for(unsigned int iPat = 0; iPat < sequence_LUT.size(); iPat++){

            // Check for bufferswap (do NOT check first pattern because that flash index is always preloaded)
            if(sequence_LUT.at(iPat).buffer_swap && iPat > 0){
                double max_time;

                // Get the average image load time
                this->GetImageLoadTime(jImage,this->verify_image_load_.Get(),&max_time);

                this->debug_.Msg("Image "+dlp::Number::ToString(jImage)+" load time\t= " + dlp::Number::ToString(max_time));
                this->debug_.Msg("Time since buffer swap\t= " + dlp::Number::ToString(time_since_buffer_swap));

                // Check if buffer had enough time to load image
                if(max_time > time_since_buffer_swap)
                    return ret.AddError(LCR4500_PATTERN_SEQUENCE_BUFFERSWAP_TIME_ERROR);

                // Reset time counter
                time_since_buffer_swap = 0;

                // Increment Image index counter
                jImage++;
            }
            time_since_buffer_swap = time_since_buffer_swap + sequence_LUT.at(iPat).period;
        }
    }

    // Stop the sequence if something is already running
    if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
        return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

    // Set power mode to normal
    if(DLPC350_SetPowerMode(PowerStandbyMode::NORMAL) < 0 )
        return ret.AddError(LCR4500_SET_POWER_MODE_FAILED);

    // Change device to pattern sequence mode

    // Check the current mode
    bool mode_previous;
    if(DLPC350_GetMode(&mode_previous)<0)
        return ret.AddError(LCR4500_GET_OPERATING_MODE_FAILED);

    // The current mode is different from the requested setting change it
    if(mode_previous != OperatingMode::PATTERN_SEQUENCE){
        if(DLPC350_SetMode(OperatingMode::PATTERN_SEQUENCE) < 0)
            return ret.AddError(LCR4500_SET_OPERATING_MODE_FAILED);
    }

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError("LCR4500_GET_STATUS_FAILED - get mode or set mode");

    if(DLPC350_SetTrigOutConfig( LCR4500_TRIGGER_OUT_1,
                                 this->trigger_out_1_invert_.Get(),
                                 this->trigger_out_1_rising_.Get(),
                                 this->trigger_out_1_falling_.Get())< 0)
        return ret.AddError(LCR4500_SET_TRIGGER_OUTPUT_CONFIG_FAILED);

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError("LCR4500_GET_STATUS_FAILED - set trigger out config 1");


    if(DLPC350_SetTrigOutConfig( LCR4500_TRIGGER_OUT_2,
                                 this->trigger_out_2_invert_.Get(),
                                 this->trigger_out_2_rising_.Get(),
                                 0)< 0)
        return ret.AddError(LCR4500_SET_TRIGGER_OUTPUT_CONFIG_FAILED);

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError("LCR4500_GET_STATUS_FAILED - set trigger out config 2");

    // Clear the LUT
    DLPC350_ClearExpLut();  // Hardcoded return value

    // Send the add the LUT entries
    for(unsigned int iEntry = 0; iEntry < sequence_LUT.size(); iEntry++ ){
        if(DLPC350_AddToExpLut(sequence_LUT.at(iEntry).trigger_type,
                               sequence_LUT.at(iEntry).pattern_number,
                               sequence_LUT.at(iEntry).bit_depth,
                               sequence_LUT.at(iEntry).LED_select,
                               sequence_LUT.at(iEntry).invert_pattern,
                               sequence_LUT.at(iEntry).insert_black,
                               sequence_LUT.at(iEntry).buffer_swap,
                               sequence_LUT.at(iEntry).trigger_out_share_prev,
                               sequence_LUT.at(iEntry).exposure,
                               sequence_LUT.at(iEntry).period)<0)
            return ret.AddError(LCR4500_ADD_EXP_LUT_ENTRY_FAILED);
    }

    // Currently only flash image source sequences are supported in this LCr4500 module
    // Set device to use flash images
    if(DLPC350_SetPatternDisplayMode(LCr4500::Pattern::Source::FLASH_IMAGES)<0)
        return ret.AddError(LCR4500_SET_PATTERN_DISPLAY_MODE_FAILED);

    // Get the pattern sequence display mode (play once or repeat)
    Parameters::PatternSequenceRepeat repeat_sequence;
    sequence_to_project.parameters.Get(&repeat_sequence);

    // Set the trigger mode
    this->debug_.Msg("Set pattern trigger mode...");
    if(DLPC350_SetPatternTriggerMode(LCr4500::Pattern::TriggerMode::MODE_3_EXP_INT_OR_EXT)<0)
        return ret.AddError(LCR4500_SET_PATTERN_TRIGGER_MODE_FAILED);

    // Send the image LUT
    this->debug_.Msg("Sending image lookup table...");
    if(DLPC350_SendVarExpImageLut( sequence_image_LUT.data(),
                                sequence_image_LUT.size())<0)
        return ret.AddError(LCR4500_SEND_EXP_IMAGE_LUT_FAILED);

    // Send the pattern LUT
    this->debug_.Msg("Sending extended pattern lookup table...");
    if(DLPC350_SendVarExpPatLut()<0)
        return ret.AddError(LCR4500_SEND_EXP_PATTERN_LUT_FAILED);

    // Setup the pattern sequence
    this->debug_.Msg("Configure pattern sequence...");
    if(temp_pattern.color != dlp::Pattern::Color::RGB){
        if(DLPC350_SetVarExpPatternConfig(sequence_LUT.size(),
                                          sequence_LUT.size(),
                                          sequence_image_LUT.size(),
										  repeat_sequence.Get())<0)
            return ret.AddError(LCR4500_SET_VAR_EXP_PATTERN_CONFIG_FAILED);
    }
    else{
        // Adjust the number of patterns to display because the green and blue
        // patterns share their trigger with the red.
        if(DLPC350_SetVarExpPatternConfig(sequence_LUT.size(),
                                          sequence_LUT.size()/3,
                                          sequence_image_LUT.size(),
										  repeat_sequence.Get())<0)
            return ret.AddError(LCR4500_SET_VAR_EXP_PATTERN_CONFIG_FAILED);
    }

    // Validate the sequence
    if(DLPC350_StartPatLutValidate() < 0)
        return ret.AddError(LCR4500_PATTERN_SEQUENCE_VALIDATION_FAILED);

    dlp::Time::Sleep::Milliseconds(100);

    bool dlpc350_ready = false;

    while(!dlpc350_ready){
        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_CheckPatLutValidate(&dlpc350_ready,&sequence_validation) < 0)
            return ret.AddError(LCR4500_PATTERN_SEQUENCE_VALIDATION_FAILED);
    }

    // Display validation data if there was an error
    if( sequence_validation != 0 ){
        this->debug_.Msg("Sequence validation FAILED!");
        ret.AddError(LCR4500_SEQUENCE_VALIDATION_FAILED);

        if((sequence_validation & BIT0) == BIT0){
            this->debug_.Msg("- Exposure or frame period OUT OF RANGE");
            ret.AddError(LCR4500_SEQUENCE_VALIDATION_EXP_OR_PERIOD_OOO);
        }

        if((sequence_validation & BIT1) == BIT1){
            this->debug_.Msg("- Pattern number in lookup table INVALID");
            ret.AddError(LCR4500_SEQUENCE_VALIDATION_PATTERN_NUMBER_INVALID);
        }

        if((sequence_validation & BIT2) == BIT2){
            this->debug_.Msg("- Continued output trigger OVERLAPS black vector");
            ret.AddError(LCR4500_SEQUENCE_VALIDATION_OVERLAP_BLACK);
        }

        if((sequence_validation & BIT3) == BIT3){
            this->debug_.Msg("- Black vector MISSING when exposure less than frame period");
            ret.AddError(LCR4500_SEQUENCE_VALIDATION_BLACK_MISSING);
        }

        if((sequence_validation & BIT4) == BIT4){
            this->debug_.Msg("Difference between exposure and frame period less than 230us");
            ret.AddError(LCR4500_SEQUENCE_VALIDATION_EXP_PERIOD_DELTA_INVALID);
        }

        return ret;
    }

    dlp::Time::Sleep::Milliseconds(10);
    this->debug_.Msg("Start pattern sequence...");
    if( DLPC350_PatternDisplay(Pattern::DisplayControl::START) < 0)
        return ret.AddError(LCR4500_PATTERN_SEQUENCE_START_FAILED);

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError("LCR4500_GET_STATUS_FAILED - pattern display start");

    return ret;
}

/** @brief  Creates a sequence of patterns and firmware images for the LightCrafter 4500,
 *          then uploads the new firmware
 * @param[in]   arg_pattern_sequence    object of \ref dlp::Pattern::Sequence type containing sequence
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS         A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_NOT_CONNECTED                       The LightCrafter 4500 EVM is NOT connected
 * @retval  PATTERN_SEQUENCE_EMPTY                      The supplied sequence contains no patterns
 * @retval  PATTERN_SEQUENCE_EXPOSURES_NOT_EQUAL        The exposure times are NOT equal for each pattern in the sequence
 * @retval  PATTERN_SEQUENCE_PERIODS_NOT_EQUAL          The periods are NOT equal for each pattern in the sequence
 * @retval  PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL    The pattern types are NOT equal for each pattern in the sequence
 */
ReturnCode LCr4500::PreparePatternSequence(const dlp::Pattern::Sequence &pattern_sequence){
    ReturnCode   ret;
    unsigned int sequnce_count = pattern_sequence.GetCount();

    dlp::Pattern::DataType sequence_type = dlp::Pattern::DataType::INVALID;

    // If A firmware upload is in progress, do NOT send any commands until upload is complete! return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that LCr4500 is connected
    if(!this->isConnected()){
        // Device NOT connected
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Check that the sequence has entries
    if(sequnce_count == 0)
        return ret.AddError(PATTERN_SEQUENCE_EMPTY);

    // Reset these values since pattern LUT will be updated
    this->previous_sequence_start_    = 0;
    this->previous_sequence_patterns_ = 0;
    this->previous_sequence_repeat_   = false;

    // Check that all pattern types are equal
    if(!pattern_sequence.EqualDataTypes())
        return ret.AddError(PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL);

    // Get the sequence type from the first pattern
    dlp::Pattern      temp_pattern;
    ret = pattern_sequence.Get(0,&temp_pattern);
    sequence_type = temp_pattern.data_type;

    // Create the new pattern sequence that the LCr4500 can use
    std::vector<std::string> lcr4500_firmware_image_list;

    switch(sequence_type){
    case dlp::Pattern::DataType::INVALID:
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);
    case dlp::Pattern::DataType::IMAGE_DATA:
    case dlp::Pattern::DataType::IMAGE_FILE:
    {
        // Need to make a new pattern sequence of parameters type

        // Create new firmware images and sequence
        std::string firmware_image_file_base = this->pattern_sequence_firmware_.Get() + ".flash_image_";
        ret = this->CreateFirmwareImages( pattern_sequence,
                                          firmware_image_file_base,
                                          this->pattern_sequence_,
                                          lcr4500_firmware_image_list);
        if(ret.hasErrors())
            return ret;

        // Upload the firmware
        if(!this->sequence_prepared_.Get()){
            this->pattern_sequence_prepared_ = false;

            // Create firmware
            ret = this->CreateFirmware(this->pattern_sequence_firmware_.Get(),lcr4500_firmware_image_list);
            if(ret.hasErrors())
                return ret;

            // Upload the firmware
            ret = this->UploadFirmware(this->pattern_sequence_firmware_.Get());
            if(ret.hasErrors())
                return ret;

            // Set flag that firmware has been uploaded
            this->sequence_prepared_.Set(true);
        }

        // Stop the display
        if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
            return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
            return ret.AddError(LCR4500_GET_STATUS_FAILED);

        // Change device to pattern sequence mode
        if(DLPC350_SetPowerMode(PowerStandbyMode::NORMAL)<0)
            return ret.AddError(LCR4500_SET_POWER_MODE_FAILED);

        if(DLPC350_SetMode(OperatingMode::PATTERN_SEQUENCE)<0)
            return ret.AddError(LCR4500_SET_OPERATING_MODE_FAILED);

        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
            return ret.AddError(LCR4500_GET_STATUS_FAILED);

        break;
    }
    case dlp::Pattern::DataType::PARAMETERS:
        this->pattern_sequence_.Clear();
        this->pattern_sequence_.Add(pattern_sequence);
        break;
    }

    this->pattern_sequence_prepared_ = true;

    return ret;
}

/** @brief      Displays a previously prepared pattern sequence
 *  @param[in]  repeat  If true, the sequence repeats after completing
 *  @warning    Must call \ref LCr4500::PreparePatternSequence() before using this method
 *  @retval     LCR4500_PATTERN_SEQUENCE_NOT_PREPARED   The pattern sequence has NOT been prepared and sent to the LightCrafter 4500
 *  @retval     LCR4500_IN_CALIBRATION_MODE             The LightCrafter 4500 is in calibration mode and the sequence cannot be started
 */
ReturnCode LCr4500::StartPatternSequence(const unsigned int &start, const unsigned int &patterns, const bool &repeat){
    ReturnCode ret;

    // Check that sequence has been prepared and that it is NOT a calibration sequence
    if(!this->pattern_sequence_prepared_ )      return ret.AddError(LCR4500_PATTERN_SEQUENCE_NOT_PREPARED);

    // Check that sequence is NOT too long
    if(patterns > LCr4500::PATTERN_LUT_SIZE)
        return ret.AddError(PATTERN_SEQUENCE_TOO_LONG);

    // Check  that the indices are NOT out of range
    if((start + patterns) > this->pattern_sequence_.GetCount())
        return ret.AddError(PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE);


    if((start    != this->previous_sequence_start_)    ||
       (patterns != this->previous_sequence_patterns_) ||
       (repeat   != this->previous_sequence_repeat_)){

        // Create the sequence
        dlp::Pattern::Sequence sequence;
        for(unsigned int iPat = start; iPat < start+patterns; iPat++){
            dlp::Pattern temp;

            // Get the pattern
            this->pattern_sequence_.Get(iPat,&temp);

            // Add it to new sequence
            sequence.Add(temp);
        }

        // Add repeat requence command if needed
        sequence.parameters.Set(Parameters::PatternSequenceRepeat(repeat));

        // Load and start the sequence
        ret = this->CreateSendStartSequenceLut(sequence);

        this->previous_sequence_start_    = start;
        this->previous_sequence_patterns_ = patterns;
        this->previous_sequence_repeat_   = repeat;
    }
    else{
        this->debug_.Msg("Start pattern sequence...");
        if( DLPC350_PatternDisplay(Pattern::DisplayControl::START) < 0)
            return ret.AddError(LCR4500_PATTERN_SEQUENCE_START_FAILED);

        dlp::Time::Sleep::Milliseconds(10);
        if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
            return ret.AddError("LCR4500_GET_STATUS_FAILED - pattern display start");
    }

    return ret;
}

/** @brief      Displays a specific pattern in a previously prepared sequence
 *  @warning    Must call \ref LCr4500::PreparePatternSequence() before using this method
 * @param[in]   pattern_index   index for the pattern to be displayed
 * @param[in]   repeat  bool value, if true, repeat the display of the pattern
 *
 * @retval  LCR4500_PATTERN_SEQUENCE_NOT_PREPARED   The pattern sequence has NOT been prepared and sent to the LightCrafter 4500
 * @retval  PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE     The pattern index number is NOT valid
 */
ReturnCode LCr4500::DisplayPatternInSequence(const unsigned int &pattern_index, const bool &repeat){
    return this->StartPatternSequence(pattern_index,1,repeat);

//    ReturnCode ret;

//    // Check that sequence has been prepared
//    if(!this->pattern_sequence_prepared_)
//        return ret.AddError(LCR4500_PATTERN_SEQUENCE_NOT_PREPARED);

//    // Check that pattern is in range
//    if(pattern_index >= this->pattern_sequence_.GetCount())
//        return ret.AddError(PATTERN_SEQUENCE_INDEX_OUT_OF_RANGE);

//    // Load only next pattern
//    dlp::Pattern next_pattern;

//    // Clear the parameters
//    next_pattern.parameters.Clear();
//    ret = this->pattern_sequence_.Get( pattern_index, &next_pattern);

//    // Check for error
//    if(ret.hasErrors()) return ret;

//    // Load the first pattern as a sequence
//    dlp::Pattern::Sequence single_pattern_sequence;
//    single_pattern_sequence.Clear();
//    single_pattern_sequence.Add(next_pattern);

//    // Add repeat requence command if needed
//    single_pattern_sequence.parameters.Set(Parameters::PatternSequenceRepeat(repeat));

//    // Create and Send the LCr4500 LUT's
//    ret = this->CreateSendStartSequenceLut(single_pattern_sequence);

//    return ret;
}

/** @brief  Stops the current display of a pattern sequence
 *  @retval  LCR4500_NOT_CONNECTED   The LightCrafter 4500 EVM is NOT connected
 */
ReturnCode LCr4500::StopPatternSequence(){
    ReturnCode ret;

    // Check that LCr4500 is connected
    if(!this->isConnected()){
        // Device NOT connected
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Stop the display
    if(DLPC350_PatternDisplay(Pattern::DisplayControl::STOP) < 0)
        return ret.AddError(LCR4500_PATTERN_DISPLAY_FAILED);

    dlp::Time::Sleep::Milliseconds(10);
    if(DLPC350_GetStatus(&this->status_hw_,&this->status_sys_,&this->status_main_)<0)
        return ret.AddError(LCR4500_GET_STATUS_FAILED);

    return ret;
}



/** @brief      Determines which LightCrafter 4500 LED configuration to use
 *              based on the supplied pattern color setting
 * @param[in]   color  color setting in a \ref dlp::Pattern object
 *
 * @retval  LCR4500_COMMAND_FAILED  Could NOT find a valid LED for the color
 */
int LCr4500::DlpPatternColorToLCr4500Led(const dlp::Pattern::Color &color){
    switch(color){
    case dlp::Pattern::Color::RED:        return dlp::LCr4500::Pattern::Led::RED;
    case dlp::Pattern::Color::GREEN:      return dlp::LCr4500::Pattern::Led::GREEN;
    case dlp::Pattern::Color::BLUE:       return dlp::LCr4500::Pattern::Led::BLUE;
    case dlp::Pattern::Color::CYAN:       return dlp::LCr4500::Pattern::Led::CYAN;
    case dlp::Pattern::Color::YELLOW:     return dlp::LCr4500::Pattern::Led::YELLOW;
    case dlp::Pattern::Color::MAGENTA:    return dlp::LCr4500::Pattern::Led::MAGENTA;
    case dlp::Pattern::Color::WHITE:      return dlp::LCr4500::Pattern::Led::WHITE;
    case dlp::Pattern::Color::NONE:       return dlp::LCr4500::Pattern::Led::NONE;
    case dlp::Pattern::Color::BLACK:      return dlp::LCr4500::Pattern::Led::NONE;
    case dlp::Pattern::Color::RGB:        return dlp::LCr4500::Pattern::Led::NONE;
    case dlp::Pattern::Color::INVALID:    return dlp::LCr4500::Pattern::Led::NONE;
    }

    return LCR4500_COMMAND_FAILED;
}

/** @brief      Determines which LightCrafter 4500 bit depth to use based on
 *              the supplied pattern bit depth setting
 * @param[in]   depth  bit depth setting in a \ref dlp::Pattern object
 *
 * @retval  LCR4500_COMMAND_FAILED  Could NOT find a valid bit depth for the pattern
 */
int LCr4500::DlpPatternBitdepthToLCr4500Bitdepth(const dlp::Pattern::Bitdepth &depth){
    switch(depth){
    case dlp::Pattern::Bitdepth::INVALID:     return 0;
    case dlp::Pattern::Bitdepth::MONO_1BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_1BPP;
    case dlp::Pattern::Bitdepth::MONO_2BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_2BPP;
    case dlp::Pattern::Bitdepth::MONO_3BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_3BPP;
    case dlp::Pattern::Bitdepth::MONO_4BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_4BPP;
    case dlp::Pattern::Bitdepth::MONO_5BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_5BPP;
    case dlp::Pattern::Bitdepth::MONO_6BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_6BPP;
    case dlp::Pattern::Bitdepth::MONO_7BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_7BPP;
    case dlp::Pattern::Bitdepth::MONO_8BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_8BPP;
    case dlp::Pattern::Bitdepth::RGB_3BPP:    return dlp::LCr4500::Pattern::Bitdepth::MONO_1BPP;
    case dlp::Pattern::Bitdepth::RGB_6BPP:    return dlp::LCr4500::Pattern::Bitdepth::MONO_2BPP;
    case dlp::Pattern::Bitdepth::RGB_9BPP:    return dlp::LCr4500::Pattern::Bitdepth::MONO_3BPP;
    case dlp::Pattern::Bitdepth::RGB_12BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_4BPP;
    case dlp::Pattern::Bitdepth::RGB_15BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_5BPP;
    case dlp::Pattern::Bitdepth::RGB_18BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_6BPP;
    case dlp::Pattern::Bitdepth::RGB_21BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_7BPP;
    case dlp::Pattern::Bitdepth::RGB_24BPP:   return dlp::LCr4500::Pattern::Bitdepth::MONO_8BPP;
    }
    return LCR4500_COMMAND_FAILED;
}

/** @brief  Uploads firmware file to LightCrafter 4500 EVM
 * @param[in]   firmware_filename  Input file path
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS                 A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_FIRMWARE_FILE_NOT_FOUND                     Could NOT find firmware file
 * @retval  LCR4500_FIRMWARE_FLASH_PARAMETERS_FILE_NOT_FOUND    LightCrafter 4500 flash device parameters file does NOT exist
 * @retval  LCR4500_NOT_CONNECTED                               The LightCrafter 4500 EVM is NOT connected
 * @retval  LCR4500_UNABLE_TO_ENTER_PROGRAMMING_MODE            The LightCrafter 4500 was NOT able to enter programming mode
 * @retval  LCR4500_GET_FLASH_MANUFACTURER_ID_FAILED            The flash manufacturer ID could NOT be read
 * @retval  LCR4500_GET_FLASH_DEVICE_ID_FAILED                  The flash device ID could NOT be read
 * @retval  LCR4500_FLASHDEVICE_PARAMETERS_NOT_FOUND            The flash parameters are NOT valid
 * @retval  LCR4500_COMMAND_FAILED                              The LightCrafter 4500 command failed
 * @retval  LCR4500_FIRMWARE_FLASH_ERASE_FAILED                 The firmware on the LightCrafter 4500 could NOT be erased
 * @retval  LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED           Memory could NOT be allocated on the LightCrafter 4500
 * @retval  LCR4500_FIRMWARE_UPLOAD_FAILED                      The firmware upload failed
 * @retval  LCR4500_FIRMWARE_CHECKSUM_VERIFICATION_FAILED       The firmware checksum could NOT be verified
 * @retval  LCR4500_FIRMWARE_CHECKSUM_MISMATCH                  The uploaded firmware's checksum does NOT match the firmware on the LightCrafter 4500
 */
ReturnCode LCr4500::UploadFirmware(std::string firmware_filename){
    ReturnCode ret;

    std::string flash_parameters_filename = this->dlpc350_flash_parameters_.Get();

    // If A firmware upload is in progress, do NOT send any commands until upload is complete! already return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot upload firmware because upload already in progress");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }
    else{
        // Set the firmware uploading flag since upload has NOT started yet
        while(this->firmware_upload_in_progress.test_and_set()){};
    }

    this->pattern_sequence_prepared_     = false;

    this->firmware_upload_restart_needed = false;

    // Check that the firmware file exists
    this->debug_.Msg("Checking that firmware file " + firmware_filename +" exists...");
    if(!dlp::File::Exists(firmware_filename)){
        this->debug_.Msg("Firmware file " + firmware_filename + " NOT found");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_FILE_NOT_FOUND);
    }
    this->firmwarePath = firmware_filename;

    // Check that the flash parameters file exists
    this->debug_.Msg("Checking that flash parameters file "+ flash_parameters_filename +" exists...");
    if(!dlp::File::Exists(flash_parameters_filename)){
        this->debug_.Msg("Flash parameters file " + flash_parameters_filename + " NOT found");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_FLASH_PARAMETERS_FILE_NOT_FOUND);
    }

    // Note the firmware upload process has begun
    this->firmware_upload_percent_erased_   = 0;
    this->firmware_upload_percent_complete_ = 0;

    unsigned short      manID = 0;
    unsigned long long  devID = 0;

    int startSector = 0;
    int i;
    int BLsize = 0;
    int lastSectorToErase;

    unsigned char *pByteArray=NULL;
    long long dataLen = 0;
    long long dataLen_full = 0;

    int bytesSent;

    unsigned int expectedChecksum = 0;
    unsigned int checksum         = 0;

    bool skip_bootloader = true;

       // Check that LCr4500 is connected
    this->debug_.Msg("Checking that device is connected...");
    if(!this->isConnected()){
        // isConnected() contains debug messages
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Enter programming mode
    this->debug_.Msg("Putting device in programming mode...");
    if(DLPC350_EnterProgrammingMode() < 0){
        this->debug_.Msg("Device did NOT enter programming mode");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_UNABLE_TO_ENTER_PROGRAMMING_MODE);
    }

    // Wait for 5 seconds
    this->firmware_upload_restart_needed = true;
    this->debug_.Msg("Waiting for 5 seconds...");
    dlp::Time::Sleep::Milliseconds(5000);

    // Disconnect the projector
    this->debug_.Msg("Disconnecting device...");
    this->Disconnect();
    this->debug_.Msg("Device disconnected");
    this->debug_.Msg("Waiting for 5 seconds...");
    dlp::Time::Sleep::Milliseconds(5000);

    // Reconnect the projector
    this->debug_.Msg("Connecting to device...");
    std::string id;
    this->GetID(&id);
    this->Connect(id);
    if(!this->isConnected()){
        // isConnected() contains debug messages
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }
    this->debug_.Msg("Connected");
    this->debug_.Msg("Waiting for 5 seconds...");
    dlp::Time::Sleep::Milliseconds(5000);
    this->firmware_upload_restart_needed = false;

    // Enter programming mode
    this->debug_.Msg("Putting device in programming mode...");
    if(DLPC350_EnterProgrammingMode() < 0){
        this->debug_.Msg("Device did NOT enter programming mode");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_UNABLE_TO_ENTER_PROGRAMMING_MODE);
    }

    // Get the connected LCr4500 flashdevice information
    this->debug_.Msg("Retrieving flash manufacturer and device IDs from connected device...");
    if((DLPC350_GetFlashManID(&manID) < 0)){
        this->debug_.Msg("Retrieving flash manufacturer ID FAILED");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_GET_FLASH_MANUFACTURER_ID_FAILED);
    }

    if((DLPC350_GetFlashDevID(&devID) < 0)){
        this->debug_.Msg("Retrieving flash device ID FAILED");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_GET_FLASH_DEVICE_ID_FAILED);
    }

    // Save flash device information to this LCr4500 object
    devID &= 0xFFFF;
    this->myFlashDevice.Mfg_ID = manID;
    this->myFlashDevice.Dev_ID = devID;

    std::ifstream   flash_param_file(flash_parameters_filename);
    std::string     flash_param_file_line;

    // Read in file line by line
    this->debug_.Msg("Searching for flash device parameters in " + flash_parameters_filename + "...");
    bool flashdevice_found = false;
    while(std::getline(flash_param_file,flash_param_file_line) &&
          flashdevice_found == false)
    {
        this->debug_.Msg("...");
        flashdevice_found = this->ProcessFlashParamsLine(flash_param_file_line);
    }
    flash_param_file.close();

    // If the flash device was NOT found in the file return error
    if(flashdevice_found == false){
        this->debug_.Msg("Flash device parameters NOT found");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FLASHDEVICE_PARAMETERS_NOT_FOUND);
    }

    this->debug_.Msg("Flash device parameters found");
    this->debug_.Msg("Waiting 1 second...");
    dlp::Time::Sleep::Milliseconds(1000);

    if(skip_bootloader)
    {
        BLsize = 128 * 1024;
        dataLen -= BLsize;
    }

    // Get the flash device memory address sectors needed for programming
    this->debug_.Msg("Determining amount of memory needed for firmware file...");
    startSector = GetSectorNum(BLsize);
    lastSectorToErase = GetSectorNum(dlp::File::GetSize(firmware_filename));

    //If perfectly aligned with last sector start addr, no need to erase last sector.
    if(dlp::File::GetSize(firmware_filename) == this->myFlashDevice.SectorArr[lastSectorToErase]){
        lastSectorToErase -= 1;
    }

    // Set the flashdevice type on connected LCr4500
    DLPC350_SetFlashType(this->myFlashDevice.Type);

    // Erase the flash sectors on connected LCr4500
    this->debug_.Msg("Erasing flash sectors " + Number::ToString(startSector) + " to " + Number::ToString(lastSectorToErase) + "...");
    for(int iSector=startSector; iSector <= lastSectorToErase; iSector++)
    {
        // Set the flash sector to be erased and erase it
        DLPC350_SetFlashAddr(this->myFlashDevice.SectorArr[iSector]);
        if(DLPC350_FlashSectorErase() < 0){
            this->debug_.Msg("Flash sector " + Number::ToString(iSector) + " FAILED to erase");
            this->firmware_upload_in_progress.clear();
            this->firmware_upload_restart_needed    = false;
            return ret.AddError(LCR4500_FIRMWARE_FLASH_ERASE_FAILED);
        }

        // Wait for the erase command to complete
        DLPC350_WaitForFlashReady();
        this->firmware_upload_percent_erased_ = iSector*100/lastSectorToErase;
        this->debug_.Msg("Flash erase " + Number::ToString(this->GetFirmwareFlashEraseComplete()) + "% complete");
    }
    this->firmware_upload_percent_erased_ = 100;
    this->debug_.Msg("Erasing flash sectors complete");

    // Allocate memory to load the firmware image
    this->debug_.Msg("Allocating memory for firmware image...");
    dataLen = dlp::File::GetSize(firmware_filename);
    pByteArray = new (std::nothrow) unsigned char [dataLen];
    if (pByteArray == nullptr){
        this->debug_.Msg("Allocating memory for firmware image FAILED");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED);
    }

    // Read the firmware iamge into memory
    this->debug_.Msg("Loading firmware image " + firmware_filename + " into memory...");
    std::ifstream firmware(firmware_filename, std::ifstream::binary);
    firmware.read((char *)pByteArray, dataLen);
    firmware.close();

    // Upload the firmware into the EVM
    DLPC350_SetFlashAddr(BLsize);
    dataLen -= BLsize;
    DLPC350_SetUploadSize(dataLen);
    dataLen_full = dataLen;
    this->debug_.Msg("Starting to upload firmware to device...");
    while(dataLen > 0)
    {
        bytesSent = DLPC350_UploadData(pByteArray+BLsize+dataLen_full-dataLen, dataLen);

        if(bytesSent < 0)
        {
            delete[] pByteArray;
            this->debug_.Msg("Firmware upload FAILED");
            this->firmware_upload_in_progress.clear();
            this->firmware_upload_restart_needed    = false;
            return ret.AddError(LCR4500_FIRMWARE_UPLOAD_FAILED);
        }

        for(i=0; i<bytesSent; i++)
        {
            expectedChecksum += pByteArray[BLsize+dataLen_full-dataLen+i];
        }

        dataLen -= bytesSent;

        if(this->firmware_upload_percent_complete_ != (((dataLen_full-dataLen)*100)/dataLen_full))
        {
            this->firmware_upload_percent_complete_ = (((dataLen_full-dataLen)*100)/dataLen_full);
            this->debug_.Msg("Uploading firmware image " + Number::ToString(this->GetFirmwareUploadPercentComplete()) + "% complete");
        }
    }
    this->debug_.Msg("Verifying checksum...");
    if(DLPC350_CalculateFlashChecksum() < 0){
        this->debug_.Msg("Checksum verification FAILED");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_CHECKSUM_VERIFICATION_FAILED);
    }
    this->debug_.Msg("Device calculating checksum...");
    DLPC350_WaitForFlashReady();
    if(DLPC350_GetFlashChecksum(&checksum) < 0){
        this->debug_.Msg("Checksum verification FAILED");
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_CHECKSUM_VERIFICATION_FAILED);
    }
    else if(checksum != expectedChecksum)
    {
        this->debug_.Msg("Checksum mismatch");
        this->debug_.Msg("Expected = " + Number::ToString(expectedChecksum));
        this->debug_.Msg("Received = " + Number::ToString(checksum));
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_FIRMWARE_CHECKSUM_MISMATCH);
    }
    else
    {
        this->debug_.Msg("Exiting programming mode...");
        DLPC350_ExitProgrammingMode(); //Exit programming mode; Start application.
    }

    delete[] pByteArray;

    this->debug_.Msg("Device rebooting...");
    this->debug_.Msg("Waiting 5 seconds...");
    this->firmware_upload_restart_needed = true;
    dlp::Time::Sleep::Milliseconds(5000);

    // Disconnect the projector    
    this->debug_.Msg("Disconnecting device...");
    this->Disconnect();    
    this->debug_.Msg("Waiting 5 seconds...");
    dlp::Time::Sleep::Milliseconds(5000);

    // Reconnect the projector
    this->debug_.Msg("Connecting to device...");
    this->GetID(&id);
    this->Connect(id);
    if(!this->isConnected()){
        // isConnected() contains debug messages
        this->firmware_upload_in_progress.clear();
        this->firmware_upload_restart_needed    = false;
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }
    this->debug_.Msg("Connected");
    this->debug_.Msg("Firmware upload complete");
    this->firmware_upload_in_progress.clear();
    this->firmware_upload_restart_needed    = false;
    return ret;
}

/** @brief  Returns true if a firmware upload is in progress */
bool LCr4500::FirmwareUploadInProgress(){
    if(!this->firmware_upload_in_progress.test_and_set()){
        // Firmware is NOT uploading, clear flag since the above
        // methods sets it
        this->firmware_upload_in_progress.clear();
        return false;
    }
    else{
        // Firmware upload is in progress
        return true;
    }
}

/** @brief  Returns the firmware upload completion in percent */
long long LCr4500::GetFirmwareUploadPercentComplete(){
    return this->firmware_upload_percent_complete_;
}

/** @brief  Returns the firmware flash erase completion in percent */
long long LCr4500::GetFirmwareFlashEraseComplete(){
    return this->firmware_upload_percent_erased_;
}

/** @brief  Gets flash memory device parameters from flash parameter file.
 * Returns false if parameters are NOT found or flash device has too few sectors
 * @param[in]   line    input line for read in from file
 */
bool LCr4500::ProcessFlashParamsLine(const std::string &line)
{
    unsigned int MfgID;
    unsigned int DevID;

    std::vector<std::string> separated_parameters;

    std::string param_line = line;

    // Check that the line is NOT empty
    if(param_line.empty()) return false;

    // Remove leading and trailing whitespace
    param_line = dlp::String::Trim(line);

    // Check that the line does NOT begin with /
    if(param_line.front() == '/') return false;

    // Separate the CSV values
    separated_parameters = dlp::String::SeparateDelimited(param_line,',');

    // Check that there are at least 9 flash device parameters
    // Mfg    MfgID    Device    DevID    Mb    Alg    Size    #sec    Sector_Addresses
    if(separated_parameters.size() <= 9) return false;

    // Read in the HEX manufacturer ID and device ID
    MfgID = dlp::String::ToNumber<unsigned int>(separated_parameters.at(1),16);
    DevID = dlp::String::ToNumber<unsigned int>(separated_parameters.at(3),16);

    // Check is these values match the device currently connected
    if((MfgID == this->myFlashDevice.Mfg_ID) && (DevID == this->myFlashDevice.Dev_ID))
    {
        this->myFlashDevice.Mfg = separated_parameters.at(0);
        this->myFlashDevice.Dev = separated_parameters.at(2);
        this->myFlashDevice.Size_MBit  = dlp::String::ToNumber<unsigned int>(separated_parameters.at(4));
        this->myFlashDevice.Type       = dlp::String::ToNumber<unsigned char>(separated_parameters.at(5));
        this->myFlashDevice.numSectors = dlp::String::ToNumber<unsigned int>(separated_parameters.at(7));

        // Check that flash device has number of sectors
        if(separated_parameters.size() < (8 + this->myFlashDevice.numSectors)) return false;

        for(unsigned int iSector = 0; iSector < this->myFlashDevice.numSectors; iSector++)
        {
            this->myFlashDevice.SectorArr[iSector] = dlp::String::ToNumber<unsigned int>(separated_parameters.at(8 + iSector),16);
        }
        return true;
    }
    return false;
}

/** @brief  Gets flash sector number for a memory address
 * @param[in]   addr    Memory address location for which the sector number is required
 */
int LCr4500::GetSectorNum(unsigned int Addr)
{
    unsigned int i;
    for(i=0; i < this->myFlashDevice.numSectors; i++)
    {
        if(this->myFlashDevice.SectorArr[i] > Addr)
            break;
    }
    return i-1;
}

/** @brief      Creates a firmware file for LightCrafter 4500 EVM
 * @param[in]   new_firmware_filename  file name desired for the created firmware file
 * @param[in]   image_filenames a vector of strings with all images to be included in firmware file
 *
 * @retval  LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS         A firmware upload is in progress, do NOT send any commands until upload is complete!
 * @retval  LCR4500_DLPC350_FIRMWARE_FILE_NOT_FOUND     Could NOT find blank DLPRR350 firmware file
 * @retval  LCR4500_FIRMWARE_FILE_NAME_INVALID          The desired firmware file name is NOT a valid file name
 * @retval  LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED   Could NOT allocate memory for the firmware file
 * @retval  LCR4500_FIRMWARE_FILE_INVALID               The firmware created is invalid
 * @retval  LCR4500_IMAGE_FILE_FORMAT_INVALID           The file formats of the images in the input vector are invalid
 */
ReturnCode LCr4500::CreateFirmware(const std::string &new_firmware_filename, const std::vector<std::string> &image_filenames){
    ReturnCode ret;

    int i = 0;
    int frwm_ret = 0;
    int count = 0;

    long long dataLen;
    unsigned char *pByteArray;

    unsigned char *newFrmwImage;
    unsigned int  newFrmwSize;

    unsigned char compression;
    unsigned int  compSize;


    // If A firmware upload is in progress return error
    if(this->FirmwareUploadInProgress()){
        this->debug_.Msg("Cannot connect because firmware is uploading");
        return ret.AddError(LCR4500_FIRMWARE_UPLOAD_IN_PROGRESS);
    }

    // Check that original DLPC350 firmware file exists
    if(!dlp::File::Exists(this->dlpc350_firmware_.Get())) {
        this->debug_.Msg( "ERROR: Ensure DLPC350 firmware is installed; download link http://www.ti.com/tool/dlpr350");
		this->debug_.Msg( "ERROR: Once installed provide firmware binary location through LCR4500_PARAMETERS_DLPC350_FIRMWARE parameter under projector configuration");
		return ret.AddError(LCR4500_DLPC350_FIRMWARE_FILE_NOT_FOUND);
	}

    // Check that new firmware file name is NOT empty
    if(new_firmware_filename.empty())
        return ret.AddError(LCR4500_FIRMWARE_FILE_NAME_INVALID);

    // Load the original DLPC350 firmware file into the firmware api

    dataLen = dlp::File::GetSize(this->dlpc350_firmware_.Get());
    pByteArray = new (std::nothrow) unsigned char [dataLen];
    if (pByteArray == nullptr)
        return ret.AddError(LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED);

    // Read the firmware into memory
    std::ifstream firmware_orig(this->dlpc350_firmware_.Get(), std::ifstream::binary);
    firmware_orig.read((char *)pByteArray, dataLen);
    firmware_orig.close();

    // Read the firmware into firmware api
    frwm_ret = DLPC350_Frmw_CopyAndVerifyImage(pByteArray, dataLen);
    if (frwm_ret)
    {
        switch(frwm_ret)
        {
        case ERROR_FRMW_FLASH_TABLE_SIGN_MISMATCH:
            this->debug_.Msg( "ERROR: Flash Table Signature doesn't match! Bad Firmware Image!");
            delete[] pByteArray;
            return ret.AddError(LCR4500_FIRMWARE_FILE_INVALID);
        case ERROR_NO_MEM_FOR_MALLOC:
            this->debug_.Msg( "Fatal Error! System Run out of memory");
            delete[] pByteArray;
            return ret.AddError(LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED);
        default:
            break;
        }
    }

    delete[] pByteArray;

    // Check the firmware version
    //if ((DLPC350_Frmw_GetVersionNumber() & 0xFFFFFF) < RELEASE_FW_VERSION)
    //    this->debug_.Msg( "WARNING: Old version of Firmware detected. Download the latest release from http://www.ti.com/tool/dlpr350.");

    firmwarePath = new_firmware_filename;

    // Initialize the firmware image buffer
    count = (int)image_filenames.size();
    DLPC350_Frmw_SPLASH_InitBuffer(count);

    // Create a log file to document firmware build process
    std::fstream log_file_out;
    log_file_out.open("Frmw-build.log", std::fstream::out);

    // Start to build firmware
    log_file_out << "Building Images from specified BMPs\n\n";
    for(i = 0; (i < MAX_SPLASH_IMAGES) && (i < count); i++)
    {
        // Check that the image file is the correct resolution
        // Also checks it the image doesn't exist
        if( !this->ImageResolutionCorrect(image_filenames.at(i)) ){
            this->debug_.Msg("Did NOT add image " + dlp::Number::ToString(image_filenames.at(i)) + " to DLPC350 firmware");
            continue;
        }

        //this->debug_.Msg( "Allocating memory for LightCrafter 4500 firmware image...");
        dataLen = dlp::File::GetSize(image_filenames.at(i));
        pByteArray = new (std::nothrow) unsigned char [dataLen];
        if (pByteArray == nullptr)
            return ret.AddError(LCR4500_IMAGE_MEMORY_ALLOCATION_FAILED);

        // Read the image into memory
        std::ifstream firmware(image_filenames.at(i), std::ifstream::binary);
        firmware.read((char *)pByteArray, dataLen);
        firmware.close();

        // Log the uncompressed image size
        log_file_out << image_filenames.at(i) << "\n";
        log_file_out << "\t" << "Uncompressed Size = " << dataLen << " Compression type : ";

        // Determine if the filename specifies the compression type
        switch (this->dlpc350_image_compression_.Get()) {
        case dlp::LCr4500::ImageCompression::NONE:
            compression = SPLASH_UNCOMPRESSED;
            break;
        case dlp::LCr4500::ImageCompression::RLE:
            compression = SPLASH_RLE_COMPRESSION;
            break;
        case dlp::LCr4500::ImageCompression::FOUR_LINE:
            compression = SPLASH_4LINE_COMPRESSION;
            break;
        case dlp::LCr4500::ImageCompression::UNSPECIFIED:
        default:
            compression = SPLASH_NOCOMP_SPECIFIED;
            break;
        }

//        if (image_filenames.at(i).find("_nocomp.bmp") != std::string::npos){
//            compression = SPLASH_UNCOMPRESSED;
//        }
//        else if (image_filenames.at(i).find("_rle.bmp") != std::string::npos){
//            compression = SPLASH_RLE_COMPRESSION;
//        }
//        else if (image_filenames.at(i).find("_4line.bmp") != std::string::npos){
//            compression = SPLASH_4LINE_COMPRESSION;
//        }
//        else{
//            compression = SPLASH_NOCOMP_SPECIFIED;
//        }

//        compression = SPLASH_UNCOMPRESSED;

        // Add the image to the firmware
        frwm_ret = DLPC350_Frmw_SPLASH_AddSplash(pByteArray, &compression, &compSize);

        // Check if there was an error
        if (frwm_ret < 0)
        {
            switch(frwm_ret)
            {
            case ERROR_NOT_BMP_FILE:
            case ERROR_NOT_24bit_BMP_FILE:
                this->debug_.Msg( "Error building firmware - " + image_filenames.at(i) + " NOT in BMP format");
                return ret.AddError(LCR4500_IMAGE_FILE_FORMAT_INVALID);
            case ERROR_NO_MEM_FOR_MALLOC:
            case ERROR_NO_SPACE_IN_FRMW:
                this->debug_.Msg( "Could not add " + image_filenames.at(i) + " - Insufficient memory");
                return ret.AddError(LCR4500_FIRMWARE_NOT_ENOUGH_MEMORY);
            default:
                this->debug_.Msg( "Error building firmware with " + image_filenames.at(i) + " - Insufficient memory");
                return ret.AddError(LCR4500_FIRMWARE_MEMORY_ALLOCATION_FAILED);
            }
        }

        this->debug_.Msg("Added image " + dlp::Number::ToString(image_filenames.at(i)) + " to DLPC350 firmware");

        // Log the compression applied to image
        switch(compression)
        {
            case SPLASH_UNCOMPRESSED:
                log_file_out << "Uncompressed";
                break;
            case SPLASH_RLE_COMPRESSION:
                log_file_out << "RLE Compression";
                break;
            case SPLASH_4LINE_COMPRESSION:
                log_file_out << "4 Line Compression";
                break;
            default:
                break;
        }

        // Log the compressed size of the image
        log_file_out << " Compressed Size = " << compSize << "\n\n";

        delete[] pByteArray;
    }

    // Close the log file
    log_file_out.close();

    // Get the new flash image data
    DLPC350_Frmw_Get_NewFlashImage(&newFrmwImage, &newFrmwSize);

    // Save the new flash image data to a file
    std::ofstream firmware(new_firmware_filename, std::ofstream::binary);
    firmware.write((char *)newFrmwImage, newFrmwSize);
    firmware.close();

    return ret;
}

/** @brief  Determines the amount of time required to load an image by loading it
 *          a number of times and recording the longest time required
 * @param[in]   index               Flash image index to test load times
 * @param[in]   load_count          Number of times to load the image, should be greater than 5
 * @param[out]  max_microseconds    Pointer to the maximum amount of time it took to load the image
 *
 * @retval  LCR4500_NOT_CONNECTED                       The LightCrafter 4500 EVM is NOT connected
 * @retval  LCR4500_MEASURE_FLASH_LOAD_TIMING_FAILED    The image failed to load or the time failed to be measured
 * @retval  LCR4500_GET_STATUS_FAILED                   Could NOT read the status of the LightCrafter 4500
 * @retval  LCR4500_READ_FLASH_LOAD_TIMING_FAILED       The time for loading the image could NOT be read
 */
ReturnCode LCr4500::GetImageLoadTime(const unsigned int &index, const unsigned int &load_count, double *max_microseconds){
    ReturnCode ret;

    // Check that LCr4500 is connected
    if(!this->isConnected()){
        // Device NOT connected
        return ret.AddError(LCR4500_NOT_CONNECTED);
    }

    // Reset the return value
    (*max_microseconds) = 0;

    // Measure the average image load time for specified image
    for(unsigned int iLoad = 0; iLoad < load_count; iLoad++){
        unsigned int  temp_time;
        unsigned char HWStatus;
        unsigned char SysStatus;
        unsigned char MainStatus;

        // Tell the controller to measure the flash image load time
        if(DLPC350_MeasureImageLoadTiming( index, 1) < 0)
            return ret.AddError(LCR4500_MEASURE_FLASH_LOAD_TIMING_FAILED);

        // Sleep and Read the status bits to allow measurement time
        dlp::Time::Sleep::Milliseconds(100);
        if(DLPC350_GetStatus(&HWStatus, &SysStatus, &MainStatus) < 0)
            return ret.AddError(LCR4500_GET_STATUS_FAILED);

        // Read the load time
        if(DLPC350_ReadImageLoadTiming(&temp_time) < 0)
            return ret.AddError(LCR4500_READ_FLASH_LOAD_TIMING_FAILED);

        // Convert temp time to microseconds
        temp_time = 1000 * temp_time / 18667;

        // Save value if new time is longer
        if(temp_time > (*max_microseconds))
            (*max_microseconds) = temp_time;
    }    

    return ret;
}

/** @brief  Creates images to be included in LightCrafter 4500 firmware from a sequence of patterns
 * @param[in]   arg_pattern_sequence    \ref dlp::Pattern::Sequence type to make firmware images from
 * @param[in]   arg_image_filename_base Base image filename desired, index is appended to final images
 * @param[out]  ret_pattern_sequence    Pointer to return new \ref dlp::Pattern::Sequence
 * @param[out]  ret_image_filename_list vector of strings containing file names of the created firmware images
 *
 * @retval  DLP_PLATFORM_NOT_SETUP                  The LightCrafter 4500 object has NOT been properly set up
 * @retval  LCR4500_FIRMWARE_IMAGE_BASENAME_EMPTY   The desired firmware file name base is invalid
 * @retval  PATTERN_SEQUENCE_EMPTY                  The sequence to create the images from is empty
 * @retval  PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL    The types for the patterns in the sequence do NOT match
 * @retval  PATTERN_DATA_TYPE_INVALID               The data type of the pattern in the sequence is NOT valid
 * @retval  FILE_DOES_NOT_EXIST                     Image file pointed to by pattern does NOT exist
 */
ReturnCode LCr4500::CreateFirmwareImages(const dlp::Pattern::Sequence  &arg_pattern_sequence,
                                         const std::string             &arg_image_filename_base,
                                         dlp::Pattern::Sequence        &ret_pattern_sequence,
                                         std::vector<std::string>      &ret_image_filename_list ){
    ReturnCode ret;

    std::string filename_temp;

    dlp::Pattern temp_pattern;

    unsigned int dmd_rows = 0;
    unsigned int dmd_cols = 0;

    // Check that DLP_Platform is setup
    if(!this->isPlatformSetup())
        return ret.AddError(DLP_PLATFORM_NOT_SETUP);


    // Get DMD columns and rows
    this->GetColumns(&dmd_cols);
    this->GetRows(&dmd_rows);

    this->debug_.Msg("DMD resolution " + dlp::Number::ToString(dmd_cols) + " x " + dlp::Number::ToString(dmd_rows));

    Image temp_firmware_image(dmd_cols,dmd_rows,Image::Format::MONO_INT);
    Image new_firmware_image( dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);

    unsigned char image_bit_position = 0;
    unsigned int  flash_image_index  = 0;

    unsigned char pattern_bpp    = 0;
    unsigned char pattern_number = 0;

    PixelRGB        pattern_pixel_rgb;
    unsigned char   pattern_pixel   = 0;
    unsigned char   pixel_bitdepth_mask    = 0;
    int             temp_pixel      = 0;
    int             fw_pixel        = 0;

    // Check that image filename base is NOT empty
    this->debug_.Msg("Check that the image filename base is NOT empty...");
    if(arg_image_filename_base.empty())
        return ret.AddError(LCR4500_FIRMWARE_IMAGE_BASENAME_EMPTY);
    this->debug_.Msg("Firmware image file basename is " + arg_image_filename_base);


    // Check that sequence is NOT empty
    this->debug_.Msg("Sequence contains " + dlp::Number::ToString(arg_pattern_sequence.GetCount()) + " patterns");
    if(arg_pattern_sequence.GetCount() <= 0 )
        return ret.AddError(PATTERN_SEQUENCE_EMPTY);

    // Check that sequence patterns all have same type
    if(!arg_pattern_sequence.EqualDataTypes())
        return ret.AddError(PATTERN_SEQUENCE_PATTERN_TYPES_NOT_EQUAL);
    this->debug_.Msg("Sequence has equal pattern types");

    // Get the first pattern
    arg_pattern_sequence.Get(0,&temp_pattern);

    // Check that sequence has images or image filenames
    if((temp_pattern.data_type != dlp::Pattern::DataType::IMAGE_DATA) &&
       (temp_pattern.data_type != dlp::Pattern::DataType::IMAGE_FILE))
        return ret.AddError(PATTERN_DATA_TYPE_INVALID);
    this->debug_.Msg("Pattern Type correct");

    // Copy the pattern sequence
    dlp::Pattern::Sequence check_sequence;
    check_sequence.Add(arg_pattern_sequence);

    // Clean the return sequence
    this->debug_.Msg("Clearing return sequence and image file list");
    ret_pattern_sequence.Clear();
    ret_image_filename_list.clear();

    // Set the image bitplane position to zero and zero the image
    image_bit_position = 0;
    temp_firmware_image.FillImage( (int)0);

    this->debug_.Msg("Set all pixels in temp firmware image to black");

    // Create the new sequence and images
    for(unsigned int iPat = 0; iPat < check_sequence.GetCount(); iPat++){
        dlp::Pattern grab_pattern;
        dlp::Image   temp_pattern_image;

        temp_pattern_image.Clear();

        // Get the current pattern from sequence
        this->debug_.Msg("Retrieving pattern " + dlp::Number::ToString(iPat));
        check_sequence.Get(iPat,&grab_pattern);

        // Import the image data
        if(grab_pattern.data_type == dlp::Pattern::DataType::IMAGE_FILE){
            this->debug_.Msg(1,"Pattern Type is image file");

            // Check that file exists
            if(!dlp::File::Exists(grab_pattern.image_file))
                return ret.AddError(FILE_DOES_NOT_EXIST);

            this->debug_.Msg(1,"Pattern image file exists");

            // Load the image if the sequence has NOT been prepared
            // because the image data is already on the device and only
            // the sequence settings need to be determined
            if(!this->sequence_prepared_.Get()){
                temp_pattern_image.Load(grab_pattern.image_file);
            }
        }
        else if(grab_pattern.data_type == dlp::Pattern::DataType::IMAGE_DATA){
            this->debug_.Msg(1,"Pattern Type is image data");

            // Perform shallow copy of the image data
            temp_pattern_image = grab_pattern.image_data;
        }

        // Copy all pattern settings
        this->debug_.Msg(1,"Copy pattern information");
        temp_pattern.id        = grab_pattern.id;
        temp_pattern.bitdepth  = grab_pattern.bitdepth;
        temp_pattern.color     = grab_pattern.color;
        temp_pattern.exposure  = grab_pattern.exposure;
        temp_pattern.period    = grab_pattern.period;
        temp_pattern.data_type = dlp::Pattern::DataType::PARAMETERS;
        temp_pattern.image_data.Clear();
        temp_pattern.parameters.Clear();

        // Check the image resolution
        if(!this->ImageResolutionCorrect(temp_pattern_image))
            return ret.AddError(LCR4500_IMAGE_RESOLUTION_INVALID);

        // If the pattern does not contain a specific trigger source
        // use the one loaded during Setup()
        if(!temp_pattern.parameters.Contains(this->trigger_source_)){
            temp_pattern.parameters.Set(this->trigger_source_);
        }

        // Determine which bitplane to save pattern image to
        pattern_bpp = (unsigned char)DlpPatternBitdepthToLCr4500Bitdepth(temp_pattern.bitdepth);
        this->debug_.Msg(1,"Pattern bpp = " + dlp::Number::ToString(pattern_bpp));
        while(!this->StartPatternImageStorage(image_bit_position,
                                              pattern_bpp,
                                              pattern_number)){
            // Increment image_bit_position counter to find correct
            // bitplane to start writing pattern image
            image_bit_position++;

            // If image_bit_position is greater than 23 the temp_firmware_image
            // hsould be converted to a RGB image and saved to a file.
            if(image_bit_position > 23){
                // If image_bit_position is greater than 23 the temp_firmware_image
                // should be converted to a RGB image and saved to a file.

                // Save the image to a file if sequence has NOT been prepared
                if(!this->sequence_prepared_.Get()){

                    // Create the firmware image filename
                    filename_temp = arg_image_filename_base + dlp::Number::ToString(flash_image_index) + ".bmp";

                    ret = this->SavePatternIntImageAsRGBfile(temp_firmware_image, filename_temp);
                    if(ret.hasErrors()) return ret;

                    // Add image filename to return image list vector
                    ret_image_filename_list.push_back(filename_temp);

                    /// BUG FIX: On Ubuntu, need to zero the matrices before releasing memory.
                    new_firmware_image.FillImage(0);
                    temp_firmware_image.FillImage(0);

                    // Clear the images
                    new_firmware_image.Clear();
                    temp_firmware_image.Clear();

                    // Reallocate the images
                    temp_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::MONO_INT);
                    new_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
                }

                // Reset image_bit_position
                image_bit_position = 0;

                // Increment the flash image index
                flash_image_index++;
            }
        }
        this->debug_.Msg(1,"Bitplane position = " + dlp::Number::ToString(image_bit_position));
        this->debug_.Msg(1,"Pattern number    = " + dlp::Number::ToString(pattern_number));
        this->debug_.Msg(1,"Flash image index = " + dlp::Number::ToString(flash_image_index));

        // Check all previous patterns for identical image data
        bool new_image_data = true;
        for(unsigned int iImageCheck = 0; new_image_data && (iImageCheck < ret_pattern_sequence.GetCount());iImageCheck++){
            dlp::Pattern image_check_pattern;

            ret_pattern_sequence.Get(iImageCheck,&image_check_pattern);

            new_image_data = !dlp::Image::Equal(temp_pattern_image,
                                                image_check_pattern.image_data);

            if(!new_image_data)
                temp_pattern.parameters = image_check_pattern.parameters;
        }

        // Add new image to composite image and create LUT parameters
        if(new_image_data){

            // Determine if pattern is monochrome or RGB
            if(temp_pattern.color != dlp::Pattern::Color::RGB){

                // Pattern is monochrome

                // If the stored image is RGB convert it to monochrome
                temp_pattern_image.ConvertToMonochrome();
                temp_pattern.image_data = temp_pattern_image; // Shallow copy

                // Create mask according to pattern bitdepth
                pixel_bitdepth_mask = (1 << pattern_bpp) - 1;

                // Add the pattern image to the temp image if sequence has NOT been prepared
                if(!this->sequence_prepared_.Get()){
                    for( unsigned int yRow = 0; yRow < dmd_rows; yRow++){
                        for( unsigned int xCol = 0; xCol < dmd_cols; xCol++){
                            // Get the monochrome pixel from the pattern image
                            temp_pattern_image.Unsafe_GetPixel(xCol,yRow,&pattern_pixel);

                            // Mask the pixels according to the pattern bitdepth
                            if(pattern_pixel > pixel_bitdepth_mask){
                                pattern_pixel = pixel_bitdepth_mask;
                            }
                            else{
                                pattern_pixel = pattern_pixel & pixel_bitdepth_mask;
                            }

                            // Convert the pattern image pixel to the FW image pixel
                            temp_pixel = (int) pattern_pixel;

                            // Shift the data to the correct starting bitplane
                            temp_pixel = temp_pixel <<  image_bit_position;

                            // Get the current fw pixel
                            temp_firmware_image.Unsafe_GetPixel( xCol, yRow, &fw_pixel);

                            // Update the fw pixel
                            fw_pixel = temp_pixel + fw_pixel;

                            // Set the uint pixel0
                            temp_firmware_image.Unsafe_SetPixel( xCol, yRow, fw_pixel);
                        }
                    }
                }

                this->debug_.Msg(1,"Setting pattern parameters");

                // Add the flash image index and pattern number to the pattern parameters
                temp_pattern.parameters.Set(Parameters::PatternImageIndex(flash_image_index));
                temp_pattern.parameters.Set(Parameters::PatternNumber(pattern_number));

                // Increment the image bit position so this pattern image is NOT overwritten
                image_bit_position = image_bit_position + pattern_bpp;
            }
            else{
                // Pattern is RGB

                // Check that image is RGB
                dlp::Image::Format temp_image_format;
                temp_pattern_image.GetDataFormat(&temp_image_format);
                if((temp_image_format != dlp::Image::Format::RGB_UCHAR) &&
                        (!this->sequence_prepared_.Get()))
                    return ret.AddError(LCR4500_IMAGE_FORMAT_INVALID);

                // Add the red channel

                // Create mask according to pattern bitdepth
                pixel_bitdepth_mask = (1 << pattern_bpp) - 1;

                // Add the pattern image to the temp image if sequence has NOT been prepared
                if(!this->sequence_prepared_.Get()){

                    for( unsigned int yRow = 0; yRow < dmd_rows; yRow++){
                        for( unsigned int xCol = 0; xCol < dmd_cols; xCol++){
                            // Get the monochrome pixel from the pattern image
                            temp_pattern_image.Unsafe_GetPixel(xCol,yRow,&pattern_pixel_rgb);

                            // Mask the pixels according to the pattern bitdepth
                            if(pattern_pixel_rgb.r > pixel_bitdepth_mask){
                                pattern_pixel_rgb.r = pixel_bitdepth_mask;
                            }
                            else{
                                pattern_pixel_rgb.r = pattern_pixel_rgb.r & pixel_bitdepth_mask;
                            }

                            // Convert the pattern image pixel to the FW image pixel
                            temp_pixel = (int) pattern_pixel_rgb.r;

                            // Shift the data to the correct starting bitplane
                            temp_pixel = temp_pixel <<  image_bit_position;

                            // Get the current fw pixel
                            temp_firmware_image.Unsafe_GetPixel( xCol, yRow, &fw_pixel);

                            // Update the fw pixel
                            fw_pixel = temp_pixel + fw_pixel;

                            // Set the uint pixel
                            temp_firmware_image.Unsafe_SetPixel( xCol, yRow, fw_pixel);
                        }
                    }
                }
                this->debug_.Msg(1,"Setting pattern parameters");

                // Add the flash image index and pattern number to the pattern parameters
                temp_pattern.parameters.Set(Parameters::PatternImageIndexRed(flash_image_index));
                temp_pattern.parameters.Set(Parameters::PatternNumberRed(pattern_number));

                // Increment the image bit position so this pattern image is NOT overwritten
                image_bit_position = image_bit_position + pattern_bpp;

                while(!this->StartPatternImageStorage(image_bit_position,
                                                      pattern_bpp,
                                                      pattern_number)){
                    // Increment image_bit_position counter to find correct
                    // bitplane to start writing pattern image
                    image_bit_position++;

                    // If image_bit_position is greater than 23 the temp_firmware_image
                    // hsould be converted to a RGB image and saved to a file.
                    if(image_bit_position > 23){
                        // If image_bit_position is greater than 23 the temp_firmware_image
                        // should be converted to a RGB image and saved to a file.

                        // Save the image to a file if sequence has NOT been prepared
                        if(!this->sequence_prepared_.Get()){
                            // Create the firmware image filename
                            filename_temp = arg_image_filename_base + dlp::Number::ToString(flash_image_index) + ".bmp";

                            ret = this->SavePatternIntImageAsRGBfile(temp_firmware_image, filename_temp);
                            if(ret.hasErrors()) return ret;

                            // Add image filename to return image list vector
                            ret_image_filename_list.push_back(filename_temp);

                            // Clear the images
                            new_firmware_image.Clear();
                            temp_firmware_image.Clear();

                            // Reallocate the images
                            temp_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::MONO_INT);
                            new_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
                        }

                        // Reset image_bit_position
                        image_bit_position = 0;

                        // Increment the flash image index
                        flash_image_index++;
                    }
                }
                this->debug_.Msg(1,"Bitplane position = " + dlp::Number::ToString(image_bit_position));
                this->debug_.Msg(1,"Pattern number    = " + dlp::Number::ToString(pattern_number));
                this->debug_.Msg(1,"Flash image index = " + dlp::Number::ToString(flash_image_index));

                // Add the green channel

                // Add the pattern image to the temp image if sequence has NOT been prepared
                if(!this->sequence_prepared_.Get()){
                    for( unsigned int yRow = 0; yRow < dmd_rows; yRow++){
                        for( unsigned int xCol = 0; xCol < dmd_cols; xCol++){
                            // Get the monochrome pixel from the pattern image
                            temp_pattern_image.Unsafe_GetPixel(xCol,yRow,&pattern_pixel_rgb);

                            // Mask the pixels according to the pattern bitdepth
                            if(pattern_pixel_rgb.g > pixel_bitdepth_mask){
                                pattern_pixel_rgb.g = pixel_bitdepth_mask;
                            }
                            else{
                                pattern_pixel_rgb.g = pattern_pixel_rgb.g & pixel_bitdepth_mask;
                            }

                            // Convert the pattern image pixel to the FW image pixel
                            temp_pixel = (int) pattern_pixel_rgb.g;

                            // Shift the data to the correct starting bitplane
                            temp_pixel = temp_pixel <<  image_bit_position;

                            // Get the current fw pixel
                            temp_firmware_image.Unsafe_GetPixel( xCol, yRow, &fw_pixel);

                            // Update the fw pixel
                            fw_pixel = temp_pixel + fw_pixel;

                            // Set the uint pixel
                            temp_firmware_image.Unsafe_SetPixel( xCol, yRow, fw_pixel);
                        }
                    }
                }

                this->debug_.Msg(1,"Setting pattern parameters");

                // Add the flash image index and pattern number to the pattern parameters
                temp_pattern.parameters.Set(Parameters::PatternImageIndexGreen(flash_image_index));
                temp_pattern.parameters.Set(Parameters::PatternNumberGreen(pattern_number));

                // Increment the image bit position so this pattern image is NOT overwritten
                image_bit_position = image_bit_position + pattern_bpp;

                while(!this->StartPatternImageStorage(image_bit_position,
                                                      pattern_bpp,
                                                      pattern_number)){
                    // Increment image_bit_position counter to find correct
                    // bitplane to start writing pattern image
                    image_bit_position++;

                    // If image_bit_position is greater than 23 the temp_firmware_image
                    // hsould be converted to a RGB image and saved to a file.
                    if(image_bit_position > 23){
                        // If image_bit_position is greater than 23 the temp_firmware_image
                        // should be converted to a RGB image and saved to a file.

                        // Save the image to a file if sequence has NOT been prepared
                        if(!this->sequence_prepared_.Get()){
                            // Create the firmware image filename
                            filename_temp = arg_image_filename_base + dlp::Number::ToString(flash_image_index) + ".bmp";

                            ret = this->SavePatternIntImageAsRGBfile(temp_firmware_image, filename_temp);
                            if(ret.hasErrors()) return ret;

                            // Add image filename to return image list vector
                            ret_image_filename_list.push_back(filename_temp);

                            // Clear the images
                            new_firmware_image.Clear();
                            temp_firmware_image.Clear();

                            // Reallocate the images
                            temp_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::MONO_INT);
                            new_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
                        }

                        // Reset image_bit_position
                        image_bit_position = 0;

                        // Increment the flash image index
                        flash_image_index++;
                    }
                }
                this->debug_.Msg(1,"Bitplane position = " + dlp::Number::ToString(image_bit_position));
                this->debug_.Msg(1,"Pattern number    = " + dlp::Number::ToString(pattern_number));
                this->debug_.Msg(1,"Flash image index = " + dlp::Number::ToString(flash_image_index));

                // Add the blue channel

                // Add the pattern image to the temp image if sequence has NOT been prepared
                if(!this->sequence_prepared_.Get()){
                    for( unsigned int yRow = 0; yRow < dmd_rows; yRow++){
                        for( unsigned int xCol = 0; xCol < dmd_cols; xCol++){
                            // Get the monochrome pixel from the pattern image
                            temp_pattern_image.Unsafe_GetPixel(xCol,yRow,&pattern_pixel_rgb);

                            // Mask the pixels according to the pattern bitdepth
                            if(pattern_pixel_rgb.b > pixel_bitdepth_mask){
                                pattern_pixel_rgb.b = pixel_bitdepth_mask;
                            }
                            else{
                                pattern_pixel_rgb.b = pattern_pixel_rgb.b & pixel_bitdepth_mask;
                            }

                            // Convert the pattern image pixel to the FW image pixel
                            temp_pixel = (int) pattern_pixel_rgb.b;

                            // Shift the data to the correct starting bitplane
                            temp_pixel = temp_pixel <<  image_bit_position;

                            // Get the current fw pixel
                            temp_firmware_image.Unsafe_GetPixel( xCol, yRow, &fw_pixel);

                            // Update the fw pixel
                            fw_pixel = temp_pixel + fw_pixel;

                            // Set the uint pixel
                            temp_firmware_image.Unsafe_SetPixel( xCol, yRow, fw_pixel);
                        }
                    }
                }

                this->debug_.Msg(1,"Setting pattern parameters");

                // Add the flash image index and pattern number to the pattern parameters
                temp_pattern.parameters.Set(Parameters::PatternImageIndexBlue(flash_image_index));
                temp_pattern.parameters.Set(Parameters::PatternNumberBlue(pattern_number));

                // Increment the image bit position so this pattern image is NOT overwritten
                image_bit_position = image_bit_position + pattern_bpp;

                while(!this->StartPatternImageStorage(image_bit_position,
                                                      pattern_bpp,
                                                      pattern_number)){
                    // Increment image_bit_position counter to find correct
                    // bitplane to start writing pattern image
                    image_bit_position++;

                    // If image_bit_position is greater than 23 the temp_firmware_image
                    // hsould be converted to a RGB image and saved to a file.
                    if(image_bit_position > 23){
                        // If image_bit_position is greater than 23 the temp_firmware_image
                        // should be converted to a RGB image and saved to a file.

                        // Save the image to a file if sequence has NOT been prepared
                        if(!this->sequence_prepared_.Get()){
                            // Create the firmware image filename
                            filename_temp = arg_image_filename_base + dlp::Number::ToString(flash_image_index) + ".bmp";

                            ret = this->SavePatternIntImageAsRGBfile(temp_firmware_image, filename_temp);
                            if(ret.hasErrors()) return ret;

                            // Add image filename to return image list vector
                            ret_image_filename_list.push_back(filename_temp);

                            // Clear the images
                            new_firmware_image.Clear();
                            temp_firmware_image.Clear();

                            // Reallocate the images
                            temp_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::MONO_INT);
                            new_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
                        }

                        // Reset image_bit_position
                        image_bit_position = 0;

                        // Increment the flash image index
                        flash_image_index++;
                    }
                }
            }

        }

        this->debug_.Msg("Adding pattern to sequence");
        ret_pattern_sequence.Add(temp_pattern);
    }

    // Save the image to a file if sequence has NOT been prepared
    if(!this->sequence_prepared_.Get()){
        // Create the firmware image filename
        filename_temp = arg_image_filename_base + dlp::Number::ToString(flash_image_index) + ".bmp";

        ret = this->SavePatternIntImageAsRGBfile(temp_firmware_image, filename_temp);
        if(ret.hasErrors()) return ret;

        // Add image filename to return image list vector
        ret_image_filename_list.push_back(filename_temp);

        // Clear the images
        new_firmware_image.Clear();
        temp_firmware_image.Clear();

        // Reallocate the images
        temp_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::MONO_INT);
        new_firmware_image.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
    }

    // Copy over any parameters
    ret_pattern_sequence.parameters = check_sequence.parameters;

    return ret;
}

/** @brief      Creates images to be included in LightCrafter 4500 firmware from a sequence of patterns
 * @param[in]   bitplane_position   The bit position the pattern will be stored in the image
 * @param[in]   mono_bpp            Bits to be stored in the image for each pixel
 * @param[out]  ret_pattern_number  Returns the LCr4500 flash image pattern number
 */
bool LCr4500::StartPatternImageStorage(const unsigned char &bitplane_position,
                                       const unsigned char &mono_bpp,
                                             unsigned char &ret_pattern_number){
    bool ret = false;
    unsigned char BitplaneStorage[] = {1,2,3,4,6,6,8,8};
    unsigned char BitplaneOffset[]  = {0,0,0,0,1,0,1,0};

    // There can only be 0-23 positions (RGB888 means 24 bitplanes available)
    if(bitplane_position > 23)
        return false;

    if((mono_bpp > 0) && (mono_bpp <= 8)){
        ret = ((bitplane_position - BitplaneOffset[mono_bpp-1]) % BitplaneStorage[mono_bpp-1]) == 0;
    }

    // Calculate pattern number
    ret_pattern_number = (bitplane_position - BitplaneOffset[mono_bpp-1])/ BitplaneStorage[mono_bpp-1];

    return ret;
}

/** @brief      Creates images to be included in LightCrafter 4500 firmware from an INT image
 * @param[in]   image_int       object of \ref dlp::Image type containing pattern image
 * @param[in]   filename        file name to save image object
 *
 * @retval  DLP_PLATFORM_NOT_SETUP  The LightCrafter 4500 object has NOT been properly set up
 */
ReturnCode LCr4500::SavePatternIntImageAsRGBfile(Image &image_int, const std::string &filename){
    ReturnCode ret;

    Image image_rgb;
    PixelRGB  new_pixel;

    int             pixel_int;
    unsigned int    pixel;

    unsigned int    dmd_rows;
    unsigned int    dmd_cols;

    // Check that DLP_Platform has been setup
    if(!this->isPlatformSetup())
        return ret.AddError(DLP_PLATFORM_NOT_SETUP);

    // Get DMD size
    this->GetColumns(&dmd_cols);
    this->GetRows(&dmd_rows);

    // Create the RGB image in memory
    ret = image_rgb.Create(dmd_cols,dmd_rows,Image::Format::RGB_UCHAR);
    if(ret.hasErrors()) return ret;

    // Fill the new RGB image with zeros
    new_pixel.r = 0;
    new_pixel.g = 0;
    new_pixel.b = 0;
    ret = image_rgb.FillImage(new_pixel);
    if(ret.hasErrors()) return ret;

    // Create the RGB image pixel by pixel
    for(unsigned int yRow = 0; yRow < dmd_rows; yRow++){
        for(unsigned int xCol = 0; xCol < dmd_cols; xCol++){

            // Get the int pixel
            image_int.Unsafe_GetPixel(xCol,yRow, &pixel_int);

            // Create the RGB pixel
            pixel = (unsigned int) pixel_int;
            new_pixel.r = (pixel >>  8) & 255;   // Pattern Image bitplanes  8 - 15
            new_pixel.g = (pixel >>  0) & 255;   // Pattern Image bitplanes  0 -  7
            new_pixel.b = (pixel >> 16) & 255;   // Pattern Image bitplanes 16 - 23

            // Set the pixel
            image_rgb.Unsafe_SetPixel(xCol,yRow,new_pixel);
        }
    }

    // Save the image to a file
    ret = image_rgb.Save(filename);

    return ret;
}

namespace String{

template <> dlp::LCr4500::ImageCompression ToNumber(const std::string &text, unsigned int base ){
    // Ignore base variable
    (void)base; // Remove unused variable warning
    if (text.compare("NONE") == 0)      return dlp::LCr4500::ImageCompression::NONE;
    if (text.compare("RLE") == 0)       return dlp::LCr4500::ImageCompression::RLE;
    if (text.compare("FOUR_LINE") == 0) return dlp::LCr4500::ImageCompression::FOUR_LINE;

    return dlp::LCr4500::ImageCompression::UNSPECIFIED;
}

template <> dlp::LCr4500::PowerStandbyMode::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::PowerStandbyMode::Enum)dlp::String::ToNumber<int>(text);
}

template <> dlp::LCr4500::ImageFlip::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::ImageFlip::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::InputSource::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::InputSource::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::ParallelPortWidth::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::ParallelPortWidth::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::ParallelClockPort::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::ParallelClockPort::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::DataSwap::SubChannels::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::DataSwap::SubChannels::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::DataSwap::Port::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::DataSwap::Port::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Video::TestPattern::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Video::TestPattern::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::OperatingMode::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::OperatingMode::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::InvertData::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::InvertData::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Source::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Source::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::TriggerMode::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::TriggerMode::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::TriggerSource::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::TriggerSource::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Led::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Led::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Bitdepth::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Bitdepth::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum)dlp::String::ToNumber<int>(text);
}
template <> dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum ToNumber( const std::string &text, unsigned int base ){
    (void)base; // Remove unused variable warning
    return (dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum)dlp::String::ToNumber<int>(text);
}
}

namespace Number{

template <> std::string ToString<dlp::LCr4500::ImageCompression>(dlp::LCr4500::ImageCompression value ){
    switch(value){
        case dlp::LCr4500::ImageCompression::NONE:          return "NONE";
        case dlp::LCr4500::ImageCompression::FOUR_LINE:     return "FOUR_LINE";
        case dlp::LCr4500::ImageCompression::RLE:           return "RLE";
        case dlp::LCr4500::ImageCompression::UNSPECIFIED:   return "UNSPECIFIED";
    }
    return "UNSPECIFIED";
}

template <> std::string ToString<dlp::LCr4500::PowerStandbyMode::Enum>( dlp::LCr4500::PowerStandbyMode::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::ImageFlip::Enum>( dlp::LCr4500::ImageFlip::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::InputSource::Enum>( dlp::LCr4500::Video::InputSource::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::ParallelPortWidth::Enum>(LCr4500::Video::ParallelPortWidth::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::ParallelClockPort::Enum>( dlp::LCr4500::Video::ParallelClockPort::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::DataSwap::SubChannels::Enum>( dlp::LCr4500::Video::DataSwap::SubChannels::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::DataSwap::Port::Enum>( dlp::LCr4500::Video::DataSwap::Port::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Video::TestPattern::Enum>( dlp::LCr4500::Video::TestPattern::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::OperatingMode::Enum>( dlp::LCr4500::OperatingMode::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::InvertData::Enum>( dlp::LCr4500::InvertData::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Source::Enum>( dlp::LCr4500::Pattern::Source::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::TriggerMode::Enum>( dlp::LCr4500::Pattern::TriggerMode::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::TriggerSource::Enum>( dlp::LCr4500::Pattern::TriggerSource::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Led::Enum>( dlp::LCr4500::Pattern::Led::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Bitdepth::Enum>( dlp::LCr4500::Pattern::Bitdepth::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_1BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_2BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_3BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_4BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_5BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_6BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_7BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}

template <> std::string ToString<dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum>( dlp::LCr4500::Pattern::Number::Mono_8BPP::Enum value ){
    return dlp::Number::ToString((int) value);
}
}
}

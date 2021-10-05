/** @file       image.hpp
 *  @ingroup    Common
 *  @brief      Defines dlp::Image class
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_IMAGE_HPP
#define DLP_SDK_IMAGE_HPP

#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/other.hpp>

#include <opencv2/opencv.hpp>

#define IMAGE_FORMAT_UNKNOWN                    "IMAGE_FORMAT_UNKNOWN"
#define IMAGE_FORMAT_NOT_MONO                   "IMAGE_FORMAT_NOT_MONO"
#define IMAGE_EMPTY                             "IMAGE_EMPTY"
#define IMAGE_ALREADY_CREATED                   "IMAGE_ALREADY_CREATED"
#define IMAGE_STORED_IN_DIFFERENT_FORMAT        "IMAGE_STORED_IN_DIFFERENT_FORMAT"
#define IMAGE_CREATION_FAILED                   "IMAGE_CREATION_FAILED"
#define IMAGE_INPUT_EMPTY                       "IMAGE_INPUT_EMPTY"
#define IMAGE_FILENAME_EMPTY                    "IMAGE_FILENAME_EMPTY"
#define IMAGE_FILE_FORMAT_INVALID               "IMAGE_FILE_FORMAT_INVALID"
#define IMAGE_FILE_LOAD_FAILED                  "IMAGE_FILE_LOAD_FAILED"
#define IMAGE_FILE_SAVE_FAILED                  "IMAGE_FILE_SAVE_FAILED"
#define IMAGE_PIXEL_OUT_OF_RANGE                "IMAGE_PIXEL_OUT_OF_RANGE"
#define IMAGE_CONVERT_TO_MONOCHROME_FAILED      "IMAGE_CONVERT_TO_MONOCHROME_FAILED"
#define IMAGE_NULL_POINTER_ARGUMENT_DATA        "IMAGE_NULL_POINTER_ARGUMENT_DATA"
#define IMAGE_NULL_POINTER_ARGUMENT_FORMAT      "IMAGE_NULL_POINTER_ARGUMENT_FORMAT"
#define IMAGE_NULL_POINTER_ARGUMENT_ROWS        "IMAGE_NULL_POINTER_ARGUMENT_ROWS"
#define IMAGE_NULL_POINTER_ARGUMENT_COLUMNS     "IMAGE_NULL_POINTER_ARGUMENT_COLUMNS"
#define IMAGE_NULL_POINTER_ARGUMENT_RET_VAL     "IMAGE_NULL_POINTER_ARGUMENT_RET_VAL"
#define IMAGE_ALREADY_MONOCHROME                "IMAGE_ALREADY_MONOCHROME"
#define IMAGE_ALREADY_RGB                       "IMAGE_ALREADY_RGB"
#define IMAGE_WINDOW_NAME_TAKEN                 "IMAGE_WINDOW_NAME_TAKEN"
#define IMAGE_WINDOW_NOT_OPEN                   "IMAGE_WINDOW_NOT_OPEN"
#define IMAGE_WINDOW_NULL_POINTER_KEY_RETURN    "IMAGE_WINDOW_NULL_POINTER_KEY_RETURN"

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @class  PixelRGB
 *  @brief  Container for unsigned char red, green, and blue values
 */
class PixelRGB{
public:
    PixelRGB();
    PixelRGB(const unsigned char &red, const unsigned char &green, const unsigned char &blue);

    unsigned char r;
    unsigned char g;
    unsigned char b;
};

/** @class      Image
 *  @ingroup    Common
 *  @brief      Container class for image data implemented with OpenCV cv::Mat
 *  @example    image_file_io.cpp
 *  @example    image_pixels.cpp
 *  @example    image_window.cpp
 */
class Image{
public:

    enum class Format{
        MONO_UCHAR,         /*!< monochrome unsigned char                   */
        MONO_CHAR,          /*!< monochrome signed char                     */
        MONO_INT,           /*!< monochrome signed int                      */
        MONO_FLOAT,         /*!< monochrome float                           */
        MONO_DOUBLE,        /*!< monochrome double                          */
        RGB_UCHAR,          /*!< color (red, green, blue) unsigned char     */
        INVALID             /*!< invalid format or image has not been setup */
    };

    /** @class      Window
     *  @ingroup    Common
     *  @brief      Displays a \ref dlp::Image with an OpenCV window
     *  @example    image_window.cpp
     */
    class Window{
    public:
        Window();
        ~Window();

        bool        isOpen();
        ReturnCode  Open(const std::string &name);
        ReturnCode  Open(const std::string &name, const Image &image);
        ReturnCode  Open(const std::string &name, const dlp::Image &image, const unsigned int &width, const unsigned int &height );
        ReturnCode  Update(const Image &image);
        ReturnCode  Update(const Image &image, const unsigned int &width, const unsigned int &height);
        ReturnCode  WaitForKey(const unsigned int &delay_millisecs, unsigned int *key_return);
        ReturnCode  WaitForKey(const unsigned int &delay_millisecs);
        void      Close();
    private:
        DISALLOW_COPY_AND_ASSIGN(Window);

        bool         open_;
        std::string  name_;
        unsigned int id_;
        cv::Mat      image_;

        static std::vector<std::string> open_windows_;
    };


    Image();
    ~Image();
    Image(const unsigned int &cols, const unsigned int &rows, const Format &format);
    Image(const unsigned int &cols, const unsigned int &rows, const Format &format, void *data);
    Image(const unsigned int &cols, const unsigned int &rows, const Format &format, void *data, const size_t &step); //Number of bytes each matrix row occupies. The value should include the padding bytes at the end of each row, if any
    Image(const cv::Mat &src_cvmat);


    static ReturnCode  ConvertFormatDLPtoOpenCV(const Format &dlp_format, int* opencv_format);
    static ReturnCode  ConvertFormatOpenCVtoDLP(const int &cv_format, Format* dlp_format);


    ReturnCode  Create(const unsigned int &columns, const unsigned int &rows, const Format &format);
    ReturnCode  Create(const unsigned int &columns, const unsigned int &rows,
                       const Format &format, void *data);
    ReturnCode  Create(const unsigned int &columns, const unsigned int &rows,
                       const Format &format, void *data, const size_t &step);
    ReturnCode  Create(const Image &src_image );
    ReturnCode  Create(const cv::Mat &src_data );

    void Clear();

    bool isEmpty()const;

    ReturnCode  GetOpenCVData(cv::Mat *data)const;
    ReturnCode  Unsafe_GetOpenCVData(cv::Mat *data);

    ReturnCode  ConvertToMonochrome();
    ReturnCode  ConvertToRGB();

    ReturnCode  Load(const std::string &filename);
    ReturnCode  Save(const std::string &filename) const;
    ReturnCode  Save(const std::string &filename, const unsigned int &comp_or_qual) const;

    ReturnCode  GetDataFormat(Format *format) const;
    ReturnCode  GetRows(unsigned int  *rows) const;
    ReturnCode  GetColumns(unsigned int *columns) const;

    ReturnCode  GetSum( double *ret_val) const;

    ReturnCode  GetMean( char          *ret_val) const;
    ReturnCode  GetMean( unsigned char *ret_val) const;
    ReturnCode  GetMean( PixelRGB      *ret_val) const;
    ReturnCode  GetMean( int           *ret_val) const;
    ReturnCode  GetMean( float         *ret_val) const;
    ReturnCode  GetMean( double        *ret_val) const;

    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,           char *ret_val) const;
    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,  unsigned char *ret_val) const;
    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,       PixelRGB *ret_val) const;
    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,            int *ret_val) const;
    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,          float *ret_val) const;
    ReturnCode  GetPixel( const unsigned int &x, const unsigned int &y,         double *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,           char *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,  unsigned char *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,       PixelRGB *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,            int *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,          float *ret_val) const;
    void Unsafe_GetPixel( const unsigned int &x, const unsigned int &y,         double *ret_val) const;

    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,           char arg_val);
    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,  unsigned char arg_val);
    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,       PixelRGB arg_val);
    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,            int arg_val);
    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,          float arg_val);
    ReturnCode  SetPixel( const unsigned int &x, const unsigned int &y,         double arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,           char arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,  unsigned char arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,       PixelRGB arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,            int arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,          float arg_val);
    void Unsafe_SetPixel( const unsigned int &x, const unsigned int &y,         double arg_val);

    ReturnCode  FillImage( unsigned char arg_val);
    ReturnCode  FillImage(          char arg_val);
    ReturnCode  FillImage(     PixelRGB arg_val);
    ReturnCode  FillImage(           int arg_val);
    ReturnCode  FillImage(         float arg_val);
    ReturnCode  FillImage(        double arg_val);

    ReturnCode FlipImage( const bool &flip_x, const bool &flip_y);

    static bool Equal(const dlp::Image &a, const dlp::Image &b);


protected:
    cv::Mat data_;
    Format  format_;
    bool    empty_;
};

namespace Number{
template <> std::string ToString<dlp::PixelRGB>( dlp::PixelRGB pixel );
template <> std::string ToString<dlp::Image::Format>( dlp::Image::Format format );
}

namespace String{
template <> dlp::PixelRGB ToNumber( const std::string &text, unsigned int base  );
}



}

#endif // #ifndef DLP_SDK_IMAGE_HPP




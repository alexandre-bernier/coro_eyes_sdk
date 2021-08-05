/** @file   image.cpp
 *  @brief  Contains methods to read and modify dlp::Image objects
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <string>
#include <vector>
#include <iostream>

#include <common/returncode.hpp>
#include <common/debug.hpp>
#include <common/other.hpp>
#include <common/image/image.hpp>

#include <opencv2/opencv.hpp>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{


bool Image::Equal(const dlp::Image &a, const dlp::Image &b){
    if(a.format_ != b.format_)       return false;
    if(a.data_.rows != b.data_.rows) return false;
    if(a.data_.cols != b.data_.cols) return false;

    cv::Mat temp_dst = a.data_.clone();

    if(a.format_ != dlp::Image::Format::RGB_UCHAR){
        cv::compare(a.data_,b.data_,temp_dst,cv::CMP_NE);
        if(cv::sum(temp_dst).val[0]>0) return false;
    }
    else{
        // Split the channels
        cv::Mat a_split[3];
        cv::Mat b_split[3];

        cv::split(a.data_,a_split);
        cv::split(b.data_,b_split);

        temp_dst.release();
        temp_dst = a_split[0].clone();

        // Compare
        cv::compare(a_split[0],b_split[0],temp_dst,cv::CMP_NE);
        if(cv::sum(temp_dst).val[0]>0) return false;

        cv::compare(a_split[1],b_split[1],temp_dst,cv::CMP_NE);
        if(cv::sum(temp_dst).val[0]>0) return false;

        cv::compare(a_split[2],b_split[2],temp_dst,cv::CMP_NE);
        if(cv::sum(temp_dst).val[0]>0) return false;
    }

    return true;

    //cv::Scalar s = cv::sum( a.data_ - b.data_ );


    //return (s[0]==0) && (s[1]==0) && (s[2]==0);
}


/** @brief      Converts \ref dlp::Image::Format to OpenCV image format value
 *  @param[in]  dlp_format              Supplied \ref dlp::Image::Format
 *  @param[out] opencv_format           Returned OpenCV image format
 *  @retval     IMAGE_FORMAT_UNKNOWN    Supplied \ref dlp::Image::Format was unknown and cannot be converted
 */
ReturnCode Image::ConvertFormatDLPtoOpenCV(const Format &dlp_format, int* opencv_format){
    ReturnCode ret;

    // Determine the image format
    switch(dlp_format){
    case Format::MONO_UCHAR:
        (*opencv_format) = CV_8UC1;
        break;
    case Format::MONO_CHAR:
        (*opencv_format) = CV_8SC1;
        break;
    case Format::MONO_INT:
        (*opencv_format) = CV_32S;
        break;
    case Format::MONO_FLOAT:
        (*opencv_format) = CV_32F;
        break;
    case Format::MONO_DOUBLE:
        (*opencv_format) = CV_64F;
        break;
    case Format::RGB_UCHAR:
        (*opencv_format) = CV_8UC3;
        break;
    case Format::INVALID:
        (*opencv_format) = 0;
    default:
        ret.AddError(IMAGE_FORMAT_UNKNOWN);
    }

    return ret;
}

/** @brief      Converts OpenCV image format value to \ref dlp::Image::Format
 *  @param[in]  cv_format           Supplied OpenCV image format
 *  @param[out] dlp_format              Returned \ref dlp::Image::Format
 *  @retval     IMAGE_FORMAT_UNKNOWN    Supplied OpenCV image format was unknown or isn't supported by \ref dlp::Image
 */
ReturnCode Image::ConvertFormatOpenCVtoDLP(const int &cv_format, Format* dlp_format){
    ReturnCode ret;

    // Determine the image format
    // Check that the format is valid within the SDK
    switch(cv_format){
    case CV_8UC1:
        (*dlp_format) = Format::MONO_UCHAR;
        break;
    case CV_8SC1:
        (*dlp_format) = Format::MONO_CHAR;
        break;
    case CV_32S:
        (*dlp_format) = Format::MONO_INT;
        break;
    case CV_32F:
        (*dlp_format) = Format::MONO_FLOAT;
        break;
    case CV_64F:
        (*dlp_format) = Format::MONO_DOUBLE;
        break;
    case CV_8UC3:
        (*dlp_format) = Format::RGB_UCHAR;
        break;
    default:
        (*dlp_format) = Format::INVALID;
        ret.AddError(IMAGE_FORMAT_UNKNOWN);
    }
    return ret;
}


/** @brief Initializes PixelRGB object with all colors equal to zero */
PixelRGB::PixelRGB(){
    this->r = 0;
    this->g = 0;
    this->b = 0;
}

/** @brief Initializes PixelRGB object with all colors equal to supplied unsigned char's */
PixelRGB::PixelRGB(const unsigned char &red,
                   const unsigned char &green,
                   const unsigned char &blue){
    this->r = red;
    this->g = green;
    this->b = blue;
}

/** @brief Initializes empty object with no set resolution or format */
Image::Image(){
    // Note that this image object has no data upon construction
    this->empty_       = true;
}

/** @brief Initializes empty object with the specified resolution and format
 *  @param[in] cols     Number of columns
 *  @param[in] rows     Number of rows
 *  @param[in] format   \ref dlp::Image::Format of image pixels
 */
Image::Image(const unsigned int &cols, const unsigned int &rows, const Format &format){
    // Note that this image object has no data upon construction
    this->empty_       = true;

    // Create blank image with supplied arguments
    this->Create(cols,rows,format);
}

/** @brief Initializes object data from supplied pointer
 *  @param[in] cols     Number of columns
 *  @param[in] rows     Number of rows
 *  @param[in] format   dlp::Image::Format of image pixels
 *  @param[in] data     Pointer to image data
 */
Image::Image(const unsigned int &cols, const unsigned int &rows, const Format &format, void *data){
    // Note that this initialized image object has no data
    this->empty_       = true;

    // Create image with supplied arguments
    this->Create(cols,rows,format,data);
}


/** @brief Initializes object from supplied pointer
 *  @param[in] cols     Number of columns
 *  @param[in] rows     Number of rows
 *  @param[in] format   \ref dlp::Image::Format of image pixels
 *  @param[in] data     Pointer to image data
 *  @param[in] step     Number of bytes each matrix row occupies (this value should include any padding bytes)
 */
Image::Image(const unsigned int &cols, const unsigned int &rows, const Format &format, void *data, const size_t &step){
    // Note that this initialized image object has no data
    this->empty_       = true;

    // Create image with supplied arguments
    this->Create(cols,rows,format,data,step);
}

/** @brief Initializes object from supplied OpenCV Mat object
 *  @param[in] src_cvmat cv::Mat object to copy image data from
 */
Image::Image(const cv::Mat &src_cvmat){
    // Note that this initialized image object has no data
    this->empty_       = true;

    // Create image with supplied arguments
    this->Create(src_cvmat);
}

/** @brief Deconstructs the object and deallocates memory */
Image::~Image(){
    // Release image data
    this->Clear();
}


/** @brief      Allocates memory for object
 *  @warning    This method clears any previous data stored in the object
 *  @param[in]  columns                 Number of columns
 *  @param[in]  rows                    Number of rows
 *  @param[in]  format                  \ref dlp::Image::Format of image pixels
 *  @retval     IMAGE_CREATION_FAILED   Memory allocation failed
 *  @retval     IMAGE_FORMAT_UNKNOWN    Supplied \ref dlp::Image::Format is invalid
 */
ReturnCode Image::Create(const unsigned int &columns, const unsigned int &rows, const Format &format){
    ReturnCode ret;
    int        cv_format   = 0;

    // Convert the DLP image format to OpenCV format
    ret = this->ConvertFormatDLPtoOpenCV(format, &cv_format);

    // Check the the format is valid
    if(ret.hasErrors())
        return ret;

    // Clear the image if it's already been created
    this->Clear();

    // Create the image data
    this->data_.create(rows,columns,cv_format);

    // Check that the image data was created successfully
    if(this->data_.empty() != true){
        this->format_   = format;       // Set the Image::Format
        this->empty_    = false;        // Set empty to false since the image has data
    }
    else{
        // Image data creation failed, set empty_ to true
        this->empty_ = true;
        this->data_.release();
        ret.AddError(IMAGE_CREATION_FAILED);
    }

    return ret;
}

/** @brief Allocates memory for object and copies data from pointer
 *  @warning This method clears any previous data stored in the object.
 *  @param[in]  columns                             Number of columns
 *  @param[in]  rows                                Number of rows
 *  @param[in]  format                              dlp::Image::Format of image pixels
 *  @param[in]  data                                Pointer to image data
 *  @retval     IMAGE_FORMAT_UNKNOWN                Supplied \ref dlp::Image::Format is invalid
 *  @retval     IMAGE_CREATION_FAILED               Memory allocation failed
 *  @retval     IMAGE_NULL_POINTER_ARGUMENT_DATA    Input argument NULL
 */
ReturnCode Image::Create(const unsigned int &columns, const unsigned int &rows,
                             const Format &format, void *data){
    ReturnCode  ret;
    int         cv_format   = 0;

    // Convert the DLP3D ImageFormat to OpenCV format
    ret = this->ConvertFormatDLPtoOpenCV(format, &cv_format);

    // Check the the format is valid
    if(ret.hasErrors())
        return ret;

    // Check that pointer is NOT NULL
    if(!data) return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_DATA);

    // Clear the image
    this->Clear();

    // Create the image data
    this->data_ = cv::Mat(rows, columns, cv_format, data).clone();

    // Check that the image data was created successfully
    if(this->data_.empty() != true){
        this->format_   = format;       // Set the ImageFormat
        this->empty_    = false;        // Set empty to false since the image has data
    }
    else{
        // Image data creation failed, set empty_ to true
        this->empty_ = true;
        ret.AddError(IMAGE_CREATION_FAILED);
    }

    return ret;
}


/** @brief Allocates memory for object and copies data from pointer
 *  @warning This method clears any previous data stored in the object.
 *  @param[in]  columns  Number of columns
 *  @param[in]  rows     Number of rows
 *  @param[in]  format   \ref dlp::Image::Format of image pixels
 *  @param[in]  data     Pointer to image data
 *  @param[in]  step     Number of bytes each matrix row occupies (this value should include any padding bytes)
 *  @retval     IMAGE_CREATION_FAILED               Memory allocation failed
 *  @retval     IMAGE_FORMAT_UNKNOWN                Supplied \ref dlp::Image::Format is invalid
 *  @retval     IMAGE_NULL_POINTER_ARGUMENT_DATA    Input argument NULL
 */
ReturnCode Image::Create(const unsigned int &columns, const unsigned int &rows,
                             const Format &format, void *data, const size_t &step){
    ReturnCode ret;
    int        cv_format   = 0;

    // Convert the DLP3D ImageFormat to OpenCV format
    ret = this->ConvertFormatDLPtoOpenCV(format, &cv_format);

    // Check the the format is valid
    if(ret.hasErrors())
        return ret;

    // Check that pointer is NOT NULL
    if(!data) return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_DATA);

    // Clear the image
    this->Clear();

    // Create the image data
    this->data_ = cv::Mat(rows, columns, cv_format, data, step).clone();

    // Check that the image data was created successfully
    if(this->data_.empty() != true){
        this->format_   = format;       // Set the ImageFormat
        this->empty_    = false;        // Set empty to false since the image has data
    }
    else{
        // Image data creation failed, set empty_ to true
        this->empty_ = true;
        ret.AddError(IMAGE_CREATION_FAILED);
    }

    return ret;
}


/** @brief Allocates memory for object and copies data from cv::Mat object
 *  @warning This method clears any previous data stored in the object.
 *  @param[in] src_data             cv::Mat object to copy image data from
 *  @retval IMAGE_INPUT_EMPTY       Supplied cv::Mat object is empty
 *  @retval IMAGE_FORMAT_UNKNOWN    Supplied cv::Mat is in an unsupported format
 */
ReturnCode Image::Create(const cv::Mat &src_data ){
    ReturnCode  ret;

    // Check that the cv::Mat argument has data
    if(src_data.empty() == true)
        return ret.AddError(IMAGE_INPUT_EMPTY);

    // Get the format of the cv::Mat argument
    int cv_format   = 0;
    cv_format = src_data.type();

    // Convert the DLP3D ImageFormat to OpenCV format
    Format format;
    ret = ConvertFormatOpenCVtoDLP(cv_format,&format);

    // Check the the format is valid
    if(ret.hasErrors())
        return ret;

    // Clear the current image
    this->Clear();

    // Clone the image data
    this->data_ = src_data.clone();

    // Check that the image data was created successfully
    if(this->data_.empty() != true){
        this->format_   = format;       // Set the ImageFormat
        this->empty_    = false;        // Set empty to false since the image has data
    }
    else{
        // Image data creation failed, set empty_ to true
        this->empty_ = true;
        ret.AddError(IMAGE_CREATION_FAILED);
    }

    return ret;
}

/** @brief Allocates memory for object and copies data from source \ref dlp::Image object
 *  @warning This method clears any previous data stored in the object.
 *  @param[in] src_image cv::Mat object to copy image data from
 *  @retval IMAGE_INPUT_EMPTY       Supplied cv::Mat object is empty
 */
ReturnCode Image::Create( const dlp::Image &src_image ){
    ReturnCode   ret;

    if(src_image.isEmpty())
        return ret.AddError(IMAGE_INPUT_EMPTY);

    cv::Mat data;
    src_image.GetOpenCVData(&data);

    // Create the new image
    ret = this->Create(data);

    // Release opencv mat variable
    data.release();

    return ret;
}

/** @brief      Loads image file into dlp::Image object
 *  @warning    This method clears any previous data stored in the object
 *  @param[in]  filename                name of file of image file
 *  @retval     FILE_DOES_NOT_EXIST     Supplied image file does NOT exist
 *  @retval     IMAGE_FILE_LOAD_FAILED  Could NOT load image from file
 *  @retval     IMAGE_CREATION_FAILED   Memory allocation failed
 *  @retval     IMAGE_FORMAT_UNKNOWN    Loaded image file format was unknown or is NOT supported by \ref dlp::Image
 */
ReturnCode Image::Load(const std::string &filename){
    ReturnCode   ret;

    // Check that file exists
    if(!dlp::File::Exists(filename))
        return ret.AddError(FILE_DOES_NOT_EXIST);

    // Clear the image
    this->Clear();

    // Load the file
    this->data_ = cv::imread(filename, cv::IMREAD_UNCHANGED);

    // Check that the image loaded correctly
    if(!this->data_.data){
        // Image failed to load properly, set empty_ to true
        this->empty_ = true;
        this->data_.release();
        return ret.AddError(IMAGE_FILE_LOAD_FAILED);
    }

    // Get the format of the cv::Mat argument
    int cv_format   = 0;
    cv_format = this->data_.type();

    // Check that the format is valid within the SDK
    Format format;
    ret = this->ConvertFormatOpenCVtoDLP(cv_format, &format);
    if(ret.hasErrors()){
        // The format was invalid
        this->empty_ = true;            // Set empty to true sine the image was never created
        this->data_.release();
        return ret;
    }

    // Check that the image data was created successfully
    if(this->data_.empty() != true){
        this->format_   = format;       // Set the ImageFormat
        this->empty_    = false;        // Set empty to false since the image has data
    }
    else{
        // Image data creation failed, set empty_ to true
        this->empty_ = true;
        this->data_.release();
        ret.AddError(IMAGE_CREATION_FAILED);
    }

    return ret;
}

/** @brief Deallocates memory for object */
void Image::Clear(){
    ReturnCode ret;

    // Release any image data
    this->data_.release();

    // Set empty_ to true
    this->empty_ = true;

    // Set format to invalid
    this->format_ = Format::INVALID;

    return;
}

/** @brief Returns true if object has NOT been created */
bool Image::isEmpty() const{
    return this->empty_;
}

/** @brief      Clones the object's data into an OpenCV cv::Mat object
 *  \note       This method is a deep copy
 *  @param[out] data cv::Mat object pointer to copy image data to
 *  @retval     IMAGE_NULL_POINTER_ARGUMENT_DATA    Input argument NULL
 *  @retval     IMAGE_EMPTY                         Object has no data to copy
 */
ReturnCode Image::GetOpenCVData(cv::Mat *data)const{
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!data) return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_DATA);

    // Check that the image has data (a.k.a. it is NOT empty)
    if(this->empty_ != true ){
        // Clone the image data
        (*data) = this->data_.clone();
    }
    else{
        // The image is empty
        ret.AddError(IMAGE_EMPTY);
    }

    return ret;
}

/** @brief      Copies the object's data pointer into an OpenCV cv::Mat object
 *  @warning    This method is a shallow copy of the image data! The cv::Mat pointer has direct access to the \ref dlp::Image data and CAN modify it
 *  @param[out] data        cv::Mat object pointer to copy image data to
 *  @retval     IMAGE_NULL_POINTER_ARGUMENT_DATA    Input argument NULL
 *  @retval     IMAGE_EMPTY                         Object has no data to copy
 */
ReturnCode Image::Unsafe_GetOpenCVData(cv::Mat *data){
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!data) return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_DATA);


    // Check that the image has data (a.k.a. it is NOT empty)
    if(this->empty_ != true ){
        (*data) = this->data_;
    }
    else{
        // The image is empty
        ret.AddError(IMAGE_EMPTY);
    }

    return ret;
}

/** @brief Saves object to file
 *
 *  Supported file formats are BMP, JPEG, and PNG
 *
 *  @param[in] filename File to save image to
 *  @retval IMAGE_EMPTY                 Object has no data to save
 *  @retval IMAGE_FILENAME_EMPTY        Supplied string is empty
 *  @retval IMAGE_FILE_FORMAT_INVALID   Filename is NOT a supported image file format
 *  @retval IMAGE_FILE_SAVE_FAILED      Could NOT save file
 */
ReturnCode Image::Save(const std::string &filename) const{
    ReturnCode ret;

    // Check that image has data
    if( this->empty_ != true ){
        unsigned int type_jpeg    = filename.rfind(".jpeg");
        unsigned int type_JPEG    = filename.rfind(".JPEG");
        unsigned int type_png     = filename.rfind(".png");
        unsigned int type_PNG     = filename.rfind(".PNG");
        unsigned int type_bmp     = filename.rfind(".bmp");
        unsigned int type_BMP     = filename.rfind(".BMP");

        // Determine filetype
        if((type_jpeg!=std::string::npos)||         // Check for JPEG
           (type_JPEG!=std::string::npos)){
            ret = this->Save(filename,95);
        }
        else if((type_png!=std::string::npos)||     // Check for PNG
                (type_PNG!=std::string::npos)){
            ret = this->Save(filename,9);
        }
        else if((type_bmp!=std::string::npos)||     // Check for BMP
                (type_BMP!=std::string::npos)){
            ret = this->Save(filename,0);
        }
        else{                                       // Invalid filetype
            ret.AddError(IMAGE_FILE_FORMAT_INVALID);
        }
    }
    else{
        ret.AddError(IMAGE_FILENAME_EMPTY);
    }

    return ret;
}

/** @brief Saves object to file
 *
 *  Supported file formats are BMP, JPEG, and PNG
 *
 *  @param[in] filename         File to save image to
 *  @param[in] comp_or_qual     Images compression (JPEG 0-100) or quality (PNG 0-9)
 *  @retval IMAGE_EMPTY                 Object has no data to save
 *  @retval IMAGE_FILENAME_EMPTY        Supplied string is empty
 *  @retval IMAGE_FILE_FORMAT_INVALID   Filename is NOT a supported image file format
 *  @retval IMAGE_FILE_SAVE_FAILED      Could NOT save file
 */
ReturnCode Image::Save(const std::string &filename, const unsigned int &comp_or_qual) const{
    ReturnCode   ret;
    int         comp_or_qual_val = comp_or_qual;

    // check that filename is NOT empty
    if(filename.empty()) return ret.AddError(IMAGE_FILENAME_EMPTY);

    // Check that image has data
    if( this->empty_ != true ){
        std::vector<int> image_params;
        unsigned int type_jpeg    = filename.rfind(".jpeg");
        unsigned int type_JPEG    = filename.rfind(".JPEG");
        unsigned int type_png     = filename.rfind(".png");
        unsigned int type_PNG     = filename.rfind(".PNG");
        unsigned int type_bmp     = filename.rfind(".bmp");
        unsigned int type_BMP     = filename.rfind(".BMP");

        // Determine filetype
        if((type_jpeg!=std::string::npos)||         // Check for JPEG
           (type_JPEG!=std::string::npos)){
            // Check comp_or_qual constraints
            if(comp_or_qual_val>100){
                comp_or_qual_val = 100;
            }

            // Setup parameter vector
            image_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            image_params.push_back((int)comp_or_qual_val);

            // Write the image file
            if(!cv::imwrite(filename,this->data_,image_params)){
                ret.AddError(IMAGE_FILE_SAVE_FAILED);
            }
        }
        else if((type_png!=std::string::npos)||     // Check for PNG
                (type_PNG!=std::string::npos)){
            // Check comp_or_qual constraints
            if(comp_or_qual_val>9){
                comp_or_qual_val = 9;
            }

            // Setup parameter vector
            image_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            image_params.push_back((int)comp_or_qual_val);

            // Write the image file
            if(!cv::imwrite(filename,this->data_,image_params)){
                ret.AddError(IMAGE_FILE_SAVE_FAILED);
            }
        }
        else if((type_bmp!=std::string::npos)||     // Check for BMP
                (type_BMP!=std::string::npos)){
            // Write the image file
            if(!cv::imwrite(filename,this->data_)){
                ret.AddError(IMAGE_FILE_SAVE_FAILED);
            }
        }
        else{                                       // Invalid filetype
            ret.AddError(IMAGE_FILE_FORMAT_INVALID);
        }
    }
    else{
        ret.AddError(IMAGE_EMPTY);
    }

    return ret;
}


/**
 * @brief       Retrieves \ref dlp::Image::Format of object
 * @param[out]  format  \ref dlp::Image::Format pointer to return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_FORMAT  Return argument is NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 */
ReturnCode Image::GetDataFormat(Format *format) const{
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!format)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_FORMAT);

    // Check that image has data
    if( this->empty_ != true ){
        (*format) = this->format_;
    }
    else{
        ret.AddError(IMAGE_EMPTY);
    }

    return ret;
}

/**
 *  @brief Retrieves number of rows (height) of the image
 *  @param[out] rows    Pointer for return value
 *  @retval IMAGE_NULL_POINTER_ARGUMENT_ROWS Return argument NULL
 *  @retval IMAGE_EMPTY                      Image has NOT been created
 */
ReturnCode Image::GetRows(unsigned int *rows) const{
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!rows)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_ROWS);

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Set ret pointer to correct value
    (*rows) = (unsigned int) this->data_.rows;

    return ret;
}

/**
 *  @brief Retrieves number of columns (width) in an image
 *  @param[out] columns     Pointer for return value
 *  @retval IMAGE_NULL_POINTER_ARGUMENT_ROWS Return argument NULL
 *  @retval IMAGE_EMPTY                      Image has NOT been created
 */
ReturnCode Image::GetColumns(unsigned int *columns) const{
    ReturnCode ret;

    // Check that pointer is NOT null
    if(!columns)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_ROWS);

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Set ret pointer to correct value
    (*columns) = (unsigned int) this->data_.cols;

    return ret;
}



/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(char *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_CHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Return value
    (*ret_val) = cv::mean(this->data_).val[0];

    return ret;
}

/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(unsigned char *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Return value
    (*ret_val) = cv::mean(this->data_).val[0];

    return ret;
}

/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(PixelRGB *ret_val) const{
    ReturnCode ret;
    cv::Scalar temp_pxl_cv;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    ret_val->r = 0;
    ret_val->g = 0;
    ret_val->b = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::RGB_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    temp_pxl_cv = cv::mean(this->data_);
    ret_val->r = temp_pxl_cv.val[2];
    ret_val->g = temp_pxl_cv.val[1];
    ret_val->b = temp_pxl_cv.val[0];

    return ret;
}

/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(int *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_INT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = cv::mean(this->data_).val[0];

    return ret;
}

/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(float *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_FLOAT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = cv::mean(this->data_).val[0];

    return ret;
}

/**
 * @brief Retrieves the average pixel value of the image
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetMean(double *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_DOUBLE )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = cv::mean(this->data_).val[0];

    return ret;
}

ReturnCode  Image::GetSum( double *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::RGB_UCHAR ){

        // Set ret pointer to correct value
        (*ret_val) = cv::sum(this->data_).val[0];
    }
    else{
        cv::Scalar temp = cv::sum(this->data_);
        (*ret_val) = temp.val[0] + temp.val[1] + temp.val[2];
    }

    return ret;
}


/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, char *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::MONO_CHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Return value
    (*ret_val) = this->data_.at<char>(y,x);

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, unsigned char *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::MONO_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Return value
    (*ret_val) = this->data_.at<unsigned char>(y,x);

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, PixelRGB *ret_val) const{
    ReturnCode ret;
    cv::Vec3b temp_pxl_cv;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    ret_val->r = 0;
    ret_val->g = 0;
    ret_val->b = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::RGB_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    temp_pxl_cv = this->data_.at<cv::Vec3b>(y,x);
    ret_val->r = temp_pxl_cv.val[2];
    ret_val->g = temp_pxl_cv.val[1];
    ret_val->b = temp_pxl_cv.val[0];

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, int *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::MONO_INT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<int>(y,x);

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, float *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::MONO_FLOAT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<float>(y,x);

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 * @retval      IMAGE_NULL_POINTER_ARGUMENT_RET_VAL Return argument NULL
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to retrieve pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::GetPixel(const unsigned int &x, const unsigned int &y, double *ret_val) const{
    ReturnCode ret;

    // Check that pointer exists
    if(!ret_val)
        return ret.AddError(IMAGE_NULL_POINTER_ARGUMENT_RET_VAL);

    // Set pointer to zero in case image is empty
    (*ret_val) = 0;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);

    // Check the format
    if( this->format_ != Format::MONO_DOUBLE )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<double>(y,x);

    return ret;
}


/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, char arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::MONO_CHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Store the pixel value
    this->data_.at<char>(y,x) = arg_val;

    return ret;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, unsigned char arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::MONO_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Store the pixel value
    this->data_.at<unsigned char>(y,x) = arg_val;

    return ret;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, PixelRGB arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::RGB_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Convert format
    cv::Vec3b temp_pxl_cv;
    temp_pxl_cv.val[2] = arg_val.r;
    temp_pxl_cv.val[1] = arg_val.g;
    temp_pxl_cv.val[0] = arg_val.b;

    // Store the pixel value
    this->data_.at<cv::Vec3b>(y,x) = temp_pxl_cv;

    return ret;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, int arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::MONO_INT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Store the pixel value
    this->data_.at<int>(y,x) = arg_val;

    return ret;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, float arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::MONO_FLOAT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Store the pixel value
    this->data_.at<float>(y,x) = arg_val;

    return ret;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_PIXEL_OUT_OF_RANGE            Supplied coordinates are NOT valid
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to store pixel value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::SetPixel( const unsigned int &x, const unsigned int &y, double arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the pixel location
    if(x >= (unsigned int) this->data_.cols ||
       y >= (unsigned int) this->data_.rows)
        return ret.AddError(IMAGE_PIXEL_OUT_OF_RANGE);


    // Check the format
    if( this->format_ != Format::MONO_DOUBLE )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Store the pixel value
    this->data_.at<double>(y,x) = arg_val;

    return ret;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, char *ret_val) const{

    // Return value
    (*ret_val) = this->data_.at<char>(y,x);

    return;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, unsigned char *ret_val) const{

    // Return value
    (*ret_val) = this->data_.at<unsigned char>(y,x);

    return;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, PixelRGB *ret_val) const{

    // Set ret pointer to correct value
    cv::Vec3b temp_pxl_cv;
    temp_pxl_cv = this->data_.at<cv::Vec3b>(y,x);
    ret_val->r = temp_pxl_cv.val[2];
    ret_val->g = temp_pxl_cv.val[1];
    ret_val->b = temp_pxl_cv.val[0];

    return;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!* @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, int *ret_val) const{

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<int>(y,x);

    return;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!* @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, float *ret_val) const{

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<float>(y,x);

    return;
}

/**
 * @brief Retrieves pixel value from specific coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[out]  ret_val     Pointer for return value
 */
void Image::Unsafe_GetPixel(const unsigned int &x, const unsigned int &y, double *ret_val) const{

    // Set ret pointer to correct value
    (*ret_val) = this->data_.at<double>(y,x);

    return;
}


/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!* @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, char arg_val){

    // Store the pixel value
    this->data_.at<char>(y,x) = arg_val;

    return;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, unsigned char arg_val){

    // Store the pixel value
    this->data_.at<unsigned char>(y,x) = arg_val;

    return;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, PixelRGB arg_val){

    // Convert format
    cv::Vec3b temp_pxl_cv;
    temp_pxl_cv.val[2] = arg_val.r;
    temp_pxl_cv.val[1] = arg_val.g;
    temp_pxl_cv.val[0] = arg_val.b;

    // Store the pixel value
    this->data_.at<cv::Vec3b>(y,x) = temp_pxl_cv;

    return;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, int arg_val){

    // Store the pixel value
    this->data_.at<int>(y,x) = arg_val;

    return;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, float arg_val){

    // Store the pixel value
    this->data_.at<float>(y,x) = arg_val;

    return;
}

/**
 * @brief Sets the pixel value at the specified coordinate
 * @warning NO error checking is performed and attempting to access an empty image or out of range pixel will crash the program!
 * @param[in]   x           Column coordinate of desired pixel
 * @param[in]   y           Row coordinate of desired pixel
 * @param[in]   arg_val     Value to store in pixel
 */
void Image::Unsafe_SetPixel( const unsigned int &x, const unsigned int &y, double arg_val){

    // Store the pixel value
    this->data_.at<double>(y,x) = arg_val;

    return;
}

/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( unsigned char arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val));

    return ret;
}

/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( char arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_CHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val));

    return ret;
}


/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( int arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_INT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val));

    return ret;
}

/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( float arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_FLOAT )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val));

    return ret;
}

/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( double arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::MONO_DOUBLE )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val));

    return ret;
}


/**
 * @brief Sets all pixels to specified value
 * @param[in]   arg_val     Value to store in image pixels
 * @retval      IMAGE_EMPTY                         Image has NOT been created
 * @retval      IMAGE_STORED_IN_DIFFERENT_FORMAT    Attempting to fill image with value in format other than the object's set \ref dlp::Image::Format
 */
ReturnCode Image::FillImage( PixelRGB arg_val){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    // Check the format
    if( this->format_ != Format::RGB_UCHAR )
        return ret.AddError(IMAGE_STORED_IN_DIFFERENT_FORMAT);

    // Fill image to specified value
    this->data_.setTo(cv::Scalar(arg_val.b,arg_val.g,arg_val.r));

    return ret;
}

/**
 * @brief   Converts an RGB_UCHAR \ref dlp::Image to MONO_UCHAR format
 * \note    If the image is NOT an RGB_UCHAR format, no change to the data format is made
 * @retval  IMAGE_EMPTY                 Image has NOT been created
 * @retval  IMAGE_ALREADY_MONOCHROME    The image is already in a monochrome format
 */
ReturnCode Image::ConvertToMonochrome(){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    cv::Mat temp;

    // Check if it is RGB
    if(this->format_ == dlp::Image::Format::RGB_UCHAR){
        // Convert the color imge to grayscale
        cv::cvtColor(this->data_, temp, cv::COLOR_RGB2GRAY);

        // Clear the current image data
        this->data_.release();

        // Save the monochrome data
        this->data_ = temp;
        temp.release();

        this->format_ = Format::MONO_UCHAR;
    }
    else{
        ret.AddWarning(IMAGE_ALREADY_MONOCHROME);
    }

    return ret;
}

/**
 * @brief   Converts an monochrome \ref dlp::Image to RGB_UCHAR format
 * \note    If the image is NOT a monochrome format, no change to the data format is made
 * @retval  IMAGE_EMPTY                 Image has NOT been created
 * @retval  IMAGE_ALREADY_RGB           The image is already in the RGB format
 */
ReturnCode Image::ConvertToRGB(){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    cv::Mat temp;

    // Check if it is RGB
    if(this->format_ == dlp::Image::Format::RGB_UCHAR){
        ret.AddWarning(IMAGE_ALREADY_RGB);
    }
    else{
        // Convert the color imge to grayscale
        cv::cvtColor(this->data_, temp, cv::COLOR_GRAY2RGB);

        // Clear the current image data
        this->data_.release();

        // Save the monochrome data
        this->data_ = temp;
        temp.release();

        this->format_ = Format::RGB_UCHAR;
    }

    return ret;
}


/**
 * @brief   Flips the \ref dlp::Image across one or both axis
 * @param[in] flip_x    Flip the image across the x axis (vertical flip)
 * @param[in] flip_y    Flip the image across the y axis (horizontal flip)
 * @retval  IMAGE_EMPTY                 Image has NOT been created
 */
ReturnCode Image::FlipImage( const bool &flip_x, const bool &flip_y){
    ReturnCode ret;

    // Check that image has data
    if(this->isEmpty())
        return ret.AddError(IMAGE_EMPTY);

    if(flip_x & !flip_y){
        // Only flip around x axis
        cv::flip(this->data_,this->data_,0);
    }
    else if(flip_x & flip_y){
        // Flip around both axis
        cv::flip(this->data_,this->data_,-1);
    }
    else if(!flip_x & flip_y){
        // Only flip around y axis
        cv::flip(this->data_,this->data_,1);
    }

    return ret;
}


namespace Number{
template <> std::string ToString<dlp::PixelRGB>( dlp::PixelRGB pixel ){
    // Ignore the base and assume base 10
    return dlp::Number::ToString(pixel.r) + ", " +
           dlp::Number::ToString(pixel.g) + ", " +
           dlp::Number::ToString(pixel.b);
}

template <> std::string ToString<dlp::Image::Format>( dlp::Image::Format format){
    switch (format) {
    case dlp::Image::Format::MONO_UCHAR:    return "MONO_UCHAR";
    case dlp::Image::Format::MONO_CHAR:     return "MONO_CHAR";
    case dlp::Image::Format::MONO_INT:      return "MONO_INT";
    case dlp::Image::Format::MONO_FLOAT:    return "MONO_FLOAT";
    case dlp::Image::Format::MONO_DOUBLE:   return "MONO_DOUBLE";
    case dlp::Image::Format::RGB_UCHAR:     return "RGB_UCHAR";
    case dlp::Image::Format::INVALID:       return "INVALID";
    default:                                return "INVALID";
    }
    return "INVALID";
}

}

namespace String{
template <> dlp::PixelRGB ToNumber(const std::string &text , unsigned int base){
    // Ignore the base

    dlp::PixelRGB ret;

    std::vector<std::string> colors = dlp::String::SeparateDelimited(text,',');

    ret.r = dlp::String::ToNumber<unsigned char>(colors.at(0));
    ret.g = dlp::String::ToNumber<unsigned char>(colors.at(1));
    ret.b = dlp::String::ToNumber<unsigned char>(colors.at(2));

    return ret;
}
}

}

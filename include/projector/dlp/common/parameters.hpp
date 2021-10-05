/** @file       parameters.hpp
 *  @ingroup    Common
 *  @brief      Contains \ref dlp::Parameters and \ref dlp::Parameters::Entry classes
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_PARAMETERS_HPP
#define DLP_SDK_PARAMETERS_HPP

#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/other.hpp>

#include <iostream>
#include <string>
#include <vector>

#define PARAMETERS_EMPTY                        "PARAMETERS_EMPTY"
#define PARAMETERS_SOURCE_EMPTY                 "PARAMETERS_SOURCE_EMPTY"
#define PARAMETERS_DESTINATION_MISSING_ENTRY    "PARAMETERS_DESTINATION_MISSING_ENTRY"
#define PARAMETERS_NO_NAME                      "PARAMETERS_NO_NAME"
#define PARAMETERS_NO_NAME_SUPPLIED             "PARAMETERS_NO_NAME_SUPPLIED"
#define PARAMETERS_NOT_FOUND                    "PARAMETERS_NOT_FOUND"
#define PARAMETERS_INDEX_OUT_OF_RANGE           "PARAMETERS_INDEX_OUT_OF_RANGE"
#define PARAMETERS_ILLEGAL_CHARACTER            "PARAMETERS_ILLEGAL_CHARACTER"
#define PARAMETERS_MISSING_VALUE                "PARAMETERS_MISSING_VALUE"
#define PARAMETERS_NULL_POINTER                 "PARAMETERS_NULL_POINTER"
#define PARAMETERS_FILE_DOES_NOT_EXIST          "PARAMETERS_FILE_DOES_NOT_EXIST"
#define PARAMETERS_FILE_OPEN_FAILED             "PARAMETERS_FILE_OPEN_FAILED"
#define PARAMETERS_FILE_PROCESSING_FAILED       "PARAMETERS_FILE_PROCESSING_FAILED"

#define DLP_NEW_PARAMETERS_ENTRY(name,string,type,default_value) \
    class name: public dlp::Parameters::Entry<type>{  \
    public:\
    name(){\
    this->Setup(string,default_value);\
    }\
    name(type value){\
    this->Setup(string,default_value);\
    this->Set(value);\
    }\
    }

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @class      Parameters
 *  @brief      Container for transferring \ref dlp::Module setup information
 *  @ingroup    Common
 *  \note       All name and values are stored in std::string vectors
 */
class Parameters{
public:


    /** @class      Entry
    *  @brief      Class for automating dlp::Settings entry setup and retrieval
    *              storage and retrieval
    *  @ingroup    Common
    */
    template <class T>
    class Entry{
    public:
        void Set(const T &value){
            this->value_ = value;
        }

        T Get() const{
            return this->value_;
        }

        T GetDefault() const{
            return this->default_;
        }

        std::string GetEntryName() const{
            return this->name_;
        }

        std::string GetEntryValue() const{
            return dlp::Number::ToString(this->value_);
        }

        std::string GetEntryDefault() const{
            return dlp::Number::ToString(this->default_);
        }

        void SetEntryValue(const std::string &value){
            this->value_ = dlp::String::ToNumber<T>(value);
        }


        operator Parameters(){
            Parameters ret;

            ret.Clear();
            ret.Set(this->GetEntryName(),this->GetEntryValue());

            return ret;
        }



    protected:
        void Setup(std::string name,T default_value){
            this->name_     = name;
            this->value_    = default_value;
            this->default_  = default_value;
        }

    private:
        std::string name_;
        T value_;
        T default_;
    };



    template <typename T>
    ReturnCode Set(const Entry<T> &option){
        return this->Set(option.GetEntryName(),option.GetEntryValue());
    }

    template <typename T>
    ReturnCode Get(Entry<T> *option) const{
        ReturnCode ret;
        std::string return_value;

        // Check that point is NOT null
        if(!option) return ret.AddError(PARAMETERS_NULL_POINTER);

        ret = this->Get(option->GetEntryName(),option->GetEntryDefault(),&return_value);

        option->SetEntryValue(return_value);

        return ret;
    }

    template <typename T>
    bool Contains(const Entry<T> &option) const{
        return this->Contains(option.GetEntryName());
    }



    /** @brief      Creates or updates a dlp::Parameters entry
     *  \note       The method converts the name to upper case and removes all whitespace.
     *  @param[in]  name     Name of entry
     *  @param[in]  value    Numerical value which is converted to a string to save in entry
     */
    template <typename T>
    ReturnCode Set(const std::string &name, const T &value){
        return (this->Set(name,dlp::Number::ToString(value)));
    }

    /** @brief  Retrieves parameter entry value or stored default value if entry does NOT exist
     *  @param[in]  name            Name of entry
     *  @param[in]  default_value   Method returns this value if parameter entry does NOT exist
     *  @param[out] value           Numerical variable pointer to return value to
     *  @retval     PARAMETERS_NO_NAME      Supplied entry name string is empty
     *  @retval     PARAMETERS_NULL_POINTER Output argument is NULL
     *  @retval     PARAMETERS_NOT_FOUND    Parameter entry not found, default value loaded to pointer.
     */
    template <typename R>
    ReturnCode Get( const std::string &name, const R &default_value, R* value) const{
        ReturnCode ret;
        std::string value_str;
        std::string default_value_str;

        // Convert the default value to a string
        default_value_str = dlp::Number::ToString(default_value);

        // Get the parameter as its string value
        ret = this->Get(name,default_value_str,&value_str);

        // Convert the string value to the template type
        (*value) = String::ToNumber<R>(value_str);

        return ret;
    }

    // String Set, Get, and IsEqual methods
    ReturnCode Set(const std::string &name, const std::string &value);
    ReturnCode Get(const std::string &name, const std::string &default_value, std::string* value) const;

    ReturnCode GetName(const int &index,  std::string* ret_name)const;
    ReturnCode Remove(const std::string &name );

    bool Contains(const std::string &name)const;
    bool Contains(const std::string &name, int *ret_index)const;

    ReturnCode Save(const std::string &filename)const;

    ReturnCode Load(const std::string &filename);
    ReturnCode Load(const std::string &filename, const bool &update_current);

    ReturnCode Load(const  Parameters &source);
    ReturnCode Load(const  Parameters &source,   const bool &update_current);

    bool isEmpty() const;
    unsigned int GetCount()const;
    void Clear();

    std::string ToString();

    private:
        std::vector<std::string> names_;
        std::vector<std::string> values_;
};

}

#endif // DLP_SDK_PARAMETERS_HPP

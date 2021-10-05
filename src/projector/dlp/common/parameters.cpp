/** @file       parameters.cpp
 *  @brief      Contains methods for \ref dlp::Parameters
 *  @copyright 2014 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <projector/dlp/common/debug.hpp>
#include <projector/dlp/common/returncode.hpp>
#include <projector/dlp/common/other.hpp>
#include <projector/dlp/common/parameters.hpp>

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @brief      Adds or sets \ref dlp::Parameters::Entry object to list
 *  @param[in]  option  \ref dlp::Parameters::Entry object
 *  @retval     PARAMETERS_NO_NAME              Supplied entry name string is empty
 *  @retval     PARAMETERS_ILLEGAL_CHARACTER    Either the name and value, or both, contain an = character
 */
//ReturnCode Parameters::Set(const Entry &option){
//    return this->Set(option.GetEntryName(),option.GetEntryValue());
//}

///** @brief      Updates the \ref dlp::Parameters::Entry object with the value stored in the list
// *  @param[out] option  Pointer for \ref dlp::Parameters::Entry object return
// */
//ReturnCode Parameters::Get(Entry *option) const{
//    ReturnCode ret;
//    std::string return_value;

//    // Check that point is NOT null
//    if(!option) return ret.AddError(PARAMETERS_NULL_POINTER);

//    ret = this->Get(option->GetEntryName(),option->GetEntryDefault(),&return_value);

//    option->SetEntryValue(return_value);

//    return ret;
//}

///** @brief  Returns true if the list contains an entry matching the
// *          \ref dlp::Parameters::Entry object
// */
//bool Parameters::Contains(const Parameters::Entry &setting) const{
//    return this->Contains(setting.GetEntryName());
//}

/** @brief      Creates or updates a list entry
 *  @param[in]  name    Name of entry
 *  @param[in]  value   String value to save in entry
 *  \note       The method converts the name to upper case and removes all whitespace. The value is NOT modified in any way.
 *  @retval     PARAMETERS_NO_NAME              Supplied entry name string is empty
 *  @retval     PARAMETERS_ILLEGAL_CHARACTER    Either the name and value, or both, contain an = character
 */
ReturnCode Parameters::Set(const std::string &name, const std::string &value){
    ReturnCode ret;
    int  parameter_index = 0;

    // Check that the string is NOT empty
    if(name.empty())
        ret.AddError(PARAMETERS_NO_NAME);

    // Check that both the name an string do NOT have an '=' character
    if((name.find_first_of("=")  != std::string::npos) ||
       (value.find_first_of("=") != std::string::npos))
        ret.AddError(PARAMETERS_ILLEGAL_CHARACTER);

    if(ret.hasErrors()) return ret;

    // Does this parameter already exist in the vector?
    if(this->Contains( dlp::String::Trim(dlp::String::ToUpperCase(name)), &parameter_index)){
        // The name was correct, update the value
        this->values_.at(parameter_index) = dlp::String::Trim(value);
    }
    else{
        // The parameter was NOT found so it is new
        this->names_.push_back(  dlp::String::Trim(dlp::String::ToUpperCase(name)));
        this->values_.push_back( dlp::String::Trim(value));
    }

    return ret;
}

/** @brief      Creates or updates a list entry value
 *  @param[in]  name            List entry name
 *  @param[in]  default_value   Method returns this value if \ref dlp::Parameters entry does NOT exist
 *  @param[out] value           Pointer for return value
 *  @retval     PARAMETERS_NO_NAME      Supplied entry name string is empty
 *  @retval     PARAMETERS_NULL_POINTER Output argument is NULL
 *  @retval     PARAMETERS_NOT_FOUND    Parameter entry NOT found, default value loaded to pointer.
 */
ReturnCode Parameters::Get(const std::string &name, const std::string &default_value, std::string* value) const{
    ReturnCode ret;
    int  parameter_index = 0;

    // Check that the string is NOT empty
    if(name.empty() == true) ret.AddError(PARAMETERS_NO_NAME);

    // Check input argument pointer
    if(!value) ret.AddError(PARAMETERS_NULL_POINTER);

    // Check for errors
    if(ret.hasErrors()) return ret;


    // Does this parameter already exist in the vector?
    if(this->Contains(dlp::String::Trim(dlp::String::ToUpperCase(name)), &parameter_index)){
        // The name was correct, update the value
        (*value) = this->values_.at(parameter_index);
    }
    else{
        // The parameter was NOT found, return error code
        ret.AddError(PARAMETERS_NOT_FOUND);
        (*value) = default_value;
    }

    return ret;
}

/** @brief      Retrieves entry name from supplied index number
 *  @param[in]  index       Entry index
 *  @param[out] ret_name    Pointer for return value
 *  @retval     PARAMETERS_INDEX_OUT_OF_RANGE   Supplied entry name string is empty
 *  @retval     PARAMETERS_NULL_POINTER         Output argument is NULL
 */
ReturnCode Parameters::GetName(  const int &index,  std::string* ret_name)const{
    ReturnCode ret;

    // Check that poiter is NOT null
    if(!ret_name)
        return ret.AddError(PARAMETERS_NULL_POINTER);

    // Check that the index is in range
    if(((unsigned int)index < this->names_.size()) && (index >= 0)){
        (*ret_name) = this->names_.at(index);
    }
    else{
        //Index was out of range
        ret.AddError(PARAMETERS_INDEX_OUT_OF_RANGE);
    }

    return ret;
}

/** @brief      Removes entry from list
 *  @param[in]  name                    Entry to remove
 *  @retval     PARAMETERS_NO_NAME      Supplied entry name string is empty
 *  @retval     PARAMETERS_NOT_FOUND    Entry NOT found
 */
ReturnCode Parameters::Remove( const std::string &name ){
    ReturnCode ret;

    // Check that the string is NOT empty
    if(name.empty() == true)
        return ret.AddError(PARAMETERS_NO_NAME);


    int  parameter_index = 0;

    // Does this parameter already exist in the vector?
    if(this->Contains(dlp::String::Trim(dlp::String::ToUpperCase(name)), &parameter_index)){
        // The name was correct, remove the parameter
        this->names_.erase(  this->names_.begin()  + parameter_index);
        this->values_.erase( this->values_.begin() + parameter_index);
    }
    else{
        // The parameter was NOT found, return error code
        ret.AddError(PARAMETERS_NOT_FOUND);
    }

    return ret;
}

/** @brief Returns true if the list contains an entry matching the supplied name */
bool Parameters::Contains(const std::string &name) const{
    int  temp_int;
    return (this->Contains(name, &temp_int));
}

/** @brief Returns true if the list contains entry matching the supplied name
 *  @param[in]  name        Entry to search for
 *  @param[out] ret_index   Pointer to return index value
 */
bool Parameters::Contains(const std::string &name, int *ret_index) const{
    bool ret = false;
    int  parameter_count = this->names_.size();

    // Does this parameter already exist in the vector?
    for( int kParameter = 0; kParameter < parameter_count; kParameter++){
        // Is this the correct parameter?
        if(this->names_.at(kParameter).compare(dlp::String::Trim(dlp::String::ToUpperCase(name))) == 0){
            ret          = true;
            if(ret_index) (*ret_index) = kParameter;
        }
    }
    return ret;
}

/** @brief      Saves the list to a text file
 *  @param[in]  filename    Output file name
 *  @retval     PARAMETERS_EMPTY            List is empty, no file saved or created.
 *  @retval     PARAMETERS_FILE_OPEN_FAILED Failed to save file.
 */
ReturnCode Parameters::Save(const std::string &filename) const{
    ReturnCode ret;

    // Check that Parameters object has entries
    if(this->isEmpty()) return ret.AddError(PARAMETERS_EMPTY);


    int param_count = this->GetCount();
    std::ofstream param_file;

    // Open the file
    param_file.open(filename.c_str(),std::ios::out);


    // Check that file opened
    if(!param_file.is_open()) return ret.AddError(PARAMETERS_FILE_OPEN_FAILED);


    for(int kParam = 0; kParam < param_count; kParam++){
        // Add information to file
        param_file << this->names_.at(kParam) << "\t=\t" << this->values_.at(kParam) << std::endl;
    }

    // Close the file
    param_file.close();

    return ret;
}

/** @brief  Loads all entries from file into list
 *  \note   Any preexising entries in list will be updated with values stored in file
 *  @param[in] filename Input file name
 *  @retval PARAMETERS_FILE_DOES_NOT_EXIST  Supplied file does NOT exist.
 *  @retval PARAMETERS_FILE_OPEN_FAILED     Failed to load file.
 *  @retval PARAMETERS_MISSING_VALUE        One or more entries in file did NOT contain a value and were NOT added to the list (only a warning)
 */
ReturnCode Parameters::Load(const std::string &filename){
    return (this->Load(filename,true));
}

/** @brief  Loads all entries from file into list
 *  @param[in] filename         Input file name
 *  @param[in] update_current   If true, any preexising entries in list will be updated with values stored in file.
 *  @retval PARAMETERS_FILE_DOES_NOT_EXIST  Supplied file does NOT exist.
 *  @retval PARAMETERS_FILE_OPEN_FAILED     Failed to load file.
 *  @retval PARAMETERS_MISSING_VALUE        One or more entries in file did NOT contain a value and were NOT added to the list (only a warning)
 */
ReturnCode Parameters::Load(const std::string &filename, const bool &update_current){
    ReturnCode ret;

    // Check that file exists
    if(!dlp::File::Exists(filename))
        return ret.AddError(PARAMETERS_FILE_DOES_NOT_EXIST);


    // Open the file to read
    std::string     line;
	std::string		tempStr;
    std::string     param_name;
    std::string     param_value;
    std::ifstream   param_file(filename.c_str());

    // Check that the file opened
    if(!param_file.is_open())
        return ret.AddError(PARAMETERS_FILE_OPEN_FAILED);

	while ( true )
    {
		if (!std::getline (param_file,line,'\n')) {
			break;
		}
		
		if(!line.empty()) {
		
		tempStr = dlp::String::Trim(line);
				   
		// Check for comment character
		if(!tempStr.empty() && (tempStr.at(0) != '#')){

				// Find the = delimiter
				std::size_t delimiter = line.find_first_of("=");

				// Check that the delimiter exists
				if(delimiter != std::string::npos){

					// Copy the name
					param_name = dlp::String::ToUpperCase(dlp::String::Trim(line.substr(0,delimiter-1)));

					// Copy the value
					param_value = dlp::String::Trim(line.substr(delimiter+1));

					// Check that name and value are NOT empty
					if((!param_name.empty())&&(!param_value.empty())){

						// Should values be updated if already present?
						if(update_current || !(this->Contains(param_name))){
							// Add the parameter
							this->Set(param_name,param_value);
						}
					}
					else{
						// Parameter is missing value
						std::string error_string = PARAMETERS_MISSING_VALUE;
						error_string += ": ";
						error_string += param_name;
						ret.AddWarning(error_string);
					}
				}
				else{
					// Parameter is missing value
					std::string error_string = PARAMETERS_MISSING_VALUE;
					error_string += ": ";
					error_string += param_name;
					ret.AddWarning(error_string);
				}
			}
		}
    }
    
    if(param_file.bad())
		return ret.AddError(PARAMETERS_FILE_PROCESSING_FAILED);
	
	param_file.close();
	
	return ret;
}

/** @brief  Loads all entries from source into list
 *  \note   Any preexising entries in list will be updated with values stored in source list.
 *  @param[in] source                   Input source list
 *  @retval PARAMETERS_SOURCE_EMPTY     Supplied list has NO entries
 */
ReturnCode Parameters::Load(const Parameters &source){
    return this->Load(source,true);
}


/** @brief  Loads all entries from source into list
 *  @param[in] source                   Input source list
 *  @param[in] update_current           If true, any preexising entries in list will be updated with values stored in source list.
 *  @retval PARAMETERS_SOURCE_EMPTY     Supplied list has NO entries
 */
ReturnCode Parameters::Load(const Parameters &source, const bool &update_current){
    ReturnCode ret;

    // Check that both lists are NOT empty
    int source_count    = source.GetCount();

    if(source_count == 0) ret.AddError(PARAMETERS_SOURCE_EMPTY);

    // Return if any errors have occurred
    if(ret.hasErrors()) return ret;

    std::string param_name;
    std::string param_value;

    for(int kParam = 0; kParam < source_count; kParam++){
        // Get the name of parameter to be updated
        source.GetName(kParam, &param_name);

        // Get the value of the parameter
        source.Get(param_name, "default", &param_value);

        // Should values be updated if already present?
        if(update_current || !(this->Contains(param_name))){
            // Add the parameter
            this->Set(param_name,param_value);
        }
    }

    return ret;
}

/** @brief Returns true if list is empty */
bool Parameters::isEmpty() const{
    if(this->names_.size() == 0) return true;
    else                         return false;
}

/** @brief Returns the number of entries in list */
unsigned int Parameters::GetCount() const{
    return (this->names_.size());
}

/** @brief Clears list of all entries */
void Parameters::Clear(){
    this->names_.clear();
    this->values_.clear();
    return;
}

std::string Parameters::ToString(){
    std::string ret;

    ret = "PARAMETER COUNT = " + dlp::Number::ToString(this->GetCount());

    if(this->GetCount() > 0){
        for(unsigned int iParams = 0; iParams < this->GetCount(); iParams++){
            ret += "\n" + this->names_.at(iParams) + " \t = " + this->values_.at(iParams);
        }
    }

    return ret;
}

}

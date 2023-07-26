/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : main
 * @created     : Tuesday Jan 31, 2023 16:41:58 CET
 */
#include <regex>
#include <iostream>
#include <string>
#include <type_traits>
#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <chrono>

const char* home = getenv("HOME");

inline bool exists_test (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}

using time_point = std::chrono::system_clock::time_point;
std::string serializeTimePoint( const time_point& time, const std::string& format)
{
    std::time_t tt = std::chrono::system_clock::to_time_t(time);
    std::tm tm = *std::gmtime(&tt); //GMT (UTC)
    //std::tm tm = *std::localtime(&tt); //Locale time-zone, usually UTC by default.
    std::stringstream ss;
    ss << std::put_time( &tm, format.c_str() );
    return ss.str();
}


std::string resolve_dir(std::string directory_name)
{
	//create directory if it does not exist
	//std::string directory_name = "~/Data";
	//std::string filename_prefix = "file";
	std::string complete_file_name = "";
	//resolve tilde if any:
	if (home)
	{
		directory_name = std::regex_replace(directory_name, std::regex("\\~"), home);
	}
	if (!exists_test(directory_name))
	{
		//create directory
		if (boost::filesystem::create_directories(directory_name)) {
			std::cout << "Directory [" << directory_name<<"] created" << std::endl;
			return directory_name;
		}
		else {
			std::cerr << "Error :  " << std::strerror(errno) << std::endl;
			return "";
		}
	}
	else
		return directory_name;

}
std::string resolve_file_time(std::string directory_name, std::string filename_prefix)
{

    time_point input = std::chrono::system_clock::now();
	std::string time_str = serializeTimePoint(input, "%Y-%m-%d-%H-%M-%S");
	std::string complete_file_name = directory_name + "/"+ time_str + filename_prefix;
	return complete_file_name;

}

std::string resolve_file_old(std::string directory_name, std::string filename_prefix)
{
	int counter = 0;
	std::string complete_file_name = "";
	while(true)
	{
		complete_file_name = directory_name + "/" +std::to_string(counter)+ "_"+ filename_prefix;
		if (exists_test(complete_file_name)) //file exists
		{
			counter++;
		}
		else
		{
			//save file
			//std::ofstream output(complete_file_name);
			//exit loop
			break;
		}
	}
	//std::cout << "saved successfully" << std::endl;
	return complete_file_name;


}



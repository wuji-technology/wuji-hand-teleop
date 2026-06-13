// Platform-specific file and pipe handling definitions
#ifndef OSFILEID_H
#define OSFILEID_H

#ifdef _WIN32
#include <Windows.h>
#define OS_File_ID HANDLE
#elif __linux__
#define OS_File_ID int
#endif
#include <string>
inline std::string GetVideoPipeName(const char* deviceSn)
{
#ifdef _WIN32
    return std::string(R"(\\.\pipe\)")+deviceSn+"_videopipe";
#elif __linux__
    return std::string(deviceSn)+ "_videopipe";
#endif
}

#endif // OSFILEID_H

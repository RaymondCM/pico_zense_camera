#ifndef PICOZENSE_DEFINE_H
#define PICOZENSE_DEFINE_H

#include "PicoZense_enums.h"
#include "PicoZense_types.h"

#ifdef PS_EXPORT_ON
    #ifdef _WIN32
        #define PICOZENSE_API_EXPORT __declspec(dllexport)
    #else
        #define PICOZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#else
    #ifdef _WIN32
        #define PICOZENSE_API_EXPORT __declspec(dllimport)
    #else
        #define PICOZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
#define PICOZENSE_C_API_EXPORT extern "C" PICOZENSE_API_EXPORT
#else
#define PICOZENSE_C_API_EXPORT PICOZENSE_API_EXPORT
#define bool uint8_t
#endif

#endif /* PICOZENSE_DEFINE_H */

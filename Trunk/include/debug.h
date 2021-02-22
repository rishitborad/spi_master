//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
// FILE:  debug.h
//
// Copyright 2013 by MEMSIC, Inc., all rights reserved.
//
// This is a compile-time prioritized debug print driver. It lets you
// print out strings (and numbers) in a way that avoids memory hogging
// printf. It does this by avoiding variable arguments and only printing
// variables at the end of the string:
//    DEBUG_STRING("This takes a string");
//    DEBUG_INT("This takes a string and prints an integer at the end: ", integer_variable);
//    DEBUG_HEX("Prints an integer in hex at the end: ", integer_variable);
//    DEBUG_FLOAT("Prints a float at the end: ", float_variable, significant_digits);
//    DEBUG_TIMESTAMP("Prints a 64 but integer: ", timestamp_variable);
//    DEBUG_ENDLINE(); // outputs "\r\n"
//
// It also lets you leave debug print statements for future debugging
// without incurring a code-space cost. Each logging function starts with
// INFO_, DEBUG_, or ERROR_ prefix. By default, for non-production builds, 
// all DEBUG_* (e.g., DEBUG_STRING)  send their data to the debug uart port. 
// Also all ERROR_* logs are sent to the debug port (that may remain true
// for production builds if we have a debug uart).
//
// If you want to see the INFO_* logs, you'll have to set the logging level
// in your file at compile time, before including this file.
//     #define LOGGING_LEVEL LEVEL_INFO
//     #include "debug.h"
// And if you are getting too much data, you can also set your logging level
// higher to quiet a module:
//     #define LOGGING_LEVEL LEVEL_ERROR
//     #include "debug.h"
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef DEBUG_H
#define DEBUG_H

extern void InitSerialCommunication(void);

#define LEVEL_NONE 0
#define LEVEL_ERROR 1
#define LEVEL_DEBUG 2
#define LEVEL_INFO 3

#ifndef LOGGING_LEVEL
#define LOGGING_LEVEL LEVEL_DEBUG
#endif

#if LOGGING_LEVEL >= LEVEL_DEBUG
#define DEBUG_STRING(s)       DebugPrintString((s));
#define DEBUG_INT(s, i)       DebugPrintInt((s),(i));
#define DEBUG_HEX(s, i)       DebugPrintHex((s),(i));
#define DEBUG_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define DEBUG_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define DEBUG_ENDLINE()       DebugPrintEndline();
#else
#define DEBUG_STRING(s)      
#define DEBUG_INT(s, i)      
#define DEBUG_HEX(s, i)   
#define DEBUG_TIMESTAMP(s, t) 
#define DEBUG_FLOAT(s, i, d) 
#define DEBUG_ENDLINE()      
#endif

#if LOGGING_LEVEL >= LEVEL_INFO
#define INFO_STRING(s)       DebugPrintString((s));
#define INFO_INT(s, i)       DebugPrintInt((s),(i));
#define INFO_HEX(s, i)       DebugPrintHex((s),(i));
#define INFO_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define INFO_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define INFO_ENDLINE()       DebugPrintEndline();
#else
#define INFO_STRING(s)      
#define INFO_INT(s, i)      
#define INFO_HEX(s, i)  
#define INFO_TIMESTAMP(s, t)
#define INFO_FLOAT(s, i, d) 
#define INFO_ENDLINE()      
#endif

#if LOGGING_LEVEL >= LEVEL_ERROR
#define ERROR_STRING(s)       DebugPrintString((s));
#define ERROR_INT(s, i)       DebugPrintInt((s),(i));
#define ERROR_HEX(s, i)       DebugPrintHex((s),(i));
#define ERROR_TIMESTAMP(s, t) DebugPrintHex((s),(t >> 32)); DebugPrintHex(NULL, (t)&0xFFFFFFFF);
#define ERROR_FLOAT(s, i, d)  DebugPrintFloat((s),(i), (d));
#define ERROR_ENDLINE()       DebugPrintEndline();
#else
#define ERROR_STRING(s)      
#define ERROR_INT(s, i)      
#define ERROR_HEX(s, i)   
#define ERROR_TIMESTAMP(s, t) 
#define ERROR_FLOAT(s, i, d) 
#define ERROR_ENDLINE()      
#endif

extern void DebugPrintString(const char * str);
extern void DebugPrintInt(const char *str, int i);
extern void DebugPrintHex(const char *str, int i);
extern void DebugPrintFloat(const char *str, float f, int sigDigits);
extern void DebugPrintEndline();

#endif /* DEBUG_H */
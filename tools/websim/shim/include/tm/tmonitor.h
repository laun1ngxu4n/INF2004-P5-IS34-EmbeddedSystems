/* websim shim for <tm/tmonitor.h>: console output goes to the web page */
#ifndef __TM_TMONITOR_H__
#define __TM_TMONITOR_H__

#include <tk/typedef.h>

#ifndef SIM_IMPORT
#define SIM_IMPORT(name) __attribute__((import_module("env"), import_name(name)))
#endif

INT tm_printf(const UB *format, ...);
INT tm_sprintf(UB *str, const UB *format, ...);
INT tm_putstring(const UB *str);

#endif

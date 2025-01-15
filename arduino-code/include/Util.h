/**
 * Simple Utility made for repetetive generic methods used within my project
 * 
 * Currently implements:
 * - Timeout implementation, used in multiple areas and can lead to being repetetive, allows to avoid repeated functions
 * 
 */
#ifndef UTIL_H
#define UTIL_H

#include <mbed.h>
#include <rtos.h>

namespace Util {
    void onTimeout(bool &timeoutOccurred);
    void beginTimeout(mbed::Timeout &timeout, bool &timeoutOccurred, float time);
}

#endif

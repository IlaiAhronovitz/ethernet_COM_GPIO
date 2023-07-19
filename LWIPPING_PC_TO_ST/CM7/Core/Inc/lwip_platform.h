/*
 * lwip_platform.h
 *
 *  Created on: Jun 27, 2023
 *      Author: ghkh1
 */

#ifndef LWIP_PLATFORM_H
#define LWIP_PLATFORM_H

#include <stdio.h>  // Include the necessary header for your diagnostic output function

// Platform-specific diagnostic output function
#define LWIP_PLATFORM_DIAG(message)   printf("%s", message)

#endif /* LWIP_PLATFORM_H */

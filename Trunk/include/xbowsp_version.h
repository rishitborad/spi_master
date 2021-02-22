/*****************************************************************************
* File name:  xbowsp_version.h
*
* File description: 
*   - Version definition based on UCB serial protocol.
*
* $Rev: 17514 $
* $Date: 2011-03-02 13:36:09 -0800 (Wed, 02 Mar 2011) $
* $Author: by-tdelong $
*                                        
***********************************************************************************/
#ifndef VERSION_H
#define VERSION_H


/** @fn xbowsp_version.h
 *	@brief	contains versioning data
 *
 * Version Data = Major, Minor, Patch, Stage and Build#
 * 	- MajorVersion changes may introduce serious incompatibilities.  
 *  - MinorVersion changes may add or modify functionality, 
 *    but maintain backward compatibility with previous minor versions.  
 *  - Patch level changes reflect bug fixes and internal modifications with 
 *    little effect on the user.  
 *  - The build stage is one of the following: 
 *		0=release candidate
 *		1=development
 *		2=alpha
 *		3=beta.  
 *	- The buildNumber is incremented with each engineering firmware build.  
 *
 * The buildNumber and stage for released firmware are both zero.  
 * The final beta candidate is v.w.x.3.y, which is then changed to v.w.x.0.1 to create the 
 * first release candidate.  The last release candidate is v.w.x.0.z, which is then 
 * changed to v.w.x.0.0 for release.
 *
 *  @author David Ammerlaan
 *
 *
 * 	@version 10.23.2006	DRA	initial creation.
 *	@n		 04.20.2007	DRA	ECO release
 *
 */

#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 0
#define VERSION_STAGE 0
#define VERSION_BUILD 10

 /* software version/part number */
#define SOFTWARE_PART "5025-0675-01_A"
#define SOFTWARE_PART_LEN	14
#define VERSION_STR   SOFTWARE_PART   /* using the version defined in the protocol version, xbowsp_version.h  */
#define N_VERSION_STR 128

/* ===========================================================================
 No more.
=========================================================================== */


#endif
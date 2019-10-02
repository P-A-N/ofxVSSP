#ifndef URG3D_DETECT_OS_H
#define URG3D_DETECT_OS_H

/*!
  \file
  \brief header to detect OS type

  \author Satofumi KAMIMURA

  $Id: urg3d_detect_os.h,v 6c17b740611a 2014/03/24 09:58:01 jun $
*/

#if defined(_WIN32)
#define URG3D_WINDOWS_OS

//#ifndef WIN32_LEAN_AND_MEAN
//#define WIN32_LEAN_AND_MEAN 1
//#endif

#ifndef M_PI
//! pi for Visual C++ 6.0
#define M_PI 3.14159265358979323846
#endif

#if defined(_MSC_VER) || defined(__BORLANDC__)
#define URG3D_MSC
#endif

#elif defined(__linux__)
#define URG3D_LINUX_OS

#else
//! if os type is not detected by above defines, set as Mac OS
#define URG3D_MAC_OS
#endif

#endif /* !URG3D_DETECT_OS_H */

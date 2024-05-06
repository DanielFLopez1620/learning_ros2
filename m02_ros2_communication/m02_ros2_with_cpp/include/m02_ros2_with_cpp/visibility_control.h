#ifndef M02_ROS2_WITH_CPP__VISIBILITY_CONTROL_H_
#define M02_ROS2_WITH_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define M02_ROS2_WITH_CPP_EXPORT __attribute__ ((dllexport))
    #define M02_ROS2_WITH_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define M02_ROS2_WITH_CPP_EXPORT __declspec(dllexport)
    #define M02_ROS2_WITH_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef M02_ROS2_WITH_CPP_BUILDING_DLL
    #define M02_ROS2_WITH_CPP_PUBLIC M02_ROS2_WITH_CPP_EXPORT
  #else
    #define M02_ROS2_WITH_CPP_PUBLIC M02_ROS2_WITH_CPP_IMPORT
  #endif
  #define M02_ROS2_WITH_CPP_PUBLIC_TYPE M02_ROS2_WITH_CPP_PUBLIC
  #define M02_ROS2_WITH_CPP_LOCAL
#else
  #define M02_ROS2_WITH_CPP_EXPORT __attribute__ ((visibility("default")))
  #define M02_ROS2_WITH_CPP_IMPORT
  #if __GNUC__ >= 4
    #define M02_ROS2_WITH_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define M02_ROS2_WITH_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define M02_ROS2_WITH_CPP_PUBLIC
    #define M02_ROS2_WITH_CPP_LOCAL
  #endif
  #define M02_ROS2_WITH_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // M02_ROS2_WITH_CPP__VISIBILITY_CONTROL_H_
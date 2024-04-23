#ifndef M02_PLUGINS_FIGURE__VISIBILITY_CONTROL_H_
#define M02_PLUGINS_FIGURE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define M02_PLUGINS_FIGURE_EXPORT __attribute__ ((dllexport))
    #define M02_PLUGINS_FIGURE_IMPORT __attribute__ ((dllimport))
  #else
    #define M02_PLUGINS_FIGURE_EXPORT __declspec(dllexport)
    #define M02_PLUGINS_FIGURE_IMPORT __declspec(dllimport)
  #endif
  #ifdef M02_PLUGINS_FIGURE_BUILDING_LIBRARY
    #define M02_PLUGINS_FIGURE_PUBLIC M02_PLUGINS_FIGURE_EXPORT
  #else
    #define M02_PLUGINS_FIGURE_PUBLIC M02_PLUGINS_FIGURE_IMPORT
  #endif
  #define M02_PLUGINS_FIGURE_PUBLIC_TYPE M02_PLUGINS_FIGURE_PUBLIC
  #define M02_PLUGINS_FIGURE_LOCAL
#else
  #define M02_PLUGINS_FIGURE_EXPORT __attribute__ ((visibility("default")))
  #define M02_PLUGINS_FIGURE_IMPORT
  #if __GNUC__ >= 4
    #define M02_PLUGINS_FIGURE_PUBLIC __attribute__ ((visibility("default")))
    #define M02_PLUGINS_FIGURE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define M02_PLUGINS_FIGURE_PUBLIC
    #define M02_PLUGINS_FIGURE_LOCAL
  #endif
  #define M02_PLUGINS_FIGURE_PUBLIC_TYPE
#endif

#endif  // M02_PLUGINS_FIGURE__VISIBILITY_CONTROL_H_

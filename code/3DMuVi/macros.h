#ifndef MACROS_H
#define MACROS_H

#ifdef WIN32
#ifdef EXPORTING_3DMUVI
#define EXPORTED __declspec(dllexport)
#else
#define EXPORTED __declspec(dllimport)
#endif
#else
#define EXPORTED
#endif

#endif // MACROS_H


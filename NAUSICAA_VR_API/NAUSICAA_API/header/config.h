#pragma once
#ifdef NAUSICAA_VR_API_EXPORTS
#define NAUSICAA_VR_API __declspec(dllexport)
#else
#define NAUSICAA_VR_API __declspec(dllimport)
#endif


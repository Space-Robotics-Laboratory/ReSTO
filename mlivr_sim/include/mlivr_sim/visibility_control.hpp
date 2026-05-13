// Copyright (c) 2026 Masazumi Imai
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MLIVR_SIM__VISIBILITY_CONTROL_HPP_
#define MLIVR_SIM__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MLIVR_SIM_EXPORT __attribute__((dllexport))
#define MLIVR_SIM_IMPORT __attribute__((dllimport))
#else
#define MLIVR_SIM_EXPORT __declspec(dllexport)
#define MLIVR_SIM_IMPORT __declspec(dllimport)
#endif
#ifdef MLIVR_SIM_BUILDING_LIBRARY
#define MLIVR_SIM_PUBLIC MLIVR_SIM_EXPORT
#else
#define MLIVR_SIM_PUBLIC MLIVR_SIM_IMPORT
#endif
#define MLIVR_SIM_PUBLIC_TYPE MLIVR_SIM_PUBLIC
#define MLIVR_SIM_LOCAL
#else
#define MLIVR_SIM_EXPORT __attribute__((visibility("default")))
#define MLIVR_SIM_IMPORT
#if __GNUC__ >= 4
#define MLIVR_SIM_PUBLIC __attribute__((visibility("default")))
#define MLIVR_SIM_LOCAL __attribute__((visibility("hidden")))
#else
#define MLIVR_SIM_PUBLIC
#define MLIVR_SIM_LOCAL
#endif
#define MLIVR_SIM_PUBLIC_TYPE
#endif

#endif  // MLIVR_SIM__VISIBILITY_CONTROL_HPP_

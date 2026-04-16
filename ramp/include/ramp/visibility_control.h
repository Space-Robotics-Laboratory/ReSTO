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

#ifndef RAMP__VISIBILITY_CONTROL_HPP_
#define RAMP__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RAMP_EXPORT __attribute__((dllexport))
#define RAMP_IMPORT __attribute__((dllimport))
#else
#define RAMP_EXPORT __declspec(dllexport)
#define RAMP_IMPORT __declspec(dllimport)
#endif
#ifdef RAMP_BUILDING_LIBRARY
#define RAMP_PUBLIC RAMP_EXPORT
#else
#define RAMP_PUBLIC RAMP_IMPORT
#endif
#define RAMP_PUBLIC_TYPE RAMP_PUBLIC
#define RAMP_LOCAL
#else
#define RAMP_EXPORT __attribute__((visibility("default")))
#define RAMP_IMPORT
#if __GNUC__ >= 4
#define RAMP_PUBLIC __attribute__((visibility("default")))
#define RAMP_LOCAL __attribute__((visibility("hidden")))
#else
#define RAMP_PUBLIC
#define RAMP_LOCAL
#endif
#define RAMP_PUBLIC_TYPE
#endif

#endif  // RAMP__VISIBILITY_CONTROL_HPP_

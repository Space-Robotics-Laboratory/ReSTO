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

#ifndef MLIVR_MODEL__VISIBILITY_CONTROL_HPP_
#define MLIVR_MODEL__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MLIVR_MODEL_EXPORT __attribute__((dllexport))
#define MLIVR_MODEL_IMPORT __attribute__((dllimport))
#else
#define MLIVR_MODEL_EXPORT __declspec(dllexport)
#define MLIVR_MODEL_IMPORT __declspec(dllimport)
#endif
#ifdef MLIVR_MODEL_BUILDING_LIBRARY
#define MLIVR_MODEL_PUBLIC MLIVR_MODEL_EXPORT
#else
#define MLIVR_MODEL_PUBLIC MLIVR_MODEL_IMPORT
#endif
#define MLIVR_MODEL_PUBLIC_TYPE MLIVR_MODEL_PUBLIC
#define MLIVR_MODEL_LOCAL
#else
#define MLIVR_MODEL_EXPORT __attribute__((visibility("default")))
#define MLIVR_MODEL_IMPORT
#if __GNUC__ >= 4
#define MLIVR_MODEL_PUBLIC __attribute__((visibility("default")))
#define MLIVR_MODEL_LOCAL __attribute__((visibility("hidden")))
#else
#define MLIVR_MODEL_PUBLIC
#define MLIVR_MODEL_LOCAL
#endif
#define MLIVR_MODEL_PUBLIC_TYPE
#endif

#endif  // MLIVR_MODEL__VISIBILITY_CONTROL_HPP_

// Copyright 2020 The Ament Cargo Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use crate::ament_cargo_errors::AmentBuildError;

struct ExportCrates {
    relative_crates: HashSet<PathBuf>,
    absolute_crates: HashSet<PathBuf>,
}

impl ExportCrates {
    fn new() -> Self {
        ExportCrates {
            relative_crates: HashSet::new(),
            absolute_crates: HashSet::new(),
        }
    }

    fn add_crate(&mut self, crate_to_add: &PathBuf) {
        if crate_to_add.is_absolute() {
            self.absolute_crates.insert(crate_to_add.clone());
        } else {
            self.relative_crates.insert(crate_to_add.clone());
        }
    }
}

fn ament_register_extension(extension_point: &str, package_name: &str, filename: &Path) {
    // #
    // # Register a CMake filename to be included as part of an extension
    // # point.
    // #
    // # :param extension_point: the name of the extension point
    // # :type extension_point: string
    // # :param package_name: the name of the package containing the CMake
    // #   file
    // # :type package_name: string
    // # :param cmake_filename: the path to a CMake file relative to the
    // #   ${package_name}_DIR folder
    // # :type cmake_filename: string
    // #
    // # @public
    // #
    // macro(ament_register_extension extension_point package_name cmake_filename)
    //   list(APPEND AMENT_EXTENSIONS_${extension_point}
    //     "${package_name}:${cmake_filename}")
    // endmacro()
}

fn register_package_hook() {
    //     macro(_ament_cmake_export_crates_register_package_hook)
    //     if(NOT DEFINED
    //         _AMENT_CMAKE_EXPORT_CRATES_PACKAGE_HOOK_REGISTERED)
    //       set(_AMENT_CMAKE_EXPORT_CRATES_PACKAGE_HOOK_REGISTERED TRUE)

    //       find_package(ament_cmake_core QUIET REQUIRED)
    //       ament_register_extension("ament_package"
    //         "ament_cmake_export_crates"
    //         "ament_cmake_export_crates_package_hook.cmake")
    //     endif()
    //   endmacro()

    //   include(
    //     "${ament_cmake_export_crates_DIR}/ament_export_crates.cmake")
}

fn ament_export_crates(crate_paths: &Vec<PathBuf>) -> Result<ExportCrates, AmentBuildError> {
    //     if(_${PROJECT_NAME}_AMENT_PACKAGE)
    //       message(FATAL_ERROR
    //         "ament_export_crates() must be called before ament_package()")
    //     endif()

    //     if(${ARGC} GREATER 0)
    //       _ament_cmake_export_crates_register_package_hook()

    //       foreach(_arg ${ARGN})
    //         if(IS_ABSOLUTE "${_arg}")
    //           if(NOT EXISTS "${_arg}")
    //             message(WARNING
    //               "ament_export_crates() package '${PROJECT_NAME}' exports the "
    //               "crate '${_arg}' which doesn't exist")
    //           else()
    //               list_append_unique(_AMENT_EXPORT_ABSOLUTE_CRATES "${_arg}")
    //           endif()
    //         else()
    //           set(_arg "\${${PROJECT_NAME}_DIR}/../../../${_arg}")
    //           list_append_unique(_AMENT_EXPORT_RELATIVE_CRATES "${_arg}")
    //         endif()
    //       endforeach()
    //     endif()
    let mut crate_exports = ExportCrates::new();

    for crate_to_add in crate_paths {
        // Make sure the crate actually exists
        if !crate_to_add.exists() {
            match crate_to_add.to_str() {
                Some(x) => {
                    return Err(AmentBuildError::ExportNotFound {
                        bad_path: x.to_owned(),
                    })
                }
                None => return Err(AmentBuildError::UnstringifyableNotFound {}),
            };
        }

        // Add crate to export container
        crate_exports.add_crate(crate_to_add)
    }

    return Ok(crate_exports);
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

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

use std::env;
use std::path::{Path, PathBuf};
use std::unimplemented;

use crate::ament_cargo_errors::{missing_path_error, AmentBuildError};

fn check_crate_path_exists(path_to_check: &PathBuf) -> Result<&PathBuf, AmentBuildError> {
    match path_to_check.exists() {
        true => Ok(path_to_check),
        false => Err(missing_path_error(path_to_check)),
    }
}

pub fn ament_register_extension(extension_point: &str, package_name: &str, filename: &Path) {
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
    unimplemented!("ament_register_extension is unimplemented")
}

pub fn register_package_hook() {
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
    unimplemented!("register_package_hook is unimplemented")
}

pub fn ament_export_crates(crate_paths: &Vec<PathBuf>) -> Result<Vec<PathBuf>, AmentBuildError> {
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
    let ament_path: PathBuf = env::var("AMENT_PREFIX_PATH")
        .map_err(|source| AmentBuildError::AmentNotFound { source })?
        .into();

    let mut export_path_error = Ok(());
    let crate_exports: Vec<PathBuf> = crate_paths
        .iter()
        .scan(
            &mut export_path_error,
            |scan_errors, res| match check_crate_path_exists(res) {
                Ok(o) => Some(o),
                Err(e) => {
                    **scan_errors = Err(e);
                    None
                }
            },
        )
        .map(|x| match x.is_absolute() {
            true => x.clone(),
            false => ament_path.join(x.clone()),
        })
        .collect();
    export_path_error?; // If a path was missing, this will contain the error

    return Ok(crate_exports);
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

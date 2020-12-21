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

use std::{env::VarError, path::PathBuf};

use thiserror::Error;

#[allow(dead_code)]
#[derive(Error, Debug)]
pub enum InvalidErrorComparison {
    #[error("Not an AmentNotFound")]
    NotAmentNotFound,

    #[error("Not an ExportNotFound")]
    NotExportNotFound,

    #[error("Not an UnstringifyableNotFound")]
    NotUnstringifyableNotFound,

    #[error("Not an IOError")]
    NotIOError,

    #[error("Not an Unknown error")]
    NotUnknown,
}

#[allow(dead_code)]
#[derive(Error, Debug, PartialEq)]
pub enum AmentBuildError {
    /// Represents that the environment variable AMENT_PREFIX_PATH is missing.
    /// Most often caused by forgetting to source the ROS installation before building.
    #[error("environment variable \"AMENT_PREFIX_PATH\" not found! Did you forget to source ROS?")]
    AmentNotFound { source: VarError },

    /// Represents a crate-export-not-found error. I.e: Ament was told to build a crate, but the
    /// path is bad.
    #[error("Cannot find crate for export at {bad_path}!")]
    ExportNotFound { bad_path: String },

    // Placeholder for unknown Ament build errors
    #[error("Unknown Ament build error occurred!")]
    Unknown,
}

impl AmentBuildError {
    pub fn is_ament_not_found(&self) -> bool {
        match *self {
            Self::AmentNotFound { source: _ } => true,
            _ => false,
        }
    }

    pub fn is_export_not_found(&self) -> bool {
        match *self {
            Self::ExportNotFound { bad_path: _ } => true,
            _ => false,
        }
    }

    pub fn is_unknown(&self) -> bool {
        match *self {
            Self::Unknown => true,
            _ => false,
        }
    }
}

/// Create an ExportNotFound error if the path is stringifyable.
/// Otherwise, create UnstringifyableNotFound.
pub fn missing_path_error(bad_path: &PathBuf) -> AmentBuildError {
    let path_str = bad_path.as_os_str().to_string_lossy().into_owned();
    AmentBuildError::ExportNotFound { bad_path: path_str }
}

mod tests {
    use std::path::PathBuf;

    use super::{missing_path_error, AmentBuildError};

    #[test]
    fn test_missing_path_error() {
        let path_str = "rutabega";
        let path = PathBuf::from(path_str);

        let mpe = missing_path_error(&path);
        let expected = AmentBuildError::ExportNotFound {
            bad_path: path_str.to_owned(),
        };

        assert_eq!(mpe, expected)
    }
}

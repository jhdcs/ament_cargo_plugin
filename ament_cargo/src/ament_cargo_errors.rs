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

use thiserror::Error;

#[derive(Error, Debug)]
pub enum AmentBuildError {
    /// Represents that the environment variable AMENT_PREFIX_PATH is missing.
    /// Most often caused by forgetting to source the ROS installation before building.
    #[error("environment variable \"AMENT_PREFIX_PATH\" not found! Did you forget to source ROS?")]
    AmentNotFound { source: std::io::Error },

    /// Represents a crate-export-not-found error. I.e: Ament was told to build a crate, but the
    /// path is bad.
    #[error("Cannot find crate for export at {bad_path}!")]
    ExportNotFound { bad_path: String },

    /// Represents a crate-export-not-found error, but the bad path is unstringifyable. So it's
    /// up to the user to figure out what the path is, unfortunately...
    #[error("Cannot find crate for export - additionally, path cannot be stringified!")]
    UnstringifyableNotFound,

    /// Represents all other cases of std::io::Error.
    #[error(transparent)]
    IOError(#[from] std::io::Error),

    // Placeholder for unknown Ament build errors
    #[error("Unknown Ament build error occurred!")]
    Unknown,
}

use std::{
    fs::{self, File, OpenOptions},
    io,
    path::{Path, PathBuf},
};

use common::Anonymize;

// Currently, the "tracing" crate does not have an implementation for logging to a file that has all the
// properties we want: Creating a fresh file per run of the game. Because of this, we use a custom
// logging implementation instead.
pub struct Logfile {
    path: PathBuf,
    file: Option<File>,
    has_error: bool,
}

impl Logfile {
    pub fn new(path: impl AsRef<Path>) -> Self {
        Self {
            path: path.as_ref().to_path_buf(),
            file: None,
            has_error: false,
        }
    }
}

impl io::Write for Logfile {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let file = match self.file {
            Some(ref mut file) => file,
            None => loop {
                if self.has_error {
                    // If we are in the error state, don't keep attempting to create or rename a file.
                    return Ok(buf.len());
                }
                match OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(&self.path)
                {
                    Ok(file) => {
                        tracing::info!("Created log file: {}", self.path.anonymize().display());
                        break self.file.insert(file);
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::AlreadyExists => {
                        fs::rename(&self.path, self.path.with_added_extension("old")).inspect_err(
                            |e| {
                                self.has_error = true;
                                tracing::error!("Failed to rename old log file: {}", e)
                            },
                        )?;
                    }
                    Err(e) => {
                        return Err(e).inspect_err(|e| {
                            self.has_error = true;
                            tracing::error!("Failed to create new log file: {}", e);
                        });
                    }
                }
            },
        };
        file.write(buf).inspect_err(|e| {
            // If we already have the file open, we should keep attempting to write to it even
            // if some writes fail. However, if an error occurs, we should only report it once
            // to avoid spamming the logs (especially since using "tracing" to report errors in
            // "tracing" can result in a feedback loop if we are not careful).
            if !self.has_error {
                self.has_error = true;
                tracing::error!("Failed to write to log file: {}", e);
            }
        })
    }

    fn flush(&mut self) -> io::Result<()> {
        if let Some(ref mut file) = self.file {
            file.flush()?;
        }
        Ok(())
    }
}

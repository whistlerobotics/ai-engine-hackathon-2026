const IMAGE_SIZE: (usize, usize) = (640, 480);
pub const BLACK_JPEG: &[u8] = include_bytes!("../assets/black.jpg");

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    #[error("the camera frame is not a JPEG")]
    NotJpeg,
    #[error("the camera frame is {width}×{height}, not {}×{}", IMAGE_SIZE.0, IMAGE_SIZE.1)]
    WrongSize { width: usize, height: usize },
}

/// Whether `jpeg` is what `images.top` must be, judged from its header alone.
///
/// # Errors
///
/// How it falls short.
pub fn check_jpeg(jpeg: &[u8]) -> Result<(), FrameError> {
    if !matches!(imagesize::image_type(jpeg), Ok(imagesize::ImageType::Jpeg)) {
        return Err(FrameError::NotJpeg);
    }
    let size = imagesize::blob_size(jpeg).map_err(|_| FrameError::NotJpeg)?;
    if (size.width, size.height) != IMAGE_SIZE {
        return Err(FrameError::WrongSize {
            width: size.width,
            height: size.height,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    /// Just enough of a JPEG for its size to be read: start of image, then a baseline frame
    /// header for a 1280×800, three-component image.
    const JPEG_1280X800: &[u8] = &[
        0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x03, 0x20, 0x05, 0x00, 0x03, 0x01, 0x22, 0x00,
        0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xD9,
    ];
    const PNG_SIGNATURE: &[u8] = &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

    #[test_case(BLACK_JPEG => Ok(()); "valid frame")]
    #[test_case(PNG_SIGNATURE => Err(FrameError::NotJpeg); "not a JPEG")]
    #[test_case(JPEG_1280X800 => Err(FrameError::WrongSize { width: 1280, height: 800 }); "wrong size")]
    fn frame_checks(jpeg: &[u8]) -> Result<(), FrameError> {
        check_jpeg(jpeg)
    }
}

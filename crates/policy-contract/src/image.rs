const IMAGE_SIZE: (usize, usize) = (640, 480);
pub const BLACK_JPEG: &[u8] = include_bytes!("../assets/black.jpg");

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    #[error("the camera frame is not a JPEG")]
    NotJpeg,
    #[error("the camera frame is {width}×{height}, not {}×{}", IMAGE_SIZE.0, IMAGE_SIZE.1)]
    WrongSize { width: usize, height: usize },
    #[error("the camera frame is {width}×{height}, not {expected_width}×{expected_height}")]
    UnexpectedSize {
        width: usize,
        height: usize,
        expected_width: usize,
        expected_height: usize,
    },
}

/// Whether `jpeg` is what `images.top` must be, judged from its header alone.
///
/// # Errors
///
/// How it falls short.
pub fn check_jpeg(jpeg: &[u8]) -> Result<(), FrameError> {
    check_jpeg_size(jpeg, IMAGE_SIZE)
}

/// Checks a native camera frame against the explicitly configured input dimensions.
pub fn check_jpeg_size(jpeg: &[u8], expected: (usize, usize)) -> Result<(), FrameError> {
    if !matches!(imagesize::image_type(jpeg), Ok(imagesize::ImageType::Jpeg)) {
        return Err(FrameError::NotJpeg);
    }
    let size = imagesize::blob_size(jpeg).map_err(|_| FrameError::NotJpeg)?;
    if (size.width, size.height) != expected {
        return Err(if expected == IMAGE_SIZE {
            FrameError::WrongSize {
                width: size.width,
                height: size.height,
            }
        } else {
            FrameError::UnexpectedSize {
                width: size.width,
                height: size.height,
                expected_width: expected.0,
                expected_height: expected.1,
            }
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

    #[test_case((1280, 800) => Ok(()); "configured native frame")]
    #[test_case((640, 360) => Err(FrameError::UnexpectedSize { width: 1280, height: 800, expected_width: 640, expected_height: 360 }); "reject wrong native dimensions")]
    fn configured_size(expected: (usize, usize)) -> Result<(), FrameError> {
        check_jpeg_size(JPEG_1280X800, expected)
    }

    #[test_case(BLACK_JPEG => Ok(()); "valid frame")]
    #[test_case(PNG_SIGNATURE => Err(FrameError::NotJpeg); "not a JPEG")]
    #[test_case(JPEG_1280X800 => Err(FrameError::WrongSize { width: 1280, height: 800 }); "wrong size")]
    fn frame_checks(jpeg: &[u8]) -> Result<(), FrameError> {
        check_jpeg(jpeg)
    }
}

/// Policy images have a different shape from the legacy recorder's single camera.
pub fn check_policy_jpeg(bytes: &[u8]) -> Result<(), &'static str> {
    use jpeg_decoder::{Decoder, PixelFormat};
    if bytes.len() > 2 * 1024 * 1024 || !bytes.ends_with(&[0xff, 0xd9]) {
        return Err("invalid or incomplete JPEG");
    }
    let mut decoder = Decoder::new(bytes);
    decoder.set_max_decoding_buffer_size(640 * 360 * 4);
    decoder.read_info().map_err(|_| "invalid JPEG header")?;
    let info = decoder.info().ok_or("missing JPEG metadata")?;
    if (info.width, info.height, info.pixel_format) != (640, 360, PixelFormat::RGB24) {
        return Err("expected 640x360 RGB JPEG");
    }
    decoder.decode().map_err(|_| "invalid JPEG data")?;
    Ok(())
}

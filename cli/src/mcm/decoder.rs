use async_std::io;
use async_std::prelude::*;


pub const MAX7456_HDR: &'static str = "MAX7456";
pub const NVM_FONT_CHAR_DATA_SIZE: usize = 54;
pub const NVM_FONT_CHAR_METADATA_SIZE: usize = 10;

pub struct MCMDecoder<R> {
    input: R,
}

pub struct PixelChar {
    // pixel data
    pub pixels: [u8; NVM_FONT_CHAR_DATA_SIZE],
    // Metadata goes into the last 10 bytes
    pub metadata: [u8; NVM_FONT_CHAR_METADATA_SIZE],
}

impl PixelChar {
    pub fn new() -> PixelChar {
        PixelChar {
            pixels: [
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0
            ],
            metadata: Default::default(),
        }
	  }
}

impl<R: io::Read + std::marker::Unpin> MCMDecoder<R> {
    pub fn new(input: R) -> MCMDecoder<R> {
        MCMDecoder{
            input: input,
        }
	  }

    // TODO: this should return iterator, similar to BufReader.lines()
    pub async fn decode(self) -> io::Result<Vec<PixelChar>> {
        let reader = io::BufReader::new(self.input);
        let mut lines = reader.lines();

        // validate first line contains the max7456 header
        let hdr = match lines.next().await {
            Some(hdr) => hdr,
            None => return Err(io::Error::new(io::ErrorKind::InvalidInput, "input stream is empty, why are you wasting my time?")),
        };

        // validate first line contains the max7456 header
        if &hdr?[..] != MAX7456_HDR {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "expected MAX7456 header, giving up"));
        }

        let mut data: Vec<u8> = vec![];

        while let Some(line) = lines.next().await {
            let l = line?;

            let chars: Vec<char> = l.chars().collect();
            if chars.len() != 8 {
			          return Err(io::Error::new(io::ErrorKind::InvalidData, "line has invalid length, expected length of 8"))
		        }

            let d = match u8::from_str_radix(&l, 2) {
                Ok(uint_char) => uint_char,
                Err(_e) => {
                    return Err(io::Error::new(io::ErrorKind::InvalidData, "failed to parse {}"));
                }
            };

            data.push(d);
        }

        let pixel_chars = data
            .chunks(NVM_FONT_CHAR_DATA_SIZE + NVM_FONT_CHAR_METADATA_SIZE)
            .fold(vec![], |mut acc, chunk| {
                let mut p = PixelChar::new();
                p.pixels.clone_from_slice(&chunk[..NVM_FONT_CHAR_DATA_SIZE]);
                p.metadata.clone_from_slice(&chunk[NVM_FONT_CHAR_DATA_SIZE..]);

                acc.push(p);
                acc
            });

        return Ok(pixel_chars);
    }
}

//
// Copyright 2024 Christopher Atherton <the8lack8ox@pm.me>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Result, Seek, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};
use std::{env, fs};

// Unique file names
pub fn generate_unique_file_name(dir: &Path, ext: &str) -> PathBuf {
    let mut time_val = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .subsec_nanos();
    let mut path = dir.join(format!("{:08x}{ext}", time_val));
    while path.exists() {
        time_val = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos();
        path = dir.join(format!("{:08x}{ext}", time_val));
    }
    path
}

// Temporary directories
pub struct TempDir {
    path: PathBuf,
}

impl TempDir {
    pub fn new(prefix: &str) -> Self {
        let mut time_val = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos();
        let mut path = env::temp_dir().join(format!("{prefix}-{:08x}", time_val));
        while path.exists() {
            time_val = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .subsec_nanos();
            path = env::temp_dir().join(format!("{prefix}-{:08x}", time_val));
        }
        fs::create_dir(&path).expect("Could not create temporary directory");
        Self { path }
    }

    pub fn path(&self) -> &Path {
        self.path.as_path()
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.path).expect("Could not remove temporary directory");
    }
}

// CRC-32 checksum
pub struct Crc32Digest {
    sum: u32,
}

impl Crc32Digest {
    pub fn new() -> Self {
        Self { sum: 0xFFFFFFFF }
    }

    pub fn update(&mut self, data: &[u8]) {
        const TABLE: [u32; 256] = [
            0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535,
            0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD,
            0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D,
            0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
            0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4,
            0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
            0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC,
            0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
            0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB,
            0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F,
            0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB,
            0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
            0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA,
            0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE,
            0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A,
            0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
            0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409,
            0xCE61E49F, 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
            0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739,
            0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
            0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1, 0xF00F9344, 0x8708A3D2, 0x1E01F268,
            0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0,
            0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8,
            0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
            0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF,
            0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703,
            0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7,
            0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
            0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE,
            0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
            0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777, 0x88085AE6,
            0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
            0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D,
            0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5,
            0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605,
            0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
            0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
        ];

        for b in data {
            self.sum = (self.sum >> 8) ^ TABLE[(((*b as u32) ^ self.sum) & 0xFF) as usize];
        }
    }

    pub fn sum(&self) -> u32 {
        !self.sum
    }
}

impl Default for Crc32Digest {
    fn default() -> Self {
        Self::new()
    }
}

// UNIX time to MS-DOS date and time
pub fn unix_to_msdos_time(secs: u64) -> u32 {
    const SECS_PER_MINUTE: u64 = 60;
    const SECS_PER_HOUR: u64 = SECS_PER_MINUTE * 60;
    const SECS_PER_DAY: u64 = SECS_PER_HOUR * 24;
    const SECS_IN_MONTH: [u64; 12] = [
        SECS_PER_DAY * 31, // January
        SECS_PER_DAY * 28, // February
        SECS_PER_DAY * 31, // March
        SECS_PER_DAY * 30, // April
        SECS_PER_DAY * 31, // May
        SECS_PER_DAY * 30, // June
        SECS_PER_DAY * 31, // July
        SECS_PER_DAY * 31, // August
        SECS_PER_DAY * 30, // September
        SECS_PER_DAY * 31, // October
        SECS_PER_DAY * 30, // November
        SECS_PER_DAY * 31, // December
    ];
    const SECS_IN_MONTH_LEAP: [u64; 12] = [
        SECS_PER_DAY * 31, // January
        SECS_PER_DAY * 29, // February
        SECS_PER_DAY * 31, // March
        SECS_PER_DAY * 30, // April
        SECS_PER_DAY * 31, // May
        SECS_PER_DAY * 30, // June
        SECS_PER_DAY * 31, // July
        SECS_PER_DAY * 31, // August
        SECS_PER_DAY * 30, // September
        SECS_PER_DAY * 31, // October
        SECS_PER_DAY * 30, // November
        SECS_PER_DAY * 31, // December
    ];
    const SECS_PER_YEAR: u64 = SECS_IN_MONTH[0] // January
        + SECS_IN_MONTH[1] // February
        + SECS_IN_MONTH[2] // March
        + SECS_IN_MONTH[3] // April
        + SECS_IN_MONTH[4] // May
        + SECS_IN_MONTH[5] // June
        + SECS_IN_MONTH[6] // July
        + SECS_IN_MONTH[7] // August
        + SECS_IN_MONTH[8] // September
        + SECS_IN_MONTH[9] // October
        + SECS_IN_MONTH[10] // November
        + SECS_IN_MONTH[11]; // December
    const SECS_PER_YEAR_LEAP: u64 = SECS_IN_MONTH_LEAP[0] // January
        + SECS_IN_MONTH_LEAP[1] // February
        + SECS_IN_MONTH_LEAP[2] // March
        + SECS_IN_MONTH_LEAP[3] // April
        + SECS_IN_MONTH_LEAP[4] // May
        + SECS_IN_MONTH_LEAP[5] // June
        + SECS_IN_MONTH_LEAP[6] // July
        + SECS_IN_MONTH_LEAP[7] // August
        + SECS_IN_MONTH_LEAP[8] // September
        + SECS_IN_MONTH_LEAP[9] // October
        + SECS_IN_MONTH_LEAP[10] // November
        + SECS_IN_MONTH_LEAP[11]; // December
    const TIME_OF_1980: u64 = 8 * SECS_PER_YEAR + 2 * SECS_PER_YEAR_LEAP;

    let mut rem = secs - TIME_OF_1980;

    // Year
    let mut year = 0;
    for i in 0.. {
        if i % 4 != 0 {
            if rem < SECS_PER_YEAR {
                break;
            }
            rem -= SECS_PER_YEAR;
            year += 1;
        } else {
            if rem < SECS_PER_YEAR_LEAP {
                break;
            }
            rem -= SECS_PER_YEAR_LEAP;
            year += 1;
        }
    }

    // Month
    let mut month = 1;
    if year % 4 != 0 {
        for i in SECS_IN_MONTH {
            if rem < i {
                break;
            }
            rem -= i;
            month += 1;
        }
    } else {
        for i in SECS_IN_MONTH_LEAP {
            if rem < i {
                break;
            }
            rem -= i;
            month += 1;
        }
    }

    // Day
    let day = rem / SECS_PER_DAY + 1;
    rem -= (day - 1) * SECS_PER_DAY;

    // Hour
    let hour = rem / SECS_PER_HOUR;
    rem -= hour * SECS_PER_HOUR;

    // Minute
    let minute = rem / SECS_PER_MINUTE;
    rem -= minute * SECS_PER_MINUTE;

    // Second
    let second = rem / 2;

    // Pack
    ((year << 25) | (month << 21) | (day << 16) | (hour << 11) | (minute << 5) | second) as u32
}

pub struct SimpleZipArchive {
    writer: Box<dyn Write>,
    directory: Vec<u8>,
    position: u32,
    entry_count: u16,
    mtime: [u8; 4],
}

impl SimpleZipArchive {
    pub fn new(writer: impl Write + 'static) -> Self {
        Self {
            writer: Box::new(writer),
            directory: Vec::new(),
            position: 0,
            entry_count: 0,
            mtime: unix_to_msdos_time(
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs(),
            )
            .to_le_bytes(),
        }
    }

    pub fn create<P: AsRef<Path>>(path: P) -> Result<Self> {
        Ok(Self::new(File::create(path)?))
    }

    pub fn write_file<P: AsRef<Path>>(&mut self, in_path: P, file_name: String) -> Result<()> {
        let metadata = in_path.as_ref().metadata()?;
        let mut in_file = File::open(in_path)?;

        // Checksum
        let mut crc = Crc32Digest::new();
        let mut buffer = [0; 8192];
        let mut count = in_file.read(&mut buffer)?;
        while count > 0 {
            crc.update(&buffer[..count]);
            count = in_file.read(&mut buffer)?;
        }
        let crc_bytes = crc.sum().to_le_bytes();

        // File size
        let file_len = metadata.len() as u32;
        let file_len_bytes = file_len.to_le_bytes();

        // File name length
        let file_name_bytes = file_name.into_bytes();
        let file_name_len = file_name_bytes.len() as u16;
        let file_name_len_bytes = file_name_len.to_le_bytes();

        // Local file header
        self.writer.write_all(&[0x50, 0x4B, 0x03, 0x04])?; // Signature
        self.writer.write_all(&[0x14, 0x00])?; // Version
        self.writer.write_all(&[0x00, 0x00])?; // Flags
        self.writer.write_all(&[0x00, 0x00])?; // Compression method
        self.writer.write_all(&self.mtime)?; // Modification time
        self.writer.write_all(&crc_bytes)?; // CRC-32 checksum
        self.writer.write_all(&file_len_bytes)?; // Compressed size
        self.writer.write_all(&file_len_bytes)?; // Uncompressed size
        self.writer.write_all(&file_name_len_bytes)?; // File name length
        self.writer.write_all(&[0x00, 0x00])?; // Extra field length
        self.writer.write_all(&file_name_bytes)?; // File name

        // Copy file
        in_file.rewind()?;
        std::io::copy(&mut in_file, &mut self.writer)?;

        // Central directory entry
        self.directory.write_all(&[0x50, 0x4B, 0x01, 0x02])?; // Signature
        self.directory.write_all(&[0x14, 0x00])?; // Version made by
        self.directory.write_all(&[0x14, 0x00])?; // Minimum version needed
        self.directory.write_all(&[0x00, 0x00])?; // Flags
        self.directory.write_all(&[0x00, 0x00])?; // Compression method
        self.directory.write_all(&self.mtime)?; // Modification time
        self.directory.write_all(&crc_bytes)?; // CRC-32 checksum
        self.directory.write_all(&file_len_bytes)?; // Compressed size
        self.directory.write_all(&file_len_bytes)?; // Uncompressed size
        self.directory.write_all(&file_name_len_bytes)?; // File name length
        self.directory.write_all(&[0x00, 0x00])?; // Extra field length
        self.directory.write_all(&[0x00, 0x00])?; // Comment length
        self.directory.write_all(&[0x00, 0x00])?; // Disk number start
        self.directory.write_all(&[0x00, 0x00])?; // Internal attributes
        self.directory.write_all(&[0x00, 0x00, 0x00, 0x00])?; // External attributes
        self.directory.write_all(&self.position.to_le_bytes())?; // Offset of local header
        self.directory.write_all(&file_name_bytes)?; // File name

        // Update position
        self.position += 30 + file_name_len as u32 + file_len;

        // Update entry count
        self.entry_count += 1;

        Ok(())
    }
}

impl Drop for SimpleZipArchive {
    fn drop(&mut self) {
        // Central directory file headers
        self.writer
            .write_all(&self.directory)
            .expect("Could not write ZIP central directory file headers");

        // End of central directory record
        // Signature
        self.writer
            .write_all(&[0x50, 0x4B, 0x05, 0x06])
            .expect("Could not write ZIP end of central directory signature");
        // Disk number
        self.writer
            .write_all(&[0x00, 0x00])
            .expect("Could not write ZIP disk number");
        // Starting disk number
        self.writer
            .write_all(&[0x00, 0x00])
            .expect("Could not write ZIP starting disk number");
        // Total entries on this disk
        self.writer
            .write_all(&self.entry_count.to_le_bytes())
            .expect("Could not write ZIP disk entry count");
        // Total entries overall
        self.writer
            .write_all(&self.entry_count.to_le_bytes())
            .expect("Could not write ZIP overall entry count");
        // Central directory size
        self.writer
            .write_all(&(self.directory.len() as u32).to_le_bytes())
            .expect("Could not write ZIP central directory size");
        // Central directory offset
        self.writer
            .write_all(&self.position.to_le_bytes())
            .expect("Could not write ZIP central directory location");
        // Comment length
        self.writer
            .write_all(&[0x00, 0x00])
            .expect("Could not write ZIP end of central directory comment length");

        // Flush
        self.writer
            .flush()
            .expect("Could not flush ZIP file buffer");
    }
}

const PROGRAM_NAME: &str = "mkcbz";
const USAGE_MESSAGE: &str = "Usage: mkcbz [--avif] OUTPUT INPUT [INPUT]...";
type ArchiveType = SimpleZipArchive;

fn convert_avif(in_path: &Path, out_path: &Path) -> Child {
    match Command::new("avifenc")
        .arg(in_path)
        .arg(out_path)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
    {
        Ok(proc) => proc,
        Err(_) => {
            eprintln!("ERROR! Failed to run avifenc on `{}`", in_path.display());
            std::process::exit(1);
        }
    }
}

#[derive(Clone, PartialEq)]
enum Conversion {
    Copy,
    Avif,
}

struct Task {
    path: PathBuf,
    conversion: Conversion,
    convert_proc: Option<Child>,
}

impl Task {
    fn new(in_path: &Path, conversion: Conversion, work_dir: &Path) -> Self {
        match conversion {
            Conversion::Copy => Self {
                path: in_path.to_path_buf(),
                conversion,
                convert_proc: None,
            },
            Conversion::Avif => {
                let out_path = generate_unique_file_name(work_dir, ".avif");
                let proc = convert_avif(in_path, &out_path);
                Self {
                    path: out_path,
                    conversion,
                    convert_proc: Some(proc),
                }
            }
        }
    }

    fn finish(&mut self, index: usize, width: usize, archive: &mut ArchiveType) -> Result<()> {
        if let Some(ref mut child) = self.convert_proc {
            if !child.wait()?.success() {
                eprintln!("ERROR! Image encoding process returned failure");
                std::process::exit(1);
            }
        }
        let ext = match self.conversion {
            Conversion::Copy => match self.path.extension() {
                Some(ext) => {
                    String::from(".") + ext.to_string_lossy().to_ascii_lowercase().as_str()
                }
                None => String::new(),
            },
            Conversion::Avif => String::from(".avif"),
        };
        archive.write_file(&self.path, format!("{:0fill$}{ext}", index, fill = width))?;
        if self.conversion != Conversion::Copy {
            fs::remove_file(&self.path).expect("Could not remove temporary image file");
        }
        Ok(())
    }
}

fn run() -> Result<()> {
    // Check command line
    if env::args().len() < 3 {
        eprintln!("{USAGE_MESSAGE}");
        std::process::exit(1);
    }
    let conversion = match env::args().nth(1).unwrap().as_str() {
        "--avif" => Conversion::Avif,
        _ => Conversion::Copy,
    };
    let output_path;
    let cli_inputs: Vec<_>;
    if conversion == Conversion::Copy {
        output_path = env::args().nth(1).unwrap();
        cli_inputs = env::args().skip(2).map(PathBuf::from).collect();
    } else {
        if env::args().len() < 4 {
            eprintln!("{USAGE_MESSAGE}");
            std::process::exit(1);
        }
        output_path = env::args().nth(2).unwrap();
        cli_inputs = env::args().skip(3).map(PathBuf::from).collect();
    }

    // Collect inputs
    let mut inputs;
    if cli_inputs.len() == 1 && cli_inputs[0].is_dir() {
        inputs = Vec::new();
        for entry in fs::read_dir(&cli_inputs[0])? {
            let path = entry?.path();
            if !path.is_file() {
                eprintln!("ERROR! `{}` is not a file", path.display());
                std::process::exit(1);
            }
            inputs.push(path);
        }
        if inputs.is_empty() {
            eprintln!("ERROR! `{}` is empty", cli_inputs[0].display());
            std::process::exit(1);
        }
    } else {
        inputs = cli_inputs;
        for path in &inputs {
            if !path.exists() {
                eprintln!("ERROR! File `{}` does not exist", path.display());
                std::process::exit(1);
            }
            if !path.is_file() {
                eprintln!("ERROR! `{}` is not a file", path.display());
                std::process::exit(1);
            }
        }
    }
    inputs.sort();
    let width = inputs.len().to_string().len();
    let mut inputs_queue = VecDeque::from(inputs);

    // Create output file
    let mut archive = if output_path == "-" {
        ArchiveType::new(std::io::stdout())
    } else {
        ArchiveType::create(PathBuf::from(output_path))?
    };

    // Create work directory
    let work_path;
    let _work_dir;
    match conversion {
        Conversion::Copy => {
            work_path = PathBuf::new();
            _work_dir = None;
        }
        _ => {
            let tmp_dir = TempDir::new(PROGRAM_NAME);
            work_path = tmp_dir.path().to_path_buf();
            _work_dir = Some(tmp_dir);
        }
    }

    // Process
    let process_count = std::thread::available_parallelism()?.get();
    let mut task_queue = VecDeque::with_capacity(process_count);
    for _ in 0..(std::cmp::min(inputs_queue.len(), process_count) - 1) {
        let input = inputs_queue.pop_front().unwrap();
        task_queue.push_back(Task::new(&input, conversion.clone(), &work_path));
    }
    let mut index = 0;
    while let Some(input) = inputs_queue.pop_front() {
        // Submit new job
        task_queue.push_back(Task::new(&input, conversion.clone(), &work_path));

        // Finish front job
        index += 1;
        task_queue
            .pop_front()
            .unwrap()
            .finish(index, width, &mut archive)?;
    }
    // Finish rest of jobs
    while let Some(mut task) = task_queue.pop_front() {
        index += 1;
        task.finish(index, width, &mut archive)?;
    }

    Ok(())
}

fn main() {
    match run() {
        Ok(()) => (),
        Err(err) => {
            eprintln!("{err}");
            std::process::exit(1);
        }
    }
}

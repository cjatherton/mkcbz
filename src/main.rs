//
// Copyright 2024-2025 Christopher Atherton <the8lack8ox@pm.me>
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
use std::ffi::OsString;
use std::fmt;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::LazyLock;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

use opencv::prelude::*;
use opencv::{core, imgcodecs, imgproc};

use clap::Parser;

// Result<> has specific requirements for our purposes
type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;

// All formats we currently export to
#[derive(clap::ValueEnum, Clone, Debug)]
enum ImageFormat {
    Avif,
    Jpeg,
    Png,
    Webp,
}

impl ImageFormat {
    // Filename extension used for this format
    fn get_extension(&self) -> &str {
        match self {
            Self::Avif => "avif",
            Self::Jpeg => "jpg",
            Self::Png => "png",
            Self::Webp => "webp",
        }
    }
}

impl fmt::Display for ImageFormat {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::Avif => write!(f, "avif"),
            Self::Jpeg => write!(f, "jpeg"),
            Self::Png => write!(f, "png"),
            Self::Webp => write!(f, "webp"),
        }
    }
}

// mkcbz errors
#[derive(Debug)]
enum MkcbzError {
    FileOpenError(PathBuf),
    ImageCompressionError(ImageFormat),
    InvalidOutputPath(PathBuf),
    NotAFileOrDirectory(PathBuf),
    NotFound(PathBuf),
    ThreadJoinError,
    UnsupportedFormat(PathBuf),
}

impl fmt::Display for MkcbzError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::FileOpenError(path) => write!(f, "Failed to open file (\"{}\")", path.display()),
            Self::ImageCompressionError(img_fmt) => {
                write!(f, "Failed to compress image format (\"{img_fmt}\")")
            }
            Self::InvalidOutputPath(path) => {
                write!(f, "Not a valid output path (\"{}\")", path.display())
            }
            Self::NotAFileOrDirectory(path) => {
                write!(f, "Not a file or directory (\"{}\")", path.display())
            }
            Self::NotFound(path) => write!(f, "Not found (\"{}\")", path.display()),
            Self::ThreadJoinError => write!(f, "Thread joining error"),
            Self::UnsupportedFormat(path) => {
                write!(f, "Unsupported format (\"{}\")", path.display())
            }
        }
    }
}

impl std::error::Error for MkcbzError {}

// Fetch size of file
fn get_file_size(path: &Path) -> Result<u64> {
    Ok(fs::metadata(path)?.len())
}

// Check filename extension to see if we can use it
fn is_acceptable_image_format(path: &Path) -> bool {
    static KNOWN_EXTS: LazyLock<Vec<OsString>> = LazyLock::new(|| {
        [
            "avif", "bmp", "dib", "jpeg", "jpg", "jpe", "jp2", "pbm", "pgm", "ppm", "png", "tiff",
            "tif", "webp",
        ]
        .iter()
        .map(OsString::from)
        .collect()
    });
    match path.extension() {
        Some(ext) => (*KNOWN_EXTS).contains(&ext.to_ascii_lowercase()),
        None => false,
    }
}

// Check to see if the output location is valid
fn validate_output_path(path: &Path) -> Result<()> {
    // Get path parent
    let parent = match path.parent() {
        Some(parent) if parent.as_os_str().is_empty() => Path::new("."),
        Some(parent) => parent,
        None => {
            return Err(MkcbzError::InvalidOutputPath(path.to_path_buf()).into());
        }
    };

    // Does parent exist?
    if !parent.exists() {
        return Err(MkcbzError::InvalidOutputPath(path.to_path_buf()).into());
    }

    // Is parent a directory?
    if !parent.is_dir() {
        return Err(MkcbzError::InvalidOutputPath(path.to_path_buf()).into());
    }

    Ok(())
}

// Get inputs out of directories provided on the command line
fn collect_inputs(inputs: Vec<PathBuf>) -> Result<Vec<PathBuf>> {
    let mut ret = Vec::new();
    for input in inputs {
        if !input.exists() {
            return Err(MkcbzError::NotFound(input).into());
        }
        if input.is_dir() {
            let mut dir_contents = Vec::new();
            for path in input.read_dir()?.flatten().map(|e| e.path()) {
                if path.is_file() && is_acceptable_image_format(&path) {
                    dir_contents.push(path);
                }
            }
            dir_contents.sort();
            ret.append(&mut dir_contents);
        } else if input.is_file() {
            if is_acceptable_image_format(&input) {
                ret.push(input);
            } else {
                return Err(MkcbzError::UnsupportedFormat(input).into());
            }
        } else {
            return Err(MkcbzError::NotAFileOrDirectory(input).into());
        }
    }
    Ok(ret)
}

// Convenience image processing configuring struct
#[derive(Clone)]
struct ImageConfig {
    format: ImageFormat,
    quality: u32,
    color_thr: f64,
    denoise: bool,
}

// Command line parsing
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[arg(help = "Location to place output")]
    output: PathBuf,
    #[arg(required = true, num_args = 1.., help = "Input images")]
    input: Vec<PathBuf>,

    #[arg(short, long, default_value_t = ImageFormat::Webp, help = "Format for image conversion")]
    format: ImageFormat,
    #[arg(short, long, default_value_t = 80, value_parser = clap::value_parser!(u32).range(..=100), help = "Quality setting")]
    quality: u32,
    #[arg(
        short,
        long,
        default_value_t = 14.0,
        allow_negative_numbers = true,
        help = "Colorfulness threshold for grayscale detection"
    )]
    threshold: f64,
    #[arg(short = 'N', long, help = "Disable bilateral denoising filter")]
    no_denoise: bool,
    #[arg(
        short = 'T',
        long,
        default_value_t = 0,
        help = "Max number of threads (0 will use all available)"
    )]
    threads: usize,
}

// Information about a page to be processed
#[derive(Clone)]
struct InputPage {
    index: usize,
    path: PathBuf,
    config: ImageConfig,
}

// Information about a page that has been processed, including the compressed image data
#[derive(Clone)]
struct ProcessedPage {
    path: PathBuf,
    image: core::Vector<u8>,
    colorfulness: f64,
}

// Perceptual colorfulness metric by Hasler and Süsstrunk
fn calculate_colorfulness(mat: &Mat) -> Result<f64> {
    // Split into r, g and b channels
    let mut channels = core::Vector::<Mat>::new();
    core::split(mat, &mut channels)?;
    let b = &channels.get(0)?;
    let g = &channels.get(1)?;
    let r = &channels.get(2)?;

    // rg
    let mut rg = Mat::default();
    core::absdiff(r, g, &mut rg)?;
    let mut rg_mean_mat = Mat::default();
    let mut rg_stddev_mat = Mat::default();
    core::mean_std_dev_def(&rg, &mut rg_mean_mat, &mut rg_stddev_mat)?;
    let rg_mean = *rg_mean_mat.at_2d::<f64>(0, 0)?;
    let rg_stddev = *rg_stddev_mat.at_2d::<f64>(0, 0)?;

    // yb
    let mut rg_avg = Mat::default();
    core::add_weighted_def(&r, 0.5, &g, 0.5, 0.0, &mut rg_avg)?;
    let mut yb = Mat::default();
    core::absdiff(&rg_avg, &b, &mut yb)?;
    let mut yb_mean_mat = Mat::default();
    let mut yb_stddev_mat = Mat::default();
    core::mean_std_dev_def(&yb, &mut yb_mean_mat, &mut yb_stddev_mat)?;
    let yb_mean = *yb_mean_mat.at_2d::<f64>(0, 0)?;
    let yb_stddev = *yb_stddev_mat.at_2d::<f64>(0, 0)?;

    // Final calculation
    let std_root = rg_stddev.hypot(yb_stddev);
    let mean_root = rg_mean.hypot(yb_mean);
    Ok(std_root + (0.3 * mean_root))
}

// Process an input page
fn process_page(in_page: InputPage) -> Result<ProcessedPage> {
    // Read image file
    let mut img = imgcodecs::imread_def(
        in_page
            .path
            .as_os_str()
            .to_str()
            .expect("Path should be convertable to string"),
    )?;
    // If imread() fails, it returns an empty matrix
    if img.empty() {
        return Err(MkcbzError::FileOpenError(in_page.path).into());
    }

    // Color/grayscale detection
    let colorfulness = calculate_colorfulness(&img)?;

    // Colorspace conversion for grayscale
    if colorfulness <= in_page.config.color_thr {
        let mut tmp = Mat::default();
        imgproc::cvt_color_def(&img, &mut tmp, imgproc::COLOR_BGR2GRAY)?;
        img = tmp;
    }

    // Denoising
    if in_page.config.denoise {
        const ZERO_SIZE: core::Size = core::Size {
            width: 0,
            height: 0,
        };
        let mut tmp = Mat::default();
        imgproc::resize(&img, &mut tmp, ZERO_SIZE, 2.0, 2.0, imgproc::INTER_LINEAR)?;
        imgproc::bilateral_filter_def(&tmp, &mut img, 17, 7.0, 110.0)?;
        imgproc::resize(&img, &mut tmp, ZERO_SIZE, 0.5, 0.5, imgproc::INTER_AREA)?;
        img = tmp;
    }

    // Compress image
    let mut params = core::Vector::new();
    match in_page.config.format {
        ImageFormat::Avif => {
            params.push(imgcodecs::ImwriteFlags::IMWRITE_AVIF_QUALITY as i32);
            params.push(in_page.config.quality as i32);
        }
        ImageFormat::Jpeg => {
            params.push(imgcodecs::ImwriteFlags::IMWRITE_JPEG_QUALITY as i32);
            params.push(in_page.config.quality as i32);
        }
        ImageFormat::Png => (),
        ImageFormat::Webp => {
            params.push(imgcodecs::ImwriteFlags::IMWRITE_WEBP_QUALITY as i32);
            params.push(in_page.config.quality as i32);
        }
    }
    let mut buf = core::Vector::new();
    let success = imgcodecs::imencode(
        (String::from(".") + in_page.config.format.get_extension()).as_str(),
        &img,
        &mut buf,
        &params,
    )?;
    if !success {
        return Err(MkcbzError::ImageCompressionError(in_page.config.format).into());
    }

    // Return processed result
    Ok(ProcessedPage {
        path: in_page.path,
        image: buf,
        colorfulness,
    })
}

fn run() -> Result<()> {
    // Parse command line
    let cli = Cli::parse();

    // Validate output path
    validate_output_path(&cli.output)?;

    // Configuration for image conversion
    let config = ImageConfig {
        format: cli.format,
        quality: cli.quality,
        color_thr: cli.threshold,
        denoise: !cli.no_denoise,
    };

    // Number of threads
    let num_threads = if cli.threads == 0 {
        thread::available_parallelism()?.get()
    } else {
        cli.threads
    };

    // Generate input pages
    let mut old_size = 0;
    let mut input_pages = VecDeque::new();
    for (index, path) in collect_inputs(cli.input)?.into_iter().enumerate() {
        old_size += get_file_size(&path)?;
        input_pages.push_back(InputPage {
            index,
            path,
            config: config.clone(),
        });
    }

    // Shared data structures
    let num_pages = input_pages.len();
    let input_pages = Arc::new(Mutex::new(input_pages));
    let processed_pages = Arc::new(Mutex::new(
        // Make this Vec one element larger than we need
        // Otherwise the writing loop will try to access
        // A non-existent element at the last iteration
        (0..=num_pages).map(|_| None).collect::<Vec<_>>(),
    ));
    let should_continue = Arc::new(AtomicBool::new(true));
    let condvar = Arc::new(Condvar::new());

    // Worker threads
    let mut worker_handles = Vec::new();
    for _ in 0..num_threads {
        // Clone resources
        let input_pages = Arc::clone(&input_pages);
        let processed_pages = Arc::clone(&processed_pages);
        let should_continue = Arc::clone(&should_continue);
        let condvar = Arc::clone(&condvar);

        // Worker closure
        let handle = thread::spawn(move || {
            while should_continue.load(Ordering::SeqCst) {
                let in_page = {
                    let mut input_pages = input_pages.lock().unwrap();
                    input_pages.pop_front()
                };

                match in_page {
                    Some(in_page) => {
                        let page_index = in_page.index;
                        let processed = process_page(in_page);
                        let mut processed_pages = processed_pages.lock().unwrap();
                        processed_pages[page_index] = Some(processed);
                        condvar.notify_one();
                    }
                    None => {
                        condvar.notify_one();
                        break;
                    }
                }
            }
        });

        // Add to handle Vec
        worker_handles.push(handle);
    }

    // ZIP file writing
    let file = fs::File::create(&cli.output)?;
    let mut zip_archive = zip::ZipWriter::new(file);
    let zip_options =
        zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored);
    let format_ext = config.format.get_extension();
    let padding = num_pages.ilog10() as usize + 1;

    // Writing loop
    let mut next_index = 0;
    while next_index < num_pages {
        let mut pages_lock = processed_pages.lock().unwrap();

        // Wait for a result
        while pages_lock[next_index].is_none() {
            pages_lock = condvar.wait(pages_lock).unwrap();
        }

        // Insert all available pages that next in line into the ZIP
        while let Some(page_result) = pages_lock[next_index].take() {
            match page_result {
                Ok(page) => {
                    // Next page is ready, insert into archive
                    next_index += 1;

                    // Write into archive
                    zip_archive.start_file(
                        format!("{next_index:0fill$}.{format_ext}", fill = padding),
                        zip_options,
                    )?;
                    zip_archive.write_all(page.image.as_slice())?;

                    // Print information
                    let clr_str = if page.colorfulness <= config.color_thr {
                        "GRY"
                    } else {
                        "CLR"
                    };
                    println!(
                        "[{next_index:0fill$}/{num_pages}][{:06.2} = {clr_str}] {}",
                        page.colorfulness,
                        page.path.display(),
                        fill = padding
                    );
                }
                Err(err) => {
                    // Something bad happened!
                    // Release lock
                    drop(pages_lock);
                    // Notify workers to exit
                    should_continue.store(false, Ordering::SeqCst);
                    // Finalize progress so far on ZIP file
                    zip_archive.finish()?;
                    // Wait on workers
                    for handle in worker_handles {
                        if handle.join().is_err() {
                            return Err(MkcbzError::ThreadJoinError.into());
                        }
                    }
                    // Exit with error
                    return Err(err);
                }
            }
        }
    }

    // Clean up worker threads
    for handle in worker_handles {
        if handle.join().is_err() {
            return Err(MkcbzError::ThreadJoinError.into());
        }
    }

    // Write final elements to ZIP
    zip_archive.finish()?;

    // File size difference
    let new_size = get_file_size(&cli.output)?;
    let ratio = (new_size as f32) / (old_size as f32);
    let old_size_mb = (old_size as f32) / 1000000.0;
    let new_size_mb = (new_size as f32) / 1000000.0;
    println!("Size difference: {new_size_mb:.1} MB / {old_size_mb:.1} MB = {ratio:.3}");

    // Done
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) if err.is::<MkcbzError>() => {
            eprintln!("ERROR: {err}");
            ExitCode::FAILURE
        }
        Err(err) => {
            eprintln!("{err}");
            eprintln!("ERROR: Internal error");
            ExitCode::FAILURE
        }
    }
}

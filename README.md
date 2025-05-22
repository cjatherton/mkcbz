# mkcbz

Pack images into a comic book archive in ZIP format (CBZ).

## Overview
mkcbz is a simple tool for creating comic book archives while performing some additional processing. Specifically, mkcbz will convert images to grayscale even when not stored as such by using a colorfulness detection algorithm. It will also apply a denoising filter, which generally improves image quality and compressibility at the expense of some significant amount of time.

## Building

### Toolchain
I'm not sure about the exact toolchain requirements at this time. This project uses Rust 2024 edition.

### Dependency: OpenCV
mkcbz uses [OpenCV](https://opencv.org/) for its image processing, and the [Rust OpenCV bindings](https://crates.io/crates/opencv) are required. Therefore you must have `clang`, `clang-devel`, `opencv` and `opencv-devel` installed to build mkcbz.

### Compiling
Just run `cargo build --release` and you'll be good to go.

## Installing

### OpenCV
Install OpenCV. Be certain that `opencv-photo` is included part of the installation. You may have to install this as an addition.

On Fedora, this is done by:
```sh
sudo dnf install opencv opencv-photo
```

## Usage
The output file is the first argument passed to mkcbz. Then the images to be included are listed afterward.

Typical use of mkcbz is like so:
```sh
mkcbz output.cbz "Folder of Images"
```
You may also provide several sources of images like so:
```sh
mkcbz output.cbz "Folder 1" "Folder 2"
```
Or individual images:
```sh
mkcbz output.cbz image1.jpg image2.jpg
```
Or combinations of images and folders:
```sh
mkcbz output.cbz image1.jpg "Some Folder"
```

mkcbz will include inputs in the output in the order they appear on the command line. When a folder is provided, the contents are sorted by filename first and then added to the list to be imported. Files in folders that are not acceptable image formats are ignored. Subfolders in folders listed as inputs are not searched.

## FAQ

### Why identify images as color or grayscale?
If an image is stored as color but can be identified as a truly grayscale image, the color element can be removed. This can lead to, sometimes drastic, improvements in file size, as well as make the picture more visually pleasing.

### Why is mkcbz mistaking color pages as grayscale?
A perceptual colorfulness metric created by Hasler and Süsstrunk is used to evaluate whether an image is color or not. You can tune this with the `--threshold` option, using the output as a guide. If you want to turn off this feature, set `--threshold` to anything negative, i.e. -1.0. A setting of 0.0 should miss every color image while catching many grayscales in some instances. The default is 14.0.

The ability to force certain images as color or grayscale is a future goal.

### Why does mkcbz run so slow?
The non-local means denoising filter slows everything down considerably. If speed is of the essence, you can turn this off with `--no-denoise`. Denoising generally improves image quality, and frequently, compressibility increases as well.

### Why is mkcbz using so much memory?
Depending on image size and the number of threads you run concurrently, mkcbz can use a lot of memory on its processing. On several test sets, it peaked at ~3GB with 32 threads for me. Reduce the number of threads used with the `--threads` option if this is a problem.

### Why does mkcbz sometimes hang for a while when it first runs?
Pages are inserted into the CBZ archive in correct sequence. Often, the first page(s) in a set is/are in color. Color images are more demanding on the denoising filter. mkcbz has to wait for that page to be ready, even though work on other pages is being performed. mkcbz prints results in sequence as they are ready, so hanging on the first page is not unusual. Once color pages are processed, a lot of pages tend to be inserted leading to a sudden surge in output.

## License
This project is licensed under the [MIT License](LICENSE).
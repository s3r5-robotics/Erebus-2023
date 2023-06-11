import math
from os import PathLike
from typing import Iterable, Optional

from PIL import Image


# def plot(og_images: list[bytes], mod_images: list[bytes], size_wh=(64, 40)):
#     """
#     Plots the images in a grid.
#     """
#
#     def convert(bgra: bytes) -> bytearray:
#         """
#         Converts the image from BGRA to RGBA.
#         """
#         bgra = bytearray(bgra)
#         for pxi in range(0, len(bgra), 4):
#             bgra[pxi], bgra[pxi + 2] = bgra[pxi + 2], bgra[pxi]
#         return bgra
#
#     # Convert the images from bytes to PIL images
#     og_images: list[Image] = [Image.frombytes('RGBA', size_wh, convert(img)) for img in og_images]
#     mod_images: list[Image] = [Image.frombytes('RGBA', size_wh, convert(img)) for img in mod_images]
#
#     rows = math.ceil(len(mod_images) / len(og_images)) + 1
#     x_margin = 5
#     y_margin = 2
#
#     new_im = Image.new('RGBA', (len(og_images) * (size_wh[0] + y_margin) + y_margin,
#                                 rows * (size_wh[1] + x_margin) + x_margin))  # Create the final image grid
#     for i, im in enumerate(og_images + mod_images):
#         row = i // len(og_images)
#         col = i % len(og_images)
#
#         new_im.paste(im, (y_margin + col * (size_wh[0] + y_margin), x_margin + row * (size_wh[1] + x_margin)))
#
#     new_im.save(r"C:\Programming\RoboCup_Erebus\Erebus-2023\VictimDetection\test.png")


class ImagePlotter:
    def __init__(self, width: int, height: int, images_per_row: int, rotate_images: bool = False):
        self.rotate_images = rotate_images
        self.width = width
        self.height = height
        self.images_per_row = images_per_row
        self.images: list[Image] = []
        self.image: Optional[Image] = None

    def add_bgra(self, *images: bytes) -> 'ImagePlotter':
        for img in images:
            img = Image.frombytes('RGBA', (self.width, self.height), self._bgra_to_rgba(img))
            self.images.append(img.rotate(90) if self.rotate_images else img)
        return self

    def add_rgb(self, *images: bytes) -> 'ImagePlotter':
        for img in images:
            img = Image.frombytes('RGB', (self.width, self.height), img)
            self.images.append(img.rotate(90) if self.rotate_images else img)
        return self

    @staticmethod
    def _bgra_to_rgba(bgra: bytes) -> bytearray:
        """
        Converts the image from BGRA to RGBA.
        """
        bgra = bytearray(bgra)
        for pxi in range(0, len(bgra), 4):
            bgra[pxi], bgra[pxi + 2] = bgra[pxi + 2], bgra[pxi]
        return bgra

    def generate_image(self, x_margin: int = 1, y_margin: int = 1) -> Image:
        rows = math.ceil(len(self.images) / self.images_per_row) + 1

        width = self.height if self.rotate_images else self.width
        height = self.width if self.rotate_images else self.height

        self.image = Image.new('RGBA', (len(self.images) * (width + y_margin) + y_margin,
                                        rows * (height + x_margin) + x_margin))  # Create the final image grid
        for i, im in enumerate(self.images):
            row = i // self.images_per_row
            col = i % self.images_per_row

            self.image.paste(im, (y_margin + col * (width + y_margin), x_margin + row * (height + x_margin)))

        return self.image

    def save(self, path: PathLike, image: Image = None) -> None:
        (image or self.image or self.generate_image()).save(path)

    def clear(self) -> None:
        self.images.clear()
        self.image = None

from os import PathLike
from typing import Optional, Literal, Union, Sequence, Iterable, Callable

import numpy as np
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
    def __init__(self, width: int, height: int, columns: int):
        self.width = width
        self.height = height
        self.columns = columns
        self.images: list[list[Image]] = [[] for _ in range(columns)]
        self.image: Optional[Image] = None

    @staticmethod
    def _bgra_to_rgba(bgra: bytes) -> bytearray:
        """
        Converts the image from BGRA to RGBA.
        """
        bgra = bytearray(bgra)
        for pxi in range(0, len(bgra), 4):
            bgra[pxi], bgra[pxi + 2] = bgra[pxi + 2], bgra[pxi]
        return bgra

    def add(self, *images: Union[bytes, Iterable[bytes]], mode: Literal["BRGA", "RGB", "L", "P"],
            rotations: Union[None, Literal[-90, 0, 90, 180], Sequence[Literal[-90, 0, 90, 180]]] = None,
            column: int = None) -> list[Image]:
        """
        Add images to the row(s) or column of the image grid

        :param images:     The images to add.
        :param mode:       The mode of the images.
        :param rotations:  The rotation to apply to all images or separate rotation for each image.
        :param column:     If None, each image will be added to its own column.
                           Otherwise, all images will be added to the specified column.
        """
        conversion_function, mode = {
            "BRGA": (self._bgra_to_rgba, "RGBA"),
        }.get(mode, (lambda x: x, mode))  # type: Callable[[bytes], bytearray], str

        # If images were provided as a list of images (single first argument), use that list
        if not isinstance(images[0], (bytes, bytearray, np.ndarray)):
            images = images[0]  # type: Iterable[bytes]

        generated_images = []
        for i, img in enumerate(images):
            if rotations is None:
                rotation = 0
            elif isinstance(rotations, int):
                rotation = rotations
            else:
                rotation = rotations[i]

            # If rotations is (multiple of) 90 degrees, the width and height are swapped
            if rotation % 180 == 90:  # Not divisible by 180
                size = self.height, self.width
            else:
                size = self.width, self.height

            img = Image.frombytes(mode, size, conversion_function(img))
            if rotation:
                img = img.rotate(rotation)

            if column is None:
                self.images[i].append(img)
            else:
                self.images[column].append(img)
            generated_images.append(img)

        return generated_images

    def generate_image(self, x_margin: int = 1, y_margin: int = 1) -> Image:
        rows = max(len(column) for column in self.images) + 1

        self.image = Image.new('RGBA', (y_margin + self.columns * (self.width + y_margin),
                                        x_margin + rows * (self.height + x_margin)))  # Create the final image grid
        for col_i, column in enumerate(self.images):
            for row_i, image in enumerate(column):
                self.image.paste(image, (y_margin + col_i * (self.width + y_margin),
                                         x_margin + row_i * (self.height + x_margin)))

        return self.image

    def save(self, path: PathLike, image: Image = None) -> None:
        (image or self.image or self.generate_image()).save(path)

    def clear(self) -> None:
        for column in self.images:
            column.clear()
        self.image = None

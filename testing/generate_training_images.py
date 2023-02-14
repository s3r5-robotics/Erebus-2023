import cv2
import numpy as np


def generate_images(image: np.ndarray, is_letter: bool, resolution: tuple[int, int], n: int) -> list[np.ndarray]:
    """Generate `n` images for character recognition AI model training. The images have a random horizontal POV."""
    images: list[np.ndarray] = []

    for i in range(n):

        img = cv2.resize(image, resolution)
        img = cv2.resize(img, (1000, 1000), interpolation=cv2.INTER_NEAREST)
        images.append(img)

    return images


def main():
    image = cv2.imread(r"C:\Programming\RoboCup - Erebus v22\Erebus-2022 - Code\testing\test.png")
    for n, i in enumerate(generate_images(image=image, is_letter=False, resolution=(10, 10), n=3)):
        cv2.imshow(f"image_{n}", i)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()

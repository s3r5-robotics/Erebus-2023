import math
import os

import cv2 as cv
import numpy as np

# Določi pot do mape, kjer se nahajajo slike.
script_dir = os.path.dirname(__file__)
image_dir = os.path.join(script_dir, "images")


# Shrani sliko v mapo z določenim imenom.
def save_image(image, filename):
    cv.imwrite(os.path.join(image_dir, filename), image)


# Normalizira kot v radianih na interval [0, 2*pi].
def normalizeRads(rad):
    rad %= 2 * math.pi
    if rad < 0:
        rad += 2 + math.pi
    return rad


# Pretvori kot v stopinje v radiane.
def degsToRads(deg):
    return deg * math.pi / 180


# Pretvori kot v radiane v stopinje.
def radsToDegs(rad):
    return rad * 180 / math.pi


# Preslika vrednost iz enega intervala v drugega.
def mapVals(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Izračuna koordinate na podlagi kota v radianih in razdalje.
def getCoordsFromRads(rad, distance):
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


# Izračuna koordinate na podlagi kota v stopinjah in razdalje.
def getCoordsFromDegs(deg, distance):
    rad = degsToRads(deg)
    y = float(distance * math.cos(rad))
    x = float(distance * math.sin(rad))
    return (x, y)


# Vrne produkt dveh seznamov.
def multiplyLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 * item2)
    return finalList


# Vrne vsoto dveh seznamov.
def sumLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 + item2)
    return finalList


# Vrne razliko dveh seznamov.
def substractLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 - item2)
    return finalList


# Vrne kvocient dveh seznamov.
def divideLists(list1, list2):
    finalList = []
    for item1, item2 in zip(list1, list2):
        finalList.append(item1 / item2)
    return finalList


# Sprejme sliko, velikost kvadrata in zamik ter vrne sliko z narisano mrežo.
# Mreža je sestavljena iz kvadratov, katerih velikost je določena z square_size.
# Kvadrati so narisani na vsakem square_size-tem pikslu v sliki, ki se nahaja v mapi images.
# Če je color parameter podan, se uporabi ta barva za risanje mreže, sicer se uporabi privzeta barva (belo).
# Funkcija vrne sliko z narisano mrežo.
def draw_grid(image, square_size, offset=[0, 0], color=255):
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if (y + offset[1]) % square_size == 0 or (x + offset[0]) % square_size == 0:
                if len(image.shape) == 3:
                    image[y][x][:] = color
                else:
                    image[y][x] = color


# Sprejme sliko, seznam koordinat in barvo ter vrne sliko z narisanimi koordinatami.
# Koordinate so podane kot seznam dveh seznamov, kjer prvi seznam vsebuje x-koordinate, drugi pa y-koordinate.
# Če je parameter xx_yy_format nastavljen na True, se koordinate obravnavajo kot seznam dveh seznamov, kjer prvi seznam vsebuje y-koordinate, drugi pa x-koordinate.
# Če je podan parameter back_image, se koordinate, ki niso znotraj meja slike, preskočijo.
# Funkcija vrne sliko z narisanimi koordinatami.
def draw_poses(image, poses, color=255, back_image=None, xx_yy_format=False):
    if xx_yy_format:
        if back_image is not None:
            in_bounds_x = (poses[0] < min(image.shape[0], back_image.shape[0]) - 1) & (poses[0] > 0)
            in_bounds_y = (poses[1] < min(image.shape[1], back_image.shape[1]) - 1) & (poses[1] > 0)
        else:
            in_bounds_x = (poses[0] < image.shape[0] - 1) & (poses[0] > 0)
            in_bounds_y = (poses[1] < image.shape[1] - 1) & (poses[1] > 0)

        poses = (poses[0][in_bounds_x & in_bounds_y], poses[1][in_bounds_x & in_bounds_y])

        if back_image is None:
            image[poses[1], poses[0], :] = color
        else:
            image[poses[1], poses[0], :] = back_image[poses[1], poses[0], :]

    else:
        in_bounds = (poses[:, 0] >= 0) & (poses[:, 0] < image.shape[1]) & (poses[:, 1] >= 0) & (
                poses[:, 1] < image.shape[0])
        poses = poses[in_bounds]

        if back_image is None:
            image[poses[:, 1], poses[:, 0], :] = color
        else:
            image[poses[:, 1], poses[:, 0], :] = back_image[poses[:, 1], poses[:, 0], :]


# Funkcija sprejme sliko, velikost kvadrata in barvo ter vrne sliko z narisanimi kvadrati.
# Kvadrati so narisani na vsakem square_size-tem pikslu v sliki, ki se nahaja v mapi images.
# Kvadrati so narisani samo na tistih lokacijah, kjer je v sliki vrednost piksla različna od 0.
# Če je color parameter podan, se uporabi ta barva za risanje kvadratov, sicer se uporabi privzeta barva (belo).
# Funkcija vrne sliko z narisanimi kvadrati.
def draw_squares_where_not_zero(image, square_size, offsets, color=(255, 255, 255)):
    ref_image = image.copy()
    for y in range(image.shape[0] // square_size):
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size) + (square_size - offsets[1]),
                ((y + 1) * square_size) + (square_size - offsets[1]),
                (x * square_size) + (square_size - offsets[0]),
                ((x + 1) * square_size) + (square_size - offsets[0])]
            square = ref_image[square_points[0]:square_points[1], square_points[2]:square_points[3]]
            non_zero_count = np.count_nonzero(square)
            if non_zero_count > 0:
                # print("Non zero count: ", non_zero_count)
                # print("max: ", np.max(square))
                cv.rectangle(image, (square_points[2], square_points[0]), (square_points[3], square_points[1]), color,
                             3)


# Funkcija sprejme sliko, velikost kvadrata in prag ter vrne seznam koordinat kvadratov.
# Kvadrati so velikosti square_size in so locirani na vsakem square_size-tem pikslu v sliki, ki se nahaja v mapi images.
# Koordinate kvadratov so vrnjene v obliki seznama dveh seznamov, kjer prvi seznam vsebuje x-koordinate, drugi pa y-koordinate.
# Kvadrati so vrnjeni samo na tistih lokacijah, kjer je povprečna vrednost piksla v kvadratu večja od threshold-a.
def get_squares(image, square_size, offsets):
    grid = []
    for y in range(image.shape[0] // square_size):
        row = []
        for x in range(image.shape[1] // square_size):
            square_points = [
                (y * square_size) + (square_size - offsets[1]),
                ((y + 1) * square_size) + (square_size - offsets[1]),
                (x * square_size) + (square_size - offsets[0]),
                ((x + 1) * square_size) + (square_size - offsets[0])]
            row.append(square_points)
        grid.append(row)
    return grid


# Funkcija sprejme pot do slike in velikost ter vrne novo sliko z določeno velikostjo.
# Slika se naloži iz datoteke, ki se nahaja na poti image_path.
# Nato se slika pomanjša ali poveča na določeno velikost, ki je podana kot parameter size.
# Funkcija vrne novo sliko z določeno velikostjo.
def resize_image_to_fixed_size(image, size):
    if image.shape[0] > size[0]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]))

    elif image.shape[1] > size[1]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height))

    elif image.shape[1] >= image.shape[0]:
        ratio = size[1] / image.shape[1]

        height = round(image.shape[0] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(size[1], height), interpolation=cv.INTER_NEAREST)

    elif image.shape[0] >= image.shape[1]:
        ratio = size[0] / image.shape[0]

        width = round(image.shape[1] * ratio)
        final_image = cv.resize(image.astype(np.uint8), dsize=(width, size[0]), interpolation=cv.INTER_NEAREST)

    return final_image


def divide_into_chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

""" simple script to convert image into c array

Image is converted into a 4 color greyscale image
and then exported as a space-optimized array.
The exact format can be found at the end of the
file as it is printed as a comment into the resulting
C file.
"""
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


def rgb2grey(rgb):
    """ convert the rgb value (3 reals) into a greyscale value (1 real) """
    # return sum(rgb[0:3]) / 3
    return 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]


def grey2q(grey):
    """ return an integer that corresponds to the quantized level of grey """
    c = 0
    if grey > 0.17:
        c = 1
    if grey > 0.5:
        c = 2
    if grey > 0.83:
        c = 3
    return c


def load(img_name):
    """ load and return image as numpy array """
    ext = img_name.split('.')[-1]
    img = mpimg.imread(img_name)

    if ext == 'gif':
        img = img / 255
    return img


def align_size(img):
    """ resize picture to multiple of 4 pixels height """
    shape = img.shape
    grow_by = (4 - (shape[0] % 4), shape[1], shape[2])
    img = np.vstack((img, np.ones(grow_by)))

    return img


def greyscale(img):
    """ replace colors and generate greyscale data for c array """
    cd = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    grey = np.zeros(img.shape)
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            g = rgb2grey(img[y][x])
            c = grey2q(g)
            cd[y][x] = int(c)
            grey[y][x] = np.hstack((np.array([1, 1, 1]) * c / 3, 1))
            # print('@{}, {} -> {:.2f} -> {} : {}'.format((y,x), img[y][x], g, c, greyscale[y][x]))
    return cd, grey


def combine_pixels(cd):
    """ combine multiple pixels into the corresponding array element """
    ca = np.zeros((cd.shape[0] / 4, cd.shape[1]), dtype=np.uint8)
    for y in range(ca.shape[0]):
        for x in range(ca.shape[1]):
            v = 0
            for n in range(4):
                v += cd[4 * y + n][x] * 4 ** n
            ca[y][x] = v

    return ca


def dump(name, ca):
    """ dump combined array into file """
    with open('{}.c'.format(name), 'w') as f:
        f.write(c_template.format(name=name, shape=ca.shape))

        f.write('{\n');
        for y in range(ca.shape[0]):
            f.write('  {')
            for x in range(ca.shape[1]):
                f.write('0x{:x}'.format(ca[y][x]))
                if x < ca.shape[1] - 1:
                    f.write(',')
            f.write('}')
            if y < ca.shape[0] - 1:
                f.write(',')
            f.write('\n')
        f.write('}\n')

    with open('{}.h'.format(name), 'w') as f:
        f.write(h_template.format(name=name, shape=ca.shape))


def convert(img_name, name):
    """ one complete conversion run for one picture """
    img = load(img_name)
    img = align_size(img)
    cd, grey = greyscale(img)
    ca = combine_pixels(cd)
    plt.imshow(grey, interpolation='none')
    dump(name, ca)
    return img, cd, greyscale, ca


def testimage():
    """ create a simple RGB test image """
    img = np.zeros((11, 4, 4))
    for i in zip(range(11), np.linspace(0, 1, 11, endpoint=True)):
        for n in range(3):
            img[i[0]][n][n] = i[1]
            img[i[0]][n][3] = 1
        img[i[0]][3][0:3] = i[1]
        img[i[0]][3][3] = 1
    return img


# quick test
# img = testimage()
# cd, grey = greyscale(img)
# plt.imshow(np.vstack((img, grey)), interpolation='none')
# plt.show()

c_template = """/** array for {name} image
automatically converted to 4 color greyscale image
usable on eInk display.

Format of the array:
    order of bytes:
        1  2  3 
        4  5  6
        7  8  9
    order of pixels in byte:
        a
        b
        c
        d

        a | b << 2 | c << 4 | d << 6
**/

UINT8 {name}[{shape[0]}][{shape[1]}] = """

h_template = """#ifndef _{name}_h_
#define _{name}_h_

extern UINT8 {name}[{shape[0]}][{shape[1]}];

#endif // _{name}_h_
"""
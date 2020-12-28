# https://stackoverflow.com/questions/13550376/pil-image-to-array-numpy-array-to-array-python
from PIL import Image
import numpy as np
import serial
import time
import sys

arguments = sys.argv[1:]


def image2pixelarray(filepath):
    """
    Parameters
    ----------
    filepath : str
        Path to an image file

    Returns
    -------
    list
        A list of lists which make it simple to access the greyscale value by
        im[y][x]
    """
    im = Image.open(filepath).convert('L')
    (width, height) = im.size
    greyscale_map = list(im.getdata())
    greyscale_map = np.array(greyscale_map)
    greyscale_map = greyscale_map.reshape((height, width))
    return greyscale_map, width, height

# wpis = input()

# while(wpis != 'q'):
#   wpis = input()
#  arduino.write(wpis.encode('ASCII'))
# print("Wyslalem: " + wpis)


def open_port():
    arduino = serial.Serial(
        port='COM16',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO
    )
    arduino.flushOutput()
    print('Serial connected')

    return arduino


def dec_to_bin(f):
    return int(bin(f)[2:])


def read_port():
    return ard.read().decode('ASCII')


def send_port(command):
    command += "\r\n"
    if read_port() == "r":
        ard.write(command.encode('ASCII'))

#img, szer, wys = image2pixelarray("tekst.bmp")
#img, szer, wys = image2pixelarray("star.bmp")
#img, szer, wys = image2pixelarray("miki1.bmp")
#img, szer, wys = image2pixelarray("klak.bmp")
#img, szer, wys = image2pixelarray("troll.jpg")
#img, szer, wys = image2pixelarray("VerticalLine.bmp")
#img, szer, wys = image2pixelarray("PrintFile.bmp")
#img, szer, wys = image2pixelarray("tuxg.gif")
#img, szer, wys = image2pixelarray("xd.bmp")
img, szer, wys = image2pixelarray(arguments[0])

print("SZEROKOSC: " + str(szer))
print("WYSOKOSC: " + str(wys))
stala = int(5000/szer)
print("STALA: " + str(stala))

# print(img[0, 0])

licznik = 0
ard = open_port()

while read_port() != "r":
    print("Waiting for 'r' from Arduino")
    time.sleep(1)

# send_port("p1000")

# main


def main():

    zm = 0
    global licznik

    for x in range(0, wys):

        send_port("e")  # 100 w gore
        print("height: " + str(x))

        if all(img[x]) is True:
            print("OPUSZCZAM")
            continue

        if zm == 0:
            zm = 1
            licznik = 0
            #print("PRZOD")
            for y in range(0, szer):
                #print (str(y))

                if dec_to_bin(img[x, y]) == 0:
                    send_port("p"+str(licznik*stala))
                    #send_port("R")
                    send_port("R1")
                    licznik = 0
                else:
                    send_port("R0")
                #print("p" + str(licznik))
                if y == szer-1:
                    send_port("p" + str(licznik*stala))

                licznik += 1

        elif zm == 1:
            zm = 0
            licznik = 0
            #print("COFAM")
            for z in range(szer-1, -1, -1):
                #print(str(z))

                if dec_to_bin(img[x, z]) == 0:
                    send_port("t"+str(licznik*stala))
                    #send_port("R")
                    send_port("R1")
                    licznik = 0
                else:
                    send_port("R0")

                #print("t" + str(licznik))

                if z == 0:
                    send_port("t" + str(licznik*stala))

                licznik += 1
    send_port("k")  # wysuwanie tacki

#end main

        # send_port("t"+str(szer*stala))
        # print("COFAM")

main()

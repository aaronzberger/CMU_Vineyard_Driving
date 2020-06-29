import png

width = 255
height = 255
img = []
for y in range(height):
    row = ()
    for x in range(width):
        if y == 7 and x == 4:
            row = row + (0,0,0)
        else:
            row = row + (255, 255, 255)
    img.append(row)
with open('gradient.png', 'wb') as f:
    w = png.Writer(width, height, greyscale=False)
    w.write(f, img)
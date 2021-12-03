# This program creates a set of test images for confirming the accuracy of the image compositor
import os

from pgmagick import DrawableGravity, Image, Geometry, Color, ColorRGB, \
                     DrawableText, DrawableRectangle, DrawableList, DrawableGravity, GravityType, DrawableColor

for i in range(1, 91):
    dl = DrawableList()

    im = Image(Geometry(1280, 960), Color("#01FE01"))
    im.strokeColor(Color("#0000"))
    im.strokeWidth(2)
    im.fillColor(Color("#FF8600"))
    rectangle = DrawableRectangle(160, 120, 1119, 839)
    dl.append(rectangle)

    im.fontPointsize(65)
    im.font("/var/lib/defoma/x-ttcidfont-conf.d/dirs/TrueType/UnBatang.ttf")
    dl.append(DrawableGravity(GravityType.CenterGravity))
    dl.append(DrawableText(0, 0, str(i)))
    im.draw(dl)
    if not os.path.exists('test_images'):
        os.makedirs('test_images')
    im.write('test_images/{}.png'.format(i))
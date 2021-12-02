# This program creates a set of test images for confirming the accuracy of the image compositor
from pgmagick import DrawableGravity, Image, Geometry, Color, ColorRGB, \
                     DrawableCircle, DrawableText, DrawableRectangle, DrawableList, DrawableGravity, GravityType, DrawableColor

for i in range(1, 91):
    dl = DrawableList()

    im = Image(Geometry(1280, 960), Color("#01FE01"))
    circle = DrawableCircle(100.0, 100.0, 20.0, 20.0)
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
    im.write('{}.png'.format(i))
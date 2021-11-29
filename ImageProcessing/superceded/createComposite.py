# Imports:
# Image: Image class, contains basic attribute functions
# DrawableCompositeImage: Method for creating a composite
from pgmagick.api import Image
from pgmagick import DrawableCompositeImage, DrawableGravity, GravityType
from pgmagick import CompositeOperator as co


"""
This opearion creates a base, black image which the comosite image of the stitches will be built on.
Parameters to make it work:
composite_width
composite_length

each image needs to have a reference point mapped to it
{image: Image,
 x: x,
 y: y}

The image's x-y location in machine space is converted relative to the output image's dimensions
"""
px_per_mm = 180
images = []
images.append({"image": Image('./image_output/set1.png'), "xabs": 10, "yabs": 12})
images.append({"image": Image('./image_output/set2.png'), "xabs": 20, "yabs": 10})
print(type(images[0]['image']))

xmin = float("inf")
ymin = float("inf")
xmax = 0
ymax = 0
xmax_width = 0
ymax_height = 0

for i in images:
    print(i['xabs'])
    if int(i['xabs']) > xmax :
        xmax = i['xabs']
        xmax_width = i['image'].width
    if int(i['yabs']) > ymax :
        ymax = i['yabs']
        ymax_height = i['image'].height

    xmin = int(i['xabs']) if int(i['xabs']) < xmin else xmin
    ymin = int(i['yabs']) if int(i['yabs']) < ymin else ymin


for i in images:        
    i['xrel_mm'] = i['xabs'] - xmin
    i['yrel_mm'] = i['yabs'] - ymin
    i['xrel_px'] = i['xrel_mm'] * px_per_mm
    i['yrel_px'] = i['yrel_mm'] * px_per_mm
    print(i['image'].height)

print("xmax {} xmin {} ymax {} ymin{} xmax_width {} ymax_height {}".format(xmax, xmin, ymax, ymin, xmax_width, ymax_height))
outWidth = (xmax - xmin) * px_per_mm + xmax_width
outHeight = (ymax - ymin) * px_per_mm + ymax_height
testOut = Image((outWidth, outHeight), 'black')
print(testOut.width, testOut.height)

for i in images:
    testOut.draw(DrawableCompositeImage(i['xrel_px'], i['yrel_px'], i['image'].width, i['image'].height, i['image'].img))
# testOut.draw(DrawableCompositeImage(0,0, './aerial_exp_out/set1.png'))
# testOut.draw(DrawableGravity(GravityType.NorthGravity))
# testOut.draw(DrawableCompositeImage(img1.width-400, 0, './aerial_exp_out/set2.png'))
testOut.write("test9.png")

exit()


img1 = Image('./aerial_exp_out/set1.png')
img2 = Image('./aerial_exp_out/set2.png')

x = img1.width + img2.width
y = img1.height + img2.height

imgOut = Image((x, y), 'black')
print(imgOut.width, imgOut.height)


# Creates composite on black background, need to figure out how to set offsets
# Need to make a drawlist and feed that into draw
imgOut.draw(DrawableCompositeImage(0,0, './aerial_exp_out/set1.png'))
imgOut.draw(DrawableGravity(GravityType.NorthGravity))
imgOut.draw(DrawableCompositeImage(img1.width-400, 0, './aerial_exp_out/set2.png'))
imgOut.write("test7.png")

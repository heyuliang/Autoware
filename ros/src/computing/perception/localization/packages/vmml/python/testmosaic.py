#!/usr/bin/python

from PIL import Image
from colour_demosaicing import *

pattern = "gbrg"

image = Image.open("../test/raw1.png")
res1 = demosaicing_CFA_Bayer_bilinear(image, pattern)
res2 = demosaicing_CFA_Bayer_Malvar2004(image, pattern)
res3 = demosaicing_CFA_Bayer_Menon2007(image, pattern, refining_step=True)

rei = Image.fromarray(res1)
rei.save("/tmp/1.png")
rei = Image.fromarray(res2)
rei.save("/tmp/2.png")
rei = Image.fromarray(res3)
rei.save("/tmp/3.png")

pass
from pxr import Usd
stage = Usd.Stage.Open("m0609_polishing.usd")
stage.Export("m0609_polishing.usda")

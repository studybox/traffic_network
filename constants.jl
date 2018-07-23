using CSV
SUMO = "sumo"
SUMOGUI = "sumo-gui"
SUMO_HOME = "/media/mSATA/UM/Simulation/sumo"
SUMO_CONFIG = "/media/mSATA/UM/Simulation/f_reduced2_AnnArbor.sumocfg"
#XBOUNDLEFT = 265395
#XBOUNDRIGHT = 281180
#YBOUNDTOP = 4691486
#YBOUNDBOT = 4676351
XBOUNDLEFT = 262000
XBOUNDRIGHT = 291000
YBOUNDTOP = 4694500
YBOUNDBOT = 4670500
BACKGROUND_FILE = "/media/mSATA/UM/Upper routing simulation/SUMOdata/f_background2.png"
data_links = CSV.read("/media/mSATA/UM/Simulation/test_ann_arbor.csv")
CARICONS = [
   "/home/boqi/Pictures/noun_Car_1822714_000000.png",
   "/home/boqi/Pictures/noun_Car_1822714_51A7F9.png",
   "/home/boqi/Pictures/noun_Car_1822714_70c1b3.png",
   "/home/boqi/Pictures/noun_Car_1822714_70C041.png",
   "/home/boqi/Pictures/noun_Car_1822714_B36AE2.png",
   "/home/boqi/Pictures/noun_Car_1822714_EC5D57.png",
   "/home/boqi/Pictures/noun_Car_1822714_F38F19.png"]
COLORPOOL = [
      (155/255, 196/255, 68/255),
      (79/255, 120/255, 255/255),
      (112/255,193/255,179/255),
      (237/255,183/255,56/255),
      (229/255,82/255,97/255),
      (133/255,135/255,132/255),
      (153/255,0/255,250/255),
      (62/255,175/255,118/255),
      (191/255,172/255,5/255),
      (253/255,121/255,143/255),
      (193/255,74/255,9/255),
      (117/255,8/255,81/255),
      (33/255,71/255,97/255),
      (1/255,84/255,130/255)]

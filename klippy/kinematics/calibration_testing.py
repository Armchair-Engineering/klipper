import math

degree_delta = 326.602
small_angle = 360 - degree_delta
delta_in_radians = small_angle * math.pi / 180
magnet_diameter = 6  # mm

radius = (magnet_diameter / 2) / math.tan(delta_in_radians / 2)

import math

def s_func(dis_max, dis_min, spd_max, spd_min, target_dis):
    if target_dis > dis_max:
        spd = spd_max
    elif target_dis < dis_min:
        spd = spd_min
    else:
        spd = (spd_max-spd_min)*((math.cos(math.pi*((target_dis-dis_min)/(dis_max-dis_min)-1))+1)/2)+spd_min
    return spd

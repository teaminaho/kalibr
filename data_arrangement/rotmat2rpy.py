import yaml
import numpy as np
from scipy.spatial.transform import Rotation as Rot

f = open("./camchain-zense.yaml", "r") 
data = yaml.load(f) 
rot_mat = np.asarray(data["cam1"]["T_cn_cnm1"])[:3,:3] 
r = Rot.from_dcm(rot_mat)
degrees = r.as_euler('yzx',degrees=True) 
print("rot_angle_roll = %f"%(-degrees[1]))
print("rot_angle_pitch = %f"%(-degrees[2]))
print("rot_angle_yaw = %f"%(-degrees[0]))


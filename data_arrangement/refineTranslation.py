import toml
import yaml
import numpy as np
import os
import os.path as osp
import glob
import copy
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import minimize
import pdb

TOML_PATH = 'recognition_parameter.toml'
#TOML_PATH = 'recognition_parameter_estimated.toml'
YAML_PATH = 'camchain-zense.yaml'
CSV_PATH = './result'

def getPoints2DFromCSV(csv_path):
    points2d = np.loadtxt(csv_path,delimiter=",")[:,:2]
    points2d_target_projected_to_another = np.loadtxt(csv_path,delimiter=",")[:,4:7]
    points2d_target_reprojected_to_another = np.loadtxt(csv_path,delimiter=",")[:,7:9]
    err = np.loadtxt(csv_path,delimiter=",")[:,9]
    projected_keypoint = np.loadtxt(csv_path,delimiter=",")[:,11:13]
    points2d = np.c_[points2d,np.ones(points2d.shape[0])]
    return points2d, points2d_target_projected_to_another, points2d_target_reprojected_to_another,err, projected_keypoint
#    return points2d

def getReprjPoints2DFromCSV(csv_path):
    points2d = np.loadtxt(csv_path,delimiter=",")[:,2:4]
#    points2d_target_projected_to_another = np.loadtxt(csv_path,delimiter=",")[:,4:7]
    points2d = np.c_[points2d,np.ones(points2d.shape[0])]
#    return points2d, points2d_target_projected_to_another
    return points2d

'''
def calcCalibrationError(_points2d_left, _points3d_right, reproj2d_left, points2d_keypoint,  P_left, P_right, P_left_inv, P_right_inv, rot_mat, translation, err, projected_keypoint):
    points2d_left = copy.deepcopy(_points2d_left)
    points3d_right = copy.deepcopy(_points3d_right)

    points3d_right_convd = points3d_right.dot(rot_mat.T) + translation
    points3d_right_convd[:,0] /= points3d_right_convd[:,2]
    points3d_right_convd[:,1] /= points3d_right_convd[:,2]
    points3d_right_convd[:,2] /= points3d_right_convd[:,2]
    points2d_left_reprojected = points3d_right_convd.dot(P_left.T)

    #rr = Rot.from_dcm(rot_mat.T)
    #test = rr.as_euler("yxz", degrees=True)
    diffs = points2d_left[:,:2] - points2d_left_reprojected[:,:2]
    error = np.mean(np.sqrt(np.sum(diffs**2,axis=1)))
    return error
'''

def calcCalibrationError(_points2d_left, _points3d_right,  P_left, P_right, P_left_inv, P_right_inv, rot_mat, translation):
    points2d_left = copy.deepcopy(_points2d_left)
    points3d_right = copy.deepcopy(_points3d_right)

    points3d_right_convd = points3d_right.dot(rot_mat.T) + translation
    points3d_right_convd[:,0] /= points3d_right_convd[:,2]
    points3d_right_convd[:,1] /= points3d_right_convd[:,2]
    points3d_right_convd[:,2] /= points3d_right_convd[:,2]
    points2d_left_reprojected = points3d_right_convd.dot(P_left.T)

    #rr = Rot.from_dcm(rot_mat.T)
    #test = rr.as_euler("yxz", degrees=True)
    diffs = points2d_left[:,:2] - points2d_left_reprojected[:,:2]
    #error = np.mean(np.sqrt(np.sum(diffs**2,axis=1)))
    error = np.median(np.sqrt(np.sum(diffs**2,axis=1)))
    return error


def minimize_func(param, merged_points2d_left, merged_points3d_right, P_left, P_right, P_left_inv, P_right_inv, rot_mat):
    translation = param
    return calcCalibrationError(merged_points2d_left, merged_points3d_right,  P_left, P_right, P_left_inv, P_right_inv, rot_mat, translation)

def merge_pcds(pcd_list):
    all_points = []
    for pcd in pcd_list:
        all_points.append(np.asarray(pcd.points))
    merged_pcd = PointCloud()
    merged_pcd.points = Vector3dVector(np.vstack(all_points))
    return merged_pcd


def main():
    toml_params = toml.load(open(TOML_PATH))

    fx_left = toml_params["CameraParameter"]["fx_left"]
    fy_left = toml_params["CameraParameter"]["fy_left"]
    cx_left = toml_params["CameraParameter"]["cx_left"]
    cy_left = toml_params["CameraParameter"]["cy_left"]
    
    fx_right = toml_params["CameraParameter"]["fx_right"]
    fy_right = toml_params["CameraParameter"]["fy_right"]
    cx_right = toml_params["CameraParameter"]["cx_right"]
    cy_right = toml_params["CameraParameter"]["cy_right"]
    
    P_left = np.zeros([3,3])
    P_left[0,0] = fx_left
    P_left[1,1] = fy_left
    P_left[0,2] = cx_left
    P_left[1,2] = cy_left
    P_left[2,2] = 1
    P_left_inv = np.linalg.inv(P_left)
    
    P_right = np.zeros([3,3])
    P_right[0,0] = fx_right
    P_right[1,1] = fy_right
    P_right[0,2] = cx_right
    P_right[1,2] = cy_right
    P_right[2,2] = 1
    P_right_inv = np.linalg.inv(P_right)

    left_csv_pathes = glob.glob(osp.join(CSV_PATH , "cam0*.csv"))
    left_csv_pathes = np.sort(left_csv_pathes)
    right_csv_pathes = glob.glob(osp.join(CSV_PATH , "cam1*.csv"))
    right_csv_pathes = np.sort(right_csv_pathes)

    f_yaml = open(YAML_PATH, "r")
    data = yaml.load(f_yaml)
    transform_mat_left2right = np.asarray(data["cam1"]["T_cn_cnm1"])
    #rot_mat = np.asarray(data["cam1"]["T_cn_cnm1"])[:3,:3]
    #translation = np.asarray(data["cam1"]["T_cn_cnm1"])[:3,3]
    #r = Rot.from_dcm(rot_mat)
    transform_mat_right2left = np.linalg.inv(transform_mat_left2right)
    rot_mat_right2left = transform_mat_right2left[:3,:3]
    translation_right2left = transform_mat_right2left[:3,3]

    points2d_left_list = []
    points3d_right_list = []
    for i in range(len(right_csv_pathes)):
        if len(np.loadtxt(left_csv_pathes[i], delimiter=","))==0:
                continue
        #points2d_left, points2d_target_projected_to_right = getPoints2DFromCSV(left_csv_pathes[i])
        points2d_left, points3d_right, points2d_keypoint, errs, projected_keypoint = getPoints2DFromCSV(left_csv_pathes[i])
        #points2d_left_reproj = getReprjPoints2DFromCSV(left_csv_pathes[i])
        
#        points2d_right = getPoints2DFromCSV(right_csv_pathes[i])
        #calcCalibrationError(points2d_left, points2d_target_projected_to_right, P_left, P_right, P_left_inv, P_right_inv, rot_mat, translation)
        #print(calcCalibrationError(points2d_left, points2d_right, points2d_left_reproj, points2d_keypoint, P_left, P_right, P_left_inv, P_right_inv, rot_mat_right2left, translation_right2left, errs, projected_keypoint))
        points2d_left_list.append(points2d_left)
        points3d_right_list.append(points3d_right)

    merged_points2d_left= np.vstack(points2d_left_list)
    merged_points3d_right= np.vstack(points3d_right_list)
    x_init = translation_right2left
    arg = (merged_points2d_left, merged_points3d_right,P_left, P_right, P_left_inv, P_right_inv, rot_mat_right2left,)
    res = minimize(minimize_func, x0=x_init, args=arg, method='COBYLA', options={'xtol': 1e-10, 'disp': False, 'maxiter':10000})
    print("final result:%f"%(res.fun))
    translation_right2left_refined = res.x
    #print(calcCalibrationError(merged_points2d_left, merged_points3d_right, P_left, P_right, P_left_inv, P_right_inv, rot_mat_right2left, translation_right2left))
    return translation_right2left_refined
        

if __name__ == "__main__":
    res = main()
    print(res)

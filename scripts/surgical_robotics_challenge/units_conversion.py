import numpy as np
from scipy.spatial.transform import Rotation as SciPyRot
from surgical_robotics_challenge.kinematics.DH import JointType

class SimToSI:
    linear_factor = 1.0
    angular_factor = 1.0

# Globals to convert between units
def get_pos(ambf_obj):
    pos = ambf_obj.get_pos()
    # Returns a numpy array [x, y, z] scaled by the factor
    return np.array([pos.x, pos.y, pos.z]) / SimToSI.linear_factor

def get_rotation_matrix(ambf_obj):
    # ambf_obj.get_rpy() returns [r, p, y]
    rpy = np.array(ambf_obj.get_rpy()) / SimToSI.angular_factor
    # Convert RPY to a 3x3 Rotation Matrix
    return SciPyRot.from_euler('xyz', rpy).as_matrix()

def get_pose(ambf_obj):
    # Returns a 4x4 transformation matrix
    rot = get_rotation_matrix(ambf_obj)
    pos = get_pos(ambf_obj)
    
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = pos
    return mat

def set_pos(ambf_obj, pos):
    # pos can be a list or numpy array [x, y, z]
    scaled_pos = np.array(pos) * SimToSI.linear_factor
    ambf_obj.set_pos(scaled_pos[0], scaled_pos[1], scaled_pos[2])

def set_rpy(ambf_obj, r, p, y):
    ambf_obj.set_rpy(r * SimToSI.angular_factor, 
                     p * SimToSI.angular_factor, 
                     y * SimToSI.angular_factor)

# --- Joint Helpers remain mostly the same but use numpy logic ---

def get_joint_factor(joint_type):
    if joint_type == JointType.PRISMATIC:
        return SimToSI.linear_factor
    elif joint_type == JointType.REVOLUTE:
        return SimToSI.angular_factor
    else:
        raise ValueError('ERROR! JOINT TYPE INVALID')

def get_joint_pos(ambf_obj, idx, joint_type):
    return ambf_obj.get_joint_pos(idx) / get_joint_factor(joint_type)

def set_joint_pos(ambf_obj, idx, joint_type, cmd):
    return ambf_obj.set_joint_pos(idx, cmd * get_joint_factor(joint_type))

def get_joint_vel(ambf_obj, idx, joint_type):
    return ambf_obj.get_joint_vel(idx) / get_joint_factor(joint_type)

def set_joint_vel(ambf_obj, idx, joint_type, cmd):
    return ambf_obj.set_joint_vel(idx, cmd * get_joint_factor(joint_type))

'''from PyKDL import Vector, Rotation, Frame
from surgical_robotics_challenge.kinematics.DH import JointType


class SimToSI:
    linear_factor = 1.0
    angular_factor = 1.0


# Globals to convert between units
def get_pos(ambf_obj):
    v = Vector(ambf_obj.get_pos().x, ambf_obj.get_pos().y, ambf_obj.get_pos().z)
    return v / SimToSI.linear_factor


def get_rotation(ambf_obj):
    return Rotation.RPY(ambf_obj.get_rpy()[0] / SimToSI.angular_factor,
                        ambf_obj.get_rpy()[1] / SimToSI.angular_factor,
                        ambf_obj.get_rpy()[2] / SimToSI.angular_factor)


def get_pose(ambf_obj):
    return Frame(get_rotation(ambf_obj), get_pos(ambf_obj))


def set_pos(ambf_obj, pos):
    pos = pos * SimToSI.linear_factor
    ambf_obj.set_pos(pos[0], pos[1], pos[2])


def set_rpy(ambf_obj, r, p, y):
    r = r * SimToSI.angular_factor
    p = p * SimToSI.angular_factor
    y = y * SimToSI.angular_factor
    ambf_obj.set_rpy(r, p, y)


def get_joint_factor(joint_type):
    if joint_type == JointType.PRISMATIC:
        factor = SimToSI.linear_factor
    elif joint_type == JointType.REVOLUTE:
        factor = SimToSI.angular_factor
    else:
        raise 'ERROR! JOINT TYPE INVALID'
    return factor


def get_joint_pos(ambf_obj, idx, joint_type):
    factor = get_joint_factor(joint_type)
    return ambf_obj.get_joint_pos(idx) / factor


def set_joint_pos(ambf_obj, idx, joint_type, cmd):
    factor = get_joint_factor(joint_type)
    return ambf_obj.set_joint_pos(idx, cmd * factor)


def get_joint_vel(ambf_obj, idx, joint_type):
    factor = get_joint_factor(joint_type)
    return ambf_obj.get_joint_vel(idx) / factor


def set_joint_vel(ambf_obj, idx, joint_type, cmd):
    factor = get_joint_factor(joint_type)
    return ambf_obj.set_joint_vel(idx, cmd * factor)'''

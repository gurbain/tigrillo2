"""
This script allows the user to create SDF and MJCF robot files
that will be used to realize a simulation with Bullet, MuJoCo
or Gazebo. USE WITH PYTHON3 ONLY!!
"""


from copy import copy
from lxml import etree
import math
import numpy as np

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "January 25th, 2018"


# MODEL PARAMETERS
model_config = {
    'p': 50.0,
    'i': 0.01,
    'd': 0.01,
    'default_height': 0.17,
    'default_density': 100000,
    'body': {
        'front': {
            'width': 0.14,
            'height': 0.03,
            'length': 0.08,
            'mass': 0.3,
        },
        'middle': {
            'width': 0.05,
            'height': 0.0025,
            'length': 0.08,
            'mass': 0.03,
        },
        'hind': {
            'width': 0.09,
            'height': 0.025,
            'length': 0.06,
            'mass': 0.1,
        },
    },
    'battery_mass': 0.117,
    'legs': {
        'FL': {
            'motor': {
                'width': 0.036,
                'length': 0.036,
                'height': 0.0506,
                'leg_attachment_height': 0.033,
                'mass': 0.067
            },
            'foot': {
                'radius': 0.006,
                'mu1': 15000,
                'mu2': 15000,
                'contact_depth': 0.0005,
            },
            'radius': 0.004,
            'joint_slack': 0.0005,
            'position': 'front',
            'femur_length': 0.07,
            'femur_angle': 0.25,
            'tibia_length': 0.09,
            'spring_length': 0.025,
            'femur_spring_tibia_joint_dst': 0.045,
            'tibia_spring_to_joint_dst': 0.035,
            'hip_damping': 0.05,
            'knee_damping': 0.01,
            'spring_stiffness': 6,
            'spring_comp_tol': 0.86,
            'actuator_kp': 254,
        },
        'FR': {
            'motor': {
                'width': 0.036,
                'length': 0.036,
                'height': 0.0506,
                'leg_attachment_height': 0.033,
                'mass': 0.067
            },
            'foot': {
                'radius': 0.006,
                'mu1': 15000,
                'mu2': 15000,
                'contact_depth': 0.0005,
            },
            'radius': 0.004,
            'joint_slack': 0.0005,
            'position': 'front',
            'femur_length': 0.07,
            'femur_angle': 0.25,
            'tibia_length': 0.09,
            'spring_length': 0.025,
            'femur_spring_tibia_joint_dst': 0.045,
            'tibia_spring_to_joint_dst': 0.035,
            'hip_damping': 0.05,
            'knee_damping': 0.01,
            'spring_stiffness': 12,
            'spring_comp_tol': 0.985,
            'actuator_kp': 254,
        },
        'BL': {
            'motor': {
                'width': 0.036,
                'length': 0.036,
                'height': 0.0506,
                'leg_attachment_height': 0.033,
                'mass': 0.067
            },
            'foot': {
                'radius': 0.006,
                'mu1': 15000,
                'mu2': 15000,
                'contact_depth': 0.0005,
            },
            'radius': 0.004,
            'joint_slack': 0.0005,
            'position': 'front',
            'femur_length': 0.07,
            'femur_angle': 0.25,
            'tibia_length': 0.09,
            'spring_length': 0.025,
            'femur_spring_tibia_joint_dst': 0.045,
            'tibia_spring_to_joint_dst': 0.035,
            'hip_damping': 0.05,
            'knee_damping': 0.08,
            'spring_stiffness': 10.1,
            'spring_comp_tol': 0.863,
            'actuator_kp': 254,
        },
        'BR': {
            'motor': {
                'width': 0.036,
                'length': 0.036,
                'height': 0.0506,
                'leg_attachment_height': 0.033,
                'mass': 0.067
            },
            'foot': {
                'radius': 0.006,
                'mu1': 15000,
                'mu2': 15000,
                'contact_depth': 0.0005,
            },
            'radius': 0.004,
            'joint_slack': 0.0005,
            'position': 'front',
            'femur_length': 0.07,
            'femur_angle': 0.25,
            'tibia_length': 0.09,
            'spring_length': 0.025,
            'femur_spring_tibia_joint_dst': 0.045,
            'tibia_spring_to_joint_dst': 0.035,
            'hip_damping': 0.05,
            'knee_damping': 0.12,
            'spring_stiffness': 11.1,
            'spring_comp_tol': 0.94,
            'actuator_kp': 254,
        },
    },
}


class SDFileGenerator(object):
    """
    Generate a SDF XML model file
    """

    def __init__(self, model_config, filename='model.sdf', gazebo=False, model_scale=1, meshes=True, VERSION=2):

        # Options
        self.model_config = model_config
        self.filename = filename
        self.model_scale = model_scale
        self.VERSION = VERSION
        self.gazebo = gazebo
        self.mesh = meshes

        # XML anchors
        self.xml_world = None
        self.xml_sdf = None
        self.xml_model = None
        self.head_array = []
        self.model_configs_array = []
        self.links_array = []
        self.joints_array = []
        self.plugins_array = []

        self.red = "1 0 0 1"
        self.green = "0 1 0 1"
        self.blue = "0 0 1 1"
        self.white = "1 1 1 1"
        self.black = "0 0 0 1"

        # Parameters
        self.z_offset = self.model_config["default_height"] / self.model_scale
        self.density = self.model_config["default_density"]
        self.front_y = (self.model_config["body"]["middle"]["length"] +
                        self.model_config["body"]["front"]["length"]) / 2 / self.model_scale
        self.back_y = - (self.model_config["body"]["middle"]["length"] +
                         self.model_config["body"]["hind"]["length"]) / 2 / self.model_scale
        self.front_x = self.model_config["body"]["front"]["width"] / 2 / self.model_scale
        self.back_x = self.model_config["body"]["hind"]["width"] / 2 / self.model_scale
        self.back_knee_y = 0
        self.back_knee_z = 0
        self.front_knee_y = 0
        self.front_knee_z = 0
        self.knee_angle = 0

    def generate_xml_header(self):
        """ Generate the XML for all the header fields """

        # Yet nothing here

    def generate_xml_model_configs(self):
        """ Generate the XML for the model config """

        xml_static = etree.Element('static')
        xml_static.text = "False"

        xml_collide = etree.Element('self_collide')
        xml_collide.text = "False"

        self.model_configs_array.extend([xml_static, xml_collide])

    def _create_body(self):
        """ Generate the XML for the body link """

        body = []

        for part in ["front", "middle", "hind"]:

            name = "body_" + part

            w = self.model_config["body"][part]["width"] / self.model_scale
            h = self.model_config["body"][part]["height"] / self.model_scale
            l = self.model_config["body"][part]["length"] / self.model_scale
            # vol = w * l * h
            m = self.model_config["body"][part]["mass"] / (self.model_scale * self.model_scale * self.model_scale)
            ih = m / 12 * (l**2 + w**2)  # z
            iw = m / 12 * (h**2 + l**2)  # x
            il = m / 12 * (h**2 + w**2)  # y

            elem = etree.Element('link', name=name)
            col = etree.Element('collision', name=name + "col")
            vis = etree.Element('visual', name=name + "vis")
            pose_elem = etree.Element('pose')
            inertial = etree.Element('inertial')
            mass = etree.Element('mass')
            inertia = etree.Element('inertia')
            ixx = etree.Element('ixx')
            ixy = etree.Element('ixy')
            ixz = etree.Element('ixz')
            iyy = etree.Element('iyy')
            iyz = etree.Element('iyz')
            izz = etree.Element('izz')

            geometry = etree.Element('geometry')

            box = etree.Element('box')
            size = etree.Element('size')
            material = etree.Element('material')
            mat_script = etree.Element('script')
            mat_uri = etree.Element('uri')
            mat_name = etree.Element('name')
            pose_vis = etree.Element('pose')
            geometry_vis = etree.Element('geometry')
            mesh = etree.Element('mesh')
            uri = etree.Element('uri')
            scale = etree.Element('scale')

            size.text = str(w) + " " + str(l) + " " + str(h)
            mass.text = str(m)
            ixx.text = str(iw)
            ixy.text = str(0)
            ixz.text = str(0)
            iyy.text = str(il)
            iyz.text = str(0)
            izz.text = str(ih)

            mat_uri.text = "file://media/materials/scripts/gazebo.material"
            uri.text = "model://tigrillo/meshes/" + part + ".dae"
            
            if part == "middle":
                pose_elem.text = "0 0 " + str(h / 2 + self.z_offset) + " 0 0 0"
                pose_vis.text = "0 0 " + str(-0.004/self.model_scale) + \
                                " 0 0 " + str(-math.pi/2)
                scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(1.0 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
                mat_name.text = "Gazebo/FlatBlack"

            elif part == "front":
                pose_elem.text = "0 " + str(self.front_y) + " " + str(h / 2 + self.z_offset) + " 0 0 0"
                pose_vis.text = "0 " + str(0.018/self.model_scale) + " " +  str(-0.018/self.model_scale) + \
                                " 0 0 " + str(-math.pi/2)
                scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(1.0 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
                mat_name.text = "Gazebo/Orange"

            else:
                pose_elem.text = "0 " + str(self.back_y) + " " + str(h / 2 + self.z_offset) + " 0 0 0"
                pose_vis.text = "0 0 " + str(-0.016/self.model_scale) + \
                                " 0 0 " + str(-math.pi/2)
                scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(0.9 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
                mat_name.text = "Gazebo/Orange"

            inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
            inertial.extend([mass, inertia])
            box.extend([size])
            geometry.extend([box])
            col.extend([geometry])
            mat_script.extend([mat_uri, mat_name])
            material.extend([mat_script])
            if not self.mesh:
                vis.extend([copy(geometry), material])
            else:
                mesh.extend([uri, scale])
                geometry_vis.extend([mesh])
                vis.extend([pose_vis, geometry_vis, material]) 
            elem.extend([pose_elem, inertial, col, vis])           
           

            # Add a fixed joint
            if part != "middle":
                joint = etree.Element("joint", name=name + "_J", type="fixed")
                j_parent = etree.Element("parent")
                j_child = etree.Element("child")
                j_parent.text = "body_middle"
                j_child.text = name
                joint.extend([j_parent, j_child])
                self.joints_array.append(joint)

            body.extend([elem])

        return body

    def _create_leg_motor(self, name, leg_id, config):
        """ Generate the XML for a the down leg link """

        name = name + "M"

        motor_w = config["motor"]["width"] / self.model_scale   # x
        motor_l = config["motor"]["length"] / self.model_scale  # y
        motor_h = config["motor"]["height"] / self.model_scale  # z
        m = config["motor"]["mass"] / (self.model_scale * self.model_scale * self.model_scale)
        # vol = motor_w * motor_l * motor_h

        if leg_id == 1 or leg_id == 2:  # Front
            y = self.front_y
            x = (-1)**(leg_id % 2) * (self.front_x - motor_w / 2)
        else:  # Back
            y = self.back_y
            x = (-1)**(leg_id % 2) * (self.back_x - motor_w / 2)
        z = - motor_h / 2 + self.z_offset

        iw = m/12 * (motor_h**2 + motor_l**2)  # x
        il = m/12 * (motor_h**2 + motor_w**2)  # y
        ih = m/12 * (motor_l**2 + motor_w**2)  # z

        motor = etree.Element('link', name=name)
        pose = etree.Element('pose')
        inertial = etree.Element('inertial')
        mass = etree.Element('mass')
        inertia = etree.Element('inertia')
        ixx = etree.Element('ixx')
        ixy = etree.Element('ixy')
        ixz = etree.Element('ixz')
        iyy = etree.Element('iyy')
        iyz = etree.Element('iyz')
        izz = etree.Element('izz')
        collision = etree.Element('collision', name=name + "_col")
        geometry = etree.Element('geometry')
        box = etree.Element('box')
        size = etree.Element('size')
        visual = etree.Element('visual', name=name + "_vis")
        material = etree.Element('material')
        mat_script = etree.Element('script')
        mat_uri = etree.Element('uri')
        mat_name = etree.Element('name')
        if self.mesh:
            geometry_vis = etree.Element('geometry')
            pose_vis = etree.Element('pose')
            mesh = etree.Element('mesh')
            uri = etree.Element('uri')
            scale = etree.Element('scale')

        mat_uri.text = "file://media/materials/scripts/gazebo.material"
        if self.mesh:
            mat_name.text = "Gazebo/Orange"
            pose_vis.text = "0 0 0 " + str(-math.pi/2) + " 0 " + str((-1)**((leg_id+1) % 2) * math.pi/2)# + " 0"
            uri.text = "model://tigrillo/meshes/motor.dae"
            scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(1.0 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
        else:
            mat_name.text = "Gazebo/Green"

        pose.text = str(x) + " " + str(y) + " " + str(z) + " 0 0 0"
        size.text = str(motor_w) + " " + str(motor_l) + " " + str(motor_h)
        mass.text = str(m)
        ixx.text = str(iw)
        ixy.text = str(0)
        ixz.text = str(0)
        iyy.text = str(il)
        iyz.text = str(0)
        izz.text = str(ih)

        inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
        inertial.extend([mass, inertia])
        box.extend([size])
        geometry.extend([box])
        mat_script.extend([mat_uri, mat_name])
        material.extend([mat_script])
        if not self.mesh:
            visual.extend([geometry, material])
        else:
            mesh.extend([uri, scale])
            geometry_vis.extend([mesh])
            visual.extend([pose_vis, geometry_vis, material])
        collision.extend([copy(geometry)])
        motor.extend([pose, inertial, collision, visual])

        # Joint
        joint_m = etree.Element("joint", name=name + "J", type="fixed")
        jm_parent = etree.Element("parent")
        jm_child = etree.Element("child")
        if leg_id == 1 or leg_id == 2:  # Front
            jm_parent.text = "body_front"
        else:  # Back
            jm_parent.text = "body_hind"
        jm_child.text = name
        joint_m.extend([jm_parent, jm_child])
        self.joints_array.append(joint_m)

        return motor

    def _create_leg_up(self, name, leg_id, config):
        """ Generate the XML for an upper leg link """

        name_m = name + "M"
        name = name + "U"

        h_attach = config["motor"]["leg_attachment_height"] / self.model_scale  # z
        l = config["femur_length"] / self.model_scale
        r = config["radius"] / self.model_scale

        slack = config["joint_slack"] / self.model_scale
        a = config['femur_angle'] / 360 * 2 * math.pi
        vol = l * math.pi * r**2

        # Attachment point 1
        z_1 = - h_attach + self.z_offset
        if leg_id == 1 or leg_id == 2:  # Front
            y_1 = self.front_y
            x_1 = (-1)**(leg_id % 2) * (self.front_x + r + slack)
        else:  # Back
            y_1 = self.back_y
            x_1 = (-1)**(leg_id % 2) * (self.back_x + r + slack)

        # Center point
        # x = x_1
        # y = y_1 + l * math.sin(a) / 2
        # z = z_1 - l * math.cos(a) / 2

        # Attachment point 2
        if leg_id == 1 or leg_id == 2:  # Front
            self.front_knee_y = y_1 + l * math.sin(a)
            self.front_knee_z = z_1 - l * math.cos(a)
        else:  # Back
            self.back_knee_y = y_1 + l * math.sin(a)
            self.back_knee_z = z_1 - l * math.cos(a)

        m = self.density * vol
        il = m / 2 * r**2
        ir = m / 12 * (3 * r**2 + l**2)
        damp = config['hip_damping']

        femur = etree.Element('link', name=name)
        pose = etree.Element('pose')
        pose_vis_col_in = etree.Element('pose')
        inertial = etree.Element('inertial')
        mass = etree.Element('mass')
        inertia = etree.Element('inertia')
        ixx = etree.Element('ixx')
        ixy = etree.Element('ixy')
        ixz = etree.Element('ixz')
        iyy = etree.Element('iyy')
        iyz = etree.Element('iyz')
        izz = etree.Element('izz')
        collision = etree.Element('collision', name=name + "_col")
        geometry = etree.Element('geometry')
        if self.gazebo:
            cylinder = etree.Element('cylinder')
            radius = etree.Element('radius')
            length = etree.Element('length')
        else:
            cylinder = etree.Element('cylinder', radius=str(r), length=str(l))
        visual = etree.Element('visual', name=name + "_vis")
        material = etree.Element('material')
        mat_script = etree.Element('script')
        mat_uri = etree.Element('uri')
        mat_name = etree.Element('name')
        if  self.mesh:
            geometry_vis = etree.Element('geometry')
            pose_vis = etree.Element('pose')
            mesh = etree.Element('mesh')
            uri = etree.Element('uri')
            scale = etree.Element('scale')

        mat_uri.text = "file://media/materials/scripts/gazebo.material"
        if  self.mesh:
            mat_name.text = "Gazebo/Turquoise"
            pose_vis.text = str(-0.004 / self.model_scale) + "0 0 0 0 " + str(math.pi/2) + " 0"
            uri.text = "model://tigrillo/meshes/femur.dae"
            scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(1.0 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
        else:
            mat_name.text = "Gazebo/Black"

        pose_vis_col_in.text = "0 0 " + str(-l / 2) + " 0 0 0"
        pose.text = str(x_1) + " " + str(y_1) + " " + str(z_1) + " " + str(a) + " 0 0"
        mass.text = str(m)
        ixx.text = str(ir)
        ixy.text = str(0)
        ixz.text = str(0)
        iyy.text = str(ir)
        iyz.text = str(0)
        izz.text = str(il)
       
        if self.gazebo:
            radius.text = str(r)
            length.text = str(l)
            cylinder.extend([radius, length])

        inertia.extend([ixx, ixy, ixz, iyy, iyz, izz])
        inertial.extend([copy(pose_vis_col_in), mass, inertia])
        geometry.extend([cylinder])
        mat_script.extend([mat_uri, mat_name])
        material.extend([mat_script])
        if not self.mesh:
            visual.extend([copy(pose_vis_col_in), geometry, material])
        else:
            mesh.extend([uri, scale])
            geometry_vis.extend([mesh])
            visual.extend([pose_vis, geometry_vis, material])
        collision.extend([copy(pose_vis_col_in), copy(geometry)])
        femur.extend([pose, inertial, collision, visual])

        # Joint
        joint = etree.Element("joint", name=name + "J", type="revolute")
        j_parent = etree.Element("parent")
        j_child = etree.Element("child")
        j_axis = etree.Element("axis")
        j_xyz = etree.Element("xyz")
        j_dynamics = etree.Element("dynamics")
        j_damping = etree.Element("damping")
        j_limit = etree.Element("limit")
        j_lower = etree.Element("lower")
        j_upper = etree.Element("upper")

        j_parent.text = name_m
        j_child.text = name
        j_xyz.text = "1 0 0"
        j_damping.text = str(damp)
        j_lower.text = str(-math.pi / 2 - a)
        j_upper.text = str(math.pi / 2 - a)

        j_limit.extend([j_lower, j_upper])
        j_axis.extend([j_xyz, j_limit])
        joint.extend([j_parent, j_child, j_axis])
        self.joints_array.append(joint)

        return femur

    def _create_leg_down(self, name, leg_id, config):
        """ Generate the XML for a down leg link """

        name_u = name + "U"
        name = name + "D"

        # Tibia values
        e = config["spring_length"] / self.model_scale
        f = config["femur_spring_tibia_joint_dst"] / self.model_scale
        g = config["tibia_spring_to_joint_dst"] / self.model_scale
        a_femur = config['femur_angle'] / 360 * 2 * math.pi
        l = config["tibia_length"] / self.model_scale
        r = config["radius"] / self.model_scale
        slack = config["joint_slack"] / self.model_scale
        compression_tol = config["spring_comp_tol"]
        a_radius = math.acos((f**2 + g**2 - e**2) / (2 * f * g))
        a = a_femur + a_radius
        self.knee_angle = a
        k_spring = config["spring_stiffness"]
        vol = l * math.pi * r**2
        m = self.density * vol
        il = m / 2 * r**2
        ir = m / 12 * (3 * r**2 + l**2)
        damp = config['knee_damping']

        # Foot values
        foot_r = config["foot"]["radius"] / self.model_scale
        # foot_vol = 4 * math.pi * r**3 / 3
        # foot_m = self.density * vol
        mu1 = config["foot"]["mu1"]
        mu2 = config["foot"]["mu2"]
        contact_depth = config["foot"]["contact_depth"]

        # Attachment point
        if leg_id == 1 or leg_id == 2:  # Front
            x_1 = (-1)**(leg_id % 2) * (self.front_x + r + slack)
            y_1 = self.front_knee_y
            z_1 = self.front_knee_z
        else:  # Back
            x_1 = (-1)**(leg_id % 2) * (self.back_x + r + slack)
            y_1 = self.back_knee_y
            z_1 = self.back_knee_z

        # Center point
        # x = x_1
        # y = y_1 + (l / 2 - g) * math.sin(a)
        # z = z_1 - (l / 2 - g) * math.cos(a)

        leg_down = etree.Element('link', name=name)
        pose = etree.Element('pose')
        tibia_pose = etree.Element('pose')
        tibia_inertial = etree.Element('inertial')
        tibia_mass = etree.Element('mass')
        tibia_inertia = etree.Element('inertia')
        tibia_ixx = etree.Element('ixx')
        tibia_ixy = etree.Element('ixy')
        tibia_ixz = etree.Element('ixz')
        tibia_iyy = etree.Element('iyy')
        tibia_iyz = etree.Element('iyz')
        tibia_izz = etree.Element('izz')
        tibia_collision = etree.Element('collision', name=name + "T_col")
        tibia_geometry = etree.Element('geometry')
        if self.gazebo:
            tibia_cylinder = etree.Element('cylinder')
            tibia_radius = etree.Element('radius')
            tibia_length = etree.Element('length')
        else:
            tibia_cylinder = etree.Element('cylinder', radius=str(r), length=str(l))

        tibia_visual = etree.Element('visual', name=name + "T_vis")
        tibia_material = etree.Element('material')
        tibia_mat_script = etree.Element('script')
        tibia_mat_uri = etree.Element('uri')
        tibia_mat_name = etree.Element('name')
        if self.mesh:
            tibia_geometry_vis = etree.Element('geometry')
            tibia_pose_vis = etree.Element('pose')
            tibia_mesh = etree.Element('mesh')
            tibia_uri = etree.Element('uri')
            tibia_scale = etree.Element('scale')


        tibia_mat_uri.text = "file://media/materials/scripts/gazebo.material"
        if self.mesh:
            tibia_mat_name.text = "Gazebo/Turquoise"
            tibia_pose_vis.text = str(0.004 / self.model_scale) + " 0 " + str(-l + 1.2 * g) + str(-math.pi/2) + " 0 " + str(math.pi/2) #+ " 0" #str(0.004 / self.model_scale) + " 0 0 0 " + str(-math.pi/2) + " -1.5"# + str(math.pi/2)
            tibia_uri.text = "model://tigrillo/meshes/tibia.dae"
            tibia_scale.text = str(1.0 / (1000 * self.model_scale)) + " " + str(1.0 / (1000 * self.model_scale)) + \
                         " " + str(1.0 / (1000 * self.model_scale))
        else:
            tibia_mat_name.text = "Gazebo/Blue"

        pose.text = str(x_1) + " " + str(y_1) + " " + str(z_1) + " " + str(a) + " 0 0"
        tibia_pose.text = "0 0 " + str(-l /2 + g) + " 0 0 0"
        tibia_mass.text = str(m)
        tibia_ixx.text = str(ir)
        tibia_ixy.text = str(0)
        tibia_ixz.text = str(0)
        tibia_iyy.text = str(ir)
        tibia_iyz.text = str(0)
        tibia_izz.text = str(il)
       
        if self.gazebo:
            tibia_radius.text = str(r)
            tibia_length.text = str(l)
            tibia_cylinder.extend([tibia_radius, tibia_length])

        tibia_inertia.extend([tibia_ixx, tibia_ixy, tibia_ixz, tibia_iyy, tibia_iyz, tibia_izz])
        tibia_inertial.extend([copy(tibia_pose), tibia_mass, tibia_inertia])


        tibia_geometry.extend([tibia_cylinder])
        tibia_mat_script.extend([tibia_mat_name, tibia_mat_uri])
        tibia_material.extend([tibia_mat_script])
        if not self.mesh: 
            tibia_visual.extend([copy(tibia_pose), tibia_geometry, tibia_material])
        else:
            tibia_mesh.extend([tibia_uri, tibia_scale])
            tibia_geometry_vis.extend([tibia_mesh])
            tibia_visual.extend([tibia_pose_vis, tibia_geometry_vis, tibia_material])
        tibia_collision.extend([copy(tibia_pose), copy(tibia_geometry)])

        # Foot
        foot_pose = etree.Element('pose')
        foot_collision = etree.Element('collision', name=name + "F_col")
        foot_geometry = etree.Element('geometry')
        if self.gazebo:
            foot_sphere = etree.Element('sphere')
            foot_radius = etree.Element('radius')
        else:
            foot_sphere = etree.Element('cylinder', radius=str(foot_r))
        foot_surface = etree.Element("surface")
        foot_friction = etree.Element("friction")
        foot_contact = etree.Element("contact")
        foot_ode2 = etree.Element("ode")
        foot_min_depth = etree.Element("min_depth")
        foot_ode = etree.Element("ode")
        foot_mu1 = etree.Element("mu")
        foot_mu2 = etree.Element("mu2")
        foot_material = etree.Element('material')
        foot_mat_script = etree.Element('script')
        foot_mat_uri = etree.Element('uri')
        foot_mat_name = etree.Element('name')

        if not self.mesh: 
            foot_visual = etree.Element('visual', name=name + "F_vis")

        foot_mat_uri.text = "file://media/materials/scripts/gazebo.material"
        if self.mesh:
            foot_mat_name.text = "Gazebo/Black"
        else:
            foot_mat_name.text = "Gazebo/Blue"
        foot_pose.text = " 0 0 " + str(-l+g) + " 0 0 0"
        foot_mu1.text = str(mu1)
        foot_mu2.text = str(mu2)
        foot_min_depth.text = str(contact_depth)
        if self.gazebo:
            foot_radius.text = str(foot_r)
            foot_sphere.extend([foot_radius])

        foot_ode2.extend([foot_min_depth])
        foot_contact.extend([foot_ode2])
        foot_ode.extend([foot_mu1, foot_mu2])
        foot_friction.extend([foot_ode])
        foot_surface.extend([foot_friction, foot_contact])
        foot_geometry.extend([foot_sphere])
        foot_collision.extend([copy(foot_pose), copy(foot_geometry), copy(foot_surface)])
        foot_mat_script.extend([foot_mat_name, foot_mat_uri])
        foot_material.extend([foot_mat_script])
        if not self.mesh:
            foot_visual.extend([copy(foot_pose),  copy(foot_geometry),  copy(foot_material)])
            leg_down.extend([pose, tibia_inertial, tibia_collision, tibia_visual, foot_collision, foot_visual])
        else:
            leg_down.extend([pose, tibia_inertial, tibia_collision, tibia_visual, foot_collision])

        # Joint
        joint = etree.Element("joint", name=name + "J", type="revolute")
        j_parent = etree.Element("parent")
        j_child = etree.Element("child")
        j_axis = etree.Element("axis")
        j_xyz = etree.Element("xyz")
        j_dynamics = etree.Element("dynamics")
        j_damping = etree.Element("damping")
        j_spring_stiffness = etree.Element("spring_stiffness")
        j_limit = etree.Element("limit")
        j_lower = etree.Element("lower")
        j_upper = etree.Element("upper")

        j_parent.text = name_u
        j_child.text = name
        j_xyz.text = "1 0 0"
        j_damping.text = str(damp)
        j_spring_stiffness.text = str(k_spring * g * math.acos((e**2 + g**2 - f**2) / (2 * e * g)))
        j_lower.text = str(compression_tol - a)
        j_upper.text = str(math.pi/2)

        j_dynamics.extend([j_damping, j_spring_stiffness])
        j_limit.extend([j_lower, j_upper])
        j_axis.extend([j_xyz, j_dynamics, j_limit])
        joint.extend([j_parent, j_child, j_axis])
        self.joints_array.append(joint)

        return leg_down
 
    def generate_xml_links_and_joints(self):
        """ Generate the XML for the model links and joints"""

        body_array = self._create_body()

        legs_array = []
        for key in self.model_config["legs"]:
            leg_id = 0
            if key.find("FL") == 0:
                leg_id = 1
            if key.find("FR") == 0:
                leg_id = 2
            if key.find("BL") == 0:
                leg_id = 3
            if key.find("BR") == 0:
                leg_id = 4
            legs_array.append(self._create_leg_motor(key, leg_id, self.model_config["legs"][key]))
            legs_array.append(self._create_leg_up(key, leg_id, self.model_config["legs"][key]))
            legs_array.append(self._create_leg_down(key, leg_id, self.model_config["legs"][key]))

        self.links_array.extend(body_array)
        self.links_array.extend(legs_array)

    def generate_xml_plugins(self):
        """ Generate the XML for the model plugins """

        xml_cust_plugin = etree.Element('plugin', name='TigrilloPlugin', filename='libtigrillo_2_plugin.so')

        param_p = etree.Element("p")
        param_i = etree.Element("i")
        param_d = etree.Element("d")

        param_p.text = str(self.model_config["p"])
        param_i.text = str(self.model_config["i"])
        param_d.text = str(self.model_config["d"])

        xml_cust_plugin.append(param_p)
        xml_cust_plugin.append(param_i)
        xml_cust_plugin.append(param_d)

        xml_imu_plugin = etree.Element('plugin', name='ImuPlugin', filename='libgazebo_ros_imu.so')

        param_on = etree.Element("alwaysOn")
        param_body = etree.Element("bodyName")
        param_topic = etree.Element("topicName")
        param_service = etree.Element("serviceName")
        param_gaussian = etree.Element("gaussianNoise")
        param_rate = etree.Element("updateRate")

        param_on.text = "true"
        param_body.text = "body_front"
        param_topic.text = "imu_data"
        param_service.text = "imu_service"
        param_gaussian.text = "0.0"
        param_rate.text = "20.0"

        xml_imu_plugin.append(param_on)
        xml_imu_plugin.append(param_body)
        xml_imu_plugin.append(param_topic)
        xml_imu_plugin.append(param_service)
        xml_imu_plugin.append(param_gaussian)
        xml_imu_plugin.append(param_rate)

        self.plugins_array.extend([xml_cust_plugin, xml_imu_plugin])

    def generate_xml_sdf(self):
        """ Generate the XML for all the sdf model """

        if not self.gazebo:
            self.xml_world = etree.Element('world', name="default")
        self.xml_sdf = etree.Element('sdf', version="1.4")
        if self.mesh:
            self.xml_model = etree.Element("model", name="tigrillo")
        else:
            self.xml_model = etree.Element("model", name="tigrillo_nm")

        self.generate_xml_model_configs()
        self.generate_xml_links_and_joints()
        self.generate_xml_plugins()

        self.xml_model.extend(self.model_configs_array)
        self.xml_model.extend(self.links_array)
        self.xml_model.extend(self.joints_array)

        if self.gazebo:
            self.xml_model.extend(self.plugins_array)
            self.xml_sdf.append(self.xml_model)
        else:
            self.xml_world.append(self.xml_model)
            self.xml_sdf.append(self.xml_world)

    def generate(self):
        """ Main function called to generate a model """

        # Generate all elements
        self.generate_xml_header()
        self.generate_xml_sdf()

        # Create the XML file
        f = open(self.filename, 'w')
        for i, l in enumerate(self.head_array):
            if i == 0:
                f.write(etree.tostring(l, pretty_print=True, xml_declaration=True).decode('utf-8'))
            else:
                f.write(etree.tostring(l, pretty_print=True).decode('utf-8'))
        if len(self.head_array) == 0:
            f.write(etree.tostring(self.xml_sdf, xml_declaration=True, pretty_print=True).decode('utf-8'))
        else:
            f.write(etree.tostring(self.xml_sdf, pretty_print=True).decode('utf-8'))
        f.flush()
        f.close()

    def get_model_generator_version(self):

        return self.VERSION


if __name__ == '__main__':

    # Generate the model with meshes
    gazebo_folder = "/home/gabs48/.gazebo/models/tigrillo/"
    fg3 = SDFileGenerator(model_config, gazebo_folder + "model.sdf", model_scale=1, gazebo=True)
    fg3.generate()

     # Generate the model and without
    gazebo_folder = "/home/gabs48/.gazebo/models/tigrillo_nm/"
    fg3 = SDFileGenerator(model_config, gazebo_folder + "model.sdf", model_scale=1, gazebo=True, meshes=False)
    fg3.generate()
import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
import math as m
import os

class robot_class:
    ''' Creating a class to save the robot parameters and use this 
    to create the URDF fie'''

    def __init__(self, robot_id, link_no):
        ''' 
        Constructor for robot.

        Args: 
            robot_id: ID of the robot object
            link_no: number of robot links
            
            User input:
                link_lengths: length of each link (must be > 0)
                joint_types: list of length (link_no) with 
                         -1 (prismatic) or 0 (fixed) or 1 (revolute)
            Note: joint_types includes first link between robot and world
            '''
        self._id = robot_id
        self._link_no = link_no+1
        self._joint_types = []
        self._link_lengths = [2]
        fixed_ang_dict = {}
        
        for k in range(link_no):
            l_len = -1
            while l_len <= 0:
                l_len = float(input('Enter length of link #{}:'.format(k+1)))

            self._link_lengths.append(l_len)
            self._joint_types.append(1)
            # if j_type == 0:
            #     fixed_ang_dict[k] = float(input('Enter orientation (deg):'))

        self._fixed_angle_orient = fixed_ang_dict.copy()
        self._dens = 100*float(input('Enter density of links:'))
        self._damp = 0.7    # Set to 0.7 by default

    def getLinkNo(self):
        '''
        Get the number of links of the robot. Returns an int'''

        return self._link_no

    def getLinkLengths(self):
        '''
        Returns a list of length (link_no)
        '''
        return self._link_lengths

    def getJointTypes(self):
        '''
        Returns a list of length (link_no)
        0 - revolute
        1 - prismatic
        '''
        return self._joint_types

    def getID(self):
        ''' Get the robot's ID. '''

        return self._id

    def getDensity(self):
        '''
        returns density of the links - used to calculate mass and inertia
        '''
        return self._dens

    def setDensity(self, new_dens):
        '''
        Change density of the links - used to calculate mass and inertia
        '''
        self.dens = new_dens

    def getDamp(self):
        '''
        Change density of the links - used to calculate mass and inertia
        '''
        return self._damp

    def __str__(self):
        '''
        String conversion for easy reading.
        Returns: 
            str: robot description as a string
        '''

        return 'Robot ID {} has {} links connected by joints {}. (Density = {})'.format\
        (self._id, self._link_no, self._joint_types,self._dens)

    def make_dir(self):
        abs_path = os.path.abspath('..')
        try:
            os.makedirs("{0}/urdf/{1}_root".format(abs_path,self.getID()))
        except FileExistsError:
            # directory already exists
            pass

    def xml_space(self,root):
        ## This part of the code is only making the xml fle readable - adds no real value to the file itself - just adds whitespace
        roughString = ElementTree.tostring(root, 'utf-8')
        reparsed = minidom.parseString(roughString)
        finalString = reparsed.toprettyxml(indent="  ")
        return finalString

    def write_file(self,finalString,fileName):
        abs_path = os.path.abspath('..')
        with open('{0}/urdf/{1}_root/{2}'.\
            format(abs_path,self.getID(),fileName),'w') as text_file:
            text_file.write(finalString)

    def write_model(self):
        root = ET.Element("model")
        name = ET.SubElement(root,'name')
        name.text = str(self.getID())
        version = ET.SubElement(root,'version')
        version.text = '1.0'
        sdf = ET.SubElement(root,'sdf')
        sdf.text = 'robot_{}.urdf'.format(self.getID())

        author = ET.SubElement(root,'author')
        a_name = ET.SubElement(author,'name')
        a_name.text = 'UserName'
        a_email = ET.SubElement(author,'email')
        a_email.text = 'username@email.address'

        description = ET.SubElement(root,'description')
        description.text = 'Robot created using robot_class. Serial manipulator with {} links'.format(self.getLinkNo())

        finalString = self.xml_space(root)
        fileName = 'model.config'
        self.write_file(finalString,fileName)
        
    def write_xacro(self):
        # Defining the robot
        root = ET.Element('robot')
        root.set('name','{}_robot'.format(self.getID()))
        root.set('xmlns:xacro','http://www.ros.org/wiki/xacro')

        # Initializing the constants for the robot
        Comment = [ET.Comment('Constants for robot dimensions')]
        root.append(Comment[-1])

        pi = ET.SubElement(root,'xacro:property')
        pi.set('name','PI')
        pi.set('value','3.1415926535897931')

        dens = ET.SubElement(root,'xacro:property')
        dens.set('name','density')
        dens.set('value',str(self._dens))

        axel_offset = ET.SubElement(root,'xacro:property')
        axel_offset.set('name','axel_offset')
        axel_offset.set('value','0.05')

        # Importing Gazebo elements
        find_gazebo = ET.SubElement(root, 'xacro:include')
        ID = self.getID()
        find_gazebo.set('filename','$(find robot_generator)/urdf/{0}_root/{0}_robot.gazebo'.format(ID))

        # Importing materials
        materials = ET.SubElement(root, 'xacro:include')
        materials.set('filename','$(find robot_generator)/urdf/materials.xacro'.format(ID))

        # World link to connect the robot to the Gazebo frame of reference with a fixed joint
        Comment.append(ET.Comment('World link for fixing robot to Gazebo frame'))
        root.append(Comment[-1])

        link = [ET.SubElement(root,'link')]
        link[-1].set('name','world')

        joint = [ET.SubElement(root,'joint')]
        joint[-1].set('name','joint0')
        joint[-1].set('type','fixed')
        parent = [ET.SubElement(joint[-1],'parent')]
        parent[-1].set('link','world')
        child = [ET.SubElement(joint[-1],'child')]
        child[-1].set('link','link1')

        # Creating newly generated robot
        Comment.append(ET.Comment('Start of generated robot'))
        root.append(Comment[-1])

        for k in range(self.getLinkNo()):
            root = self.add_link(root,k)
            if k == 0: continue
            root = self.add_joint(root,k)
            root = self.add_transmission(root,k)

        ID = self.getID()
        finalString = self.xml_space(root)

        # Write file to a urdf file with the robot's ID. Eg. robot_1.urdf
        fileName = '{}_robot.xacro'.format(ID)
        self.write_file(finalString,fileName)


    def add_link(self,root,link_no):
        
        width = 0.1
        link_lengths = self.getLinkLengths()
        link_len = link_lengths[link_no]
        if m.fmod(link_no,2) == 0:
            color = 'orange'
        else:
            color = 'black'
        mass_val = self.getDensity()*width*width*link_len
        if link_no == 0:
            orn_format = '0 0 {}'
        else:
            orn_format = '0 0 ${{{} - axel_offset}}'

        # Define a new link
        link = ET.SubElement(root,'link'.format(link_no))
        link.set('name','link{}'.format(link_no+1))

        # Collision
        collision = ET.SubElement(link,'collision')
        origin = ET.SubElement(collision,'origin')
        origin.set('xyz',orn_format.format(link_len/2))
        origin.set('rpy','0 0 0')

        geometry = ET.SubElement(collision,'geometry')
        box = ET.SubElement(geometry,'box')
        box.set('size','{} {} {}'.format(width,width,link_len))

        # visual
        visual = ET.SubElement(link,'visual')
        origin = ET.SubElement(visual,'origin')
        origin.set('xyz',orn_format.format(link_len/2))
        origin.set('rpy','0 0 0')

        geometry = ET.SubElement(visual,'geometry')
        box = ET.SubElement(geometry,'box')
        box.set('size','{} {} {}'.format(width,width,link_len))

        material = ET.SubElement(visual,'material')
        material.set('name',color)

        # Inertial
        inertial = ET.SubElement(link,'inertial')
        origin = ET.SubElement(inertial,'origin')
        origin.set('xyz',orn_format.format(link_len/2))
        origin.set('rpy','0 0 0')

        mass = ET.SubElement(inertial,'mass')
        mass.set('value','{}'.format(mass_val))

        diag_format = '${{{0} / 12.0 * ({1}*{1} + {2}*{2})}}'
        inertia = ET.SubElement(inertial,'inertia')
        inertia.set('ixx',diag_format.format(mass_val,width,link_len))
        inertia.set('ixy','0.0')
        inertia.set('ixz','0.0')
        inertia.set('iyy',diag_format.format(mass_val,link_len,width))
        inertia.set('iyz','0.0')
        inertia.set('izz',diag_format.format(mass_val,width,width))

        return root

    def add_joint(self,root,link_no):

        width = 0.1
        link_lengths = self.getLinkLengths()
        link_len = link_lengths[link_no-1]
        mass_val = self.getDensity()*width*width*link_len
        if link_no == 0:
            orn_format = '0 {} {}'
        else:
            orn_format = '0 {} ${{{} - axel_offset}}'

        joint = ET.SubElement(root,'joint')
        joint.set('name','joint{}'.format(link_no))
        joint.set('type','continuous')

        parent = ET.SubElement(joint,'parent')
        parent.set('link','link{}'.format(link_no))

        child = ET.SubElement(joint,'child')
        child.set('link','link{}'.format(link_no+1))

        origin = ET.SubElement(joint,'origin')
        origin.set('xyz', orn_format.format(width,link_len))
        origin.set('rpy','0 0 0')

        axis = ET.SubElement(joint, 'axis')
        axis.set('xyz','0 1 0')

        dynamics = ET.SubElement(joint,'dynamics')
        dynamics.set('damping','{}'.format(self.getDamp()))

        return root

    def add_transmission(self,root,link_no):

        transmission = ET.SubElement(root,'transmission')
        transmission.set('name','tran{}'.format(link_no))

        tranType = ET.SubElement(transmission,'type')
        tranType.text = "transmission_interface/SimpleTransmission"

        joint = ET.SubElement(transmission,'joint')
        joint.set('name','joint{}'.format(link_no))

        hardwareInt = ET.SubElement(joint,'hardwareInterface')
        hardwareInt.text = "hardware_interface/EffortJointInterface"

        actuator = ET.SubElement(transmission,'actuator')
        actuator.set('name','motor{}'.format(link_no))

        hardwareInt = ET.SubElement(actuator,'hardwareInterface')
        hardwareInt.text = "hardware_interface/EffortJointInterface"

        mechReduction = ET.SubElement(actuator,'mechanicalReduction')
        mechReduction.text = "1"

        return root
        
    def write_launch(self):

        with open('simulation_template.launch','r') as text_file:
            text = text_file.read()

        joint_format = '            joint{}_position_controller'

        joint_args = ['joint_state_controller']

        for k in range(self.getLinkNo()-1):
            joint_args.append(joint_format.format(k+1))


        text = text.replace('<RobotName>','{}'.format(self.getID()))
        text = text.replace('<JointArgs>','\n'.join(joint_args))

        abs_path = os.path.abspath('..')
        with open('{0}/launch/{1}_simualtion.launch'.\
            format(abs_path,self.getID()),'w') as text_file:
            text_file.write(text)

    def write_control(self):
        finalString = ['{}_robot:'.format(self.getID())]
        finalString.append('  # Publish all joint states')
        finalString.append('  joint_state_controller:')
        finalString.append('    type: joint_state_controller/JointStateController')
        finalString.append('    publish_rate: 50')

        finalString.append('')

        finalString.append('  # Position Controllers')

        for k in range(self.getLinkNo()-1):
            finalString.append('  joint{}_position_controller:'.format(k+1))
            finalString.append('    type: effort_controllers/JointPositionController')
            finalString.append('    joint: joint{}'.format(k+1))
            finalString.append('    pid: {p: 100.0, i: 0.01, d: 10.0}')
            finalString.append('')

        finalString = '\n'.join(finalString)

        abs_path = os.path.abspath('..')
        with open('{0}/config/{1}_robot_control.yaml'.format(abs_path,self.getID()),'w')\
            as yaml_file:
            yaml_file.write(finalString)
    
    def write_info(self):
        finalString = ['{}_robot'.format(self.getID())]
        finalString.append('{}'.format(self.getLinkLengths()))

        finalString = '\n'.join(finalString)

        abs_path = os.path.abspath('..')
        with open('{0}/scripts/{1}_info.txt'.format(abs_path,self.getID()),'w')\
            as yaml_file:
            yaml_file.write(finalString)

    def write_gazebo(self):
        root = ET.Element('robot')
        
        ros_control = ET.SubElement(root,'gazebo')
        plugin = ET.SubElement(ros_control,'plugin')
        plugin.set('name','gazebo_ros_control')
        plugin.set('filename','libgazebo_ros_control.so')

        robotNamespace = ET.SubElement(plugin,'robotNamespace')
        robotNamespace.text = "{}_robot".format(self.getID())

        robotSimType = ET.SubElement(plugin,'robotSimType')
        robotSimType.text = "gazebo_ros_control/DefaultRobotHWSim"

        for k in range(self.getLinkNo()):
            link = ET.SubElement(root,'gazebo')
            link.set('reference','link{}'.format(k+1))
            if m.fmod(k,2) == 0:
                mat = 'Orange'
            else:
                mat = 'Black'
            material = ET.SubElement(link,'material')
            material.text = "Gazebo/{}".format(mat)

        ID = self.getID()
        finalString = self.xml_space(root)

        # Write file to a urdf file with the robot's ID. Eg. robot_1.urdf
        fileName = '{}_robot.gazebo'.format(ID)
        self.write_file(finalString,fileName)

    def create_robot(self):
        self.make_dir()
        self.write_control()
        self.write_xacro()
        self.write_gazebo()
        self.write_launch()
        self.write_info()




    def write_urdf(self):
        root = ET.Element("robot")
        root.set("name", "RobotID{}".format(self.getID()))

        # Initializing variables
        link = []
        joint = []
        visual = []
        collision = []
        inertial = []
        c_geometry = []
        v_geometry = []
        v_set_geom = []
        c_set_geom = []
        l_origin = []
        i_origin = []
        i_mass = []
        i_inertia = []
        j_origin = []
        j_axis = []
        j_parent = []
        j_child = []
        l_base = 1
        j_limit = []
        width = 0.02
        thick = 0.1
        joint_type = {-1:'prismatic',0:'fixed',1:'continuous'}  # Setting up the joint type dictionary

        # Base link - this has to be the first link as per protocol - It is fixed and predefned, sort of as a base of the robot
        link.append(ET.SubElement(root,"link"))
        link[0].set("name","base_link")

        # visual
        visual.append(ET.SubElement(link[0], "visual"))
        v_geometry.append(ET.SubElement(visual[0],"geometry"))
        v_set_geom.append(ET.SubElement(v_geometry[0],"box"))
        v_set_geom[0].set("size","{} {} {}".format(thick, width, l_base))
        l_origin.append(ET.SubElement(visual[0],"origin"))
        l_origin[0].set("rpy","0 0 0")
        l_origin[0].set("xyz","0 0 {}".format(l_base/2))

        # collision
        collision.append(ET.SubElement(link[0],"collision"))
        c_geometry.append(ET.SubElement(collision[0],"geometry"))
        c_set_geom.append(ET.SubElement(c_geometry[0],"box"))
        c_set_geom[0].set("size","{} {} {}".format(thick, width, l_base))
        parent = 'base_link'

        # Note: No inertia properties set for the base link as it is not part of the simulation

        
        link_no = self.getLinkNo()
        link_lengths = self.getLinkLengths()
        link_lengths.insert(0,l_base)
        
        for j_ctr in range(link_no-1):
            l_ctr = j_ctr + 1

            # Start link Elements
            link.append(ET.SubElement(root,"link"))         
            link[l_ctr].set("name","Link_no_{}".format(l_ctr))      #Eg link name: Link1
            child = "Link_no_{}".format(l_ctr)

            # Visual
            visual.append(ET.SubElement(link[l_ctr], "visual"))
            v_geometry.append(ET.SubElement(visual[l_ctr],"geometry"))
            v_set_geom.append(ET.SubElement(v_geometry[l_ctr],"box"))   # All links are cube shaped - this line can be changed to change the shape or introduce a mesh
            v_set_geom[l_ctr].set("size","{} {} {}".format(thick, width, link_lengths[l_ctr]))  #thickness and width of the block are predefined; length has been defined by the user at initialization
            l_origin.append(ET.SubElement(visual[l_ctr],"origin"))  #Orgin tells rviz where to place the new link
            if self._joint_types[j_ctr] == 0:
                l_origin[l_ctr].set("rpy","0 {} 0".format(self._fixed_angle_orient[j_ctr]*3.14/180))

                #### This is where the orientation needs to be considered for the placement of the fixed joint link

                # l_origin[l_ctr].set("xyz","{} {} {}".\
                #     format(-m.sin(self._fixed_angle_orient[j_ctr])*link_lengths[l_ctr],\
                #         width*l_ctr, - m.cos(self._fixed_angle_orient[j_ctr])*link_lengths[l_ctr] + \
                #         link_lengths[l_ctr]/2))
            else:
                l_origin[l_ctr].set("rpy","0 0 0")
            #### This line should be moved to the else condition if the fixed ink orientation problem is solved
            l_origin[l_ctr].set("xyz","0 {} {}".format(width*l_ctr,link_lengths[l_ctr]/2))  #All links are progressively offset further to the right by the width
            # this is done so the collision constaint does not give trouble with the robot interacting with itelf !!! Not sure if this is the best solution

            # collision - Tells the software to avoid collision within a given area, typically the same a the object's geometry
            collision.append(ET.SubElement(link[l_ctr],"collision"))
            c_geometry.append(ET.SubElement(collision[l_ctr],"geometry"))
            c_set_geom.append(ET.SubElement(c_geometry[l_ctr],"box"))
            c_set_geom[l_ctr].set("size","{} {} {}".format(thick, width, link_lengths[l_ctr]))

            # Inertial - The usual moment of inertia matrix
            mass = width*thick*link_lengths[l_ctr]*self._dens
            inertial.append(ET.SubElement(link[l_ctr],"inertial"))
            i_origin.append(ET.SubElement(inertial[l_ctr-1],"origin"))
            i_origin[l_ctr-1].set("rpy","0 0 0")
            i_origin[l_ctr-1].set("xyz","{} {} {}".format(thick/2, width/2, link_lengths[l_ctr]/2))
            i_mass.append(ET.SubElement(inertial[l_ctr-1],'mass'))
            i_mass[l_ctr-1].set('value','{}'.format(mass))
            i_inertia.append(ET.SubElement(inertial[l_ctr-1],'inertia'))
            i_inertia[l_ctr-1].set('ixx','${{{0} / 12 * ({1}*{1} + {2}*{2})}}'.format(mass,width, link_lengths[l_ctr]))
            i_inertia[l_ctr-1].set('ixy','0.0')
            i_inertia[l_ctr-1].set('ixz','0.0')
            i_inertia[l_ctr-1].set('iyy','${{{0} / 12 * ({1}*{1} + {2}*{2})}}'.format(mass,thick, link_lengths[l_ctr]))
            i_inertia[l_ctr-1].set('iyz','0.0')
            i_inertia[l_ctr-1].set('izz','${{{0} / 12 * ({1}*{1} + {2}*{2})}}'.format(mass,width, thick))

            # Start joint elements
            joint.append(ET.SubElement(root, "joint"))
            joint[j_ctr].set("name","Joint{}".format(j_ctr))
            joint[j_ctr].set("type",joint_type[self._joint_types[j_ctr]])

            
            if self._joint_types[j_ctr] == 1:   # Continuous revolute joint
                j_axis.append(ET.SubElement(joint[j_ctr],"axis"))
                j_axis[j_ctr].set('rpy','0 0 0')
                j_axis[j_ctr].set('xyz','0 1 0')
            elif self._joint_types[j_ctr] == -1:    # Prismatic joint
                j_axis.append(ET.SubElement(joint[j_ctr],"axis"))
                j_axis[j_ctr].set('rpy','0 0 0')
                j_axis[j_ctr].set('xyz','0 0 1')


            j_parent.append(ET.SubElement(joint[j_ctr], 'parent'))
            j_parent[j_ctr].set('link',parent)
            j_child.append(ET.SubElement(joint[j_ctr], 'child'))
            j_child[j_ctr].set('link',child)

            if self._joint_types[j_ctr] == -1:  # Prismatic joint
                j_limit.append(ET.SubElement(joint[j_ctr],"limit"))
                j_limit[-1].set('effort','1000.0')
                j_limit[-1].set('lower','-{}'.format(link_lengths[l_ctr]))
                j_limit[-1].set('upper','0')
                j_limit[-1].set('velocity','0.5')

            j_origin.append(ET.SubElement(joint[j_ctr], 'origin'))
            j_origin[j_ctr].set('xyz','0 0 {}'.format(link_lengths[l_ctr-1]))
            
            parent = child    # Updating parent to current link

        ID = self.getID()
        finalString = self.xml_space(root)

        # Write file to a urdf file with the robot's ID. Eg. robot_1.urdf
        fileName = 'robot_{}.urdf'.format(ID)
        self.write_file(finalString,fileName)

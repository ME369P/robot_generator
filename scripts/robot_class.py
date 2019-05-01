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
        self._P_gains = [100.0 for k in range(link_no)]
        self._I_gains = [0.01 for k in range(link_no)]
        self._D_gains = [10.0 for k in range(link_no)]


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
        Querry density of the links - used to calculate mass and inertia
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

    def set_gains(self):
        '''
        Set gains for PID controller. 
        '''
        P_gains = [-1]
        while len(P_gains) != self._link_no - 1:
            inp = input('Enter the Proportional gains for each joint ({} values with spaces): '.format(self._link_no-1))
            P_gains = [float(x) for x in inp.split()]
        self._P_gains = P_gains

        I_gains = [-1]
        while len(I_gains) != self._link_no - 1:
            inp = input('Enter the Integral gains for each joint ({} values with spaces): '.format(self._link_no-1))
            I_gains = [float(x) for x in inp.split()]
        self._I_gains = I_gains

        D_gains = [-1]
        while len(D_gains) != self._link_no - 1:
            inp = input('Enter the Differential gains for each joint ({} values with spaces): '.format(self._link_no-1))
            D_gains = [float(x) for x in inp.split()]
        self._D_gains = D_gains


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
        elif link_no == 1:
            orn_format = '0 {} ${{{} - axel_offset}}'
        else:
            orn_format = '0 {} ${{{} - 2*axel_offset}}'        

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
            finalString.append('    pid: {{p: {0}, i: {1}, d: {2}}}'.format(self._P_gains[k],self._I_gains[k],self._D_gains[k]))
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
        with open('{0}/urdf/{1}_root/{1}_info.txt'.format(abs_path,self.getID()),'w')\
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


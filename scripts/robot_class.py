import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
import math as m

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
        self._link_no = link_no
        self._joint_types = []
        self._link_lengths = []
        fixed_ang_dict = {}
        
        for k in range(link_no):
            l_len = -1
            while l_len <= 0:
                l_len = float(input('Enter length of link #{}:'.format(k+1)))
            j_type = 2
            while j_type not in [-1, 0, 1]:
                j_type = float(input('Enter type of joint:'))
            link_mass = -1

            self._link_lengths.append(l_len)
            self._joint_types.append(j_type)
            if j_type == 0:
                fixed_ang_dict[k] = float(input('Enter orientation (deg):'))

        self._fixed_angle_orient = fixed_ang_dict.copy()
        self._dens = float(input('Enter density of links:'))

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

    def getDensty(self):
        '''
        returns density of the links - used to calculate mass and inertia
        '''
        return self._dens

    def setDensty(self, new_dens):
        '''
        Change density of the links - used to calculate mass and inertia
        '''
        self.dens = new_dens

    def __str__(self):
        '''
        String conversion for easy reading.
        Returns: 
            str: robot description as a string
        '''

        return 'Robot ID {} has {} links connected by joints {}. (Density = {})'.format\
        (self._id, self._link_no, self._joint_types,self._dens)


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
        
        for j_ctr in range(link_no):
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


        ## This part of the code is only making the xml fle readable - adds no real value to the file itself - just adds whitespace
        roughString = ElementTree.tostring(root, 'utf-8')
        reparsed = minidom.parseString(roughString)
        finalString = reparsed.toprettyxml(indent="    ")
        ID = self.getID()

        # Write file to a urdf file with the robot's ID. Eg. robot_1.urdf
        with open('robot_{}.urdf'.format(ID),'w') as text_file:
            text_file.write(finalString)

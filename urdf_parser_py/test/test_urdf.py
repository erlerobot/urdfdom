from __future__ import print_function

import unittest
import mock
from xml.dom import minidom
from xml_matching import xml_matches
from urdf_parser_py import urdf

class ParseException(Exception):
    pass

class TestURDFParser(unittest.TestCase):
    @mock.patch('urdf_parser_py.xml_reflection.on_error',
                mock.Mock(side_effect=ParseException))
    def parse(self, xml):
        return urdf.Robot.from_xml_string(xml)

    def parse_and_compare(self, orig):
        xml = minidom.parseString(orig)
        robot = urdf.Robot.from_xml_string(orig)
        rewritten = minidom.parseString(robot.to_xml_string())
        self.assertTrue(xml_matches(xml, rewritten))

    def parse_print_and_compare(self, orig):
        xml = minidom.parseString(orig)
        robot = urdf.Robot.from_xml_string(orig)
        print(robot.to_str())
        rewritten = minidom.parseString(robot.to_xml_string())
        self.assertTrue(xml_matches(xml, rewritten))

    def test_robot_comm_cog_and_sens(self):
        xml = '''<?xml version="1.0"?>
<robot name="my_robot">
  <cognition name="robot_brain">
    <parent link="link1"/>
  </cognition>
  
  <communication name="comm_node">
    <parent link="link1"/>
  </communication>

  <sensor name="my_sensor" type="IMU" rate="50.0">
    <parent link="link1"/>
    <origin xyz="0.0 0.0 1.0" rpy="0.0 0.0 3.1416"/>
    <camera>
      <image width="620" height="620" format="rgb8"/>
    </camera>    
    <imu>
      <gyroscopes>0.0 0.0 1.0</gyroscopes>
      <accelerometers>1.0 0.5 1.0</accelerometers>
    </imu>
  </sensor>

   <joint name="my_joint" type="floating">
      <origin xyz="0.0 0.0 1.0" rpy="0.0 0.0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>
   </joint>

</robot>'''
        self.parse(xml)        


    def test_communication(self):
        xml = '''<?xml version="1.0"?>
<robot name="my_robot">
  <communication name="comm_node">
    <parent link="link1"/>
  </communication>
</robot>'''
        self.parse(xml)        


    def test_cognition(self):
        xml = '''<?xml version="1.0"?>
<robot name="my_robot">
  <cognition name="robot_brain">
    <parent link="link1"/>
  </cognition>
</robot>'''
        self.parse(xml)        


    def test_simple_sensor(self):
        xml = '''<?xml version="1.0"?>
<robot name="sensor_robot">
  <sensor name="my_sensor" type="IMU" rate="50.0">
    <parent link="link1"/>
    <origin xyz="0.0 0.0 1.0" rpy="0.0 0.0 3.1416"/>
    <camera>
      <image width="620" height="620" format="rgb8"/>
    </camera>    
    <imu>
      <gyroscopes>0.0 0.0 1.0</gyroscopes>
      <accelerometers>1.0 0.5 1.0</accelerometers>
    </imu>
  </sensor>
</robot>'''
        # self.parse_print_and_compare(xml)
        self.parse_and_compare(xml)        

    def test_empty_joint(self):
        xml = '''<?xml version="1.0"?>
<robot name="only_robot">
   <joint name="my_joint" type="floating">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>
   </joint>
</robot>'''
        self.parse(xml)


    def test_simple_joint(self):
        xml = '''<?xml version="1.0"?>
<robot name="only_robot">
   <joint name="my_joint" type="floating">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="lin
      k2"/>

      <calibration rising="0.0"/>
      <dynamics damping="0.0" friction="0.0"/>
      <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
      <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
   </joint>
</robot>'''
        self.parse(xml)

    def test_simple_link(self):
        xml = '''<?xml version="1.0"?>
<robot name="only_robot">
 <link name="my_link">
   <inertial>
     <origin xyz="0 0 0.5" rpy="0 0 0"/>
     <mass value="1"/>
     <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
   </inertial>

   <visual>
     <origin xyz="0 0 0" rpy="0 0 0" />
     <geometry>
       <box size="1 1 1" />
     </geometry>
     <material name="Cyan">
       <color rgba="0 1.0 1.0 1.0"/>
     </material>
   </visual>

   <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <cylinder radius="1" length="0.5"/>
     </geometry>
   </collision>
 </link>
</robot>'''
        self.parse(xml)

    def test_empty_robot(self):
        xml = '''<?xml version="1.0"?>
<robot name="only_robot">

</robot>'''
        self.parse_and_compare(xml)


    def test_new_transmission(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_multiple_joints(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <joint name="bar_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_multiple_actuators(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
    <actuator name="bar_motor"/>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_missing_joint(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
  </transmission>
</robot>'''
        self.assertRaises(Exception, self.parse, xml)

    def test_new_transmission_missing_actuator(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
</robot>'''
        self.assertRaises(Exception, self.parse, xml)

    def test_old_transmission(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="PR2_trans" type="SimpleTransmission">
    <joint name="foo_joint"/>
    <actuator name="foo_motor"/>
    <mechanicalReduction>1.0</mechanicalReduction>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_link_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="link">
    <visual>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
      <material name="mat"/>
    </visual>
  </link>
</robot>'''
        self.parse_and_compare(xml)

    def test_robot_material(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <material name="mat">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>'''
        self.parse_and_compare(xml)

    def test_robot_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <material name="mat"/>
</robot>'''
        self.assertRaises(ParseException, self.parse, xml)


if __name__ == '__main__':
    unittest.main()

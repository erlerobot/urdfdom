from urdf_parser_py.xml_reflection.basics import *
import urdf_parser_py.xml_reflection as xmlr

# Add a 'namespace' for names so that things don't conflict between URDF and SDF?
# A type registry? How to scope that? Just make a 'global' type pointer?
# Or just qualify names? urdf.geometric, sdf.geometric

xmlr.start_namespace('urdf')

xmlr.add_type('element_link', xmlr.SimpleElementType('link', str))
xmlr.add_type('element_xyz', xmlr.SimpleElementType('xyz', 'vector3'))

verbose = True

class Pose(xmlr.Object):
	def __init__(self, xyz=None, rpy=None):
		self.xyz = xyz
		self.rpy = rpy
	
	def check_valid(self):
		assert self.xyz is not None or self.rpy is not None
	
	# Aliases for backwards compatibility
	@property
	def rotation(self): return self.rpy
	@rotation.setter
	def rotation(self, value): self.rpy = value
	@property
	def position(self): return self.xyz
	@position.setter
	def position(self, value): self.xyz = value

xmlr.reflect(Pose, params = [
	xmlr.Attribute('xyz', 'vector3', False),
	xmlr.Attribute('rpy', 'vector3', False)
	])


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)

class Color(xmlr.Object):
	def __init__(self, *args):
		# What about named colors?
		count = len(args)
		if count == 4 or count == 3:
			self.rgba = args
		elif count == 1:
			self.rgba = args[0]
		elif count == 0:
			self.rgba = None
		if self.rgba is not None:
			if len(self.rgba) == 3:
				self.rgba += [1.]
			if len(self.rgba) != 4:
				raise Exception('Invalid color argument count')

xmlr.reflect(Color, params = [
	xmlr.Attribute('rgba', 'vector4')
	])


class JointDynamics(xmlr.Object):
	def __init__(self, damping=None, friction=None):
		self.damping = damping
		self.friction = friction

xmlr.reflect(JointDynamics, params = [
	xmlr.Attribute('damping', float, False),
	xmlr.Attribute('friction', float, False)
	])


class Box(xmlr.Object):
	def __init__(self, size = None):
		self.size = size

xmlr.reflect(Box, params = [
	xmlr.Attribute('size', 'vector3')
	])


class Cylinder(xmlr.Object):
	def __init__(self, radius = 0.0, length = 0.0):
		self.radius = radius
		self.length = length

xmlr.reflect(Cylinder, params = [
	xmlr.Attribute('radius', float),
	xmlr.Attribute('length', float)
	])


class Sphere(xmlr.Object):
	def __init__(self, radius=0.0):
		self.radius = radius

xmlr.reflect(Sphere, params = [
	xmlr.Attribute('radius', float)
	])


class Mesh(xmlr.Object):
	def __init__(self, filename = None, scale = None):
		self.filename = filename
		self.scale = scale

xmlr.reflect(Mesh, params = [
	xmlr.Attribute('filename', str),
	xmlr.Attribute('scale', 'vector3', required=False)
	])


class GeometricType(xmlr.ValueType):
	def __init__(self):
		self.factory = xmlr.FactoryType('geometric', {
			'box': Box,
			'cylinder': Cylinder,
			'sphere': Sphere,
			'mesh': Mesh
			})
	
	def from_xml(self, node):
		children = xml_children(node)
		assert len(children) == 1, 'One element only for geometric'
		return self.factory.from_xml(children[0])
	
	def write_xml(self, node, obj):
		name = self.factory.get_name(obj)
		child = node_add(node, name)
		obj.write_xml(child)

xmlr.add_type('geometric', GeometricType())

class Collision(xmlr.Object):
	def __init__(self, geometry = None, origin = None):
		self.geometry = geometry
		self.origin = origin

xmlr.reflect(Collision, params = [
	origin_element,
	xmlr.Element('geometry', 'geometric')
	])


class Texture(xmlr.Object):
	def __init__(self, filename = None):
		self.filename = filename

xmlr.reflect(Texture, params = [
	xmlr.Attribute('filename', str)
	])


class Material(xmlr.Object):
	def __init__(self, name=None, color=None, texture=None):
		self.name = name
		self.color = color
		self.texture = texture
	
	def check_valid(self):
		if self.color is None and self.texture is None:
			xmlr.on_error("Material has neither a color nor texture.\n")

xmlr.reflect(Material, params = [
	name_attribute,
	xmlr.Element('color', Color, False),
	xmlr.Element('texture', Texture, False)
	])

class LinkMaterial(Material):
	def check_valid(self):
		pass

class Visual(xmlr.Object):
	def __init__(self, geometry = None, material = None, origin = None):
		self.geometry = geometry
		self.material = material
		self.origin = origin

xmlr.reflect(Visual, params = [
	origin_element,
	xmlr.Element('geometry', 'geometric'),
	xmlr.Element('material', LinkMaterial, False)
	])


class Inertia(xmlr.Object):
	KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
	
	def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
		self.ixx = ixx
		self.ixy = ixy
		self.ixz = ixz
		self.iyy = iyy
		self.iyz = iyz
		self.izz = izz
	
	def to_matrix(self):
		return [
			[self.ixx, self.ixy, self.ixz],
			[self.ixy, self.iyy, self.iyz],
			[self.ixz, self.iyz, self.izz]]

xmlr.reflect(Inertia, params = [xmlr.Attribute(key, float) for key in Inertia.KEYS])


class Inertial(xmlr.Object):
	def __init__(self, mass = 0.0, inertia = None, origin=None):
		self.mass = mass
		self.inertia = inertia
		self.origin = origin

xmlr.reflect(Inertial, params = [
	origin_element,
	xmlr.Element('mass', 'element_value'),
	xmlr.Element('inertia', Inertia, False)
	])



#FIXME: we are missing the reference position here.
class JointCalibration(xmlr.Object):
	def __init__(self, rising=None, falling=None):
		self.rising = rising
		self.falling = falling

xmlr.reflect(JointCalibration, params = [
	xmlr.Attribute('rising', float, False, 0),
	xmlr.Attribute('falling', float, False, 0)
	])

class JointLimit(xmlr.Object):
	def __init__(self, effort=None, velocity=None, lower=None, upper=None):
		self.effort = effort
		self.velocity = velocity
		self.lower = lower
		self.upper = upper

xmlr.reflect(JointLimit, params = [
	xmlr.Attribute('effort', float),
	xmlr.Attribute('lower', float, False, 0),
	xmlr.Attribute('upper', float, False, 0),
	xmlr.Attribute('velocity', float)
	])

#FIXME: we are missing __str__ here.
class JointMimic(xmlr.Object):
	def __init__(self, joint_name=None, multiplier=None, offset=None):
		self.joint = joint_name
		self.multiplier = multiplier
		self.offset = offset	

xmlr.reflect(JointMimic, params = [
	xmlr.Attribute('joint', str),
	xmlr.Attribute('multiplier', float, False),
	xmlr.Attribute('offset', float, False)
	])

class SafetyController(xmlr.Object):
	def __init__(self, velocity=None, position=None, lower=None, upper=None):
		self.k_velocity = velocity
		self.k_position = position
		self.soft_lower_limit = lower
		self.soft_upper_limit = upper

xmlr.reflect(SafetyController, params = [
	xmlr.Attribute('k_velocity', float),
	xmlr.Attribute('k_position', float, False, 0),
	xmlr.Attribute('soft_lower_limit', float, False, 0),
	xmlr.Attribute('soft_upper_limit', float, False, 0)
	])

class Joint(xmlr.Object):
	TYPES = ['unknown', 'revolute', 'continuous', 'prismatic', 'floating', 'planar', 'fixed']

	def __init__(self, name=None, parent=None, child=None, joint_type=None,
			axis=None, origin=None,
			limit=None, dynamics=None, safety_controller=None, calibration=None,
			mimic=None):
		self.name = name
		self.parent = parent
		self.child = child
		self.type = joint_type
		self.axis = axis
		self.origin = origin
		self.limit = limit
		self.dynamics = dynamics
		self.safety_controller = safety_controller
		self.calibration = calibration
		self.mimic = mimic
	
	def check_valid(self):
		assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)

	def change_parent_link(self, new_link):
		self.parent = new_link

	def change_child_link(self, new_link):
		self.child = new_link
	
	# Aliases
	@property
	def joint_type(self): return self.type
	@joint_type.setter
	def joint_type(self, value): self.type = value

xmlr.reflect(Joint, params = [
	name_attribute,
	xmlr.Attribute('type', str),
	origin_element,
	xmlr.Element('axis', 'element_xyz', False),
	xmlr.Element('parent', 'element_link'),
	xmlr.Element('child', 'element_link'),
	xmlr.Element('limit', JointLimit, False),
	xmlr.Element('dynamics', JointDynamics, False),
	xmlr.Element('safety_controller', SafetyController, False),
	xmlr.Element('calibration', JointCalibration, False),
	xmlr.Element('mimic', JointMimic, False),
	])


class Link(xmlr.Object):
	def __init__(self, name=None, visual=None, inertial=None, collision=None, origin = None):
		self.name = name
		self.visual = visual
		self.inertial = inertial
		self.collision = collision
		self.origin = origin

xmlr.reflect(Link, params = [
	name_attribute,
	origin_element,
	xmlr.Element('inertial', Inertial, False),
	xmlr.Element('visual', Visual, False),
	xmlr.Element('collision', Collision, False)
	])


class PR2Transmission(xmlr.Object):
	def __init__(self, name = None, joint = None, actuator = None, type = None, mechanicalReduction = 1):
		self.name = name
		self.type = type
		self.joint = joint
		self.actuator = actuator
		self.mechanicalReduction = mechanicalReduction

xmlr.reflect(PR2Transmission, tag = 'pr2_transmission', params = [
	name_attribute,
	xmlr.Attribute('type', str),
	xmlr.Element('joint', 'element_name'),
	xmlr.Element('actuator', 'element_name'),
	xmlr.Element('mechanicalReduction', float)
	])


class Actuator(xmlr.Object):
	def __init__(self, name = None, mechanicalReduction = 1):
		self.name = name
		self.mechanicalReduction = None

xmlr.reflect(Actuator, tag = 'actuator', params = [
		name_attribute,
		xmlr.Element('mechanicalReduction', float, required = False)
		])

class TransmissionJoint(xmlr.Object):
	def __init__(self, name = None):
		self.aggregate_init()
		self.name = name
		self.hardwareInterfaces = []

	def check_valid(self):
		assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


xmlr.reflect(TransmissionJoint, tag = 'joint', params = [
		name_attribute,
		xmlr.AggregateElement('hardwareInterface', str),
		])

class Transmission(xmlr.Object):
	""" New format: http://wiki.ros.org/urdf/XML/Transmission """
	def __init__(self, name = None):
		self.aggregate_init()
		self.name = name
		self.joints = []
		self.actuators = []

	def check_valid(self):
		assert len(self.joints) > 0, "no joint defined"
		assert len(self.actuators) > 0, "no actuator defined"

xmlr.reflect(Transmission, tag = 'new_transmission', params = [
		name_attribute,
		xmlr.Element('type', str),
		xmlr.AggregateElement('joint', TransmissionJoint),
		xmlr.AggregateElement('actuator', Actuator)
		])

xmlr.add_type('transmission', xmlr.DuckTypedFactory('transmission', [Transmission, PR2Transmission]))


class Image(xmlr.Object):

	def __init__(self, width=None, height=None, format=None, hfov=None, near=None, far=None):
		self.width = width # pixels, integer
		self.height = height # pixels, integer
		self.format = format # string, select one from https://goo.gl/8OR1t3
		self.hfov = hfov # radians, float, horizontal field of view
		self.near = near # meters, float, near clip distance of the camera in meters
		self.far = far # meters, float, far clip distance of the camera in meters
	
	def check_valid(self):
		pass
	
xmlr.reflect(Image, params = [
	xmlr.Attribute('width', int, True),
	xmlr.Attribute('height', int, True),
	xmlr.Attribute('format', str
		, True),
	xmlr.Attribute('hfov', float, False),
	xmlr.Attribute('near', float, False),
	xmlr.Attribute('far', float, False)
	])

class Camera(xmlr.Object):
	def __init__(self, image=None):
		self.image = image

xmlr.reflect(Camera, params = [
	xmlr.Element('image', Image, True)
	])

class IMU(xmlr.Object):
	def __init__(self, gyroscopes=None, accelerometers=None):
		self.gyroscopes = gyroscopes
		self.accelerometers = accelerometers


xmlr.reflect(IMU, params = [
	xmlr.Element('gyroscopes', 'vector3', True),
	xmlr.Element('accelerometers', 'vector3', True)	
	])

class Ray(xmlr.Object):
	def __init__(self, 
				 horizontal_samples=None, 	horizontal_resolution=None,
				 horizontal_min_angle=None, horizontal_max_angle=None,
				 vertical_samples=None, 	vertical_resolution=None,
				 vertical_min_angle=None, 	vertical_max_angle=None,
				 range_min=None,			range_max=None,
				 range_resolution=None):
		self.horizontal_samples = horizontal_samples 	
		self.horizontal_resolution = horizontal_resolution
		self.horizontal_min_angle = horizontal_min_angle 
		self.horizontal_max_angle = horizontal_max_angle
		self.vertical_samples = vertical_samples	
		self.vertical_resolution = vertical_resolution
		self.vertical_min_angle = vertical_min_angle 	
		self.vertical_max_angle = vertical_max_angle
		self.range_min = range_min	
		self.range_max = range_max
		self.range_resolution = range_resolution

xmlr.reflect(Ray, params=[
	xmlr.Element('horizontal_samples', float, True),
	xmlr.Element('horizontal_resolution', float, True),
	xmlr.Element('horizontal_min_angle', float, True),
	xmlr.Element('horizontal_max_angle', float, True),
	xmlr.Element('vertical_samples', float, True),
	xmlr.Element('vertical_resolution', float, True),
	xmlr.Element('vertical_min_angle', float, True),
	xmlr.Element('vertical_max_angle', float, True),
	xmlr.Element('range_min', float, True),
	xmlr.Element('range_max', float, True),
	xmlr.Element('range_resolution', float, True)
	])

class Sensor(xmlr.Object):
	def __init__(self, name=None, type=None, rate=None, parent=None, origin=None, camera=None, imu=None, ray=None):
		self.name = name
		self.type = type
		self.rate = rate
		self.parent = parent # represents the link this component belongs to
		self.origin = origin
		self.camera = camera		
		self.imu = imu
		self.ray = ray

	def check_valid(self):
		pass

	def change_link(self, new_link):
		self.parent = new_link

xmlr.reflect(Sensor, params = [
	name_attribute,
	xmlr.Attribute('type', str, True),
	xmlr.Attribute('rate', float, True),
	xmlr.Element('parent', 'element_link'),	
	origin_element,
	# A sensor tag may contain one of many of the following sensor types
	xmlr.Element('camera', Camera, False),
	xmlr.Element('imu', IMU, False),
	xmlr.Element('ray', Ray, False)
	])

class Cognition(xmlr.Object):
	def __init__(self, name=None, parent=None):
		self.name = name
		self.parent = parent # represents the link this component belongs to

	def check_valid(self):
		pass

	def change_link(self, new_link):
		self.parent = new_link

xmlr.reflect(Cognition, params = [
	name_attribute,
	xmlr.Element('parent', 'element_link')
	])

class Communication(xmlr.Object):
	def __init__(self, name=None, parent=None):
		self.name = name
		self.parent = parent # represents the link this component belongs to

	def check_valid(self):
		pass

	def change_link(self, new_link):
		self.parent = new_link

xmlr.reflect(Communication, params = [
	name_attribute,
	xmlr.Element('parent', 'element_link')
	])

class Robot(xmlr.Object):
	def __init__(self, name = None):
		self.aggregate_init()
		
		self.name = name
		self.joints = []
		self.links = []
		self.sensors = []
		self.cognitions = []
		self.communications = []

		self.materials = []
		self.gazebos = []
		self.transmissions = []
		
		self.joint_map = {}
		self.link_map = {}
		self.cognition_map = {}
		self.sensors_map = {}
		self.communication_map = {}

		self.parent_map = {}
		self.child_map = {}
	

	def add_aggregate(self, typeName, elem):
		xmlr.Object.add_aggregate(self, typeName, elem)
		
		if typeName == 'joint':
			joint = elem
			self.joint_map[joint.name] = joint
			self.parent_map[ joint.child ] = (joint.name, joint.parent)
			if joint.parent in self.child_map:
				self.child_map[joint.parent].append( (joint.name, joint.child) )
			else:
				self.child_map[joint.parent] = [ (joint.name, joint.child) ]
		elif typeName == 'link':
			link = elem
			self.link_map[link.name] = link
		elif typeName == 'sensor':
			sensor = elem
			self.sensors_map[sensor.name] = sensor
		elif typeName == 'cognition':
			cognition = elem
			self.cognition_map[cognition.name] = cognition
		elif typeName == 'communication':
			communication = elem
			self.communication_map[communication.name] = communication

	def element_from_name(self, typeName, element_name):
		if typeName == 'joint':
			return self.joint_map[element_name]
		elif typeName == 'link':
			return self.link_map[element_name]
		elif typeName == 'sensor':
			return self.sensors_map[element_name]
		elif typeName == 'cognition':
			return self.cognition_map[element_name]
		elif typeName == 'communication':
			return self.communication_map[element_name]
		else:
			print("Sorry but we can't return that element\n")

	def remove_aggregate(self, typeName, elem_name):
		elem = self.element_from_name(typeName, elem_name)
		xmlr.Object.remove_aggregate(self, elem)
		
		if typeName == 'joint':
			joint = elem
			self.joint_map.pop(joint.name, None)
		elif typeName == 'link':
			link = elem
			self.link_map.pop(link.name, None)
		elif typeName == 'sensor':
			sensor = elem
			self.sensors_map.pop(sensor.name, None)
		elif typeName == 'cognition':
			cognition = elem
			self.cognition_map.pop(cognition.name, None)
		elif typeName == 'communication':
			communication = elem
			self.communication_map.pop(communication.name, None)						

	def add_link(self, link):
		self.add_aggregate('link', link)

	def add_joint(self, joint):
		self.add_aggregate('joint', joint)

	def add_sensor(self, sensor):
		self.add_aggregate('sensor', sensor)

	def add_cognition(self, cognition):
		self.add_aggregate('cognition', cognition)

	def add_communication(self, communication):
		self.add_aggregate('communication', communication)

	def remove_link(self, link_name):
		self.remove_aggregate('link', link_name)

	def remove_joint(self, joint_name):
		self.remove_aggregate('joint', joint_name)

	def remove_sensor(self, sensor_name):
		self.remove_aggregate('sensor', sensor_name)

	def remove_cognition(self, cognition_name):
		self.remove_aggregate('cognition', cognition_name)

	def remove_communication(self, communication_name):
		self.remove_aggregate('communication', communication_name)


	def get_chain(self, root, tip, joints=True, links=True, fixed=True):
		chain = []
		if links:
			chain.append(tip)
		link = tip
		while link != root:
			(joint, parent) = self.parent_map[link]
			if joints:
				if fixed or self.joint_map[joint].joint_type != 'fixed':
					chain.append(joint)
			if links:
				chain.append(parent)
			link = parent
		chain.reverse()
		return chain

	def get_root(self):
		root = None
		for link in self.link_map:
			if link not in self.parent_map:
				assert root is None, "Multiple roots detected, invalid URDF."
				root = link
		assert root is not None, "No roots detected, invalid URDF."
		return root

	@classmethod
	def from_parameter_server(cls, key = 'robot_description'):
		"""
		Retrieve the robot model on the parameter server
		and parse it to create a URDF robot structure.

		Warning: this requires roscore to be running.
		"""
		# Could move this into xml_reflection
		import rospy
		return cls.from_xml_string(rospy.get_param(key))

	def to_str(self, verbose=True):
		"""
		Provide a simple string representation of the class		
		"""		
		output = ""
		output += "Robot: "+str(self.name)+"\n"		
		output += "Links:\n"
		for l in self.links:
			output += "- "+str(l.name)+"\n"
			# TODO print also the rest of the elements of the link
		
		output += "Joints:\n"
		for j in self.joints:
			output += "* "+str(j.name)+"\n"
			# TODO print also the rest of the elements of the joint
		
		output += "Cognition:\n"
		for brain in self.cognitions:
			output += "   name: "+str(brain.name)+"\n"
			output += "   parent: "+str(brain.parent)+"\n"
		
		output += "Communication:\n"
		for comm in self.communications:
			output += "   name: "+str(comm.name)+"\n"
			output += "   parent: "+str(comm.parent)+"\n"

		output += "Sensors:\n"		

		for s in self.sensors:
			output += "* "+str(s.name)+"\n"
			output += "   type: "+s.type+"\n"
			output += "   rate: "+str(s.rate)+"\n"
			output += "   parent: "+str(s.parent)+"\n"
			if s.origin:
				output += "   origin: "+str(s.origin)+"\n"
			if s.camera:
				output += "   Camera:\n"
				output += "      Image:\n"
				output += "         width: "+str(s.camera.image.width)+"\n"
				output += "         height: "+str(s.camera.image.height)+"\n"
				output += "         format: "+str(s.camera.image.format)+"\n"
				if s.camera.image.hfov:			
					output += "         hfov: "+str(s.camera.image.hfov)+"\n"
				if s.camera.image.near:
					output += "         near: "+str(s.camera.image.near)+"\n"
				if s.camera.image.far:
					output += "         far: "+str(s.camera.image.far)+"\n"
			if s.imu:
				output += "   IMU:\n"
				output += "      accelerometers: "+str(s.imu.accelerometers)+"\n"
				output += "      gyroscopes: "+str(s.imu.gyroscopes)+"\n"
			if s.ray:
				output += "   Ray:\n"
				output += "      horizontal: \n"
				output += "         samples: " + str(s.ray.horizontal_samples) + "\n"
				output += "         resolution: " + str(s.ray.horizontal_resolution) + "\n"
				output += "         min angle: " + str(s.ray.horizontal_min_angle) + "\n"
				output += "         max angle: " + str(s.ray.horizontal_max_angle) + "\n"
				output += "      vertical: \n"
				output += "         samples: " + str(s.ray.vertical_samples) + "\n"
				output += "         resolution: " + str(s.ray.vertical_resolution) + "\n"
				output += "         min angle: " + str(s.ray.vertical_min_angle) + "\n"
				output += "         max angle: " + str(s.ray.vertical_max_angle) + "\n"
				output += "      range: \n"
				output += "         min: " + str(s.ray.range_min) + "\n"
				output += "         max: " + str(s.ray.range_max) + "\n"
				output += "         resolution: " + str(s.ray.resolution) + "\n"
		return output
	
xmlr.reflect(Robot, tag = 'robot', params = [
	xmlr.Attribute('name', str, False), # Is 'name' a required attribute?
	xmlr.AggregateElement('link', Link),
	xmlr.AggregateElement('joint', Joint),
	xmlr.AggregateElement('sensor', Sensor),
	xmlr.AggregateElement('cognition', Cognition),
	xmlr.AggregateElement('communication', Communication),	
	xmlr.AggregateElement('gazebo', xmlr.RawType()),
	xmlr.AggregateElement('transmission', 'transmission'),
	xmlr.AggregateElement('material', Material)
	])

# Make an alias
URDF = Robot

xmlr.end_namespace()

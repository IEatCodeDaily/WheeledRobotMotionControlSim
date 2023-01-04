import pygame, sys
import numpy as np

class Window:
	def __init__(self, dim) -> None:
		#Window setting
		pygame.display.set_caption("Motion Control of Wheeled Mobile Robot")

		#dimensions
		self.width = dim[0]
		self.height= dim[1]

		#Window setting
		self.window = pygame.display.set_mode((self.width,self.height))
		self.window.fill((255,255,255))


		#State Text Box
		self.font = pygame.font.SysFont('Arial', 30)
		self.text = self.font.render('default', True, (0, 0, 0))
		self.textRect = self.text.get_rect()
		self.textRect.center = (200, 700)

		#Goal Text Box
		self.font = pygame.font.SysFont('Arial', 30)
		self.gtext = self.font.render('default', True, (0, 0, 0))
		self.gtextRect = self.text.get_rect()
		self.gtextRect.center = (200, 900)


	def write_values(self, robot):
		text = "State x: {robot.x:.2f} y: {robot.y:.2f} theta: {theta:.2f} v_L: {v1:.2f} v_R: {v2:.2f}".format(robot = robot, theta = np.degrees(robot.theta), v1 = robot.v, v2 = robot.v)
		self.text = self.font.render(text, True, (0, 0, 0))
		self.window.blit(self.text, self.textRect)
		gtext = "Goal x: {robot.end_x:.2f} y: {robot.end_y:.2f} theta: {endtheta:.2f}".format(robot = robot, endtheta = np.degrees(robot.end_theta))
		self.gtext = self.font.render(gtext, True, (0, 0, 0))
		self.window.blit(self.gtext, self.gtextRect)

	

class Robot(pygame.sprite.Sprite):
	def __init__(self, initpos, constants, robotSprite, width, height) -> None:

		#robot variables
		self.x = initpos[0]
		self.y = initpos[1]
		self.theta = initpos[2]
		self.v = 0
		self.maxspeed = 100

		#constants
		self.kl = constants[0] #Linear Velocity Constant
		self.kr = constants[1]	#Rotational Velocity Constant
		self.kphi = constants[2] #Angle adjustment constant

		self.end_x = self.x
		self.end_y = self.y
		self.end_theta = self.theta

		#robot sprite
		self.img = pygame.image.load(robotSprite)
		self.rotated = self.img
		self.rect = self.img.get_rect(center = (self.x,self.y))

		self.width = width
		self.height = height

		
		#Robot trail and marker
		self.trails = []

	def get_coord(self):
		return [self.x, self.y]


	def draw(self, screen):
		screen.blit(self.rotated, self.rect)

		
	def create_path(self):

		# end
		mouse_pos = pygame.mouse.get_pos()
		self.end_x,self.end_y =  mouse_pos[0], mouse_pos[1] 
		self.end_theta = np.arctan2(self.end_y - self.y, self.end_x - self.x)

	def move(self):

		dx = self.x - self.end_x 
		dy = self.y - self.end_y
		dtheta = self.theta - self.end_theta


		# distance vector
		rho = np.sqrt(dx**2 + dy**2)

		# angle between rho and Xb
		alpha = np.arctan2(-dy,-dx) - self.theta

		#angle between rho and Xg
		phi = np.arctan2(-dy, -dx) - self.end_theta
	
		# linear velocity
		if abs(self.v*1000) < self.maxspeed:
			self.v = self.kr*rho*np.cos(alpha)
		else:
			self.v *= 0.9*self.maxspeed/1000/self.v

		# rotational velocity
		if alpha!= 0:
			# rotational velocity   
			omega = self.kr*alpha + self.kl*(alpha - self.kphi*phi)*np.cos(alpha)*np.sin(alpha)/alpha
		else:
			# following limit sinx/x when approaching 0 = 1
			omega = self.kr*alpha + self.kl*(alpha - self.kphi*phi)
			
		
		self.theta += omega*dt


	def update(self):
		if abs(self.x/(self.end_x+1)) > 0.01 or abs(self.y/(self.end_y+1)) > 0.01 or abs(self.theta/(self.end_theta+1)) > 0.02:
			self.move()
			
		self.x += (self.v)*np.cos(self.theta)*dt
		self.y += (self.v)*np.sin(self.theta)*dt
		self.rect = self.rotated.get_rect(center = (self.x,self.y))
		self.rotated = pygame.transform.rotate(self.img, np.degrees(-self.theta))

		
	def trail(self, screen):
		x, y = self.get_coord()
		self.trails.append([x,y])
		for i in range(1,len(self.trails)-1):
			pygame.draw.line(screen.window, (255,255,0), (self.trails[i-1][0],self.trails[i-1][1]), (self.trails[i][0],self.trails[i][1]))
		if self.trails.__sizeof__()>10000:
			self.trails.pop(0)

			
	def draw_direction(self, screen):
		l =  80

		#draw direction
		x, y = self.get_coord()
		x_axis = (x + l*np.cos(-self.theta), y - l*np.sin(-self.theta))
		y_axis = (x + l*np.cos(-self.theta + np.pi/2), y - l*np.sin(-self.theta + np.pi/2))
		pygame.draw.line(screen.window, (255,0,0), (x, y), x_axis, 2)
		pygame.draw.line(screen.window, (0,255,0), (x, y), y_axis, 2)

class Robot2(pygame.sprite.Sprite):
	def __init__(self, initpos, constants, robotSprite, width) -> None:

		#robot variables
		self.x = initpos[0]
		self.y = initpos[1]
		self.theta = initpos[2]
		self.maxspeed = 100

		self.v_L = 0
		self.v_R = 0

		self.x_L = self.x + 0.5*self.width*np.sin(self.theta)
		self.y_L = 0
		self.x_R = 0
		self.y_R = 0


		#constants
		self.kl = constants[0] #Linear Velocity Constant
		self.kr = constants[1]	#Rotational Velocity Constant
		self.kphi = constants[2] #Angle adjustment constant

		self.end_x = self.x
		self.end_y = self.y
		self.end_theta = self.theta

		#robot sprite
		self.img = pygame.image.load(robotSprite)
		self.rotated = self.img
		self.rect = self.img.get_rect(center = (self.x,self.y))

		self.width = width
	
	
		#Robot trail and marker
		self.trails = []


	def get_coord(self):
		return [self.x, self.y]


	def draw(self, screen):
		screen.blit(self.rotated, self.rect)
		screen.blit(self.whl_rotated, (self.whl1_x, self.whl1_y))
		screen.blit(self.whl_rotated, (self.whl2_x, self.whl2_y))

		
	def create_path(self):

		# end
		mouse_pos = pygame.mouse.get_pos()
		self.end_x,self.end_y =  mouse_pos[0], mouse_pos[1] 
		self.end_theta = np.arctan2(self.end_y - self.y, self.end_x - self.x)

	def move(self):

		dx = self.end_x  - self.x
		dy = self.end_y - self.y
		dtheta = self.end_theta - self.theta 


		# distance vector
		rho = np.sqrt(dx**2 + dy**2) 

		# angle between rho and Xb
		alpha = np.arctan2(-dy,-dx) - self.theta

		#angle between rho and Xg
		phi = np.arctan2(-dy, -dx) - self.end_theta
	
		# linear velocity
		if abs(self.v_L*1000) < self.maxspeed:
			self.v_L = self.kr*rho*np.cos(alpha)
		else:
			self.v *= 0.9*self.maxspeed/1000/self.v

		# rotational velocity
		if alpha!= 0:
			# rotational velocity   
			omega = self.kr*alpha + self.kl*(alpha - self.kphi*phi)*np.cos(alpha)*np.sin(alpha)/alpha
		else:
			# following limit sinx/x when approaching 0 = 1
			omega = self.kr*alpha + self.kl*(alpha - self.kphi*phi)
			
		
		self.theta += omega*dt


	def update(self):
		if abs(self.x/(self.end_x+1)) > 0.01 or abs(self.y/(self.end_y+1)) > 0.01 or abs(self.theta/(self.end_theta+1)) > 0.02:
			self.move()
			
		self.x += (self.v)*np.cos(self.theta)*dt
		self.y += (self.v)*np.sin(self.theta)*dt
		self.rect = self.rotated.get_rect(center = (self.x,self.y))
		self.whl1_x = self.x - 0.5*self.width*np.sin(-self.theta)
		self.whl1_y = self.y - 0.5*self.width*np.cos(-self.theta)
		self.whl2_x = self.x + 0.5*self.width*np.sin(-self.theta)
		self.whl2_y = self.y + 0.5*self.width*np.cos(-self.theta)
		
		self.rotated = pygame.transform.rotate(self.img, np.degrees(-self.theta))
		self.whl_rotated = pygame.transform.rotate(self.whl, np.degrees(-self.theta))


	def trail(self, screen):
		self.trails.append(self.get_coord())
		for trail in self.trails:
			pygame.draw.line(screen.window, (255,255,0), (trail[0],trail[1]), (x,y))
		if self.trails.__sizeof__()>10000:
			self.trails.pop(0)


# pygame setup
pygame.init()
clock = pygame.time.Clock()

#map init
dim = [1024,1024]
screen = Window(dim)

#robot init
robotSprite = "robot.png"
initpos = [100,100,0] #x,y,theta
constants = [1, 1, 5] #linear, rotational, phi
robot = Robot(initpos, constants, robotSprite, 32, 32)

dt = 0
last_t = pygame.time.get_ticks()

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			sys.exit()

	
	pygame.display.update()
	screen.window.fill((255,255,255))

	dt = (pygame.time.get_ticks()-last_t)/1000
	last_t = pygame.time.get_ticks()
	
	robot.create_path()		
	robot.update()
	robot.draw(screen.window)
	robot.trail(screen)

	screen.draw_direction(robot)
	screen.write_values(robot)
	clock.tick(60)
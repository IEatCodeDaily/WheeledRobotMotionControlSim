import pygame, sys
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
import numpy as np

class Env:
	def __init__(self, dim) -> None:
		#Window setting
		pygame.display.set_caption("Motion Control of Wheeled Mobile Robot")

		#dimensions
		self.width = dim[0]
		self.height= dim[1]

		#Map setting
		self.map = pygame.display.set_mode((self.width,self.height))
		self.map.fill((255,255,255))

		#Robot trail and marker
		self.trails = []

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

	def trail(self, robot):
		for i in range(len(self.trails)-1):
			pygame.draw.line(self.map, (255,255,0), (self.trails[i][0],self.trails[i][1]), (self.trails[i+1][0],self.trails[i+1][1]))
		if self.trails.__sizeof__()>5000:
			self.trails.pop(0)
		self.trails.append(robot.get_coord())



	def write_values(self, robot):
		text = "State x: {robot.x:.2f} y: {robot.y:.2f} theta: {theta:.2f} v: {v:.2f}".format(robot = robot, theta = np.degrees(robot.theta), v = robot.v*1000)
		self.text = self.font.render(text, True, (0, 0, 0))
		self.map.blit(self.text, self.textRect)
		gtext = "Goal x: {robot.end_x:.2f} y: {robot.end_y:.2f} theta: {endtheta:.2f}".format(robot = robot, endtheta = np.degrees(robot.end_theta))
		self.gtext = self.font.render(gtext, True, (0, 0, 0))
		self.map.blit(self.gtext, self.gtextRect)

	
	def draw_direction(self, robot):
		n =  80

		#draw direction
		centerx, centery = robot.get_coord()
		x_axis = (centerx + n*np.cos(-robot.theta), centery - n*np.sin(-robot.theta))
		y_axis = (centerx + n*np.cos(-robot.theta + np.pi/2), centery - n*np.sin(-robot.theta + np.pi/2))
		pygame.draw.line(self.map, (255,0,0), (centerx, centery), x_axis, 2)
		pygame.draw.line(self.map, (0,255,0), (centerx, centery), y_axis, 2)

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
		if abs(self.x/self.end_x) > 0.01 or abs(self.y/self.end_y) > 0.01 or abs(self.theta/self.end_theta) > 0.01:
			self.move()
			
		self.x += (self.v)*np.cos(self.theta)*dt
		self.y += (self.v)*np.sin(self.theta)*dt
		self.rect = self.rotated.get_rect(center = (self.x,self.y))
		self.rotated = pygame.transform.rotate(self.img, np.degrees(-self.theta))


	
# pygame setup
pygame.init()
clock = pygame.time.Clock()

#map init
dim = [1024,1024]
envir = Env(dim)

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

	
	robot.create_path()		
	pygame.display.update()
	envir.map.fill((255,255,255))
	dt = (pygame.time.get_ticks()-last_t)/1000
	last_t = pygame.time.get_ticks()
	
	robot.update()
	robot.draw(envir.map)

	envir.trail(robot)
	envir.draw_direction(robot)
	envir.write_values(robot)
	clock.tick(60)
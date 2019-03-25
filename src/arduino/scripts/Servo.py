# servo class
class Servo:
	def __init__(self, mid_position=1500, max_position=1800, min_position=1200):

		self.position = mid_position
		self.prev_position = self.position - 1 
		self.mid_position = mid_position
		self.max_position = max_position
		self.min_position = min_position
		self.direction = 'ccw'
		self.step_size = 5 # servo adjustments step by this amount
		
	
	def sweep_position(self): 
		''' increments or decrements servo value by 1 '''

		# if at max_left, swap current and previous positions
		if self.position >= self.max_position:
			self.position -= self.step_size
			self.prev_position += self.step_size
			self.direction = 'cw'

		# if at max_right, swap current and previous positions
		elif self.position <= self.min_position:
			self.position += self.step_size
			self.prev_position -= self.step_size
			self.direction = 'ccw'

		# if turning servo clockwise
		elif self.direction == 'cw':
			self.position -= self.step_size
			self.prev_position -= self.step_size

		# if turning servo counter clockwise
		elif self.direction == 'ccw':
			self.position += self.step_size
			self.prev_position += self.step_size

	def reset(self):
		self.position = self.mid_position
		self.prev_position = self.position - 1 
		self.direction = 'ccw'

	def set_position(self, new_position):
		if new_position <= self.max_position and new_position >= self.min_position:
			self.prev_position = self.position
			self.position = new_position

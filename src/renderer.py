import sys, pygame, os, time, math
import numpy as np

class Renderer:

    def __init__(self, size, sonar_min_range, sonar_max_range, sonar_angle, move_callback,
                init_pos=(0,0,0), landmarks=[]):
        self.size = size
        self.sea_color = (0, 200, 255)
        self.auv_size = (25, 25)
        self.landmark_color = (100, 100, 100)
        self.observed_landmark_color = (255, 0, 255)
        self.sonar_lines_color = (0, 0 ,0)
        self.sonar_min_range = sonar_min_range
        self.sonar_max_range = sonar_max_range
        self.sonar_angle = sonar_angle
        self.auv_pos = init_pos
        self.landmarks = landmarks
        self.move_callback = move_callback
        self.observed = np.array([])

        self.screen = pygame.display.set_mode(size)
        self.sprite_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'assets')

        auv_sprite = pygame.image.load(os.path.join(self.sprite_path, "submarine.png"))
        self.auv_sprite = pygame.transform.scale(auv_sprite, self.auv_size)

    def update_auv_pos(self, x, y, heading):
        self.auv_pos = (x, y, heading)

    def update_observed_landmarks(self, observed):
        self.observed = np.array(observed).astype(int)

    def draw_auv(self):
        x, y, heading = self.auv_pos
        rotated_sprite = pygame.transform.rotate(self.auv_sprite, math.degrees(heading))
        rect = pygame.Rect(x, y, *self.auv_size)
        rect[0] -= rect[2]/2
        rect[1] -= rect[3]/2
        self.screen.blit(rotated_sprite, rect)

    def draw_landmarks(self):
        for lm in self.landmarks:
            lm = lm.astype(int)
            if lm[0] in self.observed:
                color = self.observed_landmark_color 
            else:
                color = self.landmark_color
            pygame.draw.circle(self.screen, color, tuple(lm[1:]), 5)

    def draw_sonar(self):

        x, y, heading = self.auv_pos
        x = int(x)
        y = int(y)
        min_angle = heading - self.sonar_angle/2
        max_angle = heading + self.sonar_angle/2
        min_range = self.sonar_min_range if self.sonar_min_range >=2 else 2
        max_range = self.sonar_max_range if self.sonar_max_range >=2 else 2
        
        pygame.draw.circle(self.screen, (255,0,0), (x,y), 3)

        large_rect = (x-max_range, y-max_range, 2*max_range, 2*max_range)
        small_rect = (x-min_range, y-min_range, 2*min_range, 2*min_range)
        pygame.draw.arc(self.screen, self.sonar_lines_color, large_rect, min_angle, max_angle+0.05, 2)
        pygame.draw.arc(self.screen, self.sonar_lines_color, small_rect, min_angle, max_angle+0.05, 2)
        points = [
            [x+math.cos(min_angle)*min_range, y-math.sin(min_angle)*min_range],
            [x+math.cos(min_angle)*max_range, y-math.sin(min_angle)*max_range],
            [x+math.cos(max_angle)*min_range, y-math.sin(max_angle)*min_range],
            [x+math.cos(max_angle)*max_range, y-math.sin(max_angle)*max_range]
        ]
        points = np.array(points).astype(int)

        pygame.draw.line(self.screen, self.sonar_lines_color, points[0], points[1], 2)
        pygame.draw.line(self.screen, self.sonar_lines_color, points[2], points[3], 2)

    def render(self): 
        self.clear()
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]: self.move_callback(-1,0,0)
        if keys[pygame.K_a]: self.move_callback(0,-0.3,0)
        if keys[pygame.K_s]: self.move_callback(1,0,0)
        if keys[pygame.K_d]: self.move_callback(0,0.3,0) 
        if keys[pygame.K_q]: self.move_callback(0,0,0.01) 
        if keys[pygame.K_e]: self.move_callback(0,0,-0.01)    
            
        self.draw_sonar()
        self.draw_landmarks()
        self.draw_auv()
        pygame.display.flip()

    def clear(self):
        self.screen.fill(self.sea_color)

    def start(self):
        pygame.init()
        

        
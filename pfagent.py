#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
#################################################################

import sys
import math
import time

from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.mytankdata = []
        mytanks = self.bzrc.get_mytanks()
        for tank in mytanks:
            self.mytankdata.append((0.0, 0.0)) # push initial speed_error and angle_error onto list for each tank
        
        # TODO: Move these two variables into the Tank class (in bzrc.py)!
        last_angle_error = 0 # can tweak, used only as the initial value
        last_speed_error = 0 # can tweak
        
        # FROBBING CENTRAL:
        self.k_pa = 1 # angular velocity constant
        self.k_da = .1 # angular velocity derivative constant
        self.k_ps = 1 # speed proportional control constant
        self.k_ds = -0.5 # speed proportional derivative control constant
        
        self.max_dist_to_obstacle = 80

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []
        #print('Obstacles')
        #print('\n'.join('{}: {}'.format(*k) for k in enumerate(self.bzrc.get_obstacles())))
        #print('\n'.join('{}: {}'.format(*k) for k in enumerate(self.constants)))

        #for tank in mytanks:
        #    self.attack_enemies(tank)
        tank = mytanks[0]
        self.attack_enemies(tank)

        results = self.bzrc.do_commands(self.commands)

    def attack_enemies(self, tank):
        """Find the closest enemy and chase it, shooting as you go."""
        print(self.get_potential_field_vector(tank))
        v, theta = self.get_potential_field_vector(tank)
        self.commands.append(self.pd_controller_move(tank, v, theta))
        '''best_enemy = None
        best_dist = 2 * float(self.constants['worldsize'])
        for enemy in self.enemies:
            if enemy.status != 'alive':
                continue
            dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
            if dist < best_dist:
                best_dist = dist
                best_enemy = enemy
        if best_enemy is None:
            command = Command(tank.index, 0, 0, False)
            self.commands.append(command)
        else:
            self.move_to_position(tank, best_enemy.x, best_enemy.y)'''

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    ############################################################

    def get_potential_field_vector(self, tank):
        # Attractive field
        goal_found = False
        goal_x = -1
        goal_y = -1
        #print ("Tank has flag? ")
        #print (tank.flag)
        if tank.flag == '-':
            goal = self.get_closest_flag(tank)
            if goal is not None:
                goal_found = True
                goal_x = goal.x
                goal_y = goal.y
                #print ("Closest flag: ")
                #print(goal.color)
        if not goal_found:
            bases = self.bzrc.get_bases()
            for base in bases:
                if base.color == self.constants['team']:
                    goal_x = (base.corner1_x + base.corner2_x + base.corner3_x + base.corner4_x) /4
                    goal_y = (base.corner1_y + base.corner2_y + base.corner3_y + base.corner4_y) /4
        delta_x = goal_x - tank.x
        delta_y = goal_y - tank.y
        
        # Bound the influence of the goal
        dist = math.sqrt((goal_x - tank.x)**2 + (goal_y - tank.y)**2)
        if dist > self.max_dist_to_obstacle:
            delta_x = delta_x * self.max_dist_to_obstacle / dist
            delta_y = delta_y * self.max_dist_to_obstacle / dist
        
        # Repulsive field
        
        for obstacle in self.bzrc.get_obstacles():
            center_x = 0 
            center_y = 0
            for point in obstacle:
                center_x = point[0]
                center_1 = point[1]
                dist = math.sqrt((center_x - tank.x)**2 + (center_y - tank.y)**2)
				#dist = min(abs(center_x - tank.x), abs(center_y - tank.y)) 
                if dist < self.max_dist_to_obstacle:
                    delta_x = delta_x + 0.5 * self.max_dist_to_obstacle - (center_x - tank.x)
                    delta_y = delta_y + 0.5 * self.max_dist_to_obstacle - (center_y - tank.y)
#				dist = min(abs(point[0] - tank.x), abs(point[1] - tank.y)) 
#				if dist < self.max_dist_to_obstacle:
#					delta_x = delta_x - self.max_dist_to_obstacle - (point[0] - tank.x)
#					delta_y = delta_y - self.max_dist_to_obstacle - (point[1] - tank.y)
                center_x = center_x + point[0]
                center_y = center_y + point[1]
                #print (point)
            center_x = center_x / len(obstacle)
            center_y = center_y / len(obstacle)
            #print ("Centroid: ", center_x, ", ", center_y)
            dist = math.sqrt((center_x - tank.x)**2 + (center_y - tank.y)**2)
            #dist = min(abs(center_x - tank.x), abs(center_y - tank.y)) 
            if dist < self.max_dist_to_obstacle:
                delta_x = delta_x + self.max_dist_to_obstacle - (center_x - tank.x)
                delta_y = delta_y + self.max_dist_to_obstacle - (center_y - tank.y)
                
        
        # Compute final vector
        v = min(math.sqrt(delta_x**2 + delta_y**2), float(self.constants['tankspeed']))
        theta = math.atan2(delta_y, delta_x)
        relative_theta = self.normalize_angle(theta)
        return (v, relative_theta) # v is goal vector of velocity and relative_theta is goal angle
        
    def pd_controller_move(self, tank, target_speed, target_angle):
        tank_speed = math.sqrt(tank.vx**2 + tank.vy**2)

        # TODO: We cannot store the last angle and speed errors in the Agent object, since it processes all tanks.
        # These should be fields in the tank class and be initialized there (tank.last_angle_error, tank.last_speed_error)

        last_speed_error, last_angle_error = self.mytankdata[tank.index]

        angle_error = target_angle - tank.angle
        angle_error = self.normalize_angle(angle_error)
        delta_angle_error = angle_error - last_angle_error # for the derivative portion of the controller
        delta_angle_error = self.normalize_angle(delta_angle_error)
        #print ("Angle error: ", angle_error)
        #print ("Change in angle error: ", delta_angle_error)
        last_angle_error = angle_error # update the tank's last angle error so we can computer its derivative on our next cycle
        new_angvel = self.k_pa * angle_error + self.k_da * delta_angle_error # determine the new angular velocity
        
        speed_error = target_speed - tank_speed
        delta_speed_error = speed_error - last_speed_error
        #print ("Speed error: ", speed_error)
        #print ("Change in speed error: ", delta_speed_error)
        last_speed_error = speed_error # update our last speed error so we can computer its derivative on our next cycle
        new_speed = self.k_ps * speed_error + self.k_ds * delta_speed_error
        
        self.mytankdata[tank.index] = (last_speed_error, last_angle_error)
        
        shoot = False #(Is there an enemy tank in front of us? Can we avoid shooting our own?)
        # TODO: shoot periodically using the simple metric, closest tank at angle theta is enemy tank?
        narrow_angle = 0.2
        for enemy in self.enemies:
            if enemy.status != 'alive':
                continue
            dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
            if abs(math.atan2(enemy.x - tank.x, enemy.y - tank.y)) < narrow_angle:
                shoot = True
        
        # to switch to velocity-based tank speed, use only the parameter new_speed.
        # to use an acceleration-based tank speed, use new_speed + tank_speed
        new_angvel = self.normalize_angle(new_angvel)
        command = Command(tank.index, new_speed + tank_speed, new_angvel, shoot)
        return command
       
    def get_closest_flag(self, tank):
        closest_flag = None
        best_dist = 2 * float(self.constants['worldsize'])
        flags = self.bzrc.get_flags()
        for flag in flags:
            # what about flags that are already captured?
            # what about current team's flag
            if flag.color == self.constants['team']:
                continue
            if flag.poss_color == self.constants['team']:
                continue
            dist = math.sqrt((flag.x - tank.x)**2 + (flag.y - tank.y)**2)
            if dist < best_dist:
                best_dist = dist
                closest_flag = flag
        if closest_flag is None:
            #print("There is no closest flag!")
            a = 1
        else:
            return closest_flag
     #########################################################

def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)
    
    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4

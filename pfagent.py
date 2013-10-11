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
        print('Obstacles')
        print('\n'.join('{}: {}'.format(*k) for k in enumerate(self.bzrc.get_obstacles())))
        print('\n'.join('{}: {}'.format(*k) for k in enumerate(self.constants)))

        for tank in mytanks:
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
        flag = self.get_closest_flag(tank)
        print(flag.color)
        delta_x = flag.x - tank.x
        delta_y = flag.y - tank.y
        v = min(math.sqrt(delta_x**2 + delta_y**2), float(self.constants['tankspeed']))
        theta = math.atan2(delta_y, delta_x)
        relative_theta = self.normalize_angle(theta)
        return (v, relative_theta) # v is goal vector of velocity and relative_theta is goal angle
        
    def pd_controller_move(self, tank, v, theta):
        tank_speed = math.sqrt(tank.vx**2 + tank.vy**2)
        speed_err = v - tank_speed
        angle_err = theta - tank.angle
        relative_angle_err = self.normalize_angle(angle_err)
        k_p = 0.5
        command = Command(tank.index, k_p * speed_err + tank_speed, k_p * relative_angle_err, False)
        return command
       
    def get_closest_flag(self, tank):
        closest_flag = None
        best_dist = 2 * float(self.constants['worldsize'])
        flags = self.bzrc.get_flags()
        for flag in flags:
            # what about flags that are already captured?
            # what about current team's flag
            if flag.color == 
            dist = math.sqrt((flag.x - tank.x)**2 + (flag.y - tank.y)**2)
            if dist < best_dist:
                best_dist = dist
                closest_flag = flag
        if closest_flag is None:
            print("There is no closest flag!")
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

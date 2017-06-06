#!/usr/bin/python

# interacts with wp_window
# Take in list of waypoints, return list of current_path objects

# takes from ros_plane functions in a very modular, replaceable way

import rospy
from ros_plane.msg import Current_Path
from math import *
import numpy as np

M_PI_F = 3.14159265358979323846
M_PI_2_F = 1.57079632679489661923


def get_full_current_path(wp_list): # takes in list of NED waypoints, returns list of NED points for drawing

    def mod2pi(phi):
        mod = phi % (2 * np.pi)
        return mod

    class path_manager_base:

        # Init function
        def __init__(self, wp_list):

            # Init Params
            self.params = self.params_s()
            self.params.R_min = rospy.get_param('R_min', 75.0)

            # waypoint 0 should be initial position, waypoint 1 where you want to start flying to
            self.index_a = 1

            # Class members
            self._num_waypoints = 0
            self.i = 0
            self.state = 1
            self.Va = 15  # dummy value

            # Member objects; initialize waypoints
            self._waypoints = []

            for wp in wp_list:
                new_wp = self.waypoint_temp()
                self._waypoints.append(new_wp)
                self._waypoints[self._num_waypoints].w0 = wp[0]
                self._waypoints[self._num_waypoints].w1 = wp[1]
                self._waypoints[self._num_waypoints].w2 = wp[2]
                self._waypoints[self._num_waypoints].chi_d = wp[3]
                self._waypoints[self._num_waypoints].chi_valid = True
                self._waypoints[self._num_waypoints].Va_d = self.Va
                self._waypoints[self._num_waypoints].land = False
                self._num_waypoints += 1

            self.current_path_list = []

        # Subclasses
        class waypoint_temp:
            w0 = 0.0
            w1 = 0.0
            w2 = 0.0
            chi_d = 0.0
            chi_valid = True
            Va_d = 0.0
            land = False

        # output for current_path
        class output_s():
            # Inicates strait line or orbital path (true is line, false is orbit)
            flag = True
            Va_d = 0.0  # Desired airspeed (m/s)
            r = [0.0, 0.0, 0.0]  # Vector to origin of straight line path (m)
            # Unit vector, desired direction of travel for line path
            q = [0.0, 0.0, 0.0]
            c = [0.0, 0.0, 0.0]  # Center of orbital path (m)
            rho = 0.0  # Radius of orbital path (m)
            lambda_ = 1  # Direction of orbital path (cw is 1, ccw is -1)

        class params_s():
            R_min = 0.0  # Minimum turning radius

        # Class Member Functions

        def current_path_publisher(self, output):
            current_path = Current_Path()
            # set current_path to output from manager
            current_path.flag = output.flag
            current_path.Va_d = output.Va_d

            for i in range(0, 3):
                current_path.r[i] = output.r[i]
                current_path.q[i] = output.q[i]
                current_path.c[i] = output.c[i]

            current_path.rho = output.rho
            current_path.lambda_ = output.lambda_

            if self.index_a > 1:
                current_path.land = self._waypoints[self.index_a].land
            else:
                current_path.land = False

            self.current_path_list.append(current_path)  # "publish"

        # functions
        def iterate(self):
            inpt = 'None'  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            outputs = self.output_s()
            outputs = self.manage(self.params, inpt, outputs)
            self.current_path_publisher(outputs)

        def manage(self, params, inpt, output):

            # Calc distance between waypoints
            if (self._num_waypoints >= 2):
                start = np.array(
                    [self._waypoints[self.index_a - 1].w0, self._waypoints[self.index_a - 1].w1])
                end = np.array([self._waypoints[self.index_a].w0,
                               self._waypoints[self.index_a].w1])
                dist = np.linalg.norm(start - end)

            if self._num_waypoints < 3:
                output.flag = False
                output.Va_d = 15
                output.r[0] = 999
                output.r[1] = 999
                output.r[2] = 999
                output.q[0] = 999
                output.q[1] = 999
                output.q[2] = 999
                output.c[0] = 0.0
                output.c[1] = 0.0
                output.c[2] = -60
                output.rho = self.params.R_min
                output.lambda_ = 1
                rospy.logwarn('ERROR: less than 2 waypoints!!!')

            elif self._num_waypoints >= 3 and (dist < 2 * self.params.R_min):
                # If good to go at takeoff OR headed to landing point OR Distance between waypoints < 2R
                output = self.manage_line(params, inpt, output)
            else:
                output = self.manage_dubins(params, inpt, output)

            return output

        def manage_dubins(self, params, inpt, output):

            print 'DUBINS'

            R_min = params.R_min

            now = self._waypoints[self.index_a]
            past = self.waypoint_temp()

            if (self.index_a == 0):
                past = self._waypoints[self._num_waypoints - 1]
            else:
                past = self._waypoints[self.index_a - 1]

            w_past = np.array([[past.w0], [past.w1], [past.w2]])
            chi_past = past.chi_d

            w_now = np.array([[now.w0], [now.w1], [now.w2]])
            chi_now = now.chi_d

            [L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3] = self.dubinsParameters(
                w_past, chi_past, w_now, chi_now, R_min)
                # L = length of path
                # cs = NED of start circle
                # lam_s = direction of start circle
                # ce = NED of end circle
                # lam_e = direction of end circle
                # z1 = first half plane (start circle to straight line)
                # q1 = vector normal to z1 halfplane
                # z2 = second half plane (line to end circle)
                # z3 = third half plane (circle to finish)
                # q3 = normal to third half plane

                # First arc = start at w_past, end at z1 (radius is R_min about cs)
                # Line = z1 to z2
                # Last arc = Start at z2, end at z3 (radius is R_min about ce)
            r = 'r'
            q = 'q'
            c = 'c'
            lam = 'lam'
            flag = 'flag'

            if self.state == 1:
                flag = False
                c = cs

                # figure out start and end angles
                start_angle = acos((past.w1 - cs[1]) / R_min) * 180.0 / M_PI_F
                final_angle = acos((z1[1] - cs[1]) / R_min) * 180.0 / M_PI_F
                r = np.array([[start_angle], [-999], [-999]]) # start angle
                q = np.array([[final_angle - start_angle], [-999], [-999]]) # span angle

                lam = lam_s
            elif self.state == 2:
                flag = True
                r = z1
                q = z2#q1
                c = np.array([[-999], [-999], [-999]])
                lam = 0
            elif self.state == 3:
                flag = False
                c = ce

                # figure out start and end angles
                start_angle = acos((past.w1 - cs[1]) / R_min) * 180.0 / M_PI_F
                final_angle = acos((z1[1] - cs[1]) / R_min) * 180.0 / M_PI_F
                r = np.array([[start_angle], [-999], [-999]]) # start angle
                q = np.array([[final_angle - start_angle], [-999], [-999]]) # span angle

                lam = lam_e
                #r = np.array([[-999], [-999], [-999]]) # start angle
                #q = np.array([[-999], [-999], [-999]]) # end angle
                if (self.index_a == (self._num_waypoints - 1)):
                    self.index_a = 0
                else:
                    self.index_a += 1

            if self.state == 3:
                self.state = 1
            else:
                self.state += 1

            output.flag = flag # Inicates strait line or orbital path (true is line, false is orbit)
            output.Va_d = now.Va_d # Desired airspeed (m/s)
            output.r = r # Vector to origin of straight line path (m)
            output.q = q # Unit vector, desired direction of travel for line path
            output.c = c # Center of orbital path (m)
            output.rho = R_min # Radius of orbital path (m)
            output.lambda_ = lam # Direction of orbital path (cw is 1, ccw is -1)

            return output

        def rotz(self, theta):
            R = np.array([[cos(theta), -sin(theta), 0.0],
                            [sin(theta),cos(theta),0.0],
                            [0.0, 0.0, 1.0]])
            return R

        def mo(self,inpt):
            val = 0.0
            if (inpt > 0):
                val = fmod(inpt, 2*M_PI_F)
            else:
                n = 0.0
                n = floor(inpt/2/M_PI_F)
                val = inpt - n*2*M_PI_F
            return val

        def normalize(self, v):
            # Theres probably a better way to do this, but his is what works
            # norm=np.linalg.norm(v, ord=1) # tried this, but it didn't work, result in vector that sums to 1 not magnitude 1
            norm = sqrt(v[0]**2 + v[1]**2 + v[2]**2)

            if norm==0:
                norm=np.finfo(v.dtype).eps

            temp = np.array([0.0, 0.0, 0.0])
            temp[0] = v[0]/norm
            temp[1] = v[1]/norm
            temp[2] = v[2]/norm
            return temp#v/norm

        def dubinsParameters(self, ps, chi_s, pe, chi_e, R):
            dist = np.linalg.norm(ps - pe)
            e1 = np.array([[1], [0], [0]])

            assert (dist >= 2 * R), "waypoints are too close together!"

            crs = ps + R * np.matmul(self.rotz(np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
            cls = ps + R * np.matmul(self.rotz(-np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
            cre = pe + R * np.matmul(self.rotz(np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
            cle = pe + R * np.matmul(self.rotz(-np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
            # print "cle"
            # print cle

            # compute length for case 1 rsr
            ang = atan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
            L1 = np.linalg.norm(crs-cre) + R * mod2pi(2 * np.pi + mod2pi(ang - np.pi / 2) - mod2pi(chi_s - np.pi / 2)) \
                 + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang - np.pi / 2))

            # Compute length for case 2 rsl
            ang = atan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
            l = np.linalg.norm(cle - crs)

            try:
                ang2 = ang - np.pi / 2 + asin((2 * R) / l)
                L2 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
                     + R * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))
            except ValueError:
                L2 = 9999

            # Compute length for case 3 lsr
            ang = atan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))
            l = np.linalg.norm(cre-cls)

            try:
                ang2 = acos((2 * R) / l)
                L3 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + ang2)) \
                     + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang + ang2 - np.pi))
            except ValueError:
                L3 = 9999

            # Compute length for case 4 lsl
            ang = atan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))
            L4 = np.linalg.norm(cls-cle) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + np.pi / 2)) \
                 + R * mod2pi(2 * np.pi + mod2pi(ang + np.pi / 2) - mod2pi(chi_e + np.pi / 2))

            lengths = [L1, L2, L3, L4]
            if min(lengths) == L1:
                cs = crs
                lam_s = 1
                ce = cre
                lam_e = 1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * np.matmul(self.rotz(-np.pi/2), q1)
                z2 = ce + R * np.matmul(self.rotz(-np.pi/2), q1)
                q1 = (z2 - z1) / np.linalg.norm(z2 - z1)

            elif min(lengths) == L2:
                cs = crs
                lam_s = 1
                ce = cle
                lam_e = -1
                l = np.linalg.norm(ce - cs)
                ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ang2 = ang - np.pi/2 + asin((2 * R) / l)
                q1 = np.matmul(self.rotz(ang2 + np.pi/2), e1)
                z1 = cs + R * np.matmul(self.rotz(ang2), e1)
                z2 = ce + R * np.matmul(self.rotz(ang2 + np.pi), e1)

            elif min(lengths) == L3:
                cs = cls
                lam_s = -1
                ce = cre
                lam_e = 1
                l = np.linalg.norm(ce - cs)
                ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ang2 = acos((2 * R) / l)
                q1 = np.matmul(self.rotz(ang + ang2 - np.pi/2), e1)
                z1 = cs + R * np.matmul(self.rotz(ang + ang2), e1)
                z2 = ce + R * np.matmul(self.rotz(ang + ang2 -np.pi), e1)

            # elif min(lengths) == L4:
            else:
                cs = cls
                lam_s = -1
                ce = cle
                lam_e = -1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * np.matmul(self.rotz(np.pi/2), q1)
                z2 = ce + R * np.matmul(self.rotz(np.pi/2), q1)

            z3 = pe
            q3 = np.matmul(self.rotz(chi_e), e1)

            return [min(lengths), cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3]

        def dot(self, first, second):
            # first and second are np.arrays of size 3
            temp = (first[0]*second[0] + first[1]*second[1] + first[2]*second[2])
            return temp

        def manage_line(self, params, inpt, output):
            print 'LINE'

            b = self._waypoints[self.index_a]
            a = self.waypoint_temp()
            # c = self.waypoint_temp()

            if (self.index_a == (self._num_waypoints - 1)):
                a = self._waypoints[self.index_a - 1]
                # c = self._waypoints[0]
            elif (self.index_a == 0):
                a = self._waypoints[self._num_waypoints - 1]
                # c = self._waypoints[self.index_a + 1]
            else:
                a = self._waypoints[self.index_a - 1]

            w_im1 = np.array([a.w0,a.w1,a.w2])
            w_i = np.array([b.w0,b.w1,b.w2])

            output.flag = True
            output.Va_d = a.Va_d
            output.r = [w_im1[0],w_im1[1],w_im1[2]]

            q_im1 = self.normalize(w_i - w_im1)

            output.q = [w_i[0],w_i[1],w_i[2]] # ======================
            output.c = [1, 1, 1]
            output.rho = 1
            output.lambda_ = 1

            #if (self.index_a == (self._num_waypoints - 1)): ++++++++++++++++++++++++++++
            #    self.index_a = 0
            #else:
            #    self.index_a += 1

            if (self.index_a == (self._num_waypoints - 1)):
                self.index_a = 0
            else:
                self.index_a += 1

            return output

    manager = path_manager_base(wp_list)

    while not manager.index_a == 0:
        manager.iterate()
    manager.iterate() # for circling back to home
    while not manager.index_a == 0:
        manager.iterate()

    print manager.current_path_list
    return manager.current_path_list

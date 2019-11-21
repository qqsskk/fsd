#! /usr/bin/env python


import numpy as np
import cvxpy as cv
import spline as sp
import matplotlib.pyplot as pt


class State:
    def __init__(self, x0=.0, y0=.0, psi0=.0, ey0=.0, epsi0=.0):
        self.x = x0
        self.y = y0
        self.psi = psi0
        self.ey = ey0
        self.epsi = epsi0

    def spatial_to_cartesian(self, reference_handler, reference, prediction):
        max_length = reference.path_length
        cartesian_prediction_path = Path()
        for i in range(0, max_length):
            if(i+1) >= max_length:
                pass
            else:
                length_traj = len(prediction[0][:])
                current_idx = reference.idx[i]
                # TODO: VERIFY Assumption that next element in path is close enough to be considered tangential
                # Find angle to tangent in global coordinate system
                # find next element in reference handler for tangent approximation
                dx = reference_handler.x[current_idx + 1] - reference.x[i]
                dy = reference_handler.y[current_idx + 1] - reference.y[i]
                psi = np.arctan2(dy, dx)
                # Coordinate Transformation between relative and global coordinate frame
                temp_x = reference.x[i] + (-prediction[0][i]*np.sin(psi))
                temp_y = reference.y[i] + (prediction[0, i]*np.cos(psi))
                cartesian_prediction_path.append(temp_x, temp_y)
        return cartesian_prediction_path


class Model:
    def __init__(self):
        self.y = 0
        self.psi = 0

        self.A = np.array([[1, 1], [1, 1]])
        self.B = np.array([[0], [0]])

    def update_model(self, kappa_ref, velocity, dt):
        ds = velocity*dt
        self.A = np.array([[1, ds], [-np.square(kappa_ref)*ds, 1]])
        self.B = np.array([0, ds])


class Path:
    def __init__(self, x0=[], y0=[], ds0=[], kappa0=[], idx0=[]):
        self.x = x0
        self.y = y0
        self.ds = ds0
        self.kappa = kappa0
        self.idx = idx0

        if not(x0 == []):
            self.path_length = len(self.x)
        else:
            self.path_length = 0

    def append(self, x, y, ds=0, kappa=0, idx=0):
        self.x.append(x)
        self.y.append(y)
        self.ds.append(ds)
        self.kappa.append(kappa)
        self.idx.append(idx)
        self.path_length = len(self.x)

    def reset(self):
        self.x = []
        self.y = []
        self.ds = []
        self.kappa = []
        self.idx = []
        self.path_length = 0


class Reference:
    def __init__(self, waypoints):
        self.path = Path()
        self.waypoints = waypoints

        # Reference state variables
        self.ds_current = 0
        self.ds_predict = 0
        self.kappa_ref_current = 0
        self.kappa_ref_predict = 0
        self.idx_current = 0
        self.idx_predict = 0
        self.idx_max = 0

        # Spline variables
        self.ds_step = 0.1
        self.margin_dist = 0
        self.margin_idx = 0
        self.margin_connect_dist = self.ds_step*2

        # Interpolate and initialise path
        self.define_path()

    def reset_waypoint_list(self):
        self.waypoints = np.array([[], []])

    def load_waypoints(self, waypoints):
        self.reset_waypoint_list()
        self.waypoints = waypoints
        self.define_path()

    def load_waypoints_from_file(self, directory, name):
        # TODO: add input from file here
        pass

    def callback_waypoints(self, data):
        # TODO: finish ROS callback function to redefine path on data reception
        # only update if waypoints changed/amount or position
        # reset waypoints
        # store waypoints from callback
        # spline interpolation
        pass

    def get_current_curvature(self):
        return self.path.kappa[self.idx_current]

    def update_predicted_ds(self, path_segment, velocity=None):
        self.idx_predict += int(path_segment / self.ds_step)
        if self.idx_predict >= self.idx_max:
            # TODO: Define reach_end_of_path() method that decides whether closed loop or stoping horizon
            # TODO: before creating optimisation problem, check velocity and path available,
            # necessary to brake if not enough path
            self.idx_predict = np.mod(self.idx_predict, self.idx_max)
        return self.path.x[self.idx_predict], self.path.y[self.idx_predict], self.idx_predict

    def get_predicted_curvature(self):
        return self.path.kappa[self.idx_predict]

    def get_predicted_position(self):
        return self.path.x[self.idx_predict], self.path.y[self.idx_predict]

    def update_current_ds(self, state):
        # find current ds - closest wp on path
        dx = [state.x - icx for icx in self.path.x]
        dy = [state.y - icy for icy in self.path.y]
        d = [abs(np.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        self.idx_current = d.index(min(d))
        self.ds_current = self.path.ds[self.idx_current]
        self.kappa_ref_current = self.path.kappa[self.idx_current]
        self.idx_predict = self.idx_current  # reset prediction shift

        return self.path.x[self.idx_predict], self.path.y[self.idx_predict], self.idx_predict

    def define_margin_dist(self):
        max_length = self.waypoints.path_length
        temp_ds = 0
        for i in range(0, max_length):
            if not ((i+1) > max_length-1):
                temp_ds += np.sqrt(np.square(self.waypoints.x[i + 1] - self.waypoints.x[i]) +
                                   np.square(self.waypoints.y[i + 1] - self.waypoints.y[i]))
            else:
                pass
        self.margin_dist = temp_ds*0.7

    def define_path(self):
        self.path.reset()
        max_length = self.waypoints.path_length
        self.define_margin_dist()

        # Find start index that corresponds to the desired smooth distance
        temp_ds = 0
        for i in range(0, max_length):
            if not ((i+1) > max_length-1):
                temp_ds += np.sqrt(np.square(self.waypoints.x[i + 1] - self.waypoints.x[i]) +
                                   np.square(self.waypoints.y[i + 1] - self.waypoints.y[i]))
                if temp_ds > self.margin_dist:
                    self.margin_idx = i+1
                    break
            else:
                pass

        temp_waypoints_x = self.waypoints.x[:] + self.waypoints.x[:self.margin_idx]
        temp_waypoints_y = self.waypoints.y[:] + self.waypoints.y[:self.margin_idx]

        # Interpolate the waypoints
        spline = sp.Spline2D(temp_waypoints_x, temp_waypoints_y)

        # Arrange the waypoints with a defined distance interval self.ds_step
        s = np.arange(0, spline.s[-1], self.ds_step)

        # Cut parts of interpolation to smooth interpolation for start area with curvature
        found_start_waypoint = False
        temp_first = [[], []]
        for i_s in s:
            if i_s < self.margin_dist/2.:
                pass
            else:
                i_x, i_y = spline.calc_position(i_s)
                i_k = spline.calc_curvature(i_s)
                if not found_start_waypoint:
                    temp_first[0] = i_x
                    temp_first[1] = i_y
                    found_start_waypoint = True

                if (s[-1] - i_s) < self.margin_dist/2:

                    temp_ds = np.sqrt(np.square(i_x - temp_first[0]) +
                                      np.square(i_y - temp_first[1]))
                    if temp_ds < self.margin_connect_dist:
                        break
                self.path.append(i_x, i_y, i_s, i_k)
        self.idx_max = len(self.path.x)


class PlotHelper:
    def __init__(self, path_handler, vehicle_handler=None):
        self.ref_vehicle = vehicle_handler # TODO: add reference to vehicle model class with all parameters
        self.ref_path = path_handler
        self.transform = State()  # TODO: create new class for transformation

    def plot(self, reference_predicted, t, trajectory_solution, optimal_input):
        """
        :param reference_predicted:
        :param t:
        :param trajectory_solution:
        :param optimal_input:
        :return:
        """
        # Find optimal vehicle trajectory in global coordinate frame
        optimal_trajectory_global = self.transform.spatial_to_cartesian(self.ref_path,
                                                                        reference_predicted,
                                                                        trajectory_solution)
        optimal_trajectory_global_x = np.array(optimal_trajectory_global.x)
        optimal_trajectory_global_y = np.array(optimal_trajectory_global.y)

        # PLOT DEFINITION
        fig, axes = pt.subplots(2, 2)
        ax1 = axes[0, 0]
        ax2 = axes[1, 0]
        ax3 = axes[0, 1]
        ax4 = axes[1, 1]

        # SUBPLOT 11
        # Plot path
        ht11, = ax1.plot(self.ref_path.x, self.ref_path.y, "-g",
                         linewidth=3, label="Path")
        ht12, = ax1.plot(reference_predicted.x, reference_predicted.y, ":bo",
                         markersize=10, linewidth=2, label="Reference path")
        ht13, = ax1.plot(optimal_trajectory_global_x, optimal_trajectory_global_y, "-r*",
                         markersize=10, linewidth=2, label="Vehicle trajectory")
        ax1.set_title('Path, Reference and Trajectory')
        ax1.legend(loc=3, prop={'size': 12}, handles=[ht11, ht12, ht13])
        ax1.set_xlabel('x [m]')
        ax1.set_ylabel('y [m]')
        ax1.grid(True)
        #ax1.set_aspect('equal', adjustable='box')
        ax1.axis("equal")
        # SUBPLOT 12
        ht21, = ax2.plot(reference_predicted.x, reference_predicted.y, ":bo",
                         markersize=12, linewidth=2, label="Reference path")
        ht22, = ax2.plot(optimal_trajectory_global_x, optimal_trajectory_global_y, "-r*",
                         markersize=16, linewidth=3, label="Vehicle trajectory")
        ax2.set_title('Reference and Trajectory (zoom)')
        ax2.legend(loc=3, prop={'size': 12}, handles=[ht21, ht22])
        ax2.set_xlabel('x [m]')
        ax2.set_ylabel('y [m]')
        #ax2.set_aspect('equal', adjustable='box')
        #ax2.axis("scaled")
        ax2.axis("equal")
        ax2.grid(True)

        # SUBPLOT 21
        # Plot trajectory
        t = np.array(t)
        # Adapt step array for optimal input
        t_u = t[:-1].copy()
        max_t = len(t)
        psi_deg = []
        for i in range(0, max_t):
            psi_deg += [(180./np.pi)*trajectory_solution[1][i]]
        psi_deg = np.array(psi_deg)

        ax3b = ax3.twinx()
        ht31, = ax3.plot(t, trajectory_solution[0], "-ko",
                         markersize=10, linewidth=4, label='Optimal trajectory e_y [m]')
        ht32, = ax3b.plot(t, psi_deg, "-co",
                         markersize=10, linewidth=4, label='Optimal trajectory e_psi [deg]')
        ax3.set_title('Optimal Trajectory e_y and e_psi')
        ax3.set_xlabel('Optimisation stages [k]')
        ax3.set_ylabel('e_y [m]')
        ax3b.set_ylabel('Orientation [deg]')
        ax3.tick_params(labelcolor='k')
        ax3b.tick_params(labelcolor='c')
        ax3.legend(loc=4, prop={'size': 12}, handles=[ht31, ht32])
        ax3.grid(True)


        # SUBPLOT 22
        # Get steering input
        # Prepare data for plotting
        delta = []
        for i in optimal_input[0]:
            delta += [(180./np.pi)*np.arctan(i*2.5)]  # TODO: Replace vehicle params with reference handler
        delta = np.array(delta)
        ax4b = ax4.twinx()

        delta_length = len(delta)
        ddelta = []
        for i in range(0, delta_length):
            if not ((i+1) == delta_length):
                ddelta += [(delta[i+1]-delta[i])*(1/10.)]
            else:
                ddelta += [ddelta[-1]]

        ddelta = np.array(ddelta)
        ht4, = ax4.plot(t_u, delta, "-ko",
                          markersize=10, linewidth=4, label='Optimal delta [deg]')

        ht4b, = ax4b.plot(t_u, ddelta, "-mo",
                        markersize=10, linewidth=4, label='Optimal ddelta [deg/s]')
        ax4.set_title('Optimal Input in Rad and Deg')
        ax4.set_xlabel('Optimisation stages [k]')
        ax4.set_ylabel('Steering [deg]')
        ax4b.set_ylabel('Steering derivative [deg/s]')
        ax4.tick_params(labelcolor='k')
        ax4b.tick_params(labelcolor='m')
        ax4.legend(loc=4, prop={'size': 12}, handles=[ht4, ht4b])
        ax4.grid(True)
        fig.show()
        input("Continue with key entry")


def main():
    r = 6
    wp = Path([0*r], [0])

    # Path 3
    wp.append(1*r, 1*r)
    wp.append(3*r, -1*r)
    wp.append(5*r, 1*r)
    wp.append(7*r, -1*r)


    # Path 2
    #wp.append(1*r, 0*r)
    #wp.append(2*r, 0*r)
    #wp.append(2*r, 2*r)
    #wp.append(2*r, 6*r)
    #wp.append(0*r, 8*r)


    # Path 1
    #wp.append(0, 1*r)
    #wp.append(-1*r, 0)
    #wp.append(0, -1*r)
    #print('Waypoint list: \n x={0} \n y={1}'.format(wp.x, wp.y))

    # Load waypoints and define path
    ref = Reference(wp)

    # Define plotting class for trajectory visualisation
    # TODO: reference handles for live plotting/updating of plotting variables (not fully implemented)
    plotter = PlotHelper(ref.path)

    # Dimensional definition of optimisation problem
    n = 2
    m = 1
    t_f = 20

    # Define weights of cost function
    q1 = 1000.      # state costs
    q2 = 10.
    r1 = 0.001     # input cost
    r2 = 10     # input derivative cost

    # Constant states
    sampling_frequency = 10.  # in 1/s
    sampling_period = 1. / sampling_frequency
    time_horizon = t_f*sampling_period

    wheel_base = 3
    velocity = 15
    # Minimum curvature
    """ 
    Curvature needs to be smaller than the curvature corresponding to the 
    minimum turning_radius R_T >= 4m => kappa <= 1/R_T = 1/4. = 0.25
    """
    max_ddelta_ref = 45. * (np.pi/180.)/sampling_frequency  # in deg/T

    max_dcurvature = 1./(np.square(np.cos(max_ddelta_ref))*wheel_base)  # TODO: add sampling period on top and define everything based on sampling period! important
    minimum_curvature = .25
    path_eps = 1  # terminal constraint (0.2m)
    terminal_eps = 0.2  # terminal constraint (0.2m)

    # Output constraints
    max_delta = (180./np.pi)*np.arctan(minimum_curvature*2.5)
    print('Max delta {0} deg'.format(max_delta))
    print('Maximum derivative of delta {0} in deg/s and {1} in deg/T'.format(
        (180./np.pi)*max_ddelta_ref*sampling_frequency,
        (180./np.pi)*max_ddelta_ref))
    print('Wheel-base {0} m'.format(wheel_base))
    print('Time horizon {0} s'.format(time_horizon))
    print('Amount of stages {0}'.format(t_f))

    ds = velocity*sampling_period

    # Model definition
    model = Model()

    # define cost function ONE OPTIMISATION LOOP
    x = cv.Variable((n, t_f + 1))
    u = cv.Variable((m, t_f))

    # initial position in global coordinate frame
    vehicle_state = State(wp.x[3], wp.y[3], 0, 0, 0)

    # initial state in relative coordinate frame (ey, epsi)
    x_initial = State(ey0=2, epsi0=20. * (np.pi / 180.))
    x_final = State(ey0=0., epsi0=0.)

    # Initial position in relative coordinate frame for optimisation
    # TODO: update initial and terminal states based on feedback from state estimation
    x0 = [x_initial.ey, x_initial.epsi]
    xf = [x_final.ey, x_final.epsi]

    # Define cost and constraint variables
    cost = 0
    constr = []

    x00, y00, idx00 = ref.update_current_ds(vehicle_state)
    current_curvature = ref.get_current_curvature()
    model.update_model(current_curvature, velocity, sampling_period)

    ref_predicted = Path(x0=[x00], y0=[y00], ds0=[], kappa0=[], idx0=[idx00])

    for k in range(t_f):
        pos_x, pos_y, idx_predict = ref.update_predicted_ds(ds)
        kappa_ref = ref.get_predicted_curvature()
        ref_predicted.append(pos_x, pos_y, kappa=kappa_ref, idx=idx_predict)

        # DEFINE COST
        # Define cost function for state
        cost += q1*cv.sum_squares(x[:, k + 1])

        # Define cost function for input
        #cost += r1*cv.sum_squares(u[:, k])
        #  TODO: Discuss why input should be added at all, not reasonable

        # Define cost function for input derivative (smoothness)
        if not(k == t_f-1):
            cost += r2*cv.sum_squares(u[:, k+1] - u[:, k])
            pass
        # Define cost function for state derivative (smoothness)
        if not(k == t_f-1):
            cost += q2*cv.sum_squares(x[:, k + 1] - x[:, k])
            pass

        # DEFINE CONSTRAINTS
        # Dynamic constraints
        constr += [x[:, k + 1] == model.A * x[:, k] + model.B * (u[:, k] - kappa_ref)]
        # Input constraints (ackermann)
        constr += [cv.norm(u[:, k], 'inf') <= minimum_curvature]
        # Define input constraints for input derivative (smoothness)
        if not(k == t_f-1):
            constr += [cv.norm(u[:, k+1] - u[:, k], 'inf') <= max_dcurvature]
            pass
        # State constraints
        constr += [cv.norm(x[:, t_f-int(t_f/2.)]) <= path_eps]

    # DEFINE INITIAL AND TERMINAL SET
    # Initial set constraint
    constr += [x[:, 0] == x0]
    # Terminal set constraint
    #constr += [cv.norm(x[:, t_f]) <= terminal_eps]

    problem = cv.Problem(cv.Minimize(cost), constr)
    problem.solve()

    # Visualise results in terminal
    print('Optimisation solution: {0}'.format(problem.status))
    print('Optimisation statistics: \n Solve-Time: {0} s\n Iterations for Optimum: {1} iterations\n '.format(problem.solver_stats.solve_time,problem.solver_stats.num_iters))
    step_array = range(0, t_f+1)
    x_solution = x[:, :].value
    u_solution = u[:, :].value
    #print('X solution {0}'.format(x_solution))
    #print('U solution {0}'.format(u_solution))

    # Plot via matplotlib class
    plotter.plot(ref_predicted, step_array, x_solution, u_solution)


if __name__ == "__main__":
    main()


